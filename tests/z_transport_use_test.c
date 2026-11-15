//
// Copyright (c) 2026 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//

// nano-ros issue 2596 — a send holds a USE of the session's transport, not the
// session's transport mutex, and a teardown waits for the uses to drain.
//
//   1. With TX_SPLIT_LOCK, an append is not held for a slow detached batch
//      write (it was, for the write's whole duration).
//   2. A teardown does not return, so does not free the transport, while a
//      send is still inside it (issue 0899's guarantee).
//   3. Stress: the transport is torn down and re-opened over and over while
//      several threads send; nothing is used after it is freed (run under
//      ASan) and nothing hangs (a watchdog aborts the run).
//
// No network: a fake datagram link whose write can be slowed down. Needs
// multi-thread, batching and fragmentation; case 1 also needs split lock.

#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "zenoh-pico/net/session.h"
#include "zenoh-pico/protocol/definitions/network.h"
#include "zenoh-pico/session/utils.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/transport/common/tx.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/transport/utils.h"

#if Z_FEATURE_MULTI_THREAD == 1 && Z_FEATURE_BATCHING == 1 && Z_FEATURE_FRAGMENTATION == 1

#define LINK_MTU 256

static int failures = 0;

#define CHECK(cond, what)                   \
    do {                                    \
        if (!(cond)) {                      \
            printf("  FAIL: %s\n", (what)); \
            failures++;                     \
        } else {                            \
            printf("  ok    %s\n", (what)); \
        }                                   \
    } while (0)

static atomic_int g_in_write;
static atomic_int g_write_delay_us;
static atomic_long g_writes;

static size_t fake_write(const struct _z_link_t *self, const uint8_t *ptr, size_t len, _z_sys_net_socket_t *socket) {
    (void)socket;
    (void)ptr;
    // Read the link: under ASan a write on a freed link is reported here.
    volatile uint16_t mtu = self->_mtu;
    (void)mtu;
    atomic_fetch_add(&g_in_write, 1);
    int delay = atomic_load(&g_write_delay_us);
    if (delay > 0) {
        z_sleep_us((size_t)delay);
    }
    atomic_fetch_add(&g_writes, 1);
    atomic_fetch_sub(&g_in_write, 1);
    return len;
}

static _z_session_t g_zn;

// What `_z_open` builds, minus the handshake: a heap link and a whole
// transport, then "ready". Called with `_mutex_transport` held, as the
// reconnect does.
static void open_transport(void) {
    _z_link_t *link = (_z_link_t *)z_malloc(sizeof(_z_link_t));
    memset(link, 0, sizeof(*link));
    link->_write_f = fake_write;
    link->_mtu = LINK_MTU;
    link->_cap._flow = Z_LINK_CAP_FLOW_DATAGRAM;
    memset(&g_zn._tp, 0, sizeof(g_zn._tp));
    g_zn._tp._type = _Z_TRANSPORT_UNICAST_TYPE;
    _z_transport_common_t *ztc = &g_zn._tp._transport._unicast._common;
    ztc->_link = link;
    ztc->_wbuf = _z_wbuf_make(LINK_MTU, false);
#if Z_FEATURE_TX_SPLIT_LOCK == 1
    ztc->_wbuf_spare = _z_wbuf_make(LINK_MTU, false);
    _z_mutex_init(&ztc->_mutex_link_tx);
#endif
    ztc->_zbuf = _z_zbuf_make(16);
    ztc->_sn_res = _z_sn_max(Z_SN_RESOLUTION);
    _z_mutex_init(&ztc->_mutex_tx);
    _z_mutex_rec_init(&ztc->_mutex_peer);
    ztc->_batch_state = _Z_BATCHING_ACTIVE;
    _z_atomic_bool_store(&g_zn._tp_ready, true, _z_memory_order_release);
}

static void teardown_transport(void) {
    _z_session_transport_mutex_lock(&g_zn);
    _z_session_transport_teardown(&g_zn);
    _z_session_transport_mutex_unlock(&g_zn);
}

static void open_session(void) {
    memset(&g_zn, 0, sizeof(g_zn));
    g_zn._mode = Z_WHATAMI_CLIENT;
    _z_mutex_rec_init(&g_zn._mutex_transport);
    _z_mutex_init(&g_zn._mutex_tp_users);
    _z_condvar_init(&g_zn._cv_tp_users);
    g_zn._tp_users = 0;
    _z_atomic_bool_init(&g_zn._tp_ready, false);
    _z_session_transport_mutex_lock(&g_zn);
    open_transport();
    _z_session_transport_mutex_unlock(&g_zn);
    atomic_store(&g_write_delay_us, 0);
}

static void close_session(void) {
    teardown_transport();
    _z_condvar_drop(&g_zn._cv_tp_users);
    _z_mutex_drop(&g_zn._mutex_tp_users);
    _z_mutex_rec_drop(&g_zn._mutex_transport);
}

static uint8_t g_payload[3 * LINK_MTU];

static z_result_t send_push(size_t len, bool express) {
    _z_wireexpr_t key = _z_wireexpr_null();
    key._id = 1;
    _z_bytes_t payload;
    _z_bytes_from_buf(&payload, g_payload, len);
    _z_network_message_t msg;
    _z_n_msg_make_push_put(&msg, &key, &payload, NULL, _z_n_qos_make(express, true, Z_PRIORITY_DEFAULT), NULL, NULL,
                           Z_RELIABILITY_RELIABLE, NULL);
    z_result_t ret = _z_send_n_msg(&g_zn, &msg, Z_RELIABILITY_RELIABLE, Z_CONGESTION_CONTROL_BLOCK, NULL);
    _z_bytes_drop(&payload);
    return ret;
}

static void wait_for_write(void) {
    while (atomic_load(&g_in_write) == 0) {
        z_sleep_us(100);
    }
}

#if Z_FEATURE_TX_SPLIT_LOCK == 1
static void *one_flush(void *arg) {
    (void)arg;
    _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
    return NULL;
}

static void append_is_not_held_for_a_detached_write(void) {
    printf("an append is not held for a slow detached batch write\n");
    open_session();
    unsigned long worst = 0;
    for (int round = 0; round < 5; round++) {
        send_push(16, false);
        atomic_store(&g_write_delay_us, 50000);  // a 50 ms socket write
        _z_task_t t;
        _z_task_init(&t, NULL, one_flush, NULL);
        wait_for_write();
        z_clock_t c = z_clock_now();
        send_push(16, false);  // batched, not express
        unsigned long took = z_clock_elapsed_ms(&c);
        _z_task_join(&t);
        atomic_store(&g_write_delay_us, 0);
        worst = took > worst ? took : worst;
    }
    printf("        worst append: %lu ms against a 50 ms write\n", worst);
    CHECK(worst < 25, "an append takes well under the detached write's duration");
    close_session();
}
#endif

// 2. A send parked in a slow write holds its use; the teardown waits for it.
static atomic_int g_send_done;

static void *slow_express_send(void *arg) {
    (void)arg;
    send_push(16, true);  // express: written at once
    atomic_store(&g_send_done, 1);
    return NULL;
}

static void teardown_waits_for_a_send_inside(void) {
    printf("a teardown waits for a send that is still inside the transport\n");
    open_session();
    atomic_store(&g_write_delay_us, 100000);  // 100 ms
    atomic_store(&g_send_done, 0);
    _z_task_t t;
    _z_task_init(&t, NULL, slow_express_send, NULL);
    wait_for_write();
    z_clock_t c = z_clock_now();
    teardown_transport();
    unsigned long took = z_clock_elapsed_ms(&c);
    bool send_finished_first = atomic_load(&g_send_done) == 1 || atomic_load(&g_in_write) == 0;
    _z_task_join(&t);
    printf("        teardown returned after %lu ms\n", took);
    CHECK(send_finished_first, "the teardown returned only after the send left the transport");
    CHECK(took >= 50, "the teardown waited for the write in flight");
    CHECK(send_push(16, true) == _Z_ERR_TRANSPORT_NOT_AVAILABLE, "a send after the teardown is refused");
    _z_condvar_drop(&g_zn._cv_tp_users);
    _z_mutex_drop(&g_zn._mutex_tp_users);
    _z_mutex_rec_drop(&g_zn._mutex_transport);
}

// 3. Reconnect over and over while several threads send.
#define SENDERS 4
static atomic_int g_stop;
static atomic_long g_sent_ok;
static atomic_long g_refused;

static void *sender(void *arg) {
    intptr_t id = (intptr_t)arg;
    long n = 0;
    while (!atomic_load(&g_stop)) {
        size_t len = (n++ % 7 == 0) ? 2 * LINK_MTU : 16;  // some fragment
        z_result_t r = send_push(len, (id % 2) == 0);
        if (r == _Z_RES_OK) {
            atomic_fetch_add(&g_sent_ok, 1);
        } else {
            atomic_fetch_add(&g_refused, 1);
        }
        if ((n % 5) == 0) {
            _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
        }
    }
    return NULL;
}

static void on_hang(int sig) {
    (void)sig;
    static const char msg[] = "  FAIL: the stress run hung (watchdog)\n";
    (void)write(1, msg, sizeof(msg) - 1);
    _exit(2);
}

static void reconnect_while_sending(void) {
    printf("reconnect repeatedly while %d threads send\n", SENDERS);
    open_session();
    atomic_store(&g_write_delay_us, 200);
    atomic_store(&g_stop, 0);
    signal(SIGALRM, on_hang);
    alarm(60);
    _z_task_t t[SENDERS];
    for (intptr_t i = 0; i < SENDERS; i++) {
        _z_task_init(&t[i], NULL, sender, (void *)i);
    }
    const int cycles = 200;
    for (int i = 0; i < cycles; i++) {
        z_sleep_us(2000);
        teardown_transport();
        z_sleep_us(500);  // an outage: sends must fail fast, not touch `_tp`
        _z_session_transport_mutex_lock(&g_zn);
        open_transport();
        _z_session_transport_mutex_unlock(&g_zn);
    }
    atomic_store(&g_stop, 1);
    for (int i = 0; i < SENDERS; i++) {
        _z_task_join(&t[i]);
    }
    alarm(0);
    printf("        %d reconnects, %ld send(s) delivered, %ld refused during outages, %ld link write(s)\n", cycles,
           atomic_load(&g_sent_ok), atomic_load(&g_refused), atomic_load(&g_writes));
    CHECK(atomic_load(&g_sent_ok) > 0, "sends went through between the reconnects");
    CHECK(atomic_load(&g_refused) > 0, "sends during an outage were refused, not blocked");
    CHECK(g_zn._tp_users == 0, "no transport use is left over");
    close_session();
}

int main(void) {
#if Z_FEATURE_TX_SPLIT_LOCK == 1
    append_is_not_held_for_a_detached_write();
#else
    printf("(the append case needs Z_FEATURE_TX_SPLIT_LOCK; skipped)\n");
#endif
    teardown_waits_for_a_send_inside();
    reconnect_while_sending();
    if (failures != 0) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}

#else
int main(void) {
    printf("SKIP: needs Z_FEATURE_MULTI_THREAD, BATCHING and FRAGMENTATION\n");
    return 0;
}
#endif
