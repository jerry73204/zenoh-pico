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

// nano-ros issue 2575 — with Z_FEATURE_TX_SPLIT_LOCK every link write must go
// through the link mutex, and a parked (overflow-stolen) batch, which holds
// older sequence numbers, must reach the wire before anything sent after it.
//
// The fragment path broke both: it wrote the link holding only the tx mutex,
// so a detached batch send (link mutex only) could write at the same time, and
// it never drained the parked spare, so the fragments of a message that
// overflowed a batch went out before the batch they overflowed.
//
// No network: a fake datagram link records every write (message id, SN) and
// flags a write that starts while another is still in progress.
//
// Build with -DZ_FEATURE_TX_SPLIT_LOCK=1 (batching, fragmentation and
// multi-thread on, the defaults); without split lock the test reports a skip.

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zenoh-pico/net/session.h"
#include "zenoh-pico/protocol/definitions/network.h"
#include "zenoh-pico/protocol/definitions/transport.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/transport/common/tx.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/transport/utils.h"

#if Z_FEATURE_TX_SPLIT_LOCK == 1 && Z_FEATURE_BATCHING == 1 && Z_FEATURE_FRAGMENTATION == 1 && \
    Z_FEATURE_MULTI_THREAD == 1

#define MAX_WRITES 4096
#define LINK_MTU 256

typedef struct {
    uint8_t mid;
    uint64_t sn;
} write_rec_t;

static write_rec_t g_writes[MAX_WRITES];
static atomic_int g_nwrites;
static atomic_int g_in_write;
static atomic_int g_overlaps;
static atomic_int g_write_delay_us;
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

static uint64_t decode_zint(const uint8_t *p, size_t len) {
    uint64_t v = 0;
    for (size_t i = 0; i < len && i < 9; i++) {
        v |= (uint64_t)(p[i] & 0x7f) << (7 * i);
        if ((p[i] & 0x80) == 0) {
            break;
        }
    }
    return v;
}

static size_t fake_write(const struct _z_link_t *self, const uint8_t *ptr, size_t len, _z_sys_net_socket_t *socket) {
    (void)self;
    (void)socket;
    if (atomic_fetch_add(&g_in_write, 1) != 0) {
        atomic_fetch_add(&g_overlaps, 1);
    }
    int delay = atomic_load(&g_write_delay_us);
    if (delay > 0) {
        z_sleep_us((size_t)delay);
    }
    int i = atomic_fetch_add(&g_nwrites, 1);
    if (i < MAX_WRITES && len >= 2) {
        g_writes[i].mid = ptr[0] & 0x1f;
        g_writes[i].sn = decode_zint(ptr + 1, len - 1);
    }
    atomic_fetch_sub(&g_in_write, 1);
    return len;
}

static _z_session_t g_zn;
static _z_link_t g_link;

static void open_fake_session(void) {
    memset(&g_zn, 0, sizeof(g_zn));
    memset(&g_link, 0, sizeof(g_link));
    g_link._write_f = fake_write;
    g_link._mtu = LINK_MTU;
    g_link._cap._flow = Z_LINK_CAP_FLOW_DATAGRAM;
    g_zn._mode = Z_WHATAMI_CLIENT;
    _z_mutex_rec_init(&g_zn._mutex_transport);
    _z_atomic_bool_init(&g_zn._tp_ready, true);
    g_zn._tp._type = _Z_TRANSPORT_UNICAST_TYPE;
    _z_transport_common_t *ztc = &g_zn._tp._transport._unicast._common;
    ztc->_link = &g_link;
    ztc->_wbuf = _z_wbuf_make(LINK_MTU, false);
    ztc->_wbuf_spare = _z_wbuf_make(LINK_MTU, false);
    ztc->_sn_res = _z_sn_max(Z_SN_RESOLUTION);
    _z_mutex_init(&ztc->_mutex_tx);
    _z_mutex_rec_init(&ztc->_mutex_peer);
    _z_mutex_init(&ztc->_mutex_link_tx);
    ztc->_batch_state = _Z_BATCHING_ACTIVE;
    atomic_store(&g_nwrites, 0);
    atomic_store(&g_overlaps, 0);
    atomic_store(&g_write_delay_us, 0);
}

static void close_fake_session(void) {
    _z_transport_common_t *ztc = &g_zn._tp._transport._unicast._common;
    _z_wbuf_clear(&ztc->_wbuf);
    _z_wbuf_clear(&ztc->_wbuf_spare);
}

static uint8_t g_payload[4 * LINK_MTU];

static z_result_t send_push(size_t len) {
    _z_wireexpr_t key = _z_wireexpr_null();
    key._id = 1;
    _z_bytes_t payload;
    _z_bytes_from_buf(&payload, g_payload, len);
    _z_network_message_t msg;
    _z_n_msg_make_push_put(&msg, &key, &payload, NULL, _Z_N_QOS_DEFAULT, NULL, NULL, Z_RELIABILITY_RELIABLE, NULL);
    z_result_t ret = _z_send_n_msg(&g_zn, &msg, Z_RELIABILITY_RELIABLE, Z_CONGESTION_CONTROL_BLOCK, NULL);
    _z_bytes_drop(&payload);
    return ret;
}

// The wire must carry reliable SNs in increasing order.
static bool sns_in_order(int n, int *first_bad) {
    for (int i = 1; i < n; i++) {
        if (g_writes[i].sn < g_writes[i - 1].sn) {
            *first_bad = i;
            return false;
        }
    }
    return true;
}

static void dump_writes(int n) {
    for (int i = 0; i < n && i < 16; i++) {
        printf("        write %d: %s sn=%llu\n", i,
               g_writes[i].mid == _Z_MID_T_FRAME      ? "FRAME"
               : g_writes[i].mid == _Z_MID_T_FRAGMENT ? "FRAGMENT"
                                                      : "other",
               (unsigned long long)g_writes[i].sn);
    }
}

// 1. One thread: small samples fill the batch, then a sample larger than a
//    batch overflows it. The batch is parked with the older SNs; the large
//    sample goes as fragments. The parked batch must still be written first.
static void parked_batch_precedes_fragments(void) {
    printf("parked batch precedes the fragments that overflowed it\n");
    open_fake_session();
    int small = 0;
    while (small < 64 && !g_zn._tp._transport._unicast._common._spare_pending && atomic_load(&g_nwrites) == 0) {
        if (g_zn._tp._transport._unicast._common._batch_count > 0 &&
            _z_wbuf_space_left(&g_zn._tp._transport._unicast._common._wbuf) < 64) {
            break;
        }
        send_push(16);
        small++;
    }
    printf("        %d small sample(s) batched, %d write(s) so far\n", small, atomic_load(&g_nwrites));
    CHECK(atomic_load(&g_nwrites) == 0, "small samples are batched, not written");
    CHECK(send_push(2 * LINK_MTU) == _Z_RES_OK, "a sample larger than the batch is sent");
    CHECK(_z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK) == _Z_RES_OK, "the batch flush ships what is parked");
    int n = atomic_load(&g_nwrites);
    dump_writes(n);
    int bad = -1;
    CHECK(n >= 3, "the parked batch and at least two fragments were written");
    CHECK(sns_in_order(n, &bad), "every write's SN is >= the one before it");
    CHECK(n > 0 && g_writes[0].mid == _Z_MID_T_FRAME, "the parked FRAME is written first");
    close_fake_session();
}

// 2. Two threads: a flusher keeps shipping batches (the detached send writes
//    under the link mutex only) while a publisher sends samples that fragment.
//    No two link writes may be in progress at once.
static atomic_int g_stop;

static void *flusher(void *arg) {
    (void)arg;
    while (!atomic_load(&g_stop)) {
        _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
        z_sleep_us(50);
    }
    return NULL;
}

static void *small_publisher(void *arg) {
    (void)arg;
    while (!atomic_load(&g_stop)) {
        send_push(16);
        z_sleep_us(20);
    }
    return NULL;
}

static void fragment_and_detached_send_never_overlap(void) {
    printf("a fragment write and a detached batch write never overlap\n");
    open_fake_session();
    atomic_store(&g_write_delay_us, 1000);  // a slow link widens the window
    atomic_store(&g_stop, 0);
    _z_task_t t_flush, t_small;
    _z_task_init(&t_flush, NULL, flusher, NULL);
    _z_task_init(&t_small, NULL, small_publisher, NULL);
    for (int i = 0; i < 200; i++) {
        send_push(3 * LINK_MTU);
        z_sleep_us(300);
    }
    atomic_store(&g_stop, 1);
    _z_task_join(&t_flush);
    _z_task_join(&t_small);
    _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
    int n = atomic_load(&g_nwrites);
    int overlaps = atomic_load(&g_overlaps);
    int bad = -1;
    bool ordered = sns_in_order(n < MAX_WRITES ? n : MAX_WRITES, &bad);
    printf("        %d write(s), %d overlapping, SN order %s\n", n, overlaps, ordered ? "kept" : "broken");
    CHECK(overlaps == 0, "no link write starts while another is in progress");
    CHECK(ordered, "every write's SN is >= the one before it");
    close_fake_session();
}

int main(void) {
    parked_batch_precedes_fragments();
    fragment_and_detached_send_never_overlap();
    if (failures != 0) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}

#else
int main(void) {
    printf("SKIP: needs Z_FEATURE_TX_SPLIT_LOCK, BATCHING, FRAGMENTATION and MULTI_THREAD\n");
    return 0;
}
#endif
