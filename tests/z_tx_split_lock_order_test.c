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

static z_result_t send_push_qos(size_t len, _z_n_qos_t qos) {
    _z_wireexpr_t key = _z_wireexpr_null();
    key._id = 1;
    _z_bytes_t payload;
    _z_bytes_from_buf(&payload, g_payload, len);
    _z_network_message_t msg;
    _z_n_msg_make_push_put(&msg, &key, &payload, NULL, qos, NULL, NULL, Z_RELIABILITY_RELIABLE, NULL);
    z_result_t ret = _z_send_n_msg(&g_zn, &msg, Z_RELIABILITY_RELIABLE, Z_CONGESTION_CONTROL_BLOCK, NULL);
    _z_bytes_drop(&payload);
    return ret;
}

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

// nano-ros issue 2614 -- every frame and every fragment takes one reliable SN,
// so the wire must carry them exactly consecutive. That also says a fragment
// train is unbroken (nothing can sit between two of its fragments) and that
// a train used exactly the SNs it reserved.
static bool sns_consecutive(int n, int *first_bad) {
    for (int i = 1; i < n; i++) {
        if (g_writes[i].sn != g_writes[i - 1].sn + 1) {
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
    bool consecutive = sns_consecutive(n < MAX_WRITES ? n : MAX_WRITES, &bad);
    if (!consecutive) {
        printf("        write %d: sn=%llu after sn=%llu\n", bad, (unsigned long long)g_writes[bad].sn,
               (unsigned long long)g_writes[bad - 1].sn);
    }
    CHECK(consecutive, "every write's SN is one more than the one before it (no train is broken)");
    close_fake_session();
}

// 3. nano-ros issue 2614: a fragment train is written under the link mutex
//    only, so a publisher that appends to the batch while it is on the wire
//    does not wait for it. The link takes 20 ms per write; a train of three
//    or more fragments holds it for 60 ms or more.
static atomic_int g_train_done;

static void *train_sender(void *arg) {
    (void)arg;
    send_push(3 * LINK_MTU);
    atomic_store(&g_train_done, 1);
    return NULL;
}

static void publisher_does_not_wait_out_a_train(void) {
    printf("a small publish during a fragment train does not wait for the train\n");
    open_fake_session();
    atomic_store(&g_write_delay_us, 20000);
    atomic_store(&g_train_done, 0);
    _z_task_t t_train;
    _z_task_init(&t_train, NULL, train_sender, NULL);
    while (atomic_load(&g_in_write) == 0 && atomic_load(&g_train_done) == 0) {
        z_sleep_us(100);
    }
    z_clock_t start = z_clock_now();
    z_result_t ret = send_push(16);
    unsigned long waited_ms = z_clock_elapsed_ms(&start);
    bool train_still_running = atomic_load(&g_train_done) == 0;
    _z_task_join(&t_train);
    _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
    int n = atomic_load(&g_nwrites);
    int bad = -1;
    dump_writes(n);
    printf("        the small publish took %lu ms; the train was %s when it returned\n", waited_ms,
           train_still_running ? "still on the wire" : "done");
    CHECK(ret == _Z_RES_OK, "the small publish succeeds");
    CHECK(waited_ms < 10 && train_still_running, "the small publish returns while the train is still on the wire");
    CHECK(sns_consecutive(n, &bad), "every write's SN is one more than the one before it");
    CHECK(n >= 4 && g_writes[n - 1].mid == _Z_MID_T_FRAME, "the small sample's batch follows the whole train");
    close_fake_session();
}

// 5. The periodic flush (DROP) does not wait for a busy link while it holds
//    the tx mutex: it leaves the batch, and a publisher right behind it is
//    not stalled for the rest of the train.
static void drop_flush_leaves_a_busy_link(void) {
    printf("a DROP flush during a fragment train returns at once and leaves the batch\n");
    open_fake_session();
    atomic_store(&g_write_delay_us, 20000);
    atomic_store(&g_train_done, 0);
    _z_task_t t_train;
    _z_task_init(&t_train, NULL, train_sender, NULL);
    while (atomic_load(&g_in_write) == 0 && atomic_load(&g_train_done) == 0) {
        z_sleep_us(100);
    }
    send_push(16);
    z_clock_t start = z_clock_now();
    z_result_t ret = _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_DROP);
    unsigned long flush_ms = z_clock_elapsed_ms(&start);
    size_t left = g_zn._tp._transport._unicast._common._batch_count;
    start = z_clock_now();
    send_push(16);
    unsigned long pub_ms = z_clock_elapsed_ms(&start);
    bool train_still_running = atomic_load(&g_train_done) == 0;
    _z_task_join(&t_train);
    _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
    int n = atomic_load(&g_nwrites);
    int bad = -1;
    printf("        the flush took %lu ms and left %u sample(s); the next publish took %lu ms; train %s\n", flush_ms,
           (unsigned)left, pub_ms, train_still_running ? "still on the wire" : "done");
    CHECK(ret == _Z_RES_OK, "the DROP flush reports success");
    CHECK(flush_ms < 10 && left == 1, "the DROP flush returns at once and keeps the batch");
    CHECK(pub_ms < 10 && train_still_running, "the next publish is not stalled by the train");
    CHECK(sns_consecutive(n, &bad), "every write's SN is one more than the one before it");
    close_fake_session();
}

// 6. A publisher that overflows the batch while an earlier overflow is still
//    parked ships that parked batch under the link mutex alone, so a small
//    publisher keeps appending while it is on the wire. A burst of samples of
//    0.6 batch each (every one overflows) runs over a 20 ms link while a
//    small publisher sends every 5 ms.
static atomic_int g_burst_done;

static void *burst_sender(void *arg) {
    (void)arg;
    for (int i = 0; i < 8; i++) {
        send_push((LINK_MTU * 6) / 10);
    }
    atomic_store(&g_burst_done, 1);
    return NULL;
}

static int cmp_ulong(const void *a, const void *b) {
    unsigned long x = *(const unsigned long *)a, y = *(const unsigned long *)b;
    return (x > y) - (x < y);
}

static void small_publisher_keeps_going_during_overflow_drains(void) {
    printf("a small publisher keeps appending while an overflow's parked batch is written\n");
    open_fake_session();
    atomic_store(&g_write_delay_us, 20000);
    atomic_store(&g_burst_done, 0);
    _z_task_t t_burst;
    _z_task_init(&t_burst, NULL, burst_sender, NULL);
    static unsigned long lat_us[4096];
    int n_lat = 0;
    while (atomic_load(&g_burst_done) == 0 && n_lat < 4096) {
        z_clock_t start = z_clock_now();
        send_push(16);
        lat_us[n_lat++] = z_clock_elapsed_us(&start);
        z_sleep_us(5000);
    }
    _z_task_join(&t_burst);
    _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
    qsort(lat_us, (size_t)n_lat, sizeof(lat_us[0]), cmp_ulong);
    int over = 0;
    for (int i = 0; i < n_lat; i++) {
        if (lat_us[i] >= 10000) {
            over++;
        }
    }
    int n = atomic_load(&g_nwrites);
    int bad = -1;
    printf("        %d small publish(es): p50 %lu us, p90 %lu us, max %lu us, %d of them >= 10 ms\n", n_lat,
           n_lat ? lat_us[n_lat / 2] : 0ul, n_lat ? lat_us[(n_lat * 9) / 10] : 0ul, n_lat ? lat_us[n_lat - 1] : 0ul,
           over);
    CHECK(n_lat > 0 && lat_us[(n_lat * 9) / 10] < 10000,
          "nine in ten small publishes return without a link write's wait");
    CHECK(sns_consecutive(n, &bad), "every write's SN is one more than the one before it");
    close_fake_session();
}

// 7. An express sample needs the link now, so it waits for a train on the
//    wire; it waits without the tx mutex, so a batched publisher behind it
//    is not held up.
static atomic_int g_express_done;

static void *express_sender(void *arg) {
    (void)arg;
    send_push_qos(16, _z_n_qos_make(1, 1, Z_PRIORITY_DEFAULT));
    atomic_store(&g_express_done, 1);
    return NULL;
}

static void express_waits_for_the_link_without_the_tx_mutex(void) {
    printf("an express sample waits for a train without holding up a batched publisher\n");
    open_fake_session();
    atomic_store(&g_write_delay_us, 20000);
    atomic_store(&g_train_done, 0);
    atomic_store(&g_express_done, 0);
    _z_task_t t_train, t_express;
    _z_task_init(&t_train, NULL, train_sender, NULL);
    while (atomic_load(&g_in_write) == 0 && atomic_load(&g_train_done) == 0) {
        z_sleep_us(100);
    }
    _z_task_init(&t_express, NULL, express_sender, NULL);
    z_sleep_us(5000);
    z_clock_t start = z_clock_now();
    z_result_t ret = send_push(16);
    unsigned long pub_ms = z_clock_elapsed_ms(&start);
    bool train_still_running = atomic_load(&g_train_done) == 0;
    bool express_waiting = atomic_load(&g_express_done) == 0;
    _z_task_join(&t_train);
    _z_task_join(&t_express);
    _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
    int n = atomic_load(&g_nwrites);
    int bad = -1;
    dump_writes(n);
    printf("        the batched publish took %lu ms; the express sample was %s, the train %s\n", pub_ms,
           express_waiting ? "still waiting" : "out", train_still_running ? "still on the wire" : "done");
    CHECK(ret == _Z_RES_OK, "the batched publish succeeds");
    CHECK(pub_ms < 10 && express_waiting && train_still_running,
          "the batched publish returns while the express sample waits for the train");
    CHECK(sns_consecutive(n, &bad), "every write's SN is one more than the one before it");
    close_fake_session();
}

// 4. The SNs a train reserves are counted before it is cut; the count must
//    match the serializer for every size, including across the 127 -> 128 SN
//    step where the SN's z-int, and so the fragment header, grows a byte.
static void train_uses_exactly_its_reserved_sns(void) {
    printf("a train uses exactly the SNs it reserved, for every size\n");
    open_fake_session();
    g_zn._tp._transport._unicast._common._sn_tx_reliable = 100;
    for (size_t len = LINK_MTU - 16; len <= 4 * LINK_MTU - 16; len += 7) {
        send_push(len);
        send_push(16);
    }
    _z_send_n_batch(&g_zn, Z_CONGESTION_CONTROL_BLOCK);
    int n = atomic_load(&g_nwrites);
    int bad = -1;
    bool consecutive = sns_consecutive(n < MAX_WRITES ? n : MAX_WRITES, &bad);
    printf("        %d write(s), SN %llu .. %llu\n", n, n > 0 ? (unsigned long long)g_writes[0].sn : 0ull,
           n > 0 ? (unsigned long long)g_writes[n - 1].sn : 0ull);
    if (!consecutive) {
        printf("        write %d: sn=%llu after sn=%llu\n", bad, (unsigned long long)g_writes[bad].sn,
               (unsigned long long)g_writes[bad - 1].sn);
    }
    CHECK(n > 0 && g_writes[n - 1].sn > 128, "the SNs crossed 128");
    CHECK(consecutive, "every write's SN is one more than the one before it");
    close_fake_session();
}

int main(void) {
    parked_batch_precedes_fragments();
    fragment_and_detached_send_never_overlap();
    publisher_does_not_wait_out_a_train();
    drop_flush_leaves_a_busy_link();
    small_publisher_keeps_going_during_overflow_drains();
    express_waits_for_the_link_without_the_tx_mutex();
    train_uses_exactly_its_reserved_sns();
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
