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
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

// The multi-threaded collections must release their mutex when a condition
// variable call fails. Before this test, `_Z_RETURN_IF_ERR` around
// `_z_condvar_wait` / `_z_condvar_signal` returned with the mutex still held,
// so the next caller deadlocked.
//
// The failures are injected with the linker's `--wrap`: this test compiles
// the collection sources itself, so their calls to `_z_condvar_*` resolve to
// the `__wrap_` functions below, which fail on demand.

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "zenoh-pico/collections/fifo_mt.h"
#include "zenoh-pico/collections/ring_mt.h"
#include "zenoh-pico/collections/sync_group.h"
#include "zenoh-pico/system/platform.h"

#undef NDEBUG
#include <assert.h>

#if Z_FEATURE_MULTI_THREAD == 1

static bool fail_wait = false;
static bool fail_signal = false;

z_result_t __real__z_condvar_wait(_z_condvar_t *cv, _z_mutex_t *m);
z_result_t __real__z_condvar_wait_until(_z_condvar_t *cv, _z_mutex_t *m, const z_clock_t *abstime);
z_result_t __real__z_condvar_signal(_z_condvar_t *cv);
z_result_t __real__z_condvar_signal_all(_z_condvar_t *cv);

z_result_t __wrap__z_condvar_wait(_z_condvar_t *cv, _z_mutex_t *m) {
    return fail_wait ? _Z_ERR_GENERIC : __real__z_condvar_wait(cv, m);
}

z_result_t __wrap__z_condvar_wait_until(_z_condvar_t *cv, _z_mutex_t *m, const z_clock_t *abstime) {
    return fail_wait ? _Z_ERR_GENERIC : __real__z_condvar_wait_until(cv, m, abstime);
}

z_result_t __wrap__z_condvar_signal(_z_condvar_t *cv) {
    return fail_signal ? _Z_ERR_GENERIC : __real__z_condvar_signal(cv);
}

z_result_t __wrap__z_condvar_signal_all(_z_condvar_t *cv) {
    return fail_signal ? _Z_ERR_GENERIC : __real__z_condvar_signal_all(cv);
}

// The mutex is free iff another acquisition succeeds. A try-lock from the
// same thread would succeed on a recursive mutex, so take it from another one.
static z_result_t try_lock_result;

static void *try_lock_task(void *arg) {
    _z_mutex_t *m = (_z_mutex_t *)arg;
    try_lock_result = _z_mutex_try_lock(m);
    if (try_lock_result == _Z_RES_OK) {
        _z_mutex_unlock(m);
    }
    return NULL;
}

static void assert_unlocked(_z_mutex_t *m, const char *site) {
    _z_task_t task;
    try_lock_result = _Z_ERR_GENERIC;
    assert(_z_task_init(&task, NULL, try_lock_task, m) == _Z_RES_OK);
    assert(_z_task_join(&task) == _Z_RES_OK);
    if (try_lock_result != _Z_RES_OK) {
        printf("FAIL %s: returned an error with the mutex held\n", site);
    }
    assert(try_lock_result == _Z_RES_OK);
    printf("ok   %s\n", site);
}

static void move_ptr(void *dst, void *src) { *(void **)dst = src; }
static void free_none(void **elem) { (void)elem; }

static int payload_a = 1;
static int payload_b = 2;

static void test_fifo(void) {
    _z_fifo_mt_t f;
    void *out = NULL;

    // push: wait for room fails.
    assert(_z_fifo_mt_init(&f, 1) == _Z_RES_OK);
    assert(_z_fifo_mt_push(&payload_a, &f, NULL) == _Z_RES_OK);
    fail_wait = true;
    assert(_z_fifo_mt_push(&payload_b, &f, NULL) != _Z_RES_OK);
    fail_wait = false;
    assert_unlocked(&f._mutex, "fifo push: wait");
    _z_fifo_mt_clear(&f, free_none);

    // push: signal not-empty fails.
    assert(_z_fifo_mt_init(&f, 1) == _Z_RES_OK);
    fail_signal = true;
    assert(_z_fifo_mt_push(&payload_a, &f, NULL) != _Z_RES_OK);
    fail_signal = false;
    assert_unlocked(&f._mutex, "fifo push: signal");
    _z_fifo_mt_clear(&f, free_none);

    // pull: wait for an element fails.
    assert(_z_fifo_mt_init(&f, 1) == _Z_RES_OK);
    fail_wait = true;
    assert(_z_fifo_mt_pull(&out, &f, move_ptr) != _Z_RES_OK);
    fail_wait = false;
    assert_unlocked(&f._mutex, "fifo pull: wait");

    // pull: signal not-full fails.
    assert(_z_fifo_mt_push(&payload_a, &f, NULL) == _Z_RES_OK);
    fail_signal = true;
    assert(_z_fifo_mt_pull(&out, &f, move_ptr) != _Z_RES_OK);
    fail_signal = false;
    assert_unlocked(&f._mutex, "fifo pull: signal");

    // try_pull: signal not-full fails.
    assert(_z_fifo_mt_push(&payload_a, &f, NULL) == _Z_RES_OK);
    fail_signal = true;
    assert(_z_fifo_mt_try_pull(&out, &f, move_ptr) != _Z_RES_OK);
    fail_signal = false;
    assert_unlocked(&f._mutex, "fifo try_pull: signal");

    // close: broadcast fails.
    fail_signal = true;
    assert(_z_fifo_mt_close(&f) != _Z_RES_OK);
    fail_signal = false;
    assert_unlocked(&f._mutex, "fifo close: signal_all");
    _z_fifo_mt_clear(&f, free_none);
}

static void test_ring(void) {
    _z_ring_mt_t r;
    void *out = NULL;

    assert(_z_ring_mt_init(&r, 1) == _Z_RES_OK);

    // pull: wait for an element fails.
    fail_wait = true;
    assert(_z_ring_mt_pull(&out, &r, move_ptr) != _Z_RES_OK);
    fail_wait = false;
    assert_unlocked(&r._mutex, "ring pull: wait");

    // push: signal not-empty fails.
    fail_signal = true;
    assert(_z_ring_mt_push(&payload_a, &r, NULL) != _Z_RES_OK);
    fail_signal = false;
    assert_unlocked(&r._mutex, "ring push: signal");

    // close: broadcast fails.
    fail_signal = true;
    assert(_z_ring_mt_close(&r) != _Z_RES_OK);
    fail_signal = false;
    assert_unlocked(&r._mutex, "ring close: signal_all");
    _z_ring_mt_clear(&r, free_none);
}

static void test_sync_group(void) {
    _z_sync_group_t sg;
    _z_sync_group_notifier_t n;
    assert(_z_sync_group_create(&sg) == _Z_RES_OK);
    // A live notifier keeps both waits in their loop.
    assert(_z_sync_group_create_notifier(&sg, &n) == _Z_RES_OK);
    _z_mutex_t *m = &_Z_RC_IN_VAL(&sg._state)->counter_mutex;

    fail_wait = true;
    assert(_z_sync_group_wait(&sg) != _Z_RES_OK);
    fail_wait = false;
    assert_unlocked(m, "sync_group wait: wait");

    z_clock_t deadline = z_clock_now();
    z_clock_advance_s(&deadline, 10);
    fail_wait = true;
    z_result_t ret = _z_sync_group_wait_deadline(&sg, &deadline);
    fail_wait = false;
    assert(ret != _Z_RES_OK && ret != Z_ETIMEDOUT);
    assert_unlocked(m, "sync_group wait_deadline: wait_until");

    _z_sync_group_notifier_drop(&n);
    _z_sync_group_drop(&sg);
}

int main(void) {
    test_fifo();
    test_ring();
    test_sync_group();
    return 0;
}

#else
int main(void) { return 0; }
#endif
