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

// nano-ros issue 1731: a peer is expired on the lease IT announced, never on
// ours, and the lease task may wake more often than that lease.

#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "zenoh-pico/transport/transport.h"

static _z_transport_peer_common_t fresh_peer(void) {
    _z_transport_peer_common_t p;
    memset(&p, 0, sizeof(p));
    p._received = true;  // as _z_transport_peer_unicast_add leaves it
    return p;
}

// A router announcing 60 s and speaking every 30 s, judged by a lease task that
// wakes every 10 s (our own, shorter lease): never expired.
static void test_quiet_router_keeping_its_word(void) {
    _z_transport_peer_common_t p = fresh_peer();
    assert(!_z_transport_peer_common_lease_check(&p, 60000, 0));
    for (int period = 1; period <= 30; period++) {
        p._received = (period % 3) == 0;  // one keep-alive per 30 s
        assert(!_z_transport_peer_common_lease_check(&p, 60000, 10000));
        assert(!p._expired);
        p._received = false;  // what the lease task does after the check
    }
}

// The same router going silent: expired once its OWN 60 s have passed with
// nothing, not after our 10 s.
static void test_silent_router_expires_on_its_lease(void) {
    _z_transport_peer_common_t p = fresh_peer();
    assert(!_z_transport_peer_common_lease_check(&p, 60000, 10000));
    p._received = false;
    for (int period = 1; period < 6; period++) {
        assert(!_z_transport_peer_common_lease_check(&p, 60000, 10000));
    }
    assert(_z_transport_peer_common_lease_check(&p, 60000, 10000));
    assert(p._expired);
    assert(p._silent_ms == 60000);
}

// One frame resets the count.
static void test_a_frame_resets_the_silence(void) {
    _z_transport_peer_common_t p = fresh_peer();
    p._received = false;
    for (int period = 0; period < 5; period++) {
        assert(!_z_transport_peer_common_lease_check(&p, 60000, 10000));
    }
    p._received = true;
    assert(!_z_transport_peer_common_lease_check(&p, 60000, 10000));
    assert(p._silent_ms == 0);
    p._received = false;
    assert(!_z_transport_peer_common_lease_check(&p, 60000, 10000));
}

// The client case: the task sleeps the peer's lease, so one silent period is
// enough (the old behaviour, on the right lease).
static void test_one_silent_lease_period_expires(void) {
    _z_transport_peer_common_t p = fresh_peer();
    assert(!_z_transport_peer_common_lease_check(&p, 10000, 0));
    p._received = false;
    assert(_z_transport_peer_common_lease_check(&p, 10000, 10000));
}

int main(void) {
    test_quiet_router_keeping_its_word();
    test_silent_router_expires_on_its_lease();
    test_a_frame_resets_the_silence();
    test_one_silent_lease_period_expires();
    printf("z_lease_check_test: OK\n");
    return 0;
}
