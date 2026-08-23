//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   RFC-0080 / phase-377 — CAN and CAN FD link transport.
//

// Link-level test for the CAN transport. Exercises the platform binding
// directly rather than through a session, so a failure points at the frame
// codec instead of at zenoh's transport state machine.
//
// Requires a CAN interface. Create a virtual one with:
//     sudo modprobe vcan
//     sudo ip link add dev vcan0 type vcan
//     sudo ip link set up vcan0
//
// Skips (exit 0) when the interface is absent, so it is safe in CI that has
// no CAN support.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "zenoh-pico/config.h"

#if Z_FEATURE_LINK_CAN == 1

#include <net/if.h>

#include "zenoh-pico/system/link/can.h"

#define CAN_DEV (getenv("ZP_CAN_DEV") != NULL ? getenv("ZP_CAN_DEV") : "vcan0")
#define TX_ID 0x100u
#define RX_ID 0x101u

#include <stdlib.h>

static int failures = 0;

static void check(const char *what, int ok) {
    printf("  %-56s %s\n", what, ok ? "ok" : "FAILED");
    if (!ok) {
        failures++;
    }
}

// One datagram out of `a`, one datagram in on `b`, compared byte for byte.
static void roundtrip(_z_can_socket_t *a, _z_can_socket_t *b, size_t len, const char *label) {
    uint8_t out[_Z_CAN_FD_MTU_SIZE];
    uint8_t in[_Z_CAN_FD_MTU_SIZE];

    for (size_t i = 0; i < len; i++) {
        out[i] = (uint8_t)(i * 7u + 1u);  // not all-zero, so a short read shows
    }

    size_t wb = _z_send_can(a, out, len);
    if (wb != len) {
        printf("  %-56s FAILED (send returned %zu)\n", label, wb);
        failures++;
        return;
    }

    memset(in, 0, sizeof(in));
    size_t rb = _z_read_can(b, in, sizeof(in));
    if (rb != len) {
        printf("  %-56s FAILED (read returned %zu, expected %zu)\n", label, rb, len);
        failures++;
        return;
    }

    check(label, memcmp(out, in, len) == 0);
}

int main(void) {
    const char *dev = CAN_DEV;

    if (if_nametoindex(dev) == 0) {
        printf("z_can_link_test: no interface '%s' — skipping\n", dev);
        printf("  create one with: sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0\n");
        return 0;
    }

    printf("z_can_link_test on '%s'\n", dev);

    // Two endpoints of one link: what A transmits, B receives.
    _z_can_socket_t a;
    _z_can_socket_t b;
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));

    if (_z_open_can(&a, dev, 500000u, 2000000u, TX_ID, RX_ID) != _Z_RES_OK) {
        printf("  open A FAILED\n");
        return 1;
    }
    if (_z_listen_can(&b, dev, 500000u, 2000000u, RX_ID, TX_ID) != _Z_RES_OK) {
        printf("  listen B FAILED\n");
        _z_close_can(&a);
        return 1;
    }

    printf("  mode: %s, MTU %u\n", a._fd_mode ? "CAN FD" : "classic CAN", (unsigned)a._mtu);

    // Boundaries that matter: empty, one byte, each side of a DLC step, and
    // the MTU itself. The DLC steps are where a length-prefix bug hides —
    // a 12-byte payload rides in a 12-byte frame, a 13-byte one in a 16.
    roundtrip(&a, &b, 0, "empty datagram");
    roundtrip(&a, &b, 1, "1 byte");
    roundtrip(&a, &b, 7, "7 bytes (classic CAN MTU)");
    roundtrip(&a, &b, 8, "8 bytes (DLC step)");
    if (a._fd_mode) {
        roundtrip(&a, &b, 11, "11 bytes (below a DLC step)");
        roundtrip(&a, &b, 12, "12 bytes (exact DLC step)");
        roundtrip(&a, &b, 13, "13 bytes (padded up to 16)");
        roundtrip(&a, &b, 47, "47 bytes (padded up to 48)");
        roundtrip(&a, &b, _Z_CAN_FD_MTU_SIZE, "63 bytes (CAN FD MTU)");
    }

    // Both directions — the identifier swap in listen() must actually work.
    roundtrip(&b, &a, 16, "reverse direction, 16 bytes");

    // Over-MTU writes must be refused, not truncated. zenoh's transport should
    // never send one, but a silent truncation here would corrupt a fragment.
    uint8_t big[_Z_CAN_FD_MTU_SIZE + 1];
    memset(big, 0xAB, sizeof(big));
    check("over-MTU write is refused", _z_send_can(&a, big, (size_t)a._mtu + 1u) == SIZE_MAX);

    _z_close_can(&a);
    _z_close_can(&b);

    printf("%s (%d failure%s)\n", (failures == 0) ? "PASS" : "FAIL", failures, (failures == 1) ? "" : "s");
    return (failures == 0) ? 0 : 1;
}

#else

int main(void) {
    printf("z_can_link_test: built without Z_FEATURE_LINK_CAN — skipping\n");
    return 0;
}

#endif
