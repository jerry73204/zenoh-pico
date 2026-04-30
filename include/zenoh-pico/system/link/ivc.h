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
//   Phase 100.4 — NVIDIA Tegra IVC link transport.
//

#ifndef ZENOH_PICO_SYSTEM_LINK_IVC_H
#define ZENOH_PICO_SYSTEM_LINK_IVC_H

#include <stdint.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_IVC == 1

// MTU equals the maximum zenoh batch the link will reassemble. With a
// 64-byte IVC frame and a 4-byte framing header (`total_len` u16 +
// `offset` u16), each frame carries up to 60 payload bytes; a 2048-byte
// MTU therefore costs at most ⌈2048/60⌉ = 35 frames per batch.
//
// MTU is intentionally smaller than the on-wire `total_len` field's
// `u16` ceiling (65 535) so a future build can grow `Z_FEATURE_LINK_IVC`
// without changing the framing protocol — the wire format already
// supports anything ≤ 65 535. See
// `docs/roadmap/phase-100-04-link-ivc-design.md` §5.2.
#define _Z_IVC_MTU_SIZE 2048

// Per-frame framing header — keep in sync with §5.2 of the IVC link
// design doc. Used by both ends of the link (this library and the
// CCPLEX-side `autoware_sentinel/src/ivc-bridge/` daemon).
#define _Z_IVC_FRAME_HEADER_SIZE 4  // total_len (u16 LE) + offset (u16 LE)

// IVC frame size as advertised by the driver. NVIDIA's default carveout
// uses 64 bytes; we cache the actual value at open time and never
// re-query (design doc §8.2).
//
// 64 is the worst case the link layer is sized for — `_z_ivc_socket_t`
// stores its scratch frame buffer as `_Z_IVC_FRAME_MAX`, and oversized
// frames are rejected at open as "unsupported channel". 1024 covers
// every NVIDIA Tegra IVC profile we know about.
#define _Z_IVC_FRAME_MAX 1024

typedef struct {
    // Opaque IVC channel handle returned by `_z_open_ivc(channel_id)`.
    // Lifetime: equal to the program; the FSP / unix-mock owns the
    // underlying state and `_z_close_ivc` is a no-op.
    void *_ch;

    // Frame size negotiated for this channel, captured once at open.
    uint16_t _frame_size;

    // Reassembly state — see design doc §5.4. `_expected_total == 0`
    // means "no batch in progress; next frame starts a fresh batch".
    uint16_t _expected_total;
    uint16_t _bytes_received;
    uint8_t _rx_buf[_Z_IVC_MTU_SIZE];
} _z_ivc_socket_t;

// =============================================================================
// C ABI provided by zpico-platform-shim::ivc_helpers (Phase 100.4 §4).
// These five symbols dispatch through `<P as PlatformIvc>` and are the
// entire C↔Rust contract for the new transport.
// =============================================================================

void *_z_open_ivc(uint32_t channel_id);
size_t _z_read_ivc(void *ch, uint8_t *buf, size_t len);
size_t _z_send_ivc(void *ch, const uint8_t *buf, size_t len);
void _z_close_ivc(void *ch);
void _z_ivc_notify(void *ch);

// Frame size query is a sixth symbol that doesn't appear in the
// design's "five forwarders" table because it's only called once
// (during `_z_f_link_open_ivc` to populate `_frame_size`). Keeping it
// separate makes the hot-path forwarders smaller.
uint32_t _z_ivc_frame_size(void *ch);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_LINK_IVC_H */
