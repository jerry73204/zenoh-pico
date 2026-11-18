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
#include "zenoh-pico/link/transport/framed.h"
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
// The framing is the framed links' (`transport/framed.h`); these names stay
// for the IVC backend and its bridge daemons.
#define _Z_IVC_MTU_SIZE _Z_FRAMED_MTU_SIZE
#define _Z_IVC_FRAME_HEADER_SIZE _Z_FRAMED_HEADER_SIZE
#define _Z_IVC_FRAME_MAX _Z_FRAMED_FRAME_MAX
typedef _z_framed_socket_t _z_ivc_socket_t;

// =============================================================================
// C ABI provided by zpico-platform-shim::ivc_helpers
// (Phase 100.4 §4 + Phase 11.3.A zero-copy upgrade).
//
// Eight symbols dispatch through `<P as PlatformIvc>` and are the
// entire C↔Rust contract for the new transport. NVIDIA's FSP IVC API
// is fundamentally zero-copy (borrow ring slot, fill/read in place,
// commit / release); we mirror that here so the link layer doesn't
// memcpy through stack scratch frames every batch.
// =============================================================================

void *_z_open_ivc(uint32_t channel_id);
void _z_close_ivc(void *ch);
void _z_ivc_notify(void *ch);
uint32_t _z_ivc_frame_size(void *ch);

// Zero-copy RX path. `_z_ivc_rx_get` returns a pointer into the ring
// (and writes the frame length to `*len_out`); `_z_ivc_rx_release`
// hands the slot back to the producer. Returns NULL + `*len_out = 0`
// if no frame is available.
const uint8_t *_z_ivc_rx_get(void *ch, size_t *len_out);
void _z_ivc_rx_release(void *ch);

// Zero-copy TX path. `_z_ivc_tx_get` returns a writable pointer into
// the ring (and writes the slot capacity to `*cap_out`); the caller
// fills, then `_z_ivc_tx_commit(len)` makes the slot visible to the
// peer. `_z_ivc_tx_abandon` frees the slot without sending. Returns
// NULL + `*cap_out = 0` if the ring is full.
uint8_t *_z_ivc_tx_get(void *ch, size_t *cap_out);
void _z_ivc_tx_commit(void *ch, size_t len);
void _z_ivc_tx_abandon(void *ch);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_LINK_IVC_H */
