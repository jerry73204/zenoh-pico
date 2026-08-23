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

#ifndef ZENOH_PICO_SYSTEM_LINK_CAN_H
#define ZENOH_PICO_SYSTEM_LINK_CAN_H

#include <stdint.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/result.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_CAN == 1

// One CAN frame carries one zenoh datagram.
//
// CAN FD payload lengths are quantised — the DLC encodes 0..8, 12, 16, 20, 24,
// 32, 48, 64 and nothing between — so a 40-byte datagram travels in a 48-byte
// frame and the receiver cannot recover the true length from the frame alone.
// Byte 0 of every payload is therefore the datagram length and bytes 1..N are
// the datagram, which costs one byte and keeps the link from having to parse
// zenoh's own headers to find a boundary (RFC-0080 section 4.1).
#define _Z_CAN_LEN_PREFIX 1u

#define _Z_CAN_FD_FRAME_LEN 64u
#define _Z_CAN_CLASSIC_FRAME_LEN 8u

#define _Z_CAN_FD_MTU_SIZE (_Z_CAN_FD_FRAME_LEN - _Z_CAN_LEN_PREFIX)          // 63
#define _Z_CAN_CLASSIC_MTU_SIZE (_Z_CAN_CLASSIC_FRAME_LEN - _Z_CAN_LEN_PREFIX)  // 7

// Anything larger is fragmented by zenoh's transport before it reaches the
// link (`Z_FEATURE_FRAGMENTATION`, and `tx.c` clamping to `min(mtu, batch)`),
// so the link never sees a write it cannot place in one frame.
#define _Z_CAN_MTU_SIZE _Z_CAN_FD_MTU_SIZE

typedef struct {
    _z_sys_net_socket_t _sock;
    uint32_t _tx_id;
    uint32_t _rx_id;
    uint16_t _mtu;  // _Z_CAN_FD_MTU_SIZE or _Z_CAN_CLASSIC_MTU_SIZE
    _Bool _fd_mode;
} _z_can_socket_t;

/**
 * Open a CAN link.
 *
 * `dbitrate` of 0 selects classic CAN; any other value selects CAN FD with
 * that data-phase rate. On platforms whose interface is preconfigured (a
 * Linux `ip link set`, or a Zephyr devicetree `bitrate` property) the rate
 * arguments may be advisory — the call must still succeed rather than fail
 * on a rate it cannot apply, and must report the mode it actually got in
 * `sock->_fd_mode`.
 */
z_result_t _z_open_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint32_t tx_id,
                       uint32_t rx_id);

/** Listen side. CAN is a bus with no connection setup, so this differs from
 *  `_z_open_can` only in which identifier is used for which direction. */
z_result_t _z_listen_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint32_t tx_id,
                         uint32_t rx_id);

void _z_close_can(_z_can_socket_t *sock);

/** Read one datagram. Returns its length, or `SIZE_MAX` on error. Frames whose
 *  identifier is not `_rx_id` are skipped, so a shared bus does not deliver
 *  foreign traffic into the transport. */
size_t _z_read_can(const _z_can_socket_t *sock, uint8_t *ptr, size_t len);

/** Write one datagram, which must be <= the socket's MTU. Returns the number
 *  of bytes written, or `SIZE_MAX` on error. */
size_t _z_send_can(const _z_can_socket_t *sock, const uint8_t *ptr, size_t len);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_LINK_CAN_H */
