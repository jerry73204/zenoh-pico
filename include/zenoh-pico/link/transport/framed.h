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
//   nano-ros -- one framing for links that carry zenoh batches as a sequence
//   of fixed-size frames between exactly two endpoints (IVC, RPMsg).

#ifndef ZENOH_PICO_SYSTEM_LINK_FRAMED_H
#define ZENOH_PICO_SYSTEM_LINK_FRAMED_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "zenoh-pico/config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_IVC == 1 || Z_FEATURE_LINK_RPMSG == 1

// A zenoh batch of up to `_Z_FRAMED_MTU_SIZE` bytes is carried as frames of
// the channel's frame size, each with a 4-byte header: total batch length
// (u16 LE) and this frame's offset in it (u16 LE). The receiver reassembles
// in order; the channel is a single-producer single-consumer FIFO, so frames
// never reorder. `total == 0 && offset == 0` is a reserved keep-alive.
#define _Z_FRAMED_MTU_SIZE 2048
#define _Z_FRAMED_HEADER_SIZE 4
#define _Z_FRAMED_FRAME_MAX 1024

// The channel a framed link runs on. Every call is non-blocking: `rx_get`
// returns NULL when no frame is waiting and `tx_get` returns NULL when the
// transmit side is full. `tx_get` lends a slot of at least the frame size;
// `tx_commit` sends it and answers whether it was sent (a channel that copies
// on send, like RPMsg, can fail there); `rx_get` lends the next received
// frame until `rx_release`.
typedef struct {
    void (*close)(void *ch);
    void (*notify)(void *ch);
    uint32_t (*frame_size)(void *ch);
    const uint8_t *(*rx_get)(void *ch, size_t *len_out);
    void (*rx_release)(void *ch);
    uint8_t *(*tx_get)(void *ch, size_t *cap_out);
    bool (*tx_commit)(void *ch, size_t len);
    void (*tx_abandon)(void *ch);
} _z_framed_backend_t;

typedef struct {
    const _z_framed_backend_t *_ops;
    void *_ch;
    uint16_t _frame_size;
    // Reassembly state. `_expected_total == 0` means no batch in progress.
    uint16_t _expected_total;
    uint16_t _bytes_received;
    uint8_t _rx_buf[_Z_FRAMED_MTU_SIZE];
} _z_framed_socket_t;

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_LINK_FRAMED_H */
