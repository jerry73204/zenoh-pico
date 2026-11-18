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
//   nano-ros -- RPMsg link transport. The channel calls, supplied by the
//   application (nano-ros's `zpico-link-rpmsg`), on the same lend-and-commit
//   shape as IVC's, so the framed link drives both.

#ifndef ZENOH_PICO_SYSTEM_LINK_RPMSG_H
#define ZENOH_PICO_SYSTEM_LINK_RPMSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/transport/framed.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_RPMSG == 1

// Open the RPMsg endpoint named by the locator address (`rpmsg/<name>`, the
// name not NUL-terminated). NULL when no such endpoint can be opened.
void *_z_open_rpmsg(const char *name, size_t name_len);
void _z_close_rpmsg(void *ch);
void _z_rpmsg_notify(void *ch);
uint32_t _z_rpmsg_frame_size(void *ch);
const uint8_t *_z_rpmsg_rx_get(void *ch, size_t *len_out);
void _z_rpmsg_rx_release(void *ch);
uint8_t *_z_rpmsg_tx_get(void *ch, size_t *cap_out);
bool _z_rpmsg_tx_commit(void *ch, size_t len);
void _z_rpmsg_tx_abandon(void *ch);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_LINK_RPMSG_H */
