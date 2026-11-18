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
//   nano-ros -- the link half of the framed links (IVC, RPMsg).

#ifndef ZENOH_PICO_LINK_FRAMED_H
#define ZENOH_PICO_LINK_FRAMED_H

#include "zenoh-pico/link/link.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_IVC == 1 || Z_FEATURE_LINK_RPMSG == 1

// Bind an opened channel to the link. Takes ownership of `ch`: on failure it
// is closed through `ops->close`.
z_result_t _z_framed_bind(_z_link_t *self, const _z_framed_backend_t *ops, void *ch);

// Fill in everything a framed link shares: capabilities (unicast, datagram,
// single peer), MTU and every callback but open and listen.
void _z_framed_link_init(_z_link_t *zl, _z_endpoint_t endpoint, uint8_t link_type);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_LINK_FRAMED_H */
