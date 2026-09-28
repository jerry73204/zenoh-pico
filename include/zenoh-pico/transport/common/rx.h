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
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

#ifndef ZENOH_PICO_TRANSPORT_RX_H
#define ZENOH_PICO_TRANSPORT_RX_H

#include "zenoh-pico/link/link.h"
#include "zenoh-pico/transport/transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------------------ Transmission and Reception helpers ------------------*/
size_t _z_read_stream_size(_z_zbuf_t *zbuf);
z_result_t _z_link_recv_t_msg(_z_transport_message_t *t_msg, const _z_link_t *zl, _z_sys_net_socket_t *socket);

/* Network messages the session layer REJECTED after they decoded cleanly: an
 * undeclaration for an id this session never saw, a key id whose declaration
 * arrived in a frame the link lost, a sample there was no memory for. Counted
 * rather than returned, because returning one from a frame handler stopped the
 * transport's read task for good (see `_z_handle_network_message_in_frame`).
 * Process-wide and never reset: read them over a debugger, or from a test. */
typedef struct {
    uint32_t count;
    z_result_t last_error;
} _z_rx_rejections_t;
extern volatile _z_rx_rejections_t _z_rx_rejections;

/* Hand one decoded network message to the session layer from inside a FRAME
 * or a defragmented FRAGMENT. Returns an error only when the session or the
 * connection is closing; any other failure rejects this message alone, is
 * counted in `_z_rx_rejections`, and the caller goes on with the rest of the
 * batch. */
z_result_t _z_handle_network_message_in_frame(_z_transport_common_t *zt, _z_zenoh_message_t *msg,
                                              _z_transport_peer_common_t *peer);

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_TRANSPORT_RX_H */
