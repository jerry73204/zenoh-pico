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

#include "zenoh-pico/transport/common/rx.h"

#include <stddef.h>

#include "zenoh-pico/protocol/codec/transport.h"
#include "zenoh-pico/session/utils.h"
#include "zenoh-pico/transport/multicast/rx.h"
#include "zenoh-pico/transport/unicast/rx.h"
#include "zenoh-pico/utils/endianness.h"
#include "zenoh-pico/utils/logging.h"

/*------------------ Reception helper ------------------*/
size_t _z_read_stream_size(_z_zbuf_t *zbuf) {
    uint8_t stream_size[_Z_MSG_LEN_ENC_SIZE];
    // Read the bytes from stream
    for (uint8_t i = 0; i < _Z_MSG_LEN_ENC_SIZE; i++) {
        stream_size[i] = _z_zbuf_read(zbuf);
    }
    return _z_host_le_load16(stream_size);
}

z_result_t _z_link_recv_t_msg(_z_transport_message_t *t_msg, const _z_link_t *zl, _z_sys_net_socket_t *socket) {
    z_result_t ret = _Z_RES_OK;

    // Create and prepare the buffer
    _z_zbuf_t zbf = _z_zbuf_make(Z_BATCH_UNICAST_SIZE);
    _z_zbuf_reset(&zbf);

    switch (zl->_cap._flow) {
        case Z_LINK_CAP_FLOW_STREAM:
            // Read the message length
            if (_z_link_recv_exact_zbuf(zl, &zbf, _Z_MSG_LEN_ENC_SIZE, NULL, socket) == _Z_MSG_LEN_ENC_SIZE) {
                size_t len = 0;
                for (uint8_t i = 0; i < _Z_MSG_LEN_ENC_SIZE; i++) {
                    len |= (size_t)(_z_zbuf_read(&zbf) << (i * (uint8_t)8));
                }

                size_t writable = _z_zbuf_capacity(&zbf) - _z_zbuf_len(&zbf);
                if (writable >= len) {
                    // Read enough bytes to decode the message
                    if (_z_link_recv_exact_zbuf(zl, &zbf, len, NULL, socket) != len) {
                        _Z_ERROR_LOG(_Z_ERR_TRANSPORT_RX_FAILED);
                        ret = _Z_ERR_TRANSPORT_RX_FAILED;
                    }
                } else {
                    _Z_ERROR_LOG(_Z_ERR_TRANSPORT_NO_SPACE);
                    ret = _Z_ERR_TRANSPORT_NO_SPACE;
                }
            } else {
                _Z_ERROR_LOG(_Z_ERR_TRANSPORT_RX_FAILED);
                ret = _Z_ERR_TRANSPORT_RX_FAILED;
            }
            break;
        case Z_LINK_CAP_FLOW_DATAGRAM:
            if (_z_link_recv_zbuf(zl, &zbf, NULL) == SIZE_MAX) {
                _Z_ERROR_LOG(_Z_ERR_TRANSPORT_RX_FAILED);
                ret = _Z_ERR_TRANSPORT_RX_FAILED;
            }
            break;
        default:
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
            break;
    }
    if (ret == _Z_RES_OK) {
        _z_transport_message_t l_t_msg;
        ret = _z_transport_message_decode(&l_t_msg, &zbf);
        if (ret == _Z_RES_OK) {
            _z_t_msg_copy(t_msg, &l_t_msg);
        }
    }
    _z_zbuf_clear(&zbf);

    return ret;
}

volatile _z_rx_rejections_t _z_rx_rejections = {0, _Z_RES_OK};

/* A frame carries a batch of network messages, and before this every handler
 * error in the batch was returned from the frame handler -- up through
 * `_z_unicast_process_messages`, whose caller then stopped the read task for
 * good. Nothing closed the session: the lease task kept sending, so the peer saw
 * a live link, while this side received nothing until its own lease expired two
 * periods later. On a serial link to a router that was always one lost frame, or
 * one duplicate undeclaration, away. Measured on an S32K344 over UART: the read
 * task died within a second of a router's liveliness history burst, and the
 * session closed 21 s after it opened, when the island's own lease ran out.
 *
 * A handler error is a verdict on ONE message: its key id was declared in a
 * frame the link dropped, its id was never declared to us, there was no memory
 * for its payload. The next message in the batch, and the next frame, are
 * unaffected -- so reject this one, count it, and go on. The fragment path
 * already did exactly this (it discards the handler's result); the frame paths
 * now match it. A DECODE error is different and stays fatal in the callers:
 * after one, the position in the batch is unknown. */
z_result_t _z_handle_network_message_in_frame(_z_transport_common_t *zt, _z_zenoh_message_t *msg,
                                              _z_transport_peer_common_t *peer) {
    z_result_t ret = _z_handle_network_message(zt, msg, peer);
    if ((ret == _Z_RES_OK) || (ret == _Z_ERR_CONNECTION_CLOSED) || (ret == _Z_ERR_SESSION_CLOSED)) {
        return ret;
    }
    _z_rx_rejections.count = _z_rx_rejections.count + 1u;
    _z_rx_rejections.last_error = ret;
    _Z_WARN("Rejected one received network message (%d); the link continues", (int)ret);
    return _Z_RES_OK;
}
