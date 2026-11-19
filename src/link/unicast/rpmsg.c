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
//   nano-ros -- RPMsg link transport, between images on different cores of
//   one SoC. An endpoint joins exactly two ends, so the link is a framed link
//   (`framed.c`) with RPMsg's channel calls as its backend; the frame size is
//   the RPMsg payload the backend reports (496 bytes with 512-byte buffers).

#include "zenoh-pico/link/transport/rpmsg.h"

#include <stddef.h>
#include <string.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/framed.h"
#include "zenoh-pico/link/manager.h"

#if Z_FEATURE_LINK_RPMSG == 1

// `rpmsg/<name>`, with a non-empty name.
z_result_t _z_endpoint_rpmsg_valid(_z_endpoint_t *endpoint) {
    _z_string_t rpmsg_str = _z_string_alias_str(RPMSG_SCHEMA);
    if (!_z_string_equals(&endpoint->_locator._protocol, &rpmsg_str) ||
        (_z_string_len(&endpoint->_locator._address) == 0)) {
        _Z_ERROR_LOG(_Z_ERR_CONFIG_LOCATOR_INVALID);
        return _Z_ERR_CONFIG_LOCATOR_INVALID;
    }
    return _Z_RES_OK;
}

static const _z_framed_backend_t _z_rpmsg_backend = {
    .close = _z_close_rpmsg,
    .notify = _z_rpmsg_notify,
    .frame_size = _z_rpmsg_frame_size,
    .rx_get = _z_rpmsg_rx_get,
    .rx_release = _z_rpmsg_rx_release,
    .tx_get = _z_rpmsg_tx_get,
    .tx_commit = _z_rpmsg_tx_commit,
    .tx_abandon = _z_rpmsg_tx_abandon,
    .rx_wait = _z_rpmsg_rx_wait,
};

static z_result_t _z_f_link_open_rpmsg(_z_link_t *self) {
    const _z_string_t *name = &self->_endpoint._locator._address;
    return _z_framed_bind(self, &_z_rpmsg_backend, _z_open_rpmsg(_z_string_data(name), _z_string_len(name)));
}

// An RPMsg endpoint is symmetric: listen is the same as open.
static z_result_t _z_f_link_listen_rpmsg(_z_link_t *self) { return _z_f_link_open_rpmsg(self); }

z_result_t _z_new_link_rpmsg(_z_link_t *zl, _z_endpoint_t endpoint) {
    _z_framed_link_init(zl, endpoint, _Z_LINK_TYPE_RPMSG);
    zl->_open_f = _z_f_link_open_rpmsg;
    zl->_listen_f = _z_f_link_listen_rpmsg;
    return _Z_RES_OK;
}

#endif
