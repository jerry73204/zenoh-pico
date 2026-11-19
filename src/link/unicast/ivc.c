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
// This file is the link layer for IVC: its endpoint, and its channel
// calls as a framed-link backend. The framing that fragments zenoh batches
// across 64-byte IVC frames and reassembles them is shared with RPMsg, in
// `framed.c`; its wire format is unchanged (`docs/roadmap/phase-100-04-link-ivc-design.md`
// §5, which bridge daemons on the CCPLEX side cite).
//
// Read-buffer-size assumption: the link layer always passes a
// `frame_size`-sized buffer to `_z_read_ivc`. SOCK_DGRAM on the
// unix-mock truncates beyond the buffer, and `tegra_ivc_read` requires
// at least `frame_size` bytes available — both behave correctly given
// this assumption (design doc §9 item 2).

#include "zenoh-pico/link/config/ivc.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/framed.h"
#include "zenoh-pico/link/manager.h"
#include "zenoh-pico/link/transport/ivc.h"
#include "zenoh-pico/utils/pointers.h"

#if Z_FEATURE_LINK_IVC == 1

// =============================================================================
// Endpoint validation — accept `ivc/<channel-id>` where <channel-id> is
// a non-empty decimal integer.
// =============================================================================

z_result_t _z_endpoint_ivc_valid(_z_endpoint_t *endpoint) {
    z_result_t ret = _Z_RES_OK;

    _z_string_t ivc_str = _z_string_alias_str(IVC_SCHEMA);
    if (!_z_string_equals(&endpoint->_locator._protocol, &ivc_str)) {
        _Z_ERROR_LOG(_Z_ERR_CONFIG_LOCATOR_INVALID);
        ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
    }

    if (ret == _Z_RES_OK) {
        size_t addr_len = _z_string_len(&endpoint->_locator._address);
        if (addr_len == 0) {
            _Z_ERROR_LOG(_Z_ERR_CONFIG_LOCATOR_INVALID);
            ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
        }
    }

    return ret;
}

// Parse `<channel-id>` from the locator address string. Returns
// `_Z_RES_OK` on success and writes the parsed id to `*out`. Rejects
// addresses with non-decimal characters and addresses that don't fit
// in a `uint32_t`.
static z_result_t __z_parse_channel_id(const _z_string_t *addr, uint32_t *out) {
    size_t addr_len = _z_string_len(addr);
    const char *p_start = _z_string_data(addr);
    if ((addr_len == 0) || (p_start == NULL)) {
        return _Z_ERR_CONFIG_LOCATOR_INVALID;
    }

    // Bound the address to a small stack buffer so we can null-terminate
    // for `strtoul`. 16 chars covers `4294967295\0` (uint32 max) with
    // room to spare.
    char buf[16];
    if (addr_len >= sizeof(buf)) {
        return _Z_ERR_CONFIG_LOCATOR_INVALID;
    }
    memcpy(buf, p_start, addr_len);
    buf[addr_len] = '\0';

    char *endp = NULL;
    unsigned long parsed = strtoul(buf, &endp, 10);
    if ((endp == buf) || (*endp != '\0')) {
        return _Z_ERR_CONFIG_LOCATOR_INVALID;
    }
    if (parsed > 0xFFFFFFFFUL) {
        return _Z_ERR_CONFIG_LOCATOR_INVALID;
    }
    *out = (uint32_t)parsed;
    return _Z_RES_OK;
}

// =============================================================================
// The IVC channel as a framed-link backend, and open / listen.
// =============================================================================

// IVC's commit cannot fail: the slot was lent by `tx_get`.
static bool _z_ivc_tx_commit_ok(void *ch, size_t len) {
    _z_ivc_tx_commit(ch, len);
    return true;
}

static const _z_framed_backend_t _z_ivc_backend = {
    .close = _z_close_ivc,
    .notify = _z_ivc_notify,
    .frame_size = _z_ivc_frame_size,
    .rx_get = _z_ivc_rx_get,
    .rx_release = _z_ivc_rx_release,
    .tx_get = _z_ivc_tx_get,
    .tx_commit = _z_ivc_tx_commit_ok,
    .tx_abandon = _z_ivc_tx_abandon,
    .rx_wait = NULL,
};

z_result_t _z_f_link_open_ivc(_z_link_t *self) {
    uint32_t channel_id = 0;
    z_result_t ret = __z_parse_channel_id(&self->_endpoint._locator._address, &channel_id);
    if (ret != _Z_RES_OK) {
        return ret;
    }
    return _z_framed_bind(self, &_z_ivc_backend, _z_open_ivc(channel_id));
}

// IVC is symmetric -- listen is the same as open (design doc §8.3).
z_result_t _z_f_link_listen_ivc(_z_link_t *self) { return _z_f_link_open_ivc(self); }

uint16_t _z_get_link_mtu_ivc(void) { return _Z_IVC_MTU_SIZE; }

z_result_t _z_new_link_ivc(_z_link_t *zl, _z_endpoint_t endpoint) {
    _z_framed_link_init(zl, endpoint, _Z_LINK_TYPE_IVC);
    zl->_open_f = _z_f_link_open_ivc;
    zl->_listen_f = _z_f_link_listen_ivc;
    return _Z_RES_OK;
}

#endif
