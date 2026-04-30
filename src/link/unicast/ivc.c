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
// This file is the link layer for IVC. It owns the framing state
// machine that fragments outgoing zenoh batches across 64-byte IVC
// frames and reassembles incoming ones. Wire format and rationale are
// in `docs/roadmap/phase-100-04-link-ivc-design.md` §5; bridge-daemon
// implementations on the CCPLEX side cite that doc by file path +
// commit hash.
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
#include "zenoh-pico/link/manager.h"
#include "zenoh-pico/system/link/ivc.h"
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
// Open / listen / close / free
// =============================================================================

z_result_t _z_f_link_open_ivc(_z_link_t *self) {
    uint32_t channel_id = 0;
    z_result_t ret = __z_parse_channel_id(&self->_endpoint._locator._address, &channel_id);
    if (ret != _Z_RES_OK) {
        return ret;
    }

    void *ch = _z_open_ivc(channel_id);
    if (ch == NULL) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        return _Z_ERR_GENERIC;
    }

    uint32_t fs = _z_ivc_frame_size(ch);
    if ((fs <= _Z_IVC_FRAME_HEADER_SIZE) || (fs > _Z_IVC_FRAME_MAX)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        _z_close_ivc(ch);
        return _Z_ERR_GENERIC;
    }

    self->_socket._ivc._ch = ch;
    self->_socket._ivc._frame_size = (uint16_t)fs;
    self->_socket._ivc._expected_total = 0;
    self->_socket._ivc._bytes_received = 0;
    return _Z_RES_OK;
}

// IVC is symmetric — listen is the same as open (design doc §8.3).
// Mirrors `_z_listen_serial_from_dev` → `_z_open_serial_from_dev`
// (`src/system/unix/network.c:965-968`). UDP is the asymmetric
// counter-example: `_z_open_udp_unicast` connects to a remote
// endpoint, `_z_listen_udp_unicast` binds a local one. IVC has no
// such distinction.
z_result_t _z_f_link_listen_ivc(_z_link_t *self) { return _z_f_link_open_ivc(self); }

void _z_f_link_close_ivc(_z_link_t *self) {
    if (self->_socket._ivc._ch != NULL) {
        _z_close_ivc(self->_socket._ivc._ch);
        self->_socket._ivc._ch = NULL;
    }
}

void _z_f_link_free_ivc(_z_link_t *self) { (void)(self); }

// =============================================================================
// Send path — fragment a zenoh batch across IVC frames.
//
// Design doc §5.3: per-batch doorbell (one `_z_ivc_notify` after the
// final fragment), `total_len` + `offset` header per frame, payload
// up to `frame_size - 4`.
// =============================================================================

static size_t __z_ivc_send_batch(_z_ivc_socket_t *s, const uint8_t *ptr, size_t len) {
    if (s->_ch == NULL) {
        return SIZE_MAX;
    }
    if (len > _Z_IVC_MTU_SIZE) {
        return SIZE_MAX;
    }

    const size_t frame_size = (size_t)s->_frame_size;
    const size_t payload_max = frame_size - _Z_IVC_FRAME_HEADER_SIZE;

    uint8_t frame[_Z_IVC_FRAME_MAX];
    size_t off = 0;
    while (off < len) {
        size_t chunk = len - off;
        if (chunk > payload_max) {
            chunk = payload_max;
        }
        // total_len LE16
        frame[0] = (uint8_t)(len & 0xFF);
        frame[1] = (uint8_t)((len >> 8) & 0xFF);
        // offset LE16
        frame[2] = (uint8_t)(off & 0xFF);
        frame[3] = (uint8_t)((off >> 8) & 0xFF);
        memcpy(&frame[_Z_IVC_FRAME_HEADER_SIZE], ptr + off, chunk);

        size_t wrote = _z_send_ivc(s->_ch, frame, _Z_IVC_FRAME_HEADER_SIZE + chunk);
        if ((wrote == SIZE_MAX) || (wrote != _Z_IVC_FRAME_HEADER_SIZE + chunk)) {
            return SIZE_MAX;
        }
        off += chunk;
    }

    // Doorbell once per batch (design doc §8.1). The peer drains in a
    // loop; per-frame notify would not buy meaningful latency at the
    // SPE's µs-range cycle budget.
    _z_ivc_notify(s->_ch);
    return len;
}

size_t _z_f_link_write_ivc(const _z_link_t *self, const uint8_t *ptr, size_t len, _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(socket);
    // The reassembly state on the link is `_ivc_socket_t`'s fields; the
    // send path doesn't touch them, so the const-cast is fine.
    return __z_ivc_send_batch((_z_ivc_socket_t *)&self->_socket._ivc, ptr, len);
}

size_t _z_f_link_write_all_ivc(const _z_link_t *self, const uint8_t *ptr, size_t len) {
    return __z_ivc_send_batch((_z_ivc_socket_t *)&self->_socket._ivc, ptr, len);
}

// =============================================================================
// Receive path — reassemble IVC frames into a zenoh batch.
//
// Design doc §5.4. The reassembly buffer + counters live in
// `_z_ivc_socket_t` so state survives across `_z_f_link_read_ivc`
// calls. Returns the number of bytes of one *complete* zenoh batch on
// success, 0 if no frame is available right now (the caller will
// retry), or `SIZE_MAX` on a hard error (caller should tear the link
// down).
// =============================================================================

static void __z_ivc_reset_state(_z_ivc_socket_t *s) {
    s->_expected_total = 0;
    s->_bytes_received = 0;
}

static size_t __z_ivc_recv_batch(_z_ivc_socket_t *s, uint8_t *dst, size_t dst_len) {
    if (s->_ch == NULL) {
        return SIZE_MAX;
    }

    const size_t frame_size = (size_t)s->_frame_size;
    uint8_t frame[_Z_IVC_FRAME_MAX];

    for (;;) {
        size_t n = _z_read_ivc(s->_ch, frame, frame_size);
        if (n == 0) {
            // No frame available right now. Caller (the upper transport
            // layer) treats 0 as "retry later".
            return 0;
        }
        if (n == SIZE_MAX) {
            __z_ivc_reset_state(s);
            return SIZE_MAX;
        }
        if (n < _Z_IVC_FRAME_HEADER_SIZE) {
            // Runt frame — no header. Treat as protocol error.
            __z_ivc_reset_state(s);
            return SIZE_MAX;
        }

        uint16_t total = (uint16_t)(frame[0] | ((uint16_t)frame[1] << 8));
        uint16_t off = (uint16_t)(frame[2] | ((uint16_t)frame[3] << 8));
        size_t payload_len = n - _Z_IVC_FRAME_HEADER_SIZE;

        // Reserved keep-alive ping (§5.2). Drop and keep looping.
        if ((total == 0) && (off == 0)) {
            continue;
        }

        if (total > _Z_IVC_MTU_SIZE) {
            __z_ivc_reset_state(s);
            return SIZE_MAX;
        }

        if (s->_expected_total == 0) {
            // First fragment of a new batch.
            s->_expected_total = total;
            s->_bytes_received = 0;
        } else if (total != s->_expected_total) {
            // Mid-batch `total_len` change — corrupt stream / restart.
            __z_ivc_reset_state(s);
            return SIZE_MAX;
        }

        // SPSC FIFO ⇒ no reordering. `offset` should equal the running
        // byte count; treat any deviation as protocol violation.
        if ((off != s->_bytes_received) || ((size_t)off + payload_len > (size_t)s->_expected_total)) {
            __z_ivc_reset_state(s);
            return SIZE_MAX;
        }

        memcpy(s->_rx_buf + off, &frame[_Z_IVC_FRAME_HEADER_SIZE], payload_len);
        s->_bytes_received = (uint16_t)((size_t)s->_bytes_received + payload_len);

        if (s->_bytes_received == s->_expected_total) {
            uint16_t batch_len = s->_expected_total;
            if (dst_len < batch_len) {
                __z_ivc_reset_state(s);
                return SIZE_MAX;
            }
            memcpy(dst, s->_rx_buf, batch_len);
            __z_ivc_reset_state(s);
            return (size_t)batch_len;
        }
        // Still assembling — read the next frame. Don't return 0 on a
        // partial reassembly; the caller's "retry later" loop assumes
        // one read = one complete message.
    }
}

size_t _z_f_link_read_ivc(const _z_link_t *self, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    _ZP_UNUSED(addr);
    return __z_ivc_recv_batch((_z_ivc_socket_t *)&self->_socket._ivc, ptr, len);
}

size_t _z_f_link_read_exact_ivc(const _z_link_t *self, uint8_t *ptr, size_t len, _z_slice_t *addr,
                                _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(addr);
    _ZP_UNUSED(socket);
    // For IVC, "exact" and "best-effort" collapse: one call returns
    // exactly one complete batch (= `len` bytes) or fails. The serial
    // analogue spins on `_z_read_serial` until `len` bytes accumulate;
    // our `_z_f_link_read_ivc` already does the spinning internally
    // (the for loop in `__z_ivc_recv_batch`).
    return __z_ivc_recv_batch((_z_ivc_socket_t *)&self->_socket._ivc, ptr, len);
}

size_t _z_f_link_read_socket_ivc(const _z_sys_net_socket_t socket, uint8_t *ptr, size_t len) {
    // IVC has no `_z_sys_net_socket_t` — the link layer dispatches via
    // its own callbacks instead of going through `_z_link_get_socket`.
    // This callback is wired for symmetry with the other transports;
    // call sites that hit it on an IVC link have a logic bug.
    _ZP_UNUSED(socket);
    _ZP_UNUSED(ptr);
    _ZP_UNUSED(len);
    _Z_ERROR("IVC: _z_f_link_read_socket_ivc must not be called");
    return SIZE_MAX;
}

uint16_t _z_get_link_mtu_ivc(void) { return _Z_IVC_MTU_SIZE; }

z_result_t _z_new_link_ivc(_z_link_t *zl, _z_endpoint_t endpoint) {
    z_result_t ret = _Z_RES_OK;
    zl->_type = _Z_LINK_TYPE_IVC;
    zl->_cap._transport = Z_LINK_CAP_TRANSPORT_UNICAST;
    zl->_cap._flow = Z_LINK_CAP_FLOW_DATAGRAM;
    zl->_cap._is_reliable = false;

    zl->_mtu = _z_get_link_mtu_ivc();

    zl->_endpoint = endpoint;

    zl->_open_f = _z_f_link_open_ivc;
    zl->_listen_f = _z_f_link_listen_ivc;
    zl->_close_f = _z_f_link_close_ivc;
    zl->_free_f = _z_f_link_free_ivc;

    zl->_write_f = _z_f_link_write_ivc;
    zl->_write_all_f = _z_f_link_write_all_ivc;
    zl->_read_f = _z_f_link_read_ivc;
    zl->_read_exact_f = _z_f_link_read_exact_ivc;
    zl->_read_socket_f = _z_f_link_read_socket_ivc;

    return ret;
}

#endif
