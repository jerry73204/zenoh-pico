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
//   nano-ros -- one framing for IVC and RPMsg. Moved out of `ivc.c`, which
//   held it alone, so a second link with the same shape reuses it instead of
//   carrying a copy. The wire format is unchanged (see `transport/framed.h`).

#include "zenoh-pico/link/framed.h"

#include <string.h>

#include "zenoh-pico/utils/pointers.h"

#if Z_FEATURE_LINK_IVC == 1 || Z_FEATURE_LINK_RPMSG == 1

z_result_t _z_framed_bind(_z_link_t *self, const _z_framed_backend_t *ops, void *ch) {
    if (ch == NULL) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        return _Z_ERR_GENERIC;
    }
    uint32_t fs = ops->frame_size(ch);
    if ((fs <= _Z_FRAMED_HEADER_SIZE) || (fs > _Z_FRAMED_FRAME_MAX)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ops->close(ch);
        return _Z_ERR_GENERIC;
    }
    self->_socket._framed._ops = ops;
    self->_socket._framed._ch = ch;
    self->_socket._framed._frame_size = (uint16_t)fs;
    self->_socket._framed._expected_total = 0;
    self->_socket._framed._bytes_received = 0;
    return _Z_RES_OK;
}

static void _z_f_link_close_framed(_z_link_t *self) {
    if (self->_socket._framed._ch != NULL) {
        self->_socket._framed._ops->close(self->_socket._framed._ch);
        self->_socket._framed._ch = NULL;
    }
}

static void _z_f_link_free_framed(_z_link_t *self) { (void)(self); }

// Send path -- fragment one zenoh batch across frames, writing each into the
// slot the channel lends, then ring the doorbell once for the batch.
static size_t __z_framed_send_batch(_z_framed_socket_t *s, const uint8_t *ptr, size_t len) {
    if (s->_ch == NULL) {
        return SIZE_MAX;
    }
    if (len > _Z_FRAMED_MTU_SIZE) {
        return SIZE_MAX;
    }
    const _z_framed_backend_t *ops = s->_ops;
    const size_t payload_max = (size_t)s->_frame_size - _Z_FRAMED_HEADER_SIZE;

    size_t off = 0;
    while (off < len) {
        size_t chunk = len - off;
        if (chunk > payload_max) {
            chunk = payload_max;
        }
        size_t cap = 0;
        uint8_t *slot = ops->tx_get(s->_ch, &cap);
        if (slot == NULL) {
            // Transmit side full: all-or-nothing, the caller retries.
            return SIZE_MAX;
        }
        if (cap < _Z_FRAMED_HEADER_SIZE + chunk) {
            ops->tx_abandon(s->_ch);
            return SIZE_MAX;
        }
        slot[0] = (uint8_t)(len & 0xFF);
        slot[1] = (uint8_t)((len >> 8) & 0xFF);
        slot[2] = (uint8_t)(off & 0xFF);
        slot[3] = (uint8_t)((off >> 8) & 0xFF);
        memcpy(&slot[_Z_FRAMED_HEADER_SIZE], ptr + off, chunk);
        if (!ops->tx_commit(s->_ch, _Z_FRAMED_HEADER_SIZE + chunk)) {
            // The rest of the batch is not sent; the receiver drops the
            // partial batch when the next one starts at offset 0.
            return SIZE_MAX;
        }
        off += chunk;
    }
    ops->notify(s->_ch);
    return len;
}

static size_t _z_f_link_write_framed(const _z_link_t *self, const uint8_t *ptr, size_t len,
                                     _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(socket);
    // The send path does not touch the reassembly state, so the cast is safe.
    return __z_framed_send_batch((_z_framed_socket_t *)&self->_socket._framed, ptr, len);
}

static size_t _z_f_link_write_all_framed(const _z_link_t *self, const uint8_t *ptr, size_t len) {
    return __z_framed_send_batch((_z_framed_socket_t *)&self->_socket._framed, ptr, len);
}

// Receive path -- reassemble frames into one zenoh batch. Returns the batch
// length, 0 when no frame is waiting (retry later), or SIZE_MAX on a protocol
// error. A partial batch keeps reading; one call returns one whole batch.
static void __z_framed_reset(_z_framed_socket_t *s) {
    s->_expected_total = 0;
    s->_bytes_received = 0;
}

static size_t __z_framed_recv_batch(_z_framed_socket_t *s, uint8_t *dst, size_t dst_len) {
    if (s->_ch == NULL) {
        return SIZE_MAX;
    }
    const _z_framed_backend_t *ops = s->_ops;
    for (;;) {
        size_t n = 0;
        const uint8_t *frame = ops->rx_get(s->_ch, &n);
        if (frame == NULL) {
            return 0;
        }
        if (n < _Z_FRAMED_HEADER_SIZE) {
            ops->rx_release(s->_ch);
            __z_framed_reset(s);
            return SIZE_MAX;
        }
        uint16_t total = (uint16_t)(frame[0] | ((uint16_t)frame[1] << 8));
        uint16_t off = (uint16_t)(frame[2] | ((uint16_t)frame[3] << 8));
        size_t payload_len = n - _Z_FRAMED_HEADER_SIZE;

        if ((total == 0) && (off == 0)) {  // reserved keep-alive
            ops->rx_release(s->_ch);
            continue;
        }
        if (total > _Z_FRAMED_MTU_SIZE) {
            ops->rx_release(s->_ch);
            __z_framed_reset(s);
            return SIZE_MAX;
        }
        if ((off == 0) && (s->_expected_total != 0)) {
            // A new batch while one is partial: the sender failed mid-batch
            // (`tx_commit`) and moved on. Drop the partial, keep this one.
            __z_framed_reset(s);
        }
        if (s->_expected_total == 0) {
            s->_expected_total = total;
            s->_bytes_received = 0;
        } else if (total != s->_expected_total) {
            ops->rx_release(s->_ch);
            __z_framed_reset(s);
            return SIZE_MAX;
        }
        if ((off != s->_bytes_received) || ((size_t)off + payload_len > (size_t)s->_expected_total)) {
            ops->rx_release(s->_ch);
            __z_framed_reset(s);
            return SIZE_MAX;
        }
        memcpy(s->_rx_buf + off, &frame[_Z_FRAMED_HEADER_SIZE], payload_len);
        s->_bytes_received = (uint16_t)((size_t)s->_bytes_received + payload_len);
        ops->rx_release(s->_ch);

        if (s->_bytes_received == s->_expected_total) {
            uint16_t batch_len = s->_expected_total;
            if (dst_len < batch_len) {
                __z_framed_reset(s);
                return SIZE_MAX;
            }
            memcpy(dst, s->_rx_buf, batch_len);
            __z_framed_reset(s);
            return (size_t)batch_len;
        }
    }
}

static size_t _z_f_link_read_framed(const _z_link_t *self, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    _ZP_UNUSED(addr);
    return __z_framed_recv_batch((_z_framed_socket_t *)&self->_socket._framed, ptr, len);
}

static size_t _z_f_link_read_exact_framed(const _z_link_t *self, uint8_t *ptr, size_t len, _z_slice_t *addr,
                                          _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(addr);
    _ZP_UNUSED(socket);
    // One call returns one whole batch, so "exact" and "best effort" coincide.
    return __z_framed_recv_batch((_z_framed_socket_t *)&self->_socket._framed, ptr, len);
}

static size_t _z_f_link_read_socket_framed(const _z_sys_net_socket_t socket, uint8_t *ptr, size_t len) {
    // A framed link has no socket; `_z_link_socket_recv_zbuf` reroutes to
    // `_read_f` for it. Reaching this is a logic error.
    _ZP_UNUSED(socket);
    _ZP_UNUSED(ptr);
    _ZP_UNUSED(len);
    _Z_ERROR("framed link: the socket read must not be called");
    return SIZE_MAX;
}

void _z_framed_link_init(_z_link_t *zl, _z_endpoint_t endpoint, uint8_t link_type) {
    zl->_type = link_type;
    zl->_cap._transport = Z_LINK_CAP_TRANSPORT_UNICAST;
    zl->_cap._flow = Z_LINK_CAP_FLOW_DATAGRAM;
    zl->_cap._is_reliable = false;
    zl->_cap._is_single_peer = true;  // nano-ros, RFC-0134 D2: one channel, two ends
    zl->_mtu = _Z_FRAMED_MTU_SIZE;
    zl->_endpoint = endpoint;
    zl->_close_f = _z_f_link_close_framed;
    zl->_free_f = _z_f_link_free_framed;
    zl->_write_f = _z_f_link_write_framed;
    zl->_write_all_f = _z_f_link_write_all_framed;
    zl->_read_f = _z_f_link_read_framed;
    zl->_read_exact_f = _z_f_link_read_exact_framed;
    zl->_read_socket_f = _z_f_link_read_socket_framed;
}

#endif
