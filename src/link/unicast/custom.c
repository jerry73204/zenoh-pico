//
// nros: link-custom — Phase 115.B
//
// `custom://` locator scheme. Drains a runtime-registered vtable
// from `nros_rmw::set_custom_transport(...)` (via the
// `nros_zpico_custom_take` Rust extern) and forwards every link
// op (open/close/write/read*) to the user's four fn pointers.
//
// Modeled on `src/link/unicast/serial.c`. The only conceptual
// difference: there's no medium-specific open code — open ⇒ drain
// the slot + call user's `open()`.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/config/custom.h"
#include "zenoh-pico/link/endpoint.h"
#include "zenoh-pico/link/manager.h"
#include "zenoh-pico/system/link/custom.h"
#include "zenoh-pico/utils/pointers.h"

#if Z_FEATURE_LINK_CUSTOM == 1

z_result_t _z_endpoint_custom_valid(_z_endpoint_t *endpoint) {
    z_result_t ret = _Z_RES_OK;
    _z_string_t scheme_str = _z_string_alias_str(CUSTOM_SCHEMA);
    if (!_z_string_equals(&endpoint->_locator._protocol, &scheme_str)) {
        _Z_ERROR_LOG(_Z_ERR_CONFIG_LOCATOR_INVALID);
        ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
    }
    return ret;
}

z_result_t _z_f_link_open_custom(_z_link_t *self) {
    /* Drain the registered vtable from nros_rmw's slot. */
    int32_t take_ret = nros_zpico_custom_take(&self->_socket._custom._ops);
    if (take_ret != 0) {
        _Z_ERROR_LOG(_Z_ERR_TRANSPORT_OPEN_FAILED);
        return _Z_ERR_TRANSPORT_OPEN_FAILED;
    }

    /* Tell the user's transport to bring the medium up. `params`
     * is NULL today; future minor-version bumps may thread
     * locator-extracted args through here. */
    int32_t open_ret = self->_socket._custom._ops.open(self->_socket._custom._ops.user_data, NULL);
    if (open_ret != 0) {
        _Z_ERROR_LOG(_Z_ERR_TRANSPORT_OPEN_FAILED);
        return _Z_ERR_TRANSPORT_OPEN_FAILED;
    }

    self->_socket._custom._opened = true;
    return _Z_RES_OK;
}

z_result_t _z_f_link_listen_custom(_z_link_t *self) {
    /* v1: client-only. Listen mode could be added later by exposing
     * `accept` callbacks on the user vtable. */
    _ZP_UNUSED(self);
    _Z_ERROR_LOG(_Z_ERR_TRANSPORT_OPEN_FAILED);
    return _Z_ERR_TRANSPORT_OPEN_FAILED;
}

void _z_f_link_close_custom(_z_link_t *self) {
    if (self->_socket._custom._opened) {
        self->_socket._custom._ops.close(self->_socket._custom._ops.user_data);
        self->_socket._custom._opened = false;
    }
}

void _z_f_link_free_custom(_z_link_t *self) { (void)(self); }

size_t _z_f_link_write_custom(const _z_link_t *self, const uint8_t *ptr, size_t len,
                              _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(socket);
    int32_t r = self->_socket._custom._ops.write(self->_socket._custom._ops.user_data, ptr, len);
    return (r == 0) ? len : 0u;
}

size_t _z_f_link_write_all_custom(const _z_link_t *self, const uint8_t *ptr, size_t len) {
    int32_t r = self->_socket._custom._ops.write(self->_socket._custom._ops.user_data, ptr, len);
    return (r == 0) ? len : 0u;
}

size_t _z_f_link_read_custom(const _z_link_t *self, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    _ZP_UNUSED(addr);
    /* zenoh-pico passes timeout-in-poll-loop; our v1 vtable read
     * uses 0 for non-blocking and lets the user impl bound
     * the wait. Future minor bump can plumb a real timeout
     * through. */
    int32_t r = self->_socket._custom._ops.read(self->_socket._custom._ops.user_data, ptr, len, 0u);
    return (r < 0) ? 0u : (size_t)r;
}

size_t _z_f_link_read_exact_custom(const _z_link_t *self, uint8_t *ptr, size_t len,
                                   _z_slice_t *addr, _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(addr);
    _ZP_UNUSED(socket);
    /* Loop until len bytes received or read returns 0/error. */
    size_t got = 0;
    while (got < len) {
        int32_t r = self->_socket._custom._ops.read(self->_socket._custom._ops.user_data,
                                                     ptr + got, len - got, 0u);
        if (r <= 0) {
            break;
        }
        got += (size_t)r;
    }
    return got;
}

size_t _z_f_link_read_socket_custom(const _z_sys_net_socket_t socket, uint8_t *ptr, size_t len) {
    /* The "read socket" code-path is only used by transports that
     * keep a separate `_z_sys_net_socket_t` per accepted client
     * (TCP listen). custom-link is client-only so this is never
     * called. */
    _ZP_UNUSED(socket);
    _ZP_UNUSED(ptr);
    _ZP_UNUSED(len);
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    return 0u;
}

uint16_t _z_get_link_mtu_custom(void) { return _Z_CUSTOM_MTU_SIZE; }

z_result_t _z_new_link_custom(_z_link_t *zl, _z_endpoint_t endpoint) {
    z_result_t ret = _Z_RES_OK;
    zl->_type = _Z_LINK_TYPE_CUSTOM;
    zl->_cap._transport = Z_LINK_CAP_TRANSPORT_UNICAST;
    zl->_cap._flow = Z_LINK_CAP_FLOW_DATAGRAM;
    zl->_cap._is_reliable = false;

    zl->_mtu = _z_get_link_mtu_custom();

    zl->_endpoint = endpoint;

    /* Pre-init the embedded socket struct; open() fills _ops. */
    memset(&zl->_socket._custom, 0, sizeof(zl->_socket._custom));

    zl->_open_f = _z_f_link_open_custom;
    zl->_listen_f = _z_f_link_listen_custom;
    zl->_close_f = _z_f_link_close_custom;
    zl->_free_f = _z_f_link_free_custom;

    zl->_write_f = _z_f_link_write_custom;
    zl->_write_all_f = _z_f_link_write_all_custom;
    zl->_read_f = _z_f_link_read_custom;
    zl->_read_exact_f = _z_f_link_read_exact_custom;
    zl->_read_socket_f = _z_f_link_read_socket_custom;

    return ret;
}

#endif /* Z_FEATURE_LINK_CUSTOM */
