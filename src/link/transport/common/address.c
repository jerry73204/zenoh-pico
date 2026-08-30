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

#include <stdio.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/transport/socket.h"

#if defined(ZENOH_LINUX) || defined(ZENOH_MACOS) || defined(ZENOH_BSD) || defined(ZENOH_NUTTX)
#include <arpa/inet.h>
#endif
#if defined(ZENOH_FREERTOS_LWIP)
#include "lwip/inet.h"
#endif
#if defined(ZENOH_ZEPHYR) && Z_HAS_SOCKET_LINK
#include <zephyr/net/socket.h>
#endif

#include "zenoh-pico/link/transport/socket.h"
#include "zenoh-pico/utils/logging.h"

/* nano-ros: `_z_ipv6_port_to_endpoint` below uses INET6_ADDRSTRLEN
 * unconditionally, but no include above guarantees it.
 *
 * The case this was found on is FreeRTOS + lwIP, where `lwip/inet.h` defines
 * INET_ADDRSTRLEN unconditionally and INET6_ADDRSTRLEN only under LWIP_IPV6.
 * With IPv6 off the IPv4 helper compiles and only the IPv6 one fails, which is
 * why this survived until an image needed this translation unit. The same hole
 * exists for any platform that is neither ZENOH_LINUX/MACOS/BSD (which get
 * `<arpa/inet.h>`) nor an IPv6-enabled lwIP.
 *
 * 46 is the value POSIX fixes: 45 characters for the longest IPv4-mapped form
 * (`ffff:...:255.255.255.255`) plus the terminator. A fallback rather than a
 * widened include guard, because a platform with no `<arpa/inet.h>` is exactly
 * the case that has to work.
 */
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif

#if (defined(ZENOH_WINDOWS) || defined(ZENOH_LINUX) || defined(ZENOH_MACOS) || defined(ZENOH_BSD) || \
     defined(ZENOH_FREERTOS_LWIP) || defined(ZENOH_ZEPHYR) || defined(ZENOH_NUTTX)) &&                \
    Z_HAS_SOCKET_LINK

static z_result_t _z_ipv4_port_to_endpoint(const uint8_t *address, uint16_t port, char *dst, size_t dst_len) {
    char ip[INET_ADDRSTRLEN] = {0};
    int written = -1;

#if defined(ZENOH_ZEPHYR)
    if (zsock_inet_ntop(AF_INET, address, ip, sizeof(ip)) == NULL) {
#else
    if (inet_ntop(AF_INET, address, ip, sizeof(ip)) == NULL) {
#endif
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    written = snprintf(dst, dst_len, "%s:%u", ip, (unsigned)port);
    if ((written < 0) || ((size_t)written >= dst_len)) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    return _Z_RES_OK;
}

static z_result_t _z_ipv6_port_to_endpoint(const uint8_t *address, uint16_t port, char *dst, size_t dst_len) {
    char ip[INET6_ADDRSTRLEN] = {0};
    int written = -1;

#if defined(ZENOH_ZEPHYR)
    if (zsock_inet_ntop(AF_INET6, address, ip, sizeof(ip)) == NULL) {
#else
    if (inet_ntop(AF_INET6, address, ip, sizeof(ip)) == NULL) {
#endif
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    written = snprintf(dst, dst_len, "[%s]:%u", ip, (unsigned)port);
    if ((written < 0) || ((size_t)written >= dst_len)) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    return _Z_RES_OK;
}

z_result_t _z_ip_port_to_endpoint(const uint8_t *address, size_t address_len, uint16_t port, char *dst,
                                  size_t dst_len) {
    if (address == NULL || dst == NULL || dst_len == 0) {
        _Z_ERROR_RETURN(_Z_ERR_INVALID);
    }

    if (address_len == sizeof(uint32_t)) {
        return _z_ipv4_port_to_endpoint(address, port, dst, dst_len);
    } else if (address_len == 16) {
        return _z_ipv6_port_to_endpoint(address, port, dst, dst_len);
    } else {
        _Z_ERROR_RETURN(_Z_ERR_INVALID);
    }
}

#else

z_result_t _z_ip_port_to_endpoint(const uint8_t *address, size_t address_len, uint16_t port, char *dst,
                                  size_t dst_len) {
    _ZP_UNUSED(address);
    _ZP_UNUSED(address_len);
    _ZP_UNUSED(port);
    _ZP_UNUSED(dst);
    _ZP_UNUSED(dst_len);
    _Z_ERROR_RETURN(_Z_ERR_TRANSPORT_NOT_AVAILABLE);
}

#endif
