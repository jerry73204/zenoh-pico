//
// Copyright (c) 2024 ZettaScale Technology
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

#include "zenoh-pico/system/common/platform.h"

#include <stdio.h>
#if defined(ZENOH_LINUX) || defined(ZENOH_MACOS) || defined(ZENOH_BSD) || defined(ZENOH_NUTTX)
#include <arpa/inet.h>
#endif
#if defined(ZENOH_FREERTOS_LWIP)
#include "lwip/inet.h"
/* nano-ros: lwIP defines INET6_ADDRSTRLEN only under LWIP_IPV6, but the
 * platform arm below compiles `_z_ipv6_port_to_endpoint` for every listed
 * platform INCLUDING ZENOH_FREERTOS_LWIP — so a v4-only lwIP fails to build on
 * `char ip[INET6_ADDRSTRLEN]`. Arrived with upstream 04c6b1c7 ("Connectivity
 * events and API", #1159) in 1.8.0; nano-ros ships LWIP_IPV6=0
 * (nros-board-freertos/config/lwipopts.h).
 *
 * 46 is the standard value (RFC 4291 text form + NUL). Supplying it keeps the
 * helper COMPILING rather than excising it: on a v4-only stack
 * `inet_ntop(AF_INET6, ...)` returns NULL, which the function already treats as
 * `_Z_ERR_GENERIC`. That is the honest runtime answer for a build with no IPv6,
 * and it needs no call-site surgery. */
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif
#endif

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

#include "zenoh-pico/api/olv_macros.h"
#include "zenoh-pico/utils/logging.h"

#if Z_FEATURE_MULTI_THREAD == 1

/*------------------ Thread ------------------*/
_Z_OWNED_FUNCTIONS_SYSTEM_IMPL(_z_task_t, task)

z_result_t z_task_init(z_owned_task_t *task, z_task_attr_t *attr, void *(*fun)(void *), void *arg) {
    return _z_task_init(&task->_val, attr, fun, arg);
}

z_result_t z_task_join(z_moved_task_t *task) {
    _z_task_t *ptr = &task->_this._val;
    z_result_t ret = _z_task_join(ptr);
    return ret;
}

z_result_t z_task_detach(z_moved_task_t *task) {
    _z_task_t *ptr = &task->_this._val;
    z_result_t ret = _z_task_detach(ptr);
    return ret;
}

z_result_t z_task_drop(z_moved_task_t *task) { return z_task_detach(task); }

/*------------------ Mutex ------------------*/
_Z_OWNED_FUNCTIONS_SYSTEM_IMPL(_z_mutex_t, mutex)

z_result_t z_mutex_init(z_owned_mutex_t *m) { return _z_mutex_init(&m->_val); }
z_result_t z_mutex_drop(z_moved_mutex_t *m) { return _z_mutex_drop(&m->_this._val); }

z_result_t z_mutex_lock(z_loaned_mutex_t *m) { return _z_mutex_lock(m); }
z_result_t z_mutex_try_lock(z_loaned_mutex_t *m) { return _z_mutex_try_lock(m); }
z_result_t z_mutex_unlock(z_loaned_mutex_t *m) { return _z_mutex_unlock(m); }

/*------------------ CondVar ------------------*/
_Z_OWNED_FUNCTIONS_SYSTEM_IMPL(_z_condvar_t, condvar)

z_result_t z_condvar_init(z_owned_condvar_t *cv) { return _z_condvar_init(&cv->_val); }
z_result_t z_condvar_drop(z_moved_condvar_t *cv) { return _z_condvar_drop(&cv->_this._val); }

z_result_t z_condvar_signal(z_loaned_condvar_t *cv) { return _z_condvar_signal(cv); }
z_result_t z_condvar_wait(z_loaned_condvar_t *cv, z_loaned_mutex_t *m) { return _z_condvar_wait(cv, m); }
z_result_t z_condvar_wait_until(z_loaned_condvar_t *cv, z_loaned_mutex_t *m, const z_clock_t *abstime) {
    return _z_condvar_wait_until(cv, m, abstime);
}

#endif  // Z_FEATURE_MULTI_THREAD == 1

#if defined(ZENOH_WINDOWS) || defined(ZENOH_LINUX) || defined(ZENOH_MACOS) || defined(ZENOH_BSD) || \
    defined(ZENOH_FREERTOS_LWIP) || defined(ZENOH_NUTTX)
static z_result_t _z_ipv4_port_to_endpoint(const uint8_t *address, uint16_t port, char *dst, size_t dst_len) {
    char ip[INET_ADDRSTRLEN] = {0};
    int written = -1;

    if (inet_ntop(AF_INET, address, ip, sizeof(ip)) == NULL) {
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

    if (inet_ntop(AF_INET6, address, ip, sizeof(ip)) == NULL) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    written = snprintf(dst, dst_len, "[%s]:%u", ip, (unsigned)port);
    if ((written < 0) || ((size_t)written >= dst_len)) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    return _Z_RES_OK;
}
#endif

#if defined(ZENOH_WINDOWS) || defined(ZENOH_LINUX) || defined(ZENOH_MACOS) || defined(ZENOH_BSD) || \
    defined(ZENOH_FREERTOS_LWIP) || defined(ZENOH_NUTTX)
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
#elif !defined(ZENOH_ZEPHYR)
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

/* nano-ros issue 1035 — `ZENOH_NUTTX` belongs in this list.
 *
 * NuttX compiles `src/system/unix/network.c`, which defines
 * `_z_socket_get_endpoints` at :137. Without the exclusion this fallback is
 * compiled too and the link fails:
 *
 *     multiple definition of `_z_socket_get_endpoints';
 *     network.c:(.text._z_socket_get_endpoints+0x0): first defined here
 *
 * Same class as issue 0775 in our own tree: an upstream guard that ENUMERATES
 * platforms, and NuttX is missing from the enumeration. The other guards in
 * this file (`:74`, `:105`) omit NuttX too, but they gate DEFINITIONS NuttX
 * does not reference, so they are left alone deliberately — adding NuttX there
 * would create the duplicate this one is removing. */
#if !defined(ZENOH_WINDOWS) && !defined(ZENOH_LINUX) && !defined(ZENOH_MACOS) && !defined(ZENOH_BSD) && \
    !defined(ZENOH_ZEPHYR) && !defined(ZENOH_NUTTX)
z_result_t _z_socket_get_endpoints(const _z_sys_net_socket_t *sock, char *local, size_t local_len, char *remote,
                                   size_t remote_len) {
    _ZP_UNUSED(sock);
    _ZP_UNUSED(local);
    _ZP_UNUSED(local_len);
    _ZP_UNUSED(remote);
    _ZP_UNUSED(remote_len);
    _Z_ERROR_RETURN(_Z_ERR_TRANSPORT_NOT_AVAILABLE);
}
#endif
