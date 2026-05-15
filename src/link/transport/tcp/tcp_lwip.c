//
// Copyright (c) 2026 ZettaScale Technology
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

#include "zenoh-pico/link/transport/tcp.h"

#if defined(ZP_PLATFORM_SOCKET_LWIP) && (Z_FEATURE_LINK_TCP == 1 || Z_FEATURE_LINK_TLS == 1 || Z_FEATURE_LINK_WS == 1)

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "zenoh-pico/link/transport/lwip_socket.h"
#include "zenoh-pico/utils/logging.h"
#include "zenoh-pico/utils/pointers.h"

static char _z_lwip_numeric_addrinfo_marker;

static void _z_lwip_thread_init(void) {
#if LWIP_NETCONN_SEM_PER_THREAD
    lwip_socket_thread_init();
#endif
}

static z_result_t _z_connect_tcp_timeout(int sock, const struct sockaddr *addr, socklen_t addrlen, uint32_t tout) {
    _z_lwip_thread_init();

    int original_flags = lwip_fcntl(sock, F_GETFL, 0);
    if (original_flags == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    if (lwip_fcntl(sock, F_SETFL, original_flags | O_NONBLOCK) == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    int rc = connect(sock, addr, addrlen);
    if (rc == 0) {
        (void)lwip_fcntl(sock, F_SETFL, original_flags);
        return _Z_RES_OK;
    }

    int err = errno;
    if ((err != EINPROGRESS) && (err != EALREADY) && (err != EWOULDBLOCK)) {
        (void)lwip_fcntl(sock, F_SETFL, original_flags);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(sock, &write_fds);

    struct timeval tv;
    tv.tv_sec = tout / (uint32_t)1000;
    tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;

    rc = lwip_select(sock + 1, NULL, &write_fds, NULL, &tv);
    if (rc <= 0) {
        (void)lwip_fcntl(sock, F_SETFL, original_flags);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    socklen_t optlen = sizeof(err);
    err = 0;
    if ((lwip_getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &optlen) < 0) || (err != 0)) {
        (void)lwip_fcntl(sock, F_SETFL, original_flags);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    if (lwip_fcntl(sock, F_SETFL, original_flags) == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    return _Z_RES_OK;
}

static z_result_t _z_tcp_lwip_endpoint_init(_z_sys_net_endpoint_t *ep, const char *s_address, const char *s_port) {
    _z_lwip_thread_init();

    z_result_t ret = _Z_RES_OK;

    ip4_addr_t ip4;
    if (ip4addr_aton(s_address, &ip4) == 1) {
        char *end = NULL;
        unsigned long port = strtoul(s_port, &end, 10);
        if ((end == s_port) || (*end != '\0') || (port > 65535UL)) {
            _Z_ERROR_RETURN(_Z_ERR_GENERIC);
        }

        struct addrinfo *ai = (struct addrinfo *)calloc(1, sizeof(struct addrinfo));
        struct sockaddr_in *addr = (struct sockaddr_in *)calloc(1, sizeof(struct sockaddr_in));
        if ((ai == NULL) || (addr == NULL)) {
            free(ai);
            free(addr);
            _Z_ERROR_RETURN(_Z_ERR_SYSTEM_OUT_OF_MEMORY);
        }

        addr->sin_family = AF_INET;
        addr->sin_port = lwip_htons((uint16_t)port);
        addr->sin_addr.s_addr = ip4.addr;

        ai->ai_family = AF_INET;
        ai->ai_socktype = SOCK_STREAM;
        ai->ai_protocol = IPPROTO_TCP;
        ai->ai_addrlen = sizeof(struct sockaddr_in);
        ai->ai_addr = (struct sockaddr *)addr;
        ai->ai_canonname = &_z_lwip_numeric_addrinfo_marker;
        ep->_iptcp = ai;
        return ret;
    }

    struct addrinfo hints;
    ep->_iptcp = NULL;
    (void)memset(&hints, 0, sizeof(hints));
    hints.ai_family = PF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_TCP;

    if (getaddrinfo(s_address, s_port, &hints, &ep->_iptcp) != 0) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    } else if (ep->_iptcp != NULL && ep->_iptcp->ai_addr != NULL) {
        ep->_iptcp->ai_addr->sa_family = ep->_iptcp->ai_family;
    }

    return ret;
}

static void _z_tcp_lwip_endpoint_clear(_z_sys_net_endpoint_t *ep) {
    _z_lwip_thread_init();

    if ((ep == NULL) || (ep->_iptcp == NULL)) {
        return;
    }
    if (ep->_iptcp->ai_canonname == &_z_lwip_numeric_addrinfo_marker) {
        free(ep->_iptcp->ai_addr);
        free(ep->_iptcp);
        ep->_iptcp = NULL;
        return;
    }

    freeaddrinfo(ep->_iptcp);
    ep->_iptcp = NULL;
}

static z_result_t _z_tcp_lwip_open(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t endpoint, uint32_t tout) {
    z_result_t ret = _Z_RES_OK;

    _z_lwip_thread_init();

    _z_lwip_socket_set(sock,
                       socket(endpoint._iptcp->ai_family, endpoint._iptcp->ai_socktype, endpoint._iptcp->ai_protocol));
    if (_z_lwip_socket_get(*sock) != -1) {
        z_time_t tv;
        tv.tv_sec = tout / (uint32_t)1000;
        tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(_z_lwip_socket_get(*sock), SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        // Set send timeout to match recv timeout.
        // Without SO_SNDTIMEO, lwip_send blocks indefinitely when the TCP
        // congestion window closes (tcp_write returns ERR_MEM). On FreeRTOS
        // QEMU with -icount shift=auto, the TCP ACK round-trip through slirp
        // takes wall-clock time, which can cause the send buffer to fill before
        // ACKs arrive. With a timeout, the send fails with ERR_WOULDBLOCK
        // instead of blocking forever, allowing the caller to retry.
        if ((ret == _Z_RES_OK) &&
            (setsockopt(_z_lwip_socket_get(*sock), SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        int flags = 1;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(_z_lwip_socket_get(*sock), SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#if Z_FEATURE_TCP_NODELAY == 1
        if ((ret == _Z_RES_OK) &&
            (setsockopt(_z_lwip_socket_get(*sock), IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#endif
        struct linger ling;
        ling.l_onoff = 1;
        ling.l_linger = Z_TRANSPORT_LEASE / 1000;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(_z_lwip_socket_get(*sock), SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        for (struct addrinfo *it = endpoint._iptcp; it != NULL; it = it->ai_next) {
            if ((ret == _Z_RES_OK) &&
                (_z_connect_tcp_timeout(_z_lwip_socket_get(*sock), it->ai_addr, it->ai_addrlen, tout) != _Z_RES_OK)) {
                if (it->ai_next == NULL) {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                    break;
                }
            } else {
                break;
            }
        }

        if (ret != _Z_RES_OK) {
            close(_z_lwip_socket_get(*sock));
            _z_lwip_socket_set(sock, -1);
        }
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

static z_result_t _z_tcp_lwip_listen(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t endpoint) {
    _z_lwip_thread_init();

    z_result_t ret = _Z_RES_OK;

    _z_lwip_socket_set(sock,
                       socket(endpoint._iptcp->ai_family, endpoint._iptcp->ai_socktype, endpoint._iptcp->ai_protocol));
    if (_z_lwip_socket_get(*sock) == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    int flags = 1;
    if ((ret == _Z_RES_OK) &&
        (setsockopt(_z_lwip_socket_get(*sock), SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#if Z_FEATURE_TCP_NODELAY == 1
    if ((ret == _Z_RES_OK) &&
        (setsockopt(_z_lwip_socket_get(*sock), IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#endif
    struct linger ling;
    ling.l_onoff = 1;
    ling.l_linger = Z_TRANSPORT_LEASE / 1000;
    if ((ret == _Z_RES_OK) &&
        (setsockopt(_z_lwip_socket_get(*sock), SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    if (ret != _Z_RES_OK) {
        close(_z_lwip_socket_get(*sock));
        _z_lwip_socket_set(sock, -1);
        return ret;
    }

    for (struct addrinfo *it = endpoint._iptcp; it != NULL; it = it->ai_next) {
        if (bind(_z_lwip_socket_get(*sock), it->ai_addr, it->ai_addrlen) < 0) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
            break;
        }
        if (listen(_z_lwip_socket_get(*sock), Z_LISTEN_MAX_CONNECTION_NB) < 0) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
            break;
        }
    }

    if (ret != _Z_RES_OK) {
        close(_z_lwip_socket_get(*sock));
        _z_lwip_socket_set(sock, -1);
    }
    return ret;
}

static z_result_t _z_tcp_lwip_accept(const _z_sys_net_socket_t *sock_in, _z_sys_net_socket_t *sock_out) {
    _z_lwip_thread_init();

    struct sockaddr naddr;
    socklen_t nlen = sizeof(naddr);
    _z_lwip_socket_set(sock_out, -1);
    int con_socket = lwip_accept(_z_lwip_socket_get(*sock_in), &naddr, &nlen);
    if (con_socket < 0) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    z_time_t tv;
    tv.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / (uint32_t)1000;
    tv.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % (uint32_t)1000) * (uint32_t)1000;
    if (setsockopt(con_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0) {
        lwip_close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    int flags = 1;
    if (setsockopt(con_socket, SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0) {
        lwip_close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
#if Z_FEATURE_TCP_NODELAY == 1
    if (setsockopt(con_socket, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0) {
        lwip_close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
#endif
    struct linger ling;
    ling.l_onoff = 1;
    ling.l_linger = Z_TRANSPORT_LEASE / 1000;
    if (setsockopt(con_socket, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0) {
        lwip_close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    _z_lwip_socket_set(sock_out, con_socket);
    return _Z_RES_OK;
}

static void _z_tcp_lwip_close(_z_sys_net_socket_t *sock) {
    _z_lwip_thread_init();

    if (_z_lwip_socket_get(*sock) >= 0) {
        shutdown(_z_lwip_socket_get(*sock), SHUT_RDWR);
        close(_z_lwip_socket_get(*sock));
        _z_lwip_socket_set(sock, -1);
    }
}

static size_t _z_tcp_lwip_read(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    _z_lwip_thread_init();

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(_z_lwip_socket_get(sock), &read_fds);

    struct timeval tv;
    tv.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / (uint32_t)1000;
    tv.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % (uint32_t)1000) * (uint32_t)1000;

    int ready = lwip_select(_z_lwip_socket_get(sock) + 1, &read_fds, NULL, NULL, &tv);
    if (ready <= 0) {
        return SIZE_MAX;
    }

    ssize_t rb = recv(_z_lwip_socket_get(sock), ptr, len, 0);
    if (rb < (ssize_t)0) {
        return SIZE_MAX;
    }

    return (size_t)rb;
}

static size_t _z_tcp_lwip_read_exact(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    size_t n = 0;
    uint8_t *pos = &ptr[0];

    do {
        size_t rb = _z_tcp_lwip_read(sock, pos, len - n);
        if ((rb == SIZE_MAX) || (rb == 0)) {
            n = rb;
            break;
        }

        n = n + rb;
        pos = _z_ptr_u8_offset(pos, rb);
    } while (n != len);

    return n;
}

static size_t _z_tcp_lwip_write(_z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
    _z_lwip_thread_init();

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(_z_lwip_socket_get(sock), &write_fds);

    struct timeval tv;
    tv.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / (uint32_t)1000;
    tv.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % (uint32_t)1000) * (uint32_t)1000;

    int ready = lwip_select(_z_lwip_socket_get(sock) + 1, NULL, &write_fds, NULL, &tv);
    if (ready <= 0) {
        return SIZE_MAX;
    }

    return (size_t)send(_z_lwip_socket_get(sock), ptr, len, 0);
}

z_result_t _z_tcp_endpoint_init(_z_sys_net_endpoint_t *ep, const char *address, const char *port) {
    return _z_tcp_lwip_endpoint_init(ep, address, port);
}

void _z_tcp_endpoint_clear(_z_sys_net_endpoint_t *ep) { _z_tcp_lwip_endpoint_clear(ep); }

z_result_t _z_tcp_open(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t endpoint, uint32_t tout) {
    return _z_tcp_lwip_open(sock, endpoint, tout);
}

z_result_t _z_tcp_listen(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t endpoint) {
    return _z_tcp_lwip_listen(sock, endpoint);
}

z_result_t _z_tcp_accept(const _z_sys_net_socket_t *sock_in, _z_sys_net_socket_t *sock_out) {
    return _z_tcp_lwip_accept(sock_in, sock_out);
}

void _z_tcp_close(_z_sys_net_socket_t *sock) { _z_tcp_lwip_close(sock); }

size_t _z_tcp_read(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) { return _z_tcp_lwip_read(sock, ptr, len); }

size_t _z_tcp_read_exact(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    return _z_tcp_lwip_read_exact(sock, ptr, len);
}

size_t _z_tcp_write(_z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
    return _z_tcp_lwip_write(sock, ptr, len);
}

#endif /* defined(ZP_PLATFORM_SOCKET_LWIP) */
