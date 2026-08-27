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

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#if !defined(ZENOH_NUTTX)
#include <ifaddrs.h>
#endif
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "zenoh-pico/collections/string.h"
#include "zenoh-pico/config.h"

// SocketCAN headers must follow zenoh-pico/config.h: the guard needs
// Z_FEATURE_LINK_CAN to already be defined.
#if Z_FEATURE_LINK_CAN == 1
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#endif
// Same ordering rule as SocketCAN above: needs Z_FEATURE_LINK_ISOTP defined.
// Z_FEATURE_LINK_ISOTP_VENDORED selects src/system/unix/isotp_vendored.c
// instead of everything below -- see the comment at the top of that file for
// why `unix` implements this link twice.
#if Z_FEATURE_LINK_ISOTP == 1 && Z_FEATURE_LINK_ISOTP_VENDORED == 0
#include <linux/can.h>
#include <linux/can/isotp.h>
#include <sys/socket.h>
#endif
#if Z_FEATURE_LINK_SERIAL == 1
// `_z_open_serial_from_dev` below calls `_z_connect_serial`, declared here.
// Without this include the call is an IMPLICIT DECLARATION: a warning on
// gcc <= 13, a hard error on gcc >= 14 and clang >= 15, which made the unix
// build fail outright on any host with a current compiler.
#include "zenoh-pico/system/common/serial.h"
#endif
#if Z_FEATURE_LINK_TLS == 1
#include "zenoh-pico/system/link/tls.h"
#endif
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/utils/logging.h"
#include "zenoh-pico/utils/pointers.h"

z_result_t _z_socket_set_non_blocking(const _z_sys_net_socket_t *sock) {
    int flags = fcntl(sock->_fd, F_GETFL, 0);
    if (flags == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    if (fcntl(sock->_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    return _Z_RES_OK;
}

z_result_t _z_socket_accept(const _z_sys_net_socket_t *sock_in, _z_sys_net_socket_t *sock_out) {
    struct sockaddr naddr;
    unsigned int nlen = sizeof(naddr);
    sock_out->_fd = -1;
    int con_socket = accept(sock_in->_fd, &naddr, &nlen);
    if (con_socket < 0) {
        if (errno == EBADF) {
            _Z_ERROR_RETURN(_Z_ERR_INVALID);
        } else {
            _Z_ERROR_RETURN(_Z_ERR_GENERIC);
        }
    }
    // Set socket options
    z_time_t tv;
    tv.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / (uint32_t)1000;
    tv.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % (uint32_t)1000) * (uint32_t)1000;
    if (setsockopt(con_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0) {
        close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    int flags = 1;
    if (setsockopt(con_socket, SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0) {
        close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
#if Z_FEATURE_TCP_NODELAY == 1
    if (setsockopt(con_socket, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0) {
        close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
#endif
    struct linger ling;
    ling.l_onoff = 1;
    ling.l_linger = Z_TRANSPORT_LEASE / 1000;
    if (setsockopt(con_socket, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0) {
        close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    // Note socket
    sock_out->_fd = con_socket;
    return _Z_RES_OK;
}

static z_result_t _z_sockaddr_to_endpoint(const struct sockaddr *addr, char *dst, size_t dst_len) {
    if (addr->sa_family == AF_INET) {
        const struct sockaddr_in *addr4 = (const struct sockaddr_in *)addr;
        const uint8_t *bytes = (const uint8_t *)&addr4->sin_addr;
        return _z_ip_port_to_endpoint(bytes, sizeof(addr4->sin_addr), ntohs(addr4->sin_port), dst, dst_len);
    } else if (addr->sa_family == AF_INET6) {
        const struct sockaddr_in6 *addr6 = (const struct sockaddr_in6 *)addr;
        const uint8_t *bytes = (const uint8_t *)&addr6->sin6_addr;
        return _z_ip_port_to_endpoint(bytes, sizeof(addr6->sin6_addr), ntohs(addr6->sin6_port), dst, dst_len);
    } else {
        _Z_ERROR_RETURN(_Z_ERR_INVALID);
    }
}

z_result_t _z_socket_get_endpoints(const _z_sys_net_socket_t *sock, char *local, size_t local_len, char *remote,
                                   size_t remote_len) {
    struct sockaddr_storage local_addr = {0};
    struct sockaddr_storage remote_addr = {0};
    socklen_t local_addr_len = sizeof(local_addr);
    socklen_t remote_addr_len = sizeof(remote_addr);

    if (sock == NULL || local == NULL || remote == NULL || local_len == 0 || remote_len == 0) {
        _Z_ERROR_RETURN(_Z_ERR_INVALID);
    }
    if (sock->_fd < 0) {
        _Z_ERROR_RETURN(_Z_ERR_INVALID);
    }
    if (getsockname(sock->_fd, (struct sockaddr *)&local_addr, &local_addr_len) != 0) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    if (getpeername(sock->_fd, (struct sockaddr *)&remote_addr, &remote_addr_len) != 0) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    _Z_RETURN_IF_ERR(_z_sockaddr_to_endpoint((const struct sockaddr *)&local_addr, local, local_len));
    _Z_RETURN_IF_ERR(_z_sockaddr_to_endpoint((const struct sockaddr *)&remote_addr, remote, remote_len));
    return _Z_RES_OK;
}

void _z_socket_close(_z_sys_net_socket_t *sock) {
#if Z_FEATURE_LINK_TLS == 1
    if (sock->_tls_sock != NULL) {
        _z_tls_socket_t *tls_sock = (_z_tls_socket_t *)sock->_tls_sock;
        bool peer_socket = tls_sock->_is_peer_socket;
        _z_close_tls(tls_sock);
        if (peer_socket) {
            z_free(tls_sock);
        }
        sock->_tls_sock = NULL;
        return;
    }
#endif
    if (sock->_fd >= 0) {
        shutdown(sock->_fd, SHUT_RDWR);
        close(sock->_fd);
        sock->_fd = -1;
    }
}

#if Z_FEATURE_MULTI_THREAD == 1
z_result_t _z_socket_wait_event(void *v_peers, _z_mutex_rec_t *mutex) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    // Create select mask
    _z_transport_peer_unicast_slist_t **peers = (_z_transport_peer_unicast_slist_t **)v_peers;
    _z_mutex_rec_lock(mutex);
    _z_transport_peer_unicast_slist_t *curr = *peers;
    int max_fd = 0;
    while (curr != NULL) {
        _z_transport_peer_unicast_t *peer = _z_transport_peer_unicast_slist_value(curr);
        FD_SET(peer->_socket._fd, &read_fds);
        if (peer->_socket._fd > max_fd) {
            max_fd = peer->_socket._fd;
        }
        curr = _z_transport_peer_unicast_slist_next(curr);
    }
    _z_mutex_rec_unlock(mutex);
    // Wait for events
    struct timeval timeout;
    timeout.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / 1000;
    timeout.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % 1000) * 1000;
    int result = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
    if (result < 0) {
        _Z_DEBUG("Errno: %d\n", errno);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    // Mark sockets that are pending
    _z_mutex_rec_lock(mutex);
    curr = *peers;
    while (curr != NULL) {
        _z_transport_peer_unicast_t *peer = _z_transport_peer_unicast_slist_value(curr);
        if (FD_ISSET(peer->_socket._fd, &read_fds)) {
            peer->_pending = true;
        }
        curr = _z_transport_peer_unicast_slist_next(curr);
    }
    _z_mutex_rec_unlock(mutex);
    return _Z_RES_OK;
}
#else
z_result_t _z_socket_wait_event(void *peers, _z_mutex_rec_t *mutex) {
    _ZP_UNUSED(peers);
    _ZP_UNUSED(mutex);
    return _Z_RES_OK;
}
#endif

#if Z_FEATURE_LINK_TCP == 1
/*------------------ TCP sockets ------------------*/
z_result_t _z_create_endpoint_tcp(_z_sys_net_endpoint_t *ep, const char *s_address, const char *s_port) {
    z_result_t ret = _Z_RES_OK;

    struct addrinfo hints;
    (void)memset(&hints, 0, sizeof(hints));
    hints.ai_family = PF_UNSPEC;  // Allow IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_TCP;

    if (getaddrinfo(s_address, s_port, &hints, &ep->_iptcp) < 0) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

void _z_free_endpoint_tcp(_z_sys_net_endpoint_t *ep) { freeaddrinfo(ep->_iptcp); }

/*------------------ TCP sockets ------------------*/
z_result_t _z_open_tcp(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, uint32_t tout) {
    z_result_t ret = _Z_RES_OK;

    sock->_fd = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
    if (sock->_fd != -1) {
        z_time_t tv;
        tv.tv_sec = (time_t)(tout / (uint32_t)1000);
        tv.tv_usec = (suseconds_t)((tout % (uint32_t)1000) * (uint32_t)1000);
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        int flags = 1;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#if Z_FEATURE_TCP_NODELAY == 1 && !defined(ZENOH_NUTTX)
        // NuttX: TCP_NODELAY=16 but cross-compilation picks up host's TCP_NODELAY=1
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_fd, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#endif
#ifndef ZENOH_NUTTX
        // NuttX: SO_LINGER requires CONFIG_NET_SOLINGER which is typically not enabled
        struct linger ling;
        ling.l_onoff = 1;
        ling.l_linger = Z_TRANSPORT_LEASE / 1000;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_fd, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#endif

#if defined(ZENOH_MACOS) || defined(ZENOH_BSD)
        int nosigpipe_val = 1;
        setsockopt(sock->_fd, SOL_SOCKET, SO_NOSIGPIPE, (void *)&nosigpipe_val, sizeof(int));
#endif
        struct addrinfo *it = NULL;
        for (it = rep._iptcp; it != NULL; it = it->ai_next) {
            if ((ret == _Z_RES_OK) && (connect(sock->_fd, it->ai_addr, it->ai_addrlen) < 0)) {
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
            close(sock->_fd);
            sock->_fd = -1;
        }
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
    return ret;
}

z_result_t _z_listen_tcp(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t lep) {
    z_result_t ret = _Z_RES_OK;
    // Open socket
    sock->_fd = socket(lep._iptcp->ai_family, lep._iptcp->ai_socktype, lep._iptcp->ai_protocol);
    if (sock->_fd == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    // Set options
    int value = true;
    if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
    int flags = 1;
    if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#if Z_FEATURE_TCP_NODELAY == 1
    if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#endif
    struct linger ling;
    ling.l_onoff = 1;
    ling.l_linger = Z_TRANSPORT_LEASE / 1000;
    if ((ret == _Z_RES_OK) &&
        (setsockopt(sock->_fd, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#if defined(ZENOH_MACOS) || defined(ZENOH_BSD)
    int nosigpipe_val = 1;
    setsockopt(sock->_fd, SOL_SOCKET, SO_NOSIGPIPE, (void *)&nosigpipe_val, sizeof(int));
#endif
    if (ret != _Z_RES_OK) {
        close(sock->_fd);
        return ret;
    }

    struct addrinfo *it = NULL;
    int addr_count = 0;
    for (it = lep._iptcp; it != NULL; it = it->ai_next) {
        addr_count++;
        char addr_str[INET6_ADDRSTRLEN];
        const char *family_str = (it->ai_family == AF_INET) ? "IPv4" : (it->ai_family == AF_INET6) ? "IPv6" : "Unknown";

        // Extract address string for logging
        if (it->ai_family == AF_INET) {
            inet_ntop(AF_INET, &((struct sockaddr_in *)it->ai_addr)->sin_addr, addr_str, INET_ADDRSTRLEN);
        } else if (it->ai_family == AF_INET6) {
            inet_ntop(AF_INET6, &((struct sockaddr_in6 *)it->ai_addr)->sin6_addr, addr_str, INET6_ADDRSTRLEN);
        } else {
            snprintf(addr_str, sizeof(addr_str), "%s", "unknown");
        }

        _Z_DEBUG("Trying address %d: %s (%s), family=%d", addr_count, addr_str, family_str, it->ai_family);
        if (bind(sock->_fd, it->ai_addr, it->ai_addrlen) < 0) {
            _Z_DEBUG("bind() failed for address %s: %s", addr_str, strerror(errno));
            if (it->ai_next == NULL) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
                break;
            }
            continue;  // Try next address
        }
        _Z_DEBUG("bind() successful for address %s", addr_str);

        if (listen(sock->_fd, Z_LISTEN_MAX_CONNECTION_NB) < 0) {
            _Z_DEBUG("listen() failed for address %s: %s", addr_str, strerror(errno));
            if (it->ai_next == NULL) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
                break;
            }
            continue;  // Try next address
        }
        _Z_DEBUG("listen() successful for address %s", addr_str);
        break;
    }
    if (ret != _Z_RES_OK) {
        close(sock->_fd);
        sock->_fd = -1;
    }
    return ret;
}

void _z_close_tcp(_z_sys_net_socket_t *sock) {
    if (sock->_fd >= 0) {
        shutdown(sock->_fd, SHUT_RDWR);
        close(sock->_fd);
        sock->_fd = -1;
    }
}

size_t _z_read_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    ssize_t rb = recv(sock._fd, ptr, len, 0);
    if (rb < (ssize_t)0) {
        // Errno can be 11(EAGAIN) because of SO_RCVTIMEO
        if (errno != EAGAIN) {
            _Z_DEBUG("Errno: %d\n", errno);
        }
        return SIZE_MAX;
    }

    return (size_t)rb;
}

size_t _z_read_exact_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    size_t n = 0;
    uint8_t *pos = &ptr[0];

    do {
        size_t rb = _z_read_tcp(sock, pos, len - n);
        if ((rb == SIZE_MAX) || (rb == 0)) {
            n = rb;
            break;
        }

        n = n + rb;
        pos = _z_ptr_u8_offset(pos, (ptrdiff_t)n);
    } while (n != len);

    return n;
}

size_t _z_send_tcp(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
#if defined(ZENOH_LINUX) || defined(ZENOH_NUTTX)
    return (size_t)send(sock._fd, ptr, len, MSG_NOSIGNAL);
#else
    return send(sock._fd, ptr, len, 0);
#endif
}
#endif

#if Z_FEATURE_LINK_UDP_UNICAST == 1 || Z_FEATURE_LINK_UDP_MULTICAST == 1
/*------------------ UDP sockets ------------------*/
z_result_t _z_create_endpoint_udp(_z_sys_net_endpoint_t *ep, const char *s_address, const char *s_port) {
    z_result_t ret = _Z_RES_OK;

    struct addrinfo hints;
    (void)memset(&hints, 0, sizeof(hints));
    hints.ai_family = PF_UNSPEC;  // Allow IPv4 or IPv6
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_UDP;

    if (getaddrinfo(s_address, s_port, &hints, &ep->_iptcp) < 0) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

void _z_free_endpoint_udp(_z_sys_net_endpoint_t *ep) { freeaddrinfo(ep->_iptcp); }
#endif

#if Z_FEATURE_LINK_UDP_UNICAST == 1
z_result_t _z_open_udp_unicast(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, uint32_t tout) {
    z_result_t ret = _Z_RES_OK;

    sock->_fd = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
    if (sock->_fd != -1) {
        z_time_t tv;
        tv.tv_sec = (time_t)(tout / (uint32_t)1000);
        tv.tv_usec = (suseconds_t)((tout % (uint32_t)1000) * (uint32_t)1000);
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        if (ret != _Z_RES_OK) {
            close(sock->_fd);
            sock->_fd = -1;
        }
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

z_result_t _z_listen_udp_unicast(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t lep, uint32_t tout) {
    (void)sock;
    (void)lep;
    (void)tout;
    z_result_t ret = _Z_RES_OK;

    // @TODO: To be implemented
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    ret = _Z_ERR_GENERIC;

    return ret;
}

void _z_close_udp_unicast(_z_sys_net_socket_t *sock) {
    if (sock->_fd >= 0) {
        close(sock->_fd);
        sock->_fd = -1;
    }
}

size_t _z_read_udp_unicast(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    struct sockaddr_storage raddr;
    unsigned int addrlen = sizeof(struct sockaddr_storage);

    ssize_t rb = recvfrom(sock._fd, ptr, len, 0, (struct sockaddr *)&raddr, &addrlen);
    if (rb < (ssize_t)0) {
        return SIZE_MAX;
    }
    return (size_t)rb;
}

size_t _z_read_exact_udp_unicast(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    size_t n = 0;
    uint8_t *pos = &ptr[0];

    do {
        size_t rb = _z_read_udp_unicast(sock, pos, len - n);
        if ((rb == SIZE_MAX) || (rb == 0)) {
            n = rb;
            break;
        }

        n = n + rb;
        pos = _z_ptr_u8_offset(pos, (ptrdiff_t)n);
    } while (n != len);

    return n;
}

size_t _z_send_udp_unicast(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len,
                           const _z_sys_net_endpoint_t rep) {
    return (size_t)sendto(sock._fd, ptr, len, 0, rep._iptcp->ai_addr, rep._iptcp->ai_addrlen);
}
#endif

#if Z_FEATURE_LINK_UDP_MULTICAST == 1
unsigned int __get_ip_from_iface(const char *iface, int sa_family, struct sockaddr **lsockaddr) {
    unsigned int addrlen = 0U;

#if defined(ZENOH_NUTTX)
    // NuttX may not provide getifaddrs(). For multicast, bind to INADDR_ANY.
    _ZP_UNUSED(iface);
    if (sa_family == AF_INET) {
        *lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in));
        if (*lsockaddr != NULL) {
            (void)memset(*lsockaddr, 0, sizeof(struct sockaddr_in));
            ((struct sockaddr_in *)*lsockaddr)->sin_family = AF_INET;
            ((struct sockaddr_in *)*lsockaddr)->sin_addr.s_addr = htonl(INADDR_ANY);
            addrlen = sizeof(struct sockaddr_in);
        }
    } else if (sa_family == AF_INET6) {
        *lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in6));
        if (*lsockaddr != NULL) {
            (void)memset(*lsockaddr, 0, sizeof(struct sockaddr_in6));
            ((struct sockaddr_in6 *)*lsockaddr)->sin6_family = AF_INET6;
            ((struct sockaddr_in6 *)*lsockaddr)->sin6_addr = in6addr_any;
            addrlen = sizeof(struct sockaddr_in6);
        }
    }
#else
    struct ifaddrs *l_ifaddr = NULL;
    if (getifaddrs(&l_ifaddr) != -1) {
        struct ifaddrs *tmp = NULL;
        for (tmp = l_ifaddr; tmp != NULL; tmp = tmp->ifa_next) {
            if (_z_str_eq(tmp->ifa_name, iface) == true) {
                if (tmp->ifa_addr->sa_family == sa_family) {
                    if (tmp->ifa_addr->sa_family == AF_INET) {
                        *lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in));
                        if (lsockaddr != NULL) {
                            (void)memset(*lsockaddr, 0, sizeof(struct sockaddr_in));
                            (void)memcpy(*lsockaddr, tmp->ifa_addr, sizeof(struct sockaddr_in));
                            addrlen = sizeof(struct sockaddr_in);
                        }
                    } else if (tmp->ifa_addr->sa_family == AF_INET6) {
                        *lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in6));
                        if (lsockaddr != NULL) {
                            (void)memset(*lsockaddr, 0, sizeof(struct sockaddr_in6));
                            (void)memcpy(*lsockaddr, tmp->ifa_addr, sizeof(struct sockaddr_in6));
                            addrlen = sizeof(struct sockaddr_in6);
                        }
                    } else {
                        continue;
                    }

                    break;
                }
            }
        }
        freeifaddrs(l_ifaddr);
    }
#endif

    return addrlen;
}

z_result_t _z_open_udp_multicast(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, _z_sys_net_endpoint_t *lep,
                                 uint32_t tout, const char *iface) {
    z_result_t ret = _Z_RES_OK;

    struct sockaddr *lsockaddr = NULL;
    unsigned int addrlen = __get_ip_from_iface(iface, rep._iptcp->ai_family, &lsockaddr);
    if (addrlen != 0U) {
        sock->_fd = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
        if (sock->_fd != -1) {
            z_time_t tv;
            tv.tv_sec = (time_t)(tout / (uint32_t)1000);
            tv.tv_usec = (suseconds_t)((tout % (uint32_t)1000) * (uint32_t)1000);
            if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

            if ((ret == _Z_RES_OK) && (bind(sock->_fd, lsockaddr, addrlen) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

            // Get the randomly assigned port used to discard loopback messages
            if ((ret == _Z_RES_OK) && (getsockname(sock->_fd, lsockaddr, &addrlen) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

#ifndef UNIX_NO_MULTICAST_IF
            if (lsockaddr->sa_family == AF_INET) {
                if ((ret == _Z_RES_OK) &&
                    (setsockopt(sock->_fd, IPPROTO_IP, IP_MULTICAST_IF, &((struct sockaddr_in *)lsockaddr)->sin_addr,
                                sizeof(struct in_addr)) < 0)) {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            } else if (lsockaddr->sa_family == AF_INET6) {
                int ifindex = (int)if_nametoindex(iface);
                if ((ret == _Z_RES_OK) &&
                    (setsockopt(sock->_fd, IPPROTO_IPV6, IPV6_MULTICAST_IF, &ifindex, sizeof(ifindex)) < 0)) {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            } else {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }
#endif

            // Create lep endpoint
            if (ret == _Z_RES_OK) {
                struct addrinfo *laddr = (struct addrinfo *)z_malloc(sizeof(struct addrinfo));
                if (laddr != NULL) {
                    laddr->ai_flags = 0;
                    laddr->ai_family = rep._iptcp->ai_family;
                    laddr->ai_socktype = rep._iptcp->ai_socktype;
                    laddr->ai_protocol = rep._iptcp->ai_protocol;
                    laddr->ai_addrlen = addrlen;
                    laddr->ai_addr = lsockaddr;
                    laddr->ai_canonname = NULL;
                    laddr->ai_next = NULL;
                    lep->_iptcp = laddr;
                } else {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            }

            if (ret != _Z_RES_OK) {
                close(sock->_fd);
                sock->_fd = -1;
            }
        } else {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        if (ret != _Z_RES_OK) {
            z_free(lsockaddr);
        }
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

z_result_t _z_listen_udp_multicast(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, uint32_t tout,
                                   const char *iface, const char *join) {
    z_result_t ret = _Z_RES_OK;

    struct sockaddr *lsockaddr = NULL;
    unsigned int addrlen = __get_ip_from_iface(iface, rep._iptcp->ai_family, &lsockaddr);
    if (addrlen != 0U) {
        sock->_fd = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
        if (sock->_fd != -1) {
            z_time_t tv;
            tv.tv_sec = (time_t)(tout / (uint32_t)1000);
            tv.tv_usec = (suseconds_t)((tout % (uint32_t)1000) * (uint32_t)1000);
            if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

            int value = true;
            if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value)) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }
#ifdef SO_REUSEPORT
            if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_REUSEPORT, &value, sizeof(value)) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }
#endif

            if (rep._iptcp->ai_family == AF_INET) {
                struct sockaddr_in address;
                (void)memset(&address, 0, sizeof(address));
                address.sin_family = (sa_family_t)rep._iptcp->ai_family;
                address.sin_port = ((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_port;
                inet_pton(address.sin_family, "0.0.0.0", &address.sin_addr);
                if ((ret == _Z_RES_OK) && (bind(sock->_fd, (struct sockaddr *)&address, sizeof(address)) < 0)) {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            } else if (rep._iptcp->ai_family == AF_INET6) {
                struct sockaddr_in6 address;
                (void)memset(&address, 0, sizeof(address));
                address.sin6_family = (sa_family_t)rep._iptcp->ai_family;
                address.sin6_port = ((struct sockaddr_in6 *)rep._iptcp->ai_addr)->sin6_port;
                inet_pton(address.sin6_family, "::", &address.sin6_addr);
                if ((ret == _Z_RES_OK) && (bind(sock->_fd, (struct sockaddr *)&address, sizeof(address)) < 0)) {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            } else {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

            // Join the multicast group
            if (rep._iptcp->ai_family == AF_INET) {
                struct ip_mreq mreq;
                (void)memset(&mreq, 0, sizeof(mreq));
                mreq.imr_multiaddr.s_addr = ((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_addr.s_addr;
                mreq.imr_interface.s_addr = ((struct sockaddr_in *)lsockaddr)->sin_addr.s_addr;
                if ((ret == _Z_RES_OK) &&
                    (setsockopt(sock->_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)) {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            } else if (rep._iptcp->ai_family == AF_INET6) {
                struct ipv6_mreq mreq;
                (void)memset(&mreq, 0, sizeof(mreq));
                (void)memcpy(&mreq.ipv6mr_multiaddr, &((struct sockaddr_in6 *)rep._iptcp->ai_addr)->sin6_addr,
                             sizeof(struct in6_addr));
                mreq.ipv6mr_interface = if_nametoindex(iface);
                if ((ret == _Z_RES_OK) &&
                    (setsockopt(sock->_fd, IPPROTO_IPV6, IPV6_JOIN_GROUP, &mreq, sizeof(mreq)) < 0)) {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            } else {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }
            // Join any additional multicast group
            if (join != NULL) {
                char *joins = _z_str_clone(join);
                for (char *ip = strsep(&joins, "|"); ip != NULL; ip = strsep(&joins, "|")) {
                    if (rep._iptcp->ai_family == AF_INET) {
                        struct ip_mreq mreq;
                        (void)memset(&mreq, 0, sizeof(mreq));
                        inet_pton(rep._iptcp->ai_family, ip, &mreq.imr_multiaddr);
                        mreq.imr_interface.s_addr = ((struct sockaddr_in *)lsockaddr)->sin_addr.s_addr;
                        if ((ret == _Z_RES_OK) &&
                            (setsockopt(sock->_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)) {
                            _Z_ERROR_LOG(_Z_ERR_GENERIC);
                            ret = _Z_ERR_GENERIC;
                        }
                    } else if (rep._iptcp->ai_family == AF_INET6) {
                        struct ipv6_mreq mreq;
                        (void)memset(&mreq, 0, sizeof(mreq));
                        inet_pton(rep._iptcp->ai_family, ip, &mreq.ipv6mr_multiaddr);
                        mreq.ipv6mr_interface = if_nametoindex(iface);
                        if ((ret == _Z_RES_OK) &&
                            (setsockopt(sock->_fd, IPPROTO_IPV6, IPV6_JOIN_GROUP, &mreq, sizeof(mreq)) < 0)) {
                            _Z_ERROR_LOG(_Z_ERR_GENERIC);
                            ret = _Z_ERR_GENERIC;
                        }
                    }
                }
                z_free(joins);
            }

            if (ret != _Z_RES_OK) {
                close(sock->_fd);
                sock->_fd = -1;
            }
        } else {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        z_free(lsockaddr);
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

void _z_close_udp_multicast(_z_sys_net_socket_t *sockrecv, _z_sys_net_socket_t *socksend,
                            const _z_sys_net_endpoint_t rep, const _z_sys_net_endpoint_t lep) {
    if (sockrecv->_fd >= 0) {
        if (rep._iptcp->ai_family == AF_INET) {
            struct ip_mreq mreq;
            (void)memset(&mreq, 0, sizeof(mreq));
            mreq.imr_multiaddr.s_addr = ((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_addr.s_addr;
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
            setsockopt(sockrecv->_fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
        } else if (rep._iptcp->ai_family == AF_INET6) {
            struct ipv6_mreq mreq;
            (void)memset(&mreq, 0, sizeof(mreq));
            (void)memcpy(&mreq.ipv6mr_multiaddr, &((struct sockaddr_in6 *)rep._iptcp->ai_addr)->sin6_addr,
                         sizeof(struct in6_addr));
            // mreq.ipv6mr_interface = ifindex;
            setsockopt(sockrecv->_fd, IPPROTO_IPV6, IPV6_LEAVE_GROUP, &mreq, sizeof(mreq));
        } else {
            // Do nothing. It must never not enter here.
            // Required to be compliant with MISRA 15.7 rule
        }
    }
#if defined(ZENOH_LINUX) || defined(ZENOH_NUTTX)
    if (lep._iptcp != NULL) {
        z_free(lep._iptcp->ai_addr);
    }
#else
    _ZP_UNUSED(lep);
#endif
    if (sockrecv->_fd >= 0) {
        close(sockrecv->_fd);
        sockrecv->_fd = -1;
    }
    if (socksend->_fd >= 0) {
        close(socksend->_fd);
        socksend->_fd = -1;
    }
}

size_t _z_read_udp_multicast(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len, const _z_sys_net_endpoint_t lep,
                             _z_slice_t *addr) {
    struct sockaddr_storage raddr;
    unsigned int replen = sizeof(struct sockaddr_storage);

    ssize_t rb = 0;
    do {
        rb = recvfrom(sock._fd, ptr, len, 0, (struct sockaddr *)&raddr, &replen);
        if (rb < (ssize_t)0) {
            return SIZE_MAX;
        }

        if (lep._iptcp->ai_family == AF_INET) {
            struct sockaddr_in *a = ((struct sockaddr_in *)lep._iptcp->ai_addr);
            struct sockaddr_in *b = ((struct sockaddr_in *)&raddr);
            if (!((a->sin_port == b->sin_port) && (a->sin_addr.s_addr == b->sin_addr.s_addr))) {
                // If addr is not NULL, it means that the rep was requested by the upper-layers
                if (addr != NULL) {
                    assert(addr->len >= sizeof(in_addr_t) + sizeof(in_port_t));
                    addr->len = sizeof(in_addr_t) + sizeof(in_port_t);
                    (void)memcpy((uint8_t *)addr->start, &b->sin_addr.s_addr, sizeof(in_addr_t));
                    (void)memcpy((uint8_t *)(addr->start + sizeof(in_addr_t)), &b->sin_port, sizeof(in_port_t));
                }
                break;
            }
        } else if (lep._iptcp->ai_family == AF_INET6) {
            struct sockaddr_in6 *a = ((struct sockaddr_in6 *)lep._iptcp->ai_addr);
            struct sockaddr_in6 *b = ((struct sockaddr_in6 *)&raddr);
            if (!((a->sin6_port == b->sin6_port) &&
                  (memcmp(a->sin6_addr.s6_addr, b->sin6_addr.s6_addr, sizeof(struct in6_addr)) == 0))) {
                // If addr is not NULL, it means that the rep was requested by the upper-layers
                if (addr != NULL) {
                    assert(addr->len >= sizeof(struct in6_addr) + sizeof(in_port_t));
                    addr->len = sizeof(struct in6_addr) + sizeof(in_port_t);
                    (void)memcpy((uint8_t *)addr->start, &b->sin6_addr.s6_addr, sizeof(struct in6_addr));
                    (void)memcpy((uint8_t *)(addr->start + sizeof(struct in6_addr)), &b->sin6_port, sizeof(in_port_t));
                }
                break;
            }
        } else {
            continue;  // FIXME: support error report on invalid packet to the upper layer
        }
    } while (1);

    return (size_t)rb;
}

size_t _z_read_exact_udp_multicast(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len,
                                   const _z_sys_net_endpoint_t lep, _z_slice_t *addr) {
    size_t n = 0;
    uint8_t *pos = &ptr[0];

    do {
        size_t rb = _z_read_udp_multicast(sock, pos, len - n, lep, addr);
        if ((rb == SIZE_MAX) || (rb == 0)) {
            n = rb;
            break;
        }

        n = n + rb;
        pos = _z_ptr_u8_offset(pos, (ptrdiff_t)n);
    } while (n != len);

    return n;
}

size_t _z_send_udp_multicast(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len,
                             const _z_sys_net_endpoint_t rep) {
    return (size_t)sendto(sock._fd, ptr, len, 0, rep._iptcp->ai_addr, rep._iptcp->ai_addrlen);
}

#endif

#if Z_FEATURE_LINK_BLUETOOTH == 1
#error "Bluetooth not supported yet on Unix port of Zenoh-Pico"
#endif

#if Z_FEATURE_LINK_SERIAL == 1
#include <termios.h>

#include "zenoh-pico/system/link/serial.h"
#include "zenoh-pico/protocol/codec/serial.h"
#include "zenoh-pico/utils/checksum.h"
#include "zenoh-pico/utils/encoding.h"

/*------------------ Serial sockets ------------------*/

static speed_t _get_baudrate(uint32_t baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
    }
}

z_result_t _z_open_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;

    sock->_fd = open(dev, O_RDWR | O_NOCTTY);
    if (sock->_fd < 0) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        return _Z_ERR_GENERIC;
    }

    struct termios tty;
    (void)memset(&tty, 0, sizeof(tty));
    if (tcgetattr(sock->_fd, &tty) != 0) {
        close(sock->_fd);
        sock->_fd = -1;
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        return _Z_ERR_GENERIC;
    }

    cfmakeraw(&tty);
    speed_t speed = _get_baudrate(baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control (matches Zenoh Rust defaults)
    tty.c_cflag &= ~(tcflag_t)(CSTOPB | CRTSCTS);
    tty.c_cflag |= (tcflag_t)(CLOCAL | CREAD);

    // Blocking read with per-byte timeout
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 10;  // 1 second timeout (in deciseconds)

    if (tcsetattr(sock->_fd, TCSANOW, &tty) != 0) {
        close(sock->_fd);
        sock->_fd = -1;
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        return _Z_ERR_GENERIC;
    }

    // Flush any stale data
    tcflush(sock->_fd, TCIOFLUSH);

    return (ret == _Z_RES_OK ? _z_connect_serial(*sock) : ret);
}

z_result_t _z_open_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    (void)(sock);
    (void)(txpin);
    (void)(rxpin);
    (void)(baudrate);

    // Pin-based serial not applicable on Unix
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    return _Z_ERR_GENERIC;
}

z_result_t _z_listen_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    // Serial is symmetric — listen is same as open
    return _z_open_serial_from_dev(sock, dev, baudrate);
}

z_result_t _z_listen_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    (void)(sock);
    (void)(txpin);
    (void)(rxpin);
    (void)(baudrate);

    // Pin-based serial not applicable on Unix
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    return _Z_ERR_GENERIC;
}

void _z_close_serial(_z_sys_net_socket_t *sock) {
    if (sock->_fd >= 0) {
        close(sock->_fd);
        sock->_fd = -1;
    }
}

size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    uint8_t *raw_buf = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);
    if (raw_buf == NULL) {
        _Z_ERROR("Failed to allocate serial COBS buffer");
        return SIZE_MAX;
    }
    size_t rb = 0;
    for (size_t i = 0; i < _Z_SERIAL_MAX_COBS_BUF_SIZE; i++) {
        ssize_t n = read(sock._fd, &raw_buf[i], 1);
        if (n <= 0) {
            z_free(raw_buf);
            return SIZE_MAX;
        }

        rb++;
        if (raw_buf[i] == (uint8_t)0x00) {  // End-of-packet marker
            break;
        }
    }

    uint8_t *tmp_buf = (uint8_t *)z_malloc(_Z_SERIAL_MFS_SIZE);
    if (tmp_buf == NULL) {
        _Z_ERROR("Failed to allocate serial MFS buffer");
        z_free(raw_buf);
        return SIZE_MAX;
    }
    size_t ret = _z_serial_msg_deserialize(raw_buf, rb, ptr, len, header, tmp_buf, _Z_SERIAL_MFS_SIZE);

    z_free(raw_buf);
    z_free(tmp_buf);

    return ret;
}

size_t _z_send_serial_internal(const _z_sys_net_socket_t sock, uint8_t header, const uint8_t *ptr, size_t len) {
    uint8_t *tmp_buf = (uint8_t *)z_malloc(_Z_SERIAL_MFS_SIZE);
    uint8_t *raw_buf = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);
    if ((raw_buf == NULL) || (tmp_buf == NULL)) {
        _Z_ERROR("Failed to allocate serial COBS and/or MFS buffer");
        z_free(tmp_buf);
        z_free(raw_buf);
        return SIZE_MAX;
    }
    size_t ret =
        _z_serial_msg_serialize(raw_buf, _Z_SERIAL_MAX_COBS_BUF_SIZE, ptr, len, header, tmp_buf, _Z_SERIAL_MFS_SIZE);

    if (ret == SIZE_MAX) {
        z_free(raw_buf);
        z_free(tmp_buf);
        return ret;
    }

    ssize_t wb = write(sock._fd, raw_buf, ret);

    z_free(raw_buf);
    z_free(tmp_buf);

    if (wb < 0 || (size_t)wb != ret) {
        return SIZE_MAX;
    }

    return len;
}
#endif  // Z_FEATURE_LINK_SERIAL == 1

/*------------------ CAN sockets ------------------*/
// RFC-0080 / phase-377. Linux SocketCAN. This is also the binding that talks
// to a virtual `vcan0`, which is what makes the link testable with no hardware.
#if Z_FEATURE_LINK_CAN == 1

static z_result_t __z_can_bind(_z_can_socket_t *sock, const char *dev, uint32_t dbitrate, uint32_t id, uint32_t match,
                               uint32_t mask) {
    // Bit rates are set out of band on Linux (`ip link set can0 type can
    // bitrate ...`), and a virtual interface has none at all. Honour the
    // contract in system/link/can.h: do not fail on a rate we cannot apply.
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        _Z_ERROR("CAN: socket(PF_CAN) failed: %d", errno);
        return _Z_ERR_GENERIC;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    (void)strncpy(ifr.ifr_name, dev, sizeof(ifr.ifr_name) - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        _Z_ERROR("CAN: no such interface '%s': %d", dev, errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    // Admit the whole band this bus segment uses for zenoh; the read then drops
    // our own frames. A mask of 0 matches everything, which is the default for
    // a bus carrying nothing else.
    struct can_filter filter;
    filter.can_id = match;
    filter.can_mask = mask;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        _Z_ERROR("CAN: CAN_RAW_FILTER failed: %d", errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    // Ask for CAN FD. If the interface does not support it we fall back to
    // classic framing rather than failing, and report the mode we got so the
    // link can size its MTU from reality.
    _Bool fd_mode = false;
    if (dbitrate != 0u) {
        int enable = 1;
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) == 0) {
            fd_mode = true;
        } else {
            _Z_DEBUG("CAN: interface '%s' has no CAN FD, using classic frames", dev);
        }
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        _Z_ERROR("CAN: bind('%s') failed: %d", dev, errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    sock->_sock._fd = fd;
    sock->_id = id;
    sock->_match = match;
    sock->_mask = mask;
    sock->_fd_mode = fd_mode;
    sock->_mtu = fd_mode ? _Z_CAN_FD_MTU_SIZE : _Z_CAN_CLASSIC_MTU_SIZE;
    return _Z_RES_OK;
}

z_result_t _z_open_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint32_t id,
                       uint32_t match, uint32_t mask) {
    _ZP_UNUSED(bitrate);
    return __z_can_bind(sock, dev, dbitrate, id, match, mask);
}

z_result_t _z_listen_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint32_t id,
                         uint32_t match, uint32_t mask) {
    _ZP_UNUSED(bitrate);
    // Multicast peers all listen; a bus has no connection setup.
    return __z_can_bind(sock, dev, dbitrate, id, match, mask);
}

void _z_close_can(_z_can_socket_t *sock) {
    if (sock->_sock._fd >= 0) {
        close(sock->_sock._fd);
        sock->_sock._fd = -1;
    }
}

size_t _z_send_can(const _z_can_socket_t *sock, const uint8_t *ptr, size_t len) {
    if (len > sock->_mtu) {
        _Z_ERROR("CAN: datagram %zu exceeds link MTU %u", len, (unsigned)sock->_mtu);
        return SIZE_MAX;
    }

    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = sock->_id;
    // Byte 0 carries the true length; CAN FD DLCs are quantised so the frame
    // may be longer than the datagram (system/link/can.h).
    frame.data[0] = (uint8_t)len;
    if (len > 0) {
        memcpy(&frame.data[1], ptr, len);
    }

    size_t payload = len + _Z_CAN_LEN_PREFIX;
    size_t frame_len = payload;
    if (sock->_fd_mode && (payload > 8u)) {
        // Round up to the next representable CAN FD length.
        static const uint8_t steps[] = {12, 16, 20, 24, 32, 48, 64};
        for (size_t i = 0; i < (sizeof(steps) / sizeof(steps[0])); i++) {
            if (payload <= steps[i]) {
                frame_len = steps[i];
                break;
            }
        }
        frame.flags = CANFD_BRS;  // use the fast data phase when configured
    } else if (sock->_fd_mode) {
        frame.flags = CANFD_BRS;
    }
    frame.len = (uint8_t)frame_len;

    size_t wire = sock->_fd_mode ? CANFD_MTU : CAN_MTU;
    ssize_t wb = write(sock->_sock._fd, &frame, wire);
    if (wb < 0 || (size_t)wb != wire) {
        _Z_ERROR("CAN: write failed: %d", errno);
        return SIZE_MAX;
    }

    return len;
}

size_t _z_read_can(const _z_can_socket_t *sock, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    struct canfd_frame frame;

    // Loop until a frame arrives that is not ours, mirroring
    // `_z_read_udp_multicast`: on a bus every peer hears everything, including
    // its own transmissions on a loopback-enabled interface.
    for (;;) {
        ssize_t rb = read(sock->_sock._fd, &frame, sizeof(frame));
        if (rb < 0) {
            if (errno == EINTR) {
                continue;
            }
            return SIZE_MAX;
        }
        if ((rb != (ssize_t)CANFD_MTU) && (rb != (ssize_t)CAN_MTU)) {
            continue;  // runt or error frame
        }

        uint32_t sender = frame.can_id & CAN_EFF_MASK;
        if (sender == sock->_id) {
            continue;  // our own frame
        }
        if ((sock->_mask != 0u) && ((sender & sock->_mask) != sock->_match)) {
            continue;  // outside the band this bus reserves for zenoh
        }
        if (frame.len < _Z_CAN_LEN_PREFIX) {
            continue;  // no length byte
        }

        size_t dlen = frame.data[0];
        if ((dlen > (size_t)(frame.len - _Z_CAN_LEN_PREFIX)) || (dlen > len)) {
            // Length byte disagrees with the frame, or the caller's buffer is
            // too small. Either way this datagram is unusable — drop it rather
            // than hand back a truncated one that would deserialize as garbage.
            _Z_ERROR("CAN: bad datagram length %zu (frame %u, buffer %zu)", dlen, (unsigned)frame.len, len);
            continue;
        }

        if (addr != NULL) {
            assert(addr->len >= _Z_CAN_ADDR_SIZE);
            addr->len = _Z_CAN_ADDR_SIZE;
            uint8_t *dst = (uint8_t *)addr->start;
            dst[0] = (uint8_t)(sender & 0xFFu);
            dst[1] = (uint8_t)((sender >> 8) & 0xFFu);
            dst[2] = (uint8_t)((sender >> 16) & 0xFFu);
            dst[3] = (uint8_t)((sender >> 24) & 0xFFu);
        }

        if (dlen > 0) {
            memcpy(ptr, &frame.data[1], dlen);
        }
        return dlen;
    }
}

#endif  // Z_FEATURE_LINK_CAN == 1

#if Z_FEATURE_LINK_ISOTP == 1 && Z_FEATURE_LINK_ISOTP_VENDORED == 0

// The kernel does ISO-TP for us: segmentation, flow control and the N_As/N_Bs/
// N_Cr timers all live in `net/can/isotp.c`, and a read returns one whole
// reassembled PDU. That is the entire reason this link is small -- the protocol
// is a platform capability, not something zenoh-pico reimplements.
//
// The address is the DIRECTED IDENTIFIER PAIR, carried in `sockaddr_can.can_addr.tp`.
// Note the declaration order in <linux/can.h>:
//
//     struct { canid_t rx_id, tx_id; } tp;
//
// rx comes FIRST. The fields are set by name below so that reading this code
// against the struct cannot silently swap the two -- a swap costs no error at
// bind() time and simply produces a channel where nothing ever arrives.
z_result_t _z_open_isotp(_z_isotp_socket_t *sock, const char *dev, uint32_t tx_id, uint32_t rx_id, _Bool eff) {
    int fd = socket(PF_CAN, SOCK_DGRAM, CAN_ISOTP);
    if (fd < 0) {
        // ENOPROTOOPT here means the can-isotp module is not loaded, which is
        // by far the most common way this fails on an otherwise fine system.
        _Z_ERROR("ISO-TP: socket(PF_CAN, CAN_ISOTP) failed: %d (is the can-isotp module loaded?)", errno);
        return _Z_ERR_GENERIC;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    (void)strncpy(ifr.ifr_name, dev, sizeof(ifr.ifr_name) - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        _Z_ERROR("ISO-TP: no such interface '%s': %d", dev, errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    // A 29-bit identifier is marked by CAN_EFF_FLAG on the identifier itself,
    // not by a socket option.
    addr.can_addr.tp.rx_id = eff ? (rx_id | CAN_EFF_FLAG) : rx_id;
    addr.can_addr.tp.tx_id = eff ? (tx_id | CAN_EFF_FLAG) : tx_id;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        // EADDRINUSE means another socket already holds this pair on this
        // interface -- two peers configured with the same identifiers.
        _Z_ERROR("ISO-TP: bind('%s', rx=0x%x tx=0x%x) failed: %d", dev, (unsigned)rx_id, (unsigned)tx_id, errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    sock->_sock._fd = fd;
    sock->_tx_id = tx_id;
    sock->_rx_id = rx_id;
    sock->_eff = eff;
    return _Z_RES_OK;
}

void _z_close_isotp(_z_isotp_socket_t *sock) {
    if (sock->_sock._fd >= 0) {
        close(sock->_sock._fd);
        sock->_sock._fd = -1;
    }
}

size_t _z_read_isotp_socket(const _z_sys_net_socket_t socket, uint8_t *ptr, size_t len) {
    ssize_t rb = recv(socket._fd, ptr, len, 0);
    if (rb < 0) {
        _Z_ERROR("ISO-TP: recv failed: %d", errno);
        return SIZE_MAX;
    }
    // A PDU longer than the caller's buffer is TRUNCATED rather than short-read:
    // ISO-TP is datagram-oriented and the rest is discarded by the kernel. Say
    // so instead of handing up a silently mutilated batch, which would surface
    // far away as a zenoh codec error.
    if ((size_t)rb > len) {
        _Z_ERROR("ISO-TP: PDU of %zd bytes truncated into a %zu byte buffer", rb, len);
        return SIZE_MAX;
    }
    return (size_t)rb;
}

size_t _z_read_isotp(const _z_isotp_socket_t *sock, uint8_t *ptr, size_t len) {
    return _z_read_isotp_socket(sock->_sock, ptr, len);
}

size_t _z_send_isotp(const _z_isotp_socket_t *sock, const uint8_t *ptr, size_t len) {
    if (len > _Z_ISOTP_MTU_SIZE) {
        _Z_ERROR("ISO-TP: PDU %zu exceeds the 12-bit FF_DL limit of %d", len, _Z_ISOTP_MTU_SIZE);
        return SIZE_MAX;
    }
    // One write is one PDU. It blocks until the peer's flow control has let the
    // whole thing out, so a send that returns is a send the peer has paced --
    // there is no partial write to loop over.
    ssize_t wb = send(sock->_sock._fd, ptr, len, 0);
    if (wb < 0) {
        // ECOMM is the kernel's way of reporting a flow-control timeout: the
        // peer stopped answering mid-PDU.
        _Z_ERROR("ISO-TP: send of %zu bytes failed: %d", len, errno);
        return SIZE_MAX;
    }
    return (size_t)wb;
}

#endif  // Z_FEATURE_LINK_ISOTP == 1 && Z_FEATURE_LINK_ISOTP_VENDORED == 0
