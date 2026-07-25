//
// Copyright (c) 2022 ZettaScale Technology
// Copyright (c) 2023 Fictionlab sp. z o.o.
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   João Mário Lago, <joaolago@bluerobotics.com>
//

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/api.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sockets.h"
#include "lwip/udp.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/utils/logging.h"
#include "zenoh-pico/utils/pointers.h"
#include "zenoh-pico/utils/result.h"

#if Z_FEATURE_LINK_TCP == 1
static char _z_lwip_numeric_addrinfo_marker;

static void _z_lwip_thread_init(void) {
#if LWIP_NETCONN_SEM_PER_THREAD
    lwip_socket_thread_init();
#endif
}

static z_result_t _z_connect_tcp_timeout(int sock, const struct sockaddr *addr,
                                         socklen_t addrlen, uint32_t tout) {
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
    if ((lwip_getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &optlen) < 0) ||
        (err != 0)) {
        (void)lwip_fcntl(sock, F_SETFL, original_flags);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    if (lwip_fcntl(sock, F_SETFL, original_flags) == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    return _Z_RES_OK;
}

z_result_t _z_socket_set_non_blocking(const _z_sys_net_socket_t *sock) {
    _z_lwip_thread_init();

    int flags = lwip_fcntl(sock->_socket, F_GETFL, 0);
    if (flags == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    if (lwip_fcntl(sock->_socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    return _Z_RES_OK;
}

z_result_t _z_socket_accept(const _z_sys_net_socket_t *sock_in, _z_sys_net_socket_t *sock_out) {
    _z_lwip_thread_init();

    struct sockaddr naddr;
    socklen_t nlen = sizeof(naddr);
    sock_out->_socket = -1;
    int con_socket = lwip_accept(sock_in->_socket, &naddr, &nlen);
    if (con_socket < 0) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
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
    sock_out->_socket = con_socket;
    return _Z_RES_OK;
}

void _z_socket_close(_z_sys_net_socket_t *sock) {
    _z_lwip_thread_init();

    shutdown(sock->_socket, SHUT_RDWR);
    lwip_close(sock->_socket);
}

#if Z_FEATURE_MULTI_THREAD == 1
z_result_t _z_socket_wait_event(void *v_peers, _z_mutex_rec_t *mutex) {
    _z_lwip_thread_init();

    fd_set read_fds;
    FD_ZERO(&read_fds);
    // Create select mask
    _z_transport_peer_unicast_slist_t **peers = (_z_transport_peer_unicast_slist_t **)v_peers;
    _z_mutex_rec_lock(mutex);
    _z_transport_peer_unicast_slist_t *curr = *peers;
    int max_fd = 0;
    while (curr != NULL) {
        _z_transport_peer_unicast_t *peer = _z_transport_peer_unicast_slist_value(curr);
        FD_SET(peer->_socket._socket, &read_fds);
        if (peer->_socket._socket > max_fd) {
            max_fd = peer->_socket._socket;
        }
        curr = _z_transport_peer_unicast_slist_next(curr);
    }
    _z_mutex_rec_unlock(mutex);
    // Wait for events
    struct timeval timeout;
    timeout.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / 1000;
    timeout.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % 1000) * 1000;
    if (lwip_select(max_fd + 1, &read_fds, NULL, NULL, &timeout) <= 0) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);  // Error or no data ready
    }
    // Mark sockets that are pending
    _z_mutex_rec_lock(mutex);
    curr = *peers;
    while (curr != NULL) {
        _z_transport_peer_unicast_t *peer = _z_transport_peer_unicast_slist_value(curr);
        if (FD_ISSET(peer->_socket._socket, &read_fds)) {
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

/*------------------ TCP sockets ------------------*/
z_result_t _z_create_endpoint_tcp(_z_sys_net_endpoint_t *ep, const char *s_address, const char *s_port) {
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
    (void)memset(&hints, 0, sizeof(hints));
    hints.ai_family = PF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_TCP;

    if (getaddrinfo(s_address, s_port, &hints, &ep->_iptcp) < 0) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
        return ret;
    }
    ep->_iptcp->ai_addr->sa_family = ep->_iptcp->ai_family;

    return ret;
}

void _z_free_endpoint_tcp(_z_sys_net_endpoint_t *ep) {
    _z_lwip_thread_init();

    if (ep->_iptcp == NULL) {
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

z_result_t _z_open_tcp(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, uint32_t tout) {
    z_result_t ret = _Z_RES_OK;

    _z_lwip_thread_init();
    sock->_socket = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
    if (sock->_socket != -1) {
        z_time_t tv;
        tv.tv_sec = tout / (uint32_t)1000;
        tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
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
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        int flags = 1;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_socket, SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#if Z_FEATURE_TCP_NODELAY == 1
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_socket, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#endif
        struct linger ling;
        ling.l_onoff = 1;
        ling.l_linger = Z_TRANSPORT_LEASE / 1000;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_socket, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        struct addrinfo *it = NULL;
        for (it = rep._iptcp; it != NULL; it = it->ai_next) {
            if ((ret == _Z_RES_OK) && (_z_connect_tcp_timeout(sock->_socket, it->ai_addr,
                                                              it->ai_addrlen,
                                                              tout) != _Z_RES_OK)) {
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
            close(sock->_socket);
            sock->_socket = -1;
        }
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

z_result_t _z_listen_tcp(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t lep) {
    z_result_t ret = _Z_RES_OK;

    _z_lwip_thread_init();

    sock->_socket = socket(lep._iptcp->ai_family, lep._iptcp->ai_socktype, lep._iptcp->ai_protocol);
    if (sock->_socket == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    int flags = 1;
    if ((ret == _Z_RES_OK) &&
        (setsockopt(sock->_socket, SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#if Z_FEATURE_TCP_NODELAY == 1
    if ((ret == _Z_RES_OK) &&
        (setsockopt(sock->_socket, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#endif
    struct linger ling;
    ling.l_onoff = 1;
    ling.l_linger = Z_TRANSPORT_LEASE / 1000;
    if ((ret == _Z_RES_OK) &&
        (setsockopt(sock->_socket, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
    struct addrinfo *it = NULL;
    if (ret == _Z_RES_OK) {
        for (it = lep._iptcp; it != NULL; it = it->ai_next) {
            if (bind(sock->_socket, it->ai_addr, it->ai_addrlen) < 0) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
                break;
            }
            if (listen(sock->_socket, Z_LISTEN_MAX_CONNECTION_NB) < 0) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
                break;
            }
        }
    }
    if (ret != _Z_RES_OK) {
        close(sock->_socket);
        sock->_socket = -1;
    }
    return ret;
}

void _z_close_tcp(_z_sys_net_socket_t *sock) {
    _z_lwip_thread_init();

    if (sock->_socket >= 0) {
        shutdown(sock->_socket, SHUT_RDWR);
        close(sock->_socket);
        sock->_socket = -1;
    }
}

size_t _z_read_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    _z_lwip_thread_init();

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock._socket, &read_fds);

    struct timeval tv;
    tv.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / (uint32_t)1000;
    tv.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % (uint32_t)1000) * (uint32_t)1000;

    int ready = lwip_select(sock._socket + 1, &read_fds, NULL, NULL, &tv);
    if (ready <= 0) {
        return SIZE_MAX;
    }

    ssize_t rb = recv(sock._socket, ptr, len, 0);
    if (rb < (ssize_t)0) {
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
        pos = _z_ptr_u8_offset(pos, rb);
    } while (n != len);

    return n;
}

size_t _z_send_tcp(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
    _z_lwip_thread_init();

    // Drain loop (nano-ros issue 0269): the link layer's `write_all`
    // callers assume a full write, but lwIP's send buffers are tiny
    // (TCP_SND_BUF is typically a few MSS and TCP_SND_QUEUELEN a couple
    // dozen segments), so a burst of small writes — e.g. the DECLARE
    // storm of a many-entity session open — short-writes or fails with
    // ERR_MEM/EWOULDBLOCK long before the unix-sized buffers would.
    // Loop select+send until every byte is accepted; each iteration
    // gets the full socket timeout, so a stalled peer still errors out.
    size_t sent = 0;
    while (sent < len) {
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(sock._socket, &write_fds);

        struct timeval tv;
        tv.tv_sec = Z_CONFIG_SOCKET_TIMEOUT / (uint32_t)1000;
        tv.tv_usec = (Z_CONFIG_SOCKET_TIMEOUT % (uint32_t)1000) * (uint32_t)1000;

        int ready = lwip_select(sock._socket + 1, NULL, &write_fds, NULL, &tv);
        if (ready <= 0) {
            return SIZE_MAX;
        }

        int wb = send(sock._socket, ptr + sent, len - sent, 0);
        if (wb < 0) {
            // Writable per select, yet the write was refused: lwIP can
            // still return ERR_MEM/EWOULDBLOCK when the segment queue
            // (TCP_SND_QUEUELEN) is exhausted even though byte-space
            // remains. Yield until ACKs free segments, then retry.
            if (errno == EWOULDBLOCK || errno == EAGAIN || errno == ENOMEM) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            return SIZE_MAX;
        }
        sent += (size_t)wb;
    }

    return sent;
}
#else
z_result_t _z_socket_set_non_blocking(const _z_sys_net_socket_t *sock) {
    _ZP_UNUSED(sock);
    _Z_ERROR("Function not yet supported on this system");
    _Z_ERROR_RETURN(_Z_ERR_GENERIC);
}

z_result_t _z_socket_accept(const _z_sys_net_socket_t *sock_in, _z_sys_net_socket_t *sock_out) {
    _ZP_UNUSED(sock_in);
    _ZP_UNUSED(sock_out);
    _Z_ERROR("Function not yet supported on this system");
    _Z_ERROR_RETURN(_Z_ERR_GENERIC);
}

void _z_socket_close(_z_sys_net_socket_t *sock) { _ZP_UNUSED(sock); }

z_result_t _z_socket_wait_event(void *peers, _z_mutex_rec_t *mutex) {
    _ZP_UNUSED(peers);
    _ZP_UNUSED(mutex);
    _Z_ERROR("Function not yet supported on this system");
    _Z_ERROR_RETURN(_Z_ERR_GENERIC);
}
#endif  // Z_FEATURE_LINK_TCP == 1

#if Z_FEATURE_LINK_UDP_UNICAST == 1 || Z_FEATURE_LINK_UDP_MULTICAST == 1
/*------------------ UDP sockets ------------------*/
z_result_t _z_create_endpoint_udp(_z_sys_net_endpoint_t *ep, const char *s_address, const char *s_port) {
    z_result_t ret = _Z_RES_OK;

    struct addrinfo hints;
    (void)memset(&hints, 0, sizeof(hints));
    hints.ai_family = PF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_UDP;

    if (getaddrinfo(s_address, s_port, &hints, &ep->_iptcp) < 0) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
        return ret;
    }
    ep->_iptcp->ai_addr->sa_family = ep->_iptcp->ai_family;

    return ret;
}

void _z_free_endpoint_udp(_z_sys_net_endpoint_t *ep) { freeaddrinfo(ep->_iptcp); }
#endif

#if Z_FEATURE_LINK_UDP_UNICAST == 1
z_result_t _z_open_udp_unicast(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, uint32_t tout) {
    z_result_t ret = _Z_RES_OK;

    sock->_socket = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
    if (sock->_socket != -1) {
        z_time_t tv;
        tv.tv_sec = tout / (uint32_t)1000;
        tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        if (ret != _Z_RES_OK) {
            close(sock->_socket);
            sock->_socket = -1;
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
    if (sock->_socket >= 0) {
        close(sock->_socket);
        sock->_socket = -1;
    }
}

size_t _z_read_udp_unicast(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    struct sockaddr_storage raddr;
    long unsigned int addrlen = sizeof(struct sockaddr_storage);

    ssize_t rb = recvfrom(sock._socket, ptr, len, 0, (struct sockaddr *)&raddr, &addrlen);
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
    return (size_t)sendto(sock._socket, ptr, len, 0, rep._iptcp->ai_addr, rep._iptcp->ai_addrlen);
}
#endif

#if Z_FEATURE_LINK_UDP_MULTICAST == 1
long unsigned int __get_ip_from_iface(const char *iface, int sa_family, struct sockaddr **lsockaddr) {
    unsigned int addrlen = 0U;

    struct netif *netif = netif_find(iface);
    if (netif != NULL && netif_is_up(netif)) {
        struct sockaddr_in *lsockaddr_in = (struct sockaddr_in *)z_malloc(sizeof(struct sockaddr_in));
        if (lsockaddr != NULL) {
            (void)memset(lsockaddr_in, 0, sizeof(struct sockaddr_in));
            const ip4_addr_t *ip4_addr = netif_ip4_addr(netif);
            inet_addr_from_ip4addr(&lsockaddr_in->sin_addr, ip_2_ip4(ip4_addr));
            lsockaddr_in->sin_family = AF_INET;
            lsockaddr_in->sin_port = htons(0);

            addrlen = sizeof(struct sockaddr_in);
            *lsockaddr = (struct sockaddr *)lsockaddr_in;
        }
    }

    return addrlen;
}

z_result_t _z_open_udp_multicast(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, _z_sys_net_endpoint_t *lep,
                                 uint32_t tout, const char *iface) {
    z_result_t ret = _Z_RES_OK;

    struct sockaddr *lsockaddr = NULL;
    long unsigned int addrlen = __get_ip_from_iface(iface, rep._iptcp->ai_family, &lsockaddr);
    if (addrlen != 0U) {
        sock->_socket = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
        if (sock->_socket != -1) {
            z_time_t tv;
            tv.tv_sec = tout / (uint32_t)1000;
            tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
            if ((ret == _Z_RES_OK) &&
                (setsockopt(sock->_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

            if ((ret == _Z_RES_OK) && (bind(sock->_socket, lsockaddr, addrlen) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

            // Get the randomly assigned port used to discard loopback messages
            if ((ret == _Z_RES_OK) && (getsockname(sock->_socket, lsockaddr, &addrlen) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

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
                close(sock->_socket);
                sock->_socket = -1;
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
        sock->_socket = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
        if (sock->_socket != -1) {
            z_time_t tv;
            tv.tv_sec = tout / (uint32_t)1000;
            tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
            if ((ret == _Z_RES_OK) &&
                (setsockopt(sock->_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }

            if (rep._iptcp->ai_family == AF_INET) {
                struct sockaddr_in address;
                (void)memset(&address, 0, sizeof(address));
                address.sin_family = (sa_family_t)rep._iptcp->ai_family;
                address.sin_port = ((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_port;
                inet_pton(address.sin_family, "0.0.0.0", &address.sin_addr);
                if ((ret == _Z_RES_OK) && (bind(sock->_socket, (struct sockaddr *)&address, sizeof(address)) < 0)) {
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
                    (setsockopt(sock->_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)) {
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
                            (setsockopt(sock->_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)) {
                            _Z_ERROR_LOG(_Z_ERR_GENERIC);
                            ret = _Z_ERR_GENERIC;
                        }
                    }
                }
                z_free(joins);
            }

            if (ret != _Z_RES_OK) {
                close(sock->_socket);
                sock->_socket = -1;
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
    if (sockrecv->socket >= 0) {
        if (rep._iptcp->ai_family == AF_INET) {
            struct ip_mreq mreq;
            (void)memset(&mreq, 0, sizeof(mreq));
            mreq.imr_multiaddr.s_addr = ((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_addr.s_addr;
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
            setsockopt(sockrecv->_socket, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
        } else {
            // Do nothing. It must never not enter here.
            // Required to be compliant with MISRA 15.7 rule
        }
    }
    _ZP_UNUSED(lep);

    if (sockrecv->_socket >= 0) {
        close(sockrecv->_socket);
        sockrecv->_socket = -1;
    }
    if (socksend->_socket >= 0) {
        close(socksend->_socket);
        socksend->_socket = -1;
    }
}

size_t _z_read_udp_multicast(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len, const _z_sys_net_endpoint_t lep,
                             _z_slice_t *addr) {
    struct sockaddr_storage raddr;
    long unsigned int replen = sizeof(struct sockaddr_storage);

    ssize_t rb = 0;
    do {
        rb = recvfrom(sock._socket, ptr, len, 0, (struct sockaddr *)&raddr, &replen);
        if (rb < (ssize_t)0) {
            return SIZE_MAX;
        }

        if (lep._iptcp->ai_family == AF_INET) {
            struct sockaddr_in *a = ((struct sockaddr_in *)lep._iptcp->ai_addr);
            struct sockaddr_in *b = ((struct sockaddr_in *)&raddr);
            if (!((a->sin_port == b->sin_port) && (a->sin_addr.s_addr == b->sin_addr.s_addr))) {
                // If addr is not NULL, it means that the rep was requested by the upper-layers
                if (addr != NULL) {
                    *addr = _z_slice_make(sizeof(in_addr_t) + sizeof(in_port_t));
                    (void)memcpy((uint8_t *)addr->start, &b->sin_addr.s_addr, sizeof(in_addr_t));
                    (void)memcpy((uint8_t *)(addr->start + sizeof(in_addr_t)), &b->sin_port, sizeof(in_port_t));
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
    return (size_t)sendto(sock._socket, ptr, len, 0, rep._iptcp->ai_addr, rep._iptcp->ai_addrlen);
}

#endif

#if Z_FEATURE_LINK_BLUETOOTH == 1
#error "Bluetooth not supported yet on FreeRTOS + LWIP port of Zenoh-Pico"
#endif

#if Z_FEATURE_LINK_SERIAL == 1
#error "Serial not supported yet on FreeRTOS + LWIP port of Zenoh-Pico"
#endif

#if Z_FEATURE_RAWETH_TRANSPORT == 1
#error "Raw ethernet transport not supported yet on FreeRTOS + LWIP port of Zenoh-Pico"
#endif
