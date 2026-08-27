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

#include <version.h>

#if KERNEL_VERSION_MAJOR == 2
#include <drivers/uart.h>
#else
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#endif

#include <fcntl.h>
#if defined(CONFIG_NET_SOCKETS)
#include <netdb.h>
#endif
#include <stdbool.h>
#include <stddef.h>
#if defined(CONFIG_NET_SOCKETS)
#include <sys/socket.h>
#endif
#include <unistd.h>
#include <zephyr/net/net_if.h>
#include <zephyr/posix/sys/select.h>

#include "zenoh-pico/collections/string.h"

// RFC-0080 — after the zenoh includes, so Z_FEATURE_LINK_CAN is defined.
#if Z_FEATURE_LINK_CAN == 1
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#endif
#include "zenoh-pico/config.h"
#include "zenoh-pico/protocol/codec/serial.h"
#include "zenoh-pico/system/common/serial.h"
#include "zenoh-pico/system/link/serial.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/utils/checksum.h"
#include "zenoh-pico/utils/encoding.h"
#include "zenoh-pico/utils/logging.h"
#include "zenoh-pico/utils/pointers.h"

/* Socket-backed helpers. Guarded because a serial-only build has no IP stack:
 * these use fcntl/accept/sockaddr/socklen_t and the `_fd` member, none of which
 * exist when every socket link is off. Zephyr's port compiled them
 * unconditionally, so `serial` alone did not build. */
#define _Z_ZEPHYR_HAS_SOCKET_LINK                                                  \
    ((Z_FEATURE_LINK_TCP == 1) || (Z_FEATURE_LINK_UDP_UNICAST == 1) ||             \
     (Z_FEATURE_LINK_UDP_MULTICAST == 1) || (Z_FEATURE_LINK_WS == 1))

#if _Z_ZEPHYR_HAS_SOCKET_LINK

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
    socklen_t nlen = sizeof(naddr);
    sock_out->_fd = -1;
    int con_socket = accept(sock_in->_fd, &naddr, &nlen);
    if (con_socket < 0) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    // Set socket options
#if Z_FEATURE_TCP_NODELAY == 1
    int optflag = 1;
    if (setsockopt(con_socket, IPPROTO_TCP, TCP_NODELAY, (void *)&optflag, sizeof(optflag)) < 0) {
        close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
#endif
#if LWIP_SO_LINGER == 1
    struct linger ling;
    ling.l_onoff = 1;
    ling.l_linger = Z_TRANSPORT_LEASE / 1000;
    if (setsockopt(con_socket, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0) {
        close(con_socket);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
#endif
    // Note socket
    sock_out->_fd = con_socket;
    return _Z_RES_OK;
}

void _z_socket_close(_z_sys_net_socket_t *sock) {
    if (sock->_fd >= 0) {
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
    if (result <= 0) {
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

#else /* !_Z_ZEPHYR_HAS_SOCKET_LINK */

/* zenoh-pico's transport layer references these unconditionally, so a
 * serial-only build still needs the symbols. Nothing can reach them without a
 * socket link -- there is no socket to accept, close or wait on -- so they fail
 * rather than pretend to succeed. */
z_result_t _z_socket_set_non_blocking(const _z_sys_net_socket_t *sock) {
    (void)sock;
    return _Z_ERR_GENERIC;
}

z_result_t _z_socket_accept(const _z_sys_net_socket_t *sock_in, _z_sys_net_socket_t *sock_out) {
    (void)sock_in;
    (void)sock_out;
    return _Z_ERR_GENERIC;
}

void _z_socket_close(_z_sys_net_socket_t *sock) { (void)sock; }

z_result_t _z_socket_wait_event(void *v_peers, _z_mutex_rec_t *mutex) {
    (void)v_peers;
    (void)mutex;
    return _Z_ERR_GENERIC;
}

#endif /* _Z_ZEPHYR_HAS_SOCKET_LINK */

#if Z_FEATURE_LINK_TCP == 1
/*------------------ TCP sockets ------------------*/
z_result_t _z_create_endpoint_tcp(_z_sys_net_endpoint_t *ep, const char *s_address, const char *s_port) {
    z_result_t ret = _Z_RES_OK;

    struct addrinfo hints;
    (void)memset(&hints, 0, sizeof(hints));
#if defined(CONFIG_POSIX_IPV6)
    hints.ai_family = PF_UNSPEC;  // Allow IPv4 or IPv6
#else
    hints.ai_family = AF_INET;
#endif
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

z_result_t _z_open_tcp(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, uint32_t tout) {
    z_result_t ret = _Z_RES_OK;

    sock->_fd = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
    if (sock->_fd != -1) {
        z_time_t tv;
        tv.tv_sec = tout / (uint32_t)1000;
        tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            // FIXME: setting the setsockopt is consistently failing. Commenting it
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            // until further inspection. ret = _Z_ERR_GENERIC;
        }

#if Z_FEATURE_TCP_NODELAY == 1
        int optflag = 1;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_fd, IPPROTO_TCP, TCP_NODELAY, (void *)&optflag, sizeof(optflag)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#endif

#if LWIP_SO_LINGER == 1
        struct linger ling;
        ling.l_onoff = 1;
        ling.l_linger = Z_TRANSPORT_LEASE / 1000;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_fd, SOL_SOCKET, SO_LINGER, (void *)&ling, sizeof(struct linger)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#endif

        struct addrinfo *it = NULL;
        for (it = rep._iptcp; it != NULL; it = it->ai_next) {
            if ((ret == _Z_RES_OK) && connect(sock->_fd, it->ai_addr, it->ai_addrlen) < 0) {
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
#if Z_FEATURE_TCP_NODELAY == 1
    int optflag = 1;
    if ((ret == _Z_RES_OK) &&
        (setsockopt(sock->_fd, IPPROTO_TCP, TCP_NODELAY, (void *)&optflag, sizeof(optflag)) < 0)) {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }
#endif
    if (ret != _Z_RES_OK) {
        close(sock->_fd);
        sock->_fd = -1;
        return ret;
    }
    // Activate socket
    struct addrinfo *it = NULL;
    for (it = lep._iptcp; it != NULL; it = it->ai_next) {
        if (bind(sock->_fd, it->ai_addr, it->ai_addrlen) < 0) {
            if (it->ai_next == NULL) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
                break;
            }
        }
        if (listen(sock->_fd, Z_LISTEN_MAX_CONNECTION_NB) < 0) {
            if (it->ai_next == NULL) {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
                break;
            }
        }
    }
    if (ret != _Z_RES_OK) {
        close(sock->_fd);
        sock->_fd = -1;
    }
    return ret;
}

void _z_close_tcp(_z_sys_net_socket_t *sock) {
    // shutdown(sock->_fd, SHUT_RDWR); // Not implemented in Zephyr
    if (sock->_fd >= 0) {
        close(sock->_fd);
        sock->_fd = -1;
    }
}

size_t _z_read_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    ssize_t rb = recv(sock._fd, ptr, len, 0);
    if (rb < (ssize_t)0) {
        rb = SIZE_MAX;
    }

    return rb;
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
    return send(sock._fd, ptr, len, 0);
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
        tv.tv_sec = tout / (uint32_t)1000;
        tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            // FIXME: setting the setsockopt is consistently failing. Commenting it
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            // until further inspection. ret = _Z_ERR_GENERIC;
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
    z_result_t ret = _Z_RES_OK;
    (void)sock;
    (void)lep;
    (void)tout;

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
    socklen_t addrlen = sizeof(struct sockaddr_storage);

    ssize_t rb = recvfrom(sock._fd, ptr, len, 0, (struct sockaddr *)&raddr, &addrlen);
    if (rb < (ssize_t)0) {
        rb = SIZE_MAX;
    }

    return rb;
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
        pos = _z_ptr_u8_offset(pos, rb);
    } while (n != len);

    return n;
}

size_t _z_send_udp_unicast(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len,
                           const _z_sys_net_endpoint_t rep) {
    return sendto(sock._fd, ptr, len, 0, rep._iptcp->ai_addr, rep._iptcp->ai_addrlen);
}
#endif

#if Z_FEATURE_LINK_UDP_MULTICAST == 1
z_result_t _z_open_udp_multicast(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, _z_sys_net_endpoint_t *lep,
                                 uint32_t tout, const char *iface) {
    z_result_t ret = _Z_RES_OK;

    struct sockaddr *lsockaddr = NULL;
    socklen_t addrlen = 0;
    if (rep._iptcp->ai_family == AF_INET) {
        lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in));
        if (lsockaddr != NULL) {
            (void)memset(lsockaddr, 0, sizeof(struct sockaddr_in));
            addrlen = sizeof(struct sockaddr_in);

            struct sockaddr_in *c_laddr = (struct sockaddr_in *)lsockaddr;
            c_laddr->sin_family = AF_INET;
            c_laddr->sin_addr.s_addr = INADDR_ANY;
            c_laddr->sin_port = htons(INADDR_ANY);
        } else {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
    } else if (rep._iptcp->ai_family == AF_INET6) {
        lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in6));
        if (lsockaddr != NULL) {
            (void)memset(lsockaddr, 0, sizeof(struct sockaddr_in6));
            addrlen = sizeof(struct sockaddr_in6);

            struct sockaddr_in6 *c_laddr = (struct sockaddr_in6 *)lsockaddr;
            c_laddr->sin6_family = AF_INET6;
            c_laddr->sin6_addr = in6addr_any;
            c_laddr->sin6_port = htons(INADDR_ANY);
            //        c_laddr->sin6_scope_id; // Not needed to be defined
        } else {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    if (addrlen != 0U) {
        sock->_fd = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
        if (sock->_fd != -1) {
            z_time_t tv;
            tv.tv_sec = tout / (uint32_t)1000;
            tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
            if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
                // FIXME: setting the setsockopt is consistently failing. Commenting it
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                // until further inspection. ret = _Z_ERR_GENERIC;
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
    (void)join;
    z_result_t ret = _Z_RES_OK;

    struct sockaddr *lsockaddr = NULL;
    socklen_t addrlen = 0;
    if (rep._iptcp->ai_family == AF_INET) {
        lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in));
        if (lsockaddr != NULL) {
            (void)memset(lsockaddr, 0, sizeof(struct sockaddr_in));
            addrlen = sizeof(struct sockaddr_in);

            struct sockaddr_in *c_laddr = (struct sockaddr_in *)lsockaddr;
            c_laddr->sin_family = AF_INET;
            c_laddr->sin_addr.s_addr = INADDR_ANY;
            c_laddr->sin_port = ((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_port;
        } else {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
    } else if (rep._iptcp->ai_family == AF_INET6) {
        lsockaddr = (struct sockaddr *)z_malloc(sizeof(struct sockaddr_in6));
        if (lsockaddr != NULL) {
            (void)memset(lsockaddr, 0, sizeof(struct sockaddr_in6));
            addrlen = sizeof(struct sockaddr_in6);

            struct sockaddr_in6 *c_laddr = (struct sockaddr_in6 *)lsockaddr;
            c_laddr->sin6_family = AF_INET6;
            c_laddr->sin6_addr = in6addr_any;
            c_laddr->sin6_port = ((struct sockaddr_in6 *)rep._iptcp->ai_addr)->sin6_port;
            //        c_laddr->sin6_scope_id; // Not needed to be defined
        } else {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    sock->_fd = socket(rep._iptcp->ai_family, rep._iptcp->ai_socktype, rep._iptcp->ai_protocol);
    if (sock->_fd != -1) {
        int optflag = 1;
        if ((ret == _Z_RES_OK) &&
            (setsockopt(sock->_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&optflag, sizeof(optflag)) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        z_time_t tv;
        tv.tv_sec = tout / (uint32_t)1000;
        tv.tv_usec = (tout % (uint32_t)1000) * (uint32_t)1000;
        if ((ret == _Z_RES_OK) && (setsockopt(sock->_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0)) {
            // FIXME: setting the setsockopt is consistently failing. Commenting it
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            // until further inspection. ret = _Z_ERR_GENERIC;
        }

        if ((ret == _Z_RES_OK) && (bind(sock->_fd, lsockaddr, addrlen) < 0)) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }

        // FIXME: iface passed into the locator is being ignored
        //        default if used instead
        if (ret != _Z_RES_OK) {
            struct net_if *ifa = NULL;
            ifa = net_if_get_default();
            if (ifa != NULL) {
                // Join the multicast group
                if (rep._iptcp->ai_family == AF_INET) {
                    struct net_if_mcast_addr *mcast = NULL;
                    mcast = net_if_ipv4_maddr_add(ifa, &((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_addr);
                    if (!mcast) {
                        _Z_ERROR_LOG(_Z_ERR_GENERIC);
                        ret = _Z_ERR_GENERIC;
                    }
#if KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR > 3 || KERNEL_VERSION_MAJOR >= 4
                    net_if_ipv4_maddr_join(ifa, mcast);
#else
                    net_if_ipv4_maddr_join(mcast);
#endif
                } else if (rep._iptcp->ai_family == AF_INET6) {
                    struct net_if_mcast_addr *mcast = NULL;
                    mcast = net_if_ipv6_maddr_add(ifa, &((struct sockaddr_in6 *)rep._iptcp->ai_addr)->sin6_addr);
                    if (!mcast) {
                        _Z_ERROR_LOG(_Z_ERR_GENERIC);
                        ret = _Z_ERR_GENERIC;
                    }
#if KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR > 3 || KERNEL_VERSION_MAJOR >= 4
                    net_if_ipv6_maddr_join(ifa, mcast);
#else
                    net_if_ipv6_maddr_join(mcast);
#endif
                } else {
                    _Z_ERROR_LOG(_Z_ERR_GENERIC);
                    ret = _Z_ERR_GENERIC;
                }
            } else {
                _Z_ERROR_LOG(_Z_ERR_GENERIC);
                ret = _Z_ERR_GENERIC;
            }
        }

        if (ret != _Z_RES_OK) {
            close(sock->_fd);
            sock->_fd = -1;
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
    _ZP_UNUSED(lep);
    if (sockrecv->_fd >= 0) {
        // FIXME: iface passed into the locator is being ignored
        //        default if used instead
        struct net_if *ifa = NULL;
        ifa = net_if_get_default();
        if (ifa != NULL) {
            struct net_if_mcast_addr *mcast = NULL;
            if (rep._iptcp->ai_family == AF_INET) {
                mcast = net_if_ipv4_maddr_add(ifa, &((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_addr);
                if (mcast != NULL) {
#if KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR > 3 || KERNEL_VERSION_MAJOR >= 4
                    net_if_ipv4_maddr_leave(ifa, mcast);
#else
                    net_if_ipv4_maddr_leave(mcast);
#endif
                    net_if_ipv4_maddr_rm(ifa, &((struct sockaddr_in *)rep._iptcp->ai_addr)->sin_addr);
                } else {
                    // Do nothing. The socket will be closed in any case.
                }
            } else if (rep._iptcp->ai_family == AF_INET6) {
                mcast = net_if_ipv6_maddr_add(ifa, &((struct sockaddr_in6 *)rep._iptcp->ai_addr)->sin6_addr);
                if (mcast != NULL) {
#if KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR > 3 || KERNEL_VERSION_MAJOR >= 4
                    net_if_ipv6_maddr_leave(ifa, mcast);
#else
                    net_if_ipv6_maddr_leave(mcast);
#endif
                    net_if_ipv6_maddr_rm(ifa, &((struct sockaddr_in6 *)rep._iptcp->ai_addr)->sin6_addr);
                } else {
                    // Do nothing. The socket will be closed in any case.
                }
            } else {
                // Do nothing. It must never not enter here.
                // Required to be compliant with MISRA 15.7 rule
            }
        }
    }

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
    socklen_t raddrlen = sizeof(struct sockaddr_storage);

    ssize_t rb = 0;
    do {
        rb = recvfrom(sock._fd, ptr, len, 0, (struct sockaddr *)&raddr, &raddrlen);
        if (rb < (ssize_t)0) {
            rb = SIZE_MAX;
            break;
        }

        if (lep._iptcp->ai_family == AF_INET) {
            struct sockaddr_in *a = ((struct sockaddr_in *)lep._iptcp->ai_addr);
            struct sockaddr_in *b = ((struct sockaddr_in *)&raddr);
            if (!((a->sin_port == b->sin_port) && (a->sin_addr.s_addr == b->sin_addr.s_addr))) {
                // If addr is not NULL, it means that the raddr was requested by the
                // upper-layers
                if (addr != NULL) {
                    addr->len = sizeof(uint32_t) + sizeof(uint16_t);
                    (void)memcpy((uint8_t *)addr->start, &b->sin_addr.s_addr, sizeof(uint32_t));
                    (void)memcpy((uint8_t *)(addr->start + sizeof(uint32_t)), &b->sin_port, sizeof(uint16_t));
                }
                break;
            }
        } else if (lep._iptcp->ai_family == AF_INET6) {
            struct sockaddr_in6 *a = ((struct sockaddr_in6 *)lep._iptcp->ai_addr);
            struct sockaddr_in6 *b = ((struct sockaddr_in6 *)&raddr);
            if (!((a->sin6_port == b->sin6_port) &&
                  (memcmp(a->sin6_addr.s6_addr, b->sin6_addr.s6_addr, sizeof(uint32_t) * 4UL) == 0))) {
                // If addr is not NULL, it means that the raddr was requested by the
                // upper-layers
                if (addr != NULL) {
                    addr->len = (sizeof(uint32_t) * 4UL) + sizeof(uint16_t);
                    (void)memcpy((uint8_t *)addr->start, &b->sin6_addr.s6_addr, sizeof(uint32_t) * 4UL);
                    (void)memcpy((uint8_t *)(addr->start + (sizeof(uint32_t) * 4UL)), &b->sin6_port, sizeof(uint16_t));
                }
                break;
            }
        } else {
            continue;  // FIXME: support error report on invalid packet to the upper
                       // layer
        }
    } while (1);

    return rb;
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
        pos = _z_ptr_u8_offset(pos, rb);
    } while (n != len);

    return n;
}

size_t _z_send_udp_multicast(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len,
                             _z_sys_net_endpoint_t rep) {
    return sendto(sock._fd, ptr, len, 0, rep._iptcp->ai_addr, rep._iptcp->ai_addrlen);
}
#endif  // Z_FEATURE_LINK_UDP_MULTICAST == 1

#if Z_FEATURE_LINK_SERIAL == 1
z_result_t _z_open_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;
    (void)(sock);
    (void)(txpin);
    (void)(rxpin);
    (void)(baudrate);

    // @TODO: To be implemented
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    ret = _Z_ERR_GENERIC;

    return ret;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

/* ── issue 0852: interrupt-driven RX ────────────────────────────────────────
 *
 * The polled path below this block reads with `uart_poll_in` and `k_yield()`s
 * between attempts. That makes byte capture a function of THREAD SCHEDULING: a
 * byte arrives every 87 us at 115200, the hardware FIFO holds a handful, and
 * any scheduling gap longer than that loses data the driver never reports.
 *
 * Raising the reader's priority (which the platform port now honours -- it used
 * to discard it) shrinks the gap but cannot remove it, because the reader still
 * has to be RUNNING to catch a byte. Measured: with the priority fix alone the
 * first action goal completes and later ones still hang.
 *
 * So the bytes are taken in the ISR instead, and the thread reads from a ring
 * buffer that the ISR fills. Capture no longer depends on the scheduler at all
 * -- only on interrupt latency, which is bounded and does not grow with load.
 *
 * ONE link. The buffer is static and single-instance because a zenoh-pico
 * session has one serial transport; a second concurrent serial link would need
 * a table keyed by device, and `_z_serial_rx_bind` refuses rather than
 * silently sharing this one. */

#ifndef _Z_ZEPHYR_SERIAL_RX_RING_BYTES
#define _Z_ZEPHYR_SERIAL_RX_RING_BYTES 1024
#endif

static uint8_t _z_serial_rx_storage[_Z_ZEPHYR_SERIAL_RX_RING_BYTES];
static struct ring_buf _z_serial_rx_ring;
static K_SEM_DEFINE(_z_serial_rx_sem, 0, 1);
static const struct device *_z_serial_rx_dev = NULL;
/* Sticky, ISR-set, thread-cleared: a byte the ring had no room for. Distinct
   from a UART overrun -- this one is OUR backlog, not the hardware's. */
static volatile bool _z_serial_rx_ring_full = false;

static void _z_serial_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev) == 0) {
            break;
        }
        /* Drain the hardware FIFO in as few calls as possible: the mcux LPUART
           `fifo_read` loops while the RX register is full, so a large ask is
           one call rather than one per byte. */
        uint8_t chunk[32];
        int n = uart_fifo_read(dev, chunk, sizeof(chunk));
        if (n <= 0) {
            break;
        }
        uint32_t put = ring_buf_put(&_z_serial_rx_ring, chunk, (uint32_t)n);
        if (put < (uint32_t)n) {
            _z_serial_rx_ring_full = true;
        }
        k_sem_give(&_z_serial_rx_sem);
    }
}

/* Attach the ISR to `dev`. Idempotent for the SAME device so a reconnect does
   not have to tear down; refuses a second, different device rather than
   quietly rebinding the one link the buffer can represent. */
static bool _z_serial_rx_bind(const struct device *dev) {
    if (_z_serial_rx_dev == dev) {
        return true;
    }
    if (_z_serial_rx_dev != NULL) {
        _Z_ERROR("serial RX already bound to another UART -- refusing to rebind");
        return false;
    }
    ring_buf_init(&_z_serial_rx_ring, sizeof(_z_serial_rx_storage), _z_serial_rx_storage);
    if (uart_irq_callback_user_data_set(dev, _z_serial_isr, NULL) != 0) {
        _Z_ERROR("uart_irq_callback_user_data_set failed -- falling back to polled RX");
        return false;
    }
    _z_serial_rx_dev = dev;
    uart_irq_rx_enable(dev);
    return true;
}

/* One byte from the ring, waiting up to `deadline` (absolute k_uptime_get ms).
   Returns 0 on success, -1 on timeout. */
static int _z_serial_rx_get(uint8_t *out, int64_t deadline) {
    for (;;) {
        if (ring_buf_get(&_z_serial_rx_ring, out, 1) == 1) {
            return 0;
        }
        int64_t remaining = deadline - k_uptime_get();
        if (remaining <= 0) {
            return -1;
        }
        /* The sem is a "something arrived" hint, not a count of bytes: the ISR
           gives it once per FIFO drain, which may carry many bytes or race a
           reader that already took them. So a wake is always re-checked against
           the ring, and a timeout here is not by itself an error. */
        (void)k_sem_take(&_z_serial_rx_sem, K_MSEC((uint32_t)remaining));
    }
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

z_result_t _z_open_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;

    sock->_serial = device_get_binding(dev);
    if (sock->_serial != NULL) {
        const struct uart_config config = {
            .baudrate = baudrate,
            .parity = UART_CFG_PARITY_NONE,        // Default in Zenoh Rust
            .stop_bits = UART_CFG_STOP_BITS_1,     // Default in Zenoh Rust
            .data_bits = UART_CFG_DATA_BITS_8,     // Default in Zenoh Rust
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,  // Default in Zenoh Rust
        };
        if (uart_configure(sock->_serial, &config) != 0) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
        /* issue 0852 -- bind BEFORE the handshake in `_z_connect_serial`
           below, so the peer's reply is caught by the ISR rather than raced
           for by the poll loop. A failure here is not fatal: the reader falls
           back to polling, which is what every build did before. */
        if (ret == _Z_RES_OK) {
            (void)_z_serial_rx_bind(sock->_serial);
        }
#endif
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return (ret == _Z_RES_OK ? _z_connect_serial(*sock) : ret);
}

z_result_t _z_listen_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;
    (void)(sock);
    (void)(txpin);
    (void)(rxpin);
    (void)(baudrate);

    // @TODO: To be implemented
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    ret = _Z_ERR_GENERIC;

    return ret;
}

z_result_t _z_listen_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;
    (void)(sock);
    (void)(dev);
    (void)(baudrate);

    // @TODO: To be implemented
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    ret = _Z_ERR_GENERIC;

    return ret;
}

void _z_close_serial(_z_sys_net_socket_t *sock) {}

/* Per-byte read timeout. Long enough for a peer to answer a handshake, short
   enough that a lost frame is recoverable rather than a permanent hang. */
#ifndef _Z_ZEPHYR_SERIAL_TIMEOUT_MS
#define _Z_ZEPHYR_SERIAL_TIMEOUT_MS 1000
#endif


size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    uint8_t *raw_buf = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);
    if (raw_buf == NULL) {
        _Z_ERROR("Failed to allocate serial COBS buffer");
        return SIZE_MAX;
    }
    size_t rb = 0;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
    const bool irq_rx = (_z_serial_rx_dev != NULL && _z_serial_rx_dev == sock._serial);
#else
    const bool irq_rx = false;
#endif
    for (size_t i = 0; i < _Z_SERIAL_MAX_COBS_BUF_SIZE; i++) {
        int res = -1;
        /* Bounded per-byte wait, as the flipper port does with a 200 ms
           furi_stream_buffer_receive() and the unix port with SO_RCVTIMEO: on
           expiry return SIZE_MAX, which is the contract callers already handle
           (_z_read_exact_serial breaks on it). Unbounded was the real defect --
           _z_connect_serial sends one INIT and then blocks here forever, so a
           lost INIT could never be retried and the link was unrecoverable.
           Yield rather than sleep between polls: at 115200 a byte is 87 us and
           the shortest z_sleep_ms(1) blocks a whole tick, overrunning the UART. */
        int64_t deadline = k_uptime_get() + (int64_t)_Z_ZEPHYR_SERIAL_TIMEOUT_MS;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
        if (irq_rx) {
            /* The ISR already took the byte off the wire; this only moves it.
               No `k_yield()`, so no dependence on how long the executor holds
               the CPU -- that dependence was issue 0852. */
            if (_z_serial_rx_get(&raw_buf[i], deadline) != 0) {
                z_free(raw_buf);
                return SIZE_MAX;
            }
            res = 0;
        } else
#endif
        {
            while (res != 0) {
                res = uart_poll_in(sock._serial, &raw_buf[i]);
                if (res != 0) {
                    if (k_uptime_get() > deadline) {
                        z_free(raw_buf);
                        return SIZE_MAX;
                    }
                    k_yield();
                }
            }
        }

        /* issue 0852 -- an overrun is a LOST BYTE, and it used to be silent.
           `uart_poll_in` reports "no character available", never "a character
           was destroyed before you asked", so a receiver that only checks its
           return value cannot tell a quiet link from a link it is failing to
           keep up with. That indistinguishability is what let this hide behind
           six wrong hypotheses, ending in a router bug report (issue 0848) that
           was wrong in every particular.

           Checked per byte rather than per frame because the flag is sticky
           until read: reading it here attributes the loss to the frame that
           was actually damaged, which is what made the diagnosis provable. */
        int uart_err = uart_err_check(sock._serial);
        if (uart_err > 0 && (uart_err & UART_ERROR_OVERRUN) != 0) {
            _Z_ERROR("serial RX OVERRUN after %zu byte(s) -- frame will be dropped", rb);
        }
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
        /* A DIFFERENT loss from the one above, and worth keeping apart: the
           hardware kept up and our ring did not, which says the ring is too
           small or the reader is too slow, not that the ISR is too late. */
        if (_z_serial_rx_ring_full) {
            _z_serial_rx_ring_full = false;
            _Z_ERROR("serial RX ring overflowed -- raise _Z_ZEPHYR_SERIAL_RX_RING_BYTES");
        }
#endif

        rb++;
        if (raw_buf[i] == (uint8_t)0x00) {
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
        z_free(raw_buf);
        z_free(tmp_buf);
        return SIZE_MAX;
    }
    size_t ret =
        _z_serial_msg_serialize(raw_buf, _Z_SERIAL_MAX_COBS_BUF_SIZE, ptr, len, header, tmp_buf, _Z_SERIAL_MFS_SIZE);

    if (ret == SIZE_MAX) {
        /* Both buffers leaked here: ~3 KiB per failed send, and a link that is
           retrying a handshake fails repeatedly. */
        z_free(raw_buf);
        z_free(tmp_buf);
        return ret;
    }

    for (size_t i = 0; i < ret; i++) {
        uart_poll_out(sock._serial, raw_buf[i]);
    }

    z_free(raw_buf);
    z_free(tmp_buf);

    return len;
}
#endif

#if Z_FEATURE_LINK_BLUETOOTH == 1
#error "Bluetooth not supported yet on Zephyr port of Zenoh-Pico"
#endif

#if Z_FEATURE_RAWETH_TRANSPORT == 1
#error "Raw ethernet transport not supported yet on Zephyr port of Zenoh-Pico"
#endif

/*------------------ CAN sockets ------------------*/
// RFC-0080 / phase-377. Zephyr CAN controller driver.
#if Z_FEATURE_LINK_CAN == 1

// Per-link receive queues, and per-device start refcounts.
//
// Both are pools rather than single globals because several zenoh sessions can
// share one CAN controller. A shared queue would let one link dequeue and drop
// a frame belonging to another; a shared start/stop would let one link's close
// stop the controller under the others.
#ifndef Z_CAN_RX_QUEUE_DEPTH
#define Z_CAN_RX_QUEUE_DEPTH 16
#endif

// Links per image. Each costs a queue of Z_CAN_RX_QUEUE_DEPTH frames.
#ifndef Z_CAN_MAX_LINKS
#define Z_CAN_MAX_LINKS 2
#endif

static struct k_msgq _z_can_msgq_pool[Z_CAN_MAX_LINKS];
static char _z_can_msgq_buf[Z_CAN_MAX_LINKS][Z_CAN_RX_QUEUE_DEPTH * sizeof(struct can_frame)];
static bool _z_can_msgq_taken[Z_CAN_MAX_LINKS];

// Devices this image has started, with a refcount each.
static const struct device *_z_can_started_dev[Z_CAN_MAX_LINKS];
static uint8_t _z_can_started_ref[Z_CAN_MAX_LINKS];

// One mutex guards both tables. Open and close are rare and off the data path,
// so a single lock costs nothing and avoids a torn refcount when two sessions
// come up concurrently.
K_MUTEX_DEFINE(_z_can_pool_mutex);

static struct k_msgq *__z_can_msgq_take(void) {
    struct k_msgq *out = NULL;
    k_mutex_lock(&_z_can_pool_mutex, K_FOREVER);
    for (size_t i = 0; i < Z_CAN_MAX_LINKS; i++) {
        if (!_z_can_msgq_taken[i]) {
            _z_can_msgq_taken[i] = true;
            k_msgq_init(&_z_can_msgq_pool[i], _z_can_msgq_buf[i], sizeof(struct can_frame), Z_CAN_RX_QUEUE_DEPTH);
            out = &_z_can_msgq_pool[i];
            break;
        }
    }
    k_mutex_unlock(&_z_can_pool_mutex);
    if (out == NULL) {
        _Z_ERROR("CAN: no free receive queue (Z_CAN_MAX_LINKS=%d)", (int)Z_CAN_MAX_LINKS);
    }
    return out;
}

static void __z_can_msgq_give(struct k_msgq *q) {
    if (q == NULL) {
        return;
    }
    k_mutex_lock(&_z_can_pool_mutex, K_FOREVER);
    for (size_t i = 0; i < Z_CAN_MAX_LINKS; i++) {
        if (&_z_can_msgq_pool[i] == q) {
            k_msgq_purge(q);
            _z_can_msgq_taken[i] = false;
            break;
        }
    }
    k_mutex_unlock(&_z_can_pool_mutex);
}

// Start the controller on the first link to claim it; later links just take a
// reference. Returns false if it could not be started.
static bool __z_can_dev_acquire(const struct device *dev) {
    bool ok = true;
    k_mutex_lock(&_z_can_pool_mutex, K_FOREVER);
    for (size_t i = 0; i < Z_CAN_MAX_LINKS; i++) {
        if (_z_can_started_dev[i] == dev) {
            _z_can_started_ref[i]++;
            k_mutex_unlock(&_z_can_pool_mutex);
            return true;  // already running, do not restart it under the others
        }
    }
    for (size_t i = 0; i < Z_CAN_MAX_LINKS; i++) {
        if (_z_can_started_dev[i] == NULL) {
            if (can_start(dev) < 0) {
                ok = false;
            } else {
                _z_can_started_dev[i] = dev;
                _z_can_started_ref[i] = 1;
            }
            k_mutex_unlock(&_z_can_pool_mutex);
            return ok;
        }
    }
    k_mutex_unlock(&_z_can_pool_mutex);
    _Z_ERROR("CAN: no free device slot (Z_CAN_MAX_LINKS=%d)", (int)Z_CAN_MAX_LINKS);
    return false;
}

// Stop only when the last link on this device goes away.
static void __z_can_dev_release(const struct device *dev) {
    k_mutex_lock(&_z_can_pool_mutex, K_FOREVER);
    for (size_t i = 0; i < Z_CAN_MAX_LINKS; i++) {
        if (_z_can_started_dev[i] == dev) {
            if (_z_can_started_ref[i] > 0) {
                _z_can_started_ref[i]--;
            }
            if (_z_can_started_ref[i] == 0) {
                (void)can_stop(dev);
                _z_can_started_dev[i] = NULL;
            }
            break;
        }
    }
    k_mutex_unlock(&_z_can_pool_mutex);
}

// True when some link has already started this controller, in which case its
// mode and bit rates must not be touched.
static bool __z_can_dev_in_use(const struct device *dev) {
    bool in_use = false;
    k_mutex_lock(&_z_can_pool_mutex, K_FOREVER);
    for (size_t i = 0; i < Z_CAN_MAX_LINKS; i++) {
        if ((_z_can_started_dev[i] == dev) && (_z_can_started_ref[i] > 0)) {
            in_use = true;
            break;
        }
    }
    k_mutex_unlock(&_z_can_pool_mutex);
    return in_use;
}

static z_result_t __z_can_setup(_z_can_socket_t *sock, const char *dev_name, uint32_t bitrate, uint32_t dbitrate,
                                uint32_t id, uint32_t match, uint32_t mask) {
    const struct device *dev = device_get_binding(dev_name);
    if (dev == NULL) {
        _Z_ERROR("CAN: no device '%s'", dev_name);
        return _Z_ERR_GENERIC;
    }
    if (!device_is_ready(dev)) {
        _Z_ERROR("CAN: device '%s' not ready", dev_name);
        return _Z_ERR_GENERIC;
    }

    bool shared = __z_can_dev_in_use(dev);
    _Bool fd_mode = false;

    if (shared) {
        // Another link already configured and started this controller. Read the
        // mode back rather than reconfiguring, which would disrupt it.
#ifdef CONFIG_CAN_FD_MODE
        can_mode_t mode = 0;
        if (can_get_capabilities(dev, &mode) == 0) {
            fd_mode = ((mode & CAN_MODE_FD) != 0);
        }
#endif
        _Z_DEBUG("CAN: '%s' already in use, inheriting its configuration", dev_name);
    } else {
        // A controller already started cannot be reconfigured; stopping first is
        // harmless if it was never started.
        (void)can_stop(dev);

        // CONFIG_CAN_FD_MODE gates more than performance: without it
        // `z_impl_can_set_bitrate_data` is not compiled at all, and
        // `struct can_frame.data` is 8 bytes rather than 64. Everything FD has
        // to be behind this, not behind a runtime flag.
#ifdef CONFIG_CAN_FD_MODE
        if (dbitrate != 0u) {
            can_mode_t caps = 0;
            if ((can_get_capabilities(dev, &caps) == 0) && ((caps & CAN_MODE_FD) != 0)) {
                if (can_set_mode(dev, CAN_MODE_FD) == 0) {
                    fd_mode = true;
                }
            }
            if (!fd_mode) {
                _Z_DEBUG("CAN: '%s' has no CAN FD, using classic frames", dev_name);
            }
        }
#else
        if (dbitrate != 0u) {
            _Z_DEBUG("CAN: built without CONFIG_CAN_FD_MODE, using classic frames");
        }
#endif

        // Bit rates may be fixed by devicetree. Honour the contract in
        // system/link/can.h: a rate we cannot apply is not a failure.
        if (bitrate != 0u) {
            (void)can_set_bitrate(dev, bitrate);
        }
#ifdef CONFIG_CAN_FD_MODE
        if (fd_mode && (dbitrate != 0u)) {
            (void)can_set_bitrate_data(dev, dbitrate);
        }
#endif
    }

    struct k_msgq *rx = __z_can_msgq_take();
    if (rx == NULL) {
        return _Z_ERR_GENERIC;
    }

    // Admit the whole zenoh band; the read drops our own frames. A mask of 0
    // matches every identifier, which is the default for a dedicated bus.
    const struct can_filter filter = {
        .id = match,
        .mask = mask,
        .flags = 0,
    };
    int filter_id = can_add_rx_filter_msgq(dev, rx, &filter);
    if (filter_id < 0) {
        _Z_ERROR("CAN: could not add rx filter (match 0x%x mask 0x%x)", (unsigned)match, (unsigned)mask);
        __z_can_msgq_give(rx);
        return _Z_ERR_GENERIC;
    }

    if (!__z_can_dev_acquire(dev)) {
        _Z_ERROR("CAN: could not start '%s'", dev_name);
        can_remove_rx_filter(dev, filter_id);
        __z_can_msgq_give(rx);
        return _Z_ERR_GENERIC;
    }

    sock->_sock._can_dev = dev;
    sock->_sock._can_rx_msgq = rx;
    sock->_sock._can_filter_id = filter_id;
    sock->_id = id;
    sock->_match = match;
    sock->_mask = mask;
    sock->_fd_mode = fd_mode;
    sock->_mtu = fd_mode ? _Z_CAN_FD_MTU_SIZE : _Z_CAN_CLASSIC_MTU_SIZE;
    return _Z_RES_OK;
}

z_result_t _z_open_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint32_t id,
                       uint32_t match, uint32_t mask) {
    return __z_can_setup(sock, dev, bitrate, dbitrate, id, match, mask);
}

z_result_t _z_listen_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint32_t id,
                         uint32_t match, uint32_t mask) {
    // Multicast peers all listen; a bus has no connection setup.
    return __z_can_setup(sock, dev, bitrate, dbitrate, id, match, mask);
}

void _z_close_can(_z_can_socket_t *sock) {
    const struct device *dev = (const struct device *)sock->_sock._can_dev;
    if (dev == NULL) {
        return;
    }
    // Give back the controller's filter slot; there are only a handful, and
    // leaking one per open/close cycle exhausts them.
    if (sock->_sock._can_filter_id >= 0) {
        can_remove_rx_filter(dev, sock->_sock._can_filter_id);
        sock->_sock._can_filter_id = -1;
    }
    __z_can_msgq_give(sock->_sock._can_rx_msgq);
    sock->_sock._can_rx_msgq = NULL;
    // Stops the controller only if this was the last link using it.
    __z_can_dev_release(dev);
    sock->_sock._can_dev = NULL;
}

size_t _z_send_can(const _z_can_socket_t *sock, const uint8_t *ptr, size_t len) {
    if (len > sock->_mtu) {
        _Z_ERROR("CAN: datagram %zu exceeds link MTU %u", len, (unsigned)sock->_mtu);
        return SIZE_MAX;
    }

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = sock->_id;
    // Byte 0 is the true length; the DLC may describe a longer frame because
    // CAN FD lengths are quantised (system/link/can.h).
    frame.data[0] = (uint8_t)len;
    if (len > 0) {
        memcpy(&frame.data[1], ptr, len);
    }
    frame.dlc = can_bytes_to_dlc((uint8_t)(len + _Z_CAN_LEN_PREFIX));
#ifdef CONFIG_CAN_FD_MODE
    if (sock->_fd_mode) {
        frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;
    }
#endif

    if (can_send((const struct device *)sock->_sock._can_dev, &frame, K_MSEC(100), NULL, NULL) != 0) {
        return SIZE_MAX;
    }
    return len;
}

size_t _z_read_can(const _z_can_socket_t *sock, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    struct can_frame frame;

    // Loop until a frame arrives that is not ours, mirroring
    // `_z_read_udp_multicast`: on a bus every peer hears everything.
    for (;;) {
        if (k_msgq_get(sock->_sock._can_rx_msgq, &frame, K_FOREVER) != 0) {
            return SIZE_MAX;
        }

        uint32_t sender = frame.id & CAN_EXT_ID_MASK;
        if (sender == sock->_id) {
            continue;  // our own frame
        }
        if ((sock->_mask != 0u) && ((sender & sock->_mask) != sock->_match)) {
            continue;  // outside the zenoh band
        }

        uint8_t frame_len = can_dlc_to_bytes(frame.dlc);
        if (frame_len < _Z_CAN_LEN_PREFIX) {
            continue;
        }

        size_t dlen = frame.data[0];
        if ((dlen > (size_t)(frame_len - _Z_CAN_LEN_PREFIX)) || (dlen > len)) {
            // Length byte disagrees with the frame, or the caller's buffer is
            // too small. Drop rather than deliver a truncated datagram.
            _Z_ERROR("CAN: bad datagram length %zu (frame %u, buffer %zu)", dlen, (unsigned)frame_len, len);
            continue;
        }

        if (addr != NULL) {
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
