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

// nano-ros (issue 2437): a BSD-socket `connect()` that gives up after a
// timeout. ONE spelling for every port that speaks BSD sockets with `fcntl`
// and `poll` (tcp_posix.c: Linux, macOS, BSD, NuttX; tcp_zephyr.c: Zephyr's
// own stack and native_sim's host sockets). Each includes it after its own
// socket headers, so the names below resolve to that port's calls.
//
// A blocking connect to a peer that drops the SYN waits for the stack's own
// SYN-retry budget (Linux: `tcp_syn_retries` = 6, about 127 s), whatever the
// caller's connect budget is. This switches the socket to non-blocking for
// the connect only, waits for writability with `poll` bounded by the timeout,
// reads the outcome from SO_ERROR, and restores the socket's flags.

#ifndef ZENOH_PICO_LINK_TRANSPORT_TCP_CONNECT_WITHIN_H
#define ZENOH_PICO_LINK_TRANSPORT_TCP_CONNECT_WITHIN_H

#include <errno.h>
#include <fcntl.h>
#include <poll.h>

/* Returns 0 when connected, -1 otherwise (errno says why; ETIMEDOUT when the
 * timeout ran out). A negative `timeout_ms` is a plain blocking connect. */
static inline int _z_tcp_connect_within(int fd, const struct sockaddr *addr, socklen_t addrlen, int32_t timeout_ms) {
    if (timeout_ms < 0) {
        return connect(fd, addr, addrlen);
    }
    int flags = fcntl(fd, F_GETFL, 0);
    if ((flags < 0) || (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)) {
        // No non-blocking mode: the connect keeps the stack's own bound.
        return connect(fd, addr, addrlen);
    }

    int rc = connect(fd, addr, addrlen);
    if ((rc < 0) && ((errno == EINPROGRESS) || (errno == EAGAIN) || (errno == EALREADY))) {
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLOUT;
        pfd.revents = 0;
        int pr = poll(&pfd, 1, (int)timeout_ms);
        if (pr == 1) {
            int err = 0;
            socklen_t errlen = (socklen_t)sizeof(err);
            if ((getsockopt(fd, SOL_SOCKET, SO_ERROR, (void *)&err, &errlen) == 0) && (err == 0)) {
                rc = 0;
            } else {
                rc = -1;
                if (err != 0) {
                    errno = err;
                }
            }
        } else {
            rc = -1;
            if (pr == 0) {
                errno = ETIMEDOUT;
            }
        }
    }

    int saved = errno;
    (void)fcntl(fd, F_SETFL, flags);
    errno = saved;
    return rc;
}

#endif /* ZENOH_PICO_LINK_TRANSPORT_TCP_CONNECT_WITHIN_H */
