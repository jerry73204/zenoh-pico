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
//
// nano-ros (B46): a signal is not a failed connect. A `connect()` interrupted
// by a signal handler returns EINTR while the attempt goes on in the kernel,
// and Linux never restarts it on a socket with SO_RCVTIMEO set, whatever
// SA_RESTART says. The FreeRTOS POSIX port's tick is a SIGALRM installed
// without SA_RESTART, so a tick landing inside the connect failed a
// one-attempt open at once, more often the busier the host. EINTR from
// `connect()` is now "still connecting", the same as EINPROGRESS, and EINTR
// from `poll()` waits again for the time left. The caller's budget and its
// attempt count are unchanged: this finishes the same attempt.

#ifndef ZENOH_PICO_LINK_TRANSPORT_TCP_CONNECT_WITHIN_H
#define ZENOH_PICO_LINK_TRANSPORT_TCP_CONNECT_WITHIN_H

#include <errno.h>
#include <fcntl.h>
#include <poll.h>

/* Wait for an in-progress connect on `fd` to finish, for at most `timeout_ms`
 * (negative: no bound of ours, only the stack's). Returns 0 when connected,
 * -1 otherwise (errno says why; ETIMEDOUT when the time ran out). A signal
 * interrupting the wait does not end it. */
static inline int _z_tcp_connect_wait(int fd, int32_t timeout_ms) {
    z_clock_t start = z_clock_now();
    for (;;) {
        int wait_ms = -1;
        if (timeout_ms >= 0) {
            unsigned long elapsed = z_clock_elapsed_ms(&start);
            wait_ms = (elapsed >= (unsigned long)timeout_ms) ? 0 : (int)((unsigned long)timeout_ms - elapsed);
        }
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLOUT;
        pfd.revents = 0;
        int pr = poll(&pfd, 1, wait_ms);
        if ((pr < 0) && (errno == EINTR)) {
            continue;
        }
        if (pr == 1) {
            int err = 0;
            socklen_t errlen = (socklen_t)sizeof(err);
            if ((getsockopt(fd, SOL_SOCKET, SO_ERROR, (void *)&err, &errlen) == 0) && (err == 0)) {
                return 0;
            }
            if (err != 0) {
                errno = err;
            }
            return -1;
        }
        if (pr == 0) {
            errno = ETIMEDOUT;
        }
        return -1;
    }
}

/* Is `errno` after a `connect()` an attempt still going on? */
static inline int _z_tcp_connect_in_progress(int err) {
    return (err == EINPROGRESS) || (err == EAGAIN) || (err == EALREADY) || (err == EINTR);
}

/* Returns 0 when connected, -1 otherwise (errno says why; ETIMEDOUT when the
 * timeout ran out). A negative `timeout_ms` is a blocking connect, bounded by
 * the stack alone. */
static inline int _z_tcp_connect_within(int fd, const struct sockaddr *addr, socklen_t addrlen, int32_t timeout_ms) {
    if (timeout_ms < 0) {
        int rc = connect(fd, addr, addrlen);
        if ((rc < 0) && (errno == EINTR)) {
            rc = _z_tcp_connect_wait(fd, -1);
        }
        return rc;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    if ((flags < 0) || (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)) {
        // No non-blocking mode: the connect keeps the stack's own bound.
        int rc = connect(fd, addr, addrlen);
        if ((rc < 0) && (errno == EINTR)) {
            rc = _z_tcp_connect_wait(fd, -1);
        }
        return rc;
    }

    int rc = connect(fd, addr, addrlen);
    if ((rc < 0) && _z_tcp_connect_in_progress(errno)) {
        rc = _z_tcp_connect_wait(fd, timeout_ms);
    }

    int saved = errno;
    (void)fcntl(fd, F_SETFL, flags);
    errno = saved;
    return rc;
}

#endif /* ZENOH_PICO_LINK_TRANSPORT_TCP_CONNECT_WITHIN_H */
