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

// nano-ros (issue 2389): a BSD-socket `send()` of a whole buffer that gives up
// when the peer stops taking bytes. ONE spelling for every port that speaks BSD
// sockets with `poll` (tcp_posix.c: Linux, macOS, BSD, NuttX; tcp_zephyr.c:
// Zephyr's own stack and native_sim's host sockets). Each includes it after its
// own socket headers, so the names below resolve to that port's calls.
//
// A blocking `send()` to a peer that stops reading waits until the peer reads
// again, which may be never: the socket stays open, the window stays zero, and
// the writer holds zenoh-pico's TX mutex the whole time, so every other sender
// (publishers, keep-alives, the close that the lease would send) blocks behind
// it. On Zephyr it also holds the socket's per-fd lock, so the read task cannot
// even receive. Measured on native_sim: the session went silent for good, with
// nothing on the console.
//
// So each send here is non-blocking, and the wait for room is a `poll` bounded
// by one socket timeout (Z_CONFIG_SOCKET_TIMEOUT), which on Zephyr releases the
// per-fd lock between attempts. Progress resets the clock; when the socket takes
// no byte for Z_TRANSPORT_TX_WAIT_BEFORE_CLOSE milliseconds the write fails.
// It never returns a partial count: on a stream link the bytes already sent
// cannot be taken back, so the caller has to close the link anyway, and a
// failure says that.

#ifndef ZENOH_PICO_LINK_TRANSPORT_TCP_WRITE_WITHIN_H
#define ZENOH_PICO_LINK_TRANSPORT_TCP_WRITE_WITHIN_H

#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"

/* Returns `len` when every byte was accepted, SIZE_MAX otherwise (errno says
 * why; ETIMEDOUT when the peer took no byte for the whole wait). `flags` are
 * the port's own send flags (MSG_NOSIGNAL where it has one). */
static inline size_t _z_tcp_write_within(int fd, const uint8_t *ptr, size_t len, int flags) {
    size_t sent = 0;
    z_clock_t last_progress = z_clock_now();
    bool polled_ready = false;
    while (sent < len) {
        ssize_t wb = send(fd, &ptr[sent], len - sent, flags | MSG_DONTWAIT);
        if (wb > 0) {
            sent += (size_t)wb;
            last_progress = z_clock_now();
            polled_ready = false;
            continue;
        }
        if ((wb < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR)) {
            return SIZE_MAX;
        }
        unsigned long stalled = z_clock_elapsed_ms(&last_progress);
        if (stalled >= (unsigned long)Z_TRANSPORT_TX_WAIT_BEFORE_CLOSE) {
            errno = ETIMEDOUT;
            return SIZE_MAX;
        }
        if (polled_ready) {
            // The stack said writable and still refused the bytes (a stack
            // whose POLLOUT does not track the send window): do not spin.
            z_sleep_ms(1);
        }
        unsigned long wait_ms = (unsigned long)Z_TRANSPORT_TX_WAIT_BEFORE_CLOSE - stalled;
        if (wait_ms > (unsigned long)Z_CONFIG_SOCKET_TIMEOUT) {
            wait_ms = (unsigned long)Z_CONFIG_SOCKET_TIMEOUT;
        }
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLOUT;
        pfd.revents = 0;
        // An error or hang-up shows on the next send, which then fails.
        polled_ready = (poll(&pfd, 1, (int)wait_ms) > 0);
    }
    return sent;
}

#endif /* ZENOH_PICO_LINK_TRANSPORT_TCP_WRITE_WITHIN_H */
