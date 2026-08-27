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

#include "zenoh-pico/link/transport/isotp.h"

// Same ordering rule as SocketCAN: needs Z_FEATURE_LINK_ISOTP defined.
// Z_FEATURE_LINK_ISOTP_VENDORED selects isotp_vendored_linux.c instead of
// everything below -- see the comment at the top of that file for why `unix`
// implements this link twice.
#if defined(ZENOH_LINUX) && Z_FEATURE_LINK_ISOTP == 1 && Z_FEATURE_LINK_ISOTP_VENDORED == 0

#include <errno.h>
#include <linux/can.h>
#include <linux/can/isotp.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "zenoh-pico/utils/logging.h"

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
#endif  // defined(ZENOH_LINUX) && Z_FEATURE_LINK_ISOTP == 1 && Z_FEATURE_LINK_ISOTP_VENDORED == 0
