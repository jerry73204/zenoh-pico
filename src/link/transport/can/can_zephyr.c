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

#include "zenoh-pico/link/transport/can.h"

// After the zenoh includes, so Z_FEATURE_LINK_CAN is defined.
#if defined(ZENOH_ZEPHYR) && Z_FEATURE_LINK_CAN == 1

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include "zenoh-pico/utils/logging.h"

/*------------------ CAN sockets ------------------*/
// RFC-0080 / phase-377. Zephyr CAN controller driver.

// Received frames land here from the driver's RX thread and are drained by
// _z_read_can. Depth is a compromise: deep enough that a fragmented message
// is not dropped while the reader is busy, shallow enough to be affordable on
// the parts this link targets.
#ifndef Z_CAN_RX_QUEUE_DEPTH
#define Z_CAN_RX_QUEUE_DEPTH 16
#endif

K_MSGQ_DEFINE(_z_can_rx_msgq, sizeof(struct can_frame), Z_CAN_RX_QUEUE_DEPTH, 4);

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

    // A controller already started cannot be reconfigured; stopping first is
    // harmless if it was never started.
    (void)can_stop(dev);

    _Bool fd_mode = false;
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

    // Bit rates may be fixed by devicetree. Honour the contract in
    // link/transport/can.h: a rate we cannot apply is not a failure.
    if (bitrate != 0u) {
        (void)can_set_bitrate(dev, bitrate);
    }
    if (fd_mode && (dbitrate != 0u)) {
        (void)can_set_bitrate_data(dev, dbitrate);
    }

    // Admit the whole zenoh band; the read drops our own frames. A mask of 0
    // matches every identifier, which is the default for a dedicated bus.
    const struct can_filter filter = {
        .id = match,
        .mask = mask,
        .flags = 0,
    };
    if (can_add_rx_filter_msgq(dev, &_z_can_rx_msgq, &filter) < 0) {
        _Z_ERROR("CAN: could not add rx filter (match 0x%x mask 0x%x)", (unsigned)match, (unsigned)mask);
        return _Z_ERR_GENERIC;
    }

    if (can_start(dev) < 0) {
        _Z_ERROR("CAN: could not start '%s'", dev_name);
        return _Z_ERR_GENERIC;
    }

    sock->_sock._can_dev = dev;
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
    if (sock->_sock._can_dev != NULL) {
        (void)can_stop((const struct device *)sock->_sock._can_dev);
        sock->_sock._can_dev = NULL;
    }
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
    // CAN FD lengths are quantised (link/transport/can.h).
    frame.data[0] = (uint8_t)len;
    if (len > 0) {
        memcpy(&frame.data[1], ptr, len);
    }
    frame.dlc = can_bytes_to_dlc((uint8_t)(len + _Z_CAN_LEN_PREFIX));
    if (sock->_fd_mode) {
        frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;
    }

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
        if (k_msgq_get(&_z_can_rx_msgq, &frame, K_FOREVER) != 0) {
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

#endif  // defined(ZENOH_ZEPHYR) && Z_FEATURE_LINK_CAN == 1
