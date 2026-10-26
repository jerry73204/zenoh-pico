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
//   nano-ros RFC-0109 D10 -- a CAN controller is started by layer 1.
//

#ifndef ZENOH_PICO_SYSTEM_LINK_CAN_ZEPHYR_H
#define ZENOH_PICO_SYSTEM_LINK_CAN_ZEPHYR_H

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#include "zenoh-pico/utils/logging.h"

// Both Zephyr CAN links -- the multicast CAN link and the ISO-TP unicast one --
// ASSUME a started controller. Starting it, and choosing its bit rates and
// mode, belong to whoever owns the device: devicetree states the controller
// and its `bus-speed`, and the board (nano-ros's Zephyr board glue, or the
// application) calls `can_start()` before any session opens.
//
// The links used to start the controller themselves, refcounted per device.
// That made the link the owner of a device several things share: a link's
// close stopped the bus under anything else on it, the multicast link
// re-applied bit rates from its locator, and once a board started the
// controller first, `can_start()`'s -EALREADY failed the multicast open.
//
// A stopped controller must fail LOUDLY here. A send on one does not report an
// error the link can see: `isotp_send` queues the first frame, nothing reaches
// the bus, and the only symptom is `Reception of next FC has timed out` while
// `candump` shows an idle bus.
static inline bool _z_zephyr_can_is_started(const struct device *dev, const char *link, const char *name) {
    enum can_state state;
    int rc = can_get_state(dev, &state, NULL);
    if (rc != 0) {
        _Z_ERROR("%s: cannot read the state of CAN controller '%s': %d", link, name, rc);
        return false;
    }
    if (state == CAN_STATE_STOPPED) {
        _Z_ERROR("%s: CAN controller '%s' is not started. The board starts it (can_start) before a session opens.",
                 link, name);
        return false;
    }
    return true;
}

#endif /* ZENOH_PICO_SYSTEM_LINK_CAN_ZEPHYR_H */
