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

#ifndef ZENOH_PICO_MULTICAST_TRANSPORT_H
#define ZENOH_PICO_MULTICAST_TRANSPORT_H

#include "zenoh-pico/api/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The batch size this peer's JOIN advertises on `zl`, and the one it requires a
 * peer's JOIN to state.
 *
 * Z_BATCH_MULTICAST_SIZE, unless the link's MTU is a hard frame ceiling
 * (`_cap._mtu_is_frame_ceiling`, a CAN frame) smaller than it: then no batch
 * larger than one frame can ever be sent, and advertising the constant would
 * describe a batch the link cannot carry. zenoh-rs advertises
 * min(batch, link MTU) and, like zenoh-pico, refuses a JOIN whose batch size is
 * not its own -- so on CAN FD both must state 63. On every other link this is
 * the constant, unchanged: UDP's 1450-byte MTU is a size zenoh-pico chose, and
 * a pico built with Z_BATCH_MULTICAST_SIZE=8192 to meet zenoh-rs over UDP must
 * keep advertising 8192.
 */
static inline uint16_t _z_multicast_join_batch_size(const _z_link_t *zl) {
    if ((zl != NULL) && zl->_cap._mtu_is_frame_ceiling && (zl->_mtu < Z_BATCH_MULTICAST_SIZE)) {
        return zl->_mtu;
    }
    return Z_BATCH_MULTICAST_SIZE;
}

z_result_t _z_multicast_transport_create(_z_transport_t *zt, _z_link_t *zl,
                                         _z_transport_multicast_establish_param_t *param);
z_result_t _z_multicast_open_peer(_z_transport_multicast_establish_param_t *param, const _z_link_t *zl,
                                  const _z_id_t *local_zid);
z_result_t _z_multicast_open_client(_z_transport_multicast_establish_param_t *param, const _z_link_t *zl,
                                    const _z_id_t *local_zid);
z_result_t _z_multicast_send_close(_z_transport_multicast_t *ztm, uint8_t reason, bool link_only);
z_result_t _z_multicast_transport_close(_z_transport_multicast_t *ztm, uint8_t reason);
void _z_multicast_transport_clear(_z_transport_multicast_t *ztm);

#ifdef __cplusplus
}
#endif
#endif /* ZENOH_PICO_MULTICAST_TRANSPORT_H */
