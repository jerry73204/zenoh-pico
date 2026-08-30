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

#include "zenoh-pico/transport/unicast/lease.h"

#include "zenoh-pico/session/interest.h"
#include "zenoh-pico/session/liveliness.h"
#include "zenoh-pico/session/query.h"
#include "zenoh-pico/session/utils.h"
#include "zenoh-pico/system/common/platform.h"
#include "zenoh-pico/transport/common/tx.h"
#include "zenoh-pico/transport/transport.h"
#include "zenoh-pico/transport/unicast/transport.h"
#include "zenoh-pico/utils/logging.h"

#if Z_FEATURE_UNICAST_TRANSPORT == 1

z_result_t _zp_unicast_send_keep_alive(_z_transport_unicast_t *ztu) {
    // Use non-blocking try-lock for keep-alive sends.
    //
    // The TX mutex may be held by the app task during entity declarations
    // (z_declare_queryable, z_declare_publisher). Each declaration's TCP
    // send holds _mutex_tx for a full network round-trip. On FreeRTOS QEMU
    // with -icount shift=auto (wall-clock virtual time), blocking here would
    // stall the lease task until all declarations complete, potentially
    // causing cascading delays that starve the declaration sends.
    //
    // Using try_send: if the TX path is busy, skip this keep-alive.
    // The declaration sends themselves prove liveness to the router.
    _z_transport_message_t t_msg = _z_t_msg_make_keep_alive();
    return _z_transport_tx_try_send_t_msg(&ztu->_common, &t_msg, NULL);
}
#else

z_result_t _zp_unicast_send_keep_alive(_z_transport_unicast_t *ztu) {
    _ZP_UNUSED(ztu);
    _Z_ERROR_RETURN(_Z_ERR_TRANSPORT_NOT_AVAILABLE);
}
#endif  // Z_FEATURE_UNICAST_TRANSPORT == 1

#if Z_FEATURE_MULTI_THREAD == 1 && Z_FEATURE_UNICAST_TRANSPORT == 1

static void _zp_unicast_failed(_z_transport_unicast_t *ztu) {
#if Z_FEATURE_LIVELINESS == 1 && Z_FEATURE_SUBSCRIPTION == 1
    _z_liveliness_subscription_undeclare_all(_z_transport_common_get_session(&ztu->_common));
#endif
    /* nano-ros issue 0899 — upgrade the session ref UNCONDITIONALLY, not just
     * under AUTO_RECONNECT: it is what keeps the session (and therefore the
     * transport lifetime lock taken below) alive while `_z_unicast_transport_
     * clear` drops the transport's own weak ref to it. */
    _z_session_rc_t zs = _z_session_weak_upgrade(&ztu->_common._session);

    /* Join the read task BEFORE taking the lock, never while holding it: a
     * message in flight can dispatch a user callback that publishes, and that
     * publish now takes the same lock. */
    if (ztu->_common._read_task != NULL) {
        ztu->_common._read_task_running = false;
        _z_task_join(ztu->_common._read_task);
        _z_task_free(&ztu->_common._read_task);
    }
    ztu->_common._lease_task_running = false;

    /* nano-ros issue 0899 — THE FIX, and it is a HANDSHAKE, not a critical
     * section wrapped around the teardown.
     *
     * Everything below frees resources an application task can be inside:
     * `_z_wbuf_clear` on `_wbuf`, `_z_mutex_drop` on `_mutex_tx`/`_mutex_rx`.
     * The publisher must be kept out — but the lock must NOT be held across the
     * teardown itself, because `_z_link_free` closes the socket and blocks in
     * lwIP until the stack completes it. Holding it there DEADLOCKS: measured,
     * the lease task parks in `_z_link_free` and the image goes quiet at the
     * first or second lapse instead of asserting. Trading a crash for a hang is
     * not a fix.
     *
     * So publish the INVALIDATION under the lock, and let go. `_z_send_n_msg`
     * and `_z_send_n_batch` read `_tp._type` inside that same lock, so:
     *
     *   - while this take is held, no publisher is inside the transport;
     *   - after the release, every publisher reads `_Z_TRANSPORT_NONE`, takes
     *     the `default:` arm and returns `_Z_ERR_TRANSPORT_NOT_AVAILABLE`
     *     without touching one freed byte.
     *
     * `_z_open` already sets `_type` to NONE on entry and restores it on
     * success, so the window closes by itself when the reopen lands — and a
     * reopen that keeps failing keeps publishers erroring rather than blocking
     * on a link that is not there. */
    _z_session_t *zsp = _Z_RC_IS_NULL(&zs) ? NULL : _Z_RC_IN_VAL(&zs);
    if (zsp != NULL) {
        _z_session_transport_lock(zsp);
        zsp->_tp._type = _Z_TRANSPORT_NONE;
        _z_session_transport_unlock(zsp);
    }

    _z_unicast_transport_close(ztu, _Z_CLOSE_EXPIRED);
    _z_unicast_transport_clear(ztu, true);

#if Z_FEATURE_AUTO_RECONNECT == 1
    z_result_t ret = _z_reopen(&zs);
    if (ret != _Z_RES_OK) {
        _Z_ERROR("Reopen failed: %i", ret);
    }
#endif

    _z_session_rc_drop(&zs);

    _z_task_exit();
}

void *_zp_unicast_lease_task(void *ztu_arg) {
    _z_transport_unicast_t *ztu = (_z_transport_unicast_t *)ztu_arg;
    ztu->_common._transmitted = false;

    int next_lease = (int)ztu->_common._lease;
    int next_keep_alive = (int)(ztu->_common._lease / Z_TRANSPORT_LEASE_EXPIRE_FACTOR);

    z_whatami_t mode = _z_transport_common_get_session(&ztu->_common)->_mode;
    _z_transport_peer_unicast_t *curr_peer = NULL;
    if (mode == Z_WHATAMI_CLIENT) {
        curr_peer = _z_transport_peer_unicast_slist_value(ztu->_peers);
        assert(curr_peer != NULL);
    }
    while (ztu->_common._lease_task_running) {
        // Process client lease
        if (mode == Z_WHATAMI_CLIENT) {
            if (next_lease <= 0) {
                // Check if received data
                if (curr_peer->common._received) {
                    // Reset the lease parameters
                    curr_peer->common._received = false;
                } else {
                    // THIS LOG STRING USED IN TEST, change with caution
                    _Z_INFO("Closing session because it has expired after %zums", ztu->_common._lease);
                    _zp_unicast_failed(ztu);
                    return 0;
                }
                next_lease = (int)ztu->_common._lease;
            }
            // Next keep alive process
            if (next_keep_alive <= 0) {
                _Z_DEBUG("Sending keep alive");
                // Check if need to send a keep alive
                if (!ztu->_common._transmitted) {
                    if (_zp_unicast_send_keep_alive(ztu) < 0) {
                        // THIS LOG STRING USED IN TEST, change with caution
                        _Z_INFO("Send keep alive failed.");
                        _zp_unicast_failed(ztu);
                        return 0;
                    }
                }
                // Reset the keep alive parameters
                ztu->_common._transmitted = false;
                next_keep_alive = (int)(ztu->_common._lease / Z_TRANSPORT_LEASE_EXPIRE_FACTOR);
            }
        }
#if Z_FEATURE_UNICAST_PEER == 1
        else {  // Peer lease
            if (next_lease <= 0) {
                _z_transport_peer_unicast_slist_t *prev = NULL;
                _z_transport_peer_unicast_slist_t *prev_drop = NULL;
                _z_transport_peer_mutex_lock(&ztu->_common);
                _z_transport_peer_unicast_slist_t *curr_list = ztu->_peers;
                while (curr_list != NULL) {
                    bool drop_peer = false;
                    curr_peer = _z_transport_peer_unicast_slist_value(curr_list);
                    // Check if peer received data
                    if (curr_peer->common._received) {
                        curr_peer->common._received = false;
                    } else {
                        _Z_INFO("Deleting peer because it has expired after %zums", ztu->_common._lease);
                        drop_peer = true;
                        prev_drop = prev;
                    }
                    // Update previous only if current node is not dropped
                    if (!drop_peer) {
                        prev = curr_list;
                    }
                    // Progress list
                    curr_list = _z_transport_peer_unicast_slist_next(curr_list);
                    // Drop if needed
                    if (drop_peer) {
                        _z_session_t *zs = _z_transport_common_get_session(&ztu->_common);
                        _z_interest_peer_disconnected(zs, &curr_peer->common);
                        ztu->_peers = _z_transport_peer_unicast_slist_drop_element(ztu->_peers, prev_drop);
                    }
                }
                _z_transport_peer_mutex_unlock(&ztu->_common);
                next_lease = (int)ztu->_common._lease;
            }
            if (next_keep_alive <= 0) {
                if (!ztu->_common._transmitted) {
                    _Z_DEBUG("Sending keep alive");
                    // Send keep alive to all peers (non-blocking — skip if TX busy)
                    _z_transport_message_t t_msg = _z_t_msg_make_keep_alive();
                    _z_transport_peer_mutex_lock(&ztu->_common);
                    if (!_z_transport_peer_unicast_slist_is_empty(ztu->_peers)) {
                        if (_z_transport_tx_try_send_t_msg(&ztu->_common, &t_msg, ztu->_peers) != _Z_RES_OK) {
                            _Z_INFO("Send keep alive failed.");
                        }
                    }
                    _z_transport_peer_mutex_unlock(&ztu->_common);
                }
                ztu->_common._transmitted = false;
                next_keep_alive = (int)(ztu->_common._lease / Z_TRANSPORT_LEASE_EXPIRE_FACTOR);
            }
        }
#endif

        // Query timeout process
        _z_pending_query_process_timeout(_z_transport_common_get_session(&ztu->_common));

        // Compute the target interval
        int interval;
        if (next_lease == 0) {
            interval = next_keep_alive;
        } else {
            interval = next_lease;
            if (next_keep_alive < interval) {
                interval = next_keep_alive;
            }
        }

        // The keep alive and lease intervals are expressed in milliseconds
        z_sleep_ms((size_t)interval);

        next_lease = next_lease - interval;
        next_keep_alive = next_keep_alive - interval;
    }
    return 0;
}

z_result_t _zp_unicast_start_lease_task(_z_transport_t *zt, z_task_attr_t *attr, _z_task_t *task) {
    // Init memory
    (void)memset(task, 0, sizeof(_z_task_t));
    zt->_transport._unicast._common._lease_task_running = true;  // Init before z_task_init for concurrency issue
    // Init task
    if (_z_task_init(task, attr, _zp_unicast_lease_task, &zt->_transport._unicast) != _Z_RES_OK) {
        _Z_ERROR_RETURN(_Z_ERR_SYSTEM_TASK_FAILED);
    }
    // Attach task
    zt->_transport._unicast._common._lease_task = task;
    return _Z_RES_OK;
}

z_result_t _zp_unicast_stop_lease_task(_z_transport_t *zt) {
    zt->_transport._unicast._common._lease_task_running = false;
    return _Z_RES_OK;
}
#else

void *_zp_unicast_lease_task(void *ztu_arg) {
    _ZP_UNUSED(ztu_arg);
    return NULL;
}

z_result_t _zp_unicast_start_lease_task(_z_transport_t *zt, void *attr, void *task) {
    _ZP_UNUSED(zt);
    _ZP_UNUSED(attr);
    _ZP_UNUSED(task);
    _Z_ERROR_RETURN(_Z_ERR_TRANSPORT_NOT_AVAILABLE);
}

z_result_t _zp_unicast_stop_lease_task(_z_transport_t *zt) {
    _ZP_UNUSED(zt);
    _Z_ERROR_RETURN(_Z_ERR_TRANSPORT_NOT_AVAILABLE);
}
#endif
