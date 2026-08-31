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

static bool _zp_unicast_peer_is_expired(const _z_transport_peer_unicast_t *target,
                                        const _z_transport_peer_unicast_t *peer) {
    _ZP_UNUSED(target);
    return !peer->common._received;
}

static void _zp_unicast_report_disconnected_peers(_z_transport_unicast_t *ztu,
                                                  _z_transport_peer_unicast_slist_t **dropped_peers) {
    if (dropped_peers == NULL || *dropped_peers == NULL) {
        return;
    }

#if Z_FEATURE_CONNECTIVITY == 1
    uint16_t mtu = 0;
    bool is_streamed = false;
    bool is_reliable = false;
    _z_transport_get_link_properties(&ztu->_common, &mtu, &is_streamed, &is_reliable);
#endif

    _z_session_t *zs = _z_transport_common_get_session(&ztu->_common);
    _z_transport_peer_unicast_slist_t *it = *dropped_peers;
    while (it != NULL) {
        _z_transport_peer_unicast_t *peer = _z_transport_peer_unicast_slist_value(it);
        _Z_INFO("Deleting peer because it has expired after %zums", ztu->_common._lease);
        _z_interest_peer_disconnected(zs, &peer->common);
#if Z_FEATURE_CONNECTIVITY == 1
        _z_connectivity_peer_event_data_t disconnected_peer = {0};
        _z_connectivity_peer_event_data_alias_from_common(&disconnected_peer, &peer->common);
        _z_connectivity_peer_disconnected(zs, &disconnected_peer, false, mtu, is_streamed, is_reliable);
#endif
        it = _z_transport_peer_unicast_slist_next(it);
    }
    _z_transport_peer_unicast_slist_free(dropped_peers);
}

/* Returns false when another task already owns the failover — the caller must
 * then keep looping instead of exiting. Otherwise it does not return at all
 * (`_z_task_exit`). nano-ros issue 0924. */
static bool _zp_unicast_failed(_z_transport_unicast_t *ztu) {
    _z_session_t *zs = _z_transport_common_get_session(&ztu->_common);
#if Z_FEATURE_LIVELINESS == 1 && Z_FEATURE_SUBSCRIPTION == 1
    _z_liveliness_subscription_undeclare_all(zs);
#endif
#if Z_FEATURE_CONNECTIVITY == 1
    _z_connectivity_peer_event_data_t disconnected_peer = {0};
    uint16_t mtu = 0;
    bool is_streamed = false;
    bool is_reliable = false;
    bool has_disconnected_peer = false;
    _z_transport_peer_mutex_lock(&ztu->_common);
    if (!_z_transport_peer_unicast_slist_is_empty(ztu->_peers)) {
        _z_transport_peer_unicast_t *curr_peer = _z_transport_peer_unicast_slist_value(ztu->_peers);
        _z_transport_get_link_properties(&ztu->_common, &mtu, &is_streamed, &is_reliable);
        _z_connectivity_peer_event_data_copy_from_common(&disconnected_peer, &curr_peer->common);
        has_disconnected_peer = true;
    }
    _z_transport_peer_mutex_unlock(&ztu->_common);
    if (has_disconnected_peer) {
        _z_connectivity_peer_disconnected(zs, &disconnected_peer, false, mtu, is_streamed, is_reliable);
        _z_connectivity_peer_event_data_clear(&disconnected_peer);
    }
#endif
    /* nano-ros issue 0899 — upgrade the session ref UNCONDITIONALLY, not just
     * under AUTO_RECONNECT: it is what keeps the session (and therefore the
     * transport lifetime lock taken below) alive while the teardown drops the
     * transport's own weak ref to it. */
    _z_session_rc_t zs_rc = _z_session_weak_upgrade(&ztu->_common._session);

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
    _z_session_t *zsp = _Z_RC_IS_NULL(&zs_rc) ? NULL : _Z_RC_IN_VAL(&zs_rc);

    /* nano-ros issue 0924 — CLAIM the failover, or leave it to whoever holds
     * it. `_z_reopen` below starts a NEW lease task before it returns, and that
     * task begins counting immediately: if the reopen outlasts one lease
     * period it expires and arrives here to tear down the SAME transport while
     * this frame is still inside it. Two concurrent teardowns wedge in lwIP's
     * netconn close and the session never comes back — every publish returns
     * `-10` from then on.
     *
     * Measured: with a lease long enough that a reopen always finishes first,
     * three freeze/kill outage cycles recover cleanly. With the lease alone
     * shortened below the reopen time, on the same code, the very first outage
     * wedges it — 227 failed publishes out of 247. The lease value decides how
     * OFTEN this is hit; it is not what makes it possible. */
    if (zsp != NULL) {
        _z_session_transport_mutex_lock(zsp);
        if (zsp->_reconnecting) {
            /* Someone else owns the failover. Do NOT exit this task: it is a
             * live lease task and the session will still need one if the
             * reopen in flight succeeds. Go back to the loop and let the
             * caller's `next_lease` reset. */
            _z_session_transport_mutex_unlock(zsp);
            _z_session_rc_drop(&zs_rc);
            return false;
        }
        zsp->_reconnecting = true;
        zsp->_tp._type = _Z_TRANSPORT_NONE;
        _z_session_transport_mutex_unlock(zsp);
    }

    /* NOT under `_mutex_transport`, deliberately -- upstream 1.8.0 wraps this
     * pair in the lock and that is the measured lwIP deadlock described above.
     * The handshake has already made it unnecessary: `_tp._type` is
     * `_Z_TRANSPORT_NONE`, published under the lock, so every publisher now
     * takes the `default:` arm without touching a freed byte. */
    _z_unicast_transport_close(ztu, _Z_CLOSE_EXPIRED);
    _z_transport_clear(&zs->_tp, true);

#if Z_FEATURE_AUTO_RECONNECT == 1
    z_result_t ret = _z_reopen(&zs_rc);
    if (ret != _Z_RES_OK) {
        _Z_ERROR("Reopen failed: %i", ret);
    }
#endif

    /* Release the claim before this task leaves, so the lease task the reopen
     * started can fail over in its turn if the link drops again. */
    if (zsp != NULL) {
        _z_session_transport_mutex_lock(zsp);
        zsp->_reconnecting = false;
        _z_session_transport_mutex_unlock(zsp);
    }
    _z_session_rc_drop(&zs_rc);

    _z_task_exit();
    return true;
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
                    if (_zp_unicast_failed(ztu)) {
                        return 0;
                    }
                    /* Another task owns the failover; stay alive and re-arm. */
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
                        if (_zp_unicast_failed(ztu)) {
                            return 0;
                        }
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
                _z_transport_peer_unicast_slist_t *dropped_peers = _z_transport_peer_unicast_slist_new();
                _z_transport_peer_mutex_lock(&ztu->_common);
                ztu->_peers = _z_transport_peer_unicast_slist_extract_all_filter(ztu->_peers, &dropped_peers,
                                                                                 _zp_unicast_peer_is_expired, NULL);
                _z_transport_peer_unicast_slist_t *curr_list = ztu->_peers;
                while (curr_list != NULL) {
                    curr_peer = _z_transport_peer_unicast_slist_value(curr_list);
                    curr_peer->common._received = false;
                    curr_list = _z_transport_peer_unicast_slist_next(curr_list);
                }
                _z_transport_peer_mutex_unlock(&ztu->_common);
                _zp_unicast_report_disconnected_peers(ztu, &dropped_peers);
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

        // The keep alive and lease intervals are expressed in milliseconds.
        //
        // nano-ros issue 0959 — sleep in BOUNDED CHUNKS, not for the whole
        // interval. `_zp_unicast_stop_lease_task` clears `_lease_task_running`
        // and then JOINS, so a task asleep for the full interval makes teardown
        // wait out whatever remains of it. That latency is `lease /
        // Z_TRANSPORT_LEASE_EXPIRE_FACTOR`, which nano-ros #0906 moved from
        // 3.3 s to 20 s when it raised the lease to match the ROS router's
        // keep-alive cadence — measured on one native test, 3.5 s -> 20.2 s,
        // entirely in teardown.
        //
        // The loop already decrements by what it actually slept, so chunking is
        // local: the schedule is unchanged, only the granularity at which the
        // running flag is re-read. One extra wakeup per second on an otherwise
        // idle task is not a cost worth measuring; a twenty-second join is.
        {
            size_t remaining = (size_t)interval;
            while (remaining > 0 && ztu->_common._lease_task_running) {
                size_t chunk = remaining > Z_TRANSPORT_LEASE_TASK_SLEEP_CHUNK_MS
                                   ? (size_t)Z_TRANSPORT_LEASE_TASK_SLEEP_CHUNK_MS
                                   : remaining;
                z_sleep_ms(chunk);
                remaining -= chunk;
            }
            // Account only for the time actually slept: a stop mid-interval
            // exits the outer loop anyway, but leaving the counters honest
            // keeps this correct if the flag is ever re-set.
            interval = interval - (int)remaining;
        }

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
