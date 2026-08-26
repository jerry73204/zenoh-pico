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

#ifndef INCLUDE_ZENOH_PICO_CONFIG_H
#define INCLUDE_ZENOH_PICO_CONFIG_H

#ifdef ZENOH_GENERIC
#include <zenoh_generic_config.h>
#else

/*--- CMake generated config; pass values to CMake to change the following tokens ---*/
#define Z_FRAG_MAX_SIZE 4096
#define Z_BATCH_UNICAST_SIZE 2048
#define Z_BATCH_MULTICAST_SIZE 2048
#ifndef Z_CONFIG_SOCKET_TIMEOUT
#ifdef ZENOH_NUTTX
#define Z_CONFIG_SOCKET_TIMEOUT 5000
#else
/* Zephyr must NOT use a long timeout: zsock serializes send/recv on a
 * per-fd mutex, so the blocking read task holds the socket for a full
 * SO_RCVTIMEO window between inbound packets and every tx (declare,
 * lease keepalive, publish, reply) stalls behind it. At 5000 ms the
 * client keepalives miss the 10 s lease and the router drops the
 * session. 100 ms matches the unix/freertos ports. */
#define Z_CONFIG_SOCKET_TIMEOUT 100
#endif
#endif
/* Guarded so an embedding build can set these without editing a generated
 * header. Upstream exposes both as CMake cache variables, which only works if
 * you run zenoh-pico's own CMakeLists; a build that compiles these sources
 * directly (the Zephyr module does) gets this checked-in config.h and had no
 * way in at all -- a command-line -D collided with the unconditional #define
 * and lost. Same treatment Z_CONFIG_SOCKET_TIMEOUT already has above.
 *
 * These matter on a serial link. The lease is BOTH the keepalive cadence
 * (lease / EXPIRE_FACTOR) and the peer-silence tolerance, and a peer is
 * dropped after two lease periods with nothing received. Against a router
 * that does not send keepalives on the link, a pure publisher hears nothing
 * back and closes at 2 x Z_TRANSPORT_LEASE regardless of how healthy the
 * link is. */
#ifndef Z_TRANSPORT_LEASE
#define Z_TRANSPORT_LEASE 10000
#endif
#ifndef Z_TRANSPORT_LEASE_EXPIRE_FACTOR
#define Z_TRANSPORT_LEASE_EXPIRE_FACTOR 3
#endif
#define ZP_PERIODIC_SCHEDULER_MAX_TASKS 64

/* #undef Z_FEATURE_UNSTABLE_API */
#ifndef Z_FEATURE_CONNECTIVITY
#define Z_FEATURE_CONNECTIVITY 0
#endif
#ifndef Z_FEATURE_MULTI_THREAD
#define Z_FEATURE_MULTI_THREAD 1
#endif
#ifndef Z_FEATURE_PUBLICATION
#define Z_FEATURE_PUBLICATION 1
#endif
#ifndef Z_FEATURE_ADVANCED_PUBLICATION
#define Z_FEATURE_ADVANCED_PUBLICATION 0
#endif
#ifndef Z_FEATURE_SUBSCRIPTION
#define Z_FEATURE_SUBSCRIPTION 1
#endif
#ifndef Z_FEATURE_ADVANCED_SUBSCRIPTION
#define Z_FEATURE_ADVANCED_SUBSCRIPTION 0
#endif
#ifndef Z_FEATURE_QUERY
#define Z_FEATURE_QUERY 1
#endif
#ifndef Z_FEATURE_QUERYABLE
#define Z_FEATURE_QUERYABLE 1
#endif
#ifndef Z_FEATURE_LIVELINESS
#define Z_FEATURE_LIVELINESS 1
#endif
#ifndef Z_FEATURE_RAWETH_TRANSPORT
#define Z_FEATURE_RAWETH_TRANSPORT 0
#endif
#ifndef Z_FEATURE_INTEREST
#define Z_FEATURE_INTEREST 1
#endif
#ifndef Z_FEATURE_LINK_TCP
#define Z_FEATURE_LINK_TCP 1
#endif
#ifndef Z_FEATURE_LINK_BLUETOOTH
#define Z_FEATURE_LINK_BLUETOOTH 0
#endif
#ifndef Z_FEATURE_LINK_WS
#define Z_FEATURE_LINK_WS 0
#endif
#ifndef Z_FEATURE_LINK_SERIAL
#define Z_FEATURE_LINK_SERIAL 0
#endif
#ifndef Z_FEATURE_LINK_SERIAL_USB
#define Z_FEATURE_LINK_SERIAL_USB 0
#endif
#ifndef Z_FEATURE_LINK_IVC
#define Z_FEATURE_LINK_IVC 0
#endif
/* RFC-0080 — CAN / CAN FD link transport. */
#ifndef Z_FEATURE_LINK_CAN
#define Z_FEATURE_LINK_CAN 0
#endif
#ifndef Z_FEATURE_LINK_TLS
#define Z_FEATURE_LINK_TLS 0
#endif
#ifndef Z_FEATURE_SCOUTING
#define Z_FEATURE_SCOUTING 1
#endif
#ifndef Z_FEATURE_LINK_UDP_MULTICAST
#define Z_FEATURE_LINK_UDP_MULTICAST 1
#endif
#ifndef Z_FEATURE_LINK_UDP_UNICAST
#define Z_FEATURE_LINK_UDP_UNICAST 1
#endif
#ifndef Z_FEATURE_MULTICAST_TRANSPORT
#define Z_FEATURE_MULTICAST_TRANSPORT 1
#endif
#ifndef Z_FEATURE_UNICAST_TRANSPORT
#define Z_FEATURE_UNICAST_TRANSPORT 1
#endif
#ifndef Z_FEATURE_FRAGMENTATION
#define Z_FEATURE_FRAGMENTATION 1
#endif
#ifndef Z_FEATURE_ENCODING_VALUES
#define Z_FEATURE_ENCODING_VALUES 1
#endif
#ifndef Z_FEATURE_TCP_NODELAY
#define Z_FEATURE_TCP_NODELAY 1
#endif
#ifndef Z_FEATURE_LOCAL_SUBSCRIBER
#define Z_FEATURE_LOCAL_SUBSCRIBER 0
#endif
#ifndef Z_FEATURE_LOCAL_QUERYABLE
#define Z_FEATURE_LOCAL_QUERYABLE 0
#endif
#ifndef Z_FEATURE_SESSION_CHECK
#define Z_FEATURE_SESSION_CHECK 1
#endif
#ifndef Z_FEATURE_BATCHING
#define Z_FEATURE_BATCHING 1
#endif
// nano-ros phase-282 (#145) — split tx locking: batch flush steals the wbuf
// under _mutex_tx and writes the socket under _mutex_link_tx only, so
// publishers append while a slow send is in flight. Default OFF.
#ifndef Z_FEATURE_TX_SPLIT_LOCK
#define Z_FEATURE_TX_SPLIT_LOCK 0
#endif

#ifndef Z_FEATURE_BATCH_TX_MUTEX
#define Z_FEATURE_BATCH_TX_MUTEX 0
#endif
#ifndef Z_FEATURE_BATCH_PEER_MUTEX
#define Z_FEATURE_BATCH_PEER_MUTEX 0
#endif
#ifndef Z_FEATURE_MATCHING
#define Z_FEATURE_MATCHING 1
#endif
#ifndef Z_FEATURE_RX_CACHE
#define Z_FEATURE_RX_CACHE 0
#endif
#ifndef Z_FEATURE_UNICAST_PEER
#define Z_FEATURE_UNICAST_PEER 1
#endif
#ifndef Z_FEATURE_AUTO_RECONNECT
#define Z_FEATURE_AUTO_RECONNECT 1
#endif
#ifndef Z_FEATURE_MULTICAST_DECLARATIONS
#define Z_FEATURE_MULTICAST_DECLARATIONS 0
#endif
#ifndef Z_FEATURE_PERIODIC_TASKS
#define Z_FEATURE_PERIODIC_TASKS 0
#endif
#ifndef Z_FEATURE_ADMIN_SPACE
#define Z_FEATURE_ADMIN_SPACE 0
#endif

// End of CMake generation

#endif /* ZENOH_GENERIC */

/*------------------ Runtime configuration properties ------------------*/
/**
 * The library mode.
 * Accepted values : `"client"`, `"peer"`.
 * Default value : `"client"`.
 */
#define Z_CONFIG_MODE_KEY 0x40
#define Z_CONFIG_MODE_CLIENT "client"
#define Z_CONFIG_MODE_PEER "peer"
#define Z_CONFIG_MODE_DEFAULT Z_CONFIG_MODE_CLIENT

/**
 * The locator of a peer to connect to.
 * Accepted values : `<locator>` (ex: `"tcp/10.10.10.10:7447"`).
 * Default value : None.
 * Multiple values are accepted in peer to peer unicast mode.
 */
#define Z_CONFIG_CONNECT_KEY 0x41

/**
 * A locator to listen on.
 * Accepted values : `<locator>` (ex: `"tcp/10.10.10.10:7447"`).
 * Default value : None.
 * Multiple values are not accepted in zenoh-pico.
 */
#define Z_CONFIG_LISTEN_KEY 0x42

/**
 * The user name to use for authentication.
 * Accepted values : `<string>`.
 * Default value : None.
 */
#define Z_CONFIG_USER_KEY 0x43

/**
 * The password to use for authentication.
 * Accepted values : `<string>`.
 * Default value : None.
 */
#define Z_CONFIG_PASSWORD_KEY 0x44

/**
 * Activates/Deactivates multicast scouting.
 * Accepted values : `false`, `true`.
 * Default value : `true`.
 */
#define Z_CONFIG_MULTICAST_SCOUTING_KEY 0x45
#define Z_CONFIG_MULTICAST_SCOUTING_DEFAULT "true"

/**
 * The multicast address and ports to use for multicast scouting.
 * Accepted values : `<ip address>:<port>`.
 * Default value : `"224.0.0.224:7446"`.
 */
#define Z_CONFIG_MULTICAST_LOCATOR_KEY 0x46
#define Z_CONFIG_MULTICAST_LOCATOR_DEFAULT "udp/224.0.0.224:7446"

/**
 * In client mode, the period dedicated to scouting a router before failing.
 * Accepted values : `<int in milliseconds>`.
 * Default value : `"1000"`.
 */
#define Z_CONFIG_SCOUTING_TIMEOUT_KEY 0x47
#define Z_CONFIG_SCOUTING_TIMEOUT_DEFAULT "1000"

/**
 * The entities to find in the multicast scouting, defined as a bitwise value.
 * Accepted values : [0-7]. Bitwise value are defined in :c:enum:`z_whatami_t`.
 * Default value : `3`.
 */
#define Z_CONFIG_SCOUTING_WHAT_KEY 0x48
#define Z_CONFIG_SCOUTING_WHAT_DEFAULT "3"

/**
 * A configurable and static Zenoh ID to be used on Zenoh Sessions.
 * Accepted values : `<UUDI 128-bit>`.
 */
#define Z_CONFIG_SESSION_ZID_KEY 0x49

/**
 * Indicates if data messages should be timestamped.
 * Accepted values : `false`, `true`.
 * Default value : `false`.
 */
#define Z_CONFIG_ADD_TIMESTAMP_KEY 0x4A
#define Z_CONFIG_ADD_TIMESTAMP_DEFAULT "false"

/*------------------ TLS configuration properties ------------------*/
#define Z_CONFIG_TLS_ROOT_CA_CERTIFICATE_KEY 0x4B
#define Z_CONFIG_TLS_ROOT_CA_CERTIFICATE_BASE64_KEY 0x4C
#define Z_CONFIG_TLS_LISTEN_PRIVATE_KEY_KEY 0x4D
#define Z_CONFIG_TLS_LISTEN_PRIVATE_KEY_BASE64_KEY 0x4E
#define Z_CONFIG_TLS_LISTEN_CERTIFICATE_KEY 0x4F
#define Z_CONFIG_TLS_LISTEN_CERTIFICATE_BASE64_KEY 0x50
#define Z_CONFIG_TLS_ENABLE_MTLS_KEY 0x51
#define Z_CONFIG_TLS_CONNECT_PRIVATE_KEY_KEY 0x52
#define Z_CONFIG_TLS_CONNECT_PRIVATE_KEY_BASE64_KEY 0x53
#define Z_CONFIG_TLS_CONNECT_CERTIFICATE_KEY 0x54
#define Z_CONFIG_TLS_CONNECT_CERTIFICATE_BASE64_KEY 0x55
#define Z_CONFIG_TLS_VERIFY_NAME_ON_CONNECT_KEY 0x56

/*------------------ Compile-time configuration properties ------------------*/
/**
 * Default length for Zenoh ID. Maximum size is 16 bytes.
 * This configuration will only be applied to Zenoh IDs generated by Zenoh-Pico.
 */
#define Z_ZID_LENGTH 16

/**
 * Protocol version identifier.
 * Do not change this value.
 */
#define Z_PROTO_VERSION 0x09

/**
 * Default multicast session join interval in milliseconds.
 */
#define Z_JOIN_INTERVAL 2500

#define Z_SN_RESOLUTION 0x02
#define Z_REQ_RESOLUTION 0x02

/**
 * Default size for the rx cache size (if activated).
 */
#define Z_RX_CACHE_SIZE 10

/**
 * Default get timeout in milliseconds.
 */
#define Z_GET_TIMEOUT_DEFAULT 10000

/**
 * Maximum number of connections for unicast listen sockets.
 */
#define Z_LISTEN_MAX_CONNECTION_NB 10

/**
 * Default "nop" instruction
 */
#define ZP_ASM_NOP __asm__("nop")

#endif /* INCLUDE_ZENOH_PICO_CONFIG_H */
