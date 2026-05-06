//
// nros: link-custom — Phase 115.B
//
// Runtime-pluggable custom transport for nano-ros. Mirrors
// `nros_rmw::NrosTransportOps` (Phase 115.A.2) one-to-one. The
// canonical struct definition + `abi_version` semantics live in
// `nros-rmw`; this header just re-states the layout for the
// zenoh-pico C source files that consume it.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//

#ifndef ZENOH_PICO_SYSTEM_LINK_CUSTOM_H
#define ZENOH_PICO_SYSTEM_LINK_CUSTOM_H

#include <stddef.h>
#include <stdint.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_CUSTOM == 1

#define _Z_CUSTOM_MTU_SIZE 1500u

/* Same `#[repr(C)]` layout as `nros_rmw::NrosTransportOps`. Single
 * canonical ABI — see `docs/design/portable-rmw-platform-interface.md`
 * R5 (vtable versioning). */
typedef struct {
    uint32_t abi_version;
    uint32_t _reserved;
    void *user_data;
    int32_t (*open)(void *user_data, const void *params);
    void (*close)(void *user_data);
    int32_t (*write)(void *user_data, const uint8_t *buf, size_t len);
    int32_t (*read)(void *user_data, uint8_t *buf, size_t len, uint32_t timeout_ms);
} _z_custom_ops_t;

/* Per-link socket struct embedded in `_z_link_t._socket._custom`. */
typedef struct {
    _z_custom_ops_t _ops;
    bool _opened;
} _z_custom_socket_t;

/* Drain the registered transport from `nros_rmw`'s slot into `out`.
 * Provided by `zpico-platform-custom` (Rust side). Returns 0 on
 * success, non-zero when no transport is registered.
 *
 * Called from `_z_f_link_open_custom` at session-open time. The
 * caller is the C-side custom-link factory; the Rust side reads
 * `nros_rmw::take_custom_transport()`. */
int32_t nros_zpico_custom_take(_z_custom_ops_t *out);

#endif /* Z_FEATURE_LINK_CUSTOM */

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_LINK_CUSTOM_H */
