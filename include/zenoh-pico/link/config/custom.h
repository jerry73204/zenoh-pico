//
// nros: link-custom — Phase 115.B
//
// Endpoint config for the `custom://` locator scheme. v1 has no
// configurable keys — the four fn pointers + user_data are
// supplied at runtime via `nros_rmw::set_custom_transport(...)`.
// Reserved key indexes are carved out so future framing /
// MTU-tuning args can land without breaking ABI.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//

#ifndef ZENOH_PICO_LINK_CONFIG_CUSTOM_H
#define ZENOH_PICO_LINK_CONFIG_CUSTOM_H

#include "zenoh-pico/collections/intmap.h"
#include "zenoh-pico/collections/string.h"
#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_CUSTOM == 1

/* CUSTOM_SCHEMA defined in link/endpoint.h to match the placement
 * convention of other schema strings. */

/* No configurable keys today; v1 takes the runtime-registered
 * vtable as-is. The intmap argc is 0 — caller passes an empty
 * config segment. Reserved for future minor-version detection. */
#define CUSTOM_CONFIG_ARGC 0

#define CUSTOM_CONFIG_MAPPING_BUILD _z_str_intmapping_t args[1] = {0};

size_t _z_custom_config_strlen(const _z_str_intmap_t *s);
void _z_custom_config_onto_str(char *dst, size_t dst_len, const _z_str_intmap_t *s);
char *_z_custom_config_to_str(const _z_str_intmap_t *s);
z_result_t _z_custom_config_from_str(_z_str_intmap_t *strint, const char *s);
z_result_t _z_custom_config_from_strn(_z_str_intmap_t *strint, const char *s, size_t n);

#endif /* Z_FEATURE_LINK_CUSTOM */

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_LINK_CONFIG_CUSTOM_H */
