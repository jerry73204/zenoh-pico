//
// nros: link-custom — Phase 115.B
//
// Endpoint config helpers. Boilerplate identical in shape to the
// existing `serial.c` / `tcp.c` config files; we just have zero
// configurable keys today, so the intmap is empty.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//

#include "zenoh-pico/link/config/custom.h"

#include <string.h>

#if Z_FEATURE_LINK_CUSTOM == 1

size_t _z_custom_config_strlen(const _z_str_intmap_t *s) {
    CUSTOM_CONFIG_MAPPING_BUILD
    return _z_str_intmap_strlen(s, CUSTOM_CONFIG_ARGC, args);
}

void _z_custom_config_onto_str(char *dst, size_t dst_len, const _z_str_intmap_t *s) {
    CUSTOM_CONFIG_MAPPING_BUILD
    _z_str_intmap_onto_str(dst, dst_len, s, CUSTOM_CONFIG_ARGC, args);
}

char *_z_custom_config_to_str(const _z_str_intmap_t *s) {
    CUSTOM_CONFIG_MAPPING_BUILD
    return _z_str_intmap_to_str(s, CUSTOM_CONFIG_ARGC, args);
}

z_result_t _z_custom_config_from_strn(_z_str_intmap_t *strint, const char *s, size_t n) {
    CUSTOM_CONFIG_MAPPING_BUILD
    return _z_str_intmap_from_strn(strint, s, CUSTOM_CONFIG_ARGC, args, n);
}

z_result_t _z_custom_config_from_str(_z_str_intmap_t *strint, const char *s) {
    CUSTOM_CONFIG_MAPPING_BUILD
    return _z_str_intmap_from_str(strint, s, CUSTOM_CONFIG_ARGC, args);
}

#endif /* Z_FEATURE_LINK_CUSTOM */
