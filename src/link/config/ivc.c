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
//   Phase 100.4 — NVIDIA Tegra IVC link transport.
//

#include "zenoh-pico/link/config/ivc.h"

#include <string.h>

#if Z_FEATURE_LINK_IVC == 1

// IVC has no config keys today (see `link/config/ivc.h`). These wrappers
// are stubs that mirror the serial-side codec shape so the endpoint
// parser can dispatch on the schema uniformly.

size_t _z_ivc_config_strlen(const _z_str_intmap_t *s) {
    (void)s;
    return 0;
}

void _z_ivc_config_onto_str(char *dst, size_t dst_len, const _z_str_intmap_t *s) {
    (void)s;
    if ((dst != NULL) && (dst_len > 0)) {
        dst[0] = '\0';
    }
}

char *_z_ivc_config_to_str(const _z_str_intmap_t *s) {
    (void)s;
    char *out = (char *)z_malloc(1);
    if (out != NULL) {
        out[0] = '\0';
    }
    return out;
}

z_result_t _z_ivc_config_from_strn(_z_str_intmap_t *strint, const char *s, size_t n) {
    (void)strint;
    (void)s;
    (void)n;
    return _Z_RES_OK;
}

z_result_t _z_ivc_config_from_str(_z_str_intmap_t *strint, const char *s) {
    (void)strint;
    (void)s;
    return _Z_RES_OK;
}

#endif
