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

#ifndef ZENOH_PICO_LINK_CONFIG_IVC_H
#define ZENOH_PICO_LINK_CONFIG_IVC_H

#include "zenoh-pico/collections/intmap.h"
#include "zenoh-pico/collections/string.h"
#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_IVC == 1

// IVC has no per-link config arguments today: the channel id lives in
// the locator address (`ivc/2`), and the frame size is queried from
// the driver at open time. We keep an empty intmap mapping for symmetry
// with the other link transports' config codecs and so the endpoint
// parser has something to call.
#define IVC_CONFIG_ARGC 0

#define IVC_CONFIG_MAPPING_BUILD _z_str_intmapping_t args[1] = {{0, NULL}}; /* empty */

size_t _z_ivc_config_strlen(const _z_str_intmap_t *s);

void _z_ivc_config_onto_str(char *dst, size_t dst_len, const _z_str_intmap_t *s);
char *_z_ivc_config_to_str(const _z_str_intmap_t *s);

z_result_t _z_ivc_config_from_str(_z_str_intmap_t *strint, const char *s);
z_result_t _z_ivc_config_from_strn(_z_str_intmap_t *strint, const char *s, size_t n);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_LINK_CONFIG_IVC_H */
