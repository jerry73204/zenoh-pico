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

// NOT FOR UPSTREAM. Integration-only header for embedders that compile these
// sources directly instead of through zenoh-pico's own CMake.
//
// Upstream generates `config.h` from `config.h.in`; a consumer that does not
// run that CMake has no `zenoh-pico/config.h`, and every source includes one.
// nano-ros compiles the sources from two places -- a cargo build script and a
// Zephyr CMake module -- and only the first could be handed a generated path,
// so the header lives in the tree where both find it.
//
// A passthrough is sufficient because EVERY nano-ros platform defines
// `ZENOH_GENERIC` (see `config/*/nros-platform.toml`), and under that the
// upstream template does nothing but include the header below, which the build
// script generates per image. The 48 `@TOKEN@` substitutions in the template
// are all in the `#else` branch and are dead here.

#ifndef INCLUDE_ZENOH_PICO_CONFIG_H
#define INCLUDE_ZENOH_PICO_CONFIG_H

#ifdef ZENOH_GENERIC
#include <zenoh_generic_config.h>
#else
#error "nano-ros builds define ZENOH_GENERIC; see the note above."
#endif

#endif /* INCLUDE_ZENOH_PICO_CONFIG_H */
