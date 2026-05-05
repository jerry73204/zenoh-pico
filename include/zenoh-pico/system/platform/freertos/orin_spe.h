//
// Copyright (c) 2026 NEWSLab NTU
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   Phase 11.3.B — AGX Orin SPE platform header.
//

#ifndef ZENOH_PICO_SYSTEM_FREERTOS_ORIN_SPE_TYPES_H
#define ZENOH_PICO_SYSTEM_FREERTOS_ORIN_SPE_TYPES_H

#include <time.h>

#include "FreeRTOS.h"
#include "semphr.h"

// Variant of `freertos/lwip.h` for IVC-only targets. The SPE has no
// Ethernet, no MAC, no socket layer — `Z_FEATURE_LINK_TCP` /
// `Z_FEATURE_LINK_UDP_*` are always 0 here, and the
// `_z_sys_net_socket_t` / `_z_sys_net_endpoint_t` unions degenerate
// to empty (the `link.h` link union still mentions them but they
// have no active members on this build). The `_z_ivc_socket_t`
// branch of the link union is the only active one.

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_MULTI_THREAD == 1
#include "event_groups.h"

typedef struct {
    const char *name;
    UBaseType_t priority;
    size_t stack_depth;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    bool static_allocation;
    StackType_t *stack_buffer;
    StaticTask_t *task_buffer;
#endif /* SUPPORT_STATIC_ALLOCATION */
} z_task_attr_t;

typedef struct {
    TaskHandle_t handle;
    EventGroupHandle_t join_event;
    void *(*fun)(void *);
    void *arg;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticEventGroup_t join_event_buffer;
#endif /* SUPPORT_STATIC_ALLOCATION */
} _z_task_t;

typedef struct {
    SemaphoreHandle_t handle;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticSemaphore_t buffer;
#endif /* SUPPORT_STATIC_ALLOCATION */
} _z_mutex_t;

typedef _z_mutex_t _z_mutex_rec_t;

typedef struct {
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t sem;
    int waiters;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticSemaphore_t mutex_buffer;
    StaticSemaphore_t sem_buffer;
#endif /* SUPPORT_STATIC_ALLOCATION */
} _z_condvar_t;
#endif  // Z_FEATURE_MULTI_THREAD == 1

typedef TickType_t z_clock_t;
typedef struct timeval z_time_t;

// Empty socket unions — the SPE has no socket-based link transport.
// The link.h transport union still references these fields by name,
// but for an IVC-only build (`Z_FEATURE_LINK_IVC == 1`, all other
// link features 0), no caller dereferences them. Wrap a single
// `uint8_t _placeholder` so empty-union builds don't break on
// strict-C compilers.
typedef struct {
    union {
        uint8_t _placeholder;
    };
} _z_sys_net_socket_t;

typedef struct {
    union {
        uint8_t _placeholder;
    };
} _z_sys_net_endpoint_t;

#ifdef __cplusplus
}
#endif

#endif
