//
// Copyright (c) 2026 ZettaScale Technology
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

#include "zenoh-pico/link/transport/serial.h"

#if Z_FEATURE_LINK_SERIAL == 1 && defined(ZENOH_ZEPHYR)

/* nano-ros's Zephyr UART backend. Upstream's backend polls a byte at a time
 * under the serial protocol layer's generic framing; this one owns the framing
 * (ZP_PLATFORM_SERIAL_FRAMING) so that it can receive by interrupt or DMA into
 * a ring, wake the reader once per frame, and transmit from the UART interrupt.
 * Carried over from the 1.8.0 line's src/system/zephyr/network.c. */

#include <version.h>

#if KERNEL_VERSION_MAJOR == 2
#include <drivers/uart.h>
#else
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#endif

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/transport/serial_protocol.h"
#include "zenoh-pico/protocol/codec/serial.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/logging.h"
#include "zenoh-pico/utils/pointers.h"

z_result_t _z_serial_open_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;
    (void)(sock);
    (void)(txpin);
    (void)(rxpin);
    (void)(baudrate);

    // @TODO: To be implemented
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    ret = _Z_ERR_GENERIC;

    return ret;
}

/* What the serial link did, for a debugger to read while the image runs (the
 * board's console is usually not wired: the UART belongs to zenoh). Counts only
 * go up; nothing here changes behaviour. One instance, for the same reason the
 * RX ring below has one: a session has one serial transport.
 *
 *   rx_bytes         bytes the ISR took off the wire (interrupt-driven RX only)
 *   rx_frames        frames that decoded (COBS, length and CRC all good)
 *   rx_bad_frames    frames that did not: bytes were lost or damaged, and the
 *                    frame was dropped whole
 *   rx_partial       a frame abandoned mid-way by the per-byte timeout
 *   overruns         UART hardware overruns (the ISR was too late)
 *   ring_overflows   ISR drains that did not fit the RX ring (the reader was
 *                    too late); ring_high_water is the most the ring held
 *   tx_bytes         bytes written, tx_frames frames written
 *   tx_busy_cycles   k_cycle_get_32() cycles the sending thread spent in the
 *                    send call. With interrupt-driven TX (below) that is time
 *                    spent queueing, most of it asleep waiting for room; on the
 *                    polled fallback it is the uart_poll_out busy-wait, i.e.
 *                    CPU the sender burnt transmitting
 *   framing_errors, noise_errors, parity_errors
 *                    what uart_err_check reported besides overruns: a byte
 *                    the UART received damaged (a baud-rate mismatch between
 *                    the two ends shows up here first), so its frame fails
 *                    its CRC and counts in rx_bad_frames too
 *   rx_wakes         times the reader woke on the RX semaphore (interrupt-
 *                    driven RX). About one per frame since the reader waits
 *                    for a frame delimiter; it used to be one per ISR, i.e.
 *                    per byte or two */
typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_frames;
    uint32_t rx_bad_frames;
    uint32_t rx_partial;
    uint32_t overruns;
    uint32_t ring_overflows;
    uint32_t ring_high_water;
    uint32_t tx_bytes;
    uint32_t tx_frames;
    uint32_t tx_busy_cycles;
    uint32_t framing_errors;
    uint32_t noise_errors;
    uint32_t parity_errors;
    uint32_t rx_wakes;
} _z_zephyr_serial_stats_t;
volatile _z_zephyr_serial_stats_t _z_zephyr_serial_stats;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

/* ── issue 0852: interrupt-driven RX ────────────────────────────────────────
 *
 * The polled path below this block reads with `uart_poll_in` and `k_yield()`s
 * between attempts. That makes byte capture a function of THREAD SCHEDULING: a
 * byte arrives every 87 us at 115200, the hardware FIFO holds a handful, and
 * any scheduling gap longer than that loses data the driver never reports.
 *
 * Raising the reader's priority (which the platform port now honours -- it used
 * to discard it) shrinks the gap but cannot remove it, because the reader still
 * has to be RUNNING to catch a byte. Measured: with the priority fix alone the
 * first action goal completes and later ones still hang.
 *
 * So the bytes are taken in the ISR instead, and the thread reads from a ring
 * buffer that the ISR fills. Capture no longer depends on the scheduler at all
 * -- only on interrupt latency, which is bounded and does not grow with load.
 *
 * ONE link. The buffer is static and single-instance because a zenoh-pico
 * session has one serial transport; a second concurrent serial link would need
 * a table keyed by device, and `_z_serial_rx_bind` refuses rather than
 * silently sharing this one. */

#ifndef _Z_ZEPHYR_SERIAL_RX_RING_BYTES
#define _Z_ZEPHYR_SERIAL_RX_RING_BYTES 1024
#endif

/* -- the reader wakes once per FRAME, not once per byte (nano-ros I10) -----
 *
 * The ISR used to give the RX semaphore on every FIFO drain, and the reader
 * took the ring ONE BYTE per loop, with a `k_uptime_get` and a
 * `uart_err_check` driver call per byte. At 921,600 baud a drain carries one
 * or two bytes (a byte is 10.9 us and the ISR runs about 7 us), so every
 * byte cost the reader a wake, two context switches and that per-byte
 * bookkeeping: about 6.6 us of reader CPU per wake on an S32K344 at 160 MHz,
 * which with the ISR's own time is the whole CPU. The reader runs in the
 * transport's read band, above the executor (issue 1534 requires that), so
 * for as long as a frame was on the wire -- 11.5 ms for a 1 KB
 * kinematic_state -- the executor could not run: its release was late by the
 * frame's wire time (measured: 13-19 ms late on a 10 ms spin).
 *
 * Now the ISR gives the semaphore only when a drain carried a frame
 * delimiter (0x00, the COBS frame end) or the ring is half full (a frame
 * longer than half the ring must not overflow it while the reader sleeps),
 * and the reader moves whole runs out of the ring with `ring_buf_get_claim`
 * up to and including the delimiter. Its CPU per frame is now the copy and
 * the decode, not the wire time, and between frames it sleeps. The receive
 * timeout keeps its meaning: the link quiet for _Z_ZEPHYR_SERIAL_TIMEOUT_MS
 * (counted from the last byte the reader saw) abandons the frame. */

/* -- interrupt-driven TX (nano-ros issue 1534) -------------------------------
 *
 * The send path used to be `uart_poll_out` per byte: a busy-wait of 10.9 us a
 * byte at 921,600 (86.8 us at 115200) in the SENDING thread, which on Zephyr
 * is the tx-flush task or, while it registers the image's entities, main.
 * Measured on an S32K344 at 921,600 with every contracted input flowing:
 * 9.5 % of the CPU went to that loop, and at 115200 the island's join held
 * the CPU for 620 ms without a context switch (nano-ros issue 1534).
 *
 * Now the sender copies the frame into a TX ring and the UART's TX interrupt
 * drains it with `uart_fifo_fill`. A sender that finds the ring full SLEEPS
 * on a semaphore the ISR gives, so while the wire is busy the CPU belongs to
 * whoever else is ready. That returns the busy-wait's CPU; it does NOT on its
 * own keep the read task fed: a CPU-bound thread above the read task starves
 * it just as well (measured: with this change and main registering at
 * k_thread 0, the RX ring still overflowed at the join with the read task
 * READY). The read task has to outrank, or equal, every thread that can run
 * while bytes arrive; that is the priority half of issue 1534.
 *
 * The ring has one producer (a thread) and one consumer (the ISR), which is
 * what `ring_buf` needs without a lock. The producer is single because
 * `_z_send_serial_internal` holds `_z_serial_tx_lock` across a send (see
 * there); zenoh-pico's transport TX mutex usually serialises senders too, but
 * not on every path (the handshake, and the split-lock fragment path).
 *
 * -- a send that fits does not wait (nano-ros I12) -------------------------
 *
 * The ring used to be 256 bytes, so any frame longer than that slept here
 * until the wire drained it: about 11 us a byte at 921,600, so 2 ms or more
 * for a 200-byte-plus publish. The sender holds the transport TX mutex while
 * it does, so a publish from the executor's spin thread also waited behind
 * ANOTHER sender's frame (the tx-flush or lease task), for that frame's wire
 * time. Measured on an S32K344: late spins of 1.4-2.8 ms, all of it at the
 * transport mutex.
 *
 * The ring now holds a whole maximum-size frame (_Z_SERIAL_MAX_COBS_BUF_SIZE,
 * 1516 bytes) by default, the way a Linux tty's xmit buffer or an AUTOSAR COM
 * send works: a send that fits is copied and returns, and the wire drains it
 * in the background. A send that does NOT fit (the ring still holds earlier
 * frames) queues what fits and sleeps for the rest, bounded by
 * _Z_ZEPHYR_SERIAL_TX_TIMEOUT_MS as before: reliable traffic is never dropped
 * to make room. `frames_waited` counts those sends, so a ring that ever
 * filled shows up there. A board with traffic bursts sets it larger
 * (CONFIG_NROS_ZENOH_SERIAL_TX_RING_BYTES); a smaller ring still works, it
 * only waits more. */
#ifndef _Z_ZEPHYR_SERIAL_TX_RING_BYTES
#define _Z_ZEPHYR_SERIAL_TX_RING_BYTES 2048
#endif
/* How long a sender waits for the ISR to make room before it gives up on the
   frame. The ring drains at line rate, so this only expires if TX interrupts
   stopped altogether. */
#ifndef _Z_ZEPHYR_SERIAL_TX_TIMEOUT_MS
#define _Z_ZEPHYR_SERIAL_TX_TIMEOUT_MS 1000
#endif

/* Counters for the TX path, read by a debugger like `_z_zephyr_serial_stats`.
 *   irq_bytes      bytes the TX ISR handed to the UART FIFO
 *   waits          times a sender slept for room in the TX ring
 *   timeouts       frames abandoned because no room appeared in time
 *   ring_high_water the most the TX ring held
 *   frames_waited  frames that did not fit the ring whole when queued, so
 *                  their sender waited for the wire (the ring "filled");
 *                  zero means every send was a copy and a return
 *   max_frame      the longest frame queued, in wire bytes; a ring smaller
 *                  than this makes that frame wait every time */
typedef struct {
    uint32_t irq_bytes;
    uint32_t waits;
    uint32_t timeouts;
    uint32_t ring_high_water;
    uint32_t frames_waited;
    uint32_t max_frame;
} _z_zephyr_serial_tx_stats_t;
volatile _z_zephyr_serial_tx_stats_t _z_zephyr_serial_tx_stats;

static uint8_t _z_serial_tx_storage[_Z_ZEPHYR_SERIAL_TX_RING_BYTES];
static struct ring_buf _z_serial_tx_ring;
static K_SEM_DEFINE(_z_serial_tx_sem, 0, 1);
/* Set only once TX interrupts are known to work on the bound device; until
   then (and on a device whose driver has no TX interrupt) the polled path is
   used. */
static bool _z_serial_tx_irq = false;

static uint8_t _z_serial_rx_storage[_Z_ZEPHYR_SERIAL_RX_RING_BYTES];
static struct ring_buf _z_serial_rx_ring;
static K_SEM_DEFINE(_z_serial_rx_sem, 0, 1);
static const struct device *_z_serial_rx_dev = NULL;
/* Sticky, ISR-set, thread-cleared: a byte the ring had no room for. Distinct
   from a UART overrun -- this one is OUR backlog, not the hardware's. */
static volatile bool _z_serial_rx_ring_full = false;

/* Set by a sender before it sleeps for TX room, cleared after; the ISR gives
   the TX semaphore only while it is set. Giving on every drain (the old rule)
   cost a k_sem_give per ISR while nobody waited, which with a ring larger
   than a frame is almost always. */
static volatile bool _z_serial_tx_waiting = false;

#if defined(_Z_ZEPHYR_SERIAL_ISR_STATS)
/* CONFIG_NROS_ZENOH_SERIAL_ISR_STATS (nano-ros phase-474 I13): cycles spent
   in the serial ISR and the number of calls, so a profiler can charge ISR time
   to whatever it interrupted (read the pair before and after). Off by
   default: two k_cycle_get_32 per call. */
volatile uint32_t _z_zephyr_serial_isr_cycles;
volatile uint32_t _z_zephyr_serial_isr_calls;
#endif

#if defined(_Z_ZEPHYR_SERIAL_FIFO_WATERMARK)
/* -- one interrupt per FIFO, not per byte (nano-ros phase-474 I14) --------
 *
 * CONFIG_NROS_ZENOH_SERIAL_FIFO_WATERMARK, NXP LPUART only. The mcux driver
 * leaves both FIFO watermarks at 0, so RDRF (the RX interrupt) asserts on the
 * first byte in the 4-deep RX FIFO and TDRE (TX) only once the TX FIFO is
 * empty; at 921,600 baud and about 7 us an ISR that is one ISR a byte. Its
 * `fifo_read` also loops on RDRF, which with a watermark above 0 would leave
 * bytes in the FIFO, and `fifo_fill` stops at TDRE, which drops after the
 * first byte written. So on this path the ISR reads and fills the FIFO
 * registers itself:
 *
 *   - WATER.RXWATER = _Z_ZEPHYR_SERIAL_LPUART_RXWATER (CONFIG_NROS_ZENOH_
 *     SERIAL_FIFO_RXWATER, default 1): RDRF asserts when the RX FIFO holds
 *     MORE than that, i.e. water + 1 bytes, and FIFO.RXIDEN = 1 asserts it
 *     as well after one idle character with anything below the watermark,
 *     so a frame's tail is not stranded. The ISR drains WATER.RXCOUNT bytes.
 *     Overrun margin from RDRF to OR, for a FIFO of depth D: the free slots
 *     plus the shift register, D + 1 - (water + 1) = D - water characters.
 *     On the 4-deep S32K344 FIFO at 921,600 (10.85 us a character): water 0
 *     44 us, 1 33 us, 2 22 us, 3 11 us. Water 2 measured about 3x fewer RX
 *     calls but 4 overruns a run on the island, so the default is 1: half
 *     the RX interrupts of water 0 for a quarter of its margin. Clamped to
 *     D - 1 at arm time (PARAM.RXFIFO).
 *   - TXWATER stays 0: on TDRE the ISR writes (FIFO depth - WATER.TXCOUNT)
 *     bytes, 4 on an S32K344.
 *
 * About 3x fewer RX calls and 4x fewer TX calls. Interrupt enables still go
 * through uart_irq_{rx,tx}_{enable,disable}; only data, counts and the
 * overrun flag are touched here. The register base is found from the
 * devicetree (one entry per okay nxp,lpuart node), never from the driver's
 * private config. Register layout: S32K344_LPUART.h (hal_nxp, STAT 0x14,
 * CTRL 0x18, DATA 0x1C, FIFO 0x28, WATER 0x2C), the same LPUART_Type every
 * NXP LPUART SoC header defines. */
#include <fsl_device_registers.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>

#ifndef _Z_ZEPHYR_SERIAL_LPUART_RXWATER
#define _Z_ZEPHYR_SERIAL_LPUART_RXWATER 1
#endif

/* STAT bits that are write-1-to-clear; the rest of STAT is configuration and
   is written back unchanged (the HAL's LPUART_ClearStatusFlags rule). */
#define _Z_LPUART_STAT_W1C                                                                                  \
    (LPUART_STAT_LBKDIF_MASK | LPUART_STAT_RXEDGIF_MASK | LPUART_STAT_IDLE_MASK | LPUART_STAT_OR_MASK |    \
     LPUART_STAT_NF_MASK | LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK | LPUART_STAT_MA1F_MASK |              \
     LPUART_STAT_MA2F_MASK)

typedef struct {
    const struct device *dev;
    uintptr_t base;
} _z_serial_lpuart_map_t;

#define _Z_SERIAL_LPUART_MAP_ENTRY(node) {DEVICE_DT_GET(node), DT_REG_ADDR(node)},
static const _z_serial_lpuart_map_t _z_serial_lpuart_map[] = {
    DT_FOREACH_STATUS_OKAY(nxp_lpuart, _Z_SERIAL_LPUART_MAP_ENTRY)};

/* The bound device's registers when it is an LPUART, else NULL (the generic
   ISR body runs). Set at bind, before RX interrupts are enabled; the TX
   FIFO depth with it. Zero-initialised on purpose: bss, so in DTCM with the
   rings under CONFIG_NROS_ZENOH_SERIAL_TCM (an initialised value would sit
   in .data, in cached SRAM). */
static LPUART_Type *_z_serial_lpuart;
static uint32_t _z_serial_lpuart_txfifo;

static LPUART_Type *_z_serial_lpuart_lookup(const struct device *dev) {
    for (size_t i = 0; i < ARRAY_SIZE(_z_serial_lpuart_map); i++) {
        if (_z_serial_lpuart_map[i].dev == dev) {
            return (LPUART_Type *)_z_serial_lpuart_map[i].base;
        }
    }
    return NULL;
}

/* After every uart_configure (which re-runs LPUART_Init and so rewrites
   WATER to 0): watermarks and the idle-empty RX request. The FIFO and WATER
   fields are written with the transmitter and receiver disabled, as
   LPUART_Init does; this runs while the link is being opened, so a byte
   the peer sent at this instant is lost to a handshake that retries. */
static void _z_serial_lpuart_arm(LPUART_Type *b) {
    unsigned int key = irq_lock();
    uint32_t ctrl = b->CTRL;
    b->CTRL = ctrl & ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
    /* TE/RE read back 1 until the character in flight completes: about 11 us
       at 921,600, 87 us at 115,200. Bounded so a stuck bit cannot hang. */
    for (uint32_t spin = 0; (b->CTRL & (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK)) != 0u && spin < 100000u;
         spin++) {
    }
    uint32_t fifo = b->FIFO & ~(LPUART_FIFO_RXUF_MASK | LPUART_FIFO_TXOF_MASK | LPUART_FIFO_RXFLUSH_MASK |
                                LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXIDEN_MASK);
    b->FIFO = fifo | LPUART_FIFO_RXFE_MASK | LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXIDEN(1);
    uint32_t rxfifo = 1u << ((b->PARAM & LPUART_PARAM_RXFIFO_MASK) >> LPUART_PARAM_RXFIFO_SHIFT);
    uint32_t water = (uint32_t)(_Z_ZEPHYR_SERIAL_LPUART_RXWATER);
    if (water > rxfifo - 1u) {
        water = rxfifo - 1u;
    }
    b->WATER = LPUART_WATER_RXWATER(water) | LPUART_WATER_TXWATER(0);
    _z_serial_lpuart_txfifo = 1u << ((b->PARAM & LPUART_PARAM_TXFIFO_MASK) >> LPUART_PARAM_TXFIFO_SHIFT);
    b->CTRL = ctrl;
    irq_unlock(key);
}
#endif /* _Z_ZEPHYR_SERIAL_FIFO_WATERMARK */

static void _z_serial_isr_body(const struct device *dev);

#if defined(_Z_ZEPHYR_SERIAL_FIFO_WATERMARK)
static void _z_serial_isr_body_lpuart(const struct device *dev, LPUART_Type *b);
#define _Z_SERIAL_ISR_BODY(dev)                                    \
    do {                                                           \
        LPUART_Type *_b = _z_serial_lpuart;                        \
        if (_b != NULL) {                                          \
            _z_serial_isr_body_lpuart((dev), _b);                  \
        } else {                                                   \
            _z_serial_isr_body(dev);                               \
        }                                                          \
    } while (0)
#else
#define _Z_SERIAL_ISR_BODY(dev) _z_serial_isr_body(dev)
#endif

static void _z_serial_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
#if defined(_Z_ZEPHYR_SERIAL_ISR_STATS)
    uint32_t t0 = k_cycle_get_32();
    _Z_SERIAL_ISR_BODY(dev);
    _z_zephyr_serial_isr_cycles += k_cycle_get_32() - t0;
    _z_zephyr_serial_isr_calls++;
#else
    _Z_SERIAL_ISR_BODY(dev);
#endif
}

/* About one call per byte at 921,600 baud on an LPUART (a 4-deep FIFO with
   the watermarks at 0), so the body is kept to what the reader and the
   senders need: the counters are accumulated locally and stored once per
   call, not per FIFO read (nano-ros phase-474 I13). */
static void _z_serial_isr_body(const struct device *dev) {
    /* The canonical Zephyr shape. An earlier version guarded the loop with
       `uart_irq_is_pending()` and `break`-ed when RX was not ready -- which
       can leave the interrupt ASSERTED and unhandled, because
       `uart_irq_is_pending` is true for a TX cause as well. The ISR then
       re-enters immediately and forever, and the symptom is not a lost byte
       but a CPU that stops making progress: the board completed discovery at
       boot and then went silent the moment real traffic arrived. */
    if (uart_irq_update(dev) == 0) {
        return;
    }

    uint32_t rx_n = 0;
    bool rx_wake = false;
    while (uart_irq_rx_ready(dev)) {
        /* Drain the hardware FIFO in as few calls as possible: the mcux LPUART
           `fifo_read` loops while the RX register is full, so a large ask is
           one call rather than one per byte. */
        uint8_t chunk[32];
        int n = uart_fifo_read(dev, chunk, sizeof(chunk));
        if (n <= 0) {
            break;
        }
        uint32_t put = ring_buf_put(&_z_serial_rx_ring, chunk, (uint32_t)n);
        rx_n += (uint32_t)n;
        if (put < (uint32_t)n) {
            _z_serial_rx_ring_full = true;
            _z_zephyr_serial_stats.ring_overflows++;
            rx_wake = true;
        }
        /* Wake the reader for a whole frame, not for a byte (see the note at
           the RX ring). A full ring wakes it too: it may be holding the end
           of a frame it cannot see yet. */
        if (!rx_wake && memchr(chunk, 0x00, (size_t)n) != NULL) {
            rx_wake = true;
        }
    }
    if (rx_n != 0u) {
        _z_zephyr_serial_stats.rx_bytes += rx_n;
        uint32_t held = ring_buf_size_get(&_z_serial_rx_ring);
        if (held > _z_zephyr_serial_stats.ring_high_water) {
            _z_zephyr_serial_stats.ring_high_water = held;
        }
        if (rx_wake || held >= (uint32_t)(_Z_ZEPHYR_SERIAL_RX_RING_BYTES / 2)) {
            k_sem_give(&_z_serial_rx_sem);
        }
    }

    /* TX: refill the FIFO from the ring while the UART has room; with the ring
       empty, stop the TX interrupt (TDRE is level-triggered and would
       otherwise fire forever). The thread re-enables it after it puts bytes.
       Single core, and this runs with the thread preempted, so "found empty"
       and "disabled" cannot be split by a put. */
    uint32_t tx_n = 0;
    while (uart_irq_tx_ready(dev)) {
        uint8_t *data;
        uint32_t n = ring_buf_get_claim(&_z_serial_tx_ring, &data, _Z_ZEPHYR_SERIAL_TX_RING_BYTES);
        if (n == 0u) {
            (void)ring_buf_get_finish(&_z_serial_tx_ring, 0);
            uart_irq_tx_disable(dev);
            break;
        }
        int w = uart_fifo_fill(dev, data, (int)n);
        if (w < 0) {
            w = 0;
        }
        (void)ring_buf_get_finish(&_z_serial_tx_ring, (uint32_t)w);
        tx_n += (uint32_t)w;
        if (w == 0) {
            break;
        }
    }
    if (tx_n != 0u) {
        _z_zephyr_serial_tx_stats.irq_bytes += tx_n;
        /* The same rule as RX: wake a waiting sender when there is room for a
           RUN of bytes, not after every FIFO's worth. Giving per fill woke the
           sender every few bytes, two context switches each, in the band above
           the executor. Half the ring keeps the wire busy: the sender refills
           while the other half drains. */
        if (_z_serial_tx_waiting &&
            ring_buf_space_get(&_z_serial_tx_ring) >= (uint32_t)(_Z_ZEPHYR_SERIAL_TX_RING_BYTES / 2)) {
            k_sem_give(&_z_serial_tx_sem);
        }
    }
}

#if defined(_Z_ZEPHYR_SERIAL_FIFO_WATERMARK)
/* The I14 body: the generic one with the FIFO registers read and filled
   directly (see the note at _z_serial_lpuart_arm). Same ring, same wake
   rules, same counters. */
static void _z_serial_isr_body_lpuart(const struct device *dev, LPUART_Type *b) {
    uint32_t stat = b->STAT;
    if ((stat & LPUART_STAT_OR_MASK) != 0u) {
        /* Counted here, where it is seen: the driver clears OR after this
           callback returns, so the reader's uart_err_check would mostly miss
           it on this path. Clearing OR drops nothing: the byte is already
           lost (OR means the shift register's character found the FIFO
           full), the FIFO keeps its contents, and the receiver stores
           nothing new while OR is set, so clearing it BEFORE the drain below
           is what lets reception resume soonest. Only OR is written as 1;
           the other W1C flags (IDLE among them) are written 0, so a flag
           the reader's uart_err_check wants is not consumed here. RXIDEN
           needs no clear: it asserts RDRF only while the receiver is idle
           with RXCOUNT > 0, and the drain below takes RXCOUNT to 0. */
        _z_zephyr_serial_stats.overruns++;
        b->STAT = (stat & ~_Z_LPUART_STAT_W1C) | LPUART_STAT_OR_MASK;
    }

    uint32_t rx_n = 0;
    bool rx_wake = false;
    for (;;) {
        uint32_t cnt = (b->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT;
        if (cnt == 0u) {
            break;
        }
        uint8_t chunk[8];
        if (cnt > sizeof(chunk)) {
            cnt = sizeof(chunk);
        }
        for (uint32_t i = 0; i < cnt; i++) {
            chunk[i] = (uint8_t)b->DATA;
            if (chunk[i] == 0x00u) {
                rx_wake = true;
            }
        }
        uint32_t put = ring_buf_put(&_z_serial_rx_ring, chunk, cnt);
        rx_n += cnt;
        if (put < cnt) {
            _z_serial_rx_ring_full = true;
            _z_zephyr_serial_stats.ring_overflows++;
            rx_wake = true;
        }
    }
    if (rx_n != 0u) {
        _z_zephyr_serial_stats.rx_bytes += rx_n;
        uint32_t held = ring_buf_size_get(&_z_serial_rx_ring);
        if (held > _z_zephyr_serial_stats.ring_high_water) {
            _z_zephyr_serial_stats.ring_high_water = held;
        }
        if (rx_wake || held >= (uint32_t)(_Z_ZEPHYR_SERIAL_RX_RING_BYTES / 2)) {
            k_sem_give(&_z_serial_rx_sem);
        }
    }

    /* TX only while its interrupt is enabled (TDRE is level and also true
       while the ring is idle and TIE is off). */
    if ((b->CTRL & LPUART_CTRL_TIE_MASK) == 0u || (b->STAT & LPUART_STAT_TDRE_MASK) == 0u) {
        return;
    }
    uint32_t room = _z_serial_lpuart_txfifo - ((b->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXCOUNT_SHIFT);
    uint32_t tx_n = 0;
    while (room > 0u) {
        uint8_t *data;
        uint32_t n = ring_buf_get_claim(&_z_serial_tx_ring, &data, room);
        if (n == 0u) {
            (void)ring_buf_get_finish(&_z_serial_tx_ring, 0);
            /* Disabled with the ring empty: the thread re-enables it after
               its next put, as on the generic path. */
            uart_irq_tx_disable(dev);
            break;
        }
        for (uint32_t i = 0; i < n; i++) {
            b->DATA = data[i];
        }
        (void)ring_buf_get_finish(&_z_serial_tx_ring, n);
        tx_n += n;
        room -= n;
    }
    if (tx_n != 0u) {
        _z_zephyr_serial_tx_stats.irq_bytes += tx_n;
        if (_z_serial_tx_waiting &&
            ring_buf_space_get(&_z_serial_tx_ring) >= (uint32_t)(_Z_ZEPHYR_SERIAL_TX_RING_BYTES / 2)) {
            k_sem_give(&_z_serial_tx_sem);
        }
    }
}
#endif /* _Z_ZEPHYR_SERIAL_FIFO_WATERMARK */

#if defined(_Z_ZEPHYR_SERIAL_UART_ASYNC)
/* -- the link by DMA: Zephyr's async UART API (nano-ros phase-474 I15+I16) -
 *
 * CONFIG_NROS_ZENOH_SERIAL_UART_ASYNC. The per-byte (or, with I14, per-FIFO)
 * ISR is replaced by DMA in both directions; the rings, the reader and the
 * senders are the same, only who fills and drains the rings changes.
 *
 * Both directions or neither: Zephyr keeps ONE callback API per UART when
 * both APIs are built (UART_EXCLUSIVE_API_CALLBACKS, which the mcux LPUART
 * driver requires). On that driver uart_callback_set refuses (-ENOTSUP) once
 * the interrupt callback is set, uart_irq_callback_user_data_set clears the
 * async callback, and its ISR dispatches to one of them by the API last set
 * (uart_mcux_lpuart.c, mcux_lpuart_irq_callback_set / _callback_set /
 * mcux_lpuart_isr). So TX by DMA with RX on the interrupt (I15 alone) is
 * not possible; the console's UART is a different device and keeps its
 * interrupt API.
 *
 * TX: a send still copies into the TX ring. When no transfer is in flight the
 * ring's longest contiguous run is claimed and handed to uart_tx;
 * UART_TX_DONE (the DMA ISR) finishes the claim, starts the next run and
 * gives the TX semaphore by I13's rule (only while a sender waits, and only
 * once half the ring is free). One DMA interrupt per run, i.e. per frame or
 * two at the ring's wrap.
 *
 * RX: uart_rx_enable on one of two _Z_ZEPHYR_SERIAL_ASYNC_RX_BUF_BYTES
 * buffers, UART_RX_BUF_REQUEST answered with the other. UART_RX_RDY puts the
 * new bytes into the RX ring and gives the reader's semaphore by the ISR's
 * rule (a 0x00 frame delimiter, or the ring half full). UART_RX_RDY comes
 * from the DMA ISR when a buffer fills and, for the bytes before an idle
 * line, from the driver's idle flush, which runs on the SYSTEM WORKQUEUE
 * after k_work_reschedule(K_USEC(timeout)) (uart_mcux_lpuart.c,
 * mcux_lpuart_async_isr and mcux_lpuart_async_rx_timeout): so the end of
 * every frame reaches the reader one to two system ticks after its last byte
 * (1-2 ms at CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000), however small the timeout.
 * A timeout of 0 disables that flush altogether, and a frame would wait for
 * the buffer to fill. Two producers (an ISR and a thread), so the RX ring
 * put is under a spinlock; the reader stays lock-free.
 *
 * Any RX error stops reception on that driver (UART_RX_STOPPED, then both
 * buffers released, then UART_RX_DISABLED); the reason is counted into
 * `_z_zephyr_serial_stats` and RX_DISABLED re-enables. The bytes lost make
 * a frame fail its CRC, as on the interrupt path.
 *
 * Buffers: the DMA reads the TX ring and writes the RX buffers. They are
 * this file's bss, so under CONFIG_NROS_ZENOH_SERIAL_TCM in DTCM, which is
 * not behind the Cortex-M7 D-cache; the eDMA driver converts a DTCM address
 * to the DMA's view itself (dma_mcux_edma.c EDMA_MMAP_ADDR). Without a
 * D-cache no placement is needed. */

#ifndef _Z_ZEPHYR_SERIAL_ASYNC_RX_BUF_BYTES
#define _Z_ZEPHYR_SERIAL_ASYNC_RX_BUF_BYTES 256
#endif
/* Idle time before the driver flushes a partly filled RX buffer; rounded up
   to a system tick by the driver's k_work_reschedule (see above). 25 us is
   about two characters at 921,600. */
#ifndef _Z_ZEPHYR_SERIAL_ASYNC_RX_TIMEOUT_US
#define _Z_ZEPHYR_SERIAL_ASYNC_RX_TIMEOUT_US 25
#endif

/* Debugger-readable, like the other two.
 *   active          1 while the link runs on the async API (0: it fell back
 *                   to the interrupt path, or is not bound)
 *   tx_dma          transfers started; tx_aborts, tx_errors (uart_tx refused)
 *   rx_rdy          UART_RX_RDY events (DMA buffer full or idle flush)
 *   rx_stopped      RX errors (the reason is in _z_zephyr_serial_stats)
 *   rx_rearms       re-enables after RX_DISABLED; rx_rearm_failed */
typedef struct {
    uint32_t active;
    uint32_t tx_dma;
    uint32_t tx_aborts;
    uint32_t tx_errors;
    uint32_t rx_rdy;
    uint32_t rx_stopped;
    uint32_t rx_rearms;
    uint32_t rx_rearm_failed;
} _z_zephyr_serial_async_stats_t;
volatile _z_zephyr_serial_async_stats_t _z_zephyr_serial_async_stats;

static uint8_t _z_serial_async_rx_buf[2][_Z_ZEPHYR_SERIAL_ASYNC_RX_BUF_BYTES];
/* The buffer to hand out next; both are free whenever RX is disabled. */
static uint8_t _z_serial_async_rx_next;
/* The device runs on the async API (set at bind, cleared on fallback). */
static bool _z_serial_async = false;
/* Clear while the link is being re-opened: no TX restart, no RX re-arm. */
static volatile bool _z_serial_async_run = false;
/* Bytes claimed from the TX ring by the transfer in flight; 0 = idle. */
static uint32_t _z_serial_async_tx_len = 0;
static struct k_spinlock _z_serial_async_lock;

static void _z_serial_async_tx_kick(const struct device *dev) {
    k_spinlock_key_t key = k_spin_lock(&_z_serial_async_lock);
    if (_z_serial_async_run && _z_serial_async_tx_len == 0u) {
        uint8_t *data;
        uint32_t n = ring_buf_get_claim(&_z_serial_tx_ring, &data, _Z_ZEPHYR_SERIAL_TX_RING_BYTES);
        if (n == 0u) {
            (void)ring_buf_get_finish(&_z_serial_tx_ring, 0);
        } else {
            _z_serial_async_tx_len = n;
            if (uart_tx(dev, data, n, SYS_FOREVER_US) == 0) {
                _z_zephyr_serial_async_stats.tx_dma++;
            } else {
                /* Left in the ring; the next put kicks again, and a sender
                   that waits for room times out as with stopped TX IRQs. */
                (void)ring_buf_get_finish(&_z_serial_tx_ring, 0);
                _z_serial_async_tx_len = 0;
                _z_zephyr_serial_async_stats.tx_errors++;
            }
        }
    }
    k_spin_unlock(&_z_serial_async_lock, key);
}

static void _z_serial_async_rx_push(const uint8_t *data, uint32_t len) {
    bool wake = (memchr(data, 0x00, len) != NULL);
    k_spinlock_key_t key = k_spin_lock(&_z_serial_async_lock);
    uint32_t put = ring_buf_put(&_z_serial_rx_ring, data, len);
    _z_zephyr_serial_stats.rx_bytes += len;
    if (put < len) {
        _z_serial_rx_ring_full = true;
        _z_zephyr_serial_stats.ring_overflows++;
        wake = true;
    }
    uint32_t held = ring_buf_size_get(&_z_serial_rx_ring);
    if (held > _z_zephyr_serial_stats.ring_high_water) {
        _z_zephyr_serial_stats.ring_high_water = held;
    }
    k_spin_unlock(&_z_serial_async_lock, key);
    if (wake || held >= (uint32_t)(_Z_ZEPHYR_SERIAL_RX_RING_BYTES / 2)) {
        k_sem_give(&_z_serial_rx_sem);
    }
}

static void _z_serial_async_rx_start(const struct device *dev) {
    uint8_t *buf = _z_serial_async_rx_buf[_z_serial_async_rx_next];
    _z_serial_async_rx_next ^= 1u;
    if (uart_rx_enable(dev, buf, sizeof(_z_serial_async_rx_buf[0]), _Z_ZEPHYR_SERIAL_ASYNC_RX_TIMEOUT_US) == 0) {
        _z_zephyr_serial_async_stats.rx_rearms++;
    } else {
        _z_zephyr_serial_async_stats.rx_rearm_failed++;
    }
}

static void _z_serial_async_cb_body(const struct device *dev, struct uart_event *evt) {
    switch (evt->type) {
        case UART_TX_DONE:
        case UART_TX_ABORTED: {
            k_spinlock_key_t key = k_spin_lock(&_z_serial_async_lock);
            if (_z_serial_async_tx_len != 0u) {
                uint32_t sent = (uint32_t)evt->data.tx.len;
                if (sent > _z_serial_async_tx_len) {
                    sent = _z_serial_async_tx_len;
                }
                /* A partial finish (an abort) rewinds the rest of the claim. */
                (void)ring_buf_get_finish(&_z_serial_tx_ring, sent);
                _z_serial_async_tx_len = 0;
                _z_zephyr_serial_tx_stats.irq_bytes += sent;
            }
            if (evt->type == UART_TX_ABORTED) {
                _z_zephyr_serial_async_stats.tx_aborts++;
            }
            k_spin_unlock(&_z_serial_async_lock, key);
            _z_serial_async_tx_kick(dev);
            if (_z_serial_tx_waiting &&
                ring_buf_space_get(&_z_serial_tx_ring) >= (uint32_t)(_Z_ZEPHYR_SERIAL_TX_RING_BYTES / 2)) {
                k_sem_give(&_z_serial_tx_sem);
            }
            break;
        }
        case UART_RX_RDY:
            _z_zephyr_serial_async_stats.rx_rdy++;
            _z_serial_async_rx_push(&evt->data.rx.buf[evt->data.rx.offset], (uint32_t)evt->data.rx.len);
            break;
        case UART_RX_BUF_REQUEST:
            if (_z_serial_async_run) {
                uint8_t *buf = _z_serial_async_rx_buf[_z_serial_async_rx_next];
                _z_serial_async_rx_next ^= 1u;
                (void)uart_rx_buf_rsp(dev, buf, sizeof(_z_serial_async_rx_buf[0]));
            }
            break;
        case UART_RX_STOPPED: {
            uint32_t why = (uint32_t)evt->data.rx_stop.reason;
            _z_zephyr_serial_async_stats.rx_stopped++;
            if ((why & UART_ERROR_OVERRUN) != 0u) {
                _z_zephyr_serial_stats.overruns++;
            }
            if ((why & UART_ERROR_FRAMING) != 0u) {
                _z_zephyr_serial_stats.framing_errors++;
            }
            if ((why & UART_ERROR_NOISE) != 0u) {
                _z_zephyr_serial_stats.noise_errors++;
            }
            if ((why & UART_ERROR_PARITY) != 0u) {
                _z_zephyr_serial_stats.parity_errors++;
            }
            break;
        }
        case UART_RX_DISABLED:
            if (_z_serial_async_run) {
                _z_serial_async_rx_start(dev);
            }
            break;
        default:
            break;
    }
}

static void _z_serial_async_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    ARG_UNUSED(user_data);
#if defined(_Z_ZEPHYR_SERIAL_ISR_STATS)
    /* Counts the idle flush too, which runs on the system workqueue. */
    uint32_t t0 = k_cycle_get_32();
    _z_serial_async_cb_body(dev, evt);
    _z_zephyr_serial_isr_cycles += k_cycle_get_32() - t0;
    _z_zephyr_serial_isr_calls++;
#else
    _z_serial_async_cb_body(dev, evt);
#endif
}

/* Before uart_configure() on a re-open: stop both DMA directions so the
   reconfigure (LPUART_Deinit/Init) does not run under a live transfer, and
   so neither callback restarts one. */
static void _z_serial_async_quiesce(const struct device *dev) {
    _z_serial_async_run = false;
    if (_z_serial_async_tx_len != 0u) {
        (void)uart_tx_abort(dev);
    }
    (void)uart_rx_disable(dev);
    _z_serial_async_tx_len = 0;
}

/* Every open, after uart_configure(): empty rings, then RX on buffer 0. */
static bool _z_serial_async_arm(const struct device *dev) {
    _z_serial_async_run = false;
    ring_buf_reset(&_z_serial_rx_ring);
    k_sem_reset(&_z_serial_rx_sem);
    _z_serial_rx_ring_full = false;
    ring_buf_reset(&_z_serial_tx_ring);
    k_sem_reset(&_z_serial_tx_sem);
    _z_serial_async_tx_len = 0;
    _z_serial_async_rx_next = 0;
    _z_serial_async_run = true;
    uint8_t *buf = _z_serial_async_rx_buf[_z_serial_async_rx_next];
    _z_serial_async_rx_next ^= 1u;
    if (uart_rx_enable(dev, buf, sizeof(_z_serial_async_rx_buf[0]), _Z_ZEPHYR_SERIAL_ASYNC_RX_TIMEOUT_US) != 0) {
        _z_serial_async_run = false;
        return false;
    }
    _z_serial_tx_irq = true;
    _z_zephyr_serial_async_stats.active = 1u;
    return true;
}
#endif /* _Z_ZEPHYR_SERIAL_UART_ASYNC */

/* Attach the ISR to `dev`. Idempotent for the SAME device so a reconnect does
   not have to tear down; refuses a second, different device rather than
   quietly rebinding the one link the buffer can represent. */
static bool _z_serial_rx_bind(const struct device *dev) {
    if (_z_serial_rx_dev != NULL && _z_serial_rx_dev != dev) {
        _Z_ERROR("serial RX already bound to another UART -- refusing to rebind");
        return false;
    }

    if (_z_serial_rx_dev == NULL) {
        ring_buf_init(&_z_serial_rx_ring, sizeof(_z_serial_rx_storage), _z_serial_rx_storage);
        ring_buf_init(&_z_serial_tx_ring, sizeof(_z_serial_tx_storage), _z_serial_tx_storage);
#if defined(_Z_ZEPHYR_SERIAL_UART_ASYNC)
        /* I15+I16: the async API when the device has it (-ENOSYS without
           async support, -ENOTSUP on an mcux LPUART already on the IRQ API);
           otherwise the interrupt path below, as without the option. */
        if (uart_callback_set(dev, _z_serial_async_cb, NULL) == 0) {
            _z_serial_async = true;
        } else
#endif
            if (uart_irq_callback_user_data_set(dev, _z_serial_isr, NULL) != 0) {
            _Z_ERROR("uart_irq_callback_user_data_set failed -- falling back to polled RX");
            return false;
        }
        _z_serial_rx_dev = dev;
    }

#if defined(_Z_ZEPHYR_SERIAL_UART_ASYNC)
    if (_z_serial_async) {
        if (_z_serial_async_arm(dev)) {
            return true;
        }
        /* RX DMA would not start (no channel, or the driver refused): run the
           link on the interrupt API instead, for the life of the image. */
        _Z_ERROR("serial: uart_rx_enable failed -- falling back to interrupt-driven RX/TX");
        _z_serial_async = false;
        _z_zephyr_serial_async_stats.active = 0u;
        if (uart_irq_callback_user_data_set(dev, _z_serial_isr, NULL) != 0) {
            _z_serial_rx_dev = NULL;
            _z_serial_tx_irq = false;
            return false;
        }
    }
#endif

    /* Re-arm on EVERY open, not just the first.
     *
     * `_z_serial_open_from_dev` runs `uart_configure()` immediately before this
     * on every open, and a RECONNECT is an open. On the mcux LPUART that
     * reinitialises the peripheral and clears the RX interrupt enable, while
     * this function used to return early for an already-bound device -- so RX
     * was armed on the first open and never again. One dropped session and the
     * receiver was dead for the life of the image: the handshake would succeed,
     * then not one further byte would arrive, the reader would time out once per
     * second, and the transport would reconnect forever.
     *
     * Measured on the action-server image (on the 1.10 port, where the same bug
     * exists in the file that replaced this one): 145 bytes received in total,
     * 75 read timeouts, and a ring that never held more than 19 of its 1024
     * bytes -- so this was never a buffer-capacity problem.
     *
     * Drop whatever the previous session left in the ring while re-arming. Those
     * bytes belong to a transport that no longer exists, and feeding them to the
     * new one desynchronises its COBS framing. */
    ring_buf_reset(&_z_serial_rx_ring);
    k_sem_reset(&_z_serial_rx_sem);
    _z_serial_rx_ring_full = false;
#if defined(_Z_ZEPHYR_SERIAL_FIFO_WATERMARK)
    /* I14: uart_configure() just reset WATER and FIFO; set them again. A
       device that is not an LPUART keeps the generic ISR body. */
    _z_serial_lpuart = _z_serial_lpuart_lookup(dev);
    if (_z_serial_lpuart != NULL) {
        _z_serial_lpuart_arm(_z_serial_lpuart);
    }
#endif
    uart_irq_rx_enable(dev);
    /* The same reasoning for TX: `uart_configure()` cleared the TX interrupt
       enable, and bytes a previous session left in the ring belong to a
       transport that no longer exists. Nothing is sending now (the transport
       is being opened), so the reset cannot race a producer. */
    uart_irq_tx_disable(dev);
    ring_buf_reset(&_z_serial_tx_ring);
    k_sem_reset(&_z_serial_tx_sem);
    _z_serial_tx_irq = true;
    return true;
}

/* One frame from the ring: bytes up to and including the first 0x00, or
   `cap` bytes if no delimiter came first (the byte loop's truncation, kept).
   Bytes after the delimiter stay in the ring for the next frame. Returns the
   count moved into `buf`, and sets *timed_out when the link went quiet for
   `timeout_ms` (since the last byte seen) before the frame ended. */
static size_t _z_serial_rx_frame(uint8_t *buf, size_t cap, uint32_t timeout_ms, bool *timed_out) {
    size_t n = 0;
    int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;
    *timed_out = false;
    for (;;) {
        uint8_t *run = NULL;
        uint32_t avail = ring_buf_get_claim(&_z_serial_rx_ring, &run, (uint32_t)(cap - n));
        if (avail > 0u) {
            const uint8_t *end = (const uint8_t *)memchr(run, 0x00, avail);
            uint32_t take = (end != NULL) ? (uint32_t)(end - run) + 1u : avail;
            memcpy(&buf[n], run, take);
            (void)ring_buf_get_finish(&_z_serial_rx_ring, take);
            n += take;
            if (end != NULL || n >= cap) {
                return n;
            }
            /* Progress: the quiet-link timeout counts from here. The claim may
               have stopped at the ring's wrap, so look again before sleeping. */
            deadline = k_uptime_get() + (int64_t)timeout_ms;
            continue;
        }
        (void)ring_buf_get_finish(&_z_serial_rx_ring, 0u);
        int64_t remaining = deadline - k_uptime_get();
        if (remaining <= 0) {
            *timed_out = true;
            return n;
        }
        /* A "frame end or half a ring arrived" hint, not a count: a wake is
           re-checked against the ring, and a stale give costs one loop. */
        (void)k_sem_take(&_z_serial_rx_sem, K_MSEC((uint32_t)remaining));
        _z_zephyr_serial_stats.rx_wakes++;
    }
}

/* Queue `len` bytes for the TX ISR. With room for all of them this is one copy
   and a return (the common case, nano-ros I12); otherwise it queues what fits
   and sleeps for the rest. Returns 0 when every byte is queued, -1 if no room
   appeared for _Z_ZEPHYR_SERIAL_TX_TIMEOUT_MS (TX interrupts stopped).
   Returning with bytes still queued is correct: the next frame queues behind
   them, and COBS framing does not care when a byte leaves. */
static int _z_serial_tx_put(const struct device *dev, const uint8_t *data, size_t len) {
    size_t done = 0;
    if (len > _z_zephyr_serial_tx_stats.max_frame) {
        _z_zephyr_serial_tx_stats.max_frame = (uint32_t)len;
    }
    if (ring_buf_space_get(&_z_serial_tx_ring) < (uint32_t)len) {
        _z_zephyr_serial_tx_stats.frames_waited++;
    }
    while (done < len) {
        uint32_t put = ring_buf_put(&_z_serial_tx_ring, &data[done], (uint32_t)(len - done));
        done += put;
        uint32_t held = ring_buf_size_get(&_z_serial_tx_ring);
        if (held > _z_zephyr_serial_tx_stats.ring_high_water) {
            _z_zephyr_serial_tx_stats.ring_high_water = held;
        }
#if defined(_Z_ZEPHYR_SERIAL_UART_ASYNC)
        if (_z_serial_async) {
            _z_serial_async_tx_kick(dev);
        } else
#endif
        {
            uart_irq_tx_enable(dev);
        }
        if (done < len) {
            _z_zephyr_serial_tx_stats.waits++;
            /* A "room was made" hint, like the RX sem: re-check the ring on
               every wake. A stale give from an earlier drain only costs one
               extra loop. The ISR gives only while `_z_serial_tx_waiting` is
               set and only once half the ring is free, so set it first and
               then look: room made before the flag was seen is found here,
               room made after it is given. */
            _z_serial_tx_waiting = true;
            int rc = 0;
            if (ring_buf_space_get(&_z_serial_tx_ring) < (uint32_t)(_Z_ZEPHYR_SERIAL_TX_RING_BYTES / 2)) {
                rc = k_sem_take(&_z_serial_tx_sem, K_MSEC(_Z_ZEPHYR_SERIAL_TX_TIMEOUT_MS));
            }
            _z_serial_tx_waiting = false;
            if (rc != 0 && ring_buf_space_get(&_z_serial_tx_ring) == 0u) {
                _z_zephyr_serial_tx_stats.timeouts++;
                return -1;
            }
        }
    }
    return 0;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

z_result_t _z_serial_open_from_dev(_z_sys_net_socket_t *sock, const char *dev, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;

    sock->_serial = device_get_binding(dev);
    if (sock->_serial != NULL) {
        const struct uart_config config = {
            .baudrate = baudrate,
            .parity = UART_CFG_PARITY_NONE,        // Default in Zenoh Rust
            .stop_bits = UART_CFG_STOP_BITS_1,     // Default in Zenoh Rust
            .data_bits = UART_CFG_DATA_BITS_8,     // Default in Zenoh Rust
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,  // Default in Zenoh Rust
        };
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) && defined(_Z_ZEPHYR_SERIAL_UART_ASYNC)
        if (_z_serial_async && _z_serial_rx_dev == sock->_serial) {
            _z_serial_async_quiesce(sock->_serial);
        }
#endif
        if (uart_configure(sock->_serial, &config) != 0) {
            _Z_ERROR_LOG(_Z_ERR_GENERIC);
            ret = _Z_ERR_GENERIC;
        }
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
        /* issue 0852 -- bind BEFORE the handshake (`_z_connect_serial`, which
           the serial protocol layer runs once this returns), so the peer's reply is caught by the ISR rather than raced
           for by the poll loop. A failure here is not fatal: the reader falls
           back to polling, which is what every build did before. */
        if (ret == _Z_RES_OK) {
            (void)_z_serial_rx_bind(sock->_serial);
        }
#endif
    } else {
        _Z_ERROR_LOG(_Z_ERR_GENERIC);
        ret = _Z_ERR_GENERIC;
    }

    return ret;
}

z_result_t _z_serial_listen_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    z_result_t ret = _Z_RES_OK;
    (void)(sock);
    (void)(txpin);
    (void)(rxpin);
    (void)(baudrate);

    // @TODO: To be implemented
    _Z_ERROR_LOG(_Z_ERR_GENERIC);
    ret = _Z_ERR_GENERIC;

    return ret;
}

/* Serial is symmetric: listening is opening the UART without dialling, and the
   serial protocol layer skips the INIT handshake for a listener. */
z_result_t _z_serial_listen_from_dev(_z_sys_net_socket_t *sock, const char *dev, uint32_t baudrate) {
    return _z_serial_open_from_dev(sock, dev, baudrate);
}

void _z_serial_close(_z_sys_net_socket_t *sock) {
    if (sock != NULL) {
        sock->_serial = NULL;
    }
}

/* Receive timeout: the link quiet this long (per byte on the polled path,
   since the last byte the reader saw on the interrupt-driven one). Long enough
   for a peer to answer a handshake, short enough that a lost frame is
   recoverable rather than a permanent hang. */
#ifndef _Z_ZEPHYR_SERIAL_TIMEOUT_MS
#define _Z_ZEPHYR_SERIAL_TIMEOUT_MS 1000
#endif


/* issue 0852 -- an overrun is a LOST BYTE, and it used to be silent.
   `uart_poll_in` reports "no character available", never "a character was
   destroyed before you asked", so a receiver that only checks its return value
   cannot tell a quiet link from a link it is failing to keep up with. That
   indistinguishability is what let this hide behind six wrong hypotheses,
   ending in a router bug report (issue 0848) that was wrong in every
   particular.

   The flags are sticky until read, so reading them once the frame is in hand
   still attributes a loss to the frame that was actually damaged. The polled
   path reads them per byte, as it always did; the interrupt-driven path reads
   them once per frame (a driver call per byte was part of the per-byte cost,
   see the note at the RX ring). */
static void _z_serial_rx_check_errors(const _z_sys_net_socket_t sock, size_t rb) {
    int uart_err = uart_err_check(sock._serial);
    if (uart_err > 0 && (uart_err & UART_ERROR_OVERRUN) != 0) {
        _z_zephyr_serial_stats.overruns++;
        _Z_ERROR("serial RX OVERRUN after %zu byte(s) -- frame will be dropped", rb);
    }
    if (uart_err > 0) {
        if ((uart_err & UART_ERROR_FRAMING) != 0) {
            _z_zephyr_serial_stats.framing_errors++;
        }
        if ((uart_err & UART_ERROR_NOISE) != 0) {
            _z_zephyr_serial_stats.noise_errors++;
        }
        if ((uart_err & UART_ERROR_PARITY) != 0) {
            _z_zephyr_serial_stats.parity_errors++;
        }
    }
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
    /* A DIFFERENT loss from the one above, and worth keeping apart: the
       hardware kept up and our ring did not, which says the ring is too small
       or the reader is too slow, not that the ISR is too late. */
    if (_z_serial_rx_ring_full) {
        _z_serial_rx_ring_full = false;
        _Z_ERROR("serial RX ring overflowed -- raise _Z_ZEPHYR_SERIAL_RX_RING_BYTES");
    }
#endif
}

/* Framing buffers (nano-ros I12). Send and receive used to z_malloc two
   ~1.5 KiB buffers per FRAME each (and copy the payload into a temporary
   before COBS-encoding it); they are static now, one set per direction, which
   is what one serial link per image needs (see the RX ring). The send side
   no longer needs the temporary at all (`_z_serial_msg_serialize_direct`).
   Each set has its own lock: a send and a receive run concurrently (tx-flush
   or publisher vs. the read task), two sends or two receives must not share a
   buffer, and the lock is uncontended in steady state. The receive lock is
   held across the frame wait, which is the reader's own time. */
static uint8_t _z_serial_tx_frame_buf[_Z_SERIAL_MAX_COBS_BUF_SIZE];
static K_MUTEX_DEFINE(_z_serial_tx_lock);
static uint8_t _z_serial_rx_raw_buf[_Z_SERIAL_MAX_COBS_BUF_SIZE];
static uint8_t _z_serial_rx_tmp_buf[_Z_SERIAL_MFS_SIZE];
static K_MUTEX_DEFINE(_z_serial_rx_lock);

static size_t _z_read_serial_frame_locked(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    uint8_t *raw_buf = _z_serial_rx_raw_buf;
    size_t rb = 0;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
    const bool irq_rx = (_z_serial_rx_dev != NULL && _z_serial_rx_dev == sock._serial);
#else
    const bool irq_rx = false;
#endif
    /* Bounded wait, as the flipper port does with a 200 ms
       furi_stream_buffer_receive() and the unix port with SO_RCVTIMEO: on
       expiry return SIZE_MAX, which is the contract callers already handle
       (_z_read_exact_serial breaks on it). Unbounded was the real defect --
       _z_connect_serial sends one INIT and then blocks here forever, so a lost
       INIT could never be retried and the link was unrecoverable. */
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
    if (irq_rx) {
        /* The ISR already took the bytes off the wire; this only moves the
           frame. No `k_yield()`, so no dependence on how long the executor
           holds the CPU -- that dependence was issue 0852 -- and one wake per
           frame, so no dependence of the executor on the frame's wire time
           either (nano-ros I10). */
        bool timed_out = false;
        rb = _z_serial_rx_frame(raw_buf, _Z_SERIAL_MAX_COBS_BUF_SIZE, _Z_ZEPHYR_SERIAL_TIMEOUT_MS, &timed_out);
        if (timed_out) {
            if (rb > 0) {
                _z_zephyr_serial_stats.rx_partial++;
            }
            return SIZE_MAX;
        }
        _z_serial_rx_check_errors(sock, rb);
    } else
#endif
    {
        for (size_t i = 0; i < _Z_SERIAL_MAX_COBS_BUF_SIZE; i++) {
            /* Per-byte timeout. Yield rather than sleep between polls: at
               115200 a byte is 87 us and the shortest z_sleep_ms(1) blocks a
               whole tick, overrunning the UART. */
            int64_t deadline = k_uptime_get() + (int64_t)_Z_ZEPHYR_SERIAL_TIMEOUT_MS;
            int res = -1;
            while (res != 0) {
                res = uart_poll_in(sock._serial, &raw_buf[i]);
                if (res != 0) {
                    if (k_uptime_get() > deadline) {
                        return SIZE_MAX;
                    }
                    k_yield();
                }
            }
            _z_serial_rx_check_errors(sock, rb);
            rb++;
            if (raw_buf[i] == (uint8_t)0x00) {
                break;
            }
        }
    }

    size_t ret =
        _z_serial_msg_deserialize(raw_buf, rb, ptr, len, header, _z_serial_rx_tmp_buf, sizeof(_z_serial_rx_tmp_buf));
    if (ret == SIZE_MAX) {
        _z_zephyr_serial_stats.rx_bad_frames++;
    } else {
        _z_zephyr_serial_stats.rx_frames++;
    }

    return ret;
}

size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    (void)k_mutex_lock(&_z_serial_rx_lock, K_FOREVER);
    size_t ret = _z_read_serial_frame_locked(sock, header, ptr, len);
    (void)k_mutex_unlock(&_z_serial_rx_lock);
    return ret;
}

static size_t _z_send_serial_frame_locked(const _z_sys_net_socket_t sock, uint8_t header, const uint8_t *ptr,
                                          size_t len) {
    uint8_t *raw_buf = _z_serial_tx_frame_buf;
    size_t ret = _z_serial_msg_serialize_direct(raw_buf, sizeof(_z_serial_tx_frame_buf), ptr, len, header);
    if (ret == SIZE_MAX) {
        return ret;
    }

    uint32_t t0 = k_cycle_get_32();
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
    if (_z_serial_tx_irq && _z_serial_rx_dev == sock._serial) {
        /* tx_busy_cycles now counts the time the sender spent QUEUEING,
           sleeping included -- no longer CPU burnt, since a waiting sender
           is off the CPU. */
        if (_z_serial_tx_put(sock._serial, raw_buf, ret) != 0) {
            _Z_ERROR("serial TX stalled: no room in the TX ring for %u ms", (unsigned)_Z_ZEPHYR_SERIAL_TX_TIMEOUT_MS);
            _z_zephyr_serial_stats.tx_busy_cycles += k_cycle_get_32() - t0;
            return SIZE_MAX;
        }
    } else
#endif
    {
        for (size_t i = 0; i < ret; i++) {
            uart_poll_out(sock._serial, raw_buf[i]);
        }
    }
    _z_zephyr_serial_stats.tx_busy_cycles += k_cycle_get_32() - t0;
    _z_zephyr_serial_stats.tx_bytes += (uint32_t)ret;
    _z_zephyr_serial_stats.tx_frames++;

    return len;
}

size_t _z_send_serial_internal(const _z_sys_net_socket_t sock, uint8_t header, const uint8_t *ptr, size_t len) {
    (void)k_mutex_lock(&_z_serial_tx_lock, K_FOREVER);
    size_t ret = _z_send_serial_frame_locked(sock, header, ptr, len);
    (void)k_mutex_unlock(&_z_serial_tx_lock);
    return ret;
}
#endif /* Z_FEATURE_LINK_SERIAL == 1 && defined(ZENOH_ZEPHYR) */
