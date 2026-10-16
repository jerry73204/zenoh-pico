/* Ubiquity robotics
 * =============================================================
 * Zenoh-pico stm32 threadx
 * System implementation for threadx.
 * =============================================================
 */
#if defined(ZENOH_THREADX_STM32)
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "tx_api.h"
#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/logging.h"

/* ThreadX counts TX_TIMER_TICKS_PER_SECOND ticks a second, 100 by default, so
 * every conversion multiplies before it divides: TX_TIMER_TICKS_PER_SECOND /
 * 1000 is 0 at the default rate. Durations round UP to whole ticks, so a sleep
 * or a deadline is never shorter than asked for. */
static uint64_t _z_ticks_to_ms(uint64_t ticks) { return ticks * 1000u / TX_TIMER_TICKS_PER_SECOND; }

static uint64_t _z_us_to_ticks64_ceil(uint64_t us) { return (us * TX_TIMER_TICKS_PER_SECOND + 999999u) / 1000000u; }

static ULONG _z_us_to_ticks_ceil(uint64_t us) { return (ULONG)_z_us_to_ticks64_ceil(us); }

/* tx_time_get() is a 32-bit counter, so it wraps: every 497 days at the
 * default 100 Hz, every 49.7 days at 1 kHz. z_clock_t is absolute time, so a
 * clock read straight from it jumps back to 0 at the wrap, and every deadline
 * and elapsed time spanning it is wrong. Extend it to 64 bits by counting the
 * wraps: a reading below the previous one means the counter wrapped since.
 * That holds as long as the clock is read at least once per wrap period, which
 * zenoh-pico's lease and read tasks do many times a second. The read and the
 * update run with interrupts disabled, so two threads cannot both count one
 * wrap, and neither can save a stale reading over a newer one. */
static uint64_t _z_ticks64(void) {
    if (sizeof(ULONG) >= sizeof(uint64_t)) {
        return (uint64_t)tx_time_get(); /* an LP64 port: ULONG is already 64 bits */
    }
    static ULONG last_ticks = 0;
    static ULONG wraps = 0;
    UINT posture = tx_interrupt_control(TX_INT_DISABLE);
    ULONG now = tx_time_get();
    if (now < last_ticks) {
        wraps++;
    }
    last_ticks = now;
    uint64_t ticks = ((uint64_t)wraps << 32) | (uint64_t)now;
    tx_interrupt_control(posture);
    return ticks;
}

/* pointer to threadx byte pool, should be provided by application */
extern TX_BYTE_POOL *pthreadx_byte_pool;

/*------------------ Random ------------------*/
uint8_t z_random_u8(void) { return z_random_u32(); }

uint16_t z_random_u16(void) { return z_random_u32(); }

uint32_t z_random_u32(void) { return random(); }

uint64_t z_random_u64(void) {
    uint64_t ret = 0;
    ret |= z_random_u32();
    ret = ret << 32;
    ret |= z_random_u32();
    return ret;
}

void z_random_fill(void *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        ((uint8_t *)buf)[i] = z_random_u8();
    }
}

/*------------------ Memory ------------------*/
void *z_malloc(size_t size) {
    void *ptr = NULL;

    uint8_t r = tx_byte_allocate(pthreadx_byte_pool, &ptr, size, TX_WAIT_FOREVER);
    if (r != TX_SUCCESS) {
        ptr = NULL;
    }
    return ptr;
}

void *z_realloc(void *ptr, size_t size) {
    // realloc not implemented
    return NULL;
}

void z_free(void *ptr) { tx_byte_release(ptr); }

#if Z_FEATURE_MULTI_THREAD == 1

/*------------------ Thread ------------------*/
z_result_t _z_task_init(_z_task_t *task, z_task_attr_t *attr, void *(*fun)(void *), void *arg) {
    _Z_DEBUG("Creating a new task!");

    UINT status = tx_thread_create(&(task->threadx_thread), "ztask", (VOID(*)(ULONG))fun, (ULONG)arg,
                                   task->threadx_stack, Z_TASK_STACK_SIZE, Z_TASK_PRIORITY, Z_TASK_PREEMPT_THRESHOLD,
                                   Z_TASK_TIME_SLICE, TX_AUTO_START);

    if (status != TX_SUCCESS) _Z_ERROR_RETURN(_Z_ERR_GENERIC);

    return _Z_RES_OK;
}

z_result_t _z_task_join(_z_task_t *task) {
    while (1) {
        UINT state;
        UINT status = tx_thread_info_get(&(task->threadx_thread), NULL, &state, NULL, NULL, NULL, NULL, NULL, NULL);
        if (status != TX_SUCCESS) _Z_ERROR_RETURN(_Z_ERR_GENERIC);

        if ((state == TX_COMPLETED) || (state == TX_TERMINATED)) break;

        tx_thread_sleep(1);
    }

    return _Z_RES_OK;
}

z_result_t _z_task_detach(_z_task_t *task) {
    // Not implemented
    _Z_ERROR_RETURN(_Z_ERR_GENERIC);
}

z_result_t _z_task_cancel(_z_task_t *task) {
    // Not implemented
    _Z_ERROR_RETURN(_Z_ERR_GENERIC);
}

void _z_task_exit(void) {  // NEW with new vesion
    // Not implemented
}

void _z_task_free(_z_task_t **task) {
    z_free(*task);
    *task = NULL;
}

_z_task_id_t _z_task_get_id(const _z_task_t *task) { return (TX_THREAD *)&(task->threadx_thread); }
_z_task_id_t _z_task_current_id(void) { return tx_thread_identify(); }
bool _z_task_id_equal(const _z_task_id_t *l, const _z_task_id_t *r) { return *l == *r; }

/*------------------ Mutex ------------------*/
z_result_t _z_mutex_init(_z_mutex_t *m) {
    UINT status = tx_mutex_create(m, TX_NULL, TX_INHERIT);
    if (status == TX_MUTEX_ERROR) {
        // zenoh-pico reuses mutex if zenoh_init() fails.
        status = tx_mutex_delete(m);
        if (status == TX_SUCCESS) {
            status = tx_mutex_create(m, TX_NULL, TX_INHERIT);
        }
    }
    return (status == TX_SUCCESS) ? 0 : -1;
}
z_result_t _z_mutex_drop(_z_mutex_t *m) {
    UINT status = tx_mutex_delete(m);
    return (status == TX_SUCCESS) ? 0 : -1;
}
z_result_t _z_mutex_lock(_z_mutex_t *m) {
    UINT status = tx_mutex_get(m, TX_WAIT_FOREVER);
    return (status == TX_SUCCESS) ? 0 : -1;
}
z_result_t _z_mutex_try_lock(_z_mutex_t *m) {
    UINT status = tx_mutex_get(m, TX_NO_WAIT);  // Return immediately even if the mutex was not available
    return (status == TX_SUCCESS) ? 0 : -1;
}
z_result_t _z_mutex_unlock(_z_mutex_t *m) {
    UINT status = tx_mutex_put(m);
    return (status == TX_SUCCESS) ? 0 : -1;
}

z_result_t _z_mutex_rec_init(_z_mutex_rec_t *m) { return _z_mutex_init(m); }
z_result_t _z_mutex_rec_drop(_z_mutex_rec_t *m) { return _z_mutex_drop(m); }
z_result_t _z_mutex_rec_lock(_z_mutex_rec_t *m) { return _z_mutex_lock(m); }
z_result_t _z_mutex_rec_unlock(_z_mutex_rec_t *m) { return _z_mutex_unlock(m); }

/*------------------ CondVar ------------------*/
#define CONDVAR_MAX_WAITERS_COUNT 0xff;

z_result_t _z_condvar_init(_z_condvar_t *cv) {
    if (!cv) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    UINT m_status = tx_mutex_create(&cv->mutex, TX_NULL, TX_INHERIT);
    UINT s_status = tx_semaphore_create(&cv->sem, TX_NULL, 0);
    cv->waiters = 0;

    if (m_status != TX_SUCCESS || s_status != TX_SUCCESS) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    return _Z_RES_OK;
}

z_result_t _z_condvar_drop(_z_condvar_t *cv) {
    if (!cv) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    tx_mutex_delete(&cv->mutex);
    tx_semaphore_delete(&cv->sem);
    return _Z_RES_OK;
}

z_result_t _z_condvar_signal(_z_condvar_t *cv) {
    if (!cv) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    tx_mutex_get(&cv->mutex, TX_WAIT_FOREVER);
    if (cv->waiters > 0) {
        tx_semaphore_put(&cv->sem);
        cv->waiters--;
    }
    tx_mutex_put(&cv->mutex);

    return _Z_RES_OK;
}

z_result_t _z_condvar_signal_all(_z_condvar_t *cv) {
    if (!cv) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    tx_mutex_get(&cv->mutex, TX_WAIT_FOREVER);
    while (cv->waiters > 0) {
        tx_semaphore_put(&cv->sem);
        cv->waiters--;
    }
    tx_mutex_put(&cv->mutex);

    return _Z_RES_OK;
}

z_result_t _z_condvar_wait(_z_condvar_t *cv, _z_mutex_t *m) {
    if (!cv || !m) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    tx_mutex_get(&cv->mutex, TX_WAIT_FOREVER);
    cv->waiters++;
    tx_mutex_put(&cv->mutex);

    _z_mutex_unlock(m);
    tx_semaphore_get(&cv->sem, TX_WAIT_FOREVER);

    _z_mutex_lock(m);

    return _Z_RES_OK;
}

z_result_t _z_condvar_wait_until(_z_condvar_t *cv, _z_mutex_t *m, const z_clock_t *abstime) {
    if (!cv || !m) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    /* abstime comes from z_clock_now(), which reads the 64-bit tick count, so
     * the deadline is compared there too and never in the wrapping 32-bit
     * counter. A deadline already passed blocks for 0 ticks. A wait longer than
     * ThreadX can express is capped just below TX_WAIT_FOREVER, so it times out
     * early (after 497 days at 100 Hz) rather than blocking forever. */
    uint64_t target_time =
        _z_us_to_ticks64_ceil((uint64_t)abstime->tv_sec * 1000000u + ((uint64_t)abstime->tv_nsec + 999u) / 1000u);
    uint64_t now = _z_ticks64();
    uint64_t remaining = target_time > now ? target_time - now : 0;
    ULONG block_duration = remaining < (uint64_t)TX_WAIT_FOREVER ? (ULONG)remaining : (ULONG)(TX_WAIT_FOREVER - 1u);

    tx_mutex_get(&cv->mutex, TX_WAIT_FOREVER);
    cv->waiters++;
    tx_mutex_put(&cv->mutex);

    _z_mutex_unlock(m);

    bool timed_out = tx_semaphore_get(&cv->sem, block_duration) != TX_SUCCESS;

    _z_mutex_lock(m);

    if (timed_out) {
        tx_mutex_get(&cv->mutex, TX_WAIT_FOREVER);
        cv->waiters--;
        tx_mutex_put(&cv->mutex);
        return Z_ETIMEDOUT;
    }

    return _Z_RES_OK;
}
#endif  // Z_MULTI_THREAD == 1

/*------------------ Sleep ------------------*/
z_result_t z_sleep_us(size_t time) {
    tx_thread_sleep(_z_us_to_ticks_ceil((uint64_t)time));
    return 0;
}

z_result_t z_sleep_ms(size_t time) {
    tx_thread_sleep(_z_us_to_ticks_ceil((uint64_t)time * 1000u));
    return 0;
}

z_result_t z_sleep_s(size_t time) {
    tx_thread_sleep((ULONG)((uint64_t)time * TX_TIMER_TICKS_PER_SECOND));
    return 0;
}

/*------------------ Clock ------------------*/

void __z_clock_gettime(z_clock_t *ts) {
    uint64_t ms = _z_ticks_to_ms(_z_ticks64());
    ts->tv_sec = ms / (uint64_t)1000;
    ts->tv_nsec = (ms % (uint64_t)1000) * (uint64_t)1000000;
}

z_clock_t z_clock_now(void) {
    z_clock_t now;
    __z_clock_gettime(&now);
    return now;
}

unsigned long zp_clock_elapsed_us_since(z_clock_t *instant, z_clock_t *epoch) {
    long elapsed = (1000000 * (instant->tv_sec - epoch->tv_sec) + (instant->tv_nsec - epoch->tv_nsec) / 1000);
    return elapsed > 0 ? (unsigned long)elapsed : 0;
}

unsigned long zp_clock_elapsed_ms_since(z_clock_t *instant, z_clock_t *epoch) {
    long elapsed = (1000 * (instant->tv_sec - epoch->tv_sec) + (instant->tv_nsec - epoch->tv_nsec) / 1000000);
    return elapsed > 0 ? (unsigned long)elapsed : 0;
}

unsigned long zp_clock_elapsed_s_since(z_clock_t *instant, z_clock_t *epoch) {
    long elapsed = (instant->tv_sec - epoch->tv_sec);
    return elapsed > 0 ? (unsigned long)elapsed : 0;
}

unsigned long z_clock_elapsed_us(z_clock_t *instant) {
    z_clock_t now;
    __z_clock_gettime(&now);
    return zp_clock_elapsed_us_since(&now, instant);
}

unsigned long z_clock_elapsed_ms(z_clock_t *instant) {
    z_clock_t now;
    __z_clock_gettime(&now);

    return zp_clock_elapsed_ms_since(&now, instant);
}

unsigned long z_clock_elapsed_s(z_clock_t *instant) {
    z_clock_t now;
    __z_clock_gettime(&now);

    return zp_clock_elapsed_s_since(&now, instant);
}

void z_clock_advance_us(z_clock_t *clock, unsigned long duration) {
    clock->tv_sec += duration / 1000000;
    clock->tv_nsec += (duration % 1000000) * 1000;

    if (clock->tv_nsec >= 1000000000) {
        clock->tv_sec += 1;
        clock->tv_nsec -= 1000000000;
    }
}

void z_clock_advance_ms(z_clock_t *clock, unsigned long duration) {
    clock->tv_sec += duration / 1000;
    clock->tv_nsec += (duration % 1000) * 1000000;

    if (clock->tv_nsec >= 1000000000) {
        clock->tv_sec += 1;
        clock->tv_nsec -= 1000000000;
    }
}

void z_clock_advance_s(z_clock_t *clock, unsigned long duration) { clock->tv_sec += duration; }

/*------------------ Time ------------------*/
z_time_t z_time_now(void) { return tx_time_get(); }

const char *z_time_now_as_str(char *const buf, unsigned long buflen) {
    snprintf(buf, buflen, "%lu", z_time_now());
    return buf;
}

unsigned long z_time_elapsed_us(z_time_t *time) {
    return (tx_time_get() - *time) * 1000000ULL / TX_TIMER_TICKS_PER_SECOND;
}

unsigned long z_time_elapsed_ms(z_time_t *time) {
    return (tx_time_get() - *time) * 1000ULL / TX_TIMER_TICKS_PER_SECOND;
}

unsigned long z_time_elapsed_s(z_time_t *time) { return (tx_time_get() - *time) / TX_TIMER_TICKS_PER_SECOND; }

z_result_t _z_get_time_since_epoch(_z_time_since_epoch *t) {
    ULONG64 time_ns = tx_time_get() * 1000000000ULL / TX_TIMER_TICKS_PER_SECOND;

    t->secs = time_ns / 1000000000ULL;
    t->nanos = time_ns % 1000000000ULL;

    return _Z_RES_OK;
}
#endif  // ZENOH_THREADX_STM32
