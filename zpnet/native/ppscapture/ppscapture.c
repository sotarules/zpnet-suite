/*
 * ppscapture.c — Minimal PPS second-boundary capture via clock polling
 *
 * Single function: ppscapture_next()
 *   Busy-polls clock_gettime(CLOCK_REALTIME) until the second rolls over,
 *   then captures CNTVCT_EL0 and returns the raw counter + realtime.
 *
 * Caller is responsible for:
 *   - Running on an isolated core (taskset)
 *   - Applying TDC correction (detect_ns * freq / 1e9)
 *   - All sequencing, pps_count assignment, statistics
 *
 * This library does ONE thing: detect the boundary and capture the tick.
 *
 * Build:
 *   gcc -shared -fPIC -O2 -o libppscapture.so ppscapture.c
 *
 * Usage from Python (ctypes):
 *   lib = ctypes.CDLL("libppscapture.so")
 *   counter = ctypes.c_uint64()
 *   detect_ns = ctypes.c_int64()
 *   realtime_sec = ctypes.c_int64()
 *   realtime_nsec = ctypes.c_int64()
 *   lib.ppscapture_next(ctypes.byref(counter), ctypes.byref(detect_ns),
 *                        ctypes.byref(realtime_sec), ctypes.byref(realtime_nsec))
 */

#define _GNU_SOURCE
#include <stdint.h>
#include <time.h>

/* Read ARM Generic Timer (CNTVCT_EL0) — single MRS instruction */
static inline uint64_t read_cntvct(void)
{
    uint64_t val;
    __asm__ __volatile__(
        "isb\n\t"
        "mrs %0, cntvct_el0"
        : "=r"(val)
        :
        : "memory"
    );
    return val;
}

/*
 * ppscapture_next — block until the next second boundary, capture counter.
 *
 * Parameters (out):
 *   counter_out     — raw CNTVCT_EL0 value at detection moment
 *   detect_ns_out   — nanoseconds after the boundary (polling latency)
 *   rt_sec_out      — clock_gettime tv_sec at detection (Unix epoch seconds)
 *   rt_nsec_out     — clock_gettime tv_nsec at detection (chrony-disciplined)
 *
 * Returns:
 *   0 on success
 */
int ppscapture_next(uint64_t *counter_out, int64_t *detect_ns_out,
                    int64_t *rt_sec_out, int64_t *rt_nsec_out)
{
    struct timespec ts;

    /* Get current second */
    clock_gettime(CLOCK_REALTIME, &ts);
    time_t last_sec = ts.tv_sec;

    /* Spin until second rolls over */
    while (1) {
        clock_gettime(CLOCK_REALTIME, &ts);
        if (ts.tv_sec != last_sec) {
            /* Boundary detected — capture counter immediately */
            *counter_out = read_cntvct();
            *detect_ns_out = (int64_t)ts.tv_nsec;
            *rt_sec_out = (int64_t)ts.tv_sec;
            *rt_nsec_out = (int64_t)ts.tv_nsec;
            return 0;
        }
    }
}