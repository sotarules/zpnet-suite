/*
 * pps_poll — Pi-side PPS detection via chrony-disciplined clock polling
 *
 * Dedicates a CPU core to busy-polling clock_gettime(CLOCK_REALTIME).
 * The instant the second rolls over, captures CNTVCT_EL0 (ARM arch timer).
 *
 * This exploits chrony's ~39 ns discipline of the system clock to detect
 * PPS-equivalent second boundaries without any GPIO interrupt overhead.
 *
 * Architecture (Pi-side TDC equivalent):
 *   • The polling loop acts as the "spin loop" (Teensy equivalent)
 *   • clock_gettime detects the second boundary (PPS equivalent)
 *   • detect_ns measures how late we detected it (dispatch latency)
 *   • We correct the raw counter by subtracting detect_ns in ticks
 *   • This is conceptually identical to the Teensy's software TDC
 *
 * Correction math:
 *   correction_ticks = detect_ns * freq / 1,000,000,000
 *   corrected_counter = raw_counter - correction_ticks
 *
 *   At 54 MHz: 1 ns ≈ 0.054 ticks, so detect_ns=40 → correction=2 ticks
 *   The correction is small but principled — it removes the polling
 *   loop's detection latency from the timestamp.
 *
 * Usage:
 *   gcc -O2 -o pps_poll pps_poll.c -lm
 *   sudo taskset -c 3 ./pps_poll [count]
 *
 * Default count: 30 captures.  Use a very large number for daemon mode.
 * Each line is flushed immediately for subprocess consumption.
 *
 * Output format (tab-separated fields):
 *   seq  counter  corrected  delta  residual  resid_ns  detect_ns  correction_ticks  realtime_ns
 *
 *   seq               — capture sequence number
 *   counter           — raw CNTVCT_EL0 at detection moment
 *   corrected         — counter minus detection latency (best-estimate of true PPS edge)
 *   delta             — ticks since previous corrected capture (--- on first)
 *   residual          — delta minus expected frequency (--- on first)
 *   resid_ns          — residual in nanoseconds (--- on first)
 *   detect_ns         — nanoseconds after second boundary when detected (polling latency)
 *   correction_ticks  — ticks subtracted from raw counter (detect_ns * freq / 1e9)
 *   realtime_ns       — raw tv_nsec from clock_gettime (should be near zero)
 *
 * Running statistics (Welford's) are appended after each line with n >= 2.
 * Statistics are computed on CORRECTED residuals.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

/* --------------------------------------------------------------------
 * Read ARM Generic Timer (CNTVCT_EL0) — single MRS instruction
 * Same counter that systimer.read() uses from Python.
 * -------------------------------------------------------------------- */
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

/* Read ARM Generic Timer frequency (CNTFRQ_EL0) */
static inline uint64_t read_cntfrq(void)
{
    uint64_t val;
    __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(val));
    return val;
}

/* --------------------------------------------------------------------
 * Welford's online algorithm for running statistics
 * -------------------------------------------------------------------- */
typedef struct {
    int64_t n;
    double mean;
    double m2;
} welford_t;

static void welford_init(welford_t *w)
{
    w->n = 0;
    w->mean = 0.0;
    w->m2 = 0.0;
}

static void welford_update(welford_t *w, double x)
{
    w->n++;
    double d1 = x - w->mean;
    w->mean += d1 / (double)w->n;
    double d2 = x - w->mean;
    w->m2 += d1 * d2;
}

static double welford_stddev(const welford_t *w)
{
    if (w->n < 2) return 0.0;
    return sqrt(w->m2 / (double)(w->n - 1));
}

static double welford_stderr(const welford_t *w)
{
    if (w->n < 2) return 0.0;
    return welford_stddev(w) / sqrt((double)w->n);
}

/* -------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
    int count = 30;
    if (argc > 1)
        count = atoi(argv[1]);
    if (count < 2)
        count = 2;

    /* Force line buffering on stdout so each capture line is
     * immediately available to a parent process reading our pipe. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    uint64_t freq = read_cntfrq();
    double ns_per_tick = 1e9 / (double)freq;

    printf("Polling clock_gettime for second boundaries (freq = %lu Hz, %.3f ns/tick)\n",
           (unsigned long)freq, ns_per_tick);
    printf("Captures requested: %d\n\n", count);

    printf("  %4s  %20s  %20s  %12s  %10s  %12s  %10s  %10s  %10s\n",
           "seq", "counter", "corrected", "delta", "residual", "resid_ns",
           "detect_ns", "corr_ticks", "rt_ns");
    printf("  %s\n",
           "----------------------------------------------------"
           "----------------------------------------------------"
           "--------------------");

    struct timespec ts;
    uint64_t last_corrected = 0;
    time_t last_sec = 0;
    int captured = 0;
    welford_t stats;
    welford_init(&stats);

    /* Prime: get the current second */
    clock_gettime(CLOCK_REALTIME, &ts);
    last_sec = ts.tv_sec;

    while (captured < count) {
        /* Tight poll — this is the hot loop.
         * clock_gettime(CLOCK_REALTIME) uses the vDSO on ARM64,
         * so it's a userspace-only call (no syscall). */
        clock_gettime(CLOCK_REALTIME, &ts);

        if (ts.tv_sec != last_sec) {
            /* Second just rolled over — grab the counter NOW */
            uint64_t counter = read_cntvct();

            /* ts.tv_nsec tells us how late we detected the rollover.
             * Ideally this would be 0 (detected at exactly .000000000).
             * In practice it's the polling loop period — tens of ns. */
            long detect_ns = ts.tv_nsec;

            /* Correct the counter by subtracting detection latency.
             * This is the Pi-side TDC correction:
             *   correction_ticks = detect_ns * freq / 1e9
             *   corrected = counter - correction_ticks
             *
             * Integer arithmetic to avoid floating-point rounding:
             *   correction = detect_ns * freq / 1,000,000,000
             */
            uint64_t correction_ticks = ((uint64_t)detect_ns * freq) / 1000000000ULL;
            uint64_t corrected = counter - correction_ticks;

            if (last_corrected > 0) {
                int64_t delta = (int64_t)(corrected - last_corrected);
                int64_t residual = delta - (int64_t)freq;
                double residual_ns = (double)residual * ns_per_tick;

                welford_update(&stats, residual_ns);

                printf("  %4d  %20lu  %20lu  %12ld  %10ld  %12.3f  %10ld  %10lu  %10ld",
                       captured,
                       (unsigned long)counter,
                       (unsigned long)corrected,
                       (long)delta,
                       (long)residual,
                       residual_ns,
                       detect_ns,
                       (unsigned long)correction_ticks,
                       detect_ns);

                if (stats.n >= 2) {
                    printf("  (mean=%.3f sd=%.3f se=%.3f n=%ld)",
                           stats.mean,
                           welford_stddev(&stats),
                           welford_stderr(&stats),
                           (long)stats.n);
                }
                printf("\n");
            } else {
                printf("  %4d  %20lu  %20lu  %12s  %10s  %12s  %10ld  %10lu  %10ld\n",
                       captured,
                       (unsigned long)counter,
                       (unsigned long)corrected,
                       "---", "---", "---",
                       detect_ns,
                       (unsigned long)correction_ticks,
                       detect_ns);
            }

            last_corrected = corrected;
            last_sec = ts.tv_sec;
            captured++;
        }
    }

    printf("\n--- %ld residuals captured (corrected) ---\n", (long)stats.n);
    printf("mean = %.3f ns\n", stats.mean);
    printf("sd   = %.3f ns\n", welford_stddev(&stats));
    printf("se   = %.3f ns\n", welford_stderr(&stats));

    return 0;
}