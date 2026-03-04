/*
 * gpio_signal_diag.c — Raw GPIO 25 signal quality diagnostic
 *
 * Reads GPIO 25 as fast as possible for N samples, recording
 * the raw bit value and CNTVCT_EL0 at every transition.  Then
 * prints a complete analysis of signal quality:
 *
 *   - Total HIGH phases and LOW phases observed
 *   - Duration histogram of each phase (in nanoseconds)
 *   - Glitch detection: any phase shorter than a threshold
 *   - First N transitions printed for visual inspection
 *
 * This is a pure diagnostic — no ring buffer, no PPS, no
 * infrastructure.  Just: what is the GPIO pin doing?
 *
 * Build:
 *   gcc -O2 -o gpio_signal_diag gpio_signal_diag.c
 *
 * Run:
 *   sudo taskset -c 3 ./gpio_signal_diag
 *
 * (Or from Python test runner — see companion script)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sched.h>

/* ================================================================
 * ARM Generic Timer
 * ================================================================ */

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

/* Fast version without ISB — for burst sampling where we want
 * maximum read rate and don't need cycle-accurate timestamps */
static inline uint64_t read_cntvct_fast(void)
{
    uint64_t val;
    __asm__ __volatile__(
        "mrs %0, cntvct_el0"
        : "=r"(val)
    );
    return val;
}

/* ================================================================
 * BCM2711 GPIO
 * ================================================================ */

#define GPIO_BLOCK_SIZE     0x1000
#define GPLEV0_OFFSET       0x34
#define GPIO_PIN            25
#define GPIO_MASK           (1u << GPIO_PIN)

static volatile uint32_t *gplev0 = NULL;

static int gpio_init(void)
{
    int fd = open("/dev/gpiomem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("open /dev/gpiomem");
        return -1;
    }

    void *map = mmap(NULL, GPIO_BLOCK_SIZE,
                     PROT_READ, MAP_SHARED, fd, 0);
    close(fd);

    if (map == MAP_FAILED) {
        perror("mmap");
        return -1;
    }

    gplev0 = (volatile uint32_t *)((char *)map + GPLEV0_OFFSET);
    return 0;
}

static inline int gpio_read(void)
{
    return (*gplev0 & GPIO_MASK) ? 1 : 0;
}

/* ================================================================
 * Transition record
 * ================================================================ */

typedef struct {
    uint64_t cntvct;    /* Counter at transition */
    int      new_level; /* Level AFTER transition (0 or 1) */
} transition_t;

/* Max transitions to record */
#define MAX_TRANSITIONS  200000

static transition_t transitions[MAX_TRANSITIONS];

/* ================================================================
 * Phase duration histogram
 * ================================================================ */

/* Histogram buckets in microseconds */
#define HIST_BUCKETS    201   /* 0..200 µs, bucket[i] = count of phases i µs long */
static uint64_t hist_high[HIST_BUCKETS];
static uint64_t hist_low[HIST_BUCKETS];

#define PI_TIMER_FREQ   54000000
#define NS_PER_TICK     (1e9 / PI_TIMER_FREQ)

/* ================================================================
 * Main
 * ================================================================ */

int main(int argc, char *argv[])
{
    int num_transitions = 100000;  /* default */
    if (argc > 1)
        num_transitions = atoi(argv[1]);
    if (num_transitions > MAX_TRANSITIONS)
        num_transitions = MAX_TRANSITIONS;
    if (num_transitions < 100)
        num_transitions = 100;

    printf("GPIO 25 Signal Quality Diagnostic\n");
    printf("==================================\n\n");

    if (gpio_init() != 0) {
        fprintf(stderr, "Failed to init GPIO\n");
        return 1;
    }

    /* Set SCHED_FIFO if possible */
    struct sched_param sp = { .sched_priority = 50 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        printf("⚠️  Could not set SCHED_FIFO (run with sudo)\n");
    }

    printf("Capturing %d transitions on GPIO 25...\n\n", num_transitions);

    /* ============================================================
     * CAPTURE PHASE: record every transition as fast as possible
     * ============================================================ */

    int prev = gpio_read();
    int count = 0;

    /* Warm up — wait for a clean edge to start */
    while (gpio_read() == prev)
        ;
    prev = gpio_read();

    uint64_t start_cntvct = read_cntvct_fast();

    while (count < num_transitions) {
        int level = gpio_read();
        if (level != prev) {
            transitions[count].cntvct = read_cntvct_fast();
            transitions[count].new_level = level;
            prev = level;
            count++;
        }
    }

    uint64_t end_cntvct = read_cntvct_fast();

    /* ============================================================
     * ANALYSIS PHASE
     * ============================================================ */

    double total_time_ns = (end_cntvct - start_cntvct) * NS_PER_TICK;
    double total_time_s = total_time_ns / 1e9;

    printf("Captured %d transitions in %.3f ms\n", count, total_time_ns / 1e6);
    printf("Average transitions/sec: %.0f\n", count / total_time_s);
    printf("Expected (10 KHz square wave): 20000 transitions/sec\n\n");

    /* Compute phase durations */
    int rising_count = 0;
    int falling_count = 0;
    int glitch_count = 0;
    double glitch_threshold_ns = 10000.0;  /* 10 µs — anything shorter is a glitch */

    double min_high_ns = 1e18, max_high_ns = 0;
    double min_low_ns = 1e18, max_low_ns = 0;
    double sum_high_ns = 0, sum_low_ns = 0;
    int high_phases = 0, low_phases = 0;

    memset(hist_high, 0, sizeof(hist_high));
    memset(hist_low, 0, sizeof(hist_low));

    for (int i = 1; i < count; i++) {
        double duration_ns = (transitions[i].cntvct - transitions[i-1].cntvct) * NS_PER_TICK;
        int phase_was_high = transitions[i].new_level == 0;  /* transition TO low = phase WAS high */

        int bucket = (int)(duration_ns / 1000.0);  /* µs bucket */
        if (bucket >= HIST_BUCKETS)
            bucket = HIST_BUCKETS - 1;

        if (phase_was_high) {
            hist_high[bucket]++;
            high_phases++;
            sum_high_ns += duration_ns;
            if (duration_ns < min_high_ns) min_high_ns = duration_ns;
            if (duration_ns > max_high_ns) max_high_ns = duration_ns;
        } else {
            hist_low[bucket]++;
            low_phases++;
            sum_low_ns += duration_ns;
            if (duration_ns < min_low_ns) min_low_ns = duration_ns;
            if (duration_ns > max_low_ns) max_low_ns = duration_ns;
        }

        if (transitions[i].new_level == 1)
            rising_count++;
        else
            falling_count++;

        if (duration_ns < glitch_threshold_ns)
            glitch_count++;
    }

    printf("──────────────────────────────────\n");
    printf("TRANSITION COUNTS\n");
    printf("──────────────────────────────────\n");
    printf("  Rising edges:  %d\n", rising_count);
    printf("  Falling edges: %d\n", falling_count);
    printf("  Total:         %d\n\n", count);

    printf("──────────────────────────────────\n");
    printf("PHASE DURATIONS\n");
    printf("──────────────────────────────────\n");

    if (high_phases > 0) {
        printf("  HIGH phases: %d\n", high_phases);
        printf("    mean:  %.1f ns\n", sum_high_ns / high_phases);
        printf("    min:   %.1f ns\n", min_high_ns);
        printf("    max:   %.1f ns\n", max_high_ns);
    }
    printf("\n");
    if (low_phases > 0) {
        printf("  LOW phases: %d\n", low_phases);
        printf("    mean:  %.1f ns\n", sum_low_ns / low_phases);
        printf("    min:   %.1f ns\n", min_low_ns);
        printf("    max:   %.1f ns\n", max_low_ns);
    }

    printf("\n──────────────────────────────────\n");
    printf("GLITCH ANALYSIS (< %.0f ns)\n", glitch_threshold_ns);
    printf("──────────────────────────────────\n");
    printf("  Glitches detected: %d out of %d phases (%.3f%%)\n",
           glitch_count, count - 1,
           count > 1 ? 100.0 * glitch_count / (count - 1) : 0);

    if (glitch_count > 0) {
        printf("\n  First glitches (up to 20):\n");
        int shown = 0;
        for (int i = 1; i < count && shown < 20; i++) {
            double duration_ns = (transitions[i].cntvct - transitions[i-1].cntvct) * NS_PER_TICK;
            if (duration_ns < glitch_threshold_ns) {
                printf("    transition %d: %.1f ns, %s→%s\n",
                       i, duration_ns,
                       transitions[i].new_level ? "LOW" : "HIGH",
                       transitions[i].new_level ? "HIGH" : "LOW");
                shown++;
            }
        }
    }

    printf("\n──────────────────────────────────\n");
    printf("PHASE DURATION HISTOGRAM (µs)\n");
    printf("──────────────────────────────────\n");
    printf("\n  HIGH phases:\n");
    for (int i = 0; i < HIST_BUCKETS; i++) {
        if (hist_high[i] > 0)
            printf("    %3d µs: %lu\n", i, (unsigned long)hist_high[i]);
    }
    printf("\n  LOW phases:\n");
    for (int i = 0; i < HIST_BUCKETS; i++) {
        if (hist_low[i] > 0)
            printf("    %3d µs: %lu\n", i, (unsigned long)hist_low[i]);
    }

    /* Print first 40 transitions for visual inspection */
    printf("\n──────────────────────────────────\n");
    printf("FIRST 40 TRANSITIONS\n");
    printf("──────────────────────────────────\n");
    printf("  %6s  %8s  %14s  %12s\n", "idx", "level", "cntvct", "delta_ns");
    printf("  %6s  %8s  %14s  %12s\n", "──────", "────────", "──────────────", "────────────");
    for (int i = 0; i < 40 && i < count; i++) {
        double delta_ns = 0;
        if (i > 0)
            delta_ns = (transitions[i].cntvct - transitions[i-1].cntvct) * NS_PER_TICK;
        printf("  %6d  %8s  %14lu  %12.1f\n",
               i,
               transitions[i].new_level ? "→ HIGH" : "→ LOW",
               (unsigned long)transitions[i].cntvct,
               delta_ns);
    }

    printf("\nDone.\n");
    return 0;
}