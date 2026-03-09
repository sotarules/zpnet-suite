#include "cpu_usage.h"
#include "timepop.h"
#include <Arduino.h>

// ------------------------------------------------------------
// TimePop delay constants (nanoseconds)
// ------------------------------------------------------------

static constexpr uint64_t CPU_SAMPLE_PERIOD_NS = 1000000000ULL;  // 1 second

// ------------------------------------------------------------
// Internal state
// ------------------------------------------------------------

// Accumulated busy cycles (monotonic)
static volatile uint32_t busy_cycles_accum = 0;

// Last sampled counters
static uint32_t last_total_cycles = 0;
static uint32_t last_busy_cycles  = 0;

// Computed usage [0.0, 100.0]
static float cpu_usage_percent = 0.0f;

// Sample window bookkeeping
static uint32_t last_sample_window_ms = 0;
static uint32_t last_sample_time_ms   = 0;

// ------------------------------------------------------------
// Initialization
// ------------------------------------------------------------
void cpu_usage_init(void) {
    // Enable DWT cycle counter if not already enabled
    if (!(ARM_DWT_CTRL & ARM_DWT_CTRL_CYCCNTENA)) {
        ARM_DEMCR |= ARM_DEMCR_TRCENA;
        ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
        ARM_DWT_CYCCNT = 0;
    }

    busy_cycles_accum = 0;
    last_total_cycles = ARM_DWT_CYCCNT;
    last_busy_cycles  = 0;
    cpu_usage_percent = 0.0f;

    last_sample_time_ms   = millis();
    last_sample_window_ms = 0;
}

// ------------------------------------------------------------
// CPU usage sampler (TimePop recurring task)
// ------------------------------------------------------------

static void cpu_usage_tick(timepop_ctx_t* timer, void* /*user*/) {
  cpu_usage_sample();
}

void cpu_usage_init_timer(void) {
    timepop_arm(
      CPU_SAMPLE_PERIOD_NS,
      true,                    // recurring
      cpu_usage_tick,
      nullptr,
      "cpu-usage"
    );
}

// ------------------------------------------------------------
// Busy-cycle accounting (scheduler boundary)
// ------------------------------------------------------------
void cpu_usage_account_busy(uint32_t cycles) {
    // Called from scheduled (non-ISR) context only
    busy_cycles_accum += cycles;
}

// ------------------------------------------------------------
// Sample CPU usage (degenerate-window safe)
// ------------------------------------------------------------
void cpu_usage_sample(void) {

    uint32_t total_now = ARM_DWT_CYCCNT;
    uint32_t busy_now  = busy_cycles_accum;

    uint32_t total_delta = total_now - last_total_cycles;
    uint32_t busy_delta  = busy_now  - last_busy_cycles;

    uint32_t now_ms = millis();
    uint32_t window_ms = now_ms - last_sample_time_ms;

    // --------------------------------------------------------
    // Reject degenerate samples
    //
    // A valid utilization sample requires a meaningful
    // wall-clock interval. Very small deltas are scheduling
    // artifacts and carry no semantic information.
    // --------------------------------------------------------

    // Minimum: ~1 ms worth of CPU cycles
    const uint32_t MIN_CYCLE_DELTA =
        cpu_usage_get_cpu_freq_mhz() * 1000UL;

    if (total_delta < MIN_CYCLE_DELTA) {
        // Do NOT update any derived state.
        // Preserve last meaningful sample.
        return;
    }

    // --------------------------------------------------------
    // Compute utilization over a valid window
    // --------------------------------------------------------

    float usage = (float)busy_delta / (float)total_delta;

    // Defensive clamp (should never trigger)
    if (usage < 0.0f) usage = 0.0f;
    if (usage > 1.0f) usage = 1.0f;

    cpu_usage_percent = usage * 100.0f;

    // --------------------------------------------------------
    // Commit window bookkeeping ONLY for valid samples
    // --------------------------------------------------------

    last_sample_window_ms = window_ms;
    last_sample_time_ms   = now_ms;

    last_total_cycles = total_now;
    last_busy_cycles  = busy_now;
}


// ------------------------------------------------------------
// Accessors
// ------------------------------------------------------------
float cpu_usage_get_percent(void) {
    return cpu_usage_percent;
}

uint32_t cpu_usage_get_busy_cycles(void) {
    return busy_cycles_accum;
}

uint32_t cpu_usage_get_total_cycles(void) {
    return ARM_DWT_CYCCNT;
}

uint32_t cpu_usage_get_sample_window_ms(void) {
    return last_sample_window_ms;
}

uint32_t cpu_usage_get_cpu_freq_mhz(void) {
    // F_CPU_ACTUAL is a runtime global updated by set_arm_clock().
    // This returns the ACTUAL core clock, not the compile-time default.
    return (uint32_t)(F_CPU_ACTUAL / 1000000UL);
}