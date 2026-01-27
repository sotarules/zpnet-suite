#include "cpu_usage.h"
#include "timepop.h"
#include <Arduino.h>

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
//
// This callback is invoked by TimePop at the cadence defined
// for TIMEPOP_CLASS_CPU_SAMPLE. No self-rescheduling.
//
static void cpu_usage_tick(timepop_ctx_t* timer, void* /*user*/) {
  cpu_usage_sample();
}

void cpu_usage_init_timer(void) {
    timepop_arm(
      TIMEPOP_CLASS_CPU_SAMPLE,
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
// Sample CPU usage
// ------------------------------------------------------------
void cpu_usage_sample(void) {
    uint32_t total_now = ARM_DWT_CYCCNT;
    uint32_t busy_now  = busy_cycles_accum;

    uint32_t total_delta = total_now - last_total_cycles;
    uint32_t busy_delta  = busy_now  - last_busy_cycles;

    uint32_t now_ms = millis();
    last_sample_window_ms = now_ms - last_sample_time_ms;
    last_sample_time_ms   = now_ms;

    if (total_delta > 0) {
        float usage = (float)busy_delta / (float)total_delta;

        // Clamp defensively
        if (usage < 0.0f) usage = 0.0f;
        if (usage > 1.0f) usage = 1.0f;

        cpu_usage_percent = usage * 100.0f;
    }

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
#if defined(F_CPU_ACTUAL)
    return (uint32_t)(F_CPU_ACTUAL / 1000000UL);
#else
    return (uint32_t)(F_CPU / 1000000UL);
#endif
}
