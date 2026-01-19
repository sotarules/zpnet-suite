#pragma once

#include <stdint.h>

/*
 * CPU Usage Instrument (Busy-Cycle Accounting — GOLD STANDARD)
 *
 * Semantics:
 *   • Bare-metal correct
 *   • ISR-safe (accounting occurs in scheduled context only)
 *   • DWT-based (cycle-accurate)
 *   • Measures fraction of time spent executing scheduled work
 *
 * Definition:
 *   • "Busy cycles" = cycles spent executing TimePop callbacks
 *   • "Total cycles" = wall-clock cycles elapsed between samples
 *   • Idle cycles are inferred as (total − busy)
 *
 * Usage:
 *   1) Call cpu_usage_init() once at boot
 *   2) Call cpu_usage_account_busy(cycles) from scheduler boundary
 *   3) Call cpu_usage_sample() at a fixed interval (e.g. 1 Hz)
 *   4) Read cpu_usage_get_percent() and diagnostics
 */

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

// Initialize CPU usage instrumentation
void cpu_usage_init(void);

// ------------------------------------------------------------
// Busy-cycle accounting (scheduler boundary)
// ------------------------------------------------------------

// Account CPU cycles spent executing scheduled work
// Must be called from non-ISR, scheduled context
void cpu_usage_account_busy(uint32_t cycles);

// ------------------------------------------------------------
// Sampling
// ------------------------------------------------------------

// Sample CPU usage over the most recent window
void cpu_usage_sample(void);

// ------------------------------------------------------------
// Accessors
// ------------------------------------------------------------

// CPU usage percentage [0.0, 100.0]
float    cpu_usage_get_percent(void);

// Accumulated busy cycles (monotonic)
uint32_t cpu_usage_get_busy_cycles(void);

// Total CPU cycles since boot (DWT)
uint32_t cpu_usage_get_total_cycles(void);

// Sample window duration (milliseconds)
uint32_t cpu_usage_get_sample_window_ms(void);

// CPU frequency (MHz)
uint32_t cpu_usage_get_cpu_freq_mhz(void);
