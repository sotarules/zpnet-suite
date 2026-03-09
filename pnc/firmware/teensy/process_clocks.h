#pragma once

#include <stdint.h>
#include "process.h"

// ============================================================================
// CLOCKS — Authoritative Temporal Subsystem (Teensy)
// ============================================================================
//
// CLOCKS owns ALL notions of time on the Teensy.
//
// Responsibilities:
//   • Hardware clock capture (DWT, GNSS, OCXO)
//   • Synthetic nanosecond clocks
//   • PPS-aligned truth capture
//   • 1 Hz publication of canonical clock tuple
//
// Initialization is split into two phases:
//
//   Phase 1: process_clocks_init_hardware()
//     Starts DWT, GPT2 (GNSS 10 MHz), and GPT1 (OCXO 10 MHz).
//     Must be called BEFORE timepop_init() so that GPT2 is running
//     when TimePop installs its output compare ISR.
//
//   Phase 2: process_clocks_init()
//     Configures OCXO DAC, PPS ISR, relay pins, and arms the
//     OCXO dither timer.  Must be called AFTER timepop_init().
//
// ============================================================================

// -----------------------------------------------------------------------------
// Initialization — Phase 1 (hardware only, no TimePop dependency)
// -----------------------------------------------------------------------------

/// Start DWT, GPT2 (GNSS VCLOCK), and GPT1 (OCXO).
/// Safe to call before timepop_init().
/// Idempotent — safe to call multiple times.
void process_clocks_init_hardware(void);

// -----------------------------------------------------------------------------
// Initialization — Phase 2 (full lifecycle, requires TimePop)
// -----------------------------------------------------------------------------

/// Configure PPS ISR, OCXO DAC, relay pins, arm dither timer.
/// Must be called after timepop_init().
void process_clocks_init(void);

// -----------------------------------------------------------------------------
// Process registration
// -----------------------------------------------------------------------------

void process_clocks_register(void);

// -----------------------------------------------------------------------------
// Direct accessors (escape hatches)
// -----------------------------------------------------------------------------

uint64_t clocks_dwt_cycles_now(void);
uint64_t clocks_dwt_ns_now(void);

uint64_t clocks_gnss_ticks_now(void);
uint64_t clocks_gnss_ns_now(void);

uint64_t clocks_ocxo_ticks_now(void);
uint64_t clocks_ocxo_ns_now(void);