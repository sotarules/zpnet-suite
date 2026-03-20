#pragma once

#include <stdint.h>
#include "process.h"

// ============================================================================
// CLOCKS — Authoritative Temporal Subsystem (Teensy) — v12
// ============================================================================
//
// CLOCKS owns ALL notions of time on the Teensy.
//
// Responsibilities:
//   • Hardware clock capture (DWT, GNSS, OCXO1, OCXO2)
//   • Synthetic nanosecond clocks
//   • PPS-aligned truth capture
//   • 1 Hz publication of canonical clock tuple
//   • Continuous DWT-to-GNSS calibration (campaign-independent)
//
// Initialization is split into two phases:
//
//   Phase 1: process_clocks_init_hardware()
//     Starts DWT, QTimer1 (GNSS 10 MHz), GPT1 (OCXO1 10 MHz),
//     and GPT2 (OCXO2 10 MHz).
//     Must be called BEFORE timepop_init() so that QTimer1 is running
//     when TimePop installs its compare ISR.
//
//   Phase 2: process_clocks_init()
//     Configures OCXO DACs (both), PPS ISR, relay pins, and arms
//     the OCXO dither timer.  Must be called AFTER timepop_init().
//
// DWT-to-GNSS Calibration:
//
//   A continuous tracker maintains dwt_cycles_per_gnss_second,
//   updated on every PPS edge regardless of campaign state.
//   This value is never zeroed and survives across campaigns.
//   It provides:
//     • Immediate availability at campaign start (no cold-start penalty)
//     • The foundational calibration constant for TimePop scheduling
//     • The DWT-to-GNSS ratio for reciprocal frequency counting
//     • Sub-nanosecond interpolation between PPS edges
//
// ============================================================================

// -----------------------------------------------------------------------------
// Initialization — Phase 1 (hardware only, no TimePop dependency)
// -----------------------------------------------------------------------------

/// Start DWT, QTimer1 (GNSS VCLOCK), GPT1 (OCXO1), GPT2 (OCXO2).
/// Safe to call before timepop_init().
/// Idempotent — safe to call multiple times.
void process_clocks_init_hardware(void);

// -----------------------------------------------------------------------------
// Initialization — Phase 2 (full lifecycle, requires TimePop)
// -----------------------------------------------------------------------------

/// Configure PPS ISR, OCXO DACs (both), relay pins, arm dither timer.
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

uint64_t clocks_ocxo1_ticks_now(void);
uint64_t clocks_ocxo1_ns_now(void);

uint64_t clocks_ocxo2_ticks_now(void);
uint64_t clocks_ocxo2_ns_now(void);

// -----------------------------------------------------------------------------
// DWT-to-GNSS calibration (continuous, campaign-independent)
// -----------------------------------------------------------------------------

/// Returns the most recent measured DWT cycles per GNSS second.
/// This is the raw last-second delta — no averaging, no smoothing.
/// Updated on every PPS edge.  Returns 0 if no PPS has been observed.
uint32_t clocks_dwt_cycles_per_gnss_second(void);

/// Returns true if at least one PPS-to-PPS DWT delta has been measured.
bool clocks_dwt_calibration_valid(void);