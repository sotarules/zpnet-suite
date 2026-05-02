#pragma once

#include <stdint.h>
#include "process.h"

// ============================================================================
// CLOCKS — Authoritative Temporal Subsystem (Teensy) — v12
// ============================================================================
//
// CLOCKS owns the canonical time ledgers on the Teensy.
//
// Responsibilities:
//   • Consumption of process_interrupt-authored clock captures
//   • Synthetic nanosecond clocks
//   • Compatibility delegation of ZERO/START epoch requests to process_epoch
//   • PPS/VCLOCK-selected truth capture
//   • 1 Hz publication of canonical clock tuple
//   • Continuous DWT-to-GNSS calibration (campaign-independent)
//
// Initialization is split into two phases:
//
//   Phase 1: process_clocks_init_hardware()
//     Starts DWT.  QTimer hardware custody is handled by process_interrupt.
//     Must be called before full process init so DWT timing is available.
//
//   Phase 2: process_clocks_init()
//     Configures OCXO DACs (both), PPS ISR, relay pins, and arms
//     the OCXO dither timer.  Must be called AFTER timepop_init().
//
// DWT-to-GNSS Calibration:
//
//   A continuous tracker maintains dwt_cycles_per_gnss_second,
//   updated on every selected PPS/VCLOCK edge regardless of campaign state.
//   This value is never zeroed and survives across campaigns.
//   It provides:
//     • Immediate availability at campaign start (no cold-start penalty)
//     • The foundational calibration constant for TimePop scheduling
//     • The DWT-to-GNSS ratio for reciprocal frequency counting
//     • Sub-nanosecond interpolation between selected PPS/VCLOCK edges
//
// ============================================================================

// -----------------------------------------------------------------------------
// Initialization — Phase 1 (hardware only, no TimePop dependency)
// -----------------------------------------------------------------------------

/// Start DWT. QTimer/GNSS/OCXO hardware custody belongs to process_interrupt.
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
/// Updated on every selected PPS/VCLOCK edge.  Returns 0 if no PPS/VCLOCK edge has been observed.
uint32_t clocks_dwt_cycles_per_gnss_second(void);

/// Returns true if at least one PPS-to-PPS DWT delta has been measured.
bool clocks_dwt_calibration_valid(void);