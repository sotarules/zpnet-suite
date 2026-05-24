#pragma once

#include <stdint.h>
#include "process.h"
#include "time.h"

// ============================================================================
// CLOCKS — Authoritative Temporal Subsystem (Teensy) — v12
// ============================================================================
//
// CLOCKS owns the Teensy timing ledgers and bridge-measured residual surfaces.
//
// Responsibilities:
//   • Consumption of process_interrupt-authored clock captures
//   • Nominal clock ledgers and bridge-derived measured residuals
//   • Local CLOCKS-owned ZERO/START logical zero-offset installation
//   • PPS/VCLOCK-selected truth capture
//   • 1 Hz publication of canonical clock tuple
//   • Continuous DWT-to-GNSS calibration (campaign-independent)
//   • Static PPS/GPIO-based one-second prediction audit for VCLOCK and OCXO lanes
//   • Phase-aware OCXO quiet-zone sample consumption and boundary projection
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
//     • A smooth PPS/GPIO slope source that avoids QTimer 4-cycle quantization
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

/// Configure OCXO DACs (both), subscriptions, and CLOCKS state.
/// Must be called after timepop_init().
void process_clocks_init(void);

// -----------------------------------------------------------------------------
// Process registration
// -----------------------------------------------------------------------------

void process_clocks_register(void);

// -----------------------------------------------------------------------------
// Direct accessors (escape hatches)
// -----------------------------------------------------------------------------

// DWT64 logical clock. CLOCKS/alpha owns the physical/extended DWT64 ledger.
// CLOCKS.ZERO installs the logical zero by mapping a selected DWT32 event
// coordinate to the DWT64 origin. After that, clocks_dwt_cycles_now() returns
// zero-relative logical DWT64 cycles.
bool clocks_dwt64_epoch_reset_at_dwt32(uint32_t epoch_dwt32,
                                       uint64_t* out_raw_epoch_dwt64);

uint64_t clocks_dwt_cycles_now(void);
uint64_t clocks_dwt_cycles_at_dwt(uint32_t dwt32);

uint64_t clocks_gnss_ticks_now(void);
uint64_t clocks_gnss_ns_now(void);

uint64_t clocks_ocxo1_measured_gnss_ticks_now(void);
uint64_t clocks_ocxo1_measured_gnss_ns_now(void);

uint64_t clocks_ocxo2_measured_gnss_ticks_now(void);
uint64_t clocks_ocxo2_measured_gnss_ns_now(void);


// -----------------------------------------------------------------------------
// CLOCKS static one-second prediction audit
// -----------------------------------------------------------------------------
//
// Dynamic 100 Hz prediction has been retired.  The operating doctrine is:
//
//   • Use the prior completed one-second DWT interval as the static prediction.
//   • Do not rebase during the active second from quantized QTimer samples.
//   • Let Welford/statistical surfaces absorb the remaining measurement noise.
//
// Four symmetric static prediction surfaces are published:
//   PPS    — physical GPIO PPS edge-to-edge DWT cycles
//   VCLOCK — canonical PPS/VCLOCK lattice edge-to-edge DWT cycles
//   OCXO1  — OCXO1 phase-projected boundary-to-boundary DWT cycles
//   OCXO2  — OCXO2 phase-projected boundary-to-boundary DWT cycles
// Each lane uses the prior completed interval as the next-second prediction.

struct clocks_static_prediction_snapshot_t {
  uint32_t clock_id = 0;
  bool     valid = false;

  // Number of completed one-second intervals recorded for this lane.  A static
  // residual is valid once at least two intervals have been recorded.
  uint32_t completed_interval_count = 0;

  // Prior interval, current interval, and current-minus-prior residual.
  uint32_t static_prediction_cycles = 0;
  uint32_t actual_cycles = 0;
  int32_t  static_residual_cycles = 0;
};

void clocks_static_prediction_reset_all(void);
bool clocks_static_prediction_pps_snapshot(clocks_static_prediction_snapshot_t* out);
bool clocks_static_prediction_snapshot(time_clock_id_t clock,
                                       clocks_static_prediction_snapshot_t* out);

// -----------------------------------------------------------------------------
// DWT-to-GNSS calibration (continuous, campaign-independent)
// -----------------------------------------------------------------------------

/// Returns the most recent measured DWT cycles per GNSS second.
/// This is a raw static one-second delta — no averaging, no smoothing.
/// Operationally this is now PPS/GPIO-witness based when available, avoiding
/// QTimer compare/ISR lattice quantization. Returns 0 if no PPS edge has been observed.
uint32_t clocks_dwt_cycles_per_gnss_second(void);

/// Returns true if at least one PPS-to-PPS DWT delta has been measured.
bool clocks_dwt_calibration_valid(void);
