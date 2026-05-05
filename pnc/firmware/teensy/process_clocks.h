#pragma once

#include <stdint.h>
#include "process.h"
#include "time.h"

// ============================================================================
// CLOCKS — Authoritative Temporal Subsystem (Teensy) — v12
// ============================================================================
//
// CLOCKS owns the canonical time ledgers on the Teensy.
//
// Responsibilities:
//   • Consumption of process_interrupt-authored clock captures
//   • Synthetic nanosecond clocks
//   • Local CLOCKS-owned ZERO/START logical zero-offset installation
//   • PPS/VCLOCK-selected truth capture
//   • 1 Hz publication of canonical clock tuple
//   • Continuous DWT-to-GNSS calibration (campaign-independent)
//   • Gamma next-second prediction for VCLOCK and OCXO lanes
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

uint64_t clocks_ocxo1_ticks_now(void);
uint64_t clocks_ocxo1_ns_now(void);

uint64_t clocks_ocxo2_ticks_now(void);
uint64_t clocks_ocxo2_ns_now(void);


// -----------------------------------------------------------------------------
// CLOCKS Gamma — next-second DWT prediction engine
// -----------------------------------------------------------------------------

static constexpr uint32_t CLOCKS_GAMMA_TICKS_PER_SECOND = 10000000U;
static constexpr uint32_t CLOCKS_GAMMA_CADENCE_HZ = 100U;
static constexpr uint32_t CLOCKS_GAMMA_TICKS_PER_SAMPLE =
    CLOCKS_GAMMA_TICKS_PER_SECOND / CLOCKS_GAMMA_CADENCE_HZ;

struct clocks_gamma_prediction_snapshot_t {
  uint32_t clock_id;

  // Last completed clock-second facts.  These are the values that belong in
  // TIMEBASE_FRAGMENT because they describe the second that just closed.
  uint32_t completed_edge_count;
  uint32_t completed_static_prediction_cycles;
  uint32_t completed_dynamic_prediction_cycles;
  uint32_t completed_actual_dwt_cycles_between_edges;
  uint32_t completed_match_count;
  uint32_t completed_adjust_count;
  uint32_t completed_ignored_count;
  int32_t  completed_ignored_min_error_cycles;
  int32_t  completed_ignored_max_error_cycles;
  int32_t  completed_last_error_cycles;
  uint32_t completed_last_sample_index;

  // Current in-flight clock-second facts.  These are report-only forensic
  // surfaces for the active second.
  uint32_t edge_count;
  uint32_t second_start_dwt;
  uint32_t current_static_prediction_cycles;
  uint32_t current_dynamic_prediction_cycles;
  uint32_t current_sample_count;
  uint32_t current_match_count;
  uint32_t current_adjust_count;
  uint32_t current_ignored_count;
  int32_t  current_ignored_min_error_cycles;
  int32_t  current_ignored_max_error_cycles;
  int32_t  current_last_error_cycles;
  uint32_t current_last_expected_dwt;
  uint32_t current_last_sample_dwt;
  uint32_t current_last_sample_index;
};

void clocks_gamma_reset_all(void);

// Opens or completes a lane-local 10,000,000-tick clock-second.  For OCXO lanes
// this must be called from OCXO-local logical grid milestones, not from a
// PPS/VCLOCK phase relationship.
void clocks_gamma_second_edge(time_clock_id_t clock, uint32_t dwt_at_edge);

// Legacy/sequential courtroom surface.  Kept so existing callers compile, but
// new OCXO call sites should use clocks_gamma_100hz_sample_at_index() so the
// sample is judged at its lane-local logical 100 Hz position.
void clocks_gamma_100hz_sample(time_clock_id_t clock, uint32_t dwt_at_sample);

// Indexed courtroom sample.  sample_index is 1..99 inside the active
// lane-local clock-second.  A sample_index of 10 means logical tick 1,000,000
// within that lane's current 10,000,000-tick second, regardless of VCLOCK/PPS.
void clocks_gamma_100hz_sample_at_index(time_clock_id_t clock,
                                        uint32_t sample_index,
                                        uint32_t dwt_at_sample);

bool clocks_gamma_snapshot(time_clock_id_t clock,
                           clocks_gamma_prediction_snapshot_t* out);

struct clocks_gamma_prediction_detail_sample_t {
  bool     populated;
  bool     endpoint;
  uint32_t sample_index;
  uint32_t sample_percent;

  uint32_t static_prediction_cycles;
  uint32_t dynamic_prediction_cycles;
  uint32_t dynamic_prediction_after_sample_cycles;

  uint32_t static_prediction_thus_far_cycles;
  uint32_t dynamic_prediction_thus_far_cycles;
  int32_t  dynamic_minus_static_thus_far_cycles;
  uint32_t actual_cycles_thus_far;

  int32_t  residual_cycles;
  uint32_t abs_residual_cycles;
  bool     accepted;
  bool     ignored;
  int32_t  correction_cycles;
  uint32_t gate_threshold_cycles;
};

struct clocks_gamma_prediction_detail_snapshot_t {
  bool     valid;
  uint32_t clock_id;
  uint32_t completed_edge_count;
  uint32_t anchor_dwt;
  uint32_t static_prediction_cycles;
  uint32_t dynamic_final_prediction_cycles;
  uint32_t actual_cycles;
  int32_t  static_residual_cycles;
  int32_t  dynamic_residual_cycles;
  uint32_t sample_count;
  uint32_t sample_capacity;
  uint32_t sample_step_percent;
  uint32_t gate_threshold_cycles;
  clocks_gamma_prediction_detail_sample_t samples[10];
};

bool clocks_gamma_prediction_detail_snapshot(
    time_clock_id_t clock,
    clocks_gamma_prediction_detail_snapshot_t* out);

// -----------------------------------------------------------------------------
// DWT-to-GNSS calibration (continuous, campaign-independent)
// -----------------------------------------------------------------------------

/// Returns the most recent measured DWT cycles per GNSS second.
/// This is the raw last-second delta — no averaging, no smoothing.
/// Updated on every selected PPS/VCLOCK edge.  Returns 0 if no PPS/VCLOCK edge has been observed.
uint32_t clocks_dwt_cycles_per_gnss_second(void);

/// Returns true if at least one PPS-to-PPS DWT delta has been measured.
bool clocks_dwt_calibration_valid(void);
