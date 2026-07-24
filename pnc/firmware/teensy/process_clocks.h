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
//   • PPS-synchronous CounterLedger plus exact-row PhaseLedger clockfaces
//   • Local CLOCKS-owned ZERO/START logical zero-offset installation
//   • Campaign Flash Cut: hot campaign boundary without Alpha epoch rebase
//   • PPS/VCLOCK-selected truth capture
//   • Deferred 1 Hz publication after both post-PPS OCXO edges complete
//   • Continuous DWT-to-GNSS calibration (campaign-independent)
//   • Static PPS/GPIO-based one-second prediction audit for VCLOCK and OCXO lanes
//   • VCLOCK heartbeat and OCXO one-second compare consumption as observed
//     DWT edge timing
//   • Per-second ISR-delay verdict transport: delayed/on-time, attributed
//     blocker, endpoint delay, and signed interval contamination
//   • Servo DAC intent planning with the dither owner performing all
//     hardware-facing DAC realization
//
// Initialization is split into two phases:
//
//   Phase 1: process_clocks_init_hardware()
//     Starts DWT.  QTimer hardware custody is handled by process_interrupt.
//     Must be called before full process init so DWT timing is available.
//
//   Phase 2: process_clocks_init()
//     Configures OCXO DACs (both), PPS ISR, relay pins, and CLOCKS
//     state.  Must be called AFTER timepop_init().
//
// TIMEBASE row lifecycle:
//
//   PPS opens one active scientific row.  The first OCXO1 and OCXO2
//   one-second edges carrying that PPS sequence complete their lanes and
//   resolve the PhaseLedger suffixes.  Only then may Beta publish.  There is
//   no TIMEBASE row queue; a later PPS finding the row incomplete is a
//   structural timing failure, not permission to overwrite or infer data.
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
// SmartZero physical one-second edge separation
// -----------------------------------------------------------------------------
//
// CLOCKS installs one shared logical zero while deliberately placing the
// physical OCXO one-second grids apart in real time.  Each configured delay
// is a minimum: priority-0 preemption may make a separation larger, but the
// next lane must never be installed sooner than this interval after the
// actual preceding reference/installation.
//
//   OCXO1 install >= selected PPS/VCLOCK reference + minimum
//   OCXO2 install >= actual OCXO1 install          + minimum
//
// The physical offset is intentionally invisible to the public clockface;
// CounterLedger/PhaseLedger and public-origin normalization retain common
// logical zero across VCLOCK, OCXO1, and OCXO2.
//
static constexpr uint32_t CLOCKS_SMARTZERO_MIN_EDGE_SEPARATION_US = 5U;
static constexpr uint64_t CLOCKS_SMARTZERO_MIN_EDGE_SEPARATION_NS =
    (uint64_t)CLOCKS_SMARTZERO_MIN_EDGE_SEPARATION_US * 1000ULL;

static_assert(CLOCKS_SMARTZERO_MIN_EDGE_SEPARATION_US > 0U,
              "SmartZero physical edge separation must be non-zero");

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
// Runtime science gate mode
// -----------------------------------------------------------------------------
//
// STRICT preserves the production court: a science-rejected candidate remains
// visible testimony but cannot mutate campaign math and is expected to be
// dropped by the Pi.  FORENSIC preserves the same honest verdict while allowing
// the candidate to flow through totals/Welfords/servo math for diagnosis.
// Structural corruption and WATCHDOG_ANOMALY behavior are never bypassed.

enum class clocks_gate_mode_t : uint8_t {
  STRICT   = 0,
  FORENSIC = 1,
};

const char* clocks_gate_mode_name(clocks_gate_mode_t mode);
clocks_gate_mode_t clocks_gate_mode(void);
bool clocks_gate_mode_forensic(void);

// -----------------------------------------------------------------------------
// Campaign candidate science disposition
// -----------------------------------------------------------------------------
//
// Once public PPS1 exists, survivable lane-science failures must remain visible
// as TIMEBASE_FRAGMENT candidates instead of silently stopping the campaign.
// Alpha and process_interrupt report those failures through this tiny latch;
// Beta consumes the first pending verdict at the next candidate boundary and
// stamps the fragment SCIENCE_REJECT.  Fundamental PPS/VCLOCK identity, memory,
// payload, and publication failures continue to use WATCHDOG_ANOMALY.

enum class clocks_science_reject_source_t : uint8_t {
  NONE      = 0,
  BETA      = 1,
  ALPHA     = 2,
  INTERRUPT = 3,
};

enum class clocks_science_reject_reason_t : uint16_t {
  NONE                                = 0,
  BETA_OCXO_SCIENCE_CUSTODY           = 100,
  ALPHA_COUNTERLEDGER_INTERVAL        = 300,
  ALPHA_BRIDGE_NONMONOTONIC           = 301,
  ALPHA_OCXO_PROJECTION_WINDOW        = 302,
  ALPHA_OCXO_CLOCK_APPLY              = 303,
  ALPHA_COUNTERLEDGER_CAPTURE         = 304,
};

void clocks_science_reject(clocks_science_reject_source_t source,
                           clocks_science_reject_reason_t reason,
                           uint32_t lane,
                           uint32_t detail0 = 0U,
                           uint32_t detail1 = 0U,
                           uint32_t detail2 = 0U,
                           uint32_t detail3 = 0U);

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
//   VCLOCK — observed PPS/VCLOCK edge-to-edge DWT cycles
//   OCXO1  — OCXO1 authored edge-to-edge DWT cycles
//   OCXO2  — OCXO2 authored edge-to-edge DWT cycles
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


// -----------------------------------------------------------------------------
// Alpha always-on OCXO TAU estimator
// -----------------------------------------------------------------------------
// Alpha estimates OCXO frequency continuously from lawful PhaseLedger refined
// endpoints. This surface is reset by SmartZero/Alpha epoch replacement, not by
// campaign START/STOP, so Beta can publish a mature frequency estimate at the
// first public campaign row instead of rediscovering TAU from a launch-origin
// quotient.
#ifndef CLOCKS_ALPHA_TAU_SNAPSHOT_T_DEFINED
#define CLOCKS_ALPHA_TAU_SNAPSHOT_T_DEFINED
struct clocks_alpha_tau_snapshot_t {
  bool     valid = false;
  uint32_t clock_id = 0;
  uint32_t epoch_sequence = 0;
  uint32_t reset_count = 0;
  uint32_t sample_count = 0;
  uint32_t interval_count = 0;
  uint32_t reject_count = 0;
  uint32_t gap_reset_count = 0;
  uint32_t last_pps_sequence = 0;
  uint32_t last_interval_pps_sequence = 0;
  uint64_t first_refined_ns = 0;
  uint64_t last_refined_ns = 0;
  int64_t  last_fast_residual_ns = 0;
  double   tau = 1.0;
  double   ppb = 0.0;
  double   stderr_ppb = 0.0;
  double   interval_mean_ppb = 0.0;
  double   interval_stddev_ppb = 0.0;
  double   interval_stderr_ppb = 0.0;
  int64_t  intercept_ns = 0;
};
#endif

bool clocks_alpha_ocxo_tau_snapshot(time_clock_id_t clock,
                                    clocks_alpha_tau_snapshot_t* out);

// Compatibility alias; equivalent to clocks_alpha_ocxo_tau_snapshot().
bool clocks_alpha_tau_snapshot(time_clock_id_t clock,
                               clocks_alpha_tau_snapshot_t* out);
