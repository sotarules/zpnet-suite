#pragma once

#include <stdint.h>
#include <stddef.h>
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
//   • Local CLOCKS-owned autonomous startup and explicit ZERO epoch install
//   • Campaign Flash Cut: hot campaign boundary without Alpha epoch rebase
//   • PPS/VCLOCK-selected truth capture
//   • Deferred 1 Hz completed-row handoff after both post-PPS OCXO edges complete
//   • Serialized command reporting: Priority 0 capture remains live while the
//     Priority 16 TimePop/handoff tier is excluded from re-entering command
//     report construction; SYSTEM independently owns MONITOR_FRAGMENT formatting
//   • Continuous DWT-to-GNSS calibration (campaign-independent)
//   • Static PPS/GPIO-based one-second prediction audit for VCLOCK and OCXO lanes
//   • VCLOCK heartbeat and OCXO one-second compare consumption as observed
//     DWT edge timing
//   • Lossless per-second interrupt testimony: raw arrival/preemption context,
//     software compare intent, physical COMP1/CMPLD1 readback, delayed/on-time
//     verdict, endpoint delay, and signed interval contamination
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
//     Configures OCXO DAC control registers, observes their surviving input
//     codes without updating VOUT, then initializes subscriptions and CLOCKS
//     state.  Must be called AFTER timepop_init().
//
// Completed campaign-row lifecycle:
//
//   PPS opens one active scientific row.  The first OCXO1 and OCXO2
//   one-second edges carrying that PPS sequence complete their lanes and
//   resolve the PhaseLedger suffixes.  Only then may Beta expose the immutable
//   campaign result to SYSTEM.  There is no completed-row queue; a later PPS
//   finding the row incomplete is a structural timing failure, not permission
//   to overwrite or infer data.
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
// OCXO physical one-second grid rephase spacing
// -----------------------------------------------------------------------------
//
// Autonomous startup, explicit ZERO, and RECOVER use the staged physical-grid
// transaction.  Campaign START is recording-only and never rebases Alpha.
// The transaction keeps the logical clock doctrine owned by its caller while
// deliberately placing the recurring OCXO compare grids far apart in real
// time:
//
//   OCXO1 install >= selected transaction reference + 50 ms
//   OCXO2 install >= actual OCXO1 completion        + 50 ms
//
// TimePop one-shots enforce both waits; no foreground spin is permitted.
// Priority-0 preemption may enlarge either gap, but must never compress it.
// The physical offset remains invisible to the public clockface.
//
static constexpr uint32_t CLOCKS_OCXO_GRID_REPHASE_DELAY_MS = 50U;
static constexpr uint64_t CLOCKS_OCXO_GRID_REPHASE_DELAY_NS =
    (uint64_t)CLOCKS_OCXO_GRID_REPHASE_DELAY_MS * 1000000ULL;

static_assert(CLOCKS_OCXO_GRID_REPHASE_DELAY_MS > 0U,
              "OCXO physical grid rephase delay must be non-zero");

// Report/back-compat aliases.  New implementation code should use the
// lifecycle-neutral REPHASE names above.
static constexpr uint32_t CLOCKS_SMARTZERO_MIN_EDGE_SEPARATION_US =
    CLOCKS_OCXO_GRID_REPHASE_DELAY_MS * 1000U;
static constexpr uint64_t CLOCKS_SMARTZERO_MIN_EDGE_SEPARATION_NS =
    CLOCKS_OCXO_GRID_REPHASE_DELAY_NS;

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

/// Configure and observe OCXO DACs without authoring VOUT, then initialize
/// subscriptions and CLOCKS state.
/// Must be called after timepop_init().
void process_clocks_init(void);

// -----------------------------------------------------------------------------
// Process registration
// -----------------------------------------------------------------------------

void process_clocks_register(void);

// -----------------------------------------------------------------------------
// Per-PPS science exclusion contract
// -----------------------------------------------------------------------------
//
// There is no runtime science/forensic mode. Every completed PPS second is
// preserved for MONITOR/TIMEBASE. Any layer may object while it still owns the
// evidence needed to adjudicate the second. A pending objection excludes the
// whole PPS second from Welford, TAU/PPB, servo, and DAC-control math, but does
// not suppress publication or PostgreSQL persistence.
//
// WATCHDOG_ANOMALY is reserved for continuity surrender: the instrument can no
// longer guarantee timeline or counter identity and requires campaign recovery.

enum class clocks_row_objection_source_t : uint8_t {
  NONE      = 0,
  BETA      = 1,
  ALPHA     = 2,
  INTERRUPT = 3,
};

enum class clocks_row_objection_reason_t : uint16_t {
  NONE                                = 0,
  BETA_OCXO_SCIENCE_CUSTODY           = 100,
  BETA_RECOVERY_SCIENCE_HOLD           = 101,
  ALPHA_COUNTERLEDGER_INTERVAL        = 300,
  ALPHA_BRIDGE_NONMONOTONIC           = 301,
  ALPHA_OCXO_PROJECTION_WINDOW        = 302,
  ALPHA_OCXO_CLOCK_APPLY              = 303,
  ALPHA_COUNTERLEDGER_CAPTURE         = 304,
  ALPHA_CYCLE_EXCURSION               = 305,
  ALPHA_CYCLE_INTERVAL_IMPLAUSIBLE     = 306,
};

static constexpr uint32_t CLOCKS_ROW_LANE_PPS    = 1U << 0;
static constexpr uint32_t CLOCKS_ROW_LANE_VCLOCK = 1U << 1;
static constexpr uint32_t CLOCKS_ROW_LANE_OCXO1  = 1U << 2;
static constexpr uint32_t CLOCKS_ROW_LANE_OCXO2  = 1U << 3;

void clocks_row_exclude(clocks_row_objection_source_t source,
                        clocks_row_objection_reason_t reason,
                        uint32_t lane,
                        uint32_t detail0 = 0U,
                        uint32_t detail1 = 0U,
                        uint32_t detail2 = 0U,
                        uint32_t detail3 = 0U);

// True only when no layer has objected to the candidate currently being built.
// Alpha calls this before irreversible statistical mutation; Beta consumes and
// serializes the same objection at the candidate boundary.
bool clocks_row_science_eligible(uint32_t pps_sequence = 0U);

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
// Alpha estimates OCXO frequency continuously from lawful completed-row Delta
// intervals.  It survives START, STOP, FLASH_CUT, warm RECOVER, and logical
// epoch replacement; only reboot or CLOCKS.STATS_RESET clears the estimator.
// Beta can therefore publish a mature frequency estimate on the first campaign
// row instead of rediscovering TAU from a launch-origin quotient.
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


// -----------------------------------------------------------------------------
// Typed CLOCKS -> SYSTEM MONITOR handoff
// -----------------------------------------------------------------------------
//
// CLOCKS owns measurement, campaign lifecycle, and scientific verdicts. SYSTEM
// owns the complete MONITOR_FRAGMENT wire schema. The structures below contain
// immutable domain facts only: no Payload objects, field names, or nested
// serialized fragments cross the subsystem boundary.
//
// clocks_monitor_snapshot_take() always returns the latest coherent live
// instrument view. If Beta has completed a public campaign record for the
// requested sequence, that record is copied into campaign and atomically
// consumed. A later SYSTEM publication therefore cannot accidentally combine
// campaign facts from different completed seconds.

static constexpr size_t CLOCKS_MONITOR_CAMPAIGN_NAME_MAX = 64U;
static constexpr size_t CLOCKS_MONITOR_STATE_NAME_MAX = 40U;
static constexpr size_t CLOCKS_MONITOR_REASON_MAX = 160U;
static constexpr size_t CLOCKS_MONITOR_DELAY_NAME_MAX = 40U;

struct clocks_monitor_welford_snapshot_t {
  uint64_t n = 0;
  double mean = 0.0;
  double stddev = 0.0;
  double stderr_value = 0.0;
  double min = 0.0;
  double max = 0.0;
};

struct clocks_monitor_stats_clock_snapshot_t {
  clocks_monitor_welford_snapshot_t welford{};
  bool frequency_present = false;
  double tau = 1.0;
  double ppb = 0.0;
};

struct clocks_monitor_stats_snapshot_t {
  bool valid = false;
  uint32_t reset_count = 0;
  uint32_t update_count = 0;
  uint32_t last_pps_sequence = 0;
  bool completed_row_coherent = false;

  clocks_monitor_stats_clock_snapshot_t gnss{};
  clocks_monitor_stats_clock_snapshot_t dwt{};
  clocks_monitor_stats_clock_snapshot_t vclock{};
  clocks_monitor_stats_clock_snapshot_t ocxo1{};
  clocks_monitor_stats_clock_snapshot_t ocxo2{};
  clocks_monitor_stats_clock_snapshot_t pps_witness{};

  uint64_t maturity_gnss_samples = 0;
  uint64_t maturity_dwt_samples = 0;
  uint64_t maturity_vclock_samples = 0;
  uint64_t maturity_vclock_intervals = 0;
  uint64_t maturity_ocxo1_samples = 0;
  uint64_t maturity_ocxo1_intervals = 0;
  double maturity_ocxo1_stderr_ppb = 0.0;
  uint64_t maturity_ocxo2_samples = 0;
  uint64_t maturity_ocxo2_intervals = 0;
  double maturity_ocxo2_stderr_ppb = 0.0;

  uint32_t interval_min_cycles = 0;
  uint32_t interval_max_cycles = 0;
  uint32_t vclock_reject_count = 0;
  uint32_t ocxo1_reject_count = 0;
  uint32_t ocxo2_reject_count = 0;

  clocks_monitor_welford_snapshot_t ocxo1_dac{};
  clocks_monitor_welford_snapshot_t ocxo2_dac{};
};

struct clocks_monitor_raw_cycles_lane_t {
  bool valid = false;
  uint32_t completed_interval_count = 0;
  uint32_t observed_cycles = 0;
  uint32_t previous_observed_cycles = 0;
  int32_t residual_cycles = 0;

  char delay_status[CLOCKS_MONITOR_DELAY_NAME_MAX] = {0};
  bool delay_detail_present = false;
  char delay_by[CLOCKS_MONITOR_DELAY_NAME_MAX] = {0};
  bool residual_delay_valid = false;
  int32_t residual_delay_cycles = 0;
  char residual_delay_by[CLOCKS_MONITOR_DELAY_NAME_MAX] = {0};
  bool delay_explains_residual = false;
};

struct clocks_monitor_raw_cycles_snapshot_t {
  clocks_monitor_raw_cycles_lane_t pps{};
  clocks_monitor_raw_cycles_lane_t vclock{};
  clocks_monitor_raw_cycles_lane_t ocxo1{};
  clocks_monitor_raw_cycles_lane_t ocxo2{};
};

struct clocks_monitor_dither_lane_t {
  double value = 0.0;
  uint16_t hw_code = 0;
  bool readback_valid = false;
  uint16_t readback_code = 0;
  bool active = false;
  uint16_t low_code = 0;
  uint16_t high_code = 0;
  uint16_t high_ms = 0;
  bool phase_high = false;
};

struct clocks_monitor_dac_snapshot_t {
  char servo_mode[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  bool servo_active = false;
  char realization_mode[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  bool dither_operator_enabled = false;
  bool servo_request_pending = false;
  bool actuator_service_pending = false;
  clocks_monitor_dither_lane_t ocxo1{};
  clocks_monitor_dither_lane_t ocxo2{};
};

struct clocks_monitor_science_snapshot_t {
  bool valid = false;
  bool science_worthy = false;
  bool antecedents_complete = false;
  uint64_t gnss_interval_ns = 0;
  uint64_t clock_interval_ns = 0;
  int64_t fast_residual_ns = 0;
  double fast_residual_ns_exact = 0.0;
  bool delta_raw_valid = false;
  uint32_t delta_raw_reference_interval_cycles = 0;
  uint32_t delta_raw_clock_interval_cycles = 0;
  int64_t delta_raw_fast_residual_cycles = 0;
};

struct clocks_monitor_rejection_snapshot_t {
  bool present = false;
  uint32_t reason_code = 0;
  char reason_name[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  char reason[CLOCKS_MONITOR_REASON_MAX] = {0};
  char source[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  uint32_t lane = 0;
  uint32_t detail0 = 0;
  uint32_t detail1 = 0;
  uint32_t detail2 = 0;
  uint32_t detail3 = 0;
  uint32_t reject_mask = 0;
  uint32_t objection_count = 0;
};

struct clocks_monitor_recovery_snapshot_t {
  bool present = false;
  uint32_t generation = 0;
  bool transition_active = false;
  bool timeline_ready = false;
  bool clockface_ready = false;
  bool science_ready = false;
  bool ocxo1_clockface_ready = false;
  bool ocxo2_clockface_ready = false;
  bool ocxo1_science_ready = false;
  bool ocxo2_science_ready = false;
  bool science_quarantine_active = false;
  uint32_t science_quarantine_remaining = 0;
  uint32_t no_progress_rows = 0;
  uint32_t last_progress_public_count = 0;
  bool reattach_active = false;
  bool degraded_active = false;
  bool degraded_science_hold = false;
  bool reattach_stalled = false;
  char reattach_reason[CLOCKS_MONITOR_REASON_MAX] = {0};
};

struct clocks_monitor_campaign_snapshot_t {
  bool present = false;
  uint32_t completed_second_sequence = 0;
  char campaign[CLOCKS_MONITOR_CAMPAIGN_NAME_MAX] = {0};
  char campaign_state[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  uint32_t public_count = 0;
  uint64_t gnss_ns = 0;
  char disposition[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  char servo_mode[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  bool timeline_valid = false;
  bool ocxo_clockface_valid = false;
  bool ocxo_science_valid = false;
  bool science_eligible = true;
  bool control_eligible = true;
  bool persist = true;
  bool science_excluded = false;

  clocks_monitor_rejection_snapshot_t rejection{};
  clocks_monitor_recovery_snapshot_t recovery{};

  uint64_t dwt_cycle_count_total = 0;
  uint32_t dwt_cycles_between_pps_vclock = 0;
  uint32_t dwt_at_pps_vclock = 0;
  uint32_t counter32_at_pps_vclock = 0;
  clocks_monitor_raw_cycles_snapshot_t raw_cycles{};

  uint64_t ocxo1_ns = 0;
  bool ocxo1_clockface_valid = false;
  clocks_monitor_science_snapshot_t ocxo1_science{};
  uint64_t ocxo2_ns = 0;
  bool ocxo2_clockface_valid = false;
  clocks_monitor_science_snapshot_t ocxo2_science{};

  clocks_monitor_stats_snapshot_t stats{};
  bool dac_present = false;
  clocks_monitor_dac_snapshot_t dac{};
};

struct clocks_monitor_live_snapshot_t {
  bool snapshot_ok = false;
  bool valid = false;
  bool completed_row_coherent = false;
  uint32_t completed_pps_sequence = 0;
  uint32_t instrument_age_seconds = 0;

  char campaign_state[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  bool recording = false;
  bool campaign_present = false;
  char campaign[CLOCKS_MONITOR_CAMPAIGN_NAME_MAX] = {0};
  char last_campaign[CLOCKS_MONITOR_CAMPAIGN_NAME_MAX] = {0};
  uint64_t campaign_seconds = 0;
  bool campaign_presentation_ready = false;
  char presentation_mode[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  char presentation_basis[CLOCKS_MONITOR_STATE_NAME_MAX] = {0};
  bool presentation_clockfaces_zeroed = false;

  uint32_t presentation_count = 0;
  uint64_t presentation_gnss_ns = 0;
  uint64_t presentation_dwt_cycles = 0;
  uint64_t presentation_ocxo1_ns = 0;
  uint64_t presentation_ocxo2_ns = 0;
  bool timeline_valid = false;
  bool ocxo_clockface_valid = false;

  uint64_t instrument_gnss_ns = 0;
  uint64_t instrument_dwt_cycles = 0;
  uint64_t instrument_ocxo1_ns = 0;
  uint64_t instrument_ocxo2_ns = 0;
  uint32_t instrument_pps_sequence = 0;

  uint32_t dwt_cycles_per_second = 0;
  uint32_t dwt_at_pps_vclock = 0;
  uint32_t counter32_at_pps_vclock = 0;
  uint32_t selected_reference_interval_cycles = 0;
  uint32_t vclock_interval_cycles = 0;
  uint32_t ocxo1_interval_cycles = 0;
  uint32_t ocxo2_interval_cycles = 0;

  clocks_monitor_raw_cycles_snapshot_t raw_cycles{};
  clocks_monitor_stats_snapshot_t stats{};
  clocks_monitor_dac_snapshot_t dac{};
};

struct clocks_monitor_snapshot_t {
  clocks_monitor_live_snapshot_t live{};
  clocks_monitor_campaign_snapshot_t campaign{};
};

bool clocks_monitor_snapshot_take(uint32_t completed_second_sequence,
                                  clocks_monitor_snapshot_t* out);
