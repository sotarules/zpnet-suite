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
//     report construction; SYSTEM independently owns CLOCKS_FRAGMENT formatting
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
// preserved for CLOCKS/TIMEBASE. Any layer may object while it still owns the
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
  BETA_ANTECEDENT_SCIENCE_HOLD         = 102,
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
  bool     snapshot_ok = false;
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

// Return value reports coherent snapshot acquisition only.  snapshot.valid
// separately reports whether enough intervals exist for a scientific residual.
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
  bool     snapshot_ok = false;
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

  // Stable sufficient state for persistence/recovery.  Writer seqlock state is
  // intentionally not exposed.
  uint64_t cumulative_reference_ns = 0;
  uint64_t cumulative_clock_ns = 0;
  double   cumulative_clock_ns_exact = 0.0;
  double   mean_x = 0.0;
  double   mean_y = 0.0;
  double   sxx = 0.0;
  double   sxy = 0.0;
  double   syy = 0.0;
  double   interval_m2_ppb = 0.0;

  double   tau = 1.0;
  double   ppb = 0.0;
  double   stderr_ppb = 0.0;
  double   interval_mean_ppb = 0.0;
  double   interval_stddev_ppb = 0.0;
  double   interval_stderr_ppb = 0.0;
  int64_t  intercept_ns = 0;
};
#endif

// Return value reports coherent snapshot acquisition only.  snapshot.valid
// separately reports whether the estimator has a mature frequency result.
bool clocks_alpha_ocxo_tau_snapshot(time_clock_id_t clock,
                                    clocks_alpha_tau_snapshot_t* out);

// Compatibility alias; equivalent to clocks_alpha_ocxo_tau_snapshot().
bool clocks_alpha_tau_snapshot(time_clock_id_t clock,
                               clocks_alpha_tau_snapshot_t* out);


// -----------------------------------------------------------------------------
// Typed CLOCKS -> SYSTEM CLOCKS_FRAGMENT handoff
// -----------------------------------------------------------------------------
//
// CLOCKS owns measurement, campaign lifecycle, and scientific verdicts. SYSTEM
// owns the CLOCKS_FRAGMENT wire schema.  This handoff is deliberately normalized:
// one fact has one typed home before serialization.
//
//   live     = always-on instrument truth for one completed physical second
//   campaign = optional campaign-relative state and TEMPEST-only interpretation
//
// Campaign state must not clone live raw cycles, DAC/control state, lifetime
// statistics, or instrument clockfaces.  Recovery consumes the same canonical
// live state; there is no parallel restore-state mirror.
//
// clocks_fragment_snapshot_take() returns whether the typed handoff package was
// constructed. live.snapshot_ok separately reports whether Alpha supplied a
// coherent live instrument snapshot. If Beta has completed a public campaign
// record for the requested sequence, that campaign-only record is copied into
// campaign and atomically consumed.

static constexpr size_t CLOCKS_FRAGMENT_CAMPAIGN_NAME_MAX = 64U;
static constexpr size_t CLOCKS_FRAGMENT_STATE_NAME_MAX = 40U;

struct clocks_fragment_welford_snapshot_t {
  uint64_t n = 0;
  double mean = 0.0;
  double m2 = 0.0;
  double stddev = 0.0;
  double stderr_value = 0.0;
  double min = 0.0;
  double max = 0.0;
};

// Stable Alpha TAU sufficient state required for exact resurrection.  Transient
// writer/seqlock state is intentionally excluded.
struct clocks_fragment_tau_recovery_snapshot_t {
  bool valid = false;
  uint32_t reset_count = 0;
  uint32_t sample_count = 0;
  uint32_t interval_count = 0;
  uint32_t reject_count = 0;
  uint32_t gap_reset_count = 0;
  uint32_t last_pps_sequence = 0;
  uint32_t last_interval_pps_sequence = 0;
  uint64_t first_refined_ns = 0;
  uint64_t last_refined_ns = 0;
  int64_t last_fast_residual_ns = 0;

  uint64_t cumulative_reference_ns = 0;
  uint64_t cumulative_clock_ns = 0;
  double cumulative_clock_ns_exact = 0.0;

  double mean_x = 0.0;
  double mean_y = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;

  double interval_mean_ppb = 0.0;
  double interval_m2_ppb = 0.0;
};

struct clocks_fragment_ppb_value_snapshot_t {
  uint64_t sample_count = 0;
  double ppb = 0.0;
};

// Instrument-owned PPB populations only. Campaign PPB is intentionally absent;
// campaign-relative frequency belongs in clocks_fragment_campaign_snapshot_t.
struct clocks_fragment_ppb_buckets_snapshot_t {
  clocks_fragment_ppb_value_snapshot_t minute_10{};
  clocks_fragment_ppb_value_snapshot_t minute_60{};
  clocks_fragment_ppb_value_snapshot_t hour_8{};
  clocks_fragment_ppb_value_snapshot_t hour_24{};
  clocks_fragment_ppb_value_snapshot_t total{};
};

struct clocks_fragment_stats_clock_snapshot_t {
  clocks_fragment_welford_snapshot_t welford{};
  bool frequency_present = false;
  double tau = 1.0;
  double ppb = 0.0;
  clocks_fragment_ppb_buckets_snapshot_t ppb_buckets{};
};

// Canonical always-on statistics. Derived maturity aliases and interval-admission
// flight-recorder counters are intentionally omitted; Welford/TAU state already
// carries the information needed by dashboards and exact restore.
struct clocks_fragment_stats_snapshot_t {
  bool snapshot_ok = false;
  bool valid = false;
  uint32_t reset_count = 0;
  uint32_t update_count = 0;
  uint32_t last_pps_sequence = 0;
  bool completed_row_coherent = false;

  clocks_fragment_stats_clock_snapshot_t gnss{};
  clocks_fragment_stats_clock_snapshot_t dwt{};
  clocks_fragment_stats_clock_snapshot_t vclock{};
  clocks_fragment_stats_clock_snapshot_t ocxo1{};
  clocks_fragment_stats_clock_snapshot_t ocxo2{};
  clocks_fragment_stats_clock_snapshot_t pps_witness{};

  clocks_fragment_tau_recovery_snapshot_t ocxo1_tau_state{};
  clocks_fragment_tau_recovery_snapshot_t ocxo2_tau_state{};

  clocks_fragment_welford_snapshot_t ocxo1_dac{};
  clocks_fragment_welford_snapshot_t ocxo2_dac{};
};

// raw_cycles is the permanent compact sanity-check surface.  Keep the integrated
// delay verdict with each rail; deeper interrupt/Alpha forensic transcripts do
// not belong in the 1 Hz canonical handoff.
static constexpr size_t CLOCKS_FRAGMENT_DELAY_NAME_MAX = 40U;

struct clocks_fragment_raw_cycles_lane_t {
  bool snapshot_ok = false;
  bool forensics_snapshot_ok = false;
  bool valid = false;
  uint32_t completed_interval_count = 0;
  uint32_t observed_cycles = 0;
  uint32_t previous_observed_cycles = 0;
  int32_t residual_cycles = 0;

  char delay_status[CLOCKS_FRAGMENT_DELAY_NAME_MAX] = {0};
  bool delay_detail_present = false;
  char delay_by[CLOCKS_FRAGMENT_DELAY_NAME_MAX] = {0};
  bool residual_delay_valid = false;
  int32_t residual_delay_cycles = 0;
  char residual_delay_by[CLOCKS_FRAGMENT_DELAY_NAME_MAX] = {0};
  bool delay_explains_residual = false;
};

struct clocks_fragment_raw_cycles_snapshot_t {
  clocks_fragment_raw_cycles_lane_t pps{};
  clocks_fragment_raw_cycles_lane_t vclock{};
  clocks_fragment_raw_cycles_lane_t ocxo1{};
  clocks_fragment_raw_cycles_lane_t ocxo2{};
};

// Durable DAC/control knowledge. In-flight request/service bookkeeping is not
// persistent instrument state and is intentionally absent.
struct clocks_fragment_dither_lane_t {
  double value = 0.0;
  uint16_t hw_code = 0;
  bool readback_valid = false;
  uint16_t readback_code = 0;

  double servo_last_step = 0.0;
  double servo_last_residual = 0.0;
  uint32_t servo_settle_count = 0;
  uint32_t servo_adjustments = 0;

  bool servo_predictor_initialized = false;
  double servo_last_raw_residual = 0.0;
  double servo_filtered_residual = 0.0;
  double servo_filtered_slope = 0.0;
  double servo_predicted_residual = 0.0;
  uint32_t servo_predictor_updates = 0;
};

struct clocks_fragment_dac_snapshot_t {
  char servo_mode[CLOCKS_FRAGMENT_STATE_NAME_MAX] = {0};
  bool servo_active = false;
  char realization_mode[CLOCKS_FRAGMENT_STATE_NAME_MAX] = {0};
  bool dither_operator_enabled = false;
  clocks_fragment_dither_lane_t ocxo1{};
  clocks_fragment_dither_lane_t ocxo2{};
};

// TEMPEST-specific independent clock constructions.  These remain campaign
// interpretation, not a duplicate of the canonical instrument clockface.
enum class clocks_fragment_clock_candidate_status_t : uint8_t {
  UNAVAILABLE        = 0,
  SEEDED             = 1,
  ADVANCED           = 2,
  DELTA_INVALID      = 3,
  PUBLIC_COUNT_GAP   = 4,
  ARITHMETIC_FAILURE = 5,
};

struct clocks_fragment_clock_candidate_t {
  bool available = false;
  bool continuity_valid = false;
  clocks_fragment_clock_candidate_status_t status =
      clocks_fragment_clock_candidate_status_t::UNAVAILABLE;
  uint32_t start_public_count = 0;
  uint32_t last_public_count = 0;
  uint32_t interval_count = 0;
  uint64_t ns = 0;
  double fractional_ns = 0.0;
  bool residual_available = false;
  int64_t residual_ns = 0;
  double residual_ns_exact = 0.0;
};

struct clocks_fragment_clock_candidates_snapshot_t {
  char published_source[CLOCKS_FRAGMENT_STATE_NAME_MAX] = {0};
  clocks_fragment_clock_candidate_t phaseledger{};
  clocks_fragment_clock_candidate_t delta_cycles{};
  bool comparable = false;
  int64_t delta_cycles_minus_phaseledger_ns = 0;
  bool residuals_comparable = false;
  double delta_cycles_minus_phaseledger_residual_ns_exact = 0.0;
};

struct clocks_fragment_science_snapshot_t {
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

// Minimal exclusion testimony: enough to explain a non-science row without
// transporting the historical courtroom transcript.
struct clocks_fragment_rejection_snapshot_t {
  bool present = false;
  uint32_t reason_code = 0;
  char reason_name[CLOCKS_FRAGMENT_STATE_NAME_MAX] = {0};
  char source[CLOCKS_FRAGMENT_STATE_NAME_MAX] = {0};
  uint32_t lane_mask = 0;
};

// Only recovery state that changes interpretation/admission of this public row.
// Historical counters, per-lane mirrors, and narrative strings stay on reports.
struct clocks_fragment_recovery_snapshot_t {
  bool present = false;
  uint32_t generation = 0;
  bool transition_active = false;
  bool timeline_ready = false;
  bool clockface_ready = false;
  bool science_ready = false;
  bool degraded_active = false;
  bool science_quarantine_active = false;
  uint32_t science_quarantine_remaining = 0;
  bool reattach_stalled = false;
};

// Campaign-relative PPB is a genuine different scope from instrument TOTAL and
// rolling populations, so it has a separate canonical home here.
struct clocks_fragment_campaign_stats_snapshot_t {
  clocks_fragment_ppb_value_snapshot_t gnss{};
  clocks_fragment_ppb_value_snapshot_t dwt{};
  clocks_fragment_ppb_value_snapshot_t vclock{};
  clocks_fragment_ppb_value_snapshot_t ocxo1{};
  clocks_fragment_ppb_value_snapshot_t ocxo2{};
};

struct clocks_fragment_campaign_snapshot_t {
  bool present = false;
  uint32_t completed_second_sequence = 0;
  char campaign[CLOCKS_FRAGMENT_CAMPAIGN_NAME_MAX] = {0};
  char campaign_state[CLOCKS_FRAGMENT_STATE_NAME_MAX] = {0};
  uint32_t public_count = 0;

  // Campaign-relative clockfaces are a distinct scope, not duplicates of the
  // always-on instrument clockfaces in live.
  uint64_t gnss_ns = 0;
  uint64_t dwt_cycles = 0;
  uint64_t ocxo1_ns = 0;
  uint64_t ocxo2_ns = 0;

  char disposition[CLOCKS_FRAGMENT_STATE_NAME_MAX] = {0};
  bool timeline_valid = false;
  bool ocxo_clockface_valid = false;
  bool ocxo_science_valid = false;
  bool science_eligible = true;
  bool control_eligible = true;

  clocks_fragment_rejection_snapshot_t rejection{};
  clocks_fragment_recovery_snapshot_t recovery{};

  clocks_fragment_clock_candidates_snapshot_t ocxo1_clock_candidates{};
  clocks_fragment_science_snapshot_t ocxo1_science{};
  clocks_fragment_clock_candidates_snapshot_t ocxo2_clock_candidates{};
  clocks_fragment_science_snapshot_t ocxo2_science{};
  clocks_fragment_campaign_stats_snapshot_t stats{};
};

struct clocks_fragment_live_snapshot_t {
  bool snapshot_ok = false;
  bool valid = false;
  bool completed_row_coherent = false;
  uint32_t completed_pps_sequence = 0;
  uint32_t instrument_age_seconds = 0;

  // Always-on instrument clockfaces. These are the persistence/restore source.
  uint64_t instrument_gnss_ns = 0;
  uint64_t instrument_dwt_cycles = 0;
  uint64_t instrument_ocxo1_ns = 0;
  uint64_t instrument_ocxo2_ns = 0;
  uint32_t instrument_pps_sequence = 0;

  // Current PPS/VCLOCK anchoring/calibration required by the live subsystem.
  uint32_t dwt_cycles_per_second = 0;
  uint32_t dwt_at_pps_vclock = 0;
  uint32_t counter32_at_pps_vclock = 0;

  clocks_fragment_raw_cycles_snapshot_t raw_cycles{};
  clocks_fragment_stats_snapshot_t stats{};
  clocks_fragment_dac_snapshot_t dac{};
};

struct clocks_fragment_snapshot_t {
  clocks_fragment_live_snapshot_t live{};
  clocks_fragment_campaign_snapshot_t campaign{};

  // Publication-custody metadata only; SYSTEM does not serialize this field.
  // Once public campaign time already exists, the matching campaign delta must
  // accompany this sequence before SYSTEM releases it.  Keeping the bit here
  // avoids polluting the canonical live instrument state with campaign mirrors.
  bool campaign_row_expected = false;
};

bool clocks_fragment_snapshot_take(uint32_t completed_second_sequence,
                                   clocks_fragment_snapshot_t* out);
