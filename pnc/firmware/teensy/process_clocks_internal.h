// ============================================================================
// process_clocks_internal.h - Shared Internal State (Alpha <-> Beta)
// ============================================================================
//
// Doctrine:
//
//   Teensy owns every statistical quantity published in TIMEBASE_FRAGMENT.
//   The Pi does not compute derived stats; it transcribes what the Teensy
//   says.  This prevents diffusion of authority and gives every downstream
//   consumer a single well-defined source of truth.
//
// Candidate disposition:
//
//   After public PPS1, every survivable campaign second is still published.
//   ACCEPT candidates may advance Welford/servo state and may become durable
//   TIMEBASE rows.  SCIENCE_REJECT candidates testify to the same campaign
//   identity and remain explicitly DO_NOT_USE.  In STRICT mode Beta protects
//   statistical/servo state and the Pi logs rather than persists them.  In
//   FORENSIC mode the verdict remains honest, but Beta deliberately permits the
//   rejected numbers to contaminate campaign math and the Pi persists the row.
//   WATCHDOG_ANOMALY remains the continuity surrender path when an honest
//   candidate cannot be authored; gate mode never bypasses structural failure.
//
// Epoch authority:
//
//   PPS is a witness and selector. process_interrupt observes the physical
//   PPS edge, selects the corresponding VCLOCK edge identity, and publishes
//   a PPS/VCLOCK snapshot whose DWT coordinate is the observed VCLOCK/QTimer
//   edge. This keeps Delta Cycles in the same measured-edge DWT species as
//   OCXO while the physical PPS witness remains the smooth projection ruler.
//
//   Alpha installs the selected PPS/VCLOCK snapshot as a logical-zero event.
//   The captured synthetic counter32 values become per-lane zero-offset ticks:
//     logical_ticks = counter32_at_event - zero_offset_counter32
//   Alpha then extends accepted deltas into 64-bit logical tick ledgers so
//   campaign time does not wrap at the 32-bit / 10 MHz boundary.  Raw physical
//   PPS facts and raw 16-bit capture values remain diagnostics.
//
// Statistical surface (standardized):
//
//   Every Welford accumulator published in the fragment uses the same
//   suffix set:
//
//     <prefix>_welford_n        - uint64 sample count
//     <prefix>_welford_mean     - double, in the semantic unit of the signal
//     <prefix>_welford_stddev   - double, same unit
//     <prefix>_welford_stderr   - double, same unit (= stddev / sqrt(n))
//     <prefix>_welford_min      - double, same unit
//     <prefix>_welford_max      - double, same unit
//
//   Published Welford prefixes (seven total):
//
//     dwt_welford         - Teensy CPU XTAL offset samples (ppb)
//     vclock_welford      - bridge interpolation residual samples (ns)
//     ocxo1_welford       - OCXO1 PPS-interval residual samples (ns)
//     ocxo2_welford       - OCXO2 PPS-interval residual samples (ns)
//     pps_witness_welford - reserved PPS/VCLOCK phase-error surface (ns)
//     ocxo1_dac_welford   - OCXO1 DAC fractional code samples (LSB)
//     ocxo2_dac_welford   - OCXO2 DAC fractional code samples (LSB)
//
// Authorship map:
//
//   - g_gnss_ns_at_pps_vclock         CLOCKS-owned epoch-relative VCLOCK/GNSS
//                                     nanosecond counter sampled at the latest
//                                     selected PPS/VCLOCK edge.
//   - g_ocxo1_measured_gnss_ns_at_pps_vclock
//                                     CLOCKS-owned OCXO1 measured GNSS-elapsed
//                                     ledger sampled at the same PPS/VCLOCK edge.
//   - g_ocxo2_measured_gnss_ns_at_pps_vclock
//                                     CLOCKS-owned OCXO2 measured GNSS-elapsed
//                                     ledger sampled at the same PPS/VCLOCK edge.
//   - g_dwt_at_pps_vclock             Observed DWT coordinate of the selected
//                                     PPS/VCLOCK edge.
//   - g_dwt_cycles_between_pps_vclock Effective PPS/GPIO-derived DWT
//                                     cycles per GNSS second.  This remains
//                                     the smooth DWT-GNSS calibration source;
//                                     it is distinct from the VCLOCK rail's own
//                                     edge-to-edge prediction surface.
//   - g_counter32_at_pps_vclock       Synthetic VCLOCK counter identity of
//                                     the selected PPS/VCLOCK edge.
//
//
// Static prediction role:
//
//   Dynamic 100 Hz prediction has been retired. Alpha now records the prior
//   completed one-second DWT interval as the static prediction surface for four
//   independent rails:
//     PPS    — physical GPIO PPS edge-to-edge DWT interval
//     VCLOCK — canonical PPS/VCLOCK lattice edge-to-edge DWT interval
//     OCXO1  — OCXO1 authored edge-to-edge DWT interval
//     OCXO2  — OCXO2 authored edge-to-edge DWT interval
//   Beta publishes the compact audit.
//
// VCLOCK as measured peer of OCXO:
//
//   VCLOCK is both the sovereign GNSS-disciplined 10 MHz timebase and a peer
//   clock measured with the same clock_state_t / clock_measurement_t shape as
//   OCXO1 and OCXO2.  Any non-zero VCLOCK residual is diagnostic evidence
//   about bridge / DWT bookkeeping, not about the reference clock itself.
//
// ============================================================================

#pragma once

#include "config.h"
#include "payload.h"
#include "time.h"
#include "process_interrupt.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>

// ============================================================================
// DWT register defines
// ============================================================================

#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)
#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// ============================================================================
// DWT nanosecond conversion helpers
// ============================================================================

static inline uint64_t dwt_cycles_to_ns(uint64_t cycles) {
  return (cycles * DWT_NS_NUM) / DWT_NS_DEN;
}

static inline uint64_t dwt_ns_to_cycles(uint64_t ns) {
  return (ns * DWT_NS_DEN) / DWT_NS_NUM;
}

// ============================================================================
// Always-on DWT-GNSS anchor state (alpha-owned, beta-readable)
//
// All five of these globals are authored by alpha::pps_selector_callback
// from process_interrupt's canonical PPS/VCLOCK snapshots. PPS selects and
// audits the relationship; the public DWT coordinate itself is the observed
// VCLOCK/QTimer event coordinate so the bridge, VCLOCK heartbeat, and OCXO
// one-second compare bookends share one measured-edge species.
// ============================================================================

// Canonical VCLOCK/GNSS and measured OCXO 64-bit ledgers at the most recent
// selected PPS/VCLOCK edge. Alpha authors these; Beta publishes them;
// stateless time.h projection uses them as interpolation bases.
extern volatile uint64_t g_gnss_ns_at_pps_vclock;

// Public, visible-origin-normalized OCXO nanosecond ledgers.  These are the
// values clients should use for OCXO clock-domain time after SmartZero install.
extern volatile uint64_t g_ocxo1_measured_gnss_ns_at_pps_vclock;
extern volatile uint64_t g_ocxo2_measured_gnss_ns_at_pps_vclock;

// Physical bridge-measured ledgers retained as forensic/raw evidence.
extern volatile uint64_t g_ocxo1_physical_measured_gnss_ns_at_pps_vclock;
extern volatile uint64_t g_ocxo2_physical_measured_gnss_ns_at_pps_vclock;

// Observed DWT_CYCCNT coordinate of the most recent selected PPS/VCLOCK epoch
// (snap.dwt_at_edge). Under the VCLOCK-domain architecture this is the
// selected VCLOCK QTimer edge after the physical PPS pulse, not the raw GPIO
// ISR capture and not a PPS-derived enhanced estimate.
extern volatile uint32_t g_dwt_at_pps_vclock;

// Cumulative sum of dwt_cycles_between_pps across the campaign.
// Advanced in pps_selector_callback by the latest measurement.
extern volatile uint64_t g_dwt_cycle_count_total;

// Most recent one-second DWT/GNSS slope measurement.  Preferred source is the
// physical PPS/GPIO witness interval, because the QTimer/VCLOCK event rail is
// quantized to a 4-cycle lattice.  The legacy name remains while downstream
// callers migrate; semantically this is the effective DWT cycles per GNSS
// second.
extern volatile uint32_t g_dwt_cycles_between_pps_vclock;

// Exact observed selected PPS/VCLOCK DWT edge-to-edge interval, authored
// directly from consecutive snap.dwt_at_edge values. Delta Cycles residuals
// use this species-pure reference so the reference interval is formed by the
// same observed DWT-at-edge subtraction doctrine as OCXO intervals.
extern volatile uint32_t g_pps_vclock_dwt_cycles_between_edges;
extern volatile bool     g_pps_vclock_dwt_cycles_between_edges_valid;

// process_interrupt-authored synthetic 32-bit VCLOCK identity of the most
// recent canonical PPS/VCLOCK epoch (snap.counter32_at_edge).  This is the compact
// clock identity selected by the physical PPS pulse; it is not an ambient
// hardware read.
extern volatile uint32_t g_counter32_at_pps_vclock;

// Scalar PPS→VCLOCK phase in DWT cycles.  This is the distance from the
// physical PPS edge to the selected first-after-PPS VCLOCK edge, reduced into
// one 10 MHz cell.  It is a scalar by construction; no validity flag is
// published.
extern volatile int32_t g_pps_vclock_phase_cycles;

// Diagnostic — last seen counter32_at_event (CH3 ISR capture).
// Distinct from g_counter32_at_pps_vclock: this one IS authored from the
// CH3 ISR's event.counter32_at_event, retained for VCLOCK-phase
// cross-checks; its name honestly says "event" not "pps."
extern volatile uint32_t g_last_vclock_event_counter32_at_event;

// Returns the most recent one-second measurement of DWT cycles
// between consecutive selected PPS/VCLOCK edges.  Semantic label for callers
// that want the "effective cycles per second" framing (the DWT-GNSS
// bridge in time.cpp is the primary consumer).
static inline uint32_t dwt_effective_cycles_per_pps_vclock_second(void) {
  return g_dwt_cycles_between_pps_vclock;
}

// Species-pure selected PPS/VCLOCK edge interval for Delta Cycles.
static inline uint32_t dwt_selected_pps_vclock_edge_cycles_per_second(void) {
  return g_pps_vclock_dwt_cycles_between_edges_valid
      ? g_pps_vclock_dwt_cycles_between_edges
      : 0U;
}

uint64_t clocks_dwt_cycles_at_dwt(uint32_t dwt32);

// ============================================================================
// OCXO public nanosecond authority mode
// ============================================================================
//
// TRADITIONAL_PPS_PROJECTION keeps the existing PPS-row OCXO public clock
// value: Alpha projects measured OCXO edge evidence to the selected PPS/VCLOCK
// DWT coordinate, then Beta may render the public clock from Delta Cycles
// totals.
//
// PPS_COUNTERLEDGER makes the public OCXO nanosecond clocks the exact
// PPS-synchronous CounterLedger whole-tick value plus the PhaseLedger 0..99 ns
// suffix resolved by the first observed OCXO one-second edge after that PPS.
// Alpha does not release the row to Beta until both lanes resolve the same PPS
// sequence.  Delta Cycles remains the independent observed-edge frequency
// candidate; bridge and static projection surfaces remain forensic only.

enum class clocks_ocxo_public_ns_authority_t : uint8_t {
  TRADITIONAL_PPS_PROJECTION = 0,
  PPS_COUNTERLEDGER          = 1,
};

static constexpr clocks_ocxo_public_ns_authority_t
    CLOCKS_OCXO_PUBLIC_NS_AUTHORITY =
        clocks_ocxo_public_ns_authority_t::PPS_COUNTERLEDGER;

// Keep the full CounterLedger/PhaseLedger report surface enabled while it is
// public authority; focused reports and TIMEBASE then expose the same lineage.
static constexpr bool CLOCKS_OCXO_COUNTERLEDGER_REPORT_ONLY_ENABLED = true;

// CounterLedger frequency is fundamentally a long-baseline integer-tick
// witness. One OCXO tick is 100 ns, so sub-ppb behavior emerges over
// hundreds or thousands of PPS samples rather than in a single one-second
// row. Alpha therefore accumulates a non-authoritative rolling block so
// Beta can publish an independent block-level PPB/TAU witness beside the
// existing per-row residual and campaign candidate.
static constexpr uint32_t CLOCKS_OCXO_COUNTERLEDGER_BLOCK_SECONDS = 600U;

static inline constexpr bool clocks_ocxo_counterledger_mode(void) {
  return CLOCKS_OCXO_PUBLIC_NS_AUTHORITY ==
         clocks_ocxo_public_ns_authority_t::PPS_COUNTERLEDGER;
}

static inline constexpr bool clocks_ocxo_counterledger_mode_enabled(void) {
  return clocks_ocxo_counterledger_mode();
}

static inline constexpr bool clocks_ocxo_counterledger_report_enabled(void) {
  return CLOCKS_OCXO_COUNTERLEDGER_REPORT_ONLY_ENABLED ||
         clocks_ocxo_counterledger_mode();
}

static inline constexpr const char* clocks_ocxo_public_ns_authority_name(void) {
  return clocks_ocxo_counterledger_mode()
      ? "PPS_COUNTERLEDGER"
      : "TRADITIONAL_PPS_PROJECTION";
}

// CounterLedger RECOVER instrumentation reason IDs.
//
// Alpha stores numeric IDs only. Beta translates them to fixed string literals
// at report emission so no borrowed diagnostic string pointer crosses Alpha ->
// Beta -> Payload custody.
static constexpr uint32_t CLOCKS_COUNTERLEDGER_CAPTURE_GATE_OK = 0U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_CAPTURE_GATE_NOT_INITIALIZED = 1U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_CAPTURE_GATE_MISSING = 2U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_CAPTURE_GATE_INVALID = 3U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_CAPTURE_GATE_ALL_LANES_INVALID = 4U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_CAPTURE_GATE_SEQUENCE_MISMATCH = 5U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_CAPTURE_GATE_LANE_INVALID = 6U;

static inline const char* clocks_counterledger_capture_gate_reason_name(uint32_t id) {
  switch (id) {
    case CLOCKS_COUNTERLEDGER_CAPTURE_GATE_OK: return "OK";
    case CLOCKS_COUNTERLEDGER_CAPTURE_GATE_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case CLOCKS_COUNTERLEDGER_CAPTURE_GATE_MISSING: return "MISSING";
    case CLOCKS_COUNTERLEDGER_CAPTURE_GATE_INVALID: return "INVALID";
    case CLOCKS_COUNTERLEDGER_CAPTURE_GATE_ALL_LANES_INVALID: return "ALL_LANES_INVALID";
    case CLOCKS_COUNTERLEDGER_CAPTURE_GATE_SEQUENCE_MISMATCH: return "SEQUENCE_MISMATCH";
    case CLOCKS_COUNTERLEDGER_CAPTURE_GATE_LANE_INVALID: return "LANE_INVALID";
    default: return "UNKNOWN";
  }
}

static constexpr uint32_t CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_NONE = 0U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_NOT_INITIALIZED = 1U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_SEED_ACCEPTED = 2U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_INTERVAL_ACCEPTED = 3U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_GAP_RESEED = 4U;
static constexpr uint32_t CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_IMPLAUSIBLE_RESEED = 5U;

static inline const char* clocks_counterledger_sample_decision_name(uint32_t id) {
  switch (id) {
    case CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_NONE: return "NONE";
    case CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_SEED_ACCEPTED: return "SEED_ACCEPTED";
    case CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_INTERVAL_ACCEPTED: return "INTERVAL_ACCEPTED";
    case CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_GAP_RESEED: return "GAP_RESEED";
    case CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_IMPLAUSIBLE_RESEED: return "IMPLAUSIBLE_RESEED";
    default: return "UNKNOWN";
  }
}

static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_REASON_NONE = 0U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_REASON_NOT_INITIALIZED = 1U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_REASON_NO_PENDING_PPS = 2U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_REASON_ZERO_INTERVAL = 3U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_REASON_UNBRACKETED = 4U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_REASON_BAD_COUNTER_DELTA = 5U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_REASON_RESOLVED = 6U;

static inline const char* clocks_phaseledger_resolve_reason_name(uint32_t id) {
  switch (id) {
    case CLOCKS_PHASELEDGER_RESOLVE_REASON_NONE: return "NONE";
    case CLOCKS_PHASELEDGER_RESOLVE_REASON_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case CLOCKS_PHASELEDGER_RESOLVE_REASON_NO_PENDING_PPS: return "NO_PENDING_PPS";
    case CLOCKS_PHASELEDGER_RESOLVE_REASON_ZERO_INTERVAL: return "ZERO_INTERVAL";
    case CLOCKS_PHASELEDGER_RESOLVE_REASON_UNBRACKETED: return "UNBRACKETED";
    case CLOCKS_PHASELEDGER_RESOLVE_REASON_BAD_COUNTER_DELTA: return "BAD_COUNTER_DELTA";
    case CLOCKS_PHASELEDGER_RESOLVE_REASON_RESOLVED: return "RESOLVED";
    default: return "UNKNOWN";
  }
}

static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_SOURCE_NONE = 0U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_SOURCE_OCXO_EDGE = 1U;
static constexpr uint32_t CLOCKS_PHASELEDGER_RESOLVE_SOURCE_PPS_CATCHUP = 2U;

static inline const char* clocks_phaseledger_resolve_source_name(uint32_t id) {
  switch (id) {
    case CLOCKS_PHASELEDGER_RESOLVE_SOURCE_OCXO_EDGE: return "OCXO_EDGE";
    case CLOCKS_PHASELEDGER_RESOLVE_SOURCE_PPS_CATCHUP: return "PPS_CATCHUP";
    default: return "NONE";
  }
}

struct clocks_alpha_ocxo_counterledger_snapshot_t {
  bool     valid = false;
  bool     initialized = false;
  bool     interval_valid = false;
  bool     report_enabled = false;
  bool     authority_enabled = false;
  uint32_t clock_id = 0;
  uint32_t pps_sequence = 0;
  uint32_t sample_count = 0;
  uint32_t zero_counter32 = 0;
  uint32_t last_counter32 = 0;
  uint64_t ticks64 = 0;
  uint64_t ns = 0;
  uint32_t last_delta_ticks = 0;
  uint64_t interval_ns = 0;
  int64_t  fast_residual_ns = 0;

  // PhaseLedger suffix for CounterLedger.  The integer CounterLedger rail
  // owns whole 100 ns OCXO ticks sampled at PPS.  PhaseLedger supplies only
  // the bounded 0..99 ns low-order suffix by measuring where the PPS edge
  // landed inside the adjacent observed OCXO tick lattice.
  bool     phase_valid = false;
  bool     phase_pending = false;
  bool     phase_near_boundary = false;
  uint32_t phase_source_id = 0;
  uint32_t phase_pps_sequence = 0;
  uint32_t phase_lag_pps = 0;
  uint32_t phase_pps_dwt_at_edge = 0;
  uint32_t phase_prev_ocxo_dwt_at_edge = 0;
  uint32_t phase_next_ocxo_dwt_at_edge = 0;
  uint32_t phase_ocxo_interval_cycles = 0;
  uint32_t phase_pps_delta_cycles = 0;
  uint32_t phase_after_last_00_ns = 0;
  uint32_t phase_to_next_00_ns = 0;
  int32_t  phase_raw_delta_ns = 0;
  int32_t  phase_unwrapped_delta_ns = 0;
  int64_t  phase_unwrapped_carry_ticks = 0;
  bool     phase_wrap_event = false;
  uint32_t phase_wrap_count = 0;
  uint32_t phase_resolve_count = 0;
  uint32_t phase_pending_overwrite_count = 0;

  // PhaseLedger pending-ring custody.  The legacy single pending slot could
  // lose a PPS phase fact if the next PPS arrived before the correct OCXO
  // edge pair bracketed it.  These fields expose the small per-lane ring that
  // now preserves several unresolved PPS facts without relaxing any resolve
  // gate.
  uint32_t phase_pending_capacity = 0;
  uint32_t phase_pending_depth = 0;
  uint32_t phase_pending_depth_max = 0;
  uint32_t phase_pending_enqueue_count = 0;
  uint32_t phase_pending_overflow_count = 0;
  uint32_t phase_pending_drop_count = 0;
  uint32_t phase_pending_resolve_count = 0;
  uint32_t phase_pending_unbracketed_count = 0;
  uint32_t phase_pending_oldest_pps_sequence = 0;
  uint32_t phase_pending_newest_pps_sequence = 0;
  uint32_t phase_pending_last_resolved_pps_sequence = 0;
  uint32_t phase_pending_last_dropped_pps_sequence = 0;
  uint32_t phase_pending_last_matched_index = 0;

  // PhaseLedger resolver liveness.  The pending ring is only custody memory;
  // these fields prove whether Alpha had a recent OCXO edge pair available,
  // whether PPS-sample-side catch-up attempted to use it, and which source
  // produced the last resolver verdict.
  bool     phase_last_edge_pair_valid = false;
  uint32_t phase_last_edge_pair_previous_dwt = 0;
  uint32_t phase_last_edge_pair_next_dwt = 0;
  uint32_t phase_last_edge_pair_interval_cycles = 0;
  uint32_t phase_last_edge_pair_counter_delta_ticks = 0;
  uint32_t phase_last_edge_pair_update_count = 0;
  uint32_t phase_catchup_attempt_count = 0;
  uint32_t phase_catchup_success_count = 0;
  uint32_t phase_catchup_no_edge_pair_count = 0;
  uint32_t phase_catchup_no_pending_count = 0;
  uint32_t phase_catchup_unbracketed_count = 0;
  uint32_t phase_catchup_bad_counter_delta_count = 0;
  uint32_t phase_last_catchup_pps_sequence = 0;
  uint32_t phase_last_catchup_reason_id = CLOCKS_PHASELEDGER_RESOLVE_REASON_NONE;
  uint32_t phase_last_resolve_source_id = CLOCKS_PHASELEDGER_RESOLVE_SOURCE_NONE;

  uint32_t phase_invalid_count = 0;

  bool     refined_valid = false;
  bool     refined_interval_valid = false;
  uint64_t refined_ns = 0;
  uint64_t refined_interval_ns = 0;
  int64_t  refined_fast_residual_ns = 0;

  // Capture-custody counters.  These are report-only unless
  // CLOCKS_OCXO_PUBLIC_NS_AUTHORITY == PPS_COUNTERLEDGER.
  bool     last_capture_available = false;
  bool     last_capture_valid = false;
  bool     last_capture_lane_valid = false;
  bool     last_capture_all_lanes_valid = false;
  bool     last_capture_sequence_match = false;
  uint32_t last_capture_sequence = 0;
  uint32_t last_capture_window_cycles = 0;
  uint32_t update_count = 0;
  uint32_t capture_missing_count = 0;
  uint32_t capture_invalid_count = 0;
  uint32_t lane_capture_invalid_count = 0;
  uint32_t sequence_mismatch_count = 0;
  uint32_t all_lanes_invalid_count = 0;
  uint32_t interval_gap_count = 0;
  uint32_t interval_implausible_count = 0;
  uint32_t last_implausible_delta_ticks = 0;
  uint32_t recover_reprime_count = 0;
  uint32_t plausible_min_delta_ticks = 0;
  uint32_t plausible_max_delta_ticks = 0;

  // RECOVER/capture instrumentation. Lifetime counters describe the current
  // SmartZero epoch. recover_* counters are reset at each RECOVER reprime and
  // answer whether the post-recovery PPS capture path is missing, rejecting,
  // seeding, accepting intervals, resolving PhaseLedger, and maturing refined
  // intervals.
  uint32_t capture_gate_attempt_count = 0;
  uint32_t capture_gate_ready_count = 0;
  uint32_t capture_gate_reject_count = 0;
  uint32_t capture_gate_reason_id = CLOCKS_COUNTERLEDGER_CAPTURE_GATE_OK;
  uint32_t recover_capture_gate_count = 0;
  uint32_t recover_capture_ready_count = 0;
  uint32_t recover_capture_reject_count = 0;

  uint32_t sample_attempt_count = 0;
  uint32_t sample_not_initialized_count = 0;
  uint32_t sample_seed_accept_count = 0;
  uint32_t sample_interval_accept_count = 0;
  uint32_t sample_gap_reseed_count = 0;
  uint32_t sample_implausible_reseed_count = 0;
  uint32_t recover_sample_attempt_count = 0;
  uint32_t recover_sample_seed_count = 0;
  uint32_t recover_sample_interval_accept_count = 0;
  uint32_t recover_sample_gap_reseed_count = 0;
  uint32_t recover_sample_implausible_reseed_count = 0;
  uint32_t last_sample_decision_id = CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_NONE;
  uint32_t last_sample_pps_sequence = 0;
  uint32_t last_sample_previous_pps_sequence = 0;
  uint32_t last_sample_counter32 = 0;
  uint32_t last_sample_previous_counter32 = 0;
  uint32_t last_sample_delta_ticks = 0;

  uint32_t phase_resolve_attempt_count = 0;
  uint32_t phase_resolve_no_pending_count = 0;
  uint32_t phase_resolve_zero_interval_count = 0;
  uint32_t phase_resolve_unbracketed_count = 0;
  uint32_t phase_resolve_counter_delta_bad_count = 0;
  uint32_t recover_phase_resolve_attempt_count = 0;
  uint32_t recover_phase_resolve_success_count = 0;
  uint32_t recover_phase_resolve_unbracketed_count = 0;
  uint32_t last_phase_resolve_reason_id = CLOCKS_PHASELEDGER_RESOLVE_REASON_NONE;
  uint32_t last_phase_resolve_pps_sequence = 0;
  uint32_t last_phase_resolve_counter_delta_ticks = 0;
  uint32_t last_phase_resolve_interval_cycles = 0;
  uint32_t last_phase_resolve_pps_delta_cycles = 0;

  uint32_t refined_interval_accept_count = 0;
  uint32_t recover_refined_interval_accept_count = 0;

  // Rolling block witness. These fields are report-only unless/until the
  // CounterLedger rail is promoted to public authority. block_* is the
  // in-progress block; completed_block_* is the most recent full block.
  bool     block_valid = false;
  uint32_t block_window_seconds = 0;
  uint32_t block_start_pps_sequence = 0;
  uint32_t block_end_pps_sequence = 0;
  uint32_t block_interval_count = 0;
  uint64_t block_ticks = 0;
  uint64_t block_ns = 0;
  int64_t  block_fast_residual_sum_ns = 0;
  double   block_mean_fast_residual_ns = 0.0;
  double   block_tau = 1.0;
  double   block_ppb = 0.0;
  bool     block_phase_valid = false;
  uint64_t block_ns_with_phase = 0;
  int64_t  block_fast_residual_sum_ns_with_phase = 0;
  double   block_mean_fast_residual_ns_with_phase = 0.0;
  double   block_tau_with_phase = 1.0;
  double   block_ppb_with_phase = 0.0;

  bool     completed_block_valid = false;
  uint32_t completed_block_count = 0;
  uint32_t completed_block_start_pps_sequence = 0;
  uint32_t completed_block_end_pps_sequence = 0;
  uint32_t completed_block_interval_count = 0;
  uint64_t completed_block_ticks = 0;
  uint64_t completed_block_ns = 0;
  int64_t  completed_block_fast_residual_sum_ns = 0;
  double   completed_block_mean_fast_residual_ns = 0.0;
  double   completed_block_tau = 1.0;
  double   completed_block_ppb = 0.0;
  bool     completed_block_phase_valid = false;
  uint64_t completed_block_ns_with_phase = 0;
  int64_t  completed_block_fast_residual_sum_ns_with_phase = 0;
  double   completed_block_mean_fast_residual_ns_with_phase = 0.0;
  double   completed_block_tau_with_phase = 1.0;
  double   completed_block_ppb_with_phase = 0.0;

  uint32_t block_gap_reset_count = 0;
};

bool clocks_alpha_ocxo_counterledger_snapshot(
    time_clock_id_t clock,
    clocks_alpha_ocxo_counterledger_snapshot_t* out);
bool clocks_alpha_ocxo_counterledger_ready(void);


// ============================================================================
// Alpha always-on OCXO TAU estimator
// ============================================================================
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

// ============================================================================
// Clock common tolerance
// ============================================================================
//
// Window-error tolerance applies symmetrically to all measured clocks
// (VCLOCK, OCXO1, OCXO2).  A window-error magnitude exceeding this
// constant on the bridge-derived between-edges interval increments
// window_mismatches.

static constexpr int64_t CLOCK_WINDOW_TOLERANCE_NS = 500LL;

// ============================================================================
// Clock state — compatibility mirror for measured clocks (VCLOCK, OCXO*)
// ============================================================================
//
// Canonical nanosecond clocks live in Alpha's explicit 64-bit PPS/VCLOCK-edge
// globals above.  This older struct remains as a compatibility/report mirror
// while Beta and downstream reports are migrated.
//

struct clock_state_t {
  // Ledger ns count at the edge being applied. For VCLOCK this is GNSS time;
  // for OCXO lanes this is the measured GNSS-elapsed OCXO ledger.
  volatile uint64_t ledger_ns_count_at_edge;

  // GNSS ns at the edge as reported by the bridge for this event.
  volatile uint64_t gnss_ns_at_edge;

  // Compatibility mirror of the Alpha-owned ledger sampled at the latest
  // PPS/VCLOCK edge.
  volatile uint64_t ledger_ns_count_at_pps_vclock;

  // (reference GNSS ns − ledger_ns_count_at_edge). For VCLOCK this should be
  // near zero. For OCXO this is the measured phase displacement of the OCXO
  // ledger relative to the selected PPS/VCLOCK reference.
  volatile int64_t  phase_offset_ns;

  // Set true on the first apply_edge call.
  volatile bool     zero_established;

  // Window-check accounting — the window is one bridge-derived
  // between-edges interval.  Positive window_error_ns means this edge
  // came LATE (gnss_ns_between > 1e9, clock running slow).
  volatile uint32_t window_checks;
  volatile uint32_t window_mismatches;
  volatile int64_t  window_error_ns;
};

struct clock_measurement_t {
  // Bridge-derived ns since previous edge minus 1e9.
  // Positive → clock fired early this second (running fast).
  // Negative → clock fired late this second (running slow).
  volatile int64_t  second_residual_ns;

  // Bridge-derived ns elapsed between this edge and the previous one.
  volatile uint64_t gnss_ns_between_edges;

  // ISR-captured DWT at this edge.
  volatile uint32_t dwt_at_edge;

  // DWT cycles elapsed between this edge and the previous one.
  volatile uint32_t dwt_cycles_between_edges;

  // Previous-edge tracking, used to compute the deltas above.
  volatile uint64_t prev_gnss_ns_at_edge;
  volatile uint32_t prev_dwt_at_edge;

  // Bridge bookend evidence (OCXO lanes): whether the last resolved interval
  // came from bracketing-anchor interpolation, the resolved edge's phase
  // offset from its lower anchor, the bracket span, and lifetime counts.
  volatile bool     bridge_anchored;
  volatile int32_t  bridge_phi_cycles;
  volatile uint32_t bridge_span_cycles;
  volatile uint32_t bridge_resolved_count;
  volatile uint32_t bridge_fallback_count;
};

extern clock_state_t       g_vclock_clock;
extern clock_measurement_t g_vclock_measurement;

extern clock_state_t       g_ocxo1_clock;
extern clock_state_t       g_ocxo2_clock;
extern clock_measurement_t g_ocxo1_measurement;
extern clock_measurement_t g_ocxo2_measurement;

// ============================================================================
// Alpha clock forensic snapshots
// ============================================================================
//
// These snapshots expose the last event alpha consumed for each measured
// clock, plus the exact epoch-relative counter32->ns arithmetic applied before
// handing the event to process_time's generalized projection model.  They are
// diagnostic only; summary remains the compact system-health surface.

struct clocks_alpha_lane_forensics_t {
  bool     valid;
  uint32_t update_count;

  uint32_t last_event_dwt;
  uint32_t last_event_counter32;

  // New preferred names.  The zero offset is the synthetic counter32 value
  // captured at CLOCKS.ZERO/START and used as the lane's logical origin.
  bool     zero_offset_valid;
  uint32_t zero_offset_counter32;
  uint32_t counter32_delta_since_zero_offset;
  uint32_t counter32_delta_since_previous_event;
  uint64_t logical_ticks64_since_zero;
  uint64_t nominal_ns64_since_zero;

  // Legacy aliases retained for report/back-compat consumers.
  uint32_t epoch_counter32;
  uint32_t counter32_delta_since_epoch;
  uint64_t nominal_ns_from_counter32_epoch;

  // Compatibility mirror.  When process_interrupt supplies a direct GNSS
  // timestamp this carries it; otherwise Alpha falls back to the lane ledger
  // coordinate used for projection/report compatibility.
  uint64_t event_gnss_ns;
  uint64_t previous_event_gnss_ns;

  // Process_interrupt-authored GNSS witness for the subscriber event.  OCXO
  // lanes are now consumed as ordinary edge facts.  The older quiet-zone sample
  // and boundary fields remain compatibility/report surfaces only and should
  // publish as sample == boundary with zero correction.
  bool     sample_gnss_ns_at_event_available;
  bool     previous_sample_gnss_ns_at_event_available;
  uint64_t sample_gnss_ns_at_event;
  uint64_t previous_sample_gnss_ns_at_event;

  int64_t  phase_offset_ns;

  uint64_t physical_measured_ns_at_edge;
  uint64_t visible_ns_at_edge;
  bool     visible_origin_phase_valid;
  uint32_t visible_origin_phase_offset_ns;

  uint64_t counter_nominal_ns_between_edges;
  uint64_t bridge_gnss_ns_between_edges;
  int64_t  bridge_residual_ns;
  bool     bridge_interval_valid;
  bool     bridge_anchored;
  int32_t  bridge_phi_cycles;
  uint32_t bridge_span_cycles;
  uint32_t bridge_resolved_count;
  uint32_t bridge_fallback_count;

  uint64_t ns_between_edges;
  uint32_t dwt_cycles_between_edges;

  // process_interrupt-authored DWT interval gate audit.  The normal Alpha
  // timing path consumes last_event_dwt / dwt_cycles_between_edges as the
  // effective subscriber coordinate.  These fields retain the raw observed
  // endpoint/interval and the gate decision that decided whether the EMA was
  // allowed to learn from that sample.
  bool     dwt_synthetic;
  bool     dwt_repair_candidate;
  uint32_t dwt_original_at_event;
  uint32_t dwt_predicted_at_event;
  uint32_t dwt_used_at_event;
  uint32_t dwt_isr_entry_raw;
  uint32_t dwt_event_from_isr_entry_raw;
  int32_t  dwt_isr_entry_to_event_correction_cycles;
  int32_t  dwt_published_minus_event_cycles;
  int32_t  dwt_used_minus_event_cycles;
  int32_t  dwt_synthetic_error_cycles;
  uint32_t dwt_synthetic_threshold_cycles;
  // Final DWT-at-edge publication tribunal transcript copied from
  // process_interrupt.  These fields do not authorize or repair anything in
  // Alpha/Beta; they preserve the court verdict beside the observed edge surfaces
  // so TIMEBASE_FORENSICS can explain raw_cycles excursions.
  uint32_t dwt_publication_verdict_mask;
  uint32_t dwt_publication_verdict_reason_id;
  uint32_t dwt_publication_watchdog_count;
  uint32_t dwt_publication_gate_cycles;
  uint32_t dwt_publication_cross_rail_gate_cycles;
  uint32_t dwt_publication_service_offset_gate_ticks;
  uint32_t dwt_publication_expected_counter_delta_ticks;
  uint32_t dwt_publication_observed_counter_delta_ticks;
  uint32_t dwt_publication_expected_interval_cycles;
  uint32_t dwt_publication_published_interval_cycles;
  uint32_t dwt_publication_observed_interval_cycles;
  int32_t  dwt_publication_published_interval_error_cycles;
  int32_t  dwt_publication_observed_interval_error_cycles;
  int32_t  dwt_publication_published_minus_observed_cycles;
  int32_t  dwt_publication_service_offset_signed_ticks;
  int64_t  dwt_publication_vclock_gnss_error_ns;

  bool     dwt_interval_gate_valid;
  bool     dwt_interval_sample_accepted;
  bool     dwt_interval_sample_rejected;
  bool     dwt_interval_ema_updated;
  uint32_t dwt_interval_observed_cycles;
  uint32_t dwt_interval_prediction_cycles;
  uint32_t dwt_interval_effective_cycles;
  int32_t  dwt_interval_residual_cycles;
  uint32_t dwt_interval_gate_threshold_cycles;
  uint32_t dwt_interval_accept_count;
  uint32_t dwt_interval_reject_count;
  bool     dwt_interval_resync_applied;
  uint32_t dwt_interval_resync_count;
  uint32_t dwt_interval_reject_streak;

  // process_interrupt-authored counter-adjacency audit.  For OCXO lanes this
  // proves whether the DWT interval sample was formed from adjacent one-second
  // target identities.  A rejected adjacency sample is custody evidence only:
  // process_interrupt publishes the predicted DWT endpoint and preserves the
  // observed endpoint here for TIMEBASE_FORENSICS/raw_cycles.
  bool     dwt_interval_adjacency_gate_valid;
  bool     dwt_interval_adjacency_ok;
  bool     dwt_interval_adjacency_rejected;
  uint32_t dwt_interval_counter_delta_ticks;
  uint32_t dwt_interval_expected_counter_delta_ticks;
  uint32_t dwt_interval_adjacency_reject_count;

  // process_interrupt-authored PPS-Yardstick inference audit (Stage 1 --
  // observational rail).  dwt_at_event remains EMA-authored; these fields
  // carry the parallel yardstick surface per row so TIMEBASE/raw_cycles can
  // adjudicate the Stage 2 authority flip side-by-side with the EMA math.
  bool     dwt_yardstick_valid;
  bool     dwt_yardstick_stale;
  bool     dwt_yardstick_seeded;
  bool     dwt_yardstick_excursion;
  uint32_t dwt_yardstick_pps_sequence;
  uint32_t dwt_yardstick_pps_seq_delta;
  uint32_t dwt_yardstick_g_now_cycles;
  uint32_t dwt_yardstick_g_prev_cycles;
  uint32_t dwt_yardstick_inferred_interval_cycles;
  uint32_t dwt_yardstick_observed_interval_cycles;
  int32_t  dwt_yardstick_inferred_minus_observed_cycles;
  uint32_t dwt_yardstick_inferred_endpoint_dwt;
  uint32_t dwt_yardstick_inferred_endpoint_frac_q16;
  int32_t  dwt_yardstick_endpoint_minus_observed_cycles;
  uint32_t dwt_yardstick_gate_threshold_cycles;
  uint32_t dwt_yardstick_gate_agree_count;
  uint32_t dwt_yardstick_gate_excursion_count;
  bool     dwt_yardstick_authority;
  uint32_t dwt_ema_dwt_at_event;
  uint32_t dwt_yardstick_auth_endpoint_dwt;
  uint32_t dwt_yardstick_auth_endpoint_frac_q16;
  int32_t  dwt_yardstick_auth_error_cycles;
  bool     dwt_yardstick_auth_anchor_applied;

  // SlipLedger correction summary from process_interrupt.  The subscriber
  // DWT value is already purified; these fields are the compact forensic
  // trail for any historical signed hardware-counter phase correction.
  bool     slipledger_active;
  bool     slipledger_event_corrected;
  bool     slipledger_event_violation;
  int32_t  slipledger_ticks;
  int32_t  slipledger_event_ticks;
  uint32_t slipledger_generation;
  uint32_t slipledger_observe_count;
  uint32_t slipledger_ok_count;
  uint32_t slipledger_violation_count;
  uint32_t slipledger_correction_count;
  uint32_t slipledger_noop_violation_count;
  uint32_t slipledger_early_count;
  uint32_t slipledger_late_count;
  uint32_t slipledger_one_second_observe_count;
  uint32_t slipledger_one_second_ok_count;
  uint32_t slipledger_one_second_violation_count;
  uint32_t slipledger_one_second_correction_count;
  uint32_t slipledger_last_expected_dwt;
  uint32_t slipledger_last_observed_dwt;
  uint32_t slipledger_last_authored_dwt;
  uint32_t slipledger_last_expected_interval_cycles;
  uint32_t slipledger_last_observed_interval_cycles;
  int32_t  slipledger_last_dwt_error_cycles;
  uint32_t slipledger_last_target_counter32;
  uint16_t slipledger_last_hardware_target_low16;
  uint16_t slipledger_last_ambient_low16;
  uint32_t slipledger_last_tick_mod;
  uint32_t slipledger_reason_code;
  uint32_t slipledger_last_correction_reason_code;
  int32_t  slipledger_last_correction_ticks;
  int32_t  slipledger_last_correction_dwt_error_cycles;

  int64_t  second_residual_ns;
  int64_t  window_error_ns;
  uint32_t window_checks;
  uint32_t window_mismatches;

  uint32_t diag_anchor_sequence_used;
  uint32_t diag_anchor_age_slots;
  uint32_t diag_anchor_selection_kind;
  uint32_t diag_anchor_dwt_at_edge;
  int64_t  diag_anchor_gnss_ns_at_edge;
  uint32_t diag_anchor_cps;
  uint64_t diag_anchor_ns_delta;
  uint32_t diag_anchor_failure_mask;

  // OCXO compare-service / EMA diagnostics copied from interrupt_capture_diag_t.
  // Valid only for OCXO1/OCXO2 lanes; zero for VCLOCK or unavailable diag.
  uint32_t diag_service_class;
  int32_t  diag_service_offset_signed_ticks;
  uint32_t diag_service_offset_abs_ticks;
  uint32_t diag_interpreted_late_ticks;
  uint32_t diag_early_ticks;
  uint32_t diag_target_delta_mod65536_ticks;
  uint32_t diag_arm_remaining_ticks;
  uint32_t diag_arm_to_isr_ticks;
  uint32_t diag_arm_to_isr_dwt_cycles;

  uint32_t diag_perishable_fact_sequence;
  int32_t  diag_service_correction_cycles;
  uint32_t diag_service_corrected_dwt_at_event;
  uint32_t diag_fact_ring_overflow_count;
  uint32_t diag_counter_delta_violation_count;
  uint32_t diag_last_bad_counter_delta;
  uint32_t diag_last_counter_delta_ticks;

  bool     diag_sample_phase_valid;
  uint32_t diag_sample_phase_ticks;
  uint32_t diag_sample_phase_ns;
  uint32_t diag_sample_phase_us;
  uint32_t diag_sample_period_ticks;
  uint32_t diag_sample_dwt_at_event;
  uint32_t diag_sample_counter32_at_event;
  uint32_t diag_boundary_dwt_at_event;
  uint32_t diag_boundary_counter32_at_event;
  int32_t  diag_boundary_correction_cycles;

  // SpinIdle / SpinCatch ISR-entry witness copied from interrupt_capture_diag_t.
  // These are diagnostic only: Alpha does not use them as timing authority.
  bool     spinidle_shadow_valid;
  uint32_t spinidle_shadow_dwt;
  uint32_t spinidle_shadow_to_isr_entry_cycles;
  uint32_t spinidle_shadow_valid_threshold_cycles;

  // process_interrupt's integrated causal conclusion.  Alpha transports this
  // verbatim; Beta renders it into each completed TIMEBASE row.
  interrupt_delay_forensics_t interrupt_delay{};

};

bool clocks_alpha_lane_forensics(time_clock_id_t clock,
                                 clocks_alpha_lane_forensics_t* out);

// ============================================================================
// PPS/VCLOCK edge authority courtroom
// ============================================================================
//
// Durable per-PPS proof of the selected PPS/VCLOCK DWT coordinate.  This is
// the permanent TIMEBASE_FORENSICS collateral for the foundational edge
// authority claim: the physical PPS witness, the VCLOCK observed edge, the
// phase-derived candidate, the chosen coordinate, the agreement
// gate, and the tautological GNSS self-map check.

struct clocks_pps_vclock_edge_forensics_t {
  bool     valid;
  uint32_t sequence;
  uint32_t update_count;
  uint32_t reject_count;

  uint32_t authority_dwt_at_edge;
  uint32_t pps_dwt_at_edge;
  uint32_t vclock_observed_dwt_at_edge;
  uint32_t vclock_predicted_dwt_at_edge;
  uint32_t pps_projected_vclock_dwt_at_edge;

  bool     observed_phase_valid;
  bool     learned_phase_valid;
  uint32_t observed_phase_cycles;
  uint32_t learned_phase_cycles;

  uint32_t gate_cycles;
  uint32_t agreement_span_cycles;
  uint32_t decision;
  uint32_t invalid_mask;

  int32_t  authority_minus_pps_cycles;
  int32_t  authority_minus_vclock_observed_cycles;
  int32_t  authority_minus_prediction_cycles;
  int32_t  prediction_minus_pps_projected_cycles;
  int32_t  pps_projected_minus_observed_cycles;
  int32_t  observed_minus_prediction_cycles;

  uint32_t counter32_at_edge;
  uint16_t ch3_at_edge;
  uint32_t dwt_cycles_per_second;
  uint32_t dwt_cycles_between_edges;
  uint32_t effective_dwt_cycles_per_second;

  bool     gnss_self_map_valid;
  bool     gnss_self_error_ok;
  uint32_t gnss_self_error_gate_ns;
  uint64_t expected_gnss_ns_at_edge;
  uint64_t mapped_gnss_ns_at_edge;
  int64_t  gnss_self_error_ns;

  bool     counter_identity_valid;
  uint64_t counter_identity_gnss_ns_at_edge;
  int64_t  counter_identity_minus_expected_ns;
};

bool clocks_alpha_pps_vclock_edge_forensics(
    clocks_pps_vclock_edge_forensics_t* out);

// ============================================================================
// Alpha reporting-only integrity counters
// ============================================================================
//
// Alpha owns checks that require the public time/projection species.  These
// counters are strictly diagnostic: no repair, no gating, no watchdog mutation.

struct clocks_alpha_integrity_ns_check_t {
  bool     valid = false;
  bool     last_ok = false;
  uint32_t test_count = 0;
  uint32_t ok_count = 0;
  uint32_t bad_count = 0;
  uint32_t skipped_count = 0;

  uint32_t sequence = 0;
  uint32_t gate_ns = 0;
  uint64_t expected_ns = 0;
  uint64_t observed_ns = 0;
  int64_t  observed_minus_expected_ns = 0;
};

struct clocks_alpha_integrity_ocxo_check_t {
  clocks_alpha_integrity_ns_check_t interval;

  bool     previous_edge_valid = false;
  uint64_t previous_edge_projected_gnss_ns = 0;
  bool     previous_interval_valid = false;
  uint64_t previous_interval_ns = 0;
  uint64_t current_interval_ns = 0;
};

struct clocks_alpha_integrity_snapshot_t {
  bool     valid = false;
  uint32_t snapshot_count = 0;

  // At the canonical PPS/VCLOCK edge, public time projection must map the
  // authored edge DWT back to the exact GNSS second Alpha assigned.
  clocks_alpha_integrity_ns_check_t vclock_gnss_self_map;

  // Consecutive OCXO one-second edges are projected into public GNSS ns through
  // time.h.  The interval between projections should remain identical
  // second-to-second while the oscillator/servo state is unchanged.  Servo may
  // make this diagnostic count bad samples; it still never mutates authority.
  clocks_alpha_integrity_ocxo_check_t ocxo1_projected_gnss_interval;
  clocks_alpha_integrity_ocxo_check_t ocxo2_projected_gnss_interval;
};

bool clocks_alpha_integrity_snapshot(clocks_alpha_integrity_snapshot_t* out);

// ============================================================================
// Alpha event-flow forensic snapshots
// ============================================================================
//
// Report-only control-flow counters for diagnosing the handoff from
// process_interrupt subscriber events into Alpha's per-lane measurement and
// forensics stores. These fields are intentionally not published in
// TIMEBASE_FRAGMENT; Beta exposes them through CLOCKS.REPORT_ALPHA_FLOW.

struct clocks_alpha_event_flow_snapshot_t {
  uint32_t clock_id;

  uint32_t forensics_reset_count;
  uint32_t callback_entry_count;
  uint32_t callback_diag_present_count;
  uint32_t callback_diag_missing_count;
  uint32_t callback_accepted_count;
  uint32_t callback_rejected_epoch_not_ready_count;

  uint32_t apply_entry_count;
  uint32_t apply_phase_projected_count;  // retired; should remain zero
  uint32_t apply_ticks64_success_count;
  uint32_t apply_ticks64_failure_count;
  uint32_t apply_measured_second_count;
  uint32_t apply_measured_store_missing_count;
  uint32_t apply_time_update_count;
  uint32_t apply_static_prediction_count;
  uint32_t apply_complete_count;

  uint32_t forensics_publish_count;
  uint32_t forensics_publish_missing_store_count;
  uint32_t forensics_snapshot_request_count;
  uint32_t forensics_snapshot_consistent_count;
  uint32_t forensics_snapshot_valid_true_count;
  uint32_t forensics_snapshot_valid_false_count;
  uint32_t forensics_snapshot_retry_fail_count;
  uint32_t forensics_snapshot_missing_store_count;

  uint32_t last_stage;
  uint32_t last_failure_stage;

  uint32_t last_callback_dwt_at_event;
  uint32_t last_callback_counter32_at_event;
  uint64_t last_callback_gnss_ns_at_event;
  bool     last_callback_gnss_ns_available;
  bool     last_callback_diag_present;
  uint32_t last_callback_diag_anchor_selection_kind;
  uint32_t last_callback_diag_anchor_failure_mask;
  uint32_t last_callback_diag_service_class;
  int32_t  last_callback_diag_service_offset_ticks;
  uint32_t last_callback_diag_perishable_fact_sequence;
  bool     last_callback_sample_phase_valid;  // retired; should remain false
  uint32_t last_callback_sample_phase_ticks;

  uint32_t last_rejected_dwt_at_event;
  uint32_t last_rejected_counter32_at_event;
  uint64_t last_rejected_gnss_ns_at_event;

  uint32_t last_applied_dwt_at_event;
  uint32_t last_applied_counter32_at_event;
  uint32_t last_applied_phase_ticks;
  uint32_t last_applied_phase_cycles;
  uint32_t last_applied_dwt_cycles_between_edges;
  uint64_t last_applied_gnss_ns_between_edges;
  int64_t  last_applied_second_residual_ns;
  uint64_t last_applied_ns_now;
  uint32_t last_applied_counter32_delta_since_previous_event;

  bool     last_forensics_store_valid;
  uint32_t last_forensics_update_count;
  uint32_t last_forensics_seq;
  uint32_t last_forensics_last_event_dwt;
  uint32_t last_forensics_last_event_counter32;
  bool     last_forensics_sample_gnss_available;
  uint64_t last_forensics_sample_gnss_ns_at_event;

  bool     last_snapshot_return_value;
  bool     last_snapshot_store_valid;
  uint32_t last_snapshot_update_count;
  uint32_t last_snapshot_seq;
};

bool clocks_alpha_event_flow_snapshot(time_clock_id_t clock,
                                      clocks_alpha_event_flow_snapshot_t* out);

// ============================================================================
// Alpha OCXO visible-origin phase capture
// ============================================================================
//
// Introductory surface for the campaign-visible OCXO zero doctrine. Alpha
// computes this directly at SmartZero epoch install from the selected
// PPS/VCLOCK DWT anchor and each OCXO SmartZero anchor DWT. The full anchor
// span may be milliseconds; the visible-origin phase is the nanosecond residue
// of that span inside one 10 MHz cell. This is diagnostic only in this step;
// no public clock coordinate is adjusted yet.

struct clocks_alpha_ocxo_visible_origin_snapshot_t {
  bool     valid = false;
  // Retained for report compatibility; direct SmartZero-anchor computation
  // should complete synchronously at epoch install, so this should be false.
  bool     pending = false;
  bool     phase_offset_in_range = false;
  uint32_t clock_id = 0;
  uint32_t epoch_sequence = 0;
  uint32_t smartzero_sequence = 0;
  uint32_t capture_count = 0;

  uint32_t pps_vclock_dwt = 0;
  uint32_t ocxo_anchor_dwt = 0;
  uint32_t dwt_cycles_per_second = 0;
  uint32_t elapsed_cycles_since_pps_vclock = 0;
  uint64_t elapsed_ns_since_pps_vclock = 0;
  uint32_t phase_offset_ns = 0;

  // Campaign public-origin normalization, captured once from the first
  // PPS/VCLOCK row that has a valid OCXO projection.  phase_offset_ns hides the
  // sub-100 ns waveform residue; public_origin_offset_ns hides the much larger
  // arbitrary OCXO island displacement so public OCXO time starts at the same
  // visible origin as VCLOCK/GNSS for this epoch.
  bool     public_origin_valid = false;
  uint32_t public_origin_capture_count = 0;
  uint32_t public_origin_pps_sequence = 0;
  uint64_t public_origin_vclock_ns = 0;
  uint64_t public_origin_ocxo_ns_before_offset = 0;
  int64_t  public_origin_offset_ns = 0;
  uint64_t public_origin_ocxo_ns_after_offset = 0;
};

bool clocks_alpha_ocxo_visible_origin_snapshot(
    time_clock_id_t clock,
    clocks_alpha_ocxo_visible_origin_snapshot_t* out);

// Precomputed START handoff flag.  True only after Alpha has captured both
// OCXO public-origin offsets for the current epoch.  This is intentionally a
// tiny boolean read for Beta's campaign warmup path; detailed origin state
// remains available through clocks_alpha_ocxo_visible_origin_snapshot().
bool clocks_alpha_ocxo_public_origin_ready(void);

// ============================================================================
// Alpha RECOVER re-prime
// ============================================================================
//
// RECOVER preserves the installed SmartZero/service epoch, public-origin
// offsets, and long logical tick ledgers, but it creates a deliberate
// discontinuity in OCXO measurement custody.  Previous/pending OCXO edge state
// must not bridge the outage into the first post-recovery residual.
//
// A live warm recovery may arrive after a Pi-side publication blackout while
// the Teensy itself never rebooted.  This call begins the same asynchronous
// 50 ms + 50 ms physical-grid rephase used by START, but selects PRESERVE
// logical-epoch semantics.  It does not replace the Alpha epoch, re-subscribe
// callbacks, reset SmartZero/public-origin state, or disturb VCLOCK anchors.
// A true return means the transaction was accepted or already completed; Beta
// must consult clocks_alpha_ocxo_grid_rephase_status(RECOVER) before consuming
// the PPS recovery gate.
bool clocks_alpha_recover_rearm_interrupt_service(void);

// Beta calls this from the RECOVER gate after the recovery request is observed
// on a PPS/VCLOCK row and before recovered campaign publication resumes.
void clocks_alpha_recover_reprime_ocxo_state(void);
uint32_t clocks_alpha_recover_reprime_count(void);

// Compact recovery reattachment proof.  RECOVER deliberately cuts OCXO
// measurement custody.  Readiness is layered: a fresh integer clockface may be
// published before the stricter PhaseLedger/refined-interval science proof is
// complete.  Beta may therefore publish an explicitly degraded timeline row,
// but Welford, PPB, and servo remain gated until science_ready is true for both
// lanes.  This surface is report/control-plane evidence only.
struct clocks_alpha_recover_reattach_snapshot_t {
  // Readiness is deliberately split.  clockface_ready proves that the lane
  // can publish a fresh post-RECOVER OCXO clockface.  science_ready additionally
  // proves the PhaseLedger/refined interval required by Welford, PPB, and servo.
  // ready remains a compatibility alias for science_ready.
  bool     ready = false;
  bool     clockface_ready = false;
  bool     science_ready = false;
  uint32_t clock_id = 0;
  uint32_t reprime_count = 0;

  bool     forensics_ready = false;
  bool     forensics_valid = false;
  uint32_t forensics_update_count = 0;
  uint32_t forensics_last_event_dwt = 0;
  uint32_t forensics_last_event_counter32 = 0;
  uint32_t forensics_dwt_used_at_event = 0;

  bool     edge_history_ready = false;
  bool     edge_history_current_valid = false;
  bool     edge_history_previous_valid = false;
  uint32_t edge_history_update_count = 0;

  bool     projection_ready = false;
  bool     projection_available = false;
  bool     projection_valid = false;
  uint32_t projection_update_count = 0;
  uint32_t projection_compute_count = 0;
  uint32_t projection_source = 0;
  uint32_t projection_pps_sequence = 0;
  uint64_t projection_pps_vclock_ns = 0;
  uint64_t projection_projected_ocxo_ns_at_pps = 0;
  uint32_t projection_interval_dwt_cycles = 0;

  bool     pps_vclock_match = false;
  uint64_t expected_pps_vclock_ns = 0;

  bool     public_ns_nonzero = false;
  bool     physical_ns_nonzero = false;
  uint64_t current_public_ns = 0;
  uint64_t current_physical_ns = 0;

  uint32_t static_prediction_completed_interval_count = 0;
  bool     static_prediction_valid = false;

  // CounterLedger/PhaseLedger recovery reattachment proof.  In
  // PPS_COUNTERLEDGER mode, recover publication should not depend on legacy
  // PPS projection readiness; the public OCXO rail is the PPS-captured
  // counter ledger plus PhaseLedger suffix.
  bool     counterledger_mode = false;
  bool     counterledger_snapshot_ok = false;
  bool     counterledger_valid = false;
  bool     counterledger_initialized = false;
  bool     counterledger_capture_ready = false;
  bool     counterledger_interval_valid = false;
  bool     counterledger_phase_valid = false;
  bool     counterledger_phase_lag_ok = false;
  bool     counterledger_refined_valid = false;
  bool     counterledger_refined_interval_valid = false;
  uint32_t counterledger_sample_count = 0;
  uint32_t counterledger_pps_sequence = 0;
  uint32_t counterledger_phase_pps_sequence = 0;
  uint32_t counterledger_phase_lag_pps = 0;
  uint32_t counterledger_phase_pending_capacity = 0;
  uint32_t counterledger_phase_pending_depth = 0;
  uint32_t counterledger_phase_pending_depth_max = 0;
  uint32_t counterledger_phase_pending_enqueue_count = 0;
  uint32_t counterledger_phase_pending_overflow_count = 0;
  uint32_t counterledger_phase_pending_drop_count = 0;
  uint32_t counterledger_phase_pending_resolve_count = 0;
  uint32_t counterledger_phase_pending_unbracketed_count = 0;
  uint32_t counterledger_phase_pending_oldest_pps_sequence = 0;
  uint32_t counterledger_phase_pending_newest_pps_sequence = 0;
  uint32_t counterledger_phase_pending_last_resolved_pps_sequence = 0;
  uint32_t counterledger_phase_pending_last_dropped_pps_sequence = 0;
  uint32_t counterledger_phase_pending_last_matched_index = 0;
  bool     counterledger_phase_last_edge_pair_valid = false;
  uint32_t counterledger_phase_last_edge_pair_previous_dwt = 0;
  uint32_t counterledger_phase_last_edge_pair_next_dwt = 0;
  uint32_t counterledger_phase_last_edge_pair_interval_cycles = 0;
  uint32_t counterledger_phase_last_edge_pair_counter_delta_ticks = 0;
  uint32_t counterledger_phase_last_edge_pair_update_count = 0;
  uint32_t counterledger_phase_catchup_attempt_count = 0;
  uint32_t counterledger_phase_catchup_success_count = 0;
  uint32_t counterledger_phase_catchup_no_edge_pair_count = 0;
  uint32_t counterledger_phase_catchup_no_pending_count = 0;
  uint32_t counterledger_phase_catchup_unbracketed_count = 0;
  uint32_t counterledger_phase_catchup_bad_counter_delta_count = 0;
  uint32_t counterledger_phase_last_catchup_pps_sequence = 0;
  uint32_t counterledger_phase_last_catchup_reason_id = CLOCKS_PHASELEDGER_RESOLVE_REASON_NONE;
  uint32_t counterledger_last_phase_resolve_source_id = CLOCKS_PHASELEDGER_RESOLVE_SOURCE_NONE;
  uint32_t counterledger_last_delta_ticks = 0;
  uint32_t counterledger_interval_implausible_count = 0;
  uint32_t counterledger_last_implausible_delta_ticks = 0;
  uint32_t counterledger_recover_reprime_count = 0;
  uint32_t counterledger_plausible_min_delta_ticks = 0;
  uint32_t counterledger_plausible_max_delta_ticks = 0;
  bool     counterledger_last_capture_available = false;
  bool     counterledger_last_capture_valid = false;
  bool     counterledger_last_capture_lane_valid = false;
  bool     counterledger_last_capture_all_lanes_valid = false;
  bool     counterledger_last_capture_sequence_match = false;
  uint32_t counterledger_last_capture_sequence = 0;
  uint32_t counterledger_last_capture_window_cycles = 0;
  uint32_t counterledger_capture_missing_count = 0;
  uint32_t counterledger_capture_invalid_count = 0;
  uint32_t counterledger_lane_capture_invalid_count = 0;
  uint32_t counterledger_sequence_mismatch_count = 0;
  uint32_t counterledger_all_lanes_invalid_count = 0;
  uint32_t counterledger_capture_gate_attempt_count = 0;
  uint32_t counterledger_capture_gate_ready_count = 0;
  uint32_t counterledger_capture_gate_reject_count = 0;
  uint32_t counterledger_capture_gate_reason_id = CLOCKS_COUNTERLEDGER_CAPTURE_GATE_OK;
  uint32_t counterledger_recover_capture_gate_count = 0;
  uint32_t counterledger_recover_capture_ready_count = 0;
  uint32_t counterledger_recover_capture_reject_count = 0;

  uint32_t counterledger_sample_attempt_count = 0;
  uint32_t counterledger_sample_not_initialized_count = 0;
  uint32_t counterledger_sample_seed_accept_count = 0;
  uint32_t counterledger_sample_interval_accept_count = 0;
  uint32_t counterledger_sample_gap_reseed_count = 0;
  uint32_t counterledger_sample_implausible_reseed_count = 0;
  uint32_t counterledger_recover_sample_attempt_count = 0;
  uint32_t counterledger_recover_sample_seed_count = 0;
  uint32_t counterledger_recover_sample_interval_accept_count = 0;
  uint32_t counterledger_recover_sample_gap_reseed_count = 0;
  uint32_t counterledger_recover_sample_implausible_reseed_count = 0;
  uint32_t counterledger_last_sample_decision_id = CLOCKS_COUNTERLEDGER_SAMPLE_DECISION_NONE;
  uint32_t counterledger_last_sample_pps_sequence = 0;
  uint32_t counterledger_last_sample_previous_pps_sequence = 0;
  uint32_t counterledger_last_sample_counter32 = 0;
  uint32_t counterledger_last_sample_previous_counter32 = 0;
  uint32_t counterledger_last_sample_delta_ticks = 0;

  uint32_t counterledger_phase_resolve_attempt_count = 0;
  uint32_t counterledger_phase_resolve_no_pending_count = 0;
  uint32_t counterledger_phase_resolve_zero_interval_count = 0;
  uint32_t counterledger_phase_resolve_unbracketed_count = 0;
  uint32_t counterledger_phase_resolve_counter_delta_bad_count = 0;
  uint32_t counterledger_recover_phase_resolve_attempt_count = 0;
  uint32_t counterledger_recover_phase_resolve_success_count = 0;
  uint32_t counterledger_recover_phase_resolve_unbracketed_count = 0;
  uint32_t counterledger_last_phase_resolve_reason_id = CLOCKS_PHASELEDGER_RESOLVE_REASON_NONE;
  uint32_t counterledger_last_phase_resolve_pps_sequence = 0;
  uint32_t counterledger_last_phase_resolve_counter_delta_ticks = 0;
  uint32_t counterledger_last_phase_resolve_interval_cycles = 0;
  uint32_t counterledger_last_phase_resolve_pps_delta_cycles = 0;
  uint32_t counterledger_refined_interval_accept_count = 0;
  uint32_t counterledger_recover_refined_interval_accept_count = 0;
  uint64_t counterledger_ns = 0;
  uint64_t counterledger_interval_ns = 0;
  uint64_t counterledger_refined_ns = 0;
  uint64_t counterledger_refined_interval_ns = 0;
};

bool clocks_alpha_ocxo_recover_reattach_snapshot(
    time_clock_id_t clock,
    clocks_alpha_recover_reattach_snapshot_t* out);
bool clocks_alpha_recover_ocxo_reattach_ready(void);

// ============================================================================
// Alpha OCXO PPS-edge projection forensics
// ============================================================================
//
// Step A report-only surface for the coming PPS-founded OCXO nanosecond clock
// standard.  Alpha computes what the OCXO clock value would be at the current
// PPS/VCLOCK DWT coordinate from OCXO edge facts, but does not yet promote
// that value into g_ocxo*_measured_gnss_ns_at_pps_vclock or TIMEBASE.

struct clocks_alpha_ocxo_pps_projection_snapshot_t {
  bool     valid;
  uint32_t clock_id;
  uint32_t update_count;
  uint32_t compute_count;
  uint32_t invalid_no_edge_count;
  uint32_t invalid_no_interval_count;
  uint32_t invalid_target_out_of_window_count;

  // Static projection advancement diagnostics.  When the latest observed OCXO
  // edge is behind the PPS/VCLOCK target by more than one OCXO interval, Alpha
  // may advance the synthetic static edge a bounded number of one-second
  // intervals before projecting.  These fields expose that decision instead of
  // collapsing it into TARGET_OUT_OF_WINDOW.
  uint32_t static_projection_advance_count;        // cumulative advanced edges
  uint32_t last_static_projection_advance_count;   // advanced edges this compute
  uint32_t max_static_projection_advance_count;    // max advanced edges in one compute
  uint32_t static_projection_advance_limit;        // configured cap
  uint32_t target_delta_raw_cycles;                // pps_dwt - original edge0_dwt
  uint32_t target_overrun_cycles;                  // raw delta beyond one interval
  uint32_t max_target_overrun_cycles;              // cumulative max overrun seen

  // 0=NONE, 1=ACTUAL_BRACKET, 2=STATIC_NEXT_EDGE.
  uint32_t source;
  uint32_t last_invalid_reason;

  uint32_t pps_sequence;
  uint32_t pps_dwt_at_edge;
  uint64_t pps_vclock_ns;

  uint32_t edge0_dwt_at_edge;
  uint32_t edge0_counter32_at_edge;
  uint64_t edge0_ocxo_ns_at_edge;
  uint64_t edge0_measured_ns_at_edge;
  bool     edge0_sample_gnss_available;
  uint64_t edge0_sample_gnss_ns_at_event;
  bool     edge0_boundary_gnss_available;
  uint64_t edge0_boundary_gnss_ns_at_edge;

  uint32_t edge1_dwt_at_edge;
  uint32_t edge1_counter32_at_edge;
  uint64_t edge1_ocxo_ns_at_edge;
  uint64_t edge1_measured_ns_at_edge;
  bool     edge1_sample_gnss_available;
  uint64_t edge1_sample_gnss_ns_at_event;
  bool     edge1_boundary_gnss_available;
  uint64_t edge1_boundary_gnss_ns_at_edge;

  uint32_t interval_dwt_cycles;
  uint64_t interval_ocxo_ns;
  uint32_t target_delta_cycles;
  uint32_t target_remaining_cycles;
  uint64_t projected_ocxo_ns_at_pps;
  int64_t  projected_minus_existing_pps_ns;
  int64_t  projected_minus_vclock_ns;

  uint32_t latest_actual_interval_cycles;
  uint32_t static_prediction_completed_interval_count;
  bool     static_prediction_valid;
};

bool clocks_alpha_ocxo_pps_projection_snapshot(
    time_clock_id_t clock,
    clocks_alpha_ocxo_pps_projection_snapshot_t* out);

// ============================================================================
// Last-known interrupt diagnostics (alpha-owned, beta-readable)
// ============================================================================

extern interrupt_capture_diag_t g_pps_witness_diag;
extern interrupt_capture_diag_t g_ocxo1_interrupt_diag;
extern interrupt_capture_diag_t g_ocxo2_interrupt_diag;

static inline void clocks_capture_interrupt_diag(interrupt_capture_diag_t& dst,
                                                 const interrupt_capture_diag_t* src) {
  if (!src) {
    dst = interrupt_capture_diag_t{};
    dst.enabled = false;
    return;
  }
  dst = *src;
}

// ============================================================================
// AD5693R init
// ============================================================================

extern bool g_ad5693r_init_ok;

// ============================================================================
// OCXO DAC state — dual oscillator
// ============================================================================

struct ocxo_dac_state_t {
  double   dac_fractional;
  uint16_t dac_hw_code;
  uint32_t dac_min;
  uint32_t dac_max;

  // DAC authority.  The servo and Pi control plane keep a real-valued target.
  // When dither is disabled, the AD5693R receives one static rounded integer
  // code.  When dither is enabled, a foreground-serviced one-second realization
  // alternates adjacent integer codes without performing I2C from TimePop's
  // timed callback path.
  bool     dither_enabled;
  bool     dither_active_this_frame;
  bool     dither_current_phase_high;
  bool     dither_program_dirty;

  uint16_t dither_low_code;
  uint16_t dither_high_code;
  uint16_t dither_high_ms;
  uint16_t dither_last_frame_high_ms;

  bool     dither_pending_hw_write;
  uint16_t dither_pending_hw_code;
  uint32_t dither_pending_request_count;
  uint32_t dither_pending_overwrite_count;

  uint32_t dither_frame_count;
  uint32_t dither_transition_count;
  uint32_t dither_write_count;
  uint32_t dither_write_failure_count;
  uint32_t dither_skip_same_code_count;
  uint32_t dither_schedule_failure_count;

  uint32_t dither_service_count;
  uint32_t dither_service_write_count;
  uint32_t dither_service_skip_same_count;
  uint32_t dither_service_defer_count;

  double   servo_last_step;
  double   servo_last_residual;
  uint32_t servo_settle_count;
  uint32_t servo_adjustments;

  bool     servo_predictor_initialized;
  double   servo_last_raw_residual;
  double   servo_filtered_residual;
  double   servo_filtered_slope;
  double   servo_predicted_residual;
  uint32_t servo_predictor_updates;

  // Servo/DAC ownership.  Beta's 1 Hz TIMEBASE path may only request a
  // real-valued DAC target.  The dither owner consumes the request at a frame
  // boundary, installs the fractional target, and owns every hardware-facing
  // low/high/static realization.
  uint32_t servo_hold_count;
  uint8_t  servo_hold_reason;
  uint8_t  servo_quarantine_reason;
  uint32_t servo_quarantine_remaining;
  uint32_t servo_quarantine_begin_count;
  uint32_t servo_quarantine_consumed_count;
  uint32_t servo_commit_fault_hold_count;
  uint32_t servo_request_overwrite_count;
  uint32_t servo_request_install_count;
  uint32_t servo_request_dither_frame_install_count;
  uint32_t servo_request_static_install_count;
  uint32_t servo_request_static_write_failure_count;

  bool     pacing_pending;
  double   pacing_pending_target;
  double   pacing_pending_step;
  uint16_t pacing_pending_hw_code;
  uint64_t pacing_pending_since_second;
  uint64_t pacing_last_request_second;
  uint64_t pacing_last_commit_second;
  uint32_t pacing_intents;
  uint32_t pacing_deferred_count;
  uint32_t pacing_commit_count;
  uint32_t pacing_skip_small_delta_count;

  bool     io_last_write_ok;
  bool     io_fault_latched;
  uint32_t io_write_attempts;
  uint32_t io_write_successes;
  uint32_t io_write_failures;
  uint16_t io_last_attempted_hw_code;
  uint16_t io_last_good_hw_code;
  uint8_t  io_last_failure_stage;
};

extern ocxo_dac_state_t ocxo1_dac;
extern ocxo_dac_state_t ocxo2_dac;

// ============================================================================
// OCXO servo mode
// ============================================================================

enum class servo_mode_t : uint8_t {
  OFF   = 0,
  MEAN  = 1,
  TOTAL = 2,
  NOW   = 3,
};

extern servo_mode_t calibrate_ocxo_mode;

const char* servo_mode_str(servo_mode_t mode);
servo_mode_t servo_mode_parse(const char* s);

static constexpr int32_t  SERVO_MAX_STEP                = 64;
static constexpr uint32_t SERVO_SETTLE_SECONDS          = 5;
static constexpr uint32_t SERVO_MIN_SAMPLES             = 10;
static constexpr uint16_t SERVO_MIN_DAC_CODE_DELTA_LSB  = 1;

static constexpr uint8_t SERVO_HOLD_NONE                 = 0;
static constexpr uint8_t SERVO_HOLD_PENDING_COMMIT       = 1;
static constexpr uint8_t SERVO_HOLD_SETTLE_QUARANTINE    = 2;
static constexpr uint8_t SERVO_HOLD_COMMIT_FAULT_BACKOFF = 3;
static constexpr uint8_t SERVO_HOLD_SMALL_STATIC_DELTA   = 4;

static constexpr uint32_t SERVO_DITHER_OWNER_SETTLE_QUARANTINE_ROWS = 2U;
static constexpr uint32_t SERVO_DITHER_OWNER_FAILURE_BACKOFF_ROWS    = 3U;

// Servo control doctrine:
//   positive ppb/tau>1 -> OCXO running fast  -> lower DAC code
//   negative ppb/tau<1 -> OCXO running slow  -> raise DAC code
// The DAC transfer is monotonic positive: higher DAC voltage makes the OCXO faster.
//
// DAC voltage doctrine:
//   The AD5693R OCXO DACs are configured for internal 2.5 V reference with
//   2× gain, giving an approximate 0..5 V hardware span.  OCXO EFC authority
//   is intentionally hard-limited below 3.3 V at the DAC-code boundary.  This
//   ceiling is a drop-dead hardware-protection invariant, not merely a UI hint.

static constexpr double OCXO_DAC_INTERNAL_REF_VOLTAGE = 2.5;
static constexpr double OCXO_DAC_OUTPUT_GAIN = 2.0;
static constexpr double OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE =
    OCXO_DAC_INTERNAL_REF_VOLTAGE * OCXO_DAC_OUTPUT_GAIN;
static constexpr double OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE = 3.3;
static constexpr double OCXO_DAC_CODE_SCALE = 65536.0;
static constexpr uint16_t OCXO_DAC_MAX_HW_CODE = 65535;
static constexpr uint16_t OCXO_DAC_SAFE_MAX_HW_CODE =
    (uint16_t)((OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE /
                OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE) *
               OCXO_DAC_CODE_SCALE);

static_assert(OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE >
              OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE,
              "OCXO DAC safety ceiling must be below full scale");
static_assert(OCXO_DAC_SAFE_MAX_HW_CODE < OCXO_DAC_MAX_HW_CODE,
              "OCXO DAC safety ceiling must clamp the 0..5 V span");

static inline double ocxo_dac_clamp_real_value(double value) {
  if (value < 0.0) return 0.0;
  if (value > (double)OCXO_DAC_SAFE_MAX_HW_CODE) {
    return (double)OCXO_DAC_SAFE_MAX_HW_CODE;
  }
  return value;
}

static inline uint16_t ocxo_dac_rounded_hw_code_from_value(double value) {
  const double clamped = ocxo_dac_clamp_real_value(value);
  return (uint16_t)(clamped + 0.5);
}

static inline double ocxo_dac_voltage_from_code(double code) {
  return (ocxo_dac_clamp_real_value(code) / OCXO_DAC_CODE_SCALE) *
         OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE;
}

bool ocxo_dac_set(ocxo_dac_state_t& s, double value);
bool ocxo_dac_set_desired(ocxo_dac_state_t& s, double value);
void ocxo_dac_request_servo_target(ocxo_dac_state_t& s,
                                   double target,
                                   double planned_step,
                                   uint64_t request_second);
void ocxo_dac_clear_servo_request(ocxo_dac_state_t& s);
double ocxo_dac_fractional_snapshot(const ocxo_dac_state_t& s);
bool ocxo_dac_write_hw_code(ocxo_dac_state_t& s,
                            uint16_t hw_code,
                            bool latch_fault = true);
void ocxo_dac_predictor_reset(ocxo_dac_state_t& s);
void ocxo_dac_io_reset(ocxo_dac_state_t& s);
void ocxo_dac_retry_reset(ocxo_dac_state_t& s);

bool clocks_ocxo_dac_dither_enable(void);
bool clocks_ocxo_dac_dither_disable(void);
bool clocks_ocxo_dac_dither_operator_enabled(void);
bool clocks_ocxo_dac_dither_started(void);
bool clocks_ocxo_dac_dither_service_pending(void);
uint32_t clocks_ocxo_dac_dither_global_frame_count(void);
uint32_t clocks_ocxo_dac_dither_global_schedule_failures(void);
uint32_t clocks_ocxo_dac_dither_service_arm_count(void);
uint32_t clocks_ocxo_dac_dither_service_arm_failures(void);
const char* clocks_ocxo_dac_dither_context(void);

// Dither-owned foreground DAC actuator service.
// Servo/Beta queues intent only; this owner service owns dither-frame and
// static hardware realization, including failure isolation.
bool clocks_ocxo_dac_actuator_service_pending(void);
uint32_t clocks_ocxo_dac_actuator_service_arm_count(void);
uint32_t clocks_ocxo_dac_actuator_service_arm_failures(void);
uint32_t clocks_ocxo_dac_actuator_commit_attempt_count(void);
uint32_t clocks_ocxo_dac_actuator_commit_success_count(void);
uint32_t clocks_ocxo_dac_actuator_commit_failure_count(void);
const char* clocks_ocxo_dac_actuator_context(void);

// ============================================================================
// Campaign startup prologue
// ============================================================================
//
// START forms the normal current-row forensic candidate before applying its
// private prologue court. While the court holds, that candidate is consumed as
// private PPS0 evidence: Alpha and Beta startup bookends advance, but
// campaign_seconds remains zero and no TIMEBASE_FRAGMENT is published.
//
// The first following candidate that proves startup continuity is released as
// public PPS1 (teensy_pps_vclock_count=1, gnss_ns=1e9). There is no fixed-N row
// burial.
//
// RECOVER is different: it resumes an existing public identity and continues
// emitting timeline candidates while reattachment/science readiness is exposed
// explicitly.

// ============================================================================
// Campaign state
// ============================================================================

enum class clocks_campaign_state_t {
  STOPPED,
  STARTED,
  // RECOVERING is a real lifecycle state, not just a pending boolean.
  // Pi-side recovery may arm RECOVER after a watchdog has forced STOPPED;
  // firmware must then remain visibly in recovery until the next PPS/VCLOCK
  // row consumes request_recover and resumes STARTED publication.
  RECOVERING
};

extern volatile clocks_campaign_state_t campaign_state;
extern char     campaign_name[64];
extern uint64_t campaign_seconds;

extern volatile bool request_start;
extern volatile bool request_stop;
extern volatile bool request_recover;
extern volatile bool request_zero;

extern uint64_t recover_dwt_ns;
extern uint64_t recover_gnss_ns;
extern uint64_t recover_ocxo1_ns;
extern uint64_t recover_ocxo2_ns;

// ============================================================================
// Welford — unified accumulator
// ============================================================================
//
// One struct, one API, used for every published Welford accumulator.
// Samples are stored in the semantic unit of the signal being measured
// (ppb for frequency clocks, ns for phase offsets, LSB for DAC codes).
//
// Global Welford instances, one per published prefix:
//
//   welford_dwt          — Teensy CPU XTAL offset, in ppb
//   welford_vclock       — bridge interpolation residual, in ns
//   welford_ocxo1        — OCXO1 PPS-interval residual, in ns
//   welford_ocxo2        — OCXO2 PPS-interval residual, in ns
//   welford_pps_witness  — PPS/VCLOCK phase error, in ns
//                          (counter32-based, ordering-independent)
//   welford_ocxo1_dac    — OCXO1 DAC fractional code, in LSB
//   welford_ocxo2_dac    — OCXO2 DAC fractional code, in LSB
//

struct welford_t {
  uint64_t n;
  double   mean;
  double   m2;
  double   min_val;
  double   max_val;
};

extern welford_t welford_dwt;
extern welford_t welford_vclock;
extern welford_t welford_ocxo1;
extern welford_t welford_ocxo2;
extern welford_t welford_pps_witness;
extern welford_t welford_ocxo1_dac;
extern welford_t welford_ocxo2_dac;

void   welford_reset(welford_t& w);
void   welford_update(welford_t& w, double sample);
double welford_stddev(const welford_t& w);
double welford_stderr(const welford_t& w);

// ============================================================================
// Campaign-scoped accumulators
// ============================================================================

extern uint64_t dwt_cycle_count_total;
extern uint64_t gnss_raw_64;
extern uint64_t ocxo1_measured_gnss_ticks_64;
extern uint64_t ocxo2_measured_gnss_ticks_64;

// ============================================================================
// Watchdog anomaly latch
// ============================================================================

extern volatile bool     watchdog_anomaly_active;
extern volatile bool     watchdog_anomaly_publish_pending;
extern volatile uint32_t watchdog_anomaly_sequence;
extern char              watchdog_anomaly_reason[64];
extern volatile uint32_t watchdog_anomaly_detail0;
extern volatile uint32_t watchdog_anomaly_detail1;
extern volatile uint32_t watchdog_anomaly_detail2;
extern volatile uint32_t watchdog_anomaly_detail3;
extern volatile uint32_t watchdog_anomaly_trigger_dwt;

// ============================================================================
// Beta entry points
// ============================================================================

// Consume one immutable Alpha row after PPS, OCXO1, and OCXO2 all identify
// completed_pps_sequence.  Publication latency is deliberately irrelevant.
void clocks_beta_pps(uint32_t completed_pps_sequence);
void clocks_beta_features_init(void);
void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0 = 0,
                             uint32_t detail1 = 0,
                             uint32_t detail2 = 0,
                             uint32_t detail3 = 0);

// Verbose WATCHDOG_ANOMALY boundary used by process_interrupt publication
// courts.  process_clocks/Beta owns the strong implementation so it can latch
// campaign surrender before publishing the evidence packet.
void clocks_watchdog_anomaly_payload(const char* reason,
                                     const Payload& payload,
                                     uint32_t detail0 = 0,
                                     uint32_t detail1 = 0,
                                     uint32_t detail2 = 0,
                                     uint32_t detail3 = 0);

// ============================================================================
// Local CLOCKS epoch integration
// ============================================================================

// ZERO is now owned by CLOCKS.  Alpha selects a mathematically-qualified
// SmartZero acquisition surface and installs canonical logical origins.
// The legacy interrupt-capture entry point is retained as a compatibility
// wrapper during migration.
//
// Physical-grid rephase is a separate asynchronous installation fact.  START,
// ZERO, and RECOVER share the same TimePop-staged OCXO1-then-OCXO2 transaction;
// the owner determines whether logical epoch state is replaced or preserved.
// Priority-0 preemption may enlarge either 50 ms gap; it must never compress it.
enum class clocks_alpha_ocxo_grid_rephase_owner_t : uint8_t {
  NONE            = 0,
  SMARTZERO_EPOCH = 1,
  RECOVER         = 2,
};

enum class clocks_alpha_ocxo_grid_rephase_status_t : uint8_t {
  IDLE     = 0,
  PENDING  = 1,
  COMPLETE = 2,
  FAILED   = 3,
};

clocks_alpha_ocxo_grid_rephase_status_t
clocks_alpha_ocxo_grid_rephase_status(
    clocks_alpha_ocxo_grid_rephase_owner_t owner);
void clocks_alpha_ocxo_grid_rephase_acknowledge(
    clocks_alpha_ocxo_grid_rephase_owner_t owner);

// Compatibility report transcript.  The historic SmartZero naming remains on
// the wire, but RECOVER now travels through the same physical-grid machinery.
struct clocks_alpha_smartzero_delay_snapshot_t {
  bool     valid = false;
  bool     reference_from_pps_vclock = false;
  bool     ocxo1_zero_ok = false;
  bool     ocxo2_zero_ok = false;
  bool     ocxo1_minimum_satisfied = false;
  bool     ocxo2_minimum_satisfied = false;
  bool     all_minimums_satisfied = false;

  uint32_t epoch_sequence = 0;
  uint32_t smartzero_sequence = 0;
  uint32_t install_count = 0;
  uint32_t minimum_separation_us = 0;
  uint32_t minimum_separation_cycles = 0;

  uint32_t reference_dwt = 0;

  uint32_t ocxo1_earliest_dwt = 0;
  uint32_t ocxo1_install_begin_dwt = 0;
  uint32_t ocxo1_install_complete_dwt = 0;
  uint32_t ocxo1_reference_gap_cycles = 0;
  uint32_t ocxo1_lateness_cycles = 0;
  uint32_t ocxo1_zero_counter32 = 0;

  uint32_t ocxo2_earliest_dwt = 0;
  uint32_t ocxo2_install_begin_dwt = 0;
  uint32_t ocxo2_install_complete_dwt = 0;
  uint32_t ocxo2_from_ocxo1_gap_cycles = 0;
  uint32_t ocxo2_lateness_cycles = 0;
  uint32_t ocxo2_zero_counter32 = 0;
};

bool clocks_alpha_smartzero_delay_snapshot(
    clocks_alpha_smartzero_delay_snapshot_t* out);

bool clocks_epoch_pending(void);
bool clocks_alpha_begin_smartzero_epoch(const char* reason);
bool clocks_alpha_zero_from_smartzero(const char* reason);
bool clocks_alpha_epoch_last_smartzero(interrupt_smartzero_snapshot_t* out);
bool clocks_alpha_installed_smartzero_valid(void);
uint32_t clocks_alpha_installed_smartzero_sequence(void);
bool clocks_alpha_installed_smartzero_backing_epoch(void);
bool clocks_alpha_epoch_install_in_progress(void);
uint32_t clocks_alpha_smartzero_install_attempt_count(void);
uint32_t clocks_alpha_smartzero_install_commit_count(void);
uint32_t clocks_alpha_smartzero_install_failure_count(void);
uint32_t clocks_alpha_smartzero_install_last_stage(void);
const char* clocks_alpha_smartzero_install_last_stage_name(void);
uint32_t clocks_alpha_smartzero_install_last_failure_stage(void);
const char* clocks_alpha_smartzero_install_last_failure_stage_name(void);
uint32_t clocks_alpha_smartzero_install_last_failure_code(void);
uint32_t clocks_alpha_smartzero_install_last_live_sequence(void);
uint32_t clocks_alpha_smartzero_install_last_prior_epoch_sequence(void);
uint32_t clocks_alpha_smartzero_install_last_committed_epoch_sequence(void);
uint32_t clocks_alpha_smartzero_install_last_committed_smartzero_sequence(void);
bool clocks_alpha_smartzero_install_last_success(void);
bool clocks_alpha_smartzero_install_last_atomic(void);
const char* clocks_alpha_smartzero_install_last_reason(void);
uint32_t clocks_alpha_smartzero_begin_count(void);
uint32_t clocks_alpha_smartzero_begin_failures(void);
bool clocks_alpha_smartzero_pending_active(void);
const char* clocks_alpha_smartzero_pending_reason(void);
void clocks_alpha_smartzero_pending_clear(void);
bool clocks_alpha_smartzero_last_begin_preserved_epoch(void);
uint32_t clocks_alpha_smartzero_last_begin_preserved_epoch_sequence(void);
uint32_t clocks_alpha_smartzero_begin_preserved_epoch_count(void);
uint32_t clocks_alpha_smartzero_begin_cold_count(void);
const char* clocks_alpha_smartzero_last_begin_reason(void);
bool clocks_alpha_zero_from_interrupt_capture(const char* reason);
bool clocks_alpha_epoch_initialized(void);
uint32_t clocks_alpha_epoch_sequence(void);
uint32_t clocks_alpha_epoch_install_count(void);
uint32_t clocks_alpha_epoch_install_failures(void);
uint32_t clocks_alpha_epoch_last_capture_sequence(void);
uint32_t clocks_alpha_epoch_last_capture_window_cycles(void);
bool clocks_alpha_epoch_last_vclock_capture_valid(void);
bool clocks_alpha_epoch_last_all_lanes_valid(void);
uint32_t clocks_alpha_epoch_last_dwt_at_edge(void);
uint32_t clocks_alpha_epoch_last_vclock_counter32(void);
uint32_t clocks_alpha_epoch_last_ocxo1_counter32(void);
uint32_t clocks_alpha_epoch_last_ocxo2_counter32(void);
bool clocks_alpha_epoch_last_vclock_zero_valid(void);
bool clocks_alpha_epoch_last_ocxo1_zero_valid(void);
bool clocks_alpha_epoch_last_ocxo2_zero_valid(void);
uint16_t clocks_alpha_epoch_last_vclock_hardware16_observed(void);
uint16_t clocks_alpha_epoch_last_vclock_hardware16_selected(void);
uint16_t clocks_alpha_epoch_last_ocxo1_hardware16(void);
uint16_t clocks_alpha_epoch_last_ocxo2_hardware16(void);
const char* clocks_alpha_epoch_last_reason(void);

// ============================================================================
// Campaign/accounting reset
// ============================================================================

// Beta-local campaign/statistical reset.  Alpha owns epoch installation; beta
// invokes this after CLOCKS.ZERO/START successfully installs logical origins.
void clocks_zero_all(void);
