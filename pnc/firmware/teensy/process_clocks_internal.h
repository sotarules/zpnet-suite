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
// Epoch authority:
//
//   PPS is a witness and selector.  process_interrupt observes the physical
//   PPS edge, selects the corresponding VCLOCK edge identity, and publishes
//   a PPS/VCLOCK snapshot whose DWT coordinate is authored by the VCLOCK/QTimer
//   event path.  This keeps public DWT facts in one VCLOCK coordinate species.
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
//   - g_dwt_at_pps_vclock             DWT coordinate of the selected
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
// from process_interrupt's canonical PPS/VCLOCK snapshots.  PPS selects and
// audits the relationship; the public DWT coordinate itself is authored by
// the VCLOCK/QTimer event path so the bridge, 1 kHz cadence samples, and
// one-second bookends share one coordinate species.
// ============================================================================

// Canonical VCLOCK/GNSS and measured OCXO 64-bit ledgers at the most recent
// selected PPS/VCLOCK edge. Alpha authors these; Beta publishes them;
// stateless time.h projection uses them as interpolation bases.
extern volatile uint64_t g_gnss_ns_at_pps_vclock;
extern volatile uint64_t g_ocxo1_measured_gnss_ns_at_pps_vclock;
extern volatile uint64_t g_ocxo2_measured_gnss_ns_at_pps_vclock;

// Canonical DWT_CYCCNT coordinate of the most recent selected PPS/VCLOCK epoch
// (snap.dwt_at_edge).  Under the VCLOCK-domain architecture this is the
// selected VCLOCK edge after the physical PPS pulse, not the raw GPIO ISR
// capture.
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

uint64_t clocks_dwt_cycles_at_dwt(uint32_t dwt32);

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

  uint64_t counter_nominal_ns_between_edges;
  uint64_t bridge_gnss_ns_between_edges;
  int64_t  bridge_residual_ns;
  bool     bridge_interval_valid;

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

  bool     regression_valid;
  uint32_t regression_sequence;
  uint32_t regression_sample_count;
  uint32_t regression_observed_dwt_at_event;
  uint32_t regression_inferred_dwt_at_event;
  int32_t  regression_inferred_minus_observed_cycles;
  uint32_t regression_target_counter32_at_event;
  uint16_t regression_target_hardware16_at_event;
  uint16_t regression_observed_hardware16_at_event;
  uint64_t regression_slope_q16_cycles_per_sample;
  int64_t  regression_slope_delta_q16_cycles_per_sample;
  int32_t  regression_fit_error_mean_q16_cycles;
  uint32_t regression_fit_error_stddev_q16_cycles;
  int32_t  regression_fit_error_min_cycles;
  int32_t  regression_fit_error_max_cycles;
  uint32_t regression_fit_error_gt_plus4_count;
  uint32_t regression_fit_error_lt_minus4_count;
  uint32_t regression_fit_error_abs_gt4_count;
};

bool clocks_alpha_lane_forensics(time_clock_id_t clock,
                                 clocks_alpha_lane_forensics_t* out);

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

  // Static rounded DAC authority.  The servo and Pi control plane may keep
  // a real-valued target for persistence/control math, but the AD5693R hardware
  // is always written to exactly one rounded integer code.
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
static constexpr uint16_t OCXO_DAC_SAFE_MAX_HW_CODE =
    (uint16_t)((OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE /
                OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE) *
                   65535.0 + 0.5);

static_assert(OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE >
              OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE,
              "OCXO DAC safety ceiling must be below full scale");
static_assert(OCXO_DAC_SAFE_MAX_HW_CODE < 65535,
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
  return (ocxo_dac_clamp_real_value(code) / 65535.0) *
         OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE;
}

bool ocxo_dac_set(ocxo_dac_state_t& s, double value);
bool ocxo_dac_set_desired(ocxo_dac_state_t& s, double value);
bool ocxo_dac_write_hw_code(ocxo_dac_state_t& s,
                            uint16_t hw_code,
                            bool latch_fault = true);
void ocxo_dac_predictor_reset(ocxo_dac_state_t& s);
void ocxo_dac_io_reset(ocxo_dac_state_t& s);
void ocxo_dac_retry_reset(ocxo_dac_state_t& s);

// ============================================================================
// Campaign warmup suppression
// ============================================================================
//
// A newly installed/recovered campaign deliberately suppresses the first N
// PPS-driven TIMEBASE_FRAGMENT publications. Alpha continues to measure and
// refine its internal timing state during this quiet period; beta simply
// refuses to call those records canonical campaign output.
//
// For START, public campaign identity begins after warmup: the first emitted
// fragment is teensy_pps_vclock_count=1 and gnss_ns=1e9.
//
// For RECOVER, the suppressed records are treated as real elapsed campaign
// seconds. The first emitted fragment therefore appears after a deliberate
// canonical gap, preserving the recovered absolute PPS identity.

static constexpr uint32_t CLOCKS_CAMPAIGN_WARMUP_SUPPRESS_PPS = 5;

// ============================================================================
// Campaign state
// ============================================================================

enum class clocks_campaign_state_t {
  STOPPED,
  STARTED
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

void clocks_beta_pps(void);
void clocks_watchdog_anomaly(const char* reason,
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