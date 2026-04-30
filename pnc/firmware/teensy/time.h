#pragma once

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.h — VCLOCK-authored GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// Core doctrine:
//
//   PPS is a witness and selector. It tells process_interrupt which VCLOCK
//   10 MHz edge is the canonical second boundary.
//
//   PPS/VCLOCK is the selected VCLOCK edge. It is the sovereign time anchor.
//
//   DWT is the high-resolution local ruler used to interpolate between
//   successive PPS/VCLOCK anchors.
//
// process_time.cpp owns the anchor, interpolation state, DWT next-second
// prediction, and dynamic CPS refinement. This header remains the stable
// functional interface for ordinary consumers.
//
// This module performs NO latency adjustment. Every DWT value passed in or
// returned is already an event coordinate. Raw ISR-entry DWT values must be
// normalized by the ISR owner before reaching this interface.
//
// Returned GNSS nanoseconds are local zero-based campaign/epoch coordinates,
// not wall-clock UTC. The epoch may be restarted at boot, ZERO, START, or
// RECOVER boundaries.
// ============================================================================

struct time_anchor_snapshot_t {
  // Canonical fields — use these in new code.
  uint32_t dwt_at_pps_vclock;             // DWT coordinate of selected VCLOCK edge
  uint32_t dwt_cycles_per_pps_vclock_s;   // raw DWT cycles between selected VCLOCK edges
  uint32_t counter32_at_pps_vclock;       // synthetic VCLOCK counter32 at anchor
  uint32_t pps_vclock_count;              // selected VCLOCK second edges in epoch, 1-indexed
  bool     valid;                         // true once a real rate is available
  bool     ok;                            // true if snapshot was seqlock-consistent

  // Legacy aliases — kept for TimePop / older code during migration.
  uint32_t dwt_at_pps;
  uint32_t dwt_cycles_per_s;
  uint32_t qtimer_at_pps;
  uint32_t pps_count;
};

// ============================================================================
// DWT next-second prediction surface
// ============================================================================

struct time_dwt_prediction_record_t {
  uint32_t pps_vclock_count;
  uint32_t dwt_at_pps_vclock;
  uint32_t counter32_at_pps_vclock;
  uint32_t predicted_cycles;
  uint32_t actual_cycles;
  int32_t  residual_cycles;
  bool     valid;
};

struct time_dwt_prediction_snapshot_t {
  bool     valid;
  uint32_t pps_vclock_count;
  uint32_t predicted_cycles_last;
  uint32_t actual_cycles_last;
  int32_t  residual_cycles_last;
  uint32_t predicted_cycles_next;
  uint32_t history_count;
  uint32_t history_capacity;
};

// ============================================================================
// Dynamic CPS refinement surface
// ============================================================================
//
// Dynamic CPS starts each PPS/VCLOCK interval with the raw bookend DWT delta
// and then refines an experimental cycles-per-second estimate from the 1 kHz
// VCLOCK cadence. It is currently WITNESS-ONLY: measured, reported, and
// evaluated, but not used as the operational interpolation/projection rate.
// DWT<->GNSS projection continues to use the static PPS/VCLOCK bookend count
// until the dynamic model proves it improves prediction error.

struct time_dynamic_cps_record_t {
  uint32_t pvc_sequence;
  uint32_t pvc_dwt_at_edge;
  uint32_t base_cycles;
  uint32_t final_cycles;
  int32_t  net_adjustment_cycles;
  uint32_t refine_ticks;
  uint32_t adjustments;

  // Fixed-anchor line-fit audit surface. The PPS/VCLOCK DWT edge is the
  // immutable t=0 anchor. Cadence samples begin at t=1 ms.
  uint32_t fit_samples;
  uint64_t fit_sum_t2_ms2;
  uint64_t fit_sum_tx_cycles_ms;
  uint32_t last_sample_ms;
  uint32_t last_observed_cycles;
  uint32_t last_used_cycles;
  uint32_t last_expected_static_cycles;
  uint32_t last_expected_fit_cycles;
  int32_t  last_observed_error_cycles;
  int32_t  last_used_error_cycles;
  int32_t  last_static_path_error_cycles;
  int32_t  last_fit_error_cycles;
  uint32_t last_point_slope_cycles;

  // Outlier/substitution audit. Suspicious cadence observations are not
  // allowed to bend the line; the model value is substituted and the raw
  // error is retained for diagnostics.
  uint32_t outlier_threshold_cycles;
  uint32_t accepted_samples;
  uint32_t substituted_samples;
  uint32_t first_substituted_ms;
  uint32_t last_substituted_ms;
  int64_t  substituted_error_sum_cycles;
  uint64_t substituted_abs_error_sum_cycles;
  uint32_t max_abs_observed_error_cycles;

  // Endpoint sanity: evaluated when the next PPS/VCLOCK bookend arrives.
  int32_t  endpoint_error_cycles;       // next_actual - final_cycles
  uint32_t abs_endpoint_error_cycles;
  uint32_t endpoint_sanity_tolerance_cycles;
  bool     endpoint_sanity_pass;

  // Effectiveness evaluation. A record for anchor sequence S is evaluated
  // when the next PPS/VCLOCK edge arrives and the actual DWT cycles for
  // interval S->S+1 become known.
  bool     eval_valid;
  uint32_t next_actual_cycles;
  int32_t  static_error_cycles;     // next_actual - base_cycles
  int32_t  dynamic_error_cycles;    // next_actual - final_cycles
  uint32_t abs_static_error_cycles;
  uint32_t abs_dynamic_error_cycles;
  int32_t  improvement_cycles;      // abs_static_error - abs_dynamic_error
  bool     dynamic_helped;

  bool     valid;
};

struct time_dynamic_cps_snapshot_t {
  bool     valid;
  bool     witness_only;          // true while dynamic CPS is report-only
  bool     projection_enabled;    // false: projection uses static bookend CPS
  uint32_t pvc_sequence;
  uint32_t current_pvc_dwt_at_edge;
  uint32_t pvc_dwt_at_edge;               // legacy alias for current_pvc_dwt_at_edge
  uint32_t base_cycles;
  uint32_t current_cycles;
  int32_t  net_adjustment_cycles;
  uint32_t last_reseed_value;
  bool     last_reseed_was_computed;

  // Fixed-anchor line-fit state for the current second.
  uint32_t fit_samples_this_second;
  uint64_t fit_sum_t2_ms2;
  uint64_t fit_sum_tx_cycles_ms;
  uint32_t last_sample_ms;
  uint32_t last_observed_cycles;
  uint32_t last_used_cycles;
  uint32_t last_expected_static_cycles;
  uint32_t last_expected_fit_cycles;
  int32_t  last_observed_error_cycles;
  int32_t  last_used_error_cycles;
  int32_t  last_static_path_error_cycles;
  int32_t  last_fit_error_cycles;
  uint32_t last_point_slope_cycles;

  // Outlier/substitution state for the current second and lifetime totals.
  uint32_t outlier_threshold_cycles;
  uint32_t accepted_samples_this_second;
  uint32_t substituted_samples_this_second;
  uint32_t total_accepted_samples;
  uint32_t total_substituted_samples;
  uint32_t first_substituted_ms;
  uint32_t last_substituted_ms;
  int64_t  substituted_error_sum_cycles;
  uint64_t substituted_abs_error_sum_cycles;
  uint32_t max_abs_observed_error_cycles;

  uint32_t refine_ticks_this_second;
  uint32_t adjustments_this_second;
  uint32_t total_refine_ticks;
  uint32_t total_adjustments;
  uint32_t skipped_no_anchor;
  uint32_t skipped_no_cps;
  uint32_t skipped_offset_low;
  uint32_t skipped_offset_high;

  // Dynamic-vs-static effectiveness summary.
  uint32_t eval_count;
  uint32_t helped_count;
  uint32_t hurt_count;
  uint32_t neutral_count;
  uint64_t total_abs_static_error_cycles;
  uint64_t total_abs_dynamic_error_cycles;
  uint32_t last_next_actual_cycles;
  int32_t  last_static_error_cycles;
  int32_t  last_dynamic_error_cycles;
  uint32_t last_abs_static_error_cycles;
  uint32_t last_abs_dynamic_error_cycles;
  int32_t  last_improvement_cycles;
  bool     last_dynamic_helped;

  // Last endpoint sanity result.
  int32_t  last_endpoint_error_cycles;
  uint32_t last_abs_endpoint_error_cycles;
  uint32_t endpoint_sanity_tolerance_cycles;
  bool     last_endpoint_sanity_pass;

  uint32_t history_count;
  uint32_t history_capacity;
};

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void);

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt);

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps_vclock,
                            uint32_t anchor_dwt_cycles_per_pps_vclock_s,
                            uint32_t anchor_pps_vclock_count);

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns);

// ============================================================================
// Anchor snapshot — seqlock-safe copy of the full PPS/VCLOCK anchor
// ============================================================================

time_anchor_snapshot_t time_anchor_snapshot(void);

// ============================================================================
// DWT prediction accessors
// ============================================================================

time_dwt_prediction_snapshot_t time_dwt_prediction_snapshot(void);

// Copies up to max_records recent records into out_records, oldest-to-newest.
// Returns the number of records copied. Passing nullptr or max_records=0 is
// valid and returns 0.
uint32_t time_dwt_prediction_history(time_dwt_prediction_record_t* out_records,
                                     uint32_t max_records);

bool     time_dwt_prediction_valid(void);
uint32_t time_dwt_actual_cycles_last_second(void);
uint32_t time_dwt_predicted_cycles_last_second(void);
uint32_t time_dwt_next_prediction_cycles(void);
int32_t  time_dwt_prediction_residual_cycles(void);

// ============================================================================
// Dynamic CPS accessors / update hooks
// ============================================================================

time_dynamic_cps_snapshot_t time_dynamic_cps_snapshot(void);
uint32_t time_dynamic_cps_history(time_dynamic_cps_record_t* out_records,
                                  uint32_t max_records);

void     time_dynamic_cps_reset(void);
void     time_dynamic_cps_pps_vclock_edge(uint32_t pvc_sequence,
                                           uint32_t pvc_dwt_at_edge);
void     time_dynamic_cps_cadence_update(uint32_t qtimer_event_dwt,
                                          uint32_t cadence_tick_mod_1000);

bool     time_dynamic_cps_valid(void);
uint32_t time_dynamic_cps_current_cycles(void);
uint32_t time_dynamic_cps_current(void);  // legacy alias
// Per-anchor CPS lookup used by existing projection code. While dynamic CPS
// is witness-only, this returns the static PPS/VCLOCK bookend CPS for the
// requested anchor rather than the dynamic final/current CPS.
bool     time_dynamic_cps_cycles_for_anchor(uint32_t pvc_sequence,
                                            uint32_t* out_cycles);
bool     time_dynamic_cps_for_pvc_sequence(uint32_t pvc_sequence,
                                            uint32_t* out_cycles); // legacy alias

// ============================================================================
// Status
// ============================================================================

uint32_t time_pps_vclock_count(void);

// Legacy alias: returns time_pps_vclock_count().
uint32_t time_pps_count(void);

bool time_valid(void);

// ============================================================================
// Update interface (called by process_clocks_alpha.cpp only)
// ============================================================================

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock);

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock);

// Legacy wrappers retained during migration.
void time_pps_update(uint32_t dwt_at_pps,
                     uint32_t dwt_cycles_per_s,
                     uint32_t qtimer_at_pps);

void time_pps_epoch_reset(uint32_t dwt_at_pps,
                          uint32_t qtimer_at_pps);

// ============================================================================
// Initialization
// ============================================================================

void time_init(void);
