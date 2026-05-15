#pragma once

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.h -- Preferred TIME facade + live process_time backing-store contract
// ============================================================================
//
// process_time remains alive.  It owns the PPS/VCLOCK anchor and per-clock
// projection backing store consumed by CLOCKS, TimePop, and the facade below.
//
// The public conversion language is now the explicit "at" form:
//
//   time_<clock>_ns_at_dwt(authored_dwt)
//   time_dwt_at_<clock>_ns(clock_ns)
//
// Callers provide authored DWT coordinates.  TIME does not capture DWT for
// these conversions and does not apply latency correction; those facts must
// already have been authored by the interrupt/capture layer.
//
// Retired prediction/dynamic-CPS data shapes are retained only because
// process_time.cpp still defines inert compatibility stubs that return them.
// Their callable declarations are intentionally absent from this facade.
// ============================================================================

struct time_anchor_snapshot_t {
  uint32_t dwt_at_pps_vclock;
  uint32_t dwt_cycles_per_pps_vclock_s;
  uint32_t counter32_at_pps_vclock;
  uint32_t pps_vclock_count;
  bool     valid;
  bool     ok;

  // Legacy aliases used by TimePop and older callers.
  uint32_t dwt_at_pps;
  uint32_t dwt_cycles_per_s;
  uint32_t qtimer_at_pps;
  uint32_t pps_count;
};

// ============================================================================
// Retired compatibility data shapes
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

struct time_dynamic_cps_record_t {
  uint32_t pvc_sequence;
  uint32_t pvc_dwt_at_edge;
  uint32_t base_cycles;
  uint32_t final_cycles;
  int32_t  net_adjustment_cycles;
  uint32_t refine_ticks;
  uint32_t adjustments;

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

  uint32_t outlier_threshold_cycles;
  uint32_t accepted_samples;
  uint32_t substituted_samples;
  uint32_t first_substituted_ms;
  uint32_t last_substituted_ms;
  int64_t  substituted_error_sum_cycles;
  uint64_t substituted_abs_error_sum_cycles;
  uint32_t max_abs_observed_error_cycles;

  int32_t  endpoint_error_cycles;
  uint32_t abs_endpoint_error_cycles;
  uint32_t endpoint_sanity_tolerance_cycles;
  bool     endpoint_sanity_pass;

  bool     eval_valid;
  uint32_t next_actual_cycles;
  int32_t  static_error_cycles;
  int32_t  dynamic_error_cycles;
  uint32_t abs_static_error_cycles;
  uint32_t abs_dynamic_error_cycles;
  int32_t  improvement_cycles;
  bool     dynamic_helped;

  bool     valid;
};

struct time_dynamic_cps_snapshot_t {
  bool     valid;
  bool     witness_only;
  bool     projection_enabled;
  uint32_t pvc_sequence;
  uint32_t current_pvc_dwt_at_edge;
  uint32_t pvc_dwt_at_edge;
  uint32_t base_cycles;
  uint32_t current_cycles;
  int32_t  net_adjustment_cycles;
  uint32_t last_reseed_value;
  bool     last_reseed_was_computed;

  bool     phase_probe_valid;
  uint32_t phase_probe_pps_sequence;
  uint32_t phase_probe_pps_isr_entry_dwt_raw;
  uint32_t phase_probe_arm_dwt_raw;
  uint32_t phase_probe_vclock_isr_entry_dwt_raw;
  uint32_t phase_probe_pps_dwt_adjusted;
  uint32_t phase_probe_vclock_dwt_adjusted;
  uint32_t phase_probe_adjusted_difference_cycles;
  uint32_t phase_probe_phase_offset_cycles;

  uint32_t last_completed_dynamic_prediction_cycle_count;
  uint32_t last_completed_dynamic_prediction_adjust_count;
  uint32_t last_completed_dynamic_prediction_invalid_count;
  uint32_t last_completed_dynamic_prediction_valid_count;
  int32_t  last_completed_dynamic_prediction_adjust_cycles;

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

  int32_t  last_endpoint_error_cycles;
  uint32_t last_abs_endpoint_error_cycles;
  uint32_t endpoint_sanity_tolerance_cycles;
  bool     last_endpoint_sanity_pass;

  uint32_t history_count;
  uint32_t history_capacity;
};

static constexpr uint32_t TIME_PREDICTION_DETAIL_SAMPLE_COUNT = 10;

struct time_prediction_detail_sample_t {
  bool     populated = false;
  bool     endpoint = false;
  uint32_t sample_index = 0;
  uint32_t sample_ms = 0;

  uint32_t static_prediction_cycles = 0;
  uint32_t dynamic_prediction_cycles = 0;
  uint32_t dynamic_prediction_after_sample_cycles = 0;

  uint32_t static_prediction_thus_far_cycles = 0;
  uint32_t dynamic_prediction_thus_far_cycles = 0;
  int32_t  dynamic_minus_static_thus_far_cycles = 0;

  uint32_t actual_cycles_thus_far = 0;
  int32_t  residual_cycles = 0;
  uint32_t abs_residual_cycles = 0;
  uint32_t gate_threshold_cycles = 0;
  bool     accepted = false;
  bool     ignored = false;
  int32_t  correction_cycles = 0;
};

struct time_prediction_detail_snapshot_t {
  bool     valid = false;
  uint32_t pvc_sequence = 0;
  uint32_t anchor_dwt = 0;
  uint32_t static_prediction_cycles = 0;
  uint32_t dynamic_final_prediction_cycles = 0;
  uint32_t actual_cycles = 0;
  int32_t  static_residual_cycles = 0;
  int32_t  dynamic_residual_cycles = 0;
  uint32_t sample_count = 0;
  uint32_t sample_capacity = TIME_PREDICTION_DETAIL_SAMPLE_COUNT;
  time_prediction_detail_sample_t samples[TIME_PREDICTION_DETAIL_SAMPLE_COUNT] = {};
};

// ============================================================================
// Clock identity shared by CLOCKS and TIME
// ============================================================================

enum class time_clock_id_t : uint8_t {
  NONE   = 0,
  VCLOCK = 1,
  GNSS   = 1,
  OCXO1  = 2,
  OCXO2  = 3,
};

struct time_clock_snapshot_t {
  bool     valid = false;
  bool     prediction_valid = false;
  uint32_t dwt_at_update = 0;
  uint64_t ns_at_update = 0;
  uint32_t predicted_dwt_cycles_per_second = 0;
  uint32_t update_count = 0;
  uint32_t last_observed_dwt_cycles = 0;
  uint64_t last_observed_ns = 0;
  int32_t  last_prediction_residual_cycles = 0;
};

struct time_clock_projection_t {
  time_clock_id_t clock = time_clock_id_t::NONE;
  bool     valid = false;
  bool     basis_is_live = false;
  uint32_t dwt_at_update = 0;
  uint64_t ns_at_update = 0;
  uint32_t dwt_cycles_per_second = 0;
  uint32_t update_count = 0;
  uint32_t last_observed_dwt_cycles = 0;
  uint64_t last_observed_ns = 0;
  int32_t  last_prediction_residual_cycles = 0;
};

// ============================================================================
// Preferred conversion facade
// ============================================================================

bool time_clock_projection(time_clock_id_t clock,
                           time_clock_projection_t* out);

bool time_clock_ns_at_dwt(time_clock_id_t clock,
                          uint32_t authored_dwt_cycle_count,
                          uint64_t* out_ns);

bool time_dwt_at_clock_ns(time_clock_id_t clock,
                          uint64_t clock_ns,
                          uint32_t* out_dwt_cycle_count);

uint64_t time_vclock_ns_at_dwt(uint32_t authored_dwt_cycle_count);
uint32_t time_dwt_at_vclock_ns(uint64_t vclock_ns);

uint64_t time_gnss_ns_at_dwt(uint32_t authored_dwt_cycle_count);
uint32_t time_dwt_at_gnss_ns(uint64_t gnss_ns);

uint64_t time_ocxo1_ns_at_dwt(uint32_t authored_dwt_cycle_count);
uint32_t time_dwt_at_ocxo1_ns(uint64_t ocxo1_ns);

uint64_t time_ocxo2_ns_at_dwt(uint32_t authored_dwt_cycle_count);
uint32_t time_dwt_at_ocxo2_ns(uint64_t ocxo2_ns);

// ============================================================================
// Live process_time backing-store / anchor contract
// ============================================================================

int64_t time_gnss_ns_now(void);

time_anchor_snapshot_t time_anchor_snapshot(void);

void time_clock_reset_all(void);

bool time_clock_epoch_reset(time_clock_id_t clock,
                            uint32_t dwt_at_update,
                            uint64_t ns_at_update);

bool time_clock_update(time_clock_id_t clock,
                       uint32_t dwt_at_update,
                       uint64_t ns_at_update);

bool time_clock_snapshot(time_clock_id_t clock,
                         time_clock_snapshot_t* out);

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock);

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock);

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t counter32_at_pps_vclock,
                            uint32_t pps_vclock_count,
                            uint32_t dwt_cycles_per_pps_vclock_s);

bool     time_valid(void);
uint32_t time_pps_count(void);
void     time_init(void);
