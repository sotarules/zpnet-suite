// ============================================================================
// process_time.cpp — VCLOCK-authored GNSS Nanosecond Interface + Process
// ============================================================================
//
// PPS is not the timebase here. PPS selects a VCLOCK edge; the selected
// PPS/VCLOCK edge becomes the canonical anchor. DWT is then used only as a
// high-resolution ruler to interpolate from that selected VCLOCK edge.
//
// This module performs NO latency adjustment. All DWT inputs are already
// event-coordinate values.
//
// process_time owns:
//   • PPS/VCLOCK anchor state for DWT↔GNSS interpolation
//   • DWT next-second prediction state
//   • dynamic CPS refinement state
//   • command/report surfaces for compact state and bounded history reports
// ============================================================================

#include "process_time.h"
#include "time.h"
#include "config.h"
#include "process.h"
#include "payload.h"
#include "timepop.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;
static constexpr uint32_t PREDICTION_HISTORY_CAPACITY = 32;
static constexpr uint32_t DYNAMIC_CPS_HISTORY_CAPACITY = 32;
static constexpr uint32_t HISTORY_DEFAULT_LIMIT = 16;

static constexpr uint32_t DYNAMIC_CPS_MIN_REFINE_MS = 1;
static constexpr uint32_t DYNAMIC_CPS_MAX_REFINE_MS = 999;
static constexpr uint64_t DYNAMIC_CPS_TIMEPOP_PERIOD_NS = NS_PER_MILLISECOND;
static constexpr const char* DYNAMIC_CPS_TIMEPOP_NAME = "TIME_DYNAMIC_CPS";
static constexpr uint32_t DYNAMIC_CPS_OUTLIER_THRESHOLD_CYCLES = 5;
// Servo gate: observations whose dynamic residual exceeds this threshold are
// treated as impossible ISR-late/capture outliers. They are reported, but the
// servo leaves the one-second CPS prediction unchanged.
static constexpr uint32_t DYNAMIC_CPS_SERVO_GATE_THRESHOLD_CYCLES = 100;
static constexpr uint32_t DYNAMIC_CPS_ENDPOINT_SANITY_TOLERANCE_CYCLES = 25;
static constexpr uint32_t DYNAMIC_CPS_FIRST_MS_AUDIT_START_MS = 100;
static constexpr uint32_t DYNAMIC_CPS_FIRST_MS_AUDIT_STEP_MS = 100;
static constexpr uint32_t DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT = 9;
static constexpr uint32_t DYNAMIC_CPS_FIRST_MS_AUDIT_END_MS =
    DYNAMIC_CPS_FIRST_MS_AUDIT_START_MS +
    (DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT - 1U) * DYNAMIC_CPS_FIRST_MS_AUDIT_STEP_MS;

// ============================================================================
// Anchor state (written by clocks alpha, read by time readers)
// ============================================================================

struct time_anchor_t {
  volatile uint32_t seq;
  volatile uint32_t dwt_at_pps_vclock;
  volatile uint32_t dwt_cycles_per_pps_vclock_s;
  volatile uint32_t counter32_at_pps_vclock;
  volatile uint32_t pps_vclock_count;
  volatile bool     valid;
};

static time_anchor_t anchor = {};

static inline void dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static inline uint32_t abs_i32_to_u32(int32_t v) {
  return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

// ============================================================================
// Prediction state
// ============================================================================

struct prediction_state_t {
  volatile uint32_t seq;

  bool     valid = false;
  uint32_t pps_vclock_count = 0;
  uint32_t predicted_cycles_last = 0;
  uint32_t actual_cycles_last = 0;
  int32_t  residual_cycles_last = 0;
  uint32_t predicted_cycles_next = 0;

  uint32_t history_head = 0;   // next write slot
  uint32_t history_count = 0;
  time_dwt_prediction_record_t history[PREDICTION_HISTORY_CAPACITY] = {};
};

static prediction_state_t prediction = {};

static void prediction_reset(void) {
  prediction.seq++;
  dmb();

  prediction.valid = false;
  prediction.pps_vclock_count = 0;
  prediction.predicted_cycles_last = 0;
  prediction.actual_cycles_last = 0;
  prediction.residual_cycles_last = 0;
  prediction.predicted_cycles_next = 0;
  prediction.history_head = 0;
  prediction.history_count = 0;
  for (uint32_t i = 0; i < PREDICTION_HISTORY_CAPACITY; i++) {
    prediction.history[i] = time_dwt_prediction_record_t{};
  }

  dmb();
  prediction.seq++;
}

static void prediction_push_record(const time_dwt_prediction_record_t& rec) {
  prediction.history[prediction.history_head] = rec;
  prediction.history_head = (prediction.history_head + 1U) % PREDICTION_HISTORY_CAPACITY;
  if (prediction.history_count < PREDICTION_HISTORY_CAPACITY) {
    prediction.history_count++;
  }
}

static void prediction_observe_actual(uint32_t pps_vclock_count,
                                      uint32_t dwt_at_pps_vclock,
                                      uint32_t counter32_at_pps_vclock,
                                      uint32_t actual_cycles) {
  if (actual_cycles == 0) return;

  prediction.seq++;
  dmb();

  const bool had_prediction = prediction.predicted_cycles_next != 0;
  const uint32_t predicted_for_this_second = prediction.predicted_cycles_next;
  const int32_t residual = had_prediction
      ? (int32_t)((int64_t)actual_cycles - (int64_t)predicted_for_this_second)
      : 0;

  prediction.pps_vclock_count = pps_vclock_count;
  prediction.predicted_cycles_last = predicted_for_this_second;
  prediction.actual_cycles_last = actual_cycles;
  prediction.residual_cycles_last = residual;
  prediction.predicted_cycles_next = actual_cycles;  // random-walk predictor
  prediction.valid = had_prediction;

  time_dwt_prediction_record_t rec{};
  rec.pps_vclock_count = pps_vclock_count;
  rec.dwt_at_pps_vclock = dwt_at_pps_vclock;
  rec.counter32_at_pps_vclock = counter32_at_pps_vclock;
  rec.predicted_cycles = predicted_for_this_second;
  rec.actual_cycles = actual_cycles;
  rec.residual_cycles = residual;
  rec.valid = had_prediction;
  prediction_push_record(rec);

  dmb();
  prediction.seq++;
}

// ============================================================================
// Dynamic CPS fixed-anchor witness state
// ============================================================================
//
// Dynamic CPS now drives operational DWT<->GNSS projection for the current
// PPS/VCLOCK anchor.  The model is a fixed-anchor TimePop client servo:
//
//   t = 0       -> PPS/VCLOCK anchor DWT
//   t = event   -> TimePop-authored fire_vclock_raw + fire_dwt_cyccnt
//
// Each TimePop observation compares the actual DWT elapsed cycles against the
// elapsed cycles predicted by the current one-second CPS estimate at that
// exact VCLOCK counter position.  The full observed error is added directly
// to the current CPS estimate when the sample passes the servo gate.  This is
// a servo, not a line fit, and it no longer depends on a private QTimer CH3
// cadence rail.

static inline uint32_t u32_abs_i32(int32_t v) {
  return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

static inline uint32_t line_cycles_at_ms(uint32_t cycles_per_second,
                                         uint32_t ms) {
  return (uint32_t)(((uint64_t)cycles_per_second * (uint64_t)ms + 500ULL) / 1000ULL);
}

static inline uint32_t line_cycles_at_ticks(uint32_t cycles_per_second,
                                            uint32_t vclock_ticks) {
  return (uint32_t)(((uint64_t)cycles_per_second * (uint64_t)vclock_ticks +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static inline uint32_t line_slope_from_sums(uint64_t sum_t2_ms2,
                                            uint64_t sum_tx_cycles_ms,
                                            uint32_t fallback_cycles) {
  if (sum_t2_ms2 == 0) return fallback_cycles;
  return (uint32_t)(((uint64_t)sum_tx_cycles_ms * 1000ULL + sum_t2_ms2 / 2ULL) /
                    sum_t2_ms2);
}

static inline bool audit_checkpoint_index(uint32_t sample_ms,
                                          uint32_t* out_idx) {
  if (sample_ms < DYNAMIC_CPS_FIRST_MS_AUDIT_START_MS) return false;
  if (sample_ms > DYNAMIC_CPS_FIRST_MS_AUDIT_END_MS) return false;

  const uint32_t offset = sample_ms - DYNAMIC_CPS_FIRST_MS_AUDIT_START_MS;
  if ((offset % DYNAMIC_CPS_FIRST_MS_AUDIT_STEP_MS) != 0U) return false;

  const uint32_t idx = offset / DYNAMIC_CPS_FIRST_MS_AUDIT_STEP_MS;
  if (idx >= DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT) return false;

  if (out_idx) *out_idx = idx;
  return true;
}

struct fixed_anchor_line_fit_t {
  uint64_t sum_t2_ms2 = 0;
  uint64_t sum_tx_cycles_ms = 0;
  uint32_t samples = 0;

  void reset() {
    sum_t2_ms2 = 0;
    sum_tx_cycles_ms = 0;
    samples = 0;
  }

  void add(uint32_t ms, uint32_t used_cycles) {
    sum_t2_ms2 += (uint64_t)ms * (uint64_t)ms;
    sum_tx_cycles_ms += (uint64_t)ms * (uint64_t)used_cycles;
    samples++;
  }

  uint32_t slope(uint32_t fallback_cycles) const {
    return line_slope_from_sums(sum_t2_ms2, sum_tx_cycles_ms, fallback_cycles);
  }
};

struct dynamic_cps_first_ms_audit_t {
  bool     valid = false;
  uint32_t ms = 0;
  uint32_t anchor_dwt = 0;

  // Static path: the PPS/VCLOCK bookend CPS for this second. This is the
  // conservative ruler used by the operational projection path today.
  uint32_t static_cycles_per_second = 0;
  uint32_t static_cycles_per_ms_floor = 0;
  uint32_t expected_elapsed_floor = 0;          // legacy static/floor name
  uint32_t expected_elapsed_round = 0;          // legacy static/round name
  uint32_t expected_dwt_floor = 0;              // legacy static/floor name
  uint32_t expected_dwt_round = 0;              // legacy static/round name
  uint32_t expected_elapsed_static_floor = 0;
  uint32_t expected_elapsed_static_round = 0;
  uint32_t expected_dwt_static_floor = 0;
  uint32_t expected_dwt_static_round = 0;
  int32_t  error_vs_floor_cycles = 0;           // legacy static/floor name
  int32_t  error_vs_round_cycles = 0;           // legacy static/round name
  int32_t  error_vs_static_floor_cycles = 0;
  int32_t  error_vs_static_round_cycles = 0;

  // Dynamic path: the currently refined CPS before this cadence observation is
  // consumed.  This makes the report a predictive comparison instead of a
  // post-sample tautology.
  uint32_t dynamic_cycles_per_second = 0;
  uint32_t expected_elapsed_dynamic_round = 0;
  uint32_t expected_dwt_dynamic_round = 0;
  int32_t  error_vs_dynamic_round_cycles = 0;

  // Servo gate audit. Impossible residuals are preserved as raw observation
  // facts, but the servo does not learn from them.
  uint32_t servo_gate_threshold_cycles = 0;
  uint32_t servo_abs_error_cycles = 0;
  bool     servo_gated = false;
  int32_t  servo_correction_cycles = 0;
  uint32_t interval_used_cycles = 0;
  uint32_t dynamic_cycles_after_sample = 0;

  uint32_t observed_dwt = 0;
  uint32_t observed_elapsed_cycles = 0;
};

struct dynamic_cps_state_t {
  volatile uint32_t seq;

  bool     valid = false;
  uint32_t pvc_sequence = 0;
  uint32_t current_pvc_dwt_at_edge = 0;

  // PPS→VCLOCK local phase probe, authored by process_interrupt every PPS
  // using QTimer1 CH1.  Raw ISR-entry DWT values are retained here solely as
  // diagnostic facts; the report carries only the captured facts and the
  // latency-adjusted modulo-100 phase offset.
  bool     phase_probe_valid = false;
  uint32_t phase_probe_pps_sequence = 0;
  uint32_t phase_probe_pps_isr_entry_dwt_raw = 0;
  uint32_t phase_probe_arm_dwt_raw = 0;
  uint32_t phase_probe_vclock_isr_entry_dwt_raw = 0;
  uint32_t phase_probe_pps_dwt_adjusted = 0;
  uint32_t phase_probe_vclock_dwt_adjusted = 0;
  uint32_t phase_probe_adjusted_difference_cycles = 0;
  uint32_t phase_probe_phase_offset_cycles = 0;

  dynamic_cps_first_ms_audit_t first_ms_audit[DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT] = {};
  uint32_t first_ms_audit_valid_count = 0;
  uint32_t base_cycles = 0;
  uint32_t current_cycles = 0;
  uint32_t last_reseed_value = 0;
  bool     last_reseed_was_computed = false;

  // Finalized facts from the previous PPS/VCLOCK interval. These are captured
  // before resetting the current-second servo state and are intended for
  // TIMEBASE_FRAGMENT publication by CLOCKS beta.
  uint32_t last_completed_dynamic_prediction_cycle_count = 0;
  uint32_t last_completed_dynamic_prediction_adjust_count = 0;
  uint32_t last_completed_dynamic_prediction_invalid_count = 0;
  uint32_t last_completed_dynamic_prediction_valid_count = 0;
  int32_t  last_completed_dynamic_prediction_adjust_cycles = 0;

  bool     timepop_client_armed = false;
  uint32_t timepop_arm_count = 0;
  uint32_t timepop_arm_failures = 0;
  uint32_t timepop_callback_count = 0;
  uint32_t timepop_last_handle = 0;
  uint32_t timepop_last_deadline = 0;
  uint32_t timepop_last_fire_vclock_raw = 0;
  uint32_t timepop_last_fire_dwt = 0;
  int64_t  timepop_last_fire_gnss_ns = -1;

  fixed_anchor_line_fit_t fit;

  uint32_t last_sample_ms = 0;
  uint32_t last_observed_cycles = 0;
  uint32_t last_used_cycles = 0;
  uint32_t last_expected_static_cycles = 0;
  uint32_t last_expected_fit_cycles = 0;
  int32_t  last_observed_error_cycles = 0;
  int32_t  last_used_error_cycles = 0;
  int32_t  last_static_path_error_cycles = 0;
  int32_t  last_fit_error_cycles = 0;
  uint32_t last_point_slope_cycles = 0;

  uint32_t accepted_samples_this_second = 0;
  uint32_t substituted_samples_this_second = 0;
  uint32_t total_accepted_samples = 0;
  uint32_t total_substituted_samples = 0;
  uint32_t first_substituted_ms = 0;
  uint32_t last_substituted_ms = 0;
  int64_t  substituted_error_sum_cycles = 0;
  uint64_t substituted_abs_error_sum_cycles = 0;
  uint32_t max_abs_observed_error_cycles = 0;

  uint32_t refine_ticks_this_second = 0;
  uint32_t adjustments_this_second = 0;
  uint32_t total_refine_ticks = 0;
  uint32_t total_adjustments = 0;
  uint32_t skipped_no_anchor = 0;
  uint32_t skipped_no_cps = 0;
  uint32_t skipped_offset_low = 0;
  uint32_t skipped_offset_high = 0;

  // Dynamic-vs-static effectiveness evaluation.
  uint32_t eval_count = 0;
  uint32_t helped_count = 0;
  uint32_t hurt_count = 0;
  uint32_t neutral_count = 0;
  uint64_t total_abs_static_error_cycles = 0;
  uint64_t total_abs_dynamic_error_cycles = 0;
  uint32_t last_next_actual_cycles = 0;
  int32_t  last_static_error_cycles = 0;
  int32_t  last_dynamic_error_cycles = 0;
  uint32_t last_abs_static_error_cycles = 0;
  uint32_t last_abs_dynamic_error_cycles = 0;
  int32_t  last_improvement_cycles = 0;
  bool     last_dynamic_helped = false;
  int32_t  last_endpoint_error_cycles = 0;
  uint32_t last_abs_endpoint_error_cycles = 0;
  bool     last_endpoint_sanity_pass = false;

  uint32_t history_head = 0;
  uint32_t history_count = 0;
  time_dynamic_cps_record_t history[DYNAMIC_CPS_HISTORY_CAPACITY] = {};
};

static dynamic_cps_state_t dynamic_cps = {};

static bool dynamic_cps_ensure_timepop_client(void);
static void dynamic_cps_timepop_callback(timepop_ctx_t* ctx,
                                         timepop_diag_t* diag,
                                         void* user_data);

static void dynamic_cps_note_effectiveness_locked(const time_dynamic_cps_record_t& rec) {
  if (!rec.eval_valid) return;

  dynamic_cps.eval_count++;
  dynamic_cps.total_abs_static_error_cycles += rec.abs_static_error_cycles;
  dynamic_cps.total_abs_dynamic_error_cycles += rec.abs_dynamic_error_cycles;

  if (rec.abs_dynamic_error_cycles < rec.abs_static_error_cycles) {
    dynamic_cps.helped_count++;
  } else if (rec.abs_dynamic_error_cycles > rec.abs_static_error_cycles) {
    dynamic_cps.hurt_count++;
  } else {
    dynamic_cps.neutral_count++;
  }

  dynamic_cps.last_next_actual_cycles = rec.next_actual_cycles;
  dynamic_cps.last_static_error_cycles = rec.static_error_cycles;
  dynamic_cps.last_dynamic_error_cycles = rec.dynamic_error_cycles;
  dynamic_cps.last_abs_static_error_cycles = rec.abs_static_error_cycles;
  dynamic_cps.last_abs_dynamic_error_cycles = rec.abs_dynamic_error_cycles;
  dynamic_cps.last_improvement_cycles = rec.improvement_cycles;
  dynamic_cps.last_dynamic_helped = rec.dynamic_helped;
  dynamic_cps.last_endpoint_error_cycles = rec.endpoint_error_cycles;
  dynamic_cps.last_abs_endpoint_error_cycles = rec.abs_endpoint_error_cycles;
  dynamic_cps.last_endpoint_sanity_pass = rec.endpoint_sanity_pass;
}

static void dynamic_cps_push_record_locked(uint32_t next_actual_cycles) {
  if (dynamic_cps.pvc_sequence == 0) return;

  time_dynamic_cps_record_t rec{};
  rec.pvc_sequence = dynamic_cps.pvc_sequence;
  rec.pvc_dwt_at_edge = dynamic_cps.current_pvc_dwt_at_edge;
  rec.base_cycles = dynamic_cps.base_cycles;
  rec.final_cycles = dynamic_cps.current_cycles;
  rec.net_adjustment_cycles =
      (int32_t)((int64_t)dynamic_cps.current_cycles - (int64_t)dynamic_cps.base_cycles);
  rec.refine_ticks = dynamic_cps.refine_ticks_this_second;
  rec.adjustments = dynamic_cps.adjustments_this_second;

  rec.fit_samples = dynamic_cps.fit.samples;
  rec.fit_sum_t2_ms2 = dynamic_cps.fit.sum_t2_ms2;
  rec.fit_sum_tx_cycles_ms = dynamic_cps.fit.sum_tx_cycles_ms;
  rec.last_sample_ms = dynamic_cps.last_sample_ms;
  rec.last_observed_cycles = dynamic_cps.last_observed_cycles;
  rec.last_used_cycles = dynamic_cps.last_used_cycles;
  rec.last_expected_static_cycles = dynamic_cps.last_expected_static_cycles;
  rec.last_expected_fit_cycles = dynamic_cps.last_expected_fit_cycles;
  rec.last_observed_error_cycles = dynamic_cps.last_observed_error_cycles;
  rec.last_used_error_cycles = dynamic_cps.last_used_error_cycles;
  rec.last_static_path_error_cycles = dynamic_cps.last_static_path_error_cycles;
  rec.last_fit_error_cycles = dynamic_cps.last_fit_error_cycles;
  rec.last_point_slope_cycles = dynamic_cps.last_point_slope_cycles;

  rec.outlier_threshold_cycles = DYNAMIC_CPS_SERVO_GATE_THRESHOLD_CYCLES;
  rec.accepted_samples = dynamic_cps.accepted_samples_this_second;
  rec.substituted_samples = dynamic_cps.substituted_samples_this_second;
  rec.first_substituted_ms = dynamic_cps.first_substituted_ms;
  rec.last_substituted_ms = dynamic_cps.last_substituted_ms;
  rec.substituted_error_sum_cycles = dynamic_cps.substituted_error_sum_cycles;
  rec.substituted_abs_error_sum_cycles = dynamic_cps.substituted_abs_error_sum_cycles;
  rec.max_abs_observed_error_cycles = dynamic_cps.max_abs_observed_error_cycles;

  rec.endpoint_sanity_tolerance_cycles = DYNAMIC_CPS_ENDPOINT_SANITY_TOLERANCE_CYCLES;
  rec.valid = dynamic_cps.valid && dynamic_cps.base_cycles != 0 && dynamic_cps.current_cycles != 0;

  rec.next_actual_cycles = next_actual_cycles;
  rec.eval_valid = rec.valid && next_actual_cycles != 0;
  if (rec.eval_valid) {
    rec.static_error_cycles =
        (int32_t)((int64_t)next_actual_cycles - (int64_t)dynamic_cps.base_cycles);
    rec.dynamic_error_cycles =
        (int32_t)((int64_t)next_actual_cycles - (int64_t)dynamic_cps.current_cycles);
    rec.abs_static_error_cycles = u32_abs_i32(rec.static_error_cycles);
    rec.abs_dynamic_error_cycles = u32_abs_i32(rec.dynamic_error_cycles);
    rec.improvement_cycles =
        (int32_t)((int64_t)rec.abs_static_error_cycles -
                  (int64_t)rec.abs_dynamic_error_cycles);
    rec.dynamic_helped = rec.abs_dynamic_error_cycles < rec.abs_static_error_cycles;

    rec.endpoint_error_cycles = rec.dynamic_error_cycles;
    rec.abs_endpoint_error_cycles = rec.abs_dynamic_error_cycles;
    rec.endpoint_sanity_pass =
        rec.abs_endpoint_error_cycles <= DYNAMIC_CPS_ENDPOINT_SANITY_TOLERANCE_CYCLES;

    dynamic_cps_note_effectiveness_locked(rec);
  }

  dynamic_cps.last_completed_dynamic_prediction_cycle_count = rec.valid ? rec.final_cycles : 0;
  dynamic_cps.last_completed_dynamic_prediction_adjust_count = rec.valid ? rec.adjustments : 0;
  dynamic_cps.last_completed_dynamic_prediction_invalid_count = rec.valid ? rec.substituted_samples : 0;
  dynamic_cps.last_completed_dynamic_prediction_valid_count = rec.valid ? rec.accepted_samples : 0;
  dynamic_cps.last_completed_dynamic_prediction_adjust_cycles = rec.valid ? rec.net_adjustment_cycles : 0;

  dynamic_cps.history[dynamic_cps.history_head] = rec;
  dynamic_cps.history_head = (dynamic_cps.history_head + 1U) % DYNAMIC_CPS_HISTORY_CAPACITY;
  if (dynamic_cps.history_count < DYNAMIC_CPS_HISTORY_CAPACITY) {
    dynamic_cps.history_count++;
  }
}

static void dynamic_cps_reset_first_ms_audit_locked(void) {
  for (uint32_t i = 0; i < DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT; i++) {
    dynamic_cps.first_ms_audit[i] = dynamic_cps_first_ms_audit_t{};
  }
  dynamic_cps.first_ms_audit_valid_count = 0;
}

static void dynamic_cps_reset_second_locked(void) {
  dynamic_cps.fit.reset();
  dynamic_cps_reset_first_ms_audit_locked();
  dynamic_cps.last_sample_ms = 0;
  dynamic_cps.last_observed_cycles = 0;
  dynamic_cps.last_used_cycles = 0;
  dynamic_cps.last_expected_static_cycles = 0;
  dynamic_cps.last_expected_fit_cycles = 0;
  dynamic_cps.last_observed_error_cycles = 0;
  dynamic_cps.last_used_error_cycles = 0;
  dynamic_cps.last_static_path_error_cycles = 0;
  dynamic_cps.last_fit_error_cycles = 0;
  dynamic_cps.last_point_slope_cycles = 0;
  dynamic_cps.accepted_samples_this_second = 0;
  dynamic_cps.substituted_samples_this_second = 0;
  dynamic_cps.first_substituted_ms = 0;
  dynamic_cps.last_substituted_ms = 0;
  dynamic_cps.substituted_error_sum_cycles = 0;
  dynamic_cps.substituted_abs_error_sum_cycles = 0;
  dynamic_cps.max_abs_observed_error_cycles = 0;
  dynamic_cps.refine_ticks_this_second = 0;
  dynamic_cps.adjustments_this_second = 0;
}

void time_dynamic_cps_reset(void) {
  dynamic_cps.seq++;
  dmb();

  dynamic_cps.valid = false;
  dynamic_cps.pvc_sequence = 0;
  dynamic_cps.current_pvc_dwt_at_edge = 0;
  dynamic_cps.phase_probe_valid = false;
  dynamic_cps.phase_probe_pps_sequence = 0;
  dynamic_cps.phase_probe_pps_isr_entry_dwt_raw = 0;
  dynamic_cps.phase_probe_arm_dwt_raw = 0;
  dynamic_cps.phase_probe_vclock_isr_entry_dwt_raw = 0;
  dynamic_cps.phase_probe_pps_dwt_adjusted = 0;
  dynamic_cps.phase_probe_vclock_dwt_adjusted = 0;
  dynamic_cps.phase_probe_adjusted_difference_cycles = 0;
  dynamic_cps.phase_probe_phase_offset_cycles = 0;
  dynamic_cps.last_completed_dynamic_prediction_cycle_count = 0;
  dynamic_cps.last_completed_dynamic_prediction_adjust_count = 0;
  dynamic_cps.last_completed_dynamic_prediction_invalid_count = 0;
  dynamic_cps.last_completed_dynamic_prediction_valid_count = 0;
  dynamic_cps.last_completed_dynamic_prediction_adjust_cycles = 0;
  dynamic_cps.timepop_client_armed = false;
  dynamic_cps.timepop_arm_count = 0;
  dynamic_cps.timepop_arm_failures = 0;
  dynamic_cps.timepop_callback_count = 0;
  dynamic_cps.timepop_last_handle = 0;
  dynamic_cps.timepop_last_deadline = 0;
  dynamic_cps.timepop_last_fire_vclock_raw = 0;
  dynamic_cps.timepop_last_fire_dwt = 0;
  dynamic_cps.timepop_last_fire_gnss_ns = -1;
  dynamic_cps.history_head = 0;
  dynamic_cps.history_count = 0;
  dynamic_cps_reset_second_locked();
  for (uint32_t i = 0; i < DYNAMIC_CPS_HISTORY_CAPACITY; i++) {
    dynamic_cps.history[i] = time_dynamic_cps_record_t{};
  }

  dmb();
  dynamic_cps.seq++;
}

void time_dynamic_cps_phase_probe_update(uint32_t pps_sequence,
                                         uint32_t pps_isr_entry_dwt_raw,
                                         uint32_t arm_dwt_raw,
                                         uint32_t vclock_isr_entry_dwt_raw,
                                         uint32_t pps_dwt_adjusted,
                                         uint32_t vclock_dwt_adjusted,
                                         uint32_t adjusted_difference_cycles,
                                         uint32_t phase_offset_cycles) {
  dynamic_cps.seq++;
  dmb();

  dynamic_cps.phase_probe_valid = true;
  dynamic_cps.phase_probe_pps_sequence = pps_sequence;
  dynamic_cps.phase_probe_pps_isr_entry_dwt_raw = pps_isr_entry_dwt_raw;
  dynamic_cps.phase_probe_arm_dwt_raw = arm_dwt_raw;
  dynamic_cps.phase_probe_vclock_isr_entry_dwt_raw = vclock_isr_entry_dwt_raw;
  dynamic_cps.phase_probe_pps_dwt_adjusted = pps_dwt_adjusted;
  dynamic_cps.phase_probe_vclock_dwt_adjusted = vclock_dwt_adjusted;
  dynamic_cps.phase_probe_adjusted_difference_cycles = adjusted_difference_cycles;
  dynamic_cps.phase_probe_phase_offset_cycles = phase_offset_cycles;

  dmb();
  dynamic_cps.seq++;
}

void time_dynamic_cps_pps_vclock_edge(uint32_t pvc_sequence,
                                      uint32_t pvc_dwt_at_edge) {
  dynamic_cps.seq++;
  dmb();

  const bool have_prev = (dynamic_cps.pvc_sequence != 0);
  const uint32_t base = have_prev
      ? (uint32_t)(pvc_dwt_at_edge - dynamic_cps.current_pvc_dwt_at_edge)
      : 0;

  dynamic_cps_push_record_locked(base);

  dynamic_cps.pvc_sequence = pvc_sequence;
  dynamic_cps.current_pvc_dwt_at_edge = pvc_dwt_at_edge;
  dynamic_cps.base_cycles = base;
  dynamic_cps.current_cycles = base;
  dynamic_cps.last_reseed_value = base;
  dynamic_cps.last_reseed_was_computed = have_prev;
  dynamic_cps.valid = have_prev && base != 0;
  dynamic_cps_reset_second_locked();

  dmb();
  dynamic_cps.seq++;
}

static void dynamic_cps_capture_first_ms_audit_locked(uint32_t sample_ms,
                                                       uint32_t qtimer_event_dwt,
                                                       uint32_t observed_cycles,
                                                       uint32_t event_vclock_ticks,
                                                       uint32_t dynamic_cycles_before_sample,
                                                       uint32_t dynamic_cycles_after_sample,
                                                       uint32_t interval_used_cycles,
                                                       int32_t servo_error_cycles,
                                                       uint32_t servo_abs_error_cycles,
                                                       bool servo_gated,
                                                       int32_t servo_correction_cycles) {
  uint32_t idx = 0;
  if (!audit_checkpoint_index(sample_ms, &idx)) return;
  if (!dynamic_cps.valid || dynamic_cps.base_cycles == 0) return;
  dynamic_cps_first_ms_audit_t a{};
  a.valid = true;
  a.ms = sample_ms;
  a.anchor_dwt = dynamic_cps.current_pvc_dwt_at_edge;

  a.static_cycles_per_second = dynamic_cps.base_cycles;
  a.static_cycles_per_ms_floor = dynamic_cps.base_cycles / 1000U;
  a.expected_elapsed_static_floor =
      (uint32_t)(((uint64_t)dynamic_cps.base_cycles * (uint64_t)event_vclock_ticks) /
                 (uint64_t)VCLOCK_COUNTS_PER_SECOND);
  a.expected_elapsed_static_round =
      line_cycles_at_ticks(dynamic_cps.base_cycles, event_vclock_ticks);
  a.expected_dwt_static_floor = a.anchor_dwt + a.expected_elapsed_static_floor;
  a.expected_dwt_static_round = a.anchor_dwt + a.expected_elapsed_static_round;

  // Legacy field names remain aliases for the static path so existing tooling
  // continues to read the report unchanged.
  a.expected_elapsed_floor = a.expected_elapsed_static_floor;
  a.expected_elapsed_round = a.expected_elapsed_static_round;
  a.expected_dwt_floor = a.expected_dwt_static_floor;
  a.expected_dwt_round = a.expected_dwt_static_round;

  a.dynamic_cycles_per_second = dynamic_cycles_before_sample;
  a.expected_elapsed_dynamic_round =
      line_cycles_at_ticks(dynamic_cycles_before_sample, event_vclock_ticks);
  a.expected_dwt_dynamic_round = a.anchor_dwt + a.expected_elapsed_dynamic_round;

  a.observed_dwt = qtimer_event_dwt;
  a.observed_elapsed_cycles = observed_cycles;

  a.error_vs_static_floor_cycles =
      (int32_t)((int64_t)observed_cycles - (int64_t)a.expected_elapsed_static_floor);
  a.error_vs_static_round_cycles =
      (int32_t)((int64_t)observed_cycles - (int64_t)a.expected_elapsed_static_round);
  a.error_vs_dynamic_round_cycles =
      (int32_t)((int64_t)observed_cycles - (int64_t)a.expected_elapsed_dynamic_round);

  a.servo_gate_threshold_cycles = DYNAMIC_CPS_SERVO_GATE_THRESHOLD_CYCLES;
  a.servo_abs_error_cycles = servo_abs_error_cycles;
  a.servo_gated = servo_gated;
  a.servo_correction_cycles = servo_correction_cycles;
  a.interval_used_cycles = interval_used_cycles;
  a.dynamic_cycles_after_sample = dynamic_cycles_after_sample;
  (void)servo_error_cycles;  // Same value as error_vs_dynamic_round_cycles.

  // Legacy field names remain aliases for the static path.
  a.error_vs_floor_cycles = a.error_vs_static_floor_cycles;
  a.error_vs_round_cycles = a.error_vs_static_round_cycles;

  const bool was_valid = dynamic_cps.first_ms_audit[idx].valid;
  dynamic_cps.first_ms_audit[idx] = a;
  if (!was_valid && dynamic_cps.first_ms_audit_valid_count < DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT) {
    dynamic_cps.first_ms_audit_valid_count++;
  }
}

static void time_dynamic_cps_timepop_update(uint32_t fire_dwt,
                                           uint32_t fire_vclock_raw,
                                           uint32_t scheduled_deadline,
                                           int64_t fire_gnss_ns) {
  dynamic_cps.seq++;
  dmb();

  dynamic_cps.timepop_callback_count++;
  dynamic_cps.timepop_last_deadline = scheduled_deadline;
  dynamic_cps.timepop_last_fire_vclock_raw = fire_vclock_raw;
  dynamic_cps.timepop_last_fire_dwt = fire_dwt;
  dynamic_cps.timepop_last_fire_gnss_ns = fire_gnss_ns;

  const time_anchor_snapshot_t anchor_snap = time_anchor_snapshot();

  if (dynamic_cps.pvc_sequence == 0 || !anchor_snap.ok || !anchor_snap.valid) {
    dynamic_cps.skipped_no_anchor++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  if (!dynamic_cps.valid || dynamic_cps.base_cycles == 0 || dynamic_cps.current_cycles == 0 ||
      anchor_snap.dwt_at_pps_vclock != dynamic_cps.current_pvc_dwt_at_edge) {
    dynamic_cps.skipped_no_cps++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  const uint32_t anchor_counter32 = anchor_snap.counter32_at_pps_vclock;
  const uint32_t target_vclock_ticks = scheduled_deadline - anchor_counter32;
  if (target_vclock_ticks == 0) {
    dynamic_cps.skipped_offset_low++;
    dmb();
    dynamic_cps.seq++;
    return;
  }
  if (target_vclock_ticks >= VCLOCK_COUNTS_PER_SECOND) {
    dynamic_cps.skipped_offset_high++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  const uint32_t sample_ms =
      (target_vclock_ticks + (VCLOCK_INTERVAL_COUNTS / 2U)) / VCLOCK_INTERVAL_COUNTS;
  if (sample_ms < DYNAMIC_CPS_MIN_REFINE_MS) {
    dynamic_cps.skipped_offset_low++;
    dmb();
    dynamic_cps.seq++;
    return;
  }
  if (sample_ms > DYNAMIC_CPS_MAX_REFINE_MS) {
    dynamic_cps.skipped_offset_high++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  const uint32_t event_vclock_ticks = fire_vclock_raw - anchor_counter32;
  if (event_vclock_ticks == 0) {
    dynamic_cps.skipped_offset_low++;
    dmb();
    dynamic_cps.seq++;
    return;
  }
  if (event_vclock_ticks >= VCLOCK_COUNTS_PER_SECOND) {
    dynamic_cps.skipped_offset_high++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  const uint32_t observed_cycles = fire_dwt - dynamic_cps.current_pvc_dwt_at_edge;
  const uint32_t dynamic_cycles_before_sample = dynamic_cps.current_cycles;
  const uint32_t expected_static = line_cycles_at_ticks(dynamic_cps.base_cycles,
                                                       event_vclock_ticks);
  const uint32_t expected_dynamic = line_cycles_at_ticks(dynamic_cycles_before_sample,
                                                        event_vclock_ticks);

  // Servo correction: if the TimePop fire fact is sane, move the one-second CPS
  // prediction by the full observed error. If the residual is impossibly large,
  // preserve the raw observation but do not let the servo learn from it.
  const int32_t servo_error =
      (int32_t)((int64_t)observed_cycles - (int64_t)expected_dynamic);
  const uint32_t abs_servo_error = u32_abs_i32(servo_error);
  const bool servo_gated = abs_servo_error > DYNAMIC_CPS_SERVO_GATE_THRESHOLD_CYCLES;
  const int32_t servo_correction = servo_gated ? 0 : servo_error;
  const uint32_t used_cycles = servo_gated ? expected_dynamic : observed_cycles;

  const uint32_t previous_cycles = dynamic_cps.current_cycles;
  const int64_t corrected_cycles_i64 =
      (int64_t)dynamic_cps.current_cycles + (int64_t)servo_correction;
  dynamic_cps.current_cycles =
      (corrected_cycles_i64 <= 0)
          ? 1U
          : ((corrected_cycles_i64 > (int64_t)UINT32_MAX)
                 ? UINT32_MAX
                 : (uint32_t)corrected_cycles_i64);

  const uint32_t dynamic_cycles_after_sample = dynamic_cps.current_cycles;

  dynamic_cps_capture_first_ms_audit_locked(sample_ms,
                                            fire_dwt,
                                            observed_cycles,
                                            event_vclock_ticks,
                                            dynamic_cycles_before_sample,
                                            dynamic_cycles_after_sample,
                                            used_cycles,
                                            servo_error,
                                            abs_servo_error,
                                            servo_gated,
                                            servo_correction);

  // Keep the historical fit counters populated for report compatibility, but
  // do not use them to recompute the CPS. Dynamic CPS is now servo-owned.
  dynamic_cps.fit.add(sample_ms, used_cycles);

  if (servo_gated) {
    dynamic_cps.substituted_samples_this_second++;
    dynamic_cps.total_substituted_samples++;
    if (dynamic_cps.first_substituted_ms == 0) {
      dynamic_cps.first_substituted_ms = sample_ms;
    }
    dynamic_cps.last_substituted_ms = sample_ms;
    dynamic_cps.substituted_error_sum_cycles += servo_error;
    dynamic_cps.substituted_abs_error_sum_cycles += abs_servo_error;
  } else {
    dynamic_cps.accepted_samples_this_second++;
    dynamic_cps.total_accepted_samples++;
  }

  if (abs_servo_error > dynamic_cps.max_abs_observed_error_cycles) {
    dynamic_cps.max_abs_observed_error_cycles = abs_servo_error;
  }

  dynamic_cps.last_sample_ms = sample_ms;
  dynamic_cps.last_observed_cycles = observed_cycles;
  dynamic_cps.last_used_cycles = used_cycles;
  dynamic_cps.last_expected_static_cycles = expected_static;
  dynamic_cps.last_expected_fit_cycles = expected_dynamic;
  dynamic_cps.last_observed_error_cycles = servo_error;
  dynamic_cps.last_used_error_cycles = servo_correction;
  dynamic_cps.last_static_path_error_cycles =
      (int32_t)((int64_t)observed_cycles - (int64_t)expected_static);
  dynamic_cps.last_fit_error_cycles = servo_correction;
  dynamic_cps.last_point_slope_cycles = dynamic_cps.current_cycles;

  dynamic_cps.refine_ticks_this_second++;
  dynamic_cps.total_refine_ticks++;
  if (dynamic_cps.current_cycles != previous_cycles) {
    dynamic_cps.adjustments_this_second++;
    dynamic_cps.total_adjustments++;
  }

  dmb();
  dynamic_cps.seq++;
}

static void dynamic_cps_timepop_callback(timepop_ctx_t* ctx,
                                         timepop_diag_t*,
                                         void*) {
  if (!ctx) return;
  time_dynamic_cps_timepop_update(ctx->fire_dwt_cyccnt,
                                  ctx->fire_vclock_raw,
                                  ctx->deadline,
                                  ctx->fire_gnss_ns);
}

static bool dynamic_cps_ensure_timepop_client(void) {
  if (dynamic_cps.timepop_client_armed) return true;
  if (!time_valid()) return false;

  // Named replacement makes this idempotent across epoch/recovery churn.
  timepop_cancel_by_name(DYNAMIC_CPS_TIMEPOP_NAME);
  const timepop_handle_t h = timepop_arm(DYNAMIC_CPS_TIMEPOP_PERIOD_NS,
                                         true,
                                         dynamic_cps_timepop_callback,
                                         nullptr,
                                         DYNAMIC_CPS_TIMEPOP_NAME);

  dynamic_cps.seq++;
  dmb();
  if (h == TIMEPOP_INVALID_HANDLE) {
    dynamic_cps.timepop_client_armed = false;
    dynamic_cps.timepop_arm_failures++;
    dmb();
    dynamic_cps.seq++;
    return false;
  }

  dynamic_cps.timepop_client_armed = true;
  dynamic_cps.timepop_arm_count++;
  dynamic_cps.timepop_last_handle = h;
  dmb();
  dynamic_cps.seq++;
  return true;
}

void time_dynamic_cps_cadence_update(uint32_t,
                                     uint32_t) {
  // Legacy compatibility hook for the retired private QTimer cadence rail.
  // Dynamic CPS refinement is now driven by the regular TimePop client above.
}

// ============================================================================
// Writer helpers
// ============================================================================

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock) {
  anchor.seq++;
  dmb();

  anchor.dwt_at_pps_vclock             = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s   = 0;
  anchor.counter32_at_pps_vclock       = counter32_at_pps_vclock;
  anchor.pps_vclock_count              = 1;
  anchor.valid                         = false;

  dmb();
  anchor.seq++;

  prediction_reset();
  time_dynamic_cps_reset();
}

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock) {
  uint32_t new_pps_vclock_count = 1;

  anchor.seq++;
  dmb();

  if (anchor.pps_vclock_count == 0) {
    anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
    anchor.dwt_cycles_per_pps_vclock_s = 0;
    anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
    anchor.pps_vclock_count            = 1;
    anchor.valid                       = false;
    new_pps_vclock_count               = 1;
  } else {
    anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
    anchor.dwt_cycles_per_pps_vclock_s = dwt_cycles_per_pps_vclock_s;
    anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
    anchor.pps_vclock_count++;
    anchor.valid = (dwt_cycles_per_pps_vclock_s > 0);
    new_pps_vclock_count = anchor.pps_vclock_count;
  }

  dmb();
  anchor.seq++;

  if (dwt_cycles_per_pps_vclock_s > 0) {
    prediction_observe_actual(new_pps_vclock_count,
                              dwt_at_pps_vclock,
                              counter32_at_pps_vclock,
                              dwt_cycles_per_pps_vclock_s);
  }

  (void)dynamic_cps_ensure_timepop_client();
}

void time_pps_epoch_reset(uint32_t dwt_at_pps,
                          uint32_t qtimer_at_pps) {
  time_pps_vclock_epoch_reset(dwt_at_pps, qtimer_at_pps);
}

void time_pps_update(uint32_t dwt_at_pps,
                     uint32_t dwt_cycles_per_s,
                     uint32_t qtimer_at_pps) {
  time_pps_vclock_update(dwt_at_pps, dwt_cycles_per_s, qtimer_at_pps);
}

// ============================================================================
// Internal reader — stack-local snapshot with torn-read protection
// ============================================================================

struct time_snapshot_t {
  uint32_t dwt_at_pps_vclock;
  uint32_t dwt_cycles_per_pps_vclock_s;
  uint32_t counter32_at_pps_vclock;
  uint32_t pps_vclock_count;
  bool     valid;
  bool     ok;
};

static time_snapshot_t read_anchor(void) {
  time_snapshot_t s = {};
  s.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = anchor.seq;
    dmb();

    s.dwt_at_pps_vclock           = anchor.dwt_at_pps_vclock;
    s.dwt_cycles_per_pps_vclock_s = anchor.dwt_cycles_per_pps_vclock_s;
    s.counter32_at_pps_vclock     = anchor.counter32_at_pps_vclock;
    s.pps_vclock_count            = anchor.pps_vclock_count;
    s.valid                       = anchor.valid;

    dmb();
    const uint32_t s2 = anchor.seq;

    if (s1 == s2 && (s1 & 1u) == 0u) {
      s.ok = true;
      return s;
    }
  }

  return s;
}

static uint32_t effective_cycles_for_anchor(uint32_t anchor_dwt_at_pps_vclock,
                                            uint32_t raw_cycles) {
  // Operational projection policy:
  //
  //   * If the requested anchor is the current dynamic CPS anchor, use the
  //     live gated-servo CPS estimate.  This is the best available DWT
  //     cycles-per-GNSS-second denominator for current-second interpolation.
  //
  //   * If the request is for an older/external anchor, fall back to the
  //     explicit bookend CPS supplied with that anchor.  This keeps historical
  //     and caller-owned projections reconstructive and avoids applying the
  //     current servo state to the wrong second.
  //
  //   * If the dynamic state has not yet been initialized, fall back to the
  //     static bookend CPS.  During ordinary operation current_cycles is seeded
  //     from base_cycles at every PPS/VCLOCK edge and then refined by the
  //     1 kHz gated servo.
  if (raw_cycles == 0) return 0;

  const time_dynamic_cps_snapshot_t dyn = time_dynamic_cps_snapshot();
  if (dyn.current_cycles != 0 &&
      dyn.current_pvc_dwt_at_edge == anchor_dwt_at_pps_vclock) {
    return dyn.current_cycles;
  }

  return raw_cycles;
}
static inline uint32_t effective_cycles_per_snapshot(const time_snapshot_t& s) {
  return effective_cycles_for_anchor(s.dwt_at_pps_vclock,
                                     s.dwt_cycles_per_pps_vclock_s);
}

// ============================================================================
// Public reader — seqlock-safe anchor snapshot
// ============================================================================

time_anchor_snapshot_t time_anchor_snapshot(void) {
  const time_snapshot_t s = read_anchor();
  time_anchor_snapshot_t pub = {};

  pub.dwt_at_pps_vclock           = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_pps_vclock_s = s.dwt_cycles_per_pps_vclock_s;
  pub.counter32_at_pps_vclock     = s.counter32_at_pps_vclock;
  pub.pps_vclock_count            = s.pps_vclock_count;
  pub.valid                       = s.valid;
  pub.ok                          = s.ok;

  // Legacy aliases.
  pub.dwt_at_pps       = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_s = s.dwt_cycles_per_pps_vclock_s;
  pub.qtimer_at_pps    = s.counter32_at_pps_vclock;
  pub.pps_count        = s.pps_vclock_count;

  return pub;
}

// ============================================================================
// Internal helper — DWT elapsed to GNSS nanoseconds
// ============================================================================

static inline int64_t interpolate_gnss_ns(const time_snapshot_t& s,
                                          uint32_t dwt_elapsed) {
  const uint32_t cycles = effective_cycles_per_snapshot(s);
  if (cycles == 0) return -1;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * (uint64_t)NS_PER_SECOND +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(s.pps_vclock_count - 1) * (uint64_t)NS_PER_SECOND +
                   ns_into_second);
}

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (effective_cycles_per_snapshot(s) == 0) return -1;

  const uint32_t dwt_now = ARM_DWT_CYCCNT;
  const uint32_t dwt_elapsed = dwt_now - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (effective_cycles_per_snapshot(s) == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps_vclock,
                            uint32_t anchor_dwt_cycles_per_pps_vclock_s,
                            uint32_t anchor_pps_vclock_count) {
  const uint32_t cycles = effective_cycles_for_anchor(anchor_dwt_at_pps_vclock,
                                                      anchor_dwt_cycles_per_pps_vclock_s);
  if (cycles == 0) return -1;
  if (anchor_pps_vclock_count == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - anchor_dwt_at_pps_vclock;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * (uint64_t)NS_PER_SECOND +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(anchor_pps_vclock_count - 1) * (uint64_t)NS_PER_SECOND +
                   ns_into_second);
}

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return 0;
  const uint32_t cycles = effective_cycles_per_snapshot(s);
  if (cycles == 0) return 0;
  if (gnss_ns < 0) return 0;

  const int64_t anchor_ns =
      (int64_t)(s.pps_vclock_count - 1) * (int64_t)NS_PER_SECOND;
  const int64_t ns_into_second = gnss_ns - anchor_ns;
  if (ns_into_second < 0) return 0;

  const uint32_t dwt_elapsed =
      (uint32_t)(((uint64_t)ns_into_second *
                  (uint64_t)cycles + (uint64_t)NS_PER_SECOND / 2ULL) /
                 (uint64_t)NS_PER_SECOND);

  return s.dwt_at_pps_vclock + dwt_elapsed;
}

// ============================================================================
// DWT prediction accessors
// ============================================================================

time_dwt_prediction_snapshot_t time_dwt_prediction_snapshot(void) {
  time_dwt_prediction_snapshot_t out{};

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = prediction.seq;
    dmb();

    out.valid = prediction.valid;
    out.pps_vclock_count = prediction.pps_vclock_count;
    out.predicted_cycles_last = prediction.predicted_cycles_last;
    out.actual_cycles_last = prediction.actual_cycles_last;
    out.residual_cycles_last = prediction.residual_cycles_last;
    out.predicted_cycles_next = prediction.predicted_cycles_next;
    out.history_count = prediction.history_count;
    out.history_capacity = PREDICTION_HISTORY_CAPACITY;

    dmb();
    const uint32_t s2 = prediction.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }

  return time_dwt_prediction_snapshot_t{};
}

uint32_t time_dwt_prediction_history(time_dwt_prediction_record_t* out_records,
                                     uint32_t max_records) {
  if (!out_records || max_records == 0) return 0;

  time_dwt_prediction_record_t local[PREDICTION_HISTORY_CAPACITY];
  uint32_t count = 0;
  uint32_t head = 0;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = prediction.seq;
    dmb();

    count = prediction.history_count;
    head = prediction.history_head;
    if (count > PREDICTION_HISTORY_CAPACITY) count = PREDICTION_HISTORY_CAPACITY;
    for (uint32_t i = 0; i < count; i++) local[i] = prediction.history[i];

    dmb();
    const uint32_t s2 = prediction.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) break;
    if (attempt == 3) return 0;
  }

  const uint32_t n = (count < max_records) ? count : max_records;
  const uint32_t start = (count < PREDICTION_HISTORY_CAPACITY)
      ? 0U
      : head;

  const uint32_t skip = count - n;
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t logical = skip + i;
    const uint32_t idx = (start + logical) % PREDICTION_HISTORY_CAPACITY;
    out_records[i] = local[idx];
  }

  return n;
}

bool time_dwt_prediction_valid(void) {
  return time_dwt_prediction_snapshot().valid;
}

uint32_t time_dwt_actual_cycles_last_second(void) {
  return time_dwt_prediction_snapshot().actual_cycles_last;
}

uint32_t time_dwt_predicted_cycles_last_second(void) {
  return time_dwt_prediction_snapshot().predicted_cycles_last;
}

uint32_t time_dwt_next_prediction_cycles(void) {
  return time_dwt_prediction_snapshot().predicted_cycles_next;
}

int32_t time_dwt_prediction_residual_cycles(void) {
  return time_dwt_prediction_snapshot().residual_cycles_last;
}

// ============================================================================
// Dynamic CPS accessors
// ============================================================================

time_dynamic_cps_snapshot_t time_dynamic_cps_snapshot(void) {
  time_dynamic_cps_snapshot_t out{};

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = dynamic_cps.seq;
    dmb();

    out.valid = dynamic_cps.valid;
    out.witness_only = false;
    out.projection_enabled = true;
    out.pvc_sequence = dynamic_cps.pvc_sequence;
    out.current_pvc_dwt_at_edge = dynamic_cps.current_pvc_dwt_at_edge;
    out.pvc_dwt_at_edge = dynamic_cps.current_pvc_dwt_at_edge;
    out.base_cycles = dynamic_cps.base_cycles;
    out.current_cycles = dynamic_cps.current_cycles;
    out.net_adjustment_cycles =
        (int32_t)((int64_t)dynamic_cps.current_cycles - (int64_t)dynamic_cps.base_cycles);
    out.last_reseed_value = dynamic_cps.last_reseed_value;
    out.last_reseed_was_computed = dynamic_cps.last_reseed_was_computed;
    out.last_completed_dynamic_prediction_cycle_count =
        dynamic_cps.last_completed_dynamic_prediction_cycle_count;
    out.last_completed_dynamic_prediction_adjust_count =
        dynamic_cps.last_completed_dynamic_prediction_adjust_count;
    out.last_completed_dynamic_prediction_invalid_count =
        dynamic_cps.last_completed_dynamic_prediction_invalid_count;
    out.last_completed_dynamic_prediction_valid_count =
        dynamic_cps.last_completed_dynamic_prediction_valid_count;
    out.last_completed_dynamic_prediction_adjust_cycles =
        dynamic_cps.last_completed_dynamic_prediction_adjust_cycles;

    out.phase_probe_valid = dynamic_cps.phase_probe_valid;
    out.phase_probe_pps_sequence = dynamic_cps.phase_probe_pps_sequence;
    out.phase_probe_pps_isr_entry_dwt_raw = dynamic_cps.phase_probe_pps_isr_entry_dwt_raw;
    out.phase_probe_arm_dwt_raw = dynamic_cps.phase_probe_arm_dwt_raw;
    out.phase_probe_vclock_isr_entry_dwt_raw = dynamic_cps.phase_probe_vclock_isr_entry_dwt_raw;
    out.phase_probe_pps_dwt_adjusted = dynamic_cps.phase_probe_pps_dwt_adjusted;
    out.phase_probe_vclock_dwt_adjusted = dynamic_cps.phase_probe_vclock_dwt_adjusted;
    out.phase_probe_adjusted_difference_cycles = dynamic_cps.phase_probe_adjusted_difference_cycles;
    out.phase_probe_phase_offset_cycles = dynamic_cps.phase_probe_phase_offset_cycles;

    out.fit_samples_this_second = dynamic_cps.fit.samples;
    out.fit_sum_t2_ms2 = dynamic_cps.fit.sum_t2_ms2;
    out.fit_sum_tx_cycles_ms = dynamic_cps.fit.sum_tx_cycles_ms;
    out.last_sample_ms = dynamic_cps.last_sample_ms;
    out.last_observed_cycles = dynamic_cps.last_observed_cycles;
    out.last_used_cycles = dynamic_cps.last_used_cycles;
    out.last_expected_static_cycles = dynamic_cps.last_expected_static_cycles;
    out.last_expected_fit_cycles = dynamic_cps.last_expected_fit_cycles;
    out.last_observed_error_cycles = dynamic_cps.last_observed_error_cycles;
    out.last_used_error_cycles = dynamic_cps.last_used_error_cycles;
    out.last_static_path_error_cycles = dynamic_cps.last_static_path_error_cycles;
    out.last_fit_error_cycles = dynamic_cps.last_fit_error_cycles;
    out.last_point_slope_cycles = dynamic_cps.last_point_slope_cycles;

    out.outlier_threshold_cycles = DYNAMIC_CPS_SERVO_GATE_THRESHOLD_CYCLES;
    out.accepted_samples_this_second = dynamic_cps.accepted_samples_this_second;
    out.substituted_samples_this_second = dynamic_cps.substituted_samples_this_second;
    out.total_accepted_samples = dynamic_cps.total_accepted_samples;
    out.total_substituted_samples = dynamic_cps.total_substituted_samples;
    out.first_substituted_ms = dynamic_cps.first_substituted_ms;
    out.last_substituted_ms = dynamic_cps.last_substituted_ms;
    out.substituted_error_sum_cycles = dynamic_cps.substituted_error_sum_cycles;
    out.substituted_abs_error_sum_cycles = dynamic_cps.substituted_abs_error_sum_cycles;
    out.max_abs_observed_error_cycles = dynamic_cps.max_abs_observed_error_cycles;

    out.refine_ticks_this_second = dynamic_cps.refine_ticks_this_second;
    out.adjustments_this_second = dynamic_cps.adjustments_this_second;
    out.total_refine_ticks = dynamic_cps.total_refine_ticks;
    out.total_adjustments = dynamic_cps.total_adjustments;
    out.skipped_no_anchor = dynamic_cps.skipped_no_anchor;
    out.skipped_no_cps = dynamic_cps.skipped_no_cps;
    out.skipped_offset_low = dynamic_cps.skipped_offset_low;
    out.skipped_offset_high = dynamic_cps.skipped_offset_high;
    out.eval_count = dynamic_cps.eval_count;
    out.helped_count = dynamic_cps.helped_count;
    out.hurt_count = dynamic_cps.hurt_count;
    out.neutral_count = dynamic_cps.neutral_count;
    out.total_abs_static_error_cycles = dynamic_cps.total_abs_static_error_cycles;
    out.total_abs_dynamic_error_cycles = dynamic_cps.total_abs_dynamic_error_cycles;
    out.last_next_actual_cycles = dynamic_cps.last_next_actual_cycles;
    out.last_static_error_cycles = dynamic_cps.last_static_error_cycles;
    out.last_dynamic_error_cycles = dynamic_cps.last_dynamic_error_cycles;
    out.last_abs_static_error_cycles = dynamic_cps.last_abs_static_error_cycles;
    out.last_abs_dynamic_error_cycles = dynamic_cps.last_abs_dynamic_error_cycles;
    out.last_improvement_cycles = dynamic_cps.last_improvement_cycles;
    out.last_dynamic_helped = dynamic_cps.last_dynamic_helped;
    out.last_endpoint_error_cycles = dynamic_cps.last_endpoint_error_cycles;
    out.last_abs_endpoint_error_cycles = dynamic_cps.last_abs_endpoint_error_cycles;
    out.endpoint_sanity_tolerance_cycles = DYNAMIC_CPS_ENDPOINT_SANITY_TOLERANCE_CYCLES;
    out.last_endpoint_sanity_pass = dynamic_cps.last_endpoint_sanity_pass;
    out.history_count = dynamic_cps.history_count;
    out.history_capacity = DYNAMIC_CPS_HISTORY_CAPACITY;

    dmb();
    const uint32_t s2 = dynamic_cps.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }

  return time_dynamic_cps_snapshot_t{};
}

uint32_t time_dynamic_cps_history(time_dynamic_cps_record_t* out_records,
                                  uint32_t max_records) {
  if (!out_records || max_records == 0) return 0;

  time_dynamic_cps_record_t local[DYNAMIC_CPS_HISTORY_CAPACITY];
  uint32_t count = 0;
  uint32_t head = 0;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = dynamic_cps.seq;
    dmb();

    count = dynamic_cps.history_count;
    head = dynamic_cps.history_head;
    if (count > DYNAMIC_CPS_HISTORY_CAPACITY) count = DYNAMIC_CPS_HISTORY_CAPACITY;
    for (uint32_t i = 0; i < count; i++) local[i] = dynamic_cps.history[i];

    dmb();
    const uint32_t s2 = dynamic_cps.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) break;
    if (attempt == 3) return 0;
  }

  const uint32_t n = (count < max_records) ? count : max_records;
  const uint32_t start = (count < DYNAMIC_CPS_HISTORY_CAPACITY)
      ? 0U
      : head;

  const uint32_t skip = count - n;
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t logical = skip + i;
    const uint32_t idx = (start + logical) % DYNAMIC_CPS_HISTORY_CAPACITY;
    out_records[i] = local[idx];
  }

  return n;
}

bool time_dynamic_cps_valid(void) {
  return time_dynamic_cps_snapshot().valid;
}

uint32_t time_dynamic_cps_current_cycles(void) {
  return time_dynamic_cps_snapshot().current_cycles;
}

uint32_t time_dynamic_cps_current(void) {
  return time_dynamic_cps_current_cycles();
}

bool time_dynamic_cps_cycles_for_anchor(uint32_t pvc_sequence,
                                        uint32_t* out_cycles) {
  if (!out_cycles) return false;

  // Compatibility accessor used by historical interrupt projection code.
  // It intentionally returns the static per-anchor PPS/VCLOCK bookend CPS so
  // old anchors remain reconstructive. Current-anchor operational projection
  // uses effective_cycles_for_anchor() instead.
  const time_dynamic_cps_snapshot_t snap = time_dynamic_cps_snapshot();
  if (snap.valid && snap.pvc_sequence == pvc_sequence && snap.base_cycles != 0) {
    *out_cycles = snap.base_cycles;
    return true;
  }

  time_dynamic_cps_record_t hist[DYNAMIC_CPS_HISTORY_CAPACITY];
  const uint32_t n = time_dynamic_cps_history(hist, DYNAMIC_CPS_HISTORY_CAPACITY);
  for (uint32_t i = 0; i < n; i++) {
    if (hist[i].valid && hist[i].pvc_sequence == pvc_sequence && hist[i].base_cycles != 0) {
      *out_cycles = hist[i].base_cycles;
      return true;
    }
  }

  return false;
}

bool time_dynamic_cps_for_pvc_sequence(uint32_t pvc_sequence,
                                       uint32_t* out_cycles) {
  return time_dynamic_cps_cycles_for_anchor(pvc_sequence, out_cycles);
}

// ============================================================================
// Status
// ============================================================================

uint32_t time_pps_vclock_count(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok) return 0;
  return s.pps_vclock_count;
}

uint32_t time_pps_count(void) {
  return time_pps_vclock_count();
}

bool time_valid(void) {
  const time_snapshot_t s = read_anchor();
  return s.ok && s.valid && effective_cycles_per_snapshot(s) > 0;
}

void time_init(void) {
  anchor = {};
  prediction_reset();
  time_dynamic_cps_reset();
}

void process_time_init(void) {
  time_init();
}

// ============================================================================
// REPORT helpers
// ============================================================================

static uint32_t report_limit_arg(const Payload& args,
                                 uint32_t default_limit,
                                 uint32_t max_limit) {
  const char* s = args.getString("limit");
  if (!s || !*s) return default_limit;
  const uint32_t v = (uint32_t)strtoul(s, nullptr, 10);
  if (v == 0) return default_limit;
  return (v > max_limit) ? max_limit : v;
}

static void add_anchor_scalars(Payload& out) {
  const time_anchor_snapshot_t a = time_anchor_snapshot();
  out.add("time_valid", time_valid());
  out.add("anchor_ok", a.ok);
  out.add("anchor_valid", a.valid);
  out.add("dwt_at_pps_vclock", a.dwt_at_pps_vclock);
  out.add("dwt_cycles_per_pps_vclock_s", a.dwt_cycles_per_pps_vclock_s);
  out.add("counter32_at_pps_vclock", a.counter32_at_pps_vclock);
  out.add("pps_vclock_count", a.pps_vclock_count);
}

static void add_prediction_scalars(Payload& out) {
  const time_dwt_prediction_snapshot_t p = time_dwt_prediction_snapshot();
  out.add("prediction_valid", p.valid);
  out.add("prediction_pps_vclock_count", p.pps_vclock_count);
  out.add("predicted_cycles_last", p.predicted_cycles_last);
  out.add("actual_cycles_last", p.actual_cycles_last);
  out.add("residual_cycles_last", p.residual_cycles_last);
  out.add("predicted_cycles_next", p.predicted_cycles_next);
  out.add("prediction_history_count", p.history_count);
  out.add("prediction_history_capacity", p.history_capacity);
}

static void add_dynamic_cps_scalars(Payload& out) {
  const time_dynamic_cps_snapshot_t c = time_dynamic_cps_snapshot();
  out.add("dynamic_cps_valid", c.valid);
  out.add("dynamic_cps_mode", "TIMEPOP_SERVO_PROJECTION_GATED");
  out.add("dynamic_cps_model", "FIXED_ANCHOR_TIMEPOP_GATED_SERVO_PROJECTION");
  out.add("dynamic_cps_witness_only", c.witness_only);
  out.add("dynamic_cps_projection_enabled", c.projection_enabled);
  out.add("dynamic_cps_pvc_sequence", c.pvc_sequence);
  out.add("dynamic_cps_current_pvc_dwt_at_edge", c.current_pvc_dwt_at_edge);
  out.add("dynamic_cps_base_cycles", c.base_cycles);
  out.add("dynamic_cps_current_cycles", c.current_cycles);
  out.add("dynamic_cps_net_adjustment_cycles", c.net_adjustment_cycles);
  out.add("dynamic_cps_last_reseed_value", c.last_reseed_value);
  out.add("dynamic_cps_last_reseed_was_computed", c.last_reseed_was_computed);
  out.add("dynamic_cps_last_completed_cycle_count",
          c.last_completed_dynamic_prediction_cycle_count);
  out.add("dynamic_cps_last_completed_adjust_count",
          c.last_completed_dynamic_prediction_adjust_count);
  out.add("dynamic_cps_last_completed_invalid_count",
          c.last_completed_dynamic_prediction_invalid_count);
  out.add("dynamic_cps_last_completed_valid_count",
          c.last_completed_dynamic_prediction_valid_count);
  out.add("dynamic_cps_last_completed_adjust_cycles",
          c.last_completed_dynamic_prediction_adjust_cycles);
  out.add("dynamic_cps_timepop_client_armed", dynamic_cps.timepop_client_armed);
  out.add("dynamic_cps_timepop_arm_count", dynamic_cps.timepop_arm_count);
  out.add("dynamic_cps_timepop_arm_failures", dynamic_cps.timepop_arm_failures);
  out.add("dynamic_cps_timepop_callback_count", dynamic_cps.timepop_callback_count);
  out.add("dynamic_cps_timepop_last_handle", dynamic_cps.timepop_last_handle);
  out.add("dynamic_cps_timepop_last_deadline", dynamic_cps.timepop_last_deadline);
  out.add("dynamic_cps_timepop_last_fire_vclock_raw", dynamic_cps.timepop_last_fire_vclock_raw);
  out.add("dynamic_cps_timepop_last_fire_dwt", dynamic_cps.timepop_last_fire_dwt);
  out.add("dynamic_cps_timepop_last_fire_gnss_ns", dynamic_cps.timepop_last_fire_gnss_ns);

  out.add("dynamic_cps_phase_probe_valid", c.phase_probe_valid);
  out.add("dynamic_cps_phase_probe_pps_sequence", c.phase_probe_pps_sequence);
  out.add("dynamic_cps_phase_probe_pps_isr_dwt_raw", c.phase_probe_pps_isr_entry_dwt_raw);
  out.add("dynamic_cps_phase_probe_arm_dwt_raw", c.phase_probe_arm_dwt_raw);
  out.add("dynamic_cps_phase_probe_vclock_isr_dwt_raw", c.phase_probe_vclock_isr_entry_dwt_raw);
  out.add("dynamic_cps_phase_probe_pps_dwt_adjusted", c.phase_probe_pps_dwt_adjusted);
  out.add("dynamic_cps_phase_probe_vclock_dwt_adjusted", c.phase_probe_vclock_dwt_adjusted);
  out.add("dynamic_cps_phase_probe_adjusted_difference_cycles", c.phase_probe_adjusted_difference_cycles);
  out.add("dynamic_cps_phase_probe_phase_offset_cycles", c.phase_probe_phase_offset_cycles);
  out.add("dynamic_cps_phase_probe_note", "Simplified Probe");

  out.add("dynamic_cps_fit_samples_this_second", c.fit_samples_this_second);
  out.add("dynamic_cps_fit_sum_t2_ms2", c.fit_sum_t2_ms2);
  out.add("dynamic_cps_fit_sum_tx_cycles_ms", c.fit_sum_tx_cycles_ms);
  out.add("dynamic_cps_last_sample_ms", c.last_sample_ms);
  out.add("dynamic_cps_last_observed_cycles", c.last_observed_cycles);
  out.add("dynamic_cps_last_used_cycles", c.last_used_cycles);
  out.add("dynamic_cps_last_expected_static_cycles", c.last_expected_static_cycles);
  out.add("dynamic_cps_last_expected_fit_cycles", c.last_expected_fit_cycles);
  out.add("dynamic_cps_last_observed_error_cycles", c.last_observed_error_cycles);
  out.add("dynamic_cps_last_used_error_cycles", c.last_used_error_cycles);
  out.add("dynamic_cps_last_static_path_error_cycles", c.last_static_path_error_cycles);
  out.add("dynamic_cps_last_fit_error_cycles", c.last_fit_error_cycles);
  out.add("dynamic_cps_last_point_slope_cycles", c.last_point_slope_cycles);

  out.add("dynamic_cps_outlier_threshold_cycles", c.outlier_threshold_cycles);
  out.add("dynamic_cps_accepted_samples_this_second", c.accepted_samples_this_second);
  out.add("dynamic_cps_substituted_samples_this_second", c.substituted_samples_this_second);
  out.add("dynamic_cps_total_accepted_samples", c.total_accepted_samples);
  out.add("dynamic_cps_total_substituted_samples", c.total_substituted_samples);
  out.add("dynamic_cps_first_substituted_ms", c.first_substituted_ms);
  out.add("dynamic_cps_last_substituted_ms", c.last_substituted_ms);
  out.add("dynamic_cps_substituted_error_sum_cycles", c.substituted_error_sum_cycles);
  out.add("dynamic_cps_substituted_abs_error_sum_cycles", c.substituted_abs_error_sum_cycles);
  out.add("dynamic_cps_max_abs_observed_error_cycles", c.max_abs_observed_error_cycles);

  out.add("dynamic_cps_refine_ticks_this_second", c.refine_ticks_this_second);
  out.add("dynamic_cps_adjustments_this_second", c.adjustments_this_second);
  out.add("dynamic_cps_total_refine_ticks", c.total_refine_ticks);
  out.add("dynamic_cps_total_adjustments", c.total_adjustments);
  out.add("dynamic_cps_skipped_no_anchor", c.skipped_no_anchor);
  out.add("dynamic_cps_skipped_no_cps", c.skipped_no_cps);
  out.add("dynamic_cps_skipped_offset_low", c.skipped_offset_low);
  out.add("dynamic_cps_skipped_offset_high", c.skipped_offset_high);
  out.add("dynamic_cps_eval_count", c.eval_count);
  out.add("dynamic_cps_helped_count", c.helped_count);
  out.add("dynamic_cps_hurt_count", c.hurt_count);
  out.add("dynamic_cps_neutral_count", c.neutral_count);
  out.add("dynamic_cps_total_abs_static_error_cycles", c.total_abs_static_error_cycles);
  out.add("dynamic_cps_total_abs_dynamic_error_cycles", c.total_abs_dynamic_error_cycles);
  out.add("dynamic_cps_mean_abs_static_error_cycles",
          c.eval_count ? ((double)c.total_abs_static_error_cycles / (double)c.eval_count) : 0.0, 6);
  out.add("dynamic_cps_mean_abs_dynamic_error_cycles",
          c.eval_count ? ((double)c.total_abs_dynamic_error_cycles / (double)c.eval_count) : 0.0, 6);
  const int64_t total_improvement =
      (int64_t)c.total_abs_static_error_cycles -
      (int64_t)c.total_abs_dynamic_error_cycles;
  out.add("dynamic_cps_total_improvement_cycles", total_improvement);
  out.add("dynamic_cps_mean_improvement_cycles",
          c.eval_count ? ((double)total_improvement / (double)c.eval_count) : 0.0, 6);
  out.add("dynamic_cps_last_next_actual_cycles", c.last_next_actual_cycles);
  out.add("dynamic_cps_last_static_error_cycles", c.last_static_error_cycles);
  out.add("dynamic_cps_last_dynamic_error_cycles", c.last_dynamic_error_cycles);
  out.add("dynamic_cps_last_abs_static_error_cycles", c.last_abs_static_error_cycles);
  out.add("dynamic_cps_last_abs_dynamic_error_cycles", c.last_abs_dynamic_error_cycles);
  out.add("dynamic_cps_last_improvement_cycles", c.last_improvement_cycles);
  out.add("dynamic_cps_last_dynamic_helped", c.last_dynamic_helped);
  out.add("dynamic_cps_last_endpoint_error_cycles", c.last_endpoint_error_cycles);
  out.add("dynamic_cps_last_abs_endpoint_error_cycles", c.last_abs_endpoint_error_cycles);
  out.add("dynamic_cps_endpoint_sanity_tolerance_cycles", c.endpoint_sanity_tolerance_cycles);
  out.add("dynamic_cps_last_endpoint_sanity_pass", c.last_endpoint_sanity_pass);
  out.add("dynamic_cps_history_count", c.history_count);
  out.add("dynamic_cps_history_capacity", c.history_capacity);
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_COMPACT_REPORT");
  add_anchor_scalars(out);
  add_prediction_scalars(out);
  add_dynamic_cps_scalars(out);
  return out;
}

static Payload cmd_anchor_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_ANCHOR_REPORT");
  add_anchor_scalars(out);
  return out;
}

static Payload cmd_prediction_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_DWT_PREDICTION_REPORT");
  add_prediction_scalars(out);
  return out;
}

static Payload cmd_dynamic_cps_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_DYNAMIC_CPS_REPORT");
  add_dynamic_cps_scalars(out);
  return out;
}

static Payload cmd_prediction_history(const Payload& args) {
  const uint32_t limit = report_limit_arg(args, HISTORY_DEFAULT_LIMIT,
                                          PREDICTION_HISTORY_CAPACITY);
  time_dwt_prediction_record_t hist[PREDICTION_HISTORY_CAPACITY];
  const uint32_t n = time_dwt_prediction_history(hist, limit);

  Payload out;
  out.add("model", "TIME_DWT_PREDICTION_HISTORY");
  out.add("limit", limit);
  out.add("count", n);
  out.add("capacity", (uint32_t)PREDICTION_HISTORY_CAPACITY);

  PayloadArray arr;
  for (uint32_t i = 0; i < n; i++) {
    Payload e;
    e.add("pps_vclock_count", hist[i].pps_vclock_count);
    e.add("dwt_at_pps_vclock", hist[i].dwt_at_pps_vclock);
    e.add("counter32_at_pps_vclock", hist[i].counter32_at_pps_vclock);
    e.add("predicted_cycles", hist[i].predicted_cycles);
    e.add("actual_cycles", hist[i].actual_cycles);
    e.add("residual_cycles", hist[i].residual_cycles);
    e.add("valid", hist[i].valid);
    arr.add(e);
  }
  out.add_array("prediction_history", arr);
  return out;
}

static Payload cmd_dynamic_cps_history(const Payload& args) {
  const uint32_t limit = report_limit_arg(args, HISTORY_DEFAULT_LIMIT,
                                          DYNAMIC_CPS_HISTORY_CAPACITY);
  time_dynamic_cps_record_t hist[DYNAMIC_CPS_HISTORY_CAPACITY];
  const uint32_t n = time_dynamic_cps_history(hist, limit);

  Payload out;
  out.add("model", "TIME_DYNAMIC_CPS_HISTORY");
  out.add("limit", limit);
  out.add("count", n);
  out.add("capacity", (uint32_t)DYNAMIC_CPS_HISTORY_CAPACITY);

  PayloadArray arr;
  for (uint32_t i = 0; i < n; i++) {
    Payload e;
    e.add("pvc_sequence", hist[i].pvc_sequence);
    e.add("pvc_dwt_at_edge", hist[i].pvc_dwt_at_edge);
    e.add("base_cycles", hist[i].base_cycles);
    e.add("final_cycles", hist[i].final_cycles);
    e.add("net_adjustment_cycles", hist[i].net_adjustment_cycles);
    e.add("refine_ticks", hist[i].refine_ticks);
    e.add("adjustments", hist[i].adjustments);

    e.add("fit_samples", hist[i].fit_samples);
    e.add("fit_sum_t2_ms2", hist[i].fit_sum_t2_ms2);
    e.add("fit_sum_tx_cycles_ms", hist[i].fit_sum_tx_cycles_ms);
    e.add("last_sample_ms", hist[i].last_sample_ms);
    e.add("last_observed_cycles", hist[i].last_observed_cycles);
    e.add("last_used_cycles", hist[i].last_used_cycles);
    e.add("last_expected_static_cycles", hist[i].last_expected_static_cycles);
    e.add("last_expected_fit_cycles", hist[i].last_expected_fit_cycles);
    e.add("last_observed_error_cycles", hist[i].last_observed_error_cycles);
    e.add("last_used_error_cycles", hist[i].last_used_error_cycles);
    e.add("last_static_path_error_cycles", hist[i].last_static_path_error_cycles);
    e.add("last_fit_error_cycles", hist[i].last_fit_error_cycles);
    e.add("last_point_slope_cycles", hist[i].last_point_slope_cycles);

    e.add("outlier_threshold_cycles", hist[i].outlier_threshold_cycles);
    e.add("accepted_samples", hist[i].accepted_samples);
    e.add("substituted_samples", hist[i].substituted_samples);
    e.add("first_substituted_ms", hist[i].first_substituted_ms);
    e.add("last_substituted_ms", hist[i].last_substituted_ms);
    e.add("substituted_error_sum_cycles", hist[i].substituted_error_sum_cycles);
    e.add("substituted_abs_error_sum_cycles", hist[i].substituted_abs_error_sum_cycles);
    e.add("max_abs_observed_error_cycles", hist[i].max_abs_observed_error_cycles);

    e.add("endpoint_error_cycles", hist[i].endpoint_error_cycles);
    e.add("abs_endpoint_error_cycles", hist[i].abs_endpoint_error_cycles);
    e.add("endpoint_sanity_tolerance_cycles", hist[i].endpoint_sanity_tolerance_cycles);
    e.add("endpoint_sanity_pass", hist[i].endpoint_sanity_pass);

    e.add("eval_valid", hist[i].eval_valid);
    e.add("next_actual_cycles", hist[i].next_actual_cycles);
    e.add("static_error_cycles", hist[i].static_error_cycles);
    e.add("dynamic_error_cycles", hist[i].dynamic_error_cycles);
    e.add("abs_static_error_cycles", hist[i].abs_static_error_cycles);
    e.add("abs_dynamic_error_cycles", hist[i].abs_dynamic_error_cycles);
    e.add("improvement_cycles", hist[i].improvement_cycles);
    e.add("dynamic_helped", hist[i].dynamic_helped);
    e.add("valid", hist[i].valid);
    arr.add(e);
  }
  out.add_array("dynamic_cps_history", arr);
  return out;
}

static Payload cmd_dynamic_cps_first_ms(const Payload&) {
  Payload out;
  out.add("model", "TIME_DYNAMIC_CPS_FIRST_MS_AUDIT");

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = dynamic_cps.seq;
    dmb();

    const bool valid = dynamic_cps.valid;
    const uint32_t pvc_sequence = dynamic_cps.pvc_sequence;
    const uint32_t anchor_dwt = dynamic_cps.current_pvc_dwt_at_edge;
    const uint32_t static_cycles_per_second = dynamic_cps.base_cycles;
    const uint32_t static_cycles_per_ms_floor = dynamic_cps.base_cycles / 1000U;
    const uint32_t current_cycles = dynamic_cps.current_cycles;
    const uint32_t valid_count = dynamic_cps.first_ms_audit_valid_count;
    dynamic_cps_first_ms_audit_t audit[DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT];
    for (uint32_t i = 0; i < DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT; i++) {
      audit[i] = dynamic_cps.first_ms_audit[i];
    }

    const bool phase_probe_valid = dynamic_cps.phase_probe_valid;
    const uint32_t phase_probe_pps_sequence = dynamic_cps.phase_probe_pps_sequence;
    const uint32_t phase_probe_pps_isr_entry_dwt_raw = dynamic_cps.phase_probe_pps_isr_entry_dwt_raw;
    const uint32_t phase_probe_arm_dwt_raw = dynamic_cps.phase_probe_arm_dwt_raw;
    const uint32_t phase_probe_vclock_isr_entry_dwt_raw = dynamic_cps.phase_probe_vclock_isr_entry_dwt_raw;
    const uint32_t phase_probe_pps_dwt_adjusted = dynamic_cps.phase_probe_pps_dwt_adjusted;
    const uint32_t phase_probe_vclock_dwt_adjusted = dynamic_cps.phase_probe_vclock_dwt_adjusted;
    const uint32_t phase_probe_adjusted_difference_cycles = dynamic_cps.phase_probe_adjusted_difference_cycles;
    const uint32_t phase_probe_phase_offset_cycles = dynamic_cps.phase_probe_phase_offset_cycles;

    dmb();
    const uint32_t s2 = dynamic_cps.seq;
    if (s1 != s2 || (s1 & 1u) != 0u) {
      if (attempt == 3) return Payload{};
      continue;
    }

    out.add("valid", valid);
    out.add("pvc_sequence", pvc_sequence);
    out.add("anchor_dwt", anchor_dwt);
    out.add("static_cycles_per_second", static_cycles_per_second);
    out.add("static_cycles_per_ms_floor", static_cycles_per_ms_floor);
    out.add("current_cycles", current_cycles);
    out.add("timepop_client_armed", dynamic_cps.timepop_client_armed);
    out.add("timepop_callback_count", dynamic_cps.timepop_callback_count);
    out.add("timepop_last_deadline", dynamic_cps.timepop_last_deadline);
    out.add("timepop_last_fire_vclock_raw", dynamic_cps.timepop_last_fire_vclock_raw);
    out.add("timepop_last_fire_dwt", dynamic_cps.timepop_last_fire_dwt);
    out.add("servo_gate_threshold_cycles", (uint32_t)DYNAMIC_CPS_SERVO_GATE_THRESHOLD_CYCLES);
    out.add("audit_count", valid_count);
    out.add("audit_capacity", (uint32_t)DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT);
    out.add("audit_start_ms", (uint32_t)DYNAMIC_CPS_FIRST_MS_AUDIT_START_MS);
    out.add("audit_step_ms", (uint32_t)DYNAMIC_CPS_FIRST_MS_AUDIT_STEP_MS);
    out.add("audit_end_ms", (uint32_t)DYNAMIC_CPS_FIRST_MS_AUDIT_END_MS);
    out.add("note", "DWT values are TimePop fire event coordinates shared by scheduled clients");
    out.add("comparison_note", "Rows show TimePop event expected cycles, actual cycles, residuals, and servo gate behavior for static bookend CPS versus dynamic servo CPS.");

    out.add("phase_probe_valid", phase_probe_valid);
    out.add("phase_probe_pps_sequence", phase_probe_pps_sequence);
    out.add("phase_probe_pps_isr_dwt_raw", phase_probe_pps_isr_entry_dwt_raw);
    out.add("phase_probe_arm_dwt_raw", phase_probe_arm_dwt_raw);
    out.add("phase_probe_vclock_isr_dwt_raw", phase_probe_vclock_isr_entry_dwt_raw);
    out.add("phase_probe_pps_dwt_adjusted", phase_probe_pps_dwt_adjusted);
    out.add("phase_probe_vclock_dwt_adjusted", phase_probe_vclock_dwt_adjusted);
    out.add("phase_probe_adjusted_difference_cycles", phase_probe_adjusted_difference_cycles);
    out.add("phase_probe_phase_offset_cycles", phase_probe_phase_offset_cycles);
    out.add("phase_probe_note", "Simplified Probe");

    PayloadArray arr;
    for (uint32_t i = 0; i < DYNAMIC_CPS_FIRST_MS_AUDIT_COUNT; i++) {
      Payload e;
      e.add("ms", audit[i].ms);
      e.add("anchor_dwt", audit[i].anchor_dwt);
      e.add("static_cycles_per_second", audit[i].static_cycles_per_second);
      e.add("dynamic_cycles_per_second", audit[i].dynamic_cycles_per_second);
      e.add("static_interval_expected_cycles", audit[i].expected_elapsed_static_round);
      e.add("dynamic_interval_expected_cycles", audit[i].expected_elapsed_dynamic_round);
      e.add("interval_actual_cycles", audit[i].observed_elapsed_cycles);
      e.add("static_interval_residual_cycles", audit[i].error_vs_static_round_cycles);
      e.add("dynamic_interval_residual_cycles", audit[i].error_vs_dynamic_round_cycles);
      e.add("servo_gate_threshold_cycles", audit[i].servo_gate_threshold_cycles);
      e.add("servo_abs_error_cycles", audit[i].servo_abs_error_cycles);
      e.add("servo_gated", audit[i].servo_gated);
      e.add("servo_correction_cycles", audit[i].servo_correction_cycles);
      e.add("interval_used_cycles", audit[i].interval_used_cycles);
      e.add("dynamic_cycles_after_sample", audit[i].dynamic_cycles_after_sample);
      arr.add(e);
    }
    out.add_array("first_ms_audit", arr);
    return out;
  }

  return Payload{};
}

static const process_command_entry_t TIME_COMMANDS[] = {
  { "REPORT",              cmd_report              },
  { "ANCHOR_REPORT",       cmd_anchor_report       },
  { "PREDICTION_REPORT",   cmd_prediction_report   },
  { "PREDICTION_HISTORY",  cmd_prediction_history  },
  { "DYNAMIC_CPS_REPORT",    cmd_dynamic_cps_report   },
  { "DYNAMIC_CPS_HISTORY",   cmd_dynamic_cps_history  },
  { "DYNAMIC_CPS_FIRST_MS",  cmd_dynamic_cps_first_ms },
  { nullptr,               nullptr                 }
};

static const process_vtable_t TIME_PROCESS = {
  .process_id    = "TIME",
  .commands      = TIME_COMMANDS,
  .subscriptions = nullptr,
};

void process_time_register(void) {
  process_register("TIME", &TIME_PROCESS);
}
