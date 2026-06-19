// ============================================================================
// process_interrupt.h — lean operational ABI
// ============================================================================
// Public interrupt custody API.  Comments and retired doctrine text have been
// trimmed; field layout is preserved for source compatibility with current
// CLOCKS/TIMEBASE consumers while process_interrupt.cpp removes retired
// diagnostic producers.
// ============================================================================

#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*);

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  VCLOCK,
  OCXO1,
  OCXO2,
  TIMEPOP,
};

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  QTIMER1,
  QTIMER2,
  QTIMER3,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  QTIMER1_CH0_COMP,
  QTIMER1_CH1_COMP,
  QTIMER1_CH2_COMP,
  QTIMER1_CH3_COMP,
  QTIMER2_CH0_COMP,
  QTIMER2_CH1_COMP,
  QTIMER3_CH0_COMP,
  QTIMER3_CH1_COMP,
  QTIMER3_CH3_COMP,
  GPIO_EDGE,
};

enum class interrupt_event_status_t : uint8_t {
  OK = 0,
  HOLD,
  FAULT,
};

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind     = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t   provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t            lane     = interrupt_lane_t::NONE;
  interrupt_event_status_t    status   = interrupt_event_status_t::OK;

  uint32_t dwt_at_event = 0;

  uint64_t gnss_ns_at_event = 0;

  uint32_t counter32_at_event = 0;

  uint32_t pps_coincidence_cycles = 0;
  bool     pps_coincidence_valid  = false;
};

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t   provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t            lane     = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind     = interrupt_subscriber_kind_t::NONE;

  uint32_t dwt_at_event       = 0;
  uint64_t gnss_ns_at_event   = 0;
  uint32_t counter32_at_event = 0;

  bool     spinidle_shadow_valid = false;
  uint32_t spinidle_shadow_dwt = 0;
  uint32_t spinidle_shadow_to_isr_entry_cycles = 0;
  uint32_t spinidle_shadow_valid_threshold_cycles = 0;

  bool     dwt_synthetic = false;
  bool     dwt_repair_candidate = false;
  uint32_t dwt_original_at_event = 0;
  uint32_t dwt_predicted_at_event = 0;
  uint32_t dwt_used_at_event = 0;

  uint32_t dwt_isr_entry_raw = 0;

  uint32_t dwt_event_from_isr_entry_raw = 0;
  int32_t  dwt_isr_entry_to_event_correction_cycles = 0;
  int32_t  dwt_published_minus_event_cycles = 0;
  int32_t  dwt_used_minus_event_cycles = 0;

  int32_t  dwt_synthetic_error_cycles = 0;
  uint32_t dwt_synthetic_threshold_cycles = 0;
  const char* dwt_synthetic_reason = nullptr;

  bool     dwt_interval_gate_valid = false;
  bool     dwt_interval_sample_accepted = false;
  bool     dwt_interval_sample_rejected = false;
  bool     dwt_interval_ema_updated = false;
  uint32_t dwt_interval_observed_cycles = 0;
  uint32_t dwt_interval_prediction_cycles = 0;
  uint32_t dwt_interval_effective_cycles = 0;
  int32_t  dwt_interval_residual_cycles = 0;
  uint32_t dwt_interval_gate_threshold_cycles = 0;
  uint32_t dwt_interval_accept_count = 0;
  uint32_t dwt_interval_reject_count = 0;
  bool     dwt_interval_resync_applied = false;
  uint32_t dwt_interval_resync_count = 0;
  uint32_t dwt_interval_reject_streak = 0;

  bool     dwt_interval_adjacency_gate_valid = false;
  bool     dwt_interval_adjacency_ok = false;
  bool     dwt_interval_adjacency_rejected = false;
  uint32_t dwt_interval_counter_delta_ticks = 0;
  uint32_t dwt_interval_expected_counter_delta_ticks = 0;
  uint32_t dwt_interval_adjacency_reject_count = 0;

  bool     dwt_yardstick_valid = false;
  bool     dwt_yardstick_stale = false;
  bool     dwt_yardstick_seeded = false;
  bool     dwt_yardstick_excursion = false;
  uint32_t dwt_yardstick_pps_sequence = 0;
  uint32_t dwt_yardstick_pps_seq_delta = 0;
  uint32_t dwt_yardstick_g_now_cycles = 0;
  uint32_t dwt_yardstick_g_prev_cycles = 0;
  uint32_t dwt_yardstick_inferred_interval_cycles = 0;
  uint32_t dwt_yardstick_observed_interval_cycles = 0;
  int32_t  dwt_yardstick_inferred_minus_observed_cycles = 0;
  uint32_t dwt_yardstick_inferred_endpoint_dwt = 0;
  uint32_t dwt_yardstick_inferred_endpoint_frac_q16 = 0;
  int32_t  dwt_yardstick_endpoint_minus_observed_cycles = 0;
  uint32_t dwt_yardstick_gate_threshold_cycles = 0;
  uint32_t dwt_yardstick_gate_agree_count = 0;
  uint32_t dwt_yardstick_gate_excursion_count = 0;

  bool     dwt_yardstick_authority = false;
  uint32_t dwt_ema_dwt_at_event = 0;
  uint32_t dwt_yardstick_auth_endpoint_dwt = 0;
  uint32_t dwt_yardstick_auth_endpoint_frac_q16 = 0;
  int32_t  dwt_yardstick_auth_error_cycles = 0;
  bool     dwt_yardstick_auth_anchor_applied = false;

  bool     regression_valid = false;
  uint32_t regression_sequence = 0;
  uint32_t regression_sample_count = 0;
  uint32_t regression_observed_dwt_at_event = 0;
  uint32_t regression_inferred_dwt_at_event = 0;
  int32_t  regression_inferred_minus_observed_cycles = 0;
  uint32_t regression_target_counter32_at_event = 0;
  uint16_t regression_target_hardware16_at_event = 0;
  uint16_t regression_observed_hardware16_at_event = 0;
  uint64_t regression_slope_q16_cycles_per_sample = 0;
  int64_t  regression_slope_delta_q16_cycles_per_sample = 0;
  int32_t  regression_fit_error_mean_q16_cycles = 0;
  uint32_t regression_fit_error_stddev_q16_cycles = 0;
  int32_t  regression_fit_error_min_cycles = 0;
  int32_t  regression_fit_error_max_cycles = 0;
  uint32_t regression_fit_error_gt_plus4_count = 0;
  uint32_t regression_fit_error_lt_minus4_count = 0;
  uint32_t regression_fit_error_abs_gt4_count = 0;

  uint32_t anchor_sequence_used = 0;
  uint32_t anchor_age_slots = 0;
  uint32_t anchor_selection_kind = 0;
  uint32_t anchor_dwt_at_edge = 0;
  int64_t  anchor_gnss_ns_at_edge = -1;
  uint32_t anchor_cps = 0;
  uint64_t anchor_ns_delta = 0;
  uint32_t anchor_failure_mask = 0;

  uint32_t pps_edge_sequence          = 0;
  uint32_t pps_edge_dwt_isr_entry_raw = 0;   // carries pvc.dwt_at_edge
  int64_t  pps_edge_gnss_ns           = -1;
  int64_t  pps_edge_minus_event_ns    = 0;

  uint32_t pps_edge_dwt_cycles_from_vclock = 0;
  int64_t  pps_edge_ns_from_vclock         = 0;
  uint32_t pps_edge_vclock_event_count     = 0;

  uint32_t pps_coincidence_cycles = 0;
  bool     pps_coincidence_valid  = false;

  uint32_t ocxo_service_class = 0;
  int32_t  ocxo_service_offset_signed_ticks = 0;
  uint32_t ocxo_service_offset_abs_ticks = 0;
  uint32_t ocxo_interpreted_late_ticks = 0;
  uint32_t ocxo_early_ticks = 0;
  uint32_t ocxo_target_delta_mod65536_ticks = 0;
  uint32_t ocxo_arm_remaining_ticks = 0;
  uint32_t ocxo_arm_to_isr_ticks = 0;
  uint32_t ocxo_arm_to_isr_dwt_cycles = 0;

  // Split-channel compare/counter custody witness.  OCXO1 currently uses
  // QTimer2 CH0 as the passive counter and CH1 as the active compare rail;
  // OCXO2 uses one physical CH3 rail.  These fields prove whether the counter
  // rail and compare rail remain phase-coherent across arm->ISR.
  uint16_t ocxo_arm_counter_low16 = 0;
  uint16_t ocxo_arm_compare_low16 = 0;
  uint32_t ocxo_arm_counter_minus_compare_ticks = 0;
  uint32_t ocxo_arm_compare_remaining_ticks = 0;
  uint16_t ocxo_isr_counter_low16 = 0;
  uint16_t ocxo_isr_compare_low16 = 0;
  uint32_t ocxo_isr_counter_minus_compare_ticks = 0;
  uint32_t ocxo_compare_delta_mod65536_ticks = 0;
  int32_t  ocxo_compare_service_offset_signed_ticks = 0;
  uint32_t ocxo_compare_interpreted_late_ticks = 0;
  uint32_t ocxo_compare_early_ticks = 0;
  uint32_t ocxo_compare_arm_to_isr_ticks = 0;

  uint32_t ocxo_perishable_fact_sequence = 0;
  int32_t  ocxo_service_correction_cycles = 0;
  uint32_t ocxo_service_corrected_dwt_at_event = 0;
  uint32_t ocxo_fact_ring_overflow_count = 0;
  uint32_t ocxo_counter_delta_violation_count = 0;
  uint32_t ocxo_last_bad_counter_delta = 0;
  uint32_t ocxo_last_counter_delta_ticks = 0;
  uint32_t ocxo_expected_counter_delta_ticks = 0;
  bool     ocxo_counter_adjacency_valid = false;
  bool     ocxo_counter_adjacency_ok = false;
  bool     ocxo_counter_adjacency_rejected = false;
  uint32_t ocxo_counter_adjacency_reject_count = 0;

  bool     ocxo_sample_phase_valid = false;
  uint32_t ocxo_sample_phase_ticks = 0;     // 10 MHz ticks; 2500 = +250 us
  uint32_t ocxo_sample_phase_ns = 0;
  uint32_t ocxo_sample_phase_us = 0;
  uint32_t ocxo_sample_period_ticks = 0;    // normally 10,000 ticks = 1 ms
  uint32_t ocxo_sample_dwt_at_event = 0;
  uint32_t ocxo_sample_counter32_at_event = 0;

  uint32_t ocxo_boundary_dwt_at_event = 0;
  uint32_t ocxo_boundary_counter32_at_event = 0;
  int32_t  ocxo_boundary_correction_cycles = 0;

  // Retired SlipLedger compatibility fields.  process_interrupt no longer
  // owns or updates a SlipLedger state machine; these fields remain in the
  // public diagnostic ABI so existing CLOCKS/TIMEBASE/raw_cycles consumers
  // compile unchanged and observe default zero/null values.
  bool     slipledger_active = false;
  bool     slipledger_event_corrected = false;
  bool     slipledger_event_violation = false;
  int32_t  slipledger_ticks = 0;
  int32_t  slipledger_event_ticks = 0;
  uint32_t slipledger_generation = 0;
  uint32_t slipledger_observe_count = 0;
  uint32_t slipledger_ok_count = 0;
  uint32_t slipledger_violation_count = 0;
  uint32_t slipledger_correction_count = 0;
  uint32_t slipledger_noop_violation_count = 0;
  uint32_t slipledger_early_count = 0;
  uint32_t slipledger_late_count = 0;
  uint32_t slipledger_one_second_observe_count = 0;
  uint32_t slipledger_one_second_ok_count = 0;
  uint32_t slipledger_one_second_violation_count = 0;
  uint32_t slipledger_one_second_correction_count = 0;
  uint32_t slipledger_last_expected_dwt = 0;
  uint32_t slipledger_last_observed_dwt = 0;
  uint32_t slipledger_last_authored_dwt = 0;
  uint32_t slipledger_last_expected_interval_cycles = 0;
  uint32_t slipledger_last_observed_interval_cycles = 0;
  int32_t  slipledger_last_dwt_error_cycles = 0;
  uint32_t slipledger_last_target_counter32 = 0;
  uint16_t slipledger_last_hardware_target_low16 = 0;
  uint16_t slipledger_last_ambient_low16 = 0;
  uint32_t slipledger_last_tick_mod = 0;
  uint32_t slipledger_reason_code = 0;
  uint32_t slipledger_last_correction_reason_code = 0;
  int32_t  slipledger_last_correction_ticks = 0;
  int32_t  slipledger_last_correction_dwt_error_cycles = 0;
  const char* slipledger_reason = nullptr;

  uint32_t anomaly_count = 0;
};

static constexpr int32_t DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES = 10000;

static constexpr uint32_t SMARTZERO_LANE_COUNT = 3;
static constexpr uint32_t SMARTZERO_SAMPLE_RATE_HZ = 1000;
static constexpr uint32_t SMARTZERO_COUNTER_DELTA_TICKS = 10000;
static constexpr int32_t  SMARTZERO_DEFAULT_TOLERANCE_CYCLES = 4;

enum class interrupt_smartzero_phase_t : uint8_t {
  IDLE     = 0,
  RUNNING  = 1,
  COMPLETE = 2,
  ABORTED  = 3,
};

enum class interrupt_smartzero_lane_state_t : uint8_t {
  IDLE      = 0,
  ACQUIRING = 1,
  LOCKED    = 2,
};

enum class interrupt_smartzero_decision_t : uint8_t {
  NONE             = 0,
  WAITING_FOR_CPS  = 1,
  FIRST_SAMPLE     = 2,
  ACCEPTED         = 3,
  REJECTED_DWT     = 4,
  REJECTED_COUNTER = 5,
};

struct interrupt_smartzero_lane_snapshot_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_smartzero_lane_state_t state = interrupt_smartzero_lane_state_t::IDLE;
  interrupt_smartzero_decision_t last_decision = interrupt_smartzero_decision_t::NONE;

  uint32_t sample_count = 0;
  uint32_t interval_attempt_count = 0;
  uint32_t accepted_count = 0;
  uint32_t rejected_count = 0;
  uint32_t waiting_for_cps_count = 0;

  uint32_t expected_interval_cycles = 0;
  int32_t  tolerance_cycles = SMARTZERO_DEFAULT_TOLERANCE_CYCLES;
  uint32_t required_counter_delta_ticks = SMARTZERO_COUNTER_DELTA_TICKS;
  uint32_t cps_used = 0;

  uint32_t last_sample_dwt = 0;
  uint32_t last_sample_counter32 = 0;
  uint16_t last_sample_hardware16 = 0;

  uint32_t previous_sample_dwt = 0;
  uint32_t previous_sample_counter32 = 0;
  uint16_t previous_sample_hardware16 = 0;

  uint32_t last_interval_cycles = 0;
  int32_t  last_interval_error_cycles = 0;
  uint32_t max_abs_interval_error_cycles = 0;
  uint32_t last_counter_delta_ticks = 0;

  uint32_t anchor_dwt = 0;
  uint32_t anchor_counter32 = 0;
  uint16_t anchor_hardware16 = 0;
  uint32_t anchor_pair_previous_dwt = 0;
  uint32_t anchor_pair_previous_counter32 = 0;

  uint32_t arm_count = 0;
  uint32_t fire_count = 0;
  uint32_t next_target_counter32 = 0;
};

struct interrupt_smartzero_snapshot_t {
  interrupt_smartzero_phase_t phase = interrupt_smartzero_phase_t::IDLE;
  bool running = false;
  bool complete = false;
  bool aborted = false;
  uint32_t sequence = 0;
  uint32_t begin_count = 0;
  uint32_t complete_count = 0;
  uint32_t abort_count = 0;
  uint32_t current_lane_index = 0;
  interrupt_subscriber_kind_t current_lane = interrupt_subscriber_kind_t::NONE;
  uint32_t tolerance_cycles = SMARTZERO_DEFAULT_TOLERANCE_CYCLES;
  uint32_t sample_rate_hz = SMARTZERO_SAMPLE_RATE_HZ;
  uint32_t counter_delta_ticks = SMARTZERO_COUNTER_DELTA_TICKS;
  interrupt_smartzero_lane_snapshot_t lanes[SMARTZERO_LANE_COUNT];
};

bool interrupt_smartzero_begin(void);
void interrupt_smartzero_abort(void);
bool interrupt_smartzero_running(void);
bool interrupt_smartzero_complete(void);

bool interrupt_smartzero_live_snapshot(interrupt_smartzero_snapshot_t* out);

bool interrupt_smartzero_snapshot(interrupt_smartzero_snapshot_t* out);

// ============================================================================
// Reporting-only integrity counters
// ============================================================================
//
// These counters are deliberately passive.  They never repair, re-author,
// reject, delay, or otherwise mutate the clock rails.  They count whether the
// edge facts already authored by the system satisfy draconian invariants made
// reasonable by the stability of the GF-8802/AOCJY1A-derived timing rails.

struct interrupt_integrity_interval_check_t {
  bool     valid = false;
  bool     last_ok = false;
  uint32_t test_count = 0;
  uint32_t ok_count = 0;
  uint32_t bad_count = 0;
  uint32_t skipped_count = 0;

  uint32_t sequence = 0;
  uint32_t gate_cycles = 0;
  bool     pps_interval_valid = false;
  bool     vclock_interval_valid = false;
  uint32_t pps_interval_cycles = 0;
  uint32_t vclock_interval_cycles = 0;
  int32_t  vclock_minus_pps_cycles = 0;
};

struct interrupt_integrity_counter_check_t {
  bool     valid = false;
  bool     last_ok = false;
  uint32_t test_count = 0;
  uint32_t ok_count = 0;
  uint32_t bad_count = 0;
  uint32_t skipped_count = 0;

  uint32_t sequence = 0;
  uint32_t expected_delta_ticks = 0;
  uint32_t observed_delta_ticks = 0;
  int32_t  observed_minus_expected_ticks = 0;
  uint32_t current_counter32 = 0;
  uint32_t previous_counter32 = 0;
};

struct interrupt_integrity_snapshot_t {
  bool     valid = false;
  uint32_t snapshot_count = 0;

  // VCLOCK selected-edge DWT interval must agree with the physical PPS DWT
  // interval within the configured diagnostic gate.
  interrupt_integrity_interval_check_t vclock_pps_interval;

  // Counter32 lineage: each one-second authored edge must advance by exactly
  // the lane's expected 10 MHz tick count.
  interrupt_integrity_counter_check_t vclock_counter;
  interrupt_integrity_counter_check_t ocxo1_counter;
  interrupt_integrity_counter_check_t ocxo2_counter;
};

bool interrupt_integrity_snapshot(interrupt_integrity_snapshot_t* out);

void interrupt_integrity_note_vclock_pps_interval(uint32_t sequence,
                                                  bool pps_interval_valid,
                                                  uint32_t pps_interval_cycles,
                                                  bool vclock_interval_valid,
                                                  uint32_t vclock_interval_cycles);

void interrupt_integrity_note_counter32(interrupt_subscriber_kind_t kind,
                                        uint32_t sequence,
                                        bool interval_valid,
                                        uint32_t observed_delta_ticks,
                                        uint32_t expected_delta_ticks,
                                        uint32_t current_counter32);

static constexpr int32_t VCLOCK_EPOCH_TICK_OFFSET = -2;

struct pps_t {
  uint32_t sequence          = 0;

  uint32_t dwt_at_edge       = 0;

  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge       = 0;
};

struct pps_vclock_t {
  uint32_t sequence          = 0;

  uint32_t dwt_at_edge       = 0;

  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge       = 0;

  int64_t  gnss_ns_at_edge   = -1;
};

// PPS/VCLOCK edge authority courtroom.  process_interrupt publishes the
// selected VCLOCK edge's DWT coordinate as a lawful inference from three
// witnesses: the physical PPS GPIO edge plus learned phase, the VCLOCK/QTimer
// observed edge, and the predictor/lower-envelope candidate.
static constexpr uint32_t PPS_VCLOCK_EDGE_DECISION_NONE = 0;
static constexpr uint32_t PPS_VCLOCK_EDGE_DECISION_LOWER_LAWFUL = 1;
static constexpr uint32_t PPS_VCLOCK_EDGE_DECISION_PREDICTION_FALLBACK = 2;
static constexpr uint32_t PPS_VCLOCK_EDGE_DECISION_PPS_PHASE_FALLBACK = 3;
static constexpr uint32_t PPS_VCLOCK_EDGE_DECISION_OBSERVED_FALLBACK = 4;

static constexpr uint32_t PPS_VCLOCK_EDGE_INVALID_NONE = 0;
static constexpr uint32_t PPS_VCLOCK_EDGE_INVALID_NO_PPS = 1u << 0;
static constexpr uint32_t PPS_VCLOCK_EDGE_INVALID_NO_OBSERVED = 1u << 1;
static constexpr uint32_t PPS_VCLOCK_EDGE_INVALID_NO_PREDICTION = 1u << 2;
static constexpr uint32_t PPS_VCLOCK_EDGE_INVALID_NO_PHASE = 1u << 3;
static constexpr uint32_t PPS_VCLOCK_EDGE_INVALID_AGREEMENT = 1u << 4;
static constexpr uint32_t PPS_VCLOCK_EDGE_INVALID_OBSERVED_EARLY = 1u << 5;

struct pps_vclock_edge_authority_t {
  bool     valid = false;
  uint32_t sequence = 0;
  uint32_t update_count = 0;
  uint32_t reject_count = 0;

  uint32_t authority_dwt_at_edge = 0;
  uint32_t pps_dwt_at_edge = 0;
  uint32_t vclock_observed_dwt_at_edge = 0;
  uint32_t vclock_predicted_dwt_at_edge = 0;
  uint32_t pps_projected_vclock_dwt_at_edge = 0;

  bool     observed_phase_valid = false;
  bool     learned_phase_valid = false;
  uint32_t observed_phase_cycles = 0;
  uint32_t learned_phase_cycles = 0;

  uint32_t gate_cycles = 0;
  uint32_t agreement_span_cycles = 0;
  uint32_t decision = PPS_VCLOCK_EDGE_DECISION_NONE;
  uint32_t invalid_mask = PPS_VCLOCK_EDGE_INVALID_NONE;

  int32_t  authority_minus_pps_cycles = 0;
  int32_t  authority_minus_vclock_observed_cycles = 0;
  int32_t  authority_minus_prediction_cycles = 0;
  int32_t  prediction_minus_pps_projected_cycles = 0;
  int32_t  pps_projected_minus_observed_cycles = 0;
  int32_t  observed_minus_prediction_cycles = 0;

  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge = 0;
  uint32_t dwt_cycles_per_second = 0;
};

struct pps_vclock_phase_estimate_t {
  bool     valid = false;

  uint32_t lattice_dwt_at_edge = 0;
  uint32_t estimated_dwt_at_edge = 0;
  int32_t  correction_cycles = 0;

  uint32_t phase_mod_scaled_cycles = 0;
  uint32_t tick_scaled_cycles = 0;
  uint32_t scale = 0;
  uint32_t dwt_cycles_per_second = 0;

  uint32_t pps_sequence = 0;
  uint32_t pvc_sequence = 0;
  uint32_t pps_dwt_at_edge = 0;
  uint32_t pps_counter32_at_edge = 0;
  uint32_t pvc_counter32_at_edge = 0;
};

struct pps_edge_snapshot_t {
  uint32_t sequence          = 0;

  uint32_t dwt_at_edge       = 0;
  uint32_t dwt_raw_at_edge   = 0;   // legacy alias; carries pvc.dwt_at_edge
  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge       = 0;
  int64_t  gnss_ns_at_edge   = -1;

  uint32_t physical_pps_dwt_raw_at_edge        = 0;   // carries pps.dwt_at_edge
  uint32_t physical_pps_dwt_normalized_at_edge = 0;   // carries pps.dwt_at_edge
  uint32_t physical_pps_counter32_at_read      = 0;
  uint16_t physical_pps_ch3_at_read            = 0;

  bool     spinidle_shadow_valid = false;
  uint32_t spinidle_shadow_dwt = 0;
  uint32_t spinidle_shadow_to_isr_entry_cycles = 0;
  uint32_t spinidle_shadow_valid_threshold_cycles = 0;

  uint32_t vclock_epoch_counter32              = 0;
  uint16_t vclock_epoch_ch3                    = 0;
  uint32_t vclock_epoch_ticks_after_pps        = 0;
  int32_t  vclock_epoch_counter32_offset_ticks = 0;
  int32_t  vclock_epoch_dwt_offset_cycles      = 0;
  bool     vclock_epoch_selected               = false;

  // PPS/VCLOCK edge authority courtroom copied from process_interrupt.
  pps_vclock_edge_authority_t vclock_edge_authority{};
};

struct interrupt_epoch_capture_t {
  bool     valid = false;
  uint32_t sequence = 0;

  uint32_t capture_dwt_start_raw = 0;
  uint32_t capture_dwt_after_vclock_raw = 0;
  uint32_t capture_dwt_end_raw = 0;
  uint32_t capture_window_cycles = 0;
  uint32_t vclock_read_offset_cycles = 0;

  uint32_t vclock_dwt_at_edge = 0;

  bool     vclock_capture_valid = false;
  bool     all_lanes_capture_valid = false;

  uint16_t vclock_hardware16_observed = 0;
  uint16_t vclock_hardware16_selected = 0;
  uint16_t ocxo1_hardware16 = 0;
  uint16_t ocxo2_hardware16 = 0;

  uint32_t vclock_counter32 = 0;
  uint32_t ocxo1_counter32 = 0;
  uint32_t ocxo2_counter32 = 0;
};

bool interrupt_last_epoch_capture(interrupt_epoch_capture_t* out);

void interrupt_ocxo_logical_grid_epoch(uint32_t ocxo1_epoch_counter32,
                                       uint32_t ocxo2_epoch_counter32);

struct interrupt_pps_edge_heartbeat_t {
  uint32_t edge_count = 0;
  uint32_t last_dwt = 0;       // pps_vclock.dwt_at_edge of most recent edge
  int64_t  last_gnss_ns = -1;  // pps_vclock.gnss_ns_at_edge of most recent edge
  uint32_t gpio_irq_count = 0;
  uint32_t gpio_miss_count = 0;
};

interrupt_pps_edge_heartbeat_t interrupt_pps_edge_heartbeat(void);

using interrupt_subscriber_event_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t* diag,
             void* user_data);

struct interrupt_subscription_t {
  interrupt_subscriber_kind_t  kind     = interrupt_subscriber_kind_t::NONE;
  interrupt_subscriber_event_fn on_event = nullptr;
  void*                         user_data = nullptr;
};

using pps_edge_dispatch_fn = void (*)(const pps_edge_snapshot_t& snap);

void process_interrupt_init_hardware(void);
void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start(interrupt_subscriber_kind_t kind);
bool interrupt_stop(interrupt_subscriber_kind_t kind);

void interrupt_request_pps_rebootstrap(void);
bool interrupt_pps_rebootstrap_pending(void);

const interrupt_event_t*        interrupt_last_event(interrupt_subscriber_kind_t kind);
const interrupt_capture_diag_t* interrupt_last_diag (interrupt_subscriber_kind_t kind);

void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn fn);

pps_vclock_t interrupt_last_pps_vclock(void);

void interrupt_pps_vclock_label_anchor(uint32_t sequence,
                                       uint32_t counter32_at_edge,
                                       uint64_t gnss_ns_at_edge,
                                       uint32_t dwt_cycles_per_second);

bool interrupt_last_pps_vclock_phase_estimate(
    pps_vclock_phase_estimate_t* out);

pps_edge_snapshot_t interrupt_last_pps_edge(void);

using interrupt_pps_entry_latency_handler_fn =
    void (*)(uint32_t sequence, uint32_t isr_entry_dwt_raw);

void interrupt_register_pps_entry_latency_handler(
    interrupt_pps_entry_latency_handler_fn cb);

struct interrupt_qtimer1_ch1_compare_event_t {
  uint32_t sequence = 0;
  uint32_t target_counter32 = 0;
  uint32_t counter32_at_event = 0;
  int32_t  counter32_residual_ticks = 0;

  uint32_t isr_entry_dwt_raw = 0;

  uint32_t dwt_at_event = 0;
  int64_t  gnss_ns_at_event = -1;
};

using interrupt_qtimer1_ch1_handler_fn =
    void (*)(const interrupt_qtimer1_ch1_compare_event_t& event);

void interrupt_register_qtimer1_ch1_handler(interrupt_qtimer1_ch1_handler_fn cb);
bool interrupt_qtimer1_ch1_arm_compare(uint32_t target_counter32);
void interrupt_qtimer1_ch1_disable_compare(void);

uint16_t interrupt_qtimer1_ch1_counter_now(void);
uint16_t interrupt_qtimer1_ch1_comp1_now(void);
uint16_t interrupt_qtimer1_ch1_csctrl_now(void);

using interrupt_qtimer1_ch2_handler_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t& diag);

void interrupt_register_qtimer1_ch2_handler(interrupt_qtimer1_ch2_handler_fn cb);

uint32_t interrupt_vclock_counter32_observe_ambient(void);
void     interrupt_qtimer1_ch2_arm_compare(uint32_t target_counter32);

struct interrupt_clock_snapshot_t {
  uint16_t hardware16 = 0;
  uint32_t counter32 = 0;
  uint64_t ns64 = 0;
};

bool interrupt_clock_snapshot(interrupt_subscriber_kind_t kind,
                              interrupt_clock_snapshot_t* out);

uint32_t interrupt_clock32_from_ns(uint64_t ns);
bool     interrupt_clock32_zero_from_ns(interrupt_subscriber_kind_t kind,
                                        uint64_t ns);
bool     interrupt_clock32_request_zero_from_ns(interrupt_subscriber_kind_t kind,
                                                uint64_t ns);

uint16_t interrupt_qtimer1_ch2_counter_now(void);
uint16_t interrupt_qtimer1_ch2_comp1_now(void);
uint16_t interrupt_qtimer1_ch2_csctrl_now(void);

void process_interrupt_gpio6789_irq  (uint32_t isr_entry_dwt_raw);

uint16_t interrupt_qtimer2_ch0_counter_now(void);
uint16_t interrupt_qtimer3_ch0_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);
uint32_t interrupt_qtimer1_counter32_now (void);

uint32_t interrupt_dynamic_cps(void);

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str  (interrupt_provider_kind_t   provider);
const char* interrupt_lane_str           (interrupt_lane_t            lane);
