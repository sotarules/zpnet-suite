// ============================================================================
// process_interrupt.h — observed-edge custody ABI
// ============================================================================
// process_interrupt publishes only raw, latency-adjusted DWT-at-edge facts
// paired with authored compare-target identities.  A mechanical 1 kHz OCXO
// cadence may exercise the compare path, but it contains no alternative endpoint
// estimator or repair authority.
// ============================================================================

#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

// Foreground loop service.  Call once per loop() pass before timepop_dispatch().
// It drains immutable CH2 fire facts, executes application subscriber callbacks,
// and flushes scalar feature state.  No TimePop scheduler mutation or application
// behavior is performed by Priority 0 or Priority 16.
void process_interrupt_foreground_service(void);

// Scalar wake bit set by Priority 16 whenever immutable foreground work exists.
// TimePop's foreground idle witness reads it directly so pending custody yields
// the loop without a cross-translation-unit function call.
extern volatile bool g_process_interrupt_foreground_pending;
static inline bool process_interrupt_foreground_pending(void) {
  return g_process_interrupt_foreground_pending;
}

// Compatibility TimePop callback.  It flushes feature state only and never
// re-enters the CH2/subscriber foreground bridge from inside TimePop dispatch.
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

// Historical publication-court bit assignments retained for wire/source
// compatibility.  The observed-only implementation authors only
// INTERRUPT_DWT_PUBLICATION_VERDICT_OK; all alternative-estimator bits remain
// zero and no publication court can replace an observed edge.
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_OK = 0U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_ZERO_DWT = 1u << 0;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_SOURCE_MISMATCH = 1u << 1;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_COUNTER_LOW16 = 1u << 5;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_SERVICE_OFFSET = 1u << 6;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_COUNTER_DELTA = 1u << 7;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_OBSERVED_INTERVAL = 1u << 8;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_PUBLISHED_INTERVAL = 1u << 9;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_CROSS_RAIL = 1u << 10;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_GNSS_PROJECTION = 1u << 11;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_COUNTER_ADJACENCY = 1u << 12;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_YARDSTICK_EXCURSION = 1u << 13;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_VERDICT_RULER_UNQUALIFIED = 1u << 14;

// Final DWT publication court reason IDs.  These are stable scalar IDs carried
// through diagnostic structs; reports translate them to strings at the final
// Payload emission site so no borrowed const char* crosses interrupt/Alpha/Beta
// custody boundaries.
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_OK = 0U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_ZERO_DWT = 1U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_SOURCE_MISMATCH = 2U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_LOW16 = 6U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_SERVICE_OFFSET = 7U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_DELTA = 8U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_OBSERVED_INTERVAL = 9U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_PUBLISHED_INTERVAL = 10U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_CROSS_RAIL = 11U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_GNSS_PROJECTION = 12U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_ADJACENCY = 13U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_YARDSTICK_EXCURSION = 14U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_RULER_UNQUALIFIED = 15U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_LAUNCH_ACQUISITION_HARD_QUARANTINE = 16U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_STRICT_PUBLICATION_QUARANTINE = 17U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_STARTUP_HARD_QUARANTINE = 18U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_LAUNCH_ACQUISITION_RELAXED_DIAGNOSTIC = 19U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_STARTUP_RELAXED_DIAGNOSTIC = 20U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_SIDE_RAIL_DIAGNOSTIC = 21U;
static constexpr uint32_t INTERRUPT_DWT_PUBLICATION_REASON_UNKNOWN = 22U;


static inline uint32_t interrupt_dwt_publication_reason_id_from_verdict_mask(
    uint32_t verdict_mask) {
  if (verdict_mask == INTERRUPT_DWT_PUBLICATION_VERDICT_OK) {
    return INTERRUPT_DWT_PUBLICATION_REASON_OK;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_ZERO_DWT) {
    return INTERRUPT_DWT_PUBLICATION_REASON_ZERO_DWT;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_SOURCE_MISMATCH) {
    return INTERRUPT_DWT_PUBLICATION_REASON_SOURCE_MISMATCH;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_COUNTER_LOW16) {
    return INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_LOW16;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_SERVICE_OFFSET) {
    return INTERRUPT_DWT_PUBLICATION_REASON_SERVICE_OFFSET;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_COUNTER_DELTA) {
    return INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_DELTA;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_OBSERVED_INTERVAL) {
    return INTERRUPT_DWT_PUBLICATION_REASON_OBSERVED_INTERVAL;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_PUBLISHED_INTERVAL) {
    return INTERRUPT_DWT_PUBLICATION_REASON_PUBLISHED_INTERVAL;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_CROSS_RAIL) {
    return INTERRUPT_DWT_PUBLICATION_REASON_CROSS_RAIL;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_GNSS_PROJECTION) {
    return INTERRUPT_DWT_PUBLICATION_REASON_GNSS_PROJECTION;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_COUNTER_ADJACENCY) {
    return INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_ADJACENCY;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_YARDSTICK_EXCURSION) {
    return INTERRUPT_DWT_PUBLICATION_REASON_YARDSTICK_EXCURSION;
  }
  if (verdict_mask & INTERRUPT_DWT_PUBLICATION_VERDICT_RULER_UNQUALIFIED) {
    return INTERRUPT_DWT_PUBLICATION_REASON_RULER_UNQUALIFIED;
  }
  return INTERRUPT_DWT_PUBLICATION_REASON_UNKNOWN;
}

static inline const char* interrupt_dwt_publication_reason_name(
    uint32_t reason_id) {
  switch (reason_id) {
    case INTERRUPT_DWT_PUBLICATION_REASON_OK:
      return "ok";
    case INTERRUPT_DWT_PUBLICATION_REASON_ZERO_DWT:
      return "zero_dwt";
    case INTERRUPT_DWT_PUBLICATION_REASON_SOURCE_MISMATCH:
      return "source_mismatch";
    case INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_LOW16:
      return "counter_low16";
    case INTERRUPT_DWT_PUBLICATION_REASON_SERVICE_OFFSET:
      return "service_offset";
    case INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_DELTA:
      return "counter_delta";
    case INTERRUPT_DWT_PUBLICATION_REASON_OBSERVED_INTERVAL:
      return "observed_interval";
    case INTERRUPT_DWT_PUBLICATION_REASON_PUBLISHED_INTERVAL:
      return "published_interval";
    case INTERRUPT_DWT_PUBLICATION_REASON_CROSS_RAIL:
      return "cross_rail";
    case INTERRUPT_DWT_PUBLICATION_REASON_GNSS_PROJECTION:
      return "gnss_projection";
    case INTERRUPT_DWT_PUBLICATION_REASON_COUNTER_ADJACENCY:
      return "counter_adjacency";
    case INTERRUPT_DWT_PUBLICATION_REASON_YARDSTICK_EXCURSION:
      return "yardstick_excursion";
    case INTERRUPT_DWT_PUBLICATION_REASON_RULER_UNQUALIFIED:
      return "ruler_unqualified";
    case INTERRUPT_DWT_PUBLICATION_REASON_LAUNCH_ACQUISITION_HARD_QUARANTINE:
      return "launch_acquisition_hard_quarantine";
    case INTERRUPT_DWT_PUBLICATION_REASON_STRICT_PUBLICATION_QUARANTINE:
      return "strict_publication_quarantine";
    case INTERRUPT_DWT_PUBLICATION_REASON_STARTUP_HARD_QUARANTINE:
      return "startup_hard_quarantine";
    case INTERRUPT_DWT_PUBLICATION_REASON_LAUNCH_ACQUISITION_RELAXED_DIAGNOSTIC:
      return "launch_acquisition_relaxed_diagnostic";
    case INTERRUPT_DWT_PUBLICATION_REASON_STARTUP_RELAXED_DIAGNOSTIC:
      return "startup_relaxed_diagnostic";
    case INTERRUPT_DWT_PUBLICATION_REASON_SIDE_RAIL_DIAGNOSTIC:
      return "side_rail_diagnostic";
    default:
      return "unknown";
  }
}


struct interrupt_event_t {
  interrupt_subscriber_kind_t kind     = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t   provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t            lane     = interrupt_lane_t::NONE;
  interrupt_event_status_t    status   = interrupt_event_status_t::OK;

  uint32_t dwt_at_event = 0;

  uint64_t gnss_ns_at_event = 0;

  uint32_t counter32_at_event = 0;

  // Physical PPS row that owns this event.  VCLOCK and the first OCXO
  // one-second edges after PPS carry the same sequence so CLOCKS can defer
  // publication without inferring freshness from callback order.
  uint32_t pps_sequence = 0;

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

  // Compatibility surface.  Observed edges are always published directly;
  // verdict_mask is always OK and no candidate can repair or replace DWT.
  uint32_t dwt_publication_verdict_mask = 0;
  uint32_t dwt_publication_verdict_reason_id = INTERRUPT_DWT_PUBLICATION_REASON_OK;
  uint32_t dwt_publication_watchdog_count = 0;
  uint32_t dwt_publication_gate_cycles = 0;
  uint32_t dwt_publication_cross_rail_gate_cycles = 0;
  uint32_t dwt_publication_service_offset_gate_ticks = 0;
  uint32_t dwt_publication_expected_counter_delta_ticks = 0;
  uint32_t dwt_publication_observed_counter_delta_ticks = 0;
  uint32_t dwt_publication_expected_interval_cycles = 0;
  uint32_t dwt_publication_published_interval_cycles = 0;
  uint32_t dwt_publication_observed_interval_cycles = 0;
  int32_t  dwt_publication_published_interval_error_cycles = 0;
  int32_t  dwt_publication_observed_interval_error_cycles = 0;
  int32_t  dwt_publication_published_minus_observed_cycles = 0;
  int32_t  dwt_publication_service_offset_signed_ticks = 0;
  int64_t  dwt_publication_vclock_gnss_error_ns = 0;

  bool     dwt_interval_gate_valid = false;
  bool     dwt_interval_sample_accepted = false;
  bool     dwt_interval_sample_rejected = false;
  bool     dwt_interval_ema_updated = false;  // retired predictor compatibility
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
  uint32_t dwt_ema_dwt_at_event = 0;  // retired predictor compatibility
  uint32_t dwt_yardstick_auth_endpoint_dwt = 0;
  uint32_t dwt_yardstick_auth_endpoint_frac_q16 = 0;
  int32_t  dwt_yardstick_auth_error_cycles = 0;
  bool     dwt_yardstick_auth_anchor_applied = false;


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
static constexpr uint32_t SMARTZERO_SAMPLE_RATE_HZ = 1;
static constexpr uint32_t SMARTZERO_COUNTER_DELTA_TICKS = 10000000;
static constexpr int32_t  SMARTZERO_DEFAULT_TOLERANCE_CYCLES = 32;

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

// RECOVER publication-custody reset.
//
// CLOCKS/Alpha deliberately cuts OCXO measurement custody during RECOVER so no
// previous/pending OCXO edge can bridge an outage.  process_interrupt owns an
// independent DWT-publication courtroom with per-lane previous-publication and
// cross-rail memory; that courtroom must be cut at the same boundary or the
// first post-recovery OCXO event can be judged against stale pre-recovery
// evidence and trip a false WATCHDOG_ANOMALY.
void interrupt_recover_reset_publication_custody(void);
uint32_t interrupt_recover_publication_custody_reset_count(void);

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
  bool     last_sample_counted = false;
  uint32_t test_count = 0;
  uint32_t ok_count = 0;
  uint32_t bad_count = 0;
  uint32_t skipped_count = 0;

  // Recovery boundary.  Lifetime bad_count is diagnostic evidence; launch
  // readiness is based on a current clean streak after the lane has locked.
  bool     locked = false;
  uint32_t lock_sequence = 0;
  uint32_t lock_count = 0;
  uint32_t lock_streak_required = 0;
  uint32_t consecutive_ok_count = 0;
  uint32_t prelock_ok_count = 0;
  uint32_t prelock_bad_count = 0;
  uint32_t post_lock_ok_count = 0;
  uint32_t post_lock_bad_count = 0;
  uint32_t first_bad_sequence = 0;
  uint32_t last_bad_sequence = 0;
  uint32_t last_bad_observed_delta_ticks = 0;
  uint32_t last_bad_expected_delta_ticks = 0;
  int32_t  last_bad_observed_minus_expected_ticks = 0;

  uint32_t sequence = 0;
  uint32_t expected_delta_ticks = 0;
  uint32_t observed_delta_ticks = 0;
  int32_t  observed_minus_expected_ticks = 0;
  uint32_t current_counter32 = 0;
  uint32_t previous_counter32 = 0;
};

struct interrupt_integrity_gnss_ns_check_t {
  bool     valid = false;
  bool     computed_valid = false;
  bool     last_ok = false;
  uint32_t test_count = 0;
  uint32_t match_count = 0;       // within gate_ns
  uint32_t exact_match_count = 0; // exactly 0 ns error
  uint32_t mismatch_count = 0;
  uint32_t skipped_count = 0;

  int64_t  gate_ns = 0;
  uint32_t sequence = 0;
  uint32_t dwt_at_edge = 0;
  uint32_t counter32_at_edge = 0;
  int64_t  computed_gnss_ns = -1;
  int64_t  expected_gnss_ns = -1;
  int64_t  error_ns = 0;
  int64_t  anchor_sequence_delta = 0;

  uint32_t anchor_sequence_used = 0;
  uint32_t anchor_age_slots = 0;
  uint32_t anchor_selection_kind = 0;
  uint32_t anchor_dwt_at_edge = 0;
  int64_t  anchor_gnss_ns_at_edge = -1;
  uint32_t anchor_cps = 0;
  uint64_t anchor_ns_delta = 0;
  uint32_t anchor_failure_mask = 0;
};

struct interrupt_integrity_qtimer_cntr_match_check_t {
  bool     valid = false;
  bool     last_ok = false;
  uint32_t test_count = 0;
  uint32_t match_count = 0;       // delta_signed_ticks == expected_offset_ticks
  uint32_t mismatch_count = 0;

  // Lock boundary: startup/pre-grid transients remain in lifetime/prelock
  // accounting, while post_lock_* answers whether the invariant has ever
  // failed after a lane produced a sustained run of clean observations.
  bool     locked = false;
  uint32_t lock_sequence = 0;
  uint32_t lock_count = 0;
  uint32_t lock_streak_required = 0;
  uint32_t consecutive_ok_count = 0;
  uint32_t prelock_match_count = 0;
  uint32_t prelock_mismatch_count = 0;
  uint32_t post_lock_match_count = 0;
  uint32_t post_lock_mismatch_count = 0;
  uint32_t first_mismatch_sequence = 0;
  uint32_t last_mismatch_sequence = 0;
  int32_t  last_mismatch_delta_signed_ticks = 0;
  int32_t  last_mismatch_observed_minus_expected_offset_ticks = 0;

  uint32_t sequence = 0;
  uint32_t target_counter32 = 0;
  uint16_t expected_low16 = 0;
  uint16_t ambient_low16 = 0;
  int32_t  expected_offset_ticks = 0;
  uint32_t delta_mod65536_ticks = 0;
  int32_t  delta_signed_ticks = 0;
  int32_t  observed_minus_expected_offset_ticks = 0;
  uint32_t abs_delta_ticks = 0;
  uint32_t isr_entry_dwt_raw = 0;
};


struct interrupt_integrity_qtimer_dwt_interval_check_t {
  bool     valid = false;
  bool     previous_valid = false;
  bool     ruler_qualified = false;
  uint32_t ruler_wait_count = 0;
  uint32_t first_qualified_sequence = 0;
  uint32_t ruler_qualification_count = 0;
  bool     last_ok = false;
  uint32_t test_count = 0;
  uint32_t match_count = 0;       // |error_cycles| <= gate_cycles
  uint32_t mismatch_count = 0;
  uint32_t skipped_count = 0;
  uint32_t too_short_count = 0;   // observed_cycles < expected_cycles - gate
  uint32_t too_long_count = 0;    // observed_cycles > expected_cycles + gate

  bool     locked = false;
  uint32_t lock_sequence = 0;
  uint32_t lock_count = 0;
  uint32_t lock_streak_required = 0;
  uint32_t consecutive_ok_count = 0;
  uint32_t prelock_match_count = 0;
  uint32_t prelock_mismatch_count = 0;
  uint32_t post_lock_match_count = 0;
  uint32_t post_lock_mismatch_count = 0;

  uint32_t first_mismatch_sequence = 0;
  uint32_t last_mismatch_sequence = 0;
  int32_t  last_mismatch_error_cycles = 0;
  uint32_t last_mismatch_observed_cycles = 0;
  uint32_t last_mismatch_expected_cycles = 0;

  uint32_t sequence = 0;
  uint32_t target_counter32 = 0;
  uint32_t dwt_at_match = 0;

  // Previous accepted match used to form the most recent interval.
  uint32_t previous_sequence = 0;
  uint32_t previous_target_counter32 = 0;
  uint32_t previous_dwt_at_match = 0;

  // Report copy of the previous endpoint used for the most recent interval.
  uint32_t interval_previous_sequence = 0;
  uint32_t interval_previous_target_counter32 = 0;
  uint32_t interval_previous_dwt_at_match = 0;

  uint32_t target_delta_ticks = 0;
  uint32_t observed_cycles = 0;
  uint32_t expected_cycles = 0;
  int32_t  error_cycles = 0;      // observed_cycles - expected_cycles
  uint32_t abs_error_cycles = 0;
  uint32_t gate_cycles = 0;
};

struct interrupt_integrity_qtimer_dwt_match_check_t {
  bool     valid = false;
  uint32_t sequence = 0;
  uint32_t target_counter32 = 0;
  uint32_t dwt_at_match = 0;
  bool     one_second_boundary = false;

  interrupt_integrity_qtimer_dwt_interval_check_t hz1k;
  interrupt_integrity_qtimer_dwt_interval_check_t one_second;
};

struct interrupt_integrity_snapshot_t {
  bool     valid = false;
  uint32_t snapshot_count = 0;

  // VCLOCK selected-edge DWT -> GNSS ns projection must reconstruct the
  // CLOCKS-labeled PPS/VCLOCK GNSS timeline.  The expected value is anchored
  // to the labeled anchor sequence, not to process_interrupt's local sequence
  // zero, because CLOCKS owns the campaign GNSS label epoch.
  interrupt_integrity_gnss_ns_check_t vclock_gnss_ns;

  // Immediate QuadTimer CNTR read after ISR-entry DWT must observe the
  // compare target plus the deterministic hardware/CPU read offset.  This
  // checks compare-service timing/custody without changing the compare-
  // generation gate or publishing policy.
  interrupt_integrity_qtimer_cntr_match_check_t vclock_qtimer_cntr;
  interrupt_integrity_qtimer_cntr_match_check_t ocxo1_qtimer_cntr;
  interrupt_integrity_qtimer_cntr_match_check_t ocxo2_qtimer_cntr;

  // Raw first-instruction DWT intervals between accepted compare teeth.  The
  // 1 kHz member is reserved for a later analysis pass; the initial cadence
  // experiment exercises every tooth but authors only the one-second rail.
  interrupt_integrity_qtimer_dwt_match_check_t vclock_qtimer_dwt;
  interrupt_integrity_qtimer_dwt_match_check_t ocxo1_qtimer_dwt;
  interrupt_integrity_qtimer_dwt_match_check_t ocxo2_qtimer_dwt;

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
// selected VCLOCK edge's observed QTimer DWT coordinate so VCLOCK and OCXO
// subscriber intervals share one measured-edge species. The physical PPS GPIO
// edge plus the learned PPS->VCLOCK lower-phase estimate remain diagnostic
// witnesses.
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

  uint32_t authority_dwt_at_edge = 0;       // published observed VCLOCK DWT
  uint32_t pps_dwt_at_edge = 0;
  uint32_t vclock_observed_dwt_at_edge = 0; // raw observed witness / authority
  uint32_t vclock_predicted_dwt_at_edge = 0;
  uint32_t pps_projected_vclock_dwt_at_edge = 0; // PPS+learned-phase witness

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
  bool     ocxo1_capture_valid = false;
  bool     ocxo2_capture_valid = false;
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

// Idempotent recovery service assertion.  Unlike interrupt_start(), this leaves
// healthy subscriber, VCLOCK-anchor, observed cadence, and publication
// custody untouched.  It performs a narrow restart only when the requested
// runtime or lane-local cadence is actually inactive.
bool interrupt_ensure_service(interrupt_subscriber_kind_t kind);

// RECOVER-only OCXO observed-edge rebootstrap.  Clears stale capture and
// deferred-dispatch custody, preserves the installed logical clock zero, and
// rebuilds a fresh wrap-proof 1 kHz target/tooth grid from the live hardware
// coordinate.  This guarantees that a one-second boundary remains reachable
// even when the prior 32-bit target coordinate wrapped or lost grid phase.
// Does not touch VCLOCK, SmartZero proof state, the OCXO logical zero, or DACs.
bool interrupt_recover_rebootstrap_ocxo_service(
    interrupt_subscriber_kind_t kind);

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

// CLOCKS/Beta brackets the private START prologue with this launch-acquisition
// window. While active, process_interrupt may pass non-poisonous DWT
// publication facts to Alpha so OCXO public-origin/projection evidence can
// form, but it still quarantines impossible source/DWT facts. Once PPS1 is
// released, Beta ends the window and the publication court returns to strict
// campaign law.
void interrupt_dwt_publication_launch_acquisition_begin(void);
void interrupt_dwt_publication_launch_acquisition_end(void);
bool interrupt_dwt_publication_launch_acquisition_active(void);

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

uint32_t interrupt_vclock_counter32_observe_ambient(void);

// Request the next TimePop CH2 deadline.  process_interrupt separates requested,
// deferred, and physically armed identities.  On the shared QTimer1 vector, a
// CH2 status flag is accepted only when TCF1EN, software arm custody, programmed
// COMP1 identity, and the no-outstanding-fact invariant all agree.
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
