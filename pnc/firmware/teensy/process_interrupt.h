// ============================================================================
// process_interrupt.h — simplified interrupt normalization shell
// ============================================================================
//
// process_interrupt is the sole authority for hardware interrupt custody.
//
// Current doctrine:
//
//   • PPS/DWT is sacred.
//   • DWT captured at PPS ISR entry is the authoritative PPS-side anchor.
//   • OCXO events are observed clocks, not consummated clocks.
//   • No pre-spin / spin-dry occurs here.
//   • PPS is still reported as a direct ISR-entry anchor.
//   • OCXO one-second tempo is now derived from perpetual 1 ms cadence buckets:
//       1000 adjacent windows per second, accumulated in DWT cycles and
//       GNSS nanoseconds, so shared-boundary latency largely cancels in the sum.
//   • Only the initial OCXO cadence seed is granted raw DWT authority.
//       After that, each lawful one-second OCXO edge is synthetic:
//       previous synthetic edge + bucketed one-second observation.
//   • Raw ISR-entry DWT/GNSS facts remain diagnostic only for OCXO lanes.
//       They no longer re-found the next second.
//   • TimePop owns VCLOCK scheduling and Spin-Dry for its own compare path.
//
// Notes:
//   The large interrupt_capture_diag_t surface is preserved for compatibility
//   with existing clocks/reporting code. Most legacy pre-spin fields now remain
//   zeroed or false.
//
// ============================================================================

#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*);

// ============================================================================
// Subscriber identities
// ============================================================================

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  PPS,
  OCXO1,
  OCXO2,
  TIME_TEST,
};

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  QTIMER3,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  QTIMER3_CH2_COMP,
  QTIMER3_CH3_COMP,
  GPIO_EDGE,
};

enum class interrupt_event_status_t : uint8_t {
  OK = 0,
  HOLD,
  FAULT,
};

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_event_status_t status = interrupt_event_status_t::OK;

  // PPS: first-instruction DWT capture remains the canonical PPS-side anchor.
  // OCXO: best synthetic estimate for the lawful one-second edge.
  uint32_t dwt_at_event = 0;

  // PPS: canonical GNSS nanosecond at the PPS edge.
  // OCXO: best synthetic GNSS nanosecond at the lawful one-second edge.
  uint64_t gnss_ns_at_event = 0;

  // PPS: raw 32-bit QTimer1 VCLOCK at edge.
  // OCXO: software-extended logical 32-bit count at the lawful one-second edge.
  uint32_t counter32_at_event = 0;

  // Retained for compatibility. Always zero in this simplified model.
  uint32_t dwt_event_correction_cycles = 0;
};

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  uint64_t prespin_target_gnss_ns = 0;
  uint64_t event_target_gnss_ns = 0;

  bool prespin_active = false;
  bool prespin_fired = false;
  bool prespin_timed_out = false;

  uint32_t shadow_dwt = 0;
  uint32_t dwt_isr_entry_raw = 0;
  int64_t  dwt_isr_entry_gnss_ns = -1;
  int64_t  dwt_isr_entry_minus_event_ns = 0;
  uint32_t approach_cycles = 0;

  uint32_t dwt_at_event = 0;
  uint32_t dwt_at_event_adjusted = 0;

  uint64_t gnss_ns_at_event_raw = 0;
  uint64_t gnss_ns_at_event_final = 0;
  uint64_t gnss_ns_at_event = 0;
  int64_t  gnss_ns_at_event_delta = 0;

  uint32_t counter32_at_event = 0;
  uint32_t dwt_event_correction_cycles = 0;

  bool bridge_valid = false;
  bool bridge_within_tolerance = false;
  bool bridge_skipped_invalid = false;
  bool bridge_used_prediction = false;

  uint64_t gnss_ns_at_event_bridge = 0;
  uint64_t bridge_gnss_ns_raw = 0;
  uint64_t bridge_gnss_ns_target = 0;
  uint64_t bridge_gnss_ns_final = 0;
  int64_t  bridge_raw_error_ns = 0;
  int64_t  bridge_final_error_ns = 0;

  uint32_t prespin_arm_count = 0;
  uint32_t prespin_complete_count = 0;
  uint32_t prespin_timeout_count = 0;
  uint32_t anomaly_count = 0;

  // OCXO lane instrumentation: compare that actually fired vs counter captured at IRQ.
  uint16_t counter16_at_irq = 0;
  uint16_t compare16_fired = 0;
  uint16_t compare16_next_programmed = 0;
  int32_t  counter16_minus_compare_ticks = 0;
  int64_t  counter16_minus_compare_ns = 0;

  // OCXO 1 ms bucket integrator diagnostics.
  uint32_t ocxo_bucket_interval_counts = 0;
  uint32_t ocxo_current_window_bucket_count = 0;
  uint32_t ocxo_last_second_bucket_count = 0;
  uint32_t ocxo_last_bucket_cycles = 0;
  int64_t  ocxo_last_bucket_gnss_ns = 0;
  uint64_t ocxo_current_window_cycles_sum = 0;
  int64_t  ocxo_current_window_gnss_ns_sum = 0;
  uint64_t ocxo_second_cycles_observed = 0;
  uint64_t ocxo_second_cycles_prediction = 0;
  int64_t  ocxo_second_cycles_prediction_error = 0;
  int64_t  ocxo_second_gnss_ns_observed = 0;
  int64_t  ocxo_second_gnss_ns_prediction = 0;
  int64_t  ocxo_second_gnss_ns_prediction_error = 0;
  int64_t  ocxo_second_residual_ns = 0;
  int64_t  ocxo_second_start_gnss_ns_raw = -1;
  int64_t  ocxo_second_end_gnss_ns_raw = -1;
  uint32_t ocxo_second_start_dwt_raw = 0;
  uint32_t ocxo_second_end_dwt_raw = 0;

  // Synthetic one-second ledger carried forward perpetually from the prior
  // synthetic edge, rather than re-founded from the latest raw ISR sample.
  int64_t  ocxo_second_start_gnss_ns_final = -1;
  int64_t  ocxo_second_end_gnss_ns_final = -1;
  uint32_t ocxo_second_start_dwt_final = 0;
  uint32_t ocxo_second_end_dwt_final = 0;

  // Raw-minus-synthetic forensic deltas at the second boundaries.
  int64_t  ocxo_second_start_raw_minus_final_ns = 0;
  int64_t  ocxo_second_end_raw_minus_final_ns = 0;
};

using interrupt_subscriber_event_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t* diag,
             void* user_data);

struct interrupt_subscription_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_subscriber_event_fn on_event = nullptr;
  void* user_data = nullptr;
};

void process_interrupt_init_hardware(void);
void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start(interrupt_subscriber_kind_t kind);
bool interrupt_stop(interrupt_subscriber_kind_t kind);

void interrupt_request_pps_zero(void);
bool interrupt_pps_zero_pending(void);

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind);
const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind);

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_isr_entry_raw);

uint16_t interrupt_qtimer3_ch2_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);

uint32_t interrupt_qtimer1_counter32_now(void);
