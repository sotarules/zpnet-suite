// ============================================================================
// process_interrupt.cpp — simplified interrupt normalization shell
// ============================================================================
//
// This file intentionally stops trying to solve interrupt latency.
// PPS/DWT is the yardstick; VCLOCK/TimePop owns Spin-Dry for scheduler truth.
// Here we simply capture ISR-entry DWT, normalize to GNSS nanoseconds, and
// forward lawful event facts to subscribers.
//
// OCXO QTimer3 channels remain 16-bit cadence lanes, but one-second OCXO
// tempo is now estimated from perpetual 1000×1 ms cadence buckets.  The
// one-second observation is formed from adjacent bucket intervals, and each
// lawful OCXO second edge after the initial seed is carried forward
// synthetically from the prior synthetic edge plus the observed one-second sum.
//
// ============================================================================

#include "process_interrupt.h"

#include "config.h"
#include "debug.h"
#include "process.h"
#include "payload.h"
#include "time.h"
#include "timepop.h"

#include <Arduino.h>
#include "imxrt.h"

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;
static constexpr uint64_t NS_PER_SECOND_U64 = 1000000000ULL;

// One second at 10 MHz = 10,000,000 counts.
// We now observe OCXO tempo as 1000 adjacent 1 ms windows, each 10,000 counts.
static constexpr uint32_t OCXO_COUNTS_PER_SECOND   = 10000000U;
static constexpr uint32_t OCXO_BUCKET_COUNTS       = 10000U;     // 1 ms at 10 MHz
static constexpr uint32_t OCXO_BUCKETS_PER_SECOND  = OCXO_COUNTS_PER_SECOND / OCXO_BUCKET_COUNTS;
static constexpr uint32_t QTIMER16_MODULUS         = 65536U;
static constexpr int64_t  OCXO_PREDICTION_TOLERANCE_NS = 5000LL;

static_assert((OCXO_BUCKETS_PER_SECOND * OCXO_BUCKET_COUNTS) == OCXO_COUNTS_PER_SECOND,
              "Unexpected OCXO bucket decomposition");

static inline uint32_t ns_to_dwt_cycles_runtime(uint64_t ns) {
  const uint32_t f = F_CPU_ACTUAL ? F_CPU_ACTUAL : 1008000000U;
  return (uint32_t)(((uint64_t)f * ns + 500000000ULL) / 1000000000ULL);
}

// ============================================================================
// Pin assignments
// ============================================================================

static constexpr int OCXO1_PIN = 14;
static constexpr int OCXO2_PIN = 15;

// ============================================================================
// Runtime types
// ============================================================================

enum class qtimer_clock_kind_t : uint8_t {
  NONE = 0,
  VCLOCK,
  OCXO1,
  OCXO2,
};

struct qtimer16_ocxo_hw_t {
  IMXRT_TMR_t* module = nullptr;
  uint8_t      channel = 0;
  IRQ_NUMBER_t irq = IRQ_QTIMER3;
  int          input_pin = -1;
  uint8_t      pcs = 0;
};

struct qtimer16_ocxo_runtime_t {
  qtimer_clock_kind_t kind = qtimer_clock_kind_t::NONE;
  const char*         name = nullptr;
  qtimer16_ocxo_hw_t  hw {};

  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;

  // Current 1 ms cadence target (low16 compare).
  uint16_t second_target_low16 = 0;
  uint32_t logical_count32_at_last_second_edge = 0;
  uint16_t last_counter16_at_irq = 0;

  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t miss_count = 0;
  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t event_count = 0;

  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
  uint32_t cadence_hits_since_second = 0;
  uint16_t last_programmed_compare = 0;      // next programmed compare after latest rearm
  uint16_t last_fired_compare16 = 0;         // compare value that actually fired on latest IRQ
  int32_t  last_fired_compare_delta_ticks = 0;
  int64_t  last_fired_compare_delta_ns = 0;
  uint16_t last_initial_counter16 = 0;
  uint16_t last_second_target_low16 = 0;

  // 1 ms bucket integrator state.
  bool     second_window_valid = false;
  uint32_t current_window_bucket_count = 0;

  // Continuous corrected cadence chain used to measure the lawful adjacent
  // 1 ms intervals. These are the authoritative bucket values.
  uint32_t previous_bucket_dwt = 0;
  int64_t  previous_bucket_gnss_ns = -1;

  // Continuous raw cadence chain retained for diagnostics only.
  uint32_t previous_bucket_dwt_raw = 0;
  int64_t  previous_bucket_gnss_ns_raw = -1;

  // Raw first bucket delimiter of the current one-second accumulation window.
  uint32_t second_start_dwt_raw = 0;
  int64_t  second_start_gnss_ns_raw = -1;

  // Synthetic lawful beginning of the current one-second OCXO window.
  uint32_t second_start_dwt_final = 0;
  int64_t  second_start_gnss_ns_final = -1;

  // Canonical corrected bucket/window sums.
  uint64_t current_window_cycles_sum = 0;
  int64_t  current_window_gnss_ns_sum = 0;
  uint32_t last_bucket_cycles = 0;
  int64_t  last_bucket_gnss_ns = 0;

  // Raw bucket/window sums retained for diagnostics only.
  uint64_t current_window_cycles_sum_raw = 0;
  int64_t  current_window_gnss_ns_sum_raw = 0;
  uint32_t last_bucket_cycles_raw = 0;
  int64_t  last_bucket_gnss_ns_raw = 0;

  // Completed one-second observation.
  uint32_t last_second_bucket_count = 0;

  // Canonical authoritative one-second values: synthetic final boundary delta.
  uint64_t last_second_cycles_observed = 0;
  int64_t  last_second_gnss_ns_observed = 0;

  // Raw one-second values retained for diagnostics only.
  uint64_t last_second_cycles_observed_raw = 0;
  int64_t  last_second_gnss_ns_observed_raw = 0;
  uint64_t last_second_cycles_prediction = 0;
  int64_t  last_second_gnss_ns_prediction = 0;
  int64_t  last_second_cycles_prediction_error = 0;
  int64_t  last_second_gnss_ns_prediction_error = 0;
  int64_t  last_second_residual_ns = 0;
  uint32_t last_second_start_dwt_raw = 0;
  uint32_t last_second_end_dwt_raw = 0;
  int64_t  last_second_start_gnss_ns_raw = -1;
  int64_t  last_second_end_gnss_ns_raw = -1;

  uint32_t last_second_start_dwt_final = 0;
  uint32_t last_second_end_dwt_final = 0;
  int64_t  last_second_start_gnss_ns_final = -1;
  int64_t  last_second_end_gnss_ns_final = -1;
  int64_t  last_second_start_raw_minus_final_ns = 0;
  int64_t  last_second_end_raw_minus_final_ns = 0;

  // Rolling next-second prediction ("last is best").
  bool     next_second_prediction_valid = false;
  uint64_t next_second_cycles_prediction = 0;
  int64_t  next_second_gnss_ns_prediction = 0;

  // Raw observed second-end edge vs synthetic lawful second-end edge.
  // Raw = ISR-entry sample. Final = best estimate of the actual compare edge
  // after cadence-latency correction and perpetual bucket carry-forward.
  uint32_t last_event_dwt_raw = 0;
  uint32_t last_event_dwt_final = 0;
  int64_t  last_event_gnss_ns_raw = -1;
  int64_t  last_event_gnss_ns_final = -1;
  uint32_t last_event_target_dwt = 0;
  int64_t  last_event_target_gnss_ns = -1;
};

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;

  qtimer_clock_kind_t clock_kind = qtimer_clock_kind_t::NONE;
};

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub {};

  bool subscribed = false;
  bool active = false;

  interrupt_event_t last_event {};
  interrupt_capture_diag_t last_diag {};
  bool has_fired = false;

  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t event_count = 0;
};

struct interrupt_provider_diag_t {
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t miss_count = 0;
};

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS] = {};
static uint32_t g_subscriber_count = 0;

static bool g_interrupt_hw_ready = false;
static bool g_interrupt_runtime_ready = false;
static bool g_interrupt_irqs_enabled = false;
static volatile bool g_pps_zero_pending = false;

static interrupt_provider_diag_t g_gpio_diag = {};

static interrupt_subscriber_runtime_t* g_rt_pps   = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo1 = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2 = nullptr;

static qtimer16_ocxo_runtime_t g_clock_ocxo1 = {};
static qtimer16_ocxo_runtime_t g_clock_ocxo2 = {};

static const interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  { interrupt_subscriber_kind_t::PPS,   "PPS",   interrupt_provider_kind_t::GPIO6789, interrupt_lane_t::GPIO_EDGE,        qtimer_clock_kind_t::VCLOCK },
  { interrupt_subscriber_kind_t::OCXO1, "OCXO1", interrupt_provider_kind_t::QTIMER3,  interrupt_lane_t::QTIMER3_CH2_COMP, qtimer_clock_kind_t::OCXO1  },
  { interrupt_subscriber_kind_t::OCXO2, "OCXO2", interrupt_provider_kind_t::QTIMER3,  interrupt_lane_t::QTIMER3_CH3_COMP, qtimer_clock_kind_t::OCXO2  },
};

// ============================================================================
// Helper strings
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::PPS:       return "PPS";
    case interrupt_subscriber_kind_t::OCXO1:     return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:     return "OCXO2";
    case interrupt_subscriber_kind_t::TIME_TEST: return "TIME_TEST";
    default:                                     return "NONE";
  }
}

const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider) {
  switch (provider) {
    case interrupt_provider_kind_t::QTIMER3:  return "QTIMER3";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default:                                  return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::QTIMER3_CH2_COMP: return "QTIMER3_CH2_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}

// ============================================================================
// Counter helpers
// ============================================================================

static uint32_t qtimer1_read_32_for_pps_capture(void) {
  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;

  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi2 = IMXRT_TMR1.CH[1].HOLD;

  if (hi1 != hi2) {
    return ((uint32_t)hi2 << 16) | (uint32_t)lo2;
  }
  return ((uint32_t)hi1 << 16) | (uint32_t)lo1;
}

uint32_t interrupt_qtimer1_counter32_now(void) { return qtimer1_read_32_for_pps_capture(); }
uint16_t interrupt_qtimer3_ch2_counter_now(void) { return IMXRT_TMR3.CH[2].CNTR; }
uint16_t interrupt_qtimer3_ch3_counter_now(void) { return IMXRT_TMR3.CH[3].CNTR; }

static inline void qtimer3_clear_compare_flag(uint8_t ch) {
  IMXRT_TMR3.CH[ch].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void qtimer3_program_compare(uint8_t ch, uint16_t target_low16) {
  qtimer3_clear_compare_flag(ch);
  IMXRT_TMR3.CH[ch].COMP1  = target_low16;
  IMXRT_TMR3.CH[ch].CMPLD1 = target_low16;
  qtimer3_clear_compare_flag(ch);
  IMXRT_TMR3.CH[ch].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer3_disable_compare(uint8_t ch) {
  IMXRT_TMR3.CH[ch].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer3_clear_compare_flag(ch);
}

// ============================================================================
// Runtime lookup
// ============================================================================

static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    if (g_subscribers[i].desc && g_subscribers[i].desc->kind == kind) return &g_subscribers[i];
  }
  return nullptr;
}

static qtimer16_ocxo_runtime_t* ocxo_clock_runtime_for(qtimer_clock_kind_t kind) {
  switch (kind) {
    case qtimer_clock_kind_t::OCXO1: return &g_clock_ocxo1;
    case qtimer_clock_kind_t::OCXO2: return &g_clock_ocxo2;
    default: return nullptr;
  }
}

static const char* dispatch_timer_name(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::PPS:   return "PPS_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO1: return "OCXO1_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO2: return "OCXO2_DISPATCH";
    default:                                 return "";
  }
}

// ============================================================================
// OCXO 1 ms cadence model + rolling one-second integrator
// ============================================================================

static void qtimer16_ocxo_arm_bootstrap(qtimer16_ocxo_runtime_t& clock) {
  const uint16_t now16 = clock.hw.module->CH[clock.hw.channel].CNTR;

  clock.phase_bootstrapped = true;
  clock.bootstrap_count++;
  clock.cadence_hits_since_second = 0;
  clock.last_initial_counter16 = now16;

  clock.second_target_low16 = (uint16_t)(now16 + (uint16_t)OCXO_BUCKET_COUNTS);
  clock.last_second_target_low16 = clock.second_target_low16;
  clock.last_programmed_compare = clock.second_target_low16;

  clock.second_window_valid = false;
  clock.current_window_bucket_count = 0;
  clock.previous_bucket_dwt = 0;
  clock.previous_bucket_gnss_ns = -1;
  clock.second_start_dwt_raw = 0;
  clock.second_start_gnss_ns_raw = -1;
  clock.second_start_dwt_final = 0;
  clock.second_start_gnss_ns_final = -1;
  clock.current_window_cycles_sum = 0;
  clock.current_window_gnss_ns_sum = 0;
  clock.last_bucket_cycles = 0;
  clock.last_bucket_gnss_ns = 0;

  qtimer3_program_compare(clock.hw.channel, clock.second_target_low16);
}

static void qtimer16_ocxo_advance_to_next_bucket_target(qtimer16_ocxo_runtime_t& clock) {
  clock.second_target_low16 = (uint16_t)(clock.second_target_low16 + (uint16_t)OCXO_BUCKET_COUNTS);
  clock.last_second_target_low16 = clock.second_target_low16;
  clock.last_programmed_compare = clock.second_target_low16;
  qtimer3_program_compare(clock.hw.channel, clock.second_target_low16);
}

static void qtimer16_ocxo_note_cadence_irq(qtimer16_ocxo_runtime_t& clock,
                                           uint16_t counter16_snapshot) {
  const uint16_t fired_compare16 = clock.second_target_low16;

  clock.last_counter16_at_irq = counter16_snapshot;
  clock.last_fired_compare16 = fired_compare16;
  clock.last_fired_compare_delta_ticks = (int32_t)((uint16_t)(counter16_snapshot - fired_compare16));
  clock.last_fired_compare_delta_ns = (int64_t)clock.last_fired_compare_delta_ticks * 100LL;

  clock.cadence_hits_total++;
  qtimer16_ocxo_advance_to_next_bucket_target(clock);
}

// ============================================================================
// Dispatch
// ============================================================================

static void maybe_dispatch_event(interrupt_subscriber_runtime_t& rt) {
  if (!rt.subscribed || !rt.sub.on_event) return;
  rt.sub.on_event(rt.last_event, &rt.last_diag, rt.sub.user_data);
}

static void deferred_dispatch_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;

  maybe_dispatch_event(*rt);

  if (rt->desc->kind == interrupt_subscriber_kind_t::PPS && g_pps_zero_pending) {
    g_pps_zero_pending = false;
  }
}

// ============================================================================
// Event normalization
// ============================================================================

static void fill_diag_common(interrupt_subscriber_runtime_t& rt,
                             interrupt_capture_diag_t& diag,
                             const interrupt_event_t& event,
                             uint32_t dwt_isr_entry_raw,
                             bool bridge_valid,
                             const qtimer16_ocxo_runtime_t* ocxo_clock = nullptr) {
  diag.provider = rt.desc->provider;
  diag.lane = rt.desc->lane;
  diag.kind = rt.desc->kind;

  diag.dwt_isr_entry_raw = dwt_isr_entry_raw;
  const int64_t dwt_isr_entry_gnss_ns = time_dwt_to_gnss_ns(dwt_isr_entry_raw);
  diag.dwt_isr_entry_gnss_ns = dwt_isr_entry_gnss_ns;
  diag.dwt_isr_entry_minus_event_ns =
      (dwt_isr_entry_gnss_ns >= 0 && event.gnss_ns_at_event != 0)
          ? (dwt_isr_entry_gnss_ns - (int64_t)event.gnss_ns_at_event)
          : 0;

  diag.dwt_at_event = event.dwt_at_event;
  diag.dwt_at_event_adjusted = event.dwt_at_event;
  diag.counter32_at_event = event.counter32_at_event;
  diag.dwt_event_correction_cycles = 0;
  diag.event_target_gnss_ns =
      (ocxo_clock && ocxo_clock->last_event_target_gnss_ns >= 0)
          ? (uint64_t)ocxo_clock->last_event_target_gnss_ns
          : event.gnss_ns_at_event;

  diag.gnss_ns_at_event_raw =
      (ocxo_clock && ocxo_clock->last_event_gnss_ns_raw >= 0)
          ? (uint64_t)ocxo_clock->last_event_gnss_ns_raw
          : event.gnss_ns_at_event;
  diag.gnss_ns_at_event_final = event.gnss_ns_at_event;
  diag.gnss_ns_at_event = event.gnss_ns_at_event;
  diag.gnss_ns_at_event_delta =
      (int64_t)diag.gnss_ns_at_event_final - (int64_t)diag.gnss_ns_at_event_raw;

  diag.bridge_valid = bridge_valid;
  if (ocxo_clock && ocxo_clock->last_event_target_gnss_ns >= 0) {
    const int64_t abs_err =
        (ocxo_clock->last_second_gnss_ns_prediction_error < 0)
            ? -ocxo_clock->last_second_gnss_ns_prediction_error
            : ocxo_clock->last_second_gnss_ns_prediction_error;
    diag.bridge_within_tolerance = abs_err <= OCXO_PREDICTION_TOLERANCE_NS;
  } else {
    diag.bridge_within_tolerance = bridge_valid;
  }
  diag.bridge_skipped_invalid = !bridge_valid;
  diag.bridge_used_prediction = (ocxo_clock && ocxo_clock->last_event_target_gnss_ns >= 0);

  diag.gnss_ns_at_event_bridge = diag.gnss_ns_at_event_final;
  diag.bridge_gnss_ns_raw = diag.gnss_ns_at_event_raw;
  diag.bridge_gnss_ns_target =
      (ocxo_clock && ocxo_clock->last_event_target_gnss_ns >= 0)
          ? (uint64_t)ocxo_clock->last_event_target_gnss_ns
          : diag.gnss_ns_at_event_final;
  diag.bridge_gnss_ns_final = diag.gnss_ns_at_event_final;
  diag.bridge_raw_error_ns =
      (int64_t)diag.bridge_gnss_ns_raw - (int64_t)diag.bridge_gnss_ns_target;
  diag.bridge_final_error_ns =
      (int64_t)diag.bridge_gnss_ns_final - (int64_t)diag.bridge_gnss_ns_target;

  if (ocxo_clock) {
    diag.counter16_at_irq = ocxo_clock->last_counter16_at_irq;
    diag.compare16_fired = ocxo_clock->last_fired_compare16;
    diag.compare16_next_programmed = ocxo_clock->last_programmed_compare;
    diag.counter16_minus_compare_ticks = ocxo_clock->last_fired_compare_delta_ticks;
    diag.counter16_minus_compare_ns = ocxo_clock->last_fired_compare_delta_ns;

    diag.ocxo_bucket_interval_counts = OCXO_BUCKET_COUNTS;
    diag.ocxo_current_window_bucket_count = ocxo_clock->current_window_bucket_count;
    diag.ocxo_last_second_bucket_count = ocxo_clock->last_second_bucket_count;
    diag.ocxo_last_bucket_cycles = ocxo_clock->last_bucket_cycles;
    diag.ocxo_last_bucket_gnss_ns = ocxo_clock->last_bucket_gnss_ns;
    diag.ocxo_current_window_cycles_sum = ocxo_clock->current_window_cycles_sum;
    diag.ocxo_current_window_gnss_ns_sum = ocxo_clock->current_window_gnss_ns_sum;
    diag.ocxo_second_cycles_observed = ocxo_clock->last_second_cycles_observed;
    diag.ocxo_second_cycles_observed_raw = ocxo_clock->last_second_cycles_observed_raw;
    diag.ocxo_second_cycles_prediction = ocxo_clock->last_second_cycles_prediction;
    diag.ocxo_second_cycles_prediction_error = ocxo_clock->last_second_cycles_prediction_error;
    diag.ocxo_second_gnss_ns_observed = ocxo_clock->last_second_gnss_ns_observed;
    diag.ocxo_second_gnss_ns_observed_raw = ocxo_clock->last_second_gnss_ns_observed_raw;
    diag.ocxo_second_gnss_ns_prediction = ocxo_clock->last_second_gnss_ns_prediction;
    diag.ocxo_second_gnss_ns_prediction_error = ocxo_clock->last_second_gnss_ns_prediction_error;
    diag.ocxo_second_residual_ns = ocxo_clock->last_second_residual_ns;
    diag.ocxo_second_start_gnss_ns_raw = ocxo_clock->last_second_start_gnss_ns_raw;
    diag.ocxo_second_end_gnss_ns_raw = ocxo_clock->last_second_end_gnss_ns_raw;
    diag.ocxo_second_start_dwt_raw = ocxo_clock->last_second_start_dwt_raw;
    diag.ocxo_second_end_dwt_raw = ocxo_clock->last_second_end_dwt_raw;
    diag.ocxo_second_start_gnss_ns_final = ocxo_clock->last_second_start_gnss_ns_final;
    diag.ocxo_second_end_gnss_ns_final = ocxo_clock->last_second_end_gnss_ns_final;
    diag.ocxo_second_start_dwt_final = ocxo_clock->last_second_start_dwt_final;
    diag.ocxo_second_end_dwt_final = ocxo_clock->last_second_end_dwt_final;
    diag.ocxo_second_start_raw_minus_final_ns = ocxo_clock->last_second_start_raw_minus_final_ns;
    diag.ocxo_second_end_raw_minus_final_ns = ocxo_clock->last_second_end_raw_minus_final_ns;
  }
}

static void handle_event(interrupt_subscriber_runtime_t& rt,
                         uint32_t dwt_isr_entry_raw,
                         uint32_t counter32_at_event,
                         const qtimer16_ocxo_runtime_t* ocxo_clock = nullptr) {
  interrupt_event_t event {};
  event.kind = rt.desc->kind;
  event.provider = rt.desc->provider;
  event.lane = rt.desc->lane;
  event.status = interrupt_event_status_t::OK;
  event.counter32_at_event = counter32_at_event;
  event.dwt_event_correction_cycles = 0;

  bool bridge_valid = true;

  if (rt.desc->kind == interrupt_subscriber_kind_t::PPS) {
    event.dwt_at_event = dwt_isr_entry_raw;
    event.gnss_ns_at_event = (uint64_t)rt.event_count * NS_PER_SECOND_U64;
  } else {
    const int64_t gnss_ns_signed = time_dwt_to_gnss_ns(dwt_isr_entry_raw);
    const int64_t raw_gnss_ns =
        (ocxo_clock && ocxo_clock->last_event_gnss_ns_raw >= 0)
            ? ocxo_clock->last_event_gnss_ns_raw
            : gnss_ns_signed;

    const int64_t final_gnss_ns =
        (ocxo_clock && ocxo_clock->last_event_gnss_ns_final >= 0)
            ? ocxo_clock->last_event_gnss_ns_final
            : raw_gnss_ns;

    const uint32_t final_dwt =
        (ocxo_clock && ocxo_clock->last_event_dwt_final != 0)
            ? ocxo_clock->last_event_dwt_final
            : dwt_isr_entry_raw;

    if (final_gnss_ns < 0) {
      event.gnss_ns_at_event = 0;
      event.dwt_at_event = dwt_isr_entry_raw;
      event.status = interrupt_event_status_t::HOLD;
      bridge_valid = false;
    } else {
      event.gnss_ns_at_event = (uint64_t)final_gnss_ns;
      event.dwt_at_event = final_dwt;
    }
  }

  interrupt_capture_diag_t diag {};
  fill_diag_common(rt, diag, event, dwt_isr_entry_raw, bridge_valid, ocxo_clock);

  rt.last_event = event;
  rt.last_diag = diag;
  rt.has_fired = true;
  rt.irq_count++;
  rt.dispatch_count++;
  rt.event_count++;

  timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
}

// ============================================================================
// ISR wrappers
// ============================================================================

static void pps_gpio_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  if (IMXRT_TMR3.CH[2].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch2_irq(dwt_raw);
  if (IMXRT_TMR3.CH[3].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch3_irq(dwt_raw);
}

static void handle_ocxo_qtimer16_irq(qtimer16_ocxo_runtime_t& clock,
                                     interrupt_subscriber_runtime_t& rt,
                                     uint32_t dwt_raw) {
  clock.irq_count++;
  qtimer3_clear_compare_flag(clock.hw.channel);

  if (!clock.active || !rt.active) {
    clock.miss_count++;
    return;
  }

  const uint16_t counter16_snapshot = clock.hw.module->CH[clock.hw.channel].CNTR;
  if (!clock.phase_bootstrapped) {
    clock.miss_count++;
    return;
  }

  qtimer16_ocxo_note_cadence_irq(clock, counter16_snapshot);

  const int64_t current_gnss_ns_raw = time_dwt_to_gnss_ns(dwt_raw);
  const int64_t compare_latency_ns =
      (clock.last_fired_compare_delta_ns > 0) ? clock.last_fired_compare_delta_ns : 0;
  const uint32_t compare_latency_cycles = ns_to_dwt_cycles_runtime((uint64_t)compare_latency_ns);
  const uint32_t current_dwt_final = dwt_raw - compare_latency_cycles;
  const int64_t current_gnss_ns_final =
      (current_gnss_ns_raw >= 0) ? (current_gnss_ns_raw - compare_latency_ns) : -1;

  // First cadence hit only establishes the baseline raw edge and also seeds
  // the first synthetic second boundary.  After this seed, the synthetic
  // boundary is carried forward perpetually from one completed second to the
  // next, rather than being re-founded from later raw ISR-entry samples.
  if (!clock.second_window_valid) {
    clock.second_window_valid = true;
    clock.current_window_bucket_count = 0;
    clock.previous_bucket_dwt = current_dwt_final;
    clock.previous_bucket_gnss_ns = current_gnss_ns_final;
    clock.previous_bucket_dwt_raw = dwt_raw;
    clock.previous_bucket_gnss_ns_raw = current_gnss_ns_raw;
    clock.second_start_dwt_raw = dwt_raw;
    clock.second_start_gnss_ns_raw = current_gnss_ns_raw;
    clock.second_start_dwt_final = current_dwt_final;
    clock.second_start_gnss_ns_final = current_gnss_ns_final;
    clock.current_window_cycles_sum = 0;
    clock.current_window_gnss_ns_sum = 0;
    clock.current_window_cycles_sum_raw = 0;
    clock.current_window_gnss_ns_sum_raw = 0;
    clock.last_bucket_cycles = 0;
    clock.last_bucket_gnss_ns = 0;
    clock.last_bucket_cycles_raw = 0;
    clock.last_bucket_gnss_ns_raw = 0;

    clock.last_event_dwt_raw = dwt_raw;
    clock.last_event_gnss_ns_raw = current_gnss_ns_raw;
    clock.last_event_dwt_final = current_dwt_final;
    clock.last_event_gnss_ns_final = current_gnss_ns_final;
    clock.last_event_target_dwt = 0;
    clock.last_event_target_gnss_ns = -1;
    return;
  }

  const uint32_t bucket_cycles_raw = dwt_raw - clock.previous_bucket_dwt_raw;
  clock.last_bucket_cycles_raw = bucket_cycles_raw;
  clock.current_window_cycles_sum_raw += (uint64_t)bucket_cycles_raw;

  if (current_gnss_ns_raw >= 0 && clock.previous_bucket_gnss_ns_raw >= 0) {
    const int64_t bucket_gnss_ns_raw = current_gnss_ns_raw - clock.previous_bucket_gnss_ns_raw;
    clock.last_bucket_gnss_ns_raw = bucket_gnss_ns_raw;
    clock.current_window_gnss_ns_sum_raw += bucket_gnss_ns_raw;
  } else {
    clock.last_bucket_gnss_ns_raw = 0;
  }

  const uint32_t bucket_cycles = current_dwt_final - clock.previous_bucket_dwt;
  clock.last_bucket_cycles = bucket_cycles;
  clock.current_window_cycles_sum += (uint64_t)bucket_cycles;

  if (current_gnss_ns_final >= 0 && clock.previous_bucket_gnss_ns >= 0) {
    const int64_t bucket_gnss_ns = current_gnss_ns_final - clock.previous_bucket_gnss_ns;
    clock.last_bucket_gnss_ns = bucket_gnss_ns;
    clock.current_window_gnss_ns_sum += bucket_gnss_ns;
  } else {
    clock.last_bucket_gnss_ns = 0;
  }

  clock.previous_bucket_dwt_raw = dwt_raw;
  clock.previous_bucket_gnss_ns_raw = current_gnss_ns_raw;
  clock.previous_bucket_dwt = current_dwt_final;
  clock.previous_bucket_gnss_ns = current_gnss_ns_final;
  clock.current_window_bucket_count++;
  clock.cadence_hits_since_second = clock.current_window_bucket_count;

  if (clock.current_window_bucket_count < OCXO_BUCKETS_PER_SECOND) {
    return;
  }

  clock.last_second_bucket_count = clock.current_window_bucket_count;
  clock.last_second_cycles_observed_raw = clock.current_window_cycles_sum_raw;
  clock.last_second_gnss_ns_observed_raw = clock.current_window_gnss_ns_sum_raw;
  clock.last_second_start_dwt_raw = clock.second_start_dwt_raw;
  clock.last_second_end_dwt_raw = dwt_raw;
  clock.last_second_start_gnss_ns_raw = clock.second_start_gnss_ns_raw;
  clock.last_second_end_gnss_ns_raw = current_gnss_ns_raw;

  // The lawful second begins at the prior synthetic edge and ends at the
  // prior synthetic edge plus the observed one-second bucket sum.
  clock.last_second_start_dwt_final = clock.second_start_dwt_final;
  clock.last_second_start_gnss_ns_final = clock.second_start_gnss_ns_final;

  if (clock.second_start_dwt_final != 0) {
    clock.last_second_end_dwt_final =
        clock.second_start_dwt_final + (uint32_t)clock.current_window_cycles_sum;
  } else {
    clock.last_second_end_dwt_final = current_dwt_final;
  }

  if (clock.second_start_gnss_ns_final >= 0) {
    clock.last_second_end_gnss_ns_final =
        clock.second_start_gnss_ns_final + clock.current_window_gnss_ns_sum;
  } else {
    clock.last_second_end_gnss_ns_final = current_gnss_ns_final;
  }

  // Canonical authoritative one-second values are the synthetic/final boundary deltas.
  clock.last_second_cycles_observed =
      (clock.last_second_end_dwt_final - clock.last_second_start_dwt_final);
  clock.last_second_gnss_ns_observed =
      (clock.last_second_end_gnss_ns_final >= 0 && clock.last_second_start_gnss_ns_final >= 0)
          ? (clock.last_second_end_gnss_ns_final - clock.last_second_start_gnss_ns_final)
          : 0;

  clock.last_second_start_raw_minus_final_ns =
      (clock.last_second_start_gnss_ns_raw >= 0 && clock.last_second_start_gnss_ns_final >= 0)
          ? (clock.last_second_start_gnss_ns_raw - clock.last_second_start_gnss_ns_final)
          : 0;
  clock.last_second_end_raw_minus_final_ns =
      (clock.last_second_end_gnss_ns_raw >= 0 && clock.last_second_end_gnss_ns_final >= 0)
          ? (clock.last_second_end_gnss_ns_raw - clock.last_second_end_gnss_ns_final)
          : 0;

  if (clock.next_second_prediction_valid) {
    clock.last_second_cycles_prediction = clock.next_second_cycles_prediction;
    clock.last_second_gnss_ns_prediction = clock.next_second_gnss_ns_prediction;
    clock.last_second_cycles_prediction_error =
        (int64_t)clock.last_second_cycles_observed - (int64_t)clock.last_second_cycles_prediction;
    clock.last_second_gnss_ns_prediction_error =
        clock.last_second_gnss_ns_observed - clock.last_second_gnss_ns_prediction;
  } else {
    clock.last_second_cycles_prediction = 0;
    clock.last_second_gnss_ns_prediction = 0;
    clock.last_second_cycles_prediction_error = 0;
    clock.last_second_gnss_ns_prediction_error = 0;
  }

  if (clock.last_second_gnss_ns_observed > 0) {
    clock.last_second_residual_ns = (int64_t)NS_PER_SECOND_U64 - clock.last_second_gnss_ns_observed;
  } else {
    clock.last_second_residual_ns = 0;
  }

  clock.logical_count32_at_last_second_edge += OCXO_COUNTS_PER_SECOND;
  const uint32_t logical_counter32_at_event = clock.logical_count32_at_last_second_edge;

  const uint32_t current_event_target_dwt =
      (clock.next_second_prediction_valid && clock.second_start_dwt_final != 0)
          ? (clock.second_start_dwt_final + (uint32_t)clock.next_second_cycles_prediction)
          : 0;
  const int64_t current_event_target_gnss_ns =
      (clock.next_second_prediction_valid && clock.second_start_gnss_ns_final >= 0)
          ? (clock.second_start_gnss_ns_final + clock.next_second_gnss_ns_prediction)
          : -1;

  clock.last_event_dwt_raw = dwt_raw;
  clock.last_event_gnss_ns_raw = current_gnss_ns_raw;
  clock.last_event_dwt_final = clock.last_second_end_dwt_final;
  clock.last_event_gnss_ns_final = clock.last_second_end_gnss_ns_final;
  clock.last_event_target_dwt = current_event_target_dwt;
  clock.last_event_target_gnss_ns = current_event_target_gnss_ns;

  clock.next_second_cycles_prediction = clock.last_second_cycles_observed;
  clock.next_second_gnss_ns_prediction = clock.last_second_gnss_ns_observed;
  clock.next_second_prediction_valid =
      (clock.last_second_cycles_observed > 0 && clock.last_second_gnss_ns_observed > 0);

  clock.dispatch_count++;
  handle_event(rt, dwt_raw, logical_counter32_at_event, &clock);

  // Start the next accumulation window with a fresh sum, but carry forward the
  // newly synthesized lawful second edge as the beginning of the next second.
  clock.current_window_bucket_count = 0;
  clock.current_window_cycles_sum = 0;
  clock.current_window_gnss_ns_sum = 0;
  clock.current_window_cycles_sum_raw = 0;
  clock.current_window_gnss_ns_sum_raw = 0;
  clock.second_start_dwt_raw = dwt_raw;
  clock.second_start_gnss_ns_raw = current_gnss_ns_raw;
  clock.second_start_dwt_final = clock.last_event_dwt_final;
  clock.second_start_gnss_ns_final = clock.last_event_gnss_ns_final;
  clock.previous_bucket_dwt_raw = dwt_raw;
  clock.previous_bucket_gnss_ns_raw = current_gnss_ns_raw;
  clock.previous_bucket_dwt = current_dwt_final;
  clock.previous_bucket_gnss_ns = current_gnss_ns_final;
  clock.cadence_hits_since_second = 0;
}

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw) {
  const uint32_t pps_qtimer_at_edge = qtimer1_read_32_for_pps_capture();

  g_gpio_diag.irq_count++;
  interrupt_subscriber_runtime_t* rt = g_rt_pps;
  if (!rt || !rt->active) {
    g_gpio_diag.miss_count++;
    return;
  }

  g_gpio_diag.dispatch_count++;
  handle_event(*rt, dwt_isr_entry_raw, pps_qtimer_at_edge);
}

void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_raw) {
  if (!g_rt_ocxo1) return;
  handle_ocxo_qtimer16_irq(g_clock_ocxo1, *g_rt_ocxo1, dwt_raw);
}

void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_raw) {
  if (!g_rt_ocxo2) return;
  handle_ocxo_qtimer16_irq(g_clock_ocxo2, *g_rt_ocxo2, dwt_raw);
}

// ============================================================================
// Public control
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub) {
  if (!g_interrupt_runtime_ready) return false;
  if (sub.kind == interrupt_subscriber_kind_t::NONE) return false;
  if (!sub.on_event) return false;

  interrupt_subscriber_runtime_t* rt = runtime_for(sub.kind);
  if (!rt || !rt->desc) return false;
  rt->sub = sub;
  rt->subscribed = true;
  return true;
}

bool interrupt_start(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;

  rt->active = true;
  rt->start_count++;

  if (kind == interrupt_subscriber_kind_t::PPS) {
    return true;
  }

  qtimer16_ocxo_runtime_t* clock = ocxo_clock_runtime_for(rt->desc->clock_kind);
  if (!clock || !clock->initialized) return false;

  clock->active = true;
  clock->start_count++;
  clock->logical_count32_at_last_second_edge = 0;
  clock->phase_bootstrapped = false;
  clock->cadence_hits_since_second = 0;
  clock->second_window_valid = false;
  clock->current_window_bucket_count = 0;
  clock->previous_bucket_dwt = 0;
  clock->previous_bucket_gnss_ns = -1;
  clock->previous_bucket_dwt_raw = 0;
  clock->previous_bucket_gnss_ns_raw = -1;
  clock->second_start_dwt_raw = 0;
  clock->second_start_gnss_ns_raw = -1;
  clock->second_start_dwt_final = 0;
  clock->second_start_gnss_ns_final = -1;
  clock->current_window_cycles_sum = 0;
  clock->current_window_gnss_ns_sum = 0;
  clock->current_window_cycles_sum_raw = 0;
  clock->current_window_gnss_ns_sum_raw = 0;
  clock->last_bucket_cycles = 0;
  clock->last_bucket_gnss_ns = 0;
  clock->last_bucket_cycles_raw = 0;
  clock->last_bucket_gnss_ns_raw = 0;
  clock->last_second_bucket_count = 0;
  clock->last_second_cycles_observed = 0;
  clock->last_second_gnss_ns_observed = 0;
  clock->last_second_cycles_observed_raw = 0;
  clock->last_second_gnss_ns_observed_raw = 0;
  clock->last_second_cycles_prediction = 0;
  clock->last_second_gnss_ns_prediction = 0;
  clock->last_second_cycles_prediction_error = 0;
  clock->last_second_gnss_ns_prediction_error = 0;
  clock->last_second_residual_ns = 0;
  clock->last_second_start_dwt_raw = 0;
  clock->last_second_end_dwt_raw = 0;
  clock->last_second_start_gnss_ns_raw = -1;
  clock->last_second_end_gnss_ns_raw = -1;
  clock->last_second_start_dwt_final = 0;
  clock->last_second_end_dwt_final = 0;
  clock->last_second_start_gnss_ns_final = -1;
  clock->last_second_end_gnss_ns_final = -1;
  clock->last_second_start_raw_minus_final_ns = 0;
  clock->last_second_end_raw_minus_final_ns = 0;
  clock->next_second_prediction_valid = false;
  clock->next_second_cycles_prediction = 0;
  clock->next_second_gnss_ns_prediction = 0;
  clock->last_event_dwt_raw = 0;
  clock->last_event_dwt_final = 0;
  clock->last_event_gnss_ns_raw = -1;
  clock->last_event_gnss_ns_final = -1;
  clock->last_event_target_dwt = 0;
  clock->last_event_target_gnss_ns = -1;

  qtimer16_ocxo_arm_bootstrap(*clock);
  return true;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;

  rt->active = false;
  rt->stop_count++;

  qtimer16_ocxo_runtime_t* clock = ocxo_clock_runtime_for(rt->desc->clock_kind);
  if (clock) {
    clock->active = false;
    clock->stop_count++;
    qtimer3_disable_compare(clock->hw.channel);
  }
  return true;
}

void interrupt_request_pps_zero(void) { g_pps_zero_pending = true; }
bool interrupt_pps_zero_pending(void) { return g_pps_zero_pending; }

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->has_fired) return nullptr;
  return &rt->last_event;
}

const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->has_fired) return nullptr;
  return &rt->last_diag;
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {
  // retained for linkage compatibility
}

// ============================================================================
// Lifecycle
// ============================================================================

void process_interrupt_init_hardware(void) {
  if (g_interrupt_hw_ready) return;

  g_clock_ocxo1.kind = qtimer_clock_kind_t::OCXO1;
  g_clock_ocxo1.name = "OCXO1";
  g_clock_ocxo1.hw.module = &IMXRT_TMR3;
  g_clock_ocxo1.hw.channel = 2;
  g_clock_ocxo1.hw.irq = IRQ_QTIMER3;
  g_clock_ocxo1.hw.input_pin = OCXO1_PIN;
  g_clock_ocxo1.hw.pcs = 2;

  g_clock_ocxo2.kind = qtimer_clock_kind_t::OCXO2;
  g_clock_ocxo2.name = "OCXO2";
  g_clock_ocxo2.hw.module = &IMXRT_TMR3;
  g_clock_ocxo2.hw.channel = 3;
  g_clock_ocxo2.hw.irq = IRQ_QTIMER3;
  g_clock_ocxo2.hw.input_pin = OCXO2_PIN;
  g_clock_ocxo2.hw.pcs = 3;

  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);

  *(portConfigRegister(OCXO1_PIN)) = 1;
  *(portConfigRegister(OCXO2_PIN)) = 1;
  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

  IMXRT_TMR3.CH[2].CTRL   = 0;
  IMXRT_TMR3.CH[2].SCTRL  = 0;
  IMXRT_TMR3.CH[2].CSCTRL = 0;
  IMXRT_TMR3.CH[2].LOAD   = 0;
  IMXRT_TMR3.CH[2].CNTR   = 0;
  IMXRT_TMR3.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR3.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_clock_ocxo1.hw.pcs);
  qtimer3_disable_compare(2);

  IMXRT_TMR3.CH[3].CTRL   = 0;
  IMXRT_TMR3.CH[3].SCTRL  = 0;
  IMXRT_TMR3.CH[3].CSCTRL = 0;
  IMXRT_TMR3.CH[3].LOAD   = 0;
  IMXRT_TMR3.CH[3].CNTR   = 0;
  IMXRT_TMR3.CH[3].COMP1  = 0xFFFF;
  IMXRT_TMR3.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_clock_ocxo2.hw.pcs);
  qtimer3_disable_compare(3);

  g_clock_ocxo1.initialized = true;
  g_clock_ocxo2.initialized = true;
  g_interrupt_hw_ready = true;
}

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;

  g_subscriber_count = 0;
  for (auto& rt : g_subscribers) rt = interrupt_subscriber_runtime_t{};

  for (uint32_t i = 0; i < (sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0])); i++) {
    interrupt_subscriber_runtime_t& rt = g_subscribers[g_subscriber_count++];
    rt = interrupt_subscriber_runtime_t{};
    rt.desc = &DESCRIPTORS[i];

    if (rt.desc->kind == interrupt_subscriber_kind_t::PPS) {
      g_rt_pps = &rt;
    } else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1) {
      g_rt_ocxo1 = &rt;
    } else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO2) {
      g_rt_ocxo2 = &rt;
    }
  }

  g_gpio_diag = {};
  g_pps_zero_pending = false;
  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  g_interrupt_irqs_enabled = true;
}

// ============================================================================
// REPORT
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready", g_interrupt_runtime_ready);
  p.add("irqs_enabled", g_interrupt_irqs_enabled);
  p.add("subscriber_count", g_subscriber_count);

  p.add("pps_zero_pending", g_pps_zero_pending);

  p.add("gpio_irq_count", g_gpio_diag.irq_count);
  p.add("gpio_dispatch_count", g_gpio_diag.dispatch_count);
  p.add("gpio_miss_count", g_gpio_diag.miss_count);

  p.add("ocxo1_irq_count", g_clock_ocxo1.irq_count);
  p.add("ocxo1_dispatch_count", g_clock_ocxo1.dispatch_count);
  p.add("ocxo1_miss_count", g_clock_ocxo1.miss_count);
  p.add("ocxo1_bootstrap_count", g_clock_ocxo1.bootstrap_count);
  p.add("ocxo1_cadence_hits_total", g_clock_ocxo1.cadence_hits_total);
  p.add("ocxo1_current_window_bucket_count", g_clock_ocxo1.current_window_bucket_count);
  p.add("ocxo1_last_programmed_compare", (uint32_t)g_clock_ocxo1.last_programmed_compare);
  p.add("ocxo1_last_compare16_fired", (uint32_t)g_clock_ocxo1.last_fired_compare16);
  p.add("ocxo1_last_counter16_at_irq", (uint32_t)g_clock_ocxo1.last_counter16_at_irq);
  p.add("ocxo1_last_compare_delta_ticks", g_clock_ocxo1.last_fired_compare_delta_ticks);
  p.add("ocxo1_last_compare_delta_ns", g_clock_ocxo1.last_fired_compare_delta_ns);
  p.add("ocxo1_last_bucket_cycles", g_clock_ocxo1.last_bucket_cycles);
  p.add("ocxo1_last_bucket_gnss_ns", g_clock_ocxo1.last_bucket_gnss_ns);
  p.add("ocxo1_current_window_cycles_sum", (uint64_t)g_clock_ocxo1.current_window_cycles_sum);
  p.add("ocxo1_current_window_gnss_ns_sum", g_clock_ocxo1.current_window_gnss_ns_sum);
  p.add("ocxo1_last_second_cycles_observed", (uint64_t)g_clock_ocxo1.last_second_cycles_observed);
  p.add("ocxo1_last_second_cycles_observed_raw", (uint64_t)g_clock_ocxo1.last_second_cycles_observed_raw);
  p.add("ocxo1_last_second_gnss_ns_observed", g_clock_ocxo1.last_second_gnss_ns_observed);
  p.add("ocxo1_last_second_gnss_ns_observed_raw", g_clock_ocxo1.last_second_gnss_ns_observed_raw);
  p.add("ocxo1_last_second_cycles_prediction", (uint64_t)g_clock_ocxo1.last_second_cycles_prediction);
  p.add("ocxo1_last_second_gnss_ns_prediction", g_clock_ocxo1.last_second_gnss_ns_prediction);
  p.add("ocxo1_last_second_cycles_prediction_error", g_clock_ocxo1.last_second_cycles_prediction_error);
  p.add("ocxo1_last_second_gnss_ns_prediction_error", g_clock_ocxo1.last_second_gnss_ns_prediction_error);
  p.add("ocxo1_last_second_residual_ns", g_clock_ocxo1.last_second_residual_ns);
  p.add("ocxo1_last_second_start_gnss_ns_final", g_clock_ocxo1.last_second_start_gnss_ns_final);
  p.add("ocxo1_last_second_end_gnss_ns_final", g_clock_ocxo1.last_second_end_gnss_ns_final);
  p.add("ocxo1_last_second_start_raw_minus_final_ns", g_clock_ocxo1.last_second_start_raw_minus_final_ns);
  p.add("ocxo1_last_second_end_raw_minus_final_ns", g_clock_ocxo1.last_second_end_raw_minus_final_ns);

  p.add("ocxo2_irq_count", g_clock_ocxo2.irq_count);
  p.add("ocxo2_dispatch_count", g_clock_ocxo2.dispatch_count);
  p.add("ocxo2_miss_count", g_clock_ocxo2.miss_count);
  p.add("ocxo2_bootstrap_count", g_clock_ocxo2.bootstrap_count);
  p.add("ocxo2_cadence_hits_total", g_clock_ocxo2.cadence_hits_total);
  p.add("ocxo2_current_window_bucket_count", g_clock_ocxo2.current_window_bucket_count);
  p.add("ocxo2_last_programmed_compare", (uint32_t)g_clock_ocxo2.last_programmed_compare);
  p.add("ocxo2_last_compare16_fired", (uint32_t)g_clock_ocxo2.last_fired_compare16);
  p.add("ocxo2_last_counter16_at_irq", (uint32_t)g_clock_ocxo2.last_counter16_at_irq);
  p.add("ocxo2_last_compare_delta_ticks", g_clock_ocxo2.last_fired_compare_delta_ticks);
  p.add("ocxo2_last_compare_delta_ns", g_clock_ocxo2.last_fired_compare_delta_ns);
  p.add("ocxo2_last_bucket_cycles", g_clock_ocxo2.last_bucket_cycles);
  p.add("ocxo2_last_bucket_gnss_ns", g_clock_ocxo2.last_bucket_gnss_ns);
  p.add("ocxo2_current_window_cycles_sum", (uint64_t)g_clock_ocxo2.current_window_cycles_sum);
  p.add("ocxo2_current_window_gnss_ns_sum", g_clock_ocxo2.current_window_gnss_ns_sum);
  p.add("ocxo2_last_second_cycles_observed", (uint64_t)g_clock_ocxo2.last_second_cycles_observed);
  p.add("ocxo2_last_second_cycles_observed_raw", (uint64_t)g_clock_ocxo2.last_second_cycles_observed_raw);
  p.add("ocxo2_last_second_gnss_ns_observed", g_clock_ocxo2.last_second_gnss_ns_observed);
  p.add("ocxo2_last_second_gnss_ns_observed_raw", g_clock_ocxo2.last_second_gnss_ns_observed_raw);
  p.add("ocxo2_last_second_cycles_prediction", (uint64_t)g_clock_ocxo2.last_second_cycles_prediction);
  p.add("ocxo2_last_second_gnss_ns_prediction", g_clock_ocxo2.last_second_gnss_ns_prediction);
  p.add("ocxo2_last_second_cycles_prediction_error", g_clock_ocxo2.last_second_cycles_prediction_error);
  p.add("ocxo2_last_second_gnss_ns_prediction_error", g_clock_ocxo2.last_second_gnss_ns_prediction_error);
  p.add("ocxo2_last_second_residual_ns", g_clock_ocxo2.last_second_residual_ns);
  p.add("ocxo2_last_second_start_gnss_ns_final", g_clock_ocxo2.last_second_start_gnss_ns_final);
  p.add("ocxo2_last_second_end_gnss_ns_final", g_clock_ocxo2.last_second_end_gnss_ns_final);
  p.add("ocxo2_last_second_start_raw_minus_final_ns", g_clock_ocxo2.last_second_start_raw_minus_final_ns);
  p.add("ocxo2_last_second_end_raw_minus_final_ns", g_clock_ocxo2.last_second_end_raw_minus_final_ns);

  PayloadArray subscribers;
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    const interrupt_subscriber_runtime_t& rt = g_subscribers[i];
    if (!rt.desc) continue;

    Payload s;
    s.add("slot", i);
    s.add("kind", interrupt_subscriber_kind_str(rt.desc->kind));
    s.add("provider", interrupt_provider_kind_str(rt.desc->provider));
    s.add("lane", interrupt_lane_str(rt.desc->lane));
    s.add("active", rt.active);
    s.add("subscribed", rt.subscribed);
    s.add("start_count", rt.start_count);
    s.add("stop_count", rt.stop_count);
    s.add("irq_count", rt.irq_count);
    s.add("dispatch_count", rt.dispatch_count);
    s.add("event_count", rt.event_count);

    s.add("last_gnss_ns_at_event", rt.last_event.gnss_ns_at_event);
    s.add("last_dwt_at_event", rt.last_event.dwt_at_event);
    s.add("last_counter32_at_event", rt.last_event.counter32_at_event);
    s.add("last_status", (uint32_t)rt.last_event.status);

    s.add("last_dwt_isr_entry_raw", rt.last_diag.dwt_isr_entry_raw);
    s.add("last_dwt_isr_entry_gnss_ns", rt.last_diag.dwt_isr_entry_gnss_ns);
    s.add("last_dwt_isr_entry_minus_event_ns", rt.last_diag.dwt_isr_entry_minus_event_ns);

    s.add("last_gnss_ns_at_event_raw", rt.last_diag.gnss_ns_at_event_raw);
    s.add("last_gnss_ns_at_event_final", rt.last_diag.gnss_ns_at_event_final);
    s.add("last_gnss_ns_at_event_delta", rt.last_diag.gnss_ns_at_event_delta);

    s.add("bridge_valid", rt.last_diag.bridge_valid);
    s.add("bridge_within_tolerance", rt.last_diag.bridge_within_tolerance);
    s.add("bridge_skipped_invalid", rt.last_diag.bridge_skipped_invalid);
    s.add("bridge_gnss_ns_raw", rt.last_diag.bridge_gnss_ns_raw);
    s.add("bridge_gnss_ns_target", rt.last_diag.bridge_gnss_ns_target);
    s.add("bridge_gnss_ns_final", rt.last_diag.bridge_gnss_ns_final);
    s.add("bridge_raw_error_ns", rt.last_diag.bridge_raw_error_ns);
    s.add("bridge_final_error_ns", rt.last_diag.bridge_final_error_ns);

    if (rt.desc->clock_kind == qtimer_clock_kind_t::OCXO1 ||
        rt.desc->clock_kind == qtimer_clock_kind_t::OCXO2) {
      const qtimer16_ocxo_runtime_t* clock = ocxo_clock_runtime_for(rt.desc->clock_kind);
      if (clock) {
        s.add("bootstrap_count", clock->bootstrap_count);
        s.add("cadence_hits_total", clock->cadence_hits_total);
      }
      s.add("counter16_at_irq", (uint32_t)rt.last_diag.counter16_at_irq);
      s.add("compare16_fired", (uint32_t)rt.last_diag.compare16_fired);
      s.add("compare16_next_programmed", (uint32_t)rt.last_diag.compare16_next_programmed);
      s.add("counter16_minus_compare_ticks", rt.last_diag.counter16_minus_compare_ticks);
      s.add("counter16_minus_compare_ns", rt.last_diag.counter16_minus_compare_ns);

      s.add("ocxo_bucket_interval_counts", rt.last_diag.ocxo_bucket_interval_counts);
      s.add("ocxo_current_window_bucket_count", rt.last_diag.ocxo_current_window_bucket_count);
      s.add("ocxo_last_second_bucket_count", rt.last_diag.ocxo_last_second_bucket_count);
      s.add("ocxo_last_bucket_cycles", rt.last_diag.ocxo_last_bucket_cycles);
      s.add("ocxo_last_bucket_gnss_ns", rt.last_diag.ocxo_last_bucket_gnss_ns);
      s.add("ocxo_current_window_cycles_sum", (uint64_t)rt.last_diag.ocxo_current_window_cycles_sum);
      s.add("ocxo_current_window_gnss_ns_sum", rt.last_diag.ocxo_current_window_gnss_ns_sum);
      s.add("ocxo_second_cycles_observed", (uint64_t)rt.last_diag.ocxo_second_cycles_observed);
      s.add("ocxo_second_cycles_observed_raw", (uint64_t)rt.last_diag.ocxo_second_cycles_observed_raw);
      s.add("ocxo_second_cycles_prediction", (uint64_t)rt.last_diag.ocxo_second_cycles_prediction);
      s.add("ocxo_second_cycles_prediction_error", rt.last_diag.ocxo_second_cycles_prediction_error);
      s.add("ocxo_second_gnss_ns_observed", rt.last_diag.ocxo_second_gnss_ns_observed);
      s.add("ocxo_second_gnss_ns_observed_raw", rt.last_diag.ocxo_second_gnss_ns_observed_raw);
      s.add("ocxo_second_gnss_ns_prediction", rt.last_diag.ocxo_second_gnss_ns_prediction);
      s.add("ocxo_second_gnss_ns_prediction_error", rt.last_diag.ocxo_second_gnss_ns_prediction_error);
      s.add("ocxo_second_residual_ns", rt.last_diag.ocxo_second_residual_ns);
      s.add("ocxo_second_start_gnss_ns_raw", rt.last_diag.ocxo_second_start_gnss_ns_raw);
      s.add("ocxo_second_end_gnss_ns_raw", rt.last_diag.ocxo_second_end_gnss_ns_raw);
      s.add("ocxo_second_start_dwt_raw", rt.last_diag.ocxo_second_start_dwt_raw);
      s.add("ocxo_second_end_dwt_raw", rt.last_diag.ocxo_second_end_dwt_raw);
      s.add("ocxo_second_start_gnss_ns_final", rt.last_diag.ocxo_second_start_gnss_ns_final);
      s.add("ocxo_second_end_gnss_ns_final", rt.last_diag.ocxo_second_end_gnss_ns_final);
      s.add("ocxo_second_start_dwt_final", rt.last_diag.ocxo_second_start_dwt_final);
      s.add("ocxo_second_end_dwt_final", rt.last_diag.ocxo_second_end_dwt_final);
      s.add("ocxo_second_start_raw_minus_final_ns", rt.last_diag.ocxo_second_start_raw_minus_final_ns);
      s.add("ocxo_second_end_raw_minus_final_ns", rt.last_diag.ocxo_second_end_raw_minus_final_ns);
    }

    subscribers.add(s);
  }
  p.add_array("subscribers", subscribers);

  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr    }
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id    = "INTERRUPT",
  .commands      = INTERRUPT_COMMANDS,
  .subscriptions = nullptr
};

void process_interrupt_register(void) {
  process_register("INTERRUPT", &INTERRUPT_PROCESS);
}
