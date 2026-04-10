// ============================================================================
// process_interrupt.cpp — provider authority + shared pre-spin (QTimer v23)
// ============================================================================
//
// process_interrupt is the sole authority for all precision interrupt hardware.
//
// Corrected timing architecture:
//
//   • QTimer1 CH0+CH1 — GNSS VCLOCK 10 MHz on pin 10 (capture-only from here)
//   • QTimer3 CH2     — OCXO1 10 MHz external clock on pin 14 (16-bit)
//   • QTimer3 CH3     — OCXO2 10 MHz external clock on pin 15 (16-bit)
//   • GPIO6789        — PPS rising edge on pin 1
//
// Core laws:
//
//   • NEVER read a clock to know what time it is.
//   • Interrupts are the sole lawful timing truth.
//   • DWT is the precision substrate.
//   • GNSS nanoseconds are the public timing language.
//   • process_interrupt owns provider custody, ISR vectors, pre-spin,
//     and canonical event reconstruction.
//   • TimePop owns QTimer1 scheduling and remains independent.
//   • OCXO QTimer3 channels are 16-bit only, so one-second OCXO events are
//     reconstructed in software from lawful compare interrupts.
//   • The exported OCXO counter32_at_event is NOT a live hardware readout.
//     It is a software-extended logical count derived strictly from interrupts.
//
// 16-bit OCXO reconstruction model:
//
//   One second at 10 MHz = 10,000,000 counts
//                        = 152 * 65536 + 38528
//
//   Therefore, for each OCXO lane:
//
//     • We choose a target low16 value.
//     • That target is seen once after 38,528 counts, then once every 65,536
//       counts thereafter.
//     • The 153rd lawful compare hit is exactly one OCXO second after the
//       prior second edge.
//     • Only THAT hit is published as an OCXO event.
//     • Intermediate compare hits are internal cadence only.
//
//   This preserves the doctrinal rule:
//
//     no ambient OCXO clock reads for truth;
//     only interrupt facts plus math.
//
// Prespin architecture (v23 — widened trap + direct diagnostics):
//
//   The prespin shadow-write loop runs directly in TimePop ISR context
//   (priority 16) via timepop_arm_ns(..., isr_callback=true).
//
//   Additional diagnostics now capture:
//     • the actual GNSS/DWT moment prespin began
//     • the event prediction error (actual event GNSS - target GNSS)
//     • the distance from prespin fire to actual event
//     • count of events that arrived after a prior prespin timeout
//
//   PRESPIN_LEAD_NS is temporarily widened to 100 us to help characterize
//   OCXO1 without starving the spin loop.
//
// OCXO next-second prediction (v24):
//
//   For OCXO1 / OCXO2 only, process_interrupt now tracks the last measured
//   lawful OCXO second in GNSS nanoseconds and uses that exact last-second
//   experience as the prediction for the next OCXO second.
//
//   This prediction is used ONLY for prespin scheduling. It is not canonical
//   timing truth and is not to be used for OCXO residual calculation.
//
// ============================================================================

#include "process_interrupt.h"
#include "debug.h"
#include "config.h"

#include "time.h"
#include "timepop.h"
#include "process.h"
#include "payload.h"
#include "tdc_correction.h"

#include <Arduino.h>
#include "imxrt.h"

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;

static constexpr uint64_t NS_PER_SECOND_U64      = 1000000000ULL;
static constexpr uint64_t PRESPIN_LEAD_NS        = 100000ULL;
static constexpr uint32_t PRESPIN_TIMEOUT_CYCLES = 180000;

static constexpr uint32_t PPS_ISR_FIXED_OVERHEAD_CYCLES = 48;

static inline uint64_t dwt_cycles_to_ns_runtime(uint32_t cycles) {
  const uint32_t f = F_CPU_ACTUAL ? F_CPU_ACTUAL : 1008000000U;
  return ((uint64_t)cycles * 1000000000ULL + (uint64_t)f / 2ULL) / (uint64_t)f;
}

// OCXO second decomposition in 16-bit QTimer domain.
static constexpr uint32_t OCXO_COUNTS_PER_SECOND  = 10000000U;
static constexpr uint32_t QTIMER16_MODULUS        = 65536U;
static constexpr uint32_t OCXO_SECOND_WHOLE_WRAPS = OCXO_COUNTS_PER_SECOND / QTIMER16_MODULUS; // 152
static constexpr uint32_t OCXO_SECOND_REMAINDER   = OCXO_COUNTS_PER_SECOND % QTIMER16_MODULUS; // 38528

static_assert(OCXO_SECOND_WHOLE_WRAPS == 152, "Unexpected OCXO whole-wrap decomposition");
static_assert(OCXO_SECOND_REMAINDER   == 38528, "Unexpected OCXO remainder decomposition");

// ============================================================================
// Pin assignments — authoritative for process_interrupt
// ============================================================================

static constexpr int OCXO1_PIN = 14;
static constexpr int OCXO2_PIN = 15;

// ============================================================================
// Shared prespin state — responsibility counter architecture
// ============================================================================

static volatile uint32_t g_prespin_shadow_dwt = 0;
static volatile uint32_t g_prespin_start_dwt = 0;
static volatile int32_t  g_prespin_responsibility_count = 0;
static volatile bool     g_prespin_spinning = false;

// ============================================================================
// QTimer1 PPS-aligned VCLOCK capture
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

// ============================================================================
// 16-bit OCXO QTimer runtime
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

  uint16_t second_target_low16 = 0;
  uint32_t cadence_hit_index = 0;
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
  uint32_t bootstrap_arm_failures = 0;

  uint16_t last_programmed_compare = 0;
  uint16_t last_initial_counter16 = 0;
  uint16_t last_second_target_low16 = 0;
};

// ============================================================================
// Subscriber descriptor + runtime state
// ============================================================================

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;

  uint32_t isr_overhead_cycles = 0;
  uint64_t period_ns = 0;
  bool     uses_prespin = false;
  bool     prespin_absolute_gnss = false;

  qtimer_clock_kind_t clock_kind = qtimer_clock_kind_t::NONE;
};

struct prespin_state_t {
  volatile bool active = false;

  uint64_t event_target_gnss_ns = 0;
  uint64_t prespin_target_gnss_ns = 0;

  timepop_handle_t handle = TIMEPOP_INVALID_HANDLE;

  uint32_t arm_count = 0;
  uint32_t complete_count = 0;
  uint32_t timeout_count = 0;
  uint32_t anomaly_count = 0;
  uint32_t late_after_timeout_count = 0;

  uint64_t last_schedule_now_gnss_ns = 0;
  uint64_t last_schedule_delay_ns = 0;
  int64_t  last_prespin_margin_ns = 0;
  uint64_t last_schedule_event_target_gnss_ns = 0;
  uint64_t last_schedule_prespin_target_gnss_ns = 0;

  uint32_t snap_start_dwt = 0;
  uint32_t snap_shadow_dwt = 0;
  uint32_t snap_dwt_isr_entry_raw = 0;
  uint64_t snap_prespin_fire_gnss_ns = 0;
  uint32_t snap_prespin_fire_dwt = 0;
  bool     snap_fired = false;
  bool     snap_timed_out = false;
  uint32_t snap_timeout_elapsed_cycles = 0;

  int64_t  last_event_error_ns = 0;
  int64_t  last_event_minus_prespin_fire_ns = 0;
  uint32_t last_late_after_timeout_overage_cycles = 0;
  uint64_t last_late_after_timeout_overage_ns = 0;
  uint32_t max_late_after_timeout_overage_cycles = 0;
  uint64_t max_late_after_timeout_overage_ns = 0;
};

struct ocxo_prediction_state_t {
  bool     valid = false;
  uint64_t next_second_prediction_gnss_ns = 0;
  uint64_t last_measured_second_gnss_ns = 0;
  uint64_t previous_edge_gnss_ns = 0;
  uint32_t update_count = 0;
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

  prespin_state_t prespin {};
  ocxo_prediction_state_t ocxo_prediction {};
};

struct pps_drumbeat_state_t {
  bool     initialized = false;
  bool     zero_pending = false;
  uint64_t baseline_gnss_ns = 0;
  uint64_t next_event_target_gnss_ns = 0;
  uint64_t next_prespin_target_gnss_ns = 0;
  uint64_t next_index = 0;
  uint32_t generation = 0;
};

static pps_drumbeat_state_t g_pps_drumbeat = {};

struct interrupt_provider_diag_t {
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t miss_count = 0;
};

struct interrupt_anomaly_diag_t {
  uint32_t prespin_null_runtime = 0;
  uint32_t prespin_arm_failed = 0;
  uint32_t prespin_timeout = 0;
  uint32_t no_prespin_active = 0;
  uint32_t no_shadow_dwt = 0;
  uint32_t null_desc = 0;
  uint32_t ocxo_bad_channel = 0;
  uint32_t ocxo_spurious_irq = 0;

  uint32_t last_kind = 0;
  uint32_t last_detail0 = 0;
  uint32_t last_detail1 = 0;
  uint32_t last_detail2 = 0;
};

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS] = {};
static uint32_t g_subscriber_count = 0;

static bool g_interrupt_hw_ready = false;
static bool g_interrupt_runtime_ready = false;
static bool g_interrupt_irqs_enabled = false;

static interrupt_provider_diag_t g_gpio_diag = {};
static interrupt_anomaly_diag_t g_anomaly_diag = {};

static interrupt_subscriber_runtime_t* g_rt_pps   = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo1 = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2 = nullptr;

static qtimer16_ocxo_runtime_t g_clock_ocxo1 = {};
static qtimer16_ocxo_runtime_t g_clock_ocxo2 = {};

static const interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  {
    interrupt_subscriber_kind_t::PPS,
    "PPS",
    interrupt_provider_kind_t::GPIO6789,
    interrupt_lane_t::GPIO_EDGE,
    PPS_ISR_FIXED_OVERHEAD_CYCLES,
    NS_PER_SECOND_U64,
    true,
    true,
    qtimer_clock_kind_t::VCLOCK,
  },
  {
    interrupt_subscriber_kind_t::OCXO1,
    "OCXO1",
    interrupt_provider_kind_t::QTIMER3,
    interrupt_lane_t::QTIMER3_CH2_COMP,
    QTIMER_ISR_FIXED_OVERHEAD,
    NS_PER_SECOND_U64,
    true,
    false,
    qtimer_clock_kind_t::OCXO1,
  },
  {
    interrupt_subscriber_kind_t::OCXO2,
    "OCXO2",
    interrupt_provider_kind_t::QTIMER3,
    interrupt_lane_t::QTIMER3_CH3_COMP,
    QTIMER_ISR_FIXED_OVERHEAD,
    NS_PER_SECOND_U64,
    true,
    false,
    qtimer_clock_kind_t::OCXO2,
  },
};

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

static void record_anomaly(uint32_t& counter,
                           interrupt_subscriber_kind_t kind,
                           uint32_t detail0 = 0,
                           uint32_t detail1 = 0,
                           uint32_t detail2 = 0) {
  counter++;
  g_anomaly_diag.last_kind = (uint32_t)kind;
  g_anomaly_diag.last_detail0 = detail0;
  g_anomaly_diag.last_detail1 = detail1;
  g_anomaly_diag.last_detail2 = detail2;
}

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

static void qtimer16_ocxo_arm_bootstrap(qtimer16_ocxo_runtime_t& clock) {
  const uint16_t now16 = clock.hw.module->CH[clock.hw.channel].CNTR;

  clock.phase_bootstrapped = true;
  clock.bootstrap_count++;
  clock.cadence_hit_index = 0;
  clock.cadence_hits_since_second = 0;
  clock.last_initial_counter16 = now16;

  clock.second_target_low16 = (uint16_t)(now16 + (uint16_t)OCXO_SECOND_REMAINDER);
  clock.last_second_target_low16 = clock.second_target_low16;
  clock.last_programmed_compare = clock.second_target_low16;

  qtimer3_program_compare(clock.hw.channel, clock.second_target_low16);
}

static void qtimer16_ocxo_advance_to_next_second_target(qtimer16_ocxo_runtime_t& clock) {
  clock.second_target_low16 = (uint16_t)(clock.second_target_low16 + (uint16_t)OCXO_SECOND_REMAINDER);
  clock.last_second_target_low16 = clock.second_target_low16;
  clock.last_programmed_compare = clock.second_target_low16;
  qtimer3_program_compare(clock.hw.channel, clock.second_target_low16);
}

static void qtimer16_ocxo_rearm_same_second_target(qtimer16_ocxo_runtime_t& clock) {
  clock.last_programmed_compare = clock.second_target_low16;
  qtimer3_program_compare(clock.hw.channel, clock.second_target_low16);
}

static bool qtimer16_ocxo_consume_cadence_irq(qtimer16_ocxo_runtime_t& clock,
                                              uint16_t counter16_snapshot,
                                              uint32_t& logical_counter32_at_event) {
  clock.last_counter16_at_irq = counter16_snapshot;
  clock.cadence_hits_total++;
  clock.cadence_hits_since_second++;
  clock.cadence_hit_index++;

  if (clock.cadence_hit_index < (OCXO_SECOND_WHOLE_WRAPS + 1U)) {
    qtimer16_ocxo_rearm_same_second_target(clock);
    return false;
  }

  clock.logical_count32_at_last_second_edge += OCXO_COUNTS_PER_SECOND;
  logical_counter32_at_event = clock.logical_count32_at_last_second_edge;
  clock.event_count++;
  clock.cadence_hit_index = 0;
  clock.cadence_hits_since_second = 0;
  qtimer16_ocxo_advance_to_next_second_target(clock);
  return true;
}

static void pps_gpio_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  if (IMXRT_TMR3.CH[2].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch2_irq(dwt_raw);
  if (IMXRT_TMR3.CH[3].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch3_irq(dwt_raw);
}

static const char* prespin_timer_name(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::PPS:   return "PPS_PRESPIN";
    case interrupt_subscriber_kind_t::OCXO1: return "OCXO1_PRESPIN";
    case interrupt_subscriber_kind_t::OCXO2: return "OCXO2_PRESPIN";
    default:                                 return "";
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

static void pps_drumbeat_schedule_from_edge(interrupt_subscriber_runtime_t& rt,
                                            uint64_t pps_edge_gnss_ns);
static void pps_drumbeat_bootstrap(interrupt_subscriber_runtime_t& rt);
static void pps_drumbeat_consume_edge(interrupt_subscriber_runtime_t& rt);

static void prespin_isr_callback(timepop_ctx_t* ctx,
                                 timepop_diag_t*,
                                 void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) {
    record_anomaly(g_anomaly_diag.prespin_null_runtime, interrupt_subscriber_kind_t::NONE);
    return;
  }
  if (!rt->active) return;

  prespin_state_t& ps = rt->prespin;
  ps.handle = TIMEPOP_INVALID_HANDLE;
  ps.active = true;
  ps.arm_count++;

  ps.snap_start_dwt = 0;
  ps.snap_shadow_dwt = 0;
  ps.snap_dwt_isr_entry_raw = 0;
  ps.snap_prespin_fire_gnss_ns = (ctx && ctx->fire_gnss_ns >= 0) ? (uint64_t)ctx->fire_gnss_ns : 0ULL;
  ps.snap_prespin_fire_dwt = ARM_DWT_CYCCNT;
  ps.snap_fired = false;
  ps.snap_timed_out = false;

  g_prespin_responsibility_count++;
  if (g_prespin_spinning) return;

  g_prespin_spinning = true;
  g_prespin_shadow_dwt = 0;
  g_prespin_start_dwt = 0;

  const uint32_t loop_start_dwt = ARM_DWT_CYCCNT;
  g_prespin_start_dwt = loop_start_dwt;

  volatile uint32_t* const shadow = &g_prespin_shadow_dwt;
  volatile int32_t*  const count  = &g_prespin_responsibility_count;

  for (;;) {
    *shadow = ARM_DWT_CYCCNT;

    if (*count <= 0) {
      g_prespin_spinning = false;
      return;
    }

    if ((ARM_DWT_CYCCNT - loop_start_dwt) > PRESPIN_TIMEOUT_CYCLES) {
      g_prespin_responsibility_count = 0;
      g_prespin_spinning = false;

      for (uint32_t i = 0; i < g_subscriber_count; i++) {
        interrupt_subscriber_runtime_t& sub = g_subscribers[i];
        if (!sub.desc || !sub.desc->uses_prespin) continue;
        if (!sub.prespin.active) continue;

        sub.prespin.snap_timed_out = true;
        sub.prespin.snap_start_dwt = loop_start_dwt;
        sub.prespin.snap_shadow_dwt = g_prespin_shadow_dwt;
        sub.prespin.snap_timeout_elapsed_cycles = ARM_DWT_CYCCNT - loop_start_dwt;
        sub.prespin.timeout_count++;
        sub.prespin.anomaly_count++;
        sub.prespin.active = false;

        record_anomaly(g_anomaly_diag.prespin_timeout,
                       sub.desc->kind,
                       g_prespin_shadow_dwt,
                       sub.prespin.arm_count,
                       sub.prespin.timeout_count);
      }
      return;
    }
  }
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {
  // retained for header compatibility
}

static void schedule_next_prespin_from_anchor(interrupt_subscriber_runtime_t& rt,
                                              uint64_t anchor_gnss_ns,
                                              uint64_t offset_gnss_ns) {
  if (!rt.desc || !rt.desc->uses_prespin) return;
  if (!rt.active) return;

  prespin_state_t& ps = rt.prespin;
  ps.event_target_gnss_ns = anchor_gnss_ns + offset_gnss_ns;
  ps.prespin_target_gnss_ns =
      (ps.event_target_gnss_ns >= PRESPIN_LEAD_NS)
          ? (ps.event_target_gnss_ns - PRESPIN_LEAD_NS)
          : 0ULL;

  ps.last_schedule_event_target_gnss_ns = ps.event_target_gnss_ns;
  ps.last_schedule_prespin_target_gnss_ns = ps.prespin_target_gnss_ns;
  ps.last_schedule_now_gnss_ns = anchor_gnss_ns;
  ps.last_schedule_delay_ns = offset_gnss_ns - PRESPIN_LEAD_NS;
  ps.last_prespin_margin_ns = (int64_t)(offset_gnss_ns - PRESPIN_LEAD_NS);

  timepop_handle_t h =
      timepop_arm_ns((int64_t)ps.prespin_target_gnss_ns,
                     0,
                     prespin_isr_callback,
                     &rt,
                     prespin_timer_name(rt.desc->kind),
                     true);

  if (h == TIMEPOP_INVALID_HANDLE) {
    ps.anomaly_count++;
    record_anomaly(g_anomaly_diag.prespin_arm_failed,
                   rt.desc->kind,
                   (uint32_t)(ps.prespin_target_gnss_ns & 0xFFFFFFFFu),
                   (uint32_t)(ps.prespin_target_gnss_ns >> 32),
                   ps.arm_count);
    return;
  }

  ps.handle = h;
}

static void pps_drumbeat_schedule_from_edge(interrupt_subscriber_runtime_t& rt,
                                            uint64_t pps_edge_gnss_ns) {
  const uint64_t next_event_target_gnss_ns = pps_edge_gnss_ns + NS_PER_SECOND_U64;

  g_pps_drumbeat.next_event_target_gnss_ns = next_event_target_gnss_ns;
  g_pps_drumbeat.next_prespin_target_gnss_ns =
      (next_event_target_gnss_ns >= PRESPIN_LEAD_NS)
          ? (next_event_target_gnss_ns - PRESPIN_LEAD_NS)
          : 0ULL;

  schedule_next_prespin_from_anchor(rt, pps_edge_gnss_ns, NS_PER_SECOND_U64);
}

static void clear_prespin_schedule_state(interrupt_subscriber_runtime_t& rt) {
  if (rt.prespin.handle != TIMEPOP_INVALID_HANDLE) {
    timepop_cancel(rt.prespin.handle);
    rt.prespin.handle = TIMEPOP_INVALID_HANDLE;
  }

  rt.prespin.event_target_gnss_ns = 0;
  rt.prespin.prespin_target_gnss_ns = 0;
  rt.prespin.last_schedule_now_gnss_ns = 0;
  rt.prespin.last_schedule_delay_ns = 0;
  rt.prespin.last_prespin_margin_ns = 0;
  rt.prespin.last_schedule_event_target_gnss_ns = 0;
  rt.prespin.last_schedule_prespin_target_gnss_ns = 0;
}

static void schedule_ocxo_from_edge(interrupt_subscriber_runtime_t& rt,
                                    uint64_t edge_gnss_ns) {
  if (edge_gnss_ns == 0) return;

  ocxo_prediction_state_t& pred = rt.ocxo_prediction;

  if (pred.previous_edge_gnss_ns != 0 && edge_gnss_ns > pred.previous_edge_gnss_ns) {
    pred.last_measured_second_gnss_ns = edge_gnss_ns - pred.previous_edge_gnss_ns;
    pred.next_second_prediction_gnss_ns = pred.last_measured_second_gnss_ns;
    pred.valid = true;
    pred.update_count++;
  }

  pred.previous_edge_gnss_ns = edge_gnss_ns;

  if (!pred.valid || pred.next_second_prediction_gnss_ns == 0) {
    clear_prespin_schedule_state(rt);
    return;
  }

  schedule_next_prespin_from_anchor(rt, edge_gnss_ns, pred.next_second_prediction_gnss_ns);
}

static void pps_drumbeat_bootstrap(interrupt_subscriber_runtime_t&) {
  g_pps_drumbeat.next_event_target_gnss_ns = 0;
  g_pps_drumbeat.next_prespin_target_gnss_ns = 0;
  if (!g_pps_drumbeat.initialized) {
    g_pps_drumbeat.baseline_gnss_ns = 0;
    g_pps_drumbeat.next_index = 0;
  }
}

static void pps_drumbeat_consume_edge(interrupt_subscriber_runtime_t& rt) {
  if (!rt.has_fired) return;
  const uint64_t pps_edge_gnss_ns = rt.last_event.gnss_ns_at_event;
  if (pps_edge_gnss_ns == 0) return;

  const bool establishing_baseline = !g_pps_drumbeat.initialized || g_pps_drumbeat.zero_pending;
  g_pps_drumbeat.initialized = true;
  g_pps_drumbeat.zero_pending = false;
  g_pps_drumbeat.baseline_gnss_ns = pps_edge_gnss_ns;
  g_pps_drumbeat.next_index = 1;
  if (establishing_baseline) g_pps_drumbeat.generation++;

  pps_drumbeat_schedule_from_edge(rt, pps_edge_gnss_ns);
}

static void fill_diag_common(interrupt_subscriber_runtime_t& rt,
                             interrupt_capture_diag_t& diag,
                             const interrupt_event_t& event,
                             uint32_t dwt_isr_entry_raw,
                             uint32_t approach_cycles,
                             uint32_t shadow_to_isr_cycles) {
  diag.provider = rt.desc->provider;
  diag.lane = rt.desc->lane;
  diag.kind = rt.desc->kind;
  diag.prespin_target_gnss_ns = rt.prespin.prespin_target_gnss_ns;
  diag.event_target_gnss_ns = rt.prespin.event_target_gnss_ns;
  diag.prespin_active = rt.prespin.active;
  diag.prespin_fired = rt.prespin.snap_fired;
  diag.prespin_timed_out = rt.prespin.snap_timed_out;
  diag.shadow_dwt = rt.prespin.snap_shadow_dwt;
  diag.dwt_isr_entry_raw = dwt_isr_entry_raw;
  diag.approach_cycles = approach_cycles;
  diag.shadow_to_isr_cycles = shadow_to_isr_cycles;
  diag.dwt_at_event_adjusted = event.dwt_at_event;
  diag.gnss_ns_at_event = event.gnss_ns_at_event;
  diag.counter32_at_event = event.counter32_at_event;
  diag.dwt_event_correction_cycles = event.dwt_event_correction_cycles;
  diag.prespin_arm_count = rt.prespin.arm_count;
  diag.prespin_complete_count = rt.prespin.complete_count;
  diag.prespin_timeout_count = rt.prespin.timeout_count;
  diag.anomaly_count = rt.prespin.anomaly_count;
}

static void maybe_dispatch_event(interrupt_subscriber_runtime_t& rt) {
  if (!rt.subscribed || !rt.sub.on_event) return;
  rt.sub.on_event(rt.last_event, &rt.last_diag, rt.sub.user_data);
}

static void deferred_dispatch_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;

  maybe_dispatch_event(*rt);
  if (rt->desc->kind == interrupt_subscriber_kind_t::PPS) {
    pps_drumbeat_consume_edge(*rt);
  } else {
    schedule_ocxo_from_edge(*rt, rt->last_event.gnss_ns_at_event);
  }
}

static void handle_event(interrupt_subscriber_runtime_t& rt,
                         uint32_t dwt_isr_entry_raw,
                         uint32_t counter32_at_event) {
  prespin_state_t& ps = rt.prespin;

  if (ps.active) {
    g_prespin_responsibility_count--;
    ps.active = false;
    ps.complete_count++;
    ps.last_late_after_timeout_overage_cycles = 0;
    ps.last_late_after_timeout_overage_ns = 0;
  } else if (ps.snap_timed_out) {
    ps.late_after_timeout_count++;
    const uint32_t timeout_boundary_dwt = ps.snap_start_dwt + PRESPIN_TIMEOUT_CYCLES;
    const uint32_t overage_cycles = dwt_isr_entry_raw - timeout_boundary_dwt;
    const uint64_t overage_ns = dwt_cycles_to_ns_runtime(overage_cycles);
    ps.last_late_after_timeout_overage_cycles = overage_cycles;
    ps.last_late_after_timeout_overage_ns = overage_ns;
    if (overage_cycles > ps.max_late_after_timeout_overage_cycles) {
      ps.max_late_after_timeout_overage_cycles = overage_cycles;
      ps.max_late_after_timeout_overage_ns = overage_ns;
    }
  } else {
    ps.last_late_after_timeout_overage_cycles = 0;
    ps.last_late_after_timeout_overage_ns = 0;
  }

  ps.snap_start_dwt = g_prespin_start_dwt;
  ps.snap_shadow_dwt = g_prespin_shadow_dwt;
  ps.snap_dwt_isr_entry_raw = dwt_isr_entry_raw;
  ps.snap_fired = true;
  ps.snap_timed_out = false;
  ps.snap_timeout_elapsed_cycles = 0;

  const bool prespin_expected =
      (rt.desc->kind == interrupt_subscriber_kind_t::PPS) ||
      rt.ocxo_prediction.valid;

  uint32_t approach_cycles = 0;
  uint32_t shadow_to_isr_cycles = 0;
  if (ps.snap_start_dwt != 0) approach_cycles = dwt_isr_entry_raw - ps.snap_start_dwt;
  if (ps.snap_shadow_dwt != 0) {
    shadow_to_isr_cycles = dwt_isr_entry_raw - ps.snap_shadow_dwt;
  } else if (prespin_expected) {
    ps.anomaly_count++;
    record_anomaly(g_anomaly_diag.no_shadow_dwt,
                   rt.desc->kind,
                   dwt_isr_entry_raw,
                   ps.arm_count,
                   ps.complete_count);
  }

  if (prespin_expected && !g_prespin_spinning && ps.snap_start_dwt == 0 && ps.snap_shadow_dwt == 0) {
    ps.anomaly_count++;
    record_anomaly(g_anomaly_diag.no_prespin_active,
                   rt.desc->kind,
                   dwt_isr_entry_raw,
                   ps.snap_shadow_dwt,
                   ps.arm_count);
  }

  const uint32_t correction = rt.desc->isr_overhead_cycles;
  const uint32_t dwt_at_event = dwt_isr_entry_raw - correction;

  interrupt_event_t event {};
  event.kind = rt.desc->kind;
  event.provider = rt.desc->provider;
  event.lane = rt.desc->lane;
  event.dwt_at_event = dwt_at_event;
  event.counter32_at_event = counter32_at_event;
  event.dwt_event_correction_cycles = correction;

  if (rt.desc->kind == interrupt_subscriber_kind_t::PPS) {
    const int64_t pps_index = (int64_t)rt.event_count;
    event.gnss_ns_at_event = (uint64_t)(pps_index * 1000000000LL);
    event.status = interrupt_event_status_t::OK;
  } else {
    const int64_t gnss_ns_signed = time_dwt_to_gnss_ns(dwt_at_event);
    if (gnss_ns_signed >= 0) {
      event.gnss_ns_at_event = (uint64_t)gnss_ns_signed;
      event.status = interrupt_event_status_t::OK;
    } else {
      event.gnss_ns_at_event = 0;
      event.status = interrupt_event_status_t::HOLD;
    }
  }

  if (ps.event_target_gnss_ns != 0 && event.gnss_ns_at_event != 0) {
    ps.last_event_error_ns = (int64_t)event.gnss_ns_at_event - (int64_t)ps.event_target_gnss_ns;
  } else {
    ps.last_event_error_ns = 0;
  }

  if (ps.snap_prespin_fire_gnss_ns != 0 && event.gnss_ns_at_event != 0) {
    ps.last_event_minus_prespin_fire_ns =
        (int64_t)event.gnss_ns_at_event - (int64_t)ps.snap_prespin_fire_gnss_ns;
  } else {
    ps.last_event_minus_prespin_fire_ns = 0;
  }

  interrupt_capture_diag_t diag {};
  fill_diag_common(rt, diag, event, dwt_isr_entry_raw, approach_cycles, shadow_to_isr_cycles);

  rt.last_event = event;
  rt.last_diag = diag;
  rt.has_fired = true;
  rt.irq_count++;
  rt.dispatch_count++;
  rt.event_count++;

  timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
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
    record_anomaly(g_anomaly_diag.ocxo_bad_channel,
                   rt.desc->kind,
                   clock.hw.channel,
                   counter16_snapshot,
                   0);
    clock.miss_count++;
    return;
  }

  uint32_t logical_counter32_at_event = 0;
  const bool is_second_edge = qtimer16_ocxo_consume_cadence_irq(clock,
                                                                counter16_snapshot,
                                                                logical_counter32_at_event);
  if (!is_second_edge) return;

  clock.dispatch_count++;
  handle_event(rt, dwt_raw, logical_counter32_at_event);
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
  rt->prespin.active = false;
  rt->ocxo_prediction = {};
  clear_prespin_schedule_state(*rt);

  if (kind == interrupt_subscriber_kind_t::PPS) {
    debug_log("pps.start", "requested");
    pps_drumbeat_bootstrap(*rt);
    return true;
  }

  qtimer16_ocxo_runtime_t* clock = ocxo_clock_runtime_for(rt->desc->clock_kind);
  if (!clock || !clock->initialized) return false;

  clock->active = true;
  clock->start_count++;
  clock->logical_count32_at_last_second_edge = 0;
  clock->phase_bootstrapped = false;
  clock->cadence_hit_index = 0;
  clock->cadence_hits_since_second = 0;

  qtimer16_ocxo_arm_bootstrap(*clock);
  return true;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;

  rt->active = false;
  rt->stop_count++;
  if (rt->prespin.handle != TIMEPOP_INVALID_HANDLE) {
    timepop_cancel(rt->prespin.handle);
    rt->prespin.handle = TIMEPOP_INVALID_HANDLE;
  }
  rt->prespin.active = false;
  rt->ocxo_prediction = {};
  clear_prespin_schedule_state(*rt);

  qtimer16_ocxo_runtime_t* clock = ocxo_clock_runtime_for(rt->desc->clock_kind);
  if (clock) {
    clock->active = false;
    clock->stop_count++;
    qtimer3_disable_compare(clock->hw.channel);
  }
  return true;
}

void interrupt_request_pps_zero(void) {
  g_pps_drumbeat.zero_pending = true;
}

bool interrupt_pps_zero_pending(void) {
  return g_pps_drumbeat.zero_pending;
}

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

  g_prespin_shadow_dwt = 0;
  g_prespin_start_dwt = 0;
  g_prespin_responsibility_count = 0;
  g_prespin_spinning = false;

  g_pps_drumbeat = {};
  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 0);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  g_interrupt_irqs_enabled = true;
}

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready", g_interrupt_runtime_ready);
  p.add("irqs_enabled", g_interrupt_irqs_enabled);
  p.add("subscriber_count", g_subscriber_count);

  p.add("gpio_irq_count", g_gpio_diag.irq_count);
  p.add("gpio_dispatch_count", g_gpio_diag.dispatch_count);
  p.add("gpio_miss_count", g_gpio_diag.miss_count);

  p.add("pps_drumbeat_initialized", g_pps_drumbeat.initialized);
  p.add("pps_zero_pending", g_pps_drumbeat.zero_pending);
  p.add("pps_baseline_gnss_ns", g_pps_drumbeat.baseline_gnss_ns);
  p.add("pps_next_event_target_gnss_ns", g_pps_drumbeat.next_event_target_gnss_ns);
  p.add("pps_next_prespin_target_gnss_ns", g_pps_drumbeat.next_prespin_target_gnss_ns);
  p.add("pps_next_index", g_pps_drumbeat.next_index);
  p.add("pps_generation", g_pps_drumbeat.generation);

  p.add("ocxo1_next_second_prediction_valid", g_rt_ocxo1 ? g_rt_ocxo1->ocxo_prediction.valid : false);
  p.add("ocxo1_next_second_prediction_gnss_ns", g_rt_ocxo1 ? g_rt_ocxo1->ocxo_prediction.next_second_prediction_gnss_ns : 0ULL);
  p.add("ocxo1_last_measured_second_gnss_ns", g_rt_ocxo1 ? g_rt_ocxo1->ocxo_prediction.last_measured_second_gnss_ns : 0ULL);
  p.add("ocxo1_prediction_update_count", g_rt_ocxo1 ? g_rt_ocxo1->ocxo_prediction.update_count : 0U);
  p.add("ocxo2_next_second_prediction_valid", g_rt_ocxo2 ? g_rt_ocxo2->ocxo_prediction.valid : false);
  p.add("ocxo2_next_second_prediction_gnss_ns", g_rt_ocxo2 ? g_rt_ocxo2->ocxo_prediction.next_second_prediction_gnss_ns : 0ULL);
  p.add("ocxo2_last_measured_second_gnss_ns", g_rt_ocxo2 ? g_rt_ocxo2->ocxo_prediction.last_measured_second_gnss_ns : 0ULL);
  p.add("ocxo2_prediction_update_count", g_rt_ocxo2 ? g_rt_ocxo2->ocxo_prediction.update_count : 0U);

  p.add("prespin_lead_ns", (uint64_t)PRESPIN_LEAD_NS);
  p.add("prespin_timeout_cycles", PRESPIN_TIMEOUT_CYCLES);
  p.add("prespin_responsibility_count", g_prespin_responsibility_count);
  p.add("prespin_spinning", g_prespin_spinning);

  p.add("anomaly_prespin_null_runtime", g_anomaly_diag.prespin_null_runtime);
  p.add("anomaly_prespin_arm_failed", g_anomaly_diag.prespin_arm_failed);
  p.add("anomaly_prespin_timeout", g_anomaly_diag.prespin_timeout);
  p.add("anomaly_no_prespin_active", g_anomaly_diag.no_prespin_active);
  p.add("anomaly_no_shadow_dwt", g_anomaly_diag.no_shadow_dwt);
  p.add("anomaly_null_desc", g_anomaly_diag.null_desc);
  p.add("anomaly_ocxo_bad_channel", g_anomaly_diag.ocxo_bad_channel);
  p.add("anomaly_ocxo_spurious_irq", g_anomaly_diag.ocxo_spurious_irq);

  p.add("anomaly_last_kind", g_anomaly_diag.last_kind);
  p.add("anomaly_last_detail0", g_anomaly_diag.last_detail0);
  p.add("anomaly_last_detail1", g_anomaly_diag.last_detail1);
  p.add("anomaly_last_detail2", g_anomaly_diag.last_detail2);

  PayloadArray subscribers;
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    const interrupt_subscriber_runtime_t& rt = g_subscribers[i];
    if (!rt.desc) continue;

    Payload s;
    s.add("slot", i);
    s.add("kind", interrupt_subscriber_kind_str(rt.desc->kind));
    s.add("provider", interrupt_provider_kind_str(rt.desc->provider));
    s.add("lane", interrupt_lane_str(rt.desc->lane));
    s.add("subscribed", rt.subscribed);
    s.add("active", rt.active);
    s.add("start_count", rt.start_count);
    s.add("stop_count", rt.stop_count);
    s.add("irq_count", rt.irq_count);
    s.add("dispatch_count", rt.dispatch_count);
    s.add("event_count", rt.event_count);

    s.add("prespin_active", rt.prespin.active);
    s.add("prespin_fired", rt.prespin.snap_fired);
    s.add("prespin_timed_out", rt.prespin.snap_timed_out);
    s.add("prespin_start_dwt", rt.prespin.snap_start_dwt);
    s.add("shadow_dwt", rt.prespin.snap_shadow_dwt);
    s.add("dwt_isr_entry_raw", rt.prespin.snap_dwt_isr_entry_raw);
    s.add("prespin_fire_gnss_ns", rt.prespin.snap_prespin_fire_gnss_ns);
    s.add("prespin_fire_dwt", rt.prespin.snap_prespin_fire_dwt);
    s.add("prespin_timeout_elapsed_cycles", rt.prespin.snap_timeout_elapsed_cycles);

    s.add("prespin_target_gnss_ns", rt.prespin.prespin_target_gnss_ns);
    s.add("event_target_gnss_ns", rt.prespin.event_target_gnss_ns);
    s.add("prespin_arm_count", rt.prespin.arm_count);
    s.add("prespin_complete_count", rt.prespin.complete_count);
    s.add("prespin_timeout_count", rt.prespin.timeout_count);
    s.add("late_after_timeout_count", rt.prespin.late_after_timeout_count);
    s.add("last_late_after_timeout_overage_cycles", rt.prespin.last_late_after_timeout_overage_cycles);
    s.add("last_late_after_timeout_overage_ns", rt.prespin.last_late_after_timeout_overage_ns);
    s.add("max_late_after_timeout_overage_cycles", rt.prespin.max_late_after_timeout_overage_cycles);
    s.add("max_late_after_timeout_overage_ns", rt.prespin.max_late_after_timeout_overage_ns);
    s.add("anomaly_count", rt.prespin.anomaly_count);

    s.add("ocxo_prediction_valid", rt.ocxo_prediction.valid);
    s.add("ocxo_next_second_prediction_gnss_ns", rt.ocxo_prediction.next_second_prediction_gnss_ns);
    s.add("ocxo_last_measured_second_gnss_ns", rt.ocxo_prediction.last_measured_second_gnss_ns);
    s.add("ocxo_prediction_previous_edge_gnss_ns", rt.ocxo_prediction.previous_edge_gnss_ns);
    s.add("ocxo_prediction_update_count", rt.ocxo_prediction.update_count);

    s.add("last_schedule_now_gnss_ns", rt.prespin.last_schedule_now_gnss_ns);
    s.add("last_schedule_delay_ns", rt.prespin.last_schedule_delay_ns);
    s.add("last_prespin_margin_ns", rt.prespin.last_prespin_margin_ns);
    s.add("last_schedule_event_target_gnss_ns", rt.prespin.last_schedule_event_target_gnss_ns);
    s.add("last_schedule_prespin_target_gnss_ns", rt.prespin.last_schedule_prespin_target_gnss_ns);

    s.add("last_approach_cycles", rt.last_diag.approach_cycles);
    s.add("last_shadow_to_isr_cycles", rt.last_diag.shadow_to_isr_cycles);
    s.add("last_event_error_ns", rt.prespin.last_event_error_ns);
    s.add("last_event_minus_prespin_fire_ns", rt.prespin.last_event_minus_prespin_fire_ns);

    s.add("last_dwt_at_event", rt.last_event.dwt_at_event);
    s.add("last_gnss_ns_at_event", rt.last_event.gnss_ns_at_event);
    s.add("last_counter32_at_event", rt.last_event.counter32_at_event);
    s.add("last_correction_cycles", rt.last_event.dwt_event_correction_cycles);
    s.add("last_status", (uint32_t)rt.last_event.status);
    subscribers.add(s);
  }
  p.add_array("subscribers", subscribers);

  PayloadArray ocxo_clocks;
  auto add_ocxo_clock = [&](const qtimer16_ocxo_runtime_t& clock) {
    Payload c;
    c.add("name", clock.name ? clock.name : "");
    c.add("active", clock.active);

    const interrupt_subscriber_runtime_t* rt = nullptr;
    if (clock.kind == qtimer_clock_kind_t::OCXO1) rt = g_rt_ocxo1;
    if (clock.kind == qtimer_clock_kind_t::OCXO2) rt = g_rt_ocxo2;
    c.add("next_second_prediction_valid", rt ? rt->ocxo_prediction.valid : false);
    c.add("next_second_prediction_gnss_ns", rt ? rt->ocxo_prediction.next_second_prediction_gnss_ns : 0ULL);
    c.add("last_measured_second_gnss_ns", rt ? rt->ocxo_prediction.last_measured_second_gnss_ns : 0ULL);
    c.add("prediction_update_count", rt ? rt->ocxo_prediction.update_count : 0U);
    c.add("initialized", clock.initialized);
    c.add("start_count", clock.start_count);
    c.add("stop_count", clock.stop_count);
    c.add("irq_count", clock.irq_count);
    c.add("dispatch_count", clock.dispatch_count);
    c.add("miss_count", clock.miss_count);
    c.add("event_count", clock.event_count);
    c.add("bootstrap_count", clock.bootstrap_count);
    c.add("bootstrap_arm_failures", clock.bootstrap_arm_failures);
    c.add("phase_bootstrapped", clock.phase_bootstrapped);
    c.add("cadence_hit_index", clock.cadence_hit_index);
    c.add("cadence_hits_total", clock.cadence_hits_total);
    c.add("cadence_hits_since_second", clock.cadence_hits_since_second);
    c.add("logical_count32_at_last_second_edge", clock.logical_count32_at_last_second_edge);
    c.add("last_counter16_at_irq", (uint32_t)clock.last_counter16_at_irq);
    c.add("last_programmed_compare", (uint32_t)clock.last_programmed_compare);
    c.add("last_initial_counter16", (uint32_t)clock.last_initial_counter16);
    c.add("last_second_target_low16", (uint32_t)clock.last_second_target_low16);
    c.add("counter16_now", (uint32_t)clock.hw.module->CH[clock.hw.channel].CNTR);
    c.add("comp1_now", (uint32_t)clock.hw.module->CH[clock.hw.channel].COMP1);
    c.add("csctrl_now", (uint32_t)clock.hw.module->CH[clock.hw.channel].CSCTRL);
    ocxo_clocks.add(c);
  };

  add_ocxo_clock(g_clock_ocxo1);
  add_ocxo_clock(g_clock_ocxo2);
  p.add_array("ocxo_qtimer16_clocks", ocxo_clocks);

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
