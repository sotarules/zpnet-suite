// ============================================================================
// process_interrupt.cpp — provider authority + shared pre-spin
// ============================================================================
//
// process_interrupt is the sole authority for all precision interrupt hardware.
//
// Owned hardware:
//   • GPT1 external clock (OCXO1 10 MHz) — timer bring-up, ISR vector, OCR3
//   • GPT2 external clock (OCXO2 10 MHz) — timer bring-up, ISR vector, OCR3
//   • GPIO6789 (PPS rising edge) — attachInterrupt, ISR vector
//   • All ISR vectors capture ARM_DWT_CYCCNT as their first instruction
//
// Owned machinery:
//   • Shared pre-spin shadow-write loop with responsibility counter
//   • Event reconstruction (DWT correction, GNSS ns projection)
//   • OCXO quantization discovery + best-estimate event reconstruction
//   • Subscriber dispatch with canonical event + diagnostics
//   • PPS geared drumbeat scheduling from sacred PPS baseline
//   • PPS ZERO-request latching and sacred edge consummation
//
// Terminology:
//   • approach_cycles      = total cycles from pre-spin start until interrupt fire
//   • shadow_to_isr_cycles = cycles from final shadow DWT sample to ISR entry
//
// Shared prespin architecture:
//
//   One shared shadow DWT serves all clocks.  A responsibility counter tracks
//   how many clocks are waiting for edges.
//
//   Prespin callbacks run in scheduled context (normal TimePop dispatch).
//   Each callback increments the responsibility counter and arms a named
//   ASAP timer ("PRESPIN_SPIN").  Named singleton replacement ensures only
//   one spin callback fires per dispatch pass, regardless of how many clocks
//   registered.  By the time the spin callback runs, all pending register
//   callbacks have already incremented the counter.
//
//   The spin loop runs in scheduled context from the ASAP callback.
//   Edge ISRs (priority 0) preempt it, decrement the counter, and
//   snapshot the shared shadow DWT into the per-clock runtime.
//
//   If the spin loop is preempted by non-edge activity (transport, other
//   timers), the shadow becomes momentarily stale.  This increases
//   shadow_to_isr_cycles for that event.  The correct TDC offset is the
//   MINIMUM shadow_to_isr_cycles across many events — outliers caused by
//   preemption are noise, not signal.
//
// ISR vs scheduled context:
//   • The ISR (priority 0) handles ONLY time-critical work:
//     - DWT capture and responsibility counter decrement
//     - Prespin snapshot into per-clock runtime
//     - Event construction (DWT correction, GNSS ns projection)
//     - GPT OCR3 advancement (must happen before next edge)
//     - Counter updates (irq_count, dispatch_count, event_count)
//   • Everything else runs in scheduled context via deferred dispatch:
//     - Subscriber event callbacks
//     - PPS drumbeat scheduling for next prespin
//     - OCXO prespin scheduling for next edge
//   • This separation is possible because geared scheduling computes
//     deadlines from the time anchor via pure arithmetic — the arm call
//     can happen at any point within the second without loss of precision.
//
// Scheduling intent:
//   • PPS pre-spin is a geared drumbeat from actual consummated PPS edges.
//   • ZERO is consummated only on a PPS edge and restarts that drumbeat.
//   • Before the first lawful PPS edge, there is no sacred drumbeat baseline yet.
//   • PPS critical scheduling must not interrogate ambient "now" time.
//   • OCXO1 / OCXO2 pre-spin are event-anchored and rescheduled from the just-
//     consummated OCXO edge, analogous to PPS drumbeat rescheduling.
//
// TimePop owns QTimer1. process_interrupt does not touch QTimer1.
//
// OCR1 is reserved for any existing PPS phase capture system if present.
// OCR3 is used for OCXO second-boundary scheduling to avoid conflict.
//
// GPT runs in free-running mode (FRR=1), so the counter never resets.
// OCR3 fires when counter == OCR3 value. The ISR advances OCR3 by
// OCXO_COUNTS_PER_SECOND for the next second boundary.
//
// Lifecycle:
//   1. process_interrupt_init_hardware() — GPT clock gates, pin mux, timer enable
//   2. process_interrupt_init()          — runtime slots, descriptor wiring
//   3. process_interrupt_enable_irqs()   — ISR vector installation + NVIC enable
//   4. interrupt_subscribe() / interrupt_start() — client wiring + activation
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

#include <math.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;
static constexpr uint32_t OCXO_COUNTS_PER_SECOND = 10000000;

static constexpr uint64_t NS_PER_SECOND_U64      = 1000000000ULL;
static constexpr uint64_t PRESPIN_LEAD_NS        = 10000ULL;     // 10 µs
static constexpr uint32_t PRESPIN_TIMEOUT_CYCLES = 100800;       // ~100 µs @ 1.008 GHz

// These values are intentionally separate. PPS is empirically ~48 cycles.
// GPT continues to use the profiled fixed-overhead constant.
static constexpr uint32_t PPS_ISR_FIXED_OVERHEAD_CYCLES = 48;

// Runtime quantization-model policy.
// These are discovery / estimator policy knobs, not discovered hardware constants.
static constexpr uint32_t DEQUANT_MAX_BUCKETS = 8;
static constexpr uint32_t DEQUANT_MIN_CONFIDENCE = 12;
static constexpr uint32_t DEQUANT_MIN_DISTINCT_BUCKETS = 3;

// Drop-dead-dumb mean window knob.
// Change this for testing.
// 8 or 16 are the intended test values.
static constexpr uint32_t DEQUANT_MEAN_WINDOW = 8;

// ============================================================================
// Shared prespin state — responsibility counter architecture
// ============================================================================

static volatile uint32_t g_prespin_shadow_dwt = 0;
static volatile uint32_t g_prespin_start_dwt = 0;
static volatile int32_t  g_prespin_responsibility_count = 0;
static volatile bool     g_prespin_spinning = false;

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

  uint64_t last_schedule_now_gnss_ns = 0;
  uint64_t last_schedule_delay_ns = 0;
  int64_t  last_prespin_margin_ns = 0;
  uint64_t last_schedule_event_target_gnss_ns = 0;
  uint64_t last_schedule_prespin_target_gnss_ns = 0;

  uint32_t snap_start_dwt = 0;
  uint32_t snap_shadow_dwt = 0;
  uint32_t snap_dwt_isr_entry_raw = 0;
  bool     snap_fired = false;
  bool     snap_timed_out = false;
};

struct interrupt_dequant_state_t {
  bool initialized = false;
  bool quantization_detected = false;
  bool dequant_enabled = false;

  uint32_t inferred_bucket_cycles = 0;
  uint32_t confidence = 0;

  uint32_t bucket_value[DEQUANT_MAX_BUCKETS] = {};
  uint32_t bucket_hits[DEQUANT_MAX_BUCKETS] = {};
  uint32_t bucket_count = 0;

  // Fixed-window dumb mean over recent OBSERVED deltas.
  uint32_t recent_delta[DEQUANT_MEAN_WINDOW] = {};
  uint32_t recent_count = 0;
  uint32_t recent_head = 0;

  uint32_t last_observed_dwt_at_event = 0;
  uint32_t last_best_dwt_at_event = 0;
  uint32_t last_observed_delta_cycles = 0;
  uint32_t last_best_delta_cycles = 0;
  uint32_t last_bucket_value = 0;

  uint32_t bucket_same_count = 0;
  uint32_t bucket_change_count = 0;

  double   estimated_delta_cycles = 0.0;
  double   estimated_dwt_at_event = 0.0;

  int32_t  last_innovation_cycles = 0;
  double   last_applied_correction_cycles = 0.0;
};

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub {};

  bool subscribed = false;
  bool active = false;

  uint32_t next_ocr3 = 0;

  interrupt_event_t last_event {};
  interrupt_capture_diag_t last_diag {};
  bool has_fired = false;

  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t event_count = 0;

  prespin_state_t prespin {};
  interrupt_dequant_state_t dequant {};

  uint32_t forensic_prev_dwt_isr_entry_raw = 0;
  uint32_t forensic_prev_dwt_at_event_observed = 0;
  uint32_t forensic_prev_dwt_at_event_best = 0;
  bool     forensic_prev_valid = false;
};

// ============================================================================
// PPS drumbeat state
// ============================================================================

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

// ============================================================================
// Provider diagnostics
// ============================================================================

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
static interrupt_provider_diag_t g_gpt1_diag = {};
static interrupt_provider_diag_t g_gpt2_diag = {};
static interrupt_anomaly_diag_t g_anomaly_diag = {};

static interrupt_subscriber_runtime_t* g_rt_pps   = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo1 = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2 = nullptr;

// ============================================================================
// Subscriber descriptors
// ============================================================================

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
  },
  {
    interrupt_subscriber_kind_t::OCXO1,
    "OCXO1",
    interrupt_provider_kind_t::GPT1,
    interrupt_lane_t::GPT1_COMPARE3,
    GPT_ISR_FIXED_OVERHEAD,
    NS_PER_SECOND_U64,
    true,
    false,
  },
  {
    interrupt_subscriber_kind_t::OCXO2,
    "OCXO2",
    interrupt_provider_kind_t::GPT2,
    interrupt_lane_t::GPT2_COMPARE3,
    GPT_ISR_FIXED_OVERHEAD,
    NS_PER_SECOND_U64,
    true,
    false,
  },
};

// ============================================================================
// Runtime lookup
// ============================================================================

static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    if (g_subscribers[i].desc && g_subscribers[i].desc->kind == kind) {
      return &g_subscribers[i];
    }
  }
  return nullptr;
}

// ============================================================================
// Anomaly helper
// ============================================================================

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
    case interrupt_provider_kind_t::GPT1:     return "GPT1";
    case interrupt_provider_kind_t::GPT2:     return "GPT2";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default:                                  return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::GPT1_COMPARE3: return "GPT1_COMPARE3";
    case interrupt_lane_t::GPT2_COMPARE3: return "GPT2_COMPARE3";
    case interrupt_lane_t::GPIO_EDGE:     return "GPIO_EDGE";
    default:                              return "NONE";
  }
}

// ============================================================================
// GPT hardware bring-up (provider custody)
// ============================================================================

static inline void enable_gpt1(void) {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

static inline void enable_gpt2(void) {
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
}

static void arm_gpt1_external(void) {
  enable_gpt1();
  *(portConfigRegister(OCXO1_10MHZ_PIN)) = 1;
  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = 0;
  GPT1_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT1_OCR3 = 0;
  GPT1_CR |= GPT_CR_EN;
}

static void arm_gpt2_external(void) {
  enable_gpt2();
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);
  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;
  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_PR = 0;
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT2_OCR3 = 0;
  GPT2_CR |= GPT_CR_EN;
}

uint32_t interrupt_gpt1_counter_now(void) {
  return GPT1_CNT;
}

uint32_t interrupt_gpt2_counter_now(void) {
  return GPT2_CNT;
}

// ============================================================================
// Quantization model helpers
// ============================================================================

static void dequant_reset(interrupt_dequant_state_t& dq) {
  dq = interrupt_dequant_state_t{};
}

static void dequant_insert_or_bump_bucket(interrupt_dequant_state_t& dq,
                                          uint32_t observed_delta_cycles) {
  for (uint32_t i = 0; i < dq.bucket_count; i++) {
    if (dq.bucket_value[i] == observed_delta_cycles) {
      dq.bucket_hits[i]++;
      return;
    }
  }

  if (dq.bucket_count < DEQUANT_MAX_BUCKETS) {
    dq.bucket_value[dq.bucket_count] = observed_delta_cycles;
    dq.bucket_hits[dq.bucket_count] = 1;
    dq.bucket_count++;
    return;
  }

  uint32_t min_i = 0;
  for (uint32_t i = 1; i < dq.bucket_count; i++) {
    if (dq.bucket_hits[i] < dq.bucket_hits[min_i]) {
      min_i = i;
    }
  }

  if (dq.bucket_hits[min_i] <= 1) {
    dq.bucket_value[min_i] = observed_delta_cycles;
    dq.bucket_hits[min_i] = 1;
  } else {
    dq.bucket_hits[min_i]--;
  }
}

static void dequant_sort_buckets(interrupt_dequant_state_t& dq) {
  for (uint32_t i = 0; i < dq.bucket_count; i++) {
    for (uint32_t j = i + 1; j < dq.bucket_count; j++) {
      if (dq.bucket_value[j] < dq.bucket_value[i]) {
        const uint32_t tv = dq.bucket_value[i];
        const uint32_t th = dq.bucket_hits[i];
        dq.bucket_value[i] = dq.bucket_value[j];
        dq.bucket_hits[i] = dq.bucket_hits[j];
        dq.bucket_value[j] = tv;
        dq.bucket_hits[j] = th;
      }
    }
  }
}

static void dequant_update_detected_lattice(interrupt_dequant_state_t& dq) {
  dq.quantization_detected = false;
  dq.dequant_enabled = false;
  dq.inferred_bucket_cycles = 0;

  if (dq.bucket_count < DEQUANT_MIN_DISTINCT_BUCKETS) {
    return;
  }

  dequant_sort_buckets(dq);

  uint32_t diff_value[DEQUANT_MAX_BUCKETS] = {};
  uint32_t diff_weight[DEQUANT_MAX_BUCKETS] = {};
  uint32_t diff_count = 0;

  for (uint32_t i = 1; i < dq.bucket_count; i++) {
    const uint32_t d = dq.bucket_value[i] - dq.bucket_value[i - 1];
    if (d == 0) continue;

    bool found = false;
    for (uint32_t j = 0; j < diff_count; j++) {
      if (diff_value[j] == d) {
        diff_weight[j] += dq.bucket_hits[i - 1] + dq.bucket_hits[i];
        found = true;
        break;
      }
    }

    if (!found && diff_count < DEQUANT_MAX_BUCKETS) {
      diff_value[diff_count] = d;
      diff_weight[diff_count] = dq.bucket_hits[i - 1] + dq.bucket_hits[i];
      diff_count++;
    }
  }

  if (diff_count == 0) {
    return;
  }

  uint32_t best_i = 0;
  for (uint32_t i = 1; i < diff_count; i++) {
    if (diff_weight[i] > diff_weight[best_i]) {
      best_i = i;
    }
  }

  dq.inferred_bucket_cycles = diff_value[best_i];
  dq.quantization_detected = (dq.inferred_bucket_cycles != 0);
  dq.dequant_enabled = dq.quantization_detected && (dq.confidence >= DEQUANT_MIN_CONFIDENCE);
}

static void dequant_push_recent_delta(interrupt_dequant_state_t& dq,
                                      uint32_t observed_delta_cycles) {
  dq.recent_delta[dq.recent_head] = observed_delta_cycles;
  dq.recent_head = (dq.recent_head + 1) % DEQUANT_MEAN_WINDOW;
  if (dq.recent_count < DEQUANT_MEAN_WINDOW) {
    dq.recent_count++;
  }
}

static double dequant_recent_mean(const interrupt_dequant_state_t& dq,
                                  uint32_t fallback_delta_cycles) {
  if (dq.recent_count == 0) {
    return (double)fallback_delta_cycles;
  }

  uint64_t sum = 0;
  for (uint32_t i = 0; i < dq.recent_count; i++) {
    sum += (uint64_t)dq.recent_delta[i];
  }

  return (double)sum / (double)dq.recent_count;
}

static uint32_t dequant_update_and_get_best_dwt(interrupt_subscriber_runtime_t& rt,
                                                uint32_t observed_dwt_at_event) {
  interrupt_dequant_state_t& dq = rt.dequant;

  if (!dq.initialized) {
    dq.initialized = true;
    dq.last_observed_dwt_at_event = observed_dwt_at_event;
    dq.last_best_dwt_at_event = observed_dwt_at_event;
    dq.estimated_dwt_at_event = (double)observed_dwt_at_event;
    dq.estimated_delta_cycles = 0.0;
    dq.last_observed_delta_cycles = 0;
    dq.last_best_delta_cycles = 0;
    dq.last_bucket_value = 0;
    dq.last_innovation_cycles = 0;
    dq.last_applied_correction_cycles = 0.0;
    return observed_dwt_at_event;
  }

  const uint32_t observed_delta_cycles = observed_dwt_at_event - dq.last_observed_dwt_at_event;
  dq.last_observed_delta_cycles = observed_delta_cycles;

  if (dq.last_bucket_value == observed_delta_cycles) {
    dq.bucket_same_count++;
  } else {
    dq.bucket_change_count++;
    dq.last_bucket_value = observed_delta_cycles;
  }

  dequant_insert_or_bump_bucket(dq, observed_delta_cycles);
  dq.confidence++;
  dequant_update_detected_lattice(dq);
  dequant_push_recent_delta(dq, observed_delta_cycles);

  if (!dq.dequant_enabled) {
    dq.estimated_delta_cycles = (double)observed_delta_cycles;
    dq.estimated_dwt_at_event = (double)observed_dwt_at_event;
    dq.last_best_delta_cycles = observed_delta_cycles;
    dq.last_best_dwt_at_event = observed_dwt_at_event;
    dq.last_observed_dwt_at_event = observed_dwt_at_event;
    dq.last_innovation_cycles = 0;
    dq.last_applied_correction_cycles = 0.0;
    return observed_dwt_at_event;
  }

  const double mean_delta_cycles = dequant_recent_mean(dq, observed_delta_cycles);
  const uint32_t best_dwt_u32 =
      dq.last_best_dwt_at_event + (uint32_t) llround(mean_delta_cycles);

  dq.estimated_delta_cycles = mean_delta_cycles;
  dq.estimated_dwt_at_event = (double)best_dwt_u32;
  dq.last_best_delta_cycles = best_dwt_u32 - dq.last_best_dwt_at_event;
  dq.last_best_dwt_at_event = best_dwt_u32;
  dq.last_observed_dwt_at_event = observed_dwt_at_event;
  dq.last_innovation_cycles =
      (int32_t)observed_delta_cycles - (int32_t) llround(mean_delta_cycles);
  dq.last_applied_correction_cycles =
      mean_delta_cycles - (double)observed_delta_cycles;

  return best_dwt_u32;
}

// ============================================================================
// ISR vectors — owned by process_interrupt
// ============================================================================

static void pps_gpio_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(dwt_raw);
}

static void gpt1_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;

  if (GPT1_SR & GPT_SR_OF3) {
    process_interrupt_gpt1_irq(dwt_raw);
  }
}

static void gpt2_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;

  if (GPT2_SR & GPT_SR_OF3) {
    process_interrupt_gpt2_irq(dwt_raw);
  }
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

// ============================================================================
// Forward declarations
// ============================================================================

static void pps_drumbeat_schedule_from_edge(interrupt_subscriber_runtime_t& rt,
                                            uint64_t pps_edge_gnss_ns);
static void pps_drumbeat_bootstrap(interrupt_subscriber_runtime_t& rt);
static void pps_drumbeat_consume_edge(interrupt_subscriber_runtime_t& rt);

// ============================================================================
// Prespin registration callback — scheduled context
// ============================================================================

static void prespin_register_callback(timepop_ctx_t*,
                                      timepop_diag_t*,
                                      void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) {
    record_anomaly(g_anomaly_diag.prespin_null_runtime,
                   interrupt_subscriber_kind_t::NONE);
    return;
  }

  if (!rt->active) {
    return;
  }

  prespin_state_t& ps = rt->prespin;
  ps.handle = TIMEPOP_INVALID_HANDLE;
  ps.active = true;
  ps.arm_count++;

  ps.snap_start_dwt = 0;
  ps.snap_shadow_dwt = 0;
  ps.snap_dwt_isr_entry_raw = 0;
  ps.snap_fired = false;
  ps.snap_timed_out = false;

  g_prespin_responsibility_count++;

  timepop_arm_alap(interrupt_prespin_service, nullptr, "PRESPIN_SPIN");
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (g_prespin_responsibility_count <= 0) return;
  if (g_prespin_spinning) return;

  g_prespin_spinning = true;

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

// ============================================================================
// Unified scheduling helper
// ============================================================================

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
      timepop_arm_from_anchor((int64_t)anchor_gnss_ns,
                              (int64_t)(offset_gnss_ns - PRESPIN_LEAD_NS),
                              false,
                              prespin_register_callback,
                              &rt,
                              prespin_timer_name(rt.desc->kind));

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

// ============================================================================
// PPS / OCXO scheduling progression
// ============================================================================

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

static void schedule_ocxo_from_edge(interrupt_subscriber_runtime_t& rt,
                                    uint64_t edge_gnss_ns) {
  if (edge_gnss_ns == 0) return;
  schedule_next_prespin_from_anchor(rt, edge_gnss_ns, NS_PER_SECOND_U64);
}

static void pps_drumbeat_bootstrap(interrupt_subscriber_runtime_t& rt) {
  (void)rt;

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

  const bool establishing_baseline =
      !g_pps_drumbeat.initialized || g_pps_drumbeat.zero_pending;

  g_pps_drumbeat.initialized = true;
  g_pps_drumbeat.zero_pending = false;
  g_pps_drumbeat.baseline_gnss_ns = pps_edge_gnss_ns;
  g_pps_drumbeat.next_index = 1;

  if (establishing_baseline) {
    g_pps_drumbeat.generation++;
  }

  pps_drumbeat_schedule_from_edge(rt, pps_edge_gnss_ns);
}

// ============================================================================
// Event reconstruction helpers
// ============================================================================

static void fill_diag_common(interrupt_subscriber_runtime_t& rt,
                             interrupt_capture_diag_t& diag,
                             const interrupt_event_t& event,
                             uint32_t dwt_isr_entry_raw,
                             uint32_t observed_dwt_at_event,
                             uint64_t observed_gnss_ns_at_event,
                             uint32_t best_dwt_at_event,
                             uint64_t best_gnss_ns_at_event,
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

  diag.dwt_at_event_observed = observed_dwt_at_event;
  diag.gnss_ns_at_event_observed = observed_gnss_ns_at_event;

  diag.dwt_at_event_adjusted = observed_dwt_at_event;  // backward-compat mirror
  diag.gnss_ns_at_event = observed_gnss_ns_at_event;   // backward-compat mirror

  diag.dwt_at_event_best = best_dwt_at_event;
  diag.gnss_ns_at_event_best = best_gnss_ns_at_event;

  diag.counter32_at_event = event.counter32_at_event;
  diag.dwt_event_correction_cycles = event.dwt_event_correction_cycles;

  diag.prespin_arm_count = rt.prespin.arm_count;
  diag.prespin_complete_count = rt.prespin.complete_count;
  diag.prespin_timeout_count = rt.prespin.timeout_count;
  diag.anomaly_count = rt.prespin.anomaly_count;

  diag.prev_valid = rt.forensic_prev_valid;

  if (rt.forensic_prev_valid) {
    diag.prev_dwt_isr_entry_raw = rt.forensic_prev_dwt_isr_entry_raw;
    diag.prev_dwt_at_event_observed = rt.forensic_prev_dwt_at_event_observed;
    diag.prev_dwt_at_event_adjusted = rt.forensic_prev_dwt_at_event_observed; // mirror
    diag.prev_dwt_at_event_best = rt.forensic_prev_dwt_at_event_best;

    diag.dwt_delta_raw = dwt_isr_entry_raw - rt.forensic_prev_dwt_isr_entry_raw;
    diag.dwt_delta_observed = observed_dwt_at_event - rt.forensic_prev_dwt_at_event_observed;
    diag.dwt_delta_adjusted = diag.dwt_delta_observed; // mirror
    diag.dwt_delta_best = best_dwt_at_event - rt.forensic_prev_dwt_at_event_best;
  } else {
    diag.prev_dwt_isr_entry_raw = 0;
    diag.prev_dwt_at_event_observed = 0;
    diag.prev_dwt_at_event_adjusted = 0;
    diag.prev_dwt_at_event_best = 0;
    diag.dwt_delta_raw = 0;
    diag.dwt_delta_observed = 0;
    diag.dwt_delta_adjusted = 0;
    diag.dwt_delta_best = 0;
  }

  if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1 ||
      rt.desc->kind == interrupt_subscriber_kind_t::OCXO2) {
    const interrupt_dequant_state_t& dq = rt.dequant;
    diag.quantization_detected = dq.quantization_detected;
    diag.dequant_enabled = dq.dequant_enabled;
    diag.dequant_initialized = dq.initialized;
    diag.dequant_bucket_cycles = dq.inferred_bucket_cycles;
    diag.dequant_confidence = dq.confidence;
    diag.dequant_estimated_delta_cycles = dq.estimated_delta_cycles;
    diag.dequant_innovation_cycles = dq.last_innovation_cycles;
    diag.dequant_applied_correction_cycles = dq.last_applied_correction_cycles;
    diag.dequant_bucket_same_count = dq.bucket_same_count;
    diag.dequant_bucket_change_count = dq.bucket_change_count;
    diag.dequant_bucket_count = dq.bucket_count;

    for (uint32_t i = 0; i < interrupt_capture_diag_t::MAX_DIAG_BUCKETS; i++) {
      diag.dequant_bucket_value[i] = (i < dq.bucket_count) ? dq.bucket_value[i] : 0;
      diag.dequant_bucket_hits[i] = (i < dq.bucket_count) ? dq.bucket_hits[i] : 0;
    }
  }

  rt.forensic_prev_dwt_isr_entry_raw = dwt_isr_entry_raw;
  rt.forensic_prev_dwt_at_event_observed = observed_dwt_at_event;
  rt.forensic_prev_dwt_at_event_best = best_dwt_at_event;
  rt.forensic_prev_valid = true;
}

// ============================================================================
// Deferred dispatch — scheduled context
// ============================================================================

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

// ============================================================================
// Unified ISR event handler — priority 0, time-critical work ONLY
// ============================================================================

static void handle_event(interrupt_subscriber_runtime_t& rt,
                         uint32_t dwt_isr_entry_raw,
                         uint32_t counter32_at_event) {
  prespin_state_t& ps = rt.prespin;

  if (ps.active) {
    g_prespin_responsibility_count--;
    ps.active = false;
    ps.complete_count++;
  }

  ps.snap_start_dwt = g_prespin_start_dwt;
  ps.snap_shadow_dwt = g_prespin_shadow_dwt;
  ps.snap_dwt_isr_entry_raw = dwt_isr_entry_raw;
  ps.snap_fired = true;
  ps.snap_timed_out = false;

  uint32_t approach_cycles = 0;
  uint32_t shadow_to_isr_cycles = 0;

  if (ps.snap_start_dwt != 0) {
    approach_cycles = dwt_isr_entry_raw - ps.snap_start_dwt;
  }

  if (ps.snap_shadow_dwt != 0) {
    shadow_to_isr_cycles = dwt_isr_entry_raw - ps.snap_shadow_dwt;
  } else {
    ps.anomaly_count++;
    record_anomaly(g_anomaly_diag.no_shadow_dwt,
                   rt.desc->kind,
                   dwt_isr_entry_raw,
                   ps.arm_count,
                   ps.complete_count);
  }

  if (!g_prespin_spinning && ps.snap_start_dwt == 0 && ps.snap_shadow_dwt == 0) {
    ps.anomaly_count++;
    record_anomaly(g_anomaly_diag.no_prespin_active,
                   rt.desc->kind,
                   dwt_isr_entry_raw,
                   ps.snap_shadow_dwt,
                   ps.arm_count);
  }

  const uint32_t correction = rt.desc->isr_overhead_cycles;
  const uint32_t observed_dwt_at_event = dwt_isr_entry_raw - correction;

  interrupt_event_t event {};
  event.kind = rt.desc->kind;
  event.provider = rt.desc->provider;
  event.lane = rt.desc->lane;
  event.counter32_at_event = counter32_at_event;
  event.dwt_event_correction_cycles = correction;

  uint32_t best_dwt_at_event = observed_dwt_at_event;
  uint64_t observed_gnss_ns_at_event = 0;
  uint64_t best_gnss_ns_at_event = 0;

  if (rt.desc->kind == interrupt_subscriber_kind_t::PPS) {
    const int64_t pps_index = (int64_t)rt.event_count;
    observed_gnss_ns_at_event = (uint64_t)(pps_index * 1000000000LL);
    best_gnss_ns_at_event = observed_gnss_ns_at_event;
    best_dwt_at_event = observed_dwt_at_event;
    event.status = interrupt_event_status_t::OK;
  } else {
    const int64_t observed_gnss_ns_signed = time_dwt_to_gnss_ns(observed_dwt_at_event);
    if (observed_gnss_ns_signed >= 0) {
      observed_gnss_ns_at_event = (uint64_t)observed_gnss_ns_signed;
    }

    best_dwt_at_event = dequant_update_and_get_best_dwt(rt, observed_dwt_at_event);

    const int64_t best_gnss_ns_signed = time_dwt_to_gnss_ns(best_dwt_at_event);
    if (best_gnss_ns_signed >= 0) {
      best_gnss_ns_at_event = (uint64_t)best_gnss_ns_signed;
      event.status = interrupt_event_status_t::OK;
    } else {
      best_gnss_ns_at_event = 0;
      event.status = interrupt_event_status_t::HOLD;
    }
  }

  event.dwt_at_event = best_dwt_at_event;
  event.gnss_ns_at_event = best_gnss_ns_at_event;

  interrupt_capture_diag_t diag {};
  fill_diag_common(rt,
                   diag,
                   event,
                   dwt_isr_entry_raw,
                   observed_dwt_at_event,
                   observed_gnss_ns_at_event,
                   best_dwt_at_event,
                   best_gnss_ns_at_event,
                   approach_cycles,
                   shadow_to_isr_cycles);

  rt.last_event = event;
  rt.last_diag = diag;
  rt.has_fired = true;
  rt.irq_count++;
  rt.dispatch_count++;
  rt.event_count++;

  timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
}

// ============================================================================
// GPIO / PPS ISR entry
// ============================================================================

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw) {
  g_gpio_diag.irq_count++;

  interrupt_subscriber_runtime_t* rt = g_rt_pps;
  if (!rt || !rt->active) {
    g_gpio_diag.miss_count++;
    return;
  }

  g_gpio_diag.dispatch_count++;

  const uint32_t next_pps_count = rt->event_count + 1;
  handle_event(*rt, dwt_isr_entry_raw, next_pps_count);
}

// ============================================================================
// GPT1 ISR entry
// ============================================================================

void process_interrupt_gpt1_irq(uint32_t dwt_isr_entry_raw) {
  g_gpt1_diag.irq_count++;

  if (!(GPT1_SR & GPT_SR_OF3)) {
    g_gpt1_diag.miss_count++;
    return;
  }

  GPT1_SR = GPT_SR_OF3;

  interrupt_subscriber_runtime_t* rt = g_rt_ocxo1;
  if (!rt || !rt->active) {
    return;
  }

  g_gpt1_diag.dispatch_count++;

  const uint32_t matched_count = rt->next_ocr3;

  rt->next_ocr3 += OCXO_COUNTS_PER_SECOND;
  GPT1_OCR3 = rt->next_ocr3;

  handle_event(*rt, dwt_isr_entry_raw, matched_count);
}

// ============================================================================
// GPT2 ISR entry
// ============================================================================

void process_interrupt_gpt2_irq(uint32_t dwt_isr_entry_raw) {
  g_gpt2_diag.irq_count++;

  if (!(GPT2_SR & GPT_SR_OF3)) {
    g_gpt2_diag.miss_count++;
    return;
  }

  GPT2_SR = GPT_SR_OF3;

  interrupt_subscriber_runtime_t* rt = g_rt_ocxo2;
  if (!rt || !rt->active) {
    return;
  }

  g_gpt2_diag.dispatch_count++;

  const uint32_t matched_count = rt->next_ocr3;

  rt->next_ocr3 += OCXO_COUNTS_PER_SECOND;
  GPT2_OCR3 = rt->next_ocr3;

  handle_event(*rt, dwt_isr_entry_raw, matched_count);
}

// ============================================================================
// Subscriber control
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

  rt->prespin.active = false;

  rt->forensic_prev_valid = false;
  rt->forensic_prev_dwt_isr_entry_raw = 0;
  rt->forensic_prev_dwt_at_event_observed = 0;
  rt->forensic_prev_dwt_at_event_best = 0;
  dequant_reset(rt->dequant);

  if (kind == interrupt_subscriber_kind_t::PPS) {
    debug_log("pps.start", "requested");
    pps_drumbeat_bootstrap(*rt);
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    uint32_t now = GPT1_CNT;
    rt->next_ocr3 = now + OCXO_COUNTS_PER_SECOND;
    GPT1_OCR3 = rt->next_ocr3;
    GPT1_SR = GPT_SR_OF3;
    GPT1_IR |= GPT_IR_OF3IE;
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    uint32_t now = GPT2_CNT;
    rt->next_ocr3 = now + OCXO_COUNTS_PER_SECOND;
    GPT2_OCR3 = rt->next_ocr3;
    GPT2_SR = GPT_SR_OF3;
    GPT2_IR |= GPT_IR_OF3IE;
    return true;
  }

  return false;
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

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    GPT1_IR &= ~GPT_IR_OF3IE;
  } else if (kind == interrupt_subscriber_kind_t::OCXO2) {
    GPT2_IR &= ~GPT_IR_OF3IE;
  }

  return true;
}

// ============================================================================
// PPS drumbeat control
// ============================================================================

void interrupt_request_pps_zero(void) {
  g_pps_drumbeat.zero_pending = true;
}

bool interrupt_pps_zero_pending(void) {
  return g_pps_drumbeat.zero_pending;
}

// ============================================================================
// Last event / diag access
// ============================================================================

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

// ============================================================================
// Initialization — Phase 1: hardware bring-up (early boot, no TimePop)
// ============================================================================

void process_interrupt_init_hardware(void) {
  if (g_interrupt_hw_ready) return;

  arm_gpt1_external();
  arm_gpt2_external();

  g_interrupt_hw_ready = true;
}

// ============================================================================
// Initialization — Phase 2: runtime slots (after process framework)
// ============================================================================

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;

  g_subscriber_count = 0;
  for (auto& rt : g_subscribers) {
    rt = interrupt_subscriber_runtime_t{};
  }

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

// ============================================================================
// Initialization — Phase 3: ISR vector installation + NVIC enable
// ============================================================================

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  GPT1_SR = 0x3F;
  GPT1_IR = 0;

  attachInterruptVector(IRQ_GPT1, gpt1_isr);
  NVIC_SET_PRIORITY(IRQ_GPT1, 0);
  NVIC_ENABLE_IRQ(IRQ_GPT1);

  GPT2_SR = 0x3F;
  GPT2_IR = 0;

  attachInterruptVector(IRQ_GPT2, gpt2_isr);
  NVIC_SET_PRIORITY(IRQ_GPT2, 0);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  g_interrupt_irqs_enabled = true;
}

// ============================================================================
// REPORT command
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready", g_interrupt_runtime_ready);
  p.add("irqs_enabled", g_interrupt_irqs_enabled);
  p.add("subscriber_count", g_subscriber_count);

  p.add("gpio_irq_count", g_gpio_diag.irq_count);
  p.add("gpio_dispatch_count", g_gpio_diag.dispatch_count);
  p.add("gpio_miss_count", g_gpio_diag.miss_count);

  p.add("gpt1_irq_count", g_gpt1_diag.irq_count);
  p.add("gpt1_dispatch_count", g_gpt1_diag.dispatch_count);
  p.add("gpt1_miss_count", g_gpt1_diag.miss_count);

  p.add("gpt2_irq_count", g_gpt2_diag.irq_count);
  p.add("gpt2_dispatch_count", g_gpt2_diag.dispatch_count);
  p.add("gpt2_miss_count", g_gpt2_diag.miss_count);

  p.add("pps_drumbeat_initialized", g_pps_drumbeat.initialized);
  p.add("pps_zero_pending", g_pps_drumbeat.zero_pending);
  p.add("pps_baseline_gnss_ns", g_pps_drumbeat.baseline_gnss_ns);
  p.add("pps_next_event_target_gnss_ns", g_pps_drumbeat.next_event_target_gnss_ns);
  p.add("pps_next_prespin_target_gnss_ns", g_pps_drumbeat.next_prespin_target_gnss_ns);
  p.add("pps_next_index", g_pps_drumbeat.next_index);
  p.add("pps_generation", g_pps_drumbeat.generation);

  p.add("prespin_responsibility_count", g_prespin_responsibility_count);
  p.add("prespin_spinning", g_prespin_spinning);

  p.add("anomaly_prespin_null_runtime", g_anomaly_diag.prespin_null_runtime);
  p.add("anomaly_prespin_arm_failed", g_anomaly_diag.prespin_arm_failed);
  p.add("anomaly_prespin_timeout", g_anomaly_diag.prespin_timeout);
  p.add("anomaly_no_prespin_active", g_anomaly_diag.no_prespin_active);
  p.add("anomaly_no_shadow_dwt", g_anomaly_diag.no_shadow_dwt);
  p.add("anomaly_null_desc", g_anomaly_diag.null_desc);

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

    s.add("next_ocr3", rt.next_ocr3);

    s.add("prespin_active", rt.prespin.active);
    s.add("prespin_fired", rt.prespin.snap_fired);
    s.add("prespin_timed_out", rt.prespin.snap_timed_out);
    s.add("prespin_start_dwt", rt.prespin.snap_start_dwt);
    s.add("shadow_dwt", rt.prespin.snap_shadow_dwt);
    s.add("dwt_isr_entry_raw", rt.prespin.snap_dwt_isr_entry_raw);

    s.add("prespin_target_gnss_ns", rt.prespin.prespin_target_gnss_ns);
    s.add("event_target_gnss_ns", rt.prespin.event_target_gnss_ns);
    s.add("prespin_arm_count", rt.prespin.arm_count);
    s.add("prespin_complete_count", rt.prespin.complete_count);
    s.add("prespin_timeout_count", rt.prespin.timeout_count);
    s.add("anomaly_count", rt.prespin.anomaly_count);

    s.add("last_schedule_now_gnss_ns", rt.prespin.last_schedule_now_gnss_ns);
    s.add("last_schedule_delay_ns", rt.prespin.last_schedule_delay_ns);
    s.add("last_prespin_margin_ns", rt.prespin.last_prespin_margin_ns);
    s.add("last_schedule_event_target_gnss_ns", rt.prespin.last_schedule_event_target_gnss_ns);
    s.add("last_schedule_prespin_target_gnss_ns", rt.prespin.last_schedule_prespin_target_gnss_ns);

    s.add("last_approach_cycles", rt.last_diag.approach_cycles);
    s.add("last_shadow_to_isr_cycles", rt.last_diag.shadow_to_isr_cycles);

    s.add("last_dwt_at_event_best", rt.last_event.dwt_at_event);
    s.add("last_gnss_ns_at_event_best", rt.last_event.gnss_ns_at_event);
    s.add("last_counter32_at_event", rt.last_event.counter32_at_event);
    s.add("last_correction_cycles", rt.last_event.dwt_event_correction_cycles);
    s.add("last_status", (uint32_t)rt.last_event.status);

    s.add("diag_dwt_at_event_observed", rt.last_diag.dwt_at_event_observed);
    s.add("diag_gnss_ns_at_event_observed", rt.last_diag.gnss_ns_at_event_observed);
    s.add("diag_dwt_at_event_best", rt.last_diag.dwt_at_event_best);
    s.add("diag_gnss_ns_at_event_best", rt.last_diag.gnss_ns_at_event_best);

    s.add("diag_dwt_delta_raw", rt.last_diag.dwt_delta_raw);
    s.add("diag_dwt_delta_observed", rt.last_diag.dwt_delta_observed);
    s.add("diag_dwt_delta_best", rt.last_diag.dwt_delta_best);

    s.add("diag_quantization_detected", rt.last_diag.quantization_detected);
    s.add("diag_dequant_enabled", rt.last_diag.dequant_enabled);
    s.add("diag_dequant_initialized", rt.last_diag.dequant_initialized);
    s.add("diag_dequant_bucket_cycles", rt.last_diag.dequant_bucket_cycles);
    s.add("diag_dequant_confidence", rt.last_diag.dequant_confidence);
    s.add("diag_dequant_innovation_cycles", rt.last_diag.dequant_innovation_cycles);
    s.add("diag_dequant_applied_correction_cycles", rt.last_diag.dequant_applied_correction_cycles);
    s.add("diag_dequant_estimated_delta_cycles", rt.last_diag.dequant_estimated_delta_cycles);

    subscribers.add(s);
  }

  p.add_array("subscribers", subscribers);

  const uint32_t cbcdr = CCM_CBCDR;
  const uint32_t ipg_div = ((cbcdr >> 8) & 0x3) + 1;
  const uint32_t ahb_div = ((cbcdr >> 10) & 0x7) + 1;
  const uint32_t periph_clk2_div = ((cbcdr >> 27) & 0x7) + 1;
  p.add("ccm_cbcdr_raw", cbcdr);
  p.add("ipg_podf", ipg_div);
  p.add("ahb_podf", ahb_div);
  p.add("periph_clk2_podf", periph_clk2_div);
  p.add("f_cpu", (uint32_t)F_CPU);
  p.add("ipg_freq_hz", (uint32_t)(F_CPU / ahb_div / ipg_div));
  p.add("dwt_cycles_per_ipg_tick", ahb_div * ipg_div);

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