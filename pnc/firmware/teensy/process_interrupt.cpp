// ============================================================================
// process_interrupt.cpp
// ============================================================================

#include "process_interrupt.h"

#include "time.h"
#include "process.h"
#include "payload.h"
#include "process_timepop.h"
#include "timepop.h"
#include "tdc_correction.h"

#include <Arduino.h>
#include "imxrt.h"

#include <string.h>

// ============================================================================
// Constants / limits
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;
static constexpr uint64_t PRESPIN_LEAD_TIMEPOP_NS  = 0ULL;
static constexpr uint64_t PRESPIN_LEAD_TIMETEST_NS = 5000ULL;

// ============================================================================
// Hard-fail integrity trap
// ============================================================================

[[noreturn]]
static void interrupt_integrity_trap(void) {
  __builtin_trap();
  while (true) { }
}

// ============================================================================
// Internal callback types for known subscriber descriptors
// ============================================================================

using interrupt_provider_init_fn =
    bool (*)(void);

using interrupt_provider_arm_for_target_fn =
    bool (*)(uint64_t target_gnss_ns,
             uint32_t* out_counter32_target,
             uint16_t* out_compare16);

using interrupt_provider_disarm_fn =
    void (*)(void);

using interrupt_raw_capture_fn =
    void (*)(uint32_t dwt_isr_entry_raw,
             interrupt_capture_raw_t& out_raw,
             interrupt_capture_diag_t* diag);

using interrupt_gnss_fn =
    uint64_t (*)(uint32_t dwt_at_event_adjusted);

// ============================================================================
// Descriptor + runtime state
// ============================================================================

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;

  bool exclusive_lane = true;
  uint64_t prespin_lead_ns = 0;

  // Formal event-DWT correction default for this subscriber/provider/lane.
  uint32_t default_dwt_event_correction_cycles = 0;

  interrupt_provider_init_fn provider_init_fn = nullptr;
  interrupt_provider_arm_for_target_fn provider_arm_for_target_fn = nullptr;
  interrupt_provider_disarm_fn provider_disarm_fn = nullptr;

  interrupt_raw_capture_fn raw_capture_fn = nullptr;
  interrupt_gnss_fn gnss_at_event_fn = nullptr;
};

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub {};

  bool subscribed = false;
  bool active = false;
  bool waiting_for_interrupt = false;
  bool pending_target_valid = false;

  uint64_t pending_target_gnss_ns = 0;

  // Sacred armed target state.
  uint32_t armed_counter32_target = 0;
  uint16_t armed_compare16 = 0;

  timepop_handle_t prespin_handle = TIMEPOP_INVALID_HANDLE;

  interrupt_capture_latency_t latency {};
  interrupt_event_t last_event {};
  interrupt_capture_diag_t last_diag {};

  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t schedule_count = 0;
  uint32_t arm_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t provider_init_failures = 0;
  uint32_t provider_arm_failures = 0;
  uint32_t prespin_arm_failures = 0;
};

struct interrupt_provider_diag_t {
  uint32_t irq_count = 0;
  uint32_t dispatch_misses = 0;
  uint32_t init_count = 0;
};

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS] = {};
static uint32_t g_subscriber_count = 0;

static bool g_interrupt_hw_ready = false;

static interrupt_provider_diag_t g_qtimer1_diag = {};
static interrupt_provider_diag_t g_gpt1_diag = {};
static interrupt_provider_diag_t g_gpt2_diag = {};
static interrupt_provider_diag_t g_gpio6789_diag = {};

// ============================================================================
// Forward declarations — known subscriber/provider hooks
// ============================================================================

static const interrupt_subscriber_descriptor_t* descriptor_for(interrupt_subscriber_kind_t kind);
static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind);

static void subscriber_schedule_prespin(interrupt_subscriber_runtime_t& rt);
static void subscriber_handle_interrupt(interrupt_subscriber_runtime_t& rt,
                                        uint32_t dwt_isr_entry_raw);

static void subscriber_timepop_callback(timepop_ctx_t* ctx, void* user_data);

static bool qtimer1_ch2_provider_init(void);
static bool qtimer1_ch3_provider_init(void);

static bool qtimer1_ch2_arm_for_target(uint64_t target_gnss_ns,
                                       uint32_t* out_counter32_target,
                                       uint16_t* out_compare16);
static bool qtimer1_ch3_arm_for_target(uint64_t target_gnss_ns,
                                       uint32_t* out_counter32_target,
                                       uint16_t* out_compare16);

static void qtimer1_ch2_disarm(void);
static void qtimer1_ch3_disarm(void);

static void timepop_raw_capture(uint32_t dwt_isr_entry_raw,
                                interrupt_capture_raw_t& out_raw,
                                interrupt_capture_diag_t* diag);
static void timetest_raw_capture(uint32_t dwt_isr_entry_raw,
                                 interrupt_capture_raw_t& out_raw,
                                 interrupt_capture_diag_t* diag);

// ============================================================================
// Local helpers
// ============================================================================

static inline uint32_t ic_read_dwt() {
  return DWT_CYCCNT;
}

static inline void ic_welford_add(interrupt_capture_latency_t& t, double x) {
  t.sample_count += 1;
  const double delta = x - t.mean_cycles;
  t.mean_cycles += delta / static_cast<double>(t.sample_count);
  const double delta2 = x - t.mean_cycles;
  t.m2_cycles += delta * delta2;
  t.live_profile_valid = true;
}

static inline uint32_t ic_welford_mean_u32(const interrupt_capture_latency_t& t) {
  return static_cast<uint32_t>(t.mean_cycles + 0.5);
}

static uint32_t choose_dwt_event_correction_cycles(interrupt_subscriber_runtime_t& rt,
                                                   interrupt_capture_diag_t& diag) {
  const uint32_t default_cycles = rt.latency.dwt_event_correction_cycles_default;
  const uint32_t live_cycles =
      rt.latency.live_profile_valid ? ic_welford_mean_u32(rt.latency) : default_cycles;

  diag.candidate_correction_default_cycles = default_cycles;
  diag.candidate_correction_live_cycles = live_cycles;

  if (rt.latency.live_profile_valid) {
    diag.used_live_profile = true;
    return live_cycles;
  }

  diag.used_default_profile = true;
  return default_cycles;
}

static void update_live_correction(interrupt_subscriber_runtime_t& rt,
                                   uint32_t correction_cycles) {
  rt.latency.dwt_event_correction_cycles = correction_cycles;

  if (rt.latency.sample_count == 0) {
    rt.latency.dwt_event_correction_cycles_min = correction_cycles;
    rt.latency.dwt_event_correction_cycles_max = correction_cycles;
  } else {
    if (correction_cycles < rt.latency.dwt_event_correction_cycles_min) {
      rt.latency.dwt_event_correction_cycles_min = correction_cycles;
    }
    if (correction_cycles > rt.latency.dwt_event_correction_cycles_max) {
      rt.latency.dwt_event_correction_cycles_max = correction_cycles;
    }
  }

  ic_welford_add(rt.latency, static_cast<double>(correction_cycles));
}

// ============================================================================
// Public helper strings
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::TIMEPOP:     return "TIMEPOP";
    case interrupt_subscriber_kind_t::TIME_TEST:   return "TIME_TEST";
    case interrupt_subscriber_kind_t::PPS:         return "PPS";
    case interrupt_subscriber_kind_t::VCLOCK:      return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1:       return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:       return "OCXO2";
    case interrupt_subscriber_kind_t::PHOTODIODE:  return "PHOTODIODE";
    default:                                       return "NONE";
  }
}

const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider) {
  switch (provider) {
    case interrupt_provider_kind_t::QTIMER1:  return "QTIMER1";
    case interrupt_provider_kind_t::GPT1:     return "GPT1";
    case interrupt_provider_kind_t::GPT2:     return "GPT2";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default:                                  return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::QTIMER1_CH2:   return "QTIMER1_CH2";
    case interrupt_lane_t::QTIMER1_CH3:   return "QTIMER1_CH3";
    case interrupt_lane_t::GPT1_COMPARE1: return "GPT1_COMPARE1";
    case interrupt_lane_t::GPT2_COMPARE1: return "GPT2_COMPARE1";
    case interrupt_lane_t::GPIO_EDGE:     return "GPIO_EDGE";
    default:                              return "NONE";
  }
}

// ============================================================================
// Formal DWT adjustment
// ============================================================================

uint32_t interrupt_adjust_dwt_at_event(interrupt_subscriber_kind_t,
                                       interrupt_provider_kind_t provider,
                                       interrupt_lane_t lane,
                                       uint32_t dwt_isr_entry_raw,
                                       interrupt_capture_diag_t* diag) {
  uint32_t correction_cycles = 0;

  // Formal, visible correction hook. For now this is constant-based.
  // QTimer compare currently uses the TDC correction constant.
  if (provider == interrupt_provider_kind_t::QTIMER1 &&
      (lane == interrupt_lane_t::QTIMER1_CH2 || lane == interrupt_lane_t::QTIMER1_CH3)) {
    correction_cycles = (uint32_t)TDC_ISR_LATENCY_CYCLES_QTIMER;
  } else {
    correction_cycles = 0;
  }

  if (diag) {
    diag->dwt_event_correction_cycles = correction_cycles;
    diag->dwt_at_event_adjusted = dwt_isr_entry_raw - correction_cycles;
  }

  return correction_cycles;
}

uint64_t interrupt_capture_default_gnss_projection(uint32_t dwt_at_event_adjusted) {
  const int64_t gnss = time_dwt_to_gnss_ns(dwt_at_event_adjusted);
  if (gnss < 0) interrupt_integrity_trap();
  return static_cast<uint64_t>(gnss);
}

// ============================================================================
// Known subscriber descriptors
// ============================================================================

static const interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  {
    interrupt_subscriber_kind_t::TIMEPOP,
    "TIMEPOP",
    interrupt_provider_kind_t::QTIMER1,
    interrupt_lane_t::QTIMER1_CH2,
    true,
    PRESPIN_LEAD_TIMEPOP_NS,
    (uint32_t)TDC_ISR_LATENCY_CYCLES_QTIMER,
    qtimer1_ch2_provider_init,
    qtimer1_ch2_arm_for_target,
    qtimer1_ch2_disarm,
    timepop_raw_capture,
    interrupt_capture_default_gnss_projection
  },
  {
    interrupt_subscriber_kind_t::TIME_TEST,
    "TIME_TEST",
    interrupt_provider_kind_t::QTIMER1,
    interrupt_lane_t::QTIMER1_CH3,
    true,
    PRESPIN_LEAD_TIMETEST_NS,
    (uint32_t)TDC_ISR_LATENCY_CYCLES_QTIMER,
    qtimer1_ch3_provider_init,
    qtimer1_ch3_arm_for_target,
    qtimer1_ch3_disarm,
    timetest_raw_capture,
    interrupt_capture_default_gnss_projection
  },
};

// ============================================================================
// Descriptor / runtime lookup
// ============================================================================

static const interrupt_subscriber_descriptor_t* descriptor_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < (sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0])); i++) {
    if (DESCRIPTORS[i].kind == kind) return &DESCRIPTORS[i];
  }
  return nullptr;
}

static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    if (g_subscribers[i].subscribed &&
        g_subscribers[i].sub.kind == kind) {
      return &g_subscribers[i];
    }
  }
  return nullptr;
}

// ============================================================================
// Provider init / arm / disarm — QTIMER1
// ============================================================================

static bool qtimer1_ch2_provider_init(void) {
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  g_qtimer1_diag.init_count++;
  return true;
}

static bool qtimer1_ch3_provider_init(void) {
  IMXRT_TMR1.CH[3].CTRL   = 0;
  IMXRT_TMR1.CH[3].CNTR   = IMXRT_TMR1.CH[0].CNTR;
  IMXRT_TMR1.CH[3].LOAD   = 0;
  IMXRT_TMR1.CH[3].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[3].SCTRL  = 0;
  IMXRT_TMR1.CH[3].CSCTRL = 0;
  IMXRT_TMR1.CH[3].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  g_qtimer1_diag.init_count++;
  return true;
}

static inline void qtimer1_ch2_clear_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void qtimer1_ch2_arm_compare(uint16_t target_low16) {
  qtimer1_ch2_clear_flag();
  IMXRT_TMR1.CH[2].COMP1  = target_low16;
  IMXRT_TMR1.CH[2].CMPLD1 = target_low16;
  qtimer1_ch2_clear_flag();
  IMXRT_TMR1.CH[2].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static bool qtimer1_ch2_arm_for_target(uint64_t,
                                       uint32_t* out_counter32_target,
                                       uint16_t* out_compare16) {
  // In this current minimal recap, TIMEPOP will pass the exact 32-bit target in
  // through interrupt_schedule_target() later; this arm function is just the
  // low-level compare programming path and assumes the runtime has already set
  // the sacred full target count.
  interrupt_subscriber_runtime_t* rt = runtime_for(interrupt_subscriber_kind_t::TIMEPOP);
  if (!rt) interrupt_integrity_trap();

  const uint32_t target32 = rt->armed_counter32_target;
  const uint16_t target16 = (uint16_t)(target32 & 0xFFFFu);

  qtimer1_ch2_arm_compare(target16);

  if (out_counter32_target) *out_counter32_target = target32;
  if (out_compare16) *out_compare16 = target16;
  return true;
}

static bool qtimer1_ch3_arm_for_target(uint64_t,
                                       uint32_t* out_counter32_target,
                                       uint16_t* out_compare16) {
  interrupt_subscriber_runtime_t* rt = runtime_for(interrupt_subscriber_kind_t::TIME_TEST);
  if (!rt) interrupt_integrity_trap();

  const uint32_t target32 = rt->armed_counter32_target;
  const uint16_t target16 = (uint16_t)(target32 & 0xFFFFu);

  IMXRT_TMR1.CH[3].CSCTRL  = 0;
  IMXRT_TMR1.CH[3].COMP1   = target16;
  IMXRT_TMR1.CH[3].CMPLD1  = target16;
  IMXRT_TMR1.CH[3].CSCTRL  = 0;
  IMXRT_TMR1.CH[3].CSCTRL  = TMR_CSCTRL_TCF1EN;

  if (out_counter32_target) *out_counter32_target = target32;
  if (out_compare16) *out_compare16 = target16;
  return true;
}

static void qtimer1_ch2_disarm(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
}

static void qtimer1_ch3_disarm(void) {
  IMXRT_TMR1.CH[3].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
}

// ============================================================================
// Raw capture hooks
// ============================================================================

static void timepop_raw_capture(uint32_t dwt_isr_entry_raw,
                                interrupt_capture_raw_t& out_raw,
                                interrupt_capture_diag_t* diag) {
  out_raw.dwt_isr_entry_raw = dwt_isr_entry_raw;
  out_raw.compare16 = (uint16_t)IMXRT_TMR1.CH[2].COMP1;

  // Verification facts only — not used to invent event time.
  out_raw.verify_low16 = (uint16_t)IMXRT_TMR1.CH[0].CNTR;
  out_raw.verify_high16 = (uint16_t)IMXRT_TMR1.CH[1].HOLD;

  out_raw.irq_status = IMXRT_TMR1.CH[2].CSCTRL;
  qtimer1_ch2_clear_flag();
  out_raw.irq_flags_cleared = 1u;

  out_raw.dwt_after_capture = ic_read_dwt();

  if (diag) {
    diag->raw_compare16 = out_raw.compare16;
    diag->raw_verify_low16 = out_raw.verify_low16;
    diag->raw_verify_high16 = out_raw.verify_high16;
  }
}

static void timetest_raw_capture(uint32_t dwt_isr_entry_raw,
                                 interrupt_capture_raw_t& out_raw,
                                 interrupt_capture_diag_t* diag) {
  out_raw.dwt_isr_entry_raw = dwt_isr_entry_raw;
  out_raw.compare16 = (uint16_t)IMXRT_TMR1.CH[3].COMP1;

  // Verification facts only — not used to invent event time.
  out_raw.verify_low16 = (uint16_t)IMXRT_TMR1.CH[0].CNTR;
  out_raw.verify_high16 = (uint16_t)IMXRT_TMR1.CH[1].HOLD;

  out_raw.irq_status = IMXRT_TMR1.CH[3].CSCTRL;
  IMXRT_TMR1.CH[3].CSCTRL = 0;
  out_raw.irq_flags_cleared = 1u;

  out_raw.dwt_after_capture = ic_read_dwt();

  if (diag) {
    diag->raw_compare16 = out_raw.compare16;
    diag->raw_verify_low16 = out_raw.verify_low16;
    diag->raw_verify_high16 = out_raw.verify_high16;

    extern volatile uint32_t time_test_shadow_dwt;
    diag->shadow_dwt = time_test_shadow_dwt;
    diag->shadow_valid = true;
  }
}

// ============================================================================
// Subscriber scheduling
// ============================================================================

static void subscriber_timepop_callback(timepop_ctx_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->active || !rt->pending_target_valid || !rt->desc) return;

  rt->prespin_handle = TIMEPOP_INVALID_HANDLE;

  uint32_t armed_counter32_target = 0;
  uint16_t armed_compare16 = 0;
  if (!rt->desc->provider_arm_for_target_fn ||
      !rt->desc->provider_arm_for_target_fn(rt->pending_target_gnss_ns,
                                            &armed_counter32_target,
                                            &armed_compare16)) {
    rt->provider_arm_failures++;
    interrupt_integrity_trap();
  }

  rt->arm_count++;
  rt->armed_counter32_target = armed_counter32_target;
  rt->armed_compare16 = armed_compare16;
  rt->waiting_for_interrupt = true;
  rt->latency.prev_spin_dwt = rt->latency.last_spin_dwt;
  rt->latency.last_spin_dwt = ic_read_dwt();
  rt->latency.deterministic_spin_active = true;
}

static void subscriber_schedule_prespin(interrupt_subscriber_runtime_t& rt) {
  if (!rt.active || !rt.pending_target_valid || !rt.desc) return;

  if (rt.prespin_handle != TIMEPOP_INVALID_HANDLE) {
    timepop_cancel(rt.prespin_handle);
    rt.prespin_handle = TIMEPOP_INVALID_HANDLE;
  }

  uint64_t now_ns = (uint64_t)time_gnss_ns_now();
  if ((int64_t)now_ns < 0) {
    rt.prespin_arm_failures++;
    interrupt_integrity_trap();
  }

  uint64_t wake_ns = rt.pending_target_gnss_ns;
  if (rt.desc->prespin_lead_ns < wake_ns) {
    wake_ns -= rt.desc->prespin_lead_ns;
  } else {
    wake_ns = now_ns;
  }

  uint64_t delay_ns = (wake_ns > now_ns) ? (wake_ns - now_ns) : 0;

  rt.prespin_handle = timepop_arm(delay_ns,
                                  false,
                                  subscriber_timepop_callback,
                                  &rt,
                                  rt.desc->name ? rt.desc->name : "interrupt-prespin");
  if (rt.prespin_handle == TIMEPOP_INVALID_HANDLE) {
    rt.prespin_arm_failures++;
    interrupt_integrity_trap();
  }
}

// ============================================================================
// Subscriber interrupt handling
// ============================================================================

static void subscriber_handle_interrupt(interrupt_subscriber_runtime_t& rt,
                                        uint32_t dwt_isr_entry_raw) {
  if (!rt.desc || !rt.sub.on_event) {
    interrupt_integrity_trap();
  }

  interrupt_capture_diag_t diag {};
  diag.enabled = true;
  diag.provider = rt.desc->provider;
  diag.lane = rt.desc->lane;
  diag.kind = rt.sub.kind;
  diag.spin_last_dwt = rt.latency.last_spin_dwt;
  diag.spin_prev_dwt = rt.latency.prev_spin_dwt;
  diag.dwt_isr_entry_raw = dwt_isr_entry_raw;

  interrupt_capture_raw_t raw {};
  if (!rt.desc->raw_capture_fn) {
    interrupt_integrity_trap();
  }

  rt.desc->raw_capture_fn(dwt_isr_entry_raw, raw, &diag);
  diag.dwt_after_capture = raw.dwt_after_capture;

  if (diag.shadow_valid) {
    diag.shadow_to_isr_entry_cycles = dwt_isr_entry_raw - diag.shadow_dwt;
  } else {
    diag.shadow_to_isr_entry_cycles = 0;
  }

  // QTIMER compare event truth comes from the sacred full armed target count,
  // not from any current register read.
  const uint32_t counter32_at_event = rt.armed_counter32_target;
  diag.counter32_at_event = counter32_at_event;

  const uint16_t expected_low16 = (uint16_t)(counter32_at_event & 0xFFFFu);
  const uint16_t expected_high16 = (uint16_t)((counter32_at_event >> 16) & 0xFFFFu);

  diag.expected_low16 = expected_low16;
  diag.expected_high16 = expected_high16;

  // Low-word compare must match exactly.
  if (raw.compare16 != expected_low16) {
    interrupt_integrity_trap();
  }

  // High-word neighborhood verification. Because compare only fires on low 16,
  // allow either the expected high word or the immediately previous high word
  // depending on ISR entry latency relative to rollover.
  diag.verify_high16_matches = (raw.verify_high16 == expected_high16);
  diag.verify_high16_is_previous = (raw.verify_high16 == (uint16_t)(expected_high16 - 1u));

  if (!diag.verify_high16_matches && !diag.verify_high16_is_previous) {
    interrupt_integrity_trap();
  }

  const uint32_t correction_cycles =
      interrupt_adjust_dwt_at_event(rt.sub.kind,
                                    rt.desc->provider,
                                    rt.desc->lane,
                                    dwt_isr_entry_raw,
                                    &diag);

  update_live_correction(rt, correction_cycles);

  const uint32_t dwt_at_event_adjusted = diag.dwt_at_event_adjusted;
  const uint64_t gnss_ns_at_event =
      rt.desc->gnss_at_event_fn
        ? rt.desc->gnss_at_event_fn(dwt_at_event_adjusted)
        : interrupt_capture_default_gnss_projection(dwt_at_event_adjusted);

  diag.gnss_ns_at_event = gnss_ns_at_event;

  interrupt_event_t event {};
  event.kind = rt.sub.kind;
  event.provider = rt.desc->provider;
  event.lane = rt.desc->lane;
  event.status = interrupt_event_status_t::OK;
  event.dwt_at_event = dwt_at_event_adjusted;
  event.gnss_ns_at_event = gnss_ns_at_event;
  event.counter32_at_event = counter32_at_event;
  event.dwt_event_correction_cycles = correction_cycles;
  event.raw = raw;
  event.latency = rt.latency;
  event.latency.dwt_event_correction_cycles = correction_cycles;

  rt.last_event = event;
  rt.last_diag = diag;
  rt.irq_count++;
  rt.dispatch_count++;

  interrupt_next_target_t next =
      rt.sub.on_event(event, &rt.last_diag, rt.sub.user_data);

  rt.waiting_for_interrupt = false;
  rt.pending_target_valid = false;
  rt.latency.deterministic_spin_active = false;

  if (next.schedule_next) {
    rt.pending_target_valid = true;
    rt.pending_target_gnss_ns = next.target_gnss_ns;
    rt.schedule_count++;
    subscriber_schedule_prespin(rt);
  }
}

// ============================================================================
// Public control
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub) {
  if (!g_interrupt_hw_ready) return false;
  if (sub.kind == interrupt_subscriber_kind_t::NONE) return false;
  if (!sub.on_event) return false;

  const interrupt_subscriber_descriptor_t* desc = descriptor_for(sub.kind);
  if (!desc) return false;

  interrupt_subscriber_runtime_t* existing = runtime_for(sub.kind);
  if (existing) {
    existing->sub = sub;
    existing->desc = desc;
    existing->subscribed = true;
    return true;
  }

  if (g_subscriber_count >= MAX_INTERRUPT_SUBSCRIBERS) return false;

  interrupt_subscriber_runtime_t& rt = g_subscribers[g_subscriber_count++];
  rt = {};
  rt.desc = desc;
  rt.sub = sub;
  rt.subscribed = true;
  rt.latency.dwt_event_correction_cycles_default = desc->default_dwt_event_correction_cycles;
  rt.latency.dwt_event_correction_cycles_min = desc->default_dwt_event_correction_cycles;
  rt.latency.dwt_event_correction_cycles_max = desc->default_dwt_event_correction_cycles;

  if (desc->provider_init_fn && !desc->provider_init_fn()) {
    rt.provider_init_failures++;
    interrupt_integrity_trap();
  }

  return true;
}

bool interrupt_start(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;

  rt->active = true;
  rt->start_count++;
  return true;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;

  rt->active = false;
  rt->waiting_for_interrupt = false;
  rt->pending_target_valid = false;
  rt->stop_count++;

  if (rt->prespin_handle != TIMEPOP_INVALID_HANDLE) {
    timepop_cancel(rt->prespin_handle);
    rt->prespin_handle = TIMEPOP_INVALID_HANDLE;
  }

  if (rt->desc->provider_disarm_fn) {
    rt->desc->provider_disarm_fn();
  }

  return true;
}

bool interrupt_schedule_target(interrupt_subscriber_kind_t kind,
                               uint64_t target_gnss_ns) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc || !rt->active) return false;

  rt->pending_target_valid = true;
  rt->pending_target_gnss_ns = target_gnss_ns;
  rt->schedule_count++;

  // For the current minimal QTIMER compare design, target counter32 truth must
  // be set by the known subscriber before prespin/arming occurs. The subscriber
  // uses this API only to request the cycle begin. If the target is not set,
  // that is an integrity failure.
  if (rt->armed_counter32_target == 0) {
    // For now, allow first-cycle callers to derive the target by using the low
    // 16-bit compare lane at arm time only if they already populated the target
    // elsewhere. This will be completed in TimePop / clocks integration.
  }

  subscriber_schedule_prespin(*rt);
  return true;
}

// ============================================================================
// Shared IRQ authority
// ============================================================================

void process_interrupt_qtimer1_irq(void) {
  g_qtimer1_diag.irq_count++;

  // DWT must be the first thing we capture in the actual shared ISR path.
  const uint32_t dwt_isr_entry_raw = ic_read_dwt();

  if (IMXRT_TMR1.CH[3].CSCTRL & TMR_CSCTRL_TCF1) {
    interrupt_subscriber_runtime_t* rt = runtime_for(interrupt_subscriber_kind_t::TIME_TEST);
    if (rt && rt->active && rt->waiting_for_interrupt) {
      subscriber_handle_interrupt(*rt, dwt_isr_entry_raw);
      return;
    }
    interrupt_integrity_trap();
  }

  if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    interrupt_subscriber_runtime_t* rt = runtime_for(interrupt_subscriber_kind_t::TIMEPOP);
    if (rt && rt->active && rt->waiting_for_interrupt) {
      subscriber_handle_interrupt(*rt, dwt_isr_entry_raw);
      return;
    }
    interrupt_integrity_trap();
  }

  g_qtimer1_diag.dispatch_misses++;
}

void process_interrupt_gpt1_irq(void) {
  g_gpt1_diag.irq_count++;
  g_gpt1_diag.dispatch_misses++;
}

void process_interrupt_gpt2_irq(void) {
  g_gpt2_diag.irq_count++;
  g_gpt2_diag.dispatch_misses++;
}

void process_interrupt_gpio6789_irq(void) {
  g_gpio6789_diag.irq_count++;
  g_gpio6789_diag.dispatch_misses++;
}

// ============================================================================
// Init / process registration
// ============================================================================

void process_interrupt_init(void) {
  if (g_interrupt_hw_ready) return;

  attachInterruptVector(IRQ_QTIMER1, []() {
    process_interrupt_qtimer1_irq();
  });
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  g_interrupt_hw_ready = true;
}

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("subscriber_count", g_subscriber_count);

  p.add("qtimer1_irq_count", g_qtimer1_diag.irq_count);
  p.add("qtimer1_dispatch_misses", g_qtimer1_diag.dispatch_misses);
  p.add("qtimer1_init_count", g_qtimer1_diag.init_count);

  p.add("gpt1_irq_count", g_gpt1_diag.irq_count);
  p.add("gpt1_dispatch_misses", g_gpt1_diag.dispatch_misses);
  p.add("gpt1_init_count", g_gpt1_diag.init_count);

  p.add("gpt2_irq_count", g_gpt2_diag.irq_count);
  p.add("gpt2_dispatch_misses", g_gpt2_diag.dispatch_misses);
  p.add("gpt2_init_count", g_gpt2_diag.init_count);

  p.add("gpio6789_irq_count", g_gpio6789_diag.irq_count);
  p.add("gpio6789_dispatch_misses", g_gpio6789_diag.dispatch_misses);
  p.add("gpio6789_init_count", g_gpio6789_diag.init_count);

  PayloadArray subscribers;
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    const interrupt_subscriber_runtime_t& rt = g_subscribers[i];
    if (!rt.subscribed || !rt.desc) continue;

    Payload s;
    s.add("slot", i);
    s.add("kind", interrupt_subscriber_kind_str(rt.sub.kind));
    s.add("provider", interrupt_provider_kind_str(rt.desc->provider));
    s.add("lane", interrupt_lane_str(rt.desc->lane));
    s.add("active", rt.active);
    s.add("waiting_for_interrupt", rt.waiting_for_interrupt);
    s.add("pending_target_valid", rt.pending_target_valid);
    s.add("pending_target_gnss_ns", rt.pending_target_gnss_ns);
    s.add("armed_counter32_target", rt.armed_counter32_target);
    s.add("armed_compare16", (uint32_t)rt.armed_compare16);

    s.add("correction_default_cycles", rt.latency.dwt_event_correction_cycles_default);
    s.add("correction_mean_cycles", rt.latency.mean_cycles);
    s.add("correction_samples", (int32_t)rt.latency.sample_count);

    s.add("start_count", rt.start_count);
    s.add("stop_count", rt.stop_count);
    s.add("schedule_count", rt.schedule_count);
    s.add("arm_count", rt.arm_count);
    s.add("irq_count", rt.irq_count);
    s.add("dispatch_count", rt.dispatch_count);
    s.add("provider_init_failures", rt.provider_init_failures);
    s.add("provider_arm_failures", rt.provider_arm_failures);
    s.add("prespin_arm_failures", rt.prespin_arm_failures);

    s.add("last_dwt_at_event", rt.last_event.dwt_at_event);
    s.add("last_gnss_ns_at_event", rt.last_event.gnss_ns_at_event);
    s.add("last_counter32_at_event", rt.last_event.counter32_at_event);
    s.add("last_dwt_event_correction_cycles", rt.last_event.dwt_event_correction_cycles);

    s.add("diag_dwt_isr_entry_raw", rt.last_diag.dwt_isr_entry_raw);
    s.add("diag_shadow_valid", rt.last_diag.shadow_valid);
    s.add("diag_shadow_dwt", rt.last_diag.shadow_dwt);
    s.add("diag_shadow_to_isr_entry_cycles", rt.last_diag.shadow_to_isr_entry_cycles);
    s.add("diag_dwt_event_correction_cycles", rt.last_diag.dwt_event_correction_cycles);
    s.add("diag_dwt_at_event_adjusted", rt.last_diag.dwt_at_event_adjusted);
    s.add("diag_expected_low16", (uint32_t)rt.last_diag.expected_low16);
    s.add("diag_expected_high16", (uint32_t)rt.last_diag.expected_high16);
    s.add("diag_verify_high16_matches", rt.last_diag.verify_high16_matches);
    s.add("diag_verify_high16_is_previous", rt.last_diag.verify_high16_is_previous);

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