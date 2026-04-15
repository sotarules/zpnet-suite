// ============================================================================
// process_interrupt.cpp — simplified interrupt normalization shell
// ============================================================================
//
// This file intentionally stops trying to solve interrupt latency.
// PPS/DWT is the yardstick; VCLOCK/TimePop owns Spin-Dry for scheduler truth.
// Here we simply capture ISR-entry DWT, normalize to GNSS nanoseconds, and
// forward lawful event facts to subscribers.
//
// OCXO QTimer3 channels remain 16-bit cadence lanes. One-second OCXO events are
// reconstructed in software by counting lawful compare hits.
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

// One second at 10 MHz = 10,000,000 counts = 152 * 65536 + 38528
static constexpr uint32_t OCXO_COUNTS_PER_SECOND  = 10000000U;
static constexpr uint32_t QTIMER16_MODULUS        = 65536U;
static constexpr uint32_t OCXO_SECOND_WHOLE_WRAPS = OCXO_COUNTS_PER_SECOND / QTIMER16_MODULUS; // 152
static constexpr uint32_t OCXO_SECOND_REMAINDER   = OCXO_COUNTS_PER_SECOND % QTIMER16_MODULUS; // 38528

static_assert(OCXO_SECOND_WHOLE_WRAPS == 152, "Unexpected OCXO wrap decomposition");
static_assert(OCXO_SECOND_REMAINDER   == 38528, "Unexpected OCXO remainder decomposition");

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
  uint16_t last_programmed_compare = 0;
  uint16_t last_initial_counter16 = 0;
  uint16_t last_second_target_low16 = 0;
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
// OCXO 16-bit cadence model
// ============================================================================

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
                             bool bridge_valid) {
  diag.provider = rt.desc->provider;
  diag.lane = rt.desc->lane;
  diag.kind = rt.desc->kind;

  diag.dwt_isr_entry_raw = dwt_isr_entry_raw;
  diag.dwt_at_event = event.dwt_at_event;
  diag.dwt_at_event_adjusted = event.dwt_at_event;
  diag.counter32_at_event = event.counter32_at_event;
  diag.dwt_event_correction_cycles = 0;

  diag.gnss_ns_at_event_raw = event.gnss_ns_at_event;
  diag.gnss_ns_at_event_final = event.gnss_ns_at_event;
  diag.gnss_ns_at_event = event.gnss_ns_at_event;
  diag.gnss_ns_at_event_delta = 0;

  diag.bridge_valid = bridge_valid;
  diag.bridge_within_tolerance = bridge_valid;
  diag.bridge_skipped_invalid = !bridge_valid;
  diag.bridge_used_prediction = false;
  diag.gnss_ns_at_event_bridge = event.gnss_ns_at_event;
  diag.bridge_gnss_ns_raw = event.gnss_ns_at_event;
  diag.bridge_gnss_ns_target = event.gnss_ns_at_event;
  diag.bridge_gnss_ns_final = event.gnss_ns_at_event;
  diag.bridge_raw_error_ns = 0;
  diag.bridge_final_error_ns = 0;
}

static void handle_event(interrupt_subscriber_runtime_t& rt,
                         uint32_t dwt_isr_entry_raw,
                         uint32_t counter32_at_event) {
  interrupt_event_t event {};
  event.kind = rt.desc->kind;
  event.provider = rt.desc->provider;
  event.lane = rt.desc->lane;
  event.status = interrupt_event_status_t::OK;
  event.dwt_at_event = dwt_isr_entry_raw;
  event.counter32_at_event = counter32_at_event;
  event.dwt_event_correction_cycles = 0;

  bool bridge_valid = true;

  if (rt.desc->kind == interrupt_subscriber_kind_t::PPS) {
    event.gnss_ns_at_event = (uint64_t)rt.event_count * NS_PER_SECOND_U64;
  } else {
    const int64_t gnss_ns_signed = time_dwt_to_gnss_ns(dwt_isr_entry_raw);
    if (gnss_ns_signed < 0) {
      event.gnss_ns_at_event = 0;
      event.status = interrupt_event_status_t::HOLD;
      bridge_valid = false;
    } else {
      event.gnss_ns_at_event = (uint64_t)gnss_ns_signed;
    }
  }

  interrupt_capture_diag_t diag {};
  fill_diag_common(rt, diag, event, dwt_isr_entry_raw, bridge_valid);

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
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 0);
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
  p.add("ocxo1_last_programmed_compare", (uint32_t)g_clock_ocxo1.last_programmed_compare);
  p.add("ocxo1_last_counter16_at_irq", (uint32_t)g_clock_ocxo1.last_counter16_at_irq);

  p.add("ocxo2_irq_count", g_clock_ocxo2.irq_count);
  p.add("ocxo2_dispatch_count", g_clock_ocxo2.dispatch_count);
  p.add("ocxo2_miss_count", g_clock_ocxo2.miss_count);
  p.add("ocxo2_bootstrap_count", g_clock_ocxo2.bootstrap_count);
  p.add("ocxo2_cadence_hits_total", g_clock_ocxo2.cadence_hits_total);
  p.add("ocxo2_last_programmed_compare", (uint32_t)g_clock_ocxo2.last_programmed_compare);
  p.add("ocxo2_last_counter16_at_irq", (uint32_t)g_clock_ocxo2.last_counter16_at_irq);

  PayloadArray subscribers;
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    const interrupt_subscriber_runtime_t& rt = g_subscribers[i];
    if (!rt.desc) continue;

    Payload s;
    s.add("slot", i);
    s.add("kind", interrupt_subscriber_kind_str(rt.desc->kind));
    s.add("active", rt.active);
    s.add("subscribed", rt.subscribed);
    s.add("irq_count", rt.irq_count);
    s.add("dispatch_count", rt.dispatch_count);
    s.add("event_count", rt.event_count);

    s.add("last_gnss_ns_at_event", rt.last_event.gnss_ns_at_event);
    s.add("last_dwt_at_event", rt.last_event.dwt_at_event);
    s.add("last_counter32_at_event", rt.last_event.counter32_at_event);
    s.add("last_status", (uint32_t)rt.last_event.status);

    s.add("bridge_valid", rt.last_diag.bridge_valid);
    s.add("bridge_gnss_ns_final", rt.last_diag.bridge_gnss_ns_final);
    s.add("bridge_skipped_invalid", rt.last_diag.bridge_skipped_invalid);

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
