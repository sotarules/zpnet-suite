// ============================================================================
// process_interrupt.cpp — provider authority + shared pre-spin (QTimer v19)
// ============================================================================
//
// process_interrupt is the sole authority for all precision interrupt hardware.
//
// Owned hardware:
//   • QTimer3 CH2 (OCXO1 10 MHz) — pin 14, timer bring-up, ISR, compare
//   • QTimer3 CH3 (OCXO2 10 MHz) — pin 15, timer bring-up, ISR, compare
//   • GPIO6789 (PPS rising edge)  — pin 1, attachInterrupt, ISR vector
//   • All ISR vectors capture ARM_DWT_CYCCNT as their first instruction
//
// Owned machinery:
//   • Shared pre-spin shadow-write loop with responsibility counter
//   • Event reconstruction (DWT correction, GNSS ns projection)
//   • Subscriber dispatch with canonical event + diagnostics
//   • PPS geared drumbeat scheduling from sacred PPS baseline
//   • PPS ZERO-request latching and sacred edge consummation
//
// QTimer register setup mirrors TimePop's proven QTimer1 configuration:
//
//   • No TMR_CTRL_LENGTH — counter free-runs through full 16-bit range
//   • COMP1 = 0xFFFF, CMPLD1 = 0xFFFF — wraps naturally at 65535→0
//   • CM(1) = count rising edges of primary source
//   • PCS(n) = select the channel's own external pin input
//   • Compare interrupt via SCTRL TCF / TCFIE (match on COMP1 value)
//   • Software tracks full 16-bit COMP1 target for one-second boundaries
//
// TimePop owns QTimer1. process_interrupt does not touch QTimer1, except
// for a torn-read-safe snapshot used to capture the PPS-aligned VCLOCK fact
// at the PPS boundary itself.
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
static constexpr uint32_t OCXO_COUNTS_PER_SECOND = 10000000;

static constexpr uint64_t NS_PER_SECOND_U64      = 1000000000ULL;
static constexpr uint64_t PRESPIN_LEAD_NS        = 10000ULL;
static constexpr uint32_t PRESPIN_TIMEOUT_CYCLES = 100800;

static constexpr uint32_t PPS_ISR_FIXED_OVERHEAD_CYCLES = 48;

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

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub {};

  bool subscribed = false;
  bool active = false;

  uint16_t next_comp = 0;
  uint32_t next_match_total = 0;

  uint16_t last_cntr = 0;
  uint32_t running_total = 0;
  uint32_t spurious_count = 0;
  bool first_after_accept = false;

  interrupt_event_t last_event {};
  interrupt_capture_diag_t last_diag {};
  bool has_fired = false;

  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t event_count = 0;

  prespin_state_t prespin {};
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
static interrupt_provider_diag_t g_qtimer3_ch2_diag = {};
static interrupt_provider_diag_t g_qtimer3_ch3_diag = {};
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
    interrupt_provider_kind_t::QTIMER3,
    interrupt_lane_t::QTIMER3_CH2_COMP,
    QTIMER_ISR_FIXED_OVERHEAD,
    NS_PER_SECOND_U64,
    true,
    false,
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
  case interrupt_provider_kind_t::QTIMER3:   return "QTIMER3";
  case interrupt_provider_kind_t::GPIO6789:  return "GPIO6789";
  default:                                   return "NONE";
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
// QTimer3 hardware bring-up (provider custody)
// ============================================================================

static void arm_qtimer3_ocxo_channels(void) {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);

  *(portConfigRegister(OCXO1_PIN)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  *(portConfigRegister(OCXO2_PIN)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

  IMXRT_TMR3.CH[2].CTRL   = 0;
  IMXRT_TMR3.CH[2].SCTRL  = 0;
  IMXRT_TMR3.CH[2].CSCTRL = 0;
  IMXRT_TMR3.CH[2].LOAD   = 0;
  IMXRT_TMR3.CH[2].CNTR   = 0;
  IMXRT_TMR3.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR3.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(2);

  IMXRT_TMR3.CH[3].CTRL   = 0;
  IMXRT_TMR3.CH[3].SCTRL  = 0;
  IMXRT_TMR3.CH[3].CSCTRL = 0;
  IMXRT_TMR3.CH[3].LOAD   = 0;
  IMXRT_TMR3.CH[3].CNTR   = 0;
  IMXRT_TMR3.CH[3].COMP1  = 0xFFFF;
  IMXRT_TMR3.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(3);
}

uint16_t interrupt_qtimer3_ch2_counter_now(void) {
  return IMXRT_TMR3.CH[2].CNTR;
}

uint16_t interrupt_qtimer3_ch3_counter_now(void) {
  return IMXRT_TMR3.CH[3].CNTR;
}

// ============================================================================
// ISR vectors — owned by process_interrupt
// ============================================================================

static void pps_gpio_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;

  if (IMXRT_TMR3.CH[2].SCTRL & TMR_SCTRL_TCF) {
    process_interrupt_qtimer3_ch2_irq(dwt_raw);
  }

  if (IMXRT_TMR3.CH[3].SCTRL & TMR_SCTRL_TCF) {
    process_interrupt_qtimer3_ch3_irq(dwt_raw);
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

  const uint32_t pps_qtimer_at_edge = qtimer1_read_32_for_pps_capture();
  handle_event(*rt, dwt_isr_entry_raw, pps_qtimer_at_edge);
}

// ============================================================================
// QTimer3 CH2 ISR entry — OCXO1 (v19 windowed)
// ============================================================================

void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_raw) {
  g_qtimer3_ch2_diag.irq_count++;

  IMXRT_TMR3.CH[2].SCTRL &= ~TMR_SCTRL_TCF;

  auto* rt = g_rt_ocxo1;
  if (!rt || !rt->active) {
    g_qtimer3_ch2_diag.miss_count++;
    return;
  }

  if (rt->first_after_accept) {
    rt->running_total += (uint16_t)OCXO_COUNTS_PER_SECOND;
    rt->first_after_accept = false;
  } else {
    rt->running_total += 65536;
  }

  if (rt->running_total < rt->next_match_total) {
    rt->spurious_count++;
    return;
  }

  g_qtimer3_ch2_diag.dispatch_count++;

  const uint32_t matched_total = rt->next_match_total;

  rt->next_match_total += OCXO_COUNTS_PER_SECOND;
  rt->next_comp += (uint16_t)OCXO_COUNTS_PER_SECOND;

  IMXRT_TMR3.CH[2].COMP1 = rt->next_comp;
  IMXRT_TMR3.CH[2].CMPLD1 = rt->next_comp;
  rt->first_after_accept = true;

  handle_event(*rt, dwt_raw, matched_total);

  rt->last_cntr = IMXRT_TMR3.CH[2].CNTR;
}

// ============================================================================
// QTimer3 CH3 ISR entry — OCXO2 (v19 windowed)
// ============================================================================

void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_raw) {
  g_qtimer3_ch3_diag.irq_count++;

  IMXRT_TMR3.CH[3].SCTRL &= ~TMR_SCTRL_TCF;

  auto* rt = g_rt_ocxo2;
  if (!rt || !rt->active) {
    g_qtimer3_ch3_diag.miss_count++;
    return;
  }

  if (rt->first_after_accept) {
    rt->running_total += (uint16_t)OCXO_COUNTS_PER_SECOND;
    rt->first_after_accept = false;
  } else {
    rt->running_total += 65536;
  }

  if (rt->running_total < rt->next_match_total) {
    rt->spurious_count++;
    return;
  }

  g_qtimer3_ch3_diag.dispatch_count++;

  const uint32_t matched_total = rt->next_match_total;

  rt->next_match_total += OCXO_COUNTS_PER_SECOND;
  rt->next_comp += (uint16_t)OCXO_COUNTS_PER_SECOND;

  IMXRT_TMR3.CH[3].COMP1 = rt->next_comp;
  IMXRT_TMR3.CH[3].CMPLD1 = rt->next_comp;
  rt->first_after_accept = true;

  handle_event(*rt, dwt_raw, matched_total);

  rt->last_cntr = IMXRT_TMR3.CH[3].CNTR;
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

  if (kind == interrupt_subscriber_kind_t::PPS) {
    debug_log("pps.start", "requested");
    pps_drumbeat_bootstrap(*rt);
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    uint16_t now16 = IMXRT_TMR3.CH[2].CNTR;
    rt->last_cntr = now16;
    rt->running_total = 0;
    rt->spurious_count = 0;
    rt->first_after_accept = true;
    rt->next_comp = now16 + (uint16_t)OCXO_COUNTS_PER_SECOND;
    rt->next_match_total = OCXO_COUNTS_PER_SECOND;

    IMXRT_TMR3.CH[2].SCTRL &= ~TMR_SCTRL_TCF;
    IMXRT_TMR3.CH[2].COMP1  = rt->next_comp;
    IMXRT_TMR3.CH[2].CMPLD1 = rt->next_comp;
    IMXRT_TMR3.CH[2].SCTRL |= TMR_SCTRL_TCFIE;
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    uint16_t now16 = IMXRT_TMR3.CH[3].CNTR;
    rt->last_cntr = now16;
    rt->running_total = 0;
    rt->spurious_count = 0;
    rt->first_after_accept = true;
    rt->next_comp = now16 + (uint16_t)OCXO_COUNTS_PER_SECOND;
    rt->next_match_total = OCXO_COUNTS_PER_SECOND;

    IMXRT_TMR3.CH[3].SCTRL &= ~TMR_SCTRL_TCF;
    IMXRT_TMR3.CH[3].COMP1  = rt->next_comp;
    IMXRT_TMR3.CH[3].CMPLD1 = rt->next_comp;
    IMXRT_TMR3.CH[3].SCTRL |= TMR_SCTRL_TCFIE;
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
    IMXRT_TMR3.CH[2].SCTRL &= ~TMR_SCTRL_TCFIE;
  } else if (kind == interrupt_subscriber_kind_t::OCXO2) {
    IMXRT_TMR3.CH[3].SCTRL &= ~TMR_SCTRL_TCFIE;
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

  arm_qtimer3_ocxo_channels();

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

  IMXRT_TMR3.CH[2].SCTRL &= ~(TMR_SCTRL_TCF | TMR_SCTRL_TCFIE);
  IMXRT_TMR3.CH[3].SCTRL &= ~(TMR_SCTRL_TCF | TMR_SCTRL_TCFIE);

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 0);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

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

  p.add("qtimer3_ch2_irq_count", g_qtimer3_ch2_diag.irq_count);
  p.add("qtimer3_ch2_dispatch_count", g_qtimer3_ch2_diag.dispatch_count);
  p.add("qtimer3_ch2_miss_count", g_qtimer3_ch2_diag.miss_count);

  p.add("qtimer3_ch3_irq_count", g_qtimer3_ch3_diag.irq_count);
  p.add("qtimer3_ch3_dispatch_count", g_qtimer3_ch3_diag.dispatch_count);
  p.add("qtimer3_ch3_miss_count", g_qtimer3_ch3_diag.miss_count);

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

  p.add("qtimer3_ch2_cntr_now", (uint32_t)IMXRT_TMR3.CH[2].CNTR);
  p.add("qtimer3_ch3_cntr_now", (uint32_t)IMXRT_TMR3.CH[3].CNTR);

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

    s.add("next_comp", (uint32_t)rt.next_comp);
    s.add("next_match_total", rt.next_match_total);

    s.add("running_total", rt.running_total);
    s.add("last_cntr", (uint32_t)rt.last_cntr);
    s.add("spurious_count", rt.spurious_count);

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

    s.add("last_dwt_at_event", rt.last_event.dwt_at_event);
    s.add("last_gnss_ns_at_event", rt.last_event.gnss_ns_at_event);
    s.add("last_counter32_at_event", rt.last_event.counter32_at_event);
    s.add("last_correction_cycles", rt.last_event.dwt_event_correction_cycles);
    s.add("last_status", (uint32_t)rt.last_event.status);

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
