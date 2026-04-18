// ============================================================================
// process_interrupt.cpp — per-lane 1 kHz cadence + physical PPS machinery
// ============================================================================
//
// Three lanes (VCLOCK, OCXO1, OCXO2), each cadenced by its own dedicated
// QuadTimer compare channel at 1 kHz:
//
//   VCLOCK lane:  QTimer1 CH3 compare, +10000 counts per interval
//                 (clocked by GNSS 10 MHz VCLOCK; PCS=0)
//                 ISR is registered with TimePop via
//                 timepop_register_qtimer1_ch3_isr() — TimePop's
//                 qtimer1_irq_isr captures DWT as the first instruction
//                 and dispatches here when CH3's flag is set.
//
//   OCXO1 lane:   QTimer3 CH2 compare, +10000 counts per interval
//                 (clocked by OCXO1 10 MHz; PCS=2)
//                 ISR is owned directly by process_interrupt.
//
//   OCXO2 lane:   QTimer3 CH3 compare, +10000 counts per interval
//                 (clocked by OCXO2 10 MHz; PCS=3)
//                 ISR is owned directly by process_interrupt.
//
// Physical PPS GPIO — witness, dispatch authority, and epoch anchor:
//
//   The GPIO ISR captures DWT as its first instruction, and as
//   immediately-following instructions captures QTimer1 CH0+CH1 (the
//   cascaded 32-bit VCLOCK counter) and QTimer1 CH3 (the cadence
//   channel).  These three captures define the PPS moment in
//   hardware-counter terms.
//
//   When a rebootstrap request is pending (armed by alpha), the ISR
//   ALSO reprograms CH3's compare target so the next 1 ms cadence tick
//   fires VCLOCK_INTERVAL_COUNTS ticks after the PPS moment, and
//   resets the VCLOCK lane's tick_mod_1000 to 0.  This phase-locks
//   the VCLOCK one-second event cadence to the physical PPS edge:
//   from that moment on, vclock_callback fires exactly N seconds
//   after PPS (plus GPIO ISR latency, which is consistent).
//
//   This is the mechanism that gives pulse identity to VCLOCK cycles.
//   Before PPS anchor, "which VCLOCK tick is the second boundary" is
//   boot-random.  After PPS anchor, it's the tick coincident with the
//   physical PPS edge.
//
// See process_interrupt.h for the one-second event and snapshot contracts.
// ============================================================================

#include "process_interrupt.h"

#include "config.h"
#include "debug.h"
#include "process.h"
#include "payload.h"
#include "time.h"
#include "timepop.h"
#include "process_timepop.h"

#include <Arduino.h>
#include "imxrt.h"

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;

static constexpr uint32_t VCLOCK_INTERVAL_COUNTS = 10000U;  // 1 ms at 10 MHz

static constexpr uint32_t OCXO_INTERVAL_COUNTS = 10000U;    // 1 ms at 10 MHz
static constexpr uint32_t OCXO_COUNTS_PER_SECOND = 10000000U;

// Ticks per emitted one-second event.  1000 ticks × 1 ms = 1 s.
static constexpr uint32_t TICKS_PER_SECOND_EVENT = 1000U;

static constexpr int OCXO1_PIN = 14;
static constexpr int OCXO2_PIN = 15;

// ============================================================================
// Subscriber runtime
// ============================================================================

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
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

static const interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  { interrupt_subscriber_kind_t::VCLOCK, "VCLOCK", interrupt_provider_kind_t::QTIMER1, interrupt_lane_t::QTIMER1_CH3_COMP },
  { interrupt_subscriber_kind_t::OCXO1,  "OCXO1",  interrupt_provider_kind_t::QTIMER3, interrupt_lane_t::QTIMER3_CH2_COMP },
  { interrupt_subscriber_kind_t::OCXO2,  "OCXO2",  interrupt_provider_kind_t::QTIMER3, interrupt_lane_t::QTIMER3_CH3_COMP },
};

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS] = {};
static uint32_t g_subscriber_count = 0;
static bool g_interrupt_hw_ready = false;
static bool g_interrupt_runtime_ready = false;
static bool g_interrupt_irqs_enabled = false;

// PPS-anchored VCLOCK rebootstrap flag.  When set by alpha via
// interrupt_request_pps_rebootstrap(), the next physical PPS GPIO
// edge will re-phase the CH3 cadence to align with the PPS moment.
// Cleared by the GPIO ISR once the rebootstrap has been applied.
static volatile bool g_pps_rebootstrap_pending = false;

// Accumulated count of rebootstraps the ISR has actually performed.
// Useful for diagnostics / REPORT.
static volatile uint32_t g_pps_rebootstrap_count = 0;

static interrupt_subscriber_runtime_t* g_rt_vclock = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo1  = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2  = nullptr;

// ============================================================================
// Per-lane state
// ============================================================================

struct vclock_lane_t {
  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t tick_mod_1000 = 0;
  uint32_t logical_count32_at_last_second = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
};

static vclock_lane_t g_vclock_lane;

struct ocxo_lane_t {
  const char* name = nullptr;
  IMXRT_TMR_t* module = nullptr;
  uint8_t      channel = 0;
  uint8_t      pcs = 0;
  int          input_pin = -1;
  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t tick_mod_1000 = 0;
  uint32_t logical_count32_at_last_second = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
};

static ocxo_lane_t g_ocxo1_lane;
static ocxo_lane_t g_ocxo2_lane;

// ============================================================================
// PPS GPIO witness
// ============================================================================

struct pps_gpio_witness_t {
  uint32_t edge_count = 0;
  uint32_t last_dwt = 0;
  int64_t  last_gnss_ns = 0;
};

static pps_gpio_witness_t g_pps_gpio_witness;

static uint32_t g_gpio_irq_count = 0;
static uint32_t g_gpio_miss_count = 0;

struct pps_edge_snapshot_store_t {
  volatile uint32_t seq               = 0;
  volatile uint32_t sequence          = 0;
  volatile uint32_t dwt_at_edge       = 0;
  volatile uint32_t counter32_at_edge = 0;
  volatile uint16_t ch3_at_edge       = 0;
  volatile int64_t  gnss_ns_at_edge   = -1;
};

static pps_edge_snapshot_store_t g_pps_edge_store;
static pps_edge_dispatch_fn      g_pps_edge_dispatch = nullptr;

static inline void dmb_barrier(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

// ============================================================================
// QTimer1 32-bit counter (diagnostic read and ISR-entry capture)
// ============================================================================

static inline uint32_t qtimer1_read_32_for_diag(void) {
  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;
  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi2 = IMXRT_TMR1.CH[1].HOLD;
  return (hi1 != hi2) ? (((uint32_t)hi2 << 16) | (uint32_t)lo2)
                      : (((uint32_t)hi1 << 16) | (uint32_t)lo1);
}

uint32_t interrupt_qtimer1_counter32_now(void) { return qtimer1_read_32_for_diag(); }
uint16_t interrupt_qtimer3_ch2_counter_now(void) { return IMXRT_TMR3.CH[2].CNTR; }
uint16_t interrupt_qtimer3_ch3_counter_now(void) { return IMXRT_TMR3.CH[3].CNTR; }

// ============================================================================
// Compare channel helpers
// ============================================================================

static inline void qtimer1_ch3_clear_compare_flag(void) {
  IMXRT_TMR1.CH[3].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}
static inline void qtimer1_ch3_program_compare(uint16_t target_low16) {
  qtimer1_ch3_clear_compare_flag();
  IMXRT_TMR1.CH[3].COMP1  = target_low16;
  IMXRT_TMR1.CH[3].CMPLD1 = target_low16;
  qtimer1_ch3_clear_compare_flag();
  IMXRT_TMR1.CH[3].CSCTRL |= TMR_CSCTRL_TCF1EN;
}
static inline void qtimer1_ch3_disable_compare(void) {
  IMXRT_TMR1.CH[3].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer1_ch3_clear_compare_flag();
}

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
// Subscriber lookup helpers
// ============================================================================

static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    if (g_subscribers[i].desc && g_subscribers[i].desc->kind == kind) {
      return &g_subscribers[i];
    }
  }
  return nullptr;
}

static ocxo_lane_t* ocxo_lane_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_ocxo1_lane;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_ocxo2_lane;
  return nullptr;
}

static const char* dispatch_timer_name(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK: return "VCLOCK_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO1:  return "OCXO1_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO2:  return "OCXO2_DISPATCH";
    default: return "";
  }
}

// ============================================================================
// Diag fill + event dispatch
// ============================================================================

static void fill_diag(interrupt_capture_diag_t& diag,
                      const interrupt_subscriber_runtime_t& rt,
                      const interrupt_event_t& event) {
  diag.enabled = true;
  diag.provider = rt.desc->provider;
  diag.lane = rt.desc->lane;
  diag.kind = rt.desc->kind;

  diag.dwt_at_event       = event.dwt_at_event;
  diag.gnss_ns_at_event   = event.gnss_ns_at_event;
  diag.counter32_at_event = event.counter32_at_event;

  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    diag.pps_edge_sequence          = g_pps_gpio_witness.edge_count;
    diag.pps_edge_dwt_isr_entry_raw = g_pps_gpio_witness.last_dwt;
    diag.pps_edge_gnss_ns           = g_pps_gpio_witness.last_gnss_ns;
    diag.pps_edge_minus_event_ns =
        (g_pps_gpio_witness.last_gnss_ns >= 0 && event.gnss_ns_at_event != 0)
            ? (g_pps_gpio_witness.last_gnss_ns - (int64_t)event.gnss_ns_at_event)
            : 0;
  }
}

static void maybe_dispatch_event(interrupt_subscriber_runtime_t& rt) {
  if (!rt.subscribed || !rt.sub.on_event) return;
  rt.sub.on_event(rt.last_event, &rt.last_diag, rt.sub.user_data);
}

static void deferred_dispatch_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;
  maybe_dispatch_event(*rt);
}

static void emit_one_second_event(interrupt_subscriber_runtime_t& rt,
                                  uint32_t dwt_at_event,
                                  uint32_t authored_counter32) {
  if (!rt.active) return;

  interrupt_event_t event {};
  event.kind         = rt.desc->kind;
  event.provider     = rt.desc->provider;
  event.lane         = rt.desc->lane;
  event.dwt_at_event = dwt_at_event;
  event.counter32_at_event = authored_counter32;

  const int64_t gnss_ns = time_dwt_to_gnss_ns(dwt_at_event);
  event.gnss_ns_at_event = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
  event.status = interrupt_event_status_t::OK;

  interrupt_capture_diag_t diag {};
  fill_diag(diag, rt, event);

  rt.last_event = event;
  rt.last_diag = diag;
  rt.has_fired = true;
  rt.event_count++;

  timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
}

// ============================================================================
// VCLOCK lane — QTimer1 CH3 (hosted by TimePop)
// ============================================================================

static void vclock_lane_arm_bootstrap(void) {
  const uint16_t now16 = IMXRT_TMR1.CH[3].CNTR;
  g_vclock_lane.phase_bootstrapped = true;
  g_vclock_lane.bootstrap_count++;
  g_vclock_lane.tick_mod_1000 = 0;
  g_vclock_lane.compare_target = (uint16_t)(now16 + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
}

static void vclock_lane_advance_compare(void) {
  g_vclock_lane.compare_target =
      (uint16_t)(g_vclock_lane.compare_target + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
}

static void vclock_cadence_isr(uint32_t dwt_raw) {
  g_vclock_lane.irq_count++;
  if (g_rt_vclock) g_rt_vclock->irq_count++;

  qtimer1_ch3_clear_compare_flag();

  if (!g_vclock_lane.active || !g_rt_vclock || !g_rt_vclock->active) {
    g_vclock_lane.miss_count++;
    return;
  }

  if (!g_vclock_lane.phase_bootstrapped) {
    vclock_lane_arm_bootstrap();
    g_vclock_lane.miss_count++;
    return;
  }

  vclock_lane_advance_compare();
  g_vclock_lane.cadence_hits_total++;

  if (++g_vclock_lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
    g_vclock_lane.tick_mod_1000 = 0;
    g_vclock_lane.logical_count32_at_last_second += VCLOCK_COUNTS_PER_SECOND;
    emit_one_second_event(*g_rt_vclock, dwt_raw,
                          g_vclock_lane.logical_count32_at_last_second);
  }
}

// ============================================================================
// OCXO lanes — QTimer3 CH2 / CH3
// ============================================================================

static void ocxo_lane_arm_bootstrap(ocxo_lane_t& lane) {
  const uint16_t now16 = lane.module->CH[lane.channel].CNTR;
  lane.phase_bootstrapped = true;
  lane.bootstrap_count++;
  lane.tick_mod_1000 = 0;
  lane.compare_target = (uint16_t)(now16 + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(lane.channel, lane.compare_target);
}

static void ocxo_lane_advance_compare(ocxo_lane_t& lane) {
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(lane.channel, lane.compare_target);
}

static void handle_ocxo_qtimer16_irq(ocxo_lane_t& lane,
                                     interrupt_subscriber_runtime_t& rt,
                                     uint32_t dwt_raw) {
  lane.irq_count++;
  rt.irq_count++;

  qtimer3_clear_compare_flag(lane.channel);

  if (!lane.active || !rt.active) {
    lane.miss_count++;
    return;
  }

  if (!lane.phase_bootstrapped) {
    ocxo_lane_arm_bootstrap(lane);
    lane.miss_count++;
    return;
  }

  ocxo_lane_advance_compare(lane);
  lane.cadence_hits_total++;

  if (++lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
    lane.tick_mod_1000 = 0;
    lane.logical_count32_at_last_second += OCXO_COUNTS_PER_SECOND;
    emit_one_second_event(rt, dwt_raw, lane.logical_count32_at_last_second);
  }
}

void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_raw) {
  if (g_rt_ocxo1) {
    handle_ocxo_qtimer16_irq(g_ocxo1_lane, *g_rt_ocxo1, dwt_raw);
  }
}

void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_raw) {
  if (g_rt_ocxo2) {
    handle_ocxo_qtimer16_irq(g_ocxo2_lane, *g_rt_ocxo2, dwt_raw);
  }
}

static void qtimer3_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  if (IMXRT_TMR3.CH[2].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch2_irq(dwt_raw);
  if (IMXRT_TMR3.CH[3].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch3_irq(dwt_raw);
}

// ============================================================================
// Physical PPS GPIO — witness, dispatch authority, epoch anchor
// ============================================================================

static void pps_edge_dispatch_trampoline(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_pps_edge_dispatch) return;
  const pps_edge_snapshot_t snap = interrupt_last_pps_edge();
  g_pps_edge_dispatch(snap);
}

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw) {
  // ── First-instruction captures ──
  //
  // Read the VCLOCK counter state IMMEDIATELY after the DWT capture.
  // These three captures (DWT, counter32, ch3) define the PPS moment
  // in hardware-counter terms.  Total additional cost vs. the old ISR
  // is ~8 loads (counter32 is dual-read for cascade safety).  At
  // 1 GHz this is on the order of tens of nanoseconds — well under
  // one VCLOCK period.
  //
  const uint32_t counter32 = qtimer1_read_32_for_diag();
  const uint16_t ch3_now   = IMXRT_TMR1.CH[3].CNTR;

  g_gpio_irq_count++;
  g_pps_gpio_witness.edge_count++;
  g_pps_gpio_witness.last_dwt = dwt_isr_entry_raw;

  const int64_t gnss_ns = time_dwt_to_gnss_ns(dwt_isr_entry_raw);
  g_pps_gpio_witness.last_gnss_ns = gnss_ns;

  // ── PPS-anchored VCLOCK rebootstrap ──
  //
  // If alpha has armed a rebootstrap, consume the flag and re-phase
  // the VCLOCK cadence so that:
  //
  //   • The next CH3 compare-match fires exactly VCLOCK_INTERVAL_COUNTS
  //     ticks after ch3_now (1 ms after the PPS moment).
  //   • tick_mod_1000 is reset to 0 so the first post-anchor one-second
  //     event fires exactly VCLOCK_COUNTS_PER_SECOND ticks after PPS.
  //   • logical_count32_at_last_second is seeded from counter32 so
  //     subsequent counter32_at_event values maintain the hardware-
  //     count identity: counter32_at_event N seconds after anchor
  //     equals counter32 + N * VCLOCK_COUNTS_PER_SECOND.
  //
  // The rebootstrap runs in GPIO ISR priority (0) and therefore
  // preempts the QTimer1 CH3 ISR (priority 16 via TimePop).  No race
  // with vclock_cadence_isr can occur here.
  //
  if (g_pps_rebootstrap_pending) {
    g_pps_rebootstrap_pending = false;
    g_pps_rebootstrap_count++;

    g_vclock_lane.phase_bootstrapped = true;
    g_vclock_lane.tick_mod_1000 = 0;
    g_vclock_lane.compare_target =
        (uint16_t)(ch3_now + (uint16_t)VCLOCK_INTERVAL_COUNTS);
    g_vclock_lane.logical_count32_at_last_second = counter32;
    qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
  }

  // ── Snapshot publication (seqlock) ──
  g_pps_edge_store.seq++;
  dmb_barrier();
  g_pps_edge_store.sequence          = g_pps_gpio_witness.edge_count;
  g_pps_edge_store.dwt_at_edge       = dwt_isr_entry_raw;
  g_pps_edge_store.counter32_at_edge = counter32;
  g_pps_edge_store.ch3_at_edge       = ch3_now;
  g_pps_edge_store.gnss_ns_at_edge   = gnss_ns;
  dmb_barrier();
  g_pps_edge_store.seq++;

  if (g_pps_edge_dispatch) {
    timepop_arm_asap(pps_edge_dispatch_trampoline, nullptr, "PPS_EDGE_DISPATCH");
  }
}

static void pps_gpio_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(dwt_raw);
}

void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn fn) {
  g_pps_edge_dispatch = fn;
}

pps_edge_snapshot_t interrupt_last_pps_edge(void) {
  pps_edge_snapshot_t out {};
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_pps_edge_store.seq;
    dmb_barrier();
    out.sequence          = g_pps_edge_store.sequence;
    out.dwt_at_edge       = g_pps_edge_store.dwt_at_edge;
    out.counter32_at_edge = g_pps_edge_store.counter32_at_edge;
    out.ch3_at_edge       = g_pps_edge_store.ch3_at_edge;
    out.gnss_ns_at_edge   = g_pps_edge_store.gnss_ns_at_edge;
    dmb_barrier();
    const uint32_t s2 = g_pps_edge_store.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }
  return out;
}

// ============================================================================
// Subscribe / start / stop
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub) {
  if (!g_interrupt_runtime_ready) return false;
  if (sub.kind == interrupt_subscriber_kind_t::NONE || !sub.on_event) return false;
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

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = true;
    // Leave phase_bootstrapped as-is.  If set (by a prior PPS rebootstrap
    // or a prior cadence ISR), respect it.  If unset, the next cadence
    // ISR will bootstrap.
    qtimer1_ch3_program_compare((uint16_t)(IMXRT_TMR1.CH[3].CNTR + 1));
    return true;
  }

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane || !lane->initialized) return false;
  lane->active = true;
  lane->phase_bootstrapped = false;
  lane->tick_mod_1000 = 0;
  qtimer3_program_compare(lane->channel,
                          (uint16_t)(lane->module->CH[lane->channel].CNTR + 1));
  return true;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  rt->active = false;
  rt->stop_count++;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = false;
    qtimer1_ch3_disable_compare();
    return true;
  }

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane) return false;
  lane->active = false;
  qtimer3_disable_compare(lane->channel);
  return true;
}

// ── PPS-anchored epoch installation (public API) ──

void interrupt_request_pps_rebootstrap(void) {
  g_pps_rebootstrap_pending = true;
}

bool interrupt_pps_rebootstrap_pending(void) {
  return g_pps_rebootstrap_pending;
}

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_event;
}

const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_diag;
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {}

// ============================================================================
// Hardware + runtime init
// ============================================================================

static void qtimer1_ch3_init_vclock_cadence(void) {
  IMXRT_TMR1.CH[3].CTRL   = 0;
  IMXRT_TMR1.CH[3].SCTRL  = 0;
  IMXRT_TMR1.CH[3].CSCTRL = 0;
  IMXRT_TMR1.CH[3].LOAD   = 0;
  IMXRT_TMR1.CH[3].CNTR   = 0;
  IMXRT_TMR1.CH[3].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[3].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  qtimer1_ch3_disable_compare();
}

void process_interrupt_init_hardware(void) {
  if (g_interrupt_hw_ready) return;

  g_ocxo1_lane.name = "OCXO1";
  g_ocxo1_lane.module = &IMXRT_TMR3;
  g_ocxo1_lane.channel = 2;
  g_ocxo1_lane.pcs = 2;
  g_ocxo1_lane.input_pin = OCXO1_PIN;

  g_ocxo2_lane.name = "OCXO2";
  g_ocxo2_lane.module = &IMXRT_TMR3;
  g_ocxo2_lane.channel = 3;
  g_ocxo2_lane.pcs = 3;
  g_ocxo2_lane.input_pin = OCXO2_PIN;

  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
  *(portConfigRegister(OCXO1_PIN)) = 1;
  *(portConfigRegister(OCXO2_PIN)) = 1;
  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

  IMXRT_TMR3.CH[2].CTRL = 0; IMXRT_TMR3.CH[2].SCTRL = 0;
  IMXRT_TMR3.CH[2].CSCTRL = 0; IMXRT_TMR3.CH[2].LOAD = 0;
  IMXRT_TMR3.CH[2].CNTR = 0; IMXRT_TMR3.CH[2].COMP1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo1_lane.pcs);
  qtimer3_disable_compare(2);

  IMXRT_TMR3.CH[3].CTRL = 0; IMXRT_TMR3.CH[3].SCTRL = 0;
  IMXRT_TMR3.CH[3].CSCTRL = 0; IMXRT_TMR3.CH[3].LOAD = 0;
  IMXRT_TMR3.CH[3].CNTR = 0; IMXRT_TMR3.CH[3].COMP1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo2_lane.pcs);
  qtimer3_disable_compare(3);

  g_ocxo1_lane.initialized = true;
  g_ocxo2_lane.initialized = true;

  qtimer1_ch3_init_vclock_cadence();
  g_vclock_lane.initialized = true;

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
    if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) g_rt_vclock = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1) g_rt_ocxo1 = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO2) g_rt_ocxo2 = &rt;
  }

  g_pps_gpio_witness = pps_gpio_witness_t{};
  g_gpio_irq_count = 0;
  g_gpio_miss_count = 0;
  g_pps_rebootstrap_pending = false;
  g_pps_rebootstrap_count = 0;

  g_pps_edge_store = pps_edge_snapshot_store_t{};
  g_pps_edge_dispatch = nullptr;

  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 32);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  timepop_register_qtimer1_ch3_isr(vclock_cadence_isr);

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
  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("pps_rebootstrap_count", g_pps_rebootstrap_count);

  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("gpio_edge_count", g_pps_gpio_witness.edge_count);
  p.add("gpio_last_dwt", g_pps_gpio_witness.last_dwt);
  p.add("pps_edge_gnss_ns", g_pps_gpio_witness.last_gnss_ns);

  const pps_edge_snapshot_t snap = interrupt_last_pps_edge();
  p.add("pps_edge_sequence", snap.sequence);
  p.add("pps_edge_dwt", snap.dwt_at_edge);
  p.add("pps_edge_counter32", snap.counter32_at_edge);
  p.add("pps_edge_ch3", (uint32_t)snap.ch3_at_edge);
  p.add("pps_edge_gnss_ns_snapshot", snap.gnss_ns_at_edge);
  p.add("pps_edge_dispatch_registered", g_pps_edge_dispatch != nullptr);

  p.add("vclock_irq_count", g_vclock_lane.irq_count);
  p.add("vclock_miss_count", g_vclock_lane.miss_count);
  p.add("vclock_bootstrap_count", g_vclock_lane.bootstrap_count);
  p.add("vclock_cadence_hits_total", g_vclock_lane.cadence_hits_total);
  p.add("vclock_compare_target", (uint32_t)g_vclock_lane.compare_target);
  p.add("vclock_tick_mod_1000", g_vclock_lane.tick_mod_1000);
  p.add("vclock_logical_count32", g_vclock_lane.logical_count32_at_last_second);

  p.add("ocxo1_irq_count", g_ocxo1_lane.irq_count);
  p.add("ocxo1_miss_count", g_ocxo1_lane.miss_count);
  p.add("ocxo1_bootstrap_count", g_ocxo1_lane.bootstrap_count);
  p.add("ocxo1_cadence_hits_total", g_ocxo1_lane.cadence_hits_total);
  p.add("ocxo1_compare_target", (uint32_t)g_ocxo1_lane.compare_target);
  p.add("ocxo1_tick_mod_1000", g_ocxo1_lane.tick_mod_1000);
  p.add("ocxo1_logical_count32", g_ocxo1_lane.logical_count32_at_last_second);

  p.add("ocxo2_irq_count", g_ocxo2_lane.irq_count);
  p.add("ocxo2_miss_count", g_ocxo2_lane.miss_count);
  p.add("ocxo2_bootstrap_count", g_ocxo2_lane.bootstrap_count);
  p.add("ocxo2_cadence_hits_total", g_ocxo2_lane.cadence_hits_total);
  p.add("ocxo2_compare_target", (uint32_t)g_ocxo2_lane.compare_target);
  p.add("ocxo2_tick_mod_1000", g_ocxo2_lane.tick_mod_1000);
  p.add("ocxo2_logical_count32", g_ocxo2_lane.logical_count32_at_last_second);

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

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:    return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1:     return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:     return "OCXO2";
    case interrupt_subscriber_kind_t::TIME_TEST: return "TIME_TEST";
    default:                                     return "NONE";
  }
}

const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider) {
  switch (provider) {
    case interrupt_provider_kind_t::QTIMER1:  return "QTIMER1";
    case interrupt_provider_kind_t::QTIMER3:  return "QTIMER3";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default:                                  return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::QTIMER1_CH3_COMP: return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER3_CH2_COMP: return "QTIMER3_CH2_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}