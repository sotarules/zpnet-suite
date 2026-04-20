// ============================================================================
// process_interrupt.cpp — per-lane 1 kHz cadence + physical PPS machinery
// ============================================================================
//
// Three one-second lanes (VCLOCK, OCXO1, OCXO2), each cadenced by its own
// dedicated QuadTimer compare channel at 1 kHz, plus a TimePop scheduler
// channel and a physical PPS GPIO edge:
//
//   VCLOCK lane:  QTimer1 CH3 compare, +10000 counts per interval
//                 (clocked by GNSS 10 MHz VCLOCK; PCS=0)
//                 ISR is vclock_cadence_isr, called directly from this
//                 file's qtimer1_isr dispatcher when CH3's flag is set.
//
//   OCXO1 lane:   QTimer3 CH2 compare, +10000 counts per interval
//                 (clocked by OCXO1 10 MHz; PCS=2)
//                 ISR is owned directly by process_interrupt.
//
//   OCXO2 lane:   QTimer3 CH3 compare, +10000 counts per interval
//                 (clocked by OCXO2 10 MHz; PCS=3)
//                 ISR is owned directly by process_interrupt.
//
//   TimePop:      QTimer1 CH2 compare, varied intervals based on what
//                 TimePop has scheduled.  TimePop registers an
//                 IRQ-context handler via interrupt_register_qtimer1_
//                 ch2_handler() and receives a standard interrupt_event_t
//                 (kind = TIMEPOP) on every CH2 compare-match.  The
//                 dispatcher computes the full event payload (DWT,
//                 counter32, gnss_ns) before invoking the handler.
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
#include <math.h>

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

// ── CH2 (TimePop scheduler) — flag clear only.  TimePop owns CH2
// compare-register programming via its own helpers; process_interrupt
// only clears the TCF1 flag in the QTimer1 ISR before invoking the
// registered TimePop handler.
static inline void qtimer1_ch2_clear_compare_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
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

    // Mirror the PPS/VCLOCK coincidence measurement from the event.
    diag.pps_coincidence_cycles = event.pps_coincidence_cycles;
    diag.pps_coincidence_valid  = event.pps_coincidence_valid;
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
                                  uint32_t authored_counter32,
                                  uint32_t pps_coincidence_cycles = 0,
                                  bool     pps_coincidence_valid  = false) {
  if (!rt.active) return;

  interrupt_event_t event {};
  event.kind         = rt.desc->kind;
  event.provider     = rt.desc->provider;
  event.lane         = rt.desc->lane;
  event.dwt_at_event = dwt_at_event;
  event.counter32_at_event = authored_counter32;
  event.pps_coincidence_cycles = pps_coincidence_cycles;
  event.pps_coincidence_valid  = pps_coincidence_valid;

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

    // ── PPS/VCLOCK coincidence measurement ──
    //
    // This ISR fires on the 1000th CH3 compare-match — the VCLOCK
    // one-second boundary.  Under PPS-anchored operation the next
    // physical PPS edge is the same moment, modulo ISR latencies.
    // GPIO (priority 0) preempts QTimer1 (priority 1), so when both
    // pend coincidentally, GPIO runs first and THIS ISR lands strictly
    // after the GPIO ISR completes.
    //
    // The measurement is the raw DWT-cycles difference.  See
    // process_interrupt.h for the threshold rationale.
    uint32_t coincidence_cycles = 0;
    bool     coincidence_valid  = false;
    {
      const pps_edge_snapshot_t snap = interrupt_last_pps_edge();
      if (snap.sequence > 0) {
        const int32_t cycles_since_pps =
            (int32_t)(dwt_raw - snap.dwt_at_edge);
        if (cycles_since_pps >= 0 &&
            cycles_since_pps < DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES) {
          coincidence_cycles = (uint32_t)cycles_since_pps;
          coincidence_valid  = true;
        }
      }
    }

    emit_one_second_event(*g_rt_vclock, dwt_raw,
                          g_vclock_lane.logical_count32_at_last_second,
                          coincidence_cycles,
                          coincidence_valid);
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
// QTimer1 vector — dispatches CH2 (TimePop) and CH3 (VCLOCK cadence)
// ============================================================================
//
// Single shared vector for QTimer1.  Captures DWT as the first
// instruction, then dispatches each channel whose TCF1 flag is set.
// CH2 first (TimePop scheduler heartbeat), then CH3 (VCLOCK cadence).
// Both handlers receive the same first-instruction DWT value.
//
// CH2 dispatch packages a standard interrupt_event_t (kind = TIMEPOP)
// with full payload — dwt_at_event, gnss_ns_at_event, counter32_at_event
// — and invokes the registered TimePop handler in IRQ context.
//
// CH3 dispatch invokes vclock_cadence_isr directly (no registration
// hop — it's an internal call within process_interrupt).
//
static volatile interrupt_qtimer1_ch2_handler_fn g_qtimer1_ch2_handler = nullptr;

void interrupt_register_qtimer1_ch2_handler(interrupt_qtimer1_ch2_handler_fn cb) {
  g_qtimer1_ch2_handler = cb;
}

// ============================================================================
// QTimer1 CH3 witness — torture-sanity test of timekeeping math
// ============================================================================
//
// When witness mode is ON, the CH3 branch of qtimer1_isr routes to the
// witness path instead of vclock_cadence_isr.  The witness fires nine
// times per PPS second at 100, 200, ..., 900 ms offsets from the PPS
// anchor.  Each fire captures DWT as first instruction, computes the
// residual against the caller's predicted DWT at the VCLOCK target, and
// feeds the Welford.
//
// The witness self-rotates: on slot N's fire, it arms CH3 for slot N+1.
// After slot 9 (900 ms) fires, CH3 is disabled; the next PPS edge re-
// arms slot 1 (100 ms).  PPS at the 0 ms boundary is deliberately
// skipped — that's where GPIO ISR contention would contaminate the
// measurement.
//
// DWT prediction math:
//   target_dwt[N] = anchor_dwt + round((N * 0.1) * dwt_cycles_per_second)
//   target_vclock[N] = (anchor_counter32 + N * 1_000_000) & 0xFFFF
//                      (cast to 16-bit because CH3 compares low 16 bits)
// where anchor_counter32 and anchor_dwt are captured at the most recent
// PPS edge and dwt_cycles_per_second is the current DWT rate prediction.
//
// Residual in nanoseconds (at 1008 MHz where 1 cycle = 125/126 ns):
//   residual_cycles = dwt_at_isr_entry - target_dwt[N]
//   residual_ns     = residual_cycles * 125 / 126
//
// Positive residual means DWT advanced past target before the ISR
// captured it — pure ISR entry latency.  Negative residual would imply
// DWT reads earlier than target — which should be impossible if math is
// right, so it accuses the math.

static constexpr uint32_t WITNESS_N_SLOTS = 9;

// 1 ms = 10,000 VCLOCK ticks; 100 ms = 1,000,000 VCLOCK ticks.
static constexpr uint32_t WITNESS_VCLOCK_TICKS_PER_SLOT = 1000000;

// Reject residuals above this magnitude as garbage — something must
// have missed the fire window or preempted it catastrophically.
// 100,000 cycles at 1008 MHz ≈ 99 µs.
static constexpr int32_t WITNESS_REJECT_CYCLES = 100000;

struct witness_welford_t {
  uint64_t n;
  double   mean;       // in DWT cycles
  double   m2;
  int32_t  min_val;    // cycles
  int32_t  max_val;    // cycles
};

static volatile interrupt_witness_mode_t g_witness_mode =
    interrupt_witness_mode_t::OFF;

// Slot state for the CURRENT in-flight slot.  Arm computes all three
// fields atomically; the ISR reads them to validate the fire.
struct witness_slot_t {
  uint32_t target_vclock;    // full 32-bit VCLOCK target (for validation)
  uint32_t target_dwt;       // predicted DWT at target
  uint8_t  slot_index;       // 1..WITNESS_N_SLOTS (0 / 0xFF = no slot armed)
};

static volatile witness_slot_t g_witness_current = { 0, 0, 0xFF };

// Anchor values from the most recent PPS edge (captured in alpha-
// supplied arm call).  Used to compute the NEXT slot when this one
// fires.
static volatile uint32_t g_witness_anchor_counter32 = 0;
static volatile uint32_t g_witness_anchor_dwt       = 0;
static volatile uint32_t g_witness_anchor_dwt_cps   = 1008000000;

static witness_welford_t g_witness_welford = {};
static witness_welford_t g_witness_lag_welford = {};
static witness_welford_t g_witness_gpio_counter_delay_welford = {};

static volatile uint64_t g_witness_fires_total    = 0;
static volatile uint64_t g_witness_fires_rejected = 0;

volatile uint32_t g_witness_floor_cycles = 0;

// ── Per-slot capture buffer ──
//
// Circular capture of the last second's fires (one entry per slot,
// indexed by slot_index).  Each struct preserves ALL of the raw
// inputs to the residual calculation, unreduced, so that post-hoc
// analysis of any anomalous residual can trace it back to first
// principles:  which anchor was in play, what the targets were, what
// the ISR actually captured.
//
// Written on every real (non-wrap-alias) CH3 fire in witness_ch3_isr,
// INCLUDING fires that will subsequently be rejected by the ±tolerance
// filter.  The whole point is to see what was fed into the
// accept/reject decision.
//
// Dumped via the WITNESS_DUMP command — the regular REPORT surface
// stays compact.  One full second's worth of captures survives until
// overwritten by the next second's fires.
struct witness_capture_t {
  // Which slot (0..WITNESS_N_SLOTS) this capture represents, and a
  // monotonic sequence counter.  valid=false on startup until first
  // write to this slot.
  uint32_t seq;
  uint8_t  slot_index;
  bool     valid;

  // Anchor values (from g_witness_anchor_* at arm time, as seen by
  // witness_arm_slot).  These are the PPS-edge-derived inputs that
  // target_vclock and target_dwt were computed from.
  uint32_t anchor_counter32;
  uint32_t anchor_dwt;
  uint32_t anchor_dwt_cps;

  // Targets as programmed for this slot.
  uint32_t target_vclock;
  uint32_t target_dwt;

  // ISR-entry captures (this is what the fire actually saw).
  uint32_t dwt_at_fire;         // ARM_DWT_CYCCNT at first ISR instruction
  uint32_t counter32_at_fire;   // cascade-read 32-bit VCLOCK counter
  uint16_t ch3_cntr_at_fire;    // CH3's own low-16 counter

  // Derived diagnostics (same arithmetic as the ISR performs for the
  // Welford; captured verbatim for easy correlation).
  int32_t  vclock_delta;        // counter32_at_fire - target_vclock
  int32_t  residual_cycles;     // dwt_at_fire - target_dwt
  int32_t  lag_ticks;           // (counter32 & 0xFFFF) - ch3_cntr
};

// The capture array has WITNESS_N_SLOTS + 1 entries, indexed 0..N.
// Slot 0 is retained only as an unused sentinel entry so the live
// witness slots remain naturally indexed 1..WITNESS_N_SLOTS.
static witness_capture_t g_witness_captures[WITNESS_N_SLOTS + 1] = {};
static volatile uint32_t g_witness_capture_seq = 0;

static inline void witness_captures_reset(void) {
  for (uint8_t i = 0; i <= WITNESS_N_SLOTS; i++) {
    g_witness_captures[i] = {};
  }
  g_witness_capture_seq = 0;
}

// ── Sticky anomaly buffer ──
//
// A small fixed-size buffer that captures anomalous fires and PRESERVES
// them until the next WITNESS_RESET or WITNESS_START.  Unlike the
// per-slot capture buffer (which gets overwritten every second), this
// buffer holds anomalies indefinitely so rare events survive until we
// look at them.
//
// Anomaly criterion: |residual_cycles| > WITNESS_ANOMALY_CYCLES.  The
// healthy residual cluster sits around +128 cycles with cache-state
// variation of a few cycles either side, so 500 cycles is ~4x the
// healthy mean — well outside any non-anomalous behavior.
//
// Overwrite policy: OLDEST-WINS.  Once full, the buffer ignores new
// anomalies.  The first WITNESS_ANOMALY_BUFFER_SIZE anomalies seen
// are frozen in place until reset.  This preserves startup anomalies
// (which may be transient but important to understand) rather than
// letting them churn through as the system runs.
//
// A separate monotonic counter (g_witness_anomaly_count_total) tracks
// the total number of anomalies seen regardless of buffer capacity, so
// the dump can report "N anomalies seen, first 16 captured."
//
// Only slots 1..WITNESS_N_SLOTS are active witness slots.
static constexpr int32_t  WITNESS_ANOMALY_CYCLES      = 500;
static constexpr uint8_t  WITNESS_ANOMALY_BUFFER_SIZE = 16;

struct witness_anomaly_t {
  witness_capture_t cap;       // full verbatim capture (same struct)
  uint64_t fires_total_at_capture;  // for "when during the run" context
};

static witness_anomaly_t g_witness_anomalies[WITNESS_ANOMALY_BUFFER_SIZE] = {};
static volatile uint8_t  g_witness_anomaly_slot_next = 0;   // next slot to fill
static volatile uint32_t g_witness_anomaly_count_total = 0; // monotonic, uncapped

static inline void witness_anomalies_reset(void) {
  for (uint8_t i = 0; i < WITNESS_ANOMALY_BUFFER_SIZE; i++) {
    g_witness_anomalies[i] = {};
  }
  g_witness_anomaly_slot_next   = 0;
  g_witness_anomaly_count_total = 0;
}

static inline void witness_anomaly_maybe_capture(
    const witness_capture_t& cap,
    uint64_t fires_total)
{
  if (cap.residual_cycles >  WITNESS_ANOMALY_CYCLES ||
      cap.residual_cycles < -WITNESS_ANOMALY_CYCLES) {
    g_witness_anomaly_count_total++;
    // Oldest-wins: write only while slots remain.
    if (g_witness_anomaly_slot_next < WITNESS_ANOMALY_BUFFER_SIZE) {
      g_witness_anomalies[g_witness_anomaly_slot_next].cap = cap;
      g_witness_anomalies[g_witness_anomaly_slot_next].fires_total_at_capture
          = fires_total;
      g_witness_anomaly_slot_next++;
    }
  }
}

static inline void witness_welford_reset(witness_welford_t& w) {
  w.n = 0;
  w.mean = 0.0;
  w.m2 = 0.0;
  w.min_val = 0;
  w.max_val = 0;
}

static inline void witness_welford_update(witness_welford_t& w, int32_t sample) {
  w.n++;
  if (w.n == 1) {
    w.min_val = sample;
    w.max_val = sample;
  } else {
    if (sample < w.min_val) w.min_val = sample;
    if (sample > w.max_val) w.max_val = sample;
  }
  const double delta  = (double)sample - w.mean;
  w.mean += delta / (double)w.n;
  const double delta2 = (double)sample - w.mean;
  w.m2 += delta * delta2;
}

static inline double witness_welford_stddev(const witness_welford_t& w) {
  if (w.n < 2) return 0.0;
  return sqrt(w.m2 / (double)(w.n - 1));
}

// Cycles to nanoseconds at 1008 MHz: 1 cycle = 125/126 ns exactly.
// For residuals in the ~100 ns range this rational conversion matters
// for the metrics-board headline number.
static inline double witness_cycles_to_ns(double cycles) {
  return cycles * (125.0 / 126.0);
}

static void witness_update_floor_from_captures(void) {
  bool found = false;
  uint32_t best = 0;

  for (uint8_t i = 1; i <= WITNESS_N_SLOTS; i++) {
    const witness_capture_t& cap = g_witness_captures[i];
    if (!cap.valid) continue;
    if (cap.vclock_delta != 2) continue;
    if (cap.residual_cycles < 0) continue;
    if (cap.residual_cycles > WITNESS_REJECT_CYCLES) continue;

    const uint32_t residual = (uint32_t)cap.residual_cycles;
    if (!found || residual < best) {
      best = residual;
      found = true;
    }
  }

  if (found) {
    g_witness_floor_cycles = best;
  }
}

// Compute the target_dwt for slot N given the anchor.
//   delta_dwt = round(N * dwt_cps / 10)  because slot N is at N*100ms
//   target_dwt = anchor_dwt + delta_dwt
// Uses 64-bit intermediate to avoid overflow — (9 * 1e9) fits in 64 bits.
static inline uint32_t witness_compute_target_dwt(uint32_t anchor_dwt,
                                                   uint32_t dwt_cps,
                                                   uint32_t slot_index) {
  const uint64_t delta = ((uint64_t)slot_index * (uint64_t)dwt_cps + 5ULL) / 10ULL;
  return anchor_dwt + (uint32_t)delta;
}

// Compute the target_vclock (full 32-bit) for slot N given the anchor.
static inline uint32_t witness_compute_target_vclock(uint32_t anchor_counter32,
                                                      uint32_t slot_index) {
  return anchor_counter32 + slot_index * WITNESS_VCLOCK_TICKS_PER_SLOT;
}

// Arm CH3 for a specific slot.  Called from:
//   • interrupt_witness_arm_first_slot (slot_index = 1, from PPS edge)
//   • witness_ch3_isr itself (slot_index = 2..9, self-rotation)
// At slot_index == WITNESS_N_SLOTS + 1 the witness disables CH3 and
// waits for the next PPS edge to re-arm slot 1.
static void witness_arm_slot(uint8_t slot_index) {
  if (slot_index > WITNESS_N_SLOTS) {
    qtimer1_ch3_disable_compare();
    g_witness_current.slot_index = 0xFF;  // disarmed sentinel
    return;
  }

  const uint32_t target_vclock =
      witness_compute_target_vclock(g_witness_anchor_counter32, slot_index);
  const uint32_t target_dwt =
      witness_compute_target_dwt(g_witness_anchor_dwt,
                                 g_witness_anchor_dwt_cps,
                                 slot_index);

  // Store slot state atomically from the ISR's perspective.  ISR reads
  // slot_index last; we write it last.
  g_witness_current.target_vclock = target_vclock;
  g_witness_current.target_dwt    = target_dwt;
  g_witness_current.slot_index    = slot_index;

  // Program CH3 compare for low 16 bits.  The full 32-bit target_vclock
  // is stored for the ISR's sanity check; the hardware compare operates
  // on the low 16 bits only.
  qtimer1_ch3_program_compare((uint16_t)(target_vclock & 0xFFFF));
}

// CH3 witness ISR path — called from qtimer1_isr when witness mode is ON.
//
// IMPORTANT: CH3's compare hardware only compares the LOW 16 BITS of the
// counter against COMP1.  At 10 MHz, the counter wraps every ~6.55 ms
// (65536 ticks), so the compare fires roughly 150x per second — but
// only ONE of those fires is the real target (the one where the full
// 32-bit counter equals target_vclock).  The other ~149 are spurious
// wrap-matches with the same low-16 bits but wildly different high-16.
//
// We filter by reading the 32-bit counter at ISR entry and comparing
// against the full target_vclock.  Matches (within a small tolerance
// window) are the real fires; non-matches are ignored without rotating
// slots or incrementing the rejected counter.
//
// Rotation only happens on a real match.
//
static void witness_ch3_isr(uint32_t dwt_raw) {
  qtimer1_ch3_clear_compare_flag();
  g_witness_fires_total++;

  const uint8_t slot_index = g_witness_current.slot_index;
  if (slot_index > WITNESS_N_SLOTS) {
    // Spurious fire — no slot armed (disarmed sentinel = 0xFF).
    qtimer1_ch3_disable_compare();
    g_witness_fires_rejected++;
    return;
  }

  // Read the full 32-bit counter to distinguish real fires from wrap
  // aliases.  A real fire has counter32 very close to target_vclock
  // (within a few ticks of hardware latency).  An alias has counter32
  // offset by some multiple of 65536.
  const uint32_t counter32 = qtimer1_read_32_for_diag();
  const uint16_t ch3_cntr  = IMXRT_TMR1.CH[3].CNTR;
  const uint32_t target_vclock = g_witness_current.target_vclock;

  // Signed delta — positive if counter is past target (normal case),
  // negative if target hasn't arrived yet.  Wrap-alias fires will have
  // magnitude > ~32768 (because they're at least a half-wrap away).
  //
  // Real fire tolerance: within WITNESS_VCLOCK_MATCH_WINDOW ticks of
  // target.  One VCLOCK tick is 100 ns, so a 256-tick window is 25.6 µs
  // — plenty of margin for ISR entry latency without allowing aliases.
  const int32_t vclock_delta = (int32_t)(counter32 - target_vclock);
  constexpr int32_t WITNESS_VCLOCK_MATCH_WINDOW = 256;
  if (vclock_delta < -WITNESS_VCLOCK_MATCH_WINDOW ||
      vclock_delta >  WITNESS_VCLOCK_MATCH_WINDOW) {
    // Wrap alias — not our real fire.  Don't rotate, don't count as
    // rejected, just let the compare re-fire at the next wrap and we'll
    // check again.  Eventually (within ~6.5 ms at worst) we'll see the
    // real one.
    return;
  }

  // CH3 / CH0 phase lag measurement.  Both channels clock from the
  // same 10 MHz VCLOCK source (PCS=0), but CH3 was enabled by a
  // different register write than CH0 at init time.  Any phase
  // difference between them accumulated between those two enables
  // persists forever and is a pure observable here.
  //
  // Compare the low 16 bits of the cascade (CH0) counter against the
  // CH3 counter.  At this instant, lag = CH0_low16 - CH3.  Positive
  // lag means CH3 is behind CH0 by that many ticks.
  const int32_t lag_ticks =
      (int32_t)(int16_t)((uint16_t)(counter32 & 0xFFFF) - ch3_cntr);
  witness_welford_update(g_witness_lag_welford, lag_ticks);

  const uint32_t target_dwt = g_witness_current.target_dwt;
  const int32_t  residual_cycles = (int32_t)(dwt_raw - target_dwt);

  // ── Capture raw inputs and derived diagnostics ──
  //
  // Write the capture BEFORE the reject decision.  Anomalies are
  // exactly what we want in the buffer — the whole point is to
  // inspect what was fed into accept/reject.  Indexed by slot_index
  // directly (0..WITNESS_N_SLOTS), so each slot owns one entry and
  // is overwritten by next second's fire for the same slot.
  {
    witness_capture_t& cap = g_witness_captures[slot_index];
    cap.seq                = ++g_witness_capture_seq;
    cap.slot_index         = slot_index;
    cap.valid              = true;
    cap.anchor_counter32   = g_witness_anchor_counter32;
    cap.anchor_dwt         = g_witness_anchor_dwt;
    cap.anchor_dwt_cps     = g_witness_anchor_dwt_cps;
    cap.target_vclock      = target_vclock;
    cap.target_dwt         = target_dwt;
    cap.dwt_at_fire        = dwt_raw;
    cap.counter32_at_fire  = counter32;
    cap.ch3_cntr_at_fire   = ch3_cntr;
    cap.vclock_delta       = vclock_delta;
    cap.residual_cycles    = residual_cycles;
    cap.lag_ticks          = lag_ticks;

    witness_anomaly_maybe_capture(cap, g_witness_fires_total);
  }

  // Sanity: residual must be small.  Large magnitude (in either
  // direction) means something is wrong — likely a missed fire or
  // heavy preemption.  Reject from the main Welford.
  if (residual_cycles < -WITNESS_REJECT_CYCLES ||
      residual_cycles >  WITNESS_REJECT_CYCLES) {
    g_witness_fires_rejected++;
  } else {
    witness_welford_update(g_witness_welford, residual_cycles);
    witness_update_floor_from_captures();
  }

  // Self-rotate: arm next slot.  If this was the final slot, disable
  // CH3 until the next PPS edge re-arms slot 1.
  if (slot_index < WITNESS_N_SLOTS) {
    witness_arm_slot(slot_index + 1);
  } else {
    qtimer1_ch3_disable_compare();
    g_witness_current.slot_index = 0xFF;
  }
}

// ── Public API ──

void interrupt_witness_set_mode(interrupt_witness_mode_t mode) {
  if (mode == g_witness_mode) return;

  if (mode == interrupt_witness_mode_t::ON) {
    // Going ON: disable cadence CH3 arming.  The ISR dispatcher will
    // route CH3 to the witness path on the next fire.  We also disable
    // CH3 compare until alpha calls arm_first_slot on the next PPS.
    qtimer1_ch3_disable_compare();
    g_witness_current.slot_index = 0xFF;
    g_witness_mode = mode;
  } else {
    // Going OFF: disable CH3 and let the next PPS-edge rebootstrap
    // restore normal cadence.  Alpha is responsible for calling
    // interrupt_request_pps_rebootstrap() to trigger that.
    g_witness_mode = mode;
    qtimer1_ch3_disable_compare();
    g_witness_current.slot_index = 0xFF;
  }
}

interrupt_witness_mode_t interrupt_witness_get_mode(void) {
  return g_witness_mode;
}

void interrupt_witness_arm_first_slot(uint32_t anchor_counter32,
                                       uint32_t anchor_dwt,
                                       uint32_t dwt_cycles_per_second) {
  if (g_witness_mode != interrupt_witness_mode_t::ON) return;

  g_witness_anchor_counter32 = anchor_counter32;
  g_witness_anchor_dwt       = anchor_dwt;
  g_witness_anchor_dwt_cps   = dwt_cycles_per_second;

  witness_arm_slot(1);
}

interrupt_witness_stats_t interrupt_witness_stats(void) {
  interrupt_witness_stats_t s{};
  s.mode = g_witness_mode;
  s.n = g_witness_welford.n;
  s.mean_ns   = witness_cycles_to_ns(g_witness_welford.mean);
  const double stddev_cycles = witness_welford_stddev(g_witness_welford);
  s.stddev_ns = witness_cycles_to_ns(stddev_cycles);
  s.stderr_ns = (s.n >= 2)
      ? (s.stddev_ns / sqrt((double)s.n))
      : 0.0;
  s.min_ns        = (int64_t)witness_cycles_to_ns((double)g_witness_welford.min_val);
  s.max_ns        = (int64_t)witness_cycles_to_ns((double)g_witness_welford.max_val);
  s.fires_total    = g_witness_fires_total;
  s.fires_rejected = g_witness_fires_rejected;

  // Lag welford — units are VCLOCK ticks (100 ns each).
  s.lag_n              = g_witness_lag_welford.n;
  s.lag_mean_ticks     = g_witness_lag_welford.mean;
  s.lag_stddev_ticks   = witness_welford_stddev(g_witness_lag_welford);
  s.lag_min_ticks      = g_witness_lag_welford.min_val;
  s.lag_max_ticks      = g_witness_lag_welford.max_val;

  // GPIO-ISR-entry to counter-read delay — units are DWT cycles.
  s.gpio_counter_delay_n              = g_witness_gpio_counter_delay_welford.n;
  s.gpio_counter_delay_mean_cycles    = g_witness_gpio_counter_delay_welford.mean;
  s.gpio_counter_delay_stddev_cycles  = witness_welford_stddev(g_witness_gpio_counter_delay_welford);
  s.gpio_counter_delay_min_cycles     = g_witness_gpio_counter_delay_welford.min_val;
  s.gpio_counter_delay_max_cycles     = g_witness_gpio_counter_delay_welford.max_val;

  return s;
}

void interrupt_witness_reset_stats(void) {
  witness_welford_reset(g_witness_welford);
  witness_welford_reset(g_witness_lag_welford);
  witness_welford_reset(g_witness_gpio_counter_delay_welford);
  witness_captures_reset();
  witness_anomalies_reset();
  g_witness_fires_total    = 0;
  g_witness_fires_rejected = 0;
  g_witness_floor_cycles   = 0;
}

uint32_t interrupt_witness_floor_cycles(void) {
  return g_witness_floor_cycles;
}

double interrupt_witness_floor_ns(void) {
  return witness_cycles_to_ns((double)g_witness_floor_cycles);
}

// ============================================================================
// QTimer1 vector — dispatches CH2 (TimePop) and CH3 (VCLOCK cadence or witness)
// ============================================================================

static void qtimer1_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;  // FIRST instruction

  // ── CH2 (TimePop scheduler) ──
  //
  // TimePop drives the foreground (including transport RX/TX timers).
  //
  if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_clear_compare_flag();

    if (g_qtimer1_ch2_handler) {
      const uint32_t counter32 = qtimer1_read_32_for_diag();
      const int64_t gnss_ns = time_dwt_to_gnss_ns(dwt_raw);

      interrupt_event_t event{};
      event.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      event.provider = interrupt_provider_kind_t::QTIMER1;
      event.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      event.status   = interrupt_event_status_t::OK;
      event.dwt_at_event       = dwt_raw;
      event.gnss_ns_at_event   = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
      event.counter32_at_event = counter32;

      interrupt_capture_diag_t diag{};
      diag.enabled  = true;
      diag.provider = interrupt_provider_kind_t::QTIMER1;
      diag.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      diag.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      diag.dwt_at_event       = dwt_raw;
      diag.gnss_ns_at_event   = event.gnss_ns_at_event;
      diag.counter32_at_event = counter32;

      g_qtimer1_ch2_handler(event, diag);
    }
  }

  if (IMXRT_TMR1.CH[3].CSCTRL & TMR_CSCTRL_TCF1) {
    if (g_witness_mode == interrupt_witness_mode_t::ON) {
      witness_ch3_isr(dwt_raw);
    } else {
      vclock_cadence_isr(dwt_raw);
    }
  }
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
  const uint32_t dwt_after_counter_reads = ARM_DWT_CYCCNT;

  // Diagnostic: measure the peripheral-bus delay between ISR first
  // instruction and the counter read moment.  This cost is what makes
  // `counter32` lag `dwt_isr_entry_raw` in real time.  If we're chasing
  // a residual offset in the witness, we need to know this number.
  const int32_t counter_read_delay_cycles =
      (int32_t)(dwt_after_counter_reads - dwt_isr_entry_raw);
  witness_welford_update(g_witness_gpio_counter_delay_welford,
                         counter_read_delay_cycles);

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
  // Epoch tick offset (VCLOCK_EPOCH_TICK_OFFSET):
  //   The counter32 we read is ~1 tick past the true PPS edge due to
  //   peripheral-bus-read quantization (see the constant's definition
  //   for the full physical reasoning).  The VCLOCK lane's logical
  //   count and cadence compare target are established directly here
  //   from counter32 and ch3_now — without going through alpha — so
  //   the correction MUST be applied locally for the VCLOCK lane's
  //   tick identity to agree with alpha's epoch identity downstream.
  //
  //   Note: the snapshot itself is published RAW.  Alpha applies the
  //   same offset independently when installing its epoch, and
  //   downstream consumers (witness armer, etc.) read alpha's
  //   corrected state rather than re-applying the offset.  This code
  //   is the one place in process_interrupt that establishes a new
  //   tick identity without going through alpha, so it's the one
  //   place that has to apply the correction locally.
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
        (uint16_t)(ch3_now + (uint16_t)VCLOCK_INTERVAL_COUNTS
                   + (uint16_t)VCLOCK_EPOCH_TICK_OFFSET);
    g_vclock_lane.logical_count32_at_last_second =
        counter32 + VCLOCK_EPOCH_TICK_OFFSET;
    qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
  }

  // ── Snapshot publication (seqlock) ──
  //
  // counter32_at_edge is published RAW — the honest hardware reading
  // taken ~140 cycles after the PPS edge.  Downstream consumers that
  // need the PPS-corresponding counter identity (rather than the
  // bus-read-moment identity) apply VCLOCK_EPOCH_TICK_OFFSET
  // themselves.  Alpha's epoch install does this; the witness armer
  // then reads alpha's corrected epoch rather than the raw snapshot.
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

// ── QTimer1 CH0/CH1 cascade — VCLOCK-clocked 32-bit counter ──
//
// CH0 is the low 16 bits, clocked by the external 10 MHz VCLOCK signal
// on pin 10.  CH1 is cascaded above it, advanced by CH0's overflow.
// Together they form a free-running 32-bit count of VCLOCK ticks.
// Read via qtimer1_read_32_for_diag() with torn-read protection.
//
static void qtimer1_init_vclock_base(void) {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  *(portConfigRegister(10)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IMXRT_TMR1.CH[0].CTRL = 0;
  IMXRT_TMR1.CH[1].CTRL = 0;

  IMXRT_TMR1.CH[0].SCTRL  = 0;
  IMXRT_TMR1.CH[0].CSCTRL = 0;
  IMXRT_TMR1.CH[0].LOAD   = 0;
  IMXRT_TMR1.CH[0].CNTR   = 0;
  IMXRT_TMR1.CH[0].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  IMXRT_TMR1.CH[1].SCTRL  = 0;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].LOAD   = 0;
  IMXRT_TMR1.CH[1].CNTR   = 0;
  IMXRT_TMR1.CH[1].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[1].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[1].CTRL   = TMR_CTRL_CM(7);

  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].SCTRL &= ~(TMR_SCTRL_TOFIE | TMR_SCTRL_IEF | TMR_SCTRL_IEFIE);

  (void)IMXRT_TMR1.CH[0].CNTR;
  (void)IMXRT_TMR1.CH[1].HOLD;
}

// ── QTimer1 CH2 — TimePop scheduler compare channel ──
//
// CH2 runs from the same VCLOCK source as CH0/CH1 and is used by TimePop
// as its scheduling compare channel.  TimePop owns the compare register
// programming; process_interrupt only does the one-time mode/control init
// here and clears the TCF1 flag in the QTimer1 ISR before invoking
// TimePop's registered handler.
//
static void qtimer1_init_ch2_scheduler(void) {
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
}

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

  // ── QTimer1 synchronized channel start ──
  //
  // QTimer1 channels CH0, CH1 (cascaded high-16), CH2 (TimePop scheduler),
  // and CH3 (VCLOCK cadence) all count the same external VCLOCK source
  // (PCS=0, 10 MHz phase-locked to GNSS).  If we let each channel's CTRL
  // register enable counting via separate writes, the channels start on
  // different VCLOCK edges — producing a permanent 4-5 tick phase offset
  // that the CH3 witness has made visible.
  //
  // The TMR module has a single per-module ENBL register whose low 4 bits
  // gate counting for CH0..CH3 independently.  A channel counts only when
  // BOTH its ENBL bit is set AND CTRL.CM != 0.  By clearing ENBL first,
  // configuring all four channels fully (including CTRL.CM), and then
  // writing ENBL = 0xF in a single 16-bit store, all four channels start
  // counting on the same VCLOCK edge — zero phase offset by construction.
  //
  // ENBL default value is 0x1 (CH0 auto-enabled at reset).  We clobber it
  // to 0 before any CTRL.CM writes take effect.
  IMXRT_TMR1.ENBL = 0;

  qtimer1_init_vclock_base();       // CH0 + CH1 cascade: configured, not yet counting
  qtimer1_init_ch2_scheduler();     // CH2 scheduler: configured, not yet counting
  qtimer1_ch3_init_vclock_cadence();// CH3 cadence:   configured, not yet counting
  g_vclock_lane.initialized = true;

  // Zero all four counters immediately before the atomic enable.  ENBL=0
  // means none are counting, so these writes are pure state setup; order
  // does not matter.  When ENBL goes to 0xF, all four start from zero.
  IMXRT_TMR1.CH[0].CNTR = 0;
  IMXRT_TMR1.CH[1].CNTR = 0;
  IMXRT_TMR1.CH[2].CNTR = 0;
  IMXRT_TMR1.CH[3].CNTR = 0;

  // Atomic start.  Single 16-bit register write; all four ENBL bits
  // assert within one peripheral-clock cycle, and all four channels
  // begin counting on the same VCLOCK edge.
  IMXRT_TMR1.ENBL = 0x0F;

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

  interrupt_witness_reset_stats();
  interrupt_witness_set_mode(interrupt_witness_mode_t::ON);

  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
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

  // ── Witness section ──
  const interrupt_witness_stats_t ws = interrupt_witness_stats();
  p.add("witness_active", ws.mode == interrupt_witness_mode_t::ON);
  p.add("witness_n",              (uint32_t)ws.n);
  p.add("witness_mean_ns",        ws.mean_ns);
  p.add("witness_stddev_ns",      ws.stddev_ns);
  p.add("witness_stderr_ns",      ws.stderr_ns);
  p.add("witness_min_ns",         ws.min_ns);
  p.add("witness_max_ns",         ws.max_ns);
  p.add("witness_fires_total",    (uint32_t)ws.fires_total);
  p.add("witness_fires_rejected", (uint32_t)ws.fires_rejected);

  // CH3/CH0 phase lag — unit is VCLOCK ticks (100 ns each).
  p.add("witness_lag_n",            (uint32_t)ws.lag_n);
  p.add("witness_lag_mean_ticks",   ws.lag_mean_ticks);
  p.add("witness_lag_stddev_ticks", ws.lag_stddev_ticks);
  p.add("witness_lag_min_ticks",    ws.lag_min_ticks);
  p.add("witness_lag_max_ticks",    ws.lag_max_ticks);

  // GPIO-ISR-entry to counter-read delay — unit is DWT cycles.
  p.add("witness_gpio_counter_delay_n",            (uint32_t)ws.gpio_counter_delay_n);
  p.add("witness_gpio_counter_delay_mean_cycles",  ws.gpio_counter_delay_mean_cycles);
  p.add("witness_gpio_counter_delay_stddev_cycles", ws.gpio_counter_delay_stddev_cycles);
  p.add("witness_gpio_counter_delay_min_cycles",   ws.gpio_counter_delay_min_cycles);
  p.add("witness_gpio_counter_delay_max_cycles",   ws.gpio_counter_delay_max_cycles);

  p.add("witness_floor_cycles", interrupt_witness_floor_cycles());
  p.add("witness_floor_ns", interrupt_witness_floor_ns());

  return p;
}

static Payload __attribute__((unused)) cmd_witness_start(const Payload&) {
  interrupt_witness_reset_stats();
  interrupt_witness_set_mode(interrupt_witness_mode_t::ON);
  Payload p;
  p.add("witness_active", true);
  p.add("status", "WITNESS_STARTED");
  return p;
}

static Payload __attribute__((unused)) cmd_witness_stop(const Payload&) {
  interrupt_witness_set_mode(interrupt_witness_mode_t::OFF);
  // Alpha's next PPS edge will rebootstrap naturally — but request it
  // now to make sure cadence resumes on the very next edge rather than
  // drifting silently.
  interrupt_request_pps_rebootstrap();
  const interrupt_witness_stats_t ws = interrupt_witness_stats();
  Payload p;
  p.add("witness_active", false);
  p.add("witness_n",        (uint32_t)ws.n);
  p.add("witness_mean_ns",  ws.mean_ns);
  p.add("status", "WITNESS_STOPPED");
  return p;
}

static Payload cmd_witness_reset(const Payload&) {
  interrupt_witness_reset_stats();
  Payload p;
  p.add("status", "WITNESS_RESET");
  return p;
}

// Serialize a witness_capture_t into a Payload with all raw fields
// plus a handful of derived "prediction" fields that let the reader
// verify the arithmetic with a calculator.
//
// Shared between the per-slot capture dump and the anomaly dump so
// the two views agree on field names and interpretation.
static void witness_capture_to_payload(const witness_capture_t& cap,
                                       Payload& entry)
{
  entry.add("slot_index",       (uint32_t)cap.slot_index);
  entry.add("valid",            cap.valid);
  entry.add("seq",              cap.seq);

  // Anchors (PPS-edge-derived inputs to the prediction).
  entry.add("anchor_counter32", cap.anchor_counter32);
  entry.add("anchor_dwt",       cap.anchor_dwt);
  entry.add("anchor_dwt_cps",   cap.anchor_dwt_cps);

  // ── Prediction helpers (for calculator-friendly checks) ──
  //
  // dwt_cycle_prediction          — cps verbatim, labeled for intent:
  //                                 "predicted DWT cycles per second."
  // dwt_cycle_prediction_per_slot — cps / 10 with firmware rounding:
  //                                 "roughly how many DWT cycles per
  //                                  100 ms slot."  Watch for per-slot
  //                                  rounding drift up to ±1 cycle.
  //
  // Use these to eyeball dwt_at_fire - anchor_dwt against slot_index *
  // dwt_cycle_prediction_per_slot.
  entry.add("dwt_cycle_prediction",
            cap.anchor_dwt_cps);
  entry.add("dwt_cycle_prediction_per_slot",
            (uint32_t)(((uint64_t)cap.anchor_dwt_cps + 5ULL) / 10ULL));

  // Targets as programmed for this slot.
  entry.add("target_vclock",    cap.target_vclock);
  entry.add("target_dwt",       cap.target_dwt);

  // ── Formula integrity check ──
  //
  // Re-derive the targets from the captured anchors using the SAME
  // arithmetic witness_arm_slot used.  Under normal operation these
  // match target_vclock and target_dwt exactly.  A disagreement would
  // be the smoking gun for an anchor-update race between arming and
  // firing — the ISR captures g_witness_anchor_* at fire time; if they
  // differ from what the armer saw, re-deriving from the captured
  // anchors produces a different target than the one actually armed.
  //
  // Also publish deltas for convenience (what "each slot added" in
  // both clock domains).
  const uint32_t expected_target_dwt = (cap.slot_index <= WITNESS_N_SLOTS)
      ? witness_compute_target_dwt(cap.anchor_dwt,
                                   cap.anchor_dwt_cps,
                                   cap.slot_index)
      : 0u;
  const uint32_t expected_target_vclock = (cap.slot_index <= WITNESS_N_SLOTS)
      ? witness_compute_target_vclock(cap.anchor_counter32, cap.slot_index)
      : 0u;

  entry.add("expected_target_dwt",     expected_target_dwt);
  entry.add("expected_target_vclock",  expected_target_vclock);
  entry.add("target_dwt_agrees",
            cap.target_dwt    == expected_target_dwt);
  entry.add("target_vclock_agrees",
            cap.target_vclock == expected_target_vclock);

  // Actual delta from anchor — what the DWT counter ACTUALLY advanced
  // by between the anchor moment and this fire.  Compare against
  // expected_target_dwt - anchor_dwt to see if the fire landed where
  // predicted.
  entry.add("dwt_delta_at_fire",
            (uint32_t)(cap.dwt_at_fire - cap.anchor_dwt));
  entry.add("counter32_delta_at_fire",
            (uint32_t)(cap.counter32_at_fire - cap.anchor_counter32));

  // ISR-entry captures (this is what the fire actually saw).
  entry.add("dwt_at_fire",      cap.dwt_at_fire);
  entry.add("counter32_at_fire",cap.counter32_at_fire);
  entry.add("ch3_cntr_at_fire", (uint32_t)cap.ch3_cntr_at_fire);

  // Derived diagnostics (same arithmetic as the Welford path).
  entry.add("vclock_delta",     cap.vclock_delta);
  entry.add("residual_cycles",  cap.residual_cycles);
  entry.add("lag_ticks",        cap.lag_ticks);
}

// ── Dump the per-slot capture buffer AND the anomaly buffer ──
//
// Emits two arrays:
//
//   "witness_captures"  — last-second per-slot captures (always 9).
//                         Indexed by slot (1..WITNESS_N_SLOTS mapped to
//                         array indices 0..WITNESS_N_SLOTS-1).  Slots
//                         that have never been written since the last
//                         reset carry valid=false.  Slots from different
//                         seconds can coexist transiently if the dump
//                         is issued mid-second, so the `seq` field is
//                         provided for ordering (higher seq = more
//                         recent write across all slots).
//
//   "witness_anomalies" — sticky capture of anomalous fires (|residual|
//                         > WITNESS_ANOMALY_CYCLES).  Size up to
//                         WITNESS_ANOMALY_BUFFER_SIZE.  Oldest-wins:
//                         once full, later anomalies are counted but
//                         not captured.  Preserved until WITNESS_RESET
//                         or WITNESS_START.  Each entry adds a
//                         `fires_total_at_capture` field recording
//                         the cumulative fire count at the moment of
//                         capture, for run-time context.
//
//   "witness_anomaly_count_total" — monotonic count of anomalies seen
//                         since last reset, regardless of buffer
//                         capacity.  Tells you "N anomalies observed,
//                         first K preserved in witness_anomalies."
static Payload cmd_witness_dump(const Payload&) {
  // Per-slot capture array (indices 0..WITNESS_N_SLOTS, inclusive).
  PayloadArray captures_arr;
  for (uint8_t i = 0; i <= WITNESS_N_SLOTS; i++) {
    const witness_capture_t& cap = g_witness_captures[i];
    Payload entry;
    entry.add("array_index", (uint32_t)i);
    witness_capture_to_payload(cap, entry);
    captures_arr.add(entry);
  }

  // Anomaly array — only entries that were actually written.
  PayloadArray anomalies_arr;
  const uint8_t anomaly_count_captured = g_witness_anomaly_slot_next;
  for (uint8_t i = 0; i < anomaly_count_captured; i++) {
    const witness_anomaly_t& a = g_witness_anomalies[i];
    Payload entry;
    entry.add("buffer_index",           (uint32_t)i);
    entry.add("fires_total_at_capture", a.fires_total_at_capture);
    witness_capture_to_payload(a.cap, entry);
    anomalies_arr.add(entry);
  }

  Payload p;
  p.add("witness_active",             g_witness_mode == interrupt_witness_mode_t::ON);
  p.add("witness_anomaly_threshold_cycles", (uint32_t)WITNESS_ANOMALY_CYCLES);
  p.add("witness_anomaly_buffer_size",      (uint32_t)WITNESS_ANOMALY_BUFFER_SIZE);
  p.add("witness_anomaly_count_total",      g_witness_anomaly_count_total);
  p.add("witness_anomaly_count_captured",   (uint32_t)anomaly_count_captured);
  p.add_array("witness_captures",   captures_arr);
  p.add_array("witness_anomalies",  anomalies_arr);
  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT",        cmd_report         },
  { "WITNESS_RESET", cmd_witness_reset  },
  { "WITNESS_DUMP",  cmd_witness_dump   },
  { nullptr,         nullptr            }
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
    case interrupt_subscriber_kind_t::TIMEPOP:   return "TIMEPOP";
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
    case interrupt_lane_t::QTIMER1_CH2_COMP: return "QTIMER1_CH2_COMP";
    case interrupt_lane_t::QTIMER1_CH3_COMP: return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER3_CH2_COMP: return "QTIMER3_CH2_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}
