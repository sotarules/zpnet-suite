// ============================================================================
// process_interrupt.cpp — ISR custodian, scorched
// ============================================================================
//
// See process_interrupt.h for the contract.  This module is a fountain,
// not a reservoir: it captures _raw, latency-adjusts DWT, advances the
// three synthetic 32-bit counters by gear arithmetic, computes
// gnss_ns_at_edge for the dispatched event, and hands the event to the
// subscriber.
//
// The synthetic counters are file-static and have no read accessor.  The
// only way to observe a counter32 value is through a dispatched event.
// Alpha may zero all three via interrupt_synthetic_counters_zero().
//
// Lanes:
//   PPS_VCLOCK : GPIO edge.  Counter32 in event = current g_vclock_count32.
//   VCLOCK     : QTimer1 CH3, advances g_vclock_count32 by VCLOCK_INTERVAL_COUNTS
//                per cadence tick.  TICK fires every cadence; VCLOCK fires on
//                the 1000th cadence tick.
//   OCXO1      : QTimer3 CH2, advances g_ocxo1_count32 by OCXO_INTERVAL_COUNTS
//                per cadence tick.  OCXO1 fires on the 1000th.
//   OCXO2      : QTimer3 CH3, advances g_ocxo2_count32 by OCXO_INTERVAL_COUNTS
//                per cadence tick.  OCXO2 fires on the 1000th.
//   TIMEPOP    : reserved for future scheduled-fire scheme.  Currently unused.
//
// CH2 is unused in this build.  The compare is left disabled.  A future
// "scheduled subscription" mode may use CH2 to fire at a requested
// counter32, replacing TimePop's tick-driven heartbeat for sub-ms
// scheduling.  Until then, TimePop subscribes to TICK.
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

static constexpr int64_t  GNSS_NS_PER_SECOND = 1000000000LL;

// ============================================================================
// Latency adjusters — convert raw ISR-entry DWT to event coordinates
// ============================================================================
//
// The ONLY producers of event-coordinate DWT, and the ONLY place in the
// codebase that applies hardware latency math.  Called exactly once per
// ISR on the first-instruction _raw capture.

static inline uint32_t pps_vclock_dwt_from_pps_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw + (uint32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;
}

static inline uint32_t qtimer_event_dwt_from_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw - (QTIMER_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY);
}

// ============================================================================
// Synthetic counters — file-static, no read accessors
// ============================================================================

static volatile uint32_t g_vclock_count32 = 0;
static volatile uint32_t g_ocxo1_count32  = 0;
static volatile uint32_t g_ocxo2_count32  = 0;

void interrupt_synthetic_counters_zero(void) {
  // Single-writer model: alpha is the only foreground caller, ISRs read
  // them at fire time.  Plain stores are fine on Cortex-M7 for aligned
  // 32-bit volatiles.
  g_vclock_count32 = 0;
  g_ocxo1_count32  = 0;
  g_ocxo2_count32  = 0;
}

// ============================================================================
// Subscription registry
// ============================================================================

struct subscriber_t {
  interrupt_subscription_t sub {};
  bool     active = false;
  uint32_t sequence = 0;
};

static subscriber_t g_pps_vclock;
static subscriber_t g_vclock;
static subscriber_t g_ocxo1;
static subscriber_t g_ocxo2;
static subscriber_t g_tick;

static subscriber_t* runtime_for(interrupt_subscriber_kind_t k) {
  switch (k) {
    case interrupt_subscriber_kind_t::PPS_VCLOCK: return &g_pps_vclock;
    case interrupt_subscriber_kind_t::VCLOCK:     return &g_vclock;
    case interrupt_subscriber_kind_t::OCXO1:      return &g_ocxo1;
    case interrupt_subscriber_kind_t::OCXO2:      return &g_ocxo2;
    case interrupt_subscriber_kind_t::TICK:       return &g_tick;
    default:                                      return nullptr;
  }
}

bool interrupt_subscribe(const interrupt_subscription_t& sub) {
  subscriber_t* rt = runtime_for(sub.kind);
  if (!rt || !sub.on_event) return false;
  rt->sub = sub;
  return true;
}

bool interrupt_start(interrupt_subscriber_kind_t kind) {
  subscriber_t* rt = runtime_for(kind);
  if (!rt) return false;
  rt->active = true;
  return true;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  subscriber_t* rt = runtime_for(kind);
  if (!rt) return false;
  rt->active = false;
  return true;
}

// ============================================================================
// Deferred dispatch — IRQ → foreground via TimePop ASAP
// ============================================================================
//
// Per-kind ring of size 1 (overwrite-on-stale) plus an arming TimePop
// asap call.  Indexed by kind enum.  Single-slot is fine for 1 Hz events
// at sub-ms foreground latency; for TICK at 1 kHz with foreground latency
// near 1 ms, we accept that very rare overruns may overwrite (they would
// indicate the loop is starving, which is its own diagnostic).

struct deferred_slot_t {
  volatile bool     pending = false;
  interrupt_event_t event {};
};

static deferred_slot_t g_deferred[7];   // sized to enum count

static deferred_slot_t* deferred_for(interrupt_subscriber_kind_t k) {
  const uint8_t i = (uint8_t)k;
  if (i == 0 || i >= 7) return nullptr;
  return &g_deferred[i];
}

static const char* deferred_timer_name(interrupt_subscriber_kind_t k) {
  switch (k) {
    case interrupt_subscriber_kind_t::PPS_VCLOCK: return "PPS_VCLOCK_DISPATCH";
    case interrupt_subscriber_kind_t::VCLOCK:     return "VCLOCK_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO1:      return "OCXO1_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO2:      return "OCXO2_DISPATCH";
    case interrupt_subscriber_kind_t::TICK:       return "TICK_DISPATCH";
    default:                                      return "DISPATCH";
  }
}

static void deferred_trampoline(timepop_ctx_t*, timepop_diag_t*, void* ud) {
  auto* slot = static_cast<deferred_slot_t*>(ud);
  if (!slot || !slot->pending) return;
  const interrupt_event_t e = slot->event;
  slot->pending = false;

  subscriber_t* rt = runtime_for(e.kind);
  if (!rt || !rt->sub.on_event) return;
  rt->sub.on_event(e, rt->sub.user_data);
}

static void dispatch_deferred(subscriber_t& rt,
                              uint32_t dwt_at_edge,
                              uint32_t counter32_at_edge,
                              int64_t  gnss_ns_at_edge) {
  if (!rt.active || !rt.sub.on_event) return;

  rt.sequence++;

  deferred_slot_t* slot = deferred_for(rt.sub.kind);
  if (!slot) return;

  slot->event.kind              = rt.sub.kind;
  slot->event.gnss_ns_at_edge   = gnss_ns_at_edge;
  slot->event.dwt_at_edge       = dwt_at_edge;
  slot->event.counter32_at_edge = counter32_at_edge;
  slot->event.sequence          = rt.sequence;
  slot->pending                 = true;

  timepop_arm_asap(deferred_trampoline, slot, deferred_timer_name(rt.sub.kind));
}

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
// Per-lane cadence state
// ============================================================================

struct cadence_lane_t {
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t tick_mod_1000 = 0;
};

static cadence_lane_t g_vclock_lane;
static cadence_lane_t g_ocxo1_lane;
static cadence_lane_t g_ocxo2_lane;

// ============================================================================
// PPS rebootstrap
// ============================================================================

static volatile bool g_pps_rebootstrap_pending = false;

void interrupt_request_pps_rebootstrap(void) { g_pps_rebootstrap_pending = true; }
bool interrupt_pps_rebootstrap_pending(void) { return g_pps_rebootstrap_pending; }

// ============================================================================
// VCLOCK CH3 cadence — 1 kHz, advances synthetic counter every tick,
//                      fires TICK every tick, fires VCLOCK every 1000th
// ============================================================================

static void vclock_lane_arm_bootstrap(cadence_lane_t& lane) {
  lane.phase_bootstrapped = true;
  lane.tick_mod_1000 = 0;
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(lane.compare_target);
}

static void vclock_lane_advance_compare(cadence_lane_t& lane) {
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(lane.compare_target);
}

static void vclock_cadence_isr(uint32_t isr_entry_dwt_raw) {
  qtimer1_ch3_clear_compare_flag();
  if (!g_vclock_lane.active) return;

  if (!g_vclock_lane.phase_bootstrapped) {
    vclock_lane_arm_bootstrap(g_vclock_lane);
    return;
  }
  vclock_lane_advance_compare(g_vclock_lane);

  // Advance the synthetic VCLOCK counter by one cadence step.
  g_vclock_count32 += VCLOCK_INTERVAL_COUNTS;

  // Fire the TICK subscriber on every cadence step (1 ms).
  {
    const uint32_t dwt_at_edge = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
    const int64_t  gnss_ns     = time_dwt_to_gnss_ns(dwt_at_edge);
    dispatch_deferred(g_tick, dwt_at_edge, g_vclock_count32, gnss_ns);
  }

  if (++g_vclock_lane.tick_mod_1000 < TICKS_PER_SECOND_EVENT) return;
  g_vclock_lane.tick_mod_1000 = 0;

  // One-second boundary — fire VCLOCK subscriber.
  const uint32_t dwt_at_edge = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  const int64_t  gnss_ns     = time_dwt_to_gnss_ns(dwt_at_edge);
  dispatch_deferred(g_vclock, dwt_at_edge, g_vclock_count32, gnss_ns);
}

// ============================================================================
// OCXO CH2/CH3 cadence — 1 kHz, advance synthetic counter, fire on 1000th
// ============================================================================

static void ocxo_lane_arm_bootstrap(cadence_lane_t& lane, uint8_t ch) {
  lane.phase_bootstrapped = true;
  lane.tick_mod_1000 = 0;
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(ch, lane.compare_target);
}

static void ocxo_lane_advance_compare(cadence_lane_t& lane, uint8_t ch) {
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(ch, lane.compare_target);
}

static void ocxo_cadence_isr(cadence_lane_t& lane,
                             subscriber_t& rt,
                             volatile uint32_t& synthetic_counter,
                             uint8_t ch,
                             uint32_t isr_entry_dwt_raw) {
  qtimer3_clear_compare_flag(ch);
  if (!lane.active) return;

  if (!lane.phase_bootstrapped) {
    ocxo_lane_arm_bootstrap(lane, ch);
    return;
  }
  ocxo_lane_advance_compare(lane, ch);

  // Advance this OCXO's synthetic counter by one cadence step.
  synthetic_counter += OCXO_INTERVAL_COUNTS;

  if (++lane.tick_mod_1000 < TICKS_PER_SECOND_EVENT) return;
  lane.tick_mod_1000 = 0;

  const uint32_t dwt_at_edge = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  const int64_t  gnss_ns     = time_dwt_to_gnss_ns(dwt_at_edge);
  dispatch_deferred(rt, dwt_at_edge, synthetic_counter, gnss_ns);
}

void process_interrupt_qtimer3_ch2_irq(uint32_t isr_entry_dwt_raw) {
  ocxo_cadence_isr(g_ocxo1_lane, g_ocxo1, g_ocxo1_count32, 2, isr_entry_dwt_raw);
}

void process_interrupt_qtimer3_ch3_irq(uint32_t isr_entry_dwt_raw) {
  ocxo_cadence_isr(g_ocxo2_lane, g_ocxo2, g_ocxo2_count32, 3, isr_entry_dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  if (IMXRT_TMR3.CH[2].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch2_irq(isr_entry_dwt_raw);
  if (IMXRT_TMR3.CH[3].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch3_irq(isr_entry_dwt_raw);
}

// ============================================================================
// QTimer1 vector — dispatches CH3 (VCLOCK cadence).  CH2 is unused.
// ============================================================================

static void qtimer1_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.

  if (IMXRT_TMR1.CH[3].CSCTRL & TMR_CSCTRL_TCF1) {
    vclock_cadence_isr(isr_entry_dwt_raw);
  }
}

// ============================================================================
// PPS GPIO ISR — VCLOCK epoch authority
// ============================================================================
//
// Captures _raw, computes PPS_VCLOCK event-coordinate DWT, advances the
// PPS_VCLOCK sequence, dispatches.  On rebootstrap, phase-locks CH3 to the
// PPS moment and zeroes the sequence (so gnss_ns restarts at 0).
//
// counter32_at_edge is the synthetic VCLOCK counter at the moment of the
// PPS GPIO ISR, snapped to the most recent cadence-tick advance (≤ 1 ms
// stale).  This is the same coordinate system as the cadence ISR's
// counter32 — uniform tick-axis truth, never reading hardware.

void process_interrupt_gpio6789_irq(uint32_t isr_entry_dwt_raw) {
  const uint32_t dwt_at_edge = pps_vclock_dwt_from_pps_isr_entry_raw(isr_entry_dwt_raw);

  if (g_pps_rebootstrap_pending) {
    g_pps_rebootstrap_pending = false;
    g_vclock_lane.phase_bootstrapped = true;
    g_vclock_lane.tick_mod_1000 = 0;
    // Phase-lock: read CH3 counter and program compare one interval out.
    // (This read is hardware-programming-immediate — not propagated as an
    // event-coordinate value.  It dies in this stack frame.)
    const uint16_t ch3_now = IMXRT_TMR1.CH[3].CNTR;
    g_vclock_lane.compare_target = (uint16_t)(ch3_now + (uint16_t)VCLOCK_INTERVAL_COUNTS);
    qtimer1_ch3_program_compare(g_vclock_lane.compare_target);

    // Reset PPS_VCLOCK sequence so this edge becomes gnss_ns = 0.
    g_pps_vclock.sequence = 0;
  }

  // PPS_VCLOCK gnss_ns_at_edge is sequence × 1e9.  Pre-increment via
  // dispatch_deferred so the first dispatched edge is gnss_ns = 0,
  // second is 1e9, etc.
  const int64_t gnss_ns = (int64_t)g_pps_vclock.sequence * GNSS_NS_PER_SECOND;
  dispatch_deferred(g_pps_vclock, dwt_at_edge, g_vclock_count32, gnss_ns);
}

static void pps_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(isr_entry_dwt_raw);
}

// ============================================================================
// Compatibility shim
// ============================================================================

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {}

// ============================================================================
// Hardware init
// ============================================================================

static bool g_interrupt_hw_ready      = false;
static bool g_interrupt_runtime_ready = false;
static bool g_interrupt_irqs_enabled  = false;

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

static void qtimer1_ch2_init_disabled(void) {
  // CH2 is unused — leave it disabled.  Initialize the channel state to a
  // known disabled configuration so it can't accidentally fire.
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = 0;
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

  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
  *(portConfigRegister(OCXO1_PIN)) = 1;
  *(portConfigRegister(OCXO2_PIN)) = 1;
  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

  IMXRT_TMR3.CH[2].CTRL = 0; IMXRT_TMR3.CH[2].SCTRL = 0;
  IMXRT_TMR3.CH[2].CSCTRL = 0; IMXRT_TMR3.CH[2].LOAD = 0;
  IMXRT_TMR3.CH[2].CNTR = 0; IMXRT_TMR3.CH[2].COMP1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(2);
  qtimer3_disable_compare(2);

  IMXRT_TMR3.CH[3].CTRL = 0; IMXRT_TMR3.CH[3].SCTRL = 0;
  IMXRT_TMR3.CH[3].CSCTRL = 0; IMXRT_TMR3.CH[3].LOAD = 0;
  IMXRT_TMR3.CH[3].CNTR = 0; IMXRT_TMR3.CH[3].COMP1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(3);
  qtimer3_disable_compare(3);

  // QTimer1 synchronized channel start: ENBL=0 first, configure CH0/CH1/CH2/CH3
  // fully, then a single ENBL=0xF write starts all four on the same VCLOCK
  // edge — zero phase offset by construction.
  IMXRT_TMR1.ENBL = 0;
  qtimer1_init_vclock_base();
  qtimer1_ch2_init_disabled();
  qtimer1_ch3_init_vclock_cadence();
  IMXRT_TMR1.CH[0].CNTR = 0;
  IMXRT_TMR1.CH[1].CNTR = 0;
  IMXRT_TMR1.CH[2].CNTR = 0;
  IMXRT_TMR1.CH[3].CNTR = 0;
  IMXRT_TMR1.ENBL = 0x0F;

  g_interrupt_hw_ready = true;
}

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;

  // NOTE: subscriber registry (g_pps_vclock, g_vclock, g_ocxo1, g_ocxo2,
  // g_tick) and the deferred-dispatch slots are NOT zeroed here.  They
  // are BSS-zero by program-startup contract.  Earlier boot phases may
  // have already installed subscriptions (e.g. timepop_init subscribing
  // to TICK before this function runs); re-zeroing would silently wipe
  // them.

  g_vclock_lane = cadence_lane_t{};
  g_ocxo1_lane  = cadence_lane_t{};
  g_ocxo2_lane  = cadence_lane_t{};

  g_pps_rebootstrap_pending = false;

  interrupt_synthetic_counters_zero();

  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 0);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  // Activate cadence lanes and arm CH3 compare at the next tick.
  g_vclock_lane.active = true;
  qtimer1_ch3_program_compare((uint16_t)(g_vclock_lane.compare_target + 1));

  g_ocxo1_lane.active = true;
  g_ocxo1_lane.phase_bootstrapped = false;
  qtimer3_program_compare(2, (uint16_t)(g_ocxo1_lane.compare_target + 1));

  g_ocxo2_lane.active = true;
  g_ocxo2_lane.phase_bootstrapped = false;
  qtimer3_program_compare(3, (uint16_t)(g_ocxo2_lane.compare_target + 1));

  g_interrupt_irqs_enabled = true;
}

// ============================================================================
// REPORT command — minimal status surface
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);

  p.add("pps_vclock_sequence", g_pps_vclock.sequence);
  p.add("vclock_sequence",     g_vclock.sequence);
  p.add("ocxo1_sequence",      g_ocxo1.sequence);
  p.add("ocxo2_sequence",      g_ocxo2.sequence);
  p.add("tick_sequence",       g_tick.sequence);

  p.add("pps_vclock_active", g_pps_vclock.active);
  p.add("vclock_active",     g_vclock.active);
  p.add("ocxo1_active",      g_ocxo1.active);
  p.add("ocxo2_active",      g_ocxo2.active);
  p.add("tick_active",       g_tick.active);

  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr    },
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id    = "INTERRUPT",
  .commands      = INTERRUPT_COMMANDS,
  .subscriptions = nullptr,
};

void process_interrupt_register(void) {
  process_register("INTERRUPT", &INTERRUPT_PROCESS);
}

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::PPS_VCLOCK: return "PPS_VCLOCK";
    case interrupt_subscriber_kind_t::VCLOCK:     return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1:      return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:      return "OCXO2";
    case interrupt_subscriber_kind_t::TICK:       return "TICK";
    case interrupt_subscriber_kind_t::TIMEPOP:    return "TIMEPOP";
    default:                                      return "NONE";
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