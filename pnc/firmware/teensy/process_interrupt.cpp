// ============================================================================
// process_interrupt.cpp - Timing-event custodian, sole owner of QTimer hardware
// ============================================================================
//
// process_interrupt is the hardware boundary for Teensy timing.  It owns
// QTimer1, QTimer3, the PPS GPIO ISR, all first-instruction DWT captures,
// and the precision schedule-fire lane used by TimePop.
//
// This module does not answer the casual foreground question "what time is
// it right now?"  It authors timing EVENTS:
//
//   - capture _raw DWT at ISR entry,
//   - convert _raw immediately to event-coordinate DWT,
//   - advance the appropriate event-ledger synthetic counter,
//   - compute GNSS ns at the event edge,
//   - dispatch one small interrupt_event_t to the subscriber.
//
// Synthetic counter doctrine:
//   g_vclock_count32, g_ocxo1_count32, and g_ocxo2_count32 are event
//   ledgers, not public live clocks.  They are exact at authored cadence
//   events.  They are not read from foreground and have no accessors.
//
// Lanes:
//   PPS_VCLOCK     : Canonical VCLOCK-selected PPS epoch.  The physical
//                    PPS GPIO edge is a witness/selector.
//   VCLOCK         : QTimer1 CH3 cadence.  Advances g_vclock_count32 by
//                    VCLOCK_INTERVAL_COUNTS per cadence tick; emits TICK
//                    every cadence and VCLOCK every 1000 cadences.
//   OCXO1          : QTimer3 CH2 cadence.  Advances g_ocxo1_count32 by
//                    OCXO_INTERVAL_COUNTS; emits OCXO1 every 1000 cadences.
//   OCXO2          : QTimer3 CH3 cadence.  Advances g_ocxo2_count32 by
//                    OCXO_INTERVAL_COUNTS; emits OCXO2 every 1000 cadences.
//   SCHEDULE-FIRE  : QTimer1 CH2 compare.  Direct ISR callback lane used
//                    by TimePop's explicit precision/SpinDry mode only.
//
// NVIC priority order:
//   PPS GPIO   priority  0  (sovereign)
//   QTimer1   priority 16  (VCLOCK cadence + schedule fire)
//   QTimer3   priority 32  (OCXO cadences)
//
// TimePop relationship:
//   process_interrupt provides TimePop with two very different services:
//     1. TICK events from CH3 for stable foreground scheduling.
//     2. CH2 schedule-fire for explicit precision/SpinDry work.
//
//   Conversely, process_interrupt uses TimePop ASAP to defer subscriber
//   callbacks out of ISR context.  This mutual dependency is intentional;
//   it is safe only because ordinary recurring services stay on TICK mode
//   and do not flood TimePop's ASAP queue with schedule-fire deliveries.
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

// NVIC priorities (lower number = higher priority).
static constexpr uint32_t NVIC_PRIO_PPS_GPIO = 0;
static constexpr uint32_t NVIC_PRIO_QTIMER1  = 16;
static constexpr uint32_t NVIC_PRIO_QTIMER3  = 32;

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
// Synthetic counters - event ledgers, file-static, no read accessors
// ============================================================================
//
// These are authored coordinates.  They are advanced only by their cadence
// ISR, and observed only through interrupt_event_t or the schedule-fire
// callback.  Foreground code must not treat them as live free-running
// counters.

static volatile uint32_t g_vclock_count32 = 0;
static volatile uint32_t g_ocxo1_count32  = 0;
static volatile uint32_t g_ocxo2_count32  = 0;

// Last event-authored VCLOCK-domain coordinate observed by any dispatch
// path.  This is a fallback/diagnostic reference, not a live clock.  It is
// invalidated on synthetic-counter zero so old-timeline coordinates cannot
// leak into a newly installed epoch.
static volatile uint32_t g_last_known_counter32       = 0;
static volatile bool     g_last_known_counter32_valid = false;

static inline void clear_last_known_counter32(void) {
  g_last_known_counter32       = 0;
  g_last_known_counter32_valid = false;
}

static inline void note_vclock_counter32(uint32_t c32) {
  g_last_known_counter32       = c32;
  g_last_known_counter32_valid = true;
}

void interrupt_synthetic_counters_zero(void) {
  // Single-writer model: alpha is the only foreground caller, ISRs read
  // the counters at fire time.  Plain aligned 32-bit stores are fine on
  // Cortex-M7.  Invalidate last-known because it belongs to the discarded
  // counter timeline.
  g_vclock_count32 = 0;
  g_ocxo1_count32  = 0;
  g_ocxo2_count32  = 0;
  clear_last_known_counter32();
}

uint32_t interrupt_last_known_counter32(void) {
  return g_last_known_counter32;
}

bool interrupt_last_known_counter32_valid(void) {
  return g_last_known_counter32_valid;
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

// Sized to enum count (NONE + PPS_VCLOCK + VCLOCK + OCXO1 + OCXO2 + TICK = 6).
static deferred_slot_t g_deferred[6];

static deferred_slot_t* deferred_for(interrupt_subscriber_kind_t k) {
  const uint8_t i = (uint8_t)k;
  if (i == 0 || i >= 6) return nullptr;
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

static inline void qtimer1_ch2_clear_compare_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}
static inline void qtimer1_ch2_program_compare(uint16_t target_low16) {
  qtimer1_ch2_clear_compare_flag();
  IMXRT_TMR1.CH[2].COMP1  = target_low16;
  IMXRT_TMR1.CH[2].CMPLD1 = target_low16;
  qtimer1_ch2_clear_compare_flag();
  IMXRT_TMR1.CH[2].CSCTRL |= TMR_CSCTRL_TCF1EN;
}
static inline void qtimer1_ch2_disable_compare(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer1_ch2_clear_compare_flag();
}

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
// PPS -> canonical VCLOCK epoch calibration witness
// ============================================================================
//
// PPS GPIO is treated as a physical witness/selector.  The canonical
// PPS_VCLOCK epoch published to Alpha is not the raw GPIO ISR entry time;
// it is the empirically selected VCLOCK-domain epoch:
//
//   canonical_vclock_epoch_dwt = raw_pps_isr_entry_dwt
//                              + CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES
//
// The fields below continuously re-measure that offset as implemented in
// this firmware path and expose it in REPORT.  This is a calibration
// witness: it verifies that the code path is applying the configured
// offset every PPS and makes future changes immediately visible.

static volatile uint32_t g_pps_raw_isr_entry_dwt_last = 0;
static volatile uint32_t g_pps_canonical_epoch_dwt_last = 0;
static volatile int32_t  g_pps_canonical_offset_cycles_last = 0;
static volatile int32_t  g_pps_canonical_offset_cycles_min = 2147483647;
static volatile int32_t  g_pps_canonical_offset_cycles_max = -2147483647;
static volatile uint32_t g_pps_canonical_offset_samples = 0;
static volatile uint32_t g_pps_canonical_offset_mismatches = 0;

static inline void note_pps_canonical_epoch_calibration(uint32_t raw_pps_isr_entry_dwt,
                                                        uint32_t canonical_epoch_dwt) {
  const int32_t observed = (int32_t)(canonical_epoch_dwt - raw_pps_isr_entry_dwt);

  g_pps_raw_isr_entry_dwt_last = raw_pps_isr_entry_dwt;
  g_pps_canonical_epoch_dwt_last = canonical_epoch_dwt;
  g_pps_canonical_offset_cycles_last = observed;
  if (observed < g_pps_canonical_offset_cycles_min) g_pps_canonical_offset_cycles_min = observed;
  if (observed > g_pps_canonical_offset_cycles_max) g_pps_canonical_offset_cycles_max = observed;
  g_pps_canonical_offset_samples++;

  if (observed != (int32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES) {
    g_pps_canonical_offset_mismatches++;
  }
}

// ============================================================================
// Schedule-fire (CH2) - TimePop precision / SpinDry actuator
// ============================================================================
//
// CH2 is not a general foreground scheduler.  Normal services use the
// TICK lane.  CH2 is an actuator for explicit precision work.
//
// Public input is a target synthetic VCLOCK counter32 coordinate.  CH2 is
// only a 16-bit compare, so the implementation programs the low-word fire
// implied by the target and validates the full 32-bit target at interrupt
// time by projecting the current VCLOCK coordinate from alpha's PPS_VCLOCK
// bridge and event-coordinate DWT.
//
// Consequence: long-horizon precision targets are possible, but they cost
// one premature CH2 ISR per 16-bit wrap (~6.55 ms at 10 MHz) until the
// high word matches.  This is acceptable for sparse precision witnesses
// and SpinDry work; it is intentionally not used for recurring foreground
// services.

static volatile interrupt_schedule_fire_isr_fn g_schedule_callback = nullptr;
static void* volatile                          g_schedule_user_data = nullptr;
static volatile bool     g_schedule_armed     = false;
static volatile uint32_t g_schedule_target32  = 0;
static volatile uint16_t g_schedule_target16  = 0;

static volatile uint32_t g_schedule_armed_total        = 0;
static volatile uint32_t g_schedule_dispatched_total   = 0;
static volatile uint32_t g_schedule_late_arm_total     = 0;
static volatile uint32_t g_schedule_late_fire_total    = 0;
static volatile uint32_t g_schedule_cancelled_total    = 0;
static volatile uint32_t g_schedule_premature_wraps    = 0;
static volatile uint32_t g_schedule_projection_failures = 0;

static constexpr int32_t SCHEDULE_MIN_DELTA_TICKS = -16;  // ~1.6 us grace
static constexpr int32_t SCHEDULE_MATCH_TOLERANCE_TICKS = 16;

static bool project_vclock_counter32_from_dwt(uint32_t dwt_at_edge,
                                              uint32_t& out_counter32) {
  const alpha_pps_vclock_slot_t a = alpha_pps_vclock_load();
  if (a.sequence == 0) return false;

  const uint32_t cps = alpha_dwt_cycles_per_second();
  if (cps == 0) return false;

  const uint32_t dwt_elapsed = dwt_at_edge - a.dwt_at_edge;
  const uint64_t ticks_elapsed =
      ((uint64_t)dwt_elapsed * 10000000ULL + (uint64_t)cps / 2) /
      (uint64_t)cps;

  if (ticks_elapsed > 0xFFFFFFFFULL) return false;
  out_counter32 = a.counter32_at_edge + (uint32_t)ticks_elapsed;
  return true;
}

static bool project_vclock_counter32_now(uint32_t& out_counter32) {
  if (project_vclock_counter32_from_dwt(ARM_DWT_CYCCNT, out_counter32)) {
    return true;
  }
  if (g_last_known_counter32_valid) {
    out_counter32 = g_last_known_counter32;
    return true;
  }
  return false;
}

void interrupt_schedule_register(interrupt_schedule_fire_isr_fn callback,
                                 void* user_data) {
  // Single-writer foreground registration.  No critical section needed -
  // callback pointer is installed atomically and the CH2 ISR reads it
  // through a volatile.
  g_schedule_user_data = user_data;
  g_schedule_callback  = callback;
}

bool interrupt_schedule_fire_at(uint32_t target_counter32) {
  if (!g_schedule_callback) return false;

  uint32_t primask;
  __asm__ volatile ("MRS %0, primask" : "=r" (primask));
  __asm__ volatile ("CPSID i" ::: "memory");

  uint32_t now_c32 = 0;
  if (!project_vclock_counter32_now(now_c32)) {
    g_schedule_projection_failures++;
    __asm__ volatile ("MSR primask, %0" :: "r" (primask) : "memory");
    return false;
  }

  const int32_t delta = (int32_t)(target_counter32 - now_c32);
  if (delta < SCHEDULE_MIN_DELTA_TICKS) {
    g_schedule_late_arm_total++;
    __asm__ volatile ("MSR primask, %0" :: "r" (primask) : "memory");
    return false;
  }

  // CH2.CNTR is live hardware, sampled only for immediate programming.
  // The delta is computed from the projected live synthetic coordinate,
  // not the stepped event-ledger g_vclock_count32.  This avoids mixing a
  // stale cadence ledger with a live 16-bit hardware counter.
  const uint16_t ch2_now = IMXRT_TMR1.CH[2].CNTR;
  const uint16_t ch2_target = (uint16_t)((uint32_t)ch2_now + (uint32_t)delta);

  g_schedule_target32 = target_counter32;
  g_schedule_target16 = ch2_target;
  g_schedule_armed    = true;
  g_schedule_armed_total++;

  qtimer1_ch2_program_compare(ch2_target);

  __asm__ volatile ("MSR primask, %0" :: "r" (primask) : "memory");
  return true;
}

void interrupt_schedule_fire_cancel(void) {
  uint32_t primask;
  __asm__ volatile ("MRS %0, primask" : "=r" (primask));
  __asm__ volatile ("CPSID i" ::: "memory");

  if (g_schedule_armed) {
    g_schedule_cancelled_total++;
  }
  g_schedule_armed = false;
  qtimer1_ch2_disable_compare();

  __asm__ volatile ("MSR primask, %0" :: "r" (primask) : "memory");
}

bool interrupt_schedule_fire_armed(void) {
  return g_schedule_armed;
}

uint32_t interrupt_schedule_fires_armed_total     (void) { return g_schedule_armed_total; }
uint32_t interrupt_schedule_fires_dispatched_total(void) { return g_schedule_dispatched_total; }
uint32_t interrupt_schedule_fires_late_arm_total  (void) { return g_schedule_late_arm_total; }
uint32_t interrupt_schedule_fires_cancelled_total (void) { return g_schedule_cancelled_total; }

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
  note_vclock_counter32(g_vclock_count32);

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
// QTimer1 CH2 - schedule-fire ISR
// ============================================================================
//
// CH2 fires whenever its 16-bit compare matches.  A low-word match is only
// a candidate fire.  We project the current 32-bit VCLOCK coordinate from
// event-coordinate DWT and alpha's bridge, then accept the fire only when
// the projected coordinate is within a small tolerance of g_schedule_target32.
// Otherwise the low-word match was premature; the compare remains armed and
// will fire again on the next 16-bit wrap.

void process_interrupt_qtimer1_ch2_irq(uint32_t isr_entry_dwt_raw) {
  qtimer1_ch2_clear_compare_flag();

  if (!g_schedule_armed) {
    qtimer1_ch2_disable_compare();
    return;
  }

  const uint32_t dwt_at_edge = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);

  uint32_t current_c32 = 0;
  if (!project_vclock_counter32_from_dwt(dwt_at_edge, current_c32)) {
    g_schedule_projection_failures++;
    g_schedule_armed = false;
    qtimer1_ch2_disable_compare();
    return;
  }

  const int32_t ticks_to_target = (int32_t)(g_schedule_target32 - current_c32);

  if (ticks_to_target > SCHEDULE_MATCH_TOLERANCE_TICKS) {
    // Correct low word, wrong high word.  Leave CH2 armed for the next
    // 16-bit pass at the same compare value.
    g_schedule_premature_wraps++;
    return;
  }

  if (ticks_to_target < -SCHEDULE_MATCH_TOLERANCE_TICKS) {
    // We are past the intended full 32-bit target.  Disarm; firing now
    // would turn a precision miss into a false precision event.
    g_schedule_late_fire_total++;
    g_schedule_armed = false;
    qtimer1_ch2_disable_compare();
    return;
  }

  note_vclock_counter32(current_c32);

  g_schedule_armed = false;
  qtimer1_ch2_disable_compare();
  g_schedule_dispatched_total++;

  if (g_schedule_callback) {
    g_schedule_callback(dwt_at_edge, current_c32, g_schedule_user_data);
  }
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
// QTimer1 vector — dispatches CH2 (schedule fire) and CH3 (VCLOCK cadence)
// ============================================================================

static void qtimer1_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.

  // CH2 (schedule fire) checked first so SpinDry approaches see the
  // freshest possible _raw.  CH3 immediately after.  Both lanes share
  // priority; tail-chain ordering across QTimer1 vector is a same-priority
  // visit, so this in-vector ordering controls the relative latency.
  //
  // CRITICAL: Each lane is dispatched only if BOTH TCF1 (flag) AND
  // TCF1EN (enable) are set.  The TCF1 flag sets on every compare match
  // regardless of TCF1EN, so a disarmed channel whose CNTR wraps through
  // COMP1 will leave TCF1 set in hardware.  Without the TCF1EN gate, the
  // shared QTimer1 ISR (entered for CH3's own match) would also run CH2's
  // handler on every entry once CH2 has wrapped past 0xFFFF — a kHz-rate
  // storm of spurious CH2 dispatches.
  {
    const uint32_t cs = IMXRT_TMR1.CH[2].CSCTRL;
    if ((cs & TMR_CSCTRL_TCF1) && (cs & TMR_CSCTRL_TCF1EN)) {
      process_interrupt_qtimer1_ch2_irq(isr_entry_dwt_raw);
    }
  }
  {
    const uint32_t cs = IMXRT_TMR1.CH[3].CSCTRL;
    if ((cs & TMR_CSCTRL_TCF1) && (cs & TMR_CSCTRL_TCF1EN)) {
      vclock_cadence_isr(isr_entry_dwt_raw);
    }
  }
}

// ============================================================================
// PPS GPIO ISR — physical PPS witness, canonical VCLOCK epoch publisher
// ============================================================================
//
// The physical PPS edge is not published as the sacred TIMEBASE anchor.
// Instead, PPS selects the canonical VCLOCK-domain epoch.  Empirically,
// raw_cycles.py established:
//
//   canonical_vclock_epoch_dwt - raw_pps_isr_entry_dwt
//     == CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES
//
// This ISR captures raw PPS ISR-entry DWT, converts it to the canonical
// VCLOCK-selected epoch DWT using that calibrated offset, records the
// offset as a continuous witness, and dispatches PPS_VCLOCK with the
// canonical DWT.
//
// The subscription name PPS_VCLOCK is retained for compatibility, but its
// semantic meaning is: "the canonical VCLOCK-selected PPS epoch is now
// available."  PPS itself is a witness/selector, not the final DWT fact.
//
// counter32_at_edge remains the event-ledger VCLOCK coordinate associated
// with this epoch.  It is an authored ledger coordinate, not a live QTimer
// register read.

void process_interrupt_gpio6789_irq(uint32_t isr_entry_dwt_raw) {
  const uint32_t dwt_at_edge = pps_vclock_dwt_from_pps_isr_entry_raw(isr_entry_dwt_raw);
  note_pps_canonical_epoch_calibration(isr_entry_dwt_raw, dwt_at_edge);

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
  note_vclock_counter32(g_vclock_count32);
  dispatch_deferred(g_pps_vclock, dwt_at_edge, g_vclock_count32, gnss_ns);
}

static void pps_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(isr_entry_dwt_raw);
}

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

static void qtimer1_ch2_init_for_schedule(void) {
  // CH2 is the schedule-fire lane.  Configure to count the same VCLOCK
  // tick source as CH0/CH3 with a disabled compare; the compare is armed
  // dynamically via interrupt_schedule_fire_at().
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = 0;
  // CM=1 (count rising edges of primary source), PCS=0 (CH0 input → 10 MHz
  // VCLOCK).  Same source as CH3 cadence, so CH2's CNTR tracks the same
  // tick axis as g_vclock_count32.
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  qtimer1_ch2_disable_compare();
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
  qtimer1_ch2_init_for_schedule();
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
  // them.  Same applies to schedule-fire registration — timepop_init
  // calls interrupt_schedule_register() during phase 1.

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
  NVIC_SET_PRIORITY(IRQ_QTIMER1, NVIC_PRIO_QTIMER1);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, NVIC_PRIO_QTIMER3);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, NVIC_PRIO_PPS_GPIO);

  // Activate cadence lanes and arm CH3 compare at the next tick.
  g_vclock_lane.active = true;
  qtimer1_ch3_program_compare((uint16_t)(g_vclock_lane.compare_target + 1));

  g_ocxo1_lane.active = true;
  g_ocxo1_lane.phase_bootstrapped = false;
  qtimer3_program_compare(2, (uint16_t)(g_ocxo1_lane.compare_target + 1));

  g_ocxo2_lane.active = true;
  g_ocxo2_lane.phase_bootstrapped = false;
  qtimer3_program_compare(3, (uint16_t)(g_ocxo2_lane.compare_target + 1));

  // CH2 (schedule fire) stays disabled until the consumer arms it.

  g_interrupt_irqs_enabled = true;
}

// ============================================================================
// REPORT command — state visibility for debugging
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  // Report-time derived diagnostics.  These are observational only: they
  // do not feed back into timing behavior.  They let an operator compare
  // event-ledger counters against the bridge-projected live coordinate
  // and quickly spot phase-accounting drift.
  uint32_t report_projected_c32 = 0;
  const bool report_projected_c32_valid =
      project_vclock_counter32_from_dwt(ARM_DWT_CYCCNT, report_projected_c32);

  const uint32_t tick_sequence_mod_1000 =
      g_tick.sequence % (uint32_t)TICKS_PER_SECOND_EVENT;

  // Lifecycle
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);

  // PPS / canonical VCLOCK epoch witness state
  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("canonical_vclock_epoch_offset_config_cycles",
        (int32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES);
  p.add("canonical_vclock_epoch_offset_last_cycles",
        g_pps_canonical_offset_cycles_last);
  p.add("canonical_vclock_epoch_offset_min_cycles",
        (g_pps_canonical_offset_samples > 0) ? g_pps_canonical_offset_cycles_min : 0);
  p.add("canonical_vclock_epoch_offset_max_cycles",
        (g_pps_canonical_offset_samples > 0) ? g_pps_canonical_offset_cycles_max : 0);
  p.add("canonical_vclock_epoch_offset_samples",
        g_pps_canonical_offset_samples);
  p.add("canonical_vclock_epoch_offset_mismatches",
        g_pps_canonical_offset_mismatches);
  p.add("last_raw_pps_isr_entry_dwt",
        g_pps_raw_isr_entry_dwt_last);
  p.add("last_canonical_vclock_epoch_dwt",
        g_pps_canonical_epoch_dwt_last);

  // Subscriber sequences (monotone counters per kind)
  p.add("pps_vclock_sequence", g_pps_vclock.sequence);
  p.add("vclock_sequence",     g_vclock.sequence);
  p.add("ocxo1_sequence",      g_ocxo1.sequence);
  p.add("ocxo2_sequence",      g_ocxo2.sequence);
  p.add("tick_sequence",       g_tick.sequence);

  // Sequence relationships.  These are signed so report consumers can see
  // lead/lag directly.  In steady state, PPS may lead one-second cadence
  // lanes by one edge depending on where the snapshot lands in the second.
  p.add("pps_minus_vclock_sequence", (int32_t)(g_pps_vclock.sequence - g_vclock.sequence));
  p.add("pps_minus_ocxo1_sequence",  (int32_t)(g_pps_vclock.sequence - g_ocxo1.sequence));
  p.add("pps_minus_ocxo2_sequence",  (int32_t)(g_pps_vclock.sequence - g_ocxo2.sequence));
  p.add("ocxo1_minus_ocxo2_sequence", (int32_t)(g_ocxo1.sequence - g_ocxo2.sequence));
  p.add("tick_sequence_mod_1000", tick_sequence_mod_1000);

  // Subscriber active flags
  p.add("pps_vclock_active", g_pps_vclock.active);
  p.add("vclock_active",     g_vclock.active);
  p.add("ocxo1_active",      g_ocxo1.active);
  p.add("ocxo2_active",      g_ocxo2.active);
  p.add("tick_active",       g_tick.active);

  // Cadence lane state — phase bootstrap + most recent compare target
  p.add("vclock_lane_active",             g_vclock_lane.active);
  p.add("vclock_lane_phase_bootstrapped", g_vclock_lane.phase_bootstrapped);
  p.add("vclock_lane_compare_target",     (uint32_t)g_vclock_lane.compare_target);
  p.add("vclock_lane_tick_mod_1000",      g_vclock_lane.tick_mod_1000);

  p.add("ocxo1_lane_active",             g_ocxo1_lane.active);
  p.add("ocxo1_lane_phase_bootstrapped", g_ocxo1_lane.phase_bootstrapped);
  p.add("ocxo1_lane_compare_target",     (uint32_t)g_ocxo1_lane.compare_target);
  p.add("ocxo1_lane_tick_mod_1000",      g_ocxo1_lane.tick_mod_1000);

  p.add("ocxo2_lane_active",             g_ocxo2_lane.active);
  p.add("ocxo2_lane_phase_bootstrapped", g_ocxo2_lane.phase_bootstrapped);
  p.add("ocxo2_lane_compare_target",     (uint32_t)g_ocxo2_lane.compare_target);
  p.add("ocxo2_lane_tick_mod_1000",      g_ocxo2_lane.tick_mod_1000);

  // Cadence phase relationships.  These make modulo bookkeeping visible
  // without requiring the operator to do mental arithmetic.
  p.add("vclock_mod_minus_tick_mod", (int32_t)(g_vclock_lane.tick_mod_1000 - tick_sequence_mod_1000));
  p.add("ocxo1_mod_minus_tick_mod",  (int32_t)(g_ocxo1_lane.tick_mod_1000  - tick_sequence_mod_1000));
  p.add("ocxo2_mod_minus_tick_mod",  (int32_t)(g_ocxo2_lane.tick_mod_1000  - tick_sequence_mod_1000));
  p.add("ocxo1_mod_minus_ocxo2_mod", (int32_t)(g_ocxo1_lane.tick_mod_1000  - g_ocxo2_lane.tick_mod_1000));

  // Synthetic counter ledgers and projected live VCLOCK coordinate.
  // The ledger counters are event-authored values.  projected_live_counter32
  // is a report-time bridge projection from DWT and alpha's PPS_VCLOCK slot.
  p.add("vclock_event_counter32", (uint32_t)g_vclock_count32);
  p.add("ocxo1_event_counter32",  (uint32_t)g_ocxo1_count32);
  p.add("ocxo2_event_counter32",  (uint32_t)g_ocxo2_count32);

  p.add("last_known_counter32",       g_last_known_counter32);
  p.add("last_known_counter32_valid", g_last_known_counter32_valid);

  p.add("projected_live_counter32",       report_projected_c32);
  p.add("projected_live_counter32_valid", report_projected_c32_valid);
  if (report_projected_c32_valid) {
    p.add("projected_minus_vclock_event_ticks",
          (int32_t)(report_projected_c32 - g_vclock_count32));
    if (g_last_known_counter32_valid) {
      p.add("projected_minus_last_known_ticks",
            (int32_t)(report_projected_c32 - g_last_known_counter32));
    }
  }

  // NVIC priority structure
  p.add("nvic_prio_pps_gpio", NVIC_PRIO_PPS_GPIO);
  p.add("nvic_prio_qtimer1",  NVIC_PRIO_QTIMER1);
  p.add("nvic_prio_qtimer3",  NVIC_PRIO_QTIMER3);

  // Schedule-fire (CH2) state
  p.add("schedule_callback_registered", g_schedule_callback != nullptr);
  p.add("schedule_armed",               g_schedule_armed);
  p.add("schedule_target32",            g_schedule_target32);
  p.add("schedule_target16",            (uint32_t)g_schedule_target16);
  if (g_schedule_armed && report_projected_c32_valid) {
    p.add("schedule_target_minus_projected_ticks",
          (int32_t)(g_schedule_target32 - report_projected_c32));
  }
  if (g_schedule_armed && g_last_known_counter32_valid) {
    p.add("schedule_target_minus_last_known_ticks",
          (int32_t)(g_schedule_target32 - g_last_known_counter32));
  }
  p.add("schedule_armed_total",         g_schedule_armed_total);
  p.add("schedule_dispatched_total",    g_schedule_dispatched_total);
  p.add("schedule_late_arm_total",      g_schedule_late_arm_total);
  p.add("schedule_late_fire_total",     g_schedule_late_fire_total);
  p.add("schedule_cancelled_total",     g_schedule_cancelled_total);
  p.add("schedule_premature_wraps",     g_schedule_premature_wraps);
  p.add("schedule_projection_failures", g_schedule_projection_failures);
  p.add("schedule_match_tolerance_ticks", (uint32_t)SCHEDULE_MATCH_TOLERANCE_TICKS);

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
    case interrupt_lane_t::QTIMER1_CH2_SCHED: return "QTIMER1_CH2_SCHED";
    case interrupt_lane_t::QTIMER1_CH3_COMP:  return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER3_CH2_COMP:  return "QTIMER3_CH2_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP:  return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:         return "GPIO_EDGE";
    default:                                  return "NONE";
  }
}