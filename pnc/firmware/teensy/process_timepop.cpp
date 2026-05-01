// ============================================================================
// process_timepop.cpp — TimePop (VCLOCK-exact GNSS scheduling)
// ============================================================================
//
// TimePop is the GNSS-domain timer subsystem.
//
// Core timing substrate:
//   • process_interrupt owns the VCLOCK/QTimer1 hardware
//   • TimePop sees only synthetic 32-bit VCLOCK identities and event facts
//   • one VCLOCK tick = 100 ns
//   • QTimer1 CH2 is the dynamic compare scheduler, programmed by process_interrupt
//
// Scheduling modes:
//   • relative scheduling        — timepop_arm(delay_gnss_ns, ...)
//     When recurring=true, the first fire is snapped to the next
//     PPS/VCLOCK phase-grid boundary implied by delay_gnss_ns; subsequent
//     fires remain on that grid.
//   • scheduled ASAP dispatch    — timepop_arm_asap(...)
//   • scheduled ALAP dispatch    — timepop_arm_alap(...)
//   • absolute scheduling        — timepop_arm_at(target_gnss_ns, ...)
//   • anchor-relative scheduling — timepop_arm_from_anchor(anchor_gnss_ns, offset_gnss_ns, ...)
//   • caller-owned exact path    — timepop_arm_ns(target_gnss_ns, target_dwt, ...)
//
// fire_gnss_ns is computed from VCLOCK arithmetic:
//   gnss_ns = (pps_count - 1) * 1e9 + (fire_vclock_raw - qtimer_at_pps) * 100
//
// DWT is not the authoritative time base here.
// DWT is diagnostic / cross-check material.
//
// Scheduling philosophy — gears, not rubber bands:
//
//   Absolute and anchor-relative scheduling NEVER ask "what time is it now?"
//   Instead, deadlines are computed from the time anchor via pure arithmetic:
//
//     anchor_pps_gnss_ns = (pps_count - 1) * 1,000,000,000
//     ticks_from_anchor  = (target_gnss_ns - anchor_pps_gnss_ns) / 100
//     deadline           = qtimer_at_pps + ticks_from_anchor
//
//   This is possible because VCLOCK is phase-locked to GNSS.  The counter
//   IS the clock — reading it to compute a deadline introduces jitter that
//   pure arithmetic avoids.
//
//   Only relative scheduling ("fire N ns from now") reads the ambient counter,
//   because "from now" inherently requires knowing "now."
//
// Dispatch timing:
//
//   process_interrupt owns QTimer1 hardware and invokes TimePop's CH2 handler
//   in IRQ context with a normalized DWT capture and synthetic VCLOCK
//   counter32 event identity.  TimePop uses that event fact to expire slots.
//   It does not reinterpret PPS/VCLOCK phase and it does not read timer
//   hardware directly.
//
//   All timed slots expired by the same physical CH2 event receive the same
//   fire_vclock_raw, fire_dwt_cyccnt, and fire_gnss_ns.  Callback order is
//   logical only; it must never re-author the physical timing facts.
//
// ============================================================================

#include "config.h"
#include "timepop.h"
#include "process_timepop.h"

#include "publish.h"

#include "debug.h"
#include "process.h"
#include "events.h"
#include "payload.h"
#include "cpu_usage.h"
#include "time.h"
#include "config.h"
#include "util.h"

#include <Arduino.h>
#include "imxrt.h"
#include <string.h>
#include <math.h>
#include <climits>

// ============================================================================
// Forward declarations
// ============================================================================

static inline bool deadline_expired(uint32_t deadline, uint32_t now);
static bool gnss_ns_to_vclock_deadline(int64_t target_gnss_ns,
                                       const time_anchor_snapshot_t& snap,
                                       uint32_t& out_deadline);
static int64_t vclock_to_gnss_ns(uint32_t vclock_raw,
                                 const time_anchor_snapshot_t& snap);

// ============================================================================
// Constants — 10 MHz domain
// ============================================================================

static constexpr uint32_t MAX_SLOTS = 16;
static constexpr uint32_t MAX_ASAP_SLOTS = 8;
static constexpr uint32_t MAX_ALAP_SLOTS = 4;
static constexpr uint64_t NS_PER_TICK = 100ULL;
static constexpr uint32_t MAX_DELAY_TICKS = 0x7FFFFFFFU;
static constexpr uint32_t MIN_DELAY_TICKS = 2;
static constexpr uint32_t HEARTBEAT_TICKS = 10000;
static constexpr uint32_t PREDICT_MAX_QTIMER_ELAPSED = 15000000U;


// ============================================================================
// Slot
// ============================================================================

enum class timepop_recurrence_mode_t : uint8_t {
  NONE = 0,
  RELATIVE = 1,
  ABSOLUTE = 2,
};

enum class timepop_fire_capture_source_t : uint8_t {
  NONE = 0,
  IRQ_CH2,
  SCHEDULE_NEXT,
};

static const char* fire_capture_source_str(timepop_fire_capture_source_t source) {
  switch (source) {
    case timepop_fire_capture_source_t::IRQ_CH2:        return "IRQ_CH2";
    case timepop_fire_capture_source_t::SCHEDULE_NEXT: return "SCHEDULE_NEXT";
    default:                                           return "NONE";
  }
}

struct timepop_slot_t {
  bool                active;
  bool                expired;
  bool                recurring;
  bool                is_absolute;
  timepop_recurrence_mode_t recurrence_mode;


  timepop_handle_t    handle;
  uint32_t            deadline;          // single authoritative scheduled VCLOCK target
  uint32_t            fire_vclock_raw;
  uint32_t            period_ticks;
  uint64_t            period_ns;

  int64_t             target_gnss_ns;
  int64_t             fire_gnss_ns;

  uint32_t            fire_dwt_cyccnt;
  timepop_fire_capture_source_t fire_capture_source;

  uint32_t            anchor_dwt_at_pps;
  uint32_t            anchor_dwt_cycles_per_s;
  uint32_t            anchor_qtimer_at_pps;
  uint32_t            anchor_pps_count;
  bool                anchor_valid;

  uint32_t            predicted_dwt;
  bool                prediction_valid;
  bool                isr_callback;
  bool                isr_callback_fired;

  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;

  uint32_t            recurring_rearmed_count;
  uint32_t            recurring_immediate_expire_count;
  uint32_t            recurring_catchup_count;
  int64_t             recurring_base_gnss_ns;
  uint64_t            recurring_period_gnss_ns;

  uint32_t            arm_vclock_raw;
  uint32_t            arm_delta_ticks;
  uint32_t            first_expire_now;
  bool                first_expire_recorded;
};


struct deferred_slot_t {
  bool                active;
  timepop_handle_t    handle;
  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;
};

// ============================================================================
// State
// ============================================================================

static timepop_slot_t slots[MAX_SLOTS];
static deferred_slot_t asap_slots[MAX_ASAP_SLOTS];
static deferred_slot_t alap_slots[MAX_ALAP_SLOTS];
static volatile bool   timepop_pending = false;
static uint32_t        next_handle = 1;

// ============================================================================
// Diagnostics
// ============================================================================

static volatile uint32_t isr_fire_count     = 0;
static volatile uint32_t expired_count      = 0;
static volatile uint32_t phantom_count      = 0;

static volatile uint32_t diag_slots_high_water        = 0;
static volatile uint32_t diag_asap_slots_high_water   = 0;
static volatile uint32_t diag_alap_slots_high_water   = 0;
static volatile uint32_t diag_arm_failures            = 0;
static volatile uint32_t diag_named_replacements      = 0;

static volatile uint32_t diag_asap_armed              = 0;
static volatile uint32_t diag_asap_dispatched         = 0;
static volatile uint32_t diag_asap_arm_failures       = 0;
static volatile uint32_t diag_asap_last_armed_dwt     = 0;
static volatile uint32_t diag_asap_last_dispatch_dwt  = 0;
static volatile const char* diag_asap_last_armed_name = nullptr;

static volatile uint32_t diag_alap_armed              = 0;
static volatile uint32_t diag_alap_dispatched         = 0;
static volatile uint32_t diag_alap_arm_failures       = 0;
static volatile uint32_t diag_alap_last_armed_dwt     = 0;
static volatile uint32_t diag_alap_last_dispatch_dwt  = 0;
static volatile const char* diag_alap_last_armed_name = nullptr;

static volatile uint32_t diag_dispatch_calls     = 0;
static volatile uint32_t diag_dispatch_callbacks = 0;

static volatile uint32_t diag_isr_count          = 0;
static volatile uint32_t diag_rearm_count        = 0;
static volatile uint32_t diag_heartbeat_rearms   = 0;
static volatile uint32_t diag_race_recoveries    = 0;
static volatile uint32_t diag_isr_callbacks      = 0;

static volatile uint32_t diag_schedule_next_calls_total = 0;
static volatile uint32_t diag_schedule_next_calls_from_dispatch = 0;
static volatile uint32_t diag_schedule_next_calls_from_other = 0;

static volatile uint32_t diag_deadline_negative_offset = 0;
static volatile int64_t  diag_deadline_last_target_gnss_ns = -1;
static volatile int64_t  diag_deadline_last_anchor_pps_gnss_ns = -1;
static volatile int64_t  diag_deadline_last_ns_from_anchor = 0;



// ============================================================================
// Forward declarations
// ============================================================================

static void schedule_next(void);
static inline void slot_capture(timepop_slot_t& slot,
                                uint32_t fire_vclock_raw,
                                uint32_t fire_dwt_cyccnt,
                                int64_t fire_gnss_ns,
                                const time_anchor_snapshot_t& snap,
                                timepop_fire_capture_source_t source);
static inline void expire_slot_with_capture(timepop_slot_t& slot,
                                            uint32_t fire_vclock_raw,
                                            uint32_t fire_dwt_cyccnt,
                                            const time_anchor_snapshot_t& snap,
                                            timepop_fire_capture_source_t source);
static timepop_handle_t arm_absolute_slot_internal(
  int64_t             target_gnss_ns,
  bool                recurring,
  uint64_t            recurring_period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback,
  uint32_t            target_dwt = 0);

// ============================================================================
// Helpers
// ============================================================================

static inline void update_slot_high_water(void) {
  uint32_t active = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) active++;
  }
  if (active > diag_slots_high_water) diag_slots_high_water = active;
}


static inline void update_deferred_high_water(deferred_slot_t* slots_buf,
                                              uint32_t max_slots,
                                              volatile uint32_t& high_water) {
  uint32_t active = 0;
  for (uint32_t i = 0; i < max_slots; i++) {
    if (slots_buf[i].active) active++;
  }
  if (active > high_water) high_water = active;
}

static inline int64_t time_anchor_latest_pps_gnss_ns(const time_anchor_snapshot_t& snap) {
  if (!snap.ok || !snap.valid || snap.pps_count < 1) return -1;
  return (int64_t)(snap.pps_count - 1) * NS_PER_SECOND;
}

static bool phase_locked_next_target_gnss_ns(uint64_t period_ns,
                                             const time_anchor_snapshot_t& snap,
                                             int64_t now_gnss_ns,
                                             int64_t& out_target_gnss_ns) {
  if (period_ns == 0) return false;
  if (period_ns > (uint64_t)INT64_MAX) return false;
  if (now_gnss_ns < 0) return false;

  const int64_t anchor_gnss_ns = time_anchor_latest_pps_gnss_ns(snap);
  if (anchor_gnss_ns < 0) return false;

  int64_t offset_ns = now_gnss_ns - anchor_gnss_ns;
  if (offset_ns < 0) offset_ns = 0;

  const uint64_t steps = ((uint64_t)offset_ns / period_ns) + 1ULL;
  const int64_t target = anchor_gnss_ns + (int64_t)(steps * period_ns);
  if (target <= now_gnss_ns) return false;

  out_target_gnss_ns = target;
  return true;
}

static bool configure_phase_locked_recurring_slot(timepop_slot_t& slot,
                                                  uint64_t period_ns) {
  if (period_ns == 0) return false;

  const time_anchor_snapshot_t snap = time_anchor_snapshot();
  const int64_t now_gnss_ns = time_gnss_ns_now();
  int64_t target_gnss_ns = -1;
  if (!phase_locked_next_target_gnss_ns(period_ns, snap, now_gnss_ns, target_gnss_ns)) {
    return false;
  }

  uint32_t deadline = 0;
  if (!gnss_ns_to_vclock_deadline(target_gnss_ns, snap, deadline)) {
    return false;
  }

  const int64_t anchor_gnss_ns = time_anchor_latest_pps_gnss_ns(snap);

  slot.is_absolute = true;
  slot.recurrence_mode = timepop_recurrence_mode_t::ABSOLUTE;
  slot.recurring_base_gnss_ns =
      (anchor_gnss_ns >= 0) ? (anchor_gnss_ns + (int64_t)period_ns) : target_gnss_ns;
  slot.recurring_period_gnss_ns = period_ns;
  slot.target_gnss_ns = target_gnss_ns;
  slot.deadline = deadline;
  return true;
}

// Ambient VCLOCK observations are provided by process_interrupt.
// These values are scheduling observations only, not event facts.
static inline uint32_t vclock_count(void) {
  return interrupt_vclock_counter32_observe_ambient();
}

static inline bool deadline_expired(uint32_t deadline, uint32_t now) {
  return (deadline - now) > MAX_DELAY_TICKS;
}

static inline uint32_t ns_to_ticks(uint64_t ns) {
  uint64_t ticks = ns / NS_PER_TICK;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) ticks = MAX_DELAY_TICKS;
  return (uint32_t)ticks;
}

static uint32_t predict_dwt_at_deadline(uint32_t deadline, bool& valid) {
  valid = false;
  time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid) return 0;
  if (snap.dwt_cycles_per_s == 0) return 0;

  const uint32_t qtimer_elapsed = deadline - snap.qtimer_at_pps;
  if (qtimer_elapsed > PREDICT_MAX_QTIMER_ELAPSED) return 0;

  const uint64_t ns_elapsed =
    (uint64_t)qtimer_elapsed * NS_PER_TICK;

  const uint64_t dwt_elapsed =
    (ns_elapsed * (uint64_t)snap.dwt_cycles_per_s + 500000000ULL) / 1000000000ULL;

  valid = true;
  return snap.dwt_at_pps + (uint32_t)dwt_elapsed;
}

// ============================================================================
// Absolute GNSS nanosecond → VCLOCK deadline (pure arithmetic)
// ============================================================================
//
// Converts a target GNSS nanosecond to a VCLOCK deadline using the time
// anchor.  No ambient counter read.  No "what time is it now?"
//
// The arithmetic:
//   anchor_pps_gnss_ns = (pps_count - 1) * 1,000,000,000
//   ns_from_anchor     = target_gnss_ns - anchor_pps_gnss_ns
//   ticks_from_anchor  = ns_from_anchor / 100
//   deadline           = qtimer_at_pps + ticks_from_anchor
//
// Returns true if the conversion succeeded (anchor valid, target in range).
// On success, writes the VCLOCK deadline to *out_deadline.
static bool gnss_ns_to_vclock_deadline(int64_t target_gnss_ns,
                                       const time_anchor_snapshot_t& snap,
                                       uint32_t& out_deadline) {
  if (!snap.ok || !snap.valid || snap.pps_count < 1) return false;

  const int64_t anchor_pps_gnss_ns =
    (int64_t)(snap.pps_count - 1) * (int64_t)1000000000LL;

  const int64_t ns_from_anchor = target_gnss_ns - anchor_pps_gnss_ns;

  diag_deadline_last_target_gnss_ns = target_gnss_ns;
  diag_deadline_last_anchor_pps_gnss_ns = anchor_pps_gnss_ns;
  diag_deadline_last_ns_from_anchor = ns_from_anchor;

  if (ns_from_anchor < 0) {
    diag_deadline_negative_offset++;
    return false;
  }

  const uint64_t ticks64 = (uint64_t)ns_from_anchor / NS_PER_TICK;
  if (ticks64 > 0xFFFFFFFFULL) {
    return false;
  }

  out_deadline = snap.qtimer_at_pps + (uint32_t)ticks64;
  return true;
}

// ============================================================================
// Named singleton replacement
// ============================================================================

static inline void note_named_replacement(const char*) {
  diag_named_replacements++;
}
static bool retire_existing_named_slots(const char* name) {
  if (!name || !*name) return false;

  bool retired_any = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;
    if (!slots[i].name) continue;
    if (strcmp(slots[i].name, name) != 0) continue;

    slots[i].active = false;
    slots[i].expired = false;
    retired_any = true;
    note_named_replacement(name);
  }

  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (!asap_slots[i].active) continue;
    if (!asap_slots[i].name) continue;
    if (strcmp(asap_slots[i].name, name) != 0) continue;

    asap_slots[i].active = false;
    retired_any = true;
    note_named_replacement(name);
  }

  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (!alap_slots[i].active) continue;
    if (!alap_slots[i].name) continue;
    if (strcmp(alap_slots[i].name, name) != 0) continue;

    alap_slots[i].active = false;
    retired_any = true;
    note_named_replacement(name);
  }

  return retired_any;
}

// ============================================================================
// VCLOCK-exact GNSS nanosecond computation
// ============================================================================

static int64_t vclock_to_gnss_ns(uint32_t vclock_raw,
                                 const time_anchor_snapshot_t& snap) {
  if (!snap.ok || !snap.valid || snap.pps_count < 1) return -1;

  const uint32_t ticks_since_pps = vclock_raw - snap.qtimer_at_pps;

  return (int64_t)(snap.pps_count - 1) * (int64_t)1000000000LL
       + (int64_t)((uint64_t)ticks_since_pps * NS_PER_TICK);
}

// ============================================================================
// Dispatch timing validation — landed DWT domain
// ============================================================================

// ============================================================================
// CH2 compare management
// ============================================================================
//
// process_interrupt owns QTimer1 CH2 hardware.  TimePop computes the next
// desired VCLOCK deadline and asks process_interrupt to program the compare.

static inline void ch2_arm_compare(uint32_t target_counter32) {
  interrupt_qtimer1_ch2_arm_compare(target_counter32);
}

// ============================================================================
// schedule_next
// ============================================================================
//
// This is the ONE place in TimePop that legitimately reads the ambient
// VCLOCK counter.  It must compare deadlines against the current counter
// to determine which slots have expired and how far away the nearest
// deadline is.  This is the heartbeat polling loop.

static void schedule_next(void) {
  diag_schedule_next_calls_total++;

  const uint32_t now = vclock_count();
  const time_anchor_snapshot_t snap = time_anchor_snapshot();
  uint32_t shared_fire_dwt = 0;
  bool shared_fire_captured = false;

  uint32_t soonest = 0;
  bool found = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;

    if (deadline_expired(slots[i].deadline, now)) {
      if (!slots[i].first_expire_recorded) {
        slots[i].first_expire_now = now;
        slots[i].first_expire_recorded = true;
      }

      // SCHEDULE_NEXT is a fallback/foreground discovery path, not the ideal
      // IRQ_CH2 compare path.  Even here, every slot expired by this one
      // scheduling pass receives the same captured timing facts.
      if (!shared_fire_captured) {
        shared_fire_dwt = ARM_DWT_CYCCNT;
        shared_fire_captured = true;
      }

      expire_slot_with_capture(slots[i],
                               now,
                               shared_fire_dwt,
                               snap,
                               timepop_fire_capture_source_t::SCHEDULE_NEXT);
      continue;
    }

    if (!found || ((slots[i].deadline - now) < (soonest - now))) {
      soonest = slots[i].deadline;
      found = true;
    }
  }


  uint32_t target;
  if (!found) {
    target = now + HEARTBEAT_TICKS;
    diag_heartbeat_rearms++;
  } else {
    uint32_t distance = soonest - now;
    if (distance > HEARTBEAT_TICKS) {
      target = now + HEARTBEAT_TICKS;
      diag_heartbeat_rearms++;
    } else {
      target = soonest;
    }
  }

  ch2_arm_compare(target);
  diag_rearm_count++;

  const uint16_t cntr = interrupt_qtimer1_ch2_counter_now();
  const uint16_t comp = (uint16_t)(target & 0xFFFF);
  const uint16_t distance_16 = comp - cntr;
  (void)distance_16;
}

// ============================================================================
// ISR helpers
// ============================================================================

static inline void slot_capture(
  timepop_slot_t& slot,
  uint32_t fire_vclock_raw,
  uint32_t fire_dwt_cyccnt,
  int64_t fire_gnss_ns,
  const time_anchor_snapshot_t& snap,
  timepop_fire_capture_source_t source
) {
  slot.fire_vclock_raw         = fire_vclock_raw;
  slot.fire_dwt_cyccnt         = fire_dwt_cyccnt;
  slot.fire_gnss_ns            = fire_gnss_ns;
  slot.fire_capture_source     = source;
  slot.anchor_dwt_at_pps       = snap.dwt_at_pps;
  slot.anchor_dwt_cycles_per_s = snap.dwt_cycles_per_s;
  slot.anchor_qtimer_at_pps    = snap.qtimer_at_pps;
  slot.anchor_pps_count        = snap.pps_count;
  slot.anchor_valid            = snap.ok && snap.valid;
}

static inline void expire_slot_with_capture(timepop_slot_t& slot,
                                            uint32_t fire_vclock_raw,
                                            uint32_t fire_dwt_cyccnt,
                                            const time_anchor_snapshot_t& snap,
                                            timepop_fire_capture_source_t source) {
  const int64_t fire_gnss_ns = vclock_to_gnss_ns(fire_vclock_raw, snap);
  slot_capture(slot, fire_vclock_raw, fire_dwt_cyccnt, fire_gnss_ns, snap, source);

  slot.expired = true;
  expired_count++;
  timepop_pending = true;
}

static inline void slot_build_ctx(
  const timepop_slot_t& slot,
  timepop_ctx_t& ctx
) {
  // deadline is the one authoritative scheduled VCLOCK target.
  // fire_gnss_error_ns compares the shared captured event against that target.
  ctx.handle              = slot.handle;
  ctx.fire_vclock_raw     = slot.fire_vclock_raw;
  ctx.fire_dwt_cyccnt     = slot.fire_dwt_cyccnt;
  ctx.deadline            = slot.deadline;
  ctx.fire_gnss_error_ns  = (int32_t)(slot.fire_vclock_raw - slot.deadline) * (int32_t)NS_PER_TICK;
  ctx.fire_gnss_ns        = slot.fire_gnss_ns;
}

static inline void slot_build_diag(
  const timepop_slot_t& slot,
  uint32_t dwt_at_isr_entry,
  timepop_diag_t& diag
) {
  diag.dwt_at_isr_entry        = dwt_at_isr_entry;
  diag.dwt_at_fire             = slot.fire_dwt_cyccnt;
  diag.predicted_dwt           = slot.predicted_dwt;
  diag.prediction_valid        = slot.prediction_valid;
  diag.spin_error_cycles       = (int32_t)(slot.fire_dwt_cyccnt - slot.predicted_dwt);
  diag.anchor_pps_count        = slot.anchor_pps_count;
  diag.anchor_qtimer_at_pps    = slot.anchor_qtimer_at_pps;
  diag.anchor_dwt_at_pps       = slot.anchor_dwt_at_pps;
  diag.anchor_dwt_cycles_per_s = slot.anchor_dwt_cycles_per_s;
  diag.anchor_valid            = slot.anchor_valid;
}

// ============================================================================
// QTimer1 CH2 handler — TimePop priority queue scheduler
// ============================================================================
//
// IRQ-context handler called by process_interrupt's qtimer1_isr
// dispatcher on every CH2 compare-match.  Receives the standard
// event/diag payload — DWT, gnss_ns, counter32 — already filled in by
// the dispatcher.  No need to re-read the counter or recompute gnss_ns
// here.  The CH2 TCF1 flag is cleared by the dispatcher before this
// runs, so we don't clear it ourselves.
//
void timepop_qtimer1_ch2_handler(const interrupt_event_t& event,
                                 const interrupt_capture_diag_t& /*diag*/) {

  const uint32_t dwt_entry      = event.dwt_at_event;
  const uint32_t now            = event.counter32_at_event;
  const time_anchor_snapshot_t anchor = time_anchor_snapshot();
  bool expired_this_pass[MAX_SLOTS] = {};

  diag_isr_count++;

  bool any_expired = false;

  // Phase 1: author the shared fire facts for every slot expired by this
  // physical CH2 compare event before any user callback is allowed to run.
  // This guarantees simultaneous clients receive the same DWT/VCLOCK/GNSS
  // context; callback serialization is logical only.
  const int64_t event_fire_gnss_ns = event.gnss_ns_at_event != 0
      ? (int64_t)event.gnss_ns_at_event
      : vclock_to_gnss_ns(now, anchor);

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

    slot_capture(slots[i],
                 now,
                 dwt_entry,
                 event_fire_gnss_ns,
                 anchor,
                 timepop_fire_capture_source_t::IRQ_CH2);
    slots[i].expired = true;
    slots[i].isr_callback_fired = false;
    expired_this_pass[i] = true;
    expired_count++;
    timepop_pending = true;
    any_expired = true;
  }

  // Phase 2: now that the fire facts are stable for all simultaneous slots,
  // dispatch any IRQ-context callbacks.  Slots without isr_callback will be
  // dispatched later by timepop_dispatch() using the same stored context.
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!expired_this_pass[i]) continue;
    if (!slots[i].active || !slots[i].expired) continue;
    if (!slots[i].isr_callback) continue;

    timepop_ctx_t ctx;
    slot_build_ctx(slots[i], ctx);

    timepop_diag_t diag;
    slot_build_diag(slots[i], dwt_entry, diag);

    slots[i].callback(&ctx, &diag, slots[i].user_data);
    slots[i].isr_callback_fired = true;
    diag_isr_callbacks++;
  }

  if (any_expired) {
    isr_fire_count++;
    timepop_pending = true;
  } else {
    phantom_count++;
  }

  diag_schedule_next_calls_from_other++;
  schedule_next();
}


static inline void build_deferred_ctx(timepop_handle_t handle, timepop_ctx_t& ctx) {
  ctx.handle = handle;
  ctx.fire_vclock_raw = 0;
  ctx.fire_dwt_cyccnt = 0;
  ctx.deadline = 0;
  ctx.fire_gnss_error_ns = 0;
  ctx.fire_gnss_ns = -1;
}

static void dispatch_deferred_phase(deferred_slot_t* slots_buf,
                                    uint32_t max_slots,
                                    volatile uint32_t& dispatched_count,
                                    volatile uint32_t& last_dispatch_dwt) {
  bool pending_this_pass[MAX_ASAP_SLOTS > MAX_ALAP_SLOTS ? MAX_ASAP_SLOTS : MAX_ALAP_SLOTS] = {};
  for (uint32_t i = 0; i < max_slots; i++) {
    pending_this_pass[i] = slots_buf[i].active;
  }

  for (uint32_t i = 0; i < max_slots; i++) {
    if (!pending_this_pass[i]) continue;
    if (!slots_buf[i].active) continue;

    timepop_ctx_t ctx;
    build_deferred_ctx(slots_buf[i].handle, ctx);

    uint32_t start = ARM_DWT_CYCCNT;
    slots_buf[i].callback(&ctx, nullptr, slots_buf[i].user_data);
    uint32_t end = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);
    diag_dispatch_callbacks++;
    dispatched_count++;
    last_dispatch_dwt = end;

    slots_buf[i].active = false;
  }
}

static timepop_handle_t arm_deferred(deferred_slot_t* slots_buf,
                                     uint32_t max_slots,
                                     volatile uint32_t& arm_failures,
                                     volatile uint32_t& armed_count,
                                     volatile uint32_t& last_armed_dwt,
                                     volatile const char*& last_armed_name,
                                     volatile uint32_t& high_water,
                                     timepop_callback_t callback,
                                     void* user_data,
                                     const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  const uint32_t saved = critical_enter();

  const bool retired_named = retire_existing_named_slots(name);
  if (retired_named) {
    timepop_pending = true;
  }

  for (uint32_t i = 0; i < max_slots; i++) {
    if (slots_buf[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots_buf[i] = {};
    slots_buf[i].active = true;
    slots_buf[i].handle = h;
    slots_buf[i].callback = callback;
    slots_buf[i].user_data = user_data;
    slots_buf[i].name = name;

    timepop_pending = true;
    armed_count++;
    last_armed_dwt = ARM_DWT_CYCCNT;
    last_armed_name = name;

    update_deferred_high_water(slots_buf, max_slots, high_water);

    critical_exit(saved);
    return h;
  }

  arm_failures++;
  critical_exit(saved);
  return TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// QTimer1 hardware initialization moved to process_interrupt
// ============================================================================
//
// CH0/CH1 cascade and CH2 scheduler channel are now initialized by
// process_interrupt_init_hardware().  TimePop only programs the CH2
// compare register as it schedules slots; the channel mode/control
// setup is no longer TimePop's responsibility.

// ============================================================================
// Initialization
// ============================================================================

void timepop_init(void) {
  for (uint32_t i = 0; i < MAX_SLOTS; i++) slots[i] = {};
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) asap_slots[i] = {};
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) alap_slots[i] = {};

  diag_deadline_negative_offset = 0;
  diag_deadline_last_target_gnss_ns = -1;
  diag_deadline_last_anchor_pps_gnss_ns = -1;
  diag_deadline_last_ns_from_anchor = 0;
  // QTimer1 hardware (CH0/CH1/CH2) is initialized by
  // process_interrupt_init_hardware().  IRQ vector and NVIC priority
  // are owned by process_interrupt_enable_irqs().  We just register
  // our CH2 handler here; the dispatcher will invoke it in IRQ
  // context with a fully-populated event/diag payload.
  interrupt_register_qtimer1_ch2_handler(timepop_qtimer1_ch2_handler);

  const uint32_t saved = critical_enter();
  diag_schedule_next_calls_from_other++;
  schedule_next();
  critical_exit(saved);
}

static timepop_handle_t arm_absolute_slot_internal(
  int64_t             target_gnss_ns,
  bool                recurring,
  uint64_t            recurring_period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback,
  uint32_t            target_dwt
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  const time_anchor_snapshot_t snap = time_anchor_snapshot();
  uint32_t deadline = 0;
  if (!gnss_ns_to_vclock_deadline(target_gnss_ns, snap, deadline)) {
    return TIMEPOP_INVALID_HANDLE;
  }

  const uint32_t saved = critical_enter();

  const bool retired_named = retire_existing_named_slots(name);
  if (retired_named) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
  }

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].expired        = false;
    slots[i].recurring      = recurring;
    slots[i].is_absolute    = true;
    slots[i].recurrence_mode =
        (recurring && recurring_period_gnss_ns > 0)
            ? timepop_recurrence_mode_t::ABSOLUTE
            : timepop_recurrence_mode_t::NONE;
    slots[i].handle         = h;
    slots[i].period_ns      = recurring_period_gnss_ns;
    slots[i].period_ticks   = (recurring_period_gnss_ns > 0) ? ns_to_ticks(recurring_period_gnss_ns) : 0;
    slots[i].deadline       = deadline;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;
    slots[i].fire_gnss_ns   = -1;
    slots[i].isr_callback   = isr_callback;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;
    slots[i].recurring_base_gnss_ns = target_gnss_ns;
    slots[i].recurring_period_gnss_ns = recurring_period_gnss_ns;
    slots[i].arm_vclock_raw = deadline;
    slots[i].arm_delta_ticks = 0;
    slots[i].first_expire_now = 0;
    slots[i].first_expire_recorded = false;

    if (target_dwt != 0) {
      slots[i].predicted_dwt    = target_dwt;
      slots[i].prediction_valid = true;
    } else {
      slots[i].predicted_dwt = predict_dwt_at_deadline(deadline, slots[i].prediction_valid);
    }

    diag_schedule_next_calls_from_other++;
    schedule_next();
    update_slot_high_water();
    critical_exit(saved);
    return h;
  }

  diag_arm_failures++;
  critical_exit(saved);
  return TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// Arm — relative scheduling ("from now")
// ============================================================================
//
// This is the ONLY arming path that reads the ambient VCLOCK counter,
// because "from now" inherently requires knowing "now."

timepop_handle_t timepop_arm(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  const uint32_t saved = critical_enter();

  const bool retired_named = retire_existing_named_slots(name);
  if (retired_named) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
  }

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active       = true;
    slots[i].expired      = false;
    slots[i].recurring    = recurring;
    slots[i].is_absolute  = false;
    slots[i].recurrence_mode = timepop_recurrence_mode_t::NONE;
    slots[i].handle       = h;
    slots[i].period_ns    = delay_gnss_ns;
    slots[i].callback     = callback;
    slots[i].user_data    = user_data;
    slots[i].name         = name;
    slots[i].fire_gnss_ns = -1;
    slots[i].target_gnss_ns = -1;
    slots[i].isr_callback = false;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;
    slots[i].recurring_base_gnss_ns = 0;
    slots[i].recurring_period_gnss_ns = 0;
    slots[i].arm_vclock_raw = 0;
    slots[i].arm_delta_ticks = 0;
    slots[i].first_expire_now = 0;
    slots[i].first_expire_recorded = false;

    const uint32_t ticks = ns_to_ticks(delay_gnss_ns);
    slots[i].period_ticks = ticks;

    // Relative scheduling: "from now" requires reading the counter.
    const uint32_t now = vclock_count();
    slots[i].deadline = now + ticks;
    slots[i].arm_vclock_raw = now;
    slots[i].arm_delta_ticks = ticks;

    if (recurring) {
      // Recurring timers are phase-locked to the PPS/VCLOCK grid implied by
      // their period.  The first fire snaps to the next legal grid boundary;
      // subsequent fires advance by the exact period.
      if (!configure_phase_locked_recurring_slot(slots[i], delay_gnss_ns)) {
        slots[i].recurrence_mode = timepop_recurrence_mode_t::RELATIVE;
        slots[i].recurring_period_gnss_ns = delay_gnss_ns;
      }
    }

    slots[i].predicted_dwt = predict_dwt_at_deadline(
      slots[i].deadline, slots[i].prediction_valid);

    if (slots[i].prediction_valid && slots[i].target_gnss_ns < 0) {
      slots[i].target_gnss_ns = time_dwt_to_gnss_ns(slots[i].predicted_dwt);
    }

    diag_schedule_next_calls_from_other++;
    schedule_next();

    update_slot_high_water();
    critical_exit(saved);
    return h;
  }

  diag_arm_failures++;
  critical_exit(saved);
  return TIMEPOP_INVALID_HANDLE;
}

timepop_handle_t timepop_arm_asap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_deferred(asap_slots,
                      MAX_ASAP_SLOTS,
                      diag_asap_arm_failures,
                      diag_asap_armed,
                      diag_asap_last_armed_dwt,
                      diag_asap_last_armed_name,
                      diag_asap_slots_high_water,
                      callback,
                      user_data,
                      name);
}

timepop_handle_t timepop_arm_alap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_deferred(alap_slots,
                      MAX_ALAP_SLOTS,
                      diag_alap_arm_failures,
                      diag_alap_armed,
                      diag_alap_last_armed_dwt,
                      diag_alap_last_armed_name,
                      diag_alap_slots_high_water,
                      callback,
                      user_data,
                      name);
}

// ============================================================================
// Arm — absolute scheduling (deterministic, no ambient "now")
// ============================================================================
//
// Converts the target GNSS nanosecond to a VCLOCK deadline using the time
// anchor via pure arithmetic.  No call to vclock_count().  No call to
// time_gnss_ns_now().  The deadline is geared from the anchor, not measured
// from ambient time.

timepop_handle_t timepop_arm_at(
  int64_t             target_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  // Public absolute timers remain one-shot unless a future API supplies an
  // explicit absolute recurrence period.
  (void)recurring;
  return arm_absolute_slot_internal(target_gnss_ns,
                                    false,
                                    0,
                                    callback,
                                    user_data,
                                    name,
                                    false,
                                    0);
}

// ============================================================================
// Arm — anchor-relative GNSS scheduling
// ============================================================================

timepop_handle_t timepop_arm_from_anchor(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (anchor_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  const int64_t target_gnss_ns = anchor_gnss_ns + offset_gnss_ns;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= anchor_gnss_ns && offset_gnss_ns > 0) {
    return TIMEPOP_INVALID_HANDLE;
  }

  return arm_absolute_slot_internal(target_gnss_ns,
                                    recurring,
                                    recurring ? 1000000000ULL : 0ULL,
                                    callback,
                                    user_data,
                                    name,
                                    false,
                                    0);
}

// ============================================================================
// Arm — caller-owned exact target path (deterministic, no ambient "now")
// ============================================================================
//
// The caller provides both the GNSS nanosecond target and the DWT prediction.
// The VCLOCK deadline is computed from the time anchor — no ambient read.

timepop_handle_t timepop_arm_ns(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback
) {
  return arm_absolute_slot_internal(target_gnss_ns,
                                    false,
                                    0,
                                    callback,
                                    user_data,
                                    name,
                                    isr_callback,
                                    target_dwt);
}

// ============================================================================
// Cancel
// ============================================================================

bool timepop_cancel(timepop_handle_t handle) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;

  const uint32_t saved = critical_enter();
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].handle == handle) {
      slots[i].active = false;
      diag_schedule_next_calls_from_other++;
      schedule_next();
      critical_exit(saved);
      return true;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].handle == handle) {
      asap_slots[i].active = false;
      critical_exit(saved);
      return true;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].handle == handle) {
      alap_slots[i].active = false;
      critical_exit(saved);
      return true;
    }
  }
  critical_exit(saved);
  return false;
}

uint32_t timepop_cancel_by_name(const char* name) {
  if (!name) return 0;

  uint32_t cancelled = 0;
  const uint32_t saved = critical_enter();
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name && strcmp(slots[i].name, name) == 0) {
      slots[i].active = false;
      cancelled++;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].name && strcmp(asap_slots[i].name, name) == 0) {
      asap_slots[i].active = false;
      cancelled++;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].name && strcmp(alap_slots[i].name, name) == 0) {
      alap_slots[i].active = false;
      cancelled++;
    }
  }
  if (cancelled > 0) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
    timepop_pending = true;
  }
  critical_exit(saved);
  return cancelled;
}

// ============================================================================
// Dispatch
// ============================================================================

void timepop_dispatch(void) {
  if (!timepop_pending) return;
  timepop_pending = false;
  diag_dispatch_calls++;


  dispatch_deferred_phase(asap_slots,
                          MAX_ASAP_SLOTS,
                          diag_asap_dispatched,
                          diag_asap_last_dispatch_dwt);

  const uint32_t dispatch_now = vclock_count();
  (void)dispatch_now;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;
    slots[i].expired = false;

    const bool callback_already_ran = slots[i].isr_callback_fired;
    slots[i].isr_callback_fired = false;

    if (!callback_already_ran) {
      // ── Timed scheduled-context callback ──
      timepop_ctx_t ctx;
      slot_build_ctx(slots[i], ctx);

      timepop_diag_t diag;
      slot_build_diag(slots[i], 0, diag);

      uint32_t start = ARM_DWT_CYCCNT;
      slots[i].callback(&ctx, &diag, slots[i].user_data);
      uint32_t end   = ARM_DWT_CYCCNT;

      cpu_usage_account_busy(end - start);
      diag_dispatch_callbacks++;
    }

    // ── Recurring rearm ──
    if (slots[i].recurring) {
      if (slots[i].recurrence_mode == timepop_recurrence_mode_t::ABSOLUTE &&
          slots[i].recurring_period_gnss_ns > 0 &&
          slots[i].target_gnss_ns > 0) {
        int64_t next_target_gnss_ns = slots[i].target_gnss_ns + (int64_t)slots[i].recurring_period_gnss_ns;
        const int64_t now_gnss_ns = time_gnss_ns_now();
        if (now_gnss_ns >= 0 && next_target_gnss_ns <= now_gnss_ns) {
          const uint64_t missed =
              (uint64_t)((now_gnss_ns - next_target_gnss_ns) / (int64_t)slots[i].recurring_period_gnss_ns) + 1ULL;
          next_target_gnss_ns += (int64_t)(missed * slots[i].recurring_period_gnss_ns);
          slots[i].recurring_immediate_expire_count++;
          slots[i].recurring_catchup_count += (uint32_t)missed;
        }

        const time_anchor_snapshot_t snap = time_anchor_snapshot();
        uint32_t next_deadline = 0;
        if (!gnss_ns_to_vclock_deadline(next_target_gnss_ns, snap, next_deadline)) {
          slots[i].active = false;
        } else {
          slots[i].target_gnss_ns = next_target_gnss_ns;
          slots[i].deadline = next_deadline;
          slots[i].recurring_rearmed_count++;
          slots[i].predicted_dwt = predict_dwt_at_deadline(
            slots[i].deadline, slots[i].prediction_valid);
          slots[i].expired = false;
          const uint32_t saved = critical_enter();
          schedule_next();
          critical_exit(saved);
        }
      } else if (slots[i].period_ticks == 0) {
        slots[i].active = false;
      } else {
        // Relative recurring is only a bootstrap/fallback state.  As soon as a
        // lawful time anchor is available, promote it onto the PPS/VCLOCK phase
        // grid for its period.
        const uint64_t period_ns = slots[i].recurring_period_gnss_ns
            ? slots[i].recurring_period_gnss_ns
            : slots[i].period_ns;
        if (configure_phase_locked_recurring_slot(slots[i], period_ns)) {
          slots[i].recurring_rearmed_count++;
        } else {
          // Last-resort relative recurrence: advance deadline by one period.
          slots[i].deadline += slots[i].period_ticks;
          slots[i].recurring_rearmed_count++;

          // Catch-up: if we fell behind, skip forward.
          uint32_t now = vclock_count();
          if (deadline_expired(slots[i].deadline, now)) {
            slots[i].recurring_immediate_expire_count++;
            slots[i].deadline = now + slots[i].period_ticks;
            slots[i].recurring_catchup_count++;
          }
        }

        slots[i].predicted_dwt = predict_dwt_at_deadline(
          slots[i].deadline, slots[i].prediction_valid);

        if (slots[i].prediction_valid && slots[i].target_gnss_ns < 0) {
          slots[i].target_gnss_ns = time_dwt_to_gnss_ns(slots[i].predicted_dwt);
        }

        slots[i].expired = false;
        const uint32_t saved = critical_enter();
        schedule_next();
        critical_exit(saved);
      }
    } else {
      slots[i].active = false;
    }
  }

  dispatch_deferred_phase(alap_slots,
                          MAX_ALAP_SLOTS,
                          diag_alap_dispatched,
                          diag_alap_last_dispatch_dwt);

  bool more_pending = false;
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active) { more_pending = true; break; }
  }
  if (!more_pending) {
    for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
      if (alap_slots[i].active) { more_pending = true; break; }
    }
  }
  if (more_pending) {
    timepop_pending = true;
  }
}
// ============================================================================
// Introspection
// ============================================================================

uint32_t timepop_active_count(void) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) n++;
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active) n++;
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active) n++;
  }
  return n;
}

// ============================================================================
// REPORT
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload out;

  out.add("isr_fires",         isr_fire_count);
  out.add("isr_count",         diag_isr_count);
  out.add("isr_callbacks",     diag_isr_callbacks);
  out.add("phantom_count",     phantom_count);
  out.add("expired",           expired_count);
  out.add("rearm_count",       diag_rearm_count);
  out.add("heartbeat_rearms",  diag_heartbeat_rearms);
  out.add("race_recoveries",   diag_race_recoveries);
  out.add("vclock_raw_now",    vclock_count());
  out.add("time_valid",        time_valid());
  out.add("time_pps_count",    time_pps_count());

  out.add("slots_active_now",       timepop_active_count());
  out.add("slots_high_water",       diag_slots_high_water);
  out.add("slots_max",              (uint32_t)MAX_SLOTS);
  out.add("arm_failures",           diag_arm_failures);
  out.add("named_replacements",     diag_named_replacements);

  out.add("asap_armed",             diag_asap_armed);
  out.add("asap_dispatched",        diag_asap_dispatched);
  out.add("asap_arm_failures",      diag_asap_arm_failures);
  out.add("asap_slots_high_water",  diag_asap_slots_high_water);
  out.add("asap_slots_max",         (uint32_t)MAX_ASAP_SLOTS);
  out.add("asap_last_armed_dwt",    diag_asap_last_armed_dwt);
  out.add("asap_last_dispatch_dwt", diag_asap_last_dispatch_dwt);
  out.add("asap_last_armed_name",
          (const char*)(diag_asap_last_armed_name ? diag_asap_last_armed_name : ""));

  out.add("alap_armed",             diag_alap_armed);
  out.add("alap_dispatched",        diag_alap_dispatched);
  out.add("alap_arm_failures",      diag_alap_arm_failures);
  out.add("alap_slots_high_water",  diag_alap_slots_high_water);
  out.add("alap_slots_max",         (uint32_t)MAX_ALAP_SLOTS);
  out.add("alap_last_armed_dwt",    diag_alap_last_armed_dwt);
  out.add("alap_last_dispatch_dwt", diag_alap_last_dispatch_dwt);
  out.add("alap_last_armed_name",
          (const char*)(diag_alap_last_armed_name ? diag_alap_last_armed_name : ""));

  out.add("dispatch_calls",         diag_dispatch_calls);
  out.add("dispatch_callbacks",     diag_dispatch_callbacks);
  out.add("schedule_next_calls_total",         diag_schedule_next_calls_total);
  out.add("schedule_next_calls_from_dispatch", diag_schedule_next_calls_from_dispatch);
  out.add("schedule_next_calls_from_other",    diag_schedule_next_calls_from_other);

  out.add("deadline_negative_offset",          diag_deadline_negative_offset);
  out.add("deadline_last_target_gnss_ns",      diag_deadline_last_target_gnss_ns);
  out.add("deadline_last_anchor_pps_gnss_ns",  diag_deadline_last_anchor_pps_gnss_ns);
  out.add("deadline_last_ns_from_anchor",      diag_deadline_last_ns_from_anchor);

  out.add("qtmr1_ch2_cntr",   (uint32_t)interrupt_qtimer1_ch2_counter_now());
  out.add("qtmr1_ch2_comp1",  (uint32_t)interrupt_qtimer1_ch2_comp1_now());
  out.add("qtmr1_ch2_csctrl", (uint32_t)interrupt_qtimer1_ch2_csctrl_now());

  PayloadArray timers;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;

    Payload entry;
    entry.add("slot",      i);
    entry.add("handle",    slots[i].handle);
    entry.add("name",      slots[i].name ? slots[i].name : "");
    entry.add("period_ns", slots[i].period_ns);
    entry.add("ticks",     slots[i].period_ticks);
    entry.add("deadline",  slots[i].deadline);
    entry.add("recurring", slots[i].recurring);
    entry.add("expired",   slots[i].expired);
    entry.add("isr_cb",    slots[i].isr_callback);
    entry.add("is_absolute", slots[i].is_absolute);
    entry.add("target_gnss_ns", slots[i].target_gnss_ns);
    entry.add("predicted_dwt", slots[i].predicted_dwt);
    entry.add("prediction_valid", slots[i].prediction_valid);
    entry.add("fire_capture_source",
              fire_capture_source_str(slots[i].fire_capture_source));
    entry.add("fire_gnss_ns",    slots[i].fire_gnss_ns);
    entry.add("fire_dwt_cyccnt", slots[i].fire_dwt_cyccnt);
    entry.add("fire_vclock_raw", slots[i].fire_vclock_raw);
    entry.add("recurring_rearmed_count",          slots[i].recurring_rearmed_count);
    entry.add("recurring_immediate_expire_count", slots[i].recurring_immediate_expire_count);
    entry.add("recurring_catchup_count",          slots[i].recurring_catchup_count);

    timers.add(entry);
  }
  out.add_array("timers", timers);

  return out;
}

// ============================================================================
// QTimer1 vector ownership moved to process_interrupt
// ============================================================================
//
// process_interrupt now owns IRQ_QTIMER1 and dispatches both CH2 (this
// file's timepop_qtimer1_ch2_handler, registered via
// interrupt_register_qtimer1_ch2_handler at init) and CH3 (its own
// internal vclock_cadence_isr).  TimePop is the CH2 hosted client;
// the previous CH3 hosted-client registration mechanism (where TimePop
// hosted process_interrupt's CH3 handler) has been retired.

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT",                 cmd_report                 },
  { nullptr,                  nullptr                    }
};

static const process_vtable_t TIMEPOP_PROCESS = {
  .process_id    = "TIMEPOP",
  .commands      = TIMEPOP_COMMANDS,
  .subscriptions = nullptr,
};

void process_timepop_register(void) {
  process_register("TIMEPOP", &TIMEPOP_PROCESS);
}