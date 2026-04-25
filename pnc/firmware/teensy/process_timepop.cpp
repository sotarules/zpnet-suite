// ============================================================================
// process_timepop.cpp — Foreground scheduler, TICK-driven
// ============================================================================
//
// See process_timepop.h for the contract.  TimePop owns no hardware.
// It subscribes to process_interrupt's TICK kind; on every TICK event
// (1 kHz cadence) it runs in foreground:
//
//   1. Drain the ASAP queue.
//   2. Scan timed slots; fire any expired.
//   3. Drain the ALAP queue.
//
// Timed deadlines live in synthetic VCLOCK counter32 ticks.  "Now" is the
// counter32_at_edge of the TICK event firing this scan.  Scheduling
// resolution is 1 ms.
//
// All gnss_ns ↔ counter32 conversions go through alpha's PPS_VCLOCK slot
// via alpha_pps_vclock_load() and alpha_dwt_cycles_per_second().  No
// counter reads.  No anchor caching.
// ============================================================================

#include "process_timepop.h"

#include "config.h"
#include "process.h"
#include "payload.h"
#include "publish.h"
#include "process_interrupt.h"
#include "time.h"
#include "timepop.h"
#include "debug.h"

#include <Arduino.h>
#include "imxrt.h"

#include <string.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t MAX_SLOTS       = 16;
static constexpr uint32_t MAX_ASAP_SLOTS  = 8;
static constexpr uint32_t MAX_ALAP_SLOTS  = 4;
static constexpr uint64_t NS_PER_TICK     = 100ULL;
static constexpr int64_t  GNSS_NS_PER_SECOND = 1000000000LL;

// ============================================================================
// Critical-section helpers
// ============================================================================

static inline uint32_t critical_enter(void) {
  uint32_t primask;
  __asm__ volatile ("MRS %0, primask" : "=r" (primask));
  __asm__ volatile ("CPSID i" ::: "memory");
  return primask;
}

static inline void critical_exit(uint32_t primask) {
  __asm__ volatile ("MSR primask, %0" :: "r" (primask) : "memory");
}

// ============================================================================
// Slot types
// ============================================================================

struct timepop_slot_t {
  bool                active;
  bool                expired;
  bool                recurring;
  bool                is_absolute;

  // Pre-bridge "deferred anchor": the slot was armed before alpha had a
  // valid PPS_VCLOCK slot, so we couldn't compute a counter32 deadline.
  // The slot carries delay_ticks (relative) and is anchored on the first
  // TICK that delivers a valid counter32_at_edge.  After anchoring,
  // pending_anchor=false and `deadline` is the real counter32 deadline.
  bool                pending_anchor;
  uint32_t            delay_ticks;

  timepop_handle_t    handle;

  // Deadline in synthetic VCLOCK counter32 ticks.  Compared via
  // (int32_t)(now - deadline) >= 0 to handle 32-bit wrap correctly.
  uint32_t            deadline;

  // Recurring period in counter32 ticks; 0 if not recurring.
  uint32_t            period_ticks;

  // For absolute-recurring: target gnss_ns advances by period_ns each
  // fire, and deadline is recomputed from alpha's slot.
  int64_t             target_gnss_ns;
  uint64_t            period_ns;

  // Captured at fire time (filled by tick scan).
  uint32_t            fire_dwt_cyccnt;
  uint32_t            fire_counter32;
  int64_t             fire_gnss_ns;

  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;
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

static timepop_slot_t  slots[MAX_SLOTS];
static deferred_slot_t asap_slots[MAX_ASAP_SLOTS];
static deferred_slot_t alap_slots[MAX_ALAP_SLOTS];
static uint32_t        next_handle = 1;
static volatile bool   timepop_pending = false;

// ============================================================================
// Diagnostics
// ============================================================================

static volatile uint32_t diag_tick_count          = 0;
static volatile uint32_t diag_timed_fired         = 0;
static volatile uint32_t diag_arm_failures        = 0;
static volatile uint32_t diag_named_replacements  = 0;
static volatile uint32_t diag_asap_armed          = 0;
static volatile uint32_t diag_asap_dispatched     = 0;
static volatile uint32_t diag_alap_armed          = 0;
static volatile uint32_t diag_alap_dispatched     = 0;
static volatile uint32_t diag_dispatch_calls      = 0;
static volatile uint32_t diag_slots_high_water    = 0;

static inline void update_slot_high_water(void) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) if (slots[i].active) n++;
  if (n > diag_slots_high_water) diag_slots_high_water = n;
}

// ============================================================================
// Deadline arithmetic
// ============================================================================

static inline bool deadline_expired(uint32_t deadline, uint32_t now) {
  return (int32_t)(now - deadline) >= 0;
}

static inline uint32_t ns_to_ticks_clamp(uint64_t ns) {
  uint64_t t = ns / NS_PER_TICK;
  if (t < 1) t = 1;
  if (t > 0x7FFFFFFFULL) t = 0x7FFFFFFFULL;
  return (uint32_t)t;
}

// ============================================================================
// gnss_ns → counter32 conversion via alpha's slot
// ============================================================================

static bool gnss_ns_to_counter32_deadline(int64_t target_gnss_ns,
                                          uint32_t& out_deadline) {
  const alpha_pps_vclock_slot_t a = alpha_pps_vclock_load();
  if (a.sequence == 0) return false;

  const int64_t ns_from_anchor = target_gnss_ns - a.gnss_ns_at_edge;
  if (ns_from_anchor < 0) {
    // Target is before anchor — express it as a wrapped value (the
    // expiration check handles wrap correctly).  Compute the negative
    // delta in ticks and subtract.
    const uint64_t neg_ns = (uint64_t)(-ns_from_anchor);
    const uint32_t neg_ticks = (uint32_t)(neg_ns / NS_PER_TICK);
    out_deadline = a.counter32_at_edge - neg_ticks;
    return true;
  }

  const uint64_t ticks64 = (uint64_t)ns_from_anchor / NS_PER_TICK;
  if (ticks64 > 0xFFFFFFFFULL) return false;
  out_deadline = a.counter32_at_edge + (uint32_t)ticks64;
  return true;
}

// ============================================================================
// Named-slot retirement
// ============================================================================

static bool retire_existing_named_slots(const char* name) {
  if (!name || !*name) return false;
  bool retired = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name && strcmp(slots[i].name, name) == 0) {
      slots[i].active  = false;
      slots[i].expired = false;
      retired = true;
      diag_named_replacements++;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].name && strcmp(asap_slots[i].name, name) == 0) {
      asap_slots[i].active = false;
      retired = true;
      diag_named_replacements++;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].name && strcmp(alap_slots[i].name, name) == 0) {
      alap_slots[i].active = false;
      retired = true;
      diag_named_replacements++;
    }
  }
  return retired;
}

// ============================================================================
// Deferred (asap/alap) arming and dispatch
// ============================================================================

static timepop_handle_t arm_deferred(deferred_slot_t* buf,
                                     uint32_t max_slots,
                                     volatile uint32_t& armed_count,
                                     timepop_callback_t callback,
                                     void* user_data,
                                     const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  const uint32_t saved = critical_enter();

  retire_existing_named_slots(name);

  for (uint32_t i = 0; i < max_slots; i++) {
    if (buf[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    buf[i] = {};
    buf[i].active     = true;
    buf[i].handle     = h;
    buf[i].callback   = callback;
    buf[i].user_data  = user_data;
    buf[i].name       = name;

    armed_count++;
    timepop_pending = true;
    critical_exit(saved);
    return h;
  }

  diag_arm_failures++;
  critical_exit(saved);
  return TIMEPOP_INVALID_HANDLE;
}

static void dispatch_deferred_phase(deferred_slot_t* buf,
                                    uint32_t max_slots,
                                    volatile uint32_t& dispatched_count) {
  bool snapshot_active[MAX_ASAP_SLOTS > MAX_ALAP_SLOTS ? MAX_ASAP_SLOTS : MAX_ALAP_SLOTS] = {};
  for (uint32_t i = 0; i < max_slots; i++) {
    snapshot_active[i] = buf[i].active;
  }

  for (uint32_t i = 0; i < max_slots; i++) {
    if (!snapshot_active[i] || !buf[i].active) continue;

    timepop_ctx_t ctx{};
    ctx.handle = buf[i].handle;

    buf[i].callback(&ctx, nullptr, buf[i].user_data);
    dispatched_count++;
    buf[i].active = false;
  }
}

// ============================================================================
// Public arming API
// ============================================================================

static timepop_handle_t arm_absolute_internal(int64_t target_gnss_ns,
                                              bool recurring,
                                              uint64_t period_ns,
                                              timepop_callback_t callback,
                                              void* user_data,
                                              const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  uint32_t deadline = 0;
  if (!gnss_ns_to_counter32_deadline(target_gnss_ns, deadline)) {
    return TIMEPOP_INVALID_HANDLE;
  }

  const uint32_t saved = critical_enter();
  retire_existing_named_slots(name);

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].recurring      = recurring && period_ns > 0;
    slots[i].is_absolute    = true;
    slots[i].handle         = h;
    slots[i].deadline       = deadline;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].period_ns      = period_ns;
    slots[i].period_ticks   = (period_ns > 0) ? ns_to_ticks_clamp(period_ns) : 0;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;

    update_slot_high_water();
    critical_exit(saved);
    return h;
  }

  diag_arm_failures++;
  critical_exit(saved);
  return TIMEPOP_INVALID_HANDLE;
}

timepop_handle_t timepop_arm(uint64_t delay_gnss_ns,
                             bool recurring,
                             timepop_callback_t callback,
                             void* user_data,
                             const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  // Compute delay_ticks once; valid even before the bridge is up.
  const uint32_t delay_ticks = ns_to_ticks_clamp(delay_gnss_ns);
  const uint64_t period_ns   = recurring ? delay_gnss_ns : 0;
  const uint32_t period_ticks = recurring ? delay_ticks : 0;

  // Try the bridge-aware path first.  If it succeeds, the slot is fully
  // anchored from boot.  If it fails (typical at boot before first PPS),
  // fall through to the pre-bridge deferred-anchor path.
  const int64_t now_gnss_ns = time_gnss_ns_now();
  if (now_gnss_ns >= 0) {
    const int64_t target_gnss_ns = now_gnss_ns + (int64_t)delay_gnss_ns;
    return arm_absolute_internal(target_gnss_ns,
                                 recurring,
                                 period_ns,
                                 callback,
                                 user_data,
                                 name);
  }

  // Pre-bridge: arm a deferred-anchor slot.  The first TICK after boot
  // anchors it with deadline = tick.counter32_at_edge + delay_ticks.
  // Recurring slots keep period_ticks for post-fire rearming; they do
  // NOT use the absolute (gnss_ns) recurring path because the bridge
  // isn't authoritative yet.
  const uint32_t saved = critical_enter();
  retire_existing_named_slots(name);

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].pending_anchor = true;
    slots[i].delay_ticks    = delay_ticks;
    slots[i].recurring      = recurring;
    slots[i].is_absolute    = false;
    slots[i].handle         = h;
    slots[i].period_ns      = 0;
    slots[i].period_ticks   = period_ticks;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;

    update_slot_high_water();
    critical_exit(saved);
    return h;
  }

  diag_arm_failures++;
  critical_exit(saved);
  return TIMEPOP_INVALID_HANDLE;
}

timepop_handle_t timepop_arm_at(int64_t target_gnss_ns,
                                bool recurring,
                                timepop_callback_t callback,
                                void* user_data,
                                const char* name) {
  (void)recurring;  // absolute one-shots only via this API
  return arm_absolute_internal(target_gnss_ns, false, 0, callback, user_data, name);
}

timepop_handle_t timepop_arm_from_anchor(int64_t anchor_gnss_ns,
                                         int64_t offset_gnss_ns,
                                         bool recurring,
                                         timepop_callback_t callback,
                                         void* user_data,
                                         const char* name) {
  if (anchor_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  const int64_t target = anchor_gnss_ns + offset_gnss_ns;
  if (target <= 0) return TIMEPOP_INVALID_HANDLE;
  return arm_absolute_internal(target,
                               recurring,
                               recurring ? GNSS_NS_PER_SECOND : 0,
                               callback,
                               user_data,
                               name);
}

timepop_handle_t timepop_arm_asap(timepop_callback_t callback,
                                  void* user_data,
                                  const char* name) {
  return arm_deferred(asap_slots, MAX_ASAP_SLOTS, diag_asap_armed,
                      callback, user_data, name);
}

timepop_handle_t timepop_arm_alap(timepop_callback_t callback,
                                  void* user_data,
                                  const char* name) {
  return arm_deferred(alap_slots, MAX_ALAP_SLOTS, diag_alap_armed,
                      callback, user_data, name);
}

bool timepop_cancel(timepop_handle_t handle) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;
  const uint32_t saved = critical_enter();
  bool found = false;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].handle == handle) {
      slots[i].active = false; slots[i].expired = false; found = true;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].handle == handle) {
      asap_slots[i].active = false; found = true;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].handle == handle) {
      alap_slots[i].active = false; found = true;
    }
  }
  critical_exit(saved);
  return found;
}

uint32_t timepop_cancel_by_name(const char* name) {
  if (!name || !*name) return 0;
  const uint32_t saved = critical_enter();
  uint32_t n = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name && strcmp(slots[i].name, name) == 0) {
      slots[i].active = false; slots[i].expired = false; n++;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].name && strcmp(asap_slots[i].name, name) == 0) {
      asap_slots[i].active = false; n++;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].name && strcmp(alap_slots[i].name, name) == 0) {
      alap_slots[i].active = false; n++;
    }
  }
  critical_exit(saved);
  return n;
}

uint32_t timepop_active_count(void) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) if (slots[i].active) n++;
  return n;
}

// ============================================================================
// Synthetic-counter zero handler
// ============================================================================
//
// Called by alpha after interrupt_synthetic_counters_zero() (which runs
// inside alpha_install_new_epoch_from_pps_event).  All active slots have
// deadlines anchored to the OLD synthetic-counter timeline, which was
// just discarded.  Without re-anchoring, tick-based recurring slots
// (transport RX/TX, etc.) wait for the synthetic counter to wrap back
// around to their old deadlines — minutes for slots anchored long ago.
//
// We re-anchor in place:
//
//   • is_absolute slots:
//       Recompute deadline from target_gnss_ns via the freshly-published
//       alpha slot.  Because alpha just published with counter32_at_edge=0
//       and gnss_ns_at_edge=0 (the install edge is the new origin), the
//       bridge produces a sensible post-zero deadline.  If the bridge is
//       still invalid (sequence==0, defensive), fall back to pending_anchor.
//
//   • Non-absolute (tick) slots:
//       Set pending_anchor=true.  The next TICK will compute the deadline
//       as counter32_at_edge + delay_ticks on the new timeline.  For
//       recurring slots we set delay_ticks=period_ticks so they resume
//       firing at their normal cadence; one-shots retain their original
//       delay_ticks so the intended interval is preserved.

void timepop_handle_synthetic_counter_zero(void) {
  uint32_t saved = critical_enter();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;

    slots[i].expired = false;

    if (slots[i].is_absolute) {
      uint32_t new_deadline = 0;
      if (gnss_ns_to_counter32_deadline(slots[i].target_gnss_ns, new_deadline)) {
        slots[i].deadline       = new_deadline;
        slots[i].pending_anchor = false;
      } else {
        // Bridge not yet authoritative — defer to next TICK.
        slots[i].pending_anchor = true;
        slots[i].delay_ticks    = (slots[i].recurring && slots[i].period_ticks > 0)
                                    ? slots[i].period_ticks
                                    : 0;
      }
    } else {
      // Tick-based slot — re-anchor on next TICK.
      slots[i].pending_anchor = true;
      if (slots[i].recurring && slots[i].period_ticks > 0) {
        slots[i].delay_ticks = slots[i].period_ticks;
      }
      // For one-shots, leave delay_ticks at its original value so the
      // intended interval is preserved (relative to "next TICK after zero").
    }
  }

  critical_exit(saved);
}

// ============================================================================
// Tick-driven dispatch
// ============================================================================
//
// Called from the TICK subscriber (in foreground via the deferred
// trampoline).  Drains asap, scans timed slots against the tick's
// counter32, fires expired slots, then drains alap.

static void tick_callback(const interrupt_event_t& event, void*) {
  diag_tick_count++;

  // Phase 1: ASAP
  dispatch_deferred_phase(asap_slots, MAX_ASAP_SLOTS, diag_asap_dispatched);

  // Phase 2a: anchor any pre-bridge deferred-anchor slots.  This is the
  // first TICK they've seen, so we now know a real counter32; turn the
  // delay_ticks into a concrete deadline.
  const uint32_t now = event.counter32_at_edge;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].pending_anchor) {
      slots[i].deadline       = now + slots[i].delay_ticks;
      slots[i].pending_anchor = false;
    }
  }

  // Phase 2b: timed slots — scan and fire any expired.
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (slots[i].pending_anchor)              continue;  // can't happen but safe
    if (!deadline_expired(slots[i].deadline, now)) continue;

    // Capture the firing facts from the TICK event.
    slots[i].fire_dwt_cyccnt = event.dwt_at_edge;
    slots[i].fire_counter32  = now;
    slots[i].fire_gnss_ns    = event.gnss_ns_at_edge;
    slots[i].expired         = true;

    timepop_ctx_t ctx{};
    ctx.handle             = slots[i].handle;
    ctx.fire_dwt_cyccnt    = slots[i].fire_dwt_cyccnt;
    ctx.fire_counter32     = slots[i].fire_counter32;
    ctx.fire_gnss_ns       = slots[i].fire_gnss_ns;
    ctx.target_gnss_ns     = slots[i].target_gnss_ns;
    ctx.fire_gnss_error_ns =
        (slots[i].target_gnss_ns > 0)
            ? (slots[i].fire_gnss_ns - slots[i].target_gnss_ns)
            : 0;

    slots[i].callback(&ctx, nullptr, slots[i].user_data);
    diag_timed_fired++;

    // Recurring: advance and re-arm.  One-shot: deactivate.
    if (slots[i].recurring) {
      if (slots[i].is_absolute && slots[i].period_ns > 0) {
        slots[i].target_gnss_ns += (int64_t)slots[i].period_ns;
        // Catch-up: if we fell behind, jump forward to the next future
        // boundary.
        const int64_t now_gnss = time_gnss_ns_now();
        if (now_gnss > 0 && slots[i].target_gnss_ns <= now_gnss) {
          const int64_t lag = now_gnss - slots[i].target_gnss_ns;
          const int64_t jumps = lag / (int64_t)slots[i].period_ns + 1;
          slots[i].target_gnss_ns += jumps * (int64_t)slots[i].period_ns;
        }
        uint32_t next_deadline = 0;
        if (gnss_ns_to_counter32_deadline(slots[i].target_gnss_ns, next_deadline)) {
          slots[i].deadline = next_deadline;
          slots[i].expired  = false;
        } else {
          slots[i].active = false;
        }
      } else if (slots[i].period_ticks > 0) {
        slots[i].deadline += slots[i].period_ticks;
        if (deadline_expired(slots[i].deadline, now)) {
          slots[i].deadline = now + slots[i].period_ticks;
        }
        slots[i].expired = false;
      } else {
        slots[i].active = false;
      }
    } else {
      slots[i].active = false;
    }
  }

  // Phase 3: ALAP
  dispatch_deferred_phase(alap_slots, MAX_ALAP_SLOTS, diag_alap_dispatched);
}

// ============================================================================
// loop() dispatch entrypoint
// ============================================================================
//
// Some callers (process_interrupt's deferred trampoline) drive the asap
// queue via timepop_arm_asap; foreground loop() calls timepop_dispatch()
// to drain it.  Under the tick-driven model the timed slots are scanned
// inside the TICK callback, but we still drain asap here so callers that
// arm asap from a non-TICK-event ISR get prompt service in foreground.

void timepop_dispatch(void) {
  if (!timepop_pending) return;
  timepop_pending = false;
  diag_dispatch_calls++;

  dispatch_deferred_phase(asap_slots, MAX_ASAP_SLOTS, diag_asap_dispatched);
  dispatch_deferred_phase(alap_slots, MAX_ALAP_SLOTS, diag_alap_dispatched);
}

// ============================================================================
// Init
// ============================================================================

void timepop_bootstrap(void) {
  // No hardware to bootstrap; process_interrupt owns all QTimer1 setup.
}

void timepop_init(void) {
  for (uint32_t i = 0; i < MAX_SLOTS; i++) slots[i] = {};
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) asap_slots[i] = {};
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) alap_slots[i] = {};
  next_handle = 1;
  timepop_pending = false;

  // Subscribe to TICK.  Process_interrupt will dispatch tick_callback in
  // foreground via the deferred trampoline.
  interrupt_subscription_t tick_sub{};
  tick_sub.kind     = interrupt_subscriber_kind_t::TICK;
  tick_sub.on_event = tick_callback;
  interrupt_subscribe(tick_sub);
  interrupt_start(interrupt_subscriber_kind_t::TICK);
}

// ============================================================================
// REPORT command
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("active_count",        timepop_active_count());
  p.add("slots_high_water",    diag_slots_high_water);
  p.add("tick_count",          diag_tick_count);
  p.add("timed_fired",         diag_timed_fired);
  p.add("asap_armed",          diag_asap_armed);
  p.add("asap_dispatched",     diag_asap_dispatched);
  p.add("alap_armed",          diag_alap_armed);
  p.add("alap_dispatched",     diag_alap_dispatched);
  p.add("named_replacements",  diag_named_replacements);
  p.add("arm_failures",        diag_arm_failures);
  p.add("dispatch_calls",      diag_dispatch_calls);
  return p;
}

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr    },
};

static const process_vtable_t TIMEPOP_PROCESS = {
  .process_id    = "TIMEPOP",
  .commands      = TIMEPOP_COMMANDS,
  .subscriptions = nullptr,
};

void process_timepop_register(void) {
  process_register("TIMEPOP", &TIMEPOP_PROCESS);
}