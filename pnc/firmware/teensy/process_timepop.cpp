// ============================================================================
// process_timepop.cpp — Foreground scheduler, hardware-fire-driven
// ============================================================================
//
// See process_timepop.h for the architectural overview.  This file
// implements a head-fire model:
//
//   • The head-of-queue is the active timed slot whose effective fire
//     moment is soonest (deadline for foreground slots, or
//     deadline - SPINDRY_APPROACH_TICKS for SpinDry slots).
//   • TimePop arms QTimer1 CH2 via interrupt_schedule_fire_at() to fire
//     at the head's effective fire moment.  Only one slot is armed in
//     hardware at a time.
//   • When CH2 fires, our ISR-context callback runs; it dispatches the
//     head slot (foreground asap for normal slots, synchronous spin for
//     SpinDry), recomputes the new head, and arms CH2 for the next.
//   • Any operation that mutates the timed-slot table (arm, cancel,
//     fire, expire, synthetic-counter zero) calls schedule_next_head()
//     to rearm CH2 against the new head.
//
// TICK subscription remains for two narrow responsibilities:
//   1. Pre-bridge anchoring of slots armed before the first PPS edge.
//   2. A guaranteed 1 kHz asap/alap drain (loop() is the primary drain;
//      TICK is the safety net).
//
// All scheduling math goes through alpha's PPS_VCLOCK slot via
// alpha_pps_vclock_load() and alpha_dwt_cycles_per_second().  No counter
// reads.  No anchor caching.
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

static constexpr uint32_t MAX_SLOTS               = 16;
static constexpr uint32_t MAX_ASAP_SLOTS          = 8;
static constexpr uint32_t MAX_ALAP_SLOTS          = 4;
static constexpr uint32_t FIRE_RING_SIZE          = 8;
static constexpr uint64_t NS_PER_TICK             = 100ULL;
static constexpr int64_t  GNSS_NS_PER_SECOND      = 1000000000LL;

// SpinDry approach window in counter32 ticks.  At 100 ns per tick,
// SPINDRY_APPROACH_NS / NS_PER_TICK = 50 ticks for the default 5 µs.
static constexpr uint32_t SPINDRY_APPROACH_TICKS  =
    (uint32_t)(SPINDRY_APPROACH_NS / NS_PER_TICK);

// Tolerances for diag flags (DWT cycles).  A few cycles is normal spin
// loop iteration noise; anything substantially above that is a real
// preemption or overshoot.
static constexpr int32_t  SPIN_TOLERANCE_CYCLES   = 16;

// SpinDry approach window in DWT cycles.  Exact rational conversion via
// config's DWT_NS_NUM=125 / DWT_NS_DEN=126 (1 DWT cycle = 125/126 ns at
// 1.008 GHz).  For the default 5 µs approach, this is exactly 5040 cycles.
static constexpr uint32_t SPINDRY_APPROACH_DWT_CYCLES =
    (uint32_t)((SPINDRY_APPROACH_NS * DWT_NS_DEN + DWT_NS_NUM / 2) / DWT_NS_NUM);

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
  bool                is_spindry;       // SpinDry contract: ISR-context fire

  // Pre-bridge "deferred anchor": the slot was armed before alpha had a
  // valid PPS_VCLOCK slot, so we couldn't compute a counter32 deadline.
  // The slot carries delay_ticks (relative) and is anchored on the first
  // TICK that delivers a valid counter32_at_edge.  After anchoring,
  // pending_anchor=false and `deadline` is the real counter32 deadline.
  // SpinDry slots can never be pending_anchor — they require a valid
  // bridge to compute target_dwt.
  bool                pending_anchor;
  uint32_t            delay_ticks;

  timepop_handle_t    handle;

  // Deadline in synthetic VCLOCK counter32 ticks.  Compared via
  // (int32_t)(now - deadline) >= 0 to handle 32-bit wrap correctly.
  uint32_t            deadline;

  // SpinDry: predicted DWT at deadline.  Computed fresh inside the
  // approach ISR using the latest bridge; the value stored at arm time
  // is a sanity check only.
  uint32_t            target_dwt;

  // Recurring period in counter32 ticks; 0 if not recurring.
  uint32_t            period_ticks;

  // For absolute-recurring: target gnss_ns advances by period_ns each
  // fire, and deadline is recomputed from alpha's slot.
  int64_t             target_gnss_ns;
  uint64_t            period_ns;

  // Captured at fire time (for foreground delivery via fire ring).
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

// Per-fire pending entry — claimed by the CH2 ISR for normal (foreground)
// timed slot fires, drained by the asap trampoline.  This buffer lets
// the slot be re-armed (recurring) or canceled (one-shot) without losing
// the fire's context.
struct fire_pending_t {
  volatile bool       in_use;
  timepop_callback_t  callback;
  void*               user_data;
  timepop_ctx_t       ctx;
};

// ============================================================================
// State
// ============================================================================

static timepop_slot_t  slots[MAX_SLOTS];
static deferred_slot_t asap_slots[MAX_ASAP_SLOTS];
static deferred_slot_t alap_slots[MAX_ALAP_SLOTS];
static fire_pending_t  fire_ring[FIRE_RING_SIZE];
static uint32_t        next_handle = 1;
static volatile bool   timepop_pending = false;

// ============================================================================
// Diagnostics
// ============================================================================

static volatile uint32_t diag_tick_count             = 0;
static volatile uint32_t diag_timed_fired            = 0;
static volatile uint32_t diag_arm_failures           = 0;
static volatile uint32_t diag_named_replacements     = 0;
static volatile uint32_t diag_asap_armed             = 0;
static volatile uint32_t diag_asap_dispatched        = 0;
static volatile uint32_t diag_alap_armed             = 0;
static volatile uint32_t diag_alap_dispatched        = 0;
static volatile uint32_t diag_dispatch_calls         = 0;
static volatile uint32_t diag_slots_high_water       = 0;
static volatile uint32_t diag_head_recompute_count   = 0;
static volatile uint32_t diag_schedule_fire_arm_calls    = 0;
static volatile uint32_t diag_schedule_fire_cancel_calls = 0;
static volatile uint32_t diag_pre_bridge_anchored   = 0;

static volatile uint32_t diag_spindry_fires_total       = 0;
static volatile uint32_t diag_spindry_fires_preempted   = 0;
static volatile uint32_t diag_spindry_fires_overshot    = 0;
static volatile uint32_t diag_spindry_arm_failures      = 0;
static volatile uint32_t diag_spindry_bridge_lost      = 0;
static volatile uint32_t diag_fire_ring_overflow       = 0;

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

// True if a is "earlier than" b in 32-bit wraparound space.  Valid as
// long as the spread between any two compared deadlines is < 2^31 ticks
// (~214 seconds at 10 MHz) — true for every realistic schedule.
static inline bool deadline_lt(uint32_t a, uint32_t b) {
  return (int32_t)(a - b) < 0;
}

static inline uint32_t ns_to_ticks_clamp(uint64_t ns) {
  uint64_t t = ns / NS_PER_TICK;
  if (t < 1) t = 1;
  if (t > 0x7FFFFFFFULL) t = 0x7FFFFFFFULL;
  return (uint32_t)t;
}

// ============================================================================
// Bridge conversions via alpha's slot
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

// Returns target DWT for `target_gnss_ns`, or 0 on bridge invalid.
// Wrapper around time_gnss_ns_to_dwt to make the failure mode explicit.
static inline uint32_t gnss_ns_to_dwt_safe(int64_t target_gnss_ns) {
  return time_gnss_ns_to_dwt(target_gnss_ns);
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
  bool snapshot_active[MAX_ASAP_SLOTS > MAX_ALAP_SLOTS ?
                       MAX_ASAP_SLOTS : MAX_ALAP_SLOTS] = {};
  for (uint32_t i = 0; i < max_slots; i++) {
    snapshot_active[i] = buf[i].active;
  }

  for (uint32_t i = 0; i < max_slots; i++) {
    if (!snapshot_active[i] || !buf[i].active) continue;

    timepop_ctx_t ctx{};
    ctx.handle = buf[i].handle;

    timepop_diag_t diag{};
    buf[i].callback(&ctx, &diag, buf[i].user_data);
    dispatched_count++;
    buf[i].active = false;
  }
}

// ============================================================================
// Head computation and CH2 arming
// ============================================================================
//
// schedule_next_head is the central rearm primitive.  Call after any
// mutation of the timed slot table to keep CH2 aligned with the new
// head-of-queue.
//
// Caller must hold the critical section (or be in ISR context with no
// concurrent foreground access — same effective protection).

static int find_head_for_schedule(uint32_t& out_effective_fire) {
  int best_idx = -1;
  uint32_t best_fire = 0;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active)        continue;
    if (slots[i].expired)        continue;
    if (slots[i].pending_anchor) continue;

    uint32_t eff = slots[i].deadline;
    if (slots[i].is_spindry) {
      eff = slots[i].deadline - SPINDRY_APPROACH_TICKS;
    }

    if (best_idx < 0 || deadline_lt(eff, best_fire)) {
      best_idx  = (int)i;
      best_fire = eff;
    }
  }

  if (best_idx >= 0) out_effective_fire = best_fire;
  return best_idx;
}

// Forward declaration — used by schedule_next_head in the asap fallback.
static void fire_ring_drain(timepop_ctx_t*, timepop_diag_t*, void*);
static fire_pending_t* claim_fire_pending(void);
static void fire_slot_to_foreground_asap(timepop_slot_t& slot,
                                         uint32_t dwt_at_edge,
                                         uint32_t counter32_at_edge,
                                         int64_t  gnss_ns_at_edge);

static void schedule_next_head(void) {
  // Iterative loop — each pass either arms CH2 for a future head, fires
  // a past head via foreground asap, or finds nothing and disarms.  No
  // recursion: a chain of past heads is processed in a single while loop.
  while (true) {
    diag_head_recompute_count++;

    uint32_t effective_fire = 0;
    const int head_idx = find_head_for_schedule(effective_fire);

    if (head_idx < 0) {
      // No active timed slot — disarm CH2.
      if (interrupt_schedule_fire_armed()) {
        diag_schedule_fire_cancel_calls++;
        interrupt_schedule_fire_cancel();
      }
      return;
    }

    diag_schedule_fire_arm_calls++;
    if (interrupt_schedule_fire_at(effective_fire)) {
      // Successfully armed for a future fire.
      return;
    }

    // Past target — interrupt_schedule_fire_at refused.  Fire via the
    // foreground asap fallback so the slot doesn't wait for the
    // ~7-minute cascade wrap.
    //
    // For SpinDry slots in this branch (deadline already past), we
    // deactivate — the SpinDry contract requires sub-µs landing on a
    // future deadline, and we cannot honor it.
    timepop_slot_t& s = slots[head_idx];

    if (s.is_spindry) {
      diag_spindry_arm_failures++;
      s.active  = false;
      s.expired = true;
      continue;  // try next head
    }

    // Foreground past slot — synthesize a fire context using bridge
    // values where possible, last_known_counter32 otherwise.
    const uint32_t now_c32 = interrupt_last_known_counter32_valid()
                               ? interrupt_last_known_counter32()
                               : 0;

    uint32_t fire_dwt = 0;
    int64_t  fire_gns = 0;
    if (s.is_absolute) {
      const uint32_t computed = gnss_ns_to_dwt_safe(s.target_gnss_ns);
      fire_dwt = (computed != 0) ? computed : ARM_DWT_CYCCNT;
      fire_gns = s.target_gnss_ns;
    } else {
      fire_dwt = ARM_DWT_CYCCNT;
      const int64_t now_g = time_gnss_ns_now();
      fire_gns = (now_g >= 0) ? now_g : 0;
    }

    s.expired = true;
    fire_slot_to_foreground_asap(s, fire_dwt, now_c32, fire_gns);
    diag_timed_fired++;

    // Advance: recurring re-arm or one-shot deactivate.
    if (s.recurring) {
      if (s.is_absolute && s.period_ns > 0) {
        s.target_gnss_ns += (int64_t)s.period_ns;
        const int64_t now_g = time_gnss_ns_now();
        if (now_g > 0 && s.target_gnss_ns <= now_g) {
          const int64_t lag = now_g - s.target_gnss_ns;
          const int64_t jumps = lag / (int64_t)s.period_ns + 1;
          s.target_gnss_ns += jumps * (int64_t)s.period_ns;
        }
        uint32_t next_d = 0;
        if (gnss_ns_to_counter32_deadline(s.target_gnss_ns, next_d)) {
          s.deadline = next_d;
          s.expired  = false;
        } else {
          s.active = false;
        }
      } else if (s.period_ticks > 0) {
        s.deadline += s.period_ticks;
        s.expired = false;
      } else {
        s.active = false;
      }
    } else {
      s.active = false;
    }

    // Loop iterates: find next head (may be the same slot if it was
    // re-armed for a future deadline, or a different slot, or nothing).
  }
}

// ============================================================================
// Fire-pending ring (foreground delivery from CH2 ISR)
// ============================================================================

static fire_pending_t* claim_fire_pending(void) {
  for (uint32_t i = 0; i < FIRE_RING_SIZE; i++) {
    if (!fire_ring[i].in_use) {
      return &fire_ring[i];
    }
  }
  diag_fire_ring_overflow++;
  return nullptr;
}

static void fire_ring_drain(timepop_ctx_t*, timepop_diag_t*, void* ud) {
  auto* fp = static_cast<fire_pending_t*>(ud);
  if (!fp || !fp->in_use) return;

  // Snapshot to local stack so the ring entry can be released BEFORE
  // the user callback runs (in case the callback re-arms and somehow
  // races to claim the same entry — defensive).
  timepop_callback_t cb     = fp->callback;
  void*              udata  = fp->user_data;
  timepop_ctx_t      ctx    = fp->ctx;
  timepop_diag_t     diag{};   // foreground fires: all diag fields zero
  fp->in_use = false;

  if (cb) cb(&ctx, &diag, udata);
}

static void fire_slot_to_foreground_asap(timepop_slot_t& slot,
                                         uint32_t dwt_at_edge,
                                         uint32_t counter32_at_edge,
                                         int64_t  gnss_ns_at_edge) {
  fire_pending_t* fp = claim_fire_pending();
  if (!fp) return;

  fp->callback         = slot.callback;
  fp->user_data        = slot.user_data;
  fp->ctx.handle       = slot.handle;
  fp->ctx.fire_dwt_cyccnt = dwt_at_edge;
  fp->ctx.fire_counter32  = counter32_at_edge;
  fp->ctx.fire_gnss_ns    = gnss_ns_at_edge;
  fp->ctx.target_gnss_ns  = slot.target_gnss_ns;
  fp->ctx.fire_gnss_error_ns =
      (slot.target_gnss_ns > 0) ? (gnss_ns_at_edge - slot.target_gnss_ns) : 0;
  fp->in_use = true;

  // Pass nullptr as the asap name rather than slot.name.  If we passed
  // slot.name, arm_deferred would call retire_existing_named_slots(name)
  // and find the recurring TIMED slot we're firing — killing it before
  // its post-fire re-arm runs.  This bug stops every recurring foreground
  // consumer (transport-rx, transport-tx, cpu-usage, eventbus, ...) on
  // its first fire.  The asap entry is a per-fire delivery vehicle; its
  // identity is its fire_ring slot pointer (passed via user_data), not
  // a name.
  timepop_arm_asap(fire_ring_drain, fp, nullptr);
}

// ============================================================================
// CH2 schedule-fire callback (ISR context)
// ============================================================================
//
// Invoked by process_interrupt's CH2 ISR when the armed compare matches.
// This is the SOLE entry point from hardware fire to TimePop dispatch.
//
// Sequence:
//   1. Find which slot is the head.
//   2. If no head: just return (slot table changed since arm; harmless).
//   3. If SpinDry: spin to target_dwt, populate diag, call user callback
//      synchronously, deactivate slot.
//   4. If foreground: snapshot fire context, fire via asap, advance the
//      slot (recurring re-arm or one-shot deactivate).
//   5. Recompute head and rearm CH2.

static void handle_spindry_fire(timepop_slot_t& s,
                                uint32_t approach_isr_dwt,
                                uint32_t counter32_at_edge);
static void handle_foreground_fire(timepop_slot_t& s,
                                   uint32_t dwt_at_edge,
                                   uint32_t counter32_at_edge);

static void timepop_schedule_fire_callback(uint32_t dwt_at_edge,
                                           uint32_t counter32_at_edge,
                                           void*    /*user_data*/) {
  // Find the head — should be the slot we armed for.  Use the SAME
  // head-selection logic as schedule_next_head so we agree on which
  // slot is firing.
  uint32_t effective_fire = 0;
  const int head_idx = find_head_for_schedule(effective_fire);
  if (head_idx < 0) {
    // Nothing to fire (slot was canceled between arm and fire).  Make
    // sure CH2 stays disarmed.
    return;
  }

  timepop_slot_t& s = slots[head_idx];

  if (s.is_spindry) {
    handle_spindry_fire(s, dwt_at_edge, counter32_at_edge);
  } else {
    handle_foreground_fire(s, dwt_at_edge, counter32_at_edge);
  }

  // Rearm CH2 for the new head.
  schedule_next_head();
}

static void handle_spindry_fire(timepop_slot_t& s,
                                uint32_t approach_isr_dwt,
                                uint32_t counter32_at_edge) {
  // Recompute target_dwt with the freshest bridge.  The value cached at
  // arm time may be slightly off due to rate drift; the fresh bridge
  // (refreshed every PPS) gives sub-cycle accuracy.
  const uint32_t target_dwt = gnss_ns_to_dwt_safe(s.target_gnss_ns);

  if (target_dwt == 0) {
    // Bridge went invalid between arm and fire (very rare — implies an
    // epoch zero happened in this brief window).  Skip the fire.
    diag_spindry_bridge_lost++;
    s.active  = false;
    s.expired = true;
    return;
  }

  const uint32_t approach_target_dwt = target_dwt - SPINDRY_APPROACH_DWT_CYCLES;

  // Spin until ARM_DWT_CYCCNT >= target_dwt.  This is the SpinDry core:
  // tight DWT polling, no critical section (PPS GPIO at priority 0 may
  // preempt — and we WANT it to, so PPS bookkeeping stays accurate).
  //
  // The loop is (cmp, branch) — typically 2-3 cycles per iteration on
  // Cortex-M7 with branch prediction.  Landing precision is ≤ 1 iteration.
  while ((int32_t)(target_dwt - ARM_DWT_CYCCNT) > 0) {
    // Empty body — branch predictor handles this efficiently.
  }
  const uint32_t spin_landed_dwt = ARM_DWT_CYCCNT;

  // Diagnostics
  timepop_diag_t diag{};
  diag.is_spindry           = true;
  diag.approach_isr_dwt     = approach_isr_dwt;
  diag.approach_target_dwt  = approach_target_dwt;
  diag.target_dwt           = target_dwt;
  diag.spin_landed_dwt      = spin_landed_dwt;
  diag.spin_error_cycles    = (int32_t)(spin_landed_dwt - target_dwt);

  // preempt_cycles = (actual approach duration) - (expected approach
  // duration).  Both measured in DWT cycles.
  const int32_t actual_approach   = (int32_t)(spin_landed_dwt - approach_isr_dwt);
  const int32_t expected_approach = (int32_t)(target_dwt      - approach_isr_dwt);
  const int32_t excess            = actual_approach - expected_approach;
  diag.preempt_cycles  = (excess > 0) ? (uint32_t)excess : 0;
  diag.preempted       = (excess                  > SPIN_TOLERANCE_CYCLES);
  diag.deadline_overshot = (diag.spin_error_cycles > SPIN_TOLERANCE_CYCLES);

  if (diag.preempted)         diag_spindry_fires_preempted++;
  if (diag.deadline_overshot) diag_spindry_fires_overshot++;
  diag_spindry_fires_total++;
  diag_timed_fired++;

  // Build context and dispatch synchronously in ISR.
  timepop_ctx_t ctx{};
  ctx.handle             = s.handle;
  ctx.fire_dwt_cyccnt    = spin_landed_dwt;
  ctx.fire_counter32     = counter32_at_edge;  // approach moment counter32
  ctx.fire_gnss_ns       = s.target_gnss_ns;
  ctx.target_gnss_ns     = s.target_gnss_ns;
  ctx.fire_gnss_error_ns = 0;  // by construction we landed on target

  // Mark slot inactive BEFORE invoking the callback so the callback
  // may safely re-arm a new SpinDry on the same name.
  s.active  = false;
  s.expired = true;

  if (s.callback) {
    s.callback(&ctx, &diag, s.user_data);
  }
}

static void handle_foreground_fire(timepop_slot_t& s,
                                   uint32_t dwt_at_edge,
                                   uint32_t counter32_at_edge) {
  // For foreground slots we compute fire_gnss_ns from the firing edge
  // DWT.  This is the same value alpha computes for cadence events.
  const int64_t fire_gnss_ns = time_dwt_to_gnss_ns(dwt_at_edge);

  // Stamp slot's fire facts (kept for backward-compatible inspection).
  s.fire_dwt_cyccnt = dwt_at_edge;
  s.fire_counter32  = counter32_at_edge;
  s.fire_gnss_ns    = fire_gnss_ns;
  s.expired         = true;

  diag_timed_fired++;

  // Dispatch via fire ring → asap → foreground.
  fire_slot_to_foreground_asap(s, dwt_at_edge, counter32_at_edge,
                               (fire_gnss_ns >= 0) ? fire_gnss_ns : 0);

  // Recurring re-arm or one-shot deactivate.
  if (s.recurring) {
    if (s.is_absolute && s.period_ns > 0) {
      s.target_gnss_ns += (int64_t)s.period_ns;
      const int64_t now_g = time_gnss_ns_now();
      if (now_g > 0 && s.target_gnss_ns <= now_g) {
        const int64_t lag = now_g - s.target_gnss_ns;
        const int64_t jumps = lag / (int64_t)s.period_ns + 1;
        s.target_gnss_ns += jumps * (int64_t)s.period_ns;
      }
      uint32_t next_deadline = 0;
      if (gnss_ns_to_counter32_deadline(s.target_gnss_ns, next_deadline)) {
        s.deadline = next_deadline;
        s.expired  = false;
      } else {
        s.active = false;
      }
    } else if (s.period_ticks > 0) {
      s.deadline += s.period_ticks;
      s.expired = false;
    } else {
      s.active = false;
    }
  } else {
    s.active = false;
  }
}

// ============================================================================
// Public arming API
// ============================================================================

static timepop_handle_t arm_absolute_internal(int64_t target_gnss_ns,
                                              bool recurring,
                                              uint64_t period_ns,
                                              bool is_spindry,
                                              timepop_callback_t callback,
                                              void* user_data,
                                              const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  uint32_t deadline = 0;
  if (!gnss_ns_to_counter32_deadline(target_gnss_ns, deadline)) {
    if (is_spindry) diag_spindry_arm_failures++;
    return TIMEPOP_INVALID_HANDLE;
  }

  uint32_t target_dwt = 0;
  if (is_spindry) {
    target_dwt = gnss_ns_to_dwt_safe(target_gnss_ns);
    if (target_dwt == 0) {
      diag_spindry_arm_failures++;
      return TIMEPOP_INVALID_HANDLE;
    }
  }

  const uint32_t saved = critical_enter();
  retire_existing_named_slots(name);

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].recurring      = recurring && period_ns > 0 && !is_spindry;
    slots[i].is_absolute    = true;
    slots[i].is_spindry     = is_spindry;
    slots[i].handle         = h;
    slots[i].deadline       = deadline;
    slots[i].target_dwt     = target_dwt;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].period_ns      = is_spindry ? 0 : period_ns;
    slots[i].period_ticks   = (period_ns > 0 && !is_spindry) ? ns_to_ticks_clamp(period_ns) : 0;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;

    update_slot_high_water();
    schedule_next_head();
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
                                 false,
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
    slots[i].is_spindry     = false;
    slots[i].handle         = h;
    slots[i].period_ns      = 0;
    slots[i].period_ticks   = period_ticks;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;

    update_slot_high_water();
    // No schedule_next_head — this slot can't be the head (pending_anchor).
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
  return arm_absolute_internal(target_gnss_ns, false, 0, false,
                               callback, user_data, name);
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
                               false,
                               callback,
                               user_data,
                               name);
}

timepop_handle_t timepop_arm_at_spindry(int64_t target_gnss_ns,
                                        timepop_callback_t callback,
                                        void* user_data,
                                        const char* name) {
  return arm_absolute_internal(target_gnss_ns, false, 0, true,
                               callback, user_data, name);
}

timepop_handle_t timepop_arm_from_anchor_spindry(int64_t anchor_gnss_ns,
                                                 int64_t offset_gnss_ns,
                                                 timepop_callback_t callback,
                                                 void* user_data,
                                                 const char* name) {
  if (anchor_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  const int64_t target = anchor_gnss_ns + offset_gnss_ns;
  if (target <= 0) return TIMEPOP_INVALID_HANDLE;
  return arm_absolute_internal(target, false, 0, true,
                               callback, user_data, name);
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
  bool found_timed = false;
  bool found_other = false;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].handle == handle) {
      slots[i].active = false; slots[i].expired = false;
      found_timed = true;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].handle == handle) {
      asap_slots[i].active = false; found_other = true;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].handle == handle) {
      alap_slots[i].active = false; found_other = true;
    }
  }
  if (found_timed) schedule_next_head();
  critical_exit(saved);
  return found_timed || found_other;
}

uint32_t timepop_cancel_by_name(const char* name) {
  if (!name || !*name) return 0;
  const uint32_t saved = critical_enter();
  uint32_t n = 0;
  bool found_timed = false;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name && strcmp(slots[i].name, name) == 0) {
      slots[i].active = false; slots[i].expired = false; n++;
      found_timed = true;
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
  if (found_timed) schedule_next_head();
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
// just discarded.  We re-anchor in place; see process_timepop.h for the
// slot-class breakdown.

void timepop_handle_synthetic_counter_zero(void) {
  uint32_t saved = critical_enter();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;

    slots[i].expired = false;

    if (slots[i].is_spindry) {
      // SpinDry slots reference an absolute GNSS deadline.  Recompute
      // both deadline (counter32) and target_dwt against the new
      // post-zero bridge.  If the bridge is invalid, deactivate
      // (SpinDry can't tolerate an invalid bridge).
      uint32_t new_deadline = 0;
      uint32_t new_target_dwt = 0;
      const bool d_ok  = gnss_ns_to_counter32_deadline(slots[i].target_gnss_ns, new_deadline);
      if (d_ok) new_target_dwt = gnss_ns_to_dwt_safe(slots[i].target_gnss_ns);

      if (d_ok && new_target_dwt != 0) {
        slots[i].deadline   = new_deadline;
        slots[i].target_dwt = new_target_dwt;
      } else {
        slots[i].active = false;
        diag_spindry_arm_failures++;
      }
    } else if (slots[i].is_absolute) {
      uint32_t new_deadline = 0;
      if (gnss_ns_to_counter32_deadline(slots[i].target_gnss_ns, new_deadline)) {
        slots[i].deadline       = new_deadline;
        slots[i].pending_anchor = false;
      } else {
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
    }
  }

  schedule_next_head();
  critical_exit(saved);
}

// ============================================================================
// TICK-driven housekeeping
// ============================================================================
//
// In the head-fire model, TICK no longer scans timed slots — the CH2
// hardware fire is the dispatch mechanism.  TICK retains two roles:
//
//   1. Pre-bridge anchoring: slots stashed with pending_anchor=true now
//      get their counter32 deadline.  After anchoring, the head may
//      have changed; rearm CH2.
//   2. Periodic asap/alap drain at 1 kHz (loop()'s timepop_dispatch is
//      the primary drain; TICK is the safety net).

static void tick_callback(const interrupt_event_t& event, void*) {
  diag_tick_count++;

  // Phase 1: ASAP drain
  dispatch_deferred_phase(asap_slots, MAX_ASAP_SLOTS, diag_asap_dispatched);

  // Phase 2: Pre-bridge anchoring
  bool anchored_any = false;
  const uint32_t now = event.counter32_at_edge;
  {
    const uint32_t saved = critical_enter();
    for (uint32_t i = 0; i < MAX_SLOTS; i++) {
      if (slots[i].active && slots[i].pending_anchor) {
        slots[i].deadline       = now + slots[i].delay_ticks;
        slots[i].pending_anchor = false;
        anchored_any = true;
        diag_pre_bridge_anchored++;
      }
    }
    if (anchored_any) schedule_next_head();
    critical_exit(saved);
  }

  // Phase 3: ALAP drain
  dispatch_deferred_phase(alap_slots, MAX_ALAP_SLOTS, diag_alap_dispatched);
}

// ============================================================================
// loop() dispatch entrypoint
// ============================================================================
//
// Foreground asap/alap drain.  The CH2 ISR arms an asap slot for each
// foreground timed fire; this drains it (and any other asap/alap slots
// that piled up since last dispatch).

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
  for (uint32_t i = 0; i < MAX_SLOTS; i++)        slots[i] = {};
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++)   asap_slots[i] = {};
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++)   alap_slots[i] = {};
  for (uint32_t i = 0; i < FIRE_RING_SIZE; i++)   fire_ring[i] = {};
  next_handle = 1;
  timepop_pending = false;

  // Register our schedule-fire callback with process_interrupt.  The
  // callback runs in QTimer1 CH2 ISR context when CH2's compare matches.
  interrupt_schedule_register(timepop_schedule_fire_callback, nullptr);

  // Subscribe to TICK for asap/alap drain heartbeat and pre-bridge
  // anchoring.  Process_interrupt will dispatch tick_callback in
  // foreground via the deferred trampoline.
  interrupt_subscription_t tick_sub{};
  tick_sub.kind     = interrupt_subscriber_kind_t::TICK;
  tick_sub.on_event = tick_callback;
  interrupt_subscribe(tick_sub);
  interrupt_start(interrupt_subscriber_kind_t::TICK);
}

// ============================================================================
// REPORT command — state visibility for debugging
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  // Live counts
  p.add("active_count",        timepop_active_count());
  p.add("slots_high_water",    diag_slots_high_water);

  // Per-slot inspection — useful for "what is TimePop holding right now"
  uint32_t spindry_active = 0;
  uint32_t pending_anchor_active = 0;
  uint32_t recurring_active = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;
    if (slots[i].is_spindry)     spindry_active++;
    if (slots[i].pending_anchor) pending_anchor_active++;
    if (slots[i].recurring)      recurring_active++;
  }
  p.add("spindry_active",        spindry_active);
  p.add("pending_anchor_active", pending_anchor_active);
  p.add("recurring_active",      recurring_active);

  // Tick / fire counters
  p.add("tick_count",       diag_tick_count);
  p.add("timed_fired",      diag_timed_fired);

  // ASAP / ALAP
  p.add("asap_armed",          diag_asap_armed);
  p.add("asap_dispatched",     diag_asap_dispatched);
  p.add("alap_armed",          diag_alap_armed);
  p.add("alap_dispatched",     diag_alap_dispatched);
  p.add("dispatch_calls",      diag_dispatch_calls);

  // Lifecycle counters
  p.add("named_replacements",  diag_named_replacements);
  p.add("arm_failures",        diag_arm_failures);
  p.add("pre_bridge_anchored", diag_pre_bridge_anchored);

  // Head-of-queue scheduler
  p.add("head_recompute_count",       diag_head_recompute_count);
  p.add("schedule_fire_arm_calls",    diag_schedule_fire_arm_calls);
  p.add("schedule_fire_cancel_calls", diag_schedule_fire_cancel_calls);
  p.add("schedule_fire_armed_now",    interrupt_schedule_fire_armed());
  p.add("fire_ring_overflow",         diag_fire_ring_overflow);

  // SpinDry-specific diagnostics
  p.add("spindry_fires_total",     diag_spindry_fires_total);
  p.add("spindry_fires_preempted", diag_spindry_fires_preempted);
  p.add("spindry_fires_overshot",  diag_spindry_fires_overshot);
  p.add("spindry_arm_failures",    diag_spindry_arm_failures);
  p.add("spindry_bridge_lost",     diag_spindry_bridge_lost);

  // Approach window in nanoseconds and ticks (for empirical tuning)
  p.add("spindry_approach_ns",     (uint32_t)SPINDRY_APPROACH_NS);
  p.add("spindry_approach_ticks",  SPINDRY_APPROACH_TICKS);

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