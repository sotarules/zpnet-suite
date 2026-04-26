// ============================================================================
// process_timepop.cpp — Two-mode scheduler: TICK stable + SpinDry precision
// ============================================================================
//
// TimePop now has two deliberately separate timing modes:
//
//   • TICK mode:
//       Default for timepop_arm(), timepop_arm_at(), and
//       timepop_arm_from_anchor().  The 1 kHz TICK subscriber scans these
//       slots and invokes due callbacks in foreground callback context.
//       This restores the original stable scheduler behavior for ordinary
//       services and keeps recurring 1 kHz work off the CH2 precision lane.
//
//   • SPINDRY precision mode:
//       Opt-in only via timepop_arm_at_spindry() and
//       timepop_arm_from_anchor_spindry().  These slots are scheduled by
//       process_interrupt's QTimer1 CH2 schedule-fire lane.  On approach
//       fire, TimePop spins on DWT until the target and invokes the user
//       callback in ISR context.
//
// The CH2 head-of-queue scheduler ignores all ordinary TICK-mode slots.
// This prevents transport/event/cpu recurring services from flooding the
// ASAP queue with hardware-fire delivery contexts.
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

static constexpr uint32_t MAX_SLOTS          = 16;
static constexpr uint32_t MAX_ASAP_SLOTS     = 8;
static constexpr uint32_t MAX_ALAP_SLOTS     = 4;

static constexpr uint32_t SPINDRY_APPROACH_TICKS =
    (uint32_t)(SPINDRY_APPROACH_NS / NS_PER_TICK);

static constexpr int32_t  SPIN_TOLERANCE_CYCLES = 16;

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

enum class timepop_slot_mode_t : uint8_t {
  TICK    = 0,
  SPINDRY = 1,
};

static const char* slot_mode_str(timepop_slot_mode_t mode) {
  switch (mode) {
    case timepop_slot_mode_t::SPINDRY: return "SPINDRY";
    default:                           return "TICK";
  }
}

struct timepop_slot_t {
  bool                active;
  bool                expired;
  bool                recurring;
  bool                is_absolute;
  bool                pending_anchor;
  timepop_slot_mode_t mode;

  uint32_t            delay_ticks;
  timepop_handle_t    handle;

  // Deadline in synthetic VCLOCK counter32 ticks.
  uint32_t            deadline;

  // SpinDry target DWT sanity value.  Recomputed at fire time.
  uint32_t            target_dwt;

  uint32_t            period_ticks;

  // For absolute slots: target GNSS ns.  For absolute recurring TICK
  // slots, this advances by period_ns on each fire.
  int64_t             target_gnss_ns;
  uint64_t            period_ns;

  // Last fire facts for reports.
  uint32_t            fire_dwt_cyccnt;
  uint32_t            fire_counter32;
  int64_t             fire_gnss_ns;

  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;
};

struct deferred_slot_t {
  bool                active;
  bool                has_ctx;
  timepop_handle_t    handle;
  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;
  timepop_ctx_t       ctx;
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

static volatile uint32_t diag_tick_count             = 0;
static volatile uint32_t diag_tick_timed_fired       = 0;
static volatile uint32_t diag_timed_fired            = 0;
static volatile uint32_t diag_arm_failures           = 0;
static volatile uint32_t diag_named_replacements     = 0;
static volatile uint32_t diag_asap_armed             = 0;
static volatile uint32_t diag_asap_dispatched        = 0;
static volatile uint32_t diag_alap_armed             = 0;
static volatile uint32_t diag_alap_dispatched        = 0;
static volatile uint32_t diag_dispatch_calls         = 0;
static volatile uint32_t diag_slots_high_water       = 0;
static volatile uint32_t diag_precision_head_recompute_count = 0;
static volatile uint32_t diag_schedule_fire_arm_calls        = 0;
static volatile uint32_t diag_schedule_fire_cancel_calls     = 0;
static volatile uint32_t diag_pre_bridge_anchored   = 0;

static volatile uint32_t diag_spindry_fires_total     = 0;
static volatile uint32_t diag_spindry_fires_preempted = 0;
static volatile uint32_t diag_spindry_fires_overshot  = 0;
static volatile uint32_t diag_spindry_arm_failures    = 0;
static volatile uint32_t diag_spindry_bridge_lost     = 0;
static volatile uint32_t diag_fire_arm_failures       = 0;  // retained for report compatibility

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
// Deferred ASAP/ALAP arming and dispatch
// ============================================================================

static timepop_handle_t arm_deferred_with_ctx(deferred_slot_t* buf,
                                              uint32_t max_slots,
                                              volatile uint32_t& armed_count,
                                              timepop_callback_t callback,
                                              void* user_data,
                                              const char* name,
                                              const timepop_ctx_t* ctx_or_null) {
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
    if (ctx_or_null) {
      buf[i].has_ctx = true;
      buf[i].ctx     = *ctx_or_null;
      buf[i].ctx.handle = h;
    }

    armed_count++;
    timepop_pending = true;
    critical_exit(saved);
    return h;
  }

  diag_arm_failures++;
  critical_exit(saved);
  return TIMEPOP_INVALID_HANDLE;
}

static timepop_handle_t arm_deferred(deferred_slot_t* buf,
                                     uint32_t max_slots,
                                     volatile uint32_t& armed_count,
                                     timepop_callback_t callback,
                                     void* user_data,
                                     const char* name) {
  return arm_deferred_with_ctx(buf, max_slots, armed_count,
                               callback, user_data, name, nullptr);
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

    timepop_callback_t cb     = buf[i].callback;
    void*              udata  = buf[i].user_data;
    timepop_ctx_t      ctx;
    if (buf[i].has_ctx) {
      ctx = buf[i].ctx;
    } else {
      ctx = timepop_ctx_t{};
      ctx.handle = buf[i].handle;
    }

    timepop_diag_t diag{};
    cb(&ctx, &diag, udata);
    dispatched_count++;
    buf[i].active = false;
  }
}

// ============================================================================
// Precision / SpinDry head computation and CH2 arming
// ============================================================================

static int find_precision_head_for_schedule(uint32_t& out_effective_fire) {
  int best_idx = -1;
  uint32_t best_fire = 0;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active)        continue;
    if (slots[i].expired)        continue;
    if (slots[i].pending_anchor) continue;
    if (slots[i].mode != timepop_slot_mode_t::SPINDRY) continue;

    const uint32_t eff = slots[i].deadline - SPINDRY_APPROACH_TICKS;
    if (best_idx < 0 || deadline_lt(eff, best_fire)) {
      best_idx  = (int)i;
      best_fire = eff;
    }
  }

  if (best_idx >= 0) out_effective_fire = best_fire;
  return best_idx;
}

static void schedule_next_precision_head(void) {
  while (true) {
    diag_precision_head_recompute_count++;

    uint32_t effective_fire = 0;
    const int head_idx = find_precision_head_for_schedule(effective_fire);

    if (head_idx < 0) {
      if (interrupt_schedule_fire_armed()) {
        diag_schedule_fire_cancel_calls++;
        interrupt_schedule_fire_cancel();
      }
      return;
    }

    diag_schedule_fire_arm_calls++;
    if (interrupt_schedule_fire_at(effective_fire)) {
      return;
    }

    // Precision/SpinDry missed its safe approach window.  Deactivate it;
    // this is a precision miss, not something to punt into foreground.
    slots[head_idx].active  = false;
    slots[head_idx].expired = true;
    diag_spindry_arm_failures++;
  }
}

// ============================================================================
// TICK-mode fire handling
// ============================================================================

static void fire_tick_slot(timepop_slot_t& s,
                           uint32_t now_counter32,
                           uint32_t fire_dwt,
                           int64_t fire_gnss_ns) {
  s.fire_dwt_cyccnt = fire_dwt;
  s.fire_counter32  = now_counter32;
  s.fire_gnss_ns    = fire_gnss_ns;
  s.expired         = true;

  timepop_ctx_t ctx{};
  ctx.handle             = s.handle;
  ctx.fire_dwt_cyccnt    = fire_dwt;
  ctx.fire_counter32     = now_counter32;
  ctx.fire_gnss_ns       = fire_gnss_ns;
  ctx.target_gnss_ns     = s.target_gnss_ns;
  ctx.fire_gnss_error_ns =
      (s.target_gnss_ns > 0) ? (fire_gnss_ns - s.target_gnss_ns) : 0;

  timepop_diag_t diag{};
  if (s.callback) {
    s.callback(&ctx, &diag, s.user_data);
  }

  diag_tick_timed_fired++;
  diag_timed_fired++;

  if (s.recurring) {
    if (s.is_absolute && s.period_ns > 0) {
      s.target_gnss_ns += (int64_t)s.period_ns;
      const int64_t now_gnss = time_gnss_ns_now();
      if (now_gnss > 0 && s.target_gnss_ns <= now_gnss) {
        const int64_t lag = now_gnss - s.target_gnss_ns;
        const int64_t jumps = lag / (int64_t)s.period_ns + 1;
        s.target_gnss_ns += jumps * (int64_t)s.period_ns;
      }
      uint32_t next_deadline = 0;
      if (gnss_ns_to_counter32_deadline(s.target_gnss_ns, next_deadline)) {
        s.deadline = next_deadline;
        s.expired  = false;
      } else {
        // Keep the slot alive but wait for a future TICK/bridge to anchor.
        s.pending_anchor = true;
        s.delay_ticks = s.period_ticks;
        s.expired = false;
      }
    } else if (s.period_ticks > 0) {
      s.deadline += s.period_ticks;
      if (deadline_expired(s.deadline, now_counter32)) {
        s.deadline = now_counter32 + s.period_ticks;
      }
      s.expired = false;
    } else {
      s.active = false;
    }
  } else {
    s.active = false;
  }
}

// ============================================================================
// CH2 schedule-fire callback (ISR context) — SpinDry only
// ============================================================================

static void handle_spindry_fire(timepop_slot_t& s,
                                uint32_t approach_isr_dwt,
                                uint32_t counter32_at_edge) {
  const uint32_t target_dwt = gnss_ns_to_dwt_safe(s.target_gnss_ns);

  if (target_dwt == 0) {
    diag_spindry_bridge_lost++;
    s.active  = false;
    s.expired = true;
    return;
  }

  const uint32_t approach_target_dwt = target_dwt - SPINDRY_APPROACH_DWT_CYCLES;

  while ((int32_t)(target_dwt - ARM_DWT_CYCCNT) > 0) {
    // SpinDry: intentional empty DWT spin.
  }
  const uint32_t spin_landed_dwt = ARM_DWT_CYCCNT;

  timepop_diag_t diag{};
  diag.is_spindry          = true;
  diag.approach_isr_dwt    = approach_isr_dwt;
  diag.approach_target_dwt = approach_target_dwt;
  diag.target_dwt          = target_dwt;
  diag.spin_landed_dwt     = spin_landed_dwt;
  diag.spin_error_cycles   = (int32_t)(spin_landed_dwt - target_dwt);

  const int32_t actual_approach   = (int32_t)(spin_landed_dwt - approach_isr_dwt);
  const int32_t expected_approach = (int32_t)(target_dwt      - approach_isr_dwt);
  const int32_t excess            = actual_approach - expected_approach;
  diag.preempt_cycles     = (excess > 0) ? (uint32_t)excess : 0;
  diag.preempted          = (excess > SPIN_TOLERANCE_CYCLES);
  diag.deadline_overshot  = (diag.spin_error_cycles > SPIN_TOLERANCE_CYCLES);

  if (diag.preempted)         diag_spindry_fires_preempted++;
  if (diag.deadline_overshot) diag_spindry_fires_overshot++;
  diag_spindry_fires_total++;
  diag_timed_fired++;

  timepop_ctx_t ctx{};
  ctx.handle             = s.handle;
  ctx.fire_dwt_cyccnt    = spin_landed_dwt;
  ctx.fire_counter32     = counter32_at_edge;
  ctx.fire_gnss_ns       = s.target_gnss_ns;
  ctx.target_gnss_ns     = s.target_gnss_ns;
  ctx.fire_gnss_error_ns = 0;

  s.fire_dwt_cyccnt = spin_landed_dwt;
  s.fire_counter32  = counter32_at_edge;
  s.fire_gnss_ns    = s.target_gnss_ns;

  // Single-shot precision slot.  Mark inactive before callback so the
  // callback may re-arm using the same name.
  s.active  = false;
  s.expired = true;

  if (s.callback) {
    s.callback(&ctx, &diag, s.user_data);
  }
}

static void timepop_schedule_fire_callback(uint32_t dwt_at_edge,
                                           uint32_t counter32_at_edge,
                                           void*    /*user_data*/) {
  uint32_t effective_fire = 0;
  const int head_idx = find_precision_head_for_schedule(effective_fire);
  if (head_idx < 0) return;

  timepop_slot_t& s = slots[head_idx];
  if (s.mode == timepop_slot_mode_t::SPINDRY) {
    handle_spindry_fire(s, dwt_at_edge, counter32_at_edge);
  }

  schedule_next_precision_head();
}

// ============================================================================
// Public arming API
// ============================================================================

static timepop_handle_t allocate_slot(timepop_slot_mode_t mode,
                                      bool recurring,
                                      bool is_absolute,
                                      bool pending_anchor,
                                      uint32_t delay_ticks,
                                      uint32_t deadline,
                                      uint32_t target_dwt,
                                      uint32_t period_ticks,
                                      int64_t target_gnss_ns,
                                      uint64_t period_ns,
                                      timepop_callback_t callback,
                                      void* user_data,
                                      const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  retire_existing_named_slots(name);

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].recurring      = recurring;
    slots[i].is_absolute    = is_absolute;
    slots[i].pending_anchor = pending_anchor;
    slots[i].mode           = mode;
    slots[i].delay_ticks    = delay_ticks;
    slots[i].handle         = h;
    slots[i].deadline       = deadline;
    slots[i].target_dwt     = target_dwt;
    slots[i].period_ticks   = period_ticks;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].period_ns      = period_ns;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;

    update_slot_high_water();
    return h;
  }

  diag_arm_failures++;
  return TIMEPOP_INVALID_HANDLE;
}

timepop_handle_t timepop_arm(uint64_t delay_gnss_ns,
                             bool recurring,
                             timepop_callback_t callback,
                             void* user_data,
                             const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  const uint32_t delay_ticks  = ns_to_ticks_clamp(delay_gnss_ns);
  const uint32_t period_ticks = recurring ? delay_ticks : 0;

  const uint32_t saved = critical_enter();

  // Always TICK mode.  Relative foreground timers deliberately anchor
  // on the next TICK rather than last_known_counter32.  This avoids using
  // a stale pre-zero last_known value during START/ZERO transitions and
  // restores the original "next scheduler heartbeat establishes now"
  // behavior.  The cost is at most one TICK of phase ambiguity, which is
  // exactly the contract for TICK-mode timers.
  const timepop_handle_t h =
      allocate_slot(timepop_slot_mode_t::TICK,
                    recurring,
                    false,
                    true,
                    delay_ticks,
                    0,
                    0,
                    period_ticks,
                    0,
                    0,
                    callback,
                    user_data,
                    name);

  critical_exit(saved);
  return h;
}

static timepop_handle_t arm_tick_absolute_internal(int64_t target_gnss_ns,
                                                   bool recurring,
                                                   uint64_t period_ns,
                                                   timepop_callback_t callback,
                                                   void* user_data,
                                                   const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  uint32_t deadline = 0;
  const bool have_deadline = gnss_ns_to_counter32_deadline(target_gnss_ns, deadline);

  const uint32_t period_ticks = (recurring && period_ns > 0)
                                  ? ns_to_ticks_clamp(period_ns)
                                  : 0;

  const uint32_t saved = critical_enter();
  const timepop_handle_t h =
      allocate_slot(timepop_slot_mode_t::TICK,
                    recurring && period_ns > 0,
                    true,
                    !have_deadline,
                    period_ticks,
                    have_deadline ? deadline : 0,
                    0,
                    period_ticks,
                    target_gnss_ns,
                    recurring ? period_ns : 0,
                    callback,
                    user_data,
                    name);
  critical_exit(saved);
  return h;
}

timepop_handle_t timepop_arm_at(int64_t target_gnss_ns,
                                bool recurring,
                                timepop_callback_t callback,
                                void* user_data,
                                const char* name) {
  (void)recurring;  // absolute one-shots only via this API
  return arm_tick_absolute_internal(target_gnss_ns, false, 0,
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
  return arm_tick_absolute_internal(target,
                                    recurring,
                                    recurring ? NS_PER_SECOND : 0,
                                    callback,
                                    user_data,
                                    name);
}

timepop_handle_t timepop_arm_at_spindry(int64_t target_gnss_ns,
                                        timepop_callback_t callback,
                                        void* user_data,
                                        const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  uint32_t deadline = 0;
  if (!gnss_ns_to_counter32_deadline(target_gnss_ns, deadline)) {
    diag_spindry_arm_failures++;
    return TIMEPOP_INVALID_HANDLE;
  }

  const uint32_t target_dwt = gnss_ns_to_dwt_safe(target_gnss_ns);
  if (target_dwt == 0) {
    diag_spindry_arm_failures++;
    return TIMEPOP_INVALID_HANDLE;
  }

  const uint32_t saved = critical_enter();
  const timepop_handle_t h =
      allocate_slot(timepop_slot_mode_t::SPINDRY,
                    false,
                    true,
                    false,
                    0,
                    deadline,
                    target_dwt,
                    0,
                    target_gnss_ns,
                    0,
                    callback,
                    user_data,
                    name);
  if (h != TIMEPOP_INVALID_HANDLE) {
    schedule_next_precision_head();
  }
  critical_exit(saved);
  return h;
}

timepop_handle_t timepop_arm_from_anchor_spindry(int64_t anchor_gnss_ns,
                                                 int64_t offset_gnss_ns,
                                                 timepop_callback_t callback,
                                                 void* user_data,
                                                 const char* name) {
  if (anchor_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  const int64_t target = anchor_gnss_ns + offset_gnss_ns;
  if (target <= 0) return TIMEPOP_INVALID_HANDLE;
  return timepop_arm_at_spindry(target, callback, user_data, name);
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

  bool found_precision = false;
  bool found_any = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].handle == handle) {
      if (slots[i].mode == timepop_slot_mode_t::SPINDRY) found_precision = true;
      slots[i].active = false;
      slots[i].expired = false;
      found_any = true;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].handle == handle) {
      asap_slots[i].active = false;
      found_any = true;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].handle == handle) {
      alap_slots[i].active = false;
      found_any = true;
    }
  }

  if (found_precision) schedule_next_precision_head();
  critical_exit(saved);
  return found_any;
}

uint32_t timepop_cancel_by_name(const char* name) {
  if (!name || !*name) return 0;
  const uint32_t saved = critical_enter();

  uint32_t n = 0;
  bool found_precision = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name && strcmp(slots[i].name, name) == 0) {
      if (slots[i].mode == timepop_slot_mode_t::SPINDRY) found_precision = true;
      slots[i].active = false;
      slots[i].expired = false;
      n++;
    }
  }
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].name && strcmp(asap_slots[i].name, name) == 0) {
      asap_slots[i].active = false;
      n++;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].name && strcmp(alap_slots[i].name, name) == 0) {
      alap_slots[i].active = false;
      n++;
    }
  }

  if (found_precision) schedule_next_precision_head();
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

void timepop_handle_synthetic_counter_zero(void) {
  const uint32_t saved = critical_enter();

  bool touched_precision = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;

    slots[i].expired = false;

    if (slots[i].mode == timepop_slot_mode_t::SPINDRY) {
      touched_precision = true;
      uint32_t new_deadline = 0;
      uint32_t new_target_dwt = 0;
      const bool d_ok = gnss_ns_to_counter32_deadline(slots[i].target_gnss_ns, new_deadline);
      if (d_ok) new_target_dwt = gnss_ns_to_dwt_safe(slots[i].target_gnss_ns);

      if (d_ok && new_target_dwt != 0) {
        slots[i].deadline   = new_deadline;
        slots[i].target_dwt = new_target_dwt;
      } else {
        slots[i].active = false;
        diag_spindry_arm_failures++;
      }
      continue;
    }

    if (slots[i].is_absolute) {
      uint32_t new_deadline = 0;
      if (gnss_ns_to_counter32_deadline(slots[i].target_gnss_ns, new_deadline)) {
        slots[i].deadline       = new_deadline;
        slots[i].pending_anchor = false;
      } else {
        slots[i].pending_anchor = true;
        slots[i].delay_ticks = (slots[i].recurring && slots[i].period_ticks > 0)
                                  ? slots[i].period_ticks
                                  : 0;
      }
    } else {
      slots[i].pending_anchor = true;
      if (slots[i].recurring && slots[i].period_ticks > 0) {
        slots[i].delay_ticks = slots[i].period_ticks;
      }
    }
  }

  if (touched_precision) schedule_next_precision_head();

  critical_exit(saved);
}

// ============================================================================
// TICK-driven scheduler
// ============================================================================

static void tick_callback(const interrupt_event_t& event, void*) {
  diag_tick_count++;

  // Phase 1: drain ASAP first, preserving the old TimePop rhythm.
  dispatch_deferred_phase(asap_slots, MAX_ASAP_SLOTS, diag_asap_dispatched);

  const uint32_t now = event.counter32_at_edge;

  // Phase 2a: anchor pending TICK slots.
  {
    const uint32_t saved = critical_enter();
    for (uint32_t i = 0; i < MAX_SLOTS; i++) {
      if (!slots[i].active) continue;
      if (slots[i].mode != timepop_slot_mode_t::TICK) continue;
      if (!slots[i].pending_anchor) continue;

      if (slots[i].is_absolute) {
        uint32_t d = 0;
        if (gnss_ns_to_counter32_deadline(slots[i].target_gnss_ns, d)) {
          slots[i].deadline = d;
        } else {
          slots[i].deadline = now + slots[i].delay_ticks;
        }
      } else {
        slots[i].deadline = now + slots[i].delay_ticks;
      }

      slots[i].pending_anchor = false;
      diag_pre_bridge_anchored++;
    }
    critical_exit(saved);
  }

  // Phase 2b: scan and fire due TICK slots.
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;
    if (slots[i].mode != timepop_slot_mode_t::TICK) continue;
    if (slots[i].expired) continue;
    if (slots[i].pending_anchor) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

    fire_tick_slot(slots[i], now, event.dwt_at_edge, event.gnss_ns_at_edge);
  }

  // Phase 3: drain ALAP last.
  dispatch_deferred_phase(alap_slots, MAX_ALAP_SLOTS, diag_alap_dispatched);
}

// ============================================================================
// loop() dispatch entrypoint
// ============================================================================

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
  // No hardware to bootstrap; process_interrupt owns QTimer setup.
}

void timepop_init(void) {
  for (uint32_t i = 0; i < MAX_SLOTS; i++)      slots[i] = {};
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) asap_slots[i] = {};
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) alap_slots[i] = {};
  next_handle = 1;
  timepop_pending = false;

  // Register precision/SpinDry schedule-fire callback.  Ordinary timed
  // services do not use this lane.
  interrupt_schedule_register(timepop_schedule_fire_callback, nullptr);

  interrupt_subscription_t tick_sub{};
  tick_sub.kind     = interrupt_subscriber_kind_t::TICK;
  tick_sub.on_event = tick_callback;
  interrupt_subscribe(tick_sub);
  interrupt_start(interrupt_subscriber_kind_t::TICK);
}

// ============================================================================
// REPORT command
// ============================================================================

struct timed_slot_snap_t {
  bool                active;
  bool                expired;
  bool                recurring;
  bool                is_absolute;
  bool                pending_anchor;
  timepop_slot_mode_t mode;
  uint32_t            delay_ticks;
  timepop_handle_t    handle;
  uint32_t            deadline;
  uint32_t            target_dwt;
  uint32_t            period_ticks;
  int64_t             target_gnss_ns;
  uint64_t            period_ns;
  uint32_t            fire_dwt_cyccnt;
  uint32_t            fire_counter32;
  int64_t             fire_gnss_ns;
  const char*         name;
  uintptr_t           callback_addr;
  uintptr_t           user_data_addr;
};

struct deferred_slot_snap_t {
  bool                active;
  bool                has_ctx;
  timepop_handle_t    handle;
  const char*         name;
  uintptr_t           callback_addr;
  uintptr_t           user_data_addr;
  timepop_ctx_t       ctx;
};

static Payload cmd_report(const Payload&) {
  Payload out;

  timed_slot_snap_t    slot_snap[MAX_SLOTS];
  deferred_slot_snap_t asap_snap[MAX_ASAP_SLOTS];
  deferred_slot_snap_t alap_snap[MAX_ALAP_SLOTS];

  uint32_t active_count          = 0;
  uint32_t tick_active           = 0;
  uint32_t spindry_active        = 0;
  uint32_t pending_anchor_active = 0;
  uint32_t recurring_active      = 0;
  uint32_t asap_active_count     = 0;
  uint32_t asap_with_ctx_count   = 0;
  uint32_t alap_active_count     = 0;

  {
    const uint32_t saved = critical_enter();

    for (uint32_t i = 0; i < MAX_SLOTS; i++) {
      const timepop_slot_t& s = slots[i];
      timed_slot_snap_t& d = slot_snap[i];
      d.active          = s.active;
      d.expired         = s.expired;
      d.recurring       = s.recurring;
      d.is_absolute     = s.is_absolute;
      d.pending_anchor  = s.pending_anchor;
      d.mode            = s.mode;
      d.delay_ticks     = s.delay_ticks;
      d.handle          = s.handle;
      d.deadline        = s.deadline;
      d.target_dwt      = s.target_dwt;
      d.period_ticks    = s.period_ticks;
      d.target_gnss_ns  = s.target_gnss_ns;
      d.period_ns       = s.period_ns;
      d.fire_dwt_cyccnt = s.fire_dwt_cyccnt;
      d.fire_counter32  = s.fire_counter32;
      d.fire_gnss_ns    = s.fire_gnss_ns;
      d.name            = s.name;
      d.callback_addr   = (uintptr_t)s.callback;
      d.user_data_addr  = (uintptr_t)s.user_data;

      if (s.active) {
        active_count++;
        if (s.mode == timepop_slot_mode_t::TICK) tick_active++;
        if (s.mode == timepop_slot_mode_t::SPINDRY) spindry_active++;
        if (s.pending_anchor) pending_anchor_active++;
        if (s.recurring) recurring_active++;
      }
    }

    for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
      const deferred_slot_t& s = asap_slots[i];
      deferred_slot_snap_t& d = asap_snap[i];
      d.active         = s.active;
      d.has_ctx        = s.has_ctx;
      d.handle         = s.handle;
      d.name           = s.name;
      d.callback_addr  = (uintptr_t)s.callback;
      d.user_data_addr = (uintptr_t)s.user_data;
      d.ctx            = s.ctx;
      if (s.active) {
        asap_active_count++;
        if (s.has_ctx) asap_with_ctx_count++;
      }
    }

    for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
      const deferred_slot_t& s = alap_slots[i];
      deferred_slot_snap_t& d = alap_snap[i];
      d.active         = s.active;
      d.has_ctx        = s.has_ctx;
      d.handle         = s.handle;
      d.name           = s.name;
      d.callback_addr  = (uintptr_t)s.callback;
      d.user_data_addr = (uintptr_t)s.user_data;
      d.ctx            = s.ctx;
      if (s.active) alap_active_count++;
    }

    critical_exit(saved);
  }

  uint32_t now_c32 = 0;
  bool now_c32_valid = false;
  {
    const alpha_pps_vclock_slot_t a = alpha_pps_vclock_load();
    if (a.sequence != 0) {
      const uint32_t cps = alpha_dwt_cycles_per_second();
      if (cps != 0) {
        const uint32_t dwt_now = ARM_DWT_CYCCNT;
        const uint32_t dwt_elapsed = dwt_now - a.dwt_at_edge;
        const uint64_t ticks_elapsed =
            ((uint64_t)dwt_elapsed * 10000000ULL + (uint64_t)cps / 2) /
            (uint64_t)cps;
        if (ticks_elapsed <= 0xFFFFFFFFULL) {
          now_c32 = a.counter32_at_edge + (uint32_t)ticks_elapsed;
          now_c32_valid = true;
        }
      }
    }
    if (!now_c32_valid && interrupt_last_known_counter32_valid()) {
      now_c32 = interrupt_last_known_counter32();
      now_c32_valid = true;
    }
  }

  out.add("active_count",          active_count);
  out.add("tick_active",           tick_active);
  out.add("spindry_active",        spindry_active);
  out.add("pending_anchor_active", pending_anchor_active);
  out.add("recurring_active",      recurring_active);
  out.add("slots_high_water",      diag_slots_high_water);
  out.add("slots_max",             (uint32_t)MAX_SLOTS);

  out.add("asap_active_count",   asap_active_count);
  out.add("asap_with_ctx_count", asap_with_ctx_count);
  out.add("asap_slots_max",      (uint32_t)MAX_ASAP_SLOTS);
  out.add("alap_active_count",   alap_active_count);
  out.add("alap_slots_max",      (uint32_t)MAX_ALAP_SLOTS);

  out.add("now_c32",       now_c32);
  out.add("now_c32_valid", now_c32_valid);

  out.add("tick_count",        diag_tick_count);
  out.add("timed_fired",       diag_timed_fired);
  out.add("tick_timed_fired",  diag_tick_timed_fired);

  out.add("asap_armed",      diag_asap_armed);
  out.add("asap_dispatched", diag_asap_dispatched);
  out.add("alap_armed",      diag_alap_armed);
  out.add("alap_dispatched", diag_alap_dispatched);
  out.add("dispatch_calls",  diag_dispatch_calls);

  out.add("named_replacements",  diag_named_replacements);
  out.add("arm_failures",        diag_arm_failures);
  out.add("fire_arm_failures",   diag_fire_arm_failures);
  out.add("pre_bridge_anchored", diag_pre_bridge_anchored);

  out.add("precision_head_recompute_count", diag_precision_head_recompute_count);
  // Backward-compatible names for existing dashboards/scripts.
  out.add("head_recompute_count",       diag_precision_head_recompute_count);
  out.add("schedule_fire_arm_calls",    diag_schedule_fire_arm_calls);
  out.add("schedule_fire_cancel_calls", diag_schedule_fire_cancel_calls);
  out.add("schedule_fire_armed_now",    interrupt_schedule_fire_armed());

  out.add("spindry_fires_total",     diag_spindry_fires_total);
  out.add("spindry_fires_preempted", diag_spindry_fires_preempted);
  out.add("spindry_fires_overshot",  diag_spindry_fires_overshot);
  out.add("spindry_arm_failures",    diag_spindry_arm_failures);
  out.add("spindry_bridge_lost",     diag_spindry_bridge_lost);
  out.add("spindry_approach_ns",     (uint32_t)SPINDRY_APPROACH_NS);
  out.add("spindry_approach_ticks",  SPINDRY_APPROACH_TICKS);

  PayloadArray timers;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slot_snap[i].active) continue;
    const timed_slot_snap_t& s = slot_snap[i];

    Payload entry;
    entry.add("slot",            i);
    entry.add("handle",          s.handle);
    entry.add("name",            s.name ? s.name : "");
    entry.add("mode",            slot_mode_str(s.mode));
    entry.add("period_ns",       s.period_ns);
    entry.add("period_ticks",    s.period_ticks);
    entry.add("deadline",        s.deadline);
    entry.add("recurring",       s.recurring);
    entry.add("expired",         s.expired);
    entry.add("is_absolute",     s.is_absolute);
    entry.add("is_spindry",      s.mode == timepop_slot_mode_t::SPINDRY);
    entry.add("pending_anchor",  s.pending_anchor);
    entry.add("delay_ticks",     s.delay_ticks);
    entry.add("target_gnss_ns",  s.target_gnss_ns);
    entry.add("target_dwt",      s.target_dwt);
    entry.add("fire_dwt_cyccnt", s.fire_dwt_cyccnt);
    entry.add("fire_counter32",  s.fire_counter32);
    entry.add("fire_gnss_ns",    s.fire_gnss_ns);
    entry.add("callback_addr",   (uint64_t)s.callback_addr);
    entry.add("user_data_addr",  (uint64_t)s.user_data_addr);

    if (!s.pending_anchor && now_c32_valid) {
      const int32_t delta = (int32_t)(s.deadline - now_c32);
      entry.add("deadline_minus_now_ticks", delta);
    }

    timers.add(entry);
  }
  out.add_array("timers", timers);

  PayloadArray asap;
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (!asap_snap[i].active) continue;
    const deferred_slot_snap_t& s = asap_snap[i];

    Payload entry;
    entry.add("slot",           i);
    entry.add("handle",         s.handle);
    entry.add("name",           s.name ? s.name : "");
    entry.add("has_ctx",        s.has_ctx);
    entry.add("callback_addr",  (uint64_t)s.callback_addr);
    entry.add("user_data_addr", (uint64_t)s.user_data_addr);

    if (s.has_ctx) {
      entry.add("ctx_fire_dwt_cyccnt",    s.ctx.fire_dwt_cyccnt);
      entry.add("ctx_fire_counter32",     s.ctx.fire_counter32);
      entry.add("ctx_fire_gnss_ns",       s.ctx.fire_gnss_ns);
      entry.add("ctx_target_gnss_ns",     s.ctx.target_gnss_ns);
      entry.add("ctx_fire_gnss_error_ns", s.ctx.fire_gnss_error_ns);
    }

    asap.add(entry);
  }
  out.add_array("asap", asap);

  PayloadArray alap;
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (!alap_snap[i].active) continue;
    const deferred_slot_snap_t& s = alap_snap[i];

    Payload entry;
    entry.add("slot",           i);
    entry.add("handle",         s.handle);
    entry.add("name",           s.name ? s.name : "");
    entry.add("has_ctx",        s.has_ctx);
    entry.add("callback_addr",  (uint64_t)s.callback_addr);
    entry.add("user_data_addr", (uint64_t)s.user_data_addr);

    if (s.has_ctx) {
      entry.add("ctx_fire_dwt_cyccnt",    s.ctx.fire_dwt_cyccnt);
      entry.add("ctx_fire_counter32",     s.ctx.fire_counter32);
      entry.add("ctx_fire_gnss_ns",       s.ctx.fire_gnss_ns);
      entry.add("ctx_target_gnss_ns",     s.ctx.target_gnss_ns);
      entry.add("ctx_fire_gnss_error_ns", s.ctx.fire_gnss_error_ns);
    }

    alap.add(entry);
  }
  out.add_array("alap", alap);

  return out;
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
