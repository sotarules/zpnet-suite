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
//  • critical ISR recurring    — timepop_arm_recurring_isr(period_ns, ...)
//    The callback and recurring rearm happen inside the CH2 IRQ pass before
//    schedule_next() selects the next compare.  Use only for tiny substrate
//    maintenance callbacks that TimePop itself depends on for safe scheduling.
//  • counter-base anchored ISR recurring
//      timepop_arm_recurring_isr_from_base_counter32(base_gnss_ns,
//                                                    base_counter32,
//                                                    period_ns, ...)
//    The first and later deadlines are computed from the supplied VCLOCK
//    counter32 base by gear arithmetic.  This substrate path does not ask
//    "what time is it now?" at arm time and does not require a current
//    time_anchor_snapshot() to be valid.
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
//   All timed slots reached by the same physical CH2 event receive the same
//   fire_vclock_raw, fire_dwt_cyccnt, and fire_gnss_ns.  Equality with the
//   compare target is the normal fire condition, not a suspicious edge case.
//   Callback order is logical only; it must never re-author the physical
//   timing facts.
//
//   TimePop authors fire_gnss_ns from VCLOCK arithmetic using the event's
//   process_interrupt-authored counter32 identity.  Any DWT-derived GNSS
//   coordinate in the interrupt payload is diagnostic/cross-check material
//   and is never allowed to redefine the scheduled event's GNSS identity.
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
static constexpr uint32_t ONE_HZ_TICKS = 10000000U;
static constexpr const char* WITNESS_SCHEDULER_NAME = "WITNESS_SCHEDULER";


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

enum class timepop_arm_source_t : uint8_t {
  NONE = 0,
  RELATIVE_ARM,
  RELATIVE_ARM_PHASE_LOCKED,
  ABSOLUTE_ARM,
  ANCHORED_RECURRING_ISR_ARM,
  EXACT_ARM,
  DISPATCH_ABSOLUTE_REARM,
  DISPATCH_RELATIVE_REARM,
  DISPATCH_PHASE_LOCKED_REARM,
  ISR_REARM,
  ISR_REARM_FALLBACK,
  EPOCH_REARM,
  EPOCH_REARM_FALLBACK,
};

static const char* arm_source_str(timepop_arm_source_t source) {
  switch (source) {
    case timepop_arm_source_t::RELATIVE_ARM: return "RELATIVE_ARM";
    case timepop_arm_source_t::RELATIVE_ARM_PHASE_LOCKED: return "RELATIVE_ARM_PHASE_LOCKED";
    case timepop_arm_source_t::ABSOLUTE_ARM: return "ABSOLUTE_ARM";
    case timepop_arm_source_t::ANCHORED_RECURRING_ISR_ARM: return "ANCHORED_RECURRING_ISR_ARM";
    case timepop_arm_source_t::EXACT_ARM: return "EXACT_ARM";
    case timepop_arm_source_t::DISPATCH_ABSOLUTE_REARM: return "DISPATCH_ABSOLUTE_REARM";
    case timepop_arm_source_t::DISPATCH_RELATIVE_REARM: return "DISPATCH_RELATIVE_REARM";
    case timepop_arm_source_t::DISPATCH_PHASE_LOCKED_REARM: return "DISPATCH_PHASE_LOCKED_REARM";
    case timepop_arm_source_t::ISR_REARM: return "ISR_REARM";
    case timepop_arm_source_t::ISR_REARM_FALLBACK: return "ISR_REARM_FALLBACK";
    case timepop_arm_source_t::EPOCH_REARM: return "EPOCH_REARM";
    case timepop_arm_source_t::EPOCH_REARM_FALLBACK: return "EPOCH_REARM_FALLBACK";
    default: return "NONE";
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
  bool                rearm_in_isr;

  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;

  uint32_t            recurring_rearmed_count;
  bool                recurring_base_fixed;
  bool                recurring_base_counter32_fixed;
  uint32_t            recurring_base_counter32;
  uint64_t            recurring_next_index;
  uint32_t            recurring_last_skipped_intervals;
  uint32_t            recurring_total_skipped_intervals;
  uint32_t            recurring_immediate_expire_count;
  uint32_t            recurring_catchup_count;
  int64_t             recurring_base_gnss_ns;
  uint64_t            recurring_period_gnss_ns;

  uint32_t            arm_vclock_raw;
  uint32_t            arm_delta_ticks;
  uint32_t            first_expire_now;
  bool                first_expire_recorded;

  // schedule_next() catch-up diagnostics.  These counters identify slots whose
  // deadlines were discovered already behind the ambient VCLOCK observation in
  // foreground/scheduler context instead of being reached by the normal CH2 IRQ
  // expiry path.  They are report-only; they do not alter scheduling policy.
  uint32_t            schedule_next_expired_count;
  uint32_t            schedule_next_late_max_ticks;
  uint32_t            schedule_next_last_late_ticks;
  uint32_t            schedule_next_last_now;
  uint32_t            schedule_next_last_deadline;
  uint32_t            schedule_next_last_dwt;

  // IRQ scan diagnostics.  These tell whether a slot was actually visible and
  // expirable when CH2 scanned the timed slot table.  If a slot is later caught
  // by schedule_next(), these fields show whether the IRQ path ever saw it due.
  uint32_t            irq_reached_count;
  uint32_t            irq_expired_by_irq_count;
  uint32_t            irq_late_max_ticks;
  uint32_t            irq_last_now;
  uint32_t            irq_last_dwt;
  uint32_t            irq_last_late_ticks;
  bool                irq_last_exact;

  // Arm/rearm diagnostics.  These answer the question: was this slot already
  // overdue when it was born or re-authored?  For deterministic absolute paths
  // the ambient VCLOCK read is diagnostic only; it does not participate in
  // deadline authorship.
  timepop_arm_source_t last_arm_source;
  uint32_t            last_arm_now;
  uint32_t            last_arm_deadline;
  int32_t             last_arm_delta_ticks;
  int64_t             last_arm_target_gnss_ns;
  uint32_t            last_arm_dwt;
  bool                last_arm_already_past;
  bool                last_arm_had_now;
  uint32_t            arm_already_past_count;
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
static volatile uint32_t diag_isr_recurring_rearmed = 0;
static volatile uint32_t diag_isr_recurring_rearm_failures = 0;

static volatile uint32_t diag_schedule_next_calls_total = 0;
static volatile uint32_t diag_schedule_next_calls_from_dispatch = 0;
static volatile uint32_t diag_schedule_next_calls_from_other = 0;

// Normal precision fires should be authored by IRQ_CH2.  If schedule_next()
// discovers an already-past timed slot, TimePop still records and surfaces
// the fact instead of silently pretending it was a clean hardware fire.
static volatile uint32_t diag_schedule_next_expired_passes = 0;
static volatile uint32_t diag_schedule_next_expired_slots = 0;
static volatile uint32_t diag_schedule_next_late_max_ticks = 0;
static volatile uint32_t diag_schedule_next_last_expired_slot = UINT32_MAX;
static volatile uint32_t diag_schedule_next_last_expired_handle = 0;
static volatile const char* diag_schedule_next_last_expired_name = nullptr;
static volatile uint32_t diag_schedule_next_last_expired_deadline = 0;
static volatile uint32_t diag_schedule_next_last_expired_now = 0;
static volatile uint32_t diag_schedule_next_last_expired_late_ticks = 0;
static volatile uint32_t diag_schedule_next_last_expired_dwt = 0;

// Last arm/rearm diagnostic, across all timed slots.  This is designed to
// catch timers that are created or re-authored into an already-past deadline.
static volatile uint32_t diag_arm_already_past_count = 0;
static volatile uint32_t diag_arm_last_slot = UINT32_MAX;
static volatile uint32_t diag_arm_last_handle = 0;
static volatile const char* diag_arm_last_name = nullptr;
static volatile const char* diag_arm_last_source = nullptr;
static volatile uint32_t diag_arm_last_now = 0;
static volatile uint32_t diag_arm_last_deadline = 0;
static volatile int32_t  diag_arm_last_delta_ticks = 0;
static volatile int64_t  diag_arm_last_target_gnss_ns = -1;
static volatile uint32_t diag_arm_last_dwt = 0;
static volatile bool     diag_arm_last_already_past = false;
static volatile bool     diag_arm_last_had_now = false;

// IRQ scan audit for the 1 Hz phase grid and WITNESS_SCHEDULER.  This answers
// the subtle question: when CH2 fired for the shared 1 Hz deadline, was witness
// present and expirable, or did it become overdue through some other state path?
static volatile uint32_t diag_irq_grid_audit_count = 0;
static volatile uint32_t diag_irq_grid_last_now = 0;
static volatile uint32_t diag_irq_grid_last_dwt = 0;
static volatile uint32_t diag_irq_grid_onehz_due_count = 0;
static volatile uint32_t diag_irq_grid_onehz_expired_count = 0;
static volatile uint32_t diag_irq_grid_witness_slot = UINT32_MAX;
static volatile uint32_t diag_irq_grid_witness_handle = 0;
static volatile bool     diag_irq_grid_witness_seen = false;
static volatile bool     diag_irq_grid_witness_active = false;
static volatile bool     diag_irq_grid_witness_expired_before = false;
static volatile bool     diag_irq_grid_witness_reached_before = false;
static volatile bool     diag_irq_grid_witness_passed_before = false;
static volatile bool     diag_irq_grid_witness_expired_by_irq = false;
static volatile uint32_t diag_irq_grid_witness_deadline = 0;
static volatile uint32_t diag_irq_grid_witness_distance_ticks = 0;
static volatile uint32_t diag_irq_grid_witness_late_ticks = 0;
static volatile uint32_t diag_irq_grid_witness_missed_count = 0;
static volatile uint32_t diag_irq_grid_witness_not_active_count = 0;
static volatile uint32_t diag_irq_grid_witness_already_expired_count = 0;

static volatile uint32_t diag_irq_expired_slots = 0;
static volatile uint32_t diag_irq_exact_deadline_slots = 0;
static volatile uint32_t diag_irq_late_deadline_slots = 0;
static volatile uint32_t diag_irq_late_max_ticks = 0;
static volatile uint32_t diag_irq_event_gnss_mismatch_count = 0;
static volatile int64_t  diag_irq_event_gnss_last_delta_ns = 0;

// Epoch changes rebase the VCLOCK synthetic coordinate system. Existing
// timed deadlines must not survive that boundary; recurring slots are
// re-authored into the new epoch and one-shot timed slots are cancelled.
static volatile uint32_t diag_epoch_change_count = 0;
static volatile uint32_t diag_epoch_last_sequence = 0;
static volatile uint32_t diag_epoch_timed_slots_seen = 0;
static volatile uint32_t diag_epoch_recurring_rearmed = 0;
static volatile uint32_t diag_epoch_recurring_rearm_failures = 0;
static volatile uint32_t diag_epoch_one_shot_cancelled = 0;
static volatile uint32_t diag_epoch_expired_cleared = 0;
static volatile uint32_t diag_epoch_asap_left_active = 0;
static volatile uint32_t diag_epoch_alap_left_active = 0;

static volatile uint32_t diag_deadline_negative_offset = 0;
static volatile int64_t  diag_deadline_last_target_gnss_ns = -1;
static volatile int64_t  diag_deadline_last_anchor_pps_gnss_ns = -1;
static volatile int64_t  diag_deadline_last_ns_from_anchor = 0;

// Anchor/deadline forensic surface.
//
// TimePop's scheduling contract depends on time_anchor_snapshot() being lawful
// at the exact places where slots are armed, rearmed, and rebuilt across an
// epoch change.  These counters make that contract visible before any future
// process_time retirement attempt changes anchor ownership.
static volatile uint32_t diag_anchor_snapshot_count = 0;
static volatile uint32_t diag_anchor_snapshot_ok_count = 0;
static volatile uint32_t diag_anchor_snapshot_not_ok_count = 0;
static volatile uint32_t diag_anchor_snapshot_invalid_count = 0;
static volatile uint32_t diag_anchor_snapshot_zero_pps_count = 0;
static volatile uint32_t diag_anchor_snapshot_zero_cps_count = 0;
static volatile const char* diag_anchor_snapshot_last_context = nullptr;
static volatile const char* diag_anchor_snapshot_last_bad_context = nullptr;
static volatile bool     diag_anchor_snapshot_last_ok = false;
static volatile bool     diag_anchor_snapshot_last_valid = false;
static volatile uint32_t diag_anchor_snapshot_last_pps_count = 0;
static volatile uint32_t diag_anchor_snapshot_last_dwt_at_pps = 0;
static volatile uint32_t diag_anchor_snapshot_last_cycles_per_s = 0;
static volatile uint32_t diag_anchor_snapshot_last_qtimer_at_pps = 0;
static volatile uint32_t diag_anchor_snapshot_last_counter32_at_pps_vclock = 0;

static volatile uint32_t diag_deadline_convert_count = 0;
static volatile uint32_t diag_deadline_convert_success_count = 0;
static volatile uint32_t diag_deadline_fail_no_anchor = 0;
static volatile uint32_t diag_deadline_fail_invalid_anchor = 0;
static volatile uint32_t diag_deadline_fail_zero_pps_count = 0;
static volatile uint32_t diag_deadline_fail_range = 0;
static volatile uint32_t diag_deadline_last_deadline = 0;
static volatile const char* diag_deadline_last_failure = nullptr;
static volatile bool     diag_deadline_last_anchor_ok = false;
static volatile bool     diag_deadline_last_anchor_valid = false;
static volatile uint32_t diag_deadline_last_anchor_pps_count = 0;
static volatile uint32_t diag_deadline_last_anchor_qtimer_at_pps = 0;
static volatile uint32_t diag_deadline_last_anchor_dwt_at_pps = 0;
static volatile uint32_t diag_deadline_last_anchor_cycles_per_s = 0;

static volatile bool     diag_epoch_last_anchor_ok = false;
static volatile bool     diag_epoch_last_anchor_valid = false;
static volatile uint32_t diag_epoch_last_anchor_pps_count = 0;
static volatile uint32_t diag_epoch_last_anchor_dwt_at_pps = 0;
static volatile uint32_t diag_epoch_last_anchor_cycles_per_s = 0;
static volatile uint32_t diag_epoch_last_anchor_qtimer_at_pps = 0;
static volatile uint32_t diag_epoch_last_anchor_counter32_at_pps_vclock = 0;
static volatile uint32_t diag_epoch_last_schedule_next_calls_before = 0;
static volatile uint32_t diag_epoch_last_schedule_next_calls_after = 0;



// ============================================================================
// Forward declarations
// ============================================================================
//
// Keep all private prototypes here, after constants/types/state exist and before
// helper definitions begin.  This lets the implementation below be grouped by
// conceptual layer without C++ declaration-order surprises.

static void schedule_next(void);

static void timepop_record_anchor_snapshot(const time_anchor_snapshot_t& snap,
                                           const char* context);
static time_anchor_snapshot_t timepop_anchor_snapshot(const char* context);

static inline uint32_t vclock_count(void);
static inline bool deadline_reached(uint32_t deadline, uint32_t now);
static inline bool deadline_passed(uint32_t deadline, uint32_t now);
static inline uint32_t deadline_lateness_ticks(uint32_t deadline, uint32_t now);
static inline uint32_t ns_to_ticks(uint64_t ns);
static bool period_ns_to_exact_ticks(uint64_t period_ns, uint32_t& out_ticks);

static inline void update_slot_high_water(void);
static inline void update_deferred_high_water(deferred_slot_t* slots_buf,
                                              uint32_t max_slots,
                                              volatile uint32_t& high_water);
static inline void update_max_u32(volatile uint32_t& target, uint32_t value);
static inline uint32_t slot_index_for(const timepop_slot_t& slot);
static inline int32_t signed_deadline_delta_ticks(uint32_t deadline,
                                                  uint32_t now,
                                                  bool& already_past);
static inline bool slot_name_equals(const timepop_slot_t& slot, const char* name);
static inline bool slot_is_one_hz_recurring(const timepop_slot_t& slot);
static inline void note_named_replacement(const char* name);
static bool retire_existing_named_slots(const char* name);

static inline int64_t time_anchor_latest_pps_gnss_ns(const time_anchor_snapshot_t& snap);
static bool phase_locked_next_target_gnss_ns(uint64_t period_ns,
                                             const time_anchor_snapshot_t& snap,
                                             int64_t now_gnss_ns,
                                             int64_t& out_target_gnss_ns);
static bool anchored_next_target_gnss_ns(int64_t base_gnss_ns,
                                         uint64_t period_ns,
                                         int64_t now_gnss_ns,
                                         int64_t& out_target_gnss_ns,
                                         uint32_t* out_skipped_intervals);
static bool gnss_ns_to_vclock_deadline(int64_t target_gnss_ns,
                                       const time_anchor_snapshot_t& snap,
                                       uint32_t& out_deadline);
static int64_t vclock_to_gnss_ns(uint32_t vclock_raw,
                                 const time_anchor_snapshot_t& snap);
static uint32_t predict_dwt_at_deadline(uint32_t deadline, bool& valid);

static void record_slot_arm_diag(timepop_slot_t& slot,
                                 timepop_arm_source_t source,
                                 bool has_now,
                                 uint32_t now,
                                 int64_t target_gnss_ns);

static bool configure_phase_locked_recurring_slot(timepop_slot_t& slot,
                                                  uint64_t period_ns);
static bool configure_anchored_recurring_slot(timepop_slot_t& slot,
                                              int64_t base_gnss_ns,
                                              uint64_t period_ns,
                                              timepop_arm_source_t source);
static bool configure_anchored_recurring_slot_from_counter32(timepop_slot_t& slot,
                                                             int64_t base_gnss_ns,
                                                             uint32_t base_counter32,
                                                             uint64_t period_ns,
                                                             timepop_arm_source_t source);
static bool rearm_counter32_anchored_recurring_slot_from_event(timepop_slot_t& slot,
                                                               uint32_t fire_vclock_raw,
                                                               timepop_arm_source_t source);
static bool rearm_recurring_slot_for_epoch(timepop_slot_t& slot);
static bool rearm_recurring_slot_from_irq(timepop_slot_t& slot,
                                          uint32_t fire_vclock_raw,
                                          int64_t fire_gnss_ns,
                                          const time_anchor_snapshot_t& snap);

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
static inline void slot_build_ctx(const timepop_slot_t& slot,
                                  timepop_ctx_t& ctx);
static inline void slot_build_diag(const timepop_slot_t& slot,
                                   uint32_t dwt_at_isr_entry,
                                   timepop_diag_t& diag);

static timepop_handle_t arm_anchored_recurring_isr_internal(int64_t base_gnss_ns,
                                                            uint64_t period_gnss_ns,
                                                            timepop_callback_t callback,
                                                            void* user_data,
                                                            const char* name);
static timepop_handle_t arm_anchored_recurring_isr_from_counter32_internal(int64_t base_gnss_ns,
                                                                           uint32_t base_counter32,
                                                                           uint64_t period_gnss_ns,
                                                                           timepop_callback_t callback,
                                                                           void* user_data,
                                                                           const char* name);
static timepop_handle_t arm_absolute_slot_internal(int64_t target_gnss_ns,
                                                   bool recurring,
                                                   uint64_t recurring_period_gnss_ns,
                                                   timepop_callback_t callback,
                                                   void* user_data,
                                                   const char* name,
                                                   bool isr_callback,
                                                   uint32_t target_dwt = 0);
static timepop_handle_t arm_relative_slot_internal(uint64_t delay_gnss_ns,
                                                   bool recurring,
                                                   timepop_callback_t callback,
                                                   void* user_data,
                                                   const char* name,
                                                   bool isr_callback,
                                                   bool rearm_in_isr);

// ============================================================================
// Helpers
// ============================================================================

static void timepop_record_anchor_snapshot(const time_anchor_snapshot_t& snap,
                                           const char* context) {
  diag_anchor_snapshot_count++;
  diag_anchor_snapshot_last_context = context;
  diag_anchor_snapshot_last_ok = snap.ok;
  diag_anchor_snapshot_last_valid = snap.valid;
  diag_anchor_snapshot_last_pps_count = snap.pps_count;
  diag_anchor_snapshot_last_dwt_at_pps = snap.dwt_at_pps;
  diag_anchor_snapshot_last_cycles_per_s = snap.dwt_cycles_per_s;
  diag_anchor_snapshot_last_qtimer_at_pps = snap.qtimer_at_pps;
  diag_anchor_snapshot_last_counter32_at_pps_vclock =
      snap.counter32_at_pps_vclock;

  if (snap.ok) {
    diag_anchor_snapshot_ok_count++;
  } else {
    diag_anchor_snapshot_not_ok_count++;
    diag_anchor_snapshot_last_bad_context = context;
  }

  if (!snap.valid) {
    diag_anchor_snapshot_invalid_count++;
    diag_anchor_snapshot_last_bad_context = context;
  }

  if (snap.pps_count < 1) {
    diag_anchor_snapshot_zero_pps_count++;
    diag_anchor_snapshot_last_bad_context = context;
  }

  if (snap.dwt_cycles_per_s == 0) {
    diag_anchor_snapshot_zero_cps_count++;
    diag_anchor_snapshot_last_bad_context = context;
  }
}

static time_anchor_snapshot_t timepop_anchor_snapshot(const char* context) {
  const time_anchor_snapshot_t snap = time_anchor_snapshot();
  timepop_record_anchor_snapshot(snap, context);
  return snap;
}

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

  const time_anchor_snapshot_t snap =
      timepop_anchor_snapshot("configure_phase_locked");
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
  slot.recurring_base_counter32_fixed = false;
  slot.recurring_base_counter32 = 0;
  slot.recurring_next_index = 0;
  slot.recurring_base_gnss_ns =
      (anchor_gnss_ns >= 0) ? (anchor_gnss_ns + (int64_t)period_ns) : target_gnss_ns;
  slot.recurring_period_gnss_ns = period_ns;
  slot.target_gnss_ns = target_gnss_ns;
  slot.deadline = deadline;
  return true;
}


static bool anchored_next_target_gnss_ns(int64_t base_gnss_ns,
                                          uint64_t period_ns,
                                          int64_t now_gnss_ns,
                                          int64_t& out_target_gnss_ns,
                                          uint32_t* out_skipped_intervals) {
  if (base_gnss_ns <= 0) return false;
  if (period_ns == 0 || period_ns > (uint64_t)INT64_MAX) return false;
  if (now_gnss_ns < 0) return false;

  uint64_t steps = 0;
  if (now_gnss_ns >= base_gnss_ns) {
    steps = (uint64_t)((now_gnss_ns - base_gnss_ns) / (int64_t)period_ns) + 1ULL;
  }

  const int64_t target = base_gnss_ns + (int64_t)(steps * period_ns);
  if (target <= now_gnss_ns) return false;

  out_target_gnss_ns = target;
  if (out_skipped_intervals) {
    *out_skipped_intervals = (steps > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)steps;
  }
  return true;
}

static bool period_ns_to_exact_ticks(uint64_t period_ns, uint32_t& out_ticks) {
  if (period_ns == 0) return false;
  if ((period_ns % NS_PER_TICK) != 0) return false;

  const uint64_t ticks64 = period_ns / NS_PER_TICK;
  if (ticks64 < MIN_DELAY_TICKS) return false;
  if (ticks64 > MAX_DELAY_TICKS) return false;

  out_ticks = (uint32_t)ticks64;
  return true;
}

static bool configure_anchored_recurring_slot_from_counter32(
  timepop_slot_t& slot,
  int64_t         base_gnss_ns,
  uint32_t        base_counter32,
  uint64_t        period_ns,
  timepop_arm_source_t source
) {
  if (base_gnss_ns <= 0) return false;
  if (period_ns == 0 || period_ns > (uint64_t)INT64_MAX) return false;

  uint32_t period_ticks = 0;
  if (!period_ns_to_exact_ticks(period_ns, period_ticks)) return false;

  slot.is_absolute = true;
  slot.recurrence_mode = timepop_recurrence_mode_t::ABSOLUTE;
  slot.recurring_base_fixed = true;
  slot.recurring_base_counter32_fixed = true;
  slot.recurring_base_counter32 = base_counter32;
  slot.recurring_next_index = 1ULL;
  slot.recurring_base_gnss_ns = base_gnss_ns;
  slot.recurring_period_gnss_ns = period_ns;
  slot.period_ns = period_ns;
  slot.period_ticks = period_ticks;
  slot.target_gnss_ns = base_gnss_ns + (int64_t)period_ns;
  slot.deadline = base_counter32 + period_ticks;
  slot.recurring_last_skipped_intervals = 0;
  slot.predicted_dwt = predict_dwt_at_deadline(slot.deadline, slot.prediction_valid);

  // This substrate arm path deliberately does not ask time_gnss_ns_now(), and
  // it does not require a valid time_anchor_snapshot().  The scheduled grid is
  // defined by the caller-owned base pair: (base_gnss_ns, base_counter32).
  record_slot_arm_diag(slot, source, false, 0, slot.target_gnss_ns);
  return true;
}

static bool rearm_counter32_anchored_recurring_slot_from_event(
  timepop_slot_t& slot,
  uint32_t        fire_vclock_raw,
  timepop_arm_source_t source
) {
  if (!slot.active || !slot.recurring || !slot.recurring_base_counter32_fixed) {
    return false;
  }

  const uint64_t period_ns = slot.recurring_period_gnss_ns
      ? slot.recurring_period_gnss_ns
      : slot.period_ns;
  uint32_t period_ticks = 0;
  if (!period_ns_to_exact_ticks(period_ns, period_ticks)) return false;
  if (slot.recurring_base_gnss_ns <= 0) return false;

  uint64_t index = slot.recurring_next_index ? slot.recurring_next_index : 1ULL;
  uint32_t deadline = slot.recurring_base_counter32 + (uint32_t)(index * period_ticks);

  uint32_t skipped = 0;
  while (deadline_reached(deadline, fire_vclock_raw)) {
    index++;
    skipped++;
    deadline = slot.recurring_base_counter32 + (uint32_t)(index * period_ticks);
    if (skipped > 100000U) return false;
  }

  slot.period_ns = period_ns;
  slot.period_ticks = period_ticks;
  slot.expired = false;
  slot.isr_callback_fired = false;
  slot.fire_vclock_raw = 0;
  slot.fire_dwt_cyccnt = 0;
  slot.fire_gnss_ns = -1;
  slot.fire_capture_source = timepop_fire_capture_source_t::NONE;

  slot.recurring_next_index = index;
  slot.deadline = deadline;
  slot.target_gnss_ns = slot.recurring_base_gnss_ns + (int64_t)(index * period_ns);
  slot.is_absolute = true;
  slot.recurrence_mode = timepop_recurrence_mode_t::ABSOLUTE;
  slot.recurring_last_skipped_intervals = skipped;
  slot.recurring_total_skipped_intervals += skipped;
  if (skipped > 1) {
    slot.recurring_immediate_expire_count++;
    slot.recurring_catchup_count += skipped - 1;
  }

  slot.predicted_dwt = predict_dwt_at_deadline(slot.deadline, slot.prediction_valid);
  slot.recurring_rearmed_count++;
  record_slot_arm_diag(slot, source, true, fire_vclock_raw, slot.target_gnss_ns);
  return true;
}

// Ambient VCLOCK observations are provided by process_interrupt.
// These values are scheduling observations only, not event facts.
static inline uint32_t vclock_count(void) {
  return interrupt_vclock_counter32_observe_ambient();
}

static inline bool deadline_reached(uint32_t deadline, uint32_t now) {
  return deadline == now || (deadline - now) > MAX_DELAY_TICKS;
}

static inline bool deadline_passed(uint32_t deadline, uint32_t now) {
  return (deadline - now) > MAX_DELAY_TICKS;
}

static inline uint32_t deadline_lateness_ticks(uint32_t deadline, uint32_t now) {
  return (deadline == now) ? 0U : (now - deadline);
}

static inline void update_max_u32(volatile uint32_t& target, uint32_t value) {
  if (value > target) target = value;
}

static inline uint32_t slot_index_for(const timepop_slot_t& slot) {
  const timepop_slot_t* base = &slots[0];
  const timepop_slot_t* ptr = &slot;
  if (ptr < base || ptr >= base + MAX_SLOTS) return UINT32_MAX;
  return (uint32_t)(ptr - base);
}

static inline int32_t signed_deadline_delta_ticks(uint32_t deadline,
                                                  uint32_t now,
                                                  bool& already_past) {
  already_past = deadline_passed(deadline, now);
  if (already_past) {
    const uint32_t late = deadline_lateness_ticks(deadline, now);
    return (late > (uint32_t)INT32_MAX) ? INT32_MIN : -(int32_t)late;
  }
  const uint32_t distance = deadline - now;
  return (distance > (uint32_t)INT32_MAX) ? INT32_MAX : (int32_t)distance;
}

static void record_slot_arm_diag(timepop_slot_t& slot,
                                 timepop_arm_source_t source,
                                 bool has_now,
                                 uint32_t now,
                                 int64_t target_gnss_ns) {
  bool already_past = false;
  int32_t delta_ticks = 0;
  if (has_now) {
    delta_ticks = signed_deadline_delta_ticks(slot.deadline, now, already_past);
  }

  slot.last_arm_source = source;
  slot.last_arm_now = has_now ? now : 0;
  slot.last_arm_deadline = slot.deadline;
  slot.last_arm_delta_ticks = has_now ? delta_ticks : 0;
  slot.last_arm_target_gnss_ns = target_gnss_ns;
  slot.last_arm_dwt = ARM_DWT_CYCCNT;
  slot.last_arm_already_past = already_past;
  slot.last_arm_had_now = has_now;
  if (already_past) slot.arm_already_past_count++;

  diag_arm_last_slot = slot_index_for(slot);
  diag_arm_last_handle = slot.handle;
  diag_arm_last_name = slot.name;
  diag_arm_last_source = arm_source_str(source);
  diag_arm_last_now = slot.last_arm_now;
  diag_arm_last_deadline = slot.last_arm_deadline;
  diag_arm_last_delta_ticks = slot.last_arm_delta_ticks;
  diag_arm_last_target_gnss_ns = slot.last_arm_target_gnss_ns;
  diag_arm_last_dwt = slot.last_arm_dwt;
  diag_arm_last_already_past = already_past;
  diag_arm_last_had_now = has_now;
  if (already_past) diag_arm_already_past_count++;
}

static inline bool slot_name_equals(const timepop_slot_t& slot, const char* name) {
  return slot.name && name && strcmp(slot.name, name) == 0;
}

static inline bool slot_is_one_hz_recurring(const timepop_slot_t& slot) {
  return slot.active && slot.recurring && slot.period_ticks == ONE_HZ_TICKS;
}

static inline uint32_t ns_to_ticks(uint64_t ns) {
  uint64_t ticks = ns / NS_PER_TICK;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) ticks = MAX_DELAY_TICKS;
  return (uint32_t)ticks;
}

static uint32_t predict_dwt_at_deadline(uint32_t deadline, bool& valid) {
  valid = false;
  time_anchor_snapshot_t snap =
      timepop_anchor_snapshot("predict_dwt_at_deadline");
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

static bool configure_anchored_recurring_slot(timepop_slot_t& slot,
                                               int64_t base_gnss_ns,
                                               uint64_t period_ns,
                                               timepop_arm_source_t source) {
  if (period_ns == 0) return false;

  const time_anchor_snapshot_t snap =
      timepop_anchor_snapshot("configure_anchored_recurring");
  const int64_t now_gnss_ns = time_gnss_ns_now();
  int64_t target_gnss_ns = -1;
  uint32_t skipped = 0;
  if (!anchored_next_target_gnss_ns(base_gnss_ns,
                                    period_ns,
                                    now_gnss_ns,
                                    target_gnss_ns,
                                    &skipped)) {
    return false;
  }

  uint32_t deadline = 0;
  if (!gnss_ns_to_vclock_deadline(target_gnss_ns, snap, deadline)) {
    return false;
  }

  slot.is_absolute = true;
  slot.recurrence_mode = timepop_recurrence_mode_t::ABSOLUTE;
  slot.recurring_base_fixed = true;
  slot.recurring_base_counter32_fixed = false;
  slot.recurring_base_counter32 = 0;
  slot.recurring_next_index = 0;
  slot.recurring_base_gnss_ns = base_gnss_ns;
  slot.recurring_period_gnss_ns = period_ns;
  slot.period_ns = period_ns;
  slot.period_ticks = ns_to_ticks(period_ns);
  slot.target_gnss_ns = target_gnss_ns;
  slot.deadline = deadline;
  slot.recurring_last_skipped_intervals = skipped;
  slot.recurring_total_skipped_intervals += skipped;
  slot.predicted_dwt = predict_dwt_at_deadline(slot.deadline, slot.prediction_valid);
  record_slot_arm_diag(slot, source, true, vclock_count(), slot.target_gnss_ns);
  return true;
}

// Re-author a recurring timed slot after a VCLOCK epoch change.  No old
// deadline survives the boundary.  Phase-locked recurring slots are placed on
// the new PPS/VCLOCK grid; if a lawful time anchor is not yet available, the
// slot falls back to a relative period from the new ambient VCLOCK coordinate.
static bool rearm_recurring_slot_for_epoch(timepop_slot_t& slot) {
  if (!slot.active || !slot.recurring) return false;

  const uint64_t period_ns = slot.recurring_period_gnss_ns
      ? slot.recurring_period_gnss_ns
      : slot.period_ns;
  if (period_ns == 0) return false;

  slot.expired = false;
  slot.isr_callback_fired = false;
  slot.fire_vclock_raw = 0;
  slot.fire_dwt_cyccnt = 0;
  slot.fire_gnss_ns = -1;
  slot.fire_capture_source = timepop_fire_capture_source_t::NONE;
  slot.first_expire_now = 0;
  slot.first_expire_recorded = false;

  if (configure_phase_locked_recurring_slot(slot, period_ns)) {
    slot.period_ticks = ns_to_ticks(period_ns);
    slot.period_ns = period_ns;
    slot.predicted_dwt = predict_dwt_at_deadline(
        slot.deadline, slot.prediction_valid);
    slot.recurring_rearmed_count++;
    record_slot_arm_diag(slot,
                         timepop_arm_source_t::EPOCH_REARM,
                         true,
                         vclock_count(),
                         slot.target_gnss_ns);
    return true;
  }

  const uint32_t ticks = ns_to_ticks(period_ns);
  const uint32_t now = vclock_count();
  slot.period_ticks = ticks;
  slot.period_ns = period_ns;
  slot.deadline = now + ticks;
  slot.arm_vclock_raw = now;
  slot.arm_delta_ticks = ticks;
  slot.target_gnss_ns = -1;
  slot.recurrence_mode = timepop_recurrence_mode_t::RELATIVE;
  slot.is_absolute = false;
  slot.predicted_dwt = predict_dwt_at_deadline(
      slot.deadline, slot.prediction_valid);
  if (slot.prediction_valid) {
    slot.target_gnss_ns = time_gnss_ns_at_dwt(slot.predicted_dwt);
  }
  slot.recurring_rearmed_count++;
  record_slot_arm_diag(slot,
                       timepop_arm_source_t::EPOCH_REARM_FALLBACK,
                       true,
                       now,
                       slot.target_gnss_ns);
  return true;
}



// Re-arm a critical recurring ISR slot before schedule_next() chooses the next
// CH2 compare.  This path must not depend on foreground dispatch.  It uses the
// already-authored CH2 fire facts from the current IRQ pass and falls back to a
// relative deadline from the captured VCLOCK identity if the absolute GNSS
// anchor is not yet usable.
static bool rearm_recurring_slot_from_irq(timepop_slot_t& slot,
                                          uint32_t fire_vclock_raw,
                                          int64_t fire_gnss_ns,
                                          const time_anchor_snapshot_t& snap) {
  if (!slot.active || !slot.recurring || !slot.rearm_in_isr) return false;

  const uint64_t period_ns = slot.recurring_period_gnss_ns
      ? slot.recurring_period_gnss_ns
      : slot.period_ns;
  if (period_ns == 0) return false;

  const uint32_t period_ticks = ns_to_ticks(period_ns);

  slot.period_ns = period_ns;
  slot.period_ticks = period_ticks;
  slot.expired = false;
  slot.isr_callback_fired = false;
  slot.fire_vclock_raw = 0;
  slot.fire_dwt_cyccnt = 0;
  slot.fire_gnss_ns = -1;
  slot.fire_capture_source = timepop_fire_capture_source_t::NONE;

  if (slot.recurring_base_counter32_fixed) {
    return rearm_counter32_anchored_recurring_slot_from_event(
        slot, fire_vclock_raw, timepop_arm_source_t::ISR_REARM);
  }

  bool rearmed = false;
  timepop_arm_source_t rearm_source = timepop_arm_source_t::ISR_REARM_FALLBACK;

  if (slot.recurrence_mode == timepop_recurrence_mode_t::ABSOLUTE &&
      slot.target_gnss_ns > 0 && fire_gnss_ns >= 0) {
    int64_t next_target_gnss_ns = -1;
    uint32_t skipped = 0;

    if (slot.recurring_base_fixed && slot.recurring_base_gnss_ns > 0) {
      if (anchored_next_target_gnss_ns(slot.recurring_base_gnss_ns,
                                       period_ns,
                                       fire_gnss_ns,
                                       next_target_gnss_ns,
                                       &skipped)) {
        if (next_target_gnss_ns > slot.target_gnss_ns + (int64_t)period_ns) {
          slot.recurring_immediate_expire_count++;
          slot.recurring_catchup_count += skipped;
        }
        slot.recurring_last_skipped_intervals = skipped;
        slot.recurring_total_skipped_intervals += skipped;
      }
    } else {
      next_target_gnss_ns = slot.target_gnss_ns + (int64_t)period_ns;
      if (next_target_gnss_ns <= fire_gnss_ns) {
        const uint64_t missed =
            (uint64_t)((fire_gnss_ns - next_target_gnss_ns) /
                       (int64_t)period_ns) + 1ULL;
        next_target_gnss_ns += (int64_t)(missed * period_ns);
        slot.recurring_immediate_expire_count++;
        slot.recurring_catchup_count += (uint32_t)missed;
        slot.recurring_last_skipped_intervals = (uint32_t)missed;
        slot.recurring_total_skipped_intervals += (uint32_t)missed;
      }
    }

    uint32_t next_deadline = 0;
    if (next_target_gnss_ns > 0 &&
        gnss_ns_to_vclock_deadline(next_target_gnss_ns, snap, next_deadline)) {
      slot.target_gnss_ns = next_target_gnss_ns;
      slot.deadline = next_deadline;
      slot.is_absolute = true;
      slot.recurrence_mode = timepop_recurrence_mode_t::ABSOLUTE;
      rearm_source = timepop_arm_source_t::ISR_REARM;
      rearmed = true;
    }
  }

  if (!rearmed) {
    slot.deadline = fire_vclock_raw + period_ticks;
    slot.arm_vclock_raw = fire_vclock_raw;
    slot.arm_delta_ticks = period_ticks;
    slot.is_absolute = false;
    slot.recurrence_mode = timepop_recurrence_mode_t::RELATIVE;
    slot.target_gnss_ns = (fire_gnss_ns >= 0)
        ? (fire_gnss_ns + (int64_t)period_ns)
        : -1;
    rearmed = true;
  }

  slot.predicted_dwt = predict_dwt_at_deadline(slot.deadline,
                                               slot.prediction_valid);
  slot.recurring_rearmed_count++;
  record_slot_arm_diag(slot,
                       rearm_source,
                       true,
                       fire_vclock_raw,
                       slot.target_gnss_ns);
  return rearmed;
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
  diag_deadline_convert_count++;
  diag_deadline_last_target_gnss_ns = target_gnss_ns;
  diag_deadline_last_anchor_ok = snap.ok;
  diag_deadline_last_anchor_valid = snap.valid;
  diag_deadline_last_anchor_pps_count = snap.pps_count;
  diag_deadline_last_anchor_qtimer_at_pps = snap.qtimer_at_pps;
  diag_deadline_last_anchor_dwt_at_pps = snap.dwt_at_pps;
  diag_deadline_last_anchor_cycles_per_s = snap.dwt_cycles_per_s;
  diag_deadline_last_failure = "ok";

  if (!snap.ok) {
    diag_deadline_fail_no_anchor++;
    diag_deadline_last_failure = "anchor_not_ok";
    return false;
  }

  if (!snap.valid) {
    diag_deadline_fail_invalid_anchor++;
    diag_deadline_last_failure = "anchor_invalid";
    return false;
  }

  if (snap.pps_count < 1) {
    diag_deadline_fail_zero_pps_count++;
    diag_deadline_last_failure = "anchor_zero_pps_count";
    return false;
  }

  const int64_t anchor_pps_gnss_ns =
    (int64_t)(snap.pps_count - 1) * (int64_t)1000000000LL;

  const int64_t ns_from_anchor = target_gnss_ns - anchor_pps_gnss_ns;

  diag_deadline_last_anchor_pps_gnss_ns = anchor_pps_gnss_ns;
  diag_deadline_last_ns_from_anchor = ns_from_anchor;

  if (ns_from_anchor < 0) {
    diag_deadline_negative_offset++;
    diag_deadline_last_failure = "negative_offset";
    return false;
  }

  const uint64_t ticks64 = (uint64_t)ns_from_anchor / NS_PER_TICK;
  if (ticks64 > 0xFFFFFFFFULL) {
    diag_deadline_fail_range++;
    diag_deadline_last_failure = "deadline_range";
    return false;
  }

  out_deadline = snap.qtimer_at_pps + (uint32_t)ticks64;
  diag_deadline_last_deadline = out_deadline;
  diag_deadline_convert_success_count++;
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
// VCLOCK counter.  It determines how far away the nearest deadline is and
// arms CH2 accordingly.  It does not claim exact equality; equality belongs
// to the CH2 IRQ event.  If this foreground path ever finds an already-past
// deadline, it authors a visible SCHEDULE_NEXT catch-up fact and increments
// report counters.

static void schedule_next(void) {
  diag_schedule_next_calls_total++;

  const uint32_t now = vclock_count();
  const time_anchor_snapshot_t snap =
      timepop_anchor_snapshot("schedule_next");
  uint32_t shared_fire_dwt = 0;
  bool shared_fire_captured = false;
  bool schedule_next_expired_this_pass = false;

  uint32_t soonest = 0;
  bool found = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;

    if (deadline_passed(slots[i].deadline, now)) {
      if (!schedule_next_expired_this_pass) {
        diag_schedule_next_expired_passes++;
        schedule_next_expired_this_pass = true;
      }
      diag_schedule_next_expired_slots++;
      const uint32_t late_ticks = deadline_lateness_ticks(slots[i].deadline, now);
      update_max_u32(diag_schedule_next_late_max_ticks, late_ticks);

      if (!slots[i].first_expire_recorded) {
        slots[i].first_expire_now = now;
        slots[i].first_expire_recorded = true;
      }

      // SCHEDULE_NEXT is not a precision timing source.  It is an anomaly /
      // catch-up surface: if it ever authors a timed fire, the report counters
      // above make that fact visible.
      if (!shared_fire_captured) {
        shared_fire_dwt = ARM_DWT_CYCCNT;
        shared_fire_captured = true;
      }

      slots[i].schedule_next_expired_count++;
      slots[i].schedule_next_last_late_ticks = late_ticks;
      slots[i].schedule_next_last_now = now;
      slots[i].schedule_next_last_deadline = slots[i].deadline;
      slots[i].schedule_next_last_dwt = shared_fire_dwt;
      if (late_ticks > slots[i].schedule_next_late_max_ticks) {
        slots[i].schedule_next_late_max_ticks = late_ticks;
      }

      diag_schedule_next_last_expired_slot = i;
      diag_schedule_next_last_expired_handle = slots[i].handle;
      diag_schedule_next_last_expired_name = slots[i].name;
      diag_schedule_next_last_expired_deadline = slots[i].deadline;
      diag_schedule_next_last_expired_now = now;
      diag_schedule_next_last_expired_late_ticks = late_ticks;
      diag_schedule_next_last_expired_dwt = shared_fire_dwt;

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
    const uint32_t distance = soonest - now;
    if (distance > HEARTBEAT_TICKS) {
      target = now + HEARTBEAT_TICKS;
      diag_heartbeat_rearms++;
    } else {
      target = soonest;
    }
  }

  ch2_arm_compare(target);
  diag_rearm_count++;
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
// event/diag payload — DWT and counter32 already filled in by the dispatcher.
// TimePop authors fire_gnss_ns from that counter32 identity using VCLOCK
// arithmetic; any GNSS value in the interrupt payload is diagnostic.  The
// CH2 TCF1 flag is cleared by the dispatcher before this runs, so we don't
// clear it ourselves.
//
void timepop_qtimer1_ch2_handler(const interrupt_event_t& event,
                                 const interrupt_capture_diag_t& /*diag*/) {

  const uint32_t dwt_entry      = event.dwt_at_event;
  const uint32_t now            = event.counter32_at_event;
  const time_anchor_snapshot_t anchor =
      timepop_anchor_snapshot("qtimer1_ch2_irq");
  bool expired_this_pass[MAX_SLOTS] = {};

  diag_isr_count++;

  bool any_expired = false;
  bool needs_scheduled_dispatch = false;

  // Pre-scan audit: capture the state of the 1 Hz grid and WITNESS_SCHEDULER
  // before the IRQ expiry pass mutates any slot.  This tells us whether a
  // witness timer that is later caught by schedule_next() was visible at the
  // shared one-second IRQ boundary.
  uint32_t audit_onehz_due_count = 0;
  uint32_t audit_witness_slot = UINT32_MAX;
  uint32_t audit_witness_handle = 0;
  bool audit_witness_seen = false;
  bool audit_witness_active = false;
  bool audit_witness_expired_before = false;
  bool audit_witness_reached_before = false;
  bool audit_witness_passed_before = false;
  uint32_t audit_witness_deadline = 0;
  uint32_t audit_witness_distance_ticks = 0;
  uint32_t audit_witness_late_ticks = 0;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slot_name_equals(slots[i], WITNESS_SCHEDULER_NAME)) {
      audit_witness_seen = true;
      audit_witness_slot = i;
      audit_witness_handle = slots[i].handle;
      audit_witness_active = slots[i].active;
      audit_witness_expired_before = slots[i].expired;
      audit_witness_deadline = slots[i].deadline;
      if (slots[i].active && !slots[i].expired) {
        audit_witness_reached_before = deadline_reached(slots[i].deadline, now);
        audit_witness_passed_before = deadline_passed(slots[i].deadline, now);
        if (audit_witness_passed_before) {
          audit_witness_late_ticks = deadline_lateness_ticks(slots[i].deadline, now);
        } else {
          audit_witness_distance_ticks = slots[i].deadline - now;
        }
      }
    }

    if (slot_is_one_hz_recurring(slots[i]) &&
        !slots[i].expired &&
        deadline_reached(slots[i].deadline, now)) {
      audit_onehz_due_count++;
    }
  }

  const bool audit_grid_this_irq =
      audit_onehz_due_count > 0 || audit_witness_reached_before;

  // Phase 1: author the shared fire facts for every slot reached by this
  // physical CH2 compare event before any user callback is allowed to run.
  // This guarantees simultaneous clients receive the same DWT/VCLOCK/GNSS
  // context; callback serialization is logical only.
  //
  // TimePop trusts the VCLOCK compare identity.  GNSS is therefore authored
  // from VCLOCK arithmetic here, even if process_interrupt also supplied a
  // DWT-bridge GNSS diagnostic in the event payload.
  const int64_t event_fire_gnss_ns = vclock_to_gnss_ns(now, anchor);
  if (event.gnss_ns_at_event != 0 && event_fire_gnss_ns >= 0) {
    const int64_t delta = (int64_t)event.gnss_ns_at_event - event_fire_gnss_ns;
    if (delta != 0) {
      diag_irq_event_gnss_mismatch_count++;
      diag_irq_event_gnss_last_delta_ns = delta;
    }
  }

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_reached(slots[i].deadline, now)) continue;

    const uint32_t late_ticks = deadline_lateness_ticks(slots[i].deadline, now);
    if (late_ticks == 0) {
      diag_irq_exact_deadline_slots++;
    } else {
      diag_irq_late_deadline_slots++;
      update_max_u32(diag_irq_late_max_ticks, late_ticks);
    }
    diag_irq_expired_slots++;

    slots[i].irq_reached_count++;
    slots[i].irq_last_now = now;
    slots[i].irq_last_dwt = dwt_entry;
    slots[i].irq_last_late_ticks = late_ticks;
    slots[i].irq_last_exact = (late_ticks == 0);
    if (late_ticks > slots[i].irq_late_max_ticks) {
      slots[i].irq_late_max_ticks = late_ticks;
    }

    const bool is_critical_isr_recurring =
        slots[i].isr_callback && slots[i].rearm_in_isr && slots[i].recurring;

    slot_capture(slots[i],
                 now,
                 dwt_entry,
                 event_fire_gnss_ns,
                 anchor,
                 timepop_fire_capture_source_t::IRQ_CH2);
    slots[i].expired = true;
    slots[i].irq_expired_by_irq_count++;
    slots[i].isr_callback_fired = false;
    expired_this_pass[i] = true;
    expired_count++;
    if (!is_critical_isr_recurring) {
      timepop_pending = true;
      needs_scheduled_dispatch = true;
    }
    any_expired = true;
  }

  if (audit_grid_this_irq) {
    uint32_t audit_onehz_expired_count = 0;
    for (uint32_t i = 0; i < MAX_SLOTS; i++) {
      if (slot_is_one_hz_recurring(slots[i]) && expired_this_pass[i]) {
        audit_onehz_expired_count++;
      }
    }

    const bool witness_expired_by_irq =
        audit_witness_seen &&
        audit_witness_slot < MAX_SLOTS &&
        expired_this_pass[audit_witness_slot];

    diag_irq_grid_audit_count++;
    diag_irq_grid_last_now = now;
    diag_irq_grid_last_dwt = dwt_entry;
    diag_irq_grid_onehz_due_count = audit_onehz_due_count;
    diag_irq_grid_onehz_expired_count = audit_onehz_expired_count;
    diag_irq_grid_witness_slot = audit_witness_slot;
    diag_irq_grid_witness_handle = audit_witness_handle;
    diag_irq_grid_witness_seen = audit_witness_seen;
    diag_irq_grid_witness_active = audit_witness_active;
    diag_irq_grid_witness_expired_before = audit_witness_expired_before;
    diag_irq_grid_witness_reached_before = audit_witness_reached_before;
    diag_irq_grid_witness_passed_before = audit_witness_passed_before;
    diag_irq_grid_witness_expired_by_irq = witness_expired_by_irq;
    diag_irq_grid_witness_deadline = audit_witness_deadline;
    diag_irq_grid_witness_distance_ticks = audit_witness_distance_ticks;
    diag_irq_grid_witness_late_ticks = audit_witness_late_ticks;

    if (audit_witness_seen && audit_witness_reached_before && !witness_expired_by_irq) {
      diag_irq_grid_witness_missed_count++;
    }
    if (audit_onehz_due_count > 0 && (!audit_witness_seen || !audit_witness_active)) {
      diag_irq_grid_witness_not_active_count++;
    }
    if (audit_onehz_due_count > 0 && audit_witness_seen && audit_witness_expired_before) {
      diag_irq_grid_witness_already_expired_count++;
    }
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

    if (slots[i].recurring && slots[i].rearm_in_isr) {
      if (rearm_recurring_slot_from_irq(slots[i], now, event_fire_gnss_ns, anchor)) {
        diag_isr_recurring_rearmed++;
      } else {
        slots[i] = {};
        diag_isr_recurring_rearm_failures++;
      }
    }
  }

  if (any_expired) {
    isr_fire_count++;
    if (needs_scheduled_dispatch) {
      timepop_pending = true;
    }
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
// CH0 passive VCLOCK counter and CH2 scheduler channel are initialized by
// process_interrupt_init_hardware().  TimePop only requests CH2 compare
// updates as it schedules slots; the channel mode/control setup is no longer
// TimePop's responsibility.

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
  diag_anchor_snapshot_count = 0;
  diag_anchor_snapshot_ok_count = 0;
  diag_anchor_snapshot_not_ok_count = 0;
  diag_anchor_snapshot_invalid_count = 0;
  diag_anchor_snapshot_zero_pps_count = 0;
  diag_anchor_snapshot_zero_cps_count = 0;
  diag_anchor_snapshot_last_context = nullptr;
  diag_anchor_snapshot_last_bad_context = nullptr;
  diag_anchor_snapshot_last_ok = false;
  diag_anchor_snapshot_last_valid = false;
  diag_anchor_snapshot_last_pps_count = 0;
  diag_anchor_snapshot_last_dwt_at_pps = 0;
  diag_anchor_snapshot_last_cycles_per_s = 0;
  diag_anchor_snapshot_last_qtimer_at_pps = 0;
  diag_anchor_snapshot_last_counter32_at_pps_vclock = 0;
  diag_deadline_convert_count = 0;
  diag_deadline_convert_success_count = 0;
  diag_deadline_fail_no_anchor = 0;
  diag_deadline_fail_invalid_anchor = 0;
  diag_deadline_fail_zero_pps_count = 0;
  diag_deadline_fail_range = 0;
  diag_deadline_last_deadline = 0;
  diag_deadline_last_failure = nullptr;
  diag_deadline_last_anchor_ok = false;
  diag_deadline_last_anchor_valid = false;
  diag_deadline_last_anchor_pps_count = 0;
  diag_deadline_last_anchor_qtimer_at_pps = 0;
  diag_deadline_last_anchor_dwt_at_pps = 0;
  diag_deadline_last_anchor_cycles_per_s = 0;
  diag_epoch_last_anchor_ok = false;
  diag_epoch_last_anchor_valid = false;
  diag_epoch_last_anchor_pps_count = 0;
  diag_epoch_last_anchor_dwt_at_pps = 0;
  diag_epoch_last_anchor_cycles_per_s = 0;
  diag_epoch_last_anchor_qtimer_at_pps = 0;
  diag_epoch_last_anchor_counter32_at_pps_vclock = 0;
  diag_epoch_last_schedule_next_calls_before = 0;
  diag_epoch_last_schedule_next_calls_after = 0;
  // QTimer1 hardware (CH0/CH2) is initialized by
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

  const time_anchor_snapshot_t snap =
      timepop_anchor_snapshot("arm_absolute");
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
    slots[i].rearm_in_isr  = false;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_base_fixed = false;
    slots[i].recurring_base_counter32_fixed = false;
    slots[i].recurring_base_counter32 = 0;
    slots[i].recurring_next_index = 0;
    slots[i].recurring_last_skipped_intervals = 0;
    slots[i].recurring_total_skipped_intervals = 0;
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

    record_slot_arm_diag(slots[i],
                         (target_dwt != 0)
                             ? timepop_arm_source_t::EXACT_ARM
                             : timepop_arm_source_t::ABSOLUTE_ARM,
                         true,
                         vclock_count(),
                         slots[i].target_gnss_ns);

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


static timepop_handle_t arm_anchored_recurring_isr_internal(
  int64_t             base_gnss_ns,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (base_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  if (period_gnss_ns == 0) return TIMEPOP_INVALID_HANDLE;

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
    slots[i].active = true;
    slots[i].expired = false;
    slots[i].recurring = true;
    slots[i].handle = h;
    slots[i].callback = callback;
    slots[i].user_data = user_data;
    slots[i].name = name;
    slots[i].fire_gnss_ns = -1;
    slots[i].isr_callback = true;
    slots[i].isr_callback_fired = false;
    slots[i].rearm_in_isr = true;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;

    if (!configure_anchored_recurring_slot(slots[i],
                                           base_gnss_ns,
                                           period_gnss_ns,
                                           timepop_arm_source_t::ANCHORED_RECURRING_ISR_ARM)) {
      slots[i] = {};
      diag_arm_failures++;
      critical_exit(saved);
      return TIMEPOP_INVALID_HANDLE;
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

static timepop_handle_t arm_anchored_recurring_isr_from_counter32_internal(
  int64_t             base_gnss_ns,
  uint32_t            base_counter32,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (base_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  if (period_gnss_ns == 0) return TIMEPOP_INVALID_HANDLE;

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
    slots[i].active = true;
    slots[i].expired = false;
    slots[i].recurring = true;
    slots[i].handle = h;
    slots[i].callback = callback;
    slots[i].user_data = user_data;
    slots[i].name = name;
    slots[i].fire_gnss_ns = -1;
    slots[i].isr_callback = true;
    slots[i].isr_callback_fired = false;
    slots[i].rearm_in_isr = true;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;

    if (!configure_anchored_recurring_slot_from_counter32(
            slots[i],
            base_gnss_ns,
            base_counter32,
            period_gnss_ns,
            timepop_arm_source_t::ANCHORED_RECURRING_ISR_ARM)) {
      slots[i] = {};
      diag_arm_failures++;
      critical_exit(saved);
      return TIMEPOP_INVALID_HANDLE;
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

static timepop_handle_t arm_relative_slot_internal(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback,
  bool                rearm_in_isr
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (rearm_in_isr && (!recurring || !isr_callback)) return TIMEPOP_INVALID_HANDLE;

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
    slots[i].isr_callback = isr_callback;
    slots[i].isr_callback_fired = false;
    slots[i].rearm_in_isr = rearm_in_isr;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_base_fixed = false;
    slots[i].recurring_base_counter32_fixed = false;
    slots[i].recurring_base_counter32 = 0;
    slots[i].recurring_next_index = 0;
    slots[i].recurring_last_skipped_intervals = 0;
    slots[i].recurring_total_skipped_intervals = 0;
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
      // subsequent fires advance by the exact period.  Critical ISR recurring
      // slots keep this phase grid when possible, but will rearm from the
      // captured IRQ event if the anchor is temporarily unavailable.
      if (!configure_phase_locked_recurring_slot(slots[i], delay_gnss_ns)) {
        slots[i].recurrence_mode = timepop_recurrence_mode_t::RELATIVE;
        slots[i].recurring_period_gnss_ns = delay_gnss_ns;
      }
    }

    slots[i].predicted_dwt = predict_dwt_at_deadline(
      slots[i].deadline, slots[i].prediction_valid);

    if (slots[i].prediction_valid && slots[i].target_gnss_ns < 0) {
      slots[i].target_gnss_ns = time_gnss_ns_at_dwt(slots[i].predicted_dwt);
    }

    record_slot_arm_diag(slots[i],
                         slots[i].is_absolute
                             ? timepop_arm_source_t::RELATIVE_ARM_PHASE_LOCKED
                             : timepop_arm_source_t::RELATIVE_ARM,
                         true,
                         vclock_count(),
                         slots[i].target_gnss_ns);

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

timepop_handle_t timepop_arm(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_relative_slot_internal(delay_gnss_ns,
                                    recurring,
                                    callback,
                                    user_data,
                                    name,
                                    false,
                                    false);
}

timepop_handle_t timepop_arm_recurring_isr(
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_relative_slot_internal(period_gnss_ns,
                                    true,
                                    callback,
                                    user_data,
                                    name,
                                    true,
                                    true);
}


timepop_handle_t timepop_arm_recurring_isr_from_base(
  int64_t             base_gnss_ns,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_anchored_recurring_isr_internal(base_gnss_ns,
                                             period_gnss_ns,
                                             callback,
                                             user_data,
                                             name);
}

timepop_handle_t timepop_arm_recurring_isr_from_base_counter32(
  int64_t             base_gnss_ns,
  uint32_t            base_counter32,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_anchored_recurring_isr_from_counter32_internal(base_gnss_ns,
                                                           base_counter32,
                                                           period_gnss_ns,
                                                           callback,
                                                           user_data,
                                                           name);
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

    // Keep the slot quarantined as expired while its scheduled-context
    // callback runs.  A callback is allowed to arm/cancel other TimePop work,
    // and those operations may call schedule_next().  If we clear expired
    // before the recurring rearm has installed a new deadline, schedule_next()
    // can rediscover this same slot with its old, already-past deadline and
    // manufacture a false SCHEDULE_NEXT expiry.  The slot remains expired until
    // its next appointment is fully authored below.
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

    // The callback may have cancelled or otherwise retired this slot.  Do not
    // resurrect it by performing the normal recurring rearm path afterward.
    if (!slots[i].active) {
      slots[i].expired = false;
      slots[i].isr_callback_fired = false;
      continue;
    }

    // ── Recurring rearm ──
    if (slots[i].recurring) {
      if (slots[i].recurrence_mode == timepop_recurrence_mode_t::ABSOLUTE &&
          slots[i].recurring_period_gnss_ns > 0 &&
          slots[i].target_gnss_ns > 0) {
        int64_t next_target_gnss_ns = slots[i].target_gnss_ns + (int64_t)slots[i].recurring_period_gnss_ns;
        const int64_t now_gnss_ns = time_gnss_ns_now();
        if (slots[i].recurring_base_fixed && slots[i].recurring_base_gnss_ns > 0) {
          uint32_t skipped = 0;
          if (!anchored_next_target_gnss_ns(slots[i].recurring_base_gnss_ns,
                                            slots[i].recurring_period_gnss_ns,
                                            now_gnss_ns,
                                            next_target_gnss_ns,
                                            &skipped)) {
            slots[i].active = false;
            slots[i].expired = false;
            slots[i].isr_callback_fired = false;
            continue;
          }
          if (next_target_gnss_ns > slots[i].target_gnss_ns + (int64_t)slots[i].recurring_period_gnss_ns) {
            slots[i].recurring_immediate_expire_count++;
            slots[i].recurring_catchup_count += skipped;
          }
          slots[i].recurring_last_skipped_intervals = skipped;
          slots[i].recurring_total_skipped_intervals += skipped;
        } else if (now_gnss_ns >= 0 && next_target_gnss_ns <= now_gnss_ns) {
          const uint64_t missed =
              (uint64_t)((now_gnss_ns - next_target_gnss_ns) / (int64_t)slots[i].recurring_period_gnss_ns) + 1ULL;
          next_target_gnss_ns += (int64_t)(missed * slots[i].recurring_period_gnss_ns);
          slots[i].recurring_immediate_expire_count++;
          slots[i].recurring_catchup_count += (uint32_t)missed;
          slots[i].recurring_last_skipped_intervals = (uint32_t)missed;
          slots[i].recurring_total_skipped_intervals += (uint32_t)missed;
        }

        const time_anchor_snapshot_t snap =
            timepop_anchor_snapshot("dispatch_rearm");
        uint32_t next_deadline = 0;
        if (!gnss_ns_to_vclock_deadline(next_target_gnss_ns, snap, next_deadline)) {
          slots[i].active = false;
          slots[i].expired = false;
          slots[i].isr_callback_fired = false;
        } else {
          slots[i].target_gnss_ns = next_target_gnss_ns;
          slots[i].deadline = next_deadline;
          slots[i].recurring_rearmed_count++;
          slots[i].predicted_dwt = predict_dwt_at_deadline(
            slots[i].deadline, slots[i].prediction_valid);
          slots[i].expired = false;
          record_slot_arm_diag(slots[i],
                               timepop_arm_source_t::DISPATCH_ABSOLUTE_REARM,
                               true,
                               vclock_count(),
                               slots[i].target_gnss_ns);
          const uint32_t saved = critical_enter();
          schedule_next();
          critical_exit(saved);
        }
      } else if (slots[i].period_ticks == 0) {
        slots[i].active = false;
        slots[i].expired = false;
        slots[i].isr_callback_fired = false;
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

          // Catch-up: if we fell behind, skip forward.  Equality belongs to the
          // hardware compare path; foreground catch-up only handles already-
          // past deadlines.
          uint32_t now = vclock_count();
          if (deadline_passed(slots[i].deadline, now)) {
            slots[i].recurring_immediate_expire_count++;
            slots[i].deadline = now + slots[i].period_ticks;
            slots[i].recurring_catchup_count++;
          }
        }

        slots[i].predicted_dwt = predict_dwt_at_deadline(
          slots[i].deadline, slots[i].prediction_valid);

        if (slots[i].prediction_valid && slots[i].target_gnss_ns < 0) {
          slots[i].target_gnss_ns = time_gnss_ns_at_dwt(slots[i].predicted_dwt);
        }

        slots[i].expired = false;
        record_slot_arm_diag(slots[i],
                             slots[i].is_absolute
                                 ? timepop_arm_source_t::DISPATCH_PHASE_LOCKED_REARM
                                 : timepop_arm_source_t::DISPATCH_RELATIVE_REARM,
                             true,
                             vclock_count(),
                             slots[i].target_gnss_ns);
        const uint32_t saved = critical_enter();
        schedule_next();
        critical_exit(saved);
      }
    } else {
      slots[i].active = false;
      slots[i].expired = false;
      slots[i].isr_callback_fired = false;
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
// Epoch boundary
// ============================================================================
//
// A VCLOCK epoch change rebases the synthetic counter coordinate system used
// by every timed TimePop deadline.  No pre-existing timed deadline may survive
// that boundary.  Recurring slots are re-authored into the new epoch so core
// services such as transport polling can continue; one-shot timed slots are
// cancelled because their old-epoch target cannot be interpreted safely.

void timepop_epoch_changed(uint32_t epoch_sequence) {
  const uint32_t saved = critical_enter();

  const time_anchor_snapshot_t epoch_anchor =
      timepop_anchor_snapshot("epoch_changed");
  diag_epoch_last_anchor_ok = epoch_anchor.ok;
  diag_epoch_last_anchor_valid = epoch_anchor.valid;
  diag_epoch_last_anchor_pps_count = epoch_anchor.pps_count;
  diag_epoch_last_anchor_dwt_at_pps = epoch_anchor.dwt_at_pps;
  diag_epoch_last_anchor_cycles_per_s = epoch_anchor.dwt_cycles_per_s;
  diag_epoch_last_anchor_qtimer_at_pps = epoch_anchor.qtimer_at_pps;
  diag_epoch_last_anchor_counter32_at_pps_vclock =
      epoch_anchor.counter32_at_pps_vclock;
  diag_epoch_last_schedule_next_calls_before = diag_schedule_next_calls_total;

  uint32_t timed_seen = 0;
  uint32_t recurring_rearmed = 0;
  uint32_t recurring_failures = 0;
  uint32_t one_shot_cancelled = 0;
  uint32_t expired_cleared = 0;
  uint32_t asap_active = 0;
  uint32_t alap_active = 0;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active && !slots[i].expired) continue;
    timed_seen++;
    if (slots[i].expired) expired_cleared++;

    if (slots[i].active && slots[i].recurring) {
      if (rearm_recurring_slot_for_epoch(slots[i])) {
        recurring_rearmed++;
      } else {
        slots[i] = {};
        recurring_failures++;
      }
    } else {
      slots[i] = {};
      one_shot_cancelled++;
    }
  }

  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active) asap_active++;
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active) alap_active++;
  }

  // Timed expirations from the old epoch are invalid.  Deferred ASAP/ALAP
  // callbacks are not VCLOCK-deadline facts, so leave them in place.
  timepop_pending = (asap_active > 0 || alap_active > 0);

  diag_epoch_change_count++;
  diag_epoch_last_sequence = epoch_sequence;
  diag_epoch_timed_slots_seen += timed_seen;
  diag_epoch_recurring_rearmed += recurring_rearmed;
  diag_epoch_recurring_rearm_failures += recurring_failures;
  diag_epoch_one_shot_cancelled += one_shot_cancelled;
  diag_epoch_expired_cleared += expired_cleared;
  diag_epoch_asap_left_active += asap_active;
  diag_epoch_alap_left_active += alap_active;

  diag_schedule_next_calls_from_other++;
  schedule_next();
  diag_epoch_last_schedule_next_calls_after = diag_schedule_next_calls_total;
  update_slot_high_water();

  critical_exit(saved);
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
  out.add("isr_recurring_rearmed", diag_isr_recurring_rearmed);
  out.add("isr_recurring_rearm_failures", diag_isr_recurring_rearm_failures);
  out.add("phantom_count",     phantom_count);
  out.add("expired",           expired_count);
  out.add("rearm_count",       diag_rearm_count);
  out.add("heartbeat_rearms",  diag_heartbeat_rearms);
  out.add("race_recoveries",   diag_race_recoveries);
  out.add("vclock_raw_now",    vclock_count());
  out.add("time_valid",        time_valid());
  out.add("time_pps_count",    time_pps_count());

  out.add("anchor_snapshot_count", diag_anchor_snapshot_count);
  out.add("anchor_snapshot_ok_count", diag_anchor_snapshot_ok_count);
  out.add("anchor_snapshot_not_ok_count", diag_anchor_snapshot_not_ok_count);
  out.add("anchor_snapshot_invalid_count", diag_anchor_snapshot_invalid_count);
  out.add("anchor_snapshot_zero_pps_count", diag_anchor_snapshot_zero_pps_count);
  out.add("anchor_snapshot_zero_cps_count", diag_anchor_snapshot_zero_cps_count);
  out.add("anchor_snapshot_last_context",
          (const char*)(diag_anchor_snapshot_last_context
                            ? diag_anchor_snapshot_last_context
                            : ""));
  out.add("anchor_snapshot_last_bad_context",
          (const char*)(diag_anchor_snapshot_last_bad_context
                            ? diag_anchor_snapshot_last_bad_context
                            : ""));
  out.add("anchor_snapshot_last_ok", diag_anchor_snapshot_last_ok);
  out.add("anchor_snapshot_last_valid", diag_anchor_snapshot_last_valid);
  out.add("anchor_snapshot_last_pps_count", diag_anchor_snapshot_last_pps_count);
  out.add("anchor_snapshot_last_dwt_at_pps", diag_anchor_snapshot_last_dwt_at_pps);
  out.add("anchor_snapshot_last_cycles_per_s", diag_anchor_snapshot_last_cycles_per_s);
  out.add("anchor_snapshot_last_qtimer_at_pps", diag_anchor_snapshot_last_qtimer_at_pps);
  out.add("anchor_snapshot_last_counter32_at_pps_vclock",
          diag_anchor_snapshot_last_counter32_at_pps_vclock);

  out.add("slots_active_now",       timepop_active_count());
  out.add("slots_high_water",       diag_slots_high_water);
  out.add("slots_max",              (uint32_t)MAX_SLOTS);
  out.add("arm_failures",           diag_arm_failures);
  out.add("named_replacements",     diag_named_replacements);
  out.add("epoch_change_count",     diag_epoch_change_count);
  out.add("epoch_last_sequence",    diag_epoch_last_sequence);
  out.add("epoch_timed_slots_seen", diag_epoch_timed_slots_seen);
  out.add("epoch_recurring_rearmed", diag_epoch_recurring_rearmed);
  out.add("epoch_recurring_rearm_failures", diag_epoch_recurring_rearm_failures);
  out.add("epoch_one_shot_cancelled", diag_epoch_one_shot_cancelled);
  out.add("epoch_expired_cleared",  diag_epoch_expired_cleared);
  out.add("epoch_asap_left_active", diag_epoch_asap_left_active);
  out.add("epoch_alap_left_active", diag_epoch_alap_left_active);
  out.add("epoch_last_anchor_ok", diag_epoch_last_anchor_ok);
  out.add("epoch_last_anchor_valid", diag_epoch_last_anchor_valid);
  out.add("epoch_last_anchor_pps_count", diag_epoch_last_anchor_pps_count);
  out.add("epoch_last_anchor_dwt_at_pps", diag_epoch_last_anchor_dwt_at_pps);
  out.add("epoch_last_anchor_cycles_per_s", diag_epoch_last_anchor_cycles_per_s);
  out.add("epoch_last_anchor_qtimer_at_pps", diag_epoch_last_anchor_qtimer_at_pps);
  out.add("epoch_last_anchor_counter32_at_pps_vclock",
          diag_epoch_last_anchor_counter32_at_pps_vclock);
  out.add("epoch_last_schedule_next_calls_before",
          diag_epoch_last_schedule_next_calls_before);
  out.add("epoch_last_schedule_next_calls_after",
          diag_epoch_last_schedule_next_calls_after);

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
  out.add("schedule_next_expired_passes",      diag_schedule_next_expired_passes);
  out.add("schedule_next_expired_slots",       diag_schedule_next_expired_slots);
  out.add("schedule_next_late_max_ticks",      diag_schedule_next_late_max_ticks);
  out.add("schedule_next_last_expired_slot",   diag_schedule_next_last_expired_slot);
  out.add("schedule_next_last_expired_handle", diag_schedule_next_last_expired_handle);
  out.add("schedule_next_last_expired_name",
          (const char*)(diag_schedule_next_last_expired_name
                            ? diag_schedule_next_last_expired_name
                            : ""));
  out.add("schedule_next_last_expired_deadline",
          diag_schedule_next_last_expired_deadline);
  out.add("schedule_next_last_expired_now",
          diag_schedule_next_last_expired_now);
  out.add("schedule_next_last_expired_late_ticks",
          diag_schedule_next_last_expired_late_ticks);
  out.add("schedule_next_last_expired_dwt",
          diag_schedule_next_last_expired_dwt);

  out.add("arm_already_past_count",            diag_arm_already_past_count);
  out.add("arm_last_slot",                     diag_arm_last_slot);
  out.add("arm_last_handle",                   diag_arm_last_handle);
  out.add("arm_last_name",
          (const char*)(diag_arm_last_name ? diag_arm_last_name : ""));
  out.add("arm_last_source",
          (const char*)(diag_arm_last_source ? diag_arm_last_source : ""));
  out.add("arm_last_now",                      diag_arm_last_now);
  out.add("arm_last_deadline",                 diag_arm_last_deadline);
  out.add("arm_last_delta_ticks",              diag_arm_last_delta_ticks);
  out.add("arm_last_target_gnss_ns",           diag_arm_last_target_gnss_ns);
  out.add("arm_last_dwt",                      diag_arm_last_dwt);
  out.add("arm_last_already_past",             diag_arm_last_already_past);
  out.add("arm_last_had_now",                  diag_arm_last_had_now);

  out.add("irq_grid_audit_count",              diag_irq_grid_audit_count);
  out.add("irq_grid_last_now",                 diag_irq_grid_last_now);
  out.add("irq_grid_last_dwt",                 diag_irq_grid_last_dwt);
  out.add("irq_grid_onehz_due_count",          diag_irq_grid_onehz_due_count);
  out.add("irq_grid_onehz_expired_count",      diag_irq_grid_onehz_expired_count);
  out.add("irq_grid_witness_slot",             diag_irq_grid_witness_slot);
  out.add("irq_grid_witness_handle",           diag_irq_grid_witness_handle);
  out.add("irq_grid_witness_seen",             diag_irq_grid_witness_seen);
  out.add("irq_grid_witness_active",           diag_irq_grid_witness_active);
  out.add("irq_grid_witness_expired_before",   diag_irq_grid_witness_expired_before);
  out.add("irq_grid_witness_reached_before",   diag_irq_grid_witness_reached_before);
  out.add("irq_grid_witness_passed_before",    diag_irq_grid_witness_passed_before);
  out.add("irq_grid_witness_expired_by_irq",   diag_irq_grid_witness_expired_by_irq);
  out.add("irq_grid_witness_deadline",         diag_irq_grid_witness_deadline);
  out.add("irq_grid_witness_distance_ticks",   diag_irq_grid_witness_distance_ticks);
  out.add("irq_grid_witness_late_ticks",       diag_irq_grid_witness_late_ticks);
  out.add("irq_grid_witness_missed_count",     diag_irq_grid_witness_missed_count);
  out.add("irq_grid_witness_not_active_count", diag_irq_grid_witness_not_active_count);
  out.add("irq_grid_witness_already_expired_count",
          diag_irq_grid_witness_already_expired_count);

  out.add("irq_expired_slots",                 diag_irq_expired_slots);
  out.add("irq_exact_deadline_slots",          diag_irq_exact_deadline_slots);
  out.add("irq_late_deadline_slots",           diag_irq_late_deadline_slots);
  out.add("irq_late_max_ticks",                diag_irq_late_max_ticks);
  out.add("irq_event_gnss_mismatch_count",     diag_irq_event_gnss_mismatch_count);
  out.add("irq_event_gnss_last_delta_ns",      diag_irq_event_gnss_last_delta_ns);

  out.add("deadline_convert_count",            diag_deadline_convert_count);
  out.add("deadline_convert_success_count",    diag_deadline_convert_success_count);
  out.add("deadline_fail_no_anchor",           diag_deadline_fail_no_anchor);
  out.add("deadline_fail_invalid_anchor",      diag_deadline_fail_invalid_anchor);
  out.add("deadline_fail_zero_pps_count",      diag_deadline_fail_zero_pps_count);
  out.add("deadline_negative_offset",          diag_deadline_negative_offset);
  out.add("deadline_fail_range",               diag_deadline_fail_range);
  out.add("deadline_last_target_gnss_ns",      diag_deadline_last_target_gnss_ns);
  out.add("deadline_last_anchor_pps_gnss_ns",  diag_deadline_last_anchor_pps_gnss_ns);
  out.add("deadline_last_ns_from_anchor",      diag_deadline_last_ns_from_anchor);
  out.add("deadline_last_deadline",            diag_deadline_last_deadline);
  out.add("deadline_last_failure",
          (const char*)(diag_deadline_last_failure
                            ? diag_deadline_last_failure
                            : ""));
  out.add("deadline_last_anchor_ok",           diag_deadline_last_anchor_ok);
  out.add("deadline_last_anchor_valid",        diag_deadline_last_anchor_valid);
  out.add("deadline_last_anchor_pps_count",    diag_deadline_last_anchor_pps_count);
  out.add("deadline_last_anchor_qtimer_at_pps", diag_deadline_last_anchor_qtimer_at_pps);
  out.add("deadline_last_anchor_dwt_at_pps",   diag_deadline_last_anchor_dwt_at_pps);
  out.add("deadline_last_anchor_cycles_per_s", diag_deadline_last_anchor_cycles_per_s);

  out.add("qtmr1_ch2_cntr",   (uint32_t)interrupt_qtimer1_ch2_counter_now());
  out.add("qtmr1_ch2_comp1",  (uint32_t)interrupt_qtimer1_ch2_comp1_now());
  out.add("qtmr1_ch2_csctrl", (uint32_t)interrupt_qtimer1_ch2_csctrl_now());

  // Detailed per-slot diagnostics are intentionally excluded from REPORT.
  // Use TIMEPOP.SLOTS when the full slot table is needed.


  return out;
}

// ============================================================================
// QTimer1 vector ownership moved to process_interrupt
// ============================================================================
//
// process_interrupt owns IRQ_QTIMER1 and dispatches CH2 to TimePop's
// registered handler.  VCLOCK cadence and any other white-glove substrate
// maintenance clients are represented as TimePop slots; critical recurring
// ISR slots can callback and rearm inside this CH2 IRQ pass before the next
// compare is selected.


static void add_timers_array(Payload& out) {
  PayloadArray timers;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;

    Payload entry;

    // Compact operational slot table.
    //
    // Keep this intentionally small: TIMEPOP.REPORT carries the global
    // forensic counters, while TIMEPOP.SLOTS should survive the transport /
    // Payload element ceiling even when the table is crowded.  The fields
    // below identify the slot, its next deadline, its scheduling class, and
    // the only remaining per-slot health signals we need after the witness
    // quarantine bug was found.
    entry.add("slot",      i);
    entry.add("handle",    slots[i].handle);
    entry.add("name",      slots[i].name ? slots[i].name : "");
    entry.add("deadline",  slots[i].deadline);
    entry.add("expired",   slots[i].expired);

    entry.add("period_ns", slots[i].period_ns);
    entry.add("ticks",     slots[i].period_ticks);
    entry.add("target_gnss_ns", slots[i].target_gnss_ns);

    entry.add("recurring", slots[i].recurring);
    entry.add("recurring_base_fixed", slots[i].recurring_base_fixed);
    entry.add("recurring_base_counter32_fixed", slots[i].recurring_base_counter32_fixed);
    entry.add("recurring_base_counter32", slots[i].recurring_base_counter32);
    entry.add("recurring_next_index", (uint32_t)(slots[i].recurring_next_index & 0xFFFFFFFFULL));
    entry.add("recurring_base_gnss_ns", slots[i].recurring_base_gnss_ns);
    entry.add("recurring_last_skipped_intervals", slots[i].recurring_last_skipped_intervals);
    entry.add("recurring_total_skipped_intervals", slots[i].recurring_total_skipped_intervals);
    entry.add("is_absolute", slots[i].is_absolute);
    entry.add("isr_cb",    slots[i].isr_callback);
    entry.add("rearm_in_isr", slots[i].rearm_in_isr);

    entry.add("schedule_next_expired_count", slots[i].schedule_next_expired_count);
    entry.add("schedule_next_late_max_ticks", slots[i].schedule_next_late_max_ticks);
    entry.add("irq_expired_by_irq_count", slots[i].irq_expired_by_irq_count);
    entry.add("last_arm_delta_ticks", slots[i].last_arm_delta_ticks);

    timers.add(entry);
  }
  out.add_array("timers", timers);
}

static Payload cmd_slots(const Payload&) {
  Payload out;
  out.add("slots_active_now", timepop_active_count());
  out.add("slots_high_water", diag_slots_high_water);
  out.add("slots_max", (uint32_t)MAX_SLOTS);
  out.add("vclock_raw_now", vclock_count());
  out.add("time_valid", time_valid());
  out.add("time_pps_count", time_pps_count());
  out.add("anchor_snapshot_last_ok", diag_anchor_snapshot_last_ok);
  out.add("anchor_snapshot_last_valid", diag_anchor_snapshot_last_valid);
  out.add("anchor_snapshot_last_pps_count", diag_anchor_snapshot_last_pps_count);
  out.add("anchor_snapshot_zero_pps_count", diag_anchor_snapshot_zero_pps_count);
  out.add("deadline_fail_zero_pps_count", diag_deadline_fail_zero_pps_count);
  out.add("deadline_last_failure",
          (const char*)(diag_deadline_last_failure
                            ? diag_deadline_last_failure
                            : ""));
  out.add("epoch_change_count", diag_epoch_change_count);
  out.add("epoch_last_anchor_pps_count", diag_epoch_last_anchor_pps_count);
  out.add("schedule_next_expired_passes", diag_schedule_next_expired_passes);
  out.add("schedule_next_expired_slots", diag_schedule_next_expired_slots);
  out.add("schedule_next_late_max_ticks", diag_schedule_next_late_max_ticks);
  out.add("irq_expired_slots", diag_irq_expired_slots);
  out.add("irq_exact_deadline_slots", diag_irq_exact_deadline_slots);
  out.add("irq_late_deadline_slots", diag_irq_late_deadline_slots);
  out.add("isr_recurring_rearmed", diag_isr_recurring_rearmed);
  out.add("isr_recurring_rearm_failures", diag_isr_recurring_rearm_failures);
  add_timers_array(out);
  return out;
}

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT",                 cmd_report                 },
  { "SLOTS",                  cmd_slots                  },
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