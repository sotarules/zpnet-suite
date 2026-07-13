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
//   • safe ASAP dispatch         — timepop_arm_asap(...)
//     Fixed deferred lane, not a timed slot; ISR-safe and drained before slots.
//   • safe ALAP dispatch         — timepop_arm_alap(...)
//     Fixed deferred lane, not a timed slot; ISR-safe and drained after slots.
//   • absolute scheduling        — timepop_arm_at(target_gnss_ns, ...)
//   • anchor-relative scheduling — timepop_arm_from_anchor(anchor_gnss_ns, offset_gnss_ns, ...)
//   • caller-owned exact path    — timepop_arm_ns(target_gnss_ns, target_dwt, ...)
//   • timed slot priority       — lower numeric priority runs first when
//     multiple timed slots share one captured CH2 fire fact. ASAP/ALAP are
//     fixed deferred lanes and intentionally do not participate.
//   • critical scheduler recurring — timepop_arm_recurring_isr(period_ns, ...)
//     The callback and recurring rearm happen inside the CH2 scheduler pass
//     before schedule_next() selects the next compare.  Use only for tiny substrate
//    maintenance callbacks that TimePop itself depends on for safe scheduling.
//   • counter-base anchored ISR recurring
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
//   from its priority-handoff context with a normalized DWT capture and synthetic VCLOCK
//   counter32 event identity.  TimePop uses that event fact to expire slots.
//   It does not reinterpret PPS/VCLOCK phase and it does not read timer
//   hardware directly.
//
//   Only timed slots whose authored deadline exactly equals the physical CH2
//   event counter receive a callback.  All slots exactly matched by the same
//   CH2 event receive the same fire_vclock_raw, fire_dwt_cyccnt, and
//   fire_gnss_ns.  Already-past deadlines are missed edges, not late fires;
//   TimePop quarantines them without callback and re-authors recurring grids.
//   Callback order is logical only; priority only orders callbacks after the
//   shared physical fire facts have been captured for every exact-match slot.
//   It must never re-author the physical timing facts.
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
#include <strings.h>
#include <math.h>
#include <climits>

// ============================================================================
// Constants — 10 MHz domain
// ============================================================================

static constexpr uint32_t MAX_SLOTS = 16;
static constexpr uint32_t MAX_ASAP_SLOTS = 8;
static constexpr uint32_t MAX_ALAP_SLOTS = 4;
static constexpr uint32_t MAX_DISPATCH_MUTATIONS = 24;
static constexpr uint32_t MAX_DISPATCH_NAME = 63;
static constexpr bool     TIMEPOP_IDLE_DWT_WITNESS_ENABLED = true;
static constexpr uint64_t NS_PER_TICK = 100ULL;
static constexpr uint32_t MAX_DELAY_TICKS = 0x7FFFFFFFU;
static constexpr uint32_t MIN_DELAY_TICKS = 2;
// Minimum safe lead when programming QTimer1 CH2 from scheduler context.
// A deadline closer than this is treated as already missed/too-close;
// TimePop must not arm CH2 at-or-inside the hardware race window.
static constexpr uint32_t SCHEDULE_MIN_ARM_LEAD_TICKS = 64;
static constexpr uint32_t HEARTBEAT_TICKS = 10000;
static constexpr uint32_t PREDICT_MAX_QTIMER_ELAPSED = 15000000U;
static constexpr uint32_t ONE_HZ_TICKS = 10000000U;
static constexpr bool     TIMEPOP_SLOT_PRIORITY_ORDERING_ENABLED = true;
static constexpr const char* WITNESS_SCHEDULER_NAME = "WITNESS_SCHEDULER";

// CH2 scheduler custody.
//
// process_interrupt now owns the priority handoff tier.  By the time
// process_interrupt invokes TimePop's registered CH2 handler, the priority-0
// hardware capture has already escaped into process_interrupt's priority-16
// handoff context.  TimePop therefore processes CH2 event facts directly; it
// does not own a private scheduler IRQ or second CH2 queue.
static constexpr const char* TIMEPOP_CH2_SERVICE_CONTEXT = "PROCESS_INTERRUPT_HANDOFF_DIRECT";


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
  MISSED_DEADLINE_REARM,
  MISSED_DEADLINE_REARM_FALLBACK,
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
    case timepop_arm_source_t::MISSED_DEADLINE_REARM: return "MISSED_DEADLINE_REARM";
    case timepop_arm_source_t::MISSED_DEADLINE_REARM_FALLBACK: return "MISSED_DEADLINE_REARM_FALLBACK";
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
  char                queued_name[MAX_DISPATCH_NAME + 1];

  // Timed-slot service priority. Lower numeric value runs first when multiple
  // slots share the same captured CH2 fire fact. This never affects compare
  // scheduling and never applies to ASAP/ALAP deferred lanes.
  timepop_priority_t  priority;

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
  // Dedicated non-timed deferred lane.
  //
  // ASAP/ALAP callbacks are not TimePop scheduled slots and never own a
  // VCLOCK deadline.  They may be armed from ISR context: arming only writes
  // this fixed mailbox under PRIMASK and sets timepop_pending.  It does not
  // scan/mutate/reorder the timed slot table and it never calls schedule_next().
  //
  // pending     — a callback is waiting for scheduled-context dispatch.
  // dispatching — this entry's callback is currently running in foreground.
  //               If the same named callback is re-armed while it is running,
  //               pending is set again and the new request survives the
  //               dispatch epilogue for the next pass.
  volatile bool       pending;
  volatile bool       dispatching;
  timepop_handle_t    handle;          // pending request handle
  timepop_callback_t  callback;        // pending request callback
  void*               user_data;       // pending request user data
  const char*         name;            // pending request name

  timepop_handle_t    dispatch_handle; // currently-running request handle
  const char*         dispatch_name;   // currently-running request name
  uint32_t            generation;
};

// ============================================================================
// Dispatch-time timed-slot mutation barrier
// ============================================================================
//
// Public timed TimePop APIs are valid from scheduled-context callbacks,
// including ASAP and ALAP. During a dispatch pass, user callbacks must not
// directly mutate the active timed-slot table or reprogram CH2 while TimePop
// is walking callback state. Such requests are copied into this fixed,
// allocation-free mutation queue and applied at explicit dispatch barriers
// immediately after the current callback returns.
//
// Deferred ASAP/ALAP arming remains immediate because those lanes are fixed
// mailboxes, not timed slots, and do not call schedule_next().

enum class timepop_dispatch_phase_t : uint8_t {
  IDLE = 0,
  ASAP,
  TIMED,
  ALAP,
  APPLYING_MUTATIONS,
};

static const char* dispatch_phase_str(timepop_dispatch_phase_t phase) {
  switch (phase) {
    case timepop_dispatch_phase_t::ASAP: return "ASAP";
    case timepop_dispatch_phase_t::TIMED: return "TIMED";
    case timepop_dispatch_phase_t::ALAP: return "ALAP";
    case timepop_dispatch_phase_t::APPLYING_MUTATIONS: return "APPLYING_MUTATIONS";
    default: return "IDLE";
  }
}

enum class timepop_dispatch_mutation_kind_t : uint8_t {
  NONE = 0,
  ARM_RELATIVE,
  ARM_ABSOLUTE,
  ARM_ANCHORED_ISR,
  ARM_ANCHORED_ISR_COUNTER32,
  CANCEL_HANDLE,
  CANCEL_NAME,
};

static const char* dispatch_mutation_kind_str(timepop_dispatch_mutation_kind_t kind) {
  switch (kind) {
    case timepop_dispatch_mutation_kind_t::ARM_RELATIVE: return "ARM_RELATIVE";
    case timepop_dispatch_mutation_kind_t::ARM_ABSOLUTE: return "ARM_ABSOLUTE";
    case timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR: return "ARM_ANCHORED_ISR";
    case timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR_COUNTER32: return "ARM_ANCHORED_ISR_COUNTER32";
    case timepop_dispatch_mutation_kind_t::CANCEL_HANDLE: return "CANCEL_HANDLE";
    case timepop_dispatch_mutation_kind_t::CANCEL_NAME: return "CANCEL_NAME";
    default: return "NONE";
  }
}

struct timepop_dispatch_mutation_t {
  timepop_dispatch_mutation_kind_t kind = timepop_dispatch_mutation_kind_t::NONE;
  timepop_handle_t reserved_handle = TIMEPOP_INVALID_HANDLE;
  timepop_handle_t cancel_handle = TIMEPOP_INVALID_HANDLE;

  uint64_t ns0 = 0;       // delay_ns / period_ns / recurring period
  int64_t  gnss0 = -1;    // target_gnss_ns / base_gnss_ns
  uint32_t u32_0 = 0;     // target_dwt / base_counter32

  bool recurring = false;
  bool isr_callback = false;
  bool rearm_in_isr = false;
  timepop_callback_t callback = nullptr;
  void* user_data = nullptr;
  char name[MAX_DISPATCH_NAME + 1] = {};
  timepop_priority_t priority = TIMEPOP_PRIORITY_DEFAULT;
  timepop_dispatch_phase_t queued_phase = timepop_dispatch_phase_t::IDLE;
};

// ============================================================================
// State
// ============================================================================

static timepop_slot_t slots[MAX_SLOTS];
static deferred_slot_t asap_slots[MAX_ASAP_SLOTS];
static deferred_slot_t alap_slots[MAX_ALAP_SLOTS];
static volatile bool   timepop_pending = false;
static uint32_t        next_handle = 1;

static timepop_dispatch_mutation_t dispatch_mutations[MAX_DISPATCH_MUTATIONS];
static volatile uint32_t dispatch_mutation_count = 0;
static volatile uint32_t dispatch_depth = 0;
static volatile timepop_dispatch_phase_t dispatch_phase = timepop_dispatch_phase_t::IDLE;
static volatile bool dispatch_applying_mutations = false;

// Global idle DWT witness.  This symbol is intentionally exported so ISR-side
// clients may read the freshest foreground/thread-mode DWT shadow without
// calling into TimePop.  It is written only by the idle witness loop below.
volatile uint32_t g_timepop_idle_witness_shadow_dwt = 0;

static volatile bool     diag_idle_witness_running = false;
static volatile uint32_t diag_idle_witness_enter_count = 0;
static volatile uint32_t diag_idle_witness_exit_count = 0;
static volatile uint32_t diag_idle_witness_pending_exit_count = 0;
static volatile uint32_t diag_idle_witness_last_enter_dwt = 0;
static volatile uint32_t diag_idle_witness_last_exit_dwt = 0;
static volatile uint64_t diag_idle_witness_total_cycles = 0;
static volatile uint32_t diag_idle_witness_last_residency_cycles = 0;
static volatile uint64_t diag_idle_witness_wall_cycles = 0;
static volatile uint32_t diag_idle_witness_wall_last_dwt = 0;
static volatile bool     diag_idle_witness_wall_initialized = false;

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
static volatile uint32_t diag_asap_replacements       = 0;
static volatile uint32_t diag_asap_cancelled          = 0;
static volatile uint32_t diag_asap_arm_while_dispatching = 0;

static volatile uint32_t diag_alap_armed              = 0;
static volatile uint32_t diag_alap_dispatched         = 0;
static volatile uint32_t diag_alap_arm_failures       = 0;
static volatile uint32_t diag_alap_last_armed_dwt     = 0;
static volatile uint32_t diag_alap_last_dispatch_dwt  = 0;
static volatile const char* diag_alap_last_armed_name = nullptr;
static volatile uint32_t diag_alap_replacements       = 0;
static volatile uint32_t diag_alap_cancelled          = 0;
static volatile uint32_t diag_alap_arm_while_dispatching = 0;

static volatile uint32_t diag_dispatch_calls     = 0;
static volatile uint32_t diag_dispatch_callbacks = 0;

static volatile uint32_t diag_dispatch_depth_max = 0;
static volatile uint32_t diag_dispatch_mutation_queued = 0;
static volatile uint32_t diag_dispatch_mutation_applied = 0;
static volatile uint32_t diag_dispatch_mutation_apply_failures = 0;
static volatile uint32_t diag_dispatch_mutation_overflow = 0;
static volatile uint32_t diag_dispatch_mutation_high_water = 0;
static volatile uint32_t diag_dispatch_mutation_apply_passes = 0;
static volatile uint32_t diag_dispatch_mutation_schedule_next_calls = 0;
static volatile const char* diag_dispatch_mutation_last_kind = nullptr;
static volatile const char* diag_dispatch_mutation_last_context = nullptr;
static volatile const char* diag_dispatch_mutation_last_phase = nullptr;
static volatile uint32_t diag_dispatch_mutation_last_handle = 0;
static volatile uint32_t diag_dispatch_mutation_last_queue_depth = 0;
static volatile uint32_t diag_dispatch_timed_arm_queued = 0;
static volatile uint32_t diag_dispatch_cancel_queued = 0;
static volatile uint32_t diag_dispatch_mutation_coalesced = 0;
static volatile uint32_t diag_dispatch_mutation_cancel_applied = 0;
static volatile uint32_t diag_dispatch_mutation_cancel_noop = 0;
static volatile uint32_t diag_dispatch_mutation_cancel_failures = 0;
static volatile uint32_t diag_dispatch_mutation_arm_failures = 0;
static volatile uint32_t diag_dispatch_mutation_name_copies = 0;
static volatile uint32_t diag_dispatch_mutation_name_too_long = 0;

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

// Scheduler pressure diagnostics.
//
// These counters classify scheduler lateness without changing behavior. A
// too-close deadline is not a corrupted event fact; it means TimePop refused to
// arm CH2 inside the unsafe compare race window. This surface lets us decide
// whether regular scheduled-context work is simply delayed as designed, or
// whether the scheduler itself is experiencing pressure/starvation.
static volatile bool     diag_schedule_pressure_supported = true;
static volatile uint32_t diag_schedule_next_body_cycles_last = 0;
static volatile uint32_t diag_schedule_next_body_cycles_max = 0;
static volatile uint32_t diag_schedule_next_arm_margin_ticks_last = 0;
static volatile uint32_t diag_schedule_next_arm_margin_ticks_min = UINT32_MAX;
static volatile uint32_t diag_schedule_next_arm_margin_ticks_max = 0;
static volatile uint32_t diag_schedule_next_final_safety_count = 0;
static volatile uint32_t diag_schedule_next_too_close_count = 0;
static volatile uint32_t diag_schedule_next_exact_no_irq_count = 0;
static volatile uint32_t diag_schedule_next_passed_count = 0;
static volatile uint32_t diag_schedule_next_invalid_time_count = 0;
static volatile uint32_t diag_schedule_next_too_close_max_ticks = 0;
static volatile uint32_t diag_schedule_next_passed_late_max_ticks = 0;
static volatile const char* diag_schedule_next_last_pressure_source = nullptr;
static volatile const char* diag_schedule_next_last_pressure_name = nullptr;
static volatile uint32_t diag_schedule_next_last_pressure_deadline = 0;
static volatile uint32_t diag_schedule_next_last_pressure_now = 0;
static volatile uint32_t diag_schedule_next_last_pressure_distance_ticks = 0;
static volatile uint32_t diag_schedule_next_last_pressure_late_ticks = 0;

// Scheduled-context callback latency. This is intentionally distinct from
// event-fact latency. Regular work is allowed to run late; the question is
// whether the lateness stays bounded while timing facts remain safely captured.
static volatile uint32_t diag_timed_dispatch_latency_count = 0;
static volatile uint32_t diag_timed_dispatch_latency_last_cycles = 0;
static volatile uint32_t diag_timed_dispatch_latency_max_cycles = 0;
static volatile uint64_t diag_timed_dispatch_latency_cycles_sum = 0;
static volatile uint32_t diag_timed_dispatch_callback_body_last_cycles = 0;
static volatile uint32_t diag_timed_dispatch_callback_body_max_cycles = 0;
static volatile const char* diag_timed_dispatch_last_name = nullptr;

// CH2 scheduler-entry diagnostics.
//
// TimePop no longer owns a private priority handoff.  These counters measure
// the direct CH2 scheduler pass invoked by process_interrupt's handoff tier.
static volatile uint32_t diag_ch2_direct_call_count = 0;
static volatile uint32_t diag_ch2_direct_body_cycles_last = 0;
static volatile uint32_t diag_ch2_direct_body_cycles_max = 0;
static volatile uint32_t diag_ch2_direct_last_event_counter32 = 0;
static volatile uint32_t diag_ch2_direct_last_event_dwt = 0;


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

// Missed-deadline quarantine.  TimePop must never attach the current CH2
// event DWT to a slot whose authored deadline is already in the past.  These
// counters surface slots that were cancelled or re-authored because their
// exact compare edge was missed.
static volatile uint32_t diag_missed_deadline_slots = 0;
static volatile uint32_t diag_missed_deadline_one_shot_cancelled = 0;
static volatile uint32_t diag_missed_deadline_recurring_rearmed = 0;
static volatile uint32_t diag_missed_deadline_recurring_rearm_failures = 0;
static volatile uint32_t diag_missed_deadline_too_close_count = 0;
static volatile uint32_t diag_missed_deadline_late_max_ticks = 0;
static volatile uint32_t diag_missed_deadline_last_slot = UINT32_MAX;
static volatile uint32_t diag_missed_deadline_last_handle = 0;
static volatile const char* diag_missed_deadline_last_name = nullptr;
static volatile const char* diag_missed_deadline_last_source = nullptr;
static volatile uint32_t diag_missed_deadline_last_deadline = 0;
static volatile uint32_t diag_missed_deadline_last_now = 0;
static volatile uint32_t diag_missed_deadline_last_late_ticks = 0;
static volatile uint32_t diag_missed_deadline_last_dwt = 0;

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
static inline bool deadline_exact(uint32_t deadline, uint32_t now);
static inline bool deadline_passed(uint32_t deadline, uint32_t now);
static inline bool deadline_reached_or_passed(uint32_t deadline, uint32_t now);
static inline uint32_t deadline_lateness_ticks(uint32_t deadline, uint32_t now);
static inline uint32_t ns_to_ticks(uint64_t ns);
static bool period_ns_to_exact_ticks(uint64_t period_ns, uint32_t& out_ticks);

static inline void update_slot_high_water(void);
static inline void update_deferred_high_water(deferred_slot_t* slots_buf,
                                              uint32_t max_slots,
                                              volatile uint32_t& high_water);
static inline bool deferred_entry_occupied(const deferred_slot_t& slot);
static inline bool deferred_name_equals(const deferred_slot_t& slot,
                                        const char* name);
static inline bool deferred_any_pending(const deferred_slot_t* slots_buf,
                                        uint32_t max_slots);
static uint32_t deferred_pending_count(const deferred_slot_t* slots_buf,
                                       uint32_t max_slots);
static uint32_t deferred_dispatching_count(const deferred_slot_t* slots_buf,
                                           uint32_t max_slots);
static uint32_t deferred_occupied_count(const deferred_slot_t* slots_buf,
                                        uint32_t max_slots);
static uint32_t deferred_cancel_pending_by_name_unlocked(deferred_slot_t* slots_buf,
                                                         uint32_t max_slots,
                                                         const char* name,
                                                         volatile uint32_t& cancelled_count);
static bool deferred_cancel_pending_by_handle_unlocked(deferred_slot_t* slots_buf,
                                                       uint32_t max_slots,
                                                       timepop_handle_t handle,
                                                       volatile uint32_t& cancelled_count);
static inline void deferred_clear_if_idle(deferred_slot_t& slot);
static inline timepop_handle_t allocate_handle_unlocked(void);
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
static bool timepop_quarantine_missed_deadline(timepop_slot_t& slot,
                                               uint32_t now,
                                               uint32_t dwt,
                                               const char* source,
                                               bool too_close);

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
                                                            const char* name,
                                                            timepop_priority_t priority,
                                                            timepop_handle_t forced_handle = TIMEPOP_INVALID_HANDLE);
static timepop_handle_t arm_anchored_recurring_isr_from_counter32_internal(int64_t base_gnss_ns,
                                                                           uint32_t base_counter32,
                                                                           uint64_t period_gnss_ns,
                                                                           timepop_callback_t callback,
                                                                           void* user_data,
                                                                           const char* name,
                                                                           timepop_priority_t priority,
                                                                           timepop_handle_t forced_handle = TIMEPOP_INVALID_HANDLE);
static timepop_handle_t arm_absolute_slot_internal(int64_t target_gnss_ns,
                                                   bool recurring,
                                                   uint64_t recurring_period_gnss_ns,
                                                   timepop_callback_t callback,
                                                   void* user_data,
                                                   const char* name,
                                                   bool isr_callback,
                                                   uint32_t target_dwt,
                                                   timepop_priority_t priority,
                                                   timepop_handle_t forced_handle = TIMEPOP_INVALID_HANDLE);
static timepop_handle_t arm_relative_slot_internal(uint64_t delay_gnss_ns,
                                                   bool recurring,
                                                   timepop_callback_t callback,
                                                   void* user_data,
                                                   const char* name,
                                                   bool isr_callback,
                                                   bool rearm_in_isr,
                                                   timepop_priority_t priority,
                                                   timepop_handle_t forced_handle = TIMEPOP_INVALID_HANDLE);

static bool timepop_should_queue_dispatch_mutation(void);
static timepop_handle_t queue_dispatch_relative_arm(uint64_t delay_gnss_ns,
                                                    bool recurring,
                                                    timepop_callback_t callback,
                                                    void* user_data,
                                                    const char* name,
                                                    bool isr_callback,
                                                    bool rearm_in_isr,
                                                    timepop_priority_t priority);
static timepop_handle_t queue_dispatch_absolute_arm(int64_t target_gnss_ns,
                                                    bool recurring,
                                                    uint64_t recurring_period_gnss_ns,
                                                    timepop_callback_t callback,
                                                    void* user_data,
                                                    const char* name,
                                                    bool isr_callback,
                                                    uint32_t target_dwt,
                                                    timepop_priority_t priority);
static timepop_handle_t queue_dispatch_anchored_isr_arm(int64_t base_gnss_ns,
                                                        uint64_t period_gnss_ns,
                                                        timepop_callback_t callback,
                                                        void* user_data,
                                                        const char* name,
                                                        timepop_priority_t priority);
static timepop_handle_t queue_dispatch_anchored_isr_counter32_arm(int64_t base_gnss_ns,
                                                                  uint32_t base_counter32,
                                                                  uint64_t period_gnss_ns,
                                                                  timepop_callback_t callback,
                                                                  void* user_data,
                                                                  const char* name,
                                                                  timepop_priority_t priority);
static bool queue_dispatch_cancel_handle(timepop_handle_t handle);
static bool queue_dispatch_cancel_name(const char* name);
static void timepop_dispatch_enter(timepop_dispatch_phase_t phase);
static void timepop_dispatch_set_phase(timepop_dispatch_phase_t phase);
static void timepop_dispatch_leave(void);
static void timepop_apply_dispatch_mutations(const char* context);
static void timepop_idle_witness_spin_until_pending(void);
// ============================================================================
// Dispatch-time mutation barrier helpers
// ============================================================================

static bool timepop_should_queue_dispatch_mutation(void) {
  return dispatch_depth != 0 && !dispatch_applying_mutations;
}

static void timepop_dispatch_enter(timepop_dispatch_phase_t phase) {
  const uint32_t saved = critical_enter();
  dispatch_depth++;
  if (dispatch_depth > diag_dispatch_depth_max) {
    diag_dispatch_depth_max = dispatch_depth;
  }
  dispatch_phase = phase;
  critical_exit(saved);
}

static void timepop_dispatch_set_phase(timepop_dispatch_phase_t phase) {
  const uint32_t saved = critical_enter();
  dispatch_phase = phase;
  critical_exit(saved);
}

static void timepop_dispatch_leave(void) {
  const uint32_t saved = critical_enter();
  if (dispatch_depth != 0) dispatch_depth--;
  if (dispatch_depth == 0) dispatch_phase = timepop_dispatch_phase_t::IDLE;
  critical_exit(saved);
}


static bool dispatch_mutation_is_arm(timepop_dispatch_mutation_kind_t kind) {
  return kind == timepop_dispatch_mutation_kind_t::ARM_RELATIVE ||
         kind == timepop_dispatch_mutation_kind_t::ARM_ABSOLUTE ||
         kind == timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR ||
         kind == timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR_COUNTER32;
}

static bool dispatch_mutation_copy_name(timepop_dispatch_mutation_t& m,
                                        const char* name) {
  m.name[0] = '\0';
  if (!name || !*name) return true;

  size_t length = 0;
  while (length <= MAX_DISPATCH_NAME && name[length] != '\0') {
    length++;
  }

  if (length > MAX_DISPATCH_NAME) {
    diag_dispatch_mutation_name_too_long++;
    return false;
  }

  memcpy(m.name, name, length + 1U);
  diag_dispatch_mutation_name_copies++;
  return true;
}

static inline const char* dispatch_mutation_name_or_null(
    const timepop_dispatch_mutation_t& m) {
  return m.name[0] ? m.name : nullptr;
}

static bool dispatch_mutation_name_equals(const timepop_dispatch_mutation_t& m,
                                          const char* name) {
  if (!name || !*name || !m.name[0]) return false;
  return strcmp(m.name, name) == 0;
}

static void dispatch_mutation_remove_at_unlocked(uint32_t index) {
  if (index >= dispatch_mutation_count) return;
  for (uint32_t i = index + 1U; i < dispatch_mutation_count; i++) {
    dispatch_mutations[i - 1U] = dispatch_mutations[i];
  }
  dispatch_mutation_count--;
  dispatch_mutations[dispatch_mutation_count] = timepop_dispatch_mutation_t{};
  diag_dispatch_mutation_last_queue_depth = dispatch_mutation_count;
  diag_dispatch_mutation_coalesced++;
}

static void dispatch_mutation_coalesce_for_arm_unlocked(const char* name) {
  if (!name || !*name) return;

  // A queued ARM of a named timed slot performs the normal singleton
  // replacement when applied, including retirement of an existing same-named
  // timed slot and any pending same-named deferred request.  Therefore a
  // queued CANCEL_NAME for the same name immediately before the ARM is
  // redundant and only burns queue capacity.
  for (int32_t i = (int32_t)dispatch_mutation_count - 1; i >= 0; i--) {
    if (dispatch_mutations[i].kind == timepop_dispatch_mutation_kind_t::CANCEL_NAME &&
        dispatch_mutation_name_equals(dispatch_mutations[i], name)) {
      dispatch_mutation_remove_at_unlocked((uint32_t)i);
    }
  }
}

static void dispatch_mutation_coalesce_for_cancel_name_unlocked(const char* name) {
  if (!name || !*name) return;

  // CANCEL_NAME is idempotent.  If a same-named ARM is still queued but not yet
  // installed, remove that ARM instead of queuing a future cancel for it.  This
  // preserves the user's postcondition while preventing bursty cancel+arm
  // traffic from exhausting the fixed mutation queue.  Duplicate same-name
  // cancels also collapse to a single queued cancel.
  for (int32_t i = (int32_t)dispatch_mutation_count - 1; i >= 0; i--) {
    if (!dispatch_mutation_name_equals(dispatch_mutations[i], name)) continue;

    if (dispatch_mutation_is_arm(dispatch_mutations[i].kind) ||
        dispatch_mutations[i].kind == timepop_dispatch_mutation_kind_t::CANCEL_NAME) {
      dispatch_mutation_remove_at_unlocked((uint32_t)i);
    }
  }
}

static bool dispatch_mutation_coalesce_for_cancel_handle_unlocked(timepop_handle_t handle) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;

  for (int32_t i = (int32_t)dispatch_mutation_count - 1; i >= 0; i--) {
    if (!dispatch_mutation_is_arm(dispatch_mutations[i].kind)) continue;
    if (dispatch_mutations[i].reserved_handle != handle) continue;

    dispatch_mutation_remove_at_unlocked((uint32_t)i);
    diag_dispatch_mutation_cancel_applied++;
    return true;
  }

  return false;
}

static timepop_handle_t queue_dispatch_arm_mutation(timepop_dispatch_mutation_t m) {
  if (!m.callback) return TIMEPOP_INVALID_HANDLE;

  const uint32_t saved = critical_enter();
  if (dispatch_mutation_count >= MAX_DISPATCH_MUTATIONS) {
    diag_dispatch_mutation_overflow++;
    diag_arm_failures++;
    critical_exit(saved);
    return TIMEPOP_INVALID_HANDLE;
  }

  dispatch_mutation_coalesce_for_arm_unlocked(m.name);

  const timepop_handle_t h = allocate_handle_unlocked();
  m.reserved_handle = h;
  m.queued_phase = dispatch_phase;
  dispatch_mutations[dispatch_mutation_count++] = m;
  if (dispatch_mutation_count > diag_dispatch_mutation_high_water) {
    diag_dispatch_mutation_high_water = dispatch_mutation_count;
  }
  diag_dispatch_mutation_queued++;
  diag_dispatch_timed_arm_queued++;
  diag_dispatch_mutation_last_kind = dispatch_mutation_kind_str(m.kind);
  diag_dispatch_mutation_last_phase = dispatch_phase_str(m.queued_phase);
  diag_dispatch_mutation_last_handle = h;
  diag_dispatch_mutation_last_queue_depth = dispatch_mutation_count;
  critical_exit(saved);
  return h;
}

static bool queue_dispatch_non_arm_mutation(timepop_dispatch_mutation_t m) {
  const uint32_t saved = critical_enter();

  if (m.kind == timepop_dispatch_mutation_kind_t::CANCEL_NAME) {
    dispatch_mutation_coalesce_for_cancel_name_unlocked(m.name);
  } else if (m.kind == timepop_dispatch_mutation_kind_t::CANCEL_HANDLE) {
    if (dispatch_mutation_coalesce_for_cancel_handle_unlocked(m.cancel_handle)) {
      diag_dispatch_mutation_queued++;
      diag_dispatch_cancel_queued++;
      diag_dispatch_mutation_last_kind = dispatch_mutation_kind_str(m.kind);
      diag_dispatch_mutation_last_phase = dispatch_phase_str(dispatch_phase);
      diag_dispatch_mutation_last_handle = m.cancel_handle;
      diag_dispatch_mutation_last_queue_depth = dispatch_mutation_count;
      critical_exit(saved);
      return true;
    }
  }

  if (dispatch_mutation_count >= MAX_DISPATCH_MUTATIONS) {
    diag_dispatch_mutation_overflow++;
    critical_exit(saved);
    return false;
  }

  m.queued_phase = dispatch_phase;
  dispatch_mutations[dispatch_mutation_count++] = m;
  if (dispatch_mutation_count > diag_dispatch_mutation_high_water) {
    diag_dispatch_mutation_high_water = dispatch_mutation_count;
  }
  diag_dispatch_mutation_queued++;
  diag_dispatch_cancel_queued++;
  diag_dispatch_mutation_last_kind = dispatch_mutation_kind_str(m.kind);
  diag_dispatch_mutation_last_phase = dispatch_phase_str(m.queued_phase);
  diag_dispatch_mutation_last_handle =
      (m.kind == timepop_dispatch_mutation_kind_t::CANCEL_HANDLE)
          ? m.cancel_handle
          : 0;
  diag_dispatch_mutation_last_queue_depth = dispatch_mutation_count;
  critical_exit(saved);
  return true;
}

static timepop_handle_t queue_dispatch_relative_arm(uint64_t delay_gnss_ns,
                                                    bool recurring,
                                                    timepop_callback_t callback,
                                                    void* user_data,
                                                    const char* name,
                                                    bool isr_callback,
                                                    bool rearm_in_isr,
                                                    timepop_priority_t priority) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (rearm_in_isr && (!recurring || !isr_callback)) return TIMEPOP_INVALID_HANDLE;

  timepop_dispatch_mutation_t m{};
  m.kind = timepop_dispatch_mutation_kind_t::ARM_RELATIVE;
  m.ns0 = delay_gnss_ns;
  m.recurring = recurring;
  m.isr_callback = isr_callback;
  m.rearm_in_isr = rearm_in_isr;
  m.callback = callback;
  m.user_data = user_data;
  if (!dispatch_mutation_copy_name(m, name)) return TIMEPOP_INVALID_HANDLE;
  m.priority = priority;
  return queue_dispatch_arm_mutation(m);
}

static timepop_handle_t queue_dispatch_absolute_arm(int64_t target_gnss_ns,
                                                    bool recurring,
                                                    uint64_t recurring_period_gnss_ns,
                                                    timepop_callback_t callback,
                                                    void* user_data,
                                                    const char* name,
                                                    bool isr_callback,
                                                    uint32_t target_dwt,
                                                    timepop_priority_t priority) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  timepop_dispatch_mutation_t m{};
  m.kind = timepop_dispatch_mutation_kind_t::ARM_ABSOLUTE;
  m.ns0 = recurring_period_gnss_ns;
  m.gnss0 = target_gnss_ns;
  m.u32_0 = target_dwt;
  m.recurring = recurring;
  m.isr_callback = isr_callback;
  m.callback = callback;
  m.user_data = user_data;
  if (!dispatch_mutation_copy_name(m, name)) return TIMEPOP_INVALID_HANDLE;
  m.priority = priority;
  return queue_dispatch_arm_mutation(m);
}

static timepop_handle_t queue_dispatch_anchored_isr_arm(int64_t base_gnss_ns,
                                                        uint64_t period_gnss_ns,
                                                        timepop_callback_t callback,
                                                        void* user_data,
                                                        const char* name,
                                                        timepop_priority_t priority) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (base_gnss_ns <= 0 || period_gnss_ns == 0) return TIMEPOP_INVALID_HANDLE;

  timepop_dispatch_mutation_t m{};
  m.kind = timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR;
  m.ns0 = period_gnss_ns;
  m.gnss0 = base_gnss_ns;
  m.callback = callback;
  m.user_data = user_data;
  if (!dispatch_mutation_copy_name(m, name)) return TIMEPOP_INVALID_HANDLE;
  m.priority = priority;
  return queue_dispatch_arm_mutation(m);
}

static timepop_handle_t queue_dispatch_anchored_isr_counter32_arm(int64_t base_gnss_ns,
                                                                  uint32_t base_counter32,
                                                                  uint64_t period_gnss_ns,
                                                                  timepop_callback_t callback,
                                                                  void* user_data,
                                                                  const char* name,
                                                                  timepop_priority_t priority) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (base_gnss_ns <= 0 || period_gnss_ns == 0) return TIMEPOP_INVALID_HANDLE;

  timepop_dispatch_mutation_t m{};
  m.kind = timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR_COUNTER32;
  m.ns0 = period_gnss_ns;
  m.gnss0 = base_gnss_ns;
  m.u32_0 = base_counter32;
  m.callback = callback;
  m.user_data = user_data;
  if (!dispatch_mutation_copy_name(m, name)) return TIMEPOP_INVALID_HANDLE;
  m.priority = priority;
  return queue_dispatch_arm_mutation(m);
}

static bool queue_dispatch_cancel_handle(timepop_handle_t handle) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;
  timepop_dispatch_mutation_t m{};
  m.kind = timepop_dispatch_mutation_kind_t::CANCEL_HANDLE;
  m.cancel_handle = handle;
  return queue_dispatch_non_arm_mutation(m);
}

static bool queue_dispatch_cancel_name(const char* name) {
  if (!name || !*name) return false;
  timepop_dispatch_mutation_t m{};
  m.kind = timepop_dispatch_mutation_kind_t::CANCEL_NAME;
  if (!dispatch_mutation_copy_name(m, name)) return false;
  return queue_dispatch_non_arm_mutation(m);
}

static inline void timepop_idle_witness_note_wall_cycles(uint32_t now_dwt) {
  if (!diag_idle_witness_wall_initialized) {
    diag_idle_witness_wall_last_dwt = now_dwt;
    diag_idle_witness_wall_initialized = true;
    return;
  }

  diag_idle_witness_wall_cycles +=
      (uint64_t)(uint32_t)(now_dwt - diag_idle_witness_wall_last_dwt);
  diag_idle_witness_wall_last_dwt = now_dwt;
}

static void timepop_idle_witness_spin_until_pending(void) {
  if (!TIMEPOP_IDLE_DWT_WITNESS_ENABLED) return;

  const uint32_t enter_dwt = ARM_DWT_CYCCNT;
  timepop_idle_witness_note_wall_cycles(enter_dwt);
  diag_idle_witness_enter_count++;
  diag_idle_witness_last_enter_dwt = enter_dwt;
  diag_idle_witness_running = true;

  for (;;) {
    // Read first, then test the dispatch-pending bit, and only then publish
    // the shadow.  If an interrupt sets timepop_pending between the read and
    // the test, the value we publish is still pre-interrupt evidence.  If the
    // interrupt already happened before the read, pending is true and we do
    // not overwrite the last pre-interrupt shadow with a post-interrupt value.
    const uint32_t dwt = ARM_DWT_CYCCNT;
    if (timepop_pending) {
      const uint32_t residency = dwt - enter_dwt;
      timepop_idle_witness_note_wall_cycles(dwt);
      diag_idle_witness_pending_exit_count++;
      diag_idle_witness_last_exit_dwt = dwt;
      diag_idle_witness_last_residency_cycles = residency;
      diag_idle_witness_total_cycles += (uint64_t)residency;
      break;
    }
    g_timepop_idle_witness_shadow_dwt = dwt;
  }

  diag_idle_witness_running = false;
  diag_idle_witness_exit_count++;
}

static bool pop_dispatch_mutation(timepop_dispatch_mutation_t& out) {
  const uint32_t saved = critical_enter();
  if (dispatch_mutation_count == 0) {
    critical_exit(saved);
    return false;
  }

  out = dispatch_mutations[0];
  for (uint32_t i = 1; i < dispatch_mutation_count; i++) {
    dispatch_mutations[i - 1] = dispatch_mutations[i];
  }
  dispatch_mutation_count--;
  dispatch_mutations[dispatch_mutation_count] = timepop_dispatch_mutation_t{};
  diag_dispatch_mutation_last_queue_depth = dispatch_mutation_count;
  critical_exit(saved);
  return true;
}

static void timepop_apply_dispatch_mutations(const char* context) {
  if (dispatch_applying_mutations) return;

  const uint32_t saved = critical_enter();
  if (dispatch_mutation_count == 0) {
    critical_exit(saved);
    return;
  }
  dispatch_applying_mutations = true;
  const timepop_dispatch_phase_t prior_phase = dispatch_phase;
  dispatch_phase = timepop_dispatch_phase_t::APPLYING_MUTATIONS;
  critical_exit(saved);

  diag_dispatch_mutation_apply_passes++;
  diag_dispatch_mutation_last_context = context;
  const uint32_t schedule_before = diag_schedule_next_calls_total;

  timepop_dispatch_mutation_t m{};
  while (pop_dispatch_mutation(m)) {
    bool ok = false;
    timepop_handle_t h = TIMEPOP_INVALID_HANDLE;

    diag_dispatch_mutation_last_kind = dispatch_mutation_kind_str(m.kind);
    diag_dispatch_mutation_last_phase = dispatch_phase_str(m.queued_phase);
    diag_dispatch_mutation_last_handle =
        (m.kind == timepop_dispatch_mutation_kind_t::CANCEL_HANDLE)
            ? m.cancel_handle
            : m.reserved_handle;

    switch (m.kind) {
      case timepop_dispatch_mutation_kind_t::ARM_RELATIVE:
        h = arm_relative_slot_internal(m.ns0,
                                       m.recurring,
                                       m.callback,
                                       m.user_data,
                                       dispatch_mutation_name_or_null(m),
                                       m.isr_callback,
                                       m.rearm_in_isr,
                                       m.priority,
                                       m.reserved_handle);
        ok = (h == m.reserved_handle && h != TIMEPOP_INVALID_HANDLE);
        break;

      case timepop_dispatch_mutation_kind_t::ARM_ABSOLUTE:
        h = arm_absolute_slot_internal(m.gnss0,
                                       m.recurring,
                                       m.ns0,
                                       m.callback,
                                       m.user_data,
                                       dispatch_mutation_name_or_null(m),
                                       m.isr_callback,
                                       m.u32_0,
                                       m.priority,
                                       m.reserved_handle);
        ok = (h == m.reserved_handle && h != TIMEPOP_INVALID_HANDLE);
        break;

      case timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR:
        h = arm_anchored_recurring_isr_internal(m.gnss0,
                                                m.ns0,
                                                m.callback,
                                                m.user_data,
                                                dispatch_mutation_name_or_null(m),
                                                m.priority,
                                                m.reserved_handle);
        ok = (h == m.reserved_handle && h != TIMEPOP_INVALID_HANDLE);
        break;

      case timepop_dispatch_mutation_kind_t::ARM_ANCHORED_ISR_COUNTER32:
        h = arm_anchored_recurring_isr_from_counter32_internal(m.gnss0,
                                                               m.u32_0,
                                                               m.ns0,
                                                               m.callback,
                                                               m.user_data,
                                                               dispatch_mutation_name_or_null(m),
                                                               m.priority,
                                                               m.reserved_handle);
        ok = (h == m.reserved_handle && h != TIMEPOP_INVALID_HANDLE);
        break;

      case timepop_dispatch_mutation_kind_t::CANCEL_HANDLE: {
        const bool cancelled = timepop_cancel(m.cancel_handle);
        ok = true;  // cancel is idempotent at the dispatch barrier
        if (cancelled) diag_dispatch_mutation_cancel_applied++;
        else diag_dispatch_mutation_cancel_noop++;
        break;
      }

      case timepop_dispatch_mutation_kind_t::CANCEL_NAME: {
        const uint32_t cancelled = timepop_cancel_by_name(m.name);
        ok = true;  // cancel-by-name postcondition is satisfied even if empty
        if (cancelled != 0) diag_dispatch_mutation_cancel_applied++;
        else diag_dispatch_mutation_cancel_noop++;
        break;
      }

      default:
        ok = true;
        break;
    }

    if (ok) {
      diag_dispatch_mutation_applied++;
    } else {
      diag_dispatch_mutation_apply_failures++;
      if (dispatch_mutation_is_arm(m.kind)) {
        diag_dispatch_mutation_arm_failures++;
      } else if (m.kind == timepop_dispatch_mutation_kind_t::CANCEL_HANDLE ||
                 m.kind == timepop_dispatch_mutation_kind_t::CANCEL_NAME) {
        diag_dispatch_mutation_cancel_failures++;
      }
    }
  }

  const uint32_t schedule_after = diag_schedule_next_calls_total;
  const uint32_t schedule_delta = schedule_after - schedule_before;
  if (schedule_delta != 0) {
    diag_dispatch_mutation_schedule_next_calls += schedule_delta;
    diag_schedule_next_calls_from_dispatch += schedule_delta;
    if (diag_schedule_next_calls_from_other >= schedule_delta) {
      diag_schedule_next_calls_from_other -= schedule_delta;
    } else {
      diag_schedule_next_calls_from_other = 0;
    }
  }

  const uint32_t saved2 = critical_enter();
  dispatch_phase = prior_phase;
  dispatch_applying_mutations = false;
  critical_exit(saved2);
}

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


static inline bool deferred_entry_occupied(const deferred_slot_t& slot) {
  return slot.pending || slot.dispatching;
}

static inline bool deferred_name_equals(const deferred_slot_t& slot,
                                        const char* name) {
  if (!name || !*name) return false;
  if (slot.pending && slot.name && strcmp(slot.name, name) == 0) return true;
  if (slot.dispatching && slot.dispatch_name &&
      strcmp(slot.dispatch_name, name) == 0) return true;
  return false;
}

static inline void deferred_clear_if_idle(deferred_slot_t& slot) {
  if (slot.pending || slot.dispatching) return;
  slot.handle = TIMEPOP_INVALID_HANDLE;
  slot.callback = nullptr;
  slot.user_data = nullptr;
  slot.name = nullptr;
  slot.dispatch_handle = TIMEPOP_INVALID_HANDLE;
  slot.dispatch_name = nullptr;
}

static inline timepop_handle_t allocate_handle_unlocked(void) {
  timepop_handle_t h = next_handle++;
  if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;
  return h;
}

static uint32_t deferred_pending_count(const deferred_slot_t* slots_buf,
                                       uint32_t max_slots) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < max_slots; i++) {
    if (slots_buf[i].pending) n++;
  }
  return n;
}

static uint32_t deferred_dispatching_count(const deferred_slot_t* slots_buf,
                                           uint32_t max_slots) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < max_slots; i++) {
    if (slots_buf[i].dispatching) n++;
  }
  return n;
}

static uint32_t deferred_occupied_count(const deferred_slot_t* slots_buf,
                                        uint32_t max_slots) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < max_slots; i++) {
    if (deferred_entry_occupied(slots_buf[i])) n++;
  }
  return n;
}

static inline bool deferred_any_pending(const deferred_slot_t* slots_buf,
                                        uint32_t max_slots) {
  for (uint32_t i = 0; i < max_slots; i++) {
    if (slots_buf[i].pending) return true;
  }
  return false;
}

static uint32_t deferred_cancel_pending_by_name_unlocked(deferred_slot_t* slots_buf,
                                                         uint32_t max_slots,
                                                         const char* name,
                                                         volatile uint32_t& cancelled_count) {
  if (!name || !*name) return 0;

  uint32_t cancelled = 0;
  for (uint32_t i = 0; i < max_slots; i++) {
    if (!slots_buf[i].pending || !slots_buf[i].name) continue;
    if (strcmp(slots_buf[i].name, name) != 0) continue;

    slots_buf[i].pending = false;
    cancelled++;
    cancelled_count++;
    if (!slots_buf[i].dispatching) {
      deferred_clear_if_idle(slots_buf[i]);
    }
  }
  return cancelled;
}

static bool deferred_cancel_pending_by_handle_unlocked(deferred_slot_t* slots_buf,
                                                       uint32_t max_slots,
                                                       timepop_handle_t handle,
                                                       volatile uint32_t& cancelled_count) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;

  for (uint32_t i = 0; i < max_slots; i++) {
    if (!slots_buf[i].pending || slots_buf[i].handle != handle) continue;

    slots_buf[i].pending = false;
    cancelled_count++;
    if (!slots_buf[i].dispatching) {
      deferred_clear_if_idle(slots_buf[i]);
    }
    return true;
  }
  return false;
}

static inline void update_deferred_high_water(deferred_slot_t* slots_buf,
                                              uint32_t max_slots,
                                              volatile uint32_t& high_water) {
  const uint32_t active = deferred_occupied_count(slots_buf, max_slots);
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
  while (deadline_reached_or_passed(deadline, fire_vclock_raw)) {
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

static inline bool deadline_exact(uint32_t deadline, uint32_t now) {
  return deadline == now;
}

static inline bool deadline_passed(uint32_t deadline, uint32_t now) {
  return deadline != now && ((deadline - now) > MAX_DELAY_TICKS);
}

static inline bool deadline_reached_or_passed(uint32_t deadline, uint32_t now) {
  return deadline_exact(deadline, now) || deadline_passed(deadline, now);
}

static inline uint32_t deadline_lateness_ticks(uint32_t deadline, uint32_t now) {
  return deadline_exact(deadline, now) ? 0U : (now - deadline);
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

static inline void slot_set_name(timepop_slot_t& slot,
                                 const char* name,
                                 bool copy_queued_name) {
  slot.name = name;
  if (!copy_queued_name || !name) return;

  // Queued names arrive from timepop_dispatch_mutation_t::name, which is
  // exactly this size and was NUL-terminated before queue insertion.
  memcpy(slot.queued_name, name, sizeof(slot.queued_name));
  slot.queued_name[MAX_DISPATCH_NAME] = '\0';
  slot.name = slot.queued_name;
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
// already-authored CH2 fire facts from the current scheduler pass and falls back to a
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

static bool timepop_quarantine_missed_deadline(timepop_slot_t& slot,
                                               uint32_t now,
                                               uint32_t dwt,
                                               const char* source,
                                               bool too_close) {
  const uint32_t idx = slot_index_for(slot);
  const uint32_t old_deadline = slot.deadline;
  const uint32_t late_ticks = deadline_passed(old_deadline, now)
      ? deadline_lateness_ticks(old_deadline, now)
      : 0U;

  // A too-close deadline is still in the future, but not far enough into the
  // future to arm safely.  Re-author recurring grids past the scheduler race
  // window, not merely past the ambient observation.
  const uint32_t rearm_now = too_close ? (now + SCHEDULE_MIN_ARM_LEAD_TICKS) : now;

  diag_missed_deadline_slots++;
  if (too_close) diag_missed_deadline_too_close_count++;
  update_max_u32(diag_missed_deadline_late_max_ticks, late_ticks);
  diag_missed_deadline_last_slot = idx;
  diag_missed_deadline_last_handle = slot.handle;
  diag_missed_deadline_last_name = slot.name;
  diag_missed_deadline_last_source = source;
  diag_missed_deadline_last_deadline = old_deadline;
  diag_missed_deadline_last_now = now;
  diag_missed_deadline_last_late_ticks = late_ticks;
  diag_missed_deadline_last_dwt = dwt;

  if (!slot.first_expire_recorded) {
    slot.first_expire_now = now;
    slot.first_expire_recorded = true;
  }

  slot.schedule_next_expired_count++;
  slot.schedule_next_last_late_ticks = late_ticks;
  slot.schedule_next_last_now = now;
  slot.schedule_next_last_deadline = old_deadline;
  slot.schedule_next_last_dwt = dwt;
  if (late_ticks > slot.schedule_next_late_max_ticks) {
    slot.schedule_next_late_max_ticks = late_ticks;
  }

  slot.expired = false;
  slot.isr_callback_fired = false;
  slot.fire_vclock_raw = 0;
  slot.fire_dwt_cyccnt = 0;
  slot.fire_gnss_ns = -1;
  slot.fire_capture_source = timepop_fire_capture_source_t::NONE;

  if (!slot.recurring) {
    slot = {};
    diag_missed_deadline_one_shot_cancelled++;
    return true;
  }

  bool rearmed = false;

  if (slot.recurring_base_counter32_fixed) {
    rearmed = rearm_counter32_anchored_recurring_slot_from_event(
        slot, rearm_now, timepop_arm_source_t::MISSED_DEADLINE_REARM);
  }

  if (!rearmed &&
      slot.recurrence_mode == timepop_recurrence_mode_t::ABSOLUTE &&
      slot.recurring_period_gnss_ns > 0 &&
      slot.recurring_base_fixed &&
      slot.recurring_base_gnss_ns > 0) {
    const time_anchor_snapshot_t snap =
        timepop_anchor_snapshot("missed_deadline_rearm");
    const int64_t now_gnss_ns = vclock_to_gnss_ns(rearm_now, snap);
    int64_t next_target_gnss_ns = -1;
    uint32_t skipped = 0;
    if (anchored_next_target_gnss_ns(slot.recurring_base_gnss_ns,
                                     slot.recurring_period_gnss_ns,
                                     now_gnss_ns,
                                     next_target_gnss_ns,
                                     &skipped)) {
      uint32_t next_deadline = 0;
      if (gnss_ns_to_vclock_deadline(next_target_gnss_ns, snap, next_deadline)) {
        slot.target_gnss_ns = next_target_gnss_ns;
        slot.deadline = next_deadline;
        slot.is_absolute = true;
        slot.recurrence_mode = timepop_recurrence_mode_t::ABSOLUTE;
        slot.recurring_last_skipped_intervals = skipped;
        slot.recurring_total_skipped_intervals += skipped;
        slot.predicted_dwt = predict_dwt_at_deadline(slot.deadline,
                                                     slot.prediction_valid);
        slot.recurring_rearmed_count++;
        record_slot_arm_diag(slot,
                             timepop_arm_source_t::MISSED_DEADLINE_REARM,
                             true,
                             rearm_now,
                             slot.target_gnss_ns);
        rearmed = true;
      }
    }
  }

  if (!rearmed) {
    const uint64_t period_ns = slot.recurring_period_gnss_ns
        ? slot.recurring_period_gnss_ns
        : slot.period_ns;
    uint32_t period_ticks = 0;
    if (period_ns_to_exact_ticks(period_ns, period_ticks)) {
      slot.period_ns = period_ns;
      slot.period_ticks = period_ticks;
      slot.deadline = rearm_now + period_ticks;
      slot.arm_vclock_raw = rearm_now;
      slot.arm_delta_ticks = period_ticks;
      slot.is_absolute = false;
      slot.recurrence_mode = timepop_recurrence_mode_t::RELATIVE;
      slot.target_gnss_ns = -1;
      slot.predicted_dwt = predict_dwt_at_deadline(slot.deadline,
                                                   slot.prediction_valid);
      slot.recurring_rearmed_count++;
      record_slot_arm_diag(slot,
                           timepop_arm_source_t::MISSED_DEADLINE_REARM_FALLBACK,
                           true,
                           rearm_now,
                           slot.target_gnss_ns);
      rearmed = true;
    }
  }

  if (rearmed) {
    diag_missed_deadline_recurring_rearmed++;
    return true;
  }

  slot = {};
  diag_missed_deadline_recurring_rearm_failures++;
  return false;
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

  const uint32_t asap_cancelled =
      deferred_cancel_pending_by_name_unlocked(asap_slots,
                                               MAX_ASAP_SLOTS,
                                               name,
                                               diag_asap_cancelled);
  const uint32_t alap_cancelled =
      deferred_cancel_pending_by_name_unlocked(alap_slots,
                                               MAX_ALAP_SLOTS,
                                               name,
                                               diag_alap_cancelled);
  if (asap_cancelled || alap_cancelled) {
    retired_any = true;
    for (uint32_t i = 0; i < asap_cancelled + alap_cancelled; i++) {
      note_named_replacement(name);
    }
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
// deadline, it quarantines that slot as missed.  It never authors a
// precision fire fact from foreground/scheduler context.

static void schedule_next(void) {
  const uint32_t schedule_body_start_dwt = ARM_DWT_CYCCNT;
  diag_schedule_next_calls_total++;

  const uint32_t now = vclock_count();
  const uint32_t scheduler_dwt = ARM_DWT_CYCCNT;
  const bool scheduler_time_valid = time_valid();

  uint32_t soonest = 0;
  bool found = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;

    // During SmartZero / epoch reacquisition, the public TIME anchor is
    // intentionally invalid.  Old absolute deadlines are old-coordinate facts;
    // they must not survive as precision timers.  Keep relative recurring
    // services alive, and re-author absolute recurring services into a safe
    // relative future without firing their callbacks.
    if (!scheduler_time_valid &&
        slots[i].is_absolute &&
        !slots[i].recurring_base_counter32_fixed) {
      diag_schedule_next_invalid_time_count++;
      diag_schedule_next_last_pressure_source = "schedule_next_invalid_time";
      diag_schedule_next_last_pressure_name = slots[i].name;
      diag_schedule_next_last_pressure_deadline = slots[i].deadline;
      diag_schedule_next_last_pressure_now = now;
      diag_schedule_next_last_pressure_distance_ticks = slots[i].deadline - now;
      diag_schedule_next_last_pressure_late_ticks = 0;

      timepop_quarantine_missed_deadline(slots[i],
                                         now,
                                         scheduler_dwt,
                                         "schedule_next_invalid_time",
                                         false);
      if (!slots[i].active || slots[i].expired) continue;
    }

    const bool exact = deadline_exact(slots[i].deadline, now);
    const bool passed = deadline_passed(slots[i].deadline, now);
    const uint32_t distance = slots[i].deadline - now;
    const bool too_close = !exact && !passed &&
        distance < SCHEDULE_MIN_ARM_LEAD_TICKS;

    if (exact || passed || too_close) {
      // schedule_next() is not an event source.  It cannot supply a lawful DWT
      // edge coordinate for a deadline that has already been reached, nor can
      // it safely program a compare that is already at/inside the hardware race
      // window.  Quarantine the missed slot: recurring slots are re-authored
      // onto their next lawful deadline, one-shots are cancelled.
      const char* pressure_source = too_close
          ? "schedule_next_too_close"
          : (exact ? "schedule_next_exact_no_irq" : "schedule_next_passed");
      const uint32_t pressure_distance = slots[i].deadline - now;
      const uint32_t pressure_late =
          passed ? deadline_lateness_ticks(slots[i].deadline, now) : 0U;

      diag_schedule_next_expired_passes++;
      if (too_close) {
        diag_schedule_next_too_close_count++;
        if (pressure_distance > diag_schedule_next_too_close_max_ticks) {
          diag_schedule_next_too_close_max_ticks = pressure_distance;
        }
      } else if (exact) {
        diag_schedule_next_exact_no_irq_count++;
      } else {
        diag_schedule_next_passed_count++;
        if (pressure_late > diag_schedule_next_passed_late_max_ticks) {
          diag_schedule_next_passed_late_max_ticks = pressure_late;
        }
      }

      diag_schedule_next_last_pressure_source = pressure_source;
      diag_schedule_next_last_pressure_name = slots[i].name;
      diag_schedule_next_last_pressure_deadline = slots[i].deadline;
      diag_schedule_next_last_pressure_now = now;
      diag_schedule_next_last_pressure_distance_ticks = pressure_distance;
      diag_schedule_next_last_pressure_late_ticks = pressure_late;

      if (passed) {
        diag_schedule_next_expired_slots++;
        const uint32_t late_ticks = pressure_late;
        update_max_u32(diag_schedule_next_late_max_ticks, late_ticks);
        diag_schedule_next_last_expired_slot = i;
        diag_schedule_next_last_expired_handle = slots[i].handle;
        diag_schedule_next_last_expired_name = slots[i].name;
        diag_schedule_next_last_expired_deadline = slots[i].deadline;
        diag_schedule_next_last_expired_now = now;
        diag_schedule_next_last_expired_late_ticks = late_ticks;
        diag_schedule_next_last_expired_dwt = scheduler_dwt;
      }

      timepop_quarantine_missed_deadline(slots[i],
                                         now,
                                         scheduler_dwt,
                                         pressure_source,
                                         too_close);
      if (!slots[i].active || slots[i].expired) continue;
    }

    const uint32_t post_distance = slots[i].deadline - now;
    if (deadline_reached_or_passed(slots[i].deadline, now) ||
        post_distance < SCHEDULE_MIN_ARM_LEAD_TICKS) {
      continue;
    }

    if (!found || (post_distance < (soonest - now))) {
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

  // Final safety net: never program CH2 at or inside the scheduler race window.
  // If the chosen target became too close while we were scanning, use a
  // heartbeat compare instead of risking an immediate/missed-compare storm.
  if ((target - now) < SCHEDULE_MIN_ARM_LEAD_TICKS) {
    target = now + HEARTBEAT_TICKS;
    diag_heartbeat_rearms++;
    diag_schedule_next_final_safety_count++;
  }

  const uint32_t final_margin = target - now;
  diag_schedule_next_arm_margin_ticks_last = final_margin;
  if (final_margin < diag_schedule_next_arm_margin_ticks_min) {
    diag_schedule_next_arm_margin_ticks_min = final_margin;
  }
  if (final_margin > diag_schedule_next_arm_margin_ticks_max) {
    diag_schedule_next_arm_margin_ticks_max = final_margin;
  }

  ch2_arm_compare(target);
  diag_rearm_count++;

  const uint32_t schedule_body_cycles =
      (uint32_t)(ARM_DWT_CYCCNT - schedule_body_start_dwt);
  diag_schedule_next_body_cycles_last = schedule_body_cycles;
  if (schedule_body_cycles > diag_schedule_next_body_cycles_max) {
    diag_schedule_next_body_cycles_max = schedule_body_cycles;
  }
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

static inline bool slot_priority_before(uint32_t candidate, uint32_t incumbent) {
  if (incumbent >= MAX_SLOTS) return true;
  if (candidate >= MAX_SLOTS) return false;

  // Lower numeric priority runs first only for callbacks that share the same
  // captured physical CH2 fire fact. Equal priorities, or different captured
  // fire facts, retain the old stable slot-index order.
  if (slots[candidate].fire_vclock_raw == slots[incumbent].fire_vclock_raw &&
      slots[candidate].priority != slots[incumbent].priority) {
    return slots[candidate].priority < slots[incumbent].priority;
  }
  return candidate < incumbent;
}

static uint32_t select_next_irq_callback_slot_by_priority(
  const bool expired_this_pass[MAX_SLOTS],
  const bool already_dispatched[MAX_SLOTS]
) {
  uint32_t best = MAX_SLOTS;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!expired_this_pass[i] || already_dispatched[i]) continue;
    if (!slots[i].active || !slots[i].expired) continue;
    if (!slots[i].isr_callback) continue;
    if (slot_priority_before(i, best)) best = i;
  }
  return best;
}

static uint32_t select_next_scheduled_callback_slot_by_priority(
  const bool already_dispatched[MAX_SLOTS]
) {
  uint32_t best = MAX_SLOTS;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (already_dispatched[i]) continue;
    if (!slots[i].active || !slots[i].expired) continue;
    if (slot_priority_before(i, best)) best = i;
  }
  return best;
}

// ============================================================================
// QTimer1 CH2 handler — TimePop priority queue scheduler
// ============================================================================
//
// Scheduler-context handler called by process_interrupt after priority-0
// QTimer1 capture has been handed to process_interrupt's priority-16 custody
// tier.  Receives the standard event/diag payload — DWT and counter32 already
// filled in by the dispatcher. TimePop authors fire_gnss_ns from that
// counter32 identity using VCLOCK arithmetic; any GNSS value in the interrupt
// payload is diagnostic. The CH2 TCF1 flag is cleared by process_interrupt
// before this runs, so TimePop never touches QTimer hardware here.
//
static void timepop_process_ch2_event_in_scheduler_irq(const interrupt_event_t& event,
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
        audit_witness_reached_before = (slots[i].deadline == now);
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
        slots[i].deadline == now) {
      audit_onehz_due_count++;
    }
  }

  const bool audit_grid_this_irq =
      audit_onehz_due_count > 0 || audit_witness_reached_before;

  // Pre-pass: quarantine missed timed slots.  A slot whose deadline is already
  // behind this CH2 event did not fire at this edge.  Do not attach this DWT
  // coordinate to that old deadline; re-author recurring slots and cancel
  // one-shots instead.
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_passed(slots[i].deadline, now)) continue;

    const uint32_t late_ticks = deadline_lateness_ticks(slots[i].deadline, now);
    slots[i].irq_reached_count++;
    slots[i].irq_last_now = now;
    slots[i].irq_last_dwt = dwt_entry;
    slots[i].irq_last_late_ticks = late_ticks;
    slots[i].irq_last_exact = false;
    if (late_ticks > slots[i].irq_late_max_ticks) {
      slots[i].irq_late_max_ticks = late_ticks;
    }

    diag_irq_late_deadline_slots++;
    update_max_u32(diag_irq_late_max_ticks, late_ticks);
    timepop_quarantine_missed_deadline(slots[i],
                                       now,
                                       dwt_entry,
                                       "irq_late_deadline",
                                       false);
  }

  // Phase 1: author the shared fire facts for every slot that exactly matches
  // this physical CH2 compare event before any user callback is allowed to run.
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
    if (slots[i].deadline != now) continue;

    const uint32_t late_ticks = 0;
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
  // Priority affects callback order only after Phase 1 has captured the
  // shared physical CH2 fire fact for every exact-match slot.
  bool irq_callback_dispatched[MAX_SLOTS] = {};
  for (;;) {
    const uint32_t i = select_next_irq_callback_slot_by_priority(
        expired_this_pass, irq_callback_dispatched);
    if (i >= MAX_SLOTS) break;
    irq_callback_dispatched[i] = true;

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


void timepop_qtimer1_ch2_handler(const interrupt_event_t& event,
                                 const interrupt_capture_diag_t& diag) {
  // process_interrupt already transported the CH2 hardware event out of the
  // priority-0 capture tier.  TimePop receives immutable event facts in the
  // process_interrupt handoff context and performs scheduler policy directly.
  diag_ch2_direct_call_count++;
  diag_ch2_direct_last_event_counter32 = event.counter32_at_event;
  diag_ch2_direct_last_event_dwt = event.dwt_at_event;

  const uint32_t body_start = ARM_DWT_CYCCNT;
  timepop_process_ch2_event_in_scheduler_irq(event, diag);
  const uint32_t body_cycles = ARM_DWT_CYCCNT - body_start;
  diag_ch2_direct_body_cycles_last = body_cycles;
  if (body_cycles > diag_ch2_direct_body_cycles_max) {
    diag_ch2_direct_body_cycles_max = body_cycles;
  }
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
  for (uint32_t i = 0; i < max_slots; i++) {
    timepop_callback_t callback = nullptr;
    void* user_data = nullptr;
    timepop_handle_t handle = TIMEPOP_INVALID_HANDLE;
    const char* name = nullptr;

    {
      const uint32_t saved = critical_enter();
      if (slots_buf[i].pending) {
        callback = slots_buf[i].callback;
        user_data = slots_buf[i].user_data;
        handle = slots_buf[i].handle;
        name = slots_buf[i].name;

        slots_buf[i].pending = false;
        slots_buf[i].dispatching = true;
        slots_buf[i].dispatch_handle = handle;
        slots_buf[i].dispatch_name = name;
      }
      critical_exit(saved);
    }

    if (!callback || handle == TIMEPOP_INVALID_HANDLE) continue;

    timepop_ctx_t ctx;
    build_deferred_ctx(handle, ctx);

    const uint32_t start = ARM_DWT_CYCCNT;
    callback(&ctx, nullptr, user_data);
    const uint32_t end = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);
    diag_dispatch_callbacks++;
    dispatched_count++;
    last_dispatch_dwt = end;

    timepop_apply_dispatch_mutations("deferred_callback");

    {
      const uint32_t saved = critical_enter();
      if (slots_buf[i].dispatching &&
          slots_buf[i].dispatch_handle == handle) {
        slots_buf[i].dispatching = false;
        slots_buf[i].dispatch_handle = TIMEPOP_INVALID_HANDLE;
        slots_buf[i].dispatch_name = nullptr;
      }
      if (!slots_buf[i].pending && !slots_buf[i].dispatching) {
        deferred_clear_if_idle(slots_buf[i]);
      }
      critical_exit(saved);
    }
  }
}

static timepop_handle_t arm_deferred(deferred_slot_t* slots_buf,
                                     uint32_t max_slots,
                                     deferred_slot_t* other_slots_buf,
                                     uint32_t other_max_slots,
                                     volatile uint32_t& arm_failures,
                                     volatile uint32_t& armed_count,
                                     volatile uint32_t& last_armed_dwt,
                                     volatile const char*& last_armed_name,
                                     volatile uint32_t& high_water,
                                     volatile uint32_t& replacements,
                                     volatile uint32_t& other_cancelled_count,
                                     volatile uint32_t& arm_while_dispatching_count,
                                     timepop_callback_t callback,
                                     void* user_data,
                                     const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  const uint32_t saved = critical_enter();

  // Deferred callbacks are intentionally not timed slots.  Arming ASAP/ALAP
  // from ISR context must not mutate the timed slot table and must not call
  // schedule_next().  Named replacement is therefore restricted to the
  // dedicated deferred lanes.
  if (name && *name && other_slots_buf && other_max_slots > 0) {
    const uint32_t cancelled =
        deferred_cancel_pending_by_name_unlocked(other_slots_buf,
                                                 other_max_slots,
                                                 name,
                                                 other_cancelled_count);
    if (cancelled != 0) {
      for (uint32_t i = 0; i < cancelled; i++) note_named_replacement(name);
    }
  }

  if (name && *name) {
    for (uint32_t i = 0; i < max_slots; i++) {
      if (!deferred_name_equals(slots_buf[i], name)) continue;

      const timepop_handle_t h = allocate_handle_unlocked();
      slots_buf[i].pending = true;
      slots_buf[i].handle = h;
      slots_buf[i].callback = callback;
      slots_buf[i].user_data = user_data;
      slots_buf[i].name = name;
      slots_buf[i].generation++;

      if (slots_buf[i].dispatching) {
        arm_while_dispatching_count++;
      } else {
        replacements++;
      }
      note_named_replacement(name);

      timepop_pending = true;
      armed_count++;
      last_armed_dwt = ARM_DWT_CYCCNT;
      last_armed_name = name;
      update_deferred_high_water(slots_buf, max_slots, high_water);

      critical_exit(saved);
      return h;
    }
  }

  for (uint32_t i = 0; i < max_slots; i++) {
    if (deferred_entry_occupied(slots_buf[i])) continue;

    const timepop_handle_t h = allocate_handle_unlocked();

    slots_buf[i] = {};
    slots_buf[i].pending = true;
    slots_buf[i].handle = h;
    slots_buf[i].callback = callback;
    slots_buf[i].user_data = user_data;
    slots_buf[i].name = name;
    slots_buf[i].generation = 1;

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
  for (uint32_t i = 0; i < MAX_DISPATCH_MUTATIONS; i++) dispatch_mutations[i] = {};
  dispatch_mutation_count = 0;
  dispatch_depth = 0;
  dispatch_phase = timepop_dispatch_phase_t::IDLE;
  dispatch_applying_mutations = false;

  g_timepop_idle_witness_shadow_dwt = 0;
  diag_idle_witness_running = false;
  diag_idle_witness_enter_count = 0;
  diag_idle_witness_exit_count = 0;
  diag_idle_witness_pending_exit_count = 0;
  diag_idle_witness_last_enter_dwt = 0;
  diag_idle_witness_last_exit_dwt = 0;

  diag_ch2_direct_call_count = 0;
  diag_ch2_direct_body_cycles_last = 0;
  diag_ch2_direct_body_cycles_max = 0;
  diag_ch2_direct_last_event_counter32 = 0;
  diag_ch2_direct_last_event_dwt = 0;

  diag_asap_armed = 0;
  diag_asap_dispatched = 0;
  diag_asap_arm_failures = 0;
  diag_asap_last_armed_dwt = 0;
  diag_asap_last_dispatch_dwt = 0;
  diag_asap_last_armed_name = nullptr;
  diag_asap_replacements = 0;
  diag_asap_cancelled = 0;
  diag_asap_arm_while_dispatching = 0;

  diag_alap_armed = 0;
  diag_alap_dispatched = 0;
  diag_alap_arm_failures = 0;
  diag_alap_last_armed_dwt = 0;
  diag_alap_last_dispatch_dwt = 0;
  diag_alap_last_armed_name = nullptr;
  diag_alap_replacements = 0;
  diag_alap_cancelled = 0;
  diag_alap_arm_while_dispatching = 0;

  diag_dispatch_depth_max = 0;
  diag_dispatch_mutation_queued = 0;
  diag_dispatch_mutation_applied = 0;
  diag_dispatch_mutation_apply_failures = 0;
  diag_dispatch_mutation_overflow = 0;
  diag_dispatch_mutation_high_water = 0;
  diag_dispatch_mutation_apply_passes = 0;
  diag_dispatch_mutation_schedule_next_calls = 0;
  diag_dispatch_mutation_last_kind = nullptr;
  diag_dispatch_mutation_last_context = nullptr;
  diag_dispatch_mutation_last_phase = nullptr;
  diag_dispatch_mutation_last_handle = 0;
  diag_dispatch_mutation_last_queue_depth = 0;
  diag_dispatch_timed_arm_queued = 0;
  diag_dispatch_cancel_queued = 0;
  diag_dispatch_mutation_coalesced = 0;
  diag_dispatch_mutation_cancel_applied = 0;
  diag_dispatch_mutation_cancel_noop = 0;
  diag_dispatch_mutation_cancel_failures = 0;
  diag_dispatch_mutation_arm_failures = 0;
  diag_dispatch_mutation_name_copies = 0;
  diag_dispatch_mutation_name_too_long = 0;

  diag_schedule_next_body_cycles_last = 0;
  diag_schedule_next_body_cycles_max = 0;
  diag_schedule_next_arm_margin_ticks_last = 0;
  diag_schedule_next_arm_margin_ticks_min = UINT32_MAX;
  diag_schedule_next_arm_margin_ticks_max = 0;
  diag_schedule_next_final_safety_count = 0;
  diag_schedule_next_too_close_count = 0;
  diag_schedule_next_exact_no_irq_count = 0;
  diag_schedule_next_passed_count = 0;
  diag_schedule_next_invalid_time_count = 0;
  diag_schedule_next_too_close_max_ticks = 0;
  diag_schedule_next_passed_late_max_ticks = 0;
  diag_schedule_next_last_pressure_source = nullptr;
  diag_schedule_next_last_pressure_name = nullptr;
  diag_schedule_next_last_pressure_deadline = 0;
  diag_schedule_next_last_pressure_now = 0;
  diag_schedule_next_last_pressure_distance_ticks = 0;
  diag_schedule_next_last_pressure_late_ticks = 0;

  diag_timed_dispatch_latency_count = 0;
  diag_timed_dispatch_latency_last_cycles = 0;
  diag_timed_dispatch_latency_max_cycles = 0;
  diag_timed_dispatch_latency_cycles_sum = 0;
  diag_timed_dispatch_callback_body_last_cycles = 0;
  diag_timed_dispatch_callback_body_max_cycles = 0;
  diag_timed_dispatch_last_name = nullptr;

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
  // process_interrupt_init_hardware().  IRQ vector, capture priority, and
  // priority handoff are owned by process_interrupt.  We just register our
  // CH2 handler here; process_interrupt invokes it from its handoff context
  // with a fully-populated event/diag payload.
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
  uint32_t            target_dwt,
  timepop_priority_t  priority,
  timepop_handle_t    forced_handle
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

    const timepop_handle_t h =
        (forced_handle != TIMEPOP_INVALID_HANDLE)
            ? forced_handle
            : allocate_handle_unlocked();

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
    slot_set_name(slots[i], name, forced_handle != TIMEPOP_INVALID_HANDLE);
    slots[i].priority       = priority;
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
  const char*         name,
  timepop_priority_t  priority,
  timepop_handle_t    forced_handle
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

    const timepop_handle_t h =
        (forced_handle != TIMEPOP_INVALID_HANDLE)
            ? forced_handle
            : allocate_handle_unlocked();

    slots[i] = {};
    slots[i].active = true;
    slots[i].expired = false;
    slots[i].recurring = true;
    slots[i].handle = h;
    slots[i].callback = callback;
    slots[i].user_data = user_data;
    slot_set_name(slots[i], name, forced_handle != TIMEPOP_INVALID_HANDLE);
    slots[i].priority = priority;
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
  const char*         name,
  timepop_priority_t  priority,
  timepop_handle_t    forced_handle
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

    const timepop_handle_t h =
        (forced_handle != TIMEPOP_INVALID_HANDLE)
            ? forced_handle
            : allocate_handle_unlocked();

    slots[i] = {};
    slots[i].active = true;
    slots[i].expired = false;
    slots[i].recurring = true;
    slots[i].handle = h;
    slots[i].callback = callback;
    slots[i].user_data = user_data;
    slot_set_name(slots[i], name, forced_handle != TIMEPOP_INVALID_HANDLE);
    slots[i].priority = priority;
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
  bool                rearm_in_isr,
  timepop_priority_t  priority,
  timepop_handle_t    forced_handle
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

    const timepop_handle_t h =
        (forced_handle != TIMEPOP_INVALID_HANDLE)
            ? forced_handle
            : allocate_handle_unlocked();

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
    slot_set_name(slots[i], name, forced_handle != TIMEPOP_INVALID_HANDLE);
    slots[i].priority     = priority;
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
  return timepop_arm_with_priority(delay_gnss_ns,
                                   recurring,
                                   callback,
                                   user_data,
                                   name,
                                   TIMEPOP_PRIORITY_DEFAULT);
}

timepop_handle_t timepop_arm_with_priority(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
) {
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_relative_arm(delay_gnss_ns,
                                       recurring,
                                       callback,
                                       user_data,
                                       name,
                                       false,
                                       false,
                                       priority);
  }

  return arm_relative_slot_internal(delay_gnss_ns,
                                    recurring,
                                    callback,
                                    user_data,
                                    name,
                                    false,
                                    false,
                                    priority);
}

timepop_handle_t timepop_arm_recurring_isr(
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return timepop_arm_recurring_isr_with_priority(period_gnss_ns,
                                                 callback,
                                                 user_data,
                                                 name,
                                                 TIMEPOP_PRIORITY_DEFAULT);
}

timepop_handle_t timepop_arm_recurring_isr_with_priority(
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
) {
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_relative_arm(period_gnss_ns,
                                       true,
                                       callback,
                                       user_data,
                                       name,
                                       true,
                                       true,
                                       priority);
  }

  return arm_relative_slot_internal(period_gnss_ns,
                                    true,
                                    callback,
                                    user_data,
                                    name,
                                    true,
                                    true,
                                    priority);
}


timepop_handle_t timepop_arm_recurring_isr_from_base(
  int64_t             base_gnss_ns,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return timepop_arm_recurring_isr_from_base_with_priority(base_gnss_ns,
                                                           period_gnss_ns,
                                                           callback,
                                                           user_data,
                                                           name,
                                                           TIMEPOP_PRIORITY_DEFAULT);
}

timepop_handle_t timepop_arm_recurring_isr_from_base_with_priority(
  int64_t             base_gnss_ns,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
) {
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_anchored_isr_arm(base_gnss_ns,
                                           period_gnss_ns,
                                           callback,
                                           user_data,
                                           name,
                                           priority);
  }

  return arm_anchored_recurring_isr_internal(base_gnss_ns,
                                             period_gnss_ns,
                                             callback,
                                             user_data,
                                             name,
                                             priority);
}

timepop_handle_t timepop_arm_recurring_isr_from_base_counter32(
  int64_t             base_gnss_ns,
  uint32_t            base_counter32,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return timepop_arm_recurring_isr_from_base_counter32_with_priority(base_gnss_ns,
                                                                     base_counter32,
                                                                     period_gnss_ns,
                                                                     callback,
                                                                     user_data,
                                                                     name,
                                                                     TIMEPOP_PRIORITY_DEFAULT);
}

timepop_handle_t timepop_arm_recurring_isr_from_base_counter32_with_priority(
  int64_t             base_gnss_ns,
  uint32_t            base_counter32,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
) {
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_anchored_isr_counter32_arm(base_gnss_ns,
                                                     base_counter32,
                                                     period_gnss_ns,
                                                     callback,
                                                     user_data,
                                                     name,
                                                     priority);
  }

  return arm_anchored_recurring_isr_from_counter32_internal(base_gnss_ns,
                                                           base_counter32,
                                                           period_gnss_ns,
                                                           callback,
                                                           user_data,
                                                           name,
                                                           priority);
}

timepop_handle_t timepop_arm_asap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_deferred(asap_slots,
                      MAX_ASAP_SLOTS,
                      alap_slots,
                      MAX_ALAP_SLOTS,
                      diag_asap_arm_failures,
                      diag_asap_armed,
                      diag_asap_last_armed_dwt,
                      diag_asap_last_armed_name,
                      diag_asap_slots_high_water,
                      diag_asap_replacements,
                      diag_alap_cancelled,
                      diag_asap_arm_while_dispatching,
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
                      asap_slots,
                      MAX_ASAP_SLOTS,
                      diag_alap_arm_failures,
                      diag_alap_armed,
                      diag_alap_last_armed_dwt,
                      diag_alap_last_armed_name,
                      diag_alap_slots_high_water,
                      diag_alap_replacements,
                      diag_asap_cancelled,
                      diag_alap_arm_while_dispatching,
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
  return timepop_arm_at_with_priority(target_gnss_ns,
                                      false,
                                      callback,
                                      user_data,
                                      name,
                                      TIMEPOP_PRIORITY_DEFAULT);
}

timepop_handle_t timepop_arm_at_with_priority(
  int64_t             target_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
) {
  // Public absolute timers remain one-shot unless a future API supplies an
  // explicit absolute recurrence period.
  (void)recurring;
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_absolute_arm(target_gnss_ns,
                                       false,
                                       0,
                                       callback,
                                       user_data,
                                       name,
                                       false,
                                       0,
                                       priority);
  }

  return arm_absolute_slot_internal(target_gnss_ns,
                                    false,
                                    0,
                                    callback,
                                    user_data,
                                    name,
                                    false,
                                    0,
                                    priority);
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

  return timepop_arm_from_anchor_with_priority(anchor_gnss_ns,
                                               offset_gnss_ns,
                                               recurring,
                                               callback,
                                               user_data,
                                               name,
                                               TIMEPOP_PRIORITY_DEFAULT);
}

timepop_handle_t timepop_arm_from_anchor_with_priority(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (anchor_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  const int64_t target_gnss_ns = anchor_gnss_ns + offset_gnss_ns;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= anchor_gnss_ns && offset_gnss_ns > 0) {
    return TIMEPOP_INVALID_HANDLE;
  }

  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_absolute_arm(target_gnss_ns,
                                       recurring,
                                       recurring ? 1000000000ULL : 0ULL,
                                       callback,
                                       user_data,
                                       name,
                                       false,
                                       0,
                                       priority);
  }

  return arm_absolute_slot_internal(target_gnss_ns,
                                    recurring,
                                    recurring ? 1000000000ULL : 0ULL,
                                    callback,
                                    user_data,
                                    name,
                                    false,
                                    0,
                                    priority);
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
  return timepop_arm_ns_with_priority(target_gnss_ns,
                                      target_dwt,
                                      callback,
                                      user_data,
                                      name,
                                      isr_callback,
                                      TIMEPOP_PRIORITY_DEFAULT);
}

timepop_handle_t timepop_arm_ns_with_priority(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback,
  timepop_priority_t  priority
) {
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_absolute_arm(target_gnss_ns,
                                       false,
                                       0,
                                       callback,
                                       user_data,
                                       name,
                                       isr_callback,
                                       target_dwt,
                                       priority);
  }

  return arm_absolute_slot_internal(target_gnss_ns,
                                    false,
                                    0,
                                    callback,
                                    user_data,
                                    name,
                                    isr_callback,
                                    target_dwt,
                                    priority);
}

// ============================================================================
// Cancel
// ============================================================================

bool timepop_cancel(timepop_handle_t handle) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_cancel_handle(handle);
  }

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
  if (deferred_cancel_pending_by_handle_unlocked(asap_slots,
                                                 MAX_ASAP_SLOTS,
                                                 handle,
                                                 diag_asap_cancelled)) {
    critical_exit(saved);
    return true;
  }
  if (deferred_cancel_pending_by_handle_unlocked(alap_slots,
                                                 MAX_ALAP_SLOTS,
                                                 handle,
                                                 diag_alap_cancelled)) {
    critical_exit(saved);
    return true;
  }
  critical_exit(saved);
  return false;
}

uint32_t timepop_cancel_by_name(const char* name) {
  if (!name) return 0;
  if (timepop_should_queue_dispatch_mutation()) {
    return queue_dispatch_cancel_name(name) ? 1U : 0U;
  }

  uint32_t timed_cancelled = 0;
  uint32_t deferred_cancelled = 0;
  const uint32_t saved = critical_enter();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name && strcmp(slots[i].name, name) == 0) {
      slots[i].active = false;
      timed_cancelled++;
    }
  }

  deferred_cancelled +=
      deferred_cancel_pending_by_name_unlocked(asap_slots,
                                               MAX_ASAP_SLOTS,
                                               name,
                                               diag_asap_cancelled);
  deferred_cancelled +=
      deferred_cancel_pending_by_name_unlocked(alap_slots,
                                               MAX_ALAP_SLOTS,
                                               name,
                                               diag_alap_cancelled);

  if (timed_cancelled > 0) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
  }
  if (deferred_cancelled > 0 &&
      (deferred_any_pending(asap_slots, MAX_ASAP_SLOTS) ||
       deferred_any_pending(alap_slots, MAX_ALAP_SLOTS))) {
    timepop_pending = true;
  }

  critical_exit(saved);
  return timed_cancelled + deferred_cancelled;
}

// ============================================================================
// Dispatch
// ============================================================================

void timepop_dispatch(void) {
  {
    const uint32_t saved = critical_enter();
    if (!timepop_pending) {
      critical_exit(saved);
      timepop_idle_witness_spin_until_pending();
      return;
    }
    timepop_pending = false;
    critical_exit(saved);
  }

  diag_dispatch_calls++;

  timepop_dispatch_enter(timepop_dispatch_phase_t::ASAP);

  dispatch_deferred_phase(asap_slots,
                          MAX_ASAP_SLOTS,
                          diag_asap_dispatched,
                          diag_asap_last_dispatch_dwt);

  timepop_apply_dispatch_mutations("after_asap");
  timepop_dispatch_set_phase(timepop_dispatch_phase_t::TIMED);

  const uint32_t dispatch_now = vclock_count();
  (void)dispatch_now;

  bool scheduled_callback_dispatched[MAX_SLOTS] = {};
  for (;;) {
    const uint32_t i = select_next_scheduled_callback_slot_by_priority(
        scheduled_callback_dispatched);
    if (i >= MAX_SLOTS) break;
    scheduled_callback_dispatched[i] = true;

    // Keep the slot quarantined as expired while its scheduled-context
    // callback runs.  A callback is allowed to arm/cancel other TimePop work,
    // and those operations may call schedule_next().  If we clear expired
    // before the recurring rearm has installed a new deadline, schedule_next()
    // can rediscover this same slot with its old, already-past deadline and
    // manufacture a false SCHEDULE_NEXT expiry.  The slot remains expired until
    // its next appointment is fully authored below.
    const bool callback_already_ran = slots[i].isr_callback_fired;
    const timepop_handle_t callback_handle = slots[i].handle;
    slots[i].isr_callback_fired = false;

    if (!callback_already_ran) {
      // ── Timed scheduled-context callback ──
      timepop_ctx_t ctx;
      slot_build_ctx(slots[i], ctx);

      timepop_diag_t diag;
      slot_build_diag(slots[i], 0, diag);

      const uint32_t start = ARM_DWT_CYCCNT;

      if (ctx.fire_dwt_cyccnt != 0) {
        const uint32_t latency = start - ctx.fire_dwt_cyccnt;
        diag_timed_dispatch_latency_count++;
        diag_timed_dispatch_latency_last_cycles = latency;
        diag_timed_dispatch_latency_cycles_sum += (uint64_t)latency;
        if (latency > diag_timed_dispatch_latency_max_cycles) {
          diag_timed_dispatch_latency_max_cycles = latency;
        }
        diag_timed_dispatch_last_name = slots[i].name;
      }

      slots[i].callback(&ctx, &diag, slots[i].user_data);

      const uint32_t end = ARM_DWT_CYCCNT;
      const uint32_t body_cycles = end - start;
      diag_timed_dispatch_callback_body_last_cycles = body_cycles;
      if (body_cycles > diag_timed_dispatch_callback_body_max_cycles) {
        diag_timed_dispatch_callback_body_max_cycles = body_cycles;
      }

      cpu_usage_account_busy(body_cycles);
      diag_dispatch_callbacks++;

      timepop_apply_dispatch_mutations("timed_callback");
    }

    // The callback may have cancelled or otherwise retired this slot.  Do not
    // resurrect it by performing the normal recurring rearm path afterward.
    if (!slots[i].active) {
      slots[i].expired = false;
      slots[i].isr_callback_fired = false;
      continue;
    }
    if (slots[i].handle != callback_handle) {
      // A dispatch-time mutation replaced this table entry at a safe barrier.
      // Do not let the completed callback clean up or rearm the new occupant.
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
          diag_schedule_next_calls_from_dispatch++;
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
        diag_schedule_next_calls_from_dispatch++;
        schedule_next();
        critical_exit(saved);
      }
    } else {
      slots[i].active = false;
      slots[i].expired = false;
      slots[i].isr_callback_fired = false;
    }
  }

  timepop_apply_dispatch_mutations("after_timed");
  timepop_dispatch_set_phase(timepop_dispatch_phase_t::ALAP);

  dispatch_deferred_phase(alap_slots,
                          MAX_ALAP_SLOTS,
                          diag_alap_dispatched,
                          diag_alap_last_dispatch_dwt);

  timepop_apply_dispatch_mutations("after_alap");

  if (deferred_any_pending(asap_slots, MAX_ASAP_SLOTS) ||
      deferred_any_pending(alap_slots, MAX_ALAP_SLOTS) ||
      dispatch_mutation_count != 0) {
    timepop_pending = true;
  }

  timepop_dispatch_leave();

  if (!timepop_pending) {
    timepop_idle_witness_spin_until_pending();
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

  asap_active = deferred_occupied_count(asap_slots, MAX_ASAP_SLOTS);
  alap_active = deferred_occupied_count(alap_slots, MAX_ALAP_SLOTS);

  // Timed expirations from the old epoch are invalid.  Deferred ASAP/ALAP
  // callbacks are not VCLOCK-deadline facts, so leave them in place.
  timepop_pending =
      deferred_any_pending(asap_slots, MAX_ASAP_SLOTS) ||
      deferred_any_pending(alap_slots, MAX_ALAP_SLOTS);

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
  n += deferred_occupied_count(asap_slots, MAX_ASAP_SLOTS);
  n += deferred_occupied_count(alap_slots, MAX_ALAP_SLOTS);
  return n;
}

bool timepop_idle_witness_snapshot(timepop_idle_witness_snapshot_t* out) {
  if (!out) return false;

  out->supported = true;
  out->enabled = TIMEPOP_IDLE_DWT_WITNESS_ENABLED;
  out->running = diag_idle_witness_running;
  out->shadow_dwt = g_timepop_idle_witness_shadow_dwt;
  out->enter_count = diag_idle_witness_enter_count;
  out->exit_count = diag_idle_witness_exit_count;
  out->pending_exit_count = diag_idle_witness_pending_exit_count;
  out->last_enter_dwt = diag_idle_witness_last_enter_dwt;
  out->last_exit_dwt = diag_idle_witness_last_exit_dwt;
  out->total_cycles = diag_idle_witness_total_cycles;
  out->last_residency_cycles = diag_idle_witness_last_residency_cycles;
  out->snapshot_dwt = ARM_DWT_CYCCNT;
  timepop_idle_witness_note_wall_cycles(out->snapshot_dwt);
  out->snapshot_total_cycles = out->total_cycles;
  if (out->running) {
    out->snapshot_total_cycles +=
        (uint64_t)(uint32_t)(out->snapshot_dwt - out->last_enter_dwt);
  }
  out->snapshot_wall_cycles = diag_idle_witness_wall_cycles;
  return true;
}

// ============================================================================
// REPORT
// ============================================================================

static FLASHMEM Payload cmd_report(const Payload&) {
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
  timepop_idle_witness_snapshot_t idle_witness{};
  timepop_idle_witness_snapshot(&idle_witness);
  out.add("idle_dwt_witness_supported", idle_witness.supported);
  out.add("idle_dwt_witness_enabled", idle_witness.enabled);
  out.add("idle_dwt_witness_running", idle_witness.running);
  out.add("idle_dwt_shadow_dwt", idle_witness.shadow_dwt);
  out.add("idle_dwt_witness_enter_count", idle_witness.enter_count);
  out.add("idle_dwt_witness_exit_count", idle_witness.exit_count);
  out.add("idle_dwt_witness_pending_exit_count", idle_witness.pending_exit_count);
  out.add("idle_dwt_witness_last_enter_dwt", idle_witness.last_enter_dwt);
  out.add("idle_dwt_witness_last_exit_dwt", idle_witness.last_exit_dwt);
  out.add("idle_dwt_witness_total_cycles", idle_witness.snapshot_total_cycles);
  out.add("idle_dwt_witness_completed_cycles", idle_witness.total_cycles);
  out.add("idle_dwt_witness_last_residency_cycles", idle_witness.last_residency_cycles);
  out.add("idle_dwt_witness_snapshot_dwt", idle_witness.snapshot_dwt);
  out.add("idle_dwt_witness_wall_cycles", idle_witness.snapshot_wall_cycles);
  out.add("time_pps_count",    time_pps_count());

  out.add("slot_priority_supported", true);
  out.add("slot_priority_ordering_enabled", TIMEPOP_SLOT_PRIORITY_ORDERING_ENABLED);
  out.add("slot_priority_first", (uint32_t)TIMEPOP_PRIORITY_FIRST);
  out.add("slot_priority_default", (uint32_t)TIMEPOP_PRIORITY_DEFAULT);
  out.add("slot_priority_last", (uint32_t)TIMEPOP_PRIORITY_LAST);

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
  out.add("asap_pending_now",       deferred_pending_count(asap_slots, MAX_ASAP_SLOTS));
  out.add("asap_dispatching_now",   deferred_dispatching_count(asap_slots, MAX_ASAP_SLOTS));
  out.add("asap_replacements",      diag_asap_replacements);
  out.add("asap_cancelled",         diag_asap_cancelled);
  out.add("asap_arm_while_dispatching", diag_asap_arm_while_dispatching);
  out.add("asap_non_slot_deferred", true);
  out.add("asap_last_armed_dwt",    diag_asap_last_armed_dwt);
  out.add("asap_last_dispatch_dwt", diag_asap_last_dispatch_dwt);
  out.add("asap_last_armed_name",
          (const char*)(diag_asap_last_armed_name ? diag_asap_last_armed_name : ""));

  out.add("alap_armed",             diag_alap_armed);
  out.add("alap_dispatched",        diag_alap_dispatched);
  out.add("alap_arm_failures",      diag_alap_arm_failures);
  out.add("alap_slots_high_water",  diag_alap_slots_high_water);
  out.add("alap_slots_max",         (uint32_t)MAX_ALAP_SLOTS);
  out.add("alap_pending_now",       deferred_pending_count(alap_slots, MAX_ALAP_SLOTS));
  out.add("alap_dispatching_now",   deferred_dispatching_count(alap_slots, MAX_ALAP_SLOTS));
  out.add("alap_replacements",      diag_alap_replacements);
  out.add("alap_cancelled",         diag_alap_cancelled);
  out.add("alap_arm_while_dispatching", diag_alap_arm_while_dispatching);
  out.add("alap_non_slot_deferred", true);
  out.add("alap_last_armed_dwt",    diag_alap_last_armed_dwt);
  out.add("alap_last_dispatch_dwt", diag_alap_last_dispatch_dwt);
  out.add("alap_last_armed_name",
          (const char*)(diag_alap_last_armed_name ? diag_alap_last_armed_name : ""));

  out.add("dispatch_calls",         diag_dispatch_calls);
  out.add("dispatch_callbacks",     diag_dispatch_callbacks);
  out.add("dispatch_depth_now",     dispatch_depth);
  out.add("dispatch_depth_max",     diag_dispatch_depth_max);
  out.add("dispatch_phase",
          (const char*)dispatch_phase_str(dispatch_phase));
  out.add("dispatch_mutation_supported", true);
  out.add("dispatch_mutation_queue_depth", dispatch_mutation_count);
  out.add("dispatch_mutation_queue_max", (uint32_t)MAX_DISPATCH_MUTATIONS);
  out.add("dispatch_mutation_high_water", diag_dispatch_mutation_high_water);
  out.add("dispatch_mutation_queued", diag_dispatch_mutation_queued);
  out.add("dispatch_mutation_applied", diag_dispatch_mutation_applied);
  out.add("dispatch_mutation_apply_failures", diag_dispatch_mutation_apply_failures);
  out.add("dispatch_mutation_overflow", diag_dispatch_mutation_overflow);
  out.add("dispatch_mutation_apply_passes", diag_dispatch_mutation_apply_passes);
  out.add("dispatch_mutation_schedule_next_calls",
          diag_dispatch_mutation_schedule_next_calls);
  out.add("dispatch_timed_arm_queued", diag_dispatch_timed_arm_queued);
  out.add("dispatch_cancel_queued", diag_dispatch_cancel_queued);
  out.add("dispatch_mutation_coalesced", diag_dispatch_mutation_coalesced);
  out.add("dispatch_mutation_cancel_applied", diag_dispatch_mutation_cancel_applied);
  out.add("dispatch_mutation_cancel_noop", diag_dispatch_mutation_cancel_noop);
  out.add("dispatch_mutation_cancel_failures", diag_dispatch_mutation_cancel_failures);
  out.add("dispatch_mutation_arm_failures", diag_dispatch_mutation_arm_failures);
  out.add("dispatch_mutation_name_copies", diag_dispatch_mutation_name_copies);
  out.add("dispatch_mutation_name_too_long", diag_dispatch_mutation_name_too_long);
  out.add("dispatch_mutation_last_kind",
          (const char*)(diag_dispatch_mutation_last_kind
                            ? diag_dispatch_mutation_last_kind
                            : ""));
  out.add("dispatch_mutation_last_context",
          (const char*)(diag_dispatch_mutation_last_context
                            ? diag_dispatch_mutation_last_context
                            : ""));
  out.add("dispatch_mutation_last_phase",
          (const char*)(diag_dispatch_mutation_last_phase
                            ? diag_dispatch_mutation_last_phase
                            : ""));
  out.add("dispatch_mutation_last_handle", diag_dispatch_mutation_last_handle);
  out.add("dispatch_mutation_last_queue_depth",
          diag_dispatch_mutation_last_queue_depth);
  out.add("schedule_next_calls_total",         diag_schedule_next_calls_total);
  out.add("schedule_next_calls_from_dispatch", diag_schedule_next_calls_from_dispatch);
  out.add("schedule_next_calls_from_other",    diag_schedule_next_calls_from_other);
  out.add("schedule_pressure_supported", diag_schedule_pressure_supported);
  out.add("schedule_next_body_cycles_last", diag_schedule_next_body_cycles_last);
  out.add("schedule_next_body_cycles_max", diag_schedule_next_body_cycles_max);
  out.add("schedule_next_arm_margin_ticks_last",
          diag_schedule_next_arm_margin_ticks_last);
  out.add("schedule_next_arm_margin_ticks_min",
          diag_schedule_next_arm_margin_ticks_min == UINT32_MAX
              ? 0U
              : diag_schedule_next_arm_margin_ticks_min);
  out.add("schedule_next_arm_margin_ticks_max",
          diag_schedule_next_arm_margin_ticks_max);
  out.add("schedule_next_final_safety_count",
          diag_schedule_next_final_safety_count);
  out.add("schedule_next_too_close_count", diag_schedule_next_too_close_count);
  out.add("schedule_next_exact_no_irq_count",
          diag_schedule_next_exact_no_irq_count);
  out.add("schedule_next_passed_count", diag_schedule_next_passed_count);
  out.add("schedule_next_invalid_time_count",
          diag_schedule_next_invalid_time_count);
  out.add("schedule_next_too_close_max_ticks",
          diag_schedule_next_too_close_max_ticks);
  out.add("schedule_next_passed_late_max_ticks",
          diag_schedule_next_passed_late_max_ticks);
  out.add("schedule_next_last_pressure_source",
          (const char*)(diag_schedule_next_last_pressure_source
                            ? diag_schedule_next_last_pressure_source
                            : ""));
  out.add("schedule_next_last_pressure_name",
          (const char*)(diag_schedule_next_last_pressure_name
                            ? diag_schedule_next_last_pressure_name
                            : ""));
  out.add("schedule_next_last_pressure_deadline",
          diag_schedule_next_last_pressure_deadline);
  out.add("schedule_next_last_pressure_now",
          diag_schedule_next_last_pressure_now);
  out.add("schedule_next_last_pressure_distance_ticks",
          diag_schedule_next_last_pressure_distance_ticks);
  out.add("schedule_next_last_pressure_late_ticks",
          diag_schedule_next_last_pressure_late_ticks);

  out.add("timed_dispatch_latency_count", diag_timed_dispatch_latency_count);
  out.add("timed_dispatch_latency_last_cycles",
          diag_timed_dispatch_latency_last_cycles);
  out.add("timed_dispatch_latency_max_cycles",
          diag_timed_dispatch_latency_max_cycles);
  out.add("timed_dispatch_latency_mean_cycles",
          diag_timed_dispatch_latency_count
              ? (uint32_t)(diag_timed_dispatch_latency_cycles_sum /
                           (uint64_t)diag_timed_dispatch_latency_count)
              : 0U);
  out.add("timed_dispatch_callback_body_last_cycles",
          diag_timed_dispatch_callback_body_last_cycles);
  out.add("timed_dispatch_callback_body_max_cycles",
          diag_timed_dispatch_callback_body_max_cycles);
  out.add("timed_dispatch_last_name",
          (const char*)(diag_timed_dispatch_last_name
                            ? diag_timed_dispatch_last_name
                            : ""));
  out.add("ch2_service_context", TIMEPOP_CH2_SERVICE_CONTEXT);
  out.add("ch2_direct_call_count", diag_ch2_direct_call_count);
  out.add("ch2_direct_body_cycles_last", diag_ch2_direct_body_cycles_last);
  out.add("ch2_direct_body_cycles_max", diag_ch2_direct_body_cycles_max);
  out.add("ch2_direct_last_event_counter32", diag_ch2_direct_last_event_counter32);
  out.add("ch2_direct_last_event_dwt", diag_ch2_direct_last_event_dwt);

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

  out.add("missed_deadline_slots", diag_missed_deadline_slots);
  out.add("missed_deadline_one_shot_cancelled", diag_missed_deadline_one_shot_cancelled);
  out.add("missed_deadline_recurring_rearmed", diag_missed_deadline_recurring_rearmed);
  out.add("missed_deadline_recurring_rearm_failures", diag_missed_deadline_recurring_rearm_failures);
  out.add("missed_deadline_too_close_count", diag_missed_deadline_too_close_count);
  out.add("missed_deadline_late_max_ticks", diag_missed_deadline_late_max_ticks);
  out.add("missed_deadline_last_slot", diag_missed_deadline_last_slot);
  out.add("missed_deadline_last_handle", diag_missed_deadline_last_handle);
  out.add("missed_deadline_last_name",
          (const char*)(diag_missed_deadline_last_name
                            ? diag_missed_deadline_last_name
                            : ""));
  out.add("missed_deadline_last_source",
          (const char*)(diag_missed_deadline_last_source
                            ? diag_missed_deadline_last_source
                            : ""));
  out.add("missed_deadline_last_deadline", diag_missed_deadline_last_deadline);
  out.add("missed_deadline_last_now", diag_missed_deadline_last_now);
  out.add("missed_deadline_last_late_ticks", diag_missed_deadline_last_late_ticks);
  out.add("missed_deadline_last_dwt", diag_missed_deadline_last_dwt);

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
// ISR-named slots can callback and rearm inside this CH2 scheduler pass before the next
// compare is selected.


static FLASHMEM void add_timers_array(Payload& out) {
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
    entry.add("priority",  (uint32_t)slots[i].priority);
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

static FLASHMEM Payload cmd_slots(const Payload&) {
  Payload out;
  out.add("slots_active_now", timepop_active_count());
  out.add("dispatch_mutation_queue_depth", dispatch_mutation_count);
  out.add("dispatch_mutation_queued", diag_dispatch_mutation_queued);
  out.add("dispatch_mutation_applied", diag_dispatch_mutation_applied);
  out.add("dispatch_mutation_apply_failures", diag_dispatch_mutation_apply_failures);
  out.add("dispatch_mutation_arm_failures", diag_dispatch_mutation_arm_failures);
  out.add("dispatch_mutation_name_copies", diag_dispatch_mutation_name_copies);
  out.add("dispatch_mutation_name_too_long", diag_dispatch_mutation_name_too_long);
  out.add("dispatch_mutation_cancel_failures", diag_dispatch_mutation_cancel_failures);
  out.add("dispatch_mutation_cancel_noop", diag_dispatch_mutation_cancel_noop);
  out.add("dispatch_mutation_coalesced", diag_dispatch_mutation_coalesced);
  out.add("dispatch_mutation_overflow", diag_dispatch_mutation_overflow);
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
  out.add("schedule_next_too_close_count", diag_schedule_next_too_close_count);
  out.add("schedule_next_passed_count", diag_schedule_next_passed_count);
  out.add("schedule_next_final_safety_count", diag_schedule_next_final_safety_count);
  out.add("schedule_next_body_cycles_max", diag_schedule_next_body_cycles_max);
  out.add("schedule_next_arm_margin_ticks_min",
          diag_schedule_next_arm_margin_ticks_min == UINT32_MAX
              ? 0U
              : diag_schedule_next_arm_margin_ticks_min);
  out.add("timed_dispatch_latency_max_cycles",
          diag_timed_dispatch_latency_max_cycles);
  out.add("ch2_service_context", TIMEPOP_CH2_SERVICE_CONTEXT);
  out.add("ch2_direct_call_count", diag_ch2_direct_call_count);
  out.add("ch2_direct_body_cycles_max", diag_ch2_direct_body_cycles_max);
  out.add("schedule_next_expired_slots", diag_schedule_next_expired_slots);
  out.add("schedule_next_late_max_ticks", diag_schedule_next_late_max_ticks);
  out.add("missed_deadline_slots", diag_missed_deadline_slots);
  out.add("missed_deadline_recurring_rearmed", diag_missed_deadline_recurring_rearmed);
  out.add("missed_deadline_one_shot_cancelled", diag_missed_deadline_one_shot_cancelled);
  out.add("missed_deadline_last_name",
          (const char*)(diag_missed_deadline_last_name
                            ? diag_missed_deadline_last_name
                            : ""));
  out.add("missed_deadline_last_late_ticks", diag_missed_deadline_last_late_ticks);
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
