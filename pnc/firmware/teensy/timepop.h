#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// timepop.h — Public Interface
// ============================================================================
//
// TimePop is ZPNet's phase-locked timer subsystem.
//
// Core model:
//   • QTimer1 CH0 is the passive VCLOCK counter at 10 MHz.
//   • One VCLOCK tick = 100 ns.
//   • QTimer1 CH2 is the dynamic compare scheduler.
//   • Ordinary timers use GNSS nanoseconds; the opt-in action lane uses raw DWT.
//
// Scheduling modes:
//
//   1. Relative scheduling
//        timepop_arm(delay_gnss_ns, ...)
//
//      Use this when the caller only cares about "fire after this delay."
//      If recurring=true, TimePop will phase-lock the recurrence to the
//      current lawful GNSS/VCLOCK anchor when one is available.
//
//   2. Scheduled-context ASAP dispatch
//        timepop_arm_asap(...)
//
//   3. Scheduled-context ALAP dispatch
//        timepop_arm_alap(...)
//
//   4. Absolute GNSS scheduling
//        timepop_arm_at(target_gnss_ns, ...)
//
//   5. Anchor-relative GNSS scheduling
//        timepop_arm_from_anchor(anchor_gnss_ns, offset_gnss_ns, ...)
//
//   6. Caller-owned exact target scheduling
//        timepop_arm_ns(target_gnss_ns, target_dwt, ...)
//
//   7. Critical recurring ISR scheduling
//        timepop_arm_recurring_isr(period_gnss_ns, ...)
//
//      Legacy name: these callbacks run in the foreground CH2 service pass,
//      before TimePop selects the next compare. They do NOT execute in an ISR.
//      The callback receives the same shared fire facts as other same-deadline
//      timed clients.
//
//   8. Recurring foreground service
//        timepop_arm_recurring_service(period_gnss_ns, ...)
//
//      For polling/publication work that must still run if its appointment is
//      missed. Overdue periods coalesce into one pending callback, followed by
//      a future appointment. This API never supplies precision fire facts.
//
//   9. Short-deadline interrupt action
//        timepop_action_arm_dwt(target_dwt, max_lateness_cycles, ...)
//
//      Separate one-shot CH1 lane; no ordinary slot, handle or recurrence.
//      Priority 16 wakes early and lands on a raw DWT deadline. Priority 0
//      science/photodiode capture remains able to preempt it.
//
// TimePop owns:
//   • timed slot scheduling
//   • deferred ASAP/ALAP scheduled-context dispatch
//   • next-deadline selection
//   • CH2 compare arming
//   • absolute recurring timer series
//   • Spin-Dry early-wake + deterministic landing for VCLOCK scheduling
//   • timer diagnostics
//   • always-on internal VCLOCK monitor
//
// ============================================================================

// Foreground readiness/service registry (maximum 8 lifetime registrations).
// Register once during client initialization after timepop_init(), in foreground.
// Predicates must be bounded, nonblocking and observational: no scheduler or
// registry mutation, dispatch, or application work. Synchronize ISR-owned reads
// within the predicate. True requests an idle exit, not a timed callback;
// the ordinary foreground loop must service the client's ready work.
// Callback/user_data pairs are unique; null callbacks, duplicates, capacity
// exhaustion and registration from ISR/dispatch/predicate/service execution trap.
// user_data must remain alive until reboot. No unregister operation is provided.
typedef bool (*timepop_foreground_ready_fn)(void* user_data);
void timepop_register_foreground_ready(timepop_foreground_ready_fn callback,
                                       void* user_data);

// Register a foreground-owned work source without a per-service timer or ALAP
// mailbox. Shares the registry above; the ready/user_data pair must be unique.
// TimePop checks readiness and calls service at most once per dispatch pass,
// after scheduled/deferred work, with the idle witness stopped. Pending TimePop
// or interrupt-bridge work takes precedence; a new interrupt can still arrive
// after the final check, so services must be bounded and promptly return.
// Registry evaluation/service adds no interrupt mask. Clients synchronize their
// own ISR state without blocking CLOCKS capture. Services must not recursively
// dispatch or register clients. They may arm ordinary future TimePop work.
// Both callbacks and user_data live until reboot; null callbacks trap.
typedef void (*timepop_foreground_service_fn)(void* user_data);
void timepop_register_foreground_service(timepop_foreground_ready_fn ready,
                                         timepop_foreground_service_fn service,
                                         void* user_data);

typedef uint32_t timepop_handle_t;
static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// Callback context — authoritative fire facts
// ============================================================================

typedef struct timepop_ctx_t {
  timepop_handle_t handle;
  uint32_t fire_vclock_raw;
  uint32_t fire_dwt_cyccnt;
  uint32_t deadline;
  int32_t  fire_gnss_error_ns;
  int64_t  fire_gnss_ns;
} timepop_ctx_t;

// ============================================================================
// Optional diagnostics
// ============================================================================
//
// Populated for timed callbacks and ISR callbacks.
// Both ctx and diag are null for ASAP/ALAP and recurring service callbacks.
//
// The authoritative result is still timepop_ctx_t. These diagnostics expose
// Spin-Dry / landing facts that are useful for instrumentation and later
// TIMEBASE_FRAGMENT analysis.
//

typedef struct timepop_diag_t {
  uint32_t dwt_at_isr_entry;
  uint32_t dwt_at_fire;

  uint32_t predicted_dwt;
  bool     prediction_valid;

  bool     spin_dry_used;
  uint32_t wake_target_dwt;
  int32_t  wake_error_cycles;

  int32_t  spin_error_cycles;

  uint32_t anchor_pps_count;
  uint32_t anchor_qtimer_at_pps;
  uint32_t anchor_dwt_at_pps;
  uint32_t anchor_dwt_cycles_per_s;
  bool     anchor_valid;
} timepop_diag_t;

typedef void (*timepop_callback_t)(
  timepop_ctx_t*  ctx,
  timepop_diag_t* diag,
  void*           user_data
);

timepop_handle_t timepop_arm(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// Foreground only; callback receives nullptr for both ctx and diag, even when
// CH2 captured the appointment exactly. Never dispatches early or replays missed
// periods. Uses the phase grid when available, otherwise a relative period.
// Existing cancellation, epoch rebasing and dispatch mutation rules apply.
timepop_handle_t timepop_arm_recurring_service(
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_recurring_isr(
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_asap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_alap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_at(
  int64_t             target_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_from_anchor(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_ns(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback = false
);

bool timepop_cancel(timepop_handle_t handle);
uint32_t timepop_cancel_by_name(const char* name);
void timepop_dispatch(void);
uint32_t timepop_active_count(void);

// ============================================================================
// Short-deadline action — explicit opt-in, independent of the CH2 scheduler
// ============================================================================
//
// One foreground owner, one outstanding action, one result to consume. All
// functions below are foreground-only. A second arm before take() is an
// integrity violation. Ordinary cancel/cancel_by_name/active_count do not own
// this lane. Hardware stays dormant until its first arm; no boot callback.
//
// target_dwt is an absolute *raw* ARM_DWT_CYCCNT coordinate, including wrap.
// For LOW: actual launch DWT + predicted raw flight cycles - LOW lead cycles.
// PHOTONS owns prediction, ADC ownership and the aperture/arrival margin.
// This API schedules callback dispatch, not ADC aperture or conversion end.
//
// arm() returns false if insufficient lead remains (including time spent
// programming hardware); nothing is queued and no result needs consuming.
// Null callback, ISR use, occupied lane, excessive future horizon or lateness
// tolerance trap. Limits are exposed by snapshot() in CPU cycles.
//
// The callback runs at Priority 16, without masking Priority 0. It must be a
// tiny, bounded, integer-only hardware action: trigger acquisition and return.
// No ADC completion wait, allocation, logging, scheduler calls or floating
// point. user_data must live until take() consumes the completion/cancellation.
//
// Dispatch occurs no earlier than target_dwt; an already-late action is skipped.
// Priority 0 can still preempt between the dispatch check and the peripheral
// write. The client must record its actual trigger time and assess the ADC's
// sampling aperture against arrival. DISPATCHED is not a timing guarantee for
// a later register write. A hardware ADC trigger would be needed for that.
//
enum class timepop_action_state_t : uint8_t { IDLE, ARMED, COMPLETE };
enum class timepop_action_outcome_t : uint8_t { NONE, DISPATCHED, MISSED, CANCELLED };

struct timepop_action_context_t {
  uint32_t target_dwt = 0;
  uint32_t latest_dwt = 0;
  uint32_t isr_entry_dwt = 0;
  uint32_t dispatch_dwt = 0;
};

using timepop_action_callback_t =
    void (*)(const timepop_action_context_t& context, void* user_data);

struct timepop_action_result_t {
  timepop_action_outcome_t outcome = timepop_action_outcome_t::NONE;
  timepop_action_context_t context;
  uint32_t finished_dwt = 0;
};

struct timepop_action_snapshot_t {
  timepop_action_state_t state = timepop_action_state_t::IDLE;
  uint32_t min_lead_cycles = 0;
  uint32_t max_lead_cycles = 0;
  uint32_t wake_advance_cycles = 0;
  uint32_t max_spin_cycles = 0;
  uint32_t max_lateness_cycles = 0;
  uint32_t arm_count = 0;
  uint32_t arm_too_late_count = 0;
  uint32_t dispatched_count = 0;
  uint32_t missed_count = 0;
  uint32_t cancelled_count = 0;
  uint32_t dispatch_lateness_max_cycles = 0;
  uint32_t callback_wall_max_cycles = 0;
  timepop_action_result_t last;
};

bool timepop_action_arm_dwt(uint32_t target_dwt,
                           uint32_t max_lateness_cycles,
                           timepop_action_callback_t callback,
                           void* user_data);
// Returns false if no action is armed; never erases an already completed result.
bool timepop_action_cancel(void);
// Returns true exactly once per completion; copies the result and releases lane.
bool timepop_action_take(timepop_action_result_t* out);
void timepop_action_snapshot(timepop_action_snapshot_t* out);
