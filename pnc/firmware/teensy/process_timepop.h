// ============================================================================
// process_timepop.h
// ============================================================================
//
// TimePop process interface.
//
// TimePop owns:
//   • priority-queue slot scheduling
//   • PPS/VCLOCK phase-locked recurring series
//   • shared captured fire facts for same-event timed clients
//   • critical recurring scheduler clients that callback/rearm in foreground
//     before schedule_next
//   • fixed non-slot foreground deferred callback dispatch
//   • instrumentation / reports
//   • scheduler policy for QTimer1 CH2 compare deadlines
//
// TimePop does not own:
//   • PPS/GPIO interrupt custody
//   • OCXO interrupt custody
//   • QTimer1 vector custody — process_interrupt owns IRQ_QTIMER1, transfers
//     CH2 into an immutable foreground fact mailbox, and invokes TimePop only from
//     the ordinary loop
//   • QTimer1 CH0/CH2 hardware mode init — process_interrupt does
//     the one-time CTRL/SCTRL/CSCTRL/COMP1/CMPLD1 setup
//
// Those are owned by process_interrupt. TimePop also does not own a private
// scheduler IRQ / priority-handoff tier; all CH2 scheduler processing runs in
// foreground after process_interrupt drains its immutable fact mailbox.
//
// QTimer1 CH2 foreground-ingress API (TimePop is the hosted client):
//
//   QTimer1 has only one IRQ vector shared across all four channels.
//   process_interrupt owns the vector and transfers CH2 through Priority 16
//   into a foreground fact mailbox.  VCLOCK cadence is no longer a separate QTimer1
//   compare path; it is a TimePop critical recurring scheduler client.
//
//   Foreground ingress receives a normalized DWT-at-edge capture for CH2.
//
//   For timed TimePop clients, all slots reached by one physical CH2 event
//   receive the same fire_vclock_raw, fire_dwt_cyccnt, and fire_gnss_ns.
//   Equality with the compare target is the normal fire condition.  Callback
//   order is logical only.  ASAP/ALAP are intentionally excluded from this
//   guarantee because they are fixed scheduled-context deferred lanes,
//   not timed CH2 clients.
//
//   For CH2, the dispatcher authors the synthetic counter32_at_event and a
//   latency-normalized DWT event coordinate, then packages them into a standard
//   interrupt_event_t (kind = TIMEPOP) and interrupt_capture_diag_t.  TimePop
//   treats counter32_at_event as the authoritative VCLOCK identity and authors
//   fire_gnss_ns from VCLOCK arithmetic.  DWT-derived GNSS values remain
//   diagnostic/cross-check material, not event identity.
//
//   The PPS epoch consumed through time_anchor_snapshot() is already
//   process_interrupt's canonical VCLOCK-selected epoch.  TimePop does
//   not reinterpret physical PPS GPIO timing; it schedules in that
//   canonical VCLOCK/GNSS coordinate system.
//
//   TimePop is responsible for:
//     • accepting immutable CH2 facts from process_interrupt in foreground
//     • requesting CH2 compare target updates from process_interrupt as
//       slots are scheduled
//
//   process_interrupt is responsible for:
//     • initializing CH0/CH2 hardware (one-time mode/control)
//     • disabling/clearing the fired CH2 compare in Priority 0
//     • normalizing and storing its immutable event identity in Priority 16
//     • building the event/diag payloads passed to TimePop in foreground
//
//   TimePop does not defensively second-guess exact compare fires.  Exact
//   equality between a slot deadline and the process_interrupt-authored CH2
//   event counter is the only condition that can produce a timed callback fire.
//   If IRQ scan or schedule_next() discovers an already-past timed slot, that
//   deadline is quarantined as missed: one-shots are cancelled, recurring slots
//   are advanced to their next lawful grid point, and no callback receives the
//   current event DWT under the old deadline's identity.  schedule_next() is
//   not an event source: it may choose the next compare target, quarantine
//   missed/too-close slots, or re-author recurring work, but it never authors
//   a timed fire fact.
//
//   TimePop also refuses to arm CH2 at-or-inside the scheduler race window.
//   A too-close deadline is treated as missed before hardware programming,
//   preventing immediate/missed-compare storms.  While CLOCKS/SmartZero is
//   intentionally invalidating the public TIME anchor for epoch acquisition,
//   old absolute deadlines are degraded/cancelled without callback rather than
//   allowed to survive as old-coordinate facts.
//
//   ASAP/ALAP:
//
//   timepop_arm_asap() and timepop_arm_alap() keep their public API, but are
//   implemented as fixed foreground deferred mailboxes rather than scheduled
//   slots.  Under the execution-class doctrine they are requested and dispatched
//   only in foreground.  ASAP drains before timed slots; ALAP drains after them.
//
//   ASAP/ALAP callbacks are normal scheduled-context code and may call the
//   public timed TimePop arm/cancel APIs.  TimePop serializes those requests
//   internally: while a dispatch pass is running, timed-slot arms/cancels are
//   copied into a fixed, allocation-free mutation queue and applied at dispatch
//   barriers immediately after the current callback returns.  Clients do not
//   need to know whether they are running inside ASAP, ALAP, or an ordinary
//   timed scheduled callback.  The active timed-slot table and CH2 compare
//   target are mutated only at those safe barriers, never while user callback
//   code is still on the stack.  Cancel operations are idempotent at the
//   mutation barrier: cancelling an already-absent handle or name is a
//   successful no-op, not a scheduler failure.
//
//   Critical recurring scheduler clients are the white-glove exception for tiny
//   substrate-maintenance work.  They are still ordinary TimePop slots and
//   share CH2 fire facts, but their callbacks and recurring rearm happen in the
//   foreground CH2 scheduler pass before schedule_next() selects the next compare.
//   The legacy ISR API names remain source-compatible but no callback executes
//   in Priority 0 or Priority 16.
//
//   Timed slots may also carry a service priority. Lower numeric priority runs
//   first when multiple timed slots share one exact CH2 fire fact. Priority
//   never changes compare scheduling and never applies to ASAP/ALAP. Existing
//   callers use TIMEPOP_PRIORITY_DEFAULT and retain stable slot-index order.
//
//   Name ownership:
//
//   TimePop copies every accepted name into fixed internal storage before an
//   arm operation returns.  Timed slots never retain caller pointers.  Each
//   deferred mailbox has separate owned pending and dispatching name buffers,
//   because a callback may re-arm the same mailbox while the prior request is
//   still executing.  Names longer than 63 characters are rejected rather
//   than truncated.
//
// ============================================================================

#pragma once

#include "timepop.h"
#include "process_interrupt.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);

// Foreground-only CH2 ingress. process_interrupt Priority 16 stores one immutable
// fire fact; process_interrupt_foreground_service() calls this before
// timepop_dispatch().  The capture-loss service restores only future compare
// custody and never invents the missing physical edge.
void timepop_accept_ch2_event_foreground(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t& diag);
void timepop_ch2_capture_lost_foreground(void);

// Global idle DWT witness.  TimePop updates this foreground/thread-mode
// shadow whenever scheduled-context dispatch is fully drained and the runtime
// would otherwise be idle.  ISR clients may read the exported volatile shadow
// directly; command/report code should use the snapshot helper.
extern volatile uint32_t g_timepop_idle_witness_shadow_dwt;

struct timepop_idle_witness_snapshot_t {
  bool supported = false;
  bool enabled = false;
  bool running = false;
  uint32_t shadow_dwt = 0;
  uint32_t enter_count = 0;
  uint32_t exit_count = 0;
  uint32_t pending_exit_count = 0;
  uint32_t last_enter_dwt = 0;
  uint32_t last_exit_dwt = 0;
  uint64_t total_cycles = 0;
  uint64_t snapshot_total_cycles = 0;
  uint32_t last_residency_cycles = 0;
  uint32_t snapshot_dwt = 0;
  uint64_t snapshot_wall_cycles = 0;
};

bool timepop_idle_witness_snapshot(timepop_idle_witness_snapshot_t* out);

// ============================================================================
// Retained Execution Trace
// ============================================================================
//
// Execution Trace is the scalar-only control-flow flight recorder shared by
// process_interrupt and process_timepop.  It records interrupt custody,
// priority-handoff/subscriber boundaries, TimePop callback selection and return,
// mutation barriers, and recurring rearm outcomes in one chronological ring.
//
// The live ring remains in fast ordinary RAM1.  The fault handler copies its
// committed entries into a retained RAM2 bank before reboot.  No callback or
// label string is dereferenced or copied: name_ptr remains only an address so
// corrupt metadata cannot make the recorder fault while describing the fault.

static constexpr uint32_t TIMEPOP_DISPATCH_TRACE_ENTRIES = 32U;
static constexpr uint32_t TIMEPOP_DISPATCH_TRACE_NO_SLOT = 0xFFFFFFFFUL;

enum class timepop_dispatch_trace_stage_t : uint32_t {
  NONE                      = 0,
  DISPATCH_ENTER            = 1,
  PHASE_ASAP                = 2,
  DEFERRED_SELECTED         = 3,
  CALLBACK_ENTER            = 4,
  CALLBACK_RETURN           = 5,
  DEFERRED_CLEANUP          = 6,
  MUTATION_BARRIER_ENTER    = 7,
  MUTATION_SELECTED         = 8,
  MUTATION_RESULT           = 9,
  MUTATION_BARRIER_EXIT     = 10,
  PHASE_TIMED               = 11,
  TIMED_SELECTED            = 12,
  TIMED_SLOT_AFTER_CALLBACK = 13,
  REARM_BEGIN               = 14,
  REARM_END                 = 15,
  SLOT_RETIRED              = 16,
  SLOT_REPLACED             = 17,
  PHASE_ALAP                = 18,
  DISPATCH_LEAVE            = 19,
  IRQ_SELECTED              = 20,

  ISR_ENTER                 = 32,
  ISR_CAPTURED              = 33,
  ISR_EXIT                  = 34,

  HANDOFF_ENTER             = 40,
  HANDOFF_DEQUEUE           = 41,
  HANDOFF_EXIT              = 42,

  SUBSCRIBER_SELECTED       = 48,
  SUBSCRIBER_ENTER          = 49,
  SUBSCRIBER_RETURN         = 50,
};

enum class timepop_dispatch_trace_kind_t : uint32_t {
  NONE      = 0,
  DISPATCH  = 1,
  ASAP      = 2,
  TIMED     = 3,
  ALAP      = 4,
  ISR_TIMED = 5,
  MUTATION  = 6,
  REARM     = 7,

  ISR_QTIMER1       = 16,
  ISR_VCLOCK        = 17,
  ISR_OCXO1         = 18,
  ISR_OCXO2         = 19,
  ISR_PPS           = 20,
  INTERRUPT_HANDOFF = 21,
  SUBSCRIBER_VCLOCK = 22,
  SUBSCRIBER_OCXO1  = 23,
  SUBSCRIBER_OCXO2  = 24,
};

enum class timepop_dispatch_trace_phase_t : uint32_t {
  IDLE               = 0,
  ASAP               = 1,
  TIMED              = 2,
  ALAP               = 3,
  APPLYING_MUTATIONS = 4,
};

struct timepop_dispatch_trace_entry_t {
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t stage;
  uint32_t phase;
  uint32_t kind;
  uint32_t slot_index;
  uint32_t handle;
  uint32_t callback;
  uint32_t slot_callback;
  uint32_t user_data;
  uint32_t name_ptr;
  uint32_t caller_sp;
  uint32_t site_pc;
  uint32_t dwt;
  uint32_t ipsr;
  uint32_t aux;
};

static_assert(sizeof(timepop_dispatch_trace_entry_t) == 64U,
              "TimePop dispatch trace entry must stay one cache-line pair");

struct timepop_dispatch_trace_bank_snapshot_t {
  bool valid;
  bool fault_captured;
  uint32_t count;
  uint32_t newest_sequence;
  uint32_t fault_dwt;
  uint32_t crash_sequence;
  uint32_t trace_sequence_at_capture;
  timepop_dispatch_trace_entry_t entries[TIMEPOP_DISPATCH_TRACE_ENTRIES];
};

struct timepop_dispatch_trace_snapshot_t {
  timepop_dispatch_trace_bank_snapshot_t live;
  timepop_dispatch_trace_bank_snapshot_t retained;
};

void timepop_dispatch_trace_snapshot(timepop_dispatch_trace_snapshot_t* out);
void timepop_dispatch_trace_clear_retained(void);

// Generic scalar recording surface used by process_interrupt.  Field mapping
// intentionally reuses the established TimePop entry ABI:
//
//   subject_index  -> slot_index
//   identity       -> handle
//   target         -> callback
//   related_target -> slot_callback
//   object         -> user_data
//   label_ptr      -> name_ptr
//
// Interpretation is determined by stage/kind.  caller_sp is captured by the
// macro at the caller's frame level; site_pc is captured inside the recorder.
void execution_trace_record(timepop_dispatch_trace_stage_t stage,
                            timepop_dispatch_trace_kind_t kind,
                            uint32_t subject_index,
                            uint32_t identity,
                            uint32_t target,
                            uint32_t related_target,
                            uint32_t object,
                            uint32_t label_ptr,
                            uint32_t aux,
                            uint32_t caller_sp);

extern "C" void execution_trace_capture_fault(uint32_t fault_dwt,
                                               uint32_t crash_sequence);

#if defined(__arm__)
#define ZPNET_EXECUTION_TRACE(stage, kind, subject_index, identity, target,    \
                              related_target, object, label_ptr, aux)         \
  do {                                                                        \
    uint32_t zpnet_execution_trace_sp_;                                       \
    __asm__ volatile("mov %0, sp" : "=r"(zpnet_execution_trace_sp_));         \
    execution_trace_record((stage), (kind), (subject_index), (identity),      \
                           (target), (related_target), (object), (label_ptr),  \
                           (aux), zpnet_execution_trace_sp_);                  \
  } while (0)
#else
#define ZPNET_EXECUTION_TRACE(stage, kind, subject_index, identity, target,    \
                              related_target, object, label_ptr, aux)         \
  execution_trace_record((stage), (kind), (subject_index), (identity),        \
                         (target), (related_target), (object), (label_ptr),    \
                         (aux), 0U)
#endif

typedef uint8_t timepop_priority_t;
static constexpr timepop_priority_t TIMEPOP_PRIORITY_FIRST   = 0U;
static constexpr timepop_priority_t TIMEPOP_PRIORITY_DEFAULT = 128U;
static constexpr timepop_priority_t TIMEPOP_PRIORITY_LAST    = 255U;

// Priority-capable overloads for timed TimePop slots. These are intentionally
// absent for ASAP/ALAP because those lanes are fixed deferred mailboxes, not
// scheduled CH2 deadline slots. Existing non-priority arms remain available
// through timepop.h and continue to use TIMEPOP_PRIORITY_DEFAULT.
timepop_handle_t timepop_arm_with_priority(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
);

timepop_handle_t timepop_arm_recurring_isr_with_priority(
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
);

timepop_handle_t timepop_arm_at_with_priority(
  int64_t             target_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
);

timepop_handle_t timepop_arm_from_anchor_with_priority(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
);

timepop_handle_t timepop_arm_ns_with_priority(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback,
  timepop_priority_t  priority
);

// Arm a critical recurring ISR slot on a fixed GNSS nanosecond grid:
//
//   target_k = base_gnss_ns + k * period_gnss_ns
//
// The first target is the first grid point strictly after current GNSS time.
// Later rearms remain on the original base grid; missed intervals are skipped
// rather than accumulated from the late service time.  Callback/rearm run in
// the CH2 scheduler pass, matching timepop_arm_recurring_isr().
timepop_handle_t timepop_arm_recurring_isr_from_base(
  int64_t             base_gnss_ns,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_recurring_isr_from_base_with_priority(
  int64_t             base_gnss_ns,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
);

// Arm a critical recurring ISR slot from an explicit GNSS/VCLOCK base pair.
//
//   target_k_counter32 = base_counter32 + k * (period_gnss_ns / 100)
//   target_k_gnss_ns   = base_gnss_ns   + k * period_gnss_ns
//
// This substrate-grade path does not require time_gnss_ns_now() or a valid
// time_anchor_snapshot() at arm time.  It is intended for bootstrap clients
// such as VCLOCK_CADENCE where the caller already owns the selected VCLOCK
// counter identity of the base edge.
timepop_handle_t timepop_arm_recurring_isr_from_base_counter32(
  int64_t             base_gnss_ns,
  uint32_t            base_counter32,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_recurring_isr_from_base_counter32_with_priority(
  int64_t             base_gnss_ns,
  uint32_t            base_counter32,
  uint64_t            period_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  timepop_priority_t  priority
);

// Notify TimePop that the VCLOCK synthetic coordinate system has been rebased.
// Existing timed deadlines are old-epoch coordinates and must be re-authored
// or cancelled before scheduling continues.
void timepop_epoch_changed(uint32_t epoch_sequence);


