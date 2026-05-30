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
//   • critical recurring ISR clients that callback/rearm before schedule_next
//   • fixed non-slot deferred callback dispatch
//   • instrumentation / reports
//   • scheduler policy for QTimer1 CH2 compare deadlines
//
// TimePop does not own:
//   • PPS/GPIO interrupt custody
//   • OCXO interrupt custody
//   • QTimer1 vector custody — process_interrupt owns IRQ_QTIMER1 and
//     dispatches CH2 to TimePop's registered handler in IRQ context
//   • QTimer1 CH0/CH2 hardware mode init — process_interrupt does
//     the one-time CTRL/SCTRL/CSCTRL/COMP1/CMPLD1 setup
//
// Those are owned by process_interrupt.
//
// QTimer1 CH2 hosted-handler API (TimePop is the hosted client):
//
//   QTimer1 has only one IRQ vector shared across all four channels.
//   process_interrupt owns the vector and dispatches CH2 in IRQ context to
//   TimePop's registered handler.  VCLOCK cadence is no longer a separate
//   QTimer1 compare path; it is a TimePop critical recurring ISR client.
//
//   The handler receives a normalized DWT-at-edge capture for the CH2 event.
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
//     • registering its CH2 handler at init via
//       interrupt_register_qtimer1_ch2_handler()
//     • requesting CH2 compare target updates from process_interrupt as
//       slots are scheduled
//
//   process_interrupt is responsible for:
//     • initializing CH0/CH2 hardware (one-time mode/control)
//     • clearing the CH2 TCF1 flag in the QTimer1 ISR before
//       invoking TimePop's handler
//     • building the event/diag payloads passed to TimePop
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
//   implemented as fixed deferred mailboxes rather than scheduled slots.  They
//   are safe to request from ISR context: the request only writes the deferred
//   lane under PRIMASK and sets the dispatch-pending bit.  ASAP drains before
//   timed scheduled-context slots; ALAP drains after timed slots.
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
//   Critical recurring ISR clients are the white-glove exception for tiny
//   substrate-maintenance work.  They are still ordinary TimePop slots and
//   share CH2 fire facts, but their callbacks and recurring rearm happen
//   inside the CH2 IRQ pass before schedule_next() selects the next compare.
//
//   Timed slots may also carry a service priority. Lower numeric priority runs
//   first when multiple timed slots share one exact CH2 fire fact. Priority
//   never changes compare scheduling and never applies to ASAP/ALAP. Existing
//   callers use TIMEPOP_PRIORITY_DEFAULT and retain stable slot-index order.
//
// ============================================================================

#pragma once

#include "timepop.h"
#include "process_interrupt.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);

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
// the CH2 IRQ pass, matching timepop_arm_recurring_isr().
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

// Diagnostic command surface:
//
//   TIMEPOP.PHASE_PROBE action=START phase_us=250 period_us=1000 samples=0
//   TIMEPOP.PHASE_PROBE
//   TIMEPOP.PHASE_PROBE action=STOP
//   TIMEPOP.PHASE_PROBE action=RESET
//
// PHASE_PROBE is a report-only quiet-zone instrument.  It arms a tiny critical
// recurring ISR slot at a fixed phase within the VCLOCK period and records
// scalar statistics about co-expired timed slots, deferred backlog, exact-fire
// error, and DWT prediction error.  It does not alter scheduler policy or
// timing authority.

// QTimer1 CH2 IRQ-context handler.  Registered with process_interrupt
// at init.  Called by process_interrupt's qtimer1_isr dispatcher on
// every CH2 compare-match, in IRQ context, with the standard event
// and diag payloads filled in by the dispatcher.
//
void timepop_qtimer1_ch2_handler(const interrupt_event_t& event,
                                 const interrupt_capture_diag_t& diag);