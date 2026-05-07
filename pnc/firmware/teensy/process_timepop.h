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
//   • deferred callback dispatch
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
//   guarantee because they are deferred scheduled-context queues, not timed
//   CH2 clients.
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
//   TimePop does not defensively second-guess exact compare fires.  If the
//   foreground scheduler ever has to discover an already-past timed slot,
//   that condition is surfaced through report counters rather than hidden as
//   a normal precision event.
//
//   Critical recurring ISR clients are the white-glove exception for tiny
//   substrate-maintenance work.  They are still ordinary TimePop slots and
//   share CH2 fire facts, but their callbacks and recurring rearm happen
//   inside the CH2 IRQ pass before schedule_next() selects the next compare.
//
// ============================================================================

#pragma once

#include "timepop.h"
#include "process_interrupt.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);

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

// Notify TimePop that the VCLOCK synthetic coordinate system has been rebased.
// Existing timed deadlines are old-epoch coordinates and must be re-authored
// or cancelled before scheduling continues.
void timepop_epoch_changed(uint32_t epoch_sequence);

// QTimer1 CH2 IRQ-context handler.  Registered with process_interrupt
// at init.  Called by process_interrupt's qtimer1_isr dispatcher on
// every CH2 compare-match, in IRQ context, with the standard event
// and diag payloads filled in by the dispatcher.
//
void timepop_qtimer1_ch2_handler(const interrupt_event_t& event,
                                 const interrupt_capture_diag_t& diag);