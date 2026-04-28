// ============================================================================
// process_timepop.h
// ============================================================================
//
// TimePop process interface.
//
// TimePop owns:
//   • priority-queue slot scheduling
//   • absolute recurring series
//   • Spin-Dry early-wake / landing for VCLOCK scheduling
//   • deferred callback dispatch
//   • instrumentation / reports
//   • always-on internal VCLOCK monitor
//   • scheduler policy for QTimer1 CH2 compare deadlines
//
// TimePop does not own:
//   • PPS/GPIO interrupt custody
//   • OCXO interrupt custody
//   • QTimer1 vector custody — process_interrupt owns IRQ_QTIMER1 and
//     dispatches CH2 to TimePop's registered handler in IRQ context
//   • QTimer1 CH0/CH1/CH2 hardware mode init — process_interrupt does
//     the one-time CTRL/SCTRL/CSCTRL/COMP1/CMPLD1 setup
//
// Those are owned by process_interrupt.
//
// QTimer1 CH2 hosted-handler API (TimePop is the hosted client):
//
//   QTimer1 has only one IRQ vector shared across all four channels.
//   process_interrupt owns the vector and dispatches in IRQ context:
//     • CH2 flag set → TimePop's registered handler (this file)
//     • CH3 flag set → process_interrupt's internal vclock_cadence_isr
//   Each handler receives a normalized DWT-at-edge capture for its own
//   QTimer event.
//
//   For CH2, the dispatcher additionally authors synthetic
//   counter32_at_event and gnss_ns_at_event and packages everything into a standard
//   interrupt_event_t (kind = TIMEPOP) and interrupt_capture_diag_t,
//   which it passes to TimePop's handler.  TimePop reads the event
//   payload directly and proceeds with slot dispatch — no need to
//   re-read the counter or recompute gnss_ns.
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
//     • initializing CH0/CH1/CH2 hardware (one-time mode/control)
//     • clearing the CH2 TCF1 flag in the QTimer1 ISR before
//       invoking TimePop's handler
//     • building the event/diag payloads passed to TimePop
//
// ============================================================================

#pragma once

#include "timepop.h"
#include "process_interrupt.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);

// QTimer1 CH2 IRQ-context handler.  Registered with process_interrupt
// at init.  Called by process_interrupt's qtimer1_isr dispatcher on
// every CH2 compare-match, in IRQ context, with the standard event
// and diag payloads filled in by the dispatcher.
//
void timepop_qtimer1_ch2_handler(const interrupt_event_t& event,
                                 const interrupt_capture_diag_t& diag);