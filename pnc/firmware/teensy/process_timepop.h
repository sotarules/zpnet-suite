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
//   • the IRQ_QTIMER1 vector itself (single shared vector for QTimer1)
//
// TimePop does not own:
//   • PPS/GPIO interrupt custody
//   • OCXO interrupt custody
//   • raw interrupt normalization
//   • QTimer1 CH3 (hosted client; see below)
//
// Those are owned by process_interrupt.
//
// QTimer1 CH3 hosted-client API:
//
//   QTimer1 has only one IRQ vector shared across all four channels.
//   process_interrupt owns the VCLOCK rolling integrator, which needs a
//   dedicated channel compare on QTimer1 (so the integrator's tick has
//   the same hardware-to-software latency profile as the OCXO ticks on
//   QTimer3).  Rather than fight TimePop for the vector, process_interrupt
//   registers a CH3 ISR with TimePop.  TimePop's qtimer1_irq_isr captures
//   DWT as the first instruction and then dispatches:
//     • CH2 flag set → TimePop's own scheduler ISR
//     • CH3 flag set → the registered process_interrupt ISR
//   Both handlers receive the same first-instruction DWT value, so the
//   CH3 client gets identical latency to a dedicated-vector handler.
//
//   process_interrupt is responsible for:
//     • initializing CH3 hardware (CTRL/SCTRL/CSCTRL/COMP1/CMPLD1)
//     • programming and re-programming the CH3 compare target
//     • clearing the CH3 TCF1 flag in its handler
//
//   TimePop is responsible only for:
//     • dispatching to the registered CH3 handler when the flag is set
//
// ============================================================================

#pragma once

#include "timepop.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);

// QTimer1 CH3 hosted-client registration.  See doctrine above.
//
// The callback runs in IRQ context.  dwt_isr_entry_raw is the DWT cycle
// count captured as the first instruction of TimePop's qtimer1_irq_isr —
// the same value as if the callback owned the IRQ vector itself.
//
// Pass nullptr to unregister.  Only one CH3 callback may be registered;
// subsequent calls replace the previous registration.
//
typedef void (*timepop_qtimer1_ch3_isr_fn)(uint32_t dwt_isr_entry_raw);

void timepop_register_qtimer1_ch3_isr(timepop_qtimer1_ch3_isr_fn cb);