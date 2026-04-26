// ============================================================================
// process_timepop.h
// ============================================================================
//
// TimePop process interface.
//
// TimePop is a foreground scheduler whose timed slots are fired by
// process_interrupt's hardware-driven schedule-fire mechanism (QTimer1
// CH2 compare).  TimePop owns no hardware; all hardware programming
// goes through the public interrupt_schedule_*() API.
//
// Two delivery contexts:
//
//   • FOREGROUND:  user callback runs in loop() context.  The CH2 ISR
//     queues an asap slot; foreground drains it via timepop_dispatch().
//
//   • SPINDRY:     user callback runs in CH2 ISR context, after a tight
//     DWT spin from the approach ISR fire to the exact deadline.  See
//     timepop.h for the contract and diagnostic semantics.
//
// TimePop additionally subscribes to TICK (1 kHz) for two responsibilities:
//
//   1. Pre-bridge anchoring — slots armed via timepop_arm() before the
//      first PPS edge can't yet compute counter32 deadlines (the bridge
//      isn't up).  These are stashed with `pending_anchor=true` and a
//      relative delay_ticks; the first TICK after the bridge comes up
//      anchors them to a real counter32.
//
//   2. ASAP/ALAP heartbeat — although timepop_dispatch() in loop() is
//      the primary asap/alap drain, TICK provides a guaranteed periodic
//      drain at 1 kHz for paths that don't rely on loop() turnaround.
//
// All timed deadlines are stored in synthetic VCLOCK counter32 ticks.
// The synthetic counter32 is published into events by process_interrupt;
// TimePop never reads counter hardware.
//
// Scheduling resolution:
//   • Foreground slots: bounded above by foreground loop turnaround
//     (~tens of microseconds in normal operation, up to milliseconds if
//     loop() is busy).
//   • SpinDry slots: sub-microsecond (target ≤ 50 ns of jitter under
//     normal operation; subject to PPS preemption — see diag fields).
//
// ============================================================================

#pragma once

#include "timepop.h"
#include "process_interrupt.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);