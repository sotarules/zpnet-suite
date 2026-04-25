// ============================================================================
// process_timepop.h
// ============================================================================
//
// TimePop process interface.
//
// TimePop is a foreground scheduler driven by process_interrupt's TICK
// subscription (1 kHz cadence).  It owns no hardware.  On every TICK:
//
//   1. ASAP queue is drained (callbacks fire in foreground).
//   2. Timed slots are scanned; any whose deadline ≤ tick.counter32
//      have their callbacks fired in foreground.
//   3. ALAP queue is drained.
//
// All timed deadlines are stored in synthetic VCLOCK counter32 ticks.
// The synthetic counter32 is published into each TICK event by
// process_interrupt; TimePop never reads counter hardware.
//
// Scheduling resolution is 1 ms (the TICK interval).
//
// ============================================================================

#pragma once

#include "timepop.h"
#include "process_interrupt.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);