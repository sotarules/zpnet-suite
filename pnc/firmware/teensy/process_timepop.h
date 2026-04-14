// ============================================================================
// process_timepop.h
// ============================================================================
//
// TimePop process interface.
//
// TimePop owns:
//   • priority-queue slot scheduling
//   • queue prediction / nearest-deadline selection
//   • deferred callback dispatch
//   • instrumentation / reports
//   • opt-in internal VCLOCK edge characterization
//
// TimePop does not own:
//   • QTIMER1 vector installation
//   • QTIMER1 CH2 register bring-up
//   • CH2 compare register twiddling
//   • last-mile DWT spin
//   • interrupt-latency canonicalization
//
// Those are owned by process_interrupt.
//
// ============================================================================

#pragma once

#include "timepop.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);
