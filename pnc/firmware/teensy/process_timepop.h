#pragma once

// ============================================================================
// process_timepop.h
// ============================================================================
//
// TimePop process interface.
//
// TimePop now owns:
//   • priority-queue slot scheduling
//   • queue prediction / nearest-deadline selection
//   • deferred callback dispatch
//   • instrumentation / reports
//
// TimePop no longer owns:
//   • QTIMER1 vector installation
//   • QTIMER1 CH2 register bring-up
//   • CH2 compare register twiddling
//   • last-mile DWT spin
//
// Those are now owned by process_interrupt.
//
// ============================================================================

#include "timepop.h"
#include <stdint.h>

void timepop_init(void);
void process_timepop_register(void);