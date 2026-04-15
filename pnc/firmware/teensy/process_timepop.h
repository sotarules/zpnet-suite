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
//
// TimePop does not own:
//   • PPS/GPIO interrupt custody
//   • OCXO interrupt custody
//   • raw interrupt normalization
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
