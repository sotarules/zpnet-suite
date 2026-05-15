#pragma once

#include "time.h"

// ============================================================================
// process_time.h -- legacy TIME process wrapper
// ============================================================================
//
// time.h remains the functional interface.
//
// process_time is now intentionally narrow.  It exists only to keep the legacy
// TIME process registered while it owns the PPS/VCLOCK compatibility anchor and
// per-clock projection state required by TimePop/CLOCKS during the retirement
// window.
//
// Prediction history and dynamic-CPS behavior are retired.
// ============================================================================

void process_time_init(void);
void process_time_register(void);
