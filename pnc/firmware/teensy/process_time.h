#pragma once

#include "time.h"

// ============================================================================
// process_time.h — process wrapper for the time subsystem
// ============================================================================
//
// time.h remains the stable functional interface used by ordinary code.
// process_time adds lifecycle and command/report participation.  It owns
// PPS/VCLOCK anchor state, DWT next-second prediction, and dynamic CPS
// intra-second refinement. Dynamic CPS refinement is driven by a regular
// phase-locked TimePop client, not by a private QTimer compare rail.
// ============================================================================

void process_time_init(void);
void process_time_register(void);
