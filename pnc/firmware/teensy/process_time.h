#pragma once

#include "time.h"

// ============================================================================
// process_time.h — process wrapper for the time subsystem
// ============================================================================
//
// time.h remains the stable functional interface used by ordinary code.
// process_time adds lifecycle and command/report participation.
// ============================================================================

void process_time_init(void);
void process_time_register(void);
