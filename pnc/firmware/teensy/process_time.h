#pragma once

#include "time.h"

// ============================================================================
// process_time.h — legacy process wrapper for static TIME projection
// ============================================================================
//
// time.h remains the functional interface.
//
// This wrapper still exports process_time_init() and process_time_register() so
// existing startup code and command routing survive while process_time is
// retired incrementally.  The TIME process still owns the compatibility anchor
// and lane projection state, but the local prediction history and dynamic-CPS
// command surfaces are gone.
// ============================================================================

void process_time_init(void);
void process_time_register(void);
