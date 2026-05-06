#pragma once

#include "time.h"

// ============================================================================
// process_time.h — legacy process wrapper for the time subsystem
// ============================================================================
//
// time.h remains the functional interface.  process_time currently preserves
// the legacy lifecycle and command registration surface while the timing system
// migrates toward CLOCKS-owned clock truth and stateless TIME projection.
//
// First-pass reduction keeps the exported process_time_init() and
// process_time_register() entry points, but the command/report payloads are now
// intentionally compact.  Heavy diagnostic/history reports are retired here and
// should move to their rightful subsystem owners before process_time disappears.
// ============================================================================

void process_time_init(void);
void process_time_register(void);
