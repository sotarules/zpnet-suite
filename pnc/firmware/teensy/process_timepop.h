#pragma once

// ============================================================================
// TimePop v2 — Process Interface
// ============================================================================
//
// System initialization and process registration only.
// General consumers include timepop.h, not this file.
//
// Commands:
//   REPORT — active timer diagnostics
//   TEST   — timer accuracy verification { "ns": <uint64> }
//
// ============================================================================

#include "timepop.h"
#include <stdint.h>

/// Initialize TimePop hardware (GPT2 output compare).
/// GPT2 must already be running (process_clocks_init_hardware).
/// Must be called once during setup().
void timepop_init(void);

/// Register the TIMEPOP process with the command framework.
void process_timepop_register(void);