#pragma once

// ============================================================================
// TimePop v5.0 — Process Interface
// ============================================================================
//
// System initialization and process registration only.
// General consumers include timepop.h, not this file.
//
// Commands:
//   REPORT — active timer diagnostics (includes per-slot predicted_dwt)
//   DIAG   — DWT prediction + GNSS-now Welford statistics
//   TEST   — timer accuracy verification { "ns": <uint64> }
//   NS_TEST — nano-precise timer accuracy test
//   INTERP_TEST — dual-path validation (legacy, pending removal)
//
// v5.0: Continuous DWT prediction and GNSS-now validation.
//
//   Every standard timer fire produces two measurements:
//     1. DWT prediction residual: actual DWT_CYCCNT vs predicted
//     2. GNSS-now residual: time_dwt_to_gnss_ns(captured_dwt) vs
//        the GNSS nanosecond derived from the VCLOCK position
//
//   Welford accumulators maintain running statistics across all
//   timer fires.  DIAG dumps the full statistical state.
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

//   VCLOCK_TEST — QTimer1 interval measurement { "ns": <uint64> }
//                 arm_ns range: 1,000,000–999,000,000
//                 waits for PPS edge before arming, measures VCLOCK delta