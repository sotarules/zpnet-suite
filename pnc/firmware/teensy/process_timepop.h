#pragma once

// ============================================================================
// TimePop v8.0 — Process Interface
// ============================================================================
//
// System initialization and process registration only.
// General consumers include timepop.h, not this file.
//
// Commands:
//   REPORT     — active timer diagnostics (per-slot predicted_dwt, scheduler state)
//   DIAG       — DWT prediction + GNSS-now Welford statistics
//   DIAG_RESET — reset Welford accumulators
//   TEST       — timer accuracy verification { "ns": <uint64> }
//   NS_TEST    — nano-precise timer accuracy test
//   INTERP_TEST — dual-path interpolation validation
//
// v8.0: Priority queue with single hardware comparator, 10 MHz.
//
//   QTimer1 channel map:
//     CH0 + CH1 — passive 32-bit cascaded counter (10 MHz, CM=1)
//     CH2       — dynamic compare (priority queue scheduler, CM=1)
//     CH3       — unallocated (available for future use)
//
//   All timers are nano-precise.  The ISR DWT-spins to the predicted
//   cycle count (≤100 ns) and captures fire state.  Callbacks execute
//   in scheduled context via timepop_dispatch().
//
//   A 1 ms ceiling heartbeat ensures housekeeping even when no
//   deadlines are imminent.  Phantom firings from 16-bit aliasing
//   are detected and dismissed via 32-bit qualification.
//
// ============================================================================

#include "timepop.h"
#include <stdint.h>

/// Initialize TimePop hardware (QTimer1 CH2 dynamic compare scheduler).
/// QTimer1 CH0+CH1 must already be running (process_clocks_init_hardware).
/// Must be called once during setup().
void timepop_init(void);

/// Register the TIMEPOP process with the command framework.
void process_timepop_register(void);