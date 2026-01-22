// =============================================================
// FILE: clock.h
// =============================================================
#pragma once

#include <stdint.h>

// ------------------------------------------------------------
// CLOCK SUBSYSTEM — Lazy Reconciliation + Global Guard Tick
//
// Unified 64-bit synthetic clocks for:
//   • DWT  (CPU cycles, ~600 MHz, 32-bit hardware counter)
//   • GNSS (GPT2, 10 MHz GNSS VCLOCK, 32-bit hardware counter)
//   • OCXO (GPT1, 10 MHz Kyocera REF, 32-bit hardware counter)
//
// Philosophy:
//   • Each *_now() reconciles its 32-bit counter into a 64-bit accumulator.
//   • Wrap-safe deltas (unsigned subtraction).
//   • Clocks are "correct on demand".
//   • A single global guard tick prevents rollover loss even if
//     no code queries clocks for a while.
//
// No per-process ownership.
// No prescalers.
// No clock-specific ISRs.
// ------------------------------------------------------------

// Initialize the clock subsystem (must be called from setup()).
// Enables DWT and seeds GPT domains, then starts rollover guard tick.
void clock_init(void);

// Query current synthetic time (monotonic 64-bit)
uint64_t dwt_now(void);
uint64_t gnss_now(void);
uint64_t ocxo_now(void);

// Optional: zero accumulated clocks (resets accumulator + reseeds last)
void dwt_zero(void);
void gnss_zero(void);
void ocxo_zero(void);
