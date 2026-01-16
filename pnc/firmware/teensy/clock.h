#pragma once

#include <stdint.h>

// ------------------------------------------------------------
// CLOCK SUBSYSTEM
//
// Unified 64-bit synthetic clocks for:
//   - DWT (CPU cycles, 600 MHz)
//   - GNSS (GPT2, 10 MHz GNSS VCLOCK)
//   - OCXO (GPT1, 10 MHz Kyocera REF)
//
// Public API exposes only time queries:
//   - dwt_now()
//   - gnss_now()
//   - ocxo_now()
//
// Each function auto-pumps on access and guarantees
// monotonically ascending values. Wrap-safe. Query-safe.
//
// Clients do not pump manually. Clocks may be zeroed
// externally if reset behavior is desired.
// ------------------------------------------------------------

// Initialize all clocks
void dwt_init();
//void gnss_init();
void ocxo_init();

// Query current synthetic time (auto-pumping)
uint64_t dwt_now();
uint64_t gnss_now();
uint64_t ocxo_now();

// Optional: zero accumulated clocks
void dwt_zero();
void gnss_zero();
void ocxo_zero();
