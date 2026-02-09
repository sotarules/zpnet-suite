#pragma once

#include <stdint.h>
#include "process.h"

// ============================================================================
// CLOCKS — Authoritative Temporal Subsystem (Teensy)
// ============================================================================
//
// CLOCKS owns ALL notions of time on the Teensy.
//
// Responsibilities:
//   • Hardware clock capture (DWT, GNSS, OCXO)
//   • Synthetic nanosecond clocks
//   • PPS-aligned truth capture
//   • 1 Hz publication of canonical clock tuple
//
// Philosophy:
//   • CLOCKS is the single source of truth
//   • Other subsystems consume published facts
//   • Direct access is permitted but discouraged
//
// ============================================================================

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

// Initialize hardware clocks and internal ledgers
void process_clocks_init(void);

// Register CLOCKS process command surface
void process_clocks_register(void);

// -----------------------------------------------------------------------------
// Optional direct accessors (escape hatches)
// -----------------------------------------------------------------------------

uint64_t clocks_dwt_cycles_now(void);
uint64_t clocks_dwt_ns_now(void);

uint64_t clocks_gnss_ns_now(void);
uint64_t clocks_ocxo_ns_now(void);

uint64_t clocks_gnss_zero_ns(void);

