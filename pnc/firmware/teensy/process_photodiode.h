#pragma once

#include "process.h"

// ============================================================================
// PHOTODIODE compatibility process
// ============================================================================
//
// Temporary compatibility surface while PHOTONS supersedes the legacy
// PHOTODIODE subsystem.
//
// process_interrupt owns the PD200T comparator edge and all timing evidence.
// This process does not attach an ISR, does not count optical episodes, and
// does not clear interrupt-owned evidence.
//
// Retained commands:
//   INIT   — configure the PD200T GPIO/MON pins for passive observation only
//   REPORT — report ambient pin level, PD200T MON ADC, and process_interrupt
//            PHOTODIODE diagnostics
//   COUNT  — compatibility alias for process_interrupt PHOTODIODE IRQ count
//   CLEAR  — compatibility no-op
//
// This file may be removed once callers migrate to PHOTONS / INTERRUPT.
// ============================================================================

void process_photodiode_init(void);
void process_photodiode_register(void);
