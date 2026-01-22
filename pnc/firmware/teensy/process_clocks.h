#pragma once

#include "process.h"

/**
 * ------------------------------------------------------------------
 * CLOCKS Process (process_clocks)
 *
 * Role:
 *   • Exposes the ZPNet synthetic clock substrate as an observable
 *     system service.
 *
 *   • Provides direct inspection and controlled reset of:
 *       - DWT synthetic clock   (CPU cycles)
 *       - GNSS synthetic clock  (GPT2 / VCLOCK)
 *       - OCXO synthetic clock  (GPT1 / REF)
 *
 * Philosophy:
 *   • Clocks are system infrastructure, not experiment-owned.
 *   • Time advances lazily and is reconciled on access.
 *   • No prescalers, no ISRs, no background semantics beyond
 *     rollover protection handled in clock_init().
 *
 * Lifecycle:
 *   • start: announces availability (no hardware ownership)
 *   • stop: no side effects
 *
 * Commands:
 *   • REPORT
 *       - Returns raw 64-bit synthetic counters:
 *           • dwt_cycles
 *           • gnss_ticks
 *           • ocxo_ticks
 *
 *   • CLEAR
 *       - Zeros all synthetic clocks
 *       - Intended for controlled test resets only
 *
 * ------------------------------------------------------------------
 */

// Register CLOCKS process with the process registry
void process_clocks_register(void);
