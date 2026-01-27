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
 *   • start
 *       - Announces availability via CLOCKS_INIT_ENTER event
 *       - No hardware ownership or mutation
 *
 *   • stop
 *       - Emits CLOCKS_STOP event
 *       - No hardware side effects
 *
 * Commands:
 *   • REPORT
 *       - Returns a structured Payload containing:
 *           • dwt_cycles
 *           • gnss_10khz_ticks
 *           • ocxo_10khz_ticks
 *           • dwt_ns
 *           • gnss_ns
 *           • ocxo_ns
 *
 *   • CLEAR
 *       - Zeros all synthetic clocks
 *       - Emits CLOCKS_CLEAR event
 *       - Returns no command payload (side-effect only)
 *
 * Command Contract:
 *   • All command handlers return `const Payload*`
 *   • Returning nullptr indicates "no payload"
 *   • Serialization and response envelope construction
 *     are handled by the process framework
 *
 * ------------------------------------------------------------------
 */

// Register CLOCKS process with the process registry
void process_clocks_register(void);
