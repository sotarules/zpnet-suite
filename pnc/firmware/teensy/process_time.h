#pragma once

#include "process.h"

/**
 * ------------------------------------------------------------------
 * TIME Process (process_time)
 *
 * Role:
 *   • Greenfield experimental subsystem for next-generation
 *     ZPNet time / clock architecture.
 *
 *   • Provides a controlled environment to instantiate and reason
 *     about AbstractClock-based clocks (DWT, GNSS, OCXO, RTC, etc.)
 *     without impacting the existing CLOCKS subsystem.
 *
 * Philosophy:
 *   • TIME does nothing unless explicitly instructed.
 *   • No implicit hardware ownership.
 *   • No background ticking until START.
 *   • All state transitions are command-driven and PPS-aligned.
 *
 * Lifecycle:
 *   • start (campaign)
 *       - Arms clocks and prepares for PPS-aligned anchoring
 *
 *   • stop
 *       - Disarms clocks, freezes synthetic time
 *
 *   • clear
 *       - Clears synthetic nanoseconds (no PPS wait)
 *
 *   • recover (campaign)
 *       - Rebinds baselines at next PPS without clearing nanoseconds
 *
 * Commands (initial, skeletal):
 *   • START   { campaign: string }
 *   • STOP
 *   • CLEAR
 *   • RECOVER { campaign: string }
 *   • REPORT
 *
 * Notes:
 *   • This subsystem is intentionally incomplete.
 *   • Behavior will evolve as clock abstractions solidify.
 *
 * ------------------------------------------------------------------
 */

// Explicit initialization (no side effects)
void process_time_init(void);

// Register TIME process with the process registry
void process_time_register(void);
