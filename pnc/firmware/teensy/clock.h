// =============================================================
// FILE: clock.h
// =============================================================
//
// ZPNet Clock Subsystem — Public Interface
//
// Exposes:
//   • Raw monotonic ledgers
//   • Prescaled (10 kHz) hardware-derived tick counts
//   • Synthetic nanosecond clocks (integer math, reconciled)
//   • Explicit zeroing
//
// IMPORTANT SEMANTICS:
//   • GNSS / OCXO tick domains are PRESCALED TO 10 kHz
//     (100 microseconds per tick — NOT 10 MHz)
//   • Nanosecond clocks are SYNTHETIC but PHYSICALLY ANCHORED
//
// Author: The Mule + GPT
//

#pragma once

#include <stdint.h>

// --------------------------------------------------------------
// Initialization
// --------------------------------------------------------------

/**
 * Initialize the clock subsystem.
 *
 * Arms GPT hardware, enables prescaling, and starts
 * monotonic ledger tracking.
 *
 * Must be called once during system startup.
 */
void clock_init(void);

// --------------------------------------------------------------
// Raw ledgers (authoritative, monotonic)
// --------------------------------------------------------------

/**
 * Return the current 64-bit DWT cycle count (wrap-safe).
 *
 * Units:
 *   • cycles at ~600 MHz
 *
 * This is the highest-resolution monotonic counter.
 */
uint64_t clock_dwt_cycles_now(void);

/**
 * Return GNSS-derived prescaled tick count.
 *
 * Units:
 *   • 10 kHz ticks (100 microseconds per tick)
 *
 * NOT 10 MHz.
 */
uint64_t clock_gnss_10khz_ticks(void);

/**
 * Return OCXO-derived prescaled tick count.
 *
 * Units:
 *   • 10 kHz ticks (100 microseconds per tick)
 *
 * NOT 10 MHz.
 */
uint64_t clock_ocxo_10khz_ticks(void);

// --------------------------------------------------------------
// Synthetic nanosecond clocks
// --------------------------------------------------------------

/**
 * Return synthetic nanoseconds derived from DWT cycles.
 *
 * Units:
 *   • nanoseconds since last zero
 *
 * Resolution:
 *   • ~1.67 ns (5/3 ns per cycle)
 *
 * Monotonic, not externally anchored.
 */
uint64_t clock_dwt_ns_now(void);

/**
 * Return synthetic nanoseconds anchored to GNSS pulses.
 *
 * Units:
 *   • nanoseconds since last zero
 *
 * Behavior:
 *   • Reconciled on every 10 kHz GNSS-derived edge
 *   • Linearly interpolated between edges using DWT
 *   • Error bounded to < 100 microseconds
 */
uint64_t clock_gnss_ns_now(void);

/**
 * Return synthetic nanoseconds anchored to OCXO pulses.
 *
 * Units:
 *   • nanoseconds since last zero
 *
 * Behavior:
 *   • Reconciled on every 10 kHz OCXO-derived edge
 *   • Linearly interpolated between edges using DWT
 *   • Error bounded to < 100 microseconds
 */
uint64_t clock_ocxo_ns_now(void);

// --------------------------------------------------------------
// Control
// --------------------------------------------------------------

/**
 * Zero all clock ledgers.
 *
 * Effects:
 *   • Resets DWT cycle ledger
 *   • Resets GNSS and OCXO 10 kHz tick counts
 *   • Resets nanosecond epochs
 *
 * Safe to call when clocks are running.
 */
void clock_zero_all(void);

// --------------------------------------------------------------
// Zero-time introspection
// --------------------------------------------------------------

/**
 * Return GNSS nanoseconds at last zeroing.
 *
 * Used to compute elapsed wall time since clocks were cleared.
 */
uint64_t clock_gnss_zero_ns(void);
