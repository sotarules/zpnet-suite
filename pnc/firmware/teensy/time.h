#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.h — Universal GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// Provides a single function: time_gnss_ns_now()
//
// Returns the current GNSS nanosecond — a monotonic, PPS-counted
// value with DWT interpolation between PPS edges.  Available from
// boot as soon as the second PPS edge establishes calibration.
// No campaign dependency.  No external state.  ISR-safe.
//
// The returned value is a LOCAL temporal coordinate, not a wall clock.
// It counts nanoseconds from the first PPS edge (time zero).
// Epoch alignment to UTC is the Pi's responsibility.
//
// Architecture:
//
//   process_clocks.cpp calls time_pps_update() once per PPS edge,
//   providing:
//     - dwt_at_pps:        DWT_CYCCNT captured at the PPS edge
//     - dwt_cycles_per_s:  measured DWT cycles in the prior second
//
//   time.cpp maintains a seqlock-protected anchor and a PPS counter.
//   time_gnss_ns_now() reads DWT_CYCCNT, computes elapsed cycles
//   since the last PPS, converts to nanoseconds using the measured
//   ratio, and adds to the accumulated PPS-second total.
//
// Accuracy:
//
//   ~5-7 ns (prediction tracker stddev at 1.008 GHz).
//   Resolution: ~1 ns (single DWT cycle).
//   Staleness guard: returns -1 if anchor is older than 3 seconds.
//
// Concurrency:
//
//   Seqlock pattern — safe to call from any context including ISRs.
//   The PPS callback is the sole writer.  All other callers are readers.
//
// ============================================================================

// ============================================================================
// Core API
// ============================================================================

/// Returns the current GNSS nanosecond (monotonic, PPS-counted, DWT-interpolated).
///
/// Returns -1 if:
///   - No calibration yet (fewer than 2 PPS edges observed)
///   - Anchor is stale (>3 seconds since last PPS update)
///
/// ISR-safe.  Zero allocation.  Zero blocking.  Pure register reads + arithmetic.
int64_t time_gnss_ns_now(void);

/// Returns the number of completed PPS seconds since time zero.
/// Returns 0 if not yet valid.
uint32_t time_pps_count(void);

/// Returns true if time_gnss_ns_now() is ready to return valid values.
bool time_valid(void);

// ============================================================================
// Update interface (called by process_clocks.cpp PPS callback ONLY)
// ============================================================================

/// Feed a new PPS anchor to the time module.
///
/// Called once per PPS edge from the deferred PPS callback in
/// process_clocks.cpp, after the continuous calibration block
/// has updated dwt_cycles_per_gnss_s.
///
/// On the first call: establishes time zero (pps_count = 0, anchor set).
/// On the second call: calibration becomes valid (pps_count = 1).
/// Subsequent calls: pps_count increments, anchor advances.
///
/// dwt_at_pps:       DWT_CYCCNT at the PPS edge (ISR-corrected).
/// dwt_cycles_per_s: DWT cycles measured in the prior GNSS second.
///                   Zero on the first call (no prior second yet).
///                   Must be nonzero from the second call onward.
void time_pps_update(uint32_t dwt_at_pps, uint32_t dwt_cycles_per_s);

// ============================================================================
// Initialization
// ============================================================================

/// Zero all state.  Called once at startup before PPS ISR is enabled.
void time_init(void);