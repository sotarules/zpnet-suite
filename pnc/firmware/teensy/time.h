#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.h — Universal GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// Core functions:
//
//   time_gnss_ns_now()       — current GNSS nanosecond (DWT interpolated)
//   time_dwt_to_gnss_ns()    — convert a DWT_CYCCNT value to GNSS nanoseconds
//   time_gnss_ns_to_dwt()    — convert a GNSS nanosecond to a DWT_CYCCNT value
//   time_anchor_snapshot()   — seqlock-safe copy of the PPS anchor state
//
// Returns monotonic, PPS-counted nanoseconds with DWT interpolation
// between PPS edges.  Available from boot as soon as the second PPS
// edge establishes calibration.  No campaign dependency.  ISR-safe.
//
// The returned value is a LOCAL temporal coordinate, not a wall clock.
// It counts nanoseconds from the first PPS edge (time zero).
// Epoch alignment to UTC is the Pi's responsibility.
//
// Architecture:
//
//   process_clocks_alpha.cpp calls time_pps_update() once per PPS edge,
//   providing:
//     - dwt_at_pps:        DWT_CYCCNT captured at the PPS edge
//     - dwt_cycles_per_s:  measured DWT cycles in the prior second
//     - gpt2_at_pps:       GPT2_CNT captured at the PPS edge
//
//   time.cpp maintains a seqlock-protected anchor and a PPS counter.
//   All conversions use the same anchor snapshot for consistency.
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
// Anchor snapshot — public, seqlock-safe copy of the PPS anchor
// ============================================================================

struct time_anchor_snapshot_t {
  uint32_t dwt_at_pps;       // DWT_CYCCNT at the PPS edge
  uint32_t dwt_cycles_per_s; // DWT cycles in the prior GNSS second
  uint32_t gpt2_at_pps;      // GPT2_CNT at the PPS edge (VCLOCK position)
  uint32_t pps_count;        // PPS edges seen (1-indexed)
  bool     valid;            // true after second PPS (rate available)
  bool     ok;               // true if snapshot was consistent (no torn read)
};

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

/// Returns the current GNSS nanosecond (monotonic, PPS-counted, DWT-interpolated).
///
/// Returns -1 if:
///   - No calibration yet (fewer than 2 PPS edges observed)
///   - Anchor is stale (>3 seconds since last PPS update)
///
/// ISR-safe.  Zero allocation.  Zero blocking.  Pure register reads + arithmetic.
int64_t time_gnss_ns_now(void);

/// Convert a pre-captured DWT_CYCCNT value to a GNSS nanosecond.
///
/// This is the deterministic overload — given a specific DWT cycle count
/// (captured at some moment), returns the GNSS nanosecond at that moment.
/// Used by TimePop nano-precise callbacks to convert the spin-landed
/// DWT value to a GNSS nanosecond.
///
/// Returns -1 if time is not valid or anchor is stale.
/// ISR-safe.
int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt);

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

/// Convert a target GNSS nanosecond to the corresponding DWT_CYCCNT value.
///
/// This is the reverse of time_dwt_to_gnss_ns().  Given a future GNSS
/// nanosecond, returns the DWT_CYCCNT value that will be observed at
/// that moment (assuming the current DWT-to-GNSS rate holds).
///
/// Used by TimePop nano-precise scheduling to compute the DWT spin
/// target from a user-specified GNSS nanosecond.
///
/// Returns 0 if time is not valid.  The caller must check time_valid()
/// before relying on the result.
/// ISR-safe.
uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns);

// ============================================================================
// Anchor snapshot — seqlock-safe copy of the full PPS anchor
// ============================================================================

/// Returns a consistent snapshot of the PPS anchor state.
///
/// If ok == false, a torn read occurred — retry or abandon.
/// If valid == false, calibration is not yet established.
///
/// Used by TimePop for DWT prediction and VCLOCK ground truth
/// without any dependency on the timebase fragment or campaigns.
///
/// ISR-safe.
time_anchor_snapshot_t time_anchor_snapshot(void);

// ============================================================================
// Status
// ============================================================================

/// Returns the number of PPS edges seen since time zero.
/// Returns 0 if not yet valid.
uint32_t time_pps_count(void);

/// Returns true if time_gnss_ns_now() is ready to return valid values.
bool time_valid(void);

// ============================================================================
// Update interface (called by process_clocks_alpha.cpp PPS callback ONLY)
// ============================================================================

/// Feed a new PPS anchor to the time module.
///
/// Called once per PPS edge from the deferred PPS callback in
/// process_clocks_alpha.cpp, after the continuous calibration block
/// has updated dwt_cycles_per_gnss_s.
///
/// On the first call: establishes time zero (pps_count = 1, anchor set).
/// On the second call: calibration becomes valid (pps_count = 2).
/// Subsequent calls: pps_count increments, anchor advances.
///
/// dwt_at_pps:       DWT_CYCCNT at the PPS edge (ISR-corrected).
/// dwt_cycles_per_s: DWT cycles measured in the prior GNSS second.
///                   Zero on the first call (no prior second yet).
///                   Must be nonzero from the second call onward.
/// gpt2_at_pps:      GPT2_CNT captured at the PPS edge.
///                   This is the GNSS VCLOCK position at the PPS moment.
void time_pps_update(uint32_t dwt_at_pps, uint32_t dwt_cycles_per_s, uint32_t gpt2_at_pps);

// ============================================================================
// Initialization
// ============================================================================

/// Zero all state.  Called once at startup before PPS ISR is enabled.
void time_init(void);