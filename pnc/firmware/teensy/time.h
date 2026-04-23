#pragma once

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.h — Universal GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// Core functions:
//
//   time_gnss_ns_now()         — current GNSS nanosecond (DWT interpolated)
//   time_dwt_to_gnss_ns()      — convert a DWT_CYCCNT value to GNSS nanoseconds
//   time_gnss_ns_to_dwt()      — convert a GNSS nanosecond to a DWT_CYCCNT value
//   time_anchor_snapshot()     — seqlock-safe copy of the PPS anchor state
//
// Returns monotonic, PPS-counted nanoseconds with DWT interpolation
// between PPS edges. Available from boot as soon as the second PPS
// edge establishes calibration. ISR-safe.
//
// The returned value is a LOCAL temporal coordinate, not a wall clock.
// It counts nanoseconds from the current sacred epoch.
// That epoch is established on a lawful PPS edge and may be restarted
// later (for example: startup epoch, explicit zero, campaign start epoch).
//
// Architecture:
//
//   process_clocks_alpha.cpp calls time_pps_update() once per PPS edge,
//   providing:
//     - dwt_at_pps:         DWT_CYCCNT captured at the PPS edge
//     - dwt_cycles_per_s:   measured DWT cycles in the prior second
//     - qtimer_at_pps:      QTimer1 raw counter captured at the PPS edge
//
//   process_clocks_alpha.cpp may also call time_pps_epoch_reset() on a
//   lawful PPS edge to restart the local epoch at zero.
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
//   The PPS callback is the sole writer. All other callers are readers.
//
// ============================================================================

// ============================================================================
// Latency adjusters
// ============================================================================

uint32_t dwt_at_gpio_edge(uint32_t dwt_isr_entry_raw);
uint32_t dwt_at_qtimer_edge(uint32_t dwt_isr_entry_raw);

// ============================================================================
// Anchor snapshot — public, seqlock-safe copy of the PPS anchor
// ============================================================================

struct time_anchor_snapshot_t {
  uint32_t dwt_at_pps;         // DWT_CYCCNT at the PPS edge
  uint32_t dwt_cycles_per_s;   // DWT cycles in the prior GNSS second
  uint32_t qtimer_at_pps;      // QTimer1 raw counter at the PPS edge (VCLOCK position)
  uint32_t pps_count;          // PPS edges seen in current epoch (1-indexed)
  bool     valid;              // true after second PPS in current epoch (rate available)
  bool     ok;                 // true if snapshot was consistent (no torn read)
};

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void);

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt);

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps,
                            uint32_t anchor_dwt_cycles_per_s,
                            uint32_t anchor_pps_count);

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns);

// ============================================================================
// Anchor snapshot — seqlock-safe copy of the full PPS anchor
// ============================================================================

time_anchor_snapshot_t time_anchor_snapshot(void);

// ============================================================================
// Status
// ============================================================================

uint32_t time_pps_count(void);
bool time_valid(void);

// ============================================================================
// Update interface (called by process_clocks_alpha.cpp PPS callback ONLY)
// ============================================================================

/// Normal PPS advance within the current epoch.
///
/// On the first call in a fresh epoch:
///   - pps_count becomes 1
///   - valid remains false
///
/// On the second call:
///   - pps_count becomes 2
///   - valid becomes true if dwt_cycles_per_s > 0
///
/// Subsequent calls:
///   - pps_count increments each PPS
void time_pps_update(uint32_t dwt_at_pps,
                     uint32_t dwt_cycles_per_s,
                     uint32_t qtimer_at_pps);

/// Restart the local epoch on a lawful PPS edge.
///
/// This must only be called from code that has already decided that this
/// PPS edge is the new sacred origin (e.g. startup epoch, explicit zero,
/// campaign-start epoch).
///
/// After this call:
///   - pps_count becomes 1
///   - returned GNSS nanoseconds count from this PPS edge as zero
///   - valid is false until the next PPS establishes a real second-rate
void time_pps_epoch_reset(uint32_t dwt_at_pps,
                          uint32_t qtimer_at_pps);

// ============================================================================
// Initialization
// ============================================================================

void time_init(void);