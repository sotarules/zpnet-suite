#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "payload.h"

// ============================================================================
// TimeBase — PPS-Anchored Temporal Oracle
// ============================================================================
//
// TimeBase answers the question: "What nanosecond is it right now
// in clock domain X?"
//
// Architecture:
//
//   The Pi publishes TIMEBASE records once per second, synchronized
//   to the GNSS PPS edge.  Each record contains the cumulative
//   nanosecond count for every tracked clock domain (GNSS, DWT,
//   OCXO, Pi) at that PPS boundary.
//
//   The Teensy subscribes to this feed and calls set() on each
//   arrival.  set() captures the current DWT cycle count as an
//   interpolation anchor.
//
//   Between updates, now("GNSS") reads DWT right now, computes
//   elapsed DWT nanoseconds since the anchor, scales by the
//   GNSS/DWT rate ratio (tau), and adds to the snapshot value.
//
//   This gives sub-microsecond time resolution in any domain
//   from a free-running local counter.  The Pi crystal (+7 ppm)
//   and the OCXO (~1 ppb, pending) are both accessible through
//   the same interface.
//
// Usage:
//
//   // Global instance, updated by TIMEBASE subscription
//   TimeBase timebase;
//
//   // In your TIMEBASE subscription handler:
//   void on_timebase(const Payload& p) {
//     timebase.set(p);
//   }
//
//   // In your experiment code:
//   uint64_t gnss_ns = timebase.now("GNSS");
//   uint64_t ocxo_ns = timebase.now("OCXO");
//   uint64_t race_duration_gnss = end_gnss - start_gnss;
//   uint64_t race_duration_ocxo = end_ocxo - start_ocxo;
//
// Clock domains:
//
//   "GNSS"  — 10 MHz VCLK, GNSS-disciplined (truth reference)
//   "DWT"   — 600 MHz Cortex-M7 cycle counter (interpolation clock)
//   "OCXO"  — 10 MHz oven oscillator (pending hardware)
//   "PI"    — 54 MHz ARM Generic Timer (Pi crystal, +7 ppm)
//
// Thread safety:
//
//   set() and now() are expected to run in scheduled (non-ISR)
//   context on the Teensy's single-threaded cooperative scheduler.
//   No locking is needed.
//
// ============================================================================

class TimeBase {
public:

  // --------------------------------------------------------------------------
  // Lifecycle
  // --------------------------------------------------------------------------

  TimeBase();

  // --------------------------------------------------------------------------
  // Update from TIMEBASE record
  // --------------------------------------------------------------------------
  //
  // Called when a TIMEBASE publication arrives from the Pi.
  // Parses the payload, extracts all clock domain values, and
  // captures the current DWT cycle count as the interpolation
  // anchor.
  //
  // The payload must contain at minimum:
  //   teensy_dwt_cycles  — DWT cycle count at PPS (uint64)
  //   teensy_dwt_ns      — DWT nanoseconds at PPS (uint64)
  //   teensy_gnss_ns     — GNSS nanoseconds at PPS (uint64)
  //
  // Optional fields (used when present):
  //   teensy_ocxo_ns     — OCXO nanoseconds at PPS (uint64)
  //   pi_ns              — Pi nanoseconds at PPS (uint64)
  //
  void set(const Payload& timebase_payload);

  // --------------------------------------------------------------------------
  // Current time in a clock domain
  // --------------------------------------------------------------------------
  //
  // Returns the estimated nanosecond count in the named clock
  // domain at this instant.
  //
  // Internally reads DWT_CYCCNT, computes elapsed DWT nanoseconds
  // since the last set() call, scales by the domain's rate ratio
  // (tau = domain_ns / dwt_ns), and adds to the snapshot value.
  //
  // Returns 0 if the domain is not available or no TIMEBASE has
  // been received.
  //
  uint64_t now(const char* clock_name) const;

  // --------------------------------------------------------------------------
  // Cross-domain conversion
  // --------------------------------------------------------------------------
  //
  // Given a nanosecond count in one domain, return the equivalent
  // nanosecond count in another domain.
  //
  // Uses the tau ratio between the two domains:
  //   to_ns = to_snapshot + (from_ns - from_snapshot) * (tau_to / tau_from)
  //
  // Returns 0 if either domain is unavailable.
  //
  uint64_t convert(
    const char* from_clock,
    const char* to_clock,
    uint64_t    from_ns
  ) const;

  // --------------------------------------------------------------------------
  // Rate ratio (tau)
  // --------------------------------------------------------------------------
  //
  // Returns the dimensionless rate ratio: domain_ns / gnss_ns.
  //
  // A perfect clock returns 1.0.  DWT at -0.8 ppm returns
  // 0.9999992.  Pi at +7 ppm returns 1.000007.
  //
  // Returns 0.0 if the domain is unavailable.
  //
  double tau(const char* clock_name) const;

  // --------------------------------------------------------------------------
  // Health
  // --------------------------------------------------------------------------

  // True after at least one successful set() call.
  bool valid() const;

  // Milliseconds since last set() call.
  // Uses DWT for measurement (no dependency on millis()).
  uint32_t age_ms() const;

  // PPS count from the most recent TIMEBASE record.
  uint64_t pps_count() const;

private:

  // --------------------------------------------------------------------------
  // Per-domain snapshot (populated by set())
  // --------------------------------------------------------------------------

  static constexpr size_t MAX_CLOCKS = 6;
  static constexpr size_t NAME_LEN   = 8;

  struct Clock {
    char     name[NAME_LEN];    // "GNSS", "DWT", "OCXO", "PI"
    uint64_t ns_at_pps;         // cumulative ns at last PPS
    bool     active;            // true if this domain has data
  };

  Clock  clocks_[MAX_CLOCKS];
  size_t clock_count_;

  // DWT anchor — captured at set() time for interpolation
  uint64_t anchor_dwt_cycles_;  // DWT cycle count when set() was called
  uint64_t anchor_dwt_ns_;      // DWT ns from the TIMEBASE record

  // GNSS ns at anchor (for tau computation)
  uint64_t anchor_gnss_ns_;

  // Campaign PPS count
  uint64_t pps_count_;

  // Validity
  bool valid_;

  // --------------------------------------------------------------------------
  // Internal helpers
  // --------------------------------------------------------------------------

  const Clock* find_clock(const char* name) const;
  Clock*       find_clock_or_add(const char* name);

  // Set a clock domain's snapshot value.
  // Creates the domain if it doesn't exist yet.
  void set_clock(const char* name, uint64_t ns_at_pps);
};