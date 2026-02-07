#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ============================================================================
// TimeBase — PPS-Anchored Temporal Reference Frame (Firmware-Safe)
// ============================================================================

class TimeBase {
public:
  static constexpr size_t MAX_CLOCKS = 10;
  static constexpr size_t NAME_LEN   = 8;   // "GNSS", "DWT", "OCXO", etc.

  struct Clock {
    char     name[NAME_LEN];
    uint64_t ns_at_pps;
    uint64_t cycles_at_pps;
    uint64_t ns_per_cycle_n;
    uint64_t ns_per_cycle_d;
  };

  // --------------------------------------------------------------------------
  // Lifecycle
  // --------------------------------------------------------------------------

  TimeBase();

  void clear();

  // --------------------------------------------------------------------------
  // Clock management
  // --------------------------------------------------------------------------

  bool add_clock(
    const char* name,
    uint64_t ns_at_pps,
    uint64_t cycles_at_pps,
    uint64_t ns_per_cycle_n,
    uint64_t ns_per_cycle_d
  );

  bool has_clock(const char* name) const;

  // --------------------------------------------------------------------------
  // Core operations
  // --------------------------------------------------------------------------

  uint64_t now(
    const char* clock_name,
    uint64_t cycle_count_now
  ) const;

  uint64_t convert(
    const char* from_clock,
    const char* to_clock,
    uint64_t from_ns
  ) const;

  double tau(const char* clock_name) const;

private:
  const Clock* find_clock(const char* name) const;
  Clock*       find_clock(const char* name);

  Clock  clocks_[MAX_CLOCKS];
  size_t clock_count_;
};
