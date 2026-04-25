// ============================================================================
// time.cpp — Universal GNSS Nanosecond Bridge (Teensy)
// ============================================================================
//
// See time.h for the contract.  Three pure functions, no state.
//
// Implementation:
//
//   The canonical anchor is alpha's PPS_VCLOCK slot — a single
//   {gnss_ns_at_edge, dwt_at_edge, counter32_at_edge, sequence} tuple
//   authored by alpha when it processes a PPS_VCLOCK subscription event.
//
//   This module reads gnss_ns_at_edge and dwt_at_edge.  It does not read
//   counter32_at_edge — the bridge math is purely DWT-vs-rate.  TimePop
//   is the consumer of counter32_at_edge for scheduling.
//
//   The canonical rate is alpha's measured DWT cycles per GNSS second
//   (the bookend between two consecutive PPS_VCLOCK edges).
//
//   Forward bridge:
//     gnss_ns(dwt) = anchor.gnss_ns + (dwt - anchor.dwt) × 1e9 / cps
//
//   Reverse bridge:
//     dwt(gnss_ns) = anchor.dwt + (gnss_ns - anchor.gnss_ns) × cps / 1e9
//
// Returns -1 (forward) or 0 (reverse) when:
//   • Alpha hasn't yet seen a PPS_VCLOCK edge (sequence == 0), or
//   • Rate not yet measured (need two PPS edges for a bookend), or
//   • Result would overflow the 3-second staleness guard.
// ============================================================================

#include "time.h"

#include <Arduino.h>
#include "imxrt.h"

static constexpr uint64_t NS_PER_SEC = 1000000000ULL;
static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;

// ============================================================================
// Forward bridge
// ============================================================================

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt) {
  const alpha_pps_vclock_slot_t a = alpha_pps_vclock_load();
  if (a.sequence == 0)            return -1;
  const uint32_t cps = alpha_dwt_cycles_per_second();
  if (cps == 0)                   return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - a.dwt_at_edge;
  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * NS_PER_SEC + (uint64_t)cps / 2) / (uint64_t)cps;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return a.gnss_ns_at_edge + (int64_t)ns_into_second;
}

int64_t time_gnss_ns_now(void) {
  return time_dwt_to_gnss_ns(ARM_DWT_CYCCNT);
}

// ============================================================================
// Reverse bridge
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  if (gnss_ns < 0)                return 0;
  const alpha_pps_vclock_slot_t a = alpha_pps_vclock_load();
  if (a.sequence == 0)            return 0;
  const uint32_t cps = alpha_dwt_cycles_per_second();
  if (cps == 0)                   return 0;

  const int64_t ns_into_second = gnss_ns - a.gnss_ns_at_edge;
  if (ns_into_second < 0)         return 0;

  const uint32_t dwt_elapsed =
      (uint32_t)(((uint64_t)ns_into_second * (uint64_t)cps + NS_PER_SEC / 2) / NS_PER_SEC);

  return a.dwt_at_edge + dwt_elapsed;
}