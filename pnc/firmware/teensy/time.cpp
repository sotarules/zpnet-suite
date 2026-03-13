// ============================================================================
// time.cpp — Universal GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// See time.h for full design rationale.
//
// State model:
//
//   The PPS callback in process_clocks.cpp calls time_pps_update()
//   on every PPS edge.  This advances the anchor:
//
//     Call 1 (first PPS):
//       - dwt_at_pps recorded, pps_count = 1 ("one PPS seen")
//       - dwt_cycles_per_s is 0 (no prior second to measure)
//       - NOT YET VALID — we have an anchor but no rate
//
//     Call 2 (second PPS):
//       - dwt_at_pps updated, dwt_cycles_per_s is now the first real delta
//       - pps_count = 2 ("two PPS edges seen, one complete second")
//       - VALID — time_gnss_ns_now() can interpolate
//
//     Call N:
//       - pps_count = N
//       - anchor and rate advance each second
//
//   Between PPS edges, time_gnss_ns_now() reads DWT_CYCCNT and
//   interpolates:
//
//     gnss_ns = (pps_count - 1) * 1,000,000,000
//             + (DWT_CYCCNT - dwt_at_pps) * 1,000,000,000 / dwt_cycles_per_s
//
//   pps_count is "PPS edges seen" (1-indexed).  Completed seconds
//   is pps_count - 1.  After PPS #2: pps_count=2, completed=1,
//   total at edge = 1,000,000,000.  Correct.
//
// Torn-read protection:
//
//   Seqlock pattern.  The writer (time_pps_update) increments seq
//   before and after the write, with DMB barriers.  Readers retry
//   if seq changed during the read or if seq is odd (write in progress).
//
// ============================================================================

#include "time.h"
#include "config.h"

#include <Arduino.h>
#include "imxrt.h"

// ============================================================================
// Constants
// ============================================================================

static constexpr uint64_t NS_PER_SEC = 1000000000ULL;

// Staleness guard: if DWT_CYCCNT has advanced more than this many
// nanoseconds beyond the anchor, the anchor is considered stale.
// 3 seconds matches TIMEBASE_MAX_FRAGMENT_AGE_NS in timebase.cpp.
static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;

// ============================================================================
// Anchor state (written by time_pps_update, read by time_gnss_ns_now)
// ============================================================================

struct time_anchor_t {
  volatile uint32_t seq;              // seqlock sequence number
  volatile uint32_t dwt_at_pps;       // DWT_CYCCNT at most recent PPS edge
  volatile uint32_t dwt_cycles_per_s; // DWT cycles in the prior GNSS second
  volatile uint32_t pps_count;        // PPS edges seen (1 = first PPS, 2 = first complete second)
  volatile bool     valid;            // true after second PPS (rate available)
};

static time_anchor_t anchor = {};

// ============================================================================
// DMB helper
// ============================================================================

static inline void dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

// ============================================================================
// Writer — called from PPS callback in process_clocks.cpp
// ============================================================================

void time_pps_update(uint32_t dwt_at_pps, uint32_t dwt_cycles_per_s) {

  anchor.seq++;
  dmb();

  if (!anchor.valid && anchor.pps_count == 0) {
    // First PPS: establish time zero.
    // dwt_cycles_per_s is 0 here (no prior second to measure).
    // We record the anchor and mark "one PPS seen" so the next
    // call takes the else branch.  Valid remains false — we need
    // the rate from the second PPS before interpolation works.
    anchor.dwt_at_pps       = dwt_at_pps;
    anchor.dwt_cycles_per_s = 0;
    anchor.pps_count        = 1;
    // valid remains false
  } else {
    // Second PPS onward: advance the anchor.
    anchor.dwt_at_pps       = dwt_at_pps;
    anchor.dwt_cycles_per_s = dwt_cycles_per_s;
    anchor.pps_count++;

    if (dwt_cycles_per_s > 0) {
      anchor.valid = true;
    }
  }

  dmb();
  anchor.seq++;
}

// ============================================================================
// Reader — stack-local snapshot with torn-read protection
// ============================================================================

struct time_snapshot_t {
  uint32_t dwt_at_pps;
  uint32_t dwt_cycles_per_s;
  uint32_t pps_count;
  bool     valid;
  bool     ok;       // true if snapshot was consistent
};

static time_snapshot_t read_anchor(void) {
  time_snapshot_t s = {};
  s.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t s1 = anchor.seq;
    dmb();

    s.dwt_at_pps       = anchor.dwt_at_pps;
    s.dwt_cycles_per_s = anchor.dwt_cycles_per_s;
    s.pps_count        = anchor.pps_count;
    s.valid            = anchor.valid;

    dmb();
    uint32_t s2 = anchor.seq;

    if (s1 == s2 && (s1 & 1) == 0) {
      s.ok = true;
      return s;
    }
  }

  return s;  // ok = false
}

// ============================================================================
// Core API
// ============================================================================

int64_t time_gnss_ns_now(void) {

  time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (s.dwt_cycles_per_s == 0) return -1;

  // Read DWT_CYCCNT — this is the "now" moment.
  const uint32_t dwt_now = ARM_DWT_CYCCNT;

  // Elapsed DWT cycles since the last PPS edge.
  // Unsigned subtraction handles 32-bit wrap correctly.
  const uint32_t dwt_elapsed = dwt_now - s.dwt_at_pps;

  // Convert to nanoseconds within the current second.
  // Round-to-nearest: (elapsed * 1e9 + denom/2) / denom
  const uint64_t ns_into_second =
    ((uint64_t)dwt_elapsed * NS_PER_SEC + (uint64_t)s.dwt_cycles_per_s / 2)
    / (uint64_t)s.dwt_cycles_per_s;

  // Staleness guard.
  if (ns_into_second > MAX_AGE_NS) return -1;

  // Total GNSS nanoseconds: completed seconds + interpolation.
  //
  // pps_count is "PPS edges seen" (1-indexed).
  // Completed seconds = pps_count - 1.
  //
  // After PPS #2: pps_count=2, completed=1.
  //   At the edge: 1 * 1e9 + 0 = 1,000,000,000.  Correct.
  //   Midway:      1 * 1e9 + 500,000,000 = 1,500,000,000.  Correct.
  //
  // After PPS #3: pps_count=3, completed=2.
  //   At the edge: 2 * 1e9 = 2,000,000,000.  Correct.

  return (int64_t)((uint64_t)(s.pps_count - 1) * NS_PER_SEC + ns_into_second);
}

uint32_t time_pps_count(void) {
  time_snapshot_t s = read_anchor();
  if (!s.ok) return 0;
  return s.pps_count;
}

bool time_valid(void) {
  time_snapshot_t s = read_anchor();
  return s.ok && s.valid && s.dwt_cycles_per_s > 0;
}

// ============================================================================
// Initialization
// ============================================================================

void time_init(void) {
  anchor = {};
}