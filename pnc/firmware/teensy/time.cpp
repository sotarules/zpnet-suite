// ============================================================================
// time.cpp — Universal GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// See time.h for full design rationale.
//
// State model:
//
//   The PPS callback in process_clocks_alpha.cpp calls time_pps_update()
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
// Reverse conversion (GNSS ns → DWT):
//
//   target_gnss_ns → how many nanoseconds into the current second?
//     ns_into_second = target_gnss_ns - (pps_count - 1) * 1e9
//   Convert to DWT cycles:
//     dwt_elapsed = ns_into_second * dwt_cycles_per_s / 1e9
//   Add to PPS anchor:
//     target_dwt = dwt_at_pps + dwt_elapsed
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
  volatile uint32_t gpt2_at_pps;      // GPT2_CNT at most recent PPS edge
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
// Writer — called from PPS callback in process_clocks_alpha.cpp
// ============================================================================

void time_pps_update(uint32_t dwt_at_pps, uint32_t dwt_cycles_per_s, uint32_t gpt2_at_pps) {

  anchor.seq++;
  dmb();

  if (!anchor.valid && anchor.pps_count == 0) {
    anchor.dwt_at_pps       = dwt_at_pps;
    anchor.dwt_cycles_per_s = 0;
    anchor.gpt2_at_pps      = gpt2_at_pps;
    anchor.pps_count        = 1;
  } else {
    anchor.dwt_at_pps       = dwt_at_pps;
    anchor.dwt_cycles_per_s = dwt_cycles_per_s;
    anchor.gpt2_at_pps      = gpt2_at_pps;
    anchor.pps_count++;

    if (dwt_cycles_per_s > 0) {
      anchor.valid = true;
    }
  }

  dmb();
  anchor.seq++;
}

// ============================================================================
// Internal reader — stack-local snapshot with torn-read protection
// ============================================================================

struct time_snapshot_t {
  uint32_t dwt_at_pps;
  uint32_t dwt_cycles_per_s;
  uint32_t gpt2_at_pps;
  uint32_t pps_count;
  bool     valid;
  bool     ok;
};

static time_snapshot_t read_anchor(void) {
  time_snapshot_t s = {};
  s.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t s1 = anchor.seq;
    dmb();

    s.dwt_at_pps       = anchor.dwt_at_pps;
    s.dwt_cycles_per_s = anchor.dwt_cycles_per_s;
    s.gpt2_at_pps      = anchor.gpt2_at_pps;
    s.pps_count        = anchor.pps_count;
    s.valid            = anchor.valid;

    dmb();
    uint32_t s2 = anchor.seq;

    if (s1 == s2 && (s1 & 1) == 0) {
      s.ok = true;
      return s;
    }
  }

  return s;
}

// ============================================================================
// Public reader — seqlock-safe anchor snapshot
// ============================================================================

time_anchor_snapshot_t time_anchor_snapshot(void) {
  time_snapshot_t s = read_anchor();
  time_anchor_snapshot_t pub = {};
  pub.dwt_at_pps       = s.dwt_at_pps;
  pub.dwt_cycles_per_s = s.dwt_cycles_per_s;
  pub.gpt2_at_pps      = s.gpt2_at_pps;
  pub.pps_count        = s.pps_count;
  pub.valid            = s.valid;
  pub.ok               = s.ok;
  return pub;
}

// ============================================================================
// Internal helper — DWT elapsed to GNSS nanoseconds (shared by forward APIs)
// ============================================================================

static inline int64_t interpolate_gnss_ns(const time_snapshot_t& s, uint32_t dwt_elapsed) {
  const uint64_t ns_into_second =
    ((uint64_t)dwt_elapsed * NS_PER_SEC + (uint64_t)s.dwt_cycles_per_s / 2)
    / (uint64_t)s.dwt_cycles_per_s;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(s.pps_count - 1) * NS_PER_SEC + ns_into_second);
}

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void) {
  time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (s.dwt_cycles_per_s == 0) return -1;

  const uint32_t dwt_now = ARM_DWT_CYCCNT;
  const uint32_t dwt_elapsed = dwt_now - s.dwt_at_pps;

  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt) {
  time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (s.dwt_cycles_per_s == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - s.dwt_at_pps;

  return interpolate_gnss_ns(s, dwt_elapsed);
}

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return 0;
  if (s.dwt_cycles_per_s == 0) return 0;

  const int64_t anchor_ns = (int64_t)(s.pps_count - 1) * (int64_t)NS_PER_SEC;
  const int64_t ns_into_second = gnss_ns - anchor_ns;

  const uint32_t dwt_elapsed =
    (uint32_t)(((uint64_t)ns_into_second * (uint64_t)s.dwt_cycles_per_s + NS_PER_SEC / 2)
    / NS_PER_SEC);

  return s.dwt_at_pps + dwt_elapsed;
}

// ============================================================================
// Status
// ============================================================================

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