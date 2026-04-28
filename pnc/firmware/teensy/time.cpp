// ============================================================================
// time.cpp — VCLOCK-authored GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// PPS is not the timebase here.  PPS selects a VCLOCK edge; the selected
// PPS/VCLOCK edge becomes the canonical anchor.  DWT is then used only as a
// high-resolution ruler to interpolate from that selected VCLOCK edge.
//
// This module performs NO latency adjustment.  All DWT inputs are already
// event-coordinate values.
// ============================================================================

#include "time.h"
#include "config.h"

#include <Arduino.h>
#include "imxrt.h"

static constexpr uint64_t NS_PER_SEC = 1000000000ULL;
static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;

// ============================================================================
// Anchor state (written by clocks alpha, read by time readers)
// ============================================================================

struct time_anchor_t {
  volatile uint32_t seq;
  volatile uint32_t dwt_at_pps_vclock;
  volatile uint32_t dwt_cycles_per_pps_vclock_s;
  volatile uint32_t counter32_at_pps_vclock;
  volatile uint32_t pps_vclock_count;
  volatile bool     valid;
};

static time_anchor_t anchor = {};

static inline void dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

// ============================================================================
// Writer helpers
// ============================================================================

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock) {
  anchor.seq++;
  dmb();

  anchor.dwt_at_pps_vclock             = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s   = 0;
  anchor.counter32_at_pps_vclock       = counter32_at_pps_vclock;
  anchor.pps_vclock_count              = 1;
  anchor.valid                         = false;

  dmb();
  anchor.seq++;
}

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock) {
  anchor.seq++;
  dmb();

  if (anchor.pps_vclock_count == 0) {
    anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
    anchor.dwt_cycles_per_pps_vclock_s = 0;
    anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
    anchor.pps_vclock_count            = 1;
    anchor.valid                       = false;
  } else {
    anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
    anchor.dwt_cycles_per_pps_vclock_s = dwt_cycles_per_pps_vclock_s;
    anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
    anchor.pps_vclock_count++;
    anchor.valid = (dwt_cycles_per_pps_vclock_s > 0);
  }

  dmb();
  anchor.seq++;
}

void time_pps_epoch_reset(uint32_t dwt_at_pps,
                          uint32_t qtimer_at_pps) {
  time_pps_vclock_epoch_reset(dwt_at_pps, qtimer_at_pps);
}

void time_pps_update(uint32_t dwt_at_pps,
                     uint32_t dwt_cycles_per_s,
                     uint32_t qtimer_at_pps) {
  time_pps_vclock_update(dwt_at_pps, dwt_cycles_per_s, qtimer_at_pps);
}

// ============================================================================
// Internal reader — stack-local snapshot with torn-read protection
// ============================================================================

struct time_snapshot_t {
  uint32_t dwt_at_pps_vclock;
  uint32_t dwt_cycles_per_pps_vclock_s;
  uint32_t counter32_at_pps_vclock;
  uint32_t pps_vclock_count;
  bool     valid;
  bool     ok;
};

static time_snapshot_t read_anchor(void) {
  time_snapshot_t s = {};
  s.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = anchor.seq;
    dmb();

    s.dwt_at_pps_vclock           = anchor.dwt_at_pps_vclock;
    s.dwt_cycles_per_pps_vclock_s = anchor.dwt_cycles_per_pps_vclock_s;
    s.counter32_at_pps_vclock     = anchor.counter32_at_pps_vclock;
    s.pps_vclock_count            = anchor.pps_vclock_count;
    s.valid                       = anchor.valid;

    dmb();
    const uint32_t s2 = anchor.seq;

    if (s1 == s2 && (s1 & 1u) == 0u) {
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
  const time_snapshot_t s = read_anchor();
  time_anchor_snapshot_t pub = {};

  pub.dwt_at_pps_vclock           = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_pps_vclock_s = s.dwt_cycles_per_pps_vclock_s;
  pub.counter32_at_pps_vclock     = s.counter32_at_pps_vclock;
  pub.pps_vclock_count            = s.pps_vclock_count;
  pub.valid                       = s.valid;
  pub.ok                          = s.ok;

  // Legacy aliases.
  pub.dwt_at_pps       = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_s = s.dwt_cycles_per_pps_vclock_s;
  pub.qtimer_at_pps    = s.counter32_at_pps_vclock;
  pub.pps_count        = s.pps_vclock_count;

  return pub;
}

// ============================================================================
// Internal helper — DWT elapsed to GNSS nanoseconds
// ============================================================================

static inline int64_t interpolate_gnss_ns(const time_snapshot_t& s,
                                          uint32_t dwt_elapsed) {
  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * NS_PER_SEC +
       (uint64_t)s.dwt_cycles_per_pps_vclock_s / 2ULL) /
      (uint64_t)s.dwt_cycles_per_pps_vclock_s;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(s.pps_vclock_count - 1) * NS_PER_SEC +
                   ns_into_second);
}

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (s.dwt_cycles_per_pps_vclock_s == 0) return -1;

  const uint32_t dwt_now = ARM_DWT_CYCCNT;
  const uint32_t dwt_elapsed = dwt_now - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (s.dwt_cycles_per_pps_vclock_s == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps_vclock,
                            uint32_t anchor_dwt_cycles_per_pps_vclock_s,
                            uint32_t anchor_pps_vclock_count) {
  if (anchor_dwt_cycles_per_pps_vclock_s == 0) return -1;
  if (anchor_pps_vclock_count == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - anchor_dwt_at_pps_vclock;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * NS_PER_SEC +
       (uint64_t)anchor_dwt_cycles_per_pps_vclock_s / 2ULL) /
      (uint64_t)anchor_dwt_cycles_per_pps_vclock_s;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(anchor_pps_vclock_count - 1) * NS_PER_SEC +
                   ns_into_second);
}

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return 0;
  if (s.dwt_cycles_per_pps_vclock_s == 0) return 0;
  if (gnss_ns < 0) return 0;

  const int64_t anchor_ns =
      (int64_t)(s.pps_vclock_count - 1) * (int64_t)NS_PER_SEC;
  const int64_t ns_into_second = gnss_ns - anchor_ns;
  if (ns_into_second < 0) return 0;

  const uint32_t dwt_elapsed =
      (uint32_t)(((uint64_t)ns_into_second *
                  (uint64_t)s.dwt_cycles_per_pps_vclock_s + NS_PER_SEC / 2ULL) /
                 NS_PER_SEC);

  return s.dwt_at_pps_vclock + dwt_elapsed;
}

// ============================================================================
// Status
// ============================================================================

uint32_t time_pps_vclock_count(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok) return 0;
  return s.pps_vclock_count;
}

uint32_t time_pps_count(void) {
  return time_pps_vclock_count();
}

bool time_valid(void) {
  const time_snapshot_t s = read_anchor();
  return s.ok && s.valid && s.dwt_cycles_per_pps_vclock_s > 0;
}

void time_init(void) {
  anchor = {};
}
