// ============================================================================
// timebase.cpp — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// PPS-fragment anchored Timebase.
//
// The authoritative TIMEBASE_FRAGMENT provides PPS-edge totals once per
// second.  When received, Timebase captures the local ARM_DWT_CYCCNT.
// "Now" is answered by projecting forward from that anchor:
//
//   gnss_now = fragment.gnss_ns + elapsed_dwt_cycles × 125 / 126
//
// The DWT runs at 1008 MHz.  The 125/126 ratio converts DWT cycles to
// nanoseconds.  At 19.5 ns period stddev and ±0.50 ns TDC quantization,
// the Teensy achieves sub-nanosecond timing over √n averages.
//
// All public API speaks nanoseconds.  Clock domain details (DWT cycles,
// OCXO ticks) are internal to this module and the conversion helpers.
//
// ============================================================================

#include "timebase.h"
#include "util.h"

#include <Arduino.h>
#include <stdlib.h>
#include <math.h>
#include "imxrt.h"

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t DWT_TO_NS_NUM = 125;
static constexpr uint32_t DWT_TO_NS_DEN = 126;

// Allow the most recent fragment to remain authoritative for up to 3 seconds.
// This comfortably spans the normal 1 Hz publish cadence while still rejecting
// genuinely stale state after stop or pipeline failure.
static constexpr uint64_t TIMEBASE_MAX_FRAGMENT_AGE_NS = 3000000000ULL;

// ============================================================================
// GPS epoch and leap seconds
// ============================================================================
//
// GNSS nanoseconds in ZPNet count from the GPS epoch (1980-01-06T00:00:00Z).
// The GNSS module applies the current UTC-GPS leap second correction, so
// gnss_ns is effectively UTC nanoseconds counted from the GPS epoch.
//
// GPS epoch = 1980-01-06 = Unix timestamp 315964800
// As of 2025, GPS-UTC leap seconds = 18 (already applied by module)

static constexpr uint64_t GPS_EPOCH_UNIX = 315964800ULL;

// Days in each month (non-leap, then leap)
static const uint8_t DAYS_IN_MONTH[2][12] = {
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},  // non-leap
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},   // leap
};

static inline bool is_leap_year(uint32_t y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

// ============================================================================
// Fragment state (written by TIMEBASE_FRAGMENT callback, read by anyone)
// ============================================================================

struct fragment_store_t {
  volatile uint64_t gnss_ns;
  volatile uint64_t dwt_ns;
  volatile uint64_t dwt_cycles;
  volatile uint64_t ocxo_ns;
  volatile uint32_t pps_count;
  volatile int32_t  isr_residual_dwt;
  volatile int32_t  isr_residual_gnss;
  volatile int32_t  isr_residual_ocxo;
  volatile uint32_t local_dwt_snap;
  volatile uint32_t seq;
  volatile bool     valid;
};

static fragment_store_t frag_store = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false};

// Public mirror for diagnostics / reports
static timebase_fragment_t frag_public = {};

// ============================================================================
// Safe multiply/divide helper
// ============================================================================

static inline uint64_t mul_div_u64(
  uint64_t value,
  uint64_t num,
  uint64_t den
) {
  if (den == 0) return 0;

  // Fast path: no overflow
  if (num != 0 && value <= (UINT64_MAX / num)) {
    return (value * num) / den;
  }

  // Slow path: use double for very large values
  double ratio  = (double)num / (double)den;
  double scaled = (double)value * ratio;

  if (scaled <= 0.0) return 0;
  return (uint64_t)(scaled + 0.5);
}

// ============================================================================
// Snapshot struct (stack-local, for torn-read-safe access)
// ============================================================================

struct fragment_snapshot_t {
  uint64_t gnss_ns;
  uint64_t dwt_ns;
  uint64_t dwt_cycles;
  uint64_t ocxo_ns;
  uint32_t pps_count;
  int32_t  isr_residual_dwt;
  int32_t  isr_residual_gnss;
  int32_t  isr_residual_ocxo;
  uint32_t local_dwt_snap;
  bool     valid;
  bool     ok;
};

// ============================================================================
// Fragment read with torn-read protection
// ============================================================================

static fragment_snapshot_t read_fragment(void) {
  fragment_snapshot_t snap = {};
  snap.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t s1 = frag_store.seq;
    dmb();

    snap.gnss_ns           = frag_store.gnss_ns;
    snap.dwt_ns            = frag_store.dwt_ns;
    snap.dwt_cycles        = frag_store.dwt_cycles;
    snap.ocxo_ns           = frag_store.ocxo_ns;
    snap.pps_count         = frag_store.pps_count;
    snap.isr_residual_dwt  = frag_store.isr_residual_dwt;
    snap.isr_residual_gnss = frag_store.isr_residual_gnss;
    snap.isr_residual_ocxo = frag_store.isr_residual_ocxo;
    snap.local_dwt_snap    = frag_store.local_dwt_snap;
    snap.valid             = frag_store.valid;

    dmb();
    uint32_t s2 = frag_store.seq;

    if (s1 == s2 && (s1 & 1u) == 0u) {
      snap.ok = true;
      return snap;
    }
  }

  return snap;
}

// ============================================================================
// Domain totals from fragment
// ============================================================================

static bool fragment_domain_total_ns(
  const fragment_snapshot_t& snap,
  timebase_domain_t          domain,
  uint64_t&                  out_ns
) {
  switch (domain) {
    case timebase_domain_t::GNSS:
      out_ns = snap.gnss_ns;
      return snap.gnss_ns > 0;

    case timebase_domain_t::DWT:
      out_ns = snap.dwt_ns;
      return snap.dwt_ns > 0;

    case timebase_domain_t::OCXO:
      out_ns = snap.ocxo_ns;
      return snap.ocxo_ns > 0;
  }

  out_ns = 0;
  return false;
}

// ============================================================================
// Forward projection from fragment receipt
// ============================================================================

static bool fragment_elapsed_ns(
  const fragment_snapshot_t& snap,
  uint64_t&                  out_elapsed_ns
) {
  if (!snap.ok || !snap.valid) return false;

  uint32_t dwt_now     = ARM_DWT_CYCCNT;
  uint32_t dwt_elapsed = dwt_now - snap.local_dwt_snap;

  uint64_t elapsed_ns =
    mul_div_u64((uint64_t)dwt_elapsed, DWT_TO_NS_NUM, DWT_TO_NS_DEN);

  if (elapsed_ns > TIMEBASE_MAX_FRAGMENT_AGE_NS) {
    return false;
  }

  out_elapsed_ns = elapsed_ns;
  return true;
}

static bool current_gnss_now(uint64_t& out_gnss_ns) {
  fragment_snapshot_t snap = read_fragment();
  uint64_t elapsed_ns = 0;
  if (!fragment_elapsed_ns(snap, elapsed_ns)) return false;

  out_gnss_ns = snap.gnss_ns + elapsed_ns;
  return true;
}

static bool current_dwt_now(uint64_t& out_dwt_ns) {
  fragment_snapshot_t snap = read_fragment();
  uint64_t elapsed_ns = 0;
  if (!fragment_elapsed_ns(snap, elapsed_ns)) return false;

  out_dwt_ns = snap.dwt_ns + elapsed_ns;
  return true;
}

// ============================================================================
// Domain conversion through fragment ratios
// ============================================================================

static bool convert_via_fragment(
  uint64_t           value_ns,
  timebase_domain_t  from_domain,
  timebase_domain_t  to_domain,
  uint64_t&          out_ns
) {
  if (from_domain == to_domain) {
    out_ns = value_ns;
    return true;
  }

  fragment_snapshot_t snap = read_fragment();
  if (!snap.ok || !snap.valid || snap.gnss_ns == 0) {
    return false;
  }

  uint64_t from_total_ns = 0;
  uint64_t to_total_ns   = 0;

  if (!fragment_domain_total_ns(snap, from_domain, from_total_ns)) return false;
  if (!fragment_domain_total_ns(snap, to_domain,   to_total_ns))   return false;

  // Normalize to GNSS first
  uint64_t gnss_ns = 0;
  if (from_domain == timebase_domain_t::GNSS) {
    gnss_ns = value_ns;
  } else {
    gnss_ns = mul_div_u64(value_ns, snap.gnss_ns, from_total_ns);
  }

  // Then convert to target domain
  if (to_domain == timebase_domain_t::GNSS) {
    out_ns = gnss_ns;
  } else {
    out_ns = mul_div_u64(gnss_ns, to_total_ns, snap.gnss_ns);
  }

  return true;
}

// ============================================================================
// Public API — now(domain)
// ============================================================================

int64_t timebase_now_ns(timebase_domain_t domain) {
  uint64_t now_ns = 0;

  switch (domain) {
    case timebase_domain_t::GNSS:
      if (!current_gnss_now(now_ns)) return -1;
      return (int64_t)now_ns;

    case timebase_domain_t::DWT:
      if (!current_dwt_now(now_ns)) return -1;
      return (int64_t)now_ns;

    case timebase_domain_t::OCXO: {
      uint64_t gnss_now_ns = 0;
      uint64_t ocxo_now_ns = 0;
      if (!current_gnss_now(gnss_now_ns)) return -1;
      if (!convert_via_fragment(
            gnss_now_ns,
            timebase_domain_t::GNSS,
            timebase_domain_t::OCXO,
            ocxo_now_ns)) {
        return -1;
      }
      return (int64_t)ocxo_now_ns;
    }
  }

  return -1;
}

int64_t timebase_now_gnss_ns(void) {
  uint64_t gnss_now_ns = 0;
  if (!current_gnss_now(gnss_now_ns)) return -1;
  return (int64_t)gnss_now_ns;
}

// ============================================================================
// Public API — convert(value, from, to)
// ============================================================================

int64_t timebase_convert_ns(
  uint64_t           value_ns,
  timebase_domain_t  from_domain,
  timebase_domain_t  to_domain
) {
  uint64_t converted = 0;
  if (!convert_via_fragment(value_ns, from_domain, to_domain, converted)) {
    return -1;
  }

  return (int64_t)converted;
}

// ============================================================================
// Public API — ISO 8601 formatting
// ============================================================================

int timebase_format_iso8601(uint64_t gnss_ns, char* buf, size_t buf_len) {
  if (buf == nullptr || buf_len < 31) return 0;
  if (gnss_ns == 0) return 0;

  // Convert GNSS nanoseconds to Unix seconds + sub-second nanoseconds
  uint64_t total_secs = gnss_ns / 1000000000ULL;
  uint32_t sub_ns     = (uint32_t)(gnss_ns % 1000000000ULL);

  uint64_t unix_secs = GPS_EPOCH_UNIX + total_secs;

  // Break Unix seconds into date/time components
  // Days since Unix epoch
  uint32_t days    = (uint32_t)(unix_secs / 86400ULL);
  uint32_t day_sec = (uint32_t)(unix_secs % 86400ULL);

  uint32_t hour = day_sec / 3600;
  uint32_t min  = (day_sec % 3600) / 60;
  uint32_t sec  = day_sec % 60;

  // Convert days since 1970-01-01 to year/month/day
  // Using a simple loop — called at most once per timestamp, not hot path
  uint32_t year = 1970;
  while (true) {
    uint32_t days_in_year = is_leap_year(year) ? 366 : 365;
    if (days < days_in_year) break;
    days -= days_in_year;
    year++;
  }

  uint8_t leap = is_leap_year(year) ? 1 : 0;
  uint32_t month = 0;
  while (month < 12 && days >= DAYS_IN_MONTH[leap][month]) {
    days -= DAYS_IN_MONTH[leap][month];
    month++;
  }
  month += 1;   // 1-based
  days  += 1;   // 1-based

  // Format: "2026-03-08T01:34:44.123456789Z"
  int written = snprintf(buf, buf_len,
    "%04lu-%02lu-%02luT%02lu:%02lu:%02lu.%09luZ",
    (unsigned long)year, (unsigned long)month, (unsigned long)days,
    (unsigned long)hour, (unsigned long)min, (unsigned long)sec,
    (unsigned long)sub_ns);

  return (written > 0 && (size_t)written < buf_len) ? written : 0;
}

int timebase_now_iso8601(char* buf, size_t buf_len) {
  int64_t now = timebase_now_gnss_ns();
  if (now < 0) return 0;
  return timebase_format_iso8601((uint64_t)now, buf, buf_len);
}

// ============================================================================
// Public API — PPS count
// ============================================================================

uint32_t timebase_pps_count(void) {
  fragment_snapshot_t snap = read_fragment();
  if (!snap.ok) return 0;
  return snap.pps_count;
}

// ============================================================================
// Validity
// ============================================================================

bool timebase_valid(void) {
  uint64_t gnss_now_ns = 0;
  return current_gnss_now(gnss_now_ns);
}

bool timebase_conversion_valid(void) {
  fragment_snapshot_t snap = read_fragment();
  if (!snap.ok || !snap.valid) return false;
  if (snap.gnss_ns == 0) return false;
  if (snap.dwt_ns  == 0) return false;
  if (snap.ocxo_ns == 0) return false;
  return true;
}

// ============================================================================
// Invalidation
// ============================================================================

void timebase_invalidate(void) {
  frag_store.seq++;
  dmb();

  frag_store.gnss_ns           = 0;
  frag_store.dwt_ns            = 0;
  frag_store.dwt_cycles        = 0;
  frag_store.ocxo_ns           = 0;
  frag_store.pps_count         = 0;
  frag_store.isr_residual_dwt  = 0;
  frag_store.isr_residual_gnss = 0;
  frag_store.isr_residual_ocxo = 0;
  frag_store.local_dwt_snap    = 0;
  frag_store.valid             = false;

  dmb();
  frag_store.seq++;

  frag_public = {};
}

// ============================================================================
// Fragment access
// ============================================================================

const timebase_fragment_t* timebase_last_fragment(void) {
  return &frag_public;
}

// ============================================================================
// TIMEBASE_FRAGMENT subscription callback
// ============================================================================

void on_timebase_fragment(const Payload& payload) {
  const uint64_t gnss_ns    = payload.getUInt64("gnss_ns", 0);
  const uint64_t dwt_ns     = payload.getUInt64("dwt_ns", 0);
  const uint64_t dwt_cycles = payload.getUInt64("dwt_cycles", 0);
  const uint64_t ocxo_ns    = payload.getUInt64("ocxo_ns", 0);
  const uint32_t pps_count  = (uint32_t)payload.getUInt64("teensy_pps_count", 0);

  const int32_t isr_dwt  = payload.getInt("isr_residual_dwt", 0);
  const int32_t isr_gnss = payload.getInt("isr_residual_gnss", 0);
  const int32_t isr_ocxo = payload.getInt("isr_residual_ocxo", 0);

  const uint32_t local_dwt_snap = ARM_DWT_CYCCNT;
  const bool valid = (gnss_ns > 0 && dwt_ns > 0 && ocxo_ns > 0);

  // Sequence-protected write (odd = in-progress, even = stable)
  frag_store.seq++;
  dmb();

  frag_store.gnss_ns           = gnss_ns;
  frag_store.dwt_ns            = dwt_ns;
  frag_store.dwt_cycles        = dwt_cycles;
  frag_store.ocxo_ns           = ocxo_ns;
  frag_store.pps_count         = pps_count;
  frag_store.isr_residual_dwt  = isr_dwt;
  frag_store.isr_residual_gnss = isr_gnss;
  frag_store.isr_residual_ocxo = isr_ocxo;
  frag_store.local_dwt_snap    = local_dwt_snap;
  frag_store.valid             = valid;

  dmb();
  frag_store.seq++;

  // Unsynchronized public mirror for diagnostic/report reads
  frag_public.gnss_ns           = gnss_ns;
  frag_public.dwt_ns            = dwt_ns;
  frag_public.dwt_cycles        = dwt_cycles;
  frag_public.ocxo_ns           = ocxo_ns;
  frag_public.pps_count         = pps_count;
  frag_public.isr_residual_dwt  = isr_dwt;
  frag_public.isr_residual_gnss = isr_gnss;
  frag_public.isr_residual_ocxo = isr_ocxo;
  frag_public.valid             = valid;
}

// ============================================================================
// Initialization
// ============================================================================

void timebase_init(void) {
  frag_store  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false};
  frag_public = {};
}