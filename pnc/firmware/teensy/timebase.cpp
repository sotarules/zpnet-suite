// ============================================================================
// timebase.cpp — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// PPS-fragment anchored Timebase.
//
// The authoritative TIMEBASE_FRAGMENT provides PPS-edge totals once per
// second.  The fragment carries:
//
//   dwt_cycles         — synthetic 64-bit DWT cycle count at PPS edge
//   dwt_cyccnt_at_pps  — raw DWT_CYCCNT from the spin loop pre-edge shadow
//   dwt_cycles_per_pps — Welford mean of DWT cycles per GNSS second
//
// Interpolation within the current second:
//
//   dwt_cycles_now     = fragment.dwt_cycles + (DWT_CYCCNT - dwt_cyccnt_at_pps)
//   cycles_into_second = dwt_cycles_now - fragment.dwt_cycles
//   ns_into_second     = cycles_into_second × 1,000,000,000 / dwt_cycles_per_pps
//   gnss_now           = fragment.gnss_ns + ns_into_second
//
// The product cycles_into_second × 1e9 fits in uint64 (max ~1.008e18).
// No floating point.  No mul_div_u64 double fallback.  No rounding bias.
//
// timebase_now_dwt_cycles() returns the synthetic campaign cycle count
// at any instant — for code that needs the raw cycle count before
// nanosecond conversion.
//
// The DWT runs at 1008 MHz.  The 125/126 ratio converts DWT cycles to
// nominal nanoseconds and is used only for the DWT domain.
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
static constexpr uint64_t TIMEBASE_MAX_FRAGMENT_AGE_NS = 3000000000ULL;

// ============================================================================
// GPS epoch and leap seconds
// ============================================================================

static constexpr uint64_t GPS_EPOCH_UNIX = 315964800ULL;

static const uint8_t DAYS_IN_MONTH[2][12] = {
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
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
  volatile uint32_t dwt_cyccnt_at_pps;
  volatile uint64_t dwt_cycles_per_pps;
  volatile uint32_t seq;
  volatile bool     valid;
};

static fragment_store_t frag_store = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false};

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
// Fragment copy (stack-local, torn-read-safe)
// ============================================================================

struct fragment_copy_t {
  uint64_t gnss_ns;
  uint64_t dwt_ns;
  uint64_t dwt_cycles;
  uint64_t ocxo_ns;
  uint32_t pps_count;
  int32_t  isr_residual_dwt;
  int32_t  isr_residual_gnss;
  int32_t  isr_residual_ocxo;
  uint32_t dwt_cyccnt_at_pps;
  uint64_t dwt_cycles_per_pps;
  bool     valid;
  bool     ok;
};

// ============================================================================
// Fragment read with torn-read protection
// ============================================================================

static fragment_copy_t read_fragment(void) {
  fragment_copy_t f = {};
  f.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t s1 = frag_store.seq;
    dmb();

    f.gnss_ns           = frag_store.gnss_ns;
    f.dwt_ns            = frag_store.dwt_ns;
    f.dwt_cycles        = frag_store.dwt_cycles;
    f.ocxo_ns           = frag_store.ocxo_ns;
    f.pps_count         = frag_store.pps_count;
    f.isr_residual_dwt  = frag_store.isr_residual_dwt;
    f.isr_residual_gnss = frag_store.isr_residual_gnss;
    f.isr_residual_ocxo = frag_store.isr_residual_ocxo;
    f.dwt_cyccnt_at_pps  = frag_store.dwt_cyccnt_at_pps;
    f.dwt_cycles_per_pps = frag_store.dwt_cycles_per_pps;
    f.valid              = frag_store.valid;

    dmb();
    uint32_t s2 = frag_store.seq;

    if (s1 == s2 && (s1 & 1u) == 0u) {
      f.ok = true;
      return f;
    }
  }

  return f;
}

// ============================================================================
// Domain totals from fragment
// ============================================================================

static bool fragment_domain_total_ns(
  const fragment_copy_t& f,
  timebase_domain_t      domain,
  uint64_t&              out_ns
) {
  switch (domain) {
    case timebase_domain_t::GNSS:
      out_ns = f.gnss_ns;
      return f.gnss_ns > 0;

    case timebase_domain_t::DWT:
      out_ns = f.dwt_ns;
      return f.dwt_ns > 0;

    case timebase_domain_t::OCXO:
      out_ns = f.ocxo_ns;
      return f.ocxo_ns > 0;
  }

  out_ns = 0;
  return false;
}

// ============================================================================
// Synthetic campaign DWT cycle count — "how many DWT cycles since PPS 0?"
// ============================================================================

static constexpr uint64_t NS_PER_SECOND = 1000000000ULL;

static bool current_dwt_cycles(const fragment_copy_t& f, uint64_t& out_cycles) {
  uint32_t dwt_now     = ARM_DWT_CYCCNT;
  uint32_t dwt_elapsed = dwt_now - f.dwt_cyccnt_at_pps;

  // Sanity: elapsed should be less than ~3 seconds of cycles
  if ((uint64_t)dwt_elapsed * NS_PER_SECOND / (f.dwt_cycles_per_pps ? f.dwt_cycles_per_pps : 1008000000ULL) > TIMEBASE_MAX_FRAGMENT_AGE_NS) {
    return false;
  }

  out_cycles = f.dwt_cycles + (uint64_t)dwt_elapsed;
  return true;
}

// ============================================================================
// Public API — synthetic DWT cycle count
// ============================================================================

int64_t timebase_now_dwt_cycles(void) {
  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid) return -1;
  if (f.dwt_cycles_per_pps == 0) return -1;

  uint64_t cycles = 0;
  if (!current_dwt_cycles(f, cycles)) return -1;
  return (int64_t)cycles;
}

// ============================================================================
// Forward projection — pure integer, no floating point
// ============================================================================
//
// The math:
//   dwt_cycles_now        = fragment.dwt_cycles + (DWT_CYCCNT - dwt_cyccnt_at_pps)
//   cycles_into_second    = dwt_cycles_now - fragment.dwt_cycles
//                         = DWT_CYCCNT - dwt_cyccnt_at_pps
//   ns_into_second        = cycles_into_second × 1,000,000,000 / dwt_cycles_per_pps
//   gnss_now              = fragment.gnss_ns + ns_into_second

static bool current_gnss_now(uint64_t& out_gnss_ns) {
  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid) return false;
  if (f.dwt_cycles_per_pps == 0) return false;

  uint64_t dwt_cycles_now = 0;
  if (!current_dwt_cycles(f, dwt_cycles_now)) return false;

  // Cycles elapsed since the PPS edge that anchors this fragment
  uint64_t cycles_into_second = dwt_cycles_now - f.dwt_cycles;

  // Convert to GNSS nanoseconds using the Welford mean rate.
  // Max product: ~1,008,000,000 × 1,000,000,000 = 1.008e18 < UINT64_MAX
  uint64_t ns_into_second =
    cycles_into_second * NS_PER_SECOND / f.dwt_cycles_per_pps;

  if (ns_into_second > TIMEBASE_MAX_FRAGMENT_AGE_NS) {
    return false;
  }

  out_gnss_ns = f.gnss_ns + ns_into_second;
  return true;
}

// ============================================================================
// Forward projection — DWT domain (nominal 125/126, no tau correction)
// ============================================================================

static bool current_dwt_now(uint64_t& out_dwt_ns) {
  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid) return false;

  uint32_t dwt_now     = ARM_DWT_CYCCNT;
  uint32_t dwt_elapsed = dwt_now - f.dwt_cyccnt_at_pps;

  uint64_t elapsed_dwt_ns =
    mul_div_u64((uint64_t)dwt_elapsed, DWT_TO_NS_NUM, DWT_TO_NS_DEN);

  if (elapsed_dwt_ns > TIMEBASE_MAX_FRAGMENT_AGE_NS) {
    return false;
  }

  out_dwt_ns = f.dwt_ns + elapsed_dwt_ns;
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

  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid || f.gnss_ns == 0) {
    return false;
  }

  uint64_t from_total_ns = 0;
  uint64_t to_total_ns   = 0;

  if (!fragment_domain_total_ns(f, from_domain, from_total_ns)) return false;
  if (!fragment_domain_total_ns(f, to_domain,   to_total_ns))   return false;

  // Normalize to GNSS first
  uint64_t gnss_ns = 0;
  if (from_domain == timebase_domain_t::GNSS) {
    gnss_ns = value_ns;
  } else {
    gnss_ns = mul_div_u64(value_ns, f.gnss_ns, from_total_ns);
  }

  // Then convert to target domain
  if (to_domain == timebase_domain_t::GNSS) {
    out_ns = gnss_ns;
  } else {
    out_ns = mul_div_u64(gnss_ns, to_total_ns, f.gnss_ns);
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

  uint64_t total_secs = gnss_ns / 1000000000ULL;
  uint32_t sub_ns     = (uint32_t)(gnss_ns % 1000000000ULL);

  uint64_t unix_secs = GPS_EPOCH_UNIX + total_secs;

  uint32_t days    = (uint32_t)(unix_secs / 86400ULL);
  uint32_t day_sec = (uint32_t)(unix_secs % 86400ULL);

  uint32_t hour = day_sec / 3600;
  uint32_t min  = (day_sec % 3600) / 60;
  uint32_t sec  = day_sec % 60;

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
  month += 1;
  days  += 1;

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
  fragment_copy_t f = read_fragment();
  if (!f.ok) return 0;
  return f.pps_count;
}

// ============================================================================
// Validity
// ============================================================================

bool timebase_valid(void) {
  uint64_t gnss_now_ns = 0;
  return current_gnss_now(gnss_now_ns);
}

bool timebase_conversion_valid(void) {
  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid) return false;
  if (f.gnss_ns == 0) return false;
  if (f.dwt_ns  == 0) return false;
  if (f.ocxo_ns == 0) return false;
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
  frag_store.dwt_cyccnt_at_pps  = 0;
  frag_store.dwt_cycles_per_pps = 0;
  frag_store.valid              = false;

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

  // The interpolation anchor is the raw DWT_CYCCNT register value
  // captured by the spin loop just before the PPS edge.  This is the
  // actual hardware register value — NOT the 64-bit accumulator.
  // When current_gnss_now() later computes (DWT_CYCCNT - anchor),
  // the result is elapsed cycles since the PPS edge.
  const uint32_t dwt_cyccnt_at_pps    = payload.getUInt("dwt_cyccnt_at_pps", 0);
  const uint64_t dwt_cycles_per_pps   = payload.getUInt64("dwt_cycles_per_pps", 0);
  const bool valid = (gnss_ns > 0 && dwt_ns > 0 && ocxo_ns > 0 && dwt_cycles_per_pps > 0);

  // Sequence-protected write (odd = in-progress, even = stable)
  frag_store.seq++;
  dmb();

  frag_store.gnss_ns            = gnss_ns;
  frag_store.dwt_ns             = dwt_ns;
  frag_store.dwt_cycles         = dwt_cycles;
  frag_store.ocxo_ns            = ocxo_ns;
  frag_store.pps_count          = pps_count;
  frag_store.isr_residual_dwt   = isr_dwt;
  frag_store.isr_residual_gnss  = isr_gnss;
  frag_store.isr_residual_ocxo  = isr_ocxo;
  frag_store.dwt_cyccnt_at_pps  = dwt_cyccnt_at_pps;
  frag_store.dwt_cycles_per_pps = dwt_cycles_per_pps;
  frag_store.valid              = valid;

  dmb();
  frag_store.seq++;

  // Unsynchronized public mirror for diagnostic/report reads
  frag_public.gnss_ns            = gnss_ns;
  frag_public.dwt_ns             = dwt_ns;
  frag_public.dwt_cycles         = dwt_cycles;
  frag_public.ocxo_ns            = ocxo_ns;
  frag_public.pps_count          = pps_count;
  frag_public.isr_residual_dwt   = isr_dwt;
  frag_public.isr_residual_gnss  = isr_gnss;
  frag_public.isr_residual_ocxo  = isr_ocxo;
  frag_public.dwt_cycles_per_pps = dwt_cycles_per_pps;
  frag_public.dwt_cyccnt_at_pps  = dwt_cyccnt_at_pps;
  frag_public.valid              = valid;
}

// ============================================================================
// Initialization
// ============================================================================

void timebase_init(void) {
  frag_store  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false};
  frag_public = {};
}