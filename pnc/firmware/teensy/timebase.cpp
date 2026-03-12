// ============================================================================
// timebase.cpp — ZPNet Timebase Singleton (Teensy) — v11 Dual OCXO
// ============================================================================
//
// PPS-fragment anchored Timebase.
//
// v11: Dual OCXO.  Fragment store and conversion paths support four
//      clock domains: GNSS, DWT, OCXO1, OCXO2.
//
// Rounding policy (v8, unchanged):
//   cycles→ns uses round-to-nearest: ns = (cycles * 1e9 + denom/2) / denom
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
  volatile uint64_t ocxo1_ns;
  volatile uint64_t ocxo2_ns;
  volatile uint32_t pps_count;
  volatile int32_t  isr_residual_dwt;
  volatile int32_t  isr_residual_gnss;
  volatile int32_t  isr_residual_ocxo1;
  volatile int32_t  isr_residual_ocxo2;
  volatile uint32_t dwt_cyccnt_at_pps;
  volatile uint64_t dwt_cycles_per_pps;
  volatile uint32_t gpt2_at_pps;
  volatile uint32_t seq;
  volatile bool     valid;
};

static fragment_store_t frag_store = {};

static timebase_fragment_t frag_public = {};

// ============================================================================
// Safe multiply/divide helper
// ============================================================================

static inline uint64_t mul_div_u64(uint64_t value, uint64_t num, uint64_t den) {
  if (den == 0) return 0;
  if (num != 0 && value <= (UINT64_MAX / num)) {
    return (value * num) / den;
  }
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
  uint64_t ocxo1_ns;
  uint64_t ocxo2_ns;
  uint32_t pps_count;
  int32_t  isr_residual_dwt;
  int32_t  isr_residual_gnss;
  int32_t  isr_residual_ocxo1;
  int32_t  isr_residual_ocxo2;
  uint32_t dwt_cyccnt_at_pps;
  uint64_t dwt_cycles_per_pps;
  uint32_t gpt2_at_pps;
  bool     valid;
  bool     ok;
};

static fragment_copy_t read_fragment(void) {
  fragment_copy_t f = {};
  f.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t s1 = frag_store.seq;
    dmb();

    f.gnss_ns            = frag_store.gnss_ns;
    f.dwt_ns             = frag_store.dwt_ns;
    f.dwt_cycles         = frag_store.dwt_cycles;
    f.ocxo1_ns           = frag_store.ocxo1_ns;
    f.ocxo2_ns           = frag_store.ocxo2_ns;
    f.pps_count          = frag_store.pps_count;
    f.isr_residual_dwt   = frag_store.isr_residual_dwt;
    f.isr_residual_gnss  = frag_store.isr_residual_gnss;
    f.isr_residual_ocxo1 = frag_store.isr_residual_ocxo1;
    f.isr_residual_ocxo2 = frag_store.isr_residual_ocxo2;
    f.dwt_cyccnt_at_pps  = frag_store.dwt_cyccnt_at_pps;
    f.dwt_cycles_per_pps = frag_store.dwt_cycles_per_pps;
    f.gpt2_at_pps        = frag_store.gpt2_at_pps;
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

    case timebase_domain_t::OCXO1:
      out_ns = f.ocxo1_ns;
      return f.ocxo1_ns > 0;

    case timebase_domain_t::OCXO2:
      out_ns = f.ocxo2_ns;
      return f.ocxo2_ns > 0;
  }

  out_ns = 0;
  return false;
}

// ============================================================================
// Synthetic campaign DWT cycle count
// ============================================================================

static constexpr uint64_t NS_PER_SECOND = 1000000000ULL;

static bool current_dwt_cycles(const fragment_copy_t& f, uint64_t& out_cycles) {
  uint32_t dwt_now     = ARM_DWT_CYCCNT;
  uint32_t dwt_elapsed = dwt_now - f.dwt_cyccnt_at_pps;

  if ((uint64_t)dwt_elapsed * NS_PER_SECOND / (f.dwt_cycles_per_pps ? f.dwt_cycles_per_pps : 1008000000ULL) > TIMEBASE_MAX_FRAGMENT_AGE_NS) {
    return false;
  }

  out_cycles = f.dwt_cycles + (uint64_t)dwt_elapsed;
  return true;
}

int64_t timebase_now_dwt_cycles(void) {
  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid) return -1;
  if (f.dwt_cycles_per_pps == 0) return -1;
  uint64_t cycles = 0;
  if (!current_dwt_cycles(f, cycles)) return -1;
  return (int64_t)cycles;
}

// ============================================================================
// Forward projection — GNSS (round-to-nearest)
// ============================================================================

static bool current_gnss_now(uint64_t& out_gnss_ns) {
  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid) return false;
  if (f.dwt_cycles_per_pps == 0) return false;

  uint64_t dwt_cycles_now = 0;
  if (!current_dwt_cycles(f, dwt_cycles_now)) return false;

  uint64_t cycles_into_second = dwt_cycles_now - f.dwt_cycles;

  uint64_t ns_into_second =
    (cycles_into_second * NS_PER_SECOND + f.dwt_cycles_per_pps / 2)
    / f.dwt_cycles_per_pps;

  if (ns_into_second > TIMEBASE_MAX_FRAGMENT_AGE_NS) return false;

  out_gnss_ns = f.gnss_ns + ns_into_second;
  return true;
}

// ============================================================================
// Forward projection — DWT domain (nominal 125/126)
// ============================================================================

static bool current_dwt_now(uint64_t& out_dwt_ns) {
  fragment_copy_t f = read_fragment();
  if (!f.ok || !f.valid) return false;

  uint32_t dwt_now     = ARM_DWT_CYCCNT;
  uint32_t dwt_elapsed = dwt_now - f.dwt_cyccnt_at_pps;

  uint64_t elapsed_dwt_ns =
    mul_div_u64((uint64_t)dwt_elapsed, DWT_TO_NS_NUM, DWT_TO_NS_DEN);

  if (elapsed_dwt_ns > TIMEBASE_MAX_FRAGMENT_AGE_NS) return false;

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
  if (!f.ok || !f.valid || f.gnss_ns == 0) return false;

  uint64_t from_total_ns = 0;
  uint64_t to_total_ns   = 0;

  if (!fragment_domain_total_ns(f, from_domain, from_total_ns)) return false;
  if (!fragment_domain_total_ns(f, to_domain,   to_total_ns))   return false;

  uint64_t gnss_ns = 0;
  if (from_domain == timebase_domain_t::GNSS) {
    gnss_ns = value_ns;
  } else {
    gnss_ns = mul_div_u64(value_ns, f.gnss_ns, from_total_ns);
  }

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

    case timebase_domain_t::OCXO1: {
      uint64_t gnss_now_ns = 0;
      uint64_t ocxo_now_ns = 0;
      if (!current_gnss_now(gnss_now_ns)) return -1;
      if (!convert_via_fragment(
            gnss_now_ns,
            timebase_domain_t::GNSS,
            timebase_domain_t::OCXO1,
            ocxo_now_ns)) {
        return -1;
      }
      return (int64_t)ocxo_now_ns;
    }

    case timebase_domain_t::OCXO2: {
      uint64_t gnss_now_ns = 0;
      uint64_t ocxo_now_ns = 0;
      if (!current_gnss_now(gnss_now_ns)) return -1;
      if (!convert_via_fragment(
            gnss_now_ns,
            timebase_domain_t::GNSS,
            timebase_domain_t::OCXO2,
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
// Deterministic overload — GNSS ns from pre-captured DWT + fragment
// ============================================================================

int64_t timebase_gnss_ns_from_dwt(
  uint32_t dwt_cyccnt,
  uint64_t frag_gnss_ns,
  uint32_t frag_dwt_cyccnt_at_pps,
  uint32_t frag_dwt_cycles_per_pps
) {
  if (frag_dwt_cycles_per_pps == 0) return -1;
  if (frag_gnss_ns == 0) return -1;

  uint32_t elapsed = dwt_cyccnt - frag_dwt_cyccnt_at_pps;

  uint64_t ns_into_second =
    ((uint64_t)elapsed * NS_PER_SECOND + (uint64_t)frag_dwt_cycles_per_pps / 2)
    / (uint64_t)frag_dwt_cycles_per_pps;

  if (ns_into_second > TIMEBASE_MAX_FRAGMENT_AGE_NS) return -1;

  return (int64_t)(frag_gnss_ns + ns_into_second);
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
// PPS count
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
  if (f.gnss_ns  == 0) return false;
  if (f.dwt_ns   == 0) return false;
  if (f.ocxo1_ns == 0) return false;
  if (f.ocxo2_ns == 0) return false;
  return true;
}

// ============================================================================
// Invalidation
// ============================================================================

void timebase_invalidate(void) {
  frag_store.seq++;
  dmb();

  frag_store.gnss_ns            = 0;
  frag_store.dwt_ns             = 0;
  frag_store.dwt_cycles         = 0;
  frag_store.ocxo1_ns           = 0;
  frag_store.ocxo2_ns           = 0;
  frag_store.pps_count          = 0;
  frag_store.isr_residual_dwt   = 0;
  frag_store.isr_residual_gnss  = 0;
  frag_store.isr_residual_ocxo1 = 0;
  frag_store.isr_residual_ocxo2 = 0;
  frag_store.dwt_cyccnt_at_pps  = 0;
  frag_store.dwt_cycles_per_pps = 0;
  frag_store.gpt2_at_pps        = 0;
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
  const uint64_t ocxo1_ns   = payload.getUInt64("ocxo1_ns", 0);
  const uint64_t ocxo2_ns   = payload.getUInt64("ocxo2_ns", 0);
  const uint32_t pps_count  = (uint32_t)payload.getUInt64("teensy_pps_count", 0);

  const int32_t isr_dwt   = payload.getInt("isr_residual_dwt", 0);
  const int32_t isr_gnss  = payload.getInt("isr_residual_gnss", 0);
  const int32_t isr_ocxo1 = payload.getInt("isr_residual_ocxo1", 0);
  const int32_t isr_ocxo2 = payload.getInt("isr_residual_ocxo2", 0);

  const uint32_t dwt_cyccnt_at_pps    = payload.getUInt("dwt_cyccnt_at_pps", 0);
  const uint64_t dwt_cycles_per_pps   = payload.getUInt64("dwt_cycles_per_pps", 0);
  const uint32_t gpt2_at_pps          = payload.getUInt("gpt2_at_pps", 0);
  const bool valid = (gnss_ns > 0 && dwt_ns > 0 && ocxo1_ns > 0 && dwt_cycles_per_pps > 0);

  frag_store.seq++;
  dmb();

  frag_store.gnss_ns            = gnss_ns;
  frag_store.dwt_ns             = dwt_ns;
  frag_store.dwt_cycles         = dwt_cycles;
  frag_store.ocxo1_ns           = ocxo1_ns;
  frag_store.ocxo2_ns           = ocxo2_ns;
  frag_store.pps_count          = pps_count;
  frag_store.isr_residual_dwt   = isr_dwt;
  frag_store.isr_residual_gnss  = isr_gnss;
  frag_store.isr_residual_ocxo1 = isr_ocxo1;
  frag_store.isr_residual_ocxo2 = isr_ocxo2;
  frag_store.dwt_cyccnt_at_pps  = dwt_cyccnt_at_pps;
  frag_store.dwt_cycles_per_pps = dwt_cycles_per_pps;
  frag_store.gpt2_at_pps        = gpt2_at_pps;
  frag_store.valid              = valid;

  dmb();
  frag_store.seq++;

  frag_public.gnss_ns            = gnss_ns;
  frag_public.dwt_ns             = dwt_ns;
  frag_public.dwt_cycles         = dwt_cycles;
  frag_public.ocxo1_ns           = ocxo1_ns;
  frag_public.ocxo2_ns           = ocxo2_ns;
  frag_public.pps_count          = pps_count;
  frag_public.isr_residual_dwt   = isr_dwt;
  frag_public.isr_residual_gnss  = isr_gnss;
  frag_public.isr_residual_ocxo1 = isr_ocxo1;
  frag_public.isr_residual_ocxo2 = isr_ocxo2;
  frag_public.dwt_cycles_per_pps = dwt_cycles_per_pps;
  frag_public.dwt_cyccnt_at_pps  = dwt_cyccnt_at_pps;
  frag_public.gpt2_at_pps        = gpt2_at_pps;
  frag_public.valid              = valid;
}

// ============================================================================
// Initialization
// ============================================================================

void timebase_init(void) {
  frag_store  = {};
  frag_public = {};
}