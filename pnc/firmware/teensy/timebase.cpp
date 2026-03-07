// ============================================================================
// timebase.cpp — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// See timebase.h for architecture and API documentation.
//
// Implementation highlights
// -------------------------
//
// 1. GNSS interpolation
//
//    The TIMEPULSE anchor provides:
//
//      • anchor.gnss_ns
//      • anchor.dwt_snap
//
//    To answer now(GNSS), we read ARM_DWT_CYCCNT, compute:
//
//      dwt_elapsed = dwt_now - anchor.dwt_snap
//
//    and convert DWT cycles to GNSS nanoseconds using the exact ratio:
//
//      1 DWT cycle = 125 / 126 GNSS nanoseconds
//
//    Thus:
//
//      gnss_now = anchor.gnss_ns + dwt_elapsed * 125 / 126
//
// 2. Domain conversion
//
//    The latest TIMEBASE_FRAGMENT provides authoritative PPS-edge totals:
//
//      gnss_ns, dwt_ns, ocxo_ns
//
//    These totals define the inter-domain ratios. Conversion is performed by
//    scaling through GNSS.
//
// 3. Torn-read protection
//
//    Both the anchor and fragment are multiword structures. Writers wrap each
//    update with a sequence counter:
//
//      seq++   // odd = write in progress
//      ...fields...
//      seq++   // even = write complete
//
//    Readers retry until they observe a stable even sequence number.
//
// 4. Anchor freshness
//
//    A stale anchor is rejected. This prevents Timebase from reporting stale
//    "now" values after campaign stop or stalled TIMEPULSE updates.
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

// DWT-to-GNSS conversion:
//   DWT @ 1.008 GHz
//   GNSS ticks expressed in nanoseconds
//
// 1 DWT cycle = 125 / 126 GNSS nanoseconds exactly.
static constexpr uint32_t DWT_TO_GNSS_NUM = 125;
static constexpr uint32_t DWT_TO_GNSS_DEN = 126;

// TIMEPULSE nominal period = 100 µs = 100,000 ns.
// Anchor freshness guard: if the anchor is older than this threshold,
// Timebase refuses to answer "now". This prevents stale clock narration
// after campaign stop or interrupted TIMEPULSE updates.
//
// Generous margin: 50 TIMEPULSE intervals = 5 ms.
static constexpr uint64_t TIMEBASE_MAX_ANCHOR_AGE_NS = 5000000ULL;

// ============================================================================
// Anchor state (written by TIMEPULSE ISR, read by anyone)
// ============================================================================

struct anchor_t {
  volatile uint64_t gnss_ns;
  volatile uint32_t dwt_snap;
  volatile uint32_t seq;
  volatile bool     valid;
};

static anchor_t anchor = {0, 0, 0, false};

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
  volatile uint32_t seq;
  volatile bool     valid;
};

static fragment_store_t frag_store = {0, 0, 0, 0, 0, 0, 0, 0, 0, false};

// Public mirror for diagnostics / reports
static timebase_fragment_t frag_public = {};

// ============================================================================
// Safe multiply/divide helper
// ============================================================================
//
// The naive form:
//
//   (value * num) / den
//
// overflows 64-bit for absolute nanosecond campaign totals even though the
// final result is perfectly reasonable.  We therefore compute through a ratio:
//
//   value * (num / den)
//
// using double precision when necessary.
//
// For current ZPNet time scales (absolute nanosecond totals in the low 10^12
// to 10^13 range), double precision is more than adequate and avoids wrap.
// ============================================================================

static inline uint64_t mul_div_u64(
  uint64_t value,
  uint64_t num,
  uint64_t den
) {
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
// Snapshot structs
// ============================================================================

struct anchor_snapshot_t {
  uint64_t gnss_ns;
  uint32_t dwt_snap;
  bool     valid;
  bool     ok;
};

struct fragment_snapshot_t {
  uint64_t gnss_ns;
  uint64_t dwt_ns;
  uint64_t dwt_cycles;
  uint64_t ocxo_ns;
  uint32_t pps_count;
  int32_t  isr_residual_dwt;
  int32_t  isr_residual_gnss;
  int32_t  isr_residual_ocxo;
  bool     valid;
  bool     ok;
};

// ============================================================================
// Anchor read with torn-read protection
// ============================================================================

static anchor_snapshot_t read_anchor(void) {
  anchor_snapshot_t snap = {};
  snap.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t s1 = anchor.seq;
    dmb();

    snap.gnss_ns  = anchor.gnss_ns;
    snap.dwt_snap = anchor.dwt_snap;
    snap.valid    = anchor.valid;

    dmb();
    uint32_t s2 = anchor.seq;

    if (s1 == s2 && (s1 & 1u) == 0u) {
      snap.ok = true;
      return snap;
    }
  }

  return snap;
}

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
// Fresh GNSS "now" from TIMEPULSE anchor
// ============================================================================

static bool gnss_now_from_anchor(uint64_t& out_gnss_ns) {
  anchor_snapshot_t snap = read_anchor();
  if (!snap.ok || !snap.valid) return false;

  uint32_t dwt_now     = ARM_DWT_CYCCNT;
  uint32_t dwt_elapsed = dwt_now - snap.dwt_snap;

  uint64_t gnss_elapsed_ns =
    mul_div_u64((uint64_t)dwt_elapsed, DWT_TO_GNSS_NUM, DWT_TO_GNSS_DEN);

  if (gnss_elapsed_ns > TIMEBASE_MAX_ANCHOR_AGE_NS) {
    return false;
  }

  out_gnss_ns = snap.gnss_ns + gnss_elapsed_ns;
  return true;
}

// ============================================================================
// Domain conversion through GNSS
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

  uint64_t gnss_ns;
  if (from_domain == timebase_domain_t::GNSS) {
    gnss_ns = value_ns;
  } else {
    gnss_ns = mul_div_u64(value_ns, snap.gnss_ns, from_total_ns);
  }

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
  uint64_t gnss_now_ns = 0;
  if (!gnss_now_from_anchor(gnss_now_ns)) return -1;

  if (domain == timebase_domain_t::GNSS) {
    return (int64_t)gnss_now_ns;
  }

  uint64_t converted = 0;
  if (!convert_via_fragment(
        gnss_now_ns,
        timebase_domain_t::GNSS,
        domain,
        converted)) {
    return -1;
  }

  return (int64_t)converted;
}

int64_t timebase_now_gnss_ns(void) {
  uint64_t gnss_now_ns = 0;
  if (!gnss_now_from_anchor(gnss_now_ns)) return -1;
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
// Validity
// ============================================================================

bool timebase_valid(void) {
  uint64_t gnss_now_ns = 0;
  return gnss_now_from_anchor(gnss_now_ns);
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
// TIMEPULSE anchor update (ISR context only)
// ============================================================================

void timebase_update_anchor(uint64_t gnss_ns, uint32_t dwt_snap) {
  anchor.seq++;
  dmb();

  anchor.gnss_ns  = gnss_ns;
  anchor.dwt_snap = dwt_snap;
  anchor.valid    = true;

  dmb();
  anchor.seq++;
}

// ============================================================================
// Invalidation
// ============================================================================

void timebase_invalidate(void) {
  anchor.seq++;
  dmb();
  anchor.gnss_ns  = 0;
  anchor.dwt_snap = 0;
  anchor.valid    = false;
  dmb();
  anchor.seq++;

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
  frag_store.valid             = (gnss_ns > 0 && dwt_ns > 0 && ocxo_ns > 0);

  dmb();
  frag_store.seq++;

  frag_public.gnss_ns           = gnss_ns;
  frag_public.dwt_ns            = dwt_ns;
  frag_public.dwt_cycles        = dwt_cycles;
  frag_public.ocxo_ns           = ocxo_ns;
  frag_public.pps_count         = pps_count;
  frag_public.isr_residual_dwt  = isr_dwt;
  frag_public.isr_residual_gnss = isr_gnss;
  frag_public.isr_residual_ocxo = isr_ocxo;
  frag_public.valid             = (gnss_ns > 0 && dwt_ns > 0 && ocxo_ns > 0);
}

// ============================================================================
// Initialization
// ============================================================================

void timebase_init(void) {
  anchor      = {0, 0, 0, false};
  frag_store  = {0, 0, 0, 0, 0, 0, 0, 0, 0, false};
  frag_public = {};
}