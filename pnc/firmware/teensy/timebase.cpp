// ============================================================================
// timebase.cpp — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// PPS-fragment anchored Timebase.
//
// Architecture
// ------------
//
// The authoritative TIMEBASE_FRAGMENT provides PPS-edge totals:
//
//   • gnss_ns
//   • dwt_ns
//   • dwt_cycles
//   • ocxo_ns
//
// When the fragment is received in scheduled context, Timebase captures
// the local ARM_DWT_CYCCNT at fragment receipt.  "Now" is then answered by
// projecting forward from that fragment using local DWT elapsed time.
//
// GNSS projection:
//   gnss_now = fragment.gnss_ns + elapsed_dwt_cycles * 125 / 126
//
// DWT projection:
//   dwt_now = fragment.dwt_ns + elapsed_dwt_cycles * 125 / 126
//
// OCXO projection:
//   first compute gnss_now, then convert through the fragment ratio
//   GNSS → OCXO.
//
// This design eliminates all reliance on TIMEPULSE / 10 kHz anchor updates.
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

  if (num != 0 && value <= (UINT64_MAX / num)) {
    return (value * num) / den;
  }

  double ratio  = (double)num / (double)den;
  double scaled = (double)value * ratio;

  if (scaled <= 0.0) return 0;
  return (uint64_t)(scaled + 0.5);
}

// ============================================================================
// Snapshot struct
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

  uint64_t gnss_ns = 0;
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