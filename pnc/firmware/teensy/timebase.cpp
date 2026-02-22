#include "timebase.h"
#include "process_clocks.h"

#include <string.h>
#include <stdlib.h>

// ============================================================================
// Lifecycle
// ============================================================================

TimeBase::TimeBase()
  : clock_count_(0)
  , anchor_dwt_cycles_(0)
  , anchor_dwt_ns_(0)
  , anchor_gnss_ns_(0)
  , pps_count_(0)
  , valid_(false)
{
  memset(clocks_, 0, sizeof(clocks_));
}

// ============================================================================
// Internal helpers
// ============================================================================

const TimeBase::Clock* TimeBase::find_clock(const char* name) const {
  for (size_t i = 0; i < clock_count_; i++) {
    if (strncmp(clocks_[i].name, name, NAME_LEN) == 0) {
      return &clocks_[i];
    }
  }
  return nullptr;
}

TimeBase::Clock* TimeBase::find_clock_or_add(const char* name) {
  // Find existing
  for (size_t i = 0; i < clock_count_; i++) {
    if (strncmp(clocks_[i].name, name, NAME_LEN) == 0) {
      return &clocks_[i];
    }
  }

  // Add new
  if (clock_count_ >= MAX_CLOCKS) return nullptr;

  Clock* c = &clocks_[clock_count_++];
  strncpy(c->name, name, NAME_LEN - 1);
  c->name[NAME_LEN - 1] = '\0';
  c->ns_at_pps = 0;
  c->active = false;
  return c;
}

void TimeBase::set_clock(const char* name, uint64_t ns_at_pps) {
  Clock* c = find_clock_or_add(name);
  if (!c) return;

  c->ns_at_pps = ns_at_pps;
  c->active = (ns_at_pps > 0);
}

// ============================================================================
// set() — update from TIMEBASE record
// ============================================================================
//
// This is the critical path.  When a TIMEBASE publication arrives:
//
//   1) Capture DWT cycles RIGHT NOW — this is our interpolation anchor.
//      The closer this is to the actual PPS edge, the more accurate
//      our between-PPS projections will be.
//
//   2) Parse out all clock domain nanosecond values.
//
//   3) Store the DWT anchor for now() to interpolate from.
//
// The TIMEBASE payload contains string-encoded uint64_t values.
// We parse them via strtoull since Payload stores everything as
// strings internally.
//

static uint64_t get_u64(const Payload& p, const char* key) {
  const char* s = p.getString(key);
  if (!s || !*s) return 0;
  return strtoull(s, nullptr, 10);
}

void TimeBase::set(const Payload& p) {

  // ----------------------------------------------------------
  // 1) Capture DWT anchor immediately
  // ----------------------------------------------------------
  anchor_dwt_cycles_ = clocks_dwt_cycles_now();

  // ----------------------------------------------------------
  // 2) Extract clock domain values from TIMEBASE
  // ----------------------------------------------------------
  uint64_t dwt_cycles = get_u64(p, "teensy_dwt_cycles");
  uint64_t dwt_ns     = get_u64(p, "teensy_dwt_ns");
  uint64_t gnss_ns    = get_u64(p, "teensy_gnss_ns");
  uint64_t ocxo_ns    = get_u64(p, "teensy_ocxo_ns");
  uint64_t pi_ns      = get_u64(p, "pi_ns");
  uint64_t pps_count  = get_u64(p, "teensy_pps_count");

  // DWT and GNSS are mandatory — without them we can't interpolate
  if (dwt_ns == 0 || gnss_ns == 0) return;

  // ----------------------------------------------------------
  // 3) Populate clock domains
  // ----------------------------------------------------------
  set_clock("GNSS", gnss_ns);
  set_clock("DWT",  dwt_ns);
  set_clock("OCXO", ocxo_ns);
  set_clock("PI",   pi_ns);

  // ----------------------------------------------------------
  // 4) Store interpolation anchor
  //
  // anchor_dwt_cycles_ was captured in step 1 (live DWT read).
  // anchor_dwt_ns_ is the TIMEBASE's dwt_ns at the PPS edge.
  //
  // The difference between the live read and the TIMEBASE value
  // accounts for the propagation delay (fragment → Pi → TIMEBASE
  // → Teensy subscription → this call).  This is typically 2-6 ms.
  //
  // now() computes: elapsed = (dwt_now - anchor_dwt_cycles) * 5/3
  // and adds that (scaled by tau) to each domain's PPS snapshot.
  //
  // Because anchor_dwt_cycles_ is the live value and the clock
  // snapshots are PPS-aligned, the propagation delay is absorbed
  // correctly: the "elapsed since anchor" will be small (< 1s)
  // and the projection fills in the gap until the next TIMEBASE.
  //
  // IMPORTANT: We use the TIMEBASE's dwt_cycles (at PPS edge)
  // as the anchor, NOT the live value.  This way the interpolation
  // base is the PPS edge itself, and now() adds the full elapsed
  // time since that edge.
  // ----------------------------------------------------------
  anchor_dwt_cycles_ = dwt_cycles;
  anchor_dwt_ns_     = dwt_ns;
  anchor_gnss_ns_    = gnss_ns;
  pps_count_         = pps_count;
  valid_             = true;
}

// ============================================================================
// now() — current nanosecond count in a clock domain
// ============================================================================
//
// Algorithm:
//
//   1) Read DWT cycles right now
//   2) Compute elapsed DWT cycles since the PPS-edge anchor
//   3) Convert to elapsed DWT nanoseconds: cycles * 5 / 3
//   4) Scale to target domain: elapsed_target = elapsed_dwt * tau
//      where tau = target_ns_at_pps / dwt_ns_at_pps
//   5) Return: target_ns_at_pps + elapsed_target
//
// For DWT itself, tau = 1.0 (no scaling needed).
// For GNSS, tau ≈ 1.0000008 (DWT runs ~0.8 ppm slow).
// For PI, tau ≈ 1.000007 (Pi crystal runs ~7 ppm fast).
//
// The integer math preserves nanosecond precision without
// floating point:
//
//   elapsed_in_domain = (elapsed_dwt_ns * domain_ns) / dwt_ns
//
// This is exact when dwt_ns > 0 and the product doesn't overflow.
// At 64 bits, the product overflows after ~18.4 billion seconds
// of campaign time, which is ~584 years.  We're fine.
//

uint64_t TimeBase::now(const char* clock_name) const {

  if (!valid_) return 0;

  const Clock* c = find_clock(clock_name);
  if (!c || !c->active) return 0;

  // Read DWT right now
  uint64_t dwt_now = clocks_dwt_cycles_now();

  // Elapsed DWT cycles since PPS edge
  uint64_t elapsed_cycles = dwt_now - anchor_dwt_cycles_;

  // Convert to DWT nanoseconds: cycles * 5 / 3
  uint64_t elapsed_dwt_ns = (elapsed_cycles * 5ull) / 3ull;

  // Special case: DWT domain — no scaling needed
  if (strncmp(clock_name, "DWT", NAME_LEN) == 0) {
    return anchor_dwt_ns_ + elapsed_dwt_ns;
  }

  // Scale to target domain using integer ratio
  // elapsed_target = elapsed_dwt * (target_ns / dwt_ns)
  if (anchor_dwt_ns_ == 0) return c->ns_at_pps;

  uint64_t elapsed_target =
    (elapsed_dwt_ns * c->ns_at_pps) / anchor_dwt_ns_;

  return c->ns_at_pps + elapsed_target;
}

// ============================================================================
// convert() — cross-domain nanosecond conversion
// ============================================================================
//
// Given a nanosecond reading in one domain, compute the equivalent
// reading in another domain at the same physical instant.
//
// The relationship is:
//   (from_ns - from_snapshot) / tau_from = (to_ns - to_snapshot) / tau_to
//
// Simplified with integer math:
//   to_ns = to_snapshot + (from_ns - from_snapshot) * to_snapshot / from_snapshot
//
// This works because the snapshots are all taken at the same PPS edge,
// so their ratio IS the cumulative tau ratio.
//

uint64_t TimeBase::convert(
  const char* from_clock,
  const char* to_clock,
  uint64_t    from_ns
) const {

  if (!valid_) return 0;

  const Clock* from = find_clock(from_clock);
  const Clock* to   = find_clock(to_clock);

  if (!from || !from->active || !to || !to->active) return 0;
  if (from->ns_at_pps == 0) return to->ns_at_pps;

  // Delta in the source domain since the PPS snapshot
  int64_t delta_from = (int64_t)from_ns - (int64_t)from->ns_at_pps;

  // Scale to target domain
  int64_t delta_to = (delta_from * (int64_t)to->ns_at_pps) /
                     (int64_t)from->ns_at_pps;

  return (uint64_t)((int64_t)to->ns_at_pps + delta_to);
}

// ============================================================================
// tau() — rate ratio vs GNSS
// ============================================================================

double TimeBase::tau(const char* clock_name) const {

  if (!valid_ || anchor_gnss_ns_ == 0) return 0.0;

  // GNSS is the reference — tau is always 1.0
  if (strncmp(clock_name, "GNSS", NAME_LEN) == 0) return 1.0;

  const Clock* c = find_clock(clock_name);
  if (!c || !c->active) return 0.0;

  return (double)c->ns_at_pps / (double)anchor_gnss_ns_;
}

// ============================================================================
// Health
// ============================================================================

bool TimeBase::valid() const {
  return valid_;
}

uint32_t TimeBase::age_ms() const {
  if (!valid_) return UINT32_MAX;

  uint64_t dwt_now = clocks_dwt_cycles_now();
  uint64_t elapsed_cycles = dwt_now - anchor_dwt_cycles_;
  uint64_t elapsed_ns = (elapsed_cycles * 5ull) / 3ull;

  return (uint32_t)(elapsed_ns / 1000000ull);
}

uint64_t TimeBase::pps_count() const {
  return pps_count_;
}