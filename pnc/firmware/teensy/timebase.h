// ============================================================================
// timebase.h — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// Timebase is the canonical clock-domain translation layer for the Teensy.
//
// It answers two classes of questions:
//
//   1) "What time is it right now in domain X?"
//   2) "What is the corresponding nanosecond value in domain B for a value
//       expressed in domain A?"
//
// Supported domains:
//
//   • GNSS — GNSS nanoseconds
//   • DWT  — DWT nanoseconds
//   • OCXO — OCXO nanoseconds
//
// Architecture
// ------------
//
// 1. TIMEPULSE anchor (10 KHz, written by ISR)
//    Every 100 µs, the TIMEPULSE ISR publishes an anchor:
//
//      anchor = { gnss_ns, dwt_snap }
//
//    where:
//      • gnss_ns  = quantized GNSS nanoseconds at the TIMEPULSE rising edge
//      • dwt_snap = DWT_CYCCNT captured at that same edge
//
//    This anchor is the high-resolution interpolation base.
//
// 2. TIMEBASE_FRAGMENT (1 Hz, written by pub/sub callback)
//    Once per second, CLOCKS publishes authoritative PPS-edge totals:
//
//      • gnss_ns
//      • dwt_ns
//      • dwt_cycles
//      • ocxo_ns
//      • pps_count
//      • residuals
//
//    These totals define the current inter-domain ratios ("tau").
//
// 3. now(domain)
//    Timebase first computes "now" in GNSS nanoseconds using the fresh
//    TIMEPULSE anchor:
//
//      gnss_now = anchor_gnss_ns + elapsed_dwt_cycles * 125 / 126
//
//    It then converts GNSS nanoseconds into the requested domain using the
//    most recent authoritative fragment ratios.
//
// 4. convert_ns(value, from_domain, to_domain)
//    Domain conversion always normalizes through GNSS:
//
//      from_domain → GNSS → to_domain
//
//    The conversion ratios are derived from the latest fragment totals.
//
// Design Notes
// ------------
//
// • The core API is enum-based for hot-path and ISR use.
// • No string dispatch in timing-sensitive code.
// • GNSS interpolation is integer-only and ISR-safe.
// • Both the TIMEPULSE anchor and the fragment use sequence counters so
//   readers never observe torn multiword state.
// • Anchor freshness is enforced. A stale TIMEPULSE anchor is treated as
//   invalid, which prevents Timebase from reporting stale "now" values
//   after a campaign stops or the anchor stream stalls.
//
// ============================================================================

#pragma once

#include "payload.h"

#include <stdint.h>

// ============================================================================
// Clock domain enum (core API)
// ============================================================================

enum class timebase_domain_t : uint8_t {
  GNSS = 0,
  DWT  = 1,
  OCXO = 2,
};

// ============================================================================
// Initialization
// ============================================================================

// Initialize internal state.
// Call once during clocks subsystem initialization.
void timebase_init(void);

// ============================================================================
// Primary API — "what time is it?"
// ============================================================================

// Returns the current nanosecond count in the specified domain.
// Returns -1 if the required state is not valid.
//
// ISR-safe. No allocation. Integer-only.
int64_t timebase_now_ns(timebase_domain_t domain);

// Fast-path helper for the hottest call site.
// Equivalent to timebase_now_ns(timebase_domain_t::GNSS).
int64_t timebase_now_gnss_ns(void);

// ============================================================================
// Domain conversion API
// ============================================================================

// Convert a nanosecond value from one clock domain to another.
// Returns -1 on failure (missing fragment, invalid ratio).
//
// Conversion is performed through GNSS using the latest authoritative
// TIMEBASE_FRAGMENT ratios.
int64_t timebase_convert_ns(
  uint64_t           value_ns,
  timebase_domain_t  from_domain,
  timebase_domain_t  to_domain
);

// ============================================================================
// Validity
// ============================================================================

// Returns true if Timebase can answer "now" in GNSS right now.
// This requires a fresh TIMEPULSE anchor.
bool timebase_valid(void);

// Returns true if domain-to-domain conversion is currently valid.
// This requires a valid TIMEBASE_FRAGMENT with nonzero domain totals.
bool timebase_conversion_valid(void);

// ============================================================================
// TIMEPULSE anchor update (called from TIMEPULSE ISR only)
// ============================================================================

// Update the interpolation anchor.
//
// Called by the TIMEPULSE ISR on each rising edge.
// Single write path only.
void timebase_update_anchor(uint64_t gnss_ns, uint32_t dwt_snap);

// ============================================================================
// Invalidation
// ============================================================================

// Invalidate all Timebase state.
// Safe to call when campaign truth is no longer trustworthy.
void timebase_invalidate(void);

// ============================================================================
// Fragment data access (per-second values from TIMEBASE_FRAGMENT)
// ============================================================================

struct timebase_fragment_t {
  volatile uint64_t gnss_ns;
  volatile uint64_t dwt_ns;
  volatile uint64_t dwt_cycles;
  volatile uint64_t ocxo_ns;
  volatile uint32_t pps_count;
  volatile int32_t  isr_residual_dwt;
  volatile int32_t  isr_residual_gnss;
  volatile int32_t  isr_residual_ocxo;
  volatile bool     valid;
};

// Read-only access to the last fragment.
// Returns pointer to internal static storage.
const timebase_fragment_t* timebase_last_fragment(void);

// TIMEBASE_FRAGMENT subscription callback.
void on_timebase_fragment(const Payload& payload);