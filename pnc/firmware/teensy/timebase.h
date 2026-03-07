// ============================================================================
// timebase.h — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// PPS-fragment anchored Timebase.
//
// Timebase answers two classes of questions:
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
// 1. TIMEBASE_FRAGMENT (1 Hz, written by pub/sub callback)
//    Once per second, CLOCKS publishes authoritative PPS-edge totals:
//
//      • gnss_ns
//      • dwt_ns
//      • dwt_cycles
//      • ocxo_ns
//      • pps_count
//      • residuals
//
// 2. Local receipt snapshot
//    When the fragment is received, Timebase captures the local
//    ARM_DWT_CYCCNT in scheduled context.
//
// 3. now(domain)
//    Timebase projects forward from the most recent fragment using
//    local DWT elapsed time since fragment receipt.
//
//      elapsed_ns = elapsed_dwt_cycles * 125 / 126
//
//      GNSS now = fragment.gnss_ns + elapsed_ns
//      DWT  now = fragment.dwt_ns  + elapsed_ns
//
//    OCXO "now" is derived by converting projected GNSS through the
//    most recent fragment ratio.
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
// • No TIMEPULSE support remains.
// • The core API is enum-based for hot-path use.
// • Both the fragment store and public mirror are sequence-safe / read-only.
// • Fragment freshness is enforced to prevent stale "now" narration after
//   stop or pipeline failure.
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

void timebase_init(void);

// ============================================================================
// Primary API — "what time is it?"
// ============================================================================

int64_t timebase_now_ns(timebase_domain_t domain);
int64_t timebase_now_gnss_ns(void);

// ============================================================================
// Domain conversion API
// ============================================================================

int64_t timebase_convert_ns(
  uint64_t           value_ns,
  timebase_domain_t  from_domain,
  timebase_domain_t  to_domain
);

// ============================================================================
// Validity
// ============================================================================

bool timebase_valid(void);
bool timebase_conversion_valid(void);

// ============================================================================
// Invalidation
// ============================================================================

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

const timebase_fragment_t* timebase_last_fragment(void);
void on_timebase_fragment(const Payload& payload);