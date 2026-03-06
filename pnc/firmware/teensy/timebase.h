// ============================================================================
// timebase.h — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// Timebase is a system-wide singleton that provides nanosecond-precision
// time in any clock domain.  It is the universal answer to "what time
// is it?" on the Teensy.
//
// Architecture:
//
//   Timebase subscribes to TIMEBASE_FRAGMENT via pubsub.  Each second,
//   the fragment callback stores all clock-domain values as integers
//   for fast, lock-free access.  The TIMEPULSE 10 KHz ISR provides
//   a sub-millisecond anchor (gnss_ns + DWT snap) that is updated
//   every 100 µs.
//
//   now("GNSS") returns the GNSS nanosecond count interpolated to
//   the present moment:
//
//     result = anchor_gnss_ns + (DWT_CYCCNT - anchor_dwt) * gnss_per_dwt
//
//   where gnss_per_dwt is the ratio of GNSS ns per DWT cycle, derived
//   from the anchor's known relationship.  The anchor is refreshed at
//   10 KHz by the TIMEPULSE ISR, so worst-case staleness is 100 µs.
//   DWT at 1.008 GHz provides sub-nanosecond interpolation granularity.
//
// ISR safety:
//
//   now() is safe to call from any context: ISR, scheduled, main loop.
//   No locks, no malloc, no floating point.  Integer arithmetic on
//   volatile state plus a single register read.  One UDIV instruction
//   (2-12 cycles on Cortex-M7).
//
// API:
//
//   timebase_init()           — call once at startup
//   timebase_now_ns("GNSS")  — current GNSS nanoseconds (interpolated)
//   timebase_now_ns("DWT")   — current DWT nanoseconds (direct read)
//   timebase_valid()          — true once first TIMEPULSE anchor exists
//
// Clock domain strings:
//
//   "GNSS"  — GNSS 10 MHz domain, interpolated via DWT from TIMEPULSE
//   "DWT"   — ARM DWT cycle counter, converted to ns at nominal rate
//   "OCXO"  — (future) OCXO 10 MHz domain
//   "PI"    — (future) Pi arch timer domain
//
// The string-based selector is intentional: same API on Pi side,
// easy to reason about, no enum synchronization across builds.
//
// Ownership:
//
//   Timebase owns the TIMEPULSE anchor struct.  process_clocks.cpp
//   calls timebase_update_anchor() from the TIMEPULSE ISR.
//   Timebase subscribes to TIMEBASE_FRAGMENT for per-second data.
//
// ============================================================================

#pragma once

#include "payload.h"

#include <stdint.h>

// ============================================================================
// Initialization
// ============================================================================

// Call once during clocks subsystem init.
// Initializes state.  The caller (process_clocks) must include
// timebase_subscriptions() in the CLOCKS vtable for TIMEBASE_FRAGMENT
// delivery.
void timebase_init(void);

// ============================================================================
// Primary API — "what time is it?"
// ============================================================================

// Returns the current nanosecond count in the specified clock domain.
// Returns -1 if the domain is unknown or not yet valid.
//
// ISR-safe.  No locks, no allocation, no floating point.
//
// Supported domains:
//   "GNSS"  — interpolated from TIMEPULSE anchor + DWT delta
//   "DWT"   — direct DWT_CYCCNT converted to ns at nominal 1.008 GHz
//
int64_t timebase_now_ns(const char* domain);

// Returns true once TIMEPULSE has produced at least one valid anchor.
// Until this returns true, now_ns("GNSS") returns -1.
bool timebase_valid(void);

// ============================================================================
// TIMEPULSE anchor update (called from TIMEPULSE ISR only)
// ============================================================================

// Called by the TIMEPULSE 10 KHz ISR on each rising edge.
// Stores the current gnss_ns and DWT snapshot as the interpolation anchor.
//
// This is the only write path to the anchor.  ISR context guarantees
// atomicity on Cortex-M7 (single-writer, no preemption of ISR by
// non-ISR readers causes at most a torn 64-bit read, handled by
// double-read consistency check).
//
void timebase_update_anchor(uint64_t gnss_ns, uint32_t dwt_snap);

// ============================================================================
// TIMEPULSE anchor invalidation
// ============================================================================

// Called when the campaign stops or the anchor is no longer trustworthy.
void timebase_invalidate(void);

// ============================================================================
// Fragment data access (per-second values from TIMEBASE_FRAGMENT)
// ============================================================================

// Last TIMEBASE_FRAGMENT values, stored as integers for fast access.
// Updated once per second by the fragment subscription callback.
// These are point-in-time PPS-edge values, NOT interpolated.

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

// Read-only access to the last fragment.  Returns pointer to internal
// static — do not free.  Fields are volatile; safe to read from any
// context but individual fields may update independently.
const timebase_fragment_t* timebase_last_fragment(void);

// On timebase callback.
void on_timebase_fragment(const Payload& payload);