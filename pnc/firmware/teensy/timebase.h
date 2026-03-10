// ============================================================================
// timebase.h — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// PPS-fragment anchored Timebase.  Any Teensy module that includes this
// header can ask "what time is it?" and receive GNSS-disciplined
// nanoseconds without caring about DWT cycles, clock domains, or PPS
// edge mechanics.
//
// Timebase answers three classes of questions:
//
//   1) "What time is it right now?"          → timebase_now_gnss_ns()
//   2) "How long did something take?"        → timebase_elapsed_ns()
//   3) "Convert between clock domains."      → timebase_convert_ns()
//
// Supported domains:
//
//   • GNSS — GNSS nanoseconds (the universal language of ZPNet)
//   • DWT  — DWT nanoseconds  (Teensy crystal, ~-3770 PPB drift)
//   • OCXO — OCXO nanoseconds (10 MHz disciplined oscillator)
//
// Architecture
// ------------
//
// 1. TIMEBASE_FRAGMENT (1 Hz, written by pub/sub callback)
//    Once per second, CLOCKS publishes authoritative PPS-edge totals:
//
//      • gnss_ns, dwt_ns, dwt_cycles, ocxo_ns
//      • pps_count, residuals
//
// 2. Local receipt snapshot
//    When the fragment arrives, Timebase captures ARM_DWT_CYCCNT.
//
// 3. now(domain)
//    Projects forward from the most recent fragment using local DWT
//    elapsed time since fragment receipt:
//
//      elapsed_ns = elapsed_dwt_cycles × 125 / 126
//      GNSS now   = fragment.gnss_ns + elapsed_ns
//
// 4. Durations
//    Capture a GNSS timestamp, do work, capture another.  The difference
//    is a duration in nanoseconds — directly comparable across instruments.
//
// 5. ISO 8601
//    Format any GNSS nanosecond value as "2026-03-08T01:34:44.123456789Z".
//    Uses the GPS epoch (1980-01-06) plus leap seconds.
//
// Setup
// -----
//
//   #include "timebase.h"
//
//   void setup() {
//     timebase_init();
//     pubsub_subscribe("TIMEBASE_FRAGMENT", on_timebase_fragment);
//   }
//
//   // Anywhere in your code:
//   int64_t now = timebase_now_gnss_ns();
//   if (now >= 0) { /* valid */ }
//
// ============================================================================

#pragma once

#include "payload.h"

#include <stdint.h>

// ============================================================================
// Clock domain enum
// ============================================================================

enum class timebase_domain_t : uint8_t {
  GNSS = 0,
  DWT  = 1,
  OCXO = 2,
};

// ============================================================================
// Initialization
// ============================================================================

/// Call once in setup().  Clears all fragment state.
void timebase_init(void);

// ============================================================================
// Primary API — "what time is it?"
// ============================================================================

/// Returns the current time in the specified domain, in nanoseconds.
/// Returns -1 if Timebase is not yet valid (no fragment received, or
/// fragment is stale beyond the 3-second freshness window).
int64_t timebase_now_ns(timebase_domain_t domain);

/// Convenience: equivalent to timebase_now_ns(GNSS).
/// This is the primary call for most code.
int64_t timebase_now_gnss_ns(void);

/// Compute GNSS nanoseconds from a pre-captured DWT_CYCCNT value and
/// fragment snapshot.  This is the deterministic overload: no live
/// register reads, no torn-read loop, no branching.  The caller
/// supplies values captured at a single instant (e.g., in an ISR).
///
/// Parameters are the fields from timepop_ctx_t:
///   dwt_cyccnt          — raw ARM_DWT_CYCCNT at the event
///   frag_gnss_ns        — fragment.gnss_ns at the anchoring PPS
///   frag_dwt_cyccnt_at_pps — fragment.dwt_cyccnt_at_pps (32-bit anchor)
///   frag_dwt_cycles_per_pps — DWT cycles in the prior second
///
/// Returns GNSS nanoseconds, or -1 if the inputs are invalid.
int64_t timebase_gnss_ns_from_dwt(
  uint32_t dwt_cyccnt,
  uint64_t frag_gnss_ns,
  uint32_t frag_dwt_cyccnt_at_pps,
  uint32_t frag_dwt_cycles_per_pps
);

/// Returns the synthetic campaign DWT cycle count at this instant.
/// This is fragment.dwt_cycles + elapsed cycles since the PPS edge.
/// Returns -1 if Timebase is not valid.
int64_t timebase_now_dwt_cycles(void);

// ============================================================================
// Duration API — "how long did that take?"
// ============================================================================

/// Capture a GNSS timestamp for later duration computation.
/// Returns -1 if Timebase is not valid.
///
/// Usage:
///   int64_t t0 = timebase_stamp();
///   // ... do work ...
///   int64_t elapsed = timebase_elapsed_ns(t0);
static inline int64_t timebase_stamp(void) {
  return timebase_now_gnss_ns();
}

/// Returns nanoseconds elapsed since a prior stamp.
/// Returns -1 if either the current time or the stamp is invalid.
static inline int64_t timebase_elapsed_ns(int64_t stamp) {
  if (stamp < 0) return -1;
  int64_t now = timebase_now_gnss_ns();
  if (now < 0) return -1;
  return now - stamp;
}

/// Returns the duration between two stamps (end - start) in nanoseconds.
/// Returns -1 if either stamp is invalid.
static inline int64_t timebase_duration_ns(int64_t start, int64_t end) {
  if (start < 0 || end < 0) return -1;
  return end - start;
}

// ============================================================================
// Domain conversion API
// ============================================================================

/// Convert a nanosecond value from one domain to another.
/// Conversion always normalizes through GNSS using the latest fragment
/// ratios.  Returns -1 on failure.
int64_t timebase_convert_ns(
  uint64_t           value_ns,
  timebase_domain_t  from_domain,
  timebase_domain_t  to_domain
);

// ============================================================================
// ISO 8601 formatting
// ============================================================================

/// Format a GNSS nanosecond value as ISO 8601 with nanosecond precision.
/// Writes to buf (must be at least 31 bytes): "2026-03-08T01:34:44.123456789Z"
/// Returns the number of characters written (excluding null terminator),
/// or 0 on failure.
///
/// GNSS nanoseconds count from the GPS epoch (1980-01-06T00:00:00Z) with
/// the current leap second offset applied by the GNSS module.
int timebase_format_iso8601(uint64_t gnss_ns, char* buf, size_t buf_len);

/// Convenience: format the current GNSS time as ISO 8601.
/// Returns 0 if Timebase is not valid.
int timebase_now_iso8601(char* buf, size_t buf_len);

// ============================================================================
// PPS count
// ============================================================================

/// Returns the PPS count from the most recent valid fragment.
/// Returns 0 if no fragment has been received.
uint32_t timebase_pps_count(void);

// ============================================================================
// Validity
// ============================================================================

/// True if timebase_now_gnss_ns() would succeed right now.
bool timebase_valid(void);

/// True if all three domains (GNSS, DWT, OCXO) have nonzero fragment
/// totals, meaning convert_ns() can operate between any pair.
bool timebase_conversion_valid(void);

// ============================================================================
// Invalidation
// ============================================================================

/// Clear all fragment state.  Called on STOP or pipeline failure.
void timebase_invalidate(void);

// ============================================================================
// Fragment data access (diagnostic / reporting)
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
  volatile uint64_t dwt_cycles_per_pps;
  volatile uint32_t dwt_cyccnt_at_pps;
  volatile uint32_t gpt2_at_pps;        // GPT2_CNT at PPS edge (VCLOCK anchor)
  volatile bool     valid;
};

/// Returns a pointer to the most recent fragment (diagnostic use).
const timebase_fragment_t* timebase_last_fragment(void);

// ============================================================================
// TIMEBASE_FRAGMENT subscription callback
// ============================================================================

/// Pub/sub callback — wire this to "TIMEBASE_FRAGMENT" in your setup:
///   pubsub_subscribe("TIMEBASE_FRAGMENT", on_timebase_fragment);
void on_timebase_fragment(const Payload& payload);