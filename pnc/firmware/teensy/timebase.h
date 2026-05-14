// ============================================================================
// timebase.h — ZPNet Timebase Singleton (Teensy) — v11 Dual OCXO
// ============================================================================
//
// PPS-fragment anchored Timebase.  Any Teensy module that includes this
// header can ask "what time is it?" and receive GNSS-disciplined
// nanoseconds without caring about DWT cycles, clock domains, or PPS
// edge mechanics.
//
// Supported domains:
//
//   • GNSS  — GNSS nanoseconds (the universal language of ZPNet)
//   • DWT   — DWT nanoseconds  (Teensy crystal, ~-3400 PPB drift)
//   • OCXO1 — OCXO1 nanoseconds (first 10 MHz oven oscillator)
//   • OCXO2 — OCXO2 nanoseconds (second 10 MHz oven oscillator)
//
// ============================================================================

#pragma once

#include "payload.h"

#include <stdint.h>

// ============================================================================
// Clock domain enum
// ============================================================================

enum class timebase_domain_t : uint8_t {
  GNSS  = 0,
  DWT   = 1,
  OCXO1 = 2,
  OCXO2 = 3,
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

int64_t timebase_gnss_ns_from_dwt(
  uint32_t dwt_cyccnt,
  uint64_t frag_gnss_ns,
  uint32_t frag_dwt_cyccnt_at_pps,
  uint32_t frag_dwt_cycles_per_pps
);

int64_t timebase_now_dwt_cycles(void);

// ============================================================================
// Duration API
// ============================================================================

static inline int64_t timebase_stamp(void) {
  return timebase_now_gnss_ns();
}

static inline int64_t timebase_elapsed_ns(int64_t stamp) {
  if (stamp < 0) return -1;
  int64_t now = timebase_now_gnss_ns();
  if (now < 0) return -1;
  return now - stamp;
}

static inline int64_t timebase_duration_ns(int64_t start, int64_t end) {
  if (start < 0 || end < 0) return -1;
  return end - start;
}

// ============================================================================
// Domain conversion API
// ============================================================================

int64_t timebase_convert_ns(
  uint64_t           value_ns,
  timebase_domain_t  from_domain,
  timebase_domain_t  to_domain
);

// ============================================================================
// ISO 8601 formatting
// ============================================================================

int timebase_format_iso8601(uint64_t gnss_ns, char* buf, size_t buf_len);
int timebase_now_iso8601(char* buf, size_t buf_len);

// ============================================================================
// PPS count
// ============================================================================

uint32_t timebase_pps_count(void);

// ============================================================================
// Validity
// ============================================================================

bool timebase_valid(void);

/// True if all four domains (GNSS, DWT, OCXO1, OCXO2) have nonzero
/// fragment totals, meaning convert_ns() can operate between any pair.
bool timebase_conversion_valid(void);

// ============================================================================
// Invalidation
// ============================================================================

void timebase_invalidate(void);

// ============================================================================
// Fragment data access (diagnostic / reporting)
// ============================================================================

struct timebase_fragment_t {
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
  volatile uint64_t dwt_cycles_per_pps;
  volatile uint32_t dwt_cyccnt_at_pps;
  volatile uint32_t qtimer_at_pps;
  volatile bool     valid;
};

const timebase_fragment_t* timebase_last_fragment(void);

// ============================================================================
// TIMEBASE_FRAGMENT subscription callback
// ============================================================================

void on_timebase_fragment(const Payload& payload);