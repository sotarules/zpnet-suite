#pragma once

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.h — Universal GNSS Nanosecond Bridge (Teensy)
// ============================================================================
//
// Three pure functions.  No state.  No init.  No writers.
//
//   time_dwt_to_gnss_ns(dwt)  — DWT cycle → GNSS nanosecond
//   time_gnss_ns_to_dwt(ns)   — GNSS nanosecond → DWT cycle
//   time_gnss_ns_now()        — current GNSS nanosecond (DWT live read)
//
// All three read the canonical PPS_VCLOCK anchor from alpha (via
// alpha_pps_vclock_load()) and the canonical rate from alpha (via
// alpha_dwt_cycles_per_second()).  Stateless: no anchor is held here,
// no PPS count, no seqlock — alpha owns all of it.
//
// Contract: every DWT value in or out is an event coordinate.  Latency
// math (raw ISR-entry → event coordinate) belongs to process_interrupt.
//
// Returns -1 (or 0 for the reverse direction) when no valid anchor is
// available — typically before the first PPS edge.
// ============================================================================

// ============================================================================
// PPS_VCLOCK slot — alpha-authored, time.cpp + TimePop consumed
// ============================================================================
//
// Alpha publishes the most recent PPS_VCLOCK event into a seqlock-protected
// slot.  time.cpp reads gnss_ns_at_edge and dwt_at_edge for the bridge.
// TimePop additionally reads counter32_at_edge for gnss_ns→counter32
// conversions when scheduling deadlines.
//
// sequence == 0 means "no edge ever recorded" — slot invalid.

struct alpha_pps_vclock_slot_t {
  int64_t  gnss_ns_at_edge;
  uint32_t dwt_at_edge;
  uint32_t counter32_at_edge;   // synthetic VCLOCK counter at PPS moment
  uint32_t sequence;
};

alpha_pps_vclock_slot_t alpha_pps_vclock_load       (void);
uint32_t                alpha_dwt_cycles_per_second(void);

// ============================================================================
// Bridge functions
// ============================================================================

int64_t  time_gnss_ns_now    (void);
int64_t  time_dwt_to_gnss_ns (uint32_t dwt_cyccnt);
uint32_t time_gnss_ns_to_dwt (int64_t  gnss_ns);