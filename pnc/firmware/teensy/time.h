#pragma once

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.h — VCLOCK-authored GNSS Nanosecond Interface (Teensy)
// ============================================================================
//
// Core doctrine:
//
//   PPS is a witness and selector.  It tells process_interrupt which VCLOCK
//   10 MHz edge is the canonical second boundary.
//
//   PPS/VCLOCK is the selected VCLOCK edge.  It is the sovereign time anchor.
//
//   DWT is the high-resolution local ruler used to interpolate between
//   successive PPS/VCLOCK anchors.
//
// process_time.cpp owns the anchor, interpolation state, and DWT next-second
// prediction state.  This header remains the stable general interface for
// ordinary consumers.
//
// This module performs NO latency adjustment.  Every DWT value passed in or
// returned is already an event coordinate.  Raw ISR-entry DWT values must be
// normalized by the ISR owner before reaching this interface.
//
// Returned GNSS nanoseconds are local zero-based campaign/epoch coordinates,
// not wall-clock UTC.  The epoch may be restarted at boot, ZERO, START, or
// RECOVER boundaries.
//
// Compatibility note:
//   The public snapshot still carries legacy field aliases such as dwt_at_pps,
//   qtimer_at_pps, and pps_count so older callers keep compiling.  New code
//   should prefer the PPS/VCLOCK names.
// ============================================================================

struct time_anchor_snapshot_t {
  // Canonical fields — use these in new code.
  uint32_t dwt_at_pps_vclock;             // DWT coordinate of selected VCLOCK edge
  uint32_t dwt_cycles_per_pps_vclock_s;   // DWT cycles between selected VCLOCK edges
  uint32_t counter32_at_pps_vclock;       // synthetic VCLOCK counter32 at anchor
  uint32_t pps_vclock_count;              // selected VCLOCK second edges in epoch, 1-indexed
  bool     valid;                         // true once a real rate is available
  bool     ok;                            // true if snapshot was seqlock-consistent

  // Legacy aliases — kept for TimePop / older code during migration.
  uint32_t dwt_at_pps;
  uint32_t dwt_cycles_per_s;
  uint32_t qtimer_at_pps;
  uint32_t pps_count;
};

// ============================================================================
// DWT next-second prediction surface
// ============================================================================
//
// Prediction doctrine:
//   • The actual PPS/VCLOCK-to-PPS/VCLOCK DWT delta remains canonical.
//   • The prediction is advisory and is judged after the next edge arrives.
//   • The initial model is random-walk: next predicted cycles = last actual
//     cycles.  Future predictors can replace this behind the same interface.
//
// A record describes one measured second.  `predicted_cycles` is the forecast
// made before that second completed.  `actual_cycles` is the measured value
// supplied by time_pps_vclock_update().  `residual_cycles` is actual - predicted.
// If valid is false, the record represents an actual measurement for which no
// prior prediction existed yet (typically first post-epoch measurement).

struct time_dwt_prediction_record_t {
  uint32_t pps_vclock_count;
  uint32_t dwt_at_pps_vclock;
  uint32_t counter32_at_pps_vclock;
  uint32_t predicted_cycles;
  uint32_t actual_cycles;
  int32_t  residual_cycles;
  bool     valid;
};

struct time_dwt_prediction_snapshot_t {
  bool     valid;
  uint32_t pps_vclock_count;
  uint32_t predicted_cycles_last;
  uint32_t actual_cycles_last;
  int32_t  residual_cycles_last;
  uint32_t predicted_cycles_next;
  uint32_t history_count;
  uint32_t history_capacity;
};

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void);

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt);

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps_vclock,
                            uint32_t anchor_dwt_cycles_per_pps_vclock_s,
                            uint32_t anchor_pps_vclock_count);

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns);

// ============================================================================
// Anchor snapshot — seqlock-safe copy of the full PPS/VCLOCK anchor
// ============================================================================

time_anchor_snapshot_t time_anchor_snapshot(void);

// ============================================================================
// DWT prediction accessors
// ============================================================================

time_dwt_prediction_snapshot_t time_dwt_prediction_snapshot(void);

// Copies up to max_records recent records into out_records, oldest-to-newest.
// Returns the number of records copied.  Passing nullptr or max_records=0 is
// valid and returns 0.
uint32_t time_dwt_prediction_history(time_dwt_prediction_record_t* out_records,
                                     uint32_t max_records);

bool     time_dwt_prediction_valid(void);
uint32_t time_dwt_actual_cycles_last_second(void);
uint32_t time_dwt_predicted_cycles_last_second(void);
uint32_t time_dwt_next_prediction_cycles(void);
int32_t  time_dwt_prediction_residual_cycles(void);

// ============================================================================
// Status
// ============================================================================

uint32_t time_pps_vclock_count(void);

// Legacy alias: returns time_pps_vclock_count().
uint32_t time_pps_count(void);

bool time_valid(void);

// ============================================================================
// Update interface (called by process_clocks_alpha.cpp only)
// ============================================================================

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock);

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock);

// Legacy wrappers retained during migration.
void time_pps_update(uint32_t dwt_at_pps,
                     uint32_t dwt_cycles_per_s,
                     uint32_t qtimer_at_pps);

void time_pps_epoch_reset(uint32_t dwt_at_pps,
                          uint32_t qtimer_at_pps);

// ============================================================================
// Initialization
// ============================================================================

void time_init(void);
