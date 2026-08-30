// ============================================================================
// process_clocks_gamma.cpp -- CLOCKS support layer
// ============================================================================
//
// Gamma owns the live PPS/VCLOCK anchor consumed by TimePop through
// time_anchor_snapshot(), plus the per-clock projection backing store consumed
// by the TIME facade.
//
// Gamma is not a registered process and exposes no command surface.  The public
// time_* interface remains in time.h so all existing callers retain the same
// symbols and behavior.
//
// Retired behavior remains retired:
//   * dynamic-CPS prediction
//   * prior-second prediction history
//   * prediction-detail reports
//   * legacy conversion aliases
//
// CLOCKS/Alpha owns the real four-rail prediction audit, and CLOCKS/Beta
// publishes that audit through CLOCKS_FRAGMENT.  The Gamma name is intentionally
// responsibility-neutral so other CLOCKS implementation can move here without
// changing subsystem ownership or public command identity.
//
// All DWT inputs are already authored event-coordinate values.  Gamma performs
// no latency adjustment and never captures "now" except in the legacy
// time_gnss_ns_now() convenience call.
// ============================================================================

#include "time.h"
#include "config.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stdint.h>
#include <stdbool.h>

static constexpr uint64_t TIME_NS_PER_SECOND_U64 = 1000000000ULL;
static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;

// ============================================================================
// Memory ordering
// ============================================================================

static inline void dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static inline void seqlock_write_begin(volatile uint32_t& sequence) {
  sequence++;
  dmb();
}

static inline void seqlock_write_end(volatile uint32_t& sequence) {
  dmb();
  sequence++;
}

// ============================================================================
// PPS/VCLOCK anchor state
// ============================================================================

struct time_anchor_store_t {
  volatile uint32_t seq = 0;
  volatile uint32_t dwt_at_pps_vclock = 0;
  volatile uint32_t dwt_cycles_per_pps_vclock_s = 0;
  volatile uint32_t counter32_at_pps_vclock = 0;
  volatile uint32_t pps_vclock_count = 0;
  volatile bool     valid = false;
};

static time_anchor_store_t g_time_anchor{};

// Immutable value copied from the live anchor store. Once acquired, callers
// can project from it without observing later writer mutations.
struct time_anchor_value_t {
  uint32_t dwt_at_pps_vclock = 0;
  uint32_t dwt_cycles_per_pps_vclock_s = 0;
  uint32_t counter32_at_pps_vclock = 0;
  uint32_t pps_vclock_count = 0;
  bool     valid = false;
  bool     snapshot_ok = false;
};

static time_anchor_value_t time_anchor_load(void) {
  time_anchor_value_t value{};

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = g_time_anchor.seq;
    dmb();

    value.dwt_at_pps_vclock = g_time_anchor.dwt_at_pps_vclock;
    value.dwt_cycles_per_pps_vclock_s = g_time_anchor.dwt_cycles_per_pps_vclock_s;
    value.counter32_at_pps_vclock = g_time_anchor.counter32_at_pps_vclock;
    value.pps_vclock_count = g_time_anchor.pps_vclock_count;
    value.valid = g_time_anchor.valid;

    dmb();
    const uint32_t seq2 = g_time_anchor.seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      value.snapshot_ok = true;
      return value;
    }
  }

  return time_anchor_value_t{};
}

static void time_anchor_publish(const time_anchor_value_t& value) {
  seqlock_write_begin(g_time_anchor.seq);
  g_time_anchor.dwt_at_pps_vclock = value.dwt_at_pps_vclock;
  g_time_anchor.dwt_cycles_per_pps_vclock_s =
      value.dwt_cycles_per_pps_vclock_s;
  g_time_anchor.counter32_at_pps_vclock = value.counter32_at_pps_vclock;
  g_time_anchor.pps_vclock_count = value.pps_vclock_count;
  g_time_anchor.valid = value.valid;
  seqlock_write_end(g_time_anchor.seq);
}

time_anchor_snapshot_t time_anchor_snapshot(void) {
  const time_anchor_value_t value = time_anchor_load();
  time_anchor_snapshot_t pub{};

  pub.dwt_at_pps_vclock = value.dwt_at_pps_vclock;
  pub.dwt_cycles_per_pps_vclock_s = value.dwt_cycles_per_pps_vclock_s;
  pub.counter32_at_pps_vclock = value.counter32_at_pps_vclock;
  pub.pps_vclock_count = value.pps_vclock_count;
  pub.valid = value.valid;
  pub.ok = value.snapshot_ok;

  // Legacy aliases used by TimePop and older callers.
  pub.dwt_at_pps = value.dwt_at_pps_vclock;
  pub.dwt_cycles_per_s = value.dwt_cycles_per_pps_vclock_s;
  pub.qtimer_at_pps = value.counter32_at_pps_vclock;
  pub.pps_count = value.pps_vclock_count;

  return pub;
}

// ============================================================================
// Per-clock projection state
// ============================================================================

static constexpr uint32_t TIME_PROJECTION_SLOT_COUNT = 4;  // index by time_clock_id_t value

struct time_projection_store_t {
  volatile uint32_t seq = 0;
  bool     valid = false;
  bool     projection_rate_valid = false;
  uint32_t dwt_at_update = 0;
  uint64_t ns_at_update = 0;
  uint32_t projection_cycles_per_second = DWT_EXPECTED_PER_PPS;
  uint32_t update_count = 0;
  uint32_t last_observed_dwt_cycles = 0;
  uint64_t last_observed_ns = 0;
  int32_t  last_rate_change_cycles = 0;
};

static time_projection_store_t g_time_projection_stores[TIME_PROJECTION_SLOT_COUNT]{};

static int time_projection_index(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::VCLOCK: return 1;
    case time_clock_id_t::OCXO1:  return 2;
    case time_clock_id_t::OCXO2:  return 3;
    default:                       return -1;
  }
}

static bool time_clock_load(time_clock_id_t clock, time_clock_snapshot_t& out) {
  out = time_clock_snapshot_t{};
  const int idx = time_projection_index(clock);
  if (idx < 0) return false;

  const time_projection_store_t& c = g_time_projection_stores[idx];
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = c.seq;
    dmb();

    out.valid = c.valid;
    out.prediction_valid = c.projection_rate_valid;
    out.dwt_at_update = c.dwt_at_update;
    out.ns_at_update = c.ns_at_update;
    out.predicted_dwt_cycles_per_second = c.projection_cycles_per_second;
    out.update_count = c.update_count;
    out.last_observed_dwt_cycles = c.last_observed_dwt_cycles;
    out.last_observed_ns = c.last_observed_ns;
    out.last_prediction_residual_cycles = c.last_rate_change_cycles;

    dmb();
    const uint32_t s2 = c.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return true;
  }

  out = time_clock_snapshot_t{};
  return false;
}

static void time_projection_clear(time_projection_store_t& store) {
  store.valid = false;
  store.projection_rate_valid = false;
  store.dwt_at_update = 0U;
  store.ns_at_update = 0ULL;
  store.projection_cycles_per_second = DWT_EXPECTED_PER_PPS;
  store.update_count = 0U;
  store.last_observed_dwt_cycles = 0U;
  store.last_observed_ns = 0ULL;
  store.last_rate_change_cycles = 0;
}

void time_clock_reset_all(void) {
  for (uint32_t i = 0U; i < TIME_PROJECTION_SLOT_COUNT; ++i) {
    time_projection_store_t& store = g_time_projection_stores[i];
    seqlock_write_begin(store.seq);
    time_projection_clear(store);
    seqlock_write_end(store.seq);
  }
}

bool time_clock_epoch_reset(time_clock_id_t clock,
                            uint32_t dwt_at_update,
                            uint64_t ns_at_update) {
  const int idx = time_projection_index(clock);
  if (idx < 0) return false;
  time_projection_store_t& store = g_time_projection_stores[idx];

  seqlock_write_begin(store.seq);
  time_projection_clear(store);
  store.valid = true;
  store.projection_rate_valid = true;
  store.dwt_at_update = dwt_at_update;
  store.ns_at_update = ns_at_update;
  seqlock_write_end(store.seq);
  return true;
}

bool time_clock_update(time_clock_id_t clock,
                       uint32_t dwt_at_update,
                       uint64_t ns_at_update) {
  const int idx = time_projection_index(clock);
  if (idx < 0) return false;
  time_projection_store_t& c = g_time_projection_stores[idx];

  seqlock_write_begin(c.seq);

  if (c.valid) {
    const uint32_t observed_dwt = dwt_at_update - c.dwt_at_update;
    const uint64_t observed_ns = (ns_at_update >= c.ns_at_update)
        ? (ns_at_update - c.ns_at_update)
        : 0ULL;

    if (observed_dwt != 0 && observed_ns != 0) {
      const uint32_t observed_cps =
          (uint32_t)(((uint64_t)observed_dwt * TIME_NS_PER_SECOND_U64 +
                      observed_ns / 2ULL) /
                     observed_ns);
      c.last_rate_change_cycles = c.projection_rate_valid
          ? (int32_t)((int64_t)observed_cps -
                      (int64_t)c.projection_cycles_per_second)
          : 0;
      c.projection_cycles_per_second =
          observed_cps ? observed_cps : DWT_EXPECTED_PER_PPS;
      c.projection_rate_valid = true;
      c.last_observed_dwt_cycles = observed_dwt;
      c.last_observed_ns = observed_ns;
    }
  } else {
    c.projection_cycles_per_second = DWT_EXPECTED_PER_PPS;
    c.projection_rate_valid = true;
    c.last_rate_change_cycles = 0;
    c.last_observed_dwt_cycles = 0;
    c.last_observed_ns = 0;
  }

  c.valid = true;
  c.dwt_at_update = dwt_at_update;
  c.ns_at_update = ns_at_update;
  c.update_count++;

  seqlock_write_end(c.seq);
  return true;
}

bool time_clock_ns_at_dwt(time_clock_id_t clock,
                          uint32_t authored_dwt_cycle_count,
                          uint64_t* out_ns) {
  if (!out_ns) return false;

  time_clock_snapshot_t s{};
  if (!time_clock_load(clock, s)) return false;
  if (!s.valid || !s.prediction_valid || s.predicted_dwt_cycles_per_second == 0) {
    return false;
  }

  const uint32_t elapsed_dwt = authored_dwt_cycle_count - s.dwt_at_update;
  const uint64_t elapsed_ns =
      ((uint64_t)elapsed_dwt * TIME_NS_PER_SECOND_U64 +
       (uint64_t)s.predicted_dwt_cycles_per_second / 2ULL) /
      (uint64_t)s.predicted_dwt_cycles_per_second;

  *out_ns = s.ns_at_update + elapsed_ns;
  return true;
}

bool time_clock_snapshot(time_clock_id_t clock,
                         time_clock_snapshot_t* out) {
  if (!out) return false;
  return time_clock_load(clock, *out);
}

// ============================================================================
// Anchor writers
// ============================================================================

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock) {
  time_anchor_value_t value{};
  value.dwt_at_pps_vclock = dwt_at_pps_vclock;
  value.counter32_at_pps_vclock = counter32_at_pps_vclock;
  value.pps_vclock_count = 1U;
  time_anchor_publish(value);
}

static void time_pps_vclock_publish(uint32_t dwt_at_pps_vclock,
                                    uint32_t dwt_cycles_per_pps_vclock_s,
                                    uint32_t counter32_at_pps_vclock,
                                    uint32_t pps_vclock_count) {
  time_anchor_value_t value{};
  value.dwt_at_pps_vclock = dwt_at_pps_vclock;
  value.dwt_cycles_per_pps_vclock_s = dwt_cycles_per_pps_vclock_s;
  value.counter32_at_pps_vclock = counter32_at_pps_vclock;
  value.pps_vclock_count = pps_vclock_count ? pps_vclock_count : 1U;
  value.valid = dwt_cycles_per_pps_vclock_s > 0U;
  time_anchor_publish(value);
}

// Historical 3-argument order used by CLOCKS/Alpha:
//   dwt, cycles_per_second, counter32
void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock) {
  const time_anchor_value_t prior = time_anchor_load();
  if (!prior.snapshot_ok) __builtin_trap();
  const uint32_t next_count = prior.pps_vclock_count
      ? (prior.pps_vclock_count + 1U)
      : 1U;

  time_pps_vclock_publish(dwt_at_pps_vclock,
                                  dwt_cycles_per_pps_vclock_s,
                                  counter32_at_pps_vclock,
                                  next_count);
}

// Explicit count overload retained for migration call sites.
// Header parameter names are legacy; operational order here is:
//   dwt, counter32, count, cycles_per_second
void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t counter32_at_pps_vclock,
                            uint32_t pps_vclock_count,
                            uint32_t dwt_cycles_per_pps_vclock_s) {
  time_pps_vclock_publish(dwt_at_pps_vclock,
                                  dwt_cycles_per_pps_vclock_s,
                                  counter32_at_pps_vclock,
                                  pps_vclock_count);
}

// ============================================================================
// GNSS interpolation helper
// ============================================================================

static inline int64_t interpolate_gnss_ns(const time_anchor_value_t& value,
                                          uint32_t dwt_elapsed) {
  const uint32_t cycles = value.dwt_cycles_per_pps_vclock_s;
  if (cycles == 0) return -1;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * TIME_NS_PER_SECOND_U64 +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(value.pps_vclock_count - 1) *
                   TIME_NS_PER_SECOND_U64 +
                   ns_into_second);
}

int64_t time_gnss_ns_now(void) {
  const time_anchor_value_t value = time_anchor_load();
  if (!value.snapshot_ok || !value.valid ||
      value.dwt_cycles_per_pps_vclock_s == 0U) {
    return -1;
  }

  const uint32_t dwt_elapsed = ARM_DWT_CYCCNT - value.dwt_at_pps_vclock;
  return interpolate_gnss_ns(value, dwt_elapsed);
}

// ============================================================================
// Status / init
// ============================================================================

uint32_t time_pps_count(void) {
  const time_anchor_value_t value = time_anchor_load();
  return value.snapshot_ok ? value.pps_vclock_count : 0;
}

bool time_valid(void) {
  const time_anchor_value_t value = time_anchor_load();
  return value.snapshot_ok && value.valid &&
      value.dwt_cycles_per_pps_vclock_s > 0U;
}

void time_init(void) {
  g_time_anchor = time_anchor_store_t{};
  time_clock_reset_all();
}

