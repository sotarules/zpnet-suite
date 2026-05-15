// ============================================================================
// process_time.cpp -- TIME compatibility anchor and projection backing store
// ============================================================================
//
// process_time remains alive.  It owns the live PPS/VCLOCK anchor consumed by
// TimePop through time_anchor_snapshot(), plus the per-clock projection backing
// store consumed by the TIME facade.
//
// Retired behavior has been removed from this file:
//   * dynamic-CPS prediction
//   * prior-second prediction history
//   * prediction-detail reports
//   * legacy conversion aliases
//
// CLOCKS/Alpha now owns the real four-rail prediction audit, and CLOCKS/Beta
// publishes that audit in TIMEBASE_FRAGMENT.  This module is intentionally
// narrow: anchor state, per-clock projection state, TIME.REPORT, and process
// registration.
//
// All DWT inputs are already authored event-coordinate values.  This module
// performs no latency adjustment and never captures "now" except in the legacy
// time_gnss_ns_now() convenience call.
// ============================================================================

#include "process_time.h"
#include "time.h"
#include "config.h"
#include "process.h"
#include "payload.h"

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

// ============================================================================
// PPS/VCLOCK anchor state
// ============================================================================

struct time_anchor_t {
  volatile uint32_t seq = 0;
  volatile uint32_t dwt_at_pps_vclock = 0;
  volatile uint32_t dwt_cycles_per_pps_vclock_s = 0;
  volatile uint32_t counter32_at_pps_vclock = 0;
  volatile uint32_t pps_vclock_count = 0;
  volatile bool     valid = false;
};

static time_anchor_t anchor = {};

struct time_snapshot_t {
  uint32_t dwt_at_pps_vclock = 0;
  uint32_t dwt_cycles_per_pps_vclock_s = 0;
  uint32_t counter32_at_pps_vclock = 0;
  uint32_t pps_vclock_count = 0;
  bool     valid = false;
  bool     ok = false;
};

static time_snapshot_t read_anchor(void) {
  time_snapshot_t s{};

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = anchor.seq;
    dmb();

    s.dwt_at_pps_vclock = anchor.dwt_at_pps_vclock;
    s.dwt_cycles_per_pps_vclock_s = anchor.dwt_cycles_per_pps_vclock_s;
    s.counter32_at_pps_vclock = anchor.counter32_at_pps_vclock;
    s.pps_vclock_count = anchor.pps_vclock_count;
    s.valid = anchor.valid;

    dmb();
    const uint32_t s2 = anchor.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) {
      s.ok = true;
      return s;
    }
  }

  return time_snapshot_t{};
}

static inline uint32_t effective_cycles_per_snapshot(const time_snapshot_t& s) {
  return s.dwt_cycles_per_pps_vclock_s;
}

time_anchor_snapshot_t time_anchor_snapshot(void) {
  const time_snapshot_t s = read_anchor();
  time_anchor_snapshot_t pub{};

  pub.dwt_at_pps_vclock = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_pps_vclock_s = s.dwt_cycles_per_pps_vclock_s;
  pub.counter32_at_pps_vclock = s.counter32_at_pps_vclock;
  pub.pps_vclock_count = s.pps_vclock_count;
  pub.valid = s.valid;
  pub.ok = s.ok;

  // Legacy aliases used by TimePop and older callers.
  pub.dwt_at_pps = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_s = s.dwt_cycles_per_pps_vclock_s;
  pub.qtimer_at_pps = s.counter32_at_pps_vclock;
  pub.pps_count = s.pps_vclock_count;

  return pub;
}

// ============================================================================
// Per-clock projection state
// ============================================================================

static constexpr uint32_t TIME_CLOCK_SLOT_COUNT = 4;  // index by time_clock_id_t value

struct time_clock_state_t {
  volatile uint32_t seq = 0;
  bool     valid = false;
  bool     prediction_valid = false;
  uint32_t dwt_at_update = 0;
  uint64_t ns_at_update = 0;
  uint32_t predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
  uint32_t update_count = 0;
  uint32_t last_observed_dwt_cycles = 0;
  uint64_t last_observed_ns = 0;
  int32_t  last_prediction_residual_cycles = 0;
};

static time_clock_state_t time_clocks[TIME_CLOCK_SLOT_COUNT] = {};

static int time_clock_index(time_clock_id_t clock) {
  const uint8_t v = (uint8_t)clock;
  if (v == (uint8_t)time_clock_id_t::VCLOCK) return 1;
  if (v == (uint8_t)time_clock_id_t::OCXO1) return 2;
  if (v == (uint8_t)time_clock_id_t::OCXO2) return 3;
  return -1;
}

static bool time_clock_load(time_clock_id_t clock, time_clock_snapshot_t& out) {
  out = time_clock_snapshot_t{};
  const int idx = time_clock_index(clock);
  if (idx < 0) return false;

  const time_clock_state_t& c = time_clocks[idx];
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = c.seq;
    dmb();

    out.valid = c.valid;
    out.prediction_valid = c.prediction_valid;
    out.dwt_at_update = c.dwt_at_update;
    out.ns_at_update = c.ns_at_update;
    out.predicted_dwt_cycles_per_second = c.predicted_dwt_cycles_per_second;
    out.update_count = c.update_count;
    out.last_observed_dwt_cycles = c.last_observed_dwt_cycles;
    out.last_observed_ns = c.last_observed_ns;
    out.last_prediction_residual_cycles = c.last_prediction_residual_cycles;

    dmb();
    const uint32_t s2 = c.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return true;
  }

  out = time_clock_snapshot_t{};
  return false;
}

void time_clock_reset_all(void) {
  for (uint32_t i = 0; i < TIME_CLOCK_SLOT_COUNT; i++) {
    time_clock_state_t& c = time_clocks[i];
    c.seq++;
    dmb();

    c.valid = false;
    c.prediction_valid = false;
    c.dwt_at_update = 0;
    c.ns_at_update = 0;
    c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
    c.update_count = 0;
    c.last_observed_dwt_cycles = 0;
    c.last_observed_ns = 0;
    c.last_prediction_residual_cycles = 0;

    dmb();
    c.seq++;
  }
}

bool time_clock_epoch_reset(time_clock_id_t clock,
                            uint32_t dwt_at_update,
                            uint64_t ns_at_update) {
  const int idx = time_clock_index(clock);
  if (idx < 0) return false;
  time_clock_state_t& c = time_clocks[idx];

  c.seq++;
  dmb();

  c.valid = true;
  c.prediction_valid = true;
  c.dwt_at_update = dwt_at_update;
  c.ns_at_update = ns_at_update;
  c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
  c.update_count = 0;
  c.last_observed_dwt_cycles = 0;
  c.last_observed_ns = 0;
  c.last_prediction_residual_cycles = 0;

  dmb();
  c.seq++;
  return true;
}

bool time_clock_update(time_clock_id_t clock,
                       uint32_t dwt_at_update,
                       uint64_t ns_at_update) {
  const int idx = time_clock_index(clock);
  if (idx < 0) return false;
  time_clock_state_t& c = time_clocks[idx];

  c.seq++;
  dmb();

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
      c.last_prediction_residual_cycles = c.prediction_valid
          ? (int32_t)((int64_t)observed_cps -
                      (int64_t)c.predicted_dwt_cycles_per_second)
          : 0;
      c.predicted_dwt_cycles_per_second =
          observed_cps ? observed_cps : DWT_EXPECTED_PER_PPS;
      c.prediction_valid = true;
      c.last_observed_dwt_cycles = observed_dwt;
      c.last_observed_ns = observed_ns;
    }
  } else {
    c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
    c.prediction_valid = true;
    c.last_prediction_residual_cycles = 0;
    c.last_observed_dwt_cycles = 0;
    c.last_observed_ns = 0;
  }

  c.valid = true;
  c.dwt_at_update = dwt_at_update;
  c.ns_at_update = ns_at_update;
  c.update_count++;

  dmb();
  c.seq++;
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
  anchor.seq++;
  dmb();

  anchor.dwt_at_pps_vclock = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s = 0;
  anchor.counter32_at_pps_vclock = counter32_at_pps_vclock;
  anchor.pps_vclock_count = 1;
  anchor.valid = false;

  dmb();
  anchor.seq++;
}

static void time_pps_vclock_update_explicit(uint32_t dwt_at_pps_vclock,
                                            uint32_t dwt_cycles_per_pps_vclock_s,
                                            uint32_t counter32_at_pps_vclock,
                                            uint32_t pps_vclock_count) {
  const uint32_t new_pps_vclock_count = pps_vclock_count ? pps_vclock_count : 1U;

  anchor.seq++;
  dmb();

  anchor.dwt_at_pps_vclock = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s = dwt_cycles_per_pps_vclock_s;
  anchor.counter32_at_pps_vclock = counter32_at_pps_vclock;
  anchor.pps_vclock_count = new_pps_vclock_count;
  anchor.valid = dwt_cycles_per_pps_vclock_s > 0;

  dmb();
  anchor.seq++;
}

// Historical 3-argument order used by CLOCKS/Alpha:
//   dwt, cycles_per_second, counter32
void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock) {
  const uint32_t next_count = anchor.pps_vclock_count
      ? (anchor.pps_vclock_count + 1U)
      : 1U;

  time_pps_vclock_update_explicit(dwt_at_pps_vclock,
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
  time_pps_vclock_update_explicit(dwt_at_pps_vclock,
                                  dwt_cycles_per_pps_vclock_s,
                                  counter32_at_pps_vclock,
                                  pps_vclock_count);
}

// ============================================================================
// GNSS interpolation helper
// ============================================================================

static inline int64_t interpolate_gnss_ns(const time_snapshot_t& s,
                                          uint32_t dwt_elapsed) {
  const uint32_t cycles = effective_cycles_per_snapshot(s);
  if (cycles == 0) return -1;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * TIME_NS_PER_SECOND_U64 +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(s.pps_vclock_count - 1) *
                   TIME_NS_PER_SECOND_U64 +
                   ns_into_second);
}

int64_t time_gnss_ns_now(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (effective_cycles_per_snapshot(s) == 0) return -1;

  const uint32_t dwt_elapsed = ARM_DWT_CYCCNT - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

// ============================================================================
// Status / init
// ============================================================================

uint32_t time_pps_count(void) {
  const time_snapshot_t s = read_anchor();
  return s.ok ? s.pps_vclock_count : 0;
}

bool time_valid(void) {
  const time_snapshot_t s = read_anchor();
  return s.ok && s.valid && effective_cycles_per_snapshot(s) > 0;
}

void time_init(void) {
  anchor = {};
  time_clock_reset_all();
}

void process_time_init(void) {
  time_init();
}

// ============================================================================
// Report
// ============================================================================

static void add_clock_snapshot(Payload& parent,
                               const char* key,
                               time_clock_id_t clock) {
  time_clock_snapshot_t s{};
  const bool load_ok = time_clock_snapshot(clock, &s);

  Payload p;
  p.add("load_ok", load_ok);
  p.add("valid", load_ok && s.valid);
  p.add("prediction_valid", load_ok && s.prediction_valid);
  p.add("dwt_at_update", load_ok ? s.dwt_at_update : 0U);
  p.add("ns_at_update", load_ok ? s.ns_at_update : 0ULL);
  p.add("predicted_dwt_cycles_per_second",
        load_ok ? s.predicted_dwt_cycles_per_second : 0U);
  p.add("update_count", load_ok ? s.update_count : 0U);
  p.add("last_observed_dwt_cycles",
        load_ok ? s.last_observed_dwt_cycles : 0U);
  p.add("last_observed_ns", load_ok ? s.last_observed_ns : 0ULL);
  p.add("last_prediction_residual_cycles",
        load_ok ? s.last_prediction_residual_cycles : 0);

  parent.add_object(key, p);
}

static Payload cmd_report(const Payload&) {
  const time_anchor_snapshot_t a = time_anchor_snapshot();

  Payload out;
  out.add("model", "TIME_COMPAT_ANCHOR_PROJECTION_REPORT");
  out.add("note", "process_time owns only the PPS/VCLOCK anchor and per-clock projection backing store");
  out.add("time_valid", time_valid());

  Payload anchor_payload;
  anchor_payload.add("ok", a.ok);
  anchor_payload.add("valid", a.valid);
  anchor_payload.add("dwt_at_pps_vclock", a.dwt_at_pps_vclock);
  anchor_payload.add("dwt_cycles_per_pps_vclock_s", a.dwt_cycles_per_pps_vclock_s);
  anchor_payload.add("counter32_at_pps_vclock", a.counter32_at_pps_vclock);
  anchor_payload.add("pps_vclock_count", a.pps_vclock_count);
  out.add_object("anchor", anchor_payload);

  Payload projections;
  add_clock_snapshot(projections, "vclock", time_clock_id_t::VCLOCK);
  add_clock_snapshot(projections, "ocxo1", time_clock_id_t::OCXO1);
  add_clock_snapshot(projections, "ocxo2", time_clock_id_t::OCXO2);
  out.add_object("projections", projections);

  out.add("prediction_surfaces_retired", true);
  out.add("dynamic_cps_retired", true);
  out.add("retired_dead_code_removed", true);
  return out;
}

static const process_command_entry_t TIME_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr    }
};

static const process_vtable_t TIME_PROCESS = {
  .process_id    = "TIME",
  .commands      = TIME_COMMANDS,
  .subscriptions = nullptr,
};

void process_time_register(void) {
  process_register("TIME", &TIME_PROCESS);
}
