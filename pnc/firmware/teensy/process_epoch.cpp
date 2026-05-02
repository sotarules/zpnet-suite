// ============================================================================
// process_epoch.cpp — FLAT_REPORT_V5_DWT64_CYCLES
// ============================================================================
//
// EPOCH — logical zero / epoch custody process.
//
// FLAT_REPORT_V5_DWT64_CYCLES:
//   • all reports are flat single-Payload command handlers
//   • adds CLOCKS report: one DWT sample, one expected 10 MHz tick value,
//     and actual/delta clock observations
//   • DWT64 is reported only as cycles; no logical DWT nanosecond clock
//
// Doctrine:
//
//   ZERO does not write literal zero into every clock.
//
//   ZERO defines a single logical epoch event. DWT is the reference ruler and
//   receives an epoch coordinate. Each writable synthetic clock is initialized
//   to the value it must have at its actual installation instant so that
//   back-projecting to the epoch event yields zero.
//
// Required integration:
//
//   • teensy.cpp calls process_epoch_init() and process_epoch_register().
//   • CLOCKS ZERO delegates to process_epoch_request_zero(...).
//   • The PPS/VCLOCK snapshot path calls
//       process_epoch_on_pps_vclock_snapshot(snap)
//     when process_interrupt publishes a stable selected-edge snapshot.
// ============================================================================

#include "process_epoch.h"
#include "process_timepop.h"
#include "process_interrupt.h"

#include "config.h"
#include "process.h"
#include "payload.h"
#include "time.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
// Assumed CLOCKS DWT64 API
// ============================================================================
//
// process_epoch treats DWT64 as a first-class logical clock.  CLOCKS/alpha
// owns the physical/extended implementation.  The epoch process only asks
// CLOCKS to install the DWT64 epoch origin and to read the resulting logical
// DWT64 cycle count.
//
// Expected process_clocks API:
//   bool clocks_dwt64_epoch_reset_at_dwt32(uint32_t epoch_dwt32,
//                                          uint64_t* out_raw_epoch_dwt64);
//   uint64_t clocks_dwt_cycles_now(void);
// //
// Semantics expected by process_epoch after reset:
//   clocks_dwt_cycles_now() returns the logical epoch-relative DWT64 cycles.

extern bool clocks_dwt64_epoch_reset_at_dwt32(uint32_t epoch_dwt32,
                                              uint64_t* out_raw_epoch_dwt64);
extern uint64_t clocks_dwt_cycles_now(void);

// ============================================================================
// Optional future hooks
// ============================================================================
//
// The CLOCKS report can already read VCLOCK counter32 directly. OCXO synthetic
// 32-bit values still need a richer public interrupt accessor. Until that is
// added, the weak snapshot hook lets process_interrupt provide exact synthetic
// snapshots later without changing this process_epoch command surface.

bool process_epoch_clock_snapshot_hook(epoch_clock_id_t clock_id,
                                       epoch_clock_snapshot_t* out)
    __attribute__((weak));

bool process_epoch_clock_snapshot_hook(epoch_clock_id_t,
                                       epoch_clock_snapshot_t*) {
  return false;
}

void clocks_alpha_epoch_installed(const epoch_fact_t& fact) __attribute__((weak));
void clocks_alpha_epoch_installed(const epoch_fact_t&) {}

void clocks_beta_epoch_installed(const epoch_fact_t& fact) __attribute__((weak));
void clocks_beta_epoch_installed(const epoch_fact_t&) {}

// ============================================================================
// Constants / state
// ============================================================================

static constexpr uint32_t EPOCH_HISTORY_CAPACITY = 4;
static constexpr uint32_t NEAR_BOUNDARY_NS = 10;
static constexpr uint32_t INSTALL_ELAPSED_BUDGET_NS = 10000;

static epoch_state_t g_state = epoch_state_t::IDLE;
static uint32_t g_next_request_id = 1;
static uint32_t g_epoch_sequence = 0;

static epoch_fact_t  g_fact = {};
static epoch_audit_t g_audit = {};
static epoch_history_entry_t g_history[EPOCH_HISTORY_CAPACITY] = {};
static uint32_t g_history_head = 0;
static uint32_t g_history_count = 0;

static uint32_t g_last_request_id = 0;
static epoch_reason_t g_pending_reason = epoch_reason_t::UNKNOWN;
static uint64_t g_pending_epoch_ns = 0;

// DWT64 logical clock custody.  DWT64 is the one clock process_epoch cannot
// physically write.  Its zero is installed by asking CLOCKS/alpha to store
// the raw DWT64 origin corresponding to the selected DWT32 epoch edge.
static bool     g_dwt64_epoch_valid = false;
static bool     g_dwt64_epoch_reset_ok = false;
static uint64_t g_dwt64_raw_epoch_cycles = 0;
static uint32_t g_dwt64_epoch_reset_count = 0;
static uint32_t g_dwt64_epoch_reset_failures = 0;
static uint64_t g_dwt64_last_logical_cycles = 0;

// ============================================================================
// String helpers
// ============================================================================

const char* epoch_reason_str(epoch_reason_t reason) {
  switch (reason) {
    case epoch_reason_t::STARTUP:             return "STARTUP";
    case epoch_reason_t::EPOCH_ZERO_COMMAND:  return "EPOCH_ZERO_COMMAND";
    case epoch_reason_t::CLOCKS_ZERO_COMMAND: return "CLOCKS_ZERO_COMMAND";
    case epoch_reason_t::CLOCKS_START:        return "CLOCKS_START";
    case epoch_reason_t::CLOCKS_RECOVER:      return "CLOCKS_RECOVER";
    case epoch_reason_t::INTERNAL:            return "INTERNAL";
    default:                                  return "UNKNOWN";
  }
}

const char* epoch_state_str(epoch_state_t state) {
  switch (state) {
    case epoch_state_t::IDLE:            return "IDLE";
    case epoch_state_t::REQUESTED:       return "REQUESTED";
    case epoch_state_t::INTERRUPT_ARMED: return "INTERRUPT_ARMED";
    case epoch_state_t::FACT_AUTHORED:   return "FACT_AUTHORED";
    case epoch_state_t::INSTALLING:      return "INSTALLING";
    case epoch_state_t::FINALIZED:       return "FINALIZED";
    case epoch_state_t::FAULT:           return "FAULT";
    default:                             return "UNKNOWN";
  }
}

const char* epoch_clock_id_str(epoch_clock_id_t clock_id) {
  switch (clock_id) {
    case epoch_clock_id_t::DWT:          return "DWT";
    case epoch_clock_id_t::VCLOCK:       return "VCLOCK";
    case epoch_clock_id_t::OCXO1:        return "OCXO1";
    case epoch_clock_id_t::OCXO2:        return "OCXO2";
    case epoch_clock_id_t::TIME:         return "TIME";
    case epoch_clock_id_t::TIMEPOP:      return "TIMEPOP";
    case epoch_clock_id_t::CLOCKS_ALPHA: return "CLOCKS_ALPHA";
    case epoch_clock_id_t::CLOCKS_BETA:  return "CLOCKS_BETA";
    default:                             return "NONE";
  }
}

// ============================================================================
// Utility
// ============================================================================

static inline uint32_t dwt_now(void) {
  return ARM_DWT_CYCCNT;
}

static inline uint32_t abs_i32_to_u32(int32_t v) {
  return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

static uint64_t dwt_cycles_to_ns(uint64_t cycles, uint32_t cps) {
  if (cps != 0) {
    return ((uint64_t)cycles * (uint64_t)NS_PER_SECOND + (uint64_t)cps / 2ULL) /
           (uint64_t)cps;
  }

  return ((uint64_t)cycles * (uint64_t)DWT_NS_NUM + (uint64_t)DWT_NS_DEN / 2ULL) /
         (uint64_t)DWT_NS_DEN;
}

static uint32_t saturated_u64_to_u32(uint64_t v) {
  return (v > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)v;
}

static uint64_t dwt64_logical_cycles_now(void) {
  if (!g_dwt64_epoch_valid) {
    if (!g_fact.valid) return 0;
    const uint64_t cycles = (uint32_t)(dwt_now() - g_fact.epoch_dwt);
    g_dwt64_last_logical_cycles = cycles;
    return cycles;
  }

  const uint64_t cycles = clocks_dwt_cycles_now();
  g_dwt64_last_logical_cycles = cycles;
  return cycles;
}

static uint32_t ns_to_10mhz_ticks_rounded(uint64_t ns) {
  return (uint32_t)((ns + (uint64_t)NS_PER_10MHZ_TICK / 2ULL) /
                    (uint64_t)NS_PER_10MHZ_TICK);
}

static uint32_t dwt64_cycles_to_10mhz_ticks(uint64_t cycles) {
  // DWT64 remains a cycle-count clock.  This helper derives the comparable
  // 10 MHz tick count directly from cycles, without constructing a logical
  // DWT nanosecond clock.
  return (uint32_t)((cycles * (uint64_t)VCLOCK_COUNTS_PER_SECOND +
                     (uint64_t)DWT_EXPECTED_PER_PPS / 2ULL) /
                    (uint64_t)DWT_EXPECTED_PER_PPS);
}

static uint32_t elapsed_ticks_since_epoch(uint32_t /*now_dwt*/) {
  if (!g_fact.valid) return 0;
  return dwt64_cycles_to_10mhz_ticks(dwt64_logical_cycles_now());
}

static uint16_t expected_low16_from_counter32(uint32_t counter32) {
  return (uint16_t)(counter32 & 0xFFFFU);
}

static int32_t delta32(uint32_t actual, uint32_t expected) {
  return (int32_t)(actual - expected);
}

static int32_t delta16(uint16_t actual, uint16_t expected) {
  return (int32_t)((int16_t)(actual - expected));
}

static uint32_t counter32_from_low16(uint32_t expected_counter32,
                                     uint16_t actual_low16) {
  const uint16_t expected_low16 = expected_low16_from_counter32(expected_counter32);
  const int16_t low_delta = (int16_t)(actual_low16 - expected_low16);
  return expected_counter32 + (int32_t)low_delta;
}

static void reset_fact_and_audit_for_request(uint32_t request_id,
                                             epoch_reason_t reason,
                                             uint64_t epoch_ns,
                                             uint32_t request_dwt) {
  g_fact = epoch_fact_t{};
  g_fact.request_id = request_id;
  g_fact.reason = reason;
  g_fact.epoch_ns = epoch_ns;

  g_audit = epoch_audit_t{};
  g_audit.valid = true;
  g_audit.request_id = request_id;
  g_audit.reason = reason;
  g_audit.request_dwt = request_dwt;
}

static void push_history(void) {
  epoch_history_entry_t h{};
  h.valid = g_fact.valid;
  h.request_id = g_fact.request_id;
  h.epoch_sequence = g_fact.epoch_sequence;
  h.reason = g_fact.reason;
  h.epoch_dwt = g_fact.epoch_dwt;
  h.vclock_counter32_at_epoch = g_fact.vclock_counter32_at_epoch;
  h.install_count = g_audit.install_count;
  h.coherent_install_count = g_audit.coherent_install_count;
  h.back_projection_mismatch_count = g_audit.back_projection_mismatch_count;
  h.clock_write_failure_count = g_audit.clock_write_failure_count;
  h.finalized = (g_state == epoch_state_t::FINALIZED);

  g_history[g_history_head] = h;
  g_history_head = (g_history_head + 1U) % EPOCH_HISTORY_CAPACITY;
  if (g_history_count < EPOCH_HISTORY_CAPACITY) g_history_count++;
}

static void note_install(epoch_clock_install_t& install) {
  g_audit.install_count++;
  if (install.coherent) g_audit.coherent_install_count++;

  if (install.elapsed_ns_at_install > (uint64_t)g_audit.max_install_elapsed_ns) {
    g_audit.max_install_elapsed_ns = (uint32_t)install.elapsed_ns_at_install;
  }

  const uint32_t abs_back_error =
      abs_i32_to_u32(install.back_projection_error_ticks);
  if (abs_back_error > g_audit.max_abs_back_projection_error_ticks) {
    g_audit.max_abs_back_projection_error_ticks = abs_back_error;
  }

  if (install.back_projection_error_ticks != 0) {
    g_audit.back_projection_mismatch_count++;
  }
  if (!install.write_ok) {
    g_audit.clock_write_failure_count++;
  }

  if (install.phase_ns_within_tick <= NEAR_BOUNDARY_NS ||
      install.distance_to_next_boundary_ns <= NEAR_BOUNDARY_NS) {
    g_audit.near_boundary_install_count++;
  }

  if (install.elapsed_ns_at_install > INSTALL_ELAPSED_BUDGET_NS) {
    g_audit.install_elapsed_over_budget_count++;
  }
}

static epoch_clock_install_t make_install_record(epoch_clock_id_t clock_id,
                                                 uint32_t install_dwt) {
  epoch_clock_install_t r{};
  r.attempted = true;
  r.clock_id = clock_id;
  r.install_dwt = install_dwt;

  uint64_t elapsed_cycles64 = 0;
  if (clock_id == epoch_clock_id_t::DWT && install_dwt == g_fact.epoch_dwt) {
    elapsed_cycles64 = 0;
  } else {
    elapsed_cycles64 = dwt64_logical_cycles_now();
  }

  r.elapsed_dwt_cycles = saturated_u64_to_u32(elapsed_cycles64);

  r.installed_counter32 = dwt64_cycles_to_10mhz_ticks(elapsed_cycles64);
  r.installed_low16 = expected_low16_from_counter32(r.installed_counter32);

  // The public interrupt_clock32 API still accepts nanoseconds, so derive only
  // the setter value from the 10 MHz tick count.  DWT64 itself remains a cycle
  // clock; process_epoch no longer exposes or reasons over DWT64-as-ns.
  r.elapsed_ns_at_install =
      (uint64_t)r.installed_counter32 * (uint64_t)NS_PER_10MHZ_TICK;

  r.back_projected_epoch_counter32 =
      r.installed_counter32 - dwt64_cycles_to_10mhz_ticks(elapsed_cycles64);
  r.back_projection_error_ticks = (int32_t)r.back_projected_epoch_counter32;

  r.phase_ns_within_tick = 0;
  r.distance_to_previous_boundary_ns = 0;
  r.distance_to_next_boundary_ns = NS_PER_10MHZ_TICK;
  if (r.phase_ns_within_tick == 0) {
    r.distance_to_next_boundary_ns = NS_PER_10MHZ_TICK;
  }

  r.coherent = (r.back_projection_error_ticks == 0);
  return r;
}

static bool install_synthetic_clock(epoch_clock_id_t clock_id,
                                    interrupt_subscriber_kind_t kind,
                                    epoch_clock_install_t& out_install) {
  const uint32_t install_dwt = dwt_now();
  out_install = make_install_record(clock_id, install_dwt);

  const uint64_t install_ns =
      (uint64_t)out_install.installed_counter32 * (uint64_t)NS_PER_10MHZ_TICK;

  out_install.write_ok = interrupt_clock32_zero_from_ns(kind, install_ns);
  out_install.coherent = out_install.coherent && out_install.write_ok;
  note_install(out_install);
  return out_install.write_ok;
}

static bool install_dwt64_epoch_origin(uint32_t epoch_dwt32) {
  g_dwt64_epoch_reset_count++;
  g_dwt64_raw_epoch_cycles = 0;
  g_dwt64_epoch_valid = false;
  g_dwt64_epoch_reset_ok = false;

  uint64_t raw_epoch = 0;
  const bool ok = clocks_dwt64_epoch_reset_at_dwt32(epoch_dwt32, &raw_epoch);
  if (!ok) {
    g_dwt64_epoch_reset_failures++;
    return false;
  }

  g_dwt64_raw_epoch_cycles = raw_epoch;
  g_dwt64_epoch_valid = true;
  g_dwt64_epoch_reset_ok = true;
  g_dwt64_last_logical_cycles = 0;
  return true;
}

// ============================================================================
// Epoch install
// ============================================================================

static bool install_epoch_from_snapshot(const pps_edge_snapshot_t& snap) {
  g_state = epoch_state_t::FACT_AUTHORED;
  g_audit.fact_authored_dwt = dwt_now();

  g_fact.valid = true;
  g_fact.epoch_sequence = ++g_epoch_sequence;
  g_fact.epoch_dwt = snap.dwt_at_edge;
  // DWT64 is a cycle-count clock.  Keep the cycle-to-10MHz comparison
  // anchored to the nominal DWT cycles-per-second constant; do not build a
  // logical DWT nanosecond clock from dynamic calibration here.
  g_fact.dwt_cycles_per_second_seed = DWT_EXPECTED_PER_PPS;
  g_fact.vclock_counter32_at_epoch = snap.counter32_at_edge;
  g_fact.vclock_low16_at_epoch = snap.ch3_at_edge;
  g_fact.pps_sequence = snap.sequence;
  g_fact.physical_pps_dwt_at_edge = snap.physical_pps_dwt_normalized_at_edge;
  g_fact.physical_pps_counter32_at_read = snap.physical_pps_counter32_at_read;
  g_fact.physical_pps_low16_at_read = snap.physical_pps_ch3_at_read;

  g_audit.epoch_sequence = g_fact.epoch_sequence;
  g_audit.pps_event_dwt = snap.physical_pps_dwt_normalized_at_edge;
  g_audit.selected_edge_dwt = snap.dwt_at_edge;
  g_audit.selected_vclock_counter32 = snap.counter32_at_edge;
  g_audit.selected_vclock_low16 = snap.ch3_at_edge;

  if (!install_dwt64_epoch_origin(g_fact.epoch_dwt)) {
    g_state = epoch_state_t::FAULT;
    g_audit.clock_write_failure_count++;
    return false;
  }

  g_state = epoch_state_t::INSTALLING;
  g_audit.install_begin_dwt = dwt_now();

  g_audit.dwt = make_install_record(epoch_clock_id_t::DWT, g_fact.epoch_dwt);
  g_audit.dwt.write_ok = true;
  g_audit.dwt.coherent = true;
  note_install(g_audit.dwt);

  time_pps_vclock_epoch_reset(g_fact.epoch_dwt, g_fact.vclock_counter32_at_epoch);

  g_audit.vclock = make_install_record(epoch_clock_id_t::VCLOCK, dwt_now());
  g_audit.vclock.write_ok = true;
  g_audit.vclock.coherent = g_audit.vclock.coherent;
  note_install(g_audit.vclock);

  install_synthetic_clock(epoch_clock_id_t::OCXO1,
                          interrupt_subscriber_kind_t::OCXO1,
                          g_audit.ocxo1);
  install_synthetic_clock(epoch_clock_id_t::OCXO2,
                          interrupt_subscriber_kind_t::OCXO2,
                          g_audit.ocxo2);

  clocks_alpha_epoch_installed(g_fact);
  timepop_epoch_changed(g_fact.epoch_sequence);
  clocks_beta_epoch_installed(g_fact);

  g_audit.install_end_dwt = dwt_now();
  g_audit.finalize_dwt = g_audit.install_end_dwt;
  g_state = epoch_state_t::FINALIZED;
  push_history();
  return true;
}

// ============================================================================
// Public API
// ============================================================================

void process_epoch_init(void) {
  g_state = epoch_state_t::IDLE;
  g_next_request_id = 1;
  g_epoch_sequence = 0;
  g_fact = epoch_fact_t{};
  g_audit = epoch_audit_t{};
  for (uint32_t i = 0; i < EPOCH_HISTORY_CAPACITY; i++) {
    g_history[i] = epoch_history_entry_t{};
  }
  g_history_head = 0;
  g_history_count = 0;
  g_last_request_id = 0;
  g_pending_reason = epoch_reason_t::UNKNOWN;
  g_pending_epoch_ns = 0;
  g_dwt64_epoch_valid = false;
  g_dwt64_epoch_reset_ok = false;
  g_dwt64_raw_epoch_cycles = 0;
  g_dwt64_epoch_reset_count = 0;
  g_dwt64_epoch_reset_failures = 0;
  g_dwt64_last_logical_cycles = 0;
}

epoch_request_result_t process_epoch_request_zero(epoch_reason_t reason) {
  epoch_request_result_t r{};
  r.accepted = false;
  r.request_id = g_last_request_id;
  r.state = g_state;

  if (g_state == epoch_state_t::REQUESTED ||
      g_state == epoch_state_t::INTERRUPT_ARMED ||
      g_state == epoch_state_t::FACT_AUTHORED ||
      g_state == epoch_state_t::INSTALLING) {
    r.message = "epoch_request_already_pending";
    return r;
  }

  const uint32_t request_id = g_next_request_id++;
  if (g_next_request_id == 0) g_next_request_id = 1;

  const uint32_t request_dwt = dwt_now();
  g_last_request_id = request_id;
  g_pending_reason = reason;
  g_pending_epoch_ns = 0;

  reset_fact_and_audit_for_request(request_id, reason, g_pending_epoch_ns, request_dwt);
  g_state = epoch_state_t::REQUESTED;

  const bool requested =
      interrupt_clock32_request_zero_from_ns(interrupt_subscriber_kind_t::VCLOCK,
                                             g_pending_epoch_ns);
  if (!requested) {
    g_state = epoch_state_t::FAULT;
    r.accepted = false;
    r.request_id = request_id;
    r.state = g_state;
    r.message = "interrupt_clock32_request_zero_failed";
    return r;
  }

  g_audit.interrupt_arm_dwt = dwt_now();
  interrupt_request_pps_rebootstrap();
  g_state = epoch_state_t::INTERRUPT_ARMED;

  r.accepted = true;
  r.request_id = request_id;
  r.state = g_state;
  r.message = "epoch_zero_armed";
  return r;
}

bool process_epoch_on_pps_vclock_snapshot(const pps_edge_snapshot_t& snap) {
  if (g_state != epoch_state_t::INTERRUPT_ARMED &&
      g_state != epoch_state_t::FACT_AUTHORED) {
    return false;
  }

  if (interrupt_pps_rebootstrap_pending()) {
    return false;
  }

  return install_epoch_from_snapshot(snap);
}

epoch_state_t process_epoch_state(void) { return g_state; }

bool process_epoch_pending(void) {
  return g_state == epoch_state_t::REQUESTED ||
         g_state == epoch_state_t::INTERRUPT_ARMED ||
         g_state == epoch_state_t::FACT_AUTHORED ||
         g_state == epoch_state_t::INSTALLING;
}

bool process_epoch_finalized(void) { return g_state == epoch_state_t::FINALIZED; }

uint32_t process_epoch_current_sequence(void) { return g_fact.epoch_sequence; }

uint32_t process_epoch_last_request_id(void) { return g_last_request_id; }

epoch_fact_t process_epoch_current_fact(void) { return g_fact; }

epoch_audit_t process_epoch_current_audit(void) { return g_audit; }

uint64_t process_epoch_dwt64_logical_cycles_now(void) {
  return dwt64_logical_cycles_now();
}

uint64_t process_epoch_dwt64_raw_epoch_cycles(void) {
  return g_dwt64_raw_epoch_cycles;
}

bool process_epoch_dwt64_epoch_valid(void) {
  return g_dwt64_epoch_valid;
}

// ============================================================================
// Flat report helpers
// ============================================================================

static void add_fact_fields(Payload& out, const char* prefix, const epoch_fact_t& f) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_str = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value ? value : "");
  };

  add_bool("valid", f.valid);
  add_u32("request_id", f.request_id);
  add_u32("sequence", f.epoch_sequence);
  add_str("reason", epoch_reason_str(f.reason));
  add_u64("ns", f.epoch_ns);
  add_u32("dwt", f.epoch_dwt);
  add_u32("dwt_cycles_per_second_seed", f.dwt_cycles_per_second_seed);
  add_u32("vclock_counter32", f.vclock_counter32_at_epoch);
  add_u32("vclock_low16", (uint32_t)f.vclock_low16_at_epoch);
  add_u32("pps_sequence", f.pps_sequence);
  add_u32("physical_pps_dwt_at_edge", f.physical_pps_dwt_at_edge);
  add_u32("physical_pps_counter32_at_read", f.physical_pps_counter32_at_read);
  add_u32("physical_pps_low16_at_read", (uint32_t)f.physical_pps_low16_at_read);
}

static void add_install_fields(Payload& out, const char* prefix, const epoch_clock_install_t& r) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_str = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value ? value : "");
  };

  add_bool("attempted", r.attempted);
  add_bool("write_ok", r.write_ok);
  add_bool("coherent", r.coherent);
  add_str("clock_id", epoch_clock_id_str(r.clock_id));
  add_u32("install_dwt", r.install_dwt);
  add_u32("elapsed_dwt_cycles", r.elapsed_dwt_cycles);
  add_u64("elapsed_ns_at_install", r.elapsed_ns_at_install);
  add_u32("installed_counter32", r.installed_counter32);
  add_u32("installed_low16", (uint32_t)r.installed_low16);
  add_u32("back_projected_epoch_counter32", r.back_projected_epoch_counter32);
  add_i32("back_projection_error_ticks", r.back_projection_error_ticks);
  add_u32("phase_ns_within_tick", r.phase_ns_within_tick);
  add_u32("distance_to_previous_boundary_ns", r.distance_to_previous_boundary_ns);
  add_u32("distance_to_next_boundary_ns", r.distance_to_next_boundary_ns);
}

static void add_history_entry_fields(Payload& out, const char* prefix, const epoch_history_entry_t& h) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_str = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value ? value : "");
  };

  add_bool("valid", h.valid);
  add_u32("request_id", h.request_id);
  add_u32("epoch_sequence", h.epoch_sequence);
  add_str("reason", epoch_reason_str(h.reason));
  add_u32("epoch_dwt", h.epoch_dwt);
  add_u32("vclock_counter32_at_epoch", h.vclock_counter32_at_epoch);
  add_u32("install_count", h.install_count);
  add_u32("coherent_install_count", h.coherent_install_count);
  add_u32("back_projection_mismatch_count", h.back_projection_mismatch_count);
  add_u32("clock_write_failure_count", h.clock_write_failure_count);
  add_bool("finalized", h.finalized);
}

static void add_summary_fields(Payload& out) {
  out.add("install_count", g_audit.install_count);
  out.add("coherent_install_count", g_audit.coherent_install_count);
  out.add("max_install_elapsed_ns", g_audit.max_install_elapsed_ns);
  out.add("max_abs_back_projection_error_ticks",
          g_audit.max_abs_back_projection_error_ticks);
  out.add("near_boundary_install_count", g_audit.near_boundary_install_count);
  out.add("install_elapsed_over_budget_count", g_audit.install_elapsed_over_budget_count);
  out.add("back_projection_mismatch_count", g_audit.back_projection_mismatch_count);
  out.add("clock_write_failure_count", g_audit.clock_write_failure_count);
  out.add("epoch_generation_mismatch_count", g_audit.epoch_generation_mismatch_count);
  out.add("all_installs_coherent",
          g_audit.install_count > 0 &&
          g_audit.install_count == g_audit.coherent_install_count &&
          g_audit.back_projection_mismatch_count == 0 &&
          g_audit.clock_write_failure_count == 0);
}

static void add_live_clock_fields(Payload& out,
                                  const char* prefix,
                                  uint32_t expected_counter32,
                                  bool counter32_available,
                                  uint32_t actual_counter32,
                                  bool low16_available,
                                  uint16_t actual_low16) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    out.add(key, value);
  };

  const uint16_t expected_low16 = expected_low16_from_counter32(expected_counter32);
  const int32_t counter32_delta =
      counter32_available ? delta32(actual_counter32, expected_counter32) : 0;
  const int32_t low16_delta =
      low16_available ? delta16(actual_low16, expected_low16) : 0;

  add_u32("expected_counter32", expected_counter32);
  add_u32("expected_low16", (uint32_t)expected_low16);

  add_bool("counter32_available", counter32_available);
  add_u32("actual_counter32", counter32_available ? actual_counter32 : 0);
  add_i32("counter32_delta_ticks", counter32_delta);

  add_bool("low16_available", low16_available);
  add_u32("actual_low16", low16_available ? (uint32_t)actual_low16 : 0);
  add_i32("low16_delta_ticks", low16_delta);

  add_bool("coherent",
           (!counter32_available || counter32_delta == 0) &&
           (!low16_available || low16_delta == 0));
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_zero(const Payload& args) {
  (void)args;
  const epoch_request_result_t r =
      process_epoch_request_zero(epoch_reason_t::EPOCH_ZERO_COMMAND);

  Payload out;
  out.add("model", "EPOCH_ZERO");
  out.add("report_style", "flat_v5_dwt64_cycles");
  out.add("accepted", r.accepted);
  out.add("request_id", r.request_id);
  out.add("state", epoch_state_str(r.state));
  out.add("message", r.message);
  return out;
}

static Payload cmd_status(const Payload&) {
  Payload out;
  out.add("model", "EPOCH_STATUS");
  out.add("report_style", "flat_v5_dwt64_cycles");
  out.add("valid", g_fact.valid);
  out.add("state", epoch_state_str(g_state));
  out.add("pending", process_epoch_pending());
  out.add("finalized", process_epoch_finalized());
  out.add("request_id", g_last_request_id);
  out.add("epoch_sequence", g_fact.epoch_sequence);
  out.add("reason", epoch_reason_str(g_fact.reason));
  out.add("epoch_dwt", g_fact.epoch_dwt);
  out.add("epoch_ns", g_fact.epoch_ns);
  out.add("history_count", g_history_count);
  out.add("dwt64_epoch_valid", g_dwt64_epoch_valid);
  out.add("dwt64_epoch_reset_ok", g_dwt64_epoch_reset_ok);
  out.add("dwt64_raw_epoch_cycles", g_dwt64_raw_epoch_cycles);
  out.add("dwt64_logical_cycles", dwt64_logical_cycles_now());
  return out;
}

static Payload cmd_clocks(const Payload&) {
  Payload out;
  const uint32_t sample_dwt = dwt_now();

  uint64_t elapsed_dwt64_cycles = 0;
  uint32_t elapsed_cycles = 0;
  uint32_t expected_ticks = 0;

  if (g_fact.valid) {
    elapsed_dwt64_cycles = dwt64_logical_cycles_now();
    elapsed_cycles = saturated_u64_to_u32(elapsed_dwt64_cycles);
    expected_ticks = dwt64_cycles_to_10mhz_ticks(elapsed_dwt64_cycles);
  }

  const uint32_t vclock_counter32 = interrupt_qtimer1_counter32_now();
  const uint16_t vclock_low16 =
      expected_low16_from_counter32(vclock_counter32);

  const uint16_t ocxo1_low16 = interrupt_qtimer3_ch2_counter_now();
  const uint16_t ocxo2_low16 = interrupt_qtimer3_ch3_counter_now();

  bool ocxo1_counter32_available = false;
  bool ocxo2_counter32_available = false;
  uint32_t ocxo1_counter32 = counter32_from_low16(expected_ticks, ocxo1_low16);
  uint32_t ocxo2_counter32 = counter32_from_low16(expected_ticks, ocxo2_low16);

  epoch_clock_snapshot_t hook{};
  if (process_epoch_clock_snapshot_hook(epoch_clock_id_t::OCXO1, &hook)) {
    if (hook.actual_counter32_available) {
      ocxo1_counter32_available = true;
      ocxo1_counter32 = hook.actual_counter32;
    }
  }
  hook = epoch_clock_snapshot_t{};
  if (process_epoch_clock_snapshot_hook(epoch_clock_id_t::OCXO2, &hook)) {
    if (hook.actual_counter32_available) {
      ocxo2_counter32_available = true;
      ocxo2_counter32 = hook.actual_counter32;
    }
  }

  out.add("model", "EPOCH_CLOCKS");
  out.add("report_style", "flat_v5_dwt64_cycles");
  out.add("valid", g_fact.valid);
  out.add("state", epoch_state_str(g_state));
  out.add("epoch_sequence", g_fact.epoch_sequence);
  out.add("request_id", g_fact.request_id);

  out.add("sample_dwt", sample_dwt);
  out.add("epoch_dwt", g_fact.epoch_dwt);
  out.add("dwt64_epoch_valid", g_dwt64_epoch_valid);
  out.add("dwt64_epoch_reset_ok", g_dwt64_epoch_reset_ok);
  out.add("dwt64_raw_epoch_cycles", g_dwt64_raw_epoch_cycles);
  out.add("dwt64_logical_cycles", elapsed_dwt64_cycles);
  out.add("elapsed_dwt_cycles", elapsed_cycles);
  out.add("expected_ticks", expected_ticks);
  out.add("dwt_to_10mhz_cycles_per_second", (uint32_t)DWT_EXPECTED_PER_PPS);
  out.add("dwt64_ns_clock_available", false);

  add_live_clock_fields(out, "vclock",
                        expected_ticks,
                        true,
                        vclock_counter32,
                        true,
                        vclock_low16);

  add_live_clock_fields(out, "ocxo1",
                        expected_ticks,
                        ocxo1_counter32_available,
                        ocxo1_counter32,
                        true,
                        ocxo1_low16);

  add_live_clock_fields(out, "ocxo2",
                        expected_ticks,
                        ocxo2_counter32_available,
                        ocxo2_counter32,
                        true,
                        ocxo2_low16);

  out.add("all_counter32_available",
          true && ocxo1_counter32_available && ocxo2_counter32_available);
  out.add("note",
          "OCXO counter32 requires process_epoch_clock_snapshot_hook; low16 is available now");

  return out;
}

static Payload cmd_report(const Payload&) {
  Payload out;
  const uint32_t now_dwt = dwt_now();

  out.add("model", "EPOCH_REPORT");
  out.add("report_style", "flat_v5_dwt64_cycles");
  out.add("valid", g_fact.valid);
  out.add("state", epoch_state_str(g_state));
  out.add("pending", process_epoch_pending());
  out.add("finalized", process_epoch_finalized());
  out.add("request_id", g_last_request_id);
  out.add("epoch_sequence", g_fact.epoch_sequence);
  out.add("reason", epoch_reason_str(g_fact.reason));

  add_fact_fields(out, "epoch", g_fact);

  out.add("now_dwt", now_dwt);
  out.add("dwt64_epoch_valid", g_dwt64_epoch_valid);
  out.add("dwt64_epoch_reset_ok", g_dwt64_epoch_reset_ok);
  out.add("dwt64_raw_epoch_cycles", g_dwt64_raw_epoch_cycles);
  if (g_fact.valid) {
    const uint64_t elapsed_cycles64 = dwt64_logical_cycles_now();
    const uint32_t expected_ticks = dwt64_cycles_to_10mhz_ticks(elapsed_cycles64);
    out.add("now_dwt64_logical_cycles", elapsed_cycles64);
    out.add("now_elapsed_dwt_cycles", saturated_u64_to_u32(elapsed_cycles64));
    out.add("now_expected_10mhz_ticks_since_epoch", expected_ticks);
    out.add("dwt_to_10mhz_cycles_per_second", (uint32_t)DWT_EXPECTED_PER_PPS);
    out.add("dwt64_ns_clock_available", false);
  } else {
    out.add("now_dwt64_logical_cycles", (uint64_t)0);
    out.add("now_elapsed_dwt_cycles", (uint32_t)0);
    out.add("now_expected_10mhz_ticks_since_epoch", (uint32_t)0);
    out.add("dwt_to_10mhz_cycles_per_second", (uint32_t)DWT_EXPECTED_PER_PPS);
    out.add("dwt64_ns_clock_available", false);
  }

  add_summary_fields(out);
  out.add("clock_snapshot_available", true);
  out.add("clock_snapshot_command", "CLOCKS");
  return out;
}

static Payload cmd_audit(const Payload&) {
  Payload out;
  out.add("model", "EPOCH_AUDIT");
  out.add("report_style", "flat_v5_dwt64_cycles");
  out.add("valid", g_audit.valid);
  out.add("state", epoch_state_str(g_state));
  out.add("epoch_sequence", g_audit.epoch_sequence);
  out.add("request_id", g_audit.request_id);
  out.add("reason", epoch_reason_str(g_audit.reason));

  out.add("control_request_dwt", g_audit.request_dwt);
  out.add("control_interrupt_arm_dwt", g_audit.interrupt_arm_dwt);
  out.add("control_fact_authored_dwt", g_audit.fact_authored_dwt);
  out.add("control_install_begin_dwt", g_audit.install_begin_dwt);
  out.add("control_install_end_dwt", g_audit.install_end_dwt);
  out.add("control_finalize_dwt", g_audit.finalize_dwt);
  out.add("control_pps_isr_entry_dwt_raw", g_audit.pps_isr_entry_dwt_raw);
  out.add("control_pps_event_dwt", g_audit.pps_event_dwt);
  out.add("control_selected_edge_dwt", g_audit.selected_edge_dwt);
  out.add("control_selected_vclock_counter32", g_audit.selected_vclock_counter32);
  out.add("control_selected_vclock_low16", (uint32_t)g_audit.selected_vclock_low16);
  out.add("dwt64_epoch_valid", g_dwt64_epoch_valid);
  out.add("dwt64_epoch_reset_ok", g_dwt64_epoch_reset_ok);
  out.add("dwt64_raw_epoch_cycles", g_dwt64_raw_epoch_cycles);
  out.add("dwt64_epoch_reset_count", g_dwt64_epoch_reset_count);
  out.add("dwt64_epoch_reset_failures", g_dwt64_epoch_reset_failures);

  add_install_fields(out, "dwt", g_audit.dwt);
  add_install_fields(out, "vclock", g_audit.vclock);
  add_install_fields(out, "ocxo1", g_audit.ocxo1);
  add_install_fields(out, "ocxo2", g_audit.ocxo2);
  add_summary_fields(out);
  return out;
}

static Payload cmd_history(const Payload&) {
  Payload out;
  out.add("model", "EPOCH_HISTORY");
  out.add("report_style", "flat_v5_dwt64_cycles");
  out.add("count", g_history_count);
  out.add("capacity", (uint32_t)EPOCH_HISTORY_CAPACITY);

  for (uint32_t n = 0; n < g_history_count; n++) {
    const uint32_t idx =
        (g_history_head + EPOCH_HISTORY_CAPACITY - g_history_count + n) %
        EPOCH_HISTORY_CAPACITY;
    char prefix[24];
    snprintf(prefix, sizeof(prefix), "entry%lu", (unsigned long)n);
    add_history_entry_fields(out, prefix, g_history[idx]);
  }

  return out;
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t EPOCH_COMMANDS[] = {
  { "ZERO",    cmd_zero    },
  { "STATUS",  cmd_status  },
  { "CLOCKS",  cmd_clocks  },
  { "REPORT",  cmd_report  },
  { "AUDIT",   cmd_audit   },
  { "HISTORY", cmd_history },
  { nullptr,   nullptr     }
};

static const process_vtable_t EPOCH_PROCESS = {
  .process_id = "EPOCH",
  .commands = EPOCH_COMMANDS,
  .subscriptions = nullptr,
};

void process_epoch_register(void) {
  process_register("EPOCH", &EPOCH_PROCESS);
}