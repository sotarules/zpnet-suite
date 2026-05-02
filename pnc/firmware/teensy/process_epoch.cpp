// ============================================================================
// process_epoch.cpp — simple compensated epoch zero
// ============================================================================

#include "process_epoch.h"
#include "process_clocks.h"
#include "process_interrupt.h"
#include "process_timepop.h"

#include "config.h"
#include "process.h"
#include "payload.h"
#include "time.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

void clocks_alpha_epoch_installed(const epoch_fact_t& fact) __attribute__((weak));
void clocks_alpha_epoch_installed(const epoch_fact_t&) {}
void clocks_beta_epoch_installed(const epoch_fact_t& fact) __attribute__((weak));
void clocks_beta_epoch_installed(const epoch_fact_t&) {}

static constexpr uint32_t INSTALL_COUNT = 9;
// Epoch counter estimation tuning constants (hoisted for iterative refinement).
static constexpr uint32_t ESTIMATOR_PROCESS_DELAY_CYCLES = 10;
static constexpr uint32_t WRITE_COST_CYCLES_HW16 = 9;
static constexpr uint32_t WRITE_COST_CYCLES_COUNT32 = 1;
static constexpr uint32_t WRITE_COST_CYCLES_NS64 = 1;

struct install_audit_t {
  const char* clock = "";
  const char* target = "";
  uint32_t dwt_at_write = 0;
  uint64_t elapsed_cycles = 0;
  uint64_t elapsed_ns = 0;
  uint64_t ticks = 0;
  uint64_t ns_value = 0;
  bool ok = false;
};

static bool g_pending = false;
static bool g_finalized = false;
static epoch_state_t g_state = epoch_state_t::IDLE;
static uint32_t g_next_request_id = 1;
static uint32_t g_last_request_id = 0;
static uint32_t g_epoch_sequence = 0;
static epoch_reason_t g_pending_reason = epoch_reason_t::UNKNOWN;
static epoch_fact_t g_fact = {};
static install_audit_t g_audit[INSTALL_COUNT] = {};
static uint32_t g_audit_count = 0;
static uint32_t g_install_begin_dwt = 0;
static uint32_t g_install_end_dwt = 0;
static bool g_dwt64_reset_ok = false;
static uint64_t g_dwt64_raw_epoch = 0;

static inline uint32_t dwt_now(void) { return ARM_DWT_CYCCNT; }

const char* epoch_reason_str(epoch_reason_t reason) {
  switch (reason) {
    case epoch_reason_t::STARTUP: return "STARTUP";
    case epoch_reason_t::EPOCH_ZERO_COMMAND: return "EPOCH_ZERO_COMMAND";
    case epoch_reason_t::CLOCKS_ZERO_COMMAND: return "CLOCKS_ZERO_COMMAND";
    case epoch_reason_t::CLOCKS_START: return "CLOCKS_START";
    case epoch_reason_t::CLOCKS_RECOVER: return "CLOCKS_RECOVER";
    case epoch_reason_t::INTERNAL: return "INTERNAL";
    default: return "UNKNOWN";
  }
}

const char* epoch_state_str(epoch_state_t state) {
  switch (state) {
    case epoch_state_t::IDLE: return "IDLE";
    case epoch_state_t::REQUESTED: return "REQUESTED";
    case epoch_state_t::INTERRUPT_ARMED: return "INTERRUPT_ARMED";
    case epoch_state_t::INSTALLING: return "INSTALLING";
    case epoch_state_t::FINALIZED: return "FINALIZED";
    case epoch_state_t::FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

struct tick_projection_t {
  uint64_t cycles = 0;
  uint64_t ticks = 0;
  uint64_t ns = 0;
};

static tick_projection_t dwt_cycles_to_10mhz(uint64_t cycles,
                                             uint32_t extra_cycles) {
  tick_projection_t r{};
  r.cycles = cycles;

  const uint64_t adjusted_cycles = cycles + (uint64_t)extra_cycles;
  r.ticks = (adjusted_cycles * (uint64_t)VCLOCK_COUNTS_PER_SECOND +
             (uint64_t)DWT_EXPECTED_PER_PPS / 2ULL) /
            (uint64_t)DWT_EXPECTED_PER_PPS;
  r.ns = r.ticks * (uint64_t)NS_PER_10MHZ_TICK;
  return r;
}

static tick_projection_t elapsed_from_epoch_to_ticks(uint32_t epoch_dwt,
                                                     uint32_t now_dwt,
                                                     uint32_t extra_cycles) {
  const uint64_t cycles = (uint32_t)(now_dwt - epoch_dwt);
  return dwt_cycles_to_10mhz(cycles, extra_cycles);
}

static inline uint64_t cycles_to_ns_floor(uint64_t cycles) {
  // 1 cycle = 125/126 ns at 1.008 GHz.
  return (cycles * 125ULL) / 126ULL;
}

static uint64_t estimate_ticks_from_epoch_with_write_cost(uint32_t epoch_dwt,
                                                          uint32_t write_cost_cycles,
                                                          uint32_t* out_dwt,
                                                          uint64_t* out_cycles,
                                                          uint64_t* out_ns) {
  const uint32_t now = dwt_now();
  const uint64_t elapsed_cycles = (uint32_t)(now - epoch_dwt);
  const uint64_t adjusted_cycles =
      elapsed_cycles + (uint64_t)ESTIMATOR_PROCESS_DELAY_CYCLES;
  const uint64_t adjusted_ns = cycles_to_ns_floor(adjusted_cycles);

  uint64_t ticks = adjusted_ns / (uint64_t)NS_PER_10MHZ_TICK;
  const uint64_t rem_ns = adjusted_ns % (uint64_t)NS_PER_10MHZ_TICK;
  const uint64_t write_cost_ns = cycles_to_ns_floor((uint64_t)write_cost_cycles);
  if (rem_ns + write_cost_ns >= (uint64_t)NS_PER_10MHZ_TICK) {
    ticks++;
  }

  if (out_dwt) *out_dwt = now;
  if (out_cycles) *out_cycles = elapsed_cycles;
  if (out_ns) *out_ns = ticks * (uint64_t)NS_PER_10MHZ_TICK;
  return ticks;
}

static void record(const char* clock, const char* target, uint32_t dwt,
                   uint64_t cycles, uint64_t ticks, bool ok) {
  if (g_audit_count >= INSTALL_COUNT) return;
  install_audit_t& a = g_audit[g_audit_count++];
  a.clock = clock;
  a.target = target;
  a.dwt_at_write = dwt;
  a.elapsed_cycles = cycles;
  a.ticks = ticks;
  a.elapsed_ns = ticks * (uint64_t)NS_PER_10MHZ_TICK;
  a.ns_value = a.elapsed_ns;
  a.ok = ok;
}

static bool set_target(interrupt_subscriber_kind_t kind, const char* clock,
                       const char* target, uint32_t epoch_dwt) {
  uint32_t dwt = 0;
  uint64_t cycles = 0;
  uint64_t ns = 0;
  uint32_t write_cost_cycles = WRITE_COST_CYCLES_COUNT32;
  if (target[0] == 'H') {
    write_cost_cycles = WRITE_COST_CYCLES_HW16;
  } else if (target[0] == 'C') {
    write_cost_cycles = WRITE_COST_CYCLES_COUNT32;
  } else {
    write_cost_cycles = WRITE_COST_CYCLES_NS64;
  }
  const uint64_t ticks = estimate_ticks_from_epoch_with_write_cost(
      epoch_dwt, write_cost_cycles, &dwt, &cycles, &ns);
  bool ok = false;

  if (target[0] == 'H') {
    ok = interrupt_clock16_set_from_ticks(kind, (uint32_t)ticks);
  } else if (target[0] == 'C') {
    ok = interrupt_clock32_set_from_ticks(kind, (uint32_t)ticks);
  } else {
    ok = interrupt_clock64_ns_set(kind, ns);
  }

  record(clock, target, dwt, cycles, ticks, ok);
  return ok;
}

static bool set_clock(interrupt_subscriber_kind_t kind, const char* clock,
                      uint32_t epoch_dwt) {
  bool ok = true;
  ok = set_target(kind, clock, "HW16", epoch_dwt) && ok;
  ok = set_target(kind, clock, "COUNT32", epoch_dwt) && ok;
  ok = set_target(kind, clock, "NS64", epoch_dwt) && ok;
  return ok;
}

static bool install_from_snapshot(const pps_edge_snapshot_t& snap) {
  g_state = epoch_state_t::INSTALLING;
  g_finalized = false;
  g_audit_count = 0;
  g_install_begin_dwt = dwt_now();

  g_fact = epoch_fact_t{};
  g_fact.valid = true;
  g_fact.request_id = g_last_request_id;
  g_fact.epoch_sequence = ++g_epoch_sequence;
  g_fact.reason = g_pending_reason;
  g_fact.epoch_ns = 0;
  g_fact.epoch_dwt = snap.dwt_at_edge;
  g_fact.dwt_cycles_per_second_seed = DWT_EXPECTED_PER_PPS;
  g_fact.vclock_counter32_at_epoch = snap.counter32_at_edge;
  g_fact.vclock_low16_at_epoch = snap.ch3_at_edge;
  g_fact.pps_sequence = snap.sequence;
  g_fact.physical_pps_dwt_at_edge = snap.physical_pps_dwt_normalized_at_edge;
  g_fact.physical_pps_counter32_at_read = snap.physical_pps_counter32_at_read;
  g_fact.physical_pps_low16_at_read = snap.physical_pps_ch3_at_read;

  g_dwt64_reset_ok = clocks_dwt64_epoch_reset_at_dwt32(g_fact.epoch_dwt,
                                                        &g_dwt64_raw_epoch);
  bool ok = g_dwt64_reset_ok;

  ok = set_clock(interrupt_subscriber_kind_t::VCLOCK, "vclock", g_fact.epoch_dwt) && ok;
  ok = set_clock(interrupt_subscriber_kind_t::OCXO1,  "ocxo1",  g_fact.epoch_dwt) && ok;
  ok = set_clock(interrupt_subscriber_kind_t::OCXO2,  "ocxo2",  g_fact.epoch_dwt) && ok;

  if (ok) {
    clocks_alpha_epoch_installed(g_fact);
    time_pps_vclock_epoch_reset(g_fact.epoch_dwt, g_fact.vclock_counter32_at_epoch);

    // VCLOCK was rebased above.  Any timed TimePop deadline expressed in the
    // old synthetic VCLOCK coordinate system is now invalid.  Re-author
    // recurring timers and cancel unsafe one-shots before normal foreground
    // scheduling continues.
    timepop_epoch_changed(g_fact.epoch_sequence);

    clocks_beta_epoch_installed(g_fact);
    g_state = epoch_state_t::FINALIZED;
    g_finalized = true;
  } else {
    g_state = epoch_state_t::FAULT;
  }

  g_pending = false;
  g_install_end_dwt = dwt_now();
  return ok;
}

void process_epoch_init(void) {
  g_pending = false;
  g_finalized = false;
  g_state = epoch_state_t::IDLE;
  g_next_request_id = 1;
  g_last_request_id = 0;
  g_epoch_sequence = 0;
  g_pending_reason = epoch_reason_t::UNKNOWN;
  g_fact = epoch_fact_t{};
  g_audit_count = 0;
  g_install_begin_dwt = 0;
  g_install_end_dwt = 0;
  g_dwt64_reset_ok = false;
  g_dwt64_raw_epoch = 0;
}

epoch_request_result_t process_epoch_request_zero(epoch_reason_t reason) {
  epoch_request_result_t r{};
  r.request_id = g_last_request_id;
  r.state = g_state;

  if (g_pending || g_state == epoch_state_t::INSTALLING) {
    r.accepted = false;
    r.message = "epoch_zero_already_pending";
    return r;
  }

  g_last_request_id = g_next_request_id++;
  if (g_next_request_id == 0) g_next_request_id = 1;
  g_pending_reason = reason;
  g_pending = true;
  g_finalized = false;
  g_state = epoch_state_t::INTERRUPT_ARMED;

  interrupt_request_pps_rebootstrap();

  r.accepted = true;
  r.request_id = g_last_request_id;
  r.state = g_state;
  r.message = "epoch_zero_armed";
  return r;
}

bool process_epoch_on_pps_vclock_snapshot(const pps_edge_snapshot_t& snap) {
  if (!g_pending) return false;
  if (interrupt_pps_rebootstrap_pending()) return false;
  return install_from_snapshot(snap);
}

bool process_epoch_pending(void) { return g_pending || g_state == epoch_state_t::INSTALLING; }
bool process_epoch_finalized(void) { return g_finalized; }
epoch_state_t process_epoch_state(void) { return g_state; }
uint32_t process_epoch_last_request_id(void) { return g_last_request_id; }
uint32_t process_epoch_current_sequence(void) { return g_epoch_sequence; }
epoch_fact_t process_epoch_current_fact(void) { return g_fact; }

static Payload cmd_zero(const Payload&) {
  const epoch_request_result_t r = process_epoch_request_zero(epoch_reason_t::EPOCH_ZERO_COMMAND);
  Payload p;
  p.add("model", "EPOCH_ZERO");
  p.add("accepted", r.accepted);
  p.add("request_id", r.request_id);
  p.add("state", epoch_state_str(r.state));
  p.add("message", r.message);
  return p;
}

static Payload cmd_status(const Payload&) {
  Payload p;
  p.add("model", "EPOCH_STATUS");
  p.add("pending", g_pending);
  p.add("finalized", g_finalized);
  p.add("state", epoch_state_str(g_state));
  p.add("request_id", g_last_request_id);
  p.add("epoch_sequence", g_epoch_sequence);
  p.add("reason", epoch_reason_str(g_pending_reason));
  p.add("epoch_dwt", g_fact.epoch_dwt);
  p.add("pps_sequence", g_fact.pps_sequence);
  return p;
}

static Payload cmd_audit(const Payload&) {
  Payload p;
  p.add("model", "EPOCH_AUDIT");
  p.add("state", epoch_state_str(g_state));
  p.add("request_id", g_last_request_id);
  p.add("epoch_sequence", g_epoch_sequence);
  p.add("epoch_dwt", g_fact.epoch_dwt);
  p.add("dwt64_reset_ok", g_dwt64_reset_ok);
  p.add("dwt64_raw_epoch_cycles", g_dwt64_raw_epoch);
  p.add("install_begin_dwt", g_install_begin_dwt);
  p.add("install_end_dwt", g_install_end_dwt);
  p.add("install_elapsed_cycles", (uint32_t)(g_install_end_dwt - g_install_begin_dwt));
  p.add("install_count", g_audit_count);
  for (uint32_t i = 0; i < g_audit_count; i++) {
    char k[64];
    snprintf(k, sizeof(k), "i%lu_clock", (unsigned long)i); p.add(k, g_audit[i].clock);
    snprintf(k, sizeof(k), "i%lu_target", (unsigned long)i); p.add(k, g_audit[i].target);
    snprintf(k, sizeof(k), "i%lu_dwt", (unsigned long)i); p.add(k, g_audit[i].dwt_at_write);
    snprintf(k, sizeof(k), "i%lu_elapsed_cycles", (unsigned long)i); p.add(k, g_audit[i].elapsed_cycles);
    snprintf(k, sizeof(k), "i%lu_elapsed_ns", (unsigned long)i); p.add(k, g_audit[i].elapsed_ns);
    snprintf(k, sizeof(k), "i%lu_ticks", (unsigned long)i); p.add(k, g_audit[i].ticks);
    snprintf(k, sizeof(k), "i%lu_ns_value", (unsigned long)i); p.add(k, g_audit[i].ns_value);
    snprintf(k, sizeof(k), "i%lu_ok", (unsigned long)i); p.add(k, g_audit[i].ok);
  }
  return p;
}

struct clock_report_t {
  const char* name = "";
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  bool ok = false;

  uint32_t dwt_before = 0;
  uint32_t dwt_after = 0;
  uint32_t dwt_mid = 0;
  uint32_t read_cycles = 0;

  interrupt_clock_snapshot_t raw{};

  uint32_t norm_cycles = 0;
  uint64_t norm_ticks_delta = 0;
  uint16_t norm_hw16 = 0;
  uint32_t norm_counter32 = 0;
  uint64_t norm_ticks64 = 0;
  uint64_t norm_ns64 = 0;
};

static clock_report_t read_clock_for_report(const char* name,
                                            interrupt_subscriber_kind_t kind) {
  clock_report_t r{};
  r.name = name;
  r.kind = kind;

  r.dwt_before = dwt_now();
  r.ok = interrupt_clock_snapshot(kind, &r.raw);
  r.dwt_after = dwt_now();

  r.read_cycles = r.dwt_after - r.dwt_before;
  r.dwt_mid = r.dwt_before + (r.read_cycles / 2U);

  r.norm_hw16 = r.raw.hardware16;
  r.norm_counter32 = r.raw.counter32;
  r.norm_ticks64 = r.raw.ns64 / (uint64_t)NS_PER_10MHZ_TICK;
  r.norm_ns64 = r.raw.ns64;
  return r;
}

static void normalize_clock_to_report_dwt(clock_report_t& r,
                                          uint32_t report_dwt) {
  if (!r.ok) return;

  r.norm_cycles = report_dwt - r.dwt_mid;
  const tick_projection_t delta = dwt_cycles_to_10mhz(r.norm_cycles, 0);
  r.norm_ticks_delta = delta.ticks;

  r.norm_hw16 = (uint16_t)(r.raw.hardware16 + (uint16_t)delta.ticks);
  r.norm_counter32 = r.raw.counter32 + (uint32_t)delta.ticks;
  r.norm_ticks64 = (r.raw.ns64 / (uint64_t)NS_PER_10MHZ_TICK) + delta.ticks;
  r.norm_ns64 = r.norm_ticks64 * (uint64_t)NS_PER_10MHZ_TICK;
}

static void add_clock_report(Payload& p,
                             const clock_report_t& r,
                             const clock_report_t& reference) {
  char k[80];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(k, sizeof(k), "%s_%s", r.name, suffix);
    p.add(k, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(k, sizeof(k), "%s_%s", r.name, suffix);
    p.add(k, value);
  };
  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(k, sizeof(k), "%s_%s", r.name, suffix);
    p.add(k, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(k, sizeof(k), "%s_%s", r.name, suffix);
    p.add(k, value);
  };

  const uint64_t raw_ticks64 = r.raw.ns64 / (uint64_t)NS_PER_10MHZ_TICK;

  add_bool("available", r.ok);

  // Back-compatible names now mean normalized-to-report-DWT values.
  add_u32("hw16", (uint32_t)r.norm_hw16);
  add_u32("counter32", r.norm_counter32);
  add_u64("ns64", r.norm_ns64);
  add_u64("ticks64", r.norm_ticks64);

  // Raw ambient observation, including its DWT custody window.
  add_u32("raw_hw16", (uint32_t)r.raw.hardware16);
  add_u32("raw_counter32", r.raw.counter32);
  add_u64("raw_ns64", r.raw.ns64);
  add_u64("raw_ticks64", raw_ticks64);
  add_u32("read_dwt_before", r.dwt_before);
  add_u32("read_dwt_after", r.dwt_after);
  add_u32("read_dwt_mid", r.dwt_mid);
  add_u32("read_cycles", r.read_cycles);

  // Normalization facts: how far this ambient read was projected forward.
  add_u32("norm_cycles", r.norm_cycles);
  add_u64("norm_ticks_delta", r.norm_ticks_delta);
  add_u32("norm_hw16", (uint32_t)r.norm_hw16);
  add_u32("norm_counter32", r.norm_counter32);
  add_u64("norm_ns64", r.norm_ns64);
  add_u64("norm_ticks64", r.norm_ticks64);

  if (r.ok && reference.ok) {
    add_i32("norm_counter32_delta_vs_vclock",
            (int32_t)(r.norm_counter32 - reference.norm_counter32));
    add_i32("norm_ticks64_delta_vs_vclock",
            (int32_t)(r.norm_ticks64 - reference.norm_ticks64));
    add_i32("norm_hw16_delta_vs_vclock",
            (int32_t)((int16_t)(r.norm_hw16 - reference.norm_hw16)));
  } else {
    add_i32("norm_counter32_delta_vs_vclock", 0);
    add_i32("norm_ticks64_delta_vs_vclock", 0);
    add_i32("norm_hw16_delta_vs_vclock", 0);
  }
}

static Payload cmd_clocks(const Payload&) {
  clock_report_t vclock = read_clock_for_report("vclock", interrupt_subscriber_kind_t::VCLOCK);
  clock_report_t ocxo1  = read_clock_for_report("ocxo1",  interrupt_subscriber_kind_t::OCXO1);
  clock_report_t ocxo2  = read_clock_for_report("ocxo2",  interrupt_subscriber_kind_t::OCXO2);

  const uint32_t report_dwt = dwt_now();
  normalize_clock_to_report_dwt(vclock, report_dwt);
  normalize_clock_to_report_dwt(ocxo1, report_dwt);
  normalize_clock_to_report_dwt(ocxo2, report_dwt);

  Payload p;
  p.add("model", "EPOCH_CLOCKS");

  Payload summary;
  summary.add("dwt64_cycles_total", clocks_dwt_cycles_now());
  summary.add("vclock_ns64_total", vclock.norm_ns64);
  summary.add("ocxo1_ns64_total", ocxo1.norm_ns64);
  summary.add("ocxo2_ns64_total", ocxo2.norm_ns64);
  p.add_object("summary", summary);

  Payload detail;
  detail.add("report_style", "normalized_v1");
  detail.add("state", epoch_state_str(g_state));
  detail.add("epoch_sequence", g_epoch_sequence);
  detail.add("epoch_dwt", g_fact.epoch_dwt);
  detail.add("dwt64_cycles", clocks_dwt_cycles_now());
  detail.add("report_dwt", report_dwt);
  detail.add("normalization", "project_each_ambient_read_to_report_dwt");
  detail.add("dwt_to_10mhz_cycles_per_second", (uint32_t)DWT_EXPECTED_PER_PPS);
  detail.add("ns_per_10mhz_tick", (uint32_t)NS_PER_10MHZ_TICK);

  add_clock_report(detail, vclock, vclock);
  add_clock_report(detail, ocxo1, vclock);
  add_clock_report(detail, ocxo2, vclock);
  p.add_object("detail", detail);
  return p;
}

static const process_command_entry_t EPOCH_COMMANDS[] = {
  { "ZERO",   cmd_zero   },
  { "STATUS", cmd_status },
  { "AUDIT",  cmd_audit  },
  { "CLOCKS", cmd_clocks },
  { nullptr,  nullptr    }
};

static const process_vtable_t EPOCH_PROCESS = {
  .process_id = "EPOCH",
  .commands = EPOCH_COMMANDS,
  .subscriptions = nullptr,
};

void process_epoch_register(void) {
  process_register("EPOCH", &EPOCH_PROCESS);
}
