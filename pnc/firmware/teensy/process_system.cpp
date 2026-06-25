// =============================================================
// FILE: process_system.cpp
// =============================================================
//
// SYSTEM Process — Teensy-side system truth surface
//
// SYSTEM is not a lifecycle-managed process.
// It cannot start or stop from within itself.
//
// It exposes authoritative, read-only system facts and
// provides explicit terminal transitions (shutdown, bootloader)
// that represent irreversible boundary crossings.
//
// =============================================================

#include "process_system.h"
#include "memory_info.h"
#include "config.h"
#include "process.h"
#include "events.h"
#include "payload.h"
#include "publish.h"
#include "cpu_usage.h"
#include "util.h"
#include "timepop.h"
#include "process_timepop.h"
#include "debug.h"
#include "transport.h"   // <-- NEW (for transport_get_info)

#include <string.h>
#include <strings.h>

static constexpr uint64_t FLASH_DELAY_NS = 5000000000ULL;  // 5 seconds

// --------------------------------------------------------------
// Forward declarations (terminal paths)
// --------------------------------------------------------------
void system_enter_quiescence(void);

// Bootloader entry symbol (ROM-provided, never returns)
extern "C" void enter_bootloader_cleanly(void);

static Payload system_features_tree_payload(void);
static void system_feature_schedule_fragment_publish(void);

// --------------------------------------------------------------
// Boot diagnostics
// --------------------------------------------------------------
//
// Reset cause is captured once and reported as raw silicon truth.  Decoding can
// happen Pi-side or in later tooling.  Keeping this raw avoids policy, string
// payload growth, and any new subsystem footprint.

static uint32_t g_system_boot_reset_cause_raw = 0;
static bool     g_system_boot_reset_cause_captured = false;

void system_bootdiag_capture_reset_cause(void) {
  if (g_system_boot_reset_cause_captured) return;

#if defined(SRC_SRSR)
  g_system_boot_reset_cause_raw = SRC_SRSR;
#else
  g_system_boot_reset_cause_raw = 0;
#endif

  g_system_boot_reset_cause_captured = true;
}

uint32_t system_bootdiag_reset_cause_raw(void) {
  return g_system_boot_reset_cause_raw;
}

// --------------------------------------------------------------
// Internal terminal state
// --------------------------------------------------------------
static bool system_shutdown   = false;
static bool system_bootloader = false;

static bool system_cpu_window_initialized = false;
static uint64_t system_cpu_window_last_wall_cycles = 0;
static uint64_t system_cpu_window_last_idle_cycles = 0;

// --------------------------------------------------------------
// FEATURE_STATUS_FRAGMENT publication custody
// --------------------------------------------------------------
// system_feature_set() may be reached by timing subsystems, so it only marks the
// scalar feature tree dirty and requests an ALAP foreground service.  The service
// publishes one compact FEATURE_STATUS_FRAGMENT containing Teensy-owned feature
// state.  zpnet-system on the Pi relays the unified FEATURE_STATUS tree.

static volatile bool g_system_feature_fragment_publish_enabled = false;
static volatile bool g_system_feature_fragment_dirty = false;
static volatile bool g_system_feature_fragment_service_armed = false;
static uint32_t g_system_feature_fragment_publish_count = 0;
static uint32_t g_system_feature_fragment_service_arm_count = 0;
static uint32_t g_system_feature_fragment_service_dispatch_count = 0;
static uint32_t g_system_feature_fragment_service_arm_failures = 0;

// ================================================================
// Feature status substrate
// ================================================================
//
// Shape reported by SYSTEM.FEATURES and embedded in SYSTEM.REPORT:
//
//   {
//     "TEENSY": {
//       "CLOCKS": { "SMARTZERO": "NOMINAL" }
//     }
//   }
//
// SYSTEM is the local clearing house only. It stores feature state as the
// compact scalar status for each MACHINE.SUBSYSTEM.FEATURE path. Focused
// subsystem reports remain the place for detailed diagnostics.

static constexpr size_t SYSTEM_FEATURE_MAX_FEATURES   = 64;
static constexpr size_t SYSTEM_FEATURE_SUBSYSTEM_MAX  = 24;
static constexpr size_t SYSTEM_FEATURE_NAME_MAX       = 48;

struct system_feature_slot_t {
  bool used = false;
  char subsystem[SYSTEM_FEATURE_SUBSYSTEM_MAX] = {0};
  char feature[SYSTEM_FEATURE_NAME_MAX] = {0};
  system_feature_status_t status = system_feature_status_t::INITIALIZING;
};

static system_feature_slot_t g_system_features[SYSTEM_FEATURE_MAX_FEATURES] = {};

const char* system_feature_status_str(system_feature_status_t status) {
  switch (status) {
    case system_feature_status_t::NOMINAL:      return "NOMINAL";
    case system_feature_status_t::HOLD:         return "HOLD";
    case system_feature_status_t::ANOMALY:      return "ANOMALY";
    case system_feature_status_t::INITIALIZING:
    default:                                    return "INITIALIZING";
  }
}

bool system_feature_status_parse(const char* status,
                                 system_feature_status_t* out) {
  if (!status || !*status || !out) return false;

  if (strcasecmp(status, "NOMINAL") == 0) {
    *out = system_feature_status_t::NOMINAL;
    return true;
  }
  if (strcasecmp(status, "INITIALIZING") == 0) {
    *out = system_feature_status_t::INITIALIZING;
    return true;
  }
  if (strcasecmp(status, "HOLD") == 0) {
    *out = system_feature_status_t::HOLD;
    return true;
  }
  if (strcasecmp(status, "ANOMALY") == 0 || strcasecmp(status, "DOWN") == 0) {
    *out = system_feature_status_t::ANOMALY;
    return true;
  }

  return false;
}

static void system_feature_fragment_publish_service(timepop_ctx_t*,
                                                    timepop_diag_t*,
                                                    void*) {
  g_system_feature_fragment_service_armed = false;
  g_system_feature_fragment_service_dispatch_count++;

  if (!g_system_feature_fragment_publish_enabled ||
      !g_system_feature_fragment_dirty) {
    return;
  }

  g_system_feature_fragment_dirty = false;

  Payload fragment = system_features_tree_payload();
  publish("FEATURE_STATUS_FRAGMENT", fragment);
  g_system_feature_fragment_publish_count++;

  if (g_system_feature_fragment_dirty) {
    system_feature_schedule_fragment_publish();
  }
}

static void system_feature_schedule_fragment_publish(void) {
  if (!g_system_feature_fragment_publish_enabled ||
      !g_system_feature_fragment_dirty ||
      g_system_feature_fragment_service_armed) {
    return;
  }

  const timepop_handle_t handle =
      timepop_arm_alap(system_feature_fragment_publish_service,
                       nullptr,
                       "SYSTEM_FEATURE_STATUS_FRAGMENT");

  if (handle == TIMEPOP_INVALID_HANDLE) {
    g_system_feature_fragment_service_arm_failures++;
    return;
  }

  g_system_feature_fragment_service_armed = true;
  g_system_feature_fragment_service_arm_count++;
}

static void system_feature_note_changed(void) {
  g_system_feature_fragment_dirty = true;
  system_feature_schedule_fragment_publish();
}

static int system_feature_find(const char* subsystem,
                               const char* feature) {
  if (!subsystem || !*subsystem || !feature || !*feature) return -1;

  for (size_t i = 0; i < SYSTEM_FEATURE_MAX_FEATURES; i++) {
    if (!g_system_features[i].used) continue;
    if (strcmp(g_system_features[i].subsystem, subsystem) != 0) continue;
    if (strcmp(g_system_features[i].feature, feature) != 0) continue;
    return (int)i;
  }
  return -1;
}

static int system_feature_alloc_or_find(const char* subsystem,
                                        const char* feature) {
  const int existing = system_feature_find(subsystem, feature);
  if (existing >= 0) return existing;

  for (size_t i = 0; i < SYSTEM_FEATURE_MAX_FEATURES; i++) {
    if (g_system_features[i].used) continue;
    g_system_features[i] = system_feature_slot_t{};
    g_system_features[i].used = true;
    safeCopy(g_system_features[i].subsystem,
             sizeof(g_system_features[i].subsystem),
             subsystem);
    safeCopy(g_system_features[i].feature,
             sizeof(g_system_features[i].feature),
             feature);
    return (int)i;
  }

  return -1;
}

bool system_feature_set(const char* subsystem,
                        const char* feature,
                        system_feature_status_t status,
                        const char* detail) {
  if (!subsystem || !*subsystem || !feature || !*feature) return false;

  const int existing = system_feature_find(subsystem, feature);
  const int idx = (existing >= 0)
      ? existing
      : system_feature_alloc_or_find(subsystem, feature);
  if (idx < 0) return false;

  system_feature_slot_t& slot = g_system_features[idx];

  if (existing >= 0 && slot.status == status) {
    (void)detail;
    return true;
  }

  slot.status = status;
  (void)detail;
  system_feature_note_changed();
  return true;
}

bool system_feature_set_str(const char* subsystem,
                            const char* feature,
                            const char* status,
                            const char* detail) {
  system_feature_status_t parsed{};
  if (!system_feature_status_parse(status, &parsed)) return false;
  return system_feature_set(subsystem, feature, parsed, detail);
}

bool system_feature_has(const char* subsystem,
                        const char* feature) {
  return system_feature_find(subsystem, feature) >= 0;
}

const char* system_feature_get_status(const char* subsystem,
                                      const char* feature) {
  const int idx = system_feature_find(subsystem, feature);
  if (idx < 0) return system_feature_status_str(system_feature_status_t::INITIALIZING);
  return system_feature_status_str(g_system_features[idx].status);
}

bool system_feature_is_nominal(const char* subsystem,
                               const char* feature) {
  return strcmp(system_feature_get_status(subsystem, feature), "NOMINAL") == 0;
}

static Payload system_features_tree_payload(void) {
  Payload teensy;

  for (size_t i = 0; i < SYSTEM_FEATURE_MAX_FEATURES; i++) {
    if (!g_system_features[i].used) continue;

    const char* subsystem = g_system_features[i].subsystem;

    bool already_emitted = false;
    for (size_t j = 0; j < i; j++) {
      if (!g_system_features[j].used) continue;
      if (strcmp(g_system_features[j].subsystem, subsystem) == 0) {
        already_emitted = true;
        break;
      }
    }
    if (already_emitted) continue;

    Payload subsystem_payload;
    for (size_t k = 0; k < SYSTEM_FEATURE_MAX_FEATURES; k++) {
      if (!g_system_features[k].used) continue;
      if (strcmp(g_system_features[k].subsystem, subsystem) != 0) continue;

      subsystem_payload.add(
        g_system_features[k].feature,
        system_feature_status_str(g_system_features[k].status)
      );
    }

    teensy.add_object(subsystem, subsystem_payload);
  }

  Payload root;
  root.add_object("TEENSY", teensy);
  return root;
}

// ================================================================
// Terminal helpers
// ================================================================

static void enter_bootloader_cb(timepop_ctx_t*, timepop_diag_t*, void*) {

  // Idempotent, higher priority than shutdown
  if (system_bootloader) {
    system_enter_quiescence();
  }

  system_bootloader = true;

  // Irreversible transition
  enter_bootloader_cleanly();

  // Absolute fallback (should never return)
  system_enter_quiescence();
}

bool system_is_shutdown(void) {
  return system_shutdown;
}

void system_request_shutdown(void) {

  // Idempotent terminal action
  if (system_shutdown || system_bootloader) {
    system_enter_quiescence();
  }

  system_shutdown = true;

  {
    Payload ev;
    ev.add("status", "REQUESTED");
    enqueueEvent("SYSTEM_SHUTDOWN", ev);
  }

  system_enter_quiescence();
}

// --------------------------------------------------------------
// Terminal quiescence (no return)
// --------------------------------------------------------------
void system_enter_quiescence(void) {
  while (true) {
    delay(1000);
  }
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// REPORT — authoritative system snapshot
//
// Semantics:
//   • Stateless
//   • Read-only
//   • No aggregation
//   • No inference
// ------------------------------------------------------------
static Payload cmd_report(const Payload& /*args*/) {

  Payload p;

  // Firmware identity
  p.add("fw_version", FW_VERSION);

  // CPU clock frequency — authoritative runtime value.
  // F_CPU_ACTUAL is updated by set_arm_clock() at boot.
  // This is the ACTUAL core clock, not the compile-time default.
  p.add("cpu_freq_mhz", (uint32_t)(F_CPU_ACTUAL / 1000000UL));

  // CPU temperature (best-effort)
  p.add("cpu_temp_c", cpuTempC());

  // Internal reference voltage (best-effort)
  p.add("vref_v", readVrefVolts());

  // Boot diagnostics — raw reset-cause register captured once at boot.
  p.add("boot_reset_cause_raw", g_system_boot_reset_cause_raw);
  p.add("boot_reset_cause_captured", g_system_boot_reset_cause_captured);

  // Heap availability
  //
  // Do not call maxAllocBytes() from SYSTEM.REPORT.  SYSTEM.REPORT is part of
  // the recurring telemetry surface, and maxAllocBytes() actively probes the
  // heap by repeatedly malloc/free testing large blocks.  That makes the report
  // an observer that perturbs the heap during campaigns.  Keep the recurring
  // report non-invasive; use focused MEMORY_INFO / future explicit diagnostics
  // for deeper heap probes.
  p.add("free_heap_bytes", freeHeapBytes());
  p.add("max_alloc_probe_disabled", true);

  timepop_idle_witness_snapshot_t idle{};
  timepop_idle_witness_snapshot(&idle);

  const uint64_t wall_cycles_now = idle.snapshot_wall_cycles;
  const uint64_t idle_cycles_now = idle.snapshot_total_cycles;
  uint64_t cpu_window_total_cycles = 0;
  uint64_t cpu_window_idle_cycles = 0;
  uint64_t cpu_window_work_cycles = 0;
  uint32_t cpu_work_pct_milli = 0;
  uint32_t cpu_idle_spin_pct_milli = 0;

  if (system_cpu_window_initialized) {
    cpu_window_total_cycles = wall_cycles_now - system_cpu_window_last_wall_cycles;
    cpu_window_idle_cycles = idle_cycles_now - system_cpu_window_last_idle_cycles;
    if (cpu_window_idle_cycles > cpu_window_total_cycles) {
      cpu_window_idle_cycles = cpu_window_total_cycles;
    }
    cpu_window_work_cycles =
        (uint64_t)cpu_window_total_cycles - cpu_window_idle_cycles;

    if (cpu_window_total_cycles != 0) {
      cpu_work_pct_milli = (uint32_t)((cpu_window_work_cycles * 100000ULL +
                                       (uint64_t)cpu_window_total_cycles / 2ULL) /
                                      (uint64_t)cpu_window_total_cycles);
      cpu_idle_spin_pct_milli = (uint32_t)((cpu_window_idle_cycles * 100000ULL +
                                            (uint64_t)cpu_window_total_cycles / 2ULL) /
                                           (uint64_t)cpu_window_total_cycles);
    }
  } else {
    system_cpu_window_initialized = true;
  }

  system_cpu_window_last_wall_cycles = wall_cycles_now;
  system_cpu_window_last_idle_cycles = idle_cycles_now;

  // CPU usage now means non-spin foreground/ISR work.  True core occupancy is
  // intentionally near 100% because the idle DWT witness loop runs when idle.
  p.add("cpu_usage_pct", cpu_work_pct_milli / 1000.0f);
  p.add("cpu_work_pct", cpu_work_pct_milli / 1000.0f);
  p.add("cpu_idle_spin_pct", cpu_idle_spin_pct_milli / 1000.0f);

  // CPU usage diagnostics
  p.add("cpu_work_cycles", (uint32_t)cpu_window_work_cycles);
  p.add("cpu_idle_spin_cycles", (uint32_t)cpu_window_idle_cycles);
  p.add("cpu_window_total_cycles", (uint32_t)cpu_window_total_cycles);
  p.add("cpu_work_cycles64", cpu_window_work_cycles);
  p.add("cpu_idle_spin_cycles64", cpu_window_idle_cycles);
  p.add("cpu_window_total_cycles64", cpu_window_total_cycles);
  p.add("cpu_idle_witness_total_cycles", idle_cycles_now);
  p.add("cpu_wall_cycles", wall_cycles_now);
  p.add("cpu_idle_witness_running", idle.running);
  p.add("cpu_idle_witness_enter_count", idle.enter_count);
  p.add("cpu_idle_witness_exit_count", idle.exit_count);
  p.add("cpu_idle_witness_last_residency_cycles", idle.last_residency_cycles);

  p.add("feature_status_fragment_publish_count", g_system_feature_fragment_publish_count);
  p.add("feature_status_fragment_dirty", (bool)g_system_feature_fragment_dirty);
  p.add("feature_status_fragment_service_armed", (bool)g_system_feature_fragment_service_armed);
  p.add("feature_status_fragment_service_arm_count", g_system_feature_fragment_service_arm_count);
  p.add("feature_status_fragment_service_dispatch_count", g_system_feature_fragment_service_dispatch_count);
  p.add("feature_status_fragment_service_arm_failures", g_system_feature_fragment_service_arm_failures);

  publish_info_t pub{};
  publish_get_info(&pub);
  p.add("publish_calls", pub.publish_calls);
  p.add("publish_invalid_topic", pub.publish_invalid_topic);
  p.add("publish_unsafe_context_drop", pub.publish_unsafe_context_drop);
  p.add("publish_reentrant_drop", pub.publish_reentrant_drop);
  p.add("publish_depth_high_water", pub.publish_depth_high_water);
  p.add("publish_active", pub.publish_active);
  p.add("publish_local_dispatch", pub.publish_local_dispatch);
  p.add("publish_forward_attempt", pub.publish_forward_attempt);
  p.add("publish_envelope_build", pub.publish_envelope_build);
  p.add("publish_transport_send", pub.publish_transport_send);

  // Legacy accounted callback-busy diagnostics.
  p.add("cpu_accounted_busy_pct", cpu_usage_get_percent());
  p.add("cpu_busy_cycles", cpu_usage_get_busy_cycles());
  p.add("cpu_total_cycles", cpu_usage_get_total_cycles());
  p.add("cpu_sample_window_ms", cpu_usage_get_sample_window_ms());

  p.add_object("features", system_features_tree_payload());

  return p;
}

// ============================================================================
// cmd_process_info — ZPNet SYSTEM command handler
// ============================================================================
//
// Register in your SYSTEM process command table as:
//
//   { "PROCESS_INFO", cmd_process_info },
//
// Query from Pi:
//   .tc system process_info
//
// Semantics:
//   • Read-only
//   • Snapshot-only
//   • No inference
//   • No allocation beyond Payload
//   • No transport emission
//
// Invariant checks:
//   If the system is healthy, the following MUST hold:
//     received == routed + error_missing_fields
//                        + error_unknown_subsys
//                        + error_unknown_command
//     routed == handler_invoked
//     handler_invoked == handler_completed
//     handler_completed == response_sent
//
//   Any deviation is a smoking gun.
// ============================================================================

static Payload cmd_process_info(const Payload& /*args*/) {

    process_rpc_info_t info{};
    process_get_rpc_info(&info);

    Payload p;

    // ==========================================================
    // Pipeline (happy path)
    // ==========================================================

    p.add("rpc_received",          info.received);
    p.add("rpc_routed",            info.routed);
    p.add("rpc_handler_invoked",   info.handler_invoked);
    p.add("rpc_handler_completed", info.handler_completed);
    p.add("rpc_response_sent",     info.response_sent);

    // ==========================================================
    // Errors
    // ==========================================================

    p.add("rpc_err_missing_fields",  info.error_missing_fields);
    p.add("rpc_err_unknown_subsys",  info.error_unknown_subsys);
    p.add("rpc_err_unknown_command", info.error_unknown_command);
    p.add("rpc_err_response_sent",   info.error_response_sent);

    // ==========================================================
    // Pub/sub
    // ==========================================================

    p.add("ps_dispatched",         info.ps_dispatched);

    // ==========================================================
    // Derived invariant checks
    // ==========================================================

    uint32_t expected_received = info.routed
                               + info.error_missing_fields
                               + info.error_unknown_subsys
                               + info.error_unknown_command;

    p.add("invariant_received_ok",  info.received == expected_received);
    p.add("invariant_handler_ok",   info.handler_invoked == info.handler_completed);
    p.add("invariant_response_ok",  info.handler_completed == info.response_sent);

    return p;
}

// ------------------------------------------------------------
// TRANSPORT_INFO — transport RX/TX forensic snapshot
//
// Semantics:
//   • Read-only
//   • Snapshot-only
//   • No inference
//   • No allocation beyond Payload
//   • No transport emission
// ------------------------------------------------------------
static Payload cmd_transport_info(const Payload& /*args*/) {

  transport_info_t info;
  transport_get_info(&info);

  Payload p;

  // ==========================================================
  // TX — Budget / Queue Health
  // ==========================================================

  p.add("tx_budget_max",        info.tx_budget_max);
  p.add("tx_budget_used",       info.tx_budget_used);
  p.add("tx_budget_high_water", info.tx_budget_high_water);

  p.add("tx_job_count",         info.tx_job_count);
  p.add("tx_job_high_water",    info.tx_job_high_water);

  p.add("tx_jobs_enqueued",     info.tx_jobs_enqueued);
  p.add("tx_jobs_sent",         info.tx_jobs_sent);

  p.add("tx_bytes_enqueued",    info.tx_bytes_enqueued);
  p.add("tx_bytes_sent",        info.tx_bytes_sent);

  // ==========================================================
  // TX — Failure counters
  // ==========================================================

  p.add("tx_alloc_fail",        info.tx_alloc_fail);
  p.add("tx_budget_fail",       info.tx_budget_fail);
  p.add("tx_queue_full",        info.tx_queue_full);
  p.add("tx_rr_drop_count",     info.tx_rr_drop_count);

  // ==========================================================
  // RX — Raw ingress
  // ==========================================================

  p.add("rx_blocks_total",      info.rx_blocks_total);
  p.add("rx_bytes_total",       info.rx_bytes_total);

  // ==========================================================
  // RX — Framing outcomes
  // ==========================================================

  p.add("rx_frames_complete",   info.rx_frames_complete);
  p.add("rx_frames_dispatched", info.rx_frames_dispatched);

  // ==========================================================
  // RX — State resets
  // ==========================================================

  p.add("rx_reset_hard",        info.rx_reset_hard);

  // ==========================================================
  // RX — Framing failures
  // ==========================================================

  p.add("rx_bad_stx",           info.rx_bad_stx);
  p.add("rx_bad_etx",           info.rx_bad_etx);
  p.add("rx_len_overflow",      info.rx_len_overflow);

  // ==========================================================
  // RX — Concurrency / anomaly signals
  // ==========================================================

  p.add("rx_overlap",                 info.rx_overlap);
  p.add("rx_expected_traffic_missing", info.rx_expected_traffic_missing);

  return p;
}

// ------------------------------------------------------------
// PAYLOAD_INFO — payload allocator / entry invariants snapshot
//
// Semantics:
//   • Read-only
//   • Snapshot-only
//   • No inference
//   • No allocation beyond Payload
//   • No transport emission
// ------------------------------------------------------------
static Payload cmd_payload_info(const Payload& /*args*/) {

  payload_info_t info{};
  payload_get_info(&info);

  Payload p;

  // ==========================================================
  // Lifetime totals
  // ==========================================================

  p.add("payload_instances_constructed", info.instances_constructed);
  p.add("payload_instances_destroyed",   info.instances_destroyed);

  // ==========================================================
  // Live tracking (invariant support)
  // ==========================================================

  p.add("payload_alive_now",        info.alive_now);
  p.add("payload_alive_high_water", info.alive_high_water);

  // ==========================================================
  // Arena behavior
  // ==========================================================

  p.add("payload_arena_alloc_fail", info.arena_alloc_fail);
  p.add("payload_arena_high_water", info.arena_high_water);

  // ==========================================================
  // Entry behavior
  // ==========================================================

  p.add("payload_entry_overflow",   info.entry_overflow);
  p.add("payload_entry_high_water", info.entry_high_water);

  return p;
}

// ============================================================================
// cmd_memory_info — ZPNet SYSTEM command handler
// ============================================================================
//
// Register in your SYSTEM process command table as:
//
//   { "MEMORY_INFO", cmd_memory_info },
//
// Call memory_info_init() once in setup() BEFORE other initialization.
//
// Query from Pi:
//   .tc system memory_info
//
// Semantics:
//   • Read-only
//   • Snapshot-only
//   • No inference
//   • No allocation beyond Payload
//   • No transport emission
// ============================================================================

static Payload cmd_memory_info(const Payload& /*args*/) {

    memory_info_t info{};
    memory_info_get(&info);

    Payload p;

    // ==========================================================
    // DTCM / Stack
    // ==========================================================

    p.add("dtcm_total",          info.dtcm_total);
    p.add("dtcm_static",         info.dtcm_static);
    p.add("dtcm_stack_avail",    info.dtcm_stack_avail);

    p.add("stack_current",       info.stack_current);
    p.add("stack_high_water",    info.stack_high_water);
    p.add("stack_usage_pct",     info.stack_usage_pct);

    // ==========================================================
    // Heap (RAM2)
    // ==========================================================

    p.add("heap_total",          info.heap_total);
    p.add("heap_arena",          info.heap_arena);
    p.add("heap_used",           info.heap_used);
    p.add("heap_free_internal",  info.heap_free_internal);
    p.add("heap_free_above",     info.heap_free_above);
    p.add("heap_free_total",     info.heap_free_total);

    p.add("heap_arena_high_water", info.heap_arena_high_water);

    // ==========================================================
    // Health indicators
    // ==========================================================

    p.add("heap_fragmentation_pct", info.heap_fragmentation_pct);
    p.add("heap_growing",           info.heap_growing);

    return p;
}

// ------------------------------------------------------------
// ENTER_BOOTLOADER — terminal, irreversible
// ------------------------------------------------------------
static Payload cmd_enter_bootloader(const Payload& /*args*/) {

  timepop_arm(
    FLASH_DELAY_NS,
    false,
    enter_bootloader_cb,
    nullptr,
    "bootloader-flash"
  );

  {
    Payload ev;
    ev.add("action", "scheduled");
    enqueueEvent("SYSTEM_ENTER_BOOTLOADER", ev);
  }

  return ok_payload();
}

// ------------------------------------------------------------
// SHUTDOWN — terminal, irreversible
// ------------------------------------------------------------
static Payload cmd_shutdown(const Payload& /*args*/) {

  {
    Payload ev;
    ev.add("action", "requested");
    enqueueEvent("SYSTEM_SHUTDOWN", ev);
  }

  system_request_shutdown();
  return ok_payload();
}

// ------------------------------------------------------------
// PROCESS_LIST — registry introspection (diagnostic only)
// ------------------------------------------------------------
static Payload cmd_process_list(const Payload&) {

  Payload p;
  PayloadArray arr;

  for (size_t i = 0; i < process_get_count(); i++) {
    Payload entry;
    entry.add("name", process_get_name(i));
    arr.add(entry);
  }

  p.add_array("processes", arr);
  return p;
}

// ------------------------------------------------------------
// FEATURES / REPORT_FEATURES — local Teensy feature-state tree
// ------------------------------------------------------------
static Payload cmd_features(const Payload&) {
  return system_features_tree_payload();
}

// ------------------------------------------------------------
// SET_FEATURE — manual/test ingress for local Teensy feature state
// ------------------------------------------------------------
static Payload cmd_set_feature(const Payload& args) {
  const char* subsystem = args.getString("subsystem");
  const char* feature   = args.getString("feature");
  const char* status    = args.getString("status");
  const char* detail    = args.getString("detail");

  if (!subsystem || !*subsystem || !feature || !*feature || !status || !*status) {
    Payload err;
    err.add("error", "SET_FEATURE requires subsystem, feature, status");
    return err;
  }

  if (!system_feature_set_str(subsystem, feature, status, detail)) {
    Payload err;
    err.add("error", "invalid feature status or registry full");
    err.add("allowed", "INITIALIZING NOMINAL HOLD ANOMALY");
    return err;
  }

  Payload resp;
  resp.add("status", "OK");
  resp.add("subsystem", subsystem);
  resp.add("feature", feature);
  resp.add("value", system_feature_get_status(subsystem, feature));
  return resp;
}

// ------------------------------------------------------------
// GET_FEATURE — local Teensy feature-state lookup
// ------------------------------------------------------------
static Payload cmd_get_feature(const Payload& args) {
  const char* subsystem = args.getString("subsystem");
  const char* feature   = args.getString("feature");

  if (!subsystem || !*subsystem || !feature || !*feature) {
    Payload err;
    err.add("error", "GET_FEATURE requires subsystem and feature");
    return err;
  }

  const int idx = system_feature_find(subsystem, feature);
  Payload resp;
  resp.add("known", idx >= 0);
  resp.add("subsystem", subsystem);
  resp.add("feature", feature);
  resp.add("status", system_feature_get_status(subsystem, feature));
  return resp;
}

// ------------------------------------------------------------
// DEBUG — raw debug channel test
// ------------------------------------------------------------
static Payload cmd_debug(const Payload& args) {

  Payload resp;

  if (!args.has("length")) {
    resp.add("error", "missing length");
    return resp;
  }

  uint32_t length = args.getUInt("length");
  if (length == 0 || length > 1024) {
    resp.add("error", "length must be 1..1024");
    return resp;
  }

  char buf[length];
  memset(buf, 'A', sizeof(buf));
  debug_log(
    "system.debug",
    reinterpret_cast<const uint8_t*>(buf),
    sizeof(buf)
  );
  return ok_payload();
}

// ------------------------------------------------------------
// STATUS — simple response of liveness
// ------------------------------------------------------------
static Payload cmd_status(const Payload& /*args*/) {
  return ok_payload();
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t SYSTEM_COMMANDS[] = {
  { "REPORT",           cmd_report           },
  { "PROCESS_INFO",     cmd_process_info     },
  { "TRANSPORT_INFO",   cmd_transport_info   },
  { "PAYLOAD_INFO",     cmd_payload_info     },
  { "MEMORY_INFO",      cmd_memory_info      },
  { "ENTER_BOOTLOADER", cmd_enter_bootloader },
  { "SHUTDOWN",         cmd_shutdown         },
  { "PROCESS_LIST",     cmd_process_list     },
  { "FEATURES",         cmd_features         },
  { "REPORT_FEATURES",  cmd_features         },
  { "SET_FEATURE",      cmd_set_feature      },
  { "GET_FEATURE",      cmd_get_feature      },
  { "DEBUG",            cmd_debug            },
  { "STATUS",           cmd_status           },
  { nullptr,            nullptr }
};

static const process_vtable_t SYSTEM_PROCESS = {
  .process_id    = "SYSTEM",
  .commands      = SYSTEM_COMMANDS,
  .subscriptions = nullptr,
};

void process_system_register(void) {
  system_feature_set("SYSTEM", "FEATURE_STATUS",
                     system_feature_status_t::NOMINAL,
                     "Teensy SYSTEM feature registry online");
  process_register("SYSTEM", &SYSTEM_PROCESS);
  g_system_feature_fragment_publish_enabled = true;
  system_feature_schedule_fragment_publish();
}