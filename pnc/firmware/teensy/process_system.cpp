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
#include <CrashReport.h>

static constexpr uint64_t FLASH_DELAY_NS = 5000000000ULL;  // 5 seconds

// --------------------------------------------------------------
// Forward declarations (terminal paths)
// --------------------------------------------------------------
void system_enter_quiescence(void);

// Bootloader entry symbol (ROM-provided, never returns)
extern "C" void enter_bootloader_cleanly(void);

static FLASHMEM Payload system_features_tree_payload(void);
static void system_feature_schedule_fragment_publish(void);

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

static system_feature_slot_t g_system_features[SYSTEM_FEATURE_MAX_FEATURES] DMAMEM = {};

static void system_feature_registry_reset(void) {
  for (size_t i = 0; i < SYSTEM_FEATURE_MAX_FEATURES; i++) {
    g_system_features[i] = system_feature_slot_t{};
  }
}

static char system_ascii_fold(char c) {
  return (c >= 'a' && c <= 'z') ? (char)(c - ('a' - 'A')) : c;
}

static bool system_cstr_equal(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    if (*a != *b) return false;
    ++a;
    ++b;
  }
  return *a == *b;
}

static bool system_cstr_equal_ci(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    if (system_ascii_fold(*a) != system_ascii_fold(*b)) return false;
    ++a;
    ++b;
  }
  return *a == *b;
}

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

  if (system_cstr_equal_ci(status, "NOMINAL")) {
    *out = system_feature_status_t::NOMINAL;
    return true;
  }
  if (system_cstr_equal_ci(status, "INITIALIZING")) {
    *out = system_feature_status_t::INITIALIZING;
    return true;
  }
  if (system_cstr_equal_ci(status, "HOLD")) {
    *out = system_feature_status_t::HOLD;
    return true;
  }
  if (system_cstr_equal_ci(status, "ANOMALY") || system_cstr_equal_ci(status, "DOWN")) {
    *out = system_feature_status_t::ANOMALY;
    return true;
  }

  return false;
}

static FLASHMEM void system_feature_fragment_publish_service(timepop_ctx_t*,
                                                    timepop_diag_t*,
                                                    void*) {
  g_system_feature_fragment_service_armed = false;

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
    if (!system_cstr_equal(g_system_features[i].subsystem, subsystem)) continue;
    if (!system_cstr_equal(g_system_features[i].feature, feature)) continue;
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
  const int idx = system_feature_find(subsystem, feature);
  return idx >= 0 &&
         g_system_features[idx].status == system_feature_status_t::NOMINAL;
}

static FLASHMEM Payload system_features_tree_payload(void) {
  Payload teensy;

  for (size_t i = 0; i < SYSTEM_FEATURE_MAX_FEATURES; i++) {
    if (!g_system_features[i].used) continue;

    const char* subsystem = g_system_features[i].subsystem;

    bool already_emitted = false;
    for (size_t j = 0; j < i; j++) {
      if (!g_system_features[j].used) continue;
      if (system_cstr_equal(g_system_features[j].subsystem, subsystem)) {
        already_emitted = true;
        break;
      }
    }
    if (already_emitted) continue;

    Payload subsystem_payload;
    for (size_t k = 0; k < SYSTEM_FEATURE_MAX_FEATURES; k++) {
      if (!g_system_features[k].used) continue;
      if (!system_cstr_equal(g_system_features[k].subsystem, subsystem)) continue;

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
// Teensy core CrashReport bridge
// ================================================================
//
// Safe first-pass crash forensics:
//
//   • Do not replace ARM fault handlers.
//   • Do not add retained memory of our own.
//   • Do not touch CrashReport from normal SYSTEM.REPORT except for the cheap
//     non-destructive operator-bool presence check.
//   • Capture the printable CrashReport text only on explicit CRASH_INFO.
//
// Important Teensyduino behavior: CrashReport.printTo() clears the underlying
// core crash/reset record after printing.  We therefore cache the first printed
// text in a static buffer so repeated CRASH_INFO calls during this boot do not
// destroy the operator's view of the evidence.
//

static constexpr size_t SYSTEM_CRASH_REPORT_TEXT_MAX = 2048;

static bool g_system_crash_report_captured = false;
static bool g_system_crash_report_core_fault_present = false;
static bool g_system_crash_report_truncated = false;
static uint32_t g_system_crash_report_bytes = 0;
static char g_system_crash_report_text[SYSTEM_CRASH_REPORT_TEXT_MAX] DMAMEM = {0};
static char g_system_debug_buffer[1024] DMAMEM = {0};

class system_crash_buffer_print_t : public Print {
 public:
  system_crash_buffer_print_t(char* buffer, size_t capacity)
      : _buffer(buffer),
        _capacity(capacity),
        _length(0),
        _truncated(false) {
    if (_capacity > 0 && _buffer) {
      _buffer[0] = '\0';
    }
  }

  size_t write(uint8_t b) override {
    if (!_buffer || _capacity == 0) {
      _truncated = true;
      return 1;
    }

    if (_length + 1U < _capacity) {
      _buffer[_length++] = static_cast<char>(b);
      _buffer[_length] = '\0';
    } else {
      _truncated = true;
    }
    return 1;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    if (!buffer) return 0;
    for (size_t i = 0; i < size; i++) {
      write(buffer[i]);
    }
    return size;
  }

  size_t length() const { return _length; }
  bool truncated() const { return _truncated; }

 private:
  char* _buffer;
  size_t _capacity;
  size_t _length;
  bool _truncated;
};

static FLASHMEM void system_crash_report_capture_once(void) {
  if (g_system_crash_report_captured) {
    return;
  }

  g_system_crash_report_captured = true;
  g_system_crash_report_core_fault_present = (bool)CrashReport;

  system_crash_buffer_print_t out(
      g_system_crash_report_text,
      sizeof(g_system_crash_report_text));

  CrashReport.printTo(out);

  g_system_crash_report_bytes = (uint32_t)out.length();
  g_system_crash_report_truncated = out.truncated();
}

static FLASHMEM Payload system_crash_report_payload(bool include_text) {
  Payload p;
  p.add("core_fault_present_now", (bool)CrashReport);
  p.add("captured", g_system_crash_report_captured);
  p.add("captured_core_fault_present", g_system_crash_report_core_fault_present);
  p.add("bytes", g_system_crash_report_bytes);
  p.add("truncated", g_system_crash_report_truncated);
  if (include_text) {
    p.add("text", g_system_crash_report_text);
  }
  return p;
}

// ================================================================
// Terminal helpers
// ================================================================

static FLASHMEM void enter_bootloader_cb(timepop_ctx_t*, timepop_diag_t*, void*) {

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
static FLASHMEM Payload cmd_report(const Payload& /*args*/) {

  Payload p;

  // Firmware identity
  p.add("fw_version", FW_VERSION);

  // CrashReport presence only.  This is intentionally non-destructive: the
  // full printable CrashReport is captured by explicit SYSTEM.CRASH_INFO.
  p.add("crash_report_present", (bool)CrashReport);
  p.add("crash_report_captured", g_system_crash_report_captured);
  p.add("crash_report_bytes", g_system_crash_report_bytes);
  p.add("crash_report_truncated", g_system_crash_report_truncated);

  // CPU clock frequency — authoritative runtime value.
  // F_CPU_ACTUAL is updated by set_arm_clock() at boot.
  // This is the ACTUAL core clock, not the compile-time default.
  p.add("cpu_freq_mhz", (uint32_t)(F_CPU_ACTUAL / 1000000UL));

  // CPU temperature (best-effort)
  p.add("cpu_temp_c", cpuTempC());

  // Internal reference voltage (best-effort)
  p.add("vref_v", readVrefVolts());

  // Heap availability
  p.add("free_heap_bytes", freeHeapBytes());
  p.add("max_alloc_bytes", maxAllocBytes());

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
  p.add("feature_status_fragment_service_arm_failures", g_system_feature_fragment_service_arm_failures);

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
//   PROCESS_INFO is itself an RPC handler. While this report is being built,
//   the current request has already been received, routed, and invoked, but
//   the handler has not yet returned and this response has not yet been sent.
//   Therefore a healthy live snapshot may legitimately show exactly one
//   handler in flight:
//     received == routed + error_missing_fields
//                        + error_unknown_subsys
//                        + error_unknown_command
//     routed == handler_invoked
//     handler_invoked == handler_completed
//        OR handler_invoked == handler_completed + 1
//     handler_completed == response_sent
//
//   More than one in-flight handler, impossible counter ordering, or a
//   completed response that was not sent remains a smoking gun.
// ============================================================================

static FLASHMEM Payload cmd_process_info(const Payload& /*args*/) {

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

    const bool counter_order_ok =
        info.received >= info.routed &&
        info.routed >= info.handler_invoked &&
        info.handler_invoked >= info.handler_completed &&
        info.handler_completed >= info.response_sent;

    const uint32_t routed_minus_invoked =
        (info.routed >= info.handler_invoked)
            ? (info.routed - info.handler_invoked)
            : 0xFFFFFFFFUL;
    const uint32_t handler_inflight =
        (info.handler_invoked >= info.handler_completed)
            ? (info.handler_invoked - info.handler_completed)
            : 0xFFFFFFFFUL;
    const uint32_t response_pending =
        (info.handler_completed >= info.response_sent)
            ? (info.handler_completed - info.response_sent)
            : 0xFFFFFFFFUL;

    p.add("rpc_counter_order_ok", counter_order_ok);
    p.add("rpc_routed_minus_invoked", routed_minus_invoked);
    p.add("rpc_handler_inflight", handler_inflight);
    p.add("rpc_response_pending", response_pending);
    p.add("rpc_snapshot_allows_current_handler", true);
    p.add("rpc_current_handler_expected", handler_inflight == 1U);

    p.add("invariant_received_ok", info.received == expected_received);
    p.add("invariant_routed_ok", counter_order_ok && routed_minus_invoked == 0U);
    p.add("invariant_handler_ok", counter_order_ok && handler_inflight <= 1U);
    p.add("invariant_response_ok", counter_order_ok && response_pending == 0U);

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
static FLASHMEM Payload cmd_transport_info(const Payload& /*args*/) {

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

  // ==========================================================
  // RX — Startup attach quarantine / first-corruption witness
  // ==========================================================

  p.add("rx_first_frame_seen", info.rx_first_frame_seen != 0U);
  p.add("rx_startup_grace_active", info.rx_startup_grace_active != 0U);
  p.add("rx_startup_grace_ms", info.rx_startup_grace_ms);
  p.add("rx_ms_since_init", info.rx_ms_since_init);
  p.add("rx_startup_expected_traffic_missing_suppressed",
        info.rx_startup_expected_traffic_missing_suppressed);
  p.add("rx_startup_bad_stx_suppressed", info.rx_startup_bad_stx_suppressed);
  p.add("rx_startup_bad_etx_suppressed", info.rx_startup_bad_etx_suppressed);
  p.add("rx_startup_len_overflow_suppressed",
        info.rx_startup_len_overflow_suppressed);
  p.add("rx_startup_first_missing_byte", info.rx_startup_first_missing_byte);
  p.add("rx_startup_last_missing_byte", info.rx_startup_last_missing_byte);

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
static FLASHMEM Payload cmd_payload_info(const Payload& /*args*/) {

  payload_info_t info{};
  payload_get_info(&info);

  Payload p;

  // ==========================================================
  // ABI / geometry — proves stack footprint and hard limits
  // ==========================================================

  p.add("payload_object_size",       info.payload_object_size);
  p.add("payload_entry_size",        info.payload_entry_size);
  p.add("payload_array_object_size", info.payload_array_object_size);
  p.add("payload_inline_entries",    info.payload_inline_entries);
  p.add("payload_max_entries",       info.payload_max_entries);
  p.add("payload_arena_initial",     info.payload_arena_initial);
  p.add("payload_arena_max",         info.payload_arena_max);

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
  // Entry table behavior
  // ==========================================================

  p.add("payload_entry_alloc_fail",            info.entry_alloc_fail);
  p.add("payload_entry_realloc_count",         info.entry_realloc_count);
  p.add("payload_entry_heap_bytes_alive",      info.entry_heap_bytes_alive);
  p.add("payload_entry_heap_bytes_high_water", info.entry_heap_bytes_high_water);
  p.add("payload_entry_overflow",              info.entry_overflow);
  p.add("payload_entry_high_water",            info.entry_high_water);
  p.add("payload_max_entry_capacity_seen",     info.max_entry_capacity_seen);

  // ==========================================================
  // Arena behavior
  // ==========================================================

  p.add("payload_arena_alloc_fail",            info.arena_alloc_fail);
  p.add("payload_arena_realloc_count",         info.arena_realloc_count);
  p.add("payload_arena_heap_bytes_alive",      info.arena_heap_bytes_alive);
  p.add("payload_arena_heap_bytes_high_water", info.arena_heap_bytes_high_water);
  p.add("payload_arena_high_water",            info.arena_high_water);
  p.add("payload_max_arena_capacity_seen",     info.max_arena_capacity_seen);

  // Combined heap custody for the Payload subsystem.
  p.add("payload_heap_bytes_alive",
        info.entry_heap_bytes_alive + info.arena_heap_bytes_alive);
  p.add("payload_heap_bytes_high_water",
        info.entry_heap_bytes_high_water + info.arena_heap_bytes_high_water);

  // ==========================================================
  // Serialization / parsing / integrity
  // ==========================================================

  p.add("payload_serialize_overflow", info.serialize_overflow);
  p.add("payload_to_json_fail",       info.to_json_fail);
  p.add("payload_string_truncation",  info.string_truncation);
  p.add("payload_parse_error",        info.parse_error);
  p.add("payload_integrity_fail",     info.integrity_fail);
  p.add("payload_invalid_kind",       info.invalid_kind);

  // ==========================================================
  // Defensive C-string pointer custody
  // ==========================================================

  p.add("payload_string_pointer_fault", info.string_pointer_fault);
  p.add("payload_string_pointer_null", info.string_pointer_null);
  p.add("payload_string_pointer_low_address", info.string_pointer_low_address);
  p.add("payload_string_pointer_magic_address", info.string_pointer_magic_address);
  p.add("payload_string_pointer_out_of_range", info.string_pointer_out_of_range);
  p.add("payload_string_pointer_span_out_of_range", info.string_pointer_span_out_of_range);
  p.add("payload_string_pointer_unterminated", info.string_pointer_unterminated);
  p.add("payload_string_pointer_too_long", info.string_pointer_too_long);
  p.add("payload_last_string_pointer_fault_reason", info.last_string_pointer_fault_reason);
  p.add("payload_last_string_pointer_fault_reason_name",
        payload_string_fault_reason_name(info.last_string_pointer_fault_reason));
  p.add_fmt("payload_last_string_pointer_fault_ptr", "0x%08lX",
            (unsigned long)info.last_string_pointer_fault_ptr);
  p.add("payload_last_string_pointer_fault_context",
        info.last_string_pointer_fault_context);
  p.add_fmt("payload_last_string_pointer_fault_key_ptr", "0x%08lX",
            (unsigned long)info.last_string_pointer_fault_key_ptr);
  p.add("payload_last_string_pointer_fault_key",
        info.last_string_pointer_fault_key);

  // ==========================================================
  // Payload object integrity courtroom
  // ==========================================================

  p.add("payload_self_ok_fail", info.self_ok_fail);
  p.add("payload_self_ok_magic_bad", info.self_ok_magic_bad);
  p.add("payload_self_ok_entries_null", info.self_ok_entries_null);
  p.add("payload_self_ok_entry_cap_low", info.self_ok_entry_cap_low);
  p.add("payload_self_ok_entry_cap_high", info.self_ok_entry_cap_high);
  p.add("payload_self_ok_entries_magic_address", info.self_ok_entries_magic_address);
  p.add("payload_self_ok_arena_magic_address", info.self_ok_arena_magic_address);
  p.add("payload_self_ok_entries_span_unreadable", info.self_ok_entries_span_unreadable);
  p.add("payload_self_ok_arena_span_unreadable", info.self_ok_arena_span_unreadable);
  p.add("payload_self_ok_inline_cap_mismatch", info.self_ok_inline_cap_mismatch);
  p.add("payload_self_ok_heap_cap_mismatch", info.self_ok_heap_cap_mismatch);
  p.add("payload_self_ok_count_gt_entry_cap", info.self_ok_count_gt_entry_cap);
  p.add("payload_self_ok_count_gt_max", info.self_ok_count_gt_max);
  p.add("payload_self_ok_arena_used_gt_cap", info.self_ok_arena_used_gt_cap);
  p.add("payload_self_ok_arena_cap_gt_max", info.self_ok_arena_cap_gt_max);
  p.add("payload_self_ok_arena_cap_zero_with_ptr", info.self_ok_arena_cap_zero_with_ptr);
  p.add("payload_self_ok_arena_cap_nonzero_with_null", info.self_ok_arena_cap_nonzero_with_null);
  p.add("payload_self_ok_count_without_arena", info.self_ok_count_without_arena);
  p.add("payload_self_ok_entry_kind_bad", info.self_ok_entry_kind_bad);
  p.add("payload_self_ok_entry_key_off_oob", info.self_ok_entry_key_off_oob);
  p.add("payload_self_ok_entry_val_off_oob", info.self_ok_entry_val_off_oob);
  p.add("payload_self_ok_entry_val_end_oob", info.self_ok_entry_val_end_oob);
  p.add("payload_self_ok_entry_val_unterminated", info.self_ok_entry_val_unterminated);
  p.add("payload_self_ok_entry_key_unterminated", info.self_ok_entry_key_unterminated);
  p.add("payload_last_self_ok_fail_reason", info.last_self_ok_fail_reason);
  p.add("payload_last_self_ok_fail_reason_name",
        payload_self_ok_fail_reason_name(info.last_self_ok_fail_reason));
  p.add("payload_last_self_ok_fail_op", info.last_self_ok_fail_op);
  p.add_fmt("payload_last_self_ok_fail_this", "0x%08lX",
            (unsigned long)info.last_self_ok_fail_this);
  p.add_fmt("payload_last_self_ok_fail_magic", "0x%08lX",
            (unsigned long)info.last_self_ok_fail_magic);
  p.add_fmt("payload_last_self_ok_fail_entries", "0x%08lX",
            (unsigned long)info.last_self_ok_fail_entries);
  p.add_fmt("payload_last_self_ok_fail_arena", "0x%08lX",
            (unsigned long)info.last_self_ok_fail_arena);
  p.add("payload_last_self_ok_fail_count", info.last_self_ok_fail_count);
  p.add("payload_last_self_ok_fail_entry_cap", info.last_self_ok_fail_entry_cap);
  p.add("payload_last_self_ok_fail_arena_used", info.last_self_ok_fail_arena_used);
  p.add("payload_last_self_ok_fail_arena_cap", info.last_self_ok_fail_arena_cap);
  p.add("payload_last_self_ok_fail_entry_index", info.last_self_ok_fail_entry_index);
  p.add("payload_last_self_ok_fail_entry_key_off", info.last_self_ok_fail_entry_key_off);
  p.add("payload_last_self_ok_fail_entry_val_off", info.last_self_ok_fail_entry_val_off);
  p.add("payload_last_self_ok_fail_entry_val_len", info.last_self_ok_fail_entry_val_len);
  p.add("payload_last_self_ok_fail_entry_kind", info.last_self_ok_fail_entry_kind);

  // ==========================================================
  // Last error breadcrumb
  // ==========================================================

  p.add("payload_last_error_code",  info.last_error_code);
  p.add("payload_last_error_name",  payload_error_code_name(info.last_error_code));
  p.add("payload_last_error_count", info.last_error_count);
  p.add("payload_last_error_op",    info.last_error_op);
  p.add_fmt("payload_last_error_this", "0x%08lX", (unsigned long)info.last_error_this);

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

static FLASHMEM Payload cmd_memory_info(const Payload& /*args*/) {

    memory_info_t info{};
    memory_info_get(&info);

    Payload p;

    // ==========================================================
    // DTCM / Stack
    // ==========================================================

    p.add("dtcm_total",          info.dtcm_total);
    p.add("dtcm_static",         info.dtcm_static);
    p.add("dtcm_stack_avail",    info.dtcm_stack_avail);
    p.add("memory_info_initialized", info.initialized);
    p.add_fmt("stack_init_sp", "0x%08lX",
              (unsigned long)info.stack_init_sp);

    p.add("stack_current",       info.stack_current);
    p.add("stack_high_water",    info.stack_high_water);
    p.add("stack_usage_pct",     info.stack_usage_pct);
    p.add("stack_free_pct",      info.stack_free_pct);
    p.add("stack_free_current",  info.stack_free_current);
    p.add("stack_free_high_water", info.stack_free_high_water);
    p.add("stack_collision_warn_bytes", info.stack_collision_warn_bytes);
    p.add("stack_collision_risk", info.stack_collision_risk);

    p.add("stack_paint_enabled",      info.stack_paint_enabled);
    p.add("stack_paint_overrun",      info.stack_paint_overrun);
    p.add_fmt("stack_paint_pattern", "0x%08lX",
              (unsigned long)info.stack_paint_pattern);
    p.add_fmt("stack_paint_start", "0x%08lX",
              (unsigned long)info.stack_paint_start);
    p.add_fmt("stack_paint_end", "0x%08lX",
              (unsigned long)info.stack_paint_end);
    p.add_fmt("stack_paint_deepest_addr", "0x%08lX",
              (unsigned long)info.stack_paint_deepest_addr);
    p.add("stack_paint_guard_bytes", info.stack_paint_guard_bytes);
    p.add("stack_paint_bytes",       info.stack_paint_bytes);
    p.add("stack_paint_used",        info.stack_paint_used);
    p.add("stack_paint_unused",      info.stack_paint_unused);
    p.add("stack_paint_clobbered",   info.stack_paint_clobbered);

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
static FLASHMEM Payload cmd_enter_bootloader(const Payload& /*args*/) {

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
static FLASHMEM Payload cmd_shutdown(const Payload& /*args*/) {

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
static FLASHMEM Payload cmd_process_list(const Payload&) {

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
static FLASHMEM Payload cmd_features(const Payload&) {
  return system_features_tree_payload();
}

// ------------------------------------------------------------
// SET_FEATURE — manual/test ingress for local Teensy feature state
// ------------------------------------------------------------
static FLASHMEM Payload cmd_set_feature(const Payload& args) {
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
static FLASHMEM Payload cmd_get_feature(const Payload& args) {
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
// CRASH_INFO — explicit printable Teensy core CrashReport capture
// ------------------------------------------------------------
static FLASHMEM Payload cmd_crash_info(const Payload& /*args*/) {
  system_crash_report_capture_once();
  return system_crash_report_payload(true);
}

// ------------------------------------------------------------
// CRASH_CLEAR — explicitly clear cached and core CrashReport state
// ------------------------------------------------------------
static FLASHMEM Payload cmd_crash_clear(const Payload& /*args*/) {
  CrashReport.clear();
  g_system_crash_report_captured = false;
  g_system_crash_report_core_fault_present = false;
  g_system_crash_report_truncated = false;
  g_system_crash_report_bytes = 0;
  g_system_crash_report_text[0] = '\0';

  Payload resp = ok_payload();
  resp.add("crash_report_cleared", true);
  return resp;
}

// ------------------------------------------------------------
// DEBUG — raw debug channel test
// ------------------------------------------------------------
static FLASHMEM Payload cmd_debug(const Payload& args) {

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

  memset(g_system_debug_buffer, 'A', length);
  debug_log(
    "system.debug",
    reinterpret_cast<const uint8_t*>(g_system_debug_buffer),
    length
  );
  return ok_payload();
}

// ------------------------------------------------------------
// STATUS — simple response of liveness
// ------------------------------------------------------------
static FLASHMEM Payload cmd_status(const Payload& /*args*/) {
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
  { "CRASH_INFO",       cmd_crash_info       },
  { "CRASH_CLEAR",      cmd_crash_clear      },
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
  system_feature_registry_reset();

  system_feature_set("SYSTEM", "FEATURE_STATUS",
                     system_feature_status_t::NOMINAL,
                     "Teensy SYSTEM feature registry online");
  process_register("SYSTEM", &SYSTEM_PROCESS);
  g_system_feature_fragment_publish_enabled = true;
  system_feature_schedule_fragment_publish();
}