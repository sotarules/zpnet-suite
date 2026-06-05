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
#include "cpu_usage.h"
#include "util.h"
#include "timepop.h"
#include "process_timepop.h"
#include "debug.h"
#include "transport.h"   // <-- NEW (for transport_get_info)

static constexpr uint64_t FLASH_DELAY_NS = 5000000000ULL;  // 5 seconds

// --------------------------------------------------------------
// Forward declarations (terminal paths)
// --------------------------------------------------------------
void system_enter_quiescence(void);

// Bootloader entry symbol (ROM-provided, never returns)
extern "C" void enter_bootloader_cleanly(void);

// --------------------------------------------------------------
// Internal terminal state
// --------------------------------------------------------------
static bool system_shutdown   = false;
static bool system_bootloader = false;

static bool system_cpu_window_initialized = false;
static uint64_t system_cpu_window_last_wall_cycles = 0;
static uint64_t system_cpu_window_last_idle_cycles = 0;

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

  // Legacy accounted callback-busy diagnostics.
  p.add("cpu_accounted_busy_pct", cpu_usage_get_percent());
  p.add("cpu_busy_cycles", cpu_usage_get_busy_cycles());
  p.add("cpu_total_cycles", cpu_usage_get_total_cycles());
  p.add("cpu_sample_window_ms", cpu_usage_get_sample_window_ms());

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
//   the current command has already been received/routed/invoked, but it has
//   not yet completed and its response has not yet been sent. Therefore a
//   healthy snapshot may legitimately show exactly one handler in flight:
//
//     received == routed + error_missing_fields
//                        + error_unknown_subsys
//                        + error_unknown_command
//     routed == handler_invoked
//     handler_invoked == handler_completed
//        OR handler_invoked == handler_completed + 1
//     handler_completed == response_sent
//
//   More than one in-flight handler, or completed responses not sent, remains
//   a smoking gun.
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

    const uint32_t handler_inflight =
        (info.handler_invoked >= info.handler_completed)
            ? (info.handler_invoked - info.handler_completed)
            : 0xFFFFFFFFUL;
    const uint32_t response_pending =
        (info.handler_completed >= info.response_sent)
            ? (info.handler_completed - info.response_sent)
            : 0xFFFFFFFFUL;

    p.add("rpc_handler_inflight", handler_inflight);
    p.add("rpc_response_pending", response_pending);

    p.add("invariant_received_ok",  info.received == expected_received);
    p.add("invariant_handler_ok",   handler_inflight <= 1U);
    p.add("invariant_response_ok",  response_pending == 0U);

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


// ============================================================================
// cmd_footprint_info — compile-time/static footprint visibility
// ============================================================================
//
// Register in your SYSTEM process command table as:
//
//   { "FOOTPRINT_INFO", cmd_footprint_info },
//
// Query from Pi:
//   .tc system footprint_info
//
// Semantics:
//   • Read-only
//   • Snapshot-only
//   • No scheduler work
//   • No persistent state
//   • Reports compile-time sizeof(...) facts for structures visible here
//
// This report is intentionally SYSTEM-local.  It exposes framework and SYSTEM
// structure sizes without pulling private process_interrupt implementation
// types into public headers.  Process-private feature footprints should be
// reported by the owning process next to the private structs they measure.
// ============================================================================

static FLASHMEM Payload cmd_footprint_info(const Payload& /*args*/) {

  Payload p;

  p.add("report", "SYSTEM_FOOTPRINT_INFO");
  p.add("scope", "SYSTEM_VISIBLE_TYPES_ONLY");
  p.add("private_process_types_included", false);
  p.add("notes", "Use owning process reports for private feature structs");

  // ==========================================================
  // ABI / scalar sizes
  // ==========================================================

  p.add("sizeof_bool",    (uint32_t)sizeof(bool));
  p.add("sizeof_char",    (uint32_t)sizeof(char));
  p.add("sizeof_int",     (uint32_t)sizeof(int));
  p.add("sizeof_long",    (uint32_t)sizeof(long));
  p.add("sizeof_float",   (uint32_t)sizeof(float));
  p.add("sizeof_double",  (uint32_t)sizeof(double));
  p.add("sizeof_pointer", (uint32_t)sizeof(void*));
  p.add("sizeof_size_t",  (uint32_t)sizeof(size_t));
  p.add("sizeof_uint32_t", (uint32_t)sizeof(uint32_t));
  p.add("sizeof_uint64_t", (uint32_t)sizeof(uint64_t));

  // ==========================================================
  // Core framework / report types visible to SYSTEM
  // ==========================================================

  p.add("sizeof_payload",             (uint32_t)sizeof(Payload));
  p.add("sizeof_payload_array",       (uint32_t)sizeof(PayloadArray));
  p.add("sizeof_memory_info_t",       (uint32_t)sizeof(memory_info_t));
  p.add("sizeof_process_rpc_info_t",  (uint32_t)sizeof(process_rpc_info_t));
  p.add("sizeof_transport_info_t",    (uint32_t)sizeof(transport_info_t));
  p.add("sizeof_process_command_entry_t",
        (uint32_t)sizeof(process_command_entry_t));
  p.add("sizeof_process_vtable_t",    (uint32_t)sizeof(process_vtable_t));
  p.add("sizeof_timepop_ctx_t",       (uint32_t)sizeof(timepop_ctx_t));
  p.add("sizeof_timepop_diag_t",      (uint32_t)sizeof(timepop_diag_t));
  p.add("sizeof_timepop_idle_witness_snapshot_t",
        (uint32_t)sizeof(timepop_idle_witness_snapshot_t));

  // ==========================================================
  // SYSTEM module-local static state known to this translation unit
  // ==========================================================

  p.add("system_local_static_known_bytes",
        (uint32_t)(sizeof(system_shutdown) +
                   sizeof(system_bootloader) +
                   sizeof(system_cpu_window_initialized) +
                   sizeof(system_cpu_window_last_wall_cycles) +
                   sizeof(system_cpu_window_last_idle_cycles)));

  // ==========================================================
  // Current macro memory context for correlation with sizeof deltas
  // ==========================================================

  memory_info_t mem{};
  memory_info_get(&mem);
  p.add("dtcm_total_current",       mem.dtcm_total);
  p.add("dtcm_static_current",      mem.dtcm_static);
  p.add("dtcm_stack_avail_current", mem.dtcm_stack_avail);
  p.add("heap_total_current",       mem.heap_total);
  p.add("heap_used_current",        mem.heap_used);
  p.add("heap_free_total_current",  mem.heap_free_total);
  p.add("heap_arena_high_water",    mem.heap_arena_high_water);

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
  { "FOOTPRINT_INFO",   cmd_footprint_info   },
  { "ENTER_BOOTLOADER", cmd_enter_bootloader },
  { "SHUTDOWN",         cmd_shutdown         },
  { "PROCESS_LIST",     cmd_process_list     },
  { "DEBUG",            cmd_debug            },
  { "STATUS",           cmd_status           },
  { nullptr,            nullptr }
};

static const process_vtable_t SYSTEM_PROCESS = {
  .process_id    = "SYSTEM",
  .commands      = SYSTEM_COMMANDS,
  .subscriptions = nullptr,
};

void FLASHMEM process_system_register(void) {
  process_register("SYSTEM", &SYSTEM_PROCESS);
}