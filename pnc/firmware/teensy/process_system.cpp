// =============================================================
// FILE: process_system.cpp
// =============================================================
//
// SYSTEM Process — Teensy-side system truth surface
//
// SYSTEM is not a lifecycle-managed process.
// It cannot start or stop from within itself.
//
// It exposes authoritative system facts and provides explicit terminal
// transitions (shutdown, bootloader) that represent irreversible boundary
// crossings.
//
// =============================================================

#include "process_system.h"
#include "crash_forensics.h"
#include "config.h"
#include "process.h"
#include "events.h"
#include "payload.h"
#include "publish.h"
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
static void system_dmamem_ensure_initialized(void);

// --------------------------------------------------------------
// Internal terminal state
// --------------------------------------------------------------
static bool system_shutdown   = false;
static bool system_bootloader = false;

// Ordinary BSS flag: unlike DMAMEM, this is guaranteed zeroed by startup and
// can safely guard the first RAM2 cold initialization.
static bool g_system_dmamem_initialized = false;

static bool system_cpu_window_initialized = false;
static uint64_t system_cpu_window_last_wall_cycles = 0;
static uint64_t system_cpu_window_last_idle_cycles = 0;

// ============================================================================
// Foreground crash ledger
// ============================================================================
//
// This scalar-only ledger records the imperative phase occupied by loop().
// Both compatibility banks use ordinary RAM1 in this deterministic-memory
// baseline.  The retained-bank API remains compatible, but RAM1 startup
// initialization means it does not preserve the previous boot's final image.

static constexpr uint32_t ZPNET_RUNTIME_LEDGER_MAGIC = 0x52554E32UL;  // 'RUN2'

typedef struct {
  uint32_t magic;
  uint32_t magic_inv;

  uint32_t foreground_sequence;
  uint32_t foreground_sequence_inv;
  uint32_t foreground_phase;

  uint32_t reserved[3];
} zpnet_runtime_ledger_t;

static_assert(sizeof(zpnet_runtime_ledger_t) == 32U,
              "runtime crash ledger must stay one cache line");

static zpnet_runtime_ledger_t g_runtime_ledger = {};
static zpnet_runtime_ledger_t g_runtime_ledger_retained = {};
static bool g_runtime_ledger_boot_latched = false;  // BSS: zero every boot

static bool runtime_ledger_header_valid(const zpnet_runtime_ledger_t* ledger) {
  return ledger &&
         ledger->magic == ZPNET_RUNTIME_LEDGER_MAGIC &&
         (ledger->magic ^ ledger->magic_inv) == 0xFFFFFFFFUL;
}

static bool runtime_ledger_foreground_valid(
    const zpnet_runtime_ledger_t* ledger) {
  return runtime_ledger_header_valid(ledger) &&
         (ledger->foreground_sequence ^ ledger->foreground_sequence_inv) ==
             0xFFFFFFFFUL;
}

static void runtime_ledger_initialize_live(void) {
  memset((void*)&g_runtime_ledger, 0, sizeof(g_runtime_ledger));
  g_runtime_ledger.foreground_sequence_inv = 0xFFFFFFFFUL;
  g_runtime_ledger.magic_inv = ~ZPNET_RUNTIME_LEDGER_MAGIC;
  g_runtime_ledger.magic = ZPNET_RUNTIME_LEDGER_MAGIC;
}

static void runtime_ledger_boot_latch(void) {
  if (g_runtime_ledger_boot_latched) return;
  g_runtime_ledger_boot_latched = true;

  // Both banks are ordinary RAM1 and were zeroed by startup.  Keep the
  // compatibility retained surface empty and initialize the live ledger.
  memset((void*)&g_runtime_ledger_retained, 0,
         sizeof(g_runtime_ledger_retained));
  runtime_ledger_initialize_live();
}

void zpnet_foreground_phase_note(zpnet_foreground_phase_t phase) {
  runtime_ledger_boot_latch();

  const uint32_t sequence = g_runtime_ledger.foreground_sequence + 1U;
  g_runtime_ledger.foreground_sequence_inv =
      g_runtime_ledger.foreground_sequence;  // invalidate while mutating
  g_runtime_ledger.foreground_phase = (uint32_t)phase;
  g_runtime_ledger.foreground_sequence_inv = ~sequence;
  g_runtime_ledger.foreground_sequence = sequence;  // commit last
}


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

// The timing subsystems update this registry and foreground serializes it.
// Keep the shared feature-control plane in deterministic RAM1.
static system_feature_slot_t g_system_features[SYSTEM_FEATURE_MAX_FEATURES] = {};

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

// Public FEATURE_STATUS is the mission-readiness annunciator, not a dump of
// every diagnostic witness. QTIMER_DWT_RULER remains in the internal registry
// and in INTERRUPT.REPORT_INTEGRITY, but its ISR-displacement-sensitive state
// must not strobe the operator-facing feature tree.
static bool system_feature_publicly_visible(
    const system_feature_slot_t& slot) {
  return !(system_cstr_equal(slot.subsystem, "INTERRUPT") &&
           system_cstr_equal(slot.feature, "QTIMER_DWT_RULER"));
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
  system_dmamem_ensure_initialized();
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
  if (system_feature_publicly_visible(slot)) {
    system_feature_note_changed();
  }
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
  system_dmamem_ensure_initialized();

  Payload teensy;

  for (size_t i = 0; i < SYSTEM_FEATURE_MAX_FEATURES; i++) {
    if (!g_system_features[i].used) continue;
    if (!system_feature_publicly_visible(g_system_features[i])) continue;

    const char* subsystem = g_system_features[i].subsystem;

    bool already_emitted = false;
    for (size_t j = 0; j < i; j++) {
      if (!g_system_features[j].used) continue;
      if (!system_feature_publicly_visible(g_system_features[j])) continue;
      if (system_cstr_equal(g_system_features[j].subsystem, subsystem)) {
        already_emitted = true;
        break;
      }
    }
    if (already_emitted) continue;

    Payload subsystem_payload;
    for (size_t k = 0; k < SYSTEM_FEATURE_MAX_FEATURES; k++) {
      if (!g_system_features[k].used) continue;
      if (!system_feature_publicly_visible(g_system_features[k])) continue;
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
// Layered crash forensics:
//
//   • crash_forensics captures the complete exception frame, callee-saved
//     registers, fault-control state, MPU/NVIC state, and bounded memory windows
//     before chaining into Teensyduino's core handler.
//   • Teensyduino remains responsible for its standard CrashReport, USB grace,
//     and automatic reboot.
//   • The ZPNet retained record is immutable after reboot until CRASH_CLEAR.
//   • CrashReport.printTo() remains explicit because it clears the core record.
//
// Important Teensyduino behavior: CrashReport.printTo() clears the underlying
// core crash/reset record after printing.  We therefore cache the first printed
// text in a static buffer so repeated CRASH_INFO calls during this boot do not
// destroy the operator's view of the evidence.
//

static FLASHMEM void system_crash_add_hex32(Payload& payload,
                                   const char* key,
                                   uint32_t value) {
  payload.add_fmt(key, "0x%08lX", (unsigned long)value);
}

static FLASHMEM Payload system_crash_word_window_payload(uint32_t base,
                                                uint32_t count,
                                                const uint32_t* words,
                                                size_t capacity) {
  Payload out;
  system_crash_add_hex32(out, "base", base);
  out.add("count", count);

  PayloadArray values;
  const size_t bounded = count < capacity ? count : capacity;
  for (size_t i = 0; i < bounded; ++i) {
    Payload word;
    word.add("index", (uint32_t)i);
    system_crash_add_hex32(word, "address", base + (uint32_t)(i * 4U));
    system_crash_add_hex32(word, "value", words[i]);
    values.add(word);
  }
  out.add_array("words", values);
  return out;
}

static FLASHMEM Payload system_crash_core_forensics_payload(
    const crash_forensics_core_record_t& record) {
  Payload out;
  out.add("schema", "ZPNET_CRASH_FORENSICS_CORE_V1");
  out.add("schema_version", record.schema_version);
  out.add("record_size", record.record_size);
  out.add("capture_sequence", record.capture_sequence);
  out.add("flags", record.flags);
  system_crash_add_hex32(out, "flags_hex", record.flags);
  system_crash_add_hex32(out, "committed", record.committed);
  system_crash_add_hex32(out, "committed_inv", record.committed_inv);

  const bool stage_coherent =
      (record.capture_stage ^ record.capture_stage_inv) == 0xFFFFFFFFUL;
  out.add("capture_stage_coherent", stage_coherent);
  out.add("capture_stage_id", record.capture_stage);
  out.add("capture_stage",
          stage_coherent
              ? crash_forensics_capture_stage_name(record.capture_stage)
              : "INCOHERENT");

  out.add("exception_number", record.exception_number);
  out.add("exception_name",
          crash_forensics_exception_name(record.exception_number));
  out.add("interrupted_exception_number",
          record.interrupted_exception_number);
  out.add("frame_source",
          crash_forensics_frame_source_name(record.frame_source));

  Payload frame;
  system_crash_add_hex32(frame, "raw_frame_address",
                         record.raw_frame_address);
  system_crash_add_hex32(frame, "basic_frame_address",
                         record.basic_frame_address);
  system_crash_add_hex32(frame, "interrupted_sp", record.interrupted_sp);
  system_crash_add_hex32(frame, "exc_return", record.exc_return);
  system_crash_add_hex32(frame, "r0", record.r0);
  system_crash_add_hex32(frame, "r1", record.r1);
  system_crash_add_hex32(frame, "r2", record.r2);
  system_crash_add_hex32(frame, "r3", record.r3);
  system_crash_add_hex32(frame, "r12", record.r12);
  system_crash_add_hex32(frame, "lr", record.stacked_lr);
  system_crash_add_hex32(frame, "pc", record.stacked_pc);
  system_crash_add_hex32(frame, "xpsr", record.stacked_xpsr);
  out.add_object("exception_frame", frame);

  Payload callee_saved;
  system_crash_add_hex32(callee_saved, "r4", record.r4);
  system_crash_add_hex32(callee_saved, "r5", record.r5);
  system_crash_add_hex32(callee_saved, "r6", record.r6);
  system_crash_add_hex32(callee_saved, "r7", record.r7);
  system_crash_add_hex32(callee_saved, "r8", record.r8);
  system_crash_add_hex32(callee_saved, "r9", record.r9);
  system_crash_add_hex32(callee_saved, "r10", record.r10);
  system_crash_add_hex32(callee_saved, "r11", record.r11);
  out.add_object("callee_saved", callee_saved);

  Payload control;
  system_crash_add_hex32(control, "msp", record.original_msp);
  system_crash_add_hex32(control, "psp", record.original_psp);
  system_crash_add_hex32(control, "primask", record.primask);
  system_crash_add_hex32(control, "basepri", record.basepri);
  system_crash_add_hex32(control, "faultmask", record.faultmask);
  system_crash_add_hex32(control, "control", record.control);
  system_crash_add_hex32(control, "dwt_cyccnt", record.dwt_cyccnt);
  control.add("cpu_hz", record.cpu_hz);
  out.add_object("processor_control", control);

  Payload fault;
  system_crash_add_hex32(fault, "cfsr", record.cfsr);
  system_crash_add_hex32(fault, "hfsr", record.hfsr);
  system_crash_add_hex32(fault, "dfsr", record.dfsr);
  system_crash_add_hex32(fault, "afsr", record.afsr);
  system_crash_add_hex32(fault, "mmfar", record.mmfar);
  system_crash_add_hex32(fault, "bfar", record.bfar);
  system_crash_add_hex32(fault, "shcsr", record.shcsr);
  system_crash_add_hex32(fault, "icsr", record.icsr);
  system_crash_add_hex32(fault, "fpccr", record.fpccr);
  system_crash_add_hex32(fault, "fpcar", record.fpcar);
  out.add_object("fault_registers", fault);

  Payload stack = system_crash_word_window_payload(
      record.stack_base,
      record.stack_word_count,
      record.stack_words,
      CRASH_FORENSICS_CORE_STACK_WORDS);
  stack.add("skip_reason_id", record.stack_skip_reason);
  stack.add("skip_reason",
            crash_forensics_capture_skip_reason_name(
                record.stack_skip_reason));
  out.add_object("stack", stack);

  return out;
}

static FLASHMEM Payload system_crash_forensics_payload(void) {
  crash_forensics_status_t status{};
  crash_forensics_get_status(&status);

  Payload out;
  out.add("schema", "ZPNET_CRASH_FORENSICS_V3");
  out.add("installed_now", status.installed);
  out.add("present", status.present);

  out.add("core_present", status.core_present);
  out.add("core_header_valid", status.core_header_valid);
  out.add("core_crc_valid", status.core_crc_valid);
  system_crash_add_hex32(out, "core_stored_crc",
                         status.core_stored_crc);
  system_crash_add_hex32(out, "core_computed_crc",
                         status.core_computed_crc);

  out.add("extended_present", status.extended_present);
  out.add("header_valid", status.header_valid);
  out.add("crc_valid", status.crc_valid);
  system_crash_add_hex32(out, "stored_crc", status.stored_crc);
  system_crash_add_hex32(out, "computed_crc", status.computed_crc);

  const crash_forensics_core_record_t* core =
      crash_forensics_core_record();
  if (core) {
    out.add_object("core",
                   system_crash_core_forensics_payload(*core));
  }

  const crash_forensics_record_t* record = crash_forensics_record();
  if (!record) return out;

  system_crash_add_hex32(out, "magic", record->magic);
  system_crash_add_hex32(out, "magic_inv", record->magic_inv);
  out.add("schema_version", record->schema_version);
  out.add("record_size", record->record_size);
  out.add("capture_sequence", record->capture_sequence);
  system_crash_add_hex32(out, "committed", record->committed);
  system_crash_add_hex32(out, "committed_inv", record->committed_inv);
  out.add("flags", record->flags);
  system_crash_add_hex32(out, "flags_hex", record->flags);
  out.add("exception_number", record->exception_number);
  out.add("exception_name",
          crash_forensics_exception_name(record->exception_number));
  out.add("interrupted_exception_number",
          record->interrupted_exception_number);
  out.add("frame_source",
          crash_forensics_frame_source_name(record->frame_source));
  out.add("frame_address_readable",
          (record->flags & CRASH_FORENSICS_FLAG_FRAME_ADDRESS_READABLE) != 0U);
  out.add("basic_frame_valid",
          (record->flags & CRASH_FORENSICS_FLAG_BASIC_FRAME_VALID) != 0U);
  out.add("extended_fp_frame",
          (record->flags & CRASH_FORENSICS_FLAG_EXTENDED_FP_FRAME) != 0U);
  out.add("fp_frame_captured",
          (record->flags & CRASH_FORENSICS_FLAG_FP_FRAME_CAPTURED) != 0U);
  out.add("stacking_fault",
          (record->flags & CRASH_FORENSICS_FLAG_STACKING_FAULT) != 0U);
  out.add("stack_alignment_word",
          (record->flags & CRASH_FORENSICS_FLAG_STACK_ALIGNMENT_WORD) != 0U);
  out.add("return_to_thread",
          (record->flags & CRASH_FORENSICS_FLAG_RETURN_TO_THREAD) != 0U);
  out.add("interrupted_handler",
          (record->flags & CRASH_FORENSICS_FLAG_INTERRUPTED_HANDLER) != 0U);
  out.add("frame_content_plausible",
          (record->flags & CRASH_FORENSICS_FLAG_FRAME_CONTENT_PLAUSIBLE) != 0U);
  out.add("capture_skipped",
          (record->flags & CRASH_FORENSICS_FLAG_CAPTURE_SKIPPED) != 0U);

  Payload capture_skips;
  capture_skips.add("active_stack_reason_id",
                    record->active_stack_skip_reason);
  capture_skips.add("active_stack_reason",
                    crash_forensics_capture_skip_reason_name(
                        record->active_stack_skip_reason));
  capture_skips.add("other_stack_reason_id",
                    record->other_stack_skip_reason);
  capture_skips.add("other_stack_reason",
                    crash_forensics_capture_skip_reason_name(
                        record->other_stack_skip_reason));
  capture_skips.add("pc_window_reason_id", record->pc_window_skip_reason);
  capture_skips.add("pc_window_reason",
                    crash_forensics_capture_skip_reason_name(
                        record->pc_window_skip_reason));
  capture_skips.add("lr_window_reason_id", record->lr_window_skip_reason);
  capture_skips.add("lr_window_reason",
                    crash_forensics_capture_skip_reason_name(
                        record->lr_window_skip_reason));
  out.add_object("capture_skips", capture_skips);

  Payload frame;
  system_crash_add_hex32(frame, "raw_frame_address", record->raw_frame_address);
  system_crash_add_hex32(frame, "basic_frame_address", record->basic_frame_address);
  system_crash_add_hex32(frame, "exc_return", record->exc_return);
  system_crash_add_hex32(frame, "r0", record->r0);
  system_crash_add_hex32(frame, "r1", record->r1);
  system_crash_add_hex32(frame, "r2", record->r2);
  system_crash_add_hex32(frame, "r3", record->r3);
  system_crash_add_hex32(frame, "r12", record->r12);
  system_crash_add_hex32(frame, "lr", record->stacked_lr);
  system_crash_add_hex32(frame, "pc", record->stacked_pc);
  system_crash_add_hex32(frame, "xpsr", record->stacked_xpsr);
  out.add_object("exception_frame", frame);

  Payload callee_saved;
  system_crash_add_hex32(callee_saved, "r4", record->r4);
  system_crash_add_hex32(callee_saved, "r5", record->r5);
  system_crash_add_hex32(callee_saved, "r6", record->r6);
  system_crash_add_hex32(callee_saved, "r7", record->r7);
  system_crash_add_hex32(callee_saved, "r8", record->r8);
  system_crash_add_hex32(callee_saved, "r9", record->r9);
  system_crash_add_hex32(callee_saved, "r10", record->r10);
  system_crash_add_hex32(callee_saved, "r11", record->r11);
  out.add_object("callee_saved", callee_saved);

  Payload control;
  system_crash_add_hex32(control, "msp", record->original_msp);
  system_crash_add_hex32(control, "psp", record->original_psp);
  system_crash_add_hex32(control, "primask", record->primask);
  system_crash_add_hex32(control, "basepri", record->basepri);
  system_crash_add_hex32(control, "faultmask", record->faultmask);
  system_crash_add_hex32(control, "control", record->control);
  system_crash_add_hex32(control, "dwt_cyccnt", record->dwt_cyccnt);
  control.add("cpu_hz", record->cpu_hz);
  out.add_object("processor_control", control);

  Payload fault;
  system_crash_add_hex32(fault, "cpuid", record->cpuid);
  system_crash_add_hex32(fault, "actlr", record->actlr);
  system_crash_add_hex32(fault, "cfsr", record->cfsr);
  system_crash_add_hex32(fault, "hfsr", record->hfsr);
  system_crash_add_hex32(fault, "dfsr", record->dfsr);
  system_crash_add_hex32(fault, "afsr", record->afsr);
  system_crash_add_hex32(fault, "mmfar", record->mmfar);
  system_crash_add_hex32(fault, "bfar", record->bfar);
  system_crash_add_hex32(fault, "shcsr", record->shcsr);
  system_crash_add_hex32(fault, "icsr", record->icsr);
  system_crash_add_hex32(fault, "vtor", record->vtor);
  system_crash_add_hex32(fault, "aircr", record->aircr);
  system_crash_add_hex32(fault, "scr", record->scr);
  system_crash_add_hex32(fault, "ccr", record->ccr);
  system_crash_add_hex32(fault, "shpr1", record->shpr1);
  system_crash_add_hex32(fault, "shpr2", record->shpr2);
  system_crash_add_hex32(fault, "shpr3", record->shpr3);
  system_crash_add_hex32(fault, "cpacr", record->cpacr);
  system_crash_add_hex32(fault, "demcr", record->demcr);
  system_crash_add_hex32(fault, "dwt_ctrl", record->dwt_ctrl);
  system_crash_add_hex32(fault, "syst_csr", record->syst_csr);
  system_crash_add_hex32(fault, "syst_rvr", record->syst_rvr);
  system_crash_add_hex32(fault, "syst_cvr", record->syst_cvr);
  system_crash_add_hex32(fault, "syst_calib", record->syst_calib);
  out.add_object("fault_registers", fault);

  Payload fp_control;
  system_crash_add_hex32(fp_control, "fpccr", record->fpccr);
  system_crash_add_hex32(fp_control, "fpcar", record->fpcar);
  system_crash_add_hex32(fp_control, "fpdscr", record->fpdscr);
  out.add_object("floating_point_control", fp_control);

  Payload vectors;
  system_crash_add_hex32(vectors, "hardfault", record->vector_hardfault);
  system_crash_add_hex32(vectors, "memmanage", record->vector_memmanage);
  system_crash_add_hex32(vectors, "busfault", record->vector_busfault);
  system_crash_add_hex32(vectors, "usagefault", record->vector_usagefault);
  system_crash_add_hex32(vectors, "zpnet_entry", record->zpnet_fault_entry);
  system_crash_add_hex32(vectors, "teensy_core_handler",
                         record->teensy_core_fault_handler);
  out.add_object("vectors", vectors);

  PayloadArray nvic;
  for (size_t i = 0; i < CRASH_FORENSICS_NVIC_WORDS; ++i) {
    Payload word;
    word.add("index", (uint32_t)i);
    system_crash_add_hex32(word, "iser", record->nvic_iser[i]);
    system_crash_add_hex32(word, "ispr", record->nvic_ispr[i]);
    system_crash_add_hex32(word, "iabr", record->nvic_iabr[i]);
    nvic.add(word);
  }
  out.add_array("nvic", nvic);

  Payload mpu;
  system_crash_add_hex32(mpu, "type", record->mpu_type);
  system_crash_add_hex32(mpu, "control", record->mpu_ctrl);
  mpu.add("selected_region", record->mpu_rnr);
  mpu.add("region_count", record->mpu_region_count);
  PayloadArray mpu_regions;
  const size_t bounded_mpu =
      record->mpu_region_count < CRASH_FORENSICS_MPU_REGIONS
          ? record->mpu_region_count
          : CRASH_FORENSICS_MPU_REGIONS;
  for (size_t i = 0; i < bounded_mpu; ++i) {
    Payload region;
    region.add("index", (uint32_t)i);
    system_crash_add_hex32(region, "rbar", record->mpu_rbar[i]);
    system_crash_add_hex32(region, "rasr", record->mpu_rasr[i]);
    region.add("enabled", (record->mpu_rasr[i] & 1U) != 0U);
    region.add("execute_never", (record->mpu_rasr[i] & (1UL << 28)) != 0U);
    mpu_regions.add(region);
  }
  mpu.add_array("regions", mpu_regions);
  out.add_object("mpu", mpu);

  PayloadArray fp_frame;
  const size_t bounded_fp =
      record->fp_frame_word_count < CRASH_FORENSICS_FP_FRAME_WORDS
          ? record->fp_frame_word_count
          : CRASH_FORENSICS_FP_FRAME_WORDS;
  for (size_t i = 0; i < bounded_fp; ++i) {
    Payload word;
    word.add("index", (uint32_t)i);
    system_crash_add_hex32(word, "value", record->fp_frame_words[i]);
    fp_frame.add(word);
  }
  out.add_array("fp_frame_words", fp_frame);

  out.add_object(
      "active_stack",
      system_crash_word_window_payload(
          record->active_stack_base,
          record->active_stack_word_count,
          record->active_stack_words,
          CRASH_FORENSICS_ACTIVE_STACK_WORDS));
  out.add_object(
      "other_stack",
      system_crash_word_window_payload(
          record->other_stack_base,
          record->other_stack_word_count,
          record->other_stack_words,
          CRASH_FORENSICS_OTHER_STACK_WORDS));
  out.add_object(
      "pc_window",
      system_crash_word_window_payload(
          record->pc_window_base,
          record->pc_window_word_count,
          record->pc_window_words,
          CRASH_FORENSICS_PC_WINDOW_WORDS));
  out.add_object(
      "lr_window",
      system_crash_word_window_payload(
          record->lr_window_base,
          record->lr_window_word_count,
          record->lr_window_words,
          CRASH_FORENSICS_LR_WINDOW_WORDS));

  return out;
}


// ============================================================================
// CRASH_POLICY — live exception/FPU policy plus retained-frame interpretation
// ============================================================================
//
// This report deliberately does not dereference any address from the retained
// crash record.  It compares the recorder's scalar choices against the Armv7-M
// EXC_RETURN contract and reports the live system-control/FPU policy that would
// govern a newly taken exception.
//
// Armv7-M EXC_RETURN fields used here:
//   bit 2: 0 = MSP, 1 = PSP
//   bit 3: 0 = return to Handler mode, 1 = return to Thread mode
//   bit 4: 0 = extended FP frame, 1 = basic frame
//
// An extended FP frame contributes 18 words before the eight-word basic frame.
// The optional stack-alignment word follows the basic frame and is identified
// by stacked xPSR bit 9.

static constexpr uintptr_t SYSTEM_SCS_ACTLR  = 0xE000E008UL;
static constexpr uintptr_t SYSTEM_SCB_CPUID  = 0xE000ED00UL;
static constexpr uintptr_t SYSTEM_SCB_ICSR   = 0xE000ED04UL;
static constexpr uintptr_t SYSTEM_SCB_VTOR   = 0xE000ED08UL;
static constexpr uintptr_t SYSTEM_SCB_AIRCR  = 0xE000ED0CUL;
static constexpr uintptr_t SYSTEM_SCB_SCR    = 0xE000ED10UL;
static constexpr uintptr_t SYSTEM_SCB_CCR    = 0xE000ED14UL;
static constexpr uintptr_t SYSTEM_SCB_SHCSR  = 0xE000ED24UL;
static constexpr uintptr_t SYSTEM_SCB_CFSR   = 0xE000ED28UL;
static constexpr uintptr_t SYSTEM_SCB_HFSR   = 0xE000ED2CUL;
static constexpr uintptr_t SYSTEM_SCB_DFSR   = 0xE000ED30UL;
static constexpr uintptr_t SYSTEM_SCB_MMFAR  = 0xE000ED34UL;
static constexpr uintptr_t SYSTEM_SCB_BFAR   = 0xE000ED38UL;
static constexpr uintptr_t SYSTEM_SCB_AFSR   = 0xE000ED3CUL;
static constexpr uintptr_t SYSTEM_SCB_CPACR  = 0xE000ED88UL;
static constexpr uintptr_t SYSTEM_FPU_FPCCR  = 0xE000EF34UL;
static constexpr uintptr_t SYSTEM_FPU_FPCAR  = 0xE000EF38UL;
static constexpr uintptr_t SYSTEM_FPU_FPDSCR = 0xE000EF3CUL;
static constexpr uintptr_t SYSTEM_FPU_MVFR0  = 0xE000EF40UL;
static constexpr uintptr_t SYSTEM_FPU_MVFR1  = 0xE000EF44UL;
static constexpr uintptr_t SYSTEM_DHCSR      = 0xE000EDF0UL;
static constexpr uintptr_t SYSTEM_DEMCR      = 0xE000EDFCUL;
static constexpr uintptr_t SYSTEM_DWT_CTRL   = 0xE0001000UL;

static constexpr uint32_t SYSTEM_EXC_RETURN_HANDLER_MSP_EXTENDED = 0xFFFFFFE1UL;
static constexpr uint32_t SYSTEM_EXC_RETURN_THREAD_MSP_EXTENDED  = 0xFFFFFFE9UL;
static constexpr uint32_t SYSTEM_EXC_RETURN_THREAD_PSP_EXTENDED  = 0xFFFFFFEDUL;
static constexpr uint32_t SYSTEM_EXC_RETURN_HANDLER_MSP_BASIC    = 0xFFFFFFF1UL;
static constexpr uint32_t SYSTEM_EXC_RETURN_THREAD_MSP_BASIC     = 0xFFFFFFF9UL;
static constexpr uint32_t SYSTEM_EXC_RETURN_THREAD_PSP_BASIC     = 0xFFFFFFFDUL;

static inline uint32_t system_crash_policy_read32(uintptr_t address) {
  return *reinterpret_cast<volatile const uint32_t*>(address);
}

static bool system_crash_policy_exc_return_recognized(uint32_t value) {
  switch (value) {
    case SYSTEM_EXC_RETURN_HANDLER_MSP_EXTENDED:
    case SYSTEM_EXC_RETURN_THREAD_MSP_EXTENDED:
    case SYSTEM_EXC_RETURN_THREAD_PSP_EXTENDED:
    case SYSTEM_EXC_RETURN_HANDLER_MSP_BASIC:
    case SYSTEM_EXC_RETURN_THREAD_MSP_BASIC:
    case SYSTEM_EXC_RETURN_THREAD_PSP_BASIC:
      return true;
    default:
      return false;
  }
}

static const char* system_crash_policy_exc_return_name(uint32_t value) {
  switch (value) {
    case SYSTEM_EXC_RETURN_HANDLER_MSP_EXTENDED:
      return "HANDLER_MSP_EXTENDED";
    case SYSTEM_EXC_RETURN_THREAD_MSP_EXTENDED:
      return "THREAD_MSP_EXTENDED";
    case SYSTEM_EXC_RETURN_THREAD_PSP_EXTENDED:
      return "THREAD_PSP_EXTENDED";
    case SYSTEM_EXC_RETURN_HANDLER_MSP_BASIC:
      return "HANDLER_MSP_BASIC";
    case SYSTEM_EXC_RETURN_THREAD_MSP_BASIC:
      return "THREAD_MSP_BASIC";
    case SYSTEM_EXC_RETURN_THREAD_PSP_BASIC:
      return "THREAD_PSP_BASIC";
    default:
      return "UNRECOGNIZED";
  }
}

static bool system_crash_policy_executable_address(uint32_t value) {
  if (value == 0U) return false;
  const uint32_t address = value & ~1UL;

  // Teensy 4.1 executable regions used by this firmware:
  //   ITCM:       0x00000400 .. 0x0007FFFF
  //   FlexSPI2:   0x60000000 .. 0x60FFFFFF
  if (address >= 0x00000400UL && address < 0x00080000UL) return true;
  return address >= 0x60000000UL && address < 0x61000000UL;
}

static bool system_crash_policy_dtcm_address(uint32_t value) {
  return value >= 0x20000000UL && value < 0x20080000UL;
}

static const char* system_crash_policy_assessment(
    bool exc_return_recognized,
    bool hardware_stacking_fault,
    bool selected_sp_matches,
    bool basic_address_matches,
    bool interrupted_sp_matches_or_unavailable,
    bool xpsr_thumb,
    bool xpsr_mode_consistent,
    bool pc_executable,
    bool lr_plausible) {
  if (!exc_return_recognized) return "EXC_RETURN_UNRECOGNIZED";
  if (hardware_stacking_fault) return "HARDWARE_STACKING_FAULT_REPORTED";
  if (!selected_sp_matches) return "SELECTED_STACK_POINTER_MISMATCH";
  if (!basic_address_matches) return "BASIC_FRAME_ADDRESS_MISMATCH";
  if (!interrupted_sp_matches_or_unavailable) {
    return "INTERRUPTED_STACK_POINTER_MISMATCH";
  }
  if (!xpsr_thumb) return "STACKED_XPSR_THUMB_BIT_CLEAR";
  if (!xpsr_mode_consistent) return "STACKED_XPSR_MODE_INCONSISTENT";
  if (!pc_executable) return "STACKED_PC_NOT_EXECUTABLE";
  if (!lr_plausible) return "STACKED_LR_NOT_PLAUSIBLE";
  return "FRAME_POLICY_COHERENT";
}

struct system_crash_policy_record_view_t {
  const char* source;
  uint32_t flags;
  uint32_t raw_frame_address;
  uint32_t basic_frame_address;
  uint32_t interrupted_sp;
  bool has_interrupted_sp;
  uint32_t exc_return;
  uint32_t stacked_lr;
  uint32_t stacked_pc;
  uint32_t stacked_xpsr;
  uint32_t original_msp;
  uint32_t original_psp;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t shcsr;
  uint32_t icsr;
  uint32_t fpccr;
  uint32_t fpcar;
  uint32_t ccr;
  uint32_t cpacr;
  uint32_t fpdscr;
  bool has_extended_control;
};

static system_crash_policy_record_view_t
system_crash_policy_core_view(const crash_forensics_core_record_t& record) {
  system_crash_policy_record_view_t view{};
  view.source = "CORE";
  view.flags = record.flags;
  view.raw_frame_address = record.raw_frame_address;
  view.basic_frame_address = record.basic_frame_address;
  view.interrupted_sp = record.interrupted_sp;
  view.has_interrupted_sp = true;
  view.exc_return = record.exc_return;
  view.stacked_lr = record.stacked_lr;
  view.stacked_pc = record.stacked_pc;
  view.stacked_xpsr = record.stacked_xpsr;
  view.original_msp = record.original_msp;
  view.original_psp = record.original_psp;
  view.cfsr = record.cfsr;
  view.hfsr = record.hfsr;
  view.shcsr = record.shcsr;
  view.icsr = record.icsr;
  view.fpccr = record.fpccr;
  view.fpcar = record.fpcar;
  view.has_extended_control = false;
  return view;
}

static system_crash_policy_record_view_t
system_crash_policy_extended_view(const crash_forensics_record_t& record) {
  system_crash_policy_record_view_t view{};
  view.source = "EXTENDED";
  view.flags = record.flags;
  view.raw_frame_address = record.raw_frame_address;
  view.basic_frame_address = record.basic_frame_address;
  view.interrupted_sp = 0U;
  view.has_interrupted_sp = false;
  view.exc_return = record.exc_return;
  view.stacked_lr = record.stacked_lr;
  view.stacked_pc = record.stacked_pc;
  view.stacked_xpsr = record.stacked_xpsr;
  view.original_msp = record.original_msp;
  view.original_psp = record.original_psp;
  view.cfsr = record.cfsr;
  view.hfsr = record.hfsr;
  view.shcsr = record.shcsr;
  view.icsr = record.icsr;
  view.fpccr = record.fpccr;
  view.fpcar = record.fpcar;
  view.ccr = record.ccr;
  view.cpacr = record.cpacr;
  view.fpdscr = record.fpdscr;
  view.has_extended_control = true;
  return view;
}

static FLASHMEM Payload system_crash_policy_record_payload(
    const system_crash_policy_record_view_t& record) {
  const bool exc_return_recognized =
      system_crash_policy_exc_return_recognized(record.exc_return);
  const bool use_psp = (record.exc_return & (1UL << 2)) != 0U;
  const bool return_to_thread = (record.exc_return & (1UL << 3)) != 0U;
  const bool basic_frame = (record.exc_return & (1UL << 4)) != 0U;
  const bool extended_frame = !basic_frame;
  const uint32_t extension_words = extended_frame ? 18U : 0U;
  const uint32_t selected_sp = use_psp ? record.original_psp : record.original_msp;
  const uint32_t expected_basic =
      record.raw_frame_address + extension_words * sizeof(uint32_t);
  const bool stack_alignment_word =
      (record.stacked_xpsr & (1UL << 9)) != 0U;
  const uint32_t expected_interrupted_sp =
      expected_basic + 8U * sizeof(uint32_t) +
      (stack_alignment_word ? sizeof(uint32_t) : 0U);

  const uint32_t xpsr_exception_number = record.stacked_xpsr & 0x1FFU;
  const bool xpsr_thumb = (record.stacked_xpsr & (1UL << 24)) != 0U;
  const bool xpsr_mode_consistent =
      return_to_thread ? (xpsr_exception_number == 0U)
                       : (xpsr_exception_number != 0U);
  const bool pc_executable =
      system_crash_policy_executable_address(record.stacked_pc);
  const bool lr_executable =
      system_crash_policy_executable_address(record.stacked_lr);
  const bool lr_exc_return =
      system_crash_policy_exc_return_recognized(record.stacked_lr);
  const bool lr_plausible = lr_executable || lr_exc_return;

  const bool memory_stacking_fault =
      (record.cfsr & ((1UL << 3) | (1UL << 4) | (1UL << 5))) != 0U;
  const bool bus_stacking_fault =
      (record.cfsr & ((1UL << 11) | (1UL << 12) | (1UL << 13))) != 0U;
  const bool hardware_stacking_fault =
      memory_stacking_fault || bus_stacking_fault;

  const bool selected_sp_matches =
      selected_sp == record.raw_frame_address;
  const bool basic_address_matches =
      expected_basic == record.basic_frame_address;
  const bool interrupted_sp_matches =
      !record.has_interrupted_sp ||
      expected_interrupted_sp == record.interrupted_sp;
  const bool extended_flag =
      (record.flags & CRASH_FORENSICS_FLAG_EXTENDED_FP_FRAME) != 0U;
  const bool format_flag_matches =
      extended_flag == extended_frame;

  const bool frame_content_plausible =
      exc_return_recognized &&
      !hardware_stacking_fault &&
      selected_sp_matches &&
      basic_address_matches &&
      interrupted_sp_matches &&
      xpsr_thumb &&
      xpsr_mode_consistent &&
      pc_executable &&
      lr_plausible;

  Payload out;
  out.add("source", record.source);
  system_crash_add_hex32(out, "flags", record.flags);
  system_crash_add_hex32(out, "exc_return", record.exc_return);
  out.add("exc_return_name",
          system_crash_policy_exc_return_name(record.exc_return));
  out.add("exc_return_recognized", exc_return_recognized);
  out.add("return_stack", use_psp ? "PSP" : "MSP");
  out.add("return_mode", return_to_thread ? "THREAD" : "HANDLER");
  out.add("frame_format", basic_frame ? "BASIC" : "EXTENDED_FP");
  out.add("frame_extension_words", extension_words);
  out.add("frame_extension_bytes",
          extension_words * (uint32_t)sizeof(uint32_t));

  system_crash_add_hex32(out, "raw_frame_address",
                         record.raw_frame_address);
  system_crash_add_hex32(out, "selected_stack_pointer", selected_sp);
  out.add("selected_stack_pointer_matches_raw", selected_sp_matches);
  system_crash_add_hex32(out, "recorded_basic_frame_address",
                         record.basic_frame_address);
  system_crash_add_hex32(out, "expected_basic_frame_address",
                         expected_basic);
  out.add("basic_frame_address_matches_policy", basic_address_matches);

  out.add("record_has_interrupted_sp", record.has_interrupted_sp);
  if (record.has_interrupted_sp) {
    system_crash_add_hex32(out, "recorded_interrupted_sp",
                           record.interrupted_sp);
  }
  system_crash_add_hex32(out, "expected_interrupted_sp",
                         expected_interrupted_sp);
  out.add("stack_alignment_word", stack_alignment_word);
  out.add("interrupted_sp_matches_policy", interrupted_sp_matches);

  Payload stacked;
  system_crash_add_hex32(stacked, "pc", record.stacked_pc);
  stacked.add("pc_executable", pc_executable);
  system_crash_add_hex32(stacked, "lr", record.stacked_lr);
  stacked.add("lr_executable", lr_executable);
  stacked.add("lr_is_exc_return", lr_exc_return);
  stacked.add("lr_plausible", lr_plausible);
  system_crash_add_hex32(stacked, "xpsr", record.stacked_xpsr);
  stacked.add("xpsr_thumb", xpsr_thumb);
  stacked.add("xpsr_exception_number", xpsr_exception_number);
  stacked.add("xpsr_mode_consistent", xpsr_mode_consistent);
  out.add_object("stacked_state", stacked);

  Payload fault;
  system_crash_add_hex32(fault, "cfsr", record.cfsr);
  system_crash_add_hex32(fault, "hfsr", record.hfsr);
  system_crash_add_hex32(fault, "shcsr", record.shcsr);
  system_crash_add_hex32(fault, "icsr", record.icsr);
  fault.add("iaccviol", (record.cfsr & (1UL << 0)) != 0U);
  fault.add("daccviol", (record.cfsr & (1UL << 1)) != 0U);
  fault.add("munstkerr", (record.cfsr & (1UL << 3)) != 0U);
  fault.add("mstkerr", (record.cfsr & (1UL << 4)) != 0U);
  fault.add("mlsperr", (record.cfsr & (1UL << 5)) != 0U);
  fault.add("mmarvalid", (record.cfsr & (1UL << 7)) != 0U);
  fault.add("ibuserr", (record.cfsr & (1UL << 8)) != 0U);
  fault.add("preciserr", (record.cfsr & (1UL << 9)) != 0U);
  fault.add("impreciserr", (record.cfsr & (1UL << 10)) != 0U);
  fault.add("unstkerr", (record.cfsr & (1UL << 11)) != 0U);
  fault.add("stkerr", (record.cfsr & (1UL << 12)) != 0U);
  fault.add("lsperr", (record.cfsr & (1UL << 13)) != 0U);
  fault.add("bfarvalid", (record.cfsr & (1UL << 15)) != 0U);
  fault.add("undefinstr", (record.cfsr & (1UL << 16)) != 0U);
  fault.add("invstate", (record.cfsr & (1UL << 17)) != 0U);
  fault.add("invpc", (record.cfsr & (1UL << 18)) != 0U);
  fault.add("nocp", (record.cfsr & (1UL << 19)) != 0U);
  fault.add("unaligned", (record.cfsr & (1UL << 24)) != 0U);
  fault.add("divbyzero", (record.cfsr & (1UL << 25)) != 0U);
  fault.add("hardware_stacking_fault", hardware_stacking_fault);
  out.add_object("fault_decode", fault);

  Payload fp;
  system_crash_add_hex32(fp, "fpccr", record.fpccr);
  system_crash_add_hex32(fp, "fpcar", record.fpcar);
  fp.add("fpcar_in_dtcm", system_crash_policy_dtcm_address(record.fpcar));
  fp.add("fpcar_aligned_8", (record.fpcar & 7U) == 0U);
  fp.add("fpcar_minus_raw_frame",
         (int32_t)(record.fpcar - record.raw_frame_address));
  fp.add("aspen", (record.fpccr & (1UL << 31)) != 0U);
  fp.add("lspen", (record.fpccr & (1UL << 30)) != 0U);
  fp.add("monrdy", (record.fpccr & (1UL << 8)) != 0U);
  fp.add("bfrdy", (record.fpccr & (1UL << 6)) != 0U);
  fp.add("mmrdy", (record.fpccr & (1UL << 5)) != 0U);
  fp.add("hfrdy", (record.fpccr & (1UL << 4)) != 0U);
  fp.add("thread", (record.fpccr & (1UL << 3)) != 0U);
  fp.add("user", (record.fpccr & (1UL << 1)) != 0U);
  fp.add("lspact", (record.fpccr & (1UL << 0)) != 0U);
  fp.add("automatic_eager_policy",
         (record.fpccr & (1UL << 31)) != 0U &&
         (record.fpccr & (1UL << 30)) == 0U);
  fp.add("record_extended_flag", extended_flag);
  fp.add("record_format_flag_matches_exc_return", format_flag_matches);
  if (record.has_extended_control) {
    system_crash_add_hex32(fp, "fpdscr", record.fpdscr);
    system_crash_add_hex32(fp, "cpacr", record.cpacr);
    system_crash_add_hex32(fp, "ccr", record.ccr);
  }
  out.add_object("floating_point", fp);

  out.add("record_basic_frame_valid_flag",
          (record.flags & CRASH_FORENSICS_FLAG_BASIC_FRAME_VALID) != 0U);
  out.add("record_frame_content_plausible_flag",
          (record.flags & CRASH_FORENSICS_FLAG_FRAME_CONTENT_PLAUSIBLE) != 0U);
  out.add("record_stacking_fault_flag",
          (record.flags & CRASH_FORENSICS_FLAG_STACKING_FAULT) != 0U);
  out.add("computed_frame_content_plausible", frame_content_plausible);
  out.add("assessment_is_inference", true);
  out.add("assessment",
          system_crash_policy_assessment(
              exc_return_recognized,
              hardware_stacking_fault,
              selected_sp_matches,
              basic_address_matches,
              interrupted_sp_matches,
              xpsr_thumb,
              xpsr_mode_consistent,
              pc_executable,
              lr_plausible));
  return out;
}

struct system_crash_policy_live_special_t {
  uint32_t msp;
  uint32_t psp;
  uint32_t sp;
  uint32_t lr;
  uint32_t control;
  uint32_t primask;
  uint32_t basepri;
  uint32_t faultmask;
  uint32_t ipsr;
  uint32_t xpsr;
};

static inline system_crash_policy_live_special_t
system_crash_policy_live_special_snapshot(void) {
  system_crash_policy_live_special_t out{};
#if defined(__arm__)
  __asm__ volatile("mrs %0, msp" : "=r"(out.msp));
  __asm__ volatile("mrs %0, psp" : "=r"(out.psp));
  __asm__ volatile("mov %0, sp" : "=r"(out.sp));
  __asm__ volatile("mov %0, lr" : "=r"(out.lr));
  __asm__ volatile("mrs %0, control" : "=r"(out.control));
  __asm__ volatile("mrs %0, primask" : "=r"(out.primask));
  __asm__ volatile("mrs %0, basepri" : "=r"(out.basepri));
  __asm__ volatile("mrs %0, faultmask" : "=r"(out.faultmask));
  __asm__ volatile("mrs %0, ipsr" : "=r"(out.ipsr));
  __asm__ volatile("mrs %0, xpsr" : "=r"(out.xpsr));
#endif
  return out;
}

static FLASHMEM Payload system_crash_policy_live_payload(void) {
  const system_crash_policy_live_special_t special =
      system_crash_policy_live_special_snapshot();

  const uint32_t cpuid = system_crash_policy_read32(SYSTEM_SCB_CPUID);
  const uint32_t actlr = system_crash_policy_read32(SYSTEM_SCS_ACTLR);
  const uint32_t icsr = system_crash_policy_read32(SYSTEM_SCB_ICSR);
  const uint32_t vtor = system_crash_policy_read32(SYSTEM_SCB_VTOR);
  const uint32_t aircr = system_crash_policy_read32(SYSTEM_SCB_AIRCR);
  const uint32_t scr = system_crash_policy_read32(SYSTEM_SCB_SCR);
  const uint32_t ccr = system_crash_policy_read32(SYSTEM_SCB_CCR);
  const uint32_t shcsr = system_crash_policy_read32(SYSTEM_SCB_SHCSR);
  const uint32_t cfsr = system_crash_policy_read32(SYSTEM_SCB_CFSR);
  const uint32_t hfsr = system_crash_policy_read32(SYSTEM_SCB_HFSR);
  const uint32_t dfsr = system_crash_policy_read32(SYSTEM_SCB_DFSR);
  const uint32_t mmfar = system_crash_policy_read32(SYSTEM_SCB_MMFAR);
  const uint32_t bfar = system_crash_policy_read32(SYSTEM_SCB_BFAR);
  const uint32_t afsr = system_crash_policy_read32(SYSTEM_SCB_AFSR);
  const uint32_t cpacr = system_crash_policy_read32(SYSTEM_SCB_CPACR);
  const uint32_t fpccr = system_crash_policy_read32(SYSTEM_FPU_FPCCR);
  const uint32_t fpcar = system_crash_policy_read32(SYSTEM_FPU_FPCAR);
  const uint32_t fpdscr = system_crash_policy_read32(SYSTEM_FPU_FPDSCR);
  const uint32_t mvfr0 = system_crash_policy_read32(SYSTEM_FPU_MVFR0);
  const uint32_t mvfr1 = system_crash_policy_read32(SYSTEM_FPU_MVFR1);
  const uint32_t dhcsr = system_crash_policy_read32(SYSTEM_DHCSR);
  const uint32_t demcr = system_crash_policy_read32(SYSTEM_DEMCR);
  const uint32_t dwt_ctrl = system_crash_policy_read32(SYSTEM_DWT_CTRL);

  Payload out;

  Payload processor;
  system_crash_add_hex32(processor, "msp", special.msp);
  system_crash_add_hex32(processor, "psp", special.psp);
  system_crash_add_hex32(processor, "sp", special.sp);
  system_crash_add_hex32(processor, "lr", special.lr);
  system_crash_add_hex32(processor, "control", special.control);
  system_crash_add_hex32(processor, "primask", special.primask);
  system_crash_add_hex32(processor, "basepri", special.basepri);
  system_crash_add_hex32(processor, "faultmask", special.faultmask);
  system_crash_add_hex32(processor, "ipsr", special.ipsr);
  system_crash_add_hex32(processor, "xpsr", special.xpsr);
  processor.add("exception_number", special.ipsr & 0x1FFU);
  processor.add("thread_mode", (special.ipsr & 0x1FFU) == 0U);
  processor.add("using_psp", (special.control & (1UL << 1)) != 0U);
  processor.add("unprivileged", (special.control & 1UL) != 0U);
  processor.add("fp_context_active", (special.control & (1UL << 2)) != 0U);
  out.add_object("processor", processor);

  Payload system_control;
  system_crash_add_hex32(system_control, "cpuid", cpuid);
  system_crash_add_hex32(system_control, "actlr", actlr);
  system_crash_add_hex32(system_control, "icsr", icsr);
  system_crash_add_hex32(system_control, "vtor", vtor);
  system_crash_add_hex32(system_control, "aircr", aircr);
  system_crash_add_hex32(system_control, "scr", scr);
  system_crash_add_hex32(system_control, "ccr", ccr);
  system_crash_add_hex32(system_control, "shcsr", shcsr);
  system_crash_add_hex32(system_control, "cfsr", cfsr);
  system_crash_add_hex32(system_control, "hfsr", hfsr);
  system_crash_add_hex32(system_control, "dfsr", dfsr);
  system_crash_add_hex32(system_control, "mmfar", mmfar);
  system_crash_add_hex32(system_control, "bfar", bfar);
  system_crash_add_hex32(system_control, "afsr", afsr);
  system_crash_add_hex32(system_control, "cpacr", cpacr);
  system_control.add("icsr_vectactive", icsr & 0x1FFU);
  system_control.add("icsr_rettobase", (icsr & (1UL << 11)) != 0U);
  system_control.add("icsr_vectpending", (icsr >> 12) & 0x1FFU);
  system_control.add("icsr_isrpending", (icsr & (1UL << 22)) != 0U);
  system_control.add("ccr_unalign_trp", (ccr & (1UL << 3)) != 0U);
  system_control.add("ccr_div_0_trp", (ccr & (1UL << 4)) != 0U);
  system_control.add("ccr_stkalign", (ccr & (1UL << 9)) != 0U);
  system_control.add("memfault_enabled", (shcsr & (1UL << 16)) != 0U);
  system_control.add("busfault_enabled", (shcsr & (1UL << 17)) != 0U);
  system_control.add("usagefault_enabled", (shcsr & (1UL << 18)) != 0U);
  system_control.add("cp10_access", (cpacr >> 20) & 3U);
  system_control.add("cp11_access", (cpacr >> 22) & 3U);
  system_control.add("fpu_full_access", (cpacr & 0x00F00000UL) == 0x00F00000UL);
  out.add_object("system_control", system_control);

  Payload fp;
  system_crash_add_hex32(fp, "fpccr", fpccr);
  system_crash_add_hex32(fp, "fpcar", fpcar);
  system_crash_add_hex32(fp, "fpdscr", fpdscr);
  system_crash_add_hex32(fp, "mvfr0", mvfr0);
  system_crash_add_hex32(fp, "mvfr1", mvfr1);
  fp.add("aspen", (fpccr & (1UL << 31)) != 0U);
  fp.add("lspen", (fpccr & (1UL << 30)) != 0U);
  fp.add("monrdy", (fpccr & (1UL << 8)) != 0U);
  fp.add("bfrdy", (fpccr & (1UL << 6)) != 0U);
  fp.add("mmrdy", (fpccr & (1UL << 5)) != 0U);
  fp.add("hfrdy", (fpccr & (1UL << 4)) != 0U);
  fp.add("thread", (fpccr & (1UL << 3)) != 0U);
  fp.add("user", (fpccr & (1UL << 1)) != 0U);
  fp.add("lspact", (fpccr & (1UL << 0)) != 0U);
  fp.add("fpcar_in_dtcm", system_crash_policy_dtcm_address(fpcar));
  fp.add("fpcar_aligned_8", (fpcar & 7U) == 0U);
  fp.add("expected_context_mode", "AUTOMATIC_EAGER");
  fp.add("automatic_eager_policy",
         (fpccr & (1UL << 31)) != 0U &&
         (fpccr & (1UL << 30)) == 0U);
  fp.add("context_policy_ok",
         (fpccr & (1UL << 31)) != 0U &&
         (fpccr & (1UL << 30)) == 0U &&
         (cpacr & 0x00F00000UL) == 0x00F00000UL);
  out.add_object("floating_point", fp);

  Payload debug;
  system_crash_add_hex32(debug, "dhcsr", dhcsr);
  system_crash_add_hex32(debug, "demcr", demcr);
  system_crash_add_hex32(debug, "dwt_ctrl", dwt_ctrl);
  debug.add("debug_enabled", (dhcsr & (1UL << 0)) != 0U);
  debug.add("core_halted", (dhcsr & (1UL << 17)) != 0U);
  debug.add("register_transfer_ready", (dhcsr & (1UL << 16)) != 0U);
  debug.add("reset_seen", (dhcsr & (1UL << 25)) != 0U);
  debug.add("trace_enabled", (demcr & (1UL << 24)) != 0U);
  debug.add("dwt_cycle_counter_enabled", (dwt_ctrl & 1UL) != 0U);
  debug.add("stack_watch_debugger_conflict",
            (dhcsr & (1UL << 0)) != 0U);
  out.add_object("debug", debug);

  out.add("fault_status_clear_now", cfsr == 0U && hfsr == 0U);
  return out;
}

static FLASHMEM Payload system_crash_policy_payload(void) {
  crash_forensics_status_t status{};
  crash_forensics_get_status(&status);

  Payload out;
  out.add("schema", "ZPNET_CRASH_POLICY_V1");
  out.add("architecture", "ARMV7M_CORTEX_M7");
  out.add("frame_source_rule", "EXC_RETURN[2]: 0=MSP 1=PSP");
  out.add("return_mode_rule", "EXC_RETURN[3]: 0=HANDLER 1=THREAD");
  out.add("frame_format_rule", "EXC_RETURN[4]: 0=EXTENDED_FP 1=BASIC");
  out.add("extended_frame_words", 18U);
  out.add("basic_frame_words", 8U);
  out.add_object("live", system_crash_policy_live_payload());

  Payload retained;
  retained.add("core_present", status.core_present);
  retained.add("core_header_valid", status.core_header_valid);
  retained.add("core_crc_valid", status.core_crc_valid);
  retained.add("extended_present", status.extended_present);
  retained.add("extended_header_valid", status.header_valid);
  retained.add("extended_crc_valid", status.crc_valid);

  const crash_forensics_core_record_t* core =
      crash_forensics_core_record();
  if (core) {
    retained.add_object(
        "core",
        system_crash_policy_record_payload(
            system_crash_policy_core_view(*core)));
  }

  const crash_forensics_record_t* extended =
      crash_forensics_record();
  if (extended) {
    retained.add_object(
        "extended",
        system_crash_policy_record_payload(
            system_crash_policy_extended_view(*extended)));
  }

  out.add_object("retained", retained);
  return out;
}

static constexpr size_t SYSTEM_CRASH_REPORT_TEXT_MAX = 2048;

// ================================================================
// Runtime-ledger reporting surface
// ================================================================

static const char* runtime_foreground_phase_name(uint32_t phase) {
  switch ((zpnet_foreground_phase_t)phase) {
    case zpnet_foreground_phase_t::TRANSPORT_PRE:    return "TRANSPORT_PRE";
    case zpnet_foreground_phase_t::TIMEPOP_DISPATCH: return "TIMEPOP_DISPATCH";
    case zpnet_foreground_phase_t::TRANSPORT_POST:   return "TRANSPORT_POST";
    default:                                         return "NONE";
  }
}

static FLASHMEM Payload system_runtime_ledger_bank_payload(
    const zpnet_runtime_ledger_t* ledger) {
  Payload out;
  const bool header_valid = runtime_ledger_header_valid(ledger);
  const bool foreground_valid = runtime_ledger_foreground_valid(ledger);

  out.add("header_valid", header_valid);
  out.add("foreground_valid", foreground_valid);
  if (!header_valid) return out;

  out.add("foreground_sequence", ledger->foreground_sequence);
  out.add("foreground_phase_id", ledger->foreground_phase);
  out.add("foreground_phase",
          runtime_foreground_phase_name(ledger->foreground_phase));
  return out;
}

static FLASHMEM Payload system_runtime_ledger_payload(void) {
  runtime_ledger_boot_latch();

  Payload out;
  out.add("schema", "ZPNET_RUNTIME_LEDGER_V2");
  out.add_object("live",
                 system_runtime_ledger_bank_payload(&g_runtime_ledger));
  out.add_object("retained",
                 system_runtime_ledger_bank_payload(&g_runtime_ledger_retained));
  return out;
}



static constexpr uint32_t SYSTEM_TRACE_REPORT_DEFAULT_COUNT = 8U;
static constexpr uint32_t SYSTEM_TRACE_REPORT_MAX_COUNT = 8U;



static uint32_t system_trace_report_count(const Payload& args) {
  uint32_t count = args.has("count")
      ? args.getUInt("count")
      : SYSTEM_TRACE_REPORT_DEFAULT_COUNT;
  if (count == 0U) count = 1U;
  if (count > SYSTEM_TRACE_REPORT_MAX_COUNT) {
    count = SYSTEM_TRACE_REPORT_MAX_COUNT;
  }
  return count;
}

static uint32_t system_trace_report_offset(const Payload& args) {
  return args.has("offset") ? args.getUInt("offset") : 0U;
}

static bool system_trace_report_live_bank(const Payload& args,
                                          bool* valid) {
  if (valid) *valid = true;
  const char* bank = args.getString("bank");
  if (!bank || !*bank || system_cstr_equal_ci(bank, "retained")) {
    return false;
  }
  if (system_cstr_equal_ci(bank, "live")) {
    return true;
  }
  if (valid) *valid = false;
  return false;
}

static void system_trace_bounds(uint32_t available,
                                uint32_t requested,
                                uint32_t offset_from_newest,
                                uint32_t* out_begin,
                                uint32_t* out_end) {
  uint32_t end = 0U;
  if (offset_from_newest < available) {
    end = available - offset_from_newest;
  }
  const uint32_t begin = end > requested ? end - requested : 0U;
  if (out_begin) *out_begin = begin;
  if (out_end) *out_end = end;
}

// ============================================================================
// Retained TimePop dispatch flight recorder — reporting surface
// ============================================================================
//
// The snapshot buffer lives in RAM2 so a report about main-stack control-flow
// damage does not consume several kilobytes of that same stack.  The recorder
// itself stores only scalar addresses; reporting classifies them but never
// dereferences callback, user-data, or name pointers.

static timepop_dispatch_trace_snapshot_t
    g_system_timepop_dispatch_trace_scratch DMAMEM;

static const char* timepop_dispatch_trace_stage_name(uint32_t stage) {
  switch ((timepop_dispatch_trace_stage_t)stage) {
    case timepop_dispatch_trace_stage_t::DISPATCH_ENTER:
      return "DISPATCH_ENTER";
    case timepop_dispatch_trace_stage_t::PHASE_ASAP:
      return "PHASE_ASAP";
    case timepop_dispatch_trace_stage_t::DEFERRED_SELECTED:
      return "DEFERRED_SELECTED";
    case timepop_dispatch_trace_stage_t::CALLBACK_ENTER:
      return "CALLBACK_ENTER";
    case timepop_dispatch_trace_stage_t::CALLBACK_RETURN:
      return "CALLBACK_RETURN";
    case timepop_dispatch_trace_stage_t::DEFERRED_CLEANUP:
      return "DEFERRED_CLEANUP";
    case timepop_dispatch_trace_stage_t::MUTATION_BARRIER_ENTER:
      return "MUTATION_BARRIER_ENTER";
    case timepop_dispatch_trace_stage_t::MUTATION_SELECTED:
      return "MUTATION_SELECTED";
    case timepop_dispatch_trace_stage_t::MUTATION_RESULT:
      return "MUTATION_RESULT";
    case timepop_dispatch_trace_stage_t::MUTATION_BARRIER_EXIT:
      return "MUTATION_BARRIER_EXIT";
    case timepop_dispatch_trace_stage_t::PHASE_TIMED:
      return "PHASE_TIMED";
    case timepop_dispatch_trace_stage_t::TIMED_SELECTED:
      return "TIMED_SELECTED";
    case timepop_dispatch_trace_stage_t::TIMED_SLOT_AFTER_CALLBACK:
      return "TIMED_SLOT_AFTER_CALLBACK";
    case timepop_dispatch_trace_stage_t::REARM_BEGIN:
      return "REARM_BEGIN";
    case timepop_dispatch_trace_stage_t::REARM_END:
      return "REARM_END";
    case timepop_dispatch_trace_stage_t::SLOT_RETIRED:
      return "SLOT_RETIRED";
    case timepop_dispatch_trace_stage_t::SLOT_REPLACED:
      return "SLOT_REPLACED";
    case timepop_dispatch_trace_stage_t::PHASE_ALAP:
      return "PHASE_ALAP";
    case timepop_dispatch_trace_stage_t::DISPATCH_LEAVE:
      return "DISPATCH_LEAVE";
    case timepop_dispatch_trace_stage_t::IRQ_SELECTED:
      return "IRQ_SELECTED";
    case timepop_dispatch_trace_stage_t::ISR_ENTER:
      return "ISR_ENTER";
    case timepop_dispatch_trace_stage_t::ISR_CAPTURED:
      return "ISR_CAPTURED";
    case timepop_dispatch_trace_stage_t::ISR_EXIT:
      return "ISR_EXIT";
    case timepop_dispatch_trace_stage_t::HANDOFF_ENTER:
      return "HANDOFF_ENTER";
    case timepop_dispatch_trace_stage_t::HANDOFF_DEQUEUE:
      return "HANDOFF_DEQUEUE";
    case timepop_dispatch_trace_stage_t::HANDOFF_EXIT:
      return "HANDOFF_EXIT";
    case timepop_dispatch_trace_stage_t::SUBSCRIBER_SELECTED:
      return "SUBSCRIBER_SELECTED";
    case timepop_dispatch_trace_stage_t::SUBSCRIBER_ENTER:
      return "SUBSCRIBER_ENTER";
    case timepop_dispatch_trace_stage_t::SUBSCRIBER_RETURN:
      return "SUBSCRIBER_RETURN";
    default:
      return "NONE";
  }
}

static const char* timepop_dispatch_trace_kind_name(uint32_t kind) {
  switch ((timepop_dispatch_trace_kind_t)kind) {
    case timepop_dispatch_trace_kind_t::DISPATCH:
      return "DISPATCH";
    case timepop_dispatch_trace_kind_t::ASAP:
      return "ASAP";
    case timepop_dispatch_trace_kind_t::TIMED:
      return "TIMED";
    case timepop_dispatch_trace_kind_t::ALAP:
      return "ALAP";
    case timepop_dispatch_trace_kind_t::ISR_TIMED:
      return "ISR_TIMED";
    case timepop_dispatch_trace_kind_t::MUTATION:
      return "MUTATION";
    case timepop_dispatch_trace_kind_t::REARM:
      return "REARM";
    case timepop_dispatch_trace_kind_t::ISR_QTIMER1:
      return "ISR_QTIMER1";
    case timepop_dispatch_trace_kind_t::ISR_VCLOCK:
      return "ISR_VCLOCK";
    case timepop_dispatch_trace_kind_t::ISR_OCXO1:
      return "ISR_OCXO1";
    case timepop_dispatch_trace_kind_t::ISR_OCXO2:
      return "ISR_OCXO2";
    case timepop_dispatch_trace_kind_t::ISR_PPS:
      return "ISR_PPS";
    case timepop_dispatch_trace_kind_t::INTERRUPT_HANDOFF:
      return "INTERRUPT_HANDOFF";
    case timepop_dispatch_trace_kind_t::SUBSCRIBER_VCLOCK:
      return "SUBSCRIBER_VCLOCK";
    case timepop_dispatch_trace_kind_t::SUBSCRIBER_OCXO1:
      return "SUBSCRIBER_OCXO1";
    case timepop_dispatch_trace_kind_t::SUBSCRIBER_OCXO2:
      return "SUBSCRIBER_OCXO2";
    default:
      return "NONE";
  }
}

static const char* timepop_dispatch_trace_phase_name(uint32_t phase) {
  switch ((timepop_dispatch_trace_phase_t)phase) {
    case timepop_dispatch_trace_phase_t::ASAP:
      return "ASAP";
    case timepop_dispatch_trace_phase_t::TIMED:
      return "TIMED";
    case timepop_dispatch_trace_phase_t::ALAP:
      return "ALAP";
    case timepop_dispatch_trace_phase_t::APPLYING_MUTATIONS:
      return "APPLYING_MUTATIONS";
    default:
      return "IDLE";
  }
}

static bool timepop_dispatch_trace_executable_address(uint32_t value) {
  if (value == 0U) return false;
  const uint32_t address = value & ~1U;
  if (address >= 0x00000400UL && address < 0x00080000UL) return true;
  return address >= 0x60000000UL && address < 0x61000000UL;
}

static const char* timepop_dispatch_trace_retire_reason(uint32_t reason) {
  switch (reason) {
    case 1U: return "CALLBACK_OR_MUTATION_RETIRED_SLOT";
    case 2U: return "ABSOLUTE_GRID_NEXT_TARGET_FAILED";
    case 3U: return "GNSS_TO_VCLOCK_DEADLINE_FAILED";
    case 4U: return "RECURRING_PERIOD_INVALID";
    case 5U: return "ONE_SHOT_COMPLETE";
    default: return "UNKNOWN";
  }
}

static const char* timepop_dispatch_trace_aux_meaning(uint32_t stage,
                                                       uint32_t kind) {
  const timepop_dispatch_trace_stage_t stage_id =
      (timepop_dispatch_trace_stage_t)stage;
  const timepop_dispatch_trace_kind_t kind_id =
      (timepop_dispatch_trace_kind_t)kind;

  switch (stage_id) {
    case timepop_dispatch_trace_stage_t::DISPATCH_ENTER:
      return "dispatch_call_count";
    case timepop_dispatch_trace_stage_t::PHASE_ASAP:
    case timepop_dispatch_trace_stage_t::PHASE_ALAP:
      return "pending_deferred_count";
    case timepop_dispatch_trace_stage_t::PHASE_TIMED:
      return "expired_count";
    case timepop_dispatch_trace_stage_t::DEFERRED_SELECTED:
      return "deferred_generation";
    case timepop_dispatch_trace_stage_t::CALLBACK_ENTER:
      return (kind_id == timepop_dispatch_trace_kind_t::ASAP ||
              kind_id == timepop_dispatch_trace_kind_t::ALAP)
          ? "deferred_generation"
          : "slot_flags";
    case timepop_dispatch_trace_stage_t::CALLBACK_RETURN:
      return kind_id == timepop_dispatch_trace_kind_t::ISR_TIMED
          ? "slot_flags"
          : "callback_body_cycles";
    case timepop_dispatch_trace_stage_t::DEFERRED_CLEANUP:
      return "pending(bit0)|dispatching(bit1)|generation(bits16-31)";
    case timepop_dispatch_trace_stage_t::MUTATION_BARRIER_ENTER:
      return "queued_mutation_count";
    case timepop_dispatch_trace_stage_t::MUTATION_SELECTED:
      return "mutation_kind_id";
    case timepop_dispatch_trace_stage_t::MUTATION_RESULT:
      return "kind(bits0-7)|ok(bit8)|remaining_queue(bits16-31)";
    case timepop_dispatch_trace_stage_t::MUTATION_BARRIER_EXIT:
      return "schedule_next_call_delta";
    case timepop_dispatch_trace_stage_t::TIMED_SELECTED:
      return "slot_flags|callback_already_ran(bit8)";
    case timepop_dispatch_trace_stage_t::TIMED_SLOT_AFTER_CALLBACK:
      return "slot_flags|already_ran(bit8)|handle_match(bit9)|callback_match(bit10)";
    case timepop_dispatch_trace_stage_t::REARM_BEGIN:
    case timepop_dispatch_trace_stage_t::REARM_END:
    case timepop_dispatch_trace_stage_t::IRQ_SELECTED:
      return "slot_deadline";
    case timepop_dispatch_trace_stage_t::SLOT_RETIRED:
      return "retire_reason_id";
    case timepop_dispatch_trace_stage_t::SLOT_REPLACED:
      return "prior_handle";
    case timepop_dispatch_trace_stage_t::DISPATCH_LEAVE:
      return "timepop_pending";
    case timepop_dispatch_trace_stage_t::ISR_ENTER:
      return "captured_counter_or_source_status";
    case timepop_dispatch_trace_stage_t::ISR_CAPTURED:
      return "capture_window_cycles";
    case timepop_dispatch_trace_stage_t::ISR_EXIT:
      return "captured_counter_or_source_status";
    case timepop_dispatch_trace_stage_t::HANDOFF_ENTER:
      return "pending";
    case timepop_dispatch_trace_stage_t::HANDOFF_DEQUEUE:
      return "handoff_entry_dwt";
    case timepop_dispatch_trace_stage_t::HANDOFF_EXIT:
      return "drained_count|pending_or_reentry(bit31)";
    case timepop_dispatch_trace_stage_t::SUBSCRIBER_SELECTED:
      return "dispatch_mode(bits0-1)|binding_generation(bits16-31)";
    case timepop_dispatch_trace_stage_t::SUBSCRIBER_ENTER:
    case timepop_dispatch_trace_stage_t::SUBSCRIBER_RETURN:
      return "event_counter32_or_source_specific";
    default:
      return "raw";
  }
}

static FLASHMEM Payload system_timepop_dispatch_trace_entry_payload(
    const timepop_dispatch_trace_entry_t& entry) {
  Payload out;
  out.add("sequence", entry.sequence);
  out.add("stage_id", entry.stage);
  out.add("stage", timepop_dispatch_trace_stage_name(entry.stage));
  out.add("phase_id", entry.phase);
  out.add("phase", timepop_dispatch_trace_phase_name(entry.phase));
  out.add("kind_id", entry.kind);
  out.add("kind", timepop_dispatch_trace_kind_name(entry.kind));
  out.add("slot_index", entry.slot_index);
  out.add("has_slot", entry.slot_index != TIMEPOP_DISPATCH_TRACE_NO_SLOT);
  out.add("handle", entry.handle);

  // Generic Execution Trace aliases.  The established TimePop field names
  // remain below for compatibility with TIMEPOP_DISPATCH_INFO.
  out.add("subject_index", entry.slot_index);
  out.add("identity", entry.handle);

  system_crash_add_hex32(out, "callback", entry.callback);
  out.add("callback_null", entry.callback == 0U);
  system_crash_add_hex32(out, "slot_callback", entry.slot_callback);
  out.add("callback_matches_slot",
          entry.callback != 0U && entry.callback == entry.slot_callback);
  out.add("callback_executable",
          timepop_dispatch_trace_executable_address(entry.callback));
  out.add("slot_callback_executable",
          timepop_dispatch_trace_executable_address(entry.slot_callback));

  system_crash_add_hex32(out, "target", entry.callback);
  system_crash_add_hex32(out, "related_target", entry.slot_callback);
  out.add("target_executable",
          timepop_dispatch_trace_executable_address(entry.callback));

  system_crash_add_hex32(out, "user_data", entry.user_data);
  system_crash_add_hex32(out, "name_ptr", entry.name_ptr);
  system_crash_add_hex32(out, "object", entry.user_data);
  system_crash_add_hex32(out, "label_ptr", entry.name_ptr);
  system_crash_add_hex32(out, "caller_sp", entry.caller_sp);
  system_crash_add_hex32(out, "site_pc", entry.site_pc);
  out.add("site_pc_executable",
          timepop_dispatch_trace_executable_address(entry.site_pc));
  system_crash_add_hex32(out, "dwt", entry.dwt);
  out.add("ipsr", entry.ipsr);
  system_crash_add_hex32(out, "aux", entry.aux);
  out.add("aux_meaning",
          timepop_dispatch_trace_aux_meaning(entry.stage, entry.kind));
  if ((timepop_dispatch_trace_stage_t)entry.stage ==
      timepop_dispatch_trace_stage_t::SLOT_RETIRED) {
    out.add("retire_reason",
            timepop_dispatch_trace_retire_reason(entry.aux));
  }
  if ((timepop_dispatch_trace_stage_t)entry.stage ==
      timepop_dispatch_trace_stage_t::MUTATION_RESULT) {
    out.add("mutation_ok", (entry.aux & 0x00000100UL) != 0U);
    out.add("mutation_kind_id", entry.aux & 0xFFU);
    out.add("mutation_queue_remaining", entry.aux >> 16);
  }
  return out;
}

static FLASHMEM Payload system_timepop_dispatch_trace_compact_entry_payload(
    const timepop_dispatch_trace_entry_t& entry) {
  Payload out;
  out.add("sequence", entry.sequence);
  out.add("stage_id", entry.stage);
  out.add("stage", timepop_dispatch_trace_stage_name(entry.stage));
  out.add("phase", timepop_dispatch_trace_phase_name(entry.phase));
  out.add("kind", timepop_dispatch_trace_kind_name(entry.kind));
  out.add("slot_index", entry.slot_index);
  out.add("handle", entry.handle);
  out.add("subject_index", entry.slot_index);
  out.add("identity", entry.handle);
  system_crash_add_hex32(out, "callback", entry.callback);
  system_crash_add_hex32(out, "slot_callback", entry.slot_callback);
  system_crash_add_hex32(out, "target", entry.callback);
  system_crash_add_hex32(out, "related_target", entry.slot_callback);
  out.add("callback_matches_slot",
          entry.callback != 0U && entry.callback == entry.slot_callback);
  out.add("callback_executable",
          timepop_dispatch_trace_executable_address(entry.callback));
  system_crash_add_hex32(out, "user_data", entry.user_data);
  system_crash_add_hex32(out, "name_ptr", entry.name_ptr);
  system_crash_add_hex32(out, "object", entry.user_data);
  system_crash_add_hex32(out, "label_ptr", entry.name_ptr);
  system_crash_add_hex32(out, "caller_sp", entry.caller_sp);
  system_crash_add_hex32(out, "site_pc", entry.site_pc);
  system_crash_add_hex32(out, "dwt", entry.dwt);
  out.add("ipsr", entry.ipsr);
  system_crash_add_hex32(out, "aux", entry.aux);
  out.add("aux_meaning",
          timepop_dispatch_trace_aux_meaning(entry.stage, entry.kind));
  return out;
}

static FLASHMEM Payload system_execution_trace_payload(
    const Payload& args,
    bool legacy_timepop_schema) {
  timepop_dispatch_trace_snapshot(&g_system_timepop_dispatch_trace_scratch);

  bool bank_valid = true;
  const bool live_bank = system_trace_report_live_bank(args, &bank_valid);
  if (!bank_valid) {
    Payload error;
    error.add("error", "bank must be live or retained");
    return error;
  }

  const uint32_t requested = system_trace_report_count(args);
  const uint32_t offset = system_trace_report_offset(args);
  const timepop_dispatch_trace_bank_snapshot_t& bank = live_bank
      ? g_system_timepop_dispatch_trace_scratch.live
      : g_system_timepop_dispatch_trace_scratch.retained;
  uint32_t begin = 0U;
  uint32_t end = 0U;
  system_trace_bounds(bank.count, requested, offset, &begin, &end);

  Payload out;
  out.add("schema", legacy_timepop_schema
      ? "ZPNET_TIMEPOP_DISPATCH_TRACE_V2"
      : "ZPNET_EXECUTION_TRACE_V1");
  out.add("report", legacy_timepop_schema
      ? "TIMEPOP_DISPATCH_INFO"
      : "EXECUTION_TRACE");
  out.add("bank", live_bank ? "live" : "retained");
  out.add("valid", bank.valid);
  out.add("fault_captured", bank.fault_captured);
  system_crash_add_hex32(out, "fault_dwt", bank.fault_dwt);
  out.add("crash_sequence", bank.crash_sequence);
  out.add("trace_sequence_at_capture", bank.trace_sequence_at_capture);
  out.add("capacity", TIMEPOP_DISPATCH_TRACE_ENTRIES);
  out.add("available_count", bank.count);
  out.add("newest_sequence", bank.newest_sequence);
  out.add("requested_count", requested);
  out.add("returned_count", end - begin);
  out.add("offset_from_newest", offset);
  out.add("has_older", begin != 0U);
  out.add("has_newer", offset != 0U && bank.count != 0U);
  out.add("first_sequence", begin < end ? bank.entries[begin].sequence : 0U);
  out.add("last_sequence", begin < end ? bank.entries[end - 1U].sequence : 0U);

  PayloadArray records;
  for (uint32_t i = begin; i < end; ++i) {
    Payload record =
        system_timepop_dispatch_trace_compact_entry_payload(bank.entries[i]);
    record.add("bank_index", i);
    records.add(record);
  }
  out.add_array("records", records);
  return out;
}

static FLASHMEM Payload system_execution_trace_summary_payload(void) {
  timepop_dispatch_trace_snapshot(&g_system_timepop_dispatch_trace_scratch);
  const timepop_dispatch_trace_bank_snapshot_t& retained =
      g_system_timepop_dispatch_trace_scratch.retained;

  Payload out;
  out.add("schema", "ZPNET_EXECUTION_TRACE_SUMMARY_V1");
  out.add("retained_valid", retained.valid);
  out.add("fault_captured", retained.fault_captured);
  system_crash_add_hex32(out, "fault_dwt", retained.fault_dwt);
  out.add("crash_sequence", retained.crash_sequence);
  out.add("trace_sequence_at_capture", retained.trace_sequence_at_capture);
  out.add("retained_count", retained.count);
  out.add("newest_sequence", retained.newest_sequence);
  if (retained.valid && retained.count != 0U) {
    out.add_object(
        "newest",
        system_timepop_dispatch_trace_entry_payload(
            retained.entries[retained.count - 1U]));
  }
  return out;
}

// ================================================================
// Payload flight recorder — reporting surface
// ================================================================

// The snapshot is taken into static storage before response construction
// begins, because building the response mutates the live ring.
static payload_flight_info_t g_system_payload_flight_scratch;
static payload_append_trace_snapshot_t
    g_system_payload_append_trace_scratch DMAMEM;

static FLASHMEM Payload system_payload_flight_bank_payload(
    uint32_t valid,
    uint32_t sequence,
    uint32_t count,
    const payload_flight_entry_t* entries) {
  Payload out;
  out.add("valid", valid != 0U);
  out.add("sequence", sequence);
  out.add("count", count);

  PayloadArray records;
  for (uint32_t i = 0; i < count && i < PAYLOAD_FLIGHT_ENTRIES; ++i) {
    const payload_flight_entry_t* e = &entries[i];
    Payload record;
    record.add("index", i);
    record.add("op", payload_operation_id_name(e->op_id));
    system_crash_add_hex32(record, "op_id", e->op_id);
    system_crash_add_hex32(record, "this", e->this_ptr);
    record.add("dwt", e->dwt_cyccnt);
    record.add("ipsr", (uint32_t)e->ipsr);
    record.add("error", (e->flags & PAYLOAD_FLIGHT_FLAG_ERROR) != 0U);
    record.add("integrity", (e->flags & PAYLOAD_FLIGHT_FLAG_INTEGRITY) != 0U);
    records.add(record);
  }
  out.add_array("records", records);
  return out;
}

static FLASHMEM Payload system_payload_flight_payload(bool retained_only) {
  payload_get_flight_info(&g_system_payload_flight_scratch);
  const payload_flight_info_t& f = g_system_payload_flight_scratch;

  Payload out;
  out.add("schema", "ZPNET_PAYLOAD_FLIGHT_V1");
  out.add_object("retained",
                 system_payload_flight_bank_payload(f.retained_valid,
                                                    f.retained_sequence,
                                                    f.retained_count,
                                                    f.retained));
  if (!retained_only) {
    out.add_object("live",
                   system_payload_flight_bank_payload(f.live_valid,
                                                      f.live_sequence,
                                                      f.live_count,
                                                      f.live));
  }
  return out;
}


// ================================================================
// Payload v4.1 append transaction recorder — reporting surface
// ================================================================

static const char* payload_append_trace_stage_name(uint32_t stage) {
  switch ((payload_append_trace_stage_t)stage) {
    case payload_append_trace_stage_t::ENTER:           return "ENTER";
    case payload_append_trace_stage_t::PRE_ENSURE:      return "PRE_ENSURE";
    case payload_append_trace_stage_t::ENSURE_FAILED:   return "ENSURE_FAILED";
    case payload_append_trace_stage_t::POST_ENSURE:     return "POST_ENSURE";
    case payload_append_trace_stage_t::PRE_VALUE_COPY:  return "PRE_VALUE_COPY";
    case payload_append_trace_stage_t::VALUE_COPY_DONE: return "VALUE_COPY_DONE";
    case payload_append_trace_stage_t::PRE_KEY_COPY:    return "PRE_KEY_COPY";
    case payload_append_trace_stage_t::KEY_COPY_DONE:   return "KEY_COPY_DONE";
    case payload_append_trace_stage_t::COMMIT:          return "COMMIT";
    case payload_append_trace_stage_t::FINAL_SPAN_FAIL: return "FINAL_SPAN_FAIL";
    default:                                             return "NONE";
  }
}

static FLASHMEM Payload system_payload_append_trace_entry_payload(
    const payload_append_trace_entry_t& entry) {
  Payload out;
  out.add("sequence", entry.sequence);
  out.add("stage_id", entry.stage);
  out.add("stage", payload_append_trace_stage_name(entry.stage));
  system_crash_add_hex32(out, "this", entry.this_ptr);
  system_crash_add_hex32(out, "key_ptr", entry.key_ptr);
  system_crash_add_hex32(out, "value_ptr", entry.value_ptr);
  out.add("key_len", entry.key_len);
  out.add("value_len", entry.value_len);
  out.add("kind", entry.kind);
  system_crash_add_hex32(out, "storage_ptr", entry.storage_ptr);
  out.add("capacity", entry.capacity);
  out.add("data_begin", entry.data_begin);
  out.add("entry_count", entry.entry_count);
  out.add("key_alias", (entry.alias_flags & 1U) != 0U);
  out.add("value_alias", (entry.alias_flags & 2U) != 0U);
  out.add("key_offset", entry.key_offset);
  out.add("value_offset", entry.value_offset);
  out.add("data_shift", entry.data_shift);
  out.add("key_off", entry.key_off);
  out.add("val_off", entry.val_off);
  system_crash_add_hex32(out, "dwt", entry.dwt_cyccnt);
  out.add("ipsr", entry.ipsr);
  return out;
}

static FLASHMEM Payload system_payload_append_trace_payload(
    const Payload& args) {
  payload_get_append_trace(&g_system_payload_append_trace_scratch);

  bool bank_valid = true;
  const bool live_bank = system_trace_report_live_bank(args, &bank_valid);
  if (!bank_valid) {
    Payload error;
    error.add("error", "bank must be live or retained");
    return error;
  }

  const uint32_t requested = system_trace_report_count(args);
  const uint32_t offset = system_trace_report_offset(args);
  const payload_append_trace_bank_snapshot_t& bank = live_bank
      ? g_system_payload_append_trace_scratch.live
      : g_system_payload_append_trace_scratch.retained;
  uint32_t begin = 0U;
  uint32_t end = 0U;
  system_trace_bounds(bank.count, requested, offset, &begin, &end);

  Payload out;
  out.add("schema", "ZPNET_PAYLOAD_APPEND_TRACE_V1");
  out.add("bank", live_bank ? "live" : "retained");
  out.add("valid", bank.valid != 0U);
  out.add("capacity", PAYLOAD_APPEND_TRACE_ENTRIES);
  out.add("available_count", bank.count);
  out.add("newest_sequence", bank.newest_sequence);
  out.add("requested_count", requested);
  out.add("returned_count", end - begin);
  out.add("offset_from_newest", offset);
  out.add("has_older", begin != 0U);
  out.add("has_newer", offset != 0U && bank.count != 0U);
  out.add("first_sequence", begin < end ? bank.entries[begin].sequence : 0U);
  out.add("last_sequence", begin < end ? bank.entries[end - 1U].sequence : 0U);

  PayloadArray records;
  for (uint32_t i = begin; i < end; ++i) {
    Payload record =
        system_payload_append_trace_entry_payload(bank.entries[i]);
    record.add("bank_index", i);
    records.add(record);
  }
  out.add_array("records", records);
  return out;
}

static FLASHMEM Payload system_payload_append_trace_summary_payload(void) {
  payload_get_append_trace(&g_system_payload_append_trace_scratch);
  const payload_append_trace_bank_snapshot_t& retained =
      g_system_payload_append_trace_scratch.retained;

  Payload out;
  out.add("schema", "ZPNET_PAYLOAD_APPEND_TRACE_SUMMARY_V1");
  out.add("retained_valid", retained.valid != 0U);
  out.add("retained_count", retained.count);
  out.add("newest_sequence", retained.newest_sequence);
  if (retained.valid != 0U && retained.count != 0U) {
    out.add_object(
        "newest",
        system_payload_append_trace_entry_payload(
            retained.entries[retained.count - 1U]));
  }
  return out;
}


// ================================================================
// Payload design-by-contract — reporting and event service
// ================================================================

static payload_contract_info_t
    g_system_payload_contract_info_scratch DMAMEM;

static bool system_payload_contract_incident_valid(
    const payload_contract_incident_t& incident) {
  return incident.sequence != 0U &&
         (incident.sequence ^ incident.sequence_inv) == 0xFFFFFFFFUL;
}

static FLASHMEM Payload system_payload_contract_incident_payload(
    const payload_contract_incident_t& incident) {
  Payload out;
  const bool valid = system_payload_contract_incident_valid(incident);
  out.add("valid", valid);
  if (!valid) return out;

  out.add("sequence", incident.sequence);
  out.add("phase_id", incident.phase);
  out.add("phase", payload_contract_phase_name(incident.phase));
  out.add("reason_id", incident.reason);
  out.add("reason", payload_contract_reason_name(incident.reason));
  out.add("operation_id", incident.operation_id);
  out.add("operation", payload_operation_id_name(incident.operation_id));
  system_crash_add_hex32(out, "object", incident.object_ptr);
  system_crash_add_hex32(out, "related", incident.related_ptr);
  out.add("generation", incident.generation);
  out.add("entry_index", incident.entry_index);
  system_crash_add_hex32(out, "expected0", incident.expected0);
  system_crash_add_hex32(out, "observed0", incident.observed0);
  system_crash_add_hex32(out, "expected1", incident.expected1);
  system_crash_add_hex32(out, "observed1", incident.observed1);
  system_crash_add_hex32(out, "before_fingerprint",
                         incident.before_fingerprint);
  system_crash_add_hex32(out, "after_fingerprint",
                         incident.after_fingerprint);
  system_crash_add_hex32(out, "dwt", incident.dwt_cyccnt);
  out.add("ipsr", incident.ipsr);
  return out;
}

static FLASHMEM Payload system_payload_contract_info_payload(void) {
  payload_contract_get_info(&g_system_payload_contract_info_scratch);
  const payload_contract_info_t& info =
      g_system_payload_contract_info_scratch;

  Payload out;
  out.add("schema", "ZPNET_PAYLOAD_CONTRACT_V1");
  out.add("checks", info.checks);
  out.add("successful_mutations", info.successful_mutations);
  out.add("precondition_failures", info.precondition_failures);
  out.add("pre_invariant_failures", info.pre_invariant_failures);
  out.add("post_invariant_failures", info.post_invariant_failures);
  out.add("postcondition_failures", info.postcondition_failures);
  out.add("observed_drift_failures", info.observed_drift_failures);
  out.add("mutation_failures", info.mutation_failures);
  out.add("incidents", info.incidents);
  out.add("pending_events", info.pending_events);
  out.add("pending_overflow", info.pending_overflow);
  out.add("event_emitted", info.event_emitted);
  out.add("event_emit_failed", info.event_emit_failed);
  out.add("event_incidents_suppressed",
          info.event_incidents_suppressed);
  out.add_object("first_this_boot",
                 system_payload_contract_incident_payload(
                     info.first_this_boot));
  out.add_object("latest_this_boot",
                 system_payload_contract_incident_payload(
                     info.latest_this_boot));
  out.add_object("latest_retained",
                 system_payload_contract_incident_payload(
                     info.latest_retained));
  return out;
}

static FLASHMEM Payload system_payload_contract_summary_payload(void) {
  payload_contract_get_info(&g_system_payload_contract_info_scratch);
  const payload_contract_info_t& info =
      g_system_payload_contract_info_scratch;

  Payload out;
  out.add("schema", "ZPNET_PAYLOAD_CONTRACT_SUMMARY_V1");
  out.add("incidents", info.incidents);
  out.add("pending_events", info.pending_events);
  out.add("post_invariant_failures", info.post_invariant_failures);
  out.add("postcondition_failures", info.postcondition_failures);
  out.add("observed_drift_failures", info.observed_drift_failures);
  out.add_object("latest_retained",
                 system_payload_contract_incident_payload(
                     info.latest_retained));
  return out;
}


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
  system_dmamem_ensure_initialized();

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

static crash_stack_watch_snapshot_t g_system_stack_watch_scratch DMAMEM;

static FLASHMEM Payload system_stack_watch_entry_payload(
    const crash_stack_watch_entry_t& entry,
    uint32_t watch_address,
    uint32_t watch_bytes) {
  Payload p;
  p.add("sequence", entry.sequence);
  p.add("dwt", entry.dwt);
  system_crash_add_hex32(p, "pc", entry.pc);
  system_crash_add_hex32(p, "lr", entry.lr);
  system_crash_add_hex32(p, "xpsr", entry.xpsr);
  system_crash_add_hex32(p, "writer_sp", entry.writer_sp);
  system_crash_add_hex32(p, "exc_return", entry.exc_return);
  p.add("anomalous",
        entry.writer_sp >= watch_address + watch_bytes);
  PayloadArray watched;
  for (size_t i = 0; i < 4U; ++i) {
    Payload word;
    word.add("index", (uint32_t)i);
    system_crash_add_hex32(word, "address",
                           watch_address + (uint32_t)(i * 4U));
    system_crash_add_hex32(word, "value", entry.watched[i]);
    watched.add(word);
  }
  p.add_array("watched", watched);
  return p;
}

static FLASHMEM Payload system_stack_watch_bank_payload(
    const crash_stack_watch_bank_snapshot_t& bank,
    uint32_t watch_address,
    uint32_t watch_bytes) {
  Payload p;
  p.add("valid", bank.valid);
  p.add("count", bank.count);
  p.add("newest_sequence", bank.newest_sequence);
  p.add("hits_total", bank.hits_total);
  p.add("anomalous_total", bank.anomalous_total);
  if (!bank.valid) return p;

  if (bank.first_anomalous.sequence != 0U) {
    p.add_object("first_anomalous",
                 system_stack_watch_entry_payload(bank.first_anomalous,
                                                  watch_address,
                                                  watch_bytes));
  }

  PayloadArray entries;
  for (uint32_t i = 0U; i < bank.count; ++i) {
    entries.add(system_stack_watch_entry_payload(bank.entries[i],
                                                 watch_address,
                                                 watch_bytes));
  }
  p.add_array("entries", entries);
  return p;
}

static FLASHMEM Payload system_stack_watch_payload(void) {
  system_dmamem_ensure_initialized();
  crash_stack_watch_service();
  crash_stack_watch_snapshot(&g_system_stack_watch_scratch);
  const crash_stack_watch_snapshot_t& snap = g_system_stack_watch_scratch;

  Payload p;
  p.add("schema", "ZPNET_STACK_WATCH_V1");
  p.add("enabled", snap.enabled);
  p.add("armed", snap.armed);
  p.add("arm_skip_reason", snap.arm_skip_reason);
  p.add("arm_skip_reason_name",
        crash_stack_watch_skip_reason_name(snap.arm_skip_reason));
  p.add("arm_attempts", snap.arm_attempts);
  system_crash_add_hex32(p, "last_dhcsr", snap.last_dhcsr);
  system_crash_add_hex32(p, "watch_address", snap.watch_address);
  p.add("watch_bytes", snap.watch_bytes);
  system_crash_add_hex32(p, "effective_comp_address",
                         snap.effective_comp_address);
  p.add("effective_mask", snap.effective_mask);
  p.add_object("retained",
               system_stack_watch_bank_payload(snap.retained,
                                               snap.watch_address,
                                               snap.watch_bytes));
  p.add_object("live",
               system_stack_watch_bank_payload(snap.live,
                                               snap.watch_address,
                                               snap.watch_bytes));
  return p;
}

static crash_stack_tripwire_snapshot_t g_system_stack_tripwire_scratch DMAMEM;

static FLASHMEM Payload system_stack_tripwire_entry_payload(
    const crash_stack_tripwire_entry_t& entry) {
  Payload p;
  p.add("sequence", entry.sequence);
  p.add("site", entry.site);
  p.add("site_name", crash_stack_tripwire_site_name(entry.site));
  system_crash_add_hex32(p, "msp", entry.msp);
  system_crash_add_hex32(p, "exc_return", entry.exc_return);
  system_crash_add_hex32(p, "floor", entry.floor);
  system_crash_add_hex32(p, "frame_top_estimate", entry.frame_top_estimate);
  p.add("overshoot_bytes", entry.frame_top_estimate - entry.floor);
  p.add("dwt", entry.dwt);
  p.add("ipsr", entry.ipsr);
  return p;
}

static FLASHMEM Payload system_stack_tripwire_bank_payload(
    const crash_stack_tripwire_bank_snapshot_t& bank) {
  Payload p;
  p.add("valid", bank.valid);
  p.add("count", bank.count);
  p.add("newest_sequence", bank.newest_sequence);
  p.add("violation_total", bank.violation_total);
  if (!bank.valid) return p;

  if (bank.first.sequence != 0U) {
    p.add_object("first", system_stack_tripwire_entry_payload(bank.first));
  }

  PayloadArray entries;
  for (uint32_t i = 0U; i < bank.count; ++i) {
    entries.add(system_stack_tripwire_entry_payload(bank.entries[i]));
  }
  p.add_array("entries", entries);
  return p;
}

static FLASHMEM Payload system_stack_tripwire_payload(void) {
  system_dmamem_ensure_initialized();
  crash_stack_tripwire_snapshot(&g_system_stack_tripwire_scratch);
  const crash_stack_tripwire_snapshot_t& snap = g_system_stack_tripwire_scratch;

  Payload p;
  p.add("schema", "ZPNET_STACK_TRIPWIRE_V1");
  p.add("floor_active_now", snap.floor_active_now);
  system_crash_add_hex32(p, "floor_now", snap.floor_now);
  p.add("floor_publish_count", snap.floor_publish_count);
  p.add_object("retained", system_stack_tripwire_bank_payload(snap.retained));
  p.add_object("live", system_stack_tripwire_bank_payload(snap.live));
  return p;
}

static crash_dispatch_breadcrumb_snapshot_t
    g_system_dispatch_breadcrumb_scratch DMAMEM;

static FLASHMEM Payload system_dispatch_breadcrumb_entry_payload(
    bool valid,
    const crash_dispatch_breadcrumb_t& breadcrumb) {
  Payload p;
  p.add("valid", valid);
  if (!valid) return p;

  p.add("schema_version", breadcrumb.schema_version);
  p.add("sequence", breadcrumb.sequence);
  p.add("stage_id", breadcrumb.stage);
  p.add("stage",
        crash_dispatch_breadcrumb_stage_name(breadcrumb.stage));
  system_crash_add_hex32(p, "dwt", breadcrumb.dwt);
  system_crash_add_hex32(p, "msp", breadcrumb.msp);
  system_crash_add_hex32(p, "lr", breadcrumb.lr);
  system_crash_add_hex32(p, "basepri", breadcrumb.basepri);
  p.add("ipsr", breadcrumb.ipsr);
  system_crash_add_hex32(p, "callback", breadcrumb.callback);
  p.add("subscriber_kind", breadcrumb.subscriber_kind);
  system_crash_add_hex32(p, "event_counter32",
                         breadcrumb.event_counter32);
  system_crash_add_hex32(p, "user_data", breadcrumb.user_data);
  system_crash_add_hex32(p, "runtime", breadcrumb.runtime);
  return p;
}

static FLASHMEM Payload system_dispatch_breadcrumb_payload(void) {
  system_dmamem_ensure_initialized();
  crash_dispatch_breadcrumb_snapshot(
      &g_system_dispatch_breadcrumb_scratch);

  Payload p;
  p.add("schema", "ZPNET_DISPATCH_BREADCRUMB_V1");
  p.add_object(
      "retained",
      system_dispatch_breadcrumb_entry_payload(
          g_system_dispatch_breadcrumb_scratch.retained_valid,
          g_system_dispatch_breadcrumb_scratch.retained));
  p.add_object(
      "live",
      system_dispatch_breadcrumb_entry_payload(
          g_system_dispatch_breadcrumb_scratch.live_valid,
          g_system_dispatch_breadcrumb_scratch.live));
  return p;
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
  p.add_object("extended", system_crash_forensics_payload());

  // Runtime breadcrumbs, unified Execution Trace, and Payload forensics remain
  // retained through reboot beside the processor exception evidence.
  p.add_object("runtime_ledger", system_runtime_ledger_payload());
  p.add_object("dispatch_breadcrumb",
               system_dispatch_breadcrumb_payload());
  p.add_object("execution_trace",
               system_execution_trace_summary_payload());
  p.add_object("payload_flight", system_payload_flight_payload(true));
  p.add_object("payload_append_trace",
               system_payload_append_trace_summary_payload());
  p.add_object("payload_contract",
               system_payload_contract_summary_payload());
  p.add_object("stack_watch", system_stack_watch_payload());
  p.add_object("stack_tripwire", system_stack_tripwire_payload());
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

  crash_forensics_status_t crash_status{};
  crash_forensics_get_status(&crash_status);
  p.add("crash_forensics_installed", crash_status.installed);
  p.add("crash_forensics_present", crash_status.present);
  p.add("crash_forensics_header_valid", crash_status.header_valid);
  p.add("crash_forensics_crc_valid", crash_status.crc_valid);

  // CPU clock frequency — authoritative runtime value.
  // F_CPU_ACTUAL is updated by set_arm_clock() at boot.
  // This is the ACTUAL core clock, not the compile-time default.
  p.add("cpu_freq_mhz", (uint32_t)(F_CPU_ACTUAL / 1000000UL));

  // Deliberately omit CPU temperature and internal-reference voltage from
  // periodic SYSTEM.REPORT.  Their acquisition APIs return floating point, and
  // this report executes on the shared foreground/exception MSP stack under
  // frequent timing interrupts.  Reintroduce them only through a focused,
  // explicitly isolated integer telemetry surface.
  // Heap availability
  // MULE: COMMENTED OUT for STABILITY
  //p.add("free_heap_bytes", freeHeapBytes());
  //p.add("max_alloc_bytes", maxAllocBytes());

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

  // CPU work means non-spin foreground/ISR work.  True core occupancy is
  // intentionally near 100% because the idle DWT witness loop runs when idle.
  //
  // Publish the already-computed millipercent scalars directly.  SYSTEM.REPORT
  // must remain integer-only: no floating-point division, fixed_decimal_t
  // structure return, temporary-reference binding, or numeric reformatting.
  // The Pi SYSTEM process treats this Teensy payload as a transitive dictionary.
  p.add("cpu_usage_pct_milli", cpu_work_pct_milli);
  p.add("cpu_work_pct_milli", cpu_work_pct_milli);
  p.add("cpu_idle_spin_pct_milli", cpu_idle_spin_pct_milli);

  // SpinIdle-derived CPU work diagnostics
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
  // RX — Guarded RAM2 placement experiment
  // ==========================================================

  p.add("rx_buffer_in_dmamem", info.rx_buffer_in_dmamem != 0U);
  p.add_fmt("rx_buffer_address", "0x%08lX",
            (unsigned long)info.rx_buffer_address);
  p.add("rx_buffer_size", info.rx_buffer_size);
  p.add("rx_buffer_alignment", info.rx_buffer_alignment);
  p.add("rx_buffer_alignment_ok", info.rx_buffer_alignment_ok != 0U);
  p.add_fmt("rx_poison_byte", "0x%02lX",
            (unsigned long)info.rx_poison_byte);
  p.add("rx_storage_init_count", info.rx_storage_init_count);
  p.add("rx_json_terminator_count", info.rx_json_terminator_count);
  p.add("rx_guard_check_count", info.rx_guard_check_count);
  p.add("rx_guard_failure_count", info.rx_guard_failure_count);
  p.add("rx_guard_before_failure_count",
        info.rx_guard_before_failure_count);
  p.add("rx_guard_after_failure_count",
        info.rx_guard_after_failure_count);
  p.add("rx_guard_last_stage", info.rx_guard_last_stage);
  p.add("rx_guard_last_stage_name",
        info.rx_guard_last_stage == 1U ? "BEFORE_PARSE" :
        info.rx_guard_last_stage == 2U ? "AFTER_PARSE" :
        info.rx_guard_last_stage == 3U ? "AFTER_DISPATCH" : "NONE");
  p.add("rx_guard_last_side", info.rx_guard_last_side);
  p.add("rx_guard_last_side_name",
        info.rx_guard_last_side == 1U ? "BEFORE_BUFFER" :
        info.rx_guard_last_side == 2U ? "AFTER_BUFFER" : "NONE");
  p.add("rx_guard_last_index", info.rx_guard_last_index);
  p.add_fmt("rx_guard_last_expected", "0x%02lX",
            (unsigned long)info.rx_guard_last_expected);
  p.add_fmt("rx_guard_last_observed", "0x%02lX",
            (unsigned long)info.rx_guard_last_observed);

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
  p.add("payload_json_invalid_syntax", info.json_invalid_syntax);
  p.add("payload_json_invalid_depth", info.json_invalid_depth);
  p.add("payload_json_invalid_utf8_key", info.json_invalid_utf8_key);
  p.add("payload_json_invalid_utf8_value", info.json_invalid_utf8_value);
  p.add("payload_json_invalid_raw_object", info.json_invalid_raw_object);
  p.add("payload_json_invalid_raw_array", info.json_invalid_raw_array);
  p.add("payload_json_decode_fail", info.json_decode_fail);
  p.add("payload_integrity_fail",     info.integrity_fail);
  p.add("payload_invalid_kind",       info.invalid_kind);

  // ==========================================================
  // Numeric admission / null substitution
  // ==========================================================

  p.add("payload_numeric_null_substitution", info.numeric_null_substitution);
  p.add("payload_numeric_nonfinite", info.numeric_nonfinite);
  p.add("payload_numeric_nan", info.numeric_nan);
  p.add("payload_numeric_positive_infinity", info.numeric_positive_infinity);
  p.add("payload_numeric_negative_infinity", info.numeric_negative_infinity);
  p.add("payload_numeric_invalid_token", info.numeric_invalid_token);
  p.add("payload_numeric_format_failure", info.numeric_format_failure);
  p.add("payload_numeric_null_insert_fail", info.numeric_null_insert_fail);
  p.add("payload_last_numeric_reject_reason", info.last_numeric_reject_reason);
  p.add("payload_last_numeric_reject_reason_name",
        payload_numeric_reject_reason_name(info.last_numeric_reject_reason));
  p.add("payload_last_numeric_reject_op_id", info.last_numeric_reject_op_id);
  p.add("payload_last_numeric_reject_op_name",
        payload_operation_id_name(info.last_numeric_reject_op_id));
  p.add_fmt("payload_last_numeric_reject_this", "0x%08lX",
            (unsigned long)info.last_numeric_reject_this);
  p.add("payload_last_numeric_reject_key", info.last_numeric_reject_key);
  p.add_fmt("payload_last_numeric_reject_value_bits", "0x%016llX",
            (unsigned long long)info.last_numeric_reject_value_bits);
  p.add("payload_last_numeric_reject_precision",
        info.last_numeric_reject_precision);
  p.add("payload_last_numeric_reject_format_return",
        info.last_numeric_reject_format_return);
  p.add("payload_last_numeric_reject_snprintf_return",
        info.last_numeric_reject_snprintf_return);
  p.add("payload_last_numeric_reject_text_len",
        info.last_numeric_reject_text_len);
  p.add("payload_last_numeric_reject_text_terminated",
        info.last_numeric_reject_text_terminated != 0U);
  p.add("payload_last_numeric_reject_text_truncated",
        info.last_numeric_reject_text_truncated != 0U);
  p.add("payload_last_numeric_reject_format",
        info.last_numeric_reject_format);
  p.add("payload_last_numeric_reject_text_printable",
        info.last_numeric_reject_text_printable);
  p.add("payload_last_numeric_reject_text_hex",
        info.last_numeric_reject_text_hex);

  // ==========================================================
  // First semantic serialization failure evidence
  // ==========================================================

  p.add("payload_semantic_validation_fail", info.semantic_validation_fail);
  p.add("payload_semantic_invalid_kind", info.semantic_invalid_kind);
  p.add("payload_semantic_invalid_key_utf8", info.semantic_invalid_key_utf8);
  p.add("payload_semantic_invalid_string_utf8", info.semantic_invalid_string_utf8);
  p.add("payload_semantic_invalid_number_token", info.semantic_invalid_number_token);
  p.add("payload_semantic_invalid_boolean_token", info.semantic_invalid_boolean_token);
  p.add("payload_semantic_invalid_null_token", info.semantic_invalid_null_token);
  p.add("payload_semantic_invalid_object_json", info.semantic_invalid_object_json);
  p.add("payload_semantic_invalid_array_json", info.semantic_invalid_array_json);
  p.add("payload_first_semantic_fail_captured", info.first_semantic_fail_captured);
  p.add("payload_first_semantic_fail_reason", info.first_semantic_fail_reason);
  p.add("payload_first_semantic_fail_reason_name",
        payload_semantic_fail_reason_name(info.first_semantic_fail_reason));
  p.add("payload_first_semantic_fail_op_id", info.first_semantic_fail_op_id);
  p.add("payload_first_semantic_fail_op_name",
        payload_operation_id_name(info.first_semantic_fail_op_id));
  p.add_fmt("payload_first_semantic_fail_this", "0x%08lX",
            (unsigned long)info.first_semantic_fail_this);
  p.add("payload_first_semantic_fail_entry_index", info.first_semantic_fail_entry_index);
  p.add("payload_first_semantic_fail_entry_kind", info.first_semantic_fail_entry_kind);
  p.add("payload_first_semantic_fail_key_off", info.first_semantic_fail_key_off);
  p.add("payload_first_semantic_fail_key_len", info.first_semantic_fail_key_len);
  p.add("payload_first_semantic_fail_val_off", info.first_semantic_fail_val_off);
  p.add("payload_first_semantic_fail_val_len", info.first_semantic_fail_val_len);
  p.add("payload_first_semantic_fail_key", info.first_semantic_fail_key);

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
  p.add("payload_last_string_pointer_fault_op_id",
        info.last_string_pointer_fault_op_id);
  p.add("payload_last_string_pointer_fault_op_name",
        payload_operation_id_name(info.last_string_pointer_fault_op_id));
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
  p.add("payload_last_self_ok_fail_op_id", info.last_self_ok_fail_op_id);
  p.add("payload_last_self_ok_fail_op_name",
        payload_operation_id_name(info.last_self_ok_fail_op_id));
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
  p.add("payload_last_self_ok_fail_capacity", info.last_self_ok_fail_capacity);
  p.add("payload_last_self_ok_fail_data_begin", info.last_self_ok_fail_data_begin);
  p.add("payload_last_self_ok_fail_expected_upper",
        info.last_self_ok_fail_expected_upper);
  p.add("payload_last_self_ok_fail_key_end", info.last_self_ok_fail_key_end);
  p.add("payload_last_self_ok_fail_val_end", info.last_self_ok_fail_val_end);
  p.add("payload_first_self_ok_fail_captured", info.first_self_ok_fail_captured);

  // ==========================================================
  // Last error breadcrumb
  // ==========================================================

  p.add("payload_last_error_code",  info.last_error_code);
  p.add("payload_last_error_name",  payload_error_code_name(info.last_error_code));
  p.add("payload_last_error_count", info.last_error_count);
  p.add("payload_last_error_op",    info.last_error_op);
  p.add("payload_last_error_op_id", info.last_error_op_id);
  p.add("payload_last_error_op_name",
        payload_operation_id_name(info.last_error_op_id));
  p.add_fmt("payload_last_error_this", "0x%08lX", (unsigned long)info.last_error_this);

  // ==========================================================
  // Execution-context census — handler-context Payload activity
  // ==========================================================

  p.add("payload_handler_ctx_ctor",   info.handler_ctx_ctor);
  p.add("payload_handler_ctx_mutate", info.handler_ctx_mutate);
  p.add("payload_handler_ctx_alloc",  info.handler_ctx_alloc);
  p.add("payload_handler_ctx_free",   info.handler_ctx_free);
  p.add("payload_last_handler_ctx_ipsr", info.last_handler_ctx_ipsr);
  p.add("payload_last_handler_ctx_op_id", info.last_handler_ctx_op_id);
  p.add("payload_last_handler_ctx_op_name",
        payload_operation_id_name(info.last_handler_ctx_op_id));
  p.add_fmt("payload_last_handler_ctx_this", "0x%08lX",
            (unsigned long)info.last_handler_ctx_this);
  p.add("payload_last_handler_ctx_dwt", info.last_handler_ctx_dwt);
  p.add_fmt("payload_last_handler_ctx_msp", "0x%08lX",
            (unsigned long)info.last_handler_ctx_msp);

  // ==========================================================
  // Allocator preemption-overlap tripwire
  // ==========================================================

  p.add("payload_alloc_overlap_detected", info.alloc_overlap_detected);
  p.add("payload_alloc_overlap_ipsr", info.alloc_overlap_ipsr);
  p.add("payload_alloc_overlap_op_id", info.alloc_overlap_op_id);
  p.add("payload_alloc_overlap_op_name",
        payload_operation_id_name(info.alloc_overlap_op_id));
  p.add_fmt("payload_alloc_overlap_this", "0x%08lX",
            (unsigned long)info.alloc_overlap_this);
  p.add("payload_alloc_overlap_dwt", info.alloc_overlap_dwt);
  p.add("payload_alloc_overlap_depth", info.alloc_overlap_depth);

  // ==========================================================
  // Design-by-contract court
  // ==========================================================

  p.add("payload_contract_checks", info.contract_checks);
  p.add("payload_contract_successful_mutations",
        info.contract_successful_mutations);
  p.add("payload_contract_precondition_failures",
        info.contract_precondition_failures);
  p.add("payload_contract_pre_invariant_failures",
        info.contract_pre_invariant_failures);
  p.add("payload_contract_post_invariant_failures",
        info.contract_post_invariant_failures);
  p.add("payload_contract_postcondition_failures",
        info.contract_postcondition_failures);
  p.add("payload_contract_observed_drift_failures",
        info.contract_observed_drift_failures);
  p.add("payload_contract_mutation_failures",
        info.contract_mutation_failures);
  p.add("payload_contract_incidents", info.contract_incidents);
  p.add("payload_contract_pending_events",
        info.contract_pending_events);
  p.add("payload_contract_pending_overflow",
        info.contract_pending_overflow);
  p.add("payload_contract_event_emitted",
        info.contract_event_emitted);
  p.add("payload_contract_event_emit_failed",
        info.contract_event_emit_failed);
  p.add("payload_contract_event_incidents_suppressed",
        info.contract_event_incidents_suppressed);

  return p;
}


static void system_dmamem_ensure_initialized(void) {
  if (g_system_dmamem_initialized) return;

  // Explicitly initialize the remaining SYSTEM RAM2 scratch objects before
  // early CLOCKS/INTERRUPT publishers can reach them.  RAM1 operational state
  // is reset here as well so the one-time startup transaction is deterministic.
  // This function remains lazy because those publishers run before
  // process_system_register().
  runtime_ledger_boot_latch();
  system_feature_registry_reset();
  memset(g_system_crash_report_text, 0, sizeof(g_system_crash_report_text));
  memset(g_system_debug_buffer, 0, sizeof(g_system_debug_buffer));
  memset((void*)&g_system_timepop_dispatch_trace_scratch, 0,
         sizeof(g_system_timepop_dispatch_trace_scratch));
  memset((void*)&g_system_payload_append_trace_scratch, 0,
         sizeof(g_system_payload_append_trace_scratch));
  memset((void*)&g_system_payload_contract_info_scratch, 0,
         sizeof(g_system_payload_contract_info_scratch));
  memset((void*)&g_system_dispatch_breadcrumb_scratch, 0,
         sizeof(g_system_dispatch_breadcrumb_scratch));

  // crash_forensics.cpp owns its retained RAM2 record.  Never initialize or
  // clear that record here; it may be the only surviving witness to a reboot.
  // The same custody applies to Payload flight rings and Payload append-trace
  // banks in payload.cpp: those remaining NOLOAD records are validated before
  // use and released only by explicit CRASH_CLEAR or their one-time boot latch.
  // Runtime and TimePop live recorders are RAM1 in this deterministic-memory
  // baseline and intentionally do not survive reboot.
  g_system_dmamem_initialized = true;
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
// EXECUTION_TRACE — retained/live ISR-to-TimePop control-flow transcript
// ------------------------------------------------------------
static FLASHMEM Payload cmd_execution_trace(const Payload& args) {
  return system_execution_trace_payload(args, false);
}

// ------------------------------------------------------------
// TIMEPOP_DISPATCH_INFO — compatibility alias for the shared transcript
// ------------------------------------------------------------
static FLASHMEM Payload cmd_timepop_dispatch_info(const Payload& args) {
  return system_execution_trace_payload(args, true);
}

// ------------------------------------------------------------
// PAYLOAD_FLIGHT_INFO — retained + live Payload flight recorder
// ------------------------------------------------------------
static FLASHMEM Payload cmd_payload_flight_info(const Payload& /*args*/) {
  return system_payload_flight_payload(false);
}


// ------------------------------------------------------------
// PAYLOAD_APPEND_TRACE — bounded retained/live append transaction transcript
// ------------------------------------------------------------
static FLASHMEM Payload cmd_payload_append_trace(const Payload& args) {
  return system_payload_append_trace_payload(args);
}

// ------------------------------------------------------------
// PAYLOAD_CONTRACT_INFO — contract counters and first/latest incidents
// ------------------------------------------------------------
static FLASHMEM Payload cmd_payload_contract_info(const Payload& /*args*/) {
  return system_payload_contract_info_payload();
}

// ------------------------------------------------------------
// CRASH_INFO — core CrashReport plus retained structured exception evidence
// ------------------------------------------------------------
static FLASHMEM Payload cmd_crash_info(const Payload& /*args*/) {
  system_crash_report_capture_once();
  return system_crash_report_payload(true);
}

// ------------------------------------------------------------
// CRASH_POLICY — live exception/FPU policy and retained-frame consistency
// ------------------------------------------------------------
static FLASHMEM Payload cmd_crash_policy(const Payload& /*args*/) {
  return system_crash_policy_payload();
}

// ------------------------------------------------------------
// CRASH_CLEAR — explicitly clear cached and core CrashReport state
// ------------------------------------------------------------
static FLASHMEM Payload cmd_crash_clear(const Payload& /*args*/) {
  CrashReport.clear();
  crash_forensics_clear();
  g_system_crash_report_captured = false;
  g_system_crash_report_core_fault_present = false;
  g_system_crash_report_truncated = false;
  g_system_crash_report_bytes = 0;
  g_system_crash_report_text[0] = '\0';
  memset((void*)&g_runtime_ledger_retained, 0,
         sizeof(g_runtime_ledger_retained));

  timepop_dispatch_trace_clear_retained();
  payload_clear_retained_append_trace();
  payload_contract_clear_retained();
  crash_stack_watch_clear_retained();
  crash_stack_tripwire_clear_retained();

  Payload resp = ok_payload();
  resp.add("crash_report_cleared", true);
  resp.add("crash_forensics_cleared", true);
  resp.add("dispatch_breadcrumb_retained_cleared", true);
  resp.add("runtime_ledger_cleared", true);
  resp.add("execution_trace_cleared", true);
  resp.add("timepop_dispatch_trace_cleared", true);
  resp.add("payload_append_trace_cleared", true);
  resp.add("stack_watch_retained_cleared", true);
  resp.add("stack_tripwire_retained_cleared", true);
  resp.add("payload_contract_cleared", true);
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
  { "ENTER_BOOTLOADER", cmd_enter_bootloader },
  { "SHUTDOWN",         cmd_shutdown         },
  { "PROCESS_LIST",     cmd_process_list     },
  { "FEATURES",         cmd_features         },
  { "REPORT_FEATURES",  cmd_features         },
  { "SET_FEATURE",      cmd_set_feature      },
  { "GET_FEATURE",      cmd_get_feature      },
  { "CRASH_INFO",       cmd_crash_info       },
  { "CRASH_POLICY",     cmd_crash_policy     },
  { "CRASH_CLEAR",      cmd_crash_clear      },
  { "EXECUTION_TRACE", cmd_execution_trace },
  { "TIMEPOP_DISPATCH_INFO", cmd_timepop_dispatch_info },
  { "PAYLOAD_FLIGHT_INFO", cmd_payload_flight_info },
  { "PAYLOAD_APPEND_TRACE", cmd_payload_append_trace },
  { "PAYLOAD_CONTRACT_INFO", cmd_payload_contract_info },
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
  system_dmamem_ensure_initialized();

  // Preserve feature states already published during early INTERRUPT/CLOCKS
  // hardware initialization.  The old unconditional registry reset occurred
  // after those publishers and erased their coherent boot evidence.
  system_feature_set("SYSTEM", "FEATURE_STATUS",
                     system_feature_status_t::NOMINAL,
                     "Teensy SYSTEM feature registry online");
  process_register("SYSTEM", &SYSTEM_PROCESS);
  g_system_feature_fragment_publish_enabled = true;
  system_feature_schedule_fragment_publish();

}
