// =============================================================
// FILE: process_system.cpp
// =============================================================
//
// SYSTEM Process — Teensy-side system truth surface
//
// SYSTEM is not a lifecycle-managed process.
// It cannot start or stop from within itself.
//
// It exposes authoritative system facts, owns bounded always-on integrity
// watchdog scheduling, and provides explicit terminal transitions (shutdown,
// bootloader) that represent irreversible boundary crossings.
//
// =============================================================

#include "process_system.h"
#include "memory_info.h"
#include "crash_forensics.h"
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
static constexpr uint64_t SYSTEM_MEMORY_WATCHDOG_PERIOD_NS = 5000000000ULL;

// --------------------------------------------------------------
// Forward declarations (terminal paths)
// --------------------------------------------------------------
void system_enter_quiescence(void);

// Bootloader entry symbol (ROM-provided, never returns)
extern "C" void enter_bootloader_cleanly(void);

static FLASHMEM Payload system_features_tree_payload(void);
static void system_feature_schedule_fragment_publish(void);
static void system_memory_watchdog_arm(void);
static void system_memory_watchdog_schedule_alap(void);
static void system_dmamem_ensure_initialized(void);
static void system_sentinel_service(void);

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
// Retained foreground / CPU-usage crash ledger
// ============================================================================
//
// This is deliberately tiny and scalar-only.  The live RAM2 record is updated
// by loop() and cpu_usage_tick(); on the next boot its final committed image is
// copied to the retained bank before current activity overwrites it.  Ordinary
// phase notes remain cache-local; CPU_USAGE entry flushes the whole record, so
// the 0x426 crash path commits its phase without adding another DWT read or a
// cache-maintenance operation to every loop transition.

static constexpr uint32_t ZPNET_RUNTIME_LEDGER_MAGIC = 0x52554E31UL;  // 'RUN1'
static constexpr uint32_t ZPNET_CPU_USAGE_STATE_IDLE = 0U;
static constexpr uint32_t ZPNET_CPU_USAGE_STATE_ACTIVE = 1U;

typedef struct {
  uint32_t magic;
  uint32_t magic_inv;

  uint32_t foreground_sequence;
  uint32_t foreground_sequence_inv;
  uint32_t foreground_phase;

  uint32_t cpu_usage_sequence;
  uint32_t cpu_usage_sequence_inv;
  uint32_t cpu_usage_state;
  uint32_t cpu_usage_state_inv;
  uint32_t cpu_usage_stage;
  uint32_t cpu_usage_stage_inv;
  uint32_t cpu_usage_stage_value;
  uint32_t cpu_usage_stage_value_inv;
  uint32_t cpu_usage_foreground_sequence;
  uint32_t cpu_usage_foreground_phase;
  uint32_t cpu_usage_ipsr;
} zpnet_runtime_ledger_t;

static_assert(sizeof(zpnet_runtime_ledger_t) == 64U,
              "runtime crash ledger must stay two cache lines");

static zpnet_runtime_ledger_t g_runtime_ledger DMAMEM;
static zpnet_runtime_ledger_t g_runtime_ledger_retained DMAMEM;
static bool g_runtime_ledger_boot_latched = false;  // BSS: zero every boot

volatile bool g_zpnet_sentinel_cpu_usage_focus_active = false;

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

static bool runtime_ledger_cpu_usage_valid(
    const zpnet_runtime_ledger_t* ledger) {
  return runtime_ledger_header_valid(ledger) &&
         (ledger->cpu_usage_sequence ^ ledger->cpu_usage_sequence_inv) ==
             0xFFFFFFFFUL &&
         (ledger->cpu_usage_state ^ ledger->cpu_usage_state_inv) ==
             0xFFFFFFFFUL &&
         (ledger->cpu_usage_stage ^ ledger->cpu_usage_stage_inv) ==
             0xFFFFFFFFUL &&
         (ledger->cpu_usage_stage_value ^
          ledger->cpu_usage_stage_value_inv) == 0xFFFFFFFFUL;
}

static void runtime_ledger_initialize_live(void) {
  memset((void*)&g_runtime_ledger, 0, sizeof(g_runtime_ledger));
  g_runtime_ledger.foreground_sequence_inv = 0xFFFFFFFFUL;
  g_runtime_ledger.cpu_usage_sequence_inv = 0xFFFFFFFFUL;
  g_runtime_ledger.cpu_usage_state = ZPNET_CPU_USAGE_STATE_IDLE;
  g_runtime_ledger.cpu_usage_state_inv = ~ZPNET_CPU_USAGE_STATE_IDLE;
  g_runtime_ledger.cpu_usage_stage =
      (uint32_t)zpnet_cpu_usage_stage_t::IDLE;
  g_runtime_ledger.cpu_usage_stage_inv =
      ~(uint32_t)zpnet_cpu_usage_stage_t::IDLE;
  g_runtime_ledger.cpu_usage_stage_value = 0U;
  g_runtime_ledger.cpu_usage_stage_value_inv = ~0U;
  g_runtime_ledger.magic_inv = ~ZPNET_RUNTIME_LEDGER_MAGIC;
  g_runtime_ledger.magic = ZPNET_RUNTIME_LEDGER_MAGIC;
  arm_dcache_flush((void*)&g_runtime_ledger, sizeof(g_runtime_ledger));
}

static void runtime_ledger_boot_latch(void) {
  if (g_runtime_ledger_boot_latched) return;
  g_runtime_ledger_boot_latched = true;

  if (runtime_ledger_header_valid(&g_runtime_ledger)) {
    g_runtime_ledger_retained = g_runtime_ledger;
  } else {
    memset((void*)&g_runtime_ledger_retained, 0,
           sizeof(g_runtime_ledger_retained));
  }
  arm_dcache_flush((void*)&g_runtime_ledger_retained,
                   sizeof(g_runtime_ledger_retained));

  runtime_ledger_initialize_live();
}

static inline uint32_t runtime_ledger_ipsr(void) {
  uint32_t ipsr = 0;
  __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
  return ipsr & 0x1FFU;
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

static void runtime_ledger_cpu_usage_set_stage(
    zpnet_cpu_usage_stage_t stage,
    uint32_t value) {
  const uint32_t stage_id = (uint32_t)stage;
  g_runtime_ledger.cpu_usage_stage = stage_id;
  g_runtime_ledger.cpu_usage_stage_inv = ~stage_id;
  g_runtime_ledger.cpu_usage_stage_value = value;
  g_runtime_ledger.cpu_usage_stage_value_inv = ~value;
}

void zpnet_cpu_usage_ledger_enter(void) {
  runtime_ledger_boot_latch();

  const uint32_t sequence = g_runtime_ledger.cpu_usage_sequence + 1U;
  g_runtime_ledger.cpu_usage_sequence_inv =
      g_runtime_ledger.cpu_usage_sequence;  // invalidate while mutating
  g_runtime_ledger.cpu_usage_foreground_sequence =
      g_runtime_ledger.foreground_sequence;
  g_runtime_ledger.cpu_usage_foreground_phase =
      g_runtime_ledger.foreground_phase;
  g_runtime_ledger.cpu_usage_ipsr = runtime_ledger_ipsr();
  g_runtime_ledger.cpu_usage_state = ZPNET_CPU_USAGE_STATE_ACTIVE;
  g_runtime_ledger.cpu_usage_state_inv = ~ZPNET_CPU_USAGE_STATE_ACTIVE;
  runtime_ledger_cpu_usage_set_stage(
      zpnet_cpu_usage_stage_t::CALLBACK_ENTER, 0U);
  g_runtime_ledger.cpu_usage_sequence_inv = ~sequence;
  g_runtime_ledger.cpu_usage_sequence = sequence;  // commit last
  arm_dcache_flush((void*)&g_runtime_ledger, sizeof(g_runtime_ledger));
}

void zpnet_cpu_usage_ledger_stage(zpnet_cpu_usage_stage_t stage,
                                  uint32_t value) {
  runtime_ledger_boot_latch();

  const uint32_t sequence = g_runtime_ledger.cpu_usage_sequence;
  g_runtime_ledger.cpu_usage_sequence_inv = sequence;  // invalidate
  runtime_ledger_cpu_usage_set_stage(stage, value);
  g_runtime_ledger.cpu_usage_sequence_inv = ~sequence;  // commit last
  arm_dcache_flush((void*)&g_runtime_ledger, sizeof(g_runtime_ledger));
}

void zpnet_cpu_usage_ledger_exit(void) {
  runtime_ledger_boot_latch();

  const uint32_t sequence = g_runtime_ledger.cpu_usage_sequence;
  g_runtime_ledger.cpu_usage_sequence_inv = sequence;  // invalidate
  g_runtime_ledger.cpu_usage_state = ZPNET_CPU_USAGE_STATE_IDLE;
  g_runtime_ledger.cpu_usage_state_inv = ~ZPNET_CPU_USAGE_STATE_IDLE;
  runtime_ledger_cpu_usage_set_stage(
      zpnet_cpu_usage_stage_t::CALLBACK_EXIT, 0U);
  g_runtime_ledger.cpu_usage_sequence_inv = ~sequence;  // commit last
  arm_dcache_flush((void*)&g_runtime_ledger, sizeof(g_runtime_ledger));
}

// --------------------------------------------------------------
// Always-on memory integrity watchdog
// --------------------------------------------------------------
// memory_info owns observation, rule evaluation, and the sticky health verdict.
// SYSTEM owns only TimePop scheduling and the one WATCHDOG_ANOMALY emission.

static memory_health_t g_system_memory_watchdog_health DMAMEM = {};
static timepop_handle_t g_system_memory_watchdog_handle = TIMEPOP_INVALID_HANDLE;
static bool     g_system_memory_watchdog_armed = false;
static bool     g_system_memory_watchdog_event_emitted = false;
static uint32_t g_system_memory_watchdog_arm_attempts = 0;
static uint32_t g_system_memory_watchdog_arm_failures = 0;
static uint32_t g_system_memory_watchdog_callback_count = 0;
static uint32_t g_system_memory_watchdog_event_count = 0;
static int64_t  g_system_memory_watchdog_last_fire_gnss_ns = 0;
static uint32_t g_system_memory_watchdog_last_fire_dwt = 0;

// Timed callbacks may run in interrupt context.  Keep them allocator-free and
// defer all substantive audit/serialization/publication work to one ALAP
// foreground service.
static volatile bool g_system_memory_watchdog_alap_pending = false;
static volatile bool g_system_memory_watchdog_alap_armed = false;
static uint32_t g_system_memory_watchdog_alap_arm_count = 0;
static uint32_t g_system_memory_watchdog_alap_arm_failures = 0;
static uint32_t g_system_memory_watchdog_alap_run_count = 0;

// ============================================================================
// Always-on memory integrity watchdog
// ============================================================================

static FLASHMEM void system_memory_watchdog_emit(void) {
  // Stay inside Payload's eight inline entries.  If the anomaly itself is an
  // allocator failure, the watchdog transcript must not require a heap-backed
  // entry table merely to announce it.
  Payload ev;
  ev.add("schema", "SYSTEM_MEMORY_WATCHDOG_V1");
  ev.add("source", "SYSTEM_MEMORY_WATCHDOG");
  ev.add("reason", memory_health_reason_name(
      g_system_memory_watchdog_health.primary_reason));
  ev.add("reason_id", (uint32_t)g_system_memory_watchdog_health.primary_reason);
  ev.add("audit_count", g_system_memory_watchdog_health.audit_count);
  ev.add("reason_mask", g_system_memory_watchdog_health.latched_reason_mask);

  // Two reason-specific evidence fields keep the event compact while making
  // the first failure intelligible.  Full current state remains available
  // through SYSTEM.MEMORY_INFO and SYSTEM.PAYLOAD_INFO.
  switch (g_system_memory_watchdog_health.primary_reason) {
    case memory_health_reason_t::STACK_PAINT_UNAVAILABLE:
      ev.add("paint_compiled",
             g_system_memory_watchdog_health.stack_paint_compiled_enabled);
      ev.add("paint_enabled",
             g_system_memory_watchdog_health.stack_paint_enabled);
      break;

    case memory_health_reason_t::STACK_PAINT_OVERRUN:
    case memory_health_reason_t::STACK_RUNWAY_CRITICAL:
      ev.add("stack_free_high_water",
             g_system_memory_watchdog_health.stack_free_high_water);
      ev.add("stack_paint_overrun",
             g_system_memory_watchdog_health.stack_paint_overrun);
      break;

    case memory_health_reason_t::HEAP_MAP_INVALID:
    case memory_health_reason_t::HEAP_FREE_CRITICAL:
      ev.add("heap_free_total",
             g_system_memory_watchdog_health.heap_free_total);
      ev.add("heap_free_critical",
             g_system_memory_watchdog_health.heap_free_critical_bytes);
      break;

    case memory_health_reason_t::PAYLOAD_SELF_OK_FAIL:
      ev.add("payload_self_ok_fail",
             g_system_memory_watchdog_health.payload_self_ok_fail);
      ev.add("payload_self_reason",
             g_system_memory_watchdog_health.payload_last_self_ok_fail_reason);
      break;

    case memory_health_reason_t::PAYLOAD_STRING_POINTER_FAULT:
      ev.add("payload_pointer_fault",
             g_system_memory_watchdog_health.payload_string_pointer_fault);
      ev.add("payload_pointer_reason",
             g_system_memory_watchdog_health.payload_last_string_pointer_fault_reason);
      break;

    case memory_health_reason_t::PAYLOAD_LIFETIME_MISMATCH:
      ev.add("payload_constructed",
             g_system_memory_watchdog_health.payload_instances_constructed);
      ev.add("payload_destroyed",
             g_system_memory_watchdog_health.payload_instances_destroyed);
      break;

    default:
      ev.add("payload_last_error",
             g_system_memory_watchdog_health.payload_last_error_code);
      ev.add("heap_free_total",
             g_system_memory_watchdog_health.heap_free_total);
      break;
  }

  enqueueEvent("WATCHDOG_ANOMALY", ev);
  g_system_memory_watchdog_event_count++;
}

static FLASHMEM void system_memory_watchdog_alap_cb(timepop_ctx_t*,
                                                    timepop_diag_t*,
                                                    void*) {
  g_system_memory_watchdog_alap_armed = false;

  if (!g_system_memory_watchdog_alap_pending) {
    return;
  }

  g_system_memory_watchdog_alap_pending = false;
  g_system_memory_watchdog_alap_run_count++;

  // All allocator-, Payload-, and publication-bearing work belongs here in
  // serialized foreground context, never in the timed callback.
  memory_info_audit(&g_system_memory_watchdog_health);

  if (g_system_memory_watchdog_health.status ==
          memory_health_status_t::ANOMALY &&
      !g_system_memory_watchdog_event_emitted) {
    // Latch before constructing the event so a Payload failure during the
    // best-effort anomaly transcript cannot create a recursive event storm.
    g_system_memory_watchdog_event_emitted = true;
    system_memory_watchdog_emit();
  }

  // Frame-sentinel verdicts are announced here, in serialized foreground
  // context, never from the handler pass that detected them.  The 5-second
  // watchdog cadence bounds announcement latency; the evidence itself was
  // latched at detection time and is not perishable.
  system_sentinel_service();

  // If another timed fire arrived while this ALAP service was running, arrange
  // one more serialized pass rather than doing nested work here.
  if (g_system_memory_watchdog_alap_pending) {
    system_memory_watchdog_schedule_alap();
  }
}

static void system_memory_watchdog_schedule_alap(void) {
  if (g_system_memory_watchdog_alap_armed) {
    return;
  }

  const timepop_handle_t handle =
      timepop_arm_alap(system_memory_watchdog_alap_cb,
                       nullptr,
                       "SYSTEM_MEMORY_WATCHDOG_ALAP");
  if (handle == TIMEPOP_INVALID_HANDLE) {
    g_system_memory_watchdog_alap_arm_failures++;
    return;
  }

  g_system_memory_watchdog_alap_armed = true;
  g_system_memory_watchdog_alap_arm_count++;
}

static FLASHMEM void system_memory_watchdog_cb(timepop_ctx_t* ctx,
                                               timepop_diag_t*,
                                               void*) {
  // Timed/ISR path: scalar bookkeeping only.  No mallinfo(), Payload,
  // formatting, publish(), transport work, or heap interaction.
  g_system_memory_watchdog_callback_count++;
  if (ctx) {
    g_system_memory_watchdog_last_fire_gnss_ns = ctx->fire_gnss_ns;
    g_system_memory_watchdog_last_fire_dwt = ctx->fire_dwt_cyccnt;
  }

  g_system_memory_watchdog_alap_pending = true;
  system_memory_watchdog_schedule_alap();
}

static void system_memory_watchdog_arm(void) {
  system_dmamem_ensure_initialized();
  if (g_system_memory_watchdog_armed) return;

  g_system_memory_watchdog_arm_attempts++;
  g_system_memory_watchdog_handle = timepop_arm(
      SYSTEM_MEMORY_WATCHDOG_PERIOD_NS,
      true,
      system_memory_watchdog_cb,
      nullptr,
      "SYSTEM_MEMORY_WATCHDOG");

  if (g_system_memory_watchdog_handle == TIMEPOP_INVALID_HANDLE) {
    g_system_memory_watchdog_arm_failures++;
    return;
  }

  g_system_memory_watchdog_armed = true;
}

// ================================================================
// Exception-frame sentinel — core
// ================================================================
//
// Catch corruption of a stacked exception frame while the corrupting handler
// pass is still on the CPU, with attribution.  See process_system.h for the
// usage contract (ZPNET_SENTINEL_ENTER/EXIT placement rules).
//
// Frame location strategies, in evidence-quality order:
//
//   1. FPCAR anchor — EXC_RETURN reports an extended frame and FPCCR.LSPACT
//      is still pending, so FPCAR points at the reserved FP area and the
//      8-word basic frame sits at FPCAR − 0x20.  Deterministic for the
//      FP-active foreground that Crash1 implicates.
//   2. PSP — EXC_RETURN reports the process stack as the return stack.
//   3. Bounded signature scan — observational fallback only.  A candidate must
//      match EXC_RETURN's thread/handler return semantics and carry plausible
//      stacked PC, LR, and xPSR values.  SCAN changes are counted but cannot
//      latch a frame-corruption verdict in this crash-hunt build.
//
// Everything on the enter/exit path is scalar, allocator-free, bounded, and
// runs from ITCM (deliberately not FLASHMEM).  Verdicts are latched to a
// RAM2 NOLOAD record (magic+complement, dcache-flushed) so they survive the
// reboot that typically follows successful frame corruption.

extern "C" unsigned long _estack;

#define ZPNET_SCB_FPCCR (*(volatile uint32_t*)0xE000EF34UL)
#define ZPNET_SCB_FPCAR (*(volatile uint32_t*)0xE000EF38UL)

enum {
  ZPNET_SENTINEL_FRAME_NONE  = 0,
  ZPNET_SENTINEL_FRAME_FPCAR = 1,
  ZPNET_SENTINEL_FRAME_PSP   = 2,
  ZPNET_SENTINEL_FRAME_SCAN  = 3,
};

static const char* sentinel_frame_source_name(uint32_t source) {
  switch (source) {
    case ZPNET_SENTINEL_FRAME_FPCAR: return "FPCAR";
    case ZPNET_SENTINEL_FRAME_PSP:   return "PSP";
    case ZPNET_SENTINEL_FRAME_SCAN:  return "SCAN";
    default:                         return "NONE";
  }
}

// Violation kind bitmask
static constexpr uint32_t ZPNET_SENTINEL_KIND_FRAME = 0x1U;
static constexpr uint32_t ZPNET_SENTINEL_KIND_MSP   = 0x2U;

// Bounded upward scan window from the current MSP, in bytes.
static constexpr uint32_t ZPNET_SENTINEL_SCAN_BYTES = 512U;
static constexpr uint32_t ZPNET_SENTINEL_SCAN_MAX_EXCEPTION = 255U;
static constexpr bool ZPNET_SENTINEL_SCAN_VERDICTS_ENABLED = false;

typedef struct {
  volatile uint32_t active;
  const char* name;
  uint32_t pass;
  uint32_t exc_return;
  uint32_t frame_address;
  uint32_t frame_source;
  uint32_t entry_words[8];
  uint32_t entry_checksum;
  uint32_t msp_entry;
  uint32_t fpccr_entry;
  uint32_t fpcar_entry;
  uint32_t dwt_entry;
  uint32_t ipsr;
  // Last completed pass
  uint32_t msp_exit_last;
  uint32_t exit_checksum_last;
  uint32_t dwt_exit_last;
  // Per-slot verdict counters
  uint32_t frames_located;
  uint32_t frames_missing;
  uint32_t thread_mode_passes;
  uint32_t exc_return_implausible;
  uint32_t checksum_fail;
  uint32_t scan_change_ignored;
  uint32_t msp_mismatch;
  uint32_t lspact_consumed;
} zpnet_sentinel_slot_t;

// Ordinary BSS: per-boot state, guaranteed zeroed by startup.
static zpnet_sentinel_slot_t g_sentinel_slots[ZPNET_SENTINEL_SLOT_COUNT];

static uint32_t g_sentinel_enters = 0;
static uint32_t g_sentinel_exits = 0;
static uint32_t g_sentinel_bad_slot = 0;
static uint32_t g_sentinel_unbalanced_enter = 0;
static uint32_t g_sentinel_unbalanced_exit = 0;
static uint32_t g_sentinel_frames_fpcar = 0;
static uint32_t g_sentinel_frames_psp = 0;
static uint32_t g_sentinel_frames_scan = 0;
static uint32_t g_sentinel_frames_none = 0;
static uint32_t g_sentinel_scan_changes_ignored = 0;
static uint32_t g_sentinel_violations_this_boot = 0;
static uint32_t g_sentinel_violations_suppressed = 0;
static volatile bool g_sentinel_event_pending = false;
static bool g_sentinel_event_emitted = false;

static constexpr uint32_t ZPNET_SENTINEL_MAGIC = 0x53454E33UL;  // 'SEN3'

typedef struct {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t kind;
  uint32_t slot;
  char     name[24];
  uint32_t pass;
  uint32_t frame_address;
  uint32_t frame_source;
  uint32_t exc_return;
  uint32_t ipsr;
  uint32_t msp_entry;
  uint32_t msp_exit;
  uint32_t fpccr_entry;
  uint32_t fpccr_exit;
  uint32_t fpcar_entry;
  uint32_t fpcar_exit;
  uint32_t dwt_entry;
  uint32_t dwt_exit;
  uint32_t entry_words[8];
  uint32_t exit_words[8];
  uint32_t diff_mask;
  uint32_t entry_checksum;
  uint32_t exit_checksum;
  // Foreground correlation: the outermost sentinel region that was active
  // when the violation latched (typically the thread-mode victim region the
  // corrupting handler pass preempted).
  uint32_t foreground_region_depth;
  uint32_t foreground_region_slot;
  uint32_t foreground_region_pass;
  char     foreground_region_name[24];
} zpnet_sentinel_violation_t;

// Retained verdict — RAM2 NOLOAD, first violation latched until CRASH_CLEAR.
// Deliberately excluded from system_dmamem_ensure_initialized(): like the
// crash_forensics record, it may be the only surviving witness to a reboot.
static zpnet_sentinel_violation_t g_sentinel_violation_retained DMAMEM;

// This boot's most recent violation (overwritten each time); BSS.
static zpnet_sentinel_violation_t g_sentinel_violation_last;
static bool g_sentinel_violation_last_present = false;

static bool sentinel_retained_valid(void) {
  const bool header_valid =
      g_sentinel_violation_retained.magic == ZPNET_SENTINEL_MAGIC &&
      (g_sentinel_violation_retained.magic ^
       g_sentinel_violation_retained.magic_inv) == 0xFFFFFFFFUL;
  if (!header_valid) return false;

  // Earlier builds allowed heuristic SCAN candidates to latch frame-only
  // verdicts.  The live evidence proved those were ordinary stack reuse.
  // Do not let a stale false verdict occupy the first-latch bank forever.
  if (!ZPNET_SENTINEL_SCAN_VERDICTS_ENABLED &&
      g_sentinel_violation_retained.frame_source ==
          ZPNET_SENTINEL_FRAME_SCAN &&
      (g_sentinel_violation_retained.kind & ZPNET_SENTINEL_KIND_MSP) == 0U) {
    return false;
  }
  return true;
}

static inline uint32_t sentinel_read_msp(void) {
  uint32_t msp;
  __asm__ volatile("mrs %0, msp" : "=r"(msp));
  return msp;
}

static inline uint32_t sentinel_read_psp(void) {
  uint32_t psp;
  __asm__ volatile("mrs %0, psp" : "=r"(psp));
  return psp;
}

static inline uint32_t sentinel_read_ipsr(void) {
  uint32_t ipsr;
  __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
  return ipsr & 0x1FFU;
}

static inline bool sentinel_pc_plausible(uint32_t pc) {
  if ((pc & 1U) != 0U) return false;
  if (pc >= 0x00000400UL && pc < 0x00080000UL) return true;  // runtime ITCM
  if (pc >= 0x60000000UL && pc < 0x61000000UL) return true;  // QSPI flash map
  return false;
}

static inline bool sentinel_exc_return_plausible(uint32_t value) {
  return (value & 0xFFFFFF00UL) == 0xFFFFFF00UL &&
         (value & 1U) != 0U;
}

static inline bool sentinel_stacked_lr_plausible(uint32_t lr) {
  if (sentinel_exc_return_plausible(lr)) return true;
  if ((lr & 1U) == 0U) return false;
  return sentinel_pc_plausible(lr & ~1U);
}

static inline bool sentinel_stacked_xpsr_plausible(uint32_t xpsr,
                                                    uint32_t exc_return) {
  if ((xpsr & (1UL << 24)) == 0U) return false;

  const uint32_t stacked_exception = xpsr & 0x1FFU;
  const bool return_to_thread = (exc_return & (1UL << 3)) != 0U;
  if (return_to_thread) return stacked_exception == 0U;
  return stacked_exception != 0U &&
         stacked_exception <= ZPNET_SENTINEL_SCAN_MAX_EXCEPTION;
}

static uint32_t sentinel_checksum(const uint32_t* words) {
  // FNV-1a style word fold — cheap, order-sensitive, table-free.
  uint32_t h = 0x811C9DC5UL;
  for (int i = 0; i < 8; ++i) {
    h ^= words[i];
    h *= 0x01000193UL;
  }
  return h;
}

static void sentinel_copy_name(char* dst, size_t dst_size, const char* src) {
  size_t i = 0;
  if (!dst || dst_size == 0U) return;
  if (src) {
    for (; i + 1U < dst_size && src[i] != '\0'; ++i) dst[i] = src[i];
  }
  dst[i] = '\0';
}

// ----------------------------------------------------------------
// Region beacon — "what was the firmware doing when it died?"
// ----------------------------------------------------------------
//
// Every sentinel scope (handler pass or thread-mode victim region) is pushed
// onto a small retained stack at ENTER and popped at EXIT.  The beacon lives
// in RAM2 (NOLOAD, magic-validated, dcache-flushed), so after a crash the
// previous boot's beacon answers directly: which control points were active
// at the moment of death — e.g. "SYSTEM_REPORT in thread mode, preempted by
// a HANDOFF pass that was mid-TIMEBASE_FRAGMENT".  The same snapshot is
// folded into every latched violation as the foreground correlation.
//
// Push/pop guard their read-modify-write with a PRIMASK critical section a
// few instructions long, because a handler-mode scope can preempt a
// thread-mode scope mid-update.

static constexpr uint32_t ZPNET_SENTINEL_BEACON_MAGIC = 0x53424E31UL;  // 'SBN1'
static constexpr uint32_t ZPNET_SENTINEL_BEACON_DEPTH = 4U;

typedef struct {
  uint32_t slot;
  uint32_t pass;
  uint32_t dwt_enter;
  uint32_t ipsr;
  char     name[24];
} zpnet_sentinel_beacon_entry_t;

typedef struct {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t depth;     // active nested sentinel scopes
  uint32_t overflow;  // enters observed beyond stack capacity
  zpnet_sentinel_beacon_entry_t active[ZPNET_SENTINEL_BEACON_DEPTH];
  zpnet_sentinel_beacon_entry_t last_completed;
  uint32_t last_completed_dwt_exit;
} zpnet_sentinel_beacon_t;

// NOLOAD retained pair, latched once per boot exactly like the Payload
// flight rings: the surviving live beacon (the crash-moment truth) is copied
// into the retained bank before this boot's first scope overwrites it.
static zpnet_sentinel_beacon_t g_sentinel_beacon DMAMEM;
static zpnet_sentinel_beacon_t g_sentinel_beacon_retained DMAMEM;
static bool g_sentinel_beacon_boot_latched = false;  // BSS: zero every boot

static bool sentinel_beacon_valid(const zpnet_sentinel_beacon_t* b) {
  return b->magic == ZPNET_SENTINEL_BEACON_MAGIC &&
         (b->magic ^ b->magic_inv) == 0xFFFFFFFFUL;
}

static void sentinel_beacon_boot_latch(void) {
  if (g_sentinel_beacon_boot_latched) return;
  g_sentinel_beacon_boot_latched = true;

  if (sentinel_beacon_valid(&g_sentinel_beacon)) {
    g_sentinel_beacon_retained = g_sentinel_beacon;
  } else {
    memset((void*)&g_sentinel_beacon_retained, 0,
           sizeof(g_sentinel_beacon_retained));
  }
  arm_dcache_flush((void*)&g_sentinel_beacon_retained,
                   sizeof(g_sentinel_beacon_retained));

  memset((void*)&g_sentinel_beacon, 0, sizeof(g_sentinel_beacon));
  g_sentinel_beacon.magic_inv = ~ZPNET_SENTINEL_BEACON_MAGIC;
  g_sentinel_beacon.magic = ZPNET_SENTINEL_BEACON_MAGIC;
  arm_dcache_flush((void*)&g_sentinel_beacon, sizeof(g_sentinel_beacon));
}

static inline uint32_t sentinel_irq_lock(void) {
  uint32_t primask;
  __asm__ volatile("mrs %0, primask" : "=r"(primask));
  __asm__ volatile("cpsid i" ::: "memory");
  return primask;
}

static inline void sentinel_irq_unlock(uint32_t primask) {
  if ((primask & 1U) == 0U) {
    __asm__ volatile("cpsie i" ::: "memory");
  }
}

static void sentinel_beacon_push(uint32_t slot,
                                 const char* name,
                                 uint32_t pass,
                                 uint32_t ipsr,
                                 uint32_t dwt) {
  sentinel_beacon_boot_latch();

  const uint32_t primask = sentinel_irq_lock();
  const uint32_t d = g_sentinel_beacon.depth;
  if (d < ZPNET_SENTINEL_BEACON_DEPTH) {
    zpnet_sentinel_beacon_entry_t* e = &g_sentinel_beacon.active[d];
    e->slot = slot;
    e->pass = pass;
    e->dwt_enter = dwt;
    e->ipsr = ipsr;
    sentinel_copy_name(e->name, sizeof(e->name), name);
  } else {
    g_sentinel_beacon.overflow++;
  }
  g_sentinel_beacon.depth = d + 1U;
  sentinel_irq_unlock(primask);

  arm_dcache_flush((void*)&g_sentinel_beacon, sizeof(g_sentinel_beacon));
}

static void sentinel_beacon_pop(uint32_t dwt_exit) {
  sentinel_beacon_boot_latch();

  const uint32_t primask = sentinel_irq_lock();
  uint32_t d = g_sentinel_beacon.depth;
  if (d != 0U) {
    d--;
    if (d < ZPNET_SENTINEL_BEACON_DEPTH) {
      g_sentinel_beacon.last_completed = g_sentinel_beacon.active[d];
      g_sentinel_beacon.last_completed_dwt_exit = dwt_exit;
      memset((void*)&g_sentinel_beacon.active[d], 0,
             sizeof(g_sentinel_beacon.active[d]));
    }
    g_sentinel_beacon.depth = d;
  }
  sentinel_irq_unlock(primask);

  arm_dcache_flush((void*)&g_sentinel_beacon, sizeof(g_sentinel_beacon));
}

static void sentinel_fill_violation(zpnet_sentinel_violation_t* v,
                                    const zpnet_sentinel_slot_t* s,
                                    uint32_t slot,
                                    uint32_t kind,
                                    const uint32_t* exit_words,
                                    uint32_t diff_mask,
                                    uint32_t exit_checksum,
                                    uint32_t msp_exit,
                                    uint32_t fpccr_exit,
                                    uint32_t fpcar_exit,
                                    uint32_t dwt_exit) {
  v->magic = 0U;  // invalidate while the record is inconsistent
  v->kind = kind;
  v->slot = slot;
  sentinel_copy_name(v->name, sizeof(v->name), s->name);
  v->pass = s->pass;
  v->frame_address = s->frame_address;
  v->frame_source = s->frame_source;
  v->exc_return = s->exc_return;
  v->ipsr = s->ipsr;
  v->msp_entry = s->msp_entry;
  v->msp_exit = msp_exit;
  v->fpccr_entry = s->fpccr_entry;
  v->fpccr_exit = fpccr_exit;
  v->fpcar_entry = s->fpcar_entry;
  v->fpcar_exit = fpcar_exit;
  v->dwt_entry = s->dwt_entry;
  v->dwt_exit = dwt_exit;
  for (int i = 0; i < 8; ++i) {
    v->entry_words[i] = s->entry_words[i];
    v->exit_words[i] = exit_words ? exit_words[i] : 0U;
  }
  v->diff_mask = diff_mask;
  v->entry_checksum = s->entry_checksum;
  v->exit_checksum = exit_checksum;

  // Foreground correlation: entries[0] is the outermost active sentinel
  // scope — typically the thread-mode victim region the violating handler
  // pass preempted.
  v->foreground_region_depth = g_sentinel_beacon.depth;
  if (g_sentinel_beacon.depth != 0U) {
    const zpnet_sentinel_beacon_entry_t* fg = &g_sentinel_beacon.active[0];
    v->foreground_region_slot = fg->slot;
    v->foreground_region_pass = fg->pass;
    sentinel_copy_name(v->foreground_region_name,
                       sizeof(v->foreground_region_name),
                       fg->name);
  } else {
    v->foreground_region_slot = 0U;
    v->foreground_region_pass = 0U;
    v->foreground_region_name[0] = '\0';
  }

  v->magic_inv = ~ZPNET_SENTINEL_MAGIC;
  v->magic = ZPNET_SENTINEL_MAGIC;  // commit last
}

static void sentinel_latch_violation(const zpnet_sentinel_slot_t* s,
                                     uint32_t slot,
                                     uint32_t kind,
                                     const uint32_t* exit_words,
                                     uint32_t diff_mask,
                                     uint32_t exit_checksum,
                                     uint32_t msp_exit,
                                     uint32_t fpccr_exit,
                                     uint32_t fpcar_exit,
                                     uint32_t dwt_exit) {
  g_sentinel_violations_this_boot++;
  g_sentinel_event_pending = true;

  // This boot's most recent violation is always recorded.
  sentinel_fill_violation(&g_sentinel_violation_last, s, slot, kind,
                          exit_words, diff_mask, exit_checksum, msp_exit,
                          fpccr_exit, fpcar_exit, dwt_exit);
  g_sentinel_violation_last_present = true;

  // The retained record follows first-latch policy: earlier evidence is
  // more valuable than later, and it holds until explicit CRASH_CLEAR.
  if (sentinel_retained_valid()) {
    g_sentinel_violations_suppressed++;
    return;
  }
  sentinel_fill_violation(&g_sentinel_violation_retained, s, slot, kind,
                          exit_words, diff_mask, exit_checksum, msp_exit,
                          fpccr_exit, fpcar_exit, dwt_exit);

  // RAM2 is write-back cached and a reset can discard dirty lines; flush so
  // the physical retained image matches the record just committed.
  arm_dcache_flush((void*)&g_sentinel_violation_retained,
                   sizeof(g_sentinel_violation_retained));
}

void zpnet_sentinel_enter(uint32_t slot,
                          const char* name,
                          uint32_t exc_return,
                          uint32_t caller_sp) {
  if (slot >= (uint32_t)ZPNET_SENTINEL_SLOT_COUNT) {
    g_sentinel_bad_slot++;
    return;
  }
  zpnet_sentinel_slot_t* s = &g_sentinel_slots[slot];
  if (s->active) g_sentinel_unbalanced_enter++;
  s->active = 1U;
  s->pass++;
  s->name = name;
  g_sentinel_enters++;

  const uint32_t msp = sentinel_read_msp();
  const uint32_t psp = sentinel_read_psp();
  const uint32_t fpccr = ZPNET_SCB_FPCCR;
  const uint32_t fpcar = ZPNET_SCB_FPCAR;

  s->exc_return = exc_return;
  // MSP integrity uses the SP captured by the macro at the caller's own
  // frame level.  Reading MSP inside this function would bake the differing
  // prologue depths of enter() and exit() into the comparison and fire a
  // false MSP_MISMATCH on every pass (the validation-run lesson).
  s->msp_entry = caller_sp;
  s->fpccr_entry = fpccr;
  s->fpcar_entry = fpcar;
  s->dwt_entry = ARM_DWT_CYCCNT;
  s->ipsr = sentinel_read_ipsr();

  // Every scope — handler pass or thread-mode victim region — announces
  // itself on the retained beacon so a crash mid-scope is attributable
  // after the reboot.
  sentinel_beacon_push(slot, name, s->pass, s->ipsr, s->dwt_entry);

  // EXC_RETURN values are 0xFFFFFFxx.  Anything else means the macro's LR
  // capture was already clobbered — expected whenever ENTER sits one or more
  // calls below the true handler entry (the beta control points live inside
  // alpha's PPS handoff) and always in thread mode.  IPSR, read directly, is
  // the authoritative context witness, so a failed capture only removes the
  // PSP strategy, never the verdict.  Counted only in handler mode; in a
  // thread-mode scope the capture is meaningless by construction.
  const bool exc_plausible = sentinel_exc_return_plausible(exc_return);
  if (!exc_plausible && s->ipsr != 0U) s->exc_return_implausible++;

  const uint32_t dtcm_end = (uint32_t)(uintptr_t)&_estack;
  uint32_t frame = 0U;
  uint32_t source = ZPNET_SENTINEL_FRAME_NONE;

  if (s->ipsr != 0U) {
    // 1. FPCAR anchor: lazy FP stacking still pending means an extended
    //    frame was reserved for the most recently preempted FP-active
    //    context, and FPCAR addresses its FP area, 0x20 above the basic
    //    frame.  Keyed on IPSR + LSPACT alone — architecturally valid no
    //    matter how many calls deep this scope sits, and deterministic for
    //    the FP-active foreground Crash1 implicates.
    if ((fpccr & 0x1U) != 0U) {
      const uint32_t candidate = fpcar - 0x20U;
      if (candidate >= 0x20000000UL && candidate < dtcm_end &&
          (candidate & 3U) == 0U) {
        frame = candidate;
        source = ZPNET_SENTINEL_FRAME_FPCAR;
      }
    }

    // 2. PSP: the preempted context stacked onto the process stack.
    if (frame == 0U && exc_plausible && (exc_return & 0x4U) != 0U) {
      if (psp >= 0x20000000UL && psp < dtcm_end && (psp & 3U) == 0U) {
        frame = psp;
        source = ZPNET_SENTINEL_FRAME_PSP;
      }
    }

    // 3. Bounded signature scan up the main stack.  Start at the caller's
    //    frame level when the macro supplied it — the frames of interest sit
    //    above the caller, and this keeps the sentinel's own locals out of
    //    the scan window.
    if (frame == 0U && exc_plausible) {
      const uint32_t scan_from =
          (caller_sp >= 0x20000000UL && caller_sp < dtcm_end) ? caller_sp
                                                              : msp;
      const uint32_t begin = (scan_from + 7U) & ~7U;
      for (uint32_t base = begin;
           base + 32U <= dtcm_end && base < begin + ZPNET_SENTINEL_SCAN_BYTES;
           base += 8U) {
        const volatile uint32_t* w = (const volatile uint32_t*)(uintptr_t)base;
        const uint32_t lr = w[5];
        const uint32_t pc = w[6];
        const uint32_t xpsr = w[7];
        if (sentinel_pc_plausible(pc) &&
            sentinel_stacked_lr_plausible(lr) &&
            sentinel_stacked_xpsr_plausible(xpsr, exc_return)) {
          frame = base;
          source = ZPNET_SENTINEL_FRAME_SCAN;
          break;
        }
      }
    }
  } else {
    // Thread mode: there is no live stacked frame that belongs to this
    // scope.  Anything a scan would find is dead stack memory that every
    // subsequent exception legally rewrites, so checksumming it would
    // manufacture false violations.  The scope still gets MSP integrity,
    // pass accounting, and the beacon.
    s->thread_mode_passes++;
  }

  s->frame_address = frame;
  s->frame_source = source;
  if (frame != 0U) {
    s->frames_located++;
    const volatile uint32_t* w = (const volatile uint32_t*)(uintptr_t)frame;
    for (int i = 0; i < 8; ++i) s->entry_words[i] = w[i];
    s->entry_checksum = sentinel_checksum(s->entry_words);
    switch (source) {
      case ZPNET_SENTINEL_FRAME_FPCAR: g_sentinel_frames_fpcar++; break;
      case ZPNET_SENTINEL_FRAME_PSP:   g_sentinel_frames_psp++;   break;
      default:                         g_sentinel_frames_scan++;  break;
    }
  } else if (s->ipsr != 0U) {
    s->frames_missing++;
    g_sentinel_frames_none++;
  }
}

void zpnet_sentinel_exit(uint32_t slot, uint32_t caller_sp) {
  if (slot >= (uint32_t)ZPNET_SENTINEL_SLOT_COUNT) {
    g_sentinel_bad_slot++;
    return;
  }
  zpnet_sentinel_slot_t* s = &g_sentinel_slots[slot];
  g_sentinel_exits++;
  if (!s->active) {
    g_sentinel_unbalanced_exit++;
    return;
  }
  s->active = 0U;

  const uint32_t fpccr = ZPNET_SCB_FPCCR;
  const uint32_t fpcar = ZPNET_SCB_FPCAR;
  const uint32_t dwt = ARM_DWT_CYCCNT;

  s->msp_exit_last = caller_sp;
  s->dwt_exit_last = dwt;
  // Lazy FP state consumed during this pass: an FP instruction executed in
  // handler context and flushed the foreground's FP registers into the
  // reserved stack area.  Legal — but it is exactly the lazy-stacking
  // choreography under suspicion, so count every occurrence.
  if ((s->fpccr_entry & 0x1U) != 0U && (fpccr & 0x1U) == 0U) {
    s->lspact_consumed++;
  }

  uint32_t kind = 0U;
  uint32_t exit_words[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint32_t exit_checksum = 0U;
  uint32_t diff_mask = 0U;

  if (s->frame_address != 0U) {
    const volatile uint32_t* w =
        (const volatile uint32_t*)(uintptr_t)s->frame_address;
    for (int i = 0; i < 8; ++i) exit_words[i] = w[i];
    exit_checksum = sentinel_checksum(exit_words);
    s->exit_checksum_last = exit_checksum;
    for (int i = 0; i < 8; ++i) {
      if (exit_words[i] != s->entry_words[i]) diff_mask |= (1U << (uint32_t)i);
    }
    if (diff_mask != 0U) {
      if (s->frame_source == ZPNET_SENTINEL_FRAME_SCAN &&
          !ZPNET_SENTINEL_SCAN_VERDICTS_ENABLED) {
        s->scan_change_ignored++;
        g_sentinel_scan_changes_ignored++;
      } else {
        kind |= ZPNET_SENTINEL_KIND_FRAME;
        s->checksum_fail++;
      }
    }
  }

  // Like-for-like comparison: both values were captured by the macros at
  // the caller's frame level.  A zero on either side means a capture was
  // unavailable (non-ARM build path) — no verdict from missing evidence.
  if (caller_sp != 0U && s->msp_entry != 0U && caller_sp != s->msp_entry) {
    kind |= ZPNET_SENTINEL_KIND_MSP;
    s->msp_mismatch++;
  }

  if (kind != 0U) {
    sentinel_latch_violation(s, slot, kind, exit_words, diff_mask,
                             exit_checksum, caller_sp, fpccr, fpcar, dwt);
  }

  // Pop after any latch so the violation's foreground correlation still saw
  // this scope (and its ancestors) as active.
  sentinel_beacon_pop(dwt);
}

static FLASHMEM void system_sentinel_emit(void) {
  // Stay inside Payload's eight inline entries (see the memory watchdog
  // emit rationale): the anomaly transcript must not require heap.
  const zpnet_sentinel_violation_t* v = &g_sentinel_violation_last;
  Payload ev;
  ev.add("schema", "ZPNET_SENTINEL_ANOMALY_V1");
  ev.add("name", v->name);
  ev.add("kind", v->kind);
  ev.add("pass", v->pass);
  ev.add_fmt("frame", "0x%08lX", (unsigned long)v->frame_address);
  ev.add_fmt("diff_mask", "0x%02lX", (unsigned long)v->diff_mask);
  ev.add("source", sentinel_frame_source_name(v->frame_source));
  ev.add("ipsr", v->ipsr);
  enqueueEvent("SENTINEL_ANOMALY", ev);
}

static void system_sentinel_service(void) {
  // Serialized foreground context only (watchdog ALAP).  One announcement
  // per boot; SENTINEL_INFO carries the full evidence at any time.
  if (g_sentinel_event_pending && !g_sentinel_event_emitted &&
      g_sentinel_violation_last_present) {
    g_sentinel_event_emitted = true;
    g_sentinel_event_pending = false;
    system_sentinel_emit();
  }
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

static FLASHMEM Payload system_crash_forensics_payload(void) {
  crash_forensics_status_t status{};
  crash_forensics_get_status(&status);

  Payload out;
  out.add("schema", "ZPNET_CRASH_FORENSICS_V1");
  out.add("installed_now", status.installed);
  out.add("present", status.present);
  out.add("header_valid", status.header_valid);
  out.add("crc_valid", status.crc_valid);
  system_crash_add_hex32(out, "stored_crc", status.stored_crc);
  system_crash_add_hex32(out, "computed_crc", status.computed_crc);

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

static constexpr size_t SYSTEM_CRASH_REPORT_TEXT_MAX = 2048;

// ================================================================
// Exception-frame sentinel — reporting surface
// ================================================================

static FLASHMEM Payload system_sentinel_violation_payload(
    const zpnet_sentinel_violation_t* v,
    bool valid) {
  Payload out;
  out.add("valid", valid);
  if (!valid) return out;

  out.add("kind", v->kind);
  out.add("kind_frame_checksum",
          (v->kind & ZPNET_SENTINEL_KIND_FRAME) != 0U);
  out.add("kind_msp_mismatch",
          (v->kind & ZPNET_SENTINEL_KIND_MSP) != 0U);
  out.add("slot", v->slot);
  out.add("name", v->name);
  out.add("pass", v->pass);
  system_crash_add_hex32(out, "frame_address", v->frame_address);
  out.add("frame_source", sentinel_frame_source_name(v->frame_source));
  system_crash_add_hex32(out, "exc_return", v->exc_return);
  out.add("ipsr", v->ipsr);
  system_crash_add_hex32(out, "msp_entry", v->msp_entry);
  system_crash_add_hex32(out, "msp_exit", v->msp_exit);
  system_crash_add_hex32(out, "fpccr_entry", v->fpccr_entry);
  system_crash_add_hex32(out, "fpccr_exit", v->fpccr_exit);
  system_crash_add_hex32(out, "fpcar_entry", v->fpcar_entry);
  system_crash_add_hex32(out, "fpcar_exit", v->fpcar_exit);
  out.add("dwt_entry", v->dwt_entry);
  out.add("dwt_exit", v->dwt_exit);
  system_crash_add_hex32(out, "diff_mask", v->diff_mask);
  system_crash_add_hex32(out, "entry_checksum", v->entry_checksum);
  system_crash_add_hex32(out, "exit_checksum", v->exit_checksum);

  Payload foreground;
  foreground.add("depth", v->foreground_region_depth);
  foreground.add("slot", v->foreground_region_slot);
  foreground.add("name", v->foreground_region_name);
  foreground.add("pass", v->foreground_region_pass);
  out.add_object("foreground_region", foreground);

  PayloadArray words;
  for (size_t i = 0; i < 8U; ++i) {
    Payload word;
    word.add("index", (uint32_t)i);
    system_crash_add_hex32(word, "entry", v->entry_words[i]);
    system_crash_add_hex32(word, "exit", v->exit_words[i]);
    word.add("changed", (v->diff_mask & (1U << (uint32_t)i)) != 0U);
    words.add(word);
  }
  out.add_array("frame_words", words);
  return out;
}

static FLASHMEM Payload system_sentinel_beacon_payload(
    const zpnet_sentinel_beacon_t* b,
    bool valid) {
  Payload out;
  out.add("valid", valid);
  if (!valid) return out;

  out.add("depth", b->depth);
  out.add("overflow", b->overflow);

  PayloadArray active;
  const uint32_t bounded =
      b->depth < ZPNET_SENTINEL_BEACON_DEPTH ? b->depth
                                             : ZPNET_SENTINEL_BEACON_DEPTH;
  for (uint32_t i = 0; i < bounded; ++i) {
    const zpnet_sentinel_beacon_entry_t* e = &b->active[i];
    Payload entry;
    entry.add("level", i);
    entry.add("slot", e->slot);
    entry.add("name", e->name);
    entry.add("pass", e->pass);
    entry.add("ipsr", e->ipsr);
    entry.add("dwt_enter", e->dwt_enter);
    active.add(entry);
  }
  out.add_array("active", active);

  Payload completed;
  completed.add("slot", b->last_completed.slot);
  completed.add("name", b->last_completed.name);
  completed.add("pass", b->last_completed.pass);
  completed.add("ipsr", b->last_completed.ipsr);
  completed.add("dwt_enter", b->last_completed.dwt_enter);
  completed.add("dwt_exit", b->last_completed_dwt_exit);
  out.add_object("last_completed", completed);
  return out;
}

static const char* runtime_foreground_phase_name(uint32_t phase) {
  switch ((zpnet_foreground_phase_t)phase) {
    case zpnet_foreground_phase_t::TRANSPORT_PRE:    return "TRANSPORT_PRE";
    case zpnet_foreground_phase_t::TIMEPOP_DISPATCH: return "TIMEPOP_DISPATCH";
    case zpnet_foreground_phase_t::TRANSPORT_POST:   return "TRANSPORT_POST";
    default:                                         return "NONE";
  }
}

static const char* runtime_cpu_usage_state_name(uint32_t state) {
  return state == ZPNET_CPU_USAGE_STATE_ACTIVE ? "ACTIVE" : "IDLE";
}

static const char* runtime_cpu_usage_stage_name(uint32_t stage) {
  switch ((zpnet_cpu_usage_stage_t)stage) {
    case zpnet_cpu_usage_stage_t::CALLBACK_ENTER:
      return "CALLBACK_ENTER";
    case zpnet_cpu_usage_stage_t::DWT_READ_COMPLETE:
      return "DWT_READ_COMPLETE";
    case zpnet_cpu_usage_stage_t::BUSY_READ_COMPLETE:
      return "BUSY_READ_COMPLETE";
    case zpnet_cpu_usage_stage_t::MILLIS_COMPLETE:
      return "MILLIS_COMPLETE";
    case zpnet_cpu_usage_stage_t::CALLBACK_EXIT:
      return "CALLBACK_EXIT";
    default:
      return "IDLE";
  }
}

static FLASHMEM Payload system_runtime_ledger_bank_payload(
    const zpnet_runtime_ledger_t* ledger) {
  Payload out;
  const bool header_valid = runtime_ledger_header_valid(ledger);
  const bool foreground_valid = runtime_ledger_foreground_valid(ledger);
  const bool cpu_usage_valid = runtime_ledger_cpu_usage_valid(ledger);

  out.add("header_valid", header_valid);
  out.add("foreground_valid", foreground_valid);
  out.add("cpu_usage_valid", cpu_usage_valid);
  if (!header_valid) return out;

  out.add("foreground_sequence", ledger->foreground_sequence);
  out.add("foreground_phase_id", ledger->foreground_phase);
  out.add("foreground_phase",
          runtime_foreground_phase_name(ledger->foreground_phase));
  out.add("cpu_usage_sequence", ledger->cpu_usage_sequence);
  out.add("cpu_usage_state_id", ledger->cpu_usage_state);
  out.add("cpu_usage_state",
          runtime_cpu_usage_state_name(ledger->cpu_usage_state));
  out.add("cpu_usage_stage_id", ledger->cpu_usage_stage);
  out.add("cpu_usage_stage",
          runtime_cpu_usage_stage_name(ledger->cpu_usage_stage));
  out.add("cpu_usage_stage_value", ledger->cpu_usage_stage_value);
  out.add("cpu_usage_foreground_sequence",
          ledger->cpu_usage_foreground_sequence);
  out.add("cpu_usage_foreground_phase_id",
          ledger->cpu_usage_foreground_phase);
  out.add("cpu_usage_foreground_phase",
          runtime_foreground_phase_name(ledger->cpu_usage_foreground_phase));
  out.add("cpu_usage_ipsr", ledger->cpu_usage_ipsr);
  return out;
}

static FLASHMEM Payload system_runtime_ledger_payload(void) {
  runtime_ledger_boot_latch();

  Payload out;
  out.add("schema", "ZPNET_RUNTIME_LEDGER_V1");
  out.add("cpu_usage_focus_active_now",
          (bool)g_zpnet_sentinel_cpu_usage_focus_active);
  out.add_object("live",
                 system_runtime_ledger_bank_payload(&g_runtime_ledger));
  out.add_object("retained",
                 system_runtime_ledger_bank_payload(&g_runtime_ledger_retained));
  return out;
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
  return sentinel_pc_plausible(value & ~1U);
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

  system_crash_add_hex32(out, "callback", entry.callback);
  out.add("callback_null", entry.callback == 0U);
  system_crash_add_hex32(out, "slot_callback", entry.slot_callback);
  out.add("callback_matches_slot",
          entry.callback != 0U && entry.callback == entry.slot_callback);
  out.add("callback_executable",
          timepop_dispatch_trace_executable_address(entry.callback));
  out.add("slot_callback_executable",
          timepop_dispatch_trace_executable_address(entry.slot_callback));

  system_crash_add_hex32(out, "user_data", entry.user_data);
  system_crash_add_hex32(out, "name_ptr", entry.name_ptr);
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

static FLASHMEM Payload system_timepop_dispatch_trace_bank_payload(
    const timepop_dispatch_trace_bank_snapshot_t& bank) {
  Payload out;
  out.add("valid", bank.valid);
  out.add("count", bank.count);
  out.add("newest_sequence", bank.newest_sequence);

  PayloadArray records;
  for (uint32_t i = 0U;
       i < bank.count && i < TIMEPOP_DISPATCH_TRACE_ENTRIES;
       ++i) {
    Payload record =
        system_timepop_dispatch_trace_entry_payload(bank.entries[i]);
    record.add("index", i);
    records.add(record);
  }
  out.add_array("records", records);
  return out;
}

static FLASHMEM Payload system_timepop_dispatch_trace_payload(void) {
  timepop_dispatch_trace_snapshot(&g_system_timepop_dispatch_trace_scratch);

  Payload out;
  out.add("schema", "ZPNET_TIMEPOP_DISPATCH_TRACE_V1");
  out.add("capacity", TIMEPOP_DISPATCH_TRACE_ENTRIES);
  out.add_object(
      "live",
      system_timepop_dispatch_trace_bank_payload(
          g_system_timepop_dispatch_trace_scratch.live));
  out.add_object(
      "retained",
      system_timepop_dispatch_trace_bank_payload(
          g_system_timepop_dispatch_trace_scratch.retained));
  return out;
}

static FLASHMEM Payload system_timepop_dispatch_trace_summary_payload(void) {
  timepop_dispatch_trace_snapshot(&g_system_timepop_dispatch_trace_scratch);
  const timepop_dispatch_trace_bank_snapshot_t& retained =
      g_system_timepop_dispatch_trace_scratch.retained;

  Payload out;
  out.add("schema", "ZPNET_TIMEPOP_DISPATCH_TRACE_SUMMARY_V1");
  out.add("retained_valid", retained.valid);
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

static FLASHMEM Payload system_sentinel_payload(void) {
  Payload out;
  out.add("schema", "ZPNET_SENTINEL_V1");

  out.add("enters", g_sentinel_enters);
  out.add("exits", g_sentinel_exits);
  out.add("bad_slot", g_sentinel_bad_slot);
  out.add("unbalanced_enter", g_sentinel_unbalanced_enter);
  out.add("unbalanced_exit", g_sentinel_unbalanced_exit);
  out.add("frames_fpcar", g_sentinel_frames_fpcar);
  out.add("frames_psp", g_sentinel_frames_psp);
  out.add("frames_scan", g_sentinel_frames_scan);
  out.add("frames_none", g_sentinel_frames_none);
  out.add("scan_verdicts_enabled", ZPNET_SENTINEL_SCAN_VERDICTS_ENABLED);
  out.add("scan_changes_ignored", g_sentinel_scan_changes_ignored);
  out.add("violations_this_boot", g_sentinel_violations_this_boot);
  out.add("violations_suppressed", g_sentinel_violations_suppressed);
  out.add("event_pending", (bool)g_sentinel_event_pending);
  out.add("event_emitted", g_sentinel_event_emitted);

  PayloadArray slots;
  for (uint32_t i = 0; i < (uint32_t)ZPNET_SENTINEL_SLOT_COUNT; ++i) {
    const zpnet_sentinel_slot_t* s = &g_sentinel_slots[i];
    if (s->pass == 0U) continue;  // never armed this boot
    Payload slot;
    slot.add("slot", i);
    slot.add("name", s->name ? s->name : "");
    slot.add("pass", s->pass);
    slot.add("active", s->active != 0U);
    system_crash_add_hex32(slot, "frame_address", s->frame_address);
    slot.add("frame_source", sentinel_frame_source_name(s->frame_source));
    system_crash_add_hex32(slot, "exc_return", s->exc_return);
    slot.add("ipsr", s->ipsr);
    system_crash_add_hex32(slot, "msp_entry", s->msp_entry);
    system_crash_add_hex32(slot, "msp_exit_last", s->msp_exit_last);
    system_crash_add_hex32(slot, "entry_checksum", s->entry_checksum);
    system_crash_add_hex32(slot, "exit_checksum_last", s->exit_checksum_last);
    system_crash_add_hex32(slot, "fpccr_entry", s->fpccr_entry);
    system_crash_add_hex32(slot, "fpcar_entry", s->fpcar_entry);
    slot.add("frames_located", s->frames_located);
    slot.add("frames_missing", s->frames_missing);
    slot.add("thread_mode_passes", s->thread_mode_passes);
    slot.add("exc_return_implausible", s->exc_return_implausible);
    slot.add("checksum_fail", s->checksum_fail);
    slot.add("scan_change_ignored", s->scan_change_ignored);
    slot.add("msp_mismatch", s->msp_mismatch);
    slot.add("lspact_consumed", s->lspact_consumed);
    slots.add(slot);
  }
  out.add_array("slots", slots);

  // Beacon boot-latch runs lazily; querying SENTINEL_INFO before any scope
  // has executed must still populate the retained bank.
  sentinel_beacon_boot_latch();
  out.add_object("beacon",
                 system_sentinel_beacon_payload(
                     &g_sentinel_beacon,
                     sentinel_beacon_valid(&g_sentinel_beacon)));
  out.add_object("beacon_retained",
                 system_sentinel_beacon_payload(
                     &g_sentinel_beacon_retained,
                     sentinel_beacon_valid(&g_sentinel_beacon_retained)));

  out.add_object("last_violation",
                 system_sentinel_violation_payload(
                     &g_sentinel_violation_last,
                     g_sentinel_violation_last_present));
  out.add_object("retained_violation",
                 system_sentinel_violation_payload(
                     &g_sentinel_violation_retained,
                     sentinel_retained_valid()));
  out.add_object("runtime_ledger", system_runtime_ledger_payload());
  return out;
}

// ================================================================
// Payload flight recorder — reporting surface
// ================================================================

// The snapshot is taken into static storage before response construction
// begins, because building the response mutates the live ring.
static payload_flight_info_t g_system_payload_flight_scratch;

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

  // Frame-sentinel verdict, runtime breadcrumbs, TimePop dispatch flight
  // recorder, and Payload flight recorder are retained through reboot so they
  // can sit beside the exception evidence they explain.  The beacon says which
  // sentinel regions were active; the dispatch summary says the last committed
  // callback/control-flow stage.
  p.add_object("sentinel",
               system_sentinel_violation_payload(
                   &g_sentinel_violation_retained,
                   sentinel_retained_valid()));
  sentinel_beacon_boot_latch();
  p.add_object("sentinel_beacon",
               system_sentinel_beacon_payload(
                   &g_sentinel_beacon_retained,
                   sentinel_beacon_valid(&g_sentinel_beacon_retained)));
  p.add_object("runtime_ledger", system_runtime_ledger_payload());
  p.add_object("timepop_dispatch_trace",
               system_timepop_dispatch_trace_summary_payload());
  p.add_object("payload_flight", system_payload_flight_payload(true));
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

  // CPU temperature (best-effort)
  p.add("cpu_temp_c", toFixedDecimal(cpuTempC(), 6));

  // Internal reference voltage (best-effort)
  p.add("vref_v", toFixedDecimal(readVrefVolts(), 6));

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
  p.add("cpu_usage_pct",
        toFixedDecimal(cpu_work_pct_milli / 1000.0f, 6));
  p.add("cpu_work_pct",
        toFixedDecimal(cpu_work_pct_milli / 1000.0f, 6));
  p.add("cpu_idle_spin_pct",
        toFixedDecimal(cpu_idle_spin_pct_milli / 1000.0f, 6));

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

  p.add("memory_watchdog_armed", g_system_memory_watchdog_armed);
  p.add("memory_watchdog_handle", g_system_memory_watchdog_handle);
  p.add("memory_watchdog_period_ns", SYSTEM_MEMORY_WATCHDOG_PERIOD_NS);
  p.add("memory_watchdog_arm_attempts", g_system_memory_watchdog_arm_attempts);
  p.add("memory_watchdog_arm_failures", g_system_memory_watchdog_arm_failures);
  p.add("memory_watchdog_callback_count", g_system_memory_watchdog_callback_count);
  p.add("memory_watchdog_event_emitted", g_system_memory_watchdog_event_emitted);
  p.add("memory_watchdog_event_count", g_system_memory_watchdog_event_count);
  p.add("memory_watchdog_alap_pending", (bool)g_system_memory_watchdog_alap_pending);
  p.add("memory_watchdog_alap_armed", (bool)g_system_memory_watchdog_alap_armed);
  p.add("memory_watchdog_alap_arm_count", g_system_memory_watchdog_alap_arm_count);
  p.add("memory_watchdog_alap_arm_failures", g_system_memory_watchdog_alap_arm_failures);
  p.add("memory_watchdog_alap_run_count", g_system_memory_watchdog_alap_run_count);
  p.add("memory_health_status",
        memory_health_status_name(g_system_memory_watchdog_health.status));
  p.add("memory_health_reason",
        memory_health_reason_name(g_system_memory_watchdog_health.primary_reason));

  // Legacy accounted callback-busy diagnostics.
  p.add("cpu_accounted_busy_pct",
        toFixedDecimal(cpu_usage_get_percent(), 6));
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

  return p;
}

// SYSTEM.MEMORY_INFO measures the stack, so its snapshot storage must not live
// on the stack being measured.  Keep the memory_info_t output buffer in RAM2.
static memory_info_t g_system_memory_info_scratch DMAMEM = {};

static void system_dmamem_ensure_initialized(void) {
  if (g_system_dmamem_initialized) return;

  // Teensy places DMAMEM in the NOLOAD .bss.dma section.  Explicitly
  // initialize every SYSTEM RAM2 object before early CLOCKS/INTERRUPT feature
  // publishers inspect the registry.  This function is intentionally lazy
  // because those publishers run before process_system_register().
  runtime_ledger_boot_latch();
  g_system_memory_watchdog_health = memory_health_t{};
  system_feature_registry_reset();
  memset(g_system_crash_report_text, 0, sizeof(g_system_crash_report_text));
  memset(g_system_debug_buffer, 0, sizeof(g_system_debug_buffer));
  g_system_memory_info_scratch = memory_info_t{};
  memset((void*)&g_system_timepop_dispatch_trace_scratch, 0,
         sizeof(g_system_timepop_dispatch_trace_scratch));

  // crash_forensics.cpp owns its retained RAM2 record.  Never initialize or
  // clear that record here; it may be the only surviving witness to a reboot.
  // The same custody applies to g_sentinel_violation_retained, the runtime
  // ledger pair, TimePop's dispatch-flight banks, and the Payload flight rings
  // in payload.cpp: all are NOLOAD retained evidence, validated before use and
  // released only by explicit CRASH_CLEAR or their one-time per-boot latch.
  g_system_dmamem_initialized = true;
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

    memory_info_get(&g_system_memory_info_scratch);
    memory_info_get_health(&g_system_memory_watchdog_health);
    const memory_info_t& info = g_system_memory_info_scratch;
    const memory_health_t& health = g_system_memory_watchdog_health;

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

    p.add("stack_paint_compiled_enabled", info.stack_paint_compiled_enabled);
    p.add("stack_paint_enabled",      info.stack_paint_enabled);
    p.add("stack_paint_overrun",      info.stack_paint_overrun);
    p.add("stack_paint_skip_reason",  info.stack_paint_skip_reason);
    p.add("stack_paint_skip_reason_name",
          info.stack_paint_skip_reason_name ?
              info.stack_paint_skip_reason_name : "UNKNOWN");
    p.add_fmt("stack_paint_pattern", "0x%08lX",
              (unsigned long)info.stack_paint_pattern);
    p.add_fmt("stack_paint_start", "0x%08lX",
              (unsigned long)info.stack_paint_start);
    p.add_fmt("stack_paint_end", "0x%08lX",
              (unsigned long)info.stack_paint_end);
    p.add_fmt("stack_paint_deepest_addr", "0x%08lX",
              (unsigned long)info.stack_paint_deepest_addr);
    p.add("stack_paint_guard_bytes", info.stack_paint_guard_bytes);
    p.add("stack_paint_static_guard_bytes", info.stack_paint_static_guard_bytes);
    p.add("stack_paint_window_bytes", info.stack_paint_window_bytes);
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

    // ==========================================================
    // Always-on memory watchdog validation surface
    // ==========================================================

    p.add("memory_watchdog_armed", g_system_memory_watchdog_armed);
    p.add("memory_watchdog_handle", g_system_memory_watchdog_handle);
    p.add("memory_watchdog_period_ns", SYSTEM_MEMORY_WATCHDOG_PERIOD_NS);
    p.add("memory_watchdog_arm_attempts", g_system_memory_watchdog_arm_attempts);
    p.add("memory_watchdog_arm_failures", g_system_memory_watchdog_arm_failures);
    p.add("memory_watchdog_callback_count", g_system_memory_watchdog_callback_count);
    p.add("memory_watchdog_event_emitted", g_system_memory_watchdog_event_emitted);
    p.add("memory_watchdog_event_count", g_system_memory_watchdog_event_count);
    p.add("memory_watchdog_alap_pending",
          (bool)g_system_memory_watchdog_alap_pending);
    p.add("memory_watchdog_alap_armed",
          (bool)g_system_memory_watchdog_alap_armed);
    p.add("memory_watchdog_alap_arm_count",
          g_system_memory_watchdog_alap_arm_count);
    p.add("memory_watchdog_alap_arm_failures",
          g_system_memory_watchdog_alap_arm_failures);
    p.add("memory_watchdog_alap_run_count",
          g_system_memory_watchdog_alap_run_count);
    p.add("memory_watchdog_last_fire_gnss_ns",
          g_system_memory_watchdog_last_fire_gnss_ns);
    p.add_fmt("memory_watchdog_last_fire_dwt", "0x%08lX",
              (unsigned long)g_system_memory_watchdog_last_fire_dwt);

    p.add("memory_health_status", memory_health_status_name(health.status));
    p.add("memory_health_anomaly",
          health.status == memory_health_status_t::ANOMALY);
    p.add("memory_health_latched", health.latched);
    p.add("memory_health_primary_reason_id", (uint32_t)health.primary_reason);
    p.add("memory_health_primary_reason",
          memory_health_reason_name(health.primary_reason));
    p.add("memory_health_active_reason_mask", health.active_reason_mask);
    p.add("memory_health_latched_reason_mask", health.latched_reason_mask);
    p.add("memory_health_audit_count", health.audit_count);
    p.add("memory_health_anomaly_audit_count", health.anomaly_audit_count);
    p.add("memory_health_first_anomaly_audit", health.first_anomaly_audit);
    p.add("memory_health_last_anomaly_audit", health.last_anomaly_audit);
    p.add("memory_health_heap_free_critical_bytes",
          health.heap_free_critical_bytes);
    p.add("memory_health_last_stack_free_high_water",
          health.stack_free_high_water);
    p.add("memory_health_last_stack_paint_overrun",
          health.stack_paint_overrun);
    p.add("memory_health_last_stack_collision_risk",
          health.stack_collision_risk);
    p.add("memory_health_last_heap_free_total",
          health.heap_free_total);

    p.add("memory_health_payload_lifetime_mismatch",
          health.payload_lifetime_mismatch);
    p.add("memory_health_payload_instances_constructed",
          health.payload_instances_constructed);
    p.add("memory_health_payload_instances_destroyed",
          health.payload_instances_destroyed);
    p.add("memory_health_payload_alive_now", health.payload_alive_now);
    p.add("memory_health_payload_entry_alloc_fail",
          health.payload_entry_alloc_fail);
    p.add("memory_health_payload_arena_alloc_fail",
          health.payload_arena_alloc_fail);
    p.add("memory_health_payload_entry_overflow",
          health.payload_entry_overflow);
    p.add("memory_health_payload_serialize_overflow",
          health.payload_serialize_overflow);
    p.add("memory_health_payload_to_json_fail",
          health.payload_to_json_fail);
    p.add("memory_health_payload_integrity_fail",
          health.payload_integrity_fail);
    p.add("memory_health_payload_invalid_kind",
          health.payload_invalid_kind);
    p.add("memory_health_payload_string_pointer_fault",
          health.payload_string_pointer_fault);
    p.add("memory_health_payload_self_ok_fail",
          health.payload_self_ok_fail);
    p.add("memory_health_payload_entry_high_water",
          health.payload_entry_high_water);
    p.add("memory_health_payload_max_entries", health.payload_max_entries);
    p.add("memory_health_payload_arena_high_water",
          health.payload_arena_high_water);
    p.add("memory_health_payload_arena_max", health.payload_arena_max);
    p.add("memory_health_payload_last_error_code",
          health.payload_last_error_code);
    p.add("memory_health_payload_last_string_pointer_fault_reason",
          health.payload_last_string_pointer_fault_reason);
    p.add("memory_health_payload_last_self_ok_fail_reason",
          health.payload_last_self_ok_fail_reason);

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
// SENTINEL_INFO — exception-frame sentinel state and verdicts
// ------------------------------------------------------------
static FLASHMEM Payload cmd_sentinel_info(const Payload& /*args*/) {
  return system_sentinel_payload();
}

// ------------------------------------------------------------
// TIMEPOP_DISPATCH_INFO — retained + live callback/control-flow transcript
// ------------------------------------------------------------
static FLASHMEM Payload cmd_timepop_dispatch_info(const Payload& /*args*/) {
  return system_timepop_dispatch_trace_payload();
}

// ------------------------------------------------------------
// PAYLOAD_FLIGHT_INFO — retained + live Payload flight recorder
// ------------------------------------------------------------
static FLASHMEM Payload cmd_payload_flight_info(const Payload& /*args*/) {
  return system_payload_flight_payload(false);
}

// ------------------------------------------------------------
// CRASH_INFO — core CrashReport plus retained structured exception evidence
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
  crash_forensics_clear();
  g_system_crash_report_captured = false;
  g_system_crash_report_core_fault_present = false;
  g_system_crash_report_truncated = false;
  g_system_crash_report_bytes = 0;
  g_system_crash_report_text[0] = '\0';

  // Release the retained sentinel verdict so the next violation can latch.
  g_sentinel_violation_retained.magic = 0U;
  g_sentinel_violation_retained.magic_inv = 0U;
  arm_dcache_flush((void*)&g_sentinel_violation_retained,
                   sizeof(g_sentinel_violation_retained));
  g_sentinel_violation_last_present = false;
  g_sentinel_event_pending = false;

  memset((void*)&g_runtime_ledger_retained, 0,
         sizeof(g_runtime_ledger_retained));
  arm_dcache_flush((void*)&g_runtime_ledger_retained,
                   sizeof(g_runtime_ledger_retained));

  timepop_dispatch_trace_clear_retained();

  Payload resp = ok_payload();
  resp.add("crash_report_cleared", true);
  resp.add("crash_forensics_cleared", true);
  resp.add("sentinel_cleared", true);
  resp.add("runtime_ledger_cleared", true);
  resp.add("timepop_dispatch_trace_cleared", true);
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
  { "SENTINEL_INFO",    cmd_sentinel_info    },
  { "TIMEPOP_DISPATCH_INFO", cmd_timepop_dispatch_info },
  { "PAYLOAD_FLIGHT_INFO", cmd_payload_flight_info },
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
  system_memory_watchdog_arm();
}