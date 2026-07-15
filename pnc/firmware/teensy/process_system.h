#pragma once

#include "process.h"
#include <Arduino.h>

/**
 * ------------------------------------------------------------------
 * SYSTEM Process (process_system)
 *
 * Role:
 *   • Exposes authoritative Teensy-wide system facts
 *   • Provides a stable inspection surface for intrinsic properties,
 *     including:
 *       - firmware identity
 *       - CPU temperature and utilization
 *       - memory availability
 *       - internal reference voltages
 *       - other system-owned invariants
 *
 * Model:
 *   • SYSTEM is not a lifecycle-managed process
 *   • It does not "start" or "stop"
 *   • It exists by definition once the firmware is executing
 *
 * Ontology:
 *   • SYSTEM represents the universe as observed from within itself
 *   • Transitions such as boot, shutdown, or reset are observable facts,
 *     not lifecycle actions
 *   • Any notion of SYSTEM beginning or ending would require a
 *     meta-system in which such transitions could be expressed
 *
 * Initialization:
 *   • There is no SYSTEM initialization routine
 *   • SYSTEM facts become valid as soon as the underlying hardware and
 *     instrumentation exist
 *   • Readiness and boot state are communicated exclusively via explicit
 *     events (e.g., BOOT, SYSTEM_READY), never via lifecycle callbacks
 *
 * Semantics:
 *   • SYSTEM owns no hardware directly
 *   • SYSTEM normally reports facts without implicit control actions
 *   • SYSTEM owns bounded always-on integrity watchdog scheduling for
 *     system-wide conditions that invalidate all subsystem evidence
 *   • Low-level authorities render watchdog verdicts; SYSTEM emits the
 *     resulting semantic anomaly event without reinterpreting their rules
 *
 * Observation:
 *   • REPORT returns a read-only snapshot of current system state
 *   • REPORT has no control side effects
 *   • A bounded recurring TimePop audits Teensy memory/Payload integrity and
 *     emits one WATCHDOG_ANOMALY per boot if the low-level court latches failure
 *
 * Commands:
 *   • REPORT — return authoritative system snapshot
 *       - firmware version
 *       - CPU temperature
 *       - CPU usage metrics
 *       - memory availability
 *       - internal diagnostics
 *   • MEMORY_AUDIT_INFO — return a bounded retained/live scalar transcript
 *     from either memory_info_audit() or its SYSTEM watchdog callback wrapper;
 *     accepts source=audit|watchdog, bank=retained|live, count=1..8, offset=N
 *   • TIMEPOP_DISPATCH_INFO — return a bounded TimePop callback, mutation, and
 *     rearm transcript; accepts bank=retained|live, count=1..8, offset=N
 *
 * Terminal Transitions:
 *   • Shutdown and bootloader entry are explicit, irreversible boundary
 *     crossings
 *   • These transitions are expressed via commands and events, not
 *     lifecycle semantics
 *
 * ------------------------------------------------------------------
 */

// Register SYSTEM command surface
void process_system_register(void);

// ============================================================================
// Exception-frame sentinel
// ============================================================================
//
// Purpose: catch corruption of a stacked exception frame while the corrupting
// handler pass is still on the CPU, with attribution.
//
// Current crash-hunt deployment follows the retained fault evidence:
//
//   • CPU_USAGE is a thread-mode victim marker around cpu_usage_sample();
//   • QTimer1, QTimer2, QTimer3, and PPS GPIO get distinct priority-0 slots;
//   • those priority-0 scopes arm only while CPU_USAGE is active, preserving
//     normal capture latency while still inspecting every interrupt that can
//     preempt the recurring 0x426 crash neighborhood;
//   • the priority-16 handoff and lane-specific publication tribunals remain
//     independently attributable.
//
// Usage contract:
//
//   void my_handler(void) {
//     ZPNET_SENTINEL_ENTER(ZPNET_SENTINEL_SLOT_HANDOFF_ISR, "HANDOFF_ISR");
//     ... handler body ...
//     ZPNET_SENTINEL_EXIT(ZPNET_SENTINEL_SLOT_HANDOFF_ISR);
//   }
//
//   • ENTER must run before the first function call can clobber LR.  A timing
//     handler may take its sacred first-instruction register capture before
//     ENTER, provided that capture is inline and performs no call.
//   • EXIT must be the LAST statement before every return, at the same brace
//     level as ENTER, with no VLAs or alloca between them (the MSP
//     entry/exit comparison assumes a fixed frame).
//   • Each concurrent handler site needs its own slot.  A slot is never
//     shared between handlers that can preempt one another.
//   • Both calls are scalar-only, allocator-free, and bounded; they are safe
//     at any interrupt priority.
//
// Verdicts are surfaced through SYSTEM.SENTINEL_INFO, folded into
// SYSTEM.CRASH_INFO, retained across reboot in RAM2, and announced by one
// SENTINEL_ANOMALY event per boot from serialized foreground context.

enum {
  ZPNET_SENTINEL_SLOT_HANDOFF_ISR              = 0,
  ZPNET_SENTINEL_SLOT_DWT_PUBLICATION_VCLOCK   = 1,
  ZPNET_SENTINEL_SLOT_DWT_PUBLICATION_OCXO1    = 2,
  ZPNET_SENTINEL_SLOT_DWT_PUBLICATION_OCXO2    = 3,
  ZPNET_SENTINEL_SLOT_DWT_FATAL_VCLOCK         = 4,
  ZPNET_SENTINEL_SLOT_DWT_FATAL_OCXO1          = 5,
  ZPNET_SENTINEL_SLOT_DWT_FATAL_OCXO2          = 6,
  ZPNET_SENTINEL_SLOT_CPU_USAGE                = 7,
  ZPNET_SENTINEL_SLOT_QTIMER1_ISR              = 8,
  ZPNET_SENTINEL_SLOT_QTIMER2_ISR              = 9,
  ZPNET_SENTINEL_SLOT_QTIMER3_ISR              = 10,
  ZPNET_SENTINEL_SLOT_PPS_GPIO_ISR             = 11,
  ZPNET_SENTINEL_SLOT_RESERVED                 = 12,
  ZPNET_SENTINEL_SLOT_COUNT                    = 13,
};

// name must be a string literal or other immortal storage; only the pointer
// is retained.  Both macros capture SP at the caller's own frame level, so
// the entry/exit MSP comparison sees the handler's frame rather than the
// sentinel functions' differing prologues.
void zpnet_sentinel_enter(uint32_t slot,
                          const char* name,
                          uint32_t exc_return,
                          uint32_t caller_sp);
void zpnet_sentinel_exit(uint32_t slot, uint32_t caller_sp);

#if defined(__arm__)
#define ZPNET_SENTINEL_ENTER(slot, name)                                     \
  do {                                                                       \
    uint32_t zpnet_sentinel_exc_return_;                                     \
    uint32_t zpnet_sentinel_sp_;                                             \
    __asm__ volatile("mov %0, lr"                                            \
                     : "=r"(zpnet_sentinel_exc_return_));                    \
    __asm__ volatile("mov %0, sp"                                            \
                     : "=r"(zpnet_sentinel_sp_));                            \
    zpnet_sentinel_enter((slot), (name), zpnet_sentinel_exc_return_,         \
                         zpnet_sentinel_sp_);                                \
  } while (0)
#define ZPNET_SENTINEL_EXIT(slot)                                            \
  do {                                                                       \
    uint32_t zpnet_sentinel_sp_;                                             \
    __asm__ volatile("mov %0, sp"                                            \
                     : "=r"(zpnet_sentinel_sp_));                            \
    zpnet_sentinel_exit((slot), zpnet_sentinel_sp_);                         \
  } while (0)
#else
#define ZPNET_SENTINEL_ENTER(slot, name)                                     \
  zpnet_sentinel_enter((slot), (name), 0U, 0U)
#define ZPNET_SENTINEL_EXIT(slot) zpnet_sentinel_exit((slot), 0U)
#endif

// ============================================================================
// Foreground / CPU-usage retained crash breadcrumbs
// ============================================================================
//
// The main loop records its current imperative phase.  cpu_usage_tick() then
// latches the phase and marks one CPU sample active before entering the tiny
// DWT-read victim.  Both live and previous-boot records are reported through
// SYSTEM.SENTINEL_INFO and SYSTEM.CRASH_INFO.

enum class zpnet_foreground_phase_t : uint32_t {
  NONE             = 0,
  TRANSPORT_PRE    = 1,
  TIMEPOP_DISPATCH = 2,
  TRANSPORT_POST   = 3,
};

enum class zpnet_cpu_usage_stage_t : uint32_t {
  IDLE               = 0,
  CALLBACK_ENTER     = 1,
  DWT_READ_COMPLETE  = 2,
  BUSY_READ_COMPLETE = 3,
  MILLIS_COMPLETE    = 4,
  CALLBACK_EXIT      = 5,
};

void zpnet_foreground_phase_note(zpnet_foreground_phase_t phase);
void zpnet_cpu_usage_ledger_enter(void) __attribute__((noinline));
void zpnet_cpu_usage_ledger_stage(zpnet_cpu_usage_stage_t stage,
                                  uint32_t value) __attribute__((noinline));
void zpnet_cpu_usage_ledger_exit(void) __attribute__((noinline));

// Priority-0 ISR wrappers sample this flag inline before their first call.
// It is true only across cpu_usage_sample(), so the expensive frame court does
// not run continuously on the 1 kHz timing rails.
extern volatile bool g_zpnet_sentinel_cpu_usage_focus_active;

// ============================================================================
// Feature status substrate
// ============================================================================
//
// Teensy SYSTEM owns only Teensy-local feature state.  Pi SYSTEM asks for this
// tree and unions it with Pi-local feature state into the global dashboard /
// campaign-readiness surface.
//
// Teensy also publishes FEATURE_STATUS_FRAGMENT whenever any local feature
// scalar changes.  The fragment contains only Teensy-owned feature paths; the
// Pi-side SYSTEM service relays the unified FEATURE_STATUS tree.
//
// Feature paths are reported as:
//
//   TEENSY.<SUBSYSTEM>.<FEATURE>
//
// with status values from this closed vocabulary.

enum class system_feature_status_t : uint8_t {
  INITIALIZING = 0,
  NOMINAL      = 1,
  HOLD         = 2,
  ANOMALY      = 3,
};

const char* system_feature_status_str(system_feature_status_t status);
bool system_feature_status_parse(const char* status,
                                 system_feature_status_t* out);

bool system_feature_set(const char* subsystem,
                        const char* feature,
                        system_feature_status_t status,
                        const char* detail = nullptr);

bool system_feature_set_str(const char* subsystem,
                            const char* feature,
                            const char* status,
                            const char* detail = nullptr);

bool system_feature_has(const char* subsystem,
                        const char* feature);

const char* system_feature_get_status(const char* subsystem,
                                      const char* feature);

bool system_feature_is_nominal(const char* subsystem,
                               const char* feature);