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
