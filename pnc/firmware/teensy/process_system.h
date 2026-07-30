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
 *
 * Observation:
 *   • REPORT returns a read-only snapshot of current system state
 *   • REPORT has no control side effects
 *
 * Commands:
 *   • REPORT — return authoritative integer-only system snapshot
 *       - firmware version
 *       - scaled-integer CPU usage metrics
 *       - internal diagnostics
 *       - floating-point environmental telemetry is intentionally excluded
 *   • EXECUTION_TRACE — return the bounded ISR-to-handoff-to-subscriber-to-
 *     TimePop control-flow transcript.  The live ring runs in RAM1; fault entry
 *     copies committed records into retained RAM2 before reboot.  Accepts
 *     bank=retained|live, count=1..8, offset=N; retained is the default.
 *   • TIMEPOP_DISPATCH_INFO — compatibility alias for EXECUTION_TRACE using
 *     the earlier TimePop report schema
 *   • CRASH_POLICY — return live CPU/FPU exception policy plus a compact
 *     consistency analysis of retained core and extended exception frames
 *   • PAYLOAD_CONTRACT_INFO — return design-by-contract counters plus the
 *     first/latest current-boot incident and latest retained incident
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
// Foreground crash breadcrumbs
// ============================================================================
//
// The main loop records its current imperative phase.  Live and compatibility
// retained records are reported through SYSTEM.CRASH_INFO.

enum class zpnet_foreground_phase_t : uint32_t {
  NONE             = 0,
  TRANSPORT_PRE    = 1,
  TIMEPOP_DISPATCH = 2,
  TRANSPORT_POST   = 3,
};

void zpnet_foreground_phase_note(zpnet_foreground_phase_t phase);

// ============================================================================
// Feature status substrate
// ============================================================================
//
// Teensy SYSTEM owns only Teensy-local feature state.  Every always-on
// MONITOR_FRAGMENT carries the current Teensy feature tree; Pi SYSTEM unions it
// with Pi-local feature state into the unified MONITOR dashboard /
// campaign-readiness surface.
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

// Foreground-only mutation boundary. Calls from any exception/ISR context are
// rejected by a runtime court; interrupt code must hand off fixed scalar state.
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

// ============================================================================
// PPS-aligned MONITOR_FRAGMENT side rail
// ============================================================================
// Canonical always-on tick. process_interrupt calls this for every completed
// PPS/VCLOCK second whether or not a CLOCKS campaign is active. It records only
// the scalar sequence and arms foreground publication.
//
// Foreground SYSTEM then takes one typed CLOCKS snapshot and authors the full
// MONITOR_FRAGMENT wire shape. The always-on clock view is observational; an
// optional campaign observation is included only for a completed public record.
void system_monitor_pps_tick_from_interrupt(
    uint32_t completed_second_sequence);

// Transitional CLOCKS-side entry point. It remains source-compatible with the
// campaign-coupled caller but is ignored after the canonical interrupt owner
// has supplied its first tick.
void system_monitor_pps_tick(uint32_t completed_second_sequence);

// CLOCKS/Beta calls this after it has frozen a typed public campaign observation
// for the completed instrument sequence. Usually the interrupt-owned MONITOR tick
// is already pending and this merely coalesces into it. If SYSTEM already emitted
// the observation-only fragment, this deliberately schedules one same-sequence
// copy carrying the campaign facts so durable truth cannot be lost.
void system_monitor_campaign_row_ready(uint32_t completed_second_sequence);
