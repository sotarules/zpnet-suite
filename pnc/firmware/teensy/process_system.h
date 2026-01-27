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
 *   • SYSTEM performs no implicit control actions
 *   • SYSTEM aggregates nothing and infers nothing
 *   • SYSTEM reports facts only
 *
 * Observation:
 *   • REPORT returns a stateless, read-only snapshot of current system state
 *   • REPORT has no side effects
 *   • No timers, background polling, or aggregation logic exists here
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
