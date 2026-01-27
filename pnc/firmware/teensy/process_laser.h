#pragma once

#include "process.h"
#include <Arduino.h>

/**
 * ------------------------------------------------------------------
 * LASER Process (process_laser)
 *
 * Role:
 *   • Owns authoritative configuration of the MP5491 laser controller
 *   • Gates optical emission via the LD_ON hardware control line
 *   • Samples the internal monitor photodiode via ADC
 *   • Emits ground-truth laser state as structured, durable events
 *
 * Model:
 *   • LASER is a command-driven hardware controller, not a lifecycle-managed process
 *   • Initialization is explicit and physical, not implicit or framework-owned
 *   • There is no notion of "starting" or "stopping" the LASER process
 *
 * Initialization:
 *   • laser_init() performs authoritative, idempotent hardware configuration
 *   • laser_init() may be invoked:
 *       - explicitly from teensy.cpp during boot
 *       - via the LASER.INIT command
 *   • Initialization emits a forensic LASER_INITIALIZATION event
 *
 * Semantics:
 *   • I²C defines laser capability (current setpoint, enable configuration)
 *   • LD_ON defines real-time permission to emit photons
 *   • Photodiode ADC defines physical emission truth
 *
 * Observation:
 *   • REPORT is stateless and read-only
 *   • No timers, interrupts, background polling, or aggregation
 *
 * Commands:
 *   • INIT   — perform explicit hardware initialization (wrapper for laser_init)
 *   • REPORT — return current laser state snapshot
 *       - ID1 drive current (mA)
 *       - Photodiode voltage (V)
 *       - Emission state (boolean, derived from photodiode)
 *   • ON     — permit optical emission (LD_ON HIGH)
 *   • OFF    — inhibit optical emission (LD_ON LOW)
 *
 * ------------------------------------------------------------------
 */

// Explicit, authoritative hardware initialization
void process_laser_init(void);

// Register LASER command surface
void process_laser_register(void);
