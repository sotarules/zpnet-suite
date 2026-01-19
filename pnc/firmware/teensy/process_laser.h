#pragma once

#include "process.h"
#include <Arduino.h>

/**
 * ------------------------------------------------------------------
 * LASER Process (process_laser)
 *
 * Role:
 *   • Owns authoritative configuration of the MP5491 laser controller
 *   • Gates optical emission via LD_ON hardware control
 *   • Samples internal monitor photodiode via ADC
 *   • Emits ground-truth laser state as structured events
 *
 * Contract:
 *   • Process start performs authoritative laser initialization
 *   • Process stop performs hard optical inhibit (LD_ON LOW)
 *   • All observation is read-only after initialization
 *   • No timers, interrupts, or background polling
 *
 * Semantics:
 *   • I²C defines laser capability (current, enable state)
 *   • LD_ON defines real-time permission to emit photons
 *   • Photodiode ADC defines physical emission truth
 *
 * Commands:
 *   • REPORT — return current laser state snapshot
 *       - ID1 drive current (mA)
 *       - Photodiode voltage (V)
 *       - Emission state (boolean)
 *
 * ------------------------------------------------------------------
 */

// Register LASER process with the process registry
void process_laser_register(void);
