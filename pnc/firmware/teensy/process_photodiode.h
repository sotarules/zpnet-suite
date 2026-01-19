#pragma once

#include "process.h"
#include <Arduino.h>

/**
 * ------------------------------------------------------------------
 * PHOTODIODE Process (process_photodiode)
 *
 * Role:
 *   • Observes photon arrival via ISR-driven edge detection
 *   • Observes photodiode DC level via ADC
 *   • Implements hysteresis-based episode detection
 *   • Maintains monotonic episode count
 *
 * Contract:
 *   • Process start attaches ISR and clears state
 *   • Process stop detaches ISR
 *   • No background polling or timers
 *   • All state advances only via ISR + snapshot
 *
 * Semantics:
 *   • Edge pin defines temporal photon arrival
 *   • Analog pin defines light presence confidence
 *   • Episode count increments exactly once per light episode
 *
 * Commands:
 *   • REPORT — return current photodiode state snapshot
 *       - edge_level
 *       - edge_pulse_count
 *       - analog_raw
 *       - analog_v
 *
 * ------------------------------------------------------------------
 */

// Register PHOTODIODE process with the process registry
void process_photodiode_register(void);
