#pragma once

#include "process.h"
#include <Arduino.h>

/**
 * ------------------------------------------------------------------
 * PHOTODIODE Process (process_photodiode)
 *
 * Role:
 *   • Passively observes photon arrival via ISR-driven edge detection
 *   • Samples photodiode DC level via ADC
 *   • Implements hysteresis-based episode detection
 *   • Maintains a monotonic photon-episode counter
 *
 * Model:
 *   • PHOTODIODE is an observational instrument, not a lifecycle-managed process
 *   • It has no notion of "start" or "stop"
 *   • All hardware engagement is explicit and physical
 *
 * Initialization:
 *   • photodiode_init() attaches the edge ISR and clears internal state
 *   • Initialization is explicit and must be invoked deliberately:
 *       - from teensy.cpp during boot
 *       - or via the PHOTODIODE.INIT command
 *   • Initialization emits a PHOTODIODE_INITIALIZATION event
 *
 * Semantics:
 *   • Edge pin provides temporal photon arrival information
 *   • Analog pin provides continuous light presence confidence
 *   • Episode count increments exactly once per light episode
 *   • State advances only via ISR activity and snapshot evaluation
 *
 * Observation:
 *   • REPORT is stateless and read-only
 *   • No timers, background polling, aggregation, or inference
 *
 * Commands:
 *   • INIT   — perform explicit ISR attachment and state reset
 *   • REPORT — return current photodiode state snapshot
 *       - edge_level
 *       - edge_pulse_count
 *       - analog_raw
 *       - analog_v
 *   • COUNT  — return monotonic episode counter only
 *   • CLEAR  — explicitly reset episode counter
 *
 * ------------------------------------------------------------------
 */
// Explicit, authoritative initialization
void process_photodiode_init(void);

// Register PHOTODIODE command surface
void process_photodiode_register(void);


