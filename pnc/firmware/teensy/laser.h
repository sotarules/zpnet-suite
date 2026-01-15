#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// Laser control and diagnostics
// --------------------------------------------------------------
//
// This module owns:
//   • Laser enable / disable
//   • Long-duration voltage diagnostics
//
// It does NOT:
//   • Remember laser state
//   • Infer emission
//   • Report laser truth
//
// Emission is observed exclusively via photodiode instrumentation.
//

// Initialize laser hardware
void laser_init();

// Laser control (actuation only)
void laser_on();
void laser_off();

// Diagnostic: long-duration averaged voltages
// Returns JSON body fragment (no enclosing braces)
String laser_measure_voltages();
