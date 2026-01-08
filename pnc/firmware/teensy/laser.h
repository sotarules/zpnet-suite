#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// Laser control and diagnostics
// --------------------------------------------------------------
//
// This module owns:
//   • Laser enable / disable
//   • Local laser state
//   • Long-duration voltage diagnostics
//
// It does NOT infer laser health or meaning.
// It only controls and measures.
//

// Initialize laser hardware
void laser_init();

// Laser control
void laser_on();
void laser_off();

// Query local laser enable state
bool laser_is_enabled();

// Diagnostic: long-duration averaged voltages
// Returns JSON body fragment (no enclosing braces)
String laser_measure_voltages();
