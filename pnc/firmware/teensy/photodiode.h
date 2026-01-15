#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// Photodiode instrumentation
// --------------------------------------------------------------
//
// This module owns:
//   • Edge ISR
//   • Episode detection state machine
//   • Analog sampling
//   • Pulse count lifecycle
//
// It does NOT infer laser meaning.
// It reports raw observables only.
//

// Initialize photodiode hardware and ISR
void photodiode_init();

// Maintain episode latch (call every loop)
void photodiode_update();

// Clear episode count and latch
void photodiode_clear();

// Build telemetry payloads (JSON body fragments)
String buildPhotodiodeStatusBody();
String buildPhotodiodeCountBody();
