#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// Teensy self-status (identity + basic health)
// --------------------------------------------------------------
//
// This module owns:
//   • Firmware identity
//   • Teensy-local self-observation
//
// It does NOT:
//   • Control hardware
//   • Interpret other subsystems
//   • Perform aggregation
//

// Returns firmware version string
const char* teensy_fw_version();

// Build TEENSY_STATUS payload (JSON body fragment)
String buildTeensyStatusBody();
