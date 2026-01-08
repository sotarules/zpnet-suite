#include "teensy_status.h"

#include "config.h"
#include "util.h"
#include "laser.h"

#include <Arduino.h>

static const char* FW_VERSION = "teensy-telemetry-4.3.5";

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
const char* teensy_fw_version() {
  return FW_VERSION;
}

String buildTeensyStatusBody() {
  String b;

  // Firmware identity
  b += "\"fw_version\":\"";
  b += teensy_fw_version();
  b += "\"";

  // Monotonic uptime
  b += ",\"millis\":";
  b += millis();

  // CPU temperature (best-effort)
  b += ",\"cpu_temp_c\":";
  b += cpuTempC();

  // Internal reference voltage (best-effort)
  b += ",\"vref_v\":";
  b += readVrefVolts();

  // Heap availability
  b += ",\"free_heap_bytes\":";
  b += freeHeapBytes();

  // Commanded laser state (intent, not emission)
  b += ",\"laser_enabled\":";
  b += (laser_is_enabled() ? "true" : "false");

  return b;
}
