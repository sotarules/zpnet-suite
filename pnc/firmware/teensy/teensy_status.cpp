#include "teensy_status.h"

#include "config.h"
#include "util.h"

#include <Arduino.h>

static const char* FW_VERSION = "teensy-telemetry-4.3.5";

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
const char* teensy_fw_version() {
  return FW_VERSION;
}

String buildTeensyStatusBody() {

  String out;
  out += "{";

  // Firmware identity
  out += "\"fw_version\":\"";
  out += teensy_fw_version();
  out += "\"";

  // CPU temperature (best-effort)
  out += ",\"cpu_temp_c\":";
  out += cpuTempC();

  // Internal reference voltage (best-effort)
  out += ",\"vref_v\":";
  out += readVrefVolts();

  // Heap availability
  out += ",\"free_heap_bytes\":";
  out += freeHeapBytes();

  out += "}";
  return out;
}
