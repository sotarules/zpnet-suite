#include "teensy_status.h"

#include "config.h"
#include "util.h"
#include "cpu_usage.h"

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

  // ------------------------------------------------------------
  // Firmware identity
  // ------------------------------------------------------------
  out += "\"fw_version\":\"";
  out += teensy_fw_version();
  out += "\"";

  // ------------------------------------------------------------
  // CPU temperature (best-effort)
  // ------------------------------------------------------------
  out += ",\"cpu_temp_c\":";
  out += cpuTempC();

  // ------------------------------------------------------------
  // Internal reference voltage (best-effort)
  // ------------------------------------------------------------
  out += ",\"vref_v\":";
  out += readVrefVolts();

  // ------------------------------------------------------------
  // Heap availability
  // ------------------------------------------------------------
  out += ",\"free_heap_bytes\":";
  out += freeHeapBytes();

  // ------------------------------------------------------------
  // CPU usage (authoritative, idle-cycle accounting)
  // ------------------------------------------------------------
  out += ",\"cpu_usage_pct\":";
  {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.4f", cpu_usage_get_percent());
    out += buf;
  }

  // ------------------------------------------------------------
  // CPU usage raw counters (audit + diagnostics)
  // ------------------------------------------------------------
  out += ",\"cpu_busy_cycles\":";
  out += cpu_usage_get_busy_cycles();

  out += ",\"cpu_total_cycles\":";
  out += cpu_usage_get_total_cycles();

  out += ",\"cpu_sample_window_ms\":";
  out += cpu_usage_get_sample_window_ms();

  out += ",\"cpu_freq_mhz\":";
  out += cpu_usage_get_cpu_freq_mhz();

  out += "}";
  return out;
}
