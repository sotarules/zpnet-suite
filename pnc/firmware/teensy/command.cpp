#include "bootloader.h"
#include "debug.h"
#include "command.h"
#include "event_bus.h"
#include "util.h"
#include "system.h"
#include "process.h"

#include "laser.h"
#include "photodiode.h"
#include "gnss.h"
#include "teensy_status.h"
#include "qtimer.h"
#include "gpt_count.h"
#include "transport.h"

#include <string.h>
#include <stdlib.h>

// --------------------------------------------------------------
// Internal helpers
// --------------------------------------------------------------
static bool extractCmd(const char* line, char* out, size_t out_sz) {
  const char* p = strstr(line, "\"cmd\"");
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;

  while (*p == ' ') p++;
  if (*p != '\"') return false;
  p++;

  const char* q = strchr(p, '\"');
  if (!q) return false;

  size_t n = q - p;
  if (n >= out_sz) n = out_sz - 1;

  memcpy(out, p, n);
  out[n] = '\0';
  return true;
}

static bool extractUintArg(
    const char* line,
    const char* key,
    uint64_t* out
) {
  const char* p = strstr(line, key);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;

  while (*p == ' ') p++;

  char* end = nullptr;
  unsigned long long val = strtoull(p, &end, 10);
  if (end == p) return false;

  *out = (uint64_t)val;
  return true;
}

// --------------------------------------------------------------
// Immediate framed emit (QUERY plane only)
// --------------------------------------------------------------
static inline void emitImmediateFramed(
    const char* type,
    const String& body
) {
  String out;

  out += "{\"type\":\"";
  out += type;
  out += "\"";

  if (body.length() > 0) {
    out += ",";
    out += body;
  }

  out += "}";

  transport_send_frame(out.c_str(), out.length());
}

// --------------------------------------------------------------
// Helper: process type parsing (minimal, explicit)
// --------------------------------------------------------------
static bool parseProcessType(const char* line, process_type_t& out) {
  if (strstr(line, "\"type\":\"GNSS\"")) {
    out = PROCESS_TYPE_GNSS;
    return true;
  }
  if (strstr(line, "\"type\":\"TEMPEST\"")) {
    out = PROCESS_TYPE_TEMPEST;
    return true;
  }
  if (strstr(line, "\"type\":\"LANTERN\"")) {
    out = PROCESS_TYPE_LANTERN;
    return true;
  }
  return false;
}

// --------------------------------------------------------------
// Command execution
// --------------------------------------------------------------
void command_exec(const char* line) {

  debug_log("CMD", "ENTERED command_exec (kernel)");

  char cmd[64];

  if (!extractCmd(line, cmd, sizeof(cmd))) {
    enqueueErrEvent("UNKNOWN", "missing cmd");
    return;
  }

  // ------------------------------------------------------------
  // EVENT BUS
  // ------------------------------------------------------------
  if (strcmp(cmd, "EVENTS.GET") == 0) {
    drainEventsNow();
    return;
  }

  // ============================================================
  // PROCESS COMMANDS
  // ============================================================

  if (strcmp(cmd, "PROCESS.LIST") == 0) {
    String body = process_list_json();
    emitImmediateFramed("PROCESS_LIST", body);
    return;
  }

  if (strcmp(cmd, "PROCESS.START") == 0) {
    process_type_t type;
    if (!parseProcessType(line, type)) {
      enqueueErrEvent(cmd, "missing or invalid process type");
      return;
    }

    if (!process_start(type)) {
      enqueueErrEvent(cmd, "process start failed");
      return;
    }

    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "PROCESS.STOP") == 0) {
    process_type_t type;
    if (!parseProcessType(line, type)) {
      enqueueErrEvent(cmd, "missing or invalid process type");
      return;
    }

    if (!process_stop(type)) {
      enqueueErrEvent(cmd, "process stop failed");
      return;
    }

    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "PROCESS.QUERY") == 0) {
    process_type_t type;
    if (!parseProcessType(line, type)) {
      emitImmediateFramed(
        "PROCESS_QUERY",
        "\"error\":\"missing or invalid process type\""
      );
      return;
    }

    String body;
    if (!process_query(type, body)) {
      emitImmediateFramed(
        "PROCESS_QUERY",
        "\"error\":\"process not found\""
      );
      return;
    }

    emitImmediateFramed("PROCESS_QUERY", body);
    return;
  }

  if (strcmp(cmd, "PROCESS.COMMAND") == 0) {

    process_type_t type;
    if (!parseProcessType(line, type)) {
      emitImmediateFramed(
        "PROCESS_COMMAND",
        "\"error\":\"missing or invalid process type\""
      );
      return;
    }

    const char* cmd_arg = strstr(line, "\"proc_cmd\"");
    if (!cmd_arg) {
      emitImmediateFramed(
        "PROCESS_COMMAND",
        "\"error\":\"missing proc_cmd\""
      );
      return;
    }

    cmd_arg = strchr(cmd_arg, ':');
    if (!cmd_arg) {
      emitImmediateFramed(
        "PROCESS_COMMAND",
        "\"error\":\"malformed proc_cmd (missing colon)\""
      );
      return;
    }

    cmd_arg++;
    while (*cmd_arg == ' ') cmd_arg++;

    if (*cmd_arg != '\"') {
      emitImmediateFramed(
        "PROCESS_COMMAND",
        "\"error\":\"malformed proc_cmd (missing opening quote)\""
      );
      return;
    }

    cmd_arg++;

    char proc_cmd[64];
    const char* q = strchr(cmd_arg, '\"');
    if (!q) {
      emitImmediateFramed(
        "PROCESS_COMMAND",
        "\"error\":\"malformed proc_cmd (missing closing quote)\""
      );
      return;
    }


    size_t n = q - cmd_arg;
    if (n >= sizeof(proc_cmd)) n = sizeof(proc_cmd) - 1;
    memcpy(proc_cmd, cmd_arg, n);
    proc_cmd[n] = '\0';

    String body;
    if (!process_command(type, proc_cmd, nullptr, body)) {
      emitImmediateFramed(
        "PROCESS_COMMAND",
        "\"error\":\"command rejected\""
      );
      return;
    }

    enqueueAckEvent(cmd);
    return;
  }

  // ============================================================
  // STATUS / DATA COMMANDS (LEGACY – PRESERVED)
  // ============================================================

  if (strcmp(cmd, "TEENSY.STATUS") == 0) {
    enqueueEvent("TEENSY_STATUS", buildTeensyStatusBody());
    return;
  }

  if (strcmp(cmd, "GNSS.STATUS") == 0) {
    enqueueEvent("GNSS_STATUS", buildGnssStatusBody());
    return;
  }

  if (strcmp(cmd, "GNSS.DATA") == 0) {
    enqueueEvent("GNSS_DATA", buildGnssDataBody());
    return;
  }

  // ============================================================
  // QUERY COMMANDS (NON-DESTRUCTIVE)
  // ============================================================

  if (strcmp(cmd, "PHOTODIODE.STATUS?") == 0) {
    emitImmediateFramed(
      "PHOTODIODE_STATUS",
      buildPhotodiodeStatusBody()
    );
    return;
  }

  // ============================================================
  // QTIMER COMMANDS
  // ============================================================

  if (strcmp(cmd, "QTIMER.ARM") == 0) {
    qtimer_arm();
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "QTIMER.DISARM") == 0) {
    qtimer_disarm();
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "QTIMER.READ?") == 0) {
    String b;
    b += "\"count\":";
    b += qtimer_read();
    emitImmediateFramed("QTIMER_READ", b);
    return;
  }

  if (strcmp(cmd, "QTIMER.CLEAR") == 0) {
    qtimer_clear();
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "QTIMER.STATUS?") == 0) {
    String b;
    b += "\"armed\":";
    b += qtimer_status() ? "true" : "false";
    emitImmediateFramed("QTIMER_STATUS", b);
    return;
  }

  // ------------------------------------------------------------
  // GPT CONFIRM (LEGACY, SINGLE-SAMPLE)
  // ------------------------------------------------------------
  if (strcmp(cmd, "GPT.CONFIRM") == 0) {

    uint64_t seconds = 2;
    extractUintArg(line, "\"seconds\"", &seconds);

    // Sanity clamp
    if (seconds == 0) seconds = 1;
    if (seconds > 3600) seconds = 3600;

    uint64_t cpu_cycles = 0;
    double   ratio      = 0.0;
    int64_t  error_cycles = 0;

    // Seconds-based confirm (GNSS-anchored)
    uint64_t gpt_count = gpt_count_confirm(
        (uint32_t)seconds,
        &cpu_cycles,
        &ratio,
        &error_cycles
    );

    String body;
    body += "\"seconds\":";
    body += seconds;
    body += ",\"gpt_count\":";
    body += gpt_count;
    body += ",\"cpu_cycles\":";
    body += cpu_cycles;
    body += ",\"ratio\":";

    {
      char buf[32];
      snprintf(buf, sizeof(buf), "%.12f", ratio);
      body += buf;
    }

    body += ",\"error_cycles\":";
    body += error_cycles;

    enqueueEvent("GPT_CONFIRM_RESULT", body);
    enqueueAckEvent(cmd);
    return;
  }

  // ------------------------------------------------------------
  // TAU BASELINE DISCOVERY (SELF-CALIBRATING)
  // ------------------------------------------------------------
  if (strcmp(cmd, "TAU.BASELINE") == 0) {

    uint32_t samples   = 0;
    int32_t  min_error = 0;
    int32_t  max_error = 0;
    int32_t  med_error = 0;
    int32_t  std_error = 0;
    double   stddev    = 0.0;

    bool ok = gpt_discover_standard_error(
        &samples,
        &min_error,
        &max_error,
        &med_error,
        &std_error,
        &stddev
    );

    if (!ok) {
      enqueueErrEvent(cmd, "baseline discovery failed");
      return;
    }

    String body;
    body += "\"samples\":";
    body += samples;
    body += ",\"min_error\":";
    body += min_error;
    body += ",\"max_error\":";
    body += max_error;
    body += ",\"med_error\":";
    body += med_error;
    body += ",\"std_error\":";
    body += std_error;
    body += ",\"stddev\":";

    char buf[32];
    snprintf(buf, sizeof(buf), "%.6f", stddev);
    body += buf;

    enqueueEvent("TAU_BASELINE", body);
    enqueueAckEvent(cmd);
    return;
  }

  // ------------------------------------------------------------
  // TAU PROFILING (SELF-CALIBRATING, MONTE CARLO)
  // ------------------------------------------------------------
  if (strcmp(cmd, "TAU.PROFILE") == 0) {

    uint64_t total_seconds  = 0;

    if (!extractUintArg(line, "\"total_seconds\"", &total_seconds)) {
      enqueueErrEvent(cmd, "missing or invalid arguments");
      return;
    }

    // Hard safety cap (defensive)
    if (total_seconds > (48ULL * 3600ULL)) {
      enqueueErrEvent(cmd, "total_seconds exceeds cap");
      return;
    }

    bool ok = gpt_tau_profile(
        (uint32_t)total_seconds
    );

    if (!ok) {
      enqueueErrEvent(cmd, "tau profiling failed");
      return;
    }

    enqueueAckEvent(cmd);
    return;
  }
  // ------------------------------------------------------------
  // PHOTODIODE COMMANDS
  // ------------------------------------------------------------
  if (strcmp(cmd, "PHOTODIODE.STATUS") == 0) {
    enqueueEvent("PHOTODIODE_STATUS", buildPhotodiodeStatusBody());
    return;
  }

  if (strcmp(cmd, "PHOTODIODE.COUNT") == 0) {
    enqueueEvent("PHOTODIODE_COUNT", buildPhotodiodeCountBody());
    return;
  }

  if (strcmp(cmd, "PHOTODIODE.CLEAR") == 0) {
    photodiode_clear();
    enqueueAckEvent(cmd);
    return;
  }

  // ------------------------------------------------------------
  // LASER COMMANDS
  // ------------------------------------------------------------
  if (strcmp(cmd, "LASER.ON") == 0) {
    laser_on();
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "LASER.OFF") == 0) {
    laser_off();
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "LASER.VOLTAGES") == 0) {
    String body = laser_measure_voltages();
    enqueueEvent("LASER_VOLTAGES", body);
    enqueueAckEvent(cmd);
    return;
  }

  // ------------------------------------------------------------
  // SYSTEM COMMANDS
  // ------------------------------------------------------------
  if (strcmp(cmd, "SYSTEM.SHUTDOWN") == 0) {
    enqueueAckEvent(cmd);
    system_request_shutdown();
    return;
  }

  // ------------------------------------------------------------
  // UNKNOWN COMMAND
  // ------------------------------------------------------------
  enqueueErrEvent(cmd, "unknown command");
}
