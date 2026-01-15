#include "command/command.h"

#include "event/event_bus.h"
#include "util/util.h"
#include "core/system.h"

#include "subsystems/laser/laser.h"
#include "subsystems/photodiode/photodiode.h"
#include "subsystems/gnss/gnss.h"
#include "subsystems/status/teensy_status.h"

#include "clock/dwt_clock.h"
#include "clock/qtimer.h"
#include "clock/gpt_count.h"

#include "transport/transport.h"

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
//
// Used ONLY for non-destructive CMD? queries.
// Bypasses the event queue entirely.
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
// Command execution
// --------------------------------------------------------------
void command_exec(const char* line) {
  char cmd[64];

  if (!extractCmd(line, cmd, sizeof(cmd))) {
    enqueueErrEvent("UNKNOWN", "missing cmd");
    return;
  }

  // ------------------------------------------------------------
  // STATUS / DATA COMMANDS (durable events)
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // QUERY COMMANDS (non-destructive, immediate)
  // ------------------------------------------------------------
  if (strcmp(cmd, "PHOTODIODE.STATUS?") == 0) {
    emitImmediateFramed(
      "PHOTODIODE_STATUS",
      buildPhotodiodeStatusBody()
    );
    return;
  }

  // ------------------------------------------------------------
  // QTIMER COMMANDS
  // ------------------------------------------------------------
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
  // GPT CONFIRM (durable calibration event)
  // ------------------------------------------------------------
  if (strcmp(cmd, "GPT.CONFIRM") == 0) {

    uint64_t seconds = 2;
    extractUintArg(line, "\"seconds\"", &seconds);

    if (seconds == 0) seconds = 1;
    if (seconds > 3600) seconds = 3600;

    const uint64_t TARGET_EXT_TICKS =
        seconds * 10000000ULL;

    uint64_t cpu_cycles = 0;
    double ratio = 0.0;
    int64_t error_cycles = 0;

    uint64_t gpt_count = gpt_count_confirm(
        TARGET_EXT_TICKS,
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
  // LASER COMMANDS (actuation only)
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
  // SYSTEM COMMANDS (terminal)
  // ------------------------------------------------------------
  if (strcmp(cmd, "SYSTEM.SHUTDOWN") == 0) {
    enqueueAckEvent(cmd);
    system_request_shutdown();
    // no return
  }

  // ------------------------------------------------------------
  // EVENT BUS
  // ------------------------------------------------------------
  if (strcmp(cmd, "EVENTS.GET") == 0) {
    drainEventsNow();
    return;
  }

  // ------------------------------------------------------------
  // UNKNOWN COMMAND
  // ------------------------------------------------------------
  enqueueErrEvent(cmd, "unknown command");
}
