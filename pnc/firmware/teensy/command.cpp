#include "command.h"

#include "event_bus.h"
#include "util.h"
#include "system.h"

#include "laser.h"
#include "photodiode.h"
#include "gnss.h"
#include "teensy_status.h"
#include "dwt_clock.h"
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
  // STATUS / DATA COMMANDS (event-producing, durable)
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
  // QUERY COMMANDS (NON-DESTRUCTIVE, IMMEDIATE)
  // ------------------------------------------------------------
  if (strcmp(cmd, "PHOTODIODE.STATUS?") == 0) {
    emitImmediateFramed(
      "PHOTODIODE_STATUS",
      buildPhotodiodeStatusBody()
    );
    return;
  }

  if (strcmp(cmd, "DWT.COUNT?") == 0) {
    String b;
    b += "\"cycles\":";
    b += dwt_clock_read();

    emitImmediateFramed("DWT_COUNT", b);
    return;
  }

  // ------------------------------------------------------------
  // PHOTODIODE COMMANDS (event-producing)
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
  // LASER COMMANDS (ACTUATION ONLY)
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
  // SYSTEM COMMANDS (TERMINAL)
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
