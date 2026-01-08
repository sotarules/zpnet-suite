#include "command.h"

#include "event_bus.h"
#include "util.h"
#include "system.h"

#include "laser.h"
#include "photodiode.h"
#include "gnss.h"
#include "teensy_status.h"

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
// Immediate JSON emit (QUERY plane only)
// --------------------------------------------------------------
//
// Used ONLY for non-destructive CMD? queries.
// Bypasses the event queue entirely.
//
static inline void emitImmediateJson(const char* type, const String& body) {
  Serial.print("{\"type\":\"");
  Serial.print(type);
  Serial.print("\"");
  if (body.length()) {
    Serial.print(",");
    Serial.print(body);
  }
  Serial.println("}");
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
  // STATUS / DATA COMMANDS (event-producing)
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
    emitImmediateJson(
      "PHOTODIODE_STATUS",
      buildPhotodiodeStatusBody()
    );
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
    enqueueEvent("LASER_STATE", "\"enabled\":true");
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "LASER.OFF") == 0) {
    laser_off();
    enqueueEvent("LASER_STATE", "\"enabled\":false");
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
  // UNKNOWN
  // ------------------------------------------------------------
  enqueueErrEvent(cmd, "unknown command");
}
