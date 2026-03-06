// debug.cpp
//
// ZPNet Debug Facility (Teensy side)
//
// Contract:
//   • Every log includes caller-supplied "name" (mandatory).
//   • No prefixes, timestamps, or hidden metadata.
//   • Payload values render as REAL JSON objects (never stringified).
//   • Callers should not need transforms for common types.
//
// TX rule (updated):
//   • debug.cpp may call transport_send() directly.
//   • transport_send() now enqueues only.
//   • Physical transmission is handled by transport TX pump.
//   • No async scheduling required here.
//

#include "debug.h"
#include "transport.h"
#include "payload.h"

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// =============================================================
// Initialization
// =============================================================

void debug_init(void) {
  Payload out;
  out.add("name",  "init");
  out.add("value", "ZPNet Debug Online");

  transport_send(TRAFFIC_DEBUG, out);
}

// =============================================================
// Named string logging
// =============================================================

void debug_log(const char* name, const char* msg) {
  if (!name || !*name) return;
  if (!msg) msg = "";

  Payload out;
  out.add("name", name);
  out.add("value", msg);

  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, const String& msg) {
  debug_log(name, msg.c_str());
}

// =============================================================
// Named Payload logging (KEY FEATURE)
//
// Emits EXACTLY:
//
// {
//   "name":  "<name>",
//   "value": { ... }     // real JSON object
// }
//
// No escaping, no quotes around the object.
// =============================================================

void debug_log(const char* name, const Payload& value) {
  if (!name || !*name) return;

  Payload out;
  out.add("name", name);

  char local_buf[3072];
  size_t len = value.write_json(local_buf, sizeof(local_buf));
  out.add_raw_object("value", len > 0 ? local_buf : "{}");

  transport_send(TRAFFIC_DEBUG, out);
}

// =============================================================
// Named scalar overloads
//
// Emits:
//   { "name": "<name>", "value": <scalar> }
// =============================================================

void debug_log(const char* name, int v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, unsigned int v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, int32_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, uint32_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, int64_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", (long long)v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, uint64_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", (unsigned long long)v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, float v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, double v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, bool v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  transport_send(TRAFFIC_DEBUG, out);
}

void debug_log(const char* name, const void* ptr) {
  if (!name || !*name) return;

  char buf[32];
  snprintf(buf, sizeof(buf), "0x%lx", (unsigned long)ptr);

  Payload out;
  out.add("name", name);
  out.add("value", buf);

  transport_send(TRAFFIC_DEBUG, out);
}

// =============================================================
// Named buffer dump (hex string)
// =============================================================

void debug_log(const char* name, const uint8_t* buf, size_t len) {
  if (!name || !*name) return;

  if (!buf || len == 0) {
    Payload out;
    out.add("name", name);
    out.add("value", "");
    transport_send(TRAFFIC_DEBUG, out);
    return;
  }

  char line[256];
  size_t pos = 0;

  for (size_t i = 0; i < len && pos + 3 < sizeof(line); i++) {
    pos += snprintf(line + pos, sizeof(line) - pos, "%02X", buf[i]);
    if (i + 1 < len && pos + 1 < sizeof(line)) {
      line[pos++] = ' ';
    }
  }
  line[pos] = '\0';

  Payload out;
  out.add("name", name);
  out.add("value", line);

  transport_send(TRAFFIC_DEBUG, out);
}

// =============================================================
// Blink primitive (physical, unchanged)
// =============================================================

static constexpr uint32_t BLINK_UNIT_MS  = 150;
static constexpr uint32_t BLINK_GAP_MS   = 150;
static constexpr uint32_t PATTERN_GAP_MS = 600;

void debug_blink(const char* pattern) {
  if (!pattern || !*pattern) return;

  pinMode(LED_BUILTIN, OUTPUT);

  for (const char* p = pattern; *p; ++p) {
    char c = *p;
    if (c < '0' || c > '9') continue;

    uint8_t units = (uint8_t)(c - '0');
    if (units == 0) continue;

    digitalWrite(LED_BUILTIN, HIGH);
    delay(units * BLINK_UNIT_MS);
    digitalWrite(LED_BUILTIN, LOW);
    delay(BLINK_GAP_MS);
  }

  delay(PATTERN_GAP_MS);
}
