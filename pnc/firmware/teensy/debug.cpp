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
// TX rule (critical):
//   • debug.cpp MUST NOT call transport_send() directly.
//   • All transport_send() calls occur ONLY inside a TimePOP callback,
//     so transport TX remains non-logging, non-reentrant, and timing-truthful.
//
// Scheduling note:
//   • TIMEPOP_CLASS_DEBUG_TX should be configured to run with a long-ish
//     latency budget (e.g., ~5 ms) so debug emission never competes with
//     tight transport/clock paths.
//

#include "debug.h"
#include "transport.h"
#include "payload.h"
#include "timepop.h"

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// =============================================================
// Debug TX helper (TimePOP-only transport_send)
// =============================================================

struct debug_send_ctx_t {
  Payload payload;
};

static void debug_send_async(const Payload& p) {

  return;

  // Deep copy: caller may pass stack Payload; we must outlive the call site.
  auto* ctx = new debug_send_ctx_t{ p.clone() };

  timepop_arm(
    TIMEPOP_CLASS_DEBUG_TX,   // configure to ~5ms cadence/latency
    false,                    // one-shot
    [](timepop_ctx_t*, void* user_ctx) {

      auto* send = static_cast<debug_send_ctx_t*>(user_ctx);

      // Contract: the ONLY place debug.cpp may call transport_send()
      transport_send(TRAFFIC_DEBUG, send->payload);

      delete send;
    },
    ctx,
    "debug-send"
  );
}

// =============================================================
// Initialization
// =============================================================

void debug_init(void) {
  // Caller-supplied identity is still respected: we supply it here.
  Payload out;
  out.add("name",  "init");
  out.add("value", "ZPNet Debug Online");
  debug_send_async(out);
}

void debug_beacon(void) {
  timepop_arm(
    TIMEPOP_CLASS_DEBUG_BEACON,
    true,
    [](timepop_ctx_t*, void*) {
      Payload out;
      out.add("name",  "beacon");
      out.add("value", "beacon");
      debug_send_async(out);
    },
    nullptr,
    "debug_beacon"
  );
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

  debug_send_async(out);
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

  JsonView v = value.json_view();
  out.add_raw_object("value", v.data);

  debug_send_async(out);
}

// =============================================================
// Named scalar overloads (minimal, no caller transforms)
//
// Emits:
//   { "name": "<name>", "value": <scalar> }
// =============================================================

void debug_log(const char* name, int v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  debug_send_async(out);
}

void debug_log(const char* name, unsigned int v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  debug_send_async(out);
}

void debug_log(const char* name, int32_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  debug_send_async(out);
}

void debug_log(const char* name, uint32_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  debug_send_async(out);
}

void debug_log(const char* name, int64_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", (long long)v);
  debug_send_async(out);
}

void debug_log(const char* name, uint64_t v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", (unsigned long long)v);
  debug_send_async(out);
}

void debug_log(const char* name, float v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  debug_send_async(out);
}

void debug_log(const char* name, double v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  debug_send_async(out);
}

void debug_log(const char* name, bool v) {
  if (!name || !*name) return;
  Payload out;
  out.add("name", name);
  out.add("value", v);
  debug_send_async(out);
}

void debug_log(const char* name, const void* ptr) {
  if (!name || !*name) return;

  char buf[32];
  snprintf(buf, sizeof(buf), "0x%lx", (unsigned long)ptr);

  Payload out;
  out.add("name", name);
  out.add("value", buf);

  debug_send_async(out);
}

// =============================================================
// Named buffer dump (hex string, literal)
//
// Emits:
//   { "name": "<name>", "value": "<hex bytes>" }
// =============================================================

void debug_log(const char* name, const uint8_t* buf, size_t len) {
  if (!name || !*name) return;

  if (!buf || len == 0) {
    Payload out;
    out.add("name", name);
    out.add("value", "");
    debug_send_async(out);
    return;
  }

  // Bounded hex rendering.
  // 2 chars per byte + 1 space per byte (minus last) => ~3*len
  // Keep it small and predictable.
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

  debug_send_async(out);
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
