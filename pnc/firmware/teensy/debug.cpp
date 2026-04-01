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
// Blink facility:
//   • Uses DWT cycle-counting busy-wait (not delay()/millis()).
//   • Works with interrupts disabled.
//   • Works in hard fault handlers.
//   • Works at any CPU clock speed.
//   • No SysTick dependency.
//

#include "debug.h"
#include "transport.h"
#include "payload.h"

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// =============================================================
// DWT-based busy-wait (fault-safe, ISR-safe, clock-safe)
// =============================================================
//
// Uses ARM DWT cycle counter for timing.  Does not depend on
// SysTick, millis(), delay(), or any ISR.  Works correctly
// even inside hard fault handlers with interrupts disabled.
//
// F_CPU_ACTUAL is a runtime global updated by set_arm_clock().
// At 1008 MHz: 1 ms = 1,008,000 cycles.
// At 600 MHz:  1 ms = 600,000 cycles.
//

static void dwt_delay_ms(uint32_t ms) {
  if (ms == 0) return;

  // Ensure DWT is enabled (may not be if called very early or in fault)
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  // Compute cycles to wait.
  // F_CPU_ACTUAL may be 0 if called before set_arm_clock().
  // Fall back to 600 MHz (default Teensy 4.1 clock).
  uint32_t freq = F_CPU_ACTUAL;
  if (freq == 0) freq = 600000000UL;

  uint32_t cycles_per_ms = freq / 1000;

  for (uint32_t i = 0; i < ms; i++) {
    uint32_t start = ARM_DWT_CYCCNT;
    while ((ARM_DWT_CYCCNT - start) < cycles_per_ms) {
      // spin
    }
  }
}

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

  char local_buf[4096];
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
// Blink facility (DWT-based, fault-safe)
// =============================================================
//
// Pattern digits define blink duration units.
// Example: "911" = 9-unit ON, 1-unit ON, 1-unit ON
//
// Uses dwt_delay_ms() — works with interrupts disabled,
// works inside hard fault handlers, works at any CPU clock.
//

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
    dwt_delay_ms(units * BLINK_UNIT_MS);
    digitalWrite(LED_BUILTIN, LOW);
    dwt_delay_ms(BLINK_GAP_MS);
  }

  dwt_delay_ms(PATTERN_GAP_MS);
}

void debug_sleep_ms(uint32_t ms) {
  dwt_delay_ms(ms);
}