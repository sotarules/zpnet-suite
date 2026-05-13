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
// TX rule (post-direct-emission refactor):
//   • debug_log writes the framed wire bytes DIRECTLY to Serial.
//   • There is no transport queue, no TimePop dependency, no heap.
//   • This makes debug_log usable from ISR context, from foreground,
//     from inside critical sections, and from situations where TimePop
//     or the transport TX pump has stopped servicing.
//
// Wire format (matches transport.cpp's framed assembly):
//
//   0xD0 <STX=NNN>{"name":"...","value":...}<ETX>
//
//   Where:
//     • 0xD0 is the TRAFFIC_DEBUG traffic byte.
//     • <STX=NNN> is the literal ASCII string with NNN being the JSON
//       byte length in decimal.
//     • The JSON payload is exactly the form transport_send would have
//       produced, but built directly with snprintf into a stack buffer
//       so no Payload heap allocations occur.
//     • <ETX> is the literal ASCII string.
//
// Interleave caveat:
//   debug_log calls Serial.write() directly.  The transport TX pump also
//   calls Serial.write() (via serial_send_bytes()).  If an ISR-context
//   debug_log fires while the pump's Serial.write() is mid-execution,
//   the bytes can interleave on the wire and break framing for one or
//   two frames.  The Pi-side RX state machine handles this gracefully
//   via its rx_bad_stx / rx_bad_etx counters and resynchronizes on the
//   next clean STX.  Debug emission is best-effort by design; this is
//   acceptable.
//
// Buffer sizing:
//   Every emit uses a 1024-byte stack buffer for the assembled frame.
//   This is large enough for typical debug payloads and small enough
//   to be safe from any ISR context.  Frames that would exceed this
//   are dropped (incrementing debug_dropped_count).
//
// ISR safety:
//   The string, scalar, bool, and buffer overloads of debug_log are
//   safe to call from any context, including ISRs, because they do no
//   heap allocation, take no locks, and rely only on stack memory plus
//   the Serial peripheral.  The Payload overload is foreground-only by
//   virtue of needing a constructed Payload — Payload's constructor and
//   .add() methods allocate heap, so the Payload itself cannot be
//   built from an ISR.  Once constructed, however, write_json is pure
//   and the rest of the emit path is ISR-safe.
//
// Blink facility:
//   • Uses DWT cycle-counting busy-wait (not delay()/millis()).
//   • Works with interrupts disabled.
//   • Works in hard fault handlers.
//   • Works at any CPU clock speed.
//   • No SysTick dependency.
//

#include "debug.h"
#include "transport.h"   // TRAFFIC_DEBUG constant only
#include "payload.h"

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// =============================================================
// Direct-emission framing constants
// =============================================================
//
// These mirror the transport.cpp framing without depending on it.
// The byte sequences must match exactly so the Pi-side RX state
// machine accepts the frames.

static constexpr uint8_t TRAFFIC_BYTE = TRAFFIC_DEBUG;  // 0xD0

static constexpr char   STX_OPEN_SEQ[]  = "<STX=";
static constexpr size_t STX_OPEN_LEN    = 5;
static constexpr char   STX_CLOSE_CHAR  = '>';

static constexpr char   ETX_SEQ[]       = "<ETX>";
static constexpr size_t ETX_LEN         = 5;

// Maximum total wire bytes per emission (1 traffic + header + JSON + ETX).
// Sized to be comfortable from any ISR context.
static constexpr size_t EMIT_BUFFER_SIZE = 1024;

// =============================================================
// Drop counter (best-effort observability)
// =============================================================
//
// Incremented whenever debug_log cannot emit a message — either because
// the assembled frame would exceed EMIT_BUFFER_SIZE, or because the
// caller passed invalid arguments.  This is a static so it survives the
// life of the program; it can be inspected via a debugger or surfaced
// in a future report extension by adding a public accessor.

static volatile uint32_t debug_dropped_count = 0;

// =============================================================
// JSON string escaping
// =============================================================
//
// Copies src into dst while escaping characters that JSON requires
// to be escaped.  Returns bytes written on success, or 0 if the
// destination buffer is too small.  Always advances dst on success;
// never writes past dst_max.

static size_t json_escape_into(char* dst, size_t dst_max, const char* src) {
  if (!dst || dst_max == 0 || !src) return 0;

  size_t pos = 0;
  for (const char* p = src; *p; p++) {
    unsigned char c = (unsigned char)*p;

    // Worst case: a single source byte expands to "\uXXXX" (6 bytes).
    // Conservatively require 6 bytes of headroom on every loop pass.
    if (pos + 6 >= dst_max) return 0;

    switch (c) {
      case '"':  dst[pos++] = '\\'; dst[pos++] = '"';  break;
      case '\\': dst[pos++] = '\\'; dst[pos++] = '\\'; break;
      case '\b': dst[pos++] = '\\'; dst[pos++] = 'b';  break;
      case '\f': dst[pos++] = '\\'; dst[pos++] = 'f';  break;
      case '\n': dst[pos++] = '\\'; dst[pos++] = 'n';  break;
      case '\r': dst[pos++] = '\\'; dst[pos++] = 'r';  break;
      case '\t': dst[pos++] = '\\'; dst[pos++] = 't';  break;
      default:
        if (c < 0x20) {
          int n = snprintf(dst + pos, dst_max - pos, "\\u%04x", c);
          if (n <= 0 || (size_t)n >= dst_max - pos) return 0;
          pos += (size_t)n;
        } else {
          dst[pos++] = (char)c;
        }
        break;
    }
  }
  return pos;
}

// =============================================================
// Emit primitive
// =============================================================
//
// Builds the wire frame:
//
//   0xD0<STX=NNN>{json_body}<ETX>
//
// json_body is the caller-supplied raw bytes that go between the
// outer braces would normally enclose; this function adds nothing
// to or interprets the JSON content beyond computing its length.
//
// Returns true on successful Serial.write, false if the frame was
// dropped (assembled size exceeded the stack buffer).

static bool emit_framed_json(const char* json_body, size_t json_len) {
  if (!json_body || json_len == 0) {
    debug_dropped_count++;
    return false;
  }

  // Build header: "<STX=NNN>" — NNN is the decimal JSON length.
  // Max NNN for a frame that fits in EMIT_BUFFER_SIZE: ~1000 → 4 digits.
  char header[16];
  int header_len = snprintf(header, sizeof(header), "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0 || (size_t)header_len >= sizeof(header)) {
    debug_dropped_count++;
    return false;
  }

  // Total wire size: 1 (traffic) + header + json + ETX.
  const size_t total_len = 1 + (size_t)header_len + json_len + ETX_LEN;
  if (total_len > EMIT_BUFFER_SIZE) {
    debug_dropped_count++;
    return false;
  }

  // Assemble in one contiguous stack buffer so the Serial.write call
  // is a single atomic copy into the USB TX buffer.
  uint8_t buf[EMIT_BUFFER_SIZE];
  size_t pos = 0;
  buf[pos++] = TRAFFIC_BYTE;
  memcpy(buf + pos, header, (size_t)header_len);
  pos += (size_t)header_len;
  memcpy(buf + pos, json_body, json_len);
  pos += json_len;
  memcpy(buf + pos, ETX_SEQ, ETX_LEN);
  pos += ETX_LEN;

  // Single Serial.write — no flush.  Flush would block waiting for
  // the USB host to drain the buffer; we need non-blocking from ISR
  // contexts.  Drop bytes if the USB buffer is full (Serial.write
  // returns fewer bytes than requested) rather than spin.
  Serial.write(buf, pos);
  return true;
}

// =============================================================
// JSON body builders
// =============================================================
//
// Each builder constructs the {"name":"<n>","value":<v>} JSON body
// into a caller-supplied stack buffer.  Returns the number of bytes
// written, or 0 on overflow.  Callers pass the buffer through to
// emit_framed_json without modification.

static size_t build_body_open(char* dst, size_t dst_max, const char* name) {
  if (!dst || dst_max == 0 || !name) return 0;

  // {"name":"<escaped_name>","value":
  // The literal prefix is 10 bytes: {"name":"
  // After escaped name we add 11 bytes: ","value":
  size_t pos = 0;
  const char prefix[] = "{\"name\":\"";
  const size_t prefix_len = sizeof(prefix) - 1;
  if (pos + prefix_len >= dst_max) return 0;
  memcpy(dst + pos, prefix, prefix_len);
  pos += prefix_len;

  const size_t name_escaped =
      json_escape_into(dst + pos, dst_max - pos, name);
  if (name_escaped == 0) return 0;
  pos += name_escaped;

  const char mid[] = "\",\"value\":";
  const size_t mid_len = sizeof(mid) - 1;
  if (pos + mid_len >= dst_max) return 0;
  memcpy(dst + pos, mid, mid_len);
  pos += mid_len;

  return pos;
}

static size_t build_body_close(char* dst, size_t dst_max, size_t pos) {
  if (pos == 0) return 0;
  if (pos >= dst_max) return 0;
  dst[pos++] = '}';
  return pos;
}

// Build body for string-valued debug_log.
// Produces: {"name":"<n>","value":"<v>"}
static size_t build_body_string(char* dst, size_t dst_max,
                                const char* name, const char* value) {
  if (!value) value = "";

  size_t pos = build_body_open(dst, dst_max, name);
  if (pos == 0) return 0;

  if (pos + 1 >= dst_max) return 0;
  dst[pos++] = '"';

  const size_t value_escaped =
      json_escape_into(dst + pos, dst_max - pos, value);
  // value_escaped==0 with non-empty value is overflow; with empty value
  // it's the legitimate zero-length result.
  if (value_escaped == 0 && *value != '\0') return 0;
  pos += value_escaped;

  if (pos + 1 >= dst_max) return 0;
  dst[pos++] = '"';

  return build_body_close(dst, dst_max, pos);
}

// Build body for raw-JSON-valued debug_log (Payload overload).
// Produces: {"name":"<n>","value":<raw_json>}
// The caller is responsible for ensuring raw_json is valid JSON.
static size_t build_body_raw_json(char* dst, size_t dst_max,
                                  const char* name,
                                  const char* raw_json, size_t raw_json_len) {
  if (!raw_json || raw_json_len == 0) return 0;

  size_t pos = build_body_open(dst, dst_max, name);
  if (pos == 0) return 0;

  if (pos + raw_json_len >= dst_max) return 0;
  memcpy(dst + pos, raw_json, raw_json_len);
  pos += raw_json_len;

  return build_body_close(dst, dst_max, pos);
}

// Build body for a value already formatted into ASCII (numbers, bools).
// Produces: {"name":"<n>","value":<literal>}
// The literal is copied verbatim — caller must ensure it is valid JSON.
static size_t build_body_literal(char* dst, size_t dst_max,
                                 const char* name, const char* literal) {
  if (!literal) literal = "null";

  size_t pos = build_body_open(dst, dst_max, name);
  if (pos == 0) return 0;

  const size_t lit_len = strlen(literal);
  if (pos + lit_len >= dst_max) return 0;
  memcpy(dst + pos, literal, lit_len);
  pos += lit_len;

  return build_body_close(dst, dst_max, pos);
}

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
  debug_log("init", "ZPNet Debug Online");
}

// =============================================================
// Named string logging
// =============================================================

void debug_log(const char* name, const char* msg) {
  if (!name || !*name) return;

  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_string(body, sizeof(body), name, msg);
  if (body_len == 0) {
    debug_dropped_count++;
    return;
  }
  emit_framed_json(body, body_len);
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

  // Render the Payload into a local JSON buffer.  This buffer is sized
  // generously but bounded; oversized Payloads will produce an empty
  // body via write_json's overflow contract and we emit {} as a safe
  // placeholder rather than dropping the message.
  char json_buf[EMIT_BUFFER_SIZE / 2];
  size_t json_len = value.write_json(json_buf, sizeof(json_buf));
  const char* raw_json = (json_len > 0) ? json_buf : "{}";
  size_t      raw_len  = (json_len > 0) ? json_len : 2;

  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_raw_json(body, sizeof(body),
                                        name, raw_json, raw_len);
  if (body_len == 0) {
    debug_dropped_count++;
    return;
  }
  emit_framed_json(body, body_len);
}

// =============================================================
// Named scalar overloads
//
// Emits:
//   { "name": "<name>", "value": <scalar> }
// =============================================================

void debug_log(const char* name, int v) {
  if (!name || !*name) return;
  char lit[32];
  snprintf(lit, sizeof(lit), "%d", v);
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, unsigned int v) {
  if (!name || !*name) return;
  char lit[32];
  snprintf(lit, sizeof(lit), "%u", v);
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, int32_t v) {
  if (!name || !*name) return;
  char lit[32];
  snprintf(lit, sizeof(lit), "%ld", (long)v);
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, uint32_t v) {
  if (!name || !*name) return;
  char lit[32];
  snprintf(lit, sizeof(lit), "%lu", (unsigned long)v);
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, int64_t v) {
  if (!name || !*name) return;
  char lit[32];
  snprintf(lit, sizeof(lit), "%lld", (long long)v);
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, uint64_t v) {
  if (!name || !*name) return;
  char lit[32];
  snprintf(lit, sizeof(lit), "%llu", (unsigned long long)v);
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, float v) {
  if (!name || !*name) return;
  char lit[32];
  // NaN and Inf are not valid JSON numbers; emit as null to keep the
  // frame parseable on the Pi side.
  if (isnan(v) || isinf(v)) {
    snprintf(lit, sizeof(lit), "null");
  } else {
    snprintf(lit, sizeof(lit), "%.7g", (double)v);
  }
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, double v) {
  if (!name || !*name) return;
  char lit[32];
  if (isnan(v) || isinf(v)) {
    snprintf(lit, sizeof(lit), "null");
  } else {
    snprintf(lit, sizeof(lit), "%.15g", v);
  }
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body), name, lit);
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, bool v) {
  if (!name || !*name) return;
  char body[EMIT_BUFFER_SIZE];
  size_t body_len = build_body_literal(body, sizeof(body),
                                       name, v ? "true" : "false");
  if (body_len == 0) { debug_dropped_count++; return; }
  emit_framed_json(body, body_len);
}

void debug_log(const char* name, const void* ptr) {
  if (!name || !*name) return;

  // Emit pointers as a JSON string (e.g. "0x20001f80") so the JSON parser
  // doesn't try to interpret them as numbers.
  char ptr_str[32];
  snprintf(ptr_str, sizeof(ptr_str), "0x%lx", (unsigned long)ptr);
  debug_log(name, ptr_str);
}

// =============================================================
// Named buffer dump (hex string)
// =============================================================

void debug_log(const char* name, const uint8_t* buf, size_t len) {
  if (!name || !*name) return;

  if (!buf || len == 0) {
    debug_log(name, "");
    return;
  }

  // Build hex line in a local buffer.  Cap at a reasonable length so
  // the resulting frame fits in EMIT_BUFFER_SIZE; long buffers are
  // truncated rather than dropped.  Each input byte yields up to 3
  // output chars ("FF "), so cap input bytes at ~200 to keep the line
  // under 600 bytes (plus framing).
  static constexpr size_t MAX_HEX_BYTES = 200;
  const size_t emit_len = (len > MAX_HEX_BYTES) ? MAX_HEX_BYTES : len;

  char line[3 * MAX_HEX_BYTES + 1];
  size_t pos = 0;
  for (size_t i = 0; i < emit_len; i++) {
    pos += snprintf(line + pos, sizeof(line) - pos, "%02X", buf[i]);
    if (i + 1 < emit_len && pos + 1 < sizeof(line)) {
      line[pos++] = ' ';
    }
  }
  line[pos] = '\0';

  debug_log(name, line);
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