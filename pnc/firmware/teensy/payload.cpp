#include "payload.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// =============================================================
// Configuration
// =============================================================

static constexpr size_t PAYLOAD_JSON_MAX = 10 * 1024; // 10 KiB

// =============================================================
// Internal helpers (serialization only)
// =============================================================
//
// JsonBuf is a bounded, allocator-free JSON construction buffer.
// It is reused across serializations and is NOT re-entrant.
// The produced JSON is transient and must be consumed immediately.
//

namespace {

struct JsonBuf {
  char   buf[PAYLOAD_JSON_MAX];
  size_t len;
  bool   overflow;

  JsonBuf() : len(0), overflow(false) {
    buf[0] = '\0';
  }

  inline size_t remaining() const {
    return PAYLOAD_JSON_MAX - len;
  }

  void append(const char* s) {
    if (overflow || !s) return;

    size_t n = strlen(s);
    if (n + 1 > remaining()) {
      overflow = true;
      return;
    }

    memcpy(buf + len, s, n);
    len += n;
    buf[len] = '\0';
  }

  void append_char(char c) {
    if (overflow) return;

    if (remaining() < 2) {
      overflow = true;
      return;
    }

    buf[len++] = c;
    buf[len] = '\0';
  }

  void append_escaped(const char* s) {
    if (overflow || !s) return;

    while (*s) {
      char c = *s++;
      switch (c) {
        case '\"': append("\\\""); break;
        case '\\': append("\\\\"); break;
        case '\n': append("\\n");  break;
        case '\r': append("\\r");  break;
        case '\t': append("\\t");  break;
        default:   append_char(c); break;
      }
      if (overflow) return;
    }
  }

  void reset_to_error() {
    const char* err = "{\"error\":\"payload_overflow\"}";
    strncpy(buf, err, PAYLOAD_JSON_MAX - 1);
    buf[PAYLOAD_JSON_MAX - 1] = '\0';
    len = strlen(buf);
    overflow = false; // terminal state
  }
};

} // namespace

/*
  ============================================================================

  Payload — Semantic State Holder
  -------------------------------
  entries[] is authoritative.
  JSON serialization is a bounded, heap-free derivative.

  json_view():
    • Returns a transient view into an internal static buffer
    • Caller must consume immediately
    • No allocation, no ownership transfer

  to_json():
    • Legacy convenience wrapper
    • Allocates exactly once
    • Should not be used on hot paths

  ============================================================================
*/

// =============================================================
// Payload
// =============================================================

Payload::Payload()
  : entry_count(0),
    raw("") {}

void Payload::clear() {
  entry_count = 0;
  raw = ""; // staging only
}

bool Payload::empty() const {
  return entry_count == 0;
}

// -------------------------------------------------------------
// JSON serialization — primary, zero-alloc path
// -------------------------------------------------------------

JsonView Payload::json_view() const {

  static JsonBuf jb;

  // Reset buffer
  jb.len = 0;
  jb.overflow = false;
  jb.buf[0] = '\0';

  jb.append_char('{');

  for (size_t i = 0; i < entry_count; i++) {

    if (i) jb.append_char(',');

    const Entry& e = entries[i];

    jb.append_char('\"');
    jb.append_escaped(e.key.c_str());
    jb.append("\":");

    if (jb.overflow) break;

    switch (e.kind) {

      case 'p': {
        const char* s = e.value.c_str();

        // Raw literals: true / false / numbers
        if (!strcmp(s, "true") || !strcmp(s, "false")) {
          jb.append(s);
        } else {
          char* end = nullptr;
          strtod(s, &end);

          if (s[0] != '\0' && end && *end == '\0') {
            jb.append(s); // valid number
          } else {
            jb.append_char('\"');
            jb.append_escaped(s);
            jb.append_char('\"');
          }
        }
        break;
      }

      case 'o':
      case 'a':
        // Pre-validated raw JSON fragments
        jb.append(e.value.c_str());
        break;

      default:
        jb.append("null");
        break;
    }

    if (jb.overflow) break;
  }

  jb.append_char('}');

  if (jb.overflow) {
    jb.reset_to_error();
  }

  return JsonView{ jb.buf, jb.len };
}

// -------------------------------------------------------------
// JSON serialization — legacy convenience wrapper
// -------------------------------------------------------------

String Payload::to_json() const {
  JsonView v = json_view();
  return String(v.data);
}

// =============================================================
// Semantic construction — unchanged
// =============================================================

void Payload::add(const char* key, const char* value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), String(value ? value : ""), 'p' };
}

void Payload::add(const char* key, const String& value) {
  add(key, value.c_str());
}

void Payload::add(const char* key, bool value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), value ? "true" : "false", 'p' };
}

void Payload::add(const char* key, float value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), String(value, 6), 'p' };
}

void Payload::add(const char* key, double value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), String(value, 6), 'p' };
}

void Payload::add_fmt(const char* key, const char* fmt, ...) {
  if (entry_count >= MAX_ENTRIES) return;

  char tmp[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);

  entries[entry_count++] = { String(key), String(tmp), 'p' };
}

void Payload::add_object(const char* key, const Payload& obj) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), obj.to_json(), 'o' };
}

void Payload::add_array(const char* key, const PayloadArray& arr) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), arr.to_json(), 'a' };
}

void Payload::add_raw_object(const char* key, const char* raw_json_object) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = {
    String(key),
    String(raw_json_object ? raw_json_object : "{}"),
    'o'
  };
}

// =============================================================
// Parsing — canonicalizes into entries[]
// =============================================================

bool Payload::parseJSON(const uint8_t* data, size_t len) {

  clear();

  if (!data || len < 2 || data[0] != '{') {
    return false;
  }

  size_t i = 1;

  while (i < len && entry_count < MAX_ENTRIES) {

    while (i < len && data[i] != '"') i++;
    if (i >= len) break;

    size_t key_start = ++i;
    while (i < len && data[i] != '"') i++;
    if (i >= len) break;

    String key =
      String((const char*)data + key_start).substring(0, i - key_start);

    i++; // skip closing quote

    while (i < len && data[i] != ':') i++;
    if (i >= len) break;
    i++;

    char kind =
      data[i] == '{' ? 'o' :
      data[i] == '[' ? 'a' :
      'p';

    size_t value_start = i;

    if (kind == 'p') {
      while (i < len && data[i] != ',' && data[i] != '}') i++;
    } else {
      int depth = 0;
      do {
        if (data[i] == '{' || data[i] == '[') depth++;
        if (data[i] == '}' || data[i] == ']') depth--;
        i++;
      } while (i < len && depth > 0);
    }

    String raw_value =
      String((const char*)data + value_start).substring(0, i - value_start);

    // Canonicalize quoted strings once
    if (kind == 'p' &&
        raw_value.length() >= 2 &&
        raw_value[0] == '"' &&
        raw_value[raw_value.length() - 1] == '"') {
      raw_value = raw_value.substring(1, raw_value.length() - 1);
    }

    entries[entry_count++] = { key, raw_value, kind };

    if (i < len && data[i] == ',') i++;
  }

  return true;
}


// =============================================================
// Lookup / Accessors
// =============================================================

const Payload::Entry* Payload::find(const char* key) const {
  for (size_t i = 0; i < entry_count; i++) {
    if (entries[i].key == key) return &entries[i];
  }
  return nullptr;
}

bool Payload::has(const char* key) const {
  return find(key) != nullptr;
}

// -------------------------------------------------------------
// Primitive accessors — owned, stable, order-independent
// -------------------------------------------------------------

const char* Payload::getString(const char* key) const {
  const Entry* e = find(key);
  if (!e || e->kind != 'p') return nullptr;
  return e->value.c_str();
}

// =============================================================
// Strict accessors
// =============================================================

bool Payload::tryGetBool(const char* key, bool& out) const {
  const char* s = getString(key);
  if (!s) return false;
  if (!strcmp(s, "true"))  { out = true;  return true; }
  if (!strcmp(s, "false")) { out = false; return true; }
  return false;
}

bool Payload::tryGetInt(const char* key, int32_t& out) const {
  const char* s = getString(key);
  if (!s) return false;
  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (!end || *end != '\0') return false;
  out = (int32_t)v;
  return true;
}

bool Payload::tryGetUInt(const char* key, uint32_t& out) const {
  const char* s = getString(key);
  if (!s) return false;
  char* end = nullptr;
  unsigned long v = strtoul(s, &end, 10);
  if (!end || *end != '\0') return false;
  out = (uint32_t)v;
  return true;
}

// Compatibility overload for existing callers
uint32_t Payload::getUInt(const char* key, unsigned long default_value) const {
  uint32_t v;
  return tryGetUInt(key, v) ? v : (uint32_t)default_value;
}

bool Payload::tryGetFloat(const char* key, float& out) const {
  const char* s = getString(key);
  if (!s) return false;
  char* end = nullptr;
  float v = strtof(s, &end);
  if (!end || *end != '\0') return false;
  out = v;
  return true;
}

bool Payload::tryGetDouble(const char* key, double& out) const {
  const char* s = getString(key);
  if (!s) return false;
  char* end = nullptr;
  double v = strtod(s, &end);
  if (!end || *end != '\0') return false;
  out = v;
  return true;
}

// =============================================================
// Structured accessors
// =============================================================

Payload Payload::getPayload(const char* key) const {
  Payload p;
  const Entry* e = find(key);
  if (!e || e->kind != 'o') return p;
  p.parseJSON((const uint8_t*)e->value.c_str(), e->value.length());
  return p;
}

PayloadArray Payload::getArray(const char* key) const {
  PayloadArray arr;
  const Entry* e = find(key);
  if (!e || e->kind != 'a') return arr;
  arr.parseJSON(e->value.c_str());
  return arr;
}

// =============================================================
// Diagnostics
// =============================================================

void Payload::debug_dump(const char* tag) const {

  char line[128];

  snprintf(
    line,
    sizeof(line),
    "Payload dump (%s): %u entr%s",
    tag ? tag : "",
    (unsigned)entry_count,
    entry_count == 1 ? "y" : "ies"
  );
  debug_log("payload", line);

  for (size_t i = 0; i < entry_count; i++) {

    const Entry& e = entries[i];

    const char* kind =
      e.kind == 'p' ? "primitive" :
      e.kind == 'o' ? "object" :
      e.kind == 'a' ? "array" :
      "unknown";

    debug_log("payload.key", e.key.c_str());
    debug_log("payload.kind", kind);

    const char* v = e.value.c_str();

    char preview[96];
    size_t n = strlen(v);
    if (n > sizeof(preview) - 1) {
      n = sizeof(preview) - 1;
    }
    memcpy(preview, v, n);
    preview[n] = '\0';

    debug_log("payload.value", preview);
  }
}

// =============================================================
// PayloadArray — restored for compatibility
// =============================================================

PayloadArray::PayloadArray()
  : buf(""),
    first(true),
    item_count(0) {}

void PayloadArray::clear() {
  buf = "";
  first = true;
  item_count = 0;
}

bool PayloadArray::empty() const {
  return item_count == 0 && buf.length() == 0;
}

String PayloadArray::to_json() const {

  if (item_count > 0) {
    String out = "[";
    for (size_t i = 0; i < item_count; i++) {
      if (i) out += ",";
      out += items[i].to_json();
    }
    out += "]";
    return out;
  }

  return String("[") + buf + "]";
}

void PayloadArray::add(const Payload& obj) {
  if (!first) buf += ",";
  first = false;
  buf += obj.to_json();
}

bool PayloadArray::parseJSON(const char* json) {

  item_count = 0;
  if (!json || json[0] != '[') return false;

  int i = 1; // skip '['
  while (json[i] && item_count < MAX_ITEMS) {

    while (json[i] && json[i] != '{') i++;
    if (!json[i]) break;

    int v0 = i;
    int depth = 0;

    do {
      if (json[i] == '{') depth++;
      if (json[i] == '}') depth--;
      i++;
    } while (depth > 0 && json[i]);

    items[item_count].parseJSON(
      (const uint8_t*)(json + v0),
      (size_t)(i - v0)
    );

    item_count++;
  }

  return true;
}

size_t PayloadArray::size() const {
  return item_count;
}

Payload PayloadArray::get(size_t idx) const {
  if (idx >= item_count) return Payload();
  return items[idx];
}
