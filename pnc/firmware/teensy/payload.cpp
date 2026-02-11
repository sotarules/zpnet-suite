#include "payload.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// =============================================================
// Payload
// =============================================================

Payload::Payload()
  : entry_count(0) {}

void Payload::clear() {
  entry_count = 0;
}

bool Payload::empty() const {
  return entry_count == 0;
}

// -------------------------------------------------------------
// JSON serialization — primary, zero-alloc path (per-instance)
// -------------------------------------------------------------

JsonView Payload::json_view() const {

  JsonBuf& jb = _json_buf;

  // -----------------------------------------------------------
  // Local helpers (behavior lives here, not in JsonBuf)
  // -----------------------------------------------------------

  auto remaining = [&]() -> size_t {
    return JsonBuf::JSON_MAX - jb.len;
  };

  auto append = [&](const char* s) {
    if (jb.overflow || !s) return;

    size_t n = strlen(s);
    if (n + 1 > remaining()) {
      jb.overflow = true;
      return;
    }

    memcpy(jb.buf + jb.len, s, n);
    jb.len += n;
    jb.buf[jb.len] = '\0';
  };

  auto append_char = [&](char c) {
    if (jb.overflow) return;

    if (remaining() < 2) {
      jb.overflow = true;
      return;
    }

    jb.buf[jb.len++] = c;
    jb.buf[jb.len] = '\0';
  };

  auto append_escaped = [&](const char* s) {
    if (jb.overflow || !s) return;

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
      if (jb.overflow) return;
    }
  };

  // -----------------------------------------------------------
  // Initialize buffer
  // -----------------------------------------------------------

  jb.len = 0;
  jb.overflow = false;
  jb.buf[0] = '\0';

  // -----------------------------------------------------------
  // Serialize object
  // -----------------------------------------------------------

  append_char('{');

  for (size_t i = 0; i < entry_count; i++) {

    if (i) append_char(',');

    const Entry& e = entries[i];

    append_char('\"');
    append_escaped(e.key.c_str());
    append("\":");

    if (jb.overflow) break;

    switch (e.kind) {

      case 'p': {
        const char* s = e.value.c_str();

        if (!strcmp(s, "true") || !strcmp(s, "false")) {
          append(s);
        } else {
          char* end = nullptr;
          strtod(s, &end);

          if (s[0] != '\0' && end && *end == '\0') {
            append(s); // numeric
          } else {
            append_char('\"');
            append_escaped(s);
            append_char('\"');
          }
        }
        break;
      }

      case 'o':
      case 'a':
        append(e.value.c_str());
        break;

      default:
        append("null");
        break;
    }

    if (jb.overflow) break;
  }

  append_char('}');

  // -----------------------------------------------------------
  // Overflow handling (truth-preserving fallback)
  // -----------------------------------------------------------

  if (jb.overflow) {
    const char* err = "{\"error\":\"payload_overflow\"}";
    strncpy(jb.buf, err, JsonBuf::JSON_MAX - 1);
    jb.buf[JsonBuf::JSON_MAX - 1] = '\0';
    jb.len = strlen(jb.buf);
    jb.overflow = false; // terminal state
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

// -------------------------------------------------------------
// Cloning — produces a deep copy of the payload, including all entries
// -------------------------------------------------------------

Payload Payload::clone() const {
  Payload out;

  out.entry_count = entry_count;

  for (size_t i = 0; i < entry_count; i++) {
    out.entries[i] = entries[i];  // value copy (String handles its own heap)
  }

  // JSON buffer is derived; ensure it starts clean
  out._json_buf.len = 0;
  out._json_buf.overflow = false;
  out._json_buf.buf[0] = '\0';

  return out;
}

// =============================================================
// Add
// =============================================================

void Payload::add(const char* key, int32_t value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), String(value), 'p' };
}

void Payload::add(const char* key, uint32_t value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), String(value), 'p' };
}

void Payload::add(const char* key, int64_t value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), String(value), 'p' };
}

void Payload::add(const char* key, uint64_t value) {
  if (entry_count >= MAX_ENTRIES) return;
  entries[entry_count++] = { String(key), String(value), 'p' };
}

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

    i++;

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
// Primitive accessors
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

bool Payload::getBool(const char* key, bool default_value) const {
  bool v;
  return tryGetBool(key, v) ? v : default_value;
}

int32_t Payload::getInt(const char* key, int32_t default_value) const {
  int32_t v;
  return tryGetInt(key, v) ? v : default_value;
}

uint32_t Payload::getUInt(const char* key, uint32_t default_value) const {
  uint32_t v;
  return tryGetUInt(key, v) ? v : default_value;
}

float Payload::getFloat(const char* key, float default_value) const {
  float v;
  return tryGetFloat(key, v) ? v : default_value;
}

double Payload::getDouble(const char* key, double default_value) const {
  double v;
  return tryGetDouble(key, v) ? v : default_value;
}

// =============================================================
// Structured accessors (read-only views)
// =============================================================

bool Payload::hasArray(const char* key) const {
  const Entry* e = find(key);
  return e && e->kind == 'a';
}

PayloadArrayView Payload::getArrayView(const char* key) const {
  const Entry* e = find(key);
  if (!e || e->kind != 'a') {
    return PayloadArrayView();
  }
  return PayloadArrayView(e->value.c_str(), e->value.length());
}

// =============================================================
// PayloadArrayView — implementation
// =============================================================

PayloadArrayView::PayloadArrayView()
  : _json(nullptr), _len(0) {}

PayloadArrayView::PayloadArrayView(const char* json, size_t len)
  : _json(json), _len(len) {}

bool PayloadArrayView::valid() const {
  return _json && _len >= 2 && _json[0] == '[';
}

size_t PayloadArrayView::size() const {
  if (!valid()) return 0;

  size_t count = 0;
  int depth = 0;
  bool in_string = false;

  for (size_t i = 0; i < _len; i++) {
    char c = _json[i];

    if (c == '"' && (i == 0 || _json[i - 1] != '\\')) {
      in_string = !in_string;
    }

    if (in_string) continue;

    if (c == '{' || c == '[') depth++;
    if (c == '}' || c == ']') depth--;

    if (c == ',' && depth == 1) {
      count++;
    }
  }

  return count + 1;
}

Payload PayloadArrayView::get(size_t index) const {
  Payload out;
  if (!valid()) return out;

  size_t current = 0;
  int depth = 0;
  bool in_string = false;
  size_t start = 0;

  for (size_t i = 1; i < _len; i++) {

    char c = _json[i];

    if (c == '"' && _json[i - 1] != '\\') {
      in_string = !in_string;
    }

    if (in_string) continue;

    if (c == '{' || c == '[') {
      if (depth == 0 && current == index) {
        start = i;
      }
      depth++;
    }

    if (c == '}' || c == ']') {
      depth--;
      if (depth == 0 && current == index) {
        out.parseJSON(
          (const uint8_t*)(_json + start),
          i - start + 1
        );
        return out;
      }
    }

    if (c == ',' && depth == 0) {
      current++;
    }
  }

  return Payload();
}

// =============================================================
// PayloadArray — legacy implementation (unchanged)
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

  int i = 1;
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