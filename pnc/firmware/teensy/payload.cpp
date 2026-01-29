#include "payload.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
  ============================================================================
  Payload — Single-State, Semantic Implementation
  ----------------------------------------------------------------------------

  Authoritative state:
    • entries[]
    • entry_count

  All JSON text is a pure derivative.
  No builder buffers.
  No call-order dependence.

  PayloadArray is restored below for link completeness. It remains partially
  builder-oriented by design and is NOT part of the semantic routing spine.
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
  raw = "";  // staging only, not authoritative
}

bool Payload::empty() const {
  return entry_count == 0;
}

// -------------------------------------------------------------
// JSON serialization (pure derivative of entries[])
// -------------------------------------------------------------

String Payload::to_json() const {

  String out = "{";

  for (size_t i = 0; i < entry_count; i++) {

    if (i) out += ",";

    const Entry& e = entries[i];

    out += "\"";
    out += e.key;
    out += "\":";

    switch (e.kind) {
      case 'p':
        // Primitive values are stored canonically (no quotes)
        if (e.value == "true" || e.value == "false" ||
            (e.value.length() > 0 &&
             (isdigit(e.value[0]) || e.value[0] == '-' || e.value.indexOf('.') >= 0))) {
          out += e.value;
        } else {
          out += "\"";
          out += escape(e.value.c_str());
          out += "\"";
        }
        break;

      case 'o':
      case 'a':
        // Stored as raw JSON fragments
        out += e.value;
        break;

      default:
        out += "null";
        break;
    }
  }

  out += "}";
  return out;
}

// -------------------------------------------------------------
// Escaping helper (pure function)
// -------------------------------------------------------------

String Payload::escape(const char* s) {
  String out;
  while (*s) {
    char c = *s++;
    switch (c) {
      case '\"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:   out += c;      break;
    }
  }
  return out;
}

// =============================================================
// Semantic construction — populates entries[]
// =============================================================

void Payload::add(const char* key, const char* value) {

  if (entry_count >= MAX_ENTRIES) return;

  entries[entry_count++] = {
    String(key),
    String(value ? value : ""),
    'p'
  };
}

void Payload::add(const char* key, const String& value) {
  add(key, value.c_str());
}

void Payload::add(const char* key, bool value) {

  if (entry_count >= MAX_ENTRIES) return;

  entries[entry_count++] = {
    String(key),
    value ? "true" : "false",
    'p'
  };
}

void Payload::add(const char* key, float value) {

  if (entry_count >= MAX_ENTRIES) return;

  entries[entry_count++] = {
    String(key),
    String(value, 6),
    'p'
  };
}

void Payload::add(const char* key, double value) {

  if (entry_count >= MAX_ENTRIES) return;

  entries[entry_count++] = {
    String(key),
    String(value, 6),
    'p'
  };
}

void Payload::add_fmt(const char* key, const char* fmt, ...) {

  if (entry_count >= MAX_ENTRIES) return;

  char tmp[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);

  entries[entry_count++] = {
    String(key),
    String(tmp),
    'p'
  };
}

void Payload::add_object(const char* key, const Payload& obj) {

  if (entry_count >= MAX_ENTRIES) return;

  entries[entry_count++] = {
    String(key),
    obj.to_json(),
    'o'
  };
}

void Payload::add_array(const char* key, const PayloadArray& arr) {

  if (entry_count >= MAX_ENTRIES) return;

  entries[entry_count++] = {
    String(key),
    arr.to_json(),
    'a'
  };
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
