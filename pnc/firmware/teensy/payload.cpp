#include "payload.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// =============================================================
// Payload
// =============================================================

Payload::Payload()
  : buf(""),
    first(true),
    raw(""),
    entry_count(0) {}

void Payload::clear() {
  buf = "";
  raw = "";
  first = true;
  entry_count = 0;
}

bool Payload::empty() const {
  return buf.length() == 0 && raw.length() == 0;
}

String Payload::to_json() const {
  if (raw.length() > 0) return raw;
  return String("{") + buf + "}";
}

void Payload::append_key(const char* key) {
  if (!first) buf += ",";
  first = false;
  buf += "\"";
  buf += key;
  buf += "\":";
}

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

// --------------------------------------------------
// Emit primitives
// --------------------------------------------------

void Payload::add(const char* key, const char* value) {
  append_key(key);
  buf += "\"";
  buf += escape(value ? value : "");
  buf += "\"";
}

void Payload::add(const char* key, const String& value) {
  add(key, value.c_str());
}

void Payload::add(const char* key, bool value) {
  append_key(key);
  buf += (value ? "true" : "false");
}

void Payload::add(const char* key, float value) {
  append_key(key);
  buf += String(value, 6);
}

void Payload::add(const char* key, double value) {
  append_key(key);
  buf += String(value, 6);
}

void Payload::add_fmt(const char* key, const char* fmt, ...) {
  append_key(key);

  char tmp[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);

  buf += "\"";
  buf += escape(tmp);
  buf += "\"";
}

// --------------------------------------------------
// Emit structured
// --------------------------------------------------

void Payload::add_object(const char* key, const Payload& obj) {
  append_key(key);
  buf += obj.to_json();
}

void Payload::add_array(const char* key, const PayloadArray& arr) {
  append_key(key);
  buf += arr.to_json();
}

void Payload::add_raw_object(const char* key, const char* raw_json_object) {
  append_key(key);
  buf += raw_json_object;
}

// --------------------------------------------------
// Parse / access
// --------------------------------------------------

bool Payload::parseJSON(const uint8_t* data, size_t len) {
  raw = String((const char*)data).substring(0, len);
  entry_count = 0;

  int i = 1; // skip '{'
  while (i < (int)len && entry_count < MAX_ENTRIES) {
    while (i < (int)len && raw[i] != '"') i++;
    if (i >= (int)len) break;

    int k0 = ++i;
    while (raw[i] != '"') i++;
    String key = raw.substring(k0, i++);
    while (raw[i] != ':') i++;
    i++;

    char kind = raw[i] == '{' ? 'o' :
                raw[i] == '[' ? 'a' : 'p';

    int v0 = i;
    int depth = 0;
    do {
      if (raw[i] == '{' || raw[i] == '[') depth++;
      if (raw[i] == '}' || raw[i] == ']') depth--;
      i++;
    } while (depth > 0 && i < (int)len);

    entries[entry_count++] = { key, raw.substring(v0, i), kind };
  }

  return true;
}

const Payload::Entry* Payload::find(const char* key) const {
  for (size_t i = 0; i < entry_count; i++) {
    if (entries[i].key == key) return &entries[i];
  }
  return nullptr;
}

bool Payload::has(const char* key) const {
  return find(key) != nullptr;
}

// --------------------------------------------------
// Primitive accessors
// --------------------------------------------------

const char* Payload::getString(const char* key) const {
  const Entry* e = find(key);
  if (!e || e->kind != 'p') return nullptr;

  static String tmp;
  tmp = e->value;

  if (tmp.length() >= 2 &&
      tmp[0] == '"' &&
      tmp[tmp.length() - 1] == '"') {
    tmp = tmp.substring(1, tmp.length() - 1);
  }

  return tmp.c_str();
}

// --------------------------------------------------
// Primitive accessors (strict) — tryGetX
// --------------------------------------------------

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

// --------------------------------------------------
// Primitive accessors (convenience) — getX
// --------------------------------------------------

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

// --------------------------------------------------
// Structured accessors
// --------------------------------------------------

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
// PayloadArray
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
  return buf.length() == 0 && item_count == 0;
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
