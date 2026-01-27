#include "payload.h"

#include <stdarg.h>
#include <stdio.h>

// =============================================================
// Payload
// =============================================================

Payload::Payload() : buf(""), first(true) {}

void Payload::clear() {
  buf = "";
  first = true;
}

bool Payload::empty() const {
  return buf.length() == 0;
}

String Payload::to_json() const {
  String out = "{";
  out += buf;
  out += "}";
  return out;
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
    const char c = *s++;
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

// -------------------------------------------------------------
// Primitive fields
// -------------------------------------------------------------

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

// -------------------------------------------------------------
// NEW — structural composition
// -------------------------------------------------------------

void Payload::add_object(const char* key, const Payload& obj) {
  append_key(key);
  buf += obj.to_json();
}

void Payload::add_array(const char* key, const PayloadArray& arr) {
  append_key(key);
  buf += arr.to_json();
}

void Payload::add_raw_object(const char* key, const char* raw_json_object) {
  if (!raw_json_object || raw_json_object[0] != '{') {
    // Programmer error — do not recover
    return;
  }

  append_key(key);
  buf += raw_json_object;
}

// =============================================================
// PayloadArray
// =============================================================

PayloadArray::PayloadArray() : buf(""), first(true) {}

void PayloadArray::clear() {
  buf = "";
  first = true;
}

bool PayloadArray::empty() const {
  return buf.length() == 0;
}

String PayloadArray::to_json() const {
  String out = "[";
  out += buf;
  out += "]";
  return out;
}

void PayloadArray::add(const Payload& obj) {
  if (!first) buf += ",";
  first = false;

  buf += obj.to_json();
}
