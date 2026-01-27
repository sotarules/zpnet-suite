#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <type_traits>

class PayloadArray;

class Payload {
public:
  Payload();

  void clear();
  bool empty() const;
  String to_json() const;

  // Primitive values
  void add(const char* key, const char* value);
  void add(const char* key, const String& value);
  void add(const char* key, bool value);
  void add(const char* key, float value);
  void add(const char* key, double value);

  template <typename T>
  typename std::enable_if<
      std::is_integral<T>::value && !std::is_same<T, bool>::value>::type
  add(const char* key, T value) {
    append_key(key);
    buf += String(value);
  }

  // Formatted string
  void add_fmt(const char* key, const char* fmt, ...);

  // NEW — structural composition
  void add_object(const char* key, const Payload& obj);
  void add_array(const char* key, const PayloadArray& arr);
  void add_raw_object(const char* key, const char* raw_json_object);

private:
  String buf;
  bool first;

  void append_key(const char* key);
  static String escape(const char* s);
};


// =============================================================
// PayloadArray — JSON array (object-only elements)
// =============================================================

class PayloadArray {
public:
  PayloadArray();

  void clear();
  bool empty() const;
  String to_json() const;

  // Add object elements
  void add(const Payload& obj);

private:
  String buf;
  bool first;
};
