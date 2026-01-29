#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <type_traits>

class PayloadArray;

class Payload {
public:
  Payload();

  // --------------------------------------------------
  // Build / Emit
  // --------------------------------------------------

  void clear();
  bool empty() const;
  String to_json() const;

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

  void add_fmt(const char* key, const char* fmt, ...);

  void add_object(const char* key, const Payload& obj);
  void add_array(const char* key, const PayloadArray& arr);
  void add_raw_object(const char* key, const char* raw_json_object);

  // --------------------------------------------------
  // Parse / Access
  // --------------------------------------------------

  bool parseJSON(const uint8_t* data, size_t len);

  bool has(const char* key) const;

  // --------------------------------------------------
  // Primitive accessors
  // --------------------------------------------------

  // Raw primitive (string / number / bool as text)
  const char* getString(const char* key) const;

  // Strict (validation-oriented)
  bool tryGetBool(const char* key, bool& out) const;
  bool tryGetInt(const char* key, int32_t& out) const;
  bool tryGetUInt(const char* key, uint32_t& out) const;
  bool tryGetFloat(const char* key, float& out) const;
  bool tryGetDouble(const char* key, double& out) const;

  // Convenience (ergonomic)
  bool     getBool(const char* key, bool default_value) const;
  int32_t  getInt(const char* key, int32_t default_value = 0) const;
  uint32_t getUInt(const char* key, uint32_t default_value = 0) const;
  float    getFloat(const char* key, float default_value = 0.0f) const;
  double   getDouble(const char* key, double default_value = 0.0) const;

  // --------------------------------------------------
  // Structured accessors
  // --------------------------------------------------

  Payload      getPayload(const char* key) const;
  PayloadArray getArray(const char* key) const;

private:
  // builder
  String buf;
  bool first;

  // parsed representation
  String raw;

  struct Entry {
    String key;
    String value;
    char   kind;   // 'p' = primitive, 'o' = object, 'a' = array
  };

  static constexpr size_t MAX_ENTRIES = 16;
  Entry entries[MAX_ENTRIES];
  size_t entry_count;

  void append_key(const char* key);
  static String escape(const char* s);

  const Entry* find(const char* key) const;
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

  // emit
  void add(const Payload& obj);

  // parse / view
  bool parseJSON(const char* json);
  size_t size() const;
  Payload get(size_t idx) const;

private:
  String buf;
  bool first;

  static constexpr size_t MAX_ITEMS = 16;
  Payload items[MAX_ITEMS];
  size_t item_count;
};
