#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <type_traits>
#include "debug.h"

/*
  ============================================================================
  Payload — Semantic, Single-State JSON Carrier
  ----------------------------------------------------------------------------

  HARD CONTRACT (DO NOT VIOLATE):

    • Payload has exactly ONE authoritative internal state: `entries[]`
    • JSON text is a derived representation, never state
    • Accessors must be referentially transparent
    • No accessor may depend on call order
    • No shared or ephemeral buffers may escape
    • Callers must not need to understand storage or lifetime rules

  This header reflects those constraints explicitly.
  ============================================================================
*/

class PayloadArray;

class Payload {
public:
  Payload();

  // --------------------------------------------------
  // Lifecycle
  // --------------------------------------------------

  void clear();
  bool empty() const;

  // Serialize the current semantic state to JSON.
  // This is a pure derivative of entries[].
  String to_json() const;

  // --------------------------------------------------
  // Semantic construction (authoritative)
  // --------------------------------------------------

  // All add() methods populate entries[] directly.
  // No builder buffers, no deferred state.

  void add(const char* key, const char* value);
  void add(const char* key, const String& value);
  void add(const char* key, bool value);
  void add(const char* key, float value);
  void add(const char* key, double value);

  // Integral template (excluding bool)
  // Delegates to canonical semantic path.
  template <typename T>
  typename std::enable_if<
      std::is_integral<T>::value && !std::is_same<T, bool>::value>::type
  add(const char* key, T value) {
    add(key, String(value));
  }

  void add_fmt(const char* key, const char* fmt, ...);

  void add_object(const char* key, const Payload& obj);
  void add_array(const char* key, const PayloadArray& arr);

  // Adds a raw JSON object fragment.
  // Caller is responsible for structural correctness.
  void add_raw_object(const char* key, const char* raw_json_object);

  // --------------------------------------------------
  // Parse / Access
  // --------------------------------------------------

  // Parse a bounded JSON object into semantic entries.
  // This function canonicalizes values into entries[].
  bool parseJSON(const uint8_t* data, size_t len);

  bool has(const char* key) const;

  // --------------------------------------------------
  // Primitive accessors
  // --------------------------------------------------

  // Returns a pointer to owned, stable storage.
  // Lifetime is tied to the Payload object.
  // Never call-order dependent.
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
  // Structured accessors (value semantics)
  // --------------------------------------------------

  Payload      getPayload(const char* key) const;
  PayloadArray getArray(const char* key) const;

  // --------------------------------------------------
  // Diagnostics
  // --------------------------------------------------

  // Dump parsed key/value entries for debugging.
  // Reflects semantic state, not serialization artifacts.
  void debug_dump(const char* tag) const;

private:
  // -----------------------------------------------------------------
  // INTERNAL REPRESENTATION (AUTHORITATIVE)
  // -----------------------------------------------------------------

  struct Entry {
    String key;
    String value;
    char   kind;   // 'p' = primitive, 'o' = object, 'a' = array
  };

  static constexpr size_t MAX_ENTRIES = 16;
  Entry entries[MAX_ENTRIES];
  size_t entry_count;

  // -----------------------------------------------------------------
  // Transitional / non-authoritative helpers
  // -----------------------------------------------------------------

  // Used only as a staging buffer during parseJSON().
  // Not authoritative state.
  String raw;

  // Pure helper
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

  // Serialize array contents (pure derivative)
  String to_json() const;

  // Append object (semantic)
  void add(const Payload& obj);

  // Parse array of objects (best-effort structural extraction)
  bool parseJSON(const char* json);

  size_t size() const;
  Payload get(size_t idx) const;

private:
  // NOTE:
  // PayloadArray remains partially builder-oriented for now.
  // This is acceptable because it is not a routing or semantic spine.
  // It may be refactored later using the same single-state rules.

  String buf;
  bool first;

  static constexpr size_t MAX_ITEMS = 16;
  Payload items[MAX_ITEMS];
  size_t item_count;
};
