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
    • JSON is a DERIVED REPRESENTATION, never state
    • Serialization does NOT imply ownership transfer
    • Accessors are referentially transparent
    • No accessor depends on call order
    • No shared or ephemeral buffers escape implicitly
    • Callers must not need to understand storage or lifetime rules

  DESIGN INTENT:

    Payload is a semantic object.
    JSON is a transient view over that object.

  ============================================================================
*/

class PayloadArray;
class PayloadArrayView;

// JsonView is a transient, non-owning view over a per-instance
// serialization buffer. Valid only until the next json_view() call
// on the same Payload instance.
struct JsonView {
  const char* data;
  size_t      len;
};

/*
  ============================================================================
  PayloadArrayView — Read-Only View Over JSON Array
  ----------------------------------------------------------------------------

  PayloadArrayView represents a non-owning, read-only view over a JSON array
  fragment already stored inside a Payload entry of kind 'a'.

  CONTRACT:
    • Does NOT own data
    • Does NOT allocate
    • Does NOT mutate state
    • Valid ONLY while the parent Payload exists
    • Produces value-semantic Payload objects on access
    • No caching or internal state mutation

  This type exists to enable safe, allocation-free traversal of arrays
  without changing Payload’s authoritative state model.

  ============================================================================
*/
class PayloadArrayView {
public:
  PayloadArrayView();  // invalid / empty view

  // --------------------------------------------------
  // Introspection
  // --------------------------------------------------

  bool   valid() const;
  size_t size() const;

  // --------------------------------------------------
  // Element access
  // --------------------------------------------------

  // Returns a value-semantic Payload parsed from the
  // indexed array element. If out of range or invalid,
  // returns an empty Payload.
  Payload get(size_t index) const;

private:
  friend class Payload;

  // Constructed only by Payload
  PayloadArrayView(const char* json, size_t len);

  const char* _json;   // points to '[' of array fragment
  size_t      _len;    // length of array fragment
};

class Payload {
public:
  Payload();

  // --------------------------------------------------
  // Lifecycle
  // --------------------------------------------------

  void clear();
  bool empty() const;

  // --------------------------------------------------
  // Cloning
  // --------------------------------------------------

  Payload clone() const;

  // --------------------------------------------------
  // Serialization
  // --------------------------------------------------

  /*
    Primary serialization API.

    Returns a transient, non-owning view into an internal static buffer.
    No heap allocation occurs.

    Callers MUST consume immediately.
  */
  JsonView json_view() const;

  /*
    Legacy convenience wrapper.

    Allocates exactly once and copies the JSON into a String.
    Intended for debugging, logging, and non–hot-path usage ONLY.
  */
  String to_json() const;

  // --------------------------------------------------
  // Semantic construction (authoritative)
  // --------------------------------------------------

  void add(const char* key, const char* value);
  void add(const char* key, const String& value);
  void add(const char* key, bool value);
  void add(const char* key, float value);
  void add(const char* key, double value);

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

  bool parseJSON(const uint8_t* data, size_t len);

  bool has(const char* key) const;

  // --------------------------------------------------
  // Primitive accessors
  // --------------------------------------------------

  const char* getString(const char* key) const;

  bool tryGetBool(const char* key, bool& out) const;
  bool tryGetInt(const char* key, int32_t& out) const;
  bool tryGetUInt(const char* key, uint32_t& out) const;
  bool tryGetFloat(const char* key, float& out) const;
  bool tryGetDouble(const char* key, double& out) const;

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
  // Structured accessors (read-only views)
  // --------------------------------------------------

  // Returns true only if the key exists and refers to an array.
  bool hasArray(const char* key) const;

  // Returns a read-only view over the array stored at key.
  // If absent or mismatched, returns an invalid/empty view.
  PayloadArrayView getArrayView(const char* key) const;

  // --------------------------------------------------
  // Diagnostics
  // --------------------------------------------------

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
  Entry  entries[MAX_ENTRIES];
  size_t entry_count;

  // -----------------------------------------------------------------
  // DERIVED / TRANSIENT REPRESENTATION (NON-AUTHORITATIVE)
  // -----------------------------------------------------------------
  //
  // Per-instance JSON serialization buffer.
  // This exists solely to eliminate reentrancy and shared-state hazards.
  //
  struct JsonBuf {
    static constexpr size_t JSON_MAX = 10 * 1024; // 10 KiB

    char   buf[JSON_MAX];
    size_t len;
    bool   overflow;

    JsonBuf() : len(0), overflow(false) {
      buf[0] = '\0';
    }
  };

  mutable JsonBuf _json_buf;


  // -----------------------------------------------------------------
  // Transitional / non-authoritative helpers
  // -----------------------------------------------------------------

  static String escape(const char* s);

  const Entry* find(const char* key) const;
};


// =============================================================
// PayloadArray — JSON array (object-only elements)
// =============================================================
//
// NOTE:
//   PayloadArray remains partially builder-oriented.
//   This is acceptable because it is not part of the semantic routing spine.
//   It may be refactored later to use JsonView-style semantics.
//

class PayloadArray {
public:
  PayloadArray();

  void clear();
  bool empty() const;

  String to_json() const;

  void add(const Payload& obj);

  bool parseJSON(const char* json);

  size_t  size() const;
  Payload get(size_t idx) const;

private:
  String buf;
  bool   first;

  static constexpr size_t MAX_ITEMS = 16;
  Payload items[MAX_ITEMS];
  size_t  item_count;
};