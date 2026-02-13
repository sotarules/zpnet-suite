#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <type_traits>

/*
  ============================================================================
  Payload v2 — Arena-Backed, Stack-Safe JSON Carrier
  ============================================================================

  HARD CONTRACT (DO NOT VIOLATE):

    • Payload has exactly ONE authoritative internal state: entries[] + arena
    • JSON is a DERIVED REPRESENTATION, never state
    • All strings (keys and values) live in a single contiguous arena
    • Entries store offsets into the arena, not pointers
    • Payload object itself is cheap to put on the stack (~300 bytes)
    • Serialization writes to caller-provided or shared scratch buffer
    • No Arduino String used for storage (only as convenience in add() sigs)

  STACK COST:
    Entry:  8 bytes × 32  = 256 bytes
    Fields:                ≈  24 bytes
    Total:                 ≈ 280 bytes per Payload on stack
    (vs ~10,400+ bytes in v1 due to inline 10KB JsonBuf)

  HEAP COST:
    One allocation per Payload (arena), starting at 512 bytes.
    Grows by realloc up to ARENA_MAX (8 KiB).
    Freed in destructor.

  MOVE SEMANTICS:
    Payload supports move construction and move assignment.
    Returning a Payload from a function transfers arena ownership
    with zero copying — just pointer swap.

  ============================================================================
*/

// ============================================================================
// Payload Instrumentation Snapshot (Read-Only, Monotonic)
// ============================================================================

typedef struct {

  // Lifetime totals
  uint32_t instances_constructed;
  uint32_t instances_destroyed;

  // Live tracking
  uint32_t alive_now;
  uint32_t alive_high_water;

  // Arena behavior
  uint32_t arena_alloc_fail;
  uint32_t arena_high_water;

  // Entry behavior
  uint32_t entry_overflow;
  uint32_t entry_high_water;

} payload_info_t;

void payload_get_info(payload_info_t* out);

class Payload;
class PayloadArray;
class PayloadArrayView;

// ----------------------------------------------------------------------------
// JsonView — transient, non-owning view over serialized JSON
// ----------------------------------------------------------------------------

struct JsonView {
    const char* data;
    size_t      len;
};

// ----------------------------------------------------------------------------
// PayloadArrayView — read-only view over a JSON array fragment
// ----------------------------------------------------------------------------

class PayloadArrayView {
public:
    PayloadArrayView();

    bool    valid() const;
    size_t  size() const;
    Payload get(size_t index) const;

private:
    friend class Payload;
    PayloadArrayView(const char* json, size_t len);

    const char* _json;
    size_t      _len;
};

// ============================================================================
// Payload
// ============================================================================

class Payload {
public:
    Payload();
    ~Payload();

    // Move semantics (arena ownership transfer, zero-copy)
    Payload(Payload&& other) noexcept;
    Payload& operator=(Payload&& other) noexcept;

    // Copy (deep copy of arena + entries)
    Payload(const Payload& other);
    Payload& operator=(const Payload& other);

    // --------------------------------------------------
    // Lifecycle
    // --------------------------------------------------

    void clear();
    bool empty() const;

    // --------------------------------------------------
    // Cloning (explicit deep copy)
    // --------------------------------------------------

    Payload clone() const;

    // --------------------------------------------------
    // Serialization
    // --------------------------------------------------

    /*
      Primary serialization API.
      Write JSON into caller-provided buffer.
      Returns bytes written (excluding NUL), or 0 on overflow.
      Buffer is always NUL-terminated on success.
    */
    size_t write_json(char* buf, size_t buf_size) const;

    // DEPRECATED — do not use from any path reachable by timepop callbacks.
    // Retained only for ad-hoc serial console debugging if ever needed.
    JsonView json_view() const;

    /*
      Legacy convenience wrapper.
      Heap-allocates a String. Use for debug/logging only.
    */
    String to_json() const;

    // --------------------------------------------------
    // Semantic construction
    // --------------------------------------------------

    void add(const char* key, int32_t value);
    void add(const char* key, uint32_t value);
    void add(const char* key, int64_t value);
    void add(const char* key, uint64_t value);

    void add(const char* key, const char* value);
    void add(const char* key, const String& value);
    void add(const char* key, bool value);
    void add(const char* key, float value);
    void add(const char* key, double value);

    // Catch-all for other integral types (size_t, int, unsigned, etc.)
    template <typename T>
    typename std::enable_if<
        std::is_integral<T>::value && !std::is_same<T, bool>::value>::type
    add(const char* key, T value) {
        add(key, (int64_t)value);
    }

    void add_fmt(const char* key, const char* fmt, ...);

    void add_object(const char* key, const Payload& obj);
    void add_array(const char* key, const PayloadArray& arr);
    void add_raw_object(const char* key, const char* raw_json_object);

    // --------------------------------------------------
    // Parsing
    // --------------------------------------------------

    bool parseJSON(const uint8_t* data, size_t len);

    // --------------------------------------------------
    // Access
    // --------------------------------------------------

    bool has(const char* key) const;

    const char* getString(const char* key) const;

    bool tryGetBool(const char* key, bool& out) const;
    bool tryGetInt(const char* key, int32_t& out) const;
    bool tryGetUInt(const char* key, uint32_t& out) const;
    bool tryGetFloat(const char* key, float& out) const;
    bool tryGetDouble(const char* key, double& out) const;

    bool     getBool(const char* key, bool default_value = false) const;
    int32_t  getInt(const char* key, int32_t default_value = 0) const;
    uint32_t getUInt(const char* key, uint32_t default_value = 0) const;
    float    getFloat(const char* key, float default_value = 0.0f) const;
    double   getDouble(const char* key, double default_value = 0.0) const;

    // --------------------------------------------------
    // Structured accessors
    // --------------------------------------------------

    Payload      getPayload(const char* key) const;
    PayloadArray getArray(const char* key) const;

    bool             hasArray(const char* key) const;
    PayloadArrayView getArrayView(const char* key) const;

    // --------------------------------------------------
    // Diagnostics
    // --------------------------------------------------

    size_t count() const { return _count; }
    size_t arena_used() const { return _arena_used; }
    size_t arena_capacity() const { return _arena_cap; }

    void debug_dump(const char* tag) const;

    // --------------------------------------------------
    // Limits
    // --------------------------------------------------

    static constexpr size_t MAX_ENTRIES   = 32;
    static constexpr size_t ARENA_INITIAL = 512;
    static constexpr size_t ARENA_MAX     = 8192;

static_assert(
    ARENA_MAX <= UINT16_MAX,
    "Payload ARENA_MAX exceeds uint16_t offset capacity"
);

private:
    // -----------------------------------------------------------------
    // Entry: 8 bytes each (offsets into arena)
    // -----------------------------------------------------------------

    struct Entry {
        uint16_t key_off;
        uint16_t val_off;
        uint16_t val_len;
        char     kind;       // 'p' = primitive, 'o' = object, 'a' = array
        uint8_t  _pad;
    };

    Entry  _entries[MAX_ENTRIES];
    size_t _count;

    // -----------------------------------------------------------------
    // Arena: single heap-allocated buffer for all string data
    // -----------------------------------------------------------------

    char*  _arena;
    size_t _arena_used;
    size_t _arena_cap;

    // -----------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------

    // Append a NUL-terminated string to the arena.
    // Returns offset of the stored string, or UINT16_MAX on failure.
    uint16_t _put(const char* str, size_t len);
    uint16_t _put(const char* str);

    // Ensure arena has room for `additional` more bytes.
    bool _ensure(size_t additional);

    // Get pointer to string at arena offset.
    const char* _at(uint16_t offset) const;

    // Find entry by key.
    const Entry* _find(const char* key) const;

    // Add a pre-formatted string value with given kind.
    void _add_entry(const char* key, const char* value, size_t value_len, char kind);

    // -----------------------------------------------------------------
    // Shared scratch buffer for json_view()
    // -----------------------------------------------------------------

    static constexpr size_t SCRATCH_SIZE = 10 * 1024;
    static char _scratch[SCRATCH_SIZE];
};


// ============================================================================
// PayloadArray — JSON array of Payload objects
// ============================================================================

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
    // Serialized form (builder path)
    String _buf;
    bool   _first;

    // Parsed form
    static constexpr size_t MAX_ITEMS = 16;
    Payload _items[MAX_ITEMS];
    size_t  _item_count;
};
