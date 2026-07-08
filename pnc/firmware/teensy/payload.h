#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <type_traits>

/*
  ============================================================================
  Payload v3 — Stack-Safe JSON Carrier with Small-Entry Optimization
  ============================================================================

  HARD CONTRACT:

    • Payload has one authoritative semantic state: entry table + string arena.
    • JSON text is a derived representation, never authoritative state.
    • Keys and values live in one contiguous arena owned by the Payload.
    • Entries store offsets into that arena, not raw string pointers.
    • The object itself must remain cheap to place on the stack.
    • Large capacity is obtained through explicit, bounded heap growth.
    • All allocation failures are observable through PAYLOAD_INFO.

  v3 root fix:

    v2 accidentally made Payload a ~2 KiB stack object when MAX_ENTRIES grew to
    256 because the full entry table lived inline.  That created stack-boundary
    faults in ordinary command/report paths.  v3 keeps only a small inline entry
    cache in the object and spills to a bounded heap entry table only when a
    Payload grows beyond ordinary command size.

  Storage model:

    • Entry table:
        - first INLINE_ENTRIES entries live inside the object
        - grows by realloc up to MAX_ENTRIES
        - POD entries, safe to memcpy/realloc

    • Arena:
        - starts empty
        - grows by realloc from ARENA_INITIAL up to ARENA_MAX
        - clear() reuses capacity; destructor releases it

  Serialization contract:

    • write_json() returns bytes written, excluding NUL, on success.
    • write_json() returns 0 on overflow or invalid state.
    • to_json() is a legacy/debug convenience and may allocate.

  ============================================================================
*/

// ============================================================================
// Payload Instrumentation Snapshot (Read-Only, Monotonic)
// ============================================================================

typedef struct {

  // ABI / geometry
  uint32_t payload_object_size;
  uint32_t payload_entry_size;
  uint32_t payload_array_object_size;
  uint32_t payload_inline_entries;
  uint32_t payload_max_entries;
  uint32_t payload_arena_initial;
  uint32_t payload_arena_max;

  // Lifetime totals
  uint32_t instances_constructed;
  uint32_t instances_destroyed;

  // Live tracking
  uint32_t alive_now;
  uint32_t alive_high_water;

  // Entry table behavior
  uint32_t entry_alloc_fail;
  uint32_t entry_realloc_count;
  uint32_t entry_heap_bytes_alive;
  uint32_t entry_heap_bytes_high_water;
  uint32_t entry_overflow;
  uint32_t entry_high_water;
  uint32_t max_entry_capacity_seen;

  // Arena behavior
  uint32_t arena_alloc_fail;
  uint32_t arena_realloc_count;
  uint32_t arena_heap_bytes_alive;
  uint32_t arena_heap_bytes_high_water;
  uint32_t arena_high_water;
  uint32_t max_arena_capacity_seen;

  // Serialization / parsing / integrity
  uint32_t serialize_overflow;
  uint32_t to_json_fail;
  uint32_t string_truncation;
  uint32_t parse_error;
  uint32_t integrity_fail;
  uint32_t invalid_kind;

  // Defensive C-string pointer custody
  uint32_t string_pointer_fault;
  uint32_t string_pointer_null;
  uint32_t string_pointer_low_address;
  uint32_t string_pointer_magic_address;
  uint32_t string_pointer_out_of_range;
  uint32_t string_pointer_span_out_of_range;
  uint32_t string_pointer_unterminated;
  uint32_t string_pointer_too_long;
  uint32_t last_string_pointer_fault_reason;
  uint32_t last_string_pointer_fault_ptr;
  char     last_string_pointer_fault_context[32];
  uint32_t last_string_pointer_fault_key_ptr;
  char     last_string_pointer_fault_key[64];

  // Payload object integrity courtroom
  uint32_t self_ok_fail;
  uint32_t self_ok_magic_bad;
  uint32_t self_ok_entries_null;
  uint32_t self_ok_entry_cap_low;
  uint32_t self_ok_entry_cap_high;
  uint32_t self_ok_entries_magic_address;
  uint32_t self_ok_arena_magic_address;
  uint32_t self_ok_entries_span_unreadable;
  uint32_t self_ok_arena_span_unreadable;
  uint32_t self_ok_inline_cap_mismatch;
  uint32_t self_ok_heap_cap_mismatch;
  uint32_t self_ok_count_gt_entry_cap;
  uint32_t self_ok_count_gt_max;
  uint32_t self_ok_arena_used_gt_cap;
  uint32_t self_ok_arena_cap_gt_max;
  uint32_t self_ok_arena_cap_zero_with_ptr;
  uint32_t self_ok_arena_cap_nonzero_with_null;
  uint32_t self_ok_count_without_arena;
  uint32_t self_ok_entry_kind_bad;
  uint32_t self_ok_entry_key_off_oob;
  uint32_t self_ok_entry_val_off_oob;
  uint32_t self_ok_entry_val_end_oob;
  uint32_t self_ok_entry_val_unterminated;
  uint32_t self_ok_entry_key_unterminated;
  uint32_t last_self_ok_fail_reason;
  char     last_self_ok_fail_op[32];
  uint32_t last_self_ok_fail_this;
  uint32_t last_self_ok_fail_magic;
  uint32_t last_self_ok_fail_entries;
  uint32_t last_self_ok_fail_arena;
  uint32_t last_self_ok_fail_count;
  uint32_t last_self_ok_fail_entry_cap;
  uint32_t last_self_ok_fail_arena_used;
  uint32_t last_self_ok_fail_arena_cap;
  uint32_t last_self_ok_fail_entry_index;
  uint32_t last_self_ok_fail_entry_key_off;
  uint32_t last_self_ok_fail_entry_val_off;
  uint32_t last_self_ok_fail_entry_val_len;
  uint32_t last_self_ok_fail_entry_kind;

  // Last internal error breadcrumb
  uint32_t last_error_code;
  uint32_t last_error_count;
  uint32_t last_error_this;
  char     last_error_op[32];

} payload_info_t;

void payload_get_info(payload_info_t* out);
const char* payload_error_code_name(uint32_t code);
const char* payload_string_fault_reason_name(uint32_t reason);
const char* payload_self_ok_fail_reason_name(uint32_t reason);

class Payload;
class PayloadArray;
class PayloadArrayView;

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

    // Move semantics (arena/entry ownership transfer when heap-backed)
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

    size_t write_json(char* buf, size_t buf_size) const;
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
    void add(const char* key, double value, int precision);

    template <typename T>
    typename std::enable_if<
        std::is_integral<T>::value &&
        std::is_signed<T>::value &&
        !std::is_same<T, bool>::value>::type
    add(const char* key, T value) {
        add(key, (int64_t)value);
    }

    template <typename T>
    typename std::enable_if<
        std::is_integral<T>::value &&
        std::is_unsigned<T>::value &&
        !std::is_same<T, bool>::value>::type
    add(const char* key, T value) {
        add(key, (uint64_t)value);
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
    bool tryGetUInt64(const char* key, uint64_t& out) const;
    bool tryGetFloat(const char* key, float& out) const;
    bool tryGetDouble(const char* key, double& out) const;

    bool     getBool(const char* key, bool default_value = false) const;
    int32_t  getInt(const char* key, int32_t default_value = 0) const;
    uint32_t getUInt(const char* key, uint32_t default_value = 0) const;
    uint64_t getUInt64(const char* key, uint64_t default_value = 0) const;
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
    size_t entry_capacity() const { return _entry_cap; }
    bool heap_entries() const { return _entries != _inline_entries; }

    void debug_dump(const char* tag) const;

    // --------------------------------------------------
    // Limits
    // --------------------------------------------------

    // Operational ceilings.  Payload keeps only INLINE_ENTRIES in-object,
    // so these values bound heap-backed report growth without changing the
    // stack footprint.  Keep enough room for compact TIMEBASE rows while
    // preventing full diagnostic reports from becoming enormous heap/transport
    // transactions during active campaigns.
    static constexpr size_t INLINE_ENTRIES = 8;
    static constexpr size_t MAX_ENTRIES    = 256;
    static constexpr size_t ARENA_INITIAL  = 512;
    static constexpr size_t ARENA_MAX      = 16384;

    // The crash-hunt Payload courtroom was useful, but it made every add(),
    // find(), and write_json() repeatedly walk the entire entry table/arena.
    // In normal operation keep only cheap structural checks.  Flip true only
    // for a dedicated Payload autopsy build.
    static constexpr bool HEAVY_FORENSICS = true;

    static_assert(
        ARENA_MAX <= UINT16_MAX,
        "Payload ARENA_MAX exceeds uint16_t offset capacity"
    );

private:
    friend void payload_get_info(payload_info_t* out);

    struct Entry {
        uint16_t key_off;
        uint16_t val_off;
        uint16_t val_len;
        char     kind;       // 'p' = primitive, 'o' = object, 'a' = array
        uint8_t  _pad;
    };

    static constexpr uint32_t MAGIC_LIVE = 0x5041594Cu; // "PAYL"
    static constexpr uint32_t MAGIC_DEAD = 0x44454144u; // "DEAD"

    uint32_t _magic;
    Entry*   _entries;
    size_t   _count;
    size_t   _entry_cap;

    char*    _arena;
    size_t   _arena_used;
    size_t   _arena_cap;

    Entry    _inline_entries[INLINE_ENTRIES];

    bool _ensure_entries(size_t needed_count);
    bool _ensure_arena(size_t additional);
    bool _self_ok(const char* op) const;

    void _release_storage();
    void _move_from(Payload& other);
    bool _copy_from(const Payload& other);

    uint16_t _put(const char* str, size_t len);
    uint16_t _put(const char* str);

    bool _entry_key_ok(const Entry& e, const char* op, size_t* out_len = nullptr) const;
    bool _entry_value_ok(const Entry& e, const char* op, size_t* out_len = nullptr) const;
    const char* _primitive_value(const char* key, const char* op, size_t* out_len = nullptr) const;

    const char* _at(uint16_t offset) const;
    const Entry* _find(const char* key) const;

    void _add_entry(const char* key, const char* value, size_t value_len, char kind);
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
    String _buf;
    bool   _first;

    static constexpr size_t MAX_ITEMS = 16;
    Payload _items[MAX_ITEMS];
    size_t  _item_count;
};
