#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <type_traits>

/*
  ============================================================================
  Payload v4 — Single-Store Typed JSON Carrier
  ============================================================================

  HARD CONTRACT:

    • Payload owns one contiguous storage region.
    • The entry directory and all owned bytes live in that same region.
    • Values retain their JSON type; strings are never reclassified by text.
    • Add/attach operations are preflighted and committed transactionally.
    • Public mutators return bool; existing callers may ignore the result.
    • Invalid floating-point values are represented as JSON null, never omitted
      and never serialized as non-standard NaN/Infinity tokens.
    • Failed add/copy operations leave the prior semantic document unchanged,
      except for the documented invalid-number-to-null compatibility fallback.
    • parseJSON() is a replacement operation and clears on invalid input, matching
      the established command-ingress contract.
    • Nested objects and arrays are written directly; no temporary JSON heap
      buffer is needed to attach a child.
    • Empty and small Payloads use bounded inline storage. Larger Payloads use
      one explicitly owned heap block, never independent entry/arena blocks.
    • Object metadata is held with cheap redundant complements; heap blocks are
      bound to their owning object by a pointer-derived cookie and complement.
    • Corruption handling is fail-closed and quiet: no magic-number trap, stack
      walk, logging recursion, or diagnostic Payload construction.

  Compatibility:

    The public API and payload_info_t telemetry shape remain source-compatible
    with Payload v3. Legacy entry/arena telemetry fields are retained; in v4
    they describe the single backing store where practical and remain zero when
    a v3-only concept no longer exists.

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
  uint32_t json_invalid_syntax;
  uint32_t json_invalid_depth;
  uint32_t json_invalid_utf8_key;
  uint32_t json_invalid_utf8_value;
  uint32_t json_invalid_raw_object;
  uint32_t json_invalid_raw_array;
  uint32_t json_decode_fail;
  uint32_t integrity_fail;
  uint32_t invalid_kind;

  // Numeric admission / null-substitution compatibility
  uint32_t numeric_null_substitution;
  uint32_t numeric_nonfinite;
  uint32_t numeric_nan;
  uint32_t numeric_positive_infinity;
  uint32_t numeric_negative_infinity;
  uint32_t numeric_invalid_token;
  uint32_t numeric_format_failure;
  uint32_t numeric_null_insert_fail;
  uint32_t last_numeric_reject_reason;
  uint32_t last_numeric_reject_op_id;
  uint32_t last_numeric_reject_this;
  char     last_numeric_reject_key[64];
  uint64_t last_numeric_reject_value_bits;
  int32_t  last_numeric_reject_precision;
  int32_t  last_numeric_reject_format_return;
  int32_t  last_numeric_reject_snprintf_return;
  uint32_t last_numeric_reject_text_len;
  uint32_t last_numeric_reject_text_terminated;
  uint32_t last_numeric_reject_text_truncated;
  char     last_numeric_reject_format[16];
  char     last_numeric_reject_text_printable[64];
  char     last_numeric_reject_text_hex[129];

  // First semantic serialization failure evidence
  uint32_t semantic_validation_fail;
  uint32_t semantic_invalid_kind;
  uint32_t semantic_invalid_key_utf8;
  uint32_t semantic_invalid_string_utf8;
  uint32_t semantic_invalid_number_token;
  uint32_t semantic_invalid_boolean_token;
  uint32_t semantic_invalid_null_token;
  uint32_t semantic_invalid_object_json;
  uint32_t semantic_invalid_array_json;
  uint32_t first_semantic_fail_captured;
  uint32_t first_semantic_fail_reason;
  uint32_t first_semantic_fail_op_id;
  uint32_t first_semantic_fail_this;
  uint32_t first_semantic_fail_entry_index;
  uint32_t first_semantic_fail_entry_kind;
  uint32_t first_semantic_fail_key_off;
  uint32_t first_semantic_fail_key_len;
  uint32_t first_semantic_fail_val_off;
  uint32_t first_semantic_fail_val_len;
  char     first_semantic_fail_key[64];

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
  uint32_t last_string_pointer_fault_op_id;
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
  uint32_t last_self_ok_fail_op_id;
  uint32_t last_self_ok_fail_capacity;
  uint32_t last_self_ok_fail_data_begin;
  uint32_t last_self_ok_fail_expected_upper;
  uint32_t last_self_ok_fail_key_end;
  uint32_t last_self_ok_fail_val_end;
  uint32_t first_self_ok_fail_captured;

  // Last internal error breadcrumb
  uint32_t last_error_code;
  uint32_t last_error_count;
  uint32_t last_error_this;
  uint32_t last_error_op_id;
  char     last_error_op[32];

} payload_info_t;

void payload_get_info(payload_info_t* out);
const char* payload_error_code_name(uint32_t code);
const char* payload_string_fault_reason_name(uint32_t reason);
const char* payload_self_ok_fail_reason_name(uint32_t reason);
const char* payload_numeric_reject_reason_name(uint32_t reason);
const char* payload_semantic_fail_reason_name(uint32_t reason);
const char* payload_operation_id_name(uint32_t operation_id);

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

    Payload(Payload&& other) noexcept;
    Payload& operator=(Payload&& other) noexcept;

    Payload(const Payload& other);
    Payload& operator=(const Payload& other);

    // Lifecycle
    bool clear();
    bool empty() const;
    Payload clone() const;

    // Serialization
    size_t write_json(char* buf, size_t buf_size) const;
    String to_json() const;

    // Semantic construction
    bool add(const char* key, int32_t value);
    bool add(const char* key, uint32_t value);
    bool add(const char* key, int64_t value);
    bool add(const char* key, uint64_t value);

    bool add(const char* key, const char* value);
    bool add(const char* key, const String& value);
    bool add(const char* key, bool value);
    bool add(const char* key, float value);
    bool add(const char* key, double value);
    bool add(const char* key, double value, int precision);

    template <typename T>
    typename std::enable_if<
        std::is_integral<T>::value &&
        std::is_signed<T>::value &&
        !std::is_same<T, bool>::value, bool>::type
    add(const char* key, T value) {
        return add(key, (int64_t)value);
    }

    template <typename T>
    typename std::enable_if<
        std::is_integral<T>::value &&
        std::is_unsigned<T>::value &&
        !std::is_same<T, bool>::value, bool>::type
    add(const char* key, T value) {
        return add(key, (uint64_t)value);
    }

    bool add_fmt(const char* key, const char* fmt, ...);

    bool add_object(const char* key, const Payload& obj);
    bool add_array(const char* key, const PayloadArray& arr);
    bool add_raw_object(const char* key, const char* raw_json_object);

    // Parsing
    bool parseJSON(const uint8_t* data, size_t len);

    // Access
    bool has(const char* key) const;
    // Returned storage remains owned by this Payload and is stable until the
    // next mutation, assignment, clear(), or destruction.
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

    // Structured accessors
    Payload      getPayload(const char* key) const;
    PayloadArray getArray(const char* key) const;

    bool             hasArray(const char* key) const;
    PayloadArrayView getArrayView(const char* key) const;

    // Diagnostics
    size_t count() const;
    size_t arena_used() const;
    size_t arena_capacity() const;
    size_t entry_capacity() const;
    bool heap_entries() const;

    void debug_dump(const char* tag) const;

    // Bounded operational ceilings
    static constexpr size_t INLINE_ENTRIES = 8;
    static constexpr size_t MAX_ENTRIES    = 512;
    static constexpr size_t ARENA_INITIAL  = 512;
    static constexpr size_t ARENA_MAX      = 49152;

    // Retained only so existing build/report code continues to compile.
    // Payload v4 has no heavy operational courtroom.
    static constexpr bool HEAVY_FORENSICS = false;

private:
    friend void payload_get_info(payload_info_t* out);
    friend class PayloadArray;

    enum class ValueKind : uint8_t {
        STRING = 1,
        NUMBER = 2,
        BOOLEAN = 3,
        NIL = 4,
        OBJECT = 5,
        ARRAY = 6,
    };

    struct Entry {
        uint16_t key_off;
        uint16_t key_len;
        uint16_t val_off;
        uint16_t val_len;
        uint8_t  kind;
        uint8_t  reserved;
    };

    static constexpr size_t INLINE_STORAGE = 256;
    static constexpr size_t STORAGE_MAX =
        ARENA_MAX + MAX_ENTRIES * sizeof(Entry);

    static_assert(INLINE_STORAGE >= sizeof(Entry),
                  "Payload inline store cannot hold one entry");
    static_assert(ARENA_MAX <= UINT16_MAX,
                  "Payload ARENA_MAX exceeds uint16_t storage offsets");
    static_assert(STORAGE_MAX <= UINT16_MAX,
                  "Payload STORAGE_MAX exceeds uint16_t storage offsets");

    void*     _heap_block;
    uintptr_t _heap_block_guard;
    uint16_t  _count;
    uint16_t  _count_guard;
    uint16_t  _data_begin;
    uint16_t  _data_begin_guard;
    alignas(uint32_t) uint8_t _inline_storage[INLINE_STORAGE];

    bool _heap_guard_ok() const;
    bool _count_guard_ok() const;
    bool _data_begin_guard_ok() const;
    void _set_heap_block(void* block);
    void _set_count(uint16_t count);
    void _set_data_begin(uint16_t data_begin);

    uint8_t* _storage();
    const uint8_t* _storage() const;
    size_t _capacity() const;
    size_t _data_used() const;

    Entry* _entries();
    const Entry* _entries() const;

    void _reset_empty();
    void _release_storage();
    void _move_from(Payload& other);
    bool _copy_from(const Payload& other);

    bool _self_ok(uint32_t operation_id) const;
    bool _entry_ok(const Entry& e, size_t index, uint32_t operation_id) const;
    bool _layout_ok(uint32_t operation_id) const;
    const Entry* _find(const char* key, size_t key_len) const;
    const Entry* _find(const char* key) const;

    bool _ensure_room(size_t additional_entries,
                      size_t additional_data,
                      int32_t* data_shift = nullptr);

    bool _append_value(const char* key,
                       size_t key_len,
                       const char* value,
                       size_t value_len,
                       ValueKind kind);

    bool _add_floating(const char* key,
                       double value,
                       int precision,
                       uint32_t operation_id);

    bool _append_value_writer(const char* key,
                              size_t key_len,
                              size_t value_len,
                              ValueKind kind,
                              const Payload* object_value,
                              const PayloadArray* array_value);

    size_t _json_size() const;
    size_t _write_json_unchecked(char* buf) const;

    const char* _value_ptr(const Entry& e) const;
};

// ============================================================================
// PayloadArray — JSON array of Payload objects
// ============================================================================

class PayloadArray {
public:
    PayloadArray();
    ~PayloadArray();

    PayloadArray(const PayloadArray& other);
    PayloadArray& operator=(const PayloadArray& other);
    PayloadArray(PayloadArray&& other) noexcept;
    PayloadArray& operator=(PayloadArray&& other) noexcept;

    bool clear();
    bool empty() const;

    String to_json() const;
    bool add(const Payload& obj);
    bool parseJSON(const char* json);

    size_t  size() const;
    Payload get(size_t idx) const;

private:
    friend class Payload;

    static constexpr size_t INLINE_STORAGE = 256;
    static constexpr size_t STORAGE_MAX = Payload::ARENA_MAX;

    void*     _heap_block;
    uintptr_t _heap_block_guard;
    uint16_t  _length;
    uint16_t  _length_guard;
    alignas(uint32_t) char _inline_storage[INLINE_STORAGE];

    bool _heap_guard_ok() const;
    bool _length_guard_ok() const;
    void _set_heap_block(void* block);
    void _set_length(uint16_t length);

    char* _data();
    const char* _data() const;
    size_t _capacity() const;
    bool _self_ok() const;
    bool _ensure_capacity(size_t needed);
    void _release_storage();
    void _move_from(PayloadArray& other);
    bool _copy_from(const PayloadArray& other);

    size_t _json_size() const;
    size_t _write_json_unchecked(char* out) const;
};
