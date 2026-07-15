#include "payload.h"
#include "util.h"
#include "debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stddef.h>

// ============================================================================
// Payload v4.1 instrumentation
// ============================================================================

static volatile uint32_t g_payload_instances_constructed = 0;
static volatile uint32_t g_payload_instances_destroyed = 0;
static volatile uint32_t g_payload_alive_now = 0;
static volatile uint32_t g_payload_alive_high_water = 0;

// V4 has no independently allocated entry table. These compatibility counters
// remain zero except for overflow/high-water observations.
static volatile uint32_t g_payload_entry_alloc_fail = 0;
static volatile uint32_t g_payload_entry_realloc_count = 0;
static volatile uint32_t g_payload_entry_heap_bytes_alive = 0;
static volatile uint32_t g_payload_entry_heap_bytes_high_water = 0;
static volatile uint32_t g_payload_entry_overflow = 0;
static volatile uint32_t g_payload_entry_high_water_global = 0;
static volatile uint32_t g_payload_max_entry_capacity_seen = 0;

// The v3 "arena" telemetry now describes the one heap-backed storage block.
static volatile uint32_t g_payload_arena_alloc_fail = 0;
static volatile uint32_t g_payload_arena_realloc_count = 0;
static volatile uint32_t g_payload_arena_heap_bytes_alive = 0;
static volatile uint32_t g_payload_arena_heap_bytes_high_water = 0;
static volatile uint32_t g_payload_arena_high_water_global = 0;
static volatile uint32_t g_payload_max_arena_capacity_seen = 0;

static volatile uint32_t g_payload_serialize_overflow = 0;
static volatile uint32_t g_payload_to_json_fail = 0;
static volatile uint32_t g_payload_string_truncation = 0;
static volatile uint32_t g_payload_parse_error = 0;
static volatile uint32_t g_payload_json_invalid_syntax = 0;
static volatile uint32_t g_payload_json_invalid_depth = 0;
static volatile uint32_t g_payload_json_invalid_utf8_key = 0;
static volatile uint32_t g_payload_json_invalid_utf8_value = 0;
static volatile uint32_t g_payload_json_invalid_raw_object = 0;
static volatile uint32_t g_payload_json_invalid_raw_array = 0;
static volatile uint32_t g_payload_json_decode_fail = 0;
static volatile uint32_t g_payload_integrity_fail = 0;
static volatile uint32_t g_payload_invalid_kind = 0;

// Invalid programmatic numeric inputs are represented as JSON null.  These are
// admission facts, not Payload corruption, and therefore do not increment the
// integrity/self-ok courts.
static volatile uint32_t g_payload_numeric_null_substitution = 0;
static volatile uint32_t g_payload_numeric_nonfinite = 0;
static volatile uint32_t g_payload_numeric_nan = 0;
static volatile uint32_t g_payload_numeric_positive_infinity = 0;
static volatile uint32_t g_payload_numeric_negative_infinity = 0;
static volatile uint32_t g_payload_numeric_invalid_token = 0;
static volatile uint32_t g_payload_numeric_format_failure = 0;
static volatile uint32_t g_payload_numeric_null_insert_fail = 0;
static volatile uint32_t g_payload_last_numeric_reject_reason = 0;
static volatile uint32_t g_payload_last_numeric_reject_op_id = 0;
static volatile uint32_t g_payload_last_numeric_reject_this = 0;
static char g_payload_last_numeric_reject_key[64] = {0};
static volatile uint64_t g_payload_last_numeric_reject_value_bits = 0;
static volatile int32_t g_payload_last_numeric_reject_precision = 0;
static volatile int32_t g_payload_last_numeric_reject_format_return = 0;
static volatile int32_t g_payload_last_numeric_reject_snprintf_return = 0;
static volatile uint32_t g_payload_last_numeric_reject_text_len = 0;
static volatile uint32_t g_payload_last_numeric_reject_text_terminated = 0;
static volatile uint32_t g_payload_last_numeric_reject_text_truncated = 0;
static char g_payload_last_numeric_reject_format[16] = {0};
static char g_payload_last_numeric_reject_text_printable[64] = {0};
static char g_payload_last_numeric_reject_text_hex[129] = {0};

// Serialization-time semantic evidence.  Admission should prevent these in
// normal operation; a surviving failure is retained as first-failure evidence.
static volatile uint32_t g_payload_semantic_validation_fail = 0;
static volatile uint32_t g_payload_semantic_invalid_kind = 0;
static volatile uint32_t g_payload_semantic_invalid_key_utf8 = 0;
static volatile uint32_t g_payload_semantic_invalid_string_utf8 = 0;
static volatile uint32_t g_payload_semantic_invalid_number_token = 0;
static volatile uint32_t g_payload_semantic_invalid_boolean_token = 0;
static volatile uint32_t g_payload_semantic_invalid_null_token = 0;
static volatile uint32_t g_payload_semantic_invalid_object_json = 0;
static volatile uint32_t g_payload_semantic_invalid_array_json = 0;
static volatile uint32_t g_payload_first_semantic_fail_captured = 0;
static volatile uint32_t g_payload_first_semantic_fail_reason = 0;
static volatile uint32_t g_payload_first_semantic_fail_op_id = 0;
static volatile uint32_t g_payload_first_semantic_fail_this = 0;
static volatile uint32_t g_payload_first_semantic_fail_entry_index = 0xFFFFFFFFUL;
static volatile uint32_t g_payload_first_semantic_fail_entry_kind = 0;
static volatile uint32_t g_payload_first_semantic_fail_key_off = 0;
static volatile uint32_t g_payload_first_semantic_fail_key_len = 0;
static volatile uint32_t g_payload_first_semantic_fail_val_off = 0;
static volatile uint32_t g_payload_first_semantic_fail_val_len = 0;
static char g_payload_first_semantic_fail_key[64] = {0};

static volatile uint32_t g_payload_string_pointer_fault = 0;
static volatile uint32_t g_payload_string_pointer_null = 0;
static volatile uint32_t g_payload_string_pointer_low_address = 0;
static volatile uint32_t g_payload_string_pointer_magic_address = 0;
static volatile uint32_t g_payload_string_pointer_out_of_range = 0;
static volatile uint32_t g_payload_string_pointer_span_out_of_range = 0;
static volatile uint32_t g_payload_string_pointer_unterminated = 0;
static volatile uint32_t g_payload_string_pointer_too_long = 0;
static volatile uint32_t g_payload_last_string_pointer_fault_reason = 0;
static volatile uint32_t g_payload_last_string_pointer_fault_ptr = 0;
static volatile uint32_t g_payload_last_string_pointer_fault_op_id = 0;
static char g_payload_last_string_pointer_fault_context[32] = {0};
static volatile uint32_t g_payload_last_string_pointer_fault_key_ptr = 0;
static char g_payload_last_string_pointer_fault_key[64] = {0};

static volatile uint32_t g_payload_self_ok_fail = 0;
static volatile uint32_t g_payload_self_ok_magic_bad = 0;
static volatile uint32_t g_payload_self_ok_entries_null = 0;
static volatile uint32_t g_payload_self_ok_entry_cap_low = 0;
static volatile uint32_t g_payload_self_ok_entry_cap_high = 0;
static volatile uint32_t g_payload_self_ok_entries_magic_address = 0;
static volatile uint32_t g_payload_self_ok_arena_magic_address = 0;
static volatile uint32_t g_payload_self_ok_entries_span_unreadable = 0;
static volatile uint32_t g_payload_self_ok_arena_span_unreadable = 0;
static volatile uint32_t g_payload_self_ok_inline_cap_mismatch = 0;
static volatile uint32_t g_payload_self_ok_heap_cap_mismatch = 0;
static volatile uint32_t g_payload_self_ok_count_gt_entry_cap = 0;
static volatile uint32_t g_payload_self_ok_count_gt_max = 0;
static volatile uint32_t g_payload_self_ok_arena_used_gt_cap = 0;
static volatile uint32_t g_payload_self_ok_arena_cap_gt_max = 0;
static volatile uint32_t g_payload_self_ok_arena_cap_zero_with_ptr = 0;
static volatile uint32_t g_payload_self_ok_arena_cap_nonzero_with_null = 0;
static volatile uint32_t g_payload_self_ok_count_without_arena = 0;
static volatile uint32_t g_payload_self_ok_entry_kind_bad = 0;
static volatile uint32_t g_payload_self_ok_entry_key_off_oob = 0;
static volatile uint32_t g_payload_self_ok_entry_val_off_oob = 0;
static volatile uint32_t g_payload_self_ok_entry_val_end_oob = 0;
static volatile uint32_t g_payload_self_ok_entry_val_unterminated = 0;
static volatile uint32_t g_payload_self_ok_entry_key_unterminated = 0;
static volatile uint32_t g_payload_last_self_ok_fail_reason = 0;
static char g_payload_last_self_ok_fail_op[32] = {0};
static volatile uint32_t g_payload_last_self_ok_fail_this = 0;
static volatile uint32_t g_payload_last_self_ok_fail_magic = 0;
static volatile uint32_t g_payload_last_self_ok_fail_entries = 0;
static volatile uint32_t g_payload_last_self_ok_fail_arena = 0;
static volatile uint32_t g_payload_last_self_ok_fail_count = 0;
static volatile uint32_t g_payload_last_self_ok_fail_entry_cap = 0;
static volatile uint32_t g_payload_last_self_ok_fail_arena_used = 0;
static volatile uint32_t g_payload_last_self_ok_fail_arena_cap = 0;
static volatile uint32_t g_payload_last_self_ok_fail_entry_index = 0xFFFFFFFFUL;
static volatile uint32_t g_payload_last_self_ok_fail_entry_key_off = 0;
static volatile uint32_t g_payload_last_self_ok_fail_entry_val_off = 0;
static volatile uint32_t g_payload_last_self_ok_fail_entry_val_len = 0;
static volatile uint32_t g_payload_last_self_ok_fail_entry_kind = 0;
static volatile uint32_t g_payload_last_self_ok_fail_op_id = 0;
static volatile uint32_t g_payload_last_self_ok_fail_capacity = 0;
static volatile uint32_t g_payload_last_self_ok_fail_data_begin = 0;
static volatile uint32_t g_payload_last_self_ok_fail_expected_upper = 0;
static volatile uint32_t g_payload_last_self_ok_fail_key_end = 0;
static volatile uint32_t g_payload_last_self_ok_fail_val_end = 0;
static volatile uint32_t g_payload_first_self_ok_fail_captured = 0;

static volatile uint32_t g_payload_last_error_code = 0;
static volatile uint32_t g_payload_last_error_count = 0;
static volatile uint32_t g_payload_last_error_this = 0;
static volatile uint32_t g_payload_last_error_op_id = 0;
static char g_payload_last_error_op[32] = {0};

// ============================================================================
// Error and compatibility reason codes
// ============================================================================

enum payload_error_code_t : uint32_t {
    PAYLOAD_ERR_NONE = 0,
    PAYLOAD_ERR_ENTRY_OVERFLOW = 1,
    PAYLOAD_ERR_ENTRY_ALLOC_FAIL = 2,
    PAYLOAD_ERR_ARENA_ALLOC_FAIL = 3,
    PAYLOAD_ERR_ARENA_LIMIT = 4,
    PAYLOAD_ERR_STRING_TOO_LONG = 5,
    PAYLOAD_ERR_SERIALIZE_OVERFLOW = 6,
    PAYLOAD_ERR_TO_JSON_FAIL = 7,
    PAYLOAD_ERR_STRING_TRUNCATION = 8,
    PAYLOAD_ERR_PARSE_ERROR = 9,
    PAYLOAD_ERR_INTEGRITY = 10,
    PAYLOAD_ERR_INVALID_KIND = 11,
    PAYLOAD_ERR_COPY_ALLOC_FAIL = 12,
    PAYLOAD_ERR_BAD_STRING_POINTER = 13,
    PAYLOAD_ERR_STORAGE_CORRUPT = 14,
    PAYLOAD_ERR_JSON_INVALID = 15,
    PAYLOAD_ERR_NONFINITE_NUMBER = 16,
    PAYLOAD_ERR_INVALID_NUMERIC_TOKEN = 17,
    PAYLOAD_ERR_INVALID_CHILD = 18,
    PAYLOAD_ERR_FIXED_DECIMAL_RANGE = 19,
    PAYLOAD_ERR_FLOAT_FORMAT_FORBIDDEN = 20,
};

const char* payload_error_code_name(uint32_t code) {
    switch (code) {
        case PAYLOAD_ERR_NONE:               return "NONE";
        case PAYLOAD_ERR_ENTRY_OVERFLOW:     return "ENTRY_OVERFLOW";
        case PAYLOAD_ERR_ENTRY_ALLOC_FAIL:   return "ENTRY_ALLOC_FAIL";
        case PAYLOAD_ERR_ARENA_ALLOC_FAIL:   return "ARENA_ALLOC_FAIL";
        case PAYLOAD_ERR_ARENA_LIMIT:        return "ARENA_LIMIT";
        case PAYLOAD_ERR_STRING_TOO_LONG:    return "STRING_TOO_LONG";
        case PAYLOAD_ERR_SERIALIZE_OVERFLOW: return "SERIALIZE_OVERFLOW";
        case PAYLOAD_ERR_TO_JSON_FAIL:       return "TO_JSON_FAIL";
        case PAYLOAD_ERR_STRING_TRUNCATION:  return "STRING_TRUNCATION";
        case PAYLOAD_ERR_PARSE_ERROR:        return "PARSE_ERROR";
        case PAYLOAD_ERR_INTEGRITY:          return "INTEGRITY";
        case PAYLOAD_ERR_INVALID_KIND:       return "INVALID_KIND";
        case PAYLOAD_ERR_COPY_ALLOC_FAIL:    return "COPY_ALLOC_FAIL";
        case PAYLOAD_ERR_BAD_STRING_POINTER: return "BAD_STRING_POINTER";
        case PAYLOAD_ERR_STORAGE_CORRUPT:    return "STORAGE_CORRUPT";
        case PAYLOAD_ERR_JSON_INVALID:       return "JSON_INVALID";
        case PAYLOAD_ERR_NONFINITE_NUMBER:   return "NONFINITE_NUMBER";
        case PAYLOAD_ERR_INVALID_NUMERIC_TOKEN: return "INVALID_NUMERIC_TOKEN";
        case PAYLOAD_ERR_INVALID_CHILD:      return "INVALID_CHILD";
        case PAYLOAD_ERR_FIXED_DECIMAL_RANGE: return "FIXED_DECIMAL_RANGE";
        case PAYLOAD_ERR_FLOAT_FORMAT_FORBIDDEN: return "FLOAT_FORMAT_FORBIDDEN";
        default:                             return "UNKNOWN";
    }
}

enum payload_string_fault_reason_t : uint32_t {
    PAYLOAD_STRING_FAULT_NONE = 0,
    PAYLOAD_STRING_FAULT_NULL = 1,
    PAYLOAD_STRING_FAULT_LOW_ADDRESS = 2,
    PAYLOAD_STRING_FAULT_MAGIC_ADDRESS = 3,
    PAYLOAD_STRING_FAULT_OUT_OF_RANGE = 4,
    PAYLOAD_STRING_FAULT_SPAN_OUT_OF_RANGE = 5,
    PAYLOAD_STRING_FAULT_UNTERMINATED = 6,
    PAYLOAD_STRING_FAULT_TOO_LONG = 7,
};

const char* payload_string_fault_reason_name(uint32_t reason) {
    switch (reason) {
        case PAYLOAD_STRING_FAULT_NONE:              return "NONE";
        case PAYLOAD_STRING_FAULT_NULL:              return "NULL";
        case PAYLOAD_STRING_FAULT_LOW_ADDRESS:       return "LOW_ADDRESS";
        case PAYLOAD_STRING_FAULT_MAGIC_ADDRESS:     return "MAGIC_ADDRESS";
        case PAYLOAD_STRING_FAULT_OUT_OF_RANGE:      return "OUT_OF_RANGE";
        case PAYLOAD_STRING_FAULT_SPAN_OUT_OF_RANGE: return "SPAN_OUT_OF_RANGE";
        case PAYLOAD_STRING_FAULT_UNTERMINATED:      return "UNTERMINATED";
        case PAYLOAD_STRING_FAULT_TOO_LONG:          return "TOO_LONG";
        default:                                     return "UNKNOWN";
    }
}

enum payload_self_ok_fail_reason_t : uint32_t {
    PAYLOAD_SELF_OK_NONE = 0,
    PAYLOAD_SELF_OK_MAGIC_BAD = 1,
    PAYLOAD_SELF_OK_ENTRIES_NULL = 2,
    PAYLOAD_SELF_OK_ENTRY_CAP_LOW = 3,
    PAYLOAD_SELF_OK_ENTRY_CAP_HIGH = 4,
    PAYLOAD_SELF_OK_ENTRIES_MAGIC_ADDRESS = 5,
    PAYLOAD_SELF_OK_ARENA_MAGIC_ADDRESS = 6,
    PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE = 7,
    PAYLOAD_SELF_OK_ARENA_SPAN_UNREADABLE = 8,
    PAYLOAD_SELF_OK_INLINE_CAP_MISMATCH = 9,
    PAYLOAD_SELF_OK_HEAP_CAP_MISMATCH = 10,
    PAYLOAD_SELF_OK_COUNT_GT_ENTRY_CAP = 11,
    PAYLOAD_SELF_OK_COUNT_GT_MAX = 12,
    PAYLOAD_SELF_OK_ARENA_USED_GT_CAP = 13,
    PAYLOAD_SELF_OK_ARENA_CAP_GT_MAX = 14,
    PAYLOAD_SELF_OK_ARENA_CAP_ZERO_WITH_PTR = 15,
    PAYLOAD_SELF_OK_ARENA_CAP_NONZERO_WITH_NULL = 16,
    PAYLOAD_SELF_OK_COUNT_WITHOUT_ARENA = 17,
    PAYLOAD_SELF_OK_ENTRY_KIND_BAD = 18,
    PAYLOAD_SELF_OK_ENTRY_KEY_OFF_OOB = 19,
    PAYLOAD_SELF_OK_ENTRY_VAL_OFF_OOB = 20,
    PAYLOAD_SELF_OK_ENTRY_VAL_END_OOB = 21,
    PAYLOAD_SELF_OK_ENTRY_VAL_UNTERMINATED = 22,
    PAYLOAD_SELF_OK_ENTRY_KEY_UNTERMINATED = 23,
};

const char* payload_self_ok_fail_reason_name(uint32_t reason) {
    switch (reason) {
        case PAYLOAD_SELF_OK_NONE:                        return "NONE";
        case PAYLOAD_SELF_OK_MAGIC_BAD:                   return "MAGIC_BAD_RETIRED_V4";
        case PAYLOAD_SELF_OK_ENTRIES_NULL:                return "STORAGE_NULL";
        case PAYLOAD_SELF_OK_ENTRY_CAP_LOW:               return "ENTRY_CAP_LOW";
        case PAYLOAD_SELF_OK_ENTRY_CAP_HIGH:              return "ENTRY_CAP_HIGH";
        case PAYLOAD_SELF_OK_ENTRIES_MAGIC_ADDRESS:       return "ENTRIES_MAGIC_ADDRESS_RETIRED_V4";
        case PAYLOAD_SELF_OK_ARENA_MAGIC_ADDRESS:         return "ARENA_MAGIC_ADDRESS_RETIRED_V4";
        case PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE:     return "STORAGE_BLOCK_UNREADABLE";
        case PAYLOAD_SELF_OK_ARENA_SPAN_UNREADABLE:       return "DATA_SPAN_UNREADABLE";
        case PAYLOAD_SELF_OK_INLINE_CAP_MISMATCH:         return "INLINE_CAP_MISMATCH";
        case PAYLOAD_SELF_OK_HEAP_CAP_MISMATCH:           return "HEAP_CAP_MISMATCH";
        case PAYLOAD_SELF_OK_COUNT_GT_ENTRY_CAP:          return "COUNT_GT_ENTRY_CAP";
        case PAYLOAD_SELF_OK_COUNT_GT_MAX:                return "COUNT_GT_MAX";
        case PAYLOAD_SELF_OK_ARENA_USED_GT_CAP:           return "DATA_USED_GT_CAP";
        case PAYLOAD_SELF_OK_ARENA_CAP_GT_MAX:            return "STORAGE_CAP_GT_MAX";
        case PAYLOAD_SELF_OK_ARENA_CAP_ZERO_WITH_PTR:     return "CAP_ZERO_WITH_PTR";
        case PAYLOAD_SELF_OK_ARENA_CAP_NONZERO_WITH_NULL: return "CAP_NONZERO_WITH_NULL";
        case PAYLOAD_SELF_OK_COUNT_WITHOUT_ARENA:         return "COUNT_WITHOUT_DATA";
        case PAYLOAD_SELF_OK_ENTRY_KIND_BAD:              return "ENTRY_KIND_BAD";
        case PAYLOAD_SELF_OK_ENTRY_KEY_OFF_OOB:           return "ENTRY_KEY_OFF_OOB";
        case PAYLOAD_SELF_OK_ENTRY_VAL_OFF_OOB:           return "ENTRY_VAL_OFF_OOB";
        case PAYLOAD_SELF_OK_ENTRY_VAL_END_OOB:           return "ENTRY_VAL_END_OOB";
        case PAYLOAD_SELF_OK_ENTRY_VAL_UNTERMINATED:      return "ENTRY_VAL_UNTERMINATED";
        case PAYLOAD_SELF_OK_ENTRY_KEY_UNTERMINATED:      return "ENTRY_KEY_UNTERMINATED";
        default:                                          return "UNKNOWN";
    }
}


enum payload_numeric_reject_reason_t : uint32_t {
    PAYLOAD_NUMERIC_REJECT_NONE = 0,
    PAYLOAD_NUMERIC_REJECT_NAN = 1,
    PAYLOAD_NUMERIC_REJECT_POSITIVE_INFINITY = 2,
    PAYLOAD_NUMERIC_REJECT_NEGATIVE_INFINITY = 3,
    PAYLOAD_NUMERIC_REJECT_INVALID_TOKEN = 4,
    PAYLOAD_NUMERIC_REJECT_FORMAT_FAILURE = 5,
    PAYLOAD_NUMERIC_REJECT_OUT_OF_RANGE = 6,
};

const char* payload_numeric_reject_reason_name(uint32_t reason) {
    switch (reason) {
        case PAYLOAD_NUMERIC_REJECT_NONE: return "NONE";
        case PAYLOAD_NUMERIC_REJECT_NAN: return "NAN";
        case PAYLOAD_NUMERIC_REJECT_POSITIVE_INFINITY: return "POSITIVE_INFINITY";
        case PAYLOAD_NUMERIC_REJECT_NEGATIVE_INFINITY: return "NEGATIVE_INFINITY";
        case PAYLOAD_NUMERIC_REJECT_INVALID_TOKEN: return "INVALID_TOKEN";
        case PAYLOAD_NUMERIC_REJECT_FORMAT_FAILURE: return "FORMAT_FAILURE";
        case PAYLOAD_NUMERIC_REJECT_OUT_OF_RANGE: return "OUT_OF_RANGE";
        default: return "UNKNOWN";
    }
}

enum payload_semantic_fail_reason_t : uint32_t {
    PAYLOAD_SEMANTIC_FAIL_NONE = 0,
    PAYLOAD_SEMANTIC_FAIL_INVALID_KIND = 1,
    PAYLOAD_SEMANTIC_FAIL_KEY_UTF8 = 2,
    PAYLOAD_SEMANTIC_FAIL_STRING_UTF8 = 3,
    PAYLOAD_SEMANTIC_FAIL_NUMBER_TOKEN = 4,
    PAYLOAD_SEMANTIC_FAIL_BOOLEAN_TOKEN = 5,
    PAYLOAD_SEMANTIC_FAIL_NULL_TOKEN = 6,
    PAYLOAD_SEMANTIC_FAIL_OBJECT_JSON = 7,
    PAYLOAD_SEMANTIC_FAIL_ARRAY_JSON = 8,
};

const char* payload_semantic_fail_reason_name(uint32_t reason) {
    switch (reason) {
        case PAYLOAD_SEMANTIC_FAIL_NONE: return "NONE";
        case PAYLOAD_SEMANTIC_FAIL_INVALID_KIND: return "INVALID_KIND";
        case PAYLOAD_SEMANTIC_FAIL_KEY_UTF8: return "KEY_UTF8";
        case PAYLOAD_SEMANTIC_FAIL_STRING_UTF8: return "STRING_UTF8";
        case PAYLOAD_SEMANTIC_FAIL_NUMBER_TOKEN: return "NUMBER_TOKEN";
        case PAYLOAD_SEMANTIC_FAIL_BOOLEAN_TOKEN: return "BOOLEAN_TOKEN";
        case PAYLOAD_SEMANTIC_FAIL_NULL_TOKEN: return "NULL_TOKEN";
        case PAYLOAD_SEMANTIC_FAIL_OBJECT_JSON: return "OBJECT_JSON";
        case PAYLOAD_SEMANTIC_FAIL_ARRAY_JSON: return "ARRAY_JSON";
        default: return "UNKNOWN";
    }
}


// ============================================================================
// Stable numeric operation identity
// ============================================================================
//
// These values preserve the FNV-1a numbers emitted by the previous diagnostic
// build, but they are now compile-time constants. Failure bookkeeping never
// receives or traverses an operation-label pointer.

enum payload_operation_id_t : uint32_t {
    PAYLOAD_OP_NONE = 0U,
    PAYLOAD_OP_ADD_KEY = 0xD462A255UL,
    PAYLOAD_OP_ADD_ARRAY_KEY = 0xFE1083ABUL,
    PAYLOAD_OP_ADD_ARRAY_VALUE = 0xB11A9C75UL,
    PAYLOAD_OP_ADD_BOOL_KEY = 0xAC115924UL,
    PAYLOAD_OP_ADD_CSTR_KEY = 0x295273BCUL,
    PAYLOAD_OP_ADD_CSTR_VALUE = 0xD6E6DC3AUL,
    PAYLOAD_OP_ADD_DOUBLE = 0x72DB5A0CUL,
    PAYLOAD_OP_ADD_DOUBLE_PRECISION = 0xB5BA038BUL,
    PAYLOAD_OP_ADD_FLOAT = 0xF0CDAC89UL,
    PAYLOAD_OP_ADD_FIXED_DECIMAL = 0x36F83575UL,
    PAYLOAD_OP_ADD_FIXED_DECIMAL_KEY = 0xB66DA308UL,
    PAYLOAD_OP_ADD_FMT = 0x04FEDBF4UL,
    PAYLOAD_OP_ADD_FMT_FORMAT = 0x0ADAD95DUL,
    PAYLOAD_OP_ADD_FMT_KEY = 0x891B83D5UL,
    PAYLOAD_OP_ADD_I32 = 0xA34A6729UL,
    PAYLOAD_OP_ADD_I32_KEY = 0x1E7A71ACUL,
    PAYLOAD_OP_ADD_I64 = 0x37520BEAUL,
    PAYLOAD_OP_ADD_I64_KEY = 0x7A4C9DBBUL,
    PAYLOAD_OP_ADD_OBJECT_KEY = 0x7DB29AFFUL,
    PAYLOAD_OP_ADD_OBJECT_VALUE = 0xB4039D31UL,
    PAYLOAD_OP_ADD_RAW_KEY = 0x1A1C9ED0UL,
    PAYLOAD_OP_ADD_RAW_VALUE = 0xFC7A1C46UL,
    PAYLOAD_OP_ADD_U32 = 0xB944654DUL,
    PAYLOAD_OP_ADD_U32_KEY = 0x5FA4E370UL,
    PAYLOAD_OP_ADD_U64 = 0x35385C8EUL,
    PAYLOAD_OP_ADD_U64_KEY = 0xEB27CA3FUL,
    PAYLOAD_OP_APPEND_KEY = 0x03CBF254UL,
    PAYLOAD_OP_APPEND_INPUT_VALUE = 0x91FCC762UL,
    PAYLOAD_OP_APPEND_VALUE = 0x60D27BE3UL,
    PAYLOAD_OP_APPEND_VALUE_LENGTH = 0x80AAE437UL,
    PAYLOAD_OP_APPEND_VALUE_UTF8 = 0x333B0378UL,
    PAYLOAD_OP_APPEND_FINAL_KEY_SPAN = 0x4E7E6B11UL,
    PAYLOAD_OP_APPEND_FINAL_VALUE_SPAN = 0xA8C2F6D3UL,
    PAYLOAD_OP_APPEND_WRITER = 0xCF4278D5UL,
    PAYLOAD_OP_APPEND_WRITER_KEY = 0x3151E968UL,
    PAYLOAD_OP_APPEND_WRITER_LENGTH = 0x06E0EA19UL,
    PAYLOAD_OP_APPEND_WRITER_UTF8 = 0xF44D5A06UL,
    PAYLOAD_OP_APPEND_WRITER_WRITE = 0x92F18D08UL,
    PAYLOAD_OP_ARENA_CAPACITY = 0xA6E32F79UL,
    PAYLOAD_OP_ARENA_USED = 0x008915C0UL,
    PAYLOAD_OP_ARRAY_ADD_WRITE = 0x0806A7A2UL,
    PAYLOAD_OP_ARRAY_ALLOC = 0x5942F3EDUL,
    PAYLOAD_OP_ARRAY_CAPACITY = 0x2398A2BCUL,
    PAYLOAD_OP_ARRAY_PARSE = 0x59F15837UL,
    PAYLOAD_OP_ARRAY_PARSE_INVALID = 0xE535F37AUL,
    PAYLOAD_OP_ARRAY_RELEASE_STORAGE = 0xE6B1BDEBUL,
    PAYLOAD_OP_ARRAY_RELEASE_STORAGE_GUARD = 0x78A75C2CUL,
    PAYLOAD_OP_ARRAY_VIEW_GET = 0xEF3B8ABCUL,
    PAYLOAD_OP_ARRAY_VIEW_SIZE = 0xB1DB2071UL,
    PAYLOAD_OP_ARRAY_VIEW_VALID = 0x6CA62C42UL,
    PAYLOAD_OP_CLEAR = 0x5C6E1222UL,
    PAYLOAD_OP_COPY_ASSIGN = 0x0EF80038UL,
    PAYLOAD_OP_COPY_ASSIGN_SOURCE = 0x0A7D13DFUL,
    PAYLOAD_OP_COPY_CTOR = 0x39F7EA4BUL,
    PAYLOAD_OP_COPY_FROM = 0x8FB0C939UL,
    PAYLOAD_OP_COUNT = 0x39B1DDF4UL,
    PAYLOAD_OP_CTOR = 0x3D91C44FUL,
    PAYLOAD_OP_DEBUG_DUMP = 0x68635107UL,
    PAYLOAD_OP_DEBUG_DUMP_ENTRY = 0xBF4676B5UL,
    PAYLOAD_OP_DEBUG_DUMP_TAG = 0xDB35BE39UL,
    PAYLOAD_OP_DTOR = 0x64A0BB17UL,
    PAYLOAD_OP_EMPTY = 0x18A7BEEEUL,
    PAYLOAD_OP_ENSURE_ROOM = 0xCDBCBCCBUL,
    PAYLOAD_OP_ENSURE_ROOM_ALLOC = 0xD6807594UL,
    PAYLOAD_OP_ENSURE_ROOM_CAPACITY = 0x74CBBAABUL,
    PAYLOAD_OP_ENSURE_ROOM_DATA = 0xC65F5CE7UL,
    PAYLOAD_OP_ENSURE_ROOM_ENTRIES = 0x9B2E5D25UL,
    PAYLOAD_OP_ENSURE_ROOM_GROW = 0x16AF044CUL,
    PAYLOAD_OP_ENTRY_CAPACITY = 0x8AA7EB40UL,
    PAYLOAD_OP_FIND_KEY = 0x3466FF4BUL,
    PAYLOAD_OP_FIND_LAYOUT = 0xD2714752UL,
    PAYLOAD_OP_GETARRAY = 0xC97863F4UL,
    PAYLOAD_OP_GETARRAYVIEW = 0x073EA9EDUL,
    PAYLOAD_OP_GETPAYLOAD = 0x76B94E07UL,
    PAYLOAD_OP_HASARRAY = 0x6AC7AE38UL,
    PAYLOAD_OP_HEAP_BLOCK = 0x71CF741CUL,
    PAYLOAD_OP_JSON_SIZE = 0xAE652ECDUL,
    PAYLOAD_OP_JSON_SIZE_VALUE = 0xD8354B66UL,
    PAYLOAD_OP_MOVE_ASSIGN_SOURCE = 0xECBE926FUL,
    PAYLOAD_OP_MOVE_FROM = 0x376E8D29UL,
    PAYLOAD_OP_PARSEJSON_COLON = 0x90CC8523UL,
    PAYLOAD_OP_PARSEJSON_DATA = 0xB5EF7548UL,
    PAYLOAD_OP_PARSEJSON_EOF = 0x9BC18F22UL,
    PAYLOAD_OP_PARSEJSON_KEY = 0x2456FA1FUL,
    PAYLOAD_OP_PARSEJSON_KEY_DECODE = 0x83298432UL,
    PAYLOAD_OP_PARSEJSON_LENGTH = 0xCECF6BDCUL,
    PAYLOAD_OP_PARSEJSON_OPEN = 0x0312FEA8UL,
    PAYLOAD_OP_PARSEJSON_RESERVE = 0xA69D0D48UL,
    PAYLOAD_OP_PARSEJSON_SEPARATOR = 0xAAB531E7UL,
    PAYLOAD_OP_PARSEJSON_TRAILING = 0xF0F9BD2CUL,
    PAYLOAD_OP_PARSEJSON_UNCLOSED = 0x8FED3DF7UL,
    PAYLOAD_OP_PARSEJSON_VALUE = 0x4CA3F351UL,
    PAYLOAD_OP_PARSEJSON_VALUE_DECODE = 0x956CAB1CUL,
    PAYLOAD_OP_RELEASE_STORAGE = 0x6705E500UL,
    PAYLOAD_OP_RELEASE_STORAGE_GUARD = 0x59BE62E3UL,
    PAYLOAD_OP_TO_JSON_ALLOC = 0x81EEC028UL,
    PAYLOAD_OP_TO_JSON_SIZE = 0xBF250482UL,
    PAYLOAD_OP_TO_JSON_WRITE = 0xF163FED6UL,
    PAYLOAD_OP_TRY_BOOL = 0xFA4E2B77UL,
    PAYLOAD_OP_TRY_DOUBLE = 0x92DDD42EUL,
    PAYLOAD_OP_TRY_DOUBLE_TOKEN = 0x920353D5UL,
    PAYLOAD_OP_TRY_FLOAT = 0xD41F48A7UL,
    PAYLOAD_OP_TRY_FLOAT_TOKEN = 0xEAAF4288UL,
    PAYLOAD_OP_TRY_INT = 0x28A94CECUL,
    PAYLOAD_OP_TRY_INT_TOKEN = 0x0E49D8B3UL,
    PAYLOAD_OP_TRY_UINT = 0x1EF3A67BUL,
    PAYLOAD_OP_TRY_UINT_TOKEN = 0xD004E164UL,
    PAYLOAD_OP_TRY_UINT64 = 0x55C995B9UL,
    PAYLOAD_OP_TRY_UINT64_TOKEN = 0x98721022UL,
    PAYLOAD_OP_WRITE_JSON_BUFFER = 0xE0DEFD21UL,
    PAYLOAD_OP_WRITE_JSON_SIZE = 0xA14EE86AUL,
};

const char* payload_operation_id_name(uint32_t operation_id) {
    switch (operation_id) {
        case PAYLOAD_OP_NONE: return "NONE";
        case PAYLOAD_OP_ADD_KEY: return "add.key";
        case PAYLOAD_OP_ADD_ARRAY_KEY: return "add_array.key";
        case PAYLOAD_OP_ADD_ARRAY_VALUE: return "add_array.value";
        case PAYLOAD_OP_ADD_BOOL_KEY: return "add_bool.key";
        case PAYLOAD_OP_ADD_CSTR_KEY: return "add_cstr.key";
        case PAYLOAD_OP_ADD_CSTR_VALUE: return "add_cstr.value";
        case PAYLOAD_OP_ADD_DOUBLE: return "add_double";
        case PAYLOAD_OP_ADD_DOUBLE_PRECISION: return "add_double_precision";
        case PAYLOAD_OP_ADD_FLOAT: return "add_float";
        case PAYLOAD_OP_ADD_FIXED_DECIMAL: return "add_fixed_decimal";
        case PAYLOAD_OP_ADD_FIXED_DECIMAL_KEY: return "add_fixed_decimal.key";
        case PAYLOAD_OP_ADD_FMT: return "add_fmt";
        case PAYLOAD_OP_ADD_FMT_FORMAT: return "add_fmt.format";
        case PAYLOAD_OP_ADD_FMT_KEY: return "add_fmt.key";
        case PAYLOAD_OP_ADD_I32: return "add_i32";
        case PAYLOAD_OP_ADD_I32_KEY: return "add_i32.key";
        case PAYLOAD_OP_ADD_I64: return "add_i64";
        case PAYLOAD_OP_ADD_I64_KEY: return "add_i64.key";
        case PAYLOAD_OP_ADD_OBJECT_KEY: return "add_object.key";
        case PAYLOAD_OP_ADD_OBJECT_VALUE: return "add_object.value";
        case PAYLOAD_OP_ADD_RAW_KEY: return "add_raw.key";
        case PAYLOAD_OP_ADD_RAW_VALUE: return "add_raw.value";
        case PAYLOAD_OP_ADD_U32: return "add_u32";
        case PAYLOAD_OP_ADD_U32_KEY: return "add_u32.key";
        case PAYLOAD_OP_ADD_U64: return "add_u64";
        case PAYLOAD_OP_ADD_U64_KEY: return "add_u64.key";
        case PAYLOAD_OP_APPEND_KEY: return "append.key";
        case PAYLOAD_OP_APPEND_INPUT_VALUE: return "append.value";
        case PAYLOAD_OP_APPEND_VALUE: return "append_value";
        case PAYLOAD_OP_APPEND_VALUE_LENGTH: return "append_value.length";
        case PAYLOAD_OP_APPEND_VALUE_UTF8: return "append_value.utf8";
        case PAYLOAD_OP_APPEND_FINAL_KEY_SPAN: return "append_value.final_key_span";
        case PAYLOAD_OP_APPEND_FINAL_VALUE_SPAN: return "append_value.final_value_span";
        case PAYLOAD_OP_APPEND_WRITER: return "append_writer";
        case PAYLOAD_OP_APPEND_WRITER_KEY: return "append_writer.key";
        case PAYLOAD_OP_APPEND_WRITER_LENGTH: return "append_writer.length";
        case PAYLOAD_OP_APPEND_WRITER_UTF8: return "append_writer.utf8";
        case PAYLOAD_OP_APPEND_WRITER_WRITE: return "append_writer.write";
        case PAYLOAD_OP_ARENA_CAPACITY: return "arena_capacity";
        case PAYLOAD_OP_ARENA_USED: return "arena_used";
        case PAYLOAD_OP_ARRAY_ADD_WRITE: return "array.add.write";
        case PAYLOAD_OP_ARRAY_ALLOC: return "array.alloc";
        case PAYLOAD_OP_ARRAY_CAPACITY: return "array.capacity";
        case PAYLOAD_OP_ARRAY_PARSE: return "array.parse";
        case PAYLOAD_OP_ARRAY_PARSE_INVALID: return "array.parse.invalid";
        case PAYLOAD_OP_ARRAY_RELEASE_STORAGE: return "array.release_storage";
        case PAYLOAD_OP_ARRAY_RELEASE_STORAGE_GUARD: return "array.release_storage.guard";
        case PAYLOAD_OP_ARRAY_VIEW_GET: return "array_view.get";
        case PAYLOAD_OP_ARRAY_VIEW_SIZE: return "array_view.size";
        case PAYLOAD_OP_ARRAY_VIEW_VALID: return "array_view.valid";
        case PAYLOAD_OP_CLEAR: return "clear";
        case PAYLOAD_OP_COPY_ASSIGN: return "copy_assign";
        case PAYLOAD_OP_COPY_ASSIGN_SOURCE: return "copy_assign.source";
        case PAYLOAD_OP_COPY_CTOR: return "copy_ctor";
        case PAYLOAD_OP_COPY_FROM: return "copy_from";
        case PAYLOAD_OP_COUNT: return "count";
        case PAYLOAD_OP_CTOR: return "ctor";
        case PAYLOAD_OP_DEBUG_DUMP: return "debug_dump";
        case PAYLOAD_OP_DEBUG_DUMP_ENTRY: return "debug_dump.entry";
        case PAYLOAD_OP_DEBUG_DUMP_TAG: return "debug_dump.tag";
        case PAYLOAD_OP_DTOR: return "dtor";
        case PAYLOAD_OP_EMPTY: return "empty";
        case PAYLOAD_OP_ENSURE_ROOM: return "ensure_room";
        case PAYLOAD_OP_ENSURE_ROOM_ALLOC: return "ensure_room.alloc";
        case PAYLOAD_OP_ENSURE_ROOM_CAPACITY: return "ensure_room.capacity";
        case PAYLOAD_OP_ENSURE_ROOM_DATA: return "ensure_room.data";
        case PAYLOAD_OP_ENSURE_ROOM_ENTRIES: return "ensure_room.entries";
        case PAYLOAD_OP_ENSURE_ROOM_GROW: return "ensure_room.grow";
        case PAYLOAD_OP_ENTRY_CAPACITY: return "entry_capacity";
        case PAYLOAD_OP_FIND_KEY: return "find.key";
        case PAYLOAD_OP_FIND_LAYOUT: return "find.layout";
        case PAYLOAD_OP_GETARRAY: return "getArray";
        case PAYLOAD_OP_GETARRAYVIEW: return "getArrayView";
        case PAYLOAD_OP_GETPAYLOAD: return "getPayload";
        case PAYLOAD_OP_HASARRAY: return "hasArray";
        case PAYLOAD_OP_HEAP_BLOCK: return "heap.block";
        case PAYLOAD_OP_JSON_SIZE: return "json_size";
        case PAYLOAD_OP_JSON_SIZE_VALUE: return "json_size.value";
        case PAYLOAD_OP_MOVE_ASSIGN_SOURCE: return "move_assign.source";
        case PAYLOAD_OP_MOVE_FROM: return "move_from";
        case PAYLOAD_OP_PARSEJSON_COLON: return "parseJSON.colon";
        case PAYLOAD_OP_PARSEJSON_DATA: return "parseJSON.data";
        case PAYLOAD_OP_PARSEJSON_EOF: return "parseJSON.eof";
        case PAYLOAD_OP_PARSEJSON_KEY: return "parseJSON.key";
        case PAYLOAD_OP_PARSEJSON_KEY_DECODE: return "parseJSON.key_decode";
        case PAYLOAD_OP_PARSEJSON_LENGTH: return "parseJSON.length";
        case PAYLOAD_OP_PARSEJSON_OPEN: return "parseJSON.open";
        case PAYLOAD_OP_PARSEJSON_RESERVE: return "parseJSON.reserve";
        case PAYLOAD_OP_PARSEJSON_SEPARATOR: return "parseJSON.separator";
        case PAYLOAD_OP_PARSEJSON_TRAILING: return "parseJSON.trailing";
        case PAYLOAD_OP_PARSEJSON_UNCLOSED: return "parseJSON.unclosed";
        case PAYLOAD_OP_PARSEJSON_VALUE: return "parseJSON.value";
        case PAYLOAD_OP_PARSEJSON_VALUE_DECODE: return "parseJSON.value_decode";
        case PAYLOAD_OP_RELEASE_STORAGE: return "release_storage";
        case PAYLOAD_OP_RELEASE_STORAGE_GUARD: return "release_storage.guard";
        case PAYLOAD_OP_TO_JSON_ALLOC: return "to_json.alloc";
        case PAYLOAD_OP_TO_JSON_SIZE: return "to_json.size";
        case PAYLOAD_OP_TO_JSON_WRITE: return "to_json.write";
        case PAYLOAD_OP_TRY_BOOL: return "try_bool";
        case PAYLOAD_OP_TRY_DOUBLE: return "try_double";
        case PAYLOAD_OP_TRY_DOUBLE_TOKEN: return "try_double.token";
        case PAYLOAD_OP_TRY_FLOAT: return "try_float";
        case PAYLOAD_OP_TRY_FLOAT_TOKEN: return "try_float.token";
        case PAYLOAD_OP_TRY_INT: return "try_int";
        case PAYLOAD_OP_TRY_INT_TOKEN: return "try_int.token";
        case PAYLOAD_OP_TRY_UINT: return "try_uint";
        case PAYLOAD_OP_TRY_UINT_TOKEN: return "try_uint.token";
        case PAYLOAD_OP_TRY_UINT64: return "try_uint64";
        case PAYLOAD_OP_TRY_UINT64_TOKEN: return "try_uint64.token";
        case PAYLOAD_OP_WRITE_JSON_BUFFER: return "write_json.buffer";
        case PAYLOAD_OP_WRITE_JSON_SIZE: return "write_json.size";
        default: return "UNKNOWN";
    }
}

struct payload_layout_evidence_t {
    uint32_t entry_index;
    uint32_t capacity;
    uint32_t data_begin;
    uint32_t entry_count;
    uint32_t key_off;
    uint32_t key_len;
    uint32_t val_off;
    uint32_t val_len;
    uint32_t entry_kind;
    uint32_t expected_upper;
    uint32_t key_end;
    uint32_t val_end;
};

// ============================================================================
// Small, non-recursive diagnostics
// ============================================================================

static void payload_copy_label(char* dst, size_t cap, const char* src) {
    if (!dst || cap == 0) return;
    if (!src) src = "?";
    size_t i = 0;
    while (i + 1U < cap && src[i] != '\0') {
        dst[i] = src[i];
        ++i;
    }
    dst[i] = '\0';
}


static void payload_copy_span_label(char* dst,
                                    size_t cap,
                                    const char* src,
                                    size_t len) {
    if (!dst || cap == 0) return;
    if (!src) len = 0;
    if (len >= cap) len = cap - 1U;
    if (len != 0) memcpy(dst, src, len);
    dst[len] = '\0';
}

static size_t payload_bounded_local_strlen(const char* text,
                                           size_t capacity,
                                           bool* terminated) {
    if (terminated) *terminated = false;
    if (!text || capacity == 0U) return 0U;
    for (size_t i = 0; i < capacity; ++i) {
        if (text[i] == '\0') {
            if (terminated) *terminated = true;
            return i;
        }
    }
    return capacity;
}

static char payload_hex_digit(uint8_t value) {
    return value < 10U ? (char)('0' + value)
                       : (char)('A' + (value - 10U));
}

static void payload_capture_numeric_text(const char* text,
                                         size_t capacity,
                                         int formatter_return) {
    bool terminated = false;
    const size_t observed_len =
        payload_bounded_local_strlen(text, capacity, &terminated);
    const size_t retained_len = observed_len < 63U ? observed_len : 63U;

    for (size_t i = 0; i < retained_len; ++i) {
        const uint8_t byte = (uint8_t)text[i];
        g_payload_last_numeric_reject_text_printable[i] =
            (byte >= 0x20U && byte <= 0x7EU) ? (char)byte : '.';
    }
    g_payload_last_numeric_reject_text_printable[retained_len] = '\0';

    const size_t hex_len = observed_len < 64U ? observed_len : 64U;
    for (size_t i = 0; i < hex_len; ++i) {
        const uint8_t byte = (uint8_t)text[i];
        g_payload_last_numeric_reject_text_hex[i * 2U] =
            payload_hex_digit((uint8_t)(byte >> 4));
        g_payload_last_numeric_reject_text_hex[i * 2U + 1U] =
            payload_hex_digit((uint8_t)(byte & 0x0FU));
    }
    g_payload_last_numeric_reject_text_hex[hex_len * 2U] = '\0';

    g_payload_last_numeric_reject_text_len = (uint32_t)observed_len;
    g_payload_last_numeric_reject_text_terminated = terminated ? 1U : 0U;
    g_payload_last_numeric_reject_text_truncated =
        (!terminated || formatter_return < 0 ||
         (size_t)formatter_return >= capacity) ? 1U : 0U;
}

static void payload_note_numeric_reject(uint32_t reason,
                                        uint32_t operation_id,
                                        const void* self,
                                        const char* key,
                                        size_t key_len,
                                        uint64_t source_value_bits,
                                        int precision,
                                        const char* format,
                                        size_t format_capacity,
                                        int format_return,
                                        const char* text,
                                        size_t text_capacity,
                                        int formatter_return) {
    g_payload_last_numeric_reject_reason = reason;
    g_payload_last_numeric_reject_op_id = operation_id;
    g_payload_last_numeric_reject_this = (uint32_t)(uintptr_t)self;
    payload_copy_span_label(g_payload_last_numeric_reject_key,
                            sizeof(g_payload_last_numeric_reject_key),
                            key,
                            key_len);

    // Retained telemetry field names remain source-compatible with the former
    // floating-point API.  util.cpp captures the source IEEE-754 bits before
    // Payload receives this integer-only admission object.
    g_payload_last_numeric_reject_value_bits = source_value_bits;
    g_payload_last_numeric_reject_precision = precision;
    g_payload_last_numeric_reject_format_return = format_return;
    g_payload_last_numeric_reject_snprintf_return = formatter_return;

    bool format_terminated = false;
    const size_t format_len =
        payload_bounded_local_strlen(format, format_capacity, &format_terminated);
    (void)format_terminated;
    payload_copy_span_label(g_payload_last_numeric_reject_format,
                            sizeof(g_payload_last_numeric_reject_format),
                            format,
                            format_len);
    payload_capture_numeric_text(text, text_capacity, formatter_return);

    switch (reason) {
        case PAYLOAD_NUMERIC_REJECT_NAN:
            g_payload_numeric_nonfinite++;
            g_payload_numeric_nan++;
            break;
        case PAYLOAD_NUMERIC_REJECT_POSITIVE_INFINITY:
            g_payload_numeric_nonfinite++;
            g_payload_numeric_positive_infinity++;
            break;
        case PAYLOAD_NUMERIC_REJECT_NEGATIVE_INFINITY:
            g_payload_numeric_nonfinite++;
            g_payload_numeric_negative_infinity++;
            break;
        case PAYLOAD_NUMERIC_REJECT_INVALID_TOKEN:
            g_payload_numeric_invalid_token++;
            break;
        case PAYLOAD_NUMERIC_REJECT_FORMAT_FAILURE:
        case PAYLOAD_NUMERIC_REJECT_OUT_OF_RANGE:
            g_payload_numeric_format_failure++;
            break;
        default:
            break;
    }
}

static void payload_increment_semantic_reason(uint32_t reason) {
    switch (reason) {
        case PAYLOAD_SEMANTIC_FAIL_INVALID_KIND:
            g_payload_semantic_invalid_kind++;
            break;
        case PAYLOAD_SEMANTIC_FAIL_KEY_UTF8:
            g_payload_semantic_invalid_key_utf8++;
            break;
        case PAYLOAD_SEMANTIC_FAIL_STRING_UTF8:
            g_payload_semantic_invalid_string_utf8++;
            break;
        case PAYLOAD_SEMANTIC_FAIL_NUMBER_TOKEN:
            g_payload_semantic_invalid_number_token++;
            break;
        case PAYLOAD_SEMANTIC_FAIL_BOOLEAN_TOKEN:
            g_payload_semantic_invalid_boolean_token++;
            break;
        case PAYLOAD_SEMANTIC_FAIL_NULL_TOKEN:
            g_payload_semantic_invalid_null_token++;
            break;
        case PAYLOAD_SEMANTIC_FAIL_OBJECT_JSON:
            g_payload_semantic_invalid_object_json++;
            break;
        case PAYLOAD_SEMANTIC_FAIL_ARRAY_JSON:
            g_payload_semantic_invalid_array_json++;
            break;
        default:
            break;
    }
}

static void payload_note_semantic_failure(uint32_t reason,
                                          uint32_t operation_id,
                                          const void* self,
                                          size_t entry_index,
                                          uint8_t entry_kind,
                                          uint16_t key_off,
                                          uint16_t key_len,
                                          uint16_t val_off,
                                          uint16_t val_len,
                                          const char* key) {
    g_payload_semantic_validation_fail++;
    payload_increment_semantic_reason(reason);

    if (g_payload_first_semantic_fail_captured != 0U) return;
    g_payload_first_semantic_fail_captured = 1U;
    g_payload_first_semantic_fail_reason = reason;
    g_payload_first_semantic_fail_op_id = operation_id;
    g_payload_first_semantic_fail_this = (uint32_t)(uintptr_t)self;
    g_payload_first_semantic_fail_entry_index = (uint32_t)entry_index;
    g_payload_first_semantic_fail_entry_kind = entry_kind;
    g_payload_first_semantic_fail_key_off = key_off;
    g_payload_first_semantic_fail_key_len = key_len;
    g_payload_first_semantic_fail_val_off = val_off;
    g_payload_first_semantic_fail_val_len = val_len;
    payload_copy_span_label(g_payload_first_semantic_fail_key,
                            sizeof(g_payload_first_semantic_fail_key),
                            key,
                            key_len);
}

// Flight recorder (defined with the execution-context census below).  The
// note path is scalar-only and allocator-free, so failure bookkeeping may
// call it directly.
static void payload_flight_note(uint32_t op_id, const void* self, uint16_t flags);

static inline void payload_note_error(uint32_t code,
                                      uint32_t operation_id,
                                      const void* self) {
    // Failure bookkeeping is scalar-only. operation_id is already numeric;
    // this path performs no string traversal or pointer inspection.
    g_payload_last_error_code = code;
    g_payload_last_error_count++;
    g_payload_last_error_this = (uint32_t)(uintptr_t)self;
    g_payload_last_error_op_id = operation_id;
    g_payload_last_error_op[0] = '\0';
    payload_flight_note(operation_id, self, PAYLOAD_FLIGHT_FLAG_ERROR);
}

static void payload_increment_integrity_reason(uint32_t reason) {
    switch (reason) {
        case PAYLOAD_SELF_OK_MAGIC_BAD: g_payload_self_ok_magic_bad++; break;
        case PAYLOAD_SELF_OK_ENTRIES_NULL: g_payload_self_ok_entries_null++; break;
        case PAYLOAD_SELF_OK_ENTRY_CAP_LOW: g_payload_self_ok_entry_cap_low++; break;
        case PAYLOAD_SELF_OK_ENTRY_CAP_HIGH: g_payload_self_ok_entry_cap_high++; break;
        case PAYLOAD_SELF_OK_ENTRIES_MAGIC_ADDRESS: g_payload_self_ok_entries_magic_address++; break;
        case PAYLOAD_SELF_OK_ARENA_MAGIC_ADDRESS: g_payload_self_ok_arena_magic_address++; break;
        case PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE: g_payload_self_ok_entries_span_unreadable++; break;
        case PAYLOAD_SELF_OK_ARENA_SPAN_UNREADABLE: g_payload_self_ok_arena_span_unreadable++; break;
        case PAYLOAD_SELF_OK_INLINE_CAP_MISMATCH: g_payload_self_ok_inline_cap_mismatch++; break;
        case PAYLOAD_SELF_OK_HEAP_CAP_MISMATCH: g_payload_self_ok_heap_cap_mismatch++; break;
        case PAYLOAD_SELF_OK_COUNT_GT_ENTRY_CAP: g_payload_self_ok_count_gt_entry_cap++; break;
        case PAYLOAD_SELF_OK_COUNT_GT_MAX: g_payload_self_ok_count_gt_max++; break;
        case PAYLOAD_SELF_OK_ARENA_USED_GT_CAP: g_payload_self_ok_arena_used_gt_cap++; break;
        case PAYLOAD_SELF_OK_ARENA_CAP_GT_MAX: g_payload_self_ok_arena_cap_gt_max++; break;
        case PAYLOAD_SELF_OK_ARENA_CAP_ZERO_WITH_PTR: g_payload_self_ok_arena_cap_zero_with_ptr++; break;
        case PAYLOAD_SELF_OK_ARENA_CAP_NONZERO_WITH_NULL: g_payload_self_ok_arena_cap_nonzero_with_null++; break;
        case PAYLOAD_SELF_OK_COUNT_WITHOUT_ARENA: g_payload_self_ok_count_without_arena++; break;
        case PAYLOAD_SELF_OK_ENTRY_KIND_BAD: g_payload_self_ok_entry_kind_bad++; break;
        case PAYLOAD_SELF_OK_ENTRY_KEY_OFF_OOB: g_payload_self_ok_entry_key_off_oob++; break;
        case PAYLOAD_SELF_OK_ENTRY_VAL_OFF_OOB: g_payload_self_ok_entry_val_off_oob++; break;
        case PAYLOAD_SELF_OK_ENTRY_VAL_END_OOB: g_payload_self_ok_entry_val_end_oob++; break;
        case PAYLOAD_SELF_OK_ENTRY_VAL_UNTERMINATED: g_payload_self_ok_entry_val_unterminated++; break;
        case PAYLOAD_SELF_OK_ENTRY_KEY_UNTERMINATED: g_payload_self_ok_entry_key_unterminated++; break;
        default: break;
    }
}

static void payload_note_integrity(
    uint32_t reason,
    uint32_t operation_id,
    const void* self,
    const payload_layout_evidence_t* evidence = nullptr) {
    // Integrity reporting must remain smaller than the failure it reports.
    // Capture only scalar values already in hand.  Never traverse strings,
    // allocate, emit logs, or dereference a questionable pointer here.
    g_payload_integrity_fail++;
    g_payload_self_ok_fail++;
    payload_increment_integrity_reason(reason);
    g_payload_last_self_ok_fail_reason = reason;
    g_payload_last_self_ok_fail_this = (uint32_t)(uintptr_t)self;
    g_payload_last_self_ok_fail_op_id = operation_id;
    g_payload_last_error_code = PAYLOAD_ERR_INTEGRITY;
    g_payload_last_error_count++;
    g_payload_last_error_this = (uint32_t)(uintptr_t)self;
    g_payload_last_error_op_id = g_payload_last_self_ok_fail_op_id;
    payload_flight_note(operation_id, self,
                        PAYLOAD_FLIGHT_FLAG_ERROR | PAYLOAD_FLIGHT_FLAG_INTEGRITY);

    if (g_payload_first_self_ok_fail_captured == 0U) {
        g_payload_first_self_ok_fail_captured = 1U;
        if (evidence) {
            g_payload_last_self_ok_fail_count = evidence->entry_count;
            g_payload_last_self_ok_fail_entry_cap = evidence->capacity;
            g_payload_last_self_ok_fail_arena_used =
                evidence->capacity >= evidence->data_begin
                    ? evidence->capacity - evidence->data_begin
                    : 0U;
            g_payload_last_self_ok_fail_arena_cap = evidence->capacity;
            g_payload_last_self_ok_fail_entry_index = evidence->entry_index;
            g_payload_last_self_ok_fail_entry_key_off = evidence->key_off;
            g_payload_last_self_ok_fail_entry_val_off = evidence->val_off;
            g_payload_last_self_ok_fail_entry_val_len = evidence->val_len;
            g_payload_last_self_ok_fail_entry_kind = evidence->entry_kind;
            g_payload_last_self_ok_fail_capacity = evidence->capacity;
            g_payload_last_self_ok_fail_data_begin = evidence->data_begin;
            g_payload_last_self_ok_fail_expected_upper =
                evidence->expected_upper;
            g_payload_last_self_ok_fail_key_end = evidence->key_end;
            g_payload_last_self_ok_fail_val_end = evidence->val_end;
        }
    }
}


static void payload_mark_constructed() {
    g_payload_instances_constructed++;
    g_payload_alive_now++;
    if (g_payload_alive_now > g_payload_alive_high_water) {
        g_payload_alive_high_water = g_payload_alive_now;
    }
}

static void payload_mark_destroyed() {
    g_payload_instances_destroyed++;
    if (g_payload_alive_now != 0U) g_payload_alive_now--;
}

static void payload_note_heap_delta(int32_t delta) {
    if (delta >= 0) {
        g_payload_arena_heap_bytes_alive += (uint32_t)delta;
    } else {
        const uint32_t amount = (uint32_t)(-delta);
        g_payload_arena_heap_bytes_alive =
            g_payload_arena_heap_bytes_alive >= amount
                ? g_payload_arena_heap_bytes_alive - amount
                : 0U;
    }
    if (g_payload_arena_heap_bytes_alive >
        g_payload_arena_heap_bytes_high_water) {
        g_payload_arena_heap_bytes_high_water =
            g_payload_arena_heap_bytes_alive;
    }
}

// ============================================================================
// Execution-context census + retained flight recorder
// ============================================================================
//
// Crash1 evidence implicated Payload guard-code working state, written from
// handler context, adjacent to (or over) a stacked exception frame.  The
// facilities below exist to *attribute*, not to prevent:
//
//   • The context census counts lifecycle/mutation/allocator activity that
//     executes while IPSR != 0 and latches the most recent handler-context
//     actor (IPSR, op, this, MSP, DWT).
//   • The allocator overlap tripwire detects a Payload allocator call that
//     preempted another in-flight Payload allocator call — the one shared
//     mutable resource (the newlib heap) that Payload's own arithmetic
//     cannot defend.  Detection only; behavior is unchanged.
//   • The flight recorder keeps a ring of recent lifecycle/mutation/failure
//     records in RAM2 (NOLOAD) so the final Payload operations before a
//     crash survive the reboot; each write is flushed through the data
//     cache so the physical RAM2 image is current when a reset arrives.
//
// Everything on these paths is scalar, allocator-free, and safe in handler
// context.  Counters are best-effort under preemption: forensics, never
// authoritative state.

static inline uint32_t payload_read_ipsr(void) {
#if defined(__IMXRT1062__)
    uint32_t ipsr;
    __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
    return ipsr & 0x1FFU;
#else
    return 0U;
#endif
}

static inline uint32_t payload_read_msp(void) {
#if defined(__IMXRT1062__)
    uint32_t msp;
    __asm__ volatile("mrs %0, msp" : "=r"(msp));
    return msp;
#else
    return 0U;
#endif
}

static inline uint32_t payload_read_dwt(void) {
#if defined(__IMXRT1062__)
    return ARM_DWT_CYCCNT;
#else
    return 0U;
#endif
}

static inline void payload_retained_flush(volatile void* addr, uint32_t size) {
#if defined(__IMXRT1062__)
    // RAM2 is write-back cached; a reset can discard dirty lines.  Flush so
    // the retained image in physical OCRAM matches what was recorded.
    arm_dcache_flush((void*)addr, size);
#else
    (void)addr;
    (void)size;
#endif
}

// ---- Execution-context census -------------------------------------------

static volatile uint32_t g_payload_handler_ctx_ctor = 0;
static volatile uint32_t g_payload_handler_ctx_mutate = 0;
static volatile uint32_t g_payload_handler_ctx_alloc = 0;
static volatile uint32_t g_payload_handler_ctx_free = 0;
static volatile uint32_t g_payload_last_handler_ctx_ipsr = 0;
static volatile uint32_t g_payload_last_handler_ctx_op_id = 0;
static volatile uint32_t g_payload_last_handler_ctx_this = 0;
static volatile uint32_t g_payload_last_handler_ctx_dwt = 0;
static volatile uint32_t g_payload_last_handler_ctx_msp = 0;

static void payload_note_handler_context(uint32_t op_id,
                                         const void* self,
                                         volatile uint32_t* counter) {
    const uint32_t ipsr = payload_read_ipsr();
    if (ipsr == 0U) return;
    (*counter)++;
    g_payload_last_handler_ctx_ipsr = ipsr;
    g_payload_last_handler_ctx_op_id = op_id;
    g_payload_last_handler_ctx_this = (uint32_t)(uintptr_t)self;
    g_payload_last_handler_ctx_dwt = payload_read_dwt();
    g_payload_last_handler_ctx_msp = payload_read_msp();
}

// ---- Allocator preemption-overlap tripwire --------------------------------

static volatile uint32_t g_payload_alloc_depth = 0;
static volatile uint32_t g_payload_alloc_overlap_detected = 0;
static volatile uint32_t g_payload_alloc_overlap_ipsr = 0;
static volatile uint32_t g_payload_alloc_overlap_op_id = 0;
static volatile uint32_t g_payload_alloc_overlap_this = 0;
static volatile uint32_t g_payload_alloc_overlap_dwt = 0;
static volatile uint32_t g_payload_alloc_overlap_depth = 0;

static void payload_note_alloc_overlap(uint32_t op_id, const void* self) {
    const uint32_t depth = g_payload_alloc_depth;
    if (depth == 0U) return;
    // A Payload allocator call is already in flight on this single core, so
    // this call preempted it mid-heap-operation.  Record and proceed: the
    // heap may already be the victim, and the evidence must outlive it.
    g_payload_alloc_overlap_detected++;
    g_payload_alloc_overlap_ipsr = payload_read_ipsr();
    g_payload_alloc_overlap_op_id = op_id;
    g_payload_alloc_overlap_this = (uint32_t)(uintptr_t)self;
    g_payload_alloc_overlap_dwt = payload_read_dwt();
    g_payload_alloc_overlap_depth = depth;
}

static void* payload_guarded_malloc(size_t total,
                                    uint32_t op_id,
                                    const void* self) {
    payload_note_handler_context(op_id, self, &g_payload_handler_ctx_alloc);
    payload_note_alloc_overlap(op_id, self);
    g_payload_alloc_depth++;
    void* raw = malloc(total);
    g_payload_alloc_depth--;
    return raw;
}

static void payload_guarded_free(void* block,
                                 uint32_t op_id,
                                 const void* self) {
    payload_note_handler_context(op_id, self, &g_payload_handler_ctx_free);
    payload_note_alloc_overlap(op_id, self);
    g_payload_alloc_depth++;
    free(block);
    g_payload_alloc_depth--;
}

// ---- Retained flight recorder ---------------------------------------------

#if defined(__IMXRT1062__)
#define PAYLOAD_RETAINED_MEM DMAMEM
#else
#define PAYLOAD_RETAINED_MEM
#endif

// ---- Retained append transaction recorder --------------------------------
//
// This recorder is intentionally independent of the general Payload flight
// ring. The crash under investigation lost that ring, while the exact append
// arguments are the evidence needed to separate caller custody, growth, alias
// remapping, and final-copy failures.

static constexpr uint32_t PAYLOAD_APPEND_TRACE_MAGIC = 0x50413431UL;  // 'PA41'
static constexpr uint32_t PAYLOAD_APPEND_TRACE_SCHEMA_VERSION = 1U;

struct payload_append_trace_bank_t {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t schema_version;
    uint32_t capacity;
    payload_append_trace_entry_t entries[PAYLOAD_APPEND_TRACE_ENTRIES];
};

static payload_append_trace_bank_t g_payload_append_trace_live PAYLOAD_RETAINED_MEM;
static payload_append_trace_bank_t g_payload_append_trace_retained PAYLOAD_RETAINED_MEM;
static bool g_payload_append_trace_boot_latched = false;
static volatile uint32_t g_payload_append_trace_next_sequence = 0U;

static inline void payload_append_trace_dmb() {
#if defined(__arm__)
    __asm__ volatile("dmb" ::: "memory");
#endif
}

static bool payload_append_trace_bank_valid(
    const payload_append_trace_bank_t& bank) {
    return bank.magic == PAYLOAD_APPEND_TRACE_MAGIC &&
           (bank.magic ^ bank.magic_inv) == 0xFFFFFFFFUL &&
           bank.schema_version == PAYLOAD_APPEND_TRACE_SCHEMA_VERSION &&
           bank.capacity == PAYLOAD_APPEND_TRACE_ENTRIES;
}

static bool payload_append_trace_entry_valid(
    const payload_append_trace_entry_t& entry) {
    return entry.sequence != 0U &&
           (entry.sequence ^ entry.sequence_inv) == 0xFFFFFFFFUL;
}

static void payload_append_trace_initialize_live() {
    memset((void*)&g_payload_append_trace_live, 0,
           sizeof(g_payload_append_trace_live));
    g_payload_append_trace_live.schema_version =
        PAYLOAD_APPEND_TRACE_SCHEMA_VERSION;
    g_payload_append_trace_live.capacity = PAYLOAD_APPEND_TRACE_ENTRIES;
    g_payload_append_trace_live.magic_inv = ~PAYLOAD_APPEND_TRACE_MAGIC;
    g_payload_append_trace_live.magic = PAYLOAD_APPEND_TRACE_MAGIC;
    g_payload_append_trace_next_sequence = 0U;
    payload_retained_flush(&g_payload_append_trace_live,
                           sizeof(g_payload_append_trace_live));
}

static void payload_append_trace_boot_latch() {
    if (g_payload_append_trace_boot_latched) return;
    g_payload_append_trace_boot_latched = true;

    if (payload_append_trace_bank_valid(g_payload_append_trace_live)) {
        g_payload_append_trace_retained = g_payload_append_trace_live;
    } else {
        memset((void*)&g_payload_append_trace_retained, 0,
               sizeof(g_payload_append_trace_retained));
    }
    payload_retained_flush(&g_payload_append_trace_retained,
                           sizeof(g_payload_append_trace_retained));
    payload_append_trace_initialize_live();
}

static void payload_append_trace_record(
    payload_append_trace_stage_t stage,
    const void* self,
    const char* key,
    size_t key_len,
    const char* value,
    size_t value_len,
    uint32_t kind,
    const void* storage,
    size_t capacity,
    size_t data_begin,
    size_t entry_count,
    bool key_alias,
    bool value_alias,
    size_t key_offset,
    size_t value_offset,
    int32_t data_shift,
    size_t key_off,
    size_t val_off) {
    payload_append_trace_boot_latch();

    uint32_t sequence = g_payload_append_trace_next_sequence + 1U;
    if (sequence == 0U) sequence = 1U;
    g_payload_append_trace_next_sequence = sequence;

    payload_append_trace_entry_t& entry =
        g_payload_append_trace_live.entries[
            (sequence - 1U) % PAYLOAD_APPEND_TRACE_ENTRIES];

    entry.sequence = 0U;
    entry.sequence_inv = 0U;
    payload_append_trace_dmb();
    payload_retained_flush(&entry, 2U * sizeof(uint32_t));

    entry.stage = (uint32_t)stage;
    entry.this_ptr = (uint32_t)(uintptr_t)self;
    entry.key_ptr = (uint32_t)(uintptr_t)key;
    entry.value_ptr = (uint32_t)(uintptr_t)value;
    entry.key_len = (uint32_t)key_len;
    entry.value_len = (uint32_t)value_len;
    entry.kind = kind;
    entry.storage_ptr = (uint32_t)(uintptr_t)storage;
    entry.capacity = (uint32_t)capacity;
    entry.data_begin = (uint32_t)data_begin;
    entry.entry_count = (uint32_t)entry_count;
    entry.alias_flags = (key_alias ? 1U : 0U) |
                        (value_alias ? 2U : 0U);
    entry.key_offset = (uint32_t)key_offset;
    entry.value_offset = (uint32_t)value_offset;
    entry.data_shift = data_shift;
    entry.key_off = (uint32_t)key_off;
    entry.val_off = (uint32_t)val_off;
    entry.dwt_cyccnt = payload_read_dwt();
    entry.ipsr = payload_read_ipsr();

    entry.sequence_inv = ~sequence;
    payload_append_trace_dmb();
    entry.sequence = sequence;
    payload_append_trace_dmb();
    payload_retained_flush(&entry, sizeof(entry));
}

static void payload_append_trace_snapshot_bank(
    const payload_append_trace_bank_t& bank,
    payload_append_trace_bank_snapshot_t* out) {
    memset(out, 0, sizeof(*out));
    if (!payload_append_trace_bank_valid(bank)) return;
    out->valid = 1U;

    for (uint32_t i = 0U; i < PAYLOAD_APPEND_TRACE_ENTRIES; ++i) {
        const volatile payload_append_trace_entry_t* source =
            &bank.entries[i];
        const uint32_t sequence_before = source->sequence;
        const uint32_t sequence_inv_before = source->sequence_inv;
        if (sequence_before == 0U ||
            (sequence_before ^ sequence_inv_before) != 0xFFFFFFFFUL) {
            continue;
        }

        const payload_append_trace_entry_t candidate = bank.entries[i];
        payload_append_trace_dmb();
        if (source->sequence != sequence_before ||
            source->sequence_inv != sequence_inv_before ||
            !payload_append_trace_entry_valid(candidate)) {
            continue;
        }

        uint32_t pos = out->count;
        while (pos > 0U &&
               out->entries[pos - 1U].sequence > candidate.sequence) {
            out->entries[pos] = out->entries[pos - 1U];
            --pos;
        }
        out->entries[pos] = candidate;
        ++out->count;
    }
    if (out->count != 0U) {
        out->newest_sequence = out->entries[out->count - 1U].sequence;
    }
}

void payload_get_append_trace(payload_append_trace_snapshot_t* out) {
    if (!out) return;
    payload_append_trace_boot_latch();
    memset(out, 0, sizeof(*out));
    payload_append_trace_snapshot_bank(g_payload_append_trace_live,
                                       &out->live);
    payload_append_trace_snapshot_bank(g_payload_append_trace_retained,
                                       &out->retained);
}

void payload_clear_retained_append_trace() {
    memset((void*)&g_payload_append_trace_retained, 0,
           sizeof(g_payload_append_trace_retained));
    payload_retained_flush(&g_payload_append_trace_retained,
                           sizeof(g_payload_append_trace_retained));
}

static constexpr uint32_t PAYLOAD_FLIGHT_MAGIC = 0x464C5952UL;  // 'FLYR'

struct payload_flight_ring_t {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t sequence;  // total records noted since ring initialization
    payload_flight_entry_t entries[PAYLOAD_FLIGHT_ENTRIES];
};

// NOLOAD placement: startup does not zero these, and nothing else may.  The
// live ring's survival across a reboot is the entire point.
static payload_flight_ring_t g_payload_flight_live PAYLOAD_RETAINED_MEM;
static payload_flight_ring_t g_payload_flight_retained PAYLOAD_RETAINED_MEM;

// Ordinary BSS: guaranteed zero at every boot, so the latch below runs
// exactly once per boot regardless of RAM2 contents.
static bool g_payload_flight_boot_latched = false;

static bool payload_flight_ring_valid(const payload_flight_ring_t* ring) {
    return ring->magic == PAYLOAD_FLIGHT_MAGIC &&
           (ring->magic ^ ring->magic_inv) == 0xFFFFFFFFUL;
}

static void payload_flight_boot_latch(void) {
    if (g_payload_flight_boot_latched) return;
    g_payload_flight_boot_latched = true;

    // RAM2 is NOLOAD, so at this moment the live ring still holds the
    // previous boot's final records.  Preserve them in the retained bank
    // before this boot's first record overwrites the ring.  Magic+complement
    // rejects power-on garbage.  Triggered lazily by the first Payload
    // activity of the boot; a preemption race here can only repeat the same
    // copy, never lose data.
    if (payload_flight_ring_valid(&g_payload_flight_live)) {
        g_payload_flight_retained = g_payload_flight_live;
    } else {
        memset((void*)&g_payload_flight_retained, 0,
               sizeof(g_payload_flight_retained));
    }
    payload_retained_flush(&g_payload_flight_retained,
                           sizeof(g_payload_flight_retained));

    memset((void*)g_payload_flight_live.entries, 0,
           sizeof(g_payload_flight_live.entries));
    g_payload_flight_live.sequence = 0U;
    g_payload_flight_live.magic_inv = ~PAYLOAD_FLIGHT_MAGIC;
    g_payload_flight_live.magic = PAYLOAD_FLIGHT_MAGIC;
    payload_retained_flush(&g_payload_flight_live,
                           sizeof(g_payload_flight_live));
}

static void payload_flight_note(uint32_t op_id,
                                const void* self,
                                uint16_t flags) {
    payload_flight_boot_latch();

    // Lock-free best-effort: a preempting record may interleave with this
    // one.  The recorder trades perfect ordering for zero blocking.
    const uint32_t seq = g_payload_flight_live.sequence++;
    payload_flight_entry_t* e =
        &g_payload_flight_live.entries[seq % PAYLOAD_FLIGHT_ENTRIES];
    e->op_id = op_id;
    e->this_ptr = (uint32_t)(uintptr_t)self;
    e->dwt_cyccnt = payload_read_dwt();
    e->ipsr = (uint16_t)payload_read_ipsr();
    e->flags = flags;

    // Flush the entry and the header (sequence) lines so the physical RAM2
    // image is current if a reset arrives before natural eviction.
    payload_retained_flush(e, sizeof(*e));
    payload_retained_flush(&g_payload_flight_live,
                           3U * sizeof(uint32_t));
}

void payload_get_flight_info(payload_flight_info_t* out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));

    // Ensure the retained bank is populated even if this is queried before
    // any Payload activity has occurred this boot.
    payload_flight_boot_latch();

    if (payload_flight_ring_valid(&g_payload_flight_live)) {
        out->live_valid = 1U;
        const uint32_t seq = g_payload_flight_live.sequence;
        out->live_sequence = seq;
        const uint32_t count =
            seq < PAYLOAD_FLIGHT_ENTRIES ? seq : PAYLOAD_FLIGHT_ENTRIES;
        out->live_count = count;
        for (uint32_t i = 0; i < count; ++i) {
            const uint32_t idx = (seq - count + i) % PAYLOAD_FLIGHT_ENTRIES;
            out->live[i] = g_payload_flight_live.entries[idx];
        }
    }

    if (payload_flight_ring_valid(&g_payload_flight_retained)) {
        out->retained_valid = 1U;
        const uint32_t seq = g_payload_flight_retained.sequence;
        out->retained_sequence = seq;
        const uint32_t count =
            seq < PAYLOAD_FLIGHT_ENTRIES ? seq : PAYLOAD_FLIGHT_ENTRIES;
        out->retained_count = count;
        for (uint32_t i = 0; i < count; ++i) {
            const uint32_t idx = (seq - count + i) % PAYLOAD_FLIGHT_ENTRIES;
            out->retained[i] = g_payload_flight_retained.entries[idx];
        }
    }
}

// ============================================================================
// Pointer custody
// ============================================================================

#if defined(__IMXRT1062__)
extern "C" unsigned long _estack;
#endif

struct payload_readable_range_t {
    uintptr_t begin;
    uintptr_t end;
};

static bool payload_pointer_remaining(const void* ptr,
                                      size_t* out_remaining,
                                      uint32_t* out_reason) {
    if (out_remaining) *out_remaining = 0;
    if (out_reason) *out_reason = PAYLOAD_STRING_FAULT_NONE;

    if (!ptr) {
        if (out_reason) *out_reason = PAYLOAD_STRING_FAULT_NULL;
        return false;
    }

    const uintptr_t addr = (uintptr_t)ptr;
    if (addr == 0x5041594CUL || addr == 0x44454144UL) {
        if (out_reason) *out_reason = PAYLOAD_STRING_FAULT_MAGIC_ADDRESS;
        return false;
    }
    if (addr < 0x1000UL) {
        if (out_reason) *out_reason = PAYLOAD_STRING_FAULT_LOW_ADDRESS;
        return false;
    }

#if defined(__IMXRT1062__)

    const uintptr_t dtcm_end = (uintptr_t)&_estack;
    const payload_readable_range_t ranges[] = {
        // Deliberately reject ITCM/code pointers.  Normal string storage in the
        // Teensy build lives in DTCM, OCRAM, or the QSPI flash map; accepting
        // executable bytes would let a poisoned code address masquerade as a
        // bounded C string.
        {0x20000000UL, dtcm_end},      // actual DTCM ceiling for this image
        {0x20200000UL, 0x20280000UL},  // OCRAM / malloc heap
        {0x60000000UL, 0x61000000UL},  // QSPI flash execution map
    };

    for (const auto& r : ranges) {
        if (r.end > r.begin && addr >= r.begin && addr < r.end) {
            if (out_remaining) *out_remaining = (size_t)(r.end - addr);
            return true;
        }
    }

    if (out_reason) *out_reason =
        addr < 0x1000UL ? PAYLOAD_STRING_FAULT_LOW_ADDRESS
                        : PAYLOAD_STRING_FAULT_OUT_OF_RANGE;
    return false;
#else
    // Host/unit-test builds cannot infer process memory maps portably. Low
    // sentinel addresses are still rejected; all other scans remain bounded.
    if (out_remaining) *out_remaining = SIZE_MAX;
    return true;
#endif
}

static void payload_note_string_pointer_fault(uint32_t reason,
                                              const void* ptr,
                                              uint32_t operation_id) {
    g_payload_string_pointer_fault++;
    switch (reason) {
        case PAYLOAD_STRING_FAULT_NULL: g_payload_string_pointer_null++; break;
        case PAYLOAD_STRING_FAULT_LOW_ADDRESS: g_payload_string_pointer_low_address++; break;
        case PAYLOAD_STRING_FAULT_MAGIC_ADDRESS: g_payload_string_pointer_magic_address++; break;
        case PAYLOAD_STRING_FAULT_OUT_OF_RANGE: g_payload_string_pointer_out_of_range++; break;
        case PAYLOAD_STRING_FAULT_SPAN_OUT_OF_RANGE: g_payload_string_pointer_span_out_of_range++; break;
        case PAYLOAD_STRING_FAULT_UNTERMINATED: g_payload_string_pointer_unterminated++; break;
        case PAYLOAD_STRING_FAULT_TOO_LONG: g_payload_string_pointer_too_long++; break;
        default: break;
    }
    g_payload_last_string_pointer_fault_reason = reason;
    g_payload_last_string_pointer_fault_ptr = (uint32_t)(uintptr_t)ptr;
    g_payload_last_string_pointer_fault_op_id = operation_id;
    // Do not dereference or copy diagnostic labels from a pointer-fault path.
    // The numeric reason and offending pointer are the durable evidence.
    g_payload_last_string_pointer_fault_context[0] = '\0';
    g_payload_last_string_pointer_fault_key_ptr = 0;
    g_payload_last_string_pointer_fault_key[0] = '\0';
}

static bool payload_span_readable(const void* ptr,
                                  size_t len,
                                  uint32_t operation_id) {
    if (len == 0) return true;

    size_t remaining = 0;
    uint32_t reason = PAYLOAD_STRING_FAULT_NONE;
    if (!payload_pointer_remaining(ptr, &remaining, &reason)) {
        payload_note_string_pointer_fault(reason, ptr, operation_id);
        return false;
    }
    if (len > remaining) {
        payload_note_string_pointer_fault(
            PAYLOAD_STRING_FAULT_SPAN_OUT_OF_RANGE, ptr, operation_id);
        return false;
    }
    return true;
}

static bool payload_cstr_len_checked(const char* str,
                                     size_t max_len,
                                     uint32_t operation_id,
                                     size_t* out_len,
                                     bool null_is_empty = false) {
    if (out_len) *out_len = 0;
    if (!str && null_is_empty) return true;

    size_t remaining = 0;
    uint32_t reason = PAYLOAD_STRING_FAULT_NONE;
    if (!payload_pointer_remaining(str, &remaining, &reason)) {
        payload_note_string_pointer_fault(reason, str, operation_id);
        return false;
    }

    size_t limit = max_len;
    if (limit > remaining) limit = remaining;
    for (size_t i = 0; i < limit; ++i) {
        if (str[i] == '\0') {
            if (out_len) *out_len = i;
            return true;
        }
    }

    payload_note_string_pointer_fault(
        limit < remaining ? PAYLOAD_STRING_FAULT_TOO_LONG
                          : PAYLOAD_STRING_FAULT_UNTERMINATED,
        str,
        operation_id);
    return false;
}

static void payload_note_bad_key(const char* key) {
    // Preserve only the address.  A bad-key breadcrumb must never inspect the
    // very pointer whose custody is in question.
    g_payload_last_string_pointer_fault_key_ptr = (uint32_t)(uintptr_t)key;
    g_payload_last_string_pointer_fault_key[0] = '\0';
}

// ============================================================================
// One-block heap ownership
// ============================================================================

// The allocation begins with four 32-bit words followed by the usable bytes.
// Capacity and owner cookies are stored with their complements.  The owner
// cookie binds the block to the exact Payload/PayloadArray instance, preventing
// a corrupted pointer from accidentally validating against some other live
// Payload allocation.  Header access uses memcpy; no C++ object is fabricated
// inside raw malloc storage.
static constexpr size_t PAYLOAD_HEAP_HEADER_SIZE = 16U;
static constexpr uintptr_t PAYLOAD_HEAP_ALIGNMENT = 8U;
static constexpr uint32_t PAYLOAD_HEAP_OWNER_SALT = 0xA6F19D3BUL;

static uint32_t payload_heap_owner_cookie(const void* owner) {
    const uintptr_t address = (uintptr_t)owner;
    uint32_t folded = (uint32_t)address;
#if UINTPTR_MAX > UINT32_MAX
    folded ^= (uint32_t)(address >> 32);
#endif
    return folded ^ PAYLOAD_HEAP_OWNER_SALT;
}

static uint8_t* payload_heap_bytes(void* raw) {
    return raw
        ? reinterpret_cast<uint8_t*>(raw) + PAYLOAD_HEAP_HEADER_SIZE
        : nullptr;
}

static const uint8_t* payload_heap_bytes(const void* raw) {
    return raw
        ? reinterpret_cast<const uint8_t*>(raw) + PAYLOAD_HEAP_HEADER_SIZE
        : nullptr;
}

static bool payload_heap_header_span_readable(const void* block) {
    if (!block) return false;
    const uintptr_t addr = (uintptr_t)block;
    if ((addr % PAYLOAD_HEAP_ALIGNMENT) != 0U) return false;
#if defined(__IMXRT1062__)
    return addr >= 0x20200000UL &&
           addr <= 0x20280000UL - PAYLOAD_HEAP_HEADER_SIZE;
#else
    return addr >= 0x1000UL;
#endif
}

static bool payload_heap_read_capacity(const void* raw,
                                       const void* expected_owner,
                                       size_t minimum,
                                       size_t maximum,
                                       uint32_t* out_capacity) {
    if (out_capacity) *out_capacity = 0U;
    if (!raw || !expected_owner || !payload_heap_header_span_readable(raw)) {
        return false;
    }

    uint32_t header[4] = {0U, 0U, 0U, 0U};
    memcpy(header, raw, sizeof(header));
    const uint32_t capacity = header[0];
    const uint32_t owner_cookie = payload_heap_owner_cookie(expected_owner);
    if ((capacity ^ header[1]) != 0xFFFFFFFFUL ||
        (header[2] ^ header[3]) != 0xFFFFFFFFUL ||
        header[2] != owner_cookie) {
        return false;
    }
    if (capacity < minimum || capacity > maximum) return false;
#if defined(__IMXRT1062__)
    if (!payload_span_readable(payload_heap_bytes(raw), capacity, PAYLOAD_OP_HEAP_BLOCK)) {
        return false;
    }
#endif
    if (out_capacity) *out_capacity = capacity;
    return true;
}

static void* payload_heap_allocate(size_t capacity, const void* owner) {
    if (!owner || capacity > UINT32_MAX - PAYLOAD_HEAP_HEADER_SIZE) {
        return nullptr;
    }
    const size_t total = PAYLOAD_HEAP_HEADER_SIZE + capacity;
    void* raw = payload_guarded_malloc(total, PAYLOAD_OP_HEAP_BLOCK, owner);
    if (!raw) return nullptr;
    if (((uintptr_t)raw % PAYLOAD_HEAP_ALIGNMENT) != 0U) {
        payload_guarded_free(raw, PAYLOAD_OP_HEAP_BLOCK, owner);
        return nullptr;
    }

    const uint32_t cap32 = (uint32_t)capacity;
    const uint32_t owner_cookie = payload_heap_owner_cookie(owner);
    const uint32_t header[4] = {
        cap32,
        ~cap32,
        owner_cookie,
        ~owner_cookie,
    };
    memcpy(raw, header, sizeof(header));
    return raw;
}

static size_t payload_heap_capacity(const void* raw,
                                    const void* expected_owner,
                                    size_t minimum,
                                    size_t maximum) {
    uint32_t capacity = 0U;
    return payload_heap_read_capacity(raw,
                                      expected_owner,
                                      minimum,
                                      maximum,
                                      &capacity)
        ? (size_t)capacity
        : 0U;
}

static bool payload_heap_rebind_owner(void* raw,
                                      const void* old_owner,
                                      const void* new_owner,
                                      size_t minimum,
                                      size_t maximum) {
    uint32_t capacity = 0U;
    if (!payload_heap_read_capacity(raw,
                                    old_owner,
                                    minimum,
                                    maximum,
                                    &capacity)) {
        return false;
    }
    (void)capacity;
    const uint32_t owner_cookie = payload_heap_owner_cookie(new_owner);
    const uint32_t owner_words[2] = {owner_cookie, ~owner_cookie};
    memcpy(reinterpret_cast<uint8_t*>(raw) + 2U * sizeof(uint32_t),
           owner_words,
           sizeof(owner_words));
    return true;
}

static size_t payload_growth_capacity(size_t current,
                                      size_t required,
                                      size_t maximum) {
    size_t capacity = current;
    if (capacity < Payload::ARENA_INITIAL) capacity = Payload::ARENA_INITIAL;
    while (capacity < required && capacity < maximum) {
        const size_t next = capacity <= maximum / 2U ? capacity * 2U : maximum;
        if (next <= capacity) break;
        capacity = next;
    }
    if (capacity < required) capacity = maximum;
    return capacity >= required ? capacity : 0;
}

// ============================================================================
// JSON scanner / validator
// ============================================================================

static constexpr uint32_t PAYLOAD_JSON_MAX_DEPTH = 32U;

enum class json_value_type_t : uint8_t {
    INVALID = 0,
    STRING,
    NUMBER,
    BOOLEAN,
    NIL,
    OBJECT,
    ARRAY,
};

struct json_value_span_t {
    json_value_type_t type = json_value_type_t::INVALID;
    size_t begin = 0;
    size_t end = 0;             // one past final token byte, excluding whitespace
    size_t decoded_length = 0;  // STRING only
};

struct json_cursor_t {
    const uint8_t* data;
    size_t len;
    size_t pos;
};

static void json_skip_ws(json_cursor_t& c) {
    while (c.pos < c.len) {
        const uint8_t ch = c.data[c.pos];
        if (ch != ' ' && ch != '\t' && ch != '\n' && ch != '\r') break;
        ++c.pos;
    }
}

static int json_hex_value(uint8_t c) {
    if (c >= '0' && c <= '9') return (int)(c - '0');
    if (c >= 'a' && c <= 'f') return (int)(c - 'a') + 10;
    if (c >= 'A' && c <= 'F') return (int)(c - 'A') + 10;
    return -1;
}

static bool json_read_hex4(const uint8_t* data,
                           size_t len,
                           size_t pos,
                           uint32_t* out) {
    if (!out || pos > len || len - pos < 4U) return false;
    uint32_t value = 0;
    for (size_t i = 0; i < 4U; ++i) {
        const int h = json_hex_value(data[pos + i]);
        if (h < 0) return false;
        value = (value << 4) | (uint32_t)h;
    }
    *out = value;
    return true;
}

static size_t json_utf8_encoded_length(uint32_t cp) {
    if (cp == 0U || cp > 0x10FFFFU || (cp >= 0xD800U && cp <= 0xDFFFU)) {
        return 0;
    }
    if (cp <= 0x7FU) return 1;
    if (cp <= 0x7FFU) return 2;
    if (cp <= 0xFFFFU) return 3;
    return 4;
}

static bool json_write_utf8(uint32_t cp, char* out, size_t* pos) {
    if (!out || !pos) return false;
    const size_t n = json_utf8_encoded_length(cp);
    if (n == 0) return false;
    size_t p = *pos;
    if (n == 1) {
        out[p++] = (char)cp;
    } else if (n == 2) {
        out[p++] = (char)(0xC0U | (cp >> 6));
        out[p++] = (char)(0x80U | (cp & 0x3FU));
    } else if (n == 3) {
        out[p++] = (char)(0xE0U | (cp >> 12));
        out[p++] = (char)(0x80U | ((cp >> 6) & 0x3FU));
        out[p++] = (char)(0x80U | (cp & 0x3FU));
    } else {
        out[p++] = (char)(0xF0U | (cp >> 18));
        out[p++] = (char)(0x80U | ((cp >> 12) & 0x3FU));
        out[p++] = (char)(0x80U | ((cp >> 6) & 0x3FU));
        out[p++] = (char)(0x80U | (cp & 0x3FU));
    }
    *pos = p;
    return true;
}

static bool json_validate_utf8_sequence(const uint8_t* data,
                                        size_t len,
                                        size_t pos,
                                        size_t* sequence_len) {
    if (!sequence_len || pos >= len) return false;
    const uint8_t b0 = data[pos];
    if (b0 < 0x80U) {
        *sequence_len = 1;
        return true;
    }

    size_t n = 0;
    uint32_t cp = 0;
    if (b0 >= 0xC2U && b0 <= 0xDFU) {
        n = 2;
        cp = b0 & 0x1FU;
    } else if (b0 >= 0xE0U && b0 <= 0xEFU) {
        n = 3;
        cp = b0 & 0x0FU;
    } else if (b0 >= 0xF0U && b0 <= 0xF4U) {
        n = 4;
        cp = b0 & 0x07U;
    } else {
        return false;
    }
    if (pos > len || len - pos < n) return false;

    for (size_t i = 1; i < n; ++i) {
        const uint8_t bx = data[pos + i];
        if ((bx & 0xC0U) != 0x80U) return false;
        cp = (cp << 6) | (uint32_t)(bx & 0x3FU);
    }

    if ((n == 3 && cp < 0x800U) ||
        (n == 4 && cp < 0x10000U) ||
        cp > 0x10FFFFU ||
        (cp >= 0xD800U && cp <= 0xDFFFU)) {
        return false;
    }
    *sequence_len = n;
    return true;
}

static bool json_scan_string(json_cursor_t& c,
                             size_t* out_begin,
                             size_t* out_end,
                             size_t* out_decoded_length) {
    if (c.pos >= c.len || c.data[c.pos] != '"') return false;
    const size_t begin = c.pos++;
    size_t decoded = 0;

    while (c.pos < c.len) {
        const uint8_t ch = c.data[c.pos++];
        if (ch == '"') {
            if (out_begin) *out_begin = begin;
            if (out_end) *out_end = c.pos;
            if (out_decoded_length) *out_decoded_length = decoded;
            return true;
        }
        if (ch < 0x20U) return false;
        if (ch == '\\') {
            if (c.pos >= c.len) return false;
            const uint8_t esc = c.data[c.pos++];
            switch (esc) {
                case '"': case '\\': case '/':
                case 'b': case 'f': case 'n': case 'r': case 't':
                    decoded++;
                    break;
                case 'u': {
                    uint32_t cp = 0;
                    if (!json_read_hex4(c.data, c.len, c.pos, &cp)) return false;
                    c.pos += 4U;
                    if (cp >= 0xD800U && cp <= 0xDBFFU) {
                        if (c.pos + 6U > c.len ||
                            c.data[c.pos] != '\\' ||
                            c.data[c.pos + 1U] != 'u') {
                            return false;
                        }
                        uint32_t low = 0;
                        if (!json_read_hex4(c.data, c.len, c.pos + 2U, &low) ||
                            low < 0xDC00U || low > 0xDFFFU) {
                            return false;
                        }
                        c.pos += 6U;
                        cp = 0x10000U + (((cp - 0xD800U) << 10) |
                                         (low - 0xDC00U));
                    } else if (cp >= 0xDC00U && cp <= 0xDFFFU) {
                        return false;
                    }
                    const size_t n = json_utf8_encoded_length(cp);
                    if (n == 0) return false;  // Embedded NUL is not C-string compatible.
                    decoded += n;
                    break;
                }
                default:
                    return false;
            }
            continue;
        }

        if (ch < 0x80U) {
            decoded++;
            continue;
        }

        size_t utf8_len = 0;
        const size_t sequence_start = c.pos - 1U;
        if (!json_validate_utf8_sequence(c.data, c.len,
                                         sequence_start, &utf8_len)) {
            return false;
        }
        c.pos = sequence_start + utf8_len;
        decoded += utf8_len;
    }
    return false;
}

static bool json_decode_string(const uint8_t* data,
                               size_t begin,
                               size_t end,
                               char* out,
                               size_t expected_length) {
    if (!data || !out || end <= begin + 1U || data[begin] != '"' ||
        data[end - 1U] != '"') {
        return false;
    }

    size_t i = begin + 1U;
    size_t p = 0;
    while (i < end - 1U) {
        uint8_t ch = data[i++];
        if (ch != '\\') {
            if (ch < 0x80U) {
                out[p++] = (char)ch;
            } else {
                size_t n = 0;
                if (!json_validate_utf8_sequence(data, end - 1U, i - 1U, &n)) {
                    return false;
                }
                memcpy(out + p, data + i - 1U, n);
                p += n;
                i = i - 1U + n;
            }
            continue;
        }

        if (i >= end - 1U) return false;
        const uint8_t esc = data[i++];
        switch (esc) {
            case '"': out[p++] = '"'; break;
            case '\\': out[p++] = '\\'; break;
            case '/': out[p++] = '/'; break;
            case 'b': out[p++] = '\b'; break;
            case 'f': out[p++] = '\f'; break;
            case 'n': out[p++] = '\n'; break;
            case 'r': out[p++] = '\r'; break;
            case 't': out[p++] = '\t'; break;
            case 'u': {
                uint32_t cp = 0;
                if (!json_read_hex4(data, end - 1U, i, &cp)) return false;
                i += 4U;
                if (cp >= 0xD800U && cp <= 0xDBFFU) {
                    if (i + 6U > end - 1U || data[i] != '\\' || data[i + 1U] != 'u') {
                        return false;
                    }
                    uint32_t low = 0;
                    if (!json_read_hex4(data, end - 1U, i + 2U, &low) ||
                        low < 0xDC00U || low > 0xDFFFU) {
                        return false;
                    }
                    i += 6U;
                    cp = 0x10000U + (((cp - 0xD800U) << 10) |
                                     (low - 0xDC00U));
                }
                if (!json_write_utf8(cp, out, &p)) return false;
                break;
            }
            default:
                return false;
        }
    }
    if (p != expected_length) return false;
    out[p] = '\0';
    return true;
}

static bool json_scan_number(json_cursor_t& c) {
    const size_t start = c.pos;
    if (c.pos < c.len && c.data[c.pos] == '-') ++c.pos;
    if (c.pos >= c.len) return false;

    if (c.data[c.pos] == '0') {
        ++c.pos;
        if (c.pos < c.len && isdigit((unsigned char)c.data[c.pos])) return false;
    } else if (c.data[c.pos] >= '1' && c.data[c.pos] <= '9') {
        do { ++c.pos; }
        while (c.pos < c.len && isdigit((unsigned char)c.data[c.pos]));
    } else {
        return false;
    }

    if (c.pos < c.len && c.data[c.pos] == '.') {
        ++c.pos;
        if (c.pos >= c.len || !isdigit((unsigned char)c.data[c.pos])) return false;
        do { ++c.pos; }
        while (c.pos < c.len && isdigit((unsigned char)c.data[c.pos]));
    }

    if (c.pos < c.len && (c.data[c.pos] == 'e' || c.data[c.pos] == 'E')) {
        ++c.pos;
        if (c.pos < c.len && (c.data[c.pos] == '+' || c.data[c.pos] == '-')) {
            ++c.pos;
        }
        if (c.pos >= c.len || !isdigit((unsigned char)c.data[c.pos])) return false;
        do { ++c.pos; }
        while (c.pos < c.len && isdigit((unsigned char)c.data[c.pos]));
    }
    return c.pos > start;
}

static bool json_match_literal(json_cursor_t& c, const char* literal) {
    size_t n = 0;
    while (literal[n] != '\0') ++n;
    if (c.pos > c.len || c.len - c.pos < n) return false;
    if (memcmp(c.data + c.pos, literal, n) != 0) return false;
    c.pos += n;
    return true;
}

static bool json_scan_value(json_cursor_t& c,
                            uint32_t depth,
                            json_value_span_t* out);

static bool json_scan_object(json_cursor_t& c, uint32_t depth) {
    if (depth > PAYLOAD_JSON_MAX_DEPTH) {
        g_payload_json_invalid_depth++;
        return false;
    }
    if (c.pos >= c.len || c.data[c.pos] != '{') {
        return false;
    }
    ++c.pos;
    json_skip_ws(c);
    if (c.pos < c.len && c.data[c.pos] == '}') {
        ++c.pos;
        return true;
    }

    while (c.pos < c.len) {
        if (!json_scan_string(c, nullptr, nullptr, nullptr)) return false;
        json_skip_ws(c);
        if (c.pos >= c.len || c.data[c.pos] != ':') return false;
        ++c.pos;
        json_value_span_t ignored;
        if (!json_scan_value(c, depth + 1U, &ignored)) return false;
        json_skip_ws(c);
        if (c.pos >= c.len) return false;
        if (c.data[c.pos] == '}') {
            ++c.pos;
            return true;
        }
        if (c.data[c.pos] != ',') return false;
        ++c.pos;
        json_skip_ws(c);
    }
    return false;
}

static bool json_scan_array(json_cursor_t& c, uint32_t depth) {
    if (depth > PAYLOAD_JSON_MAX_DEPTH) {
        g_payload_json_invalid_depth++;
        return false;
    }
    if (c.pos >= c.len || c.data[c.pos] != '[') {
        return false;
    }
    ++c.pos;
    json_skip_ws(c);
    if (c.pos < c.len && c.data[c.pos] == ']') {
        ++c.pos;
        return true;
    }

    while (c.pos < c.len) {
        json_value_span_t ignored;
        if (!json_scan_value(c, depth + 1U, &ignored)) return false;
        json_skip_ws(c);
        if (c.pos >= c.len) return false;
        if (c.data[c.pos] == ']') {
            ++c.pos;
            return true;
        }
        if (c.data[c.pos] != ',') return false;
        ++c.pos;
        json_skip_ws(c);
    }
    return false;
}

static bool json_scan_value(json_cursor_t& c,
                            uint32_t depth,
                            json_value_span_t* out) {
    if (!out) return false;
    if (depth > PAYLOAD_JSON_MAX_DEPTH) {
        g_payload_json_invalid_depth++;
        return false;
    }
    json_skip_ws(c);
    if (c.pos >= c.len) return false;

    json_value_span_t span;
    span.begin = c.pos;
    const uint8_t first = c.data[c.pos];
    if (first == '"') {
        span.type = json_value_type_t::STRING;
        if (!json_scan_string(c, nullptr, nullptr, &span.decoded_length)) return false;
    } else if (first == '{') {
        span.type = json_value_type_t::OBJECT;
        if (!json_scan_object(c, depth)) return false;
    } else if (first == '[') {
        span.type = json_value_type_t::ARRAY;
        if (!json_scan_array(c, depth)) return false;
    } else if (first == 't') {
        span.type = json_value_type_t::BOOLEAN;
        if (!json_match_literal(c, "true")) return false;
    } else if (first == 'f') {
        span.type = json_value_type_t::BOOLEAN;
        if (!json_match_literal(c, "false")) return false;
    } else if (first == 'n') {
        span.type = json_value_type_t::NIL;
        if (!json_match_literal(c, "null")) return false;
    } else {
        span.type = json_value_type_t::NUMBER;
        if (!json_scan_number(c)) return false;
    }
    span.end = c.pos;
    *out = span;
    return true;
}

static bool json_validate_exact(const uint8_t* data,
                                size_t len,
                                json_value_type_t required,
                                json_value_span_t* out = nullptr) {
    if (!data || len == 0) return false;
    json_cursor_t c{data, len, 0};
    json_value_span_t span;
    if (!json_scan_value(c, 0, &span)) return false;
    json_skip_ws(c);
    if (c.pos != len || (required != json_value_type_t::INVALID &&
                         span.type != required)) {
        return false;
    }
    if (out) *out = span;
    return true;
}

static bool json_size_add(size_t* total, size_t addition) {
    if (!total || addition > SIZE_MAX - *total) return false;
    *total += addition;
    return true;
}

static size_t json_escaped_size(const char* s, size_t len) {
    if (!s && len != 0) return SIZE_MAX;
    size_t total = 0;
    for (size_t i = 0; i < len; ++i) {
        const uint8_t ch = (uint8_t)s[i];
        size_t add = 1;
        if (ch == '"' || ch == '\\' || ch == '\b' || ch == '\f' ||
            ch == '\n' || ch == '\r' || ch == '\t') {
            add = 2;
        } else if (ch < 0x20U) {
            add = 6;
        }
        if (!json_size_add(&total, add)) return SIZE_MAX;
    }
    return total;
}

static char json_hex_digit(uint8_t v) {
    return v < 10U ? (char)('0' + v) : (char)('A' + (v - 10U));
}

static size_t json_write_escaped(char* out, const char* s, size_t len) {
    size_t p = 0;
    for (size_t i = 0; i < len; ++i) {
        const uint8_t ch = (uint8_t)s[i];
        switch (ch) {
            case '"': out[p++] = '\\'; out[p++] = '"'; break;
            case '\\': out[p++] = '\\'; out[p++] = '\\'; break;
            case '\b': out[p++] = '\\'; out[p++] = 'b'; break;
            case '\f': out[p++] = '\\'; out[p++] = 'f'; break;
            case '\n': out[p++] = '\\'; out[p++] = 'n'; break;
            case '\r': out[p++] = '\\'; out[p++] = 'r'; break;
            case '\t': out[p++] = '\\'; out[p++] = 't'; break;
            default:
                if (ch < 0x20U) {
                    out[p++] = '\\';
                    out[p++] = 'u';
                    out[p++] = '0';
                    out[p++] = '0';
                    out[p++] = json_hex_digit((uint8_t)(ch >> 4));
                    out[p++] = json_hex_digit((uint8_t)(ch & 0x0FU));
                } else {
                    out[p++] = (char)ch;
                }
                break;
        }
    }
    return p;
}

static bool json_number_token_valid(const char* value, size_t len) {
    if (!value || len == 0) return false;
    json_cursor_t c{reinterpret_cast<const uint8_t*>(value), len, 0};
    return json_scan_number(c) && c.pos == len;
}

static bool payload_utf8_valid(const char* value, size_t len) {
    if (!value && len != 0U) return false;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(value);
    size_t pos = 0U;
    while (pos < len) {
        if (data[pos] < 0x80U) {
            ++pos;
            continue;
        }
        size_t sequence_len = 0U;
        if (!json_validate_utf8_sequence(data, len, pos, &sequence_len)) {
            return false;
        }
        pos += sequence_len;
    }
    return true;
}

static bool json_array_locate(const char* json,
                              size_t len,
                              size_t wanted_index,
                              size_t* out_count,
                              json_value_span_t* out_value) {
    if (out_count) *out_count = 0;
    if (out_value) *out_value = json_value_span_t{};
    if (!json || len < 2U) return false;

    json_cursor_t c{reinterpret_cast<const uint8_t*>(json), len, 0};
    json_skip_ws(c);
    if (c.pos >= c.len || c.data[c.pos] != '[') return false;
    ++c.pos;
    json_skip_ws(c);

    size_t count = 0;
    if (c.pos < c.len && c.data[c.pos] == ']') {
        ++c.pos;
        json_skip_ws(c);
        if (c.pos != c.len) return false;
        if (out_count) *out_count = 0;
        return true;
    }

    while (c.pos < c.len) {
        json_value_span_t value;
        if (!json_scan_value(c, 1U, &value)) return false;
        if (count == wanted_index && out_value) *out_value = value;
        ++count;
        json_skip_ws(c);
        if (c.pos >= c.len) return false;
        if (c.data[c.pos] == ']') {
            ++c.pos;
            json_skip_ws(c);
            if (c.pos != c.len) return false;
            if (out_count) *out_count = count;
            return true;
        }
        if (c.data[c.pos] != ',') return false;
        ++c.pos;
        json_skip_ws(c);
    }
    return false;
}
// ============================================================================
// Payload storage and lifecycle
// ============================================================================

Payload::Payload()
    : _heap_block(nullptr),
      _heap_block_guard(UINTPTR_MAX),
      _count(0),
      _count_guard(UINT16_MAX),
      _data_begin((uint16_t)INLINE_STORAGE),
      _data_begin_guard((uint16_t)~(uint16_t)INLINE_STORAGE),
      _inline_storage{} {
    payload_mark_constructed();
    payload_note_handler_context(PAYLOAD_OP_CTOR, this,
                                 &g_payload_handler_ctx_ctor);
    payload_flight_note(PAYLOAD_OP_CTOR, this, 0U);
}

Payload::~Payload() {
    payload_flight_note(PAYLOAD_OP_DTOR, this, 0U);
    _release_storage();
    payload_mark_destroyed();
}

Payload::Payload(Payload&& other) noexcept
    : _heap_block(nullptr),
      _heap_block_guard(UINTPTR_MAX),
      _count(0),
      _count_guard(UINT16_MAX),
      _data_begin((uint16_t)INLINE_STORAGE),
      _data_begin_guard((uint16_t)~(uint16_t)INLINE_STORAGE),
      _inline_storage{} {
    payload_mark_constructed();
    payload_note_handler_context(PAYLOAD_OP_CTOR, this,
                                 &g_payload_handler_ctx_ctor);
    payload_flight_note(PAYLOAD_OP_CTOR, this, 0U);
    _move_from(other);
}

Payload& Payload::operator=(Payload&& other) noexcept {
    if (this == &other) return *this;
    if (!other._layout_ok(PAYLOAD_OP_MOVE_ASSIGN_SOURCE)) return *this;
    _release_storage();
    _move_from(other);
    return *this;
}

Payload::Payload(const Payload& other)
    : _heap_block(nullptr),
      _heap_block_guard(UINTPTR_MAX),
      _count(0),
      _count_guard(UINT16_MAX),
      _data_begin((uint16_t)INLINE_STORAGE),
      _data_begin_guard((uint16_t)~(uint16_t)INLINE_STORAGE),
      _inline_storage{} {
    payload_mark_constructed();
    payload_note_handler_context(PAYLOAD_OP_CTOR, this,
                                 &g_payload_handler_ctx_ctor);
    payload_flight_note(PAYLOAD_OP_CTOR, this, 0U);
    if (!_copy_from(other)) {
        payload_note_error(PAYLOAD_ERR_COPY_ALLOC_FAIL, PAYLOAD_OP_COPY_CTOR, this);
    }
}

Payload& Payload::operator=(const Payload& other) {
    if (this == &other) return *this;
    if (!other._layout_ok(PAYLOAD_OP_COPY_ASSIGN_SOURCE)) return *this;

    Payload temp;
    if (!temp._copy_from(other)) {
        payload_note_error(PAYLOAD_ERR_COPY_ALLOC_FAIL, PAYLOAD_OP_COPY_ASSIGN, this);
        return *this;
    }

    _release_storage();
    _move_from(temp);
    return *this;
}

bool Payload::_heap_guard_ok() const {
    return (((uintptr_t)_heap_block) ^ _heap_block_guard) == UINTPTR_MAX;
}

bool Payload::_count_guard_ok() const {
    return (uint16_t)(_count ^ _count_guard) == UINT16_MAX;
}

bool Payload::_data_begin_guard_ok() const {
    return (uint16_t)(_data_begin ^ _data_begin_guard) == UINT16_MAX;
}

void Payload::_set_heap_block(void* block) {
    _heap_block = block;
    _heap_block_guard = ~((uintptr_t)block);
}

void Payload::_set_count(uint16_t count) {
    _count = count;
    _count_guard = (uint16_t)~count;
}

void Payload::_set_data_begin(uint16_t data_begin) {
    _data_begin = data_begin;
    _data_begin_guard = (uint16_t)~data_begin;
}

uint8_t* Payload::_storage() {
    if (!_heap_guard_ok()) return nullptr;
    return _heap_block ? payload_heap_bytes(_heap_block) : _inline_storage;
}

const uint8_t* Payload::_storage() const {
    if (!_heap_guard_ok()) return nullptr;
    return _heap_block ? payload_heap_bytes((const void*)_heap_block) : _inline_storage;
}

size_t Payload::_capacity() const {
    if (!_heap_guard_ok()) return 0U;
    if (!_heap_block) return INLINE_STORAGE;
    return payload_heap_capacity(_heap_block, this, INLINE_STORAGE + 1U, STORAGE_MAX);
}

size_t Payload::_data_used() const {
    if (!_data_begin_guard_ok()) return 0U;
    const size_t capacity = _capacity();
    if (capacity == 0U || _data_begin > capacity) return 0U;
    return capacity - _data_begin;
}

Payload::Entry* Payload::_entries() {
    return reinterpret_cast<Entry*>(_storage());
}

const Payload::Entry* Payload::_entries() const {
    return reinterpret_cast<const Entry*>(_storage());
}

void Payload::_reset_empty() {
    _set_heap_block(nullptr);
    _set_count(0U);
    _set_data_begin((uint16_t)INLINE_STORAGE);
    memset(_inline_storage, 0, sizeof(_inline_storage));
}

void Payload::_release_storage() {
    if (!_heap_guard_ok()) {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE,
                               PAYLOAD_OP_RELEASE_STORAGE_GUARD,
                               this);
        _reset_empty();
        return;
    }
    if (_heap_block) {
        const size_t capacity =
            payload_heap_capacity(_heap_block, this, INLINE_STORAGE + 1U, STORAGE_MAX);
        if (capacity != 0) {
            payload_note_heap_delta(-(int32_t)capacity);
            payload_guarded_free(_heap_block, PAYLOAD_OP_RELEASE_STORAGE, this);
        } else {
            // Fail closed. A questionable pointer is leaked rather than passed
            // to free(); this is the only safe destructor policy after header
            // corruption.
            payload_note_integrity(PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE, PAYLOAD_OP_RELEASE_STORAGE, this);
        }
    }
    _reset_empty();
}

void Payload::_move_from(Payload& other) {
    if (!other._layout_ok(PAYLOAD_OP_MOVE_FROM)) {
        _reset_empty();
        return;
    }

    if (other._heap_block) {
        if (!payload_heap_rebind_owner(other._heap_block,
                                       &other,
                                       this,
                                       INLINE_STORAGE + 1U,
                                       STORAGE_MAX)) {
            _reset_empty();
            return;
        }
        _set_heap_block(other._heap_block);
        _set_count(other._count);
        _set_data_begin(other._data_begin);
        other._reset_empty();
        return;
    }

    memcpy(_inline_storage, other._inline_storage, INLINE_STORAGE);
    _set_heap_block(nullptr);
    _set_count(other._count);
    _set_data_begin(other._data_begin);
    other._reset_empty();
}

bool Payload::_copy_from(const Payload& other) {
    if (!other._layout_ok(PAYLOAD_OP_COPY_FROM)) return false;

    const size_t source_data_used = other._data_used();
    const size_t required =
        (size_t)other._count * sizeof(Entry) + source_data_used;

    size_t target_capacity = INLINE_STORAGE;
    if (required > INLINE_STORAGE) {
        target_capacity = payload_growth_capacity(INLINE_STORAGE,
                                                  required,
                                                  STORAGE_MAX);
        if (target_capacity == 0) return false;
        void* block = payload_heap_allocate(target_capacity, this);
        if (!block) {
            g_payload_arena_alloc_fail++;
            return false;
        }
        _set_heap_block(block);
        memset(payload_heap_bytes(block), 0, target_capacity);
        payload_note_heap_delta((int32_t)target_capacity);
        g_payload_arena_realloc_count++;
        const size_t data_capacity =
            target_capacity - (size_t)other._count * sizeof(Entry);
        if (data_capacity > g_payload_max_arena_capacity_seen) {
            g_payload_max_arena_capacity_seen = (uint32_t)data_capacity;
        }
    }

    uint8_t* target = _storage();
    const uint8_t* source = other._storage();
    const uint16_t target_data_begin =
        (uint16_t)(target_capacity - source_data_used);
    const int32_t shift =
        (int32_t)target_data_begin - (int32_t)other._data_begin;

    if (other._count != 0) {
        memcpy(target, source, (size_t)other._count * sizeof(Entry));
        Entry* entries = reinterpret_cast<Entry*>(target);
        for (size_t i = 0; i < other._count; ++i) {
            entries[i].key_off = (uint16_t)((int32_t)entries[i].key_off + shift);
            entries[i].val_off = (uint16_t)((int32_t)entries[i].val_off + shift);
        }
    }
    if (source_data_used != 0) {
        memcpy(target + target_data_begin,
               source + other._data_begin,
               source_data_used);
    }

    _set_count(other._count);
    _set_data_begin(target_data_begin);
    return true;
}

bool Payload::clear() {
    if (!_self_ok(PAYLOAD_OP_CLEAR)) {
        _release_storage();
        return false;
    }

    uint8_t* storage = _storage();
    const size_t entry_bytes = (size_t)_count * sizeof(Entry);
    const size_t data_used = _data_used();
    if (entry_bytes != 0U) memset(storage, 0, entry_bytes);
    if (data_used != 0U) memset(storage + _data_begin, 0, data_used);

    _set_count(0U);
    _set_data_begin((uint16_t)_capacity());
    return true;
}

bool Payload::empty() const {
    return !_self_ok(PAYLOAD_OP_EMPTY) || _count == 0;
}

Payload Payload::clone() const {
    return Payload(*this);
}

size_t Payload::count() const {
    return _self_ok(PAYLOAD_OP_COUNT) ? _count : 0;
}

size_t Payload::arena_used() const {
    return _self_ok(PAYLOAD_OP_ARENA_USED) ? _data_used() : 0;
}

size_t Payload::arena_capacity() const {
    if (!_self_ok(PAYLOAD_OP_ARENA_CAPACITY)) return 0;
    const size_t capacity = _capacity();
    const size_t entry_bytes = (size_t)_count * sizeof(Entry);
    return capacity >= entry_bytes ? capacity - entry_bytes : 0;
}

size_t Payload::entry_capacity() const {
    if (!_self_ok(PAYLOAD_OP_ENTRY_CAPACITY)) return 0;
    const size_t by_gap = _data_begin / sizeof(Entry);
    return by_gap < MAX_ENTRIES ? by_gap : MAX_ENTRIES;
}

bool Payload::heap_entries() const {
    return _heap_guard_ok() && _heap_block != nullptr && _capacity() != 0U;
}

// ============================================================================
// Payload integrity
// ============================================================================

bool Payload::_self_ok(uint32_t operation_id) const {
    if (!_heap_guard_ok() || !_count_guard_ok() || !_data_begin_guard_ok()) {
        payload_note_integrity(PAYLOAD_SELF_OK_INLINE_CAP_MISMATCH, operation_id, this);
        return false;
    }
    const size_t capacity = _capacity();
    const uint8_t* storage = capacity != 0 ? _storage() : nullptr;

    if (!storage || capacity == 0) {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE, operation_id, this);
        return false;
    }
    if (capacity > STORAGE_MAX) {
        payload_note_integrity(PAYLOAD_SELF_OK_ARENA_CAP_GT_MAX, operation_id, this);
        return false;
    }
    if (_count > MAX_ENTRIES) {
        payload_note_integrity(PAYLOAD_SELF_OK_COUNT_GT_MAX, operation_id, this);
        return false;
    }
    if (_data_begin > capacity) {
        payload_note_integrity(PAYLOAD_SELF_OK_ARENA_USED_GT_CAP, operation_id, this);
        return false;
    }

    const size_t entry_bytes = (size_t)_count * sizeof(Entry);
    if (entry_bytes > _data_begin) {
        payload_note_integrity(PAYLOAD_SELF_OK_COUNT_GT_ENTRY_CAP, operation_id, this);
        return false;
    }
    if (capacity - _data_begin > ARENA_MAX) {
        payload_note_integrity(PAYLOAD_SELF_OK_ARENA_USED_GT_CAP, operation_id, this);
        return false;
    }
    return true;
}

bool Payload::_entry_ok(const Entry& e,
                        size_t index,
                        uint32_t operation_id) const {
    if (!_self_ok(operation_id)) return false;
    const size_t capacity = _capacity();
    const uint8_t* storage = _storage();
    const size_t key_end = (size_t)e.key_off + (size_t)e.key_len;
    const size_t val_end = (size_t)e.val_off + (size_t)e.val_len;
    const payload_layout_evidence_t evidence = {
        (uint32_t)index,
        (uint32_t)capacity,
        (uint32_t)_data_begin,
        (uint32_t)_count,
        (uint32_t)e.key_off,
        (uint32_t)e.key_len,
        (uint32_t)e.val_off,
        (uint32_t)e.val_len,
        (uint32_t)e.kind,
        0U,
        (uint32_t)key_end,
        (uint32_t)val_end,
    };

    const uint8_t kind = e.kind;
    if (e.reserved != 0U ||
        kind < (uint8_t)ValueKind::STRING ||
        kind > (uint8_t)ValueKind::ARRAY) {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_KIND_BAD,
                               operation_id, this, &evidence);
        return false;
    }

    if (e.key_off < _data_begin || key_end >= capacity) {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_KEY_OFF_OOB,
                               operation_id, this, &evidence);
        return false;
    }
    if (e.val_off < _data_begin || val_end >= capacity) {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_VAL_END_OOB,
                               operation_id, this, &evidence);
        return false;
    }
    if (storage[key_end] != '\0') {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_KEY_UNTERMINATED,
                               operation_id, this, &evidence);
        return false;
    }
    if (storage[val_end] != '\0') {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_VAL_UNTERMINATED,
                               operation_id, this, &evidence);
        return false;
    }
    return true;
}

bool Payload::_layout_ok(uint32_t operation_id) const {
    if (!_self_ok(operation_id)) return false;

    const size_t capacity = _capacity();
    const uint8_t* storage = _storage();
    const Entry* entries = _entries();
    size_t expected_upper = capacity;

    for (size_t i = 0; i < _count; ++i) {
        const Entry& e = entries[i];
        if (!_entry_ok(e, i, operation_id)) return false;

        const size_t key_end = (size_t)e.key_off + (size_t)e.key_len;
        const size_t val_end = (size_t)e.val_off + (size_t)e.val_len;
        const payload_layout_evidence_t evidence = {
            (uint32_t)i,
            (uint32_t)capacity,
            (uint32_t)_data_begin,
            (uint32_t)_count,
            (uint32_t)e.key_off,
            (uint32_t)e.key_len,
            (uint32_t)e.val_off,
            (uint32_t)e.val_len,
            (uint32_t)e.kind,
            (uint32_t)expected_upper,
            (uint32_t)key_end,
            (uint32_t)val_end,
        };
        if (key_end + 1U != (size_t)e.val_off ||
            val_end + 1U != expected_upper) {
            payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_VAL_END_OOB,
                                   operation_id, this, &evidence);
            return false;
        }
        if (storage[key_end] != '\0' || storage[val_end] != '\0') {
            payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_VAL_UNTERMINATED,
                                   operation_id, this, &evidence);
            return false;
        }
        expected_upper = e.key_off;
    }

    if (expected_upper != _data_begin) {
        const payload_layout_evidence_t evidence = {
            0xFFFFFFFFUL,
            (uint32_t)capacity,
            (uint32_t)_data_begin,
            (uint32_t)_count,
            0U, 0U, 0U, 0U, 0U,
            (uint32_t)expected_upper,
            0U, 0U,
        };
        payload_note_integrity(PAYLOAD_SELF_OK_COUNT_WITHOUT_ARENA,
                               operation_id, this, &evidence);
        return false;
    }
    return true;
}

const Payload::Entry* Payload::_find(const char* key, size_t key_len) const {
    if (!_layout_ok(PAYLOAD_OP_FIND_LAYOUT)) return nullptr;
    const Entry* entries = _entries();
    const uint8_t* storage = _storage();
    for (size_t i = 0; i < _count; ++i) {
        const Entry& e = entries[i];
        if ((size_t)e.key_len == key_len &&
            (key_len == 0U || memcmp(storage + e.key_off, key, key_len) == 0)) {
            return &entries[i];
        }
    }
    return nullptr;
}

const Payload::Entry* Payload::_find(const char* key) const {
    if (!key) return nullptr;
    size_t key_len = 0;
    if (!payload_cstr_len_checked(key, ARENA_MAX, PAYLOAD_OP_FIND_KEY, &key_len, false)) {
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_FIND_KEY, this);
        return nullptr;
    }
    return _find(key, key_len);
}

const char* Payload::_value_ptr(const Entry& e) const {
    return reinterpret_cast<const char*>(_storage() + e.val_off);
}

// ============================================================================
// Single-store growth and transactional append
// ============================================================================

bool Payload::_ensure_room(size_t additional_entries,
                           size_t additional_data,
                           int32_t* data_shift) {
    if (data_shift) *data_shift = 0;
    if (!_self_ok(PAYLOAD_OP_ENSURE_ROOM)) return false;

    if (additional_entries > MAX_ENTRIES - _count) {
        g_payload_entry_overflow++;
        payload_note_error(PAYLOAD_ERR_ENTRY_OVERFLOW, PAYLOAD_OP_ENSURE_ROOM_ENTRIES, this);
        return false;
    }

    const size_t data_used = _data_used();
    if (additional_data > ARENA_MAX - data_used) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, PAYLOAD_OP_ENSURE_ROOM_DATA, this);
        return false;
    }

    const size_t new_count = (size_t)_count + additional_entries;
    const size_t required =
        new_count * sizeof(Entry) + data_used + additional_data;
    const size_t old_capacity = _capacity();
    if (required <= old_capacity) return true;
    if (!_layout_ok(PAYLOAD_OP_ENSURE_ROOM_GROW)) return false;

    const size_t new_capacity =
        payload_growth_capacity(old_capacity, required, STORAGE_MAX);
    if (new_capacity == 0 || new_capacity > UINT16_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, PAYLOAD_OP_ENSURE_ROOM_CAPACITY, this);
        return false;
    }

    void* new_block = payload_heap_allocate(new_capacity, this);
    if (!new_block) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_ALLOC_FAIL, PAYLOAD_OP_ENSURE_ROOM_ALLOC, this);
        return false;
    }
    payload_flight_note(PAYLOAD_OP_ENSURE_ROOM_ALLOC, this, 0U);

    uint8_t* new_storage = payload_heap_bytes(new_block);
    memset(new_storage, 0, new_capacity);
    const uint8_t* old_storage = _storage();
    const uint16_t new_data_begin =
        (uint16_t)(new_capacity - data_used);
    const int32_t shift =
        (int32_t)new_data_begin - (int32_t)_data_begin;

    if (_count != 0) {
        memcpy(new_storage, old_storage, (size_t)_count * sizeof(Entry));
        Entry* copied_entries = reinterpret_cast<Entry*>(new_storage);
        for (size_t i = 0; i < _count; ++i) {
            copied_entries[i].key_off =
                (uint16_t)((int32_t)copied_entries[i].key_off + shift);
            copied_entries[i].val_off =
                (uint16_t)((int32_t)copied_entries[i].val_off + shift);
        }
    }
    if (data_used != 0) {
        memcpy(new_storage + new_data_begin,
               old_storage + _data_begin,
               data_used);
    }

    void* old_block = _heap_block;
    const size_t old_heap_capacity = old_block ? old_capacity : 0U;
    _set_heap_block(new_block);
    _set_data_begin(new_data_begin);

    payload_note_heap_delta((int32_t)new_capacity - (int32_t)old_heap_capacity);
    g_payload_arena_realloc_count++;
    const size_t data_capacity =
        new_capacity - new_count * sizeof(Entry);
    if (data_capacity > g_payload_max_arena_capacity_seen) {
        g_payload_max_arena_capacity_seen = (uint32_t)data_capacity;
    }

    if (old_block) {
        payload_guarded_free(old_block, PAYLOAD_OP_ENSURE_ROOM_ALLOC, this);
    }
    if (data_shift) *data_shift = shift;
    return true;
}

bool Payload::_append_value(const char* key,
                            size_t key_len,
                            const char* value,
                            size_t value_len,
                            ValueKind kind) {
    payload_append_trace_record(payload_append_trace_stage_t::ENTER,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                nullptr, 0U, 0U, 0U,
                                false, false, 0U, 0U, 0, 0U, 0U);
    if (!_self_ok(PAYLOAD_OP_APPEND_VALUE)) return false;
    payload_note_handler_context(PAYLOAD_OP_APPEND_VALUE, this,
                                 &g_payload_handler_ctx_mutate);
    payload_flight_note(PAYLOAD_OP_APPEND_VALUE, this, 0U);
    if ((!key && key_len != 0) || (!value && value_len != 0)) return false;
    if (key_len > UINT16_MAX || value_len > UINT16_MAX) {
        payload_note_error(PAYLOAD_ERR_STRING_TOO_LONG, PAYLOAD_OP_APPEND_VALUE_LENGTH, this);
        return false;
    }
    if (key_len != 0 && !payload_span_readable(key, key_len, PAYLOAD_OP_APPEND_KEY)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_APPEND_KEY, this);
        return false;
    }
    if (value_len != 0 && !payload_span_readable(value, value_len, PAYLOAD_OP_APPEND_INPUT_VALUE)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_APPEND_INPUT_VALUE, this);
        return false;
    }
    if (!payload_utf8_valid(key, key_len)) {
        g_payload_parse_error++;
        g_payload_json_invalid_utf8_key++;
        payload_note_error(PAYLOAD_ERR_JSON_INVALID, PAYLOAD_OP_APPEND_VALUE_UTF8, this);
        return false;
    }
    if (kind == ValueKind::STRING && !payload_utf8_valid(value, value_len)) {
        g_payload_parse_error++;
        g_payload_json_invalid_utf8_value++;
        payload_note_error(PAYLOAD_ERR_JSON_INVALID, PAYLOAD_OP_APPEND_VALUE_UTF8, this);
        return false;
    }

    bool typed_value_valid = true;
    switch (kind) {
        case ValueKind::STRING:
            break;
        case ValueKind::NUMBER:
            typed_value_valid = json_number_token_valid(value, value_len);
            break;
        case ValueKind::BOOLEAN:
            typed_value_valid =
                (value_len == 4U && memcmp(value, "true", 4U) == 0) ||
                (value_len == 5U && memcmp(value, "false", 5U) == 0);
            break;
        case ValueKind::NIL:
            typed_value_valid =
                value_len == 4U && memcmp(value, "null", 4U) == 0;
            break;
        case ValueKind::OBJECT:
            typed_value_valid = json_validate_exact(
                reinterpret_cast<const uint8_t*>(value),
                value_len,
                json_value_type_t::OBJECT);
            break;
        case ValueKind::ARRAY:
            typed_value_valid = json_validate_exact(
                reinterpret_cast<const uint8_t*>(value),
                value_len,
                json_value_type_t::ARRAY);
            break;
        default:
            typed_value_valid = false;
            break;
    }
    if (!typed_value_valid) {
        g_payload_invalid_kind++;
        payload_note_error(PAYLOAD_ERR_JSON_INVALID, PAYLOAD_OP_APPEND_VALUE, this);
        return false;
    }

    const uint8_t* old_storage = _storage();
    const size_t old_capacity = _capacity();
    const size_t old_data_begin = _data_begin;

    bool key_alias = false;
    bool value_alias = false;
    size_t key_offset = 0;
    size_t value_offset = 0;
    if (key) {
        const uintptr_t address = (uintptr_t)key;
        const uintptr_t begin = (uintptr_t)old_storage;
        const uintptr_t end = begin + old_capacity;
        if (address >= begin && address < end) {
            key_alias = true;
            key_offset = (size_t)(address - (uintptr_t)old_storage);
        }
    }
    if (value) {
        const uintptr_t address = (uintptr_t)value;
        const uintptr_t begin = (uintptr_t)old_storage;
        const uintptr_t end = begin + old_capacity;
        if (address >= begin && address < end) {
            value_alias = true;
            value_offset = (size_t)(address - (uintptr_t)old_storage);
        }
    }

    const size_t additional_data = key_len + 1U + value_len + 1U;
    int32_t shift = 0;
    payload_append_trace_record(payload_append_trace_stage_t::PRE_ENSURE,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                old_storage, old_capacity, old_data_begin, _count,
                                key_alias, value_alias,
                                key_offset, value_offset,
                                shift, 0U, 0U);
    if (!_ensure_room(1U, additional_data, &shift)) {
        payload_append_trace_record(payload_append_trace_stage_t::ENSURE_FAILED,
                                    this,
                                    key, key_len,
                                    value, value_len,
                                    (uint32_t)kind,
                                    _storage(), _capacity(), _data_begin, _count,
                                    key_alias, value_alias,
                                    key_offset, value_offset,
                                    shift, 0U, 0U);
        return false;
    }

    uint8_t* storage = _storage();
    if (key_alias) {
        const size_t remapped = key_offset >= old_data_begin
            ? (size_t)((int32_t)key_offset + shift)
            : key_offset;
        key = reinterpret_cast<const char*>(storage + remapped);
    }
    if (value_alias) {
        const size_t remapped = value_offset >= old_data_begin
            ? (size_t)((int32_t)value_offset + shift)
            : value_offset;
        value = reinterpret_cast<const char*>(storage + remapped);
    }

    payload_append_trace_record(payload_append_trace_stage_t::POST_ENSURE,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                storage, _capacity(), _data_begin, _count,
                                key_alias, value_alias,
                                key_offset, value_offset,
                                shift, 0U, 0U);

    // V4.1 closes the validation-to-use gap. Growth, allocator activity, and
    // internal alias relocation all occur before this point, so the spans are
    // checked again at the exact custody boundary immediately preceding copy.
    if (value_len != 0 &&
        !payload_span_readable(value, value_len,
                               PAYLOAD_OP_APPEND_FINAL_VALUE_SPAN)) {
        payload_append_trace_record(payload_append_trace_stage_t::FINAL_SPAN_FAIL,
                                    this,
                                    key, key_len,
                                    value, value_len,
                                    (uint32_t)kind,
                                    storage, _capacity(), _data_begin, _count,
                                    key_alias, value_alias,
                                    key_offset, value_offset,
                                    shift, 0U, 0U);
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER,
                           PAYLOAD_OP_APPEND_FINAL_VALUE_SPAN,
                           this);
        return false;
    }
    if (key_len != 0 &&
        !payload_span_readable(key, key_len,
                               PAYLOAD_OP_APPEND_FINAL_KEY_SPAN)) {
        payload_append_trace_record(payload_append_trace_stage_t::FINAL_SPAN_FAIL,
                                    this,
                                    key, key_len,
                                    value, value_len,
                                    (uint32_t)kind,
                                    storage, _capacity(), _data_begin, _count,
                                    key_alias, value_alias,
                                    key_offset, value_offset,
                                    shift, 0U, 0U);
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER,
                           PAYLOAD_OP_APPEND_FINAL_KEY_SPAN,
                           this);
        return false;
    }

    const uint16_t val_off =
        (uint16_t)(_data_begin - (uint16_t)(value_len + 1U));
    const uint16_t key_off =
        (uint16_t)(val_off - (uint16_t)(key_len + 1U));

    payload_append_trace_record(payload_append_trace_stage_t::PRE_VALUE_COPY,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                storage, _capacity(), _data_begin, _count,
                                key_alias, value_alias,
                                key_offset, value_offset,
                                shift, key_off, val_off);
    if (value_len != 0) memmove(storage + val_off, value, value_len);
    storage[(size_t)val_off + value_len] = '\0';
    payload_append_trace_record(payload_append_trace_stage_t::VALUE_COPY_DONE,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                storage, _capacity(), _data_begin, _count,
                                key_alias, value_alias,
                                key_offset, value_offset,
                                shift, key_off, val_off);

    payload_append_trace_record(payload_append_trace_stage_t::PRE_KEY_COPY,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                storage, _capacity(), _data_begin, _count,
                                key_alias, value_alias,
                                key_offset, value_offset,
                                shift, key_off, val_off);
    if (key_len != 0) memmove(storage + key_off, key, key_len);
    storage[(size_t)key_off + key_len] = '\0';
    payload_append_trace_record(payload_append_trace_stage_t::KEY_COPY_DONE,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                storage, _capacity(), _data_begin, _count,
                                key_alias, value_alias,
                                key_offset, value_offset,
                                shift, key_off, val_off);

    Entry entry{};
    entry.key_off = key_off;
    entry.key_len = (uint16_t)key_len;
    entry.val_off = val_off;
    entry.val_len = (uint16_t)value_len;
    entry.kind = (uint8_t)kind;
    entry.reserved = 0;

    _entries()[_count] = entry;
    _set_data_begin(key_off);
    _set_count((uint16_t)(_count + 1U));
    payload_append_trace_record(payload_append_trace_stage_t::COMMIT,
                                this,
                                key, key_len,
                                value, value_len,
                                (uint32_t)kind,
                                storage, _capacity(), _data_begin, _count,
                                key_alias, value_alias,
                                key_offset, value_offset,
                                shift, key_off, val_off);

    if (_count > g_payload_entry_high_water_global) {
        g_payload_entry_high_water_global = _count;
    }
    if (_count > g_payload_max_entry_capacity_seen) {
        g_payload_max_entry_capacity_seen = _count;
    }
    const size_t used = _data_used();
    if (used > g_payload_arena_high_water_global) {
        g_payload_arena_high_water_global = (uint32_t)used;
    }
    return true;
}
bool Payload::_append_value_writer(const char* key,
                                   size_t key_len,
                                   size_t value_len,
                                   ValueKind kind,
                                   const Payload* object_value,
                                   const PayloadArray* array_value) {
    if (!_self_ok(PAYLOAD_OP_APPEND_WRITER)) return false;
    payload_note_handler_context(PAYLOAD_OP_APPEND_WRITER, this,
                                 &g_payload_handler_ctx_mutate);
    payload_flight_note(PAYLOAD_OP_APPEND_WRITER, this, 0U);
    if ((!object_value && !array_value) || (object_value && array_value)) return false;
    if (key_len > UINT16_MAX || value_len > UINT16_MAX) {
        payload_note_error(PAYLOAD_ERR_STRING_TOO_LONG, PAYLOAD_OP_APPEND_WRITER_LENGTH, this);
        return false;
    }
    if (key_len != 0 && !payload_span_readable(key, key_len, PAYLOAD_OP_APPEND_WRITER_KEY)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_APPEND_WRITER_KEY, this);
        return false;
    }
    if (!payload_utf8_valid(key, key_len)) {
        g_payload_parse_error++;
        g_payload_json_invalid_utf8_key++;
        payload_note_error(PAYLOAD_ERR_JSON_INVALID, PAYLOAD_OP_APPEND_WRITER_UTF8, this);
        return false;
    }
    const uint8_t* old_storage = _storage();
    const size_t old_capacity = _capacity();
    const size_t old_data_begin = _data_begin;
    bool key_alias = false;
    size_t key_offset = 0;
    if (key) {
        const uintptr_t address = (uintptr_t)key;
        const uintptr_t begin = (uintptr_t)old_storage;
        const uintptr_t end = begin + old_capacity;
        if (address >= begin && address < end) {
            key_alias = true;
            key_offset = (size_t)(address - (uintptr_t)old_storage);
        }
    }

    const size_t additional_data = key_len + 1U + value_len + 1U;
    int32_t shift = 0;
    if (!_ensure_room(1U, additional_data, &shift)) return false;

    uint8_t* storage = _storage();
    if (key_alias) {
        const size_t remapped = key_offset >= old_data_begin
            ? (size_t)((int32_t)key_offset + shift)
            : key_offset;
        key = reinterpret_cast<const char*>(storage + remapped);
    }

    const uint16_t val_off =
        (uint16_t)(_data_begin - (uint16_t)(value_len + 1U));
    const uint16_t key_off =
        (uint16_t)(val_off - (uint16_t)(key_len + 1U));

    size_t written = 0;
    if (object_value) {
        written = object_value->_write_json_unchecked(
            reinterpret_cast<char*>(storage + val_off));
    } else {
        written = array_value->_write_json_unchecked(
            reinterpret_cast<char*>(storage + val_off));
    }
    if (written != value_len) {
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW,
                           PAYLOAD_OP_APPEND_WRITER_WRITE,
                           this);
        return false;
    }
    storage[(size_t)val_off + value_len] = '\0';

    if (key_len != 0) memmove(storage + key_off, key, key_len);
    storage[(size_t)key_off + key_len] = '\0';

    Entry entry{};
    entry.key_off = key_off;
    entry.key_len = (uint16_t)key_len;
    entry.val_off = val_off;
    entry.val_len = (uint16_t)value_len;
    entry.kind = (uint8_t)kind;
    entry.reserved = 0;

    _entries()[_count] = entry;
    _set_data_begin(key_off);
    _set_count((uint16_t)(_count + 1U));

    if (_count > g_payload_entry_high_water_global) {
        g_payload_entry_high_water_global = _count;
    }
    if (_count > g_payload_max_entry_capacity_seen) {
        g_payload_max_entry_capacity_seen = _count;
    }
    const size_t used = _data_used();
    if (used > g_payload_arena_high_water_global) {
        g_payload_arena_high_water_global = (uint32_t)used;
    }
    return true;
}

static bool payload_key_length(const char* key, size_t* out_len) {
    if (!key) {
        if (out_len) *out_len = 0;
        return true;
    }
    return payload_cstr_len_checked(key,
                                    Payload::ARENA_MAX,
                                    PAYLOAD_OP_ADD_KEY,
                                    out_len,
                                    false);
}

static bool payload_format_checked(char* out,
                                   size_t out_size,
                                   int written,
                                   uint32_t operation_id,
                                   const void* self,
                                   size_t* out_len) {
    if (out_len) *out_len = 0;
    if (!out || out_size == 0) return false;
    if (written < 0) {
        out[0] = '\0';
        payload_note_error(PAYLOAD_ERR_STRING_TRUNCATION, operation_id, self);
        return false;
    }
    if ((size_t)written >= out_size) {
        out[out_size - 1U] = '\0';
        g_payload_string_truncation++;
        payload_note_error(PAYLOAD_ERR_STRING_TRUNCATION, operation_id, self);
        return false;
    }
    if (out_len) *out_len = (size_t)written;
    return true;
}


// ============================================================================
// Integer-only fixed-decimal rendering
// ============================================================================
//
// Conversion from float/double happens in util.cpp and produces fixed_decimal_t.
// From this boundary onward Payload operates only on integer decimal parts.

static bool payload_format_fixed_decimal(const fixed_decimal_t& value,
                                         char* out,
                                         size_t out_size,
                                         size_t* out_length) {
    static constexpr uint64_t POW10[] = {
        1ULL,
        10ULL,
        100ULL,
        1000ULL,
        10000ULL,
        100000ULL,
        1000000ULL,
        10000000ULL,
        100000000ULL,
        1000000000ULL,
        10000000000ULL,
        100000000000ULL,
        1000000000000ULL,
    };

    if (out_length) *out_length = 0U;
    if (!out || out_size == 0U) return false;
    out[0] = '\0';

    if (!value.valid() ||
        value.decimal_places > FIXED_DECIMAL_MAX_PLACES ||
        value.negative > 1U) {
        return false;
    }

    const uint64_t scale = POW10[value.decimal_places];
    if (value.fractional >= scale) return false;

    char reversed_whole[20];
    size_t whole_digits = 0U;
    uint64_t remaining_whole = value.whole;
    do {
        reversed_whole[whole_digits++] =
            (char)('0' + (uint8_t)(remaining_whole % 10ULL));
        remaining_whole /= 10ULL;
    } while (remaining_whole != 0ULL);

    const bool nonzero =
        value.whole != 0ULL || value.fractional != 0ULL;
    const size_t sign_chars =
        (value.negative != 0U && nonzero) ? 1U : 0U;
    const size_t decimal_chars =
        value.decimal_places != 0U
            ? 1U + (size_t)value.decimal_places
            : 0U;
    const size_t required = sign_chars + whole_digits + decimal_chars;
    if (required + 1U > out_size) return false;

    size_t pos = 0U;
    if (sign_chars != 0U) out[pos++] = '-';

    while (whole_digits != 0U) {
        out[pos++] = reversed_whole[--whole_digits];
    }

    if (value.decimal_places != 0U) {
        out[pos++] = '.';
        uint64_t divisor = scale / 10ULL;
        for (uint32_t i = 0U; i < value.decimal_places; ++i) {
            const uint8_t digit =
                (uint8_t)((value.fractional / divisor) % 10ULL);
            out[pos++] = (char)('0' + digit);
            divisor /= 10ULL;
        }
    }

    out[pos] = '\0';
    if (out_length) *out_length = pos;
    return true;
}

// add_fmt() is a string convenience API, not a back door into Payload's retired
// floating-point construction path.  Reject every printf floating conversion
// before va_list processing reaches libc.
static bool payload_format_uses_floating_conversion(const char* fmt,
                                                    size_t fmt_len) {
    if (!fmt) return false;

    for (size_t i = 0U; i < fmt_len; ++i) {
        if (fmt[i] != '%') continue;

        size_t j = i + 1U;
        if (j >= fmt_len) break;
        if (fmt[j] == '%') {
            i = j;
            continue;
        }

        // Optional POSIX positional argument prefix: %3$...
        size_t digits_begin = j;
        while (j < fmt_len && isdigit((unsigned char)fmt[j])) ++j;
        if (j < fmt_len && j != digits_begin && fmt[j] == '$') {
            ++j;
        } else {
            j = digits_begin;
        }

        while (j < fmt_len &&
               (fmt[j] == '-' || fmt[j] == '+' || fmt[j] == ' ' ||
                fmt[j] == '#' || fmt[j] == '0' || fmt[j] == '\'')) {
            ++j;
        }

        // Width, including an optional positional '*' argument: *4$
        if (j < fmt_len && fmt[j] == '*') {
            ++j;
            const size_t width_position_begin = j;
            while (j < fmt_len && isdigit((unsigned char)fmt[j])) ++j;
            if (j < fmt_len && j != width_position_begin && fmt[j] == '$') {
                ++j;
            }
        } else {
            while (j < fmt_len && isdigit((unsigned char)fmt[j])) ++j;
        }

        // Precision, including an optional positional '*' argument: .*5$
        if (j < fmt_len && fmt[j] == '.') {
            ++j;
            if (j < fmt_len && fmt[j] == '*') {
                ++j;
                const size_t precision_position_begin = j;
                while (j < fmt_len && isdigit((unsigned char)fmt[j])) ++j;
                if (j < fmt_len && j != precision_position_begin &&
                    fmt[j] == '$') {
                    ++j;
                }
            } else {
                while (j < fmt_len && isdigit((unsigned char)fmt[j])) ++j;
            }
        }

        if (j < fmt_len) {
            if ((fmt[j] == 'h' || fmt[j] == 'l') &&
                j + 1U < fmt_len && fmt[j + 1U] == fmt[j]) {
                j += 2U;
            } else if (fmt[j] == 'h' || fmt[j] == 'l' ||
                       fmt[j] == 'j' || fmt[j] == 'z' ||
                       fmt[j] == 't' || fmt[j] == 'L') {
                ++j;
            }
        }

        if (j >= fmt_len) break;
        switch (fmt[j]) {
            case 'a': case 'A':
            case 'e': case 'E':
            case 'f': case 'F':
            case 'g': case 'G':
                return true;
            default:
                break;
        }
        i = j;
    }

    return false;
}


// ============================================================================
// Semantic construction
// ============================================================================

bool Payload::add(const char* key, int32_t value) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_I32_KEY, this);
        return false;
    }
    char text[16];
    size_t len = 0;
    if (!payload_format_checked(text, sizeof(text),
                                snprintf(text, sizeof(text), "%ld", (long)value),
                                PAYLOAD_OP_ADD_I32, this, &len)) {
        return false;
    }
    return _append_value(key ? key : "", key_len, text, len, ValueKind::NUMBER);
}

bool Payload::add(const char* key, uint32_t value) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_U32_KEY, this);
        return false;
    }
    char text[16];
    size_t len = 0;
    if (!payload_format_checked(text, sizeof(text),
                                snprintf(text, sizeof(text), "%lu", (unsigned long)value),
                                PAYLOAD_OP_ADD_U32, this, &len)) {
        return false;
    }
    return _append_value(key ? key : "", key_len, text, len, ValueKind::NUMBER);
}

bool Payload::add(const char* key, int64_t value) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_I64_KEY, this);
        return false;
    }
    char text[32];
    size_t len = 0;
    if (!payload_format_checked(text, sizeof(text),
                                snprintf(text, sizeof(text), "%lld", (long long)value),
                                PAYLOAD_OP_ADD_I64, this, &len)) {
        return false;
    }
    return _append_value(key ? key : "", key_len, text, len, ValueKind::NUMBER);
}

bool Payload::add(const char* key, uint64_t value) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_U64_KEY, this);
        return false;
    }
    char text[32];
    size_t len = 0;
    if (!payload_format_checked(text, sizeof(text),
                                snprintf(text, sizeof(text), "%llu", (unsigned long long)value),
                                PAYLOAD_OP_ADD_U64, this, &len)) {
        return false;
    }
    return _append_value(key ? key : "", key_len, text, len, ValueKind::NUMBER);
}

bool Payload::add(const char* key, const char* value) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_CSTR_KEY, this);
        return false;
    }

    if (!value) value = "";
    size_t value_len = 0;
    if (!payload_cstr_len_checked(value,
                                  ARENA_MAX,
                                  PAYLOAD_OP_ADD_CSTR_VALUE,
                                  &value_len,
                                  false)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_CSTR_VALUE, this);
        return false;
    }
    return _append_value(key ? key : "", key_len,
                         value, value_len, ValueKind::STRING);
}

bool Payload::add(const char* key, const String& value) {
    return add(key, value.c_str());
}

bool Payload::add(const char* key, bool value) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_BOOL_KEY, this);
        return false;
    }
    const char* text = value ? "true" : "false";
    return _append_value(key ? key : "", key_len,
                         text, value ? 4U : 5U, ValueKind::BOOLEAN);
}

bool Payload::add(const char* key, const fixed_decimal_t& value) {
    static constexpr uint32_t operation_id =
        PAYLOAD_OP_ADD_FIXED_DECIMAL;

    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER,
                           PAYLOAD_OP_ADD_FIXED_DECIMAL_KEY,
                           this);
        return false;
    }

    static constexpr char FORMATTER_NAME[] = "SCALED_PARTS";
    char text[40];
    text[0] = '\0';
    int formatter_result = 0;

    auto substitute_null = [&](uint32_t reason,
                               uint32_t error_code) -> bool {
        payload_note_error(error_code, operation_id, this);
        payload_note_numeric_reject(reason,
                                    operation_id,
                                    this,
                                    key ? key : "",
                                    key_len,
                                    value.source_bits,
                                    (int)value.decimal_places,
                                    FORMATTER_NAME,
                                    sizeof(FORMATTER_NAME),
                                    (int)(sizeof(FORMATTER_NAME) - 1U),
                                    text,
                                    sizeof(text),
                                    formatter_result);
        const bool inserted = _append_value(key ? key : "",
                                            key_len,
                                            "null",
                                            4U,
                                            ValueKind::NIL);
        if (inserted) {
            g_payload_numeric_null_substitution++;
        } else {
            g_payload_numeric_null_insert_fail++;
        }
        return false;
    };

    if (!value.valid()) {
        switch (value.status) {
            case fixed_decimal_status_t::NAN_VALUE:
                return substitute_null(PAYLOAD_NUMERIC_REJECT_NAN,
                                       PAYLOAD_ERR_NONFINITE_NUMBER);
            case fixed_decimal_status_t::POSITIVE_INFINITY:
                return substitute_null(
                    PAYLOAD_NUMERIC_REJECT_POSITIVE_INFINITY,
                    PAYLOAD_ERR_NONFINITE_NUMBER);
            case fixed_decimal_status_t::NEGATIVE_INFINITY:
                return substitute_null(
                    PAYLOAD_NUMERIC_REJECT_NEGATIVE_INFINITY,
                    PAYLOAD_ERR_NONFINITE_NUMBER);
            case fixed_decimal_status_t::OUT_OF_RANGE:
                return substitute_null(PAYLOAD_NUMERIC_REJECT_OUT_OF_RANGE,
                                       PAYLOAD_ERR_FIXED_DECIMAL_RANGE);
            default:
                return substitute_null(PAYLOAD_NUMERIC_REJECT_FORMAT_FAILURE,
                                       PAYLOAD_ERR_FIXED_DECIMAL_RANGE);
        }
    }

    size_t len = 0U;
    if (!payload_format_fixed_decimal(value,
                                      text,
                                      sizeof(text),
                                      &len)) {
        formatter_result = -1;
        return substitute_null(PAYLOAD_NUMERIC_REJECT_FORMAT_FAILURE,
                               PAYLOAD_ERR_FIXED_DECIMAL_RANGE);
    }
    formatter_result = (int)len;

    if (!json_number_token_valid(text, len)) {
        return substitute_null(PAYLOAD_NUMERIC_REJECT_INVALID_TOKEN,
                               PAYLOAD_ERR_INVALID_NUMERIC_TOKEN);
    }

    return _append_value(key ? key : "",
                         key_len,
                         text,
                         len,
                         ValueKind::NUMBER);
}

bool Payload::add_fmt(const char* key, const char* fmt, ...) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_FMT_KEY, this);
        return false;
    }

    if (!fmt) fmt = "";
    size_t fmt_len = 0;
    if (!payload_cstr_len_checked(fmt, 96U, PAYLOAD_OP_ADD_FMT_FORMAT, &fmt_len, false)) {
        (void)fmt_len;
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_FMT_FORMAT, this);
        return false;
    }

    if (payload_format_uses_floating_conversion(fmt, fmt_len)) {
        payload_note_error(PAYLOAD_ERR_FLOAT_FORMAT_FORBIDDEN,
                           PAYLOAD_OP_ADD_FMT_FORMAT,
                           this);
        return false;
    }

    char text[128];
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    size_t value_len = 0;
    if (!payload_format_checked(text, sizeof(text), n,
                                PAYLOAD_OP_ADD_FMT, this, &value_len)) {
        return false;
    }
    return _append_value(key ? key : "", key_len,
                         text, value_len, ValueKind::STRING);
}

bool Payload::add_object(const char* key, const Payload& obj) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_OBJECT_KEY, this);
        return false;
    }
    const size_t value_len = obj._json_size();
    if (value_len == 0) {
        payload_note_error(PAYLOAD_ERR_INVALID_CHILD, PAYLOAD_OP_ADD_OBJECT_VALUE, this);
        return false;
    }
    if (value_len > UINT16_MAX) {
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW, PAYLOAD_OP_ADD_OBJECT_VALUE, this);
        return false;
    }
    return _append_value_writer(key ? key : "", key_len,
                                value_len, ValueKind::OBJECT, &obj, nullptr);
}

bool Payload::add_array(const char* key, const PayloadArray& arr) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_ARRAY_KEY, this);
        return false;
    }
    const size_t value_len = arr._json_size();
    if (value_len == 0) {
        payload_note_error(PAYLOAD_ERR_INVALID_CHILD, PAYLOAD_OP_ADD_ARRAY_VALUE, this);
        return false;
    }
    if (value_len > UINT16_MAX) {
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW, PAYLOAD_OP_ADD_ARRAY_VALUE, this);
        return false;
    }
    return _append_value_writer(key ? key : "", key_len,
                                value_len, ValueKind::ARRAY, nullptr, &arr);
}

bool Payload::add_raw_object(const char* key, const char* raw_json_object) {
    size_t key_len = 0;
    if (!payload_key_length(key, &key_len)) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_BAD_STRING_POINTER, PAYLOAD_OP_ADD_RAW_KEY, this);
        return false;
    }

    size_t value_len = 0;
    json_value_span_t span;
    if (!raw_json_object ||
        !payload_cstr_len_checked(raw_json_object,
                                  ARENA_MAX,
                                  PAYLOAD_OP_ADD_RAW_VALUE,
                                  &value_len,
                                  false) ||
        !json_validate_exact(reinterpret_cast<const uint8_t*>(raw_json_object),
                             value_len,
                             json_value_type_t::OBJECT,
                             &span)) {
        g_payload_parse_error++;
        g_payload_json_invalid_raw_object++;
        payload_note_error(PAYLOAD_ERR_JSON_INVALID, PAYLOAD_OP_ADD_RAW_VALUE, this);
        return false;
    }
    return _append_value(key ? key : "", key_len,
                         raw_json_object + span.begin,
                         span.end - span.begin,
                         ValueKind::OBJECT);
}

// ============================================================================
// Serialization
// ============================================================================

size_t Payload::_json_size() const {
    if (!_layout_ok(PAYLOAD_OP_JSON_SIZE)) return 0;

    size_t total = 2U;  // {}
    const Entry* entries = _entries();
    const uint8_t* storage = _storage();

    for (size_t i = 0; i < _count; ++i) {
        const Entry& e = entries[i];
        const char* value =
            reinterpret_cast<const char*>(storage + e.val_off);
        uint32_t semantic_reason = PAYLOAD_SEMANTIC_FAIL_NONE;
        switch ((ValueKind)e.kind) {
            case ValueKind::STRING:
                if (!payload_utf8_valid(value, e.val_len)) {
                    semantic_reason = PAYLOAD_SEMANTIC_FAIL_STRING_UTF8;
                }
                break;
            case ValueKind::NUMBER:
                if (!json_number_token_valid(value, e.val_len)) {
                    semantic_reason = PAYLOAD_SEMANTIC_FAIL_NUMBER_TOKEN;
                }
                break;
            case ValueKind::BOOLEAN:
                if (!((e.val_len == 4U && memcmp(value, "true", 4U) == 0) ||
                      (e.val_len == 5U && memcmp(value, "false", 5U) == 0))) {
                    semantic_reason = PAYLOAD_SEMANTIC_FAIL_BOOLEAN_TOKEN;
                }
                break;
            case ValueKind::NIL:
                if (!(e.val_len == 4U && memcmp(value, "null", 4U) == 0)) {
                    semantic_reason = PAYLOAD_SEMANTIC_FAIL_NULL_TOKEN;
                }
                break;
            case ValueKind::OBJECT:
                if (!json_validate_exact(
                        reinterpret_cast<const uint8_t*>(value),
                        e.val_len,
                        json_value_type_t::OBJECT)) {
                    semantic_reason = PAYLOAD_SEMANTIC_FAIL_OBJECT_JSON;
                }
                break;
            case ValueKind::ARRAY:
                if (!json_validate_exact(
                        reinterpret_cast<const uint8_t*>(value),
                        e.val_len,
                        json_value_type_t::ARRAY)) {
                    semantic_reason = PAYLOAD_SEMANTIC_FAIL_ARRAY_JSON;
                }
                break;
            default:
                semantic_reason = PAYLOAD_SEMANTIC_FAIL_INVALID_KIND;
                break;
        }
        const char* entry_key =
            reinterpret_cast<const char*>(storage + e.key_off);
        if (semantic_reason == PAYLOAD_SEMANTIC_FAIL_NONE &&
            !payload_utf8_valid(entry_key, e.key_len)) {
            semantic_reason = PAYLOAD_SEMANTIC_FAIL_KEY_UTF8;
        }
        if (semantic_reason != PAYLOAD_SEMANTIC_FAIL_NONE) {
            const size_t capacity = _capacity();
            const payload_layout_evidence_t evidence = {
                (uint32_t)i,
                (uint32_t)capacity,
                (uint32_t)_data_begin,
                (uint32_t)_count,
                (uint32_t)e.key_off,
                (uint32_t)e.key_len,
                (uint32_t)e.val_off,
                (uint32_t)e.val_len,
                (uint32_t)e.kind,
                0U,
                (uint32_t)((size_t)e.key_off + (size_t)e.key_len),
                (uint32_t)((size_t)e.val_off + (size_t)e.val_len),
            };
            g_payload_invalid_kind++;
            payload_note_semantic_failure(semantic_reason,
                                          PAYLOAD_OP_JSON_SIZE_VALUE,
                                          this,
                                          i,
                                          e.kind,
                                          e.key_off,
                                          e.key_len,
                                          e.val_off,
                                          e.val_len,
                                          entry_key);
            payload_note_integrity(PAYLOAD_SELF_OK_ENTRY_KIND_BAD,
                                   PAYLOAD_OP_JSON_SIZE_VALUE,
                                   this,
                                   &evidence);
            return 0;
        }

        const size_t key_size = json_escaped_size(
            reinterpret_cast<const char*>(storage + e.key_off), e.key_len);
        if (key_size == SIZE_MAX) return 0;

        size_t value_size = e.val_len;
        if ((ValueKind)e.kind == ValueKind::STRING) {
            const size_t escaped = json_escaped_size(
                reinterpret_cast<const char*>(storage + e.val_off), e.val_len);
            if (escaped == SIZE_MAX || escaped > SIZE_MAX - 2U) return 0;
            value_size = escaped + 2U;
        }

        // comma + quoted key + colon + value. The first entry has no comma.
        if (i != 0 && !json_size_add(&total, 1U)) return 0;
        if (!json_size_add(&total, 2U + key_size + 1U)) return 0;
        if (!json_size_add(&total, value_size)) return 0;
    }
    return total;
}

size_t Payload::_write_json_unchecked(char* buf) const {
    if (!buf) return 0;
    char* out = buf;
    *out++ = '{';

    const Entry* entries = _entries();
    const uint8_t* storage = _storage();
    for (size_t i = 0; i < _count; ++i) {
        const Entry& e = entries[i];
        if (i != 0) *out++ = ',';
        *out++ = '"';
        out += json_write_escaped(out,
                                  reinterpret_cast<const char*>(storage + e.key_off),
                                  e.key_len);
        *out++ = '"';
        *out++ = ':';

        const ValueKind kind = (ValueKind)e.kind;
        if (kind == ValueKind::STRING) {
            *out++ = '"';
            out += json_write_escaped(out,
                                      reinterpret_cast<const char*>(storage + e.val_off),
                                      e.val_len);
            *out++ = '"';
        } else {
            if (e.val_len != 0) {
                memcpy(out, storage + e.val_off, e.val_len);
                out += e.val_len;
            }
        }
    }
    *out++ = '}';
    *out = '\0';
    return (size_t)(out - buf);
}

size_t Payload::write_json(char* buf, size_t buf_size) const {
    if (!buf || buf_size == 0) {
        g_payload_serialize_overflow++;
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW, PAYLOAD_OP_WRITE_JSON_BUFFER, this);
        return 0;
    }
    buf[0] = '\0';

    const size_t needed = _json_size();
    if (needed == 0 || needed + 1U > buf_size) {
        g_payload_serialize_overflow++;
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW, PAYLOAD_OP_WRITE_JSON_SIZE, this);
        return 0;
    }
    return _write_json_unchecked(buf);
}

String Payload::to_json() const {
    const size_t needed = _json_size();
    if (needed == 0 || needed > UINT16_MAX) {
        g_payload_to_json_fail++;
        payload_note_error(PAYLOAD_ERR_TO_JSON_FAIL, PAYLOAD_OP_TO_JSON_SIZE, this);
        return String("{}");
    }

    char* buffer = static_cast<char*>(
        payload_guarded_malloc(needed + 1U, PAYLOAD_OP_TO_JSON_ALLOC, this));
    if (!buffer) {
        g_payload_arena_alloc_fail++;
        g_payload_to_json_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_ALLOC_FAIL, PAYLOAD_OP_TO_JSON_ALLOC, this);
        return String("{}");
    }

    const size_t written = _write_json_unchecked(buffer);
    if (written != needed) {
        payload_guarded_free(buffer, PAYLOAD_OP_TO_JSON_ALLOC, this);
        g_payload_to_json_fail++;
        payload_note_error(PAYLOAD_ERR_TO_JSON_FAIL, PAYLOAD_OP_TO_JSON_WRITE, this);
        return String("{}");
    }

    String result(buffer);
    payload_guarded_free(buffer, PAYLOAD_OP_TO_JSON_ALLOC, this);
    return result;
}

// ============================================================================
// Parsing
// ============================================================================

static uint8_t payload_kind_code_from_json_type(json_value_type_t type) {
    switch (type) {
        case json_value_type_t::STRING: return 1U;
        case json_value_type_t::NUMBER: return 2U;
        case json_value_type_t::BOOLEAN: return 3U;
        case json_value_type_t::NIL: return 4U;
        case json_value_type_t::OBJECT: return 5U;
        case json_value_type_t::ARRAY: return 6U;
        default: return 0U;
    }
}

bool Payload::parseJSON(const uint8_t* data, size_t len) {
    payload_note_handler_context(PAYLOAD_OP_PARSEJSON_DATA, this,
                                 &g_payload_handler_ctx_mutate);
    payload_flight_note(PAYLOAD_OP_PARSEJSON_DATA, this, 0U);
    if (!data || len < 2U || len > ARENA_MAX ||
        !payload_span_readable(data, len, PAYLOAD_OP_PARSEJSON_DATA)) {
        clear();
        g_payload_parse_error++;
        g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_DATA, this);
        return false;
    }

    json_cursor_t c{data, len, 0};
    json_skip_ws(c);
    if (c.pos >= c.len || c.data[c.pos] != '{') {
        clear();
        g_payload_parse_error++;
        g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_OPEN, this);
        return false;
    }
    ++c.pos;
    json_skip_ws(c);

    Payload parsed;
    if (c.pos < c.len && c.data[c.pos] == '}') {
        ++c.pos;
        json_skip_ws(c);
        if (c.pos != c.len) {
            clear();
            g_payload_parse_error++;
            g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_TRAILING, this);
            return false;
        }
        *this = static_cast<Payload&&>(parsed);
        return true;
    }

    while (c.pos < c.len) {
        size_t key_begin = 0;
        size_t key_end = 0;
        size_t key_decoded_len = 0;
        if (!json_scan_string(c, &key_begin, &key_end, &key_decoded_len)) {
            clear();
            g_payload_parse_error++;
            g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_KEY, this);
            return false;
        }

        json_skip_ws(c);
        if (c.pos >= c.len || c.data[c.pos] != ':') {
            clear();
            g_payload_parse_error++;
            g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_COLON, this);
            return false;
        }
        ++c.pos;

        json_value_span_t value;
        if (!json_scan_value(c, 1U, &value)) {
            clear();
            g_payload_parse_error++;
            g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_VALUE, this);
            return false;
        }

        const size_t value_stored_len =
            value.type == json_value_type_t::STRING
                ? value.decoded_length
                : value.end - value.begin;
        if (key_decoded_len > UINT16_MAX || value_stored_len > UINT16_MAX) {
            clear();
            g_payload_parse_error++;
            payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, PAYLOAD_OP_PARSEJSON_LENGTH, this);
            return false;
        }

        const size_t additional_data =
            key_decoded_len + 1U + value_stored_len + 1U;
        if (!parsed._ensure_room(1U, additional_data, nullptr)) {
            clear();
            g_payload_parse_error++;
            payload_note_error(PAYLOAD_ERR_ARENA_ALLOC_FAIL, PAYLOAD_OP_PARSEJSON_RESERVE, this);
            return false;
        }

        uint8_t* storage = parsed._storage();
        const uint16_t val_off =
            (uint16_t)(parsed._data_begin - (uint16_t)(value_stored_len + 1U));
        const uint16_t key_off =
            (uint16_t)(val_off - (uint16_t)(key_decoded_len + 1U));

        if (!json_decode_string(data,
                                key_begin,
                                key_end,
                                reinterpret_cast<char*>(storage + key_off),
                                key_decoded_len)) {
            clear();
            g_payload_parse_error++;
            g_payload_json_decode_fail++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_KEY_DECODE, this);
            return false;
        }

        if (value.type == json_value_type_t::STRING) {
            if (!json_decode_string(data,
                                    value.begin,
                                    value.end,
                                    reinterpret_cast<char*>(storage + val_off),
                                    value.decoded_length)) {
                clear();
                g_payload_parse_error++;
                g_payload_json_decode_fail++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_VALUE_DECODE, this);
                return false;
            }
        } else {
            if (value_stored_len != 0) {
                memcpy(storage + val_off,
                       data + value.begin,
                       value_stored_len);
            }
            storage[(size_t)val_off + value_stored_len] = '\0';
        }

        Entry entry{};
        entry.key_off = key_off;
        entry.key_len = (uint16_t)key_decoded_len;
        entry.val_off = val_off;
        entry.val_len = (uint16_t)value_stored_len;
        entry.kind = payload_kind_code_from_json_type(value.type);
        entry.reserved = 0;

        parsed._entries()[parsed._count] = entry;
        parsed._set_data_begin(key_off);
        parsed._set_count((uint16_t)(parsed._count + 1U));

        if (parsed._count > g_payload_entry_high_water_global) {
            g_payload_entry_high_water_global = parsed._count;
        }
        if (parsed._count > g_payload_max_entry_capacity_seen) {
            g_payload_max_entry_capacity_seen = parsed._count;
        }
        const size_t used = parsed._data_used();
        if (used > g_payload_arena_high_water_global) {
            g_payload_arena_high_water_global = (uint32_t)used;
        }

        json_skip_ws(c);
        if (c.pos >= c.len) {
            clear();
            g_payload_parse_error++;
            g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_EOF, this);
            return false;
        }
        if (c.data[c.pos] == '}') {
            ++c.pos;
            json_skip_ws(c);
            if (c.pos != c.len) {
                clear();
                g_payload_parse_error++;
                g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_TRAILING, this);
                return false;
            }
            *this = static_cast<Payload&&>(parsed);
            return true;
        }
        if (c.data[c.pos] != ',') {
            clear();
            g_payload_parse_error++;
            g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_SEPARATOR, this);
            return false;
        }
        ++c.pos;
        json_skip_ws(c);
    }

    clear();
    g_payload_parse_error++;
    g_payload_json_invalid_syntax++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, PAYLOAD_OP_PARSEJSON_UNCLOSED, this);
    return false;
}

// ============================================================================
// Lookup and accessors
// ============================================================================

bool Payload::has(const char* key) const {
    return _find(key) != nullptr;
}

const char* Payload::getString(const char* key) const {
    const Entry* e = _find(key);
    if (!e) return nullptr;
    const ValueKind kind = (ValueKind)e->kind;
    if (kind == ValueKind::OBJECT || kind == ValueKind::ARRAY) return nullptr;
    return _value_ptr(*e);
}

static bool payload_copy_token(const Payload* self,
                               const char* key,
                               const char* value,
                               size_t value_len,
                               uint32_t operation_id,
                               char* out,
                               size_t out_capacity) {
    if (!out || out_capacity == 0) return false;
    out[0] = '\0';
    if (!value || value_len == 0 || value_len >= out_capacity) {
        payload_note_bad_key(key);
        payload_note_error(PAYLOAD_ERR_STRING_TOO_LONG, operation_id, self);
        return false;
    }
    memcpy(out, value, value_len);
    out[value_len] = '\0';
    return true;
}

bool Payload::tryGetBool(const char* key, bool& out) const {
    const Entry* e = _find(key);
    if (!e || !_entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_TRY_BOOL)) return false;
    const ValueKind kind = (ValueKind)e->kind;
    if (kind != ValueKind::BOOLEAN && kind != ValueKind::STRING) return false;
    const char* value = _value_ptr(*e);
    if (e->val_len == 4U && memcmp(value, "true", 4U) == 0) {
        out = true;
        return true;
    }
    if (e->val_len == 5U && memcmp(value, "false", 5U) == 0) {
        out = false;
        return true;
    }
    return false;
}

bool Payload::tryGetInt(const char* key, int32_t& out) const {
    const Entry* e = _find(key);
    if (!e || !_entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_TRY_INT)) return false;
    const ValueKind kind = (ValueKind)e->kind;
    if (kind != ValueKind::NUMBER && kind != ValueKind::STRING) return false;

    const char* value = _value_ptr(*e);
    if (!json_number_token_valid(value, e->val_len)) return false;
    char text[48];
    if (!payload_copy_token(this, key, value, e->val_len,
                            PAYLOAD_OP_TRY_INT_TOKEN, text, sizeof(text))) {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const long long parsed = strtoll(text, &end, 10);
    if (errno == ERANGE || !end || *end != '\0' ||
        parsed < (long long)INT32_MIN || parsed > (long long)INT32_MAX) {
        return false;
    }
    out = (int32_t)parsed;
    return true;
}

bool Payload::tryGetUInt(const char* key, uint32_t& out) const {
    const Entry* e = _find(key);
    if (!e || !_entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_TRY_UINT)) return false;
    const ValueKind kind = (ValueKind)e->kind;
    if (kind != ValueKind::NUMBER && kind != ValueKind::STRING) return false;

    const char* value = _value_ptr(*e);
    if (e->val_len == 0 || value[0] == '-' ||
        !json_number_token_valid(value, e->val_len)) {
        return false;
    }
    char text[48];
    if (!payload_copy_token(this, key, value, e->val_len,
                            PAYLOAD_OP_TRY_UINT_TOKEN, text, sizeof(text))) {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = strtoull(text, &end, 10);
    if (errno == ERANGE || !end || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }
    out = (uint32_t)parsed;
    return true;
}

bool Payload::tryGetUInt64(const char* key, uint64_t& out) const {
    const Entry* e = _find(key);
    if (!e || !_entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_TRY_UINT64)) return false;
    const ValueKind kind = (ValueKind)e->kind;
    if (kind != ValueKind::NUMBER && kind != ValueKind::STRING) return false;

    const char* value = _value_ptr(*e);
    if (e->val_len == 0 || value[0] == '-' ||
        !json_number_token_valid(value, e->val_len)) {
        return false;
    }
    char text[48];
    if (!payload_copy_token(this, key, value, e->val_len,
                            PAYLOAD_OP_TRY_UINT64_TOKEN, text, sizeof(text))) {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = strtoull(text, &end, 10);
    if (errno == ERANGE || !end || *end != '\0') return false;
    out = (uint64_t)parsed;
    return true;
}

bool Payload::getBool(const char* key, bool default_value) const {
    bool value = false;
    return tryGetBool(key, value) ? value : default_value;
}

int32_t Payload::getInt(const char* key, int32_t default_value) const {
    int32_t value = 0;
    return tryGetInt(key, value) ? value : default_value;
}

uint32_t Payload::getUInt(const char* key, uint32_t default_value) const {
    uint32_t value = 0;
    return tryGetUInt(key, value) ? value : default_value;
}

uint64_t Payload::getUInt64(const char* key, uint64_t default_value) const {
    uint64_t value = 0;
    return tryGetUInt64(key, value) ? value : default_value;
}

Payload Payload::getPayload(const char* key) const {
    Payload result;
    const Entry* e = _find(key);
    if (!e || (ValueKind)e->kind != ValueKind::OBJECT) return result;
    if (!_entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_GETPAYLOAD)) return result;
    result.parseJSON(reinterpret_cast<const uint8_t*>(_value_ptr(*e)), e->val_len);
    return result;
}

PayloadArray Payload::getArray(const char* key) const {
    PayloadArray result;
    const Entry* e = _find(key);
    if (!e || (ValueKind)e->kind != ValueKind::ARRAY) return result;
    if (!_entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_GETARRAY)) return result;
    result.parseJSON(_value_ptr(*e));
    return result;
}

bool Payload::hasArray(const char* key) const {
    const Entry* e = _find(key);
    return e && (ValueKind)e->kind == ValueKind::ARRAY &&
           _entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_HASARRAY);
}

PayloadArrayView Payload::getArrayView(const char* key) const {
    const Entry* e = _find(key);
    if (!e || (ValueKind)e->kind != ValueKind::ARRAY ||
        !_entry_ok(*e, (size_t)(e - _entries()), PAYLOAD_OP_GETARRAYVIEW)) {
        return PayloadArrayView();
    }
    return PayloadArrayView(_value_ptr(*e), e->val_len);
}

// ============================================================================
// Explicit debug dump
// ============================================================================

void Payload::debug_dump(const char* tag) const {
    char safe_tag[32];
    size_t tag_len = 0;
    if (!tag || !payload_cstr_len_checked(tag,
                                          sizeof(safe_tag) - 1U,
                                          PAYLOAD_OP_DEBUG_DUMP_TAG,
                                          &tag_len,
                                          false)) {
        tag = "";
        tag_len = 0;
    }
    if (tag_len != 0) memcpy(safe_tag, tag, tag_len);
    safe_tag[tag_len] = '\0';

    char line[192];
    snprintf(line, sizeof(line),
             "Payload v4 (%s): entries=%u data=%u/%u heap=%u sizeof=%u",
             safe_tag,
             (unsigned)count(),
             (unsigned)arena_used(),
             (unsigned)arena_capacity(),
             (unsigned)heap_entries(),
             (unsigned)sizeof(Payload));
    debug_log("payload", line);

    if (!_self_ok(PAYLOAD_OP_DEBUG_DUMP)) return;
    const Entry* entries = _entries();
    const uint8_t* storage = _storage();
    for (size_t i = 0; i < _count; ++i) {
        const Entry& e = entries[i];
        if (!_entry_ok(e, i, PAYLOAD_OP_DEBUG_DUMP_ENTRY)) break;

        char key[80];
        size_t key_len = e.key_len;
        if (key_len >= sizeof(key)) key_len = sizeof(key) - 1U;
        if (key_len != 0) memcpy(key, storage + e.key_off, key_len);
        key[key_len] = '\0';

        char preview[96];
        size_t value_len = e.val_len;
        if (value_len >= sizeof(preview)) value_len = sizeof(preview) - 1U;
        if (value_len != 0) memcpy(preview, storage + e.val_off, value_len);
        preview[value_len] = '\0';

        debug_log("payload.key", key);
        switch ((ValueKind)e.kind) {
            case ValueKind::STRING: debug_log("payload.kind", "string"); break;
            case ValueKind::NUMBER: debug_log("payload.kind", "number"); break;
            case ValueKind::BOOLEAN: debug_log("payload.kind", "boolean"); break;
            case ValueKind::NIL: debug_log("payload.kind", "null"); break;
            case ValueKind::OBJECT: debug_log("payload.kind", "object"); break;
            case ValueKind::ARRAY: debug_log("payload.kind", "array"); break;
            default: debug_log("payload.kind", "invalid"); break;
        }
        debug_log("payload.value", preview);
    }
}
// ============================================================================
// PayloadArrayView
// ============================================================================

PayloadArrayView::PayloadArrayView()
    : _json(nullptr), _len(0) {}

PayloadArrayView::PayloadArrayView(const char* json, size_t len)
    : _json(json), _len(len) {}

bool PayloadArrayView::valid() const {
    return _json && _len >= 2U &&
           payload_span_readable(_json, _len, PAYLOAD_OP_ARRAY_VIEW_VALID) &&
           json_validate_exact(reinterpret_cast<const uint8_t*>(_json),
                               _len,
                               json_value_type_t::ARRAY);
}

size_t PayloadArrayView::size() const {
    if (!_json || _len < 2U ||
        !payload_span_readable(_json, _len, PAYLOAD_OP_ARRAY_VIEW_SIZE)) {
        return 0;
    }
    size_t count = 0;
    if (!json_array_locate(_json, _len, SIZE_MAX, &count, nullptr)) return 0;
    return count;
}

Payload PayloadArrayView::get(size_t index) const {
    Payload result;
    if (!_json || _len < 2U ||
        !payload_span_readable(_json, _len, PAYLOAD_OP_ARRAY_VIEW_GET)) {
        return result;
    }

    size_t count = 0;
    json_value_span_t value;
    if (!json_array_locate(_json, _len, index, &count, &value) ||
        index >= count || value.type != json_value_type_t::OBJECT) {
        return result;
    }
    result.parseJSON(reinterpret_cast<const uint8_t*>(_json + value.begin),
                     value.end - value.begin);
    return result;
}

// ============================================================================
// PayloadArray
// ============================================================================

PayloadArray::PayloadArray()
    : _heap_block(nullptr),
      _heap_block_guard(UINTPTR_MAX),
      _length(2),
      _length_guard((uint16_t)~(uint16_t)2U),
      _inline_storage{} {
    _inline_storage[0] = '[';
    _inline_storage[1] = ']';
    _inline_storage[2] = '\0';
}

PayloadArray::~PayloadArray() {
    _release_storage();
}

PayloadArray::PayloadArray(const PayloadArray& other)
    : _heap_block(nullptr),
      _heap_block_guard(UINTPTR_MAX),
      _length(2),
      _length_guard((uint16_t)~(uint16_t)2U),
      _inline_storage{} {
    _inline_storage[0] = '[';
    _inline_storage[1] = ']';
    _inline_storage[2] = '\0';
    (void)_copy_from(other);
}

PayloadArray& PayloadArray::operator=(const PayloadArray& other) {
    if (this == &other) return *this;
    if (other._json_size() == 0U) return *this;
    PayloadArray temp;
    if (!temp._copy_from(other)) return *this;
    _release_storage();
    _move_from(temp);
    return *this;
}

PayloadArray::PayloadArray(PayloadArray&& other) noexcept
    : _heap_block(nullptr),
      _heap_block_guard(UINTPTR_MAX),
      _length(2),
      _length_guard((uint16_t)~(uint16_t)2U),
      _inline_storage{} {
    _inline_storage[0] = '[';
    _inline_storage[1] = ']';
    _inline_storage[2] = '\0';
    _move_from(other);
}

PayloadArray& PayloadArray::operator=(PayloadArray&& other) noexcept {
    if (this == &other) return *this;
    if (other._json_size() == 0U) return *this;
    _release_storage();
    _move_from(other);
    return *this;
}

bool PayloadArray::_heap_guard_ok() const {
    return (((uintptr_t)_heap_block) ^ _heap_block_guard) == UINTPTR_MAX;
}

bool PayloadArray::_length_guard_ok() const {
    return (uint16_t)(_length ^ _length_guard) == UINT16_MAX;
}

void PayloadArray::_set_heap_block(void* block) {
    _heap_block = block;
    _heap_block_guard = ~((uintptr_t)block);
}

void PayloadArray::_set_length(uint16_t length) {
    _length = length;
    _length_guard = (uint16_t)~length;
}

char* PayloadArray::_data() {
    if (!_heap_guard_ok()) return nullptr;
    return reinterpret_cast<char*>(
        _heap_block ? payload_heap_bytes(_heap_block)
                    : reinterpret_cast<uint8_t*>(_inline_storage));
}

const char* PayloadArray::_data() const {
    if (!_heap_guard_ok()) return nullptr;
    return reinterpret_cast<const char*>(
        _heap_block ? payload_heap_bytes(_heap_block)
                    : reinterpret_cast<const uint8_t*>(_inline_storage));
}

size_t PayloadArray::_capacity() const {
    if (!_heap_guard_ok()) return 0U;
    if (!_heap_block) return INLINE_STORAGE;
    return payload_heap_capacity(_heap_block, this, INLINE_STORAGE + 1U, STORAGE_MAX);
}

bool PayloadArray::_self_ok() const {
    if (!_heap_guard_ok() || !_length_guard_ok()) return false;
    const size_t capacity = _capacity();
    const char* data = capacity != 0 ? _data() : nullptr;
    return data && capacity >= 3U && _length >= 2U &&
           (size_t)_length < capacity && data[0] == '[' &&
           data[_length - 1U] == ']' && data[_length] == '\0';
}

bool PayloadArray::_ensure_capacity(size_t needed) {
    if (!_self_ok()) return false;
    const size_t old_capacity = _capacity();
    if (needed <= old_capacity) return true;
    if (needed > STORAGE_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, PAYLOAD_OP_ARRAY_CAPACITY, this);
        return false;
    }

    const size_t new_capacity =
        payload_growth_capacity(old_capacity, needed, STORAGE_MAX);
    if (new_capacity == 0) return false;
    void* block = payload_heap_allocate(new_capacity, this);
    if (!block) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_ALLOC_FAIL, PAYLOAD_OP_ARRAY_ALLOC, this);
        return false;
    }

    memset(payload_heap_bytes(block), 0, new_capacity);
    memcpy(payload_heap_bytes(block), _data(), (size_t)_length + 1U);
    void* old_block = _heap_block;
    const size_t old_heap_capacity = old_block ? old_capacity : 0U;
    _set_heap_block(block);
    payload_note_heap_delta((int32_t)new_capacity - (int32_t)old_heap_capacity);
    g_payload_arena_realloc_count++;
    if (new_capacity > g_payload_max_arena_capacity_seen) {
        g_payload_max_arena_capacity_seen = (uint32_t)new_capacity;
    }
    if (old_block) {
        payload_guarded_free(old_block, PAYLOAD_OP_ARRAY_ALLOC, this);
    }
    return true;
}

void PayloadArray::_release_storage() {
    if (!_heap_guard_ok()) {
        payload_note_integrity(PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE,
                               PAYLOAD_OP_ARRAY_RELEASE_STORAGE_GUARD,
                               this);
        _set_heap_block(nullptr);
        _set_length(2U);
        memset(_inline_storage, 0, sizeof(_inline_storage));
        _inline_storage[0] = '[';
        _inline_storage[1] = ']';
        return;
    }
    if (_heap_block) {
        const size_t capacity =
            payload_heap_capacity(_heap_block, this, INLINE_STORAGE + 1U, STORAGE_MAX);
        if (capacity != 0) {
            payload_note_heap_delta(-(int32_t)capacity);
            payload_guarded_free(_heap_block,
                                 PAYLOAD_OP_ARRAY_RELEASE_STORAGE, this);
        } else {
            payload_note_integrity(PAYLOAD_SELF_OK_ENTRIES_SPAN_UNREADABLE,
                                   PAYLOAD_OP_ARRAY_RELEASE_STORAGE,
                                   this);
        }
    }
    _set_heap_block(nullptr);
    _set_length(2U);
    memset(_inline_storage, 0, sizeof(_inline_storage));
    _inline_storage[0] = '[';
    _inline_storage[1] = ']';
}

void PayloadArray::_move_from(PayloadArray& other) {
    if (other._json_size() == 0U) return;
    if (other._heap_block) {
        if (!payload_heap_rebind_owner(other._heap_block,
                                       &other,
                                       this,
                                       INLINE_STORAGE + 1U,
                                       STORAGE_MAX)) {
            return;
        }
        _set_heap_block(other._heap_block);
        _set_length(other._length);
        other._set_heap_block(nullptr);
        other._release_storage();
        return;
    }

    memcpy(_inline_storage, other._inline_storage, INLINE_STORAGE);
    _set_heap_block(nullptr);
    _set_length(other._length);
    other.clear();
}

bool PayloadArray::_copy_from(const PayloadArray& other) {
    if (other._json_size() == 0U) return false;
    if (!_ensure_capacity((size_t)other._length + 1U)) return false;
    memcpy(_data(), other._data(), (size_t)other._length + 1U);
    _set_length(other._length);
    return true;
}

bool PayloadArray::clear() {
    if (!_self_ok()) {
        _release_storage();
        return false;
    }
    char* data = _data();
    memset(data, 0, _capacity());
    data[0] = '[';
    data[1] = ']';
    _set_length(2U);
    return true;
}

bool PayloadArray::empty() const {
    return !_self_ok() || _length == 2U;
}

size_t PayloadArray::_json_size() const {
    if (!_self_ok()) return 0;
    return json_validate_exact(
               reinterpret_cast<const uint8_t*>(_data()),
               _length,
               json_value_type_t::ARRAY)
        ? _length
        : 0U;
}

size_t PayloadArray::_write_json_unchecked(char* out) const {
    if (!out || !_self_ok()) return 0;
    memcpy(out, _data(), _length);
    out[_length] = '\0';
    return _length;
}

String PayloadArray::to_json() const {
    return _json_size() != 0U ? String(_data()) : String("[]");
}

bool PayloadArray::add(const Payload& obj) {
    if (!_self_ok()) return false;
    const size_t object_size = obj._json_size();
    if (object_size == 0) {
        payload_note_error(PAYLOAD_ERR_INVALID_CHILD,
                           PAYLOAD_OP_ARRAY_ADD_WRITE,
                           this);
        return false;
    }

    const size_t comma = _length == 2U ? 0U : 1U;
    const size_t new_length = (size_t)_length + comma + object_size;
    if (new_length > UINT16_MAX || !_ensure_capacity(new_length + 1U)) {
        return false;
    }

    char* data = _data();
    const size_t old_length = _length;
    size_t pos = old_length - 1U;  // replace old closing bracket
    if (comma != 0) data[pos++] = ',';
    const size_t written = obj._write_json_unchecked(data + pos);
    if (written != object_size) {
        data[old_length - 1U] = ']';
        data[old_length] = '\0';
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW,
                           PAYLOAD_OP_ARRAY_ADD_WRITE,
                           this);
        return false;
    }
    pos += written;
    data[pos++] = ']';
    data[pos] = '\0';
    _set_length((uint16_t)pos);
    return true;
}

bool PayloadArray::parseJSON(const char* json) {
    size_t length = 0;
    if (!json || !payload_cstr_len_checked(json,
                                           STORAGE_MAX - 1U,
                                           PAYLOAD_OP_ARRAY_PARSE,
                                           &length,
                                           false)) {
        clear();
        g_payload_parse_error++;
        g_payload_json_invalid_raw_array++;
        payload_note_error(PAYLOAD_ERR_JSON_INVALID, PAYLOAD_OP_ARRAY_PARSE, this);
        return false;
    }

    json_value_span_t span;
    if (!json_validate_exact(reinterpret_cast<const uint8_t*>(json),
                             length,
                             json_value_type_t::ARRAY,
                             &span)) {
        clear();
        g_payload_parse_error++;
        g_payload_json_invalid_raw_array++;
        payload_note_error(PAYLOAD_ERR_JSON_INVALID, PAYLOAD_OP_ARRAY_PARSE_INVALID, this);
        return false;
    }

    const size_t trimmed_length = span.end - span.begin;
    if (trimmed_length > UINT16_MAX) {
        clear();
        return false;
    }

    PayloadArray parsed;
    if (!parsed._ensure_capacity(trimmed_length + 1U)) {
        clear();
        return false;
    }
    memcpy(parsed._data(), json + span.begin, trimmed_length);
    parsed._data()[trimmed_length] = '\0';
    parsed._set_length((uint16_t)trimmed_length);
    *this = static_cast<PayloadArray&&>(parsed);
    return true;
}

size_t PayloadArray::size() const {
    if (_json_size() == 0U) return 0U;
    size_t count = 0;
    return json_array_locate(_data(), _length, SIZE_MAX, &count, nullptr)
        ? count
        : 0U;
}

Payload PayloadArray::get(size_t idx) const {
    Payload result;
    if (!_self_ok()) return result;

    size_t count = 0;
    json_value_span_t value;
    if (!json_array_locate(_data(), _length, idx, &count, &value) ||
        idx >= count || value.type != json_value_type_t::OBJECT) {
        return result;
    }
    result.parseJSON(reinterpret_cast<const uint8_t*>(_data() + value.begin),
                     value.end - value.begin);
    return result;
}

// ============================================================================
// Instrumentation snapshot
// ============================================================================

void payload_get_info(payload_info_t* out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));

    out->payload_object_size = (uint32_t)sizeof(Payload);
    out->payload_entry_size = (uint32_t)sizeof(Payload::Entry);
    out->payload_array_object_size = (uint32_t)sizeof(PayloadArray);
    out->payload_inline_entries = (uint32_t)Payload::INLINE_ENTRIES;
    out->payload_max_entries = (uint32_t)Payload::MAX_ENTRIES;
    out->payload_arena_initial = (uint32_t)Payload::ARENA_INITIAL;
    out->payload_arena_max = (uint32_t)Payload::ARENA_MAX;

    out->instances_constructed = g_payload_instances_constructed;
    out->instances_destroyed = g_payload_instances_destroyed;
    out->alive_now = g_payload_alive_now;
    out->alive_high_water = g_payload_alive_high_water;

    out->entry_alloc_fail = g_payload_entry_alloc_fail;
    out->entry_realloc_count = g_payload_entry_realloc_count;
    out->entry_heap_bytes_alive = g_payload_entry_heap_bytes_alive;
    out->entry_heap_bytes_high_water = g_payload_entry_heap_bytes_high_water;
    out->entry_overflow = g_payload_entry_overflow;
    out->entry_high_water = g_payload_entry_high_water_global;
    out->max_entry_capacity_seen = g_payload_max_entry_capacity_seen;

    out->arena_alloc_fail = g_payload_arena_alloc_fail;
    out->arena_realloc_count = g_payload_arena_realloc_count;
    out->arena_heap_bytes_alive = g_payload_arena_heap_bytes_alive;
    out->arena_heap_bytes_high_water = g_payload_arena_heap_bytes_high_water;
    out->arena_high_water = g_payload_arena_high_water_global;
    out->max_arena_capacity_seen = g_payload_max_arena_capacity_seen;

    out->serialize_overflow = g_payload_serialize_overflow;
    out->to_json_fail = g_payload_to_json_fail;
    out->string_truncation = g_payload_string_truncation;
    out->parse_error = g_payload_parse_error;
    out->json_invalid_syntax = g_payload_json_invalid_syntax;
    out->json_invalid_depth = g_payload_json_invalid_depth;
    out->json_invalid_utf8_key = g_payload_json_invalid_utf8_key;
    out->json_invalid_utf8_value = g_payload_json_invalid_utf8_value;
    out->json_invalid_raw_object = g_payload_json_invalid_raw_object;
    out->json_invalid_raw_array = g_payload_json_invalid_raw_array;
    out->json_decode_fail = g_payload_json_decode_fail;
    out->integrity_fail = g_payload_integrity_fail;
    out->invalid_kind = g_payload_invalid_kind;

    out->numeric_null_substitution = g_payload_numeric_null_substitution;
    out->numeric_nonfinite = g_payload_numeric_nonfinite;
    out->numeric_nan = g_payload_numeric_nan;
    out->numeric_positive_infinity = g_payload_numeric_positive_infinity;
    out->numeric_negative_infinity = g_payload_numeric_negative_infinity;
    out->numeric_invalid_token = g_payload_numeric_invalid_token;
    out->numeric_format_failure = g_payload_numeric_format_failure;
    out->numeric_null_insert_fail = g_payload_numeric_null_insert_fail;
    out->last_numeric_reject_reason = g_payload_last_numeric_reject_reason;
    out->last_numeric_reject_op_id = g_payload_last_numeric_reject_op_id;
    out->last_numeric_reject_this = g_payload_last_numeric_reject_this;
    payload_copy_label(out->last_numeric_reject_key,
                       sizeof(out->last_numeric_reject_key),
                       g_payload_last_numeric_reject_key);
    out->last_numeric_reject_value_bits =
        g_payload_last_numeric_reject_value_bits;
    out->last_numeric_reject_precision =
        g_payload_last_numeric_reject_precision;
    out->last_numeric_reject_format_return =
        g_payload_last_numeric_reject_format_return;
    out->last_numeric_reject_snprintf_return =
        g_payload_last_numeric_reject_snprintf_return;
    out->last_numeric_reject_text_len =
        g_payload_last_numeric_reject_text_len;
    out->last_numeric_reject_text_terminated =
        g_payload_last_numeric_reject_text_terminated;
    out->last_numeric_reject_text_truncated =
        g_payload_last_numeric_reject_text_truncated;
    payload_copy_label(out->last_numeric_reject_format,
                       sizeof(out->last_numeric_reject_format),
                       g_payload_last_numeric_reject_format);
    payload_copy_label(out->last_numeric_reject_text_printable,
                       sizeof(out->last_numeric_reject_text_printable),
                       g_payload_last_numeric_reject_text_printable);
    payload_copy_label(out->last_numeric_reject_text_hex,
                       sizeof(out->last_numeric_reject_text_hex),
                       g_payload_last_numeric_reject_text_hex);

    out->semantic_validation_fail = g_payload_semantic_validation_fail;
    out->semantic_invalid_kind = g_payload_semantic_invalid_kind;
    out->semantic_invalid_key_utf8 = g_payload_semantic_invalid_key_utf8;
    out->semantic_invalid_string_utf8 = g_payload_semantic_invalid_string_utf8;
    out->semantic_invalid_number_token = g_payload_semantic_invalid_number_token;
    out->semantic_invalid_boolean_token = g_payload_semantic_invalid_boolean_token;
    out->semantic_invalid_null_token = g_payload_semantic_invalid_null_token;
    out->semantic_invalid_object_json = g_payload_semantic_invalid_object_json;
    out->semantic_invalid_array_json = g_payload_semantic_invalid_array_json;
    out->first_semantic_fail_captured = g_payload_first_semantic_fail_captured;
    out->first_semantic_fail_reason = g_payload_first_semantic_fail_reason;
    out->first_semantic_fail_op_id = g_payload_first_semantic_fail_op_id;
    out->first_semantic_fail_this = g_payload_first_semantic_fail_this;
    out->first_semantic_fail_entry_index = g_payload_first_semantic_fail_entry_index;
    out->first_semantic_fail_entry_kind = g_payload_first_semantic_fail_entry_kind;
    out->first_semantic_fail_key_off = g_payload_first_semantic_fail_key_off;
    out->first_semantic_fail_key_len = g_payload_first_semantic_fail_key_len;
    out->first_semantic_fail_val_off = g_payload_first_semantic_fail_val_off;
    out->first_semantic_fail_val_len = g_payload_first_semantic_fail_val_len;
    payload_copy_label(out->first_semantic_fail_key,
                       sizeof(out->first_semantic_fail_key),
                       g_payload_first_semantic_fail_key);

    out->string_pointer_fault = g_payload_string_pointer_fault;
    out->string_pointer_null = g_payload_string_pointer_null;
    out->string_pointer_low_address = g_payload_string_pointer_low_address;
    out->string_pointer_magic_address = g_payload_string_pointer_magic_address;
    out->string_pointer_out_of_range = g_payload_string_pointer_out_of_range;
    out->string_pointer_span_out_of_range =
        g_payload_string_pointer_span_out_of_range;
    out->string_pointer_unterminated = g_payload_string_pointer_unterminated;
    out->string_pointer_too_long = g_payload_string_pointer_too_long;
    out->last_string_pointer_fault_reason =
        g_payload_last_string_pointer_fault_reason;
    out->last_string_pointer_fault_ptr =
        g_payload_last_string_pointer_fault_ptr;
    out->last_string_pointer_fault_op_id =
        g_payload_last_string_pointer_fault_op_id;
    payload_copy_label(out->last_string_pointer_fault_context,
                       sizeof(out->last_string_pointer_fault_context),
                       g_payload_last_string_pointer_fault_context);
    out->last_string_pointer_fault_key_ptr =
        g_payload_last_string_pointer_fault_key_ptr;
    payload_copy_label(out->last_string_pointer_fault_key,
                       sizeof(out->last_string_pointer_fault_key),
                       g_payload_last_string_pointer_fault_key);

    out->self_ok_fail = g_payload_self_ok_fail;
    out->self_ok_magic_bad = g_payload_self_ok_magic_bad;
    out->self_ok_entries_null = g_payload_self_ok_entries_null;
    out->self_ok_entry_cap_low = g_payload_self_ok_entry_cap_low;
    out->self_ok_entry_cap_high = g_payload_self_ok_entry_cap_high;
    out->self_ok_entries_magic_address = g_payload_self_ok_entries_magic_address;
    out->self_ok_arena_magic_address = g_payload_self_ok_arena_magic_address;
    out->self_ok_entries_span_unreadable =
        g_payload_self_ok_entries_span_unreadable;
    out->self_ok_arena_span_unreadable =
        g_payload_self_ok_arena_span_unreadable;
    out->self_ok_inline_cap_mismatch = g_payload_self_ok_inline_cap_mismatch;
    out->self_ok_heap_cap_mismatch = g_payload_self_ok_heap_cap_mismatch;
    out->self_ok_count_gt_entry_cap = g_payload_self_ok_count_gt_entry_cap;
    out->self_ok_count_gt_max = g_payload_self_ok_count_gt_max;
    out->self_ok_arena_used_gt_cap = g_payload_self_ok_arena_used_gt_cap;
    out->self_ok_arena_cap_gt_max = g_payload_self_ok_arena_cap_gt_max;
    out->self_ok_arena_cap_zero_with_ptr =
        g_payload_self_ok_arena_cap_zero_with_ptr;
    out->self_ok_arena_cap_nonzero_with_null =
        g_payload_self_ok_arena_cap_nonzero_with_null;
    out->self_ok_count_without_arena = g_payload_self_ok_count_without_arena;
    out->self_ok_entry_kind_bad = g_payload_self_ok_entry_kind_bad;
    out->self_ok_entry_key_off_oob = g_payload_self_ok_entry_key_off_oob;
    out->self_ok_entry_val_off_oob = g_payload_self_ok_entry_val_off_oob;
    out->self_ok_entry_val_end_oob = g_payload_self_ok_entry_val_end_oob;
    out->self_ok_entry_val_unterminated =
        g_payload_self_ok_entry_val_unterminated;
    out->self_ok_entry_key_unterminated =
        g_payload_self_ok_entry_key_unterminated;
    out->last_self_ok_fail_reason = g_payload_last_self_ok_fail_reason;
    payload_copy_label(out->last_self_ok_fail_op,
                       sizeof(out->last_self_ok_fail_op),
                       g_payload_last_self_ok_fail_op);
    out->last_self_ok_fail_this = g_payload_last_self_ok_fail_this;
    out->last_self_ok_fail_magic = g_payload_last_self_ok_fail_magic;
    out->last_self_ok_fail_entries = g_payload_last_self_ok_fail_entries;
    out->last_self_ok_fail_arena = g_payload_last_self_ok_fail_arena;
    out->last_self_ok_fail_count = g_payload_last_self_ok_fail_count;
    out->last_self_ok_fail_entry_cap = g_payload_last_self_ok_fail_entry_cap;
    out->last_self_ok_fail_arena_used = g_payload_last_self_ok_fail_arena_used;
    out->last_self_ok_fail_arena_cap = g_payload_last_self_ok_fail_arena_cap;
    out->last_self_ok_fail_entry_index = g_payload_last_self_ok_fail_entry_index;
    out->last_self_ok_fail_entry_key_off =
        g_payload_last_self_ok_fail_entry_key_off;
    out->last_self_ok_fail_entry_val_off =
        g_payload_last_self_ok_fail_entry_val_off;
    out->last_self_ok_fail_entry_val_len =
        g_payload_last_self_ok_fail_entry_val_len;
    out->last_self_ok_fail_entry_kind = g_payload_last_self_ok_fail_entry_kind;
    out->last_self_ok_fail_op_id = g_payload_last_self_ok_fail_op_id;
    out->last_self_ok_fail_capacity = g_payload_last_self_ok_fail_capacity;
    out->last_self_ok_fail_data_begin = g_payload_last_self_ok_fail_data_begin;
    out->last_self_ok_fail_expected_upper =
        g_payload_last_self_ok_fail_expected_upper;
    out->last_self_ok_fail_key_end = g_payload_last_self_ok_fail_key_end;
    out->last_self_ok_fail_val_end = g_payload_last_self_ok_fail_val_end;
    out->first_self_ok_fail_captured = g_payload_first_self_ok_fail_captured;

    out->last_error_code = g_payload_last_error_code;
    out->last_error_count = g_payload_last_error_count;
    out->last_error_this = g_payload_last_error_this;
    out->last_error_op_id = g_payload_last_error_op_id;
    payload_copy_label(out->last_error_op,
                       sizeof(out->last_error_op),
                       g_payload_last_error_op);

    out->handler_ctx_ctor = g_payload_handler_ctx_ctor;
    out->handler_ctx_mutate = g_payload_handler_ctx_mutate;
    out->handler_ctx_alloc = g_payload_handler_ctx_alloc;
    out->handler_ctx_free = g_payload_handler_ctx_free;
    out->last_handler_ctx_ipsr = g_payload_last_handler_ctx_ipsr;
    out->last_handler_ctx_op_id = g_payload_last_handler_ctx_op_id;
    out->last_handler_ctx_this = g_payload_last_handler_ctx_this;
    out->last_handler_ctx_dwt = g_payload_last_handler_ctx_dwt;
    out->last_handler_ctx_msp = g_payload_last_handler_ctx_msp;

    out->alloc_overlap_detected = g_payload_alloc_overlap_detected;
    out->alloc_overlap_ipsr = g_payload_alloc_overlap_ipsr;
    out->alloc_overlap_op_id = g_payload_alloc_overlap_op_id;
    out->alloc_overlap_this = g_payload_alloc_overlap_this;
    out->alloc_overlap_dwt = g_payload_alloc_overlap_dwt;
    out->alloc_overlap_depth = g_payload_alloc_overlap_depth;
}