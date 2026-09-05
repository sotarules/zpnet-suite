#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <type_traits>

/*
  ============================================================================
  Payload v4.2 — Bimodal Single-Store Typed JSON Carrier
  ============================================================================

  HARD CONTRACT:

    • Payload owns one contiguous storage region.
    • The entry directory and all owned bytes live in that same region.
    • Values retain their JSON type; strings are never reclassified by text.
    • Add/attach operations are preflighted and committed transactionally.
    • Every state-changing operation is governed by explicit preconditions,
      whole-object invariants, and operation-specific postconditions.
    • Borrowed key/value spans are revalidated immediately before final copy,
      after any storage growth or internal-alias remapping.
    • Every successfully verified mutation advances an object-local generation
      and structural fingerprint.  A later mismatch is classified as drift
      between mutation boundaries rather than silently accepted.
    • Public construction mutators return void. Returning control proves the
      requested mutation completed and passed its postcondition court; a failed
      construction commits retained fatal evidence, emits WATCHDOG_ANOMALY via
      the allocator-free emergency transport path, and deliberately faults.
    • Payload runtime code performs no float or double conversion. Scientific
      values cross the construction boundary as integer-only fixed_decimal_t
      objects; floating read accessors are intentionally retired.
    • Invalid fixed-decimal objects are represented as JSON null, never omitted.
    • A failed construction or copy/move contract is terminal. No rollback or
      continued use of an object rejected by a contract is promised.
    • parseJSON() is a replacement operation and clears on invalid input, matching
      the established command-ingress contract.
    • Nested objects and arrays are written directly; no temporary JSON heap
      buffer is needed to attach a child.
    • Storage policy is selected only at construction. Default Payloads retain
      the existing dynamic inline/heap behavior; explicit FIXED Payloads use one
      caller-supplied dedicated writable store for their entire lifetime.
    • FIXED Payloads never allocate, resize, free, or transfer their backing store.
      Exceeding the declared fixed capacity is a fatal producer contract failure.
    • Object metadata is held with cheap redundant complements. Dynamic heap
      blocks remain bound to their exact owning object by pointer-derived cookie;
      fixed stores are bound by guarded pointer, capacity, and storage-mode state.
    • Corruption handling is fail-closed and quiet: no magic-number trap, stack
      walk, logging recursion, or diagnostic Payload construction.
    • Payload and PayloadArray require one foreground owner. Cross-context
      producers must hand off complete values using the system's standard SPSC
      mechanism; these classes do not provide whole-object synchronization.

  Compatibility:

    Payload() remains the default constructor and retains dynamic behavior. No
    existing caller must opt into or understand the fixed-storage path. The
    payload_info_t telemetry shape remains source-compatible with Payload v3.
    Legacy entry/arena and floating-number diagnostic fields are retained;
    retired fields remain zero unless an invalid fixed-decimal admission maps
    naturally onto the former numeric counters.

  ============================================================================
*/


// ============================================================================
// Payload design-by-contract evidence (retained, scalar-only)
// ============================================================================
//
// Payload never constructs an event from inside a mutator or integrity court.
// A failure is reduced to a fixed scalar incident and placed in the retained
// RAM2 transcript. Failure recording never re-inspects or serializes the rejected
// object. Legacy pending-event prefix fields remain present and zero. SYSTEM later
// constructs one best-effort PAYLOAD_CONTRACT_ANOMALY event from scalar evidence in
// foreground context while incident recursion is suppressed.

#define PAYLOAD_CONTRACT_INCIDENT_ENTRIES 16U
// RAM1 initiative: retain only two pending foreground anomaly events.
#define PAYLOAD_CONTRACT_PENDING_ENTRIES  2U
#define PAYLOAD_CONTRACT_PREFIX_BYTES     96U

// The contract implementation contains no conditional assembly and no inline
// assembly. Existing platform helpers provide DWT/IPSR and critical sections.

enum class payload_contract_phase_t : uint32_t {
  NONE             = 0,
  PRECONDITION     = 1,
  PRE_INVARIANT    = 2,
  POST_INVARIANT   = 3,
  POSTCONDITION    = 4,
  OBSERVED_DRIFT   = 5,
  MUTATION_FAILURE = 6,
  EVENT_EMISSION   = 7,
};

enum class payload_contract_reason_t : uint32_t {
  NONE                         = 0,
  INPUT_POINTER                = 1,
  INPUT_LENGTH                 = 2,
  INPUT_SYNTAX                 = 3,
  INPUT_KIND                   = 4,
  SOURCE_INVALID               = 5,
  ALLOCATION_FAILURE           = 6,
  SERIALIZATION_FAILURE        = 7,
  HEAP_GUARD                   = 8,
  COUNT_GUARD                  = 9,
  DATA_BEGIN_GUARD             = 10,
  HEAP_HEADER                  = 11,
  STORAGE_UNREADABLE           = 12,
  CAPACITY_RANGE               = 13,
  COUNT_RANGE                  = 14,
  DATA_RANGE                   = 15,
  DIRECTORY_DATA_OVERLAP       = 16,
  ENTRY_KIND                   = 17,
  ENTRY_KEY_RANGE              = 18,
  ENTRY_VALUE_RANGE            = 19,
  ENTRY_KEY_TERMINATOR         = 20,
  ENTRY_VALUE_TERMINATOR       = 21,
  PACKED_LAYOUT                = 22,
  SEMANTIC_KEY                 = 23,
  SEMANTIC_VALUE               = 24,
  STAMP_GUARD                  = 25,
  STAMP_MISMATCH               = 26,
  EXPECTED_COUNT               = 27,
  EXPECTED_DATA_DELTA          = 28,
  EXPECTED_SEMANTIC_PREFIX     = 29,
  EXPECTED_NEW_ENTRY           = 30,
  EXPECTED_EMPTY               = 31,
  EXPECTED_COPY                = 32,
  EXPECTED_MOVE_SOURCE_EMPTY   = 33,
  EXPECTED_PRESERVATION        = 34,
  EXPECTED_ARRAY_DELTA         = 35,
  INTERNAL_FAILURE             = 36,
  STORAGE_MODE_GUARD           = 37,
  FIXED_STORAGE_GUARD          = 38,
  FIXED_CAPACITY_GUARD         = 39,
};

struct payload_contract_incident_t {
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t phase;
  uint32_t reason;
  uint32_t operation_id;
  uint32_t object_ptr;
  uint32_t related_ptr;
  uint32_t generation;
  uint32_t entry_index;
  uint32_t expected0;
  uint32_t observed0;
  uint32_t expected1;
  uint32_t observed1;
  uint32_t before_fingerprint;
  uint32_t after_fingerprint;
  uint32_t dwt_cyccnt;
  uint32_t ipsr;
};

struct payload_contract_event_t {
  payload_contract_incident_t incident;
  // Retained for compatibility; failure recording leaves all prefix fields zero.
  uint32_t payload_prefix_valid;
  uint32_t payload_prefix_length;
  uint32_t payload_prefix_truncated;
  char payload_prefix[PAYLOAD_CONTRACT_PREFIX_BYTES];
};

struct payload_contract_bank_snapshot_t {
  uint32_t valid;
  uint32_t count;
  uint32_t newest_sequence;
  payload_contract_incident_t entries[PAYLOAD_CONTRACT_INCIDENT_ENTRIES];
};

struct payload_contract_snapshot_t {
  payload_contract_bank_snapshot_t live;
  payload_contract_bank_snapshot_t retained;
};

struct payload_contract_info_t {
  uint32_t checks;
  uint32_t successful_mutations;
  uint32_t precondition_failures;
  uint32_t pre_invariant_failures;
  uint32_t post_invariant_failures;
  uint32_t postcondition_failures;
  uint32_t observed_drift_failures;
  uint32_t mutation_failures;
  uint32_t incidents;
  uint32_t pending_events;
  uint32_t pending_overflow;
  uint32_t event_emitted;
  uint32_t event_emit_failed;
  uint32_t event_incidents_suppressed;
  payload_contract_incident_t first_this_boot;
  payload_contract_incident_t latest_this_boot;
  payload_contract_incident_t latest_retained;
};

const char* payload_contract_phase_name(uint32_t phase);
const char* payload_contract_reason_name(uint32_t reason);
void payload_contract_get_info(payload_contract_info_t* out);
void payload_contract_get_snapshot(payload_contract_snapshot_t* out);
bool payload_contract_event_peek(payload_contract_event_t* out);
void payload_contract_event_begin(void);
void payload_contract_event_end(bool emitted);
void payload_contract_clear_retained(void);

// ============================================================================
// Payload fatal construction court (retained, scalar-only)
// ============================================================================
//
// A construction mutator that cannot complete is a producer-integrity failure,
// not a recoverable client choice. The record is committed and cache-flushed
// before the allocator-free WATCHDOG_ANOMALY emergency path is attempted.

#define PAYLOAD_FATAL_SCHEMA_VERSION 1U

struct payload_fatal_record_t {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t schema_version;
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t error_code;
  uint32_t operation_id;
  uint32_t last_error_code;
  uint32_t last_error_operation_id;
  uint32_t last_error_count;
  uint32_t last_error_object_ptr;
  uint32_t object_ptr;
  uint32_t requested_bytes; // Zero when no detached attempted size was captured.
  uint32_t capacity;
  uint32_t count;
  uint32_t data_used;
  uint32_t ipsr;
  uint32_t dwt_cyccnt;
  uint32_t msp;
  uint32_t watchdog_attempted;
  uint32_t watchdog_sent;
};

bool payload_fatal_record_get(payload_fatal_record_t* out);
void payload_fatal_record_clear(void);

// ============================================================================
// Payload contract-stamp lifecycle recorder (retained, scalar-only)
// ============================================================================
//
// This recorder is deliberately narrow. It captures only constructor exit,
// clear entry, stamp failure, and fatal entry. No record traverses JSON bytes,
// recomputes hashes, allocates, emits events, or changes Payload semantics.

#define PAYLOAD_STAMP_TRACE_ENTRIES 16U

enum class payload_stamp_trace_stage_t : uint32_t {
  NONE          = 0,
  CTOR_EXIT     = 1,
  CLEAR_ENTER   = 2,
  STAMP_FAILURE = 3,
  FATAL_ENTER   = 4,
};

typedef struct {
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t stage;
  uint32_t operation_id;
  uint32_t this_ptr;
  uint32_t caller_lr;
  uint32_t msp;
  uint32_t dwt_cyccnt;
  uint32_t ipsr;

  uint32_t heap_block;
  uint32_t heap_block_guard;
  uint32_t count;
  uint32_t count_guard;
  uint32_t data_begin;
  uint32_t data_begin_guard;

  uint32_t contract_generation;
  uint32_t contract_generation_guard;
  uint32_t contract_fingerprint;
  uint32_t contract_fingerprint_guard;

  // Already-computed value supplied by the contract court when available.
  // Zero means no structural fingerprint was available at this stage.
  uint32_t structural_fingerprint;

  // Latest contract-incident sequence already committed when this stage was
  // captured. At STAMP_FAILURE this is intentionally the sequence before the
  // failure incident; FATAL_ENTER binds the subsequently committed one.
  uint32_t contract_incident_sequence;
} payload_stamp_trace_entry_t;

static_assert(sizeof(payload_stamp_trace_entry_t) == 84U,
              "Payload stamp trace entry geometry changed");

typedef struct {
  uint32_t valid;
  uint32_t count;
  uint32_t newest_sequence;
  payload_stamp_trace_entry_t entries[PAYLOAD_STAMP_TRACE_ENTRIES];
} payload_stamp_trace_bank_snapshot_t;

typedef struct {
  payload_stamp_trace_bank_snapshot_t live;
  payload_stamp_trace_bank_snapshot_t retained;
} payload_stamp_trace_snapshot_t;

const char* payload_stamp_trace_stage_name(uint32_t stage);
void payload_get_stamp_trace(payload_stamp_trace_snapshot_t* out);
void payload_clear_retained_stamp_trace(void);

// ============================================================================
// Shared newlib heap custody
// ============================================================================
//
// ZPNet runs on one core, but exception preemption can still create two software
// writers to newlib allocator metadata.  Payload and transport share this
// non-spinning court; any additional preemptible raw allocator path must join the
// same ownership domain.  The returned block has ordinary malloc semantics. A
// competing software owner is an architectural fault and traps; nullptr remains
// reserved for an actual allocator failure.
void* payload_shared_heap_malloc(size_t bytes);
void* payload_shared_heap_realloc(void* block, size_t bytes);
bool  payload_shared_heap_free(void* block);

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

  // Heap-growth strategy telemetry.  These distinguish fresh storage
  // allocation from true realloc(), and whether realloc could preserve the
  // block address (in-place extension) or had to relocate it.
  uint32_t arena_heap_alloc_count;
  uint32_t arena_heap_resize_attempt_count;
  uint32_t arena_heap_resize_reject_count;
  uint32_t arena_heap_realloc_call_count;
  uint32_t arena_heap_realloc_in_place_count;
  uint32_t arena_heap_realloc_moved_count;
  uint32_t arena_heap_realloc_fail_count;
  uint64_t arena_heap_realloc_in_place_growth_bytes;
  uint64_t arena_heap_realloc_moved_growth_bytes;

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

  // Primitive integer text-length custody. A mismatch may be repaired only when
  // the bounded local text is terminated, canonical decimal, and round-trips
  // exactly to the source integer; otherwise construction remains fatal.
  uint32_t numeric_length_mismatch;
  uint32_t numeric_length_recovered;
  uint32_t numeric_length_unrecoverable;
  uint32_t last_numeric_length_op_id;
  uint32_t last_numeric_length_this;
  uint32_t last_numeric_length_reported;
  uint32_t last_numeric_length_observed;
  uint32_t last_numeric_length_capacity;
  uint64_t last_numeric_length_value_bits;
  uint32_t last_numeric_length_terminated;
  uint32_t last_numeric_length_roundtrip;

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

  // Execution-context census — Payload activity performed while IPSR != 0.
  // Crash1 evidence implicated handler-context Payload code writing adjacent
  // to (or over) a stacked exception frame; these fields attribute that
  // activity without changing behavior.
  uint32_t handler_ctx_ctor;
  uint32_t handler_ctx_mutate;
  uint32_t handler_ctx_alloc;
  uint32_t handler_ctx_free;
  uint32_t last_handler_ctx_ipsr;
  uint32_t last_handler_ctx_op_id;
  uint32_t last_handler_ctx_this;
  uint32_t last_handler_ctx_dwt;
  uint32_t last_handler_ctx_msp;

  // Allocator preemption-overlap tripwire — one shared newlib heap owner was
  // already active when another Payload/transport allocation attempted entry.
  // The court is an atomic claim, so the detector itself has no check/increment
  // race and allocator metadata is never entered by the losing writer.
  uint32_t alloc_overlap_detected;
  uint32_t alloc_overlap_ipsr;
  uint32_t alloc_overlap_op_id;
  uint32_t alloc_overlap_this;
  uint32_t alloc_overlap_dwt;
  uint32_t alloc_overlap_depth;

  // Design-by-contract summary
  uint32_t contract_checks;
  uint32_t contract_successful_mutations;
  uint32_t contract_precondition_failures;
  uint32_t contract_pre_invariant_failures;
  uint32_t contract_post_invariant_failures;
  uint32_t contract_postcondition_failures;
  uint32_t contract_observed_drift_failures;
  uint32_t contract_mutation_failures;
  uint32_t contract_incidents;
  uint32_t contract_pending_events;
  uint32_t contract_pending_overflow;
  uint32_t contract_event_emitted;
  uint32_t contract_event_emit_failed;
  uint32_t contract_event_incidents_suppressed;

} payload_info_t;

void payload_get_info(payload_info_t* out);
const char* payload_error_code_name(uint32_t code);
const char* payload_string_fault_reason_name(uint32_t reason);
const char* payload_self_ok_fail_reason_name(uint32_t reason);
const char* payload_numeric_reject_reason_name(uint32_t reason);
const char* payload_semantic_fail_reason_name(uint32_t reason);
const char* payload_operation_id_name(uint32_t operation_id);

// ============================================================================
// Payload flight recorder (retained, read-only)
// ============================================================================
//
// A small ring of recent Payload lifecycle/mutation/failure records.  The
// live ring resides in RAM2 (NOLOAD), so the final Payload operations before
// a crash survive the reboot.  On the first Payload activity of each boot the
// surviving ring is latched into a retained snapshot bank before this boot's
// records overwrite it; both banks are validated by magic+complement so
// power-on garbage is reported as invalid rather than as evidence.
//
// Each record is scalar-only and written allocator-free; recording is safe in
// handler context.  The recorder is forensic best-effort, never authoritative.

#define PAYLOAD_FLIGHT_ENTRIES 32U

#define PAYLOAD_FLIGHT_FLAG_ERROR     0x0001U  // failure-path record
#define PAYLOAD_FLIGHT_FLAG_INTEGRITY 0x0002U  // integrity-courtroom record

typedef struct {
  uint32_t op_id;       // PAYLOAD_OP_* identity of the recorded operation
  uint32_t this_ptr;    // Payload/PayloadArray instance address
  uint32_t dwt_cyccnt;  // ARM_DWT_CYCCNT at the moment of the record
  uint16_t ipsr;        // 0 = thread mode, else active exception number
  uint16_t flags;       // PAYLOAD_FLIGHT_FLAG_*
} payload_flight_entry_t;

typedef struct {
  // This boot's ring, oldest first.
  uint32_t live_valid;
  uint32_t live_sequence;   // total records noted this boot
  uint32_t live_count;      // populated entries in live[]
  payload_flight_entry_t live[PAYLOAD_FLIGHT_ENTRIES];

  // Previous boot's final ring (the crash flight recorder), oldest first.
  uint32_t retained_valid;
  uint32_t retained_sequence;
  uint32_t retained_count;
  payload_flight_entry_t retained[PAYLOAD_FLIGHT_ENTRIES];
} payload_flight_info_t;

void payload_get_flight_info(payload_flight_info_t* out);

// ============================================================================
// Payload append transaction recorder (retained, read-only)
// ============================================================================
//
// V4.1 records key/value pointer custody across _ensure_room() and the final
// copy boundary. Each entry is scalar-only, allocation-free, committed with
// sequence/complement, and cache-flushed into RAM2.

#define PAYLOAD_APPEND_TRACE_ENTRIES 16U
#define PAYLOAD_APPEND_TRACE_KEY_PREFIX_BYTES 32U

enum class payload_append_trace_stage_t : uint32_t {
  NONE             = 0,
  ENTER            = 1,
  PRE_ENSURE       = 2,
  ENSURE_FAILED    = 3,
  POST_ENSURE      = 4,
  PRE_VALUE_COPY   = 5,
  VALUE_COPY_DONE  = 6,
  PRE_KEY_COPY     = 7,
  KEY_COPY_DONE    = 8,
  COMMIT           = 9,
  FINAL_SPAN_FAIL  = 10,
};

typedef struct {
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t stage;
  uint32_t this_ptr;
  uint32_t key_ptr;
  uint32_t value_ptr;
  uint32_t key_len;
  uint32_t value_len;
  uint32_t kind;
  uint32_t storage_ptr;
  uint32_t capacity;
  uint32_t data_begin;
  uint32_t entry_count;
  uint32_t alias_flags;   // bit 0 key alias, bit 1 value alias
  uint32_t key_offset;
  uint32_t value_offset;
  int32_t  data_shift;
  uint32_t key_off;
  uint32_t val_off;

  // Borrowed-source identity at this exact transaction stage.  Hashes use the
  // same length-seeded FNV-1a definition as the Payload contract court.
  // source_hash_valid is false for stages that intentionally precede pointer
  // admission; no source pointer is dereferenced merely to improve forensics.
  uint32_t source_hash_valid;
  uint32_t key_hash;
  uint32_t value_hash;
  uint32_t key_prefix_len;
  uint8_t  key_prefix[PAYLOAD_APPEND_TRACE_KEY_PREFIX_BYTES];

  uint32_t dwt_cyccnt;
  uint32_t ipsr;
} payload_append_trace_entry_t;

typedef struct {
  uint32_t valid;
  uint32_t count;
  uint32_t newest_sequence;
  payload_append_trace_entry_t entries[PAYLOAD_APPEND_TRACE_ENTRIES];
} payload_append_trace_bank_snapshot_t;

typedef struct {
  payload_append_trace_bank_snapshot_t live;
  payload_append_trace_bank_snapshot_t retained;
} payload_append_trace_snapshot_t;

void payload_get_append_trace(payload_append_trace_snapshot_t* out);
void payload_clear_retained_append_trace();

// ============================================================================
// Payload heap-resize transaction recorder (retained, read-only)
// ============================================================================
//
// Captures one bounded allocator-growth transaction across realloc(), header
// rewrite, semantic data relocation, offset rebasing, and the final preservation
// court.  The recorder is scalar-only, allocator-free, and retained in RAM2.
// It exists to distinguish allocator return/header/span failures from later
// Payload mutation faults without weakening any contract.

#define PAYLOAD_HEAP_RESIZE_TRACE_ENTRIES 8U

#define PAYLOAD_HEAP_RESIZE_FLAG_RAW_PRESENT      (1UL << 0)
#define PAYLOAD_HEAP_RESIZE_FLAG_ALIGNED          (1UL << 1)
#define PAYLOAD_HEAP_RESIZE_FLAG_HEADER_READABLE  (1UL << 2)
#define PAYLOAD_HEAP_RESIZE_FLAG_CAP_COMPLEMENT   (1UL << 3)
#define PAYLOAD_HEAP_RESIZE_FLAG_OWNER_COMPLEMENT (1UL << 4)
#define PAYLOAD_HEAP_RESIZE_FLAG_OWNER_MATCH      (1UL << 5)
#define PAYLOAD_HEAP_RESIZE_FLAG_CAP_EXPECTED     (1UL << 6)
#define PAYLOAD_HEAP_RESIZE_FLAG_STORAGE_SPAN     (1UL << 7)
#define PAYLOAD_HEAP_RESIZE_FLAG_MOVED            (1UL << 8)
#define PAYLOAD_HEAP_RESIZE_FLAG_CONTRACT_OK      (1UL << 9)

enum class payload_heap_resize_trace_stage_t : uint32_t {
  NONE               = 0,
  PRE_REALLOC        = 1,
  REALLOC_FAILED     = 2,
  POST_REALLOC       = 3,
  HEADER_WRITTEN     = 4,
  POST_DATA_MOVE     = 5,
  POST_OFFSET_REBASE = 6,
  POST_CONTRACT      = 7,
};

typedef struct {
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t stage;
  uint32_t this_ptr;
  uint32_t old_raw;
  uint32_t new_raw;
  uint32_t old_capacity;
  uint32_t new_capacity;
  uint32_t header0;
  uint32_t header1;
  uint32_t header2;
  uint32_t header3;
  uint32_t expected_owner_cookie;
  uint32_t span_remaining;
  uint32_t verdict_flags;
  uint32_t alloc_overlap_count;
  uint32_t alloc_overlap_depth;
  uint32_t dwt_cyccnt;
  uint32_t ipsr;
} payload_heap_resize_trace_entry_t;

typedef struct {
  uint32_t valid;
  uint32_t count;
  uint32_t newest_sequence;
  payload_heap_resize_trace_entry_t entries[PAYLOAD_HEAP_RESIZE_TRACE_ENTRIES];
} payload_heap_resize_trace_bank_snapshot_t;

typedef struct {
  payload_heap_resize_trace_bank_snapshot_t live;
  payload_heap_resize_trace_bank_snapshot_t retained;
} payload_heap_resize_trace_snapshot_t;

void payload_get_heap_resize_trace(payload_heap_resize_trace_snapshot_t* out);
void payload_clear_retained_heap_resize_trace();

// fixed_decimal_t is defined by util.h.  A forward declaration keeps Payload's
// header independent of the conversion implementation while allowing the
// integer-only publication object to cross the API by const reference.
struct fixed_decimal_t;

struct payload_contract_state_t;
struct payload_entry_access_t;

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
    enum class StorageMode : uint8_t {
        DYNAMIC = 0U,
        FIXED   = 1U,
    };

    // Existing callers remain dynamic by default. FIXED is an explicit opt-in
    // for bounded, high-frequency producers. The supplied store is dedicated to
    // this Payload, must remain writable/alive for its lifetime, and is never
    // allocated, resized, freed, or transferred by Payload.
    static constexpr size_t FIXED_STORAGE_ALIGNMENT = 4U;

    Payload();
    Payload(StorageMode mode, void* storage, size_t capacity);
    ~Payload();

    Payload(Payload&& other) noexcept;
    Payload& operator=(Payload&& other) noexcept;

    Payload(const Payload& other);
    Payload& operator=(const Payload& other);

    // Lifecycle
    void clear();
    bool empty() const;
    Payload clone() const;

    // Deterministic storage admission.  reserve() changes capacity only; it does
    // not change the semantic document.  Returning proves the requested capacity
    // exists and the preservation court passed.  Failure is fatal like add().
    void reserve(size_t minimum_capacity);

    // Serialization
    // json_size() returns the exact JSON byte count excluding the trailing NUL.
    // It performs the same integrity and semantic validation as write_json()
    // without allocating, so transport can reserve the final wire image once.
    size_t json_size() const;
    size_t write_json(char* buf, size_t buf_size) const;
    String to_json() const;

    // Semantic construction
    void add(const char* key, int32_t value);
    void add(const char* key, uint32_t value);
    void add(const char* key, int64_t value);
    void add(const char* key, uint64_t value);

    void add(const char* key, const char* value);
    void add(const char* key, const String& value);
    void add(const char* key, bool value);
    void add(const char* key, const fixed_decimal_t& value);

    // Construction is deliberately hostile to accidental FP reintroduction.
    // Call toFixedDecimal() in the owning/reporting module, then pass the
    // resulting integer-only object to the overload above.
    void add(const char* key, float value) = delete;
    void add(const char* key, double value) = delete;
    void add(const char* key, double value, int precision) = delete;

    template <typename T>
    typename std::enable_if<
        std::is_integral<T>::value &&
        std::is_signed<T>::value &&
        !std::is_same<T, bool>::value, void>::type
    add(const char* key, T value) {
        add(key, (int64_t)value);
    }

    template <typename T>
    typename std::enable_if<
        std::is_integral<T>::value &&
        std::is_unsigned<T>::value &&
        !std::is_same<T, bool>::value, void>::type
    add(const char* key, T value) {
        add(key, (uint64_t)value);
    }

    // String-format convenience only. Floating conversion specifiers
    // (%a/%e/%f/%g and uppercase forms) are rejected before vsnprintf().
    void add_fmt(const char* key, const char* fmt, ...);

    void add_object(const char* key, const Payload& obj);
    void add_array(const char* key, const PayloadArray& arr);
    void add_raw_object(const char* key, const char* raw_json_object);

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

    bool     getBool(const char* key, bool default_value = false) const;
    int32_t  getInt(const char* key, int32_t default_value = 0) const;
    uint32_t getUInt(const char* key, uint32_t default_value = 0) const;
    uint64_t getUInt64(const char* key, uint64_t default_value = 0) const;

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
    bool contract_valid() const;

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
    friend struct payload_entry_access_t;
    friend struct payload_stamp_trace_access_t;
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

    // Entry is a value record, never an object fabricated inside the byte store.
    // Directory publication and observation cross that boundary with fixed-size
    // memcpy only, so changing Entry to a non-trivial type is forbidden.
    static_assert(std::is_trivially_copyable<Entry>::value,
                  "Payload::Entry must remain trivially copyable");
    static_assert(std::is_standard_layout<Entry>::value,
                  "Payload::Entry must remain standard-layout");
    static_assert(sizeof(Entry) == 10U &&
                  offsetof(Entry, key_off) == 0U &&
                  offsetof(Entry, key_len) == 2U &&
                  offsetof(Entry, val_off) == 4U &&
                  offsetof(Entry, val_len) == 6U &&
                  offsetof(Entry, kind) == 8U &&
                  offsetof(Entry, reserved) == 9U,
                  "Payload::Entry byte schema changed");

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

    // Storage policy is immutable after construction. Dynamic mode keeps these
    // fixed-store fields canonically null/zero; FIXED mode keeps _heap_block
    // canonically null and binds the external store with redundant guards.
    void*     _fixed_storage;
    uintptr_t _fixed_storage_guard;
    uint16_t  _fixed_capacity;
    uint16_t  _fixed_capacity_guard;
    uint8_t   _storage_mode;
    uint8_t   _storage_mode_guard;

    uint16_t  _count;
    uint16_t  _count_guard;
    uint16_t  _data_begin;
    uint16_t  _data_begin_guard;
    uint32_t  _contract_generation;
    uint32_t  _contract_generation_guard;
    uint32_t  _contract_fingerprint;
    uint32_t  _contract_fingerprint_guard;
    alignas(uint32_t) uint8_t _inline_storage[INLINE_STORAGE];

    bool _heap_guard_ok() const;
    bool _fixed_storage_guard_ok() const;
    bool _fixed_capacity_guard_ok() const;
    bool _storage_mode_guard_ok() const;
    bool _fixed_mode() const;
    bool _count_guard_ok() const;
    bool _data_begin_guard_ok() const;
    void _set_heap_block(void* block);
    void _set_fixed_storage(void* storage);
    void _set_fixed_capacity(uint16_t capacity);
    void _set_storage_mode(StorageMode mode);
    void _set_count(uint16_t count);
    void _set_data_begin(uint16_t data_begin);

    uint8_t* _storage();
    const uint8_t* _storage() const;
    size_t _capacity() const;
    size_t _data_used() const;

    void _reset_empty();
    void _release_storage();
    void _move_from(Payload& other);
    bool _copy_from(const Payload& other);
    [[noreturn]] void _fatal(uint32_t fallback_error_code,
                             uint32_t fallback_operation_id,
                             size_t requested_bytes = 0U) const;

    bool _self_ok(uint32_t operation_id) const;
    bool _entry_ok(const Entry& e, size_t index, uint32_t operation_id) const;
    bool _layout_ok(uint32_t operation_id) const;

    bool _contract_inspect(payload_contract_state_t* out) const;
    bool _contract_begin(uint32_t operation_id,
                         payload_contract_state_t* before) const;
    bool _contract_finish_preserve(uint32_t operation_id,
                                   const payload_contract_state_t& before,
                                   size_t minimum_capacity);
    bool _contract_finish_add(uint32_t operation_id,
                              const payload_contract_state_t& before,
                              size_t key_len,
                              size_t value_len,
                              ValueKind kind,
                              uint32_t expected_key_hash,
                              uint32_t expected_value_hash);
    bool _contract_finish_clear(uint32_t operation_id,
                                const payload_contract_state_t& before);
    bool _contract_finish_copy(uint32_t operation_id,
                               const payload_contract_state_t& before,
                               uint32_t expected_semantic_hash);
    bool _contract_abort(uint32_t operation_id,
                         const payload_contract_state_t& before);
    void _contract_accept(const payload_contract_state_t& state);
    uint32_t _contract_semantic_hash(size_t entry_limit) const;
    uint32_t _json_hash_unchecked() const;

    bool _ensure_room(size_t additional_entries,
                      size_t additional_data,
                      int32_t* data_shift = nullptr);

    bool _append_value(const char* key,
                       size_t key_len,
                       const char* value,
                       size_t value_len,
                       ValueKind kind);

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

    void clear();
    bool empty() const;

    String to_json() const;
    void add(const Payload& obj);
    bool parseJSON(const char* json);

    size_t  size() const;
    Payload get(size_t idx) const;
    bool contract_valid() const;

private:
    friend class Payload;

    static constexpr size_t INLINE_STORAGE = 256;
    static constexpr size_t STORAGE_MAX = Payload::ARENA_MAX;

    void*     _heap_block;
    uintptr_t _heap_block_guard;
    uint16_t  _length;
    uint16_t  _length_guard;
    uint32_t  _contract_generation;
    uint32_t  _contract_generation_guard;
    uint32_t  _contract_fingerprint;
    uint32_t  _contract_fingerprint_guard;
    alignas(uint32_t) char _inline_storage[INLINE_STORAGE];

    bool _heap_guard_ok() const;
    bool _length_guard_ok() const;
    void _set_heap_block(void* block);
    void _set_length(uint16_t length);

    char* _data();
    const char* _data() const;
    size_t _capacity() const;
    bool _self_ok() const;
    bool _contract_inspect(payload_contract_state_t* out) const;
    bool _contract_begin(uint32_t operation_id,
                         payload_contract_state_t* before) const;
    bool _contract_finish_preserve(uint32_t operation_id,
                                   const payload_contract_state_t& before,
                                   size_t minimum_capacity);
    bool _contract_finish_add(uint32_t operation_id,
                              const payload_contract_state_t& before,
                              size_t object_size,
                              uint32_t object_hash);
    bool _contract_finish_clear(uint32_t operation_id,
                                const payload_contract_state_t& before);
    bool _contract_finish_copy(uint32_t operation_id,
                               const payload_contract_state_t& before,
                               uint32_t expected_hash);
    bool _contract_abort(uint32_t operation_id,
                         const payload_contract_state_t& before);
    void _contract_accept(const payload_contract_state_t& state);
    uint32_t _json_hash_unchecked() const;

    bool _ensure_capacity(size_t needed);
    void _release_storage();
    void _move_from(PayloadArray& other);
    bool _copy_from(const PayloadArray& other);
    [[noreturn]] void _fatal(uint32_t fallback_error_code,
                             uint32_t fallback_operation_id,
                             size_t requested_bytes = 0U) const;

    size_t _json_size() const;
    size_t _write_json_unchecked(char* out) const;
};