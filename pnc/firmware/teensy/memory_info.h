#pragma once

#include <stdint.h>

// ============================================================================
// ZPNet Memory Info — Teensy 4.x Memory Health Snapshot
// ============================================================================
//
// Teensy 4.1 memory layout (authoritative):
//
//   RAM1 (512KB) = ITCM (code) + DTCM (data + stack)
//     • Stack lives in DTCM, grows DOWNWARD from _estack
//     • Global/static variables grow UPWARD from _sdata
//     • These two can collide if stack usage is excessive
//
//   RAM2 (512KB) = OCRAM
//     • DMAMEM variables at base
//     • Heap (malloc/new/Payload arenas) grows UPWARD from _heap_start
//     • Heap ceiling is _heap_end (top of RAM2)
//     • Fragmentation: freed blocks inside the heap arena that
//       cannot be returned to the OS (sbrk never shrinks)
//
// What this module provides:
//
//   1) DTCM / Stack health
//      - Stack high-water mark via sentinel painting
//      - Current stack depth
//      - Total DTCM available for stack
//
//   2) Heap (RAM2) health
//      - Heap arena size (how far sbrk has advanced)
//      - Heap bytes in active use
//      - Heap bytes freed but trapped (fragmentation)
//      - Distance from heap ceiling
//      - High-water mark for heap arena
//
//   3) Trend detection
//      - Monotonic sbrk growth detection (leak indicator)
//
// Usage:
//   Call memory_info_init() once in setup() BEFORE any other code.
//   Call memory_info_get() at any time for a snapshot.
//
// ============================================================================

struct memory_info_t {

    // --------------------------------------------------------
    // DTCM / Stack
    // --------------------------------------------------------

    uint32_t dtcm_total;              // total DTCM bytes (data + bss + stack region)
    uint32_t dtcm_static;             // bytes used by globals + bss
    uint32_t dtcm_stack_avail;        // total bytes available for stack (dtcm_total - dtcm_static)

    bool     initialized;             // true after memory_info_init() completed
    uint32_t stack_init_sp;           // SP observed during memory_info_init()
    uint32_t stack_current;           // current stack depth in bytes from _estack down
    uint32_t stack_high_water;        // deepest stack usage observed (SP low-water + sentinel)

    // Empirical DTCM stack sentinel. memory_info_init() paints the unused
    // DTCM runway below the early setup() stack pointer. Later snapshots scan
    // that runway to discover the deepest address the descending stack has
    // actually touched, even if no report sampled SP at the deepest moment.
    bool     stack_paint_compiled_enabled; // true when bounded diagnostic paint is compiled in
    bool     stack_paint_enabled;     // true if the DTCM sentinel was installed
    bool     stack_paint_overrun;     // true if the painted runway was fully consumed
    uint32_t stack_paint_pattern;     // sentinel word value
    uint32_t stack_paint_guard_bytes; // bytes intentionally left below init SP
    uint32_t stack_paint_static_guard_bytes; // bytes left above static DTCM boundary
    uint32_t stack_paint_window_bytes;// maximum bounded diagnostic paint window
    uint32_t stack_paint_skip_reason; // scalar reason paint was not installed
    const char* stack_paint_skip_reason_name; // stable literal reason string
    uint32_t stack_paint_start;       // first painted DTCM address
    uint32_t stack_paint_end;         // one-past-last painted DTCM address
    uint32_t stack_paint_bytes;       // paint_end - paint_start
    uint32_t stack_paint_used;        // conservative high-end consumption of painted runway
    uint32_t stack_paint_unused;      // untouched low-end runway above static DTCM
    uint32_t stack_paint_clobbered;   // exact number of non-sentinel bytes found
    uint32_t stack_paint_deepest_addr;// lowest painted address observed non-sentinel, 0 if none

    uint32_t stack_free_current;      // current SP - static_end, 0 if already overlapping
    uint32_t stack_free_high_water;   // minimum observed free runway above static DTCM
    uint32_t stack_collision_warn_bytes; // warning threshold for stack_free_high_water
    bool     stack_collision_risk;    // high-water runway is at/below warning threshold

    // --------------------------------------------------------
    // Heap (RAM2 / OCRAM)
    // --------------------------------------------------------

    uint32_t heap_total;              // RAM2 total size
    uint32_t heap_arena;              // mallinfo().arena — heap arena committed by sbrk
    uint32_t heap_used;               // mallinfo().uordblks — bytes in active allocations
    uint32_t heap_free_internal;      // mallinfo().fordblks — freed bytes trapped inside arena
    uint32_t heap_free_above;         // bytes between __brkval and _heap_end (uncommitted)
    uint32_t heap_free_total;         // free_internal + free_above

    uint32_t heap_arena_high_water;   // highest heap_arena ever observed

    // --------------------------------------------------------
    // Derived health indicators
    // --------------------------------------------------------

    uint32_t heap_fragmentation_pct;  // (free_internal * 100) / heap_arena, 0 if arena == 0
    uint32_t stack_usage_pct;         // (stack_high_water * 100) / dtcm_stack_avail
    uint32_t stack_free_pct;          // (stack_free_high_water * 100) / dtcm_stack_avail
    bool     heap_growing;            // true if heap_arena > previous snapshot (possible leak)
};


// ============================================================================
// Memory health court — always-on rule-based verdict
// ============================================================================
//
// memory_info_t is the raw observation surface.  memory_health_t is the compact
// policy verdict consumed by SYSTEM's recurring watchdog.  The court also
// incorporates Payload allocator/object-integrity instrumentation because those
// failures are direct evidence that the memory-backed execution environment is
// no longer trustworthy.
//
// The health verdict is sticky for the life of the boot.  Once a hard anomaly
// is observed, later audits remain ANOMALY even if the immediate condition is no
// longer visible.  SYSTEM separately owns the one-event-per-boot emission latch.

enum class memory_health_status_t : uint8_t {
    UNASSESSED = 0,
    NOMINAL    = 1,
    ANOMALY    = 2,
};

enum class memory_health_reason_t : uint8_t {
    NONE                         = 0,
    NOT_INITIALIZED              = 1,
    DTCM_MAP_INVALID             = 2,
    STACK_PAINT_UNAVAILABLE      = 3,
    STACK_PAINT_OVERRUN          = 4,
    STACK_RUNWAY_CRITICAL        = 5,
    HEAP_MAP_INVALID             = 6,
    HEAP_FREE_CRITICAL           = 7,
    PAYLOAD_SELF_OK_FAIL         = 8,
    PAYLOAD_INTEGRITY_FAIL       = 9,
    PAYLOAD_STRING_POINTER_FAULT = 10,
    PAYLOAD_INVALID_KIND         = 11,
    PAYLOAD_ENTRY_ALLOC_FAIL     = 12,
    PAYLOAD_ARENA_ALLOC_FAIL     = 13,
    PAYLOAD_ENTRY_OVERFLOW       = 14,
    PAYLOAD_SERIALIZE_OVERFLOW   = 15,
    PAYLOAD_TO_JSON_FAIL         = 16,
    PAYLOAD_LIFETIME_MISMATCH    = 17,
};

struct memory_health_t {
    memory_health_status_t status;
    memory_health_reason_t primary_reason;
    uint32_t active_reason_mask;
    uint32_t latched_reason_mask;
    bool     latched;

    uint32_t audit_count;
    uint32_t anomaly_audit_count;
    uint32_t first_anomaly_audit;
    uint32_t last_anomaly_audit;
    uint32_t heap_free_critical_bytes;

    // Compact current evidence.  Full raw detail remains available through
    // SYSTEM.MEMORY_INFO and SYSTEM.PAYLOAD_INFO.
    bool     memory_initialized;
    uint32_t dtcm_total;
    uint32_t dtcm_static;
    uint32_t dtcm_stack_avail;
    uint32_t stack_high_water;
    uint32_t stack_free_high_water;
    bool     stack_paint_compiled_enabled;
    bool     stack_paint_enabled;
    bool     stack_paint_overrun;
    bool     stack_collision_risk;

    uint32_t heap_total;
    uint32_t heap_arena;
    uint32_t heap_used;
    uint32_t heap_free_total;
    uint32_t heap_arena_high_water;

    uint32_t payload_instances_constructed;
    uint32_t payload_instances_destroyed;
    uint32_t payload_alive_now;
    bool     payload_lifetime_mismatch;
    uint32_t payload_entry_alloc_fail;
    uint32_t payload_arena_alloc_fail;
    uint32_t payload_entry_overflow;
    uint32_t payload_serialize_overflow;
    uint32_t payload_to_json_fail;
    uint32_t payload_integrity_fail;
    uint32_t payload_invalid_kind;
    uint32_t payload_string_pointer_fault;
    uint32_t payload_self_ok_fail;
    uint32_t payload_entry_high_water;
    uint32_t payload_max_entries;
    uint32_t payload_arena_high_water;
    uint32_t payload_arena_max;
    uint32_t payload_last_error_code;
    uint32_t payload_last_string_pointer_fault_reason;
    uint32_t payload_last_self_ok_fail_reason;
};

// ============================================================================
// API
// ============================================================================

// Call once in setup(), BEFORE any allocations if possible.
// Paints the unused DTCM stack region with a sentinel pattern
// for high-water mark detection.
void memory_info_init();

// Populate a snapshot. Safe to call from any context.
// Does not allocate. Does not emit. Read-only.
void memory_info_get(memory_info_t* out);

// Run one complete memory/Payload health court.  Does not allocate or emit.
// The returned ANOMALY verdict is sticky for the life of the boot.
void memory_info_audit(memory_health_t* out);

// Return the most recent court verdict without running a new audit.
void memory_info_get_health(memory_health_t* out);

const char* memory_health_status_name(memory_health_status_t status);
const char* memory_health_reason_name(memory_health_reason_t reason);