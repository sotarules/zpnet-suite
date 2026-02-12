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

    uint32_t stack_current;           // current stack depth in bytes
    uint32_t stack_high_water;        // deepest stack usage observed (via sentinel painting)

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
    bool     heap_growing;            // true if heap_arena > previous snapshot (possible leak)
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