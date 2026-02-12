#include "memory_info.h"

#include <Arduino.h>
#include <malloc.h>
#include <string.h>

// ============================================================================
// Linker-defined symbols (from imxrt1062_t41.ld)
// ============================================================================

extern unsigned long _sdata;        // start of initialized data in DTCM
extern unsigned long _edata;        // end of initialized data
extern unsigned long _sbss;         // start of BSS
extern unsigned long _ebss;         // end of BSS
extern unsigned long _estack;       // top of DTCM (stack starts here, grows down)

extern unsigned long _heap_start;   // start of heap in RAM2
extern unsigned long _heap_end;     // end of RAM2

extern "C" char* __brkval;          // current top of heap (sbrk frontier)

// ============================================================================
// Module state
// ============================================================================

static bool     _initialized       = false;

// Stack pointer low-water mark (lowest SP ever observed = deepest stack)
static uint32_t _stack_sp_min      = 0xFFFFFFFF;

// Heap arena tracking
static uint32_t _heap_arena_hwm    = 0;
static uint32_t _heap_arena_prev   = 0;

// ============================================================================
// Init — safe, non-destructive
// ============================================================================

void memory_info_init() {

    // Record initial stack pointer as baseline
    register uint32_t sp asm("sp");
    _stack_sp_min = sp;

    // Initialize heap tracking
    struct mallinfo mi = mallinfo();
    _heap_arena_hwm  = mi.arena;
    _heap_arena_prev = mi.arena;

    _initialized = true;
}

// ============================================================================
// Snapshot
// ============================================================================

void memory_info_get(memory_info_t* out) {
    if (!out) return;

    memset(out, 0, sizeof(memory_info_t));

    // --------------------------------------------------------
    // DTCM / Stack
    // --------------------------------------------------------

    // DTCM spans from _sdata to _estack
    uint32_t dtcm_start = (uint32_t)&_sdata;
    uint32_t dtcm_end   = (uint32_t)&_estack;
    out->dtcm_total = dtcm_end - dtcm_start;

    // Static usage: initialized data + BSS
    uint32_t static_end = (uint32_t)&_ebss;
    out->dtcm_static = static_end - dtcm_start;

    // Available for stack = total DTCM - static
    out->dtcm_stack_avail = out->dtcm_total - out->dtcm_static;

    // Current stack depth (SP is below _estack, distance = depth)
    register uint32_t sp asm("sp");
    out->stack_current = dtcm_end - sp;

    // Update stack pointer low-water mark
    // (lowest SP = deepest stack usage ever observed)
    if (sp < _stack_sp_min) {
        _stack_sp_min = sp;
    }

    // High-water = deepest stack ever, measured from _estack down
    out->stack_high_water = dtcm_end - _stack_sp_min;

    // --------------------------------------------------------
    // Heap (RAM2 / OCRAM)
    // --------------------------------------------------------

    uint32_t ram2_start = (uint32_t)&_heap_start;
    uint32_t ram2_end   = (uint32_t)&_heap_end;
    out->heap_total = ram2_end - ram2_start;

    struct mallinfo mi = mallinfo();

    out->heap_arena         = mi.arena;       // committed by sbrk
    out->heap_used          = mi.uordblks;    // in active allocations
    out->heap_free_internal = mi.fordblks;    // freed but trapped

    // Free space above sbrk frontier
    uint32_t brkval = (uint32_t)__brkval;
    if (brkval == 0) brkval = ram2_start;     // no allocations yet
    out->heap_free_above = (brkval < ram2_end) ? (ram2_end - brkval) : 0;

    out->heap_free_total = out->heap_free_internal + out->heap_free_above;

    // --------------------------------------------------------
    // High-water and trend tracking
    // --------------------------------------------------------

    if (mi.arena > _heap_arena_hwm) {
        _heap_arena_hwm = mi.arena;
    }
    out->heap_arena_high_water = _heap_arena_hwm;

    // Leak detection: is the heap arena still growing?
    out->heap_growing = (mi.arena > _heap_arena_prev);
    _heap_arena_prev = mi.arena;

    // --------------------------------------------------------
    // Derived health indicators
    // --------------------------------------------------------

    if (mi.arena > 0) {
        out->heap_fragmentation_pct =
            (out->heap_free_internal * 100) / mi.arena;
    } else {
        out->heap_fragmentation_pct = 0;
    }

    if (out->dtcm_stack_avail > 0) {
        out->stack_usage_pct =
            (out->stack_high_water * 100) / out->dtcm_stack_avail;
    } else {
        out->stack_usage_pct = 100;
    }
}