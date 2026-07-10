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

// Stack pointer low-water mark (lowest SP ever observed = deepest sampled stack)
static uint32_t _stack_sp_min      = 0xFFFFFFFF;
static uint32_t _stack_init_sp     = 0;

// Empirical stack-paint runway.  Painted words live only in the unused DTCM
// gap between static data/BSS and the early setup() stack pointer.  The stack
// grows downward from _estack, so any non-sentinel word in this runway is
// evidence that the stack touched that address at some point after init.
static constexpr uint32_t MEMORY_STACK_PAINT_PATTERN = 0xA5A5A5A5UL;
static constexpr uint32_t MEMORY_STACK_PAINT_GUARD_BYTES = 1024UL;
static constexpr uint32_t MEMORY_STACK_COLLISION_WARN_BYTES = 4096UL;

static bool     _stack_paint_enabled = false;
static uint32_t _stack_paint_start   = 0;
static uint32_t _stack_paint_end     = 0;
static uint32_t _stack_paint_bytes   = 0;

// Heap arena tracking
static uint32_t _heap_arena_hwm    = 0;
static uint32_t _heap_arena_prev   = 0;

// ============================================================================
// Address helpers
// ============================================================================

static inline uint32_t memory_info_stack_pointer() {
    register uint32_t sp asm("sp");
    return sp;
}

static inline uint32_t memory_info_align_up4(uint32_t value) {
    return (value + 3UL) & ~3UL;
}

static inline uint32_t memory_info_align_down4(uint32_t value) {
    return value & ~3UL;
}

static inline uint32_t memory_info_sub_or_zero(uint32_t high, uint32_t low) {
    return (high > low) ? (high - low) : 0UL;
}

// ============================================================================
// Init — safe sentinel installation
// ============================================================================

void memory_info_init() {

    // Record initial stack pointer as baseline.  This is a sampled witness; the
    // painted sentinel below provides the historical high-water witness.
    const uint32_t sp = memory_info_stack_pointer();
    _stack_init_sp = sp;
    _stack_sp_min = sp;

    const uint32_t dtcm_start = (uint32_t)&_sdata;
    const uint32_t dtcm_end   = (uint32_t)&_estack;
    const uint32_t static_end = (uint32_t)&_ebss;

    const uint32_t paint_start = memory_info_align_up4(static_end);
    uint32_t paint_end = 0;
    if (sp > MEMORY_STACK_PAINT_GUARD_BYTES) {
        paint_end = memory_info_align_down4(sp - MEMORY_STACK_PAINT_GUARD_BYTES);
    }

    _stack_paint_enabled = false;
    _stack_paint_start = 0;
    _stack_paint_end = 0;
    _stack_paint_bytes = 0;

    if (dtcm_end > dtcm_start &&
        paint_start >= dtcm_start &&
        paint_end > paint_start &&
        paint_end <= dtcm_end) {
        volatile uint32_t* word = (volatile uint32_t*)paint_start;
        volatile uint32_t* end  = (volatile uint32_t*)paint_end;
        while (word < end) {
            *word++ = MEMORY_STACK_PAINT_PATTERN;
        }

        _stack_paint_enabled = true;
        _stack_paint_start = paint_start;
        _stack_paint_end = paint_end;
        _stack_paint_bytes = paint_end - paint_start;
    }

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
    out->dtcm_total = memory_info_sub_or_zero(dtcm_end, dtcm_start);

    // Static usage: initialized data + BSS
    uint32_t static_end = (uint32_t)&_ebss;
    out->dtcm_static = memory_info_sub_or_zero(static_end, dtcm_start);

    // Available for stack = total DTCM - static
    out->dtcm_stack_avail = memory_info_sub_or_zero(dtcm_end, static_end);

    out->initialized = _initialized;
    out->stack_init_sp = _stack_init_sp;

    // Current stack depth (SP is below _estack, distance = depth)
    const uint32_t sp = memory_info_stack_pointer();
    out->stack_current = memory_info_sub_or_zero(dtcm_end, sp);

    // Update stack pointer low-water mark
    // (lowest SP = deepest sampled stack usage ever observed)
    if (sp < _stack_sp_min) {
        _stack_sp_min = sp;
    }

    out->stack_free_current = memory_info_sub_or_zero(sp, static_end);

    // Empirical high-water scan.  The stack grows downward from _estack and
    // consumes the high end of the painted runway first.  We scan the full
    // runway and use the lowest non-sentinel address as a conservative
    // deepest-touched stack address.  Exact clobbered bytes are also reported
    // so sparse stack writes remain visible.
    uint32_t painted_deepest_addr = 0;
    uint32_t painted_clobbered_bytes = 0;
    if (_stack_paint_enabled) {
        volatile const uint32_t* word =
            (volatile const uint32_t*)_stack_paint_start;
        volatile const uint32_t* end =
            (volatile const uint32_t*)_stack_paint_end;
        while (word < end) {
            if (*word != MEMORY_STACK_PAINT_PATTERN) {
                if (painted_deepest_addr == 0) {
                    painted_deepest_addr = (uint32_t)(uintptr_t)word;
                }
                painted_clobbered_bytes += sizeof(uint32_t);
            }
            ++word;
        }
    }

    uint32_t deepest_stack_addr = _stack_sp_min;
    if (painted_deepest_addr != 0 && painted_deepest_addr < deepest_stack_addr) {
        deepest_stack_addr = painted_deepest_addr;
    }
    if (deepest_stack_addr == 0 || deepest_stack_addr > dtcm_end) {
        deepest_stack_addr = sp;
    }

    // High-water = deepest stack ever, measured from _estack down
    out->stack_high_water = memory_info_sub_or_zero(dtcm_end, deepest_stack_addr);

    out->stack_paint_enabled = _stack_paint_enabled;
    out->stack_paint_overrun =
        _stack_paint_enabled && painted_deepest_addr == _stack_paint_start;
    out->stack_paint_pattern = MEMORY_STACK_PAINT_PATTERN;
    out->stack_paint_guard_bytes = MEMORY_STACK_PAINT_GUARD_BYTES;
    out->stack_paint_start = _stack_paint_start;
    out->stack_paint_end = _stack_paint_end;
    out->stack_paint_bytes = _stack_paint_bytes;
    out->stack_paint_deepest_addr = painted_deepest_addr;
    out->stack_paint_clobbered = painted_clobbered_bytes;
    out->stack_paint_used =
        (_stack_paint_enabled && painted_deepest_addr != 0)
            ? memory_info_sub_or_zero(_stack_paint_end, painted_deepest_addr)
            : 0UL;
    out->stack_paint_unused =
        (_stack_paint_enabled && painted_deepest_addr != 0)
            ? memory_info_sub_or_zero(painted_deepest_addr, _stack_paint_start)
            : _stack_paint_bytes;

    out->stack_free_high_water =
        memory_info_sub_or_zero(deepest_stack_addr, static_end);
    out->stack_collision_warn_bytes = MEMORY_STACK_COLLISION_WARN_BYTES;
    out->stack_collision_risk =
        out->stack_free_high_water <= MEMORY_STACK_COLLISION_WARN_BYTES;

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
        out->stack_free_pct =
            (out->stack_free_high_water * 100) / out->dtcm_stack_avail;
    } else {
        out->stack_usage_pct = 100;
        out->stack_free_pct = 0;
    }
}