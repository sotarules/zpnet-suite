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

// Empirical stack-paint runway.
//
// Safety doctrine after the first broad-paint experiment:
//   * ordinary builds are map/read-only by default; no DTCM sentinel writes
//   * diagnostic paint, when explicitly enabled, is bounded to a small window
//     below the early setup() stack pointer rather than the entire _ebss..SP gap
//   * a larger guard is left below the setup-time SP so early USB/SysTick/IRQ
//     frames cannot race the paint loop
//
// This keeps the valuable fields/report shape while preventing memory_info_init()
// from becoming an early-boot memory mutator in the science baseline.
static constexpr bool     MEMORY_STACK_PAINT_COMPILED_ENABLED = true;
static constexpr uint32_t MEMORY_STACK_PAINT_PATTERN = 0xA5A5A5A5UL;

// Keep the diagnostic paint well away from both ends of the stack runway.
// The top guard protects the setup-time call chain plus any early interrupt
// frames.  The static guard keeps the paint away from the linker-reported
// BSS/static boundary, which faulted during the first broad-paint experiment.
// The bounded window gives us a useful high-water tripwire without turning
// memory_info_init() into a bulk DTCM scrubber.
static constexpr uint32_t MEMORY_STACK_PAINT_GUARD_BYTES = 8192UL;
static constexpr uint32_t MEMORY_STACK_PAINT_STATIC_GUARD_BYTES = 4096UL;
static constexpr uint32_t MEMORY_STACK_PAINT_WINDOW_BYTES = 8192UL;
static constexpr uint32_t MEMORY_STACK_COLLISION_WARN_BYTES = 4096UL;

static constexpr uint32_t MEMORY_STACK_PAINT_SKIP_NONE = 0UL;
static constexpr uint32_t MEMORY_STACK_PAINT_SKIP_COMPILED_DISABLED = 1UL;
static constexpr uint32_t MEMORY_STACK_PAINT_SKIP_BAD_DTCM_MAP = 2UL;
static constexpr uint32_t MEMORY_STACK_PAINT_SKIP_STACK_OUT_OF_RANGE = 3UL;
static constexpr uint32_t MEMORY_STACK_PAINT_SKIP_NO_ROOM_AFTER_GUARD = 4UL;
static constexpr uint32_t MEMORY_STACK_PAINT_SKIP_WINDOW_EMPTY = 5UL;
static constexpr uint32_t MEMORY_STACK_PAINT_SKIP_NO_ROOM_AFTER_STATIC_GUARD = 6UL;

static bool     _stack_paint_enabled = false;
static uint32_t _stack_paint_start   = 0;
static uint32_t _stack_paint_end     = 0;
static uint32_t _stack_paint_bytes   = 0;
static uint32_t _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_NONE;

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

static inline uint32_t memory_info_primask_save(void) {
    uint32_t primask = 0;
    __asm__ volatile ("mrs %0, primask" : "=r" (primask) :: "memory");
    __asm__ volatile ("cpsid i" ::: "memory");
    __asm__ volatile ("dmb" ::: "memory");
    return primask;
}

static inline void memory_info_primask_restore(uint32_t primask) {
    __asm__ volatile ("dmb" ::: "memory");
    __asm__ volatile ("msr primask, %0" :: "r" (primask) : "memory");
}

static const char* memory_info_stack_paint_skip_reason_name(uint32_t reason) {
    switch (reason) {
        case MEMORY_STACK_PAINT_SKIP_NONE:
            return "NONE";
        case MEMORY_STACK_PAINT_SKIP_COMPILED_DISABLED:
            return "COMPILED_DISABLED";
        case MEMORY_STACK_PAINT_SKIP_BAD_DTCM_MAP:
            return "BAD_DTCM_MAP";
        case MEMORY_STACK_PAINT_SKIP_STACK_OUT_OF_RANGE:
            return "STACK_OUT_OF_RANGE";
        case MEMORY_STACK_PAINT_SKIP_NO_ROOM_AFTER_GUARD:
            return "NO_ROOM_AFTER_GUARD";
        case MEMORY_STACK_PAINT_SKIP_WINDOW_EMPTY:
            return "WINDOW_EMPTY";
        case MEMORY_STACK_PAINT_SKIP_NO_ROOM_AFTER_STATIC_GUARD:
            return "NO_ROOM_AFTER_STATIC_GUARD";
        default:
            return "UNKNOWN";
    }
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

    _stack_paint_enabled = false;
    _stack_paint_start = 0;
    _stack_paint_end = 0;
    _stack_paint_bytes = 0;
    _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_NONE;

    if (!MEMORY_STACK_PAINT_COMPILED_ENABLED) {
        _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_COMPILED_DISABLED;
    } else if (!(dtcm_end > dtcm_start &&
                 static_end >= dtcm_start &&
                 static_end < dtcm_end)) {
        _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_BAD_DTCM_MAP;
    } else if (!(sp > static_end && sp <= dtcm_end)) {
        _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_STACK_OUT_OF_RANGE;
    } else if (sp <= static_end + MEMORY_STACK_PAINT_GUARD_BYTES) {
        _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_NO_ROOM_AFTER_GUARD;
    } else {
        const uint32_t static_guard_end = static_end + MEMORY_STACK_PAINT_STATIC_GUARD_BYTES;
        if (static_guard_end <= static_end || static_guard_end >= dtcm_end) {
            _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_NO_ROOM_AFTER_STATIC_GUARD;
        } else {
            // Bounded diagnostic window: paint only a small middle slice of the
            // stack runway.  It is below setup-time SP by a generous guard and
            // above static DTCM by a separate guard.  This gives a useful
            // high-water tripwire without touching the risky _ebss boundary or
            // the live setup()/interrupt stack neighborhood.
            const uint32_t paint_end = memory_info_align_down4(
                sp - MEMORY_STACK_PAINT_GUARD_BYTES);
            const uint32_t min_start = memory_info_align_up4(static_guard_end);
            uint32_t paint_start = paint_end;
            if (paint_end > MEMORY_STACK_PAINT_WINDOW_BYTES) {
                paint_start = paint_end - MEMORY_STACK_PAINT_WINDOW_BYTES;
            }
            if (paint_start < min_start) {
                paint_start = min_start;
            }
            paint_start = memory_info_align_up4(paint_start);

            if (paint_end > paint_start && paint_end <= dtcm_end) {
                const uint32_t saved_primask = memory_info_primask_save();
                volatile uint32_t* word = (volatile uint32_t*)paint_start;
                volatile uint32_t* end  = (volatile uint32_t*)paint_end;
                while (word < end) {
                    *word++ = MEMORY_STACK_PAINT_PATTERN;
                }
                memory_info_primask_restore(saved_primask);

                _stack_paint_enabled = true;
                _stack_paint_start = paint_start;
                _stack_paint_end = paint_end;
                _stack_paint_bytes = paint_end - paint_start;
                _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_NONE;
            } else {
                _stack_paint_skip_reason = MEMORY_STACK_PAINT_SKIP_WINDOW_EMPTY;
            }
        }
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

    out->stack_paint_compiled_enabled = MEMORY_STACK_PAINT_COMPILED_ENABLED;
    out->stack_paint_enabled = _stack_paint_enabled;
    out->stack_paint_overrun =
        _stack_paint_enabled && painted_deepest_addr == _stack_paint_start;
    out->stack_paint_pattern = MEMORY_STACK_PAINT_PATTERN;
    out->stack_paint_guard_bytes = MEMORY_STACK_PAINT_GUARD_BYTES;
    out->stack_paint_static_guard_bytes = MEMORY_STACK_PAINT_STATIC_GUARD_BYTES;
    out->stack_paint_window_bytes = MEMORY_STACK_PAINT_WINDOW_BYTES;
    out->stack_paint_skip_reason = _stack_paint_skip_reason;
    out->stack_paint_skip_reason_name =
        memory_info_stack_paint_skip_reason_name(_stack_paint_skip_reason);
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
        out->stack_paint_overrun ||
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