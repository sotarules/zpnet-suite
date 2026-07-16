#include "memory_info.h"
#include "payload.h"
#include "crash_forensics.h"

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
// Memory health court state
// ============================================================================

static constexpr uint32_t MEMORY_HEAP_FREE_CRITICAL_BYTES = 32768UL;

// Keep recurring audit working state in ordinary RAM1/DTCM.  These objects are
// current-boot mutable state, not retained evidence, and therefore should use
// normal C/C++ initialization semantics and deterministic tightly-coupled RAM.
// The audit trace banks below are also live audit working state in this
// deterministic-memory baseline and therefore remain in RAM1.
static memory_info_t   _memory_health_memory_scratch = {};
static payload_info_t  _memory_health_payload_scratch = {};
static memory_health_t _memory_health = {};


// ============================================================================
// Memory-audit flight recorder
// ============================================================================
//
// The audit is itself part of the trusted platform.  Both compatibility banks
// use ordinary RAM1 so the recurring audit path never crosses cached RAM2.
// The retained-bank API remains stable, but RAM1 startup initialization means
// it does not preserve evidence across a reboot in this baseline.

static constexpr uint32_t MEMORY_AUDIT_TRACE_MAGIC = 0x4D414631UL;  // 'MAF1'
static constexpr uint32_t MEMORY_AUDIT_TRACE_SCHEMA_VERSION = 1U;

struct alignas(32) memory_audit_trace_bank_t {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t schema_version;
    uint32_t capacity;
    uint32_t reserved[4];
    memory_audit_trace_entry_t entries[MEMORY_AUDIT_TRACE_ENTRIES];
};

static_assert((sizeof(memory_audit_trace_bank_t) % 32U) == 0U,
              "memory audit trace bank must occupy complete cache lines");

static memory_audit_trace_bank_t _memory_audit_trace_live = {};
static memory_audit_trace_bank_t _memory_audit_trace_retained = {};
static bool _memory_audit_trace_boot_latched = false;  // ordinary BSS
static volatile bool _memory_audit_trace_active = false;  // ordinary BSS
static uint32_t _memory_audit_trace_next_sequence = 0U;

static inline void memory_audit_trace_dmb(void) {
    __asm__ volatile("dmb" ::: "memory");
}

static inline uint32_t memory_audit_trace_ipsr(void) {
    uint32_t ipsr = 0U;
#if defined(__arm__)
    __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
#endif
    return ipsr & 0x1FFU;
}

static inline uint32_t memory_audit_trace_irq_lock(void) {
    uint32_t primask = 0U;
#if defined(__arm__)
    __asm__ volatile("mrs %0, primask" : "=r"(primask) :: "memory");
    __asm__ volatile("cpsid i" ::: "memory");
#endif
    return primask;
}

static inline void memory_audit_trace_irq_unlock(uint32_t primask) {
#if defined(__arm__)
    __asm__ volatile("msr primask, %0" :: "r"(primask) : "memory");
#else
    (void)primask;
#endif
}

static bool memory_audit_trace_bank_valid(
    const memory_audit_trace_bank_t& bank) {
    return bank.magic == MEMORY_AUDIT_TRACE_MAGIC &&
           (bank.magic ^ bank.magic_inv) == 0xFFFFFFFFUL &&
           bank.schema_version == MEMORY_AUDIT_TRACE_SCHEMA_VERSION &&
           bank.capacity == MEMORY_AUDIT_TRACE_ENTRIES;
}

static bool memory_audit_trace_entry_valid(
    const memory_audit_trace_entry_t& entry) {
    return entry.sequence != 0U &&
           (entry.sequence ^ entry.sequence_inv) == 0xFFFFFFFFUL;
}

static void memory_audit_trace_initialize_live(void) {
    memset((void*)&_memory_audit_trace_live, 0,
           sizeof(_memory_audit_trace_live));
    _memory_audit_trace_live.schema_version =
        MEMORY_AUDIT_TRACE_SCHEMA_VERSION;
    _memory_audit_trace_live.capacity = MEMORY_AUDIT_TRACE_ENTRIES;
    _memory_audit_trace_live.magic_inv = ~MEMORY_AUDIT_TRACE_MAGIC;
    _memory_audit_trace_live.magic = MEMORY_AUDIT_TRACE_MAGIC;
    _memory_audit_trace_next_sequence = 0U;
}

static void memory_audit_trace_boot_latch(void) {
    if (_memory_audit_trace_boot_latched) return;
    _memory_audit_trace_boot_latched = true;

    // Both banks are ordinary RAM1 and were zeroed by startup.  Keep the
    // compatibility retained surface empty and initialize the live recorder.
    memset((void*)&_memory_audit_trace_retained, 0,
           sizeof(_memory_audit_trace_retained));
    memory_audit_trace_initialize_live();
}

static void memory_audit_trace_record(memory_audit_trace_stage_t stage,
                                      uint32_t value0,
                                      uint32_t value1) {
    memory_audit_trace_boot_latch();

    uint32_t sequence = 0U;
    const uint32_t saved_primask = memory_audit_trace_irq_lock();
    sequence = _memory_audit_trace_next_sequence + 1U;
    if (sequence == 0U) sequence = 1U;
    _memory_audit_trace_next_sequence = sequence;
    memory_audit_trace_irq_unlock(saved_primask);

    memory_audit_trace_entry_t& entry =
        _memory_audit_trace_live.entries[
            (sequence - 1U) % MEMORY_AUDIT_TRACE_ENTRIES];

    entry.sequence = 0U;
    entry.sequence_inv = 0U;
    memory_audit_trace_dmb();

    entry.stage = (uint32_t)stage;
    entry.dwt = ARM_DWT_CYCCNT;
    entry.ipsr = memory_audit_trace_ipsr();
    entry.value0 = value0;
    entry.value1 = value1;
    entry.reserved = 0U;

    entry.sequence_inv = ~sequence;
    memory_audit_trace_dmb();
    entry.sequence = sequence;  // commit last
    memory_audit_trace_dmb();
}

static void memory_audit_trace_snapshot_bank(
    const memory_audit_trace_bank_t& bank,
    memory_audit_trace_bank_snapshot_t* out) {
    memset((void*)out, 0, sizeof(*out));
    out->valid = memory_audit_trace_bank_valid(bank);
    if (!out->valid) return;

    for (uint32_t i = 0U; i < MEMORY_AUDIT_TRACE_ENTRIES; ++i) {
        const volatile memory_audit_trace_entry_t* source = &bank.entries[i];
        const uint32_t sequence_before = source->sequence;
        const uint32_t sequence_inv_before = source->sequence_inv;
        if (sequence_before == 0U ||
            (sequence_before ^ sequence_inv_before) != 0xFFFFFFFFUL) {
            continue;
        }

        const memory_audit_trace_entry_t candidate = bank.entries[i];
        memory_audit_trace_dmb();
        if (source->sequence != sequence_before ||
            source->sequence_inv != sequence_inv_before ||
            !memory_audit_trace_entry_valid(candidate)) {
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

void memory_audit_trace_snapshot(memory_audit_trace_snapshot_t* out) {
    if (!out) return;
    memory_audit_trace_boot_latch();
    memset((void*)out, 0, sizeof(*out));
    memory_audit_trace_snapshot_bank(_memory_audit_trace_live, &out->live);
    memory_audit_trace_snapshot_bank(_memory_audit_trace_retained,
                                     &out->retained);
}

void memory_audit_trace_clear_retained(void) {
    memset((void*)&_memory_audit_trace_retained, 0,
           sizeof(_memory_audit_trace_retained));
}

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

    // Latch the previous boot's final audit transcript before any current-boot
    // audit can reuse the live ring.
    memory_audit_trace_boot_latch();

    // Establish an explicit current-boot audit baseline.  These objects now
    // use ordinary RAM1 initialization; the assignments keep re-init behavior
    // deterministic without relying on retained RAM2 semantics.
    _memory_health_memory_scratch = memory_info_t{};
    _memory_health_payload_scratch = payload_info_t{};
    _memory_health = memory_health_t{};

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

    _memory_health.status = memory_health_status_t::UNASSESSED;
    _memory_health.primary_reason = memory_health_reason_t::NONE;
    _memory_health.heap_free_critical_bytes = MEMORY_HEAP_FREE_CRITICAL_BYTES;

    _initialized = true;
}

// ============================================================================
// Snapshot
// ============================================================================

void memory_info_get(memory_info_t* out) {
    if (!out) return;

    // Only the foreground court invocation owns these internal breadcrumbs.
    // An unrelated ISR-side snapshot that preempts the audit remains a normal
    // read-only memory_info_get() call and cannot become a recorder writer.
    const bool audit_trace =
        _memory_audit_trace_active && memory_audit_trace_ipsr() == 0U;

    if (audit_trace) {
        memory_audit_trace_record(
            memory_audit_trace_stage_t::MEMORY_GET_ENTER,
            memory_info_stack_pointer(),
            _stack_sp_min);
    }

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

    if (audit_trace) {
        memory_audit_trace_record(
            memory_audit_trace_stage_t::STACK_SNAPSHOT,
            out->stack_free_current,
            out->stack_current);
        memory_audit_trace_record(
            memory_audit_trace_stage_t::STACK_PAINT_SCAN,
            _stack_paint_start,
            _stack_paint_end);
    }

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

    if (audit_trace) {
        memory_audit_trace_record(
            memory_audit_trace_stage_t::HEAP_INFO,
            ram2_start,
            ram2_end);
    }
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

    if (audit_trace) {
        memory_audit_trace_record(
            memory_audit_trace_stage_t::MEMORY_GET_EXIT,
            out->stack_free_high_water,
            out->heap_free_total);
    }
}

// ============================================================================
// Memory health court
// ============================================================================

static uint32_t memory_health_reason_bit(memory_health_reason_t reason) {
    const uint32_t id = (uint32_t)reason;
    if (id == 0U || id > 32U) return 0U;
    return 1UL << (id - 1U);
}

static void memory_health_add_reason(uint32_t& mask,
                                     memory_health_reason_t& primary,
                                     memory_health_reason_t reason) {
    mask |= memory_health_reason_bit(reason);
    if (primary == memory_health_reason_t::NONE) {
        primary = reason;
    }
}

const char* memory_health_status_name(memory_health_status_t status) {
    switch (status) {
        case memory_health_status_t::NOMINAL:    return "NOMINAL";
        case memory_health_status_t::ANOMALY:    return "ANOMALY";
        case memory_health_status_t::UNASSESSED:
        default:                                 return "UNASSESSED";
    }
}

const char* memory_health_reason_name(memory_health_reason_t reason) {
    switch (reason) {
        case memory_health_reason_t::NONE:                         return "NONE";
        case memory_health_reason_t::NOT_INITIALIZED:              return "NOT_INITIALIZED";
        case memory_health_reason_t::DTCM_MAP_INVALID:             return "DTCM_MAP_INVALID";
        case memory_health_reason_t::STACK_PAINT_UNAVAILABLE:      return "STACK_PAINT_UNAVAILABLE";
        case memory_health_reason_t::STACK_PAINT_OVERRUN:          return "STACK_PAINT_OVERRUN";
        case memory_health_reason_t::STACK_RUNWAY_CRITICAL:        return "STACK_RUNWAY_CRITICAL";
        case memory_health_reason_t::HEAP_MAP_INVALID:             return "HEAP_MAP_INVALID";
        case memory_health_reason_t::HEAP_FREE_CRITICAL:           return "HEAP_FREE_CRITICAL";
        case memory_health_reason_t::PAYLOAD_SELF_OK_FAIL:         return "PAYLOAD_SELF_OK_FAIL";
        case memory_health_reason_t::PAYLOAD_INTEGRITY_FAIL:       return "PAYLOAD_INTEGRITY_FAIL";
        case memory_health_reason_t::PAYLOAD_STRING_POINTER_FAULT: return "PAYLOAD_STRING_POINTER_FAULT";
        case memory_health_reason_t::PAYLOAD_INVALID_KIND:         return "PAYLOAD_INVALID_KIND";
        case memory_health_reason_t::PAYLOAD_ENTRY_ALLOC_FAIL:     return "PAYLOAD_ENTRY_ALLOC_FAIL";
        case memory_health_reason_t::PAYLOAD_ARENA_ALLOC_FAIL:     return "PAYLOAD_ARENA_ALLOC_FAIL";
        case memory_health_reason_t::PAYLOAD_ENTRY_OVERFLOW:       return "PAYLOAD_ENTRY_OVERFLOW";
        case memory_health_reason_t::PAYLOAD_SERIALIZE_OVERFLOW:   return "PAYLOAD_SERIALIZE_OVERFLOW";
        case memory_health_reason_t::PAYLOAD_TO_JSON_FAIL:         return "PAYLOAD_TO_JSON_FAIL";
        case memory_health_reason_t::PAYLOAD_LIFETIME_MISMATCH:    return "PAYLOAD_LIFETIME_MISMATCH";
        default:                                                   return "UNKNOWN";
    }
}

void memory_info_audit(memory_health_t* out) {
    // Publish this audit's SP as the stack-tripwire floor: while the audit
    // runs, the live foreground SP is always at or below this value, so any
    // exception frame stacked above it is unlawful.  See crash_forensics.h.
    crash_stack_tripwire_floor_enter();

    memory_audit_trace_record(
        memory_audit_trace_stage_t::AUDIT_ENTER,
        _memory_health.audit_count + 1U,
        (uint32_t)_memory_health.status);
    _memory_audit_trace_active = true;

    memory_info_get(&_memory_health_memory_scratch);

    memory_audit_trace_record(
        memory_audit_trace_stage_t::PAYLOAD_INFO_ENTER,
        _memory_health_payload_scratch.alive_now,
        _memory_health_payload_scratch.last_error_code);
    payload_get_info(&_memory_health_payload_scratch);
    memory_audit_trace_record(
        memory_audit_trace_stage_t::PAYLOAD_INFO_RETURN,
        _memory_health_payload_scratch.alive_now,
        _memory_health_payload_scratch.last_error_code);
    memory_audit_trace_record(
        memory_audit_trace_stage_t::PAYLOAD_SELF_OK,
        _memory_health_payload_scratch.self_ok_fail,
        _memory_health_payload_scratch.integrity_fail);
    memory_audit_trace_record(
        memory_audit_trace_stage_t::STRING_POINTERS,
        _memory_health_payload_scratch.string_pointer_fault,
        _memory_health_payload_scratch.last_string_pointer_fault_reason);

    const memory_info_t& memory = _memory_health_memory_scratch;
    const payload_info_t& payload = _memory_health_payload_scratch;

    const bool payload_lifetime_mismatch =
        payload.instances_constructed < payload.instances_destroyed ||
        (payload.instances_constructed - payload.instances_destroyed) != payload.alive_now;

    const bool dtcm_map_invalid =
        memory.dtcm_total == 0U ||
        memory.dtcm_static > memory.dtcm_total ||
        memory.dtcm_stack_avail == 0U ||
        memory.dtcm_static + memory.dtcm_stack_avail != memory.dtcm_total;

    const bool heap_map_invalid =
        memory.heap_total == 0U ||
        memory.heap_arena > memory.heap_total ||
        memory.heap_used > memory.heap_arena ||
        memory.heap_free_internal > memory.heap_arena ||
        memory.heap_free_total > memory.heap_total;

    uint32_t active_reason_mask = 0U;
    memory_health_reason_t primary_reason = memory_health_reason_t::NONE;

    const uint32_t map_flags =
        (dtcm_map_invalid ? 1U : 0U) |
        (heap_map_invalid ? 2U : 0U);
    memory_audit_trace_record(
        memory_audit_trace_stage_t::HEALTH_RULES,
        map_flags,
        payload_lifetime_mismatch ? 1U : 0U);

    if (!memory.initialized) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::NOT_INITIALIZED);
    }
    if (dtcm_map_invalid) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::DTCM_MAP_INVALID);
    }
    if (memory.stack_paint_compiled_enabled && !memory.stack_paint_enabled) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::STACK_PAINT_UNAVAILABLE);
    }
    if (memory.stack_paint_overrun) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::STACK_PAINT_OVERRUN);
    }
    if (memory.stack_collision_risk ||
        memory.stack_free_high_water <= memory.stack_collision_warn_bytes) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::STACK_RUNWAY_CRITICAL);
    }
    if (heap_map_invalid) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::HEAP_MAP_INVALID);
    }
    if (!heap_map_invalid &&
        memory.heap_free_total <= MEMORY_HEAP_FREE_CRITICAL_BYTES) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::HEAP_FREE_CRITICAL);
    }

    // Payload corruption courts are higher-value than pressure/high-water
    // telemetry.  Do not treat ordinary realloc counts, fragmentation, or a
    // lawful high-water mark as campaign-fatal evidence.
    if (payload.self_ok_fail != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_SELF_OK_FAIL);
    }
    if (payload.integrity_fail != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_INTEGRITY_FAIL);
    }
    if (payload.string_pointer_fault != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_STRING_POINTER_FAULT);
    }
    if (payload.invalid_kind != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_INVALID_KIND);
    }
    if (payload.entry_alloc_fail != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_ENTRY_ALLOC_FAIL);
    }
    if (payload.arena_alloc_fail != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_ARENA_ALLOC_FAIL);
    }
    if (payload.entry_overflow != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_ENTRY_OVERFLOW);
    }
    if (payload.serialize_overflow != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_SERIALIZE_OVERFLOW);
    }
    if (payload.to_json_fail != 0U) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_TO_JSON_FAIL);
    }
    if (payload_lifetime_mismatch) {
        memory_health_add_reason(active_reason_mask, primary_reason,
                                 memory_health_reason_t::PAYLOAD_LIFETIME_MISMATCH);
    }

    memory_audit_trace_record(
        memory_audit_trace_stage_t::HEALTH_COMMIT,
        active_reason_mask,
        (uint32_t)primary_reason);

    _memory_health.audit_count++;
    _memory_health.active_reason_mask = active_reason_mask;
    _memory_health.heap_free_critical_bytes = MEMORY_HEAP_FREE_CRITICAL_BYTES;

    _memory_health.memory_initialized = memory.initialized;
    _memory_health.dtcm_total = memory.dtcm_total;
    _memory_health.dtcm_static = memory.dtcm_static;
    _memory_health.dtcm_stack_avail = memory.dtcm_stack_avail;
    _memory_health.stack_high_water = memory.stack_high_water;
    _memory_health.stack_free_high_water = memory.stack_free_high_water;
    _memory_health.stack_paint_compiled_enabled = memory.stack_paint_compiled_enabled;
    _memory_health.stack_paint_enabled = memory.stack_paint_enabled;
    _memory_health.stack_paint_overrun = memory.stack_paint_overrun;
    _memory_health.stack_collision_risk = memory.stack_collision_risk;

    _memory_health.heap_total = memory.heap_total;
    _memory_health.heap_arena = memory.heap_arena;
    _memory_health.heap_used = memory.heap_used;
    _memory_health.heap_free_total = memory.heap_free_total;
    _memory_health.heap_arena_high_water = memory.heap_arena_high_water;

    _memory_health.payload_instances_constructed = payload.instances_constructed;
    _memory_health.payload_instances_destroyed = payload.instances_destroyed;
    _memory_health.payload_alive_now = payload.alive_now;
    _memory_health.payload_lifetime_mismatch = payload_lifetime_mismatch;
    _memory_health.payload_entry_alloc_fail = payload.entry_alloc_fail;
    _memory_health.payload_arena_alloc_fail = payload.arena_alloc_fail;
    _memory_health.payload_entry_overflow = payload.entry_overflow;
    _memory_health.payload_serialize_overflow = payload.serialize_overflow;
    _memory_health.payload_to_json_fail = payload.to_json_fail;
    _memory_health.payload_integrity_fail = payload.integrity_fail;
    _memory_health.payload_invalid_kind = payload.invalid_kind;
    _memory_health.payload_string_pointer_fault = payload.string_pointer_fault;
    _memory_health.payload_self_ok_fail = payload.self_ok_fail;
    _memory_health.payload_entry_high_water = payload.entry_high_water;
    _memory_health.payload_max_entries = payload.payload_max_entries;
    _memory_health.payload_arena_high_water = payload.arena_high_water;
    _memory_health.payload_arena_max = payload.payload_arena_max;
    _memory_health.payload_last_error_code = payload.last_error_code;
    _memory_health.payload_last_string_pointer_fault_reason =
        payload.last_string_pointer_fault_reason;
    _memory_health.payload_last_self_ok_fail_reason =
        payload.last_self_ok_fail_reason;

    if (active_reason_mask != 0U) {
        _memory_health.anomaly_audit_count++;
        _memory_health.last_anomaly_audit = _memory_health.audit_count;
        _memory_health.latched_reason_mask |= active_reason_mask;

        if (!_memory_health.latched) {
            _memory_health.latched = true;
            _memory_health.primary_reason = primary_reason;
            _memory_health.first_anomaly_audit = _memory_health.audit_count;
        }
    }

    _memory_health.status = _memory_health.latched
        ? memory_health_status_t::ANOMALY
        : memory_health_status_t::NOMINAL;

    memory_audit_trace_record(
        memory_audit_trace_stage_t::OUTPUT_COPY,
        out ? 1U : 0U,
        _memory_health.audit_count);
    if (out) {
        *out = _memory_health;
    }

    memory_audit_trace_record(
        memory_audit_trace_stage_t::AUDIT_EXIT,
        _memory_health.audit_count,
        (uint32_t)_memory_health.status);
    _memory_audit_trace_active = false;

    crash_stack_tripwire_floor_exit();
}

void memory_info_get_health(memory_health_t* out) {
    if (!out) return;
    *out = _memory_health;
}
