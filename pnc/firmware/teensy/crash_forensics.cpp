#include "crash_forensics.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// Core linkage and retained storage
// ============================================================================

extern "C" {
extern void (* volatile _VectorsRam[])(void);
void unused_interrupt_vector(void);
extern unsigned long _stext;
extern unsigned long _etext;
extern unsigned long _estack;
void crash_forensics_fault_entry(void);

// Ordinary BSS: cleared by startup before startup_late_hook() installs us.
volatile uint32_t g_crash_forensics_capture_active = 0U;
alignas(8) uint32_t g_crash_forensics_emergency_stack[512];
}

alignas(32) static crash_forensics_core_record_t
    g_crash_forensics_core_record DMAMEM;
alignas(32) static crash_forensics_record_t g_crash_forensics_record DMAMEM;

static constexpr uint32_t CRASH_FORENSICS_CORE_MAGIC = 0x5A504343UL; // "ZPCC"
static constexpr uint32_t CRASH_FORENSICS_CORE_COMMITTED = 0x434F5245UL; // "CORE"
static constexpr uint32_t CRASH_FORENSICS_MAGIC = 0x5A504346UL;     // "ZPCF"
static constexpr uint32_t CRASH_FORENSICS_COMMITTED = 0x434F4D54UL; // "COMT"
static constexpr size_t CRASH_FORENSICS_VECTOR_HARDFAULT = 3U;
static constexpr size_t CRASH_FORENSICS_VECTOR_MEMMANAGE = 4U;
static constexpr size_t CRASH_FORENSICS_VECTOR_BUSFAULT = 5U;
static constexpr size_t CRASH_FORENSICS_VECTOR_USAGEFAULT = 6U;
static constexpr size_t CRASH_STACK_WATCH_VECTOR_DEBUGMON = 12U;

// Cortex-M7 system-register addresses.  Fixed addresses keep the fault path
// independent of CMSIS naming differences across Teensyduino releases.
static constexpr uintptr_t REG_CPUID  = 0xE000ED00UL;
static constexpr uintptr_t REG_ICSR   = 0xE000ED04UL;
static constexpr uintptr_t REG_VTOR   = 0xE000ED08UL;
static constexpr uintptr_t REG_AIRCR  = 0xE000ED0CUL;
static constexpr uintptr_t REG_SCR    = 0xE000ED10UL;
static constexpr uintptr_t REG_CCR    = 0xE000ED14UL;
static constexpr uintptr_t REG_SHPR1  = 0xE000ED18UL;
static constexpr uintptr_t REG_SHPR2  = 0xE000ED1CUL;
static constexpr uintptr_t REG_SHPR3  = 0xE000ED20UL;
static constexpr uintptr_t REG_SHCSR  = 0xE000ED24UL;
static constexpr uintptr_t REG_CFSR   = 0xE000ED28UL;
static constexpr uintptr_t REG_HFSR   = 0xE000ED2CUL;
static constexpr uintptr_t REG_DFSR   = 0xE000ED30UL;
static constexpr uintptr_t REG_MMFAR  = 0xE000ED34UL;
static constexpr uintptr_t REG_BFAR   = 0xE000ED38UL;
static constexpr uintptr_t REG_AFSR   = 0xE000ED3CUL;
static constexpr uintptr_t REG_CPACR  = 0xE000ED88UL;
static constexpr uintptr_t REG_DEMCR  = 0xE000EDFCUL;
static constexpr uintptr_t REG_DWT_CTRL = 0xE0001000UL;
static constexpr uintptr_t REG_DWT_CYCCNT = 0xE0001004UL;
static constexpr uintptr_t REG_DWT_COMP1 = 0xE0001030UL;
static constexpr uintptr_t REG_DWT_MASK1 = 0xE0001034UL;
static constexpr uintptr_t REG_DWT_FUNCTION1 = 0xE0001038UL;
static constexpr uintptr_t REG_ACTLR = 0xE000E008UL;
static constexpr uintptr_t REG_SYST_CSR = 0xE000E010UL;
static constexpr uintptr_t REG_SYST_RVR = 0xE000E014UL;
static constexpr uintptr_t REG_SYST_CVR = 0xE000E018UL;
static constexpr uintptr_t REG_SYST_CALIB = 0xE000E01CUL;

static constexpr uintptr_t REG_FPCCR  = 0xE000EF34UL;
static constexpr uintptr_t REG_FPCAR  = 0xE000EF38UL;
static constexpr uintptr_t REG_FPDSCR = 0xE000EF3CUL;

static constexpr uintptr_t REG_NVIC_ISER = 0xE000E100UL;
static constexpr uintptr_t REG_NVIC_ISPR = 0xE000E200UL;
static constexpr uintptr_t REG_NVIC_IABR = 0xE000E300UL;

static constexpr uintptr_t REG_MPU_TYPE = 0xE000ED90UL;
static constexpr uintptr_t REG_MPU_CTRL = 0xE000ED94UL;
static constexpr uintptr_t REG_MPU_RNR  = 0xE000ED98UL;
static constexpr uintptr_t REG_MPU_RBAR = 0xE000ED9CUL;
static constexpr uintptr_t REG_MPU_RASR = 0xE000EDA0UL;

static constexpr uint32_t CFSR_STACKING_ERROR_MASK =
    (1UL << 3)  | // MUNSTKERR
    (1UL << 4)  | // MSTKERR
    (1UL << 5)  | // MLSPERR
    (1UL << 11) | // UNSTKERR
    (1UL << 12) | // STKERR
    (1UL << 13);  // LSPERR

struct crash_forensics_entry_context_t {
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;
    uint32_t frame_sp;
    uint32_t exc_return;
    uint32_t msp;
    uint32_t psp;
    uint32_t primask;
    uint32_t basepri;
    uint32_t faultmask;
    uint32_t control;
};

static_assert(sizeof(crash_forensics_entry_context_t) == 64U,
              "Fault entry assembly offsets no longer match the C structure");

// ============================================================================
// Tiny fault-safe primitives
// ============================================================================

static inline uint32_t reg32(uintptr_t address) {
    return *reinterpret_cast<volatile const uint32_t*>(address);
}

static inline void reg32_write(uintptr_t address, uint32_t value) {
    *reinterpret_cast<volatile uint32_t*>(address) = value;
}

static inline uint32_t read_ipsr(void) {
    uint32_t value;
    asm volatile("mrs %0, ipsr" : "=r"(value) :: "memory");
    return value;
}

static inline uint32_t read_primask(void) {
    uint32_t value;
    asm volatile("mrs %0, primask" : "=r"(value) :: "memory");
    return value;
}

static inline void write_primask(uint32_t value) {
    asm volatile("msr primask, %0" :: "r"(value) : "memory");
}

static inline void disable_interrupts(void) {
    asm volatile("cpsid i" ::: "memory");
}

static inline void crash_barrier(void) {
    asm volatile("dsb\nisb" ::: "memory");
}

static void zero_record(crash_forensics_record_t& record) {
    uint32_t* words = reinterpret_cast<uint32_t*>(&record);
    for (size_t i = 0; i < sizeof(record) / sizeof(uint32_t); ++i) {
        words[i] = 0U;
    }
}

static void zero_core_record(crash_forensics_core_record_t& record) {
    uint32_t* words = reinterpret_cast<uint32_t*>(&record);
    for (size_t i = 0; i < sizeof(record) / sizeof(uint32_t); ++i) {
        words[i] = 0U;
    }
}

static uint32_t crash_crc32_words(const uint32_t* p,
                                  const uint32_t* end) {
    uint32_t crc = 0xFFFFFFFFUL;
    while (p < end) {
        crc ^= *p++;
        for (uint32_t bit = 0; bit < 32U; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return crc;
}

static uint32_t core_record_crc32(
    const crash_forensics_core_record_t& record) {
    const uint32_t* const begin =
        reinterpret_cast<const uint32_t*>(&record);
    const uint32_t* const end = reinterpret_cast<const uint32_t*>(
        reinterpret_cast<const uint8_t*>(&record) +
        offsetof(crash_forensics_core_record_t, crc32));
    return crash_crc32_words(begin, end);
}

static uint32_t record_crc32(const crash_forensics_record_t& record) {
    const uint32_t* const begin =
        reinterpret_cast<const uint32_t*>(&record);
    const uint32_t* const end = reinterpret_cast<const uint32_t*>(
        reinterpret_cast<const uint8_t*>(&record) +
        offsetof(crash_forensics_record_t, crc32));
    return crash_crc32_words(begin, end);
}

static bool range_contains(uintptr_t begin,
                           uintptr_t end,
                           uintptr_t address,
                           size_t bytes) {
    if (end <= begin || address < begin || address >= end) return false;
    if (bytes > static_cast<size_t>(end - address)) return false;
    return true;
}

static constexpr uintptr_t CRASH_FORENSICS_EXECUTABLE_FLOOR = 0x00000400UL;

static bool stack_region_for(uintptr_t address,
                             uintptr_t* out_begin,
                             uintptr_t* out_end) {
    const uintptr_t dtcm_end = reinterpret_cast<uintptr_t>(&_estack);
    if (range_contains(0x20000000UL, dtcm_end, address, 1U)) {
        if (out_begin) *out_begin = 0x20000000UL;
        if (out_end) *out_end = dtcm_end;
        return true;
    }
    if (range_contains(0x20200000UL, 0x20280000UL, address, 1U)) {
        if (out_begin) *out_begin = 0x20200000UL;
        if (out_end) *out_end = 0x20280000UL;
        return true;
    }
    return false;
}

static bool executable_region_for(uintptr_t address,
                                  uintptr_t* out_begin,
                                  uintptr_t* out_end) {
    uintptr_t itcm_begin = reinterpret_cast<uintptr_t>(&_stext);
    const uintptr_t itcm_end = reinterpret_cast<uintptr_t>(&_etext);
    if (itcm_begin < CRASH_FORENSICS_EXECUTABLE_FLOOR) {
        itcm_begin = CRASH_FORENSICS_EXECUTABLE_FLOOR;
    }

    if (range_contains(itcm_begin, itcm_end, address, 1U)) {
        if (out_begin) *out_begin = itcm_begin;
        if (out_end) *out_end = itcm_end;
        return true;
    }
    if (range_contains(0x60000000UL, 0x61000000UL, address, 1U)) {
        if (out_begin) *out_begin = 0x60000000UL;
        if (out_end) *out_end = 0x61000000UL;
        return true;
    }
    return false;
}

static bool mpu_privileged_read_allowed(uint32_t rasr) {
    const uint32_t ap = (rasr >> 24) & 0x7U;
    return ap == 1U || ap == 2U || ap == 3U || ap == 5U || ap == 6U;
}

static bool mpu_span_allows(uintptr_t address,
                            size_t bytes,
                            bool require_execute) {
    if (bytes == 0U || address > UINT32_MAX ||
        bytes - 1U > static_cast<size_t>(UINT32_MAX - address)) {
        return false;
    }

    const uint32_t control = reg32(REG_MPU_CTRL);
    if ((control & 1U) == 0U) {
        return true;
    }

    const uint32_t saved_rnr = reg32(REG_MPU_RNR);
    uint32_t region_count = (reg32(REG_MPU_TYPE) >> 8) & 0xFFU;
    if (region_count > CRASH_FORENSICS_MPU_REGIONS) {
        region_count = CRASH_FORENSICS_MPU_REGIONS;
    }

    bool matched = false;
    uint32_t matched_rasr = 0U;
    const uint64_t first = static_cast<uint64_t>(address);
    const uint64_t last = first + static_cast<uint64_t>(bytes) - 1ULL;

    for (uint32_t region = 0U; region < region_count; ++region) {
        reg32_write(REG_MPU_RNR, region);
        crash_barrier();
        const uint32_t rbar = reg32(REG_MPU_RBAR);
        const uint32_t rasr = reg32(REG_MPU_RASR);
        if ((rasr & 1U) == 0U) continue;

        const uint32_t size_field = (rasr >> 1) & 0x1FU;
        if (size_field < 4U) continue;
        const uint64_t region_size = 1ULL << (size_field + 1U);
        const uint64_t region_base =
            static_cast<uint64_t>(rbar) & ~(region_size - 1ULL);
        const uint64_t region_end = region_base + region_size;
        if (first < region_base || last >= region_end) continue;

        if (region_size >= 256ULL) {
            const uint64_t subregion_size = region_size / 8ULL;
            const uint32_t first_subregion =
                static_cast<uint32_t>((first - region_base) / subregion_size);
            const uint32_t last_subregion =
                static_cast<uint32_t>((last - region_base) / subregion_size);
            const uint32_t disabled = (rasr >> 8) & 0xFFU;
            bool subregions_enabled = true;
            for (uint32_t subregion = first_subregion;
                 subregion <= last_subregion;
                 ++subregion) {
                if ((disabled & (1U << subregion)) != 0U) {
                    subregions_enabled = false;
                    break;
                }
            }
            if (!subregions_enabled) continue;
        }

        // ARMv7-M gives the highest-numbered matching region priority.  The
        // ascending scan therefore intentionally overwrites an earlier match.
        matched = true;
        matched_rasr = rasr;
    }

    reg32_write(REG_MPU_RNR, saved_rnr);
    crash_barrier();

    if (!matched) {
        // Privileged background mapping is valid only when PRIVDEFENA is set.
        return (control & (1UL << 2)) != 0U;
    }
    if (!mpu_privileged_read_allowed(matched_rasr)) return false;
    if (require_execute && (matched_rasr & (1UL << 28)) != 0U) return false;
    return true;
}

static bool stack_span_readable(uintptr_t address, size_t bytes) {
    uintptr_t begin = 0U;
    uintptr_t end = 0U;
    return stack_region_for(address, &begin, &end) &&
           range_contains(begin, end, address, bytes) &&
           mpu_span_allows(address, bytes, false);
}

static bool executable_span_readable(uintptr_t address, size_t bytes) {
    uintptr_t begin = 0U;
    uintptr_t end = 0U;
    return executable_region_for(address, &begin, &end) &&
           range_contains(begin, end, address, bytes) &&
           mpu_span_allows(address, bytes, true);
}

static bool exception_return_value(uint32_t value) {
    return (value & 0xFFFFFF00UL) == 0xFFFFFF00UL && (value & 1U) != 0U;
}

static bool stacked_pc_plausible(uint32_t pc) {
    if (pc == 0U || (pc & 1U) != 0U) return false;
    return executable_span_readable(pc, sizeof(uint16_t));
}

static bool stacked_lr_plausible(uint32_t lr) {
    if (exception_return_value(lr)) return true;
    if (lr == 0U || (lr & 1U) == 0U) return false;
    return executable_span_readable(lr & ~1UL, sizeof(uint16_t));
}

static uint32_t capture_stack_words(
    uint32_t base,
    uint32_t* destination,
    size_t capacity,
    uint32_t* out_skip_reason) {
    if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_NONE;
    if (!destination || capacity == 0U) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_INVALID_ARGUMENT;
        }
        return 0U;
    }
    if (base == 0U) {
        if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_ZERO_ADDRESS;
        return 0U;
    }
    if ((base & 3U) != 0U) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_UNALIGNED_ADDRESS;
        }
        return 0U;
    }

    uintptr_t region_begin = 0U;
    uintptr_t region_end = 0U;
    if (!stack_region_for(base, &region_begin, &region_end)) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_OUTSIDE_STACK_MEMORY;
        }
        return 0U;
    }

    const size_t available_words =
        static_cast<size_t>(region_end - static_cast<uintptr_t>(base)) /
        sizeof(uint32_t);
    const size_t count = capacity < available_words ? capacity : available_words;
    if (count == 0U || count > UINT32_MAX / sizeof(uint32_t)) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_ADDRESS_OVERFLOW;
        }
        return 0U;
    }

    const size_t bytes = count * sizeof(uint32_t);
    if (!stack_span_readable(base, bytes)) {
        if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_MPU_DENIED;
        return 0U;
    }

    // Preflight the complete bounded span before touching the first suspect
    // word.  A denied or malformed window is skipped atomically.
    for (size_t i = 0U; i < count; ++i) {
        const uintptr_t address = static_cast<uintptr_t>(base) + i * 4U;
        if (address < static_cast<uintptr_t>(base) ||
            !stack_span_readable(address, sizeof(uint32_t))) {
            if (out_skip_reason) {
                *out_skip_reason = address < static_cast<uintptr_t>(base)
                    ? CRASH_FORENSICS_SKIP_ADDRESS_OVERFLOW
                    : CRASH_FORENSICS_SKIP_MPU_DENIED;
            }
            return 0U;
        }
    }

    for (size_t i = 0U; i < count; ++i) {
        const uintptr_t address = static_cast<uintptr_t>(base) + i * 4U;
        destination[i] = *reinterpret_cast<volatile const uint32_t*>(address);
    }
    return static_cast<uint32_t>(count);
}

static uint32_t capture_centered_executable_window(
    uint32_t center,
    bool link_register,
    uint32_t* out_base,
    uint32_t* destination,
    size_t capacity,
    uint32_t* out_skip_reason) {
    if (out_base) *out_base = 0U;
    if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_NONE;
    if (!destination || capacity == 0U) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_INVALID_ARGUMENT;
        }
        return 0U;
    }
    if (center == 0U) {
        if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_ZERO_ADDRESS;
        return 0U;
    }

    uint32_t normalized = center;
    if (link_register) {
        if (!stacked_lr_plausible(center) || exception_return_value(center)) {
            if (out_skip_reason) {
                *out_skip_reason = CRASH_FORENSICS_SKIP_LR_IMPLAUSIBLE;
            }
            return 0U;
        }
        normalized &= ~1UL;
    } else if (!stacked_pc_plausible(center)) {
        if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_PC_IMPLAUSIBLE;
        return 0U;
    }

    uintptr_t region_begin = 0U;
    uintptr_t region_end = 0U;
    if (!executable_region_for(normalized, &region_begin, &region_end)) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_OUTSIDE_EXECUTABLE_MEMORY;
        }
        return 0U;
    }

    const uintptr_t aligned = static_cast<uintptr_t>(normalized & ~3UL);
    const size_t before_words = capacity / 2U;
    const uint64_t before_bytes = static_cast<uint64_t>(before_words) * 4ULL;
    uintptr_t base = aligned;
    if (static_cast<uint64_t>(aligned) >= before_bytes) {
        const uintptr_t candidate =
            static_cast<uintptr_t>(static_cast<uint64_t>(aligned) - before_bytes);
        base = candidate < region_begin ? region_begin : candidate;
    } else {
        base = region_begin;
    }
    base = (base + 3U) & ~static_cast<uintptr_t>(3U);

    const size_t available_words =
        static_cast<size_t>(region_end - base) / sizeof(uint32_t);
    const size_t count = capacity < available_words ? capacity : available_words;
    if (count == 0U) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_OUTSIDE_EXECUTABLE_MEMORY;
        }
        return 0U;
    }

    for (size_t i = 0U; i < count; ++i) {
        const uintptr_t address = base + i * 4U;
        if (address < base ||
            !executable_span_readable(address, sizeof(uint32_t))) {
            if (out_skip_reason) {
                *out_skip_reason = address < base
                    ? CRASH_FORENSICS_SKIP_ADDRESS_OVERFLOW
                    : CRASH_FORENSICS_SKIP_MPU_DENIED;
            }
            return 0U;
        }
    }

    for (size_t i = 0U; i < count; ++i) {
        const uintptr_t address = base + i * 4U;
        destination[i] = *reinterpret_cast<volatile const uint32_t*>(address);
    }
    if (out_base) *out_base = static_cast<uint32_t>(base);
    return static_cast<uint32_t>(count);
}

static bool core_stack_span_readable(uintptr_t address, size_t bytes) {
    uintptr_t begin = 0U;
    uintptr_t end = 0U;
    return stack_region_for(address, &begin, &end) &&
           range_contains(begin, end, address, bytes);
}

static uint32_t capture_core_stack_words(
    uint32_t base,
    uint32_t* destination,
    size_t capacity,
    uint32_t* out_skip_reason) {
    if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_NONE;
    if (!destination || capacity == 0U) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_INVALID_ARGUMENT;
        }
        return 0U;
    }
    if (base == 0U) {
        if (out_skip_reason) *out_skip_reason = CRASH_FORENSICS_SKIP_ZERO_ADDRESS;
        return 0U;
    }
    if ((base & 3U) != 0U) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_UNALIGNED_ADDRESS;
        }
        return 0U;
    }

    uintptr_t begin = 0U;
    uintptr_t end = 0U;
    if (!stack_region_for(base, &begin, &end)) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_OUTSIDE_STACK_MEMORY;
        }
        return 0U;
    }

    const size_t available =
        static_cast<size_t>(end - static_cast<uintptr_t>(base)) /
        sizeof(uint32_t);
    const size_t count = capacity < available ? capacity : available;
    if (count == 0U) {
        if (out_skip_reason) {
            *out_skip_reason = CRASH_FORENSICS_SKIP_ADDRESS_OVERFLOW;
        }
        return 0U;
    }

    for (size_t i = 0U; i < count; ++i) {
        destination[i] = *reinterpret_cast<volatile const uint32_t*>(
            static_cast<uintptr_t>(base) + i * sizeof(uint32_t));
    }
    return static_cast<uint32_t>(count);
}

static bool core_pc_plausible(uint32_t pc) {
    if (pc == 0U || (pc & 1U) != 0U) return false;
    uintptr_t begin = 0U;
    uintptr_t end = 0U;
    return executable_region_for(pc, &begin, &end) &&
           range_contains(begin, end, pc, sizeof(uint16_t));
}

static bool core_lr_plausible(uint32_t lr) {
    if (exception_return_value(lr)) return true;
    if (lr == 0U || (lr & 1U) == 0U) return false;
    const uint32_t address = lr & ~1UL;
    uintptr_t begin = 0U;
    uintptr_t end = 0U;
    return executable_region_for(address, &begin, &end) &&
           range_contains(begin, end, address, sizeof(uint16_t));
}

struct crash_basic_frame_selection_t {
    bool readable;
    bool plausible;
    bool lr_plausible;
    uint32_t offset_words;
    uint32_t address;
};

static crash_basic_frame_selection_t core_basic_frame_candidate(
    uint32_t frame_sp,
    uint32_t offset_words) {
    crash_basic_frame_selection_t candidate{};
    candidate.offset_words = offset_words;

    const uint32_t offset_bytes = offset_words * sizeof(uint32_t);
    if ((frame_sp & 3U) != 0U ||
        frame_sp > UINT32_MAX - offset_bytes) {
        return candidate;
    }

    candidate.address = frame_sp + offset_bytes;
    candidate.readable =
        core_stack_span_readable(candidate.address, 8U * sizeof(uint32_t));
    if (!candidate.readable) return candidate;

    const volatile uint32_t* const basic =
        reinterpret_cast<volatile const uint32_t*>(candidate.address);
    const uint32_t lr = basic[5];
    const uint32_t pc = basic[6];
    const uint32_t xpsr = basic[7];
    candidate.lr_plausible = core_lr_plausible(lr);
    candidate.plausible =
        (xpsr & (1UL << 24)) != 0U &&
        core_pc_plausible(pc);
    return candidate;
}

static crash_basic_frame_selection_t extended_basic_frame_candidate(
    uint32_t frame_sp,
    uint32_t offset_words) {
    crash_basic_frame_selection_t candidate{};
    candidate.offset_words = offset_words;

    const uint32_t offset_bytes = offset_words * sizeof(uint32_t);
    if ((frame_sp & 3U) != 0U ||
        frame_sp > UINT32_MAX - offset_bytes) {
        return candidate;
    }

    candidate.address = frame_sp + offset_bytes;
    candidate.readable =
        stack_span_readable(candidate.address, 8U * sizeof(uint32_t));
    if (!candidate.readable) return candidate;

    const volatile uint32_t* const basic =
        reinterpret_cast<volatile const uint32_t*>(candidate.address);
    const uint32_t lr = basic[5];
    const uint32_t pc = basic[6];
    const uint32_t xpsr = basic[7];
    candidate.lr_plausible = stacked_lr_plausible(lr);
    candidate.plausible =
        (xpsr & (1UL << 24)) != 0U &&
        stacked_pc_plausible(pc);
    return candidate;
}

static crash_basic_frame_selection_t select_basic_frame(
    const crash_basic_frame_selection_t& architectural,
    const crash_basic_frame_selection_t& alternate) {
    // EXC_RETURN is the architectural starting point, but lazy FP stacking or
    // damaged exception metadata can make its bit-4 frame-shape hint disagree
    // with the words actually present on the stack.  Prefer a frame that proves
    // itself through xPSR, executable PC and lawful LR.
    if (architectural.plausible && alternate.plausible) {
        // A lawful LR strengthens a candidate, but an implausible stacked LR is
        // itself valid crash evidence and must never disqualify an otherwise
        // proven exception frame.
        if (architectural.lr_plausible != alternate.lr_plausible) {
            return architectural.lr_plausible ? architectural : alternate;
        }
        return architectural;
    }
    if (architectural.plausible) return architectural;
    if (alternate.plausible) return alternate;

    // If neither candidate is fully plausible, preserve the architectural
    // preference when it is at least readable.  This keeps malformed-frame
    // evidence available without allowing the hint to override a proven frame.
    if (architectural.readable) return architectural;
    return alternate;
}

static bool core_record_header_valid(
    const crash_forensics_core_record_t& record) {
    return record.magic == CRASH_FORENSICS_CORE_MAGIC &&
           record.magic_inv == ~CRASH_FORENSICS_CORE_MAGIC &&
           record.schema_version == CRASH_FORENSICS_CORE_SCHEMA_VERSION &&
           record.record_size == sizeof(crash_forensics_core_record_t) &&
           record.committed == CRASH_FORENSICS_CORE_COMMITTED &&
           record.committed_inv == ~CRASH_FORENSICS_CORE_COMMITTED;
}

static bool core_record_crc_valid(
    const crash_forensics_core_record_t& record) {
    return core_record_header_valid(record) &&
           core_record_crc32(record) == record.crc32;
}

static void core_record_publish_stage(
    crash_forensics_capture_stage_t stage) {
    crash_forensics_core_record_t& record = g_crash_forensics_core_record;
    const uint32_t value = static_cast<uint32_t>(stage);

    // Invalidate the pair first.  A fault during the update leaves the core
    // record valid while making only the progress marker incoherent.
    record.capture_stage_inv = value;
    crash_barrier();
    record.capture_stage = value;
    record.capture_stage_inv = ~value;
    crash_barrier();

    arm_dcache_flush_delete(&record, sizeof(record));
    crash_barrier();
}

static void capture_core_record_from_entry(
    const crash_forensics_entry_context_t* entry,
    uint32_t sequence) {
    crash_forensics_core_record_t& record = g_crash_forensics_core_record;
    zero_core_record(record);

    record.magic = CRASH_FORENSICS_CORE_MAGIC;
    record.magic_inv = ~CRASH_FORENSICS_CORE_MAGIC;
    record.schema_version = CRASH_FORENSICS_CORE_SCHEMA_VERSION;
    record.record_size = sizeof(crash_forensics_core_record_t);
    record.capture_sequence = sequence;
    record.capture_stage = CRASH_FORENSICS_STAGE_CORE_BEGIN;
    record.capture_stage_inv = ~CRASH_FORENSICS_STAGE_CORE_BEGIN;

    record.exception_number = read_ipsr() & 0x1FFU;
    record.exc_return = entry->exc_return;
    record.original_msp = entry->msp;
    record.original_psp = entry->psp;
    record.primask = entry->primask;
    record.basepri = entry->basepri;
    record.faultmask = entry->faultmask;
    record.control = entry->control;
    record.dwt_cyccnt = reg32(REG_DWT_CYCCNT);
    record.cpu_hz = static_cast<uint32_t>(F_CPU_ACTUAL);

    record.r4 = entry->r4;
    record.r5 = entry->r5;
    record.r6 = entry->r6;
    record.r7 = entry->r7;
    record.r8 = entry->r8;
    record.r9 = entry->r9;
    record.r10 = entry->r10;
    record.r11 = entry->r11;

    record.cfsr = reg32(REG_CFSR);
    record.hfsr = reg32(REG_HFSR);
    record.dfsr = reg32(REG_DFSR);
    record.afsr = reg32(REG_AFSR);
    record.mmfar = reg32(REG_MMFAR);
    record.bfar = reg32(REG_BFAR);
    record.shcsr = reg32(REG_SHCSR);
    record.icsr = reg32(REG_ICSR);
    record.fpccr = reg32(REG_FPCCR);
    record.fpcar = reg32(REG_FPCAR);

    record.frame_source = (entry->exc_return & (1UL << 2))
        ? CRASH_FORENSICS_FRAME_PSP
        : CRASH_FORENSICS_FRAME_MSP;
    record.raw_frame_address = entry->frame_sp;

    const bool extended_frame = (entry->exc_return & (1UL << 4)) == 0U;
    if (extended_frame) {
        record.flags |= CRASH_FORENSICS_FLAG_EXTENDED_FP_FRAME;
    }
    if (entry->exc_return & (1UL << 3)) {
        record.flags |= CRASH_FORENSICS_FLAG_RETURN_TO_THREAD;
    }

    const bool stacking_fault =
        (record.cfsr & CFSR_STACKING_ERROR_MASK) != 0U;
    if (stacking_fault) {
        record.flags |= CRASH_FORENSICS_FLAG_STACKING_FAULT;
    }

    const uint32_t architectural_offset_words =
        extended_frame ? 18U : 0U;
    const uint32_t alternate_offset_words =
        extended_frame ? 0U : 18U;
    const crash_basic_frame_selection_t architectural =
        core_basic_frame_candidate(entry->frame_sp,
                                   architectural_offset_words);
    const crash_basic_frame_selection_t alternate =
        core_basic_frame_candidate(entry->frame_sp,
                                   alternate_offset_words);
    const crash_basic_frame_selection_t selected =
        select_basic_frame(architectural, alternate);

    const uint32_t selected_basic_offset_words = selected.offset_words;
    record.basic_frame_address = selected.address;
    const bool frame_readable = selected.readable;
    if (frame_readable) {
        record.flags |= CRASH_FORENSICS_FLAG_FRAME_ADDRESS_READABLE;
    }

    if (frame_readable && !stacking_fault) {
        const volatile uint32_t* const basic =
            reinterpret_cast<volatile const uint32_t*>(
                record.basic_frame_address);

        record.r0 = basic[0];
        record.r1 = basic[1];
        record.r2 = basic[2];
        record.r3 = basic[3];
        record.r12 = basic[4];
        record.stacked_lr = basic[5];
        record.stacked_pc = basic[6];
        record.stacked_xpsr = basic[7];
        record.interrupted_exception_number = record.stacked_xpsr & 0x1FFU;
        record.flags |= CRASH_FORENSICS_FLAG_BASIC_FRAME_VALID;

        uint32_t complete_frame_words = selected_basic_offset_words + 8U;
        if ((record.stacked_xpsr & (1UL << 9)) != 0U) {
            complete_frame_words++;
            record.flags |= CRASH_FORENSICS_FLAG_STACK_ALIGNMENT_WORD;
        }
        if (entry->frame_sp <=
            UINT32_MAX - complete_frame_words * sizeof(uint32_t)) {
            record.interrupted_sp =
                entry->frame_sp + complete_frame_words * sizeof(uint32_t);
        }

        if (record.interrupted_exception_number != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_INTERRUPTED_HANDLER;
        }
        if ((record.stacked_xpsr & (1UL << 24)) != 0U &&
            core_pc_plausible(record.stacked_pc) &&
            core_lr_plausible(record.stacked_lr)) {
            record.flags |= CRASH_FORENSICS_FLAG_FRAME_CONTENT_PLAUSIBLE;
        }
    }

    record.stack_base = entry->frame_sp;
    if (stacking_fault) {
        record.stack_skip_reason = CRASH_FORENSICS_SKIP_STACKING_FAULT;
    } else {
        record.stack_word_count = capture_core_stack_words(
            record.stack_base,
            record.stack_words,
            CRASH_FORENSICS_CORE_STACK_WORDS,
            &record.stack_skip_reason);
        if (record.stack_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_ACTIVE_STACK_CAPTURED;
        }
    }

    record.crc32 = core_record_crc32(record);
    crash_barrier();
    record.committed = CRASH_FORENSICS_CORE_COMMITTED;
    record.committed_inv = ~CRASH_FORENSICS_CORE_COMMITTED;
    record.capture_stage = CRASH_FORENSICS_STAGE_CORE_COMMITTED;
    record.capture_stage_inv = ~CRASH_FORENSICS_STAGE_CORE_COMMITTED;
    crash_barrier();

    arm_dcache_flush_delete(&record, sizeof(record));
    crash_barrier();
}


static uint32_t coherent_prior_sequence(void) {
    if (core_record_crc_valid(g_crash_forensics_core_record)) {
        return g_crash_forensics_core_record.capture_sequence;
    }

    const crash_forensics_record_t& record = g_crash_forensics_record;
    if (record.magic != CRASH_FORENSICS_MAGIC ||
        record.magic_inv != ~CRASH_FORENSICS_MAGIC ||
        record.schema_version != CRASH_FORENSICS_SCHEMA_VERSION ||
        record.record_size != sizeof(crash_forensics_record_t) ||
        record.committed != CRASH_FORENSICS_COMMITTED ||
        record.committed_inv != ~CRASH_FORENSICS_COMMITTED ||
        record_crc32(record) != record.crc32) {
        return 0U;
    }
    return record.capture_sequence;
}

// ============================================================================
// Fault-time recorder
// ============================================================================

extern "C" void crash_forensics_capture_from_entry(
    const crash_forensics_entry_context_t* entry) {
    if (!entry) return;

    crash_forensics_record_t& record = g_crash_forensics_record;
    uint32_t sequence = coherent_prior_sequence() + 1U;
    if (sequence == 0U) sequence = 1U;

    // Phase 1: commit the small, direct-read core record before any MPU walk,
    // executable-window inspection, or extended stack capture can fault.
    capture_core_record_from_entry(entry, sequence);
    core_record_publish_stage(CRASH_FORENSICS_STAGE_EXTENDED_BEGIN);

    // Phase 2: preserve the existing rich recorder as best-effort evidence.
    zero_record(record);
    record.magic = CRASH_FORENSICS_MAGIC;
    record.magic_inv = ~CRASH_FORENSICS_MAGIC;
    record.schema_version = CRASH_FORENSICS_SCHEMA_VERSION;
    record.record_size = sizeof(crash_forensics_record_t);
    record.capture_sequence = sequence;

    record.exception_number = read_ipsr() & 0x1FFU;
    record.exc_return = entry->exc_return;
    record.original_msp = entry->msp;
    record.original_psp = entry->psp;
    record.primask = entry->primask;
    record.basepri = entry->basepri;
    record.faultmask = entry->faultmask;
    record.control = entry->control;
    record.dwt_cyccnt = reg32(REG_DWT_CYCCNT);
    record.cpu_hz = static_cast<uint32_t>(F_CPU_ACTUAL);

    record.r4 = entry->r4;
    record.r5 = entry->r5;
    record.r6 = entry->r6;
    record.r7 = entry->r7;
    record.r8 = entry->r8;
    record.r9 = entry->r9;
    record.r10 = entry->r10;
    record.r11 = entry->r11;

    record.cpuid = reg32(REG_CPUID);
    record.actlr = reg32(REG_ACTLR);
    record.cfsr = reg32(REG_CFSR);
    record.hfsr = reg32(REG_HFSR);
    record.dfsr = reg32(REG_DFSR);
    record.afsr = reg32(REG_AFSR);
    record.mmfar = reg32(REG_MMFAR);
    record.bfar = reg32(REG_BFAR);
    record.shcsr = reg32(REG_SHCSR);
    record.icsr = reg32(REG_ICSR);
    record.vtor = reg32(REG_VTOR);
    record.aircr = reg32(REG_AIRCR);
    record.scr = reg32(REG_SCR);
    record.ccr = reg32(REG_CCR);
    record.shpr1 = reg32(REG_SHPR1);
    record.shpr2 = reg32(REG_SHPR2);
    record.shpr3 = reg32(REG_SHPR3);
    record.cpacr = reg32(REG_CPACR);
    record.demcr = reg32(REG_DEMCR);
    record.dwt_ctrl = reg32(REG_DWT_CTRL);
    record.syst_csr = reg32(REG_SYST_CSR);
    record.syst_rvr = reg32(REG_SYST_RVR);
    record.syst_cvr = reg32(REG_SYST_CVR);
    record.syst_calib = reg32(REG_SYST_CALIB);
    record.fpccr = reg32(REG_FPCCR);
    record.fpcar = reg32(REG_FPCAR);
    record.fpdscr = reg32(REG_FPDSCR);

    record.frame_source = (entry->exc_return & (1UL << 2))
        ? CRASH_FORENSICS_FRAME_PSP
        : CRASH_FORENSICS_FRAME_MSP;
    record.raw_frame_address = entry->frame_sp;

    const bool extended_frame = (entry->exc_return & (1UL << 4)) == 0U;
    if (extended_frame) {
        record.flags |= CRASH_FORENSICS_FLAG_EXTENDED_FP_FRAME;
    }
    if (entry->exc_return & (1UL << 3)) {
        record.flags |= CRASH_FORENSICS_FLAG_RETURN_TO_THREAD;
    }

    const bool stacking_fault = (record.cfsr & CFSR_STACKING_ERROR_MASK) != 0U;
    if (stacking_fault) {
        record.flags |= CRASH_FORENSICS_FLAG_STACKING_FAULT;
    }

    const uint32_t architectural_offset_words =
        extended_frame ? 18U : 0U;
    const uint32_t alternate_offset_words =
        extended_frame ? 0U : 18U;
    const crash_basic_frame_selection_t architectural =
        extended_basic_frame_candidate(entry->frame_sp,
                                       architectural_offset_words);
    const crash_basic_frame_selection_t alternate =
        extended_basic_frame_candidate(entry->frame_sp,
                                       alternate_offset_words);
    const crash_basic_frame_selection_t selected =
        select_basic_frame(architectural, alternate);

    const uint32_t selected_basic_offset_words = selected.offset_words;
    record.basic_frame_address = selected.address;
    const bool frame_readable = selected.readable;
    if (frame_readable) {
        record.flags |= CRASH_FORENSICS_FLAG_FRAME_ADDRESS_READABLE;
    }

    if (frame_readable && !stacking_fault) {
        const volatile uint32_t* const raw_frame =
            reinterpret_cast<volatile const uint32_t*>(entry->frame_sp);
        const volatile uint32_t* const basic =
            reinterpret_cast<volatile const uint32_t*>(
                record.basic_frame_address);

        record.r0 = basic[0];
        record.r1 = basic[1];
        record.r2 = basic[2];
        record.r3 = basic[3];
        record.r12 = basic[4];
        record.stacked_lr = basic[5];
        record.stacked_pc = basic[6];
        record.stacked_xpsr = basic[7];
        record.interrupted_exception_number = record.stacked_xpsr & 0x1FFU;
        record.flags |= CRASH_FORENSICS_FLAG_BASIC_FRAME_VALID;

        const bool xpsr_plausible =
            (record.stacked_xpsr & (1UL << 24)) != 0U;
        const bool pc_plausible = stacked_pc_plausible(record.stacked_pc);
        const bool lr_plausible = stacked_lr_plausible(record.stacked_lr);
        if (xpsr_plausible && pc_plausible && lr_plausible) {
            record.flags |= CRASH_FORENSICS_FLAG_FRAME_CONTENT_PLAUSIBLE;
        } else {
            if (!xpsr_plausible) {
                record.pc_window_skip_reason =
                    CRASH_FORENSICS_SKIP_FRAME_XPSR_IMPLAUSIBLE;
                record.lr_window_skip_reason =
                    CRASH_FORENSICS_SKIP_FRAME_XPSR_IMPLAUSIBLE;
            } else {
                if (!pc_plausible) {
                    record.pc_window_skip_reason =
                        CRASH_FORENSICS_SKIP_PC_IMPLAUSIBLE;
                }
                if (!lr_plausible) {
                    record.lr_window_skip_reason =
                        CRASH_FORENSICS_SKIP_LR_IMPLAUSIBLE;
                }
            }
        }

        if (record.interrupted_exception_number != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_INTERRUPTED_HANDLER;
        }
        if (record.stacked_xpsr & (1UL << 9)) {
            record.flags |= CRASH_FORENSICS_FLAG_STACK_ALIGNMENT_WORD;
        }

        if (extended_frame &&
            selected_basic_offset_words == 18U && stack_span_readable(
                entry->frame_sp,
                CRASH_FORENSICS_FP_FRAME_WORDS * sizeof(uint32_t))) {
            record.fp_frame_word_count = CRASH_FORENSICS_FP_FRAME_WORDS;
            for (size_t i = 0; i < CRASH_FORENSICS_FP_FRAME_WORDS; ++i) {
                record.fp_frame_words[i] = raw_frame[i];
            }
            record.flags |= CRASH_FORENSICS_FLAG_FP_FRAME_CAPTURED;
        }
    }

    core_record_publish_stage(CRASH_FORENSICS_STAGE_EXTENDED_STACKS);

    record.active_stack_base = entry->frame_sp;
    if (stacking_fault) {
        record.active_stack_skip_reason =
            CRASH_FORENSICS_SKIP_STACKING_FAULT;
    } else {
        record.active_stack_word_count = capture_stack_words(
            record.active_stack_base,
            record.active_stack_words,
            CRASH_FORENSICS_ACTIVE_STACK_WORDS,
            &record.active_stack_skip_reason);
        if (record.active_stack_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_ACTIVE_STACK_CAPTURED;
        }
    }

    record.other_stack_base =
        (record.frame_source == CRASH_FORENSICS_FRAME_PSP)
            ? entry->msp
            : entry->psp;
    if (record.other_stack_base == record.active_stack_base &&
        record.other_stack_base != 0U) {
        record.other_stack_skip_reason =
            CRASH_FORENSICS_SKIP_DUPLICATE_STACK;
    } else {
        record.other_stack_word_count = capture_stack_words(
            record.other_stack_base,
            record.other_stack_words,
            CRASH_FORENSICS_OTHER_STACK_WORDS,
            &record.other_stack_skip_reason);
        if (record.other_stack_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_OTHER_STACK_CAPTURED;
        }
    }

    core_record_publish_stage(CRASH_FORENSICS_STAGE_EXTENDED_WINDOWS);

    if ((record.flags & CRASH_FORENSICS_FLAG_FRAME_CONTENT_PLAUSIBLE) != 0U) {
        record.pc_window_word_count = capture_centered_executable_window(
            record.stacked_pc,
            false,
            &record.pc_window_base,
            record.pc_window_words,
            CRASH_FORENSICS_PC_WINDOW_WORDS,
            &record.pc_window_skip_reason);
        if (record.pc_window_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_PC_WINDOW_CAPTURED;
        }

        record.lr_window_word_count = capture_centered_executable_window(
            record.stacked_lr,
            true,
            &record.lr_window_base,
            record.lr_window_words,
            CRASH_FORENSICS_LR_WINDOW_WORDS,
            &record.lr_window_skip_reason);
        if (record.lr_window_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_LR_WINDOW_CAPTURED;
        }
    } else if ((record.flags & CRASH_FORENSICS_FLAG_BASIC_FRAME_VALID) == 0U) {
        record.pc_window_skip_reason = stacking_fault
            ? CRASH_FORENSICS_SKIP_STACKING_FAULT
            : CRASH_FORENSICS_SKIP_OUTSIDE_STACK_MEMORY;
        record.lr_window_skip_reason = record.pc_window_skip_reason;
    }

    if (record.active_stack_skip_reason != CRASH_FORENSICS_SKIP_NONE ||
        record.other_stack_skip_reason != CRASH_FORENSICS_SKIP_NONE ||
        record.pc_window_skip_reason != CRASH_FORENSICS_SKIP_NONE ||
        record.lr_window_skip_reason != CRASH_FORENSICS_SKIP_NONE) {
        record.flags |= CRASH_FORENSICS_FLAG_CAPTURE_SKIPPED;
    }

    core_record_publish_stage(CRASH_FORENSICS_STAGE_EXTENDED_NVIC);

    record.vector_hardfault =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(
            _VectorsRam[CRASH_FORENSICS_VECTOR_HARDFAULT]));
    record.vector_memmanage =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(
            _VectorsRam[CRASH_FORENSICS_VECTOR_MEMMANAGE]));
    record.vector_busfault =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(
            _VectorsRam[CRASH_FORENSICS_VECTOR_BUSFAULT]));
    record.vector_usagefault =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(
            _VectorsRam[CRASH_FORENSICS_VECTOR_USAGEFAULT]));

    record.zpnet_fault_entry =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(
            &crash_forensics_fault_entry));
    record.teensy_core_fault_handler =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(
            &unused_interrupt_vector));

    volatile const uint32_t* const nvic_iser =
        reinterpret_cast<volatile const uint32_t*>(REG_NVIC_ISER);
    volatile const uint32_t* const nvic_ispr =
        reinterpret_cast<volatile const uint32_t*>(REG_NVIC_ISPR);
    volatile const uint32_t* const nvic_iabr =
        reinterpret_cast<volatile const uint32_t*>(REG_NVIC_IABR);
    for (size_t i = 0; i < CRASH_FORENSICS_NVIC_WORDS; ++i) {
        record.nvic_iser[i] = nvic_iser[i];
        record.nvic_ispr[i] = nvic_ispr[i];
        record.nvic_iabr[i] = nvic_iabr[i];
    }
    record.flags |= CRASH_FORENSICS_FLAG_NVIC_CAPTURED;

    core_record_publish_stage(CRASH_FORENSICS_STAGE_EXTENDED_MPU);

    record.mpu_type = reg32(REG_MPU_TYPE);
    record.mpu_ctrl = reg32(REG_MPU_CTRL);
    record.mpu_rnr = reg32(REG_MPU_RNR);
    uint32_t mpu_regions = (record.mpu_type >> 8) & 0xFFU;
    if (mpu_regions > CRASH_FORENSICS_MPU_REGIONS) {
        mpu_regions = CRASH_FORENSICS_MPU_REGIONS;
    }
    record.mpu_region_count = mpu_regions;
    for (uint32_t i = 0; i < mpu_regions; ++i) {
        reg32_write(REG_MPU_RNR, i);
        crash_barrier();
        record.mpu_rbar[i] = reg32(REG_MPU_RBAR);
        record.mpu_rasr[i] = reg32(REG_MPU_RASR);
    }
    reg32_write(REG_MPU_RNR, record.mpu_rnr);
    crash_barrier();
    record.flags |= CRASH_FORENSICS_FLAG_MPU_CAPTURED;

    record.crc32 = record_crc32(record);
    crash_barrier();
    record.committed = CRASH_FORENSICS_COMMITTED;
    record.committed_inv = ~CRASH_FORENSICS_COMMITTED;
    crash_barrier();

    arm_dcache_flush_delete(&record, sizeof(record));
    crash_barrier();

    core_record_publish_stage(CRASH_FORENSICS_STAGE_EXTENDED_COMMITTED);
}

// ============================================================================
// Exception entry shim
// ============================================================================
//
// R0-R3, R12, LR, PC and xPSR are already hardware-stacked.  The shim uses
// those volatile registers to establish an emergency MSP, but first preserves
// R4-R11 plus all entry-control facts in a fixed 64-byte context.  After the
// recorder returns, it restores the original MSP and EXC_RETURN, then branches
// into Teensyduino's unused_interrupt_vector.  That core handler creates the
// standard CrashReport, keeps USB alive briefly, and performs the proven reset.
// ============================================================================

extern "C" __attribute__((naked, used, noreturn))
void crash_forensics_fault_entry(void) {
    asm volatile(
        "mrs r0, msp\n"
        "mrs r1, psp\n"
        "mov r2, lr\n"
        "tst r2, #4\n"
        "ite eq\n"
        "moveq r3, r0\n"
        "movne r3, r1\n"

        // If the recorder itself faults, immediately surrender to the core
        // handler rather than recursively attempting another extended capture.
        "ldr r12, =g_crash_forensics_capture_active\n"
        "ldr r12, [r12]\n"
        "cmp r12, #0\n"
        "bne 2f\n"

        // Move onto the dedicated emergency stack before entering C++.
        "ldr r12, =g_crash_forensics_emergency_stack\n"
        "add.w r12, r12, #2048\n"
        "msr msp, r12\n"
        "sub sp, sp, #64\n"

        // Callee-saved registers as they existed at exception entry.
        "str r4,  [sp, #0]\n"
        "str r5,  [sp, #4]\n"
        "str r6,  [sp, #8]\n"
        "str r7,  [sp, #12]\n"
        "str r8,  [sp, #16]\n"
        "str r9,  [sp, #20]\n"
        "str r10, [sp, #24]\n"
        "str r11, [sp, #28]\n"

        // Selected frame, EXC_RETURN and original stack pointers.
        "str r3, [sp, #32]\n"
        "str r2, [sp, #36]\n"
        "str r0, [sp, #40]\n"
        "str r1, [sp, #44]\n"

        // Publish the recursion guard only after every live callee-saved
        // register has been copied to the emergency stack.
        "ldr r12, =g_crash_forensics_capture_active\n"
        "movs r3, #1\n"
        "str r3, [r12]\n"

        // Original interrupt/control masks, captured before CPSID i.
        "mrs r3, primask\n"
        "str r3, [sp, #48]\n"
        "mrs r3, basepri\n"
        "str r3, [sp, #52]\n"
        "mrs r3, faultmask\n"
        "str r3, [sp, #56]\n"
        "mrs r3, control\n"
        "str r3, [sp, #60]\n"
        "cpsid i\n"

        "mov r0, sp\n"
        "bl crash_forensics_capture_from_entry\n"

        // Restore the core handler's expected entry state.
        "ldr r1, [sp, #40]\n"
        "ldr r2, [sp, #36]\n"
        "msr msp, r1\n"
        "mov lr, r2\n"
        "b unused_interrupt_vector\n"

        // Recursive-fault escape: MSP and LR are still the nested exception's
        // native values because this path did not switch stacks.
        "2:\n"
        "b unused_interrupt_vector\n"
    );
}

// ============================================================================
// Retained stack watchpoint (DWT comparator 1 + DebugMonitor)
// ============================================================================
//
// See crash_forensics.h for doctrine.  The watch window and enable are
// compile-time constants so a diagnostic build can retarget them in one edit.
// The watched window derives from the reproducible boot-crash reconstructions:
// crash #1 poisoned saved-LR slots at 0x20037EA4/0x20037EAC (mallinfo path);
// crash #2 poisoned a slot near 0x20037D24 (payload_get_info path).  MASK = 10
// covers the aligned 1 KB window 0x20037C00..0x20037FFF spanning both.

static constexpr bool     CRASH_STACK_WATCH_ENABLED = true;
static constexpr uint32_t CRASH_STACK_WATCH_ADDRESS = 0x20037C00UL;
static constexpr uint32_t CRASH_STACK_WATCH_MASK = 10UL;
static constexpr uint32_t CRASH_STACK_WATCH_BYTES = 1UL << CRASH_STACK_WATCH_MASK;

static constexpr uint32_t CRASH_STACK_WATCH_MAGIC = 0x5A505357UL;  // "ZPSW"
static constexpr uint32_t CRASH_STACK_WATCH_SCHEMA_VERSION = 1U;

static constexpr uintptr_t REG_DHCSR = 0xE000EDF0UL;
static constexpr uint32_t DHCSR_C_DEBUGEN = 1UL << 0;

static constexpr uint32_t DEMCR_TRCENA = 1UL << 24;
static constexpr uint32_t DEMCR_MON_EN = 1UL << 16;
static constexpr uint32_t DFSR_DWTTRAP = 1UL << 2;
static constexpr uint32_t DWT_FUNCTION_WRITE_WATCH = 0x6UL;

struct alignas(32) crash_stack_watch_bank_t {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t schema_version;
    uint32_t capacity;
    uint32_t hits_total;
    uint32_t anomalous_total;
    uint32_t reserved0;
    uint32_t reserved1;
    crash_stack_watch_entry_t first_anomalous;
    crash_stack_watch_entry_t entries[CRASH_STACK_WATCH_ENTRIES];
    uint32_t reserved_tail[4];
};

static_assert((sizeof(crash_stack_watch_bank_t) % 32U) == 0U,
              "Stack watch bank must occupy complete Cortex-M7 cache lines");

alignas(32) static crash_stack_watch_bank_t g_crash_stack_watch_live DMAMEM;
alignas(32) static crash_stack_watch_bank_t g_crash_stack_watch_retained DMAMEM;

// Ordinary BSS: cleared each boot, so the latch runs exactly once per boot.
static bool g_crash_stack_watch_boot_latched = false;
static bool g_crash_stack_watch_armed = false;
static uint32_t g_crash_stack_watch_skip_reason = CRASH_STACK_WATCH_SKIP_NONE;
static uint32_t g_crash_stack_watch_arm_attempts = 0U;
static uint32_t g_crash_stack_watch_last_dhcsr = 0U;
static uint32_t g_crash_stack_watch_effective_comp = 0U;
static uint32_t g_crash_stack_watch_effective_mask = 0U;

extern "C" void crash_stack_watch_debugmon_entry(void);

static bool crash_stack_watch_bank_valid(const crash_stack_watch_bank_t& bank) {
    return bank.magic == CRASH_STACK_WATCH_MAGIC &&
           (bank.magic ^ bank.magic_inv) == 0xFFFFFFFFUL &&
           bank.schema_version == CRASH_STACK_WATCH_SCHEMA_VERSION &&
           bank.capacity == CRASH_STACK_WATCH_ENTRIES;
}

static void crash_stack_watch_flush_bank(crash_stack_watch_bank_t& bank) {
    arm_dcache_flush(&bank, sizeof(bank));
}

static void crash_stack_watch_initialize_live(void) {
    crash_stack_watch_bank_t& bank = g_crash_stack_watch_live;
    for (size_t i = 0; i < sizeof(bank) / sizeof(uint32_t); ++i) {
        reinterpret_cast<volatile uint32_t*>(&bank)[i] = 0U;
    }
    bank.schema_version = CRASH_STACK_WATCH_SCHEMA_VERSION;
    bank.capacity = CRASH_STACK_WATCH_ENTRIES;
    bank.magic_inv = ~CRASH_STACK_WATCH_MAGIC;
    bank.magic = CRASH_STACK_WATCH_MAGIC;
    crash_stack_watch_flush_bank(bank);
}

// Latch the previous boot's final hit transcript before current-boot hits can
// overwrite it, then start a fresh live ring.  DMAMEM survives the warm reboot;
// the DebugMonitor handler flushes every mutation so a sudden reset cannot
// strand entries in the data cache.
static void crash_stack_watch_boot_latch(void) {
    if (g_crash_stack_watch_boot_latched) return;
    g_crash_stack_watch_boot_latched = true;

    if (crash_stack_watch_bank_valid(g_crash_stack_watch_live)) {
        g_crash_stack_watch_retained = g_crash_stack_watch_live;
        crash_stack_watch_flush_bank(g_crash_stack_watch_retained);
    }
    crash_stack_watch_initialize_live();
}

static void crash_stack_watch_arm(void) {
    if (!CRASH_STACK_WATCH_ENABLED) {
        g_crash_stack_watch_skip_reason = CRASH_STACK_WATCH_SKIP_DISABLED;
        return;
    }

    crash_stack_watch_boot_latch();

    // Monitor-mode debug only engages while halting debug is off.  The Teensy
    // bootloader chip programs this part over the debug port and can leave
    // DHCSR.C_DEBUGEN latched after a flash; with it set, MON_EN is ignored
    // and the first watchpoint match HALTS the core.  Software writes cannot
    // modify C_DEBUGEN (only the debug port or a power-on reset can), so a
    // latched debug session is a retryable decline, not a failure:
    // crash_stack_watch_service() re-attempts from foreground paths and arms
    // the moment the session is released.
    g_crash_stack_watch_arm_attempts++;
    g_crash_stack_watch_last_dhcsr = reg32(REG_DHCSR);
    if ((g_crash_stack_watch_last_dhcsr & DHCSR_C_DEBUGEN) != 0U) {
        g_crash_stack_watch_skip_reason =
            CRASH_STACK_WATCH_SKIP_HALTING_DEBUG_ACTIVE;
        return;
    }

    _VectorsRam[CRASH_STACK_WATCH_VECTOR_DEBUGMON] =
        crash_stack_watch_debugmon_entry;

    reg32_write(REG_DEMCR, reg32(REG_DEMCR) | DEMCR_TRCENA | DEMCR_MON_EN);
    reg32_write(REG_DWT_COMP1, CRASH_STACK_WATCH_ADDRESS);
    reg32_write(REG_DWT_MASK1, CRASH_STACK_WATCH_MASK);
    crash_barrier();

    // Verify the hardware accepted the address mask.  If not, fall back to
    // watching the single highest-value word exactly: _mallinfo_r's saved-LR
    // slot in the reconstructed frame.
    g_crash_stack_watch_effective_mask = reg32(REG_DWT_MASK1);
    if (g_crash_stack_watch_effective_mask != CRASH_STACK_WATCH_MASK) {
        reg32_write(REG_DWT_MASK1, 0U);
        reg32_write(REG_DWT_COMP1, 0x20037EA4UL);
        crash_barrier();
        g_crash_stack_watch_effective_mask = reg32(REG_DWT_MASK1);
    }
    g_crash_stack_watch_effective_comp = reg32(REG_DWT_COMP1);

    (void)reg32(REG_DWT_FUNCTION1);  // read clears a stale MATCHED bit
    reg32_write(REG_DWT_FUNCTION1, DWT_FUNCTION_WRITE_WATCH);
    crash_barrier();

    g_crash_stack_watch_skip_reason = CRASH_STACK_WATCH_SKIP_NONE;
    g_crash_stack_watch_armed = true;
}

// Cheap idempotent retry: two register reads when already armed or still
// declined, the full arm sequence the first time halting debug reads clear.
// Safe from any foreground context; allocation-free, logging-free.
void crash_stack_watch_service(void) {
    if (!CRASH_STACK_WATCH_ENABLED) return;
    if (g_crash_stack_watch_armed) return;
    crash_stack_watch_arm();
}

// Naked shim: select the stacked frame, preserve alignment, call the body.
extern "C" __attribute__((naked)) void crash_stack_watch_debugmon_entry(void) {
    __asm__ volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mov r1, lr\n"
        "push {r4, lr}\n"
        "bl crash_stack_watch_debugmon_body\n"
        "pop {r4, pc}\n");
}

extern "C" void crash_stack_watch_debugmon_body(uint32_t* frame,
                                                uint32_t exc_return) {
    // One-shot disarm so this handler's own stores cannot re-match, then
    // acknowledge the debug event.
    (void)reg32(REG_DWT_FUNCTION1);  // read clears MATCHED
    reg32_write(REG_DWT_FUNCTION1, 0U);
    reg32_write(REG_DFSR, DFSR_DWTTRAP);

    crash_stack_watch_bank_t& bank = g_crash_stack_watch_live;
    if (!crash_stack_watch_bank_valid(bank)) {
        reg32_write(REG_DWT_FUNCTION1, DWT_FUNCTION_WRITE_WATCH);
        return;
    }

    uint32_t pc = 0U;
    uint32_t lr = 0U;
    uint32_t xpsr = 0U;
    uint32_t writer_sp = 0U;
    if (frame != nullptr) {
        pc = frame[6];
        lr = frame[5];
        xpsr = frame[7];
        uint32_t frame_bytes = ((exc_return & (1UL << 4)) == 0U) ? 0x68U : 0x20U;
        if ((xpsr & (1UL << 9)) != 0U) frame_bytes += 4U;
        writer_sp = reinterpret_cast<uint32_t>(frame) + frame_bytes;
    }

    const uint32_t sequence = ++bank.hits_total;
    crash_stack_watch_entry_t& entry =
        bank.entries[(sequence - 1U) % CRASH_STACK_WATCH_ENTRIES];

    entry.sequence = 0U;
    entry.sequence_inv = 0U;
    entry.dwt = reg32(REG_DWT_CYCCNT);
    entry.pc = pc;
    entry.lr = lr;
    entry.xpsr = xpsr;
    entry.writer_sp = writer_sp;
    entry.exc_return = exc_return;
    volatile const uint32_t* watched =
        reinterpret_cast<volatile const uint32_t*>(CRASH_STACK_WATCH_ADDRESS);
    entry.watched[0] = watched[0];
    entry.watched[1] = watched[1];
    entry.watched[2] = watched[2];
    entry.watched[3] = watched[3];
    entry.sequence_inv = ~sequence;
    entry.sequence = sequence;

    // Lawful code never stores below its own live frame.  A writer whose SP
    // sits entirely above the watched window is the rogue store.
    if (writer_sp >= CRASH_STACK_WATCH_ADDRESS + CRASH_STACK_WATCH_BYTES) {
        bank.anomalous_total++;
        if (bank.first_anomalous.sequence == 0U) {
            bank.first_anomalous = entry;
        }
    }

    crash_stack_watch_flush_bank(bank);
    reg32_write(REG_DWT_FUNCTION1, DWT_FUNCTION_WRITE_WATCH);
}

static void crash_stack_watch_snapshot_bank(
    const crash_stack_watch_bank_t& bank,
    crash_stack_watch_bank_snapshot_t* out) {
    for (size_t i = 0; i < sizeof(*out); ++i) {
        reinterpret_cast<uint8_t*>(out)[i] = 0U;
    }
    out->valid = crash_stack_watch_bank_valid(bank);
    if (!out->valid) return;

    out->hits_total = bank.hits_total;
    out->anomalous_total = bank.anomalous_total;
    out->first_anomalous = bank.first_anomalous;

    for (uint32_t i = 0U; i < CRASH_STACK_WATCH_ENTRIES; ++i) {
        const crash_stack_watch_entry_t& candidate = bank.entries[i];
        if (candidate.sequence == 0U ||
            (candidate.sequence ^ candidate.sequence_inv) != 0xFFFFFFFFUL) {
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

FLASHMEM void crash_stack_watch_snapshot(crash_stack_watch_snapshot_t* out) {
    if (!out) return;
    crash_stack_watch_boot_latch();

    out->enabled = CRASH_STACK_WATCH_ENABLED;
    out->armed = g_crash_stack_watch_armed;
    out->arm_skip_reason = g_crash_stack_watch_skip_reason;
    out->arm_attempts = g_crash_stack_watch_arm_attempts;
    out->last_dhcsr = g_crash_stack_watch_last_dhcsr;
    out->watch_address = CRASH_STACK_WATCH_ADDRESS;
    out->watch_bytes = CRASH_STACK_WATCH_BYTES;
    out->effective_comp_address = g_crash_stack_watch_effective_comp;
    out->effective_mask = g_crash_stack_watch_effective_mask;

    const uint32_t saved_primask = read_primask();
    disable_interrupts();
    crash_stack_watch_snapshot_bank(g_crash_stack_watch_live, &out->live);
    crash_stack_watch_snapshot_bank(g_crash_stack_watch_retained,
                                    &out->retained);
    write_primask(saved_primask);
}

FLASHMEM const char* crash_stack_watch_skip_reason_name(uint32_t reason) {
    switch (reason) {
        case CRASH_STACK_WATCH_SKIP_NONE: return "NONE";
        case CRASH_STACK_WATCH_SKIP_DISABLED: return "DISABLED";
        case CRASH_STACK_WATCH_SKIP_HALTING_DEBUG_ACTIVE:
            return "HALTING_DEBUG_ACTIVE";
        default: return "UNKNOWN";
    }
}

FLASHMEM void crash_stack_watch_clear_retained(void) {
    const uint32_t saved_primask = read_primask();
    disable_interrupts();
    for (size_t i = 0;
         i < sizeof(g_crash_stack_watch_retained) / sizeof(uint32_t); ++i) {
        reinterpret_cast<volatile uint32_t*>(
            &g_crash_stack_watch_retained)[i] = 0U;
    }
    crash_stack_watch_flush_bank(g_crash_stack_watch_retained);
    write_primask(saved_primask);
}

// ============================================================================
// Stack altitude tripwire (SP-floor overlap detector)
// ============================================================================

static constexpr uint32_t CRASH_STACK_TRIPWIRE_MAGIC = 0x5A505457UL;  // "ZPTW"
static constexpr uint32_t CRASH_STACK_TRIPWIRE_SCHEMA_VERSION = 1U;

struct alignas(32) crash_stack_tripwire_bank_t {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t schema_version;
    uint32_t capacity;
    uint32_t violation_total;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t reserved2;
    crash_stack_tripwire_entry_t first;
    crash_stack_tripwire_entry_t entries[CRASH_STACK_TRIPWIRE_ENTRIES];
    uint32_t reserved_tail[4];
};

static_assert((sizeof(crash_stack_tripwire_bank_t) % 32U) == 0U,
              "Stack tripwire bank must occupy complete Cortex-M7 cache lines");

alignas(32) static crash_stack_tripwire_bank_t g_crash_stack_tripwire_live DMAMEM;
alignas(32) static crash_stack_tripwire_bank_t g_crash_stack_tripwire_retained DMAMEM;

// Ordinary BSS: cleared each boot.
volatile uint32_t g_crash_stack_tripwire_floor = 0U;
volatile uint32_t g_crash_stack_tripwire_floor_publish_count = 0U;
static bool g_crash_stack_tripwire_boot_latched = false;

static bool crash_stack_tripwire_bank_valid(
    const crash_stack_tripwire_bank_t& bank) {
    return bank.magic == CRASH_STACK_TRIPWIRE_MAGIC &&
           (bank.magic ^ bank.magic_inv) == 0xFFFFFFFFUL &&
           bank.schema_version == CRASH_STACK_TRIPWIRE_SCHEMA_VERSION &&
           bank.capacity == CRASH_STACK_TRIPWIRE_ENTRIES;
}

static void crash_stack_tripwire_flush_bank(crash_stack_tripwire_bank_t& bank) {
    arm_dcache_flush(&bank, sizeof(bank));
}

static void crash_stack_tripwire_initialize_live(void) {
    crash_stack_tripwire_bank_t& bank = g_crash_stack_tripwire_live;
    for (size_t i = 0; i < sizeof(bank) / sizeof(uint32_t); ++i) {
        reinterpret_cast<volatile uint32_t*>(&bank)[i] = 0U;
    }
    bank.schema_version = CRASH_STACK_TRIPWIRE_SCHEMA_VERSION;
    bank.capacity = CRASH_STACK_TRIPWIRE_ENTRIES;
    bank.magic_inv = ~CRASH_STACK_TRIPWIRE_MAGIC;
    bank.magic = CRASH_STACK_TRIPWIRE_MAGIC;
    crash_stack_tripwire_flush_bank(bank);
}

static void crash_stack_tripwire_boot_latch(void) {
    if (g_crash_stack_tripwire_boot_latched) return;
    g_crash_stack_tripwire_boot_latched = true;

    if (crash_stack_tripwire_bank_valid(g_crash_stack_tripwire_live)) {
        g_crash_stack_tripwire_retained = g_crash_stack_tripwire_live;
        crash_stack_tripwire_flush_bank(g_crash_stack_tripwire_retained);
    }
    crash_stack_tripwire_initialize_live();
}

// Violation latcher.  Runs in ISR context; scalar-only, allocation-free.
// Rare by construction (never fires in a healthy system), so the full-bank
// cache flush is acceptable.
void crash_stack_tripwire_latch(uint32_t site,
                                uint32_t msp,
                                uint32_t exc_return,
                                uint32_t floor,
                                uint32_t frame_top_estimate) {
    crash_stack_tripwire_bank_t& bank = g_crash_stack_tripwire_live;
    if (!crash_stack_tripwire_bank_valid(bank)) return;

    const uint32_t sequence = ++bank.violation_total;
    crash_stack_tripwire_entry_t& entry =
        bank.entries[(sequence - 1U) % CRASH_STACK_TRIPWIRE_ENTRIES];

    entry.sequence = 0U;
    entry.sequence_inv = 0U;
    entry.site = site;
    entry.msp = msp;
    entry.exc_return = exc_return;
    entry.floor = floor;
    entry.frame_top_estimate = frame_top_estimate;
    entry.dwt = reg32(REG_DWT_CYCCNT);
    entry.ipsr = read_ipsr();
    entry.reserved[0] = 0U;
    entry.reserved[1] = 0U;
    entry.reserved[2] = 0U;
    entry.sequence_inv = ~sequence;
    entry.sequence = sequence;

    if (bank.first.sequence == 0U) {
        bank.first = entry;
    }

    crash_stack_tripwire_flush_bank(bank);
}

static void crash_stack_tripwire_snapshot_bank(
    const crash_stack_tripwire_bank_t& bank,
    crash_stack_tripwire_bank_snapshot_t* out) {
    for (size_t i = 0; i < sizeof(*out); ++i) {
        reinterpret_cast<uint8_t*>(out)[i] = 0U;
    }
    out->valid = crash_stack_tripwire_bank_valid(bank);
    if (!out->valid) return;

    out->violation_total = bank.violation_total;
    out->first = bank.first;

    for (uint32_t i = 0U; i < CRASH_STACK_TRIPWIRE_ENTRIES; ++i) {
        const crash_stack_tripwire_entry_t& candidate = bank.entries[i];
        if (candidate.sequence == 0U ||
            (candidate.sequence ^ candidate.sequence_inv) != 0xFFFFFFFFUL) {
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

FLASHMEM void crash_stack_tripwire_snapshot(crash_stack_tripwire_snapshot_t* out) {
    if (!out) return;
    crash_stack_tripwire_boot_latch();

    out->floor_active_now = (g_crash_stack_tripwire_floor != 0U);
    out->floor_now = g_crash_stack_tripwire_floor;
    out->floor_publish_count = g_crash_stack_tripwire_floor_publish_count;

    const uint32_t saved_primask = read_primask();
    disable_interrupts();
    crash_stack_tripwire_snapshot_bank(g_crash_stack_tripwire_live, &out->live);
    crash_stack_tripwire_snapshot_bank(g_crash_stack_tripwire_retained,
                                       &out->retained);
    write_primask(saved_primask);
}

FLASHMEM void crash_stack_tripwire_clear_retained(void) {
    const uint32_t saved_primask = read_primask();
    disable_interrupts();
    for (size_t i = 0;
         i < sizeof(g_crash_stack_tripwire_retained) / sizeof(uint32_t); ++i) {
        reinterpret_cast<volatile uint32_t*>(
            &g_crash_stack_tripwire_retained)[i] = 0U;
    }
    crash_stack_tripwire_flush_bank(g_crash_stack_tripwire_retained);
    write_primask(saved_primask);
}

FLASHMEM const char* crash_stack_tripwire_site_name(uint32_t site) {
    switch (site) {
        case CRASH_TRIPWIRE_SITE_HANDOFF:  return "HANDOFF_ISR";
        case CRASH_TRIPWIRE_SITE_QTIMER1:  return "QTIMER1_ISR";
        case CRASH_TRIPWIRE_SITE_QTIMER2:  return "QTIMER2_ISR";
        case CRASH_TRIPWIRE_SITE_QTIMER3:  return "QTIMER3_ISR";
        case CRASH_TRIPWIRE_SITE_PPS_GPIO: return "PPS_GPIO_ISR";
        default:                           return "UNKNOWN";
    }
}

// ============================================================================
// Installation, validation and reporting API
// ============================================================================

FLASHMEM void crash_forensics_install(void) {
    const uint32_t saved_primask = read_primask();
    disable_interrupts();

    _VectorsRam[CRASH_FORENSICS_VECTOR_HARDFAULT] = crash_forensics_fault_entry;
    _VectorsRam[CRASH_FORENSICS_VECTOR_MEMMANAGE] = crash_forensics_fault_entry;
    _VectorsRam[CRASH_FORENSICS_VECTOR_BUSFAULT] = crash_forensics_fault_entry;
    _VectorsRam[CRASH_FORENSICS_VECTOR_USAGEFAULT] = crash_forensics_fault_entry;

    crash_barrier();
    g_crash_forensics_capture_active = 0U;

    crash_stack_watch_arm();
    crash_stack_tripwire_boot_latch();

    write_primask(saved_primask);
}

FLASHMEM bool crash_forensics_installed(void) {
    return _VectorsRam[CRASH_FORENSICS_VECTOR_HARDFAULT] ==
               crash_forensics_fault_entry &&
           _VectorsRam[CRASH_FORENSICS_VECTOR_MEMMANAGE] ==
               crash_forensics_fault_entry &&
           _VectorsRam[CRASH_FORENSICS_VECTOR_BUSFAULT] ==
               crash_forensics_fault_entry &&
           _VectorsRam[CRASH_FORENSICS_VECTOR_USAGEFAULT] ==
               crash_forensics_fault_entry;
}

FLASHMEM void crash_forensics_get_status(crash_forensics_status_t* out) {
    if (!out) return;

    *out = crash_forensics_status_t{};
    out->installed = crash_forensics_installed();

    const crash_forensics_core_record_t& core =
        g_crash_forensics_core_record;
    out->core_stored_crc = core.crc32;
    out->core_present =
        core.magic == CRASH_FORENSICS_CORE_MAGIC &&
        core.magic_inv == ~CRASH_FORENSICS_CORE_MAGIC;
    if (out->core_present) {
        out->core_header_valid = core_record_header_valid(core);
        if (out->core_header_valid) {
            out->core_computed_crc = core_record_crc32(core);
            out->core_crc_valid =
                out->core_computed_crc == core.crc32;
        }
    }

    const crash_forensics_record_t& record = g_crash_forensics_record;
    out->stored_crc = record.crc32;
    out->extended_present =
        record.magic == CRASH_FORENSICS_MAGIC &&
        record.magic_inv == ~CRASH_FORENSICS_MAGIC;
    if (out->extended_present) {
        out->header_valid =
            record.schema_version == CRASH_FORENSICS_SCHEMA_VERSION &&
            record.record_size == sizeof(crash_forensics_record_t) &&
            record.committed == CRASH_FORENSICS_COMMITTED &&
            record.committed_inv == ~CRASH_FORENSICS_COMMITTED;
        if (out->header_valid) {
            out->computed_crc = record_crc32(record);
            out->crc_valid = out->computed_crc == record.crc32;
        }
    }

    out->present = out->core_present || out->extended_present;
}

FLASHMEM const crash_forensics_core_record_t*
crash_forensics_core_record(void) {
    crash_forensics_status_t status{};
    crash_forensics_get_status(&status);
    return status.core_crc_valid
        ? &g_crash_forensics_core_record
        : nullptr;
}

FLASHMEM const crash_forensics_record_t* crash_forensics_record(void) {
    crash_forensics_status_t status{};
    crash_forensics_get_status(&status);
    return status.crc_valid ? &g_crash_forensics_record : nullptr;
}

FLASHMEM void crash_forensics_clear(void) {
    const uint32_t saved_primask = read_primask();
    disable_interrupts();
    zero_core_record(g_crash_forensics_core_record);
    zero_record(g_crash_forensics_record);
    crash_barrier();
    arm_dcache_flush_delete(&g_crash_forensics_core_record,
                            sizeof(g_crash_forensics_core_record));
    arm_dcache_flush_delete(&g_crash_forensics_record,
                            sizeof(g_crash_forensics_record));
    crash_barrier();
    write_primask(saved_primask);
}

FLASHMEM const char* crash_forensics_exception_name(uint32_t exception_number) {
    switch (exception_number) {
        case 2U: return "NMI";
        case 3U: return "HARDFAULT";
        case 4U: return "MEMMANAGE";
        case 5U: return "BUSFAULT";
        case 6U: return "USAGEFAULT";
        default: return "UNKNOWN";
    }
}

FLASHMEM const char* crash_forensics_frame_source_name(uint32_t frame_source) {
    switch (frame_source) {
        case CRASH_FORENSICS_FRAME_MSP: return "MSP";
        case CRASH_FORENSICS_FRAME_PSP: return "PSP";
        default: return "NONE";
    }
}

FLASHMEM const char* crash_forensics_capture_stage_name(uint32_t stage) {
    switch ((crash_forensics_capture_stage_t)stage) {
        case CRASH_FORENSICS_STAGE_CORE_BEGIN:
            return "CORE_BEGIN";
        case CRASH_FORENSICS_STAGE_CORE_COMMITTED:
            return "CORE_COMMITTED";
        case CRASH_FORENSICS_STAGE_EXTENDED_BEGIN:
            return "EXTENDED_BEGIN";
        case CRASH_FORENSICS_STAGE_EXTENDED_STACKS:
            return "EXTENDED_STACKS";
        case CRASH_FORENSICS_STAGE_EXTENDED_WINDOWS:
            return "EXTENDED_WINDOWS";
        case CRASH_FORENSICS_STAGE_EXTENDED_NVIC:
            return "EXTENDED_NVIC";
        case CRASH_FORENSICS_STAGE_EXTENDED_MPU:
            return "EXTENDED_MPU";
        case CRASH_FORENSICS_STAGE_EXTENDED_COMMITTED:
            return "EXTENDED_COMMITTED";
        default:
            return "NONE";
    }
}

FLASHMEM const char* crash_forensics_capture_skip_reason_name(uint32_t reason) {
    switch (reason) {
        case CRASH_FORENSICS_SKIP_NONE: return "NONE";
        case CRASH_FORENSICS_SKIP_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case CRASH_FORENSICS_SKIP_ZERO_ADDRESS: return "ZERO_ADDRESS";
        case CRASH_FORENSICS_SKIP_UNALIGNED_ADDRESS: return "UNALIGNED_ADDRESS";
        case CRASH_FORENSICS_SKIP_ADDRESS_OVERFLOW: return "ADDRESS_OVERFLOW";
        case CRASH_FORENSICS_SKIP_OUTSIDE_STACK_MEMORY: return "OUTSIDE_STACK_MEMORY";
        case CRASH_FORENSICS_SKIP_OUTSIDE_EXECUTABLE_MEMORY:
            return "OUTSIDE_EXECUTABLE_MEMORY";
        case CRASH_FORENSICS_SKIP_MPU_DENIED: return "MPU_DENIED";
        case CRASH_FORENSICS_SKIP_STACKING_FAULT: return "STACKING_FAULT";
        case CRASH_FORENSICS_SKIP_FRAME_XPSR_IMPLAUSIBLE:
            return "FRAME_XPSR_IMPLAUSIBLE";
        case CRASH_FORENSICS_SKIP_PC_IMPLAUSIBLE: return "PC_IMPLAUSIBLE";
        case CRASH_FORENSICS_SKIP_LR_IMPLAUSIBLE: return "LR_IMPLAUSIBLE";
        case CRASH_FORENSICS_SKIP_DUPLICATE_STACK: return "DUPLICATE_STACK";
        default: return "UNKNOWN";
    }
}

// Install after Teensyduino has created its RAM vector table but before global
// C++ constructors run.  setup() installs once more as an explicit boot-order
// assertion and to repair any constructor that might have replaced a vector.
extern "C" FLASHMEM void startup_late_hook(void) {
    crash_forensics_install();
}
