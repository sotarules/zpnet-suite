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

alignas(32) static crash_forensics_record_t g_crash_forensics_record DMAMEM;

static constexpr uint32_t CRASH_FORENSICS_MAGIC = 0x5A504346UL;     // "ZPCF"
static constexpr uint32_t CRASH_FORENSICS_COMMITTED = 0x434F4D54UL; // "COMT"
static constexpr size_t CRASH_FORENSICS_VECTOR_HARDFAULT = 3U;
static constexpr size_t CRASH_FORENSICS_VECTOR_MEMMANAGE = 4U;
static constexpr size_t CRASH_FORENSICS_VECTOR_BUSFAULT = 5U;
static constexpr size_t CRASH_FORENSICS_VECTOR_USAGEFAULT = 6U;

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

static uint32_t record_crc32(const crash_forensics_record_t& record) {
    const uint32_t* p = reinterpret_cast<const uint32_t*>(&record);
    const uint32_t* const end = reinterpret_cast<const uint32_t*>(
        reinterpret_cast<const uint8_t*>(&record) +
        offsetof(crash_forensics_record_t, crc32));

    uint32_t crc = 0xFFFFFFFFUL;
    while (p < end) {
        crc ^= *p++;
        for (uint32_t bit = 0; bit < 32U; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return crc;
}

static bool range_contains(uintptr_t begin,
                           uintptr_t end,
                           uintptr_t address,
                           size_t bytes) {
    if (end <= begin || address < begin || address >= end) return false;
    if (bytes > static_cast<size_t>(end - address)) return false;
    return true;
}

static bool stack_span_readable(uintptr_t address, size_t bytes) {
    const uintptr_t dtcm_end = reinterpret_cast<uintptr_t>(&_estack);
    return range_contains(0x20000000UL, dtcm_end, address, bytes) ||
           range_contains(0x20200000UL, 0x20280000UL, address, bytes);
}

static bool general_span_readable(uintptr_t address, size_t bytes) {
    const uintptr_t itcm_begin = reinterpret_cast<uintptr_t>(&_stext);
    const uintptr_t itcm_end = reinterpret_cast<uintptr_t>(&_etext);
    const uintptr_t dtcm_end = reinterpret_cast<uintptr_t>(&_estack);
    return range_contains(itcm_begin, itcm_end, address, bytes) ||
           range_contains(0x20000000UL, dtcm_end, address, bytes) ||
           range_contains(0x20200000UL, 0x20280000UL, address, bytes) ||
           range_contains(0x60000000UL, 0x61000000UL, address, bytes);
}

static uint32_t capture_words(uint32_t base,
                              uint32_t* destination,
                              size_t capacity,
                              bool stack_only) {
    if (!destination || capacity == 0U || (base & 3U) != 0U) return 0U;

    uint32_t count = 0U;
    for (size_t i = 0; i < capacity; ++i) {
        const uintptr_t address = static_cast<uintptr_t>(base) + i * 4U;
        const bool readable = stack_only
            ? stack_span_readable(address, sizeof(uint32_t))
            : general_span_readable(address, sizeof(uint32_t));
        if (!readable) break;
        destination[i] = *reinterpret_cast<volatile const uint32_t*>(address);
        ++count;
    }
    return count;
}

static uint32_t capture_centered_window(uint32_t center,
                                        uint32_t* out_base,
                                        uint32_t* destination,
                                        size_t capacity) {
    if (out_base) *out_base = 0U;
    if (!destination || capacity == 0U || center == 0U) return 0U;

    const uint32_t aligned = center & ~3UL;
    const uint32_t before_words = static_cast<uint32_t>(capacity / 2U);
    uint32_t base = aligned;
    const uint32_t before_bytes = before_words * 4U;
    if (aligned >= before_bytes) base = aligned - before_bytes;

    uint32_t count = capture_words(base, destination, capacity, false);
    if (count == 0U && base != aligned) {
        base = aligned;
        count = capture_words(base, destination, capacity, false);
    }
    if (count != 0U && out_base) *out_base = base;
    return count;
}

static uint32_t coherent_prior_sequence(void) {
    const crash_forensics_record_t& record = g_crash_forensics_record;
    if (record.magic != CRASH_FORENSICS_MAGIC ||
        record.magic_inv != ~CRASH_FORENSICS_MAGIC ||
        record.schema_version != CRASH_FORENSICS_SCHEMA_VERSION ||
        record.record_size != sizeof(crash_forensics_record_t) ||
        record.committed != CRASH_FORENSICS_COMMITTED ||
        record.committed_inv != ~CRASH_FORENSICS_COMMITTED) {
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
    const uint32_t basic_offset_words = extended_frame ? 18U : 0U;
    record.basic_frame_address = entry->frame_sp + basic_offset_words * 4U;
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

    const size_t required_frame_bytes =
        static_cast<size_t>(basic_offset_words + 8U) * sizeof(uint32_t);
    const bool frame_readable = (entry->frame_sp & 3U) == 0U &&
        stack_span_readable(entry->frame_sp, required_frame_bytes);
    if (frame_readable) {
        record.flags |= CRASH_FORENSICS_FLAG_FRAME_ADDRESS_READABLE;
    }

    if (frame_readable && !stacking_fault) {
        const volatile uint32_t* const raw_frame =
            reinterpret_cast<volatile const uint32_t*>(entry->frame_sp);
        const volatile uint32_t* const basic = raw_frame + basic_offset_words;

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

        if (record.interrupted_exception_number != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_INTERRUPTED_HANDLER;
        }
        if (record.stacked_xpsr & (1UL << 9)) {
            record.flags |= CRASH_FORENSICS_FLAG_STACK_ALIGNMENT_WORD;
        }

        if (extended_frame && stack_span_readable(
                entry->frame_sp,
                CRASH_FORENSICS_FP_FRAME_WORDS * sizeof(uint32_t))) {
            record.fp_frame_word_count = CRASH_FORENSICS_FP_FRAME_WORDS;
            for (size_t i = 0; i < CRASH_FORENSICS_FP_FRAME_WORDS; ++i) {
                record.fp_frame_words[i] = raw_frame[i];
            }
            record.flags |= CRASH_FORENSICS_FLAG_FP_FRAME_CAPTURED;
        }
    }

    record.active_stack_base = entry->frame_sp;
    record.active_stack_word_count = capture_words(
        record.active_stack_base,
        record.active_stack_words,
        CRASH_FORENSICS_ACTIVE_STACK_WORDS,
        true);
    if (record.active_stack_word_count != 0U) {
        record.flags |= CRASH_FORENSICS_FLAG_ACTIVE_STACK_CAPTURED;
    }

    record.other_stack_base =
        (record.frame_source == CRASH_FORENSICS_FRAME_PSP)
            ? entry->msp
            : entry->psp;
    if (record.other_stack_base != 0U &&
        record.other_stack_base != record.active_stack_base) {
        record.other_stack_word_count = capture_words(
            record.other_stack_base,
            record.other_stack_words,
            CRASH_FORENSICS_OTHER_STACK_WORDS,
            true);
        if (record.other_stack_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_OTHER_STACK_CAPTURED;
        }
    }

    if (record.flags & CRASH_FORENSICS_FLAG_BASIC_FRAME_VALID) {
        record.pc_window_word_count = capture_centered_window(
            record.stacked_pc,
            &record.pc_window_base,
            record.pc_window_words,
            CRASH_FORENSICS_PC_WINDOW_WORDS);
        if (record.pc_window_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_PC_WINDOW_CAPTURED;
        }

        record.lr_window_word_count = capture_centered_window(
            record.stacked_lr & ~1UL,
            &record.lr_window_base,
            record.lr_window_words,
            CRASH_FORENSICS_LR_WINDOW_WORDS);
        if (record.lr_window_word_count != 0U) {
            record.flags |= CRASH_FORENSICS_FLAG_LR_WINDOW_CAPTURED;
        }
    }

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

    out->installed = crash_forensics_installed();
    out->present = false;
    out->header_valid = false;
    out->crc_valid = false;
    out->stored_crc = g_crash_forensics_record.crc32;
    out->computed_crc = 0U;

    const crash_forensics_record_t& record = g_crash_forensics_record;
    out->present = record.magic == CRASH_FORENSICS_MAGIC &&
                   record.magic_inv == ~CRASH_FORENSICS_MAGIC;
    if (!out->present) return;

    out->header_valid =
        record.schema_version == CRASH_FORENSICS_SCHEMA_VERSION &&
        record.record_size == sizeof(crash_forensics_record_t) &&
        record.committed == CRASH_FORENSICS_COMMITTED &&
        record.committed_inv == ~CRASH_FORENSICS_COMMITTED;
    if (!out->header_valid) return;

    out->computed_crc = record_crc32(record);
    out->crc_valid = out->computed_crc == record.crc32;
}

FLASHMEM const crash_forensics_record_t* crash_forensics_record(void) {
    crash_forensics_status_t status{};
    crash_forensics_get_status(&status);
    return status.crc_valid ? &g_crash_forensics_record : nullptr;
}

FLASHMEM void crash_forensics_clear(void) {
    const uint32_t saved_primask = read_primask();
    disable_interrupts();
    zero_record(g_crash_forensics_record);
    crash_barrier();
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

// Install after Teensyduino has created its RAM vector table but before global
// C++ constructors run.  setup() installs once more as an explicit boot-order
// assertion and to repair any constructor that might have replaced a vector.
extern "C" FLASHMEM void startup_late_hook(void) {
    crash_forensics_install();
}
