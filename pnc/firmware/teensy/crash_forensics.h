#pragma once

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// ZPNet retained Cortex-M7 crash forensics
// ============================================================================
//
// The fault-time recorder is allocation-free, logging-free, and Payload-free.
// It snapshots the exception frame and surrounding processor state into RAM2,
// flushes the record from cache, then chains into Teensyduino's proven
// CrashReport handler so USB grace and automatic reboot behavior are preserved.
//
// The retained record is intentionally all-uint32_t.  This keeps the ABI stable,
// naturally aligned, and simple to validate after reboot.
// ============================================================================

static constexpr uint32_t CRASH_FORENSICS_SCHEMA_VERSION = 1U;
static constexpr size_t CRASH_FORENSICS_NVIC_WORDS = 5U;
static constexpr size_t CRASH_FORENSICS_MPU_REGIONS = 16U;
static constexpr size_t CRASH_FORENSICS_FP_FRAME_WORDS = 18U;
static constexpr size_t CRASH_FORENSICS_ACTIVE_STACK_WORDS = 64U;
static constexpr size_t CRASH_FORENSICS_OTHER_STACK_WORDS = 16U;
static constexpr size_t CRASH_FORENSICS_PC_WINDOW_WORDS = 24U;
static constexpr size_t CRASH_FORENSICS_LR_WINDOW_WORDS = 24U;

enum crash_forensics_frame_source_t : uint32_t {
    CRASH_FORENSICS_FRAME_NONE = 0U,
    CRASH_FORENSICS_FRAME_MSP = 1U,
    CRASH_FORENSICS_FRAME_PSP = 2U,
};

enum crash_forensics_flag_t : uint32_t {
    CRASH_FORENSICS_FLAG_FRAME_ADDRESS_READABLE = 1UL << 0,
    CRASH_FORENSICS_FLAG_BASIC_FRAME_VALID = 1UL << 1,
    CRASH_FORENSICS_FLAG_EXTENDED_FP_FRAME = 1UL << 2,
    CRASH_FORENSICS_FLAG_FP_FRAME_CAPTURED = 1UL << 3,
    CRASH_FORENSICS_FLAG_STACKING_FAULT = 1UL << 4,
    CRASH_FORENSICS_FLAG_STACK_ALIGNMENT_WORD = 1UL << 5,
    CRASH_FORENSICS_FLAG_RETURN_TO_THREAD = 1UL << 6,
    CRASH_FORENSICS_FLAG_INTERRUPTED_HANDLER = 1UL << 7,
    CRASH_FORENSICS_FLAG_ACTIVE_STACK_CAPTURED = 1UL << 8,
    CRASH_FORENSICS_FLAG_OTHER_STACK_CAPTURED = 1UL << 9,
    CRASH_FORENSICS_FLAG_PC_WINDOW_CAPTURED = 1UL << 10,
    CRASH_FORENSICS_FLAG_LR_WINDOW_CAPTURED = 1UL << 11,
    CRASH_FORENSICS_FLAG_MPU_CAPTURED = 1UL << 12,
    CRASH_FORENSICS_FLAG_NVIC_CAPTURED = 1UL << 13,
};

struct crash_forensics_record_t {
    // Retained-record envelope.
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t schema_version;
    uint32_t record_size;
    uint32_t capture_sequence;
    uint32_t flags;

    // Exception-entry identity and stack selection.
    uint32_t exception_number;
    uint32_t interrupted_exception_number;
    uint32_t frame_source;
    uint32_t raw_frame_address;
    uint32_t basic_frame_address;
    uint32_t exc_return;
    uint32_t original_msp;
    uint32_t original_psp;
    uint32_t primask;
    uint32_t basepri;
    uint32_t faultmask;
    uint32_t control;
    uint32_t dwt_cyccnt;
    uint32_t cpu_hz;

    // Hardware-stacked basic frame.
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_xpsr;

    // Software-preserved callee-saved registers at exception entry.
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;

    // Cortex-M fault and control registers.
    uint32_t cpuid;
    uint32_t actlr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t shcsr;
    uint32_t icsr;
    uint32_t vtor;
    uint32_t aircr;
    uint32_t scr;
    uint32_t ccr;
    uint32_t shpr1;
    uint32_t shpr2;
    uint32_t shpr3;
    uint32_t cpacr;
    uint32_t demcr;
    uint32_t dwt_ctrl;
    uint32_t syst_csr;
    uint32_t syst_rvr;
    uint32_t syst_cvr;
    uint32_t syst_calib;

    // Floating-point context-control registers.
    uint32_t fpccr;
    uint32_t fpcar;
    uint32_t fpdscr;

    // Vector/capture implementation identity.
    uint32_t vector_hardfault;
    uint32_t vector_memmanage;
    uint32_t vector_busfault;
    uint32_t vector_usagefault;
    uint32_t zpnet_fault_entry;
    uint32_t teensy_core_fault_handler;

    // NVIC enable, pending, and active snapshots for all Teensy 4.x IRQ words.
    uint32_t nvic_iser[CRASH_FORENSICS_NVIC_WORDS];
    uint32_t nvic_ispr[CRASH_FORENSICS_NVIC_WORDS];
    uint32_t nvic_iabr[CRASH_FORENSICS_NVIC_WORDS];

    // Complete MPU region snapshot, preserving the selected RNR afterward.
    uint32_t mpu_type;
    uint32_t mpu_ctrl;
    uint32_t mpu_rnr;
    uint32_t mpu_region_count;
    uint32_t mpu_rbar[CRASH_FORENSICS_MPU_REGIONS];
    uint32_t mpu_rasr[CRASH_FORENSICS_MPU_REGIONS];

    // Raw low floating-point exception frame: S0-S15, FPSCR, reserved word.
    uint32_t fp_frame_word_count;
    uint32_t fp_frame_words[CRASH_FORENSICS_FP_FRAME_WORDS];

    // Raw memory evidence.  Counts state how many leading entries are valid.
    uint32_t active_stack_base;
    uint32_t active_stack_word_count;
    uint32_t active_stack_words[CRASH_FORENSICS_ACTIVE_STACK_WORDS];

    uint32_t other_stack_base;
    uint32_t other_stack_word_count;
    uint32_t other_stack_words[CRASH_FORENSICS_OTHER_STACK_WORDS];

    uint32_t pc_window_base;
    uint32_t pc_window_word_count;
    uint32_t pc_window_words[CRASH_FORENSICS_PC_WINDOW_WORDS];

    uint32_t lr_window_base;
    uint32_t lr_window_word_count;
    uint32_t lr_window_words[CRASH_FORENSICS_LR_WINDOW_WORDS];

    // Keep the complete object cache-line sized.  The CRC covers all words
    // before crc32; committed and its complement are the final publication gate.
    uint32_t reserved_tail[2];
    uint32_t crc32;
    uint32_t committed;
    uint32_t committed_inv;
};

struct crash_forensics_status_t {
    bool installed;
    bool present;
    bool header_valid;
    bool crc_valid;
    uint32_t stored_crc;
    uint32_t computed_crc;
};

static_assert((sizeof(crash_forensics_record_t) % 32U) == 0U,
              "Crash record must occupy complete Cortex-M7 cache lines");

void crash_forensics_install(void);
bool crash_forensics_installed(void);

void crash_forensics_get_status(crash_forensics_status_t* out);
const crash_forensics_record_t* crash_forensics_record(void);
void crash_forensics_clear(void);

const char* crash_forensics_exception_name(uint32_t exception_number);
const char* crash_forensics_frame_source_name(uint32_t frame_source);
