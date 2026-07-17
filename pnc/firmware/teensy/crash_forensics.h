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

static constexpr uint32_t CRASH_FORENSICS_SCHEMA_VERSION = 2U;
static constexpr uint32_t CRASH_FORENSICS_CORE_SCHEMA_VERSION = 1U;
static constexpr size_t CRASH_FORENSICS_NVIC_WORDS = 5U;
static constexpr size_t CRASH_FORENSICS_MPU_REGIONS = 16U;
static constexpr size_t CRASH_FORENSICS_FP_FRAME_WORDS = 18U;
static constexpr size_t CRASH_FORENSICS_CORE_STACK_WORDS = 32U;
static constexpr size_t CRASH_FORENSICS_ACTIVE_STACK_WORDS = 64U;
static constexpr size_t CRASH_FORENSICS_OTHER_STACK_WORDS = 16U;
static constexpr size_t CRASH_FORENSICS_PC_WINDOW_WORDS = 24U;
static constexpr size_t CRASH_FORENSICS_LR_WINDOW_WORDS = 24U;

enum crash_forensics_frame_source_t : uint32_t {
    CRASH_FORENSICS_FRAME_NONE = 0U,
    CRASH_FORENSICS_FRAME_MSP = 1U,
    CRASH_FORENSICS_FRAME_PSP = 2U,
};

enum crash_forensics_capture_stage_t : uint32_t {
    CRASH_FORENSICS_STAGE_NONE = 0U,
    CRASH_FORENSICS_STAGE_CORE_BEGIN = 1U,
    CRASH_FORENSICS_STAGE_CORE_COMMITTED = 2U,
    CRASH_FORENSICS_STAGE_EXTENDED_BEGIN = 3U,
    CRASH_FORENSICS_STAGE_EXTENDED_STACKS = 4U,
    CRASH_FORENSICS_STAGE_EXTENDED_WINDOWS = 5U,
    CRASH_FORENSICS_STAGE_EXTENDED_NVIC = 6U,
    CRASH_FORENSICS_STAGE_EXTENDED_MPU = 7U,
    CRASH_FORENSICS_STAGE_EXTENDED_COMMITTED = 8U,
};

enum crash_forensics_capture_skip_reason_t : uint32_t {
    CRASH_FORENSICS_SKIP_NONE = 0U,
    CRASH_FORENSICS_SKIP_INVALID_ARGUMENT = 1U,
    CRASH_FORENSICS_SKIP_ZERO_ADDRESS = 2U,
    CRASH_FORENSICS_SKIP_UNALIGNED_ADDRESS = 3U,
    CRASH_FORENSICS_SKIP_ADDRESS_OVERFLOW = 4U,
    CRASH_FORENSICS_SKIP_OUTSIDE_STACK_MEMORY = 5U,
    CRASH_FORENSICS_SKIP_OUTSIDE_EXECUTABLE_MEMORY = 6U,
    CRASH_FORENSICS_SKIP_MPU_DENIED = 7U,
    CRASH_FORENSICS_SKIP_STACKING_FAULT = 8U,
    CRASH_FORENSICS_SKIP_FRAME_XPSR_IMPLAUSIBLE = 9U,
    CRASH_FORENSICS_SKIP_PC_IMPLAUSIBLE = 10U,
    CRASH_FORENSICS_SKIP_LR_IMPLAUSIBLE = 11U,
    CRASH_FORENSICS_SKIP_DUPLICATE_STACK = 12U,
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
    CRASH_FORENSICS_FLAG_FRAME_CONTENT_PLAUSIBLE = 1UL << 14,
    CRASH_FORENSICS_FLAG_CAPTURE_SKIPPED = 1UL << 15,
};

struct crash_forensics_core_record_t {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t schema_version;
    uint32_t record_size;
    uint32_t capture_sequence;
    uint32_t flags;

    uint32_t exception_number;
    uint32_t interrupted_exception_number;
    uint32_t frame_source;
    uint32_t raw_frame_address;
    uint32_t basic_frame_address;
    uint32_t interrupted_sp;
    uint32_t exc_return;
    uint32_t original_msp;
    uint32_t original_psp;
    uint32_t primask;
    uint32_t basepri;
    uint32_t faultmask;
    uint32_t control;
    uint32_t dwt_cyccnt;
    uint32_t cpu_hz;

    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_xpsr;

    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;

    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t shcsr;
    uint32_t icsr;
    uint32_t fpccr;
    uint32_t fpcar;

    uint32_t stack_base;
    uint32_t stack_word_count;
    uint32_t stack_skip_reason;
    uint32_t stack_words[CRASH_FORENSICS_CORE_STACK_WORDS];

    uint32_t crc32;
    uint32_t committed;
    uint32_t committed_inv;

    // Progress is deliberately outside the core CRC.  The core record remains
    // valid even if a later stage update or extended capture faults.
    uint32_t capture_stage;
    uint32_t capture_stage_inv;
    uint32_t reserved_tail[1];
};

static_assert((sizeof(crash_forensics_core_record_t) % 32U) == 0U,
              "Core crash record must occupy complete Cortex-M7 cache lines");

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

    // A skipped capture is evidence, not an error path.  Each reason is scalar
    // and survives even when a suspect window was deliberately not touched.
    uint32_t active_stack_skip_reason;
    uint32_t other_stack_skip_reason;
    uint32_t pc_window_skip_reason;
    uint32_t lr_window_skip_reason;

    // Keep the complete object cache-line sized.  The CRC covers all words
    // before crc32; committed and its complement are the final publication gate.
    uint32_t reserved_tail[6];
    uint32_t crc32;
    uint32_t committed;
    uint32_t committed_inv;
};

struct crash_forensics_status_t {
    bool installed;
    bool present;

    bool core_present;
    bool core_header_valid;
    bool core_crc_valid;
    uint32_t core_stored_crc;
    uint32_t core_computed_crc;

    bool extended_present;
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
const crash_forensics_core_record_t* crash_forensics_core_record(void);
const crash_forensics_record_t* crash_forensics_record(void);
void crash_forensics_clear(void);

const char* crash_forensics_exception_name(uint32_t exception_number);
const char* crash_forensics_frame_source_name(uint32_t frame_source);
const char* crash_forensics_capture_stage_name(uint32_t stage);
const char* crash_forensics_capture_skip_reason_name(uint32_t reason);

// ============================================================================
// Retained stack watchpoint (DWT comparator + DebugMonitor)
// ============================================================================
//
// Diagnostic trap for the boot-time MEMMANAGE investigation.  A DWT comparator
// watches CPU writes to a small fixed window of DTCM stack that held the
// corrupted saved return addresses.  Each write raises the DebugMonitor
// exception; the handler records the writer's stacked PC/LR and the current
// window contents into a DMAMEM ring that survives the subsequent fault and
// reboot, in the same retained-bank idiom as the other flight recorders.
//
// The handler is allocation-free, logging-free, and Payload-free.  A write is
// classified anomalous when the writer's own stack pointer sits entirely above
// the watched window: lawful code never stores below its live frame, so such a
// hit is the rogue store this trap exists to catch.
//
// The DWT observes CPU stores only.  A DMA-mastered write would not trigger.

static constexpr uint32_t CRASH_STACK_WATCH_ENTRIES = 16U;

struct crash_stack_watch_entry_t {
    uint32_t sequence;
    uint32_t sequence_inv;
    uint32_t dwt;
    uint32_t pc;
    uint32_t lr;
    uint32_t xpsr;
    uint32_t writer_sp;
    uint32_t exc_return;
    uint32_t watched[4];
};

static_assert(sizeof(crash_stack_watch_entry_t) == 48U,
              "Stack watch entry layout drifted");

struct crash_stack_watch_bank_snapshot_t {
    bool valid;
    uint32_t count;
    uint32_t newest_sequence;
    uint32_t hits_total;
    uint32_t anomalous_total;
    crash_stack_watch_entry_t first_anomalous;
    crash_stack_watch_entry_t entries[CRASH_STACK_WATCH_ENTRIES];
};

struct crash_stack_watch_snapshot_t {
    bool enabled;
    bool armed;
    uint32_t arm_skip_reason;
    uint32_t arm_attempts;
    uint32_t last_dhcsr;
    uint32_t watch_address;
    uint32_t watch_bytes;
    uint32_t effective_comp_address;
    uint32_t effective_mask;
    crash_stack_watch_bank_snapshot_t live;
    crash_stack_watch_bank_snapshot_t retained;
};

enum crash_stack_watch_skip_reason_t : uint32_t {
    CRASH_STACK_WATCH_SKIP_NONE = 0U,
    CRASH_STACK_WATCH_SKIP_DISABLED = 1U,
    CRASH_STACK_WATCH_SKIP_HALTING_DEBUG_ACTIVE = 2U,
};

void crash_stack_watch_snapshot(crash_stack_watch_snapshot_t* out);
void crash_stack_watch_clear_retained(void);
const char* crash_stack_watch_skip_reason_name(uint32_t reason);

// ============================================================================
// Stack altitude tripwire (SP-floor overlap detector)
// ============================================================================
//
// Companion diagnostic for the boot-crash investigation.  The memory-watchdog
// audit publishes its entry SP as a floor while it runs.  During the audit,
// the true foreground SP is always at or below that floor, so any exception
// frame whose top lies ABOVE the floor was stacked against a stale/wrong SP —
// the alien-frame event under investigation.  ISR entries call the inline
// check; a violation latches retained scalar evidence.  Cost when the floor
// is inactive: one volatile load and a compare.
//
// The check is advisory instrumentation: it attributes, it never alters
// control flow.  A negative result across armed crashes is also evidence —
// it argues the corruption is a software write, not mis-stacked entry.

static constexpr uint32_t CRASH_STACK_TRIPWIRE_ENTRIES = 8U;

static constexpr uint32_t CRASH_TRIPWIRE_SITE_NONE      = 0U;
static constexpr uint32_t CRASH_TRIPWIRE_SITE_HANDOFF   = 1U;
static constexpr uint32_t CRASH_TRIPWIRE_SITE_QTIMER1   = 2U;
static constexpr uint32_t CRASH_TRIPWIRE_SITE_QTIMER2   = 3U;
static constexpr uint32_t CRASH_TRIPWIRE_SITE_QTIMER3   = 4U;
static constexpr uint32_t CRASH_TRIPWIRE_SITE_PPS_GPIO  = 5U;

struct crash_stack_tripwire_entry_t {
    uint32_t sequence;
    uint32_t sequence_inv;
    uint32_t site;
    uint32_t msp;
    uint32_t exc_return;
    uint32_t floor;
    uint32_t frame_top_estimate;
    uint32_t dwt;
    uint32_t ipsr;
    uint32_t reserved[3];
};

static_assert(sizeof(crash_stack_tripwire_entry_t) == 48U,
              "Stack tripwire entry layout drifted");

struct crash_stack_tripwire_bank_snapshot_t {
    bool valid;
    uint32_t count;
    uint32_t newest_sequence;
    uint32_t violation_total;
    crash_stack_tripwire_entry_t first;
    crash_stack_tripwire_entry_t entries[CRASH_STACK_TRIPWIRE_ENTRIES];
};

struct crash_stack_tripwire_snapshot_t {
    bool floor_active_now;
    uint32_t floor_now;
    uint32_t floor_publish_count;
    crash_stack_tripwire_bank_snapshot_t live;
    crash_stack_tripwire_bank_snapshot_t retained;
};

void crash_stack_tripwire_snapshot(crash_stack_tripwire_snapshot_t* out);
void crash_stack_tripwire_clear_retained(void);
const char* crash_stack_tripwire_site_name(uint32_t site);

// Out-of-line violation latcher; called only on overlap.
void crash_stack_tripwire_latch(uint32_t site,
                                uint32_t msp,
                                uint32_t exc_return,
                                uint32_t floor,
                                uint32_t frame_top_estimate);

// Published SP floor.  Zero means inactive.  Written from foreground only.
extern volatile uint32_t g_crash_stack_tripwire_floor;
extern volatile uint32_t g_crash_stack_tripwire_floor_publish_count;

// Foreground publisher.  Call at the guarded region's entry and exit.
static inline void crash_stack_tripwire_floor_enter(void) {
    uint32_t sp;
    __asm__ volatile("mov %0, sp" : "=r"(sp));
    g_crash_stack_tripwire_floor = sp;
    g_crash_stack_tripwire_floor_publish_count =
        g_crash_stack_tripwire_floor_publish_count + 1U;
    __asm__ volatile("dmb" ::: "memory");
}

static inline void crash_stack_tripwire_floor_exit(void) {
    __asm__ volatile("dmb" ::: "memory");
    g_crash_stack_tripwire_floor = 0U;
}

// ISR-entry check.  Place after the sacred first capture and before the first
// out-of-line call so LR still holds EXC_RETURN; sites that must run later
// (after calls) are handled by the conservative frame-size fallback below.
static inline void crash_stack_tripwire_isr_check(uint32_t site) {
    const uint32_t floor = g_crash_stack_tripwire_floor;
    if (floor == 0U) return;

    uint32_t msp;
    uint32_t lr;
    __asm__ volatile("mrs %0, msp" : "=r"(msp));
    __asm__ volatile("mov %0, lr" : "=r"(lr));

    // Frame size from EXC_RETURN when LR still plausibly holds it (0xFFFFFFxx).
    // Otherwise assume the SMALL basic frame: under-estimating the frame top
    // can only suppress a report, never fabricate one.
    uint32_t frame_bytes = 0x20U;
    if ((lr & 0xFFFFFF00UL) == 0xFFFFFF00UL && (lr & 0x10U) == 0U) {
        frame_bytes = 0x68U;
    }

    // msp here is at or below the hardware frame base (the compiler prologue
    // may already have pushed), so this estimate is a LOWER bound on the true
    // frame top.  Lawful stacking keeps the true top at or below the live SP,
    // which is at or below the floor; an estimate above the floor is proof.
    const uint32_t frame_top_estimate = msp + frame_bytes;
    if (frame_top_estimate > floor) {
        crash_stack_tripwire_latch(site, msp, lr, floor, frame_top_estimate);
    }
}

// Retryable arming.  Software cannot clear DHCSR.C_DEBUGEN (only the debug
// port or a power-on reset can), so when the bootloader chip leaves halting
// debug latched after a flash, the install-time arm attempt is declined.  This
// service retries cheaply from foreground paths; it arms the instant the
// debug session is released and is a no-op once armed.
void crash_stack_watch_service(void);

// Retryable arming.  Software cannot clear DHCSR.C_DEBUGEN (only the debug
// port or a power-on reset can), so when the bootloader chip leaves halting
// debug latched after a flash, the install-time arm attempt is declined.  This
// service retries cheaply from foreground paths; it arms the instant the
// debug session is released and is a no-op once armed.
void crash_stack_watch_service(void);
