#if 0
// Retired: Frame Sentinel is excluded from the firmware baseline.

#include "frame_sentinel.h"

#include "events.h"
#include "payload.h"

#include <string.h>

extern "C" unsigned long _estack;

#define FRAME_SENTINEL_FPCCR (*(volatile uint32_t*)0xE000EF34UL)
#define FRAME_SENTINEL_FPCAR (*(volatile uint32_t*)0xE000EF38UL)

static constexpr uint32_t FRAME_SENTINEL_MAGIC =
    0x46535636UL;  // 'FSV6'
static constexpr uint32_t FRAME_SENTINEL_BEACON_MAGIC =
    0x46534236UL;  // 'FSB6'
static constexpr uint32_t FRAME_SENTINEL_SCHEMA_VERSION = 6U;
static constexpr uint32_t FRAME_SENTINEL_BASIC_WORDS = 8U;
static constexpr uint32_t FRAME_SENTINEL_FP_WORDS = 18U;
static constexpr uint32_t FRAME_SENTINEL_CONTEXT_WORDS = 32U;
static constexpr uint32_t FRAME_SENTINEL_SCAN_BYTES = 512U;
static constexpr uint32_t FRAME_SENTINEL_BEACON_DEPTH = 6U;
static constexpr uint32_t FRAME_SENTINEL_MAX_EXCEPTION = 255U;
static constexpr uint32_t FRAME_SENTINEL_BASIC_BYTES =
    FRAME_SENTINEL_BASIC_WORDS * sizeof(uint32_t);
static constexpr uint32_t FRAME_SENTINEL_FP_BYTES =
    FRAME_SENTINEL_FP_WORDS * sizeof(uint32_t);

enum : uint32_t {
  FRAME_SENTINEL_SOURCE_NONE  = 0U,
  FRAME_SENTINEL_SOURCE_FPCAR = 1U,
  FRAME_SENTINEL_SOURCE_PSP   = 2U,
  FRAME_SENTINEL_SOURCE_SCAN  = 3U,
};

struct frame_sentinel_geometry_t {
  uint32_t source;
  uint32_t raw_frame_address;
  uint32_t fp_frame_address;
  uint32_t basic_frame_address;
  uint32_t context_address;
  uint32_t extended;
  uint32_t alignment_word;
  uint32_t core_content_plausible;
  uint32_t fp_evidence_trusted;
  uint32_t valid;
};

struct frame_sentinel_scope_t {
  volatile uint32_t active;
  char name[24];
  uint32_t pass;
  uint32_t ipsr;
  uint32_t exc_return_entry;
  uint32_t caller_sp_entry;
  uint32_t dwt_entry;
  uint32_t fpccr_entry;
  uint32_t fpcar_entry;

  frame_sentinel_geometry_t geometry_entry;
  uint32_t frame_entry[FRAME_SENTINEL_BASIC_WORDS];
  uint32_t frame_entry_checksum;

  uint32_t fp_entry[FRAME_SENTINEL_FP_WORDS];
  uint32_t fp_entry_checksum;

  uint32_t context_count;
  uint32_t context_entry[FRAME_SENTINEL_CONTEXT_WORDS];
  uint32_t context_entry_checksum;

  uint32_t enters;
  uint32_t exits;
  uint32_t frame_located;
  uint32_t frame_missing;
  uint32_t context_located;
  uint32_t context_missing;
  uint32_t extended_frames;
  uint32_t fpcar_geometry_used;
  uint32_t lspact_consumed;
  uint32_t frame_type_changed;
  uint32_t geometry_changed;
  uint32_t unbalanced_enter;
  uint32_t unbalanced_exit;
};

struct frame_sentinel_beacon_entry_t {
  uint32_t slot;
  uint32_t pass;
  uint32_t ipsr;
  uint32_t dwt_enter;
  char name[24];
};

struct alignas(32) frame_sentinel_beacon_t {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t schema_version;
  uint32_t depth;
  uint32_t overflow;
  frame_sentinel_beacon_entry_t active[FRAME_SENTINEL_BEACON_DEPTH];
  frame_sentinel_beacon_entry_t last_completed;
  uint32_t last_completed_dwt_exit;
};

struct alignas(32) frame_sentinel_violation_t {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t schema_version;
  uint32_t kind;
  uint32_t slot;
  uint32_t pass;
  uint32_t ipsr;
  uint32_t exc_return_entry;
  uint32_t exc_return_exit;
  char name[24];

  uint32_t caller_sp_entry;
  uint32_t caller_sp_exit;
  uint32_t dwt_entry;
  uint32_t dwt_exit;
  uint32_t fpccr_entry;
  uint32_t fpccr_exit;
  uint32_t fpcar_entry;
  uint32_t fpcar_exit;
  uint32_t lspact_consumed;
  uint32_t exc_return_entry_valid;
  uint32_t exc_return_exit_valid;
  uint32_t frame_type_entry_extended;
  uint32_t frame_type_exit_extended;

  frame_sentinel_geometry_t geometry_entry;
  frame_sentinel_geometry_t geometry_exit;

  uint32_t frame_diff_mask;
  uint32_t frame_entry_checksum;
  uint32_t frame_exit_checksum;
  uint32_t frame_entry[FRAME_SENTINEL_BASIC_WORDS];
  uint32_t frame_exit[FRAME_SENTINEL_BASIC_WORDS];

  uint32_t fp_diff_mask;
  uint32_t fp_entry_checksum;
  uint32_t fp_exit_checksum;
  uint32_t fp_first_diff_index;
  uint32_t fp_first_diff_address;
  uint32_t fp_first_diff_entry;
  uint32_t fp_first_diff_exit;
  uint32_t fp_entry[FRAME_SENTINEL_FP_WORDS];
  uint32_t fp_exit[FRAME_SENTINEL_FP_WORDS];

  uint32_t context_count;
  uint32_t context_diff_mask;
  uint32_t context_entry_checksum;
  uint32_t context_exit_checksum;
  uint32_t context_first_diff_index;
  uint32_t context_first_diff_address;
  uint32_t context_first_diff_entry;
  uint32_t context_first_diff_exit;
  uint32_t context_entry[FRAME_SENTINEL_CONTEXT_WORDS];
  uint32_t context_exit[FRAME_SENTINEL_CONTEXT_WORDS];

  uint32_t victim_depth;
  uint32_t victim_slot;
  uint32_t victim_pass;
  uint32_t victim_ipsr;
  char victim_name[24];
};

static frame_sentinel_scope_t
    g_frame_sentinel_scopes[FRAME_SENTINEL_SLOT_COUNT];

// Live/retained beacon pair.  The live bank survives a fault; init copies it to
// retained before beginning the next boot's observations.
static frame_sentinel_beacon_t g_frame_sentinel_beacon DMAMEM;
static frame_sentinel_beacon_t g_frame_sentinel_beacon_retained DMAMEM;
static frame_sentinel_violation_t g_frame_sentinel_violation_retained DMAMEM;

static frame_sentinel_violation_t g_frame_sentinel_violation_last;
static bool g_frame_sentinel_violation_last_present = false;
static bool g_frame_sentinel_initialized = false;
static bool g_frame_sentinel_event_pending = false;
static bool g_frame_sentinel_event_emitted = false;
static uint32_t g_frame_sentinel_enters = 0U;
static uint32_t g_frame_sentinel_exits = 0U;
static uint32_t g_frame_sentinel_bad_slot = 0U;
static uint32_t g_frame_sentinel_violations = 0U;
static uint32_t g_frame_sentinel_suppressed = 0U;
static uint32_t g_frame_sentinel_active_depth = 0U;

static inline void frame_sentinel_dmb(void) {
  __asm__ volatile("dmb" ::: "memory");
}

static inline uint32_t frame_sentinel_read_ipsr(void) {
  uint32_t value = 0U;
#if defined(__arm__)
  __asm__ volatile("mrs %0, ipsr" : "=r"(value));
#endif
  return value & 0x1FFU;
}

static inline uint32_t frame_sentinel_read_msp(void) {
  uint32_t value = 0U;
#if defined(__arm__)
  __asm__ volatile("mrs %0, msp" : "=r"(value));
#endif
  return value;
}

static inline uint32_t frame_sentinel_read_psp(void) {
  uint32_t value = 0U;
#if defined(__arm__)
  __asm__ volatile("mrs %0, psp" : "=r"(value));
#endif
  return value;
}

static inline uint32_t frame_sentinel_irq_lock(void) {
  uint32_t primask = 0U;
#if defined(__arm__)
  __asm__ volatile("mrs %0, primask" : "=r"(primask));
  __asm__ volatile("cpsid i" ::: "memory");
#endif
  return primask;
}

static inline void frame_sentinel_irq_unlock(uint32_t primask) {
#if defined(__arm__)
  if ((primask & 1U) == 0U) {
    __asm__ volatile("cpsie i" ::: "memory");
  }
#else
  (void)primask;
#endif
}

static void frame_sentinel_copy_name(char* dst,
                                     size_t dst_size,
                                     const char* src) {
  if (!dst || dst_size == 0U) return;
  size_t i = 0U;
  if (src) {
    while (i + 1U < dst_size && src[i] != '\0') {
      dst[i] = src[i];
      ++i;
    }
  }
  dst[i] = '\0';
}

static bool frame_sentinel_range_readable(uint32_t address, uint32_t bytes) {
  if ((address & 3U) != 0U || bytes == 0U) return false;
  const uint32_t end = (uint32_t)(uintptr_t)&_estack;
  if (address < 0x20000000UL || address >= end) return false;
  return bytes <= end - address;
}

static bool frame_sentinel_pc_plausible(uint32_t pc) {
  if ((pc & 1U) != 0U) return false;
  if (pc >= 0x00000400UL && pc < 0x00080000UL) return true;
  return pc >= 0x60000000UL && pc < 0x61000000UL;
}

static bool frame_sentinel_exc_return_plausible(uint32_t value) {
  return (value & 0xFFFFFF00UL) == 0xFFFFFF00UL &&
         (value & 1U) != 0U;
}

static bool frame_sentinel_lr_plausible(uint32_t lr) {
  if (frame_sentinel_exc_return_plausible(lr)) return true;
  if ((lr & 1U) == 0U) return false;
  return frame_sentinel_pc_plausible(lr & ~1U);
}

static bool frame_sentinel_xpsr_plausible(uint32_t xpsr,
                                          uint32_t exc_return) {
  if ((xpsr & (1UL << 24)) == 0U) return false;
  const uint32_t exception = xpsr & 0x1FFU;
  const bool return_to_thread = (exc_return & (1UL << 3)) != 0U;
  if (return_to_thread) return exception == 0U;
  return exception != 0U && exception <= FRAME_SENTINEL_MAX_EXCEPTION;
}

static uint32_t frame_sentinel_checksum(const uint32_t* words,
                                        uint32_t count) {
  uint32_t hash = 0x811C9DC5UL;
  for (uint32_t i = 0U; i < count; ++i) {
    hash ^= words[i];
    hash *= 0x01000193UL;
  }
  return hash;
}

static bool frame_sentinel_beacon_valid(
    const frame_sentinel_beacon_t& beacon) {
  return beacon.magic == FRAME_SENTINEL_BEACON_MAGIC &&
         (beacon.magic ^ beacon.magic_inv) == 0xFFFFFFFFUL &&
         beacon.schema_version == FRAME_SENTINEL_SCHEMA_VERSION;
}

static bool frame_sentinel_violation_valid(
    const frame_sentinel_violation_t& violation) {
  return violation.magic == FRAME_SENTINEL_MAGIC &&
         (violation.magic ^ violation.magic_inv) == 0xFFFFFFFFUL &&
         violation.schema_version == FRAME_SENTINEL_SCHEMA_VERSION;
}

void frame_sentinel_init(void) {
  if (g_frame_sentinel_initialized) return;
  g_frame_sentinel_initialized = true;

  if (frame_sentinel_beacon_valid(g_frame_sentinel_beacon)) {
    g_frame_sentinel_beacon_retained = g_frame_sentinel_beacon;
  } else {
    memset((void*)&g_frame_sentinel_beacon_retained, 0,
           sizeof(g_frame_sentinel_beacon_retained));
  }
  arm_dcache_flush((void*)&g_frame_sentinel_beacon_retained,
                   sizeof(g_frame_sentinel_beacon_retained));

  memset((void*)&g_frame_sentinel_beacon, 0,
         sizeof(g_frame_sentinel_beacon));
  g_frame_sentinel_beacon.schema_version = FRAME_SENTINEL_SCHEMA_VERSION;
  g_frame_sentinel_beacon.magic_inv = ~FRAME_SENTINEL_BEACON_MAGIC;
  g_frame_sentinel_beacon.magic = FRAME_SENTINEL_BEACON_MAGIC;
  arm_dcache_flush((void*)&g_frame_sentinel_beacon,
                   sizeof(g_frame_sentinel_beacon));
}

static void frame_sentinel_beacon_push(uint32_t slot,
                                       const char* name,
                                       uint32_t pass,
                                       uint32_t ipsr,
                                       uint32_t dwt) {
  frame_sentinel_init();
  const uint32_t primask = frame_sentinel_irq_lock();
  const uint32_t depth = g_frame_sentinel_beacon.depth;
  if (depth < FRAME_SENTINEL_BEACON_DEPTH) {
    frame_sentinel_beacon_entry_t& entry =
        g_frame_sentinel_beacon.active[depth];
    entry.slot = slot;
    entry.pass = pass;
    entry.ipsr = ipsr;
    entry.dwt_enter = dwt;
    frame_sentinel_copy_name(entry.name, sizeof(entry.name), name);
  } else {
    g_frame_sentinel_beacon.overflow++;
  }
  g_frame_sentinel_beacon.depth = depth + 1U;
  g_frame_sentinel_active_depth = depth + 1U;
  frame_sentinel_irq_unlock(primask);
  arm_dcache_flush((void*)&g_frame_sentinel_beacon,
                   sizeof(g_frame_sentinel_beacon));
}

static void frame_sentinel_beacon_pop(uint32_t dwt_exit) {
  const uint32_t primask = frame_sentinel_irq_lock();
  uint32_t depth = g_frame_sentinel_beacon.depth;
  if (depth != 0U) {
    depth--;
    if (depth < FRAME_SENTINEL_BEACON_DEPTH) {
      g_frame_sentinel_beacon.last_completed =
          g_frame_sentinel_beacon.active[depth];
      g_frame_sentinel_beacon.last_completed_dwt_exit = dwt_exit;
      memset((void*)&g_frame_sentinel_beacon.active[depth], 0,
             sizeof(g_frame_sentinel_beacon.active[depth]));
    }
    g_frame_sentinel_beacon.depth = depth;
  }
  g_frame_sentinel_active_depth = depth;
  frame_sentinel_irq_unlock(primask);
  arm_dcache_flush((void*)&g_frame_sentinel_beacon,
                   sizeof(g_frame_sentinel_beacon));
}

bool frame_sentinel_scope_active(void) {
  return g_frame_sentinel_active_depth != 0U;
}

static bool frame_sentinel_copy_words(uint32_t address,
                                      uint32_t* out,
                                      uint32_t count) {
  if (!out ||
      !frame_sentinel_range_readable(address, count * sizeof(uint32_t))) {
    return false;
  }
  const volatile uint32_t* words =
      (const volatile uint32_t*)(uintptr_t)address;
  for (uint32_t i = 0U; i < count; ++i) out[i] = words[i];
  return true;
}

static bool frame_sentinel_basic_frame_plausible(uint32_t address,
                                                 uint32_t exc_return) {
  if (!frame_sentinel_range_readable(address, FRAME_SENTINEL_BASIC_BYTES)) {
    return false;
  }
  const volatile uint32_t* words =
      (const volatile uint32_t*)(uintptr_t)address;
  return frame_sentinel_pc_plausible(words[6]) &&
         frame_sentinel_lr_plausible(words[5]) &&
         frame_sentinel_xpsr_plausible(words[7], exc_return);
}

static uint32_t frame_sentinel_exc_return_extended(uint32_t exc_return) {
  return frame_sentinel_exc_return_plausible(exc_return) &&
         (exc_return & (1UL << 4)) == 0U;
}

static frame_sentinel_geometry_t frame_sentinel_geometry_from_basic(
    uint32_t source,
    uint32_t basic_frame_address,
    uint32_t extended,
    uint32_t exc_return,
    bool require_plausible_core) {
  frame_sentinel_geometry_t geometry{};
  geometry.source = source;
  geometry.basic_frame_address = basic_frame_address;
  geometry.extended = extended ? 1U : 0U;

  if (!frame_sentinel_range_readable(
          basic_frame_address, FRAME_SENTINEL_BASIC_BYTES)) {
    return geometry;
  }

  geometry.core_content_plausible =
      frame_sentinel_basic_frame_plausible(
          basic_frame_address, exc_return) ? 1U : 0U;
  if (require_plausible_core && !geometry.core_content_plausible) {
    return geometry;
  }

  const volatile uint32_t* basic =
      (const volatile uint32_t*)(uintptr_t)basic_frame_address;
  geometry.alignment_word =
      (basic[7] & (1UL << 9)) != 0U ? 1U : 0U;

  if (geometry.extended) {
    if (basic_frame_address < FRAME_SENTINEL_FP_BYTES) return geometry;
    geometry.raw_frame_address =
        basic_frame_address - FRAME_SENTINEL_FP_BYTES;
    geometry.fp_frame_address = geometry.raw_frame_address;
    if (!frame_sentinel_range_readable(
            geometry.fp_frame_address, FRAME_SENTINEL_FP_BYTES)) {
      return geometry;
    }
    // FPCAR and PSP are architectural anchors.  SCAN is heuristic and may
    // observe stack words that merely resemble a frame, so its FP prefix is
    // descriptive only and cannot independently convict FP mutation.
    geometry.fp_evidence_trusted =
        source == FRAME_SENTINEL_SOURCE_SCAN ? 0U : 1U;
  } else {
    geometry.raw_frame_address = basic_frame_address;
    geometry.fp_frame_address = 0U;
    geometry.fp_evidence_trusted = 0U;
  }

  geometry.context_address =
      basic_frame_address + FRAME_SENTINEL_BASIC_BYTES +
      geometry.alignment_word * sizeof(uint32_t);
  if (!frame_sentinel_range_readable(
          geometry.context_address,
          FRAME_SENTINEL_CONTEXT_WORDS * sizeof(uint32_t))) {
    geometry.context_address = 0U;
  }

  geometry.valid = 1U;
  return geometry;
}

static frame_sentinel_geometry_t frame_sentinel_locate_geometry(
    uint32_t ipsr,
    uint32_t exc_return,
    uint32_t caller_sp,
    uint32_t fpccr,
    uint32_t fpcar) {
  frame_sentinel_geometry_t none{};
  if (ipsr == 0U) return none;

  const uint32_t dtcm_end = (uint32_t)(uintptr_t)&_estack;
  const bool exc_plausible =
      frame_sentinel_exc_return_plausible(exc_return);
  const uint32_t extended =
      frame_sentinel_exc_return_extended(exc_return);

  // FPCAR points 0x20 bytes into the reserved lazy-FP area on the observed
  // Cortex-M7 stack geometry.  Therefore:
  //
  //   raw FP frame = FPCAR - 0x20
  //   core frame   = FPCAR + 0x28
  //
  // LSPACT makes this an architectural anchor.  Preserve the geometry even
  // when the saved core words are already implausible; plausibility is reported
  // separately rather than being allowed to erase deterministic evidence.
  if ((fpccr & 0x1U) != 0U && fpcar >= 0x20000020UL) {
    const uint32_t observed_basic = fpcar + 0x28U;
    frame_sentinel_geometry_t geometry =
        frame_sentinel_geometry_from_basic(
            FRAME_SENTINEL_SOURCE_FPCAR,
            observed_basic,
            true,
            exc_return,
            false);
    if (geometry.valid) return geometry;

  }

  // PSP points at the raw hardware frame.  For an extended frame the eight
  // core words begin after the 18-word FP prefix.
  if (exc_plausible && (exc_return & 0x4U) != 0U) {
    const uint32_t psp = frame_sentinel_read_psp();
    const uint32_t basic =
        extended ? psp + FRAME_SENTINEL_FP_BYTES : psp;
    frame_sentinel_geometry_t geometry =
        frame_sentinel_geometry_from_basic(
            FRAME_SENTINEL_SOURCE_PSP,
            basic,
            extended,
            exc_return,
            true);
    if (geometry.valid) return geometry;
  }

  if (!exc_plausible) return none;

  const uint32_t msp = frame_sentinel_read_msp();
  const uint32_t scan_from =
      (caller_sp >= 0x20000000UL && caller_sp < dtcm_end)
          ? caller_sp
          : msp;
  const uint32_t begin = (scan_from + 7U) & ~7U;
  for (uint32_t raw = begin;
       raw + FRAME_SENTINEL_BASIC_BYTES +
               (extended ? FRAME_SENTINEL_FP_BYTES : 0U) <= dtcm_end &&
       raw < begin + FRAME_SENTINEL_SCAN_BYTES;
       raw += 8U) {
    // Respect EXC_RETURN's frame type while scanning.  For an extended frame,
    // raw points at S0 and the core frame begins 18 words later.  Scanning raw
    // FP words as though they were R0-xPSR recreates the exact category error
    // this diagnostic is intended to detect.
    const uint32_t basic =
        raw + (extended ? FRAME_SENTINEL_FP_BYTES : 0U);
    frame_sentinel_geometry_t geometry =
        frame_sentinel_geometry_from_basic(
            FRAME_SENTINEL_SOURCE_SCAN,
            basic,
            extended,
            exc_return,
            true);
    if (geometry.valid) return geometry;
  }

  return none;
}

void frame_sentinel_enter(uint32_t slot,
                          const char* name,
                          uint32_t exc_return,
                          uint32_t caller_sp) {
  frame_sentinel_init();
  if (slot >= FRAME_SENTINEL_SLOT_COUNT) {
    g_frame_sentinel_bad_slot++;
    return;
  }

  frame_sentinel_scope_t& scope = g_frame_sentinel_scopes[slot];
  if (scope.active != 0U) scope.unbalanced_enter++;
  scope.active = 1U;
  scope.pass++;
  scope.enters++;
  g_frame_sentinel_enters++;

  frame_sentinel_copy_name(scope.name, sizeof(scope.name), name);
  scope.ipsr = frame_sentinel_read_ipsr();
  scope.exc_return_entry = exc_return;
  scope.caller_sp_entry = caller_sp;
  scope.dwt_entry = ARM_DWT_CYCCNT;
  scope.fpccr_entry = FRAME_SENTINEL_FPCCR;
  scope.fpcar_entry = FRAME_SENTINEL_FPCAR;

  frame_sentinel_beacon_push(slot, name, scope.pass, scope.ipsr,
                             scope.dwt_entry);

  scope.geometry_entry = frame_sentinel_locate_geometry(
      scope.ipsr, exc_return, caller_sp, scope.fpccr_entry,
      scope.fpcar_entry);
  scope.context_count = 0U;

  if (!scope.geometry_entry.valid) {
    if (scope.ipsr != 0U) scope.frame_missing++;
    return;
  }

  scope.frame_located++;
  if (scope.geometry_entry.extended) scope.extended_frames++;
  if (scope.geometry_entry.source == FRAME_SENTINEL_SOURCE_FPCAR) {
    scope.fpcar_geometry_used++;
  }

  if (!frame_sentinel_copy_words(
          scope.geometry_entry.basic_frame_address,
          scope.frame_entry,
          FRAME_SENTINEL_BASIC_WORDS)) {
    scope.frame_missing++;
    return;
  }
  scope.frame_entry_checksum =
      frame_sentinel_checksum(scope.frame_entry,
                              FRAME_SENTINEL_BASIC_WORDS);

  memset(scope.fp_entry, 0, sizeof(scope.fp_entry));
  scope.fp_entry_checksum = 0U;
  if (scope.geometry_entry.extended &&
      scope.geometry_entry.fp_frame_address != 0U &&
      frame_sentinel_copy_words(
          scope.geometry_entry.fp_frame_address,
          scope.fp_entry,
          FRAME_SENTINEL_FP_WORDS)) {
    scope.fp_entry_checksum =
        frame_sentinel_checksum(scope.fp_entry,
                                FRAME_SENTINEL_FP_WORDS);
  }

  if (scope.geometry_entry.context_address == 0U) {
    scope.context_missing++;
    return;
  }

  scope.context_located++;
  scope.context_count = FRAME_SENTINEL_CONTEXT_WORDS;
  if (!frame_sentinel_copy_words(
          scope.geometry_entry.context_address,
          scope.context_entry,
          scope.context_count)) {
    scope.context_count = 0U;
    scope.context_missing++;
    return;
  }
  scope.context_entry_checksum =
      frame_sentinel_checksum(scope.context_entry, scope.context_count);
}

static void frame_sentinel_capture_victim(
    frame_sentinel_violation_t& violation) {
  violation.victim_depth = g_frame_sentinel_beacon.depth;
  if (g_frame_sentinel_beacon.depth == 0U) return;

  // The outermost active scope is the preempted victim region.
  const frame_sentinel_beacon_entry_t& victim =
      g_frame_sentinel_beacon.active[0];
  violation.victim_slot = victim.slot;
  violation.victim_pass = victim.pass;
  violation.victim_ipsr = victim.ipsr;
  frame_sentinel_copy_name(violation.victim_name,
                           sizeof(violation.victim_name),
                           victim.name);
}

static void frame_sentinel_fill_violation(
    frame_sentinel_violation_t& violation,
    const frame_sentinel_scope_t& scope,
    uint32_t slot,
    uint32_t kind,
    uint32_t exc_return_exit,
    uint32_t caller_sp_exit,
    uint32_t dwt_exit,
    uint32_t fpccr_exit,
    uint32_t fpcar_exit,
    const frame_sentinel_geometry_t& geometry_exit,
    const uint32_t* frame_exit,
    uint32_t frame_diff_mask,
    uint32_t frame_exit_checksum,
    const uint32_t* fp_exit,
    uint32_t fp_diff_mask,
    uint32_t fp_exit_checksum,
    uint32_t fp_first_diff_index,
    const uint32_t* context_exit,
    uint32_t context_diff_mask,
    uint32_t context_exit_checksum,
    uint32_t context_first_diff_index) {
  memset((void*)&violation, 0, sizeof(violation));
  violation.kind = kind;
  violation.slot = slot;
  violation.pass = scope.pass;
  violation.ipsr = scope.ipsr;
  violation.exc_return_entry = scope.exc_return_entry;
  violation.exc_return_exit = exc_return_exit;
  frame_sentinel_copy_name(violation.name, sizeof(violation.name),
                           scope.name);

  violation.caller_sp_entry = scope.caller_sp_entry;
  violation.caller_sp_exit = caller_sp_exit;
  violation.dwt_entry = scope.dwt_entry;
  violation.dwt_exit = dwt_exit;
  violation.fpccr_entry = scope.fpccr_entry;
  violation.fpccr_exit = fpccr_exit;
  violation.fpcar_entry = scope.fpcar_entry;
  violation.fpcar_exit = fpcar_exit;
  violation.lspact_consumed =
      ((scope.fpccr_entry & 0x1U) != 0U &&
       (fpccr_exit & 0x1U) == 0U) ? 1U : 0U;
  violation.exc_return_entry_valid =
      frame_sentinel_exc_return_plausible(scope.exc_return_entry) ? 1U : 0U;
  violation.exc_return_exit_valid =
      frame_sentinel_exc_return_plausible(exc_return_exit) ? 1U : 0U;
  violation.frame_type_entry_extended =
      violation.exc_return_entry_valid
          ? frame_sentinel_exc_return_extended(scope.exc_return_entry)
          : 0U;
  violation.frame_type_exit_extended =
      violation.exc_return_exit_valid
          ? frame_sentinel_exc_return_extended(exc_return_exit)
          : 0U;

  violation.geometry_entry = scope.geometry_entry;
  violation.geometry_exit = geometry_exit;

  violation.frame_diff_mask = frame_diff_mask;
  violation.frame_entry_checksum = scope.frame_entry_checksum;
  violation.frame_exit_checksum = frame_exit_checksum;
  for (uint32_t i = 0U; i < FRAME_SENTINEL_BASIC_WORDS; ++i) {
    violation.frame_entry[i] = scope.frame_entry[i];
    violation.frame_exit[i] = frame_exit ? frame_exit[i] : 0U;
  }

  violation.fp_diff_mask = fp_diff_mask;
  violation.fp_entry_checksum = scope.fp_entry_checksum;
  violation.fp_exit_checksum = fp_exit_checksum;
  violation.fp_first_diff_index = fp_first_diff_index;
  if (fp_first_diff_index < FRAME_SENTINEL_FP_WORDS) {
    violation.fp_first_diff_address =
        scope.geometry_entry.fp_frame_address +
        fp_first_diff_index * sizeof(uint32_t);
    violation.fp_first_diff_entry = scope.fp_entry[fp_first_diff_index];
    violation.fp_first_diff_exit = fp_exit[fp_first_diff_index];
  }
  for (uint32_t i = 0U; i < FRAME_SENTINEL_FP_WORDS; ++i) {
    violation.fp_entry[i] = scope.fp_entry[i];
    violation.fp_exit[i] = fp_exit ? fp_exit[i] : 0U;
  }

  violation.context_count = scope.context_count;
  violation.context_diff_mask = context_diff_mask;
  violation.context_entry_checksum = scope.context_entry_checksum;
  violation.context_exit_checksum = context_exit_checksum;
  violation.context_first_diff_index = context_first_diff_index;
  if (context_first_diff_index < scope.context_count) {
    violation.context_first_diff_address =
        scope.geometry_entry.context_address +
        context_first_diff_index * sizeof(uint32_t);
    violation.context_first_diff_entry =
        scope.context_entry[context_first_diff_index];
    violation.context_first_diff_exit =
        context_exit[context_first_diff_index];
  }
  for (uint32_t i = 0U; i < FRAME_SENTINEL_CONTEXT_WORDS; ++i) {
    violation.context_entry[i] = scope.context_entry[i];
    violation.context_exit[i] = context_exit ? context_exit[i] : 0U;
  }

  frame_sentinel_capture_victim(violation);
  violation.schema_version = FRAME_SENTINEL_SCHEMA_VERSION;
  violation.magic_inv = ~FRAME_SENTINEL_MAGIC;
  violation.magic = FRAME_SENTINEL_MAGIC;
}

static void frame_sentinel_latch(
    const frame_sentinel_scope_t& scope,
    uint32_t slot,
    uint32_t kind,
    uint32_t exc_return_exit,
    uint32_t caller_sp_exit,
    uint32_t dwt_exit,
    uint32_t fpccr_exit,
    uint32_t fpcar_exit,
    const frame_sentinel_geometry_t& geometry_exit,
    const uint32_t* frame_exit,
    uint32_t frame_diff_mask,
    uint32_t frame_exit_checksum,
    const uint32_t* fp_exit,
    uint32_t fp_diff_mask,
    uint32_t fp_exit_checksum,
    uint32_t fp_first_diff_index,
    const uint32_t* context_exit,
    uint32_t context_diff_mask,
    uint32_t context_exit_checksum,
    uint32_t context_first_diff_index) {
  g_frame_sentinel_violations++;
  g_frame_sentinel_event_pending = true;

  frame_sentinel_fill_violation(
      g_frame_sentinel_violation_last, scope, slot, kind,
      exc_return_exit, caller_sp_exit, dwt_exit, fpccr_exit, fpcar_exit,
      geometry_exit, frame_exit, frame_diff_mask, frame_exit_checksum,
      fp_exit, fp_diff_mask, fp_exit_checksum, fp_first_diff_index,
      context_exit, context_diff_mask, context_exit_checksum,
      context_first_diff_index);
  g_frame_sentinel_violation_last_present = true;

  if (frame_sentinel_violation_valid(
          g_frame_sentinel_violation_retained)) {
    g_frame_sentinel_suppressed++;
    return;
  }

  frame_sentinel_fill_violation(
      g_frame_sentinel_violation_retained, scope, slot, kind,
      exc_return_exit, caller_sp_exit, dwt_exit, fpccr_exit, fpcar_exit,
      geometry_exit, frame_exit, frame_diff_mask, frame_exit_checksum,
      fp_exit, fp_diff_mask, fp_exit_checksum, fp_first_diff_index,
      context_exit, context_diff_mask, context_exit_checksum,
      context_first_diff_index);
  arm_dcache_flush((void*)&g_frame_sentinel_violation_retained,
                   sizeof(g_frame_sentinel_violation_retained));
}

void frame_sentinel_exit_ex(uint32_t slot,
                            uint32_t exc_return_exit,
                            uint32_t caller_sp) {
  if (slot >= FRAME_SENTINEL_SLOT_COUNT) {
    g_frame_sentinel_bad_slot++;
    return;
  }

  frame_sentinel_scope_t& scope = g_frame_sentinel_scopes[slot];
  g_frame_sentinel_exits++;
  scope.exits++;
  if (scope.active == 0U) {
    scope.unbalanced_exit++;
    return;
  }
  scope.active = 0U;

  const uint32_t dwt_exit = ARM_DWT_CYCCNT;
  const uint32_t fpccr_exit = FRAME_SENTINEL_FPCCR;
  const uint32_t fpcar_exit = FRAME_SENTINEL_FPCAR;
  const bool lspact_consumed =
      (scope.fpccr_entry & 0x1U) != 0U &&
      (fpccr_exit & 0x1U) == 0U;
  if (lspact_consumed) scope.lspact_consumed++;

  uint32_t frame_exit[FRAME_SENTINEL_BASIC_WORDS] = {};
  const uint32_t fp_exit_empty[FRAME_SENTINEL_FP_WORDS] = {};
  const uint32_t context_exit_empty[FRAME_SENTINEL_CONTEXT_WORDS] = {};
  const frame_sentinel_geometry_t no_exit_geometry = {};

  const bool geometry_scalars_valid =
      scope.geometry_entry.valid <= 1U &&
      scope.geometry_entry.extended <= 1U &&
      scope.geometry_entry.alignment_word <= 1U &&
      scope.geometry_entry.core_content_plausible <= 1U &&
      scope.geometry_entry.fp_evidence_trusted <= 1U &&
      scope.geometry_entry.source <= FRAME_SENTINEL_SOURCE_SCAN;

  bool metadata_valid = geometry_scalars_valid;
  if (scope.context_count != 0U &&
      scope.context_count != FRAME_SENTINEL_CONTEXT_WORDS) {
    metadata_valid = false;
  }

  if (scope.geometry_entry.valid != 0U) {
    if (!frame_sentinel_range_readable(
            scope.geometry_entry.basic_frame_address,
            FRAME_SENTINEL_BASIC_WORDS * sizeof(uint32_t))) {
      metadata_valid = false;
    }
    if (scope.geometry_entry.context_address != 0U &&
        !frame_sentinel_range_readable(
            scope.geometry_entry.context_address,
            FRAME_SENTINEL_CONTEXT_WORDS * sizeof(uint32_t))) {
      metadata_valid = false;
    }
    if (scope.geometry_entry.extended != 0U &&
        scope.geometry_entry.fp_frame_address != 0U &&
        !frame_sentinel_range_readable(
            scope.geometry_entry.fp_frame_address,
            FRAME_SENTINEL_FP_WORDS * sizeof(uint32_t))) {
      metadata_valid = false;
    }
  }

  if (!metadata_valid) {
    frame_sentinel_latch(
        scope, slot, FRAME_SENTINEL_KIND_SCOPE_METADATA_CORRUPT,
        exc_return_exit, caller_sp, dwt_exit, fpccr_exit, fpcar_exit,
        no_exit_geometry,
        frame_exit, 0U, 0U,
        fp_exit_empty, 0U, 0U, UINT32_MAX,
        context_exit_empty, 0U, 0U, UINT32_MAX);
    frame_sentinel_beacon_pop(dwt_exit);
    return;
  }

  uint32_t frame_diff_mask = 0U;
  if (scope.geometry_entry.valid != 0U &&
      frame_sentinel_copy_words(
          scope.geometry_entry.basic_frame_address,
          frame_exit,
          FRAME_SENTINEL_BASIC_WORDS)) {
    for (uint32_t i = 0U; i < FRAME_SENTINEL_BASIC_WORDS; ++i) {
      if (frame_exit[i] != scope.frame_entry[i]) {
        frame_diff_mask |= 1U << i;
      }
    }
    if (frame_diff_mask != 0U) {
      frame_sentinel_latch(
          scope, slot, FRAME_SENTINEL_KIND_FRAME_WORD_CHANGED,
          exc_return_exit, caller_sp, dwt_exit, fpccr_exit, fpcar_exit,
          no_exit_geometry,
          frame_exit, frame_diff_mask, 0U,
          fp_exit_empty, 0U, 0U, UINT32_MAX,
          context_exit_empty, 0U, 0U, UINT32_MAX);
      frame_sentinel_beacon_pop(dwt_exit);
      return;
    }
  }

  const bool trusted_lazy_fp_transition =
      lspact_consumed &&
      scope.geometry_entry.valid != 0U &&
      scope.geometry_entry.extended != 0U &&
      scope.geometry_entry.fp_evidence_trusted != 0U &&
      scope.geometry_entry.source == FRAME_SENTINEL_SOURCE_FPCAR;
  if (trusted_lazy_fp_transition) {
    frame_sentinel_beacon_pop(dwt_exit);
    return;
  }

  uint32_t kind = 0U;
  const uint32_t effective_exc_return =
      frame_sentinel_exc_return_plausible(exc_return_exit)
          ? exc_return_exit
          : scope.exc_return_entry;
  const frame_sentinel_geometry_t geometry_exit =
      frame_sentinel_locate_geometry(
          scope.ipsr, effective_exc_return, caller_sp,
          fpccr_exit, fpcar_exit);

  uint32_t fp_exit[FRAME_SENTINEL_FP_WORDS] = {};
  uint32_t fp_diff_mask = 0U;
  uint32_t fp_first_diff_index = UINT32_MAX;
  if (scope.geometry_entry.extended != 0U &&
      scope.geometry_entry.fp_frame_address != 0U &&
      frame_sentinel_copy_words(
          scope.geometry_entry.fp_frame_address,
          fp_exit,
          FRAME_SENTINEL_FP_WORDS)) {
    for (uint32_t i = 0U; i < FRAME_SENTINEL_FP_WORDS; ++i) {
      if (fp_exit[i] != scope.fp_entry[i]) {
        fp_diff_mask |= 1U << i;
        if (fp_first_diff_index == UINT32_MAX) fp_first_diff_index = i;
      }
    }
    if (fp_diff_mask != 0U &&
        scope.geometry_entry.fp_evidence_trusted != 0U) {
      kind |= FRAME_SENTINEL_KIND_FP_WORD_CHANGED;
    }
  }

  uint32_t context_exit[FRAME_SENTINEL_CONTEXT_WORDS] = {};
  uint32_t context_diff_mask = 0U;
  uint32_t context_first_diff_index = UINT32_MAX;
  if (scope.geometry_entry.context_address != 0U &&
      scope.context_count == FRAME_SENTINEL_CONTEXT_WORDS &&
      frame_sentinel_copy_words(
          scope.geometry_entry.context_address,
          context_exit,
          FRAME_SENTINEL_CONTEXT_WORDS)) {
    for (uint32_t i = 0U; i < FRAME_SENTINEL_CONTEXT_WORDS; ++i) {
      if (context_exit[i] != scope.context_entry[i]) {
        context_diff_mask |= 1U << i;
        if (context_first_diff_index == UINT32_MAX) {
          context_first_diff_index = i;
        }
      }
    }
    if (context_diff_mask != 0U) {
      kind |= FRAME_SENTINEL_KIND_CONTEXT_WORD_CHANGED;
    }
  }

  if (caller_sp != 0U && scope.caller_sp_entry != 0U &&
      caller_sp != scope.caller_sp_entry) {
    kind |= FRAME_SENTINEL_KIND_CALLER_SP_CHANGED;
  }

  if (frame_sentinel_exc_return_plausible(exc_return_exit) &&
      frame_sentinel_exc_return_extended(scope.exc_return_entry) !=
          frame_sentinel_exc_return_extended(exc_return_exit)) {
    kind |= FRAME_SENTINEL_KIND_FRAME_TYPE_CHANGED;
    scope.frame_type_changed++;
  }

  if (scope.geometry_entry.valid != 0U && geometry_exit.valid != 0U &&
      (scope.geometry_entry.basic_frame_address !=
           geometry_exit.basic_frame_address ||
       scope.geometry_entry.raw_frame_address !=
           geometry_exit.raw_frame_address ||
       scope.geometry_entry.extended != geometry_exit.extended)) {
    kind |= FRAME_SENTINEL_KIND_FP_GEOMETRY_CHANGED;
    scope.geometry_changed++;
  }

  if (kind != 0U) {
    frame_sentinel_latch(
        scope, slot, kind, exc_return_exit, caller_sp, dwt_exit,
        fpccr_exit, fpcar_exit, geometry_exit,
        frame_exit, frame_diff_mask, 0U,
        fp_exit, fp_diff_mask, 0U, fp_first_diff_index,
        context_exit, context_diff_mask, 0U,
        context_first_diff_index);
  }

  frame_sentinel_beacon_pop(dwt_exit);
}

void frame_sentinel_exit(uint32_t slot, uint32_t caller_sp) {
  frame_sentinel_exit_ex(slot, 0U, caller_sp);
}

static const char* frame_sentinel_source_name(uint32_t source) {
  switch (source) {
    case FRAME_SENTINEL_SOURCE_FPCAR: return "FPCAR";
    case FRAME_SENTINEL_SOURCE_PSP:   return "PSP";
    case FRAME_SENTINEL_SOURCE_SCAN:  return "SCAN";
    default:                          return "NONE";
  }
}

static void frame_sentinel_emit(void) {
  const frame_sentinel_violation_t& violation =
      g_frame_sentinel_violation_last;

  Payload event;
  event.add("schema", "ZPNET_FRAME_SENTINEL_V6_ANOMALY");
  event.add("name", violation.name);
  event.add("slot", violation.slot);
  event.add("kind", violation.kind);
  event.add_fmt("frame_diff", "0x%02lX",
                (unsigned long)violation.frame_diff_mask);
  event.add_fmt("context_diff", "0x%08lX",
                (unsigned long)violation.context_diff_mask);
  event.add("victim", violation.victim_name);
  event.add_fmt("first_changed_address", "0x%08lX",
                (unsigned long)violation.context_first_diff_address);
  enqueueEvent("SENTINEL_ANOMALY", event);
}

void frame_sentinel_service(void) {
  if (!g_frame_sentinel_event_pending ||
      g_frame_sentinel_event_emitted ||
      !g_frame_sentinel_violation_last_present) {
    return;
  }

  g_frame_sentinel_event_emitted = true;
  g_frame_sentinel_event_pending = false;
  frame_sentinel_emit();
}

Payload frame_sentinel_report_payload(void) {
  frame_sentinel_init();

  const frame_sentinel_violation_t* violation = nullptr;
  const char* bank = "none";
  if (frame_sentinel_violation_valid(
          g_frame_sentinel_violation_retained)) {
    violation = &g_frame_sentinel_violation_retained;
    bank = "retained";
  } else if (g_frame_sentinel_violation_last_present &&
             frame_sentinel_violation_valid(
                 g_frame_sentinel_violation_last)) {
    violation = &g_frame_sentinel_violation_last;
    bank = "live";
  }

  Payload report;
  report.add("schema", "ZPNET_FRAME_SENTINEL_V6");
  report.add("version", FRAME_SENTINEL_SCHEMA_VERSION);
  report.add("standalone", true);
  report.add("service", "main_loop");
  report.add("fpu_aware", true);
  report.add("exit_checksums_enabled", false);
  report.add("core_change_fast_latch", true);
  report.add("fixed_exit_bounds", true);
  report.add("scope_metadata_court", true);
  report.add("trusted_lazy_fp_core_only", true);
  report.add("slot_count", (uint32_t)FRAME_SENTINEL_SLOT_COUNT);
  report.add("basic_frame_words", FRAME_SENTINEL_BASIC_WORDS);
  report.add("fp_frame_words", FRAME_SENTINEL_FP_WORDS);
  report.add("context_words", FRAME_SENTINEL_CONTEXT_WORDS);
  report.add("enters", g_frame_sentinel_enters);
  report.add("exits", g_frame_sentinel_exits);
  report.add("bad_slot", g_frame_sentinel_bad_slot);
  report.add("violations", g_frame_sentinel_violations);
  report.add("suppressed", g_frame_sentinel_suppressed);
  report.add("active_depth", g_frame_sentinel_active_depth);
  report.add("violation_present", violation != nullptr);
  report.add("violation_bank", bank);

  if (violation) {
    Payload evidence;
    evidence.add("name", violation->name);
    evidence.add("slot", violation->slot);
    evidence.add("pass", violation->pass);
    evidence.add("kind", violation->kind);
    evidence.add("kind_scope_metadata_corrupt",
                 (violation->kind &
                  FRAME_SENTINEL_KIND_SCOPE_METADATA_CORRUPT) != 0U);
    evidence.add("ipsr", violation->ipsr);
    evidence.add_fmt("exc_return_entry", "0x%08lX",
                     (unsigned long)violation->exc_return_entry);
    evidence.add_fmt("exc_return_exit", "0x%08lX",
                     (unsigned long)violation->exc_return_exit);
    evidence.add("exc_return_entry_valid",
                 violation->exc_return_entry_valid != 0U);
    evidence.add("exc_return_exit_valid",
                 violation->exc_return_exit_valid != 0U);
    evidence.add("frame_type_entry_known",
                 violation->exc_return_entry_valid != 0U);
    evidence.add("frame_type_exit_known",
                 violation->exc_return_exit_valid != 0U);
    evidence.add("frame_type_entry_extended",
                 violation->frame_type_entry_extended != 0U);
    evidence.add("frame_type_exit_extended",
                 violation->frame_type_exit_extended != 0U);
    evidence.add_fmt("caller_sp_entry", "0x%08lX",
                     (unsigned long)violation->caller_sp_entry);
    evidence.add_fmt("caller_sp_exit", "0x%08lX",
                     (unsigned long)violation->caller_sp_exit);
    evidence.add_fmt("fpccr_entry", "0x%08lX",
                     (unsigned long)violation->fpccr_entry);
    evidence.add_fmt("fpccr_exit", "0x%08lX",
                     (unsigned long)violation->fpccr_exit);
    evidence.add_fmt("fpcar_entry", "0x%08lX",
                     (unsigned long)violation->fpcar_entry);
    evidence.add_fmt("fpcar_exit", "0x%08lX",
                     (unsigned long)violation->fpcar_exit);
    evidence.add("lspact_consumed",
                 violation->lspact_consumed != 0U);

    Payload entry_geometry;
    entry_geometry.add("valid", violation->geometry_entry.valid != 0U);
    entry_geometry.add("source",
        frame_sentinel_source_name(violation->geometry_entry.source));
    entry_geometry.add("extended",
                       violation->geometry_entry.extended != 0U);
    entry_geometry.add("alignment_word",
                       violation->geometry_entry.alignment_word != 0U);
    entry_geometry.add("core_content_plausible",
                       violation->geometry_entry.core_content_plausible != 0U);
    entry_geometry.add("fp_evidence_trusted",
                       violation->geometry_entry.fp_evidence_trusted != 0U);
    entry_geometry.add_fmt("raw_frame", "0x%08lX",
        (unsigned long)violation->geometry_entry.raw_frame_address);
    entry_geometry.add_fmt("fp_frame", "0x%08lX",
        (unsigned long)violation->geometry_entry.fp_frame_address);
    entry_geometry.add_fmt("basic_frame", "0x%08lX",
        (unsigned long)violation->geometry_entry.basic_frame_address);
    entry_geometry.add_fmt("context", "0x%08lX",
        (unsigned long)violation->geometry_entry.context_address);
    evidence.add_object("entry_geometry", entry_geometry);

    Payload exit_geometry;
    exit_geometry.add("valid", violation->geometry_exit.valid != 0U);
    exit_geometry.add("source",
        frame_sentinel_source_name(violation->geometry_exit.source));
    exit_geometry.add("extended",
                      violation->geometry_exit.extended != 0U);
    exit_geometry.add("alignment_word",
                      violation->geometry_exit.alignment_word != 0U);
    exit_geometry.add("core_content_plausible",
                      violation->geometry_exit.core_content_plausible != 0U);
    exit_geometry.add("fp_evidence_trusted",
                      violation->geometry_exit.fp_evidence_trusted != 0U);
    exit_geometry.add_fmt("raw_frame", "0x%08lX",
        (unsigned long)violation->geometry_exit.raw_frame_address);
    exit_geometry.add_fmt("fp_frame", "0x%08lX",
        (unsigned long)violation->geometry_exit.fp_frame_address);
    exit_geometry.add_fmt("basic_frame", "0x%08lX",
        (unsigned long)violation->geometry_exit.basic_frame_address);
    exit_geometry.add_fmt("context", "0x%08lX",
        (unsigned long)violation->geometry_exit.context_address);
    evidence.add_object("exit_geometry", exit_geometry);

    evidence.add_fmt("frame_diff_mask", "0x%02lX",
                     (unsigned long)violation->frame_diff_mask);
    evidence.add_fmt("fp_diff_mask", "0x%08lX",
                     (unsigned long)violation->fp_diff_mask);
    evidence.add("fp_first_diff_index",
                 violation->fp_first_diff_index);
    evidence.add_fmt("fp_first_diff_address", "0x%08lX",
                     (unsigned long)violation->fp_first_diff_address);
    evidence.add_fmt("fp_first_diff_entry", "0x%08lX",
                     (unsigned long)violation->fp_first_diff_entry);
    evidence.add_fmt("fp_first_diff_exit", "0x%08lX",
                     (unsigned long)violation->fp_first_diff_exit);
    evidence.add("context_count", violation->context_count);
    evidence.add_fmt("context_diff_mask", "0x%08lX",
                     (unsigned long)violation->context_diff_mask);
    evidence.add("context_first_diff_index",
                 violation->context_first_diff_index);
    evidence.add_fmt("context_first_diff_address", "0x%08lX",
                     (unsigned long)violation->context_first_diff_address);
    evidence.add_fmt("context_first_diff_entry", "0x%08lX",
                     (unsigned long)violation->context_first_diff_entry);
    evidence.add_fmt("context_first_diff_exit", "0x%08lX",
                     (unsigned long)violation->context_first_diff_exit);
    evidence.add("victim_depth", violation->victim_depth);
    evidence.add("victim", violation->victim_name);
    evidence.add("victim_slot", violation->victim_slot);
    evidence.add("victim_pass", violation->victim_pass);
    evidence.add("victim_ipsr", violation->victim_ipsr);
    report.add_object("evidence", evidence);
  }

  Payload beacon;
  const bool retained_beacon_valid =
      frame_sentinel_beacon_valid(g_frame_sentinel_beacon_retained);
  beacon.add("valid", retained_beacon_valid);
  if (retained_beacon_valid) {
    beacon.add("depth", g_frame_sentinel_beacon_retained.depth);
    beacon.add("overflow", g_frame_sentinel_beacon_retained.overflow);
    if (g_frame_sentinel_beacon_retained.depth != 0U) {
      const frame_sentinel_beacon_entry_t& active =
          g_frame_sentinel_beacon_retained.active[0];
      beacon.add("outermost_name", active.name);
      beacon.add("outermost_slot", active.slot);
      beacon.add("outermost_pass", active.pass);
      beacon.add("outermost_ipsr", active.ipsr);
    }
    beacon.add("last_completed",
               g_frame_sentinel_beacon_retained.last_completed.name);
  }
  report.add_object("previous_boot_beacon", beacon);
  return report;
}

void frame_sentinel_clear(void) {
  memset((void*)&g_frame_sentinel_violation_retained, 0,
         sizeof(g_frame_sentinel_violation_retained));
  arm_dcache_flush((void*)&g_frame_sentinel_violation_retained,
                   sizeof(g_frame_sentinel_violation_retained));

  memset((void*)&g_frame_sentinel_beacon_retained, 0,
         sizeof(g_frame_sentinel_beacon_retained));
  arm_dcache_flush((void*)&g_frame_sentinel_beacon_retained,
                   sizeof(g_frame_sentinel_beacon_retained));

  memset((void*)&g_frame_sentinel_violation_last, 0,
         sizeof(g_frame_sentinel_violation_last));
  g_frame_sentinel_violation_last_present = false;
  g_frame_sentinel_event_pending = false;
  g_frame_sentinel_event_emitted = false;
}

#endif  // Retired Frame Sentinel
