#include "execution_trace.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stddef.h>
#include <string.h>

// ============================================================================
// Storage
// ============================================================================

static constexpr uint32_t EXECUTION_TRACE_BANK_MAGIC = 0x45544233UL;  // 'ETB3'
static constexpr uint32_t EXECUTION_TRACE_SET_MAGIC  = 0x45545333UL;  // 'ETS3'
static constexpr uint32_t EXECUTION_TRACE_SCHEMA_VERSION = 3U;
static constexpr uint32_t EXECUTION_TRACE_FLAG_FAULT_CAPTURED = 1U;

struct alignas(32) execution_trace_bank_t {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t schema_version;
  uint32_t context;
  uint32_t next_sequence;
  uint32_t reserved[3];
  execution_trace_entry_t entries[EXECUTION_TRACE_ENTRIES_PER_CONTEXT];
};

static_assert((sizeof(execution_trace_bank_t) % 32U) == 0U,
              "Execution Trace bank must occupy complete cache lines");

struct alignas(32) execution_trace_retained_set_t {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t schema_version;
  uint32_t context_count;
  uint32_t fault_dwt;
  uint32_t crash_sequence;
  uint32_t flags;
  uint32_t reserved;
  execution_trace_bank_t banks[EXECUTION_TRACE_CONTEXT_COUNT];
};

static_assert((sizeof(execution_trace_retained_set_t) % 32U) == 0U,
              "Execution Trace retained set must occupy complete cache lines");

static execution_trace_bank_t
    g_execution_trace_live[EXECUTION_TRACE_CONTEXT_COUNT] = {
  { EXECUTION_TRACE_BANK_MAGIC, ~EXECUTION_TRACE_BANK_MAGIC,
    EXECUTION_TRACE_SCHEMA_VERSION,
    (uint32_t)execution_trace_context_t::PRIORITY0, 0U, {0U, 0U, 0U}, {} },
  { EXECUTION_TRACE_BANK_MAGIC, ~EXECUTION_TRACE_BANK_MAGIC,
    EXECUTION_TRACE_SCHEMA_VERSION,
    (uint32_t)execution_trace_context_t::PRIORITY16, 0U, {0U, 0U, 0U}, {} },
  { EXECUTION_TRACE_BANK_MAGIC, ~EXECUTION_TRACE_BANK_MAGIC,
    EXECUTION_TRACE_SCHEMA_VERSION,
    (uint32_t)execution_trace_context_t::PRIORITY32, 0U, {0U, 0U, 0U}, {} },
  { EXECUTION_TRACE_BANK_MAGIC, ~EXECUTION_TRACE_BANK_MAGIC,
    EXECUTION_TRACE_SCHEMA_VERSION,
    (uint32_t)execution_trace_context_t::FOREGROUND, 0U, {0U, 0U, 0U}, {} },
};
alignas(32) static execution_trace_retained_set_t
    g_execution_trace_retained DMAMEM;

// ============================================================================
// Tiny primitives
// ============================================================================

static inline void execution_trace_dmb(void) {
  __asm__ volatile("dmb" ::: "memory");
}

static inline uint32_t execution_trace_ipsr(void) {
  uint32_t ipsr = 0U;
#if defined(__arm__)
  __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr) :: "memory");
#endif
  return ipsr & 0x1FFU;
}

static int execution_trace_context_index(execution_trace_context_t context) {
  const uint32_t value = (uint32_t)context;
  if (value < (uint32_t)execution_trace_context_t::PRIORITY0 ||
      value > (uint32_t)execution_trace_context_t::FOREGROUND) {
    return -1;
  }
  return (int)(value - (uint32_t)execution_trace_context_t::PRIORITY0);
}

static bool execution_trace_bank_valid(const execution_trace_bank_t& bank,
                                       execution_trace_context_t context) {
  return bank.magic == EXECUTION_TRACE_BANK_MAGIC &&
         (bank.magic ^ bank.magic_inv) == 0xFFFFFFFFUL &&
         bank.schema_version == EXECUTION_TRACE_SCHEMA_VERSION &&
         bank.context == (uint32_t)context;
}

static bool execution_trace_entry_valid(const execution_trace_entry_t& entry) {
  return entry.sequence != 0U &&
         (entry.sequence ^ entry.sequence_inv) == 0xFFFFFFFFUL;
}

static execution_trace_bank_t* execution_trace_live_bank(
    execution_trace_context_t context) {
  const int index = execution_trace_context_index(context);
  if (index < 0) return nullptr;
  execution_trace_bank_t& bank = g_execution_trace_live[index];
  return execution_trace_bank_valid(bank, context) ? &bank : nullptr;
}

static bool execution_trace_retained_valid(void) {
  return g_execution_trace_retained.magic == EXECUTION_TRACE_SET_MAGIC &&
         (g_execution_trace_retained.magic ^
          g_execution_trace_retained.magic_inv) == 0xFFFFFFFFUL &&
         g_execution_trace_retained.schema_version ==
             EXECUTION_TRACE_SCHEMA_VERSION &&
         g_execution_trace_retained.context_count ==
             EXECUTION_TRACE_CONTEXT_COUNT;
}

// ============================================================================
// Recording
// ============================================================================

void __attribute__((noinline)) execution_trace_record(
    execution_trace_context_t context,
    execution_trace_stage_t stage,
    execution_trace_kind_t kind,
    uint32_t phase,
    uint32_t lineage_id,
    uint32_t subject_index,
    uint32_t identity,
    uint32_t target,
    uint32_t related_target,
    uint32_t object,
    uint32_t aux) {
  execution_trace_bank_t* const bank = execution_trace_live_bank(context);
  if (!bank) return;

  uint32_t sequence = bank->next_sequence + 1U;
  if (sequence == 0U) sequence = 1U;
  bank->next_sequence = sequence;

  execution_trace_entry_t& entry =
      bank->entries[(sequence - 1U) % EXECUTION_TRACE_ENTRIES_PER_CONTEXT];

  // Invalidate before reuse.  If a fault preempts this writer, fault capture
  // preserves either the previous committed cell or this explicitly invalid one.
  entry.sequence = 0U;
  entry.sequence_inv = 0U;
  execution_trace_dmb();

  entry.dwt = ARM_DWT_CYCCNT;
  entry.context = (uint32_t)context;
  entry.ipsr = execution_trace_ipsr();
  entry.stage = (uint32_t)stage;
  entry.phase = phase;
  entry.kind = (uint32_t)kind;
  entry.lineage_id = lineage_id;
  entry.subject_index = subject_index;
  entry.identity = identity;
  entry.target = target;
  entry.related_target = related_target;
  entry.object = object;
  entry.site_pc =
      (uint32_t)(uintptr_t)__builtin_return_address(0);
  entry.aux = aux;

  entry.sequence_inv = ~sequence;
  execution_trace_dmb();
  entry.sequence = sequence;  // commit last
  execution_trace_dmb();
}

// ============================================================================
// Snapshot
// ============================================================================

static void execution_trace_snapshot_bank(
    const execution_trace_bank_t& bank,
    execution_trace_context_t context,
    execution_trace_context_snapshot_t* out) {
  memset((void*)out, 0, sizeof(*out));
  out->context = (uint32_t)context;
  out->valid = execution_trace_bank_valid(bank, context);
  if (!out->valid) return;

  out->sequence_at_capture = bank.next_sequence;

  for (uint32_t i = 0U; i < EXECUTION_TRACE_ENTRIES_PER_CONTEXT; ++i) {
    const volatile execution_trace_entry_t* const source = &bank.entries[i];
    const uint32_t sequence_before = source->sequence;
    const uint32_t sequence_inv_before = source->sequence_inv;
    if (sequence_before == 0U ||
        (sequence_before ^ sequence_inv_before) != 0xFFFFFFFFUL) {
      continue;
    }

    execution_trace_entry_t candidate = bank.entries[i];
    execution_trace_dmb();
    if (source->sequence != sequence_before ||
        source->sequence_inv != sequence_inv_before ||
        !execution_trace_entry_valid(candidate)) {
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

void execution_trace_snapshot(execution_trace_snapshot_t* out) {
  if (!out) return;
  memset((void*)out, 0, sizeof(*out));

  for (uint32_t i = 0U; i < EXECUTION_TRACE_CONTEXT_COUNT; ++i) {
    const execution_trace_context_t context =
        (execution_trace_context_t)(
            (uint32_t)execution_trace_context_t::PRIORITY0 + i);
    execution_trace_bank_t* const live = execution_trace_live_bank(context);
    if (live) {
      execution_trace_snapshot_bank(*live, context, &out->live[i]);
    }
  }

  out->retained_valid = execution_trace_retained_valid();
  if (!out->retained_valid) {
    for (uint32_t i = 0U; i < EXECUTION_TRACE_CONTEXT_COUNT; ++i) {
      out->retained[i].context =
          (uint32_t)execution_trace_context_t::PRIORITY0 + i;
    }
    return;
  }

  out->fault_captured =
      (g_execution_trace_retained.flags &
       EXECUTION_TRACE_FLAG_FAULT_CAPTURED) != 0U;
  out->fault_dwt = g_execution_trace_retained.fault_dwt;
  out->crash_sequence = g_execution_trace_retained.crash_sequence;

  for (uint32_t i = 0U; i < EXECUTION_TRACE_CONTEXT_COUNT; ++i) {
    const execution_trace_context_t context =
        (execution_trace_context_t)(
            (uint32_t)execution_trace_context_t::PRIORITY0 + i);
    execution_trace_snapshot_bank(g_execution_trace_retained.banks[i],
                                  context,
                                  &out->retained[i]);
  }
}

const char* execution_trace_context_name(uint32_t context) {
  switch ((execution_trace_context_t)context) {
    case execution_trace_context_t::PRIORITY0:  return "PRIORITY0";
    case execution_trace_context_t::PRIORITY16: return "PRIORITY16";
    case execution_trace_context_t::PRIORITY32: return "PRIORITY32";
    case execution_trace_context_t::FOREGROUND: return "FOREGROUND";
    default:                                     return "NONE";
  }
}

// ============================================================================
// Fault freeze / retained clearing
// ============================================================================

extern "C" void execution_trace_capture_fault(uint32_t fault_dwt,
                                               uint32_t crash_sequence) {
  execution_trace_retained_set_t& retained = g_execution_trace_retained;

  // Persistently invalidate the previous retained publication before replacing
  // it.  A nested failure during this best-effort copy must not resurrect the
  // older transcript after reboot.
  retained.magic = 0U;
  retained.magic_inv = 0U;
  execution_trace_dmb();
  arm_dcache_flush_delete(&retained, 32U);
  __asm__ volatile("dsb\nisb" ::: "memory");

  retained.schema_version = EXECUTION_TRACE_SCHEMA_VERSION;
  retained.context_count = EXECUTION_TRACE_CONTEXT_COUNT;
  retained.fault_dwt = fault_dwt;
  retained.crash_sequence = crash_sequence;
  retained.flags = EXECUTION_TRACE_FLAG_FAULT_CAPTURED;
  retained.reserved = 0U;

  volatile uint32_t* const destination =
      reinterpret_cast<volatile uint32_t*>(retained.banks);
  const volatile uint32_t* const source =
      reinterpret_cast<const volatile uint32_t*>(g_execution_trace_live);
  constexpr size_t bank_bytes = sizeof(retained.banks);
  static_assert((bank_bytes % sizeof(uint32_t)) == 0U,
                "Execution Trace banks must be word-copyable");
  constexpr size_t bank_words = bank_bytes / sizeof(uint32_t);
  for (size_t i = 0U; i < bank_words; ++i) {
    destination[i] = source[i];
  }

  retained.magic_inv = ~EXECUTION_TRACE_SET_MAGIC;
  execution_trace_dmb();
  retained.magic = EXECUTION_TRACE_SET_MAGIC;  // commit last
  __asm__ volatile("dsb\nisb" ::: "memory");

  arm_dcache_flush_delete(&retained, sizeof(retained));
  __asm__ volatile("dsb\nisb" ::: "memory");
}

void execution_trace_clear_retained(void) {
  memset((void*)&g_execution_trace_retained, 0,
         sizeof(g_execution_trace_retained));
  arm_dcache_flush_delete(&g_execution_trace_retained,
                          sizeof(g_execution_trace_retained));
  __asm__ volatile("dsb\nisb" ::: "memory");
}
