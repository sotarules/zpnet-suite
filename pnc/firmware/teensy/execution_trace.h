#pragma once

#include <stdint.h>

// ============================================================================
// Execution Trace — per-execution-context causal breadcrumbs
// ============================================================================
//
// Each execution class owns an independent live ring.  A writer may be
// preempted only by a different execution class, so no cross-context atomic
// sequence allocator is required.  DWT supplies the common chronological ruler;
// lineage_id carries immutable event/transaction identity across custody layers.
//
// Live rings remain in ordinary RAM1.  Fault capture freezes all committed live
// entries into one retained RAM2 set before Teensyduino performs its reboot.
// Reporting may merge rings by DWT, but the per-context rings are the testimony.

static constexpr uint32_t EXECUTION_TRACE_ENTRIES_PER_CONTEXT = 32U;
static constexpr uint32_t EXECUTION_TRACE_CONTEXT_COUNT = 4U;
static constexpr uint32_t EXECUTION_TRACE_NO_SUBJECT = 0xFFFFFFFFUL;

enum class execution_trace_context_t : uint32_t {
  NONE       = 0,
  PRIORITY0  = 1,
  PRIORITY16 = 2,
  PRIORITY32 = 3,
  FOREGROUND = 4,
};

enum class execution_trace_stage_t : uint32_t {
  NONE                      = 0,
  DISPATCH_ENTER            = 1,
  PHASE_ASAP                = 2,
  DEFERRED_SELECTED         = 3,
  CALLBACK_ENTER            = 4,
  CALLBACK_RETURN           = 5,
  DEFERRED_CLEANUP          = 6,
  MUTATION_BARRIER_ENTER    = 7,
  MUTATION_SELECTED         = 8,
  MUTATION_RESULT           = 9,
  MUTATION_BARRIER_EXIT     = 10,
  PHASE_TIMED               = 11,
  TIMED_SELECTED            = 12,
  TIMED_SLOT_AFTER_CALLBACK = 13,
  REARM_BEGIN               = 14,
  REARM_END                 = 15,
  SLOT_RETIRED              = 16,
  SLOT_REPLACED             = 17,
  PHASE_ALAP                = 18,
  DISPATCH_LEAVE            = 19,
  IRQ_SELECTED              = 20,

  ISR_ENTER                 = 32,
  ISR_CAPTURED              = 33,
  ISR_EXIT                  = 34,

  HANDOFF_ENTER             = 40,
  HANDOFF_DEQUEUE           = 41,
  HANDOFF_EXIT              = 42,

  SUBSCRIBER_SELECTED       = 48,
  SUBSCRIBER_ENTER          = 49,
  SUBSCRIBER_RETURN         = 50,
};

enum class execution_trace_kind_t : uint32_t {
  NONE      = 0,
  DISPATCH  = 1,
  ASAP      = 2,
  TIMED     = 3,
  ALAP      = 4,
  ISR_TIMED = 5,
  MUTATION  = 6,
  REARM     = 7,

  ISR_QTIMER1       = 16,
  ISR_VCLOCK        = 17,
  ISR_OCXO1         = 18,
  ISR_OCXO2         = 19,
  ISR_PPS           = 20,
  INTERRUPT_HANDOFF = 21,
  SUBSCRIBER_VCLOCK = 22,
  SUBSCRIBER_OCXO1  = 23,
  SUBSCRIBER_OCXO2  = 24,
};

enum class execution_trace_phase_t : uint32_t {
  IDLE               = 0,
  ASAP               = 1,
  TIMED              = 2,
  ALAP               = 3,
  APPLYING_MUTATIONS = 4,
};

struct execution_trace_entry_t {
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t dwt;
  uint32_t context;
  uint32_t ipsr;
  uint32_t stage;
  uint32_t phase;
  uint32_t kind;
  uint32_t lineage_id;
  uint32_t subject_index;
  uint32_t identity;
  uint32_t target;
  uint32_t related_target;
  uint32_t object;
  uint32_t site_pc;
  uint32_t aux;
};

static_assert(sizeof(execution_trace_entry_t) == 64U,
              "Execution Trace entry must remain 64 bytes");

struct execution_trace_context_snapshot_t {
  bool valid;
  uint32_t context;
  uint32_t count;
  uint32_t newest_sequence;
  uint32_t sequence_at_capture;
  execution_trace_entry_t entries[EXECUTION_TRACE_ENTRIES_PER_CONTEXT];
};

struct execution_trace_snapshot_t {
  bool retained_valid;
  bool fault_captured;
  uint32_t fault_dwt;
  uint32_t crash_sequence;
  execution_trace_context_snapshot_t live[EXECUTION_TRACE_CONTEXT_COUNT];
  execution_trace_context_snapshot_t retained[EXECUTION_TRACE_CONTEXT_COUNT];
};

const char* execution_trace_context_name(uint32_t context);

void execution_trace_record(execution_trace_context_t context,
                            execution_trace_stage_t stage,
                            execution_trace_kind_t kind,
                            uint32_t phase,
                            uint32_t lineage_id,
                            uint32_t subject_index,
                            uint32_t identity,
                            uint32_t target,
                            uint32_t related_target,
                            uint32_t object,
                            uint32_t aux);

void execution_trace_snapshot(execution_trace_snapshot_t* out);
void execution_trace_clear_retained(void);

extern "C" void execution_trace_capture_fault(uint32_t fault_dwt,
                                               uint32_t crash_sequence);

#define ZPNET_EXECUTION_TRACE(context, stage, kind, phase, lineage_id,        \
                              subject_index, identity, target, related_target, \
                              object, aux)                                     \
  execution_trace_record((context), (stage), (kind), (phase), (lineage_id),   \
                         (subject_index), (identity), (target),                \
                         (related_target), (object), (aux))

// Compatibility vocabulary for TimePop's existing breadcrumb sites.  The ABI
// now lives here rather than in process_timepop.h; these aliases keep the
// scheduling implementation readable while the recorder itself is independent.
using timepop_dispatch_trace_stage_t = execution_trace_stage_t;
using timepop_dispatch_trace_kind_t = execution_trace_kind_t;
using timepop_dispatch_trace_phase_t = execution_trace_phase_t;
using timepop_dispatch_trace_entry_t = execution_trace_entry_t;

static constexpr uint32_t TIMEPOP_DISPATCH_TRACE_ENTRIES =
    EXECUTION_TRACE_ENTRIES_PER_CONTEXT;
static constexpr uint32_t TIMEPOP_DISPATCH_TRACE_NO_SLOT =
    EXECUTION_TRACE_NO_SUBJECT;
