#pragma once

#include <Arduino.h>

class Payload;

// ============================================================================
// Frame Sentinel V2
// ============================================================================
//
// Frame Sentinel is an independent platform diagnostic.  It has no dependency
// on the SYSTEM memory watchdog, TimePop, or any application process.
//
// A scope may be placed around any code region.  In thread mode the scope is a
// retained victim/beacon marker.  In handler mode it additionally snapshots:
//
//   * the interrupted hardware basic exception frame;
//   * a bounded window of the interrupted ordinary stack above that frame;
//   * MSP, FPCCR, FPCAR, EXC_RETURN, IPSR, and DWT state.
//
// At scope exit the same state is compared.  The first violation is retained
// in RAM2 until FRAME_SENTINEL_CLEAR / SYSTEM.CRASH_CLEAR.
//
// Each handler that can preempt another instrumented scope must own a distinct
// slot.  Thread-mode victim regions also receive distinct slots so retained
// attribution is unambiguous.

enum {
  FRAME_SENTINEL_SLOT_HANDOFF_ISR            = 0,
  FRAME_SENTINEL_SLOT_DWT_PUBLICATION_VCLOCK = 1,
  FRAME_SENTINEL_SLOT_DWT_PUBLICATION_OCXO1  = 2,
  FRAME_SENTINEL_SLOT_DWT_PUBLICATION_OCXO2  = 3,
  FRAME_SENTINEL_SLOT_DWT_FATAL_VCLOCK       = 4,
  FRAME_SENTINEL_SLOT_DWT_FATAL_OCXO1        = 5,
  FRAME_SENTINEL_SLOT_DWT_FATAL_OCXO2        = 6,
  FRAME_SENTINEL_SLOT_CPU_USAGE              = 7,
  FRAME_SENTINEL_SLOT_QTIMER1_ISR            = 8,
  FRAME_SENTINEL_SLOT_QTIMER2_ISR            = 9,
  FRAME_SENTINEL_SLOT_QTIMER3_ISR            = 10,
  FRAME_SENTINEL_SLOT_PPS_GPIO_ISR           = 11,
  FRAME_SENTINEL_SLOT_FLOORLINE_SEAL         = 12,
  FRAME_SENTINEL_SLOT_MEMORY_AUDIT            = 13,
  FRAME_SENTINEL_SLOT_RESERVED_14            = 14,
  FRAME_SENTINEL_SLOT_RESERVED_15            = 15,
  FRAME_SENTINEL_SLOT_COUNT                  = 16,
};

enum : uint32_t {
  FRAME_SENTINEL_KIND_FRAME_WORD_CHANGED   = 1U << 0,
  FRAME_SENTINEL_KIND_CALLER_SP_CHANGED    = 1U << 1,
  FRAME_SENTINEL_KIND_CONTEXT_WORD_CHANGED = 1U << 2,
  FRAME_SENTINEL_KIND_FRAME_MOVED          = 1U << 3,
};

void frame_sentinel_init(void);
void frame_sentinel_enter(uint32_t slot,
                          const char* name,
                          uint32_t exc_return,
                          uint32_t caller_sp);
void frame_sentinel_exit(uint32_t slot, uint32_t caller_sp);

// Called directly by loop().  Emits at most one SENTINEL_ANOMALY event per boot.
void frame_sentinel_service(void);

// SYSTEM-facing inspection and explicit retained-state release.
Payload frame_sentinel_report_payload(void);
void frame_sentinel_clear(void);

// True while at least one instrumented scope is active.  Priority-zero wrappers
// may use this as a cheap first-instruction gate before entering their own slot.
bool frame_sentinel_scope_active(void);

#if defined(__arm__)
#define FRAME_SENTINEL_ENTER(slot, name)                                      \
  do {                                                                         \
    uint32_t frame_sentinel_exc_return_;                                       \
    uint32_t frame_sentinel_sp_;                                               \
    __asm__ volatile("mov %0, lr"                                              \
                     : "=r"(frame_sentinel_exc_return_));                      \
    __asm__ volatile("mov %0, sp"                                              \
                     : "=r"(frame_sentinel_sp_));                              \
    frame_sentinel_enter((slot), (name), frame_sentinel_exc_return_,           \
                         frame_sentinel_sp_);                                  \
  } while (0)

#define FRAME_SENTINEL_EXIT(slot)                                              \
  do {                                                                         \
    uint32_t frame_sentinel_sp_;                                               \
    __asm__ volatile("mov %0, sp"                                              \
                     : "=r"(frame_sentinel_sp_));                              \
    frame_sentinel_exit((slot), frame_sentinel_sp_);                           \
  } while (0)
#else
#define FRAME_SENTINEL_ENTER(slot, name)                                       \
  frame_sentinel_enter((slot), (name), 0U, 0U)
#define FRAME_SENTINEL_EXIT(slot) frame_sentinel_exit((slot), 0U)
#endif
