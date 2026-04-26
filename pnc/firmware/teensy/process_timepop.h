// ============================================================================
// process_timepop.h
// ============================================================================
//
// TimePop process interface.
//
// TimePop is a foreground scheduler with two internal timing modes:
//
//   • TICK mode:
//       The default mode used by timepop_arm(), timepop_arm_at(), and
//       timepop_arm_from_anchor().  Slots are scanned from the 1 kHz TICK
//       subscriber and user callbacks run in foreground callback context.
//       This is the mode for ordinary services: transport RX/TX, event
//       publishing, CPU usage, DAC pacing, PPS relay off, watchdog work,
//       and any callback that may touch foreground APIs.
//
//   • PRECISION / SPINDRY mode:
//       The opt-in mode used only by timepop_arm_at_spindry() and
//       timepop_arm_from_anchor_spindry().  TimePop asks process_interrupt
//       to arm the QTimer1 CH2 schedule-fire lane for an approach fire,
//       then spins on DWT and invokes the user callback in ISR context.
//
// TimePop owns no QTimer hardware.  process_interrupt remains the sole
// owner of QTimer1/QTimer3 registers.  TimePop uses the public
// interrupt_schedule_*() API only for the precision/SpinDry lane.
//
// All ordinary timed services are deliberately kept off the CH2 precision
// path.  This preserves the stability of the original 1 kHz scheduler and
// prevents recurring foreground work from flooding the ASAP queue with
// hardware-fire delivery contexts.
//
// ============================================================================

#pragma once

#include "timepop.h"
#include "process_interrupt.h"
#include <stdint.h>

void timepop_bootstrap(void);
void timepop_init(void);
void process_timepop_register(void);
