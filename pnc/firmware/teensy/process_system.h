// =============================================================
// FILE: process_system.h
// =============================================================
//
// SYSTEM Process (process_system)
//
// Role:
//   • Exposes Teensy-wide system status
//   • Serves as the future consolidation point for:
//       - firmware identity
//       - CPU health
//       - memory
//       - clocks
//       - laser / photodiode (later)
//       - any other system-owned facts
//
// Initial scope (Phase 1):
//   • Replace TEENSY.STATUS with SYSTEM.REPORT
//   • Payload is IDENTICAL to legacy TEENSY.STATUS
//
// Philosophy:
//   • SYSTEM is authoritative but non-invasive
//   • REPORT is snapshot-only
//   • No control commands (yet)
//
// =============================================================

#pragma once

#include "process.h"

// Register SYSTEM process with the process registry
void process_system_register(void);
