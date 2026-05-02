// ============================================================================
// process_epoch.h
// ============================================================================
//
// EPOCH process public interface.
//
// process_epoch is the controller/custodian for logical epoch installation.
// It owns requests, state transitions, forensic audit, and JSON reports.
//
// It does not own the hardware.  process_interrupt authors physical timing
// facts.  process_epoch consumes those facts and coordinates the consumers.
// ============================================================================

#pragma once

#include "epoch.h"
#include "process_interrupt.h"

#include <stdint.h>
#include <stdbool.h>

void process_epoch_init(void);
void process_epoch_register(void);

// Typed firmware-internal API.  Prefer this over routing through the command
// system when another process delegates ZERO/START/RECOVER semantics.
epoch_request_result_t process_epoch_request_zero(epoch_reason_t reason);

// Called by the PPS/VCLOCK consumer path when a stable canonical selected-edge
// snapshot is available.  During a pending epoch request this installs and
// finalizes the epoch.  Outside a pending request it returns false.
bool process_epoch_on_pps_vclock_snapshot(const pps_edge_snapshot_t& snap);

// Status/accessors.
epoch_state_t process_epoch_state(void);
bool process_epoch_pending(void);
bool process_epoch_finalized(void);
uint32_t process_epoch_current_sequence(void);
uint32_t process_epoch_last_request_id(void);
epoch_fact_t process_epoch_current_fact(void);
epoch_audit_t process_epoch_current_audit(void);

// DWT64 logical cycle-clock accessors. DWT64 is zeroed by epoch-origin subtraction,
// not by writing the hardware DWT_CYCCNT register. process_epoch deliberately
// does not expose a logical nanosecond DWT clock.
uint64_t process_epoch_dwt64_logical_cycles_now(void);
uint64_t process_epoch_dwt64_raw_epoch_cycles(void);
bool process_epoch_dwt64_epoch_valid(void);

// String helpers for reports and diagnostics.
const char* epoch_reason_str(epoch_reason_t reason);
const char* epoch_state_str(epoch_state_t state);
const char* epoch_clock_id_str(epoch_clock_id_t clock_id);
