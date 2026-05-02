// ============================================================================
// process_epoch.h
// ============================================================================
//
// EPOCH is a deliberately small zeroing coordinator.
//
// ZERO means: request that the next canonical PPS/VCLOCK selected edge become
// logical zero.  DWT cannot be physically reset, so the selected DWT coordinate
// is installed as the DWT64 epoch origin.  VCLOCK, OCXO1, and OCXO2 are then
// explicitly zeroed by writing their 16-bit hardware counters, 32-bit synthetic
// counters, and 64-bit nanosecond ledgers to the DWT-compensated elapsed value
// appropriate at each individual write.
// ============================================================================

#pragma once

#include "epoch.h"
#include "process_interrupt.h"

#include <stdint.h>
#include <stdbool.h>

void process_epoch_init(void);
void process_epoch_register(void);

epoch_request_result_t process_epoch_request_zero(epoch_reason_t reason);
bool process_epoch_on_pps_vclock_snapshot(const pps_edge_snapshot_t& snap);

bool process_epoch_pending(void);
bool process_epoch_finalized(void);
epoch_state_t process_epoch_state(void);
uint32_t process_epoch_last_request_id(void);
uint32_t process_epoch_current_sequence(void);
epoch_fact_t process_epoch_current_fact(void);

const char* epoch_reason_str(epoch_reason_t reason);
const char* epoch_state_str(epoch_state_t state);
