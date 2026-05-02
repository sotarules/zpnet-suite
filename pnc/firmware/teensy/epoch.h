// ============================================================================
// epoch.h
// ============================================================================
//
// Shared representational structures for ZPNet epoch/zero custody.
//
// Doctrine:
//
//   ZERO does not mean "write literal zero into every counter."
//
//   ZERO defines one logical epoch event.  Every writable clock is then
//   initialized to the value it must have at its actual installation instant
//   so that back-projecting to the epoch event yields zero.
//
//   DWT is the reference ruler and is not reset after boot.  It receives an
//   epoch coordinate.  Writable synthetic clocks receive epoch-consistent
//   present values.
//
// This header is intentionally free of process framework dependencies.
// ============================================================================

#pragma once

#include <stdint.h>
#include <stdbool.h>

enum class epoch_reason_t : uint8_t {
  UNKNOWN = 0,
  STARTUP,
  EPOCH_ZERO_COMMAND,
  CLOCKS_ZERO_COMMAND,
  CLOCKS_START,
  CLOCKS_RECOVER,
  INTERNAL,
};

enum class epoch_state_t : uint8_t {
  IDLE = 0,
  REQUESTED,
  INTERRUPT_ARMED,
  FACT_AUTHORED,
  INSTALLING,
  FINALIZED,
  FAULT,
};

enum class epoch_clock_id_t : uint8_t {
  NONE = 0,
  DWT,
  VCLOCK,
  OCXO1,
  OCXO2,
  TIME,
  TIMEPOP,
  CLOCKS_ALPHA,
  CLOCKS_BETA,
};

struct epoch_request_result_t {
  bool          accepted = false;
  uint32_t      request_id = 0;
  epoch_state_t state = epoch_state_t::IDLE;
  const char*   message = "";
};

struct epoch_fact_t {
  bool          valid = false;
  uint32_t      request_id = 0;
  uint32_t      epoch_sequence = 0;
  epoch_reason_t reason = epoch_reason_t::UNKNOWN;

  uint64_t      epoch_ns = 0;

  uint32_t      epoch_dwt = 0;
  uint32_t      dwt_cycles_per_second_seed = 0;

  uint32_t      vclock_counter32_at_epoch = 0;
  uint16_t      vclock_low16_at_epoch = 0;

  uint32_t      pps_sequence = 0;
  uint32_t      physical_pps_dwt_at_edge = 0;
  uint32_t      physical_pps_counter32_at_read = 0;
  uint16_t      physical_pps_low16_at_read = 0;
};

struct epoch_clock_install_t {
  bool              attempted = false;
  bool              write_ok = false;
  bool              coherent = false;
  epoch_clock_id_t  clock_id = epoch_clock_id_t::NONE;

  uint32_t          install_dwt = 0;
  uint32_t          elapsed_dwt_cycles = 0;
  uint64_t          elapsed_ns_at_install = 0;

  uint32_t          installed_counter32 = 0;
  uint16_t          installed_low16 = 0;

  uint32_t          back_projected_epoch_counter32 = 0;
  int32_t           back_projection_error_ticks = 0;

  uint32_t          phase_ns_within_tick = 0;
  uint32_t          distance_to_previous_boundary_ns = 0;
  uint32_t          distance_to_next_boundary_ns = 0;
};

struct epoch_clock_snapshot_t {
  bool              available = false;
  bool              actual_counter32_available = false;
  bool              actual_low16_available = false;
  bool              coherent = false;
  epoch_clock_id_t  clock_id = epoch_clock_id_t::NONE;

  uint32_t          expected_counter32 = 0;
  uint32_t          actual_counter32 = 0;
  int32_t           counter32_delta_ticks = 0;

  uint16_t          expected_low16 = 0;
  uint16_t          actual_low16 = 0;
  int32_t           low16_delta_ticks = 0;
};

struct epoch_audit_t {
  bool          valid = false;
  uint32_t      request_id = 0;
  uint32_t      epoch_sequence = 0;
  epoch_reason_t reason = epoch_reason_t::UNKNOWN;

  uint32_t      request_dwt = 0;
  uint32_t      interrupt_arm_dwt = 0;
  uint32_t      fact_authored_dwt = 0;
  uint32_t      install_begin_dwt = 0;
  uint32_t      install_end_dwt = 0;
  uint32_t      finalize_dwt = 0;

  uint32_t      pps_isr_entry_dwt_raw = 0;
  uint32_t      pps_event_dwt = 0;
  uint32_t      selected_edge_dwt = 0;
  uint32_t      selected_vclock_counter32 = 0;
  uint16_t      selected_vclock_low16 = 0;

  uint32_t      install_count = 0;
  uint32_t      coherent_install_count = 0;
  uint32_t      max_install_elapsed_ns = 0;
  uint32_t      max_abs_back_projection_error_ticks = 0;

  uint32_t      near_boundary_install_count = 0;
  uint32_t      install_elapsed_over_budget_count = 0;
  uint32_t      back_projection_mismatch_count = 0;
  uint32_t      clock_write_failure_count = 0;
  uint32_t      epoch_generation_mismatch_count = 0;

  epoch_clock_install_t dwt;
  epoch_clock_install_t vclock;
  epoch_clock_install_t ocxo1;
  epoch_clock_install_t ocxo2;
};

struct epoch_history_entry_t {
  bool          valid = false;
  uint32_t      request_id = 0;
  uint32_t      epoch_sequence = 0;
  epoch_reason_t reason = epoch_reason_t::UNKNOWN;
  uint32_t      epoch_dwt = 0;
  uint32_t      vclock_counter32_at_epoch = 0;
  uint32_t      install_count = 0;
  uint32_t      coherent_install_count = 0;
  uint32_t      back_projection_mismatch_count = 0;
  uint32_t      clock_write_failure_count = 0;
  bool          finalized = false;
};
