// =============================================================
// FILE: dwt_clock.h
// =============================================================
//
// ZPNet — DWT Cycle Clock
//
// Responsibilities:
//   • Enable and manage ARM DWT cycle counter
//   • Maintain wrap-safe 64-bit cycle ledger
//   • Capture PPS-aligned baselines (ephemeral)
//   • Provide synthetic nanosecond interpolation
//
// This clock:
//   • Cannot be set
//   • Cannot recover cycle state after reboot
//   • Relies on AbstractClock for nanosecond continuity
//
// =============================================================

#pragma once

#include <stdint.h>
#include "abstract_clock.h"

class DwtClock : public AbstractClock {
public:
    // cycles_per_second ≈ 600 MHz on Teensy 4.1
    explicit DwtClock(uint32_t cycles_per_second);

    const char* name() const override;

    // Interpolated synthetic nanoseconds "now"
    uint64_t nanoseconds_now() const override;

    // Raw cycle access (wrap-safe, monotonic)
    uint64_t cycles_now();

protected:
    // PPS-aligned anchor hook
    void capture_baseline_at_pps() override;

    // Interpolation within the current second
    uint64_t interpolate_ns() const override;

private:
    // ---- Hardware enable ----
    void enable_dwt();

    // ---- Raw DWT access ----
    uint32_t read_dwt_cycle_counter() const;

    // ---- Durable cycle ledger ----
    uint64_t cycles_64_;
    uint32_t cycles_last_;

    // ---- Ephemeral interpolation baseline ----
    uint32_t baseline_cycles_;

    // ---- Clock parameters ----
    const uint32_t cycles_per_second_;
};
