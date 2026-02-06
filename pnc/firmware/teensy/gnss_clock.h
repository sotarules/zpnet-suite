// =============================================================
// FILE: gnss_clock.h
// =============================================================
//
// ZPNet — GNSS Pulse Clock
//
// Responsibilities:
//   • Track GNSS-derived prescaled pulse ticks (e.g. 10 kHz)
//   • Capture DWT baselines at each pulse edge
//   • Provide synthetic nanosecond interpolation
//
// This clock:
//   • Is pulse-driven (not free-running)
//   • Uses DWT as an interpolation ruler
//   • Treats pulses as coarse position, DWT as fine offset
//   • Relies on AbstractClock for nanosecond continuity
//
// =============================================================

#pragma once

#include <stdint.h>
#include "abstract_clock.h"

class DwtClock;

class GnssClock : public AbstractClock {
public:
    // ticks_per_second = 10'000 for 10 kHz GNSS prescale
    GnssClock(DwtClock& dwt,
              uint32_t ticks_per_second);

    const char* name() const override;

    // Called from GNSS prescaled-edge ISR
    void on_gnss_tick();

    // Interpolated synthetic nanoseconds "now"
    uint64_t nanoseconds_now() const override;

protected:
    // PPS-aligned anchor hook
    void capture_baseline_at_pps() override;

    // Interpolation within current tick interval
    uint64_t interpolate_ns() const override;

private:
    // ---- External ruler ----
    DwtClock& dwt_;

    // ---- Coarse pulse ledger ----
    volatile uint64_t tick_count_;

    // ---- Ephemeral interpolation baseline ----
    volatile uint32_t dwt_baseline_;

    // ---- Clock parameters ----
    const uint32_t ticks_per_second_;
    const uint64_t ns_per_tick_;
};
