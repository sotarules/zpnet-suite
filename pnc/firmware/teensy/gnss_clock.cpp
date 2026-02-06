// =============================================================
// FILE: gnss_clock.cpp
// =============================================================
//
// ZPNet — GNSS Pulse Clock implementation
//
// =============================================================

#include "gnss_clock.h"
#include "dwt_clock.h"

// -------------------------------------------------------------
// Constructor
// -------------------------------------------------------------
GnssClock::GnssClock(DwtClock& dwt,
                     uint32_t ticks_per_second)
    : dwt_(dwt),
      tick_count_(0),
      dwt_baseline_(0),
      ticks_per_second_(ticks_per_second),
      ns_per_tick_(1000000000ull / ticks_per_second)
{
}

// -------------------------------------------------------------
// Identity
// -------------------------------------------------------------
const char* GnssClock::name() const {
    return "GNSS";
}

// -------------------------------------------------------------
// GNSS prescaled edge (ISR-facing)
// -------------------------------------------------------------
void GnssClock::on_gnss_tick() {
    tick_count_++;
    dwt_baseline_ = static_cast<uint32_t>(dwt_.cycles_now());
}

// -------------------------------------------------------------
// PPS-aligned anchor
//
// NOTE:
//   For GNSS, PPS alignment and prescaled ticks may coincide
//   or be distinct depending on hardware wiring.
//   We treat PPS anchoring as "capture baseline, do not advance".
// -------------------------------------------------------------
void GnssClock::capture_baseline_at_pps() {
    dwt_baseline_ = static_cast<uint32_t>(dwt_.cycles_now());
}

// -------------------------------------------------------------
// Interpolation (within current tick interval)
// -------------------------------------------------------------
uint64_t GnssClock::interpolate_ns() const {
    uint32_t now_cycles =
        static_cast<uint32_t>(dwt_.cycles_now());

    uint32_t delta_cycles =
        static_cast<uint32_t>(now_cycles - dwt_baseline_);

    // Convert cycles → nanoseconds using DWT's tempo
    // cycles * (5 / 3) matches your existing 600 MHz → ns math
    uint64_t interp_ns = (delta_cycles * 5ull) / 3ull;

    // Clamp so we never exceed one tick interval
    if (interp_ns > ns_per_tick_) {
        interp_ns = ns_per_tick_;
    }

    return interp_ns;
}

// -------------------------------------------------------------
// Synthetic nanoseconds "now"
// -------------------------------------------------------------
uint64_t GnssClock::nanoseconds_now() const {
    uint64_t base_ns = tick_count_ * ns_per_tick_;
    return base_ns + interpolate_ns();
}
