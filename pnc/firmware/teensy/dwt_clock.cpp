// =============================================================
// FILE: dwt_clock.cpp
// =============================================================
//
// ZPNet — DWT Cycle Clock implementation
//
// =============================================================

#include "dwt_clock.h"

#include <Arduino.h>
#include "imxrt.h"

// -------------------------------------------------------------
// DWT Registers (ARM CoreSight)
// -------------------------------------------------------------
#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)

#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// -------------------------------------------------------------
// Constructor
// -------------------------------------------------------------
DwtClock::DwtClock(uint32_t cycles_per_second)
    : cycles_64_(0),
      cycles_last_(0),
      baseline_cycles_(0),
      cycles_per_second_(cycles_per_second)
{
    enable_dwt();
}

// -------------------------------------------------------------
// Identity
// -------------------------------------------------------------
const char* DwtClock::name() const {
    return "DWT";
}

// -------------------------------------------------------------
// Hardware enable
// -------------------------------------------------------------
void DwtClock::enable_dwt() {
    DEMCR |= DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;

    cycles_last_ = DWT_CYCCNT;
}

// -------------------------------------------------------------
// Raw DWT read
// -------------------------------------------------------------
uint32_t DwtClock::read_dwt_cycle_counter() const {
    return DWT_CYCCNT;
}

// -------------------------------------------------------------
// Wrap-safe cycle ledger
// -------------------------------------------------------------
uint64_t DwtClock::cycles_now() {
    uint32_t now = read_dwt_cycle_counter();

    // unsigned subtraction handles wrap correctly
    cycles_64_ += static_cast<uint32_t>(now - cycles_last_);
    cycles_last_ = now;

    return cycles_64_;
}

// -------------------------------------------------------------
// PPS-aligned baseline capture
// -------------------------------------------------------------
void DwtClock::capture_baseline_at_pps() {
    baseline_cycles_ = read_dwt_cycle_counter();
}

// -------------------------------------------------------------
// Interpolation (ephemeral, per-interval only)
// -------------------------------------------------------------
uint64_t DwtClock::interpolate_ns() const {
    uint32_t now_cycles = read_dwt_cycle_counter();
    uint32_t delta_cycles = static_cast<uint32_t>(now_cycles - baseline_cycles_);

    // fraction of second elapsed
    // NOTE: double is acceptable here for clarity;
    //       integer-only versions can come later.
    double fraction =
        static_cast<double>(delta_cycles) /
        static_cast<double>(cycles_per_second_);

    return static_cast<uint64_t>(fraction * 1e9);
}

// -------------------------------------------------------------
// Synthetic nanoseconds "now"
// -------------------------------------------------------------
uint64_t DwtClock::nanoseconds_now() const {
    return AbstractClock::nanoseconds_now();
}
