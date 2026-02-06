// abstract_clock.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

class AbstractClock {
public:
    virtual ~AbstractClock() {}

    // ---- Lifecycle (ritual-level, PPS-aligned) ----
    void request_start();     // zero ns at next PPS
    void request_recover();   // preserve ns at next PPS
    void request_stop();
    void request_clear();     // clears ns immediately (policy choice)

    // Called exactly at PPS edge by orchestrator
    void anchor_at_pps(bool reset_nanoseconds);

    // ---- Synthetic time ----
    uint64_t nanoseconds() const;

    // Interpolated "now" (uses derived clock data)
    virtual uint64_t nanoseconds_now() const = 0;

    // ---- Identity / metadata ----
    virtual const char* name() const = 0;

protected:
    AbstractClock();

    // ---- Durable ledger state ----
    uint64_t ns_;   // synthetic nanoseconds (monotonic, persisted)

    // ---- Hooks for derived clocks ----
    virtual void capture_baseline_at_pps() = 0;
    virtual uint64_t interpolate_ns() const = 0;

    // ---- Helpers for derived classes ----
    void set_nanoseconds(uint64_t ns);
    void advance_nanoseconds(uint64_t delta_ns);

private:
    bool armed_start_;
    bool armed_recover_;
};
