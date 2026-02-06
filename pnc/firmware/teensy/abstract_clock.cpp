// abstract_clock.cpp
#include "abstract_clock.h"

AbstractClock::AbstractClock()
    : ns_(0),
      armed_start_(false),
      armed_recover_(false) {}

void AbstractClock::request_start() {
    armed_start_ = true;
    armed_recover_ = false;
}

void AbstractClock::request_recover() {
    armed_recover_ = true;
    armed_start_ = false;
}

void AbstractClock::request_stop() {
    armed_start_ = false;
    armed_recover_ = false;
}

void AbstractClock::request_clear() {
    ns_ = 0;
}

void AbstractClock::anchor_at_pps(bool reset_nanoseconds) {
    // Capture new baseline for the *upcoming* second
    capture_baseline_at_pps();

    if (reset_nanoseconds) {
        ns_ = 0;
    }

    armed_start_ = false;
    armed_recover_ = false;
}

uint64_t AbstractClock::nanoseconds() const {
    return ns_;
}

void AbstractClock::set_nanoseconds(uint64_t ns) {
    ns_ = ns;
}

void AbstractClock::advance_nanoseconds(uint64_t delta_ns) {
    ns_ += delta_ns;
}

uint64_t AbstractClock::nanoseconds_now() const {
    return ns_ + interpolate_ns();
}
