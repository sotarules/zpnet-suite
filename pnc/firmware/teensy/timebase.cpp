#include "timebase.h"

// ============================================================================
// Lifecycle
// ============================================================================

TimeBase::TimeBase()
  : clock_count_(0) {}

void TimeBase::clear() {
  clock_count_ = 0;
}

// ============================================================================
// Internal helpers
// ============================================================================

TimeBase::Clock* TimeBase::find_clock(const char* name) {
  for (size_t i = 0; i < clock_count_; i++) {
    if (strncmp(clocks_[i].name, name, NAME_LEN) == 0) {
      return &clocks_[i];
    }
  }
  return nullptr;
}

const TimeBase::Clock* TimeBase::find_clock(const char* name) const {
  for (size_t i = 0; i < clock_count_; i++) {
    if (strncmp(clocks_[i].name, name, NAME_LEN) == 0) {
      return &clocks_[i];
    }
  }
  return nullptr;
}

// ============================================================================
// Clock management
// ============================================================================

bool TimeBase::add_clock(
  const char* name,
  uint64_t ns_at_pps,
  uint64_t cycles_at_pps,
  uint64_t ns_per_cycle_n,
  uint64_t ns_per_cycle_d
) {
  if (!name || clock_count_ >= MAX_CLOCKS) {
    return false;
  }

  Clock* c = find_clock(name);
  if (!c) {
    c = &clocks_[clock_count_++];
    strncpy(c->name, name, NAME_LEN - 1);
    c->name[NAME_LEN - 1] = '\0';
  }

  c->ns_at_pps      = ns_at_pps;
  c->cycles_at_pps  = cycles_at_pps;
  c->ns_per_cycle_n = ns_per_cycle_n;
  c->ns_per_cycle_d = ns_per_cycle_d;

  return true;
}

bool TimeBase::has_clock(const char* name) const {
  return find_clock(name) != nullptr;
}

// ============================================================================
// Core operations
// ============================================================================

uint64_t TimeBase::now(
  const char* clock_name,
  uint64_t cycle_count_now
) const {
  const Clock* c = find_clock(clock_name);
  if (!c) return 0;

  if (c->cycles_at_pps == 0) {
    return c->ns_at_pps;
  }

  uint64_t delta_cycles = cycle_count_now - c->cycles_at_pps;
  uint64_t delta_ns =
    (delta_cycles * c->ns_per_cycle_n) / c->ns_per_cycle_d;

  return c->ns_at_pps + delta_ns;
}

uint64_t TimeBase::convert(
  const char* from_clock,
  const char* to_clock,
  uint64_t from_ns
) const {
  const Clock* from = find_clock(from_clock);
  const Clock* to   = find_clock(to_clock);

  if (!from || !to) return 0;

  int64_t delta_ns =
    (int64_t)from_ns - (int64_t)from->ns_at_pps;

  if (from->ns_per_cycle_n == 0 || to->ns_per_cycle_n == 0) {
    return to->ns_at_pps;
  }

  int64_t delta_cycles =
    (delta_ns * (int64_t)from->ns_per_cycle_d) /
    (int64_t)from->ns_per_cycle_n;

  int64_t to_delta_ns =
    (delta_cycles * (int64_t)to->ns_per_cycle_n) /
    (int64_t)to->ns_per_cycle_d;

  return (uint64_t)((int64_t)to->ns_at_pps + to_delta_ns);
}

// ============================================================================
// Tau
// ============================================================================

double TimeBase::tau(const char* clock_name) const {
  if (strcmp(clock_name, "GNSS") == 0) return 1.0;

  const Clock* c  = find_clock(clock_name);
  const Clock* gn = find_clock("GNSS");

  if (!c || !gn) return 0.0;

  double num =
    (double)c->ns_per_cycle_n / (double)c->ns_per_cycle_d;
  double den =
    (double)gn->ns_per_cycle_n / (double)gn->ns_per_cycle_d;

  return num / den;
}
