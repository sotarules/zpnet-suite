/*
 * systimer.h — ARM Generic Timer (CNTVCT_EL0) userspace access
 *
 * The ARM architected system timer is a dedicated 64-bit hardware
 * counter that runs at a fixed frequency independent of CPU clock
 * scaling, sleep states, or thermal throttling.
 *
 * On Raspberry Pi 4 and Pi 5 the counter runs at 54 MHz
 * (18.519 ns per tick).  The actual frequency is read from
 * CNTFRQ_EL0 at runtime — never hardcoded.
 *
 * Reading CNTVCT_EL0 is a single MRS instruction with no syscall,
 * no kernel transition, and no vDSO interpolation — making it
 * significantly less jittery than clock_gettime(CLOCK_MONOTONIC).
 *
 * These functions are designed to be called from Python via ctypes
 * through the companion libsystimer.so shared library.
 */

#ifndef ZPNET_SYSTIMER_H
#define ZPNET_SYSTIMER_H

#include <stdint.h>

/*
 * Read the virtual counter (CNTVCT_EL0).
 *
 * Returns a monotonically increasing 64-bit tick count.
 * At 54 MHz the counter wraps after ~10,800 years.
 *
 * This is the primary capture function — call it as close
 * to the PPS edge as possible for best precision.
 */
uint64_t systimer_read(void);

/*
 * Read the counter frequency (CNTFRQ_EL0).
 *
 * Returns ticks per second (e.g. 54000000 on Pi 4/5).
 * This value is set by firmware at boot and never changes.
 * Read once at init and cache the result.
 */
uint64_t systimer_frequency(void);

#endif /* ZPNET_SYSTIMER_H */