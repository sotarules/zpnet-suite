/*
 * systimer.c — ARM Generic Timer (CNTVCT_EL0) userspace access
 *
 * Provides two functions for reading the ARM architected system
 * timer from EL0 (userspace) on AArch64.  No kernel module,
 * no device file, no privileges required.
 *
 * Build:
 *   make              (from zpnet/native/)
 *   gcc -shared -fPIC -O2 -o libsystimer.so systimer.c
 *
 * Usage from Python:
 *   from zpnet.native import systimer
 *   ticks = systimer.read()
 *   freq  = systimer.frequency()
 *
 * ISB (instruction synchronisation barrier):
 *   Placed before MRS to ensure the read is not speculated
 *   or reordered past earlier instructions.  This gives a
 *   well-defined capture point — critical when correlating
 *   the timer read with a PPS edge.
 *
 * See: ARM Architecture Reference Manual, D13.2 "Generic Timer"
 */

#include "systimer.h"

uint64_t systimer_read(void)
{
    uint64_t val;
    __asm__ __volatile__(
        "isb\n\t"
        "mrs %0, cntvct_el0"
        : "=r" (val)
        :
        : "memory"
    );
    return val;
}

uint64_t systimer_frequency(void)
{
    uint64_t val;
    __asm__ __volatile__(
        "mrs %0, cntfrq_el0"
        : "=r" (val)
    );
    return val;
}