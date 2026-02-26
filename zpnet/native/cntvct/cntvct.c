/*
 * cntvct.c — Read ARM Generic Timer (CNTVCT_EL0)
 *
 * Single function: read_cntvct_el0()
 *   Returns the raw 64-bit counter value.
 *
 * Used by PITIMER in PPS mode where libppscapture is bypassed.
 *
 * Build:
 *   gcc -shared -fPIC -O2 -o libcntvct.so cntvct.c
 */

#include <stdint.h>

uint64_t read_cntvct_el0(void)
{
    uint64_t val;
    __asm__ __volatile__(
        "isb\n\t"
        "mrs %0, cntvct_el0"
        : "=r"(val)
        :
        : "memory"
    );
    return val;
}