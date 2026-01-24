#pragma once

/*
 * ============================================================================
 *  bootloader.h
 * ============================================================================
 *
 *  ZPNet Stage-0 Bootloader Entry Interface
 *
 *  This header declares a single, authoritative function that transfers
 *  execution from the running firmware into the Teensy HalfKay bootloader.
 *
 *  DESIGN INTENT
 *  -------------
 *  • This is NOT an application feature.
 *  • This is NOT a process.
 *  • This is NOT recoverable.
 *
 *  This function represents a TERMINAL SYSTEM TRANSITION.
 *
 *  Once invoked:
 *    • All interrupts are disabled
 *    • All timing infrastructure is halted
 *    • USB runtime state is abandoned
 *    • Control is transferred to the ROM bootloader
 *    • This function NEVER RETURNS
 *
 *  LINKAGE & VISIBILITY
 *  --------------------
 *  The function is declared with `extern "C"` to ensure:
 *
 *    • No C++ name mangling
 *    • Stable symbol name across translation units
 *    • Compatibility with low-level bootloader symbols
 *
 *  Any module that includes this header may request entry into the bootloader,
 *  but ONLY system-level code should ever invoke it.
 *
 *  SAFETY CONTRACT
 *  ---------------
 *  • Safe to call from scheduled (non-ISR) context
 *  • Must NOT be called from an interrupt
 *  • Must NOT rely on any code executing afterward
 *
 *  This header intentionally exposes no other functionality.
 *
 * ============================================================================
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * enter_bootloader_cleanly()
 *
 * Perform a clean, deterministic transition into the Teensy HalfKay bootloader.
 *
 * This function:
 *   • Disables all interrupts
 *   • Halts all active timers
 *   • Prevents DMA side effects
 *   • Enforces memory ordering
 *   • Jumps directly to the bootloader entry point
 *
 * This function does not return.
 */
void enter_bootloader_cleanly(void);

#ifdef __cplusplus
}
#endif
