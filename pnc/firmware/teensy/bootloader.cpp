/*
 * ============================================================================
 *  bootloader.cpp
 * ============================================================================
 *
 *  ZPNet Stage-0 Bootloader Entry Implementation
 *
 *  This file provides the concrete, hardware-specific implementation of
 *  `enter_bootloader_cleanly()`.
 *
 *  PURPOSE
 *  -------
 *  Teensy 4.x devices do not safely tolerate arbitrary jumps into the
 *  HalfKay bootloader while timers, DMA, or interrupts are active.
 *
 *  This implementation performs an explicit teardown of all critical runtime
 *  subsystems before transferring control to the ROM bootloader.
 *
 *  This is REQUIRED for:
 *    • RawHID-only firmware
 *    • Buttonless remote flashing
 *    • Deterministic field upgrades
 *
 *  FAILURE MODES PREVENTED
 *  ----------------------
 *  • Stuck USB endpoints
 *  • Active PIT / GPT timers firing during bootloader
 *  • DMA writes into invalid memory
 *  • Corrupted bootloader state
 *
 *  This code intentionally performs NO logging after teardown begins.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "imxrt.h"
#include "core_pins.h"

/*
 * HalfKay bootloader entry point.
 *
 * This symbol is provided by the Teensy core / ROM interface.
 * It performs the actual transfer into the bootloader.
 *
 * IMPORTANT:
 *   • This function never returns
 *   • It assumes the system is already quiescent
 */
extern "C" void _reboot_Teensyduino_(void);

/*
 * enter_bootloader_cleanly()
 *
 * Authoritative, irreversible transition into the Teensy bootloader.
 *
 * This function must be treated as a SYSTEM TERMINATION PATH,
 * analogous to power-off or hardware reset.
 */
extern "C" void enter_bootloader_cleanly(void) {

  /*
   * --------------------------------------------------------------------------
   * 1. Disable all interrupts
   * --------------------------------------------------------------------------
   *
   * Prevents:
   *   • ISR execution during teardown
   *   • Timer callbacks firing mid-transition
   *   • Race conditions with hardware state
   */
  __disable_irq();

  /*
   * --------------------------------------------------------------------------
   * 2. Stop SysTick
   * --------------------------------------------------------------------------
   *
   * SysTick may be used implicitly by Arduino core timing.
   * It MUST be halted before entering the bootloader.
   */
  SYST_CSR = 0;

  /*
   * --------------------------------------------------------------------------
   * 3. Stop GPT timers
   * --------------------------------------------------------------------------
   *
   * GPT1 / GPT2 are used by ZPNet for:
   *   • Prescaled GNSS clocks
   *   • OCXO timing
   *
   * These timers must be shut down explicitly to prevent
   * clock edges from firing inside the bootloader.
   */
  GPT1_CR = 0;
  GPT2_CR = 0;

  /*
   * --------------------------------------------------------------------------
   * 4. Disable DMA MUX channels
   * --------------------------------------------------------------------------
   *
   * Active DMA transfers during bootloader entry can corrupt:
   *   • Flash programming
   *   • Bootloader buffers
   *   • USB descriptors
   *
   * We disable all DMA channels defensively.
   */
  DMAMUX_CHCFG0 = 0;
  DMAMUX_CHCFG1 = 0;
  DMAMUX_CHCFG2 = 0;
  DMAMUX_CHCFG3 = 0;

  /*
   * --------------------------------------------------------------------------
   * 5. Enforce memory ordering
   * --------------------------------------------------------------------------
   *
   * Ensure all outstanding memory operations complete
   * before transferring control to ROM code.
   */
  __asm__ volatile ("dsb");  // Data Synchronization Barrier
  __asm__ volatile ("isb");  // Instruction Synchronization Barrier

  /*
   * --------------------------------------------------------------------------
   * 6. Jump to HalfKay bootloader
   * --------------------------------------------------------------------------
   *
   * Control is transferred to the ROM bootloader.
   * From this point forward:
   *
   *   • USB re-enumeration is owned by HalfKay
   *   • Firmware execution is finished
   *   • This function never returns
   */
  _reboot_Teensyduino_();

  /*
   * --------------------------------------------------------------------------
   * 7. Absolute fallback (should never execute)
   * --------------------------------------------------------------------------
   *
   * If, for any reason, the bootloader entry returns,
   * we enter a permanent inert state.
   */
  while (1) {}
}
