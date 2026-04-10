/*
 * ============================================================================
 *  bootloader.cpp
 * ============================================================================
 *
 *  ZPNet Stage-0 Bootloader Entry Implementation
 *
 *  FIX:
 *    Explicit USB teardown before entering HalfKay.
 *
 *  RATIONALE:
 *    HalfKay inherits hardware state. If USB is left active,
 *    flash programming can fail after firmware-initiated entry.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "imxrt.h"
#include "core_pins.h"

extern "C" void _reboot_Teensyduino_(void);

extern "C" void enter_bootloader_cleanly(void) {

  /*
   * --------------------------------------------------------------------------
   * 1. Disable all interrupts
   * --------------------------------------------------------------------------
   */
  __disable_irq();

  /*
   * --------------------------------------------------------------------------
   * 2. Stop SysTick
   * --------------------------------------------------------------------------
   */
  SYST_CSR = 0;

  /*
   * --------------------------------------------------------------------------
   * 4. Disable DMA MUX channels
   * --------------------------------------------------------------------------
   */
  DMAMUX_CHCFG0 = 0;
  DMAMUX_CHCFG1 = 0;
  DMAMUX_CHCFG2 = 0;
  DMAMUX_CHCFG3 = 0;

  /*
   * --------------------------------------------------------------------------
   * 5. USB teardown (CRITICAL)
   * --------------------------------------------------------------------------
   *
   * We must leave the USB controller and PHY in a cold state.
   * HalfKay assumes firmware has not configured USB.
   */

  // Disable USB interrupts
  USB1_USBINTR = 0;

  // Disable all endpoints
  USB1_ENDPTCTRL0 = 0;
  USB1_ENDPTCTRL1 = 0;
  USB1_ENDPTCTRL2 = 0;
  USB1_ENDPTCTRL3 = 0;
  USB1_ENDPTCTRL4 = 0;
  USB1_ENDPTCTRL5 = 0;
  USB1_ENDPTCTRL6 = 0;
  USB1_ENDPTCTRL7 = 0;

  // Disable USB controller
  USB1_USBCMD = 0;

  // Small delay to allow hardware settle (cycle-based, no SysTick)
  for (volatile int i = 0; i < 10000; i++) {
    __asm__ volatile ("nop");
  }

  // Disable USB PHY
  USBPHY1_CTRL = USBPHY_CTRL_CLKGATE;
  USBPHY1_PWD  = 0xFFFFFFFF;

  /*
   * --------------------------------------------------------------------------
   * 6. Enforce memory ordering
   * --------------------------------------------------------------------------
   */
  __asm__ volatile ("dsb");
  __asm__ volatile ("isb");

  /*
   * --------------------------------------------------------------------------
   * 7. Jump to HalfKay bootloader
   * --------------------------------------------------------------------------
   */
  _reboot_Teensyduino_();

  /*
   * --------------------------------------------------------------------------
   * 8. Absolute fallback
   * --------------------------------------------------------------------------
   */
  while (1) {}
}