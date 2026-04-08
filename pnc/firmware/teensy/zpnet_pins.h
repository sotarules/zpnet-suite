#pragma once
/*
===============================================================================
 ZPNet — Pin, Rail, and Bus Assignment Ledger
===============================================================================

 STATUS: AUTHORITATIVE HUMAN-READABLE SOURCE

 This file is intentionally NON-OPERATIONAL.
 It contains no constants, no logic, and no compile-time meaning.

 It exists to:
   • Prevent wiring ambiguity
   • Capture color and rail semantics
   • Serve as the precursor to formal pin bindings

===============================================================================
*/

/*=============================================================================
 (1) RAIL ASSIGNMENTS — 10-RAIL BACKPLANE
-------------------------------------------------------------------------------

Rail #    Color            Signal Name    Meaning / Notes
---------------------------------------------------------------------------
1         Black            GND             System ground (battery return)
2         White            +5V             Primary +5 V supply
3         Pink / Gray      RESET           Reserved / reset reference
4         Purple           +3V3            Primary +3.3 V logic rail
5         Blue             SDA1            Primary I2C data (SMBUS1)
6         Green            SDA2            Secondary I2C data (SMBUS2)
7         Yellow           SCL1            Primary I2C clock (SMBUS1)
8         Orange           SCL2            Secondary I2C clock (SMBUS2)
9         Brown            AUX1            Reserved auxiliary rail
10        Red              AUX2            Reserved auxiliary rail
=============================================================================*/


/*=============================================================================
 RASPBERRY PI PIN ASSIGNMENTS (UNCHANGED)
=============================================================================*/


/*=============================================================================
 (2) TEENSY 4.1 PIN ASSIGNMENTS — v18 QTimer OCXO Migration
-------------------------------------------------------------------------------

v18 CHANGES (from v17):

  • OCXO clocks migrated from GPT to QTimer3.
  • GNSS remains on QTimer1.
  • OCXO1 → QTimer3 CH2 (pin 14, GPIO_AD_B1_02, ALT1, PCS(2))
  • OCXO2 → QTimer3 CH3 (pin 15, GPIO_AD_B1_03, ALT1, PCS(3))
  • GPT1/GPT2 no longer used for OCXO timing.
  • Pin 13 (LED_BUILTIN) preserved for fault Morse annunciator.
  • Pin 15 repurposed from photodiode ADC to OCXO2 timing.
  • Photodiode analog input needs reassignment to another pin.

  CRITICAL PIN NOTES:
    Pin 11 = QTIMER1_TIMER2 (CH2) — TimePop compare scheduler.
    DO NOT use pin 11 for external clock input.

-------------------------------------------------------------------------------

Teensy Pin    Wire Color    Signal Name        Source / Destination                 Notes
------------------------------------------------------------------------------------------
VIN           White         VIN_5V5            INA260 (5.5 V rail)                  Dedicated 5V+ CPU power
GND           Black         GND                Battery branching ground             Direct return to battery
1             Twisted Pair  GNSS_PPS_IN        GF-8802 PPS                          1 Hz absolute time reference
4             Green         GNSS_LOCK_IN       GF-8802 LOCK                         Lock status signal
10            Twisted Pair  GNSS_10MHZ_IN      GF-8802 VCLOCK                       10 MHz reference (QTimer1 ch0+ch1 counter)

14            Twisted Pair  OCXO1_10MHZ_IN     OCXO1                                QTimer3 CH2 external clock
15            Twisted Pair  OCXO2_10MHZ_IN     OCXO2                                QTimer3 CH3 external clock

23            Green         DAC_VREF_OUT       AD5693R VREF (both)                  Software-controlled reference voltage
32            Orange        GNSS_PPS_RELAY     GPIO relay to Pi                     Splits to GPIO18 and GPIO25
34            Orange        PHOTODIODE_INT     Photodiode interrupt                 Digital interrupt (TBD: analog may move)
20            White         LASER_PD_PLUS      Laser diode PD+                      Laser on/off monitor
30            Green         LASER_EN           EV5491-C-00A EN pin                  Laser driver enable
18            Blue          SDA1               Rail bus SDA1                        Primary I2C data
19            Yellow        SCL1               Rail bus SCL1                        Primary I2C clock

-------------------------------------------------------------------------------
Unassigned pins (available for future use):
  5, 6, 7, 8, 9, 11, 12, 22, 24, 25

  NOTE: Pin 11 is physically available but electrically reserved —
  it is QTimer1 CH2, owned by TimePop as the compare scheduler.
  Using it for anything that calls portConfigRegister(11)=1 will
  destroy TimePop's compare channel.

  Photodiode analog input (previously pin 15) needs reassignment
  to one of the available analog-capable pins.

-------------------------------------------------------------------------------
Timer hardware binding summary:

  Pin 14  →  QTimer3 CH2 external clock  (OCXO1 10 MHz, single-edge, 16-bit free-run)
  Pin 15  →  QTimer3 CH3 external clock  (OCXO2 10 MHz, single-edge, 16-bit free-run)
  Pin 10  →  QTimer1 ch0+ch1 counter     (GNSS 10 MHz input, cascaded 32-bit)
  Pin 10  →  QTimer1 ch2 compare         (GNSS TimePop compare / interrupt path)
  Pin 23  →  DAC1 analog output          (VREF for both AD5693R DACs)

Notes:
• QTimer1 remains the GNSS sovereign clock domain (TimePop).
• QTimer3 CH2+CH3 host both OCXO clocks on a shared NVIC vector.
• QTimer2 is completely unused / available.
• All timing paths use direct IOMUX ALT1 routing (no XBAR).
• Pin 13 (LED_BUILTIN) is preserved for fault Morse annunciator.
• STP shield drains at source end.

=============================================================================*/


/*=============================================================================
 (3) TIMING SIGNAL WIRING SUMMARY
-------------------------------------------------------------------------------

Signal Name          Source          Destination       Frequency    Timer HW                 Shield Drain
----------------------------------------------------------------------------------------------------------
GNSS_PPS_IN          GF-8802 P17     Teensy pin 1      1 Hz         GPIO IRQ                 GF-8802 end
GNSS_10MHZ_IN        GF-8802 P11     Teensy pin 10     10 MHz       QTimer1 ch0+ch1 (+ ch2)  GF-8802 end
OCXO1_10MHZ_IN       OCXO1           Teensy pin 14     10 MHz       QTimer3 CH2              OCXO1 end
OCXO2_10MHZ_IN       OCXO2           Teensy pin 15     10 MHz       QTimer3 CH3              OCXO2 end
GNSS_PPS_RELAY       Teensy pin 32   Pi GPIO18/25      1 Hz         —                        Teensy end

=============================================================================*/


/*=============================================================================
 (9) OCXO1 — AOCJY1-A
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 14 (QTimer3 CH2) via STP
  CTL input      ←  AD5693R 0x4E VOUT (green wire, direct)
  VREF source    ←  Teensy pin 23 (DAC1, software-controlled)
  Power          →  Dedicated 5V domain

QTimer3 CH2, 16-bit free-running, external clock count.
Shield drain at OCXO1 end.

Pin-to-timer: GPIO_AD_B1_02 → ALT1 = QTIMER3_TIMER2 → PCS(2)
Register setup mirrors QTimer1 CH0 (proven TimePop pattern).

History: GPT1/pin25 → QTimer2/pin13 (LED conflict) → QTimer4/pin6
(XBAR, no direct access) → GPT1/pin25 → QTimer3/pin11 (TimePop conflict!)
→ QTimer3 CH2/pin14 (final).

=============================================================================*/


/*=============================================================================
 (10) OCXO2 — AOCJY1-A (second unit)
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 15 (QTimer3 CH3) via STP
  CTL input      ←  AD5693R 0x4C VOUT (green wire, direct)
  VREF source    ←  Teensy pin 23 (DAC1, software-controlled)
  Power          →  Dedicated 5V domain

QTimer3 CH3, 16-bit free-running, external clock count.
Symmetric with OCXO1. Shield drain at OCXO2 end.

Pin-to-timer: GPIO_AD_B1_03 → ALT1 = QTIMER3_TIMER3 → PCS(3)

History: QTimer1/pin10 → GPT2/pin14 → QTimer2/pin13 (LED conflict)
→ QTimer3 CH3/pin15 (final).

=============================================================================*/