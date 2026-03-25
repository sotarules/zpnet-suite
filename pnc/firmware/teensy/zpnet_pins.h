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
 RASPBERRY PI PIN ASSIGNMENTS
-------------------------------------------------------------------------------

Pi Label / Pin    Wire Color    Signal Name        Connected To / Rail           Notes
---------------------------------------------------------------------------------------
5V                White         PI_5V              INA260 0x40 (Bus 2)            Dedicated Pi 5V power domain
GND               Black         GND                Battery branching ground       Shared system ground
SDA               Blue          SDA1               Rail SDA1 (Blue)               Primary I2C data (Bus 1)
SCL               Yellow        SCL1               Rail SCL1 (Yellow)             Primary I2C clock (Bus 1)
TXD               Yellow        PI_TXD             GF-8802 RX (Pin 12)            Pi → GNSS serial
RXD               Blue          PI_RXD             GF-8802 TX (Pin 13)            GNSS → Pi serial
IO18 (GPIO18)     Orange        GNSS_PPS_RELAY     GF-8802 PPS via Teensy         GNSS PPS relay
IO23 (GPIO23)     Green         SDA2               Rail SDA2 (Green)              Secondary I2C data (Bus 2)
IO24 (GPIO24)     Orange        SCL2               Rail SCL2 (Orange)             Secondary I2C clock (Bus 2)
=============================================================================*/


/*=============================================================================
 (2) TEENSY 4.1 PIN ASSIGNMENTS — v17 Software-Controlled DAC Reference
-------------------------------------------------------------------------------

v17 CHANGES (from v16):
  • Pin 23 (DAC1) assigned as VREF source for both AD5693R DACs.
    The Teensy's onboard 12-bit DAC generates the reference voltage,
    allowing the servo control window to be tuned in software.
    Both AD5693R VREF pins are wired to pin 23 (one wire each,
    sharing the same terminal).
  • AD5693R internal reference disabled; external reference from
    pin 23 used instead.  At 1× gain, the DAC output spans
    0 V to VREF, where VREF is software-controlled.
  • Pin 23 removed from unassigned pool.

v16 CHANGES (from v15):
  • OCXO CTL pins 22 and 11 retired.  OCXO frequency control is now
    performed by two AD5693R 16-bit I2C DACs on Bus 1 (0x4C, 0x4E).
    The DAC VOUT pins connect directly to each OCXO CTL input via
    short green wires.  PWM dithering is eliminated.
  • Pins 22 and 11 return to the unassigned pool.

v15 CHANGES (from v14):
  • GNSS moved fully onto QTimer1 production ownership.
  • QTimer1 ch0 + ch1 form the cascaded 32-bit GNSS raw counter on pin 10.
  • QTimer1 ch2 is reserved as the GNSS TimePop compare / interrupt channel.
  • OCXO1 remains on GPT1 (pin 25).
  • OCXO2 remains on GPT2 (pin 14).
  • Clock ownership is now symmetric: GPT1 = OCXO1, GPT2 = OCXO2,
    QTimer1 = GNSS.

Teensy Pin    Wire Color    Signal Name        Source / Destination                 Notes
------------------------------------------------------------------------------------------
VIN           White         VIN_5V5            INA260 (5.5 V rail)                  Dedicated 5V+ CPU power
GND           Black         GND                Battery branching ground             Direct return to battery
1             Twisted Pair  GNSS_PPS_IN        GF-8802 PPS                          1 Hz absolute time reference
4             Green         GNSS_LOCK_IN       GF-8802 LOCK                         Lock status signal
10            Twisted Pair  GNSS_10MHZ_IN      GF-8802 VCLOCK                       10 MHz reference (QTimer1 ch0+ch1 counter)
14            Twisted Pair  OCXO2_10MHZ_IN     OCXO2                                GPT2 external clock
23            Green         DAC_VREF_OUT       AD5693R VREF (both)                  Software-controlled reference voltage
25            Twisted Pair  OCXO1_10MHZ_IN     OCXO1                                GPT1 external clock
32            Orange        GNSS_PPS_RELAY     GPIO relay to Pi                     Splits to GPIO18 and GPIO25
15            Yellow        PHOTODIODE_ADC     Photodiode output                    Analog input (ADC)
34            Orange        PHOTODIODE_INT     Photodiode interrupt                 Digital interrupt
20            White         LASER_PD_PLUS      Laser diode PD+                      Laser on/off monitor
30            Green         LASER_EN           EV5491-C-00A EN pin                  Laser driver enable
18            Blue          SDA1               Rail bus SDA1                        Primary I2C data
19            Yellow        SCL1               Rail bus SCL1                        Primary I2C clock

-------------------------------------------------------------------------------
Unassigned pins (available for future use):
  5, 6, 7, 8, 11, 12, 13, 22

Timer hardware binding summary:

  Pin 25  →  GPT1 external clock        (OCXO1 10 MHz, single-edge, 32-bit)
  Pin 14  →  GPT2 external clock        (OCXO2 10 MHz, single-edge, 32-bit)
  Pin 10  →  QTimer1 ch0+ch1 counter    (GNSS 10 MHz input, cascaded 32-bit)
  Pin 10  →  QTimer1 ch2 compare        (GNSS TimePop compare / interrupt path)
  Pin 23  →  DAC1 analog output         (VREF for both AD5693R DACs)

Notes:
• QTimer1 ch2 compare matches only the low 16 bits in hardware; production
  TimePop qualifies the full 32-bit target in software before firing.
• Pin 13 (LED_BUILTIN) reserved for fault Morse annunciator + PPS heartbeat.
• Pin 23 (DAC1) is a true 12-bit DAC output (not PWM).  It drives the
  external reference voltage for both AD5693R devices.  The reference
  voltage can be adjusted in software to optimize 16-bit resolution
  around the OCXO servo lock point.
• STP shield drains at source end.
=============================================================================*/


/*=============================================================================
 (3) TIMING SIGNAL WIRING SUMMARY
-------------------------------------------------------------------------------

Signal Name          Source          Destination       Frequency    Timer HW                 Shield Drain
----------------------------------------------------------------------------------------------------------
GNSS_PPS_IN          GF-8802 P17     Teensy pin 1      1 Hz         GPIO IRQ                 GF-8802 end
GNSS_10MHZ_IN        GF-8802 P11     Teensy pin 10     10 MHz       QTimer1 ch0+ch1 (+ ch2)  GF-8802 end
OCXO1_10MHZ_IN       OCXO1           Teensy pin 25     10 MHz       GPT1                     OCXO1 end
OCXO2_10MHZ_IN       OCXO2           Teensy pin 14     10 MHz       GPT2                     OCXO2 end
GNSS_PPS_RELAY       Teensy pin 32   Pi GPIO18/25      1 Hz         —                        Teensy end
=============================================================================*/


/*=============================================================================
 (4) LASER DIODE MODULE — FFIME LSDLD131
-------------------------------------------------------------------------------

Pin #    Wire Color    Signal Name     Electrical / Voltage        Notes
---------------------------------------------------------------------------
1        Blue          CASE_GND        Chassis / case              Parked
2        Black         LD- / PD-        GND (EV5491 GND)           Common return
3        Red           LD+              ~1.397 V                   EV5491 ID1
4        White         PD+              ~0.918 V                   Monitored (laser on/off)
=============================================================================*/

/*=============================================================================
 (5) CONTROLLER MODULE — EV5491-C-00A LASER DRIVER (USED CONNECTIONS)
-------------------------------------------------------------------------------

Signal     Wire Color    Connected To        Teensy Pin    Notes
---------------------------------------------------------------------------
EN         Green         Teensy GPIO         30            Laser enable
ID1        Red           Laser diode LD+     N/A           Drives LD+
GND        Black         Laser diode GND     N/A           Common return

Terminal   Wire Color    Connected To        Rail
---------------------------------------------------------------------------
VIN1       Purple        Backplane           +3V3
VIN2       Purple        Backplane           +3V3
GND        Black         Backplane           GND

Signal     Wire Color    Connected To        Rail
---------------------------------------------------------------------------
SDA        Blue          Backplane           SDA1
SCL        Yellow        Backplane           SCL1
GND        Black         Backplane           GND
=============================================================================*/

/*=============================================================================
 (6) CONTROLLER MODULE — GF-8802 GNSS DISCIPLINED OSCILLATOR
-------------------------------------------------------------------------------

Pin #    Signal Name        Wire Color    Connected To        Destination     Notes
----------------------------------------------------------------------------------
2        VIN (antenna)     White         Backplane           +5V            Antenna supply
5        GND               Black         Backplane           GND            Primary ground
9        VCC In            Purple        Backplane           +3V3           Logic power
17       PPS Output        Orange        Teensy              Pin 1          Primary PPS
15       LOCK Signal       Green         Teensy              Pin 4          Lock status
11       BCLOCK Out        Twisted Pair  Teensy              Pin 10         10 MHz (QTimer1 counter + compare base)
13       Serial Out (TX)   Blue          Raspberry Pi        RXD            GNSS → Pi
12       Serial In (RX)    Yellow        Raspberry Pi        TXD            Pi → GNSS

Unused: pins 1, 6, 7, 8, 14, 16, 18.
=============================================================================*/

/*=============================================================================
 (7) I2C BUS 2 (SMBUS2) — SDA2 (Green) / SCL2 (Orange) — Not yet energized
-------------------------------------------------------------------------------

Device Type   Address   A0 Strap   A1 Strap   Function
---------------------------------------------------------------------------
INA260        0x40      Open       Open       Power monitor — Pi
INA260        0x41      Bridged    Open       Power monitor — OCXO2
INA260        0x44      Open       Bridged    Power monitor — OCXO1
INA260        0x45      Bridged    Bridged    Power monitor — Teensy
=============================================================================*/

/*=============================================================================
 (8) I2C BUS 1 (SMBUS1) — SDA1 (Blue) / SCL1 (Yellow) — Active
-------------------------------------------------------------------------------

Device Type   Address   A0 Strap   Function                    VDD     Wire Colors
----------------------------------------------------------------------------------
INA260        0x40      A1=Brdg   Power monitor — 3.3 V rail  3.3 V   Blk/Blu/Yel/Pur
INA260        0x41      A0=Brdg   Power monitor — 5.0 V rail  3.3 V   Blk/Blu/Yel/Pur
INA260        0x44      Default   Power monitor — Battery      3.3 V   Blk/Blu/Yel/Pur
AD5693R       0x4E      A0=High   OCXO1 DAC (16-bit)           5.0 V   Blk/Blu/Yel/Wht
AD5693R       0x4C      A0=Low    OCXO2 DAC (16-bit)           5.0 V   Blk/Blu/Yel/Wht
BME280        0x76      Fixed     Environmental sensor         3.3 V   Blk/Blu/Yel/Pur
EV5491        0x66      Fixed     Laser controller (I2C)       3.3 V   Blk/Blu/Yel/Pur

Notes:
• AD5693R devices use 5 V VDD (white wire).  Internal reference is
  disabled; external reference voltage is supplied by Teensy pin 23
  (DAC1, 12-bit true DAC).  At 1× gain, output spans 0 V to VREF.
• AD5693R VOUT connects directly to OCXO CTL input via short green wire.
• AD5693R VREF on both devices connects to Teensy pin 23 (one wire each,
  shared terminal).
• LDAC on both AD5693R breakouts is tied to GND (immediate update on write).
• The software-controlled VREF allows the servo control window to be
  optimized around the OCXO lock point for maximum 16-bit resolution.
=============================================================================*/

/*=============================================================================
 (9) OCXO1 — AOCJY1-A
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 25 (GPT1) via STP
  CTL input      ←  AD5693R 0x4E VOUT (green wire, direct)
  VREF source    ←  Teensy pin 23 (DAC1, software-controlled)
  Power          →  Dedicated 5V domain

GPT1, 32-bit, single-edge.  Shield drain at OCXO1 end.
History: GPT1/pin25 → QTimer2/pin13 (LED conflict) → QTimer4/pin6
(XBAR, no direct access) → GPT1/pin25 (final).
CTL history: Teensy pin 22 (PWM 12-bit + dither) → AD5693R 0x4E (I2C 16-bit).
VREF history: Internal 2.5 V → External from Teensy DAC1 pin 23.
=============================================================================*/

/*=============================================================================
 (10) OCXO2 — AOCJY1-A (second unit)
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 14 (GPT2) via STP
  CTL input      ←  AD5693R 0x4C VOUT (green wire, direct)
  VREF source    ←  Teensy pin 23 (DAC1, software-controlled)
  Power          →  Dedicated 5V domain

GPT2, 32-bit, single-edge.  Symmetric with OCXO1.  Shield drain at OCXO2 end.
Previously QTimer1/pin10 — moved to GPT2 to eliminate ±15,000 count
phase aliasing artifacts from the earlier dual-edge OCXO experiment.
CTL history: Teensy pin 5 (PWM 12-bit + dither) → AD5693R 0x4C (I2C 16-bit).
VREF history: Internal 2.5 V → External from Teensy DAC1 pin 23.
=============================================================================*/