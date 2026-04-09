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
 (2) TEENSY 4.1 PIN ASSIGNMENTS — v20 FINAL QTimer STABILIZED ARCHITECTURE
-------------------------------------------------------------------------------

FINAL DECISION:

  • No re-wiring required.
  • VCLOCK remains on QTimer1.
  • OCXO1 and OCXO2 remain on QTimer3 CH2/CH3.
  • 16-bit counters retained with correct rollover reconstruction.
  • Quantization eliminated (GPT path removed).
  • Symmetry implemented in software.

FINAL TIMER TOPOLOGY:

  QTimer1 → VCLOCK → pin 10
  QTimer3 → OCXO1  → pin 14 (CH2)
  QTimer3 → OCXO2  → pin 15 (CH3)

CRITICAL PIN NOTES:

  Pin 11 = QTIMER1_TIMER2 (CH2) — TimePop compare scheduler
  DO NOT use pin 11 for external clock input

-------------------------------------------------------------------------------

Teensy Pin    Wire Color    Signal Name        Source / Destination                 Notes
------------------------------------------------------------------------------------------
VIN           White         VIN_5V5            INA260 (5.5 V rail)                  Dedicated 5V+ CPU power
GND           Black         GND                Battery branching ground             Direct return to battery
1             Twisted Pair  GNSS_PPS_IN        GF-8802 PPS                          1 Hz absolute time reference
4             Green         GNSS_LOCK_IN       GF-8802 LOCK                         Lock status signal
10            Twisted Pair  GNSS_10MHZ_IN      GF-8802 VCLOCK                       QTimer1 ch0+ch1

14            Twisted Pair  OCXO1_10MHZ_IN     OCXO1                                QTimer3 CH2
15            Twisted Pair  OCXO2_10MHZ_IN     OCXO2                                QTimer3 CH3
18            Blue          SDA1               Rail bus SDA1
19            Yellow        SCL1               Rail bus SCL1
20            White         LASER_PD_PLUS      Laser diode PD+

23            Green         DAC_VREF_OUT       AD5693R VREF (both)
32            Orange        GNSS_PPS_RELAY     GPIO relay to Pi
34            Orange        PHOTODIODE_INT     Photodiode interrupt

30            Green         LASER_EN           EV5491-C-00A EN pin

-------------------------------------------------------------------------------
Timer hardware binding summary:

  Pin 14  →  QTimer3 CH2
  Pin 15  →  QTimer3 CH3
  Pin 10  →  QTimer1 ch0+ch1
  Pin 10  →  QTimer1 ch2 compare

=============================================================================*/



/*=============================================================================
 (3) TIMING SIGNAL WIRING SUMMARY
-------------------------------------------------------------------------------

Signal Name          Source          Destination       Frequency    Timer HW
----------------------------------------------------------------------------------------------------------
GNSS_PPS_IN          GF-8802 P17     Teensy pin 1      1 Hz         GPIO IRQ
GNSS_10MHZ_IN        GF-8802 P11     Teensy pin 10     10 MHz       QTimer1
OCXO1_10MHZ_IN       OCXO1           Teensy pin 14     10 MHz       QTimer3 CH2
OCXO2_10MHZ_IN       OCXO2           Teensy pin 15     10 MHz       QTimer3 CH3
GNSS_PPS_RELAY       Teensy pin 32   Pi GPIO18/25      1 Hz         —

===========================================================================*/


/*=============================================================================
 (9) OCXO1 — AOCJY1-A
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 14 (QTimer3 CH2) via STP
  CTL input      ←  AD5693R 0x4E VOUT
  VREF source    ←  Teensy pin 23
  Power          →  Dedicated 5V domain

QTimer3 CH2, 16-bit free-running external clock count.
Rollover handled in software.

Shield drain at OCXO1 end.

History: GPT1 → QTimer2 → QTimer4 → QTimer3 CH2 (final stable)

=============================================================================*/


/*=============================================================================
 (10) OCXO2 — AOCJY1-A (second unit)
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 15 (QTimer3 CH3) via STP
  CTL input      ←  AD5693R 0x4C VOUT
  VREF source    ←  Teensy pin 23
  Power          →  Dedicated 5V domain

QTimer3 CH3, 16-bit free-running external clock count.
Rollover handled in software.

Symmetric with OCXO1.

=============================================================================*/

/*=============================================================================
 (4) LASER DIODE MODULE — FFIME LSDLD131
-------------------------------------------------------------------------------

Device:
FFIME LSDLD131
Wavelength: 1310 nm
Optical power: ~3 mW
Connector: FC/APC
Driver: EV5491

Pin numbering starts at the gold pin and proceeds clockwise.

Pin #    Wire Color    Signal Name     Electrical / Voltage        Notes
---------------------------------------------------------------------------
1        Blue          CASE_GND        Chassis / case              Parked
2        Black         LD- / PD-        GND (EV5491 GND)           Common return
3        Red           LD+              ~1.397 V                   EV5491 ID1
4        White         PD+              ~0.918 V                   Monitored (laser on/off)

---------------------------------------------------------------------------
Notes:
• Case ground is intentionally parked and not bonded by default.
• LD+ / LD- are driven exclusively by the EV5491 laser driver.
• PD+ / PD- provide laser activity monitoring.
• No pin on this device should ever be exposed to 5 V.
• Optical output is active once LD+ is energized.
=============================================================================*/

/*=============================================================================
 (5) CONTROLLER MODULE — EV5491-C-00A LASER DRIVER (USED CONNECTIONS)
-------------------------------------------------------------------------------

Controller:
EV5491-C-00A
Function: Laser diode driver for FFIME LSDLD131

This table documents ONLY the connections that are actually used.
Unused pins are intentionally omitted to reduce cognitive load.

-------------------------------------------------------------------------------
Control / Signal Connections
-------------------------------------------------------------------------------

Signal     Wire Color    Connected To        Teensy Pin    Notes
---------------------------------------------------------------------------
EN         Green         Teensy GPIO         30            Laser enable
ID1        Red           Laser diode LD+     N/A           Drives LD+ (~1.397 V)
GND        Black         Laser diode GND     N/A           Common return

-------------------------------------------------------------------------------
Power Connections
-------------------------------------------------------------------------------

Terminal   Wire Color    Connected To        Rail           Notes
---------------------------------------------------------------------------
VIN1       Purple        Backplane           +3V3           Primary supply
VIN2       Purple        Backplane           +3V3           Secondary supply
GND        Black         Backplane           GND            Power ground
GND        —             —                   —              Unused

-------------------------------------------------------------------------------
I2C Connections
-------------------------------------------------------------------------------

Signal     Wire Color    Connected To        Rail           Notes
---------------------------------------------------------------------------
SDA        Blue          Backplane           SDA1           Primary I2C data
SCL        Yellow        Backplane           SCL1           Primary I2C clock
GND        Black         Backplane           GND            I2C reference

-------------------------------------------------------------------------------
Notes:
• EN is the only controller signal driven by the Teensy.
• ID1 directly drives the laser diode LD+ pin.
• All controller power is sourced from the +3V3 rail.
• I2C is present for configuration/monitoring as supported.
• Unused controller pins are intentionally left undocumented.
=============================================================================*/

/*=============================================================================
 (6) CONTROLLER MODULE — GF-8802 GNSS DISCIPLINED OSCILLATOR
-------------------------------------------------------------------------------

Device:
GF-8802
Function: GNSS timing reference and disciplined oscillator

Pin order below matches the physical pin layout when viewed from above.
All pins are listed; unused pins are explicitly marked.

-------------------------------------------------------------------------------
Power / Ground Pins
-------------------------------------------------------------------------------

Pin #    Signal Name        Wire Color    Connected To        Rail / Pin     Notes
----------------------------------------------------------------------------------
2        VIN (antenna)     White         Backplane           +5V            Antenna supply
5        GND               Black         Backplane           GND            Primary ground
7        GND               —             —                   —              Unused
8        Backup Power In   —             —                   —              Unused
9        VCC In            Purple        Backplane           +3V3           Logic power

-------------------------------------------------------------------------------
Timing / Control Outputs
-------------------------------------------------------------------------------

Pin #    Signal Name        Wire Color    Connected To        Destination     Notes
----------------------------------------------------------------------------------
18       EPPS Output       —             —                   —              Unused
17       PPS Output        STP           Teensy              Pin 1          Primary PPS
16       GLCK Out          —             —                   —              Unused
15       LOCK Signal       Green         Teensy              4            Lock status
14       Alarm             —             —                   —              Unused
11       VCLOCK Out        STP           Teensy              10             10 MHz square wave

-------------------------------------------------------------------------------
Communication Pins
-------------------------------------------------------------------------------

Pin #    Signal Name        Wire Color    Connected To        Destination     Notes
----------------------------------------------------------------------------------
13       Serial Out (TX)   Blue          Raspberry Pi        RXD             GNSS → Pi
12       Serial In (RX)    Yellow        Raspberry Pi        TXD             Pi → GNSS

-------------------------------------------------------------------------------
Unused / RF Pins
-------------------------------------------------------------------------------

Pin #    Signal Name        Notes
-----------------------------------------
1        Reset             Unused
6        RF Pin            Unused

-------------------------------------------------------------------------------
Notes:
• PPS (Pin 17) is the primary absolute time reference for the system.
• VCLOCK (Pin 11) provides a 10 MHz square-wave reference to the Teensy.
• LOCK signal is wired but Teensy pin assignment is pending.
• Antenna VIN is powered from the +5V rail.
• All unused pins are intentionally left unconnected.
• This table reflects physical wiring as built, not schematic ideals.
=============================================================================*/

/*=============================================================================
INA260 Address Mapping (Adafruit Boards)
-------------------------------------------------------------------------------

A0 = FALSE, A1 = FALSE   -> 0x40
A0 = FALSE, A1 = TRUE    -> 0x41
A0 = TRUE,  A1 = FALSE   -> 0x44
A0 = TRUE,  A1 = TRUE    -> 0x45

Notes:
• Although INA260 supports additional addresses in theory, Adafruit boards
  expose only the configurations listed above.
• Address 0x45 is currently unused on SMBUS1.
• All devices on this bus use SDA1/SCL1 (Blue / Yellow).
• This bus is already energized and operational.
=============================================================================*/
