#pragma once
/*
===============================================================================
 ZPNet — Pin, Rail, and Bus Assignment Ledger
-------------------------------------------------------------------------------

 STATUS: AUTHORITATIVE HUMAN-READABLE SOURCE
 PURPOSE: Establish a stable, shared understanding of physical wiring,
          rails, buses, and pin intent before final operational binding.

 This file is intentionally NON-OPERATIONAL.
 It contains no constants, no logic, and no compile-time meaning.

 It exists to:
   • Prevent wiring ambiguity
   • Capture color and rail semantics
   • Provide a safe surface for iteration
   • Serve as the precursor to formal pin bindings

 This file WILL change.
 That is expected.
===============================================================================
*/

/*=============================================================================
 (1) RAIL ASSIGNMENTS — 10-RAIL BACKPLANE
-------------------------------------------------------------------------------

Each row represents one physical rail on the shared backplane.
Rail number, color, and meaning are semantic commitments.

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

---------------------------------------------------------------------------
Notes:
• SMBUS2 is permanently identified by Green (SDA2) / Orange (SCL2).
• Devices attached to SMBUS2 inherit this color and bus identity.
• Rails are semantic, not just electrical voltages.
• Changes here represent physical rewiring commitments.
=============================================================================*/

/*=============================================================================
 RASPBERRY PI PIN ASSIGNMENTS — CONSOLIDATED
-------------------------------------------------------------------------------

Pins are labeled as on the Pi screw-terminal breakout (HAT).
GPIO numbers are BCM numbering (as printed on the board, e.g. IO23).

Pi Label / Pin    Wire Color    Signal Name        Connected To / Rail           Notes
---------------------------------------------------------------------------------------
5V                White         PI_5V              INA260 0x40 (Bus 2)            Dedicated Pi 5V power domain

GND               Black         GND                Battery branching ground       Shared system ground

SDA               Blue          SDA1               Rail SDA1 (Blue)               Primary I2C data (Bus 1)
SCL               Yellow        SCL1               Rail SCL1 (Yellow)             Primary I2C clock (Bus 1)

TXD               Yellow        PI_TXD             GF-8802 RX (Pin 12)            Pi → GNSS serial
RXD               Blue          PI_RXD             GF-8802 TX (Pin 13)            GNSS → Pi serial

IO18 (GPI018)     Orange        GNSS_PPS           GF-8802 PPS (Pin 17)           GNSS PPS shared with Teensy (critical timing)

IO23 (GPIO23)     Green         SDA2               Rail SDA2 (Green)              Secondary I2C data (Bus 2)
IO24 (GPIO24)     Orange        SCL2               Rail SCL2 (Orange)             Secondary I2C clock (Bus 2)

---------------------------------------------------------------------------------------
Notes:
• Bus 1 uses hardware I2C (SDA/SCL).
• Bus 2 uses software (bit-banged) I2C on GPIO23/24.
• Bus 2 is wired but not yet energized.
• Pi power is isolated via its own INA260 on Bus 2.
• No Pi GPIO is exposed to 5V signaling.
=============================================================================*/


/*=============================================================================
 (2) TEENSY 4.1 PIN ASSIGNMENTS — PROVISIONAL
-------------------------------------------------------------------------------

Teensy Pin    Wire Color    Signal Name        Source / Destination                 Notes
------------------------------------------------------------------------------------------
VIN           White         VIN_5V5            INA260 (5.5 V rail)                  Dedicated 5V+ CPU power (INA260 addr TBD)
GND           Black         GND                Battery branching ground             Direct return to battery

20            White         LASER_PD_PLUS      Laser diode PD+                      Indicates laser on/off (active signal)

19            Blue          SDA1               Rail bus SDA1                        Primary I2C data
18            Yellow        SCL1               Rail bus SCL1                        Primary I2C clock

15            Yellow        PHOTODIODE_ADC     Photodiode output                    Analog input (ADC)
14            Orange        GNSS_VCLOCK        GNSS vclock output                   Passive clamp to 0–3.3 V (GF-8802)

30            Green         LASER_EN           EV5491-C-00A EN pin                  Laser driver enable

32            Orange        GNSS_PPS_IN        GNSS PPS                             1 hz pulse for absolute time reference

34            Orange        PHOTODIODE_INT     Photodiode interrupt                 Digital interrupt

Teensy Pin    Wire Color    Signal Name        Source / Destination                 Notes
------------------------------------------------------------------------------------------
2             Yellow        RTC1_SQW_IN        DS3231 (Bus 1)                       Square wave output
3             Yellow        RTC2_SQW_IN        DS3231 (Bus 2)                       Square wave output

4             Green         GNSS_LOCK_IN       GF-8802                              Lock status signal

6             Orange        OCXO_10MHZ_IN      OCXO                                 GPT1 capture (critical)

-------------------------------------------------------------------------------
Notes:
• Wire colors are pragmatic (limited Kynar stock), not canonical signal colors.
• VIN and GND belong to isolated CPU power domain.
• Laser PD+ is intentionally monitored (not parked).
• GNSS_VCLOCK is passively clamped to protect Teensy input.
• Table reflects current physical wiring, not final design.
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
17       PPS Output        Brown         Teensy              Pin 33          Primary PPS
16       GLCK Out          —             —                   —              Unused
15       LOCK Signal       Green         Teensy              TBD             Lock status
14       Alarm             —             —                   —              Unused
11       BCLOCK Out        Orange        Teensy              Pin 14          10 MHz square wave

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
• BCLOCK (Pin 11) provides a 10 MHz square-wave reference to the Teensy.
• LOCK signal is wired but Teensy pin assignment is pending.
• Antenna VIN is powered from the +5V rail.
• All unused pins are intentionally left unconnected.
• This table reflects physical wiring as built, not schematic ideals.
=============================================================================*/

/*=============================================================================
 (8) I2C BUS 1 (SMBUS1) — DEVICE ASSIGNMENTS
-------------------------------------------------------------------------------

Bus Identity:
SMBUS1
Wiring: SDA1 (Blue) / SCL1 (Yellow)
Status: Active and in use

This table reflects the actual address usage of Adafruit INA260 boards
and other devices on SMBUS1.

-------------------------------------------------------------------------------
Device Inventory
-------------------------------------------------------------------------------

Device Type   Address   A0 Strap   A1 Strap   Function / Notes
---------------------------------------------------------------------------
INA260        0x40      Open       Open       Power monitor — Battery
INA260        0x41      Open       Bridged    Power monitor — 3.3 V rail
INA260        0x44      Bridged    Open       Power monitor — 5.0 V rail

BME280        0x76      Fixed      Fixed      Environmental sensor
EV5491        0x66      Fixed      Fixed      Laser controller (I2C)
DS3231 RTC    0x68      Fixed      Fixed      Secondary real-time clock

-------------------------------------------------------------------------------
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

/*=============================================================================
 (7) I2C BUS 2 (SMBUS2) — DEVICE ASSIGNMENTS
-------------------------------------------------------------------------------

Bus Identity:
SMBUS2
Wiring: SDA2 (Green) / SCL2 (Orange)
Status: Wired, not yet energized

All device addresses are inferred from hardware strap configuration.

-------------------------------------------------------------------------------
Device Inventory
-------------------------------------------------------------------------------

Device Type   Address   A0 Strap   A1 Strap   Function / Notes
---------------------------------------------------------------------------
INA260        0x40      Open       Open       Power monitor — Teensy
INA260        0x41      Bridged    Open       Power monitor — 24V stepper motors
INA260        0x44      Open       Bridged    Power monitor — Raspberry Pi
INA260        0x45      Bridged    Bridged    Power monitor — 3.3V OCXO

DS3231 RTC    0x68      Fixed      Fixed      Secondary real-time clock

-------------------------------------------------------------------------------
Notes:
• INA260 base address is 0x40.
• Address selection formula: 0x40 + (A1 << 1) + A0.
• DS3231 address is fixed at 0x68 (no strap options).
• All devices on this bus use SDA2/SCL2 (Green / Orange).
• This bus has not yet been powered or scanned.
• Address conflicts are not expected given current strapping.
=============================================================================*/
