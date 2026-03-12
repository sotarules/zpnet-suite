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

IO18 (GPIO18)     Orange        GNSS_PPS_RELAY     GF-8802 PPS via Teensy         GNSS PPS relay (critical timing)

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

1             Orange        GNSS_PPS_IN        GNSS PPS                             1 Hz pulse for absolute time reference

20            White         LASER_PD_PLUS      Laser diode PD+                      Indicates laser on/off (active signal)

18            Blue          SDA1               Rail bus SDA1                        Primary I2C data
19            Yellow        SCL1               Rail bus SCL1                        Primary I2C clock

15            Yellow        PHOTODIODE_ADC     Photodiode output                    Analog input (ADC)
14            Twisted Pair  GNSS_VCLOCK        GNSS vclock output                   10 MHz reference from GF-8802 (GPT2 external clock)

22            Green         OCXO1_CTL          OCXO1 Pin 1 (CTL)                   PWM output for OCXO1 frequency trim

30            Green         LASER_EN           EV5491-C-00A EN pin                  Laser driver enable

32            Orange        GNSS_PPS_RELAY     GPIO relay to Pi for GNSS PPS        Splits to GPIO18 and GPIO25

34            Orange        PHOTODIODE_INT     Photodiode interrupt                 Digital interrupt

4             Green         GNSS_LOCK_IN       GF-8802                              Lock status signal

25            Twisted Pair  OCXO1_10MHZ_IN     OCXO1                                GPT1 external clock (critical timing)

10            Twisted Pair  OCXO2_10MHZ_IN     OCXO2                                QTimer1 ch0 external clock (critical timing)

11            Green         OCXO2_CTL          OCXO2 CTL                            PWM output for OCXO2 frequency trim

-------------------------------------------------------------------------------
Unassigned pins (available for future use):
  5, 6, 7, 8, 12

Timer hardware binding summary:

  Pin 14  →  GPT2 external clock   (GNSS VCLOCK 10 MHz)
             IOMUXC ALT8, CLKSRC=3
             Also hosts TimePop output compare (OCR1)

  Pin 25  →  GPT1 external clock   (OCXO1 10 MHz)
             IOMUXC ALT1, CLKSRC=3

  Pin 10  →  QTimer1 ch0 input     (OCXO2 10 MHz)
             IOMUXC ALT1 (GPIO_B0_00 → QTIMER1_TIMER0)
             Cascaded with ch1 for 32-bit free-running count
             16-bit ch0 wraps every ~6.5 ms at 10 MHz
             32-bit cascaded wraps every ~429 s (same as GPT)

  Pin 22  →  FlexPWM (OCXO1 CTL)   analogWrite 12-bit + dither
  Pin 11  →  FlexPWM (OCXO2 CTL)   analogWrite 12-bit + dither

Notes:
• Wire colors are pragmatic (limited Kynar stock), not canonical signal colors.
• VIN and GND belong to isolated CPU power domain.
• Laser PD+ is intentionally monitored (not parked).
• GNSS_VCLOCK is passively clamped to protect Teensy input.
• OCXO1_10MHZ_IN and OCXO2_10MHZ_IN use shielded twisted pair,
  consistent with all other timing-critical signals.
• GND for STP shield drains uses dupont female header (not screw terminal),
  as screw terminals are fully committed.
• Table reflects current physical wiring, not final design.
=============================================================================*/


/*=============================================================================
 (3) TIMING SIGNAL WIRING SUMMARY
-------------------------------------------------------------------------------

This section consolidates all timing-critical signal paths for reference.
All timing signals use Shielded Twisted Pair (STP).
All shields are drained at the SOURCE end only to prevent ground loops.

Signal Name          Source          Destination       Frequency    Timer HW         Shield Drain
--------------------------------------------------------------------------------------------------
GNSS_PPS_IN          GF-8802 P17     Teensy pin 1      1 Hz         GPIO IRQ         GF-8802 end
GNSS_VCLOCK          GF-8802 P11     Teensy pin 14     10 MHz       GPT2 ext clk     GF-8802 end
OCXO1_10MHZ_IN       OCXO1           Teensy pin 25     10 MHz       GPT1 ext clk     OCXO1 end
OCXO2_10MHZ_IN       OCXO2           Teensy pin 10     10 MHz       QTimer1 ch0      OCXO2 end
GNSS_PPS_RELAY 1     Teensy pin 32   Pi GPIO18         1 Hz         —                Teensy end
GNSS_PPS_RELAY 2     Teensy pin 32   Pi GPIO25         1 Hz         —                Teensy end

--------------------------------------------------------------------------------------------------
Notes:
• GPT2 also hosts TimePop output compare (OCR1) on the same counter.
  The output compare does not disturb the free-running counter.
• QTimer1 ch0 (pin 10) is cascaded with ch1 for 32-bit counting.
  At 10 MHz: ch0 wraps every ~6.5 ms, cascaded wraps every ~429 s.
• All three 10 MHz signals (GNSS, OCXO1, OCXO2) are captured in the
  PPS ISR simultaneously for cross-clock correlation.
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
18       EPPS Output       —             —                   —                Unused
17       PPS Output        Orange        Teensy              Pin 1            Primary PPS
16       GLCK Out          —             —                   —                Unused
15       LOCK Signal       Green         Teensy              Pin 4            Lock status
14       Alarm             —             —                   —                Unused
11       BCLOCK Out        Twisted Pair  Teensy              Pin 14           10 MHz square wave (GPT2)

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
• LOCK signal is wired to Teensy pin 4.
• Antenna VIN is powered from the +5V rail.
• All unused pins are intentionally left unconnected.
• This table reflects physical wiring as built, not schematic ideals.
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

-------------------------------------------------------------------------------
Notes:
• INA260 base address is 0x40.
• Address selection formula: 0x40 + (A1 << 1) + A0.
• All devices on this bus use SDA2/SCL2 (Green / Orange).
• This bus has not yet been powered or scanned.
• Address conflicts are not expected given current strapping.
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
INA260        0x40      Open       Bridged    Power monitor — 3.3 V rail
INA260        0x41      Bridged    Open       Power monitor — 5.0 V rail
INA260        0x44      Open       Open       Power monitor — Battery

BME280        0x76      Fixed      Fixed      Environmental sensor
EV5491        0x66      Fixed      Fixed      Laser controller (I2C)

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
 (9) OCXO1 — AOCJY1-A OVEN-CONTROLLED CRYSTAL OSCILLATOR (ORIGINAL)
-------------------------------------------------------------------------------

Device:
AOCJY1-A
Function: 10 MHz precision oven-controlled oscillator
Power domain: Independent (dedicated INA260 on SMBUS2, 0x45)

Connections:
  10 MHz output  →  Teensy pin 25 (GPT1 external clock) via STP
  CTL input      →  Teensy pin 22 (PWM DAC output, 12-bit + dither)
  Power          →  Dedicated 5V domain
  GND            →  System ground

Notes:
• OCXO1 is intentionally free-running post-calibration as a local
  inertial clock whose ratio to GNSS is the primary experimental signal.
• CTL voltage controls frequency trim.  Higher voltage = higher frequency.
• The servo loop runs in process_clocks.cpp with sub-ppb convergence.
• DAC value is persisted across campaigns in SYSTEM config.
• Shield drain is at the OCXO1 end (source-side grounding).
=============================================================================*/

/*=============================================================================
 (10) OCXO2 — AOCJY1-A OVEN-CONTROLLED CRYSTAL OSCILLATOR (SECOND)
-------------------------------------------------------------------------------

Device:
AOCJY1-A (identical to OCXO1)
Function: 10 MHz precision oven-controlled oscillator
Mounting: SDM (Small Device Module) — perfboard with 6mm standoffs on PETG base
Power domain: Independent (INA260 TBD or shared with OCXO1)

Connections:
  10 MHz output  →  Teensy pin 10 (QTimer1 ch0 external clock) via STP
  CTL input      →  Teensy pin 11 (PWM DAC output, 12-bit + dither)
  Power          →  Dedicated 5V domain (TBD)
  GND            →  System ground

Timer hardware:
  QTimer1 ch0 counts the 10 MHz input directly.
  QTimer1 ch1 cascades from ch0 overflow for 32-bit range.
  At 10 MHz: ch0 (16-bit) wraps every ~6.5 ms.
  Cascaded 32-bit wraps every ~429 seconds (same as GPT1/GPT2).
  64-bit extension via delta accumulation in process_clocks.cpp,
  identical pattern to OCXO1 on GPT1.

Notes:
• OCXO2 provides an independent local inertial clock on a separate
  power domain from OCXO1.  The two OCXOs share no electrical path
  other than the system ground reference.
• The experimental signal of interest is the ratio between OCXO1 and
  OCXO2, measured against GNSS as the common reference.  Any altitude-
  dependent rate change should appear symmetrically in both OCXOs
  relative to GNSS, and the OCXO1/OCXO2 ratio should remain constant
  if both experience the same gravitational potential.
• CTL voltage controls frequency trim.  Identical dither servo as OCXO1.
• Shield drain is at the OCXO2 end (source-side grounding).
=============================================================================*/

/*=============================================================================
 (11) CLOCK DOMAIN SUMMARY — FOUR AUTHORITATIVE CLOCKS
-------------------------------------------------------------------------------

ZPNet measures time using exactly four clock domains:

Domain     Source             Timer HW           Pin    Frequency    Resolution
--------------------------------------------------------------------------------
DWT        ARM Cortex-M7      DWT_CYCCNT          —     1008 MHz     ~1 ns
GNSS       GF-8802 VCLOCK     GPT2 (32-bit)      14     10 MHz       100 ns
OCXO1      AOCJY1-A #1        GPT1 (32-bit)      25     10 MHz       100 ns
OCXO2      AOCJY1-A #2        QTimer1 (32-bit)   10     10 MHz       100 ns

All four counters free-run continuously from power-on.
All three 10 MHz counters are captured simultaneously in the PPS ISR.
DWT is captured in the same ISR, ~2 ns after the 10 MHz captures.

The PPS edge from the GF-8802 (pin 1) is the synchronization event.
At each PPS, the ISR reads:
  • DWT_CYCCNT      (ARM register, immediate)
  • GPT2_CNT        (GNSS 10 MHz ticks)
  • GPT1_CNT        (OCXO1 10 MHz ticks)
  • QTimer1 ch0+ch1 (OCXO2 10 MHz ticks, cascaded 32-bit)

These four values, captured within ~10 ns of each other, form the
per-second clock tuple that anchors all TIMEBASE records.

GNSS is the deterministic reference: campaign_seconds × 1,000,000,000.
DWT, OCXO1, and OCXO2 are measured against it.
The experimental signal is the tau ratio of each clock to GNSS.
=============================================================================*/