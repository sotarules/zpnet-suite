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
  • Each clock lane uses its pin-bound QTimer channel for BOTH count and compare.
  • VCLOCK remains on QTimer1 CH0 / pin 10.
  • OCXO1 remains on QTimer2 CH0 / pin 13.
  • OCXO2 remains on QTimer3 CH3 / pin 15.
  • TimePop remains a scheduler on QTimer1 CH2 and is not VCLOCK authority.

FINAL TIMER TOPOLOGY:

  QTimer1 CH0 → VCLOCK → pin 10  (count + compare)
  QTimer2 CH0 → OCXO1  → pin 13  (count + compare)
  QTimer3 CH3 → OCXO2  → pin 15  (count + compare)
  QTimer1 CH2 → TimePop scheduler only

CRITICAL PIN NOTES:

  QTimer1 CH2 is reserved for TimePop compare scheduling.
  DO NOT use QTimer1 CH2 as VCLOCK edge authority.

-------------------------------------------------------------------------------

Teensy Pin    Wire Color    Signal Name        Source / Destination                 Notes
------------------------------------------------------------------------------------------
VIN           White         VIN_5V5            INA260 (5.5 V rail)                  Dedicated 5V+ CPU power
GND           Black         GND                Battery branching ground             Direct return to battery
1             Twisted Pair  GNSS_PPS_IN        GF-8802 PPS                          1 Hz absolute time reference
4             Green         GNSS_LOCK_IN       GF-8802 LOCK                         Lock status signal
10            Twisted Pair  GNSS_10MHZ_IN      GF-8802 VCLOCK                       QTimer1 CH0 count+compare

13            Twisted Pair  OCXO1_10MHZ_IN     OCXO1                                QTimer2 CH0 count+compare
15            Twisted Pair  OCXO2_10MHZ_IN     OCXO2                                QTimer3 CH3 count+compare
18            Blue          SDA1               Rail bus SDA1
19            Yellow        SCL1               Rail bus SCL1
20            White         LASER_PD_PLUS      Laser diode PD+

23            Green         DAC_VREF_OUT       AD5693R VREF (both)
32            Orange        GNSS_PPS_RELAY     GPIO relay to Pi
34            Coax          PHOTODIODE_INT     Koheron PD200T TTL out                Comparator timing / GPIO2[29] IRQ P48
35            Coax/pigtail  LASER_MOD          TC4427 MDM -> Koheron DRV200 MOD     Active-high: LOW idle; HIGH positive modulation
38            Coax          PHOTODIODE_ANALOG_IN  Koheron PD200T PD OUT              Analog photodetector output / A14 ADC

30            --            FREE               --                                   Released 2026-09-13; EV5491 retired

-------------------------------------------------------------------------------
Timer hardware binding summary:

  Pin 10  →  QTimer1 CH0  VCLOCK count + compare authority
  Pin 13  →  QTimer2 CH0  OCXO1 count + compare authority
  Pin 15  →  QTimer3 CH3  OCXO2 count + compare authority
  QTimer1 CH2             TimePop scheduler only

Interrupt priority / GPIO routing summary:

  Priority 0   →  PPS GPIO, OCXO1, OCXO2 sovereign CLOCKS capture
  Priority 16  →  QTimer1 VCLOCK + TimePop shared vector
  Priority 32  →  process_interrupt continuation/handoff
  Priority 48  →  PHOTODIODE_INT, pin 34 / GPIO2[29] / IRQ_GPIO2_16_31

  Pin 34 wiring does NOT change.  process_interrupt remaps GPIO_B1_13 internally
  from Teensy's fast GPIO7[29] alias to ordinary GPIO2[29] so the detector gets
  an independently prioritizable vector below every CLOCKS tier.  CLOCKS may
  delay PHOTODIODE; PHOTODIODE must never delay CLOCKS.

=============================================================================*/



/*=============================================================================
 (3) TIMING SIGNAL WIRING SUMMARY
-------------------------------------------------------------------------------

Signal Name          Source          Destination       Frequency    Timer HW
----------------------------------------------------------------------------------------------------------
GNSS_PPS_IN          GF-8802 P17     Teensy pin 1      1 Hz         GPIO IRQ
GNSS_10MHZ_IN        GF-8802 P11     Teensy pin 10     10 MHz       QTimer1 CH0
OCXO1_10MHZ_IN       OCXO1           Teensy pin 13     10 MHz       QTimer2 CH0
OCXO2_10MHZ_IN       OCXO2           Teensy pin 15     10 MHz       QTimer3 CH3
GNSS_PPS_RELAY       Teensy pin 32   Pi GPIO18/25      1 Hz         —

===========================================================================*/


/*=============================================================================
 (9) OCXO1 — AOCJY1-A
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 13 (QTimer2 CH0) via STP
  CTL input      ←  AD5693R 0x4E VOUT
  VREF source    ←  Teensy pin 23
  Power          →  Dedicated 5V domain

QTimer2 CH0, 16-bit free-running external clock count + compare.
Rollover handled in software.

Shield drain at OCXO1 end.

History: GPT1 → QTimer2 → QTimer4 → split-channel experiment → QTimer2 CH0 same-channel custody.

=============================================================================*/


/*=============================================================================
 (10) OCXO2 — AOCJY1-A (second unit)
-------------------------------------------------------------------------------

  10 MHz output  →  Teensy pin 15 (QTimer3 CH3) via STP
  CTL input      ←  AD5693R 0x4C VOUT
  VREF source    ←  Teensy pin 23
  Power          →  Dedicated 5V domain

QTimer3 CH3, 16-bit free-running external clock count + compare.
Rollover handled in software.

Symmetric with OCXO1 by doctrine: same-channel count + compare, even though
the channel number is pin-routing-specific.

=============================================================================*/

/*=============================================================================
 (4) OPTICAL RECEIVER — KOHERON PD200T
-------------------------------------------------------------------------------

Device:
Koheron PD200T
Function: Optical pulse detection and comparator timing

Used outputs:

Output / Signal      Teensy Pin    ZPNet Signal Name       Purpose
---------------------------------------------------------------------------
TTL out              34            PHOTODIODE_INT          Comparator timing edge / GPIO IRQ
PD OUT               38 / A14      PHOTODIODE_ANALOG_IN    Analog photodetector output / ADC

Notes:
• TTL out is the authoritative digital photodetector timing signal presented
  to process_interrupt for DWT-at-edge capture.
• The physical TTL coax remains on Teensy pin 34.  process_interrupt remaps that
  pad to GPIO2[29] and services IRQ_GPIO2_16_31 at Priority 48, below the entire
  CLOCKS timing hierarchy.  If CLOCKS delays detector ISR entry, the optical
  endpoint retains that delay testimony and the corresponding race is expendable.
• PD OUT is the analog photodetector waveform and is ADC-read on pin 38/A14.
• During comparator-pot commissioning, pin 38/A14 shows received optical signal
  amplitude while the independent pin 34 TTL interrupt count shows comparator
  decisions.
• MON is the blue-pot comparator-threshold monitor and is not connected to the
  Teensy in the current architecture.
• Commissioned PD200T comparator threshold (2026-09-16): MON ≈0.950 V.
• With the commissioned optical/driver settings, TTL OUT reliably responds to
  PHOTONS pulses down through the original 20 ns command-width design target at
  a 100 ms test interval.
• PHOTODIODE_ANALOG_IN is telemetry only and is never a timing endpoint.
=============================================================================*/


/*=============================================================================
FFIME / presumed LSDLD131 — EXACT VARIANT UNVERIFIED

Nominal wavelength: 1310 nm (working attribution; exact device identity unverified)
Optical power: unknown; candidate LSDLD131 documentation specifies ~2 mW typ / 4 mW max
Connector: FC/APC
Driver: Koheron DRV200-A-40

IMPORTANT:
Numeric package-pin mapping is intentionally NOT asserted.
The functional wire assignments below are based primarily on empirical testing.

Wire Color    Functional Signal     Current Connection     Evidence / Notes
--------------------------------------------------------------------------------
Blue          CASE                  Parked                 0 Ω to metal can; empirically proved

Black         LD-                   DRV200 LD-             Laser cathode; floating from ZPNet GND

Red           LD+ / likely PD-      DRV200 LD+             Laser anode empirically proved:
                                                           original controller produced laser output;
                                                           DRV200 V(LD+-LD-) ≈ +1.521 V.
                                                           RED↔WHITE also forms separate diode junction,
                                                           consistent with shared monitor-PD terminal.

White         Monitor PD lead       Parked                 Likely PD+.
                                                           RED→WHITE diode test ≈0.492 V;
                                                           reverse direction OL.
                                                           DO NOT connect directly to Teensy pin 20
                                                           pending proper floating-domain interface.

Observed physical clockwise wire order:
BLUE → BLACK → RED → WHITE
(view/orientation recorded separately; do not infer package pin numbers from this alone)

DRV200 doctrine:
- RED -> LD+
- BLACK -> LD-
- BLUE parked
- WHITE parked
- laser diode must remain floating from ZPNet/system ground

/*=============================================================================
 (6) OPTICAL SOURCE DRIVER — KOHERON DRV200-A-40 + TC4427 MDM
-------------------------------------------------------------------------------

Driver:
Koheron DRV200-A-40
Function: Floating laser-current driver with analog MOD input

Interface module:
TC4427 MDM with 220 ohm series output resistor

-------------------------------------------------------------------------------
Control / Signal Connections
-------------------------------------------------------------------------------

Signal       Source / Destination                         Notes
---------------------------------------------------------------------------
LASER_MOD    Teensy pin 35 -> MDM SMA IN                  Active-high logic command
MDM OUT      MDM SMA OUT -> DRV200 MOD SMA                ~5 V unloaded; ~0.9 V into 50 ohm MOD input
LD+          DRV200 LD+ -> laser red                      Floating laser anode drive
LD-          DRV200 LD- -> laser black                    Floating laser return; never ZPNet GND
IMON         DRV200 terminal                              100 mV per mA on A-40; commissioning telemetry

-------------------------------------------------------------------------------
Commissioned DRV200 Settings (2026-09-16)
-------------------------------------------------------------------------------
DC bias        PHOTONS.OFF: IMON ≈0.897 V = 8.97 mA.
Static ON      PHOTONS.ON:  IMON ≈1.066 V = 10.66 mA.
MOD delta      ≈0.169 V IMON = 1.69 mA measured current increase.
MODGAIN        M.  On DRV200-A-40 this is 2 mA/V.
ILIM           L.  On DRV200-A-40 this limits current to 32 mA.
               H would raise the limit to 48 mA; L is intentionally retained.
VCC jumper     5 V.
MOD drive      Measured HIGH ≈0.862 V at the loaded MOD input; LOW ≈0.008-0.010 V.
               At MODGAIN=M, predicted HIGH current addition is ≈1.72 mA,
               closely matching the measured ≈1.69 mA IMON delta.

-------------------------------------------------------------------------------
Notes:
• Pin 35 is no longer an active-low gate. LOW means zero added modulation; HIGH
  applies positive DRV200 modulation through the non-inverting TC4427 MDM.
• DRV200 bias current and the hardware ON/OFF switch are local driver controls;
  Teensy pin 35 does not enable or remove the DC bias current.
• IMON is a DC/low-frequency monitor.  A sparse 20 ns WAVE pulse every 100 ms
  does not materially change a DMM IMON reading; use static PHOTONS.ON/OFF when
  verifying the modulation-current delta.
• The retired EV5491/MP5491 I2C controller and MOSFET daughterboard are removed.
• Teensy pin 30 (former EV5491 EN / LD_ON) is free.
• The DRV200 has no ZPNet I2C connection.
=============================================================================*/

/*=============================================================================
 (7) CONTROLLER MODULE — GF-8802 GNSS DISCIPLINED OSCILLATOR
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
15       LOCK Signal       Green         Teensy              4              Lock status
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
