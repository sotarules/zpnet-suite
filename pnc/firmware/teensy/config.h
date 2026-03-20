#pragma once

#include <cstdint>
#include <cstddef>

#define ZPNET_SERIAL Serial
#define FW_VERSION_STR "zpnet-teensy-5.2.0"
static constexpr const char* FW_VERSION = FW_VERSION_STR;

// -------------------------------------------------------------
// Transport backend selection (runtime)
// -------------------------------------------------------------

typedef enum {
  TRANSPORT_NONE = 0,
  TRANSPORT_HID,
  TRANSPORT_SERIAL
} transport_backend_t;

static constexpr transport_backend_t ZPNET_TRANSPORT = TRANSPORT_HID;

// --------------------------------------------------------------
// Base units
// --------------------------------------------------------------

static constexpr uint64_t NS_PER_SECOND      = 1000000000ULL;
static constexpr uint64_t NS_PER_MILLISECOND = 1000000ULL;
static constexpr uint64_t NS_PER_MICROSECOND = 1000ULL;

// --------------------------------------------------------------
// 10 MHz clock constants
// --------------------------------------------------------------
//
// All three external 10 MHz clocks (GNSS VCLOCK, OCXO1, OCXO2)
// share the same nominal frequency. These constants are used
// for ISR-level residual computation and PPS edge validation.
//
// NOTE:
//   GNSS VCLOCK is on QTimer1 ch0+ch1 as a dual-edge raw counter
//   (50 ns raw ticks). TimePop compare uses QTimer1 ch2.
//   OCXO1 is on GPT1.
//   OCXO2 is on GPT2.
//

// Expected ticks per PPS second at 10 MHz
static constexpr uint32_t TICKS_10MHZ_PER_SECOND = 10000000U;

// Nanoseconds per 10 MHz tick
static constexpr uint32_t NS_PER_10MHZ_TICK = 100U;

// PPS edge validation tolerance (ticks).
// Any PPS edge whose 10 MHz elapsed count modulo TICKS_10MHZ_PER_SECOND
// has a remainder outside ±PPS_VCLOCK_TOLERANCE is rejected as spurious.
static constexpr uint32_t PPS_VCLOCK_TOLERANCE = 100;

// --------------------------------------------------------------
// Prescaled clock domains
// --------------------------------------------------------------

// SmartPOP / clock prescale
static constexpr uint32_t SMARTPOP_HZ       = 10000U; // 10 kHz
static constexpr uint64_t NS_PER_SMART_TICK = NS_PER_SECOND / SMARTPOP_HZ; // 100,000 ns

// --------------------------------------------------------------
// DWT conversion (1008 MHz core clock)
// --------------------------------------------------------------

// Exact rational conversion: 1 DWT cycle = 125/126 ns
static constexpr uint64_t DWT_NS_NUM = 125ULL;
static constexpr uint64_t DWT_NS_DEN = 126ULL;

// Expected DWT cycles per PPS second at 1008 MHz
static constexpr uint32_t DWT_EXPECTED_PER_PPS = 1008000000U;

// --------------------------------------------------------------
// ARM Cortex-M7 ISR entry latency
// --------------------------------------------------------------
//
// When a hardware interrupt fires (PPS edge, GPT2 output compare),
// the ARM core saves registers and branches to the vector before
// the ISR's first instruction executes. The DWT_CYCCNT read in
// the ISR is therefore late by this many DWT cycles.
//
// Measured empirically via tdc_analyzer over 23,170 PPS edges
// (campaign Shakeout1, 6+ hours):
//
//   mean   = 52.00 cycles  (51.6 ns)
//   stddev =  1.42 cycles  ( 1.4 ns)
//   min    = 50 cycles, max = 54 cycles
//   distribution: uniform across 50-54 (5 clusters, ~20% each)
//
// This constant is used to correct ISR-captured DWT values back
// to the true event moment. It applies to:
//   - PPS anchor (dwt_cyccnt_at_pps in TIMEBASE_FRAGMENT)
//   - TimePop fire (fire_dwt_cyccnt in timepop_ctx_t)
//   - Any future ISR-captured DWT timestamp (e.g., photon detector)
//
// At 52 cycles the maximum residual error is ±2 cycles (±2 ns).
//
static constexpr uint32_t ISR_ENTRY_DWT_CYCLES = 52;

// --------------------------------------------------------------
// Serial configuration
// --------------------------------------------------------------
static const unsigned long USB_SERIAL_BAUD = 115200;
static const unsigned long GNSSDO_BAUD     = 38400;

// --------------------------------------------------------------
// Laser control pins
// --------------------------------------------------------------
static const int LD_ON_PIN          = 30;
static const int LASER_MONITOR_PIN  = 20;

// --------------------------------------------------------------
// Photodiode (TDM split-pin design)
// --------------------------------------------------------------
//
// PHOTODIODE_EDGE_PIN:
//   Fast digital edge suitable for interrupts
//
// PHOTODIODE_ANALOG_PIN:
//   Continuous analog voltage while light is present
//
static const int PHOTODIODE_EDGE_PIN   = 34;
static const int PHOTODIODE_ANALOG_PIN = 15;

// --------------------------------------------------------------
// GNSS timing pins
// --------------------------------------------------------------
//
// GNSS_PPS_PIN:
//   1 Hz pulse-per-second from GF-8802 (pin 17)
//
// GNSS_VCLK_PIN:
//   10 MHz VCLOCK square wave from GF-8802 (pin 10)
//   Counted by QTimer1 ch0+ch1 (dual-edge raw counting, 50 ns raw ticks)
//
// GNSS_LOCK_PIN:
//   GNSS lock status (true/false)
//
static const int GNSS_PPS_PIN  = 1;
static const int GNSS_VCLK_PIN = 10;
static const int GNSS_LOCK_PIN = 4;

static const int GNSS_PPS_RELAY = 32;

// --------------------------------------------------------------
// OCXO1 — AOCJY1-A #1 (original)
// --------------------------------------------------------------
//
// OCXO1_10MHZ_PIN:
//   10 MHz output from OCXO1, counted by GPT1.
//
// OCXO1_CTL_PIN:
//   PWM output (12-bit + dither) driving OCXO1 CTL input.
//   Pin 22.
//   0–3.3V for frequency trim.
//   Positive pull slope: higher voltage = higher frequency.
//
static const int OCXO1_10MHZ_PIN = 25;
static const int OCXO1_CTL_PIN   = 22;

// OCXO1 DAC defaults
static constexpr uint32_t OCXO1_DAC_DEFAULT = 2048;
static constexpr uint32_t OCXO1_DAC_MIN     = 0;
static constexpr uint32_t OCXO1_DAC_MAX     = 4095;

// --------------------------------------------------------------
// OCXO2 — AOCJY1-A #2 (second oscillator)
// --------------------------------------------------------------
//
// OCXO2_10MHZ_PIN:
//   10 MHz output from OCXO2, counted by GPT2 on pin 14.
//
// OCXO2_CTL_PIN:
//   PWM output (12-bit + dither) driving OCXO2 CTL input.
//   Pin 5.
//   0–3.3V for frequency trim.
//   Positive pull slope: higher voltage = higher frequency.
//
static const int OCXO2_10MHZ_PIN = 14;
static const int OCXO2_CTL_PIN   = 5;

// OCXO2 DAC defaults (identical range to OCXO1)
static constexpr uint32_t OCXO2_DAC_DEFAULT = 2048;
static constexpr uint32_t OCXO2_DAC_MIN     = 0;
static constexpr uint32_t OCXO2_DAC_MAX     = 4095;

// --------------------------------------------------------------
// QTimer1 cascade configuration
// --------------------------------------------------------------
//
// QTimer1 is used for GNSS VCLOCK counting and compare.
//
// ch0: primary external count source on pin 10 (GNSS 10 MHz)
// ch1: cascaded extension for 32-bit range
// ch2: TimePop compare doorbell (low-word compare; full 32-bit qualified in software)
// ch3: reserved for VCLOCK_TEST / historical compare validation
//
// NOTE:
//   The raw QTimer count may reflect dual-edge behavior depending on the
//   configured input mode. Software consuming this domain must account
//   for that interpretation explicitly.
//
// The 32-bit QTimer value is read in the PPS ISR alongside
// GPT1_CNT, GPT2_CNT, and DWT_CYCCNT.
//
static constexpr uint32_t QTIMER1_CH0_BITS = 16;
static constexpr uint32_t QTIMER1_CH0_MASK = 0xFFFF;

// --------------------------------------------------------------
// Timer hardware summary
// --------------------------------------------------------------
//
// Domain    Timer           Pin   Clock Source           Resolution
// -------   -------------   ---   --------------------   ----------
// DWT       ARM_DWT_CYCCNT   —    CPU core (1008 MHz)    ~1 ns
// GNSS      QTimer1 ch0+1    10   GF-8802 VCLOCK 10 MHz  50 ns raw
// OCXO1     GPT1             25   AOCJY1-A #1   10 MHz   100 ns
// OCXO2     GPT2             14   AOCJY1-A #2   10 MHz   100 ns
//
// All timing domains free-run continuously...
// All are captured simultaneously in the PPS ISR.
// GPT1 and GPT2 are 32-bit native.
// QTimer1 is 16-bit cascaded to 32-bit.
// 64-bit extension is via delta accumulation where needed.
//
// GNSS VCLOCK is intentionally hosted on QTimer1 because it is the
// sovereign clock domain used by TimePop and broader timing semantics.
// OCXO2 is hosted on GPT2.
//

// --------------------------------------------------------------
// Event bus sizing (authoritative)
// --------------------------------------------------------------
//
// These define the maximum durable telemetry backlog.
// Events are small, frequent, and cheap.
// Large payloads belong in request/response, not events.
//
// Overflow results in dropped events (explicitly observable).
//
static constexpr size_t EVT_MAX      = 128;
static constexpr size_t EVT_TYPE_MAX = 32;
static constexpr size_t EVT_BODY_MAX = 512;

// --------------------------------------------------------------
// GNSS ingestion limits
// --------------------------------------------------------------
static const size_t GNSS_LINE_MAX                = 192;
static const unsigned long GNSS_SILENCE_FLUSH_MS = 50;

// --------------------------------------------------------------
// GNSS PPS telemetry
// --------------------------------------------------------------
//
// PPS events are rate-limited to avoid serial flooding.
//
static const uint32_t PPS_EMIT_INTERVAL_MS = 10000;

// --------------------------------------------------------------
// Photodiode episode detection parameters
// --------------------------------------------------------------
//
// Hysteresis thresholds (volts)
//
static const float PHOTODIODE_ON_THRESHOLD_V  = 0.20f;
static const float PHOTODIODE_OFF_THRESHOLD_V = 0.05f;

//
// Darkness stability window (milliseconds)
//
static const uint32_t PHOTODIODE_OFF_STABLE_MS = 20;

// --------------------------------------------------------------
// ADC scaling assumptions
// --------------------------------------------------------------
static const float ADC_FS_VOLTS  = 3.3f;
static const float ADC_FS_COUNTS = 4095.0f;