#pragma once

#include <cstdint>
#include <cstddef>

#define ZPNET_SERIAL Serial
#define FW_VERSION_STR "zpnet-teensy-5.1.2"
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
// Prescaled clock domains
// --------------------------------------------------------------

// SmartPOP / clock prescale
static constexpr uint32_t SMARTPOP_HZ        = 10000U;      // 10 kHz
static constexpr uint64_t NS_PER_SMART_TICK  = NS_PER_SECOND / SMARTPOP_HZ; // 100,000 ns

// --------------------------------------------------------------
// DWT conversion (600 MHz core clock)
// --------------------------------------------------------------

// Exact rational conversion: 1 DWT cycle = 125/126:
static constexpr uint64_t DWT_NS_NUM = 125ULL;
static constexpr uint64_t DWT_NS_DEN = 126ULL;

// --------------------------------------------------------------
// ARM Cortex-M7 ISR entry latency
// --------------------------------------------------------------
//
// When a hardware interrupt fires (PPS edge, GPT2 output compare),
// the ARM core saves registers and branches to the vector before
// the ISR's first instruction executes.  The DWT_CYCCNT read in
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
// to the true event moment.  It applies to:
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
static const int LD_ON_PIN = 30;
static const int LASER_MONITOR_PIN = 20;

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
//   1 Hz pulse-per-second
//
// GNSS_VCLK_PIN:
//   10 MHz VCLOCK square wave
//
// GNSS_LOCK_PIN:
//   GNSS lock status (true/false)
//
static const int GNSS_PPS_PIN  = 1;
static const int GNSS_VCLK_PIN = 14;
static const int GNSS_LOCK_PIN = 4;

static const int GNSS_PPS_RELAY = 32;

// --------------------------------------------------------------
// OCXO control pin (DAC output)
// --------------------------------------------------------------
//
// OCXO_CTL_PIN:
//   Teensy DAC output (A22, pin 22) driving AOCJY1-A Pin 1 (CTL).
//   0–3.3V analog output for frequency trim.
//   12-bit resolution (0–4095).
//   Positive pull slope: higher voltage = higher frequency.
//
static const int OCXO_CTL_PIN = 22;

// Default DAC value at boot (midpoint = ~1.65V)
static constexpr uint32_t OCXO_DAC_DEFAULT = 2048;

// DAC limits
static constexpr uint32_t OCXO_DAC_MIN = 0;
static constexpr uint32_t OCXO_DAC_MAX = 4095;

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
static constexpr size_t EVT_MAX       = 128;
static constexpr size_t EVT_TYPE_MAX  = 32;
static constexpr size_t EVT_BODY_MAX  = 512;

// --------------------------------------------------------------
// GNSS ingestion limits
// --------------------------------------------------------------
static const size_t GNSS_LINE_MAX             = 192;
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

// --------------------------------------------------------------
// TIMEPULSE 10 KHz relay and quantized clock constants
// --------------------------------------------------------------

static const int GNSS_10KHZ_RELAY = 9;

// GPT2 output compare half-period: 500 GNSS ticks = 50 µs.
// The ISR toggles on each match, producing a 10 KHz square wave
// (full period = 1,000 GNSS ticks = 100 µs).
// TIMEPULSE advances the quantized clock on the rising edge only.
static constexpr uint32_t TIMEPULSE_GNSS_HALF_PERIOD = 500;

// Full period in GNSS ticks (diagnostic / documentation only).
static constexpr uint32_t TIMEPULSE_GNSS_FULL_PERIOD = 1000;

// Nanoseconds per TIMEPULSE tick.
// 100 µs = 100,000 ns.  10,000 ticks × 100,000 ns = 1 second.
static constexpr uint64_t TIMEPULSE_NS_PER_TICK = 100000;