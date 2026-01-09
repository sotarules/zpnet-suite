#pragma once

// --------------------------------------------------------------
// Serial configuration
// --------------------------------------------------------------
static const unsigned long USB_SERIAL_BAUD = 115200;
static const unsigned long GNSSDO_BAUD     = 38400;

// --------------------------------------------------------------
// Laser control pins
// --------------------------------------------------------------
//
// EN_PIN:
//   Enables laser power stage
//
// LD_ON_PIN:
//   Controls laser emission
//
static const int EN_PIN    = 20;
static const int LD_ON_PIN = 21;

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
static const int PHOTODIODE_EDGE_PIN   = 14;
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
static const int GNSS_PPS_PIN  = 33;
static const int GNSS_VCLK_PIN = 9;

// --------------------------------------------------------------
// Event bus sizing
// --------------------------------------------------------------
//
// These define the maximum durable telemetry backlog.
// Overflow results in dropped events, counted explicitly.
//
static const size_t EVT_MAX       = 128;
static const size_t EVT_TYPE_MAX  = 32;
static const size_t EVT_BODY_MAX  = 512;

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
