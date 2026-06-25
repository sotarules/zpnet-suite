#pragma once

#include <cstdint>
#include <cstddef>

#define ZPNET_SERIAL Serial
#define FW_VERSION_STR "zpnet-teensy-5.2.0"
static constexpr const char* FW_VERSION = FW_VERSION_STR;

// -------------------------------------------------------------
// Transport doctrine
// -------------------------------------------------------------
//
// Teensy firmware is USB CDC Serial only.  RawHID/HID transport selection
// has been retired; transport.cpp owns the single physical transport.

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
// All three domains count at 10 MHz (100 ns per tick).  Clock lanes now use
// the pin-bound same-channel doctrine: the channel that receives the physical
// clock input also owns compare-match event custody.
//   GNSS VCLOCK on QTimer1 CH0, pin 10 (count + compare)
//   OCXO1      on QTimer2 CH0, pin 13 (count + compare)
//   OCXO2      on QTimer3 CH3, pin 15 (count + compare)
//   TimePop    on QTimer1 CH2 as scheduler only
//

// Expected ticks per PPS second at 10 MHz
static constexpr uint32_t TICKS_10MHZ_PER_SECOND = 10000000U;

// Nanoseconds per 10 MHz tick
static constexpr uint32_t NS_PER_10MHZ_TICK = 100U;

// PPS edge validation tolerance (ticks).
// Any PPS edge whose 10 MHz elapsed count modulo TICKS_10MHZ_PER_SECOND
// has a remainder outside ±PPS_VCLOCK_TOLERANCE is rejected as spurious.
static constexpr uint32_t PPS_VCLOCK_TOLERANCE = 100;

static constexpr uint32_t VCLOCK_COUNTS_PER_SECOND = 10000000U;

// --------------------------------------------------------------
// Prescaled clock domains
// --------------------------------------------------------------

// SmartPOP constants retained for reference.
// SmartPOP functionality is subsumed by TimePop v8.0's phase-locked
// priority queue scheduler.
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
static const int PHOTODIODE_ANALOG_PIN = 22;

// --------------------------------------------------------------
// GNSS timing pins
// --------------------------------------------------------------
//
// GNSS_PPS_PIN:
//   1 Hz pulse-per-second from GF-8802 (pin 17)
//
// GNSS_VCLK_PIN:
//   10 MHz VCLOCK square wave from GF-8802 (pin 10)
//   Counted and compared by QTimer1 CH0 (single-edge, 100 ns per tick)
//
// GNSS_LOCK_PIN:
//   GNSS lock status (true/false)
//
static const int GNSS_PPS_PIN  = 1;
static const int GNSS_VCLK_PIN = 10;
static const int GNSS_LOCK_PIN = 4;

static const int GNSS_PPS_RELAY = 32;

static constexpr int OCXO1_PIN = 13;
static constexpr int OCXO2_PIN = 15;

// --------------------------------------------------------------
// QTimer1 channel configuration
// --------------------------------------------------------------
//
// QTimer1 hosts two deliberately separate duties.
//
// ch0: VCLOCK pin-bound clock lane on pin 10: count + compare authority
// ch1: retired / reserved; not timing authority
// ch2: TimePop dynamic compare scheduler only; not VCLOCK authority
// ch3: reserved diagnostic/time-test rail if needed
//
// The raw QTimer count is in 10 MHz ticks (100 ns per tick).
// No domain translation is required — one tick = one GNSS cycle.
//
// The synthetic 32-bit VCLOCK identity is reconstructed from QTimer1 CH0;
// sibling channels are not used to author VCLOCK timing facts.
//
static constexpr uint32_t QTIMER1_CH0_BITS = 16;
static constexpr uint32_t QTIMER1_CH0_MASK = 0xFFFF;

// --------------------------------------------------------------
// Timer hardware summary
// --------------------------------------------------------------
//
// Domain    Timer/Channel    Pin   Clock Source           Resolution
// -------   ---------------  ---   --------------------   ----------
// DWT       ARM_DWT_CYCCNT    —    CPU core (1008 MHz)    ~1 ns
// GNSS      QTimer1 CH0       10   GF-8802 VCLOCK 10 MHz  100 ns
// OCXO1     QTimer2 CH0       13   AOCJY1-A #1   10 MHz   100 ns
// OCXO2     QTimer3 CH3       15   AOCJY1-A #2   10 MHz   100 ns
// TimePop   QTimer1 CH2       —    scheduler rail         100 ns domain
//
// Clock lanes use same-channel count + compare custody.  The scheduler rail
// may wake callbacks but must not author VCLOCK edge identity.
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
// ADC scaling assumptions:
// --------------------------------------------------------------
static const float ADC_FS_VOLTS  = 3.3f;
static const float ADC_FS_COUNTS = 4095.0f;

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;

static constexpr uint32_t VCLOCK_INTERVAL_COUNTS = 10000U;  // 1 ms at 10 MHz

// OCXO cadence constants retained for acquisition/report compatibility.
// Steady-state OCXO publication is a one-second compare on each lane's
// pin-bound channel; SmartZero may still use short acquisition samples.
static constexpr uint32_t OCXO_INTERVAL_COUNTS = 20000U;       // 2 ms at 10 MHz
static constexpr uint32_t OCXO_COUNTS_PER_SECOND = 10000000U;
static constexpr uint32_t OCXO_PHASE_OFFSET_TICKS =
    OCXO_INTERVAL_COUNTS / 2U;                                 // 1 ms

// Ticks per emitted one-second event.  Lane-specific because OCXO and VCLOCK
// run on different cadences.
static constexpr uint32_t TICKS_PER_SECOND_EVENT      = 1000U;  // VCLOCK: 1 kHz
static constexpr uint32_t TICKS_PER_SECOND_EVENT_OCXO = 500U;   // OCXO:   500 Hz

// ============================================================================
// Latency constants
// ============================================================================

static constexpr uint32_t DWT_CYCCNT_TO_MEMORY_CYCLES = 1;
static constexpr uint32_t WITNESS_STIMULATE_LATENCY = 5;
static constexpr uint32_t QTIMER_READ_LATENCY = 39;

static constexpr uint32_t GPIO_TOTAL_LATENCY = 63;
static constexpr uint32_t QTIMER_TOTAL_LATENCY = 46;

static constexpr uint32_t PPS_ISR_ENTRY_OVERHEAD    = 45;
static constexpr uint32_t QTIMER_ISR_ENTRY_OVERHEAD = 19;

// Observed canonical VCLOCK epoch offset relative to raw PPS ISR entry.
//
// After redefining the canonical PPS epoch as the first VCLOCK-domain edge
// selected by the physical PPS pulse, raw_cycles.py consistently shows:
//
//   canonical_dwt_at_pps - raw_pps_isr_entry_dwt ≈ +34 cycles
//
// This is an empirical system calibration constant, not a fully decomposed
// latency model.  Witness measurements predict part of this offset from the
// GPIO-vs-QTimer sink-path latency difference, but leave a small stable
// remainder (~15 cycles) that may come from GF-8802 PPS/VCLOCK phase skew,
// input synchronizers, pad routing, QTimer compare maturation semantics, or
// other fixed hardware path differences.
//
// Priority testing showed the value remains ~34 cycles even when QTimer1 and
// PPS GPIO are both set to NVIC priority 0, so this does not appear to be a
// QTimer IRQ priority artifact.
//
// Treat this as the observed calibration offset between the physical/raw PPS
// interrupt timestamp and the canonical VCLOCK-selected PPS epoch.  Re-measure
// if the GF-8802 wiring, pin routing, ISR paths, QTimer setup, optimization
// level, or latency correction model changes.
static constexpr int32_t CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES = 34;
