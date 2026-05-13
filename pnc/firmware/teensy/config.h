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
// All three domains count at 10 MHz (100 ns per tick):
//   GNSS VCLOCK on QTimer1 ch0+ch1 (single-edge, CM=1)
//   OCXO1 on QTimer2 ch0   (single-edge, CM=1)
//   OCXO2 on QTimer3 ch3   (single-edge, CM=1)
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
//   Counted by QTimer1 ch0+ch1 (single-edge, 100 ns per tick)
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
// QTimer1 cascade configuration
// --------------------------------------------------------------
//
// QTimer1 is used for GNSS VCLOCK counting and scheduling.
//
// ch0: primary external count source on pin 10 (GNSS 10 MHz, CM=1)
// ch1: cascaded extension for 32-bit range (CM=7)
// ch2: TimePop dynamic compare scheduler (priority queue, CM=1)
// ch3: TIME_TEST compare (VCLOCK edge capture for time audit)
//
// The raw QTimer count is in 10 MHz ticks (100 ns per tick).
// No domain translation is required — one tick = one GNSS cycle.
//
// The 32-bit QTimer value is read in the PPS ISR alongside the
// OCXO1 and OCXO2 QTimer counter values and DWT_CYCCNT.
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
// GNSS      QTimer1 ch0+1    10   GF-8802 VCLOCK 10 MHz  100 ns
// OCXO1     QTimer2 ch0      13   AOCJY1-A #1   10 MHz   100 ns
// OCXO2     QTimer3 ch3      15   AOCJY1-A #2   10 MHz   100 ns
//
// All timing domains free-run continuously.
// All are captured simultaneously in the PPS ISR.
// QTimer1, QTimer2, and QTimer3 channels are 16-bit native; lane synthetic
// 32-bit counter32 extension is owned by process_interrupt and tended by
// the TimePop CADENCE_MINDER recurring ISR slot on QTimer1 CH2.
// 64-bit extension is via delta accumulation where needed.
//
// GNSS VCLOCK is intentionally hosted on QTimer1 because it is the sovereign
// clock domain used by TimePop and broader timing semantics.  OCXO1 and OCXO2
// are hosted on QTimer2 and QTimer3 respectively; this matches the NVIC
// vector layout consumed by process_interrupt and is consistent with the
// priority architecture below (PPS=0, OCXO=16, QTimer1=32).
//
// Note: earlier versions of this header described OCXO1/OCXO2 as hosted on
// GPT1/GPT2.  Those references were vestigial — the system migrated to
// QTimer2/QTimer3 some time ago.  The current QTimer-based wiring is the
// authoritative layout.
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

// OCXO cadence (deliberately distinct from VCLOCK).
//
// Experimental config: each OCXO lane fires every 2 ms (20000 OCXO ticks).
// 500 compares per OCXO second × 20000 ticks = 10,000,000 ticks = 1 s.
//
// Phase offset between the two lanes is OCXO_INTERVAL_COUNTS / 2 (1 ms).
// OCXO1 fires on even ms boundaries, OCXO2 fires on odd ms boundaries.
// They cannot share an ISR window.
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
//
// Calibration regime note (Patch 8):
//
//   Every constant in this section was measured under the pre-Patch-8 NVIC
//   priority arrangement, in which QTimer1 and PPS GPIO were both at priority
//   0 (peers) and QTimer2/QTimer3 (OCXO) were at priority 16.  Under that
//   scheme PPS and QTimer1 could not preempt each other (same-priority
//   interrupts on Cortex-M do not preempt), and OCXO ISRs were preemptible
//   by both.
//
//   The Patch 8 priority architecture is:
//
//     PPS GPIO  : 0    (sovereign labeling witness, uncontested)
//     OCXO 1/2  : 16   (physical hardware compare events)
//     QTimer1   : 32   (TimePop bookkeeping + CADENCE_MINDER)
//
//   The values in this section are retained at their pre-Patch-8 measurements
//   and are NOT being modified by Patch 8.  The first-instruction ISR-entry
//   latencies (PPS_ISR_ENTRY_OVERHEAD, QTIMER_ISR_ENTRY_OVERHEAD) and the
//   hardware sink-path latencies (GPIO_TOTAL_LATENCY, QTIMER_TOTAL_LATENCY,
//   QTIMER_READ_LATENCY, WITNESS_STIMULATE_LATENCY, DWT_CYCCNT_TO_MEMORY_CYCLES)
//   should not be priority-sensitive: same-priority peers do not preempt each
//   other, so the ISR-entry path was already operating in an "uncontested
//   peer" regime in the pre-Patch-8 arrangement, and that regime is preserved
//   (and slightly strengthened, since PPS is now fully sovereign).
//
//   Spot-check after the Patch 8 flash by comparing post-flash measurements
//   against the pre-flash baseline.  If any of these constants shifts by more
//   than one or two cycles, re-measure and update the constant.  Do not
//   speculatively adjust the numbers without measurement.

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
// Historical observation (pre-Patch-8):
//   Priority testing under the pre-Patch-8 priority arrangement showed the
//   value remained ~34 cycles even when QTimer1 and PPS GPIO were both set
//   to NVIC priority 0.  Under that arrangement this constant did not appear
//   to be a QTimer IRQ priority artifact.
//
// Patch 8 calibration-regime note:
//   The Patch 8 priority architecture (PPS=0, OCXO=16, QTimer1=32) is a
//   different regime than the one in which the observation above was made.
//   The historical priority-invariance result was specifically about moving
//   QTimer1 from one priority down to priority 0 to match PPS — a change
//   that, in the pre-Patch-8 world, made QTimer1 a peer of PPS rather than
//   a lower-priority preemptee.  Patch 8 moves QTimer1 in the opposite
//   direction (to priority 32), so the prior priority-invariance result
//   does not strictly cover the new regime.
//
//   The first-instruction ISR-entry latency for the PPS GPIO ISR should not
//   change under Patch 8, because PPS was already running uncontested by
//   QTimer1 (same-priority peers do not preempt) and is now even more
//   sovereign.  But this constant deserves a spot-check against post-flash
//   raw_cycles.py output.  If it has shifted by more than a cycle or two,
//   re-measure and update.
//
// Treat this as the observed calibration offset between the physical/raw PPS
// interrupt timestamp and the canonical VCLOCK-selected PPS epoch.  Re-measure
// if the GF-8802 wiring, pin routing, ISR paths, QTimer setup, optimization
// level, latency correction model, or NVIC priority architecture changes.
static constexpr int32_t CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES = 34;