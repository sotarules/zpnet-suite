#pragma once

#include <cstdint>
#include <cstddef>

#define ZPNET_SERIAL Serial

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
//   GNSS VCLOCK on QTimer1 ch0 low-word (single-edge, CM=1)
//   OCXO1 on QTimer2 ch0             (single-edge, CM=1)
//   OCXO2 on QTimer3 ch3             (single-edge, CM=1)
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

static_assert(TICKS_10MHZ_PER_SECOND ==
              (NS_PER_SECOND / NS_PER_10MHZ_TICK),
              "10 MHz tick geometry broken");
static_assert(VCLOCK_COUNTS_PER_SECOND == TICKS_10MHZ_PER_SECOND,
              "VCLOCK must remain a 10 MHz clock");

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
//   Counted by QTimer1 ch0 low-word (single-edge, 100 ns per tick)
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
// QTimer1 VCLOCK / TimePop configuration
// --------------------------------------------------------------
//
// QTimer1 hosts the sovereign GNSS VCLOCK domain.  process_interrupt owns
// the hardware registers and presents synthetic 32-bit VCLOCK identities
// to TimePop and CLOCKS.
//
// ch0: primary external count source on pin 10 (GNSS 10 MHz, CM=1)
//      This is the passive VCLOCK low-word counter.
// ch1: hosted VCLOCK-domain compare rail for clients such as WITNESS.
//      It is not a cascade high word.
// ch2: TimePop dynamic compare scheduler and CADENCE_MINDER rail (CM=1).
//      VCLOCK one-second bookends are authored from this proven TimePop path.
// ch3: reserved/off in the current interrupt architecture.
//
// The raw QTimer count is in 10 MHz ticks (100 ns per tick).
// No domain translation is required — one tick = one GNSS cycle.
//
// VCLOCK 32-bit identity is synthetic and process_interrupt-owned.  The PPS
// ISR samples QTimer1 CH0 low-word state; CADENCE_MINDER refreshes the
// synthetic VCLOCK anchor on the TimePop/CH2 rail.
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
// GNSS      QTimer1 ch0      10   GF-8802 VCLOCK 10 MHz  100 ns
// OCXO1     QTimer2 ch0      13   AOCJY1-A #1   10 MHz   100 ns
// OCXO2     QTimer3 ch3      15   AOCJY1-A #2   10 MHz   100 ns
//
// All timing domains free-run continuously.  The PPS ISR samples the lane
// low words in one custody window when they are initialized, but those raw
// 16-bit reads are forensic evidence, not public timing authority.
//
// QTimer channels are 16-bit native.  Synthetic 32-bit counter32 extension is
// owned by process_interrupt, but custody is intentionally asymmetric:
//
//   VCLOCK : TimePop/CADENCE_MINDER on QTimer1 CH2 refreshes the sovereign
//            VCLOCK synthetic anchor and authors VCLOCK/PPS bookends.
//   OCXO1  : QTimer2 CH0 owns a private 1 kHz mini-scheduler.
//   OCXO2  : QTimer3 CH3 owns a private 1 kHz mini-scheduler.
//
// OCXO lanes do not use TimePop for cadence, compare arming, rollover
// custody, or post-ISR dispatch.  Each OCXO lane advances its own compare
// target by exactly 10,000 OCXO ticks per fire and emits a one-second edge
// every 1,000 cadence fires.
//
// GNSS VCLOCK is intentionally hosted on QTimer1 because it is the sovereign
// clock domain used by TimePop and broader timing semantics.  OCXO1 and OCXO2
// are hosted on QTimer2 and QTimer3 respectively; this matches the NVIC
// vector layout consumed by process_interrupt and the current priority
// architecture below (QTimer1/PPS=0, OCXO=16).
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

// VCLOCK substrate cadence.  This is the TimePop/CADENCE_MINDER rail.
static constexpr uint32_t VCLOCK_INTERVAL_COUNTS = 10000U;  // 1 ms at 10 MHz

// OCXO lane-local mini-scheduler cadence.
//
// Each OCXO lane now owns its own QTimer compare chain.  The compare target
// advances by exactly 10,000 lane-local OCXO ticks per fire.  Most fires only
// maintain 16-bit-to-32-bit custody; every 1,000 epoch-aligned fires authors
// the OCXO one-second edge delivered to CLOCKS/Alpha.
static constexpr uint32_t OCXO_INTERVAL_COUNTS = 10000U;       // 1 ms at 10 MHz
static constexpr uint32_t OCXO_COUNTS_PER_SECOND = TICKS_10MHZ_PER_SECOND;

// Retired compatibility surface.  The new OCXO schedulers do not impose an
// artificial phase offset between OCXO1 and OCXO2.  Their relative phase is
// whatever the hardware oscillators and CLOCKS-selected zero offsets say it is.
static constexpr uint32_t OCXO_PHASE_OFFSET_TICKS = 0U;

// Cadence fires per emitted one-second event.
static constexpr uint32_t TICKS_PER_SECOND_EVENT =
    VCLOCK_COUNTS_PER_SECOND / VCLOCK_INTERVAL_COUNTS;
static constexpr uint32_t TICKS_PER_SECOND_EVENT_OCXO =
    OCXO_COUNTS_PER_SECOND / OCXO_INTERVAL_COUNTS;

static_assert(TICKS_PER_SECOND_EVENT == 1000U,
              "VCLOCK cadence must remain 1 kHz");
static_assert(TICKS_PER_SECOND_EVENT_OCXO == 1000U,
              "OCXO mini-scheduler cadence must remain 1 kHz");
static_assert((VCLOCK_COUNTS_PER_SECOND % VCLOCK_INTERVAL_COUNTS) == 0U,
              "VCLOCK one-second grid must be an integer number of cadence fires");
static_assert((OCXO_COUNTS_PER_SECOND % OCXO_INTERVAL_COUNTS) == 0U,
              "OCXO one-second grid must be an integer number of cadence fires");

// ============================================================================
// Latency constants
// ============================================================================
//
// Calibration regime note:
//
//   The current asymmetric interrupt architecture is:
//
//     QTimer1   : 0    (VCLOCK / TimePop / CADENCE_MINDER sovereign rail)
//     PPS GPIO  : 0    (physical PPS witness + VCLOCK selector)
//     OCXO 1/2  : 16   (lane-local QTimer compare mini-schedulers)
//
//   QTimer1 and PPS GPIO remain same-priority peers; same-priority interrupts
//   on Cortex-M do not preempt one another.  OCXO lanes are lower priority and
//   may be preempted by the sovereign VCLOCK/PPS rail.
//
//   The values in this section are retained from measurement.  The
//   first-instruction ISR-entry latencies (PPS_ISR_ENTRY_OVERHEAD,
//   QTIMER_ISR_ENTRY_OVERHEAD) and the hardware sink-path latencies
//   (GPIO_TOTAL_LATENCY, QTIMER_TOTAL_LATENCY, QTIMER_READ_LATENCY,
//   WITNESS_STIMULATE_LATENCY, DWT_CYCCNT_TO_MEMORY_CYCLES) should be
//   spot-checked after interrupt-priority or QTimer setup changes.  If any
//   constant shifts by more than one or two cycles, re-measure and update it;
//   do not speculatively adjust the numbers without measurement.
//
//   Known first-cut measurement artifact: QuadTimer event DWT captures can
//   appear on a 4-cycle lattice.  The OCXO mini-scheduler intentionally emits
//   honest latency-adjusted ISR-entry facts for now.  Later estimator work may
//   use the full 1,000-cadence-edge history inside each OCXO second to
//   neutralize that quantization.

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
// Current-priority calibration note:
//   QTimer1 and PPS GPIO are both priority-0 peers in the current asymmetric
//   architecture.  Same-priority peers do not preempt one another, so this
//   constant should remain in the same measurement family as the historical
//   priority-0 peer tests.  It still deserves a spot-check against post-flash
//   raw_cycles.py output after any QTimer setup, optimization, wiring, or NVIC
//   priority change.  If it shifts by more than a cycle or two, re-measure and
//   update.
//
// Treat this as the observed calibration offset between the physical/raw PPS
// interrupt timestamp and the canonical VCLOCK-selected PPS epoch.  Re-measure
// if the GF-8802 wiring, pin routing, ISR paths, QTimer setup, optimization
// level, latency correction model, or NVIC priority architecture changes.
static constexpr int32_t CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES = 34;