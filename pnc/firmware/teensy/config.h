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
static constexpr uint64_t NS_PER_TICK        = 100ULL;

// --------------------------------------------------------------
// 10 MHz clock constants
// --------------------------------------------------------------
//
// All three external 10 MHz clocks (GNSS VCLOCK, OCXO1, OCXO2)
// share the same nominal frequency. These constants are used
// for ISR-level residual computation and PPS edge validation.
//
// All three domains count at 10 MHz (100 ns per tick):
//   GNSS VCLOCK on QTimer1 (sourced via CH0; cadence on CH3)
//   OCXO1 on QTimer3 CH2
//   OCXO2 on QTimer3 CH3
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
//   Sourced into QTimer1 CH0 as the primary count source for the
//   QTimer1 channels (single-edge, 100 ns per tick).
//
// GNSS_LOCK_PIN:
//   GNSS lock status (true/false)
//
static const int GNSS_PPS_PIN  = 1;
static const int GNSS_VCLK_PIN = 10;
static const int GNSS_LOCK_PIN = 4;

static const int GNSS_PPS_RELAY = 32;

static constexpr int OCXO1_PIN = 14;
static constexpr int OCXO2_PIN = 15;

// --------------------------------------------------------------
// QTimer1 channel assignments
// --------------------------------------------------------------
//
// QTimer1 hosts the GNSS-disciplined 10 MHz VCLOCK domain.  It is the
// sovereign timer for TimePop scheduling and broader timing semantics.
//
// All four channels run on the same primary count source (the GF-8802
// VCLOCK on pin 10), routed in via CH0.  This was previously a 16-bit
// cascade (CH0 + CH1 → 32-bit count) but the cascade has been retired:
// 32-bit ledger coordinates are now authored synthetically in
// process_interrupt rather than read from cascaded hardware.
//
// Current per-channel role:
//
//   ch0: Primary external count source on pin 10 (GNSS 10 MHz, CM=1).
//        All other QTimer1 channels share this tick axis.
//
//   ch1: PPS↔VCLOCK irreducible-offset measurement (CM=1, one-shot
//        compare).  Armed inside the PPS GPIO ISR to fire on the next
//        VCLOCK rising edge; the resulting timestamp is differenced
//        against the latency-compensated PPS-edge DWT to empirically
//        witness the irreducible hardware-path offset between two
//        physically synchronous events.  See process_interrupt's
//        REPORT and CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES.
//
//   ch2: TimePop dynamic compare scheduler — precision/SpinDry actuator
//        (CM=1, dynamically armed).
//
//   ch3: VCLOCK cadence — 1 kHz TICK + 1 Hz VCLOCK one-second event
//        (CM=1, recurring compare).
//
// The raw QTimer count is in 10 MHz ticks (100 ns per tick).  No domain
// translation is required — one tick = one GNSS VCLOCK cycle.
//
// Each individual channel is 16 bits.  The synthetic 32-bit event-ledger
// counters (g_vclock_count32, g_ocxo1_count32, g_ocxo2_count32) are
// authored in software by the cadence ISRs in process_interrupt and are
// the only 32-bit counter values consumed downstream.
//
static constexpr uint32_t QTIMER1_CH0_BITS = 16;
static constexpr uint32_t QTIMER1_CH0_MASK = 0xFFFF;

// --------------------------------------------------------------
// Timer hardware summary
// --------------------------------------------------------------
//
// Domain    Timer            Pin   Clock Source           Resolution
// -------   --------------   ---   --------------------   ----------
// DWT       ARM_DWT_CYCCNT    —    CPU core (1008 MHz)    ~1 ns
// VCLOCK    QTimer1 ch0+CH3   10   GF-8802 VCLOCK 10 MHz  100 ns
// OCXO1     QTimer3 ch2       14   AOCJY1-A #1   10 MHz   100 ns
// OCXO2     QTimer3 ch3       15   AOCJY1-A #2   10 MHz   100 ns
//
// All timing domains free-run continuously.
// All authored cadence events are dispatched as interrupt_event_t
// payloads by process_interrupt — see process_interrupt.h.
//
// QTimer hardware channels are 16 bits.  The 32-bit event-ledger
// coordinates published downstream are synthetic — authored in
// software by the cadence ISRs and advanced by gear arithmetic, not
// read from cascaded hardware.
//
// VCLOCK is intentionally hosted on QTimer1 because it is the
// sovereign clock domain used by TimePop and broader timing semantics.
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

static constexpr uint32_t OCXO_INTERVAL_COUNTS = 10000U;    // 1 ms at 10 MHz
static constexpr uint32_t OCXO_COUNTS_PER_SECOND = 10000000U;

// Ticks per emitted one-second event.  1000 ticks × 1 ms = 1 s.
static constexpr uint32_t TICKS_PER_SECOND_EVENT = 1000U;

// ============================================================================
// Latency constants
// ============================================================================
//
// These constants come from the dedicated hardware witness experiments in
// process_interrupt's WITNESS_LATENCY mode.  They are expressed in DWT cycles
// at 1.008 GHz and represent our current best empirical floor estimates for
// the path from a real electrical event to ISR entry.
//
// IMPORTANT DISTINCTIONS
//
// 1. WITNESS_STIMULATE_LATENCY
//    This is the measured source-side cost of stimulating the witness signal:
//    the DWT cycles between the pre-write timestamp and the completion of the
//    digitalWriteFast(HIGH) action used to create the witness edge.
//
//    In other words, this is the cost of "causing" the witness edge in
//    software.  It is small (~5 cycles) and is INCLUDED in the total witness
//    latency measurements below.
//
// 2. GPIO_TOTAL_LATENCY
//    This is the measured total latency from the witness source timestamp
//    (taken immediately before digitalWriteFast(HIGH)) to entry into the GPIO
//    interrupt handler on the receiving pin.
//
//    Because the source timestamp is taken BEFORE the witness pin is driven
//    high, this total includes both:
//      • WITNESS_STIMULATE_LATENCY
//      • the sink-side GPIO path latency
//
//    Therefore, GPIO_TOTAL_LATENCY is a source-to-ISR-entry constant, not a
//    pure electrical-edge-to-ISR-entry constant.  To approximate the latter,
//    subtract WITNESS_STIMULATE_LATENCY.
//
// 3. QTIMER_TOTAL_LATENCY
//    This is the measured total latency from the witness source timestamp
//    (again taken immediately before digitalWriteFast(HIGH)) to entry into the
//    QTimer compare ISR when that incoming witness edge is counted by the
//    timer hardware and matures into an interrupt.
//
//    Like GPIO_TOTAL_LATENCY, this includes the source-side stimulate cost.
//    Therefore, it is also a source-to-ISR-entry constant, not a pure
//    electrical-edge-to-ISR-entry constant.  To approximate the latter,
//    subtract WITNESS_STIMULATE_LATENCY.
//
// CURRENT INTERPRETATION
//
//   GPIO sink-only floor   ≈ GPIO_TOTAL_LATENCY   - WITNESS_STIMULATE_LATENCY
//   QTIMER sink-only floor ≈ QTIMER_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY
//
// With current measurements this means roughly:
//   GPIO sink-only floor   ≈ 72 - 5 = 67 cycles
//   QTIMER sink-only floor ≈ 53 - 5 = 48 cycles
//
// process_interrupt uses these constants directly:
//
//   • The QTimer event-coordinate adjuster subtracts QTIMER_TOTAL_LATENCY
//     − WITNESS_STIMULATE_LATENCY from raw QTimer ISR-entry DWT to land
//     on a "DWT at electrical edge" timestamp for VCLOCK / OCXO / CH1 /
//     CH2 events.
//
//   • The PPS-edge offset-measurement adjuster subtracts GPIO_TOTAL_LATENCY
//     − WITNESS_STIMULATE_LATENCY from raw PPS GPIO ISR-entry DWT to land
//     on the matching electrical-edge frame for the CH1 PPS↔VCLOCK
//     irreducible-offset measurement.
//
// DEPRECATED CONSTANTS
//
// PPS_ISR_ENTRY_OVERHEAD and QTIMER_ISR_ENTRY_OVERHEAD are older,
// partial-model constants from the earlier TDC-correction era.  They were
// useful when we did not yet have full hardware witness measurements, but
// they no longer represent the best end-to-end empirical latency model.
//
// They are retained here as documentation of the older calibration path
// only.  No firmware translation unit references them.  New "at edge"
// timing work should use GPIO_TOTAL_LATENCY and QTIMER_TOTAL_LATENCY
// (optionally adjusted by WITNESS_STIMULATE_LATENCY when the intent is to
// estimate the actual electrical edge at the Teensy pin).
//
// NOTES ON USAGE
//
// • PPS / GPIO-origin events:
//     Use GPIO_TOTAL_LATENCY when translating raw DWT ISR-entry timestamps
//     into best-guess "DWT at electrical edge" timestamps.
//
// • QTimer-origin compare events (VCLOCK / OCXO compare paths):
//     Use QTIMER_TOTAL_LATENCY when translating raw DWT ISR-entry timestamps
//     into best-guess "DWT at electrical edge" timestamps.
//
// • Counter identity is a separate issue:
//     Correcting DWT-at-edge is not the same thing as correcting the timer
//     counter value read later in the ISR.  A counter read performed after ISR
//     entry is already late by at least the sink-side latency and then by
//     whatever additional cycles the read itself costs.
//
// These numbers are empirical and may need to be re-measured if the ISR entry
// path, pin routing, optimization level, or witness logic changes.
// ============================================================================

static constexpr uint32_t WITNESS_STIMULATE_LATENCY = 5;

static constexpr uint32_t GPIO_TOTAL_LATENCY = 72;
static constexpr uint32_t QTIMER_TOTAL_LATENCY = 53;

// Deprecated — older partial-model constants from the TDC-correction era.
// No firmware translation unit references these; retained for historical
// documentation only.
static constexpr uint32_t PPS_ISR_ENTRY_OVERHEAD = 47;
static constexpr uint32_t QTIMER_ISR_ENTRY_OVERHEAD = 16;

// ============================================================================
// PPS ↔ VCLOCK canonical epoch offset (empirical)
// ============================================================================
//
// process_interrupt's PPS GPIO ISR publishes a "canonical VCLOCK-domain
// PPS epoch" DWT timestamp to alpha by adjusting raw PPS ISR-entry DWT
// by this constant:
//
//   canonical_vclock_epoch_dwt = raw_pps_isr_entry_dwt
//                              + CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES
//
// WHAT IT REPRESENTS
//
// PPS GPIO and the GF-8802's 10 MHz VCLOCK are physically synchronous —
// the same emission moment.  After full latency compensation on both
// sides (GPIO sink-path latency removed from the PPS-ISR DWT; QTimer
// sink-path latency removed from the VCLOCK-ISR DWT), the residual DWT-
// cycle difference between the two ISR timestamps is a fixed hardware-
// path artifact: GPIO sink path vs. QTimer sink path, input
// synchronizers, pad routing, GF-8802 internal PPS-to-VCLOCK skew,
// QTimer compare maturation semantics.  This constant is that residual,
// referenced into the *raw PPS ISR entry* frame so that adding it
// directly to a raw PPS ISR-entry DWT yields the canonical VCLOCK-domain
// epoch DWT consumed downstream.
//
// HOW IT WAS MEASURED
//
// process_interrupt's QTimer1 CH1 lane runs an empirical witness every
// PPS:
//
//   1. PPS GPIO ISR fires.  Capture raw PPS ISR-entry DWT, subtract the
//      GPIO sink-path latency (GPIO_TOTAL_LATENCY − WITNESS_STIMULATE_
//      LATENCY) to produce the latency-compensated PPS-edge DWT.
//   2. Arm CH1 to fire on the next VCLOCK rising edge.
//   3. CH1 fires.  Capture raw QTimer ISR-entry DWT, subtract the QTimer
//      sink-path latency (QTIMER_TOTAL_LATENCY − WITNESS_STIMULATE_
//      LATENCY) to produce the latency-compensated VCLOCK-edge DWT.
//   4. Compute (vclock_dwt − pps_dwt) mod 100.  The mod-100 absorbs the
//      case where firmware caught the second or third VCLOCK edge instead
//      of the immediate next one (a VCLOCK period is ~100,800 DWT cycles;
//      mod 100 strips the integer-period component cleanly).
//   5. Update Welford state across many PPS edges.
//
// HISTORY
//
// This constant was previously set to +34 cycles, hand-tuned during the
// TDC-correction era so that downstream residuals zeroed.  That tuning
// was tied to a different reference frame and a different set of
// adjustments along the path; it does not contradict the current value.
// When the live CH1 measurement was reinstated and the PPS / VCLOCK
// timestamps were referenced to a common electrical-edge frame for the
// first time, the empirical offset converged to ~44 cycles with stddev
// ≈ 2.9 cycles across hundreds of samples.  +44 is the truth.
//
// LIVE WITNESS
//
// process_interrupt's REPORT publishes a continuously-updated Welford
// summary of this offset (pps_vclock_offset_mean_cycles, _last_cycles,
// _min, _max, _stddev, _samples).  If those fields drift away from the
// configured value here, the configured value is no longer correct.
//
// When this constant should be re-measured:
//   • GF-8802 wiring or pin routing changes
//   • Pad / IOMUX configuration changes for PPS pin or VCLOCK input
//   • QTimer1 setup changes (CM, PCS, source selection)
//   • NVIC priority changes for IRQ_GPIO6789 or IRQ_QTIMER1
//   • Optimization level or compiler version changes
//   • Any change to GPIO_TOTAL_LATENCY or QTIMER_TOTAL_LATENCY
//
static constexpr int32_t CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES = 44;

// Adjustment for QTimer counter32 reads.  Currently zero; placeholder for
// future use if a counter-read offset is empirically established.
static constexpr int32_t QTIMER_COUNTER32_TICKS_LATE_AT_READ = 0;