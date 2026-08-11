#include "process_photons.h"

#include "config.h"
#include "events.h"
#include "payload.h"
#include "process.h"
#include "process_interrupt.h"
#include "publish.h"
#include "timepop.h"
#include "util.h"

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// PHOTONS scaffold doctrine
// ============================================================================
//
// process_interrupt owns the physical PD200T comparator interrupt and the
// first-instruction DWT coordinate.
//
// PHOTONS consumes that immutable edge fact through the specialized high-rate
// PHOTODIODE subscription.  The ISR callback below is intentionally tiny:
// scalar updates only, no Payload, no publication, no CLOCKS call, no TimePop
// mutation.
//
// Once per second a normal foreground TimePop callback snapshots the toy state
// and publishes PHOTONS_FRAGMENT.  This is scaffolding for validating custody,
// not the final optical science implementation.
// ============================================================================

static constexpr uint64_t PHOTONS_FRAGMENT_PERIOD_NS = 1000000000ULL;


// ============================================================================
// Optical device control / telemetry
// ============================================================================
//
// PHOTONS is the umbrella owner for photon-producing and photon-detecting
// devices. process_interrupt still owns PD200T comparator edge custody.
//
// Laser driver: MP5491 / EV5491-C-00A
//
static constexpr uint8_t MP5491_ADDR = 0x66;

static constexpr uint8_t MP5491_REG_CTL0    = 0x00;
static constexpr uint8_t MP5491_REG_CTL1    = 0x01;
static constexpr uint8_t MP5491_REG_ID1_MSB = 0x07;
static constexpr uint8_t MP5491_REG_ID1_LSB = 0x08;

static constexpr uint8_t MP5491_SYSEN_BIT  = 0x80;
static constexpr uint8_t MP5491_ID_EN_BIT  = 0x80;
static constexpr uint8_t MP5491_ID1_EN_BIT = 0x08;

// Existing authoritative laser setting: ID1 = 20 mA, 0.25 mA / LSB.
static constexpr uint8_t PHOTONS_LASER_ID1_CURRENT_MSB = 0x14;
static constexpr uint8_t PHOTONS_LASER_ID1_CURRENT_LSB = 0x00;

static constexpr float PHOTONS_LASER_EMIT_THRESHOLD_V = 0.75f;


// ============================================================================
// Temporary PD200T physical emulator
// ============================================================================
//
// Until the Koheron PD200T arrives, PHOTONS runs an always-on physical emulator
// below the interrupt custody boundary:
//
//   pin 33 OUTPUT --> physical jumper --> pin 34 PHOTODIODE_EDGE_PIN
//
// PHOTONS never calls its own detector callback.  Every emulated detector edge
// is a real electrical rising edge on pin 34 and therefore enters through
// process_interrupt exactly as the future PD200T comparator will.
//
// The optical launch itself is real: LD_ON is pulsed for approximately 20 ns.
// Detector timing is deliberately synthetic.  One ordinary recurring TimePop
// timer provides a fixed 10 ms emulator cadence; its callback advances state only
// and never mutates TimePop scheduling.
//
// One emulator train is fixed and explicit:
//
//   laser pulse
//   hit 1  -- physical pin-33 -> pin-34 edge; ignored by lap semantics
//   hit 2  -- lap start
//   hit 3  -- lap 1 end
//   hit 4  -- lap 2 end
//   hit 5  -- lap 3 end
//   dead lap -- one full synthetic-lap delay with no detector edge
//   repeat at the next laser pulse
//
// Each cadence tick advances exactly one state.  Hit 1 is emitted immediately
// after the real laser pulse; hit 2 follows on the next cadence tick.  After hit 5,
// the next tick is the dead lap: no detector edge is emitted and the following
// train begins on that same tick.  Every hit still traverses the physical pin-33
// jumper and real pin-34 ISR.
//
// PD200T MON on pin 38/A14 is not sampled while this emulator is installed.
// Instead a plausible synthetic ADC value is maintained as slow telemetry.
// Delete this entire block when the physical PD200T becomes authoritative.
// ============================================================================

static constexpr bool PHOTONS_EMULATOR_ENABLED = true;
static constexpr int PHOTONS_EMULATOR_EDGE_OUTPUT_PIN = 33;
static constexpr uint32_t PHOTONS_EMULATOR_LASER_PULSE_CYCLES = 20U;
static constexpr uint32_t PHOTONS_EMULATOR_CADENCE_NS = 10000000U;  // synthetic 10 ms
static constexpr uint32_t PHOTONS_EMULATOR_HITS_PER_TRAIN = 5U;
static constexpr uint32_t PHOTONS_EMULATOR_MEASURED_LAPS = 3U;

// Provisional PD200T MON range for bring-up only.  1200-1800 ADC counts maps to
// roughly 0.97-1.45 V on the Teensy 3.3 V / 12-bit ADC scale.
static constexpr uint16_t PHOTONS_EMULATOR_MON_RAW_MIN = 1200U;
static constexpr uint16_t PHOTONS_EMULATOR_MON_RAW_MAX = 1800U;

enum class photons_emulator_stage_t : uint8_t {
  IDLE = 0,
  WAIT_HIT2,
  WAIT_HIT3,
  WAIT_HIT4,
  WAIT_HIT5,
  DEAD_LAP,
};


struct photons_emulator_state_t {
  bool initialized = false;
  bool train_active = false;

  timepop_handle_t cadence_timer = TIMEPOP_INVALID_HANDLE;
  uint32_t cadence_timer_arm_count = 0U;
  uint32_t cadence_timer_arm_fail_count = 0U;
  uint32_t cadence_tick_count = 0U;

  photons_emulator_stage_t stage = photons_emulator_stage_t::IDLE;
  uint32_t train_count = 0U;
  uint32_t relaunch_count = 0U;

  uint32_t laser_pulse_count = 0U;
  uint32_t last_laser_pulse_cycles = 0U;
  uint32_t max_laser_pulse_cycles = 0U;

  uint32_t generated_edge_count = 0U;
  uint32_t expected_hit_ordinal = 0U;
  uint32_t unexpected_edge_count = 0U;
  uint32_t state_error_count = 0U;

  uint32_t hit1_count = 0U;
  uint32_t hit2_count = 0U;
  uint32_t hit3_count = 0U;
  uint32_t hit4_count = 0U;
  uint32_t hit5_count = 0U;

  uint32_t lap_start_count = 0U;
  uint32_t lap1_complete_count = 0U;
  uint32_t lap2_complete_count = 0U;
  uint32_t lap3_complete_count = 0U;
  uint32_t dead_lap_count = 0U;

  uint32_t lap_start_dwt = 0U;
  uint32_t previous_lap_endpoint_dwt = 0U;
  uint32_t lap1_last_cycles = 0U;
  uint32_t lap2_last_cycles = 0U;
  uint32_t lap3_last_cycles = 0U;

  uint16_t mon_raw = PHOTONS_EMULATOR_MON_RAW_MIN;
};


static photons_emulator_state_t g_photons_emulator{};

static void photons_emulator_cadence_tick(
    timepop_ctx_t* ctx,
    timepop_diag_t* diag,
    void* user_data);

struct photons_device_snapshot_t {
  bool     laser_enabled = false;
  uint16_t laser_id1_raw = 0;
  float    laser_id1_current_ma = 0.0f;
  uint16_t laser_monitor_raw = 0;
  float    laser_monitor_v = 0.0f;
  bool     laser_emitting = false;

  int      photodiode_edge_level = 0;
  uint16_t photodiode_mon_raw = 0;
  float    photodiode_mon_v = 0.0f;
};

static void photons_i2c_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MP5491_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t photons_i2c_read(uint8_t reg) {
  Wire.beginTransmission(MP5491_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MP5491_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static float photons_adc_voltage(uint16_t raw) {
  return (raw / ADC_FS_COUNTS) * ADC_FS_VOLTS;
}

static void photons_laser_inhibit(void) {
  digitalWrite(LD_ON_PIN, LOW);
}


static const char* photons_emulator_stage_name(
    photons_emulator_stage_t stage) {
  switch (stage) {
    case photons_emulator_stage_t::IDLE:      return "IDLE";
    case photons_emulator_stage_t::WAIT_HIT2: return "WAIT_HIT2";
    case photons_emulator_stage_t::WAIT_HIT3: return "WAIT_HIT3";
    case photons_emulator_stage_t::WAIT_HIT4: return "WAIT_HIT4";
    case photons_emulator_stage_t::WAIT_HIT5: return "WAIT_HIT5";
    case photons_emulator_stage_t::DEAD_LAP:  return "DEAD_LAP";
    default:                                  return "UNKNOWN";
  }
}


static void photons_emulator_refresh_mon(void) {
  // Slow synthetic telemetry only.  Deterministic bounded variation avoids
  // touching an ADC merely to make the temporary PD200T MON surface move.
  static uint16_t step = 0U;
  const uint16_t span =
      PHOTONS_EMULATOR_MON_RAW_MAX - PHOTONS_EMULATOR_MON_RAW_MIN + 1U;
  step = (uint16_t)((step + 137U) % span);
  g_photons_emulator.mon_raw =
      (uint16_t)(PHOTONS_EMULATOR_MON_RAW_MIN + step);
}


static void photons_emulator_emit_detector_edge(uint32_t hit_ordinal) {
  // The ordinal is emulator intent only.  The resulting observation is still a
  // real electrical edge on pin 34 whose timestamp belongs to process_interrupt.
  g_photons_emulator.expected_hit_ordinal = hit_ordinal;
  digitalWriteFast(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, HIGH);
  digitalWriteFast(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, LOW);
  g_photons_emulator.generated_edge_count++;
}

static void photons_emulator_emit_laser_pulse(void) {
  // DWT runs at approximately 1.008 GHz, so twenty cycles is approximately
  // 19.8 ns.  Measure the complete commanded HIGH/LOW transaction, not merely
  // the busy-wait body, so REPORT shows the actual software pulse wall time.
  const uint32_t pulse_start = ARM_DWT_CYCCNT;
  digitalWriteFast(LD_ON_PIN, HIGH);
  const uint32_t high_start = ARM_DWT_CYCCNT;
  while ((uint32_t)(ARM_DWT_CYCCNT - high_start) <
         PHOTONS_EMULATOR_LASER_PULSE_CYCLES) {
  }
  digitalWriteFast(LD_ON_PIN, LOW);
  const uint32_t pulse_cycles = ARM_DWT_CYCCNT - pulse_start;

  g_photons_emulator.laser_pulse_count++;
  g_photons_emulator.last_laser_pulse_cycles = pulse_cycles;
  if (pulse_cycles > g_photons_emulator.max_laser_pulse_cycles) {
    g_photons_emulator.max_laser_pulse_cycles = pulse_cycles;
  }
}

static void photons_emulator_begin_train(void) {
  if (g_photons_emulator.train_count != 0U) {
    g_photons_emulator.relaunch_count++;
  }

  g_photons_emulator.train_count++;
  g_photons_emulator.train_active = true;
  g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT2;
  g_photons_emulator.expected_hit_ordinal = 0U;
  g_photons_emulator.lap_start_dwt = 0U;
  g_photons_emulator.previous_lap_endpoint_dwt = 0U;

  photons_emulator_refresh_mon();

  // Step 1: real laser pulse.
  photons_emulator_emit_laser_pulse();

  // Step 2: hit 1.  It remains a real pin-34 observation but is intentionally
  // ignored as a lap endpoint by photons_emulator_observe_edge().
  photons_emulator_emit_detector_edge(1U);
}


static void photons_emulator_cadence_tick(
    timepop_ctx_t* /*ctx*/,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {
  g_photons_emulator.cadence_tick_count++;

  switch (g_photons_emulator.stage) {
    case photons_emulator_stage_t::IDLE:
      photons_emulator_begin_train();
      return;

    case photons_emulator_stage_t::WAIT_HIT2:
      photons_emulator_refresh_mon();
      photons_emulator_emit_detector_edge(2U);
      g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT3;
      return;

    case photons_emulator_stage_t::WAIT_HIT3:
      photons_emulator_refresh_mon();
      photons_emulator_emit_detector_edge(3U);
      g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT4;
      return;

    case photons_emulator_stage_t::WAIT_HIT4:
      photons_emulator_refresh_mon();
      photons_emulator_emit_detector_edge(4U);
      g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT5;
      return;

    case photons_emulator_stage_t::WAIT_HIT5:
      photons_emulator_refresh_mon();
      photons_emulator_emit_detector_edge(5U);
      g_photons_emulator.train_active = false;
      g_photons_emulator.stage = photons_emulator_stage_t::DEAD_LAP;
      return;

    case photons_emulator_stage_t::DEAD_LAP:
      // This cadence cell deliberately emits no detector edge.  It represents
      // the missing fourth lap, then starts the next train immediately.
      g_photons_emulator.dead_lap_count++;
      photons_emulator_begin_train();
      return;

    default:
      g_photons_emulator.state_error_count++;
      g_photons_emulator.train_active = false;
      g_photons_emulator.expected_hit_ordinal = 0U;
      g_photons_emulator.stage = photons_emulator_stage_t::IDLE;
      return;
  }
}


static void photons_emulator_prepare(void) {
  g_photons_emulator = photons_emulator_state_t{};

  pinMode(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, OUTPUT);
  digitalWriteFast(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, LOW);
  photons_emulator_refresh_mon();
  g_photons_emulator.initialized = true;
}

static void photons_emulator_start(void) {
  g_photons_emulator.cadence_timer_arm_count++;
  g_photons_emulator.cadence_timer =
      timepop_arm(
          PHOTONS_EMULATOR_CADENCE_NS,
          true,
          photons_emulator_cadence_tick,
          nullptr,
          "PHOTONS_EMULATOR_CADENCE");
  if (g_photons_emulator.cadence_timer == TIMEPOP_INVALID_HANDLE) {
    g_photons_emulator.cadence_timer_arm_fail_count++;
  }
}

static void photons_laser_initialize_hardware(void) {
  pinMode(LD_ON_PIN, OUTPUT);
  photons_laser_inhibit();

  pinMode(LASER_MONITOR_PIN, INPUT);
  pinMode(PHOTODIODE_MON_PIN, INPUT);
  analogReadResolution(12);

  photons_i2c_write(MP5491_REG_CTL0, MP5491_SYSEN_BIT);

  const uint8_t ctl1 = photons_i2c_read(MP5491_REG_CTL1);
  photons_i2c_write(
      MP5491_REG_CTL1,
      MP5491_ID_EN_BIT | MP5491_ID1_EN_BIT | (ctl1 & 0x07));

  photons_i2c_write(
      MP5491_REG_ID1_MSB,
      PHOTONS_LASER_ID1_CURRENT_MSB);

  const uint8_t lsb = photons_i2c_read(MP5491_REG_ID1_LSB);
  photons_i2c_write(
      MP5491_REG_ID1_LSB,
      (lsb & ~0x03U) | PHOTONS_LASER_ID1_CURRENT_LSB);
}

static photons_device_snapshot_t photons_device_snapshot(void) {
  photons_device_snapshot_t out{};

  const uint8_t msb = photons_i2c_read(MP5491_REG_ID1_MSB);
  const uint8_t lsb = photons_i2c_read(MP5491_REG_ID1_LSB);
  out.laser_id1_raw = ((uint16_t)msb << 2) | (lsb & 0x03U);
  out.laser_id1_current_ma = out.laser_id1_raw * 0.25f;

  out.laser_enabled = digitalRead(LD_ON_PIN) == HIGH;

  out.laser_monitor_raw = analogRead(LASER_MONITOR_PIN);
  out.laser_monitor_v = photons_adc_voltage(out.laser_monitor_raw);
  out.laser_emitting =
      out.laser_monitor_v > PHOTONS_LASER_EMIT_THRESHOLD_V;

  // Ambient pin level is diagnostic only. Timing evidence comes exclusively
  // from process_interrupt's PHOTODIODE edge capture.
  out.photodiode_edge_level = digitalRead(PHOTODIODE_EDGE_PIN);

  // PD200T MON is emulated until the physical detector arrives.  Do not read
  // pin 38/A14 in this bring-up configuration.
  out.photodiode_mon_raw = g_photons_emulator.mon_raw;
  out.photodiode_mon_v = photons_adc_voltage(out.photodiode_mon_raw);

  return out;
}

static void photons_emit_laser_initialization_event(void) {
  const photons_device_snapshot_t device = photons_device_snapshot();

  Payload p;
  p.add("id1_raw", device.laser_id1_raw);
  p.add("id1_current_ma",
        toFixedDecimal(device.laser_id1_current_ma, 6));
  p.add("pd_voltage", toFixedDecimal(device.laser_monitor_v, 6));
  p.add("laser_emitting", device.laser_emitting);
  enqueueEvent("LASER_INITIALIZATION", p);
}

// -----------------------------------------------------------------------------
// ISR-authored live state
// -----------------------------------------------------------------------------
//
// Single writer: PHOTODIODE callback.
// Foreground readers use generation as a tiny seqlock.  No interrupt masking is
// required, so PHOTONS reporting/publication never delays sovereign CLOCKS IRQs.
//
struct photons_live_state_t {
  volatile uint32_t generation = 0;
  photons_toy_capture_t capture{};

  bool     previous_edge_valid = false;
  uint32_t previous_dwt_at_edge = 0;

  bool     previous_interval_valid = false;
  uint32_t previous_interval_cycles = 0;
};

static photons_live_state_t g_photons_live{};

// -----------------------------------------------------------------------------
// Foreground-owned publication state
// -----------------------------------------------------------------------------

static photons_toy_fragment_t g_last_fragment{};
static uint32_t g_fragment_sequence = 0;
static uint32_t g_publish_count = 0;
static uint32_t g_publish_reject_count = 0;
static uint32_t g_last_published_edge_count = 0;

static bool g_initialized = false;
static bool g_subscription_ok = false;
static bool g_interrupt_started = false;

static timepop_handle_t g_fragment_timer = TIMEPOP_INVALID_HANDLE;

static inline void photons_compiler_barrier(void) {
  __asm__ volatile("" ::: "memory");
}

static void photons_emulator_observe_edge(
    const interrupt_photodiode_edge_t& edge) {
  if (!PHOTONS_EMULATOR_ENABLED) return;

  const uint32_t hit = g_photons_emulator.expected_hit_ordinal;
  g_photons_emulator.expected_hit_ordinal = 0U;

  switch (hit) {
    case 1U:
      g_photons_emulator.hit1_count++;
      // Deliberately ignored for lap measurement.
      return;

    case 2U:
      g_photons_emulator.hit2_count++;
      g_photons_emulator.lap_start_count++;
      g_photons_emulator.lap_start_dwt = edge.dwt_at_edge;
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      return;

    case 3U:
      g_photons_emulator.hit3_count++;
      if (g_photons_emulator.previous_lap_endpoint_dwt == 0U) {
        g_photons_emulator.unexpected_edge_count++;
        return;
      }
      g_photons_emulator.lap1_last_cycles =
          edge.dwt_at_edge - g_photons_emulator.previous_lap_endpoint_dwt;
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      g_photons_emulator.lap1_complete_count++;
      return;

    case 4U:
      g_photons_emulator.hit4_count++;
      if (g_photons_emulator.previous_lap_endpoint_dwt == 0U) {
        g_photons_emulator.unexpected_edge_count++;
        return;
      }
      g_photons_emulator.lap2_last_cycles =
          edge.dwt_at_edge - g_photons_emulator.previous_lap_endpoint_dwt;
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      g_photons_emulator.lap2_complete_count++;
      return;

    case 5U:
      g_photons_emulator.hit5_count++;
      if (g_photons_emulator.previous_lap_endpoint_dwt == 0U) {
        g_photons_emulator.unexpected_edge_count++;
        return;
      }
      g_photons_emulator.lap3_last_cycles =
          edge.dwt_at_edge - g_photons_emulator.previous_lap_endpoint_dwt;
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      g_photons_emulator.lap3_complete_count++;
      return;

    default:
      // A physical photodiode edge without an emulator-authored role is still
      // preserved by the ordinary PHOTONS capture path below.  It is merely
      // unexpected with respect to this temporary state machine.
      g_photons_emulator.unexpected_edge_count++;
      return;
  }
}


// ============================================================================
// High-rate PHOTODIODE subscriber
// ============================================================================

static void photons_on_photodiode_edge(
    const interrupt_photodiode_edge_t& edge,
    const interrupt_photodiode_diag_t& /*diag*/,
    void* /*user_data*/) {

  // Begin seqlock write: odd generation means foreground must retry.
  g_photons_live.generation++;
  photons_compiler_barrier();

  photons_toy_capture_t& c = g_photons_live.capture;

  photons_emulator_observe_edge(edge);

  c.edge_count++;
  c.last_edge_sequence = edge.sequence;
  c.last_pps_sequence = edge.pps_sequence;
  c.last_dwt_at_edge = edge.dwt_at_edge;
  c.last_isr_entry_dwt_raw = edge.isr_entry_dwt_raw;
  c.isr_entry_to_edge_correction_cycles =
      edge.isr_entry_to_edge_correction_cycles;

  if (g_photons_live.previous_edge_valid) {
    const uint32_t interval =
        edge.dwt_at_edge - g_photons_live.previous_dwt_at_edge;

    c.interval_valid = true;
    c.last_interval_cycles = interval;

    if (c.min_interval_cycles == 0U || interval < c.min_interval_cycles) {
      c.min_interval_cycles = interval;
    }
    if (interval > c.max_interval_cycles) {
      c.max_interval_cycles = interval;
    }

    // Toy predictor only.  The final PHOTONS engine will decide its own
    // acceptance/exclusion semantics.
    if (g_photons_live.previous_interval_valid) {
      c.prediction_valid = true;
      c.prediction_cycles = g_photons_live.previous_interval_cycles;
      c.residual_cycles =
          (int32_t)(interval - g_photons_live.previous_interval_cycles);
    } else {
      c.prediction_valid = false;
      c.prediction_cycles = 0U;
      c.residual_cycles = 0;
    }

    g_photons_live.previous_interval_cycles = interval;
    g_photons_live.previous_interval_valid = true;
  }

  g_photons_live.previous_dwt_at_edge = edge.dwt_at_edge;
  g_photons_live.previous_edge_valid = true;

  photons_compiler_barrier();
  g_photons_live.generation++;  // commit: even generation
}

// ============================================================================
// Coherent toy snapshots
// ============================================================================

bool photons_toy_capture_snapshot(photons_toy_capture_t* out) {
  if (!out) return false;

  for (;;) {
    const uint32_t before = g_photons_live.generation;
    if (before & 1U) continue;

    photons_compiler_barrier();
    const photons_toy_capture_t snapshot = g_photons_live.capture;
    photons_compiler_barrier();

    const uint32_t after = g_photons_live.generation;
    if (before == after && !(after & 1U)) {
      *out = snapshot;
      return true;
    }
  }
}

bool photons_toy_fragment_snapshot(photons_toy_fragment_t* out) {
  if (!out) return false;
  *out = g_last_fragment;
  return true;
}

// ============================================================================
// PHOTONS_FRAGMENT toy publication
// ============================================================================

static Payload photons_fragment_payload(const photons_toy_fragment_t& f) {
  Payload p;

  p.add("schema", "PHOTONS_FRAGMENT_TOY_V0");
  p.add("sequence", f.sequence);
  p.add("publish_count", f.publish_count);

  p.add("edge_count_total", f.edge_count_total);
  p.add("edges_this_second", f.edges_this_second);

  p.add("last_edge_sequence", f.capture.last_edge_sequence);
  p.add("last_pps_sequence", f.capture.last_pps_sequence);
  p.add("last_dwt_at_edge", f.capture.last_dwt_at_edge);
  p.add("last_isr_entry_dwt_raw", f.capture.last_isr_entry_dwt_raw);
  p.add("isr_entry_to_edge_correction_cycles",
        f.capture.isr_entry_to_edge_correction_cycles);

  p.add("interval_valid", f.capture.interval_valid);
  p.add("last_interval_cycles", f.capture.last_interval_cycles);
  p.add("min_interval_cycles", f.capture.min_interval_cycles);
  p.add("max_interval_cycles", f.capture.max_interval_cycles);

  p.add("prediction_valid", f.capture.prediction_valid);
  p.add("prediction_cycles", f.capture.prediction_cycles);
  p.add("residual_cycles", f.capture.residual_cycles);

  p.add("interrupt_irq_count", f.interrupt_irq_count);
  p.add("interrupt_callback_count", f.interrupt_callback_count);
  p.add("interrupt_callback_missing_count",
        f.interrupt_callback_missing_count);
  p.add("interrupt_inactive_edge_count", f.interrupt_inactive_edge_count);
  p.add("interrupt_source_pin", f.interrupt_source_pin);
  p.add("interrupt_last_callback_wall_cycles",
        f.interrupt_last_callback_wall_cycles);
  p.add("interrupt_max_callback_wall_cycles",
        f.interrupt_max_callback_wall_cycles);

  return p;
}

static void photons_fragment_tick(
    timepop_ctx_t* /*ctx*/,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {

  photons_toy_capture_t capture{};
  if (!photons_toy_capture_snapshot(&capture)) return;

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

  photons_toy_fragment_t fragment{};
  fragment.sequence = ++g_fragment_sequence;
  fragment.publish_count = g_publish_count + 1U;

  fragment.edge_count_total = capture.edge_count;
  fragment.edges_this_second =
      capture.edge_count - g_last_published_edge_count;

  fragment.capture = capture;

  fragment.interrupt_irq_count = interrupt_diag.irq_count;
  fragment.interrupt_callback_count = interrupt_diag.callback_count;
  fragment.interrupt_callback_missing_count =
      interrupt_diag.callback_missing_count;
  fragment.interrupt_inactive_edge_count =
      interrupt_diag.inactive_edge_count;
  fragment.interrupt_source_pin = interrupt_diag.source_pin;
  fragment.interrupt_last_callback_wall_cycles =
      interrupt_diag.last_callback_wall_cycles;
  fragment.interrupt_max_callback_wall_cycles =
      interrupt_diag.max_callback_wall_cycles;

  // Author the foreground snapshot before transport admission so REPORT can
  // show exactly what PHOTONS attempted to publish.
  g_last_fragment = fragment;
  g_last_published_edge_count = capture.edge_count;

  Payload payload = photons_fragment_payload(fragment);
  if (publish("PHOTONS_FRAGMENT", payload)) {
    g_publish_count++;
    g_last_fragment.publish_count = g_publish_count;
  } else {
    g_publish_reject_count++;
  }
}

// ============================================================================
// Initialization
// ============================================================================

void process_photons_init(void) {
  if (g_initialized) return;

  g_photons_live = photons_live_state_t{};
  g_last_fragment = photons_toy_fragment_t{};
  g_fragment_sequence = 0U;
  g_publish_count = 0U;
  g_publish_reject_count = 0U;
  g_last_published_edge_count = 0U;

  photons_emulator_prepare();
  photons_laser_initialize_hardware();
  photons_emit_laser_initialization_event();

  interrupt_photodiode_subscription_t subscription{};
  subscription.on_edge = photons_on_photodiode_edge;
  subscription.user_data = nullptr;

  g_subscription_ok = interrupt_photodiode_subscribe(subscription);
  g_interrupt_started =
      g_subscription_ok &&
      interrupt_start(interrupt_subscriber_kind_t::PHOTODIODE);

  g_fragment_timer = timepop_arm(
      PHOTONS_FRAGMENT_PERIOD_NS,
      true,
      photons_fragment_tick,
      nullptr,
      "PHOTONS_FRAGMENT");

  g_initialized = true;
  if (PHOTONS_EMULATOR_ENABLED) {
    photons_emulator_start();
  }
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_report(const Payload& /*args*/) {
  photons_toy_capture_t capture{};
  (void)photons_toy_capture_snapshot(&capture);

  photons_toy_fragment_t fragment{};
  (void)photons_toy_fragment_snapshot(&fragment);

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

  const photons_device_snapshot_t device = photons_device_snapshot();

  Payload p;

  p.add("initialized", g_initialized);
  p.add("subscription_ok", g_subscription_ok);
  p.add("interrupt_started", g_interrupt_started);
  p.add("fragment_timer_armed",
        g_fragment_timer != TIMEPOP_INVALID_HANDLE);

  p.add("laser_enabled", device.laser_enabled);
  p.add("laser_id1_raw", device.laser_id1_raw);
  p.add("laser_id1_current_ma",
        toFixedDecimal(device.laser_id1_current_ma, 6));
  p.add("laser_monitor_raw", device.laser_monitor_raw);
  p.add("laser_monitor_v",
        toFixedDecimal(device.laser_monitor_v, 6));
  p.add("laser_emitting", device.laser_emitting);

  p.add("photodiode_edge_pin", PHOTODIODE_EDGE_PIN);
  p.add("photodiode_edge_level", device.photodiode_edge_level);
  p.add("photodiode_mon_pin", PHOTODIODE_MON_PIN);
  p.add("photodiode_mon_raw", device.photodiode_mon_raw);
  p.add("photodiode_mon_v",
        toFixedDecimal(device.photodiode_mon_v, 6));
  p.add("photodiode_mon_emulated", PHOTONS_EMULATOR_ENABLED);

  p.add("emulator_enabled", PHOTONS_EMULATOR_ENABLED);
  p.add("emulator_initialized", g_photons_emulator.initialized);
  p.add("emulator_edge_output_pin", PHOTONS_EMULATOR_EDGE_OUTPUT_PIN);
  p.add("emulator_loopback_input_pin", PHOTODIODE_EDGE_PIN);
  p.add("emulator_timing_fidelity", "SYNTHETIC_NOT_OPTICAL");
  p.add("emulator_cadence_strategy", "ONE_RECURRING_TIMEPOP_TIMER");
  p.add("emulator_nominal_lap_ns", PHOTONS_EMULATOR_CADENCE_NS);
  p.add("emulator_cadence_period_ns", PHOTONS_EMULATOR_CADENCE_NS);
  p.add("emulator_cadence_tick_count", g_photons_emulator.cadence_tick_count);
  p.add("emulator_hits_per_train", PHOTONS_EMULATOR_HITS_PER_TRAIN);
  p.add("emulator_measured_laps", PHOTONS_EMULATOR_MEASURED_LAPS);
  p.add("emulator_stage", photons_emulator_stage_name(g_photons_emulator.stage));
  p.add("emulator_train_active", g_photons_emulator.train_active);
  p.add("emulator_train_count", g_photons_emulator.train_count);
  p.add("emulator_relaunch_count", g_photons_emulator.relaunch_count);

  p.add("emulator_laser_pulse_requested_cycles",
        PHOTONS_EMULATOR_LASER_PULSE_CYCLES);
  p.add("emulator_laser_pulse_count", g_photons_emulator.laser_pulse_count);
  p.add("emulator_last_laser_pulse_cycles",
        g_photons_emulator.last_laser_pulse_cycles);
  p.add("emulator_max_laser_pulse_cycles",
        g_photons_emulator.max_laser_pulse_cycles);

  p.add("emulator_generated_edge_count",
        g_photons_emulator.generated_edge_count);
  p.add("emulator_expected_hit_ordinal",
        g_photons_emulator.expected_hit_ordinal);
  p.add("emulator_unexpected_edge_count",
        g_photons_emulator.unexpected_edge_count);
  p.add("emulator_state_error_count",
        g_photons_emulator.state_error_count);
  p.add("emulator_hit1_count", g_photons_emulator.hit1_count);
  p.add("emulator_hit2_count", g_photons_emulator.hit2_count);
  p.add("emulator_hit3_count", g_photons_emulator.hit3_count);
  p.add("emulator_hit4_count", g_photons_emulator.hit4_count);
  p.add("emulator_hit5_count", g_photons_emulator.hit5_count);
  const uint32_t emulator_observed_hit_count =
      g_photons_emulator.hit1_count +
      g_photons_emulator.hit2_count +
      g_photons_emulator.hit3_count +
      g_photons_emulator.hit4_count +
      g_photons_emulator.hit5_count;
  p.add("emulator_observed_hit_count", emulator_observed_hit_count);

  p.add("emulator_lap_start_count", g_photons_emulator.lap_start_count);
  p.add("emulator_lap1_complete_count",
        g_photons_emulator.lap1_complete_count);
  p.add("emulator_lap2_complete_count",
        g_photons_emulator.lap2_complete_count);
  p.add("emulator_lap3_complete_count",
        g_photons_emulator.lap3_complete_count);
  p.add("emulator_dead_lap_count", g_photons_emulator.dead_lap_count);
  p.add("emulator_lap_start_dwt", g_photons_emulator.lap_start_dwt);
  p.add("emulator_lap1_last_cycles", g_photons_emulator.lap1_last_cycles);
  p.add("emulator_lap2_last_cycles", g_photons_emulator.lap2_last_cycles);
  p.add("emulator_lap3_last_cycles", g_photons_emulator.lap3_last_cycles);

  p.add("emulator_cadence_timer_arm_count",
        g_photons_emulator.cadence_timer_arm_count);
  p.add("emulator_cadence_timer_arm_fail_count",
        g_photons_emulator.cadence_timer_arm_fail_count);
  p.add("emulator_cadence_timer_handle_valid",
        g_photons_emulator.cadence_timer != TIMEPOP_INVALID_HANDLE);

  p.add("capture_edge_count", capture.edge_count);
  p.add("capture_last_edge_sequence", capture.last_edge_sequence);
  p.add("capture_last_pps_sequence", capture.last_pps_sequence);
  p.add("capture_last_dwt_at_edge", capture.last_dwt_at_edge);
  p.add("capture_last_isr_entry_dwt_raw", capture.last_isr_entry_dwt_raw);
  p.add("capture_isr_entry_to_edge_correction_cycles",
        capture.isr_entry_to_edge_correction_cycles);

  p.add("capture_interval_valid", capture.interval_valid);
  p.add("capture_last_interval_cycles", capture.last_interval_cycles);
  p.add("capture_min_interval_cycles", capture.min_interval_cycles);
  p.add("capture_max_interval_cycles", capture.max_interval_cycles);

  p.add("capture_prediction_valid", capture.prediction_valid);
  p.add("capture_prediction_cycles", capture.prediction_cycles);
  p.add("capture_residual_cycles", capture.residual_cycles);

  p.add("fragment_sequence", fragment.sequence);
  p.add("fragment_publish_count", g_publish_count);
  p.add("fragment_publish_reject_count", g_publish_reject_count);
  p.add("fragment_edge_count_total", fragment.edge_count_total);
  p.add("fragment_edges_this_second", fragment.edges_this_second);

  p.add("interrupt_subscribed", interrupt_diag.subscribed);
  p.add("interrupt_active", interrupt_diag.active);
  p.add("interrupt_irq_count", interrupt_diag.irq_count);
  p.add("interrupt_callback_count", interrupt_diag.callback_count);
  p.add("interrupt_callback_missing_count",
        interrupt_diag.callback_missing_count);
  p.add("interrupt_inactive_edge_count",
        interrupt_diag.inactive_edge_count);
  p.add("interrupt_source_pin", interrupt_diag.source_pin);
  p.add("interrupt_last_callback_wall_cycles",
        interrupt_diag.last_callback_wall_cycles);
  p.add("interrupt_max_callback_wall_cycles",
        interrupt_diag.max_callback_wall_cycles);

  p.add("emulator_custody_counts_match",
        g_photons_emulator.generated_edge_count ==
            interrupt_diag.irq_count &&
        g_photons_emulator.generated_edge_count ==
            interrupt_diag.callback_count &&
        g_photons_emulator.generated_edge_count ==
            capture.edge_count &&
        g_photons_emulator.generated_edge_count ==
            emulator_observed_hit_count);

  return p;
}

static Payload cmd_init(const Payload& /*args*/) {
  photons_laser_initialize_hardware();
  photons_emit_laser_initialization_event();
  return ok_payload();
}

static Payload cmd_on(const Payload& /*args*/) {
  digitalWrite(LD_ON_PIN, HIGH);

  Payload ev;
  ev.add("action", "allow_emission");
  enqueueEvent("LASER_ON", ev);

  return ok_payload();
}

static Payload cmd_off(const Payload& /*args*/) {
  photons_laser_inhibit();

  Payload ev;
  ev.add("action", "inhibit_emission");
  enqueueEvent("LASER_OFF", ev);

  return ok_payload();
}

// ============================================================================
// Registration
// ============================================================================

static const process_command_entry_t PHOTONS_COMMANDS[] = {
  { "INIT",   cmd_init   },
  { "REPORT", cmd_report },
  { "ON",     cmd_on     },
  { "OFF",    cmd_off    },
  { nullptr, nullptr }
};

static const process_vtable_t PHOTONS_PROCESS = {
  .process_id = "PHOTONS",
  .commands = PHOTONS_COMMANDS,
  .subscriptions = nullptr,
};

void process_photons_register(void) {
  process_register("PHOTONS", &PHOTONS_PROCESS);
}
