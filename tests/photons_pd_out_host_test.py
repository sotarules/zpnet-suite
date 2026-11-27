#!/usr/bin/env python3
"""Run the production PD OUT acquisition against simulated ADC/DWT registers.

Run: python3 tests/photons_pd_out_host_test.py
Needs a C++17 host compiler (CXX, default g++), no hardware or database.
ARM barriers and interrupt-mask reads are replaced for the host; production
acquisition, voting, moments, raw capture and window code runs unchanged.
Simulated conversion delays exercise software ordering, not physical aperture,
electrical behavior, real interrupt latency or the Teensy firmware build.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "pnc/firmware/teensy"

ARDUINO = r'''
#pragma once
#include <cstdint>
#include <cassert>
#include <array>
#define DMAMEM
#define F_CPU_ACTUAL 600000000U
#define INPUT_DISABLE 5
#define ADC_HS_COCO0 1U
#define ADC_HC_ADCH(n) (n)
#define ADC_CFG_ADICLK(n) (n)
#define ADC_CFG_ADIV(n) ((n) << 5)
#define ADC_CFG_MODE(n) ((n) << 2)
#define ADC_CFG_ADHSC (1U << 10)
enum { IRQ_ADC2 };
static bool adc_irq_enabled;
static uint32_t clock_ms;
struct CycleCounter {
  uint32_t value = 0;
  operator uint32_t() { const auto out = value; value += 12U; return out; }
} ARM_DWT_CYCCNT;
static uint32_t ADC1_CFG = 0x1234, ADC1_GC = 0x5678;
static uint32_t ADC1_HC0, ADC1_HS = ADC_HS_COCO0, ADC1_R0 = 100U;
static uint32_t ADC2_CFG, ADC2_GC;
static std::array<uint32_t, 3> inputs;
static uint32_t next_input, current_input, conversion_start, third_write_delay;
static uint32_t conversion_cycles = 1020U;
static bool converting;
struct Control {
  uint32_t value = 31U;
  void operator=(uint32_t v) {
    value = v;
    if (v == 31U) { converting = false; return; }
    assert(v == 9U && !converting && next_input < inputs.size());
    current_input = next_input++;
    if (current_input == 2U) ARM_DWT_CYCCNT.value += third_write_delay;
    conversion_start = ARM_DWT_CYCCNT.value;
    converting = true;
  }
  operator uint32_t() const { return value; }
} ADC2_HC0;
struct Status {
  operator uint32_t() const {
    return converting && uint32_t(ARM_DWT_CYCCNT.value - conversion_start) >= conversion_cycles
        ? ADC_HS_COCO0 : 0U;
  }
} ADC2_HS;
struct Result {
  operator uint32_t() const {
    if (!converting) return 0U; // initialization/stop drains a disabled ADC
    assert(uint32_t(ADC2_HS) & ADC_HS_COCO0);
    converting = false;
    return inputs[current_input];
  }
} ADC2_R0;
static int analog_pin, analog_mode;
inline void pinMode(int pin, int mode) { analog_pin = pin; analog_mode = mode; }
inline uint32_t millis() { return clock_ms; }
inline void NVIC_DISABLE_IRQ(int) { adc_irq_enabled = false; }
inline void NVIC_CLEAR_PENDING(int) {}
inline bool NVIC_IS_ENABLED(int) { return adc_irq_enabled; }
'''

HARNESS = r'''
#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include "photons_pd_out.cpp"

static timepop_action_callback_t action;
static void* action_data;
static timepop_action_result_t action_result;
void interrupt_adc_high_configure() { adc_irq_enabled = true; }
bool timepop_action_arm_dwt(uint32_t target, uint32_t, timepop_action_callback_t fn, void* data) {
  action = fn; action_data = data; action_result = {};
  action_result.context.target_dwt = target;
  return true;
}
bool timepop_action_cancel() { return false; }
bool timepop_action_take(timepop_action_result_t* out) { *out = action_result; return true; }

static void pulse(std::array<uint32_t, 3> codes, uint32_t edge = 10000U,
                  uint64_t width = 5000U, uint32_t handoff_delay = 0U,
                  uint32_t write_delay = 0U, uint32_t entry_work_cycles = 0U,
                  uint32_t adc_cycles = 1020U) {
  photons_pd_out_set_pulse_ns(width);
  inputs = codes; next_input = 0U; third_write_delay = write_delay;
  conversion_cycles = adc_cycles;
  ARM_DWT_CYCCNT.value = edge + entry_work_cycles;
  photons_pd_out_high_isr(edge);
  assert(next_input == 1U); // P0 only starts the original HIGH conversion
  ARM_DWT_CYCCNT.value += handoff_delay;
  photons_pd_out_complete_isr();
  assert(next_input == 3U && !converting && g_pd_phase == pd_phase_t::IDLE);
}
static photons_pd_out_batch_t cut() {
  clock_ms += 1000U;
  auto b = photons_pd_out_take_batch(clock_ms);
  assert(b.sample_count == b.highv_sample_count + b.highv_late_count + b.highv_short_pulse_count);
  return b;
}
static void rebase() { photons_pd_out_rebase(clock_ms); }

struct Record {
  std::map<std::string, uint64_t> ints;
  std::map<std::string, fixed_decimal_t> decimals;
  std::map<std::string, std::string> strings;
  void clear() { ints.clear(); decimals.clear(); strings.clear(); }
  template<class T> void add(const char* key, T value) { ints[key] = value; }
  void add(const char* key, fixed_decimal_t value) { decimals[key] = value; }
  void add(const char* key, const char* value) { strings[key] = value; }
};
static Record render(photons_pd_out_batch_t batch) {
  struct { photons_pd_out_batch_t pd_out; } f{batch};
  static Record high_window;
  // PRODUCTION_HIGHV_RENDER
  return high_window;
}

int main() {
  photons_pd_out_init();
  assert(analog_pin == 40 && analog_mode == INPUT_DISABLE && adc_irq_enabled);
  assert(ADC1_CFG == 0x1234 && ADC1_GC == 0x5678);
  assert(ADC2_GC == 0U); // no hardware averaging or continuous conversions
  assert(PHOTONS_RACE_PULSE_NS == 5000U && PHOTONS_PD_OUT_CAPTURE_SAMPLES == 3U);
  rebase();

  // The first reading remains HIGH even when the vote rejects its amplitude.
  pulse({900, 500, 501});
  assert(uint32_t(ARM_DWT_CYCCNT.value - 10000U) > 3000U); // read 3 is after 5 us
  auto b = cut();
  assert(b.sample_count == 1 && b.high_sum == 900 && b.high_sum_squares == 810000);
  assert(b.highv_sample_count == 1 && b.highv_sum == 501 && b.highv_sum_squares == 251001);
  assert(b.high5_window.sample_count == 1 && b.high5_window.high_sum == 900);
  const auto& capture = photons_pd_out_capture_latest();
  assert(capture.stored_rows == 1 && capture.rows_seen == 1);
  assert(capture.samples[0][0] == 900 && capture.samples[0][1] == 500 && capture.samples[0][2] == 501);
  assert(capture.start_cycles[0] < capture.start_cycles[1]);
  assert(capture.start_cycles[1] < capture.start_cycles[2]);
  assert(b.highv_last_start_max_cycles >= capture.start_cycles[2]);
  auto wire = render(b);
  assert(wire.strings.at("schema") == "PD_OUT_HIGHV_MEDIAN3_V1");
  assert(wire.ints.at("sample_count") == 1U);
  assert(wire.decimals.at("high_mean_mv").whole == 403U);
  assert(wire.decimals.at("high_mean_mv").fractional == 736264U);
  b = cut();
  assert(b.sample_count == 0 && b.highv_sample_count == 0 && b.highv_sum == 0);
  assert(b.highv_last_start_max_cycles == 0);
  assert(render(b).decimals.empty()); // reused builder has no stale/zero means

  // Field report 2026-11-27: first trigger 307 cycles (~512 ns), then
  // ~1.8 us per conversion. Normal third triggers already exceed 4 us.
  // Model that acquisition instead of only the optimistic 200 ns minimum.
  pulse({900, 500, 501}, 10000U, 5000U, 0U, 0U, 295U, 1065U);
  b = cut();
  const auto measured_start = b.highv_last_start_max_cycles;
  assert(photons_pd_out_capture_latest().start_cycles[0] == 307U);
  assert(measured_start > 2400U && measured_start <= 2558U);
  assert(b.highv_sample_count == 1 && b.highv_sum == 501 && b.highv_late_count == 0);
  assert(b.high_sum == 900); // first-sample HIGH is still independent of the vote

  // Replay observed final-trigger bounds: 4263.333 ns is ordinary, while
  // 5328.333 ns is delayed. Both times are software observations, not apertures.
  pulse({900, 500, 501}, 10000U, 5000U, 0U, 2558U - measured_start, 295U, 1065U);
  b = cut();
  assert(b.highv_last_start_max_cycles == 2558U && b.highv_sample_count == 1);
  pulse({900, 500, 501}, 10000U, 5000U, 0U, 3197U - measured_start, 295U, 1065U);
  b = cut();
  assert(b.highv_last_start_max_cycles == 3197U && b.highv_late_count == 1);
  assert(b.highv_sample_count == 0 && b.high_sum == 900);

  // The revised 4500 ns budget is inclusive; the next CPU cycle is outside.
  pulse({900, 500, 501}, 10000U, 5000U, 0U, 2700U - measured_start, 295U, 1065U);
  b = cut();
  assert(b.highv_last_start_max_cycles == 2700U && b.highv_sample_count == 1);
  pulse({900, 500, 501}, 10000U, 5000U, 0U, 2701U - measured_start, 295U, 1065U);
  b = cut();
  assert(b.highv_last_start_max_cycles == 2701U && b.highv_late_count == 1);
  assert(b.highv_sample_count == 0 && b.high_sum == 900);

  // All orderings, repeated samples and the entire ADC range are ordinary data.
  std::array<uint32_t, 3> ordered{1, 1234, 4095};
  uint64_t high_sum = 0;
  do { pulse(ordered); high_sum += ordered[0]; }
  while (std::next_permutation(ordered.begin(), ordered.end()));
  pulse({0, 0, 4095}); pulse({4095, 0, 4095}); pulse({77, 77, 77});
  b = cut();
  assert(b.highv_sample_count == 9 && b.highv_sum == 6U * 1234U + 4095U + 77U);
  assert(b.high_sum == high_sum + 4095U + 77U);

  // Delayed handoff and preemption around the third register write cannot
  // masquerade as plateau votes. Original HIGH still counts exactly once.
  rebase();
  pulse({500, 501, 502}, 10000U, 5000U, 6000U);
  pulse({600, 601, 602}, 20000U, 5000U, 0U, 1200U);
  pulse({700, 701, 702}, 30000U, 1000U);
  b = cut();
  assert(b.sample_count == 3 && b.high_sum == 1800 && b.highv_sample_count == 0);
  assert(b.highv_late_count == 2 && b.highv_short_pulse_count == 1);
  assert(b.highv_last_start_max_cycles > 2400U);
  assert(b.high5_window.sample_count == 3 && b.high5_window.high_sum == 1800);

  // Rebase discards the preceding partial second, including vote outcomes.
  pulse({100, 200, 300}); rebase();
  pulse({900, 500, 501}, 0xfffffff0U); // unsigned DWT rollover
  b = cut();
  assert(b.sample_count == 1 && b.highv_sum == 501 && b.highv_late_count == 0);
  assert(b.highv_short_pulse_count == 0 && b.high5_window.sample_count == 1);

  // LOW/HIGH pairing retains one first-sample HIGH, not the median or 3 votes.
  rebase();
  ARM_DWT_CYCCNT.value = 40000U;
  photons_pd_out_low_prepare(1U, 3000U, 120000U);
  photons_pd_out_low_launch(40000U, 1U);
  ARM_DWT_CYCCNT.value = action_result.context.target_dwt;
  action_result.outcome = timepop_action_outcome_t::DISPATCHED;
  action(action_result.context, action_data);
  pulse({900, 500, 501}, 43000U);
  b = cut();
  assert(b.low_high.outcomes[0] == 1U && b.low_high.last.high_samples == 1U);
  assert(b.low_high.last.high_counts == 900U && b.low_high.paired[1].sum == 900);
  assert(b.low_high.paired[0].sum == 100 && b.highv_sum == 501);
  assert(ADC1_CFG == 0x1234 && ADC1_GC == 0x5678);

  // Both old windows retain first-sample membership across many report cuts;
  // HIGHV has only the current fragment's population. Raw rows stay bounded.
  rebase();
  for (unsigned second = 0; second < 7; ++second) {
    for (unsigned n = 0; n < 1100; ++n) pulse({600, 499, 501});
    b = cut();
    assert(b.sample_count == 1100 && b.high_sum == 660000 && b.highv_sum == 551100);
    assert(b.highv_sample_count == 1100 && b.highv_sum_squares == 1100ULL * 501U * 501U);
    const auto& raw = photons_pd_out_capture_latest();
    assert(raw.stored_rows == 1000 && raw.rows_seen == 1100);
    assert(b.high5_window.sample_count == std::min(second + 1U, 5U) * 1100U);
    assert(b.high5_window.high_sum == b.high5_window.sample_count * 600U);
    assert(b.high_window.sample_count == (second + 1U) * 1100U);
  }
  photons_pd_out_stop();
  assert(ADC2_HC0 == 31U);
  std::cout << "PASS: production triplets, median votes, timing/width outcomes, HIGH5 custody, "
               "LOW pairing, raw banks, rebase, DWT wrap and 7700-pulse windows\n";
}
'''


def main():
    source = (FW / "photons_pd_out.cpp").read_text()
    for instruction in ("dsb", "dmb"):
        source = source.replace(
            f'__asm__ volatile ("{instruction}" ::: "memory");',
            '__asm__ volatile ("" ::: "memory");',
        )
    for register in ("primask", "basepri"):
        source = source.replace(
            f'__asm__ volatile ("mrs %0, {register}" : "=r" ({register}));',
            f'{register} = 0U;',
        )
    publisher = (FW / "process_photons.cpp").read_text()
    start = publisher.index('  high_window.clear();\n  high_window.add("schema", "PD_OUT_HIGHV_MEDIAN3_V1");')
    end = publisher.index('  pd_out.add_object("highv", high_window);', start)
    harness = HARNESS.replace('// PRODUCTION_HIGHV_RENDER', publisher[start:end])
    with tempfile.TemporaryDirectory(prefix="photons-pd-out-") as temp:
        temp = Path(temp)
        (temp / "Arduino.h").write_text(ARDUINO)
        (temp / "photons_pd_out.cpp").write_text(source)
        (temp / "test.cpp").write_text(harness)
        binary = temp / "test"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-O2", "-Wall", "-Wextra",
                        "-I", str(temp), "-I", str(FW), str(temp / "test.cpp"),
                        str(FW / "double.cpp"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=30)


if __name__ == "__main__":
    main()
