#!/usr/bin/env python3
"""Exercise the production PD OUT state machine with simulated registers.

Run: python3 tests/photons_pd_out_host_test.py
Needs a C++17 host compiler (CXX, default g++). No hardware or PostgreSQL.
Only ARM memory-barrier instructions are replaced by host compiler barriers;
the acquisition, accumulation, boundary, and cancellation code is unchanged.
This does not test electrical behavior, ADC aperture, or interrupt latency.
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
#define HIGH 1
#define LOW 0
#define GPT_CR_SWR (1u << 15)
#define GPT_CR_CLKSRC(n) ((n) << 6)
#define GPT_CR_EN_24M (1u << 10)
#define GPT_CR_ENMOD (1u << 1)
#define GPT_CR_FRR (1u << 9)
#define GPT_CR_EN 1u
#define GPT_SR_OF1 1u
#define GPT_IR_OF1IE 1u
#define ADC_HS_COCO0 1u
#define ADC_HC_AIEN (1u << 7)
#define ADC_HC_ADCH(n) (n)
#define ADC_CFG_ADICLK(n) (n)
#define ADC_CFG_ADIV(n) ((n) << 5)
#define ADC_CFG_MODE(n) ((n) << 2)
#define ADC_CFG_ADHSC (1u << 10)
#define CCM_CCGR0_GPT2_BUS(n) ((n) << 24)
#define CCM_CCGR0_GPT2_SERIAL(n) ((n) << 26)
#define CCM_CCGR_ON 3u
#define INPUT_DISABLE 5
enum { IRQ_GPT2, IRQ_ADC2 };
static uint32_t ADC1_CFG = 0x12345678, ADC1_HC0 = 15;
static uint32_t ADC2_HS, ADC2_CFG, ADC2_GC, ARM_DWT_CYCCNT;
static uint32_t GPT2_CNT, GPT2_PR, GPT2_OCR1, GPT2_SR, GPT2_IR, CCM_CCGR0;
struct Control {
  uint32_t value = 0;
  void operator=(uint32_t v) { value = v; ADC2_HS = 0; }
  operator uint32_t() const { return value; }
} ADC2_HC0;
struct Result {
  uint32_t value = 0;
  operator uint32_t() const { ADC2_HS = 0; return value; }
} ADC2_R0;
struct TimerControl {
  uint32_t value = 0;
  void operator=(uint32_t v) { value = v & ~GPT_CR_SWR; }
  operator uint32_t() const { return value; }
} GPT2_CR;
static void (*vectors[2])();
static bool enabled[2];
static uint32_t priorities[2];
static int analog_pin = -1, analog_mode = -1;
inline void pinMode(int pin, int mode) { analog_pin = pin; analog_mode = mode; }
inline void attachInterruptVector(int n, void (*fn)()) { vectors[n] = fn; }
inline void NVIC_SET_PRIORITY(int n, int p) { priorities[n] = p; }
inline void NVIC_CLEAR_PENDING(int) {}
inline void NVIC_ENABLE_IRQ(int n) { enabled[n] = true; }
inline void NVIC_DISABLE_IRQ(int n) { enabled[n] = false; }
inline bool NVIC_IS_ENABLED(int n) { return enabled[n]; }
'''

HARNESS = r'''
#include <map>
#include <string>
#include <iostream>
#include "photons_pd_out.cpp"

static void finish_high(uint32_t code, uint32_t dwt) {
  ADC2_R0.value = code; ADC2_HS = ADC_HS_COCO0;
  ARM_DWT_CYCCNT = dwt;
  vectors[IRQ_GPT2]();
  assert(g_pd_phase == pd_phase_t::SAMPLING_LOW);
  assert(ADC2_HC0 == (ADC_HC_AIEN | 9u));
  assert(!(GPT2_CR & GPT_CR_EN));
}
static void finish_low(uint32_t code) {
  ADC2_R0.value = code; ADC2_HS = ADC_HS_COCO0;
  vectors[IRQ_ADC2]();
  assert(g_pd_phase == pd_phase_t::IDLE);
}
static void pair(uint32_t high, uint32_t low, uint32_t edge = 100u) {
  ARM_DWT_CYCCNT = edge + 30u;
  photons_pd_out_high_isr(edge);
  assert(ADC2_HC0 == 9u);
  assert(GPT2_CR & GPT_CR_EN);
  finish_high(high, edge + 1300u);
  finish_low(low);
}
struct Record {
  std::map<std::string, uint64_t> ints;
  std::map<std::string, fixed_decimal_t> decimals;
  std::map<std::string, Record> objects;
  void clear() { ints.clear(); decimals.clear(); objects.clear(); }
  template<class T> void add(const char* key, T value) { ints[key] = value; }
  void add(const char* key, fixed_decimal_t value) { decimals[key] = value; }
  void add_object(const char* key, const Record& value) { objects[key] = value; }
};
static Record render(photons_pd_out_batch_t batch) {
  struct { photons_pd_out_batch_t pd_out; } f{batch};
  Record pd_out, instrument;
  // PRODUCTION_RENDER
  return instrument.objects.at("pd_out");
}
int main() {
  photons_pd_out_init();
  assert(analog_pin == 40 && analog_mode == INPUT_DISABLE);
  assert(GPT2_OCR1 == 48);
  assert((GPT2_CR & GPT_CR_CLKSRC(7)) == GPT_CR_CLKSRC(1));
  assert(!(GPT2_CR & GPT_CR_EN_24M));
  assert(enabled[IRQ_GPT2] && enabled[IRQ_ADC2]);
  assert(priorities[IRQ_GPT2] == 64 && priorities[IRQ_ADC2] == 64);
  assert(ADC1_CFG == 0x12345678 && ADC1_HC0 == 15); // ADC1 untouched
  assert(!(ADC2_GC & (1u << 5))); // no averaging across the short pulse

  // Complete pairs, real decimal mean serialization, then an empty interval.
  photons_pd_out_rebase();
  pair(601, 400); pair(602, 401);
  auto batch = photons_pd_out_take_batch();
  assert(batch.pair_count == 2 && batch.high_sum == 1203 && batch.low_sum == 801);
  assert(batch.high_delay_cycles_sum == 60 && batch.low_delay_cycles_sum == 2540);
  assert(batch.high_start_count_total == 2 && batch.timer_irq_count_total == 2);
  assert(batch.low_irq_count_total == 2 && batch.phase == 0);
  assert(batch.irq_enabled_mask == 3);
  auto wire = render(batch);
  assert(wire.ints.at("pair_count") == 2);
  assert(wire.ints.at("timer_irq_count_total") == 2);
  assert(wire.ints.at("phase") == 0);
  assert(wire.decimals.at("high_mean_counts").whole == 601);
  assert(wire.decimals.at("high_mean_counts").fractional == 500000);
  assert(wire.decimals.at("low_mean_counts").whole == 400);
  assert(wire.decimals.at("low_mean_counts").fractional == 500000);
  batch = photons_pd_out_take_batch();
  assert(batch.pair_count == 0 && batch.high_sum == 0 && batch.low_sum == 0);
  assert(render(batch).decimals.empty()); // no stale or invented zero means

  // A fragment boundary between HIGH and LOW never splits the pair.
  ARM_DWT_CYCCNT = 500; photons_pd_out_high_isr(470);
  GPT2_CNT = 17;
  auto high_wait = photons_pd_out_take_batch();
  assert(high_wait.pair_count == 0 && high_wait.phase == 1);
  assert(high_wait.high_start_count_total == 3 && high_wait.timer_irq_count_total == 2);
  assert(high_wait.gpt_cnt == 17 && high_wait.adc_hc0 == 9);
  assert(render(high_wait).ints.at("gpt_cnt") == 17);
  // Another physical edge cannot overwrite the in-flight HIGH or rearm GPT2.
  ARM_DWT_CYCCNT = 600; photons_pd_out_high_isr(570);
  assert(g_pd_high_dwt == 500);
  finish_high(800, 1770);
  auto low_wait = photons_pd_out_take_batch();
  assert(low_wait.pair_count == 0 && low_wait.phase == 2);
  assert(low_wait.timer_irq_count_total == 3 && low_wait.low_irq_count_total == 2);
  assert(low_wait.adc_hc0 == (ADC_HC_AIEN | 9u));
  photons_pd_out_high_isr(1800); // still busy while LOW is converting
  finish_low(450);
  batch = photons_pd_out_take_batch();
  assert(batch.pair_count == 1 && batch.high_sum == 800 && batch.low_sum == 450);
  assert(batch.busy_edges == 1); // first busy edge belonged to preceding batch

  // ADC not ready at LOW deadline: omit both halves and account for the miss.
  photons_pd_out_high_isr(2000);
  vectors[IRQ_GPT2]();
  batch = photons_pd_out_take_batch();
  assert(batch.high_not_ready == 1 && batch.pair_count == 0);
  assert(g_pd_phase == pd_phase_t::IDLE && ADC2_HC0 == 31);
  pair(700, 300);
  assert(photons_pd_out_take_batch().pair_count == 1);

  // DISABLE cancellation during either phase cannot leak a partial result.
  photons_pd_out_high_isr(3000); photons_pd_out_stop();
  pair(900, 200);
  batch = photons_pd_out_take_batch();
  assert(batch.pair_count == 1 && batch.high_sum == 900);
  photons_pd_out_high_isr(4000); finish_high(1234, 5300);
  photons_pd_out_stop(); pair(1000, 250);
  batch = photons_pd_out_take_batch();
  assert(batch.pair_count == 1 && batch.high_sum == 1000 && batch.low_sum == 250);

  // Rebase excludes pre-publication history; zero and full-scale codes are data.
  pair(1, 1); photons_pd_out_rebase(); pair(4095, 0);
  batch = photons_pd_out_take_batch();
  assert(batch.pair_count == 1 && batch.high_sum == 4095 && batch.low_sum == 0);
  assert(render(batch).decimals.at("low_mean_counts").whole == 0);

  // DWT wrap uses unsigned elapsed arithmetic, not signed absolute times.
  pair(100, 50, 0xfffffff0u);
  batch = photons_pd_out_take_batch();
  assert(batch.high_delay_cycles_sum == 30 && batch.low_delay_cycles_sum == 1270);

  // Sustained paired accumulation and repeated fragment cuts.
  for (unsigned second = 0; second < 3; ++second) {
    for (unsigned n = 0; n < 1000; ++n) pair(600 + n % 2, 400);
    batch = photons_pd_out_take_batch();
    assert(batch.pair_count == 1000 && batch.high_sum == 600500 && batch.low_sum == 400000);
  }
  std::cout << "PASS: paired means, empty/split fragments, overlap, missed conversion, "
               "cancellation, rebase, zero/full scale, DWT wrap, and 3000-pair run\n";
}
'''

def main():
    source = (FW / "photons_pd_out.cpp").read_text()
    for instruction in ("dsb", "dmb"):
        source = source.replace(
            f'__asm__ volatile ("{instruction}" ::: "memory");',
            '__asm__ volatile ("" ::: "memory");',
        )
    fragment = (FW / "process_photons.cpp").read_text()
    start = fragment.index('  pd_out.clear();')
    end = fragment.index('  pd_out.clear();', start + len('  pd_out.clear();'))
    render = fragment[start:end + len('  pd_out.clear();')]
    with tempfile.TemporaryDirectory(prefix="photons-pd-out-") as temp:
        temp = Path(temp)
        (temp / "Arduino.h").write_text(ARDUINO)
        (temp / "photons_pd_out.cpp").write_text(source)
        (temp / "test.cpp").write_text(HARNESS.replace('// PRODUCTION_RENDER', render))
        binary = temp / "test"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-O2", "-Wall", "-Wextra",
                        "-I", str(temp), "-I", str(FW), str(temp / "test.cpp"),
                        str(FW / "double.cpp"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    main()
