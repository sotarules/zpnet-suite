#include "process_clocks.h"

#include "events.h"
#include "payload.h"
#include "publish.h"
#include "timepop.h"
#include "config.h"

#include <Arduino.h>
#include "imxrt.h"

// ============================================================================
// DWT (CPU cycle counter)
// ============================================================================

#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)

#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

static uint64_t dwt_cycles_64  = 0;
static uint32_t dwt_last_32    = 0;

static inline void dwt_enable(void) {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  dwt_last_32 = DWT_CYCCNT;
}

uint64_t clocks_dwt_cycles_now(void) {
  uint32_t now = DWT_CYCCNT;
  dwt_cycles_64 += (uint32_t)(now - dwt_last_32);
  dwt_last_32 = now;
  return dwt_cycles_64;
}

// Exact rational conversion: 1 cycle = 5/3 ns
uint64_t clocks_dwt_ns_now(void) {
  return (clocks_dwt_cycles_now() * 5ull) / 3ull;
}

// ============================================================================
// GNSS / OCXO PRESCALED DOMAINS (10 kHz)
// ============================================================================

#define GPT_PRESCALE_DIVIDER   1000u
#define GPT_PRESCALE_VALUE     (GPT_PRESCALE_DIVIDER - 1)

static volatile uint64_t gnss_ticks_10khz = 0;
static volatile uint64_t ocxo_ticks_10khz = 0;

static volatile uint32_t gnss_dwt_baseline = 0;
static volatile uint32_t ocxo_dwt_baseline = 0;

static bool gpt1_armed = false;
static bool gpt2_armed = false;

// -----------------------------------------------------------------------------
// GPT clock gating
// -----------------------------------------------------------------------------

static inline void enable_gpt1(void) {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

static inline void enable_gpt2(void) {
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
}

// -----------------------------------------------------------------------------
// GNSS VCLOCK → GPT2
// -----------------------------------------------------------------------------

static void arm_gpt2_external(void) {
  if (gpt2_armed) return;

  enable_gpt2();

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_PR = GPT_PRESCALE_VALUE;
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;

  GPT2_OCR1 = GPT2_CNT + 1;
  GPT2_IR |= GPT_IR_OF1IE;

  attachInterruptVector(IRQ_GPT2, [] {
    GPT2_SR = GPT_SR_OF1;
    GPT2_OCR1 += 1;
    gnss_ticks_10khz++;
    gnss_dwt_baseline = DWT_CYCCNT;
  });

  NVIC_ENABLE_IRQ(IRQ_GPT2);
  GPT2_CR |= GPT_CR_EN;
  gpt2_armed = true;
}

// -----------------------------------------------------------------------------
// OCXO → GPT1
// -----------------------------------------------------------------------------

static void arm_gpt1_internal(void) {
  if (gpt1_armed) return;

  enable_gpt1();

  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = GPT_PRESCALE_VALUE;
  GPT1_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR;

  GPT1_OCR1 = GPT1_CNT + 1;
  GPT1_IR |= GPT_IR_OF1IE;

  attachInterruptVector(IRQ_GPT1, [] {
    GPT1_SR = GPT_SR_OF1;
    GPT1_OCR1 += 1;
    ocxo_ticks_10khz++;
    ocxo_dwt_baseline = DWT_CYCCNT;
  });

  NVIC_ENABLE_IRQ(IRQ_GPT1);
  GPT1_CR |= GPT_CR_EN;
  gpt1_armed = true;
}

// ============================================================================
// SYNTHETIC NANOSECOND CLOCKS
// ============================================================================

static constexpr uint64_t NS_PER_10KHZ = 100000ULL;

uint64_t clocks_gnss_ns_now(void) {
  uint64_t base = gnss_ticks_10khz * NS_PER_10KHZ;
  uint32_t delta = DWT_CYCCNT - gnss_dwt_baseline;
  uint64_t interp = (delta * 5ull) / 3ull;
  if (interp > NS_PER_10KHZ) interp = NS_PER_10KHZ;
  return base + interp;
}

uint64_t clocks_ocxo_ns_now(void) {
  uint64_t base = ocxo_ticks_10khz * NS_PER_10KHZ;
  uint32_t delta = DWT_CYCCNT - ocxo_dwt_baseline;
  uint64_t interp = (delta * 5ull) / 3ull;
  if (interp > NS_PER_10KHZ) interp = NS_PER_10KHZ;
  return base + interp;
}

// ============================================================================
// ZEROING
// ============================================================================

static uint64_t gnss_zero_ns = 0;

void clocks_zero_all(void) {
  dwt_cycles_64 = 0;
  dwt_last_32   = DWT_CYCCNT;

  gnss_ticks_10khz = 0;
  ocxo_ticks_10khz = 0;

  gnss_dwt_baseline = DWT_CYCCNT;
  ocxo_dwt_baseline = DWT_CYCCNT;

  gnss_zero_ns = clocks_gnss_ns_now();
}

uint64_t clocks_gnss_zero_ns(void) {
  return gnss_zero_ns;
}

// ============================================================================
// PPS CAPTURE + 1 Hz PUBLICATION
// ============================================================================

static volatile uint64_t pps_count = 0;
static volatile uint64_t last_pps_dwt_cycles = 0;
static volatile bool pps_scheduled = false;

static void pps_isr(void) {
  last_pps_dwt_cycles = clocks_dwt_cycles_now();
  pps_count++;

  if (!pps_scheduled) {
    pps_scheduled = true;
    timepop_arm(
      TIMEPOP_CLASS_ASAP,
      false,
      [](timepop_ctx_t*, void*) {

        Payload p;
        p.add("pps_count",   pps_count);
        p.add("dwt_cycles", last_pps_dwt_cycles);
        p.add("dwt_ns",     clocks_dwt_ns_now());
        p.add("gnss_ns",    clocks_gnss_ns_now());
        p.add("ocxo_ns",    clocks_ocxo_ns_now());
        p.add("gnss_lock",  digitalRead(GNSS_LOCK_PIN));

        publish("CLOCKS/PPS", p);
        pps_scheduled = false;
      },
      nullptr,
      "pps"
    );
  }
}

/*
 * format_hms
 *
 * Convert elapsed seconds into HH:MM:SS (zero-padded).
 *
 * Semantics:
 *   • hours are unbounded (roll past 24 if needed)
 *   • minutes and seconds are 00–59
 *   • output is always null-terminated
 *
 * Example:
 *   0        -> "00:00:00"
 *   59       -> "00:00:59"
 *   61       -> "00:01:01"
 *   3661     -> "01:01:01"
 *   90061    -> "25:01:01"
 */
static inline void format_hms(
    uint64_t seconds,
    char*    out,
    size_t   out_sz
) {
  if (!out || out_sz == 0) return;

  uint64_t h = seconds / 3600;
  uint64_t m = (seconds % 3600) / 60;
  uint64_t s = seconds % 60;

  // Minimum required size: "HH:MM:SS" + '\0' = 9 bytes
  // Using snprintf guarantees null termination.
  snprintf(
    out,
    out_sz,
    "%02llu:%02llu:%02llu",
    (unsigned long long)h,
    (unsigned long long)m,
    (unsigned long long)s
  );
}

// ================================================================
// Canonical clock report builder
// ================================================================
static Payload build_clock_report_payload(void) {

  Payload p;

  // Raw authoritative clocks
  const uint64_t dwt_ns  = clocks_dwt_ns_now();
  const uint64_t gnss_ns = clocks_gnss_ns_now();
  const uint64_t ocxo_ns = clocks_ocxo_ns_now();

  p.add("dwt_ns",  dwt_ns);
  p.add("gnss_ns", gnss_ns);
  p.add("ocxo_ns", ocxo_ns);

  // Elapsed wall time since last GNSS zero
  uint64_t zero_ns    = clocks_gnss_zero_ns();
  uint64_t elapsed_ns = (gnss_ns >= zero_ns) ? (gnss_ns - zero_ns) : 0;
  uint64_t elapsed_s  = elapsed_ns / 1000000000ULL;

  char hms[16];
  format_hms(elapsed_s, hms, sizeof(hms));

  p.add("elapsed_hms",     hms);
  p.add("elapsed_seconds", elapsed_s);

  // Drift + tempo metrics
  if (gnss_ns > 0) {

    int64_t drift_dwt_ns  = (int64_t)dwt_ns  - (int64_t)gnss_ns;
    int64_t drift_ocxo_ns = (int64_t)ocxo_ns - (int64_t)gnss_ns;

    p.add("dwt_drift_ns",  drift_dwt_ns);
    p.add("ocxo_drift_ns", drift_ocxo_ns);

    double tau_dwt  = (double)dwt_ns  / (double)gnss_ns;
    double tau_ocxo = (double)ocxo_ns / (double)gnss_ns;

    p.add_fmt("tau_dwt",  "%.12f", tau_dwt);
    p.add_fmt("tau_ocxo", "%.12f", tau_ocxo);

    double ppb_dwt  = ((double)drift_dwt_ns  / (double)gnss_ns) * 1e9;
    double ppb_ocxo = ((double)drift_ocxo_ns / (double)gnss_ns) * 1e9;

    p.add_fmt("dwt_ppb",  "%.6f", ppb_dwt);
    p.add_fmt("ocxo_ppb", "%.6f", ppb_ocxo);

  } else {
    p.add("dwt_drift_ns",  0);
    p.add("ocxo_drift_ns", 0);
    p.add("tau_dwt",       0.0);
    p.add("tau_ocxo",      0.0);
    p.add("dwt_ppb",       0.0);
    p.add("ocxo_ppb",      0.0);
  }

  p.add("gnss_lock", digitalRead(GNSS_LOCK_PIN));

  return p;
}

// ============================================================================
// PROCESS COMMANDS
// ============================================================================

static Payload cmd_report(const Payload&) {
  return build_clock_report_payload();
}

static Payload cmd_clear(const Payload&) {
  clocks_zero_all();
  Payload ev;
  ev.add("action", "zeroed");
  publish("CLOCKS_CLEAR", ev);
  return ok_payload();
}

// ============================================================================
// PROCESS REGISTRATION
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "REPORT", cmd_report },
  { "CLEAR",  cmd_clear  },
  { nullptr,  nullptr }
};

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = nullptr
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void process_clocks_init(void) {

  dwt_enable();
  arm_gpt2_external();
  arm_gpt1_internal();

  pinMode(GNSS_PPS_PIN, INPUT);
  pinMode(GNSS_LOCK_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(GNSS_PPS_PIN),
    pps_isr,
    RISING
  );
}
