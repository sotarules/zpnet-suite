// ============================================================================
// process_clocks.cpp — ZPNet CLOCKS (Teensy)
// ============================================================================
//
// CLOCKS is a dormant, PPS-anchored measurement instrument.
//
// • Boots fully silent
// • Does NOT track PPS unless a campaign is active or pending
// • Nanosecond clocks are ZERO unless a campaign is STARTED
// • All three counters are always live (capability, not narration)
//
// START / STOP / RECOVER are *requests*.
// All authoritative state transitions occur on the PPS boundary.
//
// Clock domains:
//
//   DWT    — ARM Cortex-M7 cycle counter, 600 MHz, internal
//   GNSS   — GF-8802 10 MHz VCLOCK, external via GPT2 pin 14
//   OCXO   — AOCJY1-A 10 MHz oven oscillator, external via GPT1 pin 25
//
// All three counters are structurally identical:
//   • Hardware register counts autonomously
//   • Software periodically latches and extends to 64-bit
//   • No prescaler, no ISR, no interrupts
//
// At 10 MHz the GPT counters wrap every ~429 seconds (~7 minutes).
// The 1-second PPS publish guarantees we read them well before wrap.
//
// ============================================================================

#include "process_clocks.h"

#include "payload.h"
#include "publish.h"
#include "timepop.h"
#include "config.h"
#include "util.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>

// ============================================================================
// Campaign State
// ============================================================================

enum class clocks_campaign_state_t {
  STOPPED,
  STARTED
};


static volatile clocks_campaign_state_t campaign_state =
  clocks_campaign_state_t::STOPPED;

static char campaign_name[64] = {0};

// Request flags (applied at PPS boundary)
static volatile bool request_start   = false;
static volatile bool request_stop    = false;
static volatile bool request_recover = false;

// Recovery parameters (authoritative, supplied by Pi)
static uint64_t recover_dwt_cycles = 0;
static uint64_t recover_gnss_ns    = 0;
static uint64_t recover_ocxo_ns    = 0;

// Campaign-scoped PPS second counter
static uint64_t campaign_seconds = 0;

// ============================================================================
// PPS residual tracking — per-clock-domain
// ============================================================================
//
// On each PPS pulse, we snapshot each clock counter, compute the
// delta from the prior PPS, and derive the residual:
//
//   residual = delta - expected_ticks_per_pps
//
// Running statistics are maintained using Welford's online algorithm,
// which computes mean, variance, and standard deviation incrementally
// without storing individual samples.
//
// Expected ticks per PPS interval:
//
//   DWT    600,000,000   (600 MHz internal)
//   GNSS    10,000,000   (10 MHz external)
//   OCXO    10,000,000   (10 MHz external)
//

static constexpr int64_t DWT_EXPECTED_PER_PPS  = 600000000LL;
static constexpr int64_t GNSS_EXPECTED_PER_PPS =  10000000LL;
static constexpr int64_t OCXO_EXPECTED_PER_PPS =  10000000LL;

// ----------------------------------------------------------------------------
// Per-clock residual state
// ----------------------------------------------------------------------------

struct pps_residual_t {
  uint64_t ticks_at_last_pps;   // snapshot of 64-bit counter at prior PPS
  int64_t  delta;               // raw tick count between two most recent PPS
  int64_t  residual;            // delta - expected
  bool     valid;               // false until the second PPS of a campaign

  // Welford's online statistics
  uint64_t n;                   // number of residual samples
  double   mean;                // running mean of residuals
  double   m2;                  // sum of squared differences from mean
};

static pps_residual_t residual_dwt  = {};
static pps_residual_t residual_gnss = {};
static pps_residual_t residual_ocxo = {};

// ----------------------------------------------------------------------------
// Reset all residual state (called on campaign start)
// ----------------------------------------------------------------------------

static void residual_reset(pps_residual_t& r) {
  r.ticks_at_last_pps = 0;
  r.delta             = 0;
  r.residual          = 0;
  r.valid             = false;
  r.n                 = 0;
  r.mean              = 0.0;
  r.m2                = 0.0;
}

// ----------------------------------------------------------------------------
// Update residual with a new PPS snapshot
// ----------------------------------------------------------------------------

static void residual_update(pps_residual_t& r,
                            uint64_t ticks_now,
                            int64_t  expected) {

  if (r.ticks_at_last_pps > 0) {
    r.delta    = (int64_t)(ticks_now - r.ticks_at_last_pps);
    r.residual = r.delta - expected;
    r.valid    = true;

    // Welford's online algorithm
    r.n++;
    const double x     = (double)r.residual;
    const double delta = x - r.mean;
    r.mean += delta / (double)r.n;
    const double delta2 = x - r.mean;
    r.m2 += delta * delta2;

  } else {
    r.delta    = 0;
    r.residual = 0;
    r.valid    = false;
  }

  r.ticks_at_last_pps = ticks_now;
}

// ----------------------------------------------------------------------------
// Derived statistics (safe accessors)
// ----------------------------------------------------------------------------

static inline double residual_variance(const pps_residual_t& r) {
  return (r.n >= 2) ? r.m2 / (double)(r.n - 1) : 0.0;
}

static inline double residual_stddev(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) : 0.0;
}

static inline double residual_stderr(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) / sqrt((double)r.n) : 0.0;
}

// ============================================================================
// DWT (CPU cycle counter — 600 MHz internal)
// ============================================================================

#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)

#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

static uint64_t dwt_cycles_64 = 0;
static uint32_t dwt_last_32   = 0;

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

uint64_t clocks_dwt_ns_now(void) {
  return (clocks_dwt_cycles_now() * 5ull) / 3ull;
}

// ============================================================================
// GNSS VCLOCK (10 MHz external via GPT2 — raw, polled)
// ============================================================================
//
// GPT2 counts the GNSS 10 MHz VCLOCK directly with no prescaler.
// The 32-bit counter is extended to 64-bit by periodic latching,
// structurally identical to the DWT and OCXO counters.
//
// Pin 14 = AD_B1_02, ALT8 = GPT2_CLK
//

static uint64_t gnss_ticks_64 = 0;
static uint32_t gnss_last_32  = 0;

static bool gpt2_armed = false;

static inline void enable_gpt2(void) {
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
}

static void arm_gpt2_external(void) {
  if (gpt2_armed) return;

  enable_gpt2();

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_PR = 0;                                // No prescaler — raw 10 MHz
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;   // External clock, free-run
  GPT2_CR |= GPT_CR_EN;

  gnss_last_32 = GPT2_CNT;
  gpt2_armed = true;
}

uint64_t clocks_gnss_ticks_now(void) {
  uint32_t now = GPT2_CNT;
  gnss_ticks_64 += (uint32_t)(now - gnss_last_32);
  gnss_last_32 = now;
  return gnss_ticks_64;
}

uint64_t clocks_gnss_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  // 10 MHz → 100 ns per tick
  return clocks_gnss_ticks_now() * 100ull;
}

// ============================================================================
// OCXO (10 MHz external via GPT1 — raw, polled)
// ============================================================================
//
// GPT1 counts the OCXO 10 MHz output directly with no prescaler.
// The 32-bit counter is extended to 64-bit by periodic latching,
// structurally identical to the DWT and GNSS counters.
//
// Pin 25 = AD_B0_13, ALT1 = GPT1_CLK
//

static uint64_t ocxo_ticks_64 = 0;
static uint32_t ocxo_last_32  = 0;

static bool gpt1_armed = false;

static inline void enable_gpt1(void) {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

static void arm_gpt1_external(void) {
  if (gpt1_armed) return;

  enable_gpt1();

  // Pin 25 = AD_B0_13, ALT1 = GPT1_CLK
  *(portConfigRegister(25)) = 1;

  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = 0;                                // No prescaler — raw 10 MHz
  GPT1_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;   // External clock, free-run
  GPT1_CR |= GPT_CR_EN;

  ocxo_last_32 = GPT1_CNT;
  gpt1_armed = true;
}

uint64_t clocks_ocxo_ticks_now(void) {
  uint32_t now = GPT1_CNT;
  ocxo_ticks_64 += (uint32_t)(now - ocxo_last_32);
  ocxo_last_32 = now;
  return ocxo_ticks_64;
}

uint64_t clocks_ocxo_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  // 10 MHz → 100 ns per tick
  return clocks_ocxo_ticks_now() * 100ull;
}

// ============================================================================
// Zeroing (campaign-scoped)
// ============================================================================

static void clocks_zero_all(void) {
  dwt_cycles_64 = 0;
  dwt_last_32   = DWT_CYCCNT;

  gnss_ticks_64 = 0;
  gnss_last_32  = GPT2_CNT;

  ocxo_ticks_64 = 0;
  ocxo_last_32  = GPT1_CNT;

  campaign_seconds = 0;

  // Reset all PPS residual tracking
  residual_reset(residual_dwt);
  residual_reset(residual_gnss);
  residual_reset(residual_ocxo);
}

// ============================================================================
// PPS handling
// ============================================================================

static volatile bool pps_scheduled = false;

static void pps_isr(void) {

  if (!request_start && !request_stop && !request_recover &&
      campaign_state != clocks_campaign_state_t::STARTED) {
    return;  // PPS is irrelevant when dormant
  }

  if (pps_scheduled) return;
  pps_scheduled = true;

  timepop_arm(
    TIMEPOP_CLASS_ASAP,
    false,
    [](timepop_ctx_t*, void*) {

      if (request_stop) {
        campaign_state = clocks_campaign_state_t::STOPPED;
        request_stop = false;
      }

      if (request_recover) {
        dwt_cycles_64 = recover_dwt_cycles;
        gnss_ticks_64 = recover_gnss_ns / 100ull;   // ns → 10 MHz ticks
        ocxo_ticks_64 = recover_ocxo_ns / 100ull;   // ns → 10 MHz ticks

        gnss_last_32 = GPT2_CNT;
        ocxo_last_32 = GPT1_CNT;

        request_recover = false;
      }

      if (request_start) {
        clocks_zero_all();
        campaign_state = clocks_campaign_state_t::STARTED;
        request_start = false;
      }

      if (campaign_state == clocks_campaign_state_t::STARTED) {

        // --------------------------------------------------------
        // Snapshot all clocks and compute PPS-to-PPS residuals
        // --------------------------------------------------------

        residual_update(residual_dwt,  clocks_dwt_cycles_now(), DWT_EXPECTED_PER_PPS);
        residual_update(residual_gnss, clocks_gnss_ticks_now(), GNSS_EXPECTED_PER_PPS);
        residual_update(residual_ocxo, clocks_ocxo_ticks_now(), OCXO_EXPECTED_PER_PPS);

        // --------------------------------------------------------
        // Publish TIMEBASE_FRAGMENT
        // --------------------------------------------------------

        Payload p;
        p.add("campaign",         campaign_name);
        p.add("dwt_cycles",       clocks_dwt_cycles_now());
        p.add("dwt_ns",           clocks_dwt_ns_now());
        p.add("gnss_ns",          clocks_gnss_ns_now());
        p.add("ocxo_ns",          clocks_ocxo_ns_now());
        p.add("teensy_pps_count", campaign_seconds);
        p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));

        // PPS residuals (most recent pulse)
        p.add("dwt_pps_residual",  residual_dwt.residual);
        p.add("gnss_pps_residual", residual_gnss.residual);
        p.add("ocxo_pps_residual", residual_ocxo.residual);

        publish("TIMEBASE_FRAGMENT", p);

        if (campaign_state == clocks_campaign_state_t::STARTED) {
          campaign_seconds++;
        }
      }

      pps_scheduled = false;
    },
    nullptr,
    "pps"
  );
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_start(const Payload& args) {
  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("error", "missing campaign");
    return err;
  }

  safeCopy(campaign_name, sizeof(campaign_name), name);
  request_start = true;
  request_stop  = false;

  Payload p;
  p.add("status", "start_requested");
  return p;
}

static Payload cmd_stop(const Payload&) {
  request_stop  = true;
  request_start = false;

  Payload p;
  p.add("status", "stop_requested");
  return p;
}

static Payload cmd_recover(const Payload& args) {

  const char* s_dwt  = args.getString("dwt_cycles");
  const char* s_gnss = args.getString("gnss_ns");
  const char* s_ocxo = args.getString("ocxo_ns");

  if (!s_dwt || !s_gnss || !s_ocxo) {
    Payload err;
    err.add("error", "missing recovery parameters");
    return err;
  }

  recover_dwt_cycles = strtoull(s_dwt,  nullptr, 10);
  recover_gnss_ns    = strtoull(s_gnss, nullptr, 10);
  recover_ocxo_ns    = strtoull(s_ocxo, nullptr, 10);

  request_recover = true;
  request_start   = true;
  request_stop    = false;

  Payload p;
  p.add("status", "recover_requested");
  return p;
}

// ============================================================================
// Helper — emit residual stats for one clock domain into a Payload
// ============================================================================

static void report_residual(Payload& p,
                            const char* prefix,
                            const pps_residual_t& r) {

  char key[48];

  snprintf(key, sizeof(key), "%s_pps_valid", prefix);
  p.add(key, r.valid);

  snprintf(key, sizeof(key), "%s_pps_delta", prefix);
  p.add(key, r.delta);

  snprintf(key, sizeof(key), "%s_pps_residual", prefix);
  p.add(key, r.residual);

  snprintf(key, sizeof(key), "%s_pps_n", prefix);
  p.add(key, r.n);

  snprintf(key, sizeof(key), "%s_pps_mean", prefix);
  p.add_fmt(key, "%.3f", r.mean);

  snprintf(key, sizeof(key), "%s_pps_stddev", prefix);
  p.add_fmt(key, "%.3f", residual_stddev(r));

  snprintf(key, sizeof(key), "%s_pps_stderr", prefix);
  p.add_fmt(key, "%.3f", residual_stderr(r));
}

// ============================================================================
// REPORT — diagnostic introspection only
// ============================================================================

static Payload cmd_report(const Payload&) {

  Payload p;

  // ------------------------------------------------------------
  // Campaign state
  // ------------------------------------------------------------

  p.add(
    "campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED
      ? "STARTED"
      : "STOPPED"
  );

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign", campaign_name);
    p.add("campaign_seconds", campaign_seconds);
  }

  // ------------------------------------------------------------
  // Pending requests
  // ------------------------------------------------------------

  p.add("request_start",   request_start);
  p.add("request_stop",    request_stop);
  p.add("request_recover", request_recover);

  // ------------------------------------------------------------
  // Raw clocks (capability + campaign-scoped)
  // ------------------------------------------------------------

  const uint64_t dwt_cycles = clocks_dwt_cycles_now();
  const uint64_t dwt_ns     = clocks_dwt_ns_now();
  const uint64_t gnss_ns    = clocks_gnss_ns_now();
  const uint64_t ocxo_ns    = clocks_ocxo_ns_now();

  p.add("dwt_cycles_now", dwt_cycles);
  p.add("dwt_ns_now",     dwt_ns);
  p.add("gnss_ns_now",    gnss_ns);
  p.add("ocxo_ns_now",    ocxo_ns);

  p.add("gnss_lock", digitalRead(GNSS_LOCK_PIN));

  // ------------------------------------------------------------
  // PPS residual statistics — all clock domains
  // ------------------------------------------------------------
  //
  // For each clock domain, we report:
  //
  //   *_pps_valid     true after the second PPS of a campaign
  //   *_pps_delta     raw tick count between two most recent PPS
  //   *_pps_residual  delta - expected  (signed, most recent)
  //   *_pps_n         number of residual samples collected
  //   *_pps_mean      running mean of residuals
  //   *_pps_stddev    sample standard deviation of residuals
  //   *_pps_stderr    standard error of the mean
  //

  report_residual(p, "dwt",  residual_dwt);
  report_residual(p, "gnss", residual_gnss);
  report_residual(p, "ocxo", residual_ocxo);

  // ------------------------------------------------------------
  // Derived diagnostics (ONLY if meaningful)
  // ------------------------------------------------------------
  //
  // These are NOT authoritative time claims.
  // They exist purely to validate behavior during development.
  //

  if (campaign_state == clocks_campaign_state_t::STARTED && gnss_ns > 0) {

    // --- Tau (dimensionless ratio) ---
    //
    // tau = clock_ns / gnss_ns
    //

    const double tau_dwt  = (double)dwt_ns  / (double)gnss_ns;
    const double tau_ocxo = (double)ocxo_ns / (double)gnss_ns;

    p.add_fmt("tau_dwt",  "%.12f", tau_dwt);
    p.add_fmt("tau_ocxo", "%.12f", tau_ocxo);

    // --- Deviation from GNSS in parts-per-billion ---
    //
    // ppb = (clock_ns - gnss_ns) / gnss_ns * 1e9
    //

    const double ppb_dwt =
      ((double)((int64_t)dwt_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9;

    const double ppb_ocxo =
      ((double)((int64_t)ocxo_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9;

    p.add_fmt("dwt_ppb",  "%.3f", ppb_dwt);
    p.add_fmt("ocxo_ppb", "%.3f", ppb_ocxo);

  } else {
    // Explicitly report absence of meaning
    p.add("tau_dwt",  0.0);
    p.add("tau_ocxo", 0.0);
    p.add("dwt_ppb",  0.0);
    p.add("ocxo_ppb", 0.0);
  }

  return p;
}


// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",   cmd_start   },
  { "STOP",    cmd_stop    },
  { "RECOVER", cmd_recover },
  { "REPORT",  cmd_report  },
  { nullptr,   nullptr     }
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
// Initialization
// ============================================================================

void process_clocks_init(void) {

  dwt_enable();
  arm_gpt2_external();    // GNSS VCLOCK → GPT2 pin 14, raw 10 MHz, polled
  arm_gpt1_external();    // OCXO 10 MHz → GPT1 pin 25, raw 10 MHz, polled

  pinMode(GNSS_PPS_PIN, INPUT);
  pinMode(GNSS_LOCK_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(GNSS_PPS_PIN),
    pps_isr,
    RISING
  );
}