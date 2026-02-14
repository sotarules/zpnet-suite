// ============================================================================
// process_clocks.cpp — ZPNet CLOCKS (Teensy)
// ============================================================================
//
// CLOCKS is a dormant, PPS-anchored measurement instrument.
//
// • Boots fully silent
// • Does NOT track PPS unless a campaign is active or pending
// • Nanosecond clocks are ZERO unless a campaign is STARTED
// • DWT cycle counter is always live (capability, not narration)
//
// START / STOP / RECOVER are *requests*.
// All authoritative state transitions occur on the PPS boundary.
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
// DWT (CPU cycle counter)
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
    if (campaign_state == clocks_campaign_state_t::STARTED) {
      gnss_ticks_10khz++;
      gnss_dwt_baseline = DWT_CYCCNT;
    }
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
    if (campaign_state == clocks_campaign_state_t::STARTED) {
      ocxo_ticks_10khz++;
      ocxo_dwt_baseline = DWT_CYCCNT;
    }
  });

  NVIC_ENABLE_IRQ(IRQ_GPT1);
  GPT1_CR |= GPT_CR_EN;
  gpt1_armed = true;
}

// ============================================================================
// Zeroing (campaign-scoped)
// ============================================================================

static void clocks_zero_all(void) {
  dwt_cycles_64 = 0;
  dwt_last_32   = DWT_CYCCNT;

  gnss_ticks_10khz = 0;
  ocxo_ticks_10khz = 0;

  gnss_dwt_baseline = DWT_CYCCNT;
  ocxo_dwt_baseline = DWT_CYCCNT;

  campaign_seconds = 0;
}

// ============================================================================
// Synthetic nanoseconds (campaign-scoped)
// ============================================================================

static constexpr uint64_t NS_PER_10KHZ = 100000ULL;

uint64_t clocks_gnss_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  uint64_t base = gnss_ticks_10khz * NS_PER_10KHZ;
  uint32_t delta = DWT_CYCCNT - gnss_dwt_baseline;
  uint64_t interp = (delta * 5ull) / 3ull;
  return (interp > NS_PER_10KHZ) ? base + NS_PER_10KHZ : base + interp;
}

uint64_t clocks_ocxo_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  uint64_t base = ocxo_ticks_10khz * NS_PER_10KHZ;
  uint32_t delta = DWT_CYCCNT - ocxo_dwt_baseline;
  uint64_t interp = (delta * 5ull) / 3ull;
  return (interp > NS_PER_10KHZ) ? base + NS_PER_10KHZ : base + interp;
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
        dwt_cycles_64    = recover_dwt_cycles;
        gnss_ticks_10khz = recover_gnss_ns / NS_PER_10KHZ;
        ocxo_ticks_10khz = recover_ocxo_ns / NS_PER_10KHZ;

        gnss_dwt_baseline = DWT_CYCCNT;
        ocxo_dwt_baseline = DWT_CYCCNT;

        request_recover = false;
      }

      if (request_start) {
        clocks_zero_all();
        campaign_state = clocks_campaign_state_t::STARTED;
        request_start = false;
      }

      if (campaign_state == clocks_campaign_state_t::STARTED) {

        Payload p;
        p.add("campaign",         campaign_name);
        p.add("dwt_cycles",       clocks_dwt_cycles_now());
        p.add("dwt_ns",           clocks_dwt_ns_now());
        p.add("gnss_ns",          clocks_gnss_ns_now());
        p.add("ocxo_ns",          clocks_ocxo_ns_now());
        p.add("teensy_pps_count", campaign_seconds);
        p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));

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
