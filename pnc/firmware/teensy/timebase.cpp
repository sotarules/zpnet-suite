// ============================================================================
// timebase.cpp — ZPNet Timebase Singleton (Teensy)
// ============================================================================
//
// See timebase.h for architecture and API documentation.
//
// Implementation notes:
//
//   The GNSS interpolation uses a fixed-point ratio to avoid division
//   in the hot path.  The DWT-to-GNSS conversion factor is:
//
//     GNSS runs at 10 MHz  → 100 ns per GNSS tick
//     DWT runs at 1.008 GHz → ~0.992 ns per DWT cycle
//
//   For now() we need: elapsed_gnss_ns = elapsed_dwt_cycles * (100 / 100.8)
//
//   In fixed-point (shift 32):
//     ratio = (100 << 32) / 100.8 = (NS_PER_GNSS_TICK << 32) / NS_PER_DWT_TICK_x10
//
//   But we avoid floating point entirely.  The exact relationship is:
//     10 MHz GNSS = 100 ns per tick
//     1.008 GHz DWT = 1008 cycles per µs
//     So 1 DWT cycle = 1,000,000,000 / 1,008,000,000 GNSS ns
//                     = 125 / 126 GNSS ns
//
//   For N DWT cycles elapsed: gnss_ns_elapsed = N * 125 / 126
//
//   This is exact integer arithmetic.  No floating point.  No shift.
//   The division by 126 is a single integer divide, which on Cortex-M7
//   is 2-12 cycles via UDIV.
//
//   Torn-read protection:
//     The anchor is a 64-bit gnss_ns + 32-bit dwt_snap, written by the
//     10 KHz ISR.  On Cortex-M7, 64-bit writes are NOT atomic.  A reader
//     in scheduled context could see a torn gnss_ns (low half updated,
//     high half stale).  We use a sequence counter to detect this:
//     the ISR increments the counter before and after writing.  The reader
//     retries if the counter changed or is odd (write in progress).
//
// ============================================================================

#include "timebase.h"
#include "process.h"
#include "payload.h"
#include "debug.h"

#include <Arduino.h>
#include <stdlib.h>
#include "imxrt.h"

// ============================================================================
// Constants
// ============================================================================

// DWT-to-GNSS conversion: 1 DWT cycle = 125/126 GNSS nanoseconds.
// Derived from: DWT @ 1.008 GHz, GNSS @ 10 MHz (100 ns per tick).
//   1 DWT cycle = 1e9 / 1.008e9 ns = 125/126 ns exactly.
static constexpr uint32_t DWT_TO_GNSS_NUM = 125;
static constexpr uint32_t DWT_TO_GNSS_DEN = 126;

// DWT nominal frequency for direct ns conversion
static constexpr uint64_t DWT_FREQ_HZ = 1008000000ULL;

// Nanoseconds per second
static constexpr uint64_t NS_PER_SECOND = 1000000000ULL;

// ============================================================================
// Anchor state (written by TIMEPULSE ISR, read by anyone)
// ============================================================================

struct anchor_t {
  volatile uint64_t gnss_ns;
  volatile uint32_t dwt_snap;
  volatile uint32_t seq;        // sequence counter for torn-read detection
};

static anchor_t anchor = {0, 0, 0};
static volatile bool anchor_valid = false;

// ============================================================================
// Fragment state (written by pubsub callback, read by anyone)
// ============================================================================

static timebase_fragment_t frag = {};

// ============================================================================
// ARM CMSIS intrinsic
// ===========================================================================

static inline void dmb(void) { __asm volatile("dmb" ::: "memory"); }

// ============================================================================
// TIMEPULSE anchor update (ISR context only)
// ============================================================================

void timebase_update_anchor(uint64_t gnss_ns, uint32_t dwt_snap) {
  anchor.seq++;               // odd = write in progress
  dmb();
  anchor.gnss_ns  = gnss_ns;
  anchor.dwt_snap = dwt_snap;
  dmb();
  anchor.seq++;               // even = write complete
  anchor_valid = true;
}

void timebase_invalidate(void) {
  anchor_valid = false;
}

// ============================================================================
// Anchor read with torn-read protection
// ============================================================================

struct anchor_snapshot_t {
  uint64_t gnss_ns;
  uint32_t dwt_snap;
  bool     ok;
};

static inline anchor_snapshot_t read_anchor(void) {
  anchor_snapshot_t snap;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t s1 = anchor.seq;
    dmb();
    snap.gnss_ns  = anchor.gnss_ns;
    snap.dwt_snap = anchor.dwt_snap;
    dmb();
    uint32_t s2 = anchor.seq;

    if (s1 == s2 && (s1 & 1) == 0) {
      snap.ok = true;
      return snap;
    }
  }

  snap.ok = false;
  return snap;
}

// ============================================================================
// now_ns — the core API
// ============================================================================

int64_t timebase_now_ns(const char* domain) {
  if (!domain) return -1;

  // --- GNSS domain: interpolate from TIMEPULSE anchor ---
  if (domain[0] == 'G') {
    if (!anchor_valid) return -1;

    anchor_snapshot_t snap = read_anchor();
    if (!snap.ok) return -1;

    // Elapsed DWT cycles since anchor (unsigned 32-bit wrap-safe)
    uint32_t dwt_now = ARM_DWT_CYCCNT;
    uint32_t dwt_elapsed = dwt_now - snap.dwt_snap;

    // Convert DWT cycles to GNSS nanoseconds:
    //   gnss_ns_elapsed = dwt_elapsed * 125 / 126
    //
    // For dwt_elapsed up to ~100,000 cycles (100 µs at 1.008 GHz),
    // dwt_elapsed * 125 fits easily in 32 bits (max ~12.5M).
    // Even for longer intervals (e.g., 1 ms = ~1,008,000 cycles),
    // 1,008,000 * 125 = 126,000,000 — still well within 32 bits.
    //
    // Guard: if dwt_elapsed exceeds ~34M cycles (~34 ms), the
    // multiply could overflow 32 bits.  This should never happen
    // since the anchor updates at 10 KHz (100 µs), but we use
    // 64-bit arithmetic for safety.
    uint64_t gnss_elapsed_ns = (uint64_t)dwt_elapsed * DWT_TO_GNSS_NUM / DWT_TO_GNSS_DEN;

    return (int64_t)(snap.gnss_ns + gnss_elapsed_ns);
  }

  // --- DWT domain: direct register read, nominal conversion ---
  if (domain[0] == 'D') {
    // DWT nanoseconds from campaign start.
    // This requires the fragment to have valid dwt_ns as a base.
    // For now, return the fragment's dwt_ns plus interpolation
    // from the fragment's DWT cycles to current DWT_CYCCNT.
    if (!frag.valid) return -1;

    //uint32_t dwt_now = ARM_DWT_CYCCNT;
    // We don't have a DWT snap at the PPS edge stored in the anchor,
    // so use the fragment's dwt_cycles (which is the 64-bit DWT at PPS).
    // This is a point-in-time value, not interpolated.
    // For true DWT now_ns, we would need a DWT anchor similar to GNSS.
    // For this first cut, return the fragment value (PPS-edge granularity).
    return (int64_t)frag.dwt_ns;
  }

  // --- Unknown domain ---
  return -1;
}

// ============================================================================
// Validity check
// ============================================================================

bool timebase_valid(void) {
  return anchor_valid;
}

// ============================================================================
// Fragment access
// ============================================================================

const timebase_fragment_t* timebase_last_fragment(void) {
  return &frag;
}

// ============================================================================
// TIMEBASE_FRAGMENT subscription callback
// ============================================================================
//
// Called once per second from pubsub dispatch (scheduled context).
// Parses the fragment payload into integer fields for fast access.
//

// Helper: parse uint64 from Payload string field
static uint64_t payload_u64(const Payload& p, const char* key) {
  const char* s = p.getString(key);
  if (!s) return 0;
  return strtoull(s, nullptr, 10);
}

// Helper: parse int32 from Payload with default
static int32_t payload_i32(const Payload& p, const char* key) {
  return p.getInt(key, 0);
}

void on_timebase_fragment(const Payload& payload) {

  frag.gnss_ns           = payload_u64(payload, "gnss_ns");
  frag.dwt_ns            = payload_u64(payload, "dwt_ns");
  frag.dwt_cycles        = payload_u64(payload, "dwt_cycles");
  frag.ocxo_ns           = payload_u64(payload, "ocxo_ns");
  frag.pps_count         = (uint32_t)payload_u64(payload, "teensy_pps_count");
  frag.isr_residual_dwt  = payload_i32(payload, "isr_residual_dwt");
  frag.isr_residual_gnss = payload_i32(payload, "isr_residual_gnss");
  frag.isr_residual_ocxo = payload_i32(payload, "isr_residual_ocxo");
  frag.valid = true;
}

// ============================================================================
// Initialization
// ============================================================================

void timebase_init(void) {
  anchor       = {0, 0, 0};
  anchor_valid = false;
  frag         = {};
}