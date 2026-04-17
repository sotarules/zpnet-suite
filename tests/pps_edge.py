"""
ZPNet PPS Edge Report — GNSS timeline comparison

Two GNSS ns values per PPS second, side by side:

  gnss_ns         Cumulative GNSS ns at the VCLOCK synthetic boundary
                  (published at fragment top level by beta; originates
                  from alpha's g_gnss_ns_count_at_pps).  Increments by
                  ~1,000,000,000 ns per PPS by construction of the
                  GNSS-disciplined 10 MHz substrate.

  gnss_ns_at_isr  GNSS ns at the physical PPS edge.  Computed by
                  translating the DWT value captured as the first
                  instruction of pps_gpio_isr through the DWT↔GNSS
                  bridge.  Also increments by ~1e9 per PPS.

  Δgnss_res       gnss_ns[N] - gnss_ns[N-1] - 1,000,000,000
                  Per-second residual of the synthetic boundary's
                  GNSS ns increment.  Reflects bridge translation
                  noise at the boundary moment.

  Δedge_res       gnss_ns_at_isr[N] - gnss_ns_at_isr[N-1] - 1,000,000,000
                  Per-second residual of the physical PPS edge's
                  GNSS ns increment.  Reflects the composite jitter
                  of the GNSS receiver's PPS output plus GPIO ISR
                  entry latency, both funnelled through the bridge.

  edge-synth      gnss_ns_at_isr[N] - gnss_ns[N]
                  Phase offset between the physical PPS edge and the
                  VCLOCK synthetic boundary, same number at the
                  previously-named pps_diag_gpio_minus_synthetic_ns.
                  Constant to first order (locked by GNSS discipline);
                  its jitter is the quality metric.

Summary statistics (Welford):
  • per-second deltas of both timelines
  • edge-synth distribution (absolute offset)
  • edge-synth drift (offset minus campaign mean)

Usage:
    python -m zpnet.tests.pps_edge <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000


# ---------------------------------------------------------------------
# Welford online accumulator
# ---------------------------------------------------------------------

class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        if x < self.min_val:
            self.min_val = x
        if x > self.max_val:
            self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


# ---------------------------------------------------------------------
# DB access
# ---------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::bigint ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        result.append(payload)

    return result


# ---------------------------------------------------------------------
# Fragment helpers — tolerate both top-level and nested fragment dicts
# ---------------------------------------------------------------------

def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    if not isinstance(inner, dict):
        return {}
    frag = inner.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _first_present(d: Dict[str, Any], *keys: str) -> Any:
    """Return the first present value across candidate keys (None if none)."""
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return None


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


# ---------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------

def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 2,
               signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _print_welford(name: str, w: Welford, unit: str = "",
                   decimals: int = 3) -> None:
    if w.n < 2:
        if w.n == 1:
            suf = f" {unit}" if unit else ""
            print(f"  {name}: n=1, value={w.mean:.{decimals}f}{suf}")
        return
    suffix = f" {unit}" if unit else ""
    print(f"  {name}")
    print(f"    n       = {w.n:,}")
    print(f"    mean    = {w.mean:+,.{decimals}f}{suffix}")
    print(f"    stddev  = {w.stddev:,.{decimals}f}{suffix}")
    print(f"    stderr  = {w.stderr:,.{decimals}f}{suffix}")
    print(f"    min     = {w.min_val:+,.{decimals}f}{suffix}")
    print(f"    max     = {w.max_val:+,.{decimals}f}{suffix}")
    print(f"    range   = {w.max_val - w.min_val:,.{decimals}f}{suffix}")
    print()


# ---------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows):,} rows)")
    print(f"NS_PER_SECOND = {NS_PER_SECOND:,}")
    print()
    print("Two GNSS-ns timelines, tracked per PPS:")
    print()
    print("  gnss_ns         = GNSS ns at VCLOCK synthetic boundary")
    print("                    (from alpha's g_gnss_ns_count_at_pps)")
    print("  gnss_ns_at_isr  = GNSS ns at physical PPS edge")
    print("                    (DWT captured first-instruction in")
    print("                    pps_gpio_isr, translated via the bridge)")
    print("  Δgnss_res       = gnss_ns delta minus 1e9 (cycles)")
    print("  Δedge_res       = gnss_ns_at_isr delta minus 1e9 (cycles)")
    print("  edge-synth      = gnss_ns_at_isr - gnss_ns")
    print("                    Phase offset, expected ~constant.")
    print()

    header = (
        f"{'pps':>6s}  "
        f"{'gnss_ns':>16s}  "
        f"{'gnss_ns_at_isr':>16s}  "
        f"{'Δgnss_res':>10s}  "
        f"{'Δedge_res':>10s}  "
        f"{'edge-synth':>15s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*16}  "
        f"{'─'*16}  "
        f"{'─'*10}  "
        f"{'─'*10}  "
        f"{'─'*15}"
    )
    print(header)
    print(sep)

    # Accumulators.
    w_dgnss_res   = Welford()  # synthetic delta minus 1e9
    w_dedge_res   = Welford()  # edge delta minus 1e9
    w_edge_synth  = Welford()  # absolute phase offset
    w_drift       = Welford()  # offset minus campaign mean (second pass)

    prev_gnss: Optional[int] = None
    prev_edge: Optional[int] = None
    prev_pps: Optional[int] = None

    # Retain per-row edge-synth for the second drift pass.
    samples: List[Dict[str, Any]] = []

    shown = 0
    gaps = 0
    missing_edge = 0

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _as_int(_first_present(root, "pps_count", "teensy_pps_count"))
        if pps is None:
            pps = _as_int(_first_present(frag, "pps_count", "teensy_pps_count"))
        if pps is None:
            continue

        # gnss_ns — synthetic boundary, typically top-level.
        gnss_ns = _as_int(_first_present(root, "gnss_ns", "teensy_gnss_ns"))
        if gnss_ns is None:
            gnss_ns = _as_int(_first_present(frag, "gnss_ns", "teensy_gnss_ns"))

        # gnss_ns_at_isr — physical PPS edge, under pps_diag prefix.
        gnss_ns_at_isr = _as_int(frag.get("pps_diag_gnss_ns_at_isr"))

        # Detect gaps — invalidate prev so we don't compute a cross-gap delta.
        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps:,} → {pps:,} ---")
            prev_gnss = None
            prev_edge = None
        prev_pps = pps

        # Compute deltas.
        dgnss_res: Optional[int] = None
        dedge_res: Optional[int] = None
        if gnss_ns is not None and prev_gnss is not None:
            dgnss_res = (gnss_ns - prev_gnss) - NS_PER_SECOND
        if gnss_ns_at_isr is not None and prev_edge is not None:
            dedge_res = (gnss_ns_at_isr - prev_edge) - NS_PER_SECOND

        # Phase offset (edge - synth).
        edge_synth: Optional[int] = None
        if gnss_ns_at_isr is not None and gnss_ns is not None:
            edge_synth = gnss_ns_at_isr - gnss_ns

        # Track missing physical edges (witness not yet populated).
        if gnss_ns_at_isr is None or gnss_ns_at_isr == 0:
            missing_edge += 1

        # Update Welfords.
        if dgnss_res is not None:
            w_dgnss_res.update(float(dgnss_res))
        if dedge_res is not None:
            w_dedge_res.update(float(dedge_res))
        if edge_synth is not None:
            w_edge_synth.update(float(edge_synth))

        samples.append({
            "pps": pps,
            "edge_synth": edge_synth,
        })

        # Print row.
        print(
            f"{pps:>6d}  "
            f"{_fmt_int(gnss_ns, 16)}  "
            f"{_fmt_int(gnss_ns_at_isr, 16)}  "
            f"{_fmt_int(dgnss_res, 10, signed=True)}  "
            f"{_fmt_int(dedge_res, 10, signed=True)}  "
            f"{_fmt_int(edge_synth, 15, signed=True)}"
        )

        # Save for next iteration.
        if gnss_ns is not None:
            prev_gnss = gnss_ns
        if gnss_ns_at_isr is not None and gnss_ns_at_isr != 0:
            prev_edge = gnss_ns_at_isr

        shown += 1
        if limit and shown >= limit:
            break

    # Second pass — drift.
    if w_edge_synth.n >= 2:
        campaign_mean = w_edge_synth.mean
        for s in samples:
            if s["edge_synth"] is not None:
                w_drift.update(float(s["edge_synth"]) - campaign_mean)

    print()
    print(f"Rows shown:    {shown:,}")
    print(f"Gaps:          {gaps:,}")
    print(f"Missing edges: {missing_edge:,}  "
          f"(rows where gnss_ns_at_isr was 0 or absent)")
    print()

    # -----------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------
    print("═" * 72)
    print("Per-second delta residuals (cycles minus 1,000,000,000 ns)")
    print("═" * 72)
    print()
    _print_welford("Δgnss_res (synthetic boundary increment - 1e9)",
                   w_dgnss_res, "ns")
    _print_welford("Δedge_res (physical PPS edge increment - 1e9)",
                   w_dedge_res, "ns")

    print("═" * 72)
    print("Phase offset between physical PPS edge and VCLOCK synthetic boundary")
    print("═" * 72)
    print()
    _print_welford("edge-synth (absolute offset)",
                   w_edge_synth, "ns")
    _print_welford("edge-synth drift (offset minus campaign mean)",
                   w_drift, "ns")

    # -----------------------------------------------------------------
    # Interpretation
    # -----------------------------------------------------------------
    print("═" * 72)
    print("Interpretation")
    print("═" * 72)
    print()

    if w_dgnss_res.n < 2:
        print("  (insufficient data)")
        return

    print(f"  Δgnss_res stddev:   {w_dgnss_res.stddev:10.3f} ns")
    if w_dedge_res.n >= 2:
        print(f"  Δedge_res stddev:   {w_dedge_res.stddev:10.3f} ns")
    if w_drift.n >= 2:
        print(f"  edge-synth stddev:  {w_drift.stddev:10.3f} ns")
    print()

    # Commentary.
    synth_std = w_dgnss_res.stddev
    edge_std  = w_dedge_res.stddev if w_dedge_res.n >= 2 else None
    drift_std = w_drift.stddev if w_drift.n >= 2 else None

    if synth_std < 10.0:
        print("  The synthetic boundary's GNSS-ns increment is effectively")
        print("  deterministic — bridge translation at boundary time is noise-free")
        print("  to the sub-10-ns level.  This is what you'd expect: the bridge")
        print("  uses the most recent PPS-anchored state, so boundary-time")
        print("  extrapolation is sub-nanosecond over a 1-second horizon.")
        print()

    if edge_std is not None:
        if edge_std < 500.0:
            print(f"  The physical PPS edge's GNSS-ns increment has {edge_std:.1f} ns")
            print("  stddev.  This is the composite second-to-second jitter of the")
            print("  GNSS receiver's PPS output plus GPIO ISR entry latency, both")
            print("  translated through the bridge.  A figure this tight indicates")
            print("  the GF-8802's PPS output is well-disciplined and the GPIO ISR")
            print("  latency is consistent.")
        else:
            print(f"  The physical PPS edge's GNSS-ns increment has {edge_std:.1f} ns")
            print("  stddev.  This is higher than expected — likely driven by either")
            print("  GPIO ISR preemption jitter or GNSS PPS output irregularity.")
        print()

    if drift_std is not None:
        if drift_std < 500.0:
            print(f"  The phase offset (edge minus synthetic) has {drift_std:.1f} ns")
            print("  stddev around its mean.  Since both endpoints derive from the")
            print("  same DWT↔GNSS bridge, this stddev is closer to pure ISR-entry")
            print("  latency differential than the prior gpio_minus_synthetic_ns")
            print("  view, which mixed bridge translation noise with synthetic-")
            print("  boundary emission noise.")
        print()

    # Absolute phase offset — mostly informational.
    if w_edge_synth.n >= 2:
        print(f"  Mean phase offset: {w_edge_synth.mean:+,.0f} ns")
        print("  This is the static offset between when VCLOCK thinks the PPS")
        print("  moment is (its synthetic boundary) and when the physical PPS")
        print("  edge actually arrives.  It's whatever phase CH3 happened to be")
        print("  in when the lane was bootstrapped.  Stable offset = healthy")
        print("  GNSS discipline; drift = the lanes aren't sharing a reference.")
        print()


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: pps_edge <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()