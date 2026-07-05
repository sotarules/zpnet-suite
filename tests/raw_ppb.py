"""
ZPNet Raw PPB — naive vs GNSS-confession-corrected running PPB.

Reads TIMEBASE rows for one campaign and prints one line per campaign PPS row
for the two OCXO lanes.  Every row shows the one-second fast residual and the
running cumulative PPB two ways, side by side:

  naive      — the residual exactly as measured against the PPS/VCLOCK
               reference (what raw_ppb always showed).
  corrected  — the same residual with the GF-8802's own published PPS
               placement change subtracted:

                   res_corrected(t) = res(t) - [g_pps_err(t) - g_pps_err(t-1)]

The idea: g_pps_err is the receiver's running confession of where its PPS
pulse actually landed relative to true GNSS time.  When that value is FLAT the
yardstick's frequency is honest (whatever its level).  When it MOVES, the
interval between pulses was stretched or shrunk by exactly the amount it
moved, and that amount is deposited into every clock's residual as a fake
frequency excursion.  Subtracting the per-row change intercepts the deposit
before it enters the ledger, so corrected PPB holds steady through receiver
steering events instead of walking away and slowly crawling back at 1/T.

The correction is always on.  Both columns are always printed so the naive
surface remains auditable.

Usage:
    python -m zpnet.tests.raw_ppb <campaign_name>
    python raw_ppb.py <campaign_name>

Columns:
    pps       Campaign PPS/VCLOCK row identity.
    o1_res    OCXO1 one-second fast residual, ns.  Positive means fast.
    o1_cres   OCXO1 corrected residual (confession delta subtracted).
    o1_ppb    Running cumulative mean of naive residuals (ns/s == ppb).
    o1_cppb   Running cumulative mean of corrected residuals.
    o2_*      Same four for OCXO2.
    g_err     gnss.pps_timing_error_ns — the receiver's confession: its own
              estimate of this PPS pulse's placement error vs true GNSS time.
    d_g       Per-row change in g_err.  This exact value is what gets
              subtracted from each lane's residual on this row.
    g_acc     gnss.estimated_accuracy_ns — receiver's estimate of its own
              time accuracy (context for how much UNconfessed wander exists).
    g_frq     gnss.freq_error_ppb — receiver's coarse estimate of its output
              frequency error (quantized; corroborates events).
    g_clk     gnss.clock_drift_ppb — receiver internal crystal drift vs GNSS
              (thermal proxy for the receiver).
    g_raw     extra_clocks.gnss_raw_drift_ppb — raw receiver crystal drift
              (another thermal proxy; useful against slow common-mode swell).
    fl        Row flags:
                c   correction applied this row (d_g != 0)
                S   confession step exceeded the segment gate (treated as a
                    solution redefinition, NOT corrected; segment boundary)
                g   PPS gap before this row (no correction across a gap)
                m   g_err missing this row (no correction possible)

What the correction does NOT fix: only confessed error is corrected.  Slow
reference/environment wander that g_pps_err does not narrate passes through
both columns identically and is only tamed by integration time.

This report intentionally computes no tau and no Welfords.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
LANE_KEYS = {"OCXO1": "ocxo1", "OCXO2": "ocxo2"}
LANE_MICRO_PREFIXES = {"ocxo1": "o1", "ocxo2": "o2"}

# A single-row confession step at or beyond this many ns is treated as a
# receiver solution redefinition (reacquisition / mode change), not steering.
# Delta-correction is only lawful for steering; redefinitions are segment
# boundaries and are flagged instead of corrected.
SEGMENT_STEP_GATE_NS = 50.0


# -----------------------------------------------------------------------------
# Database and schema helpers
# -----------------------------------------------------------------------------


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch TIMEBASE rows in campaign PPS order."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'teensy_pps_vclock_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'teensy_pps_vclock_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'teensy_pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'fragment', 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'fragment', 'teensy_pps_vclock_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'fragment', 'teensy_pps_count'), '')::bigint
            ) ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            out.append(payload)
    return out


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics_root(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    forensic = root.get("forensics")
    return forensic if isinstance(forensic, dict) else {}


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        out = _as_int(value)
        if out is not None:
            return out
    return None


def _as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _first_float(*values: Any) -> Optional[float]:
    for value in values:
        out = _as_float(value)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 3, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _round_nearest_int(value: float) -> int:
    """Round like firmware integer-ns rendering, away from zero at half."""
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


# -----------------------------------------------------------------------------
# TIMEBASE extraction
# -----------------------------------------------------------------------------


def pps_count_from_schema(root: Dict[str, Any],
                          frag: Dict[str, Any],
                          forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
        forensic.get("teensy_pps_vclock_count"),
        forensic.get("teensy_pps_count"),
    )


def selected_reference_cycles(root: Dict[str, Any],
                              frag: Dict[str, Any],
                              forensic: Dict[str, Any]) -> Optional[int]:
    """Selected PPS/VCLOCK DWT edge interval for residual fallback."""
    return _first_int(
        _nested_get(forensic, "pps_vclock_edge", "dwt_cycles_between_edges"),
        _nested_get(frag, "pps_vclock_edge", "dwt_cycles_between_edges"),
        forensic.get("dwt_cycles_between_pps_vclock"),
        frag.get("dwt_cycles_between_pps_vclock"),
        root.get("dwt_cycles_between_pps_vclock"),
        _nested_get(frag, "dwt", "cycles_between_pps_vclock"),
        _nested_get(forensic, "pps_vclock_edge", "effective_dwt_cycles_per_second"),
    )


def science_from_schema(frag: Dict[str, Any], clock: str) -> Dict[str, Any]:
    key = LANE_KEYS.get(clock)
    if not key:
        return {}
    obj = _nested_get(frag, key, "science")
    return obj if isinstance(obj, dict) else {}


def lane_alpha_forensics(root: Dict[str, Any],
                         frag: Dict[str, Any],
                         forensic: Dict[str, Any],
                         lane: str) -> Dict[str, Any]:
    candidates = (
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
        _nested_get(forensic, lane, "dwt_forensics"),
        _nested_get(frag, lane, "dwt_forensics"),
        _nested_get(root, "clock_forensics", lane, "alpha_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event"),
        _nested_get(forensic, lane, "alpha_event"),
        _nested_get(frag, lane, "alpha_event"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def _micro_first_int(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str,
                     *suffixes: str) -> Optional[int]:
    prefix = LANE_MICRO_PREFIXES.get(lane)
    if not prefix:
        return None

    values: List[Any] = []
    for suffix in suffixes:
        key = f"{prefix}_{suffix}"
        for source in (forensic, frag, root):
            if isinstance(source, dict):
                values.append(source.get(key))
    return _first_int(*values)


def gnss_discipline_fields(root: Dict[str, Any],
                           frag: Dict[str, Any]) -> Dict[str, Optional[float]]:
    """GNSS receiver discipline fields for reference-movement accounting."""
    gnss = root.get("gnss") if isinstance(root.get("gnss"), dict) else {}
    frag_gnss = frag.get("gnss") if isinstance(frag.get("gnss"), dict) else {}
    extra = root.get("extra_clocks") if isinstance(root.get("extra_clocks"), dict) else {}

    return {
        "g_err": _first_float(gnss.get("pps_timing_error_ns"), frag_gnss.get("pps_timing_error_ns")),
        "g_acc": _first_float(gnss.get("estimated_accuracy_ns"), frag_gnss.get("estimated_accuracy_ns")),
        "g_frq": _first_float(gnss.get("freq_error_ppb"), frag_gnss.get("freq_error_ppb")),
        "g_clk": _first_float(gnss.get("clock_drift_ppb"), frag_gnss.get("clock_drift_ppb")),
        "g_raw": _first_float(extra.get("gnss_raw_drift_ppb")),
    }


def fast_residual_ns(reference_cycles: Optional[int], clock_cycles: Optional[int]) -> Optional[int]:
    """Positive-fast ns residual from cycle interval comparison."""
    if reference_cycles is None or clock_cycles is None or reference_cycles <= 0:
        return None
    exact = (float(reference_cycles) - float(clock_cycles)) * float(NS_PER_SECOND) / float(reference_cycles)
    return _round_nearest_int(exact)


def ocxo_residual(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  clock: str,
                  fallback_ref_cycles: Optional[int]) -> Optional[int]:
    lane = LANE_KEYS[clock]
    sci = science_from_schema(frag, clock)
    f = lane_alpha_forensics(root, frag, forensic, lane)

    residual = _first_int(
        sci.get("delta_raw_fast_residual_ns"),
        sci.get("fast_residual_ns"),
        _nested_get(frag, lane, "pps_residual", "fast_residual_ns"),
        _nested_get(root, "fragment", lane, "pps_residual", "fast_residual_ns"),
    )
    if residual is not None:
        return residual

    cycles = _first_int(
        sci.get("delta_raw_clock_interval_cycles"),
        sci.get("clock_observed_interval_cycles"),
        sci.get("clock_floorline_interval_cycles"),
        _micro_first_int(root, frag, forensic, lane, "pub_cyc", "eff"),
        f.get("dwt_cycles_between_edges"),
        _nested_get(f, "dwt_interval_gate", "effective_cycles"),
    )
    ref = _first_int(sci.get("delta_raw_reference_interval_cycles"), fallback_ref_cycles)
    return fast_residual_ns(ref, cycles)


# -----------------------------------------------------------------------------
# Analysis
# -----------------------------------------------------------------------------


class RunningMean:
    def __init__(self) -> None:
        self.n = 0
        self.total = 0.0

    def add(self, v: float) -> None:
        self.n += 1
        self.total += float(v)

    @property
    def mean(self) -> Optional[float]:
        return (self.total / self.n) if self.n else None


def collect_rows(records: Iterable[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    stats: Dict[str, Any] = {
        "records_seen": 0,
        "rows_collected": 0,
        "gaps": 0,
        "corrected_rows": 0,
        "segment_steps": 0,
        "g_err_missing": 0,
        "correction_abs_total_ns": 0.0,
        "g_err_first": None,
        "g_err_last": None,
    }

    naive = {"OCXO1": RunningMean(), "OCXO2": RunningMean()}
    corrected = {"OCXO1": RunningMean(), "OCXO2": RunningMean()}

    prev_pps: Optional[int] = None
    prev_g_err: Optional[float] = None

    for rec in records:
        stats["records_seen"] += 1
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics_root(rec)
        pps = pps_count_from_schema(root, frag, forensic)
        if pps is None:
            continue

        gap = prev_pps is not None and pps != prev_pps + 1
        if gap:
            stats["gaps"] += 1
            # A gap breaks confession continuity; do not delta across it.
            prev_g_err = None

        ref = selected_reference_cycles(root, frag, forensic)
        o1_res = ocxo_residual(root, frag, forensic, "OCXO1", ref)
        o2_res = ocxo_residual(root, frag, forensic, "OCXO2", ref)
        g = gnss_discipline_fields(root, frag)
        g_err = g["g_err"]

        if g_err is not None:
            if stats["g_err_first"] is None:
                stats["g_err_first"] = g_err
            stats["g_err_last"] = g_err

        # --- confession delta ---------------------------------------------
        d_g: Optional[float] = None
        segment_step = False
        if g_err is None:
            stats["g_err_missing"] += 1
        elif prev_g_err is not None:
            d_g = g_err - prev_g_err
            if abs(d_g) >= SEGMENT_STEP_GATE_NS:
                # Solution redefinition, not steering.  Correction is not
                # lawful; mark the boundary and pass the row through naive.
                segment_step = True
                stats["segment_steps"] += 1
                d_g = None

        correction = d_g if d_g is not None else 0.0
        if d_g is not None and d_g != 0.0:
            stats["corrected_rows"] += 1
            stats["correction_abs_total_ns"] += abs(d_g)

        # --- residuals, naive and corrected --------------------------------
        row: Dict[str, Any] = {
            "pps": pps,
            "gap": gap,
            "segment_step": segment_step,
            "g_err_missing": g_err is None,
            "d_g": d_g,
            **g,
        }

        for clock, res in (("OCXO1", o1_res), ("OCXO2", o2_res)):
            prefix = "o1" if clock == "OCXO1" else "o2"
            row[f"{prefix}_res"] = res
            if res is not None:
                naive[clock].add(res)
                cres = float(res) - correction
                corrected[clock].add(cres)
                row[f"{prefix}_cres"] = cres
            else:
                row[f"{prefix}_cres"] = None
            row[f"{prefix}_ppb"] = naive[clock].mean
            row[f"{prefix}_cppb"] = corrected[clock].mean

        rows.append(row)
        stats["rows_collected"] += 1
        prev_pps = pps
        if g_err is not None:
            prev_g_err = g_err

    stats["naive"] = naive
    stats["corrected"] = corrected
    return rows, stats


# -----------------------------------------------------------------------------
# Output
# -----------------------------------------------------------------------------


def _row_flags(row: Dict[str, Any]) -> str:
    flags = ""
    if row.get("d_g") not in (None, 0.0):
        flags += "c"
    if row.get("segment_step"):
        flags += "S"
    if row.get("gap"):
        flags += "g"
    if row.get("g_err_missing"):
        flags += "m"
    return flags if flags else "."


def print_table(rows: List[Dict[str, Any]]) -> None:
    columns = [
        ("pps", lambda r: _fmt_int(r.get("pps"))),
        ("o1_res", lambda r: _fmt_int(r.get("o1_res"), signed=True)),
        ("o1_cres", lambda r: _fmt_float(r.get("o1_cres"), decimals=1, signed=True)),
        ("o1_ppb", lambda r: _fmt_float(r.get("o1_ppb"), signed=True)),
        ("o1_cppb", lambda r: _fmt_float(r.get("o1_cppb"), signed=True)),
        ("o2_res", lambda r: _fmt_int(r.get("o2_res"), signed=True)),
        ("o2_cres", lambda r: _fmt_float(r.get("o2_cres"), decimals=1, signed=True)),
        ("o2_ppb", lambda r: _fmt_float(r.get("o2_ppb"), signed=True)),
        ("o2_cppb", lambda r: _fmt_float(r.get("o2_cppb"), signed=True)),
        ("g_err", lambda r: _fmt_float(r.get("g_err"), decimals=1, signed=True)),
        ("d_g", lambda r: _fmt_float(r.get("d_g"), decimals=1, signed=True)),
        ("g_acc", lambda r: _fmt_float(r.get("g_acc"), decimals=1)),
        ("g_frq", lambda r: _fmt_float(r.get("g_frq"), decimals=1, signed=True)),
        ("g_clk", lambda r: _fmt_float(r.get("g_clk"), decimals=3, signed=True)),
        ("g_raw", lambda r: _fmt_float(r.get("g_raw"), decimals=3, signed=True)),
        ("fl", lambda r: _row_flags(r)),
    ]

    rendered = [[fn(row) for _, fn in columns] for row in rows]
    widths: List[int] = []
    for i, (name, _) in enumerate(columns):
        data_width = max((len(row[i]) for row in rendered), default=0)
        widths.append(max(len(name), data_width))

    print("  ".join(name.rjust(widths[i]) for i, (name, _) in enumerate(columns)))
    print("  ".join(("─" * widths[i]) for i in range(len(columns))))
    for row in rendered:
        print("  ".join(row[i].rjust(widths[i]) for i in range(len(columns))))


def analyze(campaign: str) -> None:
    records = fetch_timebase(campaign)
    rows, stats = collect_rows(records)

    print(f"ZPNet raw_ppb — campaign={campaign}")
    print(f"records={stats['records_seen']:,}  rows={stats['rows_collected']:,}  gaps={stats['gaps']:,}")
    print()

    if not rows:
        print("No TIMEBASE rows found.")
        return

    print_table(rows)

    naive = stats["naive"]
    corrected = stats["corrected"]

    print()
    print("Correction audit")
    print("════════════════")
    print(f"  rows corrected (d_g != 0)     : {stats['corrected_rows']:,}")
    print(f"  total |correction| applied    : {stats['correction_abs_total_ns']:,.1f} ns")
    print(f"  segment steps (>= {SEGMENT_STEP_GATE_NS:.0f} ns gate) : {stats['segment_steps']:,}")
    print(f"  rows with g_err missing       : {stats['g_err_missing']:,}")
    g0, g1 = stats["g_err_first"], stats["g_err_last"]
    if g0 is not None and g1 is not None and naive["OCXO1"].n:
        net = g1 - g0
        print(f"  confession endpoints          : {g0:+.1f} -> {g1:+.1f} ns  (net {net:+.1f} ns)")
        print(f"  max naive-vs-corrected final  : {abs(net) / naive['OCXO1'].n:.4f} ppb "
              f"(net confession / {naive['OCXO1'].n:,} s — the telescoped bound)")

    print()
    print("Final PPB")
    print("═════════")
    for clock, prefix in (("OCXO1", "o1"), ("OCXO2", "o2")):
        n_ppb = naive[clock].mean
        c_ppb = corrected[clock].mean
        d = (c_ppb - n_ppb) if (n_ppb is not None and c_ppb is not None) else None
        print(f"  {clock}: naive {_fmt_float(n_ppb, 0, 3, signed=True)}   "
              f"corrected {_fmt_float(c_ppb, 0, 3, signed=True)}   "
              f"(corrected-naive {_fmt_float(d, 0, 4, signed=True)})")

    print()
    print("Notes")
    print("═════")
    print("  • res_corrected(t) = res(t) - [g_err(t) - g_err(t-1)].  The correction is")
    print("    applied to every row; on flat-confession rows it is zero by construction.")
    print("  • g_err (gnss.pps_timing_error_ns) is the receiver's own running estimate of")
    print("    where its PPS pulse landed relative to true GNSS time — its confession.")
    print("    Its LEVEL is harmless to frequency; only its CHANGES stretch the yardstick.")
    print("  • The two running means converge as the confession returns toward its start:")
    print("    the total possible difference is (final - first confession) / campaign_s.")
    print("  • g_acc (estimated_accuracy_ns) is the receiver's claim about its own time")
    print("    accuracy — a ceiling on how much UNconfessed wander to expect.  The")
    print("    correction only launders confessed error; slow unconfessed wander passes")
    print("    through both columns and is only tamed by integration time.")
    print("  • g_frq (freq_error_ppb) is the receiver's coarse output-frequency-error")
    print("    estimate; it typically pulses during the same events g_err narrates.")
    print("  • g_clk (clock_drift_ppb) and g_raw (extra_clocks.gnss_raw_drift_ppb) are")
    print("    receiver-crystal drift measures — thermal proxies, useful when deciding")
    print("    whether slow common-mode wander is GNSS or enclosure temperature.")
    print("  • Confession steps >= the segment gate are treated as solution")
    print("    redefinitions (reacquisition/mode change): flagged 'S', never corrected.")
    print("  • Positive residual means the OCXO is running fast (project convention).")


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: raw_ppb <campaign_name>")
        raise SystemExit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()