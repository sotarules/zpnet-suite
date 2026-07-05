"""
ZPNet Raw Residuals — PhaseLedger vs Delta Cycles side-by-side.

Reads TIMEBASE rows for a campaign and prints, for one OCXO lane, the two
current-generation one-second residual calculations next to each other:

  * Delta Cycles  — same-species DWT edge-to-edge interval comparison from
                    fragment.<clock>.science.delta_raw_* antecedents.
                    Recomputed externally by this report:
                        fast_ns = (ref_cyc - clk_cyc) * 1e9 / ref_cyc
  * PhaseLedger   — CounterLedger integer tick rail plus the 0..99 ns
                    PhaseLedger suffix, from fragment.<clock>.counterledger.
                    The refined fields are Teensy-authored; this report
                    transcribes them (Pi-as-stenographer).

This report intentionally does not compute tau or Welfords.  PPB columns are
simple running means of the per-second fast residuals (1 ns/s == 1 ppb), shown
alongside the firmware campaign-total PPB from fragment.stats.<clock>.ppb.

Usage:
    python -m zpnet.tests.raw_residuals <campaign_name> [clock] [limit]
    python raw_residuals.py <campaign_name> --clock OCXO2 --limit 300
    python raw_residuals.py <campaign_name> OCXO1 200

Clock filter:
    OCXO1, OCXO2   (CounterLedger/PhaseLedger exists only for OCXO lanes)

Column doctrine:
    pps       campaign PPS/VCLOCK row identity.
    ref_cyc   Delta Cycles delayed PPS/VCLOCK reference DWT interval (cycles).
    clk_cyc   Delta Cycles OCXO observed DWT interval (cycles).
    d_cyc     clk_cyc - ref_cyc (Dave-subtraction; negative means OCXO fast).
    gnss_ns   canonical PPS/VCLOCK GNSS nanosecond clock for this row.
    ocxo_ns   PhaseLedger refined OCXO nanosecond clock (integer rail +
              carry + 0..99 ns suffix).  Falls back to the integer-rail ns
              when the refined value is not yet valid ('i' flag).
    off_ns    ocxo_ns - gnss_ns.  Signed accumulated clock offset.
    res_pl    PhaseLedger refined one-second fast residual (ns);
              positive means OCXO fast.
    res_dc    Delta Cycles one-second fast residual (ns, rounded);
              positive means OCXO fast.
    Δres      res_pl - res_dc (rounded).  The side-by-side sanity number.
    fl        row flags:  w  PhaseLedger wrap event this row
                          b  phase resolved near a 00 boundary
                          l  phase lag > 1 row
                          i  refined invalid (integer-rail fallback shown)
                          g  PPS-count gap before this row
    ppb_pl    running mean of res_pl over valid rows (== ppb).
    ppb_dc    running mean of exact res_dc over valid rows (== ppb).
    ppb_tb    firmware campaign-total PPB from fragment.stats.<clock>.ppb.

Note on the one-row phase lag:
    PhaseLedger's sub-tick suffix for PPS row N cannot be resolved until the
    next OCXO one-second edge arrives, so the refined residual stream is the
    true phase walk delayed by one row.  Same-row Δres is therefore expected
    to disagree for exactly one row at any sharp transient even when both
    methods are healthy.  The summary reports the Δres statistics both
    same-row and with res_dc lagged by one row.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
OCXO_CLOCKS = ("OCXO1", "OCXO2")
LANE_KEYS = {"OCXO1": "ocxo1", "OCXO2": "ocxo2"}


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


def _as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def _first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = _as_int(v)
        if out is not None:
            return out
    return None


def _first_float(*values: Any) -> Optional[float]:
    for v in values:
        out = _as_float(v)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 3,
               signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _round_nearest_int(v: Optional[float]) -> Optional[int]:
    """Round like the firmware's published integer ns fields."""
    if v is None:
        return None
    if v >= 0:
        return int(math.floor(v + 0.5))
    return int(math.ceil(v - 0.5))


def normalize_clock(clock: str) -> str:
    c = (clock or "").strip().upper()
    if c in ("O1", "OCXO1"):
        return "OCXO1"
    if c in ("O2", "OCXO2"):
        return "OCXO2"
    raise SystemExit(f"clock must be OCXO1 or OCXO2 (got '{clock}')")


# -----------------------------------------------------------------------------
# TIMEBASE field extraction
# -----------------------------------------------------------------------------


def pps_count_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def gnss_ns_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("gnss_ns"),
        frag.get("pps_vclock_gnss_ns_at_edge"),
        root.get("gnss_ns"),
        _nested_get(frag, "gnss", "ns"),
        _nested_get(frag, "vclock", "ns"),
    )


def science_from_schema(frag: Dict[str, Any], clock: str) -> Dict[str, Any]:
    obj = _nested_get(frag, LANE_KEYS[clock], "science")
    return obj if isinstance(obj, dict) else {}


def counterledger_from_schema(frag: Dict[str, Any], clock: str) -> Dict[str, Any]:
    obj = _nested_get(frag, LANE_KEYS[clock], "counterledger")
    return obj if isinstance(obj, dict) else {}


def stats_ppb_from_schema(root: Dict[str, Any],
                          frag: Dict[str, Any],
                          clock: str) -> Optional[float]:
    lane = LANE_KEYS[clock]
    return _first_float(
        _nested_get(frag, "stats", lane, "ppb"),
        _nested_get(root, "fragment", "stats", lane, "ppb"),
    )


# -----------------------------------------------------------------------------
# Per-row extraction
# -----------------------------------------------------------------------------


class Row:
    __slots__ = (
        "pps", "gap",
        "ref_cyc", "clk_cyc", "d_cyc",
        "dc_fast_exact", "dc_fast",
        "gnss_ns", "ocxo_ns", "off_ns",
        "pl_fast", "pl_refined_valid",
        "pl_wrap", "pl_near_boundary", "pl_lag",
        "ppb_tb",
    )

    def __init__(self) -> None:
        self.pps: Optional[int] = None
        self.gap: bool = False
        self.ref_cyc: Optional[int] = None
        self.clk_cyc: Optional[int] = None
        self.d_cyc: Optional[int] = None
        self.dc_fast_exact: Optional[float] = None
        self.dc_fast: Optional[int] = None
        self.gnss_ns: Optional[int] = None
        self.ocxo_ns: Optional[int] = None
        self.off_ns: Optional[int] = None
        self.pl_fast: Optional[int] = None
        self.pl_refined_valid: bool = False
        self.pl_wrap: bool = False
        self.pl_near_boundary: bool = False
        self.pl_lag: Optional[int] = None
        self.ppb_tb: Optional[float] = None


def extract_row(rec: Dict[str, Any], clock: str) -> Optional[Row]:
    root = _root(rec)
    frag = _frag(rec)
    if not frag:
        return None

    row = Row()
    row.pps = pps_count_from_schema(root, frag)
    if row.pps is None:
        return None

    row.gnss_ns = gnss_ns_from_schema(root, frag)
    row.ppb_tb = stats_ppb_from_schema(root, frag, clock)

    # --- Delta Cycles: external recompute from science cycle antecedents ---
    sci = science_from_schema(frag, clock)
    if sci and bool(sci.get("valid")) and bool(sci.get("delta_raw_valid", False)):
        ref = _as_int(sci.get("delta_raw_reference_interval_cycles"))
        clk = _as_int(sci.get("delta_raw_clock_interval_cycles"))
        if ref is not None and clk is not None and ref > 0 and clk > 0:
            row.ref_cyc = ref
            row.clk_cyc = clk
            row.d_cyc = clk - ref
            fast_exact = float(ref - clk) * float(NS_PER_SECOND) / float(ref)
            row.dc_fast_exact = fast_exact
            row.dc_fast = _round_nearest_int(fast_exact)

    # --- PhaseLedger: transcribe Teensy-authored counterledger fields ---
    cl = counterledger_from_schema(frag, clock)
    if cl and bool(cl.get("initialized", False)):
        refined_valid = bool(cl.get("refined_valid", False))
        row.pl_refined_valid = refined_valid
        ocxo_ns = _as_int(cl.get("refined_ns")) if refined_valid else None
        if ocxo_ns is None:
            sample_count = _as_int(cl.get("sample_count"))
            if sample_count is not None and sample_count > 0:
                ocxo_ns = _as_int(cl.get("ns"))
        row.ocxo_ns = ocxo_ns
        if row.ocxo_ns is not None and row.gnss_ns is not None:
            row.off_ns = row.ocxo_ns - row.gnss_ns

        if bool(cl.get("refined_interval_valid", False)):
            row.pl_fast = _as_int(cl.get("refined_fast_residual_ns"))
        elif bool(cl.get("interval_valid", False)):
            # Integer-rail fallback so the row is not blind while phase warms up.
            row.pl_fast = _as_int(cl.get("fast_residual_ns"))
            row.pl_refined_valid = False

        phase = cl.get("phaseledger")
        if isinstance(phase, dict):
            row.pl_wrap = bool(phase.get("wrap_event", False))
            row.pl_near_boundary = bool(phase.get("near_boundary", False))
            row.pl_lag = _as_int(phase.get("lag_pps"))

    return row


# -----------------------------------------------------------------------------
# Statistics helpers (report-local; not a Welford surface)
# -----------------------------------------------------------------------------


class RunningMean:
    def __init__(self) -> None:
        self.n = 0
        self.total = 0.0

    def add(self, v: float) -> None:
        self.n += 1
        self.total += v

    @property
    def mean(self) -> Optional[float]:
        return (self.total / self.n) if self.n else None


def _min_max_mean(samples: List[float]) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    if not samples:
        return None, None, None
    return min(samples), max(samples), sum(samples) / len(samples)


def _stddev(samples: List[float]) -> Optional[float]:
    n = len(samples)
    if n < 2:
        return None
    mean = sum(samples) / n
    var = sum((s - mean) ** 2 for s in samples) / (n - 1)
    return math.sqrt(var)


# -----------------------------------------------------------------------------
# Report
# -----------------------------------------------------------------------------


HEADER = (
    f"{'pps':>6}  {'ref_cyc':>13}  {'clk_cyc':>13}  {'d_cyc':>6}  "
    f"{'gnss_ns':>16}  {'ocxo_ns':>16}  {'off_ns':>12}  "
    f"{'res_pl':>7}  {'res_dc':>7}  {'Δres':>6}  {'fl':>3}  "
    f"{'ppb_pl':>9}  {'ppb_dc':>9}  {'ppb_tb':>9}"
)

RULE = (
    "──────  ─────────────  ─────────────  ──────  "
    "────────────────  ────────────────  ────────────  "
    "───────  ───────  ──────  ───  "
    "─────────  ─────────  ─────────"
)


def _row_flags(row: Row) -> str:
    flags = ""
    if row.pl_wrap:
        flags += "w"
    if row.pl_near_boundary:
        flags += "b"
    if row.pl_lag is not None and row.pl_lag > 1:
        flags += "l"
    if row.pl_fast is not None and not row.pl_refined_valid:
        flags += "i"
    if row.gap:
        flags += "g"
    return flags if flags else "."


def analyze(campaign: str, clock: str = "OCXO1", limit: int = 0) -> None:
    records = fetch_timebase(campaign)
    if not records:
        print(f"No TIMEBASE rows found for campaign '{campaign}'.")
        return

    rows: List[Row] = []
    prev_pps: Optional[int] = None
    for rec in records:
        row = extract_row(rec, clock)
        if row is None:
            continue
        if prev_pps is not None and row.pps != prev_pps + 1:
            row.gap = True
        prev_pps = row.pps
        rows.append(row)
        if limit and len(rows) >= limit:
            break

    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE rows, view={clock})")
    print()
    print("Raw residuals — PhaseLedger vs Delta Cycles")
    print("═══════════════════════════════════════════")
    print("  res_pl: Teensy-authored PhaseLedger refined fast residual (transcribed).")
    print("  res_dc: Delta Cycles fast residual recomputed from science cycle antecedents:")
    print("          (ref_cyc - clk_cyc) * 1e9 / ref_cyc, rounded.")
    print("  Positive residual means the OCXO is running fast (project sign convention).")
    print("  Δres = res_pl - res_dc.  PhaseLedger's suffix lags one PPS row by design,")
    print("  so transient rows disagree same-row; see the lag-adjusted summary below.")
    print("  ppb_pl/ppb_dc are running means of the fast residuals (1 ns/s == 1 ppb).")
    print("  ppb_tb is the firmware campaign-total from fragment.stats.<clock>.ppb.")
    print()
    print(HEADER)
    print(RULE)

    run_pl = RunningMean()
    run_dc = RunningMean()

    pl_samples: List[float] = []
    dc_samples: List[float] = []
    diff_same_row: List[float] = []
    diff_lagged: List[float] = []

    counts = {
        "rows": 0,
        "pl_valid": 0,
        "pl_integer_fallback": 0,
        "dc_valid": 0,
        "both_valid": 0,
        "wraps": 0,
        "near_boundary": 0,
        "gaps": 0,
    }

    prev_dc_fast_exact: Optional[float] = None
    prev_row_contiguous = False
    last_ppb_tb: Optional[float] = None

    for row in rows:
        counts["rows"] += 1
        if row.gap:
            counts["gaps"] += 1
        if row.pl_wrap:
            counts["wraps"] += 1
        if row.pl_near_boundary:
            counts["near_boundary"] += 1
        if row.ppb_tb is not None:
            last_ppb_tb = row.ppb_tb

        if row.pl_fast is not None:
            if row.pl_refined_valid:
                counts["pl_valid"] += 1
            else:
                counts["pl_integer_fallback"] += 1
            run_pl.add(float(row.pl_fast))
            pl_samples.append(float(row.pl_fast))

        if row.dc_fast_exact is not None:
            counts["dc_valid"] += 1
            run_dc.add(row.dc_fast_exact)
            dc_samples.append(row.dc_fast_exact)

        d_res: Optional[int] = None
        if row.pl_fast is not None and row.dc_fast is not None:
            counts["both_valid"] += 1
            d_res = row.pl_fast - row.dc_fast
            diff_same_row.append(float(row.pl_fast) - row.dc_fast_exact)
            if (prev_dc_fast_exact is not None and prev_row_contiguous
                    and not row.gap):
                diff_lagged.append(float(row.pl_fast) - prev_dc_fast_exact)

        prev_row_contiguous = row.dc_fast_exact is not None
        prev_dc_fast_exact = row.dc_fast_exact

        print(
            f"{_fmt_int(row.pps, 6)}  "
            f"{_fmt_int(row.ref_cyc, 13)}  "
            f"{_fmt_int(row.clk_cyc, 13)}  "
            f"{_fmt_int(row.d_cyc, 6, signed=True)}  "
            f"{_fmt_int(row.gnss_ns, 16)}  "
            f"{_fmt_int(row.ocxo_ns, 16)}  "
            f"{_fmt_int(row.off_ns, 12, signed=True)}  "
            f"{_fmt_int(row.pl_fast, 7, signed=True)}  "
            f"{_fmt_int(row.dc_fast, 7, signed=True)}  "
            f"{_fmt_int(d_res, 6, signed=True)}  "
            f"{_row_flags(row):>3}  "
            f"{_fmt_float(run_pl.mean, 9, 3, signed=True)}  "
            f"{_fmt_float(run_dc.mean, 9, 3, signed=True)}  "
            f"{_fmt_float(row.ppb_tb, 9, 3, signed=True)}"
        )

    print()
    print("Row accounting")
    print("══════════════")
    print(f"  rows={counts['rows']}  gaps={counts['gaps']}")
    print(f"  PhaseLedger refined rows={counts['pl_valid']}  "
          f"integer-rail fallback rows={counts['pl_integer_fallback']}")
    print(f"  Delta Cycles rows={counts['dc_valid']}  "
          f"both-valid rows={counts['both_valid']}")
    print(f"  wrap events={counts['wraps']}  near-boundary rows={counts['near_boundary']}")

    pl_min, pl_max, pl_mean = _min_max_mean(pl_samples)
    dc_min, dc_max, dc_mean = _min_max_mean(dc_samples)
    print()
    print("Residual summary (fast, ns)")
    print("═══════════════════════════")
    print(f"  PhaseLedger : n={len(pl_samples)}  "
          f"mean={_fmt_float(pl_mean, 0, 3, signed=True)}  "
          f"min={_fmt_float(pl_min, 0, 0, signed=True)}  "
          f"max={_fmt_float(pl_max, 0, 0, signed=True)}  "
          f"stddev={_fmt_float(_stddev(pl_samples), 0, 3)}")
    print(f"  Delta Cycles: n={len(dc_samples)}  "
          f"mean={_fmt_float(dc_mean, 0, 3, signed=True)}  "
          f"min={_fmt_float(dc_min, 0, 0, signed=True)}  "
          f"max={_fmt_float(dc_max, 0, 0, signed=True)}  "
          f"stddev={_fmt_float(_stddev(dc_samples), 0, 3)}")

    print()
    print("Concordance (res_pl - res_dc, ns)")
    print("═════════════════════════════════")
    sr_min, sr_max, sr_mean = _min_max_mean(diff_same_row)
    lg_min, lg_max, lg_mean = _min_max_mean(diff_lagged)
    print(f"  same-row : n={len(diff_same_row)}  "
          f"mean={_fmt_float(sr_mean, 0, 3, signed=True)}  "
          f"min={_fmt_float(sr_min, 0, 1, signed=True)}  "
          f"max={_fmt_float(sr_max, 0, 1, signed=True)}  "
          f"stddev={_fmt_float(_stddev(diff_same_row), 0, 3)}")
    print(f"  dc lag 1 : n={len(diff_lagged)}  "
          f"mean={_fmt_float(lg_mean, 0, 3, signed=True)}  "
          f"min={_fmt_float(lg_min, 0, 1, signed=True)}  "
          f"max={_fmt_float(lg_max, 0, 1, signed=True)}  "
          f"stddev={_fmt_float(_stddev(diff_lagged), 0, 3)}")
    print("  A near-zero mean in either alignment says the two methods measure")
    print("  the same physics; a persistent bias isolates a systematic in one method.")

    print()
    print("Final PPB")
    print("═════════")
    print(f"  ppb_pl (running mean) = {_fmt_float(run_pl.mean, 0, 3, signed=True)}")
    print(f"  ppb_dc (running mean) = {_fmt_float(run_dc.mean, 0, 3, signed=True)}")
    print(f"  ppb_tb (TIMEBASE)     = {_fmt_float(last_ppb_tb, 0, 3, signed=True)}")

    print()
    print("Notes")
    print("═════")
    print("  • This report recomputes Delta Cycles from persisted science cycle")
    print("    antecedents and transcribes PhaseLedger values authored by the Teensy.")
    print("  • res_pl at row N carries the phase suffix resolved for row N-1 by")
    print("    design (the closing OCXO edge arrives after the PPS).  The lag-")
    print("    adjusted concordance line accounts for this.")
    print("  • Rows flagged 'i' show the integer CounterLedger rail (100 ns")
    print("    quantized) because the refined interval was not valid on that row.")


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------


def parse_args(argv: List[str]) -> Tuple[str, str, int]:
    if len(argv) < 2:
        print("Usage: raw_residuals <campaign_name> [clock] [limit]")
        print("       raw_residuals <campaign_name> --clock OCXO2 --limit 300")
        print("       raw_residuals <campaign_name> OCXO1 200")
        raise SystemExit(1)

    campaign = argv[1]
    clock = "OCXO1"
    limit = 0

    i = 2
    while i < len(argv):
        arg = argv[i]
        if arg == "--clock":
            if i + 1 >= len(argv):
                raise SystemExit("--clock requires OCXO1 or OCXO2")
            clock = argv[i + 1]
            i += 2
            continue
        if arg.startswith("--clock="):
            clock = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--limit":
            if i + 1 >= len(argv):
                raise SystemExit("--limit requires an integer")
            try:
                limit = int(argv[i + 1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 2
            continue
        if arg.startswith("--limit="):
            try:
                limit = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 1
            continue
        upper = arg.upper()
        if upper in OCXO_CLOCKS or upper in ("O1", "O2"):
            clock = arg
            i += 1
            continue
        try:
            limit = int(arg)
        except ValueError as exc:
            raise SystemExit(f"unknown argument '{arg}'") from exc
        i += 1

    return campaign, normalize_clock(clock), limit


def main() -> None:
    campaign, clock, limit = parse_args(sys.argv)
    analyze(campaign, clock=clock, limit=limit)


if __name__ == "__main__":
    main()