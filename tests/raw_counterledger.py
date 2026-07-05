"""
ZPNet Raw CounterLedger — report-only OCXO integer-ledger sanity report.

Reads TIMEBASE rows for one campaign and prints one line per campaign PPS row.
For each OCXO lane it validates the CounterLedger side rail published under:

    fragment.ocxo1.counterledger
    fragment.ocxo2.counterledger

The report is intentionally observational. It does not recompute public OCXO
science. It checks whether the report-only integer ledger is internally
self-consistent row by row:

    ticks64 * 100 ns == counterledger.ns
    interval_ticks * 100 ns == interval_ns
    interval_ns - 1e9 == fast_residual_ns
    candidate_public_ns - gnss_ns == candidate_minus_gnss_ns
    candidate_public_ns - fragment.<ocxo>.ns == candidate_minus_public_ns
    ticks64 deltas agree with interval_ticks on contiguous rows
    sample_count / pps_sequence advance monotonically on contiguous rows

If the 600-second block-window fields are present, the report also displays
active and completed block PPB witnesses and sanity-checks their mean/tau/ppb
rendering.

Usage:
    python -m zpnet.tests.raw_counterledger <campaign_name>
    python raw_counterledger.py <campaign_name>
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
NS_PER_TICK = 100
LANES = (("o1", "ocxo1"), ("o2", "ocxo2"))


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
        return float(v)
    except (TypeError, ValueError):
        return None


def _as_bool(v: Any) -> Optional[bool]:
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        if v.lower() in ("true", "t", "yes", "1"):
            return True
        if v.lower() in ("false", "f", "no", "0"):
            return False
    if isinstance(v, (int, float)):
        return bool(v)
    return None


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        out = _as_int(value)
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
    if v is None or not math.isfinite(v):
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _fmt_bool(v: Optional[bool]) -> str:
    if v is None:
        return "---"
    return "Y" if v else "N"


def _fmt_flags(flags: List[str]) -> str:
    return "OK" if not flags else ",".join(flags)


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
        _nested_get(frag, "gnss", "ns"),
        root.get("gnss_ns"),
        _nested_get(root, "gnss", "ns"),
    )


def lane_public_ns(frag: Dict[str, Any], lane: str) -> Optional[int]:
    return _first_int(_nested_get(frag, lane, "ns"))


def counterledger_obj(frag: Dict[str, Any], lane: str) -> Dict[str, Any]:
    obj = _nested_get(frag, lane, "counterledger")
    return obj if isinstance(obj, dict) else {}


# -----------------------------------------------------------------------------
# Analysis
# -----------------------------------------------------------------------------


def _near(a: Optional[float], b: Optional[float], tol: float = 0.001) -> bool:
    if a is None or b is None:
        return False
    return abs(a - b) <= tol


def _ppb_from_tau(tau: Optional[float]) -> Optional[float]:
    if tau is None:
        return None
    return (tau - 1.0) * 1_000_000_000.0


def _tau_from_ticks(ticks: Optional[int], intervals: Optional[int]) -> Optional[float]:
    if ticks is None or intervals is None or intervals <= 0:
        return None
    expected_ticks = intervals * 10_000_000
    if expected_ticks <= 0:
        return None
    return float(ticks) / float(expected_ticks)


def _ppb_from_ticks(ticks: Optional[int], intervals: Optional[int]) -> Optional[float]:
    tau = _tau_from_ticks(ticks, intervals)
    return _ppb_from_tau(tau)


def analyze_lane(
    prefix: str,
    lane: str,
    frag: Dict[str, Any],
    gnss_ns: Optional[int],
    prev: Optional[Dict[str, Any]],
    stats: Dict[str, int],
) -> Dict[str, Any]:
    cl = counterledger_obj(frag, lane)
    row: Dict[str, Any] = {}
    flags: List[str] = []

    if not cl:
        stats[f"{prefix}_missing_counterledger"] += 1
        flags.append("MISS")
        row.update({
            "valid": None,
            "seq_match": None,
            "cap_valid": None,
            "pps_seq": None,
            "n": None,
            "ticks": None,
            "ns": None,
            "iticks": None,
            "ires": None,
            "ppb": None,
            "cand_g": None,
            "cand_p": None,
            "bn": None,
            "bppb": None,
            "cbppb": None,
            "flags": _fmt_flags(flags),
        })
        return row

    valid = _as_bool(cl.get("valid"))
    interval_valid = _as_bool(cl.get("interval_valid"))
    seq_match = _as_bool(cl.get("capture_sequence_match"))
    cap_valid = _as_bool(cl.get("capture_valid"))
    enabled = _as_bool(cl.get("enabled"))

    pps_seq = _as_int(cl.get("pps_sequence"))
    sample_count = _as_int(cl.get("sample_count"))
    ticks64 = _as_int(cl.get("ticks64"))
    ns = _as_int(cl.get("ns"))
    interval_ticks = _as_int(cl.get("interval_ticks"))
    interval_ns = _as_int(cl.get("interval_ns"))
    fast_residual_ns = _as_int(cl.get("fast_residual_ns"))
    candidate_public_ns = _as_int(cl.get("candidate_public_ns"))
    candidate_minus_gnss = _as_int(cl.get("candidate_minus_gnss_ns"))
    candidate_minus_public = _as_int(cl.get("candidate_minus_public_ns"))
    public_ns = lane_public_ns(frag, lane)

    # Existing per-row invariants.
    if enabled is False:
        flags.append("DIS")
        stats[f"{prefix}_disabled"] += 1
    if valid is not True:
        flags.append("INV")
        stats[f"{prefix}_invalid"] += 1
    if interval_valid is not True:
        flags.append("IINV")
        stats[f"{prefix}_interval_invalid"] += 1
    if seq_match is not True:
        flags.append("SEQ")
        stats[f"{prefix}_sequence_mismatch"] += 1
    if cap_valid is not True:
        flags.append("CAP")
        stats[f"{prefix}_capture_invalid"] += 1

    expected_ns = ticks64 * NS_PER_TICK if ticks64 is not None else None
    if ns is not None and expected_ns is not None and ns != expected_ns:
        flags.append("NS")
        stats[f"{prefix}_bad_ns"] += 1

    expected_interval_ns = interval_ticks * NS_PER_TICK if interval_ticks is not None else None
    if interval_ns is not None and expected_interval_ns is not None and interval_ns != expected_interval_ns:
        flags.append("INS")
        stats[f"{prefix}_bad_interval_ns"] += 1

    expected_residual = interval_ns - NS_PER_SECOND if interval_ns is not None else None
    if fast_residual_ns is not None and expected_residual is not None and fast_residual_ns != expected_residual:
        flags.append("RES")
        stats[f"{prefix}_bad_residual"] += 1

    expected_cand_g = candidate_public_ns - gnss_ns if candidate_public_ns is not None and gnss_ns is not None else None
    if candidate_minus_gnss is not None and expected_cand_g is not None and candidate_minus_gnss != expected_cand_g:
        flags.append("C-G")
        stats[f"{prefix}_bad_candidate_gnss"] += 1

    expected_cand_p = candidate_public_ns - public_ns if candidate_public_ns is not None and public_ns is not None else None
    if candidate_minus_public is not None and expected_cand_p is not None and candidate_minus_public != expected_cand_p:
        flags.append("C-P")
        stats[f"{prefix}_bad_candidate_public"] += 1

    # Contiguous-row invariants.
    if prev is not None:
        prev_ticks = prev.get(f"{prefix}_ticks")
        prev_n = prev.get(f"{prefix}_n")
        prev_seq = prev.get(f"{prefix}_pps_seq")
        prev_candidate = prev.get(f"{prefix}_candidate_public_ns")
        pps_contig = prev.get("pps") is not None and _as_int(prev.get("pps")) is not None

        if pps_contig:
            if ticks64 is not None and prev_ticks is not None and interval_ticks is not None:
                delta_ticks = ticks64 - int(prev_ticks)
                if delta_ticks != interval_ticks:
                    flags.append("DTK")
                    stats[f"{prefix}_bad_delta_ticks"] += 1
            if ns is not None and prev.get(f"{prefix}_ns") is not None and interval_ns is not None:
                delta_ns = ns - int(prev[f"{prefix}_ns"])
                if delta_ns != interval_ns:
                    flags.append("DNS")
                    stats[f"{prefix}_bad_delta_ns"] += 1
            if sample_count is not None and prev_n is not None and sample_count != int(prev_n) + 1:
                flags.append("DN")
                stats[f"{prefix}_bad_sample_count_step"] += 1
            if pps_seq is not None and prev_seq is not None and pps_seq != int(prev_seq) + 1:
                flags.append("DSEQ")
                stats[f"{prefix}_bad_pps_sequence_step"] += 1
            if candidate_public_ns is not None and prev_candidate is not None and interval_ns is not None:
                delta_candidate = candidate_public_ns - int(prev_candidate)
                if delta_candidate != interval_ns:
                    flags.append("DCP")
                    stats[f"{prefix}_bad_candidate_delta"] += 1

    # Optional block-window fields from the CounterLedger block step.
    block_field_names = (
        "block_window_seconds",
        "block_valid",
        "block_interval_count",
        "block_ticks",
        "block_fast_residual_sum_ns",
        "block_mean_fast_residual_ns",
        "block_tau",
        "block_ppb",
        "completed_block_valid",
        "completed_block_interval_count",
        "completed_block_ticks",
        "completed_block_fast_residual_sum_ns",
        "completed_block_mean_fast_residual_ns",
        "completed_block_tau",
        "completed_block_ppb",
    )
    has_any_block_field = any(name in cl for name in block_field_names)
    if not has_any_block_field:
        stats[f"{prefix}_block_fields_missing"] += 1
    else:
        block_n = _as_int(cl.get("block_interval_count"))
        block_ticks = _as_int(cl.get("block_ticks"))
        block_sum = _as_int(cl.get("block_fast_residual_sum_ns"))
        block_mean = _as_float(cl.get("block_mean_fast_residual_ns"))
        block_tau = _as_float(cl.get("block_tau"))
        block_ppb = _as_float(cl.get("block_ppb"))
        completed_n = _as_int(cl.get("completed_block_interval_count"))
        completed_ticks = _as_int(cl.get("completed_block_ticks"))
        completed_sum = _as_int(cl.get("completed_block_fast_residual_sum_ns"))
        completed_mean = _as_float(cl.get("completed_block_mean_fast_residual_ns"))
        completed_tau = _as_float(cl.get("completed_block_tau"))
        completed_ppb = _as_float(cl.get("completed_block_ppb"))

        if block_n and block_n > 0:
            expected_block_sum = (block_ticks * NS_PER_TICK - block_n * NS_PER_SECOND) if block_ticks is not None else None
            if expected_block_sum is not None and block_sum is not None and block_sum != expected_block_sum:
                flags.append("BSUM")
                stats[f"{prefix}_bad_block_sum"] += 1
            if block_sum is not None and block_mean is not None and not _near(block_mean, block_sum / float(block_n)):
                flags.append("BMEAN")
                stats[f"{prefix}_bad_block_mean"] += 1
            expected_block_ppb = _ppb_from_ticks(block_ticks, block_n)
            if block_ppb is not None and expected_block_ppb is not None and not _near(block_ppb, expected_block_ppb, 0.01):
                flags.append("BPPB")
                stats[f"{prefix}_bad_block_ppb"] += 1
            expected_block_tau = _tau_from_ticks(block_ticks, block_n)
            if block_tau is not None and expected_block_tau is not None and not _near(block_tau, expected_block_tau, 1e-12):
                flags.append("BTAU")
                stats[f"{prefix}_bad_block_tau"] += 1

        if completed_n and completed_n > 0:
            expected_completed_sum = (completed_ticks * NS_PER_TICK - completed_n * NS_PER_SECOND) if completed_ticks is not None else None
            if expected_completed_sum is not None and completed_sum is not None and completed_sum != expected_completed_sum:
                flags.append("CBSUM")
                stats[f"{prefix}_bad_completed_block_sum"] += 1
            if completed_sum is not None and completed_mean is not None and not _near(completed_mean, completed_sum / float(completed_n)):
                flags.append("CBMEAN")
                stats[f"{prefix}_bad_completed_block_mean"] += 1
            expected_completed_ppb = _ppb_from_ticks(completed_ticks, completed_n)
            if completed_ppb is not None and expected_completed_ppb is not None and not _near(completed_ppb, expected_completed_ppb, 0.01):
                flags.append("CBPPB")
                stats[f"{prefix}_bad_completed_block_ppb"] += 1
            expected_completed_tau = _tau_from_ticks(completed_ticks, completed_n)
            if completed_tau is not None and expected_completed_tau is not None and not _near(completed_tau, expected_completed_tau, 1e-12):
                flags.append("CBTAU")
                stats[f"{prefix}_bad_completed_block_tau"] += 1

    row.update({
        "valid": valid,
        "seq_match": seq_match,
        "cap_valid": cap_valid,
        "pps_seq": pps_seq,
        "n": sample_count,
        "ticks": ticks64,
        "ns": ns,
        "iticks": interval_ticks,
        "ires": fast_residual_ns,
        "ppb": None,  # filled by collect_rows running mean
        "cand_g": candidate_minus_gnss,
        "cand_p": candidate_minus_public,
        "candidate_public_ns": candidate_public_ns,
        "bn": _as_int(cl.get("block_interval_count")),
        "bppb": _as_float(cl.get("block_ppb")),
        "cbppb": _as_float(cl.get("completed_block_ppb")),
        "flags": _fmt_flags(flags),
    })
    return row


def collect_rows(records: Iterable[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], Dict[str, int]]:
    stats: Dict[str, int] = {
        "records_seen": 0,
        "rows_collected": 0,
        "gaps": 0,
    }
    for prefix, _ in LANES:
        for name in (
            "missing_counterledger", "disabled", "invalid", "interval_invalid",
            "sequence_mismatch", "capture_invalid", "bad_ns", "bad_interval_ns",
            "bad_residual", "bad_candidate_gnss", "bad_candidate_public",
            "bad_delta_ticks", "bad_delta_ns", "bad_sample_count_step",
            "bad_pps_sequence_step", "bad_candidate_delta", "block_fields_missing",
            "bad_block_sum", "bad_block_mean", "bad_block_ppb", "bad_block_tau",
            "bad_completed_block_sum", "bad_completed_block_mean",
            "bad_completed_block_ppb", "bad_completed_block_tau",
        ):
            stats[f"{prefix}_{name}"] = 0

    rows: List[Dict[str, Any]] = []
    prev_row: Optional[Dict[str, Any]] = None
    prev_pps: Optional[int] = None
    residual_sums = {"o1": 0.0, "o2": 0.0}
    residual_counts = {"o1": 0, "o2": 0}

    for rec in records:
        stats["records_seen"] += 1
        root = _root(rec)
        frag = _frag(rec)
        pps = pps_count_from_schema(root, frag)
        if pps is None:
            continue

        if prev_pps is not None and pps != prev_pps + 1:
            stats["gaps"] += 1
            prev_row = None

        gnss_ns = gnss_ns_from_schema(root, frag)
        row: Dict[str, Any] = {"pps": pps, "gnss_ns": gnss_ns}

        for prefix, lane in LANES:
            lane_row = analyze_lane(prefix, lane, frag, gnss_ns, prev_row, stats)
            for key, value in lane_row.items():
                row[f"{prefix}_{key}"] = value

            residual = lane_row.get("ires")
            if residual is not None:
                residual_sums[prefix] += float(residual)
                residual_counts[prefix] += 1
                row[f"{prefix}_ppb"] = residual_sums[prefix] / float(residual_counts[prefix])
            else:
                row[f"{prefix}_ppb"] = None

        rows.append(row)
        stats["rows_collected"] += 1
        prev_row = row
        prev_pps = pps

    return rows, stats


# -----------------------------------------------------------------------------
# Output
# -----------------------------------------------------------------------------


def print_table(rows: List[Dict[str, Any]]) -> None:
    columns = [
        ("pps", lambda r: _fmt_int(r.get("pps"))),
        ("o1_n", lambda r: _fmt_int(r.get("o1_n"))),
        ("o1_seq", lambda r: _fmt_int(r.get("o1_pps_seq"))),
        ("o1_itk", lambda r: _fmt_int(r.get("o1_iticks"))),
        ("o1_res", lambda r: _fmt_int(r.get("o1_ires"), signed=True)),
        ("o1_ppb", lambda r: _fmt_float(r.get("o1_ppb"), signed=True)),
        ("o1_c-g", lambda r: _fmt_int(r.get("o1_cand_g"), signed=True)),
        ("o1_c-p", lambda r: _fmt_int(r.get("o1_cand_p"), signed=True)),
        ("o1_bn", lambda r: _fmt_int(r.get("o1_bn"))),
        ("o1_bppb", lambda r: _fmt_float(r.get("o1_bppb"), signed=True)),
        ("o1_cbppb", lambda r: _fmt_float(r.get("o1_cbppb"), signed=True)),
        ("o1_ok", lambda r: str(r.get("o1_flags", "---"))),
        ("o2_n", lambda r: _fmt_int(r.get("o2_n"))),
        ("o2_seq", lambda r: _fmt_int(r.get("o2_pps_seq"))),
        ("o2_itk", lambda r: _fmt_int(r.get("o2_iticks"))),
        ("o2_res", lambda r: _fmt_int(r.get("o2_ires"), signed=True)),
        ("o2_ppb", lambda r: _fmt_float(r.get("o2_ppb"), signed=True)),
        ("o2_c-g", lambda r: _fmt_int(r.get("o2_cand_g"), signed=True)),
        ("o2_c-p", lambda r: _fmt_int(r.get("o2_cand_p"), signed=True)),
        ("o2_bn", lambda r: _fmt_int(r.get("o2_bn"))),
        ("o2_bppb", lambda r: _fmt_float(r.get("o2_bppb"), signed=True)),
        ("o2_cbppb", lambda r: _fmt_float(r.get("o2_cbppb"), signed=True)),
        ("o2_ok", lambda r: str(r.get("o2_flags", "---"))),
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


def print_sanity_summary(stats: Dict[str, int]) -> None:
    print("Sanity")
    print("══════")
    for prefix in ("o1", "o2"):
        interesting = []
        for key in sorted(k for k in stats if k.startswith(prefix + "_")):
            value = stats[key]
            if value:
                interesting.append(f"{key[len(prefix)+1:]}={value:,}")
        if interesting:
            print(f"  {prefix}: " + "  ".join(interesting))
        else:
            print(f"  {prefix}: OK")


def analyze(campaign: str) -> None:
    records = fetch_timebase(campaign)
    rows, stats = collect_rows(records)

    print(f"ZPNet raw_counterledger — campaign={campaign}")
    print(f"records={stats['records_seen']:,}  rows={stats['rows_collected']:,}  gaps={stats['gaps']:,}")
    print()

    if not rows:
        print("No TIMEBASE rows found.")
        return

    print_sanity_summary(stats)
    print()
    print_table(rows)

    print()
    print("Notes")
    print("═════")
    print("  • *_itk is counterledger.interval_ticks; 10,000,000 ticks is nominal one second.")
    print("  • *_res is counterledger.fast_residual_ns; expected value is interval_ticks*100 - 1e9.")
    print("  • *_ppb is this report's running mean of *_res, so ns/sec is numerically ppb.")
    print("  • *_c-g and *_c-p are candidate_public_ns minus GNSS/public OCXO ns.")
    print("  • *_bn / *_bppb are the active CounterLedger block interval count and PPB, if firmware publishes them.")
    print("  • *_cbppb is the most recently completed block PPB, if firmware publishes it.")
    print("  • *_ok is OK when all row-level invariants pass; otherwise it lists compact failure codes.")


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: raw_counterledger <campaign_name>")
        raise SystemExit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()
