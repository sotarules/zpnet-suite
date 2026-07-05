"""
ZPNet Raw PhaseLedger — CounterLedger low-order nanosecond sanity report.

Reads TIMEBASE rows for one campaign and validates the PhaseLedger suffix
published under:

    fragment.ocxo1.counterledger.phaseledger
    fragment.ocxo2.counterledger.phaseledger

This report is aimed at the PhaseLedger V1 failure mode where the low-order
phase is a circular 0..99 ns residue. A naive subtraction like:

    phase_now - phase_prev

turns a lawful wrap from 98 -> 2 into -96 ns, even though the physical
low-order phase only advanced by +4 ns. If the integer PPS CounterLedger tick
rail does not carry that extra 100 ns tick, the report exposes the mismatch and
also prints the virtual unwrapped residual that PhaseLedger should use.

Primary checks:

    phase_after_last_00_ns + phase_to_next_00_ns == 100 mod 100
    refined_interval_ns == interval_ns + raw_phase_delta       (current/raw law)
    refined_interval_ns == interval_ns + unwrapped_phase_delta (desired law)
    candidate_public_ns delta agrees with refined_interval_ns

Usage:
    python -m zpnet.tests.raw_phaseledger <campaign_name>
    python raw_phaseledger.py <campaign_name>

Optional file mode for pasted TIMEBASE JSONL/text captures:
    python raw_phaseledger.py --file /path/to/timebase.txt
"""

from __future__ import annotations

import json
import math
import re
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    from zpnet.shared.db import open_db
except Exception:  # file mode / offline syntax checks
    open_db = None  # type: ignore


NS_PER_SECOND = 1_000_000_000
NS_PER_TICK = 100
PHASE_MOD_NS = 100
WRAP_GATE_NS = 50
LANES = (("o1", "ocxo1"), ("o2", "ocxo2"))


# -----------------------------------------------------------------------------
# Data loading
# -----------------------------------------------------------------------------


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch TIMEBASE rows in campaign PPS order."""
    if open_db is None:
        raise RuntimeError("open_db unavailable; use --file for offline captures")

    with open_db(row_dict=True) as conn:  # type: ignore[misc]
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


def load_timebase_file(path: str) -> List[Dict[str, Any]]:
    """Load TIMEBASE records from a pasted text file.

    Accepts lines like:
        TIMEBASE { ... }
    or raw JSON objects. It ignores non-JSON report/table lines.
    """
    records: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()

    # Fast path: line-oriented TIMEBASE JSON.
    for line in text.splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("TIMEBASE "):
            s = s[len("TIMEBASE "):].strip()
        if not s.startswith("{"):
            continue
        try:
            obj = json.loads(s)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            records.append(obj)

    if records:
        return records

    # Fallback: scan for JSON objects after TIMEBASE markers in a wrapped file.
    for m in re.finditer(r"TIMEBASE\s+(\{.*?\})(?=\s*TIMEBASE\s+\{|\s*\Z)", text, re.S):
        try:
            obj = json.loads(m.group(1))
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            records.append(obj)
    return records


# -----------------------------------------------------------------------------
# Schema helpers
# -----------------------------------------------------------------------------


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


def pps_count_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def lane_public_ns(frag: Dict[str, Any], lane: str) -> Optional[int]:
    return _first_int(_nested_get(frag, lane, "ns"))


def counterledger_obj(frag: Dict[str, Any], lane: str) -> Dict[str, Any]:
    obj = _nested_get(frag, lane, "counterledger")
    return obj if isinstance(obj, dict) else {}


def phaseledger_obj(frag: Dict[str, Any], lane: str) -> Dict[str, Any]:
    obj = _nested_get(frag, lane, "counterledger", "phaseledger")
    return obj if isinstance(obj, dict) else {}


# -----------------------------------------------------------------------------
# Formatting
# -----------------------------------------------------------------------------


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


# -----------------------------------------------------------------------------
# Phase arithmetic
# -----------------------------------------------------------------------------


def unwrap_phase_delta(raw_delta: int) -> Tuple[int, int]:
    """Return (unwrapped_delta_ns, carry_delta_ticks).

    The phase residue is [0,100). If the raw delta crosses the branch cut,
    choose the nearest circular delta and return the virtual 100 ns carry needed
    to make the refined clock continuous.

        98 -> 2:  raw -96, carry +1, unwrapped +4
         2 -> 98: raw +96, carry -1, unwrapped -4
    """
    carry = 0
    delta = raw_delta
    if delta < -WRAP_GATE_NS:
        delta += PHASE_MOD_NS
        carry = +1
    elif delta > WRAP_GATE_NS:
        delta -= PHASE_MOD_NS
        carry = -1
    return delta, carry


def _mean(values: List[float]) -> Optional[float]:
    return sum(values) / len(values) if values else None


def _stddev(values: List[float]) -> Optional[float]:
    if len(values) < 2:
        return 0.0 if values else None
    m = sum(values) / len(values)
    return math.sqrt(sum((x - m) ** 2 for x in values) / (len(values) - 1))


# -----------------------------------------------------------------------------
# Analysis
# -----------------------------------------------------------------------------


def analyze_lane(
    prefix: str,
    lane: str,
    frag: Dict[str, Any],
    prev: Optional[Dict[str, Any]],
    stats: Dict[str, int],
    raw_residuals: Dict[str, List[float]],
    unwrap_residuals: Dict[str, List[float]],
) -> Dict[str, Any]:
    cl = counterledger_obj(frag, lane)
    ph = phaseledger_obj(frag, lane)
    flags: List[str] = []

    if not cl:
        stats[f"{prefix}_missing_counterledger"] += 1
        flags.append("MISS_CL")
    if not ph:
        stats[f"{prefix}_missing_phaseledger"] += 1
        flags.append("MISS_PH")

    phase_valid = _as_bool(ph.get("valid")) if ph else None
    phase_pending = _as_bool(ph.get("pending")) if ph else None
    near_boundary = _as_bool(ph.get("near_boundary")) if ph else None
    phase_seq = _as_int(ph.get("pps_sequence")) if ph else None
    lag_pps = _as_int(ph.get("lag_pps")) if ph else None
    phase_after = _as_int(ph.get("phase_after_last_00_ns")) if ph else None
    phase_to_next = _as_int(ph.get("phase_to_next_00_ns")) if ph else None
    resolve_count = _as_int(ph.get("resolve_count")) if ph else None

    interval_ns = _as_int(cl.get("interval_ns")) if cl else None
    interval_ticks = _as_int(cl.get("interval_ticks")) if cl else None
    refined_interval_ns = _as_int(cl.get("refined_interval_ns")) if cl else None
    refined_fast_residual_ns = _as_int(cl.get("refined_fast_residual_ns")) if cl else None
    refined_valid = _as_bool(cl.get("refined_valid")) if cl else None
    refined_interval_valid = _as_bool(cl.get("refined_interval_valid")) if cl else None
    candidate_public_ns = _as_int(cl.get("candidate_public_ns")) if cl else None
    candidate_minus_gnss_ns = _as_int(cl.get("candidate_minus_gnss_ns")) if cl else None
    block_sum_phase = _as_int(cl.get("block_fast_residual_sum_ns_with_phase")) if cl else None
    block_ppb_phase = _as_float(cl.get("block_ppb_with_phase")) if cl else None
    completed_ppb_phase = _as_float(cl.get("completed_block_ppb_with_phase")) if cl else None

    if phase_valid is not True:
        stats[f"{prefix}_phase_invalid"] += 1
        flags.append("PH_INV")
    if refined_valid is not True:
        stats[f"{prefix}_refined_invalid"] += 1
        flags.append("REF_INV")
    if refined_interval_valid is not True:
        stats[f"{prefix}_refined_interval_invalid"] += 1
        flags.append("RINT_INV")
    if phase_pending is True:
        stats[f"{prefix}_pending_true"] += 1
    if near_boundary is True:
        stats[f"{prefix}_near_boundary"] += 1
        flags.append("NEAR")
    if lag_pps is not None and lag_pps != 1:
        stats[f"{prefix}_lag_not_1"] += 1
        flags.append("LAG")

    closure_err: Optional[int] = None
    if phase_after is not None and phase_to_next is not None:
        closure_err = (phase_after + phase_to_next) % PHASE_MOD_NS
        if closure_err != 0:
            stats[f"{prefix}_closure_bad"] += 1
            flags.append("CLOSE")
    if phase_after is not None and not (0 <= phase_after < PHASE_MOD_NS):
        stats[f"{prefix}_phase_range_bad"] += 1
        flags.append("RANGE")

    raw_delta: Optional[int] = None
    unwrapped_delta: Optional[int] = None
    carry_delta: Optional[int] = None
    expected_raw_interval: Optional[int] = None
    expected_unwrapped_interval: Optional[int] = None
    raw_error: Optional[int] = None
    unwrap_error: Optional[int] = None
    candidate_delta: Optional[int] = None
    candidate_error: Optional[int] = None
    virtual_carry_ticks = 0

    if prev is not None and phase_after is not None:
        prev_phase = prev.get(f"{prefix}_phase")
        if isinstance(prev_phase, int):
            raw_delta = phase_after - prev_phase
            unwrapped_delta, carry_delta = unwrap_phase_delta(raw_delta)
            virtual_carry_ticks = carry_delta

            if interval_ns is not None:
                expected_raw_interval = interval_ns + raw_delta
                expected_unwrapped_interval = interval_ns + unwrapped_delta
            if refined_interval_ns is not None:
                if expected_raw_interval is not None:
                    raw_error = refined_interval_ns - expected_raw_interval
                    if raw_error != 0:
                        stats[f"{prefix}_refined_not_raw"] += 1
                if expected_unwrapped_interval is not None:
                    unwrap_error = refined_interval_ns - expected_unwrapped_interval
                    if unwrap_error != 0:
                        stats[f"{prefix}_refined_not_unwrapped"] += 1

            if raw_delta < -WRAP_GATE_NS or raw_delta > WRAP_GATE_NS:
                stats[f"{prefix}_wrap_events"] += 1
                flags.append("WRAP")
                if interval_ticks == 10_000_000:
                    stats[f"{prefix}_wrap_without_integer_tick_carry"] += 1
                    flags.append("NO_TICK_CARRY")

            if refined_fast_residual_ns is not None:
                raw_residuals[prefix].append(float(refined_fast_residual_ns))
                if abs(refined_fast_residual_ns) > WRAP_GATE_NS:
                    stats[f"{prefix}_large_firmware_residual"] += 1
                    flags.append("BIG_RES")
            if expected_unwrapped_interval is not None:
                unwrap_residual = expected_unwrapped_interval - NS_PER_SECOND
                unwrap_residuals[prefix].append(float(unwrap_residual))
                if abs(unwrap_residual) > WRAP_GATE_NS:
                    stats[f"{prefix}_large_unwrapped_residual"] += 1

    if prev is not None and candidate_public_ns is not None:
        prev_candidate = prev.get(f"{prefix}_candidate_public_ns")
        if isinstance(prev_candidate, int):
            candidate_delta = candidate_public_ns - prev_candidate
            if refined_interval_ns is not None:
                candidate_error = candidate_delta - refined_interval_ns
                if candidate_error != 0:
                    stats[f"{prefix}_candidate_delta_not_refined_interval"] += 1

    return {
        "phase_valid": phase_valid,
        "phase_pending": phase_pending,
        "near_boundary": near_boundary,
        "phase_seq": phase_seq,
        "lag": lag_pps,
        "resolve": resolve_count,
        "phase": phase_after,
        "to_next": phase_to_next,
        "closure": closure_err,
        "raw_delta": raw_delta,
        "unwrapped_delta": unwrapped_delta,
        "carry": carry_delta,
        "fw_res": refined_fast_residual_ns,
        "fw_int": refined_interval_ns,
        "exp_unwrap_int": expected_unwrapped_interval,
        "unwrap_err": unwrap_error,
        "raw_err": raw_error,
        "cand_g": candidate_minus_gnss_ns,
        "cand_delta": candidate_delta,
        "cand_err": candidate_error,
        "candidate_public_ns": candidate_public_ns,
        "block_sum_phase": block_sum_phase,
        "block_ppb_phase": block_ppb_phase,
        "completed_ppb_phase": completed_ppb_phase,
        "virtual_carry_ticks": virtual_carry_ticks,
        "flags": _fmt_flags(flags),
    }


def collect_rows(records: Iterable[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], Dict[str, int], Dict[str, List[float]], Dict[str, List[float]]]:
    stats: Dict[str, int] = {
        "records_seen": 0,
        "rows_collected": 0,
        "gaps": 0,
    }
    for prefix, _ in LANES:
        for name in (
            "missing_counterledger", "missing_phaseledger", "phase_invalid",
            "refined_invalid", "refined_interval_invalid", "pending_true",
            "near_boundary", "lag_not_1", "closure_bad", "phase_range_bad",
            "refined_not_raw", "refined_not_unwrapped", "wrap_events",
            "wrap_without_integer_tick_carry", "large_firmware_residual",
            "large_unwrapped_residual", "candidate_delta_not_refined_interval",
        ):
            stats[f"{prefix}_{name}"] = 0

    rows: List[Dict[str, Any]] = []
    raw_residuals: Dict[str, List[float]] = {"o1": [], "o2": []}
    unwrap_residuals: Dict[str, List[float]] = {"o1": [], "o2": []}
    prev_row: Optional[Dict[str, Any]] = None
    prev_pps: Optional[int] = None
    carry_total = {"o1": 0, "o2": 0}
    unwrapped_sum = {"o1": 0.0, "o2": 0.0}
    unwrapped_count = {"o1": 0, "o2": 0}

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
            carry_total = {"o1": 0, "o2": 0}

        row: Dict[str, Any] = {"pps": pps}
        for prefix, lane in LANES:
            lane_row = analyze_lane(prefix, lane, frag, prev_row, stats, raw_residuals, unwrap_residuals)
            carry_delta = lane_row.get("carry")
            if isinstance(carry_delta, int):
                carry_total[prefix] += carry_delta
            lane_row["carry_total"] = carry_total[prefix]

            unwrapped_delta = lane_row.get("unwrapped_delta")
            if isinstance(unwrapped_delta, int):
                unwrapped_sum[prefix] += float(unwrapped_delta)
                unwrapped_count[prefix] += 1
                lane_row["unwrap_ppb"] = unwrapped_sum[prefix] / float(unwrapped_count[prefix])
            else:
                lane_row["unwrap_ppb"] = None

            for key, value in lane_row.items():
                row[f"{prefix}_{key}"] = value

        rows.append(row)
        stats["rows_collected"] += 1
        prev_row = row
        prev_pps = pps

    return rows, stats, raw_residuals, unwrap_residuals


# -----------------------------------------------------------------------------
# Output
# -----------------------------------------------------------------------------


def print_sanity_summary(stats: Dict[str, int], raw_residuals: Dict[str, List[float]], unwrap_residuals: Dict[str, List[float]]) -> None:
    print("Sanity")
    print("══════")
    for prefix in ("o1", "o2"):
        interesting = []
        for key in sorted(k for k in stats if k.startswith(prefix + "_")):
            value = stats[key]
            if value:
                interesting.append(f"{key[len(prefix)+1:]}={value:,}")
        raw_sd = _stddev(raw_residuals[prefix])
        unwrap_sd = _stddev(unwrap_residuals[prefix])
        raw_mean = _mean(raw_residuals[prefix])
        unwrap_mean = _mean(unwrap_residuals[prefix])
        summary = (
            f"firmware_mean={_fmt_float(raw_mean, signed=True)} "
            f"firmware_sd={_fmt_float(raw_sd)} "
            f"unwrapped_mean={_fmt_float(unwrap_mean, signed=True)} "
            f"unwrapped_sd={_fmt_float(unwrap_sd)}"
        )
        if interesting:
            print(f"  {prefix}: {summary}  " + "  ".join(interesting))
        else:
            print(f"  {prefix}: {summary}  OK")


def print_table(rows: List[Dict[str, Any]]) -> None:
    columns = [
        ("pps", lambda r: _fmt_int(r.get("pps"))),
        ("o1_ph", lambda r: _fmt_int(r.get("o1_phase"))),
        ("o1_d", lambda r: _fmt_int(r.get("o1_raw_delta"), signed=True)),
        ("o1_du", lambda r: _fmt_int(r.get("o1_unwrapped_delta"), signed=True)),
        ("o1_carry", lambda r: _fmt_int(r.get("o1_carry_total"), signed=True)),
        ("o1_fw", lambda r: _fmt_int(r.get("o1_fw_res"), signed=True)),
        ("o1_uppb", lambda r: _fmt_float(r.get("o1_unwrap_ppb"), signed=True)),
        ("o1_c-g", lambda r: _fmt_int(r.get("o1_cand_g"), signed=True)),
        ("o1_err", lambda r: _fmt_int(r.get("o1_unwrap_err"), signed=True)),
        ("o1_ok", lambda r: str(r.get("o1_flags", "---"))),
        ("o2_ph", lambda r: _fmt_int(r.get("o2_phase"))),
        ("o2_d", lambda r: _fmt_int(r.get("o2_raw_delta"), signed=True)),
        ("o2_du", lambda r: _fmt_int(r.get("o2_unwrapped_delta"), signed=True)),
        ("o2_carry", lambda r: _fmt_int(r.get("o2_carry_total"), signed=True)),
        ("o2_fw", lambda r: _fmt_int(r.get("o2_fw_res"), signed=True)),
        ("o2_uppb", lambda r: _fmt_float(r.get("o2_unwrap_ppb"), signed=True)),
        ("o2_c-g", lambda r: _fmt_int(r.get("o2_cand_g"), signed=True)),
        ("o2_err", lambda r: _fmt_int(r.get("o2_unwrap_err"), signed=True)),
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


def analyze_records(label: str, records: List[Dict[str, Any]]) -> None:
    rows, stats, raw_residuals, unwrap_residuals = collect_rows(records)

    print(f"ZPNet raw_phaseledger — {label}")
    print(f"records={stats['records_seen']:,}  rows={stats['rows_collected']:,}  gaps={stats['gaps']:,}")
    print()

    if not rows:
        print("No TIMEBASE rows found.")
        return

    print_sanity_summary(stats, raw_residuals, unwrap_residuals)
    print()
    print_table(rows)
    print()
    print("Notes")
    print("═════")
    print("  • *_ph is phase_after_last_00_ns, the low-order 0..99 ns residue.")
    print("  • *_d is the raw phase delta; *_du is the circular/unwrapped phase delta.")
    print("  • *_carry is the cumulative virtual 100 ns tick carry implied by phase wraps.")
    print("  • *_fw is firmware refined_fast_residual_ns.")
    print("  • *_uppb is this report's running mean of unwrapped phase residuals; ns/sec == ppb.")
    print("  • *_err is firmware refined_interval_ns minus the unwrapped expected interval.")
    print("  • WRAP with NO_TICK_CARRY means the phase crossed 00 but interval_ticks stayed 10,000,000.")
    print("  • A high firmware SD and low unwrapped SD indicates branch-cut spikes, not oscillator behavior.")


def main() -> None:
    if len(sys.argv) == 3 and sys.argv[1] == "--file":
        path = sys.argv[2]
        analyze_records(f"file={path}", load_timebase_file(path))
        return
    if len(sys.argv) != 2:
        print("Usage: raw_phaseledger <campaign_name>")
        print("       raw_phaseledger --file /path/to/timebase.txt")
        raise SystemExit(1)
    campaign = sys.argv[1]
    analyze_records(f"campaign={campaign}", fetch_timebase(campaign))


if __name__ == "__main__":
    main()
