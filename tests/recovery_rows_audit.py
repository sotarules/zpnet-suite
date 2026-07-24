"""Inspect the Servo2 recovery boundary at PPS rows 6366 and 6367.

This report is intentionally narrow.  It distinguishes:

* the campaign-facing public clockface;
* recovery/degraded aliases;
* CounterLedger's candidate public clockface;
* CounterLedger's fresh local-epoch clockface;
* compact science totals and Welford carry-over.

The purpose is to decide whether the campaign analyzer is selecting the wrong
field or whether firmware emitted internally inconsistent TIMEBASE rows.

Usage:
    python -m zpnet.tests.recovery_rows_audit
    python -m zpnet.tests.recovery_rows_audit Servo2 6366 6367
"""

from __future__ import annotations

import json
import sys
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_SECOND = 1_008_000_000


def d(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def i(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def f(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed


def b(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
    return None


def path(obj: Dict[str, Any], dotted: str) -> Any:
    current: Any = obj
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = i(value)
        if parsed is not None:
            return parsed
    return None


def root(record: Dict[str, Any]) -> Dict[str, Any]:
    payload = d(record.get("payload"))
    return payload or record


def fragment(record: Dict[str, Any]) -> Dict[str, Any]:
    r = root(record)
    frag = d(r.get("fragment"))
    return frag or r


def pps_count(record: Dict[str, Any]) -> Optional[int]:
    r = root(record)
    frag = fragment(record)
    return first_int(
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("campaign_seconds"),
        r.get("teensy_pps_vclock_count"),
        r.get("pps_count"),
    )


def fetch_boundary(campaign: str, targets: Sequence[int]) -> List[Dict[str, Any]]:
    if not targets:
        raise ValueError("at least one target PPS count is required")

    low = min(targets)
    high = max(targets)
    pps_sql = """
        COALESCE(
            NULLIF(payload->>'teensy_pps_vclock_count', '')::bigint,
            NULLIF(payload->>'pps_count', '')::bigint,
            NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
            NULLIF(payload->'fragment'->>'pps_count', '')::bigint,
            NULLIF(payload->'fragment'->>'campaign_seconds', '')::bigint
        )
    """

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            f"""
            WITH previous AS (
                SELECT id, ts, payload, {pps_sql} AS pps
                FROM timebase
                WHERE campaign = %s
                  AND {pps_sql} < %s
                ORDER BY {pps_sql} DESC, id DESC
                LIMIT 1
            ),
            selected AS (
                SELECT id, ts, payload, {pps_sql} AS pps
                FROM timebase
                WHERE campaign = %s
                  AND {pps_sql} BETWEEN %s AND %s
            )
            SELECT * FROM previous
            UNION ALL
            SELECT * FROM selected
            ORDER BY pps, id
            """,
            (campaign, low, campaign, low, high),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if not isinstance(payload, dict):
            continue
        payload["_db_id"] = row["id"]
        payload["_db_ts"] = str(row["ts"])
        payload["_query_pps"] = i(row["pps"])
        result.append(payload)
    return result


def fmt_int(value: Optional[int]) -> str:
    return "---" if value is None else f"{value:,d}"


def fmt_float(value: Optional[float], digits: int = 9) -> str:
    return "---" if value is None else f"{value:+.{digits}f}"


def bool_text(value: Optional[bool]) -> str:
    return "YES" if value is True else "NO" if value is False else "UNKNOWN"


def clock_fields(record: Dict[str, Any], lane: str) -> Dict[str, Any]:
    frag = fragment(record)
    lane_obj = d(frag.get(lane))
    gnss = d(frag.get("gnss"))
    counter = d(lane_obj.get("counterledger"))
    phase = d(counter.get("phaseledger"))
    science = d(lane_obj.get("science"))
    stats = d(path(frag, f"stats.{lane}"))
    welford = d(stats.get("welford"))

    return {
        # Public/schema-facing candidates.
        "lane_ns": i(lane_obj.get("ns")),
        "gnss_alias_ns": i(gnss.get(f"{lane}_ns")),
        "measured_gnss_ns": i(lane_obj.get("measured_gnss_ns")),
        "counter_candidate_public_ns": i(counter.get("candidate_public_ns")),
        "counter_candidate_minus_public_ns": i(counter.get("candidate_minus_public_ns")),
        "counter_candidate_minus_gnss_ns": i(counter.get("candidate_minus_gnss_ns")),
        # Fresh local-epoch custody.
        "counter_local_ns": i(counter.get("ns")),
        "counter_refined_ns": i(counter.get("refined_ns")),
        "counter_ticks64": i(counter.get("ticks64")),
        "counter_sample_count": i(counter.get("sample_count")),
        "phase_valid": b(phase.get("valid")),
        "phase_refined_interval_ns": i(counter.get("refined_interval_ns")),
        # One-second and campaign science.
        "pps_interval_ns": i(path(lane_obj, "pps_residual.clock_interval_ns")),
        "pps_gnss_interval_ns": i(path(lane_obj, "pps_residual.gnss_interval_ns")),
        "pps_fast_residual_ns": i(path(lane_obj, "pps_residual.fast_residual_ns")),
        "pps_valid": b(path(lane_obj, "pps_residual.valid")),
        "science_valid": b(science.get("valid")),
        "science_ready": b(lane_obj.get("science_ready")),
        "science_antecedents": b(science.get("antecedents_complete")),
        "science_total_valid": b(science.get("total_valid")),
        "science_total_fast_residual_ns": i(science.get("total_fast_residual_ns")),
        "science_total_ppb": f(science.get("total_ppb")),
        "science_total_tau": f(science.get("total_tau")),
        "science_total_sample_count": i(science.get("total_sample_count")),
        "science_public_ns_mode": science.get("public_ns_mode"),
        "science_frequency_source": science.get("frequency_source"),
        # Published panel/stat surfaces.
        "published_ppb": f(stats.get("ppb")),
        "published_tau": f(stats.get("tau")),
        "welford_n": i(welford.get("n")),
        "welford_mean": f(welford.get("mean")),
        "welford_stddev": f(welford.get("stddev")),
        # Recovery state.
        "clockface_valid": b(lane_obj.get("clockface_valid")),
        "recovery_degraded": b(lane_obj.get("recovery_degraded")),
    }


def dwt_fields(record: Dict[str, Any]) -> Dict[str, Any]:
    frag = fragment(record)
    dwt = d(frag.get("dwt"))
    stats = d(path(frag, "stats.dwt"))
    welford = d(stats.get("welford"))
    return {
        "cycles": first_int(dwt.get("cycle_count_total"), dwt.get("cycles"), dwt.get("value")),
        "interval": first_int(dwt.get("cycles_between_pps_vclock"),
                              frag.get("dwt_cycles_between_pps_vclock")),
        "residual": i(dwt.get("second_residual_cycles")),
        "published_ppb": f(stats.get("ppb")),
        "welford_n": i(welford.get("n")),
        "welford_mean": f(welford.get("mean")),
        "welford_stddev": f(welford.get("stddev")),
    }


def print_row(record: Dict[str, Any]) -> None:
    r = root(record)
    frag = fragment(record)
    count = pps_count(record)
    gnss_ns = first_int(path(frag, "gnss.ns"), frag.get("gnss_ns"))
    vclock_ns = first_int(path(frag, "vclock.ns"), path(frag, "gnss.ns"))
    print()
    print("=" * 96)
    print(f"PPS {count}  db_id={record.get('_db_id')}  ts={record.get('_db_ts')}")
    print("=" * 96)
    print(
        "Recovery: "
        f"reason={frag.get('recover_reattach_reason')}  "
        f"degraded={bool_text(b(frag.get('recover_degraded_active')))}  "
        f"science_hold={bool_text(b(frag.get('recover_degraded_science_hold')))}  "
        f"transition={bool_text(b(frag.get('recover_transition_active')))}  "
        f"generation={frag.get('recover_generation')}"
    )
    print(
        "Custody:  "
        f"timeline_valid={bool_text(b(frag.get('timeline_valid')))}  "
        f"candidate={frag.get('candidate_disposition')}  "
        f"ocxo_clockface_valid={bool_text(b(frag.get('ocxo_clockface_valid')))}  "
        f"ocxo_science_valid={bool_text(b(frag.get('ocxo_science_valid')))}"
    )
    print(f"GNSS public ns:   {fmt_int(gnss_ns)}")
    print(f"VCLOCK public ns: {fmt_int(vclock_ns)}")

    dwt = dwt_fields(record)
    print(
        "DWT: "
        f"cycles={fmt_int(dwt['cycles'])}  interval={fmt_int(dwt['interval'])}  "
        f"residual={fmt_int(dwt['residual'])}  ppb={fmt_float(dwt['published_ppb'], 6)}  "
        f"Welford(n={fmt_int(dwt['welford_n'])}, mean={fmt_float(dwt['welford_mean'], 6)}, "
        f"sd={fmt_float(dwt['welford_stddev'], 6)})"
    )

    for label, lane in (("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
        x = clock_fields(record, lane)
        print()
        print(f"[{label}] competing clockface fields")
        print(f"  fragment.{lane}.ns                         = {fmt_int(x['lane_ns'])}")
        print(f"  fragment.gnss.{lane}_ns                    = {fmt_int(x['gnss_alias_ns'])}")
        print(f"  fragment.{lane}.measured_gnss_ns           = {fmt_int(x['measured_gnss_ns'])}")
        print(f"  counterledger.candidate_public_ns          = {fmt_int(x['counter_candidate_public_ns'])}")
        print(f"  counterledger.candidate_minus_public_ns    = {fmt_int(x['counter_candidate_minus_public_ns'])}")
        print(f"  counterledger.candidate_minus_gnss_ns      = {fmt_int(x['counter_candidate_minus_gnss_ns'])}")
        print(f"  counterledger.ns             (LOCAL EPOCH) = {fmt_int(x['counter_local_ns'])}")
        print(f"  counterledger.refined_ns     (LOCAL EPOCH) = {fmt_int(x['counter_refined_ns'])}")
        print(
            f"  local custody: ticks64={fmt_int(x['counter_ticks64'])} "
            f"samples={fmt_int(x['counter_sample_count'])} phase_valid={bool_text(x['phase_valid'])}"
        )
        print(
            "  second science: "
            f"valid={bool_text(x['pps_valid'])} clock={fmt_int(x['pps_interval_ns'])} "
            f"gnss={fmt_int(x['pps_gnss_interval_ns'])} residual={fmt_int(x['pps_fast_residual_ns'])}"
        )
        print(
            "  total science:  "
            f"valid={bool_text(x['science_valid'])}/{bool_text(x['science_total_valid'])} "
            f"ready={bool_text(x['science_ready'])} antecedents={bool_text(x['science_antecedents'])} "
            f"total_res={fmt_int(x['science_total_fast_residual_ns'])} "
            f"ppb={fmt_float(x['science_total_ppb'], 9)} "
            f"tau={fmt_float(x['science_total_tau'], 12)} "
            f"N={fmt_int(x['science_total_sample_count'])}"
        )
        print(
            "  published stats: "
            f"ppb={fmt_float(x['published_ppb'], 9)} tau={fmt_float(x['published_tau'], 12)} "
            f"Welford(n={fmt_int(x['welford_n'])}, mean={fmt_float(x['welford_mean'], 9)}, "
            f"sd={fmt_float(x['welford_stddev'], 9)})"
        )
        print(
            "  modes: "
            f"public_ns_mode={x['science_public_ns_mode']} "
            f"frequency_source={x['science_frequency_source']} "
            f"clockface_valid={bool_text(x['clockface_valid'])} "
            f"recovery_degraded={bool_text(x['recovery_degraded'])}"
        )


def projection(prev: Dict[str, Any], current: Dict[str, Any], lane: str, selector: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    prev_frag = fragment(prev)
    curr_frag = fragment(current)
    prev_gnss = first_int(path(prev_frag, "gnss.ns"), prev_frag.get("gnss_ns"))
    curr_gnss = first_int(path(curr_frag, "gnss.ns"), curr_frag.get("gnss_ns"))
    if prev_gnss is None or curr_gnss is None or prev_gnss <= 0:
        return None, None, None

    prev_fields = clock_fields(prev, lane)
    curr_fields = clock_fields(current, lane)
    prev_value = i(prev_fields.get(selector))
    curr_value = i(curr_fields.get(selector))
    if prev_value is None or curr_value is None:
        return None, curr_value, None

    projected = (curr_gnss * prev_value) // prev_gnss
    return projected, curr_value, curr_value - projected


def print_boundary_comparison(prev: Dict[str, Any], rows: Sequence[Dict[str, Any]]) -> None:
    prev_count = pps_count(prev)
    print()
    print("#" * 96)
    print(f"BOUNDARY COMPARISON — previous persisted PPS {prev_count}")
    print("#" * 96)

    selectors = (
        ("lane_ns", "fragment.ocxoN.ns"),
        ("gnss_alias_ns", "fragment.gnss.ocxoN_ns"),
        ("measured_gnss_ns", "fragment.ocxoN.measured_gnss_ns"),
        ("counter_candidate_public_ns", "counterledger.candidate_public_ns"),
        ("counter_local_ns", "counterledger.ns (local epoch)"),
        ("counter_refined_ns", "counterledger.refined_ns (local epoch)"),
    )

    for current in rows:
        count = pps_count(current)
        print()
        print(f"Recovery target PPS {count}")
        for label, lane in (("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
            print(f"  [{label}]")
            for selector, title in selectors:
                projected, actual, error = projection(prev, current, lane, selector)
                print(
                    f"    {title:<44s} projected={fmt_int(projected):>18s} "
                    f"actual={fmt_int(actual):>18s} error={fmt_int(error):>18s}"
                )


def print_adjacent_comparison(first: Dict[str, Any], second: Dict[str, Any]) -> None:
    p1 = pps_count(first)
    p2 = pps_count(second)
    print()
    print("#" * 96)
    print(f"ADJACENT RECOVERY ROW CHECK — PPS {p1} -> {p2}")
    print("#" * 96)
    for label, lane in (("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
        a = clock_fields(first, lane)
        z = clock_fields(second, lane)
        print(f"[{label}]")
        for selector, title in (
            ("lane_ns", "fragment.ocxoN.ns"),
            ("gnss_alias_ns", "fragment.gnss.ocxoN_ns"),
            ("counter_candidate_public_ns", "counterledger.candidate_public_ns"),
            ("counter_local_ns", "counterledger.ns"),
            ("counter_refined_ns", "counterledger.refined_ns"),
        ):
            av = i(a.get(selector))
            zv = i(z.get(selector))
            delta = None if av is None or zv is None else zv - av
            print(f"  {title:<42s} {fmt_int(av):>18s} -> {fmt_int(zv):>18s}  delta={fmt_int(delta):>18s}")


def verdict(prev: Dict[str, Any], first: Dict[str, Any], second: Dict[str, Any]) -> None:
    print()
    print("=" * 96)
    print("FOCUSED VERDICT")
    print("=" * 96)

    findings: List[str] = []
    firmware_conflicts = 0
    analyzer_selector_traps = 0

    for label, lane in (("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
        x = clock_fields(first, lane)
        public_values = {
            "fragment.ocxoN.ns": x["lane_ns"],
            "fragment.gnss.ocxoN_ns": x["gnss_alias_ns"],
            "measured_gnss_ns": x["measured_gnss_ns"],
            "candidate_public_ns": x["counter_candidate_public_ns"],
        }
        non_null_public = {k: v for k, v in public_values.items() if v is not None}
        distinct_public = set(non_null_public.values())

        local_values = {v for v in (x["counter_local_ns"], x["counter_refined_ns"]) if v is not None}

        if len(distinct_public) > 1:
            firmware_conflicts += 1
            findings.append(
                f"{label}: firmware row exposes conflicting campaign-facing clockfaces: "
                + ", ".join(f"{k}={v:,d}" for k, v in non_null_public.items())
            )

        if x["lane_ns"] in local_values and x["gnss_alias_ns"] not in local_values:
            analyzer_selector_traps += 1
            findings.append(
                f"{label}: fragment.{lane}.ns equals the fresh local-epoch ledger while "
                f"fragment.gnss.{lane}_ns carries a different campaign-facing value; "
                "campaign_analyzer currently selects the local field first."
            )

        if x["published_ppb"] is not None and x["science_total_ppb"] is not None:
            delta = x["published_ppb"] - x["science_total_ppb"]
            if abs(delta) > 0.001:
                findings.append(
                    f"{label}: published stats ppb differs from science.total_ppb by {delta:+.9f}."
                )

    if firmware_conflicts:
        print("Classification: FIRMWARE/SCHEMA INCONSISTENCY PRESENT")
        print(
            "The analyzer may be selecting the wrong alias, but it is being presented with "
            "more than one incompatible campaign-facing OCXO clockface in the same row."
        )
    elif analyzer_selector_traps:
        print("Classification: ANALYZER FIELD-SELECTION ISSUE")
        print(
            "The row contains one coherent public clockface plus a separately labeled local "
            "epoch clockface, but the analyzer selects the local value."
        )
    else:
        print("Classification: NO OBVIOUS CLOCKFACE CONFLICT")
        print("The focused fields are mutually consistent; investigate analyzer recovery math next.")

    for item in findings:
        print(f"  - {item}")

    print()
    print("Interpretation rule:")
    print(
        "  counterledger.ns/refined_ns may lawfully restart after a flash because they are "
        "fresh local-epoch custody.  Any field advertised as fragment.ocxoN.ns, "
        "fragment.gnss.ocxoN_ns, measured_gnss_ns, or candidate_public_ns should agree on "
        "the recovered campaign-facing clockface."
    )


def main(argv: Sequence[str]) -> None:
    campaign = argv[1] if len(argv) >= 2 else "Servo2"
    targets = [int(v) for v in argv[2:]] if len(argv) >= 3 else [6366, 6367]
    if len(targets) != 2:
        raise SystemExit("Usage: recovery_rows_audit [CAMPAIGN] [FIRST_PPS SECOND_PPS]")

    rows = fetch_boundary(campaign, targets)
    by_count = {pps_count(row): row for row in rows}
    previous_candidates = [row for row in rows if (pps_count(row) or 0) < min(targets)]
    if not previous_candidates:
        raise SystemExit(f"No persisted row found before PPS {min(targets)} for campaign {campaign!r}")
    previous = previous_candidates[-1]

    missing = [target for target in targets if target not in by_count]
    if missing:
        raise SystemExit(f"Missing requested TIMEBASE row(s) for {campaign!r}: {missing}")

    selected = [by_count[target] for target in targets]

    print(f"Campaign: {campaign}")
    print(f"Requested rows: {targets[0]}, {targets[1]}")
    print(f"Previous persisted row: {pps_count(previous)}")
    print(
        "Purpose: distinguish recovered public clockfaces from fresh local CounterLedger/"
        "PhaseLedger epoch values."
    )

    print_row(previous)
    for row in selected:
        print_row(row)

    print_boundary_comparison(previous, selected)
    print_adjacent_comparison(selected[0], selected[1])
    verdict(previous, selected[0], selected[1])


if __name__ == "__main__":
    main(sys.argv)