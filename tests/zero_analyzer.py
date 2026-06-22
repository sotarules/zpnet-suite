"""
ZPNet Zero Analyzer — first TIMEBASE row startup validator

Focused START/SmartZero verifier for TIMEBASE_V3 campaigns.  Unlike the full
campaign_analyzer, this script intentionally looks only at the first persisted
TIMEBASE row and asks one narrow question:

    Did the campaign zero/start handoff produce sane first-row public ledgers?

Usage:
    python -m zpnet.tests.zero_analyzer <campaign_name>
    .zt zero_analyzer SmartZero11

Optional pasted-log mode:
    python -m zpnet.tests.zero_analyzer --file /path/to/timebase.log [campaign]
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_PPS = 1_008_000_000

# Startup target.  The ideal first OCXO public value is essentially one
# campaign second, with only sub-100 ns origin/rounding/phase residue.  We use a
# symmetric pass band because the current public-origin convention may publish a
# few ns either side of GNSS; a large positive hundreds-of-ms value is the real
# pathology this tool is designed to catch.
OCXO_ZERO_PASS_ABS_NS = 100
OCXO_ZERO_WARN_ABS_NS = 500
OCXO_ZERO_GROSS_ABS_NS = 1_000_000  # 1 ms; hundreds of ms will scream here.

DWT_FIRST_ROW_WARN_CYCLES = 20_000


class Check:
    __slots__ = ("level", "label", "detail")

    def __init__(self, level: str, label: str, detail: str):
        self.level = level
        self.label = label
        self.detail = detail


def _as_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _path_get(obj: Dict[str, Any], path: str, default: Any = None) -> Any:
    cur: Any = obj
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur.get(part)
    return default if cur is None else cur


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _as_bool(v: Any) -> Optional[bool]:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("true", "yes", "1"):
            return True
        if s in ("false", "no", "0"):
            return False
    return None


def _first_int(*values: Any) -> Optional[int]:
    for v in values:
        parsed = _as_int(v)
        if parsed is not None:
            return parsed
    return None


def _hline() -> None:
    print("-" * 78)


def _verdict(checks: List[Check]) -> str:
    if any(c.level == "FAIL" for c in checks):
        return "FAIL"
    if any(c.level == "WARN" for c in checks):
        return "WARN"
    return "PASS"


def _add(checks: List[Check], level: str, label: str, detail: str) -> None:
    checks.append(Check(level, label, detail))


def _check_eq(checks: List[Check], label: str, observed: Any, expected: Any) -> None:
    if observed == expected:
        _add(checks, "PASS", label, f"{observed!r}")
    else:
        _add(checks, "FAIL", label, f"observed={observed!r} expected={expected!r}")


def fetch_first_timebase(campaign: str) -> Optional[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE((payload->>'teensy_pps_vclock_count')::int,
                              (payload->>'pps_count')::int,
                              2147483647) ASC,
                     ts ASC
            LIMIT 1
            """,
            (campaign,),
        )
        row = cur.fetchone()

    if row is None:
        return None

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    if isinstance(payload, dict) and "payload" in payload and isinstance(payload["payload"], dict):
        # Defensive: tolerate records copied as {"payload": ...}.
        payload = payload["payload"]
    payload["_db_id"] = row["id"]
    payload["_db_ts"] = str(row["ts"])
    return payload


def _iter_timebase_payloads_from_file(path: str) -> Iterable[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("TIMEBASE "):
                line = line[len("TIMEBASE "):]
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            payload = obj.get("payload") if isinstance(obj, dict) else None
            if isinstance(payload, dict):
                yield payload


def fetch_first_timebase_from_file(path: str, campaign: Optional[str]) -> Optional[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for p in _iter_timebase_payloads_from_file(path):
        if campaign and p.get("campaign") != campaign:
            continue
        rows.append(p)
    if not rows:
        return None
    rows.sort(key=lambda p: (_first_int(p.get("teensy_pps_vclock_count"), p.get("pps_count")) or 2**31,
                             p.get("system_time_utc", "")))
    return rows[0]


def analyze_first_row(row: Dict[str, Any], *, source: str) -> int:
    fragment = _as_dict(row.get("fragment"))
    forensics = _as_dict(row.get("forensics"))

    checks: List[Check] = []

    campaign = row.get("campaign", "?")
    pps = _first_int(row.get("teensy_pps_vclock_count"), row.get("pps_count"))
    frag_pps = _first_int(fragment.get("teensy_pps_vclock_count"), fragment.get("pps_count"))
    for_pps = _first_int(forensics.get("teensy_pps_vclock_count"), forensics.get("pps_count"))

    gnss_ns = _first_int(_path_get(fragment, "gnss.ns"), fragment.get("gnss_ns"), row.get("gnss_ns"))
    vclock_ns = _first_int(_path_get(fragment, "vclock.ns"), gnss_ns)
    ocxo1_ns = _first_int(_path_get(fragment, "ocxo1.ns"), _path_get(fragment, "gnss.ocxo1_ns"))
    ocxo2_ns = _first_int(_path_get(fragment, "ocxo2.ns"), _path_get(fragment, "gnss.ocxo2_ns"))
    ocxo1_measured_ns = _first_int(_path_get(fragment, "ocxo1.measured_gnss_ns"))
    ocxo2_measured_ns = _first_int(_path_get(fragment, "ocxo2.measured_gnss_ns"))

    dwt_total = _first_int(_path_get(fragment, "dwt.cycle_count_total"), _path_get(fragment, "dwt.cycles"))
    dwt_interval = _first_int(_path_get(fragment, "dwt.cycles_between_pps_vclock"),
                              fragment.get("dwt_cycles_between_pps_vclock"),
                              forensics.get("dwt_cycles_between_pps_vclock"))

    print("=" * 78)
    print(f"ZERO ANALYSIS: {campaign}  (first TIMEBASE row)")
    print("=" * 78)
    print()
    print(f"  Source:             {source}")
    print(f"  Schema:             {row.get('schema', '?')} / {fragment.get('schema', '?')}")
    print(f"  Time:               {row.get('gnss_time_utc') or row.get('system_time_utc', '?')}")
    print(f"  pps_count:          {pps}")
    print(f"  campaign_elapsed:   {row.get('campaign_elapsed', '?')}")
    print()

    # Identity checks.
    _check_eq(checks, "top-level first pps_count", pps, 1)
    _check_eq(checks, "fragment pps_count matches", frag_pps, pps)
    if for_pps is not None:
        _check_eq(checks, "forensics pps_count matches", for_pps, pps)

    expected_gnss = (pps or 0) * NS_PER_SECOND
    _check_eq(checks, "GNSS public ns identity", gnss_ns, expected_gnss)
    _check_eq(checks, "VCLOCK public ns identity", vclock_ns, expected_gnss)

    if dwt_total is not None and dwt_interval is not None and pps == 1:
        dwt_error = int(dwt_total) - int(dwt_interval)
        level = "PASS" if abs(dwt_error) <= DWT_FIRST_ROW_WARN_CYCLES else "WARN"
        _add(checks, level, "DWT first-row total == interval",
             f"total={dwt_total:,} interval={dwt_interval:,} error={dwt_error:+,} cycles")
    else:
        _add(checks, "WARN", "DWT first-row total == interval", "missing DWT fields")

    if dwt_interval is not None:
        dwt_pps_error = int(dwt_interval) - DWT_EXPECTED_PER_PPS
        _add(checks, "PASS", "DWT interval present",
             f"{dwt_interval:,} cycles ({dwt_pps_error:+,} vs nominal)")
    else:
        _add(checks, "WARN", "DWT interval present", "missing")

    # OCXO startup checks.
    for label, ns_value, measured_value in (
        ("OCXO1", ocxo1_ns, ocxo1_measured_ns),
        ("OCXO2", ocxo2_ns, ocxo2_measured_ns),
    ):
        if ns_value is None or gnss_ns is None:
            _add(checks, "FAIL", f"{label} first-row ns present", "missing")
            continue
        delta = int(ns_value) - int(gnss_ns)
        if abs(delta) <= OCXO_ZERO_PASS_ABS_NS:
            level = "PASS"
            meaning = "zero-scale"
        elif abs(delta) <= OCXO_ZERO_WARN_ABS_NS:
            level = "WARN"
            meaning = "small but outside target"
        elif abs(delta) >= OCXO_ZERO_GROSS_ABS_NS:
            level = "FAIL"
            meaning = "gross startup offset"
        else:
            level = "FAIL"
            meaning = "outside startup target"
        sign_note = "ahead of GNSS" if delta > 0 else ("behind GNSS" if delta < 0 else "equal to GNSS")
        _add(checks, level, f"{label} public zero offset",
             f"ns={ns_value:,}  gnss={gnss_ns:,}  delta={delta:+,} ns ({sign_note}; {meaning})")
        if measured_value is not None:
            _check_eq(checks, f"{label} measured_gnss_ns matches ns", measured_value, ns_value)

        projected = _as_bool(_path_get(fragment, f"{label.lower()}.pps_projected_valid"))
        if projected is not None:
            _check_eq(checks, f"{label} pps_projected_valid", projected, True)
        source_id = _as_int(_path_get(fragment, f"{label.lower()}.ns_source_id"))
        if source_id is not None:
            _check_eq(checks, f"{label} ns_source_id", source_id, 2)

    # First row should not yet have a valid per-second OCXO residual.  There is
    # no prior public row inside the campaign.
    for label in ("ocxo1", "ocxo2"):
        rv = _as_bool(_path_get(fragment, f"{label}.pps_residual.valid"))
        if rv is not None:
            _check_eq(checks, f"{label.upper()} first residual invalid", rv, False)

    if forensics:
        _check_eq(checks, "forensics pair role", forensics.get("pair_role"), "forensics")
        _check_eq(checks, "forensics GNSS identity", _as_int(forensics.get("gnss_ns")), expected_gnss)
        _check_eq(checks, "PPS/VCLOCK edge available", _as_bool(forensics.get("pps_vclock_edge_available")), True)
        _check_eq(checks, "OCXO1 forensics projected", _as_bool(forensics.get("ocxo1_pps_projected")), True)
        _check_eq(checks, "OCXO2 forensics projected", _as_bool(forensics.get("ocxo2_pps_projected")), True)

    print("CLOCKS")
    _hline()
    print(f"  GNSS:     {gnss_ns:,}" if gnss_ns is not None else "  GNSS:     missing")
    print(f"  VCLOCK:   {vclock_ns:,}" if vclock_ns is not None else "  VCLOCK:   missing")
    if gnss_ns is not None and ocxo1_ns is not None:
        print(f"  OCXO1:    {ocxo1_ns:,}  delta={ocxo1_ns - gnss_ns:+,} ns")
    else:
        print("  OCXO1:    missing")
    if gnss_ns is not None and ocxo2_ns is not None:
        print(f"  OCXO2:    {ocxo2_ns:,}  delta={ocxo2_ns - gnss_ns:+,} ns")
    else:
        print("  OCXO2:    missing")
    if dwt_total is not None:
        print(f"  DWT:      {dwt_total:,} cycles")
    print()

    print("CHECKS")
    _hline()
    for c in checks:
        print(f"  {c.level:<4s} {c.label:<36s} {c.detail}")
    print()
    print("=" * 78)
    verdict = _verdict(checks)
    if verdict == "PASS":
        print("VERDICT: PASS — first TIMEBASE row zero/start values are sane")
    elif verdict == "WARN":
        print("VERDICT: WARN — first row is usable but has caution flags")
    else:
        print("VERDICT: FAIL — first row startup zero is not certified")
    print("=" * 78)
    return 0 if verdict in ("PASS", "WARN") else 2


def usage() -> None:
    print("Usage:")
    print("  zero_analyzer <campaign>")
    print("  zero_analyzer --file <timebase_log_path> [campaign]")


def main(argv: List[str]) -> int:
    if len(argv) < 2:
        usage()
        return 1

    if argv[1] == "--file":
        if len(argv) < 3:
            usage()
            return 1
        path = argv[2]
        campaign = argv[3] if len(argv) >= 4 else None
        row = fetch_first_timebase_from_file(path, campaign)
        if row is None:
            print(f"No TIMEBASE rows found in {path!r}" + (f" for campaign {campaign!r}" if campaign else ""))
            return 1
        return analyze_first_row(row, source=os.path.abspath(path))

    campaign = argv[1]
    row = fetch_first_timebase(campaign)
    if row is None:
        print(f"No TIMEBASE rows found for campaign {campaign!r}")
        return 1
    return analyze_first_row(row, source="database")


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
