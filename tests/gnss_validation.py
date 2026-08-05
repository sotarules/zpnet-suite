"""ZPNet GNSS Validation Report — TEMPEST PPS integrity analysis.

GNSS campaign nanoseconds are defined by the campaign PPS identity, so they
cannot validate themselves.  This report instead asks whether the one-second
reference is coherent with durable, independently authored timing surfaces:

  • physical PPS raw DWT interval
  • OCXO1 canonical measured one-second duration
  • OCXO2 canonical measured one-second duration
  • physical-PPS versus VCLOCK raw-interval closure

The report reads completed TEMPEST campaign_detail rows.  It does not use the
always-on relational pps_count as campaign time; campaign identity comes from
payload.campaign.tempest.teensy_pps_vclock_count.

Usage:
    python -m zpnet.tests.gnss_validation <campaign_name> [limit]
    .zt gnss_validation GCA4
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


DWT_NOMINAL = 1_008_000_000
NS_NOMINAL = 1_000_000_000
PPS_VCLOCK_CLEAN_GATE_CYCLES = 16
CAMPAIGN_TYPE = "TEMPEST"
PPS_COUNT_SQL = """
NULLIF(
    payload #>> '{campaign,tempest,teensy_pps_vclock_count}',
    ''
)::bigint
"""


def _dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_int(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _as_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "on"}:
            return True
        if text in {"0", "false", "no", "off"}:
            return False
    return None


def _first(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _root(payload: Dict[str, Any]) -> Dict[str, Any]:
    if any(
        key in payload
        for key in ("schema", "monitor_fragment", "fragment", "campaign_row", "campaign")
    ):
        return payload
    inner = _dict(payload.get("payload"))
    return inner or payload


def _tempest_decoration(payload: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(payload)
    return _dict(_dict(root.get("campaign")).get("tempest"))


def _fragment(payload: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(payload)
    direct = _dict(root.get("fragment")) or _dict(root.get("campaign_row"))
    monitor = _dict(root.get("monitor_fragment"))
    embedded = _dict(monitor.get("campaign_row"))
    decoration = _tempest_decoration(payload)
    source = embedded or direct
    if source or decoration:
        merged = dict(source)
        merged.update(decoration)
        return merged
    return root


def _science_interval_ns(frag: Dict[str, Any], lane: str) -> Optional[int]:
    science = _dict(_dict(frag.get(lane)).get("science"))
    interval = _as_int(science.get("clock_interval_ns"))
    if interval is not None:
        return interval
    gnss_interval = _as_int(science.get("gnss_interval_ns"))
    residual = _as_int(science.get("fast_residual_ns"))
    if gnss_interval is not None and residual is not None:
        return gnss_interval - residual
    return None


def _normalize(payload: Dict[str, Any], db_pps: int, db_ts: str) -> Dict[str, Any]:
    root = _root(payload)
    frag = _fragment(payload)
    raw = _dict(frag.get("raw_cycles"))
    pps_raw = _dict(raw.get("pps"))
    vclock_raw = _dict(raw.get("vclock"))
    dwt = _dict(frag.get("dwt"))
    root_gnss = _dict(root.get("gnss"))

    payload_pps = _as_int(
        _first(
            frag.get("teensy_pps_vclock_count"),
            frag.get("pps_count"),
            root.get("teensy_pps_vclock_count"),
            root.get("pps_count"),
        )
    )
    if payload_pps is not None and payload_pps != db_pps:
        raise ValueError(
            f"campaign_detail campaign PPS mismatch db={db_pps} payload={payload_pps}"
        )

    pps_cycles = _as_int(pps_raw.get("observed_cycles"))
    vclock_cycles = _as_int(vclock_raw.get("observed_cycles"))

    return {
        "pps_count": db_pps,
        "pps_raw_cycles": pps_cycles,
        "pps_raw_valid": _as_bool(pps_raw.get("valid")),
        "vclock_raw_cycles": vclock_cycles,
        "ocxo1_interval_ns": _science_interval_ns(frag, "ocxo1"),
        "ocxo2_interval_ns": _science_interval_ns(frag, "ocxo2"),
        "dwt_selected_cycles": _as_int(dwt.get("cycles_between_pps_vclock")),
        "pps_minus_vclock_cycles": (
            pps_cycles - vclock_cycles
            if pps_cycles is not None and vclock_cycles is not None
            else None
        ),
        "gnss_time_utc": _first(
            root_gnss.get("gnss_time_utc"),
            root.get("published_at_utc"),
            db_ts,
        ),
    }


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch completed TEMPEST details in campaign-public PPS order."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            f"""
            SELECT id, ts, {PPS_COUNT_SQL} AS pps_count, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND payload #> '{{campaign,tempest}}' IS NOT NULL
              AND {PPS_COUNT_SQL} IS NOT NULL
            ORDER BY {PPS_COUNT_SQL} ASC, id ASC
            """,
            (CAMPAIGN_TYPE, campaign),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            result.append(_normalize(payload, int(row["pps_count"]), str(row["ts"])))
    return result


class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val: Optional[float] = None
        self.max_val: Optional[float] = None

    def update(self, x: float) -> None:
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (x - self.mean)
        self.min_val = x if self.min_val is None else min(self.min_val, x)
        self.max_val = x if self.max_val is None else max(self.max_val, x)

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def range(self) -> float:
        if self.min_val is None or self.max_val is None:
            return 0.0
        return self.max_val - self.min_val


def _pearson(a: List[float], b: List[float]) -> float:
    n = min(len(a), len(b))
    if n < 2:
        return 0.0
    a = a[:n]
    b = b[:n]
    ma = sum(a) / n
    mb = sum(b) / n
    covariance = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    va = sum((x - ma) ** 2 for x in a)
    vb = sum((y - mb) ** 2 for y in b)
    denominator = math.sqrt(va * vb)
    return covariance / denominator if denominator > 0 else 0.0


def analyze(campaign: str, limit: int = 0) -> None:
    all_rows = fetch_timebase(campaign)
    if not all_rows:
        print(f"No TEMPEST campaign_detail rows for campaign '{campaign}'")
        return

    rows = all_rows[:limit] if limit else all_rows
    print("=" * 72)
    print(f"GNSS VALIDATION REPORT: {campaign}")
    print("=" * 72)
    print(f"  Records: {len(all_rows):,} (analyzing {len(rows):,})")
    print()

    w_dwt = Welford()
    w_o1 = Welford()
    w_o2 = Welford()
    w_dwt_selected = Welford()
    w_pps_vclock = Welford()
    pps_vclock_dist: Counter[int] = Counter()

    prev_pps: Optional[int] = None
    gaps = 0
    doubled = 0
    invalid_pps = 0
    dwt_ppb_samples: List[float] = []
    o1_ppb_samples: List[float] = []
    o2_ppb_samples: List[float] = []

    for rec in rows:
        pps = _as_int(rec.get("pps_count"))
        if pps is None:
            continue
        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
        prev_pps = pps

        dwt_cycles = _as_int(rec.get("pps_raw_cycles"))
        if rec.get("pps_raw_valid") is False:
            invalid_pps += 1
        if dwt_cycles is not None and dwt_cycles > DWT_NOMINAL * 1.5:
            doubled += 1
            continue

        if dwt_cycles is not None and dwt_cycles > 0:
            w_dwt.update(float(dwt_cycles))
            dwt_ppb_samples.append((dwt_cycles - DWT_NOMINAL) / DWT_NOMINAL * 1e9)

        for key, stats, samples in (
            ("ocxo1_interval_ns", w_o1, o1_ppb_samples),
            ("ocxo2_interval_ns", w_o2, o2_ppb_samples),
        ):
            interval = _as_int(rec.get(key))
            if interval is not None and interval > 0:
                stats.update(float(interval))
                samples.append((NS_NOMINAL - interval) / NS_NOMINAL * 1e9)

        selected = _as_int(rec.get("dwt_selected_cycles"))
        if selected is not None and selected > 0:
            w_dwt_selected.update(float(selected))

        closure = _as_int(rec.get("pps_minus_vclock_cycles"))
        if closure is not None:
            w_pps_vclock.update(float(closure))
            pps_vclock_dist[closure] += 1

    print("─" * 72)
    print("1. PPS STREAM HEALTH")
    print("─" * 72)
    print(f"  Campaign rows:        {len(rows):,}")
    print(f"  Campaign PPS gaps:    {gaps:,}")
    print(f"  Doubled DWT intervals:{doubled:>8,d}")
    print(f"  Invalid PPS snapshots:{invalid_pps:>8,d}")
    if gaps == 0 and doubled == 0 and invalid_pps == 0:
        print("  Assessment:           CLEAN")
    else:
        print("  Assessment:           ANOMALIES PRESENT")
    print()

    print("─" * 72)
    print("2. DURABLE ONE-SECOND WITNESSES")
    print("─" * 72)
    witnesses = [
        ("Physical PPS in DWT cycles", w_dwt, DWT_NOMINAL, "cycles", False),
        ("OCXO1 measured duration", w_o1, NS_NOMINAL, "ns", True),
        ("OCXO2 measured duration", w_o2, NS_NOMINAL, "ns", True),
    ]
    for name, stats, nominal, unit, reciprocal_sign in witnesses:
        print(f"  {name}:")
        if stats.n < 2:
            print("    insufficient data")
            continue
        offset_ppb = (
            (nominal - stats.mean) / nominal * 1e9
            if reciprocal_sign
            else (stats.mean - nominal) / nominal * 1e9
        )
        stability_ppb = stats.stddev / nominal * 1e9
        print(f"    n={stats.n:,} mean={stats.mean:,.3f} sd={stats.stddev:.3f} {unit}")
        print(f"    min={stats.min_val:,.0f} max={stats.max_val:,.0f} range={stats.range:,.0f}")
        print(f"    signed frequency offset: {offset_ppb:+.6f} ppb")
        print(f"    per-second stability:    {stability_ppb:.6f} ppb (1σ)")
    print()

    if min(len(dwt_ppb_samples), len(o1_ppb_samples), len(o2_ppb_samples)) >= 10:
        print("  Cross-witness correlation (signed per-second PPB):")
        print(f"    DWT   vs OCXO1: {_pearson(dwt_ppb_samples, o1_ppb_samples):+.4f}")
        print(f"    DWT   vs OCXO2: {_pearson(dwt_ppb_samples, o2_ppb_samples):+.4f}")
        print(f"    OCXO1 vs OCXO2: {_pearson(o1_ppb_samples, o2_ppb_samples):+.4f}")
        print()

    print("─" * 72)
    print("3. SELECTED PPS/VCLOCK DWT INTERVAL STABILITY")
    print("─" * 72)
    if w_dwt_selected.n >= 2:
        stability = w_dwt_selected.stddev / w_dwt_selected.mean * 1e9
        print(
            f"  n={w_dwt_selected.n:,} mean={w_dwt_selected.mean:,.3f} "
            f"sd={w_dwt_selected.stddev:.3f} cycles"
        )
        print(
            f"  min={w_dwt_selected.min_val:,.0f} max={w_dwt_selected.max_val:,.0f} "
            f"range={w_dwt_selected.range:,.0f}"
        )
        print(f"  stability: {stability:.6f} ppb (1σ)")
    else:
        print("  Insufficient data")
    print()

    print("─" * 72)
    print("4. PHYSICAL PPS VERSUS VCLOCK CLOSURE")
    print("─" * 72)
    if w_pps_vclock.n >= 2:
        outliers = sum(
            count
            for value, count in pps_vclock_dist.items()
            if abs(value) > PPS_VCLOCK_CLEAN_GATE_CYCLES
        )
        print(
            f"  n={w_pps_vclock.n:,} mean={w_pps_vclock.mean:+.3f} "
            f"sd={w_pps_vclock.stddev:.3f} DWT cycles"
        )
        print(
            f"  min={w_pps_vclock.min_val:+.0f} max={w_pps_vclock.max_val:+.0f} "
            f"range={w_pps_vclock.range:.0f}"
        )
        print(f"  |PPS-VCLOCK| > {PPS_VCLOCK_CLEAN_GATE_CYCLES}: {outliers:,}")
        print("  Distribution:")
        for value in sorted(pps_vclock_dist):
            count = pps_vclock_dist[value]
            print(f"    {value:+6d}  {count:>6,d}x  {'#' * min(count, 60)}")
    else:
        outliers = 0
        print("  Insufficient data")
    print()

    print("─" * 72)
    print("5. GNSS UTC COHERENCE")
    print("─" * 72)
    utc_stalls = 0
    utc_checked = 0
    previous_utc: Optional[str] = None
    for rec in rows:
        current = rec.get("gnss_time_utc")
        if current and previous_utc:
            utc_checked += 1
            if current == previous_utc:
                utc_stalls += 1
        if current:
            previous_utc = str(current)
    if utc_checked:
        print(f"  Adjacent UTC labels checked: {utc_checked:,}")
        print(f"  Repeated/stalled labels:     {utc_stalls:,}")
    else:
        print("  No GNSS UTC data to check")
    print()

    issues: List[str] = []
    if gaps:
        issues.append(f"{gaps} campaign PPS gap(s)")
    if doubled:
        issues.append(f"{doubled} doubled DWT interval(s)")
    if invalid_pps:
        issues.append(f"{invalid_pps} invalid PPS snapshot(s)")
    if w_pps_vclock.n and outliers > w_pps_vclock.n * 0.01:
        issues.append(f"{outliers} PPS/VCLOCK closure outlier(s)")
    if utc_stalls:
        issues.append(f"{utc_stalls} GNSS UTC stall(s)")

    print("─" * 72)
    print("VERDICT")
    print("─" * 72)
    if not issues:
        print("  GNSS PPS INTEGRITY: VALIDATED")
        print("  Durable timing witnesses are coherent and the campaign timeline is continuous.")
    else:
        print(f"  GNSS PPS INTEGRITY: ANOMALIES ({len(issues)})")
        for issue in issues:
            print(f"    • {issue}")
    print("=" * 72)


def list_campaigns() -> None:
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                f"""
                SELECT campaign, count(*) AS cnt
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND campaign IS NOT NULL
                  AND payload #> '{{campaign,tempest}}' IS NOT NULL
                  AND {PPS_COUNT_SQL} IS NOT NULL
                GROUP BY campaign
                ORDER BY max(ts) DESC
                LIMIT 10
                """,
                (CAMPAIGN_TYPE,),
            )
            rows = cur.fetchall()
        if rows:
            print("Available TEMPEST campaigns:")
            for row in rows:
                print(f"  {row['campaign']:<20s} {int(row['cnt']):>8,d}")
    except Exception:
        pass


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: gnss_validation <campaign_name> [limit]")
        print()
        list_campaigns()
        raise SystemExit(1)
    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
