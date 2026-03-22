from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

from zpnet.shared.db import open_db


MAX_POINTS_DEFAULT = 100
BAR_WIDTH_DEFAULT = 60


@dataclass
class Sample:
    pps_count: int
    seconds_from_start: int
    gnss_ns: int
    ocxo1_ns: int
    ocxo2_ns: int
    ocxo1_tau: float
    ocxo2_tau: float
    ocxo1_ppb: float
    ocxo2_ppb: float
    gnss_time_utc: Optional[str]
    system_time_utc: Optional[str]


def fetch_timebase_rows(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []

    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)

        payload["_db_id"] = row["id"]
        payload["_db_ts"] = str(row["ts"])
        result.append(payload)

    return result


def compute_tau(clock_ns: int, gnss_ns: int) -> float:
    if gnss_ns <= 0:
        raise ValueError("gnss_ns must be > 0")
    return clock_ns / gnss_ns


def compute_cumulative_ppb(clock_ns: int, gnss_ns: int) -> float:
    if gnss_ns <= 0:
        raise ValueError("gnss_ns must be > 0")
    return ((clock_ns - gnss_ns) / gnss_ns) * 1e9


def rows_to_samples(rows: Sequence[Dict[str, Any]]) -> List[Sample]:
    if not rows:
        return []

    first_pps = int(rows[0]["pps_count"])
    samples: List[Sample] = []

    for row in rows:
        gnss_ns = int(row["teensy_gnss_ns"])
        ocxo1_ns = int(row["teensy_ocxo1_ns"])
        ocxo2_ns = int(row["teensy_ocxo2_ns"])
        pps_count = int(row["pps_count"])

        samples.append(
            Sample(
                pps_count=pps_count,
                seconds_from_start=pps_count - first_pps,
                gnss_ns=gnss_ns,
                ocxo1_ns=ocxo1_ns,
                ocxo2_ns=ocxo2_ns,
                ocxo1_tau=compute_tau(ocxo1_ns, gnss_ns),
                ocxo2_tau=compute_tau(ocxo2_ns, gnss_ns),
                ocxo1_ppb=compute_cumulative_ppb(ocxo1_ns, gnss_ns),
                ocxo2_ppb=compute_cumulative_ppb(ocxo2_ns, gnss_ns),
                gnss_time_utc=row.get("gnss_time_utc"),
                system_time_utc=row.get("system_time_utc"),
            )
        )

    return samples


def reduce_samples(samples: Sequence[Sample], max_points: int) -> List[Sample]:
    if max_points <= 0:
        raise ValueError("max_points must be > 0")

    if len(samples) <= max_points:
        return list(samples)

    bucket_size = math.ceil(len(samples) / max_points)
    reduced: List[Sample] = []

    for i in range(0, len(samples), bucket_size):
        bucket = samples[i:i + bucket_size]
        reduced.append(bucket[-1])

    return reduced


def fmt_duration(seconds: int) -> str:
    hrs = seconds // 3600
    mins = (seconds % 3600) // 60
    secs = seconds % 60
    return f"{hrs:02d}:{mins:02d}:{secs:02d}"


def slope_ppb_per_hour(a: Sample, b: Sample, attr: str) -> float:
    dt = b.seconds_from_start - a.seconds_from_start
    if dt <= 0:
        return 0.0

    dv = getattr(b, attr) - getattr(a, attr)
    return dv * 3600.0 / dt


def render_ascii_bar_plot(
    samples: Sequence[Sample],
    title: str,
    attr: str,
    width: int = BAR_WIDTH_DEFAULT,
) -> str:
    if not samples:
        return f"{title}\n(no data)"

    values = [abs(getattr(s, attr)) for s in samples]
    max_abs = max(values)

    if max_abs <= 0.0:
        max_abs = 1.0

    rule_width = width + 50
    lines: List[str] = []

    lines.append(title)
    lines.append("-" * rule_width)
    lines.append(
        f"{'idx':>4s}  {'time':>8s}  {'pps':>8s}  {'ppb':>12s}  |bar"
    )
    lines.append("-" * rule_width)

    for i, s in enumerate(samples):
        ppb = getattr(s, attr)
        bar_len = int(round((abs(ppb) / max_abs) * width))
        bar = "#" * bar_len

        lines.append(
            f"{i:4d}  "
            f"{fmt_duration(s.seconds_from_start):>8s}  "
            f"{s.pps_count:8d}  "
            f"{ppb:12.6f}  "
            f"|{bar}"
        )

    lines.append("-" * rule_width)
    lines.append(f"max |ppb| = {max_abs:.6f}")

    return "\n".join(lines)


def print_summary(samples: Sequence[Sample]) -> None:
    first = samples[0]
    last = samples[-1]

    print("=" * 100)
    print("DECAY ANALYZER")
    print("=" * 100)
    print(f"records:              {len(samples):,}")
    print(f"time span:            {fmt_duration(last.seconds_from_start)}")
    print(f"pps_count range:      {first.pps_count} -> {last.pps_count}")
    print(f"gnss_time_utc:        {first.gnss_time_utc} -> {last.gnss_time_utc}")
    print()
    print("final cumulative ppb:")
    print(f"  ocxo1:              {last.ocxo1_ppb:+.6f}")
    print(f"  ocxo2:              {last.ocxo2_ppb:+.6f}")
    print()
    print("net change since first row:")
    print(f"  ocxo1 delta ppb:    {last.ocxo1_ppb - first.ocxo1_ppb:+.6f}")
    print(f"  ocxo2 delta ppb:    {last.ocxo2_ppb - first.ocxo2_ppb:+.6f}")
    print()
    print("average slope over campaign:")
    print(f"  ocxo1 ppb/hour:     {slope_ppb_per_hour(first, last, 'ocxo1_ppb'):+.6f}")
    print(f"  ocxo2 ppb/hour:     {slope_ppb_per_hour(first, last, 'ocxo2_ppb'):+.6f}")
    print("=" * 100)


def print_ocxo_report(
    samples: Sequence[Sample],
    title: str,
    ns_attr: str,
    tau_attr: str,
    ppb_attr: str,
) -> None:
    print()
    print(title)
    print("-" * 120)
    print(
        f"{'idx':>4s}  {'time':>8s}  {'pps':>8s}  "
        f"{'ocxo_ns':>15s}  {'gnss_ns':>15s}  {'tau':>14s}  {'ppb':>12s}"
    )
    print("-" * 120)

    for i, s in enumerate(samples):
        ocxo_ns = getattr(s, ns_attr)
        tau = getattr(s, tau_attr)
        ppb = getattr(s, ppb_attr)

        print(
            f"{i:4d}  "
            f"{fmt_duration(s.seconds_from_start):>8s}  "
            f"{s.pps_count:8d}  "
            f"{ocxo_ns:15d}  "
            f"{s.gnss_ns:15d}  "
            f"{tau:14.12f}  "
            f"{ppb:12.6f}"
        )


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: decay_analyzer <campaign> [max_points]")
        sys.exit(1)

    campaign = sys.argv[1]
    max_points = int(sys.argv[2]) if len(sys.argv) >= 3 else MAX_POINTS_DEFAULT

    rows = fetch_timebase_rows(campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        sys.exit(1)

    full_samples = rows_to_samples(rows)
    reduced = reduce_samples(full_samples, max_points)

    print_summary(full_samples)

    print()
    print(render_ascii_bar_plot(reduced, "OCXO1 DECAY", "ocxo1_ppb"))
    print()
    print(render_ascii_bar_plot(reduced, "OCXO2 DECAY", "ocxo2_ppb"))

    print_ocxo_report(
        reduced,
        "OCXO1 REPORT",
        "ocxo1_ns",
        "ocxo1_tau",
        "ocxo1_ppb",
    )

    print_ocxo_report(
        reduced,
        "OCXO2 REPORT",
        "ocxo2_ns",
        "ocxo2_tau",
        "ocxo2_ppb",
    )


if __name__ == "__main__":
    main()