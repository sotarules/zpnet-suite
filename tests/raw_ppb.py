"""
ZPNet Raw PPB — canonical running OCXO PPB.

Reads TEMPEST-decorated campaign_detail rows and prints one line per campaign PPS row
for the two OCXO lanes. Every row shows the canonical one-second fast residual
and its running cumulative mean in ns/s, numerically equivalent to PPB.

Usage:
    python -m zpnet.tests.raw_ppb <campaign_name>
    python raw_ppb.py <campaign_name>

Columns:
    pps       Campaign PPS/VCLOCK row identity.
    o1_res    OCXO1 canonical one-second fast residual, ns.
    o1_ppb    OCXO1 running cumulative mean residual, ppb.
    o2_res    OCXO2 canonical one-second fast residual, ns.
    o2_ppb    OCXO2 running cumulative mean residual, ppb.
    pps_err   GF-8802 pps_timing_error_ns, retained as receiver telemetry.
    g_acc     GF-8802 estimated_accuracy_ns.
    g_frq     GF-8802 output-frequency-error estimate.
    g_clk     GF-8802 internal crystal drift.
    g_raw     Pi-owned GNSS_RAW drift.
    fl        "g" when the campaign PPS identity contains a gap, "." otherwise.

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
CAMPAIGN_TYPE = "TEMPEST"
PPS_COUNT_SQL = """
NULLIF(
    payload #>> '{campaign,public_count}',
    ''
)::bigint
"""

# -----------------------------------------------------------------------------
# Database and schema helpers
# -----------------------------------------------------------------------------


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch TEMPEST campaign_detail payloads in campaign-public PPS order."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            f"""
            SELECT id, {PPS_COUNT_SQL} AS pps_count, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND payload #>> '{{campaign,schema}}' = 'TEMPEST_FRAGMENT_V1'
              AND {PPS_COUNT_SQL} IS NOT NULL
            ORDER BY {PPS_COUNT_SQL} ASC, id ASC
            """,
            (CAMPAIGN_TYPE, campaign),
        )
        rows = cur.fetchall()

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if not isinstance(payload, dict):
            continue

        root = _root(payload)
        campaign = _campaign(payload)
        payload_count = pps_count_from_schema(campaign)
        db_count = int(row["pps_count"])
        if payload_count is not None and payload_count != db_count:
            raise ValueError(
                "campaign_detail relational/payload campaign PPS mismatch: "
                f"id={row['id']} db={db_count} payload={payload_count}"
            )
        out.append(payload)
    return out


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    """Return one persisted CLOCKS_V4 state detail."""
    if not isinstance(rec, dict):
        return {}
    if rec.get("schema") == "CLOCKS_V4":
        return rec
    inner = rec.get("payload")
    return inner if isinstance(inner, dict) and inner.get("schema") == "CLOCKS_V4" else rec


def _campaign(rec: Dict[str, Any]) -> Dict[str, Any]:
    campaign = _root(rec).get("campaign")
    return campaign if isinstance(campaign, dict) and campaign.get("schema") == "TEMPEST_FRAGMENT_V1" else {}


def _clocks(rec: Dict[str, Any]) -> Dict[str, Any]:
    clocks = _root(rec).get("clocks")
    return clocks if isinstance(clocks, dict) else {}


def _adjudication(rec: Dict[str, Any]) -> Dict[str, Any]:
    adjudication = _campaign(rec).get("adjudication")
    return adjudication if isinstance(adjudication, dict) else {}


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
        text = "---"
    else:
        text = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{text:>{width}s}" if width else text


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 3, signed: bool = False) -> str:
    if v is None:
        text = "---"
    else:
        text = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{text:>{width}s}" if width else text


def _round_nearest_int(value: float) -> int:
    """Round like firmware integer-ns rendering, away from zero at half."""
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def pps_count_from_schema(campaign: Dict[str, Any]) -> Optional[int]:
    """Return TEMPEST campaign-relative public identity."""
    return _as_int(campaign.get("public_count"))


def selected_reference_cycles(root: Dict[str, Any]) -> Optional[int]:
    """Return canonical same-second VCLOCK raw DWT interval."""
    return _as_int(_nested_get(_clocks(root), "raw_cycles", "vclock", "observed_cycles"))


def science_from_schema(campaign: Dict[str, Any], clock: str) -> Dict[str, Any]:
    key = LANE_KEYS.get(clock)
    if not key:
        return {}
    obj = _nested_get(campaign, key, "science")
    return obj if isinstance(obj, dict) else {}


def gnss_discipline_fields(root: Dict[str, Any]) -> Dict[str, Optional[float]]:
    """GNSS receiver telemetry plus campaign-relative GNSS_RAW adjudication."""
    gnss = root.get("gnss") if isinstance(root.get("gnss"), dict) else {}
    discipline = gnss.get("discipline") if isinstance(gnss.get("discipline"), dict) else {}
    pps = gnss.get("pps") if isinstance(gnss.get("pps"), dict) else {}
    clock = gnss.get("clock") if isinstance(gnss.get("clock"), dict) else {}
    adjudication = _adjudication(root)
    extra = adjudication.get("extra_clocks") if isinstance(adjudication.get("extra_clocks"), dict) else {}
    live_gnss_raw = _nested_get(_clocks(root), "gnss_raw")
    live_gnss_raw = live_gnss_raw if isinstance(live_gnss_raw, dict) else {}

    return {
        "pps_err": _first_float(
            gnss.get("pps_timing_error_ns"), discipline.get("pps_timing_error_ns")
        ),
        "g_acc": _first_float(
            gnss.get("estimated_accuracy_ns"), pps.get("estimated_accuracy_ns")
        ),
        "g_frq": _first_float(
            gnss.get("freq_error_ppb"), discipline.get("freq_error_ppb")
        ),
        "g_clk": _first_float(
            gnss.get("clock_drift_ppb"), clock.get("drift_ppb")
        ),
        "g_raw": _first_float(
            extra.get("gnss_raw_drift_ppb"), live_gnss_raw.get("drift_ppb")
        ),
    }


def fast_residual_ns(reference_cycles: Optional[int], clock_cycles: Optional[int]) -> Optional[int]:
    """Positive-fast ns residual from cycle interval comparison."""
    if reference_cycles is None or clock_cycles is None or reference_cycles <= 0:
        return None
    exact = (float(reference_cycles) - float(clock_cycles)) * float(NS_PER_SECOND) / float(reference_cycles)
    return _round_nearest_int(exact)


def ocxo_residual(campaign: Dict[str, Any], clock: str, fallback_ref_cycles: Optional[int]) -> Optional[int]:
    """Return canonical firmware fast residual; raw cycle math is fallback only."""
    sci = science_from_schema(campaign, clock)
    residual = _first_int(
        sci.get("delta_raw_fast_residual_ns"),
        sci.get("fast_residual_ns"),
    )
    if residual is not None:
        return residual

    cycles = _first_int(sci.get("delta_raw_clock_interval_cycles"))
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
    }

    running = {"OCXO1": RunningMean(), "OCXO2": RunningMean()}

    prev_pps: Optional[int] = None

    for rec in records:
        stats["records_seen"] += 1
        root = _root(rec)
        campaign = _campaign(rec)
        pps = pps_count_from_schema(campaign)
        if pps is None:
            continue

        gap = prev_pps is not None and pps != prev_pps + 1
        if gap:
            stats["gaps"] += 1

        ref = selected_reference_cycles(root)
        o1_res = ocxo_residual(campaign, "OCXO1", ref)
        o2_res = ocxo_residual(campaign, "OCXO2", ref)
        g = gnss_discipline_fields(root)

        row: Dict[str, Any] = {
            "pps": pps,
            "gap": gap,
            **g,
        }

        for clock, res in (("OCXO1", o1_res), ("OCXO2", o2_res)):
            prefix = "o1" if clock == "OCXO1" else "o2"
            row[f"{prefix}_res"] = res
            if res is not None:
                running[clock].add(res)
            row[f"{prefix}_ppb"] = running[clock].mean

        rows.append(row)
        stats["rows_collected"] += 1
        prev_pps = pps

    stats["running"] = running
    return rows, stats


# -----------------------------------------------------------------------------
# Output
# -----------------------------------------------------------------------------


def _row_flags(row: Dict[str, Any]) -> str:
    return "g" if row.get("gap") else "."


def print_table(rows: List[Dict[str, Any]]) -> None:
    columns = [
        ("pps", lambda r: _fmt_int(r.get("pps"))),
        ("o1_res", lambda r: _fmt_int(r.get("o1_res"), signed=True)),
        ("o1_ppb", lambda r: _fmt_float(r.get("o1_ppb"), signed=True)),
        ("o2_res", lambda r: _fmt_int(r.get("o2_res"), signed=True)),
        ("o2_ppb", lambda r: _fmt_float(r.get("o2_ppb"), signed=True)),
        ("pps_err", lambda r: _fmt_float(r.get("pps_err"), decimals=1, signed=True)),
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
        print("No TEMPEST campaign_detail rows found.")
        return

    print_table(rows)

    running = stats["running"]

    print()
    print("Final PPB")
    print("═════════")
    for clock in ("OCXO1", "OCXO2"):
        print(f"  {clock}: {_fmt_float(running[clock].mean, 0, 3, signed=True)}")

    print()
    print("Notes")
    print("═════")
    print("  • OCXO residual and PPB values come from the canonical Teensy-authored")
    print("    science surface, with raw cycle evidence used only as a schema fallback.")
    print("  • pps_err (gnss.pps_timing_error_ns) remains visible as receiver telemetry;")
    print("    it is not applied to OCXO residuals or accumulated PPB.")
    print("  • g_acc (estimated_accuracy_ns) is the receiver's time-accuracy estimate.")
    print("  • g_frq (freq_error_ppb) is the receiver's coarse output-frequency-error")
    print("    estimate.")
    print("  • g_clk (clock_drift_ppb) and g_raw (campaign.adjudication.extra_clocks, with live GNSS_RAW fallback) are")
    print("    receiver-crystal drift measures — thermal proxies, useful when deciding")
    print("    whether slow common-mode wander is GNSS or enclosure temperature.")
    print("  • Positive residual means the OCXO is running fast (project convention).")


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: raw_ppb <campaign_name>")
        raise SystemExit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()