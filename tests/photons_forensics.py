"""PHOTONS forensic correlation reconnaissance.

Search every independent numeric fact carried by PHOTONS_V1 for relationships
with the canonical one-second race mean:

    zt photons_forensics Pelican1
    zt photons_forensics Pelican1 --bin-seconds 60 --max-lag-seconds 1800
    zt photons_forensics Pelican1 --output-dir /tmp/Pelican1-forensics
    zt photons_forensics Pelican1 --input /tmp/Pelican1.jsonl

The target is photons.race.flight_ns.mean.  Every populated one-second fragment
has equal weight regardless of accepted race count.  Cumulative/campaign means,
raw-cycle centers, the lower envelope, and other LAP-derived surfaces are never
used as predictors.  Monotonic counters are converted to per-second rates rather
than correlated in cumulative form.  Database access is read-only and restricted
to the selected LANTERN campaign.

The report screens each usable predictor several ways: raw Pearson and Spearman
association, predictor-versus-time correlation, linear-time-detrended association,
within-epoch detrended association, adjacent-fragment changes, block means, lag
scans on identical support, and a frozen later-data holdout compared with both a
training mean and elapsed-time-only drift.  The output directory contains an HTML
index, one SVG diagnostic sheet per analyzed predictor, summary CSV/JSON, omitted
field inventory, and a temperature-plateau report.

This is hypothesis discovery, not automatic calibration.  Adjacent observations
are autocorrelated, many related telemetry fields are screened, and no IID p-values
are reported.  A high rank identifies an experiment to perform; it does not prove
causation.  DWT frequency is intentionally retained but flagged because it is also
an input to the cycles-to-nanoseconds conversion.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import math
import re
import statistics as st
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


SCHEMA = "PHOTONS_V1"
TARGET_PATH = "photons.race.flight_ns.mean"
TEMPERATURE_PATH = "environment.temperature_c"
ELAPSED_PATH = "derived.elapsed_time_s"
ACCEPTED_PATH = "derived.accepted_races_this_fragment"
MIN_FIT = 3
MIN_HOLDOUT = 12
EPSILON = 1e-24

# These surfaces contain the target itself, alternate representations of it, or
# statistics mathematically computed from the same accepted-lap population.
EXCLUDED_PREFIXES = (
    "campaign.stats.",
    "photons.envelope.",
    "photons.raw_cycles.",
    "photons.stats.",
    "photons.science.accepted.projected_lap_ns.",
    "photons.science.accepted.raw_cycles.",
    "photons.science.excluded.projected_lap_ns.",
    "photons.science.excluded.raw_cycles.",
)

CORE_ALLOWED = {
    "dwt_cycles_per_second",
    "input_count",
    "retained_count",
    "rejected_count_total",
    "rejected_early_count",
    "rejected_late_count",
    "minimum_count",
    "minimum_radius_cycles",
    "mad_multiplier",
    "minimum_retained_pct",
}

SCIENCE_ALLOWED_PREFIXES = (
    "photons.science.candidate_count",
    "photons.science.candidates_this_fragment",
    "photons.science.excluded.count",
    "photons.science.excluded.count_this_fragment",
    "photons.science.exclusion_reasons.",
    "photons.science.max_reject_streak",
    "photons.science.reject_streak",
    "photons.science.seed_pending_count",
)

IDENTIFIER_PATHS = {
    "campaign.campaign_id",
    "campaign.start_after_sequence",
    "photons.core.sequence",
    "photons.interrupt.source_pin",
    "photons.science.last_candidate_index",
    "photons.science.last_disposition_id",
    "photons.science.last_pps_sequence",
    "photons.projection.last_pps_sequence",
    "photons.science.last_reason_code",
}

# Known cumulative counters.  A raw counter is omitted and its run-local rate is
# exposed as rate.<path>.  Names containing this_fragment remain direct gauges.
COUNTER_BASENAMES = {
    "announcement_sequence",
    "attempt_count",
    "callback_count",
    "callback_missing_count",
    "edge_count_total",
    "exact_match_count",
    "fallback_match_count",
    "inactive_edge_count",
    "irq_count",
    "malformed_count",
    "pps_count",
    "publish_count",
    "public_count",
    "queue_overflow_count",
    "read_fail_count",
    "received_count",
    "recovery_count",
    "reject_count",
    "retry_count",
    "samples_used",
    "sequence",
    "success_count",
    "update_count",
    "uptime_s",
}

NETWORK_COUNTER_BASENAMES = {
    "bytes_recv",
    "bytes_sent",
    "packets_recv",
    "packets_sent",
}

STRUCTURAL_NOTES = {
    "photons.core.dwt_cycles_per_second": (
        "DWT scale input: flight_ns is converted from DWT cycles using this value; "
        "association can contain mathematical coupling as well as clock physics."
    ),
    "photons.projection.anchor_dwt_cycles_per_second": (
        "DWT anchor scale: related to the conversion timebase and not wholly "
        "independent of the nanosecond target."
    ),
    ELAPSED_PATH: "Baseline drift predictor; association is descriptive, not a mechanism.",
    "photons.race.cadence_hz": "Acquisition setting; changes may be confounded with epoch boundaries.",
    "photons.race.cadence_ns": "Acquisition setting; changes may be confounded with epoch boundaries.",
}


@dataclass(frozen=True)
class Sample:
    row_id: int
    time: float
    sequence: int
    reset: int
    pps: Optional[int]
    lap_ns: float
    races: int
    signature: str
    values: Dict[str, float]
    epoch: int = 0
    run: int = 0


@dataclass(frozen=True)
class VariableSpec:
    key: str
    source_path: str
    label: str
    unit: str
    mode: str
    note: str = ""


@dataclass(frozen=True)
class Point:
    run: int
    epoch: int
    index: int
    time: float
    x: float
    lap_ns: float


@dataclass(frozen=True)
class Fit:
    x0: float
    t0: float
    y0: float
    beta: float
    drift: float
    r2: float
    residual_sd_ns: float

    def predict(self, x: float, t: float = 0.0) -> float:
        return self.y0 + self.beta * (x - self.x0) + self.drift * (t - self.t0)


@dataclass(frozen=True)
class LagResult:
    support_n: int
    scan_seconds: int
    zero_r: Optional[float]
    zero_detrended_r: Optional[float]
    best_signed_lag_s: Optional[int]
    best_signed_r: Optional[float]
    best_signed_detrended_r: Optional[float]
    chosen_causal_lag_s: Optional[int]
    holdout_n: int
    holdout_mean_rmse_ps: Optional[float]
    holdout_time_rmse_ps: Optional[float]
    holdout_predictor_rmse_ps: Optional[float]
    holdout_combined_rmse_ps: Optional[float]
    holdout_predictor_improvement_vs_time_pct: Optional[float]
    holdout_combined_improvement_vs_time_pct: Optional[float]


@dataclass(frozen=True)
class Metric:
    key: str
    label: str
    source_path: str
    unit: str
    mode: str
    note: str
    n: int
    coverage_pct: float
    distinct: int
    minimum: float
    maximum: float
    span: float
    mean: float
    raw_r: Optional[float]
    spearman_r: Optional[float]
    predictor_time_r: Optional[float]
    detrended_r: Optional[float]
    within_epoch_detrended_r: Optional[float]
    delta_n: int
    delta_r: Optional[float]
    first_half_r: Optional[float]
    second_half_r: Optional[float]
    slope_ps_per_unit: Optional[float]
    span_effect_ps: Optional[float]
    residual_sd_ps: Optional[float]
    block_n: int
    block_r: Optional[float]
    block_detrended_r: Optional[float]
    block_delta_r: Optional[float]
    configuration_only: bool
    lag_support_n: int
    lag_scan_seconds: int
    zero_lag_r: Optional[float]
    zero_lag_detrended_r: Optional[float]
    best_signed_lag_s: Optional[int]
    best_signed_r: Optional[float]
    best_signed_detrended_r: Optional[float]
    chosen_causal_lag_s: Optional[int]
    holdout_n: int
    holdout_mean_rmse_ps: Optional[float]
    holdout_time_rmse_ps: Optional[float]
    holdout_predictor_rmse_ps: Optional[float]
    holdout_combined_rmse_ps: Optional[float]
    holdout_predictor_improvement_vs_time_pct: Optional[float]
    holdout_combined_improvement_vs_time_pct: Optional[float]
    plot_file: str


@dataclass(frozen=True)
class Plateau:
    start_utc: str
    end_utc: str
    duration_s: float
    observations: int
    temperature_min_c: float
    temperature_max_c: float
    temperature_span_c: float
    lap_start_ns: float
    lap_end_ns: float
    endpoint_change_ps: float
    fitted_change_ps: float
    fitted_slope_ps_per_minute: float
    lap_span_ps: float


def utc(value: Any) -> float:
    dt = value if isinstance(value, datetime) else datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if dt.tzinfo is None:
        raise ValueError(f"timestamp requires a timezone: {value!r}")
    return dt.timestamp()


def stamp(value: float) -> str:
    return datetime.fromtimestamp(value, timezone.utc).isoformat(timespec="seconds")


def number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"expected a finite number, got {value!r}")
    return float(value)


def integer(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"expected a nonnegative integer, got {value!r}")
    return value


def fmt(value: Optional[float], digits: int = 4) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def compact(value: Optional[float], digits: int = 5) -> str:
    if value is None:
        return "n/a"
    if value == 0:
        return "0"
    if abs(value) >= 1_000_000 or abs(value) < 0.001:
        return f"{value:.3e}"
    return f"{value:.{digits}f}".rstrip("0").rstrip(".")


def flatten_numeric(value: Any, prefix: str = "") -> Tuple[Dict[str, float], Dict[str, str]]:
    """Flatten finite numeric and boolean leaves while preserving rail labels."""
    out: Dict[str, float] = {}
    labels: Dict[str, str] = {}

    def visit(node: Any, path: str, inherited_label: str = "") -> None:
        if isinstance(node, dict):
            local_label = node.get("label") if isinstance(node.get("label"), str) else inherited_label
            for key, child in node.items():
                if key == "label":
                    continue
                child_path = f"{path}.{key}" if path else str(key)
                visit(child, child_path, local_label)
            return
        if isinstance(node, bool):
            out[path] = 1.0 if node else 0.0
        elif isinstance(node, (int, float)) and math.isfinite(node):
            out[path] = float(node)
        else:
            return
        labels[path] = f"{path} [{inherited_label}]" if inherited_label else path

    visit(value, prefix)
    return out, labels


def acquisition_signature(p: Dict[str, Any]) -> str:
    ph = p["photons"]
    race = ph["race"]
    core = ph.get("core") or {}
    return json.dumps({
        "race": {key: race.get(key) for key in (
            "schema", "cadence_ns", "cadence_hz", "pulse_ns", "capture_min_ns", "capture_max_ns",
            "launch_bracket_max_cycles", "flight_interpretation", "launch_timing_policy",
        )},
        "core": {key: core.get(key) for key in (
            "algorithm", "minimum_count", "minimum_radius_cycles", "mad_multiplier", "minimum_retained_pct",
        )},
    }, sort_keys=True)


def parse_sample(row_id: int, p: Dict[str, Any]) -> Tuple[Optional[Sample], str, Dict[str, str]]:
    """Required science is strict; optional telemetry is simply absent."""
    if p["schema"] != SCHEMA:
        raise ValueError(f"expected {SCHEMA}")
    ph = p["photons"]
    if ph["fragment_period_ns"] != 1_000_000_000:
        raise ValueError("forensic analysis requires one-second fragments")
    if ph["snapshot_ok"] is not True:
        return None, "snapshot_not_ok", {}
    race = ph["race"]
    if race["active"] is not True:
        return None, "autonomous_races_inactive", {}
    flight = race["flight_ns"]
    races = integer(flight["n"])
    if races == 0:
        return None, "no_accepted_races", {}
    core = ph.get("core") or {}
    if core and integer(core["sequence"]) != integer(p["sequence"]):
        raise ValueError("core diagnostic belongs to a different fragment")
    if core and integer(core["retained_count"]) != races:
        raise ValueError("core retained count does not match flight population")
    values, labels = flatten_numeric(p)
    projection = ph.get("projection") or {}
    pps = projection.get("anchor_pps_count")
    reset = integer((ph.get("stats") or {})["reset_count"])
    return Sample(
        row_id=row_id,
        time=utc(p["published_at_utc"]),
        sequence=integer(p["sequence"]),
        reset=reset,
        pps=None if pps is None else integer(pps),
        lap_ns=number(flight["mean"]),
        races=races,
        signature=acquisition_signature(p),
        values=values,
    ), "used", labels


def database_rows(args: argparse.Namespace) -> Iterable[Tuple[int, Dict[str, Any]]]:
    from zpnet.shared.db import open_db

    # Transfer broad telemetry but omit the very large cumulative/recovery and
    # alternate LAP surfaces that cannot be legitimate predictors.
    sql = """
        SELECT id, jsonb_build_object(
            'schema', payload -> 'schema',
            'sequence', payload -> 'sequence',
            'publish_count', payload -> 'publish_count',
            'pps_count', payload -> 'pps_count',
            'published_at_utc', payload -> 'published_at_utc',
            'campaign', payload -> 'campaign',
            'battery', payload -> 'battery',
            'environment', payload -> 'environment',
            'gnss', payload -> 'gnss',
            'location', payload -> 'location',
            'network', payload -> 'network',
            'pi', payload -> 'pi',
            'power', payload -> 'power',
            'sensors', payload -> 'sensors',
            'photons', jsonb_build_object(
                'snapshot_ok', payload #> '{photons,snapshot_ok}',
                'fragment_period_ns', payload #> '{photons,fragment_period_ns}',
                'dead_lap_count', payload #> '{photons,dead_lap_count}',
                'edge_count_total', payload #> '{photons,edge_count_total}',
                'edges_this_fragment', payload #> '{photons,edges_this_fragment}',
                'projected_laps_this_fragment', payload #> '{photons,projected_laps_this_fragment}',
                'train_count', payload #> '{photons,train_count}',
                'valid', payload #> '{photons,valid}',
                'stats', jsonb_build_object(
                    'reset_count', payload #> '{photons,stats,reset_count}'
                ),
                'core', payload #> '{photons,core}',
                'interrupt', payload #> '{photons,interrupt}',
                'projection', payload #> '{photons,projection}',
                'race', payload #> '{photons,race}',
                'science', payload #> '{photons,science}'
            )
        ) AS forensic_payload
        FROM campaign_detail
        WHERE campaign_type = %s AND campaign = %s AND payload ->> 'schema' = %s
        ORDER BY id ASC
    """
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="photons_forensics")
        cur.itersize = args.batch_size
        cur.execute(sql, ("LANTERN", args.campaign, SCHEMA))
        for row in cur:
            payload = row["forensic_payload"]
            yield int(row["id"]), json.loads(payload) if isinstance(payload, str) else payload


def file_rows(path: Path, campaign: str) -> Iterable[Tuple[int, Dict[str, Any]]]:
    text = path.read_text(encoding="utf-8-sig").strip()
    if text.endswith("&#x20;"):
        text = text[:-6].rstrip()
    if text.startswith("["):
        records = json.loads(text)
    else:
        if text.startswith("PHOTONS "):
            text = text[len("PHOTONS "):]
        try:
            records = [json.loads(text)]
        except json.JSONDecodeError:
            records = [json.loads(line.removeprefix("PHOTONS ")) for line in text.splitlines() if line.strip()]
    for index, record in enumerate(records, 1):
        payload = record.get("payload", record)
        campaign_block = payload.get("campaign") if isinstance(payload.get("campaign"), dict) else {}
        if payload.get("schema") == SCHEMA and campaign_block.get("campaign") == campaign:
            yield index, payload


def collect(rows: Iterable[Tuple[int, Dict[str, Any]]], args: argparse.Namespace):
    samples: List[Sample] = []
    counts: Counter = Counter()
    reasons: Dict[int, str] = {}
    label_registry: Dict[str, str] = {}
    previous: Optional[Sample] = None
    seen: Dict[Tuple[int, int, Optional[int]], Sample] = {}
    epoch = run = 0
    origin: Optional[float] = None

    for row_id, payload in rows:
        counts["rows_read"] += 1
        try:
            timestamp = utc(payload["published_at_utc"])
            if origin is None:
                origin = timestamp
            if ((args.start is not None and timestamp < args.start) or
                    (args.end is not None and timestamp >= args.end) or
                    timestamp < origin + args.skip_seconds):
                counts["outside_selected_time"] += 1
                continue
            sample, disposition, labels = parse_sample(row_id, payload)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"row {row_id}: {exc}") from exc
        if sample is None:
            counts[disposition] += 1
            continue
        label_registry.update(labels)
        key = (sample.reset, sample.sequence, sample.pps)
        if key in seen:
            old = seen[key]
            if (sample.lap_ns, sample.races, sample.signature) != (old.lap_ns, old.races, old.signature):
                raise ValueError(f"row {row_id}: repeated fragment has conflicting science/configuration")
            counts["duplicate_fragment"] += 1
            continue
        seen[key] = sample

        changes: List[str] = []
        if previous is not None:
            if sample.reset != previous.reset:
                changes.append("statistics reset")
            if sample.signature != previous.signature:
                changes.append("acquisition/core settings changed")
            if sample.sequence <= previous.sequence:
                changes.append("sequence restart")
            if sample.pps is not None and previous.pps is not None and sample.pps < previous.pps:
                changes.append("PPS restart")
            if sample.time <= previous.time:
                changes.append("publication time moved backward")
        if previous is None or changes:
            epoch += 1
            run += 1
            reasons[epoch] = "; ".join(changes) or "start of selected observations"
        elif sample.sequence != previous.sequence + 1 or not 0.25 <= sample.time - previous.time <= 1.75:
            run += 1
            counts["continuity_breaks"] += 1

        sample = replace(sample, epoch=epoch, run=run)
        samples.append(sample)
        previous = sample
        counts["used"] += 1

    return samples, counts, reasons, label_registry


def exclusion_reason(path: str) -> Optional[str]:
    if path == TARGET_PATH or path.startswith("photons.race.flight_ns."):
        return "target family / same accepted-lap population"
    if path in IDENTIFIER_PATHS:
        return "identifier rather than physical predictor"
    if any(path.startswith(prefix) for prefix in EXCLUDED_PREFIXES):
        return "cumulative or alternate LAP-derived surface"
    if path.startswith("photons.core."):
        name = path.rsplit(".", 1)[-1]
        if name not in CORE_ALLOWED:
            return "core statistic derived from the current LAP distribution"
    if path.startswith("photons.science.") and not any(
            path == prefix or path.startswith(prefix) for prefix in SCIENCE_ALLOWED_PREFIXES):
        return "science predictor/residual/cycle surface derived from LAP processing"
    if path in {
        "photons.race.reference_cycles",
        "photons.race.reference_gate_cycles",
        "photons.projection.last_end_dwt",
        "photons.projection.last_end_gnss_ns",
        "photons.projection.last_lap_gnss_ns",
        "photons.projection.last_raw_cycles",
        "photons.projection.last_start_dwt",
        "photons.projection.last_start_gnss_ns",
    }:
        return "direct LAP predictor/reference representation"
    return None


def is_counter(path: str) -> bool:
    name = path.rsplit(".", 1)[-1]
    if "this_fragment" in name or name.startswith("last_") or name.startswith("max_"):
        return False
    if name.endswith("_total"):
        return True
    if name in NETWORK_COUNTER_BASENAMES:
        return True
    if name in COUNTER_BASENAMES:
        return True
    if path.startswith("photons.projection.") and name.endswith("_count"):
        return True
    if path.startswith("photons.interrupt.") and "_count" in name:
        return True
    if path.startswith("photons.science.exclusion_reasons.") and "this_fragment" not in name:
        return True
    if path.startswith("photons.science.") and name in {"candidate_count", "count"}:
        return True
    return False


def unit_for(path: str, rate: bool = False) -> str:
    name = path.rsplit(".", 1)[-1]
    if name in {"active", "valid", "read_ok", "stale", "present", "sensor_present",
                "source_fresh", "matched_target_utc", "used_prior_announcement",
                "currently_undervolted", "previously_undervolted", "final"} or name.endswith("_valid"):
        return "0/1"
    mapping = (
        ("temperature_c", "degC"),
        ("_pct", "%"),
        ("_hpa", "hPa"),
        ("_mbps", "Mb/s"),
        ("_ppb", "ppb"),
        ("_hz", "Hz"),
        ("_ns", "ns"),
        ("_ms", "ms"),
        ("_minutes", "min"),
        ("_cycles", "cycles"),
        ("_wh", "Wh"),
        ("wh_", "Wh"),
        ("bytes_recv", "bytes"),
        ("bytes_sent", "bytes"),
        ("packets_recv", "packets"),
        ("packets_sent", "packets"),
        ("percent", "%"),
        ("_deg", "deg"),
        ("_gb", "GB"),
        ("_mb", "MB"),
        ("_v", "V"),
        ("volts", "V"),
        ("amps", "A"),
        ("watts", "W"),
        ("_m", "m"),
        ("_s", "s"),
    )
    unit = "count"
    for suffix, candidate in mapping:
        if name == suffix or name.endswith(suffix) or (suffix == "wh_" and name.startswith("wh_")):
            unit = candidate
            break
    return f"{unit}/s" if rate else unit


def prepare_variables(samples: Sequence[Sample], label_registry: Dict[str, str]):
    """Remove circular surfaces and convert cumulative counters to rates."""
    specs: Dict[str, VariableSpec] = {}
    excluded: Dict[str, str] = {}
    transformed: Dict[str, str] = {}
    previous_by_run: Dict[int, Sample] = {}
    origin = samples[0].time
    prepared: List[Sample] = []

    for sample in samples:
        values: Dict[str, float] = {
            ELAPSED_PATH: sample.time - origin,
            ACCEPTED_PATH: float(sample.races),
        }
        specs.setdefault(ELAPSED_PATH, VariableSpec(
            ELAPSED_PATH, ELAPSED_PATH, "elapsed time", "s", "derived", STRUCTURAL_NOTES[ELAPSED_PATH],
        ))
        specs.setdefault(ACCEPTED_PATH, VariableSpec(
            ACCEPTED_PATH, TARGET_PATH.rsplit(".", 1)[0] + ".n",
            "accepted races this fragment", "count", "derived",
        ))
        previous = previous_by_run.get(sample.run)
        for path, value in sample.values.items():
            reason = exclusion_reason(path)
            if reason is not None:
                excluded[path] = reason
                continue
            label = label_registry.get(path, path)
            if is_counter(path):
                key = f"rate.{path}"
                transformed[path] = key
                if previous is not None and path in previous.values:
                    delta = value - previous.values[path]
                    elapsed = sample.time - previous.time
                    if delta >= 0 and elapsed > 0:
                        values[key] = delta / elapsed
                specs.setdefault(key, VariableSpec(
                    key=key,
                    source_path=path,
                    label=f"rate({label})",
                    unit=unit_for(path, rate=True),
                    mode="counter_rate",
                    note="Run-local first difference divided by publication elapsed time.",
                ))
                continue
            values[path] = value
            specs.setdefault(path, VariableSpec(
                key=path,
                source_path=path,
                label=label,
                unit=unit_for(path),
                mode="gauge",
                note=STRUCTURAL_NOTES.get(path, ""),
            ))
        prepared.append(replace(sample, values=values))
        previous_by_run[sample.run] = sample

    return prepared, specs, excluded, transformed


def centered(values: Sequence[float]) -> List[float]:
    average = st.fmean(values)
    return [value - average for value in values]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return math.fsum(x * y for x, y in zip(a, b))


def pearson(x: Sequence[float], y: Sequence[float]) -> Optional[float]:
    if len(x) < MIN_FIT or len(x) != len(y):
        return None
    xc, yc = centered(x), centered(y)
    xx, yy = dot(xc, xc), dot(yc, yc)
    if xx <= EPSILON or yy <= EPSILON:
        return None
    return max(-1.0, min(1.0, dot(xc, yc) / math.sqrt(xx * yy)))


def ranks(values: Sequence[float]) -> List[float]:
    order = sorted(range(len(values)), key=values.__getitem__)
    result = [0.0] * len(values)
    index = 0
    while index < len(order):
        end = index + 1
        while end < len(order) and values[order[end]] == values[order[index]]:
            end += 1
        rank = (index + end - 1) / 2.0 + 1.0
        for position in order[index:end]:
            result[position] = rank
        index = end
    return result


def spearman(x: Sequence[float], y: Sequence[float]) -> Optional[float]:
    return pearson(ranks(x), ranks(y)) if len(x) >= MIN_FIT else None


def fit(x: Sequence[float], y: Sequence[float], times: Optional[Sequence[float]] = None) -> Optional[Fit]:
    if len(x) < MIN_FIT or len(x) != len(y):
        return None
    xc, yc = centered(x), centered(y)
    xx = dot(xc, xc)
    if xx <= EPSILON:
        return None
    t0 = 0.0
    drift = 0.0
    if times is None:
        beta = dot(xc, yc) / xx
        residual = [b - beta * a for a, b in zip(xc, yc)]
    else:
        if len(times) != len(x):
            raise ValueError("time vector length mismatch")
        tc = centered(times)
        tt = dot(tc, tc)
        if tt <= EPSILON:
            return None
        xt = dot(xc, tc) / tt
        yt = dot(yc, tc) / tt
        xr = [a - xt * t for a, t in zip(xc, tc)]
        yr = [b - yt * t for b, t in zip(yc, tc)]
        xr2 = dot(xr, xr)
        if xr2 <= 1e-10 * xx:
            return None
        beta = dot(xr, yr) / xr2
        drift = yt - beta * xt
        t0 = st.fmean(times)
        residual = [b - beta * a - drift * t for a, b, t in zip(xc, yc, tc)]
    yy = dot(yc, yc)
    r2 = 1.0 - dot(residual, residual) / yy if yy > EPSILON else 0.0
    residual_sd = st.stdev(residual)
    return Fit(st.fmean(x), t0, st.fmean(y), beta, drift, r2, residual_sd)


def detrended_vectors(x: Sequence[float], y: Sequence[float], times: Sequence[float]):
    x_time = fit(times, x)
    y_time = fit(times, y)
    if x_time is None or y_time is None:
        return None
    xr = [value - x_time.predict(t) for value, t in zip(x, times)]
    yr = [value - y_time.predict(t) for value, t in zip(y, times)]
    if dot(xr, xr) <= 1e-10 * dot(centered(x), centered(x)):
        return None
    if dot(yr, yr) <= 1e-10 * dot(centered(y), centered(y)):
        return None
    return xr, yr


def detrended_r(x: Sequence[float], y: Sequence[float], times: Sequence[float]) -> Optional[float]:
    residuals = detrended_vectors(x, y, times)
    return None if residuals is None else pearson(*residuals)


def pairs_for(samples: Sequence[Sample], key: str):
    selected = [sample for sample in samples if key in sample.values]
    if not selected:
        return [], [], [], []
    origin = selected[0].time
    return (
        selected,
        [sample.values[key] for sample in selected],
        [sample.lap_ns for sample in selected],
        [sample.time - origin for sample in selected],
    )


def within_epoch_detrended(samples: Sequence[Sample], key: str) -> Optional[float]:
    by_epoch: Dict[int, List[Sample]] = defaultdict(list)
    for sample in samples:
        if key in sample.values:
            by_epoch[sample.epoch].append(sample)
    xr_all: List[float] = []
    yr_all: List[float] = []
    for group in by_epoch.values():
        if len(group) < MIN_FIT:
            continue
        x = [sample.values[key] for sample in group]
        y = [sample.lap_ns for sample in group]
        t0 = group[0].time
        times = [sample.time - t0 for sample in group]
        residuals = detrended_vectors(x, y, times)
        if residuals is None:
            continue
        xr, yr = residuals
        xr_all.extend(xr)
        yr_all.extend(yr)
    return pearson(xr_all, yr_all)


def adjacent_changes(samples: Sequence[Sample], key: str):
    dx: List[float] = []
    dy: List[float] = []
    previous: Optional[Sample] = None
    for sample in samples:
        if previous is not None and sample.run == previous.run:
            if key in previous.values and key in sample.values:
                dx.append(sample.values[key] - previous.values[key])
                dy.append(sample.lap_ns - previous.lap_ns)
        previous = sample
    return dx, dy


def block_points(samples: Sequence[Sample], key: str, width: int, minimum_coverage: float):
    points: List[Point] = []
    omitted_seconds = 0
    bucket: List[Sample] = []
    current_run: Optional[int] = None
    block_index = 0
    minimum_values = max(1, math.ceil(width * minimum_coverage))

    def consume(items: List[Sample], index: int) -> None:
        nonlocal omitted_seconds
        values = [sample.values[key] for sample in items if key in sample.values]
        if len(items) == width and len(values) >= minimum_values:
            points.append(Point(
                run=items[0].run,
                epoch=items[0].epoch,
                index=index,
                time=st.fmean(sample.time for sample in items),
                x=st.fmean(values),
                lap_ns=st.fmean(sample.lap_ns for sample in items),
            ))
        else:
            omitted_seconds += len(items)

    for sample in samples:
        if current_run is None or sample.run != current_run:
            if bucket:
                consume(bucket, block_index)
            bucket = []
            current_run = sample.run
            block_index = 0
        bucket.append(sample)
        if len(bucket) == width:
            consume(bucket, block_index)
            block_index += 1
            bucket = []
    if bucket:
        consume(bucket, block_index)
    return points, omitted_seconds


def block_adjacent(points: Sequence[Point]):
    adjacent = [(a, b) for a, b in zip(points, points[1:])
                if a.run == b.run and b.index == a.index + 1]
    return ([b.x - a.x for a, b in adjacent],
            [b.lap_ns - a.lap_ns for a, b in adjacent])


def common_lag_support(points: Sequence[Point], requested_steps: int):
    lookup = {(point.run, point.index): point for point in points}
    for steps in range(requested_steps, -1, -1):
        targets = [point for point in points if all(
            (point.run, point.index + offset) in lookup for offset in range(-steps, steps + 1)
        )]
        if len(targets) >= MIN_HOLDOUT or steps == 0:
            return steps, lookup, targets
    raise AssertionError("lag support search exhausted")


def lag_pairs(targets: Sequence[Point], lookup: Dict[Tuple[int, int], Point], lag: int,
              time_origin: float):
    x = [lookup[(point.run, point.index - lag)].x for point in targets]
    y = [point.lap_ns for point in targets]
    times = [point.time - time_origin for point in targets]
    return x, y, times


def rmse(actual: Sequence[float], predicted: Sequence[float]) -> float:
    return math.sqrt(st.fmean((a - p) ** 2 for a, p in zip(actual, predicted)))


def improvement(reference: Optional[float], candidate: Optional[float]) -> Optional[float]:
    if reference is None or candidate is None or reference <= 0:
        return None
    return (1.0 - candidate / reference) * 100.0


def lag_analysis(points: Sequence[Point], width: int, max_lag_seconds: int) -> LagResult:
    empty = LagResult(0, 0, None, None, None, None, None, None, 0,
                      None, None, None, None, None, None)
    if len(points) < MIN_HOLDOUT:
        return empty
    requested_steps = max_lag_seconds // width
    steps, lookup, targets = common_lag_support(points, requested_steps)
    if len(targets) < MIN_FIT:
        return empty

    time_origin = targets[0].time
    scored = []
    for lag in range(-steps, steps + 1):
        x, y, times = lag_pairs(targets, lookup, lag, time_origin)
        model = fit(x, y)
        if model is None:
            continue
        scored.append((lag, pearson(x, y), detrended_r(x, y, times), model.r2))
    zero = next((item for item in scored if item[0] == 0), None)
    best_signed = None
    if scored:
        best_signed = max(scored, key=lambda item: (
            abs(item[2]) if item[2] is not None else abs(item[1] or 0.0),
            -abs(item[0]),
            -item[0],
        ))

    cut = 2 * len(targets) // 3
    train = targets[:cut]
    test = targets[cut + 1:]
    chosen_lag: Optional[int] = None
    mean_rmse = time_rmse = predictor_rmse = combined_rmse = None
    if len(train) >= 8 and len(test) >= 4:
        candidates = []
        for lag in range(steps + 1):
            x, y, _ = lag_pairs(train, lookup, lag, time_origin)
            model = fit(x, y)
            if model is not None:
                candidates.append((lag, model))
        if candidates:
            chosen_lag, predictor_model = max(candidates, key=lambda item: (item[1].r2, -item[0]))
            x, y, times = lag_pairs(train, lookup, chosen_lag, time_origin)
            tx, ty, test_times = lag_pairs(test, lookup, chosen_lag, time_origin)
            time_model = fit(times, y)
            combined_model = fit(x, y, times)
            baseline = [st.fmean(y)] * len(test)
            predictor_prediction = [predictor_model.predict(value) for value in tx]
            mean_rmse = rmse(ty, baseline) * 1000.0
            predictor_rmse = rmse(ty, predictor_prediction) * 1000.0
            if time_model is not None:
                time_rmse = rmse(ty, [time_model.predict(value) for value in test_times]) * 1000.0
            if combined_model is not None:
                combined_rmse = rmse(
                    ty,
                    [combined_model.predict(value, moment) for value, moment in zip(tx, test_times)],
                ) * 1000.0

    return LagResult(
        support_n=len(targets),
        scan_seconds=steps * width,
        zero_r=None if zero is None else zero[1],
        zero_detrended_r=None if zero is None else zero[2],
        best_signed_lag_s=None if best_signed is None else best_signed[0] * width,
        best_signed_r=None if best_signed is None else best_signed[1],
        best_signed_detrended_r=None if best_signed is None else best_signed[2],
        chosen_causal_lag_s=None if chosen_lag is None else chosen_lag * width,
        holdout_n=len(test) if len(train) >= 8 and len(test) >= 4 else 0,
        holdout_mean_rmse_ps=mean_rmse,
        holdout_time_rmse_ps=time_rmse,
        holdout_predictor_rmse_ps=predictor_rmse,
        holdout_combined_rmse_ps=combined_rmse,
        holdout_predictor_improvement_vs_time_pct=improvement(time_rmse, predictor_rmse),
        holdout_combined_improvement_vs_time_pct=improvement(time_rmse, combined_rmse),
    )


def configuration_only(samples: Sequence[Sample], key: str) -> bool:
    global_values = {sample.values[key] for sample in samples if key in sample.values}
    if len(global_values) < 2:
        return False
    by_epoch: Dict[int, set] = defaultdict(set)
    for sample in samples:
        if key in sample.values:
            by_epoch[sample.epoch].add(sample.values[key])
    return all(len(values) <= 1 for values in by_epoch.values())


def analyze_variable(samples: Sequence[Sample], spec: VariableSpec, args: argparse.Namespace) -> Optional[Metric]:
    selected, x, y, times = pairs_for(samples, spec.key)
    coverage = len(selected) / len(samples) if samples else 0.0
    distinct = len(set(x))
    if len(x) < args.min_observations or coverage < args.min_coverage or distinct < 2:
        return None

    model = fit(x, y)
    split = len(x) // 2
    first_r = pearson(x[:split], y[:split]) if split >= MIN_FIT else None
    second_r = pearson(x[split:], y[split:]) if len(x) - split >= MIN_FIT else None
    dx, dy = adjacent_changes(samples, spec.key)
    points, _ = block_points(samples, spec.key, args.bin_seconds, args.min_block_coverage)
    bx = [point.x for point in points]
    by = [point.lap_ns for point in points]
    bt = [point.time - points[0].time for point in points] if points else []
    bdx, bdy = block_adjacent(points)
    lag = lag_analysis(points, args.bin_seconds, args.max_lag_seconds)
    plot_name = slug(spec.key) + ".svg"

    return Metric(
        key=spec.key,
        label=spec.label,
        source_path=spec.source_path,
        unit=spec.unit,
        mode=spec.mode,
        note=spec.note,
        n=len(x),
        coverage_pct=coverage * 100.0,
        distinct=distinct,
        minimum=min(x),
        maximum=max(x),
        span=max(x) - min(x),
        mean=st.fmean(x),
        raw_r=pearson(x, y),
        spearman_r=spearman(x, y),
        predictor_time_r=pearson(x, times),
        detrended_r=detrended_r(x, y, times),
        within_epoch_detrended_r=within_epoch_detrended(samples, spec.key),
        delta_n=len(dx),
        delta_r=pearson(dx, dy),
        first_half_r=first_r,
        second_half_r=second_r,
        slope_ps_per_unit=None if model is None else model.beta * 1000.0,
        span_effect_ps=None if model is None else model.beta * (max(x) - min(x)) * 1000.0,
        residual_sd_ps=None if model is None else model.residual_sd_ns * 1000.0,
        block_n=len(points),
        block_r=pearson(bx, by),
        block_detrended_r=detrended_r(bx, by, bt) if len(points) >= MIN_FIT else None,
        block_delta_r=pearson(bdx, bdy),
        configuration_only=configuration_only(samples, spec.key),
        lag_support_n=lag.support_n,
        lag_scan_seconds=lag.scan_seconds,
        zero_lag_r=lag.zero_r,
        zero_lag_detrended_r=lag.zero_detrended_r,
        best_signed_lag_s=lag.best_signed_lag_s,
        best_signed_r=lag.best_signed_r,
        best_signed_detrended_r=lag.best_signed_detrended_r,
        chosen_causal_lag_s=lag.chosen_causal_lag_s,
        holdout_n=lag.holdout_n,
        holdout_mean_rmse_ps=lag.holdout_mean_rmse_ps,
        holdout_time_rmse_ps=lag.holdout_time_rmse_ps,
        holdout_predictor_rmse_ps=lag.holdout_predictor_rmse_ps,
        holdout_combined_rmse_ps=lag.holdout_combined_rmse_ps,
        holdout_predictor_improvement_vs_time_pct=lag.holdout_predictor_improvement_vs_time_pct,
        holdout_combined_improvement_vs_time_pct=lag.holdout_combined_improvement_vs_time_pct,
        plot_file=f"plots/{plot_name}",
    )


def temperature_plateaus(samples: Sequence[Sample], seconds: int, span_c: float) -> List[Plateau]:
    if seconds < MIN_FIT:
        return []
    by_run: Dict[int, List[Sample]] = defaultdict(list)
    for sample in samples:
        by_run[sample.run].append(sample)
    candidates: List[Tuple[int, int, Plateau]] = []
    stride = max(1, seconds // 4)
    for run, group in by_run.items():
        for start in range(0, max(0, len(group) - seconds + 1), stride):
            window = group[start:start + seconds]
            if len(window) < seconds:
                continue
            if any(TEMPERATURE_PATH not in sample.values for sample in window):
                continue
            temperatures = [sample.values[TEMPERATURE_PATH] for sample in window]
            observed_span = max(temperatures) - min(temperatures)
            if observed_span > span_c:
                continue
            times = [sample.time - window[0].time for sample in window]
            laps = [sample.lap_ns for sample in window]
            trend = fit(times, laps)
            if trend is None:
                continue
            duration = window[-1].time - window[0].time
            plateau = Plateau(
                start_utc=stamp(window[0].time),
                end_utc=stamp(window[-1].time),
                duration_s=duration,
                observations=len(window),
                temperature_min_c=min(temperatures),
                temperature_max_c=max(temperatures),
                temperature_span_c=observed_span,
                lap_start_ns=laps[0],
                lap_end_ns=laps[-1],
                endpoint_change_ps=(laps[-1] - laps[0]) * 1000.0,
                fitted_change_ps=trend.beta * duration * 1000.0,
                fitted_slope_ps_per_minute=trend.beta * 60_000.0,
                lap_span_ps=(max(laps) - min(laps)) * 1000.0,
            )
            candidates.append((run, start, plateau))

    # Keep the strongest non-overlapping windows so the report is not an echo
    # chamber of nearly identical sliding intervals.
    chosen: List[Tuple[int, int, Plateau]] = []
    for candidate in sorted(candidates, key=lambda item: abs(item[2].fitted_change_ps), reverse=True):
        run, start, plateau = candidate
        if any(run == other_run and abs(start - other_start) < seconds for other_run, other_start, _ in chosen):
            continue
        chosen.append(candidate)
    return [item[2] for item in sorted(chosen, key=lambda item: item[2].start_utc)]


def slug(value: str) -> str:
    stem = re.sub(r"[^a-zA-Z0-9._-]+", "-", value).strip("-.")[:100] or "variable"
    digest = hashlib.sha1(value.encode("utf-8")).hexdigest()[:8]
    return f"{stem}-{digest}"


def downsample(items: Sequence[Any], limit: int) -> List[Any]:
    if len(items) <= limit:
        return list(items)
    if limit < 2:
        return [items[0]]
    return [items[round(index * (len(items) - 1) / (limit - 1))] for index in range(limit)]


def safe_range(values: Sequence[float], pad_fraction: float = 0.05) -> Tuple[float, float]:
    low, high = min(values), max(values)
    if low == high:
        pad = max(1.0, abs(low) * 0.01)
    else:
        pad = (high - low) * pad_fraction
    return low - pad, high + pad


def map_value(value: float, low: float, high: float, start: float, end: float) -> float:
    return (start + end) / 2.0 if high == low else start + (value - low) / (high - low) * (end - start)


def svg_text(x: float, y: float, text: str, css: str = "label", anchor: str = "start") -> str:
    return f'<text x="{x:.1f}" y="{y:.1f}" class="{css}" text-anchor="{anchor}">{html.escape(text)}</text>'


def axis_ticks(low: float, high: float, count: int = 5) -> List[float]:
    if count <= 1 or low == high:
        return [low]
    return [low + (high - low) * index / (count - 1) for index in range(count)]


def scatter_panel(x: Sequence[float], y: Sequence[float], bounds, title: str,
                  x_label: str, y_label: str, fit_line: bool = True) -> str:
    left, top, width, height = bounds
    if len(x) < 2 or len(set(x)) < 2 or len(set(y)) < 2:
        return (f'<rect x="{left}" y="{top}" width="{width}" height="{height}" class="panel"/>'
                + svg_text(left + 12, top + 24, title, "title")
                + svg_text(left + width / 2, top + height / 2, "insufficient variation", "muted", "middle"))
    x_low, x_high = safe_range(x)
    y_low, y_high = safe_range(y)
    plot_left, plot_right = left + 64, left + width - 20
    plot_top, plot_bottom = top + 36, top + height - 54
    parts = [f'<rect x="{left}" y="{top}" width="{width}" height="{height}" class="panel"/>',
             svg_text(left + 12, top + 24, title, "title")]
    for tick in axis_ticks(x_low, x_high):
        px = map_value(tick, x_low, x_high, plot_left, plot_right)
        parts.append(f'<line x1="{px:.1f}" y1="{plot_top}" x2="{px:.1f}" y2="{plot_bottom}" class="grid"/>')
        parts.append(svg_text(px, plot_bottom + 18, compact(tick, 3), "tick", "middle"))
    for tick in axis_ticks(y_low, y_high):
        py = map_value(tick, y_low, y_high, plot_bottom, plot_top)
        parts.append(f'<line x1="{plot_left}" y1="{py:.1f}" x2="{plot_right}" y2="{py:.1f}" class="grid"/>')
        parts.append(svg_text(plot_left - 8, py + 4, compact(tick, 3), "tick", "end"))
    for xv, yv in zip(x, y):
        px = map_value(xv, x_low, x_high, plot_left, plot_right)
        py = map_value(yv, y_low, y_high, plot_bottom, plot_top)
        parts.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="1.8" class="point"/>')
    if fit_line:
        model = fit(x, y)
        if model is not None:
            y1, y2 = model.predict(x_low), model.predict(x_high)
            parts.append(
                f'<line x1="{plot_left}" y1="{map_value(y1, y_low, y_high, plot_bottom, plot_top):.1f}" '
                f'x2="{plot_right}" y2="{map_value(y2, y_low, y_high, plot_bottom, plot_top):.1f}" class="fit"/>'
            )
    parts.append(svg_text((plot_left + plot_right) / 2, top + height - 10, x_label, "axis", "middle"))
    parts.append(svg_text(left + 13, (plot_top + plot_bottom) / 2, y_label, "axis"))
    return "".join(parts)


def time_panel(times: Sequence[float], x: Sequence[float], y: Sequence[float], bounds,
               x_label: str) -> str:
    left, top, width, height = bounds
    parts = [f'<rect x="{left}" y="{top}" width="{width}" height="{height}" class="panel"/>',
             svg_text(left + 12, top + 24, "Time alignment (standardized)", "title")]
    if len(times) < 2 or len(set(times)) < 2:
        parts.append(svg_text(left + width / 2, top + height / 2, "insufficient time support", "muted", "middle"))
        return "".join(parts)
    x_sd = st.stdev(x) if len(set(x)) > 1 else 0.0
    y_sd = st.stdev(y) if len(set(y)) > 1 else 0.0
    if x_sd <= 0 or y_sd <= 0:
        parts.append(svg_text(left + width / 2, top + height / 2, "constant series", "muted", "middle"))
        return "".join(parts)
    xz = [(value - st.fmean(x)) / x_sd for value in x]
    yz = [(value - st.fmean(y)) / y_sd for value in y]
    z_low, z_high = safe_range(xz + yz)
    t_low, t_high = min(times), max(times)
    plot_left, plot_right = left + 64, left + width - 20
    plot_top, plot_bottom = top + 42, top + height - 54
    for tick in axis_ticks(t_low, t_high):
        px = map_value(tick, t_low, t_high, plot_left, plot_right)
        parts.append(f'<line x1="{px:.1f}" y1="{plot_top}" x2="{px:.1f}" y2="{plot_bottom}" class="grid"/>')
        parts.append(svg_text(px, plot_bottom + 18, compact(tick / 60.0, 2), "tick", "middle"))
    for tick in axis_ticks(z_low, z_high):
        py = map_value(tick, z_low, z_high, plot_bottom, plot_top)
        parts.append(f'<line x1="{plot_left}" y1="{py:.1f}" x2="{plot_right}" y2="{py:.1f}" class="grid"/>')
        parts.append(svg_text(plot_left - 8, py + 4, compact(tick, 2), "tick", "end"))

    def path_for(values: Sequence[float], css: str) -> str:
        points = [
            f"{map_value(moment, t_low, t_high, plot_left, plot_right):.1f},"
            f"{map_value(value, z_low, z_high, plot_bottom, plot_top):.1f}"
            for moment, value in zip(times, values)
        ]
        return f'<polyline points="{" ".join(points)}" class="{css}"/>'

    parts.append(path_for(yz, "lap-line"))
    parts.append(path_for(xz, "variable-line"))
    parts.append(svg_text(plot_left, top + 37, "LAP", "lap-key"))
    parts.append(svg_text(plot_left + 62, top + 37, x_label, "variable-key"))
    parts.append(svg_text((plot_left + plot_right) / 2, top + height - 10, "elapsed minutes", "axis", "middle"))
    parts.append(svg_text(left + 13, (plot_top + plot_bottom) / 2, "z", "axis"))
    return "".join(parts)


def write_plot(path: Path, metric: Metric, samples: Sequence[Sample], max_points: int) -> None:
    selected, x, y, times = pairs_for(samples, metric.key)
    zipped = downsample(list(zip(times, x, y)), max_points)
    pt = [item[0] for item in zipped]
    px = [item[1] for item in zipped]
    py = [item[2] for item in zipped]
    lap_center = st.fmean(y)
    py_ps = [(value - lap_center) * 1000.0 for value in py]

    residuals = detrended_vectors(x, y, times)
    if residuals is None:
        dxr: List[float] = []
        dyr: List[float] = []
    else:
        rx, ry = residuals
        residual_pairs = downsample(list(zip(rx, [value * 1000.0 for value in ry])), max_points)
        dxr = [item[0] for item in residual_pairs]
        dyr = [item[1] for item in residual_pairs]
    delta_x, delta_y = adjacent_changes(samples, metric.key)
    delta_pairs = downsample(list(zip(delta_x, [value * 1000.0 for value in delta_y])), max_points)
    ddx = [item[0] for item in delta_pairs]
    ddy = [item[1] for item in delta_pairs]

    summary = (
        f"N={metric.n:,}  raw r={fmt(metric.raw_r)}  detrended r={fmt(metric.detrended_r)}  "
        f"delta r={fmt(metric.delta_r)}  block r={fmt(metric.block_r)}"
    )
    lag_summary = (
        f"best signed lag={metric.best_signed_lag_s if metric.best_signed_lag_s is not None else 'n/a'} s  "
        f"causal training lag={metric.chosen_causal_lag_s if metric.chosen_causal_lag_s is not None else 'n/a'} s  "
        f"holdout improvement vs time={fmt(metric.holdout_predictor_improvement_vs_time_pct, 2)}%"
    )
    note = metric.note or "No special structural warning."
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="1040" viewBox="0 0 1200 1040">',
        """<style>
            text { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; fill: #18202a; }
            .heading { font-size: 21px; font-weight: 700; }
            .subheading { font-size: 13px; }
            .note { font-size: 11px; fill: #5b6470; }
            .panel { fill: #ffffff; stroke: #9aa4b2; stroke-width: 1; }
            .title { font-size: 14px; font-weight: 700; }
            .label, .axis { font-size: 11px; }
            .tick { font-size: 9px; fill: #59636f; }
            .muted { font-size: 12px; fill: #7b8490; }
            .grid { stroke: #e2e7ec; stroke-width: 1; }
            .point { fill: #285a8e; fill-opacity: 0.32; }
            .fit { stroke: #b33d2e; stroke-width: 2; }
            .lap-line { fill: none; stroke: #285a8e; stroke-width: 1.4; }
            .variable-line { fill: none; stroke: #b36b1e; stroke-width: 1.4; }
            .lap-key { font-size: 11px; fill: #285a8e; font-weight: 700; }
            .variable-key { font-size: 11px; fill: #b36b1e; font-weight: 700; }
        </style>""",
        '<rect width="1200" height="1040" fill="#f5f7f9"/>',
        svg_text(42, 36, metric.label, "heading"),
        svg_text(42, 60, summary, "subheading"),
        svg_text(42, 79, lag_summary, "subheading"),
        svg_text(42, 97, f"unit={metric.unit}; mode={metric.mode}; {note}", "note"),
        scatter_panel(px, py_ps, (40, 120, 550, 400), "Raw predictor versus LAP",
                      f"{metric.label} ({metric.unit})", "LAP offset (ps)"),
        time_panel(pt, px, py_ps, (610, 120, 550, 400), metric.label),
        scatter_panel(dxr, dyr, (40, 540, 550, 400), "After removing linear time trends",
                      f"predictor residual ({metric.unit})", "LAP residual (ps)"),
        scatter_panel(ddx, ddy, (610, 540, 550, 400), "Adjacent-fragment changes",
                      f"delta predictor ({metric.unit})", "delta LAP (ps)"),
        svg_text(40, 982,
                 f"range={compact(metric.minimum)}..{compact(metric.maximum)} {metric.unit}; "
                 f"slope={compact(metric.slope_ps_per_unit)} ps/{metric.unit}; "
                 f"fitted span effect={compact(metric.span_effect_ps)} ps", "subheading"),
        svg_text(40, 1004,
                 "Screening report only: autocorrelation, shared drift, mathematical coupling, and multiple comparisons remain.",
                 "note"),
        "</svg>",
    ]
    path.write_text("".join(svg), encoding="utf-8")


def write_summary_csv(path: Path, metrics: Sequence[Metric]) -> None:
    rows = [asdict(metric) for metric in metrics]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["key"])
        writer.writeheader()
        writer.writerows(rows)


def write_exclusions_csv(path: Path, excluded: Dict[str, str], transformed: Dict[str, str],
                         omitted: Dict[str, str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("source_path", "disposition", "detail"))
        for source, reason in sorted(excluded.items()):
            writer.writerow((source, "excluded", reason))
        for source, key in sorted(transformed.items()):
            writer.writerow((source, "transformed", key))
        for key, reason in sorted(omitted.items()):
            writer.writerow((key, "omitted", reason))


def write_plateaus_csv(path: Path, plateaus: Sequence[Plateau]) -> None:
    fields = list(asdict(plateaus[0])) if plateaus else [field.name for field in Plateau.__dataclass_fields__.values()]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for plateau in plateaus:
            writer.writerow(asdict(plateau))


def write_samples_csv(path: Path, samples: Sequence[Sample], metrics: Sequence[Metric]) -> None:
    keys = [metric.key for metric in metrics]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("row_id", "published_at_utc", "sequence", "reset_count", "epoch", "run",
                         "lap_mean_ns", "accepted_races", *keys))
        for sample in samples:
            writer.writerow((
                sample.row_id, stamp(sample.time), sample.sequence, sample.reset, sample.epoch, sample.run,
                sample.lap_ns, sample.races, *(sample.values.get(key, "") for key in keys),
            ))


def metric_value(metric: Metric, field: str) -> str:
    value = getattr(metric, field)
    if value is None:
        return ""
    if isinstance(value, bool):
        return "yes" if value else ""
    if isinstance(value, int):
        return str(value)
    return f"{value:.6g}"


def html_metric_table(metrics: Sequence[Metric]) -> str:
    columns = (
        ("label", "Variable"),
        ("n", "N"),
        ("coverage_pct", "Coverage %"),
        ("raw_r", "Raw r"),
        ("spearman_r", "Spearman"),
        ("predictor_time_r", "X~time r"),
        ("detrended_r", "Detrended r"),
        ("within_epoch_detrended_r", "Within-epoch r"),
        ("delta_r", "Delta r"),
        ("first_half_r", "First-half r"),
        ("second_half_r", "Second-half r"),
        ("block_r", "Block r"),
        ("configuration_only", "Config-only"),
        ("best_signed_lag_s", "Best lag s"),
        ("holdout_predictor_improvement_vs_time_pct", "Holdout vs time %"),
        ("slope_ps_per_unit", "Slope ps/unit"),
        ("span_effect_ps", "Span effect ps"),
    )
    head = "".join(f'<th data-key="{key}">{html.escape(title)}</th>' for key, title in columns)
    rows = []
    for metric in metrics:
        cells = []
        for key, _ in columns:
            if key == "label":
                value = (f'<a href="{html.escape(metric.plot_file)}">{html.escape(metric.label)}</a>'
                         f'<div class="path">{html.escape(metric.key)}</div>')
            else:
                raw = getattr(metric, key)
                sort_value = "" if raw is None else str(raw)
                value = f'<span data-value="{html.escape(sort_value)}">{html.escape(metric_value(metric, key))}</span>'
            cells.append(f"<td>{value}</td>")
        rows.append("<tr>" + "".join(cells) + "</tr>")
    return f'<table id="metrics"><thead><tr>{head}</tr></thead><tbody>{"".join(rows)}</tbody></table>'


def leaderboard(metrics: Sequence[Metric], field: str, title: str, limit: int = 12,
                positive_only: bool = False) -> str:
    candidates = [metric for metric in metrics if getattr(metric, field) is not None and metric.key != ELAPSED_PATH]
    if positive_only:
        candidates = [metric for metric in candidates if getattr(metric, field) > 0]
        candidates.sort(key=lambda metric: getattr(metric, field), reverse=True)
    else:
        candidates.sort(key=lambda metric: abs(getattr(metric, field)), reverse=True)
    items = "".join(
        f'<li><a href="{html.escape(metric.plot_file)}">{html.escape(metric.label)}</a>: '
        f'{html.escape(metric_value(metric, field))}</li>'
        for metric in candidates[:limit]
    )
    return f"<section><h2>{html.escape(title)}</h2><ol>{items or '<li>none</li>'}</ol></section>"


def write_index(path: Path, campaign: str, samples: Sequence[Sample], counts: Counter,
                reasons: Dict[int, str], metrics: Sequence[Metric], plateaus: Sequence[Plateau],
                omitted: Dict[str, str], args: argparse.Namespace) -> None:
    plateau_rows = "".join(
        "<tr>" + "".join(f"<td>{html.escape(str(value))}</td>" for value in (
            plateau.start_utc, plateau.end_utc, f"{plateau.temperature_span_c:.6f}",
            f"{plateau.fitted_change_ps:+.3f}", f"{plateau.fitted_slope_ps_per_minute:+.3f}",
            f"{plateau.lap_span_ps:.3f}",
        )) + "</tr>" for plateau in plateaus[:20]
    )
    epoch_rows = "".join(
        f"<li>Epoch {epoch}: {html.escape(reason)}</li>" for epoch, reason in sorted(reasons.items())
    )
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>PHOTONS forensics — {html.escape(campaign)}</title>
<style>
body {{ font-family: system-ui, sans-serif; margin: 28px; color: #17212b; background: #f5f7f9; }}
a {{ color: #174f86; }} h1 {{ margin-bottom: 4px; }} h2 {{ margin-top: 30px; }}
.card {{ background: white; border: 1px solid #cfd6dd; padding: 18px; margin: 16px 0; border-radius: 6px; }}
.grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 16px; }}
table {{ border-collapse: collapse; width: 100%; background: white; font-size: 12px; }}
th, td {{ border: 1px solid #d5dbe1; padding: 6px; text-align: right; vertical-align: top; }}
th {{ position: sticky; top: 0; background: #e9eef3; cursor: pointer; }}
th:first-child, td:first-child {{ text-align: left; min-width: 330px; }}
.path {{ color: #697480; font-family: monospace; font-size: 10px; margin-top: 3px; }}
.warning {{ border-left: 5px solid #b16d18; }} code {{ font-family: ui-monospace, monospace; }}
</style></head><body>
<h1>PHOTONS forensic correlation reconnaissance</h1>
<p><strong>Campaign:</strong> {html.escape(campaign)} &nbsp; <strong>Interval:</strong> {stamp(samples[0].time)} .. {stamp(samples[-1].time)}</p>
<div class="card warning"><strong>Interpretation boundary.</strong> This report screens hypotheses. It does not prove cause, does not install a correction, and deliberately reports no IID p-values. Shared time drift, autocorrelation, related telemetry surfaces, mathematical coupling, and multiple comparisons can all produce impressive-looking associations.</div>
<div class="card"><strong>Target:</strong> <code>{TARGET_PATH}</code>; one equal-weight observation per populated one-second fragment.<br>
<strong>Usable seconds:</strong> {len(samples):,}; <strong>epochs:</strong> {len(reasons):,}; <strong>runs:</strong> {len(set(sample.run for sample in samples)):,}; <strong>analyzed predictors:</strong> {len(metrics):,}; <strong>omitted after screening:</strong> {len(omitted):,}.<br>
<strong>Block width:</strong> {args.bin_seconds}s; <strong>maximum requested lag:</strong> {args.max_lag_seconds}s; <strong>minimum variable coverage:</strong> {args.min_coverage:.0%}; <strong>minimum block coverage:</strong> {args.min_block_coverage:.0%}.</div>
<div class="grid">
{leaderboard(metrics, 'detrended_r', 'Largest time-detrended associations')}
{leaderboard(metrics, 'within_epoch_detrended_r', 'Largest within-epoch detrended associations')}
{leaderboard(metrics, 'delta_r', 'Largest adjacent-change associations')}
{leaderboard(metrics, 'holdout_predictor_improvement_vs_time_pct', 'Predictors that beat elapsed time on holdout', positive_only=True)}
</div>
<h2>Temperature plateaus</h2>
<p>Sliding {args.plateau_seconds}s windows with BME280 span at most {args.plateau_temperature_span_c:.6f} C. Strongest non-overlapping windows are retained.</p>
<table><thead><tr><th>Start</th><th>End</th><th>Temperature span C</th><th>Fitted LAP change ps</th><th>Slope ps/min</th><th>LAP span ps</th></tr></thead><tbody>{plateau_rows or '<tr><td colspan="6">No qualifying plateau windows.</td></tr>'}</tbody></table>
<h2>All analyzed predictors</h2>
<p>Click a column heading to sort. Click a variable for its four-panel SVG: raw scatter, standardized time alignment, linearly detrended scatter, and adjacent changes.</p>
{html_metric_table(metrics)}
<h2>Epoch boundaries</h2><ul>{epoch_rows}</ul>
<h2>Files</h2><ul><li><a href="summary.csv">summary.csv</a></li><li><a href="report.json">report.json</a></li><li><a href="field_dispositions.csv">field_dispositions.csv</a></li><li><a href="temperature_plateaus.csv">temperature_plateaus.csv</a></li></ul>
<h2>Method</h2><div class="card"><p>Raw Pearson can be dominated by a shared monotonic trend. Spearman catches monotonic nonlinearity. Detrended r removes one linear time trend from both series. Within-epoch r performs that operation separately inside stable acquisition epochs. Delta r compares adjacent-fragment changes. Block results average non-overlapping consecutive seconds without interpolating gaps. Lag scans use the same target blocks at every tested lag. Positive lag means the predictor is earlier than LAP. The causal holdout lag is selected only on the first two thirds and frozen before evaluating the last third, with one block withheld between them.</p><p>Cumulative counters are converted to run-local rates. LAP-derived means, cycle centers, residuals, envelopes, and cumulative LAP populations are excluded to prevent circular discoveries. DWT scale remains visible with an explicit coupling warning because clock-scale behavior is one of the stated hypotheses.</p></div>
<script>
for (const th of document.querySelectorAll('#metrics th')) {{
  th.addEventListener('click', () => {{
    const table = th.closest('table'); const body = table.tBodies[0]; const index = th.cellIndex;
    const ascending = th.dataset.asc !== 'true';
    for (const other of table.querySelectorAll('th')) delete other.dataset.asc;
    th.dataset.asc = String(ascending);
    const rows = Array.from(body.rows);
    rows.sort((a, b) => {{
      const av = a.cells[index].querySelector('[data-value]')?.dataset.value ?? a.cells[index].innerText;
      const bv = b.cells[index].querySelector('[data-value]')?.dataset.value ?? b.cells[index].innerText;
      const an = Number(av), bn = Number(bv); let result;
      if (av !== '' && bv !== '' && Number.isFinite(an) && Number.isFinite(bn)) result = an - bn;
      else result = av.localeCompare(bv);
      return ascending ? result : -result;
    }});
    for (const row of rows) body.appendChild(row);
  }});
}}
</script></body></html>"""
    path.write_text(document, encoding="utf-8")


def print_leaderboard(metrics: Sequence[Metric], field: str, title: str, limit: int,
                      positive_only: bool = False) -> None:
    candidates = [metric for metric in metrics if getattr(metric, field) is not None and metric.key != ELAPSED_PATH]
    if positive_only:
        candidates = [metric for metric in candidates if getattr(metric, field) > 0]
        candidates.sort(key=lambda metric: getattr(metric, field), reverse=True)
    else:
        candidates.sort(key=lambda metric: abs(getattr(metric, field)), reverse=True)
    print(f"\n{title}")
    print("  value       raw_r      time_r     delta_r    variable")
    for metric in candidates[:limit]:
        print(f"  {metric_value(metric, field):>9}  {fmt(metric.raw_r):>9}  "
              f"{fmt(metric.predictor_time_r):>9}  {fmt(metric.delta_r):>9}    {metric.label}")
    if not candidates:
        print("  none")


def arguments(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("campaign")
    parser.add_argument("--input", type=Path, help="read PHOTONS JSON/JSONL instead of PostgreSQL")
    parser.add_argument("--output-dir", type=Path, help="report directory (default ./<campaign>-photons-forensics)")
    parser.add_argument("--bin-seconds", type=int, default=60,
                        help="non-overlapping block duration for lag/holdout analysis (default 60)")
    parser.add_argument("--max-lag-seconds", type=int, default=1800,
                        help="scan both lag directions up to this duration (default 1800)")
    parser.add_argument("--min-observations", type=int, default=30)
    parser.add_argument("--min-coverage", type=float, default=0.50,
                        help="minimum fraction of usable seconds containing a predictor (default 0.50)")
    parser.add_argument("--min-block-coverage", type=float, default=0.80,
                        help="minimum predictor coverage inside each complete block (default 0.80)")
    parser.add_argument("--plateau-seconds", type=int, default=600,
                        help="temperature-flat window duration (default 600)")
    parser.add_argument("--plateau-temperature-span-c", type=float, default=0.02,
                        help="maximum BME280 span in a flat window (default 0.02 C)")
    parser.add_argument("--include", help="regular expression selecting predictor keys")
    parser.add_argument("--exclude", help="regular expression removing predictor keys")
    parser.add_argument("--top", type=int, default=15, help="rows per console leaderboard (default 15)")
    parser.add_argument("--max-plot-points", type=int, default=3000,
                        help="deterministic display-only downsample per plot panel (default 3000)")
    parser.add_argument("--skip-seconds", type=int, default=0,
                        help="omit this duration from campaign's first stored publication")
    parser.add_argument("--start", type=utc, help="inclusive timestamp with timezone")
    parser.add_argument("--end", type=utc, help="exclusive timestamp with timezone")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--csv", type=Path, help="optional wide CSV of all usable one-second observations")
    args = parser.parse_args(argv[1:])

    positive_ints = (args.bin_seconds, args.min_observations, args.plateau_seconds,
                     args.top, args.max_plot_points, args.batch_size)
    if min(positive_ints) < 1 or min(args.max_lag_seconds, args.skip_seconds) < 0:
        parser.error("durations/counts must be positive and lag/skip nonnegative")
    if not 0 < args.min_coverage <= 1 or not 0 < args.min_block_coverage <= 1:
        parser.error("coverage values must be in (0, 1]")
    if args.plateau_temperature_span_c < 0:
        parser.error("plateau temperature span must be nonnegative")
    if args.start is not None and args.end is not None and args.start >= args.end:
        parser.error("--start must precede --end")
    if args.include:
        re.compile(args.include)
    if args.exclude:
        re.compile(args.exclude)
    if args.output_dir is None:
        campaign_stem = re.sub(r"[^a-zA-Z0-9._-]+", "-", args.campaign).strip("-.") or "campaign"
        args.output_dir = Path(f"{campaign_stem}-photons-forensics")
    return args


def main(argv: Sequence[str]) -> int:
    args = arguments(argv)
    source = file_rows(args.input, args.campaign) if args.input else database_rows(args)
    samples, counts, reasons, labels = collect(source, args)
    print(f"PHOTONS FORENSIC CORRELATION ANALYSIS — {args.campaign}")
    print(f"  Target: {TARGET_PATH}; equal weight per populated one-second fragment.")
    print("  Counts: " + "; ".join(f"{key}={value:,}" for key, value in counts.items()))
    if not samples:
        print("  No usable observations in the selected campaign/time interval.")
        return 1

    samples, specs, excluded, transformed = prepare_variables(samples, labels)
    include = re.compile(args.include) if args.include else None
    exclude = re.compile(args.exclude) if args.exclude else None
    omitted: Dict[str, str] = {}
    metrics: List[Metric] = []

    for key in sorted(specs):
        if include and not include.search(key):
            omitted[key] = "did not match --include"
            continue
        if exclude and exclude.search(key):
            omitted[key] = "matched --exclude"
            continue
        present = [sample.values[key] for sample in samples if key in sample.values]
        coverage = len(present) / len(samples)
        if len(present) < args.min_observations:
            omitted[key] = f"only {len(present)} observations; minimum {args.min_observations}"
            continue
        if coverage < args.min_coverage:
            omitted[key] = f"coverage {coverage:.3f}; minimum {args.min_coverage:.3f}"
            continue
        if len(set(present)) < 2:
            omitted[key] = "constant over selected observations"
            continue
        metric = analyze_variable(samples, specs[key], args)
        if metric is None:
            omitted[key] = "insufficient identifiable variation"
        else:
            metrics.append(metric)

    metrics.sort(key=lambda metric: (
        -(abs(metric.detrended_r) if metric.detrended_r is not None else -1.0),
        -(abs(metric.delta_r) if metric.delta_r is not None else -1.0),
        metric.key,
    ))
    plateaus = temperature_plateaus(samples, args.plateau_seconds, args.plateau_temperature_span_c)

    output = args.output_dir
    plots = output / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    for old in plots.glob("*.svg"):
        old.unlink()
    for metric in metrics:
        write_plot(output / metric.plot_file, metric, samples, args.max_plot_points)

    write_summary_csv(output / "summary.csv", metrics)
    write_exclusions_csv(output / "field_dispositions.csv", excluded, transformed, omitted)
    write_plateaus_csv(output / "temperature_plateaus.csv", plateaus)
    if args.csv:
        write_samples_csv(args.csv, samples, metrics)

    report = {
        "schema": "PHOTONS_FORENSICS_V1",
        "campaign": args.campaign,
        "target": TARGET_PATH,
        "selected_start_utc": stamp(samples[0].time),
        "selected_end_utc": stamp(samples[-1].time),
        "usable_seconds": len(samples),
        "epochs": reasons,
        "runs": len(set(sample.run for sample in samples)),
        "counts": dict(counts),
        "arguments": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "metrics": [asdict(metric) for metric in metrics],
        "temperature_plateaus": [asdict(plateau) for plateau in plateaus],
        "interpretation": [
            "screening does not establish causation",
            "adjacent observations are autocorrelated",
            "many related predictors are screened",
            "cumulative counters are analyzed as run-local rates",
            "LAP-derived predictor surfaces are excluded",
            "DWT scale fields are retained with a mathematical-coupling warning",
        ],
    }
    (output / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    write_index(output / "index.html", args.campaign, samples, counts, reasons, metrics, plateaus, omitted, args)

    print(f"  Interval: {stamp(samples[0].time)} .. {stamp(samples[-1].time)}")
    print(f"  Epochs={len(reasons):,}; runs={len(set(sample.run for sample in samples)):,}; "
          f"numeric predictors analyzed={len(metrics):,}; omitted={len(omitted):,}")
    print_leaderboard(metrics, "detrended_r", "LARGEST TIME-DETRENDED ASSOCIATIONS", args.top)
    print_leaderboard(metrics, "within_epoch_detrended_r", "LARGEST WITHIN-EPOCH ASSOCIATIONS", args.top)
    print_leaderboard(metrics, "delta_r", "LARGEST ADJACENT-CHANGE ASSOCIATIONS", args.top)
    print_leaderboard(metrics, "holdout_predictor_improvement_vs_time_pct",
                      "HOLDOUT PREDICTORS THAT BEAT ELAPSED TIME", args.top, positive_only=True)

    print("\nTEMPERATURE PLATEAUS")
    if not plateaus:
        print(f"  None found at {args.plateau_seconds}s / {args.plateau_temperature_span_c:.6f} C.")
    else:
        for plateau in sorted(plateaus, key=lambda item: abs(item.fitted_change_ps), reverse=True)[:args.top]:
            print(f"  {plateau.start_utc} .. {plateau.end_utc}  "
                  f"Tspan={plateau.temperature_span_c:.6f} C  "
                  f"fitted LAP change={plateau.fitted_change_ps:+.3f} ps  "
                  f"slope={plateau.fitted_slope_ps_per_minute:+.3f} ps/min")

    print(f"\n  HTML report: {output / 'index.html'}")
    print(f"  Summary CSV: {output / 'summary.csv'}")
    print("  No database, firmware, LAP, campaign, or correction value was changed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
