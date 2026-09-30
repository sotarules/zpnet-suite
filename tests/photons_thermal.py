"""Characterize BME280 temperature versus PHOTONS fragment mean flight time.

    zt photons_thermal Campaign1 Campaign2 Campaign3
    zt photons_thermal Campaign1 --bin-seconds 60 --csv /tmp/thermal.csv
    zt photons_thermal Campaign1 --max-lag-seconds 600
    zt photons_thermal Campaign1 --input /tmp/photons.jsonl

Reads only each named LANTERN campaign's PHOTONS_V1 rows. The paired facts are
from the SAME message: environment.temperature_c and photons.race.flight_ns.mean.
The latter is already nanoseconds for that one-second accepted population;
cumulative campaign/instrument means are never regression inputs. Every usable
second has equal weight, regardless of the number of flights within it.

Reports a linear candidate function, temperature-binned shape, warming/cooling
agreement, a frozen later-data prediction, and cross-campaign prediction where
recorded configurations and temperature ranges overlap. No normalization is
installed and no stored data or live statistics are changed.

Nonoverlapping time blocks reduce second-to-second noise. Blocks and optional
lag alignment never cross gaps, resets, recovery, or recorded setting changes.
Lag +N means temperature N seconds BEFORE the measured lap. Lag selection uses
training data only. The later holdout is separated by the maximum tested lag
plus one block; held-out outcomes never select the model. Default lag is zero.

Stored BME280 snapshots may repeat: read_ok/stale describe sensor-read outcome,
not precise sample age. Missing temperature/quality observations are counted
and omitted, never filled in or assumed successful. A large row count is not
a count of independent thermal experiments. No IID p-values or standard errors
are claimed. A high R2 alone cannot distinguish heat from elapsed time or
establish a deterministic law.

Only the standard library and zpnet.shared.db are needed. main(argv) includes
the program name, matching the existing zt runner. --input accepts PHOTONS JSON,
a JSON array or JSONL, optionally prefixed with PHOTONS. --start/--end require a
timezone. Unrecorded hardware/firmware changes must be separated by the operator.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics as st
import sys
from collections import Counter
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


SCHEMA = "PHOTONS_V1"
MIN_FIT = 3
MIN_TRAIN = 8
MIN_TEST = 4


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


@dataclass(frozen=True)
class Sample:
    row_id: int
    time: float
    sequence: int
    reset: int
    pps: Optional[int]
    temperature: float
    lap_ns: float
    races: int
    signature: str
    update: Optional[int]
    generation: Optional[int]
    receipt: bool
    epoch: int = 0
    run: int = 0


def parse_sample(row_id: int, p: Dict[str, Any]) -> Tuple[Optional[Sample], str]:
    """Optional environmental context may be absent; science remains strict."""
    if p["schema"] != SCHEMA:
        raise ValueError(f"expected {SCHEMA}")
    ph = p["photons"]
    if ph["fragment_period_ns"] != 1_000_000_000:
        raise ValueError("thermal analysis requires one-second fragments")
    if ph["snapshot_ok"] is not True:
        return None, "snapshot_not_ok"
    # SYSTEM serves REPORT before its first platform poll completes; PHOTONS
    # lawfully carries {} for environmental context that is not available yet.
    env = p.get("environment")
    if env is None:
        return None, "temperature_not_recorded"
    if not isinstance(env, dict):
        raise ValueError("environment must be an object or null")
    if env.get("temperature_c") is None:
        return None, "temperature_not_recorded"
    if any(key not in env for key in ("sensor_present", "read_ok", "stale")):
        return None, "environment_quality_not_recorded"
    if env["sensor_present"] is not True or env["read_ok"] is not True or env["stale"] is not False:
        return None, "environment_unavailable_or_stale"
    race = ph["race"]
    flight = race["flight_ns"]
    n = integer(flight["n"])
    if n == 0:
        return None, "no_accepted_races"
    if race["active"] is not True:
        return None, "autonomous_races_inactive"
    core = ph.get("core") or {}
    if core and integer(core["sequence"]) != integer(p["sequence"]):
        raise ValueError("core diagnostic belongs to a different fragment")
    if core and integer(core["retained_count"]) != n:
        raise ValueError("core retained count does not match flight population")
    # Per-second center/radius/state are data, not configuration changes.
    signature = json.dumps({
        "race": {k: race.get(k) for k in (
            "schema", "cadence_ns", "pulse_ns", "capture_gate", "capture_min_ns", "capture_max_ns",
            "launch_bracket_max_cycles", "flight_interpretation", "launch_timing_policy")},
        "core": {k: core.get(k) for k in (
            "algorithm", "minimum_count", "minimum_radius_cycles", "mad_multiplier", "minimum_retained_pct")},
    }, sort_keys=True)
    projection = ph.get("projection") or {}
    pps = projection.get("anchor_pps_count")
    update = ph["stats"].get("update_count")
    generation = (ph.get("recovery") or {}).get("generation")
    return Sample(
        row_id=row_id, time=utc(p["published_at_utc"]), sequence=integer(p["sequence"]),
        reset=integer(ph["stats"]["reset_count"]), pps=None if pps is None else integer(pps),
        temperature=number(env["temperature_c"]), lap_ns=number(flight["mean"]), races=n,
        signature=signature,
        update=None if update is None else integer(update),
        generation=None if generation is None else integer(generation),
        receipt=p.get("recovery_receipt") is not None,
    ), "used"


def database_rows(args: argparse.Namespace) -> Iterable[Tuple[int, Dict[str, Any]]]:
    from zpnet.shared.db import open_db

    # Project just the analysis surface; do not transfer large recovery rings
    # or cumulative science populations. Keep the exact campaign predicate.
    sql = """
        SELECT id, jsonb_build_object(
            'schema', payload -> 'schema',
            'sequence', payload -> 'sequence',
            'published_at_utc', payload -> 'published_at_utc',
            'environment', payload -> 'environment',
            'recovery_receipt', payload -> 'recovery_receipt',
            'photons', jsonb_build_object(
                'snapshot_ok', payload #> '{photons,snapshot_ok}',
                'fragment_period_ns', payload #> '{photons,fragment_period_ns}',
                'stats', jsonb_build_object(
                    'reset_count', payload #> '{photons,stats,reset_count}',
                    'update_count', payload #> '{photons,stats,update_count}'),
                'recovery', jsonb_build_object('generation', payload #> '{photons,recovery,generation}'),
                'projection', jsonb_build_object('anchor_pps_count', payload #> '{photons,projection,anchor_pps_count}'),
                'core', payload #> '{photons,core}',
                'race', jsonb_build_object(
                    'active', payload #> '{photons,race,active}',
                    'flight_ns', payload #> '{photons,race,flight_ns}',
                    'schema', payload #> '{photons,race,schema}',
                    'cadence_ns', payload #> '{photons,race,cadence_ns}',
                    'pulse_ns', payload #> '{photons,race,pulse_ns}',
                    'capture_gate', payload #> '{photons,race,capture_gate}',
                    'capture_min_ns', payload #> '{photons,race,capture_min_ns}',
                    'capture_max_ns', payload #> '{photons,race,capture_max_ns}',
                    'launch_bracket_max_cycles', payload #> '{photons,race,launch_bracket_max_cycles}',
                    'flight_interpretation', payload #> '{photons,race,flight_interpretation}',
                    'launch_timing_policy', payload #> '{photons,race,launch_timing_policy}'
                )
            )
        ) AS thermal_payload
        FROM campaign_detail
        WHERE campaign_type = %s AND campaign = %s AND payload ->> 'schema' = %s
        ORDER BY id ASC
    """
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="photons_thermal")
        cur.itersize = args.batch_size
        cur.execute(sql, ("LANTERN", args.campaign, SCHEMA))
        for row in cur:
            p = row["thermal_payload"]
            yield int(row["id"]), json.loads(p) if isinstance(p, str) else p


def file_rows(path: Path, campaign: str) -> Iterable[Tuple[int, Dict[str, Any]]]:
    text = path.read_text(encoding="utf-8-sig").strip()
    if text.startswith("["):
        records = json.loads(text)
    else:
        if text.startswith("PHOTONS "):
            text = text[len("PHOTONS "):]
        try:
            records = [json.loads(text)]
        except json.JSONDecodeError:
            records = [json.loads(line.removeprefix("PHOTONS ")) for line in text.splitlines() if line.strip()]
    for i, record in enumerate(records, 1):
        p = record.get("payload", record)
        if p.get("schema") == SCHEMA and (p.get("campaign") or {}).get("campaign") == campaign:
            yield i, p


def collect(rows: Iterable[Tuple[int, Dict[str, Any]]], args: argparse.Namespace):
    samples: List[Sample] = []
    counts: Counter = Counter()
    reasons: Dict[int, str] = {}
    prev: Optional[Sample] = None
    epoch = run = 0
    origin = None
    for row_id, p in rows:
        counts["rows_read"] += 1
        try:
            t = utc(p["published_at_utc"])
            if origin is None:
                origin = t
            if ((args.start is not None and t < args.start) or
                    (args.end is not None and t >= args.end) or t < origin + args.skip_seconds):
                counts["outside_selected_time"] += 1
                continue
            sample, disposition = parse_sample(row_id, p)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"row {row_id}: {exc}") from exc
        if sample is None:
            counts[disposition] += 1
            if counts[disposition] == 1 and disposition in (
                    "temperature_not_recorded", "environment_quality_not_recorded"):
                print(f"  {args.campaign}: first {disposition} at row {row_id}; omitted from thermal pairs.")
            continue
        changes = []
        if prev is not None:
            if sample.receipt:
                changes.append("producer recovery receipt")
            if sample.generation != prev.generation:
                changes.append("recovery generation changed")
            if sample.reset != prev.reset:
                changes.append("statistics reset")
            # Only consecutive, same-generation republications are duplicates.
            # A reboot may reuse old counters: never globally deduplicate them.
            identity = (sample.reset, sample.sequence, sample.pps, sample.update)
            previous_identity = (prev.reset, prev.sequence, prev.pps, prev.update)
            same_generation = sample.generation == prev.generation
            repeated_receipt = not sample.receipt or prev.receipt
            if same_generation and repeated_receipt and identity == previous_identity:
                if (sample.lap_ns, sample.races, sample.signature) != (prev.lap_ns, prev.races, prev.signature):
                    raise ValueError(f"row {row_id}: repeated fragment has conflicting science/settings")
                counts["duplicate_fragment"] += 1
                continue
            if sample.signature != prev.signature:
                changes.append("acquisition/core settings changed")
            if (sample.sequence <= prev.sequence or
                    (sample.pps is not None and prev.pps is not None and sample.pps < prev.pps) or
                    (sample.update is not None and prev.update is not None and sample.update <= prev.update)):
                changes.append("sequence/PPS/update restart")
            if sample.time <= prev.time:
                changes.append("publication time moved backward")
        if prev is None or changes:
            epoch += 1
            run += 1
            reasons[epoch] = "; ".join(changes) or "start of selected observations"
        else:
            counter_gap = (sample.sequence != prev.sequence + 1 or
                           (sample.update is not None and prev.update is not None and
                            sample.update != prev.update + 1))
            publication_gap = not 0.25 <= sample.time - prev.time <= 1.75
            if counter_gap or publication_gap:
                run += 1
                counts["continuity_breaks"] += 1
                counts["counter_gap_breaks"] += int(counter_gap)
                counts["publication_timing_breaks"] += int(publication_gap)
        sample = replace(sample, epoch=epoch, run=run)
        samples.append(sample)
        prev = sample
        counts["used"] += 1
    return samples, counts, reasons


@dataclass(frozen=True)
class Point:
    run: int
    index: int
    time: float
    temperature: float
    lap_ns: float


def blocks(samples: Sequence[Sample], width: int) -> Tuple[List[Point], int]:
    out = []
    bucket: List[Sample] = []
    index = 0
    used = 0
    for s in samples:
        if bucket and s.run != bucket[0].run:
            bucket = []
            index = 0
        elif out and not bucket and s.run != out[-1].run:
            index = 0
        bucket.append(s)
        if len(bucket) == width:
            out.append(Point(s.run, index, st.fmean(v.time for v in bucket),
                             st.fmean(v.temperature for v in bucket), st.fmean(v.lap_ns for v in bucket)))
            index += 1
            used += width
            bucket = []
    return out, len(samples) - used


def centered(values: Sequence[float]) -> List[float]:
    avg = st.fmean(values)
    return [v - avg for v in values]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return math.fsum(x * y for x, y in zip(a, b))


def pearson(x: Sequence[float], y: Sequence[float]) -> Optional[float]:
    if len(x) < MIN_FIT:
        return None
    a, b = centered(x), centered(y)
    xx, yy = dot(a, a), dot(b, b)
    return max(-1.0, min(1.0, dot(a, b) / math.sqrt(xx * yy))) if xx > 1e-24 and yy > 1e-24 else None


@dataclass(frozen=True)
class Fit:
    x0: float
    t0: float
    y0: float
    beta: float
    drift: float
    r2: float
    residual_sd_ns: float

    def predict(self, x: float, t: float) -> float:
        return self.y0 + self.beta * (x - self.x0) + self.drift * (t - self.t0)


def fit(x: Sequence[float], y: Sequence[float], times: Optional[Sequence[float]] = None) -> Optional[Fit]:
    if len(x) < MIN_FIT:
        return None
    xc, yc = centered(x), centered(y)
    xx = dot(xc, xc)
    if xx <= 1e-24:
        return None
    t0 = drift = 0.0
    if times is None:
        beta = dot(xc, yc) / xx
        residual = [b - beta * a for a, b in zip(xc, yc)]
    else:
        tc = centered(times)
        tt = dot(tc, tc)
        if tt <= 1e-24:
            return None
        xt, yt = dot(xc, tc) / tt, dot(yc, tc) / tt
        xr = [a - xt * t for a, t in zip(xc, tc)]
        yr = [b - yt * t for b, t in zip(yc, tc)]
        xr2 = dot(xr, xr)
        # Temperature proportional to elapsed time cannot identify two slopes.
        if xr2 <= 1e-10 * xx:
            return None
        beta = dot(xr, yr) / xr2
        drift = yt - beta * xt
        t0 = st.fmean(times)
        residual = [b - beta * a - drift * t for a, b, t in zip(xc, yc, tc)]
    yy = dot(yc, yc)
    r2 = 1.0 - dot(residual, residual) / yy if yy > 1e-24 else 0.0
    return Fit(st.fmean(x), t0, st.fmean(y), beta, drift, r2, st.stdev(residual))


def detrended_r(x, y, t) -> Optional[float]:
    tx, ty = fit(t, x), fit(t, y)
    if tx is None or ty is None:
        return None
    xr = [v - tx.predict(s, 0) for v, s in zip(x, t)]
    yr = [v - ty.predict(s, 0) for v, s in zip(y, t)]
    if (dot(xr, xr) <= 1e-10 * dot(centered(x), centered(x)) or
            dot(yr, yr) <= 1e-10 * dot(centered(y), centered(y))):
        return None
    return pearson(xr, yr)


def fmt(v: Optional[float], digits: int = 4) -> str:
    return "n/a" if v is None else f"{v:.{digits}f}"


def association(points: Sequence[Point]) -> Optional[Fit]:
    x, y = [p.temperature for p in points], [p.lap_ns for p in points]
    model = fit(x, y)
    if model is None:
        print("  Function unavailable: need at least three blocks with temperature variation.")
        return None
    print(f"  Candidate: LAP_ns = {model.y0:.9f} + ({model.beta:+.9f}) * (BME280_C - {model.x0:.6f})")
    print(f"  Sensitivity {model.beta*1000:+.3f} ps/C; r={fmt(pearson(x,y))}; R2={model.r2:.4f}")
    print(f"  Between-block LAP SD {st.stdev(y)*1000:.3f} ps -> fitted residual SD {model.residual_sd_ns*1000:.3f} ps")
    times = [p.time-points[0].time for p in points]
    controlled = fit(x, y, times)
    print(f"  After removing a linear time trend: r={fmt(detrended_r(x,y,times))}; "
          f"temperature slope={fmt(None if controlled is None else controlled.beta*1000,3)} ps/C")
    if controlled is None:
        print("  Temperature and elapsed time do not separately identify a thermal slope here.")
    pairs = [(a,b) for a,b in zip(points,points[1:]) if a.run == b.run and b.index == a.index+1]
    print(f"  Adjacent changes corr(delta T, delta LAP)={fmt(pearson([b.temperature-a.temperature for a,b in pairs], [b.lap_ns-a.lap_ns for a,b in pairs]))}")
    return model


def directions(points: Sequence[Point], threshold: float) -> Dict[Tuple[int, int], str]:
    """Centered local temperature rate; never compare across a broken run."""
    lookup = {(p.run,p.index): p for p in points}
    out = {}
    for p in points:
        before = lookup.get((p.run,p.index-1), p)
        after = lookup.get((p.run,p.index+1), p)
        rate = ((after.temperature-before.temperature)*60/(after.time-before.time)
                if after.time > before.time else 0.0)
        out[(p.run,p.index)] = "warming" if rate > threshold else "cooling" if rate < -threshold else "steady"
    return out


def temperature_groups(points: Sequence[Point], requested_width: float):
    low, high = min(p.temperature for p in points), max(p.temperature for p in points)
    # Keep the console table compact; never silently print hundreds of bins.
    width = requested_width * max(1, math.ceil((high-low)/(12*requested_width)))
    origin = math.floor(low/width)*width
    groups = {}
    for p in points:
        index = math.floor((p.temperature-origin)/width)
        groups.setdefault(index, []).append(p)
    return width, origin, groups


def shape_report(points: Sequence[Point], model: Optional[Fit], args) -> None:
    direction = directions(points, args.direction_threshold_c_per_minute)
    width, origin, groups = temperature_groups(points, args.temperature_bin_c)
    print(f"\n  Temperature shape ({width:g} C bins; SD describes block means, not individual laps):")
    print("    mean_T_C    blocks       mean_LAP_ns     SD_ps     mean-fit_ps")
    for _, bucket in sorted(groups.items()):
        x, y = [p.temperature for p in bucket], [p.lap_ns for p in bucket]
        residual = None if model is None else st.fmean(y)-model.predict(st.fmean(x),0)
        print(f"    {st.fmean(x):8.4f} {len(y):9,d} {st.fmean(y):17.9f} "
              f"{fmt(st.stdev(y)*1000 if len(y)>1 else None,3):>9} "
              f"{fmt(None if residual is None else residual*1000,3):>15}")
    counts = Counter(direction.values())
    print(f"\n  Warming/cooling check: {dict(counts)}; steady band +/-{args.direction_threshold_c_per_minute:g} C/min")
    comparisons = []
    if model is not None:
        for index, bucket in sorted(groups.items()):
            warm = [p for p in bucket if direction[(p.run,p.index)] == "warming"]
            cool = [p for p in bucket if direction[(p.run,p.index)] == "cooling"]
            if len(warm) < 2 or len(cool) < 2:
                continue
            tw, tc = st.fmean(p.temperature for p in warm), st.fmean(p.temperature for p in cool)
            # Adjust unequal within-bin temperatures to a common temperature.
            # This is a descriptive linear adjustment, not a new calibration.
            gap = st.fmean(p.lap_ns for p in warm)-st.fmean(p.lap_ns for p in cool)-model.beta*(tw-tc)
            comparisons.append((origin+index*width, len(warm),len(cool),tw,tc,gap*1000))
    if not comparisons:
        print("    Insufficient shared-temperature warming/cooling support (need >=2 blocks each per bin).")
        print("    A single warmup cannot establish a repeatable temperature-only function.")
        return
    print("    bin_low_C    warm cool    warm_T_C  cool_T_C   warm-minus-cool_ps")
    for low, nw, nc, tw, tc, gap in comparisons:
        print(f"    {low:9.4f} {nw:7d} {nc:4d} {tw:11.4f} {tc:9.4f} {gap:21.3f}")
    print(f"    Largest absolute direction gap: {max(abs(c[-1]) for c in comparisons):.3f} ps.")
    print("    Gaps use the fitted linear slope to adjust within-bin T differences; curvature can affect them.")


def prediction_errors(actual: Sequence[float], predicted: Sequence[float]):
    errors = [a-b for a,b in zip(actual,predicted)]
    return math.sqrt(st.fmean(e*e for e in errors)), st.fmean(errors), st.stdev(errors) if len(errors)>1 else None


def holdout_report(points: Sequence[Point], args) -> None:
    # All lag candidates use identical targets. A run must contain the entire
    # earlier predictor history; gaps are never interpolated or bridged.
    steps = args.max_lag_seconds // args.bin_seconds
    lookup = {(p.run,p.index): p for p in points}
    targets = [p for p in points if p.index >= steps]
    cut = 2*len(targets)//3
    train, test = targets[:cut], targets[cut+steps+1:]
    print(f"\n  Frozen later-data prediction (lags 0..{steps*args.bin_seconds}s; {len(train)} train / {len(test)} test blocks):")
    if len(train) < MIN_TRAIN or len(test) < MIN_TEST:
        print("    Need >=8 training and >=4 test blocks after lag support and separation.")
        print("    Collect longer, reduce --bin-seconds, or reduce --max-lag-seconds.")
        return
    def temperatures(selected, lag):
        return [lookup[(p.run,p.index-lag)].temperature for p in selected]
    y = [p.lap_ns for p in train]
    times = [p.time for p in train]
    candidates = [(lag,fit(temperatures(train,lag),y)) for lag in range(steps+1)]
    candidates = [(lag,m) for lag,m in candidates if m is not None]
    if not candidates:
        print("    Training temperature has no identifiable slope.")
        return
    lag, model = min(candidates, key=lambda item: (-item[1].r2,item[0]))
    x, tx = temperatures(train,lag), temperatures(test,lag)
    ty, tt = [p.lap_ns for p in test], [p.time for p in test]
    trend = fit(times,y)
    print(f"    Train {stamp(train[0].time)} .. {stamp(train[-1].time)}; test {stamp(test[0].time)} .. {stamp(test[-1].time)}")
    print(f"    Training-selected lag +{lag*args.bin_seconds}s; slope {model.beta*1000:+.3f} ps/C")
    print(f"    Frozen function: LAP_ns = {model.y0:.9f} + ({model.beta:+.9f}) * (T_C - {model.x0:.6f})")
    low, high = min(x), max(x)
    outside = sum(t < low or t > high for t in tx)
    print(f"    Training T {low:.4f}..{high:.4f} C; {outside}/{len(tx)} test predictors outside that range.")
    predictions = [("training mean",[st.fmean(y)]*len(test)),
                   ("temperature",[model.predict(t,0) for t in tx])]
    if trend is not None:
        predictions.append(("elapsed time only",[trend.predict(t,0) for t in tt]))
    print("    model                  RMSE_ps       bias_ps   residual_SD_ps")
    scores = {}
    for name, predicted in predictions:
        rmse, bias, sd = prediction_errors(ty,predicted)
        scores[name] = rmse
        print(f"    {name:20s} {rmse*1000:10.3f} {bias*1000:13.3f} {fmt(None if sd is None else sd*1000,3):>16}")
    if scores["training mean"] > 0:
        print(f"    Temperature RMSE reduction vs training mean: {(1-scores['temperature']/scores['training mean'])*100:+.2f}%")
    if "elapsed time only" in scores and scores["temperature"] >= scores["elapsed time only"]:
        print("    Temperature did not beat elapsed-time prediction on this holdout.")
    if lag == steps and steps:
        print("    Chosen lag reaches the search limit; its optimum is not bracketed.")
    print("    Coefficients/lag were frozen before evaluation; no test-set recentering.")


@dataclass
class Series:
    label: str
    campaign: str
    signature: str
    points: List[Point]
    model: Optional[Fit]


def epoch_report(campaign: str, samples: Sequence[Sample], reason: str, args) -> Series:
    first, last = samples[0], samples[-1]
    x, y = [s.temperature for s in samples], [s.lap_ns for s in samples]
    label = f"{campaign}/epoch{first.epoch}"
    print(f"\n{label}: {reason}")
    print(f"  {stamp(first.time)} .. {stamp(last.time)}; {len(samples):,} usable seconds; {len(set(s.run for s in samples)):,} continuous runs")
    print(f"  BME280 {min(x):.6f}..{max(x):.6f} C (span {max(x)-min(x):.6f}); LAP {min(y):.9f}..{max(y):.9f} ns")
    same = longest = 1
    repeated = 0
    for a,b in zip(samples,samples[1:]):
        if a.run == b.run and a.temperature == b.temperature:
            same += 1
            repeated += 1
            longest = max(longest,same)
        else:
            same = 1
    print(f"  Temperature snapshots: {len(set(x)):,} distinct values; {repeated:,} unchanged adjacent pairs; longest constant run {longest}s")
    points, omitted = blocks(samples,args.bin_seconds)
    print(f"  Analysis uses {len(points):,} complete {args.bin_seconds}s blocks; {omitted:,} seconds in partial blocks omitted.")
    model = association(points)
    if points:
        shape_report(points,model,args)
        holdout_report(points,args)
    return Series(label,campaign,first.signature,points,model)


def transfer_report(series: Sequence[Series]) -> None:
    print("\nCROSS-CAMPAIGN REPEATABILITY")
    print("  Fit each source epoch separately; predict another campaign without refitting its offset.")
    print("  Only matching recorded settings and temperatures inside the source's measured range are compared.")
    comparisons = 0
    for source in series:
        if source.model is None:
            continue
        low, high = min(p.temperature for p in source.points),max(p.temperature for p in source.points)
        for target in series:
            if target.campaign == source.campaign or source.signature != target.signature:
                continue
            selected = [p for p in target.points if low <= p.temperature <= high]
            if len(selected) < MIN_TEST:
                continue
            actual = [p.lap_ns for p in selected]
            prediction = [source.model.predict(p.temperature,0) for p in selected]
            rmse,bias,sd = prediction_errors(actual,prediction)
            baseline = prediction_errors(actual,[source.model.y0]*len(selected))[0]
            print(f"  {source.label} -> {target.label}: {len(selected)}/{len(target.points)} overlapping blocks")
            print(f"    RMSE {rmse*1000:.3f} ps; bias {bias*1000:+.3f} ps; residual SD {fmt(None if sd is None else sd*1000,3)} ps; source-mean RMSE {baseline*1000:.3f} ps")
            comparisons += 1
    if not comparisons:
        print("  No eligible comparison: supply multiple campaigns with matching settings and >=4 overlapping blocks.")
    print("  These use contemporaneous BME280 (zero lag). Epochs/campaigns are never pooled to manufacture a slope.")


def export_csv(path: Path, campaigns: Sequence[Tuple[str, Sequence[Sample]]]) -> None:
    with path.open("w",newline="",encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("campaign","row_id","published_at_utc","sequence","reset_count","update_count",
                         "recovery_generation","epoch","run","bme280_temperature_c","lap_mean_ns","accepted_races"))
        for campaign,samples in campaigns:
            for s in samples:
                writer.writerow((campaign,s.row_id,datetime.fromtimestamp(s.time,timezone.utc).isoformat(),
                                 s.sequence,s.reset,s.update,s.generation,s.epoch,s.run,s.temperature,s.lap_ns,s.races))


def arguments(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("campaigns",nargs="+",help="one or more exact campaign names; analyzed separately")
    parser.add_argument("--input",type=Path,help="read local PHOTONS JSON/JSONL instead of PostgreSQL")
    parser.add_argument("--bin-seconds",type=int,default=60,help="nonoverlapping block duration (default 60)")
    parser.add_argument("--temperature-bin-c",type=float,default=0.1,help="minimum temperature-bin width (default 0.1 C; enlarged for a compact table)")
    parser.add_argument("--direction-threshold-c-per-minute",type=float,default=0.002,help="warming/cooling deadband (default +/-0.002 C/min)")
    parser.add_argument("--max-lag-seconds",type=int,default=0,help="optional past-temperature lag search on training data (default 0)")
    parser.add_argument("--skip-seconds",type=int,default=0,help="omit this duration from each campaign's first stored publication")
    parser.add_argument("--start",type=utc,help="inclusive publication timestamp, with timezone")
    parser.add_argument("--end",type=utc,help="exclusive publication timestamp, with timezone")
    parser.add_argument("--batch-size",type=int,default=512)
    parser.add_argument("--csv",type=Path,help="export the used raw per-second pairs, with campaign/epoch/run IDs")
    args = parser.parse_args(argv[1:])
    if args.bin_seconds < 1 or args.batch_size < 1 or min(args.max_lag_seconds,args.skip_seconds) < 0:
        parser.error("bin/batch sizes must be positive and lag/skip durations nonnegative")
    if (not math.isfinite(args.temperature_bin_c) or args.temperature_bin_c <= 0 or
            not math.isfinite(args.direction_threshold_c_per_minute) or args.direction_threshold_c_per_minute < 0):
        parser.error("temperature-bin width must be positive and direction threshold nonnegative; both finite")
    if args.start is not None and args.end is not None and args.start >= args.end:
        parser.error("--start must precede --end")
    if len(set(args.campaigns)) != len(args.campaigns):
        parser.error("campaign names must be distinct")
    return args


def main(argv: Sequence[str]) -> int:
    args = arguments(argv)
    print("PHOTONS / BME280 RELATIONSHIP")
    print("  Same-message temperature vs accepted one-second mean lap; equal weight per populated second.")
    print("  Stored BME280 is a cached snapshot (normally ~30s or slower), not one fresh reading per second.")
    print("  Exact sensor age is not recorded; repeated values are reported, never counted as independent experiments.")
    print("  Continuity breaks distinguish counter gaps from publication timing; reasons may overlap and do not count crashes.")
    all_samples = []
    series = []
    empty = False
    for campaign in args.campaigns:
        selected_args = argparse.Namespace(**vars(args),campaign=campaign)
        rows = file_rows(args.input,campaign) if args.input else database_rows(selected_args)
        samples,counts,reasons = collect(rows,selected_args)
        print(f"\nCAMPAIGN {campaign}: " + "; ".join(f"{k}={v:,}" for k,v in counts.items()))
        if not samples:
            print("  No usable one-second pairs in this campaign/time range.")
            empty = True
            continue
        all_samples.append((campaign,samples))
        for epoch,reason in reasons.items():
            series.append(epoch_report(campaign,[s for s in samples if s.epoch == epoch],reason,args))
    if len(args.campaigns) > 1:
        transfer_report(series)
    if args.csv:
        export_csv(args.csv,all_samples)
        print(f"\nRaw paired observations exported: {args.csv}")
    print("\nREADING THE RESULT")
    print("  Look for consistent slopes, small same-temperature direction gaps, and low frozen prediction error.")
    print("  A high R2 from one warmup does not establish determinism; compare cooling and separate campaigns.")
    print("  SD here describes variation among time-block means, not individual-lap scatter or calibration uncertainty.")
    print("  Adjacent observations can be correlated: no independent-sample p-values or standard errors are reported.")
    print("  Unknown changes in hardware, clock speed or laser settings still require separate operator-selected runs.")
    print("  Analysis only: raw data, live masthead and statistical populations are unchanged.")
    return 1 if empty else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
