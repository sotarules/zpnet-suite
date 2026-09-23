"""Explore BME280 temperature versus PHOTONS one-second mean flight time.

Install beside photons_analyzer.py in the existing ZPNet test directory:
    zt photons_thermal Pelican1
    zt photons_thermal Pelican1 --bin-seconds 120 --max-lag-seconds 3600
    zt photons_thermal Pelican1 --csv /tmp/Pelican1-thermal.csv

The main(argv) convention matches photons_analyzer: argv includes the program
name. Only the Python standard library and the existing zpnet.shared.db helper
are needed. Database access is read-only, restricted to this exact campaign.
Offline PHOTONS JSON, JSON arrays, or JSONL (optionally prefixed PHOTONS) can be
read with --input PATH. This is analysis only; no corrections are installed.

Measurement: photons.race.flight_ns.mean, NOT campaign.stats.mean_lap_ns,
photons.stats.mean_lap_ns, or the diagnostic lower envelope. Every populated
one-second fragment has equal weight regardless of its accepted race count.
Thermal blocks are nonoverlapping averages of consecutive one-second records.
Incomplete blocks are reported and omitted; gaps are never interpolated.
Lag +N compares temperature N seconds EARLIER with the current LAP. Negative
lags use later temperature and are diagnostic only, never chosen for holdout.

The lag scan compares the SAME target blocks at every tested lag. A temperature
model is selected using only the first two thirds of that common support and
evaluated on the last third, with one block withheld between them. Its
coefficients and lag are frozen for evaluation. Constant and elapsed-time-only
models are evaluated on the identical holdout. Temperature + time and
detrended/differenced associations help expose a shared monotonic drift.
These checks do not establish a thermal cause or eliminate nonlinear drift.

No IID standard errors or p-values are reported: adjacent seconds/blocks may
be strongly correlated. A single holdout is exploratory, not independent
calibration. For background on dependence in measurement data, see NIST:
https://www.nist.gov/programs-projects/uncertainty-analysis-autocorrelated-measurement-data

Epochs separate reset/reboot evidence and changes to recorded acquisition/core
settings; detected missing fragments break runs within an epoch. Unknown
hardware/firmware changes cannot be inferred. Keep settings fixed and use
--start/--end (timezone required) to select a known stable interval if needed.
Timing uses published_at_utc for selection/display and fragment sequence for
lag alignment; fragment_period_ns must be one second. BME280 is the snapshot
attached to each publication, not a proven synchronous per-race reading.
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
MIN_HOLDOUT = 12


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


def optional_number(value: Any) -> Optional[float]:
    return None if value is None else number(value)


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
    core_state: str
    input_ns: Optional[float]
    cycles: Optional[float]
    dwt_hz: Optional[float]
    epoch: int = 0
    run: int = 0


def parse_sample(row_id: int, p: Dict[str, Any]) -> Tuple[Optional[Sample], str]:
    """Missing/malformed required schema is an integrity error, not imputed data."""
    if p["schema"] != SCHEMA:
        raise ValueError(f"expected {SCHEMA}")
    ph = p["photons"]
    if ph["fragment_period_ns"] != 1_000_000_000:
        raise ValueError("thermal analysis requires one-second fragments")
    if ph["snapshot_ok"] is not True:
        return None, "snapshot_not_ok"
    env = p["environment"]
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
            "schema", "cadence_ns", "pulse_ns", "capture_min_ns", "capture_max_ns",
            "launch_bracket_max_cycles", "flight_interpretation", "launch_timing_policy")},
        "core": {k: core.get(k) for k in (
            "algorithm", "minimum_count", "minimum_radius_cycles", "mad_multiplier", "minimum_retained_pct")},
    }, sort_keys=True)
    projection = ph.get("projection") or {}
    pps = projection.get("anchor_pps_count")
    return Sample(
        row_id=row_id, time=utc(p["published_at_utc"]), sequence=integer(p["sequence"]),
        reset=integer(ph["stats"]["reset_count"]), pps=None if pps is None else integer(pps),
        temperature=number(env["temperature_c"]), lap_ns=number(flight["mean"]), races=n,
        signature=signature, core_state=str(core.get("state", "NOT_RECORDED")),
        input_ns=optional_number(core.get("input_mean_ns")),
        cycles=optional_number(core.get("retained_mean_cycles")),
        dwt_hz=optional_number(core.get("dwt_cycles_per_second")),
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
            'photons', jsonb_build_object(
                'snapshot_ok', payload #> '{photons,snapshot_ok}',
                'fragment_period_ns', payload #> '{photons,fragment_period_ns}',
                'stats', jsonb_build_object('reset_count', payload #> '{photons,stats,reset_count}'),
                'projection', jsonb_build_object('anchor_pps_count', payload #> '{photons,projection,anchor_pps_count}'),
                'core', payload #> '{photons,core}',
                'race', jsonb_build_object(
                    'active', payload #> '{photons,race,active}',
                    'flight_ns', payload #> '{photons,race,flight_ns}',
                    'schema', payload #> '{photons,race,schema}',
                    'cadence_ns', payload #> '{photons,race,cadence_ns}',
                    'pulse_ns', payload #> '{photons,race,pulse_ns}',
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
        if p.get("schema") == SCHEMA and p["campaign"]["campaign"] == campaign:
            yield i, p


def collect(rows: Iterable[Tuple[int, Dict[str, Any]]], args: argparse.Namespace):
    samples: List[Sample] = []
    counts: Counter = Counter()
    reasons: Dict[int, str] = {}
    prev: Optional[Sample] = None
    seen: Dict[Tuple[int, int, Optional[int]], Sample] = {}
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
            continue
        key = (sample.reset, sample.sequence, sample.pps)
        if key in seen:
            old = seen[key]
            # Final campaign snapshots may republish the same physical fragment.
            # The original sensor pairing is retained; a later snapshot adds no data.
            if (sample.lap_ns, sample.races, sample.signature) != (old.lap_ns, old.races, old.signature):
                raise ValueError(f"row {row_id}: repeated fragment has conflicting science/configuration")
            counts["duplicate_fragment"] += 1
            continue
        seen[key] = sample
        changes = []
        if prev is not None:
            if sample.reset != prev.reset:
                changes.append("statistics reset")
            if sample.signature != prev.signature:
                changes.append("acquisition/core settings changed")
            if sample.sequence <= prev.sequence or (sample.pps is not None and prev.pps is not None and sample.pps < prev.pps):
                changes.append("sequence/PPS restart")
            if sample.time <= prev.time:
                changes.append("publication time moved backward")
        if prev is None or changes:
            epoch += 1
            run += 1
            reasons[epoch] = "; ".join(changes) or "start of selected observations"
        elif sample.sequence != prev.sequence + 1 or not 0.25 <= sample.time - prev.time <= 1.75:
            run += 1
            counts["continuity_breaks"] += 1
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


def describe(name: str, x: Sequence[float], y: Sequence[float], times: Sequence[float]) -> None:
    model = fit(x, y)
    print(f"  {name}: N={len(x):,}; r={fmt(pearson(x, y))}")
    if model is None:
        print("    Slope unavailable: insufficient points or no temperature variation.")
        return
    print(f"    Slope={model.beta * 1000:+.3f} ps/C; R2={model.r2:.4f}; "
          f"LAP SD={st.stdev(y)*1000:.3f} ps; fitted residual SD={model.residual_sd_ns*1000:.3f} ps")
    print(f"    Temperature-implied change over observed range={model.beta * (max(x)-min(x))*1000:+.3f} ps")
    controlled = fit(x, y, times)
    print(f"    After removing linear time trends: r={fmt(detrended_r(x, y, times))}; "
          f"temperature slope={fmt(None if controlled is None else controlled.beta*1000, 3)} ps/C")


def lag_support(points: Sequence[Point], requested_steps: int):
    lengths = Counter(p.run for p in points)
    # Keep at least 12 target blocks in the longest run; print any reduced range.
    steps = min(requested_steps, max(0, (max(lengths.values()) - MIN_HOLDOUT) // 2))
    lookup = {(p.run, p.index): p for p in points}
    targets = [p for p in points if p.index >= steps and p.index + steps < lengths[p.run]]
    return steps, lookup, targets


def paired(targets: Sequence[Point], lookup, lag: int):
    return ([lookup[(p.run, p.index-lag)].temperature for p in targets],
            [p.lap_ns for p in targets], [p.time for p in targets])


def lag_report(points: Sequence[Point], width: int, max_lag: int) -> None:
    if len(points) < MIN_HOLDOUT:
        print(f"  Lag/holdout analysis needs at least {MIN_HOLDOUT} complete blocks.")
        return
    steps, lookup, targets = lag_support(points, max_lag // width)
    print(f"\n  Lag scan: +/-{steps*width}s in {width}s steps; identical {len(targets):,} target blocks per lag.")
    print("    Positive lag = temperature earlier than LAP; negative lag = temperature later.")
    if steps*width < max_lag:
        print("    Range rounded/reduced to available contiguous data and block resolution.")
    scored = []
    for lag in range(-steps, steps+1):
        x, y, t = paired(targets, lookup, lag)
        model = fit(x, y)
        if model is not None:
            scored.append((lag, model, pearson(x, y), detrended_r(x, y, t)))
    if not scored:
        print("    No identifiable temperature slope on common lag support.")
        return
    top = sorted(scored, key=lambda item: (-item[1].r2, abs(item[0]), item[0]))[:5]
    zero = next((s for s in scored if s[0] == 0), None)
    if zero is not None and zero not in top:
        top.append(zero)
    print("    lag_s      r       detrended_r     slope_ps/C      R2")
    for lag, model, r, rd in top:
        print(f"    {lag*width:+6d}  {fmt(r):>8}  {fmt(rd):>12}  {model.beta*1000:>13.3f}  {model.r2:>7.4f}")
    best = top[0][0]
    if steps and abs(best) == steps:
        print("    Highest |r| reaches the scan boundary; the optimum is not bracketed.")
    if best < 0:
        print("    Strongest full-sample association uses later temperature; inspect sensor lag/common drift.")

    cut = 2 * len(targets) // 3
    train, test = targets[:cut], targets[cut+1:]
    if len(train) < 8 or len(test) < 4:
        print("    Too little common support for an 8-block train / 4-block holdout after the gap.")
        return
    candidates = []
    for lag in range(steps+1):
        x, y, _ = paired(train, lookup, lag)
        model = fit(x, y)
        if model is not None:
            candidates.append((lag, model))
    if not candidates:
        print("    Training portion has no temperature variation; holdout model unavailable.")
        return
    chosen, model = min(candidates, key=lambda item: (-item[1].r2, item[0]))
    x, y, t = paired(train, lookup, chosen)
    tx, ty, tt = paired(test, lookup, chosen)
    trend = fit(t, y)
    combined = fit(x, y, t)
    models = [("training mean", [st.fmean(y)] * len(test)),
              ("temperature", [model.predict(a, b) for a, b in zip(tx, tt)])]
    if trend:
        models.insert(1, ("elapsed time only", [trend.predict(b, 0) for b in tt]))
    if combined:
        models.append(("temperature + time", [combined.predict(a, b) for a, b in zip(tx, tt)]))
    print(f"\n  Frozen later-data check: train {len(train):,} blocks, holdout {len(test):,}, one block gap.")
    print(f"    Train targets:   {stamp(train[0].time)} .. {stamp(train[-1].time)}")
    print(f"    Holdout targets: {stamp(test[0].time)} .. {stamp(test[-1].time)}")
    print(f"    Training-selected lag=+{chosen*width}s; slope={model.beta*1000:+.3f} ps/C.")
    print(f"    Prediction: LAP_ns={model.y0:.9f} + ({model.beta:+.9f})*(T_C-{model.x0:.6f})")
    print(f"    Predictor temperature: train {min(x):.3f}..{max(x):.3f} C; holdout {min(tx):.3f}..{max(tx):.3f} C")
    train_low, train_high = min(x), max(x)
    outside = sum(v < train_low or v > train_high for v in tx)
    print(f"    Holdout predictors outside training temperature range: {outside}/{len(tx)}")
    print("    model                    RMSE_ps       bias_ps     residual_SD_ps")
    errors = {}
    for name, predicted in models:
        residual = [actual-estimated for actual, estimated in zip(ty, predicted)]
        rmse = math.sqrt(st.fmean(v*v for v in residual))
        errors[name] = rmse
        print(f"    {name:<24} {rmse*1000:>10.3f}  {st.fmean(residual)*1000:>12.3f}  {st.stdev(residual)*1000:>17.3f}")
    if errors["training mean"] > 0:
        print(f"    Temperature RMSE change versus training mean: {(errors['temperature']/errors['training mean']-1)*100:+.2f}% (negative improves).")
    held_fit = fit(tx, ty)
    print(f"    Separately refitted holdout slope: {fmt(None if held_fit is None else held_fit.beta*1000, 3)} ps/C")
    if chosen == steps and steps:
        print("    Selected causal lag is at the search limit; consider a wider scan.")
    if "elapsed time only" in errors and errors["temperature"] >= errors["elapsed time only"]:
        print("    Temperature alone did not outperform elapsed-time drift on this holdout.")
    print("    Full-sample lag ranking above was NOT used to choose this prediction model.")


def epoch_report(samples: Sequence[Sample], reason: str, args: argparse.Namespace) -> None:
    first, last = samples[0], samples[-1]
    x = [s.temperature for s in samples]
    y = [s.lap_ns for s in samples]
    t = [s.time-first.time for s in samples]
    print(f"\nEPOCH {first.epoch}: {reason}")
    print(f"  {stamp(first.time)} .. {stamp(last.time)}; {len(samples):,} usable seconds; {len(set(s.run for s in samples)):,} continuous runs")
    print(f"  BME280: {min(x):.3f}..{max(x):.3f} C; span={max(x)-min(x):.3f} C; {len(set(x)):,} distinct reported values")
    print(f"  LAP: {min(y):.9f}..{max(y):.9f} ns; mean={st.fmean(y):.9f} ns")
    print(f"  Core states: {dict(Counter(s.core_state for s in samples))}")
    print(f"  Recorded configuration: {first.signature}")
    describe("One-second observations", x, y, t)
    points, omitted = blocks(samples, args.bin_seconds)
    print(f"\n  {args.bin_seconds}s thermal blocks: {len(points):,} complete; {omitted:,} seconds in partial blocks omitted")
    if len(points) < MIN_FIT:
        print("  Insufficient complete blocks; collect longer or reduce --bin-seconds.")
        return
    bx, by = [p.temperature for p in points], [p.lap_ns for p in points]
    bt = [p.time-points[0].time for p in points]
    describe("Block means", bx, by, bt)
    adjacent = [(a, b) for a, b in zip(points, points[1:]) if a.run == b.run and b.index == a.index+1]
    dx, dy = [b.temperature-a.temperature for a, b in adjacent], [b.lap_ns-a.lap_ns for a, b in adjacent]
    print(f"  Adjacent-block changes: N={len(adjacent):,}; corr(delta T, delta LAP)={fmt(pearson(dx, dy))}")
    print(f"  Adjacent-block LAP correlation={fmt(pearson([a.lap_ns for a,b in adjacent], [b.lap_ns for a,b in adjacent]))}")
    print(f"  Adjacent-block temperature correlation={fmt(pearson([a.temperature for a,b in adjacent], [b.temperature for a,b in adjacent]))}")
    if max(bx) > min(bx):
        print("  Temperature bins (block means, descriptive; not independent repetitions):")
        span = max(bx)-min(bx)
        for i in range(6):
            selected = [p for p in points if min(5, int((p.temperature-min(bx))/span*6)) == i]
            if selected:
                temps, laps = [p.temperature for p in selected], [p.lap_ns for p in selected]
                sd = st.stdev(laps)*1000 if len(laps)>1 else None
                print(f"    T={st.fmean(temps):.4f} C; blocks={len(laps):5d}; LAP={st.fmean(laps):.9f} ns; SD={fmt(sd,3)} ps")
    lag_report(points, args.bin_seconds, args.max_lag_seconds)


def export_csv(path: Path, samples: Sequence[Sample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("row_id", "published_at_utc", "sequence", "reset_count", "epoch", "run",
                         "temperature_c", "lap_mean_ns", "accepted_races", "core_state",
                         "core_input_mean_ns", "core_retained_mean_cycles", "dwt_cycles_per_second"))
        for s in samples:
            writer.writerow((s.row_id, datetime.fromtimestamp(s.time, timezone.utc).isoformat(), s.sequence,
                             s.reset, s.epoch, s.run, s.temperature, s.lap_ns, s.races, s.core_state,
                             s.input_ns, s.cycles, s.dwt_hz))


def arguments(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("campaign")
    parser.add_argument("--input", type=Path, help="read local PHOTONS JSON/JSONL instead of PostgreSQL")
    parser.add_argument("--bin-seconds", type=int, default=60, help="nonoverlapping thermal block duration (default 60)")
    parser.add_argument("--max-lag-seconds", type=int, default=1800, help="scan both lag directions up to this duration (default 1800)")
    parser.add_argument("--skip-seconds", type=int, default=0, help="omit this duration from campaign's first stored publication")
    parser.add_argument("--start", type=utc, help="inclusive publication timestamp, e.g. 2026-09-23T08:00:00Z")
    parser.add_argument("--end", type=utc, help="exclusive publication timestamp; timezone required")
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--csv", type=Path, help="export all used one-second pairs and epoch/run IDs")
    args = parser.parse_args(argv[1:])
    if args.bin_seconds < 1 or args.batch_size < 1 or min(args.max_lag_seconds, args.skip_seconds) < 0:
        parser.error("bin/batch sizes must be positive and lag/skip durations nonnegative")
    if args.start is not None and args.end is not None and args.start >= args.end:
        parser.error("--start must precede --end")
    return args


def main(argv: Sequence[str]) -> int:
    args = arguments(argv)
    source = file_rows(args.input, args.campaign) if args.input else database_rows(args)
    samples, counts, reasons = collect(source, args)
    print(f"PHOTONS THERMAL ANALYSIS — {args.campaign}")
    print("  Source: environment.temperature_c vs photons.race.flight_ns.mean")
    print("  Each populated second has equal weight; cumulative/campaign means are not inputs.")
    print("  Counts: " + "; ".join(f"{k}={v:,}" for k, v in counts.items()))
    if not samples:
        print("  No usable one-second pairs in the selected campaign/time interval.")
        return 1
    for epoch in reasons:
        epoch_report([s for s in samples if s.epoch == epoch], reasons[epoch], args)
    if args.csv:
        export_csv(args.csv, samples)
        print(f"\n  One-second pairs exported: {args.csv}")
    print("\nINTERPRETATION")
    print("  ps/C is a fitted association. R2/residual SD describe this data, not causal proof or uncertainty.")
    print("  No independent-sample p-values/SEs: successive records may be correlated.")
    print("  BME280 snapshots may repeat between polls; sensor-to-component thermal lag is not known.")
    print("  The holdout tests transfer within this epoch; a separate stable-location campaign is the next check.")
    print("  No database, firmware, LAP, or campaign values were changed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
