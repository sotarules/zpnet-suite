"""Infer BME280 temperature compensation for one PHOTONS campaign.

    zt photons_thermal Short2
    zt photons_thermal Short2 --max-lag-seconds 600
    python3 photons_thermal.py Short2 --input Short2.jsonl

Prints the complete report to STDOUT; no files or database values are changed.
Uses the same-message BME280 and one-second mean lap, never cumulative means.
Models: constant, linear, quadratic, cubic. Model selection uses forward-only
validation within the first 70%; the last 30% tests the frozen choice once.
The default uses current temperature only. Optional lag uses past temperatures
on complete, uninterrupted blocks. Positive lag means temperature BEFORE lap.

Standard library only, plus zpnet.shared.db when reading the database.
main(argv) includes the program name, as required by the zt runner.
"""
from __future__ import annotations

import argparse
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
VERSION = "2026-10-05.1"


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


def fmt(v: Optional[float], digits: int = 4) -> str:
    return "n/a" if v is None else f"{v:.{digits}f}"


@dataclass(frozen=True)
class Polynomial:
    reference: float
    coefficients: Tuple[float, ...]  # powers of (temperature - reference), ns/C^k
    low: float
    high: float

    def predict(self, temperature: float) -> float:
        value = 0.0
        for coefficient in reversed(self.coefficients):
            value = value * (temperature - self.reference) + coefficient
        return value

    def correction(self, temperature: float) -> float:
        return self.predict(temperature) - self.coefficients[0]


def polynomial(x, y, degree):
    """Small least squares fit, scaled and solved by reorthogonalized QR.

    A rank-deficient temperature population cannot identify this degree; that
    candidate is omitted explicitly rather than filled with guessed coefficients.
    """
    reference = st.fmean(x)
    scale = max(abs(t-reference) for t in x)
    if degree and scale < 1e-12:
        return None
    scale = scale or 1.0
    z = [(t-reference)/scale for t in x]
    columns = [[t**j for t in z] for j in range(degree+1)]
    q = []
    r = [[0.0]*(degree+1) for _ in range(degree+1)]
    for j, column in enumerate(columns):
        v = column[:]
        for _ in range(2):
            for i, basis in enumerate(q):
                projection = dot(basis, v)
                r[i][j] += projection
                v = [a-projection*b for a,b in zip(v,basis)]
        norm = math.sqrt(dot(v,v))
        if norm < 1e-10*math.sqrt(len(x)):
            return None
        r[j][j] = norm
        q.append([a/norm for a in v])
    y0 = st.fmean(y)
    rhs = [dot(basis, [v-y0 for v in y]) for basis in q]
    coefficients = [0.0]*(degree+1)
    for j in reversed(range(degree+1)):
        coefficients[j] = (rhs[j]-math.fsum(r[j][k]*coefficients[k]
                            for k in range(j+1,degree+1)))/r[j][j]
    coefficients[0] += y0
    coefficients = tuple(c/scale**j for j,c in enumerate(coefficients))
    return Polynomial(reference, coefficients, min(x), max(x))


def quantile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered)-1)*fraction
    i = int(position)
    return ordered[i] + (ordered[min(i+1,len(ordered)-1)]-ordered[i])*(position-i)


def rmse(errors):
    return math.sqrt(st.fmean(e*e for e in errors))


def reduction(before, after):
    return 'n/a' if before == 0 else f'{100*(1-after/before):+.2f}%'


def spread(label, values):
    sd = st.stdev(values) if len(values)>1 else 0.0
    print(f'  {label:25s} {len(values):8,d} {st.fmean(values):14.6f} '
          f'{sd*1000:12.3f} {(max(values)-min(values))*1000:12.3f} '
          f'{(quantile(values,.95)-quantile(values,.05))*1000:12.3f}')


def spread_header():
    print('  series                           N        mean_ns        SD_ps       p-p_ps     P95-P5_ps')


def name(degree, lag, width):
    return f'{("constant", "linear", "quadratic", "cubic")[degree]} / lag {lag*width}s'


def formula(model, lag_seconds):
    print(f'  T_ref = {model.reference:.9f} C; x = T_C - T_ref')
    print(f'  T_C is {"current block temperature" if not lag_seconds else str(lag_seconds)+" seconds earlier block temperature"}.')
    expression = f'{model.coefficients[0]:.12g}'
    for j,c in enumerate(model.coefficients[1:],1):
        expression += f' {c:+.12g}*x' + (f'^{j}' if j>1 else '')
    print(f'  predicted_LAP_ns = {expression}')
    correction = ' '.join(f'{c:+.12g}*x'+(f'^{j}' if j>1 else '')
                          for j,c in enumerate(model.coefficients[1:],1)) or '0'
    print(f'  normalized_LAP_ns = measured_LAP_ns - ({correction})')
    print(f'  Fitted temperature range: {model.low:.6f} .. {model.high:.6f} C')
    for j,c in enumerate(model.coefficients):
        print(f'    coefficient c{j} = {c:.12g} ns'+(f'/C^{j}' if j else ''))
    if len(model.coefficients)>1:
        print(f'  Sensitivity at T_ref: {model.coefficients[1]*1000:+.3f} ps/C')


def shape(points, x, residual, args):
    low, high = min(x), max(x)
    width = max(args.temperature_bin_c,
                args.temperature_bin_c*math.ceil((high-low)/(12*args.temperature_bin_c)))
    origin = math.floor(low/width)*width
    groups = {}
    for i,t in enumerate(x):
        groups.setdefault(math.floor((t-origin)/width),[]).append(i)
    print(f'\n  FINAL TEST TEMPERATURE SHAPE ({width:g} C bins; residual = actual - frozen prediction)')
    print('     mean_T_C    blocks     mean_LAP_ns    residual_ps   residual_SD_ps')
    for _,indices in sorted(groups.items()):
        e = [residual[i] for i in indices]
        print(f'  {st.fmean(x[i] for i in indices):11.5f} {len(indices):9d} '
              f'{st.fmean(points[i].lap_ns for i in indices):15.6f} '
              f'{st.fmean(e)*1000:14.3f} '
              f'{fmt(st.stdev(e)*1000 if len(e)>1 else None,3):>16}')
    direction = {}
    for i in range(1,len(points)):
        a,b = points[i-1],points[i]
        if a.run == b.run and b.index == a.index+1:
            rate = (x[i]-x[i-1])*60/(b.time-a.time)
            direction[i] = 'warm' if rate>args.direction_threshold_c_per_minute else (
                'cool' if rate < -args.direction_threshold_c_per_minute else 'steady')
    print('\n  FINAL TEST WARMING / COOLING (past-to-current predictor change)')
    print(f'  Direction counts: {dict(Counter(direction.values()))}')
    print('    mean_T_C   warm_N   cool_N   warm-minus-cool residual_ps')
    gaps = []
    for _,indices in sorted(groups.items()):
        warm = [residual[i] for i in indices if direction.get(i)=='warm']
        cool = [residual[i] for i in indices if direction.get(i)=='cool']
        if len(warm)>=2 and len(cool)>=2:
            gap = (st.fmean(warm)-st.fmean(cool))*1000
            gaps.append(gap)
            print(f'  {st.fmean(x[i] for i in indices):11.5f} {len(warm):8d} {len(cool):8d} {gap:32.3f}')
    if not gaps:
        print('  Insufficient overlapping warming/cooling blocks for a direction comparison.')
    else:
        print(f'  Largest absolute direction gap: {max(map(abs,gaps)):.3f} ps.')
    print('  Persistent residual shape or direction gaps suggest temperature alone is incomplete.')


def analyze(samples, reason, args):
    print(f'\nEPOCH {samples[0].epoch}: {reason}')
    print(f'  UTC {stamp(samples[0].time)} .. {stamp(samples[-1].time)}')
    print(f'  Settings: {samples[0].signature}')
    print(f'  Usable seconds: {len(samples):,}; continuous runs: {len(set(s.run for s in samples)):,}')
    temperatures = [s.temperature for s in samples]
    print(f'  BME280 range: {min(temperatures):.6f} .. {max(temperatures):.6f} C')
    repeated = sum(a.run==b.run and a.temperature==b.temperature for a,b in zip(samples,samples[1:]))
    print(f'  Adjacent identical BME280 readings: {repeated:,} (cached snapshots are not new independent readings).')
    spread_header()
    spread('all raw one-second means', [s.lap_ns for s in samples])
    print(f'  Raw lap range: {min(s.lap_ns for s in samples):.9f} .. {max(s.lap_ns for s in samples):.9f} ns')
    points, omitted = blocks(samples,args.bin_seconds)
    print(f'  Complete {args.bin_seconds}-second blocks: {len(points):,}; seconds in incomplete blocks: {omitted:,}')
    if len(points)<60:
        print('  Need at least 60 complete blocks for model selection plus a later test.')
        print('  Use a smaller --bin-seconds if continuity breaks leave too few complete blocks.')
        return
    print(f'  Descriptive block correlations: T vs LAP {fmt(pearson([p.temperature for p in points], [p.lap_ns for p in points]))}; '
          f'T vs time {fmt(pearson([p.temperature for p in points], [p.time for p in points]))}')
    lookup = {(p.run,p.index):p for p in points}
    steps = args.max_lag_seconds//args.bin_seconds
    targets = [p for p in points if p.index>=steps]
    print(f'  Common target blocks for every candidate lag: {len(targets):,}/{len(points):,}')
    if len(targets)<60:
        print('  Too few blocks with complete lag history; reduce --max-lag-seconds or collect longer runs.')
        return
    cut = 7*len(targets)//10
    development = targets[:cut]
    # Time embargo prevents blocks/history from touching the preceding fit.
    embargo = (steps+1)*args.bin_seconds
    test = [p for p in targets[cut:] if p.time-development[-1].time>embargo]
    folds = []
    for fraction in (.4,.6,.8):
        end = int(len(development)*fraction)
        stop = int(len(development)*(fraction+.2)+1e-8)
        train = development[:end]
        validation = [p for p in development[end:stop] if p.time-train[-1].time>embargo]
        if len(train)<10 or len(validation)<4:
            print('  Not enough forward-validation support after separation; shorten blocks or lag range.')
            return
        folds.append((train,validation))
    if len(test)<8:
        print('  Need at least 8 final test blocks after separation; reduce the block/lag duration.')
        return
    def xs(rows,lag):
        return [lookup[(p.run,p.index-lag)].temperature for p in rows]
    def ys(rows):
        return [p.lap_ns for p in rows]
    print('\n  MODEL SELECTION (first 70% only; three expanding forward folds)')
    print(f'  Separation: >{embargo}s; final test: {len(test):,} blocks, not used to choose degree or lag.')
    for i,(train,val) in enumerate(folds,1):
        print(f'  Fold {i}: fit {len(train):,}, validate {len(val):,}; validation {stamp(val[0].time)} .. {stamp(val[-1].time)}')
    scored = []
    for degree in range(args.max_degree+1):
        for lag in (range(steps+1) if degree else [0]):
            errors = []
            fold_scores = []
            for train,val in folds:
                model = polynomial(xs(train,lag),ys(train),degree)
                if model is None:
                    break
                e = [p.lap_ns-model.predict(t) for p,t in zip(val,xs(val,lag))]
                errors.extend(e)
                fold_scores.append(rmse(e))
            if len(fold_scores)==3:
                scored.append((degree,lag,rmse(errors),fold_scores))
    print('  Best lag per degree; scores are prediction RMSE in ps:')
    print('    candidate                    pooled       fold1       fold2       fold3')
    best = []
    for degree in range(args.max_degree+1):
        entries = [entry for entry in scored if entry[0]==degree]
        if not entries:
            print(f'    degree {degree}: unidentifiable temperature population')
            continue
        winner = min(entries,key=lambda e:(e[2],e[1]))
        best.append(winner)
        print(f'    {name(*winner[:2],args.bin_seconds):27s} '+ ' '.join(f'{v*1000:11.3f}' for v in [winner[2],*winner[3]]))
    optimum = min(e[2] for e in scored)
    # The tolerance is a declared engineering simplicity rule, not a confidence interval.
    eligible = [e for e in scored if e[2]<=optimum*(1+args.simplicity_pct/100)+1e-12]
    chosen = min(eligible,key=lambda e:(e[0],e[1]))
    degree,lag,_,_ = chosen
    print(f'  Choose simplest degree, then shortest lag, within {args.simplicity_pct:g}% of best CV RMSE.')
    print(f'  FROZEN CHOICE: {name(degree,lag,args.bin_seconds)}')
    model = polynomial(xs(development,lag),ys(development),degree)
    assert model is not None
    print('\n  FROZEN FORMULA (fitted on development only; no later-data recentering)')
    formula(model,lag*args.bin_seconds)
    tx, actual = xs(test,lag),ys(test)
    predictions = [model.predict(t) for t in tx]
    residual = [a-b for a,b in zip(actual,predictions)]
    normalized = [a-model.correction(t) for a,t in zip(actual,tx)]
    baseline = st.fmean(ys(development))
    baseline_errors = [y-baseline for y in actual]
    trend = polynomial([p.time for p in development],ys(development),1)
    print('\n  FINAL UNSEEN TEST (same blocks in every comparison)')
    print(f'  UTC {stamp(test[0].time)} .. {stamp(test[-1].time)}')
    outside = [i for i,t in enumerate(tx) if not model.low<=t<=model.high]
    print(f'  Predictor range: {min(tx):.6f} .. {max(tx):.6f} C; outside fit range: {len(outside):,}/{len(test):,}')
    print('    prediction                       RMSE_ps        bias_ps     error_SD_ps')
    comparisons = [('constant development mean',baseline_errors),('selected temperature formula',residual)]
    if trend is not None:
        comparisons.append(('elapsed-time line (diagnostic)',[p.lap_ns-trend.predict(p.time) for p in test]))
    for label,e in comparisons:
        print(f'    {label:31s} {rmse(e)*1000:12.3f} {st.fmean(e)*1000:14.3f} {st.stdev(e)*1000:15.3f}')
    for label, indices in [('inside fit range',[i for i in range(len(tx)) if model.low<=tx[i]<=model.high]),
                           ('outside fit range',outside)]:
        if indices:
            print(f'  {label}: {len(indices):,} blocks; formula RMSE {rmse([residual[i] for i in indices])*1000:.3f} ps; '
                  f'constant RMSE {rmse([baseline_errors[i] for i in indices])*1000:.3f} ps')
    spread_header()
    spread('raw test block means',actual)
    spread('normalized test blocks',normalized)
    print(f'  Test RMSE reduction vs constant: {reduction(rmse(baseline_errors),rmse(residual))}')
    print(f'  Test SD reduction: {reduction(st.stdev(actual),st.stdev(normalized))}')
    print(f'  Test peak-to-peak reduction: {reduction(max(actual)-min(actual),max(normalized)-min(normalized))}')
    print(f'  Residual correlation with T: {fmt(pearson(tx,residual))}; '
          f'with elapsed time: {fmt(pearson([p.time for p in test],residual))}')
    # Report actual one-second correction, separate from block-level fit performance.
    # A block contains exactly width consecutive retained fragments in a run.
    if lag==0:
        selected = {(p.run,p.index) for p in test}
        raw_seconds, corrected_seconds = [],[]
        ordinal = Counter()
        for s in samples:
            index = ordinal[s.run]//args.bin_seconds
            ordinal[s.run] += 1
            if (s.run,index) in selected:
                raw_seconds.append(s.lap_ns)
                corrected_seconds.append(s.lap_ns-model.correction(s.temperature))
        print('\n  SAME FINAL TEST, AT ONE-SECOND RESOLUTION (cached recorded temperature)')
        spread_header()
        spread('raw one-second means',raw_seconds)
        spread('normalized one-second',corrected_seconds)
        print('  Polynomial evaluated per second here; nonlinear averaging can differ from block evaluation.')
    else:
        print('  Lagged correction is evaluated at block resolution only; no interpolated one-second results.')
    print('\n  FINAL TEST IN SIX CHRONOLOGICAL WINDOWS')
    print('    UTC start                    N    mean_T_C  raw_mean_ns  normalized_mean_ns  residual_mean_ps')
    for i in range(6):
        indices = list(range(i*len(test)//6,(i+1)*len(test)//6))
        if indices:
            print(f'    {stamp(test[indices[0]].time):25s} {len(indices):5d} '
                  f'{st.fmean(tx[j] for j in indices):11.5f} '
                  f'{st.fmean(actual[j] for j in indices):12.6f} '
                  f'{st.fmean(normalized[j] for j in indices):19.6f} '
                  f'{st.fmean(residual[j] for j in indices)*1000:17.3f}')
    shape(test,tx,residual,args)
    print('\n  INTERPRETATION')
    if degree==0:
        print('  Forward validation did not justify a temperature correction under the simplicity rule.')
    elif rmse(residual)<rmse(baseline_errors) and st.stdev(normalized)<st.stdev(actual):
        print('  The frozen temperature formula improved both later prediction and later stability.')
        print('  This is a candidate calibration for this setup and measured range, not yet a transfer test.')
    else:
        print('  The frozen formula did not improve both prediction and stability on later data.')
        print('  Do not adopt it as an established correction; the residual tables identify what remains.')
    if trend is not None and rmse(residual)>=rmse(comparisons[-1][1]):
        print('  Temperature did not beat a simple elapsed-time line in this test; shared drift remains plausible.')
    if outside:
        print('  Some test temperatures are extrapolations; inspect inside-range results separately.')
    if lag and lag==steps:
        print('  Selected lag reaches the search boundary; a longer lag has not been evaluated.')
    if degree==args.max_degree and degree>1:
        print('  Selected curvature reaches the allowed degree; this alone is not a reason to increase it.')
    print('  Full-data refitting is intentionally deferred so this displayed formula retains an honest test.')


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('campaign',help='exact campaign name, e.g. Short2')
    parser.add_argument('--input',type=Path,help='PHOTONS JSON/JSONL instead of PostgreSQL')
    parser.add_argument('--bin-seconds',type=int,default=60,help='complete nonoverlapping blocks (default 60)')
    parser.add_argument('--max-degree',type=int,choices=(1,2,3),default=3,help='maximum polynomial degree (default 3)')
    parser.add_argument('--max-lag-seconds',type=int,default=0,help='optional past-temperature search (default 0; multiple of block width)')
    parser.add_argument('--simplicity-pct',type=float,default=5,help='prefer simpler models within this percent of best validation RMSE (default 5)')
    parser.add_argument('--temperature-bin-c',type=float,default=.1,help='minimum report temperature-bin width (default .1)')
    parser.add_argument('--direction-threshold-c-per-minute',type=float,default=.002)
    parser.add_argument('--skip-seconds',type=int,default=0,help='omit initial campaign seconds (default 0)')
    parser.add_argument('--start',type=utc,help='inclusive UTC/offset timestamp')
    parser.add_argument('--end',type=utc,help='exclusive UTC/offset timestamp')
    parser.add_argument('--batch-size',type=int,default=512)
    args = parser.parse_args(argv[1:])
    if min(args.bin_seconds,args.batch_size)<1 or min(args.max_lag_seconds,args.skip_seconds)<0:
        parser.error('block/batch size must be positive; lag/skip must be nonnegative')
    if args.max_lag_seconds%args.bin_seconds:
        parser.error('--max-lag-seconds must be a multiple of --bin-seconds')
    if any(not math.isfinite(v) or v<0 for v in
           (args.simplicity_pct,args.direction_threshold_c_per_minute)) or not math.isfinite(args.temperature_bin_c) or args.temperature_bin_c<=0:
        parser.error('finite nonnegative tolerances and positive temperature-bin width required')
    if args.start is not None and args.end is not None and args.start>=args.end:
        parser.error('--start must precede --end')
    return args


def main(argv):
    args = arguments(argv)
    print(f'PHOTONS THERMAL NORMALIZATION REPORT {VERSION}',flush=True)
    print(f'Campaign: {args.campaign}; generated UTC: {datetime.now(timezone.utc).isoformat(timespec="seconds")}')
    print(f'Options: block={args.bin_seconds}s, max_degree={args.max_degree}, max_lag={args.max_lag_seconds}s, simplicity={args.simplicity_pct:g}%')
    print(f'Selection: start={args.start}, end={args.end}, skip={args.skip_seconds}s; source={args.input or "read-only database"}')
    print('Predictor: environment.temperature_c (BME280); target: photons.race.flight_ns.mean (ns).')
    print('Equal weight per usable second; no cumulative means; no rejection based on lap magnitude.')
    print('Loading campaign observations...',flush=True)
    rows = file_rows(args.input,args.campaign) if args.input else database_rows(args)
    samples,counts,reasons = collect(rows,args)
    print('\nDATA ACCOUNTING')
    for key,value in counts.items():
        print(f'  {key}: {value:,}')
    print('  Continuity-break reason counts can overlap. Partial blocks and lag-history losses are reported per epoch.')
    if not samples:
        print('No usable temperature/lap pairs. Report the accounting above before changing any filters.')
        return 1
    epochs = {}
    for sample in samples:
        epochs.setdefault(sample.epoch,[]).append(sample)
    for epoch,observations in epochs.items():
        analyze(observations,reasons[epoch],args)
    print('\nREPORT NOTES')
    print('  Epochs are fitted separately across recorded settings changes, resets and recovery.')
    print('  Blocks and lag history never cross continuity gaps. Unknown hardware changes need explicit time selection.')
    print('  All printed SDs describe one-second or block means, not individual-photon jitter or calibration uncertainty.')
    print('  Time-correlated observations and cached temperatures are not independent thermal experiments.')
    print('  No p-values or independent-sample confidence claims; validation tests prediction, not causation.')
    print('  Repeatedly tuning against this final test makes it exploratory; confirm revisions on newly collected data.')
    print('  No database writes, live compensation, stored measurements, or additional output files.')
    print('END PHOTONS THERMAL REPORT')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
