"""Estimate the OCXO tuning slopes from isolated DAC steps in a TEMPEST campaign."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from itertools import islice
from statistics import median
from time import sleep
from typing import Iterator


LANES = ("ocxo1", "ocxo2")
APPLIED = "Pi-timed DAC exposure"
LEGACY = "legacy requested DAC targets"
SERVO_MODES = {"10-MIN", "TOTAL", "CAMP"}
BILLION = 1_000_000_000
VOLTS_PER_CODE = Decimal(5) / 65536
UV_PER_CODE = VOLTS_PER_CODE * 1_000_000
DEFAULT_STEP_CODES = Decimal("0.25")
DITHER_CODE_RESOLUTION = Decimal("0.001")


@dataclass(frozen=True)
class Sample:
    sequence: int
    campaign_count: int
    epoch: tuple[str, str]
    code: Decimal
    reference_cycles: int
    ocxo_cycles: int


@dataclass(frozen=True)
class Step:
    first_pps: int
    last_pps: int
    delta_code: Decimal
    delta_ppb: Decimal

    @property
    def slope(self) -> Decimal:
        return self.delta_ppb / self.delta_code


def campaign_rows(
    campaign: str, after_id: int = 0, *, witness_span: tuple[int, int] | None = None,
) -> Iterator[dict]:
    from zpnet.shared.db import open_db

    first_pps, last_pps = witness_span or (None, None)
    # Read only physical period evidence and actuator history. Published
    # synthetic PPB already contains a slope and would make the fit circular.
    with open_db(row_dict=True) as db:
        db.execute("SET TRANSACTION READ ONLY")
        db.execute("SET LOCAL statement_timeout = '30s'")
        with db.cursor(name="clocks_voltage_calibration") as cur:
            cur.itersize = 512
            cur.execute(
                """
                SELECT id, sequence, campaign,
                       payload #>> '{campaign,public_count}' AS campaign_count,
                       payload #>> '{clocks,stats,reset_count}' AS stats_epoch,
                       payload #> '{clocks,raw_cycles}' AS raw_cycles,
                       (payload -> 'clocks') ? 'dac_exposure' AS exposure_recorded,
                       payload #> '{clocks,dac_exposure}' AS exposure,
                       payload #> '{clocks,dac_observation}' AS observation,
                       CASE WHEN %s THEN jsonb_build_object(
                           'ocxo1', payload #> '{campaign,ocxo1,science}',
                           'ocxo2', payload #> '{campaign,ocxo2,science}'
                       ) END AS pps_witness
                FROM campaign_detail
                WHERE campaign_type = 'TEMPEST'
                  AND campaign = %s
                  AND id > %s
                  AND viable IS TRUE
                  AND payload #>> '{schema}' = 'CLOCKS_V4'
                  AND payload #>> '{campaign,schema}' = 'TEMPEST_FRAGMENT_V1'
                  AND (%s::bigint IS NULL
                       OR (payload #>> '{campaign,public_count}')::bigint BETWEEN %s AND %s)
                  AND payload -> 'holistic_restore_superseded' IS DISTINCT FROM 'true'::jsonb
                ORDER BY id
                """,
                (witness_span is not None, campaign, after_id, first_pps, first_pps, last_pps),
            )
            yield from cur


def active_campaign_tail(campaign: str) -> int:
    from zpnet.shared.db import open_db

    with open_db(row_dict=True) as db:
        row = db.execute(
            """
            SELECT COALESCE((
                SELECT MAX(detail.id)
                FROM campaign_detail AS detail
                WHERE detail.campaign_type = master.campaign_type
                  AND detail.campaign = master.campaign
            ), 0) AS detail_id
            FROM campaign_master AS master
            WHERE master.campaign_type = 'TEMPEST'
              AND master.campaign = %s AND master.active IS TRUE
            """,
            (campaign,),
        ).fetchone()
    if row is None:
        raise RuntimeError(f"Run the DAC sequence against an active TEMPEST campaign ({campaign!r})")
    return int(row["detail_id"])


def clocks_command(command: str, args: dict | None = None) -> dict:
    from zpnet.processes.processes import send_command

    result = send_command(machine="PI", subsystem="CLOCKS", command=command, args=args)
    if not result.get("success"):
        raise RuntimeError(f"CLOCKS.{command} failed: {result.get('message')} {result.get('payload') or ''}")
    return result["payload"]


def calibration_dac_info() -> dict:
    info = clocks_command("DAC_INFO")
    if info["servo_mode"] != "10-MIN" or not info["dither_operator_enabled"]:
        raise RuntimeError("Run the DAC sequence with the 10-MIN servo and dithering enabled")
    return info


def run_dac_steps(
    campaign: str, hold_seconds: int, step_codes: Decimal,
) -> dict[str, tuple[int, int]]:
    ranges = {}
    duration_minutes, duration_seconds = divmod(16 * hold_seconds, 60)
    print(
        f"Running DAC steps for {campaign}. "
        f"Hold time {hold_seconds} s per setting, about {duration_minutes}m {duration_seconds:02d}s total.",
        flush=True,
    )
    for lane, argument in zip(LANES, ("dac1", "dac2")):
        start_id = active_campaign_tail(campaign)
        state = calibration_dac_info()[lane]
        center = Decimal(str(state["desired"]))
        low, high = center - step_codes, center + step_codes
        if low < 0 or high > state["safe_max_hw_code"]:
            raise RuntimeError(f"{lane.upper()} step would exceed the CLOCKS DAC limits")
        low = low.quantize(DITHER_CODE_RESOLUTION)
        high = high.quantize(DITHER_CODE_RESOLUTION)
        targets = (low, high, low, high, low, high, low)
        span_uv = (high - low) * UV_PER_CODE
        print(
            f"{lane.upper()} starting target {center} codes. "
            f"Requested low {low}, high {high}, nominal span {span_uv:.3f} µV.",
            flush=True,
        )
        try:
            for number, target in enumerate(targets, start=1):
                clocks_command("SET_DAC", {argument: str(target)})
                print(
                    f"  {number}/{len(targets)} target {target} codes, holding {hold_seconds} s",
                    flush=True,
                )
                sleep(hold_seconds)
                active_campaign_tail(campaign)
                end_target = calibration_dac_info()[lane]["desired"]
                print(f"      target at end of hold {end_target:.6f} codes", flush=True)
        finally:
            try:
                clocks_command("SET_DAC", {argument: str(center)})
            except (OSError, RuntimeError) as exc:
                raise RuntimeError(
                    f"{lane.upper()} restore failed. Run pc clocks set_dac {argument}={center}"
                ) from exc
            print(f"  {lane.upper()} restored to {center} codes", flush=True)

        # Allow the final pre-restore plateau to reach campaign persistence and
        # keep the next lane's measurement window separate.
        sleep(hold_seconds)
        ranges[lane] = (start_id, active_campaign_tail(campaign))
    return ranges


def extract_samples(row: dict) -> tuple[str, dict[str, Sample]]:
    if row["exposure_recorded"]:
        source = APPLIED
        evidence = row["exposure"]
        if (
            not evidence
            or evidence.get("schema") != "CLOCKS_DAC_EXPOSURE_V1"
            or evidence["campaign"] != row["campaign"]
            or evidence["reference_ns"] != BILLION
        ):
            return source, {}
    else:
        source = LEGACY
        evidence = row["observation"]
        if not evidence or evidence.get("schema") != "DAC_OBSERVATION_V1":
            return source, {}

    mode = evidence.get("servo_mode")
    if mode not in SERVO_MODES or not row["campaign_count"] or row["stats_epoch"] is None:
        return source, {}

    raw = row["raw_cycles"] or {}
    reference = raw.get("vclock") or {}
    if not reference.get("valid") or not reference.get("observed_cycles"):
        return source, {}
    reference_cycles = int(reference["observed_cycles"])
    sequence = int(row["sequence"])
    campaign_count = int(row["campaign_count"])
    epoch = (row["stats_epoch"], mode)
    samples = {}
    for lane in LANES:
        observed = raw.get(lane) or {}
        if not observed.get("valid") or not observed.get("observed_cycles"):
            continue
        value = evidence.get(f"{lane}_code_nano" if source == APPLIED else lane)
        if value is None:
            continue
        code = Decimal(str(value))
        if source == APPLIED:
            code /= BILLION
        samples[lane] = Sample(
            sequence, campaign_count, epoch, code, reference_cycles,
            int(observed["observed_cycles"]),
        )
    return source, samples


def cycle_rate_ppb(reference_cycles: int, observed_cycles: int) -> Decimal:
    return Decimal((reference_cycles - observed_cycles) * BILLION) / observed_cycles


def period_rate_ppb(samples: list[Sample]) -> Decimal:
    # These are DWT cycles per oscillator period. A shorter period means a
    # higher frequency. Aggregate elapsed periods before taking the ratio.
    reference = sum(sample.reference_cycles for sample in samples)
    observed = sum(sample.ocxo_cycles for sample in samples)
    return cycle_rate_ppb(reference, observed)


class StepSearch:
    def __init__(self, window: int, settle: int, plateau_uv: Decimal, min_step_uv: Decimal):
        self.window = window
        self.settle = settle
        self.plateau_code = plateau_uv / UV_PER_CODE
        self.min_step_code = min_step_uv / UV_PER_CODE
        self.samples: deque[Sample] = deque(maxlen=2 * window + settle)
        self.steps: list[Step] = []
        self.count = 0
        self.minimum: Decimal | None = None
        self.maximum: Decimal | None = None

    def add(self, sample: Sample) -> None:
        self.count += 1
        self.minimum = sample.code if self.minimum is None else min(self.minimum, sample.code)
        self.maximum = sample.code if self.maximum is None else max(self.maximum, sample.code)
        if self.samples:
            previous = self.samples[-1]
            if (
                sample.sequence != previous.sequence + 1
                or sample.campaign_count != previous.campaign_count + 1
                or sample.epoch != previous.epoch
            ):
                self.samples.clear()
        self.samples.append(sample)
        if len(self.samples) != self.samples.maxlen:
            return

        # Most windows contain no step. Compare their endpoints before
        # scanning both plateaus for movement.
        if abs(self.samples[-1].code - self.samples[0].code) < (
            self.min_step_code - 2 * self.plateau_code
        ):
            return
        before = list(islice(self.samples, self.window))
        after = list(islice(self.samples, self.window + self.settle, None))
        before_codes = [item.code for item in before]
        after_codes = [item.code for item in after]
        if (
            max(before_codes) - min(before_codes) > self.plateau_code
            or max(after_codes) - min(after_codes) > self.plateau_code
        ):
            return
        delta_code = (sum(after_codes) - sum(before_codes)) / self.window
        if abs(delta_code) < self.min_step_code:
            return

        self.steps.append(
            Step(
                before[0].campaign_count, after[-1].campaign_count, delta_code,
                period_rate_ppb(after) - period_rate_ppb(before),
            )
        )
        # Count a transition once and keep the compared windows disjoint.
        self.samples.clear()


def show_result(lane: str, source: str, search: StepSearch, *, show_steps: bool) -> Decimal | None:
    print(f"\n{lane.upper()}  {source}")
    if not search.count:
        print("  No paired servo-enabled measurements.")
        return None
    span_uv = (search.maximum - search.minimum) * UV_PER_CODE
    ups = [step.slope for step in search.steps if step.delta_code > 0]
    downs = [step.slope for step in search.steps if step.delta_code < 0]
    print(f"  Paired intervals {search.count:,}   DAC span {span_uv:.3f} µV")
    print(f"  Isolated steps {len(search.steps):,}   up {len(ups):,}   down {len(downs):,}")
    if show_steps and search.steps:
        print("    PPS first  PPS last      ΔDAC (µV)      Δrate (ppb)      ppb/code")
        for step in search.steps:
            print(
                f"    {step.first_pps:9d}  {step.last_pps:8d}  "
                f"{step.delta_code * UV_PER_CODE:+14.3f}  {step.delta_ppb:+15.6f}  "
                f"{step.slope:+12.6f}"
            )
    if not search.steps:
        print("  No fit. Look for isolated steps or adjust the plateau/window settings.")
        return None

    slopes = [step.slope for step in search.steps]
    center = median(slopes)
    deviation = median(abs(value - center) for value in slopes)
    up_center = median(ups) if ups else None
    down_center = median(downs) if downs else None
    print(f"  Median slope {center:+.6f} ppb/code  ({center / VOLTS_PER_CODE:+.3f} ppb/V)")
    print(f"  Median absolute slope deviation {deviation:.6f} ppb/code")
    up_label = "---" if up_center is None else f"{up_center:+.6f}"
    down_label = "---" if down_center is None else f"{down_center:+.6f}"
    print(f"  Directional medians  up {up_label}  down {down_label} ppb/code")
    if (
        source == APPLIED and len(ups) >= 2 and len(downs) >= 2
        and up_center * down_center > 0 and deviation < abs(center)
    ):
        return center
    print("  Candidate command requires applied evidence and agreeing up/down responses.")
    return None


def raw_lane(row: dict, lane: str) -> dict:
    return (row["raw_cycles"] or {}).get(lane) or {}


def witness_cycles(row: dict, lane: str, reference: str) -> tuple[int, int] | None:
    if reference == "PPS":
        science = (row["pps_witness"] or {}).get(lane) or {}
        if not science.get("delta_raw_valid"):
            return None
        ref = science.get("delta_raw_reference_interval_cycles")
        observed = science.get("delta_raw_clock_interval_cycles")
    else:
        ref_lane, observed_lane = raw_lane(row, "vclock"), raw_lane(row, lane)
        if not ref_lane.get("valid") or not observed_lane.get("valid"):
            return None
        ref = ref_lane.get("observed_cycles")
        observed = observed_lane.get("observed_cycles")
    if ref and observed:
        return int(ref), int(observed)
    return None


def plateau_rate(rows: list[dict], lane: str, reference: str) -> Decimal | None:
    ref_total = observed_total = 0
    for row in rows:
        cycles = witness_cycles(row, lane, reference)
        if cycles is None:
            return None
        ref, observed = cycles
        ref_total += ref
        observed_total += observed
    return cycle_rate_ppb(ref_total, observed_total)


def plateau_voltage(rows: list[dict], lane: str) -> Decimal | None:
    total = Decimal(0)
    for row in rows:
        source, samples = extract_samples(row)
        if source != APPLIED or lane not in samples:
            return None
        total += samples[lane].code
    return total * UV_PER_CODE / len(rows)


def difference(before: Decimal | None, after: Decimal | None) -> Decimal | None:
    return None if before is None or after is None else after - before


def signed_number(value: Decimal | None, decimals: int) -> str:
    return "---" if value is None else f"{value:+.{decimals}f}"


def witness_integrity(rows: list[dict]) -> list[str]:
    issues = []
    for left, right in zip(rows, rows[1:]):
        pps = right["campaign_count"]
        if int(right["sequence"]) != int(left["sequence"]) + 1:
            issues.append(f"fragment sequence discontinuity at PPS {pps}")
        if left["stats_epoch"] != right["stats_epoch"]:
            issues.append(f"statistics reset at PPS {pps}")
        for lane in ("vclock", *LANES):
            left_count = raw_lane(left, lane).get("completed_interval_count")
            right_count = raw_lane(right, lane).get("completed_interval_count")
            if (left_count is None or right_count is None
                    or (int(right_count) - int(left_count)) % (1 << 32) != 1):
                issues.append(f"{lane.upper()} counter {left_count}->{right_count} at PPS {pps}")
    return issues


def compare_witness_windows(
    campaign: str, starts_by_lane: dict[str, list[int]], window: int, settle: int,
) -> bool:
    width = 2 * window + settle
    selections = [(lane, first) for lane, starts in starts_by_lane.items() for first in starts]
    span = min(first for _, first in selections), max(first for _, first in selections) + width - 1
    by_pps = defaultdict(list)
    for row in campaign_rows(campaign, witness_span=span):
        by_pps[int(row["campaign_count"])].append(row)

    print(f"TEMPEST witness comparison  {campaign}  ({window} s / {settle} s / {window} s)")
    print("Each rate column is the post-step minus pre-step rate in ppb.")
    print("Pair is the stepped lane minus the companion lane, using campaign PPS.")
    print("PPS and VCLOCK share the OCXO measurement when the lane cycle counts match.")
    print(
        f"{'LANE':<6}{'PPS WINDOW':>13}{'ΔDAC µV':>11}"
        f"{'VCLOCK':>13}{'PPS':>13}{'OTHER VCLK':>13}{'OTHER PPS':>13}{'PAIR PPS':>13}"
    )
    complete = True
    for lane, first in selections:
        last = first + width - 1
        counts = range(first, last + 1)
        unmatched = [pps for pps in counts if len(by_pps[pps]) != 1]
        label = f"{lane.upper():<6}{f'{first}-{last}':>13}"
        if unmatched:
            complete = False
            detail = ", ".join(f"{pps} ({len(by_pps[pps])} rows)" for pps in unmatched)
            print(f"{label}  Missing or duplicate PPS rows: {detail}")
            continue

        rows = [by_pps[pps][0] for pps in counts]
        before, after = rows[:window], rows[-window:]
        other = LANES[1] if lane == LANES[0] else LANES[0]
        delta_voltage = difference(plateau_voltage(before, lane), plateau_voltage(after, lane))
        rates = [
            difference(plateau_rate(before, measured, reference), plateau_rate(after, measured, reference))
            for measured in (lane, other) for reference in ("VCLOCK", "PPS")
        ]
        pair = difference(rates[3], rates[1])
        values = "".join(f"{signed_number(value, 6):>13}" for value in (*rates, pair))
        print(f"{label}{signed_number(delta_voltage, 3):>11}{values}")

        issues = witness_integrity(rows)
        if issues:
            complete = False
            print(f"  Continuity: {', '.join(issues)}")
        else:
            print(f"  Continuity: all {width - 1} increments are +1 on VCLOCK and both OCXO lanes")
        pps_references = [
            row["campaign_count"] for row in before + after
            if (selected := witness_cycles(row, lane, "PPS")) is not None
            and (companion := witness_cycles(row, other, "PPS")) is not None
            and selected[0] != companion[0]
        ]
        if pps_references:
            complete = False
            print(f"  Campaign PPS reference differs between lanes at {pps_references}")
        for measured in (lane, other):
            mismatched = []
            unavailable = []
            for row in before + after:
                vclock = witness_cycles(row, measured, "VCLOCK")
                pps = witness_cycles(row, measured, "PPS")
                if vclock is None or pps is None:
                    unavailable.append(row["campaign_count"])
                elif vclock[1] != pps[1]:
                    mismatched.append(row["campaign_count"])
            if mismatched or unavailable:
                complete = False
                print(f"  {measured.upper()} lane cycles differ at {mismatched}, unavailable at {unavailable}")
        if delta_voltage is None:
            complete = False
            print("  Applied DAC exposure unavailable for part of this window")
    return complete


def positive_decimal(value: str) -> Decimal:
    try:
        result = Decimal(value)
    except InvalidOperation as error:
        raise argparse.ArgumentTypeError("enter a positive finite decimal") from error
    if not result.is_finite() or result <= 0:
        raise argparse.ArgumentTypeError("enter a positive finite decimal")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", help="TEMPEST campaign name")
    parser.add_argument("--window", type=int, help="intervals on each plateau (12 by default, 6 for step runs and witness comparisons)")
    parser.add_argument("--settle", type=int, default=4, help="intervals skipped between plateaus (default 4)")
    parser.add_argument("--plateau-uv", type=positive_decimal,
                        help="maximum plateau voltage span (2 µV by default, 3 µV with --run-steps)")
    parser.add_argument("--min-step-uv", type=positive_decimal,
                        help="minimum voltage difference (5 µV by default, 8 µV with --run-steps)")
    parser.add_argument("--steps", action="store_true", help="list the paired plateaus used in each estimate")
    parser.add_argument("--run-steps", action="store_true",
                        help="apply reversible DAC steps in the active campaign, then fit this run")
    parser.add_argument("--step-codes", type=positive_decimal,
                        help="DAC codes on each side of the starting target (default 0.25 with --run-steps)")
    for lane in LANES:
        parser.add_argument(
            f"--witness-{lane}", nargs="+", type=int, default=[], metavar="FIRST_PPS",
            help=f"compare the exact {lane.upper()} step windows starting at these PPS counts",
        )
    args = parser.parse_args()
    if args.step_codes is not None and not args.run_steps:
        parser.error("--step-codes requires --run-steps")
    starts = {lane: getattr(args, f"witness_{lane}") for lane in LANES}
    compare_witnesses = any(starts.values())
    if args.window is None:
        args.window = 6 if args.run_steps or compare_witnesses else 12
    if args.plateau_uv is None:
        args.plateau_uv = Decimal(3 if args.run_steps else 2)
    if args.min_step_uv is None:
        args.min_step_uv = Decimal(8 if args.run_steps else 5)
    if args.window < 2 or args.settle < 1:
        parser.error("--window must be at least 2 and --settle at least 1")
    if compare_witnesses:
        if args.run_steps:
            parser.error("omit --run-steps when comparing existing witness windows")
        if any(pps <= 0 for counts in starts.values() for pps in counts):
            parser.error("witness window PPS counts must be positive")
        if not compare_witness_windows(args.campaign, starts, args.window, args.settle):
            parser.exit(1, "Some witness windows have missing or inconsistent evidence.\n")
        return
    if args.min_step_uv <= 2 * args.plateau_uv:
        parser.error("--min-step-uv must exceed twice --plateau-uv")

    lane_ranges = {}
    if args.run_steps:
        step_codes = args.step_codes if args.step_codes is not None else DEFAULT_STEP_CODES
        if step_codes < args.min_step_uv / (2 * UV_PER_CODE):
            parser.error("the nominal low-to-high DAC span must reach --min-step-uv")
        try:
            lane_ranges = run_dac_steps(
                args.campaign, max(18, 2 * args.window + args.settle + 2), step_codes,
            )
        except KeyboardInterrupt:
            parser.exit(130, "DAC sequence interrupted.\n")
        except (OSError, RuntimeError) as exc:
            parser.exit(1, f"{exc}\n")

    searches = {
        (source, lane): StepSearch(args.window, args.settle, args.plateau_uv, args.min_step_uv)
        for source in (APPLIED, LEGACY)
        for lane in LANES
    }
    row_count = 0
    after_id = min(start for start, _ in lane_ranges.values()) if lane_ranges else 0
    for row in campaign_rows(args.campaign, after_id):
        if lane_ranges and row["id"] > lane_ranges[LANES[-1]][1]:
            break
        row_count += 1
        source, samples = extract_samples(row)
        for lane, sample in samples.items():
            if lane_ranges:
                start_id, end_id = lane_ranges[lane]
                if not start_id < row["id"] <= end_id:
                    continue
            searches[(source, lane)].add(sample)

    print(f"TEMPEST DAC slope report  {args.campaign}  ({row_count:,} admitted rows)")
    print(f"Plateau {args.window} s  settling gap {args.settle} s  minimum step {args.min_step_uv} µV")
    candidates = {}
    for source in (APPLIED, LEGACY):
        if source == LEGACY and not any(searches[(source, lane)].count for lane in LANES):
            continue
        for lane in LANES:
            candidate = show_result(lane, source, searches[(source, lane)], show_steps=args.steps)
            if candidate is not None:
                candidates[lane] = candidate
    print("\nSlope K is the change in physical frequency for an increase in DAC code.")
    print("The reconstructed OCXO rate uses −K × (applied code − baseline code).")
    print("Pi exposure uses fragment receipt timing. Confirm PPS alignment on the bench.")
    print("Servo steps respond to disturbances, so these estimates can be biased.")
    print("Compare directions and repeat runs at steady conditions or use deliberate reversible steps.")
    if len(candidates) == len(LANES):
        print("\nCandidate values for review")
        assignments = " ".join(f"{lane}={candidates[lane]:.6f}" for lane in LANES)
        print(f"  pc clocks set_dac_slope {assignments}")


if __name__ == "__main__":
    main()
