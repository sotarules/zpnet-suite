"""Locate excessive PHOTONS race counts in a recorded LANTERN campaign.

Run in the same Python environment as photons_analyzer.py:
    python photons_race_onset.py Mat1
    python photons_race_onset.py Mat1 --threshold 1000 --sustain 5
    python photons_race_onset.py Mat1 --from-id 3468968

Read-only; streams Mat1's own rows in database-id order. Counts are the
producer's per-fragment values, exactly as used by the PHOTONS rolling readout.
SPURIOUS already includes EARLY + DUP + LATE + UNARM; never add them twice.
Missing historical telemetry is unavailable, not zero. Database timestamps
describe recorded observations, not the exact instant an electrical fault began.

Field mappings checked against zpnet/metrics/readout_blocks.py at 1edee0f3.
Database access follows the supplied photons_analyzer.py.
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from zoneinfo import ZoneInfo


@dataclass(frozen=True)
class Sample:
    db_id: int
    ts: datetime
    second: int
    sequence: int
    reset: int
    update: int
    generation: int | None
    receipt: bool
    race: dict
    accepted: int
    excluded: int

    @property
    def spurious(self):
        return self.race.get("spurious_this_fragment")

    def follows(self, previous):
        """Only consecutive observations within one recorded producer epoch."""
        return (
            previous is not None
            and self.second == previous.second + 1
            and self.sequence == previous.sequence + 1
            and self.reset == previous.reset
            and self.update == previous.update + 1
            and self.generation == previous.generation
            and not self.receipt
            and self.race.get("capture_gate") == previous.race.get("capture_gate")
            and self.race.get("capture_min_ns") == previous.race.get("capture_min_ns")
            and self.race.get("capture_max_ns") == previous.race.get("capture_max_ns")
        )


def iter_samples(args):
    # Delayed import also permits --help and offline detector checks without DB.
    from zpnet.shared.db import open_db

    where = ["campaign_type = %s", "campaign = %s",
             "payload ->> 'schema' = %s"]
    params = ["LANTERN", args.campaign, "PHOTONS_V1"]
    if args.from_id:
        where.append("id >= %s")
        params.append(args.from_id)
    if args.to_id:
        where.append("id <= %s")
        params.append(args.to_id)
    sql = """
        SELECT id, ts,
            payload #> '{campaign,public_count}' AS second,
            payload -> 'sequence' AS sequence,
            payload #> '{photons,stats,reset_count}' AS reset,
            payload #> '{photons,stats,update_count}' AS update,
            payload #> '{photons,recovery,generation}' AS generation,
            (payload -> 'recovery_receipt' IS NOT NULL
             AND payload -> 'recovery_receipt' <> 'null'::jsonb) AS receipt,
            payload #> '{photons,race}' AS race,
            payload #> '{photons,science,accepted,count_this_fragment}' AS accepted,
            payload #> '{photons,science,excluded,count_this_fragment}' AS excluded
        FROM campaign_detail
        WHERE """ + " AND ".join(where) + " ORDER BY id ASC"
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        conn.execute("SET LOCAL TIME ZONE 'UTC'")
        with conn.cursor(name="photons_race_onset") as cur:
            cur.itersize = 512
            cur.execute(sql, tuple(params))
            for row in cur:
                race = row["race"]
                if isinstance(race, str):
                    race = json.loads(race)
                # Old rows may predate race telemetry entirely.
                row["race"] = {} if race is None else race
                row["db_id"] = row.pop("id")
                yield Sample(**row)


class Scan:
    def __init__(self, threshold, sustain, context):
        self.threshold, self.sustain, self.context = threshold, sustain, context
        self.history = deque(maxlen=context + sustain + 1)
        self.windows = {}
        self.first = self.last = self.peak = None
        self.rows = self.known = self.high = self.breaks = self.run = 0
        self.longest = 0
        self.first_by_reason = {}

    def add_window(self, name, center):
        self.windows[name] = {
            "center": center,
            "rows": [(i, row) for i, row in self.history
                     if center - self.context <= i <= center + self.context],
        }

    def add(self, row):
        self.rows += 1
        index = self.rows
        for window in self.windows.values():
            if index <= window["center"] + self.context:
                window["rows"].append((index, row))
        self.history.append((index, row))
        adjacent = row.follows(self.last)
        if self.last is not None and not adjacent:
            self.breaks += 1
        if not adjacent:
            self.run = 0
        if self.first is None:
            self.first = row
        count = row.spurious
        if count is not None:
            self.known += 1
            if self.peak is None or count > self.peak.spurious:
                self.peak = row
        if count is not None and count >= self.threshold:
            self.high += 1
            self.run += 1
            self.longest = max(self.longest, self.run)
            if "FIRST CROSSING" not in self.windows:
                self.add_window("FIRST CROSSING", index)
            if self.run == self.sustain and "FIRST SUSTAINED ONSET" not in self.windows:
                self.add_window("FIRST SUSTAINED ONSET", index - self.sustain + 1)
        else:
            self.run = 0
        for reason in ("early", "duplicate", "late", "unarmed"):
            value = row.race.get(f"spurious_{reason}_this_fragment")
            if value is not None and value >= self.threshold:
                self.first_by_reason.setdefault(reason, row)
        self.last = row


def shown(value, width=8, digits=None):
    if value is None:
        return f"{'NA':>{width}}"
    return (f"{value:>{width}.{digits}f}" if digits is not None
            else f"{value:>{width}}")


def stamp(row, zone):
    return row.ts.astimezone(zone).isoformat(timespec="seconds")


def describe(row, zone):
    return (f"id={row.db_id} db_time={stamp(row, zone)} SEC={row.second} "
            f"sequence={row.sequence} reset={row.reset} update={row.update} "
            f"generation={row.generation} receipt={row.receipt} "
            f"SPURIOUS={row.spurious}")


def print_report(scan, args):
    zone = ZoneInfo(args.timezone)
    print(f"PHOTONS race onset: {args.campaign}")
    print(f"Rule: SPURIOUS >= {args.threshold} per fragment; sustained = "
          f"{args.sustain} consecutive recorded campaign seconds.")
    print(f"Database timestamps in {args.timezone}; NA = telemetry not recorded.")
    if scan.rows == 0:
        print("No matching PHOTONS_V1 campaign rows in this range.")
        return
    print(f"Scanned {scan.rows:,} rows; SPURIOUS available {scan.known:,}; "
          f"unavailable {scan.rows - scan.known:,}; above threshold {scan.high:,}.")
    print(f"Continuity/epoch/gate boundaries: {scan.breaks:,}; "
          f"longest consecutive elevated run: {scan.longest:,} rows.")
    print("First: " + describe(scan.first, zone))
    print("Last:  " + describe(scan.last, zone))
    if scan.peak is not None:
        print("Peak:  " + describe(scan.peak, zone))
    for name in ("FIRST CROSSING", "FIRST SUSTAINED ONSET"):
        print(f"\n{name}")
        if name not in scan.windows:
            print("Not observed in the available telemetry with this rule.")
            continue
        window = scan.windows[name]
        center = window["center"]
        onset = next(row for i, row in window["rows"] if i == center)
        print(describe(onset, zone))
        print(f"Capture gate={onset.race.get('capture_gate')} "
              f"window_ns={onset.race.get('capture_min_ns')}.."
              f"{onset.race.get('capture_max_ns')}")
        print(" * marks onset; | marks a continuity/epoch/gate boundary.")
        print("   DB_TIME                         ID      SEC    ACCEPT     EXCL   MISSED "
              "SPURIOUS    EARLY      DUP     LATE    UNARM       LAP_NS       SD_NS")
        previous = None
        for i, row in window["rows"]:
            boundary = "|" if previous is not None and not row.follows(previous) else " "
            marker = "*" if i == center else " "
            race = row.race
            flight = race.get("flight_ns") or {}
            counts = [row.accepted, row.excluded, race.get("missed_this_fragment"),
                      row.spurious] + [race.get(f"spurious_{r}_this_fragment")
                                      for r in ("early", "duplicate", "late", "unarmed")]
            print(f"{marker}{boundary} {stamp(row, zone):<25} {row.db_id:>8} {row.second:>8} "
                  + " ".join(shown(v) for v in counts)
                  + " " + shown(flight.get("mean"), 12, 6)
                  + " " + shown(flight.get("stddev"), 11, 6))
            previous = row
        if center == 1:
            print("Already elevated at the first scanned row; onset may predate this range.")
    if scan.first_by_reason:
        print("\nFIRST CROSSING BY REASON (same threshold)")
        for reason, row in scan.first_by_reason.items():
            print(f"{reason.upper():9} {describe(row, zone)} "
                  f"count={row.race[f'spurious_{reason}_this_fragment']}")
    print("\nOnsets are first recorded evidence under the chosen rule. Gaps and "
          "missing telemetry do not establish normal behavior between observations.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("campaign", nargs="?", default="Mat1")
    parser.add_argument("--threshold", type=int, default=100,
                        help="SPURIOUS per fragment triggering a crossing (default: 100)")
    parser.add_argument("--sustain", type=int, default=5,
                        help="consecutive elevated rows confirming onset (default: 5)")
    parser.add_argument("--context", type=int, default=5,
                        help="rows before/after each onset (default: 5)")
    parser.add_argument("--from-id", type=int, default=0)
    parser.add_argument("--to-id", type=int, default=0)
    parser.add_argument("--timezone", default="America/Los_Angeles")
    args = parser.parse_args()
    if min(args.threshold, args.sustain) < 1 or min(args.context, args.from_id, args.to_id) < 0:
        parser.error("threshold/sustain must be positive; context/IDs nonnegative")
    if args.to_id and args.from_id > args.to_id:
        parser.error("--from-id exceeds --to-id")
    ZoneInfo(args.timezone)
    scan = Scan(args.threshold, args.sustain, args.context)
    for row in iter_samples(args):
        scan.add(row)
    print_report(scan, args)


if __name__ == "__main__":
    main()
