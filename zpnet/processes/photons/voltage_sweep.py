"""Bounded, volatile commissioning survey; no fitting or LAP feedback.

    pc photons sweep_start targets_mv=440,447.5,455
    pc photons sweep_report
    pc photons sweep_stop

Optional START arguments: settle_seconds=30 measure_seconds=180
seek_timeout_seconds=600 passes=2. Passes alternates supplied/reversed order.
At most ten targets and two passes; one report is retained until the next START
or process restart. Save command STDOUT before restarting. Normal campaign
persistence also retains the small sweep identity in DAC ingress witnesses.

Every visit first requires continuous in-band HIGH5, then records ALL complete
fragments for a fixed measurement duration, including excursions and servo
corrections. Never select only in-band measurement rows. Missing chronology,
empty populations, stale data or lifecycle holds interrupt the survey rather
than hide unfavorable intervals. An unreachable target times out and is listed.
Targets are prescribed by the operator: LAP statistics never choose a voltage.
"""

import copy
import math
import threading
import time
from datetime import datetime, timezone


class Moments:
    def __init__(self):
        self.n = 0
        self.mean = self.m2 = 0.0
        self.low = self.high = None

    def add(self, x):
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self.m2 += d * (x - self.mean)
        self.low = x if self.low is None else min(self.low, x)
        self.high = x if self.high is None else max(self.high, x)

    def report(self):
        return {"n": self.n, "mean": self.mean if self.n else None,
                "sd": math.sqrt(max(0, self.m2 / (self.n - 1))) if self.n > 1 else None,
                "min": self.low, "max": self.high}


class Relation:
    """Centered online covariance; descriptive regression, never a controller."""
    def __init__(self):
        self.x, self.y = Moments(), Moments()
        self.c = 0.0

    def add(self, x, y):
        dx = x - self.x.mean
        self.x.add(x)
        self.y.add(y)
        self.c += dx * (y - self.y.mean)

    def report(self):
        slope = self.c / self.x.m2 if self.x.m2 > 0 else None
        corr = (max(-1., min(1., self.c / math.sqrt(self.x.m2 * self.y.m2)))
                if self.x.m2 > 0 and self.y.m2 > 0 else None)
        return {"slope": slope, "correlation": corr}


class Visit:
    def __init__(self, target, number, pass_number):
        self.target, self.number, self.pass_number = target, number, pass_number
        self.status = "SEEKING"
        self.metrics = {k: Moments() for k in (
            "raw_second_mean_ns", "published_second_mean_ns", "high5_mv", "high_fragment_mv",
            "accepted_races_per_fragment", "dac_v",
            "raw_10_fragment_block_mean_ns", "pi_c", "pd_c", "out_c", "rtd_c")}
        self.time_relation, self.high_relation = Relation(), Relation()
        self.races = 0
        self.race_mean = self.race_m2 = self.within_m2 = 0.0
        self.within_df = 0
        self.block_n = self.block_races = 0
        self.block_sum = 0.0
        self.first = self.last = None
        self.first_ms = self.last_ms = None
        self.first_writes = self.last_writes = None
        self.previous_v = None
        self.travel = 0.0
        self.outside_band = self.at_rail = 0

    def add(self, f, deadband, max_code):
        raw = f["raw_flight_ns"]
        n, mean, m2 = raw["n"], raw["mean"], raw["m2"]
        witness = f["dac_witness"]
        v = witness["commanded_dac_v"]
        end = f["high5"]["completed_at_ms"]
        if self.first is None:
            self.first = f["fragment_received_at_utc"]
            self.first_ms = end
            self.first_writes = witness["write_count"]
        self.last = f["fragment_received_at_utc"]
        self.last_ms, self.last_writes = end, witness["write_count"]
        high = f["high5"]["high_mean_mv"]
        for k, x in (("raw_second_mean_ns", mean),
                     ("published_second_mean_ns", f["published_flight_ns"]["mean"]),
                     ("high5_mv", high), ("high_fragment_mv", f["high_fragment_mv"]),
                     ("accepted_races_per_fragment", n), ("dac_v", v)):
            self.metrics[k].add(x)
        for k, x in f["temperatures_c"].items():
            if x is not None:
                self.metrics[k + "_c"].add(x)
        self.time_relation.add((end - self.first_ms) / 1000, mean)
        self.high_relation.add(high, mean)
        total = self.races + n
        d = mean - self.race_mean
        self.race_m2 += m2 + d * d * self.races * n / total
        self.race_mean += d * n / total
        self.races = total
        self.within_m2 += m2
        self.within_df += n - 1
        self.block_n += 1
        self.block_races += n
        self.block_sum += n * mean
        if self.block_n == 10:
            self.metrics["raw_10_fragment_block_mean_ns"].add(self.block_sum / self.block_races)
            self.block_n = self.block_races = 0
            self.block_sum = 0.0
        if self.previous_v is not None:
            self.travel += abs(v - self.previous_v)
        self.previous_v = v
        self.outside_band += abs(high - self.target) > deadband
        self.at_rail += witness["commanded_code"] in (0, max_code)

    def report(self):
        count = self.metrics["raw_second_mean_ns"].n
        span = (self.last_ms - self.first_ms) / 1000 if count > 1 else 0
        writes = self.last_writes - self.first_writes if count else 0
        return {
            "visit": self.number, "pass": self.pass_number, "target_mv": self.target,
            "status": self.status, "first_received_utc": self.first,
            "last_received_utc": self.last, "observed_span_seconds": span,
            "metrics": {k: v.report() for k, v in self.metrics.items()},
            "accepted_races": self.races,
            "pooled_raw_race_sd_ns": math.sqrt(max(0, self.race_m2 / (self.races - 1))) if self.races > 1 else None,
            "pooled_within_fragment_sd_ns": math.sqrt(max(0, self.within_m2 / self.within_df)) if self.within_df else None,
            "raw_drift_ns_per_second": self.time_relation.report()["slope"],
            "raw_vs_high5": self.high_relation.report(),
            "outside_target_band_fraction": self.outside_band / count if count else None,
            "at_dac_rail_fraction": self.at_rail / count if count else None,
            "dac_writes_between_first_and_last": writes,
            "dac_writes_per_minute": writes * 60 / span if span else None,
            "dac_total_travel_v": self.travel,
            "dac_travel_v_per_minute": self.travel * 60 / span if span else None,
            "incomplete_block_fragments": self.block_n,
        }


class VoltageSweep:
    def __init__(self):
        self.lock = threading.RLock()
        self.phase = "IDLE"
        self.visits = []
        self.index = 0
        self.run_id = None
        self.reason = None
        self.config = None
        self.previous_target = None
        self.last_identity = None
        self.stable_since = None
        self.phase_started = 0.0
        self.signature = None

    def active(self):
        return self.phase in ("SEEKING", "MEASURING")

    def brief(self):
        with self.lock:
            return {"run_id": self.run_id, "phase": self.phase,
                    "visit": self.index + 1 if self.visits else None,
                    "target_mv": self.visits[self.index].target if self.visits else None,
                    "reason": self.reason}

    def start(self, args, previous_target):
        args = args or {}
        allowed = {"targets_mv", "settle_seconds", "measure_seconds", "seek_timeout_seconds", "passes"}
        if set(args) - allowed:
            raise ValueError("unknown SWEEP_START argument")
        targets = [float(x) for x in str(args.get("targets_mv", "440,447.5,455")).split(",")]
        if (not 2 <= len(targets) <= 10 or len(set(targets)) != len(targets)
                or any(not math.isfinite(x) or not 0 < x < 3300 for x in targets)):
            raise ValueError("targets_mv requires 2..10 distinct finite PD OUT millivolt values in (0,3300)")
        cfg = {}
        for key, default, lo, hi in (("settle_seconds", 30, 15, 300),
                                    ("measure_seconds", 180, 60, 3600),
                                    ("seek_timeout_seconds", 600, 30, 1800),
                                    ("passes", 2, 1, 2)):
            value = str(args.get(key, default))
            if not value.isascii() or not value.isdigit() or not lo <= int(value) <= hi:
                raise ValueError(f"{key} must be an integer in {lo}..{hi}")
            cfg[key] = int(value)
        if cfg["seek_timeout_seconds"] <= cfg["settle_seconds"] + 7:
            raise ValueError("seek_timeout_seconds must exceed settle_seconds + 7")
        with self.lock:
            if self.active():
                raise ValueError("a sweep is already active; SWEEP_STOP before replacing it")
            self.config = {**cfg, "targets_mv": targets}
            self.previous_target = previous_target
            self.run_id = datetime.now(timezone.utc).isoformat()
            self.visits = [Visit(t, p * len(targets) + i + 1, p + 1)
                           for p in range(cfg["passes"])
                           for i, t in enumerate(targets if p % 2 == 0 else reversed(targets))]
            self.index = 0
            self.reason = None
            self.signature = None
            self._seek()
            return self.visits[0].target

    def _seek(self):
        self.phase = "SEEKING"
        self.phase_started = time.monotonic()
        self.stable_since = self.last_identity = None

    def cancel(self, reason):
        with self.lock:
            if self.active():
                self.visits[self.index].status = reason
                self.phase, self.reason = "INTERRUPTED", reason
                return self.previous_target
            return None

    def _next(self):
        if self.index + 1 == len(self.visits):
            self.phase = "COMPLETE"
            return self.previous_target
        self.index += 1
        self._seek()
        return self.visits[self.index].target

    def observe(self, f, hold, servo, ready_at, max_code):
        with self.lock:
            if not self.active():
                return None
            now = time.monotonic()
            if hold:
                return self.cancel(hold)
            if f is None or f["received_monotonic"] is None or not 0 <= now - f["received_monotonic"] <= 2.5:
                return self.cancel("STALE_FEEDBACK")
            if self.signature is None:
                self.signature = copy.deepcopy(f["acquisition_signature"])
            elif self.signature != f["acquisition_signature"]:
                return self.cancel("ACQUISITION_CONFIGURATION_CHANGED")
            visit = self.visits[self.index]
            if self.phase == "SEEKING" and now - self.phase_started >= self.config["seek_timeout_seconds"]:
                visit.status = "TARGET_NOT_SETTLED"
                return self._next()
            w = f["high5"]
            identity = (f["reset_count"], f["sequence"], w["completed_at_ms"])
            if identity == self.last_identity:
                return None
            if self.last_identity is not None:
                reset, seq, ms = self.last_identity
                if reset != identity[0] or seq + 1 != identity[1] or not 500 <= identity[2] - ms <= 1500:
                    return self.cancel("FEEDBACK_DISCONTINUITY")
            self.last_identity = identity
            if (f["fragment_period_ns"] != 1_000_000_000 or not f["snapshot_ok"]
                    or f["raw_flight_ns"]["n"] < 2 or f["published_flight_ns"] is None
                    or f["high_fragment_mv"] is None
                    or w["sample_count"] < 1000 or w["elapsed_ms"] < 5000 or w["fragments"] < 5):
                return self.cancel("INCOMPLETE_MEASUREMENT")
            if self.phase == "SEEKING":
                # Reject feedback captured before the target change/settling gate.
                if (f["received_monotonic"] < ready_at
                        or abs(w["high_mean_mv"] - visit.target) > servo["deadband_mv"]):
                    self.stable_since = None
                    return None
                if self.stable_since is None:
                    self.stable_since = w["completed_at_ms"]
                if w["completed_at_ms"] - self.stable_since < self.config["settle_seconds"] * 1000:
                    return None
                self.phase = visit.status = "MEASURING"
            witness = f["dac_witness"]
            if witness is None or witness["state"] != "CONFIGURED":
                return self.cancel("MISSING_DAC_WITNESS")
            visit.add(f, servo["deadband_mv"], max_code)
            if w["completed_at_ms"] - visit.first_ms >= self.config["measure_seconds"] * 1000:
                visit.status = "COMPLETE"
                return self._next()
            return None

    def report(self):
        with self.lock:
            rows = [v.report() for v in self.visits]
            ranked = []
            for target in (self.config or {}).get("targets_mv", []):
                group = [r for r in rows if r["target_mv"] == target]
                if not all(r["status"] == "COMPLETE" for r in group):
                    continue
                ranked.append({"target_mv": target, "completed_visits": len(group),
                               "worst_visit_raw_second_sd_ns": max(r["metrics"]["raw_second_mean_ns"]["sd"] for r in group),
                               "mean_high5_sd_mv": sum(r["metrics"]["high5_mv"]["sd"] for r in group) / len(group),
                               "mean_dac_writes_per_minute": sum(r["dac_writes_per_minute"] for r in group) / len(group)})
            ranked.sort(key=lambda r: r["worst_visit_raw_second_sd_ns"])
            return {"schema": "PI_PHOTONS_VOLTAGE_SWEEP_V1", **self.brief(),
                    "configuration": copy.deepcopy(self.config),
                    "restore_target_mv": self.previous_target,
                    "acquisition_signature": copy.deepcopy(self.signature),
                    "visits": rows, "descriptive_ranking": ranked,
                    "interpretation": [
                        "Volatile: this report is lost on process restart or the next sweep_start. Save STDOUT.",
                        "Ranked by lowest worst-visit SD of raw one-second means; no target is automatically selected.",
                        "RAW is pre-normalization; published scatter is recorded-model context only, not the ranking criterion.",
                        "Within-fragment SD is pooled from raw M2; between-second and nonoverlapping block SD describe different scales.",
                        "All measurement fragments count, including out-of-band excursions; seek/transition fragments do not.",
                        "HIGH5 windows overlap: these SDs are descriptive, not independent-sample confidence estimates.",
                        "Low HIGH5 scatter can reflect clipping or lost sensitivity. This report cannot certify absence of saturation.",
                        "DAC is witnessed at fragment receipt, not per-race amplitude; write counts include that observation boundary.",
                        "Compare temperatures, race counts, voltage headroom, both pass results and RAW drift before selecting a candidate.",
                        "No NOR6 fit: repeat candidates through held-out thermal cycles before adopting a model."]}
