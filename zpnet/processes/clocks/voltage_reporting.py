"""Pi CLOCKS ownership of DAC exposure, baseline cuts, and reporting recovery."""

from collections import deque
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
import json
import logging
from math import isqrt
import os
from pathlib import Path
from threading import RLock
from time import monotonic_ns
from uuid import uuid4

from zpnet.processes.clocks.voltage_clock import (
    DAC_CODE_SCALE,
    PPB_SCALE,
    PROVISIONAL_SLOPE_MICRO_PPB_PER_CODE,
    VoltageClock,
    code_nano_to_voltage_nv,
    round_ratio,
)
from zpnet.shared.db import open_db

LANES = ("ocxo1", "ocxo2")
NS_PER_SECOND = 1_000_000_000
CONFIG_KEY = "CLOCKS_VOLTAGE_REPORTING"
# Receipt time is the available Pi-side timing witness. The configured delay
# can be measured against PPS at the bench. Delivery jitter remains visible in
# host_interval_ns and limits the accuracy of the exposure estimate.
RECEIPT_DELAY_NS = int(os.environ.get("ZPNET_CLOCKS_PPS_RECEIPT_DELAY_NS", "0"))
MAX_INTERVAL_SKEW_NS = 250_000_000
MAX_CHECKPOINT_AGE_S = 25 * 60 * 60


class DacExposure:
    def __init__(self):
        self.lock = RLock()
        # One stream per lane, including unknown output after a failed write.
        self.transitions = {lane: deque(maxlen=4096) for lane in LANES}

    def note(self, lane, timestamp_ns, code):
        with self.lock:
            history = self.transitions[lane]
            if not history or history[-1][1] != code:
                history.append((timestamp_ns, code))

    def paired_means(self, start_ns, end_ns):
        with self.lock:
            return {lane: self._mean(lane, start_ns, end_ns) for lane in LANES}

    def _mean(self, lane, start_ns, end_ns):
        history = self.transitions[lane]
        area = 0
        next_ns = end_ns
        for timestamp_ns, code in reversed(history):
            if timestamp_ns >= next_ns:
                continue
            if code is None:
                return None
            segment_start = max(start_ns, timestamp_ns)
            area += code * (next_ns - segment_start)
            if segment_start == start_ns:
                return round_ratio(area * DAC_CODE_SCALE, end_ns - start_ns)
            next_ns = segment_start
        return None


def empty_dac_population():
    return {
        "n": 0, "duration_ns": 0,
        "code_ns": dict.fromkeys(LANES, 0),
        "sum_squares": dict.fromkeys(LANES, 0),
        "minimum": {}, "maximum": {},
    }


def add_exposure(population, exposure):
    duration_ns = exposure["reference_ns"]
    population["n"] += 1
    population["duration_ns"] += duration_ns
    for lane in LANES:
        value = exposure[f"{lane}_code_nano"]
        population["code_ns"][lane] += value * duration_ns
        population["sum_squares"][lane] += value * value
        population["minimum"][lane] = min(value, population["minimum"].get(lane, value))
        population["maximum"][lane] = max(value, population["maximum"].get(lane, value))


def dac_population_means(population):
    if not population or not population["n"]:
        return None
    return {
        lane: round_ratio(population["code_ns"][lane], population["duration_ns"])
        for lane in LANES
    }


def recorded_dac_means(recording):
    applied = recording.get("applied")
    means = dac_population_means(applied)
    if means is not None:
        return means, applied["n"], "applied_exposure"
    counts = [recording.get(lane, {}).get("n", 0) for lane in LANES]
    if not counts[0] or counts[0] != counts[1]:
        raise ValueError("Campaign has no paired DAC mean")
    means = {
        lane: int(Decimal(str(recording[lane]["mean"])) * DAC_CODE_SCALE)
        for lane in LANES
    }
    return means, counts[0], "legacy_target_code"


def parse_slope(value):
    try:
        scaled = Decimal(str(value)) * PPB_SCALE
        if not scaled.is_finite() or not scaled or scaled != scaled.to_integral_value():
            raise ValueError
        return int(scaled)
    except (InvalidOperation, ValueError) as exc:
        raise ValueError("Enter a signed, nonzero ppb per DAC code with up to six decimal places") from exc


def _read_config():
    with open_db(row_dict=True) as conn:
        row = conn.execute("SELECT payload FROM config WHERE config_key = %s", (CONFIG_KEY,)).fetchone()
    return row["payload"] if row else {}


def _write_config(payload):
    # All writes are serialized by VoltageReporting.lock. The operation ID in
    # payload allows a lost commit acknowledgement to be resolved by readback.
    try:
        with open_db() as conn:
            encoded = json.dumps(payload)
            cursor = conn.execute(
                "UPDATE config SET payload = %s::jsonb WHERE config_key = %s",
                (encoded, CONFIG_KEY),
            )
            if cursor.rowcount == 0:
                conn.execute(
                    "INSERT INTO config (config_key, payload) VALUES (%s, %s::jsonb)",
                    (CONFIG_KEY, encoded),
                )
            elif cursor.rowcount != 1:
                raise RuntimeError("CLOCKS voltage config must be a singleton")
    except Exception:
        if _read_config().get("operation_id") != payload["operation_id"]:
            raise


def _checkpoint_rows(campaign_type, boot_id, baseline_id, effective_at):
    # Keep all recent seconds and the first endpoint in each reporting
    # statistics minute. Also retain the exact newest endpoint after a gap.
    with open_db(row_dict=True) as conn:
        rows = conn.execute(
            """
            WITH samples AS (
                SELECT d.id, v.checkpoint,
                       v.checkpoint #> '{clock,statistics_origin}' AS statistics_origin,
                       CASE WHEN d.ts >= now() - interval '11 minutes' THEN -d.id
                            ELSE COALESCE(
                                ((v.checkpoint #>> '{clock,endpoint,0}')::bigint
                                - (v.checkpoint #>> '{clock,statistics_origin,0}')::bigint)
                                / 60000000000, 0)
                       END AS sample_bucket
                FROM campaign_detail AS d
                CROSS JOIN LATERAL
                    (SELECT d.payload #> '{clocks,voltage_checkpoint}' AS checkpoint) AS v
                WHERE d.campaign_type = %s
                  AND d.ts >= GREATEST(now() - make_interval(secs => %s), %s::timestamptz)
                  AND NOT (d.payload @> '{"holistic_restore_superseded":true}'::jsonb)
                  AND v.checkpoint ->> 'boot_id' = %s
                  AND v.checkpoint ->> 'baseline_id' IS NOT DISTINCT FROM %s
            )
            SELECT id, checkpoint
            FROM (
                (SELECT DISTINCT ON (statistics_origin, sample_bucket) id, checkpoint
                 FROM samples ORDER BY statistics_origin, sample_bucket, id)
                UNION
                (SELECT id, checkpoint FROM samples ORDER BY id DESC LIMIT 1)
            ) AS retained ORDER BY id
            """,
            (campaign_type, MAX_CHECKPOINT_AGE_S, effective_at, boot_id, baseline_id),
        ).fetchall()
        if not rows:
            rows = conn.execute(
                """
                SELECT payload #> '{clocks,voltage_checkpoint}' AS checkpoint
                FROM campaign_detail
                WHERE campaign_type = %s AND ts >= %s::timestamptz
                  AND NOT (payload @> '{"holistic_restore_superseded":true}'::jsonb)
                  AND payload #>> '{clocks,voltage_checkpoint,boot_id}' = %s
                  AND payload #>> '{clocks,voltage_checkpoint,baseline_id}' = %s
                ORDER BY id DESC LIMIT 1
                """,
                (campaign_type, effective_at, boot_id, baseline_id),
            ).fetchall()
    return [row["checkpoint"] for row in rows]


class VoltageReporting:
    def __init__(self):
        self.lock = RLock()
        self.boot_id = None
        self.config = {}
        self.baseline = None
        self.clock = None
        self.request = None
        self.last_request = None
        self.last_edge = None
        self.last_exposure = None
        self.timeline_ns = 0
        self.system_dac = empty_dac_population()
        self.segment_dac = empty_dac_population()
        self.campaign = None
        self.segment_id = None
        self.rail_interrupted = False
        self.latest = {"status": "awaiting_baseline"}
        self.exposure = DacExposure()

    def load(self, campaign_type):
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.config = _read_config()
        baseline = self.config.get("baseline")
        if baseline and baseline["boot_id"] == self.boot_id:
            self.baseline = baseline
            self.last_request = {"status": "committed", "baseline": baseline}
        elif baseline:
            self.latest = {"status": "rebaseline_required"}
            return

        self.rail_interrupted = bool(self.config.get("rebaseline_required"))
        if self.baseline is None:
            if self.rail_interrupted:
                self.latest = {"status": "rebaseline_required"}
            return

        rows = _checkpoint_rows(
            campaign_type, self.boot_id, self.baseline["id"], self.baseline["effective_at_utc"],
        )
        if rows:
            self.clock = VoltageClock.restore(
                self.baseline, rows[-1]["clock"], (row["clock"] for row in rows),
            )
        else:
            self.clock = VoltageClock(self.baseline)
        if rows:
            last = rows[-1]
            self.timeline_ns = last["timeline_ns"]
            self.last_edge = last["edge"]
            self.system_dac = last["system_dac"]
            self.segment_dac = last["segment_dac"]
            self.campaign = last["campaign"]
            self.segment_id = last["segment_id"]
        else:
            boundary = self.baseline["boundary"]
            self.timeline_ns = boundary["timeline_ns"]
            self.last_edge = boundary["edge"]
            self.campaign = boundary["campaign"]
            self.segment_id = boundary["segment_id"]
            self.clock.advance(self.timeline_ns, None, self.segment_id)
        self.latest = {
            "status": "reacquiring_exposure", "baseline": self.baseline,
            "segment_id": self.segment_id, "dac": self._dac_view(),
        }
        # No applied-voltage record exists across a process restart. Keep the
        # last edge to measure the gap, but never bridge it with a DAC code.
        self.last_edge = {**self.last_edge, "qualified": False}

    def stage_slope(self, slopes):
        with self.lock:
            candidate = {
                **self.config,
                "calibration": {"source": "operator", "slope_micro_ppb_per_code": slopes},
                "operation_id": uuid4().hex,
            }
            _write_config(candidate)
            self.config = candidate
            return candidate["calibration"]

    def request_baseline(self, source, saved, request_id):
        with self.lock:
            if self.baseline and self.baseline["id"] == request_id:
                return {"status": "committed", "baseline": self.baseline}
            if self.request:
                if self.request["request_id"] != request_id and self.request["source"] != source:
                    raise ValueError("A DAC baseline capture is already pending")
                return {"status": "pending", "request_id": self.request["request_id"]}
            if saved is None and not self.system_dac["n"]:
                raise ValueError("No paired SYSTEM DAC MEAN yet")
            self.request = {
                "source": source, "saved": saved, "request_id": request_id,
                "requested_at_ns": monotonic_ns(),
            }
            self.last_request = {"status": "pending", "request_id": request_id}
            return self.last_request

    def _install_baseline(self, edge, sequence, campaign):
        request = self.request
        saved = request["saved"]
        if saved is None:
            means = dac_population_means(self.system_dac)
            if means is None:
                return False
            saved = means, self.system_dac["n"], "applied_exposure"
        means, count, mean_basis = saved
        calibration = self.config.get("calibration") or {
            "source": "provisional_controller_estimate",
            "slope_micro_ppb_per_code": {
                lane: PROVISIONAL_SLOPE_MICRO_PPB_PER_CODE for lane in LANES
            },
        }
        segment_id = uuid4().hex if campaign else None
        baseline = {
            "id": request["request_id"],
            "boot_id": self.boot_id,
            "source": request["source"],
            "source_n": count,
            "mean_basis": mean_basis,
            "calibration": calibration,
            "effective_at_utc": datetime.now(timezone.utc).isoformat(),
            "boundary": {
                "sequence": sequence, "timeline_ns": self.timeline_ns, "edge": edge,
                "campaign": campaign, "segment_id": segment_id,
            },
        }
        for lane in LANES:
            baseline[lane] = {
                "code_nano": means[lane],
                "voltage_nv": code_nano_to_voltage_nv(means[lane]),
                "slope_micro_ppb_per_code": calibration["slope_micro_ppb_per_code"][lane],
            }
        config = {
            **self.config, "baseline": baseline, "rebaseline_required": False,
            "operation_id": request["request_id"],
        }
        _write_config(config)
        self.config = config
        self.baseline = baseline
        self.clock = VoltageClock(baseline)
        self.system_dac = empty_dac_population()
        self.segment_dac = empty_dac_population()
        self.campaign = campaign
        self.segment_id = segment_id
        self.last_request = {"status": "committed", "baseline": baseline}
        self.request = None
        return True

    def reset_statistics(self):
        with self.lock:
            self.system_dac = empty_dac_population()
            if self.clock:
                self.clock.reset_statistics()

    def _set_campaign(self, campaign):
        if campaign != self.campaign:
            self.campaign = campaign
            self.segment_id = uuid4().hex if campaign else None
            self.segment_dac = empty_dac_population()

    def _interrupt_power(self):
        self.rail_interrupted = True
        self.system_dac = empty_dac_population()
        self.segment_dac = empty_dac_population()
        self.campaign = None
        self.segment_id = None
        self.baseline = None
        self.clock = None
        if self.request:
            self.last_request = {"status": "power_interrupted", "request_id": self.request["request_id"]}
            self.request = None
        if self.config.get("baseline") is not None:
            config = {
                **self.config, "baseline": None, "rebaseline_required": True,
                "operation_id": uuid4().hex,
            }
            _write_config(config)
            self.config = config

    def consume(
        self, *, gnss_ns, sequence, ingress_ns, qualified, power_interrupted,
        campaign, campaign_active, servo_mode,
    ):
        with self.lock:
            receipt_ns = ingress_ns - RECEIPT_DELAY_NS
            if power_interrupted and (not self.rail_interrupted or self.config.get("baseline")):
                self._interrupt_power()
                if self.last_edge:
                    self.last_edge = {**self.last_edge, "qualified": False}
                self.last_exposure = None
                self.latest = {"status": "rebaseline_required", "dac": self._dac_view()}
            if campaign is None and campaign_active:
                campaign = self.campaign
            previous = self.last_edge
            if previous and receipt_ns <= previous["receipt_ns"]:
                return self.latest, None, None
            if previous and sequence == previous["sequence"] and gnss_ns == previous["gnss_ns"]:
                # Beta can strengthen the same instrument fragment with a
                # campaign attachment. Preserve the first arrival's exposure.
                if campaign and campaign != self.campaign:
                    self._set_campaign(campaign)
                    self.last_edge = {**previous, "campaign": campaign}
                    self.latest = {
                        **self.latest, "segment_id": self.segment_id, "dac": self._dac_view(),
                    }
                    if self.clock:
                        self.clock.set_campaign(self.segment_id)
                        self.latest.update(self.clock.snapshot())
                return self.latest, self.last_exposure, self._checkpoint()
            self.last_edge = {
                "receipt_ns": receipt_ns, "gnss_ns": gnss_ns, "sequence": sequence,
                "qualified": qualified and not power_interrupted, "campaign": campaign,
            }
            exposure = None
            if previous:
                host_ns = receipt_ns - previous["receipt_ns"]
                reference_ns = gnss_ns - previous["gnss_ns"]
                sequence_delta = sequence - previous["sequence"]
                same_timeline = sequence_delta > 0 and reference_ns == sequence_delta * NS_PER_SECOND
                adjacent = same_timeline and sequence_delta == 1
                self.timeline_ns += reference_ns if same_timeline else max(
                    1, round_ratio(host_ns, NS_PER_SECOND)
                ) * NS_PER_SECOND
                if (adjacent and previous["qualified"] and self.last_edge["qualified"]
                        and abs(host_ns - reference_ns) <= MAX_INTERVAL_SKEW_NS):
                    means = self.exposure.paired_means(previous["receipt_ns"], receipt_ns)
                    if all(value is not None for value in means.values()):
                        exposure = {
                            "schema": "CLOCKS_DAC_EXPOSURE_V1",
                            "reference_ns": reference_ns,
                            "host_interval_ns": host_ns,
                            "receipt_delay_ns": RECEIPT_DELAY_NS,
                            "time_basis": "pi_fragment_receipt",
                            "servo_mode": servo_mode,
                            "campaign": campaign if campaign == previous["campaign"] else None,
                            **{f"{lane}_code_nano": means[lane] for lane in LANES},
                        }
                        add_exposure(self.system_dac, exposure)

            cut = False
            if (self.request and exposure is not None
                    and receipt_ns >= self.request["requested_at_ns"]):
                try:
                    cut = self._install_baseline(self.last_edge, sequence, campaign)
                except Exception:
                    logging.exception("CLOCKS DAC baseline commit failed")
                    self.last_request = {"status": "commit_failed", "request_id": self.request["request_id"]}
                    self.request = None
                if cut:
                    self.rail_interrupted = False

            changed_campaign = campaign != self.campaign
            self._set_campaign(campaign)
            if exposure and campaign and exposure["campaign"] == campaign and not cut and not changed_campaign:
                add_exposure(self.segment_dac, exposure)
            reporting = {
                "status": "awaiting_baseline", "segment_id": self.segment_id,
                "as_of_utc": datetime.now(timezone.utc).isoformat(),
            }
            if self.clock:
                applied = None
                if exposure and not cut:
                    applied = {lane: exposure[f"{lane}_code_nano"] for lane in LANES}
                reporting.update(self.clock.advance(self.timeline_ns, applied, self.segment_id))
                reporting["status"] = "baseline_installed" if cut else (
                    "tracking" if applied else "exposure_gap"
                )
                reporting["baseline"] = self.baseline
            elif self.rail_interrupted or self.config.get("baseline"):
                reporting["status"] = "rebaseline_required"
            reporting["dac"] = self._dac_view()
            self.latest = reporting
            self.last_exposure = exposure
            return reporting, exposure, self._checkpoint()

    def _checkpoint(self):
        # Each row owns immutable aggregates while the next interval advances.
        return {
            "boot_id": self.boot_id,
            "baseline_id": self.baseline["id"] if self.baseline else None,
            "clock": self.clock.checkpoint() if self.clock else None,
            "edge": self.last_edge,
            "timeline_ns": self.timeline_ns,
            "system_dac": self._copy_population(self.system_dac),
            "segment_dac": self._copy_population(self.segment_dac),
            "campaign": self.campaign,
            "segment_id": self.segment_id,
        }

    @staticmethod
    def _copy_population(population):
        return {key: value.copy() if isinstance(value, dict) else value
                for key, value in population.items()}

    def _dac_view(self):
        system = dac_population_means(self.system_dac) or {}
        campaign = dac_population_means(self.segment_dac) or {}
        result = {}
        count = self.system_dac["n"]
        for lane in LANES:
            values = {
                "system_n": count,
                "campaign_n": self.segment_dac["n"],
            }
            if count:
                # Every admitted GNSS interval is one second.
                total = self.system_dac["code_ns"][lane] // NS_PER_SECOND
                squares = self.system_dac["sum_squares"][lane]
                variance = count * squares - total * total
                denominator = count * (count - 1)
                values.update({
                    "system_min_code_nano": self.system_dac["minimum"][lane],
                    "system_max_code_nano": self.system_dac["maximum"][lane],
                    "system_stddev_code_nano": isqrt(variance // denominator) if count > 1 else 0,
                    "system_stderr_code_nano": isqrt(variance // (denominator * count)) if count > 1 else 0,
                })
            if self.baseline:
                values["baseline_code_nano"] = self.baseline[lane]["code_nano"]
                values["baseline_nv"] = self.baseline[lane]["voltage_nv"]
            for scope, means in (("system", system), ("campaign", campaign)):
                if lane in means:
                    values[f"{scope}_mean_code_nano"] = means[lane]
                    values[f"{scope}_mean_nv"] = code_nano_to_voltage_nv(means[lane])
                    if self.baseline:
                        values[f"{scope}_delta_nv"] = code_nano_to_voltage_nv(
                            means[lane] - self.baseline[lane]["code_nano"]
                        )
            result[lane] = values
        return result

    def info(self):
        with self.lock:
            return {
                "reporting": self.latest,
                "baseline_request": self.last_request,
                "staged_calibration": self.config.get("calibration"),
            }
