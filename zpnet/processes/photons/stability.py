"""Pi PHOTONS block stability, exact one-second continuation, no race replay.

Each PHOTONS_V1 row contains both a small operator summary and this calculator's
complete bounded checkpoint. Inserting that one row in campaign_detail commits the
measurement and its statistical continuation together. A process restart restores
from the same producer-authoritative canonical row chosen by PHOTONS recovery;
there is no second independently advancing persistence cursor or shadow database.

A block is a *disjoint campaign-local interval* with exactly its configured
number of one-second observations. If a producer sequence, campaign public count,
or publication clock jumps, the open intervals are invalidated. Previously
completed blocks remain evidence and the next full, grid-aligned block can enter
its own statistics. Different acquisition or Pi normalization identities create
separate analysis epochs; the raw one-second measurements remain immutable.

Block SD describes completed block means, not race jitter or campaign mean
uncertainty. Adjacent-block deviation is Allan-style descriptive evidence;
no assertion of stationarity or sub-100 ps reproducibility is made here.
"""

from __future__ import annotations

import copy
import math
from datetime import datetime

SCHEMA = "PI_PHOTONS_STABILITY_V1"
WINDOWS = (60, 600, 3600, 14400, 28800, 86400)
TIMING_GAP_SECONDS = 3.0


def _time(value):
    return datetime.fromisoformat(str(value).replace("Z", "+00:00")).timestamp()


def _population():
    return {
        "n": 0, "mean_ns": 0.0, "m2_ns2": 0.0,
        "minimum_ns": None, "maximum_ns": None,
        "previous_index": None, "previous_mean_ns": None,
        "last_completed_index": None,
        "allan_pairs": 0, "allan_sum_sq_ns2": 0.0,
        "x_mean": 0.0, "sxx": 0.0, "sxy": 0.0,
    }


def _window(seconds):
    return {
        "seconds": seconds, "open": None,
        "raw": _population(), "normalized": _population(),
        "discarded_partial_blocks": 0,
    }


def _append(pop, index, mean):
    n = pop["n"] + 1
    dy = mean - pop["mean_ns"]
    updated_mean = pop["mean_ns"] + dy / n
    pop["m2_ns2"] += dy * (mean - updated_mean)
    pop["mean_ns"] = updated_mean
    pop["minimum_ns"] = mean if pop["minimum_ns"] is None else min(pop["minimum_ns"], mean)
    pop["maximum_ns"] = mean if pop["maximum_ns"] is None else max(pop["maximum_ns"], mean)
    # Online covariance between exact campaign-aligned block index and its mean.
    dx = index - pop["x_mean"]
    new_x = pop["x_mean"] + dx / n
    pop["sxx"] += dx * (index - new_x)
    pop["sxy"] += dx * (mean - updated_mean)
    pop["x_mean"] = new_x
    if pop["previous_index"] is not None and index == pop["previous_index"] + 1:
        pop["allan_pairs"] += 1
        pop["allan_sum_sq_ns2"] += (mean - pop["previous_mean_ns"]) ** 2
    pop["previous_index"] = index
    pop["previous_mean_ns"] = mean
    pop["last_completed_index"] = index
    pop["n"] = n


def _window_summary(window):
    result = {"seconds": window["seconds"],
              "discarded_partial_blocks": window["discarded_partial_blocks"]}
    opened = window["open"]
    for name in ("raw", "normalized"):
        pop = window[name]
        n = pop["n"]
        active_n = opened[name + "_n"] if opened is not None else 0
        active_mean = opened[name + "_mean_ns"] if active_n else None
        result[name] = {
            "block_count": n,
            "current_block_mean_ns": active_mean,
            "current_block_observed_seconds": opened["slots"] if opened else 0,
            "current_block_accepted_races": active_n,
            "completed_mean_ns": pop["mean_ns"] if n else None,
            "latest_completed_mean_ns": (pop["mean_ns"] if n else None),
            "last_completed_start_second": (pop["last_completed_index"] * window["seconds"] + 1
                                            if pop["last_completed_index"] is not None else None),
            "last_completed_end_second": ((pop["last_completed_index"] + 1) * window["seconds"]
                                          if pop["last_completed_index"] is not None else None),
            "block_sd_ps": (1000.0 * math.sqrt(max(0.0, pop["m2_ns2"] / (n - 1)))
                            if n > 1 else None),
            "allan_ps": (1000.0 * math.sqrt(pop["allan_sum_sq_ns2"] / (2 * pop["allan_pairs"]))
                         if pop["allan_pairs"] else None),
            "trend_ps_per_hour": (1000.0 * 3600.0 * pop["sxy"] / (pop["sxx"] * window["seconds"])
                                  if n >= 3 and pop["sxx"] > 0 else None),
            "uncertainty_ps": None,  # Correlation-aware campaign uncertainty not yet established.
        }
    return result


def _add_second(open_block, name, flight):
    if not isinstance(flight, dict) or flight.get("n", 0) == 0:
        return
    n = int(flight["n"])
    value = float(flight["mean"])
    if n <= 0 or not math.isfinite(value):
        raise ValueError("PHOTONS stability requires finite positive-count mean")
    prior = open_block[name + "_n"]
    total = prior + n
    open_block[name + "_mean_ns"] += (value - open_block[name + "_mean_ns"]) * n / total
    open_block[name + "_n"] = total
    open_block[name + "_slots"] += 1


class StabilityAccumulator:
    """Single state-worker owner; carry last committed snapshot across Pi restart."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._state = {"schema": SCHEMA, "sequence": None, "published_at_utc": None,
                       "campaign_id": None, "campaign": None, "public_count": None,
                       "acquisition_id": None, "normalization_id": None,
                       "complete_from_start": False, "gap_count": 0,
                       "windows": {str(s): _window(s) for s in WINDOWS}}

    def restore(self, canonical):
        """Restore from one exact durable PHOTONS source, never from older convenient history."""
        self.reset()
        if canonical is None:
            return
        sequence = int(canonical["sequence"])
        campaign = canonical.get("campaign") or {}
        source_id = campaign.get("campaign_id")
        public_count = campaign.get("public_count")
        measurement = canonical["measurement"]
        acquisition_id = measurement["acquisition_id"]
        normalization_id = measurement["normalization_id"]
        existing = canonical.get("stability")
        if existing is not None:
            if existing["schema"] != SCHEMA:
                raise ValueError("unsupported durable PHOTONS stability schema")
            state = copy.deepcopy(existing["checkpoint"])
            if (state["schema"] != SCHEMA or state["sequence"] != sequence
                    or state["campaign_id"] != source_id
                    or state["campaign"] != campaign.get("campaign")
                    or state["public_count"] != public_count
                    or state["acquisition_id"] != acquisition_id
                    or state["normalization_id"] != normalization_id
                    or set(state["windows"]) != {str(x) for x in WINDOWS}):
                raise ValueError("durable PHOTONS stability checkpoint disagrees with source")
            self._state = state
            return
        # Pre-stability canonical history has no checkpoint. Never claim an old
        # campaign's first N seconds were witnessed by the new calculator.
        self._state.update({
            "sequence": sequence, "published_at_utc": canonical["published_at_utc"],
            "campaign_id": source_id, "campaign": campaign.get("campaign"),
            "public_count": public_count, "acquisition_id": acquisition_id,
            "normalization_id": normalization_id,
            "complete_from_start": False,
        })

    def _new_epoch(self, campaign, measurement, prior, *, first):
        self.reset()
        self._state.update({
            "campaign_id": campaign["campaign_id"], "campaign": campaign["campaign"],
            "acquisition_id": measurement["acquisition_id"],
            "normalization_id": measurement["normalization_id"],
            "complete_from_start": bool(first),
            "gap_count": 0,  # A new campaign/model epoch has its own gap ledger.
        })

    def ingest(self, photons):
        # Rejected input must not partially advance the live checkpoint. The
        # state worker only publishes a returned result after this transaction.
        before = self._state
        self._state = copy.deepcopy(before)
        try:
            return self._ingest(photons)
        except Exception:
            self._state = before
            raise

    def _ingest(self, photons):
        s = self._state
        sequence = int(photons["sequence"])
        measurement = photons["measurement"]
        campaign = photons.get("campaign") or {}
        stamp = photons["published_at_utc"]
        if s["sequence"] is not None and sequence <= s["sequence"]:
            raise ValueError("PHOTONS stability sequence did not advance")

        campaign_id = campaign.get("campaign_id")
        public_count = campaign.get("public_count")
        if campaign_id is None:
            # No campaign owns the current observation; avoid reusing a completed
            # campaign's partial block in an unrelated future START.
            self.reset()
            s = self._state
            s.update({"sequence": sequence, "published_at_utc": stamp,
                      "acquisition_id": measurement["acquisition_id"],
                      "normalization_id": measurement["normalization_id"]})
            return self._surface()

        if not isinstance(public_count, int) or public_count <= 0:
            raise ValueError("PHOTONS stability requires positive campaign public_count")
        fresh = (s["campaign_id"] != campaign_id
                 or s["acquisition_id"] != measurement["acquisition_id"]
                 or s["normalization_id"] != measurement["normalization_id"])
        if fresh:
            self._new_epoch(campaign, measurement, s, first=public_count == 1)
            s = self._state
        else:
            prior_public = s["public_count"]
            consistent = (prior_public is not None
                          and public_count == prior_public + 1
                          and (s["sequence"] is None or sequence == s["sequence"] + 1)
                          and (s["published_at_utc"] is None
                               or 0.0 < _time(stamp) - _time(s["published_at_utc"]) <= TIMING_GAP_SECONDS))
            if not consistent:
                s["gap_count"] += 1
                s["complete_from_start"] = False
                for window in s["windows"].values():
                    if window["open"] is not None:
                        window["discarded_partial_blocks"] += 1
                    window["open"] = None
                    for name in ("raw", "normalized"):
                        window[name]["previous_index"] = None
                        window[name]["previous_mean_ns"] = None

        raw = measurement["raw_flight_ns"]
        normalized = measurement["normalized_flight_ns"]
        for seconds in WINDOWS:
            window = s["windows"][str(seconds)]
            index = (public_count - 1) // seconds
            position = (public_count - 1) % seconds + 1
            opened = window["open"]
            if opened is None or opened["index"] != index:
                if opened is not None:
                    window["discarded_partial_blocks"] += 1
                    for name in ("raw", "normalized"):
                        window[name]["previous_index"] = None
                        window[name]["previous_mean_ns"] = None
                opened = {"index": index, "slots": 0,
                          "first_position": position,
                          "raw_n": 0, "raw_mean_ns": 0.0, "raw_slots": 0,
                          "normalized_n": 0, "normalized_mean_ns": 0.0,
                          "normalized_slots": 0}
                window["open"] = opened
            opened["slots"] += 1
            _add_second(opened, "raw", raw)
            _add_second(opened, "normalized", normalized)
            if position == seconds:
                complete = (opened["first_position"] == 1 and opened["slots"] == seconds)
                if complete and opened["raw_slots"] == seconds:
                    _append(window["raw"], index, opened["raw_mean_ns"])
                else:
                    window["raw"]["previous_index"] = None
                    window["raw"]["previous_mean_ns"] = None
                if complete and opened["normalized_slots"] == seconds:
                    _append(window["normalized"], index, opened["normalized_mean_ns"])
                else:
                    window["normalized"]["previous_index"] = None
                    window["normalized"]["previous_mean_ns"] = None
                if not complete or opened["raw_slots"] != seconds:
                    window["discarded_partial_blocks"] += 1
                window["open"] = None

        s.update({"sequence": sequence, "published_at_utc": stamp,
                  "public_count": public_count})
        return self._surface()

    def _surface(self):
        s = self._state
        return {"schema": SCHEMA, "status": ("ACTIVE" if s["campaign_id"] else "NO_CAMPAIGN"),
                "campaign": s["campaign"], "campaign_id": s["campaign_id"],
                "complete_from_start": s["complete_from_start"],
                "gap_count": s["gap_count"],
                "uncertainty_status": "NOT_ESTABLISHED_CORRELATED_DRIFT",
                "windows": {key: _window_summary(window) for key, window in s["windows"].items()},
                "checkpoint": copy.deepcopy(s)}
