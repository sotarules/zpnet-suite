"""Pi PHOTONS-owned physical temperature populations with one-row resurrection.

TOTAL and CAMP retain their original full-precision Welford (N, mean, M2)
state, committed atomically with the canonical one-second PHOTONS measurement.
A restart always recovers those exact populations from the PHOTONS Phase-5
selected durable row. There is no independent statistical clock and no SQL
replay or older-row lookup in temperature recovery.

For trailing windows, a Welford tuple alone cannot tell which old observation
must expire. This module instead follows PHOTONS/CLOCKS Better-Buckets: it keeps
an exact, 600-position one-second rolling tail for 10-MIN and a bounded ring of
minute-aligned *sufficient statistics* (count and integer temperature sum) for
the 60-MIN/8-HOUR/24-HOUR means. A partial minute is checkpointed separately.
Long windows advance only when the next COMPLETE calendar-free producer minute
closes; between closures their latest means remain unchanged, with up to one
minute of reporting latency. The one-minute grid is anchored to firmware's
statistics update_count, never a wall clock or an optional LANTERN campaign.

The rolling checkpoint uses signed microdegree-C integers (1e-6 C resolution),
a numeric precision finer than these physical sensors warrant. Quantization
applies only to the operational trailing bucket means. Full-precision physical
witnesses, TOTAL and CAMP Welfords, and every raw record are unchanged.
The encoded fixed-width rings are compressed, SHA256-checked, bounded, and
stored in the existing config.PHOTONS_RECOVERY singleton in the SAME transaction
as the canonical PHOTONS row. That singleton is already read and identity-checked
by Phase-5 recovery; adding temperature continuation creates no new SQL call.
The cumulative Welford image remains in the canonical record, while the large
rolling history is updated IN PLACE in the one existing recovery singleton.
This avoids multiplying database size by 86,400 checkpoint copies per day.

Migration from a pre-image V1 row preserves its TOTAL/CAMP continuation exactly,
but starts rolling buckets honestly partial. All future minute/second bins are
read from newly observed rows, never reconstructed or invented. A tilde in
Metrics identifies incomplete coverage. Each window's n describes actual
physical temperatures recorded, not independent experiments; SD is descriptive.
"""

from __future__ import annotations

import base64
import copy
import hashlib
import logging
import math
import struct
import zlib
from collections import deque
from dataclasses import dataclass

SCHEMA = "PI_PHOTONS_TEMPERATURE_STATS_V1"
ROLLING_SCHEMA = "PI_PHOTONS_TEMPERATURE_ROLLING_V2"
ROLLING_ENCODING = "LE_Q1E6_SECOND600_MINUTE1440_ZLIB_BASE64_V1"
SENSORS = ("pi", "pd", "out", "rtd")
WINDOWS = (600, 3600, 28800, 86400)
SHORT_LIMIT = 600
MINUTE_LIMIT = 1440
TEMPERATURE_SCALE = 1_000_000
MISSING = -(1 << 31)
# [producer sequence, firmware update_count, gap flag, four microdegree samples]
_SECOND = struct.Struct("<qqB4i")
# [minute index, producer observations, continuity breaks, four sensor sample
# counts, four microdegree temperature sums]. All minute counts fit uint8.
_MINUTE = struct.Struct("<qBB4B4q")
# Explicit delimiter ensures any future layout revision must change the schema.
_HEADER = struct.Struct("<III")   # seconds, completed minutes, active minute


def _quantized(values):
    out = []
    for name in SENSORS:
        value = values[name]
        if value is None:
            out.append(MISSING)
            continue
        q = round(float(value) * TEMPERATURE_SCALE)
        if not MISSING < q <= (1 << 31) - 1:
            raise ValueError(f"PHOTONS {name} temperature exceeds the bounded rolling representation")
        out.append(q)
    return tuple(out)


def _temperatures(q):
    return {name: None if value == MISSING else value / TEMPERATURE_SCALE
            for name, value in zip(SENSORS, q)}


def _new_minute(index):
    return {"index": index, "slots": 0, "gaps": 0,
            "counts": [0, 0, 0, 0], "sums": [0, 0, 0, 0]}


def _minute_append(minute, q, gap):
    minute["slots"] += 1
    minute["gaps"] += int(gap)
    if minute["slots"] > 60 or minute["gaps"] > 60:
        raise ValueError("PHOTONS thermal minute grid contains excess observations")
    for i, value in enumerate(q):
        if value != MISSING:
            minute["counts"][i] += 1
            minute["sums"][i] += value


def _minute_pack(minute):
    return _MINUTE.pack(minute["index"], minute["slots"], minute["gaps"],
                        *minute["counts"], *minute["sums"])


def _minute_unpack(values):
    index, slots, gaps, *tail = values
    counts, sums = list(tail[:4]), list(tail[4:])
    if not (0 <= slots <= 60 and 0 <= gaps <= slots
            and all(0 <= c <= slots for c in counts)
            and all(c != 0 or s == 0 for c,s in zip(counts,sums))):
        raise ValueError("PHOTONS rolling minute sufficient state is invalid")
    return {"index":index, "slots":slots,"gaps":gaps,"counts":counts,"sums":sums}

def _empty():
    return {"n": 0, "mean": 0.0, "m2": 0.0}


def _populations():
    return {name: _empty() for name in SENSORS}


def _add(p, value):
    n = p["n"] + 1
    delta = value - p["mean"]
    mean = p["mean"] + delta / n
    p["m2"] += delta * (value - mean)
    p["mean"] = mean
    p["n"] = n


def _remove(p, value):
    n = p["n"]
    if n == 1:
        p.update(_empty())
    else:
        mean = (p["mean"] * n - value) / (n - 1)
        p["m2"] = max(0.0, p["m2"] - (value - p["mean"]) * (value - mean))
        p["mean"] = mean
        p["n"] = n - 1


def _surface(p):
    n = p["n"]
    return {
        "n": n,
        "mean_c": p["mean"] if n else None,
        "sd_c": math.sqrt(max(0.0, p["m2"] / (n - 1))) if n > 1 else None,
    }


def _observations(environment, rtd):
    """No fallback paths: an absent/invalid physical sensor remains absent."""
    result = {}
    for name in SENSORS:
        node = rtd if name == "rtd" else environment.get(name)
        node = node or {}
        if node.get("status") != "OK":
            result[name] = None
            continue
        value = node.get("temperature_c")
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"PHOTONS {name} OK temperature is not finite numeric testimony")
        result[name] = float(value)
    return result


@dataclass
class _Window:
    seconds: int

    def __post_init__(self):
        self.entries = deque()
        self.populations = _populations()
        self.gaps = 0

    def append(self, sequence, values, gap):
        self.entries.append((sequence, values, bool(gap)))
        self.gaps += bool(gap)
        for name, value in values.items():
            if value is not None:
                _add(self.populations[name], value)
        while len(self.entries) > self.seconds:
            _, expired, was_gap = self.entries.popleft()
            self.gaps -= was_gap
            for name, value in expired.items():
                if value is not None:
                    _remove(self.populations[name], value)

    def summarize(self):
        # The first entry's gap flag refers to the predecessor OUTSIDE this
        # window. It must not contaminate an otherwise complete local window.
        interrupted = self.gaps - (int(self.entries[0][2]) if self.entries else 0)
        return {
            name: {**_surface(pop),
                   "complete": (len(self.entries) == self.seconds
                                and interrupted == 0 and pop["n"] == self.seconds)}
            for name, pop in self.populations.items()
        }


class TemperatureAccumulator:
    """One PHOTONS state-worker owner; immutable checkpoint in the source row."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = {
            "schema": SCHEMA, "sequence": None, "reset_count": None,
            "campaign_id": None, "campaign": None, "public_count": None,
            "total_complete": True, "campaign_complete": False,
            "gap_count": 0, "total_seconds": 0, "camp_seconds": 0,
            "total": _populations(), "camp": _populations(),
        }
        self._clear_rolling()

    def _clear_rolling(self):
        # All four windows share this ONE minute ring and recent-second tail.
        self.short = _Window(SHORT_LIMIT)
        self.second_tail = deque(maxlen=SHORT_LIMIT)
        self.minutes = deque(maxlen=MINUTE_LIMIT)
        self.open_minute = None
        self.last_rolling_sequence = None
        self.last_rolling_update_count = None
        self.last_rolling_reset_count = None

    def _append_rolling(self, sequence, update_count, reset_count, values):
        if (self.last_rolling_sequence is not None and
                (reset_count != self.last_rolling_reset_count
                 or sequence <= self.last_rolling_sequence
                 or update_count <= self.last_rolling_update_count)):
            # Physical stats reset/source cutover is a distinct rolling epoch.
            self._clear_rolling()
        gap = (self.last_rolling_sequence is not None and
               (sequence != self.last_rolling_sequence + 1
                or update_count != self.last_rolling_update_count + 1))
        q = _quantized(values)
        entry = (sequence, update_count, int(gap), q)
        self.second_tail.append(entry)
        self.short.append(sequence, _temperatures(q), gap)

        minute_index = (update_count - 1) // 60
        if self.open_minute is None:
            self.open_minute = _new_minute(minute_index)
        elif minute_index != self.open_minute["index"]:
            if minute_index <= self.open_minute["index"]:
                raise ValueError("PHOTONS temperature minute index moved backward")
            # A producer gap can skip some or all of a minute; no synthetic
            # complete interval is invented for the missing observations.
            self.minutes.append(self.open_minute)
            self.open_minute = _new_minute(minute_index)
        _minute_append(self.open_minute, q, gap)
        if update_count % 60 == 0:
            self.minutes.append(self.open_minute)
            self.open_minute = None

        self.last_rolling_sequence = sequence
        self.last_rolling_update_count = update_count
        self.last_rolling_reset_count = reset_count

    def _minute_surface(self, span):
        """Most recent completed producer-minute grid, not a 1-Hz window.

        A mean may be available even when the window has not yet accumulated
        all 60/480/1440 complete minutes; the resulting incomplete flag is
        crucial testimony. Missing sensor values reduce N, never interpolate.
        A gap between physically neighboring complete minutes is represented
        by a gap in their producer indices or by an interruption in one minute.
        """
        count = span // 60
        current_update = self.last_rolling_update_count or 0
        end_index = ((current_update - 1) // 60
                     - (1 if current_update % 60 else 0))
        # A producer-minute becomes available on its sixtieth sample. A
        # partially acquired next minute does not replace that endpoint.
        start_index = end_index - count + 1
        selected = [m for m in self.minutes if start_index <= m["index"] <= end_index]
        complete_grid = (len(selected) == count
                         and all(m["index"] == start_index+i and m["slots"] == 60
                                 and m["gaps"] == 0 for i,m in enumerate(selected)))
        out = {}
        for i, name in enumerate(SENSORS):
            n = sum(m["counts"][i] for m in selected)
            total_q = sum(m["sums"][i] for m in selected)
            out[name] = {"n": n,
                         "mean_c": total_q / (n*TEMPERATURE_SCALE) if n else None,
                         "sd_c": None,
                         "complete": complete_grid and n == span}
        return out

    def _rolling_image(self):
        """A single bounded compressed image, saved with each PHOTONS row.

        No separate WAL, no retrieval of historical rows. Current sampling
        float64 source observations remain in the PHOTONS payload; only this
        ephemeral operational rolling-cache image uses 1e-6 C quantization.
        """
        n_second = len(self.second_tail)
        n_minute = len(self.minutes)
        n_open = int(self.open_minute is not None)
        packed = bytearray(_HEADER.pack(n_second, n_minute, n_open))
        for sequence, update_count, gap, q in self.second_tail:
            packed.extend(_SECOND.pack(sequence, update_count, gap, *q))
        for minute in self.minutes:
            packed.extend(_minute_pack(minute))
        if self.open_minute is not None:
            packed.extend(_minute_pack(self.open_minute))
        compressed = zlib.compress(packed, level=4)
        return {"schema": ROLLING_SCHEMA, "encoding": ROLLING_ENCODING,
                "sequence": self.last_rolling_sequence,
                "update_count": self.last_rolling_update_count,
                "reset_count": self.last_rolling_reset_count,
                "sha256": hashlib.sha256(packed).hexdigest(),
                "image": base64.b64encode(compressed).decode("ascii"),
                "short_population": copy.deepcopy(self.short.populations),
                "short_gaps": self.short.gaps}

    def _restore_rolling(self, image, canonical):
        if image["schema"] != ROLLING_SCHEMA or image["encoding"] != ROLLING_ENCODING:
            raise ValueError("unsupported PHOTONS temperature rolling checkpoint")
        sequence = int(canonical["sequence"])
        stats = canonical["photons"]["stats"]
        update_count = int(stats["update_count"])
        reset_count = int(stats["reset_count"])
        if (image["sequence"] != sequence or image["update_count"] != update_count
                or image["reset_count"] != reset_count):
            raise ValueError("temperature rolling checkpoint does not match selected source")
        compressed = base64.b64decode(image["image"], validate=True)
        max_size = (_HEADER.size + SHORT_LIMIT*_SECOND.size
                    + (MINUTE_LIMIT + 1)*_MINUTE.size)
        inflater = zlib.decompressobj()
        raw = inflater.decompress(compressed, max_size + 1)
        if (len(raw) > max_size or not inflater.eof or inflater.unused_data
                or inflater.unconsumed_tail):
            raise ValueError("PHOTONS temperature rolling image exceeds fixed bounds")
        if hashlib.sha256(raw).hexdigest() != image["sha256"]:
            raise ValueError("PHOTONS temperature rolling checkpoint checksum mismatch")
        if len(raw) < _HEADER.size:
            raise ValueError("PHOTONS temperature rolling checkpoint has no header")
        n_second, n_minute, n_open = _HEADER.unpack_from(raw)
        expected_size = (_HEADER.size + n_second*_SECOND.size
                         + (n_minute+n_open)*_MINUTE.size)
        if (n_second > SHORT_LIMIT or n_minute > MINUTE_LIMIT or n_open > 1
                or len(raw) != expected_size):
            raise ValueError("PHOTONS temperature rolling image dimensions are invalid")
        offset = _HEADER.size
        prev_seq = prev_update = None
        for _ in range(n_second):
            s, u, gap, *q = _SECOND.unpack_from(raw, offset)
            offset += _SECOND.size
            if gap not in (0, 1) or u <= 0:
                raise ValueError("PHOTONS rolling second record is malformed")
            if prev_seq is not None and (s <= prev_seq or u <= prev_update
                   or gap != int(s != prev_seq+1 or u != prev_update+1)):
                raise ValueError("PHOTONS rolling second continuity evidence disagrees")
            value = _temperatures(q)
            self.second_tail.append((s, u, gap, tuple(q)))
            self.short.append(s, value, bool(gap))
            prev_seq, prev_update = s,u
        if not self.second_tail or self.second_tail[-1][:2] != (sequence, update_count):
            raise ValueError("PHOTONS rolling second tail does not reach source")
        if tuple(self.second_tail[-1][3]) != _quantized(_observations(
                canonical.get("environment") or {}, canonical.get("rtd") or {})):
            raise ValueError("PHOTONS rolling last temperature disagrees with canonical source")
        prev_index = None
        for _ in range(n_minute):
            minute = _minute_unpack(_MINUTE.unpack_from(raw, offset))
            offset += _MINUTE.size
            if prev_index is not None and minute["index"] <= prev_index:
                raise ValueError("PHOTONS completed temperature minutes are not ordered")
            self.minutes.append(minute)
            prev_index = minute["index"]
        if n_open:
            self.open_minute = _minute_unpack(_MINUTE.unpack_from(raw, offset))
            if prev_index is not None and self.open_minute["index"] <= prev_index:
                raise ValueError("PHOTONS open temperature minute precedes completed minute")
        current_minute = (update_count - 1)//60
        if (self.open_minute is None) != (update_count % 60 == 0):
            raise ValueError("PHOTONS rolling open-minute presence disagrees with source")
        if self.open_minute is not None and self.open_minute["index"] != current_minute:
            raise ValueError("PHOTONS rolling open-minute index disagrees with source")
        if self.open_minute is None and (not self.minutes or self.minutes[-1]["index"] != current_minute):
            raise ValueError("PHOTONS rolling most recent completed minute disagrees")
        saved_pop = image["short_population"]
        if set(saved_pop) != set(SENSORS) or image["short_gaps"] != self.short.gaps:
            raise ValueError("PHOTONS 10-minute population checkpoint disagrees")
        for name in SENSORS:
            p = saved_pop[name]
            if (set(p) != {"n", "mean", "m2"}
                    or p["n"] != sum(e[3][SENSORS.index(name)] != MISSING
                                      for e in self.second_tail)
                    or not math.isfinite(p["mean"]) or not math.isfinite(p["m2"])
                    or p["m2"] < -1e-9):
                raise ValueError("PHOTONS 10-minute Welford checkpoint is invalid")
        self.short.populations = copy.deepcopy(saved_pop)
        self.last_rolling_sequence = sequence
        self.last_rolling_update_count = update_count
        self.last_rolling_reset_count = reset_count
        logging.info(
            "[photons/temperature] recovered rolling checkpoints from canonical "
            "sequence=%d update=%d seconds=%d minute_bins=%d image_bytes=%d; no SQL",
            sequence, update_count, n_second, n_minute, len(compressed))

    def rolling_checkpoint(self):
        """Snapshot the same one-second position as our cumulative checkpoint.

        This object travels ONLY through the ordered persistence queue to the
        existing singleton; it is not copied into published/canonical PHOTONS.
        The state worker must call it immediately after ingest, before any next
        observed fragment advances the accumulator.
        """
        return self._rolling_image()

    def restore(self, canonical, rolling_checkpoint=None):
        """Resume from the PHOTONS Phase-5 canonical + existing recovery singleton.

        Both source objects were atomically committed together, and the
        ordinary producer recovery court has already established their common
        reset_count/update_count identity. We never run historical SQL here.
        """
        self.reset()
        if canonical is None:
            return
        sequence = int(canonical["sequence"])
        stats = canonical["photons"]["stats"]
        reset_count = int(stats["reset_count"])
        update_count = int(stats["update_count"])
        campaign = canonical.get("campaign") or {}
        existing = canonical.get("temperature_stats")
        if existing is None:
            self.state.update({
                "sequence": sequence, "reset_count": reset_count,
                "campaign_id": campaign.get("campaign_id"),
                "campaign": campaign.get("campaign"),
                "public_count": campaign.get("public_count"),
                "total_complete": False, "campaign_complete": False})
        else:
            if existing["schema"] != SCHEMA:
                raise ValueError("unknown durable PHOTONS temperature statistics schema")
            state = copy.deepcopy(existing["checkpoint"])
            image = state.pop("rolling", None)
            # A short-lived development revision previously stored the whole
            # rolling image inside canonical temperature_stats.checkpoint.
            # Preserve its ability to recover if such a row is encountered.
            if image is None:
                image = rolling_checkpoint
            # The physical I2C2 sensor moved from the OTDR interior to ambient
            # outside the case. Its OLD 'otdr' observations are not measurements
            # of the new 'out' environment. Nor can we reinterpret the positional
            # rolling image: it has no sensor-role identifier.
            # Preserve the canonical old checkpoint untouched, but begin a new
            # explicitly partial temperature epoch from this producer cursor.
            # This is a legitimate sensor-role cutover, not missing data and not
            # a reason to reject the otherwise valid PHOTONS recovery source.
            if (set(state["total"]) == {"pi", "pd", "otdr", "rtd"}
                    and set(state["camp"]) == {"pi", "pd", "otdr", "rtd"}):
                if (state["schema"] != SCHEMA or state["sequence"] != sequence
                        or state["reset_count"] != reset_count
                        or state["campaign_id"] != campaign.get("campaign_id")
                        or state["public_count"] != campaign.get("public_count")
                        or state["campaign"] != campaign.get("campaign")
                        or not isinstance(state["total_seconds"], int)
                        or not isinstance(state["camp_seconds"], int)):
                    raise ValueError("legacy OTDR temperature checkpoint disagrees with recovery source")
                self.state.update({
                    "sequence": sequence, "reset_count": reset_count,
                    "campaign_id": campaign.get("campaign_id"),
                    "campaign": campaign.get("campaign"),
                    "public_count": campaign.get("public_count"),
                    "total_complete": False, "campaign_complete": False,
                })
                self.last_rolling_sequence = sequence
                self.last_rolling_update_count = update_count
                self.last_rolling_reset_count = reset_count
                logging.info(
                    "[photons/temperature] OTDR -> OUT physical-role cutover "
                    "at durable sequence=%d; prior temperature statistics "
                    "remain historical; new populations start partial", sequence,
                )
                return
            if (state["schema"] != SCHEMA or state["sequence"] != sequence
                    or state["reset_count"] != reset_count
                    or state["campaign_id"] != campaign.get("campaign_id")
                    or state["public_count"] != campaign.get("public_count")
                    or state["campaign"] != campaign.get("campaign")
                    or set(state["total"]) != set(SENSORS)
                    or set(state["camp"]) != set(SENSORS)
                    or not isinstance(state["total_seconds"], int)
                    or not isinstance(state["camp_seconds"], int)):
                raise ValueError("PHOTONS temperature checkpoint disagrees with recovery source")
            self.state = state
            if image is not None:
                self._restore_rolling(image, canonical)
                return
        # Legacy checkpoint migration: cumulative source state is valid,
        # but earlier rolling observation history is not contained within it.
        # Mark future windows partial until ordinary new observations fill them.
        self.last_rolling_sequence = sequence
        self.last_rolling_update_count = update_count
        self.last_rolling_reset_count = reset_count
        logging.info("[photons/temperature] selected canonical has no rolling image; "
                     "TOTAL/CAMP restored, rolling windows start partial (no SQL)")

    def preflight(self, canonical):
        """Validate before ANY accumulator (including stability) advances.

        PHOTONS' producer and stability calculations are stateful. A bad SYSTEM
        temperature must not be discovered only after another one-second
        accumulator has accepted that same ultimately rejected observation.
        """
        sequence = int(canonical["sequence"])
        stats = canonical["photons"]["stats"]
        reset_count = int(stats["reset_count"])
        update_count = int(stats["update_count"])
        campaign = canonical.get("campaign") or {}
        campaign_id = campaign.get("campaign_id")
        public_count = campaign.get("public_count")
        values = _observations(canonical.get("environment") or {}, canonical.get("rtd") or {})
        if campaign_id is not None and (not isinstance(public_count, int) or public_count <= 0):
            raise ValueError("invalid PHOTONS temperature campaign public count")
        s = self.state
        if s["sequence"] is not None and s["reset_count"] == reset_count and sequence <= s["sequence"]:
            raise ValueError("PHOTONS temperature sequence did not advance")
        # Screen sample values for deterministic bounded checkpoint encoding
        # before ANY other stateful subsystem consumes the new canonical row.
        _quantized(values)
        return sequence, reset_count, update_count, campaign, campaign_id, public_count, values

    def ingest(self, canonical, *, prepared=None):
        (sequence, reset_count, update_count, campaign, campaign_id,
         public_count, values) = prepared if prepared is not None else self.preflight(canonical)
        s = self.state
        new_epoch = s["reset_count"] is not None and reset_count != s["reset_count"]
        if new_epoch:
            self.reset()
            s = self.state
            s["total_complete"] = update_count == 1
        elif s["sequence"] is None:
            # Firmware sequence can continue across STATS_RESET, so the
            # epoch-local update_count (not global sequence) defines TOTAL's
            # first full statistical observation.
            s["total_complete"] = update_count == 1
        if s["sequence"] is not None and sequence != s["sequence"] + 1:
            s["total_complete"] = False
            s["gap_count"] += 1
            if campaign_id == s["campaign_id"]:
                s["campaign_complete"] = False
        if campaign_id != s["campaign_id"]:
            s["camp"] = _populations()
            s["camp_seconds"] = 0
            s["campaign_complete"] = bool(campaign_id is not None and public_count == 1)
        elif campaign_id is not None and s["public_count"] is not None:
            if public_count != s["public_count"] + 1:
                s["campaign_complete"] = False
        s["total_seconds"] += 1
        if campaign_id is not None:
            s["camp_seconds"] += 1
        for name, value in values.items():
            if value is not None:
                _add(s["total"][name], value)
                if campaign_id is not None:
                    _add(s["camp"][name], value)
        self._append_rolling(sequence, update_count, reset_count, values)
        s.update({"sequence": sequence, "reset_count": reset_count,
                  "campaign_id": campaign_id, "campaign": campaign.get("campaign"),
                  "public_count": public_count})
        return self._surface(values)

    def _surface(self, current):
        s = self.state
        # TOTAL/CAMP Welfords remain in the canonical PHOTONS row; the bounded
        # rolling ring is instead stored in config.PHOTONS_RECOVERY as another
        # component of the same atomically committed resurrection image.
        checkpoint = copy.deepcopy(s)
        return {
            "schema": SCHEMA,
            "observed": current,
            "windows": {"600": self.short.summarize(),
                        **{str(span): self._minute_surface(span)
                           for span in WINDOWS if span != SHORT_LIMIT}},
            "total": {name: {**_surface(p),
                             "complete": (s["total_complete"]
                                          and p["n"] == s["total_seconds"])}
                      for name, p in s["total"].items()},
            "camp": ({name: {**_surface(p),
                             "complete": (s["campaign_complete"]
                                          and p["n"] == s["camp_seconds"])}
                      for name, p in s["camp"].items()} if s["campaign_id"] is not None else None),
            "gap_count": s["gap_count"],
            "checkpoint": checkpoint,
        }
