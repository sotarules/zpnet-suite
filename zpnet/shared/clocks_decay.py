"""Pi-only local linear drift estimation from admitted DAC target observations.

Rates are physical DAC-code increases per GNSS second. Correction is applied
to observations before averaging, never to a firmware clock or servo input.
Minute blocks retain exact regression sufficient statistics, not a second-by-
second history. Integer nanocode quantization is far below the reported scale.
"""

from __future__ import annotations

import copy
import math
from collections import deque
from fractions import Fraction
from uuid import uuid4


LANES = ("ocxo1", "ocxo2")
NS = 1_000_000_000
CODE_SCALE = 1_000_000_000
MINUTE = 60 * NS
MAX_HISTORY = 48 * 3600 * NS
VOLTS_PER_CODE = Fraction(5, 65536)
PPB_PER_CODE = {"ocxo1": Fraction(364, 1000), "ocxo2": Fraction(384, 1000)}


def rational(value: Fraction) -> dict:
    return {"numerator": value.numerator, "denominator": value.denominator}


def fraction(value: dict) -> Fraction:
    return Fraction(value["numerator"], value["denominator"])


def _block(t: int) -> dict:
    return {"minute": t // MINUTE, "first_ns": t, "last_ns": t,
            "n": 0, "x": 0, "xx": 0,
            "lanes": {name: {"y": 0, "xy": 0, "yy": 0} for name in LANES}}


def _add(block: dict, t: int, codes: dict) -> None:
    block["last_ns"] = t
    block["n"] += 1
    block["x"] += t
    block["xx"] += t * t
    for name in LANES:
        y = round(Fraction(str(codes[name])) * CODE_SCALE)
        lane = block["lanes"][name]
        lane["y"] += y
        lane["xy"] += t * y
        lane["yy"] += y * y


def fit_blocks(blocks: list[dict]) -> dict | None:
    """OLS over all admitted observations; blocks do not change their weight.

    Residual RMS uses minute means and is descriptive, not an uncertainty or
    independent-sample standard error: servo observations are correlated.
    """
    if len(blocks) < 2:
        return None
    n = sum(b["n"] for b in blocks)
    sx = sum(b["x"] for b in blocks)
    sxx = sum(b["xx"] for b in blocks)
    denominator = n * sxx - sx * sx
    if denominator == 0:
        return None
    result = {"n": n, "blocks": len(blocks), "first_ns": blocks[0]["first_ns"],
              "last_ns": blocks[-1]["last_ns"], "span_seconds":
              (blocks[-1]["last_ns"] - blocks[0]["first_ns"]) / NS}
    for name in LANES:
        sy = sum(b["lanes"][name]["y"] for b in blocks)
        sxy = sum(b["lanes"][name]["xy"] for b in blocks)
        slope_ns = Fraction(n * sxy - sx * sy, denominator * CODE_SCALE)
        # Only descriptive RMS uses float. Center the block means before
        # multiplying; exact OLS and the applied slope remain rational. This
        # avoids thousands of large Fraction squares under the producer lock.
        mean_x = sx / n
        mean_y = sy / n
        slope_float = float(slope_ns)
        error = sum(((b["lanes"][name]["y"] / b["n"] - mean_y) / CODE_SCALE
                     - slope_float * (b["x"] / b["n"] - mean_x)) ** 2
                    for b in blocks) / len(blocks)
        rate = slope_ns * NS
        result[name] = {
            "slope_codes_per_second": rational(rate),
            "rate_nv_per_second": float(rate * VOLTS_PER_CODE * NS),
            "rate_ppb_per_day": float(-rate * PPB_PER_CODE[name] * 86400),
            "minute_residual_rms_ppb": math.sqrt(float(error)) * float(PPB_PER_CODE[name]),
        }
    return result


class DecayProfiler:
    """Caller serializes access and persists checkpoint with accepted observations.

    A fresh installation learns by default with zero applied correction. Starting
    another learning session retains the accepted model until explicit freeze.
    The logical GNSS coordinate advances across positive GNSS clock intervals,
    including reception gaps. A clock zero/reboot never becomes a huge negative
    correction; its unknown elapsed interval is recorded as a discontinuity.
    """

    def __init__(self, window_hours: float = 24):
        self.profile_enabled = True
        self.window_hours = self._window(window_hours)
        self.model_id = uuid4().hex
        self.session_id = uuid4().hex
        self.time_ns = 0
        self.gnss_ns = None
        self.anchor_ns = 0
        self.anchor_gnss_ns = None
        self.offset = {name: Fraction(0) for name in LANES}
        self.slope = {name: Fraction(0) for name in LANES}
        self.discontinuities = 0
        self.skipped_servo_samples = 0
        self.blocks = deque()
        self.current = None
        self._fits = {}
        self.accepted_fit = None

    @staticmethod
    def _window(hours: float) -> float:
        hours = float(hours)
        if not math.isfinite(hours) or not 1 <= hours <= 48:
            raise ValueError("window_hours must be between 1 and 48")
        return hours

    def correction(self, name: str) -> Fraction:
        return self.offset[name] + self.slope[name] * Fraction(self.time_ns - self.anchor_ns, NS)

    def _restart_fit(self) -> None:
        self.session_id = uuid4().hex
        self.blocks.clear()
        self.current = None
        self._fits = {}

    def restart_profile(self) -> None:
        """Discard learned samples after a deliberate target/control change."""
        self._restart_fit()

    def _refresh_fits(self) -> None:
        self._fits = {}
        for hours in sorted({6, 12, 24, 48, self.window_hours}):
            cutoff = self.time_ns - int(hours * 3600 * NS)
            # Whole minute blocks make the selected window accurate to one minute.
            selected = [b for b in self.blocks if b["first_ns"] >= cutoff]
            self._fits[f"{hours:g}"] = fit_blocks(selected)

    def observe(self, gnss_ns: int, codes: dict, servo_mode: str) -> None:
        gnss_ns = int(gnss_ns)
        if self.gnss_ns is not None:
            dt = gnss_ns - self.gnss_ns
            if dt == 0:
                return
            if dt < 0 or dt > 120 * NS:
                self.discontinuities += 1
                self._restart_fit()
            if dt > 0:
                self.time_ns += dt
        self.gnss_ns = gnss_ns
        if not self.profile_enabled:
            return
        # The intended profiling condition is a stationary 10-MIN lock. Other
        # controller modes never silently enter the same regression population.
        if servo_mode != "10-MIN":
            self.skipped_servo_samples += 1
            if self.blocks or self.current is not None:
                self._restart_fit()
            return
        minute = self.time_ns // MINUTE
        if self.current is not None and self.current["minute"] != minute:
            self.blocks.append(self.current)
            self.current = None
            while self.blocks and self.blocks[0]["last_ns"] < self.time_ns - MAX_HISTORY:
                self.blocks.popleft()
            self._refresh_fits()
        if self.current is None:
            self.current = _block(self.time_ns)
        _add(self.current, self.time_ns, codes)

    def set_profiling(self, enabled: bool, window_hours: float | None = None) -> None:
        if window_hours is not None:
            self.window_hours = self._window(window_hours)
            self._refresh_fits()
        if enabled == self.profile_enabled:
            return
        if enabled:
            self._restart_fit()
            self.profile_enabled = True
            return
        self._refresh_fits()
        candidate = self._fits.get(f"{self.window_hours:g}")
        if candidate is None or candidate["span_seconds"] < 600:
            # Ten minutes is a minimum identifiable population, not a claim
            # that its slope predicts a 12--48 hour experiment accurately.
            raise ValueError("Decay profiling needs at least 10 minutes of completed 10-MIN servo observations")
        # Re-anchor the accepted linear correction continuously. Freezing changes
        # the derivative, not the current offset or an existing DAC baseline.
        self.offset = {name: self.correction(name) for name in LANES}
        self.slope = {name: fraction(candidate[name]["slope_codes_per_second"]) for name in LANES}
        self.anchor_ns = self.time_ns
        self.anchor_gnss_ns = self.gnss_ns
        self.model_id = uuid4().hex
        self.accepted_fit = copy.deepcopy(candidate)
        self.profile_enabled = False

    def snapshot(self) -> dict:
        return {"schema": "CLOCKS_DECAY_V1", "profile_enabled": self.profile_enabled,
                "window_hours": self.window_hours, "model_id": self.model_id,
                "session_id": self.session_id, "time_ns": self.time_ns,
                "gnss_ns": self.gnss_ns, "last_gnss_ns": self.gnss_ns, "anchor_ns": self.anchor_ns,
                "anchor_gnss_ns": self.anchor_gnss_ns,
                "offset_codes": {name: rational(self.offset[name]) for name in LANES},
                "slope_codes_per_second": {name: rational(self.slope[name]) for name in LANES},
                "correction_codes": {name: rational(self.correction(name)) for name in LANES},
                "candidate": copy.deepcopy(self._fits.get(f"{self.window_hours:g}")),
                "fits": copy.deepcopy(self._fits), "accepted_fit": copy.deepcopy(self.accepted_fit),
                "discontinuities": self.discontinuities,
                "skipped_servo_samples": self.skipped_servo_samples}

    def restore_timing(self, snapshot: dict) -> None:
        """Resume an accepted model from its last recorded observation.

        Profiling always begins a fresh fitting session after a Pi restart; the
        durable accepted coefficient survives independently of learning history.
        """
        if snapshot["model_id"] != self.model_id:
            return
        self.time_ns = int(snapshot["time_ns"])
        self.gnss_ns = snapshot["last_gnss_ns"]
        self.discontinuities = int(snapshot["discontinuities"])
        self._restart_fit()

    def checkpoint(self) -> dict:
        state = self.snapshot()
        state["blocks"] = list(self.blocks)
        state["current"] = copy.deepcopy(self.current)
        return copy.deepcopy(state)

    @classmethod
    def from_checkpoint(cls, state: dict) -> "DecayProfiler":
        if state["schema"] != "CLOCKS_DECAY_V1":
            raise ValueError("Unknown decay profile schema")
        result = cls(state["window_hours"])
        for name in ("profile_enabled", "model_id", "session_id", "time_ns", "gnss_ns",
                     "anchor_ns", "anchor_gnss_ns", "discontinuities", "skipped_servo_samples",
                     "accepted_fit", "current"):
            setattr(result, name, copy.deepcopy(state[name]))
        result.offset = {name: fraction(state["offset_codes"][name]) for name in LANES}
        result.slope = {name: fraction(state["slope_codes_per_second"][name]) for name in LANES}
        result.blocks = deque(copy.deepcopy(state["blocks"]))
        result._refresh_fits()
        return result
