"""Slow Pi-owned optical-amplitude control; LAP never enters this controller.

Operator commands (DAC volts and detector millivolts are different units):
    pc photons servo_info
    pc photons servo_start target_mv=447.5
    pc photons servo_stop
    pc photons set_dac volts=1.4  # also stops automatic control

Startup deliberately establishes 1.4 V, including on a Pi-only restart.
Automatic mode is volatile and starts enabled by default. STOP campaign does
not stop this instrument controller; SERVO_STOP holds the last DAC setting.
NOR5 remains unchanged: servo experiments must establish a future NOR6 fit.
"""

import copy
import logging
import math
import threading
import time

from zpnet.processes.photons.voltage_sweep import VoltageSweep
from zpnet.processes.photons.laser_dac import MAX_CODE, REFERENCE_V, CODE_SCALE, requested_code

# Commissioning policy: edit here, not in firmware. Positive gain assumes that
# increasing modulation amplitude increases measured PD OUT HIGH5.
STARTUP_DAC_V = 1.4
STARTUP_SERVO_ENABLED = True
TARGET_MV = 447.5
DEADBAND_MV = 0.5
GAIN_V_PER_MV = 0.001
MAX_STEP_V = 0.010
SETTLE_SECONDS = 7.0
MAX_FEEDBACK_AGE_SECONDS = 2.5
MIN_WINDOW_SAMPLES = 1000


class LaserServo:
    def __init__(self, dac):
        self.dac = dac
        self._sweep = VoltageSweep()
        self._control = threading.RLock()
        # Snapshot readers never acquire the actuator lock or wait for I2C.
        self._lock = threading.Lock()
        self._started = False
        self._previous = None
        self._ready_at = 0.0
        self._state = {
            "schema": "PI_PHOTONS_HIGH5_SERVO_V1",
            "mode": "STARTING", "status": "WAITING_STARTUP",
            "target_mv": TARGET_MV, "deadband_mv": DEADBAND_MV,
            "gain_v_per_mv": GAIN_V_PER_MV, "max_step_v": MAX_STEP_V,
            "settle_seconds": SETTLE_SECONDS,
            "max_feedback_age_seconds": MAX_FEEDBACK_AGE_SECONDS,
            "minimum_window_samples": MIN_WINDOW_SAMPLES,
            "startup_dac_v": STARTUP_DAC_V,
            "startup_servo_enabled": STARTUP_SERVO_ENABLED,
            "feedback_source": "PD_OUT_HIGH5_MV",
            "high5_mv": None, "error_mv": None, "sequence": None,
            "adjustment_count": 0, "last_error": None,
        }

    def snapshot(self):
        with self._lock:
            state = copy.deepcopy(self._state)
        state["sweep"] = self._sweep.brief()
        return state

    def _update(self, **fields):
        with self._lock:
            self._state.update(fields)

    def _settle(self):
        self._previous = None
        self._ready_at = time.monotonic() + SETTLE_SECONDS

    def _fault(self, exc):
        # Shared DAC transfers already perform bounded retries. Do not keep
        # reinitializing/zeroing a failed actuator once per feedback tick.
        self._sweep.cancel("DAC_OR_CONTROLLER_FAULT")
        self._update(mode="FAULT", status="FAULT", last_error=str(exc))
        logging.exception("[photons/servo] latched; explicit SET_DAC then SERVO_START required")

    def startup(self):
        with self._control:
            if self._started:
                return
            self._started = True
            try:
                self.dac.set_code(requested_code({"volts": STARTUP_DAC_V}))
                self._settle()
                self._update(mode="AUTO" if STARTUP_SERVO_ENABLED else "MANUAL",
                             status="SETTLING")
                logging.info("[photons/servo] startup DAC=%.6f target=%.3f auto=%s",
                             STARTUP_DAC_V, TARGET_MV, STARTUP_SERVO_ENABLED)
            except Exception as exc:
                self._fault(exc)

    def manual(self, code):
        with self._control:
            self._sweep.cancel("MANUAL_DAC_COMMAND")
            self._started = True  # An early explicit command supersedes startup.
            self._update(mode="MANUAL", status="MANUAL", last_error=None)
            self._settle()
            try:
                return self.dac.set_code(code)
            except Exception as exc:
                self._fault(exc)
                raise

    def start(self, args):
        args = args or {}
        if set(args) - {"target_mv"}:
            raise ValueError("SERVO_START accepts only target_mv=<PD OUT millivolts>")
        value = args.get("target_mv", self.snapshot()["target_mv"])
        if isinstance(value, bool):
            raise ValueError("target_mv must be numeric")
        target = float(value)
        if not math.isfinite(target) or not 0 < target < 3300:
            raise ValueError("target_mv must be finite and between 0 and 3300")
        with self._control:
            if self.dac.snapshot()["state"] != "CONFIGURED":
                raise ValueError("configure the DAC with SET_DAC before SERVO_START")
            self._sweep.cancel("EXPLICIT_SERVO_START")
            self._started = True
            self._settle()
            self._update(mode="AUTO", status="SETTLING", target_mv=target, last_error=None)
            logging.info("[photons/servo] enabled target=%.3f mV", target)
            return self.snapshot()

    def stop(self):
        with self._control:
            self._sweep.cancel("SERVO_STOP")
            # Stop does not write, zero the output, or clear a DAC fault.
            self._started = True
            self._update(mode="MANUAL", status="HOLDING")
            self._settle()
            logging.info("[photons/servo] stopped; holding DAC output")
            return self.snapshot()

    def sweep_start(self, args):
        with self._control:
            state = self.snapshot()
            if state["mode"] != "AUTO" or self.dac.snapshot()["state"] != "CONFIGURED":
                raise ValueError("SWEEP_START requires an enabled servo and configured DAC")
            target = self._sweep.start(args, state["target_mv"])
            self._settle()
            self._update(target_mv=target, status="SETTLING")
            logging.info("[photons/sweep] started %s", self._sweep.brief())

    def sweep_stop(self):
        with self._control:
            target = self._sweep.cancel("OPERATOR_STOP")
            if target is not None:
                # Restore the pre-survey target through the existing bounded
                # servo, not a sudden DAC jump. Manual commands cancel instead.
                self._settle()
                self._update(target_mv=target, status="SETTLING")
            logging.info("[photons/sweep] stopped %s", self._sweep.brief())

    def sweep_report(self):
        return self._sweep.report()

    def tick(self, feedback, hold_reason):
        with self._control:
            if self.snapshot()["mode"] != "AUTO":
                return
            try:
                if self.dac.snapshot()["state"] != "CONFIGURED":
                    raise RuntimeError("DAC is not configured")
                target = self._sweep.observe(feedback, hold_reason, self.snapshot(),
                                             self._ready_at, MAX_CODE)
                if target is not None:
                    self._settle()
                    self._update(target_mv=target, status="SETTLING")
                    logging.info("[photons/sweep] %s target=%.3f", self._sweep.brief(), target)
                self._tick(feedback, hold_reason)
            except Exception as exc:
                self._fault(exc)

    def _tick(self, feedback, hold_reason):
        now = time.monotonic()
        dac = self.dac.snapshot()
        if dac["state"] != "CONFIGURED":
            raise RuntimeError("DAC is not configured; automatic writes suspended")
        if hold_reason or feedback is None:
            self._settle()
            self._update(status=hold_reason or "WAITING_FEEDBACK")
            return
        received = feedback["received_monotonic"]
        if received is None or not 0 <= now - received <= MAX_FEEDBACK_AGE_SECONDS:
            self._settle()
            self._update(status="STALE_FEEDBACK")
            return
        w = feedback["high5"]
        seq = feedback["sequence"]
        identity = (feedback["reset_count"], seq, w["completed_at_ms"])
        if identity == self._previous:
            return  # Each accepted producer fragment is used at most once.
        if self._previous is not None:
            reset, previous_seq, previous_ms = self._previous
            if (identity[0] != reset or seq != previous_seq + 1
                    or w["completed_at_ms"] <= previous_ms):
                self._ready_at = now + SETTLE_SECONDS
        self._previous = identity
        if (w["target_ms"] != 5000 or w["elapsed_ms"] < 5000
                or w["fragments"] < 5 or w["sample_count"] < MIN_WINDOW_SAMPLES):
            self._ready_at = now + SETTLE_SECONDS
            self._update(status="WAITING_FULL_HIGH5", sequence=seq)
            return
        high = w["high_mean_mv"]
        if not math.isfinite(high) or not 0 < high < w["reference_mv"]:
            raise ValueError("HIGH5 is outside the ADC measurement domain")
        state = self.snapshot()
        error = state["target_mv"] - high
        self._update(high5_mv=high, error_mv=error, sequence=seq)
        # Require a freshly received window AFTER the settling boundary, not
        # merely a late worker waking up with pre-adjustment feedback in hand.
        if received < self._ready_at:
            self._update(status="SETTLING")
            return
        if abs(error) <= DEADBAND_MV:
            self._update(status="IN_BAND")
            return
        delta = max(-MAX_STEP_V, min(MAX_STEP_V, GAIN_V_PER_MV * error))
        code = max(0, min(MAX_CODE, int(
            dac["commanded_code"] + delta / REFERENCE_V * CODE_SCALE + 0.5)))
        # Incremental proportional control has no separate accumulated error:
        # at either rail there is no hidden windup waiting to be unwound.
        if code == dac["commanded_code"]:
            self._update(status="LIMIT_HIGH" if error > 0 else "LIMIT_LOW")
            return
        result = self.dac.set_code(code)
        self._ready_at = time.monotonic() + SETTLE_SECONDS
        self._update(status="SETTLING", adjustment_count=state["adjustment_count"] + 1)
        logging.info("[photons/servo] seq=%d HIGH5=%.3f target=%.3f DAC=%.6f",
                     seq, high, state["target_mv"], result["commanded_dac_v"])
