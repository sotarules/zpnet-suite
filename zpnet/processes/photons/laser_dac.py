"""Pi-owned manual laser amplitude control. No background writes or servo.

The DAC register is digital testimony, not a measurement of SMA voltage.
Process restart does not touch hardware. The first explicit SET_DAC zeros
the output before configuring the reference/gain, then applies the request.
"""

import copy
import math
import threading
from datetime import datetime, timezone

from zpnet.shared.ad5693r import read_input, write_register
from zpnet.shared.i2c_identity import I2C_BUS_EXPANDED

DAC_BUS = I2C_BUS_EXPANDED
DAC_ADDRESS = 0x4C
REFERENCE_V = 2.5
CODE_SCALE = 65536
# Bench ceiling: 1.8 V before the 47 ohm resistor is about 0.928 V
# into 50 ohms. This is a nominal circuit estimate, not an overshoot limit.
MAX_OUTPUT_V = 1.8
MAX_CODE = int(MAX_OUTPUT_V / REFERENCE_V * CODE_SCALE)
WRITE_DAC_AND_INPUT = 0x30
WRITE_CONTROL = 0x40
INTERNAL_REFERENCE_GAIN_ONE = 0x0000


class LaserDac:
    def __init__(self):
        self._lock = threading.RLock()
        self._io_lock = threading.Lock()
        self._state = {
            "schema": "PI_PHOTONS_LASER_DAC_V1",
            "owner": "PI.PHOTONS",
            "device": "AD5693R",
            "bus": DAC_BUS,
            "address": "0x4C",
            "mode": "MANUAL",
            "reference_v": REFERENCE_V,
            "gain": 1,
            "max_output_v": MAX_OUTPUT_V,
            "max_code": MAX_CODE,
            "startup_policy": "NO_IO_UNTIL_EXPLICIT_COMMAND",
            "state": "UNCONFIGURED",
            "commanded_code": None,
            "commanded_dac_v": None,
            "estimated_mod_v_into_50_ohm": None,
            "readback_input_code": None,
            "last_write_at_utc": None,
            "write_count": 0,
            "last_error": None,
        }

    def snapshot(self):
        """Memory only: safe for fragment ingress; never accesses I2C."""
        with self._lock:
            return copy.deepcopy(self._state)

    def _update(self, **changes):
        with self._lock:
            self._state.update(changes)

    @staticmethod
    def _write(command, value):
        write_register(DAC_BUS, DAC_ADDRESS, command, value)

    @staticmethod
    def _read():
        return read_input(DAC_BUS, DAC_ADDRESS)

    def readback(self):
        """Read input register only; cannot prove reference/gain or analog VOUT."""
        with self._io_lock:
            try:
                code = self._read()
                self._update(readback_input_code=code)
                if (self._state["state"] == "CONFIGURED"
                        and code != self._state["commanded_code"]):
                    raise RuntimeError("laser DAC input register differs from commanded code")
                return self.snapshot()
            except Exception as exc:
                self._update(state="FAULT", last_error=str(exc))
                raise

    def set_code(self, code):
        if type(code) is not int or not 0 <= code <= MAX_CODE:
            raise ValueError(f"code must be an integer in 0..{MAX_CODE}")
        with self._io_lock:
            initialize = self._state["state"] != "CONFIGURED"
            self._update(state="APPLYING")
            try:
                if initialize:
                    self._write(WRITE_DAC_AND_INPUT, 0)
                    self._write(WRITE_CONTROL, INTERNAL_REFERENCE_GAIN_ONE)
                self._write(WRITE_DAC_AND_INPUT, code)
                # Retry this read alone; never replay initialization/zero just
                # because the verification transfer encountered a transient NACK.
                observed = self._read()
                if observed != code:
                    raise RuntimeError(
                        f"laser DAC readback {observed} != requested {code}")
                volts = code * REFERENCE_V / CODE_SCALE
                self._update(
                    state="CONFIGURED", commanded_code=code,
                    commanded_dac_v=volts,
                    estimated_mod_v_into_50_ohm=volts * 50 / 97,
                    readback_input_code=observed,
                    last_write_at_utc=datetime.now(timezone.utc).isoformat(),
                    write_count=self._state["write_count"] + 1,
                    last_error=None,
                )
                return self.snapshot()
            except Exception as exc:
                # Never report the requested code as successfully applied after
                # a partial transaction. Recovery requires another explicit set.
                self._update(state="FAULT", last_error=str(exc))
                raise


def requested_code(args):
    """Validate operator input before any hardware transaction; never clamp."""
    if not isinstance(args, dict) or set(args) not in ({"code"}, {"volts"}):
        raise ValueError("SET_DAC requires exactly code=<integer> or volts=<DAC volts>")
    if "code" in args:
        value = args["code"]
        if isinstance(value, bool) or not str(value).isascii() or not str(value).isdigit():
            raise ValueError("code must be a nonnegative decimal integer")
        code = int(value)
    else:
        if isinstance(args["volts"], bool):
            raise ValueError("volts must be numeric")
        volts = float(args["volts"])
        if not math.isfinite(volts) or not 0 <= volts <= MAX_OUTPUT_V:
            raise ValueError(f"volts must be finite in 0..{MAX_OUTPUT_V}")
        code = min(MAX_CODE, int(volts / REFERENCE_V * CODE_SCALE + 0.5))
    if not 0 <= code <= MAX_CODE:
        raise ValueError(f"code must be in 0..{MAX_CODE}")
    return code
