"""Host-only Pi laser DAC startup policy tests; no I2C operations reach hardware.

Run with: zt photons_dac_resume_test
Prints to STDOUT. All DAC I/O is replaced with local witness callbacks.
"""
from __future__ import annotations

import sys
from zpnet.processes.photons.laser_dac import (
    LaserDac, MAX_CODE, REFERENCE_V, CODE_SCALE,
    WRITE_DAC_AND_INPUT, WRITE_CONTROL, INTERNAL_REFERENCE_GAIN_ONE,
    requested_code,
)
from zpnet.processes.photons.laser_servo import LaserServo


def fixture(codes):
    dac = LaserDac()
    reads = iter(codes)
    writes = []
    dac._read = lambda: next(reads)
    dac._write = lambda command, code: writes.append((command, code))
    return dac, writes


def main(_argv):
    warm_code = requested_code({"volts": 1.7})
    dac, writes = fixture([warm_code, warm_code + 1])
    servo = LaserServo(dac)
    servo.startup()
    state = dac.snapshot()
    assert state["state"] == "CONFIGURED"
    assert state["commanded_code"] == warm_code
    assert state["startup_policy"] == "RETAINED_LIVE_DAC_INPUT"
    assert state["write_count"] == 0 and state["last_write_at_utc"] is None
    assert servo.snapshot()["startup_source"] == "RETAINED_LIVE_DAC_INPUT"
    assert servo.snapshot()["mode"] == "AUTO"
    assert writes == [], "warm process restart must not glitch the DAC"
    dac.set_code(warm_code + 1)
    assert writes == [(WRITE_DAC_AND_INPUT, warm_code + 1)], "later servo writes must not reinitialize DAC"

    cold_code = requested_code({"volts": 1.4})
    dac, writes = fixture([0, cold_code])
    servo = LaserServo(dac)
    servo.startup()
    assert dac.snapshot()["commanded_code"] == cold_code
    assert dac.snapshot()["write_count"] == 1
    assert servo.snapshot()["startup_source"] == "COLD_ZERO_SCALE_COMMISSION"
    assert writes == [(WRITE_DAC_AND_INPUT, 0),
                      (WRITE_CONTROL, INTERNAL_REFERENCE_GAIN_ONE),
                      (WRITE_DAC_AND_INPUT, cold_code)]

    dac, writes = fixture([MAX_CODE + 1])
    try:
        dac.adopt_live_input()
    except RuntimeError as exc:
        assert "exceeds safe range" in str(exc)
    else:
        raise AssertionError("out-of-range retained input was not rejected")
    assert dac.snapshot()["state"] == "FAULT" and writes == []

    dac, writes = fixture([])
    try:
        dac.adopt_live_input()
    except StopIteration:
        pass
    else:
        raise AssertionError("missing DAC readback was not rejected")
    assert dac.snapshot()["state"] == "FAULT" and writes == []

    print("PHOTONS DAC RESUME: warm retain, later increment, cold init, range/read faults PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
