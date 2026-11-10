"""Recorded Teensy voltage models; validation only, never a second correction.

NOR4 retains the HighLow2 HIGH60 fit; NOR5 uses the HighLow3 HIGH5 fit.
Their offsets preserve NOR3 at 455 mV; historical model identities never change.
NOR6 starts a separate servo-era, zero-correction baseline at 440 mV.
NOR7 adds a frozen linear residual correction fitted on NOR6-1.
NONE publishes raw GNSS-projected flight times with no voltage correction.
HIGH5 remains a servo/diagnostic witness; the zero polynomial is a wire-format
compatibility contract, not a fitted model. Historical NOR identities stay frozen.
The schema remains additive V1; model identity selects the polynomial contract.
"""
import math

QUADRATIC_MODEL = "PD200T_HIGH60_QUADRATIC_V1"  # historical NOR4
HIGH5_MODEL = "PD200T_HIGH5_QUADRATIC_V1"  # historical NOR5
SERVO_MODEL = "PD200T_HIGH5_SERVO_BASELINE_V1"  # NOR6, not a fitted calibration
SERVO_LINEAR_MODEL = "PD200T_HIGH5_SERVO_LINEAR_V1"  # NOR7, provisional NOR6-1 fit
NONE_MODEL = "NONE"
QUADRATIC_MODELS = (QUADRATIC_MODEL, HIGH5_MODEL, SERVO_MODEL, SERVO_LINEAR_MODEL, NONE_MODEL)
VOLTAGE_MODELS = {
    "PD200T_HIGH_LINEAR_V1": (0x4E4F5231, "FRAGMENT_HIGH_MV"),
    "PD200T_HIGH60_LINEAR_V1": (0x4E4F5232, "PD_OUT_HIGH60_MV"),
    "PD200T_HIGH60_LINEAR_V2": (0x4E4F5233, "PD_OUT_HIGH60_MV"),
    QUADRATIC_MODEL: (0x4E4F5234, "PD_OUT_HIGH60_MV"),
    HIGH5_MODEL: (0x4E4F5235, "PD_OUT_HIGH5_MV"),
    SERVO_MODEL: (0x4E4F5236, "PD_OUT_HIGH5_MV"),
    SERVO_LINEAR_MODEL: (0x4E4F5237, "PD_OUT_HIGH5_MV"),
    NONE_MODEL: (0x4E4F4E45, "PD_OUT_HIGH5_MV"),  # ASCII NONE; distinct raw epoch
}
QUADRATIC_COEFFICIENTS = {
    "reference_mv": 455.0,
    "offset_ns": -7.091998360876317,
    "slope_ns_per_mv": 0.36537694032628687,
    "quadratic_ns_per_mv2": -0.004691243835679416,
}

HIGH5_COEFFICIENTS = {
    "reference_mv": 455.0,
    "offset_ns": -7.091998360876317,
    "slope_ns_per_mv": 0.38806607923641423,
    "quadratic_ns_per_mv2": -0.004808419702287202,
}
# The servo's nominal operating point is not a fitted timing coefficient.
# Keep it fixed even if an operator changes the servo target during a sweep.
# Do not inherit NOR5's intercept/curvature or regress timing on the DAC:
# equal HIGH5 does not establish equal edge shape, and a changing DAC is an
# actuator witness, not a calibrated flight-time correction. Initially RAW
# equals published LAP. Fit residual response on a development subset of the
# new hardware data, then check a held-out thermal interval before deployment.
# Any later coefficient revision needs a new model identity/acquisition epoch;
# never reinterpret recorded NOR6 baseline rows using newly fitted numbers.
SERVO_COEFFICIENTS = {
    "reference_mv": 440.0,
    "offset_ns": 0.0,
    "slope_ns_per_mv": 0.0,
    "quadratic_ns_per_mv2": 0.0,
}
# NOR6-1, 2026-11-09: 60-second blocks, first 646 for fitting,
# two-block embargo, last 276 for model selection. Validation centered RMS
# 0.436678 -> 0.337821 ns; quadratic worsened it to 0.557308 ns.
# Frozen candidate, not an independent confirmation or absolute timing fit.
# The remaining between-period mean drift is not removed by this slope.
# Reference stays fixed when the servo target changes; no DAC/time feedback.
SERVO_LINEAR_COEFFICIENTS = {
    "reference_mv": 440.0,
    "offset_ns": 0.0,
    "slope_ns_per_mv": 0.258677287479,
    "quadratic_ns_per_mv2": 0.0,
}
# Keep the existing telemetry fields so old NOR reports remain readable.
# NONE never converts HIGH5, temperature, or DAC voltage into a timing offset.
NONE_COEFFICIENTS = {
    "reference_mv": 440.0,
    "offset_ns": 0.0,
    "slope_ns_per_mv": 0.0,
    "quadratic_ns_per_mv2": 0.0,
}
FROZEN_QUADRATICS = {QUADRATIC_MODEL: QUADRATIC_COEFFICIENTS,
                     HIGH5_MODEL: HIGH5_COEFFICIENTS,
                     SERVO_MODEL: SERVO_COEFFICIENTS,
                     SERVO_LINEAR_MODEL: SERVO_LINEAR_COEFFICIENTS,
                     NONE_MODEL: NONE_COEFFICIENTS}
VOLTAGE_WINDOWS = {
    "PD_OUT_HIGH60_MV": ("high_window", "high60", 60000),
    "PD_OUT_HIGH5_MV": ("high5_window", "high5", 5000),
}


def voltage_window_spec(model_id):
    """Recorded model identity selects its physical predictor; no fallback."""
    return VOLTAGE_WINDOWS[VOLTAGE_MODELS[model_id][1]]


def voltage_window(witness, pd_out):
    """Check exact producer population custody before using either window."""
    key, prefix, target_ms = voltage_window_spec(witness["model_id"])
    window = pd_out[key]
    if witness["voltage_source"] != VOLTAGE_MODELS[witness["model_id"]][1]:
        raise ValueError("PHOTONS voltage predictor identity mismatch")
    if window["target_ms"] != target_ms or window["reference_mv"] != 3300:
        raise ValueError("PHOTONS voltage window target/reference mismatch")
    names = ("sample_count", "elapsed_ms", "fragments", "high_sum_counts", "high_sum_squares_counts")
    if any(type(window[name]) is not int or window[name] < 0 for name in names):
        raise ValueError("PHOTONS voltage window requires nonnegative integer moments")
    n = window["sample_count"]
    total, squares = window["high_sum_counts"], window["high_sum_squares_counts"]
    if (n < pd_out["sample_count"] or (n and not window["fragments"])
            or total > 4095 * n or squares > 4095 * total or n * squares < total * total):
        raise ValueError("PHOTONS voltage window moments/custody mismatch")
    if (any(witness[prefix + "_" + name] != window[name]
            for name in ("sample_count", "elapsed_ms", "fragments"))
            or witness[prefix + "_complete"] != (window["elapsed_ms"] >= target_ms)):
        raise ValueError("PHOTONS normalization window population mismatch")
    if n:
        exact_mv = total * (3300 / 4095) / n
        for value in (window["high_mean_mv"], witness["high_mv"]):
            if not math.isclose(float(value), exact_mv, rel_tol=0.0, abs_tol=1e-6):
                raise ValueError("PHOTONS predictor differs from exact ADC window mean")
    elif (witness["raw_flight_ns"]["n"] or float(witness["high_mv"]) != 0.0
          or float(witness["adjustment_ns"]) != 0.0):
        raise ValueError("empty voltage window cannot support a correction")
    return window


def voltage_coefficient_fields(model_id):
    if model_id not in VOLTAGE_MODELS:
        raise ValueError("unknown PHOTONS voltage model: " + str(model_id))
    fields = ("reference_mv", "slope_ns_per_mv")
    return fields + (("offset_ns", "quadratic_ns_per_mv2")
                     if model_id in QUADRATIC_MODELS else ())


def voltage_coefficients(witness):
    model = witness["model_id"]
    fields = voltage_coefficient_fields(model)
    if (witness["schema"] != "PHOTONS_VOLTAGE_NORMALIZATION_V1"
            or witness["application"] != "UNIFORM_OFFSET_AT_FRAGMENT_DRAIN"):
        raise ValueError("unsupported PHOTONS voltage contract")
    if model != "PD200T_HIGH_LINEAR_V1" and witness["voltage_source"] != VOLTAGE_MODELS[model][1]:
        raise ValueError("PHOTONS voltage predictor identity mismatch")
    values = {key: float(witness[key]) for key in fields}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("PHOTONS voltage coefficients must be finite")
    if model in QUADRATIC_MODELS:
        for key, expected in FROZEN_QUADRATICS[model].items():
            # Firmware Double has 16 significant decimal digits.
            if not math.isclose(values[key], expected, rel_tol=1e-14, abs_tol=0.0):
                raise ValueError(model + " coefficient disagrees with frozen model: " + key)
        return (values["reference_mv"], values["offset_ns"],
                values["slope_ns_per_mv"], values["quadratic_ns_per_mv2"])
    if "offset_ns" in witness or "quadratic_ns_per_mv2" in witness:
        raise ValueError("linear identity cannot carry polynomial terms")
    # These are the mathematical zero coefficients of the historical linear models.
    return values["reference_mv"], 0.0, values["slope_ns_per_mv"], 0.0


def voltage_adjustment(witness, voltage_mv):
    reference, offset, slope, quadratic = voltage_coefficients(witness)
    x = float(voltage_mv) - reference
    result = offset + x * (slope + quadratic * x)
    if not math.isfinite(result):
        raise ValueError("PHOTONS polynomial requires finite voltage and adjustment")
    return result


def voltage_local_slope(witness, voltage_mv):
    reference, _, slope, quadratic = voltage_coefficients(witness)
    result = slope + 2.0 * quadratic * (float(voltage_mv) - reference)
    if not math.isfinite(result):
        raise ValueError("PHOTONS polynomial derivative is not finite")
    return result
