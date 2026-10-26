"""Recorded Teensy voltage models; validation only, never a second correction.

NOR4 is the frozen HighLow2 development fit. Offset preserves NOR3 at 455 mV.
The schema remains additive V1; model identity selects the polynomial contract.
"""
import math

QUADRATIC_MODEL = "PD200T_HIGH60_QUADRATIC_V1"
VOLTAGE_MODELS = {
    "PD200T_HIGH_LINEAR_V1": (0x4E4F5231, "FRAGMENT_HIGH_MV"),
    "PD200T_HIGH60_LINEAR_V1": (0x4E4F5232, "PD_OUT_HIGH60_MV"),
    "PD200T_HIGH60_LINEAR_V2": (0x4E4F5233, "PD_OUT_HIGH60_MV"),
    QUADRATIC_MODEL: (0x4E4F5234, "PD_OUT_HIGH60_MV"),
}
QUADRATIC_COEFFICIENTS = {
    "reference_mv": 455.0,
    "offset_ns": -7.091998360876317,
    "slope_ns_per_mv": 0.36537694032628687,
    "quadratic_ns_per_mv2": -0.004691243835679416,
}


def voltage_coefficient_fields(model_id):
    if model_id not in VOLTAGE_MODELS:
        raise ValueError("unknown PHOTONS voltage model: " + str(model_id))
    fields = ("reference_mv", "slope_ns_per_mv")
    return fields + (("offset_ns", "quadratic_ns_per_mv2")
                     if model_id == QUADRATIC_MODEL else ())


def voltage_coefficients(witness):
    model = witness["model_id"]
    fields = voltage_coefficient_fields(model)
    if (witness["schema"] != "PHOTONS_VOLTAGE_NORMALIZATION_V1"
            or witness["application"] != "UNIFORM_OFFSET_AT_FRAGMENT_DRAIN"):
        raise ValueError("unsupported PHOTONS voltage contract")
    if model != "PD200T_HIGH_LINEAR_V1" and witness["voltage_source"] != "PD_OUT_HIGH60_MV":
        raise ValueError("PHOTONS voltage predictor identity mismatch")
    values = {key: float(witness[key]) for key in fields}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("PHOTONS voltage coefficients must be finite")
    if model == QUADRATIC_MODEL:
        for key, expected in QUADRATIC_COEFFICIENTS.items():
            # Firmware Double has 16 significant decimal digits.
            if not math.isclose(values[key], expected, rel_tol=1e-14, abs_tol=0.0):
                raise ValueError("NOR4 coefficient disagrees with frozen model: " + key)
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
