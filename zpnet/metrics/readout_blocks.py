"""
ZPNet Metrics Readout Blocks — Generalized Campaign Detail Edition

Data source:
  The CLOCKS_V4 heartbeat owns the live operator view.  ``clocks`` is the
  canonical always-on instrument; optional top-level ``campaign`` is TEMPEST
  enrichment; baseline and platform context remain top-level CLOCKS surfaces.
  Metrics never waits for or reads TIMEBASE.

Stats policy (Pi is a stenographer):
  Every statistical quantity shown in this panel is read verbatim from the
  Teensy-authored CLOCKS instrument/campaign surfaces, including the GNSS reference Welford.
  GNSS residual samples are definitionally zero, but its N is the real
  Alpha-owned always-on Welford population, never campaign PPS count.
  No Pi-side means, stddevs, stderrs, PPB windows, or campaign frequencies are
  computed here.  The only Pi-side arithmetic is deterministic presentation
  conversion: DAC code → voltage and baseline delta (NOW - BASE).

Clock row doctrine:
  The dense operator table shows GNSS, VCLOCK, OCXO1, and OCXO2 only.
  GN_RAW and DWT remain available in CLOCKS/focused diagnostics but are omitted
  here because they are not primary TEMPEST science clocks.
  VALUE is the Beta-authored campaign-relative clockface while a campaign is
  active; otherwise it is the canonical always-on instrument clockface.
  10-MIN/60-MIN/8-HOUR/24-HOUR are firmware/Pi-producer-authored rolling PPB
  buckets.  Metrics never estimates a rolling population from repaint history.
  TOTAL is the always-on population since boot or the last statistics reset.
  TOTAL owns the displayed Welford population, baseline NOW value, and the
  legacy stats.<lane>.ppb fallback while producers migrate to ppb_buckets.
  CAMP is the firmware-authored campaign population.  It is --- outside an
  active campaign and is never reconstructed by the metrics process.
  RES is the firmware-published residual for the prior PPS interval.

Column layout (CLK rows):
  NAME   VALUE   10-MIN   60-MIN   8-HOUR   24-HOUR   TOTAL   CAMP   RES
         MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (DAC rows):
  NAME   DAC_VALUE_VOLTAGE   (blanks)   MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (INT rows):
  NAME   END_GNSS_NS   DELTA_NS
"""

import time

from zpnet.processes.processes import create_pubsub_cache, send_command
from zpnet.shared.db import open_db

# AD5693R DAC doctrine mirrors firmware: internal 2.5 V reference with 2x
# gain, yielding an effective 0..5 V code span.  This is presentation-only;
# Teensy remains DAC authority.
VREF = 5.0

DAC_FINE_STEP = 1.0
DAC_COARSE_STEP = 10.0
DAC_MIN_CODE = 0.0
DAC_MAX_CODE = 65535.0

FEATURE_GRID_COLUMNS = 4
FEATURE_GRID_CELL_WIDTH = 39

PPB_BUCKET_KEYS = ("10_min", "60_min", "8_hour", "24_hour", "total")
PPB_VISIBLE_COLUMN_COUNT = len(PPB_BUCKET_KEYS) + 1  # + campaign

CLOCKS_TOPIC = "CLOCKS"
_LIVE_CACHE = create_pubsub_cache(CLOCKS_TOPIC)

# Mission-control readiness board. The feature payload remains scalar-only;
# this table is just the operator-facing projection of CLOCKS.features.
FEATURE_STATUS_GRID = (
    ("NET", "PI.SYSTEM.NETWORK"),
    ("BATTERY", "PI.SYSTEM.BATTERY"),
    ("GNSS", "PI.GNSS.REPORT"),
    ("PI_HOST", "PI.SYSTEM.HOST"),
    ("POWER", "PI.SYSTEM.POWER"),
    ("ENV", "PI.SYSTEM.ENVIRONMENT"),
    ("SENSORS", "PI.SYSTEM.SENSORS"),
    ("OBS_EDGE", "TEENSY.INTERRUPT.OBSERVED_EDGE_AUTHORITY"),
    ("QTIMER_CNT", "TEENSY.INTERRUPT.QTIMER_COUNTER_CUSTODY"),
    ("CTR32_LINE", "TEENSY.INTERRUPT.COUNTER32_LINEAGE"),
    ("PPS/V_AUTH", "TEENSY.INTERRUPT.PPS_VCLOCK_AUTHORITY"),
    ("ALPHA_EPOCH", "TEENSY.CLOCKS.ALPHA_EPOCH"),
    ("DWT_CAL", "TEENSY.CLOCKS.DWT_CALIBRATION"),
    ("STATIC_PRED", "TEENSY.CLOCKS.STATIC_PREDICTION"),
    ("OCXO_ORIGIN", "TEENSY.CLOCKS.OCXO_PUBLIC_ORIGIN"),
)


# ---------------------------------------------------------------------
# Data fetchers
# ---------------------------------------------------------------------

def _get_system_snapshot() -> dict:
    """Return latest CLOCKS without command/response traffic."""
    return _LIVE_CACHE.get(CLOCKS_TOPIC) or {}


def _get_feature_status_payload(force: bool = False) -> tuple[dict, str | None]:
    """Return the feature tree carried by the latest CLOCKS snapshot."""
    _ = force
    monitor = _get_system_snapshot()
    features = monitor.get("features")
    if isinstance(features, dict):
        return features, None
    return {}, _LIVE_CACHE.error()


def _monitor_root() -> dict:
    """Return the CLOCKS publication itself.

    PubSubTapCache already removes the transport envelope and returns the
    published payload.  CLOCKS also legitimately contains a top-level
    ``payload`` block for Teensy Payload allocator/serialization metrics; that
    block is data, not another message envelope.  Unwrapping it hides sibling
    surfaces such as ``clocks``, ``features``, and ``gnss``.
    """
    return _get_system_snapshot()


def _get_pi_clocks_report() -> dict:
    """Return one presentation view over the canonical CLOCKS_V4 surfaces."""
    root = _monitor_root()
    clocks = root.get("clocks")
    if not isinstance(clocks, dict):
        return {}

    report = dict(clocks)
    campaign = root.get("campaign") if isinstance(root.get("campaign"), dict) else {}
    public_count = _to_int(campaign.get("public_count"))
    state = str(campaign.get("state") or "STOPPED").upper()

    report["campaign_delta"] = campaign
    report["campaign_present"] = bool(campaign) and state != "STOPPED"
    report["campaign_state"] = state
    report["campaign"] = campaign.get("name") if campaign else None
    report["campaign_type"] = "TEMPEST" if campaign.get("schema") == "TEMPEST_FRAGMENT_V1" else None
    report["campaign_elapsed"] = _seconds_to_hms(public_count) if public_count is not None else "00:00:00"
    report["instrument_elapsed"] = _seconds_to_hms(_to_int(clocks.get("instrument_age_seconds")) or 0)
    report["baseline"] = root.get("baseline") if isinstance(root.get("baseline"), dict) else None
    report["published_at_utc"] = root.get("published_at_utc")
    return report

def _get_pi_clocks_report_dac() -> dict:
    # CLOCKS_V4 control is already part of the canonical live report.
    return {}

def _get_clocks_baseline() -> dict | None:
    """Return the top-level CLOCKS_V4 baseline read model."""
    root = _monitor_root()
    baseline = root.get("baseline")
    if not isinstance(baseline, dict) or not baseline.get("baseline_set"):
        return None
    return baseline

def _seconds_to_hms(seconds) -> str:
    value = _to_int(seconds)
    if value is None or value < 0:
        value = 0
    return f"{value // 3600:02d}:{(value % 3600) // 60:02d}:{value % 60:02d}"


# ---------------------------------------------------------------------
# GNSS normalization
# ---------------------------------------------------------------------

def _merge_missing(dst: dict, src: dict | None) -> dict:
    if not isinstance(src, dict):
        return dst
    for key, value in src.items():
        if value is not None and dst.get(key) is None:
            dst[key] = value
    return dst


def _derive_gnss_lock_quality(g: dict) -> str:
    explicit = g.get("lock_quality")
    if explicit:
        return str(explicit).upper()

    freq_mode = str(g.get("freq_mode_name") or "").upper()
    time_status = str(g.get("time_status") or "").upper()
    traim = str(g.get("traim") or "").upper()
    pps_valid = _to_bool(g.get("pps_valid"))
    pps_active = _to_bool(g.get("pps_active"))

    acc = _to_float(g.get("estimated_accuracy_ns"))
    timing_err = _to_float(g.get("pps_timing_error_ns"))
    hdop = _to_float(g.get("hdop"))
    sats = _to_int(g.get("satellites"))

    fine_lock = freq_mode == "FINE_LOCK"
    time_locked = time_status in ("LS_FIX", "UTC", "LOCKED", "FIX")
    traim_ok = traim in ("OK", "", "NONE")
    pps_ok = (pps_valid is not False) and (pps_active is not False)
    acc_ok = acc is None or acc <= 50.0
    timing_ok = timing_err is None or abs(timing_err) <= 100.0
    hdop_ok = hdop is None or hdop <= 1.5
    sats_ok = sats is None or sats >= 8

    if fine_lock and time_locked and traim_ok and pps_ok and acc_ok and timing_ok and hdop_ok and sats_ok:
        return "STRONG"
    if fine_lock and time_locked:
        return "STRONG"
    if fine_lock or time_locked or pps_ok:
        return "MEDIUM"
    return "WEAK"


def _gnss_from_timebase(r: dict) -> dict:
    return {
        "lock_quality": _field(r, "gnss.lock_quality"),
        "pos_mode": _field(r, "gnss.pos_mode"),
        "freq_mode_name": _field(r, "gnss.freq_mode_name"),
        "satellites": _field(r, "gnss.satellites"),
        "hdop": _field(r, "gnss.hdop"),
        "traim": _field(r, "gnss.traim"),
        "latitude_deg": _field(r, "gnss.latitude_deg", "location.latitude_deg"),
        "longitude_deg": _field(r, "gnss.longitude_deg", "location.longitude_deg"),
        "altitude_m": _field(r, "environment.gnss_altitude_m", "gnss.altitude_m"),
        "ellipsoid_height_m": _field(r, "environment.ellipsoid_height_m", "gnss.ellipsoid_height_m"),
        "geoid_sep_m": _field(r, "environment.geoid_sep_m", "gnss.geoid_sep_m"),
        "pps_valid": _field(r, "gnss.pps_valid"),
        "pps_active": _field(r, "gnss.pps_active"),
        "estimated_accuracy_ns": _field(r, "gnss.estimated_accuracy_ns"),
        "pps_timing_error_ns": _field(r, "gnss.pps_timing_error_ns"),
        "time_status": _field(r, "gnss.time_status"),
        "pps_sync": _field(r, "gnss.pps_sync"),
        "clock_drift_ppb": _field(r, "gnss.clock_drift_ppb"),
        "temperature_c": _field(r, "gnss.temperature_c", "environment.gnss_temp_c"),
    }


def _gnss_from_system_snapshot(snapshot: dict) -> dict:
    if not isinstance(snapshot, dict):
        return {}

    gnss = snapshot.get("gnss", {}) if isinstance(snapshot.get("gnss"), dict) else {}
    discipline = gnss.get("discipline", {}) if isinstance(gnss.get("discipline"), dict) else {}
    clock = gnss.get("clock", {}) if isinstance(gnss.get("clock"), dict) else {}
    integrity = gnss.get("integrity", {}) if isinstance(gnss.get("integrity"), dict) else {}
    survey = gnss.get("survey_mode", {}) if isinstance(gnss.get("survey_mode"), dict) else {}
    pps = gnss.get("pps", {}) if isinstance(gnss.get("pps"), dict) else {}

    return {
        "lock_quality": gnss.get("lock_quality"),
        "pos_mode": gnss.get("pos_mode") or integrity.get("pos_mode") or survey.get("receiver_mode"),
        "freq_mode_name": gnss.get("freq_mode_name") or discipline.get("freq_mode_name"),
        "satellites": gnss.get("satellites"),
        "hdop": gnss.get("hdop"),
        "traim": gnss.get("traim") or integrity.get("traim"),
        "latitude_deg": gnss.get("latitude_deg"),
        "longitude_deg": gnss.get("longitude_deg"),
        "altitude_m": gnss.get("altitude_m"),
        "ellipsoid_height_m": gnss.get("ellipsoid_height_m"),
        "geoid_sep_m": gnss.get("geoid_sep_m"),
        "pps_valid": gnss.get("pps_valid"),
        "pps_active": pps.get("active"),
        "estimated_accuracy_ns": gnss.get("estimated_accuracy_ns") or pps.get("estimated_accuracy_ns"),
        "pps_timing_error_ns": gnss.get("pps_timing_error_ns") or discipline.get("pps_timing_error_ns"),
        "time_status": gnss.get("time_status") or clock.get("time_status_name"),
        "pps_sync": gnss.get("pps_sync") or clock.get("pps_sync"),
        "clock_drift_ppb": gnss.get("clock_drift_ppb") or clock.get("drift_ppb"),
        "temperature_c": gnss.get("temperature_c") or clock.get("temperature_c"),
    }


def _gnss_status(r: dict | None = None, snapshot: dict | None = None) -> dict:
    # TIMEBASE owns accepted clock-correlated GNSS evidence. CLOCKS supplies
    # the broader last-known-good receiver snapshot.
    g: dict = {}
    if r is not None:
        _merge_missing(g, _gnss_from_timebase(r))
    if snapshot is not None:
        _merge_missing(g, _gnss_from_system_snapshot(snapshot))

    g["lock_quality"] = _derive_gnss_lock_quality(g)
    return g


# ---------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------

def _fmt(value, fmt_str, width, fallback="---"):
    if value is None:
        return f"{fallback:>{width}}"
    return f"{value:{fmt_str}}"


def _sign_int(value, width):
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>+{width}d}"


def _sign_float(value, width, decimals=1):
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>+{width}.{decimals}f}"


def _comma_int(value, width):
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width},d}"


def _safe_lines(fn):
    try:
        return fn()
    except Exception as e:
        return [f"ERROR: {e}"]


def _path_get(obj, path: str, default=None):
    if not isinstance(obj, dict):
        return default
    cur = obj
    for part in path.split("."):
        if not isinstance(cur, dict):
            return default
        if part not in cur:
            return default
        cur = cur.get(part)
    return cur if cur is not None else default


def _payload_root(r: dict) -> dict:
    # UI helpers receive the already-normalized CLOCKS.clocks presentation view.
    return r if isinstance(r, dict) else {}

def _fragment_root(r: dict) -> dict:
    # CLOCKS_V4 has no embedded TIMEBASE/fragment mirror in the live instrument.
    return _payload_root(r)

def _frag(r: dict, key: str, default=None):
    """Read one dotted field from the canonical CLOCKS_V4 presentation view."""
    root = _payload_root(r)
    frag = _fragment_root(root)
    for source in (frag, root):
        val = _path_get(source, key, None)
        if val is not None:
            return val
    return default


def _extra(r: dict, key: str, default=None):
    root = _payload_root(r)
    gnss_raw = root.get("gnss_raw")
    if not isinstance(gnss_raw, dict):
        return default
    aliases = {
        "gnss_raw_ns": ("instrument.ns",),
        "gnss_raw_ref_ns": ("instrument.ref_ns",),
        "gnss_raw_tau": ("instrument.tau",),
        "gnss_raw_ppb": ("instrument.ppb",),
        "gnss_raw_drift_ppb": ("drift_ppb",),
        "gnss_raw_welford_n": ("welford.n",),
        "gnss_raw_welford_mean": ("welford.mean",),
        "gnss_raw_welford_stddev": ("welford.stddev",),
        "gnss_raw_welford_stderr": ("welford.stderr",),
        "gnss_raw_welford_min": ("welford.min",),
        "gnss_raw_welford_max": ("welford.max",),
    }
    for path in aliases.get(key, (key,)):
        val = _path_get(gnss_raw, path, None)
        if val is not None:
            return val
    return default

def _field(r: dict, *keys, default=None):
    for key in keys:
        val = _frag(r, key, None)
        if val is not None:
            return val
    return default


def _stats_value(r: dict, prefix: str, field: str):
    val = _frag(r, f"stats.{prefix}.{field}")
    if val is not None:
        return val
    return _frag(r, f"{prefix}_{field}")


def _freq_value(r: dict, prefix: str, field: str):
    val = _frag(r, f"stats.{prefix}.{field}")
    if val is not None:
        return val
    return _frag(r, f"{prefix}_{field}")


def _ppb_bucket(r: dict, lane: str, bucket: str):
    """Read one producer-authored always-on PPB bucket.

    Canonical V1 publication is ``stats.<lane>.ppb_buckets.<bucket>``.  The
    aliases keep this readout usable while the producer surfaces are introduced
    independently.  Only TOTAL may fall back to the legacy scalar PPB because
    that scalar already denotes the always-on population.  A missing rolling
    bucket remains missing; substituting TOTAL would falsely label its window.
    """
    if lane == "gnss":
        return 0.0

    aliases = {
        "10_min": ("10_min", "min_10", "m10", "10m"),
        "60_min": ("60_min", "min_60", "m60", "60m"),
        "8_hour": ("8_hour", "hour_8", "h8", "8h"),
        "24_hour": ("24_hour", "hour_24", "h24", "24h"),
        "total": ("total",),
    }.get(bucket, (bucket,))

    paths = []
    for alias in aliases:
        paths.extend((
            f"stats.{lane}.ppb_buckets.{alias}",
            f"stats.{lane}.ppb_windows.{alias}",
            f"{lane}.ppb_buckets.{alias}",
            f"{lane}.ppb_windows.{alias}",
        ))

    value = _to_float(_field(r, *paths, default=None))
    if value is not None:
        return value

    if bucket != "total":
        return None

    if lane == "gnss_raw":
        return _to_float(_extra(r, "gnss_raw_ppb"))

    return _to_float(_freq_value(r, lane, "ppb"))


def _ppb_cols_fragment(r: dict, lane: str, width: int) -> str:
    values = [_ppb_bucket(r, lane, bucket) for bucket in PPB_BUCKET_KEYS]
    values.append(_campaign_ppb(r, lane))
    return "".join(_fmt(value, f">{width}.3f", width) for value in values)


def _welford_value(r: dict, prefix: str, field: str):
    if prefix.endswith("_dac"):
        lane = prefix[:-4]
        return _field(r, f"stats.auxiliary_welford.{lane}_dac.{field}", default=None)
    return _field(r, f"stats.{prefix}.welford.{field}", default=None)

def _to_float(v):
    if v is None:
        return None
    try:
        return float(v)
    except Exception:
        return None


def _to_bool(v):
    if isinstance(v, bool):
        return v
    if v is None:
        return None
    s = str(v).strip().lower()
    if s in ("true", "1", "yes", "on"):
        return True
    if s in ("false", "0", "no", "off"):
        return False
    return None


def _to_int(v):
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def _has_any(r: dict, *keys) -> bool:
    return any(_frag(r, k, None) is not None for k in keys)


def _none_if_zero(v):
    iv = _to_int(v)
    return None if iv == 0 else iv


def _ns_interval_from_residual(residual_ns):
    res = _to_int(residual_ns)
    return None if res is None else 1_000_000_000 + res


def _vclock_interval_ns(r: dict):
    # VCLOCK is the disciplined reference in V4; its nanosecond interval is exact.
    return 1_000_000_000

def _vclock_residual_ns(r: dict):
    return 0

def _ocxo_interval_ns(r: dict, key: str):
    campaign = r.get("campaign_delta") if isinstance(r.get("campaign_delta"), dict) else {}
    science = _path_get(campaign, f"{key}.science", {})
    direct = _to_int(science.get("clock_interval_ns")) if isinstance(science, dict) else None
    if direct is not None:
        return direct
    residual = _ocxo_residual_ns(r, key)
    return _ns_interval_from_residual(-residual) if residual is not None else None

def _ocxo_residual_ns(r: dict, key: str):
    campaign = r.get("campaign_delta") if isinstance(r.get("campaign_delta"), dict) else {}
    science = _path_get(campaign, f"{key}.science", {})
    if isinstance(science, dict):
        value = _to_int(science.get("fast_residual_ns"))
        if value is not None:
            return value
    return _to_int(_field(r, f"stats.{key}_tau_state.last_fast_residual_ns", default=None))

def _prediction_value(r: dict, lane: str, field: str, default=None):
    """Read the static-prediction surface for a clock lane.

    Supports both hierarchical V3:
      prediction.<lane>.prediction_cycles / actual_cycles / residual_cycles

    and the older flat/legacy spellings:
      <lane>_prediction_cycles / <lane>_actual_cycles / <lane>_residual_cycles
      prediction.<lane>_static_prediction_cycles / ...
    """
    raw_field = {
        "prediction_cycles": "previous_observed_cycles",
        "actual_cycles": "observed_cycles",
        "residual_cycles": "residual_cycles",
        "completed_interval_count": "completed_interval_count",
        "valid": "valid",
    }.get(field)
    if raw_field is not None:
        value = _field(r, f"raw_cycles.{lane}.{raw_field}", default=None)
        if value is not None:
            return value

    aliases = {
        "prediction_cycles": ("prediction_cycles", "static_prediction_cycles"),
        "actual_cycles":     ("actual_cycles",),
        "residual_cycles":   ("residual_cycles", "static_residual_cycles"),
        "completed_interval_count": ("completed_interval_count",),
        "valid":             ("valid", "prediction_valid", "static_prediction_valid"),
    }.get(field, (field,))

    keys = []
    for alias in aliases:
        keys.append(f"prediction.{lane}.{alias}")
        keys.append(f"prediction.{lane}_{alias}")
        keys.append(f"{lane}_{alias}")

    for key in keys:
        val = _field(r, key, default=None)
        if val is not None:
            return val
    return default


def _derived_tau_ppb(actual_cycles, prediction_cycles):
    """Derive a local interval ratio from the static-prediction surface.

    This is used only when firmware does not publish stats.<lane>.tau/ppb.
    It is intentionally not a cumulative clock-value tau; it is the best
    available truth for the local interval surface instead of spoofing 1.0.
    """
    actual = _to_float(actual_cycles)
    pred = _to_float(prediction_cycles)
    if actual is None or pred is None or pred == 0.0:
        return None, None
    tau = actual / pred
    return tau, (tau - 1.0) * 1_000_000_000.0


def _servo_state(r: dict) -> str:
    control = r.get("control") if isinstance(r.get("control"), dict) else {}
    mode = str(control.get("servo_mode") or "OFF").strip().upper()
    if mode in ("OFF", "NONE", "IDLE"):
        return "IDLE"
    return mode

# ---------------------------------------------------------------------
# DAC presentation helpers
# ---------------------------------------------------------------------

DAC_CODE_SCALE = 65536.0
DAC_VOLTAGE_DECIMALS = 9
DAC_VOLTAGE_EXTRA_DECIMALS = DAC_VOLTAGE_DECIMALS - 6


def _dac_value(r: dict, lane: str):
    return _to_float(_field(r, f"control.{lane}.target_code", default=None))

def _dac_voltage(dac_code):
    code = _to_float(dac_code)
    if code is None:
        return None
    # Presentation mirror of the firmware DAC doctrine: AD5693R internal 2.5 V
    # reference with 2x gain, scaled by the 16-bit code span.
    return code * VREF / DAC_CODE_SCALE


def _dac_label(dac_code, dac_voltage=None):
    code = _to_float(dac_code)
    volts = _to_float(dac_voltage)
    if volts is None:
        volts = _dac_voltage(code)
    if code is None or volts is None:
        return "---"
    return f"{code:>.3f} {volts:.{DAC_VOLTAGE_DECIMALS}f}V"


def _dac_tick_payload(report_dac: dict | None) -> dict:
    if isinstance(report_dac, dict) and report_dac.get("schema") == "CLOCKS_DAC_TICK_V2":
        return report_dac
    return {}


def _dac_report_lane(report_dac: dict | None, lane: str) -> dict:
    if not isinstance(report_dac, dict):
        return {}
    obj = report_dac.get(lane)
    return obj if isinstance(obj, dict) else {}


def _dac_report_value(report_dac: dict | None, lane: str):
    obj = _dac_report_lane(report_dac, lane)
    dither = obj.get("dither") if isinstance(obj.get("dither"), dict) else {}
    return _to_float(obj.get("dac") if obj.get("dac") is not None else dither.get("desired"))


def _dac_report_voltage(report_dac: dict | None, lane: str):
    obj = _dac_report_lane(report_dac, lane)

    # Retained only for explicit REPORT_DAC/back-compat payloads.  The normal
    # metrics path now reads DAC code from CLOCKS and computes presentation
    # voltage locally from the firmware DAC doctrine above.
    for key in ("v_eff", "v_target", "v", "dac_voltage"):
        val = _to_float(obj.get(key))
        if val is not None:
            return val
    return None


def _dac_current_value(r: dict, report_dac: dict | None, lane: str):
    tick_val = _dac_report_value(report_dac, lane)
    if tick_val is not None:
        return tick_val
    return _dac_value(r, lane)


def _dac_current_voltage(r: dict, report_dac: dict | None, lane: str):
    tick_volts = _dac_report_voltage(report_dac, lane)
    if tick_volts is not None:
        return tick_volts
    return _dac_voltage(_dac_current_value(r, report_dac, lane))


def _clamp_dac_code(value: float) -> float:
    if value < DAC_MIN_CODE:
        return DAC_MIN_CODE
    if value > DAC_MAX_CODE:
        return DAC_MAX_CODE
    return value


def _manual_dac_current_value(lane: str) -> float:
    """Fetch the live DAC value for keyboard nudging.

    Read the live DAC value from the current CLOCKS.clocks surface.
    Metrics does not subscribe to CLOCKS_DAC_TICK and does not poll verbose
    TEENSY REPORT_DAC during repaint or keyboard nudging.
    """
    report = {}
    try:
        p = _get_pi_clocks_report()
        report = p.get("report") if isinstance(p.get("report"), dict) else p
    except Exception:
        report = {}

    current = _dac_current_value(report, {}, lane)
    if current is None:
        raise RuntimeError(f"{lane.upper()} DAC value unavailable")

    return float(current)


def adjust_ocxo_dac(*, lane: str, direction: int, step_kind: str) -> dict:
    """Apply one keyboard DAC nudge through PI CLOCKS SET_DAC.

    The Pi command is the authority for manual DAC updates: it persists the
    SYSTEM config seed and best-effort pushes TEENSY CLOCKS SET_DAC.  Metrics
    only selects the new value and invokes that control-plane command.
    """
    if lane not in ("ocxo1", "ocxo2"):
        raise ValueError(f"unknown DAC lane: {lane!r}")
    if direction not in (-1, 1):
        raise ValueError(f"invalid DAC direction: {direction!r}")

    coarse = "coarse" in str(step_kind).lower()
    step = DAC_COARSE_STEP if coarse else DAC_FINE_STEP
    old_value = _manual_dac_current_value(lane)
    new_value = _clamp_dac_code(old_value + float(direction) * step)

    arg_name = "DAC1" if lane == "ocxo1" else "DAC2"
    resp = send_command(
        machine="PI",
        subsystem="CLOCKS",
        command="SET_DAC",
        args={arg_name: f"{new_value:.6f}"},
    )

    ok = bool(resp.get("success"))
    msg = resp.get("message") or ("OK" if ok else "FAILED")
    payload = resp.get("payload", {}) if isinstance(resp.get("payload"), dict) else {}
    teensy_pushed = payload.get("teensy_pushed_now")

    volts = _dac_voltage(new_value)
    voltage_text = f"{volts:.6f}V" if volts is not None else "---"
    prefix = "DAC" if ok else "DAC FAILED"
    suffix = "" if teensy_pushed is not False else " (persisted; Teensy push not confirmed)"

    return {
        "success": ok,
        "message": (
            f"{prefix}: {lane.upper()} {step_kind} "
            f"{old_value:.3f} -> {new_value:.3f} ({voltage_text}) "
            f"PI CLOCKS SET_DAC: {msg}{suffix}"
        ),
        "old_value": old_value,
        "new_value": new_value,
        "step": step,
        "lane": lane,
        "response": resp,
    }


def _dac_dither_summary_from_fractional_code(dac_code) -> str:
    """Format the two integer DAC codes used to realize a fractional code.

    The firmware dither doctrine is a one-second fractional-code realization:
    low=floor(code), high=ceil(code), and the fractional part determines the
    high-code duty count over 1000 scheduler ticks.  This is presentation only;
    the desired fractional code itself still comes from CLOCKS.
    """
    code = _to_float(dac_code)
    if code is None:
        return ""

    code = _clamp_dac_code(code)
    low_code = int(code)
    high_code = low_code if low_code >= int(DAC_MAX_CODE) else low_code + 1
    high_ms = int(round((code - float(low_code)) * 1000.0))
    if high_ms < 0:
        high_ms = 0
    if high_ms > 1000:
        high_ms = 1000
    low_ms = 1000 - high_ms

    return f"{low_code}:{low_ms:03d} {high_code}:{high_ms:03d}"


def _dac_timebase_dither_summary(r: dict, lane: str, dac_now=None) -> str:
    """Render the canonical CLOCKS_V4 control realization."""
    control = r.get("control") if isinstance(r.get("control"), dict) else {}
    enabled = _to_bool(control.get("dither_operator_enabled"))
    realization = str(control.get("realization_mode") or "").upper()
    if enabled is False or realization == "STATIC_ROUNDED":
        return "OFF"
    return _dac_dither_summary_from_fractional_code(
        dac_now if dac_now is not None else _dac_value(r, lane)
    ) or (realization or "ON")

def _dac_report_dither_summary(r: dict, report_dac: dict | None, lane: str, dac_now=None) -> str:
    tick = _dac_tick_payload(report_dac)
    obj = _dac_report_lane(report_dac, lane)

    if tick:
        if _to_bool(tick.get("dither")) is not True:
            return "OFF"
        low_code = _to_int(obj.get("lo"))
        high_code = _to_int(obj.get("hi"))
        high_ms = _to_int(obj.get("hi_ms"))
        if low_code is None or high_code is None or high_ms is None:
            return _dac_timebase_dither_summary(r, lane, dac_now) or "ON"
        low_ms = max(0, 1000 - high_ms)
        return f"{low_code}:{low_ms:03d} {high_code}:{high_ms:03d}"

    obj = _dac_report_lane(report_dac, lane)
    d = obj.get("dither") if isinstance(obj.get("dither"), dict) else {}
    if not d:
        return _dac_timebase_dither_summary(r, lane, dac_now)

    enabled = _to_bool(d.get("enabled"))
    rate = _to_int(d.get("rate_hz"))
    low = _to_int(d.get("last_window_low_count"))
    high = _to_int(d.get("last_window_high_count"))
    low_code = _to_int(d.get("last_window_low_code"))
    high_code = _to_int(d.get("last_window_high_code"))

    if enabled is not True:
        return "OFF"
    if rate is None and (low is None or high is None):
        return _dac_timebase_dither_summary(r, lane, dac_now) or "ON"
    if low is None or high is None:
        tb = _dac_timebase_dither_summary(r, lane, dac_now)
        return f"{rate}Hz {tb}" if tb else f"{rate}Hz"

    low_s = f"{low:03d}"
    high_s = f"{high:03d}"

    if low_code is None or high_code is None:
        return f"{rate}Hz {low_s}/{high_s}" if rate is not None else f"{low_s}/{high_s}"
    prefix = f"{rate}Hz " if rate is not None else ""
    return f"{prefix}{low_code}:{low_s} {high_code}:{high_s}"


def _dac_detail_lines(r: dict, baseline: dict | None, report_dac: dict | None) -> list[str]:
    """Render DAC telemetry as a dedicated, spacious table.

    DAC rows no longer inherit the clock-table geometry. VALUE and VOUT are
    separate columns, and every subsequent field has an explicit gutter so
    adjacent numbers can never visually merge.
    """
    lines: list[str] = []

    W_NAME = 6
    W_VALUE = 10
    W_VOUT = 13
    W_DITHER = 21
    W_MEAN = 10
    W_SD = 8
    W_SE = 8
    W_N = 7
    W_BASE = 10
    W_NOW = 10
    W_DELTA = 10
    G = " "

    lines.append(
        f"{'DAC':<{W_NAME}}"
        f"{'VALUE':>{W_VALUE}}{G}"
        f"{'VOUT':>{W_VOUT}}{G}"
        f"{'DITHER':>{W_DITHER}}{G}"
        f"{'MEAN':>{W_MEAN}}{G}"
        f"{'SD':>{W_SD}}{G}"
        f"{'SE':>{W_SE}}{G}"
        f"{'N':>{W_N}}{G}"
        f"{'BASE':>{W_BASE}}{G}"
        f"{'NOW':>{W_NOW}}{G}"
        f"{'DELTA':>{W_DELTA}}"
    )

    for name, key in (("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
        dac_now = _dac_current_value(r, report_dac, key)
        dac_voltage = _dac_current_voltage(r, report_dac, key)
        dither_summary = _dac_report_dither_summary(r, report_dac, key, dac_now)

        base_dac = None
        if baseline:
            base_dac = baseline.get("baseline_dac", {}).get(key)
            if base_dac is None:
                base_dac = baseline.get("baseline_dac_mean", {}).get(key)

        mean = _to_float(_welford_value(r, f"{key}_dac", "mean"))
        sd = _to_float(_welford_value(r, f"{key}_dac", "stddev"))
        se = _to_float(_welford_value(r, f"{key}_dac", "stderr"))
        wn = _to_int(_welford_value(r, f"{key}_dac", "n"))
        if wn == 0:
            mean = sd = se = wn = None

        delta = None
        if base_dac is not None and dac_now is not None:
            delta = float(dac_now) - float(base_dac)

        lines.append(
            f"{name:<{W_NAME}}"
            f"{_fmt(dac_now, f'>{W_VALUE}.3f', W_VALUE)}{G}"
            f"{_fmt(dac_voltage, f'>{W_VOUT}.9f', W_VOUT)}{G}"
            f"{dither_summary:>{W_DITHER}}{G}"
            f"{_fmt(mean, f'>{W_MEAN}.3f', W_MEAN)}{G}"
            f"{_fmt(sd, f'>{W_SD}.3f', W_SD)}{G}"
            f"{_fmt(se, f'>{W_SE}.3f', W_SE)}{G}"
            f"{_fmt(wn, f'>{W_N}d', W_N)}{G}"
            f"{_fmt(base_dac, f'>{W_BASE}.3f', W_BASE)}{G}"
            f"{_fmt(dac_now, f'>{W_NOW}.3f', W_NOW)}{G}"
            f"{_fmt(delta, f'>+{W_DELTA}.3f', W_DELTA)}"
        )

    return lines


# ---------------------------------------------------------------------
# Mission-control feature grid
# ---------------------------------------------------------------------

def _feature_status_at(tree: dict, name: str, default: str = "---") -> str:
    cur = tree if isinstance(tree, dict) else {}
    for part in name.split("."):
        if not isinstance(cur, dict):
            return default
        cur = cur.get(part.upper())
        if cur is None:
            return default

    return _feature_status_label(cur)


def _feature_status_label(value) -> str:
    s = str(value or "---").strip().upper()
    if s == "DOWN":
        return "ANOMALY"
    if s in ("INITIALIZING", "NOMINAL", "HOLD", "ANOMALY"):
        return s
    return s or "---"


def feature_status_grid_lines() -> list[str]:
    features, error = _get_feature_status_payload()

    lines = []

    if error and not features:
        lines.append(f"UNAVAILABLE: {error}")
        lines.append("")
        return lines

    cells = []
    for label, path in FEATURE_STATUS_GRID:
        cells.append(f"{label}: {_feature_status_at(features, path)}")

    for row in range(0, len(cells), FEATURE_GRID_COLUMNS):
        chunk = cells[row:row + FEATURE_GRID_COLUMNS]
        lines.append(
            "".join(
                f"{cell:<{FEATURE_GRID_CELL_WIDTH}}" for cell in chunk
            ).rstrip()
        )

    if error:
        lines.append(f"FEATURE_STATUS TAP ERROR: {error[:120]}")

    lines.append("")
    return lines


def _feature_subtree_health(tree: dict, path: str) -> str:
    node = _path_get(tree, path, None)
    if not isinstance(node, dict) or not node:
        return "?"
    leaves = []
    stack = [node]
    while stack:
        item = stack.pop()
        for value in item.values():
            if isinstance(value, dict):
                stack.append(value)
            else:
                leaves.append(str(value).strip().upper())
    if not leaves:
        return "?"
    if any(value in {"ANOMALY", "DOWN"} for value in leaves):
        return "ANOMALY"
    if any(value in {"HOLD", "INITIALIZING"} for value in leaves):
        return "HOLD"
    return "NOMINAL" if all(value == "NOMINAL" for value in leaves) else "?"


# ---------------------------------------------------------------------
# Status header
# ---------------------------------------------------------------------

def status_header() -> str:
    try:
        s = _get_system_snapshot()
        net = s.get("network", {}).get("network_status", "?")
        pi_health = s.get("pi", {}).get("health_state", "?")
        features = s.get("features") if isinstance(s.get("features"), dict) else {}
        teensy_health = _feature_subtree_health(features, "TEENSY")
        try:
            clocks_report = _get_pi_clocks_report()
        except Exception:
            clocks_report = {}
        gnss_status = _gnss_status(clocks_report, s)
        gnss_lock = gnss_status.get("lock_quality", "?")

        bat_v = "?"
        power = s.get("power", {})
        for bus_key, devices in power.items():
            if not str(bus_key).startswith("i2c-") or not isinstance(devices, dict):
                continue
            for rail in devices.values():
                if isinstance(rail, dict) and str(rail.get("label", "")).lower() == "battery":
                    volts = _to_float(rail.get("volts"))
                    if volts is not None:
                        bat_v = f"{volts:.2f}V"

        return (
            f" NET: {net}"
            f"  BAT: {bat_v}"
            f"  PI: {pi_health}"
            f"  TEENSY: {teensy_health}"
            f"  GNSS: {gnss_lock}"
        )
    except Exception:
        return " STATUS: UNAVAILABLE"

# ---------------------------------------------------------------------
# Baseline comparison helper
# ---------------------------------------------------------------------

def _baseline_comp(base_val, now_val, width=9):
    if base_val is not None and now_val is not None:
        delta = float(now_val) - float(base_val)
        return (
            f"{_fmt(base_val, f'>{width}.3f', width)}"
            f"{_fmt(now_val, f'>{width}.3f', width)}"
            f"{_fmt(delta, f'>+{width}.3f', width)}"
        )
    return f"{'---':>{width}}{'---':>{width}}{'---':>{width}}"


# ---------------------------------------------------------------------
# Welford row renderer — reads firmware-owned statistics verbatim
# ---------------------------------------------------------------------

def _welford_cols_fragment(r, prefix, w_mean, w_sd, w_se, w_n, mean_decimals=3):
    mean = _to_float(_welford_value(r, prefix, "mean"))
    sd   = _to_float(_welford_value(r, prefix, "stddev"))
    se   = _to_float(_welford_value(r, prefix, "stderr"))
    wn   = _to_int  (_welford_value(r, prefix, "n"))
    if wn == 0:
        mean = sd = se = wn = None
    return (
        f"{_fmt(mean, f'>{w_mean}.{mean_decimals}f', w_mean)}"
        f"{_fmt(sd,   f'>{w_sd}.3f',   w_sd)}"
        f"{_fmt(se,   f'>{w_se}.3f',   w_se)}"
        f"{_fmt(wn,   f'>{w_n}d',      w_n)}"
    )


def _welford_cols_extra(r, prefix, w_mean, w_sd, w_se, w_n, mean_decimals=3):
    mean = _to_float(_extra(r, f"{prefix}_welford_mean"))
    sd   = _to_float(_extra(r, f"{prefix}_welford_stddev"))
    se   = _to_float(_extra(r, f"{prefix}_welford_stderr"))
    wn   = _to_int  (_extra(r, f"{prefix}_welford_n"))
    if wn == 0:
        mean = sd = se = wn = None
    return (
        f"{_fmt(mean, f'>{w_mean}.{mean_decimals}f', w_mean)}"
        f"{_fmt(sd,   f'>{w_sd}.3f',   w_sd)}"
        f"{_fmt(se,   f'>{w_se}.3f',   w_se)}"
        f"{_fmt(wn,   f'>{w_n}d',      w_n)}"
    )


def _welford_cols_zero(w_mean, w_sd, w_se, w_n, n=None, mean_decimals=3):
    """Display a definitionally zero Welford row.

    Used for reference-style rows where the visible residual surface is fixed at
    zero by construction.  This is display normalization, not Pi-side stats.
    """
    wn = _to_int(n)
    return (
        f"{_fmt(0.0, f'>{w_mean}.{mean_decimals}f', w_mean)}"
        f"{_fmt(0.0, f'>{w_sd}.3f',   w_sd)}"
        f"{_fmt(0.0, f'>{w_se}.3f',   w_se)}"
        f"{_fmt(wn,  f'>{w_n}d',     w_n)}"
    )


def _welford_cols_fragment_or_zero(r, prefix, w_mean, w_sd, w_se, w_n, mean_decimals=3):
    """Render firmware Welford stats, falling back to zero values with no N.

    Older TIMEBASE rows may omit a reference-style Welford.  Zero-valued
    MEAN/SD/SE remain truthful, but campaign PPS count is not a statistical
    population and must never be substituted for Welford N.
    """
    mean = _to_float(_welford_value(r, prefix, "mean"))
    sd   = _to_float(_welford_value(r, prefix, "stddev"))
    se   = _to_float(_welford_value(r, prefix, "stderr"))
    wn   = _to_int  (_welford_value(r, prefix, "n"))

    if (mean is None and sd is None and se is None and wn is None) or not wn:
        return _welford_cols_zero(w_mean, w_sd, w_se, w_n, n=None, mean_decimals=mean_decimals)

    return (
        f"{_fmt(mean, f'>{w_mean}.{mean_decimals}f', w_mean)}"
        f"{_fmt(sd,   f'>{w_sd}.3f',   w_sd)}"
        f"{_fmt(se,   f'>{w_se}.3f',   w_se)}"
        f"{_fmt(wn,   f'>{w_n}d',      w_n)}"
    )



def _dwt_expected_cycles(r: dict):
    """Return the firmware-authored PPS static prediction in DWT cycles.

    CLOCKS_V4 carries this as
    raw_cycles.pps.previous_observed_cycles. Older prediction/FloorLine aliases
    remain readable, but the current ACTUAL interval is never used as EXPECTED.
    """
    explicit = _prediction_value(r, "pps", "prediction_cycles")
    if explicit is not None:
        return _to_int(explicit)

    return _to_int(_field(
        r,
        # Transitional explicit DWT/GNSS prediction aliases.
        "prediction.dwt.prediction_cycles",
        "dwt.prediction_cycles",
        "dwt.static_prediction_cycles",
        "dwt_prediction_cycles",
        "dwt_static_prediction_cycles",
        # Older FloorLine prediction surfaces retained only for historical rows.
        "vclock.science.clock_floorline_interval_cycles",
        "vclock.science.vclock_floorline_interval_cycles",
        "vclock.science.floorline_interval_cycles",
        "vclock.science.clock_floorline.endpoint_interval_cycles",
        "vclock.science.floorline.endpoint_interval_cycles",
        "forensics.v_fl_cyc",
        "forensics.v_court_fl_int",
        "v_fl_cyc",
        "v_court_fl_int",
        "vclock_floorline_interval_cycles",
        "vclock_fl_cyc",
        "gnss_floorline_interval_cycles",
        "gnss_fl_cyc",
        "prediction.gnss.floorline_cycles",
        "prediction.gnss.floorline_interval_cycles",
        "prediction.gnss.floorline_prediction_cycles",
        "prediction.gnss.floorline.prediction_cycles",
        "prediction.gnss.floorline.interval_cycles",
        "prediction.pps.floorline_prediction_cycles",
        "prediction.pps.floorline_interval_cycles",
    ))


# ---------------------------------------------------------------------
# Campaign history readout
# ---------------------------------------------------------------------


def _get_campaign_rows() -> list[dict]:
    """Return typed TEMPEST campaign masters as a direct read model.

    ``campaign_master.payload.report`` is the compact campaign-list projection.
    The UI performs no campaign_detail lookup and no per-campaign aggregation.
    A campaign without a report remains visible with empty science columns.
    """
    with open_db(row_dict=True) as conn:
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT
                    master.id,
                    master.ts,
                    master.campaign_type,
                    master.campaign,
                    master.active,
                    master.payload ? 'report' AS report_present,
                    (master.payload #>> '{report,science_eligible}')::boolean
                        AS viable,
                    master.payload #>> '{report,sequence}'
                        AS sequence,
                    master.payload #>> '{report,public_count}'
                        AS pps_count,

                    master.payload #>> '{report,fragment,clockfaces,ocxo1_ns}'
                        AS ocxo1_ns,
                    master.payload #>> '{report,clocks,stats,ocxo1,ppb_buckets,10_min}'
                        AS ocxo1_ppb_10_min,
                    master.payload #>> '{report,clocks,stats,ocxo1,ppb_buckets,60_min}'
                        AS ocxo1_ppb_60_min,
                    master.payload #>> '{report,clocks,stats,ocxo1,ppb_buckets,8_hour}'
                        AS ocxo1_ppb_8_hour,
                    master.payload #>> '{report,clocks,stats,ocxo1,ppb_buckets,24_hour}'
                        AS ocxo1_ppb_24_hour,
                    COALESCE(
                        master.payload #>> '{report,clocks,stats,ocxo1,ppb_buckets,total}',
                        master.payload #>> '{report,clocks,stats,ocxo1,ppb}'
                    ) AS ocxo1_ppb_total,
                    master.payload #>> '{report,fragment,stats,ppb,ocxo1}'
                        AS ocxo1_ppb_campaign,
                    master.payload #>> '{report,fragment,ocxo1,science,fast_residual_ns}'
                        AS ocxo1_residual,
                    master.payload #>> '{report,clocks,stats,ocxo1,welford,mean}'
                        AS ocxo1_mean,
                    master.payload #>> '{report,clocks,stats,ocxo1,welford,stddev}'
                        AS ocxo1_stddev,
                    master.payload #>> '{report,clocks,stats,ocxo1,welford,stderr}'
                        AS ocxo1_stderr,
                    master.payload #>> '{report,clocks,stats,ocxo1,welford,n}'
                        AS ocxo1_n,

                    master.payload #>> '{report,fragment,clockfaces,ocxo2_ns}'
                        AS ocxo2_ns,
                    master.payload #>> '{report,clocks,stats,ocxo2,ppb_buckets,10_min}'
                        AS ocxo2_ppb_10_min,
                    master.payload #>> '{report,clocks,stats,ocxo2,ppb_buckets,60_min}'
                        AS ocxo2_ppb_60_min,
                    master.payload #>> '{report,clocks,stats,ocxo2,ppb_buckets,8_hour}'
                        AS ocxo2_ppb_8_hour,
                    master.payload #>> '{report,clocks,stats,ocxo2,ppb_buckets,24_hour}'
                        AS ocxo2_ppb_24_hour,
                    COALESCE(
                        master.payload #>> '{report,clocks,stats,ocxo2,ppb_buckets,total}',
                        master.payload #>> '{report,clocks,stats,ocxo2,ppb}'
                    ) AS ocxo2_ppb_total,
                    master.payload #>> '{report,fragment,stats,ppb,ocxo2}'
                        AS ocxo2_ppb_campaign,
                    master.payload #>> '{report,fragment,ocxo2,science,fast_residual_ns}'
                        AS ocxo2_residual,
                    master.payload #>> '{report,clocks,stats,ocxo2,welford,mean}'
                        AS ocxo2_mean,
                    master.payload #>> '{report,clocks,stats,ocxo2,welford,stddev}'
                        AS ocxo2_stddev,
                    master.payload #>> '{report,clocks,stats,ocxo2,welford,stderr}'
                        AS ocxo2_stderr,
                    master.payload #>> '{report,clocks,stats,ocxo2,welford,n}'
                        AS ocxo2_n
                FROM campaign_master AS master
                WHERE master.campaign_type = 'TEMPEST'
                ORDER BY master.ts DESC, master.id DESC
                """
            )
            return list(cur.fetchall())


def _campaign_cell(name: str, active: bool, width: int) -> str:
    """Return one bounded campaign label, reserving the final cell for ``*``."""
    marker = "*" if active else ""
    usable = max(1, width - len(marker))
    label = str(name or "?")
    if len(label) > usable:
        label = label[:max(1, usable - 1)] + "~"
    return f"{label + marker:<{width}}"


def campaigns_readout() -> list[str]:
    """Render each campaign with the same OCXO science fields as CLOCKS."""
    try:
        rows = _get_campaign_rows()
    except Exception as e:
        return ["\0CAMPAIGNS:ERROR", f"CAMPAIGNS: UNAVAILABLE: {e}"]

    newest_identity = str(rows[0].get("id")) if rows else "EMPTY"
    lines = [f"\0CAMPAIGN:{newest_identity}"]

    W_CAMPAIGN = 20
    W_DEV = 6
    W_VALUE = 19
    W_PPB_BUCKET = 9
    W_RES = 6
    W_MEAN = 9
    W_SD = 7
    W_SE = 7
    W_N = 7
    G = " "

    lines.append(
        f"{'CAMPAIGN':<{W_CAMPAIGN}}{G}"
        f"{'DEV':<{W_DEV}}"
        f"{'VALUE':>{W_VALUE}}"
        f"{'10-MIN':>{W_PPB_BUCKET}}"
        f"{'60-MIN':>{W_PPB_BUCKET}}"
        f"{'8-HOUR':>{W_PPB_BUCKET}}"
        f"{'24-HOUR':>{W_PPB_BUCKET}}"
        f"{'TOTAL':>{W_PPB_BUCKET}}"
        f"{'CAMP':>{W_PPB_BUCKET}}"
        f"{'RES':>{W_RES}}"
        f"{'MEAN':>{W_MEAN}}"
        f"{'SD':>{W_SD}}"
        f"{'SE':>{W_SE}}"
        f"{'N':>{W_N}}"
    )

    if not rows:
        lines.append("")
        lines.append("NO CAMPAIGNS")
        return lines

    for row_index, row in enumerate(rows):
        campaign_name = str(row.get("campaign") or "?")
        if not row.get("report_present"):
            campaign_name += " [WAIT]"
        elif row.get("viable") is False:
            campaign_name += " [X]"
        campaign_cell = _campaign_cell(
            campaign_name,
            bool(row.get("active")),
            W_CAMPAIGN,
        )

        for lane_name, lane_key in (("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
            value = _to_int(row.get(f"{lane_key}_ns"))
            ppb_values = (
                _to_float(row.get(f"{lane_key}_ppb_10_min")),
                _to_float(row.get(f"{lane_key}_ppb_60_min")),
                _to_float(row.get(f"{lane_key}_ppb_8_hour")),
                _to_float(row.get(f"{lane_key}_ppb_24_hour")),
                _to_float(row.get(f"{lane_key}_ppb_total")),
                _to_float(row.get(f"{lane_key}_ppb_campaign")),
            )
            residual = _to_int(row.get(f"{lane_key}_residual"))
            mean = _to_float(row.get(f"{lane_key}_mean"))
            stddev = _to_float(row.get(f"{lane_key}_stddev"))
            stderr = _to_float(row.get(f"{lane_key}_stderr"))
            n = _to_int(row.get(f"{lane_key}_n"))

            lines.append(
                f"{campaign_cell}{G}"
                f"{lane_name:<{W_DEV}}"
                f"{_comma_int(value, W_VALUE)}"
                f"{''.join(_fmt(v, f'>{W_PPB_BUCKET}.3f', W_PPB_BUCKET) for v in ppb_values)}"
                f"{_sign_int(residual, W_RES)}"
                f"{_fmt(mean, f'>{W_MEAN}.3f', W_MEAN)}"
                f"{_fmt(stddev, f'>{W_SD}.3f', W_SD)}"
                f"{_fmt(stderr, f'>{W_SE}.3f', W_SE)}"
                f"{_fmt(n, f'>{W_N}d', W_N)}"
            )

        if row_index != len(rows) - 1:
            lines.append("")

    return lines



def _clockface_value(r: dict, lane: str):
    """Return the operator clockface in the currently active coordinate scope."""
    if lane == "vclock":
        # V4 deliberately has no duplicate VCLOCK ns clockface; GNSS is the
        # canonical disciplined nanosecond reference in both coordinate scopes.
        lane = "gnss"

    field = {
        "gnss": "gnss_ns",
        "ocxo1": "ocxo1_ns",
        "ocxo2": "ocxo2_ns",
        "dwt": "dwt_cycles",
    }.get(lane)
    if field is None:
        return None

    campaign = r.get("campaign_delta") if isinstance(r.get("campaign_delta"), dict) else {}
    if r.get("campaign_present"):
        clockfaces = campaign.get("clockfaces") if isinstance(campaign.get("clockfaces"), dict) else {}
        return _to_int(clockfaces.get(field))

    return _to_int(_field(r, f"clockfaces.{field}", default=None))

def _campaign_ppb(r: dict, lane: str):
    """Return firmware-authored TEMPEST campaign PPB from the optional delta."""
    campaign = r.get("campaign_delta") if isinstance(r.get("campaign_delta"), dict) else {}
    if not campaign or str(campaign.get("state") or "STOPPED").upper() != "STARTED":
        return None
    if lane == "gnss_raw":
        adjudication = campaign.get("adjudication") if isinstance(campaign.get("adjudication"), dict) else {}
        return _to_float(_path_get(adjudication, "extra_clocks.gnss_raw_ppb", None))
    return _to_float(_path_get(campaign, f"stats.ppb.{lane}", None))

def _monitor_count(r: dict) -> int:
    value = _to_int(_field(r, "stats.ocxo1.welford.n", default=None))
    if value is not None:
        return value
    return _to_int(r.get("instrument_age_seconds")) or 0

# ---------------------------------------------------------------------
# Combined clocks readout
# ---------------------------------------------------------------------

def clocks_combined_readout() -> list[str]:
    lines = []

    try:
        p = _get_pi_clocks_report()
    except Exception:
        return ["CLOCKS: UNAVAILABLE"]

    report_dac = None
    try:
        report_dac = _get_pi_clocks_report_dac()
    except Exception:
        report_dac = None

    report = p.get("report") if isinstance(p.get("report"), dict) else p
    if not isinstance(report, dict) or not report:
        return ["CLOCKS: FEED UNAVAILABLE", "", *feature_status_grid_lines()]

    r = report
    state = str(r.get("campaign_state") or ("STARTED" if r.get("campaign_present") else "STOPPED")).upper()
    campaign = r.get("campaign") or "STOPPED"
    elapsed = r.get("campaign_elapsed") or "00:00:00"
    instrument_elapsed = r.get("instrument_elapsed") or "00:00:00"
    n = _monitor_count(r)

    servo_state = str(report_dac.get("servo") or _servo_state(r)).upper() if isinstance(report_dac, dict) else _servo_state(r)

    baseline = _get_clocks_baseline()
    baseline_ppb = baseline.get("baseline_ppb", {}) if baseline else {}
    baseline_id = baseline.get("baseline_id", "?") if baseline else None
    baseline_campaign = baseline.get("baseline_campaign", "?") if baseline else None

    servo_str = servo_state
    baseline_str = f"BASELINE: {baseline_campaign}" if baseline_id else "BASELINE: NONE"

    if state == "STARTED" or r.get("campaign_present"):
        identity = f"CAMPAIGN: {campaign}  ELAPSED: {elapsed}  n={n}"
    else:
        identity = f"CAMPAIGN: STOPPED  INSTRUMENT: {instrument_elapsed}  n={n}"
    lines.append(identity + f"    SERVO: {servo_str}" + f"    {baseline_str}")
    lines.append("")

    # ── Column widths ──
    W_NAME  = 6
    W_VALUE = 19
    W_PPB_BUCKET = 9
    W_RES   = 6
    W_MEAN  = 9
    W_SD    = 7
    W_SE    = 7
    W_N     = 6
    W_BASE  = 9
    W_NOW   = 9
    W_DELTA = 9

    # ── CLK header ──
    lines.append(
        f"{'CLK':<{W_NAME}}"
        f"{'VALUE':>{W_VALUE}}"
        f"{'10-MIN':>{W_PPB_BUCKET}}"
        f"{'60-MIN':>{W_PPB_BUCKET}}"
        f"{'8-HOUR':>{W_PPB_BUCKET}}"
        f"{'24-HOUR':>{W_PPB_BUCKET}}"
        f"{'TOTAL':>{W_PPB_BUCKET}}"
        f"{'CAMP':>{W_PPB_BUCKET}}"
        f"{'RES':>{W_RES}}"
        f"{'MEAN':>{W_MEAN}}"
        f"{'SD':>{W_SD}}"
        f"{'SE':>{W_SE}}"
        f"{'N':>{W_N}}"
        f" "
        f"{'BASE':>{W_BASE}}"
        f"{'NOW':>{W_NOW}}"
        f"{'DELTA':>{W_DELTA}}"
    )

    # ── GNSS (reference nanosecond clock) ──
    gnss_ns = _clockface_value(r, "gnss")
    gnss_total_ppb = _ppb_bucket(r, "gnss", "total")
    gnss_res = _to_int(_field(r, "gnss.second_residual_ns", "gnss_residual_ns")) or 0
    lines.append(
        f"{'GNSS':<{W_NAME}}"
        f"{_comma_int(gnss_ns, W_VALUE)}"
        f"{_ppb_cols_fragment(r, 'gnss', W_PPB_BUCKET)}"
        f"{_sign_int(gnss_res, W_RES)}"
        f"{_welford_cols_fragment(r, 'gnss', W_MEAN, W_SD, W_SE, W_N)}"
        f" {_baseline_comp(baseline_ppb.get('gnss'), gnss_total_ppb, W_BASE)}"
    )

    # ── VCLOCK (GNSS-disciplined nanosecond clock) ──
    vclock_ns  = _clockface_value(r, "vclock")
    vclock_total_ppb = _ppb_bucket(r, "vclock", "total")
    vclock_res = _vclock_residual_ns(r)

    if vclock_res is None:
        vclock_res = 0

    lines.append(
        f"{'VCLOCK':<{W_NAME}}"
        f"{_comma_int(vclock_ns, W_VALUE)}"
        f"{_ppb_cols_fragment(r, 'vclock', W_PPB_BUCKET)}"
        f"{_sign_int(vclock_res, W_RES)}"
        f"{_welford_cols_fragment_or_zero(r, 'vclock', W_MEAN, W_SD, W_SE, W_N)}"
        f" {_baseline_comp(baseline_ppb.get('vclock'), vclock_total_ppb, W_BASE)}"
    )

    # ── OCXO1, OCXO2 (nanosecond clocks) ──
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        ocxo_ns = _clockface_value(r, key)
        total_ppb = _ppb_bucket(r, key, "total")
        res          = _ocxo_residual_ns(r, key)
        lines.append(
            f"{name:<{W_NAME}}"
            f"{_comma_int(ocxo_ns, W_VALUE)}"
            f"{_ppb_cols_fragment(r, key, W_PPB_BUCKET)}"
            f"{_sign_int(res, W_RES)}"
            f"{_welford_cols_fragment(r, key, W_MEAN, W_SD, W_SE, W_N)}"
            f" {_baseline_comp(baseline_ppb.get(key), total_ppb, W_BASE)}"
        )

    # GN_RAW and DWT remain available in CLOCKS and focused reports, but are
    # intentionally omitted from this operator-facing clock table.

    # ── DAC detail ──
    lines.append("")
    lines.extend(_dac_detail_lines(r, baseline, report_dac))

    lines.append("")

    # ── Time ──
    gnss_time = r.get("gnss_time_utc") or _field(r, "time.gnss", default="---")
    system_time = r.get("system_time_utc") or r.get("published_at_utc") or _field(r, "time.system", default="---")
    lines.append(f"TIME  GNSS: {gnss_time}    SYSTEM: {system_time}")
    lines.append("")

    # ── DWT detail ──
    dwt_actual = _to_int(_field(r, "raw_cycles.pps.observed_cycles", default=None))
    dwt_residual = _to_int(_field(
        r,
        "dwt.second_residual_cycles",
        "raw_cycles.pps.residual_cycles",
        "dwt_second_residual_cycles",
    ))

    # EXPECTED is the next-second DWT-cycle prediction.  Contemporary firmware
    # uses the prior completed GNSS/PPS interval as its static prediction, while
    # older rows may still expose an explicit prediction or FloorLine witness.
    dwt_expected = _dwt_expected_cycles(r)

    dwt_at_anchor = _field(r, "anchor.dwt_at_pps_vclock", default=None)
    counter32 = _field(r, "anchor.counter32_at_pps_vclock", default=None)
    if any(v is not None for v in [dwt_actual, dwt_expected, dwt_at_anchor, counter32]):
        dwt_label_w = 12
        dwt_num_w = 20
        edge_label_w = 11
        edge_num_w = 14
        lines.append(
            f"DWT   "
            f"{'EXPECTED:':>{dwt_label_w}} {_comma_int(dwt_expected, dwt_num_w)}"
            f"    {'':<{dwt_label_w}} {'':>{dwt_num_w}}"
            f"    {'DWT@PPS/V:':<{edge_label_w}} {_comma_int(dwt_at_anchor, edge_num_w)}"
        )
        lines.append(
            f"      "
            f"{'ACTUAL:':>{dwt_label_w}} {_comma_int(dwt_actual, dwt_num_w)}"
            f"    {'':<{dwt_label_w}} {'':>{dwt_num_w}}"
            f"    {'CTR32:':<{edge_label_w}} {_comma_int(counter32, edge_num_w)}"
        )
        lines.append("")

    # ── GNSS status ──
    try:
        snapshot = _get_system_snapshot()
    except Exception:
        snapshot = {}

    gnss = _gnss_status(r, snapshot)
    freq_mode_name = gnss.get("freq_mode_name") or "?"
    pos_mode = gnss.get("pos_mode") or "?"
    traim = gnss.get("traim") or "?"
    sats = gnss.get("satellites") if gnss.get("satellites") is not None else "?"
    hdop = gnss.get("hdop")
    hdop_str = f"{float(hdop):.2f}" if hdop is not None else "---"
    lat = gnss.get("latitude_deg")
    lon = gnss.get("longitude_deg")
    alt_gnss = gnss.get("altitude_m")
    ellipsoid = gnss.get("ellipsoid_height_m")
    geoid = gnss.get("geoid_sep_m")
    gnss_line = (
        f"GNSS  MODE: {pos_mode}"
        f"  DISC: {freq_mode_name}"
        f"  SATS: {sats}"
        f"  HDOP: {hdop_str}"
        f"  TRAIM: {traim}"
    )
    pos_parts = []
    if lat is not None and lon is not None:
        pos_parts.append(f"LAT: {float(lat):.6f}")
        pos_parts.append(f"LON: {float(lon):.6f}")
    if alt_gnss is not None:
        pos_parts.append(f"ALT(MSL): {float(alt_gnss):.1f}m")
    if ellipsoid is not None:
        pos_parts.append(f"ELLIP: {float(ellipsoid):.1f}m")
    if geoid is not None:
        pos_parts.append(f"GEOID: {float(geoid):.1f}m")
    if pos_parts:
        gnss_line += "  " + "  ".join(pos_parts)
    lines.append(gnss_line)
    lines.append("")

    # ── Environment ──
    env = snapshot.get("environment", {})
    pi_data = snapshot.get("pi", {})
    teensy_data = snapshot.get("teensy", {})
    temp_c = env.get("temperature_c")
    humidity = env.get("humidity_pct")
    pressure = env.get("pressure_hpa")
    baro_alt = env.get("altitude_m")
    pi_temp = pi_data.get("cpu_temp_c")
    teensy_temp = teensy_data.get("cpu_temp_c")
    env_parts = []
    if temp_c is not None:
        env_parts.append(f"AMBIENT: {temp_c:.1f}°C")
    if humidity is not None:
        env_parts.append(f"RH: {humidity:.0f}%")
    if pressure is not None:
        env_parts.append(f"BARO: {pressure:.1f}hPa")
    if baro_alt is not None:
        env_parts.append(f"BARO_ALT: {baro_alt:.1f}m")
    if pi_temp is not None:
        env_parts.append(f"PI: {pi_temp:.1f}°C")
    if teensy_temp is not None:
        env_parts.append(f"TEENSY: {teensy_temp:.1f}°C")
    lines.append("ENV   " + "  ".join(env_parts))
    lines.append("")

    lines.extend(feature_status_grid_lines())

    return lines


READOUTS = [
    ("CLOCKS", lambda: _safe_lines(clocks_combined_readout), False),
    ("CAMPAIGNS", lambda: _safe_lines(campaigns_readout), True),
]
