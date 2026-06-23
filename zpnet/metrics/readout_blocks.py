"""
ZPNet Metrics Readout Blocks — Dense Clocks Panel

Data source:
  The Pi CLOCKS report is the most recent TIMEBASE record plus campaign_state.
  TIMEBASE_FRAGMENT_V2 is hierarchical: canonical clock state lives in the
  fragment object, with lane-owned objects for dwt, vclock, ocxo1, ocxo2,
  stats, dac, prediction, and pps.

Stats policy (Pi is a stenographer):
  Every statistical quantity shown in this panel is read verbatim from the
  Teensy-authored TIMEBASE_FRAGMENT, except the GNSS reference row, whose
  residual/Welford surface is definitionally zero for display consistency.
  No Pi-side means, stddevs, stderrs, or residuals are computed here.  The only
  Pi-side arithmetic is presentation conversion (DAC code → voltage) and
  baseline delta (NOW - BASE), neither of which is a statistic.

Clock row doctrine:
  VALUE is the canonical 64-bit clock value.
  RAW is the firmware-published one-second interval surface.  For standard
  nanosecond clocks (GNSS, VCLOCK, OCXO1, OCXO2), RAW is nanoseconds added to
  that clock during the prior GNSS/PPS second.  DWT is the deliberate exception:
  its VALUE and RAW fields are cycle-oriented.
  RES is the firmware-published residual for that interval.

  For OCXO lanes, RAW/RES come from ocxoN.pps_residual, not the older
  edge-domain bridge measurement.  That keeps the metrics panel locked to the
  same PPS-founded residual doctrine used by TIMEBASE statistics and servo
  input.

Column layout (CLK rows):
  NAME   VALUE   TAU   PPB   RAW   RES   MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (DAC rows):
  NAME   DAC_VALUE_VOLTAGE   (blanks)   MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (INT rows):
  NAME   END_GNSS_NS   DELTA_NS
"""

import json
import socket
import threading
import time

from zpnet.processes.processes import send_command

VREF = 3.003

DAC_FINE_STEP = 1.0
DAC_COARSE_STEP = 10.0
DAC_MIN_CODE = 0.0
DAC_MAX_CODE = 65535.0

FEATURE_POLL_INTERVAL_S = 5.0
FEATURE_GRID_COLUMNS = 4
FEATURE_GRID_CELL_WIDTH = 39

PUBSUB_TAP_SOCKET = "/tmp/zpnet_pubsub_tap.sock"
DAC_TICK_TOPIC = "CLOCKS_DAC_TICK"
DAC_TICK_STALE_S = 5.0
DAC_TICK_RECONNECT_S = 1.0

# Mission-control readiness board.  The feature payload remains scalar-only;
# this table is just the operator-facing projection of the global PI SYSTEM
# feature tree into the CLOCKS panel's empty space.
FEATURE_STATUS_GRID = (
    ("NET", "PI.SYSTEM.NETWORK"),
    ("BATTERY", "PI.SYSTEM.BATTERY"),
    ("GNSS", "PI.GNSS.REPORT"),
    ("PI_HOST", "PI.SYSTEM.HOST"),
    ("POWER", "PI.SYSTEM.POWER"),
    ("T_IMPORT", "PI.SYSTEM.TEENSY_FEATURE_IMPORT"),
    ("T_FEATURE", "TEENSY.SYSTEM.FEATURE_STATUS"),
    ("PPS/V_AUTH", "TEENSY.INTERRUPT.PPS_VCLOCK_AUTHORITY"),
    ("FLOORLINE", "TEENSY.INTERRUPT.FLOORLINE"),
    ("QTIMER_CNT", "TEENSY.INTERRUPT.QTIMER_COUNTER_CUSTODY"),
    ("QTIMER_DWT", "TEENSY.INTERRUPT.QTIMER_DWT_RULER"),
    ("CTR32_LINE", "TEENSY.INTERRUPT.COUNTER32_LINEAGE"),
    ("DWT_CAL", "TEENSY.CLOCKS.DWT_CALIBRATION"),
    ("STATIC_PRED", "TEENSY.CLOCKS.STATIC_PREDICTION"),
    ("SMARTZERO", "TEENSY.CLOCKS.SMARTZERO"),
    ("ALPHA_EPOCH", "TEENSY.CLOCKS.ALPHA_EPOCH"),
    ("OCXO_ORIGIN", "TEENSY.CLOCKS.OCXO_PUBLIC_ORIGIN"),
    ("SCI_RES", "TEENSY.CLOCKS.SCIENCE_RESIDUALS"),
    ("TB_PUB", "TEENSY.CLOCKS.TIMEBASE_PUBLICATION"),
    ("ENV", "PI.SYSTEM.ENVIRONMENT"),
    ("SENSORS", "PI.SYSTEM.SENSORS"),
)

_FEATURE_STATUS_CACHE: dict | None = None
_FEATURE_STATUS_CACHE_TS = 0.0
_FEATURE_STATUS_CACHE_ERROR: str | None = None

_DAC_TICK_CACHE: dict | None = None
_DAC_TICK_CACHE_TS = 0.0
_DAC_TICK_CACHE_ERROR: str | None = None
_DAC_TICK_THREAD: threading.Thread | None = None
_DAC_TICK_LOCK = threading.Lock()


# ---------------------------------------------------------------------
# Data fetchers
# ---------------------------------------------------------------------

def _get_system_snapshot() -> dict:
    return send_command(machine="PI", subsystem="SYSTEM", command="REPORT")["payload"]


def _get_feature_status_payload(force: bool = False) -> tuple[dict, str | None]:
    """Return the global PI SYSTEM feature tree using command/response.

    Metrics repaints once per second, but feature readiness is human-facing
    state.  Poll the command path at a slower cadence so SYSTEM.FEATURES stays
    explicit, current, and cheap.  If a poll fails after a good read, keep the
    last good tree visible and mark the board stale.
    """
    global _FEATURE_STATUS_CACHE, _FEATURE_STATUS_CACHE_TS, _FEATURE_STATUS_CACHE_ERROR

    now = time.monotonic()
    if (
        not force
        and _FEATURE_STATUS_CACHE is not None
        and (now - _FEATURE_STATUS_CACHE_TS) < FEATURE_POLL_INTERVAL_S
    ):
        return _FEATURE_STATUS_CACHE, _FEATURE_STATUS_CACHE_ERROR

    try:
        resp = send_command(machine="PI", subsystem="SYSTEM", command="FEATURES")
        if not resp.get("success", True):
            raise RuntimeError(resp.get("message") or "PI SYSTEM FEATURES failed")
        payload = resp.get("payload", {})
        if not isinstance(payload, dict):
            raise RuntimeError("PI SYSTEM FEATURES returned a non-object payload")

        _FEATURE_STATUS_CACHE = payload
        _FEATURE_STATUS_CACHE_TS = now
        _FEATURE_STATUS_CACHE_ERROR = None
        return payload, None

    except Exception as e:
        _FEATURE_STATUS_CACHE_TS = now
        _FEATURE_STATUS_CACHE_ERROR = str(e)
        if _FEATURE_STATUS_CACHE is not None:
            return _FEATURE_STATUS_CACHE, _FEATURE_STATUS_CACHE_ERROR
        return {}, _FEATURE_STATUS_CACHE_ERROR


def _get_pi_clocks_report() -> dict:
    return send_command(machine="PI", subsystem="CLOCKS", command="REPORT")["payload"]


def _dac_tick_listener_loop() -> None:
    """Background tap listener for the compact CLOCKS_DAC_TICK feed.

    This is intentionally best-effort and cache-only.  The readout path never
    blocks on the socket and never polls TEENSY CLOCKS REPORT_DAC.  If the tap is
    temporarily unavailable, callers simply fall back to the most recent CLOCKS
    report/TIMEBASE fields until a fresh tick arrives.
    """
    global _DAC_TICK_CACHE, _DAC_TICK_CACHE_TS, _DAC_TICK_CACHE_ERROR

    subscribe = json.dumps(
        {"type": "set_topics", "topics": [DAC_TICK_TOPIC]},
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"

    while True:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.settimeout(2.0)
                sock.connect(PUBSUB_TAP_SOCKET)
                sock.settimeout(None)

                with sock.makefile("rwb") as stream:
                    stream.write(subscribe)
                    stream.flush()

                    with _DAC_TICK_LOCK:
                        _DAC_TICK_CACHE_ERROR = None

                    for raw in stream:
                        try:
                            msg = json.loads(raw.decode("utf-8"))
                        except Exception:
                            continue

                        if not isinstance(msg, dict):
                            continue
                        if msg.get("type") != "publish":
                            continue
                        if msg.get("topic") != DAC_TICK_TOPIC:
                            continue

                        payload = msg.get("payload")
                        if not isinstance(payload, dict):
                            continue
                        if payload.get("schema") != "CLOCKS_DAC_TICK_V2":
                            continue

                        with _DAC_TICK_LOCK:
                            _DAC_TICK_CACHE = payload
                            _DAC_TICK_CACHE_TS = time.monotonic()
                            _DAC_TICK_CACHE_ERROR = None

        except Exception as e:
            with _DAC_TICK_LOCK:
                _DAC_TICK_CACHE_ERROR = str(e)
            time.sleep(DAC_TICK_RECONNECT_S)


def _ensure_dac_tick_listener() -> None:
    global _DAC_TICK_THREAD

    if _DAC_TICK_THREAD is not None and _DAC_TICK_THREAD.is_alive():
        return

    with _DAC_TICK_LOCK:
        if _DAC_TICK_THREAD is not None and _DAC_TICK_THREAD.is_alive():
            return
        _DAC_TICK_THREAD = threading.Thread(
            target=_dac_tick_listener_loop,
            name="zpnet-dac-tick-listener",
            daemon=True,
        )
        _DAC_TICK_THREAD.start()


def _get_dac_tick_payload(max_age_s: float = DAC_TICK_STALE_S) -> dict | None:
    _ensure_dac_tick_listener()

    with _DAC_TICK_LOCK:
        payload = _DAC_TICK_CACHE
        age = time.monotonic() - _DAC_TICK_CACHE_TS if _DAC_TICK_CACHE_TS else 1.0e9

    if not isinstance(payload, dict):
        return None
    if age > max_age_s:
        return None
    return payload


def _get_pi_clocks_report_dac() -> dict:
    # Compatibility name: the metrics panel now consumes the compact pub/sub
    # CLOCKS_DAC_TICK feed instead of polling TEENSY CLOCKS REPORT_DAC.
    return _get_dac_tick_payload() or {}


def _get_pi_gnss_report() -> dict:
    return send_command(machine="PI", subsystem="GNSS", command="REPORT")["payload"]


def _get_clocks_baseline() -> dict | None:
    try:
        resp = send_command(machine="PI", subsystem="CLOCKS", command="BASELINE_INFO")
        if resp.get("success"):
            p = resp.get("payload", {})
            if p.get("baseline_set"):
                return p
    except Exception:
        pass
    return None



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


def _gnss_from_direct_report() -> dict:
    try:
        p = _get_pi_gnss_report()
    except Exception:
        return {}

    if not isinstance(p, dict):
        return {}

    discipline = p.get("discipline", {}) if isinstance(p.get("discipline"), dict) else {}
    clock = p.get("clock", {}) if isinstance(p.get("clock"), dict) else {}
    integrity = p.get("integrity", {}) if isinstance(p.get("integrity"), dict) else {}
    survey = p.get("survey_mode", {}) if isinstance(p.get("survey_mode"), dict) else {}
    pps = p.get("pps", {}) if isinstance(p.get("pps"), dict) else {}

    return {
        "lock_quality": p.get("lock_quality"),
        "pos_mode": integrity.get("pos_mode") or survey.get("receiver_mode"),
        "freq_mode_name": discipline.get("freq_mode_name"),
        "satellites": p.get("satellites"),
        "hdop": p.get("hdop"),
        "traim": integrity.get("traim"),
        "latitude_deg": p.get("latitude_deg"),
        "longitude_deg": p.get("longitude_deg"),
        "altitude_m": p.get("altitude_m"),
        "ellipsoid_height_m": p.get("ellipsoid_height_m"),
        "geoid_sep_m": p.get("geoid_sep_m"),
        "pps_valid": p.get("pps_valid"),
        "pps_active": pps.get("active"),
        "estimated_accuracy_ns": pps.get("estimated_accuracy_ns"),
        "pps_timing_error_ns": discipline.get("pps_timing_error_ns"),
        "time_status": clock.get("time_status_name"),
        "pps_sync": clock.get("pps_sync"),
        "clock_drift_ppb": clock.get("drift_ppb"),
        "temperature_c": clock.get("temperature_c"),
    }


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
    # Priority order:
    #   1. Direct PI/GNSS report, if available.
    #   2. Current TIMEBASE record carried by the CLOCKS report.
    #   3. Consolidated SYSTEM report as a compatibility fallback.
    g: dict = {}
    _merge_missing(g, _gnss_from_direct_report())
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
    if isinstance(r, dict) and isinstance(r.get("payload"), dict):
        return r["payload"]
    return r if isinstance(r, dict) else {}


def _fragment_root(r: dict) -> dict:
    root = _payload_root(r)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else root


def _frag(r: dict, key: str, default=None):
    """Read a TIMEBASE field from hierarchical V2 or legacy flat layouts.

    key may be either a legacy flat key ("ocxo1_ns") or a dotted V2 path
    ("ocxo1.pps_residual.fast_residual_ns").  The fragment object is searched
    first, then the surrounding TIMEBASE/CLOCKS report object.
    """
    root = _payload_root(r)
    frag = _fragment_root(root)
    for source in (frag, root):
        val = _path_get(source, key, None)
        if val is not None:
            return val
    return default


def _extra(r: dict, key: str, default=None):
    root = _payload_root(r)
    ec = root.get("extra_clocks")
    if isinstance(ec, dict):
        val = _path_get(ec, key, None)
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


def _welford_value(r: dict, prefix: str, field: str):
    if prefix.endswith("_dac"):
        lane = prefix[:-4]
        keys = (
            f"stats.dac.{lane}.{field}",
            f"stats.{prefix}.welford.{field}",
            f"{prefix}_welford_{field}",
        )
    else:
        keys = (
            f"stats.{prefix}.welford.{field}",
            f"stats.{prefix}.pps_residual.welford.{field}",
            f"stats.{prefix}_pps_residual.welford.{field}",
            f"stats.pps_{prefix}.welford.{field}",
            f"stats.pps_{prefix}.{field}",
            f"stats.{prefix}.{field}",
            f"{prefix}_welford_{field}",
        )
        if prefix == "vclock":
            keys += (
                f"stats.pps_vclock.welford.{field}",
                f"stats.pps_vclock.{field}",
                f"stats.pps_v.welford.{field}",
                f"stats.pps_v.{field}",
                f"stats.ppsv.welford.{field}",
                f"stats.ppsv.{field}",
                f"vclock.pps_residual.welford.{field}",
                f"pps_vclock_welford_{field}",
                f"pps_v_welford_{field}",
                f"vclock_residual_welford_{field}",
            )
    for key in keys:
        val = _frag(r, key, None)
        if val is not None:
            return val
    return None


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
    """Return VCLOCK nanoseconds added during the prior PPS second."""
    direct = _to_int(_field(
        r,
        "vclock.pps_residual.clock_interval_ns",
        "vclock.measurement.clock_interval_ns",
        "vclock.measurement.gnss_ns_between_edges",
        "vclock.interval.clock_interval_ns",
        "vclock.interval.gnss_ns_between_edges",
        "vclock_gnss_ns_between_edges",
    ))
    if direct is not None:
        return direct
    return _ns_interval_from_residual(_field(
        r,
        "vclock.pps_residual.fast_residual_ns",
        "vclock.measurement.second_residual_ns",
        "vclock_second_residual_ns",
    ))


def _vclock_residual_ns(r: dict):
    return _to_int(_field(
        r,
        "vclock.pps_residual.fast_residual_ns",
        "vclock.measurement.second_residual_ns",
        "vclock_second_residual_ns",
    ))


def _ocxo_interval_ns(r: dict, key: str):
    return _to_int(_field(
        r,
        f"{key}.pps_residual.clock_interval_ns",
        f"{key}.measurement.clock_interval_ns",
        f"{key}.measurement.gnss_ns_between_edges",
        f"{key}.interval.clock_interval_ns",
        f"{key}.interval.bridge_gnss_ns_between_edges",
        f"{key}_gnss_ns_between_edges",
    ))


def _ocxo_residual_ns(r: dict, key: str):
    return _to_int(_field(
        r,
        f"{key}.pps_residual.fast_residual_ns",
        f"{key}.measurement.second_residual_ns",
        f"{key}_second_residual_ns",
    ))


def _prediction_value(r: dict, lane: str, field: str, default=None):
    """Read the static-prediction surface for a clock lane.

    Supports both hierarchical V3:
      prediction.<lane>.prediction_cycles / actual_cycles / residual_cycles

    and the older flat/legacy spellings:
      <lane>_prediction_cycles / <lane>_actual_cycles / <lane>_residual_cycles
      prediction.<lane>_static_prediction_cycles / ...
    """
    aliases = {
        "prediction_cycles": ("prediction_cycles", "static_prediction_cycles"),
        "actual_cycles":     ("actual_cycles",),
        "residual_cycles":   ("residual_cycles", "static_residual_cycles"),
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
    """Return the best available OCXO servo state for the header.

    New TIMEBASE_FRAGMENT / TIMEBASE_FORENSICS rows carry explicit servo_mode
    and servo_active fields at the pair-identity level; active rows also carry
    dac.calibrate_ocxo / dac.servo_mode beside the persisted DAC values.
    Prefer those explicit firmware-authored fields.  ACTIVE remains only as a
    legacy fallback for older records whose DAC persistence object proves the
    servo was running but does not name the selected mode.
    """
    keys = (
        "dac.calibrate_ocxo",
        "dac.servo_mode",
        "dac.mode",
        "servo.mode",
        "servo_mode",
        "calibrate_ocxo",
        "forensics.dac.calibrate_ocxo",
        "forensics.dac.servo_mode",
        "forensics.dac.mode",
        "forensics.servo.mode",
        "forensics.servo_mode",
        "forensics.calibrate_ocxo",
        "environmental.calibrate_ocxo",
        "forensics.environmental.calibrate_ocxo",
    )
    saw_explicit_off = False
    for key in keys:
        val = _field(r, key, default=None)
        if val is None:
            continue
        s = str(val).strip().upper()
        if s and s not in ("OFF", "NONE", "IDLE"):
            return s
        if s in ("OFF", "NONE", "IDLE"):
            saw_explicit_off = True

    # Last-ditch inference from any published servo-input source.  This covers
    # detailed CLOCKS.REPORT_DAC-style payloads and older TIMEBASE builds that
    # carried the selected input surface.
    for lane in ("ocxo1", "ocxo2"):
        for key in (
            f"dac.{lane}.servo_input.selected_source",
            f"dac.{lane}.servo_input.selected_source_id",
            f"{lane}.servo_input.selected_source",
            f"{lane}.servo_input.selected_source_id",
            f"forensics.dac.{lane}.servo_input.selected_source",
            f"forensics.dac.{lane}.servo_input.selected_source_id",
            f"forensics.{lane}.servo_input.selected_source",
            f"forensics.{lane}.servo_input.selected_source_id",
        ):
            val = _field(r, key, default=None)
            if val is None:
                continue

            # Refactored firmware mode ids:
            #   1 = MEAN, 2 = TOTAL.  Older builds used id 1 for TOTAL and
            #   id 2 for NOW, so prefer string names when available.
            if isinstance(val, (int, float)):
                if int(val) == 1:
                    return "MEAN"
                if int(val) == 2:
                    return "TOTAL"

            s = str(val).strip().upper()
            if "TOTAL" in s or "TAU" in s:
                return "TOTAL"
            if "MEAN" in s or "WELFORD" in s:
                return "MEAN"

    # Explicit activity without explicit mode should be rare after the Beta
    # fix, but preserve a clear fallback so the panel never lies and calls an
    # active servo IDLE.
    for key in ("servo_active", "dac.servo_active", "forensics.servo_active", "forensics.dac.servo_active"):
        val = _field(r, key, default=None)
        if val is True or str(val).strip().lower() == "true":
            return "ACTIVE"

    # Legacy TIMEBASE_FRAGMENT_V3 only included fragment.dac while
    # clocks_servo_active() was true.  That minimal persistence feed proved
    # activity but not mode.
    dac_o1 = _dac_value(r, "ocxo1")
    dac_o2 = _dac_value(r, "ocxo2")
    if dac_o1 is not None or dac_o2 is not None:
        return "ACTIVE"

    return "IDLE" if saw_explicit_off else "IDLE"


# ---------------------------------------------------------------------
# DAC presentation helpers
# ---------------------------------------------------------------------

DAC_CODE_SCALE = 65536.0


def _dac_value(r: dict, lane: str):
    """Return the instantaneous firmware-published AD5693R DAC code.

    TIMEBASE_FRAGMENT_V3 currently publishes the live servo output as
    fragment.dac.ocxo1_dac / fragment.dac.ocxo2_dac.  Older/detail reports may
    carry dac.<lane>.value, dac.<lane>.dac, or a flat <lane>_dac key.  This
    helper intentionally reads only already-published firmware fields; the Pi
    does no servo reconstruction here.
    """
    return _to_float(_field(
        r,
        f"dac.{lane}_dac",
        f"dac.{lane}.value",
        f"dac.{lane}.dac",
        f"dac.{lane}.code",
        f"dac.{lane}.dac_code",
        f"{lane}_dac",
        f"{lane}_dac_code",
    ))


def _dac_voltage(dac_code):
    code = _to_float(dac_code)
    if code is None:
        return None
    # Fallback only.  Prefer firmware-authored voltage from CLOCKS_DAC_TICK
    # when available because the active DAC reference/gain doctrine lives in
    # firmware, not in this panel.
    return code * VREF / DAC_CODE_SCALE


def _dac_label(dac_code, dac_voltage=None):
    code = _to_float(dac_code)
    volts = _to_float(dac_voltage)
    if volts is None:
        volts = _dac_voltage(code)
    if code is None or volts is None:
        return "---"
    return f"{code:>.3f} {volts:.6f}V"


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
    return _to_float(
        obj.get("v")
        if obj.get("v") is not None
        else obj.get("dac_voltage")
    )


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

    Prefer the compact CLOCKS_DAC_TICK pub/sub cache and fall back to the
    current CLOCKS report/TIMEBASE value.  This avoids polling the verbose
    Teensy REPORT_DAC command path while the operator is nudging DAC values.
    """
    report = {}
    try:
        p = _get_pi_clocks_report()
        report = p.get("report") if isinstance(p.get("report"), dict) else p
    except Exception:
        report = {}

    report_dac = None
    try:
        report_dac = _get_pi_clocks_report_dac()
    except Exception:
        report_dac = None

    current = _dac_current_value(report, report_dac, lane)
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


def _dac_report_dither_summary(report_dac: dict | None, lane: str) -> str:
    tick = _dac_tick_payload(report_dac)
    obj = _dac_report_lane(report_dac, lane)

    if tick:
        if _to_bool(tick.get("dither")) is not True:
            return "OFF"
        low_code = _to_int(obj.get("lo"))
        high_code = _to_int(obj.get("hi"))
        high_ms = _to_int(obj.get("hi_ms"))
        if low_code is None or high_code is None or high_ms is None:
            return "ON"
        low_ms = max(0, 1000 - high_ms)
        return f"{low_code}:{low_ms:03d} {high_code}:{high_ms:03d}"

    obj = _dac_report_lane(report_dac, lane)
    d = obj.get("dither") if isinstance(obj.get("dither"), dict) else {}
    if not d:
        return ""

    enabled = _to_bool(d.get("enabled"))
    rate = _to_int(d.get("rate_hz"))
    low = _to_int(d.get("last_window_low_count"))
    high = _to_int(d.get("last_window_high_count"))
    low_code = _to_int(d.get("last_window_low_code"))
    high_code = _to_int(d.get("last_window_high_code"))

    if enabled is not True:
        return "OFF"
    if rate is None:
        return "ON"
    if low is None or high is None:
        return f"{rate}Hz"

    low_s = f"{low:03d}"
    high_s = f"{high:03d}"

    if low_code is None or high_code is None:
        return f"{rate}Hz {low_s}/{high_s}"
    return f"{rate}Hz {low_code}:{low_s} {high_code}:{high_s}"


def _dac_detail_lines(r: dict, baseline: dict | None, report_dac: dict | None,
                      W_NAME: int, W_VALUE: int, W_TAU: int, W_PPB: int,
                      W_RAW: int, W_RES: int, W_MEAN: int, W_SD: int,
                      W_SE: int, W_N: int, W_BASE: int, W_NOW: int,
                      W_DELTA: int) -> list[str]:
    lines: list[str] = []
    dither_width = W_TAU + W_PPB + W_RAW + W_RES

    lines.append(
        f"{'DAC':<{W_NAME}}"
        f"{'VALUE/VOUT':>{W_VALUE}}"
        f"{'':>{dither_width}}"
        f"{'MEAN':>{W_MEAN}}"
        f"{'SD':>{W_SD}}"
        f"{'SE':>{W_SE}}"
        f"{'N':>{W_N}}"
        f"  "
        f"{'BASE':>{W_BASE}}"
        f"{'NOW':>{W_NOW}}"
        f"{'DELTA':>{W_DELTA}}"
    )
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        dac_now = _dac_current_value(r, report_dac, key)
        dac_label = _dac_label(dac_now, _dac_current_voltage(r, report_dac, key))
        dither_summary = _dac_report_dither_summary(report_dac, key)

        base_dac = None
        if baseline:
            base_dac = baseline.get("baseline_dac_mean", {}).get(key)

        lines.append(
            f"{name:<{W_NAME}}"
            f"{dac_label:>{W_VALUE}}"
            f"{dither_summary:>{dither_width}}"
            f"{_welford_cols_fragment(r, f'{key}_dac', W_MEAN, W_SD, W_SE, W_N)}"
            f"  {_baseline_comp(base_dac, dac_now)}"
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
        lines.append(f"LAST POLL ERROR: {error[:120]}")

    lines.append("")
    return lines


# ---------------------------------------------------------------------
# Status header
# ---------------------------------------------------------------------

def status_header() -> str:
    try:
        s = _get_system_snapshot()

        net = s.get("network", {}).get("network_status", "?")
        pi_health = s.get("pi", {}).get("health_state", "?")
        teensy_health = s.get("teensy", {}).get("health_state", "?")
        try:
            clocks_report = _get_pi_clocks_report()
        except Exception:
            clocks_report = {}
        gnss_status = _gnss_status(clocks_report, s)
        gnss_lock = gnss_status.get("lock_quality", "?")

        bat_v = "?"
        power = s.get("power", {})
        for bus_key, devices in power.items():
            if not bus_key.startswith("i2c-"):
                continue
            if isinstance(devices, dict):
                for rail in devices.values():
                    if isinstance(rail, dict) and rail.get("label", "").lower() == "battery":
                        bat_v = f"{rail['volts']:.2f}V"

        battery = s.get("battery", {})
        pct = battery.get("remaining_pct")
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

def _baseline_comp(base_val, now_val):
    if base_val is not None and now_val is not None:
        delta = float(now_val) - float(base_val)
        return (
            f"{_fmt(base_val, '>10.3f', 10)}"
            f"{_fmt(now_val, '>10.3f', 10)}"
            f"{_fmt(delta, '>+10.3f', 10)}"
        )
    return f"{'---':>10}{'---':>10}{'---':>10}"


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


def _welford_cols_fragment_or_zero(r, prefix, w_mean, w_sd, w_se, w_n, n=None, mean_decimals=3):
    """Render firmware Welford stats, falling back to a visible zero row.

    Reference-style rows such as VCLOCK may not have a published Welford
    object in older TIMEBASE builds, or may publish n=0 while the visible
    residual surface is definitionally zero.  In either case, render a zero
    accumulator with the current campaign count so the top clock block remains
    visually consistent.
    """
    mean = _to_float(_welford_value(r, prefix, "mean"))
    sd   = _to_float(_welford_value(r, prefix, "stddev"))
    se   = _to_float(_welford_value(r, prefix, "stderr"))
    wn   = _to_int  (_welford_value(r, prefix, "n"))

    if (mean is None and sd is None and se is None and wn is None) or not wn:
        return _welford_cols_zero(w_mean, w_sd, w_se, w_n, n=n, mean_decimals=mean_decimals)

    return (
        f"{_fmt(mean, f'>{w_mean}.{mean_decimals}f', w_mean)}"
        f"{_fmt(sd,   f'>{w_sd}.3f',   w_sd)}"
        f"{_fmt(se,   f'>{w_se}.3f',   w_se)}"
        f"{_fmt(wn,   f'>{w_n}d',      w_n)}"
    )


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
    state = report.get("campaign_state") or p.get("campaign_state")
    if state is None:
        # TIMEBASE-bearing CLOCKS reports imply a running campaign even if the
        # wrapper did not include campaign_state.
        state = "STARTED" if _fragment_root(report) else "IDLE"
    if state != "STARTED":
        lines.append(f"CLOCKS: {state}")
        lines.append("")

        baseline = _get_clocks_baseline()
        W_NAME  = 6
        W_VALUE = 20
        W_TAU   = 18
        W_PPB   = 10
        W_RAW   = 14
        W_RES   = 8
        W_MEAN  = 10
        W_SD    = 8
        W_SE    = 8
        W_N     = 6
        W_BASE  = 10
        W_NOW   = 10
        W_DELTA = 10

        lines.extend(_dac_detail_lines(
            report, baseline, report_dac,
            W_NAME, W_VALUE, W_TAU, W_PPB, W_RAW, W_RES,
            W_MEAN, W_SD, W_SE, W_N, W_BASE, W_NOW, W_DELTA,
        ))
        lines.append("")
        lines.extend(feature_status_grid_lines())
        return lines

    r = report
    campaign = r.get("campaign", "?")
    elapsed = r.get("campaign_elapsed", "00:00:00")
    n = _to_int(_field(r, "teensy_pps_vclock_count", "teensy_pps_count", "pps_count")) or 0

    servo_state = str(report_dac.get("servo") or _servo_state(r)).upper() if isinstance(report_dac, dict) else _servo_state(r)

    baseline = _get_clocks_baseline()
    baseline_ppb = baseline.get("baseline_ppb", {}) if baseline else {}
    baseline_id = baseline.get("baseline_id", "?") if baseline else None
    baseline_campaign = baseline.get("baseline_campaign", "?") if baseline else None

    servo_str = servo_state
    baseline_str = f"BASELINE: {baseline_campaign}" if baseline_id else "BASELINE: NONE"

    lines.append(
        f"CAMPAIGN: {campaign}  ELAPSED: {elapsed}  n={n}"
        f"    SERVO: {servo_str}"
        f"    {baseline_str}"
    )
    lines.append("")

    # ── Column widths ──
    W_NAME  = 6
    W_VALUE = 20
    W_TAU   = 18
    W_PPB   = 10
    W_RAW   = 14
    W_RES   = 8
    W_MEAN  = 10
    W_SD    = 8
    W_SE    = 8
    W_N     = 6
    W_BASE  = 10
    W_NOW   = 10
    W_DELTA = 10

    # ── CLK header ──
    lines.append(
        f"{'CLK':<{W_NAME}}"
        f"{'VALUE':>{W_VALUE}}"
        f"{'TAU':>{W_TAU}}"
        f"{'PPB':>{W_PPB}}"
        f"{'RAW':>{W_RAW}}"
        f"{'RES':>{W_RES}}"
        f"{'MEAN':>{W_MEAN}}"
        f"{'SD':>{W_SD}}"
        f"{'SE':>{W_SE}}"
        f"{'N':>{W_N}}"
        f"  "
        f"{'BASE':>{W_BASE}}"
        f"{'NOW':>{W_NOW}}"
        f"{'DELTA':>{W_DELTA}}"
    )

    # ── GNSS (reference nanosecond clock) ──
    gnss_ns = _to_int(_field(r, "gnss.ns", "gnss_ns"))
    gnss_tau = 1.0
    gnss_ppb = 0.0
    gnss_raw = _to_int(_field(r, "gnss.clock_interval_ns", "gnss.interval_ns", "gnss_interval_ns")) or 1_000_000_000
    gnss_res = _to_int(_field(r, "gnss.second_residual_ns", "gnss_residual_ns")) or 0
    lines.append(
        f"{'GNSS':<{W_NAME}}"
        f"{_comma_int(gnss_ns, W_VALUE)}"
        f"{_fmt(gnss_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(gnss_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{_comma_int(gnss_raw, W_RAW)}"
        f"{_sign_int(gnss_res, W_RES)}"
        f"{_welford_cols_zero(W_MEAN, W_SD, W_SE, W_N, n)}"
        f"  {_baseline_comp(baseline_ppb.get('gnss'), gnss_ppb)}"
    )

    # ── VCLOCK (GNSS-disciplined nanosecond clock) ──
    vclock_ns  = _to_int(_field(r, "vclock.ns", "gnss.vclock_ns", "gnss.pps_vclock_ns", "vclock_gnss_ns_at_pps_vclock", "gnss_ns"))
    vclock_tau = _to_float(_freq_value(r, "vclock", "tau"))
    vclock_ppb = _to_float(_freq_value(r, "vclock", "ppb"))

    # RAW for VCLOCK is intentionally nanoseconds added to the VCLOCK ledger
    # during the prior PPS second.  Cycle-domain truth still exists elsewhere
    # (raw_cycles/DWT detail), but this panel groups VCLOCK with the standard
    # nanosecond clocks.
    vclock_res = _vclock_residual_ns(r)
    vclock_raw = _vclock_interval_ns(r)

    # VCLOCK is the GNSS-disciplined nanosecond ledger.  If the current
    # TIMEBASE build does not expose a named VCLOCK interval object, the
    # visible nanosecond interval is still 1e9 plus its visible residual.
    # This keeps VCLOCK aligned with GNSS/OCXO rows while DWT remains the
    # cycle-oriented exception.
    if vclock_res is None:
        vclock_res = 0
    if vclock_raw is None:
        vclock_raw = 1_000_000_000 + vclock_res

    # If firmware has not yet published frequency stats, derive the displayed
    # interval ratio from the nanosecond interval surface, not from cycle RAW.
    if vclock_tau is None or vclock_ppb is None:
        vclock_tau = vclock_tau if vclock_tau is not None else (float(vclock_raw) / 1_000_000_000.0)
        vclock_ppb = vclock_ppb if vclock_ppb is not None else ((float(vclock_raw) / 1_000_000_000.0) - 1.0) * 1_000_000_000.0

    lines.append(
        f"{'VCLOCK':<{W_NAME}}"
        f"{_comma_int(vclock_ns, W_VALUE)}"
        f"{_fmt(vclock_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(vclock_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{_comma_int(vclock_raw, W_RAW)}"
        f"{_sign_int(vclock_res, W_RES)}"
        f"{_welford_cols_fragment_or_zero(r, 'vclock', W_MEAN, W_SD, W_SE, W_N, n=n)}"
        f"  {_baseline_comp(baseline_ppb.get('vclock'), vclock_ppb)}"
    )

    # ── OCXO1, OCXO2 (nanosecond clocks) ──
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        ocxo_ns = _to_int(_field(r, f"{key}.ns", f"gnss.{key}_ns", f"{key}_ns", f"{key}_ns_at_pps_vclock", f"{key}_ns_count_at_pps"))
        tau     = _to_float(_freq_value(r, key, "tau"))
        ppb     = _to_float(_freq_value(r, key, "ppb"))
        raw     = _none_if_zero(_ocxo_interval_ns(r, key))
        res     = _ocxo_residual_ns(r, key)
        lines.append(
            f"{name:<{W_NAME}}"
            f"{_comma_int(ocxo_ns, W_VALUE)}"
            f"{_fmt(tau, f'>{W_TAU}.12f', W_TAU)}"
            f"{_fmt(ppb, f'>{W_PPB}.3f', W_PPB)}"
            f"{_comma_int(raw, W_RAW)}"
            f"{_sign_int(res, W_RES)}"
            f"{_welford_cols_fragment(r, key, W_MEAN, W_SD, W_SE, W_N)}"
            f"  {_baseline_comp(baseline_ppb.get(key), ppb)}"
        )

    # Separate standard nanosecond clocks from synthetic/cycle-oriented clocks.
    lines.append("")

    # ── GN_RAW (GNSS receiver self-reported drift; Pi-side synthetic) ──
    gnss_raw_ns  = _to_int(_extra(r, "gnss_raw_ns"))
    gnss_raw_tau = _to_float(_extra(r, "gnss_raw_tau"))
    gnss_raw_ppb = _to_float(_extra(r, "gnss_raw_ppb"))
    gnss_raw_res = _to_float(_extra(r, "gnss_raw_drift_ppb"))
    gnss_raw_interval_ns = int(1_000_000_000.0 + gnss_raw_ppb) if gnss_raw_ppb is not None else None
    lines.append(
        f"{'GN_RAW':<{W_NAME}}"
        f"{_comma_int(gnss_raw_ns, W_VALUE)}"
        f"{_fmt(gnss_raw_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(gnss_raw_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{_comma_int(gnss_raw_interval_ns, W_RAW)}"
        f"{_fmt(gnss_raw_res, f'>{W_RES}.1f', W_RES)}"
        f"{_welford_cols_extra(r, 'gnss_raw', W_MEAN, W_SD, W_SE, W_N)}"
        f"  {_baseline_comp(baseline_ppb.get('gnss_raw'), gnss_raw_ppb)}"
    )

    # ── DWT (cycle-oriented clock) ──
    dwt_present = _has_any(
        r,
        "dwt.ns",
        "dwt.cycle_count_total",
        "stats.dwt.ppb",
        "dwt.cycles_between_pps_vclock",
        "dwt_ns",
        "dwt_cycle_count_total",
        "dwt_welford_mean",
    )
    if dwt_present:
        dwt_total = _to_int(_field(
            r,
            "dwt.value",
            "dwt.cycles",
            "dwt.cycle_count_total",
            "dwt_cycle_count_total",
            "dwt.ns",
            "dwt_ns",
        ))
        dwt_tau   = _to_float(_freq_value(r, "dwt", "tau"))
        dwt_ppb   = _to_float(_freq_value(r, "dwt", "ppb"))
        dwt_raw   = _to_int(_field(r, "dwt.cycles_between_pps_vclock", "dwt_cycles_between_pps_vclock", "dwt_cycle_count_between_pps"))
        dwt_res   = _to_int(_field(r, "dwt.second_residual_cycles", "dwt_second_residual_cycles"))
        lines.append(
            f"{'DWT':<{W_NAME}}"
            f"{_comma_int(dwt_total, W_VALUE)}"
            f"{_fmt(dwt_tau, f'>{W_TAU}.12f', W_TAU)}"
            f"{_fmt(dwt_ppb, f'>{W_PPB}.3f', W_PPB)}"
            f"{_comma_int(dwt_raw, W_RAW)}"
            f"{_sign_int(dwt_res, W_RES)}"
            f"{_welford_cols_fragment(r, 'dwt', W_MEAN, W_SD, W_SE, W_N)}"
            f"  {_baseline_comp(baseline_ppb.get('dwt'), dwt_ppb)}"
        )

    # ── DAC detail ──
    lines.append("")
    lines.extend(_dac_detail_lines(
        r, baseline, report_dac,
        W_NAME, W_VALUE, W_TAU, W_PPB, W_RAW, W_RES,
        W_MEAN, W_SD, W_SE, W_N, W_BASE, W_NOW, W_DELTA,
    ))

    lines.append("")

    # ── Time ──
    gnss_time = r.get("gnss_time_utc", "---")
    system_time = r.get("system_time_utc", "---")
    lines.append(f"TIME  GNSS: {gnss_time}    SYSTEM: {system_time}")
    lines.append("")

    # ── DWT detail ──
    dwt_actual = _to_int(_field(
        r,
        "dwt.cycles_between_pps_vclock",
        "dwt_cycles_between_pps_vclock",
        "dwt_cycle_count_between_pps",
    ))
    dwt_residual = _to_int(_field(
        r,
        "dwt.second_residual_cycles",
        "dwt_second_residual_cycles",
    ))

    # EXPECTED is the firmware's next-second DWT-cycle prediction, not the
    # nominal 1.008 GHz constant.  In the current paired TIMEBASE schema, the
    # DWT row is driven by the PPS/GPIO witness interval, so its prediction
    # lives on prediction.pps.* rather than prediction.dwt.*.  Older aliases
    # remain as compatibility fallbacks.
    dwt_expected = _to_int(_field(
        r,
        "prediction.pps.prediction_cycles",
        "prediction.pps.static_prediction_cycles",
        "pps_prediction_cycles",
        "pps_static_prediction_cycles",
        "prediction.dwt.prediction_cycles",
        "prediction.dwt.static_prediction_cycles",
        "dwt.prediction_cycles",
        "dwt.static_prediction_cycles",
    ))
    # Only fall back to explicit expected fields if they are non-nominal; older
    # firmware used these names for the fixed 1.008 GHz constant, which is no
    # longer the desired display here.
    if dwt_expected is None:
        explicit_expected = _to_int(_field(
            r,
            "dwt.expected_per_pps_vclock",
            "dwt_expected_per_pps_vclock",
            "dwt_expected_per_pps",
            "dwt.expected_cycles_per_second",
            "dwt_expected_cycles_per_second",
            "dwt.expected_cycles",
            "dwt_expected_cycles",
        ))
        if explicit_expected is not None and explicit_expected != 1008000000:
            dwt_expected = explicit_expected

    dwt_at_anchor = _field(
        r,
        "dwt.at_pps_vclock",
        "dwt_at_pps_vclock",
        "dwt_cycle_count_at_pps",
    )
    counter32 = _field(r, "counter32_at_pps_vclock", "qtimer_at_pps")
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
    ("CLOCKS", lambda: _safe_lines(clocks_combined_readout)),
]
