"""
ZPNet Metrics Readout Blocks — Generalized Campaign Detail Edition

Data source:
  CLOCKS_V4 owns the clock operator view and PHOTONS_V1 owns the optical
  operator view.  Both are canonical always-on instruments with optional
  campaign decoration. Baselines are campaign_master relationships resolved on
  demand; they are not instrument state. Metrics never waits for or reads
  TIMEBASE.

Stats policy:
  CLOCKS statistics are read verbatim from the producer-authored instrument and
  campaign surfaces, including the GNSS reference Welford and Pi-owned DAC
  Welfords.  PHOTONS publishes cumulative accepted-lap Welford/grand-ratio
  sufficient state.  The live PHOTONS rolling tail retains actual PUBSUB
  publications in memory and subtracts adjacent snapshots to recover one-second
  populations for display; PostgreSQL is not in that live-tail path.  Durable
  campaign/baseline history remains a database read model.  This never synthesizes
  individual samples, changes admission, or feeds science back into PHOTONS.
  Missing PHOTONS PPB buckets remain visibly missing; Metrics never estimates
  rolling windows from repaint history.  Other arithmetic is
  deterministic presentation/control math such as DAC conversion/nudging and
  baseline deltas.

Clock row doctrine:
  The dense operator table shows GNSS, VCLOCK, OCXO1, and OCXO2 only.
  GN_RAW and DWT remain available in CLOCKS/focused diagnostics but are omitted
  here because they are not primary TEMPEST science clocks.
  VALUE is the Beta-authored campaign-relative clockface while a campaign is
  active; otherwise it is the canonical always-on instrument clockface.
  10-MIN/60-MIN/8-HOUR/24-HOUR are firmware/Pi-producer-authored rolling PPB
  buckets.  Metrics never estimates a rolling population from repaint history.
  TOTAL is the always-on population since boot or the last statistics reset.
  TOTAL owns the displayed Welford population and the legacy stats.<lane>.ppb
  fallback while producers migrate to ppb_buckets.
  CAMP is the firmware-authored campaign population. BASE/NOW/DELTA compare
  the referenced campaign CAMP result to the active campaign CAMP result.
  RES is the firmware-published residual for the prior PPS interval.

Column layout (CLK rows):
  NAME   VALUE   10-MIN   60-MIN   8-HOUR   24-HOUR   TOTAL   CAMP   RES
         MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (DAC rows):
  NAME   DAC_VALUE_VOLTAGE   (blanks)   MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (INT rows):
  NAME   END_GNSS_NS   DELTA_NS
"""

import json
import math
import socket
import threading
import time
from collections import deque

from zpnet.processes.processes import PUBSUB_TAP_SOCKET, create_pubsub_cache, send_command
from zpnet.shared.db import open_db

# AD5693R doctrine mirrors Pi CLOCKS, the sole actuator authority:
# internal 2.5 V reference, 2x gain, nominal 0..5 V output span, with the
# OCXO EFC path hard-limited to the 3.3 V equivalent code.
DAC_INTERNAL_REF_VOLTAGE = 2.5
DAC_OUTPUT_GAIN = 2.0
DAC_OUTPUT_FULL_SCALE_VOLTAGE = 5.0
DAC_SAFE_MAX_OUTPUT_VOLTAGE = 3.3
DAC_CODE_SCALE = 65536.0

DAC_FINE_STEP = 1.0
DAC_COARSE_STEP = 10.0
DAC_MIN_CODE = 0.0
DAC_MAX_CODE = float(
    int((DAC_SAFE_MAX_OUTPUT_VOLTAGE / DAC_OUTPUT_FULL_SCALE_VOLTAGE) * DAC_CODE_SCALE)
)

FEATURE_GRID_COLUMNS = 4
FEATURE_GRID_CELL_WIDTH = 39

PPB_BUCKET_KEYS = ("10_min", "60_min", "8_hour", "24_hour", "total")
PPB_VISIBLE_COLUMN_COUNT = len(PPB_BUCKET_KEYS) + 1  # + campaign

CLOCKS_TOPIC = "CLOCKS"
PHOTONS_TOPIC = "PHOTONS"
CAMPAIGN_TYPE_LANTERN = "LANTERN"
PHOTONS_ROLLING_ROWS = 25

_LIVE_CACHE = create_pubsub_cache(CLOCKS_TOPIC)


class _RollingPubSubTap:
    """Retain one topic's latest payload plus bounded arrival-ordered history."""

    def __init__(self, topic: str, *, max_payloads: int, reconnect_s: float = 1.0):
        if not str(topic):
            raise ValueError("rolling PUBSUB tap requires a topic")
        if int(max_payloads) < 2:
            raise ValueError("rolling PUBSUB tap requires at least two payload slots")
        self._topic = str(topic)
        self._reconnect_s = float(reconnect_s)
        self._lock = threading.Lock()
        self._payloads = deque(maxlen=int(max_payloads))
        self._error = None
        self._thread = None

    def start(self):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return self
            self._thread = threading.Thread(
                target=self._listen_loop,
                name=f"zpnet-metrics-tail-{self._topic.lower()}",
                daemon=True,
            )
            self._thread.start()
        return self

    def get(self) -> dict | None:
        self.start()
        with self._lock:
            return dict(self._payloads[-1]) if self._payloads else None

    def history(self) -> list[dict]:
        self.start()
        with self._lock:
            return [dict(payload) for payload in self._payloads]

    def error(self) -> str | None:
        self.start()
        with self._lock:
            return self._error

    def _listen_loop(self) -> None:
        subscribe = (
            json.dumps(
                {"type": "set_topics", "topics": [self._topic]},
                separators=(",", ":"),
            ).encode("utf-8")
            + b"\n"
        )

        while True:
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                    sock.settimeout(2.0)
                    sock.connect(PUBSUB_TAP_SOCKET)
                    sock.settimeout(None)
                    with sock.makefile("rwb") as stream:
                        stream.write(subscribe)
                        stream.flush()
                        with self._lock:
                            self._error = None

                        for raw in stream:
                            try:
                                msg = json.loads(raw.decode("utf-8"))
                            except Exception:
                                continue
                            if not isinstance(msg, dict) or msg.get("type") != "publish":
                                continue
                            if str(msg.get("topic") or "") != self._topic:
                                continue
                            payload = msg.get("payload")
                            if not isinstance(payload, dict):
                                continue
                            with self._lock:
                                self._payloads.append(dict(payload))
                                self._error = None
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                time.sleep(self._reconnect_s)


_PHOTONS_LIVE_TAP = _RollingPubSubTap(
    PHOTONS_TOPIC,
    max_payloads=PHOTONS_ROLLING_ROWS + 1,
).start()

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


def _get_photons_snapshot() -> dict:
    """Return latest PHOTONS publication received by the live rolling tap."""
    return _PHOTONS_LIVE_TAP.get() or {}


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
    report["published_at_utc"] = root.get("published_at_utc")
    return report

def _get_clocks_baseline() -> dict | None:
    """Return the active campaign's referenced baseline campaign read model."""
    resp = send_command(
        machine="PI",
        subsystem="CLOCKS",
        command="BASELINE_INFO",
        retries=1,
        retry_delay_s=0.0,
    )
    if not isinstance(resp, dict) or not resp.get("success"):
        return None
    payload = resp.get("payload")
    if not isinstance(payload, dict) or not payload.get("baseline_set"):
        return None
    return payload

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

DAC_VOLTAGE_DECIMALS = 9
DAC_VOLTAGE_EXTRA_DECIMALS = DAC_VOLTAGE_DECIMALS - 6


def _dac_value(r: dict, lane: str):
    return _to_float(_field(r, f"control.{lane}.target_code", default=None))

def _dac_voltage(dac_code):
    code = _to_float(dac_code)
    if code is None:
        return None
    # Presentation mirror of Pi CLOCKS' AD5693R realization.
    return code * DAC_OUTPUT_FULL_SCALE_VOLTAGE / DAC_CODE_SCALE


def _dac_label(dac_code, dac_voltage=None):
    code = _to_float(dac_code)
    volts = _to_float(dac_voltage)
    if volts is None:
        volts = _dac_voltage(code)
    if code is None or volts is None:
        return "---"
    return f"{code:>.3f} {volts:.{DAC_VOLTAGE_DECIMALS}f}V"


def _dac_current_value(r: dict, lane: str):
    """Return the Pi-owned target code carried by canonical CLOCKS."""
    return _dac_value(r, lane)


def _dac_current_voltage(r: dict, lane: str):
    """Return presentation voltage for the canonical Pi-owned target."""
    return _dac_voltage(_dac_current_value(r, lane))


def _clamp_dac_code(value: float) -> float:
    if value < DAC_MIN_CODE:
        return DAC_MIN_CODE
    if value > DAC_MAX_CODE:
        return DAC_MAX_CODE
    return value


def _manual_dac_current_value(lane: str) -> float:
    """Fetch the live DAC value for keyboard nudging.

    Read the Pi-owned target from the current canonical CLOCKS.clocks.control
    surface.  Keyboard nudging never polls or commands the Teensy.
    """
    report = {}
    try:
        p = _get_pi_clocks_report()
        report = p.get("report") if isinstance(p.get("report"), dict) else p
    except Exception:
        report = {}

    current = _dac_current_value(report, lane)
    if current is None:
        raise RuntimeError(f"{lane.upper()} DAC value unavailable")

    return float(current)


def adjust_ocxo_dac(*, lane: str, direction: int, step_kind: str) -> dict:
    """Apply one keyboard DAC nudge through the sole PI CLOCKS actuator owner.

    The current target is read from canonical CLOCKS.control. Metrics computes
    one bounded coarse/fine target and sends exactly one PI CLOCKS SET_DAC
    command. The response payload is then authoritative for the accepted
    target, current hardware code, voltage, realization mode, and write status;
    no Teensy command/response path participates.
    """
    if lane not in ("ocxo1", "ocxo2"):
        raise ValueError(f"unknown DAC lane: {lane!r}")
    if direction not in (-1, 1):
        raise ValueError(f"invalid DAC direction: {direction!r}")

    coarse = "coarse" in str(step_kind).lower()
    step = DAC_COARSE_STEP if coarse else DAC_FINE_STEP
    old_value = _manual_dac_current_value(lane)
    requested_value = _clamp_dac_code(old_value + float(direction) * step)

    arg_name = "DAC1" if lane == "ocxo1" else "DAC2"
    response_target_key = f"{lane}_dac"
    response_hw_key = f"{lane}_dac_hw_code"
    response_voltage_key = f"{lane}_dac_voltage"
    response_write_key = f"{lane}_dac_last_write_ok"

    resp = send_command(
        machine="PI",
        subsystem="CLOCKS",
        command="SET_DAC",
        args={arg_name: f"{requested_value:.6f}"},
    )

    if not isinstance(resp, dict):
        raise RuntimeError("PI CLOCKS SET_DAC returned a malformed response")

    ok = bool(resp.get("success"))
    msg = str(resp.get("message") or ("OK" if ok else "FAILED"))
    payload = resp.get("payload") if isinstance(resp.get("payload"), dict) else {}

    accepted_value = _to_float(payload.get(response_target_key))
    if accepted_value is None:
        accepted_value = requested_value

    hw_code = _to_int(payload.get(response_hw_key))
    volts = _to_float(payload.get(response_voltage_key))
    if volts is None:
        volts = _dac_voltage(accepted_value)

    write_ok = _to_bool(payload.get(response_write_key))
    realization = str(payload.get("realization_mode") or "").upper()
    owner = str(payload.get("owner") or "PI.CLOCKS")

    voltage_text = f"{volts:.9f}V" if volts is not None else "---"
    hw_text = f" hw={hw_code}" if hw_code is not None else ""
    realization_text = f" {realization}" if realization else ""
    prefix = "DAC" if ok else "DAC FAILED"

    if ok:
        result_text = (
            f"{prefix}: {lane.upper()} {step_kind} "
            f"{old_value:.3f} -> {accepted_value:.3f}"
            f"{hw_text} {voltage_text}{realization_text} [{owner}]"
        )
    else:
        result_text = (
            f"{prefix}: {lane.upper()} {step_kind} "
            f"requested {requested_value:.3f}; {owner}: {msg}"
        )

    return {
        "success": ok,
        "message": result_text,
        "old_value": old_value,
        "requested_value": requested_value,
        "new_value": accepted_value,
        "hw_code": hw_code,
        "voltage": volts,
        "write_ok": write_ok,
        "realization_mode": realization or None,
        "owner": owner,
        "step": step,
        "lane": lane,
        "response": resp,
    }


def _dac_dither_summary_from_fractional_code(dac_code) -> str:
    """Format the two integer DAC codes used to realize a fractional code.

    Pi CLOCKS uses a one-second fractional-code realization:
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

def _dac_detail_lines(r: dict, baseline: dict | None) -> list[str]:
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
        dac_now = _dac_current_value(r, key)
        dac_voltage = _dac_current_voltage(r, key)
        dither_summary = _dac_timebase_dither_summary(r, key, dac_now)

        baseline_report = (
            baseline.get("baseline_report")
            if isinstance(baseline, dict)
            else None
        )
        base_dac = _campaign_dac_from_master_report(baseline_report, key)

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
    W_VALUE = 20
    W_PPB_BUCKET = 9
    W_RES = 6
    W_MEAN = 9
    W_SD = 7
    W_SE = 7
    W_N = 8
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


def _campaign_ppb_from_master_report(master_report: dict, lane: str):
    """Read one CAMP PPB value from campaign_master.payload.report."""
    if not isinstance(master_report, dict):
        return None
    if lane == "gnss_raw":
        return _to_float(_path_get(master_report, "extra_clocks.gnss_raw_ppb", None))
    return _to_float(_path_get(master_report, f"fragment.stats.ppb.{lane}", None))


def _campaign_dac_from_master_report(master_report: dict, lane: str):
    """Read the latest DAC target carried by a campaign master report."""
    return _to_float(_path_get(master_report, f"clocks.control.{lane}.target_code", None))

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

    report = p.get("report") if isinstance(p.get("report"), dict) else p
    if not isinstance(report, dict) or not report:
        return ["CLOCKS: FEED UNAVAILABLE", "", *feature_status_grid_lines()]

    r = report
    state = str(r.get("campaign_state") or ("STARTED" if r.get("campaign_present") else "STOPPED")).upper()
    campaign = r.get("campaign") or "STOPPED"
    elapsed = r.get("campaign_elapsed") or "00:00:00"
    instrument_elapsed = r.get("instrument_elapsed") or "00:00:00"
    n = _monitor_count(r)

    servo_state = _servo_state(r)

    baseline = _get_clocks_baseline()
    baseline_report = (
        baseline.get("baseline_report")
        if isinstance(baseline, dict)
        else None
    )
    baseline_campaign = baseline.get("baseline_campaign") if baseline else None

    servo_str = servo_state
    baseline_str = f"BASELINE: {baseline_campaign}" if baseline_campaign else "BASELINE: NONE"

    if state == "STARTED" or r.get("campaign_present"):
        identity = f"CAMPAIGN: {campaign}  ELAPSED: {elapsed}  n={n}"
    else:
        identity = f"CAMPAIGN: STOPPED  INSTRUMENT: {instrument_elapsed}  n={n}"
    lines.append(identity + f"    SERVO: {servo_str}" + f"    {baseline_str}")
    lines.append("")

    # ── Column widths ──
    W_NAME  = 6
    W_VALUE = 20
    W_PPB_BUCKET = 9
    W_RES   = 6
    W_MEAN  = 9
    W_SD    = 7
    W_SE    = 7
    W_N     = 7
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
    gnss_campaign_ppb = _campaign_ppb(r, "gnss")
    gnss_res = _to_int(_field(r, "gnss.second_residual_ns", "gnss_residual_ns")) or 0
    gnss_baseline_comp = _baseline_comp(
        _campaign_ppb_from_master_report(baseline_report, "gnss"),
        gnss_campaign_ppb,
        W_BASE,
    )
    lines.append(
        f"{'GNSS':<{W_NAME}}"
        f"{_comma_int(gnss_ns, W_VALUE)}"
        f"{_ppb_cols_fragment(r, 'gnss', W_PPB_BUCKET)}"
        f"{_sign_int(gnss_res, W_RES)}"
        f"{_welford_cols_fragment(r, 'gnss', W_MEAN, W_SD, W_SE, W_N)}"
        f" {gnss_baseline_comp}"
    )

    # ── VCLOCK (GNSS-disciplined nanosecond clock) ──
    vclock_ns  = _clockface_value(r, "vclock")
    vclock_campaign_ppb = _campaign_ppb(r, "vclock")
    vclock_res = _vclock_residual_ns(r)

    if vclock_res is None:
        vclock_res = 0
    vclock_baseline_comp = _baseline_comp(
        _campaign_ppb_from_master_report(baseline_report, "vclock"),
        vclock_campaign_ppb,
        W_BASE,
    )

    lines.append(
        f"{'VCLOCK':<{W_NAME}}"
        f"{_comma_int(vclock_ns, W_VALUE)}"
        f"{_ppb_cols_fragment(r, 'vclock', W_PPB_BUCKET)}"
        f"{_sign_int(vclock_res, W_RES)}"
        f"{_welford_cols_fragment_or_zero(r, 'vclock', W_MEAN, W_SD, W_SE, W_N)}"
        f" {vclock_baseline_comp}"
    )

    # ── OCXO1, OCXO2 (nanosecond clocks) ──
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        ocxo_ns = _clockface_value(r, key)
        campaign_ppb = _campaign_ppb(r, key)
        res          = _ocxo_residual_ns(r, key)
        campaign_baseline_comp = _baseline_comp(
            _campaign_ppb_from_master_report(baseline_report, key),
            campaign_ppb,
            W_BASE,
        )
        lines.append(
            f"{name:<{W_NAME}}"
            f"{_comma_int(ocxo_ns, W_VALUE)}"
            f"{_ppb_cols_fragment(r, key, W_PPB_BUCKET)}"
            f"{_sign_int(res, W_RES)}"
            f"{_welford_cols_fragment(r, key, W_MEAN, W_SD, W_SE, W_N)}"
            f" {campaign_baseline_comp}"
        )

    # GN_RAW and DWT remain available in CLOCKS and focused reports, but are
    # intentionally omitted from this operator-facing clock table.

    # ── DAC detail ──
    lines.append("")
    lines.extend(_dac_detail_lines(r, baseline))

    lines.append("")

    # ── Time ──
    gnss_time = r.get("gnss_time_utc") or _field(r, "time.gnss", default="---")
    system_time = r.get("system_time_utc") or r.get("published_at_utc") or _field(r, "time.system", default="---")
    lines.append(f"TIME  GNSS: {gnss_time}    SYSTEM: {system_time}")
    lines.append("")

    # ── Raw cycle static prediction ──
    raw_cycle_rows = (
        ("GNSS", "pps"),
        ("VCLOCK", "vclock"),
        ("OCXO1", "ocxo1"),
        ("OCXO2", "ocxo2"),
    )
    raw_cycle_values = []
    for name, lane in raw_cycle_rows:
        valid = _to_bool(_prediction_value(r, lane, "valid"))
        predicted = _to_int(_prediction_value(r, lane, "prediction_cycles"))
        actual = _to_int(_prediction_value(r, lane, "actual_cycles"))
        residual = _to_int(_prediction_value(r, lane, "residual_cycles"))
        if valid is not True:
            predicted = actual = residual = None
        raw_cycle_values.append((name, predicted, actual, residual))

    if any(value is not None for _, predicted, actual, residual in raw_cycle_values
           for value in (predicted, actual, residual)):
        cycle_name_w = 7
        cycle_count_w = 20
        cycle_res_w = 8
        lines.append(
            f"{'CLK':<{cycle_name_w}}"
            f"{'PREDICTED':>{cycle_count_w}}"
            f"{'ACTUAL':>{cycle_count_w}}"
            f"{'RES':>{cycle_res_w}}"
        )
        for name, predicted, actual, residual in raw_cycle_values:
            lines.append(
                f"{name:<{cycle_name_w}}"
                f"{_comma_int(predicted, cycle_count_w)}"
                f"{_comma_int(actual, cycle_count_w)}"
                f"{_sign_int(residual, cycle_res_w)}"
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


# ---------------------------------------------------------------------
# PHOTONS / LANTERN presentation
# ---------------------------------------------------------------------


def _json_object(value) -> dict:
    """Normalize a JSONB/text payload into a dictionary for read-only display."""
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        try:
            import json
            parsed = json.loads(value)
        except Exception:
            return {}
        return parsed if isinstance(parsed, dict) else {}
    return {}


def _photons_cumulative_stats(payload: dict | None) -> dict | None:
    """Return the firmware cumulative accepted-lap sufficient state."""
    root = _json_object(payload)
    instrument = root.get("photons") if isinstance(root.get("photons"), dict) else {}
    stats = instrument.get("stats") if isinstance(instrument.get("stats"), dict) else {}
    w = stats.get("lap_time") if isinstance(stats.get("lap_time"), dict) else {}
    science = instrument.get("science") if isinstance(instrument.get("science"), dict) else {}
    accepted = science.get("accepted") if isinstance(science.get("accepted"), dict) else {}
    excluded = science.get("excluded") if isinstance(science.get("excluded"), dict) else {}

    n = _to_int(w.get("n"))
    if n is None:
        n = _to_int(stats.get("lap_count"))
    mean = _to_float(w.get("mean"))
    m2 = _to_float(w.get("m2"))
    total = _to_int(stats.get("total_lap_gnss_ns"))
    accepted_total = _to_int(accepted.get("count"))
    excluded_total = _to_int(excluded.get("count"))

    if n is None or mean is None or m2 is None:
        return None
    return {
        "n": n,
        "mean": mean,
        "m2": m2,
        "total": total,
        "accepted_total": accepted_total,
        "excluded_total": excluded_total,
    }


def _photons_population_delta(before_payload: dict | None, after_payload: dict | None) -> dict | None:
    """Reverse two cumulative Welford snapshots into their exact intervening population.

    PHOTONS publishes sufficient state, so Metrics can reconstruct a campaign or
    one-second population without inventing samples or averaging repaint history.
    The grand-ratio total supplies the displayed mean when available; Welford
    subtraction supplies M2/SD/SE.
    """
    before = _photons_cumulative_stats(before_payload)
    after = _photons_cumulative_stats(after_payload)
    if after is None:
        return None

    if before is None:
        return None

    n0 = int(before["n"])
    n1 = int(after["n"])
    n = n1 - n0
    if n <= 0:
        return None

    # Reverse the Welford merge formula.  Use cumulative Welford means for the
    # cross term, then prefer the integer grand-ratio difference for display.
    if n0 == 0:
        subset_mean_w = float(after["mean"])
        subset_m2 = float(after["m2"])
    else:
        subset_mean_w = (
            n1 * float(after["mean"]) - n0 * float(before["mean"])
        ) / n
        delta = subset_mean_w - float(before["mean"])
        cross = delta * delta * n0 * n / n1
        subset_m2 = float(after["m2"]) - float(before["m2"]) - cross

    # Published doubles can leave a microscopic negative after subtraction.
    if subset_m2 < 0.0 and abs(subset_m2) < 1.0e-6 * max(1.0, abs(float(after["m2"]))):
        subset_m2 = 0.0
    if subset_m2 < 0.0:
        return None

    total0 = before.get("total")
    total1 = after.get("total")
    if total0 is not None and total1 is not None and int(total1) >= int(total0):
        mean = (int(total1) - int(total0)) / n
    else:
        mean = subset_mean_w

    sd = math.sqrt(subset_m2 / (n - 1)) if n >= 2 else 0.0
    se = sd / math.sqrt(n) if n >= 2 else 0.0

    accepted0 = before.get("accepted_total")
    accepted1 = after.get("accepted_total")
    excluded0 = before.get("excluded_total")
    excluded1 = after.get("excluded_total")

    return {
        "n": n,
        "mean": mean,
        "m2": subset_m2,
        "stddev": sd,
        "stderr": se,
        "accepted": (
            int(accepted1) - int(accepted0)
            if accepted0 is not None and accepted1 is not None and int(accepted1) >= int(accepted0)
            else n
        ),
        "excluded": (
            int(excluded1) - int(excluded0)
            if excluded0 is not None and excluded1 is not None and int(excluded1) >= int(excluded0)
            else None
        ),
    }


def _photons_instrument_stats(payload: dict | None) -> dict | None:
    """Return the current always-on accepted-lap Welford without differencing."""
    state = _photons_cumulative_stats(payload)
    if state is None:
        return None
    n = int(state["n"])
    sd = math.sqrt(float(state["m2"]) / (n - 1)) if n >= 2 else 0.0
    se = sd / math.sqrt(n) if n >= 2 else 0.0
    return {
        "n": n,
        "mean": float(state["mean"]),
        "m2": float(state["m2"]),
        "stddev": sd,
        "stderr": se,
        "accepted": state.get("accepted_total"),
        "excluded": state.get("excluded_total"),
    }


def _photons_ppb(observed_mean, baseline_mean):
    observed = _to_float(observed_mean)
    baseline = _to_float(baseline_mean)
    if observed is None or baseline is None or baseline == 0.0:
        return None
    return ((observed / baseline) - 1.0) * 1_000_000_000.0


def _get_lantern_campaign_summaries() -> list[dict]:
    """Return LANTERN masters plus exact campaign-local accepted-lap statistics.

    campaign_master remains provenance/latest-read-model authority.  The local
    Welford is reconstructed from the cumulative PHOTONS sufficient state at the
    campaign's exclusive start boundary and its latest labeled durable row.
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
                    master.payload,
                    baseline.id AS baseline_campaign_id,
                    baseline.campaign AS baseline_campaign,
                    first_detail.sequence AS first_sequence,
                    last_detail.sequence AS last_sequence,
                    prior_detail.payload AS prior_payload,
                    last_detail.payload AS last_payload
                FROM campaign_master AS master
                LEFT JOIN campaign_master AS baseline
                  ON baseline.id = (master.payload ->> 'baseline_campaign_id')::bigint
                 AND baseline.campaign_type = master.campaign_type
                LEFT JOIN LATERAL (
                    SELECT d.id, d.sequence, d.payload
                    FROM campaign_detail AS d
                    WHERE d.campaign_type = %s
                      AND d.campaign = master.campaign
                    ORDER BY d.id ASC
                    LIMIT 1
                ) AS first_detail ON true
                LEFT JOIN LATERAL (
                    SELECT d.id, d.sequence, d.payload
                    FROM campaign_detail AS d
                    WHERE d.campaign_type = %s
                      AND d.campaign = master.campaign
                    ORDER BY d.id DESC
                    LIMIT 1
                ) AS last_detail ON true
                LEFT JOIN LATERAL (
                    SELECT d.id, d.sequence, d.payload
                    FROM campaign_detail AS d
                    WHERE d.campaign_type = %s
                      AND d.id < first_detail.id
                    ORDER BY d.id DESC
                    LIMIT 1
                ) AS prior_detail ON true
                WHERE master.campaign_type = %s
                ORDER BY master.ts DESC, master.id DESC
                """,
                (
                    CAMPAIGN_TYPE_LANTERN,
                    CAMPAIGN_TYPE_LANTERN,
                    CAMPAIGN_TYPE_LANTERN,
                    CAMPAIGN_TYPE_LANTERN,
                ),
            )
            rows = list(cur.fetchall())

    summaries: list[dict] = []
    for row in rows:
        master_payload = _json_object(row.get("payload"))
        prior_payload = _json_object(row.get("prior_payload"))
        last_payload = _json_object(row.get("last_payload"))
        stats = _photons_population_delta(prior_payload, last_payload)
        summaries.append({
            "id": _to_int(row.get("id")),
            "campaign": row.get("campaign"),
            "active": bool(row.get("active")),
            "started_at": master_payload.get("started_at"),
            "stopped_at": master_payload.get("stopped_at"),
            "interrupted_at": master_payload.get("interrupted_at"),
            "start_after_sequence": _to_int(master_payload.get("start_after_sequence")),
            "stop_after_sequence": _to_int(master_payload.get("stop_after_sequence")),
            "first_sequence": _to_int(row.get("first_sequence")),
            "last_sequence": _to_int(row.get("last_sequence")),
            "baseline_campaign_id": _to_int(row.get("baseline_campaign_id")),
            "baseline_campaign": row.get("baseline_campaign"),
            "stats": stats,
            # Private presentation helpers used to derive live/rolling campaign
            # populations. They are never rendered as instrument testimony.
            "_prior_payload": prior_payload,
            "_last_payload": last_payload,
        })

    by_id = {s["id"]: s for s in summaries if s.get("id") is not None}
    for summary in summaries:
        base = by_id.get(summary.get("baseline_campaign_id"))
        base_stats = base.get("stats") if isinstance(base, dict) else None
        stats = summary.get("stats")
        base_mean = base_stats.get("mean") if isinstance(base_stats, dict) else None
        now_mean = stats.get("mean") if isinstance(stats, dict) else None
        summary["baseline_mean_lap_ns"] = base_mean
        summary["residual_ps"] = (
            (float(now_mean) - float(base_mean)) * 1000.0
            if now_mean is not None and base_mean is not None
            else None
        )
        summary["campaign_ppb"] = _photons_ppb(now_mean, base_mean)
    return summaries


def _get_photons_rolling_payloads() -> list[dict]:
    """Return arrival-ordered PHOTONS publications seen by this Metrics process.

    This is deliberately a live PUBSUB tail.  Metrics does not bootstrap or
    repair it from campaign_detail: after Metrics starts, the window fills from
    actual PHOTONS publications and stops moving immediately if the feed stops.
    """
    return _PHOTONS_LIVE_TAP.history()


def _photons_rolling_rows(live: dict, summaries: list[dict]) -> list[dict]:
    """Return oldest-to-newest one-second populations from the live PHOTONS tail."""
    _ = live
    payloads = _get_photons_rolling_payloads()
    by_id = {s.get("id"): s for s in summaries if s.get("id") is not None}
    rows: list[dict] = []

    for before, current in zip(payloads, payloads[1:]):
        second_stats = _photons_population_delta(before, current)
        if second_stats is None:
            continue

        instrument = current.get("photons") if isinstance(current.get("photons"), dict) else {}
        science = instrument.get("science") if isinstance(instrument.get("science"), dict) else {}
        accepted = science.get("accepted") if isinstance(science.get("accepted"), dict) else {}
        excluded = science.get("excluded") if isinstance(science.get("excluded"), dict) else {}
        accepted_this = _to_int(accepted.get("count_this_fragment"))
        excluded_this = _to_int(excluded.get("count_this_fragment"))
        if accepted_this is None:
            accepted_this = _to_int(second_stats.get("accepted"))
        if excluded_this is None:
            excluded_this = _to_int(second_stats.get("excluded"))

        sequence = _to_int(current.get("sequence"))
        campaign = current.get("campaign") if isinstance(current.get("campaign"), dict) else {}
        campaign_id = _to_int(campaign.get("campaign_id"))
        start_after = _to_int(campaign.get("start_after_sequence"))
        second_number = (
            sequence - start_after
            if sequence is not None and start_after is not None and sequence > start_after
            else sequence
        )

        summary = by_id.get(campaign_id)
        baseline_mean = (
            summary.get("baseline_mean_lap_ns")
            if isinstance(summary, dict)
            else None
        )
        campaign_to_date = None
        if isinstance(summary, dict) and summary.get("_prior_payload"):
            campaign_to_date = _photons_population_delta(
                summary.get("_prior_payload"), current
            )
        campaign_mean = (
            campaign_to_date.get("mean")
            if isinstance(campaign_to_date, dict)
            else None
        )

        rows.append({
            "sequence": sequence,
            "second": second_number,
            "campaign_id": campaign_id,
            "campaign": campaign.get("campaign"),
            "value_ns": second_stats.get("mean"),
            "accepted": accepted_this,
            "excluded": excluded_this,
            # Rolling PPB buckets do not exist yet. Keep them visibly absent.
            "ppb_10_min": None,
            "ppb_60_min": None,
            "ppb_8_hour": None,
            "ppb_24_hour": None,
            "ppb_total": None,
            "campaign_ppb": _photons_ppb(campaign_mean, baseline_mean),
            "residual_ps": (
                (float(second_stats["mean"]) - float(baseline_mean)) * 1000.0
                if second_stats.get("mean") is not None and baseline_mean is not None
                else None
            ),
            "mean": second_stats.get("mean"),
            "stddev": second_stats.get("stddev"),
            "stderr": second_stats.get("stderr"),
            "n": second_stats.get("n"),
        })

    return rows[-PHOTONS_ROLLING_ROWS:]


def photons_detail_readout() -> list[str]:
    """Render the live optical detail plus a fixed-header 25-second rolling tail."""
    live = _get_photons_snapshot()
    if not isinstance(live, dict) or not live:
        return ["PHOTONS: FEED UNAVAILABLE"]

    try:
        summaries = _get_lantern_campaign_summaries()
    except Exception as exc:
        summaries = []
        campaign_error = str(exc)
    else:
        campaign_error = None

    campaign = live.get("campaign") if isinstance(live.get("campaign"), dict) else {}
    campaign_id = _to_int(campaign.get("campaign_id"))
    summary_by_id = {s.get("id"): s for s in summaries}
    active_summary = summary_by_id.get(campaign_id)

    if campaign:
        start_after = _to_int(campaign.get("start_after_sequence"))
        sequence = _to_int(live.get("sequence"))
        elapsed_s = (
            max(0, sequence - start_after)
            if sequence is not None and start_after is not None
            else 0
        )
        campaign_name = str(campaign.get("campaign") or "?")
        baseline_name = campaign.get("baseline_campaign") or (
            active_summary.get("baseline_campaign") if isinstance(active_summary, dict) else None
        )
        identity = (
            f"PHOTONS  CAMPAIGN: {campaign_name}  ELAPSED: {_seconds_to_hms(elapsed_s)}"
            f"  BASELINE: {baseline_name or 'NONE'}"
        )
    else:
        identity = "PHOTONS  CAMPAIGN: STOPPED  INSTRUMENT: ALWAYS ON  BASELINE: NONE"

    lines = [identity]
    if campaign_error:
        lines.append(f"LANTERN READ MODEL: UNAVAILABLE: {campaign_error}")
    lines.append("")

    # Current campaign-local sufficient state when a campaign exists; otherwise
    # show the authoritative always-on instrument Welford.
    current_stats = None
    if isinstance(active_summary, dict) and active_summary.get("_prior_payload"):
        current_stats = _photons_population_delta(active_summary.get("_prior_payload"), live)
    if current_stats is None:
        current_stats = _photons_instrument_stats(live)

    baseline_mean = (
        active_summary.get("baseline_mean_lap_ns")
        if isinstance(active_summary, dict)
        else None
    )
    now_mean = current_stats.get("mean") if isinstance(current_stats, dict) else None
    campaign_ppb = _photons_ppb(now_mean, baseline_mean)

    try:
        rolling = _photons_rolling_rows(live, summaries)
    except Exception:
        rolling = []
    latest_residual = rolling[-1].get("residual_ps") if rolling else None

    W_NAME = 6
    W_VALUE = 14
    W_PPB = 9
    W_RES = 11
    W_MEAN = 12
    W_SD = 10
    W_SE = 10
    W_N = 9
    W_BASE = 12
    G = " "

    lines.append(
        f"{'LAP':<{W_NAME}}"
        f"{'VALUE_NS':>{W_VALUE}}{G}"
        f"{'10-MIN':>{W_PPB}}{G}"
        f"{'60-MIN':>{W_PPB}}{G}"
        f"{'8-HOUR':>{W_PPB}}{G}"
        f"{'24-HOUR':>{W_PPB}}{G}"
        f"{'TOTAL':>{W_PPB}}{G}"
        f"{'CAMP':>{W_PPB}}{G}"
        f"{'RES_PS':>{W_RES}}{G}"
        f"{'MEAN':>{W_MEAN}}{G}"
        f"{'SD':>{W_SD}}{G}"
        f"{'SE':>{W_SE}}{G}"
        f"{'N':>{W_N}}{G}"
        f"{'BASE':>{W_BASE}}{G}"
        f"{'NOW':>{W_BASE}}{G}"
        f"{'DELTA':>{W_BASE}}"
    )

    if isinstance(current_stats, dict):
        baseline_delta = (
            float(now_mean) - float(baseline_mean)
            if now_mean is not None and baseline_mean is not None
            else None
        )
        lines.append(
            f"{'LAP':<{W_NAME}}"
            f"{_fmt(now_mean, f'>{W_VALUE}.3f', W_VALUE)}{G}"
            f"{G.join(_fmt(None, f'>{W_PPB}.3f', W_PPB) for _ in range(5))}{G}"
            f"{_fmt(campaign_ppb, f'>{W_PPB}.3f', W_PPB)}{G}"
            f"{_fmt(latest_residual, f'>+{W_RES}.3f', W_RES)}{G}"
            f"{_fmt(current_stats.get('mean'), f'>{W_MEAN}.3f', W_MEAN)}{G}"
            f"{_fmt(current_stats.get('stddev'), f'>{W_SD}.3f', W_SD)}{G}"
            f"{_fmt(current_stats.get('stderr'), f'>{W_SE}.3f', W_SE)}{G}"
            f"{_fmt(_to_int(current_stats.get('n')), f'>{W_N}d', W_N)}{G}"
            f"{_fmt(baseline_mean, f'>{W_BASE}.3f', W_BASE)}{G}"
            f"{_fmt(now_mean, f'>{W_BASE}.3f', W_BASE)}{G}"
            f"{_fmt(baseline_delta, f'>+{W_BASE}.3f', W_BASE)}"
        )
    else:
        lines.append("PHOTONS statistics unavailable")

    lines.append("")
    lines.append(
        f"LIVE {PHOTONS_ROLLING_ROWS}-SECOND ACCEPTED-LAP TAIL  "
        "(PUBSUB; newest at bottom)"
    )

    W_SEC = 7
    W_ACC = 8
    W_EXCL = 8
    W_ROLL_VALUE = 14
    W_ROLL_MEAN = 12
    W_ROLL_SD = 10
    W_ROLL_SE = 10
    W_ROLL_N = 8

    # Fixed header: this readout itself is not scrollable; each newly received
    # PHOTONS publication advances the in-memory tail beneath this line.
    lines.append(
        f"{'SEC':>{W_SEC}}{G}"
        f"{'LAP_NS':>{W_ROLL_VALUE}}{G}"
        f"{'ACCEPT':>{W_ACC}}{G}"
        f"{'EXCL':>{W_EXCL}}{G}"
        f"{'10-MIN':>{W_PPB}}{G}"
        f"{'60-MIN':>{W_PPB}}{G}"
        f"{'8-HOUR':>{W_PPB}}{G}"
        f"{'24-HOUR':>{W_PPB}}{G}"
        f"{'TOTAL':>{W_PPB}}{G}"
        f"{'CAMP':>{W_PPB}}{G}"
        f"{'RES_PS':>{W_RES}}{G}"
        f"{'MEAN':>{W_ROLL_MEAN}}{G}"
        f"{'SD':>{W_ROLL_SD}}{G}"
        f"{'SE':>{W_ROLL_SE}}{G}"
        f"{'N':>{W_ROLL_N}}"
    )

    if not rolling:
        history_count = len(_get_photons_rolling_payloads())
        lines.append(
            f"WAITING FOR LIVE PHOTONS TAIL "
            f"({history_count}/2 publications needed for first one-second row)"
        )
        return lines

    for row in rolling:
        ppb_values = (
            row.get("ppb_10_min"),
            row.get("ppb_60_min"),
            row.get("ppb_8_hour"),
            row.get("ppb_24_hour"),
            row.get("ppb_total"),
            row.get("campaign_ppb"),
        )
        lines.append(
            f"{_fmt(_to_int(row.get('second')), f'>{W_SEC}d', W_SEC)}{G}"
            f"{_fmt(row.get('value_ns'), f'>{W_ROLL_VALUE}.3f', W_ROLL_VALUE)}{G}"
            f"{_fmt(_to_int(row.get('accepted')), f'>{W_ACC}d', W_ACC)}{G}"
            f"{_fmt(_to_int(row.get('excluded')), f'>{W_EXCL}d', W_EXCL)}{G}"
            f"{G.join(_fmt(v, f'>{W_PPB}.3f', W_PPB) for v in ppb_values)}{G}"
            f"{_fmt(row.get('residual_ps'), f'>+{W_RES}.3f', W_RES)}{G}"
            f"{_fmt(row.get('mean'), f'>{W_ROLL_MEAN}.3f', W_ROLL_MEAN)}{G}"
            f"{_fmt(row.get('stddev'), f'>{W_ROLL_SD}.3f', W_ROLL_SD)}{G}"
            f"{_fmt(row.get('stderr'), f'>{W_ROLL_SE}.3f', W_ROLL_SE)}{G}"
            f"{_fmt(_to_int(row.get('n')), f'>{W_ROLL_N}d', W_ROLL_N)}"
        )

    return lines


def photons_campaigns_readout() -> list[str]:
    """Render LANTERN campaign history with optical mean/Welford science."""
    try:
        rows = _get_lantern_campaign_summaries()
    except Exception as exc:
        return ["\0LANTERN_CAMPAIGNS:ERROR", f"LANTERN CAMPAIGNS: UNAVAILABLE: {exc}"]

    newest_identity = str(rows[0].get("id")) if rows else "EMPTY"
    lines = [f"\0LANTERN_CAMPAIGN:{newest_identity}"]

    W_CAMPAIGN = 20
    W_VALUE = 14
    W_PPB = 9
    W_RES = 11
    W_MEAN = 12
    W_SD = 10
    W_SE = 10
    W_N = 9
    W_BASELINE = 18
    G = " "

    lines.append(
        f"{'CAMPAIGN':<{W_CAMPAIGN}}{G}"
        f"{'LAP_NS':>{W_VALUE}}{G}"
        f"{'10-MIN':>{W_PPB}}{G}"
        f"{'60-MIN':>{W_PPB}}{G}"
        f"{'8-HOUR':>{W_PPB}}{G}"
        f"{'24-HOUR':>{W_PPB}}{G}"
        f"{'TOTAL':>{W_PPB}}{G}"
        f"{'CAMP':>{W_PPB}}{G}"
        f"{'RES_PS':>{W_RES}}{G}"
        f"{'MEAN':>{W_MEAN}}{G}"
        f"{'SD':>{W_SD}}{G}"
        f"{'SE':>{W_SE}}{G}"
        f"{'N':>{W_N}}{G}"
        f"{'BASELINE':<{W_BASELINE}}"
    )

    if not rows:
        lines.extend(["", "NO LANTERN CAMPAIGNS"])
        return lines

    for row in rows:
        name = str(row.get("campaign") or "?")
        if row.get("stats") is None:
            name += " [WAIT]"
        elif row.get("interrupted_at"):
            name += " [INT]"
        campaign_cell = _campaign_cell(name, bool(row.get("active")), W_CAMPAIGN)
        stats = row.get("stats") if isinstance(row.get("stats"), dict) else {}
        ppb_values = (None, None, None, None, None, row.get("campaign_ppb"))
        baseline_name = str(row.get("baseline_campaign") or "---")
        if len(baseline_name) > W_BASELINE:
            baseline_name = baseline_name[:W_BASELINE - 1] + "~"

        lines.append(
            f"{campaign_cell}{G}"
            f"{_fmt(stats.get('mean'), f'>{W_VALUE}.3f', W_VALUE)}{G}"
            f"{G.join(_fmt(v, f'>{W_PPB}.3f', W_PPB) for v in ppb_values)}{G}"
            f"{_fmt(row.get('residual_ps'), f'>+{W_RES}.3f', W_RES)}{G}"
            f"{_fmt(stats.get('mean'), f'>{W_MEAN}.3f', W_MEAN)}{G}"
            f"{_fmt(stats.get('stddev'), f'>{W_SD}.3f', W_SD)}{G}"
            f"{_fmt(stats.get('stderr'), f'>{W_SE}.3f', W_SE)}{G}"
            f"{_fmt(_to_int(stats.get('n')), f'>{W_N}d', W_N)}{G}"
            f"{baseline_name:<{W_BASELINE}}"
        )

    return lines


READOUTS = [
    ("CLOCKS", lambda: _safe_lines(clocks_combined_readout), False),
    ("CAMPAIGNS", lambda: _safe_lines(campaigns_readout), True),
    ("PHOTONS", lambda: _safe_lines(photons_detail_readout), False),
    ("PHOTON CAMPAIGNS", lambda: _safe_lines(photons_campaigns_readout), True),
]
