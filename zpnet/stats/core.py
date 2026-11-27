"""HTTP service for the ZPNet field display."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path

from zpnet.processes.processes import create_pubsub_cache
from zpnet.shared.campaign_endpoints import last_lantern, last_tempest
from zpnet.shared.display_format import altitude_text


FONT = Path("/usr/share/fonts/truetype/3270/3270-Regular.ttf")
FEED_MAX_AGE_S = 4.0


def field(payload, *keys):
    for key in keys:
        if not isinstance(payload, dict):
            return None
        payload = payload.get(key)
    return payload


def number(value, spec):
    if value is None:
        return None
    value = float(value)
    return format(value, spec) if math.isfinite(value) else None


def temperature(reading):
    if field(reading, "status") == "OK":
        return number(reading.get("temperature_c"), ".2f")
    return None


def fresh(cache, topic):
    age = cache.age_s(topic)
    return cache.get(topic) if age is not None and age <= FEED_MAX_AGE_S else None


def difference(start, end):
    return end - start if start is not None and end is not None else None


def altitude_bookends(campaign, observation, active):
    start = campaign.get("start_altitude_m")
    end = (field(observation, "gnss", "altitude_m") if active
           else campaign.get("end_altitude_m"))
    return {
        "start_alt": altitude_text(start),
        "end_alt": altitude_text(end),
        "delta_alt": altitude_text(difference(start, end), signed=True),
    }


def tempest_snapshot(clocks):
    if clocks is None:
        return None
    campaign = clocks.get("campaign") or {}
    active = campaign.get("state") == "STARTED"
    if not active:
        campaign = last_tempest.get()
    if not campaign:
        return None
    window = campaign.get("ppb_bookend_window") or "60_sec"
    result = {"name": campaign["name" if active else "campaign"], "active": active,
              "ppb_window": "10-MIN" if window == "10_min" else "60-SEC",
              **altitude_bookends(campaign, clocks, active)}
    for lane in ("ocxo1", "ocxo2"):
        start = field(campaign, "start_ppb", lane)
        end = (field(clocks, "clocks", "stats", lane, "ppb_buckets", window)
               if active else field(campaign, "end_ppb", lane))
        result[lane] = {"start": number(start, "+.3f"), "end": number(end, "+.3f"),
                        "delta": number(difference(start, end), "+.3f")}
    return result


def lantern_snapshot(photons):
    if photons is None:
        return None
    campaign = photons.get("campaign") or {}
    active = bool(campaign) and not campaign.get("final")
    if not active:
        campaign = last_lantern.get()
    if not campaign:
        return None
    start = campaign.get("start_lap_ns")
    end = (field(photons, "photons", "stats", "lap_buckets", "60_sec", "mean_lap_ns")
           if active else campaign.get("end_lap_ns"))
    return {
        "name": campaign["campaign"],
        "active": active,
        **altitude_bookends(campaign, photons, active),
        "start_lap": number(start, ",.6f"),
        "end_lap": number(end, ",.6f"),
        "delta": number(difference(start, end), "+.6f"),
    }


def snapshot(cache):
    clocks = fresh(cache, "CLOCKS")
    photons = fresh(cache, "PHOTONS")
    system_time = field(clocks, "clocks", "system_time_utc") or field(
        clocks, "published_at_utc"
    )
    lap = field(photons, "photons", "stats", "lap_buckets", "60_sec", "mean_lap_ns")
    high5 = field(photons, "photons", "pd_out", "high5_window") or {}
    stats = field(clocks, "clocks", "stats")
    return {
        "system_time": system_time[11:19] if system_time else None,
        "altitude": altitude_text(field(clocks, "gnss", "altitude_m")),
        "max31865": temperature(field(clocks, "rtd")),
        "bme_pd": temperature(field(clocks, "environment", "pd")),
        "bme_out": temperature(field(clocks, "environment", "out")),
        "lap": number(lap, ",.6f"),
        "high5": number(high5.get("high_mean_mv"), ".3f") if high5.get("sample_count", 0) else None,
        "lantern": lantern_snapshot(photons),
        "ocxo1": number(field(stats, "ocxo1", "ppb_buckets", "60_sec"), "+.3f"),
        "ocxo2": number(field(stats, "ocxo2", "ppb_buckets", "60_sec"), "+.3f"),
        "ocxo1_10": number(field(stats, "ocxo1", "ppb_buckets", "10_min"), "+.3f"),
        "ocxo2_10": number(field(stats, "ocxo2", "ppb_buckets", "10_min"), "+.3f"),
        "tempest": tempest_snapshot(clocks),
        "clocks_live": clocks is not None,
        "photons_live": photons is not None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--font", type=Path, default=FONT, help="IBM 3270 Regular TTF")
    args = parser.parse_args()

    try:
        font = args.font.read_bytes()
    except OSError as exc:
        parser.error(f"IBM 3270 font unavailable: {exc}. Install fonts-3270 or use --font.")
    assets = {
        "/": ("text/html; charset=utf-8", Path(__file__).with_name("index.html").read_bytes()),
        "/3270.ttf": ("font/ttf", font),
    }

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/stats.json":
                content_type = "application/json"
                body = json.dumps(snapshot(cache)).encode()
            elif self.path in assets:
                content_type, body = assets[self.path]
            else:
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "max-age=86400" if self.path == "/3270.ttf" else "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_request(self, code="-", size="-"):
            if self.path != "/stats.json" or code != 200:
                super().log_request(code, size)

    cache = create_pubsub_cache("CLOCKS", "PHOTONS")
    with ThreadingHTTPServer((args.host, args.port), Handler) as server:
        print(f"ZPNet STATS listening on {args.host}:{server.server_port}", flush=True)
        server.serve_forever()
