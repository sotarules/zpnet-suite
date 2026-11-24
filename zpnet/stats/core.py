"""HTTP service for the ZPNet field display."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path

from zpnet.processes.processes import create_pubsub_cache


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


def snapshot(cache):
    clocks = fresh(cache, "CLOCKS")
    photons = fresh(cache, "PHOTONS")
    system_time = field(clocks, "clocks", "system_time_utc") or field(
        clocks, "published_at_utc"
    )
    flight = field(photons, "photons", "race", "flight_ns") or {}
    lap = flight.get("mean") if int(flight.get("n", 0)) > 0 else None
    stats = field(clocks, "clocks", "stats")
    return {
        "system_time": system_time[11:19] if system_time else None,
        "altitude": number(field(clocks, "gnss", "altitude_m"), ",.0f"),
        "max31865": temperature(field(clocks, "rtd")),
        "bme_pd": temperature(field(clocks, "environment", "pd")),
        "bme_out": temperature(field(clocks, "environment", "out")),
        "lap": number(lap, ",.6f"),
        "ocxo1": number(field(stats, "ocxo1", "ppb_buckets", "10_min"), "+.3f"),
        "ocxo2": number(field(stats, "ocxo2", "ppb_buckets", "10_min"), "+.3f"),
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
