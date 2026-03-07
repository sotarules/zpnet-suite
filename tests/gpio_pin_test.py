"""gpio25_python_edge_test — Watch GPIO25 edges from Python via libgpiod"""

import sys
import time

try:
    import gpiod
except Exception as e:
    print("ERROR: Python gpiod bindings not available.")
    print(f"Import failure: {e}")
    sys.exit(1)

CHIP_NAME = "/dev/gpiochip0"
LINE_OFFSET = 25
RUN_SECONDS = 12.0

print("=== GPIO25 Python Edge Test ===\n")
print(f"Chip:         {CHIP_NAME}")
print(f"Line offset:  {LINE_OFFSET}")
print(f"Run seconds:  {RUN_SECONDS}")
print()

chip = gpiod.Chip(CHIP_NAME)

request = chip.request_lines(
    config={
        LINE_OFFSET: gpiod.LineSettings(
            direction=gpiod.line.Direction.INPUT,
            edge_detection=gpiod.line.Edge.BOTH,
            bias=gpiod.line.Bias.DISABLED,
        )
    },
    consumer="gpio25_python_edge_test",
)

print("Listening for edges...\n")

start_monotonic = time.monotonic()
last_event_ts_ns = None
last_rising_ts_ns = None
last_falling_ts_ns = None
edge_count = 0

def classify_event(ev) -> str:
    et = getattr(ev, "event_type", None)

    # Common libgpiod python variants
    name = str(et)
    if "RISING" in name.upper():
        return "RISING"
    if "FALLING" in name.upper():
        return "FALLING"

    # Numeric fallback seen in some builds
    if et == 1:
        return "RISING"
    if et == 2:
        return "FALLING"

    return f"TYPE={et}"

try:
    while True:
        if time.monotonic() - start_monotonic >= RUN_SECONDS:
            break

        if not request.wait_edge_events(timeout=1.0):
            continue

        events = request.read_edge_events()
        for ev in events:
            edge_count += 1

            ts_ns = ev.timestamp_ns
            kind = classify_event(ev)

            if last_event_ts_ns is None:
                delta_prev_ms = None
            else:
                delta_prev_ms = (ts_ns - last_event_ts_ns) / 1e6

            if kind == "RISING":
                if last_rising_ts_ns is None:
                    delta_same_ms = None
                else:
                    delta_same_ms = (ts_ns - last_rising_ts_ns) / 1e6
                last_rising_ts_ns = ts_ns
            elif kind == "FALLING":
                if last_falling_ts_ns is None:
                    delta_same_ms = None
                else:
                    delta_same_ms = (ts_ns - last_falling_ts_ns) / 1e6
                last_falling_ts_ns = ts_ns
            else:
                delta_same_ms = None

            last_event_ts_ns = ts_ns

            print(
                f"{edge_count:03d}  {kind:8s}  "
                f"ts_ns={ts_ns}  "
                f"dt_prev_ms={delta_prev_ms if delta_prev_ms is not None else '—'}  "
                f"dt_same_ms={delta_same_ms if delta_same_ms is not None else '—'}"
            )

finally:
    request.release()

print("\n=== Done ===")