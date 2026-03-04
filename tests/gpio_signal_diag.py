"""gpio_signal_diag — Build and run GPIO 25 signal quality diagnostic

Compiles gpio_signal_diag.c, pins to isolated core, and runs.
Reports phase durations, glitch detection, and histogram.

Usage:
  sudo -E zt gpio_signal_diag [num_transitions]

Default: 100000 transitions (~5 seconds of 10 KHz signal)
"""

import os
import subprocess
import sys

NATIVE_DIR = os.path.expanduser("~/zpnet/zpnet/native/pulsecapture")
SRC = os.path.join(NATIVE_DIR, "gpio_signal_diag.c")
BIN = os.path.join(NATIVE_DIR, "gpio_signal_diag")

ISOLATED_CORE = 3
NUM_TRANSITIONS = sys.argv[1] if len(sys.argv) > 1 else "100000"

# Build if needed (or if source is newer)
if not os.path.isfile(SRC):
    print(f"❌ Source not found: {SRC}")
    print("   Place gpio_signal_diag.c in ~/zpnet/zpnet/native/pulsecapture/")
    sys.exit(1)

needs_build = (
    not os.path.isfile(BIN) or
    os.path.getmtime(SRC) > os.path.getmtime(BIN)
)

if needs_build:
    print(f"Building {BIN}...")
    rc = subprocess.run(
        ["gcc", "-O2", "-o", BIN, SRC],
        capture_output=True, text=True,
    )
    if rc.returncode != 0:
        print(f"❌ Build failed:\n{rc.stderr}")
        sys.exit(1)
    print("✅ Built successfully\n")

# Run pinned to isolated core
cmd = ["taskset", "-c", str(ISOLATED_CORE), BIN, NUM_TRANSITIONS]
print(f"Running: {' '.join(cmd)}\n")
os.execvp("taskset", cmd)