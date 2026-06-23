#!/usr/bin/env python3
"""Listen to live ZPNet pub/sub topics through the PUBSUB ad-hoc tap.

This replaces the old "tail zpnet-pubsub.log | grep TOPIC" style debugging
loop with a live subscription to PUBSUB's observer-only tap socket.

Usage:
    python tests/pubsub_listen.py TIMEBASE
    python tests/pubsub_listen.py CLOCKS_DAC_TICK SYSTEM_FEATURES_TICK
    python tests/pubsub_listen.py '*' --pretty

The program prints one JSON object per matching publication.  Control-plane
messages from the tap itself are written to stderr so stdout remains usable for
pipelines such as jq.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Any, Dict, List


DEFAULT_SOCKET = "/tmp/zpnet_pubsub_tap.sock"


def _compact(obj: Any) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False)


def _pretty(obj: Any) -> str:
    return json.dumps(obj, indent=2, ensure_ascii=False)


def _emit(obj: Any, *, pretty: bool) -> None:
    print(_pretty(obj) if pretty else _compact(obj), flush=True)


def _send(sock: socket.socket, msg: Dict[str, Any]) -> None:
    sock.sendall((_compact(msg) + "\n").encode("utf-8"))


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Listen to ZPNet pub/sub topics through /tmp/zpnet_pubsub_tap.sock",
    )
    parser.add_argument(
        "topics",
        nargs="+",
        help="topic name(s) to listen for, for example TIMEBASE or CLOCKS_DAC_TICK; '*' listens to all topics",
    )
    parser.add_argument(
        "--socket",
        default=DEFAULT_SOCKET,
        help=f"tap socket path (default: {DEFAULT_SOCKET})",
    )
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="pretty-print JSON instead of compact newline JSON",
    )
    parser.add_argument(
        "--payload-only",
        action="store_true",
        help="print only the publication payload instead of the tap envelope",
    )
    return parser.parse_args(argv)


def main(argv: List[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.connect(args.socket)
        _send(sock, {"type": "set_topics", "topics": args.topics})

        buf = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                print("tap disconnected", file=sys.stderr)
                return 1

            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue

                try:
                    msg = json.loads(line.decode("utf-8"))
                except json.JSONDecodeError:
                    print(f"bad JSON from tap: {line!r}", file=sys.stderr)
                    continue

                if msg.get("type") != "publish":
                    print(_compact(msg), file=sys.stderr, flush=True)
                    continue

                obj = msg.get("payload") if args.payload_only else msg
                _emit(obj, pretty=args.pretty)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
