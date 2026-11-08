"""Lossless, process-lifetime storage for ordered PHOTONS persistence work.

Expanded recovery images cost far more RAM than their serialized contents.
Keep a bounded compressed payload budget and spill excess to private files on
disk. This is staging, NOT durable recovery: PostgreSQL remains the authority.
Files are never discovered or replayed after restart. A killed process may leave
an orphan photons-persistence-* directory; it contains no committed authority.

Pickle preserves Python integers, floats and container types without rounding.
Only bytes created by this instance are loaded, from its own private directory;
never accept operator-supplied pickle files. No observations are coalesced.
"""

from collections import Counter, deque
from dataclasses import dataclass
import os
from pathlib import Path
import pickle
import queue
import tempfile
import threading
import zlib


PERSISTENCE_MEMORY_BYTES = 32 * 1024 * 1024
# /tmp can be tmpfs, which would defeat spilling. Keep this on disk-backed
# storage; an installation may explicitly select its data volume instead.
PERSISTENCE_SPOOL_DIRECTORY = os.environ.get("ZPNET_PHOTONS_SPOOL_DIR", "/var/tmp")


@dataclass
class _Entry:
    payload: bytes | Path
    size: int
    campaign: str


class PersistenceQueue:
    """Single producer/consumer FIFO with small metadata per pending row.

    The budget covers retained compressed payloads, not Python metadata or the
    one image being encoded/decoded. Disk usage grows with backlog. Disk-full
    and decoding failures propagate to the subsystem's integrity failure path;
    a failed dequeue leaves the head in place. No silent eviction or fallback
    to an unlimited RAM queue is permitted.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._entries = deque()
        self._campaigns = Counter()
        self._directory = None
        self._memory_bytes = 0
        self._disk_bytes = 0
        self._disk_entries = 0
        self._spilled_total = 0

    def put_nowait(self, item):
        # Serialize before publication into the FIFO. The producer can release
        # every nested recovery object after this call, including shared ones.
        payload = zlib.compress(pickle.dumps(item, protocol=pickle.HIGHEST_PROTOCOL), 1)
        campaign = str((item["photons"].get("campaign") or {}).get("campaign") or "").strip()
        size = len(payload)
        with self._lock:
            stored = payload
            if self._memory_bytes + size > PERSISTENCE_MEMORY_BYTES:
                if self._directory is None:
                    self._directory = tempfile.TemporaryDirectory(
                        prefix="photons-persistence-", dir=PERSISTENCE_SPOOL_DIRECTORY,
                    )
                # Close each file immediately: backlog must not consume an fd
                # per observation. No fsync: this queue is explicitly volatile.
                with tempfile.NamedTemporaryFile(dir=self._directory.name, delete=False) as stream:
                    path = Path(stream.name)
                    try:
                        stream.write(payload)
                        stream.flush()
                    except BaseException:
                        path.unlink()
                        raise
                stored = path
                self._disk_bytes += size
                self._disk_entries += 1
                self._spilled_total += 1
            else:
                self._memory_bytes += size
            self._entries.append(_Entry(stored, size, campaign))
            if campaign:
                self._campaigns[campaign] += 1

    def _remove_head(self):
        entry = self._entries[0]
        if isinstance(entry.payload, Path):
            entry.payload.unlink()
            self._disk_bytes -= entry.size
            self._disk_entries -= 1
        else:
            self._memory_bytes -= entry.size
        self._entries.popleft()
        if entry.campaign:
            self._campaigns[entry.campaign] -= 1
            if not self._campaigns[entry.campaign]:
                del self._campaigns[entry.campaign]

    def get_nowait(self):
        with self._lock:
            if not self._entries:
                raise queue.Empty
            entry = self._entries[0]
            payload = entry.payload.read_bytes() if isinstance(entry.payload, Path) else entry.payload
            item = pickle.loads(zlib.decompress(payload))
            self._remove_head()
            return item

    def discard_pending(self):
        """Existing explicit maintenance retirement; no decode or replay."""
        with self._lock:
            count = len(self._entries)
            while self._entries:
                self._remove_head()
            return count

    def qsize(self):
        with self._lock:
            return len(self._entries)

    def empty(self):
        return self.qsize() == 0

    def campaign_names(self):
        with self._lock:
            return set(self._campaigns)

    def snapshot(self):
        with self._lock:
            return {
                "schema": "PHOTONS_PERSISTENCE_STORAGE_V1",
                "queue_depth": len(self._entries),
                "memory_payload_bytes": self._memory_bytes,
                "memory_payload_limit_bytes": PERSISTENCE_MEMORY_BYTES,
                "disk_payload_bytes": self._disk_bytes,
                "disk_entries": self._disk_entries,
                "spilled_entries_total": self._spilled_total,
                "spool_directory": self._directory.name if self._directory else None,
                "restart_replay": False,
            }
