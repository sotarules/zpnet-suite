"""Completed campaign endpoints for the live operator displays."""

import logging
import math
import threading
import time

from zpnet.processes.processes import send_command


class LastCompletedCampaign:
    """Read the latest completed endpoints without blocking a display refresh."""

    def __init__(self, subsystem):
        self.subsystem = subsystem
        self.row = None
        self.next_read = 0
        self.lock = threading.Lock()

    def get(self):
        with self.lock:
            if time.monotonic() >= self.next_read:
                self.next_read = math.inf
                threading.Thread(target=self._refresh, daemon=True).start()
            return self.row

    def _refresh(self):
        row = None
        try:
            response = send_command(machine="PI", subsystem=self.subsystem, command="LIST_CAMPAIGNS",
                                    args={"latest_closed": True}, retries=1)
            if response["success"]:
                rows = response["payload"]["campaigns"]
                if len(rows) == 1 and not rows[0]["active"]:
                    row = rows[0]
        except Exception:
            logging.exception("Could not read the last completed %s campaign", self.subsystem)
        finally:
            with self.lock:
                self.row = row
                self.next_read = time.monotonic() + 5


last_tempest = LastCompletedCampaign("CLOCKS")
last_lantern = LastCompletedCampaign("PHOTONS")
