"""Unit tests for pausing ZPNet Server REST egress during field work."""

import unittest
from unittest.mock import patch

from zpnet.processes.events import core


class EventsRemoteEgressTests(unittest.TestCase):
    def _start_until_main_sleep(self, enabled):
        with (patch.object(core, "REMOTE_DESPOOL_ENABLED", enabled),
              patch.object(core, "setup_logging"),
              patch.object(core, "server_setup") as setup,
              patch.object(core.threading, "Thread") as thread,
              patch.object(core.time, "sleep", side_effect=KeyboardInterrupt)):
            with self.assertRaises(KeyboardInterrupt):
                core.run()
            setup.assert_called_once_with(
                subsystem="EVENTS",
                publication_handler=core.on_publication,
                blocking=False,
            )
            return thread

    def test_default_pauses_both_rest_despoolers(self):
        self.assertFalse(core.REMOTE_DESPOOL_ENABLED)
        thread = self._start_until_main_sleep(False)
        thread.assert_not_called()

    def test_reenable_restores_both_existing_workers(self):
        thread = self._start_until_main_sleep(True)
        self.assertEqual(
            [call.kwargs["name"] for call in thread.call_args_list],
            ["events-despooler", "campaign-detail-despooler"],
        )
        self.assertEqual(thread.return_value.start.call_count, 2)

    def test_local_teensy_events_still_persist(self):
        with patch.object(core, "create_event") as create:
            core.on_publication(
                "EVENTS",
                {"events": [{"event_type": "FIELD_EVENT", "payload": {"run": 1}}]},
            )
            create.assert_called_once_with("FIELD_EVENT", {"run": 1})


if __name__ == "__main__":
    unittest.main()
