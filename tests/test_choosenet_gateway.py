"""Regression checks for field Wi-Fi gateway retention and hotspot failover."""

import subprocess
import unittest
from unittest.mock import patch

from zpnet.zpnet_choosenet import core


class ChoosenetGatewayTests(unittest.TestCase):
    @patch.object(core.subprocess, "run")
    def test_associated_hotspot_with_gateway_is_healthy_without_internet(self, run):
        run.side_effect = [
            subprocess.CompletedProcess(["iwgetid"], 0, 'wlan0    ESSID:"Galaxy S23"\n'),
            subprocess.CompletedProcess(["ip"], 0, "default via 192.168.43.1 dev wlan0\n"),
        ]
        self.assertTrue(core.wifi_gateway_available())
        self.assertEqual(run.call_args.args[0],
                         ["ip", "-4", "route", "show", "default", "dev", "wlan0"])

    @patch.object(core.subprocess, "run")
    def test_disassociated_hotspot_requires_recovery(self, run):
        run.return_value = subprocess.CompletedProcess(["iwgetid"], 1, "")
        self.assertFalse(core.wifi_gateway_available())
        run.assert_called_once()

    @patch.object(core.subprocess, "run")
    def test_associated_hotspot_without_gateway_requires_recovery(self, run):
        run.side_effect = [
            subprocess.CompletedProcess(["iwgetid"], 0, 'wlan0    ESSID:"Galaxy S23"\n'),
            subprocess.CompletedProcess(["ip"], 0, ""),
        ]
        self.assertFalse(core.wifi_gateway_available())

    @patch.object(core, "run_choosenet")
    @patch.object(core.time, "sleep", side_effect=StopIteration)
    @patch.object(core, "wifi_gateway_available", return_value=True)
    @patch.object(core, "get_ssid", return_value="iPhone")
    def test_healthy_local_network_never_invokes_healer(self, ssid, gateway, sleep, healer):
        with self.assertRaises(StopIteration):
            core.run()
        healer.assert_not_called()
        sleep.assert_called_once_with(core.CHOOSENET_RETRY_INTERVAL_S)

    @patch.object(core, "run_choosenet")
    @patch.object(core.time, "sleep", side_effect=[None, StopIteration])
    @patch.object(core, "wifi_gateway_available", side_effect=[False, True])
    @patch.object(core, "get_ssid", return_value="iPhone")
    def test_transient_route_loss_does_not_bounce_wifi(self, ssid, gateway, sleep, healer):
        with self.assertRaises(StopIteration):
            core.run()
        healer.assert_not_called()
        self.assertEqual(gateway.call_count, 2)
        self.assertEqual(sleep.call_args_list[0].args[0], core.GATEWAY_CONFIRM_DELAY_S)

    @patch.object(core, "run_choosenet", return_value=(False, ""))
    @patch.object(core.time, "sleep", side_effect=[None, StopIteration])
    @patch.object(core, "wifi_gateway_available", side_effect=[False, False])
    @patch.object(core, "get_ssid", return_value="")
    def test_dead_hotspot_invokes_healer_after_confirmation(self, ssid, gateway, sleep, healer):
        with self.assertRaises(StopIteration):
            core.run()
        healer.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
