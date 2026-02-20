"""
zpnet.native.pps_raw — Raw PPS capture via /dev/pps_raw kernel module

Captures raw ARM arch timer (CNTVCT_EL0) values at PPS GPIO
interrupt edges, free of chrony/NTP rate adjustments.
"""

from zpnet.native.pps_raw.pps_raw import PpsRawReader, PpsRawCapture

__all__ = ["PpsRawReader", "PpsRawCapture"]
