"""
ZPNet GNSS Data Monitor package

Requests authoritative GNSS_DATA snapshots from the Teensy.
These snapshots represent canonical clock truth.

Author: The Mule + GPT
"""

from .core import run, bootstrap
