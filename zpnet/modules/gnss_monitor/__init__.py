"""
ZPNet GNSS Monitor package

Requests lightweight GNSS liveness snapshots from the Teensy
and relies on downstream aggregation for health classification.

Author: The Mule + GPT
"""

from .core import run, bootstrap