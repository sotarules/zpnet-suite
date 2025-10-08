"""
ZPNet package entrypoint.

Allows running `python3 -m zpnet` to launch the scheduler.
"""

from . import main


if __name__ == "__main__":
    main.bootstrap()
