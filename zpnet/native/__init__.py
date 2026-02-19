"""
zpnet.native — Native (C) shared libraries for ZPNet

This package contains Python wrappers for performance-critical
native code that runs on the Pi.  Each module loads a companion
.so built from C sources in this directory.

Available modules:
    systimer    ARM Generic Timer (CNTVCT_EL0) access

Build all native libraries:
    cd zpnet/native && make
"""