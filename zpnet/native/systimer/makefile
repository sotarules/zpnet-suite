# zpnet/native/Makefile
#
# Build native shared libraries for ZPNet.
# Run on the Pi (AArch64) — not cross-compiled.
#
# Usage:
#   make            Build all libraries
#   make clean      Remove built artifacts
#   make install    Copy to Python package (zpnet/native/)
#
# Libraries:
#   libsystimer.so  ARM Generic Timer (CNTVCT_EL0) access

CC      = gcc
CFLAGS  = -shared -fPIC -O2 -Wall -Wextra -Werror

# All shared libraries to build
LIBS = libsystimer.so

.PHONY: all clean

all: $(LIBS)

# ---------------------------------------------------------------
# systimer — ARM architected system timer
# ---------------------------------------------------------------

libsystimer.so: systimer.c systimer.h
	$(CC) $(CFLAGS) -o $@ systimer.c

# ---------------------------------------------------------------
# Housekeeping
# ---------------------------------------------------------------

clean:
	rm -f $(LIBS)