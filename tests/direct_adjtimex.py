import subprocess, struct, fcntl, os

# Direct syscall to adjtimex
import ctypes

class timex(ctypes.Structure):
    _fields_ = [
        ('modes', ctypes.c_uint),
        ('offset', ctypes.c_long),
        ('freq', ctypes.c_long),
        ('maxerror', ctypes.c_long),
        ('esterror', ctypes.c_long),
        ('status', ctypes.c_int),
        ('constant', ctypes.c_long),
        ('precision', ctypes.c_long),
        ('tolerance', ctypes.c_long),
        ('time_sec', ctypes.c_long),
        ('time_usec', ctypes.c_long),
        ('tick', ctypes.c_long),
        ('ppsfreq', ctypes.c_long),
        ('jitter', ctypes.c_long),
        ('shift', ctypes.c_int),
        ('stabil', ctypes.c_long),
        ('jitcnt', ctypes.c_long),
        ('calcnt', ctypes.c_long),
        ('errcnt', ctypes.c_long),
        ('stbcnt', ctypes.c_long),
        ('tai', ctypes.c_int),
    ]

libc = ctypes.CDLL('libc.so.6')
tx = timex()
tx.modes = 0  # read-only
libc.adjtimex(ctypes.byref(tx))

ppm_classic = tx.freq / 65536.0
ppb_classic = ppm_classic * 1000

print(f'Raw freq field:  {tx.freq}')
print(f'Classic PPM:     {ppm_classic:.3f}')
print(f'Classic PPB:     {ppb_classic:.1f}')
print(f'Status:          {tx.status} (0x{tx.status:04x})')
print(f'Tick:            {tx.tick}')
print()
print(f'If chrony says 7.698 ppm, expected raw = {7.698 * 65536:.0f}')
