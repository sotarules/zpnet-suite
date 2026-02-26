/*
 * ppsfetch.c — PPS_FETCH ioctl wrapper for /dev/pps0
 *
 * Two functions:
 *   pps_open(path)          — open a PPS device, return fd
 *   pps_fetch_assert(fd, timeout_sec, sec_out, nsec_out, seq_out)
 *                           — block until next PPS assert edge
 *
 * The kernel timestamps the GPIO interrupt with CLOCK_REALTIME at the
 * actual PPS edge.  This wrapper lets Python call the ioctl correctly
 * without guessing at struct layout or ioctl numbers — we include
 * <linux/pps.h> and let the compiler sort it out.
 *
 * Build:
 *   gcc -shared -fPIC -O2 -o libppsfetch.so ppsfetch.c
 */

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/pps.h>

/*
 * pps_open — open a PPS device.
 *
 * Returns fd on success, -1 on error.
 */
int pps_open(const char *path)
{
    return open(path, O_RDONLY);
}

/*
 * pps_fetch_assert — block until next PPS assert edge.
 *
 * Parameters:
 *   fd          — file descriptor from pps_open()
 *   timeout_sec — maximum seconds to wait (0 = indefinite)
 *
 * Parameters (out):
 *   sec_out     — CLOCK_REALTIME seconds at GPIO interrupt
 *   nsec_out    — CLOCK_REALTIME nanoseconds at GPIO interrupt
 *   seq_out     — kernel PPS assert sequence number
 *
 * Returns: 0 on success, -1 on error.
 */
int pps_fetch_assert(int fd, int timeout_sec,
                     int64_t *sec_out, int32_t *nsec_out, uint32_t *seq_out)
{
    struct pps_fdata fdata;

    memset(&fdata, 0, sizeof(fdata));
    fdata.timeout.sec = timeout_sec;
    fdata.timeout.nsec = 0;
    fdata.timeout.flags = PPS_CAPTUREASSERT;

    if (ioctl(fd, PPS_FETCH, &fdata) < 0)
        return -1;

    *sec_out  = (int64_t)fdata.info.assert_tu.sec;
    *nsec_out = (int32_t)fdata.info.assert_tu.nsec;
    *seq_out  = (uint32_t)fdata.info.assert_sequence;
    return 0;
}