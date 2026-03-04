/*
 * pulsecapture.c — 10 KHz GNSS Edge Capture via Hardware Edge Detection
 *
 * v5: Per-edge ring write + cleanup to prevent Pi freeze.
 *
 *   v4 used batched captures which introduced a flush gap where
 *   edges could coalesce in GPEDS0.  v5 writes each edge to the
 *   ring immediately after capture — the cost is ~50 ns per edge,
 *   negligible against the 100 µs inter-edge period.
 *
 *   v5 also adds pulsecapture_cleanup() which disables edge
 *   detection in GPREN0 and clears pending events in GPEDS0.
 *   Without this, exiting the program leaves hardware edge
 *   detection active with no consumer, which can freeze the Pi.
 *
 *   The capture loop checks a 'stop' flag on each edge so it
 *   can exit cleanly for cleanup.
 *
 * Build:
 *   gcc -shared -fPIC -O2 -o libpulsecapture.so pulsecapture.c
 */

#define _GNU_SOURCE
#include <stdint.h>
#include <stdatomic.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

/* ================================================================
 * Public ring buffer interface
 * ================================================================ */

#define PULSE_RING_SIZE     32768
#define PULSE_RING_MASK     (PULSE_RING_SIZE - 1)

#define PPS_CHECK_INTERVAL  1000

#define PULSE_FLAG_NONE     0
#define PULSE_FLAG_PPS      1

typedef struct {
    uint64_t cntvct;
    uint64_t edge_seq;
    uint32_t flags;
    int64_t  pps_rt_sec;
    int64_t  pps_rt_nsec;
} pulse_entry_t;

typedef struct {
    _Alignas(64) atomic_uint_fast64_t head;
    _Alignas(64) atomic_uint_fast64_t tail;
    _Alignas(64) atomic_uint_fast64_t edges_total;
    _Alignas(64) atomic_uint_fast64_t pps_total;
    _Alignas(64) atomic_uint_fast64_t overflows;
    _Alignas(64) atomic_int           running;
    _Alignas(64) atomic_int           stop;       /* Set to 1 to request clean exit */

    pulse_entry_t entries[PULSE_RING_SIZE];
} pulse_ring_t;

/* ================================================================
 * ARM Generic Timer
 * ================================================================ */

static inline uint64_t read_cntvct(void)
{
    uint64_t val;
    __asm__ __volatile__(
        "isb\n\t"
        "mrs %0, cntvct_el0"
        : "=r"(val)
        :
        : "memory"
    );
    return val;
}

/* ================================================================
 * BCM2711 GPIO — hardware edge detection
 * ================================================================ */

#define GPIO_BLOCK_SIZE     0x1000
#define GPEDS0_OFFSET       0x40
#define GPREN0_OFFSET       0x4C
#define GPIO_PIN            25
#define GPIO_MASK           (1u << GPIO_PIN)

static volatile uint32_t *gpio_base = NULL;
static volatile uint32_t *gpeds0 = NULL;
static volatile uint32_t *gpren0 = NULL;

static int gpio_init(void)
{
    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0)
        return -1;

    void *map = mmap(NULL, GPIO_BLOCK_SIZE,
                     PROT_READ | PROT_WRITE, MAP_SHARED,
                     fd, 0);
    close(fd);

    if (map == MAP_FAILED)
        return -1;

    gpio_base = (volatile uint32_t *)map;
    gpeds0 = (volatile uint32_t *)((char *)map + GPEDS0_OFFSET);
    gpren0 = (volatile uint32_t *)((char *)map + GPREN0_OFFSET);

    /* Enable rising edge detection on GPIO 25 */
    *gpren0 |= GPIO_MASK;

    /* Clear any pending event */
    *gpeds0 = GPIO_MASK;

    return 0;
}

/*
 * gpio_cleanup — disable edge detection and clear pending events.
 *
 * MUST be called before exit to prevent the Pi from freezing.
 * When GPREN0 is left enabled with no consumer clearing GPEDS0,
 * the kernel's GPIO interrupt handler can enter a storm.
 */
static void gpio_cleanup(void)
{
    if (gpren0) {
        /* Disable rising edge detection on GPIO 25 */
        *gpren0 &= ~GPIO_MASK;
    }
    if (gpeds0) {
        /* Clear any pending event */
        *gpeds0 = GPIO_MASK;
    }
}

static inline int gpio_edge_pending(void)
{
    return (*gpeds0 & GPIO_MASK) ? 1 : 0;
}

static inline void gpio_edge_clear(void)
{
    *gpeds0 = GPIO_MASK;
}

/* ================================================================
 * Main capture loop
 * ================================================================ */

/*
 * pulsecapture_run — spin on hardware edge detection, capture
 * CNTVCT_EL0 on every rising edge, write to ring immediately.
 *
 * Returns:
 *   -1 if GPIO init fails
 *    0 if stopped cleanly via the stop flag
 */
int pulsecapture_run(pulse_ring_t *ring)
{
    if (!ring)
        return -1;

    if (gpio_init() != 0)
        return -1;

    /* Initialize ring state */
    atomic_store(&ring->head, 0);
    atomic_store(&ring->tail, 0);
    atomic_store(&ring->edges_total, 0);
    atomic_store(&ring->pps_total, 0);
    atomic_store(&ring->overflows, 0);
    atomic_store(&ring->stop, 0);
    atomic_store(&ring->running, 1);

    uint64_t edge_seq = 0;
    uint32_t pps_countdown = PPS_CHECK_INTERVAL;

    /* PPS tracking */
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    time_t last_sec = ts.tv_sec;

    for (;;) {

        /* Check stop flag */
        if (atomic_load_explicit(&ring->stop, memory_order_relaxed))
            break;

        /* Spin until hardware latches a rising edge */
        while (!gpio_edge_pending()) {
            /* Check stop flag periodically during spin */
            if (atomic_load_explicit(&ring->stop, memory_order_relaxed))
                goto done;
        }

        /* Capture CNTVCT_EL0 immediately */
        uint64_t cntvct = read_cntvct();

        /* Clear the event for next edge */
        gpio_edge_clear();

        /* PPS check — throttled */
        uint32_t flags = PULSE_FLAG_NONE;
        int64_t pps_sec = 0;
        int64_t pps_nsec = 0;

        if (--pps_countdown == 0) {
            pps_countdown = PPS_CHECK_INTERVAL;
            clock_gettime(CLOCK_REALTIME, &ts);

            if (ts.tv_sec != last_sec) {
                flags = PULSE_FLAG_PPS;
                pps_sec = (int64_t)ts.tv_sec;
                pps_nsec = (int64_t)ts.tv_nsec;
                last_sec = ts.tv_sec;
                atomic_fetch_add(&ring->pps_total, 1);
            }
        }

        /* Write directly to ring */
        uint64_t head = atomic_load_explicit(&ring->head, memory_order_relaxed);
        uint64_t tail = atomic_load_explicit(&ring->tail, memory_order_acquire);

        if (head - tail >= PULSE_RING_SIZE) {
            atomic_fetch_add(&ring->overflows, 1);
        } else {
            uint64_t idx = head & PULSE_RING_MASK;
            ring->entries[idx].cntvct      = cntvct;
            ring->entries[idx].edge_seq    = edge_seq;
            ring->entries[idx].flags       = flags;
            ring->entries[idx].pps_rt_sec  = pps_sec;
            ring->entries[idx].pps_rt_nsec = pps_nsec;

            atomic_store_explicit(&ring->head, head + 1, memory_order_release);
        }

        edge_seq++;
        atomic_store(&ring->edges_total, edge_seq);
    }

done:
    /* Clean up GPIO hardware state before exit */
    gpio_cleanup();
    atomic_store(&ring->running, 0);
    return 0;
}

/*
 * pulsecapture_stop — request clean shutdown of the capture loop.
 *
 * Sets the stop flag.  The capture loop will exit within one
 * edge period (~100 µs) and call gpio_cleanup() automatically.
 */
void pulsecapture_stop(pulse_ring_t *ring)
{
    if (ring)
        atomic_store(&ring->stop, 1);
}

/*
 * pulsecapture_cleanup — emergency cleanup if the loop didn't
 * exit cleanly (e.g. process killed).  Safe to call multiple times.
 */
void pulsecapture_cleanup(void)
{
    gpio_cleanup();
}

/* ================================================================
 * Bulk drain
 * ================================================================ */

uint32_t pulsecapture_drain(pulse_ring_t *ring, pulse_entry_t *out,
                            uint32_t max_entries)
{
    uint32_t count = 0;
    uint64_t head = atomic_load_explicit(&ring->head, memory_order_acquire);
    uint64_t tail = atomic_load_explicit(&ring->tail, memory_order_relaxed);

    while (tail < head && count < max_entries) {
        out[count] = ring->entries[tail & PULSE_RING_MASK];
        count++;
        tail++;
    }

    atomic_store_explicit(&ring->tail, tail, memory_order_release);
    return count;
}

/* ================================================================
 * Convenience
 * ================================================================ */

uint32_t pulsecapture_ring_size(void)
{
    return PULSE_RING_SIZE;
}

uint64_t pulsecapture_entry_size(void)
{
    return sizeof(pulse_entry_t);
}

uint64_t pulsecapture_ring_struct_size(void)
{
    return sizeof(pulse_ring_t);
}