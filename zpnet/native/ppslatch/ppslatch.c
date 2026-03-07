/*
 * ppslatch.c — Sacred PPS Latch on GPIO25
 *
 * Purpose:
 *   Latch CNTVCT_EL0 at the sacred PPS rising edge on GPIO25 and make
 *   the capture available to userspace via a lock-free ring buffer.
 *
 * Design:
 *   • Hardware edge detection via BCM2711 GPEDS0 / GPREN0
 *   • Tight burst polling on the latched edge bit
 *   • Periodic sched_yield() to keep the kernel watchdog alive
 *   • Read CNTVCT_EL0 immediately when the edge is observed
 *   • Maintain a continuously updated shadow CNTVCT value while waiting
 *   • Store captured/shadow/delta in a ring for later drain by PITIMER
 *
 * Why yield?
 *   A bare spin loop with no syscalls starves the kernel's RCU grace
 *   period machinery.  After ~21 seconds the kernel panics (softlockup
 *   or RCU stall, depending on config).  Yielding every N poll
 *   iterations provides proof-of-life to the scheduler while keeping
 *   worst-case latch latency well under 1 µs.
 *
 * Notes:
 *   • This is NOT a kernel ISR.  It is a focused native latch engine.
 *   • Rising edge on GPIO25 is the sacred PPS boundary.
 *   • The shadow count is diagnostic only.
 *
 * Build:
 *   gcc -shared -fPIC -O2 -o libppslatch.so ppslatch.c
 */

#define _GNU_SOURCE
#include <stdint.h>
#include <stdatomic.h>
#include <fcntl.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

/* ================================================================
 * Yield tuning
 * ================================================================
 *
 * YIELD_INTERVAL controls how many tight poll iterations we execute
 * before calling sched_yield().  Each poll iteration is roughly
 * 20-50 ns (ISB + MRS + volatile load + branch), so:
 *
 *   2048 iterations × ~30 ns ≈ ~60 µs between yields
 *
 * sched_yield() itself costs ~100-300 ns when the core has no other
 * runnable tasks, so worst-case latch latency is:
 *
 *   poll loop body time + yield overhead ≈ ~350 ns worst case
 *
 * This is comfortably sub-microsecond while giving the kernel's
 * RCU and watchdog subsystems the heartbeat they need.
 *
 * For a dedicated isolated core (isolcpus= + SCHED_FIFO), you can
 * safely increase this to 65536 or even disable yielding entirely
 * by setting YIELD_INTERVAL to 0 at runtime via ppslatch_set_yield().
 */

#define DEFAULT_YIELD_INTERVAL  2048

static _Atomic uint32_t yield_interval = DEFAULT_YIELD_INTERVAL;

/* ================================================================
 * Public ring buffer interface
 * ================================================================ */

#define PPSLATCH_RING_SIZE   4096
#define PPSLATCH_RING_MASK   (PPSLATCH_RING_SIZE - 1)

#define PPSLATCH_FLAG_NONE   0u

typedef struct {
    uint64_t captured_cntvct;
    uint64_t shadow_cntvct;
    uint64_t delta_ticks;
    uint64_t edge_seq;
    uint64_t shadow_seq;
    uint32_t flags;
} ppslatch_entry_t;

typedef struct {
    _Alignas(64) atomic_uint_fast64_t head;
    _Alignas(64) atomic_uint_fast64_t tail;
    _Alignas(64) atomic_uint_fast64_t edges_total;
    _Alignas(64) atomic_uint_fast64_t overflows;
    _Alignas(64) atomic_uint_fast64_t last_shadow_cntvct;
    _Alignas(64) atomic_uint_fast64_t last_shadow_seq;
    _Alignas(64) atomic_int           running;
    _Alignas(64) atomic_int           stop;

    ppslatch_entry_t entries[PPSLATCH_RING_SIZE];
} ppslatch_ring_t;

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
 * BCM2711 GPIO — hardware rising-edge detection on GPIO25
 * ================================================================ */

#define GPIO_BLOCK_SIZE      0x1000
#define GPEDS0_OFFSET        0x40
#define GPREN0_OFFSET        0x4C
#define GPIO_PIN             25
#define GPIO_MASK            (1u << GPIO_PIN)

static volatile uint32_t *gpio_base = 0;
static volatile uint32_t *gpeds0 = 0;
static volatile uint32_t *gpren0 = 0;

static int gpio_init(void)
{
    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0)
        return -1;

    void *map = mmap(0, GPIO_BLOCK_SIZE,
                     PROT_READ | PROT_WRITE, MAP_SHARED,
                     fd, 0);
    close(fd);

    if (map == MAP_FAILED)
        return -1;

    gpio_base = (volatile uint32_t *)map;
    gpeds0 = (volatile uint32_t *)((char *)map + GPEDS0_OFFSET);
    gpren0 = (volatile uint32_t *)((char *)map + GPREN0_OFFSET);

    /* Enable rising edge detection on GPIO25. */
    *gpren0 |= GPIO_MASK;

    /* Clear any stale pending event before starting. */
    *gpeds0 = GPIO_MASK;

    return 0;
}

static void gpio_cleanup(void)
{
    if (gpren0) {
        *gpren0 &= ~GPIO_MASK;
    }
    if (gpeds0) {
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
 * Main latch loop — with periodic yield
 * ================================================================ */

int ppslatch_run(ppslatch_ring_t *ring)
{
    if (!ring)
        return -1;

    if (gpio_init() != 0)
        return -1;

    atomic_store(&ring->head, 0);
    atomic_store(&ring->tail, 0);
    atomic_store(&ring->edges_total, 0);
    atomic_store(&ring->overflows, 0);
    atomic_store(&ring->last_shadow_cntvct, 0);
    atomic_store(&ring->last_shadow_seq, 0);
    atomic_store(&ring->stop, 0);
    atomic_store(&ring->running, 1);

    uint64_t edge_seq = 0;
    uint64_t shadow_seq = 0;

    for (;;) {
        if (atomic_load_explicit(&ring->stop, memory_order_relaxed))
            break;

        uint64_t shadow_cntvct = 0;
        uint32_t spin_count = 0;
        uint32_t yi = atomic_load_explicit(&yield_interval, memory_order_relaxed);

        while (!gpio_edge_pending()) {
            if (atomic_load_explicit(&ring->stop, memory_order_relaxed))
                goto done;

            shadow_cntvct = read_cntvct();
            shadow_seq++;
            atomic_store_explicit(&ring->last_shadow_cntvct, shadow_cntvct, memory_order_relaxed);
            atomic_store_explicit(&ring->last_shadow_seq, shadow_seq, memory_order_relaxed);

            /*
             * Periodic yield: give the kernel a scheduling opportunity
             * so RCU grace periods can advance and the watchdog stays
             * quiet.  When yi == 0, yielding is disabled (for use on
             * an isolated core with SCHED_FIFO).
             */
            if (yi && ++spin_count >= yi) {
                sched_yield();
                spin_count = 0;
            }
        }

        /* Sacred capture: read CNTVCT immediately after edge is observed. */
        uint64_t captured_cntvct = read_cntvct();

        /* Clear latched event for the next edge. */
        gpio_edge_clear();

        uint64_t delta_ticks = captured_cntvct - shadow_cntvct;

        uint64_t head = atomic_load_explicit(&ring->head, memory_order_relaxed);
        uint64_t tail = atomic_load_explicit(&ring->tail, memory_order_acquire);

        if (head - tail >= PPSLATCH_RING_SIZE) {
            atomic_fetch_add(&ring->overflows, 1);
        } else {
            uint64_t idx = head & PPSLATCH_RING_MASK;
            ring->entries[idx].captured_cntvct = captured_cntvct;
            ring->entries[idx].shadow_cntvct   = shadow_cntvct;
            ring->entries[idx].delta_ticks     = delta_ticks;
            ring->entries[idx].edge_seq        = edge_seq;
            ring->entries[idx].shadow_seq      = shadow_seq;
            ring->entries[idx].flags           = PPSLATCH_FLAG_NONE;

            atomic_store_explicit(&ring->head, head + 1, memory_order_release);
        }

        edge_seq++;
        atomic_store(&ring->edges_total, edge_seq);
    }

done:
    gpio_cleanup();
    atomic_store(&ring->running, 0);
    return 0;
}

void ppslatch_stop(ppslatch_ring_t *ring)
{
    if (ring)
        atomic_store(&ring->stop, 1);
}

void ppslatch_cleanup(void)
{
    gpio_cleanup();
}

/* ================================================================
 * Yield tuning — runtime control
 * ================================================================ */

void ppslatch_set_yield(uint32_t interval)
{
    atomic_store(&yield_interval, interval);
}

uint32_t ppslatch_get_yield(void)
{
    return atomic_load(&yield_interval);
}

/* ================================================================
 * Bulk drain
 * ================================================================ */

uint32_t ppslatch_drain(ppslatch_ring_t *ring, ppslatch_entry_t *out,
                        uint32_t max_entries)
{
    uint32_t count = 0;
    uint64_t head = atomic_load_explicit(&ring->head, memory_order_acquire);
    uint64_t tail = atomic_load_explicit(&ring->tail, memory_order_relaxed);

    while (tail < head && count < max_entries) {
        out[count] = ring->entries[tail & PPSLATCH_RING_MASK];
        count++;
        tail++;
    }

    atomic_store_explicit(&ring->tail, tail, memory_order_release);
    return count;
}

/* ================================================================
 * Convenience
 * ================================================================ */

uint32_t ppslatch_ring_size(void)
{
    return PPSLATCH_RING_SIZE;
}

uint64_t ppslatch_entry_size(void)
{
    return sizeof(ppslatch_entry_t);
}

uint64_t ppslatch_ring_struct_size(void)
{
    return sizeof(ppslatch_ring_t);
}