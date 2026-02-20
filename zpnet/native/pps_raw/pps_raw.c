/*
 * pps_raw — Raw ARM arch timer capture at PPS GPIO edge
 *
 * This kernel module captures the raw CNTVCT_EL0 counter value at
 * each PPS pulse edge via GPIO interrupt.  Unlike the standard
 * pps-gpio driver which converts through the kernel timekeeping
 * layer (and thus includes chrony/NTP rate adjustments), this
 * module stores the unmodified hardware counter value.
 *
 * The captured values are exposed via /dev/pps_raw as a simple
 * character device.  A blocking read returns the next PPS capture
 * as a binary struct:
 *
 *   struct pps_raw_capture {
 *       uint64_t counter;    // raw CNTVCT_EL0 ticks
 *       uint32_t sequence;   // monotonic sequence number
 *       uint32_t missed;     // overrun count (captures missed by reader)
 *   };
 *
 * Multiple readers are supported.  Each reader maintains its own
 * "last seen sequence" so it gets every capture independently.
 *
 * GPIO strategy:
 *   On kernel 6.12+, the legacy gpio_request/gpio_free and the
 *   intermediate gpiod_request/gpiod_free APIs are gone.  The
 *   "proper" approach (gpiod_get with a platform device parent)
 *   is heavyweight for a simple PPS capture module.
 *
 *   Instead, we use gpio_to_desc() to get the descriptor, then
 *   gpiod_direction_input() + gpiod_to_irq() to obtain the IRQ
 *   number, and request_irq() to own the interrupt.  We don't
 *   formally "claim" the GPIO — we only claim the IRQ, which is
 *   sufficient since our sole purpose is to timestamp edges.
 *
 * Build:
 *   make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
 *
 * Load:
 *   sudo insmod pps_raw.ko gpio_pin=530
 *   (530 = Linux GPIO for BCM18 on Pi 4/5, base 512 + offset 18)
 *
 * Usage from Python:
 *   fd = os.open("/dev/pps_raw", os.O_RDONLY)
 *   data = os.read(fd, 16)  # blocks until next PPS
 *   counter, sequence, missed = struct.unpack("=QII", data)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/arch_timer.h>

#define DEVICE_NAME "pps_raw"
#define CLASS_NAME  "pps_raw"

/* Module parameters */
static int gpio_pin = 530;
module_param(gpio_pin, int, 0444);
MODULE_PARM_DESC(gpio_pin, "Linux GPIO number for PPS input (default: 530 = BCM18 on Pi 4/5, base 512 + offset 18)");

static int active_edge = 1;  /* 1 = rising, 0 = falling */
module_param(active_edge, int, 0444);
MODULE_PARM_DESC(active_edge, "Active edge: 1=rising (default), 0=falling");

/* Capture structure — matches what userspace reads */
struct pps_raw_capture {
    u64 counter;      /* raw CNTVCT_EL0 */
    u32 sequence;     /* monotonic, starts at 1 */
    u32 missed;       /* placeholder, always 0 for now */
};

/* Global state — updated in IRQ context */
static struct pps_raw_capture latest_capture;
static u32 capture_sequence;   /* monotonic, incremented in IRQ */
static DEFINE_SPINLOCK(capture_lock);
static DECLARE_WAIT_QUEUE_HEAD(capture_wq);

/* Per-reader state */
struct pps_raw_reader {
    u32 last_seen;  /* sequence number last returned to this reader */
};

/* Character device plumbing */
static dev_t dev_num;
static struct cdev pps_cdev;
static struct class *pps_class;
static struct device *pps_device;

/* GPIO / IRQ state */
static struct gpio_desc *pps_gpio_desc;
static int irq_number;

/*
 * IRQ handler — the critical path.
 *
 * Reads CNTVCT_EL0 as the very first operation, before any other
 * work.  This is a single MRS instruction on ARM64 — the minimum
 * possible latency after the interrupt fires.
 *
 * The GPIO interrupt on the Pi fires with ~1µs latency from the
 * physical edge.  This is orders of magnitude better than anything
 * achievable from userspace.
 */
static irqreturn_t pps_raw_irq_handler(int irq, void *dev_id)
{
    u64 now = __arch_counter_get_cntvct();
    unsigned long flags;

    spin_lock_irqsave(&capture_lock, flags);
    capture_sequence++;
    latest_capture.counter = now;
    latest_capture.sequence = capture_sequence;
    latest_capture.missed = 0;
    spin_unlock_irqrestore(&capture_lock, flags);

    wake_up_interruptible(&capture_wq);

    return IRQ_HANDLED;
}

/*
 * File operations
 */
static int pps_raw_open(struct inode *inode, struct file *filp)
{
    struct pps_raw_reader *reader;

    reader = kzalloc(sizeof(*reader), GFP_KERNEL);
    if (!reader)
        return -ENOMEM;

    reader->last_seen = capture_sequence;
    filp->private_data = reader;

    return 0;
}

static int pps_raw_release(struct inode *inode, struct file *filp)
{
    kfree(filp->private_data);
    return 0;
}

/*
 * Blocking read — returns the next PPS capture.
 *
 * If a new capture is already available (sequence advanced past
 * this reader's last_seen), it returns immediately.  Otherwise
 * it sleeps until the next IRQ fires.
 *
 * Returns exactly sizeof(struct pps_raw_capture) bytes on success.
 */
static ssize_t pps_raw_read(struct file *filp, char __user *buf,
                            size_t count, loff_t *ppos)
{
    struct pps_raw_reader *reader = filp->private_data;
    struct pps_raw_capture cap;
    unsigned long flags;
    int ret;

    if (count < sizeof(struct pps_raw_capture))
        return -EINVAL;

    /* Wait for a new capture */
    if (filp->f_flags & O_NONBLOCK) {
        spin_lock_irqsave(&capture_lock, flags);
        if (capture_sequence == reader->last_seen) {
            spin_unlock_irqrestore(&capture_lock, flags);
            return -EAGAIN;
        }
        cap = latest_capture;
        spin_unlock_irqrestore(&capture_lock, flags);
    } else {
        ret = wait_event_interruptible(capture_wq,
                capture_sequence != reader->last_seen);
        if (ret)
            return ret;

        spin_lock_irqsave(&capture_lock, flags);
        cap = latest_capture;
        spin_unlock_irqrestore(&capture_lock, flags);
    }

    /* Track how many captures this reader missed */
    if (reader->last_seen > 0 && cap.sequence > reader->last_seen + 1)
        cap.missed = cap.sequence - reader->last_seen - 1;

    reader->last_seen = cap.sequence;

    if (copy_to_user(buf, &cap, sizeof(cap)))
        return -EFAULT;

    return sizeof(cap);
}

static __poll_t pps_raw_poll(struct file *filp,
                             struct poll_table_struct *wait)
{
    struct pps_raw_reader *reader = filp->private_data;
    __poll_t mask = 0;

    poll_wait(filp, &capture_wq, wait);

    if (capture_sequence != reader->last_seen)
        mask |= EPOLLIN | EPOLLRDNORM;

    return mask;
}

static const struct file_operations pps_raw_fops = {
    .owner   = THIS_MODULE,
    .open    = pps_raw_open,
    .release = pps_raw_release,
    .read    = pps_raw_read,
    .poll    = pps_raw_poll,
};

/*
 * Module init/exit
 */
static int __init pps_raw_init(void)
{
    int ret;
    unsigned long irq_flags;

    pr_info(DEVICE_NAME ": initializing (gpio=%d, edge=%s)\n",
            gpio_pin, active_edge ? "rising" : "falling");

    /*
     * Get GPIO descriptor from pin number.
     * We do NOT formally claim the GPIO — we only need the IRQ.
     * On kernel 6.12+ the legacy gpio_request() and intermediate
     * gpiod_request() APIs are removed; the full gpiod_get()
     * requires a platform device parent.  Since we only need to
     * timestamp edges, claiming just the IRQ is sufficient.
     */
    pps_gpio_desc = gpio_to_desc(gpio_pin);
    if (!pps_gpio_desc) {
        pr_err(DEVICE_NAME ": no descriptor for GPIO %d\n", gpio_pin);
        return -EINVAL;
    }

    /* Ensure pin is configured as input */
    ret = gpiod_direction_input(pps_gpio_desc);
    if (ret) {
        pr_err(DEVICE_NAME ": failed to set GPIO %d as input: %d\n",
               gpio_pin, ret);
        return ret;
    }

    /* Map GPIO to IRQ number */
    irq_number = gpiod_to_irq(pps_gpio_desc);
    if (irq_number < 0) {
        pr_err(DEVICE_NAME ": failed to get IRQ for GPIO %d: %d\n",
               gpio_pin, irq_number);
        return irq_number;
    }

    /* Request IRQ — this is our only kernel resource claim */
    irq_flags = active_edge ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
    ret = request_irq(irq_number, pps_raw_irq_handler,
                      irq_flags, DEVICE_NAME, NULL);
    if (ret) {
        pr_err(DEVICE_NAME ": failed to request IRQ %d: %d\n",
               irq_number, ret);
        return ret;
    }

    /* Allocate character device region */
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret) {
        pr_err(DEVICE_NAME ": failed to allocate chrdev region: %d\n", ret);
        goto err_irq;
    }

    /* Initialize and add cdev */
    cdev_init(&pps_cdev, &pps_raw_fops);
    pps_cdev.owner = THIS_MODULE;

    ret = cdev_add(&pps_cdev, dev_num, 1);
    if (ret) {
        pr_err(DEVICE_NAME ": failed to add cdev: %d\n", ret);
        goto err_chrdev;
    }

    /* Create device class and device node */
    pps_class = class_create(CLASS_NAME);
    if (IS_ERR(pps_class)) {
        ret = PTR_ERR(pps_class);
        pr_err(DEVICE_NAME ": failed to create class: %d\n", ret);
        goto err_cdev;
    }

    pps_device = device_create(pps_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(pps_device)) {
        ret = PTR_ERR(pps_device);
        pr_err(DEVICE_NAME ": failed to create device: %d\n", ret);
        goto err_class;
    }

    pr_info(DEVICE_NAME ": ready — /dev/%s (GPIO %d, IRQ %d)\n",
            DEVICE_NAME, gpio_pin, irq_number);

    return 0;

err_class:
    class_destroy(pps_class);
err_cdev:
    cdev_del(&pps_cdev);
err_chrdev:
    unregister_chrdev_region(dev_num, 1);
err_irq:
    free_irq(irq_number, NULL);
    return ret;
}

static void __exit pps_raw_exit(void)
{
    device_destroy(pps_class, dev_num);
    class_destroy(pps_class);
    cdev_del(&pps_cdev);
    unregister_chrdev_region(dev_num, 1);
    free_irq(irq_number, NULL);

    pr_info(DEVICE_NAME ": removed\n");
}

module_init(pps_raw_init);
module_exit(pps_raw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ZPNet");
MODULE_DESCRIPTION("Raw ARM arch timer capture at PPS GPIO edge");
MODULE_VERSION("1.0");