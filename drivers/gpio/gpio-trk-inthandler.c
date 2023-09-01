#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/sched/signal.h>

#define REG_CURRENT_TASK _IOW('a', 'a', int32_t *)
#define SET_TARGET_PID _IOW('a', 'b', int32_t)

static struct task_struct *target_task = NULL;
static int target_pid = -1; // Target application's PID

static dev_t dev = 0;
static struct class *dev_class;
static struct cdev etx_cdev;
static struct device_node *gpio_node;
static int irq_number;

static int __init etx_driver_init(void);
static void __exit etx_driver_exit(void);
static int etx_open(struct inode *inode, struct file *file);
static int etx_release(struct inode *inode, struct file *file);
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t etx_write(struct file *filp, const char __user *buf, size_t len, loff_t *off);
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .read = etx_read,
    .write = etx_write,
    .open = etx_open,
    .unlocked_ioctl = etx_ioctl,
    .release = etx_release,
};

static irqreturn_t irq_handler(int irq, void *dev_id)
{
    struct kernel_siginfo info;
    printk(KERN_INFO "GPIO Interrupt Occurred");

    memset(&info, 0, sizeof(struct kernel_siginfo));
    info.si_signo = SIGUSR1;
    info.si_code = SI_QUEUE;
    info.si_int = 1;

    if (target_task != NULL)
    {
        printk(KERN_INFO "Sending SIGUSR1 signal to PID %d\n", target_pid);
        if (send_sig_info(SIGUSR1, &info, target_task) < 0)
        {
            printk(KERN_INFO "Unable to send signal\n");
        }
    }
    else
    {
        printk(KERN_INFO "No target PID specified\n");
    }

    return IRQ_HANDLED;
}

static int etx_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "Device File Opened...!!!\n");
    return 0;
}

static int etx_release(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "Device File Closed...!!!\n");

    if (target_task != NULL)
    {
        printk(KERN_INFO "Clearing target PID\n");
        target_pid = -1;
        target_task = NULL;
    }
    return 0;
}

static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    printk(KERN_INFO "Read Function\n");
    // Your read function logic here
    return 0;
}

static ssize_t etx_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    printk(KERN_INFO "Write function\n");
    // Your write function logic here
    return 0;
}

static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    if (cmd == REG_CURRENT_TASK)
    {
        printk(KERN_INFO "REG_CURRENT_TASK\n");
        target_task = get_current();
        target_pid = task_pid_nr(target_task);
    }
    else if (cmd == SET_TARGET_PID)
    {
        printk(KERN_INFO "SET_TARGET_PID\n");
        if (copy_from_user(&target_pid, (int32_t *)arg, sizeof(int32_t)))
        {
            return -EFAULT;
        }
        target_task = pid_task(find_get_pid(target_pid), PIDTYPE_PID);
    }
    return 0;
}

static int __init etx_driver_init(void)
{
    int ret;

    if ((ret = alloc_chrdev_region(&dev, 0, 1, "etx_Dev")) < 0)
    {
        printk(KERN_INFO "Cannot allocate major number\n");
        return ret;
    }
    printk(KERN_INFO "Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));

    cdev_init(&etx_cdev, &fops);

    if ((ret = cdev_add(&etx_cdev, dev, 1)) < 0)
    {
        printk(KERN_INFO "Cannot add the device to the system\n");
        goto r_class;
    }

    if (IS_ERR(dev_class = class_create(THIS_MODULE, "etx_class")))
    {
        printk(KERN_INFO "Cannot create the struct class\n");
        ret = PTR_ERR(dev_class);
        goto r_class;
    }

    if (IS_ERR(device_create(dev_class, NULL, dev, NULL, "etx_device")))
    {
        printk(KERN_INFO "Cannot create the Device\n");
        ret = PTR_ERR(device_create);
        goto r_device;
    }

    gpio_node = of_find_node_by_name(NULL, "gpio_interrupt");
    if (!gpio_node)
    {
        printk(KERN_INFO "GPIO node not found\n");
        ret = -ENODEV;
        goto r_irq;
    }

    irq_number = irq_of_parse_and_map(gpio_node, 0);
    if (!irq_number)
    {
        printk(KERN_INFO "Failed to parse and map GPIO interrupt\n");
        ret = -ENXIO;
        goto r_irq;
    }

    if (request_irq(irq_number, irq_handler, IRQF_SHARED, "etx_device", (void *)(irq_handler)))
    {
        printk(KERN_INFO "Unable to register GPIO interrupt\n");
        ret = -EBUSY;
        goto r_irq;
    }

    printk(KERN_INFO "Device Driver Insert...Done!!!\n");
    return 0;

r_irq:
    device_destroy(dev_class, dev);
r_device:
    class_destroy(dev_class);
r_class:
    unregister_chrdev_region(dev, 1);
    return ret;
}

static void __exit etx_driver_exit(void)
{
    free_irq(irq_number, (void *)(irq_handler));
    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&etx_cdev);
    unregister_chrdev_region(dev, 1);
    printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}

module_init(etx_driver_init);
module_exit(etx_driver_exit);

MODULE_LICENSE("Closed");
MODULE_AUTHOR("Trackonmy");
MODULE_DESCRIPTION("GPIO Interrupt Driver with Signal to Specific PID");
MODULE_VERSION("1.0");

