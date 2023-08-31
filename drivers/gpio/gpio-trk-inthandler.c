#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/device.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("GPIO Interrupt Driver");
MODULE_VERSION("0.1");

#define NUM_GPIO_PINS 5

static struct platform_device *pdev_global;
static struct gpio_desc *gpio_descs[NUM_GPIO_PINS];
static unsigned int irq_num[NUM_GPIO_PINS];
static unsigned int interrupt_count[NUM_GPIO_PINS] = {0};

static irqreturn_t gpio_irq_handler(int irq, void *dev_id) {
    unsigned int pin_index = (unsigned int)dev_id;
    interrupt_count[pin_index]++;
    return IRQ_HANDLED;
}

static ssize_t interrupt_count_show(struct device *dev, struct device_attribute *attr, char *buf) {
    int pin_index = to_platform_device(dev)->id;
    return sprintf(buf, "%u\n", interrupt_count[pin_index]);
}

static DEVICE_ATTR_RO(interrupt_count);

static int gpio_interrupt_probe(struct platform_device *pdev) {
    struct device_node *np;
    int ret, i;

    pdev_global = pdev;

    np = pdev->dev.of_node;
    if (!np) {
        dev_err(&pdev->dev, "Device tree node not found\n");
        return -ENODEV;
    }

    // List of GPIO names to be fetched from the device tree
    const char *gpio_names[NUM_GPIO_PINS] = {
        "environment", "pressure", "temperature", "gps", "mcu"
    };

    for (i = 0; i < NUM_GPIO_PINS; i++) {
        struct gpio_desc *desc;

        desc = devm_gpiod_get_optional(&pdev->dev, gpio_names[i], GPIOD_IN);
        if (!desc) {
            dev_err(&pdev->dev, "Failed to get GPIO pin %s\n", gpio_names[i]);
            continue;
        }

        gpio_descs[i] = desc;

        irq_num[i] = gpiod_to_irq(gpio_descs[i]);
        ret = request_irq(irq_num[i], gpio_irq_handler, IRQF_TRIGGER_RISING, "gpio_interrupt", (void *)i);
        if (ret) {
            dev_err(&pdev->dev, "Failed to request IRQ for GPIO pin %s\n", gpio_names[i]);
            continue;
        }

        ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_interrupt_count.attr);
        if (ret) {
            dev_err(&pdev->dev, "Failed to create sysfs entry for GPIO pin %s\n", gpio_names[i]);
            free_irq(irq_num[i], (void *)i);
            continue;
        }
    }

    return 0;
}

static int gpio_interrupt_remove(struct platform_device *pdev) {
    int i;

    for (i = 0; i < NUM_GPIO_PINS; i++) {
        free_irq(irq_num[i], (void *)i);
    }

    return 0;
}

static const struct of_device_id gpio_interrupt_of_match[] = {
    { .compatible = "gpio-trk-inthandler", },
    {},
};
MODULE_DEVICE_TABLE(of, gpio_interrupt_of_match);

static struct platform_driver gpio_interrupt_driver = {
    .driver = {
        .name = "gpio-trk-inthandler",
        .of_match_table = gpio_interrupt_of_match,
    },
    .probe = gpio_interrupt_probe,
    .remove = gpio_interrupt_remove,
};

module_platform_driver(gpio_interrupt_driver);
