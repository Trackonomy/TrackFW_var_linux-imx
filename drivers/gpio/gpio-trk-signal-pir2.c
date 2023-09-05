
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include<linux/slab.h>
#include<linux/uaccess.h>
#include <linux/gpio.h>     //GPIO
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/err.h>
#define SEND_SIGNAL SIGRTMIN+6
 
#define REG_CURRENT_TASK _IOW('a','a',int32_t*)
 
#define IRQ_NO 236

#define GPIO_IN (102)

unsigned int GPIO_irqNumber;
 
/* Signaling to Application */
static struct task_struct *task = NULL;
static int signum = 0;
 
int32_t value = 0;
 
dev_t dev = 0;
static struct class *dev_class;
static struct cdev etx_cdev;
 unsigned int GPIO_irqNumber;
 
static int __init etx_driver_init(void);
static void __exit etx_driver_exit(void);
static int etx_open(struct inode *inode, struct file *file);
static int etx_release(struct inode *inode, struct file *file);
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len,loff_t * off);
static ssize_t etx_write(struct file *filp, const char *buf, size_t len, loff_t * off);
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
 
static struct file_operations fops =
{
        .owner          = THIS_MODULE,
        .read           = etx_read,
        .write          = etx_write,
        .open           = etx_open,
        .unlocked_ioctl = etx_ioctl,
        .release        = etx_release,
};
 
//Interrupt handler for IRQ 11. 
static irqreturn_t irq_handler(int irq,void *dev_id) {
    struct kernel_siginfo info;
    printk(KERN_INFO "GPIO Interrupt Occurred\n");
    
    //Sending signal to app
    memset(&info, 0, sizeof(struct kernel_siginfo));
    info.si_signo = SEND_SIGNAL;
    info.si_code = SI_QUEUE;
    info.si_int = 1;
 
    if (task != NULL) {
        printk(KERN_INFO "Sending signal to app\n");
        if(send_sig_info(SEND_SIGNAL, &info, task) < 0) {
            printk(KERN_INFO "Unable to send signal\n");
        }
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
    struct task_struct *ref_task = get_current();
    printk(KERN_INFO "Device File Closed...!!!\n");
    
    //delete the task
    if(ref_task == task) {
        task = NULL;
    }
    return 0;
}
 
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    printk(KERN_INFO "Read Function\n");  
    // asm volatile (
    //     "mov w0, %w0\n"    // Load the interrupt number into w0 register
    //     "svc #0\n"         // Generate a software interrupt (svc instruction)
    //     :: "r" (11)        // Input operand: the interrupt number
    //     : "w0", "memory"   // Clobbered registers and memory
    // );
    return 0;
}
static ssize_t etx_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    printk(KERN_INFO "Write function\n");
    return 0;
}
 
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    if (cmd == REG_CURRENT_TASK) {
        printk(KERN_INFO "REG_CURRENT_TASK\n");
        task = get_current();
        signum = SEND_SIGNAL;
    }
    return 0;
}
 
 
static int __init etx_driver_init(void)
{
    /*Allocating Major number*/
    if((alloc_chrdev_region(&dev, 0, 1, "pir2_Dev")) <0){
            printk(KERN_INFO "Cannot allocate major number\n");
            return -1;
    }
    printk(KERN_INFO "Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
 
    /*Creating cdev structure*/
    cdev_init(&etx_cdev,&fops);
 
    /*Adding character device to the system*/
    if((cdev_add(&etx_cdev,dev,1)) < 0){
        printk(KERN_INFO "Cannot add the device to the system\n");
        goto r_class;
    }
 
    /*Creating struct class*/
    if(IS_ERR(dev_class = class_create(THIS_MODULE,"etx_class"))){
        printk(KERN_INFO "Cannot create the struct class\n");
        goto r_class;
    }
 
    /*Creating device*/
    if(IS_ERR(device_create(dev_class,NULL,dev,NULL,"pir2_device"))){
        printk(KERN_INFO "Cannot create the Device 1\n");
        goto r_device;
    }
 


  if(gpio_is_valid(GPIO_IN) == false){
    pr_err("GPIO %d is not valid\n", GPIO_IN);
    goto r_gpio_in;
  }
  
  //Requesting the GPIO
  if(gpio_request(GPIO_IN,"GPIO_IN") < 0){
    pr_err("ERROR: GPIO %d request\n", GPIO_IN);
   // goto r_gpio_in;
  }
  
  //configure the GPIO as input
  gpio_direction_input(GPIO_IN);
  if(gpio_export(GPIO_IN,false) < 0){
    pr_err("ERROR: GPIO %d export\n", GPIO_IN);
   // goto r_gpio_in;
  }
  GPIO_irqNumber = gpio_to_irq(GPIO_IN);
    pr_err("gpio no %d Interrupt \n", GPIO_irqNumber);
  
    if (request_irq(GPIO_irqNumber,  (void *)irq_handler, IRQF_TRIGGER_RISING, "pir2_device", NULL)) {
        
            pr_err("ERROR: setting GPIO %d interruopt \n", GPIO_irqNumber);
        goto r_gpio_in;
    }
 
    printk(KERN_INFO "Device Driver Insert...Done!!!\n");
    return 0;

r_gpio_in:
    gpio_free(GPIO_IN);
r_device:
    class_destroy(dev_class);
r_class:
    unregister_chrdev_region(dev,1);
    return -1;
}
 
static void __exit etx_driver_exit(void)
{
    free_irq(GPIO_IN,NULL);
    gpio_free(GPIO_IN);
    gpio_unexport(GPIO_IN);
    device_destroy(dev_class,dev);
    class_destroy(dev_class);
    cdev_del(&etx_cdev);
    unregister_chrdev_region(dev, 1);
    
    printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}
 
module_init(etx_driver_init);
module_exit(etx_driver_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Trackonomy");
MODULE_DESCRIPTION("Signal to user spcae test");
MODULE_VERSION("1.0");