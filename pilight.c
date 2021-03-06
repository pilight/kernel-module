#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/pid.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define OUTPUT_BUFFER_SIZE			4000
#define FIFO_BUFFER_SIZE			25
#define GPIO_IN						18
#define LONGEST_VALID_PULSE			20000
#define SHORTEST_VALID_PULSE		150

#define GPIO_PIN_DEV_STR			"pin"
#define GPIO_PIN_STR				"pilight rf-in"
#define DEVICE_NAME					"pilight"

#define IOCTL_GPIO_IN				10
#define IOCTL_LONGEST_V_P			11
#define IOCTL_SHORTEST_V_P			12
#define IOCTL_START_RECEIVER		13
#define IOCTL_STOP_RECEIVER			14

#define PILIGHT_DEVICES_N			1

unsigned short _free_irq = 0;
unsigned short _free_gpio = 0;
unsigned short gpio_pin = GPIO_IN;
unsigned short irq_gpio_pin = -1;
char out_buf[OUTPUT_BUFFER_SIZE];

int longest_valid_pulse = LONGEST_VALID_PULSE;
int shortest_valid_pulse = SHORTEST_VALID_PULSE;

struct {
	unsigned long first;
	unsigned long second;
	int old_period;
	int new_period;
} timestamp;

struct timeval tv;
unsigned char valid_buffer = 0x00;

static DECLARE_WAIT_QUEUE_HEAD(fifo_list);
static DECLARE_KFIFO_PTR(pulse_fifo, int);

struct gpio_chip *gpiochip;
struct irq_chip *irqchip;
struct irq_data *irqdata;

static DEFINE_SPINLOCK(lock);        /* GPIO registers */

static int pilight_filter(int pulse_length) {
	valid_buffer <<= 1;
  
	if(pulse_length > shortest_valid_pulse && pulse_length < longest_valid_pulse) {
		valid_buffer |= 0x01;
		if(valid_buffer == 0xFF) {
			return pulse_length;
		}
	}

	return 0;
}

static irqreturn_t pilight_irq(int irq, void *dev_id, struct pt_regs *regs) {
	int signal;
	int pulse_length = 0;

	signal = gpiochip->get(gpiochip, gpio_pin);
	irqchip->irq_unmask(irqdata);

	//printk(KERN_INFO "pilight_irq()\n");

	do_gettimeofday(&tv);
	timestamp.first = timestamp.second;
	timestamp.second = 1000000 * (unsigned int)tv.tv_sec + (unsigned int)tv.tv_usec;

	pulse_length = pilight_filter((int)timestamp.second-(int)timestamp.first);
	
	if(pulse_length > 0) {
		//printk(KERN_INFO "pulse %d: %d\n", n, pulse_length);
		kfifo_put(&pulse_fifo, &pulse_length);
		wake_up_interruptible(&fifo_list);
	}
	return IRQ_HANDLED;
}

static struct class *cl = NULL;
static int major_device_num = 0;
static struct cdev *pilight_devices = NULL;
static int pilight_devices_n = PILIGHT_DEVICES_N;

static int pilight_open(struct inode *i, struct file *f) {
	//printk(KERN_INFO "pilight_open()\n");
	int pulse_length = 0;
	int t = 0;
	while(kfifo_len(&pulse_fifo) > 0) {
		t = kfifo_get(&pulse_fifo, &pulse_length);
	}
	return nonseekable_open(i, f);
}

static int pilight_close(struct inode *i, struct file *f) {
	//printk(KERN_INFO "pilight_close()\n");
	return 0;
}

static ssize_t pilight_read(struct file *f, char __user *buf, size_t len, loff_t *off) {

	ssize_t retval = 0;
	int pulse = 0;
	int t = 0;

	wait_event_interruptible(fifo_list, kfifo_len(&pulse_fifo) != 0);
	t = kfifo_get(&pulse_fifo, &pulse);
	sprintf(out_buf, "%d", pulse);
	if(copy_to_user(buf, out_buf, len ) != 0 ) {
		retval = -EFAULT;
	}else{
		retval = strlen(buf);
	}
	return retval;
}

static ssize_t pilight_write(struct file *f, const char __user *buf, size_t len, loff_t *off) {
	//printk(KERN_INFO "pilight_write()\n");
	return len;
}

static int is_right_chip(struct gpio_chip *chip, void *data) {
	//printk(KERN_INFO "is_right_chip %s %d\n", chip->label, strcmp(data, chip->label));

	if(strcmp(data, chip->label) == 0) {
		return 1;
	}
	return 0;
}

static int pilight_init_gpio(void) {

	int ret, irq;
	unsigned long flags;

	gpiochip = gpiochip_find("bcm2708_gpio", is_right_chip);

	if(!gpiochip) {
		return -ENODEV;
	}

	if(gpio_request(gpio_pin, GPIO_PIN_STR)) {
		printk(KERN_ALERT  ": cant claim gpio pin %d\n", gpio_pin);
		ret = -ENODEV;
        goto exit;
	}
	_free_gpio = 1;
	gpiochip->direction_input(gpiochip, gpio_pin);
	irq = gpiochip->to_irq(gpiochip, gpio_pin);
	//printk(KERN_INFO "to_irq %d\n", irq);
	irqdata = irq_get_irq_data(irq);

    if(irqdata && irqdata->chip) {
		irqchip = irqdata->chip;
	} else {
		ret = -ENODEV;
		goto exit;
	}
	
	if((irq_gpio_pin = gpiochip->to_irq(gpiochip, gpio_pin)) < 0) {
		printk(KERN_INFO "gpio to irq mapping failure %d\n", gpio_pin);
		goto exit;
		ret = -1;
	}	
	
	if(request_irq(irq_gpio_pin, (irq_handler_t)pilight_irq, 0, GPIO_PIN_STR, (void *)0) < 0) {
		printk(KERN_INFO "cannot request irq for gpio %d\n", gpio_pin);
		ret = -1;
	}

	_free_irq = 1;

	spin_lock_irqsave(&lock, flags);
	irqchip->irq_set_type(irqdata, IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING);
	irqchip->irq_unmask(irqdata);

	spin_unlock_irqrestore(&lock, flags);
	return 0;

exit:
	if(_free_gpio) {
		gpio_free(gpio_pin);
	}
	return ret;
}

static void pilight_deinit_gpio(void) {
	unsigned long flags;

	if(_free_irq) {
		spin_lock_irqsave(&lock, flags);
		irqchip->irq_set_type(irqdata, 0);
        irqchip->irq_mask(irqdata);
		spin_unlock_irqrestore(&lock, flags);
		free_irq(irq_gpio_pin, (void *)0);
		_free_irq = 0;
	}
	if(_free_gpio) {
		gpio_free(gpio_pin);
		_free_gpio = 0;
	}
}

static long pilight_ioctl(struct file *f, unsigned int name, unsigned long val) {
	//printk(KERN_INFO "pilight_ioctl()\n");
	 // switch(name) {
		// case IOCTL_GPIO_IN:
			// gpio_pin = (int)val;
			// break;
		// case IOCTL_LONGEST_V_P:
			// longest_valid_pulse = (int)val;
			// break;
		// case IOCTL_SHORTEST_V_P:
			// shortest_valid_pulse = (int)val;
			// break;
		// case IOCTL_START_RECEIVER:
			// printk(KERN_INFO "starting receiver with gpio pin: %d, shortest valid pulse: %d, longest valid pulse: %d\n", gpio_pin, shortest_valid_pulse, longest_valid_pulse);
			// if(pilight_init_gpio() != 0) {
				// return -1;
			// }
			// enable_irq(irq_gpio_pin);
			// break;
		// case IOCTL_STOP_RECEIVER:
			// printk(KERN_INFO "stopping receiver\n");
			// disable_irq(irq_gpio_pin);
			// pilight_deinit_gpio();
			// break;
			 
		// default:
			// return -1;
	// }
	return 0;
}

static struct file_operations pugs_fops = {
	.owner 			= THIS_MODULE,
	.open 			= pilight_open,
	.release		= pilight_close,
	.read 			= pilight_read,
	.write			= pilight_write,
	.unlocked_ioctl	= pilight_ioctl
};

static int pilight_construct_device(struct cdev *cdevice, int minor, struct class *cl){
	int error_n;
	dev_t device_n = MKDEV(major_device_num, minor);
	struct device *device = NULL;
	
	cdev_init(cdevice, &pugs_fops);
	
	error_n = cdev_add(cdevice, device_n, 1);
	if(error_n) {
		printk(KERN_WARNING "error %d while trying to add %s%d", error_n, DEVICE_NAME, minor);
		return error_n;
	}
	
	device = device_create(cl, NULL, device_n, NULL, DEVICE_NAME "%d", minor);
	if(IS_ERR(device)){
		printk(KERN_WARNING "error %d while trying to create %s%d", error_n, DEVICE_NAME, minor);
		error_n=PTR_ERR(device);
		cdev_del(cdevice);
		return error_n;
	}
	
	return 0;
}

static void pilight_destroy_device(struct cdev *cdevice, int minor, struct class *cl){
	device_destroy(cl, MKDEV(major_device_num, minor));
	cdev_del(cdevice);
}

static void pilight_cleanup(int devices_to_destroy){
	int i;

	if(pilight_devices){
		for(i = 0; i < devices_to_destroy; i++){
			pilight_destroy_device(&pilight_devices[i], i, cl);
		}
		kfree(pilight_devices);
	}

	if(cl) {
		class_destroy(cl);
	}
	
	unregister_chrdev_region(MKDEV(major_device_num, 0), pilight_devices_n);
	pilight_deinit_gpio();
}

static int __init pilight_init(void) {
	
	int error_n = 0, i = 0;
	int devices_to_destroy = 0;	
	dev_t dev = 0;

	//printk(KERN_INFO "%s\n", __func__);	
	
	memset(out_buf, '\0', OUTPUT_BUFFER_SIZE);
	
	error_n = kfifo_alloc(&pulse_fifo, FIFO_BUFFER_SIZE, GFP_KERNEL);
	
	if(error_n){
		printk(KERN_ERR "error on fifo allocation\n");
		return error_n;
	}

	if(pilight_devices_n <= 0) {
		printk(KERN_INFO "invalid number of pilight devices (%d)\n", pilight_devices_n);
		return -EINVAL;
	}

	if(alloc_chrdev_region(&dev, 0, pilight_devices_n, DEVICE_NAME) < 0) {
		printk(KERN_INFO "failed to allocate pilight %d devices\n", pilight_devices_n);
		pilight_deinit_gpio();
		return -1;
	}

	major_device_num = MAJOR(dev);

	cl = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(cl)) {
		printk(KERN_INFO "failed to create pilight device\n");
		pilight_cleanup(devices_to_destroy);
		return PTR_ERR(cl);
	}

	pilight_devices = (struct cdev *)kzalloc(pilight_devices_n * sizeof(struct cdev), GFP_KERNEL);
	if(!pilight_devices){
		pilight_cleanup(devices_to_destroy);
		return -ENOMEM;
	}

	for(i = 0; i < pilight_devices_n; ++i){
		error_n = pilight_construct_device(&pilight_devices[i], i, cl);
		if(error_n){
			devices_to_destroy = 1;
			pilight_cleanup(devices_to_destroy);
			return error_n;
		}
	}	
	
	if(pilight_init_gpio() != 0) {
		pilight_cleanup(devices_to_destroy);
		return -1;
	}
	
	printk(KERN_INFO "pilight device driver registered, major %d\n", major_device_num);
	
	return 0;
}

static void __exit pilight_exit(void) {

	pilight_cleanup(pilight_devices_n);
	kfifo_free(&pulse_fifo);

	//printk(KERN_INFO "%s\n", __func__);

}

module_init(pilight_init);
module_exit(pilight_exit);

module_param(gpio_pin, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_pin, "GPIO pin that is connected to the 433 Mhz receiver");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CurlyMoo & mercurio.");
MODULE_DESCRIPTION("pilight rf-filter module");