#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/pid.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/delay.h>

#define MAX_USLEEP					100*1000

#define READ_BUFFER_SIZE			4000
#define FIFO_BUFFER_SIZE			1800
#define GPIO_IN						23
#define GPIO_OUT					27
#define LONGEST_VALID_PULSE			20000
#define SHORTEST_VALID_PULSE		150

#define GPIO_IN_PIN_DEV_STR			"in_pin"
#define GPIO_IN_PIN_STR				"pilight rf-in"
#define GPIO_OUT_PIN_DEV_STR		"out_pin"
#define GPIO_OUT_PIN_STR			"pilight rf-out"
#define DEVICE_NAME					"pilight"

#define dprintk(fmt, args...) \
do { if (debug) printk(KERN_DEBUG DEVICE_NAME ": " fmt, ## args); } while (0)

#define iprintk(fmt, args...) \
do { printk(KERN_INFO DEVICE_NAME ": " fmt, ## args); } while (0)


#define IOCTL_GPIO_IN				10
#define IOCTL_LONGEST_V_P			11
#define IOCTL_SHORTEST_V_P			12
#define IOCTL_START_RECEIVER		13
#define IOCTL_STOP_RECEIVER			14
#define IOCTL_FILTER_ON				15
#define IOCTL_GPIO_OUT				16
#define IOCTL_START_TRANSMITTER		17
#define IOCTL_STOP_TRANSMITTER		18

#define PILIGHT_DEVICES_N			1

#define FILTER_ON					0

unsigned short debug = 0;

module_param(debug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(debug, "Turn debuging output on/off");

unsigned short _free_irq = 0;
unsigned short _free_gpio_in = 0;
unsigned short _free_gpio_out = 0;

unsigned short gpio_in_pin = -1;
unsigned short gpio_out_pin = -1;
unsigned short irq_gpio_pin = -1;

static int pulse_tollerance = 5;

module_param(pulse_tollerance, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(pulse_tollerance, "Pulse length tollerance (us) for 433 MHz transmitter");

module_param(gpio_in_pin, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_in_pin, "GPIO pin that is connected to the 433 MHz receiver");

module_param(gpio_out_pin, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_out_pin, "GPIO pin that is connected to the 433 MHz sender");

char read_buf[READ_BUFFER_SIZE];

int longest_valid_pulse = -1;
int shortest_valid_pulse = -1;
short filter_on = -1;

struct {
	unsigned long first;
	unsigned long second;
	int old_period;
	int new_period;
} timestamp;

struct timeval tv;
unsigned char valid_buffer = 0x00;

static DECLARE_WAIT_QUEUE_HEAD(fifo_in_list);
static DECLARE_KFIFO_PTR(pulse_fifo_in, int);


static int pilight_filter(int pulse_length) {
	//dprintk("pilight_filter()\n");
	valid_buffer <<= 1;

	if(pulse_length > shortest_valid_pulse && pulse_length < longest_valid_pulse) {
		valid_buffer |= 0x01;
		if((filter_on) && (valid_buffer == 0xFF)) {
			return pulse_length;
		}else{
			if((! filter_on)){
				return pulse_length;
			}
		}
	}

	return 0;
}

static irqreturn_t pilight_irq(int irq, void *dev_id, struct pt_regs *regs) {
	int pulse_length = 0;
	static int n = 0, error_n = 0;
	//dprintk("pilight_irq()\n");

	do_gettimeofday(&tv);
	timestamp.first = timestamp.second;
	timestamp.second = 1000000 * (unsigned int)tv.tv_sec + (unsigned int)tv.tv_usec;

	pulse_length = pilight_filter((int)timestamp.second-(int)timestamp.first);

	if(pulse_length > 0) {
		//dprintk("pulse %d: %d\n", n, pulse_length);
		n++;
		error_n = kfifo_put(&pulse_fifo_in, &pulse_length);
		wake_up_interruptible(&fifo_in_list);
	}

	return IRQ_HANDLED;
}

static struct class *cl = NULL;
static int major_device_num = 0;
static struct cdev *pilight_devices = NULL;
static int pilight_devices_n = PILIGHT_DEVICES_N;

static int safe_usleep(int microseconds){
	if((microseconds > 0) && (microseconds < MAX_USLEEP)){
		usleep_range((microseconds-pulse_tollerance),(microseconds+pulse_tollerance));
	}else{
		usleep_range((MAX_USLEEP - pulse_tollerance),(MAX_USLEEP + pulse_tollerance));
	}
	return 1;
}

static int pilight_open(struct inode *i, struct file *f) {
	dprintk("pilight_open()\n");
	return nonseekable_open(i, f);
}

static int pilight_close(struct inode *i, struct file *f) {
	dprintk("pilight_close()\n");
	return 0;
}

static ssize_t pilight_read(struct file *f, char __user *buf, size_t len, loff_t *off) {
	ssize_t retval = 0;
	int pulse = 0;

	//dprintk("pilight_read()\n");
	wait_event_interruptible(fifo_in_list, kfifo_len(&pulse_fifo_in) != 0);
	retval = kfifo_get(&pulse_fifo_in, &pulse);
	sprintf(read_buf, "%d", pulse);
	if(copy_to_user(buf, read_buf, len ) != 0 ){
		retval = -EFAULT;
	}else{
		retval = strlen(buf);
	}
	return retval;
}

static ssize_t pilight_write(struct file *f, const char __user *buf, size_t len, loff_t *off) {
	int pulse_count = 0, i;
	int *pulse_buf;
	struct timeval t;
	struct tm broken;

	//dprintk("pilight_write()\n");

	do_gettimeofday(&tv);

	pulse_count = len / sizeof(int);

	if (len % sizeof(int))
		return -EINVAL;

	pulse_buf = memdup_user(buf, len);

	if (IS_ERR(pulse_buf))
		return PTR_ERR(pulse_buf);

	disable_irq(irq_gpio_pin);

	for (i = 0; i < (pulse_count-1); i++) {
		if (i%2){
			//dprintk("gpio pin set to 0, pulse: %d, pulse length: %d\n",i , pulse_buf[i]);
			gpio_set_value(gpio_out_pin,0);
			safe_usleep(pulse_buf[i]);
		}
		else{
			//dprintk("gpio pin set to 1, pulse: %d, pulse length: %d\n",i , pulse_buf[i]);
			gpio_set_value(gpio_out_pin,1);
			safe_usleep(pulse_buf[i]);
		}
	}

	if (pulse_count%2){
		//dprintk("gpio pin set to 1, last pulse\n");
		gpio_set_value(gpio_out_pin,1);
		safe_usleep(MAX_USLEEP);
	}

	//dprintk("gpio pin set to 0\n");
	gpio_set_value(gpio_out_pin,0);

	enable_irq(irq_gpio_pin);

	return len;
}

int pilight_init_gpio_in(void) {

	int error_n = 0;

	dprintk("starting gpio in request\n");

	if(!gpio_is_valid(gpio_in_pin)) {
		printk(KERN_INFO "gpio number %d is invalid\n", gpio_in_pin);
		return -1;
	}

	error_n = gpio_request(gpio_in_pin, GPIO_IN_PIN_STR);

	if(error_n != 0) {
		iprintk("cannot claim gpio in pin %d (error: %d)\n", gpio_in_pin,error_n);
		return error_n;
	}
	_free_gpio_in = 1;

	if(gpio_direction_input(gpio_in_pin) < 0) {
		iprintk("cannot set gpio pin %d as input\n", gpio_in_pin);
		gpio_free(gpio_in_pin);
		_free_gpio_in = 0;
		return -1;
	}	

	dprintk("gpio in request done, starting irq request\n");

	if((irq_gpio_pin = gpio_to_irq(gpio_in_pin)) < 0) {
		iprintk("gpio to irq mapping failure %d\n", gpio_in_pin);
		gpio_free(gpio_in_pin);
		_free_gpio_in = 0;
		return -1;
	}

	dprintk("irq %d mapped to gpio pin %d\n", irq_gpio_pin, gpio_in_pin);

	if(request_irq(irq_gpio_pin, (irq_handler_t)pilight_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, GPIO_IN_PIN_STR, GPIO_IN_PIN_DEV_STR) < 0) {
		iprintk("cannot request irq for gpio %d\n", gpio_in_pin);
		return -1;
	}
	disable_irq(irq_gpio_pin);
	_free_irq = 1;

	dprintk("successfully requested irq %d for gpio %d\n", irq_gpio_pin, gpio_in_pin);
	return 0;
}

int pilight_init_gpio_out(void) {

	int error_n = 0;

	if(!gpio_is_valid(gpio_out_pin)) {
		printk(KERN_INFO "gpio number %d is invalid\n", gpio_out_pin);
		return -1;
	}

	error_n = gpio_request(gpio_out_pin, GPIO_OUT_PIN_STR);

	if(error_n != 0) {
		iprintk("cannot claim gpio out pin %d (error: %d)\n", gpio_out_pin,error_n);
		return error_n;
	}
	_free_gpio_out = 1;

	if(gpio_direction_output(gpio_out_pin,1) < 0) {
		iprintk("cannot set gpio pin %d as output\n", gpio_out_pin);
		gpio_free(gpio_out_pin);
		_free_gpio_out = 0;
		return -1;
	}

	gpio_set_value(gpio_out_pin,0);

	dprintk("successfully set gpio %d pin as output\n", gpio_out_pin);
	return 0;
}

static void pilight_deinit_gpio_in(void) {
	dprintk("pilight_deinit_gpio_in()\n");
	if(_free_irq) {
		dprintk("free gpio in irq\n");
		free_irq(irq_gpio_pin, GPIO_IN_PIN_DEV_STR);
		_free_irq = 0;
	}
	if(_free_gpio_in) {
		dprintk("free gpio in\n");
		gpio_free(gpio_in_pin);
		_free_gpio_in = 0;
	}
}

static void pilight_deinit_gpio_out(void) {
	dprintk("pilight_deinit_gpio_out()\n");
	if(_free_gpio_out) {
		dprintk("free gpio out\n");
		gpio_free(gpio_out_pin);
		_free_gpio_out = 0;
	}
}

static long pilight_ioctl(struct file *f, unsigned int name, unsigned long val) {
	dprintk("pilight_ioctl()\n");
	switch(name) {
		case IOCTL_GPIO_IN:
			gpio_in_pin = (int)val;
			break;
		case IOCTL_GPIO_OUT:
			gpio_out_pin = (int)val;
			break;
		case IOCTL_LONGEST_V_P:
			longest_valid_pulse = (int)val;
			break;
		case IOCTL_SHORTEST_V_P:
			shortest_valid_pulse = (int)val;
			break;
		case IOCTL_FILTER_ON:
			filter_on = (int)val;
			break;
		case IOCTL_START_RECEIVER:
			dprintk("starting receiver\n");
			if(gpio_in_pin == -1){
				iprintk("gpio in pin not set!\n");
				return -1;
			}
			if(shortest_valid_pulse == -1){
				iprintk("shortest valid pulse not set! setting to default (%d)\n", SHORTEST_VALID_PULSE);
				shortest_valid_pulse = SHORTEST_VALID_PULSE;
			}
			if(longest_valid_pulse == -1){
				iprintk("longest valid pulse not set! setting to default (%d)\n", LONGEST_VALID_PULSE);
				longest_valid_pulse = LONGEST_VALID_PULSE;
			}
			if(filter_on == -1){
				iprintk("attiny prefilter status not set! setting to default (%d)\n", FILTER_ON);
				filter_on = FILTER_ON;
			}
			dprintk("starting receiver initialization\n");

			if(pilight_init_gpio_in() != 0) {
				pilight_deinit_gpio_in();
				return -1;
			}

			if(!filter_on){
				iprintk("started receiver without kernel prefilter, gpio pin: %d, shortest valid pulse: %d, longest valid pulse: %d\n",gpio_in_pin,shortest_valid_pulse,longest_valid_pulse);
			}else{
				iprintk("started receiver with kernel prefilter, gpio pin: %d, shortest valid pulse: %d, longest valid pulse: %d\n",gpio_in_pin,shortest_valid_pulse,longest_valid_pulse);
			}
			enable_irq(irq_gpio_pin);
			break;
		case IOCTL_STOP_RECEIVER:
			dprintk("stopping receiver\n");
			disable_irq(irq_gpio_pin);
			pilight_deinit_gpio_in();
			iprintk("stopped receiver\n");

			break;
		case IOCTL_START_TRANSMITTER:
			dprintk("starting transmitter\n");
			if(gpio_out_pin == -1){
				iprintk("gpio out pin not set!\n");
				return -1;
			}

			if(pilight_init_gpio_out() != 0) {
				pilight_deinit_gpio_out();
				return -1;
			}
			iprintk("started transmitter, gpio pin: %d\n",gpio_out_pin);
			break;
		case IOCTL_STOP_TRANSMITTER:
			dprintk("stopping transmitter\n");
			pilight_deinit_gpio_out();
			iprintk("stopped transmitter\n");
			break;
		default:
			return -1;
	}
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

static int pilight_consturct_device(struct cdev *cdevice, int minor, struct class *cl){
	int error_n;
	dev_t device_n = MKDEV(major_device_num, minor);
	struct device *device = NULL;

	cdev_init(cdevice, &pugs_fops);

	error_n = cdev_add(cdevice, device_n, 1);
	if(error_n) {
		iprintk("error %d while trying to add %s%d", error_n, DEVICE_NAME, minor);
		return error_n;
	}

	device = device_create(cl,NULL,device_n,NULL,DEVICE_NAME "%d",minor);
	if(IS_ERR(device)){
		iprintk("error %d while trying to create %s%d", error_n, DEVICE_NAME, minor);
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

	if(cl)
	{
		class_destroy(cl);
	}

	unregister_chrdev_region(MKDEV(major_device_num, 0), pilight_devices_n);
	pilight_deinit_gpio_in();
	pilight_deinit_gpio_out();
}

static int __init pilight_init(void) {
	int error_n = 0, i = 0;
	int devices_to_destroy = 0;
	dev_t dev = 0;

	dprintk("%s\n", __func__);

	memset(read_buf, '\0', READ_BUFFER_SIZE);

	error_n = kfifo_alloc(&pulse_fifo_in, FIFO_BUFFER_SIZE, GFP_KERNEL);

	if(error_n){
		iprintk("error on fifo allocation\n");
		return error_n;
	}


	if(pilight_devices_n <= 0) {
		iprintk("invalid number of pilight devices (%d)\n", pilight_devices_n);
		return -EINVAL;
	}

	if(alloc_chrdev_region(&dev, 0, pilight_devices_n, DEVICE_NAME) < 0) {
		iprintk("failed to allocate pilight %d devices\n", pilight_devices_n);
		return -1;
	}

	major_device_num = MAJOR(dev);

	cl = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(cl)) {
		iprintk("failed to create pilight device\n");
		pilight_cleanup(devices_to_destroy);
		return PTR_ERR(cl);
	}

	pilight_devices = (struct cdev *)kzalloc(pilight_devices_n * sizeof(struct cdev), GFP_KERNEL);
	if(!pilight_devices){
		pilight_cleanup(devices_to_destroy);
		return -ENOMEM;
	}

	for(i = 0; i < pilight_devices_n; ++i){
		error_n = pilight_consturct_device(&pilight_devices[i], i, cl);
		if(error_n){
			devices_to_destroy = 1;
			pilight_cleanup(devices_to_destroy);
			return error_n;
		}
	}

	iprintk("pilight device driver registered, major %d\n",major_device_num);

	return 0;
}

static void __exit pilight_exit(void) {

	pilight_cleanup(pilight_devices_n);
	kfifo_free(&pulse_fifo_in);

	dprintk("%s\n", __func__);

}

module_init(pilight_init);
module_exit(pilight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CurlyMoo & mercuri0");
MODULE_DESCRIPTION("pilight rf-filter module");