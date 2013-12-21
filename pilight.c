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

#define OUTPUT_BUFFER_SIZE			4000
#define FIFO_BUFFER_SIZE			1800
#define GPIO_IN						23
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
	int pulse_length = 0;
	static int n = 0, error_n = 0;
	
	//printk(KERN_INFO "pilight_irq()\n");
	
	
	do_gettimeofday(&tv);
	timestamp.first = timestamp.second;
	timestamp.second = 1000000 * (unsigned int)tv.tv_sec + (unsigned int)tv.tv_usec;

	pulse_length = pilight_filter((int)timestamp.second-(int)timestamp.first);
	
	if(pulse_length > 0) {
		//printk(KERN_INFO "pulse %d: %d\n", n, pulse_length);
		n++;
		error_n = kfifo_put(&pulse_fifo, &pulse_length);
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
	return nonseekable_open(i, f);
}

static int pilight_close(struct inode *i, struct file *f) {
	//printk(KERN_INFO "pilight_close()\n");
	return 0;
}

static ssize_t pilight_read(struct file *f, char __user *buf, size_t len, loff_t *off) {
	
	ssize_t retval = 0;
	//unsigned int copied;
	int pulse = 0;

	wait_event_interruptible(fifo_list, kfifo_len(&pulse_fifo) != 0);
	retval = kfifo_get(&pulse_fifo, &pulse);
	sprintf(out_buf, "%d", pulse);
	if(copy_to_user(buf, out_buf, len ) != 0 ){
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

int pilight_init_gpio(void) {

	int error_n = 0;

	if(!gpio_is_valid(gpio_pin)) {
		printk(KERN_INFO "gpio number %d is invalid\n", gpio_pin);
		return -1;
	}

	error_n = gpio_request(gpio_pin, GPIO_PIN_STR);

	if(error_n != 0) {
		printk(KERN_INFO "cannot claim gpio %d (error: %d)\n", gpio_pin,error_n);
		return -1;
	}
	_free_gpio = 1;

	if(gpio_direction_input(gpio_pin) < 0) {
		printk(KERN_INFO "cannot set gpio %d as input\n", gpio_pin);
		gpio_free(gpio_pin);
		return -1;
	}
	
	if((irq_gpio_pin = gpio_to_irq(gpio_pin)) < 0) {
		printk(KERN_INFO "gpio to irq mapping failure %d\n", gpio_pin);
		gpio_free(gpio_pin);
		return -1;
	}	
	
	if(request_irq(irq_gpio_pin, (irq_handler_t)pilight_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, GPIO_PIN_STR, GPIO_PIN_DEV_STR) < 0) {
		printk(KERN_INFO "cannot request irq for gpio %d\n", gpio_pin);
		return -1;
	}
	disable_irq(irq_gpio_pin);
	_free_irq = 1;

	printk(KERN_INFO "successfully requested irq %d for gpio %d\n", irq_gpio_pin, gpio_pin);
	return 0;
}

static void pilight_deinit_gpio(void) {
	if(_free_irq) {
		free_irq(irq_gpio_pin, GPIO_PIN_DEV_STR);
		_free_irq = 0;
	}
	if(_free_gpio) {
		gpio_free(gpio_pin);
		_free_gpio = 0;
	}
}

static long pilight_ioctl(struct file *f, unsigned int name, unsigned long val) {
	//printk(KERN_INFO "pilight_ioctl()\n");
	 switch(name) {
		case IOCTL_GPIO_IN:
			gpio_pin = (int)val;
			break;
		case IOCTL_LONGEST_V_P:
			longest_valid_pulse = (int)val;
			break;
		case IOCTL_SHORTEST_V_P:
			shortest_valid_pulse = (int)val;
			break;
		case IOCTL_START_RECEIVER:
			printk(KERN_INFO "starting receiver with gpio pin: %d, shortest valid pulse: %d, longest valid pulse: %d\n",gpio_pin,shortest_valid_pulse,longest_valid_pulse);
			if(pilight_init_gpio() != 0) {
				return -1;
			}
			enable_irq(irq_gpio_pin);
			break;
		case IOCTL_STOP_RECEIVER:
			printk(KERN_INFO "stopping receiver\n");
			disable_irq(irq_gpio_pin);
			pilight_deinit_gpio();
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
		printk(KERN_WARNING "error %d while trying to add %s%d", error_n, DEVICE_NAME, minor);
		return error_n;
	}
	
	device = device_create(cl,NULL,device_n,NULL,DEVICE_NAME "%d",minor);
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
	
	if(cl)
	{
		class_destroy(cl);
	}
	
	unregister_chrdev_region(MKDEV(major_device_num, 0), pilight_devices_n);
	pilight_deinit_gpio();
}

static int __init pilight_init(void) {
	//printk(KERN_INFO "%s\n", __func__);
	int error_n = 0, i = 0;
	int devices_to_destroy = 0;
	
	dev_t dev = 0;
	
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
		error_n = pilight_consturct_device(&pilight_devices[i], i, cl);
		if(error_n){
			devices_to_destroy = 1;
			pilight_cleanup(devices_to_destroy);
			return error_n;
		}
	}
	
	printk(KERN_INFO "pilight device driver registered, major %d\n",major_device_num);
	
	return 0;
}

static void __exit pilight_exit(void) {

	pilight_cleanup(pilight_devices_n);
	kfifo_free(&pulse_fifo);

	//printk(KERN_INFO "%s\n", __func__);

}

module_init(pilight_init);
module_exit(pilight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CurlyMoo & mercurio.");
MODULE_DESCRIPTION("pilight rf-filter module");