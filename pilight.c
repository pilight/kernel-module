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
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/poll.h>

#define BUFFER_SIZE					4000
#define QUEUE_SIZE					1800
#define GPIO_IN						18
#define GPIO_PIN_DEV_STR			"pin"
#define GPIO_PIN_STR				"pilight rf-in"

unsigned short _free_irq = 0;
unsigned short _free_gpio = 0;
unsigned short gpio_pin = GPIO_IN;
unsigned short irq_pin = -1;
char output[BUFFER_SIZE];

struct {
	unsigned long first;
	unsigned long second;
	int old_period;
	int new_period;
} timestamp;

struct timeval tv;
unsigned char valid_buffer = 0x00;

typedef struct pulsequeue_t {
	int pulse;
	struct pulsequeue_t *next;
} pulsequeue_t;

struct pulsequeue_t *pulsequeue;
struct pulsequeue_t *pulsequeue_head;
wait_queue_head_t pulsequeue_inq;
int pulsequeue_number = 0;
int pulsequeue_capture = 0;
struct pulsequeue_t *tmp = NULL;
struct pulsequeue_t *node = NULL;

static int pilight_filter(int delta) {
  valid_buffer <<= 1;
  
  if(delta > 150 && delta < 20000) {
		valid_buffer |= 0x01;
		if(valid_buffer == 0xFF) {
			return delta;
		}
	}
  
	return 0;
}

static irqreturn_t pilight_irq(int irq, void *dev_id, struct pt_regs *regs) {
	//printk(KERN_INFO "pilight_irq()\n");
	long pulse = 0;

	if(pulsequeue_capture) {
		do_gettimeofday(&tv);
		timestamp.first = timestamp.second;
		timestamp.second = 1000000 * (unsigned int)tv.tv_sec + (unsigned int)tv.tv_usec;

		pulse = pilight_filter((int)timestamp.second-(int)timestamp.first);
		if(pulse > 0) {
			if(pulsequeue_number < QUEUE_SIZE) {
				rcu_read_lock();
				node = kmalloc(sizeof(struct pulsequeue_t), GFP_KERNEL);
				node->pulse = (int)timestamp.second-(int)timestamp.first;

				if(pulsequeue_number == 0) {
					rcu_assign_pointer(pulsequeue, node);
					rcu_assign_pointer(pulsequeue_head, node);
				} else {
					rcu_assign_pointer(pulsequeue_head->next, node);
					rcu_assign_pointer(pulsequeue_head, node);
				}

				pulsequeue_number++;
				rcu_read_unlock();
			}
			wake_up_interruptible(&pulsequeue_inq);
			// printk(KERN_INFO "pilight_irq() %d\n", pulsequeue_number);
		}
	}

	return IRQ_HANDLED;
}

static dev_t first;
static struct cdev c_dev;
static struct class *cl;

static int pilight_open(struct inode *i, struct file *f) {
	//printk(KERN_INFO "pilight_open()\n");
	pulsequeue_capture = 1;
	return nonseekable_open(i, f);
}

static int pilight_close(struct inode *i, struct file *f) {
	//printk(KERN_INFO "pilight_close()\n");
	pulsequeue_capture = 0;
	return 0;
}

static ssize_t pilight_read(struct file *f, char __user *buf, size_t len, loff_t *off) {
	
	ssize_t retval = 0;

	if(pulsequeue_number == 0) {
		wait_event_interruptible(pulsequeue_inq, (pulsequeue_number > 0));
	}

	if(pulsequeue_number > 0) {
		rcu_read_lock();
		if((tmp = rcu_dereference(pulsequeue)) != NULL) {
			sprintf(output, "%d", pulsequeue->pulse);
			if(copy_to_user(buf, output, len) != 0) {
				retval = -EFAULT;
			} else {
				retval = strlen(buf);
			}

			pulsequeue = pulsequeue->next;
			kfree(tmp);
			pulsequeue_number--;
			rcu_read_unlock();
			return retval;
		} else {
			rcu_read_unlock();
			return -1;
		}
	}
	return 0;
}

static ssize_t pilight_write(struct file *f, const char __user *buf, size_t len, loff_t *off) {
	//printk(KERN_INFO "pilight_write()\n");
	return len;
}

static int pilight_init_gpio(void) {

	int irq = -1;

	if(!gpio_is_valid(gpio_pin)) {
		printk(KERN_INFO "gpio number %d is invalid\n", gpio_pin);
		return -1;
	}

	if(gpio_request(gpio_pin, GPIO_PIN_STR)) {
		printk(KERN_INFO "cannot claim gpio %d\n", gpio_pin);
		return -1;
	}
	_free_gpio = 1;

	if(gpio_direction_input(gpio_pin) < 0) {
		printk(KERN_INFO "cannot set gpio %d as input\n", gpio_pin);
		gpio_free(gpio_pin);
		return -1;
	}
	
	if((irq_pin = gpio_to_irq(gpio_pin)) < 0) {
		printk(KERN_INFO "gpio to irq mapping failure %d\n", gpio_pin);
		gpio_free(gpio_pin);
		return -1;
	}	
	
	if((irq = request_irq(irq_pin, (irq_handler_t)pilight_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, GPIO_PIN_STR, GPIO_PIN_DEV_STR)) < 0) {
		printk(KERN_INFO "cannot request irq for gpio %d\n", gpio_pin);
		return -1;
	}
	_free_irq = 1;

	printk(KERN_INFO "successfully requested irq %d for gpio %d\n", irq, gpio_pin);
	return 0;
}

static void pilight_deinit_gpio(void) {
	if(_free_irq) {
		free_irq(irq_pin, GPIO_PIN_DEV_STR);
		_free_irq = 0;
	}
	if(_free_gpio) {
		gpio_free(gpio_pin);
		_free_gpio = 0;
	}
}

static long pilight_ioctl(struct file *f, unsigned int name, unsigned long val) {
	printk(KERN_INFO "pilight_ioctl()\n");
	// switch(name) {
		// case 0:
			// printk(KERN_INFO "pilight_ioctl gpio: %d", (int)val);
			// gpio_pin = (int)val;
			// pilight_deinit_gpio();
			// if(pilight_init_gpio() != 0) {
				// return -1;
			// }
		// break;
		// case 1:
			// printk(KERN_INFO "pilight_ioctl min. pulse: %d", (int)val);
		// break;
		// case 10:
			// printk(KERN_INFO "pilight_ioctl max. pulse: %d", (int)val);
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

static int __init pilight_init(void) {
	//printk(KERN_INFO "%s\n", __func__);

	memset(output, '\0', BUFFER_SIZE);

	init_waitqueue_head(&pulsequeue_inq);
	
	if(pilight_init_gpio() != 0) {
		return -1;
	}	

	if(alloc_chrdev_region(&first, 0, 1, "pilight") < 0) {
		printk(KERN_INFO "failed to allocate pilight device\n");
		pilight_deinit_gpio();
		return -1;
	}

	if((cl = class_create(THIS_MODULE, "chardrv")) == NULL) {
		printk(KERN_INFO "failed to create pilight device\n");
		unregister_chrdev_region(first, 1);
		pilight_deinit_gpio();
		return -1;
	}
	
	if(device_create(cl, NULL, first, NULL, "pilight") == NULL) {
		printk(KERN_INFO "failed to register pilight device\n");
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		pilight_deinit_gpio();
		return -1;
	}

	cdev_init(&c_dev, &pugs_fops);

	if(cdev_add(&c_dev, first, 1) == -1) {
		device_destroy(cl, first);
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		pilight_deinit_gpio();
		return -1;
	}
	return 0;
}

static void __exit pilight_exit(void) {

	cdev_del(&c_dev);
	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first, 1);

	pilight_deinit_gpio();	

	while(pulsequeue_number > 0) {
		if((tmp = rcu_dereference(pulsequeue)) != NULL) {
			pulsequeue = pulsequeue->next;
			kfree(tmp);
			pulsequeue_number--;
		} else {
			break;
		}
	}	

	//printk(KERN_INFO "%s\n", __func__);

}

module_init(pilight_init);
module_exit(pilight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CurlyMoo & mercurio.");
MODULE_DESCRIPTION("pilight rf-filter module");