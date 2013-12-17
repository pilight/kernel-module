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

#define DRIVER_AUTHOR				""
#define DRIVER_DESC					"pilight GPIO 433 MHz OOK receiver driver"
#define DEVICE_NAME					"pilight"
#define GPIO_PIN_DEV_STR			"pin"
#define GPIO_PIN_STR				"pilight rf-in"

#define PULSE_VALID_SIGNAL			40
#define PULSE_INVALID_SIGNAL		60
/* Maxumum length of a block that can be read or written in one operation */
#define PILIGHT_DEVICE_BLOCK_SIZE	512
/* Number of devices to create (default: cfake0 and cfake1) */
#define PILIGHT_DEVICE_NDEVICES		1
/* Size of a buffer used for data storage */
#define PILIGHT_DEVICE_BUFFER_SIZE	4000
#define READ_BUFF_LEN				6
#define DEFAULT_GPIO_PIN			23

/* The structure to represent 'pilight' devices. 
 *  data - data buffer;
 *  buffer_size - size of the data buffer;
 *  block_size - maximum number of bytes that can be read or written 
 *    in one call;
 *  pilight_device_mutex - a mutex to protect the fields of this structure;
 *  cdev - character device structure.
 */
struct pilight_device {
	unsigned char *data;
	unsigned long buffer_size; 
	unsigned long block_size;  
	struct mutex pilight_device_mutex; 
	struct cdev cdev;
};
 
/****************************************************************************/
/* GPIO functions declaration block                                         */
/****************************************************************************/
/* set GPIO pin g as input */
#define GPIO_DIR_INPUT(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
/* set GPIO pin g as output */
#define GPIO_DIR_OUTPUT(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
/* get logical value from gpio pin g */
#define GPIO_READ_PIN(g) (*(gpio+13) & (1<<(g))) && 1
/* sets   bits which are 1 ignores bits which are 0 */
#define GPIO_SET_PIN(g)	*(gpio+7) = 1<<g;
/* clears bits which are 1 ignores bits which are 0 */
#define GPIO_CLEAR_PIN(g) *(gpio+10) = 1<<g;
/* Clear GPIO interrupt on the pin we use */
#define GPIO_INT_CLEAR(g) *(gpio+16) = (*(gpio+16) | (1<<g));
/* GPREN0 GPIO Pin Rising Edge Detect Enable/Disable */
#define GPIO_INT_RISING(g,v) *(gpio+19) = v ? (*(gpio+19) | (1<<g)) : (*(gpio+19) ^ (1<<g))
/* GPFEN0 GPIO Pin Falling Edge Detect Enable/Disable */
#define GPIO_INT_FALLING(g,v) *(gpio+22) = v ? (*(gpio+22) | (1<<g)) : (*(gpio+22) ^ (1<<g))

 
/****************************************************************************/
/* Module params declaration block                                          */
/****************************************************************************/
static short int gpio_pin = DEFAULT_GPIO_PIN;
volatile unsigned *gpio;

static int pilight_ndevices = PILIGHT_DEVICE_NDEVICES;
static unsigned long pilight_device_buffer_size = PILIGHT_DEVICE_BUFFER_SIZE;
static unsigned long pilight_device_block_size = PILIGHT_DEVICE_BLOCK_SIZE;
 
module_param(pilight_ndevices, int, S_IRUGO);
module_param(pilight_device_buffer_size, ulong, S_IRUGO);
module_param(pilight_device_block_size, ulong, S_IRUGO);

/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/
short int irq_enabled = 0;    // this is only a flag
static struct timeval last_tv = {0, 0};
unsigned char valid_buffer = 0x00;

/****************************************************************************/
/* Function variables block                                                 */
/****************************************************************************/
pid_t send_to_pid = 0;	// here will be pid of process to which
								// signal will be send
short int signal_valid_pulse = PULSE_VALID_SIGNAL;
short int signal_invalid_pulse = PULSE_INVALID_SIGNAL;

/****************************************************************************/
/* Device variables block                                                   */
/****************************************************************************/
static int major_device_num = 0;
static struct pilight_device *pilight_devices = NULL;
static struct class *pilight_device_class = NULL;

static int pilight_device_open(struct inode *, struct file *);
static int pilight_device_release(struct inode *, struct file *);
static ssize_t pilight_device_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t pilight_device_write(struct file *, const char __user *, size_t, loff_t *);
static loff_t pilight_device_llseek(struct file *, loff_t, int);

static struct file_operations fops = {
	.owner   = THIS_MODULE,
	.read    = pilight_device_read,
	.write   = pilight_device_write,
	.open    = pilight_device_open,
	.release = pilight_device_release,
	.llseek  = pilight_device_llseek,
};

/****************************************************************************/
/* Device funtions                                                          */
/****************************************************************************/
static int pilight_device_open(struct inode *inode, struct file *filp) {

	unsigned int mj = imajor(inode);
	unsigned int mn = iminor(inode);

	struct pilight_device *dev = NULL;
	
	if(mj != major_device_num || mn < 0 || mn >= pilight_ndevices) {
		printk(KERN_WARNING DEVICE_NAME ": [target] No device found with minor=%d and major=%d\n", mj, mn);
		return -ENODEV; /* No such device */
	}

	/* store a pointer to struct pilight_device here for other methods */
	dev = &pilight_devices[mn];
	filp->private_data = dev; 
	
	if(inode->i_cdev != &dev->cdev) {
		printk(KERN_WARNING DEVICE_NAME ": [target] open: internal error\n");
		return -ENODEV; /* No such device */
	}

	/* if opened the 1st time, allocate the buffer */
	if(!dev->data) {
		dev->data = (unsigned char*)kzalloc(dev->buffer_size, GFP_KERNEL);
		if(!dev->data) {
			printk(KERN_WARNING DEVICE_NAME ": [target] open(): out of memory\n");
			return -ENOMEM;
		}
	}
 
   //printk(KERN_INFO DEVICE_NAME ": pilight device open");
   
   return 0;
}
 
static ssize_t pilight_device_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {

	struct pilight_device *dev = (struct pilight_device *)filp->private_data;
	ssize_t retval = 0;
	
	if(mutex_lock_killable(&dev->pilight_device_mutex)) {
		return -EINTR;
	}

	if(*f_pos >= dev->buffer_size) {
		goto out;
	}
	
	if(*f_pos + count > dev->buffer_size) {
		count = dev->buffer_size - *f_pos;
	}
	
	if(count > dev->block_size) {
		count = dev->block_size;
	}

	// printk(KERN_INFO DEVICE_NAME ": device_read buffer data: %s, count: %d, retval: %d\n", &(dev->data[*f_pos]), count, retval);

	if(copy_to_user(buf, &(dev->data[*f_pos]), count) != 0) {
		retval = -EFAULT;
		goto out;
	}

	*f_pos += count;
	retval = count;

out:
	mutex_unlock(&dev->pilight_device_mutex);
	return retval;
}

static ssize_t pilight_device_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {

	struct pilight_device *dev = (struct pilight_device *)filp->private_data;
	ssize_t retval = 0;
	
	if(mutex_lock_killable(&dev->pilight_device_mutex)) {
		return -EINTR;
	}

	if(*f_pos >= dev->buffer_size) {
		/* Writing beyond the end of the buffer is not allowed. */
		retval = -EINVAL;
        goto out;
	}

	if(*f_pos + count > dev->buffer_size) {
		count = dev->buffer_size - *f_pos;
	}

	if(count > dev->block_size) {
		count = dev->block_size;
	}
	
	if(copy_from_user(&(dev->data[*f_pos]), buf, count) != 0) {
		retval = -EFAULT;
		goto out;
	}
	
	if(sscanf(dev->data, "%d", &send_to_pid)  == 1) {
		// printk(KERN_INFO DEVICE_NAME ": send_to_pid set: %d\n", send_to_pid);
		irq_enabled = 1;	// enable interrupt processing flag
	} else {
		printk(KERN_WARNING DEVICE_NAME ": read error\n");
	}
   
	*f_pos += count;
	retval = count;

out:
	mutex_unlock(&dev->pilight_device_mutex);
	return retval;
}
 
static loff_t pilight_device_llseek(struct file *filp, loff_t off, int whence) {

	struct pilight_device *dev = (struct pilight_device *)filp->private_data;
	loff_t newpos = 0;
	
	switch(whence) {
		case 0: /* SEEK_SET */
			newpos = off;
		break;
		case 1: /* SEEK_CUR */
			newpos = filp->f_pos + off;
		break;
		case 2: /* SEEK_END */
			newpos = dev->buffer_size + off;
		break;
		default: /* can't happen */
		return -EINVAL;
	}

	if(newpos < 0 || newpos > dev->buffer_size) {
		return -EINVAL;
	}

	filp->f_pos = newpos;
	return newpos;
}

static int pilight_device_release(struct inode *inode, struct file *filp) {
	return 0;
}

static int pilight_construct_device(struct pilight_device *dev, int minor, struct class *class) {

	int err = 0;
	dev_t devno = MKDEV(major_device_num, minor);
	struct device *device = NULL;

	BUG_ON(dev == NULL || class == NULL);

	/* Memory is to be allocated when the device is opened the first time */
	dev->data = NULL;     
	dev->buffer_size = pilight_device_buffer_size;
	dev->block_size = pilight_device_block_size;
	mutex_init(&dev->pilight_device_mutex);
    
	cdev_init(&dev->cdev, &fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &fops;
   
	err = cdev_add(&dev->cdev, devno, 1);
	if(err) {
		printk(KERN_WARNING "[target] Error %d while trying to add %s%d", err, DEVICE_NAME, minor);
		return err;
	}

	device = device_create(class, NULL, /* no parent device */ devno, NULL, /* no additional data */ DEVICE_NAME "%d", minor);

    if(IS_ERR(device)) {
		err = PTR_ERR(device);
		printk(KERN_WARNING "[target] Error %d while trying to create %s%d", err, DEVICE_NAME, minor);
		cdev_del(&dev->cdev);
		return err;
    }

	return 0;
}

/* Destroy the device and free its buffer */
static void pilight_destroy_device(struct pilight_device *dev, int minor, struct class *class) {
	BUG_ON(dev == NULL || class == NULL);
	device_destroy(class, MKDEV(major_device_num, minor));
	cdev_del(&dev->cdev);
	kfree(dev->data);

	return;
}

/****************************************************************************/
/* Module params binding block.                                             */
/****************************************************************************/
module_param(gpio_pin, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_pin, "GPIO pin that is connected to the 433 Mhz receiver");
 
module_param(signal_valid_pulse, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(signal_valid_pulse, "SIGNAL number for valid pulse notification");

module_param(signal_invalid_pulse, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(signal_invalid_pulse, "SIGNAL number for invalid pulse notification");

/****************************************************************************/
/* RF Data Filter                                                           */
/****************************************************************************/
static int rf_filter(int delta) {
  valid_buffer <<= 1;
  
  if(delta > 150 && delta < 20000) {
		valid_buffer |= 0x01;
		if(valid_buffer == 0xFF) {
			return delta;
		}
	}
  
	return 0;
}

/****************************************************************************/
/* IRQ handler                                                              */
/****************************************************************************/
static irqreturn_t pilight_device_irq_handler(int irq, void *dev_id, struct pt_regs *regs) {
 
	struct pilight_device *dev = &(pilight_devices[0]);
	struct task_struct *task_from_pid;
   
	int pulselength = 0;
	struct timeval tv;
	long deltv;
   
	/* get current time */
	do_gettimeofday(&tv);

	/* calc time since last interrupt in microseconds */
	deltv = tv.tv_sec-last_tv.tv_sec;
   
	// printk(KERN_WARNING DEVICE_NAME ": unfiltered pulselength: %d\n", (int) (deltv * 1000000 + (tv.tv_usec - last_tv.tv_usec)) );
   
	pulselength = rf_filter((int) (deltv * 1000000 + (tv.tv_usec - last_tv.tv_usec)));
   
	// printk(KERN_WARNING DEVICE_NAME ": filtered pulselength: %d\n", pulselength);
  
	last_tv = tv;

	// if interrupt flag is disabled, return
	if(irq_enabled == 0 ) {
		return IRQ_HANDLED;
	}
   
	irq_enabled = 0;
   
	if(pulselength > 0) {
		// copy pulselength to data buffer
		sprintf(dev->data, "%d\n", pulselength);
      
		// get the task from pid for funcion send_sig_info
		if(!(task_from_pid = pid_task(find_vpid(send_to_pid), PIDTYPE_PID))) {
			printk(KERN_WARNING DEVICE_NAME ": can't find task assocaied with pid: %d\n", send_to_pid);
			return IRQ_HANDLED;
		}
   
		// send signal (without info struct) to pid descriped by task
		if(send_sig_info(signal_valid_pulse, SEND_SIG_FORCED, task_from_pid) < 0 ) {
			printk(KERN_INFO DEVICE_NAME ": signal not send\n");
		} else {
			// printk(KERN_INFO DEVICE_NAME ": signal %d was send to pid %d\n", signal_valid_pulse, send_to_pid);
		}
	} else {
		// copy pulselength to data buffer
		sprintf(dev->data, "%d\n", pulselength);
      
		// get the task from pid for funcion send_sig_info
		if(!(task_from_pid = pid_task(find_vpid(send_to_pid), PIDTYPE_PID))) {
			printk(KERN_WARNING DEVICE_NAME ": can't find task assocaied with pid: %d\n", send_to_pid);
			return IRQ_HANDLED;
		}
   
		// send signal (without info struct) to pid descriped by task
		if(send_sig_info(signal_invalid_pulse, SEND_SIG_FORCED, task_from_pid) < 0) {
			printk(KERN_INFO DEVICE_NAME ": signal not send\n");
		} else {
			// printk(KERN_INFO DEVICE_NAME ": signal %d was send to pid %d\n", signal_invalid_pulse, send_to_pid);
		}
	}

	return IRQ_HANDLED;
}
 
 
/****************************************************************************/
/* This function configures interrupts.                                     */
/****************************************************************************/
int pilight_device_interrupt_config(void) {

	if(gpio_request(gpio_pin, GPIO_PIN_STR) != 0) {
		printk(KERN_WARNING DEVICE_NAME ": GPIO request failure %d\n", gpio_pin);
		return -1;
	}
 
	if((gpio_pin = gpio_to_irq(gpio_pin)) < 0) {
		printk(KERN_WARNING DEVICE_NAME ": GPIO to IRQ mapping failure %d\n", gpio_pin);
		return -1;
	}
	
	printk(KERN_INFO DEVICE_NAME ": mapped intterrupt %d\n", gpio_pin);
 
	if(request_irq(gpio_pin, (irq_handler_t ) pilight_device_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, GPIO_PIN_STR, GPIO_PIN_DEV_STR)) {
		printk(KERN_WARNING DEVICE_NAME ": irq request failed\n");
		return -1;
	}

	printk(KERN_INFO DEVICE_NAME ": interrupt configured signal is %d; GPIO pin is %d\n", signal_valid_pulse, gpio_pin);
 
	return 0;
}
 
 
/****************************************************************************/
/* This function releases interrupts.                                       */
/****************************************************************************/
void pilight_device_interrupt_release(void) {

	free_irq(gpio_pin, GPIO_PIN_DEV_STR);
	gpio_free(gpio_pin);

}

/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/
static void pilight_device_module_cleanup(int devices_to_destroy) {
	int i;
	
	/* Get rid of character devices (if any exist) */
	if(pilight_devices) {
		for(i = 0; i < devices_to_destroy; ++i) {
			pilight_destroy_device(&pilight_devices[i], i, pilight_device_class);
        }
		kfree(pilight_devices);
	}
 
	if(pilight_device_class) {
		class_destroy(pilight_device_class);
	}

	/* [NB] pilight_cleanup_module is never called if alloc_chrdev_region() has failed. */
	unregister_chrdev_region(MKDEV(major_device_num, 0), pilight_ndevices);
}

int pilight_device_module_init(void) {
	int err = 0;
	int i = 0;

	int devices_to_destroy = 0;
	dev_t dev = 0;

	printk(KERN_WARNING DEVICE_NAME ": starting pilight device driver\n");
	if(pilight_device_interrupt_config() < 0) {
		return -1;
	}
	
	if(pilight_ndevices <= 0) {
		printk(KERN_WARNING DEVICE_NAME ": [target] Invalid value of cfake_ndevices: %d\n", pilight_ndevices);
		err = -EINVAL;
		return err;
	}
	
	/* Get a range of minor numbers (starting with 0) to work with */
	err = alloc_chrdev_region(&dev, 0, pilight_ndevices, DEVICE_NAME);
	if(err < 0) {
		printk(KERN_WARNING DEVICE_NAME ": [target] alloc_chrdev_region() failed\n");
		return err;
	}

	major_device_num = MAJOR(dev);

	/* Create device class (before allocation of the array of devices) */
	pilight_device_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(pilight_device_class)) {
		err = PTR_ERR(pilight_device_class);
		goto fail;
    }

	/* Allocate the array of devices */
	pilight_devices = (struct pilight_device *)kzalloc(pilight_ndevices * sizeof(struct pilight_device), GFP_KERNEL);
	if(!pilight_devices) {
		err = -ENOMEM;
		goto fail;
	}	

	/* Construct devices */
	for(i = 0; i < pilight_ndevices; ++i) {
		err = pilight_construct_device(&pilight_devices[i], i, pilight_device_class);
        if(err) {
			devices_to_destroy = i;
			goto fail;
		}
	}
 
	printk(KERN_INFO DEVICE_NAME ": pilight device driver registered, major %d\n", major_device_num);

	return 0; /* success */

fail:
	pilight_device_module_cleanup(devices_to_destroy);
	return err;
}

void pilight_device_module_exit(void) {

	pilight_device_interrupt_release();
	pilight_device_module_cleanup(pilight_ndevices);

	printk(KERN_WARNING DEVICE_NAME ": stopped pilight device driver\n");
	return;
}
 
module_init(pilight_device_module_init);
module_exit(pilight_device_module_exit);
 
/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

/****************************************************************************/
/* Module function blocks.                                                  */
/****************************************************************************/