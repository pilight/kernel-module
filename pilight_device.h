/* cfake.h */

#ifndef PILIGHT_DEVICE_H_1727_INCLUDED
#define PILIGHT_DEVICE_H_1727_INCLUDED

/* Number of devices to create (default: cfake0 and cfake1) */
#ifndef PILIGHT_DEVICE_NDEVICES
#define PILIGHT_DEVICE_NDEVICES 1    
#endif

/* Size of a buffer used for data storage */
#ifndef PILIGHT_DEVICE_BUFFER_SIZE
#define PILIGHT_DEVICE_BUFFER_SIZE 4000
#endif

/* Maxumum length of a block that can be read or written in one operation */
#ifndef PILIGHT_DEVICE_BLOCK_SIZE
#define PILIGHT_DEVICE_BLOCK_SIZE 512
#endif

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
#endif /* PILIGHT_DEVICE_H_1727_INCLUDED */