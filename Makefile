#make file for cross compiling
MODULE=pilight
TESTAPP=test_module

CC  = ${CCPREFIX}gcc
LD  = ${CCPREFIX}ld


# this two variables, depends where you have you raspberry kernel source and tools installed
CCPREFIX=/home/parallels/raspberry/tools-master/arm-bcm2708/arm-bcm2708hardfp-linux-gnueabi/bin/arm-bcm2708hardfp-linux-gnueabi-
KERNEL_SRC=/home/parallels/raspberry/linux-rpi-3.10.23+

obj-m += ${MODULE}.o

module_upload=${MODULE}.ko

all: clean module tester

module: 
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ${KERNEL_SRC} M=$(PWD) modules

tester: ${TESTAPP}.c
	$(CC) -o ${TESTAPP} ${TESTAPP}.c

clean:
	make -C ${KERNEL_SRC} M=$(PWD) clean

# this just copies a file to raspberry (rb is an alias in /etc/hosts as ip of my raspberry pi)
install: all
	scp ${module_upload} pi@raspberryip:/home/pi/pilight/kernel-module
	scp ${TESTAPP} pi@raspberryip:/home/pi/pilight/kernel-module

# show information about module
info:
	modinfo ${module_upload}