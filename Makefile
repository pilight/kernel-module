obj-m = pilight.o
KVERSION = $(shell uname -r)
#KPATH = /media/rpi/xbian/linux-headers-3.10.12+/
KPATH = /media/rpi/xbian/linux-4e77f7f1261f65cff06918bc5e66d02a418fc842/
#KPATH = /lib/modules/$(KVERSION)/build/

all:
	make -C $(KPATH) ARCH=arm CROSS_COMPILE=/home/xbian/x-tools/arm-xbian-linux-gnueabi/bin/arm-xbian-linux-gnueabi- M=$(PWD) modules
	#cp pilight.ko /lib/modules/3.10.18+/kernel/
	#modprobe pilight

clean:
	make -C $(KPATH) M=$(PWD) clean
