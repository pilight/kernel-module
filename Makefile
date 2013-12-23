obj-m = pilight.o
KVERSION = $(shell uname -r)
KPATH3_10_12 = /media/rpi/xbian/linux-headers-3.10.12+/
KPATH3_10_18 = /media/rpi/xbian/linux-headers-3.10.18+/
KPATH3_6_11 = /media/rpi/xbian/linux-headers-3.6.11+/
#KPATH = /lib/modules/$(KVERSION)/build/

all:
	rm *.o || true; rm *.cmd || true; rm *.order || true; rm *.symvers  || true;
	make -C $(KPATH3_10_12) ARCH=arm CROSS_COMPILE=/home/xbian/x-tools/arm-xbian-linux-gnueabi/bin/arm-xbian-linux-gnueabi- M=$(PWD) modules
	mv pilight.ko pilight.ko-3.10.12+
	rm *.o || true; rm *.cmd || true; rm *.order || true; rm *.symvers  || true;
	make -C $(KPATH3_6_11) ARCH=arm CROSS_COMPILE=/home/xbian/x-tools/arm-xbian-linux-gnueabi/bin/arm-xbian-linux-gnueabi- M=$(PWD) modules
	mv pilight.ko pilight.ko-3.6.11+
	rm *.o || true; rm *.cmd || true; rm *.order || true; rm *.symvers  || true;
	make -C $(KPATH3_10_18) ARCH=arm CROSS_COMPILE=/home/xbian/x-tools/arm-xbian-linux-gnueabi/bin/arm-xbian-linux-gnueabi- M=$(PWD) modules
	cp pilight.ko pilight.ko-3.10.18+

clean:
	make -C $(KPATH) M=$(PWD) clean
