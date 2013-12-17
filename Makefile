# Makefile for compiling Kernel
# modules on the fly.

# Modulename
MODULE=pilight_receiver

obj-m += ${MODULE}.o

KVERSION := $(shell uname -r)
INCLUDE += -I/usr/src/linux-headers-$(KVERSION)/arch/arm/mach-omap2/include
INCLUDE += -I/usr/src/linux-headers-$(KVERSION)/arch/arm/plat-omap/include

pilight_receiver_test: pilight_receiver_test.o
	gcc -o pilight_receiver_test pilight_receiver_test.o

pilight_receiver_test.o: pilight_receiver_test.c
	gcc -c pilight_receiver_test.c

module:
	make -C $(INCLUDE) M=$(PWD) modules

clean:
	make -C $(INCLUDE) M=$(PWD) clean
