EXTRA_CFLAGS += -Wno-strict-prototypes -Wall -Werror -Wno-declaration-after-statement -Wno-unused-function -std=gnu99 -g
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
#KERNEL_DIR ?= ../debug/linux-3.9
MAKE := make -C $(KERNEL_DIR) M=$(PWD)

obj-m += pcsgu250.o
pcsgu250-objs := scope.o scopecmd.o

all:
	$(MAKE) modules

clean:
	$(MAKE) clean

test: pcsgu250.ko
	cp pcsgu250.ko ../debug
	cp pcsgu250.ko ../debug/initrd/root/
	cp ../res/fw.bin ../debug/initrd/lib/firmware/pcsgu250.bin
	cd ../debug && ./build-initrd.sh
	cd ../debug && sudo ./debug-vm.sh -usb -usbdevice host:10cf:2501

hosttest: pcsgu250.ko
	sudo rmmod pcsgu250.ko || true
	sudo insmod pcsgu250.ko
