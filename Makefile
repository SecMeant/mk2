ifneq ($(KERNELRELEASE),)
	obj-m := mk2.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm Module.symvers modules.order mk2.ko mk2.mod mk2.mod.c mk2.mod.o mk2.o
endif

