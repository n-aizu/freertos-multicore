default: rpmsg_client_sample2

CROSS_COMPILE ?= arm-none-eabi-
KERNELDIR ?= /path/to/linux/kernel/source
ccflags-y = -g
PWD = `pwd`

ifeq ($(KERNELRELEASE),)

.PHONY: clean.
clean:
	rm -rf *.o *.~ *.ko *.mod.c .*.cmd \
		.tmp_versions Module.symvers modules.order

rpmsg_client_sample2:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

else
	obj-m:= rpmsg_client_sample2.o
endif
