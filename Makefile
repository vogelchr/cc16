ifeq ($(KERNELRELEASE),)
###
# set up environment, then relaunch kernel Makefile
###
KVER:=$(shell uname -r)
KDIR=/lib/modules/$(KVER)/build
PWD:= $(shell pwd)

default : cc16.ko

cc16.ko : cc16.c cc16.h
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

CFLAGS=-Wall -g -I.

.PHONY: clean
clean :
	rm -f *~ *.o *.ko *.mod.[co] .*.cmd cc16-test testpoll
	rm -rf .tmp_versions Module.symvers Modules.symvers
else
###
# objects for our simple module
###

obj-m:= cc16.o

endif
