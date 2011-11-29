#
# Makefile for the Linux network iniCAN device drivers.
#

ifeq ($(KERNELRELEASE),)

KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)
TOPDIR    := $(PWD)/../../../..

modules modules_install clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $@ TOPDIR=$(TOPDIR)

else

-include $(TOPDIR)/Makefile.common

obj-$(CONFIG_CAN_INICAN)	+= inican.o
obj-$(CONFIG_CAN_PPC405EZ)	+= ppc405ez_can.o

endif
