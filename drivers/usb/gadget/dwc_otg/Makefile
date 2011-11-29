#
# Makefile for DWC_otg Highspeed USB controller driver
#

# CPPFLAGS	+= -DDEBUG

# Use one of the following flags to compile the software in host-only or
# device-only mode based on the configuration selected by the user

#ifeq ($(CONFIG_USB_DWC_MODE_HOST_ONLY),y)
#	CPPFLAGS        += -DDWC_HOST_ONLY
#else
#ifeq ($(CONFIG_USB_DWC_MODE_DEVICE_ONLY),y)
#	CPPFLAGS        += -DDWC_DEVICE_ONLY
#else
#	CPPFLAGS        += -DDWC_OTG_MODE
#endif
#endif

ifeq ($(CONFIG_405EX),y)
# don't enable high-power workaround
#KBUILD_CPPFLAGS    += -DOTG_EXT_CHG_PUMP

# PLB DMA mode
KBUILD_CPPFLAGS	+= -Dlinux -DDWC_SLAVE -DOTG_PLB_DMA -DOTG_PLB_DMA_TASKLET  #-DDWC_DEVICE_ONLY # -DDWC_HS_ELECT_TST  -DDWC_SLAVE -DDWC_HOST_ONLY
endif

ifeq ($(CONFIG_460EX),y)
# currently no PLB DMA mode
KBUILD_CPPFLAGS	+= -Dlinux -DDWC_SLAVE
endif

ifeq ($(CONFIG_405EZ),y)
KBUILD_CPPFLAGS        += -Dlinux -DDWC_DEVICE_ONLY
endif

#CPPFLAGS	+= -Dlinux -DDWC_SLAVE #-DDWC_DEVICE_ONLY # -DDWC_HS_ELECT_TST  -DDWC_SLAVE -DDWC_HOST_ONLY

KBUILD_CPPFLAGS        += -DCONFIG_PPC4xx_EDMA

obj-$(CONFIG_USB_DWC_OTG)	+= dwc_otg.o

dwc_otg-objs	:= dwc_otg_driver.o dwc_otg_attr.o
dwc_otg-objs	+= dwc_otg_cil.o dwc_otg_cil_intr.o
dwc_otg-objs	+= dwc_otg_pcd.o dwc_otg_pcd_intr.o
dwc_otg-objs	+= dwc_otg_hcd.o dwc_otg_hcd_intr.o dwc_otg_hcd_queue.o
dwc_otg-objs	+= ppc4xx_dma.o


clean:
	rm -rf   *.o *.ko .*cmd *.mod.c .tmp_versions cscope.out
