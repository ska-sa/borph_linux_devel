obj-$(CONFIG_KILAUEA)				+= kilauea.o
obj-$(CONFIG_MAKALU)				+= makalu.o
obj-$(CONFIG_WALNUT)				+= walnut.o
obj-$(CONFIG_XILINX_VIRTEX_GENERIC_BOARD)	+= virtex.o
obj-$(CONFIG_EP405)				+= ep405.o

ifeq ($(CONFIG_MTD_NAND_NDFC),y)
obj-$(CONFIG_KILAUEA)				+= kilauea-nand.o
endif
ifeq ($(CONFIG_USB_GADGET_DWC_OTG),y)
obj-$(CONFIG_KILAUEA)				+= kilauea-usb.o
endif
