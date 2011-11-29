obj-$(CONFIG_44x)	:= misc_44x.o
obj-$(CONFIG_EBONY)	+= ebony.o
obj-$(CONFIG_TAISHAN)	+= taishan.o
obj-$(CONFIG_BAMBOO)	+= bamboo.o
obj-$(CONFIG_SEQUOIA)	+= sequoia.o
obj-$(CONFIG_KATMAI)	+= katmai.o
obj-$(CONFIG_RAINIER)	+= rainier.o
obj-$(CONFIG_WARP)	+= warp.o
obj-$(CONFIG_WARP)	+= warp-nand.o
obj-$(CONFIG_CANYONLANDS) += canyonlands.o

ifeq ($(CONFIG_MTD_NAND_NDFC),y)
obj-$(CONFIG_CANYONLANDS) += canyonlands-nand.o
endif
ifeq ($(CONFIG_USB_GADGET_DWC_OTG),y)
obj-$(CONFIG_CANYONLANDS) += canyonlands-usb.o
endif
