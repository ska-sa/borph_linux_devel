/*
 *	mb86290fb_main.c  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/string.h>

#include "mb86290fbdev.h"

#define GEO_HEADER_INIT		0x40000000L

struct mb86290fb_info mb86290fb_info[MB86290FB_MAX_CARD];
struct mb86290fb_info *mb86290fb_pInfo = &mb86290fb_info[0];

static void initialize_mb86290fb_info(struct mb86290fb_info *pinfo)
{
	/* MB86290FB_INITPARAM initparam; */
	pinfo->initparam.cputype	= MB86290FB_DFLT_IP_CPUTYPE;
	pinfo->initparam.gdctype	= MB86290FB_DFLT_IP_GDCTYPE;
	pinfo->initparam.gdcbase	= MB86290FB_DFLT_IP_GDCBASE;
	pinfo->initparam.geoclock	= MB86290FB_DFLT_IP_GEOCLOCK;
	pinfo->initparam.otherclock	= MB86290FB_DFLT_IP_OTHERCLOCK;
	pinfo->initparam.locate		= MB86290FB_DFLT_IP_LOCATE;
	pinfo->initparam.memorymode	= MB86290FB_DFLT_IP_MEMORYMODE;
	pinfo->initparam.disp.mode	= MB86290FB_DFLT_IP_MODE;
	pinfo->initparam.disp.htp	= MB86290FB_DFLT_IP_HTP;
	pinfo->initparam.disp.hsp	= MB86290FB_DFLT_IP_HSP;
	pinfo->initparam.disp.hsw	= MB86290FB_DFLT_IP_HSW;
	pinfo->initparam.disp.hdp	= MB86290FB_DFLT_IP_HDP;
	pinfo->initparam.disp.vtr	= MB86290FB_DFLT_IP_VTR;
	pinfo->initparam.disp.vsp	= MB86290FB_DFLT_IP_VSP;
	pinfo->initparam.disp.vsw	= MB86290FB_DFLT_IP_VSW;
	pinfo->initparam.disp.vdp	= MB86290FB_DFLT_IP_VDP;

	/* fb_ops ops; */
	pinfo->ops.owner	= NULL;
	pinfo->ops.fb_open	= NULL;
	pinfo->ops.fb_release	= NULL;
	pinfo->ops.fb_pan_display = NULL;
	pinfo->ops.fb_ioctl	= mb86290fb_ioctl;
	pinfo->ops.fb_mmap	= mb86290fb_mmap;

	/* struct fb_var_screeninfo screeninfo; */
	pinfo->screeninfo.xres		= MB86290FB_DFLT_SI_XRES;
	pinfo->screeninfo.yres		= MB86290FB_DFLT_SI_YRES;
	pinfo->screeninfo.xres_virtual	= MB86290FB_DFLT_SI_XRES_VIRTUAL;
	pinfo->screeninfo.yres_virtual	= MB86290FB_DFLT_SI_YRES_VIRTUAL;
	pinfo->screeninfo.xoffset	= MB86290FB_DFLT_SI_XOFFSET;
	pinfo->screeninfo.yoffset	= MB86290FB_DFLT_SI_YOFFSET;
	pinfo->screeninfo.bits_per_pixel = MB86290FB_DFLT_SI_BITS_PER_PIXEL;
	pinfo->screeninfo.grayscale	= MB86290FB_DFLT_SI_GRAYSCALE;
	pinfo->screeninfo.red.offset	= MB86290FB_DFLT_SI_RED_OFFSET;
	pinfo->screeninfo.red.length	= MB86290FB_DFLT_SI_RED_LENGTH;
	pinfo->screeninfo.red.msb_right	= MB86290FB_DFLT_SI_RED_RIGHT;
	pinfo->screeninfo.green.offset	= MB86290FB_DFLT_SI_GREEN_OFFSET;
	pinfo->screeninfo.green.length	= MB86290FB_DFLT_SI_GREEN_LENGTH;
	pinfo->screeninfo.green.msb_right = MB86290FB_DFLT_SI_GREEN_RIGHT;
	pinfo->screeninfo.blue.offset	= MB86290FB_DFLT_SI_BLUE_OFFSET;
	pinfo->screeninfo.blue.length	= MB86290FB_DFLT_SI_BLUE_LENGTH;
	pinfo->screeninfo.blue.msb_right = MB86290FB_DFLT_SI_BLUE_RIGHT;
	pinfo->screeninfo.transp.offset	= MB86290FB_DFLT_SI_TRANSP_OFFSET;
	pinfo->screeninfo.transp.length	= MB86290FB_DFLT_SI_TRANSP_LENGTH;
	pinfo->screeninfo.transp.msb_right = MB86290FB_DFLT_SI_TRANSP_RIGHT;

	pinfo->screeninfo.nonstd	= MB86290FB_DFLT_SI_NONSTD;
	pinfo->screeninfo.activate	= MB86290FB_DFLT_SI_ACTIVATE;
	pinfo->screeninfo.height	= MB86290FB_DFLT_SI_HEIGHT;
	pinfo->screeninfo.width		= MB86290FB_DFLT_SI_WIDTH;
	pinfo->screeninfo.accel_flags	= MB86290FB_DFLT_SI_ACCEL_FLAGS;
	pinfo->screeninfo.pixclock	= MB86290FB_DFLT_SI_PIXCLOCK;
	pinfo->screeninfo.left_margin	= MB86290FB_DFLT_SI_LEFT_MARGIN;
	pinfo->screeninfo.right_margin	= MB86290FB_DFLT_SI_RIGHT_MARGIN;
	pinfo->screeninfo.upper_margin	= MB86290FB_DFLT_SI_UPPER_MARGIN;
	pinfo->screeninfo.lower_margin	= MB86290FB_DFLT_SI_LOWER_MARGIN;
	pinfo->screeninfo.hsync_len	= MB86290FB_DFLT_SI_HSYNC_LEN;
	pinfo->screeninfo.vsync_len	= MB86290FB_DFLT_SI_VSYNC_LEN;
	pinfo->screeninfo.sync		= MB86290FB_DFLT_SI_SYNC;
	pinfo->screeninfo.vmode		= MB86290FB_DFLT_SI_VMODE;

}

static void Wait(int ms)
{
	int onejit = 1000000 / HZ;

	if (ms * 1000 < onejit)
		schedule_timeout(1);
	else
		schedule_timeout(ms * 1000 / onejit);
}

#if !defined(CONFIG_FB_PRE_INIT_FB)
static void set_disp_clock(GDC_ULONG mode)
{
	GDC_ULONG newmode;

	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		/* for MB86293 later */
		newmode = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE);
		newmode = (newmode & ~GDC_DISP_DCM_MASK) |
			  (mode & GDC_DISP_DCM_MASK);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_EXT_MODE, newmode);
	} else {
		/* for MB86290A/86291/86291A/86292 */
		newmode = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_MODE);
		newmode = (newmode & ~GDC_DISP_DCM_MASK) |
			  (mode & GDC_DISP_DCM_MASK);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_MODE, newmode);
	}
}

static void set_disp_timing(unsigned long htp, unsigned long hsp,
			    unsigned long hsw, unsigned long hdp,
			    unsigned long vtr, unsigned long vsp,
			    unsigned long vsw, unsigned long vdp)
{
	unsigned long hdb;

	/* Parameters are decreased 1 because hardware spec is 0 to (n-1) */
	hdp -= 1;
	vdp -= 1;
	htp -= 1;
	hsp -= 1;
	hsw -= 1;
	vtr -= 1;
	vsp -= 1;
	vsw -= 1;

	hdb = hdp;

	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_H_TOTAL, htp << 16);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_H_PERIOD, (hdb << 16) | (hdp));
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_V_H_W_H_POS,
				      (vsw << 24) | (hsw << 16) | (hsp));
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_V_TOTAL, vtr << 16);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_V_PERIOD_POS, (vdp << 16) | (vsp));
	PDEBUG("HTP: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_H_TOTAL) );
	PDEBUG("HDP: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_H_PERIOD) );
	PDEBUG("VHW-HP: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_V_H_W_H_POS) );
	PDEBUG("VTR: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_V_TOTAL) );
	PDEBUG("VDP-P: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_V_PERIOD_POS) );
}
#endif

static void init_disp_param(void)
{
	GDC_ULONG win_size;

	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_C_TRANS, 0);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_MLMR_TRANS, 0);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_CURSOR_MODE, 0x00030100);

	/* Sets default cursor position */
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_CUR1_POS,
				      MB86290FB_XY_PACK(0, 0));
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_CUR2_POS,
				      MB86290FB_XY_PACK(0, 0));

	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		win_size = ((mb86290fb_pInfo->initparam.disp.vdp - 1) << 16) |
			    (mb86290fb_pInfo->initparam.disp.hdp);

#if !defined(CONFIG_FB_PRE_INIT_FB)
		/* for MB86293 later */
		/* Set default display mode */
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L0_EXT_MODE, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L1_EXT_MODE, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L2_EXT_MODE, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L3_EXT_MODE, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L4_EXT_MODE, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L5_EXT_MODE, 0);

		/* Set default pos and size */
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L0_WIN_POS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L0_WIN_SIZE,
					      win_size);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L1_WIN_POS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L1_WIN_SIZE,
					      win_size);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L2_WIN_POS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L2_WIN_SIZE,
					      win_size);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L3_WIN_POS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L3_WIN_SIZE,
					      win_size);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L4_WIN_POS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L4_WIN_SIZE,
					      win_size);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L5_WIN_POS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L5_WIN_SIZE,
					      win_size);

		/* Set default blend mode */
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L0, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L1, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L2, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L3, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L4, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L5, 0);

		/* Set default transparency mode */
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L0_TRANS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L1_TRANS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L2_TRANS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L3_TRANS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L4_TRANS, 0);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L5_TRANS, 0);
#endif
	}
}

#if !defined(CONFIG_FB_PRE_INIT_FB)
static void enable_backlight(void)
{
	unsigned long fpga1_phy_base = 0xc4000000;
	unsigned long fpga1_base;

	fpga1_base = (unsigned long)ioremap64(0x100000000ULL|fpga1_phy_base, 0x1000);
	PDEBUG("PWM ctrl 0x%lx\n", *(unsigned long*)(fpga1_base + 0x20));
	PDEBUG("PWM duty cycle 0x%lx\n", *(unsigned long*)(fpga1_base + 0x24));
	if ( *(unsigned long*)(fpga1_base + 0x20) != 0x701 )
		*(unsigned long*)(fpga1_base + 0x20) = 0x701;
	if ( *(unsigned long*)(fpga1_base + 0x24) != 0x64 )
		*(unsigned long*)(fpga1_base + 0x24) = 0x64;
	iounmap((void*)fpga1_base);
}
#endif

int initialize_gdc(int init_flag)
{
	if (init_flag) {
#if !defined(CONFIG_LWMON5)
		/* Remap GDC base address */
		GdcSystemInfo.gdcbase = (GDC_ULONG) ioremap(mb86290fb_pInfo->base_phys,
							    MB86290FB_MAPSIZE);
#else
		#ifndef MB86290FB_USE_PCI
		mb86290fb_pInfo->base_phys = MB86290FB_DFLT_IP_GDCBASE;
		mb86290fb_pInfo->irq = 8;
		#endif
		if (!request_mem_region(mb86290fb_pInfo->base_phys, MB86290FB_MAPSIZE,
					"mb86290fb FB/MMIO")) {
			printk(KERN_ERR "MB86290: cannot reserve framebuffer/io memory\n");
			return -ENODEV;
		}

		/* Map ppc44x 36bit addr. Address range check/fixup for Lime is also
		   added in fixup_bigphys_addr() to be able to mmap from /dev/fb0 or /dev/mem. */
		GdcSystemInfo.gdcbase = (GDC_ULONG) ioremap64((0x100000000ULL | mb86290fb_pInfo->base_phys),
							      MB86290FB_MAPSIZE);
#endif
		PDEBUG("GDC base phys/virt address: 0x%08lx/0x%08lx\n",
		       mb86290fb_pInfo->base_phys, GdcSystemInfo.gdcbase);

		/* GdcSystemInfo */
		GdcSystemInfo.cputype = mb86290fb_pInfo->initparam.cputype;
		GdcSystemInfo.gdctype = mb86290fb_pInfo->initparam.gdctype;
		GdcSystemInfo.tran_unit = MB86290FB_TRAN_UNIT;
		GdcSystemInfo.mapsize = MB86290FB_MAPSIZE;
		GdcSystemInfo.locate = mb86290fb_pInfo->initparam.locate;

		switch (mb86290fb_pInfo->initparam.cputype) {
		case MB86290FB_CPU_x86:
		/*case MB86290FB_CPU_MIPS: */
			GdcSystemInfo.host_reg = (volatile GDC_ULONG *)
			    (GdcSystemInfo.gdcbase + GDC_HOST_BASE);
			GdcSystemInfo.disp_reg = (volatile GDC_ULONG *)
			    (GdcSystemInfo.gdcbase + GDC_DISP_BASE);
			GdcSystemInfo.draw_reg = (volatile GDC_ULONG *)
			    (GdcSystemInfo.gdcbase + GDC_DRAW_BASE);
			GdcSystemInfo.geo_reg  = (volatile GDC_ULONG *)
			   (GdcSystemInfo.gdcbase + GDC_GEO_BASE);
			GdcSystemInfo.cap_reg = (volatile GDC_ULONG *)
			    (GdcSystemInfo.gdcbase + GDC_CAP_BASE);
			GdcSystemInfo.i2c_reg = (volatile GDC_ULONG *)
			    (GdcSystemInfo.gdcbase + GDC_I2C_BASE);
			break;
		}

		/* GdcSystemInfo for User */
		GdcSystemInfoUser.fd = MB86290FB_MAGIC_NUMBER;
		GdcSystemInfoUser.cputype = mb86290fb_pInfo->initparam.cputype;
		GdcSystemInfoUser.gdctype = mb86290fb_pInfo->initparam.gdctype;
		GdcSystemInfoUser.tran_unit = MB86290FB_TRAN_UNIT;
		GdcSystemInfoUser.mapsize = MB86290FB_MAPSIZE;
		GdcSystemInfoUser.locate = mb86290fb_pInfo->initparam.locate;

		switch (mb86290fb_pInfo->initparam.cputype) {
		case MB86290FB_CPU_x86:
		/*case MB86290FB_CPU_MIPS: */
			GdcSystemInfoUser.gdcbase = 0;
			GdcSystemInfoUser.host_reg = (volatile GDC_ULONG *)GDC_HOST_BASE;
			GdcSystemInfoUser.disp_reg = (volatile GDC_ULONG *)GDC_DISP_BASE;
			GdcSystemInfoUser.draw_reg = (volatile GDC_ULONG *)GDC_DRAW_BASE;
			GdcSystemInfoUser.geo_reg = (volatile GDC_ULONG *)GDC_GEO_BASE;
			GdcSystemInfoUser.cap_reg = (volatile GDC_ULONG *)GDC_CAP_BASE;
			GdcSystemInfoUser.i2c_reg = (volatile GDC_ULONG *)GDC_I2C_BASE;
			break;
		}
	} /* init_flag */
#if !defined(CONFIG_FB_PRE_INIT_FB)
	/* Video signal output is turned off */
	if (mb86290fb_pInfo->initparam.gdctype >= GDC_TYPE_MB86293) {
		/* for MB86293 later */
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_EXT_MODE, 0x0);
	} else {
		/* for MB86290A/86291/86291A/86292 */
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_MODE, 0x0);
	}
	if (mb86290fb_pInfo->initparam.gdctype >= GDC_TYPE_MB86293) {
		/* Sets internal clock */
		MB86290FB_WRITE_HOST_REGISTER((GDC_HOST_REG_INTERNAL_CLOCK << 2),
					      MB86290FB_CLOCK_PACK(mb86290fb_pInfo->initparam.geoclock,
								   mb86290fb_pInfo->initparam.otherclock));
		/* Waits for more than 200 microsec */
		Wait(1); /* 1msec */
	}

	/* Software reset */
	MB86290FB_WRITE_HOST_REGISTER((GDC_HOST_REG_SOFTWARE_RESET << 2), 1);
	Wait(1);

	/* Sets memory interface mode */
	MB86290FB_WRITE_HOST_REGISTER((GDC_HOST_REG_MEMORY_MODE << 2),
				      mb86290fb_pInfo->initparam.memorymode);
	Wait(1);
#endif
	if (init_flag
	    && (mb86290fb_pInfo->initparam.locate != GDC_REG_LOCATE_CENTER)
	    && (mb86290fb_pInfo->initparam.gdctype >= GDC_TYPE_MB86293)) {
		GDC_ULONG offset = 0x2000000 / 4;

		/* Sets register location */
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_REGISTER_LOCATION_SWITCH << 2,
					      mb86290fb_pInfo->initparam.locate);
		Wait(1);

		GdcSystemInfo.host_reg += offset;
		GdcSystemInfo.disp_reg += offset;
		GdcSystemInfo.draw_reg += offset;
		GdcSystemInfo.geo_reg += offset;
		GdcSystemInfo.cap_reg += offset;
		GdcSystemInfo.i2c_reg += offset;

		GdcSystemInfoUser.host_reg += offset;
		GdcSystemInfoUser.disp_reg += offset;
		GdcSystemInfoUser.draw_reg += offset;
		GdcSystemInfoUser.geo_reg += offset;
		GdcSystemInfoUser.cap_reg += offset;
		GdcSystemInfoUser.i2c_reg += offset;
	}
	/* Initializes the Graphics Controller */
	init_disp_param();
	if (GdcSystemInfo.gdctype == GDC_TYPE_MB86291A) {
		/* Initializes FIFO mode(for MB86291A) */
		MB86290FB_WRITE_HOST_REGISTER((GDC_HOST_REG_SID << 2),
					      GDC_FIFO_FULL_16 |
					      GDC_FIFO_NO_WAIT);
	}

	if (mb86290fb_pInfo->initparam.gdctype >= GDC_TYPE_MB86291 &&
	    mb86290fb_pInfo->initparam.gdctype != GDC_TYPE_MB86296) {
		/* Initializes the geometry engine */
		/* GdcGeoInitialize(); */
		int i;
		/* For MB86291A */
		struct {
			GDC_ULONG offset;
			GDC_ULONG data;
		} MB86291A_geo_firm[GDC_FIRM_NUM] = {
			{
			0x3fec, 0x1813}, {
			0x3fe8, 0x5618}, {
			0x3fe4, 0x1861}, {
			0x3fe0, 0xffff}, {
			0x3fdc, 0x19d6}, {
			0x3fd8, 0x1618}, {
			0x3fd4, 0x1a88}, {
			0x3fd0, 0xffff}, {
			0x3fcc, 0x0001}
		};
		struct {
			GDC_ULONG offset;
			GDC_ULONG data;
		} MB86291A_setup_firm[GDC_FIRM_NUM] = {
			{
			0x7fec, 0x053e}, {
			0x7fe8, 0x70de}, {
			0x7fe4, 0x0541}, {
			0x7fe0, 0x98ed}, {
			0x7fdc, 0x0546}, {
			0x7fd8, 0x70db}, {
			0x7fd4, 0x0549}, {
			0x7fd0, 0x98bd}, {
			0x7fcc, 0x0001}
		};
		/* For MB86292 */
		struct {
			GDC_ULONG offset;
			GDC_ULONG data;
		} MB86292_geo_firm[GDC_FIRM_NUM] = {
			{
			0x3fec, 0x1ae7}, {
			0x3fe8, 0x3ef2}, {
			0x3fe4, 0x1ae9}, {
			0x3fe0, 0xdb03}, {
			0x3fdc, 0x1aea}, {
			0x3fd8, 0xb612}, {
			0x3fd4, 0x1aeb}, {
			0x3fd0, 0xb612}, {
			0x3fcc, 0x0001}
		};
		struct {
			GDC_ULONG offset;
			GDC_ULONG data;
		} MB86292_setup_firm[GDC_FIRM_NUM] = {
			{
			0x7fec, 0x007e}, {
			0x7fe8, 0xbf00}, {
			0x7fe4, 0x007f}, {
			0x7fe0, 0x1410}, {
			0x7fdc, 0x00fe}, {
			0x7fd8, 0xbf00}, {
			0x7fd4, 0x00ff}, {
			0x7fd0, 0x1422}, {
			0x7fcc, 0x0001}
		};

		switch (GdcSystemInfo.gdctype) {
		case GDC_TYPE_MB86291A:
			/* For MB86291A */
			for (i = 0; i < GDC_FIRM_NUM; i++) {
				MB86290FB_WRITE_GEO_REGISTER(MB86291A_geo_firm[i].offset,
							     MB86291A_geo_firm[i].data);
				MB86290FB_WRITE_GEO_REGISTER(MB86291A_setup_firm[i].offset,
							     MB86291A_setup_firm[i].data);
			}
			break;
		case GDC_TYPE_MB86292:
			/* For MB86292 */
			for (i = 0; i < GDC_FIRM_NUM; i++) {
				MB86290FB_WRITE_GEO_REGISTER(MB86292_geo_firm[i].offset,
							     MB86292_geo_firm[i].data);
				MB86290FB_WRITE_GEO_REGISTER(MB86292_setup_firm[i].offset,
							     MB86292_setup_firm[i].data);
			}
			break;
		}

		/* Write G_Init command to FIFO */
		MB86290FB_WRITE_GEO_REGISTER(GDC_GEO_REG_INPUT_FIFO<<2, GEO_HEADER_INIT);
	}
#if !defined(CONFIG_FB_PRE_INIT_FB)
	/* Initializes display parameter */
	set_disp_clock(mb86290fb_pInfo->initparam.disp.mode);

	/* Set display timing parameter */
	set_disp_timing(mb86290fb_pInfo->initparam.disp.htp,
			mb86290fb_pInfo->initparam.disp.hsp,
			mb86290fb_pInfo->initparam.disp.hsw,
			mb86290fb_pInfo->initparam.disp.hdp,
			mb86290fb_pInfo->initparam.disp.vtr,
			mb86290fb_pInfo->initparam.disp.vsp,
			mb86290fb_pInfo->initparam.disp.vsw,
			mb86290fb_pInfo->initparam.disp.vdp);
#endif
	/* Set CBM(Caputure Buffer Mode) register */
	MB86290FB_WRITE_CAP_REGISTER(GDC_CAP_REG_BUFFER_MODE, 0);

	return 0;
}

int mb86290fb_ioctl_softwarereset(mb86290fb_drawman_t * drawman)
{
	if (initialize_gdc(0)) {
		printk(KERN_ALERT "MB86290: SoftwareReset failed.\n");
		return -EINVAL;
	}
	/* Set Interrupt Mask */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_INTERRUPT_MASK<<2,
				      MB86290FB_INTERRUPT_MASK);

	return 0;
}

/* mb86290fb_intr */
void mb86290fb_do_tasklet(unsigned long);
DECLARE_TASKLET(mb86290fb_tasklet, mb86290fb_do_tasklet, 0);

irqreturn_t mb86290fb_intr(int irq, void *dev_id)
{
	mb86290fb_info_t *info;
	GDC_ULONG reg_ist, reg_bst;

	/* Get Interrupt Status */
	reg_ist = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_INTERRUPT_STATUS << 2);
	reg_ist &= MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_INTERRUPT_MASK << 2);
	if (reg_ist == 0)
		return IRQ_HANDLED;

	/*
	 * Clear interrupt status
	 */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_INTERRUPT_STATUS << 2,
				      ~reg_ist);
	reg_bst = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_BST);

	info = (mb86290fb_info_t *) dev_id;

	if (reg_ist & GDC_HOST_REG_IST_MASK_VSYNC) {
		/*
		 * Set Layer ML,MR,BL,BR (L2 to L5)
		 */
		{
			mb86290fb_scrollinfoMB_t *mbinfo;
			mb86290fb_scrollparamMB_t *rp;
			unsigned long addr_LxDXY;
			unsigned long addr_LxDA;
			int i;

			addr_LxDXY = GDC_DISP_REG_ML_DISP_POS;
			addr_LxDA = GDC_DISP_REG_ML_DISP_ADR1;
			for (i = 0; i < 4; i++) {
				mbinfo = &info->scrollinfo.MB[i];
				rp = mbinfo->rp;
				if (rp->ready) {
					MB86290FB_WRITE_DISP_REGISTER(addr_LxDXY, rp->xy);
					MB86290FB_WRITE_DISP_REGISTER(addr_LxDA,
								      rp->disp_addr0);
					MB86290FB_WRITE_DISP_REGISTER(addr_LxDA + 8,
								      rp->disp_addr1);

					if (atomic_read(&mbinfo->waitcnt)) {
						wake_up_interruptible(&mbinfo->waitq);
					}
					rp->ready = 0;
				}
				addr_LxDXY += 0x18;
				addr_LxDA += 0x18;
			}
		}

		/*
		 * Set Layer C (L0)
		 */
		{
			mb86290fb_scrollinfoC_t *cinfo;
			mb86290fb_scrollparamC_t *rp;

			cinfo = &info->scrollinfo.C;
			rp = cinfo->rp;
			if (rp->ready) {
				MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_C_DISP_POS, rp->xy);
				MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_C_DISP_ADR, rp->disp_addr);
				if (atomic_read(&cinfo->waitcnt)) {
					wake_up_interruptible(&cinfo->waitq);
				}
				rp->ready = 0;
			}
		}

		/*
		 * Set Layer W (L1)
		 */
		{
			mb86290fb_scrollinfoW_t *winfo;
			mb86290fb_scrollparamW_t *rp;

			winfo = &info->scrollinfo.W;
			rp = winfo->rp;
			if (rp->ready) {
				MB86290FB_WRITE_DISP_REGISTER
				    (GDC_DISP_REG_W_ORG_ADR, rp->disp_addr);
				if (atomic_read(&winfo->waitcnt)) {
					wake_up_interruptible(&winfo->waitq);
				}
				rp->ready = 0;
			}
		}
	}

	if ((reg_ist & GDC_HOST_REG_IST_MASK_CEND)
	    || (reg_ist & GDC_HOST_REG_IST_MASK_TC)) {

		mb86290fb_drawqhead_t *curq, *lastq;
		mb86290fb_drawlist_t *old_rp, *new_rp;
		mb86290fb_eventflip_t *flip;
		unsigned long flag;
		int i;

		curq = info->drawman.cur_drawq;
		lastq = info->drawman.last_drawq;
		old_rp = lastq->rp;
		flag = old_rp->flag;

		/* check transfer mode */
		if ((reg_ist & GDC_HOST_REG_IST_MASK_TC)
		    && !(reg_ist & GDC_HOST_REG_IST_MASK_CEND)
		    && (old_rp->dst_addr == GDC_DL_FIFO)) {
			goto EXIT_CEND;
		}

		/* Flipping */
		for (i = 0; i < FLIPLAYER_NUM; i++) {
			flip = &info->eventinfo.flip[i];
			if (flip->condition & flag) {
				MB86290FB_DISP_DO_FLIP(flip->reg, flip->bank);
				flip->bank ^= 1;
			}
		}

		/* Set complete status */
		SET_DRAWING_COMPLETED(old_rp->complete);

		/* Update read pointer */
		inc_drawlistp(lastq->rp, lastq->top, lastq->bottom);

		/* Check request for suspend */
		if (IS_DRAWQUEUE_REQ_SUSPEND(info->drawman.flags)) {
			SET_DRAWQUEUE_SUSPEND(info->drawman.flags);
		} else {
			/* Flush display list */
			for (;;) {
				new_rp = curq->rp;
				if (new_rp == curq->wp) {
					SET_DRAWQUEUE_SUSPEND(info->drawman.flags);
					break;
				}
				if (IS_DRAWING_COMPLETED(new_rp->complete)) {
					inc_drawlistp(curq->rp, curq->top,
						      curq->bottom);
					continue;
				}

				info->drawman.last_drawq = info->drawman.cur_drawq;
				mb86290fb_transfer(new_rp);

				break;
			}
		}

		/* Wake up processes */
		if (atomic_read(&old_rp->waitcnt)) {
			wake_up_interruptible(&old_rp->waitq);
		}

		/* Update event flag */
		if (flag) {
			info->eventinfo.eventflag |= flag;
			if (atomic_read(&info->eventinfo.waitcnt)) {
				tasklet_schedule(&mb86290fb_tasklet);
			}
		}
	}
EXIT_CEND:

	/* serial */
	if (reg_ist & GDC_HOST_REG_IST_MASK_SII) {
		info->serial.data_arrive = 1;
		wake_up_interruptible(&info->serial.waitq);
	}

	/* GPIO */
	if (reg_ist & GDC_HOST_REG_IST_MASK_GI) {
		info->gpio.data_read = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_GD);
		wake_up_interruptible(&info->gpio.waitq);
	}

	/* CERR or SYNCERR */
	if ((reg_ist & GDC_HOST_REG_IST_MASK_CERR)
	    || (reg_ist & GDC_HOST_REG_IST_MASK_SYNCERR)) {
		PDEBUG("IH: CERR or SYNCERR occured.\n");
		mb86290fb_sendsignal();
	}

	return IRQ_HANDLED;
}

void mb86290fb_do_tasklet(unsigned long unused)
{
	mb86290fb_eventwakeup(&mb86290fb_pInfo->eventinfo);
}

int mb86290fb_initialize_gdc(void)
{
	int ret;
	GDC_ULONG reg_bst;

	/* Initialize driver information */
	initialize_mb86290fb_info(mb86290fb_pInfo);

	if (initialize_gdc(1)) {
		printk(KERN_ALERT "MB86290: initialize_gdc failed.\n");
		return -EINVAL;
	}

	/* Initialize drawman */
	ret = mb86290fb_initdrawman(&mb86290fb_pInfo->drawman);
	if (ret == -1) {
		PDEBUG("Drawman initialize failed\n");
		return -ENODEV;
	}

	/* Initialize eventinfo */
	ret = mb86290fb_initeventinfo(&mb86290fb_pInfo->eventinfo);
	if (ret != 0) {
		PDEBUG("Eventinfo initialize failed\n");
		return -ENODEV;
	}

	/* Initialize scrollinfo */
	mb86290fb_initscrollinfo(&mb86290fb_pInfo->scrollinfo);

	/* Initialize capture info */
	mb86290fb_initcapinfo(&mb86290fb_pInfo->capinfo);

	/* Initialize Ext. pins info */
	mb86290fb_initextpinsinfo(&mb86290fb_pInfo->extpinsinfo);

	/* Initialize VRAM management info */
	mb86290fb_pInfo->vram_ctl = (mb86290fb_memman_t *) mb86290fb_pInfo->vram_ctlarea;
	mb86290fb_memman_init(mb86290fb_pInfo->vram_ctl,
			      MB86290FB_VRAM_MAXSEGMENT,
			      (void *)MB86290FB_VRAM_DUMMY_BASE,
			      MB86290FB_VRAM_SIZE);

	/* Init waiting resource */
	init_waitqueue_head(&mb86290fb_pInfo->serial.waitq);
	init_waitqueue_head(&mb86290fb_pInfo->gpio.waitq);
	mb86290fb_pInfo->serial.data_arrive = 1;
	mb86290fb_pInfo->gpio.data_arrive = 1;

	/* Init semaphore */
	sema_init(&mb86290fb_pInfo->sem[MB86290FB_SEM_LAYER], 1);
	sema_init(&mb86290fb_pInfo->sem[MB86290FB_SEM_L1], 1);
	sema_init(&mb86290fb_pInfo->sem[MB86290FB_SEM_CURSOR], 1);
	sema_init(&mb86290fb_pInfo->sem[MB86290FB_SEM_GPIO], 1);

	/* Register interrupt routine */
	ret = request_irq(mb86290fb_pInfo->irq, mb86290fb_intr,
			  IRQF_DISABLED | IRQF_SHARED,
			  MB86290FB_DRIVER_NAME, (void *)mb86290fb_pInfo);
	if (ret) {
		printk(KERN_ALERT "request_irq failed: %d\n", ret);
		return -ENODEV;
	}

	/* Clear interrupt status */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_INTERRUPT_STATUS << 2, 0);
	reg_bst = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_BST);

	/* Set interrupt mask */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_INTERRUPT_MASK << 2,
				      MB86290FB_INTERRUPT_MASK);

#if defined CONFIG_MEDIA5200
	/* Enable LCD output */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_IOM, 0x00001280);
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_GD, 0x00000005 | 0x00250000);
#endif
#if defined(CONFIG_LWMON5) && !defined(CONFIG_FB_PRE_INIT_FB)
	enable_backlight();
#endif
	printk(KERN_INFO "Fujitsu MB8629x driver init complete\n");
	PDEBUG("o 18 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_WIN_POS) );
	PDEBUG("o 1C 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_WIN_SIZE) );
	PDEBUG("o 20 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_MODE_W_H) );
	PDEBUG("o 24 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_ORG_ADR) );
	PDEBUG("o 28 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_DISP_ADR) );
	PDEBUG("o 2C 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_DISP_POS) );
	PDEBUG("o 110 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_EXT_MODE) );
	PDEBUG("o 114 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_WIN_POS) );
	PDEBUG("o 118 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_WIN_SIZE) );
	return 0;
}

#if 0	/* FIXME */
static int get_gdc_addr_pci(unsigned long *addr_phys)
{
	int ret;
	struct pci_dev *dev;
	unsigned long intcsr;

	/* It is not necessary to call pci_present() in kernel 2.4. */

	/* Search PCI device */
	dev = pci_find_device(MB86290FB_DFLT_PCI_VENDOR_ID,
			      MB86290FB_DFLT_PCI_DEVICE_ID, 0);
	if (dev == 0) {
		PDEBUG("pci_find_device failed(%x, %x).\n",
		       MB86290FB_DFLT_PCI_VENDOR_ID,
		       MB86290FB_DFLT_PCI_DEVICE_ID);
		return -ENODEV;
	}
#if defined(CONFIG_MIPS) & defined(CONFIG_MIPS_SNSC_MPU210)
	{
		u32 base = 0;
		pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &base);
		if (!request_mem_region(base, 0x04000000, "mb86290")) {
			base = 0x0c000000;
			if (!request_mem_region(base, 0x04000000, "mb86290")) {
				printk(KERN_INFO "request_mem_region error\n");
				return -ENODEV;
			}
			pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, base);
			dev->resource[MB86290FB_PCI_BASEADDRESS].start = base;
			dev->resource[MB86290FB_PCI_BASEADDRESS].end =
			    base + 0x03ffffff;
		}
	}
#endif

	pci_enable_device(dev);

	*addr_phys = dev->resource[MB86290FB_PCI_BASEADDRESS].start;

	/* Read IRQ */
	ret =
	    pci_read_config_byte(dev, PCI_INTERRUPT_LINE,
				 &mb86290fb_pInfo->irq);
	if (ret) {
		printk(KERN_ALERT "[MB86290FB] Cannot read IRQ\n");
		return -ENODEV;
	}

	if (mb86290fb_pInfo->initparam.gdctype != GDC_TYPE_MB86295) {
		/* Enable interrupt on FPGA */
		mb86290fb_pInfo->fpga_base =
		    (unsigned long *)ioremap(*addr_phys + 0x04001000, 0x1000);
		*(mb86290fb_pInfo->fpga_base + 0x4 / 4) = 0x10;

		/* Enable interrupt on PLX */
		mb86290fb_pInfo->plx_base_phys = dev->resource[0].start;
		mb86290fb_pInfo->plx_base =
		    (unsigned long *)ioremap(mb86290fb_pInfo->plx_base_phys,
					     0x100);
		intcsr = *(mb86290fb_pInfo->plx_base + 0x68 / 4);
		intcsr |= 0x800;
		*(mb86290fb_pInfo->plx_base + 0x68 / 4) = intcsr;
	}

	return 0;
}
#endif
