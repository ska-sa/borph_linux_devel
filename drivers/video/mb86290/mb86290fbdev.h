/*
 *	mb86290fbdev.h	--  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#ifndef _MB86290FBDEV_H_
#define _MB86290FBDEV_H_

#include <linux/fb.h>
#include <linux/interrupt.h>
#include <asm/semaphore.h>

#include "gdctypes.h"
#include "gdcreg.h"
#include "gdcdef.h"
#include "mb86290fb.h"
#include "mb86290fb_disp.h"
#include "mb86290fb_dma.h"
#include "mb86290fb_event.h"
#include "mb86290fb_memman.h"
#include "mb86290fb_extpins.h"

/* user defined parameter */
#include "mb86290fbsys.h"

/* Debug Print */
/* #define	MB86290FB_DEBUG */
#undef PDEBUG
#ifdef MB86290FB_DEBUG
#  ifdef __KERNEL__
#    define PDEBUG(fmt, args...) printk(KERN_ALERT "MB86290: " fmt, ## args)
#  else
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...)
#endif

/* System Information */
extern struct mb86290fb_info *mb86290fb_pInfo;
#define	GdcSystemInfo		(mb86290fb_pInfo->gdcsysinfo)
#define	GdcSystemInfoUser	(mb86290fb_pInfo->gdcsysinfo_user)

/* Initial Parmeters */
#define	MB86290FB_DRIVER_NAME			"MB86290"

#define	MB86290FB_MAX_CARD			1
#define	MB86290FB_CPU_x86			GDC_CPU_WINDOWS
#define	MB86290FB_CPU_MIPS			GDC_CPU_WINDOWS
#define MB86290FB_PCI_VENDOR_ID_PLX		0x10b5
#define MB86290FB_PCI_VENDOR_ID_FUJITSU		0x10CF
#define MB86290FB_PCI_DEVICE_ID_MB86292EV	0x9080
#define MB86290FB_PCI_DEVICE_ID_MB86295		0x201e

#define	MB86290FB_MAPSIZE_16MB			0x01000000
#define	MB86290FB_MAPSIZE_32MB			0x02000000
#define	MB86290FB_MAPSIZE_64MB			0x04000000

#define MB86290FB_TRUE			1
#define MB86290FB_FALSE			0

/* Graphics controller type */
#define MB86290FB_TYPE_MB86290A		0
#define MB86290FB_TYPE_MB86291		10
#define MB86290FB_TYPE_MB86291A		11
#define MB86290FB_TYPE_MB86292		20
#define MB86290FB_TYPE_MB86293		30
#define MB86290FB_TYPE_MB86294		40
#define MB86290FB_TYPE_MB86295		50
#define MB86290FB_TYPE_MB86296		60

#define MB86290FB_REG_LOCATE_CENTER	0x00000000L
#define MB86290FB_REG_LOCATE_BOTTOM	0x00000001L
#define MB86290FB_CLOCK_100MHZ		0
#define MB86290FB_CLOCK_133MHZ		1
#define MB86290FB_CLOCK_166MHZ		2

#if MB86290FB_DFLT_IP_LOCATE == MB86290FB_REG_LOCATE_CENTER
#define	MB86290FB_MAPSIZE		MB86290FB_MAPSIZE_32MB
#else
#define	MB86290FB_MAPSIZE		MB86290FB_MAPSIZE_64MB
#endif

/* packing */
#define MB86290FB_XY_PACK(y, x)          (((y) << 16) | (x))
#define MB86290FB_HI_WORD(data)          ((data) >> 16)
#define MB86290FB_LO_WORD(data)          ((data) & 0x0000ffffL)
#define MB86290FB_CLOCK_PACK(geo, other)	\
	( (((geo) << 18) & 0x000c0000) | (((other) << 16) & 0x00030000) )

/* VRAM dummy base address (for memman routines) */
#define MB86290FB_VRAM_DUMMY_BASE	0x10000000

/* Default Interrupt Mask */
#define	MB86290FB_INTERRUPT_MASK	(GDC_HOST_REG_IST_MASK_CERR|	\
					GDC_HOST_REG_IST_MASK_CEND|	\
					GDC_HOST_REG_IST_MASK_SYNCERR|	\
					GDC_HOST_REG_IST_MASK_TC)

/* Initialize parameter table */
typedef struct {
	unsigned long cputype;		/* CPU type                     */
	unsigned long gdctype;		/* GDC type                     */
	unsigned long long gdcbase;	/* GDC mapping address          */
	unsigned long geoclock;		/* Geometry clock               */
	unsigned long otherclock;	/* Other clock                  */
	unsigned long locate;		/* GDC register location        */
	unsigned long memorymode;	/* Memory interface mode        */
	struct {
		unsigned long mode;	/* Display clock mode           */
		unsigned long htp;	/* Horizontal Total Pixels      */
		unsigned long hsp;	/* Horizontal Synchronize pulse
					   Position */
		unsigned long hsw;	/* Horizontal Synchronize pulse
					   Width */
		unsigned long hdp;	/* Horizontal Display Period    */
		unsigned long vtr;	/* Vertical Total Rasters       */
		unsigned long vsp;	/* Vertical Synchronize pulse
					   Position */
		unsigned long vsw;	/* Vertical Synchronize pulse
					   Width */
		unsigned long vdp;	/* Vertical Display Period      */
	} disp;
} MB86290FB_INITPARAM;

typedef struct {
	unsigned long using;
	pid_t pid_using;
} mb86290fb_capinfo_t;

/* mb86290fb information */
#define DMAMEM_CTLAREA_SIZE	\
	MB86290FB_MEMMAN_SIZE(MB86290FB_DMAMEM_MAXSEGMENT)/4

#define VRAM_CTLAREA_SIZE	\
	MB86290FB_MEMMAN_SIZE(MB86290FB_VRAM_MAXSEGMENT)/4

#define MB86290FB_SEM_NUM	4

typedef struct mb86290fb_info {
	unsigned long		base_phys;
	unsigned long		plx_base_phys;
	unsigned long		*plx_base;
	unsigned long		*fpga_base;
	unsigned char		*dmamem;
	unsigned long		dmamem_size;
	unsigned long		dmamem_ctlarea[DMAMEM_CTLAREA_SIZE];
	mb86290fb_memman_t	*dmamem_ctl;

	unsigned long		vram_ctlarea[VRAM_CTLAREA_SIZE];
	mb86290fb_memman_t	*vram_ctl;

	MB86290FB_INITPARAM	initparam;
	GDC_SYSTEMINFO		gdcsysinfo;
	GDC_SYSTEMINFO		gdcsysinfo_user;
	GDC_DISPPARAM		gdcdispparam;

	mb86290fb_drawman_t	drawman;
	mb86290fb_eventinfo_t	eventinfo;
	mb86290fb_scrollinfo_t	scrollinfo;
	mb86290fb_capinfo_t	capinfo;
	mb86290fb_extpinsinfo_t extpinsinfo;

	u8			irq;
	int			intr_count;

	int			currcon;
	struct {
		unsigned char red;
		unsigned char green;
		unsigned char blue;
	} palette[256];
	unsigned short cmap[16];

	struct fb_info		fb_info;
	struct fb_ops		ops;
	struct fb_fix_screeninfo display;
	struct fb_var_screeninfo screeninfo;

	struct {
		int data_arrive;
		int data_read;
		int mask;
		int pattern;
		int logic;
		wait_queue_head_t waitq;
	} serial, gpio;

	struct semaphore sem[MB86290FB_SEM_NUM];
				/* [0]: DCE/DCEE */
				/* [1]: CPM      */
} mb86290fb_info_t;

#ifdef CONFIG_LWMON5
static inline __u32 gdc_readl(const volatile void __iomem *addr)
{
        return *(__force volatile __u32 *)(addr);
}
static inline void gdc_writel(__u32 b, volatile void __iomem *addr)
{
        *(__force volatile __u32 *)(addr) = b;
}
#else
#define gdc_readl readl
#define gdc_writel writel
#endif

/* Register access function */
#define	MB86290FB_WRITE_REG(offset, data)	\
{						\
	gdc_writel((data), (volatile void __iomem *) \
			   (mb86290fb_pInfo->gdcsysinfo.gdcbase+(offset))); \
}
#define MB86290FB_WRITE_REGISTER(reg, data)				\
{									\
    gdc_writel((data), (volatile GDC_ULONG*)				\
		       (mb86290fb_pInfo->gdcsysinfo.gdcbase + (reg)));	\
}
#define MB86290FB_WRITE_HOST_REGISTER(reg, data)			\
{									\
    gdc_writel((data), (mb86290fb_pInfo->gdcsysinfo.host_reg + ((reg)>>2))); \
}
#define MB86290FB_WRITE_DISP_REGISTER(reg, data)			\
{									\
    gdc_writel((data), (mb86290fb_pInfo->gdcsysinfo.disp_reg + ((reg)>>2))); \
}
#define MB86290FB_WRITE_DRAW_REGISTER(reg, data)			\
{									\
    gdc_writel((data), (mb86290fb_pInfo->gdcsysinfo.draw_reg + ((reg)>>2))); \
}
#define MB86290FB_WRITE_GEO_REGISTER(reg, data)				\
{									\
    gdc_writel((data), (mb86290fb_pInfo->gdcsysinfo.geo_reg + ((reg)>>2))); \
}
#define MB86290FB_WRITE_CAP_REGISTER(reg, data)				\
{									\
    gdc_writel((data), (mb86290fb_pInfo->gdcsysinfo.cap_reg + ((reg)>>2))); \
}
#define MB86290FB_WRITE_I2C_REGISTER(reg, data)				\
{									\
    gdc_writel((data), (mb86290fb_pInfo->gdcsysinfo.i2c_reg + ((reg)>>2))); \
}

#define	MB86290FB_READ_REG(offset)	\
		((unsigned long)gdc_readl((const volatile void __iomem *) \
					  (mb86290fb_pInfo->gdcsysinfo.gdcbase+(offset))))
#define MB86290FB_READ_REGISTER(reg)		\
		((unsigned long)gdc_readl((volatile GDC_ULONG*)	\
					  (mb86290fb_pInfo->gdcsysinfo.gdcbase + (reg))) )
#define MB86290FB_READ_HOST_REGISTER(reg)	\
		((unsigned long)gdc_readl(mb86290fb_pInfo->gdcsysinfo.host_reg + ((reg)>>2)))
#define MB86290FB_READ_DISP_REGISTER(reg)	\
		((unsigned long)gdc_readl(mb86290fb_pInfo->gdcsysinfo.disp_reg + ((reg)>>2)))
#define MB86290FB_READ_DRAW_REGISTER(reg)	\
		((unsigned long)gdc_readl(mb86290fb_pInfo->gdcsysinfo.draw_reg + ((reg)>>2)))
#define MB86290FB_READ_GEO_REGISTER(reg)	\
		((unsigned long)gdc_readl(mb86290fb_pInfo->gdcsysinfo.geo_reg  + ((reg)>>2)))
#define MB86290FB_READ_CAP_REGISTER(reg)	\
		((unsigned long)gdc_readl(mb86290fb_pInfo->gdcsysinfo.cap_reg  + ((reg)>>2)))
#define MB86290FB_READ_I2C_REGISTER(reg)	\
		((unsigned long)gdc_readl(mb86290fb_pInfo->gdcsysinfo.i2c_reg  + ((reg)>>2)))

/* frame buffer methods */
int mb86290fb_get_fix(struct fb_fix_screeninfo *fix,
		      int con, struct fb_info *info);
int mb86290fb_get_var(struct fb_var_screeninfo *var,
		      int con, struct fb_info *info);
int mb86290fb_set_var(struct fb_var_screeninfo *var,
		      int con, struct fb_info *info);
int mb86290fb_get_cmap(struct fb_cmap *cmap,
		       int kspc, int con, struct fb_info *info);
int mb86290fb_set_cmap(struct fb_cmap *cmap,
		       int kspc, int con, struct fb_info *info);
int mb86290fb_pan_display(struct fb_var_screeninfo *var,
			  int con, struct fb_info *info);
int mb86290fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
int mb86290fb_mmap(struct fb_info *info, struct vm_area_struct *vma);

int mb86290fb_update_var(int con, struct fb_info *info);
int mb86290fb_switch_con(int con, struct fb_info *info);
int mb86290fb_blank(int blank, struct fb_info *info);

/* ioctl */
int mb86290fb_ioctl_getsem(mb86290fb_info_t * pinfo, unsigned long arg);
int mb86290fb_ioctl_ungetsem(mb86290fb_info_t * pinfo, unsigned long arg);
int mb86290fb_ioctl_read_register(mb86290fb_drawman_t * drawman,
				  unsigned long arg);
int mb86290fb_ioctl_write_register(mb86290fb_drawman_t * drawman,
				   unsigned long arg);
int mb86290fb_ioctl_getsysinfo(unsigned long arg);
int mb86290fb_ioctl_data_trans(mb86290fb_drawman_t * drawman,
			       unsigned long arg);
int mb86290fb_ioctl_freedmamemory(mb86290fb_drawman_t * drawman,
				  unsigned long arg);
int mb86290fb_ioctl_gensignal(mb86290fb_drawman_t * drawman, unsigned long arg);
int mb86290fb_ioctl_stopsignal(mb86290fb_drawman_t * drawman,
			       unsigned long arg);
int mb86290fb_ioctl_softwarereset(mb86290fb_drawman_t * drawman);
int mb86290fb_ioctl_setframesize(GDC_DISPPARAM * dispparam, unsigned long arg);
int mb86290fb_ioctl_sioreadwrite(mb86290fb_info_t * pinfo, unsigned long arg);
int mb86290fb_ioctl_waitgpio(mb86290fb_info_t * pinfo, unsigned long arg);
int mb86290fb_ioctl_setbit(mb86290fb_info_t * pinfo, unsigned long arg);
int mb86290fb_ioctl_clearbit(mb86290fb_info_t * pinfo, unsigned long arg);

/* internal function */
int mb86290fb_initialize_gdc(void);
int mb86290fb_memory_mapping(struct vm_area_struct *vma);
void mb86290fb_transfer(mb86290fb_drawlist_t * dlist);
irqreturn_t mb86290fb_intr(int irq, void *dev_id);

/* allocate dma memory */
int mb86290fb_dmamem_allocate(unsigned char **addr, unsigned long *size);
void mb86290fb_dmamem_free(unsigned char *addr);
int mb86290fb_dmamem_memmap(struct vm_area_struct *vma);
int mb86290fb_dmamem_munmap(unsigned long addr);
unsigned long mb86290fb_dmamem_user_to_bus(unsigned long user_addr);

/* signal */
int mb86290fb_sendsignal(void);

void mb86290fb_initcapinfo(mb86290fb_capinfo_t * capinfo);
int mb86290fb_ioctl_getcapture(mb86290fb_capinfo_t * capinfo);
int mb86290fb_ioctl_releasecapture(mb86290fb_capinfo_t * capinfo);

int mb86290fb_ioctl_read_disp_register(unsigned long arg);
int mb86290fb_ioctl_write_disp_register(unsigned long arg);
int mb86290fb_ioctl_read_cap_register(unsigned long arg);
int mb86290fb_ioctl_write_cap_register(unsigned long arg);
int mb86290fb_ioctl_read_geo_register(unsigned long arg);
int mb86290fb_ioctl_write_geo_register(unsigned long arg);
int mb86290fb_ioctl_read_draw_register(unsigned long arg);
int mb86290fb_ioctl_write_draw_register(unsigned long arg);

#endif /*_MB86290FBDEV_H_*/
