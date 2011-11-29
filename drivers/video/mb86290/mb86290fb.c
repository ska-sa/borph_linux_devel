/*
 * drivers/video/mb86290fb.c - driver for MB86290 Logic chipsets
 */

#define MB86290FB_VERSION "0.9"

//#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/selection.h>
#include <linux/interrupt.h>
#include <asm/pgtable.h>

#include <linux/pci.h>

/*
 * debugging and utility macros
 */

/* enable debug output? */
/* #define MB86290FB_DEBUG 1 */

/* disable runtime assertions? */
/* #define MB86290FB_NDEBUG */

/* debug output */
#ifdef MB86290FB_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

/* debugging assertions */
#ifndef MB86290FB_NDEBUG
#define assert(expr) \
        if(!(expr)) { \
        printk( "Assertion failed! %s,%s,%s,line=%d\n",\
        #expr,__FILE__,__FUNCTION__,__LINE__); \
        }
#else
#define assert(expr)
#endif

#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif
#define TRUE  1
#define FALSE 0

#define MB_ (1024*1024)
#define KB_ (1024)

#define MAX_NUM_BOARDS 7

#include "mb86290fbdev.h"

#ifdef MB86290FB_USE_PCI
/*
 * chipset information
 */

/* board types */
typedef enum {
	BT_NONE = 0,
	BT_CORALP,
	BT_CORALPA,
} mb86290fb_board_t;

static struct pci_device_id mb86290fb_pci_table[] = {
	{0x10cf, 0x2019, PCI_ANY_ID, PCI_ANY_ID, 0, 0, BT_CORALP},
	{0x10cf, 0x201e, PCI_ANY_ID, PCI_ANY_ID, 0, 0, BT_CORALPA},
	{0,}
};

MODULE_DEVICE_TABLE(pci, mb86290fb_pci_table);
#endif


void mb86290fb_initdispparam_console(void);

static u32 mb86290fb_pseudo_palette[256];

static int mb86290fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	/* PDEBUG("mb86290fb_check_par\n"); */
	return 0;
}

static int mb86290fb_set_par(struct fb_info *info)
{
	mb86290fb_initdispparam_console();
	return 0;
}

static int mb86290fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			       unsigned blue, unsigned transp,
			       struct fb_info *info)
{
	if (regno > 255)
		return 1;
	red >>= 8;
	green >>= 8;
	blue >>= 8;

	if (regno < 16) {
		mb86290fb_pseudo_palette[regno] = ((red & 0xf8) << 7) |
						  ((green & 0xf8) << 2) |
						  ((blue & 0xf8) >> 3);
	}
	return 0;
}

/* function table of the above functions */
static struct fb_ops mb86290fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mb86290fb_check_var,
	.fb_set_par = mb86290fb_set_par,
	.fb_setcolreg = mb86290fb_setcolreg,
	.fb_blank = mb86290fb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

/* initialize fb data */
static void set_fb_var_screeninfo(void)
{
	struct fb_info *pfb_info = &mb86290fb_pInfo->fb_info;
#ifdef CONFIG_FB_MB86290_640X480_16BPP
	pfb_info->var.xres = 640;
	pfb_info->var.yres = 480;
	pfb_info->var.xres_virtual = 640;
	pfb_info->var.yres_virtual = 480;
#else
	pfb_info->var.xres = 1024;
	pfb_info->var.yres = 768;
	pfb_info->var.xres_virtual = 1024;
	pfb_info->var.yres_virtual = 768;
#endif
	pfb_info->var.xoffset = 0;
	pfb_info->var.yoffset = 0;
	pfb_info->var.bits_per_pixel = 16;
	pfb_info->var.grayscale = 0;
	pfb_info->var.red.offset = 10;
	pfb_info->var.red.length = 5;
	pfb_info->var.red.msb_right = 0;
	pfb_info->var.green.offset = 5;
	pfb_info->var.green.length = 5;
	pfb_info->var.green.msb_right = 0;
	pfb_info->var.blue.offset = 0;
	pfb_info->var.blue.length = 5;
	pfb_info->var.blue.msb_right = 0;
	pfb_info->var.nonstd = 0;
	pfb_info->var.activate = FB_ACTIVATE_TEST;
	pfb_info->var.height = -1;
	pfb_info->var.width = -1;
	pfb_info->var.pixclock = 20000;
	pfb_info->var.left_margin = 64;
	pfb_info->var.right_margin = 64;
	pfb_info->var.upper_margin = 32;
	pfb_info->var.lower_margin = 32;
	pfb_info->var.hsync_len = 64;
	pfb_info->var.vsync_len = 2;
	pfb_info->var.vmode = FB_VMODE_NONINTERLACED;
};

static void set_fb_fix_screeninfo(void)
{
	struct fb_info *pfb_info = &mb86290fb_pInfo->fb_info;
	strcpy(pfb_info->fix.id, "mb86290fb");
	pfb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	pfb_info->fix.visual = FB_VISUAL_TRUECOLOR;
	pfb_info->fix.xpanstep = 0;
	pfb_info->fix.ypanstep = 0;
	pfb_info->fix.ywrapstep = 0;
#ifdef CONFIG_FB_MB86290_640X480_16BPP
	pfb_info->fix.line_length = 640 * 2;
#else
	pfb_info->fix.line_length = 1024 * 2;
#endif
	pfb_info->fix.accel = FB_ACCEL_NONE;
	pfb_info->fix.smem_start = mb86290fb_pInfo->base_phys;
	pfb_info->fix.smem_len = MB86290FB_MAPSIZE;
};

static void set_fb_info(void)
{
	struct fb_info *pfb_info = &mb86290fb_pInfo->fb_info;

	pfb_info->flags = FBINFO_DEFAULT;
	pfb_info->screen_base = (char __iomem *)mb86290fb_pInfo->gdcsysinfo.gdcbase;
	pfb_info->screen_size = MB86290FB_MAPSIZE;
	pfb_info->fbops = &mb86290fb_ops;
	pfb_info->pseudo_palette = &mb86290fb_pseudo_palette;
	fb_alloc_cmap(&pfb_info->cmap, 256, 0);
}

int mb86290fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	GDC_ULONG *parg;

	int err = 0;

	parg = (GDC_ULONG *) arg;

	switch (cmd) {
	case FBIO_MB86290_GETSEM:
		err = mb86290fb_ioctl_getsem(mb86290fb_pInfo, arg);
		break;

	case FBIO_MB86290_UNGETSEM:
		err = mb86290fb_ioctl_ungetsem(mb86290fb_pInfo, arg);
		break;

	case FBIO_MB86290_DMATRANSFER:
		err = mb86290fb_ioctl_dmatransfer(&mb86290fb_pInfo->drawman, arg);
		break;

	case FBIO_MB86290_GETSYSINFO:
		err = mb86290fb_ioctl_getsysinfo(arg);
		break;

	case FBIO_MB86290_WAITDRAWCOMPLETE:
		err = mb86290fb_ioctl_waitdrawcomplete(&mb86290fb_pInfo->drawman,
						       arg);
		break;

	case FBIO_MB86290_POLLDRAWCOMPLETE:
		err =  mb86290fb_ioctl_polldrawcomplete(&mb86290fb_pInfo->drawman,
							arg);
		break;

	case FBIO_MB86290_RESETDRAWQUEUE:
		err = mb86290fb_ioctl_resetdrawqueue(&mb86290fb_pInfo->drawman);
		break;

	case FBIO_MB86290_CANCELDRAW:
		err = mb86290fb_ioctl_canceldraw(&mb86290fb_pInfo->drawman, arg);
		break;

	case FBIO_MB86290_CANCELDRAWALL:
		err = mb86290fb_ioctl_canceldrawall(&mb86290fb_pInfo->drawman);
		break;

	case FBIO_MB86290_ADDDRAWQUEUE:
		err = mb86290fb_ioctl_adddrawqueue(&mb86290fb_pInfo->drawman,
						   arg);
		break;

	case FBIO_MB86290_SWITCHDRAWQUEUE:
		err = mb86290fb_ioctl_switchdrawqueue(&mb86290fb_pInfo->drawman,
						      arg);
		break;

	case FBIO_MB86290_SUSPENDDRAWQUEUE:
		err = mb86290fb_ioctl_suspenddrawqueue(&mb86290fb_pInfo->drawman);
		break;

	case FBIO_MB86290_GENSIGNAL:
		err = mb86290fb_ioctl_gensignal(&mb86290fb_pInfo->drawman, arg);
		break;

	case FBIO_MB86290_STOPSIGNAL:
		err = mb86290fb_ioctl_stopsignal(&mb86290fb_pInfo->drawman, arg);
		break;

	case FBIO_MB86290_SOFTWARERESET:
		err = mb86290fb_ioctl_softwarereset(&mb86290fb_pInfo->drawman);
		break;

	case FBIO_MB86290_FREEDMAMEMORY:
		err = mb86290fb_ioctl_freedmamemory(&mb86290fb_pInfo->drawman,
						    arg);
		break;

	case FBIO_MB86290_GETDISPINFO:
		err = mb86290fb_ioctl_getdispinfo(&mb86290fb_pInfo->drawman, arg);
		break;

	case FBIO_MB86290_DISPFLIPAUTOF:
		err = mb86290fb_ioctl_dispflipautof(&mb86290fb_pInfo->eventinfo,
						    arg);
		break;

	case FBIO_MB86290_DISPPOSSYNC:
		err = mb86290fb_ioctl_disppossync(&mb86290fb_pInfo->scrollinfo,
						  arg);
		break;

	case FBIO_MB86290_SETFRAMESIZE:
		err = mb86290fb_ioctl_setframesize(&mb86290fb_pInfo->gdcdispparam,
						   arg);
		break;

	case FBIO_MB86290_ALLOCVRAM:
		err = mb86290fb_ioctl_allocvram(mb86290fb_pInfo->vram_ctl, arg);
		break;

	case FBIO_MB86290_FREEVRAM:
		err = mb86290fb_ioctl_freevram(mb86290fb_pInfo->vram_ctl, arg);
		break;

	case FBIO_MB86290_SETFLAG:
		err = mb86290fb_ioctl_setflag(&mb86290fb_pInfo->eventinfo, arg);
		break;

	case FBIO_MB86290_CLEARFLAG:
		err = mb86290fb_ioctl_clearflag(&mb86290fb_pInfo->eventinfo, arg);
		break;

	case FBIO_MB86290_WAITFLAG:
		err = mb86290fb_ioctl_waitflag(&mb86290fb_pInfo->eventinfo, arg);
		break;

	case FBIO_MB86290_READ_REGISTER:
		err = mb86290fb_ioctl_read_register(&mb86290fb_pInfo->drawman,
						    arg);
		break;

	case FBIO_MB86290_WRITE_REGISTER:
		err = mb86290fb_ioctl_write_register(&mb86290fb_pInfo->drawman,
						     arg);
		break;

	case FBIO_MB86290_READDRAWMAN:
		err = mb86290fb_ioctl_readdrawman(&mb86290fb_pInfo->drawman, arg);
		break;

	case FBIO_MB86290_READDRAWHEAD:
		err = mb86290fb_ioctl_readdrawqueuehead(&mb86290fb_pInfo->drawman,
							arg);
		break;

	case FBIO_MB86290_READDRAWLIST:
		err = mb86290fb_ioctl_readdrawlist(&mb86290fb_pInfo->drawman,
						   arg);
		break;

	case FBIO_MB86290_ENABLELDMA:
		err = mb86290fb_ldma_enable();
		break;

	case FBIO_MB86290_DISABLELDMA:
		err = mb86290fb_ldma_disable();
		break;

	case FBIO_MB86290_GETEXTPINS:
		err = mb86290fb_ioctl_getextpins(&mb86290fb_pInfo->extpinsinfo,
						 arg);
		break;

	case FBIO_MB86290_RELEASEEXTPINS:
		err = mb86290fb_ioctl_releaseextpins(&mb86290fb_pInfo->extpinsinfo,
						     arg);
		break;

	case FBIO_MB86290_GETCAPTURE:
		err = mb86290fb_ioctl_getcapture(&mb86290fb_pInfo->capinfo);
		break;

	case FBIO_MB86290_RELEASECAPTURE:
		err = mb86290fb_ioctl_releasecapture(&mb86290fb_pInfo->capinfo);
		break;

	case FBIO_MB86290_SIOREADWRITE:
		err = mb86290fb_ioctl_sioreadwrite(mb86290fb_pInfo, arg);
		break;

	case FBIO_MB86290_WAITGPIO:
		err = mb86290fb_ioctl_waitgpio(mb86290fb_pInfo, arg);
		break;

	case FBIO_MB86290_SETBIT:
		err = mb86290fb_ioctl_setbit(mb86290fb_pInfo, arg);
		break;

	case FBIO_MB86290_CLEARBIT:
		err = mb86290fb_ioctl_clearbit(mb86290fb_pInfo, arg);
		break;

	case FBIO_MB86290_READ_DISP_REG:
		err = mb86290fb_ioctl_read_disp_register(arg);
		break;

	case FBIO_MB86290_WRITE_DISP_REG:
		err = mb86290fb_ioctl_write_disp_register(arg);
		break;

	case FBIO_MB86290_READ_CAP_REG:
		err = mb86290fb_ioctl_read_cap_register(arg);
		break;

	case FBIO_MB86290_WRITE_CAP_REG:
		err = mb86290fb_ioctl_write_cap_register(arg);
		break;

	case FBIO_MB86290_READ_GEO_REG:
		err = mb86290fb_ioctl_read_geo_register(arg);
		break;

	case FBIO_MB86290_WRITE_GEO_REG:
		err = mb86290fb_ioctl_write_geo_register(arg);
		break;

	case FBIO_MB86290_READ_DRAW_REG:
		err = mb86290fb_ioctl_read_draw_register(arg);
		break;

	case FBIO_MB86290_WRITE_DRAW_REG:
		err = mb86290fb_ioctl_write_draw_register(arg);
		break;

	default:
		return -ENOTTY;
	}

	return err;
}

int mb86290fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	PDEBUG("mb86290fb_mmap 0x%lx\n", (unsigned long)vma);
	return mb86290fb_memory_mapping(vma);
}

int mb86290fb_blank(int blank, struct fb_info *info)
{
	unsigned long reg;

	if (blank) {
		/* DispOff */
		reg = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_EXT_MODE,
					      reg & ~0x00010000);
		reg = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE);
		PDEBUG("mb86290fb_blank 0x%lx\n", reg);
	} else {
		/* DispOn */
		reg = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE);
		MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_EXT_MODE,
					      reg | 0x80010000);
		reg = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE);
		PDEBUG("mb86290fb_unblank 0x%lx\n", reg);
	}
	return 0;
}

/* mb86290fb_initdispparam_console */
void mb86290fb_initdispparam_console(void)
{
#if !defined(CONFIG_FB_PRE_INIT_FB)
	unsigned long reg;

	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L0_EXT_MODE, 0);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_C_DISP_POS, 0);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_L0_WIN_SIZE,
				      ((MB86290FB_DFLT_SI_YRES - 1) ) |
				      MB86290FB_DFLT_SI_XRES  << 16);

	/* DispDimension */
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_C_MODE_W_H,
				      (1 << 31) |
				      ((MB86290FB_DFLT_SI_XRES *
					MB86290FB_DFLT_SI_BITS_PER_PIXEL / 8 /
					64) << 16) | (MB86290FB_DFLT_SI_YRES -
						      1));
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_C_ORG_ADR, 0);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_C_DISP_ADR, 0);

	/* Both HW-Cursors off (to be on the safe side) */
	reg = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_CURSOR_MODE);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_CURSOR_MODE, reg & ~0x00300000);

	/* DispOn */
	reg = MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE);
	MB86290FB_WRITE_DISP_REGISTER(GDC_DISP_REG_EXT_MODE, reg | 0x80010000);
#endif
	PDEBUG("mb86290fb_initdispparam_console\n");
	PDEBUG("HTP: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_H_TOTAL) );
	PDEBUG("HDP: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_H_PERIOD) );
	PDEBUG("VHW-HP: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_V_H_W_H_POS) );
	PDEBUG("VTR: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_V_TOTAL) );
	PDEBUG("VDP-P: 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_V_PERIOD_POS) );
	PDEBUG("o 18 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_WIN_POS) );
	PDEBUG("o 1C 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_WIN_SIZE) );
	PDEBUG("o 20 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_MODE_W_H) );
	PDEBUG("o 24 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_ORG_ADR) );
	PDEBUG("o 28 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_DISP_ADR) );
	PDEBUG("o 2C 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_DISP_POS) );
	PDEBUG("o 100 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE) );
	PDEBUG("o 110 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_EXT_MODE) );
	PDEBUG("o 114 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_WIN_POS) );
	PDEBUG("o 118 0x%08lx\n", MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_WIN_SIZE) );
	PDEBUG("MMR 0x%08lx\n", MB86290FB_READ_HOST_REGISTER((GDC_HOST_REG_MEMORY_MODE << 2)) );
	PDEBUG("CCF 0x%08lx\n",	MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_INTERNAL_CLOCK << 2));
}

/* mb86290fb_setdmamemory */
int __init mb86290fb_setdmamemory(unsigned char *addr, unsigned long size)
{
	mb86290fb_pInfo->dmamem = addr;
	mb86290fb_pInfo->dmamem_size = size;
	return 0;
}

/* mb86290fb_setup */
int __init mb86290fb_setup(char *options)
{
	PDEBUG("fb_setup called.\n");
	return 0;
}

static void __devexit mb86290fb_cleanup(struct fb_info *info)
{
	PDEBUG("mb86290fb_cleanup\n");
	unregister_framebuffer(info);
}

static int mb86290fb_register_driver(void)
{
	if (mb86290fb_initialize_gdc()) {
		printk(KERN_ALERT "MB86290: initialize_gdc failed.\n");
		return -EINVAL;
	}

	set_fb_info();
	set_fb_var_screeninfo();
	set_fb_fix_screeninfo();

	if (register_framebuffer(&mb86290fb_pInfo->fb_info) < 0) {
		printk(KERN_ALERT "MB86290: register_framebuffer failed.\n");
		return -EINVAL;
	}
	return 0;
}

#ifdef MB86290FB_USE_PCI
static int mb86290fb_pci_register(struct pci_dev *pdev,
				  const struct pci_device_id *ent)
{
	int ret;

	ret = pci_enable_device(pdev);
	if (ret < 0) {
		printk(KERN_ERR "mb86290fb: Cannot enable PCI device\n");
		return ret;
	}

	ret = pci_request_regions(pdev, "mb86290fb");
	if (ret < 0) {
		printk(KERN_ERR "mb86290fb: Cannot reserve region for PCI device\n");
		return ret;
	}

	mb86290fb_pInfo->base_phys = pci_resource_start(pdev, 0);
	pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &mb86290fb_pInfo->irq);

	return mb86290fb_register_driver();
}

void __devexit mb86290fb_pci_unregister(struct pci_dev *pdev)
{
	mb86290fb_cleanup((struct fb_info *)pci_get_drvdata(pdev));
}

static struct pci_driver mb86290fb_pci_driver = {
	.name = "mb86290fb",
	.id_table = mb86290fb_pci_table,
	.probe = mb86290fb_pci_register,
	.remove = __devexit_p(mb86290fb_pci_unregister),
};
#endif

/*
 *  Modularization
 */

int __init mb86290fb_init(void)
{
#ifdef MB86290FB_USE_PCI
#ifndef CONFIG_PCI
#error "PCI support is required to use MB86290 driver in PCI mode."
#endif
	return pci_register_driver(&mb86290fb_pci_driver);
#else
	return mb86290fb_register_driver();
#endif
}

void __exit mb86290fb_exit(void)
{
#ifdef MB86290FB_USE_PCI
	pci_unregister_driver(&mb86290fb_pci_driver);
#else
	mb86290fb_cleanup(&mb86290fb_pInfo->fb_info);
#endif
}

module_init(mb86290fb_init);

#ifdef MODULE
module_exit(mb86290fb_exit);
#endif

MODULE_LICENSE("GPL");
