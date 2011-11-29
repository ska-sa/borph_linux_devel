/*
 *  linux/drivers/ide/ppc/sc3ide.c -- SolidCard III IDE Driver
 *
 *  (C) Copyright 2006
 *  Heiko Schocher, DENX Software Engineering, <hs@denx.de>.
 *
 *  From:
 *  2003 Juergen Beisert, info@eurodsn.de
 *  This file based in parts on cpci405ide.c from stefan.roese@esd-electronics.de
 *
 *  From cpci405ide.c:
 *  Copyright (C) 2001-2003 Stefan Roese, stefan.roese@esd-electronics.de
 *
 *  This driver was written based on information obtained from the MacOS IDE
 *  driver binary by Mikael Forselius
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>

#include <asm/machdep.h>

extern unsigned long isa_io_access;	/* from arch/ppc/platforms/solidcard3.c */

/*
 * There are two possibilities to access the IDE drives
 *  - through standard I/O ISA bus interface
 *  - through separate way for higher data rates
 *
 * We could change this way, but we assume u-boot does it the right way...
 * So this driver leave it unchanged and only reads back the current configuration
 *
 * Standard ISA bus IDE:
 * Base of the ISA emulation ISA-IO-BASE + 0x1F0/0x3F0
 *
 * Boosted IDE:
 * Base of Memory Bank 5 + 0x000/0x008
 *
 * The IDE IRQ (on x86 platforms IRQ14 for the first IDE) is connected to the
 * external IRQ 1 -> logically IRQ26
 */

#define SC3_IRQ_IDE 26

/*
 *  IDE register offsets
 */

static int sc3ide_ISA_offsets[IDE_NR_PORTS] __initdata = {	/* ISA emulation */
	0x1F0, 0x1F1, 0x1F2, 0x1F3,
	0x1F4, 0x1F5, 0x1F6, 0x1F7,
	0x3F6,
	0	/* IDE_IRQ_OFFSET */
};

static int sc3ide_PIO_offsets[IDE_NR_PORTS] __initdata = {	/* boosted access */
	0x000, 0x001, 0x002, 0x003,
	0x004, 0x005, 0x006, 0x007,
	0x00E,
	0	/* IDE_IRQ_OFFSET */
};

/**
 * sc3_ide_init - Probe for an IDE interface on SolidCard III CPU card
 *
 * The ISA emulation is already mapped (I hope so...)
 * This driver assumes Memory Bank #5 for probing boosted IDE access
 */

static int __init sc3_ide_init(void)
{
	hw_regs_t hw;
	int index = -1;
	unsigned int raw;
	unsigned int sc3_ide_base_vmapped;
	unsigned int sc3_ide_base_pmapped;
	unsigned int	sc3_is_cameron;

	/* read back Memory Bank 5 configuration */
	mtdcr(0x012, 0x05);
	sc3_ide_base_pmapped = mfdcr(0x013);
	mtdcr(0x012, 0x06);
	sc3_is_cameron = mfdcr(0x013);

	/* On the Cameron board we have no IDE, so if Memory Bank 6
	   empty, we think it is a cameron board. */
	if (sc3_is_cameron == 0x0) {
		printk (KERN_INFO "SC3: Cameron has no IDE\n");
		return 0;
	}
	if (sc3_ide_base_pmapped & 0x18000) {
		/*
		 * Memory Bank 5 is assigned. So the separate way for
		 * IDE access is enabled
		 */
		sc3_ide_base_pmapped &= 0xFFF00000;
		/*
		 * It's nice to see the area locked, if this driver runs
		 */
		if (!request_mem_region(sc3_ide_base_pmapped,1*1024*1024,"Boosted IDE interface")) {
			printk (KERN_ERR"Cannot request area for boosted IDE interface\n");
			return 0;
		}
		/* at least 1 MiB */
		raw = (unsigned long)ioremap_nocache(sc3_ide_base_pmapped, 1*1024*1024);
		if (raw) {
			/* 
			 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			 * !!! we use the PCI in/out functions, so _IO_BASE is 
			 * added to the port address in ide_iops.c, we have to 
			 * substract here. 
			 */
			sc3_ide_base_vmapped = raw - _IO_BASE;	
			ide_setup_ports(&hw,
				sc3_ide_base_vmapped,	/* base address */
				sc3ide_PIO_offsets,	/* offsets */
				0,			/* ctrl */
				0,			/* intr */
				NULL,			/* ack_intr */
				SC3_IRQ_IDE);		/* irq */
			index = ide_register_hw(&hw, NULL);
			if (index == -1) {
				printk(KERN_ERR "SC3 IDE: error during register\n");
			}
		} else {
			printk(KERN_ERR "Cannot map IDE\n");
			release_mem_region(sc3_ide_base_pmapped,1*1024*1024);
		}
	} else {
		/*
		 * Memory Bank 5 is not assigned. So we have to use the
		 * standard ISA emulation. This slows down our data rate.
		 *
		 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		 * !!! we use the PCI in/out functions, so _IO_BASE is 
		 * added to the port address in ide_iops.c, we have to 
		 * substract here. 
		 */
		sc3_ide_base_vmapped = isa_io_access - _IO_BASE;
		ide_setup_ports(&hw,
			sc3_ide_base_vmapped,	/* base address */
			sc3ide_ISA_offsets,	/* offsets */
			0,			/* ctrl */
			0,			/* intr */
			NULL,			/* ack_intr */
			SC3_IRQ_IDE);		/* irq */
		index = ide_register_hw(&hw, NULL);
		if (index == -1) {
			printk(KERN_ERR "SC3 IDE: error during register\n");
		}
	}
	return 0;
}

module_init(sc3_ide_init);

/*
 * Clean up routine
 */
#ifdef MODULE
static void __exit sc3_ide_cleanup(void)
{
}
module_exit(sc3_ide_cleanup);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juergen Beisert <jbeisert@netscape.net>");
MODULE_DESCRIPTION("Board-specific layer for IDE access on SolidCard III card");
