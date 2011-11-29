/*
 * drivers/mtd/nand/sc3nand.c
 *
 * Copyright (C) 2003 Juergen Beisert (jbeisert@netscape.net)
 *
 * $Id$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Overview:
 *   This is a device driver for the NAND flash device found on the
 *   SolidCard III, an IBM PowerPC 405GP(r) based CPU card which utilizes the
 *   Samsung KM29U128IT. This is a 128Mbit (16MiB x 8 bits) NAND flash device.
 *
 * This file is based on the SPIA NAND driver by Steven J. Hill (sjhill@cotw.com)
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <linux/ioport.h>

/*
 * Values specific to the SolidCard III card (used with PPC405GP(r) processor)
 *
 * The NAND chip is connected to the 405GP(r) chipset through CS1#.
 * All accesses to the data lines are selected through CS1#. This enables
 * WE# and RD#. All other signals are controlled via GPIO.
 * - Command Latch Enable: GPIO1
 * - Address Latch Enable: GPIO2
 * - Chip Enable: GPIO4
 * - The Ready/Busy# signal can read back via GPIO17
 *
 */

#undef writel
#undef readl
#define writel(b,addr) ((*(volatile u32 *) (addr)) = (b))
#define readl(addr) (*(volatile u32 *) (addr))

/* Define SC3_NAND_INTERRUPT to avoid busy waiting */
/* #define SC3_NAND_INTERRUPT 25 */
#undef SC3_NAND_INTERRUPT	/* DO NOT USE IT. ITS NOT WORKING YET */

/* our private data for this chip */
struct sc3_nand {
	unsigned long sc3_nand_paddr;	/* This is the physical baseaddress of the NAND chip */
	void *sc3_io_base;		/* This is the virtual baseaddress of the NAND chip */
	void *sc3_control_base;		/* This is the virtual baseaddress of the GPIO */
};

/*
 * MTD structure for SolidCard III card
 */
static struct mtd_info *sc3_mtd = NULL;

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };

#include <linux/mtd/partitions.h>
/*
 * Define partitions for flash device
 */
#define NUM_PARTITIONS 1

const static struct mtd_partition partition_info[NUM_PARTITIONS] =
{
	{
		.name	= "SC3 NAND flash",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	}
};
#endif

/*
 * Where the control bits are located (GPIO)
 */
#define SC3_NAND_ALE 29 /* GPIO PIN 3 */
#define SC3_NAND_CLE 30	/* GPIO PIN 2 */
#define SC3_NAND_CE  27 /* GPIO PIN 5 */

static void sc3_select_chip(struct mtd_info *mtd, int chipnr)
{
	void *sc3_control_base=(void*)((struct sc3_nand*)(((struct nand_chip*)\
				mtd->priv))->priv)->sc3_control_base;
	clear_bit(SC3_NAND_CE,sc3_control_base);
}

/**
 *	sc3_nand_ready	- Checks if the device is ready after last command
 *	@mtd: Pointer to current mtd device
 *
 * Checks if the NAND device is ready again after last given command.
 * Returns 0 if not ready
 */
static int sc3_nand_ready(struct mtd_info *mtd)
{
void *ioAddr = (void*)((struct sc3_nand*)(((struct nand_chip*)mtd->priv))->priv)->sc3_control_base;

	if (!(readl(ioAddr+0x1C) & 0x4000))
		return(0);

	return(1);
}

/**
 *	sc3_nand_command - Send a command to the onboard NAND chip
 *	@mtd: pointer to current mtd device
 *	@command: Command to be sent
 *	@column:
 *	@page_addr:
 *
 * Sends a command to the NAND chip. After some short commands
 * this function waits for completion.
 */
static void sc3_nand_command(struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
void *sc3_control_base=(void*)((struct sc3_nand*)(((struct nand_chip*)mtd->priv))->priv)->sc3_control_base;
void *ioAddr = (void*)(((struct nand_chip*)mtd->priv)->IO_ADDR_W);

/*
 * Begin command latch cycle
 */
	set_bit(SC3_NAND_CLE,sc3_control_base);
/*
 * Write out the command to the device.
 */
	if (command != NAND_CMD_SEQIN)
		out_8(ioAddr,command);
	else {
		if (column >= 256) {
			if (column < 512) {
				column -= 256;
				out_8(ioAddr,NAND_CMD_READ1);
				out_8(ioAddr,NAND_CMD_SEQIN);
			} else {
				column -= 512;
				out_8(ioAddr,NAND_CMD_READOOB);
				out_8(ioAddr,NAND_CMD_SEQIN);
			}
		} else {
			out_8(ioAddr,NAND_CMD_READ0);
			out_8(ioAddr,NAND_CMD_SEQIN);
		}
	}
/*
 * Clear CLE, because command phase ends
 */
	clear_bit(SC3_NAND_CLE,sc3_control_base);

	if (column != -1 || page_addr != -1) {
		set_bit(SC3_NAND_ALE,sc3_control_base);	/* Start address cycle */
	/*
	 * Serialize input address
	 */
		if (column != -1)
			out_8(ioAddr,column);
		if (page_addr != -1)
			out_le16(ioAddr,page_addr);
		clear_bit(SC3_NAND_ALE,sc3_control_base);	/* Latch in address */
	}
/*
 * program and erase have their own busy handlers
 * status and sequential in needs no delay
*/
	switch (command) {
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		return;
	}
/*
 * wait until command is processed
 */
	while (!sc3_nand_ready(mtd));
}

#ifdef SC3_NAND_INTERRUPT
/**
 *	sc3_nand_interrupt	- If the last command is finished, the NAND wakes us
 * @irq: interrupt number, not used here
 * @dev_id: current context (shared interrupt lines), not used here
 * @regs: pointer to register contents before calling
 *
 * This function is called, whenever the NAND Ready/Busy# line changes from
 * low to high. We are using an inverter between the NAND line and the
 * interrupt entry pin. So we are waiting for the falling edge.
 */

static void sc3_nand_interrupt(int irq,void *dev_id,struct pt_regs *regs)
{
}

/**
 *	sc3_nand_wait	- Wait for the NAND flash
 * @mtd: mtd device waiting for
 * @this: NAND chip waiting for
 * @state: cause of calling
 *
 * This function uses the interrupt feature to
 * let the CPU do other things instead of busy waiting
 * It replaces the standard nand_wait() in mtd/nand/nand.c
 */

static int sc3_nand_wait(struct mtd_info *mtd, struct nand_chip *this, int state)
{
	unsigned long	timeo = jiffies;
	int	status;

	if (state == FL_ERASING)
		 timeo += (HZ * 400) / 1000;
	else
		 timeo += (HZ * 20) / 1000;

	spin_lock_bh (&this->chip_lock);
	this->cmdfunc (mtd, NAND_CMD_STATUS, -1, -1);

	while (time_before(jiffies, timeo)) {
		/* Check, if we were interrupted */
		if (this->state != state) {
			spin_unlock_bh (&this->chip_lock);
			return 0;
		}
		if (this->dev_ready) {
			if (this->dev_ready ())
				break;
		}
		if (readb (this->IO_ADDR_R) & 0x40)
			break;

		spin_unlock_bh (&this->chip_lock);
		yield ();
		spin_lock_bh (&this->chip_lock);
	}
	status = (int) readb (this->IO_ADDR_R);
	spin_unlock_bh (&this->chip_lock);

	return status;
}
#endif

/**
 *	sc3_nand_init	- Main initialization routine
 *
 * Maps the NAND flash memory into vitual space and
 * creates structures mtd needs.
 * Returns:
 *  - 0: all ok
 *  - -ENOMEM: No memory, no mapping, no resource, unable to register
 *  - -ENXIO: No NAND chip found
 */

static int __init sc3_nand_init(void)
{
	unsigned long sc3_nand_paddr = 0UL;
	void *sc3_io_base = NULL;
	void *sc3_control_base = NULL;
	struct nand_chip *this;
	struct sc3_nand *priv;
	int	rc = -ENOMEM;
	const char *part_type = 0;
	int mtd_parts_nb = 0;
	struct mtd_partition *mtd_parts = NULL;

/*
 * Where is the flash memory mapped physically?
 * Solution: Read back where the bootloader has mapped CS1#
 */
	mtdcr(0x012,0x01);
	sc3_nand_paddr=mfdcr(0x013);

	if (!(sc3_nand_paddr & 0x18000)) {
		printk (KERN_ERR"Seems there is no NAND flash present in this system!\n");
		goto escape1;
	}
	sc3_nand_paddr&=0xFFF00000;
/*
 * It's nice to see the area locked, if this driver runs
 */
	if (!request_mem_region(sc3_nand_paddr,1*1024*1024,"16MiB OnCard NAND flash")) {
		printk (KERN_ERR"Cannot request area for NAND flash\n");
		goto escape1;
	}
/*
 * We need access to the NAND chip itself and the control lines
 */
	if (!(sc3_io_base=ioremap_nocache(sc3_nand_paddr,1*1024*1024))) {
		printk (KERN_ERR "Cannot map NAND area\n");
		goto escape2;
	}
	if (!(sc3_control_base=ioremap_nocache(0xEF600700,0x20))) {
		printk (KERN_ERR "Cannot map GPIO area for NAND flash access\n");
		goto escape3;
	}
/*
 * Allocate memory for MTD device structure and private data
 */
	if (!(sc3_mtd = kmalloc (sizeof(struct mtd_info)
									 + sizeof(struct nand_chip)
									 + sizeof(struct sc3_nand),GFP_KERNEL))) {
		printk (KERN_ERR "Unable to allocate NAND MTD device structure.\n");
		goto escape4;
	}
/*
 * >sc3_mtd< is the pointer to the whole structure.
 * Calulate pointer to our NAND- and private data
 */
	this = (struct nand_chip*) (&sc3_mtd[1]);		/* private data in mtd */
	priv = (struct sc3_nand*) (&this[1]);			/* private data in nand_chip */

	/* Initialize structures */
	memset((char *) sc3_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));
	sc3_mtd->priv = this;
	this->priv=priv;
/*
 * At first save our private data
 */
	priv->sc3_nand_paddr=sc3_nand_paddr;
	priv->sc3_io_base=sc3_io_base;
	priv->sc3_control_base=sc3_control_base;

/*
 * We are using our own functions to
 * - send a command to the chip
 * - wait for chip ready
 * - wait for NAND ready
 */
	this->cmdfunc=sc3_nand_command;
	this->dev_ready=sc3_nand_ready;
	this->select_chip = sc3_select_chip;
/*
 * Our chips has anomalities
 */
	this->options=NAND_NO_AUTOINCR;

#ifdef SC3_NAND_INTERRUPT
	if (!request_irq(SC3_NAND_INTERRUPT,
					sc3_nand_interrupt,
					IRQF_SAMPLE_RANDOM,
					"SC3 NAND flash",
					NULL)) {
		disable_irg(SC3_NAND_INTERRUPT);	/* we need it, but later! */
		this->waitfunc=sc3_wait;
	}
	else
		printk(KERN_INFO"Unable to claim IRQ%d for NAND flash. Running without...\n",
				 SC3_NAND_INTERRUPT);
#endif
/*
 * Predefine something:
 * - set address of NAND IO lines
 * - 7 us command delay time (see KM29U128IT datasheet)
 * - we need Software-ECC
 */
	this->IO_ADDR_R = this->IO_ADDR_W = sc3_io_base;
	this->chip_delay = 7;
	this->ecc.mode = NAND_ECC_SOFT;
/*
 * Scan to find existence of the device
 */
	if (nand_scan (sc3_mtd,1)) {
		rc=-ENXIO;
		goto escape5;
	}
#ifdef CONFIG_MTD_PARTITIONS
	sc3_mtd->name = "sc3-nand";
	mtd_parts_nb = parse_mtd_partitions(sc3_mtd, part_probes, &mtd_parts, 0);
	if (mtd_parts_nb > 0)
		part_type = "command line";
	else
		mtd_parts_nb = 0;
#endif
	if (mtd_parts_nb == 0) {
		if (sc3_mtd->size >= (16 * 0x100000))
			mtd_parts = (struct mtd_partition *)partition_info;
		mtd_parts_nb = NUM_PARTITIONS;
		part_type = "static";
	}
	/* Register the partitions */
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
	add_mtd_partitions(sc3_mtd, mtd_parts, mtd_parts_nb);

/*
 * Return happy ;-)
 */
	return 0;
/*
 * Something went wrong. Clean up...
 */
escape5:
	kfree (sc3_mtd);
escape4:
	iounmap(sc3_control_base);
escape3:
	iounmap(sc3_io_base);
escape2:
	release_mem_region(sc3_nand_paddr,1*1024*1024);
escape1:
	return(rc);
}
module_init(sc3_nand_init);

/*
 * Clean up routine
 */
#ifdef MODULE
static void __exit sc3_nand_cleanup (void)
{
struct nand_chip *this = (struct nand_chip *) &sc3_mtd[1];
struct sc3_nand *priv=this->priv;
#ifdef SC3_NAND_INTERRUPT
	free_irq(SC3_NAND_INTERRUPT,NULL);
#endif
/*
 * Unregister the device
 */
	del_mtd_device (sc3_mtd);

/*
 * Free internal data buffer
 */

	iounmap(priv->sc3_control_base);
	iounmap(priv->sc3_io_base);
	release_mem_region(priv->sc3_nand_paddr,1*1024*1024);

/*
 * Free the MTD device structure
 */
	kfree (sc3_mtd);
}
module_exit(sc3_nand_cleanup);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juergen Beisert <jbeisert@netscape.net>");
MODULE_DESCRIPTION("Board-specific layer for NAND flash on SolidCard III card");
