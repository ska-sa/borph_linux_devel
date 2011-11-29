/*
 * PPC44x USB host/device support
 *
 * Stefan Roese <sr@denx.de>
 *
 * Based on arch/ppc sequoia pci bits, that are
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * 	Wade Farnsworth <wfarnsworth@mvista.com>
 *      Copyright 2004 MontaVista Software Inc.
 *      Copyright 2006 AMCC
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/ndfc.h>

#include <mm/mmu_decl.h>

#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/prom.h>

#undef DEBUG

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

static struct ndfc_controller_settings ndfc_settings;
static struct platform_nand_ctrl nand_ctrl = {
	.priv = &ndfc_settings,
};

static struct ndfc_chip_settings chip_settings;
static struct nand_ecclayout ecclayout;
static struct mtd_partition *nand_parts;

static struct platform_nand_chip nand_chip = {
	.ecclayout = &ecclayout,
	.priv = &chip_settings,
};

static struct resource r;

static struct platform_device ndfc_dev = {
	.name = "ndfc-nand",
	.id = 0,
	.num_resources = 1,
	.resource = &r,
	.dev = {
		.platform_data = &nand_ctrl,
	}
};

static struct platform_device nand_dev = {
	.name = "ndfc-chip",
	.id = 0,
	.num_resources = 1,
	.resource = &r,
	.dev = {
		.platform_data = &nand_chip,
		.parent = &ndfc_dev.dev,
	}
};

/* Until this will be settled*/
static inline u32 of_get_int(struct device_node *np, const char *name)
{
	unsigned int size;
	const u32 *prop = of_get_property(np, name, &size);

	if ((prop == NULL) || (size != sizeof(int))) {
		printk(KERN_WARNING"%s property missing!\n", __FUNCTION__);
		return 0;
	}

	return *prop;
}

static int ppc4xx_setup_nand_chip_node(struct device_node *dev)
{
	unsigned int what = -ENODEV;
	unsigned int size, amnt;
	const u32 *prop;
	int i;

	/* process necessary properties */
	what = of_get_int(dev, "chip-nr");
	nand_chip.nr_chips = what;

	what = of_get_int(dev, "chip-offset");
	nand_chip.chip_offset = what;

	what = of_get_int(dev, "chip-delay");
	nand_chip.chip_delay = what;

	what = of_get_int(dev, "ecc-bytes");
	ecclayout.eccbytes = what;

	what = of_get_int(dev, "chip-bank-settings");
	chip_settings.bank_settings = what;

	prop = of_get_property(dev, "ecc-pos", &size);
	for (i = 0; i < (size/sizeof(unsigned int)); i++)
		ecclayout.eccpos[i] = prop[i];

	prop = of_get_property(dev, "ecc-oobfree", &size);
	amnt = size/sizeof(unsigned int);

	for (i = 0; i < amnt; i += 2) {
		nand_chip.ecclayout->oobfree[i].offset = prop[i];
		nand_chip.ecclayout->oobfree[i].length = prop[i+1];
	}

	nand_chip.nr_partitions = of_parse_flash_partitions(dev, &nand_parts);
	nand_chip.partitions = nand_parts;

	return 0;
}

static int __init ppc4xx_setup_nand_node(struct device_node *dev)
{
	struct device_node *child = NULL;
	int ret = 0;
	memset(&r, 0, sizeof(r));

	/* generic NDFC regoster */
	ret = of_address_to_resource(dev, 0, &r);
	if (ret)
		goto err;

	/* Now let's create platform_data stuff based on dts entries */
	ret = of_get_int(dev, "ccr-settings");

	ndfc_settings.ccr_settings = ret;
	ndfc_settings.ndfc_erpn = r.start & 0xf00000000ULL;

	child = of_get_next_child(dev, NULL);
	/* NAND platform device is sole, so assuming one child of ndfc node */
	if (child != NULL)
		ppc4xx_setup_nand_chip_node(child);

	ndfc_dev.resource = &r;
	nand_dev.resource = &r;

	platform_device_register(&ndfc_dev);
	platform_device_register(&nand_dev);

err:
	return ret;
}

static int ppc4xx_init_nand(void)
{
	struct device_node *np = of_find_compatible_node(NULL, "nand",
							 "ibm,nand");

	if (np != NULL)
		ppc4xx_setup_nand_node(np);

	return 0;
}
arch_initcall(ppc4xx_init_nand);
