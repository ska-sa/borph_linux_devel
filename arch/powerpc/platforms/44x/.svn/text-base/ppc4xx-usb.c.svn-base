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

static int __init ppc4xx_setup_usb_node(struct device_node *dev)
{
	struct platform_device *usb_dev = NULL;
	struct resource r[2];
	int ret = 0;

	memset(&r, 0, sizeof(r));

	ret = of_address_to_resource(dev, 0, &r[0]);
	if (ret)
		goto err;

	of_irq_to_resource(dev, 0, &r[1]);

	usb_dev = platform_device_register_simple("musbhsfc_udc", 1, r, 2);
	DBG("registered IBM gadget at %llx\n", r[0].start);

	if (IS_ERR(usb_dev))
		ret = PTR_ERR(usb_dev);

err:
	return ret;
}

static int ppc4xx_init_usb(void)
{
	struct device_node *np;

	for (np = NULL;
	     (np = of_find_compatible_node(np, "usb", "musbhsfc_udc")) != NULL;)
		ppc4xx_setup_usb_node(np);

	return 0;
}
arch_initcall(ppc4xx_init_usb);
