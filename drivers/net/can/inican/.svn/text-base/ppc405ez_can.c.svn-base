/*
 * drivers/can/inican/ppc405ez_can.c
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <asm/io.h>

#include "inican.h"

#define DRV_NAME      "ppc405ez_can"

/*
 * Since the CAN clock calculation is board/cpu specific,
 * this functions is not included in the generic inican.c
 * code, but in the board/cpu specific part.
 */
u32 inican_get_can_clock(void)
{
	u32 plld, primad, pll_fbk_div, pll_opb_div;
	u32 opbclk;
	u32 sysclk;
	void *cpld;

	/*
	 * Read CPLD version register, needed for SYSCLK detection
	 * since there are different Acadia board revisions with
	 * different SYSCLK's.
	 *
	 * This needs to be seperated from the PPC405 part, since
	 * it's Acadia specific.
	 */
	cpld = ioremap(ACADIA_CPLD_ADDR, PAGE_SIZE);
	sysclk = (in_8(cpld) == 0x0c) ? BOARD_SYSCLK_REV10 : BOARD_SYSCLK_REV11;
	iounmap(cpld);

	plld = CPR_READ(CPR_PLLD);
	primad = CPR_READ(CPR_PRIMAD);

	pll_fbk_div = (plld & CPR_PLLD_FBDV_MASK)  >> 24;
	pll_fbk_div = (pll_fbk_div == 0) ? 256 : pll_fbk_div;

	pll_opb_div = (primad & CPR_PRIMAD_OPBDV_MASK) >> 8;
	pll_opb_div = (pll_opb_div == 0) ? 16 : pll_opb_div;

	opbclk = (sysclk * pll_fbk_div) / pll_opb_div;

	return opbclk;
}

static int ppc405ez_can_drv_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct inican_priv *priv;
	struct resource *mem;
	u32 mem_size;
	int ret = -ENODEV;
	u32 clock;

	dev = alloc_inicandev(sizeof(struct inican_priv));
	if (!dev)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->irq = platform_get_irq(pdev, 0);
	if (!mem || !dev->irq)
		goto req_error;

	mem_size = mem->end - mem->start + 1;
	if (!request_mem_region(mem->start, mem_size, pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		goto req_error;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);
	SET_MODULE_OWNER(THIS_MODULE);

	dev->base_addr = (unsigned long)ioremap_nocache(mem->start, mem_size);

	if (!dev->base_addr) {
		dev_err(&pdev->dev, "failed to map can port\n");
		ret = -ENOMEM;
		goto fail_map;
	}

	priv = netdev_priv(dev);

	priv->can.can_sys_clock = inican_get_can_clock();

	/*
	 * The standard routines for CAN bit rate parameter calculation do
	 * not work on values like 66.6MHz. So we have to round these values
	 * to 66MHz.
	 */
	clock = inican_get_can_clock();
	priv->can.can_sys_clock = clock - (clock % 1000000);

	platform_set_drvdata(pdev, dev);

	/* Override CAN Present bit using SDR */
	SDR_WRITE(0x4085, 0x50524553);

	ret = register_inicandev(dev);
	if (ret >= 0) {
		dev_info(&pdev->dev, "probe for a port 0x%lX done\n",
			 dev->base_addr);
		return ret;
	}

	iounmap((unsigned long *)dev->base_addr);
fail_map:
	release_mem_region(mem->start, mem_size);
req_error:
	free_inicandev(dev);
	dev_err(&pdev->dev, "probe failed\n");

	return ret;
}

static int ppc405ez_can_drv_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct resource *mem;

	platform_set_drvdata(pdev, NULL);
	unregister_inicandev(dev);

	iounmap((volatile void __iomem *)(dev->base_addr));
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, mem->end - mem->start + 1);
	free_inicandev(dev);

	return 0;
}

#ifdef CONFIG_PM
static int ppc405ez_can_drv_suspend(struct platform_device *pdev,
				    pm_message_t state)
{
	return 0;
}

static int ppc405ez_can_drv_resume(struct platform_device *pdev)
{
	return 0;
}
#endif /* CONFIG_PM */

static struct platform_driver ppc405ez_can_driver = {
	.driver		= {
		.name		= DRV_NAME,
	},
	.probe		= ppc405ez_can_drv_probe,
	.remove		= ppc405ez_can_drv_remove,
#ifdef CONFIG_PM
	.suspend	= ppc405ez_can_drv_suspend,
	.resume		= ppc405ez_can_drv_resume,
#endif	/* CONFIG_PM */
};

static int __init ppc405ez_can_init(void)
{
	printk(KERN_INFO "%s initializing\n", ppc405ez_can_driver.driver.name);
	return platform_driver_register(&ppc405ez_can_driver);
}

static void __exit ppc405ez_can_cleanup(void)
{
	platform_driver_unregister(&ppc405ez_can_driver);
	printk(KERN_INFO "%s unloaded\n", ppc405ez_can_driver.driver.name);
}

module_init(ppc405ez_can_init);
module_exit(ppc405ez_can_cleanup);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver PPC405EZ processor");
