/*
 * Copyright (C) 2007 Wolfgang Grandegger <wg@grandegger.com>
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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <asm/io.h>

#include "sja1000.h"


#define DRV_NAME  "can-ems-pci"

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("Socket-CAN driver for EMS CPC PCI CAN cards");
MODULE_SUPPORTED_DEVICE("EMS CPC PCI CAN card");
MODULE_LICENSE("GPL v2");

struct ems_pci {
	int channel;
	struct pci_dev *pci_dev;
	struct net_device *slave_dev;
	volatile void __iomem *conf_addr;
};

#define EMS_PCI_CAN_CLOCK   (16000000 / 2)

#define EMS_PCI_MASTER	    1 /* multi channel device, this device is master */
#define EMS_PCI_SLAVE 	    2 /* multi channel device, this is slave */

/*
 * Register definitions and descriptions are from LinCAN 0.3.3.
 *
 * PSB4610 PITA-2 bridge control registers
 */
#define PITA2_ICR           0x00	/* Interrupt Control Register */
#define PITA2_ICR_INT0      0x00000002	/* [RC] INT0 Active/Clear */
#define PITA2_ICR_INT0_EN   0x00020000	/* [RW] Enable INT0 */

#define PITA2_MISC          0x1c	/* Miscellaneous Register */
#define PITA2_MISC_CONFIG   0x04000000	/* Multiplexed Parallel_interface_model */

/*
 * The board configuration is probably following:
 * RX1 is connected to ground.
 * TX1 is not connected.
 * CLKO is not connected.
 * Setting the OCR register to 0xDA is a good idea.
 * This means  normal output mode , push-pull and the correct polarity.
 */
#define EMS_PCI_OCR         (OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL)

/*
 * In the CDR register, you should set CBP to 1.
 * You will probably also want to set the clock divider value to 7
 * (meaning direct oscillator output) because the second SJA1000 chip
 * is driven by the first one CLKOUT output.
 */
#define EMS_PCI_CDR_MASTER  (CDR_CBP | CDR_CLKOUT_MASK)
#define EMS_PCI_CDR_SLAVE   (CDR_CBP | CDR_CLKOUT_MASK | CDR_CLK_OFF)
#define EMS_PCI_CONF_SIZE   0x0100  /* Size of the config io-memory */
#define EMS_PCI_PORT_START  0x0400  /* Start of the channel io-memory */
#define EMS_PCI_PORT_SIZE   0x0200  /* Size of a channel io-memory */


#define EMS_PCI_PORT_BYTES  0x4     /* Each register occupies 4 bytes */

#define EMS_PCI_VENDOR_ID   0x110a  /* PCI device and vendor ID */
#define EMS_PCI_DEVICE_ID   0x2104

static struct pci_device_id ems_pci_tbl[] = {
	{EMS_PCI_VENDOR_ID, EMS_PCI_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
        { 0,}
};
MODULE_DEVICE_TABLE (pci, ems_pci_tbl);


static u8 ems_pci_read_reg(struct net_device *dev, int port)
{
        return readb((const volatile void __iomem *)
		     (dev->base_addr + (port * EMS_PCI_PORT_BYTES)));
}

static void ems_pci_write_reg(struct net_device *dev, int port, u8 val)
{
	writeb(val, (volatile void __iomem *)
	       (dev->base_addr + (port * EMS_PCI_PORT_BYTES)));
}

static void ems_pci_post_irq(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ems_pci *board = priv->priv;

	writel(PITA2_ICR_INT0_EN | PITA2_ICR_INT0,
	       board->conf_addr + PITA2_ICR);
}

static void ems_pci_del_chan(struct net_device *dev, int init_step)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ems_pci *board;

	if (!dev || !(priv = netdev_priv(dev)) || !(board = priv->priv))
		return;

	switch (init_step) {
	case 0:		/* Full cleanup */
		printk("Removing %s device %s\n", DRV_NAME, dev->name);
		unregister_sja1000dev(dev);
	case 3:
		iounmap((void *)dev->base_addr);
	case 2:
		if (board->channel != EMS_PCI_SLAVE)
			iounmap((void *)board->conf_addr);
	case 1:
		free_sja1000dev(dev);
		break;
	}

}

static int ems_pci_add_chan(struct pci_dev *pdev, int channel,
			     struct net_device **master_dev)
{
	struct net_device *dev;
	struct sja1000_priv *priv;
	struct ems_pci *board;
	unsigned long addr;
	int err, init_step;

	dev = alloc_sja1000dev(sizeof(struct ems_pci));
	if (dev == NULL)
		return -ENOMEM;
	init_step = 1;

	priv = netdev_priv(dev);
	board = priv->priv;

	board->pci_dev = pdev;
	board->channel = channel;

	if (channel != EMS_PCI_SLAVE) {

		addr = pci_resource_start(pdev, 0);
		board->conf_addr = ioremap(addr, EMS_PCI_CONF_SIZE);
		if (board->conf_addr == 0) {
			err = -ENODEV;
			goto failure;
		}
		init_step = 2;

		/* Configure PITA-2 parallel interface */
		writel(PITA2_MISC_CONFIG, board->conf_addr + PITA2_MISC);
		/* Enable interrupts from card */
		writel(PITA2_ICR_INT0_EN, board->conf_addr + PITA2_ICR);
	} else {
		struct sja1000_priv *master_priv = netdev_priv(*master_dev);
		struct ems_pci *master_board = master_priv->priv;
		master_board->slave_dev = dev;
		board->conf_addr = master_board->conf_addr;
	}

	addr = pci_resource_start(pdev, 1) + EMS_PCI_PORT_START;
	if (channel == EMS_PCI_SLAVE)
		addr += EMS_PCI_PORT_SIZE;

	dev->base_addr = (unsigned long)ioremap(addr, EMS_PCI_PORT_SIZE);
	if (dev->base_addr == 0) {
		err = -ENOMEM;
		goto failure;
	}
	init_step = 3;

	priv->read_reg = ems_pci_read_reg;
	priv->write_reg = ems_pci_write_reg;
	priv->post_irq = ems_pci_post_irq;

	priv->can.can_sys_clock = EMS_PCI_CAN_CLOCK;

	priv->ocr = EMS_PCI_OCR;

	if (channel == EMS_PCI_MASTER)
		priv->cdr = EMS_PCI_CDR_MASTER;
	else
		priv->cdr = EMS_PCI_CDR_SLAVE;

	dev->irq = pdev->irq;

	printk("%s: base_addr=%#lx conf_addr=%p irq=%d\n", DRV_NAME,
	       dev->base_addr, board->conf_addr, dev->irq);

	SET_NETDEV_DEV(dev, &pdev->dev);

	/* Register SJA1000 device */
	err = register_sja1000dev(dev);
	if (err) {
		printk(KERN_ERR "Registering %s device failed (err=%d)\n",
		       DRV_NAME, err);
		goto failure;
	}

	if (channel != EMS_PCI_SLAVE)
		*master_dev = dev;

	return 0;

failure:
	ems_pci_del_chan(dev, init_step);
	return err;
}

static int __devinit ems_pci_init_one(struct pci_dev *pdev,
				       const struct pci_device_id *ent)
{
	int err;
	struct net_device *master_dev = NULL;

	printk("%s: initializing device %04x:%04x\n",
	       DRV_NAME, pdev->vendor, pdev->device);

	if ((err = pci_enable_device(pdev)))
		goto failure;

	if ((err = pci_request_regions(pdev, DRV_NAME)))
		goto failure;

	if ((err = ems_pci_add_chan(pdev, EMS_PCI_MASTER, &master_dev)))
		goto failure_cleanup;
	if ((err = ems_pci_add_chan(pdev, EMS_PCI_SLAVE, &master_dev)))
		goto failure_cleanup;

	pci_set_drvdata(pdev, master_dev);
	return 0;

failure_cleanup:
	if (master_dev)
		ems_pci_del_chan(master_dev, 0);

	pci_release_regions(pdev);

failure:
	return err;

}

static void __devexit ems_pci_remove_one(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ems_pci *board = priv->priv;

	if (board->slave_dev)
		ems_pci_del_chan(board->slave_dev, 0);
	ems_pci_del_chan(dev, 0);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_driver ems_pci_driver = {
	.name = DRV_NAME,
	.id_table = ems_pci_tbl,
	.probe = ems_pci_init_one,
	.remove = __devexit_p(ems_pci_remove_one),
};

static int __init ems_pci_init(void)
{
	return pci_register_driver(&ems_pci_driver);
}

static void __exit ems_pci_exit(void)
{
	pci_unregister_driver(&ems_pci_driver);
}

module_init(ems_pci_init);
module_exit(ems_pci_exit);
