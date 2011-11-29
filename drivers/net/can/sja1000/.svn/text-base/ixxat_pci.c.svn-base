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


#define DRV_NAME  "can-ixxat-pci"

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("Socket-CAN driver for IXXAT PC-I 04/PCI PCI cards");
MODULE_SUPPORTED_DEVICE("IXXAT PC-I 04/PCI card");
MODULE_LICENSE("GPL v2");

struct ixxat_pci {
	int channel;
	struct pci_dev *pci_dev;
	struct net_device *slave_dev;
	int conf_addr;
};

#define IXXAT_PCI_CAN_CLOCK  (16000000 / 2)

#define IXXAT_PCI_OCR	     (OCR_TX0_PUSHPULL | OCR_TX0_INVERT | \
			      OCR_TX1_PUSHPULL)
#define IXXAT_PCI_CDR	     0

#define IXXAT_PCI_SINGLE     0 /* this is a single channel device */
#define IXXAT_PCI_MASTER     1 /* multi channel master device */
#define IXXAT_PCI_SLAVE      2 /* multi channel slave device */

#define CHANNEL_OFFSET       0x200
#define CHANNEL_MASTER_RESET 0x110
#define CHANNEL_SLAVE_RESET  (CHANNEL_MASTER_RESET + CHANNEL_OFFSET)

#define INTCSR_OFFSET        0x4c /* Offset in PLX9050 conf registers */
#define INTCSR_MASTER        0x41 /* LINT1 and PCI interrupt enabled */
#define INTCSR_SLAVE         0x08 /* LINT2 enabled */

/* PCI vender, device and sub-device ID */
#define IXXAT_PCI_VENDOR_ID  0x10b5
#define IXXAT_PCI_DEVICE_ID  0x9050
#define IXXAT_PCI_SUB_SYS_ID 0x2540

#define IXXAT_PCI_CONF_SIZE  0x0080
#define IXXAT_PCI_BASE_SIZE  0x0400

static struct pci_device_id ixxat_pci_tbl[] = {
	{IXXAT_PCI_VENDOR_ID, IXXAT_PCI_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
        { 0,}
};

MODULE_DEVICE_TABLE(pci, ixxat_pci_tbl);

static u8 ixxat_pci_read_reg(struct net_device *dev, int port)
{
	u8 val;
	val = readb((const volatile void __iomem *)(dev->base_addr + port));
	return val;
}

static void ixxat_pci_write_reg(struct net_device *dev, int port, u8 val)
{
	writeb(val, (volatile void __iomem *)(dev->base_addr + port));
}

static void ixxat_pci_del_chan(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ixxat_pci *board;
	u8 intcsr;

	if (!dev || !(priv = netdev_priv(dev)) || !(board = priv->priv))
		return;

	printk("Removing %s device %s\n", DRV_NAME, dev->name);
	unregister_sja1000dev(dev);

	/* Disable PCI interrupts */
	intcsr = inb(board->conf_addr + INTCSR_OFFSET);
	if (board->slave_dev) {
		intcsr &= ~INTCSR_MASTER;
		outb(intcsr, board->conf_addr + INTCSR_OFFSET);
		writeb(0x1, (volatile void __iomem *)
		       (dev->base_addr + CHANNEL_MASTER_RESET));
	} else {
		intcsr &= ~INTCSR_SLAVE;
		outb(intcsr, board->conf_addr + INTCSR_OFFSET);
		writeb(0x1, (volatile void __iomem *)
		       (dev->base_addr + CHANNEL_SLAVE_RESET));
		iounmap((void *)dev->base_addr);
	}
	free_sja1000dev(dev);
}

static int ixxat_pci_add_chan(struct pci_dev *pdev, int channel,
			      struct net_device **master_dev,
			      int conf_addr,
			      volatile void __iomem *base_addr)
{
	struct net_device *dev;
	struct sja1000_priv *priv;
	struct ixxat_pci *board;
	u8 intcsr;
	int err;

	dev = alloc_sja1000dev(sizeof(struct ixxat_pci));
	if (dev == NULL)
		return -ENOMEM;

	priv = netdev_priv(dev);
	board = priv->priv;

	board->pci_dev = pdev;
	board->conf_addr = conf_addr;
	dev->base_addr = (unsigned long)base_addr;

	if (channel == IXXAT_PCI_SLAVE) {
		struct sja1000_priv *master_priv = netdev_priv(*master_dev);
		struct ixxat_pci *master_board = master_priv->priv;
		master_board->slave_dev = dev;
	}

	priv->read_reg = ixxat_pci_read_reg;
	priv->write_reg = ixxat_pci_write_reg;

	priv->can.can_sys_clock = IXXAT_PCI_CAN_CLOCK;

	priv->ocr = IXXAT_PCI_OCR;
	priv->cdr = IXXAT_PCI_CDR;

	/* Set and enable PCI interrupts */
	dev->irq = pdev->irq;
	intcsr = inb(board->conf_addr + INTCSR_OFFSET);
	if (channel == IXXAT_PCI_SLAVE)
		intcsr |= INTCSR_SLAVE;
	else
		intcsr |= INTCSR_MASTER;
	outb(intcsr, board->conf_addr + INTCSR_OFFSET);

	printk("%s: base_addr=%#lx conf_addr=%#x irq=%d\n", DRV_NAME,
	       dev->base_addr, board->conf_addr, dev->irq);

	SET_NETDEV_DEV(dev, &pdev->dev);

	err = register_sja1000dev(dev);
	if (err) {
		printk(KERN_ERR "Registering %s failed (err=%d)\n",
		       DRV_NAME, err);
		goto failure;
	}

	if (channel != IXXAT_PCI_SLAVE)
		*master_dev = dev;

	return 0;

failure:
	free_sja1000dev(dev);
	return err;
}

static int __devinit ixxat_pci_init_one(struct pci_dev *pdev,
				       const struct pci_device_id *ent)
{
	struct net_device *master_dev = NULL;
	volatile void __iomem *base_addr;
	unsigned long addr;
	int conf_addr, channel, err;
	u16 sub_sys_id;

	printk("%s: initializing device %04x:%04x\n",
	       DRV_NAME, pdev->vendor, pdev->device);

	if ((err = pci_enable_device(pdev)))
		goto failure;

	if ((err = pci_request_regions(pdev, DRV_NAME)))
		goto failure;

	if ((err = pci_read_config_word(pdev, 0x2e, &sub_sys_id)))
		goto failure_release_pci;

	if (sub_sys_id != IXXAT_PCI_SUB_SYS_ID)
		return -ENODEV;

	/* Enable memory and I/O space */
	if ((err = pci_write_config_word(pdev, 0x04, 0x3)))
		goto failure_release_pci;

	conf_addr = pci_resource_start(pdev, 1);

	addr = pci_resource_start(pdev, 2);
	base_addr = ioremap(addr, IXXAT_PCI_BASE_SIZE);
	if (base_addr == 0) {
		err = -ENODEV;
		goto failure_release_pci;
	}

	/* Check if second channel is available after reset */
	writeb(0x1, (volatile void __iomem *)base_addr + CHANNEL_MASTER_RESET);
	writeb(0x1, (volatile void __iomem *)base_addr + CHANNEL_SLAVE_RESET);
	udelay(100);
	if (readb(base_addr + CHANNEL_OFFSET + REG_MOD) != 0x21 ||
	    readb(base_addr + CHANNEL_OFFSET + REG_SR ) != 0x0c ||
	    readb(base_addr + CHANNEL_OFFSET + REG_IR ) != 0xe0)
		channel = IXXAT_PCI_SINGLE;
	else
		channel = IXXAT_PCI_MASTER;

	if ((err = ixxat_pci_add_chan(pdev, channel, &master_dev,
				      conf_addr, base_addr)))
		goto failure_iounmap;

	if (channel != IXXAT_PCI_SINGLE) {
		channel = IXXAT_PCI_SLAVE;
		if ((err = ixxat_pci_add_chan(pdev, channel,
					      &master_dev, conf_addr,
					      base_addr + CHANNEL_OFFSET)))
			goto failure_iounmap;
	}

	pci_set_drvdata(pdev, master_dev);
	return 0;

failure_iounmap:
	if (master_dev)
		ixxat_pci_del_chan(master_dev);
	iounmap(base_addr);

failure_release_pci:
	pci_release_regions(pdev);

failure:
	return err;
}

static void __devexit ixxat_pci_remove_one(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ixxat_pci *board = priv->priv;

	if (board->slave_dev)
		ixxat_pci_del_chan(board->slave_dev);
	ixxat_pci_del_chan(dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_driver ixxat_pci_driver = {
	.name = DRV_NAME,
	.id_table = ixxat_pci_tbl,
	.probe = ixxat_pci_init_one,
	.remove = __devexit_p(ixxat_pci_remove_one),
};

static int __init ixxat_pci_init(void)
{
	return pci_register_driver(&ixxat_pci_driver);
}

static void __exit ixxat_pci_exit(void)
{
	pci_unregister_driver(&ixxat_pci_driver);
}

module_init(ixxat_pci_init);
module_exit(ixxat_pci_exit);
