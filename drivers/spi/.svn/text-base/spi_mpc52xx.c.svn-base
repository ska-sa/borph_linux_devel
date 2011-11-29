/*
 * MPC52xx SPI controller driver.
 *
 * Copyright (C) 2006 Secret Lab Technologies Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#undef DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/spi/mpc52xx-spi.h>

#include <asm/io.h>

#ifdef CONFIG_SPI_DEBUG
static void pr_debug_buf(const char* msg, const void* buf, int len) {
	if (!buf)
		return;
	if (msg)
		printk(msg);
	while(len--)
		printk("%.2x", *(u8*)buf++);
}
#else
#define pr_debug_buf(msg, buf, len)
#endif

/* Structure of the hardware registers */
struct mpc52xx_spi {
	u8 ctrl1;		/* SPI + 0x00 */
	u8 ctrl2;		/* SPI + 0x01 */
	u8 reserved0[2];
	u8 bandrate;		/* SPI + 0x04 */
	u8 status;		/* SPI + 0x05 */
	u8 reserved1[3];
	u8 data;		/* SPI + 0x09 */
	u8 reserved2[3];
	u8 port_data;		/* SPI + 0x0D */
	u8 reserved3[2];
	u8 direction;		/* SPI + 0x10 */
} __attribute__ ((packed, aligned(1)));

#define MPC52xx_SPICR1		0xf00;
#define MPC52xx_SPICR2		0xf01;
#define MPC52xx_SPIBRR		0xf04;
#define MPC52xx_SPISR		0xf05;
#define MPC52xx_SPIDR		0xf09;
#define MPC52xx_SPIPDR		0xf0d;
#define MPC52xx_SPIDDR		0xf10;


#define DRIVER_NAME	"mpc52xx-spi"
static unsigned char __iomem *membase;	/* for mpc52xx_spi registers */
static unsigned long mapbase;
static unsigned long mapsize;
static int irqnum;
#define SPI ((struct mpc52xx_spi __iomem *)(membase))


static struct semaphore sema;	/* mutex to enforce one transfer at a time */
static DECLARE_WAIT_QUEUE_HEAD(wait_queue);

static int mpc52xx_spi_transfer_real(int sid, const void* in, void* out,
                                     size_t count, int period);

#define SPI_MIN_DELAY		1
#define spi_clk_high()		(out_8(&gpio_wkup->wkup_dvo, \
				       in_8(&gpio_wkup->wkup_dvo) | 0x04))
#define spi_clk_low()		(out_8(&gpio_wkup->wkup_dvo, \
				       in_8(&gpio_wkup->wkup_dvo) & ~0x04))
#define spi_mosi_set(val)	(out_be32(&gpio->simple_dvo, \
				    (in_be32(&gpio->simple_dvo) & ~0x1000) | \
				    ((val) ? 0x1000 : 0)))
#define spi_miso_read()		((in_be32(&gpio->simple_ival) & 0x2000) ? 1 : 0)
#define spi_cs_assert()		(out_8(&gpio->sint_dvo, \
				       in_8(&gpio->sint_dvo) & ~0x04))
#define spi_cs_deassert()	(out_8(&gpio->sint_dvo, \
				       in_8(&gpio->sint_dvo) | 0x04))
#define spi_select_cpld()	(out_be32(&gpio->simple_dvo, \
				          in_be32(&gpio->simple_dvo) & ~0x40))
#define spi_select_slave()	(out_be32(&gpio->simple_dvo, \
				          in_be32(&gpio->simple_dvo) | 0x40))

static struct mpc52xx_gpio *gpio;
static struct mpc52xx_gpio_wkup *gpio_wkup;

static void mpc52xx_spi_slave_select(int sid, int enable)
{
}

static int mpc52xx_spi_transfer_real(int sid, const void* tx, void* rx, size_t count, int period)
{
	int ret;

	pr_debug("SPI: cs=%.2x, count=%i, period=%i",
	           sid, count, period);
	pr_debug_buf(", out=", tx, count);

	udelay(SPI_MIN_DELAY);
	out_8(&SPI->port_data, 0x00);	/* Activate SS */

	/* Write first data to SPI to trigger the interupt */
	/* the interupt routine will start the rest of the transfer */
	udelay(SPI_MIN_DELAY);
	transfer_count = count;
	rx_buf_ptr = rx;
	tx_buf_ptr = tx;
	if (tx_buf_ptr)
		out_8(&SPI->data, *(u8*)tx_buf_ptr);
	else
		out_8(&SPI->data, 0);

	ret = wait_event_interruptible(wait_queue, transfer_count == 0);

	udelay(SPI_MIN_DELAY);
	out_8(&SPI->port_data, 0x08);/* Deactivate SS */

	/* Turn off clock inversion */
	out_8(&SPI->ctrl1, in_8(&SPI->ctrl1) & ~0x08);

	pr_debug_buf(", in=", rx, count);
	pr_debug("\n");

	return ret;
}

int mpc52xx_spi_transfer(int sid, const u8* tx, u8* rx, size_t count, int period)
{
	int ret;

	if (count < 1)
		return -EINVAL;
	if (period < SPI_MIN_DELAY)
		period = SPI_MIN_DELAY;

	/* Grab the semaphore, only one transfer allowed at a time */
	ret = down_interruptible(&sema);
	if (ret)
		return ret;

	/* use GPIO as slave select if necessary */
	mpc52xx_spi_slave_select(sid, 1);
	ret = mpc52xx_spi_transfer_real(sid, tx, rx, count, period);
	mpc52xx_spi_slave_select(sid, 0);

	up(&sema);

	return ret;
}

int mpc52xx_spi_pause(void)
{
	return down_interruptible(&sema);
}

void mpc52xx_spi_resume(void)
{
	up(&sema);
}

static irqreturn_t mpc52xx_spi_int(int irq, void *dev_id, struct pt_regs *regs)
{
	u8 status;
	u8 data;

	if (irq != irqnum) {
		printk(KERN_WARNING
		       "mpc52xx_spi_int : "
		       "Received wrong int %d. Waiting for %d\n", irq, irqnum);
		return IRQ_NONE;
	}

	status = in_8(&SPI->status);

	if (transfer_count == 0) {
		printk(KERN_WARNING "mpc52xx_spi_int : Unexpected IRQ event\n");
		return IRQ_HANDLED;
	}

	if (status & (1 << 7)) {
		/* Data Ready */
		data = in_8(&SPI->data);
		if (rx_buf_ptr)
			*rx_buf_ptr++ = data;
		transfer_count--;

		/* Only wakeup process if data transfer is complete */
		if (transfer_count == 0)
			wake_up_interruptible(&wait_queue);
		else {
			/* start another transfer if not done yet */
			tx_buf_ptr++;
			udelay(20);	/* FIXME, if without we miss spi interrupt ?! */
			if (tx_buf_ptr)
				out_8(&SPI->data, *(u8*)tx_buf_ptr);
			else
				out_8(&SPI->data, 0);
		}
	}

	return IRQ_HANDLED;
}

static int __devinit mpc52xx_spi_probe(struct platform_device *pdev)
{
	volatile uint __iomem *reg_ptr;
	struct resource *mem_io, *irqres;
	int ret = -ENODEV;

	/* Assume CDM Clock Enable Register has spi_clk_en = 1 
	 * which is the boot up default
	 */

	mem_io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	/* Get the IRQ from resource */
	/* No need for modf, we will not use SS to detect SPI master error */
	irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "spif");

	if ((!mem_io) | (!irqres))
		goto out_err;

	mapbase = mem_io->start;	/* save value for release_mem_region used */
	mapsize = mem_io->end - mem_io->start + 1;
	irqnum = irqres->start;

	/* Request IRQ */
	ret = request_irq(irqnum, mpc52xx_spi_int,
			  SA_INTERRUPT, "mpc52xx_spi", NULL);
	if (ret)
		goto out_err;

	if (!request_mem_region(mapbase, mapsize, DRIVER_NAME)) {
		printk(KERN_ERR DRIVER_NAME " - resource unavailable\n");
		goto request_mem_error;
	}

	membase = ioremap_nocache(mapbase, sizeof(struct mpc52xx_spi));

	if (!membase) {
		printk(KERN_ERR DRIVER_NAME " - failed to map spi regs\n");
		ret = -ENOMEM;
		goto map_io_error;
	}

	/* Specify for the AIC26 CODEC board */

	/* Setup the SPI Port */
	out_8(&SPI->ctrl1, 0x14);
	out_8(&SPI->ctrl2, 0x00);

	out_8(&SPI->bandrate, (7 << 4) | (7 << 0));

	/* SS as general input GPIO */
	/* configurate CLK (must for master mode) and data out as output pins */
	out_8(&SPI->direction, 0x0E);
	out_8(&SPI->port_data, 0x08);

	udelay(10);

	/* Now enable the whole thing */
	out_8(&SPI->ctrl1, 0xd4);

	/* --- CPLD/SLAVE select pin --- */
	reg_ptr = (volatile u32 *)(MPC52xx_MBAR_VIRT + 0x0b04); /* gpio mode */
	out_be32(reg_ptr, in_be32(reg_ptr) | 0x00000040);
	reg_ptr = (u32 *)(MPC52xx_MBAR_VIRT + 0x0b0c); /* data direction */
	out_be32(reg_ptr, in_be32(reg_ptr) | 0x00000040);

	return 0;

map_io_error:
	release_mem_region(mapbase, mapsize);
request_mem_error:
	free_irq(irqnum, NULL);
out_err:
	printk(KERN_ERR "SPI: MPC52xx SPI init FAILED !!!\n");
	return ret;
}

static int mpc52xx_spi_remove(struct platform_device *dev)
{
	free_irq(irqnum, NULL);
	release_mem_region(mapbase, mapsize);
	iounmap(membase);

	return 0;
}

static struct platform_driver mpc52xx_spi_platform_driver = {
	.probe = mpc52xx_spi_probe,
	.remove = mpc52xx_spi_remove,
#ifdef CONFIG_PM
	/* .suspend = mpc52xx_spi_suspend,      TODO */
	/* .resume = mpc52xx_spi_resume,        TODO */
#endif
	.driver = {
		   .name = DRIVER_NAME,
		   },
};

static int __init mpc52xx_spi_init(void)
{
	gpio = ioremap(MPC52xx_PA(MPC52xx_GPIO_OFFSET), MPC52xx_GPIO_SIZE);
	gpio_wkup = ioremap(MPC52xx_PA(MPC52xx_GPIO_WKUP_OFFSET),
	                    MPC52xx_GPIO_WKUP_SIZE);
	init_MUTEX(&sema);

	printk(KERN_INFO "SPI: MPC52xx SPI driver\n");
	return platform_driver_register(&mpc52xx_spi_platform_driver);
}

#ifdef MODULE
static void __exit mpc52xx_spi_exit(void)
{
	platform_driver_unregister(&mpc52xx_spi_platform_driver);
	iounmap(gpio);
	iounmap(gpio_wkup);
}
#endif

#ifndef MODULE
subsys_initcall(mpc52xx_spi_init);
#else
module_init(mpc52xx_spi_init);
module_exit(mpc52xx_spi_exit);
#endif

EXPORT_SYMBOL(mpc52xx_spi_transfer);
EXPORT_SYMBOL(mpc52xx_spi_pause);
EXPORT_SYMBOL(mpc52xx_spi_resume);

MODULE_LICENSE("GPL");
