/*
 * drivers/i2c/busses/i2c-tqm82xx.c
 *
 * Author: Heiko Schocher <hs@denx.de>
 *
 * Copyright (c) 2006 DENX
 *
 * This file is licensed under the terms of the GNU General Public 
 * License version 2. This program is licensed "as is" without any 
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <asm/mpc8260.h>
#include <asm/immap_cpm2.h>
#include <asm/io.h>

#if defined(CONFIG_TQM8272)
#define SCL	0x20000	/* PD 14 */
#define SDA	0x10000	/* PD 15 */

#define PAR	immap->im_ioport.iop_ppard
#define DIR	immap->im_ioport.iop_pdird
#define DAT	immap->im_ioport.iop_pdatd
#endif

#define	MIT_PLBUS	0

static	volatile cpm2_map_t* immap;

static void tqm82xx_bit_setscl(void *data, int val)
{

	if (val)
		DAT |= SCL;
	else {
		DAT &=	~SCL;
	}
}

static void tqm82xx_bit_setsda(void *data, int val)
{

	if (val)
		DIR &= ~SDA;
	else {
		DAT &= ~SDA;
		DIR |= SDA;
	}
}

static int tqm82xx_bit_getscl(void *data)
{

	return (DAT & SCL) != 0;
}	

static int tqm82xx_bit_getsda(void *data)
{
	int res;

	res = (DAT & SDA) != 0;

	return res;
}	

static void tqm82xx_i2c_line_init (void)
{
	PAR &= ~(SCL | SDA); DIR |= SCL;
}

struct tqm82xx_i2c_data {
	struct i2c_adapter adapter;
	struct i2c_algo_bit_data algo_data;
};

static int tqm82xx_i2c_probe(struct platform_device *plat_dev)
{
	int err;
	struct tqm82xx_i2c_data *drv_data = 
		kzalloc(sizeof(struct tqm82xx_i2c_data), GFP_KERNEL);

	if(!drv_data)
		return -ENOMEM;

	immap = ioremap(CPM_MAP_ADDR, sizeof(cpm2_map_t));
	tqm82xx_i2c_line_init ();
	drv_data->algo_data.setsda = tqm82xx_bit_setsda;
	drv_data->algo_data.setscl = tqm82xx_bit_setscl;
	drv_data->algo_data.getsda = tqm82xx_bit_getsda;
	drv_data->algo_data.getscl = tqm82xx_bit_getscl;
	drv_data->algo_data.udelay = 10;
	drv_data->algo_data.timeout = 100;

	drv_data->adapter.id = I2C_HW_B_MPC;
	drv_data->adapter.class = I2C_CLASS_HWMON;
	if (plat_dev)
		strlcpy(drv_data->adapter.name, plat_dev->dev.driver->name,
		I2C_NAME_SIZE);
	else
		sprintf(drv_data->adapter.name, "i2c-bb");

	drv_data->adapter.algo_data = &drv_data->algo_data;

	drv_data->adapter.dev.parent = &platform_bus;
	if ((err = i2c_bit_add_bus(&drv_data->adapter) != 0)) {
		if (plat_dev)
			printk(KERN_ERR "ERROR: Could not install %s\n",
				plat_dev->dev.bus_id);
		else
			printk(KERN_ERR "ERROR: Could not install i2c-bb\n");

		kfree(drv_data);
		return err;
	}

	if (plat_dev)
		platform_set_drvdata(plat_dev, drv_data);

	return 0;
}

static int __init tqm82xx_i2c_init(void)
{
	int	ret = 0;

	tqm82xx_i2c_probe (NULL);
	return ret;
}

static void __exit tqm82xx_i2c_exit(void)
{
	iounmap(immap);
}

module_init(tqm82xx_i2c_init);
module_exit(tqm82xx_i2c_exit);

MODULE_DESCRIPTION("MPC82xx-based I2C adapter");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Heiko Schocher <hs@denx.de>");

