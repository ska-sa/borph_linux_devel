/*
 * Copyright (C) 2004 Promess Incorporated
 * Copyright (C) 2007 DENX Software Engineering
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * I2C device driver for the AD7416 temperature sensor
 *
 * Integrated into Linux source tree by Piotr Kruszynski <ppk@semihalf.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>		/* get the user-level API */
#include <linux/i2c.h>
#include <linux/ioctl.h>

#define DRV_NAME	"ad7416"
#define DRV_DESC	"AD7416 temperature sensor driver"
#define DRV_VERSION	"1.0.3"
#define DRV_DATE	"11-APR-07"

#define HW_NAME			"ad7416"
#define I2C_DRIVERID_AD7416	73
#define I2C_AD7416_MAJOR	195
#define AD7416_DEV		0x49

/* Registers definition */
#define AD7416_TEMP		0	/* Read-Only */
#define AD7416_CONFIG		1	/* Read/Write */
#define AD7416_THYST		2	/* Read/Write */
#define AD7416_TOTI		3	/* Read/Write */

/*
 * Definitions for ioctl
 */

/* Use I2C-AD7416_MAJOR as magic number */
#define I2C_AD7416_IOC_MAGIC	I2C_AD7416_MAJOR
#define I2C_AD7416_IOC_RESET	_IO(I2C_AD7416_IOC_MAGIC, 0)
#define I2C_AD7416_IOC_GETTEMP	_IOWR(I2C_AD7416_IOC_MAGIC, 1, short)
#define I2C_AD7416_IOC_MAXNR	1

/* Addresses to scan */
static unsigned short normal_i2c[]	= { AD7416_DEV, I2C_CLIENT_END };
static unsigned short ignore		= I2C_CLIENT_END;

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

struct ad7416_info {
	struct mutex ad7416_lock;
	struct i2c_client *client;
};

static struct ad7416_info *info;

static int ad7416_probe(struct i2c_adapter *adap);
static int ad7416_detach(struct i2c_client *device);
static int ad7416_command(struct i2c_client *device, unsigned int cmd,
				void *arg);

static struct i2c_driver i2c_driver_ad7416 = {
	.driver = {
		.name	= HW_NAME,
	},
	.id		= I2C_DRIVERID_AD7416,
	.attach_adapter	= ad7416_probe,
	.detach_client	= ad7416_detach,
	.command	= ad7416_command,
};

static short ad7416_read_temp(void)
{
	char msg[2];
	short temp;

	/* Ask for temperature */
	msg[0] = AD7416_TEMP;

	mutex_lock(&info->ad7416_lock);
	i2c_master_send(info->client, msg, 1);
	i2c_master_recv(info->client, msg, 2);
	mutex_unlock(&info->ad7416_lock);

	temp = msg[0];
	temp = (temp << 8) | msg[1];
	return temp;
}

/* attach to an instance of the device that was probed on a bus */
static int ad7416_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct ad7416_info *m;
	struct i2c_client *client;
	int err;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (client == NULL)
		return -ENOMEM;
	client->adapter = adap;
	client->addr = addr;
	client->driver = &i2c_driver_ad7416;
	sprintf(client->name, "%s-%x", HW_NAME, addr);
	if ((err = i2c_attach_client(client)) < 0) {
		kfree(client);
		return err;
	}

	m = kmalloc(sizeof(*m), GFP_KERNEL);
	if (m == NULL) {
		i2c_detach_client(client);
		kfree(client);
		return -ENOMEM;
	}

	m->client = client;
	info = m;
	mutex_init(&info->ad7416_lock);
	return 0;
}

static int ad7416_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &ad7416_attach);
}

static int ad7416_detach(struct i2c_client *device)
{
	int rc;

	if ((rc = i2c_detach_client(device)) != 0) {
		printk(HW_NAME "detach failed: %d\n", rc);
	} else
		kfree(device);
	return rc;
}

static int ad7416_command(struct i2c_client *device, unsigned int cmd,
				void *arg)
{
	return 0;
}


static int i2c_ad7416_open(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);

	if (minor != 0)
		return -ENODEV;
	return 0;
}

static int i2c_ad7416_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t i2c_ad7416_read(struct file *filp, char *buf,
				size_t count, loff_t *f_pos)
{
	return -EOPNOTSUPP;
}

static ssize_t i2c_ad7416_write(struct file *filp, const char *buf,
				size_t count, loff_t *f_pos)
{
	return -EOPNOTSUPP;
}

static int i2c_ad7416_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long arg)
{
	int err = 0;
	short ad7416_temp;

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != I2C_AD7416_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > I2C_AD7416_IOC_MAXNR) return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;


	switch(cmd) {
		case I2C_AD7416_IOC_RESET:
			/* Do nothing */
			break;
		case I2C_AD7416_IOC_GETTEMP:
			/* Get: arg is pointer to result */
			ad7416_temp = ad7416_read_temp();
			if (copy_to_user((char *)arg, (char *)&(ad7416_temp),
						sizeof(short)))
				return -EFAULT;
			break;
		default:
			return -ENOTTY;
	}
	return 0;

}

/*
 * File operations supported by this driver.
 */
static struct file_operations i2c_ad7416_fops = {
	.owner		= THIS_MODULE,
	.open		= i2c_ad7416_open,
	.release	= i2c_ad7416_release,
	.read		= i2c_ad7416_read,
	.write		= i2c_ad7416_write,
	.ioctl		= i2c_ad7416_ioctl,
};

static int i2c_ad7416_read_proc(char *buf, char **start, off_t offset,
		int count, int *eof, void *data)
{
	int len;
	short temp;

	temp = ad7416_read_temp();
	len = snprintf(buf, count, "%s %s %s, major: %d\ntemp: %d\n",
		DRV_DESC, DRV_VERSION, DRV_DATE, I2C_AD7416_MAJOR,
		(int)((temp >> 8) & 0xff));
	*eof = 1;
	return len;
}

static void i2c_ad7416_create_proc(void)
{
	struct proc_dir_entry *p;

	p = create_proc_read_entry("ad7416", 0 /* default mode */,
			NULL /* parent dir */, i2c_ad7416_read_proc,
			NULL /* client data */);
	if (p == NULL)
		printk(KERN_WARNING "Unable to initialize /proc/%s entry\n",
			DRV_NAME);
}

static void i2c_ad7416_remove_proc(void)
{
	/* no problem if it was not registered */
	remove_proc_entry("ad7416", NULL /* parent dir */);
}

static int init_ad7416(void)
{
	int err = 0;

	err = i2c_add_driver(&i2c_driver_ad7416);
	if (err < 0) {
		printk(KERN_ALERT "ERROR: Couldn't add driver %s\n", HW_NAME);
		return err;
	}

	/*
	 * Finally register the driver.
	 */
	err = register_chrdev(I2C_AD7416_MAJOR, DRV_NAME, &i2c_ad7416_fops);
	if (err < 0) {
		printk(KERN_ALERT "ERROR: Couldn't register driver %s"
			"major: %d\n\n",DRV_DESC, I2C_AD7416_MAJOR);
		i2c_del_driver(&i2c_driver_ad7416);
		return err;
	}

	i2c_ad7416_create_proc();
	printk(KERN_INFO "%s temperature sensor driver initialized\n",
		HW_NAME);
	return 0;
}

static void cleanup_ad7416(void)
{
	i2c_ad7416_remove_proc();
	unregister_chrdev(I2C_AD7416_MAJOR, DRV_NAME);
	i2c_del_driver(&i2c_driver_ad7416);
}

MODULE_AUTHOR ("Robert McCullough");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);

module_init(init_ad7416);
module_exit(cleanup_ad7416);
