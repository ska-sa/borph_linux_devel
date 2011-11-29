/*
 * Copyright (C) 2003-2004 Stefano Barbato <stefano@codesink.org>
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
 * I2C device driver for the 24C01A EEPROM.
 *
 * Integrated into Linux source tree by Piotr Kruszynski <ppk@semihalf.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>

#define DRV_NAME		"eeprom"
#define DRV_DESC		"24C01A EEPROM driver"
#define DRV_VERSION		"2.1.0"
#define DRV_DATE		"11-APR-07"
#define I2C_EEPROM_MAJOR	194

#define HW_NAME			"24c01a"
#define I2C_DRIVERID_EEPROM24	80
#define EEPROM_SIZE		128
#define EEPROM_PAGE_SIZE	2
#define WRITE_DELAY		5	/* 2ms per page + 3ms extra delay */

/* Addresses to scan */
static unsigned short normal_i2c[]	= { 0x50, 0x52, 0x54, I2C_CLIENT_END };
#define MAX_DEVICES	((sizeof(normal_i2c) / sizeof(normal_i2c[0])) - 1)

static unsigned short ignore		= I2C_CLIENT_END;

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

struct eeprom_info {
	struct mutex eeprom_lock;
	struct i2c_client *client;
	int devno;
	u8 data[EEPROM_SIZE + 1];
	u8 attached;
};

static struct eeprom_info info[MAX_DEVICES];

static int eeprom_probe(struct i2c_adapter *adap);
static int eeprom_detach(struct i2c_client *device);
static int eeprom_command(struct i2c_client *device, unsigned int cmd,
				void *arg);

static struct i2c_driver i2c_driver_eeprom = {
	.driver		= {
		.name	= HW_NAME,
	},
	.id		= I2C_DRIVERID_EEPROM24,
	.attach_adapter	= eeprom_probe,
	.detach_client	= eeprom_detach,
	.command	= eeprom_command,
};

static int eeprom_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	int idx = -1;
	int err;
	int i;

	for (i = 0; normal_i2c[i] != I2C_CLIENT_END; i++) {
		if (addr == normal_i2c[i]) {
			idx = i;
			break;
		}
	}

	if (idx < 0)
		return -ENODEV;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (client == NULL)
		return -ENOMEM;
	client->adapter = adap;
	client->addr = addr;
	client->driver = &i2c_driver_eeprom;
	sprintf(client->name, "%s-%x", HW_NAME, addr);
	if ((err = i2c_attach_client(client)) < 0) {
		kfree(client);
		return err;
	}

	info[idx].client = client;
	info[idx].attached = 1;
	mutex_init(&info[idx].eeprom_lock);
	return 0;
}

static int eeprom_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &eeprom_attach);
}

static int eeprom_detach(struct i2c_client *device)
{
	int rc = 0;
	int idx = -1;
	int i;

	for (i = 0; normal_i2c[i] != I2C_CLIENT_END; i++) {
		if (device->addr == normal_i2c[i]) {
			idx = i;
			break;
		}
	}

	if (idx < 0) {
		printk(HW_NAME "eeprom_detach unknown address: 0x%x\n",
			device->addr);
	} else
		info[idx].attached = 0;

	if ((rc = i2c_detach_client(device)) != 0) {
		printk(HW_NAME "detach failed: %d\n", rc);
	} else
		kfree(device);

	return rc;
}

static int eeprom_command(struct i2c_client *device, unsigned int cmd,
				void *arg)
{
	return 0;
}

static int i2c_eeprom_open(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);

	if (minor >= MAX_DEVICES)
		return -ENODEV;

	if (info[minor].attached == 0)
		return -ENODEV;

	filp->private_data = &info[minor];
	return 0;
}

static int i2c_eeprom_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static ssize_t i2c_eeprom_read(struct file *filp, char *buf, size_t count,
				loff_t *f_pos)
{
	struct eeprom_info *dev = filp->private_data;
	ssize_t ret;
	u8 offset;

	if (*f_pos >= EEPROM_SIZE)
		return -EIO;

	offset = (*f_pos);
	count = ((EEPROM_SIZE - offset) < count) ?
			(EEPROM_SIZE - offset) : count;

	mutex_lock(&dev->eeprom_lock);
	ret = i2c_master_send(dev->client, &offset, sizeof(offset));
	if (ret < 0)
		goto done;
	if (ret != sizeof(offset)) {
		ret = -EIO;
		goto done;
	}

	ret = i2c_master_recv(dev->client, dev->data, count);
	if (ret < 0)
		goto done;
	if ((ret > 0) && (copy_to_user(buf, dev->data, ret) != 0)) {
		ret = -EFAULT;
		goto done;
	}

	(*f_pos) += ret;
done:
	mutex_unlock(&dev->eeprom_lock);
	return ret;
}

static ssize_t i2c_eeprom_write(struct file *filp, const char *buf,
				size_t count, loff_t *f_pos)
{
	struct eeprom_info *dev=filp->private_data;
	ssize_t left;
	ssize_t wbytes;
	ssize_t ret;
	u8 offset;

	if (*f_pos >= EEPROM_SIZE)
		return -EIO;

	offset = (*f_pos);
	count = ((EEPROM_SIZE - offset) < count) ?
			(EEPROM_SIZE - offset) : count;
	left = count;

	mutex_lock(&dev->eeprom_lock);
	if (copy_from_user(dev->data + offset + 1, buf, count)) {
		ret = -EFAULT;
		goto done;
	}

	wbytes = EEPROM_PAGE_SIZE - offset % EEPROM_PAGE_SIZE;
	wbytes = (wbytes < left) ? wbytes : left;

	while (left > 0) {
		dev->data[offset] = offset;
		ret = i2c_master_send(dev->client, dev->data + offset,
					wbytes + 1);
		if (ret < 0) {
			printk("i2c_master_send failed: %d, %d\n", ret, wbytes);
			goto done;
		}
		if (ret != wbytes + 1) {
			ret = -EIO;
			goto done;
		}

		offset += wbytes;
		left -= wbytes;
		wbytes = (EEPROM_PAGE_SIZE < left) ? EEPROM_PAGE_SIZE : left;
		udelay(WRITE_DELAY * 1000);
	}
	(*f_pos) = offset;
	ret = count;
done:
	mutex_unlock(&dev->eeprom_lock);
	return ret;
}

static struct file_operations i2c_eeprom_fops = {
	.owner		= THIS_MODULE,
	.open		= i2c_eeprom_open,
	.release	= i2c_eeprom_release,
	.read		= i2c_eeprom_read,
	.write		= i2c_eeprom_write,
};

static int i2c_eeprom_read_proc(char *buf, char **start, off_t offset,
				int count, int *eof, void *data)
{
	int len = 0;
	int i;

	len += snprintf(buf, count - len, "%s %s %s %s, major: %d\n\n"
			"minor\taddress\n", HW_NAME, DRV_DESC, DRV_VERSION,
			DRV_DATE, I2C_EEPROM_MAJOR);

	for (i = 0; i < MAX_DEVICES; i++) {
		if (info[i].attached == 0)
			continue;
		if (len >= count)
			break;
		len += snprintf(buf + len, count - len, "%d\t%x\n", i,
			info[i].client->addr);
	}
	*eof = 1;
	return len;
}

static void i2c_eeprom_create_proc(void)
{
	struct proc_dir_entry *p;

	p = create_proc_read_entry(DRV_NAME, 0 /* default mode */,
			NULL /* parent dir */, i2c_eeprom_read_proc,
			NULL /* client data */);
	if (p == NULL)
		printk(KERN_WARNING "Unable to initialize /proc/%s entry\n",
			DRV_NAME);
}

static void i2c_eeprom_remove_proc(void)
{
	/* no problem if it was not registered */
	remove_proc_entry(DRV_NAME, NULL /* parent dir */);
}

static int init_eeprom(void)
{
	int err = 0;

	err = i2c_add_driver(&i2c_driver_eeprom);
	if (err < 0) {
		printk(KERN_ALERT "ERROR: Couldn't add driver %s\n", HW_NAME);
		return err;
	}

	/*
	 * Finally register the driver.
	 */
	err = register_chrdev(I2C_EEPROM_MAJOR, DRV_NAME, &i2c_eeprom_fops);

	if (err < 0) {
		printk(KERN_ALERT "ERROR: Couldn't register driver %s,"
			" major: %d\n\n",DRV_DESC, I2C_EEPROM_MAJOR);
		i2c_del_driver(&i2c_driver_eeprom);
		return err;
	}

	i2c_eeprom_create_proc();
	printk(KERN_INFO "%s EEPROM driver initialized\n", HW_NAME);
	return 0;
}

static void cleanup_eeprom(void)
{
	i2c_eeprom_remove_proc();
	unregister_chrdev(I2C_EEPROM_MAJOR, DRV_NAME);
	i2c_del_driver(&i2c_driver_eeprom);
}

MODULE_AUTHOR ("Sreenivasa reddy, Stefano Barbato");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);

module_init(init_eeprom);
module_exit(cleanup_eeprom);
