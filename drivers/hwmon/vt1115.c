#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>

/* Addresses to scan */
static unsigned short normal_i2c[] = { 0x7a, 0x7b, 0x7c, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(vt1115);


/* Size of registers in bytes */
#define REG_SIZE		1

static int vt1115_attach_adapter(struct i2c_adapter *adapter);
static int vt1115_detect(struct i2c_adapter *adapter, int address, int kind);
static int vt1115_detach_client(struct i2c_client *client);

#define VTT11X5_INIT        15500 // 1.5500V init value for vid  VT115M_MULT
#define VTT11X5_VID40_DEC     250 // 0.0250V decrement
#define VTT11X5_VID5_DEC      125
#define VTT11X5_VID5_LIMIT   7625
#define VTT11X5_MULT             10000

static int vid_to_mvolt(int vid)
{
	int ival;
	vid = 0x3f & vid;

	if (0x20 & vid) {
		ival = VTT11X5_VID5_LIMIT - (vid * VTT11X5_VID5_DEC);
	} else {
		ival = VTT11X5_INIT - (vid * VTT11X5_VID40_DEC);
	}
	ival /= 10;
	return ival;
}

/* This is the driver that will be inserted */
static struct i2c_driver vt1115_driver = {
	.driver = {
		.name	= "vt1115",
	},
	.id		= 100,
	.attach_adapter	= vt1115_attach_adapter,
	.detach_client	= vt1115_detach_client,
};

static ssize_t show_vt1115(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	int j;

	j = i2c_smbus_read_byte_data(client, attr->index);

	if (j < 0)
		return j;
	else
		return sprintf(buf, "%02x\n", j);
}

static ssize_t show_real_millivolt(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;

	val = i2c_smbus_read_byte_data(client, 28);

	if (val < 0)
		return val;

	val = vid_to_mvolt(val);
	return sprintf(buf, "%d\n", val);
}

static ssize_t show_real_milliamp(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;

	val = i2c_smbus_read_byte_data(client, 31);

	if (val < 0)
		return val;

	val = val * 24000 / 124;
	return sprintf(buf, "%d\n", val);
}


static SENSOR_DEVICE_ATTR(sbcmd, S_IRUGO, show_vt1115, NULL, 7);
static SENSOR_DEVICE_ATTR(sid, S_IRUGO, show_vt1115, NULL, 8);
static SENSOR_DEVICE_ATTR(srdbk, S_IRUGO, show_vt1115, NULL, 10);
static SENSOR_DEVICE_ATTR(sstat, S_IRUGO, show_vt1115, NULL, 12);
static SENSOR_DEVICE_ATTR(mstat, S_IRUGO, show_vt1115, NULL, 13);
static SENSOR_DEVICE_ATTR(nslaves, S_IRUGO, show_vt1115, NULL, 14);
static SENSOR_DEVICE_ATTR(dac, S_IRUGO, show_vt1115, NULL, 24);
static SENSOR_DEVICE_ATTR(ovride, S_IRUGO, show_vt1115, NULL, 25);
static SENSOR_DEVICE_ATTR(vid, S_IRUGO, show_vt1115, NULL, 28);
static SENSOR_DEVICE_ATTR(we, S_IRUGO, show_vt1115, NULL, 29);
static SENSOR_DEVICE_ATTR(ampa, S_IRUGO, show_vt1115, NULL, 30);
static SENSOR_DEVICE_ATTR(ampp, S_IRUGO, show_vt1115, NULL, 31);
static SENSOR_DEVICE_ATTR(ampfp, S_IRUGO, show_vt1115, NULL, 32);
static SENSOR_DEVICE_ATTR(volt, S_IRUGO, show_vt1115, NULL, 33);
static SENSOR_DEVICE_ATTR(radc2, S_IRUGO, show_vt1115, NULL, 2);
static SENSOR_DEVICE_ATTR(radc3, S_IRUGO, show_vt1115, NULL, 3);
static SENSOR_DEVICE_ATTR(radc4, S_IRUGO, show_vt1115, NULL, 4);
static SENSOR_DEVICE_ATTR(radc5, S_IRUGO, show_vt1115, NULL, 5);
static SENSOR_DEVICE_ATTR(radc6, S_IRUGO, show_vt1115, NULL, 6);

static SENSOR_DEVICE_ATTR(real_millivolt, S_IRUGO, show_real_millivolt, NULL, 0);
static SENSOR_DEVICE_ATTR(real_milliamp, S_IRUGO, show_real_milliamp, NULL, 0);

static struct attribute *vt1115_attributes[] = {
	&sensor_dev_attr_sbcmd.dev_attr.attr,
	&sensor_dev_attr_sid.dev_attr.attr,
	&sensor_dev_attr_srdbk.dev_attr.attr,
	&sensor_dev_attr_sstat.dev_attr.attr,
	&sensor_dev_attr_mstat.dev_attr.attr,
	&sensor_dev_attr_nslaves.dev_attr.attr,
	&sensor_dev_attr_dac.dev_attr.attr,
	&sensor_dev_attr_ovride.dev_attr.attr,
	&sensor_dev_attr_vid.dev_attr.attr,
	&sensor_dev_attr_we.dev_attr.attr,
	&sensor_dev_attr_ampa.dev_attr.attr,
	&sensor_dev_attr_ampp.dev_attr.attr,
	&sensor_dev_attr_ampfp.dev_attr.attr,
	&sensor_dev_attr_volt.dev_attr.attr,
	&sensor_dev_attr_radc2.dev_attr.attr,
	&sensor_dev_attr_radc3.dev_attr.attr,
	&sensor_dev_attr_radc4.dev_attr.attr,
	&sensor_dev_attr_radc5.dev_attr.attr,
	&sensor_dev_attr_radc6.dev_attr.attr,
	&sensor_dev_attr_real_millivolt.dev_attr.attr,
	&sensor_dev_attr_real_milliamp.dev_attr.attr,
	NULL,
};

static const struct attribute_group vt1115_group = {
	.attrs = vt1115_attributes,
};

static int vt1115_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, vt1115_detect);
}

/* This function is called by i2c_probe */
static int vt1115_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;

	/* There are three ways we can read the REG data:
	   (1) I2C block reads (faster, but unsupported by most adapters)
	   (2) Consecutive byte reads (100% overhead)
	   (3) Regular byte data reads (200% overhead)
	   The third method is not implemented by this driver because all
	   known adapters support at least the second. */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BYTE_DATA
					    | I2C_FUNC_SMBUS_BYTE))
		goto exit;

	new_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &vt1115_driver;
	new_client->flags = 0;

	err = i2c_smbus_read_byte_data(new_client, 2);
	if (err < 0)
		goto exit_free;

	/* Fill in the remaining client fields */
	strlcpy(new_client->name, "vt1115", I2C_NAME_SIZE);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_free;

	if ((err = sysfs_create_group(&new_client->dev.kobj, &vt1115_group)))
		goto exit_detach;

	return 0;

exit_detach:
	i2c_detach_client(new_client);
exit_free:
	kfree(new_client);
exit:
	return err;
}

static int vt1115_detach_client(struct i2c_client *client)
{
	int err;

	sysfs_remove_group(&client->dev.kobj, &vt1115_group);

	err = i2c_detach_client(client);
	if (err)
		return err;

	kfree(i2c_get_clientdata(client));

	return 0;
}

static int __init vt1115_init(void)
{
	return i2c_add_driver(&vt1115_driver);
}

static void __exit vt1115_exit(void)
{
	i2c_del_driver(&vt1115_driver);
}


MODULE_DESCRIPTION("vt1115 driver");
MODULE_LICENSE("GPL");

module_init(vt1115_init);
module_exit(vt1115_exit);
