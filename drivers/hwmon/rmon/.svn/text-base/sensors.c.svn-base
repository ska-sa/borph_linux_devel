#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include "rmon.h"
#include "sensors.h"
#include "reg.h"

ssize_t show_sensor(struct device *dev, struct device_attribute *da, char *buf);

struct rmon_sensor_attribute rmon_sensor_attributes[] = {
   RMON_SENSOR_ATTR(in1_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_1V5AUX_V,  RMON_FS_1V5AUX_V),
   RMON_SENSOR_ATTR(in2_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_1V0_V,     RMON_FS_1V0_V),
   RMON_SENSOR_ATTR(in3_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_1V5_V,     RMON_FS_1V5_V),
   RMON_SENSOR_ATTR(in4_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_1V8_V,     RMON_FS_1V8_V),
   RMON_SENSOR_ATTR(in5_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_2V5_V,     RMON_FS_2V5_V),
   RMON_SENSOR_ATTR(in6_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_3V3_V,     RMON_FS_3V3_V),
   RMON_SENSOR_ATTR(in7_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_5V0_V,     RMON_FS_5V0_V),
   RMON_SENSOR_ATTR(in8_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_voltage, RMON_VOLTAGE, RMON_AQ_12V_V,     RMON_FS_12V_V),
   RMON_SENSOR_ATTR(curr1_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_current, RMON_CURRENT, RMON_AQ_1V0_I,     RMON_FS_1V0_I),
   RMON_SENSOR_ATTR(curr2_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_current, RMON_CURRENT, RMON_AQ_1V5_I,     RMON_FS_1V5_I),
   RMON_SENSOR_ATTR(curr3_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_current, RMON_CURRENT, RMON_AQ_1V8_I,     RMON_FS_1V8_I),
   RMON_SENSOR_ATTR(curr4_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_current, RMON_CURRENT, RMON_AQ_2V5_I,     RMON_FS_2V5_I),
   RMON_SENSOR_ATTR(curr5_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_current, RMON_CURRENT, RMON_AQ_3V3_I,     RMON_FS_3V3_I),
   RMON_SENSOR_ATTR(curr6_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_current, RMON_CURRENT, RMON_AQ_5V0_I,     RMON_FS_5V0_I),
   RMON_SENSOR_ATTR(temp1_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_temp,    RMON_TEMP,    RMON_AQ_V5_T,      0),
   RMON_SENSOR_ATTR(temp2_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_temp,    RMON_TEMP,    RMON_AQ_PPC_T,     0),
   RMON_SENSOR_ATTR(temp3_input, S_IRUGO, show_sensor, NULL, rmon_get_aq_temp,    RMON_TEMP,    RMON_AQ_MON_T,     0),
   RMON_SENSOR_ATTR(power1_input, S_IRUGO, show_sensor, NULL, rmon_get_powergood,  RMON_LOGICAL, RMON_PM_3V3AUX_PG, 0),
   RMON_SENSOR_ATTR(power2_input, S_IRUGO, show_sensor, NULL, rmon_get_powergood,  RMON_LOGICAL, RMON_PM_MGTPLL_PG, 0),
   RMON_SENSOR_ATTR(power3_input, S_IRUGO, show_sensor, NULL, rmon_get_powergood,  RMON_LOGICAL, RMON_PM_MGTVTT_PG, 0),
   RMON_SENSOR_ATTR(power4_input, S_IRUGO, show_sensor, NULL, rmon_get_powergood,  RMON_LOGICAL, RMON_PM_MGTVCC_PG, 0),
   RMON_SENSOR_ATTR(fan1_input, S_IRUGO, show_sensor, NULL, rmon_get_fan,        RMON_FAN,     RMON_FC_V5_FAN,    0),
   RMON_SENSOR_ATTR(fan2_input, S_IRUGO, show_sensor, NULL, rmon_get_fan,        RMON_FAN,     RMON_FC_CASE_FAN,  0),
   RMON_SENSOR_ATTR(dummy_input, S_IRUGO, show_sensor, NULL, NULL,                0,            0,                 0)};


/***** Sensor Acquire Functions *******/
int rmon_get_aq_voltage(struct rmon_data *data, int *value, int which, int fullscale)
{
  unsigned short rmon_value;
  mutex_lock(&data->lock);
  rmon_read_single(data, RMON_REG_AQ + which, &rmon_value);
  mutex_unlock(&data->lock);
  (*value) = (((int)rmon_value) * fullscale) / (4096);
  return 0;
}

int rmon_get_aq_current(struct rmon_data *data, int *value, int which, int fullscale)
{
  unsigned short rmon_value;
  mutex_lock(&data->lock);
  rmon_read_single(data, RMON_REG_AQ + which, &rmon_value);
  mutex_unlock(&data->lock);
  (*value) = ((((int)rmon_value) - RMON_CUR_OFFSET)  * fullscale) / (4096);
  return 0;
}

int rmon_get_aq_temp(struct rmon_data *data, int *value, int which, int unused)
{
  unsigned short rmon_value;
  mutex_lock(&data->lock);
  rmon_read_single(data, RMON_REG_AQ + which, &rmon_value);
  mutex_unlock(&data->lock);
  (*value) = ((((int)rmon_value)) / (4) - 273 - 5) * 1000;
  return 0;
}

int rmon_get_powergood(struct rmon_data *data, int *value, int which, int unused)
{
  unsigned short rmon_value;
  mutex_lock(&data->lock);
  rmon_read_single(data, RMON_REG_PG, &rmon_value);
  mutex_unlock(&data->lock);
  (*value) = (((int)rmon_value) & (1 << which)) != 0;
  return 0;

}

int rmon_get_fan(struct rmon_data *data, int *value, int which, int unused)
{
  unsigned short rmon_value;
  mutex_lock(&data->lock);
  rmon_read_single(data, RMON_REG_FANSENSE + which, &rmon_value);
  mutex_unlock(&data->lock);
  (*value) = (((int)rmon_value)) * (60);
  return 0;
}

ssize_t show_sensor(struct device *dev, struct device_attribute *da, char *buf)
{
  struct rmon_sensor_attribute *attr = to_rmon_sensor_attr(da);
  struct i2c_client *client = to_i2c_client(dev);
  struct rmon_data *data = i2c_get_clientdata(client);
  int value;
  attr->rmsensor.get(data, &value, attr->rmsensor.get_which, attr->rmsensor.get_arg);
  return sprintf(buf, "%d %s\n", value, rmon_sensor_unit[attr->rmsensor.type]);
}

int rmon_sensors_attributes_init(struct rmon_data* data)
{
  int n = 0;
  int i;

  /* Get length of sensor list */
  while (1) {
    if (rmon_sensor_attributes[n].rmsensor.get == NULL){
      break;
    }
    n++;
  }

  if (!n){
    printk(KERN_ERR "rmon_sensors: no sensors present\n");
    return -1;
  }

  data->attribute_list = kmalloc((n+1)*(sizeof(struct attribute*)), GFP_KERNEL);
  if (data->attribute_list == NULL){
    printk(KERN_ERR "rmon_sensors: could not allocate memory for sensor attributes\n");
    return -ENOMEM; 
  }

  for (i=0; i < n; i++){
    data->attribute_list[i] = &(rmon_sensor_attributes[i].dev_attr.attr);
  }

  data->attribute_list[n] = NULL;
  data->rmon_sensor_group.attrs = data->attribute_list;
  printk(KERN_INFO "rmon_sensors: initialized %d sensors\n", n);
  return 0;
}

void rmon_sensors_attribute_clear(struct rmon_data* data)
{
  kfree(data->attribute_list);
}

/******** Sensors init and exit *********/

int rmon_sensors_init(struct i2c_client *client)
{
	/* Register sysfs sensor hooks */
	struct rmon_data *data = i2c_get_clientdata(client);
  int ret;

  if ((ret=rmon_sensors_attributes_init(data))){
    return ret;
  }

	if ((ret = sysfs_create_group(&client->dev.kobj, &data->rmon_sensor_group))){
    rmon_sensors_attribute_clear(data);
    return ret;
  }

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		sysfs_remove_group(&client->dev.kobj, &data->rmon_sensor_group);
    rmon_sensors_attribute_clear(data);
    return ret;
  }

  return 0;
}

void rmon_sensors_exit(struct i2c_client *client)
{
	struct rmon_data *data = i2c_get_clientdata(client);
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &data->rmon_sensor_group);
  rmon_sensors_attribute_clear(data);
}
