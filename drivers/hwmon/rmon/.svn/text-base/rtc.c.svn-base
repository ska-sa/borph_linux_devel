#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include "rmon.h"
#include "reg.h"

static int rmon_rtc_read_time(struct device *dev, struct rtc_time *rt)
{
  struct i2c_client *client = to_i2c_client(dev);
  struct rmon_data *data = i2c_get_clientdata(client);
  unsigned short raw_time[3];
  unsigned long time;
  int ret;
  mutex_lock(&data->lock);
  if ((ret = rmon_read_single(data, RMON_REG_TIME + 0, &raw_time[0])))
    goto exit_err;
  if ((ret = rmon_read_single(data, RMON_REG_TIME + 1, &raw_time[1])))
    goto exit_err;
  if ((ret = rmon_read_single(data, RMON_REG_TIME + 2, &raw_time[2])))
    goto exit_err;
  mutex_unlock(&data->lock);
  /* 32 bit time only */
  time = (raw_time[1] << (1*16)) + (raw_time[2] << (0*16));
  rtc_time_to_tm(time, rt);
  return 0;

exit_err:
  mutex_unlock(&data->lock);
  return ret;
}

static int rmon_rtc_set_time(struct device *dev, struct rtc_time *rt)
{
  struct i2c_client *client = to_i2c_client(dev);
  struct rmon_data *data = i2c_get_clientdata(client);
  unsigned long time;
  int ret;

  rtc_tm_to_time(rt, &time);

  mutex_lock(&data->lock);
  /* 32 bit time only */
  if ((ret = rmon_write_single(data, RMON_REG_TIME + 0, 0)))
    goto exit_err;
  if ((ret = rmon_write_single(data, RMON_REG_TIME + 1, (time >> (1*16)) & 0xffff)))
    goto exit_err;
  if ((ret = rmon_write_single(data, RMON_REG_TIME + 2, (time >> (0*16)) & 0xffff)))
    goto exit_err;
  mutex_unlock(&data->lock);
  return 0;

exit_err:
  mutex_unlock(&data->lock);
  return ret;
}

/******** RTC init and exit *********/

/* Only support setting and getting time */
static const struct rtc_class_ops rmon_rtc_ops = {
  .read_time  = rmon_rtc_read_time,
  .set_time = rmon_rtc_set_time,
  .proc   = NULL,
  .read_alarm = NULL,
  .set_alarm  = NULL
};


int rmon_rtc_init(struct i2c_client *client)
{
	/* Register sysfs sensor hooks */
	struct rmon_data *data = i2c_get_clientdata(client);

  data->rtc = rtc_device_register(client->name,
                                  &client->dev,
                                  &rmon_rtc_ops,
                                  THIS_MODULE);

  if (IS_ERR(data->rtc)) {
    return PTR_ERR(data->rtc);
  }

  return 0;
}

void rmon_rtc_exit(struct i2c_client *client)
{
	struct rmon_data *data = i2c_get_clientdata(client);
  if (data->rtc)
    rtc_device_unregister(data->rtc);

}
