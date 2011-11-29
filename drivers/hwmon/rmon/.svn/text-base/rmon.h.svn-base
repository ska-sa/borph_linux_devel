#ifndef INCLUDE_RMON_H_
#define INCLUDE_RMON_H_

struct rmon_data{
  struct delayed_work dwork;
  struct i2c_client client;
  struct mutex lock;
  struct device *hwmon_dev;
  /* irq */
  int irq_num;
  /* sysfs sensors */
  struct attribute **attribute_list;
  struct attribute_group rmon_sensor_group;
  /* rtc */
  struct rtc_device *rtc;
};

int rmon_read_single(struct rmon_data *data, unsigned short addr, unsigned short *value);
int rmon_write_single(struct rmon_data *data, unsigned short addr, unsigned short value);

#define PPC_440EPX_IRQ_1 62
#define RMON_I2C_ADDR 0x0f

#define RMON_ROACH_BID   0xbeef

#define RMON_CMD_NOP     0x0
#define RMON_CMD_READ    0x1
#define RMON_CMD_WRITE   0x2
#define RMON_CMD_PING    0x8

#define RMON_RESP_IDLE   0x0
#define RMON_RESP_ACK    0x1
#define RMON_RESP_PING   0x8
#define RMON_RESP_CMDERR 0x253
#define RMON_RESP_BUSERR 0x254
#define RMON_RESP_OVER   0x255

#define RMON_IRQ_USER    (1<<4) 
#define RMON_IRQ_BUSERR  (1<<3)
#define RMON_IRQ_BUSVIOL (1<<2)
#define RMON_IRQ_HEALTH  (1<<1)
#define RMON_IRQ_POWER   (1<<0)

#define rmon_is_irq_user(x)   (x & RMON_IRQ_USER   )
#define rmon_is_irq_buserr(x) (x & RMON_IRQ_BUSERR )
#define rmon_is_irq_busviol(x)(x & RMON_IRQ_BUSVIOL)
#define rmon_is_irq_health(x) (x & RMON_IRQ_HEALTH )
#define rmon_is_irq_power(x)  (x & RMON_IRQ_POWER  )

#endif
