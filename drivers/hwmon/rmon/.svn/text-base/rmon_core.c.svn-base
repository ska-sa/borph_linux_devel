#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/reboot.h>

#include "rmon.h"
#include "reg.h"
#include "rtc.h"

static struct i2c_driver rmon_driver;

extern int rmon_sensors_init(struct i2c_client *client);
extern int rmon_sensors_exit(struct i2c_client *client);

static struct workqueue_struct *workqueue;

/* I2C macro which magically generated structure called "addr_data" */
static const unsigned short normal_i2c[] = { RMON_I2C_ADDR, I2C_CLIENT_END };
I2C_CLIENT_INSMOD_1(rmon);

/*********** IRQ Handler ***********/

irqreturn_t rmon_irq(int irq, void *dev_id)
{
  struct rmon_data *data = (struct rmon_data*)dev_id;

  /* Schedule irq work */
  
  queue_delayed_work(workqueue, &data->dwork, msecs_to_jiffies(1));
  /* Note that this irq MUST be edge triggered as clearing
     can only happen in the delayed work */

  return IRQ_HANDLED;
}

void post_irq_work(struct work_struct *work)
{
  struct rmon_data *data = container_of(work, struct rmon_data, dwork.work);
  unsigned short irq_src = 0;
  unsigned short irq_mask = 0;
  int ack_backoff = 0;
  int do_shutdown = 0;

  mutex_lock(&data->lock);
  if (rmon_read_single(data, RMON_REG_IRQFLG, &irq_src)){
    printk(KERN_ERR "rmon: warning - failed to get irq source\n"); 
  }
  if (rmon_read_single(data, RMON_REG_IRQMSK, &irq_mask)){
    printk(KERN_ERR "rmon: warning - failed to get irq mask\n"); 
  }
  mutex_unlock(&data->lock);
  printk(KERN_DEBUG "rmon: irq src=%x, msk=%x\n",irq_src,irq_mask); 

  /* The mask is used to generate the IRQ, not to latch the src reg.
     So we mask out the bits which didn't cause the IRQ. */
  irq_src = irq_src & irq_mask;

  if (rmon_is_irq_user(irq_src)){
    printk(KERN_DEBUG "rmon: got user IRQ\n"); 
  }
  if (rmon_is_irq_buserr(irq_src)){
    printk(KERN_DEBUG "rmon: got bus error\n"); 
  }
  if (rmon_is_irq_busviol(irq_src)){
    printk(KERN_DEBUG "rmon: got bus violation\n"); 
  }
  if (rmon_is_irq_health(irq_src)){
    printk(KERN_DEBUG "rmon: got system health warning\n"); 
    ack_backoff = 1000; /* back-off for 1000 ms to avoid swamping */
  }
  if (rmon_is_irq_power(irq_src)){
    printk(KERN_INFO "rmon: got chassis power button activity, shutting down\n");
    do_shutdown = 1;
    ack_backoff = 1; /* back-off for 500 ms to avoid swamping */
    /* Disable further power-down IRQs */
  }

  if (do_shutdown) {
    orderly_poweroff(0);
    msleep(500);
    if (rmon_write_single(data, RMON_REG_IRQMSK, irq_mask & (~RMON_IRQ_POWER))){
      printk(KERN_ERR "rmon: warning - failed to clear powerdown mask\n"); 
    }
  }

  msleep(ack_backoff);
  mutex_lock(&data->lock);
  if (rmon_write_single(data, RMON_REG_IRQFLG, 0)){
    printk(KERN_ERR "rmon: warning - failed to clear irq reg\n"); 
  }
  mutex_unlock(&data->lock);
}

/*********** I2C Access functions ***********/

void rmon_decode_error(int code)
{
  switch (code){
    case RMON_RESP_IDLE:
      printk(KERN_ERR "rmon: got bad read response, code = %d [Idle on response or busy]\n", code); 
      break;
    case RMON_RESP_CMDERR:
      printk(KERN_ERR "rmon: got bad read response, code = %d [Unknown command]\n", code); 
      break;
    case RMON_RESP_BUSERR:
      printk(KERN_ERR "rmon: got bad read response, code = %d [Monitor bus error]\n", code); 
      break;
    case RMON_RESP_OVER:
      printk(KERN_ERR "rmon: got bad read response, code = %d [Buffer overflow]\n", code); 
      break;
    default:
      printk(KERN_ERR "rmon: got bad read response, code = %d [Unknown]\n", code); 
      break;
   }
}

void rmon_write_flush(struct rmon_data *data){
  int i;
  unsigned char buf = RMON_CMD_NOP;
  for (i=0; i < 128; i++){
    i2c_master_send(&data->client, &buf, 1);
  }
}

void rmon_read_flush(struct rmon_data *data){
  int i;
  unsigned char buf;
  for (i=0; i < 128; i++){
    i2c_master_send(&data->client, &buf, 1);
  }
}

int rmon_write_single(struct rmon_data *data, unsigned short addr, unsigned short value)
{
  unsigned char buf[5] = {RMON_CMD_WRITE, addr & 0xFF, (addr & 0xFF00) >> 8,
                          value & 0xFF, (value & 0xFF00) >> 8};
  struct i2c_client *client = &data->client;
  int ret;

  if ((ret = i2c_master_send(client, buf, 5)) <= 0) {
    printk(KERN_ERR "rmon: write i2c_master_send error, code = %d\n", ret); 
    goto exit_cleanup;
  }

  if ((ret = i2c_master_recv(client, buf, 1)) <= 0) {
    printk(KERN_ERR "rmon: write i2c_master_recv error, code = %d\n", ret); 
    goto exit_cleanup;
  }

  ret = 0;
  switch (buf[0]){
    case RMON_RESP_ACK:
      break;
    default:
      rmon_decode_error(buf[0]);
      ret = 1;
      break;
  }
  return ret;

exit_cleanup:
  /* 
     If there is an I2C error for some reason make sure we clean up
     or remainder will screw up subsequent access
  */
     
  rmon_write_flush(data);
  rmon_read_flush(data);
  return ret;
}

int rmon_read_single(struct rmon_data *data, unsigned short addr, unsigned short *value)
{
  unsigned char buf[3] = {RMON_CMD_READ, addr & 0xFF, (addr & 0xFF00) >> 8};
  struct i2c_client *client = &data->client;
  int ret;

  if ((ret = i2c_master_send(client, buf, 3)) <= 0) {
    printk(KERN_ERR "rmon: read i2c_master_send error, code = %d\n", ret); 
    goto exit_cleanup;
  }

  if ((ret = i2c_master_recv(client, buf, 3)) <= 0) {
    printk(KERN_ERR "rmon: read i2c_master_recv error, code = %d\n", ret); 
    goto exit_cleanup;
  }

  ret = 0;
  switch (buf[0]){
    case RMON_RESP_ACK:
      *value = buf[1] | (buf[2] << 8);
      break;
    default:
      rmon_decode_error(buf[0]);
      ret = 1;
      break;
  }
  return ret;

exit_cleanup:
  /* 
     If there is an I2C error for some reason make sure we clean up
     or remainder will screw up subsequent access
  */
     
  rmon_write_flush(data);
  rmon_read_flush(data);
  return ret;
}

static char *aq_channel_name[32] = {
        "1V5AUX",
        "unused", "unused",     "V5 temp",
        "12VATX", "12VATX",     "unused",
        "5VATX",  "5VATX Cur",  "PPC temp",
        "3V3ATX", "3V3ATX Cur", "unused",
        "1V",     "1V Cur",     "unused",
        "unused", "unused",     "unused",
        "unused", "unused",     "unused",
        "1V5",    "1V5 Cur",    "unused",
        "1V8",    "1V8 Cur",    "unused",
        "2V5",    "2V5 Cur",    "unused",
        "Monitor temp"};

#define GET_AQ_CHANNEL_NAME(x) (x >= 32 ? "error" : aq_channel_name[x])


int rmon_get_crashinfo(struct rmon_data *data)
{
  unsigned short value;
  if (rmon_read_single(data, RMON_REG_CRSHCTL, &value)){
    return -1;
  }
  if (value){
    printk(KERN_ERR "rmon: warning - non-zero crash control register (%d); have you crashed recently?\n", value);
    if (rmon_read_single(data, RMON_REG_CRSHSRC, &value)){
      return -1;
    }
    printk(KERN_ERR "rmon: last crashed cause by channel %d - %s \n", value, GET_AQ_CHANNEL_NAME(value));
  }
  return 0;
}

int rmon_get_revinfo(struct rmon_data *data)
{
  int ret;
  unsigned short bid, rev_maj, rev_min, rev_rcs;

  if ((ret=rmon_read_single(data, RMON_REG_BID, &bid))){
    return ret;
  }

  if ((ret=rmon_read_single(data, RMON_REG_REVMAJ, &rev_maj))){
    return ret;
  }

  if ((ret=rmon_read_single(data, RMON_REG_REVMIN, &rev_min))){
    return ret;
  }

  if ((ret=rmon_read_single(data, RMON_REG_REVRCS, &rev_rcs))){
    return ret;
  }

  if (bid == RMON_ROACH_BID){
    printk(KERN_INFO "rmon: found Roach Monitor - Rev %d.%d.%d\n", rev_maj, rev_min, rev_rcs);
    return 0;
  } else {
    printk(KERN_ERR "rmon: bad board ID, %x\n", bid);
    return 1;
  }
}

/*********** Driver Init and Exit ***********/

int rmon_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *client;
	struct rmon_data *data;
	int err = 0;
	const char *name = "rmon";

	/* Check if we have a valid client*/
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
		goto exit;


	data = kzalloc(sizeof(struct rmon_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
  }

  workqueue = create_singlethread_workqueue("krmond");
  if (!workqueue){
    err = -ENOMEM;
    goto exit_free;
  }
  INIT_DELAYED_WORK(&data->dwork, post_irq_work);

  /* Set i2c client data */
	client = &data->client;
	i2c_set_clientdata(client, data);
	client->addr = address;
	client->adapter = adapter;
	client->driver = &rmon_driver;
	strlcpy(client->name, name, I2C_NAME_SIZE);

  /* Create mutex */
	mutex_init(&data->lock);
  /* TODO: destroy mutex */

  mutex_lock(&data->lock);
  err = rmon_get_revinfo(data);

  if (err){
    printk(KERN_ERR "rmon: warning - failed to identify Roach Monitor\n");
  }
  err = rmon_get_crashinfo(data);
  if (err < 0){
    printk(KERN_ERR "rmon: error getting crash info\n");
  }

  if (rmon_write_single(data, RMON_REG_IRQFLG, 0)){
    printk(KERN_ERR "rmon: warning - failed to setup irq register\n"); 
  }
  if (rmon_write_single(data, RMON_REG_IRQMSK, RMON_IRQ_POWER)){
    printk(KERN_ERR "rmon: warning - failed to setup irq mask\n"); 
  }

  mutex_unlock(&data->lock);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(client))){
		goto exit_workq;
  }

	/* Register sensors */
  if ((err = rmon_sensors_init(client))){
    goto exit_detach;
  }

	/* Register rtc */
  if ((err = rmon_rtc_init(client))){
    goto exit_sensors;
  }
  
  data->irq_num = PPC_440EPX_IRQ_1;

  if ((err = request_irq(data->irq_num, rmon_irq, 0, "rmon", (void*)(data)))){
    printk(KERN_ERR "rmon: error, failed to enable IRQ\n");
    goto exit_rtc;
  }

	return 0;

exit_rtc:
	rmon_rtc_exit(client);
exit_sensors:
	rmon_sensors_exit(client);
exit_detach:
	i2c_detach_client(client);
exit_workq:
  destroy_workqueue(workqueue);
exit_free:
	kfree(data);
exit:
	return err;
}


static int rmon_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, rmon_detect);
}

static int rmon_detach_client(struct i2c_client *client)
{
  struct rmon_data *data = i2c_get_clientdata(client);
  free_irq(data->irq_num, (void*)(data));
  rmon_rtc_exit(client);
  rmon_sensors_exit(client);
	i2c_detach_client(client);
  destroy_workqueue(workqueue);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static struct i2c_driver rmon_driver = {
	.driver = {
		.name = "rmon",
	},
	/*legacy binding model*/
	.attach_adapter		=	rmon_attach_adapter,
	.detach_client		=	rmon_detach_client,
};

static int __init rmon_init(void)
{
	return i2c_add_driver(&rmon_driver);
}

static void __exit rmon_exit(void)
{
	i2c_del_driver(&rmon_driver);
}

MODULE_AUTHOR("David George and Shanly Rajan");
MODULE_DESCRIPTION("Roach Monitor Driver");
MODULE_LICENSE("GPL");

module_init(rmon_init);
module_exit(rmon_exit);

