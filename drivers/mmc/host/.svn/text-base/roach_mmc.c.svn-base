/*
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/pnp.h>
#include <linux/highmem.h>
#include <linux/mmc/host.h>
#include <linux/scatterlist.h>
#include <linux/mmc/mmc.h>
#include <linux/crc7.h>
#include <linux/crc-itu-t.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/dma.h>

#include "roach_mmc.h"


#define DRIVER_NAME "roach_mmc"

#define DBG(x...) \
  pr_debug(DRIVER_NAME ": " x)
#define DBGF(f, x...) \
  pr_debug(DRIVER_NAME " [%s()]: " f, __func__ , ##x)


//#define RMMC_INFO
//#define RMMC_DEBUG

/**********************************\
 * Register Manipulation Routines *
\**********************************/

unsigned char rmmc_get_revmajor(struct rmmc_host *host)
{
  return in_8((void *)(host->base + RMMC_REG_REV_MAJ));
}

void rmmc_set_find_busy(struct rmmc_host *host)
{
  unsigned char prev = in_8((void *)(host->base + RMMC_REG_ADV));
  out_8((void *)(host->base) + RMMC_REG_ADV, prev | RMMC_FIND_BUSY);
}

void rmmc_clear_find_busy(struct rmmc_host *host)
{
  unsigned char prev = in_8((void *)(host->base + RMMC_REG_ADV));
  out_8((void *)(host->base) + RMMC_REG_ADV, prev | RMMC_FIND_BUSY);
}

void rmmc_set_clock(struct rmmc_host *host, unsigned char freq)
{
  unsigned char prev = in_8((void *)(host->base + RMMC_REG_CLK));
  out_8((void *)(host->base) + RMMC_REG_CLK, RMMC_INS_CLK(prev, freq));
}

void rmmc_set_data_width(struct rmmc_host *host, unsigned char dw)
{
  unsigned char prev = in_8((void *)(host->base + RMMC_REG_CLK));
  out_8((void *)(host->base + RMMC_REG_CLK), RMMC_INS_DW(prev, dw));
}

void rmmc_set_oes(struct rmmc_host *host, unsigned char oes)
{
  unsigned char prev = in_8((void *)(host->base + RMMC_REG_CLK));
  out_8(host->base + RMMC_REG_CLK, RMMC_INS_OE(prev, oes));
}

unsigned char rmmc_get_cd(struct rmmc_host *host)
{
  return !(in_8((void *)(host->base + RMMC_REG_STAT)) & RMMC_REG_CD);
}

unsigned char rmmc_get_ro(struct rmmc_host *host)
{
  return in_8((void *)(host->base + RMMC_REG_STAT)) & RMMC_REG_RO;
}

//Manually advance the MMC CLK
void rmmc_set_advmode(struct rmmc_host *host, unsigned char mode)
{
  unsigned char prev = in_8((void *)(host->base + RMMC_REG_ADV));
  out_8(host->base + RMMC_REG_ADV, RMMC_INS_ADV(prev, mode));
}

unsigned char rmmc_get_advmode(struct rmmc_host *host)
{
  return RMMC_GET_ADV(in_8((void *)(host->base + RMMC_REG_ADV)));
}

//Manually advance the MMC CLK
inline void rmmc_man_adv(struct rmmc_host *host)
{
  out_8(host->base + RMMC_REG_MAN, 1);
}

//Check if the MMC CLK advance is complete
unsigned char rmmc_adv_done(struct rmmc_host *host)
{
  return in_8((void *)(host->base + RMMC_REG_MAN));
}

//Set MMC Write DAT
inline void rmmc_set_dat(struct rmmc_host *host, unsigned char data)
{
  out_8(host->base + RMMC_REG_DAT, data);
}

//Set MMC Write CMD
inline void rmmc_set_cmd(struct rmmc_host *host, unsigned char data)
{
  out_8(host->base + RMMC_REG_CMD, data);
}

//Get MMC CMD
inline unsigned char rmmc_get_cmd(struct rmmc_host *host)
{
  return in_8(host->base + RMMC_REG_CMD);
}

//Get MMC Read DAT 
inline unsigned char rmmc_get_dat(struct rmmc_host *host)
{
  return in_8(host->base + RMMC_REG_DAT);
}

//Get CMD CRC
unsigned char rmmc_cmd_crc(struct rmmc_host *host)
{
  return in_8((void *)(host->base + RMMC_REG_CRC_CMD));
}


static char *rmmc_sg_to_buffer(struct scatterlist *sg)
{
  return sg_virt(sg);
}

#define RMMC_ADV_WAIT_THRESH 10000

int rmmc_wait_adv(struct rmmc_host *host)
{
  int wait_count = 0;
  while(1){
    if (rmmc_adv_done(host))
      return 0;
    if (wait_count >= RMMC_ADV_WAIT_THRESH){
      return -1;
    }
    wait_count++;
  }
}

inline int rmmc_send_cmd_byte(struct rmmc_host *host, unsigned char data)
{
  unsigned char i = 0;

  for (i = 0; i < 8; i++){
    rmmc_set_cmd(host, data >> (7 - i));
    rmmc_man_adv(host);
    if (rmmc_wait_adv(host)){
      return -1;
    }
  }
  return 0;
}

inline void rmmc_tick_clock(struct rmmc_host *host, unsigned int cycles)
{
  volatile unsigned int i;
  for (i = 0; i < cycles; i++){
    rmmc_man_adv(host);
    rmmc_wait_adv(host);
  }
  return;
}

void rmmc_init_card(struct rmmc_host *host)
{
  printk(KERN_INFO "rmmc: performing card initialization\n");
  rmmc_set_advmode(host, RMMC_ADV_NONE);
  rmmc_set_cmd(host, 0xff);
  rmmc_set_dat(host, 0xff);
  rmmc_set_oes(host, RMMC_OE_CMD | RMMC_OE_DAT);

  rmmc_tick_clock(host, 80);
  rmmc_set_oes(host, RMMC_OE_NONE);
}

unsigned char soft_crc7(struct mmc_command *cmd)
{
  unsigned char crc_data[5];
  crc_data[0] = RMMC_START_BIT | RMMC_TRANS_BIT | (cmd->opcode & 0x3f);
  crc_data[1] = (cmd->arg & 0xff000000) >> 24;
  crc_data[2] = (cmd->arg & 0x00ff0000) >> 16;
  crc_data[3] = (cmd->arg & 0x0000ff00) >> 8; 
  crc_data[4] = (cmd->arg & 0x000000ff) >> 0; 
  return crc7(0, crc_data, 5);
}

static unsigned short soft_crc16(unsigned char *crc_data, int length)
{
  unsigned short crc;

  crc = crc_itu_t(0x0 , crc_data, length);

  return crc;
}

inline int rmmc_get_data_byte(struct rmmc_host *host, int ticks, int poll)
{
  unsigned char i = 0;
  unsigned char din = 0;

  if (poll) {
    for (i=0; i < ticks; i++){
      rmmc_man_adv(host);
      if (rmmc_wait_adv(host))
        return -1;
      switch (ticks) {
        case 1:
          din = rmmc_get_dat(host);
          break;
        case 2:
          din |= (rmmc_get_dat(host) & 0xf) << (4 * (1 - i)) ;
          break;
        default:
          din |= (rmmc_get_dat(host) & 0x1) << (7 - i) ;
          break;
      }
    }
  } else {
    return rmmc_get_dat(host);
  }
  return din;
}

inline int rmmc_get_cmd_byte(struct rmmc_host *host)
{
  unsigned char i = 0;
  unsigned char din = 0;

  for (i = 0; i < 8; i++){
    rmmc_man_adv(host);
    if (rmmc_wait_adv(host))
      return -1;
    din |= (rmmc_get_cmd(host) & 0x1) << (7 - i);
  }
  return din;
}

int rmmc_get_response_data(struct rmmc_host *host, struct mmc_command *cmd)
{
  host->partial_buf_len = 0;

  /* TODO: implement responses + data. It might be risky ignoring response */

  return 0;
}

/* TODO: fix all the polling */

int rmmc_data_wait(struct rmmc_host *host, int max_ticks)
{
  int n = 0;
  while (1){
    rmmc_tick_clock(host, 1);
    if (!(rmmc_get_dat(host) & 0x1)){
#ifdef RMMC_DEBUG
      printk(KERN_DEBUG "rmmc: got data after %d tries\n", n);
#endif
      return 0;
    }
    if (n == max_ticks)
      return -ETIMEDOUT;
    n++;
  }
}

int rmmc_rdauto_wait(struct rmmc_host *host, int max_ticks)
{
  int n = 0;
  while (1){
    if ((in_8(host->base + RMMC_REG_ADV) & 0x1)){
#ifdef RMMC_DEBUG
      printk(KERN_DEBUG "rmmc: got rdauto data after %d tries\n", n);
#endif
      return 0;
    }
    if (n == max_ticks){
#ifdef RMMC_DEBUG
      printk(KERN_ERR "rmmc: read wait failed after %d tries\n", n);
#endif
      return -ETIMEDOUT;
    }
    n++;
  }
}

int rmmc_response_wait(struct rmmc_host *host, int max_ticks)
{
  int n = 0;
  while (1){
    rmmc_tick_clock(host, 1);
    if (!(rmmc_get_cmd(host) & 0x1)){
#ifdef RMMC_DEBUG
      printk(KERN_DEBUG "rmmc: got response after %d tries\n", n);
#endif
      return 0;
    }
    if (n == max_ticks){
      return -ETIMEDOUT;
    }
    n++;
  }
}

int rmmc_ready_wait(struct rmmc_host *host, int max_ticks)
{
  int n = 0;
  while (1){
    rmmc_tick_clock(host, 1);
    if (RMMC_WR_RDY(rmmc_get_data_byte(host, 1, 1))){
#ifdef RMMC_DEBUG
      printk(KERN_INFO "rmmc: got ready after %d tries\n", n);
#endif
      return 0;
    }
    if (n == max_ticks){
#ifdef RMMC_DEBUG
      printk(KERN_INFO "rmmc: no write ready after %d tries\n", n);
#endif
      return -ETIMEDOUT;
    }
    n++;
  }
}

/**********************************\
 *     MMC Protocol Routines      *
\**********************************/

int rmmc_get_response(struct rmmc_host *host, struct mmc_command *cmd)
{
  unsigned char first_byte = 0;
  int i;
  int cmd_data = 0;
  u32 resp_data = 0;
  int ret;

  rmmc_set_oes(host, RMMC_OE_NONE);
  rmmc_set_advmode(host, RMMC_ADV_NONE);

  if ((ret=rmmc_response_wait(host, RMMC_RESPONSE_WAIT_MAX)))
    return ret;

  first_byte |= (rmmc_get_cmd(host) & 0x1) << 7;
  //manually clock in 7 bits
  for (i=1; i < 8; i++) {
    rmmc_man_adv(host);
    if (rmmc_wait_adv(host)){
#ifdef RMMC_DEBUG
      printk(KERN_DEBUG "rmmc: manadv timeout\n");
#endif
      return -ETIMEDOUT;
    }
    first_byte |= (rmmc_get_cmd(host) & 0x1) << (7 - i);
  }

  /* if long resp, 16 bytes otherwise 5 bytes */
  for (i = 0; i < (cmd->flags & MMC_RSP_136 ? 16 : 5); i++){
    if ((cmd_data = rmmc_get_cmd_byte(host)) < 0){
#ifdef RMMC_DEBUG
      printk(KERN_DEBUG "rmmc: manadv timeout\n");
#endif
      return -ETIMEDOUT;
    }
    resp_data |= (((unsigned int)(cmd_data)) & 0xff) << (24 - ((i%4)*8)); 
    if ((i%4) == 3) {
      cmd->resp[(i)/4] = resp_data;
      resp_data = 0;
    }
  }

  return 0;
}

int rmmc_send_command(struct rmmc_host *host, struct mmc_command *cmd)
{
  int ret = 0;
  int soft_crc;

  if (cmd->opcode == 0) {
    rmmc_set_dat(host, 0xff);
    rmmc_set_oes(host, RMMC_OE_CMD | RMMC_OE_DAT);
  } else {
    rmmc_set_oes(host, RMMC_OE_CMD);
  }

  rmmc_set_advmode(host, RMMC_ADV_NONE);

  soft_crc = soft_crc7(cmd);

  if (rmmc_send_cmd_byte(host, RMMC_START_BIT | RMMC_TRANS_BIT | (cmd->opcode & 0x3f)) ||
      rmmc_send_cmd_byte(host, (cmd->arg & 0xff000000) >> 24)                          ||
      rmmc_send_cmd_byte(host, (cmd->arg & 0x00ff0000) >> 16)                          ||
      rmmc_send_cmd_byte(host, (cmd->arg & 0x0000ff00) >> 8)                           ||
      rmmc_send_cmd_byte(host, (cmd->arg & 0x000000ff) >> 0)){
    ret = -ETIMEDOUT;
  }

  if (rmmc_send_cmd_byte(host, (soft_crc << 1)  | RMMC_STOP_BIT)){
    ret = -ETIMEDOUT;
  }


  rmmc_set_oes(host, RMMC_OE_NONE);

  if (ret) {
#ifdef RMMC_DEBUG
    printk(KERN_DEBUG "rmmc: wait for adv_done error\n");
#endif
    return ret;
 }

 /* get response */
 if ((cmd->flags & MMC_RSP_PRESENT) && cmd->data && (cmd->data->flags & MMC_DATA_READ)) {
   ret = rmmc_get_response_data(host, cmd);
 } else if (cmd->flags & MMC_RSP_PRESENT) {
   ret = rmmc_get_response(host, cmd);
   rmmc_tick_clock(host, 3);
 } else {
   rmmc_tick_clock(host, 8);
 }

 return ret;
}

int rmmc_read_data(struct rmmc_host *host, struct mmc_data *data)
{
  int sg_index = 0; /* the current sg_buffer index */
  int sg_remain = 0; /* how much data is left in the scatter gather list */
  unsigned char *buf = NULL; /* our current buffer */
  int partial = 0; /* how many bytes have been collected prior to the block read */
  int ret = 0;
  int i;
  int ticks, poll;
  unsigned char din;

  rmmc_set_oes(host, RMMC_OE_NONE);

#ifdef RMMC_INFO
  printk(KERN_INFO "rmmc: [BLK READ] blocksize = %d, #blocks = %d, noterm = %d\n", data->blksz, data->blocks, !(data->stop));
#endif

  if (data->sg_len == 0 || !data->sg){
    printk(KERN_ERR "rmmc: error - got no sg buffers\n");
    goto done;
  }

  buf = rmmc_sg_to_buffer(&(data->sg[0]));
  sg_remain = data->sg[0].length;

  if (sg_remain == 0 || buf == NULL){
     printk(KERN_ERR "rmmc: error - empty sg buffers\n");
     ret = -ETIMEDOUT;
     goto done;
  }

  if (host->partial_buf_len){
     printk(KERN_WARNING "rmmc: got a partial buffer\n");
  }

  if ((host->clk == RMMC_CLK_375K) && (data->blksz == 512)) {
    rmmc_set_advmode(host, RMMC_ADV_NONE);
    poll = 1;
  } else {
    rmmc_set_advmode(host, RMMC_ADV_DAT_RD);
    poll = 0;
  }

  ticks = (host->bus_width == RMMC_DWIDTH_8) ? 1 : 
          ((host->bus_width == RMMC_DWIDTH_4) ? 2 : 8);

  data->bytes_xfered = 0;

  while (1){

    if (sg_remain == 0){
      if (sg_index == data->sg_len - 1){
        goto done;
      } else {
        sg_index++;
        buf = rmmc_sg_to_buffer(&(data->sg[sg_index]));
        sg_remain = data->sg[sg_index].length;

        if (sg_remain == 0 || buf == NULL){
           printk(KERN_ERR "rmmc: error - empty sg buffers\n");
           ret = -ETIMEDOUT;
           goto done;
        }
      }
    }

    if (sg_remain < (data->blksz - partial)){
      printk(KERN_ERR "rmmc: error - write buffer needs to be multiple of 512 bytes\n");
      BUG();
      ret = -EILSEQ;
      goto done;
    }

    /* First we must wait for the ready */
    if (partial == 0) {
      if (poll) {
        if ((ret = rmmc_data_wait(host, RMMC_DATA_WAIT_MAX))){
          goto done;
        }
      } else {
        if ((rmmc_rdauto_wait(host, RMMC_DATA_WAIT_MAX))){
          goto done;
        }
      }
    }

    for (i = 0; i < (data->blksz - partial); i++){
      *buf = rmmc_get_data_byte(host, ticks, poll);
      
      buf++;
      sg_remain--;
    }

    data->bytes_xfered += data->blksz;
    /* clear partial */
    partial = 0;
    /* Read CRC bytes */
    if (poll){
      din = rmmc_get_data_byte(host, 8, 1);
    } else {
      din = rmmc_get_data_byte(host, ticks, poll);
    }
    /* TODO: add CRC on reads */

    if (poll){
      din = rmmc_get_data_byte(host, 8, 1);
    } else {
      din = rmmc_get_data_byte(host, ticks, poll);
    }

    if (poll){
      din = rmmc_get_data_byte(host, 1, 1);
    } else {
      din = rmmc_get_data_byte(host, ticks, poll);
    }
  }

done:
  rmmc_set_advmode(host, RMMC_ADV_NONE);

#ifdef RMMC_INFO
  printk(KERN_INFO "rmmc: [BLK READ] bytes_xfered = %d\n", data->bytes_xfered);
#endif

  /* send Term CMD*/
  if (data->stop) {
#ifdef RMMC_DEBUG
    printk(KERN_DEBUG "rmmc: sending stop command\n");
#endif
    rmmc_send_command(host, data->stop);
  } else {
    rmmc_tick_clock(host, 4);
  }
  return ret;
}

inline int rmmc_send_data_byte(struct rmmc_host *host, unsigned char data, int ticks, int poll)
{
  unsigned char i = 0;

  if (poll) {
    for (i = 0; i < ticks; i++){
      switch (ticks) {
        case 1:
          rmmc_set_dat(host, data);
          break;
        case 2:
          rmmc_set_dat(host, data >> (4 * (1 - i)));
          break;
        default:
          rmmc_set_dat(host, data >> (7 - i));
          break;
      }
      rmmc_man_adv(host);
      if (rmmc_wait_adv(host))
        return -1;
    }
  } else {
    rmmc_set_dat(host, data);
  }
  return 0;
}

static unsigned char move_bit(unsigned char data, char frm, char to)
{
  unsigned char ret;

  if (to - frm >= 0){
    ret = ((data & (1 << frm)) << ((unsigned char)(to - frm)));
  } else {
    ret = ((data & (1 << frm)) >> ((unsigned char)(frm - to)));
  }

  return ret;
}

static unsigned char reorder_wrdata(char lane, char *data)
{
  unsigned char ret;

  ret = (move_bit(data[0], (4 + lane), 7) | move_bit(data[0],lane,6) |
         move_bit(data[1], (4 + lane), 5) | move_bit(data[1],lane,4) |
         move_bit(data[2], (4 + lane), 3) | move_bit(data[2],lane,2) |
         move_bit(data[3], (4 + lane), 1) | move_bit(data[3],lane,0) );

  return ret;
}

void assemble_crc16(unsigned short *in, unsigned short *out)
{
  int i, j; 

  for (i = 0; i < 4; i++){
    out[i] = 0;
  }
  /* 
    This reorders the 4 CRC16 shorts, corresponding to the CRC on the 4 MMC
    data lines,into 4 shorts which can be directly clocked into MMC controller
  */
  for(i = 0; i < 4; i++){
    for(j = 0; j < 16; j++){
      out[i] |= (((in[j % 4] & (1 << ((3 - i) * 4 + (j / 4)))) != 0) << j);
    }     
  }
}

int rmmc_send_crc(struct rmmc_host *host, unsigned char *data, int blocksz, int ticks, int poll)
{
  int i;
  int ret = 0;
  unsigned short soft_crc[4], crc_unord[4];
  unsigned char c_3[128],c_2[128],c_1[128],c_0[128];

  switch(ticks){
    case 2:/* 4-bit mode */
      /* Reorder data for per line CRC calulation */
      for(i = 0; i < (blocksz / 4); i++){
        c_0[i] = reorder_wrdata(0,data);
        c_1[i] = reorder_wrdata(1,data);
        c_2[i] = reorder_wrdata(2,data);
        c_3[i] = reorder_wrdata(3,data);
        data += 4;
      }

      /*Calculate CRC with reordered databits */
      crc_unord[0] = soft_crc16(c_0, 128);
      crc_unord[1] = soft_crc16(c_1, 128);
      crc_unord[2] = soft_crc16(c_2, 128);
      crc_unord[3] = soft_crc16(c_3, 128);


      /* Reorder CRC16s so they can be clocked directly */
      assemble_crc16(crc_unord, soft_crc);

      /* Clocking CRC bits out */
      for(i = 0; i < 4; i++) {
        if (rmmc_send_data_byte(host, (soft_crc[i] & 0xff00) >> 8, 2, poll)){
          printk(KERN_ERR "rmmc_send_data:CRC Tx Error\n");
          ret = -ETIMEDOUT;
        }
        if (rmmc_send_data_byte(host, (soft_crc[i] & 0x00ff) >> 0, 2, poll)){
          printk(KERN_ERR "rmmc_send_data:CRC Tx Error\n");
          ret = -ETIMEDOUT;
        }
      }
      break;
    case 8:/*1-bit mode*/
#ifdef RMMC_DEBUG
      printk(KERN_DEBUG "rmmc_send_crc:1-bit mode\n");
#endif
      soft_crc[0] = soft_crc16(data, 512);
      if (rmmc_send_data_byte(host, (soft_crc[0] & 0xff00) >> 8, 8, poll)){
        printk(KERN_ERR "rmmc_send_data:CRC Tx Error\n");
        ret = -ETIMEDOUT;
      }
      if (rmmc_send_data_byte(host, (soft_crc[0] & 0x00ff) >> 0, 8, poll)){
        printk(KERN_ERR "rmmc_send_data:CRC Tx Error\n");
        ret = -ETIMEDOUT;
      }
      break;
    default:
      ret = -EINVAL; 
      break;
  }
  return 0;
}

int rmmc_write_block(struct rmmc_host *host,unsigned char *data, int blocksz, int ticks, int poll)
{
  int i;
  int ret = 0;

  /* Sending start bit*/  
  rmmc_set_advmode(host, RMMC_ADV_NONE);
  if (rmmc_send_data_byte(host, RMMC_START_BIT, 1, 1)){
    ret = -ETIMEDOUT;
  }

  /* Sending Data bits*/  
  if (!poll)
    rmmc_set_advmode(host, RMMC_ADV_DAT_WR);

  for(i = 0; i < blocksz; i++) {
    if (rmmc_send_data_byte(host, data[i], ticks, poll)){
      printk(KERN_ERR "rmmc_send_data:Data Tx Error\n");
      ret = -ETIMEDOUT;
    }
  }

  /* Sending CRC bits*/  
  if ((ret = rmmc_send_crc(host, data, blocksz, ticks, poll))){
    return ret;
  }

  /* Sending stop bit*/  

  rmmc_set_advmode(host, RMMC_ADV_NONE);

  if (rmmc_send_data_byte(host, RMMC_STOP_BIT, 1, 1)){
    ret = -ETIMEDOUT;
  }

#ifdef RMMC_DEBUG
  printk(KERN_DEBUG "rmmc_write_block:stop bit sent\n");
#endif

  return 0;
}

int rmmc_write_data(struct rmmc_host *host, struct mmc_data *data)
{
  int ret = 0;
  int i;
  int ticks, poll;
  unsigned char din;
  unsigned char wr_data[512];

#ifdef RMMC_INFO
  printk(KERN_INFO "rmmc: [BLK WRITE] blocksize = %d, #blocks = %d, noterm = %d\n", data->blksz, data->blocks, !(data->stop));
#endif

  if (!data->sg){
    printk(KERN_ERR "rmmc: error - no sg buffers\n");
    goto done;
  }

  /* Controller setup for transaction */

  rmmc_set_oes(host, RMMC_OE_NONE);

  if ((host->clk == RMMC_CLK_375K) && (data->blksz == 512)) {
    poll = 1;
  } else {
    poll = 0;
  }

  ticks = (host->bus_width == RMMC_DWIDTH_8) ? 1 : 
    ((host->bus_width == RMMC_DWIDTH_4) ? 2 : 8);

  while (1){

    if (host->sg_remain == 0){
      if (host->sg_index == data->sg_len){
        goto done;
      } else {
        host->sg_buf = rmmc_sg_to_buffer(&(data->sg[host->sg_index]));
        host->sg_remain = data->sg[host->sg_index].length;

        if (host->sg_remain == 0 || host->sg_buf == NULL){
          printk(KERN_ERR "rmmc: error - empty sg buffers\n");
          ret = -ETIMEDOUT;
          goto done;
        }
        host->sg_index++;
      }
    }

    if (host->sg_remain  < data->blksz){
      printk(KERN_ERR "rmmc: error - write buffer needs to be multiple of 512 bytes\n");
      BUG();
      ret = -EILSEQ;
      goto done;
    }

    if (!host->sg_buf){
      printk(KERN_ERR "rmmc: error null buffer on write\n");
      BUG();
      ret = -EILSEQ;
      goto done;
    }

    for (i = 0; i < data->blksz; i++){
      wr_data[i] = *host->sg_buf;
      host->sg_buf++;
      host->sg_remain--;
    }

    rmmc_tick_clock(host, 2); 

    rmmc_set_oes(host, RMMC_OE_DAT);
    rmmc_write_block(host, wr_data, data->blksz, ticks, poll);
    rmmc_set_oes(host, RMMC_OE_NONE);
    rmmc_set_advmode(host, RMMC_ADV_NONE);

    /*Read 1 byte:Z+S-CRCStatus-E+ZZ*/
    din = rmmc_get_data_byte(host, 8, 1);

    if(RMMC_CRC_OK(din)){
      data->bytes_xfered += data->blksz;
      if (rmmc_ready_wait(host, RMMC_WR_RDY_POLL_COUNT)){
        if (host->irq){
          /* Tell further down to enable rdy search irq */
          return 1;
        }
        break;
      }
    } else{
      printk(KERN_ERR "rmmc: CRC error on write, resp = %x\n", (din & 0x38) >> 3);
      ret = -EILSEQ;
      break;
    }
  }
done:

#ifdef RMMC_INFO
  printk(KERN_INFO "rmmc: [BLK WRITE] bytes_xfered = %d\n",  data->bytes_xfered);
#endif

  /* send Term CMD*/
  if (data->stop) {
#ifdef RMMC_DEBUG
    printk(KERN_DEBUG "rmmc: sending stop command\n");
#endif
    rmmc_send_command(host, data->stop);
  } else {
    rmmc_tick_clock(host, 4);
  }
  return ret;
}

/**********************************\
 *           Tasklets             *
\**********************************/

irqreturn_t rmmc_irq(int irq, void *dev_id)
{
  struct rmmc_host *host = (struct rmmc_host*)dev_id;

  unsigned char src = in_8((void*)(host->base + RMMC_REG_IRQ_REG)) & in_8((void*)(host->base + RMMC_REG_IRQ_MASK));
  out_8((void*)(host->base + RMMC_REG_IRQ_REG), 0x00);

  /* Schedule tasklets as needed */

  if (src & RMMC_INT_CARD){
    tasklet_schedule(&host->card_tasklet);
  }

  if (src & RMMC_INT_BUSY){
    tasklet_schedule(&host->write_tasklet);
  }

  return IRQ_HANDLED;
}

static void rmmc_card_tasklet(unsigned long param)
{
  struct rmmc_host *host = (struct rmmc_host*)(param);
  spin_lock(&host->lock);

  mdelay(2000);
#ifdef RMMC_DEBUG
  printk(KERN_DEBUG "rmmc: irq\n");
#endif
  host->init_done = 0;
  spin_unlock(&host->lock);
  mmc_detect_change(host->mmc, msecs_to_jiffies(1000));
  return;
}

static void rmmc_read_tasklet(unsigned long param)
{
  struct rmmc_host *host = (struct rmmc_host*)(param);
  struct mmc_command *cmd = host->mrq->cmd;

  spin_lock(&host->lock);
  cmd->error=rmmc_read_data(host, cmd->data);
  spin_unlock(&host->lock);
  mmc_request_done(host->mmc, host->mrq);
  spin_lock(&host->lock);
  host->mrq = NULL;
  spin_unlock(&host->lock);
  return;
}

static void rmmc_write_tasklet(unsigned long param)
{
  struct rmmc_host *host = (struct rmmc_host*)(param);
  struct mmc_command *cmd = host->mrq->cmd;
  int ret;

  spin_lock(&host->lock);
  ret = rmmc_write_data(host, cmd->data);
  if (ret > 0) {
    rmmc_set_find_busy(host);
    spin_unlock(&host->lock);
    return;
  }
  cmd->error = ret;
  spin_unlock(&host->lock);

  mmc_request_done(host->mmc, host->mrq);

  spin_lock(&host->lock);
  host->mrq = NULL;
  spin_unlock(&host->lock);
  return;
}

/**********************************\
 *     MMC Core Routines          *
\**********************************/

static void rmmc_free_mmc(struct device *dev)
{
  struct mmc_host *mmc;
  struct rmmc_host *host;

  mmc = dev_get_drvdata(dev);
  if (!mmc)
    return;

  host = mmc_priv(mmc);
  BUG_ON(host == NULL);

  mmc_free_host(mmc);

  dev_set_drvdata(dev, NULL);
}

/*  Scan for known chip id:s */

static int __devinit rmmc_scan(struct rmmc_host *host)
{
/* Check for host controller presence */

  return 0;
}

static int __devinit rmmc_check(struct rmmc_host *host)
{
  /* Check for host compatibility */
  unsigned char rev_major;
  rev_major = rmmc_get_revmajor(host);
  if (rev_major < 7){
    printk(KERN_ERR "%s: your current ROACH CPLD firmware does not support MMC.\n", "roach-mmc");
    return -ENODEV;
  }
  if (rev_major < 8){
    host->irq = 0;
  } else {
    host->irq = 1;
  }

  return 0;
}

/* Allocate all resources for the host */

static int __devinit rmmc_request_resources(struct rmmc_host *host)
{
  if (!request_mem_region(ROACH_CPLD_BASE, ROACH_CPLD_LENGTH, "roach-cpld-mmc")) {
    printk(KERN_ERR "%s: memory range 0x%08x to 0x%08x is in use\n",
                          "roach-cpld",
                          (resource_size_t) ROACH_CPLD_BASE,
                          (resource_size_t)(ROACH_CPLD_BASE + ROACH_CPLD_LENGTH));
    return ENOMEM;
  }

  host->base = ioremap64(ROACH_CPLD_BASE, ROACH_CPLD_LENGTH);
#ifdef RMMC_DEBUG
  printk(KERN_DEBUG "rmmc-wrtest: ioremap - req = %x, ret = %x\n", ROACH_CPLD_BASE, (unsigned int) host->base);
#endif

  if (!(host->base)) {
    release_mem_region(ROACH_CPLD_BASE, ROACH_CPLD_LENGTH);
    return ENOMEM;
  }

  return 0;
}

/*
 * Release all resources for the host.
 */

static void rmmc_release_resources(struct rmmc_host *host)
{

  if (host->base){
    iounmap(host->base);
  }
  release_mem_region(ROACH_CPLD_BASE, ROACH_CPLD_LENGTH);
}

/*
 * Configure the resources the chip should use.
 */

static void rmmc_chip_config(struct rmmc_host *host)
{
  rmmc_set_clock(host, RMMC_CLK_375K); 
  rmmc_set_data_width(host, RMMC_DWIDTH_1);
  host->init_done = 0;
}

/*****************************************************************************\
 *                                                                           *
 *                   MMC layer callbacks                                     *
 *                                                                           *
\*****************************************************************************/


static inline char *sg_to_buffer(struct scatterlist* sl)
{
  return sg_virt(sl);
}

static void rmmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
  struct rmmc_host *host = mmc_priv(mmc);
  struct mmc_command *cmd;

  /*
   * Disable tasklets to avoid a deadlock.
   */

  spin_lock_bh(&host->lock);

  BUG_ON(host->mrq != NULL);

  cmd = mrq->cmd;

  host->mrq = mrq;

  /* Clear the irqs */
  out_8((void*)(host->base + RMMC_REG_IRQ_REG), CLEAR_IRQS);
  /* Only enable the Card presence activity irq */
  out_8((void*)(host->base + RMMC_REG_IRQ_MASK), RMMC_INT_CARD | (host->irq ? RMMC_INT_BUSY : 0));

  /* check card presence */
  if (!rmmc_get_cd(host)) {
    cmd->error = -ENOMEDIUM;
    goto done;
  }

  /* check card initialization */
  if (!host->init_done) {
    rmmc_init_card(host);
    host->init_done = 1;
  }

  /*
   * Check that there is actually a card in the slot.
   */

  if (!host->dma) {
    cmd->error=rmmc_send_command(host, cmd);
    if (cmd->error)
      goto done;
  } else {
    cmd->error = -ETIMEDOUT;
    goto done;
  }

  if (cmd->data && !cmd->error){
    if (cmd->data->flags & MMC_DATA_READ){
      tasklet_schedule(&host->read_tasklet);
      spin_unlock_bh(&host->lock);
      return;
    } else if (cmd->data->flags & MMC_DATA_WRITE) {
      host->sg_index = 0;
      host->sg_remain = 0;
      cmd->data->bytes_xfered = 0;

      tasklet_schedule(&host->write_tasklet);
      spin_unlock_bh(&host->lock);
      return;
      goto done;
    } else {
      cmd->error = -ETIMEDOUT;
      goto done;
    }
  }

done:
  host->mrq = NULL;
  spin_unlock_bh(&host->lock);

  mmc_request_done(host->mmc, mrq);
}

static void rmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
  struct rmmc_host *host = mmc_priv(mmc);
  u8 clk;
  u8 old_clk;
  u8 old_bw;

  spin_lock_bh(&host->lock);

  old_clk = host->clk;

  if (ios->clock >= 40000000)
    clk = RMMC_CLK_40M;
  else if (ios->clock >= 20000000)
    clk = RMMC_CLK_20M;
  else if (ios->clock >= 10000000)
    clk = RMMC_CLK_10M;
  else
    clk = RMMC_CLK_375K;

  rmmc_set_clock(host, clk); 
  host->clk = clk;

  old_bw = host->bus_width;
  /* NOTE: MMC card might need pin 1 to be set to high here */
  switch (ios->bus_width){
    case MMC_BUS_WIDTH_1:
      rmmc_set_data_width(host, RMMC_DWIDTH_1);
      host->bus_width = RMMC_DWIDTH_1;
      break;
    case MMC_BUS_WIDTH_4:
      rmmc_set_data_width(host, RMMC_DWIDTH_4);
      host->bus_width = RMMC_DWIDTH_4;
      break;
      /* Not Supported by Kernel
    case MMC_BUS_WIDTH_8:
      set_data_width(RMMC_DWIDTH_8);
      host->bus_width = RMMC_DWIDTH_8;
      break;
      */
    default:
      break;
  }
  #ifdef RMMC_INFO
  if (host->clk != old_clk || host->bus_width != old_bw)
    printk(KERN_DEBUG "rmmc: ios status - clk = %d, width = %d\n", host->clk, host->bus_width);
  #endif

  spin_unlock_bh(&host->lock);
}

static int rmmc_is_ro(struct mmc_host *mmc)
{
  return 0;
}

static const struct mmc_host_ops rmmc_ops = {
  .request  = rmmc_request,
  .set_ios  = rmmc_set_ios,
  .get_ro    = rmmc_is_ro
};


/********************** Misc ***********************/

static int __devinit rmmc_alloc_mmc(struct device *dev)
{
  struct mmc_host *mmc;
  struct rmmc_host *host;

  /*
   * Allocate MMC structure.
   */
  mmc = mmc_alloc_host(sizeof(struct rmmc_host), dev);
  if (!mmc)
    return -ENOMEM;

  host = mmc_priv(mmc);
  host->mmc = mmc;

  host->dma = 0;

  /*
   * Set host parameters.
   */
  mmc->ops = &rmmc_ops;
  mmc->f_min = 375000;
  mmc->f_max = 40000000;
  mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
  mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

  spin_lock_init(&host->lock);

  /*
   * Host can only manage 512 efficiently
   */
  mmc->max_blk_size = 512;
  mmc->max_req_size = 65536;
  mmc->max_hw_segs = 128;
  mmc->max_phys_segs = 128;
  mmc->max_blk_count = 128;

  dev_set_drvdata(dev, mmc);

  return 0;
}


/*****************************************************************************\
 *                                                                           *
 * Devices setup and shutdown                                                *
 *                                                                           *
\*****************************************************************************/


static int __devinit rmmc_init(struct device *dev)
{
  struct rmmc_host *host = NULL;
  struct mmc_host *mmc = NULL;
  int ret;

  ret = rmmc_alloc_mmc(dev);
  if (ret)
    return ret;

  mmc = dev_get_drvdata(dev);
  host = mmc_priv(mmc);

  /* Scan for hardware */
  ret = rmmc_scan(host);
  if (ret) { 
    /* no device found, exit */
    rmmc_free_mmc(dev);
    return ret;
  }

  /* Request resources */
  ret = rmmc_request_resources(host);
  if (ret) {
    rmmc_free_mmc(dev);
    return ret;
  }

  /* Check CPLD Revision Number */
  ret = rmmc_check(host);

  if (ret){
    rmmc_release_resources(host);
    rmmc_free_mmc(dev);
    return ret;
  }


  /* Set up tasklets */
  tasklet_init(&(host->card_tasklet), rmmc_card_tasklet, (unsigned long)(host));
  tasklet_init(&(host->read_tasklet), rmmc_read_tasklet, (unsigned long)(host));
  tasklet_init(&(host->write_tasklet), rmmc_write_tasklet, (unsigned long)(host));

  ret = request_irq(PPC_440EPX_IRQ_5, rmmc_irq, 0, DRIVER_NAME, (void*)(host));

  if (ret){
    tasklet_kill(&(host->card_tasklet));
    tasklet_kill(&(host->read_tasklet));
    tasklet_kill(&(host->write_tasklet));
    rmmc_release_resources(host);
    rmmc_free_mmc(dev);
    return ret;
  }

  rmmc_chip_config(host);

  mmc_add_host(mmc);

  printk(KERN_INFO "rmmc: initialized mmc\n");

  return 0;
}


static int __devinit rmmc_probe(struct platform_device *dev)
{
  /* Use the module parameters for resources */
  return rmmc_init(&dev->dev);
}

static int __devexit rmmc_remove(struct platform_device *dev)
{
  struct mmc_host *mmc = dev_get_drvdata(&dev->dev);
  struct rmmc_host *host;

  if (!mmc)
    return 0;

  host = mmc_priv(mmc);

  mmc_remove_host(mmc);

  BUG_ON(host == NULL);

  free_irq(PPC_440EPX_IRQ_5, (void*)(host));

  tasklet_kill(&(host->card_tasklet));
  tasklet_kill(&(host->read_tasklet));
  tasklet_kill(&(host->write_tasklet));

  rmmc_release_resources(host);

  mmc_free_host(mmc);

  dev_set_drvdata(&dev->dev, NULL);

  return 0;
}

static struct platform_device *rmmc_device;

static struct platform_driver rmmc_driver = {
  .probe    = rmmc_probe,
  .remove    = __devexit_p(rmmc_remove),

  .suspend  = NULL,
  .resume    = NULL,
  .driver    = {
    .name  = DRIVER_NAME,
    .owner  = THIS_MODULE,
  },
};

/*
 * Module loading/unloading
 */

static int __init rmmc_drv_init(void)
{
  int result;

  printk(KERN_INFO DRIVER_NAME ": Roach MMC/SD driver\n");

  result = platform_driver_register(&rmmc_driver);
  if (result < 0)
    return result;

  rmmc_device = platform_device_alloc(DRIVER_NAME, -1);
  if (!rmmc_device) {
    platform_driver_unregister(&rmmc_driver);
    return -ENOMEM;
  }

  result = platform_device_add(rmmc_device);
  if (result) {
    platform_device_put(rmmc_device);
    platform_driver_unregister(&rmmc_driver);
    return result;
  }

  return 0;
}

static void __exit rmmc_drv_exit(void)
{
  platform_device_unregister(rmmc_device);
  platform_driver_unregister(&rmmc_driver);
  DBG("unloaded\n");
}

module_init(rmmc_drv_init);
module_exit(rmmc_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Shanly Rajan and David George");
MODULE_DESCRIPTION("ROACH MMC interface");
