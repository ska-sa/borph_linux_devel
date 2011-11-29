/* A set of dodgy routines to perform I2C transactions
 * to get 'late' i2c access for ROACH Monitor
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>


#define EPX_IIC0_BASE 0x1EF600700

static void* roach_late_iic_base = NULL;

#define IIC0_MDBUF 0x0
#define IIC0_LMADDR 0x4
#define IIC0_HMADDR 0x5
#define IIC0_CNTRL 0x6
#define IIC0_MDCTRL 0x7
#define IIC0_STS 0x8
#define IIC0_INTRMSK 0xd
#define IIC0_EXSTS 0x9

void setreg(int addr, unsigned char val)
{
  *((volatile unsigned char *)(roach_late_iic_base + addr)) = val;
}

unsigned char getreg(int addr)
{
  return *((volatile unsigned char *)(roach_late_iic_base + addr));
}

int roach_late_iic_init(void)
{
  if (!(roach_late_iic_base = ioremap64(EPX_IIC0_BASE, 0x100))){
    return -1;
  }

  setreg(IIC0_INTRMSK, 0);
  setreg(IIC0_MDCTRL, 0);
  return 0;
}

#define IIC_START 0x1
#define IIC_RD    0x2
#define IIC_WR    0x0
#define IIC_DONE  0x1
int roach_late_iic_xfer(int iic_addr, int rd_w_n, unsigned char* data, int size)
{
  int xfer_size = size - 1; /* controller starts at 1 */
  int i;
  int ret=0;

  if (xfer_size > 3)
    xfer_size = 3;
  if (xfer_size <= 0)
    xfer_size = 0;

  for (i=0; i < 4; i++){
    getreg(IIC0_MDBUF);
  }

  setreg(IIC0_STS, 0);
  setreg(IIC0_EXSTS, 0xf);

  if (!rd_w_n) {
    for (i=0; i <= xfer_size; i++){
      setreg(IIC0_MDBUF, data[i]);
    }
  }

  setreg(IIC0_HMADDR, 0);
  setreg(IIC0_LMADDR, (iic_addr & 0x7f) << 1);

  setreg(IIC0_CNTRL, ((xfer_size & 0x3) << 4) | IIC_START | (rd_w_n ? IIC_RD : IIC_WR));

  for (i=0; i < 1000000; i++){
    if (!(getreg(IIC0_STS) & IIC_DONE))
      break;
  }

  if (i == 1000000){
    ret = -1;
  }

  if (rd_w_n){
    for (i=0; i <= xfer_size; i++){
      data[i] = getreg(IIC0_MDBUF);
    }
  }

  return ret;
}
