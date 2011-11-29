/*
 */
#define ROACH_CPLD_BASE    0x1C0001000
#define ROACH_CPLD_LENGTH  0x000001000

/* Interrupt Definitions */
#define PPC_440EPX_IRQ_5    64
#define RMMC_INT_CARD      0x1
#define RMMC_INT_BUSY      0x2
#define CLEAR_IRQS        0x00
#define CARD_IRQ_EN       0x01


#define RMMC_RESPONSE_WAIT_MAX 128
#define RMMC_DATA_WAIT_MAX     1000000

/* Register Offsets */
#define RMMC_REG_STAT     0x4
#define RMMC_REG_CMD      0x10
#define RMMC_REG_DAT      0x11
#define RMMC_REG_ADV      0x12
#define RMMC_REG_MAN      0x13
#define RMMC_REG_CLK      0x14
#define RMMC_REG_CRC_CMD  0x15
#define RMMC_REG_CRC_DAT1 0x16
#define RMMC_REG_IRQ_MASK 0x1e
#define RMMC_REG_IRQ_REG  0x1f
#define RMMC_REG_REV_MAJ  0x1a

/* Register Bit Defines */
#define RMMC_ADV_DONE (1 << 3)
#define RMMC_MAN_ADV  (1 << 3)
#define RMMC_REG_CD   (1 << 4)
#define RMMC_REG_RO   (1 << 5)

#define RMMC_FIND_BUSY 0x40

#define RMMC_INS_ADV(x,y)  (((x) & 0xcf) | (((y) & 0x3) << 4))
#define RMMC_INS_CLK(x,y)  (((x) & 0xfc) | (((y) & 0x3) << 0))
#define RMMC_INS_DW(x,y)   (((x) & 0xf3) | (((y) & 0x3) << 2))
#define RMMC_INS_OE(x,y)   (((x) & 0xcf) | (((y) & 0x3) << 4)) 

#define RMMC_GET_ADV(x) (((x) & 0x30) >> 4)

#define RMMC_OE_DAT  (1<<1)
#define RMMC_OE_CMD  (1<<0)
#define RMMC_OE_NONE 0x0

/* Clock Advance Defines */
#define RMMC_ADV_NONE   0x0
#define RMMC_ADV_DAT_RD 0x1
#define RMMC_ADV_DAT_WR 0x2

/* Post write checks */
#define RMMC_CRC_OK(x) ( (((x) & 0x38) >> 3) == (0x2))
#define RMMC_WR_RDY(x) ((x) & 0x1)
#define RMMC_WR_RDY_POLL_COUNT 1000

/* MMC DAT/CMD Data Bit Defines */
#define RMMC_START_BIT 0x0
#define RMMC_TRANS_BIT (1 << 6)
#define RMMC_STOP_BIT  (0x1)

#define RMMC_CLK_40M   0x0
#define RMMC_CLK_20M   0x1
#define RMMC_CLK_10M   0x2
#define RMMC_CLK_375K  0x3

#define RMMC_DWIDTH_1  0x0
#define RMMC_DWIDTH_4  0x1
#define RMMC_DWIDTH_8  0x2

struct rmmc_host
{
  struct mmc_host*  mmc;    /* MMC structure */

  spinlock_t    lock;    /* Mutex */

  struct mmc_request*  mrq;    /* Current request */

  unsigned char   partial_buf [128];  /* Partial data read buffer */
  unsigned int    partial_buf_len;  /* Partial data read buffer */

  char*      dma_buffer;  /* ISA DMA buffer */
  dma_addr_t    dma_addr;  /* Physical address for same */

  u8      clk;    /* Current clock speed */
  unsigned char    bus_width;  /* Current bus width */

  unsigned char    init_done;  /* Card initialization is done */

  void   *base;    /* I/O port base */
  int      dma;    /* DMA channel */
  int      irq;    /* Are IRQs fully supported */

  struct tasklet_struct  card_tasklet;  /* Tasklet structures */
  struct tasklet_struct  read_tasklet;  /* Tasklet structures */
  struct tasklet_struct  write_tasklet;  /* Tasklet structures */

  int sg_index;
  int sg_remain;
  unsigned char* sg_buf;

  struct timer_list  ignore_timer;  /* Ignore detection timer */
};
