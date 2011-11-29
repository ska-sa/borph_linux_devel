/******************************************************************
 * SelectMap driver
 *
 * A lot of the code in thie file are borrowed from the selectmap device 
 * driver by Andrew Shultz.  Since the OS is using this selectmap link 
 * exclusively, there is no need for a device driver with BORPH.  But since
 * the communication mechanism with the physical hw is essentail the 
 * same, the code from device driver can be reuse here to a large extent
 *
 *
 * There are two registers, both 32-bits.  The register at base+0 is
 * the control register which has the following layout: 
 *
 *  0-3   4      5       6       7        8-15         16-23    24-31
 * +------------------------------------------------------------------+
 * |   | MODE | PROG_B | INIT_B | DONE | RdFifo Cnt | WrFifo Cnt |    |
 * +------------------------------------------------------------------+
 *
 * MODE:
 * 0 = "Configure" mode, has appropriate timing for config/readback
 * 1 = "FIFO" mode, has timing for general purpose two-way fifo
 *
 * PROG_B:
 * 0 = PROG pin asserted
 * 1 = PROG pin deasserted
 *
 * INIT_B:
 * 0 = Init asserted (after prog pulse in config mode)/Interrupt
 *     deasserted (fifo mode)
 * 1 = Init deasserted (config mode)/Interrupt asserted (fifo mode)
 *
 * DONE:
 * 0 = Chip unconfigured
 * 1 = Chip configured
 *
 * RdFifo Cnt:
 * # elements in read fifo
 *
 * WrFifo Cnt:
 * # elements in write fifo (space left is 256 - WrFifo Cnt)
 *
 * The other register is base+4 and that is the data register which you
 * read/write for configuration and for FIFO access.  
 */
#ifndef _KSELECTMAP_H_
#define _KSELECTMAP_H_
#undef SELECTMAP_WORD_XFER

#include <asm/io.h>        /* in/out functions         */
#include <asm/types.h>
#include <asm/semaphore.h>

#define SELECTMAP_NUM_DEVS       5
#define SELECTMAP_LENGTH         8
#define SELECTMAP_WIDTH          1
#define SELECTMAP_FIFO_NUM_WORDS 600
#define SELECTMAP_FIFO_NUM_BYTES 127
#define SELECTMAP_FIFO_MAX       255
#define SELECTMAP_BUFSIZE        (SELECTMAP_WIDTH * SELECTMAP_FIFO_NUM_WORDS)

/*****************************************************************
 * Data macros
 *****************************************************************/

#define SELMAP_MODE            0x08
#define SELMAP_PROG            0x04
#define SELMAP_INIT            0x02
#define SELMAP_DONE            0x01

#define SELECTMAP_STATUS(word)    ((u8)((word >> 24) & 0xff))
#define SELECTMAP_MODE(word)      (u8)(SELECTMAP_STATUS(word) & MODE)
#define SELECTMAP_PROG(word)      (u8)(SELECTMAP_STATUS(word) & PROG)
#define SELECTMAP_INIT(word)      (u8)(SELECTMAP_STATUS(word) & INIT)
#define SELECTMAP_DONE(word)      (u8)(SELECTMAP_STATUS(word) & DONE)
#define SELECTMAP_RFIFO_CNT(word) (u8)(((word) >> 16) & 0xff)
#define SELECTMAP_WFIFO_CNT(word) (u8)(((word) >> 8)  & 0xff)

/*****************************************************************
 * Data structures
 *****************************************************************/
typedef unsigned char buf_t;
typedef struct selectmap {
    unsigned long base;    /* Base address of memory region   */
    void *vbase;           /* Virtual base address of region  */
    buf_t *buf;    /* Read/write buffer               */
    int id;                /* Identifier                      */
    struct semaphore sem;
//    atomic_t available;    /* Single open flag                */
} selectmap_t;

/*****************************************************************************
 * debugging fix up
 ********************/
/* if we are testing using uml, there's no real (ppc+xilinx specific) i/o */

#if defined(__arch_um__)
static inline void um_out_be32(volatile unsigned *addr, int val)
{
	printk("out_be32(%p, 0x%x)\n", addr, val);
}

static inline void um_out_8(volatile unsigned *addr, uint8_t val)
{
	printk("out_8(%p, 0x%x)\n", addr, val);
}

static inline unsigned um_in_be32(volatile unsigned *addr)
{
	unsigned retval;
	static unsigned fakeval[4] = { 
	    0x0F027f00, 0x0F027f00, 0x0F027f00 , 0x0F027f00,
//		0x00000000, 0x04007f00, 0x02000200, 0x0d2e7f00
	};
	static int idx = 0;
	retval = fakeval[idx];
	idx += 1;
	if (idx == 4) {
		idx = 0;
	}
	printk("um_in_be32 returns 0x%p, 0x%x\n", addr, retval);
	return retval;
}

static inline unsigned char um_in_8(volatile unsigned *addr)
{
	unsigned retval;
#define FAKEVALSIZE 46
	static unsigned char fakeval[FAKEVALSIZE] = { 
	    2, 0, 0x42, 0, 0, 0, 0, 4, 0xFF, 0xAB, 0x12, 0x34, /*12*/
	    2, 0, 0x42, 0, 0, 0, 0, 1, 0x7,                    /*9*/
	    3, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0x64,         /*13*/
	    2, 0, 0x42, 0, 0, 0, 0, 4, 0xDE, 0xAD, 0xBE, 0xEF, /*12*/
	};
	static int idx = 0;
	retval = fakeval[idx];
	idx += 1;
	if (idx == FAKEVALSIZE) {
		idx = 1;
	}
	printk("um_in_8 returns 0x%x\n", retval);
	return retval;
}

#define out_be32(a, v) um_out_be32(a, v)
//#define in_be32(a) (0x0F027f00)
#define in_be32(a) um_in_be32(a)
#define out_8(a, v) um_out_8(a, v)
//#define in_8(a) (0xF4)
#define in_8(a) um_in_8(a)
#define iounmap(a)
#define ioremap(a, len) NULL
#define XPAR_OPB_SELECTMAP_0_BASEADDR 0
#define XPAR_OPB_INTC_0_OPB_SELECTMAP_0_IP2INTC_IRPT_INTR 0
#elif defined(CONFIG_ROACH) || defined(CONFIG_ROACH2)
#define XPAR_OPB_SELECTMAP_0_BASEADDR 0
#define XPAR_OPB_INTC_0_OPB_SELECTMAP_0_IP2INTC_IRPT_INTR 0
#else
#include "xparameters.h"
#endif

/*****************************************************************************/
#ifdef SELECTMAP_WORD_XFER
#define selectmap_out_data(dev, val) \
  do {out_be32((dev)->vbase+4, val); wmb();} while (0)
#define selectmap_in_data(dev, val) \
  do {val = in_be32((dev)->vbase+4); rmb();} while (0)
#define selectmap_out_status(dev, val) \
  do {out_be32((dev)->vbase, val); wmb();} while (0)
#define selectmap_in_status(dev, val) \
  do {val = in_be32(dev->vbase); rmb();} while (0)

#define selectmap_set_fifomode(dev) \
  do {u32 s = in_be32((dev)->vbase); rmb(); out_be32((dev)->vbase, s|0x08000000); wmb();} while (0)
#define selectmap_set_cfgmode(dev) \
  do {u32 s = in_be32((dev)->vbase); rmb(); out_be32((dev)->vbase, s&~0x08000000); wmb();} while (0)
#else  /* byte transfer */
#define selectmap_out_data(dev, bval) \
  do {out_8((dev)->vbase+4, bval); wmb();} while (0)
#define selectmap_in_data(dev, val) \
  do {val = in_8((dev)->vbase+4); rmb();} while (0)

#define selectmap_out_status(dev, val) \
  do {out_be32((dev)->vbase, val); wmb();} while (0)
#define selectmap_in_status(dev, val) \
  do {val = in_be32(dev->vbase); rmb();} while (0)

#define selectmap_set_fifomode(dev) \
  do {u32 s = in_be32((dev)->vbase); rmb(); out_be32((dev)->vbase, s|0x08000000); wmb();} while (0)
#define selectmap_set_cfgmode(dev) \
  do {u32 s = in_be32((dev)->vbase); rmb(); out_be32((dev)->vbase, s&~0x08000000); wmb();} while (0)

#endif /* SELECTMAP_WORD_XFER */

/*************************************************************************
 * interrupt stuff 
 *************************************************************************/
#define SELECTMAP_IRQ (31 - XPAR_OPB_INTC_0_OPB_SELECTMAP_0_IP2INTC_IRPT_INTR)
#define SELMAP_INTR_DISR mkd_info->intrreg
#define SELMAP_INTR_DIPR (mkd_info->intrreg + 0x4)
#define SELMAP_INTR_DIER (mkd_info->intrreg + 0x8)
#define SELMAP_INTR_DIIR (mkd_info->intrreg + 0x18)
#define SELMAP_INTR_DGIER (mkd_info->intrreg + 0x1c)
#define SELMAP_INTR_IPISR (mkd_info->intrreg + 0x20)
#define SELMAP_INTR_IPIER (mkd_info->intrreg + 0x28)

#define out_intr_reg(reg, val) \
  do {out_be32((reg), val); wmb();} while (0)
#define in_intr_reg(reg, val) \
  do {(val) = in_be32(reg); rmb();} while (0)


/* helper functions defined in kselectmap.c */
extern selectmap_t* get_selmapdev(int addr);
extern void put_selmapdev(selectmap_t* dev);
extern int kselectmap_init(void);
#endif
