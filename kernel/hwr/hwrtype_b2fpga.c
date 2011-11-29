/*********************************************************************
 * $Id: hwrtype_b2fpga.c,v 1.1 2006/10/31 07:28:57 skhay Exp $
 * File  : hwrtype_b2fpga.c
 * Author: Hayden Kwok-Hay So
 * Date  : 1/16/2006
 * Description:
 *   Define hwrtype for BEE2 per FPGA programming
 *********************************************************************/
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/slab.h>    /* kmalloc/kfree            */
#include <linux/init.h>

#include <linux/bof.h>
#include <linux/borph.h>
#define HDEBUG
#define HDBG_NAME "hwrtype_b2fpga"
#ifdef CONFIG_MKD
# define HDBG_LVL mkd_info->dbg_lvl
#else
# define HDBG_LVL 9
#endif
#include <linux/hdebug.h>
#include <linux/kselectmap.h>

/**************************************************************************
 * helper functions
 **************************************************************************/
/*
 * thse are defined in 2.6.x, but not 2.4.x
 */
/*
 * look like we don't need it in 2.6.x
static inline void schedule_timeout_interruptible(int j)
{
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(j);
}
*/
/**************************************************************************
 * selectmap helper
 **************************************************************************/

static inline int selectmap_poll(selectmap_t* dev, uint32_t pin, uint32_t val)
{
	int count = 4;
	uint32_t status;
	do {
		selectmap_in_status(dev, status);
		PDEBUG(9, "POLL: status register = 0x%08x\n", status);
		if ((SELECTMAP_STATUS(status) & pin) == val)
			break;
		schedule_timeout_interruptible(0.01*HZ);
		count--;
	} while (count);
	return (count?0:-1);
}

static inline void selectmap_clear_config(selectmap_t *dev)
{
	u32 status;
	selectmap_in_status(dev, status);
	selectmap_out_status(dev, status & ~(SELMAP_PROG << 24));
	schedule_timeout_interruptible(0.01 * HZ);
	selectmap_out_status(dev, status | (SELMAP_PROG << 24));
}

/* HHH Old code from borph.c */

#ifndef __arch_um__
/* as of 2.4.30, asm-ppc/atomic.h doesn't define this */
#ifndef atmoic_inc_and_test
#define atomic_inc_and_test(v) (atomic_inc_return(v) == 0)
#endif
#endif

/* HAC type */
#if defined(CONFIG_ROACH) || defined(CONFIG_ROACH2)
#define HAC_TYPE HAC_ROACH
#else
#define HAC_TYPE HAC_BEE2FPGA
#endif


/*********************************************************************
 * Hardware Region (HWR) related functions
 *********************************************************************/
static struct phyhwr hwr_b2fpga[SELECTMAP_NUM_DEVS];

/* HHH I still have a feeling this should be part of something 
 * specific to this hwr */

/*
 * reserve_hwr checks if hwr addressed by hwr_addr is free
 * if so, return a pointer to that hwr
 * returns NULL otherwise.
 * 
 * It is like an atomic version of test and get_hwr
 */
struct phyhwr* b2fpga_reserve_hwr(struct hwr_addr* a)
{
	struct phyhwr* ret;
	// HHH because soon I will move hwr_b2fpga stuff into
	// hwrtype_b2fpga.c
	if (a && a->class != HAC_TYPE) return NULL;
	
	/* safety check */
	if (a->addr >= SELECTMAP_NUM_DEVS)
		return NULL;
	ret = &hwr_b2fpga[a->addr];
	if (!atomic_inc_and_test(&ret->count)) {
		atomic_dec(&ret->count);
		/* was being used */
		return NULL;
	}
	// count is now a usage count
	atomic_inc(&ret->count);
	return ret;
}

void b2fpga_release_hwr(struct hwr_addr* a)
{
	struct phyhwr* hwr;
	if (a && a->class != HAC_TYPE)
		return;
	if (a->addr >= SELECTMAP_NUM_DEVS) 
		return;

	hwr = &hwr_b2fpga[a->addr];
	if (atomic_dec_and_test(&hwr->count)) {
		hwr->task = NULL;
		atomic_set(&hwr->count, -1);
	}
}

struct phyhwr* b2fpga_get_hwr(struct hwr_addr* a)
{
	struct phyhwr* ret;
	if (a->addr >= SELECTMAP_NUM_DEVS)
		return NULL;
	ret = &hwr_b2fpga[a->addr];
	atomic_inc(&ret->count);
	return ret;
}

void b2fpga_put_hwr(struct hwr_addr* a)
{
	struct phyhwr* hwr;
	if (a->addr >= SELECTMAP_NUM_DEVS) 
		return;

	hwr = &hwr_b2fpga[a->addr];
	if (atomic_dec_and_test(&hwr->count)) {
		hwr->task = NULL;
		atomic_set(&hwr->count, -1);
	}
}

void __put_hwr(struct phyhwr* hwr)
{
	if (atomic_dec_and_test(&hwr->count)) {
		hwr->task = NULL;
		atomic_set(&hwr->count, -1);
	}
}

/* New code funtions that need to be completed */

static ssize_t b2fpga_send_iobuf (struct hwr_iobuf* iobuf, ssize_t ask)
{
  PDEBUG(9, "B2FPGA send_iobuf");
	return 0; //HHH
}

static ssize_t b2fpga_recv_iobuf (struct hwr_iobuf* iobuf, ssize_t ask)
{
  PDEBUG(9, "B2FPGA recv_iobuf");
	return 0; //HHH
}

static struct hwr_iobuf* b2fpga_get_iobuf(struct borph_ioreg * reg, int size)
{
  PDEBUG(9, "B2FPGA get_iobuf");
  /* If there is different buffer for differ hwr, should
   * deferentiate them here using *reg* */
  return iobuf; //HHH
}

static ssize_t b2fpga_put_iobuf (struct hwr_iobuf* iobuf)
{
  PDEBUG(9, "B2FPGA put_iobuf");
	return 0; //HHH
}

/* HHH end old code */

/*****************************************************************
 * functions definitions
 *****************************************************************/
#ifdef min
#undef min
#endif
#define min(x, y) (x<y?x:y)

/**************************************************************
 * bee2 fpga configure/unconfigure routine
 **************************************************************/

static int configure_b2fpga(struct hwr_addr* addr, struct file* file, uint32_t offset, uint32_t len)
{
  selectmap_t *dev;
  int retval = -EIO;
  u32 status;
  int count, i;

  PDEBUG(9, "configuring bee2 fpga %u from (offset %u, len %u) of %s\n",
      addr->addr, offset, len, file->f_dentry->d_name.name);
  if (addr->addr >= SELECTMAP_NUM_DEVS) {
    return -EIO;
  }
  if (addr->addr == 4) {
    // HACK ALERT
    printk("not configuring control FPGA in this hacky version\n");
    if ((dev = get_selmapdev(addr->addr)) == NULL) {
      return -EIO;
    }
    retval = 0;
    goto done_prog;
  }

  if ((dev = get_selmapdev(addr->addr)) == NULL) {
    return -EIO;
	}

	retval = -EIO;
	PDEBUG(9, "set selectmap to config mode\n");
	selectmap_set_cfgmode(dev);

	// to configure, first need to cycle assert/deassert PROG_N
	PDEBUG(9, "pulse prog_n\n");
	selectmap_clear_config(dev);

	// poll till INIT pin is asserted from FPGA
	PDEBUG(9, "poll init_n\n");
	if (selectmap_poll(dev, SELMAP_INIT, SELMAP_INIT) < 0) {
		goto out;
	}
	PDEBUG(9, "done polling init_n\n");

	// now, stream data to selectmap
	while (len > 0) {
		buf_t *bufp;
#if defined(__arch_um__)
		break;
#endif
		count = min(SELECTMAP_BUFSIZE, len);
		retval = kernel_read(file, offset, dev->buf, count);
		bufp = dev->buf;
		if (retval < 0) {
			goto out;
		}
		if (retval != count) {
			printk("kernel_read returns less than requested...\n");
			count = retval;
		}

		len -= count;
		offset += count;

		/* we have count bytes in dev->buf to write */
		i = 0; retval = -EIO;
		selectmap_in_status(dev, status);
		PDEBUG(10, "count = %d, buf[0 1 2 3]=%02x %02x %02x %02x\n", 
		       count, dev->buf[0], dev->buf[1], dev->buf[2], 
		       dev->buf[3]);
		while(count > 0) {
			selectmap_out_data(dev, dev->buf[i]);
			i++;
			count -= 1;
		}
	}

	// poll till DONE pin is asserted from FPGA
	PDEBUG(9, "poll done\n");
	if (selectmap_poll(dev, SELMAP_DONE, SELMAP_DONE) < 0) {
	    PDEBUG(9, "polling done returns error\n");
		goto out;
	}
	PDEBUG(9, "finished polling done\n");
	retval = 0;
 done_prog:
	/* synchronizing read */
	/*
	PDEBUG(9, "Synchronizing read. It should work with the new selectmap\n");
	selectmap_set_fifomode(dev);
	selectmap_in_data(dev, status);
	*/
	/* flush fifo, just to be safe
	 * Note that the current version of opb_selectmap v1.01a needs a
	 * synchronizing read too */
	PDEBUG(9, "flush fifo + synch readnow\n");
	selectmap_set_fifomode(dev);
	/* synchronization read */
	selectmap_in_data(dev, status);
	/* flushing loop */
  for (i = 0; i < 10; i++) {
    selectmap_in_status(dev, status);
    PDEBUG(9, "status register = 0x%08x\n", status);
    count = SELECTMAP_RFIFO_CNT(status);
    if (!count) break;
    PDEBUG(9, "has %d words in fifo, flush them\n", count);
    while (count--) {
      selectmap_in_data(dev, status);
    }
  }
  if (i == 10) {
	    PDEBUG(9, "I have flushed the fifo 10 times yet it still has stuff in it.. what?\n");
	}
 out:
	put_selmapdev(dev);
	return retval;
}

static int unconfigure_b2fpga(struct hwr_addr* addr)
{
	selectmap_t *dev;

	printk("unconfigure bee2 fpga %u\n", addr->addr);
	if (addr->addr >= SELECTMAP_NUM_DEVS) {
		return -EIO;
	}

	if ((dev = get_selmapdev(addr->addr)) == NULL) {
		return -EIO;
	}
	/* Send Greet message */
	selectmap_out_data(dev, PP_CMD_BYE);
	selectmap_out_data(dev, 0x55);
	selectmap_out_data(dev, 0xAA);
	selectmap_out_data(dev, 0xFF);

	// HACK ALERT
	if (addr->addr != 4) {
	    selectmap_clear_config(dev);
	}

	put_selmapdev(dev);

	return 0;
}

static struct hwr_operations b2fpga_hwr_operations = {
	configure: configure_b2fpga,
	unconfigure: unconfigure_b2fpga,
  reserve_hwr: b2fpga_reserve_hwr,
  release_hwr: b2fpga_release_hwr,
  get_iobuf: b2fpga_get_iobuf,
  put_iobuf: b2fpga_put_iobuf,
  send_iobuf: b2fpga_send_iobuf
};

static struct hwrtype hwrtype_b2fpga = {
	name: "b2fpga",
	type: HAC_B2FPGA,
	count: ATOMIC_INIT(0),
  num_devs: SELECTMAP_NUM_DEVS,
	hwr_ops: &b2fpga_hwr_operations,
};

static int __init hwrtype_b2fpga_init(void)
{
	int retval = 0;

	if ((retval = register_hwrtype(&hwrtype_b2fpga)) < 0) {
		printk("Error registering hwrtype\n");
		goto out;
	} else {
		printk("hwrtype_b2fpga version CVS-$Revision: 1.1 $ registered\n");
	}
/** HHH untested below **/
	/* initialize hwr array */
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		atomic_set(&(hwr_b2fpga[i]).count, -1);
	}

	if (kselectmap_init()) return;
/** HHH untested above **/

 out:
	return retval;
}

static void __exit hwrtype_b2fpga_exit(void)
{
	if (unregister_hwrtype(&hwrtype_b2fpga)) {
		printk("Error unregistering hwrtyp\n");
	} else {
		printk("hwrtype_b2fpga CVS-$Revision: 1.1 $ unregistered\n");
	}
}

module_init(hwrtype_b2fpga_init);
module_exit(hwrtype_b2fpga_exit);

MODULE_AUTHOR("Hayden So");
MODULE_DESCRIPTION("Add hwrtype b2fpga to program FPGA on B2 as hw process");
MODULE_LICENSE("GPL");
