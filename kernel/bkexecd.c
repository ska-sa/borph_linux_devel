/*********************************************************************
 * $Id: bkexecd.c,v 1.1 2006/10/31 07:28:57 skhay Exp $
 * File  : bkexecd.c
 * Author: Hayden Kwok-Hay So
 * Date  : 12/09/2005
 * Description:
 *   This is the main code for bkexecd. It is responsible for
 * configuring and de-configure FPGA as part of a BORPH process
 *
 *   On a BEE2, bkexecd is responsible for all communication to "user
 * fpga" via SelectMap interface.  A FPGA is configured during exec of
 * a bof file.  Therefore. bkexecd simply sleep upon start up and wait
 * forever until a bof file is exec-ed and binfmt_bof wakes it up.
 *********************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/borph.h>
#include <linux/bof.h>
#include <linux/init.h>
#include <linux/highmem.h>
#include <linux/kselectmap.h>
#include <linux/binfmts.h>
#include <linux/gfp.h>

#if defined(CONFIG_ROACH) || defined(CONFIG_ROACH2)
#define HAC_TYPE HAC_ROACH
#else
#define HAC_TYPE HAC_BEE2FPGA
#endif

#define HDEBUG
#define HDBG_NAME "bkexecd"
#ifdef CONFIG_MKD
# define HDBG_LVL mkd_info->dbg_lvl
#else
# define HDBG_LVL 9
#endif
#include <linux/hdebug.h>

// HHH move me to mkd.c
int mkdbuf_reset(struct hwr_addr* a) {
	if (!a || a->class != HAC_TYPE) return -ENOEXEC;
	if (a->addr >= SELECTMAP_NUM_DEVS ) return -ENOEXEC;
	mkd_info->mkd_bufs[a->addr] = mkd_info->mkd_bufe[a->addr];
	return 0;
}

struct bkexecd_info bked_info;
static pid_t bked_pid;

static size_t get_args(char* buf, struct linux_binprm* bprm, size_t size) {
	char* bptr = buf;
	char* kaddr;
	size_t ret = 0;
	unsigned long offset, len, p;
	struct page* page;
	int argc;

	if (!buf || !bprm || !size) return -EINVAL;
	
	PDEBUG(9, "bprm: p=%lu, argc=%d, envc=%d\n", bprm->p, bprm->argc, bprm->envc);
	argc = bprm->argc;
	p = bprm->p;
	while (argc && (ret < size)) {
		offset = p % PAGE_SIZE;
		len = PAGE_SIZE - offset;

#ifdef CONFIG_MMU
		get_user_pages(current, bprm->mm, p, 1, 0, 1, &page, NULL);
#else
		page = bprm->page[p / PAGE_SIZE];
#endif
		kaddr = kmap(page) + offset;
		
		if (ret + len > size) {
			len = size - ret;
		}

		while (len-- && argc) {
			char c;
			*bptr++ = c = *kaddr++;
			if (!c) {
				argc -= 1;
			}
			p += 1;
			ret += 1;
		}
		kunmap(page);
	}

	for (argc = 0, bptr = buf; argc < ret; argc++) {
		PDEBUG(9, "%.2x ", *bptr++);
	}
	printk("\n");
	return ret;
}

static inline int 
__send_greet(struct hwr_addr* addr, struct linux_binprm* bprm, int sendarg)
{
	struct hwr_iobuf *output_buffer;         
  int ask, retval = 0;

	/* Get a buffer from the device */
        output_buffer = get_iobuf(addr);
	if (!output_buffer) {
		return retval;
        }
	
	if ((retval = get_args(output_buffer->data, bprm, PAGE_SIZE - 12)) < 0) {
		goto out;
	}

	output_buffer->cmd = PP_CMD_GREET;
	output_buffer->location = 0;
	output_buffer->offset = 0;
	if (sendarg) {
    ask = retval;
		retval += 6;
	} else {
		retval = 4;
    ask = 0;
	}
	
	/* Send the data to the device */
  send_iobuf(addr, output_buffer, ask);
out:
	/* Return the buffer */
        put_iobuf(addr, output_buffer);
        return retval;	
}

static int send_greet(struct hwr_addr* addr, struct linux_binprm* bprm)
{
	return __send_greet(addr, bprm, 1);
}

static int send_oldgreet(struct hwr_addr* addr, struct linux_binprm* bprm)
{
	return __send_greet(addr, bprm, 0);
}


static inline struct phyhwr* bpr_nextfree(struct hwr_addr* a)
{
	int i;
	struct phyhwr* retval;

	// we only know about b2fpga now
	if (!a || a->class != HAC_TYPE) return NULL;
	/* Make sure it never checks control FPGA */

	for (i = 0; i < 4; i++) {
		a->addr = i;
		retval = reserve_hwr(a);
		if (retval) {
			return retval;
		}
	}
	return NULL;
}

static inline int borph_load_hw(struct execq_item *execq_item)
{
	struct borph_info* bi;
	struct borph_hw_region* region;
	struct bofhdr bofhdr;
	struct hwr_operations* hwrops;
	uint32_t cur_foff;
	int retval;
	struct hwrhdr hwrhdr;

	PDEBUG(9, "configure from file: %s\n", 
	       execq_item->bprm->filename);

	// SHOULD USE SLAB!!!
	bi = kmalloc(sizeof(struct borph_info), GFP_KERNEL);
	memset(bi, 0, sizeof(struct borph_info));
	INIT_LIST_HEAD(&bi->hw_region);
	INIT_LIST_HEAD(&bi->ioreg);
	INIT_LIST_HEAD(&bi->hcptr);
	rwlock_init(&bi->hcptr_lock);
	bi->ioreg_mode = 1;  // default change to 1 on 2006/08/19 HS 
	bi->status = 0;  // this is redundant, but make sure status is cleared

	execq_item->task->borph_info = bi;

	retval = -EIO;
        bofhdr = *((struct bofhdr *) execq_item->bprm->buf);

	PDEBUG(9, "version=%d, numchip=%d, elf_off=0x%x, hw_off=0x%x, ekver=0x%x\n",
	       bofhdr.b_version, bofhdr.b_numchip, bofhdr.b_elfoff, 
	       bofhdr.b_hwoff, bofhdr.b_ekver);
	cur_foff = bofhdr.b_hwoff;
	while (bofhdr.b_numchip--) {
		struct phyhwr* hwr;
		char* strtab = NULL;
		uint32_t strtab_len = 0;
		uint32_t strtab_foff;

		if (cur_foff == 0) {
			PDEBUG(9, "invalid bof file\n");
			goto out_delregion;
		}
		retval = kernel_read(execq_item->bprm->file, cur_foff, 
				     (char*) &hwrhdr, sizeof(struct hwrhdr));
		if (retval < 0) {
			goto out_delregion;
		}
		cur_foff += sizeof(struct hwrhdr);

		PDEBUG(9, "hwrhdr: flag=0x%x, pl_off=0x%x, pl_len=0x%x," 
		       " next_hwr=0x%x, strtab_off=0x%x, nr_symbol=%d\n", 
		       hwrhdr.flag, hwrhdr.pl_off, hwrhdr.pl_len, 
		       hwrhdr.next_hwr, hwrhdr.strtab_off, hwrhdr.nr_symbol);

		retval = -EBUSY;
		if (hwrhdr.flag & HFG_PLACED) {
			hwr = reserve_hwr(&hwrhdr.addr);

			if (!hwr ){
				goto out_delregion;
			}
		} else {
			// need P&R
			hwr = bpr_nextfree(&hwrhdr.addr);
			if (!hwr) {
				goto out_delregion;
			}
		}
		hwr->task = execq_item->task;
		
		PDEBUG(5, "configuring fpga %d\n", hwrhdr.addr.addr);
		/* strtab */
		retval = -ENOEXEC;
		strtab_foff = 
			cur_foff + hwrhdr.nr_symbol * sizeof(struct bofioreg);
		strtab_len = hwrhdr.pl_off - hwrhdr.strtab_off;
		if (!strtab_len && hwrhdr.nr_symbol) {
			PDEBUG(0, "0 length string table\n");
			goto out_delregion;
		}
		strtab = kmalloc(strtab_len, GFP_KERNEL);
		if (!strtab) {
		    PDEBUG(9, "kmalloc strtab failed\n");
		    goto out_delregion;
		}
		retval = kernel_read(execq_item->bprm->file, strtab_foff,
				     (char*) strtab, strtab_len);
		if (retval < 0) goto out_delregion;

		/* region */
		// should use slab
		region = kmalloc(sizeof(struct borph_hw_region), GFP_KERNEL);
		if (!region) {
			PDEBUG(9, "kmalloc region failed\n");
		} else {
			PDEBUG(9, "got region\n");
		}
		region->addr = hwrhdr.addr;
		region->strtab = strtab;
		list_add(&(region->list), &(bi->hw_region));
	
		PDEBUG(9, "getting hwrops from class %d\n", hwrhdr.addr.class);
		hwrops = get_hwrops(&hwrhdr.addr);
		if (hwrops && hwrops->configure) {
			PDEBUG(9, "Before we configure, let's clear our bookkeeping first\n");
#ifdef CONFIG_MKD
			retval = mkdbuf_reset(&(hwrhdr.addr));

			if (retval) goto out_delregion;
#endif
			/* HHH: What I really want, is to have a different
			 * hwr class for control FPGA... but we're in a rush */
//			if (hwrhdr.flag & HFG_NOCONFIG) {
//				printk("Skipping configuration for control fpga\n");
//				retval = 0;
//			} else {
				retval = hwrops->configure(&(hwrhdr.addr), 
							   execq_item->bprm->file, 
							   hwrhdr.pl_off,
							   hwrhdr.pl_len);
//			}
		} else {
			printk(KERN_INFO "no hwrops from hwr class %d\n",
			       hwrhdr.addr.class);
			retval = -ENOEXEC;
			goto out_delregion;
		}
		put_hwrops(&hwrhdr.addr);

		/* ioreg */
		while (hwrhdr.nr_symbol--) {
			struct borph_ioreg *reg;
			struct bofioreg bof_ioreg;

			// should use slab
			reg = kmalloc(sizeof(struct borph_ioreg), GFP_KERNEL);
			if (!reg) {
				PDEBUG(9, "kmalloc borph_ioreg failed\n");
			}
			retval = kernel_read(execq_item->bprm->file, cur_foff,
					     (char*) &bof_ioreg, 
					     sizeof(struct bofioreg));
			if (retval < 0) {
				goto out_delregion;
			}
			cur_foff += sizeof(struct bofioreg);
		    
			reg->name = strtab + bof_ioreg.name;
			reg->mode = bof_ioreg.mode;
			reg->loc = bof_ioreg.loc;
			reg->len = bof_ioreg.len;
			reg->hwraddr = &region->addr;
			PDEBUG(5, "got ioreg %s, mode=%x, loc=%x, len=%x\n", 
			       reg->name, reg->mode, reg->loc, reg->len);
			list_add(&(reg->list), &(bi->ioreg));
		}
		if (retval < 0) {
			goto out_delregion;
		}
#if !defined(CONFIG_ROACH) || !defined(CONFIG_ROACH2)
    /* we dont want to write random stuff into the ROACH registers */
    PDEBUG(9, "sending greet\n");
    if (bofhdr.b_ekver == 0x0000e001) {
      retval = send_greet(&(hwrhdr.addr), execq_item->bprm);
    } else {
      retval = send_oldgreet(&(hwrhdr.addr), execq_item->bprm);
		}
		if (retval < 0)
			goto out_delregion;
#endif

		/* done with all the configuration for this chip
		 * tell the rest of the kernel about it */
		hwr_activate(&(hwrhdr.addr));
		/* enable interrupt for this hwr */
#ifdef CONFIG_MKD
		{
		    uint32_t d, mask;
		    mask = 1UL << hwrhdr.addr.addr;
		    in_intr_reg(SELMAP_INTR_IPISR, d);
		    if (d & mask) {
			/* clear old interrupt, if any */
			out_intr_reg(SELMAP_INTR_IPISR, mask);
		    }
		    in_intr_reg(SELMAP_INTR_IPIER, d);
		    out_intr_reg(SELMAP_INTR_IPIER, d | mask);
		}
#endif		
		/* prepare for next hardware region */
		cur_foff = hwrhdr.next_hwr;
	}
	//done
	bi->status |= BORPH_STATUS_DEVICE_READY;
	PDEBUG(9, "chip load done\n");
	return 0;
 out_delregion:
	borph_exit_fpga(execq_item->task);
	return retval;
}

/* program FPGA according to execq_item until no more item on
   execq_list */
static inline void run_execq(void)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&bked_info.execq_lock, flags);
	while (!list_empty(&bked_info.execq_list)) {
		struct execq_item* execq_item;
		struct bofhdr* bhdr;
		PDEBUG(9, "files list not empty, do something... \n");
		execq_item = list_entry(bked_info.execq_list.next, struct execq_item, list);
		list_del_init(bked_info.execq_list.next);
		spin_unlock_irqrestore(&bked_info.execq_lock, flags);

		/***** actual fpga configuration *****/
		retval = borph_load_hw(execq_item);
		bhdr = (struct bofhdr*) (execq_item->bprm->buf);
		bhdr->load_err = retval;

		wake_up_process(execq_item->task);
		/***** end fpga configuration *****/
		kfree(execq_item);  // HHH return to slab

		// get lock in preparation for list_empty check
		spin_lock_irqsave(&bked_info.execq_lock, flags);
		wake_up_interruptible(&bked_info.exec_done);
	}
	spin_unlock_irqrestore(&bked_info.execq_lock, flags);
}

static int bkexecd(void *dummy)
{
	DECLARE_WAITQUEUE(wait, current);

	/* initialize bked_info */
	INIT_LIST_HEAD(&bked_info.execq_list);
	spin_lock_init(&bked_info.execq_lock);
	init_waitqueue_head(&bked_info.more_exec);
	init_waitqueue_head(&bked_info.exec_done);

	/* detach myself from calling process (e.g. insmod) */
	daemonize("bkexecd");
	
	//reparent_to_init();	/* reparent_to_init not needed in 2.6.x*/

	/* my info */
  sprintf(current->comm, "bkexecd");

	/* Block all signals except SIGKILL and SIGSTOP */
	spin_lock_irq(&current->sighand->siglock);
	siginitsetinv(&current->blocked, sigmask(SIGKILL) | sigmask(SIGSTOP) );
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);

	// loop forever until we have any signal (SIGKILL | SIGSTOP)
  for(;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&bked_info.more_exec, &wait);
		if (list_empty(&bked_info.execq_list)) {
			schedule();
		}
		__set_current_state(TASK_RUNNING);
		remove_wait_queue(&bked_info.more_exec, &wait);

		if (signal_pending(current)) {
			// we got SIGKILL | SIGSTOP
			break;
		}

		if (!list_empty(&bked_info.execq_list)) {
			run_execq();
		}
	}
	PDEBUG(9, "bkexecd thread exit\n");
  return 0;
}

static __init int bkexecd_init(void)
{
	bked_pid = kernel_thread(bkexecd, NULL, CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	return 0;
}

static __exit void bkexecd_exit(void) {
//	kill_proc(bked_pid, SIGKILL, 0);
}

EXPORT_SYMBOL(bked_info);

module_init(bkexecd_init);
module_exit(bkexecd_exit);
MODULE_LICENSE("GPL");
