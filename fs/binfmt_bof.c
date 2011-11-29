/*********************************************************************
 * $Id: binfmt_bof.c,v 1.5 2006/10/31 07:28:56 skhay Exp $
 * File  : binfmt_bof.c
 * Author: Hayden Kwok-Hay So
 * Date  : 12/09/2005
 * Description:
 *   Thie file describes a new binary format to be loaded by a Linux
 * kernel, the BORPH Object File (BOF) format.  A file in BOF format
 * encapsulates both an ELF image and a configuration information for
 * one or more FPGA.
 *
 * Ported to kernel 2.6: 2008/11/03
 *********************************************************************/
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/binfmts.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/elf.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/kselectmap.h>
#include <linux/gfp.h>
#include <linux/kernel.h>

#include <linux/bof.h>
#include <linux/borph.h>
#define HDEBUG
#define HDBG_NAME "binfmt_bof"
#define HDBG_LVL  9
#include <linux/hdebug.h>


#if defined(CONFIG_ROACH) || defined(CONFIG_ROACH2)
#define HAC_TYPE HAC_ROACH
#else
#define HAC_TYPE HAC_BEE2FPGA
#endif

int mkdbuf_reset(struct hwr_addr* a) {
	if (!a || a->class != HAC_TYPE) return -ENOEXEC;
	if (a->addr >= SELECTMAP_NUM_DEVS ) return -ENOEXEC;
	mkd_info->mkd_bufs[a->addr] = mkd_info->mkd_bufe[a->addr];
	return 0;
}

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
		    goto out_releasehwr;
		}
		retval = kernel_read(execq_item->bprm->file, strtab_foff,
				     (char*) strtab, strtab_len);
		if (retval < 0) goto out_releasehwr;

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

			if (retval) goto out_releasehwr;
#endif
			retval = hwrops->configure(&(hwrhdr.addr), 
							   execq_item->bprm->file, 
							   hwrhdr.pl_off,
							   hwrhdr.pl_len);
			if (retval) goto out_releasehwr;
		} else {
			printk(KERN_INFO "no hwrops from hwr class %d\n",
			       hwrhdr.addr.class);
			retval = -ENOEXEC;
			goto out_releasehwr;
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
				goto out_releasehwr;
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
			goto out_releasehwr;
		}

#if !(defined(CONFIG_ROACH) || defined(CONFIG_ROACH2))
    /* we dont want to write random stuff into the ROACH registers */
    PDEBUG(9, "sending greet\n");
    if (bofhdr.b_ekver == 0x0000e001) {
      retval = send_greet(&(hwrhdr.addr), execq_item->bprm);
    } else {
      retval = send_oldgreet(&(hwrhdr.addr), execq_item->bprm);
		}
		if (retval < 0)
			goto out_releasehwr;
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

  /* Store the structure last to ensure files appearing in proc are ready to use */
	execq_item->task->borph_info = bi;

	PDEBUG(9, "chip load done\n");
	return 0;
out_releasehwr:
	release_hwr(&hwrhdr.addr);
 out_delregion:
	borph_exit_fpga(execq_item->task);
	return retval;
}

static inline int bof_has_fpga(struct bofhdr* bhdr) {
	return (bhdr->b_numchip > 0);
}

static int load_bof_binary(struct linux_binprm *bprm, struct  pt_regs *regs)
{
	int retval;
	struct bofhdr bhdr;

	bhdr = *((struct bofhdr *) bprm->buf);
	if (bhdr.ident[0] != 0x19 || bhdr.ident[1] != 'B' ||
	    bhdr.ident[2] != 'O' || bhdr.ident[3] != 'F') {
		return -ENOEXEC;
	}
	PDEBUG(9, "b_machine=0x%x, b_elfmachine=0x%x, b_version=0x%x\n",
	       bhdr.b_machine, bhdr.b_elfmachine, bhdr.b_version);

	// check machine
	if (
#if defined(CONFIG_ROACH) || defined(CONFIG_ROACH2)
	    bhdr.b_machine != BM_ROACH ||
#else
	    bhdr.b_machine != BM_BEE2 ||
#endif
#ifndef __arch_um__
	    bhdr.b_elfmachine != EM_PPC
#else
	    (bhdr.b_elfmachine != EM_386 && bhdr.b_elfmachine != EM_486)
#endif
		) {
		PDEBUG(9, "Wrong b_machine or b_elfmachine!\n");
		return -ENOEXEC;
	}

	// only handle version 6
	if (bhdr.b_version != 6) {
		return -ENOEXEC;
	}

	// bof file is valid
  retval = 0;
	if (bof_has_fpga(&bhdr)) {    
		struct execq_item* execq_item;

		execq_item = kmalloc(sizeof(struct execq_item), GFP_KERNEL);
		execq_item->bprm = bprm;
		execq_item->task = current;
    
	  retval = borph_load_hw(execq_item);

    kfree(execq_item);  // HHH return to slab
	}

	if (retval) {
		PDEBUG(5, "hw load error: %d\n",retval);
		return retval;
	}

	// make it looks like an ELF and start over
	retval = kernel_read(bprm->file, bhdr.b_elfoff, bprm->buf, BINPRM_BUF_SIZE);
	if (retval < 0) {
		PDEBUG(5, "kernel_read failed\n");
		return -ENOEXEC;
	}

	PDEBUG(9, "read elf header at 0x%x [%02x %02x %02x %02x]\n", 
	       bhdr.b_elfoff,
	       bprm->buf[0], bprm->buf[1], bprm->buf[2], bprm->buf[3]);
	return search_binary_handler(bprm,regs);
}

static struct linux_binfmt bof_format = {
	.module		= THIS_MODULE,
	.load_binary	= load_bof_binary,
};

static int __init init_bof_binfmt (void) {
	PDEBUG(0, "binfmt_bof v4 loaded\n");
	return register_binfmt(&bof_format);
}

static void __exit exit_bof_binfmt(void) {
	PDEBUG(0, "binfmt_bof v4 unloaded\n");
	unregister_binfmt(&bof_format);
}


core_initcall(init_bof_binfmt);
module_exit(exit_bof_binfmt);
MODULE_AUTHOR("Hayden Kwok-Hay So");
MODULE_LICENSE("GPL");
