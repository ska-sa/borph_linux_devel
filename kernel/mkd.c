/*********************************************************************
 * $Id: mkd.c,v 1.1 2006/10/31 07:28:57 skhay Exp $
 * File  : mkd.c
 * Author: Hayden Kwok-Hay So
 * Date  : 12/15/2005
 * Description:
 *   mkd is the thread that is responsible for bridging between
 * hardware message passing system and the linux kernel.  It is the
 * main brain of BORPH
 *********************************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#define HDEBUG
//#undef HDEBUG
#define HDBG_LVL mkd_info->dbg_lvl
#define HDBG_NAME "mkd"
#include <linux/hdebug.h>
#include <linux/kselectmap.h>
#include <linux/borph.h>
#ifndef __arch_um__
#include "xparameters.h"
#endif

#define USEKTHREAD
struct pphandler_arg {
	struct task_struct* task;
	int devId;
	uint32_t loc;
	uint32_t offset;
	uint32_t size;
};
kmem_cache_t* pphandler_arg_cachep;


/*****************************************************************
 * Global Constant Definitions
 *****************************************************************/
// #undef USEINTERRUPT
#define USEINTERRUPT

#define MKD_VER "cvs-$Revision: 1.1 $"
extern selectmap_t* selectmap_devs;
static int mkd_interrupt_called = 0;

/* helper functions */
static inline int selectmap2buf(selectmap_t *dev, uint32_t cnt)
{
	PDEBUG(9, "selectmap2buf, try to pull %d bytes\n", cnt);
	int id = dev->id;
	unsigned char* bufe = mkd_info->mkd_bufe[id];
	unsigned char* bufs = mkd_info->mkd_bufs[id];
	unsigned buffree = mkdbuf_freep(bufs, bufe);
	int partial_read = 0;

	if (cnt == 0) {
		return 0;
	}
	/* partial read */
	if (buffree < cnt) {
		/* can dead lock if uk sending packet > ~1k */
		PDEBUG(9, "Partial read (dev %d): buffree=%d, cnt=%d\n", 
		       id, buffree, cnt);
		partial_read = 1;
		cnt = buffree;
	}
	/* we are certain we have at least cnt bytes in kernel buffer */
	while (cnt--) {
		selectmap_in_data(dev, *bufe);
		bufe = mkdbuf_add(bufe, 1);
	}
	mkd_info->mkd_bufe[id] = bufe;

	if (partial_read) return -1;

	return 0;
}

int kernel_file_read(struct file *file, char * buf,
		     size_t count, loff_t offset)
{
	mm_segment_t old_fs;
	int retval = -ENOSYS;
	if (offset) {
		if (!file->f_op || !file->f_op->llseek) {
			PDEBUG(0,"offset=%llu !=0 but no llseek found\n",offset);
			goto out;
		}
		retval = file->f_op->llseek(file, offset, 0 /*SEEK_SET*/);
		if (retval)
			goto out;
	}
	if (!file->f_op || !file->f_op->read) {
		retval = -ENOSYS;
		PDEBUG(0,"NO READ FUNCTION!\n");
		goto out;
	}
	PDEBUG(9,"READ FUNCTION is here, let's read!\n");

	old_fs = get_fs();
	set_fs(get_ds());
	retval = file->f_op->read(file, buf, count, &file->f_pos);
	set_fs(old_fs);
 out:
	return retval;
}

int kernel_write(struct file *file, const char * buf,
		 size_t count, loff_t offset)
{
	mm_segment_t old_fs;
	int retval = -ENOSYS;
	if (offset) {
		if (!file->f_op || !file->f_op->llseek) {
			PDEBUG(0,"offset=%llu !=0 but no llseek found\n",offset);
			goto out;
		}
		retval = file->f_op->llseek(file, offset, 0 /*SEEK_SET*/);
		if (retval)
			goto out;
	}
	if (!file->f_op || !file->f_op->write) {
		retval = -ENOSYS;
		PDEBUG(0,"NO WRITE FUNCTION!\n");
		goto out;
	}
	PDEBUG(9,"WRITE FUNCTION is here, let's write!\n");

	old_fs = get_fs();
	set_fs(get_ds());
	retval = file->f_op->write(file, buf, count, &file->f_pos);
	set_fs(old_fs);
 out:
	return retval;
}

static inline struct file * fcheck_task(struct task_struct* task, 
					unsigned int fd)
{
         struct file * file = NULL;
         struct files_struct *files = task->files;

         if (fd < files->max_fds)
                 file = files->fd[fd];
         return file;
}


static void mkd_interrupt(int irq, void *dev_id, struct pt_regs *regs) {
	int i, needmkd;
	uint32_t reg, d, mask;
	selectmap_t* dev;
	struct timeval tv[2];

	if (HDBG_LVL >= 8) {
		do_gettimeofday(tv);
	}

	mkd_interrupt_called += 1;
	/* we need both isr and ier to figure out who calls me
	 * because when a FPGA is unconfigured, it's intr pin
	 * might be high.  Even we can disable that interrupt
	 * via ier, this 1 still shows up at isr.  So we have 
	 * to mask it out ourselves */
	in_intr_reg(SELMAP_INTR_IPISR, reg);
	in_intr_reg(SELMAP_INTR_IPIER, d);
	reg = reg & d;

	mkd_info->selectmap_pending |= reg;
	/* pull all the bytes into my mkd_buf[id] */
	needmkd = 0;
	for (i = 0, mask=0x1UL; i < SELECTMAP_NUM_DEVS; i++, mask <<= 1) {
		if (reg & mask) {
			int buffull, oldipier;
			dev = &selectmap_devs[i];
			/* disable interrupt */
			PDEBUG(9, "disable interrupt for device %d\n", i);
			in_intr_reg(SELMAP_INTR_IPIER, oldipier);
			out_intr_reg(SELMAP_INTR_IPIER, oldipier & ~mask);
			/* clear interrupt (first clear the source from
			 * selectmap, then clear ISR on opbipif*/
			selectmap_out_status(dev, 0x0e000000);
			out_intr_reg(SELMAP_INTR_IPISR, mask);

			selectmap_in_status(dev, d);
			d = SELECTMAP_RFIFO_CNT(d);

			buffull = selectmap2buf(dev, d);
			needmkd = 1;
			/* renable only if not buffer full*/
			if (buffull) {
				// reinstate interrupt, but don't enable it
				PDEBUG(9, "reinstate interupt\n");
				mkd_info->fifo_pending |= mask;
			} else {
				PDEBUG(9, "reenable interupt\n");
				mkd_info->fifo_pending &= ~mask;
				out_intr_reg(SELMAP_INTR_IPIER, oldipier);
			}
		}
	}
	if (needmkd) {
		wake_up_interruptible(&mkd_info->mkdrdq);
	}
	if (HDBG_LVL >= 8) {
		do_gettimeofday(tv+1);
		PDEBUG(8, "ACCT mkd_interrupt:%ld,%ld:%ld,%ld\n", 
		       tv[0].tv_sec, tv[0].tv_usec,tv[1].tv_sec, tv[1].tv_usec);
	}
	return;
}

/**************************** begin ppacket **********************************/

static int selectmap_avail(selectmap_t* dev, int size)
{
	int retry = 400, burst;
	u32 d;
	while (retry) {
		selectmap_in_status(dev, d);
		burst = SELECTMAP_WFIFO_CNT(d);
		PDEBUG(9, "selectmap_avail: request %d, got %d avail\n",
		       size, burst);
		if (burst >= size) return 0;

/*		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(0.01*HZ); */
		retry -= 1;
	}
	return -1;
}

static int selectmap_sendbuf(selectmap_t* dev, int size, buf_t* buf)
{
	int ret, burst;

	PDEBUG(9, "selectmap_sendbuf\n");
	do {
		burst = SELECTMAP_FIFO_NUM_BYTES;
		if (size < burst) burst = size;
		PDEBUG(9, "Try to get %d bytes space\n", burst);
		if ((ret = selectmap_avail(dev, burst)) < 0) {
			return ret;
		}
		PDEBUG(9, "Got it, sending data now");
		size -= burst;
		do {
			selectmap_out_data(dev, *buf++);
		} while (--burst);
	} while (size);
	return 0;
}

static inline void pp_send_readack(unsigned char* buf, int fd, 
				   int size, int devId)
{
	selectmap_t* dev;
	dev = get_selmapdev(devId);
	if (!dev) {
		PDEBUG(0, "pp_send_readack: cannot get dev %d\n", devId);
		return;
	}

	PDEBUG(9, "pp_send_readack, size=%d, buf[0,1,2,3]=[%x %x %x %x]\n",
	       size, buf[0], buf[1], buf[2], buf[3]);
	/* Send header */
	if (selectmap_avail(dev, 8) < 0) {
		PDEBUG(9, "timeout selectmap_avail(8)\n");
	}
	PDEBUG(9, "pp_send_readack got seleptmap buffer\n");
	selectmap_out_data(dev, PP_CMD_READACK);
	selectmap_out_data(dev, (fd >> 16) & 0xFF);
	selectmap_out_data(dev, (fd >> 8) & 0xFF);
	selectmap_out_data(dev, fd & 0xFF);
	selectmap_out_data(dev, (size >> 24) & 0xFF);
	selectmap_out_data(dev, (size >> 16) & 0xFF);
	selectmap_out_data(dev, (size >> 8) & 0xFF);
	selectmap_out_data(dev, size & 0xFF);
	if (size > 0) {
		if (selectmap_sendbuf(dev, size, buf)) {
			printk("Cannot send readack buffer of size %d to dev %d\n",
			       size, devId);
		}
	}
	put_selmapdev(dev);
	return;
}

static inline void pp_send_writeack(int fd, int size, int devId)
{
	selectmap_t* dev;
	PDEBUG(9, "pp_send_writeack, fd=%d, size=%d\n", fd, size);

	dev = get_selmapdev(devId);
	if (!dev) {
		PDEBUG(0, "pp_send_writeack: cannot get dev %d\n", devId);
		return;
	}
	selectmap_out_data(dev, PP_CMD_WRITEACK);
	selectmap_out_data(dev, (fd >> 16) & 0xFF);
	selectmap_out_data(dev, (fd >> 8) & 0xFF);
	selectmap_out_data(dev, fd & 0xFF);
	selectmap_out_data(dev, (size >> 24) & 0xFF);
	selectmap_out_data(dev, (size >> 16) & 0xFF);
	selectmap_out_data(dev, (size >> 8) & 0xFF);
	selectmap_out_data(dev, size & 0xFF);

	put_selmapdev(dev);
	return;
}

static int fringe_chksig(void)
{
	int signaled = 0;
	while(signal_pending(current)) {
		siginfo_t info;
		unsigned long signr;

		signaled = 1;
		spin_lock_irq(&current->sigmask_lock);
		signr = dequeue_signal(&current->blocked, &info);
		spin_unlock_irq(&current->sigmask_lock);
 
		switch(signr) {
		case SIGSTOP: case SIGTSTP: case SIGTTIN: case SIGTTOU:
			PDEBUG(9, "fringe got stopping signal #%lu\n", signr);
			set_current_state(TASK_STOPPED);
			schedule();
			break;
		case SIGKILL: case SIGINT: case SIGTERM:
			PDEBUG(9, "fringe got KILL-ing signal %lu\n", signr);
			return -1;
		case SIGCONT:
			PDEBUG(9, "fringe got SIGCONT\n");
			/* need to repeat the read */
			break;
		default:
			PDEBUG(0, "Unhandled signal %lu received\n", signr);
			break; 
		}
	}
	return signaled;
}

static int read_handler_fringe(void* p)
{
	struct pphandler_arg* arg;
	uint32_t loc, offset, size;
	struct task_struct *task;
	int retval;
	char* page;
	struct files_struct* files;
	struct file* file; 
	struct task_list* tskitem;
	struct borph_info* bi;
	int signaled;
	struct timeval tv[4];

	arg = (struct pphandler_arg*) p;
	loc = arg->loc;
	offset = arg->offset;
	size = arg->size;
	task = arg->task;

	while (!task->did_exec) {
		schedule_timeout(10);
	}

	/* make sure we are connected to the same tty we are supposed to
	 * It is like daemonize(), except we want us to be exactly like task */
	exit_mm(current);
	current->session = task->session;
	current->pgrp = task->pgrp;
	current->tty = task->tty;

	/* Clone my "parent's" fs, files, sighand */
	exit_fs(current);
	current->fs = task->fs;
	atomic_inc(&current->fs->count);

	exit_files(current);
        current->files = task->files;
	atomic_inc(&current->files->count);

	exit_sighand(current);
	current->sig = task->sig;
	atomic_inc(&current->sig->count);

	/* It is a mystery here:
	 * For some reason, the task->blocked is set to everything but SIGKILL
	 * the *first time*, and only the first time. we get to this point,
	 * regardless of whether task->did_exec is set.
	 *
	 * The second time around we get to this point, task->blocked is always
	 * automatically set to the default of all zero (except sig 64).
	 * 
	 * I have no idea why is it the case.  The odd thing is, if I enable
	 * BORPH debugging level to 9, when tons of debugging messages are
	 * printed for each packet received, somehow this odd "all-blocked" 
	 * behavior never shows up.  It is highly likely something to
	 * do with scheduling or some magic signal handling code that
	 * I still haven't figured all out.
	 *
	 * For now, I am giving up and manually set current->block to
	 * empty
	 *
	 * Hayden So (8/1/06)
	 * [for reference here are the old code ]
	 * memcpy(&current->blocked, &task->blocked, sizeof(sigset_t));
	 * printk("MYDEBUG: current->blocked: [1 0] = [0x%lx 0x%lx]\n", current->blocked.sig[1], current->blocked.sig[0]);
	 * printk("       : task->blocked: [1 0] = [0x%lx 0x%lx]\n", task->blocked.sig[1], task->blocked.sig[0]);
	 * printk("       : task->did_exec = %d\n", task->did_exec);
	 */
	sigemptyset(&current->blocked);
	flush_signals(current);
	recalc_sigpending(current);

	reparent_to_init();

	/* this is essentially fget except it doesn't use current */
	retval = -EINVAL;
	files = task->files;
	read_lock(&files->file_lock);
	file = fcheck_task(task, loc);
	if (!file) {
		PDEBUG(0, "invalid loc %d in pp_read_hander (file not found)\n", loc);
		read_unlock(&files->file_lock);
		goto out_freearg;
	}
	get_file(file);
	read_unlock(&files->file_lock);

	if (!(page = (char*) __get_free_page(GFP_KERNEL)))
		goto out_putfile;

	retval = -ENOMEM;
	tskitem = kmem_cache_alloc(task_list_cachep, GFP_KERNEL);
	if (!tskitem) {
		goto out_freepage;
	}
	tskitem->tsk = current;
	tskitem->data = arg;

	bi = task->borph_info;
	write_lock(&bi->hcptr_lock);
	list_add(&tskitem->tsk_list, &bi->hcptr);
	bi->fringes[loc] = tskitem;
	write_unlock(&bi->hcptr_lock);

	/**************************************************
	 * Main Reading Loop 
	 *************************************************/
 loop:
	PDEBUG(9, "Reading from pid=%d, fd=%d, offset=%d, size=%d\n",
	       task->pid, loc, offset, size);
	do {
		/* if a read is interrupted by harmless
		   signal, we repeat the read automatically */
		if (HDBG_LVL >= 8) {
			do_gettimeofday(tv);
		}
		retval = kernel_file_read(file, page, size, offset);
		if (HDBG_LVL >= 8) {
			do_gettimeofday(tv+1);
		}
		PDEBUG(9, "kernel_file_read returns %d bytes\n", retval);
		signaled = fringe_chksig();
		if (signaled < 0)
			goto out;
	} while (signaled);
	/* task might have died while we were blocked on the read */
	if (!task->pid) goto out;

	/* send read ack */
	if (HDBG_LVL >= 8) {
		do_gettimeofday(tv+2);
	}
	pp_send_readack(page, loc, retval, arg->devId);
	if (HDBG_LVL >= 8) {
		do_gettimeofday(tv+3);
		PDEBUG(8, "ACCT read_handler_fringe:%ld,%ld:%ld,%ld:%ld,%ld:%ld,%ld\n",
		       tv[0].tv_sec,tv[0].tv_usec,tv[1].tv_sec,tv[1].tv_usec,
		       tv[2].tv_sec,tv[2].tv_usec,tv[3].tv_sec,tv[3].tv_usec);
	}

	write_lock(&tskitem->data_lock);
	kmem_cache_free(pphandler_arg_cachep, arg);
	tskitem->data = NULL;
	arg = NULL;
	write_unlock(&tskitem->data_lock);
	/**************************************************
	 * Now go to sleep to standby for next read
	 *************************************************/

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!tskitem->data) {
			schedule();
		}
		__set_current_state(TASK_RUNNING);
		
		if (fringe_chksig() < 0)
			goto out;
		if (tskitem->data) {
			arg = (struct pphandler_arg*) tskitem->data;
			if (loc != arg->loc || task != arg->task ) {
				printk("read_fringe woken up with invalid arg:\n");
				printk(" arg->loc = %d, loc = %d\n", arg->loc, loc);
				printk(" arg->task = %p, task = %p\n", arg->task, task);
				retval = -EINVAL;
				goto out;
			}
			offset = arg->offset;
			size = arg->size;
			goto loop;
		}
	}
	
 out:
	tskitem->tsk = NULL;
	tskitem->data = NULL;
	write_lock(&bi->hcptr_lock);
	list_del_init(&tskitem->tsk_list);
	bi->fringes[loc] = NULL;
	write_unlock(&bi->hcptr_lock);
	kmem_cache_free(task_list_cachep, tskitem);
 out_freepage:
	free_page((unsigned long)page);
 out_putfile:
	put_filp(file);
 out_freearg:
	if (arg) {
		kmem_cache_free(pphandler_arg_cachep, arg);
	}
	return retval;
}

/* in this hacky version, loc# always means fd# */
static void pp_read_handler(unsigned char* buf, struct task_struct* task, 
			    int devId)
{
	uint32_t loc, offset, size;
	struct pphandler_arg* arg;
	struct borph_info* bi;
	struct task_list* tskitem;
	if (!task || !task->borph_info) {
		PDEBUG(0, "pp_read_handler error: task==null\n");
		return;
	}
	bi = task->borph_info;

	loc = mkdbuf_getword(devId, 0) & 0xFFFFFF;
	offset = mkdbuf_getword(devId, 4);
	size = mkdbuf_getword(devId, 8);
	IncrBufS(devId, 12);

	arg = kmem_cache_alloc(pphandler_arg_cachep, GFP_KERNEL);
	if (!arg) {
		printk("no mem when getting arg!\n");
		return;
	}
	arg->task = task;
	arg->devId = devId;
	arg->loc = loc;
	arg->offset = offset;
	arg->size = size;
#ifdef USEKTHREAD
	if (loc >= 0 && loc < NR_OPEN_DEFAULT) {
		read_lock(&bi->hcptr_lock);
		tskitem = bi->fringes[loc];
		if (tskitem && tskitem->tsk) {
			write_lock(&tskitem->data_lock);
			tskitem->data = arg;
			write_unlock(&tskitem->data_lock);
			wake_up_process(tskitem->tsk);
		} else {
			kernel_thread(read_handler_fringe, arg, CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
		}
		read_unlock(&bi->hcptr_lock);
	} else {
		printk("Invalid loc=%d in pp_read_handler\n", loc);
	}
#else
	read_handler_helper(arg);
	kmem_cache_free(pphandler_arg_cachep, arg);
#endif
	return;
}

/*
 * We are gauranteed that our packet is contiguous in buf
 */
static void pp_write_handler(unsigned char* buf, struct task_struct* task, 
			     int devId)
{
	uint32_t loc, offset, size;
	buf_t* payload;
	int retval;
	struct files_struct* files;
	struct file* file;
	struct timeval tv[2];

	if (!task) {
		PDEBUG(0, "pp_write_handler error: task==null\n");
		return;
	}

	loc = be32_to_cpup((__u32*) buf) & 0xFFFFFF;
	offset = be32_to_cpup((__u32*) (buf + 4));
	size = be32_to_cpup((__u32*) (buf + 8));
	payload = buf+12;

	PDEBUG(9, "Writing to pid=%d, fd=%d, offset=%d, size=%d, payload=[%x %x %x %x]\n",
	       task->pid, loc, offset, size, buf[12], buf[13], buf[14], buf[15]);
	/* this is similar fget except it writes and doesn't use current */
	files = task->files;
	write_lock(&files->file_lock);
	file = fcheck_task(task, loc);
	if (!file) {
		PDEBUG(0, "invalid loc %d in pp_write_handler\n", loc);
		goto out;
	}
	get_file(file);
	write_unlock(&files->file_lock);

	PDEBUG(9, "just before we call kernel_write: file=%p,"
	       "buf=[%x %x %x %x], size=%d, offset=%d", file, 
	       payload[0], payload[1], payload[2], payload[3], 
	       size, offset);
	if (HDBG_LVL >= 8) {
		do_gettimeofday(tv+0);
	}
	retval = kernel_write(file, payload, size, offset);
	if (HDBG_LVL >= 8) {
		do_gettimeofday(tv+1);
		PDEBUG(8, "ACCT pp_write_handler:%ld,%ld:%ld,%ld\n",
		       tv[0].tv_sec,tv[0].tv_usec,tv[1].tv_sec,tv[1].tv_usec);

	}
	PDEBUG(9, "kernel_write returns %d\n", retval);

//	pp_send_writeack(loc, retval, devId);
	put_filp(file);
 out:
	/* pull packet from mkd_buf[id]*/
	IncrBufS(devId, 12+size);
}

static void pp_exit_handler(struct task_struct* task, int devId)
{
	int exit_code;
	struct siginfo info;
	if (!task) goto out;

	exit_code = mkdbuf_getword(devId, 1) & 0xFFFF;
	PDEBUG(9, "exit code = %d\n", exit_code);
	info.si_signo = SIGKILL;
	info.si_errno = (1 << 16) | exit_code;
	info.si_code = SI_KERNEL;
	info.si_pid = 0;
	info.si_uid = 0;
	send_sig_info(SIGKILL, &info, task);
 out:
	IncrBufS(devId, 5);
}

/* Note, we don't have semaphore on selectmap dev here 
 * return 0 on success
 *        1 if we see partial packet
 *        2 if buffer is empty
 *        <0 on error
 *       -1: misc error
 *       -2: invalid command
 */
static int handle_ppacket(int devId)
{
	struct phyhwr* hwr;
	// int id = dev->id;
	unsigned char* bufs;
	unsigned char* bufe;
	unsigned char* end;
	unsigned char c;
	int buflen;
	struct hwr_addr a;
	int size;
	int retval = 0;

	/* get lock on mkd_buf before we access any mkd_buf*/
	if ((retval = down_interruptible(&mkd_info->bufrsem[devId])) != 0) {
		printk("Failed to get semaphore bufrsem: retval=%d\n", retval);
		return 1;
	}
	retval = 0;

	bufs = mkd_info->mkd_bufs[devId];
	bufe = mkd_info->mkd_bufe[devId];
	buflen = (bufe < bufs)?(bufe + (PAGE_SIZE / 4) - bufs):(bufe-bufs);
	if (buflen <= 0) {
		retval = 2;
		goto out;
	}

	a.addr = devId;
	a.class = HAC_BEE2FPGA;
	hwr = get_hwr(&a);
	if (!hwr) {
		PDEBUG(0, "cannot get hwr %d\n", devId);
		retval = -1;
		goto out;
	}

	if (HDBG_LVL >= 8) {
		struct timeval tv;
		do_gettimeofday(&tv);
		PDEBUG(8, "ACCT handle_ppacket:%ld,%ld\n",tv.tv_sec,tv.tv_usec);
	}
	c = *bufs;
	switch(c) {
	case PP_CMD_READ :
		PDEBUG(9, "received packet PP_CMD_READ.\n");
		retval = 1;
		if (buflen >= 12) {
			pp_read_handler(bufs, hwr->task, devId);
			retval = 0;
		}
		break;
	case PP_CMD_READACK :
		/*  as if I am interrupt handler */
		PDEBUG(9, "received packet PP_CMD_READACK. Wakeup waiting process\n");
		if (buflen < 8) {
			PDEBUG(9, "buflen < 8, packet ignored for now\n");
			retval = 1;
			break;
		}
		size = mkdbuf_getword(devId, 4);
		if (buflen < 8 + size) {
			PDEBUG(9, "readack packet payload not ready, ignored (buflen=%d, size=%d)\n", buflen, size);
			retval = 1;
			break;
		}

		up(&mkd_info->bufrsem[devId]);
		__put_hwr(hwr);
		wake_up_interruptible(&mkd_info->bufrdq[devId]);
		return 1;
	case PP_CMD_WRITEACK :
		/*  as if I am interrupt handler */
		PDEBUG(9, "received packet PP_CMD_WRITEACK. Wakeup waiting process\n");
		if (buflen < 8) {
			PDEBUG(9, "buflen < 8, packet ignored for now\n");
			retval = 1;
			break;
		}
		up(&mkd_info->bufrsem[devId]);
		__put_hwr(hwr);
		wake_up_interruptible(&mkd_info->bufrdq[devId]);
		return 1;
	case PP_CMD_WRITE :
		PDEBUG(9, "received packet PP_CMD_WRITE.\n");
		PDEBUG(10, "buf dump:\n");
		PDEBUG(10, " [%02x %02x %02x %02x %02x %02x %02x %02x "
		       "%02x %02x %02x %02x %02x %02x %02x %02x]\n", 
		       bufs[0], bufs[1], bufs[2], bufs[3], 
		       bufs[4], bufs[5], bufs[6], bufs[7], 
		       bufs[8], bufs[9], bufs[10], bufs[11], 
		       bufs[12], bufs[13], bufs[14], bufs[15]);
		if (buflen < 12) {
			PDEBUG(9, "bunlen < 12, packet ignored for now\n");
			retval = 1;
			break;
		}
		size = mkdbuf_getword(devId, 8);
		if (buflen < 12 + size) {
			PDEBUG(9, "packet payload not ready, ignored (buflen=%d, size=%d)\n", buflen, size);
			retval = 1;
			break;
		}

		/* copy to contigous page before calling handler */
		end = mkdbuf_add(bufs, 12 + size);
		if (end < bufs) {
			buf_t* origin = mkd_info->mkd_buf[devId];
			int clen = origin + (PAGE_SIZE / 4) - bufs;
			buf_t* b = (buf_t*)get_free_page(GFP_KERNEL);
			PDEBUG(5, "Wrap around in packet buffer: origin=%p, end=%p, bufs=%p, clen=%d, size=%d\n", origin, end, bufs, clen, size);
			memcpy(b, bufs, clen);
			memcpy(b+clen, origin, size + 12 - clen);
			pp_write_handler(b, hwr->task, devId);
			free_page((unsigned long) b);
		} else {
			pp_write_handler(bufs, hwr->task, devId);
		}
		break;
	case PP_CMD_EXIT:
		PDEBUG(9, "got PP_CMD_EXIT\n");
		if (buflen < 5) {
			PDEBUG(9, "packet payload not ready, ignored (buflen=%d)\n", buflen);
		}
		pp_exit_handler(hwr->task, devId);
		break;
	default:
		printk("Unknown packet command %d\n", c);
		/* kill the task */
		force_sig(SIGKILL, hwr->task);
		
		{
		    buf_t* p;
		    int i;
		    printk("bufs = 0x%p, bufe = 0x%p, buflen=%d\n", 
			   bufs, bufe, buflen);
		    printk("buf dump:\n");
		    for (p = mkd_info->mkd_buf[devId], i = 0; 
			 i < 1024; i += 8, p += 8) {
			printk("%p  %02x %02x %02x %02x %02x %02x %02x %02x\n", p, p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
		    }
		}
		retval = -1;
	}
	__put_hwr(hwr);
 out:
	up(&mkd_info->bufrsem[devId]);
	return retval;
}

/************ end ppacket ********************************************/

/* NOT USED FOR INTERRUPT CASE */
static inline int handle_selectmap_dev(int devnum)
{
	selectmap_t *dev;
	uint32_t status, cnt;
	int retval;

	dev = get_selmapdev(devnum);
	if (!dev) {
		return -1;
	}

	selectmap_in_status(dev, status);
	cnt = SELECTMAP_RFIFO_CNT(status);
	if (cnt && (cnt <= mkdbuf_free(dev->id))) {
		PDEBUG(9, "selectmap %d has %d bytes available\n", 
		       devnum, cnt);
		selectmap2buf(dev, cnt);
		put_selmapdev(dev);
		
		PDEBUG(9, "BEFORE: bufs=0x%p bufe=0x%p\n", 
		       mkd_info->mkd_bufs[dev->id], 
		       mkd_info->mkd_bufe[dev->id]);
		do { /* nothing */
		} while ((retval = handle_ppacket(dev->id)) == 0);
		PDEBUG(9, "AFTER: bufs=0x%p bufe=0x%p\n", 
		       mkd_info->mkd_bufs[dev->id], 
		       mkd_info->mkd_bufe[dev->id]);
	}
	put_selmapdev(dev);

	return retval;
}

static inline void poll_selectmap(void)
{
	int devnum;
	struct hwr_addr a;
	a.class = HAC_BEE2FPGA;
	PDEBUG(10, "polling selectmap\n");
	for (devnum = 0; devnum < SELECTMAP_NUM_DEVS; devnum++) {
		a.addr = devnum;
		if (hwr_inuse(&a) <= 0) 
			continue;
		if (handle_selectmap_dev(devnum)) continue;
	}
}

/*
 * this is used in interrupted version
 * instead of randomly polling, we chenck the 
 * pending flag in mkd_info , which is set by interrpt 
 * handler
 */
static inline void handle_selectmap(void)
{
	int i, ret;
	unsigned mask = 1UL;
	uint32_t d;
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		if (mkd_info->selectmap_pending & mask) {
			mkd_info->selectmap_pending &= ~mask;
			do { /* nothing */
			} while ((ret = handle_ppacket(i)) == 0);
			
			// re-enable interrupt to make sure we 
			// get interrupt when the remaining of
			// packet arrives
			in_intr_reg(SELMAP_INTR_IPIER, d);
			if (!(d & mask)) {
				PDEBUG(10, "Reenable interrupt for device %d\n", i);
				out_intr_reg(SELMAP_INTR_IPIER, d | mask);
			}
		}
		mask = mask << 1;
	}
}


/*
 * proc entries for debug
 */

static struct proc_dir_entry *p_borph_dir, *p_allmkdbuf;
static struct proc_dir_entry *p_empty[SELECTMAP_NUM_DEVS];
static struct proc_dir_entry *p_mkdbuf[SELECTMAP_NUM_DEVS];
static struct proc_dir_entry *p_dbg_lvl;
static struct proc_dir_entry *p_bversion;
#if SELECTMAP_NUM_DEVS == 5
static char* str_empty[SELECTMAP_NUM_DEVS] = {"empty_mkdbuf0", "empty_mkdbuf1", "empty_mkdbuf2", "empty_mkdbuf3", "empty_mkdbuf4"};
static char* str_mkdbuf[SELECTMAP_NUM_DEVS] = {"mkdbuf0", "mkdbuf1", "mkdbuf2", "mkdbuf3", "mkdbuf4"};
#else
static char* str_empty[SELECTMAP_NUM_DEVS] = {"empty_mkdbuf0", "empty_mkdbuf1", "empty_mkdbuf2", "empty_mkdbuf3"};
static char* str_mkdbuf[SELECTMAP_NUM_DEVS] = {"mkdbuf0", "mkdbuf1", "mkdbuf2", "mkdbuf3"};
#endif
static int proc_empty_mkdbuf_write(struct file *file,
				   const char *buffer,
				   unsigned long count, 
				   void *data)
{
	char c;
	int id = (int) data;
	if (!mkd_info)
	    return -EBUSY;

        MOD_INC_USE_COUNT;
	if (copy_from_user(&c, buffer, 1)) {
		MOD_DEC_USE_COUNT;
		return -EFAULT;
	}
	if (c == '1') {
		mkd_info->mkd_bufs[id] = mkd_info->mkd_bufe[id];
	}
	MOD_DEC_USE_COUNT;
	return count;
}

static int proc_mkdbuf_write(struct file *file, const char *buffer,
			     unsigned long count, void *data)
{
	int id, len;
	buf_t *start, *next, *bufe, *bufs;
	if (!mkd_info)
	    return -EBUSY;

        MOD_INC_USE_COUNT;
	id = (int) data;
	
	start = mkd_info->mkd_buf[id];
	bufe = mkd_info->mkd_bufe[id];
	bufs = mkd_info->mkd_bufs[id];
	printk("proc_mkdbuf_write id=%d bufe=0x%p\n", id, bufe);
	/* I'm lazy */
	if (!access_ok(VERIFY_READ, buffer, count)) {
                MOD_DEC_USE_COUNT;
		return -EFAULT;
	}
	next = bufe;
	
	next = ((((unsigned long) bufe) + 1) % (PAGE_SIZE/4)) + start;
	len = count;
	while (next != bufs && len--) {
		__get_user(*bufe, buffer++);
		printk("buf = 0x%X\n", *bufe);
		bufe = next;
		next = ((((unsigned long) bufe) + 1) % (PAGE_SIZE/4)) + start;
	}

	mkd_info->mkd_bufe[id] = bufe;
	printk("proc_mkdbuf_write exit, bufe=0x%p\n", bufe);
	MOD_DEC_USE_COUNT;
	return count;
}

static int proc_allmkdbuf_read(char *page, char **start,
                            off_t off, int count, 
                            int *eof, void *data)
{
	if (!mkd_info)
	    return -EBUSY;

        MOD_INC_USE_COUNT;
        memcpy(page, mkd_info->mkd_buf[0], PAGE_SIZE);
        MOD_DEC_USE_COUNT;

        return PAGE_SIZE;
}

static int proc_dbglvl_read(char *page, char **start,
			      off_t off, int count, 
			      int *eof, void *data)
{
	if (!mkd_info)
		return -EBUSY;

        MOD_INC_USE_COUNT;
	sprintf(page, "%d\n", mkd_info->dbg_lvl);
        MOD_DEC_USE_COUNT;

        return strlen(page);
}

static int proc_dbglvl_write(struct file *file,
			     const char *buffer,
			     unsigned long count, 
			     void *data)
{
	char c;
	if (!mkd_info)
		return -EBUSY;

        MOD_INC_USE_COUNT;
	if (copy_from_user(&c, buffer, 1)) {
		MOD_DEC_USE_COUNT;
		return -EFAULT;
	}
	if ('0' <= c && c <= '9') {
		mkd_info->dbg_lvl = (c - '0');
	}
	MOD_DEC_USE_COUNT;
	return count;
}

static int proc_bversion_read(char *page, char **start,
                            off_t off, int count, 
                            int *eof, void *data)
{
	const char * bver = "1.0-rc4";
	int bver_len = strlen(bver) + 1;
        MOD_INC_USE_COUNT;
        memcpy(page, bver, bver_len);
        MOD_DEC_USE_COUNT;

        return bver_len;
}

static int __init proc_mkd_init(void)
{
	int retval = -ENOMEM;
	struct proc_dir_entry *tmp;
	int i;

	/* main borth proc entry */
	p_borph_dir = proc_mkdir("borph", NULL);
        if (p_borph_dir == NULL) {
                goto out;
        }
	/* read all buffers */
	p_allmkdbuf = create_proc_read_entry("allmkdbuf", 
                                              0400, p_borph_dir, 
                                              proc_allmkdbuf_read,
                                              NULL);
	if (p_allmkdbuf == NULL) {
		goto no_allmkdbuf;
	}
	p_allmkdbuf->owner = THIS_MODULE;

	/* empty particular mkdbuf */
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		tmp = create_proc_entry(str_empty[i], 0200, p_borph_dir);
		if(tmp == NULL) {
			goto partial_empty;
		}
		tmp->write_proc = proc_empty_mkdbuf_write;
		tmp->data = (void*) i;
		tmp->owner = THIS_MODULE;
		p_empty[i] = tmp;
	}

	/* inject bytes into mkd_buf */
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		tmp = create_proc_entry(str_mkdbuf[i], 0200, p_borph_dir);
		if(tmp == NULL) {
			goto partial_mkdbuf;
		}
		tmp->owner = THIS_MODULE;
		tmp->write_proc = proc_mkdbuf_write;
		tmp->data = (void *) i;
		p_mkdbuf[i] = tmp;
	}

	/* change borph debugging level */
	tmp = create_proc_entry("debug", 0666, p_borph_dir);
	if (tmp == NULL) {
		goto partial_mkdbuf;
	}
	tmp->owner = THIS_MODULE;
	tmp->read_proc = proc_dbglvl_read;
	tmp->write_proc = proc_dbglvl_write;
	tmp->data = NULL;
	p_dbg_lvl = tmp;

	/* store BORPH specific version information */
	p_bversion = create_proc_read_entry("version", 
                                             0444, p_borph_dir, 
                                             proc_bversion_read,
                                             NULL);
	if (p_bversion == NULL) {
		goto partial_dbglvl;
	}
	p_bversion->owner = THIS_MODULE;

	retval = 0;
	goto out;

 partial_dbglvl:
	remove_proc_entry("debug", p_dbg_lvl);
 partial_mkdbuf:
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		if (p_mkdbuf[i]) 
			remove_proc_entry(str_mkdbuf[i], p_borph_dir);
	}
 partial_empty:
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		if (p_empty[i]) 
			remove_proc_entry(str_empty[i], p_borph_dir);
	}
	remove_proc_entry("allmkdbuf", p_borph_dir);
 no_allmkdbuf:
	remove_proc_entry("borph", NULL);
 out:
	return retval;
}

static void __exit proc_mkd_exit(void)
{
	int i;
	remove_proc_entry("debug", p_dbg_lvl);
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		if (p_mkdbuf[i]) 
			remove_proc_entry(str_mkdbuf[i], p_borph_dir);
	}
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
		if (p_empty[i]) 
			remove_proc_entry(str_empty[i], p_borph_dir);
	}
	remove_proc_entry("allmkdbuf", p_borph_dir);
	remove_proc_entry("borph", NULL);
}

static void selectmap_flush(void)
{
	int devnum;
	selectmap_t *dev;
	uint32_t d, old, mask;

	for (devnum = 0, mask = 0x1UL; devnum < SELECTMAP_NUM_DEVS; devnum++, mask <<= 1) {
		dev = get_selmapdev(devnum);
		if (!dev) return;

		/* make sure we won't race with interrupt handler */
		in_intr_reg(SELMAP_INTR_IPIER, old);
		out_intr_reg(SELMAP_INTR_IPIER, old & ~mask);

		if (mkd_info->fifo_pending & mask) {
			selectmap_in_status(dev, d);
			d = SELECTMAP_RFIFO_CNT(d);
			if (!selectmap2buf(dev, d)) {
				mkd_info->selectmap_pending |= mask;
				mkd_info->fifo_pending &= ~mask;
			}
		}
		out_intr_reg(SELMAP_INTR_IPIER, old);

		put_selmapdev(dev);
	}
}

/*
 * Main mkd thread
 */
static int mkd(void *dummy)
{
	int irq_result, i;
	uint32_t d;

        MOD_INC_USE_COUNT;

	/* detach myself from calling process (e.g. insmod) */
	daemonize();
	reparent_to_init();

	/* my info */
        sprintf(current->comm, "mkd");

#if 0
	/* Block all signals except SIGKILL and SIGSTOP */
	spin_lock_irq(&current->sigmask_lock);
	siginitsetinv(&current->blocked, sigmask(SIGKILL) | sigmask(SIGSTOP) );
	recalc_sigpending(current);
	spin_unlock_irq(&current->sigmask_lock);
#endif
	/* Maybe we want to block everything */
	sigfillset(&current->blocked);

	/* initialize info about myself */
	mkd_info = (struct mkd_struct *) kmalloc(sizeof(struct mkd_struct), 
						 GFP_KERNEL);
	if (mkd_info == NULL) {
		printk("mkd: no mem\n");
		return -ENOMEM;
	}
	mkd_info->dbg_lvl = 0;
	PDEBUG(0, "mkd thread enter...\n");

	/* tell my interrupt handler about myself */
	mkd_info->selectmap_pending = 0;
	init_waitqueue_head(&(mkd_info->mkdrdq));
	for (i = 0; i < SELECTMAP_NUM_DEVS; i++) {
	    init_waitqueue_head(&(mkd_info->bufrdq[i]));
	    init_MUTEX(&(mkd_info->bufrsem[i]));
	}

	/* allocate buffer for packets */
	mkd_info->mkd_buf[0] = (unsigned char *)get_free_page(GFP_KERNEL);
	mkd_info->mkd_buf[1] = mkd_info->mkd_buf[0] + (PAGE_SIZE >> 2);
	mkd_info->mkd_buf[2] = mkd_info->mkd_buf[1] + (PAGE_SIZE >> 2);
	mkd_info->mkd_buf[3] = mkd_info->mkd_buf[2] + (PAGE_SIZE >> 2);
	mkd_info->mkd_bufe[0] = mkd_info->mkd_bufs[0] = mkd_info->mkd_buf[0];
	mkd_info->mkd_bufe[1] = mkd_info->mkd_bufs[1] = mkd_info->mkd_buf[1];
	mkd_info->mkd_bufe[2] = mkd_info->mkd_bufs[2] = mkd_info->mkd_buf[2];
	mkd_info->mkd_bufe[3] = mkd_info->mkd_bufs[3] = mkd_info->mkd_buf[3];
#if SELECTMAP_NUM_DEVS == 5
	/* The odd 5th selectmap */
	mkd_info->mkd_buf[4] = (unsigned char *)get_free_page(GFP_KERNEL);
	mkd_info->mkd_bufe[4] = mkd_info->mkd_bufs[4] = mkd_info->mkd_buf[4];
	PDEBUG(9, "mkd_bufs[0 1 2 3 4] @ 0x%p 0x%p 0x%p 0x%p 0x%p\n",
	       mkd_info->mkd_bufs[0], mkd_info->mkd_bufs[1], 
	       mkd_info->mkd_bufs[2], mkd_info->mkd_bufs[3],
	       mkd_info->mkd_bufs[4]);
#else
	PDEBUG(9, "mkd_bufs[0 1 2 3] @ 0x%p 0x%p 0x%p 0x%p\n",
	       mkd_info->mkd_bufs[0], mkd_info->mkd_bufs[1], 
	       mkd_info->mkd_bufs[2], mkd_info->mkd_bufs[3]);
#endif

#ifdef USEINTERRUPT
	/*
	 * Setup Interrupt
	 */
	PDEBUG(5, "setup interrupt...\n");
	/* HHH This seems really odd that selectmap interrupt registers
	 * are mapped here.  It is done so during debugging, when mkd is
	 * compiled as a module and loaded independently.
	 * Once mkd is merged to the main tree (and not modularable), all 
	 * these code should go back to kselectmap.c */
	mkd_info->intrreg = 
		ioremap(XPAR_OPB_SELECTMAP_0_BASEADDR+0x100, 64);
	PDEBUG(5, "ipifreg mapped I/O memory at 0x%p)\n", 
	       mkd_info->intrreg);
	in_intr_reg(SELMAP_INTR_IPISR, d);
	PDEBUG(10, "INTR_IPISR = 0x%08X\n", d);
//	out_be32(SELMAP_INTR_DIER, 0x4);   // this line caused MACHINE CHECK
//	PDEBUG(5, "done writing to INTR_DIER\n");
	irq_result = request_irq(SELECTMAP_IRQ, mkd_interrupt,
				 SA_INTERRUPT, "kselectmap",
				 &mkd_info);
	if (irq_result) {
		PDEBUG(0, "can't get irq %i\n", SELECTMAP_IRQ);
		return -EBUSY;
	}
	PDEBUG(10, "Now enable interrupt on the device\n");
	out_intr_reg(SELMAP_INTR_DGIER, 0x80000000);
	PDEBUG(10, "done writing to INTR_DGIER\n");

	PDEBUG(5, "setup interrupt...done\n");
#endif

	// loop forever until we have any signal (SIGKILL | SIGSTOP)
        for(;;) {
		/* interrupt for selectmap is not quite working
		 * yet.  For now, I'll do slow polling... */
#undef DEBUGINTERRUPT /* debugging interrupt */
#if defined(USEINTERRUPT) && !defined(DEBUGINTERRUPT)
		wait_event_interruptible((mkd_info->mkdrdq), 
					 (mkd_info->selectmap_pending != 0));
		// schedule();
#else
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(50);  // 1000 is about 1 sec
#endif /* USEINTERRUPT */

		if (signal_pending(current)) {
			// we got SIGKILL | SIGSTOP
			break;
		}

#if defined(USEINTERRUPT) && !defined(DEBUGINTERRUPT) 
		handle_selectmap();
		// to make sure the rest of the system can run
		schedule_timeout(10);
		/* It might happen that we have a full mkdbuf when 
		 * user FPGA interrupts me.  In this case, the data are
		 * stuck in the user FPGA FIFO.  But then since it will 
		 * not interrupt us anymore, mkd_interrupt will be not 
		 * called.  Therefore, selectmap_flush is called here, 
		 * which takes the roll of what mkd_interrupt should have done
		 */
		selectmap_flush();
#else
		PDEBUG(10, "interrupt has been called %d times\n", mkd_interrupt_called);
		d = in_be32(SELMAP_INTR_IPISR);
		PDEBUG(10, "IPISR = 0x%08x\n", d);
		d = in_be32(SELMAP_INTR_IPIER);
		PDEBUG(10, "IPIER = 0x%08x\n", d);
		d = in_be32(SELMAP_INTR_DGIER);
		PDEBUG(10, "DGIER = 0x%08x\n", d);
		poll_selectmap();
#endif /* USEINTERRUPT */
	}

#ifdef USEINTERRUPT
	/*
	 * Release Interrupt
	 */
	PDEBUG(5, "freeing interrupt...\n");
	free_irq(SELECTMAP_IRQ, &mkd_info);
#endif


	printk("mkd thread exit\n");
	free_page((unsigned long) mkd_info->mkd_buf[0]);
#if SELECTMAP_NUM_DEVS == 5
	free_page((unsigned long) mkd_info->mkd_buf[4]);
#endif
	{
	    /* This is very ugly.  We really should move such important
	     * centralized information out of mkd.  It was originally here
	     * only because I was writing mkd as a module for easy 
	     * development */
	    struct mkd_struct * tmpstruct = mkd_info;
	    mkd_info = NULL;
	    kfree(tmpstruct);
	}

        MOD_DEC_USE_COUNT;
        return 0;
}


static __init int mkd_init(void)
{
	pphandler_arg_cachep = kmem_cache_create("pphandler_arg", 
						 sizeof(struct pphandler_arg),
						 0, 0, NULL, NULL);
	if (!pphandler_arg_cachep) {
	    return -ENOMEM;
	}
	proc_mkd_init();
	kernel_thread(mkd, NULL, CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	return 0;
}

static __exit void mkd_exit(void)
{
	proc_mkd_exit();
}

/*****************************************************************
 * Module Information (Use only during debugging
 *****************************************************************/
MODULE_AUTHOR("Hayden So");
MODULE_DESCRIPTION("Main mk thread for BORPH");
MODULE_LICENSE("GPL");
#if 0
/*
 * module parameters
 */
static int debug = 0;  /* set to non-0 set mkif into debug mode */
static int reset = 0;  /* set to 1 cause mkif to reset mk at load time */
MODULE_PARM (debug, "i");
MODULE_PARM (reset, "i");
#endif
#define MODULE_NAME "mkd"
module_init(mkd_init);
module_exit(mkd_exit);

