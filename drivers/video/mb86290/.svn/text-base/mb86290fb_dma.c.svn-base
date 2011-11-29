/*
 * 	mb86290fb_dma.c  --  MB86290 Series FrameBuffer Driver
 *
 *	Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "mb86290fbdev.h"
#if 0
#include <linux/wrapper.h>
#endif
#define mem_map_reserve(p)      set_bit(PG_reserved, &((p)->flags))
#define mem_map_unreserve(p)    clear_bit(PG_reserved, &((p)->flags))

typedef struct {
	mb86290fb_drawqhead_t	*head;
	unsigned long		draw_id;
	unsigned long		src_addr;
	unsigned long		dst_addr;
	unsigned long		blksize;
	unsigned long		count;
	unsigned long		trans_mode;
	unsigned long		flag;
} mb86290fb_drawparam_t;

#ifdef	MB86290FB_DMAMEM_HIMEM
mb86290fb_addrman_t	addrman_tab[MB86290FB_DMAMEM_MAXSEGMENT] = {{0,}};
unsigned long		dmamem_bus_addr;
#endif

static int mb86290fb_pushdrawq(const mb86290fb_drawparam_t *param);
static mb86290fb_drawqhead_t *mb86290fb_searchdrawqueue(mb86290fb_drawman_t *drawman,
							unsigned long num);
static int mb86290fb_wakeupprocs(mb86290fb_drawman_t* drawman);

int mb86290fb_initdrawq(mb86290fb_drawqhead_t *head,
			unsigned long num)
{
	head->top = (mb86290fb_drawlist_t*)kmalloc(sizeof(mb86290fb_drawlist_t)*num,
						   GFP_KERNEL);
	if (head->top == 0) {
		return -ENOMEM;
	}

	head->bottom = head->top + (num-1);
	head->wp	 = head->top;
	head->rp	 = head->top;
	head->next	 = 0;

	return 0;
}

int mb86290fb_initdrawman(mb86290fb_drawman_t* drawman)
{
	int	ret = 0;

	drawman->draw_id   = 0;
	drawman->drawq	   = 0;
	drawman->drawlist_num  = DRAWQUEUE_ENTRYMAX;
	drawman->flags	   = 0; /* Set all bits to 0 */
	drawman->cur_drawq_num = 0;
	drawman->top_drawq   =
	(mb86290fb_drawqhead_t*)kmalloc(sizeof(mb86290fb_drawqhead_t),
					GFP_KERNEL);
	if (drawman->top_drawq == 0) {
		return -ENOMEM;
	}

	/* Initialize draw queue */
	ret = mb86290fb_initdrawq(drawman->top_drawq, drawman->drawlist_num);
	drawman->cur_drawq = drawman->top_drawq;

	/* Enable local DMA */
	SET_LDMA_ACTIVE(&drawman->ldmaact);

    return ret;
}

int mb86290fb_releasedrawman(mb86290fb_drawman_t* drawman)
{
	mb86290fb_drawqhead_t	*head, *save;

	head = drawman->top_drawq;
	do {
		/* Release draw list area */
		kfree(head->top);
		save = head;
		head = (mb86290fb_drawqhead_t*)head->next;

		/* Release drawq header area */
		kfree(save);
	} while (head!=0);

	return 0;
}

mb86290fb_drawlist_t *mb86290fb_searchdrawlist(mb86290fb_drawqhead_t *head,
					       unsigned long draw_id)
{
	mb86290fb_drawlist_t	*lrp, *lwp;

	lwp = head->wp;
	lrp = head->rp;

	while (lwp != lrp) {
		dec_drawlistp(lwp, head->top, head->bottom);
		if (lwp->draw_id == draw_id)
			return lwp;
	}

	return (mb86290fb_drawlist_t*)0;
}

mb86290fb_drawlist_t *mb86290fb_searchdrawlistall(mb86290fb_drawman_t *drawman,
						  unsigned long draw_id)
{
	mb86290fb_drawqhead_t	*head;
	mb86290fb_drawlist_t	*list;

	head = drawman->top_drawq;
	do {
		list = mb86290fb_searchdrawlist(head, draw_id);
		if (list!=0)
			return list;
		head = (mb86290fb_drawqhead_t*)head->next;
	} while (head!=0);

	return (mb86290fb_drawlist_t*)0;
}

int mb86290fb_ioctl_waitdrawcomplete(mb86290fb_drawman_t *drawman,
				     unsigned long arg)
{
	mb86290fb_drawlist_t	*list;
	unsigned long		*parg;
	unsigned long		draw_id;

	parg = (unsigned long*)arg;
	get_user(draw_id, parg);

	list = mb86290fb_searchdrawlistall(drawman, draw_id);

	if (list==0)
		 return 0; /* NOT found, the list has been rewritten already */

	if (IS_DRAWING_COMPLETED(list->complete))
		return 0;

	atomic_inc(&list->waitcnt);

	/* Drawing has not completed yet, sleeping... zzz */
	wait_event_interruptible(list->waitq,
				 (IS_DRAWING_COMPLETED(list->complete) ||
				  IS_SLEEPING_CANCELED(list->complete) ||
				  (list->draw_id!=draw_id)));

	/* Woke up! */
	if (list->draw_id!=draw_id)
		return 0; /* rewritten already */

	atomic_dec(&list->waitcnt);

	if (IS_DRAWING_COMPLETED(list->complete))
		return 0;	/* completed */
	if (IS_SLEEPING_CANCELED(list->complete))
		return -EINTR; /* incomplete */

	return 0;
}

int mb86290fb_ioctl_polldrawcomplete(mb86290fb_drawman_t *drawman,
				     unsigned long arg)
{
	mb86290fb_drawlist_t	*list;
	unsigned long		*parg;
	unsigned long		draw_id;

	parg = (unsigned long*)arg;
	get_user(draw_id, parg);
	list = mb86290fb_searchdrawlistall(drawman, draw_id);
	if (list==0 || IS_DRAWING_COMPLETED(list->complete))
		return 1; /* completed */

	return 0; /* waiting */
}

static int mb86290fb_wakeupprocs(mb86290fb_drawman_t* drawman)
{
	mb86290fb_drawqhead_t	*head;
	mb86290fb_drawlist_t	*lwp, *lrp;

	head = drawman->top_drawq;
	do {
		lwp = head->wp;
		lrp = head->rp;
		while (lwp != lrp) {
			dec_drawlistp(lwp, head->top, head->bottom);
			SET_SLEEPING_CANCELED(lwp->complete);
			wake_up_interruptible(&lwp->waitq);
		}
		head = (mb86290fb_drawqhead_t*)head->next;
	} while (head!=0);

	return 0;
}

int mb86290fb_ioctl_canceldraw(mb86290fb_drawman_t* drawman,
			       unsigned long arg)
{
	mb86290fb_drawlist_t	*list;
	unsigned long		*parg;
	unsigned long		draw_id;

	parg = (unsigned long*)arg;
	get_user(draw_id, parg);
	list = mb86290fb_searchdrawlistall(drawman, draw_id);
	if (list==0)
		return 0;

	SET_DRAWING_COMPLETED(list->complete);
	wake_up_interruptible(&list->waitq);

	return 0;
}

int mb86290fb_ioctl_canceldrawall(mb86290fb_drawman_t* drawman)
{
	mb86290fb_drawqhead_t	*head;
	mb86290fb_drawlist_t	*list;

	head = drawman->top_drawq;
	do {
		list = head->wp;
		while (list != head->rp) {
			dec_drawlistp(list, head->top, head->bottom);
			SET_DRAWING_COMPLETED(list->complete);
			wake_up_interruptible(&list->waitq);
		}
		head = (mb86290fb_drawqhead_t*)head->next;
	} while (head!=0);

	return 0;
}

int mb86290fb_ioctl_adddrawqueue(mb86290fb_drawman_t* drawman,
				 unsigned long arg)
{
	mb86290fb_drawqhead_t	*head;
	unsigned long		*parg;
	unsigned long		nump;
	unsigned long		num;
	int			err;

	parg = (unsigned long*)arg;
	head = drawman->top_drawq;
	for (num=1; head->next!=0; num++) {
		head = (mb86290fb_drawqhead_t*)head->next;
	}

	head->next = (mb86290fb_drawqhead_t*)kmalloc(sizeof(mb86290fb_drawqhead_t),
						     GFP_KERNEL);
	if (head->next == 0)
		return -ENOMEM;

	err = mb86290fb_initdrawq(head->next, drawman->drawlist_num);
	if (err!=0)
		return err;

	get_user(nump, parg);
	parg = (unsigned long*)nump;
	put_user(num, parg);
	return 0;
}

static unsigned long user_to_kv(unsigned long user_addr)
{
	unsigned long	offset;
	pgd_t		*pgd_off;
	pmd_t		*pmd_off;
	pte_t		*pte_off;
	struct		page *spg;
	void		*kvadr;

	offset = user_addr & ~PAGE_MASK;

	pgd_off = pgd_offset(current->mm, user_addr);
	pmd_off = pmd_offset(pgd_off, user_addr);
	#if 1 /* FIXME: not sure if this is correct */
	pte_off = pte_offset_kernel(pmd_off, user_addr);
	#endif
	spg = pte_page(*pte_off);
	kvadr = page_address(spg);

	return ((unsigned long)kvadr + offset);
}

unsigned long user_to_bus(unsigned long user_addr)
{
	return mb86290fb_dmamem_user_to_bus(user_addr);
}

unsigned long kvadr_to_bus(unsigned long kern_addr)
{
	unsigned long	offset;
	unsigned long	bus_base;
	unsigned long	bus_addr;
	pgd_t		*pgd_off;
	pmd_t		*pmd_off;
	pte_t		*pte_off;
	struct page	*spg;
	void		*kvadr;

	offset = kern_addr & ~PAGE_MASK;

	pgd_off = pgd_offset(&init_mm, kern_addr);
	pmd_off = pmd_offset(pgd_off, kern_addr);
	#if 1 /* FIXME: not sure if this is correct */
	pte_off = pte_offset_kernel(pmd_off, kern_addr);
	#endif
	spg = pte_page(*pte_off);
	kvadr = page_address(spg);
	bus_base = virt_to_bus(kvadr);
	bus_addr = bus_base + offset;
	return bus_addr;
}

#ifdef	MB86290FB_DMAMEM_GFP
int mb86290fb_dmamem_allocate(unsigned char **addr, unsigned long *size)
{
	*addr = (unsigned char*)__get_free_pages(__GFP_DMA, MB86290FB_DMAMEM_SIZE);

	if (*addr==0) {
		return -1;
	} else {
		PDEBUG("dmamem: GFP success.\n");
		*size = MB86290FB_DMAMEM_SIZE_BYTE;
		return 0;
	}
}

void mb86290fb_dmamem_free(unsigned char *addr)
{
	free_pages((unsigned long)addr, MB86290FB_DMAMEM_SIZE);
}

int mb86290fb_dmamem_memmap(struct vm_area_struct *vma)
{
	struct page	*page, *pagestart, *pageend;
	unsigned long	dmamem, dmamem_size;

	dmamem_size = vma->vm_end-vma->vm_start;
	dmamem = (unsigned long)mb86290fb_memman_alloc(mb86290fb_pInfo->dmamem_ctl,
						       dmamem_size);
	if (dmamem==0) {
		printk(KERN_ALERT"[mb86290fb] No space left on DMA memory.\n");
		return -ENOMEM;
	}

	pagestart = virt_to_page(dmamem);
	pageend   = virt_to_page(dmamem + dmamem_size -1);
	for (page=pagestart; page<=pageend; page++) {
		mem_map_reserve(page);
	}

	if (remap_pfn_range(vma, vma->vm_start, (__pa(dmamem)) >> PAGE_SHIFT,
			    vma->vm_end-vma->vm_start, vma->vm_page_prot)) {
		printk(KERN_ALERT"[mb86290fb]mmap: remap failed.\n");
		return -ENOMEM;
	}
	return 0;
}

int mb86290fb_dmamem_munmap(unsigned long addr)
{
	int err;

	err = mb86290fb_memman_free(mb86290fb_pInfo->dmamem_ctl,
				    (void*)user_to_kv(addr));
	if (err==0) {
		return 0;
	} else {
		PDEBUG("freedma: memman delete failed.\n");
		return -EINVAL;
	}
}

unsigned long mb86290fb_dmamem_user_to_bus(unsigned long user_addr)
{
	unsigned long	offset;
	unsigned long	phys_base;
	unsigned long	phys_addr;
	pgd_t		*pgd_off;
	pmd_t		*pmd_off;
	pte_t		*pte_off;
	struct		page *spg;
	void		*kvadr;

	offset = user_addr & ~PAGE_MASK;

	pgd_off = pgd_offset(current->mm, user_addr);
	pmd_off = pmd_offset(pgd_off, user_addr);
	#if 1 /* FIXME: not sure if this is correct */
	pte_off = pte_offset_kernel(pmd_off, user_addr);
	#endif
	spg = pte_page(*pte_off);
	kvadr = page_address(spg);
	phys_base = virt_to_bus(kvadr);
	phys_addr = phys_base + offset;

	return phys_addr;
}
#endif /* MB86290FB_DMAMEM_GFP */


static int mb86290fb_pushdrawq(const mb86290fb_drawparam_t *param)
{
	mb86290fb_drawqhead_t	*head;
	mb86290fb_drawlist_t	*wp;
	mb86290fb_drawlist_t	*rp_dec;
	unsigned long		reg_src;
	unsigned long		reg_dst;
	unsigned long		reg_count;
#ifdef CORAL_LP
	unsigned long		reg_bsr;
	unsigned long		bsize;
	unsigned long		transsize;
#endif

	head = param->head;
	wp = head->wp;
	rp_dec = head->rp;
	dec_drawlistp(rp_dec, head->top, head->bottom);
	if (wp == rp_dec)
		return -ENOMEM; /* Draw queue has fulled */

#ifdef CORAL_LP
	/* BCR.BSIZE */
	if (param->blksize>=4 && param->blksize<=31) {
		bsize = (param->blksize << 22) & 0x0f000000;
	} else {
		bsize = 0x08000000; /* 8 words */
	}

	/* transfer length as a number of dwords */
	transsize = param->count/4;

	/* source, destination */
	switch (param->trans_mode&0x7) {
	case MB86290FB_BURST_LOCAL:
		reg_src = param->src_addr;	/* VRAN offset */
		reg_dst = param->dst_addr;	/* VRAN offset */
		break;
	case MB86290FB_BURST_MASTER_READ:
		reg_src = user_to_bus(param->src_addr);
		reg_dst = param->dst_addr;	/* VRAN offset */
		break;
	case MB86290FB_BURST_MASTER_WRITE:
		reg_src = param->src_addr;	/* VRAN offset */
		reg_dst = user_to_bus(param->dst_addr);
		break;
	case MB86290FB_BURST_MASTER_RW:
		reg_src = user_to_bus(param->src_addr);
		reg_dst = user_to_bus(param->dst_addr);
		break;
	case MB86290FB_BURST_SLAVE_WRITE:
		reg_src = user_to_bus(param->src_addr);
		reg_dst = param->dst_addr;	/* VRAN offset */
		break;
	case MB86290FB_BURST_SLAVE_READ:
		reg_src = param->src_addr;	/* VRAN offset */
		reg_dst = user_to_bus(param->dst_addr);
		break;
	default:
		printk("<1>[MB86]no burst mode selected.\n");
		return -EINVAL;
	}

	/* burst setup register */
	reg_bsr = (param->trans_mode & 0x7)|(MB86290FB_BSR_MASK);

	/* burst control register */
	reg_count = bsize | transsize;

	/* address increment mode */
	if (param->trans_mode & MB86290FB_BURST_SRCINC) {
		reg_count |= MB86290FB_BURST_SRCINC;
	}
	if (param->trans_mode & MB86290FB_BURST_DSTINC) {
		reg_count |= MB86290FB_BURST_DSTINC;
	}

#else /* Coral-LP */
	switch (param->trans_mode&0xF) {
	case MB86290FB_BURST_LOCAL:
		reg_src = param->src_addr;
		reg_dst = param->dst_addr;
		reg_count = param->count/4;
		break;
	default:
		printk("<1>[MB86]supports only local transfer.\n");
		return -EINVAL;
	}
#endif /* Coral-LP */

	/* Set Queue */
	wp->draw_id = param->draw_id;
	wp->src_addr = reg_src;
	wp->dst_addr = reg_dst;
	wp->count = reg_count;
	wp->flag = param->flag;
	CLEAR_DRAWING_COMPLETED(wp->complete);
	CLEAR_SLEEPING_CANCELED(wp->complete);
	atomic_set(&wp->waitcnt, 0);
#ifdef CORAL_LP
	wp->mb86295.reg_bsr = reg_bsr;
#endif

	init_waitqueue_head(&wp->waitq);

	inc_drawlistp(head->wp, head->top, head->bottom);

	return 0;
}

int mb86290fb_ioctl_dmatransfer(mb86290fb_drawman_t* drawman,
				unsigned long arg)
{
	mb86290fb_drawparam_t	drawparam;
	unsigned long		drawqnum;
	unsigned long		draw_idp;
	long			*i32p;
	unsigned long		*parg;
	int			err;

	parg = (unsigned long*)arg;
	get_user(drawparam.trans_mode, parg);
	get_user(drawparam.src_addr, parg+1);
	get_user(drawparam.dst_addr, parg+2);
	get_user(drawparam.blksize, parg+3);
	get_user(drawparam.count, parg+4);
	get_user(drawparam.flag, parg+5);
	get_user(drawqnum, parg+6);
	get_user(draw_idp, parg+7);

	/* Get new draw ID */
	drawparam.draw_id = mb86290fb_getdrawid(drawman);


	/* Search draw queue */
	drawparam.head = mb86290fb_searchdrawqueue(drawman, drawqnum);
	if (drawparam.head == 0)
		return -EINVAL; /* NOT exist */

	/* BCU active? */
	if (!IS_LDMA_ACTIVE(&drawman->ldmaact) && drawparam.head==drawman->cur_drawq)
		return -EAGAIN;

	/* Push to draw queue */
	err = mb86290fb_pushdrawq(&drawparam);
	if (err!=0)
		return err;

	/* Flush display list */
	if (drawparam.head==drawman->cur_drawq &&
	    drawparam.head->rp!=drawparam.head->wp &&
	    IS_DRAWQUEUE_RUNNING(drawman->flags)==0 &&
	    IS_DRAWQUEUE_REQ_SUSPEND(drawman->flags)==0 ) {

		SET_DRAWQUEUE_RUNNING(drawman->flags);
		drawman->last_drawq = drawman->cur_drawq;
		mb86290fb_transfer(drawparam.head->rp);
	}

	/* Return draw ID */
	i32p = (long*)draw_idp;
	put_user(drawparam.draw_id, i32p);

	return 0;
}

int mb86290fb_ioctl_switchdrawqueue(mb86290fb_drawman_t* drawman,
			    unsigned long arg)
{
	mb86290fb_drawqhead_t	*head;
	unsigned long		*parg;
	unsigned long		drawqnum;

	/* BCU active? */
	if (!IS_LDMA_ACTIVE(&drawman->ldmaact))
		return -EAGAIN;

	parg = (unsigned long*)arg;
	get_user(drawqnum, parg);

	/* Search draw queue */
	head = mb86290fb_searchdrawqueue(drawman, drawqnum);
	if (head == 0)
		return -EINVAL; /* NOT exist */

	/* Set current queue */
	drawman->cur_drawq = head;
	drawman->cur_drawq_num = drawqnum;

	/* Clear suspend request */
	CLEAR_DRAWQUEUE_REQ_SUSPEND(drawman->flags);

	/* Check DMA exec status */
	if (IS_DRAWQUEUE_RUNNING(drawman->flags))
		return 0;

	for(;;) {
		/* Queue is empty? */
		if (head->rp == head->wp)
			return 0;

		if (IS_DRAWING_COMPLETED(head->rp->complete) == 0)
			break;
		inc_drawlistp(head->rp, head->top, head->bottom);
	}

	/* Set to `running' */
	SET_DRAWQUEUE_RUNNING(drawman->flags);

	drawman->last_drawq = drawman->cur_drawq;
	mb86290fb_transfer(head->rp);
	return 0;
}

/*
 * Search Draw Queue header
 */
static
mb86290fb_drawqhead_t *mb86290fb_searchdrawqueue(mb86290fb_drawman_t *drawman,
						 unsigned long num) {
	mb86290fb_drawqhead_t	*head;
	int				i;

	head = drawman->top_drawq;
	for (i=0; i<num; i++) {
		if (head->next == 0)
			return 0;
		head = (mb86290fb_drawqhead_t*)head->next;
	}
	return head;
}

int mb86290fb_ioctl_suspenddrawqueue(mb86290fb_drawman_t* drawman) {
	SET_DRAWQUEUE_REQ_SUSPEND(drawman->flags);
	return 0;
}

/*
 * For DEBUG
 */
int mb86290fb_ioctl_readdrawman(mb86290fb_drawman_t* drawman,
				unsigned long arg)
{
	unsigned long	*parg;

	parg = (unsigned long*)arg;
	put_user(drawman->draw_id, parg);
	put_user(drawman->drawq, parg+1);
	put_user(drawman->drawlist_num, parg+2);
	put_user(drawman->flags, parg+3);
	put_user(drawman->cur_drawq_num, parg+4);
	put_user(drawman->cur_drawq, parg+5);
	put_user(drawman->top_drawq, parg+6);

	return 0;
}

int mb86290fb_ioctl_readdrawqueuehead(mb86290fb_drawman_t* drawman,
				      unsigned long arg)
{
	mb86290fb_drawqhead_t	*head;
	unsigned long	*parg;
	unsigned long	num;
	int			i;

	parg = (unsigned long*)arg;
	get_user(num, parg);

	head = drawman->top_drawq;
	for (i=0; i<num; i++) {
		head = head->next;
		if (head==0)
			return -1;
	}
	put_user(head, parg);
	put_user(head->top, parg+1);
	put_user(head->bottom, parg+2);
	put_user(head->wp, parg+3);
	put_user(head->rp, parg+4);
	put_user(head->next, parg+5);
	return 0;
}

int mb86290fb_ioctl_readdrawlist(mb86290fb_drawman_t* drawman,
				 unsigned long arg)
{
	mb86290fb_drawqhead_t	*drawq;
	mb86290fb_drawlist_t	*top;
	unsigned long		*parg, *p;
	unsigned long		qNo, num, waitcnt;
	int 	i;

	p = parg = (unsigned long*)arg;
	get_user(qNo, p);
	get_user(num, p+1);

	drawq = drawman->top_drawq;
	for (i=0; i<qNo; i++) {
		drawq = drawq->next;
		if (drawq==0)
			return -1;
	}

	top = drawq->top;
	waitcnt = atomic_read(&top[num].waitcnt);
	put_user(top[num].draw_id, parg);
	put_user(top[num].src_addr, parg+1);
	put_user(top[num].dst_addr, parg+2);
	put_user(top[num].blksize, parg+3);
	put_user(top[num].count, parg+4);
	put_user(top[num].flag, parg+5);
	put_user(top[num].complete, parg+6);
	put_user(waitcnt, parg+7);
	put_user(top[num].mb86295.reg_bsr, parg+8);

	return 0;
}

int mb86290fb_ioctl_resetdrawqueue(mb86290fb_drawman_t* drawman)
{
	mb86290fb_drawqhead_t	*head;

	mb86290fb_wakeupprocs(drawman);

	drawman->draw_id = 0;
	drawman->drawq = 0;
	drawman->drawlist_num = DRAWQUEUE_ENTRYMAX;
	SET_DRAWQUEUE_SUSPEND(drawman->flags);
	CLEAR_DRAWQUEUE_REQ_SUSPEND(drawman->flags);
	drawman->cur_drawq = drawman->top_drawq;

	for (head = drawman->top_drawq; ;head=head->next) {
		head->bottom = head->top + (drawman->drawlist_num-1);
		head->wp = head->top;
		head->rp = head->top;

		if (head->next == 0)
			break;
	}
	return 0;
}

int mb86290fb_ioctl_freedmamemory(mb86290fb_drawman_t* drawman,
				  unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	addr;
	int		err;

	parg = (GDC_ULONG*)arg;
	get_user(addr, parg);

	err = mb86290fb_dmamem_munmap(addr);
	if (err) {
		PDEBUG("freedma: memman delete failed.\n");
	}
	return err;
}

/* Sample : Slave Write */
void slave_write_set_master(unsigned long *src, unsigned long *dst, int size)
{
	int	i;
	int	regoff;

	while (size) {
		for (regoff=0; regoff<8; regoff++) {
			*(dst+regoff) = *src++;
			if (--size==0) {
				/* For saving TC flag,              */
				/* the last BC flag is not checked. */
				goto exit_trans;
			}
		}
		for (i=0; 1; i++) {
			if ((MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_BST)&0x40000000)!=0) {
				break;
			}
			schedule_timeout(10);
			if (i>1000) {
				PDEBUG("waiting BST failed\n");
				return;
			}
		}
	}

exit_trans:
	return;
}

/* Sample : Slave Read */
void slave_read_set_master(unsigned long *src, unsigned long *dst, int size)
{
	int		i;
	int		regoff;
	unsigned long	stat;

	while (size) {
		for (i=0; 1; i++) {
			stat = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_BST);
			if ((stat&0x40000000)!=0) {
				break;
			}
			schedule_timeout(10);
			if (i>1000) {
				PDEBUG("waiting BST failed\n");
				return;
			}
		}
		for (regoff=0; regoff<8; regoff++) {
			*dst++ = *(src+regoff);
			if (--size==0) {
				goto exit_trans;
			}
		}
	}

exit_trans:
	return;
}

/*=========================================================================
    mb86290fb_trans
---------------------------------------------------------------------------
=========================================================================*/
void mb86290fb_transfer(mb86290fb_drawlist_t *dlist)
{
#ifdef CORAL_LP

	GDC_ULONG	stat;

	/* Burst Transfer */

	stat = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_BST);
	if (stat&0x00FFFFFF) {
		PDEBUG("Burst: data was left(BST=0x%08lx)\n", stat);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BER, 0x00010000);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BST, 0x00000000);
	}

	switch (dlist->mb86295.reg_bsr & 0x7) {
	case MB86290FB_BURST_LOCAL:
	case MB86290FB_BURST_MASTER_READ:
	case MB86290FB_BURST_MASTER_WRITE:
	case MB86290FB_BURST_MASTER_RW:
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BSA, dlist->src_addr);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BDA, dlist->dst_addr);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BCR, dlist->count);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BSR, dlist->mb86295.reg_bsr);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BER, 0x00000001);
		break;
	case MB86290FB_BURST_SLAVE_WRITE:
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BSA, 0);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BDA, dlist->dst_addr);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BCR, dlist->count);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BSR, dlist->mb86295.reg_bsr);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BER, 0x00000001);

		/*** Start : User definition of Slave Write ***/
		slave_write_set_master(__va(dlist->src_addr),
				       (unsigned long*)GdcSystemInfo.host_reg+0x8040/4,
				       dlist->count&0xFFFFFF);
		/***  End  : User definition of Slave Write ***/
		break;

	case MB86290FB_BURST_SLAVE_READ:
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BSA, dlist->src_addr);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BDA, 0);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BCR, dlist->count);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_SRBS, 0x0);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BSR, dlist->mb86295.reg_bsr);
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_BER, 0x00000001);

		/*** Start : User definition of Slave Read ***/
		slave_read_set_master((unsigned long*)GdcSystemInfo.host_reg+0x8040/4,
				      __va(dlist->dst_addr), dlist->count&0xFFFFFF);
		/***  End  : User definition of Slave Read ***/
		break;
	}

#else /* CORAL_LP */

	while (MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_LSTA))
		;
	/* Sets source address */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_LSA, dlist->src_addr);

	/* Sets transfer count */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_LCO, dlist->count);

	/* Transfer request */
	MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_LREQ, 0x1);

#endif /* CORAL_LP */
}

int mb86290fb_ldma_enable()
{
	mb86290fb_localdmaact_t	*ldmaact;

	ldmaact = &mb86290fb_pInfo->drawman.ldmaact;
	if (!IS_LDMA_ACTIVE(ldmaact)) {
		if(current->pid!=ldmaact->pid_disabler) {
			/* No authority */
			return -EACCES;
		}
		SET_LDMA_ACTIVE(ldmaact);
	}
	return 0;
}

int mb86290fb_ldma_disable()
{
	mb86290fb_drawman_t *drawman;
	mb86290fb_localdmaact_t	*ldmaact;

	drawman = &mb86290fb_pInfo->drawman;
	ldmaact = &drawman->ldmaact;
	if (!IS_LDMA_ACTIVE(ldmaact))
		return -EINPROGRESS;
	if (IS_DRAWQUEUE_RUNNING(drawman->flags))
		return -EAGAIN;

	drawman->ldmaact.pid_disabler = current->pid;
	SET_LDMA_INACTIVE(ldmaact);
	return 0;
}
