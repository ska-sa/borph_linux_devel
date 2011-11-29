/*
 *	mb86290fb_dma.h  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#ifndef _MB86290FB_DMA_H_
#define _MB86290FB_DMA_H_

#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <asm/atomic.h>

#define DRAWQUEUE_ENTRYMAX	32

#define SET_DRAWQUEUE_RUNNING(flags)	set_bit(0,(void*)&(flags))
#define SET_DRAWQUEUE_REQ_SUSPEND(flags)	\
					set_bit(1,(void*)&(flags))

#define SET_DRAWQUEUE_SUSPEND(flags)	clear_bit(0,(void*)&(flags))
#define CLEAR_DRAWQUEUE_REQ_SUSPEND(flags)	\
					clear_bit(1,(void*)&(flags))

#define IS_DRAWQUEUE_RUNNING(flags)	test_bit(0,(void*)&(flags))
#define IS_DRAWQUEUE_REQ_SUSPEND(flags)	test_bit(1,(void*)&(flags))

#define SET_DRAWING_COMPLETED(flags)	set_bit(0,(void*)&(flags))
#define SET_SLEEPING_CANCELED(flags)	set_bit(1,(void*)&(flags))

#define CLEAR_DRAWING_COMPLETED(flags)	clear_bit(0,(void*)&(flags))
#define CLEAR_SLEEPING_CANCELED(flags)	clear_bit(1,(void*)&(flags))

#define IS_DRAWING_COMPLETED(flags)	test_bit(0,(void*)&(flags))
#define IS_SLEEPING_CANCELED(flags)	test_bit(1,(void*)&(flags))

#define SET_LDMA_ACTIVE(ldmaact)	atomic_set(&(ldmaact)->active,1)
#define SET_LDMA_INACTIVE(ldmaact)	atomic_set(&(ldmaact)->active,0)
#define IS_LDMA_ACTIVE(ldmaact)		atomic_read(&(ldmaact)->active)

/*
 * Draw list
 */
typedef struct {
	unsigned long	draw_id;
	unsigned long	src_addr;
	unsigned long	dst_addr;
	unsigned long	blksize;
	unsigned long	count;
	unsigned long	flag;
	unsigned long	complete;
	atomic_t		waitcnt;
	wait_queue_head_t	waitq;
	struct {
		unsigned long	reg_bsr;
	} mb86295;
} mb86290fb_drawlist_t;


/*
 * Draw queue header
 */
typedef struct {
	mb86290fb_drawlist_t	*top;
	mb86290fb_drawlist_t	*bottom;
	mb86290fb_drawlist_t	*wp;
	mb86290fb_drawlist_t	*rp;
	void			*next;
} mb86290fb_drawqhead_t;


/*
 * Local DMA active/inactive
 */
typedef struct {
	atomic_t	active;
	pid_t	pid_disabler;
} mb86290fb_localdmaact_t;

/*
 * Draw proc. management info.
 */
typedef struct {
	unsigned long		draw_id;
	unsigned long		drawq;
	unsigned long		drawlist_num;
	unsigned long		flags;
	unsigned long		cur_drawq_num;
	mb86290fb_drawqhead_t	*cur_drawq;
	mb86290fb_drawqhead_t	*top_drawq;
	mb86290fb_drawqhead_t	*last_drawq;
	mb86290fb_localdmaact_t	ldmaact;
} mb86290fb_drawman_t;


#define inc_drawlistp(ptr,top,bottom)	\
{\
	if(ptr == (bottom)) ptr = (top); \
	else ptr++; \
}

#define dec_drawlistp(ptr,top,bottom)	\
{\
	if(ptr == (top)) ptr = (bottom); \
	else ptr--; \
}

#define mb86290fb_getdrawid(drawman)	( ((drawman)->draw_id ==0xffffffff) ? \
						((drawman)->draw_id=1) : \
						++(drawman)->draw_id )

/*
 * DMA memory address manager
 */
struct mb86290fb_addrman_tag {
    pid_t   	    pid;
    void    	    *user_addr;
    unsigned long   offset;
    size_t  	    size;
};
typedef struct mb86290fb_addrman_tag	mb86290fb_addrman_t;


int mb86290fb_initdrawman(mb86290fb_drawman_t *drawman);
int mb86290fb_releasedrawman(mb86290fb_drawman_t *drawman);

int		mb86290fb_addrman_append(pid_t pid, void *user_addr,
					 unsigned long offset, size_t size);
int		mb86290fb_addrman_delete(pid_t pid, void *addr);
unsigned long	mb86290fb_addrman_search(pid_t pid, void *addr);

int mb86290fb_ioctl_waitdrawcomplete(mb86290fb_drawman_t *drawman,
				     unsigned long arg);
int mb86290fb_ioctl_polldrawcomplete(mb86290fb_drawman_t *drawman,
				     unsigned long arg);
int mb86290fb_ioctl_canceldraw(mb86290fb_drawman_t* drawman, unsigned long arg);
int mb86290fb_ioctl_canceldrawall(mb86290fb_drawman_t* drawman);
int mb86290fb_ioctl_wakeup(mb86290fb_drawman_t* drawman);
int mb86290fb_ioctl_adddrawqueue(mb86290fb_drawman_t* drawman,
				 unsigned long arg);
int mb86290fb_ioctl_switchdrawqueue(mb86290fb_drawman_t* drawman,
				    unsigned long arg);
int mb86290fb_ioctl_suspenddrawqueue(mb86290fb_drawman_t* drawman);
int mb86290fb_ioctl_resetdrawqueue(mb86290fb_drawman_t* drawman);
int mb86290fb_ioctl_dmatransfer(mb86290fb_drawman_t* drawman,
				unsigned long arg);

int mb86290fb_ioctl_readdrawman(mb86290fb_drawman_t* drawman, unsigned long arg);
int mb86290fb_ioctl_readdrawqueuehead(mb86290fb_drawman_t* drawman, unsigned long arg);
int mb86290fb_ioctl_readdrawlist(mb86290fb_drawman_t* drawman,
				 unsigned long arg);

int mb86290fb_ldma_enable(void);
int mb86290fb_ldma_disable(void);

#endif /* _MB86290FB_DMA_H_ */
