/*
 *	mb86290fb_disp.h  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#ifndef _MB86290FB_DISP_H_
#define _MB86290FB_DISP_H_

#include <linux/sched.h>
#include <asm/atomic.h>
#include "mb86290fb_event.h"
#include "mb86290fb_dma.h"

#define MB86290FB_DISP_DO_FLIP(reg, bank)	\
{\
	unsigned long	value;\
\
	value = MB86290FB_READ_DISP_REGISTER(reg);\
	value = (value & ~GDC_DISP_BANK_MASK) | (bank << GDC_DISP_BANK_SHIFT);\
	MB86290FB_WRITE_DISP_REGISTER(reg, value);\
}

/*
 * Scroll control info
 */
typedef struct {
	unsigned long	ready;
	unsigned long	disp_addr;
	unsigned long	xy;
} mb86290fb_scrollparamC_t;

typedef struct {
	atomic_t			waitcnt;
	wait_queue_head_t		waitq;
	mb86290fb_scrollparamC_t	*rp;
	mb86290fb_scrollparamC_t	*wp;
	mb86290fb_scrollparamC_t	param[2];
} mb86290fb_scrollinfoC_t;

typedef struct {
	unsigned long	ready;
	unsigned long	disp_addr;
} mb86290fb_scrollparamW_t;

typedef struct {
	atomic_t			waitcnt;
	wait_queue_head_t		waitq;
	mb86290fb_scrollparamW_t	*rp;
	mb86290fb_scrollparamW_t	*wp;
	mb86290fb_scrollparamW_t	param[2];
} mb86290fb_scrollinfoW_t;

typedef struct {
	unsigned long	ready;
	unsigned long	disp_addr0;
	unsigned long	disp_addr1;
	unsigned long	xy;
} mb86290fb_scrollparamMB_t;

typedef struct {
	atomic_t			waitcnt;
	wait_queue_head_t		waitq;
	mb86290fb_scrollparamMB_t	*rp;
	mb86290fb_scrollparamMB_t	*wp;
	mb86290fb_scrollparamMB_t	param[2];
} mb86290fb_scrollinfoMB_t;

typedef struct {
	mb86290fb_scrollinfoC_t	C;
	mb86290fb_scrollinfoW_t	W;
	mb86290fb_scrollinfoMB_t	MB[4];
} mb86290fb_scrollinfo_t;


void mb86290fb_initscrollinfo(mb86290fb_scrollinfo_t *scroll);
int mb86290fb_ioctl_dispflipautof(mb86290fb_eventinfo_t *event,
				  unsigned long arg);
int mb86290fb_ioctl_disppossync(mb86290fb_scrollinfo_t *scroll,
				unsigned long arg);
int mb86290fb_ioctl_getdispinfo(mb86290fb_drawman_t *drawman,
				unsigned long arg);
#endif /* _MB86290FB_DISP_H_ */
