/*
 *	mb86290fb_event.h  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#ifndef _MB86290FB_EVENT_H_
#define _MB86290FB_EVENT_H_

#include <asm/atomic.h>
#include <linux/spinlock.h>
#include "mb86290fbsys.h"

/*
 * Event sync flip info
 */
typedef struct {
	unsigned long	condition;
	unsigned long	reg;
	unsigned long	bank;
} mb86290fb_eventflip_t;

/*
 * Event process private box
 */
typedef struct {
	unsigned long	using;
	unsigned long	pattern;
	unsigned long	mode;
	unsigned long	eventflag;
} mb86290fb_eventpbox_t;

/*
 * Event info
 */
#define FLIPLAYER_NUM			4
typedef struct {
	unsigned long		eventflag;
	spinlock_t		evlock;
	mb86290fb_eventflip_t	flip[FLIPLAYER_NUM];
	atomic_t		waitcnt;
	wait_queue_head_t	eventq;
	mb86290fb_eventpbox_t	pbox[MB86290FB_EVENTWAIT_MAX];
} mb86290fb_eventinfo_t;


int mb86290fb_initeventinfo(mb86290fb_eventinfo_t *event);
int mb86290fb_ioctl_setflag(mb86290fb_eventinfo_t *event, unsigned long arg);
int mb86290fb_ioctl_clearflag(mb86290fb_eventinfo_t *event, unsigned long arg);
int mb86290fb_ioctl_waitflag(mb86290fb_eventinfo_t *event, unsigned long arg);
void mb86290fb_eventwakeup(mb86290fb_eventinfo_t *event);

#endif /* _MB86290FB_EVENT_H_ */
