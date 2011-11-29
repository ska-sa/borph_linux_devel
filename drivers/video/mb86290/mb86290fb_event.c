/*
 * 	mb86290fb_event.c  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
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
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "mb86290fbdev.h"

static mb86290fb_eventpbox_t *get_eventpbox(mb86290fb_eventinfo_t *event);
static void release_eventpbox(mb86290fb_eventpbox_t *pbox);

int mb86290fb_initeventinfo(mb86290fb_eventinfo_t *event)
{
	int i;

	event->eventflag = 0;
	for (i=0; i<FLIPLAYER_NUM; i++) {
		event->flip[i].condition = 0;
		event->flip[i].bank      = 0;
	}

	event->flip[0].reg = GDC_DISP_REG_ML_MODE_W_H;
	event->flip[1].reg = GDC_DISP_REG_MR_MODE_W_H;
	event->flip[2].reg = GDC_DISP_REG_BL_MODE_W_H;
	event->flip[3].reg = GDC_DISP_REG_BR_MODE_W_H;

	atomic_set(&event->waitcnt, 0);
	init_waitqueue_head(&event->eventq);

	spin_lock_init(&event->evlock);

	for (i=0; i<MB86290FB_EVENTWAIT_MAX; i++) {
		event->pbox[i].using = 0;
	}
	return 0;
}

int mb86290fb_ioctl_setflag(mb86290fb_eventinfo_t *event,
			    unsigned long arg)
{
	mb86290fb_eventflip_t	*flip;
	GDC_ULONG	*parg;
	GDC_ULONG	pattern;
	unsigned long	flags;
	int	i;

	parg = (GDC_ULONG*)arg;
	get_user(pattern, parg);

	/* Flipping */
	for (i=0; i<FLIPLAYER_NUM; i++) {
		flip = &event->flip[i];
		if (flip->condition & pattern) {
			MB86290FB_DISP_DO_FLIP(flip->reg, flip->bank);
			flip->bank ^= 1;
		}
	}

	/* Update event flag */
	spin_lock_irqsave(&event->evlock, flags);
	event->eventflag |= pattern;
	spin_unlock_irqrestore(&event->evlock, flags);

	mb86290fb_eventwakeup(event);

	return 0;
}

int mb86290fb_ioctl_clearflag(mb86290fb_eventinfo_t *event,
			      unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	clear;
	unsigned long	flags;

	parg = (GDC_ULONG*)arg;
	get_user(clear, parg);

	spin_lock_irqsave(&event->evlock, flags);
	event->eventflag &= clear;
	spin_unlock_irqrestore(&event->evlock, flags);

	return 0;
}

int mb86290fb_ioctl_waitflag(mb86290fb_eventinfo_t *event,
			     unsigned long arg)
{
	mb86290fb_eventpbox_t	*pbox;
	GDC_ULONG	*parg;
	GDC_ULONG	pattern, mode, flagp;

	parg = (GDC_ULONG*)arg;
	get_user(pattern, parg);
	get_user(mode   , parg+1);
	get_user(flagp  , parg+2);

	pbox = get_eventpbox(event);
	if (pbox==0)
		return -ENOMEM;

	pbox->pattern   = pattern;
	pbox->mode      = mode;
	pbox->eventflag = event->eventflag;

	atomic_inc(&event->waitcnt);
	switch (mode) {
	case GDC_WAIT_AND:
		wait_event_interruptible(event->eventq,
					 ((pbox->eventflag&pattern)==pattern) );
		break;
	case GDC_WAIT_OR:
		wait_event_interruptible(event->eventq,
					 (pbox->eventflag&pattern));
		break;
	}
	atomic_dec(&event->waitcnt);

	parg = (GDC_ULONG*)flagp;
	put_user(pbox->eventflag, parg);

	release_eventpbox(pbox);

	return 0;
}

void mb86290fb_eventwakeup(mb86290fb_eventinfo_t *event)
{
	mb86290fb_eventpbox_t	*pbox;
	unsigned long	evflag;
	int		i;

	for (i=0; i<MB86290FB_EVENTWAIT_MAX; i++) {
		pbox = &event->pbox[i];
		if (pbox->using == 1) {
			switch (pbox->mode) {
			case GDC_WAIT_AND:
				evflag = event->eventflag;
				if ((evflag&pbox->pattern)==pbox->pattern) {
					pbox->eventflag = evflag;
					wake_up_interruptible(&event->eventq);
				}
				break;
			case GDC_WAIT_OR:
				evflag = event->eventflag;
				if (evflag & pbox->pattern) {
					pbox->eventflag = evflag;
					wake_up_interruptible(&event->eventq);
				}
				break;
			}
		}
	}
}

static mb86290fb_eventpbox_t *get_eventpbox(mb86290fb_eventinfo_t *event)
{
	int i;

	for (i=0; i<MB86290FB_EVENTWAIT_MAX; i++) {
		if (event->pbox[i].using == 0) {
			event->pbox[i].eventflag = event->eventflag;
			event->pbox[i].using = 1;
			return &event->pbox[i];
		}
	}
	return 0;
}

static void release_eventpbox(mb86290fb_eventpbox_t *pbox)
{
	pbox->using = 0;
}
