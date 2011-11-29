/*
 *	mb86290fb_signal.c  --  MB86290 Series FrameBuffer Driver
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
#include <linux/sched.h>
#include <asm/uaccess.h>

#include "mb86290fbdev.h"

#define	MAX_FACTOR		4

#define	ERR_ALREADY_REGISTERED	-1
#define	ERR_ALREADY_USED	-2
#define	ERR_NOT_USED		-3

struct {
	pid_t	pid;
	int	signum;
} plist[MAX_FACTOR];

int mb86290fb_ioctl_gensignal(mb86290fb_drawman_t* drawman, unsigned long arg)
{
	unsigned long	factor;
	unsigned long	signum;
	unsigned long	*parg;

	parg = (unsigned long*)arg;
	get_user(factor, parg);
	get_user(signum, parg+1);
	PDEBUG("IOCTL: gensignal\n");
	if (plist[factor].pid) {
		if (plist[factor].pid==current->pid) {
			return 0;
		} else {
			return ERR_ALREADY_USED;
		}
	} else {
		plist[factor].pid = current->pid;
		plist[factor].signum = signum;
	}

    return 0;
}

int mb86290fb_ioctl_stopsignal(mb86290fb_drawman_t* drawman, unsigned long arg)
{
	unsigned long	factor;
	unsigned long	*parg;

	parg = (unsigned long*)arg;
	get_user(factor, parg);
	PDEBUG("IOCTL: stopsignal\n");
	if (plist[factor].pid==current->pid) {
		plist[factor].pid = 0;
	} else {
		return ERR_NOT_USED;
	}

	return 0;
}

int mb86290fb_sendsignal(void)
{
	int i;
	PDEBUG("fb_sendsignal: %d\n", plist[0].pid);
	for (i=0; i<MAX_FACTOR; i++) {
		if (plist[i].pid) {
			kill_proc(plist[i].pid, plist[i].signum, 0);
		}
	}
	PDEBUG("fb_sendsignal done\n");

	return 0;
}
