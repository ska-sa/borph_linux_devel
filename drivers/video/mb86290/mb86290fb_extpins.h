/*
 * 	mb86290fb_extpins.h  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#ifndef _MB86290FB_EXTPINS_H_
#define _MB86290FB_EXTPINS_H_

#include <linux/sched.h>

#define MB86290FB_EXTPINS_NUM	14

typedef struct {
	unsigned long	pins_rsvd;
	struct {
		pid_t		pid_using;
		unsigned long	purpose;
	} pin[MB86290FB_EXTPINS_NUM];
} mb86290fb_extpinsinfo_t;

void mb86290fb_initextpinsinfo(mb86290fb_extpinsinfo_t *extpinsinfo);
int  mb86290fb_ioctl_getextpins(mb86290fb_extpinsinfo_t *extpinsinfo,
				unsigned long arg);
int  mb86290fb_ioctl_releaseextpins(mb86290fb_extpinsinfo_t *extpinsinfo,
				    unsigned long arg);
#endif
