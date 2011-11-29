/*
 * 	mb86290fb_extpins.c  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */

#include <asm/uaccess.h>

#include "mb86290fbdev.h"

void mb86290fb_initextpinsinfo(mb86290fb_extpinsinfo_t *extpinsinfo)
{
	unsigned long	pins;
	int		i;

	/* Clear all reserved bits */
	extpinsinfo->pins_rsvd = 0;

	if (MB86290FB_USE_EEPROM) {
		/* EDO, EDI, ECK, ECS, EE */
		extpinsinfo->pins_rsvd |= GDC_EXTPINS_EEPROM;
		pins = GDC_EXTPINS_EEPROM;
		for (i=0; i<MB86290FB_EXTPINS_NUM; i++) {
			if (pins&0x1) {
				extpinsinfo->pin[i].purpose = GDC_EXTPINS_PURPOSE_EEPROM;
			}
			pins >>= 1;
		}
	}

	if (MB86290FB_USE_BCUCTL) {
		/* BURSTC, TRANSC, SBUSY, BURSTEN */
		extpinsinfo->pins_rsvd |= GDC_EXTPINS_BCUCTL;
		pins = GDC_EXTPINS_BCUCTL;
		for (i=0; i<MB86290FB_EXTPINS_NUM; i++) {
			if(pins&0x1) {
				extpinsinfo->pin[i].purpose = GDC_EXTPINS_PURPOSE_BCUCTL;
			}
			pins >>= 1;
		}
	}
}

int mb86290fb_ioctl_getextpins(mb86290fb_extpinsinfo_t *extpinsinfo,
			       unsigned long arg)
{
	unsigned long	*parg;
	unsigned long	pins, rsvd, purpose;
	int		i;

	parg = (unsigned long*)arg;
	get_user(pins, parg); parg++;
	get_user(purpose, parg);

	/* Using ? */
	rsvd = extpinsinfo->pins_rsvd & pins;
	if (rsvd) {
		for (i=0; i<MB86290FB_EXTPINS_NUM; i++) {
			if (rsvd & 0x1) {
				if (current->pid!=extpinsinfo->pin[i].pid_using ||
				    purpose!=extpinsinfo->pin[i].purpose) {
					return -EAGAIN;
				}
			}
			rsvd >>= 1;
		}
	}

	/* reserves ext. pins */
	extpinsinfo->pins_rsvd |= pins;

	/* sets PID */
	for(i=0; i<MB86290FB_EXTPINS_NUM; i++){
		if (pins & 0x1) {
			extpinsinfo->pin[i].pid_using = current->pid;
			extpinsinfo->pin[i].purpose   = purpose;
		}
		pins >>= 1;
	}
	return 0;
}

int mb86290fb_ioctl_releaseextpins(mb86290fb_extpinsinfo_t *extpinsinfo,
				   unsigned long arg)
{
	unsigned long	*parg;
	unsigned long	pins, lpins;
	int		i;

	parg = (unsigned long*)arg;
	get_user(pins, parg);

	/* Check PID */
	lpins = pins;
	for (i=0; i<MB86290FB_EXTPINS_NUM; i++) {
		if (lpins & 0x1) {
			if (extpinsinfo->pin[i].pid_using != current->pid) {
				return -EACCES;
			}
		}
		lpins >>= 1;
	}

	/* release ext. pins */
	extpinsinfo->pins_rsvd &= ~pins;

	return 0;
}
