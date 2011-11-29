/*
 *	mb86290fb_memman.h  --  MB86290 Series FrameBuffer Driver
 *
 *      Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#ifndef __MEMMAN_H__
#define __MEMMAN_H__

#define MB86290FB_MEMMAN_SIZE(num)	\
	(sizeof(mb86290fb_memlist_t*)+sizeof(mb86290fb_memlist_t)*(num+1))

typedef struct mb86290fb_memlist_tag	mb86290fb_memlist_t;

struct mb86290fb_memlist_tag {
	unsigned long		flags;	/* bit0 is set: alive this item */
					/* bit1 is set: empty           */
	void			*top;
	mb86290fb_memlist_t	*prev;
	mb86290fb_memlist_t	*next;
};

typedef struct {
	mb86290fb_memlist_t	*memlist_end;
	mb86290fb_memlist_t	memlist_terminator;
	mb86290fb_memlist_t	memlist[1];
} mb86290fb_memman_t;


int mb86290fb_memman_init(mb86290fb_memman_t *memman, int memlist_num,
			  void *mem, unsigned long memsize);
void *mb86290fb_memman_alloc(mb86290fb_memman_t *memman, unsigned long size);
int mb86290fb_memman_free(mb86290fb_memman_t *memman, void *free);
int mb86290fb_ioctl_allocvram(mb86290fb_memman_t* vramctl, unsigned long arg);
int mb86290fb_ioctl_freevram(mb86290fb_memman_t* vramctl, unsigned long arg);
#endif
