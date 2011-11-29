/*
 *	mb86290fb_memman.c  --  MB86290 Series FrameBuffer Driver
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

#define MEMLIST_ALIVE	1
#define MEMLIST_EMPTY	2

#define ADD_VOIDPTR(ptr,offset)		(void*)((char*)(ptr) + (offset))

static void memman_addnewitem(mb86290fb_memlist_t *ob,
			      mb86290fb_memlist_t *new,
			      unsigned long size);
static mb86290fb_memlist_t *memman_getitem(mb86290fb_memman_t *memman);
static mb86290fb_memlist_t *memman_combine(mb86290fb_memlist_t *item1,
					   mb86290fb_memlist_t *item2);

int mb86290fb_memman_init(mb86290fb_memman_t *memman,
			  int memlist_num,
			  void *mem,
			  unsigned long memsize)
{
	mb86290fb_memlist_t	*p;

	if (memlist_num<1)
		return -1;

	p = &memman->memlist_terminator;
	p->flags = MEMLIST_ALIVE;
	p->top   = ADD_VOIDPTR(mem, memsize);
	p->prev  = &memman->memlist[0];
	p->next  = 0;

	p = &memman->memlist[0];
	p->flags  = MEMLIST_ALIVE | MEMLIST_EMPTY;
	p->top    = mem;
	p->prev   = 0;
	p->next   = &memman->memlist_terminator;

	memman->memlist_end = p + memlist_num;

	for (p+=1; p<memman->memlist_end; p++) {
		p->flags = 0;
	}
	return 0;
}

void *mb86290fb_memman_alloc(mb86290fb_memman_t *memman, unsigned long size)
{
	mb86290fb_memlist_t	*p, *new;
	unsigned long	empsize;

	for (p=&memman->memlist[0]; p->next!=0; p=p->next) {
		if (p->flags & MEMLIST_EMPTY) {
			empsize = (char*)p->next->top - (char*)p->top;
			if (empsize > size) {
				new = memman_getitem(memman);
				if (new == 0)
					return (void*)0;
				memman_addnewitem(p, new, size);
				return p->top;
			} else if (empsize == size) {
				p->flags &= ~MEMLIST_EMPTY;
				return p->top;
			}
		}
	}
	return (void*)0;
}

int mb86290fb_memman_free(mb86290fb_memman_t *memman, void *free)
{
	mb86290fb_memlist_t	*p, *prev;

	for (p=&memman->memlist[0]; p->next!=0; p=p->next) {
		if(p->top == free && (p->flags & MEMLIST_EMPTY) == 0) {
			p->flags |= MEMLIST_EMPTY;

			prev = p->prev;
			if ((prev!=0) && (prev->flags & MEMLIST_EMPTY)) {
				p = memman_combine(prev, p);
			}

			if (p->next->flags & MEMLIST_EMPTY) {
				memman_combine(p, p->next);
			}
			return 0;
		}
	}
	return -1;
}

int mb86290fb_ioctl_allocvram(mb86290fb_memman_t* vramctl,
			      unsigned long arg)
{
	unsigned long	*u32p;
	unsigned long	size;
	unsigned long	mem;

	u32p = (unsigned long*)arg;
	get_user(size, u32p);

	mem = (unsigned long)mb86290fb_memman_alloc(vramctl, size);
	if (mem==0)
		return -ENOMEM;

	mem -= MB86290FB_VRAM_DUMMY_BASE;
	u32p++;
	put_user(mem, u32p);
	return 0;
}

int mb86290fb_ioctl_freevram(mb86290fb_memman_t* vramctl,
			     unsigned long arg)
{
	unsigned long	*u32p;
	unsigned long	addr;
	int			err;

	u32p = (unsigned long*)arg;
	get_user(addr, u32p);

	addr += MB86290FB_VRAM_DUMMY_BASE;
	err = mb86290fb_memman_free(vramctl, (void*)addr);
	if (err==-1)
		return -EINVAL;

	return 0;
}

static void memman_addnewitem(mb86290fb_memlist_t *item1,
			      mb86290fb_memlist_t *item_new,
			      unsigned long size)
{
	mb86290fb_memlist_t	*item2;

	item2 = item1->next;

	item_new->flags = MEMLIST_ALIVE | MEMLIST_EMPTY;
	item_new->top   = ADD_VOIDPTR(item1->top, size);
	item_new->prev  = item1;
	item_new->next  = item2;

	item1->flags &= ~MEMLIST_EMPTY;
	item1->next = item_new;

	item2->prev = item_new;
}

static mb86290fb_memlist_t *memman_getitem(mb86290fb_memman_t *memman)
{
	mb86290fb_memlist_t	*p;

	for (p=&memman->memlist[0]; p<memman->memlist_end; p++) {
		if ((p->flags & MEMLIST_ALIVE) == 0)
			return p;
	}
	return (void*)0;
}

static mb86290fb_memlist_t *memman_combine(mb86290fb_memlist_t *item1,
					   mb86290fb_memlist_t *item2)
{
	mb86290fb_memlist_t	*item3;

	item3 = item2->next;

	item1->next = item3;
	item3->prev = item1;

	item2->flags &= ~MEMLIST_ALIVE;

	return item1;
}

/*
 * For DEBUG
 */
#ifdef DEBUG
#include <stdio.h>
#include <stdlib.h>

void mb86290fb_memman_dump(mb86290fb_memman_t *memman)
{
	mb86290fb_memlist_t	*p;
	int	i;

	printf("memlist_end: 0x%08x\n",(unsigned int)memman->memlist_end);
	for (i=0; ;i++) {
		p = &memman->memlist[i];
		if (p == memman->memlist_end)
			break;

		printf("[%d]\n",i);
		if (p->flags & MEMLIST_ALIVE) {
			if (p->flags & MEMLIST_EMPTY)
				printf("  EMPTY");
			else
				printf("  USING");

			printf("  Size[0x%x]\n", (unsigned int)(p->next->top - p->top));
		}

		printf("  top  : 0x%08x\n", (unsigned int)p->top);

		printf("  prev : 0x%08x", (unsigned int)p->prev);
		if (p->prev!=0)
			printf(" [%d]\n", p->prev - &memman->memlist[0]);
		else
			printf("\n");

		printf("  next : 0x%08x", (unsigned int)p->next);
		if (p->next!=0)
			printf(" [%d]\n", p->next - &memman->memlist[0]);
		else
			printf("\n");
	}
}

void mb86290fb_memman_verify(mb86290fb_memman_t *memman)
{
	mb86290fb_memlist_t	*p, *next, *top;
	int	err = 0;

	top = &memman->memlist[0];
	for (p=top; p->next!=0; p=p->next) {
		next = p->next;
		if (p->top >= next->top) {
			printf("*** Address error : [%d]\n", p - top);
			err++;
		}

		if (next->prev != p) {
			printf("*** Backword link error : [%d]\n", p - top);
			err++;
		}
	}

	if (!err)
		printf("Memlist verify OK\n");
}

int memman_querysize(memman_t *memman, void *mem)
{
	mb86290fb_memlist_t	*p, *top;

	top = &memman->memlist[0];
	for (p=top; p->next!=0; p=p->next) {
		if (p->top == mem) {
			return p->next->top - p->top;
		}
	}
	return -1;
}
#endif /* DEBUG */
