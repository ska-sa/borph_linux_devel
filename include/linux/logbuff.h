/*
 * (C) Copyright 2007
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _LOGBUFF_H_
#define _LOGBUFF_H_

#ifdef CONFIG_LOGBUFFER

#define LOGBUFF_MAGIC		0xc0de4ced
#define LOGBUFF_LEN		16384
#define LOGBUFF_OVERHEAD	4096
#define LOGBUFF_RESERVE		(LOGBUFF_LEN + LOGBUFF_OVERHEAD)

/* The mapping used here has to be the same as in logbuff_init_ptrs ()
   in u-boot/common/cmd_log.c */

typedef struct {
	unsigned long	tag;
	unsigned long	start;
	unsigned long	con;	/* next char to be sent to consoles	*/
	unsigned long	end;
	unsigned long	chars;
	unsigned char	buf[0];
} logbuff_t;

extern void setup_ext_logbuff(void);
/* arch specific */
#ifdef CONFIG_ALT_LB_LOCATION
extern int setup_ext_logbuff_mem(volatile logbuff_t **lhead, char **lbuf);
#else
extern int setup_ext_logbuff_mem(logbuff_t **lhead, char **lbuf);
#endif

#endif /* CONFIG_LOGBUFFER */

#endif /* _LOGBUFF_H_ */
