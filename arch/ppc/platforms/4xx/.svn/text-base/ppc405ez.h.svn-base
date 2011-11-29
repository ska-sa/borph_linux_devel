/*
 * arch/ppc/platforms/4xx/ppc405ez.h
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Copyright 2006 AMCC (www.amcc.com)
 *
 * Based on ibm405gp.h
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifdef __KERNEL__
#ifndef __ASM_PPC405EZ_H__
#define __ASM_PPC405EZ_H__

/* ibm405.h at bottom of this file */
#define PPC4xx_ONB_IO_PADDR	((uint)0xef600000)
#define PPC4xx_ONB_IO_VADDR	PPC4xx_ONB_IO_PADDR
#define PPC4xx_ONB_IO_SIZE	((uint)4*1024)

/* serial port defines */
#define RS_TABLE_SIZE	2

#define UART0_INT       5
#define UART1_INT       6

#define PCIL0_BASE	0xEF400000
#define UART0_IO_BASE	0xEF600300
#define UART1_IO_BASE	0xEF600400

#define EMAC0_BASE      0xEF600900

#define BD_EMAC_ADDR(e,i) bi_enetaddr[i]

#define STD_UART_OP(num)				\
	{ 0, BASE_BAUD, 0, UART##num##_INT,		\
		(ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST),\
		iomem_base: (u8 *)UART##num##_IO_BASE,	\
		io_type: SERIAL_IO_MEM},

#if defined(CONFIG_UART0_TTYS0)
#define SERIAL_DEBUG_IO_BASE	UART0_IO_BASE
#define SERIAL_PORT_DFNS	\
	STD_UART_OP(0)		\
	STD_UART_OP(1)
#endif

#if defined(CONFIG_UART0_TTYS1)
#define SERIAL_DEBUG_IO_BASE	UART1_IO_BASE
#define SERIAL_PORT_DFNS	\
	STD_UART_OP(1)		\
	STD_UART_OP(0)
#endif

#include <asm/ibm405.h>

#ifndef __ASSEMBLY__
unsigned int get_base_baud(unsigned int port, unsigned int sysclk);
#endif /* ! __ASSEMBLY__ */

#endif				/* __ASM_PPC405EZ_H__ */
#endif				/* __KERNEL__ */
