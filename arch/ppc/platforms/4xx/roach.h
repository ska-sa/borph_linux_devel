/*
 * arch/ppc/platforms/4xx/roach.h
 *
 * Roach board definitions
 *
 * Copyright 2006-2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Wade Farnsworth <wfarnsworth@mvista.com>
 *
 * Copyright 2004 MontaVista Software Inc.
 * Copyright 2006 AMCC
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifdef __KERNEL__
#ifndef __ASM_ROACH_H__
#define __ASM_ROACH_H__

#include <platforms/4xx/ppc440epx.h>

/* Default clock rate */
#if 0
#define ROACH_TMRCLK     (3 * 16500000)
#endif
#define ROACH_TMRCLK     50000000

/*
 * Serial port defines
 */
#define RS_TABLE_SIZE			4
/* UART mappings used before early_serial_setup so should be coherent with U-Boot */
#define UART0_IO_BASE			0xEF600300
#define UART1_IO_BASE			0xEF600400
#define UART2_IO_BASE			0xEF600500
#define UART3_IO_BASE			0xEF600600

#define BASE_BAUD			33177600/3/16

#define STD_UART_OP(num)						\
	{ 0, BASE_BAUD, 0, UART##num##_INT,				\
			(ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST),	\
			iomem_base: (void*)UART##num##_IO_BASE,		\
			io_type: SERIAL_IO_MEM},

#define SERIAL_PORT_DFNS			\
	STD_UART_OP(0)				\
	STD_UART_OP(1)				\
	STD_UART_OP(2)				\
	STD_UART_OP(3)

#endif                          /* __ASM_ROACH_H__ */
#endif                          /* __KERNEL__ */
