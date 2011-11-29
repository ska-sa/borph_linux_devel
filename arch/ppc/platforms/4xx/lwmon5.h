/*
 * arch/ppc/platforms/4xx/lwmon5.h
 *
 * LWMON5 board definitions
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifdef __KERNEL__
#ifndef __ASM_LWMON5_H__
#define __ASM_LWMON5_H__

#include <platforms/4xx/ppc440epx.h>

/* Default clock rate */
#define LWMON5_TMRCLK     50000000
#define LWMON5_SYSCLK     33300000

#define LWMON5_OCM			0xe0010000
/*
 * Alternative Log Buffer location settings
 */
#define BOARD_ALT_LH_ADDR		PPC440EPX_GPT0_COMP1
#define BOARD_ALT_LB_ADDR		LWMON5_OCM

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

/* PCI support */
#define LWMON5_PCI_CFGREGS_BASE		0x00000001eec00000ULL
#define LWMON5_PCI_CFGA_OFFSET		0
#define LWMON5_PCI_CFGD_OFFSET		0x4

#define LWMON5_PCI_IO_BASE		0x00000001e8000000ULL
#define LWMON5_PCI_IO_SIZE		0x00010000
#define LWMON5_PCI_MEM_OFFSET	  	0x00000000
#define LWMON5_PCI_PHY_MEM_BASE	0x000000080000000ULL
/* PLB base address base as seen by the core, implemented on PLB3 */
/* PLB base address as seen by the SOC : 0x000000180000000ULL    */

#define LWMON5_PCI_LOWER_IO		0x00000000
#define LWMON5_PCI_UPPER_IO		0x0000ffff
#define LWMON5_PCI_LOWER_MEM		0x80000000
#define LWMON5_PCI_UPPER_MEM		0x8fffffff
#define LWMON5_PCI_MEM_BASE		0x80000000

#define LWMON5_PCIL0_BASE		0x00000001ef400000ULL
#define LWMON5_PCIL0_SIZE		0x40

#define LWMON5_PCIL0_PMM0LA		0x000
#define LWMON5_PCIL0_PMM0MA		0x004
#define LWMON5_PCIL0_PMM0PCILA		0x008
#define LWMON5_PCIL0_PMM0PCIHA		0x00C
#define LWMON5_PCIL0_PMM1LA		0x010
#define LWMON5_PCIL0_PMM1MA		0x014
#define LWMON5_PCIL0_PMM1PCILA		0x018
#define LWMON5_PCIL0_PMM1PCIHA		0x01C
#define LWMON5_PCIL0_PMM2LA		0x020
#define LWMON5_PCIL0_PMM2MA		0x024
#define LWMON5_PCIL0_PMM2PCILA		0x028
#define LWMON5_PCIL0_PMM2PCIHA		0x02C
#define LWMON5_PCIL0_PTM1MS		0x030
#define LWMON5_PCIL0_PTM1LA		0x034
#define LWMON5_PCIL0_PTM2MS		0x038
#define LWMON5_PCIL0_PTM2LA		0x03C

#ifdef CONFIG_WD_LWMON5
#define ARCH_HAS_NMI_WATCHDOG		/* See include/linux/nmi.h */
#define BOARD_WATCHDOG_FUNC		wd_lwmon5_kick_early
#endif

#endif                          /* __ASM_LWMON5_H__ */
#endif                          /* __KERNEL__ */
