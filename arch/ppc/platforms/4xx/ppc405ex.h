/*
 * arch/ppc/platforms/4xx/ppc405ex.h
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifdef __KERNEL__
#ifndef __ASM_PPC405EX_H__
#define __ASM_PPC405EX_H__

/* ibm405.h at bottom of this file */
#define PPC4xx_ONB_IO_PADDR	((uint)0xef600000)
#define PPC4xx_ONB_IO_VADDR	PPC4xx_ONB_IO_PADDR
#define PPC4xx_ONB_IO_SIZE	((uint)4*1024)

/* serial port defines */
#define RS_TABLE_SIZE	2

#define UART0_INT       26
#define UART1_INT       1
#define IIC0_INT	2
#define IIC1_INT	7
#define EMAC0_INT	24
#define EMAC1_INT	25

#define UART0_IO_BASE	0xEF600200
#define UART1_IO_BASE	0xEF600300

#define EMAC0_BASE      0xEF600900
#define EMAC1_BASE      0xEF600A00

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

#define NR_UICS			3
#define DCRN_UIC0_BASE		0xc0
#define DCRN_UIC1_BASE		0xd0
#define DCRN_UIC2_BASE		0xe0
#define UIC0			DCRN_UIC0_BASE
#define UIC1			DCRN_UIC1_BASE
#define UIC2			DCRN_UIC2_BASE
#define UIC0_UIC1NC      	0x00000002
#define UIC0_UIC2NC		0x00000008

#define DCRN_MAL_BASE           0x180

#define DCRN_SDR_MFR		0x4300

/* Clock and Power Management CPM0 */
#define IBM_CPM_IIC0		0x80000000	/* IIC interface */
#define IBM_CPM_IIC1		0x40000000	/* IIC interface */
#define IBM_CPM_PCI		0x20000000	/* PCI bridge */
#define IBM_CPM_USB1H		0x08000000	/* USB 1.1 Host */
#define IBM_CPM_FPU		0x04000000	/* floating point unit */
#define IBM_CPM_CPU		0x02000000	/* processor core */
#define IBM_CPM_DMA		0x01000000	/* DMA controller */
#define IBM_CPM_BGO		0x00800000	/* PLB3 to OPB bus arbiter */
#define IBM_CPM_EBC		0x00200000	/* External Bus Controller */
#define IBM_CPM_RGMII		0x00100000	/* Reduced Gigabit MII Bridge */
#define IBM_CPM_DMC		0x00080000	/* SDRAM peripheral controller */
#define IBM_CPM_PLB4		0x00040000	/* PLB4 bus arbiter */
#define IBM_CPM_PLB4x3		0x00020000	/* PLB4 to PLB3 bridge controller */
#define IBM_CPM_PLB3x4		0x00010000	/* PLB3 to PLB4 bridge controller */
#define IBM_CPM_PLB3		0x00008000	/* PLB3 bus arbiter */
#define IBM_CPM_NDFC 		0x00004000      /* NAND Flash Controller */
#define IBM_CPM_UIC1		0x00001000	/* Universal Interrupt Controller */
#define IBM_CPM_GPIO0		0x00000800	/* General Purpose IO (??) */
#define IBM_CPM_GPT		0x00000400	/* General Purpose Timers  */
#define IBM_CPM_UART0		0x00000200	/* serial port 0 */
#define IBM_CPM_UART1		0x00000100	/* serial port 1 */
#define IBM_CPM_UIC0		0x00000080	/* Universal Interrupt Controller */
#define IBM_CPM_TMRCLK		0x00000040	/* CPU timers */
#define IBM_CPM_EMAC0		0x00000020	/* ethernet port 0 */
#define IBM_CPM_UART2		0x00000010	/* serial port 2 */
#define IBM_CPM_UART3		0x00000008	/* serial port 3 */
#define IBM_CPM_EMAC1		0x00000004	/* ethernet port 1 */
#define IBM_CPM_P42OPB1		0x00000002	/* USB 2.0 Host*/
#define IBM_CPM_OPB2P4		0x00000001	/* USB 2.0 Host */

/* Clock and Power Management CPM1*/
#define IBM_CPM_UIC2		0x80000000	/* Universal Interrupt Controller 2 */
#define IBM_CPM_SRAM0		0x40000000	/* Internal SRAM Controller */
#define IBM_CPM_MAL0		0x20000000	/* Memory Access Layer */
#define IBM_CPM_USB2D0		0x10000000	/* USB2.0 Device */
#define IBM_CPM_USB2H		0x08000000	/* USB 2.0 HOST */
#define IBM_CPM_CRYP0		0x04000000	/* Security Engine */
#define IBM_CPM_KASU0		0x02000000	/* Kasumi Engine */

#include <asm/ibm405.h>

#endif				/* __ASM_PPC405EX_H__ */
#endif				/* __KERNEL__ */
