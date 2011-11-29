/*
 * Acadia board definitions
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Copyright 2006 AMCC (www.amcc.com)
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifdef __KERNEL__
#ifndef __ASM_ACADIA_H__
#define __ASM_ACADIA_H__

#include <platforms/4xx/ppc405ez.h>
#include <asm/ppcboot.h>

/* Acadia Boards with different revisions may have different SysClk
 * input frequencies
 */
#define BOARD_SYSCLK_REV10	66666666
#define BOARD_SYSCLK_REV11	33333000

#define CS_NAND_0		3	/* use chip select 3 for NAND device 0 */

#define ACADIA_NAND_FLASH_ADDR	0xD0000000ULL
#define ACADIA_NAND_FLASH_SIZE	0x2000

#define ACADIA_CAN0_ADDR	0xef601000UL
#define ACADIA_CAN0_SIZE	0x800
#define ACADIA_CAN0_IRQ		7
#define ACADIA_CAN1_ADDR	0xef601800UL
#define ACADIA_CAN1_SIZE	0x800
#define ACADIA_CAN1_IRQ		8

#define ACADIA_CPLD_ADDR	0x80000000UL

/* The UART clock is based off an internal clock -
 * define BASE_BAUD based on the internal clock and divider(s).
 * Since BASE_BAUD must be a constant, we will initialize it
 * using clock/divider values which U-Boot initializes
 * for typical configurations at various CPU speeds.
 */
#define BASE_BAUD		0

#define PPC4xx_MACHINE_NAME	"AMCC Acadia"

#endif /* __ASM_ACADIA_H__ */
#endif /* __KERNEL__ */
