/*
 * Kilauea board definitions
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
#ifndef __ASM_KILAUEA_H__
#define __ASM_KILAUEA_H__

#include <platforms/4xx/ppc405ex.h>
#include <asm/ppcboot.h>

/* Kilauea Boards with different revisions may have different SysClk
 * input frequencies
 */
#define BOARD_SYSCLK		33333000

#define CS_NAND_0		1	/* use chip select 1 for NAND device 0 */

#define KILAUEA_NAND_FLASH_ADDR	0xF8000000ULL
#define KILAUEA_NAND_FLASH_SIZE	0x2000

/* The UART clock is based off an internal clock -
 * define BASE_BAUD based on the internal clock and divider(s).
 * Since BASE_BAUD must be a constant, we will initialize it
 * using clock/divider values which U-Boot initializes
 * for typical configurations at various CPU speeds.
 */
#define BASE_BAUD		0

#define PPC4xx_MACHINE_NAME	"AMCC Kilauea"

/* PCIe support */
#define KILAUEA_PCIE_LOWER_MEM	0x90000000
#define KILAUEA_PCIE_MEM_SIZE	0x08000000
#define BOARD_PCIE_MEM_SIZE	KILAUEA_PCIE_MEM_SIZE	/* used in syslib/ppc4xx_pcie.c */

#define KILAUEA_PCIE_LOWER_IO	0xe0000000
#define KILAUEA_PCIE_IO_SIZE	0x00010000
#define BOARD_PCIE_IO_SIZE	KILAUEA_PCIE_IO_SIZE	/* used in syslib/ppc4xx_pcie.c */

#define BOARD_PCIE_INBOUND_BASE	0x0000000000000000ULL

/*
 * Some cards like LSI8408E need delay before enumeration.
 * At this point calibrate_delay hasn't been called yet so
 * the mdelay value does not reflect exact millisecs value.
 *
 * This value varies from board to board, so let's define a value
 * here in the board config file.
 */
#define BOARD_PCIE_SCAN_DELAY	250

#endif /* __ASM_KILAUEA_H__ */
#endif /* __KERNEL__ */
