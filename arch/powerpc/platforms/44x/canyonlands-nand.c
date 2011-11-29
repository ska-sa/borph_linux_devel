/*
 * AMCC Canyonlands NAND flash specific routines
 *
 * Copyright 2008 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Based on the warp-nand.c which is:
 *   Copyright (c) 2008 PIKA Technologies
 *     Sean MacLennan <smaclennan@pikatech.com>
 */

#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/ndfc.h>

#define CS_NAND_0	3	/* use chip select 3 for NAND device 0 */

#define CANYONLANDS_NAND_FLASH_REG_ADDR	0x4E0000000ULL
#define CANYONLANDS_NAND_FLASH_REG_SIZE	0x2000

static struct resource canyonlands_ndfc = {
	.start = CANYONLANDS_NAND_FLASH_REG_ADDR,
	.end   = CANYONLANDS_NAND_FLASH_REG_ADDR + CANYONLANDS_NAND_FLASH_REG_SIZE,
	.flags = IORESOURCE_MEM,
};

static struct mtd_partition nand_parts[] = {
	{
		.name   = "u-boot",
		.offset = 0,
		.size   = 0x0100000
	},
	{
		.name   = "env",
		.offset = 0x0100000,
		.size   = 0x0040000
	},
	{
		.name   = "content",
		.offset = 0x0140000,
		.size   = 0x7EC0000
	},
};

struct ndfc_controller_settings canyonlands_ndfc_settings = {
	.ccr_settings = (NDFC_CCR_BS(CS_NAND_0) | NDFC_CCR_ARAC1),
	.ndfc_erpn = 0,
};

static struct ndfc_chip_settings canyonlands_chip0_settings = {
	.bank_settings = 0x80002222,
};

struct platform_nand_ctrl canyonlands_nand_ctrl = {
	.priv = &canyonlands_ndfc_settings,
};

static struct platform_device canyonlands_ndfc_device = {
	.name = "ndfc-nand",
	.id = 0,
	.dev = {
		.platform_data = &canyonlands_nand_ctrl,
	},
	.num_resources = 1,
	.resource = &canyonlands_ndfc,
};

static struct platform_nand_chip canyonlands_nand_chip0 = {
	.nr_chips = 1,
	.chip_offset = CS_NAND_0,
	.nr_partitions = ARRAY_SIZE(nand_parts),
	.partitions = nand_parts,
	.chip_delay = 50,
	.ecclayout = NULL,		/* use default ECC layout */
	.priv = &canyonlands_chip0_settings,
};

static struct platform_device canyonlands_nand_device = {
	.name = "ndfc-chip",
	.id = 0,
	.num_resources = 1,
	.resource = &canyonlands_ndfc,
	.dev = {
		.platform_data = &canyonlands_nand_chip0,
		.parent = &canyonlands_ndfc_device.dev,
	}
};

static int canyonlands_setup_nand_flash(void)
{
	platform_device_register(&canyonlands_ndfc_device);
	platform_device_register(&canyonlands_nand_device);

	return 0;
}
device_initcall(canyonlands_setup_nand_flash);
