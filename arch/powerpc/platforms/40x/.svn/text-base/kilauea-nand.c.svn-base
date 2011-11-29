/*
 * AMCC Kilauea NAND flash specific routines
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

#define CS_NAND_0	1	/* use chip select 1 for NAND device 0 */

#define KILAUEA_NAND_FLASH_REG_ADDR	0xF8000000UL
#define KILAUEA_NAND_FLASH_REG_SIZE	0x2000

static struct resource kilauea_ndfc = {
	.start = KILAUEA_NAND_FLASH_REG_ADDR,
	.end   = KILAUEA_NAND_FLASH_REG_ADDR + KILAUEA_NAND_FLASH_REG_SIZE,
	.flags = IORESOURCE_MEM,
};

static struct mtd_partition nand_parts[] = {
	{
		.name   = "u-boot",
		.offset = 0,
		.size   = 0x0060000
	},
	{
		.name   = "env",
		.offset = 0x0060000,
		.size   = 0x0008000
	},
	{
		.name   = "content",
		.offset = 0x0068000,
		.size   = 0x3F98000
	},
};

struct ndfc_controller_settings kilauea_ndfc_settings = {
	.ccr_settings = (NDFC_CCR_BS(CS_NAND_0) | NDFC_CCR_ARAC1),
	.ndfc_erpn = 0,
};

static struct ndfc_chip_settings kilauea_chip0_settings = {
	.bank_settings = 0x80002222,
};

struct platform_nand_ctrl kilauea_nand_ctrl = {
	.priv = &kilauea_ndfc_settings,
};

static struct platform_device kilauea_ndfc_device = {
	.name = "ndfc-nand",
	.id = 0,
	.dev = {
		.platform_data = &kilauea_nand_ctrl,
	},
	.num_resources = 1,
	.resource = &kilauea_ndfc,
};

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 3,
	.eccpos = { 0, 1, 2, 3, 6, 7 },
	.oobfree = { {.offset = 8, .length = 16} }
};

static struct platform_nand_chip kilauea_nand_chip0 = {
	.nr_chips = 1,
	.chip_offset = CS_NAND_0,
	.nr_partitions = ARRAY_SIZE(nand_parts),
	.partitions = nand_parts,
	.chip_delay = 50,
	.ecclayout = &nand_oob_16,
	.priv = &kilauea_chip0_settings,
};

static struct platform_device kilauea_nand_device = {
	.name = "ndfc-chip",
	.id = 0,
	.num_resources = 1,
	.resource = &kilauea_ndfc,
	.dev = {
		.platform_data = &kilauea_nand_chip0,
		.parent = &kilauea_ndfc_device.dev,
	}
};

static int kilauea_setup_nand_flash(void)
{
	platform_device_register(&kilauea_ndfc_device);
	platform_device_register(&kilauea_nand_device);

	return 0;
}
device_initcall(kilauea_setup_nand_flash);
