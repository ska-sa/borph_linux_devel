/*
 * Acadia board specific routines
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Based on liveoak.c provided by AMCC
 *
 * Copyright 2006 AMCC (www.amcc.com)
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/blkdev.h>
#include <linux/pci.h>
#include <linux/rtc.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/ndfc.h>
#include <linux/mtd/physmap.h>

#include <asm/system.h>
#include <asm/pci-bridge.h>
#include <asm/processor.h>
#include <asm/machdep.h>
#include <asm/page.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/todc.h>
#include <asm/kgdb.h>
#include <asm/ocp.h>
#include <asm/ibm_ocp_pci.h>

#include <platforms/4xx/ppc405ez.h>

extern bd_t __res;

static struct resource acadia_can_resource0[] = {
	[0] = {
		.start = ACADIA_CAN0_ADDR,
		.end = ACADIA_CAN0_ADDR + ACADIA_CAN0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start	= ACADIA_CAN0_IRQ,
		.end	= ACADIA_CAN0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device acadia_can_device0 = {
	.name		= "ppc405ez_can",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(acadia_can_resource0),
	.resource	= acadia_can_resource0,
};

static struct resource acadia_can_resource1[] = {
	[0] = {
		.start = ACADIA_CAN1_ADDR,
		.end = ACADIA_CAN1_ADDR + ACADIA_CAN1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start	= ACADIA_CAN1_IRQ,
		.end	= ACADIA_CAN1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device acadia_can_device1 = {
	.name		= "ppc405ez_can",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(acadia_can_resource1),
	.resource	= acadia_can_resource1,
};

/*
 * NOR FLASH configuration (using mtd physmap driver)
 */

/* start will be added dynamically, end is always fixed */
static struct resource acadia_nor_resource = {
	.flags = IORESOURCE_MEM,
};

#define RW_PART0_OF	0
#define RW_PART0_SZ	0x180000
#define RW_PART1_SZ	0x200000
/* Partition 2 will be autosized dynamically... */
#define RW_PART3_SZ	0x80000
#define RW_PART4_SZ	0x40000

static struct mtd_partition acadia_nor_parts[] = {
	{
		.name = "kernel",
		.offset = 0,
		.size = RW_PART0_SZ
	},
	{
		.name = "root",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART1_SZ,
	},
	{
		.name = "user",
		.offset = MTDPART_OFS_APPEND,
	},
	{
		.name = "env",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART3_SZ,
	},
	{
		.name = "u-boot",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART4_SZ,
	}
};

static struct physmap_flash_data acadia_nor_data = {
	.width		= 4,
	.parts		= acadia_nor_parts,
	.nr_parts	= ARRAY_SIZE(acadia_nor_parts),
};

static struct platform_device acadia_nor_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev = {
		.platform_data = &acadia_nor_data,
	},
	.num_resources	= 1,
	.resource	= &acadia_nor_resource,
};

/*
 * NAND FLASH configuration (NDFC)
 */
static struct resource acadia_ndfc = {
	.start = (u32)ACADIA_NAND_FLASH_ADDR,
	.end = (u32)ACADIA_NAND_FLASH_ADDR + ACADIA_NAND_FLASH_SIZE - 1,
	.flags = IORESOURCE_MEM,
};

static struct mtd_partition acadia_nand_parts[] = {
        {
                .name   = "content",
                .offset = 0,
                .size   = MTDPART_SIZ_FULL,
        }
};

struct ndfc_controller_settings acadia_ndfc_settings = {
	.ccr_settings = (NDFC_CCR_BS(CS_NAND_0) | NDFC_CCR_ARAC1),
	.ndfc_erpn = (ACADIA_NAND_FLASH_ADDR) >> 32,
};

struct platform_nand_ctrl acadia_nand_ctrl = {
	.priv = &acadia_ndfc_settings,
};

static struct platform_device acadia_ndfc_device = {
	.name = "ndfc-nand",
	.id = 0,
	.dev = {
		.platform_data = &acadia_nand_ctrl,
	},
	.num_resources = 1,
	.resource = &acadia_ndfc,
};

static struct ndfc_chip_settings acadia_chip0_settings = {
	.bank_settings = 0x80002222,
};

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 6, 7},
	.oobfree = {
		 {.offset = 8,
		  .length = 8}}
};

static struct platform_nand_chip acadia_nand_chip0 = {
	.nr_chips = 1,
	.chip_offset = CS_NAND_0,
	.nr_partitions = ARRAY_SIZE(acadia_nand_parts),
	.partitions = acadia_nand_parts,
	.chip_delay = 50,
	.ecclayout = &nand_oob_16,
	.priv = &acadia_chip0_settings,
};

static struct platform_device acadia_nand_device = {
	.name = "ndfc-chip",
	.id = 0,
	.num_resources = 0,
	.dev = {
		.platform_data = &acadia_nand_chip0,
		.parent = &acadia_ndfc_device.dev,
	}
};

static int acadia_setup_platform_devices(void)
{
	/* NOR-FLASH */
	acadia_nor_resource.start = __res.bi_flashstart;
	acadia_nor_resource.end = __res.bi_flashstart +
		__res.bi_flashsize - 1;

	/*
	 * Adjust partition 2 to flash size
	 */
	acadia_nor_parts[2].size = __res.bi_flashsize -
		RW_PART0_SZ - RW_PART1_SZ - RW_PART3_SZ - RW_PART4_SZ;

	platform_device_register(&acadia_nor_device);

	/* NAND-FLASH */
	platform_device_register(&acadia_ndfc_device);
	platform_device_register(&acadia_nand_device);

	/* CAN */
	platform_device_register(&acadia_can_device0);
	platform_device_register(&acadia_can_device1);

	return 0;
}
arch_initcall(acadia_setup_platform_devices);

/* The serial clock for the chip is an internal clock determined by
 * different clock speeds/dividers.
 * Calculate the proper input baud rate and setup the serial driver.
 */
static void __init acadia_early_serial_map(void)
{
	struct uart_port port;
	void *cpld_base;
	u8 board_rev;
	u32 board_sysclk;

	/* Acadia Boards with different revisions may have different
	 * SysClk input frequencies
	 */
	cpld_base = ioremap(ACADIA_CPLD_ADDR, PAGE_SIZE);
	board_rev = in_8(cpld_base);
	if (board_rev <= 0x0c)
		board_sysclk = BOARD_SYSCLK_REV10;
	else
		board_sysclk = BOARD_SYSCLK_REV11;

	/* Setup serial port access */
	memset(&port, 0, sizeof(port));
	port.membase = (void*)UART0_IO_BASE;
	port.irq = UART0_INT;
	port.uartclk = get_base_baud(0, board_sysclk) * 16;
	port.regshift = 0;
	port.iotype = UPIO_MEM;
	port.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	port.line = 0;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 0 failed\n");

	port.membase = (void*)UART1_IO_BASE;
	port.irq = UART1_INT;
	port.uartclk = get_base_baud(1, board_sysclk) * 16;
	port.line = 1;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 1 failed\n");

	iounmap(cpld_base);
}

void __init acadia_setup_arch(void)
{
	ppc4xx_setup_arch();

	ibm_ocp_set_emac(0, 0);

	ocp_sys_info.plb_bus_freq = __res.bi_busfreq;

        acadia_early_serial_map();

	/* Identify the system */
	printk("AMCC PowerPC 405EZ Acadia Platform\n");
}

void __init acadia_map_io(void)
{
	ppc4xx_map_io();
}

void __init platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
			  unsigned long r6, unsigned long r7)
{
	ppc4xx_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = acadia_setup_arch;
	ppc_md.setup_io_mappings = acadia_map_io;
}
