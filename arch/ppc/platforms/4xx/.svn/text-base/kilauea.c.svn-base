/*
 * Kilauea (405EX) & Haleakala (405EXr) board specific routines
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
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
#include <linux/delay.h>
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
#include <linux/ethtool.h>
#include <linux/i2c.h>

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

#include <platforms/4xx/ppc405ex.h>
#include <syslib/ppc4xx_pcie.h>

extern bd_t __res;

/*
 * I2C RTC
 */
static struct i2c_board_info __initdata kilauea_i2c_devices[] = {
	{
		I2C_BOARD_INFO("rtc-ds1307", 0x68),
		.type = "ds1338",
	},
};

/*
 * NOR FLASH configuration (using mtd physmap driver)
 */

/* start will be added dynamically, end is always fixed */
static struct resource kilauea_nor_resource = {
	.flags = IORESOURCE_MEM,
};

#define RW_PART0_OF	0
#define RW_PART0_SZ	0x200000
#define RW_PART1_SZ	0x200000
/* Partition 2 will be autosized dynamically... */
#define RW_PART3_SZ	0x40000
#define RW_PART4_SZ	0x60000

static struct mtd_partition kilauea_nor_parts[] = {
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

static struct physmap_flash_data kilauea_nor_data = {
	.width		= 2,
	.parts		= kilauea_nor_parts,
	.nr_parts	= ARRAY_SIZE(kilauea_nor_parts),
};

static struct platform_device kilauea_nor_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev = {
		.platform_data = &kilauea_nor_data,
	},
	.num_resources	= 1,
	.resource	= &kilauea_nor_resource,
};

/*
 * NAND FLASH configuration (NDFC)
 */
static struct resource kilauea_ndfc = {
	.start = (u32)KILAUEA_NAND_FLASH_ADDR,
	.end = (u32)KILAUEA_NAND_FLASH_ADDR + KILAUEA_NAND_FLASH_SIZE - 1,
	.flags = IORESOURCE_MEM,
};

static struct mtd_partition kilauea_nand_parts[] = {
        {
                .name   = "content",
                .offset = 0,
                .size   = MTDPART_SIZ_FULL,
        }
};

struct ndfc_controller_settings kilauea_ndfc_settings = {
	.ccr_settings = (NDFC_CCR_BS(CS_NAND_0) | NDFC_CCR_ARAC1),
	.ndfc_erpn = (KILAUEA_NAND_FLASH_ADDR) >> 32,
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

static struct ndfc_chip_settings kilauea_chip0_settings = {
	.bank_settings = 0x80002222,
};

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 6, 7},
	.oobfree = {
		 {.offset = 8,
		  .length = 8}}
};

static struct platform_nand_chip kilauea_nand_chip0 = {
	.nr_chips = 1,
	.chip_offset = CS_NAND_0,
	.nr_partitions = ARRAY_SIZE(kilauea_nand_parts),
	.partitions = kilauea_nand_parts,
	.chip_delay = 50,
	.ecclayout = &nand_oob_16,
	.priv = &kilauea_chip0_settings,
};

static struct platform_device kilauea_nand_device = {
	.name = "ndfc-chip",
	.id = 0,
	.num_resources = 0,
	.dev = {
		.platform_data = &kilauea_nand_chip0,
		.parent = &kilauea_ndfc_device.dev,
	}
};

static int __init kilauea_setup_platform_devices(void)
{
	/* NOR-FLASH */
	kilauea_nor_resource.start = __res.bi_flashstart;
	kilauea_nor_resource.end = __res.bi_flashstart +
		__res.bi_flashsize - 1;

	/*
	 * Adjust partition 2 to flash size
	 */
	kilauea_nor_parts[2].size = __res.bi_flashsize -
		RW_PART0_SZ - RW_PART1_SZ - RW_PART3_SZ - RW_PART4_SZ;

	platform_device_register(&kilauea_nor_device);

	/* NAND-FLASH */
	platform_device_register(&kilauea_ndfc_device);
	platform_device_register(&kilauea_nand_device);

	/* I2C devices */
	i2c_register_board_info(0, kilauea_i2c_devices,
				ARRAY_SIZE(kilauea_i2c_devices));

	return 0;
}
arch_initcall(kilauea_setup_platform_devices);

#ifdef CONFIG_PCI
enum kilauea_hoses {
	HOSE_PCIE0,
	HOSE_PCIE1,
	HOSE_MAX
};

static enum kilauea_hoses hose_type[2];

/*
 * Some IRQs unique to Kilauea
 */
int __init kilauea_map_irq(struct pci_dev *dev, unsigned char idsel,
			   unsigned char pin)
{
	struct pci_controller *hose = pci_bus_to_hose(dev->bus->number);

	if (hose_type[hose->index] == HOSE_PCIE0) {
		static char pci_irq_table[][4] =
			/*
			 *  PCI IDSEL/INTPIN->INTLINE
			 *    A   B   C   D
			 */
			{
				{ 64, 65, 66, 67 },
			};
		const long min_idsel = 1, max_idsel = 1, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else if (hose_type[hose->index] == HOSE_PCIE1) {
		static char pci_irq_table[][4] =
			/*
			 *  PCI IDSEL/INTPIN->INTLINE
			 *    A   B   C   D
			 */
			{
				{ 75, 76, 77, 78 },
			};
		const long min_idsel = 1, max_idsel = 1, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}

	return -1;
}

static int __init kilauea_pcie_card_present(int port)
{
	/*
	 * Kilauea can't detect a PCIe board, so we always report it
	 * as present
	 */
	return 1;
}

static void __init kilauea_setup_hoses(void)
{
	struct pci_controller *hose;
	char name[20];
	int hs;
	int bus_no = 0;
	int hose_max = 2;

	/*
	 * Haleakala (405EXr) only supports one PCIe interface
	 */
	if ((mfspr(SPRN_PVR) & 0x00000004) == 0x00000000)
		hose_max--;

	for (hs = 0; hs < hose_max; ++hs) {
		if (!kilauea_pcie_card_present(hs))
			continue;

		pr_debug(KERN_INFO "PCIE%d: card present\n", hs);

		if (ppc4xx_init_pcie_root_or_endport(hs)) {
			printk(KERN_ERR "PCIE%d: initialization "
			       "failed\n", hs);
			continue;
		}

		hose = pcibios_alloc_controller();
		if (!hose)
			return;

		sprintf(name, "PCIE%d host bridge", hs);

		hose->io_space.start = KILAUEA_PCIE_LOWER_IO +
			hs * KILAUEA_PCIE_IO_SIZE;
		hose->io_space.end = hose->io_space.start +
			KILAUEA_PCIE_IO_SIZE - 1;

		pci_init_resource(&hose->io_resource,
				  hose->io_space.start,
				  hose->io_space.end,
				  IORESOURCE_IO,
				  name);

		hose->mem_space.start = KILAUEA_PCIE_LOWER_MEM +
			hs * KILAUEA_PCIE_MEM_SIZE;
		hose->mem_space.end = hose->mem_space.start +
			KILAUEA_PCIE_MEM_SIZE - 1;

		pci_init_resource(&hose->mem_resources[0],
				  hose->mem_space.start,
				  hose->mem_space.end,
				  IORESOURCE_MEM,
				  name);

		hose->first_busno = bus_no;
		hose->last_busno = 0xff;
		hose_type[hose->index] = hs;

		if (ppc4xx_setup_pcie(hose, hs) != 0) {
			printk(KERN_ERR "PCIE setup failed for hose no %d\n", hs);
			continue;
		}

		/*
		 * Some cards like LSI8408E need delay before enumeration.
		 * At this point calibrate_delay hasn't been called yet so
		 * the mdelay value does not reflect exact millisecs value.
		 */
		mdelay(500);

		hose->last_busno = pciauto_bus_scan(hose, hose->first_busno);
		bus_no = hose->last_busno + 1;
		pr_debug("%s: resources allocated\n", name);
	}

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = kilauea_map_irq;
}
#endif

/* The serial clock for the chip is an internal clock determined by
 * different clock speeds/dividers.
 * Calculate the proper input baud rate and setup the serial driver.
 */
static void __init kilauea_early_serial_map(void)
{
	struct uart_port port;

	/* Setup serial port access */
	memset(&port, 0, sizeof(port));
	port.membase = (void*)UART0_IO_BASE;
	port.irq = UART0_INT;
	port.uartclk = 11059200;
	port.regshift = 0;
	port.iotype = UPIO_MEM;
	port.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	port.line = 0;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 0 failed\n");

	port.membase = (void*)UART1_IO_BASE;
	port.irq = UART1_INT;
	port.line = 1;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 1 failed\n");
}

static void __init kilauea_set_emacdata(void)
{
	struct ocp_def *def;
	struct ocp_func_emac_data *emacdata;

	/* Set mac_addr, phy mode and unsupported phy features for each EMAC */

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 0);
	emacdata = def->additions;
	memcpy(emacdata->mac_addr, __res.bi_enetaddr, 6);
	emacdata->phy_mode = PHY_MODE_RGMII;
	emacdata->phy_feat_exc = SUPPORTED_Autoneg;

	/*
	 * Haleakala (405EXr) only supports one PCIe interface
	 */
	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 1);
	if ((mfspr(SPRN_PVR) & 0x00000004) == 0x00000000) {
		def->additions = 0;
	} else {
		emacdata = def->additions;
		memcpy(emacdata->mac_addr, __res.bi_enet1addr, 6);
		emacdata->phy_mode = PHY_MODE_RGMII;
		emacdata->phy_feat_exc = SUPPORTED_Autoneg;
	}
}

void __init kilauea_setup_arch(void)
{
	kilauea_set_emacdata();

	ppc4xx_setup_arch();

	ibm_ocp_set_emac(0, 0);

	ocp_sys_info.plb_bus_freq = __res.bi_busfreq;

        kilauea_early_serial_map();

#ifdef CONFIG_PCI
	kilauea_setup_hoses();
#endif

	/* Identify the system */
	if ((mfspr(SPRN_PVR) & 0x00000004) == 0x00000000)
		printk("AMCC PowerPC 405EXr Haleakala Platform\n");
	else
		printk("AMCC PowerPC 405EX Kilauea Platform\n");
}

void __init kilauea_map_io(void)
{
	ppc4xx_map_io();
}

void __init platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
			  unsigned long r6, unsigned long r7)
{
	ppc4xx_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = kilauea_setup_arch;
	ppc_md.setup_io_mappings = kilauea_map_io;
}
