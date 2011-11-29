/*
 * arch/ppc/platforms/4xx/sequoia.c
 *
 * Sequoia board specific routines
 *
 * Copyright 2006-2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Based on bamboo.c from Wade Farnsworth <wfarnsworth@mvista.com>
 *	Copyright 2004 MontaVista Software Inc.
 *	Copyright 2006 AMCC
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <linux/irq.h>
#include <linux/root_dev.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/ndfc.h>
#include <linux/mtd/physmap.h>

#include <asm/machdep.h>
#include <asm/ocp.h>
#include <asm/bootinfo.h>
#include <asm/ppc4xx_pic.h>
#include <asm/ppcboot.h>

#include <syslib/gen550.h>
#include <syslib/ibm440gx_common.h>

#if defined(CONFIG_SEQUOIA)
#define BOARDNAME  "440EPx Sequoia"
#elif defined(CONFIG_ROACH)
#define BOARDNAME  "440EPx Roach"
#elif defined(CONFIG_ROACH2)
#define BOARDNAME  "440EPx Roach2"
#else
#define BOARDNAME  "440GRx Rainier"
#endif

extern bd_t __res;

static struct ibm44x_clocks clocks __initdata;

/*
 * Sequoia external IRQ triggering/polarity settings
 */
unsigned char ppc4xx_uic_ext_irq_cfg[] __initdata = {
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index0 - IRQ4: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index1 - IRQ7: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index2 - IRQ8: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index3 - IRQ9: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index4 - IRQ0: Ethernet 0 */
	(IRQ_SENSE_EDGE | IRQ_POLARITY_POSITIVE), /* Index5 - IRQ1: Ethernet 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index6 - IRQ5: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index7 - IRQ6: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index8 - IRQ2: PCI slots */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index9 - IRQ3: STTM alert */
};

/*
 * NOR FLASH configuration (using mtd physmap driver)
 */

/* start will be added dynamically, end is always fixed */
static struct resource sequoia_nor_resource = {
		.flags = IORESOURCE_MEM,
};

#define RW_PART0_OF	0
#define RW_PART0_SZ	0x180000
#define RW_PART1_SZ	0x280000
/* Partition 2 will be autosized dynamically... */
#define RW_PART3_SZ	0x40000
#define RW_PART4_SZ	0x60000

static struct mtd_partition sequoia_nor_parts[] = {
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

static struct physmap_flash_data sequoia_nor_data = {
	.width		= 2,
	.parts		= sequoia_nor_parts,
	.nr_parts	= ARRAY_SIZE(sequoia_nor_parts),
};

static struct platform_device sequoia_nor_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev = {
			.platform_data = &sequoia_nor_data,
		},
	.num_resources	= 1,
	.resource	= &sequoia_nor_resource,
};

/*
 * NAND FLASH configuration (using 440EP(x) NDFC)
 */
static struct resource sequoia_ndfc = {
	.start = (u32)SEQUOIA_NAND_FLASH_REG_ADDR,
	.end = (u32)SEQUOIA_NAND_FLASH_REG_ADDR + SEQUOIA_NAND_FLASH_REG_SIZE - 1,
	.flags = IORESOURCE_MEM,
};

/* todo: add logic to detect booting from NAND (NAND on CS0) */
#define CS_NAND_0	3	/* use chip select 3 for NAND device 0 */

static struct mtd_partition sequoia_nand_parts[] = {
        {
                .name   = "content",
                .offset = 0,
                .size   = MTDPART_SIZ_FULL,
        }
};

struct ndfc_controller_settings sequoia_ndfc_settings = {
	.ccr_settings = (NDFC_CCR_BS(CS_NAND_0) |
			 NDFC_CCR_ARAC1),
	.ndfc_erpn = SEQUOIA_NAND_FLASH_REG_ADDR & 0xf00000000ULL,
};

struct platform_nand_ctrl sequoia_nand_ctrl = {
	.priv = &sequoia_ndfc_settings,
};

static struct platform_device sequoia_ndfc_device = {
	.name = "ndfc-nand",
	.id = 0,
	.dev = {
		.platform_data = &sequoia_nand_ctrl,
	},
	.num_resources = 1,
	.resource = &sequoia_ndfc,
};

static struct ndfc_chip_settings sequoia_chip0_settings = {
	.bank_settings = 0x80002222,
};

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 6, 7},
	.oobfree = {
		 {.offset = 8,
		  .length = 8}}
};

static struct platform_nand_chip sequoia_nand_chip0 = {
	.nr_chips = 1,
	.chip_offset = CS_NAND_0,
	.nr_partitions = ARRAY_SIZE(sequoia_nand_parts),
	.partitions = sequoia_nand_parts,
	.chip_delay = 50,
	.ecclayout = &nand_oob_16,
	.priv = &sequoia_chip0_settings,
};

static struct platform_device sequoia_nand_device = {
	.name = "ndfc-chip",
	.id = 0,
	.num_resources = 0,
	.dev = {
		.platform_data = &sequoia_nand_chip0,
		.parent = &sequoia_ndfc_device.dev,
	}
};

static int sequoia_setup_flash(void)
{
	sequoia_nor_resource.start = __res.bi_flashstart;
	sequoia_nor_resource.end = __res.bi_flashstart +
		__res.bi_flashsize - 1;

	/*
	 * Adjust partition 2 to flash size
	 */
	sequoia_nor_parts[2].size = __res.bi_flashsize -
		RW_PART0_SZ - RW_PART1_SZ - RW_PART3_SZ - RW_PART4_SZ;

	platform_device_register(&sequoia_nor_device);

	/* todo: add logic to detect booting from NAND (NAND on CS0) */

	//platform_device_register(&sequoia_ndfc_device);
	//platform_device_register(&sequoia_nand_device);

	return 0;
}
arch_initcall(sequoia_setup_flash);

/*
 * get size of system memory from Board Info .
 */
unsigned long __init sequoia_find_end_of_memory(void)
{
	/* board info structure defined in /include/asm-ppc/ppcboot.h */
	return  __res.bi_memsize;
}

static void __init sequoia_calibrate_decr(void)
{
	unsigned int freq;

	if (mfspr(SPRN_CCR1) & CCR1_TCS)
		freq = SEQUOIA_TMRCLK;
	else
		freq = clocks.cpu;

	ibm44x_calibrate_decr(freq);

}

static int sequoia_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: AMCC\n");
	seq_printf(m, "machine\t\t: PPC" BOARDNAME "\n");

	return 0;
}

static inline int
sequoia_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	static char pci_irq_table[][4] =
	/*
	 *	PCI IDSEL/INTPIN->INTLINE
	 * 	   A   B   C   D
	 */
	{
		{ 67, 67, 67, 67 },	/* IDSEL 10 - PCI Slot 1, ext. IRQ 2 */
		{ 67, 67, 67, 67 },	/* IDSEL 11 - PCI Slot x, ext. IRQ 2 */
		{ 67, 67, 67, 67 },	/* IDSEL 12 - PCI Slot 0, ext. IRQ 2 */
	};

	const long min_idsel = 10, max_idsel = 12, irqs_per_slot = 4;
	return PCI_IRQ_TABLE_LOOKUP;
}

static void __init sequoia_set_emacdata(void)
{
	struct ocp_def *def;
	struct ocp_func_emac_data *emacdata;

	/* Set mac_addr, phy mode and unsupported phy features for each EMAC */

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 0);
	emacdata = def->additions;
	memcpy(emacdata->mac_addr, __res.bi_enetaddr, 6);
	emacdata->phy_mode = PHY_MODE_RGMII;

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 1);
	emacdata = def->additions;
	memcpy(emacdata->mac_addr, __res.bi_enet1addr, 6);
	emacdata->phy_mode = PHY_MODE_RGMII;

        /*
         * Clear PLB4A0_ACR[WRP] (Write Pipeline Control)
         * This fix will make the MAL burst disabling patch for the Linux
         * EMAC driver obsolete.
         */
	mtdcr(DCRN_PLB4A0_ACR, mfdcr(DCRN_PLB4A0_ACR) & ~(0x80000000 >> 7));
}

static int sequoia_exclude_device(unsigned char bus, unsigned char devfn)
{
	return (bus == 0 && devfn == 0);
}

#define PCI_READW(offset) \
        (readw((void *)((u32)pci_reg_base + offset)))

#define PCI_WRITEW(value, offset) \
	(writew(value, (void *)((u32)pci_reg_base + offset)))

#define PCI_WRITEL(value, offset) \
	(writel(value, (void *)((u32)pci_reg_base + offset)))

#define PCI_CFG_OUT(offset, value) \
	(out_le32 (pci_cfg_base + offset, value))

#define PCI_CFG_IN(offset) \
	(in_le32(pci_cfg_base + offset))

static void __init sequoia_setup_pci(void)
{
	void *pci_reg_base;
	void *pci_cfg_base;
	unsigned long memory_size;

	memory_size = ppc_md.find_end_of_memory();

	pci_reg_base = ioremap64(SEQUOIA_PCIL0_BASE, SEQUOIA_PCIL0_SIZE);
	pci_cfg_base = ioremap64(SEQUOIA_PCI_CFGREGS_BASE, 64);

	PCI_CFG_OUT(SEQUOIA_PCI_CFGA_OFFSET, 0x80000000 | (PCI_COMMAND & 0xfc));
	PCI_CFG_OUT(SEQUOIA_PCI_CFGD_OFFSET,
		    (PCI_CFG_IN(SEQUOIA_PCI_CFGD_OFFSET) |
		     PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));

	/* Disable region first */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM0MA);

	/* PLB starting addr: 0x0000000180000000 */
	PCI_WRITEL(SEQUOIA_PCI_PHY_MEM_BASE, SEQUOIA_PCIL0_PMM0LA);

	/* PCI start addr, 0x80000000 (PCI Address) */
	PCI_WRITEL(SEQUOIA_PCI_MEM_BASE, SEQUOIA_PCIL0_PMM0PCILA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM0PCIHA);

	/* Enable no pre-fetch, enable region */
	PCI_WRITEL(((0xffffffff -
		     (SEQUOIA_PCI_UPPER_MEM - SEQUOIA_PCI_MEM_BASE)) | 0x01),
		   SEQUOIA_PCIL0_PMM0MA);

	/* Disable region one */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1MA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1LA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1PCILA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1PCIHA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1MA);

	/* Disable region two */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2MA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2LA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2PCILA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2PCIHA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2MA);

	/* Now configure the PCI->PLB windows, we only use PTM1
	 *
	 * For Inbound flow, set the window size to all available memory
	 * This is required because if size is smaller,
	 * then Eth/PCI DD would fail as PCI card not able to access
	 * the memory allocated by DD.
	 */

	PCI_WRITEL(0, SEQUOIA_PCIL0_PTM1MS);	/* disabled region 1 */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PTM1LA);	/* begin of address map */

	memory_size = 1 << fls(memory_size - 1);

	/* Size low + Enabled */
	PCI_WRITEL((0xffffffff - (memory_size - 1)) | 0x1, SEQUOIA_PCIL0_PTM1MS);

	eieio();
	iounmap(pci_reg_base);
	iounmap(pci_cfg_base);
}

static void __init sequoia_setup_hose(void)
{
	unsigned int bar_response, bar;
	struct pci_controller *hose;

	sequoia_setup_pci();

	hose = pcibios_alloc_controller();

	if (!hose)
		return;

	hose->first_busno = 0;
	hose->last_busno = 0xff;

	hose->pci_mem_offset = SEQUOIA_PCI_MEM_OFFSET;

	pci_init_resource(&hose->io_resource,
			  SEQUOIA_PCI_LOWER_IO,
			  SEQUOIA_PCI_UPPER_IO,
			  IORESOURCE_IO,
			  "PCI host bridge");

	pci_init_resource(&hose->mem_resources[0],
			  SEQUOIA_PCI_LOWER_MEM,
			  SEQUOIA_PCI_UPPER_MEM,
			  IORESOURCE_MEM,
			  "PCI host bridge");

	ppc_md.pci_exclude_device = sequoia_exclude_device;

	hose->io_space.start = SEQUOIA_PCI_LOWER_IO;
	hose->io_space.end = SEQUOIA_PCI_UPPER_IO;
	hose->mem_space.start = SEQUOIA_PCI_LOWER_MEM;
	hose->mem_space.end = SEQUOIA_PCI_UPPER_MEM;
	isa_io_base =
		(unsigned long)ioremap64(SEQUOIA_PCI_IO_BASE, SEQUOIA_PCI_IO_SIZE);
	hose->io_base_virt = (void *)isa_io_base;

	setup_indirect_pci(hose, PCIX0_CFGA, PCIX0_CFGD);
	hose->set_cfg_type = 1;

	/* Zero config bars */
	for (bar = PCI_BASE_ADDRESS_1; bar <= PCI_BASE_ADDRESS_2; bar += 4) {
		early_write_config_dword(hose, hose->first_busno,
					 PCI_FUNC(hose->first_busno), bar,
					 0x00000000);
		early_read_config_dword(hose, hose->first_busno,
					PCI_FUNC(hose->first_busno), bar,
					&bar_response);
	}

	hose->last_busno = pciauto_bus_scan(hose, hose->first_busno);

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = sequoia_map_irq;
}

static void __init sequoia_early_serial_map(void)
{
	struct uart_port port;

	/* Setup ioremapped serial port access */
	memset(&port, 0, sizeof(port));
	port.membase = ioremap64(PPC440EPX_UART0_ADDR, 8);
	port.irq = UART0_INT;
	port.uartclk = clocks.uart0;
	port.regshift = 0;
	port.iotype = SERIAL_IO_MEM;
	port.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
	port.line = 0;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 0 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/* Configure debug serial access */
	gen550_init(0, &port);
#endif

	port.membase = ioremap64(PPC440EPX_UART1_ADDR, 8);
	port.irq = UART1_INT;
	port.uartclk = clocks.uart1;
	port.line = 1;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 1 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/* Configure debug serial access */
	gen550_init(1, &port);
#endif

	port.membase = ioremap64(PPC440EPX_UART2_ADDR, 8);
	port.irq = UART2_INT;
	port.uartclk = clocks.uart2;
	port.line = 2;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 2 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/* Configure debug serial access */
	gen550_init(2, &port);
#endif

	port.membase = ioremap64(PPC440EPX_UART3_ADDR, 8);
	port.irq = UART3_INT;
	port.uartclk = clocks.uart3;
	port.line = 3;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 3 failed\n");
}

static void __init sequoia_setup_arch(void)
{
	sequoia_set_emacdata();

	/* parm1 = sys clock is OK , parm 2 ser_clock to be checked */
	ibm440gx_get_clocks(&clocks, 33000000, 6 * 1843200);
	ocp_sys_info.opb_bus_freq = clocks.opb;
	ocp_sys_info.plb_bus_freq = clocks.plb;

	/* init to some ~sane value until calibrate_delay() runs */
        loops_per_jiffy = 50000000/HZ;

	/* Setup PCI host bridge */
	//sequoia_setup_hose();

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else {
#ifdef CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_HDA1;
#endif
	}
#endif

	sequoia_early_serial_map();

	/* Identify the system */
	printk("AMCC PowerPC " BOARDNAME " Platform\n");
}

static void __init sequoia_init_irq(void)
{
	ppc4xx_pic_init();
}

void __init platform_init(unsigned long r3, unsigned long r4,
			  unsigned long r5, unsigned long r6, unsigned long r7)
{
	ibm44x_platform_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = sequoia_setup_arch;
	ppc_md.show_cpuinfo = sequoia_show_cpuinfo;
	ppc_md.find_end_of_memory = sequoia_find_end_of_memory;
	ppc_md.get_irq = NULL;		/* Set in ppc4xx_pic_init() */

	ppc_md.calibrate_decr = sequoia_calibrate_decr;
	ppc_md.time_init = NULL;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;

  /*
  ppc_md.restart    = powerpmc250_restart;
  ppc_md.power_off  = powerpmc250_power_off;
  ppc_md.halt   = powerpmc250_halt;
  */


	ppc_md.init_IRQ = sequoia_init_irq;

#ifdef CONFIG_KGDB
	ppc_md.early_serial_map = sequoia_early_serial_map;
#endif
}
