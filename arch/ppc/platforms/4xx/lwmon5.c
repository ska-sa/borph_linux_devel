/*
 * arch/ppc/platforms/4xx/lwmon5.c
 *
 * LWMON5 board specific routines
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Based on sequoia.c from Stefan Roese
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
#include <linux/serial_8250.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/wd.h>
#include <linux/wd_hw.h>
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

#define BOARDNAME  "440EPx LWMON5"

#define GPIO1_BASE		0x1ef600c00ULL

extern bd_t __res;

static struct ibm44x_clocks clocks __initdata;

/*
 * We need to map the GPIO output register very early on bootup.
 * Mapping in wd_lwmon5_init() is too late since the WD needs to
 * get kicked earlier because of the very short timeout. So the
 * GPIO output register gets mapped in the board specific platform
 * code upon bootup.
 */
u32 __iomem *lwmon5_gpio1_or;

/*
 * Lwmon5 external IRQ triggering/polarity settings
 */
unsigned char ppc4xx_uic_ext_irq_cfg[] __initdata = {
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index0 - IRQ4: */
	(IRQ_SENSE_EDGE | IRQ_POLARITY_NEGATIVE),  /* Index1 - IRQ7: Powerfail */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index2 - IRQ8: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index3 - IRQ9: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index4 - IRQ0: Ethernet 0 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index5 - IRQ1: Ethernet 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index6 - IRQ5: */
	(IRQ_SENSE_EDGE | IRQ_POLARITY_NEGATIVE),  /* Index7 - IRQ6: dsPIC-KBD */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index8 - IRQ2: PCI slots */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index9 - IRQ3: STTM alert */
};

/*
 * NOR FLASH configuration (using mtd physmap driver)
 */

/* start will be added dynamically, end is always fixed */
static struct resource lwmon5_nor_resource[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
	}
};

#define RW_PART0_SZ	0x800000
#define RW_PART1_SZ	0x800000
#define RW_PART2_SZ	0x200000
/* Partition 3 will be autosized dynamically... */
#define RW_PART4_SZ	0x1000000
#define RW_PART5_SZ	0x2000000
#define RW_PART6_SZ	0x80000
#define RW_PART7_SZ	0x80000

static struct mtd_partition lwmon5_nor_parts[] = {
	{
		.name = "kernel",
		.offset = 0,
		.size = RW_PART0_SZ,
	},
	{
		.name = "kernel_bak",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART1_SZ,
	},
	{
		.name = "bitmap",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART2_SZ,
	},
	{
		.name = "appdisk",
		.offset = MTDPART_OFS_APPEND,
/*		.size = RW_PART3_SZ */ /* will be adjusted dynamically */
	},
	{
		.name = "uprootfs",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART4_SZ,
	},
	{
		.name = "rootfs",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART5_SZ,
	},
	{
		.name = "env",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART6_SZ,
	},
	{
		.name = "u-boot",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART7_SZ,
	}
};

static struct physmap_flash_data lwmon5_nor_data = {
	.width		= 4,
	.parts		= lwmon5_nor_parts,
	.nr_parts	= ARRAY_SIZE(lwmon5_nor_parts),
};

static struct platform_device lwmon5_nor_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev = {
			.platform_data = &lwmon5_nor_data,
		},
	.num_resources	= ARRAY_SIZE(lwmon5_nor_resource),
	.resource	= lwmon5_nor_resource,
};

static int lwmon5_setup_flash(void)
{
	lwmon5_nor_resource[0].start = __res.bi_flashstart;
	lwmon5_nor_resource[1].start = __res.bi_flashstart +
		(__res.bi_flashsize >> 1);
	lwmon5_nor_resource[0].end = lwmon5_nor_resource[1].start - 1;
	lwmon5_nor_resource[1].end = 0xffffffff;

	/*
	 * Adjust partition 3 to flash size
	 */
	lwmon5_nor_parts[3].size = __res.bi_flashsize
		- RW_PART0_SZ - RW_PART1_SZ - RW_PART2_SZ - RW_PART4_SZ
		- RW_PART5_SZ - RW_PART6_SZ - RW_PART7_SZ;

	platform_device_register(&lwmon5_nor_device);

	return 0;
}
arch_initcall(lwmon5_setup_flash);

/*
 * get size of system memory from Board Info .
 */
unsigned long __init lwmon5_find_end_of_memory(void)
{
	/* board info structure defined in /include/asm-ppc/ppcboot.h */
	return  __res.bi_memsize;
}

static void __init lwmon5_calibrate_decr(void)
{
	unsigned int freq;

	if (mfspr(SPRN_CCR1) & CCR1_TCS)
		freq = LWMON5_TMRCLK;
	else
		freq = clocks.cpu;

	ibm44x_calibrate_decr(freq);

}

static int lwmon5_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: AMCC\n");
	seq_printf(m, "machine\t\t: PPC" BOARDNAME "\n");

	return 0;
}

static inline int
lwmon5_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
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

static void __init lwmon5_set_emacdata(void)
{
	struct ocp_def *def;
	struct ocp_func_emac_data *emacdata;

	/* Set mac_addr, phy mode and unsupported phy features for each EMAC */

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 0);
	emacdata = def->additions;
	memcpy(emacdata->mac_addr, __res.bi_enetaddr, 6);
	emacdata->phy_mode = PHY_MODE_SMII;
	emacdata->phy_map = 0x00000007;	/* Skip 0, 1, 2 */

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 1);
	emacdata = def->additions;
	memcpy(emacdata->mac_addr, __res.bi_enet1addr, 6);
	emacdata->phy_mode = PHY_MODE_SMII;
	emacdata->phy_map = 0x00000001;	/* Skip 0 */

        /*
         * Clear PLB4A0_ACR[WRP] (Write Pipeline Control)
         * This fix will make the MAL burst disabling patch for the Linux
         * EMAC driver obsolete.
         */
	mtdcr(DCRN_PLB4A0_ACR, mfdcr(DCRN_PLB4A0_ACR) & ~(0x80000000 >> 7));
}

static int lwmon5_exclude_device(unsigned char bus, unsigned char devfn)
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

static void __init lwmon5_setup_pci(void)
{
	void *pci_reg_base;
	void *pci_cfg_base;
	unsigned long memory_size;

	memory_size = ppc_md.find_end_of_memory();

	pci_reg_base = ioremap64(LWMON5_PCIL0_BASE, LWMON5_PCIL0_SIZE);
	pci_cfg_base = ioremap64(LWMON5_PCI_CFGREGS_BASE, 64);

	PCI_CFG_OUT(LWMON5_PCI_CFGA_OFFSET, 0x80000000 | (PCI_COMMAND & 0xfc));
	PCI_CFG_OUT(LWMON5_PCI_CFGD_OFFSET,
		    (PCI_CFG_IN(LWMON5_PCI_CFGD_OFFSET) |
		     PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));

	/* Disable region first */
	PCI_WRITEL(0, LWMON5_PCIL0_PMM0MA);

	/* PLB starting addr: 0x0000000180000000 */
	PCI_WRITEL(LWMON5_PCI_PHY_MEM_BASE, LWMON5_PCIL0_PMM0LA);

	/* PCI start addr, 0x80000000 (PCI Address) */
	PCI_WRITEL(LWMON5_PCI_MEM_BASE, LWMON5_PCIL0_PMM0PCILA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM0PCIHA);

	/* Enable no pre-fetch, enable region */
	PCI_WRITEL(((0xffffffff -
		     (LWMON5_PCI_UPPER_MEM - LWMON5_PCI_MEM_BASE)) | 0x01),
		   LWMON5_PCIL0_PMM0MA);

	/* Disable region one */
	PCI_WRITEL(0, LWMON5_PCIL0_PMM1MA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM1LA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM1PCILA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM1PCIHA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM1MA);

	/* Disable region two */
	PCI_WRITEL(0, LWMON5_PCIL0_PMM2MA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM2LA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM2PCILA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM2PCIHA);
	PCI_WRITEL(0, LWMON5_PCIL0_PMM2MA);

	/* Now configure the PCI->PLB windows, we only use PTM1
	 *
	 * For Inbound flow, set the window size to all available memory
	 * This is required because if size is smaller,
	 * then Eth/PCI DD would fail as PCI card not able to access
	 * the memory allocated by DD.
	 */

	PCI_WRITEL(0, LWMON5_PCIL0_PTM1MS);	/* disabled region 1 */
	PCI_WRITEL(0, LWMON5_PCIL0_PTM1LA);	/* begin of address map */

	memory_size = 1 << fls(memory_size - 1);

	/* Size low + Enabled */
	PCI_WRITEL((0xffffffff - (memory_size - 1)) | 0x1, LWMON5_PCIL0_PTM1MS);

	eieio();
	iounmap(pci_reg_base);
	iounmap(pci_cfg_base);
}

static void __init lwmon5_setup_hose(void)
{
	unsigned int bar_response, bar;
	struct pci_controller *hose;

	lwmon5_setup_pci();

	hose = pcibios_alloc_controller();

	if (!hose)
		return;

	hose->first_busno = 0;
	hose->last_busno = 0xff;

	hose->pci_mem_offset = LWMON5_PCI_MEM_OFFSET;

	pci_init_resource(&hose->io_resource,
			  LWMON5_PCI_LOWER_IO,
			  LWMON5_PCI_UPPER_IO,
			  IORESOURCE_IO,
			  "PCI host bridge");

	pci_init_resource(&hose->mem_resources[0],
			  LWMON5_PCI_LOWER_MEM,
			  LWMON5_PCI_UPPER_MEM,
			  IORESOURCE_MEM,
			  "PCI host bridge");

	ppc_md.pci_exclude_device = lwmon5_exclude_device;

	hose->io_space.start = LWMON5_PCI_LOWER_IO;
	hose->io_space.end = LWMON5_PCI_UPPER_IO;
	hose->mem_space.start = LWMON5_PCI_LOWER_MEM;
	hose->mem_space.end = LWMON5_PCI_UPPER_MEM;
	isa_io_base =
		(unsigned long)ioremap64(LWMON5_PCI_IO_BASE, LWMON5_PCI_IO_SIZE);
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
	ppc_md.pci_map_irq = lwmon5_map_irq;
}

static void __init lwmon5_early_serial_map(void)
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

static void __init lwmon5_setup_arch(void)
{
#if defined(CONFIG_WD) && defined(CONFIG_WD_LWMON5)
	lwmon5_gpio1_or = ioremap64(GPIO1_BASE, 4);

        /* Init watchdog functions structure */
        wd_hw_functions.wd_init = wd_lwmon5_init;
        wd_hw_functions.wd_kick = wd_lwmon5_kick;
        wd_hw_functions.wd_delete = wd_lwmon5_delete;
        wd_hw_functions.wd_machine_restart = wd_lwmon5_machine_restart;
#endif /* CONFIG_WD && CONFIG_WD_LWMON5 */

	lwmon5_set_emacdata();

	/* parm1 = sys clock is OK , parm 2 ser_clock to be checked */
	ibm440gx_get_clocks(&clocks, LWMON5_SYSCLK, 6 * 1843200);
	ocp_sys_info.opb_bus_freq = clocks.opb;
	ocp_sys_info.plb_bus_freq = clocks.plb;

	/* init to some ~sane value until calibrate_delay() runs */
        loops_per_jiffy = 50000000/HZ;

	/* Setup PCI host bridge */
	lwmon5_setup_hose();

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

	lwmon5_early_serial_map();

	/* Identify the system */
	printk("AMCC PowerPC " BOARDNAME " Platform\n");
}

static void __init lwmon5_init_irq(void)
{
	ppc4xx_pic_init();
}

static void lwmon5_restart(char *cmd)
{
	volatile void * gpio_base;

	/* GPIO58_EPX is the reset pin */
	gpio_base = ioremap64(GPIO_BASE, GPIO_SIZE);
	out_be32(gpio_base + GPIO1_OR_OFFS,
		 in_be32(gpio_base + GPIO1_OR_OFFS) | (0x80000000 >> 26));
	iounmap(gpio_base);
}

void __init platform_init(unsigned long r3, unsigned long r4,
			  unsigned long r5, unsigned long r6, unsigned long r7)
{
	ibm44x_platform_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = lwmon5_setup_arch;
	ppc_md.show_cpuinfo = lwmon5_show_cpuinfo;
	ppc_md.find_end_of_memory = lwmon5_find_end_of_memory;
	ppc_md.get_irq = NULL;		/* Set in ppc4xx_pic_init() */

	ppc_md.calibrate_decr = lwmon5_calibrate_decr;
	ppc_md.time_init = NULL;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;

	ppc_md.init_IRQ = lwmon5_init_irq;

#ifdef CONFIG_KGDB
	ppc_md.early_serial_map = lwmon5_early_serial_map;
#endif
	ppc_md.restart = lwmon5_restart;
}
