/*
 * Board setup routines for the  P3M750 Prodrive Board.
 *
 * Author: Wolfgang Denk <wd@denx.de>
 *
 * Based on code done by Lee Nicks <allinux@gmail.com>
 * Based on code done by Rabeeh Khoury - rabeeh@galileo.co.il
 * Based on code done by - Mark A. Greer <mgreer@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/console.h>
#include <linux/initrd.h>
#include <linux/root_dev.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/bootmem.h>
#include <linux/mtd/physmap.h>
#include <linux/mv643xx.h>
#include <linux/platform_device.h>
#ifdef CONFIG_BOOTIMG
#include <linux/bootimg.h>
#endif
#include <asm/page.h>
#include <asm/time.h>
#include <asm/smp.h>
#include <asm/todc.h>
#include <asm/bootinfo.h>
#include <asm/ppcboot.h>
#include <asm/mv64x60.h>
#include <asm/machdep.h>
#include <platforms/p3m750.h>

#define BOARD_VENDOR    "Prodrive"
#define BOARD_MACHINE   "P3M-750"

static struct		mv64x60_handle bh;
static void __iomem	*sram_base;

static u32		p3m750_flash_size;

static u32		p3m750_bus_frequency;

unsigned char	__res[sizeof(bd_t)];

TODC_ALLOC();

static int __init
p3m750_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	struct pci_controller	*hose = pci_bus_to_hose(dev->bus->number);

	if (hose->index == 0) {
		static char pci_irq_table[][4] =
			/*
			 *	PCI IDSEL/INTPIN->INTLINE
			 * 	   A   B   C   D
			 */
			{
				{P3M750_PCI_INTA, P3M750_PCI_INTB, 0, 0}, /* Peripheral/Oxford */
				{P3M750_PCI_INTC, 0, 0, 0}, /* PLX */
				{P3M750_PCI_INTA, P3M750_PCI_INTB, P3M750_PCI_INTC, P3M750_PCI_INTD}, /* Universe */
				{P3M750_PCI_INTB, P3M750_PCI_INTC, P3M750_PCI_INTD, P3M750_PCI_INTA}, /* PrPMC site 2 secondary */
				{P3M750_PCI_INTB, P3M750_PCI_INTC, P3M750_PCI_INTD, P3M750_PCI_INTA}, /* PrPMC site 2 primary */
				{0, 0, 0, 0}, /* PrPMC site 1 secondary */
				{0, 0, 0, 0}, /* PrPMC site 1 primary */
			};

		const long min_idsel = 9, max_idsel = 15, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}

	return 0;
}

static void __init
p3m750_setup_bridge(void)
{
	struct mv64x60_setup_info si;
	int i;

	memset(&si, 0, sizeof(si));

	si.phys_reg_base = CONFIG_MV64X60_NEW_BASE;

#if defined(CONFIG_PCI)
        si.pci_0.enable_bus = 1;
        si.pci_0.pci_io.cpu_base = P3M750_PCI0_IO_START_PROC_ADDR;
        si.pci_0.pci_io.pci_base_hi = 0;
        si.pci_0.pci_io.pci_base_lo = P3M750_PCI0_IO_START_PCI_ADDR;
        si.pci_0.pci_io.size = P3M750_PCI0_IO_SIZE;
        si.pci_0.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;
        si.pci_0.pci_mem[0].cpu_base = P3M750_PCI0_MEM_START_PROC_ADDR;
        si.pci_0.pci_mem[0].pci_base_hi = P3M750_PCI0_MEM_START_PCI_HI_ADDR;
        si.pci_0.pci_mem[0].pci_base_lo = P3M750_PCI0_MEM_START_PCI_LO_ADDR;
        si.pci_0.pci_mem[0].size = P3M750_PCI0_MEM_SIZE;
        si.pci_0.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;
        si.pci_0.pci_cmd_bits = 0;
        si.pci_0.latency_timer = 0x80;

	si.pci_1.enable_bus = 1;
	si.pci_1.pci_io.cpu_base = P3M750_PCI1_IO_START_PROC_ADDR;
	si.pci_1.pci_io.pci_base_hi = 0;
	si.pci_1.pci_io.pci_base_lo = P3M750_PCI1_IO_START_PCI_ADDR;
	si.pci_1.pci_io.size = P3M750_PCI1_IO_SIZE;
	si.pci_1.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_1.pci_mem[0].cpu_base = P3M750_PCI1_MEM_START_PROC_ADDR;
	si.pci_1.pci_mem[0].pci_base_hi = P3M750_PCI1_MEM_START_PCI_HI_ADDR;
	si.pci_1.pci_mem[0].pci_base_lo = P3M750_PCI1_MEM_START_PCI_LO_ADDR;
	si.pci_1.pci_mem[0].size = P3M750_PCI1_MEM_SIZE;
	si.pci_1.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_1.pci_cmd_bits = 0;
	si.pci_1.latency_timer = 0x80;
#else
	si.pci_0.enable_bus = 0;
	si.pci_1.enable_bus = 0;
#endif

	for (i = 0; i < MV64x60_CPU2MEM_WINDOWS; i++) {
#if defined(CONFIG_NOT_COHERENT_CACHE)
		si.cpu_prot_options[i] = 0;
		si.enet_options[i] = MV64360_ENET2MEM_SNOOP_NONE;
		si.mpsc_options[i] = MV64360_MPSC2MEM_SNOOP_NONE;
		si.idma_options[i] = MV64360_IDMA2MEM_SNOOP_NONE;

		si.pci_0.acc_cntl_options[i] = si.pci_1.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_NONE |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES;
#else
		si.cpu_prot_options[i] = 0;
		si.enet_options[i] = MV64360_ENET2MEM_SNOOP_NONE; /* errata */
		si.mpsc_options[i] = MV64360_MPSC2MEM_SNOOP_NONE; /* errata */
		si.idma_options[i] = MV64360_IDMA2MEM_SNOOP_NONE; /* errata */

		si.pci_1.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_WB |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_32_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_32_BYTES;
#endif
	}

	if (mv64x60_init(&bh, &si))
		printk(KERN_WARNING "Bridge initialization failed.\n");

#if defined(CONFIG_PCI)
	pci_dram_offset = 0; /* sys mem at same addr on PCI & cpu bus */
	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = p3m750_map_irq;
	ppc_md.pci_exclude_device = mv64x60_pci_exclude_device;

	mv64x60_set_bus(&bh, 1, 0);
	bh.hose_b->first_busno = 0;
	bh.hose_b->last_busno = 0xff;
#endif
}

void __init
p3m750_setup_peripherals(void)
{
	u32 base;
	u32 data;

	/* Set up window for boot CS */
	mv64x60_set_32bit_window(&bh, MV64x60_CPU2BOOT_WIN,
		 P3M750_BOOT_WINDOW_BASE, P3M750_BOOT_WINDOW_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2BOOT_WIN);

	mv64x60_get_32bit_window(&bh, MV64x60_CPU2BOOT_WIN, &base,
		&p3m750_flash_size);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2SRAM_WIN,
		P3M750_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2SRAM_WIN);
	sram_base = ioremap(P3M750_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE);

#if defined(CONFIG_NOT_COHERENT_CACHE)
	mv64x60_write(&bh, MV64360_SRAM_CONFIG, 0x00160000);
#else
	mv64x60_write(&bh, MV64360_SRAM_CONFIG, 0x001600b2);
#endif

	/*
	 * Setting the SRAM to 0. Note that this generates parity errors on
	 * internal data path in SRAM since it's first time accessing it
	 * while after reset it's not configured.
	 */
	memset(sram_base, 0, MV64360_SRAM_SIZE);

	/* setting pci interrupts */
	/* MPPs are GPIO */
	data = mv64x60_read(&bh, MV64x60_MPP_CNTL_3);
	data &= ~(0x00f0f000);
	mv64x60_write(&bh, MV64x60_MPP_CNTL_3, data);
	data = mv64x60_read(&bh, MV64x60_MPP_CNTL_2);
	data &= ~(0x000000ff);
	mv64x60_write(&bh, MV64x60_MPP_CNTL_2, data);

#define GPP_EXTERNAL_INTERRUPTS		 \
		((1 << P3M750_PCI_INTA) |\
		 (1 << P3M750_PCI_INTB) |\
		 (1 << P3M750_PCI_INTC) |\
		 (1 << P3M750_PCI_INTD))

	/* PCI interrupts are inputs */
	mv64x60_clr_bits(&bh, MV64x60_GPP_IO_CNTL, GPP_EXTERNAL_INTERRUPTS);
	/* PCI interrupts are active low */
	mv64x60_set_bits(&bh, MV64x60_GPP_LEVEL_CNTL, GPP_EXTERNAL_INTERRUPTS);

	/* Clear any pending interrupts for these inputs and enable them. */
	mv64x60_write(&bh, MV64x60_GPP_INTR_CAUSE, ~GPP_EXTERNAL_INTERRUPTS);
	mv64x60_set_bits(&bh, MV64x60_GPP_INTR_MASK, GPP_EXTERNAL_INTERRUPTS);
}

static void __init
p3m750_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("p3m750_setup_arch: enter", 0);

	set_tb(0, 0);

#if defined(CONFIG_BLK_DEV_INITRD)
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif
#if defined(CONFIG_ROOT_NFS)
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_SDA2;
#endif

	/*
	 * Set up the L2CR register.
	 */
	_set_L2CR(L2CR_L2E | L2CR_L2PE);

	if (ppc_md.progress)
		ppc_md.progress("p3m750_setup_arch: calling setup_bridge", 0);

	p3m750_setup_bridge();
	p3m750_setup_peripherals();
	p3m750_bus_frequency = p3m750_bus_freq();

	printk(KERN_INFO "%s %s port (C) 2006 DENX Engeneering "
		"(wd@denx.de)\n", BOARD_VENDOR, BOARD_MACHINE);

	if (ppc_md.progress)
		ppc_md.progress("p3m750_setup_arch: exit", 0);
}

/* Platform device data fixup routines. */
#if defined(CONFIG_SERIAL_MPSC)
static void __init
p3m750_fixup_mpsc_pdata(struct platform_device *pdev)
{
	struct mpsc_pdata *pdata;

	pdata = (struct mpsc_pdata *)pdev->dev.platform_data;

	pdata->max_idle = 40;
	pdata->default_baud = P3M750_DEFAULT_BAUD;
	pdata->brg_clk_src = P3M750_MPSC_CLK_SRC;
	/*
	 * TCLK (not SysCLk) is routed to BRG, then to the MPSC.  On most parts,
	 * TCLK == SysCLK but on 64460, they are separate pins.
	 * SysCLK can go up to 200 MHz but TCLK can only go up to 133 MHz.
	 */
	pdata->brg_clk_freq = min(p3m750_bus_frequency, MV64x60_TCLK_FREQ_MAX);
}
#endif

#if defined(CONFIG_MV643XX_ETH)
static void __init
p3m750_fixup_eth_pdata(struct platform_device *pdev)
{
	struct mv643xx_eth_platform_data *eth_pd;
	static u16 phy_addr[] = {
		P3M750_ETH0_PHY_ADDR,
		P3M750_ETH1_PHY_ADDR,
		P3M750_ETH2_PHY_ADDR,
	};

	eth_pd = pdev->dev.platform_data;
	eth_pd->force_phy_addr = 1;
	eth_pd->phy_addr = phy_addr[pdev->id];
	eth_pd->tx_queue_size = P3M750_ETH_TX_QUEUE_SIZE;
	eth_pd->rx_queue_size = P3M750_ETH_RX_QUEUE_SIZE;
}
#endif

static int
p3m750_platform_notify(struct device *dev)
{
	static struct {
		char	*bus_id;
		void	((*rtn)(struct platform_device *pdev));
	} dev_map[] = {
#if defined(CONFIG_SERIAL_MPSC)
		{ MPSC_CTLR_NAME ".0", p3m750_fixup_mpsc_pdata },
		{ MPSC_CTLR_NAME ".1", p3m750_fixup_mpsc_pdata },
#endif
#if defined(CONFIG_MV643XX_ETH)
		{ MV643XX_ETH_NAME ".0", p3m750_fixup_eth_pdata },
		{ MV643XX_ETH_NAME ".1", p3m750_fixup_eth_pdata },
		{ MV643XX_ETH_NAME ".2", p3m750_fixup_eth_pdata },
#endif
	};
	struct platform_device	*pdev;
	int	i;

	if (dev && dev->bus_id)
		for (i=0; i<ARRAY_SIZE(dev_map); i++)
			if (!strncmp(dev->bus_id, dev_map[i].bus_id,
				BUS_ID_SIZE)) {

				pdev = container_of(dev,
					struct platform_device, dev);
				dev_map[i].rtn(pdev);
			}

	return 0;
}

#if defined(CONFIG_MTD_PHYSMAP)

#if !defined(MB)
#define MB	(1 << 20)
#endif

/*
 * MTD Layout.
 *
 * FLASH Amount:	0xff800000 - 0xffffffff
 * -------------	-----------------------
 * Reserved1:		0xff800000 - 0xff0Bffff
 * Reserved2:		0xffc00000 - 0xffdfffff
 * U-Boot:		0xffe00000 - 0xffffffff
 */
static int __init
p3m750_setup_mtd(void)
{
	u32	size;
	int	ptbl_entries;
	static struct mtd_partition	*ptbl;

	size = p3m750_flash_size;
	if (!size)
		return -ENOMEM;

	ptbl_entries = 3;

	if ((ptbl = kmalloc(ptbl_entries * sizeof(struct mtd_partition),
		GFP_KERNEL)) == NULL) {

		printk(KERN_WARNING "Can't alloc MTD partition table\n");
		return -ENOMEM;
	}
	memset(ptbl, 0, ptbl_entries * sizeof(struct mtd_partition));

	ptbl[0].name = "reserved1";
	ptbl[0].offset = 0;
	ptbl[0].size = P3M750_MTD_RESERVED1_SIZE;

	ptbl[1].name = "reserved2";
	ptbl[1].offset = MTDPART_OFS_APPEND;
	ptbl[1].size = P3M750_MTD_RESERVED2_SIZE;

	ptbl[2].name = "u-boot";
	ptbl[2].offset = MTDPART_OFS_APPEND;
	ptbl[2].size = P3M750_MTD_UBOOT_SIZE;

	physmap_set_partitions(ptbl, ptbl_entries);

	return 0;
}

arch_initcall(p3m750_setup_mtd);
#endif

static void
p3m750_restart(char *cmd)
{
	int i = 0xffffffff, data;

	local_irq_disable();

	/*
	 * Set MPP[30] to reset the module
	 */

	/* Configure MPP30 as GPP */
	data = mv64x60_read(&bh, MV64x60_MPP_CNTL_3);
	data &= ~(0x0f000000);
	mv64x60_write(&bh, MV64x60_MPP_CNTL_3, data);

	/* Enable pin GPP30 for output */
	data = mv64x60_read(&bh, MV64x60_GPP_IO_CNTL);
	data |= (1 << P3M750_RST_PIN);
	mv64x60_write(&bh, MV64x60_GPP_IO_CNTL, data);

	/* Toggle GPP30 pin to reset the board */
	mv64x60_write(&bh, MV64x60_GPP_VALUE_CLR, 1 << P3M750_RST_PIN);
	mv64x60_write(&bh, MV64x60_GPP_VALUE_SET, 1 << P3M750_RST_PIN);

	while (i-- > 0) ;
	panic("restart failed\n");
}

static void
p3m750_halt(void)
{
	local_irq_disable();

	while (1) ;
	/* NOTREACHED */
}

static void
p3m750_power_off(void)
{
	p3m750_halt();
	/* NOTREACHED */
}

static int
p3m750_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: " BOARD_VENDOR "\n");
	seq_printf(m, "machine\t\t: " BOARD_MACHINE "\n");
	seq_printf(m, "bus speed\t: %dMHz\n", p3m750_bus_frequency/1000/1000);

	return 0;
}

static void __init
p3m750_calibrate_decr(void)
{
	u32 freq;

	freq = p3m750_bus_frequency / 4;

	printk(KERN_INFO "time_init: decrementer frequency = %lu.%.6lu MHz\n",
	       (long)freq / 1000000, (long)freq % 1000000);

	tb_ticks_per_jiffy = freq / HZ;
	tb_to_us = mulhwu_scale_factor(freq, 1000000);
}

unsigned long __init
p3m750_find_end_of_memory(void)
{
	return mv64x60_get_mem_size(CONFIG_MV64X60_NEW_BASE,
		MV64x60_TYPE_MV64360);
}

static inline void
p3m750_set_bat(void)
{
	mb();
	mtspr(SPRN_DBAT2U, 0xf0001ffe);
	mtspr(SPRN_DBAT2L, 0xf000002a);
	mb();
}

#if defined(CONFIG_SERIAL_TEXT_DEBUG) && defined(CONFIG_SERIAL_MPSC_CONSOLE)
static void __init
p3m750_map_io(void)
{
	io_block_mapping(CONFIG_MV64X60_NEW_BASE, \
			 CONFIG_MV64X60_NEW_BASE, \
			 0x00020000, _PAGE_IO);
}
#endif

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

	/* ASSUMPTION:  If both r3 (bd_t pointer) and r6 (cmdline pointer)
	 * are non-zero, then we should use the board info from the bd_t
	 * structure and the cmdline pointed to by r6 instead of the
	 * information from birecs, if any.  Otherwise, use the information
	 * from birecs as discovered by the preceeding call to
	 * parse_bootinfo().  This rule should work with both PPCBoot, which
	 * uses a bd_t board info structure, and the kernel boot wrapper,
	 * which uses birecs.
	 */
	if (r3 && r6) {
		/* copy board info structure */
		memcpy( (void *)__res,(void *)(r3+KERNELBASE), sizeof(bd_t) );
		/* copy command line */
		*(char *)(r7+KERNELBASE) = 0;
		strcpy(cmd_line, (char *)(r6+KERNELBASE));
	}
#if defined(CONFIG_ISA)
	isa_mem_base = 0;
#endif

	ppc_md.setup_arch = p3m750_setup_arch;
	ppc_md.show_cpuinfo = p3m750_show_cpuinfo;
	ppc_md.init_IRQ = mv64360_init_irq;
	ppc_md.get_irq = mv64360_get_irq;
	ppc_md.restart = p3m750_restart;
	ppc_md.power_off = p3m750_power_off;
	ppc_md.halt = p3m750_halt;
	ppc_md.find_end_of_memory = p3m750_find_end_of_memory;
	ppc_md.init = NULL;

	ppc_md.time_init = NULL;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;
	ppc_md.nvram_read_val = NULL;
	ppc_md.nvram_write_val = NULL;
	ppc_md.calibrate_decr = p3m750_calibrate_decr;

#if defined(CONFIG_SERIAL_TEXT_DEBUG) && defined(CONFIG_SERIAL_MPSC_CONSOLE)
	ppc_md.setup_io_mappings = p3m750_map_io;
	ppc_md.progress = mv64x60_mpsc_progress;
	mv64x60_progress_init(CONFIG_MV64X60_NEW_BASE);
#endif

#if defined(CONFIG_SERIAL_MPSC) || defined(CONFIG_MV643XX_ETH)
	platform_notify = p3m750_platform_notify;
#endif

	p3m750_set_bat(); /* Need for p3m750_find_end_of_memory and progress */
}

