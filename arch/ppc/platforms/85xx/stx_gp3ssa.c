/*
 * STx GP3 SSA board specific routines
 *
 * Copied from mpc8560_ads.c
 * Copyright 2002, 2003 Motorola Inc.
 *
 * Dan Malek <dan@embeddededge.com>
 * Copyright 2004 Embedded Edge, LLC
 *
 * Ported to 2.6, Matt Porter <mporter@kernel.crashing.org>
 * Copyright 2004-2005 MontaVista Software, Inc.
 *
 * GP3 SSA Updates.  I just didn't want to fill stx_gp3.c with more #ifdefs.
 * Copyright 2006 Embedded Alley Solutions, Inc.
 *	Dan Malek <dan@embeddedalley.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/root_dev.h>
#include <linux/seq_file.h>
#include <linux/serial.h>
#include <linux/ide.h>
#include <linux/initrd.h>
#include <linux/module.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/rio.h>
#include <linux/i2c.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/open_pic.h>
#include <asm/bootinfo.h>
#include <asm/pci-bridge.h>
#include <asm/mpc85xx.h>
#include <asm/irq.h>
#include <asm/immap_85xx.h>
#include <asm/cpm2.h>
#include <asm/mpc85xx.h>
#include <asm/ppc_sys.h>

#include <syslib/cpm2_pic.h>
#include <syslib/ppc85xx_common.h>
#include <syslib/ppc85xx_rio.h>

unsigned char __res[sizeof(bd_t)];

/*
 * I2C RTC
 */
static struct i2c_board_info __initdata gp3ssa_i2c_devices[] = {
	{
		I2C_BOARD_INFO("rtc-ds1307", 0x68),
		.type = "ds1339",
	},
};

#ifndef CONFIG_PCI
unsigned long isa_io_base = 0;
unsigned long isa_mem_base = 0;
unsigned long pci_dram_offset = 0;
#endif

/* PCI interrupt controller */
#define P1IRQA		MPC85xx_IRQ_EXT2
#define P1IRQB		MPC85xx_IRQ_EXT3
#define P1IRQC		MPC85xx_IRQ_EXT4
#define P2IRQA		MPC85xx_IRQ_EXT5
#define P2IRQB		MPC85xx_IRQ_EXT9
#define P2IRQC		MPC85xx_IRQ_EXT11

/* Internal interrupts are all Level Sensitive, and Positive Polarity */
static u8 gp3ssa_openpic_initsenses[] __initdata = {
	MPC85XX_INTERNAL_IRQ_SENSES,
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 0: TSEC1 PHY */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 1: TSEC2 PHY */
#if defined(CONFIG_PCI)
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 2: PCI INTA */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 3: PCI INTB */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 4: PCI INTC */
#else
	0x0,				/* External  2: */
	0x0,				/* External  3: */
	0x0,				/* External  4: */
#endif
#if defined(CONFIG_PCI) && defined(CONFIG_85xx_PCI2)
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 5: PCI2 INTA */
#else
	0x0,				/* External  5: */
#endif
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 6: Localbus A */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 7: Localbus B */
		/* External Interrupt from the CPM expansion connector */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 8 */
#if defined(CONFIG_PCI) && defined(CONFIG_85xx_PCI2)
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 9: PCI2 INTB */
#else
	0x0,				/* External  9: */
#endif
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 10: FCC PHY */
#if defined(CONFIG_PCI) && defined(CONFIG_85xx_PCI2)
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* Ext 11: PCI2 INTC */
#else
	0x0,				/* External 11: */
#endif
};

static int gp3ssa_setup_devices(void)
{
	/* I2C devices */
	i2c_register_board_info(1, gp3ssa_i2c_devices,
				ARRAY_SIZE(gp3ssa_i2c_devices));

	return 0;
}
arch_initcall(gp3ssa_setup_devices);


/*
 * Setup the architecture
 */
static void __init
gp3ssa_setup_arch(void)
{
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;
	struct gianfar_platform_data *pdata;
	struct gianfar_mdio_data *mdata;

#ifdef CONFIG_CPM2
	cpm2_reset();
#endif

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	if (ppc_md.progress)
		ppc_md.progress("gp3ssa_setup_arch()", 0);

	/* Set loops_per_jiffy to a half-way reasonable value,
	   for use until calibrate_delay gets called. */
	loops_per_jiffy = freq / HZ;

#ifdef CONFIG_PCI
	/* setup PCI host bridges */
	mpc85xx_setup_hose();
#endif

#ifdef CONFIG_SERIAL_8250
        mpc85xx_early_serial_map();
#endif

#ifdef CONFIG_SERIAL_TEXT_DEBUG
	/* Invalidate the entry we stole earlier the serial ports
	 * should be properly mapped */
	invalidate_tlbcam_entry(num_tlbcam_entries - 1);
#endif

	/* setup the board related info for the MDIO bus */
	mdata = (struct gianfar_mdio_data *) ppc_sys_get_pdata(MPC85xx_MDIO);

	mdata->irq[1] = MPC85xx_IRQ_EXT10;	/* FCC PHY */
	mdata->irq[2] = MPC85xx_IRQ_EXT0;
	mdata->irq[4] = MPC85xx_IRQ_EXT1;
	mdata->irq[31] = -1;

	/* setup the board related information for the enet controllers */
	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC85xx_TSEC1);
	if (pdata) {
	/*	pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR; */
		pdata->bus_id = 0;
		pdata->phy_id = 2;
		memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC85xx_TSEC2);
	if (pdata) {
	/*	pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR; */
		pdata->bus_id = 0;
		pdata->phy_id = 4;
		memcpy(pdata->mac_addr, binfo->bi_enet1addr, 6);
	}

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif
#ifdef	CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_HDA1;
#endif

	printk ("bi_immr_base = %8.8lx\n", binfo->bi_immr_base);
}

#ifdef CONFIG_CPM2
static irqreturn_t cpm2_cascade(int irq, void *dev_id)
{
	while ((irq = cpm2_get_irq()) >= 0)
		__do_IRQ(irq);

	return IRQ_HANDLED;
}

static struct irqaction cpm2_irqaction = {
	.handler	= cpm2_cascade,
	.flags		= IRQF_DISABLED,
	.mask		= CPU_MASK_NONE,
	.name		= "cpm2_cascade",
};
#endif

static void __init
gp3ssa_init_IRQ(void)
{
	bd_t *binfo = (bd_t *) __res;

	/*
	 * Setup OpenPIC
	 */

	/* Determine the Physical Address of the OpenPIC regs */
	phys_addr_t OpenPIC_PAddr =
	    binfo->bi_immr_base + MPC85xx_OPENPIC_OFFSET;
	OpenPIC_Addr = ioremap(OpenPIC_PAddr, MPC85xx_OPENPIC_SIZE);
	OpenPIC_InitSenses = gp3ssa_openpic_initsenses;
	OpenPIC_NumInitSenses = sizeof (gp3ssa_openpic_initsenses);

	/* Skip reserved space and internal sources */
	openpic_set_sources(0, 32, OpenPIC_Addr + 0x10200);

	/* Map PIC IRQs 0-11 */
	openpic_set_sources(48, 12, OpenPIC_Addr + 0x10000);

	/*
	 * Let openpic interrupts starting from an offset, to
	 * leave space for cascading interrupts underneath.
	 */
	openpic_init(MPC85xx_OPENPIC_IRQ_OFFSET);

#ifdef CONFIG_CPM2
	/* Setup CPM2 PIC */
        cpm2_init_IRQ();

	setup_irq(MPC85xx_IRQ_CPM, &cpm2_irqaction);
#endif

	return;
}

static int
gp3ssa_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	bd_t *binfo = (bd_t *) __res;
	uint	memsize;
	unsigned int freq;
	extern unsigned long total_memory;	/* in mm/init */

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	pvid = mfspr(SPRN_PVR);
	svid = mfspr(SPRN_SVR);

	memsize = total_memory;

	seq_printf(m, "Vendor\t\t: RPC Electronics STx \n");
	seq_printf(m, "Machine\t\t: GP3 - MPC%s\n", cur_ppc_sys_spec->ppc_sys_name);
	seq_printf(m, "bus freq\t: %u.%.6u MHz\n", freq / 1000000,
		   freq % 1000000);
	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);

	/* Display cpu Pll setting */
	phid1 = mfspr(SPRN_HID1);
	seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t: %d MB\n", memsize / (1024 * 1024));

	return 0;
}

#ifdef CONFIG_PCI
int mpc85xx_map_irq(struct pci_dev *dev, unsigned char idsel,
		    unsigned char pin)
{
	struct pci_controller *hose = pci_bus_to_hose(dev->bus->number);

	if (!hose->index) {
		/* PCI1 interrupts */
		char pci_irq_table[][3] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE
		     *        A      B      C      D
		     */
		{
			{ P1IRQA, P1IRQB, P1IRQC },	/* PCI Slot */
			{ P1IRQB, P1IRQC, P1IRQA },	/* PMC Connector */
			{ P1IRQC, P1IRQA, P1IRQB },	/* Mini-PCI (IRQB not used) */
		};

		const long min_idsel = 11, max_idsel = 13, irqs_per_slot = 3;
		return PCI_IRQ_TABLE_LOOKUP;
	}
	else {
		/* PCI2 interrupts */
		char pci_irq_table[][3] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE
		     *        A      B      C      D
		     */
		{
			{ P2IRQA, P2IRQB, P2IRQC },	/* Mini-PCI */
			{ P2IRQB, P2IRQC, P2IRQA },	/* FireWire */
			{ P2IRQC, P2IRQA, P2IRQB },	/* IDE */
		};

		const long min_idsel = 16, max_idsel = 18, irqs_per_slot = 3;
		return PCI_IRQ_TABLE_LOOKUP;

	}
}

extern int mpc85xx_pci1_last_busno;

int mpc85xx_exclude_device(u_char bus, u_char devfn)
{
	/* Since there aren't any PCI-other bus bridges, we can exclude
	 * the host bridge in all cases to get both PCI1 and PCI2.
	 */
	if (PCI_SLOT(devfn) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return PCIBIOS_SUCCESSFUL;
}

/* What a hack....  Due to bugs in the IDE code, it was easier to do
 * this here.  The on-board IDE controller pins for the second channel
 * are not connected to anything, so the probe just hangs.  The ide1=noprobe
 * option doesn't work, it's forcibly set based on io port values.
 * So, we trap it here, check for the ide1 channel, and ensure the
 * probe isn't done.
 */
static  void
gp3ssa_ide_init_hwif(hw_regs_t *hw, unsigned long io_addr,
				       unsigned long ctl_addr, int *irq)
{
	struct pci_dev *dev = NULL;

	ide_std_init_ports(hw, io_addr, ctl_addr);
	hw->io_ports[IDE_IRQ_OFFSET] = 0;

	while ((dev = pci_find_device(PCI_VENDOR_ID_ITE, PCI_DEVICE_ID_ITE_8211, dev)) != NULL) {
		/* We want to make sure we only capture the controller
		 * on the board, not one that someone may have plugged
		 * into a slot with the same chipset.
		 */
		if (PCI_SLOT(dev->devfn) == 18) {
			if (pci_resource_start(dev, 2) == io_addr) {
				/* This is IDE1.  Setting the data offset
				 * to zero, will cause the noprobe to be
				 * set to one.
				 */
				hw->io_ports[IDE_DATA_OFFSET] = 0;
				break;
			}

		}
	}
}

#endif /* CONFIG_PCI */

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	/* parse_bootinfo must always be called first */
	parse_bootinfo(find_bootinfo());

	/*
	 * If we were passed in a board information, copy it into the
	 * residual data area.
	 */
	if (r3) {
		memcpy((void *) __res, (void *) (r3 + KERNELBASE),
		       sizeof (bd_t));

	}
#ifdef CONFIG_SERIAL_TEXT_DEBUG
	{
		bd_t *binfo = (bd_t *) __res;
		struct uart_port p;

		/* Use the last TLB entry to map CCSRBAR to allow access to DUART regs */
		settlbcam(num_tlbcam_entries - 1, binfo->bi_immr_base,
				binfo->bi_immr_base, MPC85xx_CCSRBAR_SIZE, _PAGE_IO, 0);

		memset(&p, 0, sizeof (p));
		p.iotype = UPIO_MEM;
		p.membase = (void *) binfo->bi_immr_base + MPC85xx_UART0_OFFSET;
		p.uartclk = binfo->bi_busfreq;

		gen550_init(0, &p);

		memset(&p, 0, sizeof (p));
		p.iotype = UPIO_MEM;
		p.membase = (void *) binfo->bi_immr_base + MPC85xx_UART1_OFFSET;
		p.uartclk = binfo->bi_busfreq;

		gen550_init(1, &p);
	}
#endif

#if defined(CONFIG_BLK_DEV_INITRD)
	/*
	 * If the init RAM disk has been configured in, and there's a valid
	 * starting address for it, set it up.
	 */
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif				/* CONFIG_BLK_DEV_INITRD */

	/* Copy the kernel command line arguments to a safe place. */

	if (r6) {
		*(char *) (r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *) (r6 + KERNELBASE));
	}

	identify_ppc_sys_by_id(mfspr(SPRN_SVR));

	/* setup the PowerPC module struct */
	ppc_md.setup_arch = gp3ssa_setup_arch;
	ppc_md.show_cpuinfo = gp3ssa_show_cpuinfo;

	ppc_md.init_IRQ = gp3ssa_init_IRQ;
	ppc_md.get_irq = openpic_get_irq;

	ppc_md.restart = mpc85xx_restart;
	ppc_md.power_off = mpc85xx_power_off;
	ppc_md.halt = mpc85xx_halt;

	ppc_md.find_end_of_memory = mpc85xx_find_end_of_memory;

	ppc_md.calibrate_decr = mpc85xx_calibrate_decr;

#ifdef CONFIG_PCI
	ppc_ide_md.ide_init_hwif = gp3ssa_ide_init_hwif;
#endif

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	ppc_md.progress = gen550_progress;
#endif /* CONFIG_SERIAL_8250 && CONFIG_SERIAL_TEXT_DEBUG */
#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_KGDB)
	ppc_md.early_serial_map = mpc85xx_early_serial_map;
#endif	/* CONFIG_SERIAL_8250 && CONFIG_KGDB */

	if (ppc_md.progress)
		ppc_md.progress("platform_init(): exit", 0);

	return;
}
