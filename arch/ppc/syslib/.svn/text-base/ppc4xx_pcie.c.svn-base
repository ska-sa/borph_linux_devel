/*
 * Copyright (c) 2005 Cisco Systems.  All rights reserved.
 * Roland Dreier <rolandd@cisco.com>
 *
 * Major rework to not only support 440SPe but other 4xx PPC's with the
 * same PCIe core as well (like 405EX):
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/reg.h>
#include <asm/io.h>
#ifdef CONFIG_40x
#include <asm/ibm405.h>
#else
#include <asm/ibm44x.h>
#endif

#include "ppc4xx_pcie.h"

/* Configuration space is mapped for first 16 buses (0..15) */
#define MAX_BUS_MAPPED		0x10

int first_busno_to_port[MAX_BUS_MAPPED];

/*
 * Enable for early debug output on PCIe config cycles which
 * is very helpful for debugging problems when Linux hangs upon
 * bootup in the PCI configuration scan.
 *
 * Note: Don't forget to enable CONFIG_SERIAL_TEXT_DEBUG!
 */
#undef DEBUG_PCIE_CONFIG_ACCESS

#if defined DEBUG_PCIE_CONFIG_ACCESS
#include <asm/machdep.h>

#ifndef CONFIG_SERIAL_TEXT_DEBUG
#warning "Enable CONFIG_SERIAL_TEXT_DEBUG for early PCIe config cycle output!"
#endif

static void inline print_pcie_cfg(struct pci_bus *bus, unsigned int devfn,
				  int offset, int len, char *read_write)
{
	char str[80];

	sprintf(str, "pcie-config-%s: bus=%3d devfn=0x%04x offset=0x%04x len=%d",
		read_write, bus->number, devfn, offset, len);
	if (ppc_md.progress)
		ppc_md.progress(str, 0x200);
}
#else
static void inline print_pcie_cfg(struct pci_bus *bus, unsigned int devfn,
				  int offset, int len, char *read_write)
{
}
#endif

static int is_end_point(int port)
{
	u32 val;

	if (port < 0)
		return 1;

	val = SDR_READ(SDRN_PESDR_DLPSET(port));

	if (((val >> 20) & 0xf) == PTYPE_LEGACY_ENDPOINT)
		return 1;
	else
		return 0;
}

static void pcie_dmer_disable(void)
{
	mtdcr(DCRN_PCIE0_BASE + 0x16, mfdcr(DCRN_PCIE0_BASE + 0x16)
	      | GPL_DMER_MASK_DISA);	/* DMER disabled */
	mtdcr(DCRN_PCIE1_BASE + 0x16, mfdcr(DCRN_PCIE1_BASE + 0x16)
	      | GPL_DMER_MASK_DISA);	/* DMER disabled */
#if CFG_PCIE_NR_PORTS > 2
	mtdcr(DCRN_PCIE2_BASE + 0x16, mfdcr(DCRN_PCIE2_BASE + 0x16)
	      | GPL_DMER_MASK_DISA);	/* DMER disabled */
#endif
}

static void pcie_dmer_enable(void)
{
	mtdcr(DCRN_PCIE0_BASE + 0x16, mfdcr(DCRN_PCIE0_BASE + 0x16)
	      & ~GPL_DMER_MASK_DISA);	/* DMER enabled */
	mtdcr(DCRN_PCIE1_BASE + 0x16, mfdcr(DCRN_PCIE1_BASE + 0x16)
	      & ~GPL_DMER_MASK_DISA);	/* DMER enabled */
#if CFG_PCIE_NR_PORTS > 2
	mtdcr(DCRN_PCIE2_BASE + 0x16, mfdcr(DCRN_PCIE2_BASE + 0x16)
	      & ~GPL_DMER_MASK_DISA);	/* DMER enabled */
#endif
}

static int pcie_validate_bdf(struct pci_bus *bus, unsigned int devfn)
{
	struct pci_controller *hose = bus->sysdata;
	static int message = 0;

	/*
	 * Endpoint can not generate upstream(remote) config cycles.
	 */
	if (is_end_point(first_busno_to_port[hose->first_busno]))
		return PCIBIOS_DEVICE_NOT_FOUND;

	/*
	 * NOTICE: configuration space ranges are currenlty mapped only for
	 * buses from 0 to MAX_BUS_MAPPED - 1 , so such limit must be imposed.
	 * In case more buses are required MAX_BUS_MAPPED define needs to be
	 * altered accordingly (one bus takes 1 MB of memory space).
	 */
	if (bus->number >= MAX_BUS_MAPPED) {
		if (!message) {
			printk(KERN_WARNING "Warning! Probing bus %u\n"
			       "Linux on PPC4xx supports buses 0-%d\n"
			       "Devices on buses %d-255 won't be detected\n",
			       bus->number, MAX_BUS_MAPPED - 1, MAX_BUS_MAPPED);
			message++;
		}

		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/*
	 * Only single device is expected on the secondary bus of 4xx
	 * host bridge.
	 */
	if (bus->number == hose->first_busno && PCI_SLOT(devfn) != 1)
		return PCIBIOS_DEVICE_NOT_FOUND;

	return 0;
}

static void __iomem *pcie_get_base(struct pci_bus *bus, unsigned int devfn)
{
	struct pci_controller *hose = bus->sysdata;

	return (void __iomem *)(hose->cfg_data +
		(((bus->number) << 20) | (devfn << 12)) + CFG_PCIE_CFGBASE_OFFS);
}

static int pcie_read_config(struct pci_bus *bus, unsigned int devfn,
			    int offset, int len, u32 *val)
{
	volatile void __iomem *addr;

	if (pcie_validate_bdf(bus, devfn) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = pcie_get_base(bus, devfn);

	/*
	 * Reading from configuration space of non-existing device can
	 * generate transaction errors. For the read duration we suppress
	 * assertion of machine check exceptions to avoid those.
	 */
	pcie_dmer_disable();

	print_pcie_cfg(bus, devfn, offset, len, "read ");
	switch (len) {
	case 1:
		*val = in_8((u8 *)(addr + offset));
		break;
	case 2:
		*val = in_le16((u16 *)(addr + offset));
		break;
	default:
		*val = in_le32((u32 *)(addr + offset));
		break;
	}

	pcie_dmer_enable();

	return PCIBIOS_SUCCESSFUL;
}

static int pcie_write_config(struct pci_bus *bus, unsigned int devfn,
			     int offset, int len, u32 val)
{
	volatile void __iomem *addr;

	if (pcie_validate_bdf(bus, devfn) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = pcie_get_base(bus, devfn);

	/*
	 * Suppress MCK exceptions, similar to pcie_read_config()
	 */
	pcie_dmer_disable();

	print_pcie_cfg(bus, devfn, offset, len, "write");
	switch (len) {
	case 1:
		out_8((u8 *)(addr + offset), val);
		break;
	case 2:
		out_le16((u16 *)(addr + offset), val);
		break;
	default:
		out_le32((u32 *)(addr + offset), val);
		break;
	}

	pcie_dmer_enable();

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops pcie_pci_ops =
{
	.read  = pcie_read_config,
	.write = pcie_write_config
};

static int ppc4xx_init_pcie_common(void)
{
	int i;

	for (i = 0; i < MAX_BUS_MAPPED; i++)
		first_busno_to_port[i] = -1;

	return 0;
}

/*
 * Set up UTL registers
 */
#if defined(CONFIG_440SPE)
static void ppc4xx_setup_utl(u32 port)
{
	void __iomem *utl_base;

	/*
	 * Map UTL at 0xc_1000_n000
	 */
	switch (port) {
	case 0:
		mtdcr(DCRN_PEGPL_REGBAH(PCIE0), U64_TO_U32_HIGH(CFG_PCIE_UTLBASE));
		mtdcr(DCRN_PEGPL_REGBAL(PCIE0), U64_TO_U32_LOW(CFG_PCIE_UTLBASE) +
			0x1000 * port);
		mtdcr(DCRN_PEGPL_REGBAL(PCIE0), 0x10000000);
		mtdcr(DCRN_PEGPL_REGMSK(PCIE0), 0x00007001);
		mtdcr(DCRN_PEGPL_SPECIAL(PCIE0), 0x68782800);
		break;

	case 1:
		mtdcr(DCRN_PEGPL_REGBAH(PCIE1), U64_TO_U32_HIGH(CFG_PCIE_UTLBASE));
		mtdcr(DCRN_PEGPL_REGBAL(PCIE1), U64_TO_U32_LOW(CFG_PCIE_UTLBASE) +
			0x1000 * port);
		mtdcr(DCRN_PEGPL_REGMSK(PCIE1), 0x00007001);
		mtdcr(DCRN_PEGPL_SPECIAL(PCIE1), 0x68782800);
		break;

	case 2:
		mtdcr(DCRN_PEGPL_REGBAH(PCIE2), U64_TO_U32_HIGH(CFG_PCIE_UTLBASE));
		mtdcr(DCRN_PEGPL_REGBAL(PCIE2), U64_TO_U32_LOW(CFG_PCIE_UTLBASE) +
			0x1000 * port);
		mtdcr(DCRN_PEGPL_REGMSK(PCIE2), 0x00007001);
		mtdcr(DCRN_PEGPL_SPECIAL(PCIE2), 0x68782800);
		break;
	}
	utl_base = ioremap64(CFG_PCIE_UTLBASE + 0x1000 * port, 0x100);

	/*
	 * Set buffer allocations and then assert VRB and TXE.
	 */
	out_be32(utl_base + PEUTL_OUTTR,   0x08000000);
	out_be32(utl_base + PEUTL_INTR,    0x02000000);
	out_be32(utl_base + PEUTL_OPDBSZ,  0x10000000);
	out_be32(utl_base + PEUTL_PBBSZ,   0x53000000);
	out_be32(utl_base + PEUTL_IPHBSZ,  0x08000000);
	out_be32(utl_base + PEUTL_IPDBSZ,  0x10000000);
	out_be32(utl_base + PEUTL_RCIRQEN, 0x00f00000);
	out_be32(utl_base + PEUTL_PCTL,    0x80800066);

	iounmap(utl_base);
}

static int check_error(void)
{
	u32 valPE0, valPE1, valPE2;
	int err = 0;

	/* SDR0_PEGPLLLCT1 reset */
	if (!(valPE0 = SDR_READ(PESDR0_PLLLCT1) & 0x01000000)) {
		/*
		 * the PCIe core was probably already initialised
		 * by firmware - let's re-reset RCSSET regs
		 */
		pr_debug("PCIE: SDR0_PLLLCT1 already reset.\n");
		SDR_WRITE(PESDR0_RCSSET, 0x01010000);
		SDR_WRITE(PESDR1_RCSSET, 0x01010000);
		SDR_WRITE(PESDR2_RCSSET, 0x01010000);
	}

	valPE0 = SDR_READ(PESDR0_RCSSET);
	valPE1 = SDR_READ(PESDR1_RCSSET);
	valPE2 = SDR_READ(PESDR2_RCSSET);

	/* SDR0_PExRCSSET rstgu */
	if (!(valPE0 & 0x01000000) ||
	    !(valPE1 & 0x01000000) ||
	    !(valPE2 & 0x01000000)) {
		printk(KERN_INFO "PCIE: SDR0_PExRCSSET rstgu error\n");
		err = -1;
	}

	/* SDR0_PExRCSSET rstdl */
	if (!(valPE0 & 0x00010000) ||
	    !(valPE1 & 0x00010000) ||
	    !(valPE2 & 0x00010000)) {
		printk(KERN_INFO "PCIE: SDR0_PExRCSSET rstdl error\n");
		err = -1;
	}

	/* SDR0_PExRCSSET rstpyn */
	if ((valPE0 & 0x00001000) ||
	    (valPE1 & 0x00001000) ||
	    (valPE2 & 0x00001000)) {
		printk(KERN_INFO "PCIE: SDR0_PExRCSSET rstpyn error\n");
		err = -1;
	}

	/* SDR0_PExRCSSET hldplb */
	if ((valPE0 & 0x10000000) ||
	    (valPE1 & 0x10000000) ||
	    (valPE2 & 0x10000000)) {
		printk(KERN_INFO "PCIE: SDR0_PExRCSSET hldplb error\n");
		err = -1;
	}

	/* SDR0_PExRCSSET rdy */
	if ((valPE0 & 0x00100000) ||
	    (valPE1 & 0x00100000) ||
	    (valPE2 & 0x00100000)) {
		printk(KERN_INFO "PCIE: SDR0_PExRCSSET rdy error\n");
		err = -1;
	}

	/* SDR0_PExRCSSET shutdown */
	if ((valPE0 & 0x00000100) ||
	    (valPE1 & 0x00000100) ||
	    (valPE2 & 0x00000100)) {
		printk(KERN_INFO "PCIE: SDR0_PExRCSSET shutdown error\n");
		err = -1;
	}

	return err;
}

/*
 * Initialize PCI Express core as described in User Manual
 */
static int ppc4xx_init_pcie(void)
{
	int time_out = 20;

	ppc4xx_init_pcie_common();

	/* Set PLL clock receiver to LVPECL */
	SDR_WRITE(PESDR0_PLLLCT1, SDR_READ(PESDR0_PLLLCT1) | 1 << 28);

	if (check_error())
		return -1;

	if (!(SDR_READ(PESDR0_PLLLCT2) & 0x10000)) {
		printk(KERN_INFO "PCIE: PESDR_PLLCT2 resistance calibration "
		       "failed (0x%08x)\n",
		       SDR_READ(PESDR0_PLLLCT2));
		return -1;
	}

	/* De-assert reset of PCIe PLL, wait for lock */
	SDR_WRITE(PESDR0_PLLLCT1, SDR_READ(PESDR0_PLLLCT1) & ~(1 << 24));
	udelay(3);

	while (time_out) {
		if (!(SDR_READ(PESDR0_PLLLCT3) & 0x10000000)) {
			time_out--;
			udelay(1);
		} else
			break;
	}
	if (!time_out) {
		printk(KERN_INFO "PCIE: VCO output not locked\n");
		return -1;
	}

	pr_debug("PCIE initialization OK\n");

	return 0;
}
#else
static void ppc4xx_setup_utl(u32 port)
{
	void __iomem *utl_base;

	switch (port) {
	case 0:
		mtdcr(DCRN_PEGPL_REGBAH(PCIE0), 0x00000000);
		mtdcr(DCRN_PEGPL_REGBAL(PCIE0), CFG_PCIE_UTLBASE);
		mtdcr(DCRN_PEGPL_REGMSK(PCIE0), 0x00007001);
		mtdcr(DCRN_PEGPL_SPECIAL(PCIE0), 0);
		break;

	case 1:
		mtdcr(DCRN_PEGPL_REGBAH(PCIE1), 0x00000000);
		mtdcr(DCRN_PEGPL_REGBAL(PCIE1), CFG_PCIE_UTLBASE + 0x1000 * port);
		mtdcr(DCRN_PEGPL_REGMSK(PCIE1), 0x00007001);
		mtdcr(DCRN_PEGPL_SPECIAL(PCIE1), 0);
		break;

	}
	utl_base = ioremap(CFG_PCIE_UTLBASE + 0x1000 * port, 0x100);

	/*
	 * Set buffer allocations and then assert VRB and TXE.
	 */
	out_be32(utl_base + PEUTL_OUTTR,   0x02000000);
	out_be32(utl_base + PEUTL_INTR,    0x02000000);
	out_be32(utl_base + PEUTL_OPDBSZ,  0x04000000);
	out_be32(utl_base + PEUTL_PBBSZ,   0x21000000);
	out_be32(utl_base + PEUTL_IPHBSZ,  0x02000000);
	out_be32(utl_base + PEUTL_IPDBSZ,  0x04000000);
	out_be32(utl_base + PEUTL_RCIRQEN, 0x00f00000);
	out_be32(utl_base + PEUTL_PCTL,    0x80800066);

	out_be32(utl_base + PEUTL_PBCTL,   0x0800000c);
	out_be32(utl_base + PEUTL_RCSTA,
		 in_be32(utl_base + PEUTL_RCSTA) | 0x000040000);

	iounmap(utl_base);
}

static int ppc4xx_init_pcie(void)
{
	pr_debug("PCIE initialization OK\n");

	ppc4xx_init_pcie_common();

	/*
	 * Nothing to do on 405EX
	 */
	return 0;
}
#endif

/*
 * Initialize various parts of the PCI Express core for our port:
 *
 * - Set as a root port and enable max width
 *   (PXIE0 -> X8, PCIE1 and PCIE2 -> X4).
 * - Set up UTL configuration.
 * - Increase SERDES drive strength to levels suggested by AMCC.
 * - De-assert RSTPYN, RSTDL and RSTGU.
 *
 * NOTICE for 440SPE revB chip: PESDRn_UTLSET2 is not set - we leave it
 * with default setting 0x11310000. The register has new fields,
 * PESDRn_UTLSET2[LKINE] in particular: clearing it leads to PCIE core
 * hang.
 */
#if defined(CONFIG_440SPE)
static int ppc4xx_init_pcie_port_hw(int port, int rootport)
{
	u32 val = 1 << 24;

	if (rootport)
		val = PTYPE_ROOT_PORT << 20;
	else
		val = PTYPE_LEGACY_ENDPOINT << 20;

	if (port == 0)
		val |= LNKW_X8 << 12;
	else
		val |= LNKW_X4 << 12;

	SDR_WRITE(SDRN_PESDR_DLPSET(port), val);
	SDR_WRITE(SDRN_PESDR_UTLSET1(port), 0x20222222);
	if (!ppc440spe_revB())
		SDR_WRITE(SDRN_PESDR_UTLSET2(port), 0x11000000);
	SDR_WRITE(SDRN_PESDR_HSSL0SET1(port), 0x35000000);
	SDR_WRITE(SDRN_PESDR_HSSL1SET1(port), 0x35000000);
	SDR_WRITE(SDRN_PESDR_HSSL2SET1(port), 0x35000000);
	SDR_WRITE(SDRN_PESDR_HSSL3SET1(port), 0x35000000);
	if (port == 0) {
		SDR_WRITE(PESDR0_HSSL4SET1, 0x35000000);
		SDR_WRITE(PESDR0_HSSL5SET1, 0x35000000);
		SDR_WRITE(PESDR0_HSSL6SET1, 0x35000000);
		SDR_WRITE(PESDR0_HSSL7SET1, 0x35000000);
	}
	SDR_WRITE(SDRN_PESDR_RCSSET(port), (SDR_READ(SDRN_PESDR_RCSSET(port)) &
					    ~(1 << 24 | 1 << 16)) | 1 << 12);

	return 0;
}
#endif /* CONFIG_440SPE */

#if defined(CONFIG_405EX)
static void ppc4xx_pcie_phy_reset(int port)
{
	/* Assert the PE0_PHY reset */
	SDR_WRITE(SDRN_PESDR_RCSSET(port), 0x01010000);
	udelay(1000);

	/* deassert the PE0_hotreset */
	if (is_end_point(port))
		SDR_WRITE(SDRN_PESDR_RCSSET(port), 0x01111000);
	else
		SDR_WRITE(SDRN_PESDR_RCSSET(port), 0x01101000);

	/* poll for phy !reset */
	while (!(SDR_READ(SDRN_PESDR_PHYSTA(port)) & 0x00001000))
		;

	/* deassert the PE0_gpl_utl_reset */
	SDR_WRITE(SDRN_PESDR_RCSSET(port), 0x00101000);
}

static int ppc4xx_init_pcie_port_hw(int port, int rootport)
{
	u32 val;

	if (is_end_point(port))
		val = PTYPE_LEGACY_ENDPOINT;
	else
		val = PTYPE_ROOT_PORT;

	SDR_WRITE(SDRN_PESDR_DLPSET(port), 1 << 24 | val << 20 | LNKW_X1 << 12);
	SDR_WRITE(SDRN_PESDR_UTLSET1(port), 0x00000000);
	SDR_WRITE(SDRN_PESDR_UTLSET2(port), 0x01010000);
	SDR_WRITE(SDRN_PESDR_PHYSET1(port), 0x720F0000);
	SDR_WRITE(SDRN_PESDR_PHYSET2(port), 0x70600003);

	/*
	 * Only reset the PHY when no link is currently established.
	 * This is for the Atheros PCIe board which has problems to establish
	 * the link (again) after this PHY reset. All other currently tested
	 * PCIe boards don't show this problem.
	 * This has to be re-tested and fixed in a later release!
	 */
	val = SDR_READ(SDRN_PESDR_LOOP(port));
	if (!(val & 0x00001000))
		ppc4xx_pcie_phy_reset(port);

	if (port == 0)
		mtdcr(DCRN_PEGPL_CFG(PCIE0), 0x10000000);  /* guarded on */
	else
		mtdcr(DCRN_PEGPL_CFG(PCIE1), 0x10000000);  /* guarded on */

	return 0;
}
#endif /* CONFIG_405EX */

/*
 * We map PCI Express configuration access into the 512MB regions
 *
 * NOTICE: revB is very strict about PLB real addressess and ranges to
 * be mapped for config space; it seems to only work with d_nnnn_nnnn
 * range (hangs the core upon config transaction attempts when set
 * otherwise) while revA uses c_nnnn_nnnn.
 *
 * For revA:
 *     PCIE0: 0xc_4000_0000
 *     PCIE1: 0xc_8000_0000
 *     PCIE2: 0xc_c000_0000
 *
 * For revB:
 *     PCIE0: 0xd_0000_0000
 *     PCIE1: 0xd_2000_0000
 *     PCIE2: 0xd_4000_0000
 *
 * For 405EX:
 *     PCIE0: 0xa000_0000
 *     PCIE1: 0xc000_0000
 */
static inline u64 ppc4xx_get_cfgaddr(int port)
{
#if defined(CONFIG_405EX)
	if (port == 0)
		return (u64)CFG_PCIE0_CFGBASE;
	else
		return (u64)CFG_PCIE1_CFGBASE;
#endif
#if defined(CONFIG_440SPE)
	if (ppc440spe_revB()) {
		switch (port) {
		default:	/* to satisfy compiler */
		case 0:
			return 0x0000000d00000000ULL;
		case 1:
			return 0x0000000d20000000ULL;
		case 2:
			return 0x0000000d40000000ULL;
		}
	} else {
		switch (port) {
		default:	/* to satisfy compiler */
		case 0:
			return 0x0000000c40000000ULL;
		case 1:
			return 0x0000000c80000000ULL;
		case 2:
			return 0x0000000cc0000000ULL;
		}
	}
#endif
}

/*
 *  Yucca board as End point and root point setup
 *                    and
 *    testing inbound and out bound windows
 *
 *  YUCCA board can be plugged into another yucca board or you can get PCI-E
 *  cable which can be used to setup loop back from one port to another port.
 *  Please rememeber that unless there is a endpoint plugged in to root port it
 *  will not initialize. It is the same in case of endpoint , unless there is
 *  root port attached it will not initialize.
 *
 *  In this release of software all the PCI-E ports are configured as either
 *  endpoint or rootpoint.In future we will have support for selective ports
 *  setup as endpoint and root point in single board.
 *
 *  Once your board came up as root point , you can verify by reading
 *  /proc/bus/pci/devices. Where you can see the configuration registers
 *  of end point device attached to the port.
 *
 *  Enpoint cofiguration can be verified by connecting Yucca board to any
 *  host or another yucca board. Then try to scan the device. In case of
 *  linux use "lspci" or appripriate os command.
 *
 *  To verify the inbound and outbound windows on yucca to yucca configuration
 *  windows already configured for memory region 0. On root point side memory
 *  map the 36 bit address value 0x4 0000 0000(SRAM) then do the read write to
 *  the memory mapped address. On endpoint board memory map the 0x4 0000 0000
 *  read the data to verify if writes happened or not.For inbound window
 *  verificatio do the reverse way of write and read .
 */
int ppc4xx_init_pcie_root_or_endport(u32 port)
{
	static int core_init;
	int attempts;
	u32 val = 0;
	u64 addr;
	u32 low, high;

	if (!core_init) {
		if(ppc4xx_init_pcie())
			return -1;
		++core_init;
	}

	/*
	 * Initialize various parts of the PCI Express core for our port
	 */
	ppc4xx_init_pcie_port_hw(port, !is_end_point(port));

	/*
	 * Notice: the following delay has critical impact on device
	 * initialization - if too short (<50ms) the link doesn't get up.
	 */
	mdelay(BOARD_PCIE_SCAN_DELAY);

	val = SDR_READ(SDRN_PESDR_RCSSTS(port));
	if (val & (1 << 20)) {
		printk(KERN_WARNING "PCIE%d: PGRST failed %08x\n", port, val);
		return -1;
	}

	/*
	 * Verify link is up
	 */
	val = SDR_READ(SDRN_PESDR_LOOP(port));
	if (!(val & 0x00001000)) {
		printk(KERN_INFO "PCIE%d: link is not up.\n", port);
		return -1;
	}

	/*
	 * Setup UTL registers - but only on revA!
	 * We use default settings for revB chip.
	 */
	if (!ppc440spe_revB())
		ppc4xx_setup_utl(port);

	/*
	 * We map PCI Express configuration access into the 512MB regions
	 */
	addr = ppc4xx_get_cfgaddr(port);
	low = U64_TO_U32_LOW(addr);
	high = U64_TO_U32_HIGH(addr);

	switch (port) {
	case 0:
		mtdcr(DCRN_PEGPL_CFGBAH(PCIE0), high);
		mtdcr(DCRN_PEGPL_CFGBAL(PCIE0), low);
		mtdcr(DCRN_PEGPL_CFGMSK(PCIE0), 0xe0000001); /* 512MB region, valid */
		break;
	case 1:
		mtdcr(DCRN_PEGPL_CFGBAH(PCIE1), high);
		mtdcr(DCRN_PEGPL_CFGBAL(PCIE1), low);
		mtdcr(DCRN_PEGPL_CFGMSK(PCIE1), 0xe0000001); /* 512MB region, valid */
		break;
#if CFG_PCIE_NR_PORTS > 2
	case 2:
		mtdcr(DCRN_PEGPL_CFGBAH(PCIE2), high);
		mtdcr(DCRN_PEGPL_CFGBAL(PCIE2), low);
		mtdcr(DCRN_PEGPL_CFGMSK(PCIE2), 0xe0000001); /* 512MB region, valid */
		break;
#endif
	}

	/*
	 * Check for VC0 active and assert RDY.
	 */
	attempts = 10;
	while (!(SDR_READ(SDRN_PESDR_RCSSTS(port)) & (1 << 16))) {
		if (!(attempts--)) {
			printk(KERN_INFO "PCIE%d: VC0 not active\n", port);
			return -1;
		}
		mdelay(1000);
	}
	SDR_WRITE(SDRN_PESDR_RCSSET(port),
		  SDR_READ(SDRN_PESDR_RCSSET(port)) | 1 << 20);
	mdelay(100);

	return 0;
}

int ppc4xx_setup_pcie(struct pci_controller *hose, u32 port)
{
	void __iomem *mbase = 0;
	int attempts = 0;

#if defined(CONFIG_440SPE)
	if (ppc440spe_revB()) {
		/*
		 * Map configuration space for first MAX_BUS_MAPPED buses.
		 * One bus takes 1 MB of memory.
		 */
		hose->cfg_data = ioremap64(0xd00000000ull + port * 0x20000000,
					   MAX_BUS_MAPPED * 0x100000);
		/* Map local configuration space */
		hose->cfg_addr = ioremap64(0xd10000000ull + port * 0x20000000,
					   0x400);
	} else {
		/*
		 * Map configuration space for first MAX_BUS_MAPPED buses.
		 * One bus takes 1 MB of memory.
		 */
		hose->cfg_data = ioremap64(0xc40000000ull + port * 0x40000000,
					   MAX_BUS_MAPPED * 0x100000);
		/* Map local configuration space */
		hose->cfg_addr = ioremap64(0xc50000000ull + port * 0x40000000,
					   0x1000);
	}
#endif

#if defined(CONFIG_405EX)
	/*
	 * Map configuration space for first MAX_BUS_MAPPED buses.
	 * One bus takes 1 MB of memory.
	 */
	hose->cfg_data = ioremap(CFG_PCIE0_CFGBASE + port * 0x20000000,
				 MAX_BUS_MAPPED * 0x100000);
	/* Map local configuration space */
	hose->cfg_addr = ioremap(CFG_PCIE0_XCFGBASE + port * 0x20000000,
				 0x1000);
#endif

	if (!hose->cfg_addr || !hose->cfg_data) {
		printk(KERN_INFO "ppc4xx_setup_pcie: ioremap failed\n");
		return -1;
	}

	mbase = (void __iomem *)hose->cfg_addr;

	hose->ops = &pcie_pci_ops;

	/*
	 * Set bus numbers on our root port
	 */
	if (!is_end_point(port)) {
		out_8(mbase + PCI_PRIMARY_BUS, hose->first_busno);
		out_8(mbase + PCI_SECONDARY_BUS, ++hose->first_busno);
		out_8(mbase + PCI_SUBORDINATE_BUS, 0xff);

		/*
		 * Create a mapping between hose->first_busno and port number
		 * needed later in is_end_point() check in config cycles
		 */
		first_busno_to_port[hose->first_busno] = port;
	}

	/*
	 * Set up outbound translation to hose->mem_space from PLB
	 * addresses at an offset of 0xd_0000_0000.  We set the low
	 * bits of the mask to 11 to turn off splitting into 8
	 * subregions and to enable the outbound translation.
	 * POMs are set different for root and endpoints to
	 * different window ranges fron inbound and out bound transactions.
	 */
	if (is_end_point(port)) {
		out_le32(mbase + PECFG_POM0LAH, 0x0000fff8);
		out_le32(mbase + PECFG_POM0LAL, 0x0001fff8);
	} else {
		out_le32(mbase + PECFG_POM0LAH, 0);
		out_le32(mbase + PECFG_POM0LAL, hose->mem_space.start);

		out_le32(mbase + PECFG_POM2LAH, 0);
		out_le32(mbase + PECFG_POM2LAL, hose->io_space.start);
	}

	switch (port) {
	case 0:
		mtdcr(DCRN_PEGPL_OMR1BAH(PCIE0), CFG_PCIE_ADDR_HIGH);
		mtdcr(DCRN_PEGPL_OMR1BAL(PCIE0), hose->mem_space.start);
		mtdcr(DCRN_PEGPL_OMR1MSKH(PCIE0), 0x7fffffff); /* 2^27 (128M) */
		mtdcr(DCRN_PEGPL_OMR1MSKL(PCIE0), ~(BOARD_PCIE_MEM_SIZE - 1) | 3);

		mtdcr(DCRN_PEGPL_OMR3BAH(PCIE0), 0x00000000);
		mtdcr(DCRN_PEGPL_OMR3BAL(PCIE0), hose->io_space.start);
		mtdcr(DCRN_PEGPL_OMR3MSKH(PCIE0), 0x7fffffff); /* 2^27 (128M) */
		mtdcr(DCRN_PEGPL_OMR3MSKL(PCIE0), ~(BOARD_PCIE_IO_SIZE - 1) | 3);

		pr_debug("0:PEGPL_OMR1BA=%08x.%08x MSK=%08x.%08x\n",
			 mfdcr(DCRN_PEGPL_OMR1BAH(PCIE0)),
			 mfdcr(DCRN_PEGPL_OMR1BAL(PCIE0)),
			 mfdcr(DCRN_PEGPL_OMR1MSKH(PCIE0)),
			 mfdcr(DCRN_PEGPL_OMR1MSKL(PCIE0)));
		break;
	case 1:
		mtdcr(DCRN_PEGPL_OMR1BAH(PCIE1), CFG_PCIE_ADDR_HIGH);
		mtdcr(DCRN_PEGPL_OMR1BAL(PCIE1), hose->mem_space.start);
		mtdcr(DCRN_PEGPL_OMR1MSKH(PCIE1), 0x7fffffff); /* 2^27 (128M) */
		mtdcr(DCRN_PEGPL_OMR1MSKL(PCIE1), ~(BOARD_PCIE_MEM_SIZE - 1) | 3);

		mtdcr(DCRN_PEGPL_OMR3BAH(PCIE1), 0x00000000);
		mtdcr(DCRN_PEGPL_OMR3BAL(PCIE1), hose->io_space.start);
		mtdcr(DCRN_PEGPL_OMR3MSKH(PCIE1), 0x7fffffff); /* 2^27 (128M) */
		mtdcr(DCRN_PEGPL_OMR3MSKL(PCIE1), ~(BOARD_PCIE_IO_SIZE - 1) | 3);

		pr_debug("1:PEGPL_OMR1BA=%08x.%08x MSK=%08x.%08x\n",
			 mfdcr(DCRN_PEGPL_OMR1BAH(PCIE1)),
			 mfdcr(DCRN_PEGPL_OMR1BAL(PCIE1)),
			 mfdcr(DCRN_PEGPL_OMR1MSKH(PCIE1)),
			 mfdcr(DCRN_PEGPL_OMR1MSKL(PCIE1)));
		break;
#if CFG_PCIE_NR_PORTS > 2
	case 2:
		mtdcr(DCRN_PEGPL_OMR1BAH(PCIE2), CFG_PCIE_ADDR_HIGH);
		mtdcr(DCRN_PEGPL_OMR1BAL(PCIE2), hose->mem_space.start);
		mtdcr(DCRN_PEGPL_OMR1MSKH(PCIE2), 0x7fffffff); /* 2^27 (128M) */
		mtdcr(DCRN_PEGPL_OMR1MSKL(PCIE2), ~(BOARD_PCIE_MEM_SIZE - 1) | 3);

		mtdcr(DCRN_PEGPL_OMR3BAH(PCIE2), 0x00000000);
		mtdcr(DCRN_PEGPL_OMR3BAL(PCIE2), hose->io_space.start);
		mtdcr(DCRN_PEGPL_OMR3MSKH(PCIE2), 0x7fffffff); /* 2^27 (128M) */
		mtdcr(DCRN_PEGPL_OMR3MSKL(PCIE2), ~(BOARD_PCIE_IO_SIZE - 1) | 3);

		pr_debug("2:PEGPL_OMR1BA=%08x.%08x MSK=%08x.%08x\n",
			 mfdcr(DCRN_PEGPL_OMR1BAH(PCIE2)),
			 mfdcr(DCRN_PEGPL_OMR1BAL(PCIE2)),
			 mfdcr(DCRN_PEGPL_OMR1MSKH(PCIE2)),
			 mfdcr(DCRN_PEGPL_OMR1MSKL(PCIE2)));
		break;
#endif
	}

	out_le32(mbase + PCI_BASE_ADDRESS_0, 0);
	out_le32(mbase + PCI_BASE_ADDRESS_1, 0);

	if (!is_end_point(port)) {
		out_le32(mbase + PECFG_BAR0HMPA, 0x7fffffc);
		out_le32(mbase + PECFG_BAR0LMPA, 0);

		out_le32(mbase + PECFG_PIM0LAL, 0x00000000);
		out_le32(mbase + PECFG_PIM0LAH, 0x00000004);
		out_le32(mbase + PECFG_PIM1LAL, 0x00000000);
		out_le32(mbase + PECFG_PIM1LAH, 0x00000004);
		out_le32(mbase + PECFG_PIM01SAH, 0xffff0000);
		out_le32(mbase + PECFG_PIM01SAL, 0x00000000);
	} else {
		/* Set up 64MB inbound memory window at 0 */
		out_le32(mbase + PCI_BASE_ADDRESS_0, 0);
		out_le32(mbase + PCI_BASE_ADDRESS_1, 0);

		out_le32(mbase + PECFG_PIM01SAH, 0xffffffff);
		out_le32(mbase + PECFG_PIM01SAL, 0xfc000000);

		/* Setup BAR0 */
		out_le32(mbase + PECFG_BAR0HMPA, 0x7fffffff);
		out_le32(mbase + PECFG_BAR0LMPA, 0xfc000000 | PCI_BASE_ADDRESS_MEM_TYPE_64);

		/* Disable BAR1 & BAR2 */
		out_le32(mbase + PECFG_BAR1MPA, 0);
		out_le32(mbase + PECFG_BAR2HMPA, 0);
		out_le32(mbase + PECFG_BAR2LMPA, 0);

		out_le32(mbase + PECFG_PIM0LAL, U64_TO_U32_LOW(BOARD_PCIE_INBOUND_BASE));
		out_le32(mbase + PECFG_PIM0LAH, U64_TO_U32_HIGH(BOARD_PCIE_INBOUND_BASE));
	}

	out_le32(mbase + PECFG_PIMEN, 0x1);

	/* Enable I/O, Mem, and Busmaster cycles */
	out_le16(mbase + PCI_COMMAND,
		 in_le16(mbase + PCI_COMMAND) |
		 PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

	if (!is_end_point(port)) {
		out_le16(mbase + 0x200, 0xaaa0 + port);
		out_le16(mbase + 0x202, 0xbed0 + port);

		/* Set Class Code to PCI-PCI bridge and Revision Id to 1 */
		out_le32(mbase + 0x208, 0x06040001);

		printk(KERN_INFO "PCIE%d: successfully set as root-complex\n", port);
	} else {
		attempts = 10;
		while(!(SDR_READ(SDRN_PESDR_RCSSTS(port)) & (1 << 8))) {
			if (!(attempts--)) {
				printk(KERN_INFO "PCIE%d: BME not active\n", port);
				return -1;
			}
			mdelay(1000);
		}

		out_le16(mbase + 0x200, 0xeee0 + port);
		out_le16(mbase + 0x202, 0xfed0 + port);

		/* Set Class Code to Processor/PPC */
		out_le32(mbase + 0x208, 0x0b200001);

		printk(KERN_INFO "PCIE%d: successfully set as endpoint\n", port);
	}

	pr_debug("vendor-id 0x%x\n", in_le16(mbase + 0x0));
	pr_debug("device-id 0x%x\n", in_le16(mbase + 0x2));

	/*
	 * This code works as is with yucca plugged in another yucca board or any endpoint device
	 * pugged in yucca board as root point device.
	 * If you want to change configuration according to your configuration . Here are few gotch's.
	 * There could be hangs due to different reasons
	 * 1 -- It could be that endpoint is not initialyzed. So always initialize the endpoint first.
	 * 2 -- May be your POM and BARs are not set properly
	 * 3 -- careful with masks which decide window sizes
	 * Here is the test code which used for testing sram read . This test dumps remote and local sram
	 * locations which help in comparing visually.
	 * u32 *rsram,*lsram;
	 * int i=0;
	 * lsram = ioremap64(0x400000000 ,0x200);
	 * #ifndef CONFIG_PCIE_ENDPOINT
	 *    rsram = ioremap64(0xd00000000 + hose->mem_space.start,0x200);
	 *	 if(port == 1)
	 *		 for(i=0;i<20;i+=4)
	 *			 printk(KERN_INFO"endp sram 0x%x root sram 0x%x\n",*(rsram+i),*(lsram+i));
	 *#else
	 *    rsram = ioremap64(0xd00000000 + hose->mem_space.start,0x200);
	 *    for(i=0;i<20;i+=4)
	 *		 printk(KERN_INFO"rootp sram 0x%x endp sram 0x%x\n",*(rsram+i),*(lsram+i));
	 * #endif
	 */

	return 0;
}
