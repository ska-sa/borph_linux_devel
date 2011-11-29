/*
 * arch/ppc/platforms/4xx/ppc405ez.c
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Based on the ibm405gp.c
 *
 * Copyright 2006 AMCC (www.amcc.com)
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/param.h>
#include <linux/string.h>
#include <platforms/4xx/ppc405ez.h>
#include <asm/ibm4xx.h>
#include <asm/ocp.h>
#include <asm/ppc4xx_pic.h>

static struct ocp_func_emac_data ppc405ez_emac0_def = {
	.rgmii_idx	= -1,		/* No RGMII */
	.rgmii_mux	= -1,		/* No RGMII */
	.zmii_idx	= -1,		/* ZMII device index */
	.zmii_mux	= 0,		/* ZMII input of this EMAC */
	.mal_idx	= 0,		/* MAL device index */
	.mal_rx_chan	= 0,		/* MAL rx channel number */
	.mal_tx_chan	= 0,		/* MAL tx channel number */
	.wol_irq	= 17,		/* WOL interrupt number */
	.mdio_idx	= -1,		/* No shared MDIO */
	.tah_idx	= -1,		/* No TAH */
};
OCP_SYSFS_EMAC_DATA()

static struct ocp_func_mal_data ppc405ez_mal0_def = {
	.num_tx_chans	= 1,		/* Number of TX channels */
	.num_rx_chans	= 1,		/* Number of RX channels */
	.txeob_irq	= 19,		/* TX End Of Buffer IRQ  */
	.rxeob_irq	= 21,		/* RX End Of Buffer IRQ  */
	.txde_irq	= 18,		/* TX Descriptor Error IRQ */
	.rxde_irq	= 18,		/* RX Descriptor Error IRQ */
	.serr_irq	= 18,		/* MAL System Error IRQ    */
	.dcr_base	= DCRN_MAL_BASE /* MAL0_CFG DCR number */
};
OCP_SYSFS_MAL_DATA()

static struct ocp_func_iic_data ppc405ez_iic0_def = {
	.fast_mode	= 0,		/* Use standad mode (100Khz) */
};
OCP_SYSFS_IIC_DATA()

struct ocp_def core_ocp[] = {
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_OPB,
	  .index	= 0,
	  .paddr	= 0xEF600000,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_16550,
	  .index	= 0,
	  .paddr	= UART0_IO_BASE,
	  .irq		= UART0_INT,
	  .pm		= IBM_CPM_UART0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_16550,
	  .index	= 1,
	  .paddr	= UART1_IO_BASE,
	  .irq		= UART1_INT,
	  .pm		= IBM_CPM_UART1
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_IIC,
	  .paddr	= 0xEF600500,
	  .irq		= 10,
	  .pm		= IBM_CPM_IIC0,
	  .additions	= &ppc405ez_iic0_def,
	  .show		= &ocp_show_iic_data,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_GPIO,
	  .paddr	= 0xEF600700,
	  .irq		= OCP_IRQ_NA,
	  .pm		= IBM_CPM_GPIO0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_MAL,
	  .paddr	= OCP_PADDR_NA,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ppc405ez_mal0_def,
	  .show		= &ocp_show_mal_data,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_EMAC,
	  .index	= 0,
	  .paddr	= EMAC0_BASE,
	  .irq		= 16,
	  .pm		= IBM_CPM_EMAC0,
	  .additions	= &ppc405ez_emac0_def,
	  .show		= &ocp_show_emac_data,
	},
	{ .vendor	= OCP_VENDOR_INVALID
	}
};

/* Polarity and triggering settings for internal interrupt sources */
struct ppc4xx_uic_settings ppc4xx_core_uic_cfg[] __initdata = {
	{
		.polarity 	= 0xffffffe0,
		.triggering	= 0x00000200,
		.ext_irq_mask	= 0x0000001f,	/* IRQ0 - IRQ4 */
	},
	{ /* 1588 UIC Snapshot Sources */
		.polarity 	= 0xffffffe0,
		.triggering	= 0x00000200,
		.ext_irq_mask	= 0x0000001f,	/* IRQ0 - IRQ4 */
	}
};

static struct resource usb_gadget_resources[] = {
	[0] = {
		.start	= 0xEF640000,
		.end 	= 0xEF67FFFF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		/*.name	= "usb_device_irq",*/
		.start	= 15,
		.end	= 15,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ohci_usb_resources[] = {
	[0] = {
		.start	= 0x0EF603000,
		.end	= 0x0EF6031FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 13,
		.end	= 14,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 dma_mask = 0xffffffffULL;

static struct platform_device ohci_usb_device = {
	.name		= "ppc-soc-ohci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(ohci_usb_resources),
	.resource	= ohci_usb_resources,
	.dev		= {
		.dma_mask = &dma_mask,
		.coherent_dma_mask = 0xffffffffULL,
	}
};

static struct platform_device usb_gadget_device = {
	.name		= "dwc_otg",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(usb_gadget_resources),
	.resource       = usb_gadget_resources,
	.dev		= {
		.dma_mask = &dma_mask,
		.coherent_dma_mask = 0xffffffffULL,
	}
};

static struct platform_device *ppc405ez_devs[] __initdata = {
	&ohci_usb_device,
	&usb_gadget_device,
};

static int __init
ppc405ez_platform_add_devices(void)
{
	return platform_add_devices(ppc405ez_devs, ARRAY_SIZE(ppc405ez_devs));
}
arch_initcall(ppc405ez_platform_add_devices);

unsigned int get_base_baud(unsigned int port, unsigned int sysclk)
{
        unsigned int pllc_data, plld_data, perd0_data,
		pllFwdDiv, pllFbkDiv, pllFwdDivB,
		plloutb, uart_div, uart_clk, baud_base;

	pllc_data = CPR_READ(CPR_PLLC);
	plld_data = CPR_READ(CPR_PLLD);
	perd0_data = CPR_READ(CPR_PERD0);

	pllFwdDiv = (plld_data & CPR_PLLD_FWDVA_MASK) >> 16;

	pllFbkDiv = (plld_data & CPR_PLLD_FBDV_MASK)  >> 24;
	pllFbkDiv = (pllFbkDiv == 0) ? 256 : pllFbkDiv;

	pllFwdDivB = (plld_data & CPR_PLLD_FWDVB_MASK) >> 8;
	pllFwdDivB = (pllFwdDivB == 0) ? 8 : pllFwdDivB;

	uart_div = (perd0_data & CPR_PERD0_U0DV_MASK) >> 8;
	uart_div = (uart_div == 0) ? 256 : uart_div;

	if (pllc_data & CPR_PLLC_SRC_MASK)
		plloutb = (sysclk * pllFwdDivB * pllFbkDiv) / pllFwdDivB;
	else
		plloutb = (sysclk * pllFwdDiv * pllFbkDiv) / pllFwdDivB;

	uart_clk = plloutb / uart_div;
	baud_base = uart_clk / 16;

	return baud_base;
}
