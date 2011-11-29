/*
 * arch/ppc/platforms/4xx/ppc405ex.c
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
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
#include <platforms/4xx/ppc405ex.h>
#include <asm/ibm4xx.h>
#include <asm/ocp.h>
#include <asm/ppc4xx_pic.h>

static struct ocp_func_emac_data ppc405ex_emac0_def = {
	.rgmii_idx	= 0,            /* RGMII device index */
	.rgmii_mux	= 0,
	.mal_idx        = 0,            /* MAL device index */
	.mal_rx_chan    = 0,            /* MAL rx channel number */
	.mal_tx_chan    = 0,            /* MAL tx channel number */
	.wol_irq        = 61,		/* WOL interrupt number - same as 440EP */
	.mdio_idx       = -1,           /* No shared MDIO but always via ZMII bridge */
	.tah_idx	= -1,           /* No TAH */
	.txcoal_irq 	= 71,  		/* Interrupt coalescence TX IRQ */
	.rxcoal_irq 	= 73,  		/* Interrupt coalescence RX IRQ */
};

static struct ocp_func_emac_data ppc405ex_emac1_def = {
	.rgmii_idx	= 0,            /* RGMII */
	.rgmii_mux	= 1,            /* RGMII */
	.mal_idx        = 0,            /* MAL device index */
	.mal_rx_chan    = 1,            /* MAL rx channel number */
	.mal_tx_chan    = 1,            /* MAL tx channel number */
	.wol_irq        = 63,  		/* WOL interrupt number _- same as 440EP */
	.mdio_idx       = -1,           /* no shared MDIO but always via ZMII bridge */
	.tah_idx	= -1,           /* No TAH */
	.txcoal_irq 	= 72,  		/* Interrupt coalescence TX IRQ */
	.rxcoal_irq 	= 74,  		/* Interrupt coalescence RX IRQ */
};
OCP_SYSFS_EMAC_DATA()

static struct ocp_func_mal_data ppc405ex_mal0_def = {
	.num_tx_chans   = 4,  		/* Number of TX channels */
	.num_rx_chans   = 4,    	/* Number of RX channels */
	.txeob_irq	= 10,		/* TX End Of Buffer IRQ  - same as 440EP */
	.rxeob_irq	= 11,		/* RX End Of Buffer IRQ  - same as 440EP*/
	.txde_irq	= 33,		/* TX Descriptor Error IRQ - same as 440EP */
	.rxde_irq	= 34,		/* RX Descriptor Error IRQ - same as 440EP*/
	.serr_irq	= 32,		/* MAL System Error IRQ  - same as 440EP   */
	.dcr_base	= DCRN_MAL_BASE /* MAL0_CFG DCR number */
};
OCP_SYSFS_MAL_DATA()

static struct ocp_func_iic_data ppc405ex_iic0_def = {
	.fast_mode	= 0,		/* Use standad mode (100Khz) */
};

static struct ocp_func_iic_data ppc405ex_iic1_def = {
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
	  .index	= 0,
	  .paddr	= 0xEF600400,
	  .irq		= IIC0_INT,
	  .pm		= IBM_CPM_IIC0,
	  .additions	= &ppc405ex_iic0_def,
	  .show		= &ocp_show_iic_data
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_IIC,
	  .index	= 1,
	  .paddr	= 0xEF600500,
	  .irq		= IIC1_INT,
	  .pm		= IBM_CPM_IIC1,
	  .additions	= &ppc405ex_iic1_def,
	  .show		= &ocp_show_iic_data
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
	  .additions	= &ppc405ex_mal0_def,
	  .show		= &ocp_show_mal_data,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_EMAC,
	  .index	= 0,
	  .paddr	= EMAC0_BASE,
	  .irq		= EMAC0_INT,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ppc405ex_emac0_def,
	  .show		= &ocp_show_emac_data,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_EMAC,
	  .index	= 1,
	  .paddr	= EMAC1_BASE,
	  .irq		= EMAC1_INT,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ppc405ex_emac1_def,
	  .show		= &ocp_show_emac_data,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_RGMII,
	  .paddr	= 0xEF600B00,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	},
	{ .vendor	= OCP_VENDOR_INVALID
	}
};

/* Polarity and triggering settings for internal interrupt sources */
struct ppc4xx_uic_settings ppc4xx_core_uic_cfg[] __initdata = {
	{ .polarity	= 0xffbfefef,
	  .triggering	= 0x00007000,
	  .ext_irq_mask	= 0x00400010,   /* IRQ0 IRQ4 */
	},
	{ .polarity	= 0xfffac785,
	  .triggering	= 0x001d0040,
	  .ext_irq_mask	= 0x0000383a,   /* IRQ7-IRQ8-IRQ9 IRQ2,IRQ5 IRQ6 IRQ1 */
	},
	{ .polarity	= 0xf7ffffff,
	  .triggering	= 0x01e1fff8,
	  .ext_irq_mask	= 0x08000000,   /* IRQ3*/
	},
};

static struct resource usb_otg_resources[] = {
        [0] = {
                .start  = 0xEF6C0000,
                .end    = 0xEF6CFFFF,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                /*.name = "usb_device_irq",*/
                .start  = 94,
                .end    = 94,
                .flags  = IORESOURCE_IRQ,
        },
};

static u64 dma_mask = 0xffffffffULL;

static struct platform_device usb_otg_device = {
        .name = "dwc_otg",
        .id = 0,
        .num_resources = ARRAY_SIZE(usb_otg_resources),
        .resource = usb_otg_resources,
        .dev = {
                .dma_mask = &dma_mask,
                .coherent_dma_mask = 0xffffffffULL,
        }
};

static struct platform_device *ppc405ex_devs[] __initdata = {
        &usb_otg_device,
};

static int __init ppc405ex_platform_add_devices(void)
{
        return platform_add_devices(ppc405ex_devs, ARRAY_SIZE(ppc405ex_devs));
}
device_initcall(ppc405ex_platform_add_devices);
