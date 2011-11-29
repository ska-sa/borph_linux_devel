/*
 * ibm_ocp.h
 *
 *      (c) Benjamin Herrenschmidt (benh@kernel.crashing.org)
 *          Mipsys - France
 *
 *          Derived from work (c) Armin Kuster akuster@pacbell.net
 *
 *          Additional support and port to 2.6 LDM/sysfs by
 *          Matt Porter <mporter@kernel.crashing.org>
 *          Copyright 2003-2004 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifdef __KERNEL__
#ifndef __IBM_OCP_H__
#define __IBM_OCP_H__

#include <asm/types.h>

/*
 * IBM 4xx OCP system information
 */
struct ocp_sys_info_data {
	int	opb_bus_freq;	/* OPB Bus Frequency (Hz) */
	int	ebc_bus_freq;	/* EBC Bus Frequency (Hz) */
	int	plb_bus_freq;	/* PLB Bus Frequency (Hz) */
};

extern struct ocp_sys_info_data ocp_sys_info;

/*
 *  Create sysfs attribute files for device.
 */
static inline int ocp_create_files (struct device *dev, struct device_attribute **p)
{
	int i, ret = 0;

	for (i=0; p[i]; i++) {
		if (unlikely(ret = device_create_file(dev, p[i]))) {
			/* roll-back */
			dev_err(dev, "Failed creating device attrs\n");
			while (i)
				device_remove_file(dev, p[--i]);
			break;
		}
	}

	return ret;
}

/*
 * EMAC additional data and sysfs support
 *
 * Note about mdio_idx: When you have a zmii, it's usually
 * not necessary, it covers the case of the 405EP which has
 * the MDIO lines on EMAC0 only
 *
 * Note about phy_map: Per EMAC map of PHY ids which should
 * be probed by emac_probe. Different EMACs can have
 * overlapping maps.
 *
 * Note, this map uses inverse logic for bits:
 *  0 - id should be probed
 *  1 - id should be ignored
 *
 * Default value of 0x00000000 - will result in usual
 * auto-detection logic.
 *
 */

struct ocp_func_emac_data {
	int	rgmii_idx;	/* RGMII device index or -1 */
	int	rgmii_mux;	/* RGMII input of this EMAC */
	int	zmii_idx;	/* ZMII device index or -1 */
	int	zmii_mux;	/* ZMII input of this EMAC */
	int	mal_idx;	/* MAL device index */
	int	mal_rx_chan;	/* MAL rx channel number */
	int	mal_tx_chan;	/* MAL tx channel number */
	int	wol_irq;	/* WOL interrupt */
	int	mdio_idx;	/* EMAC idx of MDIO master or -1 */
	int	tah_idx;	/* TAH device index or -1 */
	int	phy_mode;	/* PHY type or configurable mode */
	u8	mac_addr[6];	/* EMAC mac address */
	u32	phy_map;	/* EMAC phy map */
	u32	phy_feat_exc;	/* Excluded PHY features */
	int 	txcoal_irq;	/* Interrupt coalescence TX IRQ  */
	int 	rxcoal_irq;	/* Interrupt coalescence RX IRQ */
};

/* Sysfs support */
#define OCP_SYSFS_EMAC_DATA()						\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, rgmii_idx)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, rgmii_mux)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, zmii_idx)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, zmii_mux)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, mal_idx)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, mal_rx_chan)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, mal_tx_chan)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, wol_irq)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, mdio_idx)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, tah_idx)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, phy_mode)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "0x%08x\n", emac, phy_map)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "0x%08x\n", emac, phy_feat_exc)\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, txcoal_irq)	\
OCP_SYSFS_ADDTL(struct ocp_func_emac_data, "%d\n", emac, rxcoal_irq)	\
									\
static struct device_attribute *emac_attrs[] = {			\
	&dev_attr_emac_rgmii_idx,					\
	&dev_attr_emac_rgmii_mux,					\
	&dev_attr_emac_zmii_idx,					\
	&dev_attr_emac_zmii_mux,					\
	&dev_attr_emac_mal_idx,						\
	&dev_attr_emac_mal_rx_chan,					\
	&dev_attr_emac_mal_tx_chan,					\
	&dev_attr_emac_wol_irq,						\
	&dev_attr_emac_mdio_idx,					\
	&dev_attr_emac_tah_idx,						\
	&dev_attr_emac_phy_mode,					\
	&dev_attr_emac_phy_map,						\
	&dev_attr_emac_phy_feat_exc,					\
	&dev_attr_emac_txcoal_irq,					\
	&dev_attr_emac_rxcoal_irq, 					\
	NULL								\
};									\
									\
void ocp_show_emac_data(struct device *dev)				\
{									\
	ocp_create_files(dev, emac_attrs);				\
}

/*
 * PHY mode settings (EMAC <-> ZMII/RGMII bridge <-> PHY)
 */
#define PHY_MODE_NA	0
#define PHY_MODE_MII	1
#define PHY_MODE_RMII	2
#define PHY_MODE_SMII	3
#define PHY_MODE_RGMII	4
#define PHY_MODE_TBI	5
#define PHY_MODE_GMII	6
#define PHY_MODE_RTBI	7
#define PHY_MODE_SGMII	8

#ifdef CONFIG_40x
/*
 * Helper function to copy MAC addresses from the bd_t to OCP EMAC
 * additions.
 *
 * The range of EMAC indices (inclusive) to be copied are the arguments.
 */
static inline void ibm_ocp_set_emac(int start, int end)
{
	int i;
	struct ocp_def *def;

	/* Copy MAC addresses to EMAC additions */
	for (i=start; i<=end; i++) {
		def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, i);
		if (i == 0)
			memcpy(((struct ocp_func_emac_data *)def->additions)->mac_addr,
			       __res.bi_enetaddr, 6);
#if defined(CONFIG_405EP) || defined(CONFIG_44x)
		else if (i == 1)
			memcpy(((struct ocp_func_emac_data *)def->additions)->mac_addr,
			       __res.bi_enet1addr, 6);
#endif
#if defined(CONFIG_440GX)
		else if (i == 2)
			memcpy(((struct ocp_func_emac_data *)def->additions)->mac_addr,
			       __res.bi_enet2addr, 6);
		else if (i == 3)
			memcpy(((struct ocp_func_emac_data *)def->additions)->mac_addr,
			       __res.bi_enet3addr, 6);
#endif
	}
}
#endif

/*
 * MAL additional data and sysfs support
 */
struct ocp_func_mal_data {
	int	num_tx_chans;	/* Number of TX channels */
	int	num_rx_chans;	/* Number of RX channels */
	int 	txeob_irq;	/* TX End Of Buffer IRQ  */
	int 	rxeob_irq;	/* RX End Of Buffer IRQ  */
	int	txde_irq;	/* TX Descriptor Error IRQ */
	int	rxde_irq;	/* RX Descriptor Error IRQ */
	int	serr_irq;	/* MAL System Error IRQ    */
	int	dcr_base;	/* MALx_CFG DCR number   */
};

#define OCP_SYSFS_MAL_DATA()						\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, num_tx_chans)	\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, num_rx_chans)	\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, txeob_irq)	\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, rxeob_irq)	\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, txde_irq)	\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, rxde_irq)	\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, serr_irq)	\
OCP_SYSFS_ADDTL(struct ocp_func_mal_data, "%d\n", mal, dcr_base)	\
									\
static struct device_attribute *mal_attrs[] = {				\
	&dev_attr_mal_num_tx_chans,					\
	&dev_attr_mal_num_rx_chans,					\
	&dev_attr_mal_txeob_irq,					\
	&dev_attr_mal_rxeob_irq,					\
	&dev_attr_mal_txde_irq,						\
	&dev_attr_mal_rxde_irq,						\
	&dev_attr_mal_serr_irq,						\
	&dev_attr_mal_dcr_base,						\
	NULL								\
};									\
									\
void ocp_show_mal_data(struct device *dev)				\
{									\
	ocp_create_files(dev, mal_attrs);				\
}

/*
 * IIC additional data and sysfs support
 */
struct ocp_func_iic_data {
	int	fast_mode;	/* IIC fast mode enabled */
};

#define OCP_SYSFS_IIC_DATA()						\
OCP_SYSFS_ADDTL(struct ocp_func_iic_data, "%d\n", iic, fast_mode)	\
									\
void ocp_show_iic_data(struct device *dev)				\
{									\
	if(unlikely(device_create_file(dev, &dev_attr_iic_fast_mode))){	\
		dev_err(dev, "Failed creating device attrs\n");		\
	}								\
}
#endif /* __IBM_OCP_H__ */
#endif /* __KERNEL__ */
