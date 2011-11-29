/*
 * inican.c - Inicore iniCAN network device driver
 *
 * Copyright 2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Based on ccan driver by
 *	Sascha Hauer, Marc Kleine-Budde, Pengutronix
 *	Simon Kallweit, intefo AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __INICAN_H__
#define __INICAN_H__

#include <linux/can.h>
#include <linux/platform_device.h>

/*
 * iniCAN register layout
 */
#define CAN_ISTAT	0x00
#define CAN_IEN		0x04
#define CAN_RXBSTAT	0x08
#define CAN_TXBSTAT	0x0c
#define CAN_ERRSTAT	0x10
#define CAN_CMD		0x14
#define CAN_CFG		0x18

#define CAN_TX_OFFS	0x20	/* Offset to CAN TX registers */
#define CAN_TXCTRL	0x00
#define CAN_TXMSGID	0x04
#define CAN_TXDATAHI	0x08
#define CAN_TXDATALO	0x0c
#define CAN_TX_SIZE	0x10	/* Size of CAN TX registers */

#define CAN_RX_OFFS	0x220	/* Offset to CAN RX registers */
#define CAN_RXCTRL	0x00
#define CAN_RXMSGID	0x04
#define CAN_RXDATAHI	0x08
#define CAN_RXDATALO	0x0c
#define CAN_RXAMR	0x10
#define CAN_RXACR	0x14
#define CAN_RXAMRDAT	0x18
#define CAN_RXACRDAT	0x1c
#define CAN_RX_SIZE	0x20	/* Size of CAN RX registers */

/* Interrupt Status Register */
#define CAN_INT_RXMSG		0x00001000
#define CAN_INT_TXMSG		0x00000800
#define CAN_INT_RXMSGLOSS	0x00000400
#define CAN_INT_BUSOFF		0x00000200
#define CAN_INT_CRCERR		0x00000100
#define CAN_INT_FORMERR		0x00000080
#define CAN_INT_ACKERR		0x00000040
#define CAN_INT_STUFERR		0x00000020
#define CAN_INT_BITERR		0x00000010
#define CAN_INT_OVRLOAD		0x00000008
#define CAN_INT_ARBLOSS		0x00000004
#define CAN_INT_GLOBAL		0x00000001

/* CAN Op Mode Register */
#define CAN_OPMODE_LISTEN	0x00000002
#define CAN_OPMODE_RUN		0x00000001

/* CAN Config Register */
#define CAN_CFG_BITRT_ENCODE(bitrate)	(((bitrate) << 16) & 0x7FFF0000)
#define CAN_CFG_ARB_FIXED		0x00002000
#define CAN_CFG_TSEG1_ENCODE(tseg1)	(((tseg1) << 8) & 0x00000F00)
#define CAN_CFG_TSEG2_ENCODE(tseg2)	(((tseg2) << 5) & 0x000000E0)
#define CAN_CFG_AUTORESTRT		0x00000010
#define CAN_CFG_SJW_ENCODE(sjw)		(((sjw) << 2) & 0x0000000C)
#define CAN_CFG_SMPLMODMAJ		0x00000002
#define CAN_CFG_EDGEMODBOTH		0x00000001

/* Some Pre-defined Config values */
#define CAN_BITRATE_DEFAULT		0
#define	CFG_TSEG1_DEFAULT		3
#define	CFG_TSEG2_DEFAULT		2
#define CAN_CFG_SJW_DEFAULT		2

/* TX Msg Buf Control Reg */
#define CAN_TXMSG_CNTRLFLGS1_WPN	0x00800000
#define CAN_TXMSG_RTR			0x00200000
#define CAN_TXMSG_EXTD			0x00100000
#define CAN_TXMSG_DLC_ENCODE(dlc)	(((dlc) << 16) & 0x000F0000)
#define CAN_TXMSG_CNTRLFLGS2_WPN	0x00000008
#define CAN_TXMSG_INTEBL		0x00000004
#define CAN_TXMSG_ABORT			0x00000002
#define CAN_TXMSG_TXREQ			0x00000001 /* R/W, To Xmit Message or check Pending */
#define CAN_ENCODE_MSGLEN(len)		(((len) << 16) & 0x000F0000)
#define CAN_ENCODE_MSGID_S(id)		((id) << 21)
#define CAN_ENCODE_MSGID_E(id)		((id) << 3)

/* RX Msg Buf Control Reg */
#define CAN_RXMSG_RTRABRT		0x00000004
#define CAN_RXMSG_RTRPLYPNDG		0x00000002
#define CAN_RXMSG_MSGAVBL		0x00000001
#define CAN_RXMSG_ACKMSG		0x00000001
#define CAN_RXMSG_CNTRLFLGS_WPNH	0x00800000
#define CAN_RXMSG_ISRTR			0x00200000
#define CAN_RXMSG_ISEXTD		0x00100000
#define CAN_RXMSG_WPNL			0x00000080
#define CAN_RXMSG_LNKFLG		0x00000040
#define CAN_RXMSG_RXINTEBL		0x00000020
#define CAN_RXMSG_AUTORTR		0x00000010
#define CAN_RXMSG_BUFEBL		0x00000008
#define CAN_RXMSG_AMR_DEFAULT		0xFFFFFFFF
#define CAN_RXMSG_ACR_DEFAULT		0x00000000
#define CAN_DECODE_MSGLEN(ctrl)		(((ctrl) >> 16) & 0x0000000F)
#define CAN_DECODE_MSGID_S(ctrl)	((ctrl) >> 21)
#define CAN_DECODE_MSGID_E(ctrl)	((ctrl) >> 3)

#define CAN_ERRSTAT_ERRSTE_MASK		0x00030000
#define CAN_ERRSTAT_ERRSTE_ACTIVE	0x00000000
#define CAN_ERRSTAT_ERRSTE_PASSIVE	0x00010000
#define CAN_ERRSTAT_ERRSTE_BUSOFF1	0x00020000
#define CAN_ERRSTAT_ERRSTE_BUSOFF2	0x00030000

/*
 * helper macros to access rx-/tx-objects
 */
#define CAN_TX_REG(i, reg)	(CAN_TX_OFFS + (i * CAN_TX_SIZE) + reg)
#define CAN_RX_REG(i, reg)	(CAN_RX_OFFS + (i * CAN_RX_SIZE) + reg)

#define RX_OBJECTS		32
#define TX_OBJECTS		32

struct inican_priv {
	struct can_priv can;
	struct net_device *dev;
	int tx_object;
	int last_status;
	struct delayed_work work;
};

extern struct net_device *alloc_inicandev(int sizeof_priv);
extern void free_inicandev(struct net_device *dev);
extern int register_inicandev(struct net_device *dev);
extern void unregister_inicandev(struct net_device *dev);
extern u32 inican_get_can_clock(void);

#endif /* __INICAN_H__ */
