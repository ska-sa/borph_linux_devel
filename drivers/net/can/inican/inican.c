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

#ifdef CONFIG_CAN_DEBUG_DEVICES
#define DBG(args...) printk(args)
#else
#define DBG(args...)
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/can.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <asm/io.h>

#include <linux/can/dev.h>
#include <linux/can/error.h>
#include "inican.h"

static int inican_write_object(struct net_device *dev,
			       struct can_frame *frame, int i)
{
	u32 ctrl = CAN_TXMSG_TXREQ;
	u32 id = 0;
	u32 val;

	if (frame->can_id & CAN_RTR_FLAG)
		ctrl |= CAN_TXMSG_RTR;

	/*
	 * Only called from inican_hard_start_xmit() with
	 * spin_lock_irqsave() already called, so no need for
	 * spin_lock here anymore
	 */

	if (frame->can_id & CAN_EFF_FLAG) {
		ctrl |= CAN_TXMSG_EXTD;
		id = frame->can_id & CAN_EFF_MASK;
		out_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXMSGID)),
			 CAN_ENCODE_MSGID_E(frame->can_id));
	} else {
		id = frame->can_id & CAN_SFF_MASK;
		out_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXMSGID)),
			 CAN_ENCODE_MSGID_S(frame->can_id));
	}

	memcpy(&val, &frame->data[0], 4);
	out_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXDATAHI)), val);
	memcpy(&val, &frame->data[4], 4);
	out_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXDATALO)), val);

	ctrl |= CAN_ENCODE_MSGLEN(frame->can_dlc & 0xf);

	/* trigger message transmission */
	out_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXCTRL)), ctrl);

	return 0;
}

static int inican_read_object(struct net_device *dev, int i)
{
	struct inican_priv *priv = netdev_priv(dev);
	u32 data;
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 ctrl;
	u32 id;

	skb = dev_alloc_skb(sizeof(struct can_frame));
	skb->dev = dev;

	frame = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

	/*
	 * Only called from ISR, so spin_lock_irqsave() is already called
	 */

	ctrl = in_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXCTRL)));

	if (!(ctrl & CAN_RXMSG_MSGAVBL)) {
		DBG("%s: 1 i=%d message should be here but is not\n", __func__, i);
		return -1;
	}

	frame->can_dlc = CAN_DECODE_MSGLEN(ctrl) & 0xf;

	data = in_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXDATAHI)));
	memcpy(&frame->data[0], &data, 4);
	data = in_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXDATALO)));
	memcpy(&frame->data[4], &data, 4);

	id = in_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXMSGID)));
	if (ctrl & CAN_RXMSG_ISEXTD) {
		id = CAN_DECODE_MSGID_E(id);
		frame->can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
	} else {
		id = CAN_DECODE_MSGID_S(id);
		frame->can_id = id & CAN_SFF_MASK;
	}

	if (ctrl & CAN_RXMSG_ISRTR)
		frame->can_id |= CAN_RTR_FLAG;

	/* clear message available in rx msg buf */
	out_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXCTRL)),
		 ctrl | CAN_RXMSG_MSGAVBL);

	skb->protocol = __constant_htons(ETH_P_CAN);
	netif_rx(skb);

	priv->can.net_stats.rx_packets++;
	priv->can.net_stats.rx_bytes += frame->can_dlc;

	return 0;
}

static int inican_setup_receive_object(struct net_device *dev,
				       int i, unsigned int mask,
				       unsigned int id)
{
	struct inican_priv *priv = netdev_priv(dev);
	u32 ctrl;
	unsigned long flags;

	spin_lock_irqsave(&priv->can.irq_lock, flags);

	/* enable Rx interrupt and Rx buffer */
	ctrl = CAN_RXMSG_CNTRLFLGS_WPNH | CAN_RXMSG_WPNL |
		CAN_RXMSG_RXINTEBL | CAN_RXMSG_BUFEBL;
	if (i != (RX_OBJECTS - 1))
		ctrl |= CAN_RXMSG_LNKFLG;
	out_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXCTRL)), ctrl);

	/* all messages accepted by CAN node */
	out_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXAMR)),
		 CAN_RXMSG_AMR_DEFAULT);
	out_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXAMRDAT)),
		 CAN_RXMSG_AMR_DEFAULT);
	out_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXACR)),
		 CAN_RXMSG_ACR_DEFAULT);
	out_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXACRDAT)),
		 CAN_RXMSG_ACR_DEFAULT);

	spin_unlock_irqrestore(&priv->can.irq_lock, flags);

	DBG("%s: obj no %d RXCTRL: 0x%08x\n", __FUNCTION__,
	    i, in_be32((u32 *)(dev->base_addr + CAN_RX_REG(i, CAN_RXCTRL))));

	return 0;
}

static int inican_setup_transmit_object(struct net_device *dev,
					int i, unsigned int mask,
					unsigned int id)
{
	struct inican_priv *priv = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->can.irq_lock, flags);

	/* enable Tx interrupt */
	out_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXCTRL)),
		 CAN_TXMSG_CNTRLFLGS1_WPN | CAN_TXMSG_CNTRLFLGS2_WPN |
		 CAN_TXMSG_INTEBL);

	spin_unlock_irqrestore(&priv->can.irq_lock, flags);

	DBG("%s: obj no %d TXCTRL: 0x%08x\n", __FUNCTION__,
	    i, in_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXCTRL))));

	return 0;
}

static int inican_abort_transmit(struct net_device *dev)
{
	struct inican_priv *priv = netdev_priv(dev);
	int i;

	/*
	 * Only called from ISR, so spin_lock_irqsave() is already called
	 */

	for (i = 0; i < TX_OBJECTS; i++) {
		if (in_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXCTRL)))
		    & CAN_TXMSG_TXREQ) {
			DBG("%s: aborting obj %d\n", __func__, i);
			priv->can.net_stats.tx_dropped++;
			priv->can.net_stats.tx_aborted_errors++;
			out_be32((u32 *)(dev->base_addr + CAN_TX_REG(i, CAN_TXCTRL)),
				 CAN_TXMSG_ABORT);
		}
	}

	return 0;
}

static int inican_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct inican_priv *priv = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *)skb->data;
	unsigned long flags;

	spin_lock_irqsave(&priv->can.irq_lock, flags);

	inican_write_object(dev, frame, priv->tx_object);
	priv->tx_object++;
	if (priv->tx_object >= TX_OBJECTS)
		netif_stop_queue(dev);

	spin_unlock_irqrestore(&priv->can.irq_lock, flags);

	priv->can.net_stats.tx_packets++;
	priv->can.net_stats.tx_bytes += frame->can_dlc;

	dev->trans_start = jiffies;
	dev_kfree_skb(skb);

	return 0;
}

static void inican_tx_timeout(struct net_device *dev)
{
	struct inican_priv *priv = netdev_priv(dev);

	priv->can.net_stats.tx_errors++;
}

static int inican_set_mode(struct net_device *dev, can_mode_t mode)
{
	switch (mode) {
	case CAN_MODE_START:
		DBG("%s: CAN_MODE_START requested\n", __FUNCTION__);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int inican_get_state(struct net_device *dev, can_state_t *state)
{
	u32 reg_status;

	reg_status = in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT));

	switch (reg_status & CAN_ERRSTAT_ERRSTE_MASK) {
	case CAN_ERRSTAT_ERRSTE_ACTIVE:
		*state = CAN_STATE_ACTIVE;
		break;
	case CAN_ERRSTAT_ERRSTE_PASSIVE:
		*state = CAN_STATE_BUS_PASSIVE;
		break;
	case CAN_ERRSTAT_ERRSTE_BUSOFF1:
	case CAN_ERRSTAT_ERRSTE_BUSOFF2:
		*state = CAN_STATE_BUS_OFF;
		break;
	}

	return 0;
}

static irqreturn_t inican_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct inican_priv *priv = netdev_priv(dev);
	unsigned long flags;
	u32 istat;
	u32 rxbstat;
	int i;

	spin_lock_irqsave(&priv->can.irq_lock, flags);

	istat = in_be32((u32 *)(dev->base_addr + CAN_ISTAT));
	while (istat) {

		if (istat & CAN_INT_TXMSG) {
			priv->tx_object--;
			netif_wake_queue(dev);

			/* Ack Tx interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_TXMSG);
		}

		if (istat & CAN_INT_RXMSG) {
			rxbstat = in_be32((u32 *)(dev->base_addr + CAN_RXBSTAT));
			for (i = 0; i < RX_OBJECTS; i++)
				if (rxbstat & (1 << i))
					inican_read_object(dev, i);

			/* Ack Rx interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_RXMSG);
		}

		/* Handle OverLoad frame detected */
		if (istat & CAN_INT_OVRLOAD) {
			DBG("%s: OVRLOAD-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/* Ack Overload interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_OVRLOAD);
		}

		/* Handle Rx message loss interrupt */
		if (istat & CAN_INT_RXMSGLOSS) {
			DBG("%s: RXMSGLOSS-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			priv->can.net_stats.rx_errors++;

			/* Ack Rx message loss interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_RXMSGLOSS);
		}

		/* Handle CAN Bus Off Interrupt */
		if (istat & CAN_INT_BUSOFF) {
			DBG("%s: BUSOFF-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/* Ack CAN bus off interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_BUSOFF);
		}

		/* Handle CAN CRC Error Interrupt */
		if (istat & CAN_INT_CRCERR) {
			DBG("%s: CRCERR-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/* Ack CAN CRC Error interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_CRCERR);
		}

		/* Handle CAN Message Format Error Interrupt */
		if (istat & CAN_INT_FORMERR) {
			DBG("%s: FORERR-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/* Ack CAN Message Format Error interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_FORMERR);
		}

		/* Handle CAN Message Ack Error Interrupt */
		if (istat & CAN_INT_ACKERR) {
			DBG("%s: ACKERR-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/*
			 * Need to cancel all messages currently in the
			 * transmitted
			 */
			inican_abort_transmit(dev);

			/* Ack CAN Message Ack Error interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_ACKERR);
		}

		/* Handle CAN Bit Stuff Error Interrupt */
		if (istat & CAN_INT_STUFERR) {
			DBG("%s: STUFERR-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/* Ack CAN Bit Stuff Error interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_STUFERR);
		}

		/* Handle CAN Bit Error Interrupt */
		if (istat & CAN_INT_BITERR) {
			DBG("%s: BITERR-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/* Ack CAN Bit Error interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_BITERR);
		}

		/* Handle CAN Arb Loss Interrupt */
		if (istat & CAN_INT_ARBLOSS) {
			DBG("%s: ARBLOSS-IRQ (ERRSTAT=%08x)\n", __func__,
			    in_be32((u32 *)(dev->base_addr + CAN_ERRSTAT)));

			/* Ack CAN Arb Loss interrupt */
			out_be32((u32 *)(dev->base_addr + CAN_ISTAT),
				 CAN_INT_ARBLOSS);
		}

		istat = in_be32((u32 *)(dev->base_addr + CAN_ISTAT));
	}

	spin_unlock_irqrestore(&priv->can.irq_lock, flags);

	return IRQ_HANDLED;
}

static int inican_open(struct net_device *dev)
{
	struct inican_priv *priv = netdev_priv(dev);
	unsigned long flags;

	if (request_irq(dev->irq, &inican_isr, 0, dev->name, dev)) {
		dev_err(ND2D(dev), "failed to attach interrupt %d\n", dev->irq);
		return -EAGAIN;
	}

	spin_lock_irqsave(&priv->can.irq_lock, flags);

	/* Enable Interrupts all! */
	out_be32((u32 *)(dev->base_addr + CAN_IEN),
		 in_be32((u32 *)(dev->base_addr + CAN_IEN)) |
		 CAN_INT_RXMSG | CAN_INT_TXMSG |
		 CAN_INT_OVRLOAD | CAN_INT_GLOBAL |
		 CAN_INT_RXMSGLOSS | CAN_INT_BUSOFF |
		 CAN_INT_CRCERR | CAN_INT_FORMERR |
		 CAN_INT_STUFERR | CAN_INT_BITERR |
		 CAN_INT_ARBLOSS | CAN_INT_ACKERR);

	/* Clear error register */
	out_be32((u32 *)(dev->base_addr + CAN_ERRSTAT), 0);

	/* Don't use listen only mode */
	out_be32((u32 *)(dev->base_addr + CAN_CMD),
		 in_be32((u32 *)(dev->base_addr + CAN_CMD)) & ~CAN_OPMODE_LISTEN);

	/* Set CAN device to Run Mode */
	out_be32((u32 *)(dev->base_addr + CAN_CMD),
		 in_be32((u32 *)(dev->base_addr + CAN_CMD)) | CAN_OPMODE_RUN);

	spin_unlock_irqrestore(&priv->can.irq_lock, flags);

	netif_wake_queue(dev);

	return 0;
}

static int inican_stop(struct net_device *dev)
{
	struct inican_priv *priv = netdev_priv(dev);
	unsigned long flags;

	netif_stop_queue(dev);

	cancel_delayed_work(&priv->work);
	flush_scheduled_work();

	/* mask all IRQs */
	spin_lock_irqsave(&priv->can.irq_lock, flags);

	/* Stop the CAN Controller */
	out_be32((u32 *)(dev->base_addr + CAN_CMD),
		 in_be32((u32 *)(dev->base_addr + CAN_CMD)) & ~CAN_OPMODE_RUN);

	spin_unlock_irqrestore(&priv->can.irq_lock, flags);

	free_irq(dev->irq, dev);

	return 0;
}

static int inican_set_bit_time(struct net_device *dev, struct can_bittime *bit_time)
{
	struct inican_priv *priv = netdev_priv(dev);
	u32 can_cfg;
	unsigned long flags;

	can_cfg = (CAN_CFG_BITRT_ENCODE(bit_time->std.brp - 1) |
		   CAN_CFG_TSEG1_ENCODE(bit_time->std.prop_seg + bit_time->std.phase_seg1 - 1) |
		   CAN_CFG_TSEG2_ENCODE(bit_time->std.phase_seg2 - 1) |
		   CAN_CFG_SJW_ENCODE(CAN_CFG_SJW_DEFAULT)) &
		(~CAN_CFG_ARB_FIXED & ~CAN_CFG_AUTORESTRT &
		 ~CAN_CFG_SMPLMODMAJ & ~CAN_CFG_EDGEMODBOTH);

	spin_lock_irqsave(&priv->can.irq_lock, flags);
	out_be32((u32 *)(dev->base_addr + CAN_CFG), can_cfg);
	spin_unlock_irqrestore(&priv->can.irq_lock, flags);

	DBG("%s: setting CANx_CFG to %08x\n", __FUNCTION__, can_cfg);

	return 0;
}

static int inican_chip_config(struct net_device *dev)
{
	struct inican_priv *priv = netdev_priv(dev);
	struct can_bittime bit_time;
	int ret, i;

	/* set bit timing */
	bit_time.type = CAN_BITTIME_STD;
	ret = can_calc_bit_time(&priv->can, priv->can.baudrate, &bit_time.std);
	if (ret == 0)
		ret = inican_set_bit_time(dev, &bit_time);
	if (ret)
		dev_err(ND2D(dev), "failed to config bit timing\n");

	/* setup message objects */
	for (i = 0; i < RX_OBJECTS; i++)
		inican_setup_receive_object(dev, i, 0, 0);
	for (i = 0; i < TX_OBJECTS; i++)
		inican_setup_transmit_object(dev, i, 0, 0);

	return 0;
}

struct net_device *alloc_inicandev(int sizeof_priv)
{
	struct net_device *dev;
	struct inican_priv *priv;

	dev = alloc_candev(sizeof_priv);
	if(!dev)
		return NULL;

	priv = netdev_priv(dev);

	dev->open = inican_open;
	dev->stop = inican_stop;
	dev->hard_start_xmit = inican_hard_start_xmit;
	dev->tx_timeout = inican_tx_timeout;

	priv->can.baudrate = 500000;

	priv->can.do_set_bit_time = inican_set_bit_time;
	priv->can.do_get_state = inican_get_state;
	priv->can.do_set_mode = inican_set_mode;

	priv->dev = dev;
	priv->tx_object = 0;

	return dev;
}
EXPORT_SYMBOL(alloc_inicandev);

void free_inicandev(struct net_device *dev)
{
	free_candev(dev);
}
EXPORT_SYMBOL(free_inicandev);

int register_inicandev(struct net_device *dev)
{
	inican_set_mode(dev, CAN_MODE_START);

	inican_chip_config(dev);

	return register_netdev(dev);
}
EXPORT_SYMBOL(register_inicandev);

void unregister_inicandev(struct net_device *dev)
{
	struct inican_priv *priv = netdev_priv(dev);

	inican_set_mode(dev, CAN_MODE_START);

	cancel_delayed_work(&priv->work);
	flush_scheduled_work();

	unregister_netdev(dev);
}
EXPORT_SYMBOL(unregister_inicandev);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver for iniCAN based chips");
