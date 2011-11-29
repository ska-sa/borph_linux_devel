/*
 * Definitions for P3M7448 Prodrive Board.
 *
 * Author: Wolfgang Denk <ws@denx.de>
 *
 * Based on code done by Lee Nicks <allinux@gmail.com> 
 * Based on code done by Rabeeh Khoury - rabeeh@galileo.co.il
 * Based on code done by Mark A. Greer <mgreer@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

/*
 * The MV64360 has 2 PCI buses each with 1 window from the CPU bus to
 * PCI I/O space and 4 windows from the CPU bus to PCI MEM space.
 *
 * This is the CPU physical memory map (windows must be at least 64KB and start
 * on a boundary that is a multiple of the window size):
 *
 *    0x42000000-0x4203ffff      - Internal SRAM
 *    0x80000000-0x87ffffff      - PCI0 MEM
 *    0x88000000-0x8effffff      - PCI1 MEM
 *    0xf1000000-0xf100ffff      - MV64360 Registers (CONFIG_MV64X60_NEW_BASE)
 *    0xfa000000-0xfaffffff      - PCI0 I/O
 *    0xfb000000-0xfbffffff      - PCI1 I/O
 *    0xff000000-0xffffffff      - Boot window, 8 MB flash
 */

#ifndef __PPC_PLATFORMS_P3M7448_H
#define __PPC_PLATFORMS_P3M7448_H

#define P3M7448_PCI_INTA			27
#define P3M7448_PCI_INTB			29
#define P3M7448_PCI_INTC			16
#define P3M7448_PCI_INTD			17

#define P3M7448_RESET_PIN			30

/* CPU Physical Memory Map setup. */
#define P3M7448_BOOT_WINDOW_BASE		0xff000000
#define P3M7448_BOOT_WINDOW_SIZE		0x01000000 /* 16 MB */
#define P3M7448_INTERNAL_SRAM_BASE		0x42000000

#define P3M7448_PCI0_MEM_START_PROC_ADDR	0x88000000
#define P3M7448_PCI0_MEM_START_PCI_HI_ADDR	0x00000000
#define P3M7448_PCI0_MEM_START_PCI_LO_ADDR	0x80000000
#define P3M7448_PCI0_MEM_SIZE			0x08000000 /* 128 MB */
#define P3M7448_PCI0_IO_START_PROC_ADDR		0xfa000000
#define P3M7448_PCI0_IO_START_PCI_ADDR		0x00000000
#define P3M7448_PCI0_IO_SIZE			0x01000000 /* 16 MB */

#define P3M7448_PCI1_MEM_START_PROC_ADDR	0x88000000
#define P3M7448_PCI1_MEM_START_PCI_HI_ADDR	0x00000000
#define P3M7448_PCI1_MEM_START_PCI_LO_ADDR	0x88000000
#define P3M7448_PCI1_MEM_SIZE			0x08000000 /* 128 MB */
#define P3M7448_PCI1_IO_START_PROC_ADDR		0xfb000000
#define P3M7448_PCI1_IO_START_PCI_ADDR		0x00000000
#define P3M7448_PCI1_IO_SIZE			0x01000000 /* 16 MB */

#define P3M7448_DEFAULT_BAUD			115200
#define P3M7448_MPSC_CLK_SRC			8	   /* TCLK */
#define P3M7448_MPSC_CLK_FREQ			133333333

#define P3M7448_MTD_RESERVED1_SIZE		0x00800000 /* 8 MB */
#define P3M7448_MTD_RESERVED2_SIZE		0x00700000 /* 7 MB */
#define P3M7448_MTD_UBOOT_SIZE			0x00100000 /* 1 MB */

#define P3M7448_ETH0_PHY_ADDR			1
#define P3M7448_ETH1_PHY_ADDR			3

#define P3M7448_ETH_TX_QUEUE_SIZE		800
#define P3M7448_ETH_RX_QUEUE_SIZE		400

#define P3M7448_ETH_PORT_CONFIG_VALUE			\
	ETH_UNICAST_NORMAL_MODE			|	\
	ETH_DEFAULT_RX_QUEUE_0			|	\
	ETH_DEFAULT_RX_ARP_QUEUE_0		|	\
	ETH_RECEIVE_BC_IF_NOT_IP_OR_ARP		|	\
	ETH_RECEIVE_BC_IF_IP			|	\
	ETH_RECEIVE_BC_IF_ARP			|	\
	ETH_CAPTURE_TCP_FRAMES_DIS		|	\
	ETH_CAPTURE_UDP_FRAMES_DIS		|	\
	ETH_DEFAULT_RX_TCP_QUEUE_0		|	\
	ETH_DEFAULT_RX_UDP_QUEUE_0		|	\
	ETH_DEFAULT_RX_BPDU_QUEUE_0

#define P3M7448_ETH_PORT_CONFIG_EXTEND_VALUE		\
	ETH_SPAN_BPDU_PACKETS_AS_NORMAL		|	\
	ETH_PARTITION_DISABLE

#define GT_ETH_IPG_INT_RX(value)			\
	((value & 0x3fff) << 8)

#define P3M7448_ETH_PORT_SDMA_CONFIG_VALUE		\
	ETH_RX_BURST_SIZE_4_64BIT		|	\
	GT_ETH_IPG_INT_RX(0)			|	\
	ETH_TX_BURST_SIZE_4_64BIT

#define P3M7448_ETH_PORT_SERIAL_CONTROL_VALUE		\
	ETH_FORCE_LINK_PASS			|	\
	ETH_ENABLE_AUTO_NEG_FOR_DUPLX		|	\
	ETH_DISABLE_AUTO_NEG_FOR_FLOW_CTRL	|	\
	ETH_ADV_SYMMETRIC_FLOW_CTRL		|	\
	ETH_FORCE_FC_MODE_NO_PAUSE_DIS_TX	|	\
	ETH_FORCE_BP_MODE_NO_JAM		|	\
	BIT9					|	\
	ETH_DO_NOT_FORCE_LINK_FAIL		|	\
	ETH_RETRANSMIT_16_ATTEMPTS		|	\
	ETH_ENABLE_AUTO_NEG_SPEED_GMII		|	\
	ETH_DTE_ADV_0				|	\
	ETH_DISABLE_AUTO_NEG_BYPASS		|	\
	ETH_AUTO_NEG_NO_CHANGE			|	\
	ETH_MAX_RX_PACKET_9700BYTE		|	\
	ETH_CLR_EXT_LOOPBACK			|	\
	ETH_SET_FULL_DUPLEX_MODE		|	\
	ETH_ENABLE_FLOW_CTRL_TX_RX_IN_FULL_DUPLEX

static inline u32
p3m7448_bus_freq(void)
{
	return 133333333;
}

#endif	/* __PPC_PLATFORMS_P3M7448_H */
