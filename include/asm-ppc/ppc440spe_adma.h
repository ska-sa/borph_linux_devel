/*
 * 2006-2007 (C) DENX Software Engineering.
 *
 * Author: Yuri Tikhonov <yur@emcraft.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of
 * any kind, whether express or implied.
 */

#ifndef PPC440SPE_ADMA_H
#define PPC440SPE_ADMA_H

#include <linux/types.h>
#include <asm/ppc440spe_dma.h>
#include <asm/ppc440spe_xor.h>

#define to_ppc440spe_adma_chan(chan) container_of(chan,ppc440spe_ch_t,common)
#define to_ppc440spe_adma_device(dev) container_of(dev,ppc440spe_dev_t,common)
#define tx_to_ppc440spe_adma_slot(tx) container_of(tx,ppc440spe_desc_t,async_tx)

#define PPC440SPE_R6_PROC_ROOT	"driver/440spe_raid6"
/* Default polynomial (for 440SP is only available) */
#define PPC440SPE_DEFAULT_POLY	0x4d

#define PPC440SPE_ADMA_WATCHDOG_MSEC	3
#define PPC440SPE_ADMA_THRESHOLD	5

#define PPC440SPE_DMA0_ID	0
#define PPC440SPE_DMA1_ID	1
#define PPC440SPE_XOR_ID	2

#define PPC440SPE_ADMA_DMA_MAX_BYTE_COUNT	0xFFFFFFUL
/* this is the XOR_CBBCR width */
#define PPC440SPE_ADMA_XOR_MAX_BYTE_COUNT	(1 << 31)
#define PPC440SPE_ADMA_ZERO_SUM_MAX_BYTE_COUNT PPC440SPE_ADMA_XOR_MAX_BYTE_COUNT

#define PPC440SPE_RXOR_RUN	0

#undef ADMA_LL_DEBUG

/**
 * struct ppc440spe_adma_device - internal representation of an ADMA device
 * @pdev: Platform device
 * @id: HW ADMA Device selector
 * @dma_desc_pool: base of DMA descriptor region (DMA address)
 * @dma_desc_pool_virt: base of DMA descriptor region (CPU address)
 * @common: embedded struct dma_device
 */
typedef struct ppc440spe_adma_device {
	struct platform_device *pdev;
	void *dma_desc_pool_virt;

	int id;
	dma_addr_t dma_desc_pool;
	struct dma_device common;
} ppc440spe_dev_t;

/**
 * struct ppc440spe_adma_chan - internal representation of an ADMA channel
 * @lock: serializes enqueue/dequeue operations to the slot pool
 * @device: parent device
 * @chain: device chain view of the descriptors
 * @common: common dmaengine channel object members
 * @all_slots: complete domain of slots usable by the channel
 * @pending: allows batching of hardware operations
 * @completed_cookie: identifier for the most recently completed operation
 * @slots_allocated: records the actual size of the descriptor slot pool
 * @hw_chain_inited: h/w descriptor chain initialization flag
 * @irq_tasklet: bottom half where ppc440spe_adma_slot_cleanup runs
 * @needs_unmap: if buffers should not be unmapped upon final processing
 */
typedef struct ppc440spe_adma_chan {
	spinlock_t lock;
	struct ppc440spe_adma_device *device;
	struct timer_list cleanup_watchdog;
	struct list_head chain;
	struct dma_chan common;
	struct list_head all_slots;
	struct ppc440spe_adma_desc_slot *last_used;
	int pending;
	dma_cookie_t completed_cookie;
	int slots_allocated;
	int hw_chain_inited;
	struct tasklet_struct irq_tasklet;
	u8 needs_unmap;
} ppc440spe_ch_t;

typedef struct ppc440spe_rxor {
	u32 addrl;
	u32 addrh;
	int len;
	int xor_count;
	int addr_count;
	int desc_count;
	int state;
} ppc440spe_rxor_cursor_t;

/**
 * struct ppc440spe_adma_desc_slot - PPC440SPE-ADMA software descriptor
 * @phys: hardware address of the hardware descriptor chain
 * @group_head: first operation in a transaction
 * @hw_next: pointer to the next descriptor in chain
 * @async_tx: support for the async_tx api
 * @slot_node: node on the iop_adma_chan.all_slots list
 * @chain_node: node on the op_adma_chan.chain list
 * @group_list: list of slots that make up a multi-descriptor transaction
 *      for example transfer lengths larger than the supported hw max
 * @unmap_len: transaction bytecount
 * @hw_desc: virtual address of the hardware descriptor chain
 * @stride: currently chained or not
 * @idx: pool index
 * @slot_cnt: total slots used in an transaction (group of operations)
 * @src_cnt: number of sources set in this descriptor
 * @dst_cnt: number of destinations set in the descriptor
 * @slots_per_op: number of slots per operation
 * @descs_per_op: number of slot per P/Q operation see comment
 * for ppc440spe_prep_dma_pqxor function
 * @flags: desc state/type
 * @reverse_flags: 1 if a corresponding rxor address uses reversed address order
 * @xor_check_result: result of zero sum
 * @crc32_result: result crc calculation
 */
typedef struct ppc440spe_adma_desc_slot {
	dma_addr_t phys;
	struct ppc440spe_adma_desc_slot *group_head;
	struct ppc440spe_adma_desc_slot *hw_next;
	struct dma_async_tx_descriptor async_tx;
	struct list_head slot_node;
	struct list_head chain_node; /* node in channel ops list */
	struct list_head group_list; /* list */
	unsigned int unmap_len;
	void *hw_desc;
	u16 stride;
	u16 idx;
	u16 slot_cnt;
	u8 src_cnt;
	u8 dst_cnt;
	u8 slots_per_op;
	u8 descs_per_op;
	unsigned long flags;
	unsigned long reverse_flags[8];

#define PPC440SPE_DESC_INT	0	/* generate interrupt on complete */
#define PPC440SPE_ZERO_DST	1	/* this chain includes CDBs for zeroing dests */
#define PPC440SPE_COHERENT	2	/* src/dst are coherent */

#define PPC440SPE_DESC_WXOR	4	/* WXORs are in chain */
#define PPC440SPE_DESC_RXOR	5	/* RXOR is in chain */

#define PPC440SPE_DESC_RXOR123	8	/* CDB for RXOR123 operation */
#define PPC440SPE_DESC_RXOR124	9	/* CDB for RXOR124 operation */
#define PPC440SPE_DESC_RXOR125	10	/* CDB for RXOR125 operation */
#define PPC440SPE_DESC_RXOR12	11	/* CDB for RXOR12 operation */
#define PPC440SPE_DESC_RXOR_REV	12	/* CDB contains srcs in reversed order */
#define PPC440SPE_DESC_RXOR_MSK	0x3

	ppc440spe_rxor_cursor_t rxor_cursor;

	union {
		u32 *xor_check_result;
		u32 *crc32_result;
	};
} ppc440spe_desc_t;

typedef struct ppc440spe_adma_platform_data {
	int hw_id;
	dma_cap_mask_t cap_mask;
	size_t pool_size;
} ppc440spe_aplat_t;

#endif /* PPC440SPE_ADMA_H */
