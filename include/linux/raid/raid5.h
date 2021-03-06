#ifndef _RAID5_H
#define _RAID5_H

#include <linux/raid/md.h>
#include <linux/raid/xor.h>
#include <linux/rbtree.h>

/*
 *
 * Each stripe contains one buffer per disc.  Each buffer can be in
 * one of a number of states stored in "flags".  Changes between
 * these states happen *almost* exclusively under a per-stripe
 * spinlock.  Some very specific changes can happen in bi_end_io, and
 * these are not protected by the spin lock.
 *
 * The flag bits that are used to represent these states are:
 *   R5_UPTODATE and R5_LOCKED
 *
 * State Empty == !UPTODATE, !LOCK
 *        We have no data, and there is no active request
 * State Want == !UPTODATE, LOCK
 *        A read request is being submitted for this block
 * State Dirty == UPTODATE, LOCK
 *        Some new data is in this buffer, and it is being written out
 * State Clean == UPTODATE, !LOCK
 *        We have valid data which is the same as on disc
 *
 * The possible state transitions are:
 *
 *  Empty -> Want   - on read or write to get old data for  parity calc
 *  Empty -> Dirty  - on compute_parity to satisfy write/sync request.(RECONSTRUCT_WRITE)
 *  Empty -> Clean  - on compute_block when computing a block for failed drive
 *  Want  -> Empty  - on failed read
 *  Want  -> Clean  - on successful completion of read request
 *  Dirty -> Clean  - on successful completion of write request
 *  Dirty -> Clean  - on failed write
 *  Clean -> Dirty  - on compute_parity to satisfy write/sync (RECONSTRUCT or RMW)
 *
 * The Want->Empty, Want->Clean, Dirty->Clean, transitions
 * all happen in b_end_io at interrupt time.
 * Each sets the Uptodate bit before releasing the Lock bit.
 * This leaves one multi-stage transition:
 *    Want->Dirty->Clean
 * This is safe because thinking that a Clean buffer is actually dirty
 * will at worst delay some action, and the stripe will be scheduled
 * for attention after the transition is complete.
 *
 * There is one possibility that is not covered by these states.  That
 * is if one drive has failed and there is a spare being rebuilt.  We
 * can't distinguish between a clean block that has been generated
 * from parity calculations, and a clean block that has been
 * successfully written to the spare ( or to parity when resyncing).
 * To distingush these states we have a stripe bit STRIPE_INSYNC that
 * is set whenever a write is scheduled to the spare, or to the parity
 * disc if there is no spare.  A sync request clears this bit, and
 * when we find it set with no buffers locked, we know the sync is
 * complete.
 *
 * Buffers for the md device that arrive via make_request are attached
 * to the appropriate stripe in one of two lists linked on b_reqnext.
 * One list (bh_read) for read requests, one (bh_write) for write.
 * There should never be more than one buffer on the two lists
 * together, but we are not guaranteed of that so we allow for more.
 *
 * If a buffer is on the read list when the associated cache buffer is
 * Uptodate, the data is copied into the read buffer and it's b_end_io
 * routine is called.  This may happen in the end_request routine only
 * if the buffer has just successfully been read.  end_request should
 * remove the buffers from the list and then set the Uptodate bit on
 * the buffer.  Other threads may do this only if they first check
 * that the Uptodate bit is set.  Once they have checked that they may
 * take buffers off the read queue.
 *
 * When a buffer on the write list is committed for write it is copied
 * into the cache buffer, which is then marked dirty, and moved onto a
 * third list, the written list (bh_written).  Once both the parity
 * block and the cached buffer are successfully written, any buffer on
 * a written list can be returned with b_end_io.
 *
 * The write list and read list both act as fifos.  The read list is
 * protected by the device_lock.  The write and written lists are
 * protected by the stripe lock.  The device_lock, which can be
 * claimed while the stipe lock is held, is only for list
 * manipulations and will only be held for a very short time.  It can
 * be claimed from interrupts.
 *
 *
 * Stripes in the stripe cache can be on one of two lists (or on
 * neither).  The "inactive_list" contains stripes which are not
 * currently being used for any request.  They can freely be reused
 * for another stripe.  The "handle_list" contains stripes that need
 * to be handled in some way.  Both of these are fifo queues.  Each
 * stripe is also (potentially) linked to a hash bucket in the hash
 * table so that it can be found by sector number.  Stripes that are
 * not hashed must be on the inactive_list, and will normally be at
 * the front.  All stripes start life this way.
 *
 * The inactive_list, handle_list and hash bucket lists are all protected by the
 * device_lock.
 *  - stripes on the inactive_list never have their stripe_lock held.
 *  - stripes have a reference counter. If count==0, they are on a list.
 *  - If a stripe might need handling, STRIPE_HANDLE is set.
 *  - When refcount reaches zero, then if STRIPE_HANDLE it is put on
 *    handle_list else inactive_list
 *
 * This, combined with the fact that STRIPE_HANDLE is only ever
 * cleared while a stripe has a non-zero count means that if the
 * refcount is 0 and STRIPE_HANDLE is set, then it is on the
 * handle_list and if recount is 0 and STRIPE_HANDLE is not set, then
 * the stripe is on inactive_list.
 *
 * The possible transitions are:
 *  activate an unhashed/inactive stripe (get_active_stripe())
 *     lockdev check-hash unlink-stripe cnt++ clean-stripe hash-stripe unlockdev
 *  activate a hashed, possibly active stripe (get_active_stripe())
 *     lockdev check-hash if(!cnt++)unlink-stripe unlockdev
 *  attach a request to an active stripe (add_stripe_bh())
 *     lockdev attach-buffer unlockdev
 *  handle a stripe (handle_stripe())
 *     lockstripe clrSTRIPE_HANDLE ...
 *		(lockdev check-buffers unlockdev) ..
 *		change-state ..
 *		record io/ops needed unlockstripe schedule io/ops
 *  release an active stripe (release_stripe())
 *     lockdev if (!--cnt) { if  STRIPE_HANDLE, add to handle_list else add to inactive-list } unlockdev
 *
 * The refcount counts each thread that have activated the stripe,
 * plus raid5d if it is handling it, plus one for each active request
 * on a cached buffer, and plus one if the stripe is undergoing stripe
 * operations.
 *
 * Stripe operations are performed outside the stripe lock,
 * the stripe operations are:
 * -copying data between the stripe cache and user application buffers
 * -computing blocks to save a disk access, or to recover a missing block
 * -updating the parity on a write operation (reconstruct write and
 *  read-modify-write)
 * -checking parity correctness
 * -running i/o to disk
 * These operations are carried out by raid5_run_ops which uses the async_tx
 * api to (optionally) offload operations to dedicated hardware engines.
 * When requesting an operation handle_stripe sets the pending bit for the
 * operation and increments the count.  raid5_run_ops is then run whenever
 * the count is non-zero.
 * There are some critical dependencies between the operations that prevent some
 * from being requested while another is in flight.
 * 1/ Parity check operations destroy the in cache version of the parity block,
 *    so we prevent parity dependent operations like writes and compute_blocks
 *    from starting while a check is in progress.  Some dma engines can perform
 *    the check without damaging the parity block, in these cases the parity
 *    block is re-marked up to date (assuming the check was successful) and is
 *    not re-read from disk.
 * 2/ When a write operation is requested we immediately lock the affected
 *    blocks, and mark them as not up to date.  This causes new read requests
 *    to be held off, as well as parity checks and compute block operations.
 * 3/ Once a compute block operation has been requested handle_stripe treats
 *    that block as if it is up to date.  raid5_run_ops guaruntees that any
 *    operation that is dependent on the compute block result is initiated after
 *    the compute block completes.
 */

struct stripe_queue;
struct stripe_head {
	struct hlist_node	hash;
	struct list_head	lru;			/* inactive_list or handle_list */
	sector_t		sector;			/* sector of this row */
	unsigned long		state;			/* state flags */
	atomic_t		count;			/* nr of active thread/requests */
	/* stripe_operations
	 * @pending - pending ops flags (set for request->issue->complete)
	 * @ack - submitted ops flags (set for issue->complete)
	 * @complete - completed ops flags (set for complete)
	 * @target - STRIPE_OP_COMPUTE_BLK target
	 * @count - raid5_runs_ops is set to run when this is non-zero
	 */
	struct stripe_operations {
		unsigned long	   pending;
		unsigned long	   ack;
		unsigned long	   complete;
		int		   target;
		int                target2;  /* second target for RAID-6 */
		int		   count;
		u32                zero_sum_result;     /* P-parity check */
		u32                zero_qsum_result;    /* Q-parity check */
	} ops;
	struct stripe_queue *sq; /* list of pending bios for this stripe */
	struct r5dev {
		struct bio	req;
		struct bio_vec	vec;
		struct page	*page;
		struct page	*dpage;	/* direct ptr to a bio buffer */
		unsigned long flags;
	} dev[1]; /* allocated with extra space depending of RAID geometry */
};

/* stripe_head_state - collects and tracks the dynamic state of a stripe_head
 *     for handle_stripe.  It is only valid under spin_lock(sq->lock);
 */
struct stripe_head_state {
	int syncing, expanding, expanded;
	int locked, uptodate, to_read, to_write, failed, written;
	int to_fill, compute, req_compute, non_overwrite;
	int failed_num;
};

/* r6_state - extra state data only relevant to r6 */
struct r6_state {
	int p_failed, q_failed, qd_idx, failed_num[2];
};

/* stripe_queue
 * @sector - rb_tree key
 * @lock
 * @sh - our stripe_head in the cache
 * @list_node - once this queue object satisfies some constraint (like full
 *  stripe write) it is placed on a list for processing by the cache
 * @overwrite_count - how many blocks are set to be overwritten
 */
struct stripe_queue {
	struct rb_node rb_node;
	sector_t sector;

	/* stripe queues are allocated with extra space to hold the following
	 * four bitmaps.  One bit for each block in the stripe_head.  These
	 * bitmaps enable use of hweight to count the number of blocks
	 * undergoing read, write, overwrite.
	 */
	unsigned long *to_read;
	unsigned long *to_write;
	unsigned long *overwrite;
	unsigned long *overlap; /* There is a pending overlapping request */
	spinlock_t lock; /* protect bio lists and stripe_head state */
	struct raid5_private_data *raid_conf;
	unsigned long state;
	struct list_head list_node;
	int bm_seq; /* sequence number for bitmap flushes */
	int pd_idx; /* parity disk index */
	int disks; /* disks in stripe */
	atomic_t count;
	struct r5_queue_dev {
		sector_t sector; /* hw starting sector for this block */
		struct bio *toread, *read, *towrite, *written;
	} dev[1];
};

/* Flags */
#define	R5_UPTODATE	0	/* page contains current data */
#define	R5_LOCKED	1	/* IO has been submitted on "req" */
#define	R5_OVERWRITE	2	/* towrite covers whole page */
/* and some that are internal to handle_stripe */
#define	R5_Insync	3	/* rdev && rdev->in_sync at start */
#define	R5_Wantread	4	/* want to schedule a read */
#define	R5_Wantwrite	5
#define	R5_ReadError	8	/* seen a read error here recently */
#define	R5_ReWrite	9	/* have tried to over-write the readerror */

#define	R5_Expanded	10	/* This block now has post-expand data */
#define	R5_Wantcompute	11 /* compute_block in progress treat as
				    * uptodate
				    */
#define	R5_Wantfill	12 /* dev->toread contains a bio that needs
				    * filling
				    */
#define	R5_Wantprexor	13 /* distinguish blocks ready for rmw from
				    * other "towrites"
				    */
/*
 * Write method
 */
#define RECONSTRUCT_WRITE	1
#define READ_MODIFY_WRITE	2
/* not a write method, but a compute_parity mode */
#define	CHECK_PARITY		3

/*
 * Stripe state
 */
#define STRIPE_HANDLE		2
#define	STRIPE_SYNCING		3
#define	STRIPE_INSYNC		4
#define	STRIPE_DEGRADED		7
#define	STRIPE_EXPAND_SOURCE	10
#define	STRIPE_EXPAND_READY	11

/*
 * Operations flags (in issue order)
 */
#define STRIPE_OP_BIOFILL	0
#define STRIPE_OP_COMPUTE_BLK	1
#define STRIPE_OP_PREXOR	2
#define STRIPE_OP_BIODRAIN	3
#define STRIPE_OP_POSTXOR	4
#define STRIPE_OP_CHECK	5
#define STRIPE_OP_IO		6

/* modifiers to the base operations
 * STRIPE_OP_MOD_REPAIR_PD - compute the parity block and write it back
 * STRIPE_OP_MOD_DMA_CHECK - parity is not corrupted by the check
 */
#define STRIPE_OP_MOD_REPAIR_PD 7
#define STRIPE_OP_MOD_DMA_CHECK 8

#define STRIPE_OP_CHECK_PP	9
#define STRIPE_OP_CHECK_QP	10
#define STRIPE_OP_UPDATE_PP	11
#define STRIPE_OP_UPDATE_QP	12

/*
 * Stripe-queue state
 */
#define STRIPE_QUEUE_BIT_DELAY 0
#define STRIPE_QUEUE_EXPANDING 1
#define STRIPE_QUEUE_PREREAD_ACTIVE 2

/*
 * Plugging:
 *
 * To improve write throughput, we need to delay the handling of some
 * stripes until there has been a chance that several write requests
 * for the one stripe have all been collected.
 * In particular, any write request that would require pre-reading
 * is put on a "delayed" queue until there are no stripes currently
 * in a pre-read phase.  Further, if the "delayed" queue is empty when
 * a stripe is put on it then we "plug" the queue and do not process it
 * until an unplug call is made. (the unplug_io_fn() is called).
 *
 * When preread is initiated on a stripe, we set PREREAD_ACTIVE and add
 * it to the count of prereading stripes.
 * When write is initiated, or the stripe refcnt == 0 (just in case) we
 * clear the PREREAD_ACTIVE flag and decrement the count
 * Whenever the 'handle' queue is empty and the device is not plugged, we
 * move any strips from delayed to handle and clear the DELAYED flag and set
 * PREREAD_ACTIVE.
 * In stripe_handle, if we find pre-reading is necessary, we do it if
 * PREREAD_ACTIVE is set, else we set DELAYED which will send it to the delayed queue.
 * HANDLE gets cleared if stripe_handle leave nothing locked.
 */
 

struct disk_info {
	mdk_rdev_t	*rdev;
};

struct raid5_private_data {
	struct hlist_head	*stripe_hashtbl;
	struct rb_root		stripe_queue_tree;
	mddev_t			*mddev;
	struct disk_info	*spare;
	int			chunk_size, level, algorithm;
	int			max_degraded;
	int			raid_disks;
	int			max_nr_stripes;

	/* used during an expand */
	sector_t		expand_progress;	/* MaxSector when no expand happening */
	sector_t		expand_lo; /* from here up to expand_progress it out-of-bounds
					    * as we haven't flushed the metadata yet
					    */
	int			previous_raid_disks;

	struct list_head	handle_list; /* stripes needing handling */
	struct list_head	bitmap_q_list; /* queues delaying awaiting
						  bitmap update */
	struct list_head	delayed_q_list; /* queues that have plugged
						 * requests
						 */
	struct list_head	io_hi_q_list; /* reads and full stripe writes */
	struct list_head	io_lo_q_list; /* sub-stripe-width writes */
	struct workqueue_struct *workqueue; /* attaches sq's to sh's */
	struct work_struct	stripe_queue_work;
	char 			workqueue_name[20];

	struct bio		*retry_read_aligned; /* currently retrying aligned bios   */
	struct bio		*retry_read_aligned_list; /* aligned bios retry list  */
	atomic_t		active_aligned_reads;
	atomic_t		preread_active_queues; /* queues with scheduled
							* io
							*/

	atomic_t		reshape_stripes; /* stripes with pending writes for reshape */
	/* unfortunately we need two cache names as we temporarily have
	 * two caches.
	 */
	int			active_name;
	char			sh_cache_name[2][20];
	char			sq_cache_name[2][20];
	struct kmem_cache	*sh_slab_cache;
	struct kmem_cache	*sq_slab_cache;

	int			seq_flush, seq_write;
	int			quiesce;

	int			fullsync;  /* set to 1 if a full sync is needed,
					    * (fresh device added).
					    * Cleared when a sync completes.
					    */

	/*
	 * Free queue pool
	 */
	atomic_t		active_queues;
	struct list_head	inactive_q_list;
	wait_queue_head_t	wait_for_queue;
	wait_queue_head_t	wait_for_overlap;
	int			inactive_queue_blocked;

	/*
	 * Free stripes pool
	 */
	atomic_t		active_stripes;
	struct list_head	inactive_list;
	wait_queue_head_t	wait_for_stripe;
	int			inactive_blocked;	/* release of inactive stripes blocked,
							 * waiting for 25% to be free
							 */
	int			pool_size; /* number of disks in stripeheads in pool */
	spinlock_t		device_lock;
	struct disk_info	*disks;
};

typedef struct raid5_private_data raid5_conf_t;

#define mddev_to_conf(mddev) ((raid5_conf_t *) mddev->private)

/*
 * Our supported algorithms
 */
#define ALGORITHM_LEFT_ASYMMETRIC	0
#define ALGORITHM_RIGHT_ASYMMETRIC	1
#define ALGORITHM_LEFT_SYMMETRIC	2
#define ALGORITHM_RIGHT_SYMMETRIC	3

#endif
