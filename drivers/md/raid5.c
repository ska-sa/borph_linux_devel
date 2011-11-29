/*
 * raid5.c : Multiple Devices driver for Linux
 *	   Copyright (C) 1996, 1997 Ingo Molnar, Miguel de Icaza, Gadi Oxman
 *	   Copyright (C) 1999, 2000 Ingo Molnar
 *	   Copyright (C) 2002, 2003 H. Peter Anvin
 *
 * RAID-4/5/6 management functions.
 * Thanks to Penguin Computing for making the RAID-6 development possible
 * by donating a test server!
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * You should have received a copy of the GNU General Public License
 * (for example /usr/src/linux/COPYING); if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * BITMAP UNPLUGGING:
 *
 * The sequencing for updating the bitmap reliably is a little
 * subtle (and I got it wrong the first time) so it deserves some
 * explanation.
 *
 * We group bitmap updates into batches.  Each batch has a number.
 * We may write out several batches at once, but that isn't very important.
 * conf->bm_write is the number of the last batch successfully written.
 * conf->bm_flush is the number of the last batch that was closed to
 *    new additions.
 * When we discover that we will need to write to any block in a stripe
 * (in add_queue_bio) we update the in-memory bitmap and record in the
 * stripe_queue that a bitmap write was started.  Then, in handle_stripe when
 * we have a stripe_head available, we update sq->bm_seq to record the
 * sequence number (target batch number) of this request.  This is bm_flush+1.
 * When we are ready to do a write, if that batch hasn't been written yet,
 *   we plug the array and queue the stripe for later.
 * When an unplug happens, we increment bm_flush, thus closing the current
 *   batch.
 * When we notice that bm_flush > bm_write, we write out all pending updates
 * to the bitmap, and advance bm_write to where bm_flush was.
 * This may occasionally write a bit out twice, but is sure never to
 * miss any bits.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/bitops.h>
#include <linux/kthread.h>
#include <asm/atomic.h>
#include "raid6.h"

#include <linux/raid/bitmap.h>
#include <linux/async_tx.h>

/*
 * Stripe cache
 */

#define NR_STRIPES		256
#define STRIPE_SIZE		PAGE_SIZE
#define STRIPE_SHIFT		(PAGE_SHIFT - 9)
#define STRIPE_SECTORS		(STRIPE_SIZE>>9)
#define	IO_THRESHOLD		1
#define NR_HASH			(PAGE_SIZE / sizeof(struct hlist_head))
#define HASH_MASK		(NR_HASH - 1)
#define STRIPE_QUEUE_SIZE 2 /* multiple of nr_stripes */

#define stripe_hash(conf, sect)	(&((conf)->stripe_hashtbl[((sect) >> STRIPE_SHIFT) & HASH_MASK]))

/* bio's attached to a stripe+device for I/O are linked together in bi_sector
 * order without overlap.  There may be several bio's per stripe+device, and
 * a bio could span several devices.
 * When walking this list for a particular stripe+device, we must never proceed
 * beyond a bio that extends past this device, as the next bio might no longer
 * be valid.
 * This macro is used to determine the 'next' bio in the list, given the sector
 * of the current stripe+device
 */
#define r5_next_bio(bio, sect) ( ( (bio)->bi_sector + ((bio)->bi_size>>9) < sect + STRIPE_SECTORS) ? (bio)->bi_next : NULL)
#define r5_io_weight_size(devs) (sizeof(unsigned long) * \
				  (ALIGN(devs, BITS_PER_LONG) / BITS_PER_LONG))
/*
 * The following can be used to debug the driver
 */
#define RAID5_PARANOIA	1
#if RAID5_PARANOIA && defined(CONFIG_SMP)
# define CHECK_DEVLOCK() assert_spin_locked(&conf->device_lock)
#else
# define CHECK_DEVLOCK()
#endif

#ifdef DEBUG
#define inline
#define __inline__
#endif

#if !RAID6_USE_EMPTY_ZERO_PAGE
/* In .bss so it's zeroed */
const char raid6_empty_zero_page[PAGE_SIZE] __attribute__((aligned(256)));
#endif

static inline int raid6_next_disk(int disk, int raid_disks)
{
	disk++;
	return (disk < raid_disks) ? disk : 0;
}

static void return_io(struct bio *return_bi)
{
	struct bio *bi = return_bi;
	while (bi) {

		return_bi = bi->bi_next;
		bi->bi_next = NULL;
		bi->bi_size = 0;
		bi->bi_end_io(bi,
			      test_bit(BIO_UPTODATE, &bi->bi_flags)
			        ? 0 : -EIO);
		bi = return_bi;
	}
}

#if BITS_PER_LONG == 32
#define hweight hweight32
#else
#define hweight hweight64
#endif
static unsigned long io_weight(unsigned long *bitmap, int disks)
{
	unsigned long weight = hweight(*bitmap);

	for (bitmap++; disks > BITS_PER_LONG; disks -= BITS_PER_LONG, bitmap++)
		weight += hweight(*bitmap);

	return weight;
}

static void print_raid5_conf (raid5_conf_t *conf);

/* __release_queue - route the stripe_queue based on pending i/o's.  The
 * queue object is allowed to bounce around between 4 lists up until
 * it is attached to a stripe_head.  The lists in order of priority are:
 * 1/ io_hi: all data blocks are set to be overwritten, no prereads
 * 2/ io_lo: read requests that get past chunk_aligned_read, or write requests
 *    activated for pre-reading
 * 3/ delayed_q: write requests pending pre-read activation
 * 4/ bitmap_q: stripes waiting for bitmap updates
 */
static void __release_queue(raid5_conf_t *conf, struct stripe_queue *sq)
{
	if (atomic_dec_and_test(&sq->count)) {
		struct list_head *io_list = NULL;
		int disks = sq->disks;
		int data_disks = disks - conf->max_degraded;
		int to_write = io_weight(sq->to_write, disks);

		BUG_ON(!list_empty(&sq->list_node));
		BUG_ON(atomic_read(&conf->active_queues) == 0);

		if (to_write &&
		    io_weight(sq->overwrite, disks) == data_disks)
			io_list = &conf->io_hi_q_list;
		else if (io_weight(sq->to_read, disks))
			io_list = &conf->io_lo_q_list;
		else if (to_write &&
			 test_bit(STRIPE_QUEUE_PREREAD_ACTIVE, &sq->state))
			io_list = &conf->io_lo_q_list;
		else if (to_write) {
			list_add_tail(&sq->list_node, &conf->delayed_q_list);
			blk_plug_device(conf->mddev->queue);
		} else {
			atomic_dec(&conf->active_queues);
			if (test_and_clear_bit(STRIPE_QUEUE_PREREAD_ACTIVE,
					       &sq->state)) {
				atomic_dec(&conf->preread_active_queues);
				if (atomic_read(&conf->preread_active_queues) <
				    IO_THRESHOLD)
					queue_work(conf->workqueue,
						   &conf->stripe_queue_work);
			}
			if (!test_bit(STRIPE_QUEUE_EXPANDING, &sq->state)) {
				list_add_tail(&sq->list_node,
					      &conf->inactive_q_list);
				wake_up(&conf->wait_for_queue);
			}
		}

		if (io_list) {
			if (test_bit(STRIPE_QUEUE_BIT_DELAY, &sq->state) &&
			    sq->bm_seq - conf->seq_write > 0) {
				list_add_tail(&sq->list_node,
					      &conf->bitmap_q_list);
				blk_plug_device(conf->mddev->queue);
			} else {
				clear_bit(STRIPE_QUEUE_BIT_DELAY, &sq->state);
				list_add_tail(&sq->list_node, io_list);
			}
			queue_work(conf->workqueue, &conf->stripe_queue_work);
		}
	}
}

static void __release_stripe(raid5_conf_t *conf, struct stripe_head *sh)
{
	struct stripe_queue *sq = sh->sq;

	if (atomic_dec_and_test(&sh->count)) {
		BUG_ON(!list_empty(&sh->lru));
		BUG_ON(atomic_read(&conf->active_stripes)==0);
		if (test_bit(STRIPE_HANDLE, &sh->state)) {
			list_add_tail(&sh->lru, &conf->handle_list);
			md_wakeup_thread(conf->mddev->thread);
		} else {
			BUG_ON(sh->ops.pending);
			atomic_dec(&conf->active_stripes);
			if (!test_bit(STRIPE_QUEUE_EXPANDING, &sq->state)) {
				list_add_tail(&sh->lru, &conf->inactive_list);
				wake_up(&conf->wait_for_stripe);
				if (conf->retry_read_aligned)
					md_wakeup_thread(conf->mddev->thread);
			}
			__release_queue(conf, sq);
			sh->sq = NULL;
		}
	}
}

static void release_queue(struct stripe_queue *sq)
{
	raid5_conf_t *conf = sq->raid_conf;
	unsigned long flags;

	spin_lock_irqsave(&conf->device_lock, flags);
	__release_queue(conf, sq);
	spin_unlock_irqrestore(&conf->device_lock, flags);
}

static void release_stripe(struct stripe_head *sh)
{
	raid5_conf_t *conf = sh->sq->raid_conf;
	unsigned long flags;

	spin_lock_irqsave(&conf->device_lock, flags);
	__release_stripe(conf, sh);
	spin_unlock_irqrestore(&conf->device_lock, flags);
}

static inline void remove_hash(struct stripe_head *sh)
{
	pr_debug("remove_hash(), stripe %llu\n",
		(unsigned long long)sh->sector);

	hlist_del_init(&sh->hash);
}

static inline void insert_hash(raid5_conf_t *conf, struct stripe_head *sh)
{
	struct hlist_head *hp = stripe_hash(conf, sh->sector);

	pr_debug("insert_hash(), stripe %llu\n",
		(unsigned long long)sh->sector);

	CHECK_DEVLOCK();
	hlist_add_head(&sh->hash, hp);
}


/* find an idle stripe, make sure it is unhashed, and return it. */
static struct stripe_head *get_free_stripe(raid5_conf_t *conf)
{
	struct stripe_head *sh = NULL;
	struct list_head *first;

	CHECK_DEVLOCK();
	if (list_empty(&conf->inactive_list))
		goto out;
	first = conf->inactive_list.next;
	sh = list_entry(first, struct stripe_head, lru);
	list_del_init(first);
	remove_hash(sh);
	atomic_inc(&conf->active_stripes);
	BUG_ON(sh->sq != NULL);
out:
	return sh;
}

static struct stripe_queue *get_free_queue(raid5_conf_t *conf)
{
	struct stripe_queue *sq = NULL;
	struct list_head *first;

	CHECK_DEVLOCK();
	if (list_empty(&conf->inactive_q_list))
		goto out;
	first = conf->inactive_q_list.next;
	sq = list_entry(first, struct stripe_queue, list_node);
	list_del_init(first);
	rb_erase(&sq->rb_node, &conf->stripe_queue_tree);
	atomic_inc(&conf->active_queues);
out:
	return sq;
}

static void shrink_buffers(struct stripe_head *sh, int num)
{
	struct page *p;
	int i;

	for (i=0; i<num ; i++) {
		p = sh->dev[i].page;
		if (!p)
			continue;
		sh->dev[i].page = NULL;
		put_page(p);
	}
}

static int grow_buffers(struct stripe_head *sh, int num)
{
	int i;

	for (i=0; i<num; i++) {
		struct page *page;

		if (!(page = alloc_page(GFP_KERNEL))) {
			return 1;
		}
		sh->dev[i].page = page;
	}
	return 0;
}

static void raid5_build_block (struct stripe_head *sh, int i);

static void
init_stripe(struct stripe_head *sh, struct stripe_queue *sq, int disks)
{
	raid5_conf_t *conf = sq->raid_conf;
	sector_t sector = sq->sector;
	int i;

	pr_debug("init_stripe called, stripe %llu\n",
		(unsigned long long)sector);

	BUG_ON(atomic_read(&sh->count) != 0);
	BUG_ON(test_bit(STRIPE_HANDLE, &sh->state));
	BUG_ON(sh->ops.pending || sh->ops.ack || sh->ops.complete);

	CHECK_DEVLOCK();

	remove_hash(sh);

	sh->sector = sector;
	sh->state = 0;

	for (i = disks; i--;) {
		struct r5dev *dev = &sh->dev[i];

		if (test_bit(R5_LOCKED, &dev->flags)) {
			printk(KERN_ERR "sector=%llx i=%d %d\n",
			       (unsigned long long)sector, i,
			       test_bit(R5_LOCKED, &dev->flags));
			BUG();
		}
		dev->flags = 0;
		raid5_build_block(sh, i);
	}
	insert_hash(conf, sh);
}

static struct stripe_head *__find_stripe(raid5_conf_t *conf, sector_t sector, int disks)
{
	struct stripe_head *sh;
	struct hlist_node *hn;

	CHECK_DEVLOCK();
	pr_debug("__find_stripe, sector %llu\n", (unsigned long long)sector);
	hlist_for_each_entry(sh, hn, stripe_hash(conf, sector), hash)
		if (sh->sector == sector && disks == disks)
			return sh;
	pr_debug("__stripe %llu not in cache\n", (unsigned long long)sector);
	return NULL;
}

static struct stripe_queue *__find_queue(raid5_conf_t *conf, sector_t sector)
{
	struct rb_node *n = conf->stripe_queue_tree.rb_node;
	struct stripe_queue *sq;

	pr_debug("%s, sector %llu\n", __FUNCTION__, (unsigned long long)sector);
	while (n) {
		sq = rb_entry(n, struct stripe_queue, rb_node);

		if (sector < sq->sector)
			n = n->rb_left;
		else if (sector > sq->sector)
			n = n->rb_right;
		else
			return sq;
	}
	pr_debug("__queue %llu not in tree\n", (unsigned long long)sector);
	return NULL;
}

static struct stripe_queue *
__insert_active_sq(raid5_conf_t *conf, sector_t sector, struct rb_node *node)
{
	struct rb_node **p = &conf->stripe_queue_tree.rb_node;
	struct rb_node *parent = NULL;
	struct stripe_queue *sq;

	while (*p) {
		parent = *p;
		sq = rb_entry(parent, struct stripe_queue, rb_node);

		if (sector < sq->sector)
			p = &(*p)->rb_left;
		else if (sector > sq->sector)
			p = &(*p)->rb_right;
		else
			return sq;
	}

	rb_link_node(node, parent, p);

	return NULL;
}

static struct stripe_queue *
insert_active_sq(raid5_conf_t *conf, sector_t sector, struct rb_node *node)
{
	struct stripe_queue *sq = __insert_active_sq(conf, sector, node);

	if (sq)
		goto out;
	rb_insert_color(node, &conf->stripe_queue_tree);
 out:
	return sq;
}

static sector_t compute_blocknr(raid5_conf_t *conf, int raid_disks,
	sector_t sector, int pd_idx, int i);

static void
pickup_cached_stripe(struct stripe_head *sh, struct stripe_queue *sq)
{
	raid5_conf_t *conf = sq->raid_conf;

	if (atomic_read(&sh->count))
		BUG_ON(!list_empty(&sh->lru));
	else {
		if (!test_bit(STRIPE_HANDLE, &sh->state)) {
			atomic_inc(&conf->active_stripes);
			BUG_ON(sh->sq != NULL);
		}
		if (list_empty(&sh->lru) &&
		    !test_bit(STRIPE_QUEUE_EXPANDING, &sq->state))
			BUG();
		list_del_init(&sh->lru);
	}
}

static void unplug_slaves(mddev_t *mddev);
static void raid5_unplug_device(struct request_queue *q);

static void
__wait_for_inactive_stripe(raid5_conf_t *conf, struct stripe_queue *sq)
{
	conf->inactive_blocked = 1;
	wait_event_lock_irq(conf->wait_for_stripe,
			    (!list_empty(&conf->inactive_list) &&
			     (atomic_read(&conf->active_stripes)
			      < (conf->max_nr_stripes * 3/4) ||
			      !conf->inactive_blocked)),
			    conf->device_lock,
			    raid5_unplug_device(conf->mddev->queue));
	conf->inactive_blocked = 0;
}

static struct stripe_head *
get_active_stripe(struct stripe_queue *sq, int disks, int noblock)
{
	raid5_conf_t *conf = sq->raid_conf;
	sector_t sector = sq->sector;
	struct stripe_head *sh;

	if (test_bit(STRIPE_QUEUE_BIT_DELAY, &sq->state))
		return NULL;

	spin_lock_irq(&conf->device_lock);

	pr_debug("get_stripe, sector %llu\n", (unsigned long long)sq->sector);

	do {
		/* try to get a cached stripe */
		sh = __find_stripe(conf, sector, disks);

		/* try to activate a new stripe */
		if (!sh) {
			if (!conf->inactive_blocked)
				sh = get_free_stripe(conf);
			if (noblock && sh == NULL)
				break;
			if (!sh)
				__wait_for_inactive_stripe(conf, sq);
			else
				init_stripe(sh, sq, disks);
		} else
			pickup_cached_stripe(sh, sq);
	} while (sh == NULL);

	BUG_ON(sq->sector != sector);

	if (sh) {
		atomic_inc(&sh->count);

		if (sh->sq)
			BUG_ON(sh->sq != sq);
		else {
			sh->sq = sq;
			atomic_inc(&sq->count);
		}

		BUG_ON(!list_empty(&sq->list_node));
	}

	spin_unlock_irq(&conf->device_lock);

	return sh;
}

static void init_queue(struct stripe_queue *sq, sector_t sector,
		int disks, int pd_idx)
{
	raid5_conf_t *conf = sq->raid_conf;
	int i;

	pr_debug("%s: %llu -> %llu [%p]\n",
		__FUNCTION__, (unsigned long long) sq->sector,
		(unsigned long long) sector, sq);

	BUG_ON(atomic_read(&sq->count) != 0);
	BUG_ON(io_weight(sq->to_read, disks));
	BUG_ON(io_weight(sq->to_write, disks));
	BUG_ON(io_weight(sq->overwrite, disks));

	sq->sector = sector;
	sq->pd_idx = pd_idx;
	sq->disks = disks;
	sq->state = 0;

	for (i = disks; i--;) {
		struct r5_queue_dev *dev_q = &sq->dev[i];

		if (dev_q->toread || dev_q->read || dev_q->towrite ||
		    dev_q->written) {
			printk(KERN_ERR "sector=%llx i=%d %p %p %p %p\n",
			       (unsigned long long)sq->sector, i, dev_q->toread,
			       dev_q->read, dev_q->towrite, dev_q->written);
			BUG();
		}
		dev_q->sector = compute_blocknr(conf, disks, sector, pd_idx, i);
	}

	sq = insert_active_sq(conf, sector, &sq->rb_node);
	if (unlikely(sq)) {
		printk(KERN_ERR "%s: sq: %p sector: %llu bounced off the "
			"stripe_queue rb_tree\n", __FUNCTION__, sq,
			(unsigned long long) sq->sector);
		BUG();
	}
}

static void __wait_for_inactive_queue(raid5_conf_t *conf)
{
	conf->inactive_queue_blocked = 1;
	wait_event_lock_irq(conf->wait_for_queue,
			    !list_empty(&conf->inactive_q_list) &&
			    (atomic_read(&conf->active_queues)
			     < conf->max_nr_stripes *
			     STRIPE_QUEUE_SIZE * 7/8 ||
			    !conf->inactive_queue_blocked),
			    conf->device_lock,
			    /* nothing */);
	conf->inactive_queue_blocked = 0;
}


static struct stripe_queue *
get_active_queue(raid5_conf_t *conf, sector_t sector, int disks, int pd_idx,
		  int noblock)
{
	struct stripe_queue *sq;

	pr_debug("%s, sector %llu\n", __FUNCTION__,
		(unsigned long long)sector);

	spin_lock_irq(&conf->device_lock);

	do {
		wait_event_lock_irq(conf->wait_for_queue,
				    conf->quiesce == 0,
				    conf->device_lock,
				    /* nothing */);
		sq = __find_queue(conf, sector);
		if (!sq) {
			if (!conf->inactive_queue_blocked)
				sq = get_free_queue(conf);
			if (noblock && sq == NULL)
				break;
			if (!sq)
				__wait_for_inactive_queue(conf);
			else
				init_queue(sq, sector, disks, pd_idx);
		} else {
			if (atomic_read(&sq->count))
				BUG_ON(!list_empty(&sq->list_node));
			else if (io_weight(sq->to_write, disks) == 0 &&
				 io_weight(sq->to_read, disks) == 0)
				atomic_inc(&conf->active_queues);

			list_del_init(&sq->list_node);
		}
	} while (sq == NULL);

	if (sq)
		atomic_inc(&sq->count);

	spin_unlock_irq(&conf->device_lock);

	return sq;
}


/* test_and_ack_op() ensures that we only dequeue an operation once */
#define test_and_ack_op(op, pend) \
do {							\
	if (test_bit(op, &sh->ops.pending) &&		\
		!test_bit(op, &sh->ops.complete)) {	\
		if (test_and_set_bit(op, &sh->ops.ack)) \
			clear_bit(op, &pend);		\
		else					\
			ack++;				\
	} else						\
		clear_bit(op, &pend);			\
} while (0)

/* find new work to run, do not resubmit work that is already
 * in flight
 */
static unsigned long get_stripe_work(struct stripe_head *sh)
{
	unsigned long pending;
	int ack = 0;

	pending = sh->ops.pending;

	test_and_ack_op(STRIPE_OP_BIOFILL, pending);
	test_and_ack_op(STRIPE_OP_COMPUTE_BLK, pending);
	test_and_ack_op(STRIPE_OP_PREXOR, pending);
	test_and_ack_op(STRIPE_OP_BIODRAIN, pending);
	test_and_ack_op(STRIPE_OP_POSTXOR, pending);
	test_and_ack_op(STRIPE_OP_CHECK, pending);
	if (test_and_clear_bit(STRIPE_OP_IO, &sh->ops.pending))
		ack++;

	sh->ops.count -= ack;
	if (unlikely(sh->ops.count < 0)) {
		printk(KERN_ERR "pending: %#lx ops.pending: %#lx ops.ack: %#lx "
			"ops.complete: %#lx\n", pending, sh->ops.pending,
			sh->ops.ack, sh->ops.complete);
		BUG();
	}

	return pending;
}

static void
raid5_end_read_request(struct bio *bi, int error);
static void
raid5_end_write_request(struct bio *bi, int error);

static void ops_run_io(struct stripe_head *sh)
{
	struct stripe_queue *sq = sh->sq;
	raid5_conf_t *conf = sq->raid_conf;
	int i, disks = sq->disks;

	might_sleep();

	for (i = disks; i--;) {
		int rw;
		struct bio *bi;
		mdk_rdev_t *rdev;
		if (test_and_clear_bit(R5_Wantwrite, &sh->dev[i].flags))
			rw = WRITE;
		else if (test_and_clear_bit(R5_Wantread, &sh->dev[i].flags))
			rw = READ;
		else
			continue;

		bi = &sh->dev[i].req;

		bi->bi_rw = rw;
		if (rw == WRITE)
			bi->bi_end_io = raid5_end_write_request;
		else
			bi->bi_end_io = raid5_end_read_request;

		rcu_read_lock();
		rdev = rcu_dereference(conf->disks[i].rdev);
		if (rdev && test_bit(Faulty, &rdev->flags))
			rdev = NULL;
		if (rdev)
			atomic_inc(&rdev->nr_pending);
		rcu_read_unlock();

		if (rdev) {
			if (test_bit(STRIPE_SYNCING, &sh->state) ||
				test_bit(STRIPE_EXPAND_SOURCE, &sh->state) ||
				test_bit(STRIPE_EXPAND_READY, &sh->state))
				md_sync_acct(rdev->bdev, STRIPE_SECTORS);

			bi->bi_bdev = rdev->bdev;
			pr_debug("%s: for %llu schedule op %ld on disc %d\n",
				__FUNCTION__, (unsigned long long)sh->sector,
				bi->bi_rw, i);
			atomic_inc(&sh->count);
			bi->bi_sector = sh->sector + rdev->data_offset;
			bi->bi_flags = 1 << BIO_UPTODATE;
			bi->bi_vcnt = 1;
			bi->bi_max_vecs = 1;
			bi->bi_idx = 0;
			bi->bi_io_vec = &sh->dev[i].vec;
			bi->bi_io_vec[0].bv_len = STRIPE_SIZE;
			bi->bi_io_vec[0].bv_offset = 0;
			bi->bi_size = STRIPE_SIZE;
			bi->bi_next = NULL;
			if (rw == WRITE &&
			    test_bit(R5_ReWrite, &sh->dev[i].flags))
				atomic_add(STRIPE_SECTORS,
					&rdev->corrected_errors);
			generic_make_request(bi);
		} else {
			if (rw == WRITE)
				set_bit(STRIPE_DEGRADED, &sh->state);
			pr_debug("skip op %ld on disc %d for sector %llu\n",
				bi->bi_rw, i, (unsigned long long)sh->sector);
			clear_bit(R5_LOCKED, &sh->dev[i].flags);
			set_bit(STRIPE_HANDLE, &sh->state);
		}
	}
}

static struct dma_async_tx_descriptor *
async_copy_data(int frombio, struct bio *bio, struct page *page,
	sector_t sector, struct dma_async_tx_descriptor *tx)
{
	struct bio_vec *bvl;
	struct page *bio_page;
	int i;
	int page_offset;

	if (bio->bi_sector >= sector)
		page_offset = (signed)(bio->bi_sector - sector) * 512;
	else
		page_offset = (signed)(sector - bio->bi_sector) * -512;
	bio_for_each_segment(bvl, bio, i) {
		int len = bio_iovec_idx(bio, i)->bv_len;
		int clen;
		int b_offset = 0;

		if (page_offset < 0) {
			b_offset = -page_offset;
			page_offset += b_offset;
			len -= b_offset;
		}

		if (len > 0 && page_offset + len > STRIPE_SIZE)
			clen = STRIPE_SIZE - page_offset;
		else
			clen = len;

		if (clen > 0) {
			b_offset += bio_iovec_idx(bio, i)->bv_offset;
			bio_page = bio_iovec_idx(bio, i)->bv_page;
			if (frombio)
				tx = async_memcpy(page, bio_page, page_offset,
					b_offset, clen,
					ASYNC_TX_DEP_ACK,
					tx, NULL, NULL);
			else
				tx = async_memcpy(bio_page, page, b_offset,
					page_offset, clen,
					ASYNC_TX_DEP_ACK,
					tx, NULL, NULL);
		}
		if (clen < len) /* hit end of page */
			break;
		page_offset +=  len;
	}

	return tx;
}

static void ops_complete_biofill(void *stripe_head_ref)
{
	struct stripe_head *sh = stripe_head_ref;
	struct bio *return_bi = NULL;
	struct stripe_queue *sq = sh->sq;
	raid5_conf_t *conf = sq->raid_conf;
	int i;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	/* clear completed biofills */
	for (i = sq->disks; i--;) {
		struct r5dev *dev = &sh->dev[i];
		struct r5_queue_dev *dev_q = &sq->dev[i];

		/* acknowledge completion of a biofill operation */
		/* and check if we need to reply to a read request,
		 * new R5_Wantfill requests are held off until
		 * !test_bit(STRIPE_OP_BIOFILL, &sh->ops.pending)
		 */
		if (test_and_clear_bit(R5_Wantfill, &dev->flags)) {
			struct bio *rbi, *rbi2;

			/* The access to dev_q->read is outside of the
			 * spin_lock_irq(&conf->device_lock), but is protected
			 * by the STRIPE_OP_BIOFILL pending bit
			 */
			BUG_ON(!dev_q->read);
			rbi = dev_q->read;
			dev_q->read = NULL;
			while (rbi && rbi->bi_sector <
				dev_q->sector + STRIPE_SECTORS) {
				rbi2 = r5_next_bio(rbi, dev_q->sector);
				spin_lock_irq(&conf->device_lock);
				if (--rbi->bi_phys_segments == 0) {
					rbi->bi_next = return_bi;
					return_bi = rbi;
				}
				spin_unlock_irq(&conf->device_lock);
				rbi = rbi2;
			}
		}
	}
	set_bit(STRIPE_OP_BIOFILL, &sh->ops.complete);

	return_io(return_bi);

	set_bit(STRIPE_HANDLE, &sh->state);
	release_stripe(sh);
}

static void ops_run_biofill(struct stripe_head *sh)
{
	struct dma_async_tx_descriptor *tx = NULL;
	struct stripe_queue *sq = sh->sq;
	raid5_conf_t *conf = sq->raid_conf;
	int i;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	for (i = sq->disks; i--;) {
		struct r5dev *dev = &sh->dev[i];
		struct r5_queue_dev *dev_q = &sq->dev[i];

		if (test_bit(R5_Wantfill, &dev->flags)) {
			struct bio *rbi;
			spin_lock_irq(&conf->device_lock);
			dev_q->read = rbi = dev_q->toread;
			dev_q->toread = NULL;
			clear_bit(i, sq->to_read);
			spin_unlock_irq(&conf->device_lock);
			while (rbi && rbi->bi_sector <
				dev_q->sector + STRIPE_SECTORS) {
				tx = async_copy_data(0, rbi, dev->page,
					dev_q->sector, tx);
				rbi = r5_next_bio(rbi, dev_q->sector);
			}
		}
	}

	atomic_inc(&sh->count);
	async_trigger_callback(ASYNC_TX_DEP_ACK | ASYNC_TX_ACK, tx,
		ops_complete_biofill, sh);
}

static void ops_complete_compute(void *stripe_head_ref)
{
	struct stripe_head *sh = stripe_head_ref;
	int target, i;
	struct r5dev *tgt;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	/* mark the computed target(s) as uptodate */
	for (i = 0; i < 2; i++) {
		target = (!i) ? sh->ops.target : sh->ops.target2;
		if (target < 0)
			continue;
		tgt = &sh->dev[target];
		set_bit(R5_UPTODATE, &tgt->flags);
		BUG_ON(!test_bit(R5_Wantcompute, &tgt->flags));
		clear_bit(R5_Wantcompute, &tgt->flags);
	}

	set_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete);
	set_bit(STRIPE_HANDLE, &sh->state);
	release_stripe(sh);
}

static struct dma_async_tx_descriptor *
ops_run_compute5(struct stripe_head *sh, unsigned long pending)
{
	/* kernel stack size limits the total number of disks */
	int disks = sh->sq->disks;
	struct page *xor_srcs[disks];
	int target = sh->ops.target;
	struct r5dev *tgt = &sh->dev[target];
	struct page *xor_dest = tgt->page;
	int count = 0;
	struct dma_async_tx_descriptor *tx;
	int i;

	pr_debug("%s: stripe %llu block: %d\n",
		__FUNCTION__, (unsigned long long)sh->sector, target);
	BUG_ON(!test_bit(R5_Wantcompute, &tgt->flags));

	for (i = disks; i--; )
		if (i != target)
			xor_srcs[count++] = sh->dev[i].page;

	atomic_inc(&sh->count);

	if (unlikely(count == 1))
		tx = async_memcpy(xor_dest, xor_srcs[0], 0, 0, STRIPE_SIZE,
			0, NULL, ops_complete_compute, sh);
	else
		tx = async_xor(xor_dest, xor_srcs, 0, count, STRIPE_SIZE,
			ASYNC_TX_XOR_ZERO_DST, NULL,
			ops_complete_compute, sh);

	/* ack now if postxor is not set to be run */
	if (tx && !test_bit(STRIPE_OP_POSTXOR, &pending))
		async_tx_ack(tx);

	return tx;
}

static struct dma_async_tx_descriptor *
ops_run_compute6_1(struct stripe_head *sh, unsigned long pending)
{
	/* kernel stack size limits the total number of disks */
	int disks = sh->sq->disks;
	int target = sh->ops.target < 0 ? sh->ops.target2 : sh->ops.target;
	struct r5dev *tgt = &sh->dev[target];
	struct page *dest = sh->dev[target].page;
	struct page *srcs[disks];
	int count = 0;
	int pd_idx = sh->sq->pd_idx;
	int qd_idx = raid6_next_disk(pd_idx, disks);
	struct dma_async_tx_descriptor *tx;
	int i;

	pr_debug("%s: stripe %llu block: %d\n",
		__FUNCTION__, (unsigned long long)sh->sector, target);
	BUG_ON(!test_bit(R5_Wantcompute, &tgt->flags));

	atomic_inc(&sh->count);

	if (target == qd_idx) {
		/* We are actually computing the Q drive*/
		for (i = disks; i-- ; ) {
			if (i != target && i != pd_idx && i != qd_idx)
				srcs[count++] = sh->dev[i].page;
		}
		/* Synchronous calculations need two destination pages,
		 * so use P-page too
		 */
		tx = async_pqxor(sh->dev[pd_idx].page, dest,
			srcs, (char *)raid6_gfexp,
			0, count, STRIPE_SIZE,
			ASYNC_TX_XOR_ZERO_DST, NULL,
			ops_complete_compute, sh);
	} else {
		/* Compute any data- or p-drive using XOR */
		for (i = disks; i-- ; ) {
			if (i != target && i != qd_idx)
				srcs[count++] = sh->dev[i].page;
		}

		tx = async_xor(dest, srcs, 0, count, STRIPE_SIZE,
			ASYNC_TX_XOR_ZERO_DST, NULL,
			ops_complete_compute, sh);
	}

	/* ack now if postxor is not set to be run */
	if (tx && !test_bit(STRIPE_OP_POSTXOR, &pending))
		async_tx_ack(tx);

	return tx;
}

static struct dma_async_tx_descriptor *
ops_run_compute6_2(struct stripe_head *sh, unsigned long pending)
{
	/* kernel stack size limits the total number of disks */
	int disks = sh->sq->disks;
	int target = sh->ops.target;
	int target2 = sh->ops.target2;
	struct r5dev *tgt = &sh->dev[target];
	struct r5dev *tgt2 = &sh->dev[target2];
	struct page *srcs[disks];
	int count = 0;
	int pd_idx = sh->sq->pd_idx;
	int qd_idx = raid6_next_disk(pd_idx, disks);
	int d0_idx = raid6_next_disk(qd_idx, disks);
	struct dma_async_tx_descriptor *tx;
	int i, faila, failb;

	/* faila and failb are disk numbers relative to d0_idx;
	 * pd_idx become disks-2 and qd_idx become disks-1.
	 */
	faila = (target < d0_idx) ? target + (disks - d0_idx) :
			target - d0_idx;
	failb = (target2 < d0_idx) ? target2 + (disks - d0_idx) :
			target2 - d0_idx;

	BUG_ON(faila == failb);
	if ( failb < faila ) {
		int tmp = faila;
		faila = failb;
		failb = tmp;
	}

	pr_debug("%s: stripe %llu block1: %d block2: %d\n",
		__FUNCTION__, (unsigned long long)sh->sector, target, target2);
	BUG_ON(!test_bit(R5_Wantcompute, &tgt->flags));
	BUG_ON(!test_bit(R5_Wantcompute, &tgt2->flags));

	atomic_inc(&sh->count);

	if ( failb == disks-1 ) {
		/* Q disk is one of the missing disks */
		i = d0_idx;
		do {
			if (i != target && i != target2) {
				srcs[count++] = sh->dev[i].page;
				if (!test_bit(R5_UPTODATE, &sh->dev[i].flags))
					pr_debug("%s with missing block %d/%d\n",
						__FUNCTION__, count, i);
			}
			i = raid6_next_disk(i, disks);
		} while ( i != d0_idx );

		if ( faila == disks - 2 ) {
			/* Missing P+Q, just recompute */
			tx = async_pqxor(sh->dev[pd_idx].page,
			    sh->dev[qd_idx].page, srcs, (char *)raid6_gfexp,
			    0, count, STRIPE_SIZE, ASYNC_TX_XOR_ZERO_DST, NULL,
			    ops_complete_compute, sh);
		} else {
			/* Missing D+Q; recompute D from P */
			tx = async_xor(sh->dev[qd_idx == target ? target2 :
			    target].page, srcs, 0, count, STRIPE_SIZE,
			    ASYNC_TX_XOR_ZERO_DST, NULL,
			    ops_complete_compute, sh);
			/* recompute Q then? */
		}

		/* ack now if postxor is not set to be run */
		if (tx && !test_bit(STRIPE_OP_POSTXOR, &pending))
			async_tx_ack(tx);
		return tx;
	}

	/* We're missing D+P or D+D */
	i = d0_idx;
	do {
		srcs[count++] = sh->dev[i].page;
		i = raid6_next_disk(i, disks);
		if (i != target && i != target2 &&
		    !test_bit(R5_UPTODATE, &sh->dev[i].flags))
			pr_debug("%s with missing block %d/%d\n", __FUNCTION__, count, i);
	} while ( i != d0_idx );

	if ( failb == disks - 2 ) {
		/* We're missing D+P. */
		tx = async_r6_dp_recov(disks, STRIPE_SIZE, faila, srcs,
				0, NULL, ops_complete_compute, sh);
	} else {
		/* We're missing D+D. */
		tx = async_r6_dd_recov(disks, STRIPE_SIZE, faila, failb, srcs,
				0, NULL, ops_complete_compute, sh);
	}

	if (tx && !test_bit(STRIPE_OP_POSTXOR, &pending))
		async_tx_ack(tx);

	return tx;
}

static void ops_complete_prexor(void *stripe_head_ref)
{
	struct stripe_head *sh = stripe_head_ref;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	set_bit(STRIPE_OP_PREXOR, &sh->ops.complete);
}

static struct dma_async_tx_descriptor *
ops_run_prexor(struct stripe_head *sh, struct dma_async_tx_descriptor *tx)
{
	/* kernel stack size limits the total number of disks */
	struct stripe_queue *sq = sh->sq;
	int disks = sq->disks;
	struct page *xor_srcs[disks];
	int count = 0, pd_idx = sq->pd_idx, i;

	/* existing parity data subtracted */
	struct page *xor_dest = xor_srcs[count++] = sh->dev[pd_idx].page;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	for (i = disks; i--; ) {
		struct r5dev *dev = &sh->dev[i];
		struct r5_queue_dev *dev_q = &sq->dev[i];
		/* Only process blocks that are known to be uptodate */
		if (dev_q->towrite && test_bit(R5_Wantprexor, &dev->flags))
			xor_srcs[count++] = dev->page;
	}

	tx = async_xor(xor_dest, xor_srcs, 0, count, STRIPE_SIZE,
		ASYNC_TX_DEP_ACK | ASYNC_TX_XOR_DROP_DST, tx,
		ops_complete_prexor, sh);

	return tx;
}

static struct dma_async_tx_descriptor *
ops_run_biodrain(struct stripe_head *sh, struct dma_async_tx_descriptor *tx,
		 unsigned long pending)
{
	struct stripe_queue *sq = sh->sq;
	int disks = sq->disks;
	int pd_idx = sq->pd_idx;
#ifdef CONFIG_MD_RAID_SKIP_BIO_COPY
	int qd_idx = raid6_next_disk(pd_idx, disks);
	int fswrite = 1;
#endif
	int i;

	/* check if prexor is active which means only process blocks
	 * that are part of a read-modify-write (Wantprexor)
	 */
	int prexor = test_bit(STRIPE_OP_PREXOR, &pending);

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

#ifdef CONFIG_MD_RAID_SKIP_BIO_COPY
	/* initially assume that the operation is a full-stripe write*/
	for (i = disks; i-- ;) {
		struct r5dev *dev;

		if (unlikely(i == pd_idx))
			continue;
		if ((sq->raid_conf->level == 6) && unlikely(i == qd_idx))
			continue;
		if (unlikely(!sq->dev[i].towrite || prexor))
			goto do_copy;
		dev = &sh->dev[i];
		if ((test_bit(R5_OVERWRITE, &dev->flags)) &&
		    !r5_next_bio(sq->dev[i].towrite, sq->dev[i].sector)) {
			/* now check if there is only one bio_vec within
			 * the bio covers the sh->dev[i]
			 */
			struct bio *pbio = sq->dev[i].towrite;
			struct bio_vec *bvl;
			int found = 0;
			int bvec_page = pbio->bi_sector << 9, k;
			int dev_page = sq->dev[i].sector << 9;

			/* search for the bio_vec that covers dev[i].page */
			bio_for_each_segment(bvl, pbio, k) {
				if (bvec_page == dev_page &&
				    bio_iovec_idx(pbio,k)->bv_len ==
				      STRIPE_SIZE) {
					/* found the vector which covers the
					 * strip fully
					 */
					found = 1;
					break;
				}
				bvec_page += bio_iovec_idx(pbio,k)->bv_len;
			}

			if (found) {
				/* save the direct pointer to buffer */
				BUG_ON(dev->dpage);
				dev->dpage = bio_iovec_idx(pbio,k)->bv_page;
				continue;
			}
		}

do_copy:
		/* come here in two cases:
		 * - the dev[i] is not covered fully with the bio;
		 * - there are more than one bios cover the dev[i].
		 * in both cases do copy from bio to dev[i].page
		 */
		pr_debug("%s: do copy because of disk %d\n", __FUNCTION__, i);
		do {
			/* restore dpages set */
			sh->dev[i].dpage = NULL;
		} while (i++ != disks);
		fswrite = 0;
		break;
	}

	if (fswrite) {
		/* won't add new txs right now, so run ops currently pending */
		async_tx_issue_pending_all();
	}
#endif

	for (i = disks; i--; ) {
		struct r5dev *dev = &sh->dev[i];
		struct r5_queue_dev *dev_q = &sq->dev[i];
		struct bio *chosen;
		int towrite;

		towrite = 0;
		if (prexor) { /* rmw */
			if (dev_q->towrite &&
			    test_bit(R5_Wantprexor, &dev->flags))
				towrite = 1;
		} else { /* rcw */
			if (sq->raid_conf->level == 6) {
				if (i != raid6_next_disk(pd_idx, disks) &&
				    i != pd_idx && dev_q->towrite &&
				    test_bit(R5_LOCKED, &dev->flags))
					towrite = 1;
			} else
			if (i != pd_idx && dev_q->towrite &&
				test_bit(R5_LOCKED, &dev->flags))
				towrite = 1;
		}

		if (towrite) {
			struct bio *wbi;

			spin_lock(&sq->lock);
			chosen = dev_q->towrite;
			dev_q->towrite = NULL;
			clear_bit(i, sq->to_write);
			BUG_ON(dev_q->written);
			wbi = dev_q->written = chosen;
			spin_unlock(&sq->lock);

#ifdef CONFIG_MD_RAID_SKIP_BIO_COPY
			if (fswrite) {
				/* just update dev bio vec pointer */
				dev->vec.bv_page = dev->dpage;
				continue;
			}
#endif
			/* schedule the copy op(s) */
			while (wbi && wbi->bi_sector <
				dev_q->sector + STRIPE_SECTORS) {
				tx = async_copy_data(1, wbi, dev->page,
					dev_q->sector, tx);
				wbi = r5_next_bio(wbi, dev_q->sector);
			}
		}
	}

	return tx;
}

static void ops_complete_postxor(void *stripe_head_ref)
{
	struct stripe_head *sh = stripe_head_ref;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	set_bit(STRIPE_OP_POSTXOR, &sh->ops.complete);
	set_bit(STRIPE_HANDLE, &sh->state);
	release_stripe(sh);
}

static void ops_complete_write(void *stripe_head_ref)
{
	struct stripe_head *sh = stripe_head_ref;
	struct stripe_queue *sq = sh->sq;
	int disks = sq->disks, i;
	int pd_idx = sq->pd_idx;
	int qd_idx = (sq->raid_conf->level != 6) ? -1 :
		raid6_next_disk(pd_idx, disks);

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	for (i = disks; i--; ) {
		struct r5dev *dev = &sh->dev[i];
		struct r5_queue_dev *dev_q = &sq->dev[i];

		if (dev_q->written || i == pd_idx || i == qd_idx)
			set_bit(R5_UPTODATE, &dev->flags);
	}

	set_bit(STRIPE_OP_BIODRAIN, &sh->ops.complete);
	set_bit(STRIPE_OP_POSTXOR, &sh->ops.complete);

	set_bit(STRIPE_HANDLE, &sh->state);
	release_stripe(sh);
}

static void
ops_run_postxor(struct stripe_head *sh, struct dma_async_tx_descriptor *tx,
		unsigned long pending)
{
	/* kernel stack size limits the total number of disks */
	struct stripe_queue *sq = sh->sq;
	int disks = sq->disks;
	struct page *xor_srcs[disks];

	int count = 0;
	int pd_idx = sq->pd_idx;
	int qd_idx = (sq->raid_conf->level != 6) ? -1 :
		raid6_next_disk(pd_idx, disks);
	int i;
	struct page *xor_dest;
	struct page *q_dest = NULL;
	int prexor = test_bit(STRIPE_OP_PREXOR, &pending);
	unsigned long flags;
	dma_async_tx_callback callback;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	/* check if prexor is active which means only process blocks
	 * that are part of a read-modify-write (written)
	 */
	if (prexor) {
		xor_dest = xor_srcs[count++] = sh->dev[pd_idx].page;
		BUG_ON(!(qd_idx < 0));
		for (i = disks; i--; ) {
			struct r5dev *dev = &sh->dev[i];
			struct r5_queue_dev *dev_q = &sq->dev[i];

			if (dev_q->written)
				xor_srcs[count++] = dev->dpage ?
					dev->dpage : dev->page;
		}
	} else {
		xor_dest = sh->dev[pd_idx].page;
		q_dest = (qd_idx < 0) ? NULL : sh->dev[qd_idx].page;
		for (i = disks; i--; ) {
			struct r5dev *dev = &sh->dev[i];
			if (i != pd_idx)
				xor_srcs[count++] = dev->dpage ?
					dev->dpage : dev->page;
		}
	}

	/* check whether this postxor is part of a write */
	callback = test_bit(STRIPE_OP_BIODRAIN, &pending) ?
		ops_complete_write : ops_complete_postxor;

	/* 1/ if we prexor'd then the dest is reused as a source
	 * 2/ if we did not prexor then we are redoing the parity
	 * set ASYNC_TX_XOR_DROP_DST and ASYNC_TX_XOR_ZERO_DST
	 * for the synchronous xor case
	 */
	flags = ASYNC_TX_DEP_ACK | ASYNC_TX_ACK |
		(prexor ? ASYNC_TX_XOR_DROP_DST : ASYNC_TX_XOR_ZERO_DST);

	atomic_inc(&sh->count);

	if (unlikely(count == 1)) {
		BUG_ON(!(qd_idx < 0));
		flags &= ~(ASYNC_TX_XOR_DROP_DST | ASYNC_TX_XOR_ZERO_DST);
		tx = async_memcpy(xor_dest, xor_srcs[0], 0, 0, STRIPE_SIZE,
			flags, tx, callback, sh);
	} else {
		if (qd_idx < 0)
			tx = async_xor(xor_dest, xor_srcs, 0, count,
				STRIPE_SIZE, flags, tx, callback, sh);
		else
			tx = async_pqxor(xor_dest, q_dest, xor_srcs,
				(char *)raid6_gfexp, 0, count, STRIPE_SIZE,
				flags, tx, callback, sh);
	}
}

static void ops_complete_check(void *stripe_head_ref)
{
	struct stripe_head *sh = stripe_head_ref;
	int pd_idx = sh->sq->pd_idx;
	int qd_idx = (sh->sq->raid_conf->level != 6) ? -1 :
		raid6_next_disk(pd_idx, sh->sq->disks);

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	if (test_and_clear_bit(STRIPE_OP_MOD_DMA_CHECK, &sh->ops.pending)) {
		if (sh->ops.zero_sum_result == 0)
			set_bit(R5_UPTODATE, &sh->dev[pd_idx].flags);
		if (!(qd_idx < 0) && sh->ops.zero_qsum_result == 0)
			set_bit(R5_UPTODATE, &sh->dev[qd_idx].flags);
	}

	set_bit(STRIPE_OP_CHECK, &sh->ops.complete);
	set_bit(STRIPE_HANDLE, &sh->state);
	release_stripe(sh);
}

static void ops_run_check5(struct stripe_head *sh)
{
	/* kernel stack size limits the total number of disks */
	struct stripe_queue *sq = sh->sq;
	int disks = sq->disks;
	struct page *xor_srcs[disks];
	struct dma_async_tx_descriptor *tx;

	int count = 0;
	int pd_idx = sq->pd_idx;
	int i;
	struct page *xor_dest = xor_srcs[count++] = sh->dev[pd_idx].page;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	for (i = disks; i--; ) {
		struct r5dev *dev = &sh->dev[i];
		if (i != pd_idx)
			xor_srcs[count++] = dev->page;
	}

	tx = async_xor_zero_sum(xor_dest, xor_srcs, 0, count, STRIPE_SIZE,
		&sh->ops.zero_sum_result, 0, NULL, NULL, NULL);

	if (tx)
		set_bit(STRIPE_OP_MOD_DMA_CHECK, &sh->ops.pending);
	else
		clear_bit(STRIPE_OP_MOD_DMA_CHECK, &sh->ops.pending);

	atomic_inc(&sh->count);
	tx = async_trigger_callback(ASYNC_TX_DEP_ACK | ASYNC_TX_ACK, tx,
		ops_complete_check, sh);
}

static void ops_run_check6(struct stripe_head *sh)
{
	/* kernel stack size limits the total number of disks */
	struct stripe_queue *sq = sh->sq;
	int disks = sq->disks;
	struct page *srcs[disks];
	struct dma_async_tx_descriptor *tx;

	int count = 0;
	int pd_idx = sq->pd_idx;
	int qd_idx = raid6_next_disk(pd_idx, disks);
	int i;

	struct page *qxor_dest = srcs[count++] = sh->dev[qd_idx].page;
	struct page *pxor_dest = srcs[count++] = sh->dev[pd_idx].page;

	pr_debug("%s: stripe %llu\n", __FUNCTION__,
		(unsigned long long)sh->sector);

	for (i = 0; i < disks; i++) {
		if (i != pd_idx && i != qd_idx)
			srcs[count++] = sh->dev[i].page;
	}

	if (test_bit(STRIPE_OP_CHECK_PP, &sh->ops.pending) &&
	    test_bit(STRIPE_OP_CHECK_QP, &sh->ops.pending)) {
		/* check both P and Q */
		pr_debug("%s: check both P&Q\n", __FUNCTION__);
		tx = async_pqxor_zero_sum(pxor_dest, qxor_dest,
			srcs, (char *)raid6_gfexp,
			0, count, STRIPE_SIZE,
			&sh->ops.zero_sum_result, &sh->ops.zero_qsum_result,
			0, NULL, NULL, NULL);
	} else if (test_bit(STRIPE_OP_CHECK_QP, &sh->ops.pending)) {
		/* check Q only */
		srcs[1] = NULL;
		pr_debug("%s: check Q\n", __FUNCTION__);
		tx = async_pqxor_zero_sum(NULL, qxor_dest,
			srcs, (char *)raid6_gfexp,
			0, count, STRIPE_SIZE,
			&sh->ops.zero_sum_result, &sh->ops.zero_qsum_result,
			0, NULL, NULL, NULL);
	} else {
		/* check P only */
		srcs[0] = NULL;
		tx = async_xor_zero_sum(pxor_dest,
			&srcs[1], 0, count-1, STRIPE_SIZE,
			&sh->ops.zero_sum_result,
			0, NULL, NULL, NULL);
	}

	if (tx)
		set_bit(STRIPE_OP_MOD_DMA_CHECK, &sh->ops.pending);
	else
		clear_bit(STRIPE_OP_MOD_DMA_CHECK, &sh->ops.pending);

	atomic_inc(&sh->count);
	tx = async_trigger_callback(ASYNC_TX_DEP_ACK | ASYNC_TX_ACK, tx,
		ops_complete_check, sh);
}

static void raid_run_ops(struct stripe_head *sh, unsigned long pending)
{
	struct stripe_queue *sq = sh->sq;
	int overlap_clear = 0;
	int disks = sq->disks;
	int i;
	int level = sq->raid_conf->level;
	struct dma_async_tx_descriptor *tx = NULL;

	if (test_bit(STRIPE_OP_BIOFILL, &pending)) {
		ops_run_biofill(sh);
		overlap_clear++;
	}

	if (test_bit(STRIPE_OP_COMPUTE_BLK, &pending)) {
		if (level != 6)
			tx = ops_run_compute5(sh, pending);
		else {
			if (sh->ops.target2 < 0 || sh->ops.target < 0)
				tx = ops_run_compute6_1(sh, pending);
			else
				tx = ops_run_compute6_2(sh, pending);
		}
	}

	if (test_bit(STRIPE_OP_PREXOR, &pending))
		tx = ops_run_prexor(sh, tx);

	if (test_bit(STRIPE_OP_BIODRAIN, &pending)) {
		tx = ops_run_biodrain(sh, tx, pending);
		overlap_clear++;
	}

	if (test_bit(STRIPE_OP_POSTXOR, &pending))
		ops_run_postxor(sh, tx, pending);

	if (test_bit(STRIPE_OP_CHECK, &pending)) {
		if (level != 6)
			ops_run_check5(sh);
		else
			ops_run_check6(sh);
	}

	if (test_bit(STRIPE_OP_IO, &pending))
		ops_run_io(sh);

	if (overlap_clear) {
		for (i = disks; i--;)
			if (test_and_clear_bit(i, sq->overlap))
				wake_up(&sq->raid_conf->wait_for_overlap);
	}
}

static int grow_one_stripe(raid5_conf_t *conf)
{
	struct stripe_head *sh;
	struct stripe_queue init_sq = { .raid_conf = conf };

	sh = kmem_cache_alloc(conf->sh_slab_cache, GFP_KERNEL);
	if (!sh)
		return 0;
	memset(sh, 0, sizeof(*sh) + (conf->raid_disks-1)*sizeof(struct r5dev));

	if (grow_buffers(sh, conf->raid_disks)) {
		shrink_buffers(sh, conf->raid_disks);
		kmem_cache_free(conf->sh_slab_cache, sh);
		return 0;
	}

	/* we just created an active stripe so... */
	atomic_set(&sh->count, 1);
	atomic_inc(&conf->active_stripes);
	INIT_LIST_HEAD(&sh->lru);
	atomic_set(&init_sq.count, 2); /* bypass release_queue() */
	sh->sq = &init_sq;
	release_stripe(sh);

	return 1;
}

static int grow_one_queue(raid5_conf_t *conf)
{
	struct stripe_queue *sq;
	int disks = conf->raid_disks;
	void *weight_map;
	sq = kmem_cache_alloc(conf->sq_slab_cache, GFP_KERNEL);
	if (!sq)
		return 0;
	memset(sq, 0, (sizeof(*sq)+(disks-1) * sizeof(struct r5_queue_dev)) +
		r5_io_weight_size(disks) + r5_io_weight_size(disks) +
		r5_io_weight_size(disks) + r5_io_weight_size(disks));

	/* set the queue weight bitmaps to the free space at the end of sq */
	weight_map = ((void *) sq) + offsetof(typeof(*sq), dev) +
			sizeof(struct r5_queue_dev) * disks;
	sq->to_read = weight_map;
	weight_map += r5_io_weight_size(disks);
	sq->to_write = weight_map;
	weight_map += r5_io_weight_size(disks);
	sq->overwrite = weight_map;
	weight_map += r5_io_weight_size(disks);
	sq->overlap = weight_map;

	spin_lock_init(&sq->lock);
	sq->sector = MaxSector;
	sq->raid_conf = conf;
	sq->disks = disks;

	/* we just created an active queue so... */
	atomic_set(&sq->count, 1);
	atomic_inc(&conf->active_queues);
	INIT_LIST_HEAD(&sq->list_node);
	RB_CLEAR_NODE(&sq->rb_node);
	release_queue(sq);

	return 1;
}

static int grow_stripes(raid5_conf_t *conf, int num)
{
	struct kmem_cache *sc;
	int devs = conf->raid_disks, num_q = num * STRIPE_QUEUE_SIZE;

	sprintf(conf->sh_cache_name[0], "raid5-%s", mdname(conf->mddev));
	sprintf(conf->sh_cache_name[1], "raid5-%s-alt", mdname(conf->mddev));
	sprintf(conf->sq_cache_name[0], "raid5q-%s", mdname(conf->mddev));
	sprintf(conf->sq_cache_name[1], "raid5q-%s-alt", mdname(conf->mddev));

	conf->active_name = 0;
	sc = kmem_cache_create(conf->sh_cache_name[conf->active_name],
			       sizeof(struct stripe_head)+(devs-1)*sizeof(struct r5dev),
			       0, 0, NULL);
	if (!sc)
		return 1;
	conf->sh_slab_cache = sc;
	conf->pool_size = devs;
	while (num--)
		if (!grow_one_stripe(conf))
			return 1;

	sc = kmem_cache_create(conf->sq_cache_name[conf->active_name],
			       (sizeof(struct stripe_queue)+(devs-1) *
				sizeof(struct r5_queue_dev)) +
				r5_io_weight_size(devs) +
				r5_io_weight_size(devs) +
				r5_io_weight_size(devs) +
				r5_io_weight_size(devs), 0, 0, NULL);
	if (!sc)
		return 1;

	conf->sq_slab_cache = sc;
	while (num_q--)
		if (!grow_one_queue(conf))
			return 1;

	return 0;
}

#ifdef CONFIG_MD_RAID5_RESHAPE
static int resize_stripes(raid5_conf_t *conf, int newsize)
{
	/* Make all the stripes able to hold 'newsize' devices.
	 * New slots in each stripe get 'page' set to a new page.
	 *
	 * This happens in stages:
	 * 1/ create a new kmem_cache and allocate the required number of
	 *    stripe_heads.
	 * 2/ gather all the old stripe_heads and tranfer the pages across
	 *    to the new stripe_heads.  This will have the side effect of
	 *    freezing the array as once all stripe_heads have been collected,
	 *    no IO will be possible.  Old stripe heads are freed once their
	 *    pages have been transferred over, and the old kmem_cache is
	 *    freed when all stripes are done.
	 * 3/ reallocate conf->disks to be suitable bigger.  If this fails,
	 *    we simple return a failre status - no need to clean anything up.
	 * 4/ allocate new pages for the new slots in the new stripe_heads.
	 *    If this fails, we don't bother trying the shrink the
	 *    stripe_heads down again, we just leave them as they are.
	 *    As each stripe_head is processed the new one is released into
	 *    active service.
	 *
	 * Once step2 is started, we cannot afford to wait for a write,
	 * so we use GFP_NOIO allocations.
	 */
	struct stripe_head *osh, *nsh;
	struct stripe_queue *osq, *nsq;
	LIST_HEAD(newstripes);
	LIST_HEAD(newqueues);
	struct disk_info *ndisks;
	int err = 0;
	struct kmem_cache *sc, *sc_q;
	int i, j;

	if (newsize <= conf->pool_size)
		return 0; /* never bother to shrink */

	md_allow_write(conf->mddev);

	/* Step 1 */
	sc = kmem_cache_create(conf->sh_cache_name[1-conf->active_name],
			       sizeof(struct stripe_head)+(newsize-1)*sizeof(struct r5dev),
			       0, 0, NULL);
	if (!sc)
		return -ENOMEM;

	sc_q = kmem_cache_create(conf->sq_cache_name[1-conf->active_name],
			       (sizeof(struct stripe_queue)+(newsize-1) *
				sizeof(struct r5_queue_dev)) +
				r5_io_weight_size(newsize) +
				r5_io_weight_size(newsize) +
				r5_io_weight_size(newsize) +
				r5_io_weight_size(newsize),
				0, 0, NULL);

	if (!sc_q) {
		kmem_cache_destroy(sc);
		return -ENOMEM;
	}

	for (i = conf->max_nr_stripes; i; i--) {
		struct stripe_queue *nsq_per_sh[STRIPE_QUEUE_SIZE];

		nsh = kmem_cache_alloc(sc, GFP_KERNEL);
		if (!nsh)
			break;

		/* allocate STRIPE_QUEUE_SIZE queues per stripe */
		for (j = 0; j < ARRAY_SIZE(nsq_per_sh); j++)
			nsq_per_sh[j] = kmem_cache_alloc(sc_q, GFP_KERNEL);

		for (j = 0; j < ARRAY_SIZE(nsq_per_sh); j++)
			if (!nsq_per_sh[j])
				break;

		if (j <= ARRAY_SIZE(nsq_per_sh)) {
			kmem_cache_free(sc, nsh);
			do
				if (nsq_per_sh[j])
					kmem_cache_free(sc_q, nsq_per_sh[j]);
			while (--j >= 0);
			break;
		}

		memset(nsh, 0, sizeof(*nsh) + (newsize-1)*sizeof(struct r5dev));
		list_add(&nsh->lru, &newstripes);

		for (j = 0; j < ARRAY_SIZE(nsq_per_sh); j++) {
			void *weight_map;
			nsq = nsq_per_sh[j];
			memset(nsq, 0, (sizeof(*nsq)+(newsize-1) *
				sizeof(struct r5_queue_dev)) +
				r5_io_weight_size(newsize) +
				r5_io_weight_size(newsize) +
				r5_io_weight_size(newsize) +
				r5_io_weight_size(newsize));
			/* set the queue weight bitmaps to the free space at
			 * the end of nsq
			 */
			weight_map = ((void *) nsq) +
					offsetof(typeof(*nsq), dev) +
					sizeof(struct r5_queue_dev) * newsize;
			nsq->to_read = weight_map;
			weight_map += r5_io_weight_size(newsize);
			nsq->to_write = weight_map;
			weight_map += r5_io_weight_size(newsize);
			nsq->overwrite = weight_map;
			weight_map += r5_io_weight_size(newsize);
			nsq->overlap = weight_map;

			nsq->raid_conf = conf;
			RB_CLEAR_NODE(&nsq->rb_node);
			spin_lock_init(&nsq->lock);
			list_add(&nsq->list_node, &newqueues);
		}
	}
	if (i) {
		/* didn't get enough, give up */
		while (!list_empty(&newstripes)) {
			nsh = list_entry(newstripes.next, struct stripe_head, lru);
			list_del(&nsh->lru);
			kmem_cache_free(sc, nsh);
		}
		while (!list_empty(&newqueues)) {
			nsq = list_entry(newqueues.next,
					 struct stripe_queue,
					 list_node);
			list_del(&nsh->lru);
			kmem_cache_free(sc_q, nsq);
		}
		kmem_cache_destroy(sc_q);
		kmem_cache_destroy(sc);
		return -ENOMEM;
	}
	/* Step 2 - Must use GFP_NOIO now.
	 * OK, we have enough stripes, start collecting inactive
	 * stripes and copying them over
	 */
	list_for_each_entry(nsq, &newqueues, list_node) {
		spin_lock_irq(&conf->device_lock);
		wait_event_lock_irq(conf->wait_for_queue,
				    !list_empty(&conf->inactive_q_list),
				    conf->device_lock,
				    unplug_slaves(conf->mddev));
		osq = get_free_queue(conf);
		spin_unlock_irq(&conf->device_lock);
		atomic_set(&nsq->count, 1);
		kmem_cache_free(conf->sq_slab_cache, osq);
	}
	kmem_cache_destroy(conf->sq_slab_cache);

	list_for_each_entry(nsh, &newstripes, lru) {
		spin_lock_irq(&conf->device_lock);
		wait_event_lock_irq(conf->wait_for_stripe,
				    !list_empty(&conf->inactive_list),
				    conf->device_lock,
				    unplug_slaves(conf->mddev)
			);
		osh = get_free_stripe(conf);
		spin_unlock_irq(&conf->device_lock);
		atomic_set(&nsh->count, 1);
		for(i=0; i<conf->pool_size; i++)
			nsh->dev[i].page = osh->dev[i].page;
		for( ; i<newsize; i++)
			nsh->dev[i].page = NULL;
		kmem_cache_free(conf->sh_slab_cache, osh);
	}
	kmem_cache_destroy(conf->sh_slab_cache);

	/* Step 3.
	 * At this point, we are holding all the stripes so the array
	 * is completely stalled, so now is a good time to resize
	 * conf->disks.
	 */
	ndisks = kzalloc(newsize * sizeof(struct disk_info), GFP_NOIO);
	if (ndisks) {
		for (i=0; i<conf->raid_disks; i++)
			ndisks[i] = conf->disks[i];
		kfree(conf->disks);
		conf->disks = ndisks;
	} else
		err = -ENOMEM;

	/* Step 4, return new stripes to service */
	while (!list_empty(&newstripes)) {
		struct stripe_queue init_sq = { .raid_conf = conf };
		nsh = list_entry(newstripes.next, struct stripe_head, lru);
		list_del_init(&nsh->lru);
		for (i=conf->raid_disks; i < newsize; i++)
			if (nsh->dev[i].page == NULL) {
				struct page *p = alloc_page(GFP_NOIO);
				nsh->dev[i].page = p;
				if (!p)
					err = -ENOMEM;
			}
		atomic_set(&init_sq.count, 2); /* bypass release_queue() */
		nsh->sq = &init_sq;
		release_stripe(nsh);
	}

	/* Step 4a, return new queues to service */
	while (!list_empty(&newqueues)) {
		nsq = list_entry(newqueues.next, struct stripe_queue,
				 list_node);
		list_del_init(&nsq->list_node);
		release_queue(nsq);
	}

	/* critical section pass, GFP_NOIO no longer needed */

	conf->sh_slab_cache = sc;
	conf->sq_slab_cache = sc_q;
	conf->active_name = 1-conf->active_name;
	conf->pool_size = newsize;
	return err;
}
#endif

static int drop_one_stripe(raid5_conf_t *conf)
{
	struct stripe_head *sh;

	spin_lock_irq(&conf->device_lock);
	sh = get_free_stripe(conf);
	spin_unlock_irq(&conf->device_lock);
	if (!sh)
		return 0;
	BUG_ON(atomic_read(&sh->count));
	shrink_buffers(sh, conf->pool_size);
	kmem_cache_free(conf->sh_slab_cache, sh);
	atomic_dec(&conf->active_stripes);
	return 1;
}

static int drop_one_queue(raid5_conf_t *conf)
{
	struct stripe_queue *sq;

	spin_lock_irq(&conf->device_lock);
	sq = get_free_queue(conf);
	spin_unlock_irq(&conf->device_lock);
	if (!sq)
		return 0;
	kmem_cache_free(conf->sq_slab_cache, sq);
	atomic_dec(&conf->active_queues);
	return 1;
}

static void shrink_stripes(raid5_conf_t *conf)
{
	while (drop_one_stripe(conf))
		;

	while (drop_one_queue(conf))
		;

	if (conf->sh_slab_cache)
		kmem_cache_destroy(conf->sh_slab_cache);
	conf->sh_slab_cache = NULL;

	if (conf->sq_slab_cache)
		kmem_cache_destroy(conf->sq_slab_cache);
	conf->sq_slab_cache = NULL;
}

static void raid5_end_read_request(struct bio * bi, int error)
{
 	struct stripe_head *sh = bi->bi_private;
	struct stripe_queue *sq = sh->sq;
	raid5_conf_t *conf = sq->raid_conf;
	int disks = sq->disks;
	int uptodate = test_bit(BIO_UPTODATE, &bi->bi_flags);
	int i;
	char b[BDEVNAME_SIZE];
	mdk_rdev_t *rdev;


	for (i=0 ; i<disks; i++)
		if (bi == &sh->dev[i].req)
			break;

	pr_debug("end_read_request %llu/%d, count: %d, uptodate %d.\n",
		(unsigned long long)sh->sector, i, atomic_read(&sh->count),
		uptodate);
	if (i == disks) {
		BUG();
		return;
	}

	if (uptodate) {
		set_bit(R5_UPTODATE, &sh->dev[i].flags);
		if (test_bit(R5_ReadError, &sh->dev[i].flags)) {
			rdev = conf->disks[i].rdev;
			printk(KERN_INFO "raid5:%s: read error corrected (%lu sectors at %llu on %s)\n",
			       mdname(conf->mddev), STRIPE_SECTORS,
			       (unsigned long long)(sh->sector + rdev->data_offset),
			       bdevname(rdev->bdev, b));
			clear_bit(R5_ReadError, &sh->dev[i].flags);
			clear_bit(R5_ReWrite, &sh->dev[i].flags);
		}
		if (atomic_read(&conf->disks[i].rdev->read_errors))
			atomic_set(&conf->disks[i].rdev->read_errors, 0);
	} else {
		const char *bdn = bdevname(conf->disks[i].rdev->bdev, b);
		int retry = 0;
		rdev = conf->disks[i].rdev;

		clear_bit(R5_UPTODATE, &sh->dev[i].flags);
		atomic_inc(&rdev->read_errors);
		if (conf->mddev->degraded)
			printk(KERN_WARNING "raid5:%s: read error not correctable (sector %llu on %s).\n",
			       mdname(conf->mddev),
			       (unsigned long long)(sh->sector + rdev->data_offset),
			       bdn);
		else if (test_bit(R5_ReWrite, &sh->dev[i].flags))
			/* Oh, no!!! */
			printk(KERN_WARNING "raid5:%s: read error NOT corrected!! (sector %llu on %s).\n",
			       mdname(conf->mddev),
			       (unsigned long long)(sh->sector + rdev->data_offset),
			       bdn);
		else if (atomic_read(&rdev->read_errors)
			 > conf->max_nr_stripes)
			printk(KERN_WARNING
			       "raid5:%s: Too many read errors, failing device %s.\n",
			       mdname(conf->mddev), bdn);
		else
			retry = 1;
		if (retry)
			set_bit(R5_ReadError, &sh->dev[i].flags);
		else {
			clear_bit(R5_ReadError, &sh->dev[i].flags);
			clear_bit(R5_ReWrite, &sh->dev[i].flags);
			md_error(conf->mddev, rdev);
		}
	}
	rdev_dec_pending(conf->disks[i].rdev, conf->mddev);
	clear_bit(R5_LOCKED, &sh->dev[i].flags);
	set_bit(STRIPE_HANDLE, &sh->state);
	release_stripe(sh);
}

static void raid5_end_write_request (struct bio *bi, int error)
{
 	struct stripe_head *sh = bi->bi_private;
	struct stripe_queue *sq = sh->sq;
	raid5_conf_t *conf = sq->raid_conf;
	int disks = sq->disks;
	int uptodate = test_bit(BIO_UPTODATE, &bi->bi_flags);
	int i;

	for (i=0 ; i<disks; i++)
		if (bi == &sh->dev[i].req)
			break;

	pr_debug("end_write_request %llu/%d, count %d, uptodate: %d.\n",
		(unsigned long long)sh->sector, i, atomic_read(&sh->count),
		uptodate);
	if (i == disks) {
		BUG();
		return;
	}

	if (!uptodate)
		md_error(conf->mddev, conf->disks[i].rdev);

	rdev_dec_pending(conf->disks[i].rdev, conf->mddev);

	clear_bit(R5_LOCKED, &sh->dev[i].flags);
	set_bit(STRIPE_HANDLE, &sh->state);
	release_stripe(sh);
}

static void raid5_build_block (struct stripe_head *sh, int i)
{
	struct r5dev *dev = &sh->dev[i];

	bio_init(&dev->req);
	dev->req.bi_io_vec = &dev->vec;
	dev->req.bi_vcnt++;
	dev->req.bi_max_vecs++;
	dev->vec.bv_page = dev->page;
	dev->vec.bv_len = STRIPE_SIZE;
	dev->vec.bv_offset = 0;

	dev->req.bi_sector = sh->sector;
	dev->req.bi_private = sh;
}

static void error(mddev_t *mddev, mdk_rdev_t *rdev)
{
	char b[BDEVNAME_SIZE];
	raid5_conf_t *conf = (raid5_conf_t *) mddev->private;
	pr_debug("raid5: error called\n");

	if (!test_bit(Faulty, &rdev->flags)) {
		set_bit(MD_CHANGE_DEVS, &mddev->flags);
		if (test_and_clear_bit(In_sync, &rdev->flags)) {
			unsigned long flags;
			spin_lock_irqsave(&conf->device_lock, flags);
			mddev->degraded++;
			spin_unlock_irqrestore(&conf->device_lock, flags);
			/*
			 * if recovery was running, make sure it aborts.
			 */
			set_bit(MD_RECOVERY_ERR, &mddev->recovery);
		}
		set_bit(Faulty, &rdev->flags);
		printk (KERN_ALERT
			"raid5: Disk failure on %s, disabling device."
			" Operation continuing on %d devices\n",
			bdevname(rdev->bdev,b), conf->raid_disks - mddev->degraded);
	}
}

/*
 * Input: a 'big' sector number,
 * Output: index of the data and parity disk, and the sector # in them.
 */
static sector_t raid5_compute_sector(sector_t r_sector, unsigned int raid_disks,
			unsigned int data_disks, unsigned int * dd_idx,
			unsigned int * pd_idx, raid5_conf_t *conf)
{
	long stripe;
	unsigned long chunk_number;
	unsigned int chunk_offset;
	sector_t new_sector;
	int sectors_per_chunk = conf->chunk_size >> 9;

	/* First compute the information on this sector */

	/*
	 * Compute the chunk number and the sector offset inside the chunk
	 */
	chunk_offset = sector_div(r_sector, sectors_per_chunk);
	chunk_number = r_sector;
	BUG_ON(r_sector != chunk_number);

	/*
	 * Compute the stripe number
	 */
	stripe = chunk_number / data_disks;

	/*
	 * Compute the data disk and parity disk indexes inside the stripe
	 */
	*dd_idx = chunk_number % data_disks;

	/*
	 * Select the parity disk based on the user selected algorithm.
	 */
	switch(conf->level) {
	case 4:
		*pd_idx = data_disks;
		break;
	case 5:
		switch (conf->algorithm) {
		case ALGORITHM_LEFT_ASYMMETRIC:
			*pd_idx = data_disks - stripe % raid_disks;
			if (*dd_idx >= *pd_idx)
				(*dd_idx)++;
			break;
		case ALGORITHM_RIGHT_ASYMMETRIC:
			*pd_idx = stripe % raid_disks;
			if (*dd_idx >= *pd_idx)
				(*dd_idx)++;
			break;
		case ALGORITHM_LEFT_SYMMETRIC:
			*pd_idx = data_disks - stripe % raid_disks;
			*dd_idx = (*pd_idx + 1 + *dd_idx) % raid_disks;
			break;
		case ALGORITHM_RIGHT_SYMMETRIC:
			*pd_idx = stripe % raid_disks;
			*dd_idx = (*pd_idx + 1 + *dd_idx) % raid_disks;
			break;
		default:
			printk(KERN_ERR "raid5: unsupported algorithm %d\n",
				conf->algorithm);
		}
		break;
	case 6:

		/**** FIX THIS ****/
		switch (conf->algorithm) {
		case ALGORITHM_LEFT_ASYMMETRIC:
			*pd_idx = raid_disks - 1 - (stripe % raid_disks);
			if (*pd_idx == raid_disks-1)
				(*dd_idx)++; 	/* Q D D D P */
			else if (*dd_idx >= *pd_idx)
				(*dd_idx) += 2; /* D D P Q D */
			break;
		case ALGORITHM_RIGHT_ASYMMETRIC:
			*pd_idx = stripe % raid_disks;
			if (*pd_idx == raid_disks-1)
				(*dd_idx)++; 	/* Q D D D P */
			else if (*dd_idx >= *pd_idx)
				(*dd_idx) += 2; /* D D P Q D */
			break;
		case ALGORITHM_LEFT_SYMMETRIC:
			*pd_idx = raid_disks - 1 - (stripe % raid_disks);
			*dd_idx = (*pd_idx + 2 + *dd_idx) % raid_disks;
			break;
		case ALGORITHM_RIGHT_SYMMETRIC:
			*pd_idx = stripe % raid_disks;
			*dd_idx = (*pd_idx + 2 + *dd_idx) % raid_disks;
			break;
		default:
			printk (KERN_CRIT "raid6: unsupported algorithm %d\n",
				conf->algorithm);
		}
		break;
	}

	/*
	 * Finally, compute the new sector number
	 */
	new_sector = (sector_t)stripe * sectors_per_chunk + chunk_offset;
	return new_sector;
}


static sector_t
compute_blocknr(raid5_conf_t *conf, int raid_disks, sector_t sector,
	int pd_idx, int i)
{
	int data_disks = raid_disks - conf->max_degraded;
	sector_t new_sector = sector, check;
	int sectors_per_chunk = conf->chunk_size >> 9;
	sector_t stripe;
	int chunk_offset;
	int chunk_number, dummy1, dummy2, dd_idx = i;
	sector_t r_sector;


	chunk_offset = sector_div(new_sector, sectors_per_chunk);
	stripe = new_sector;
	BUG_ON(new_sector != stripe);

	if (i == pd_idx)
		return 0;
	switch(conf->level) {
	case 4: break;
	case 5:
		switch (conf->algorithm) {
		case ALGORITHM_LEFT_ASYMMETRIC:
		case ALGORITHM_RIGHT_ASYMMETRIC:
			if (i > pd_idx)
				i--;
			break;
		case ALGORITHM_LEFT_SYMMETRIC:
		case ALGORITHM_RIGHT_SYMMETRIC:
			if (i < pd_idx)
				i += raid_disks;
			i -= (pd_idx + 1);
			break;
		default:
			printk(KERN_ERR "raid5: unsupported algorithm %d\n",
			       conf->algorithm);
		}
		break;
	case 6:
		if (i == raid6_next_disk(pd_idx, raid_disks))
			return 0; /* It is the Q disk */
		switch (conf->algorithm) {
		case ALGORITHM_LEFT_ASYMMETRIC:
		case ALGORITHM_RIGHT_ASYMMETRIC:
			if (pd_idx == raid_disks-1)
				i--;	/* Q D D D P */
			else if (i > pd_idx)
				i -= 2; /* D D P Q D */
			break;
		case ALGORITHM_LEFT_SYMMETRIC:
		case ALGORITHM_RIGHT_SYMMETRIC:
			if (pd_idx == raid_disks-1)
				i--; /* Q D D D P */
			else {
				/* D D P Q D */
				if (i < pd_idx)
					i += raid_disks;
				i -= (pd_idx + 2);
			}
			break;
		default:
			printk (KERN_CRIT "raid6: unsupported algorithm %d\n",
				conf->algorithm);
		}
		break;
	}

	chunk_number = stripe * data_disks + i;
	r_sector = (sector_t)chunk_number * sectors_per_chunk + chunk_offset;

	check = raid5_compute_sector (r_sector, raid_disks, data_disks, &dummy1, &dummy2, conf);
	if (check != sector || dummy1 != dd_idx || dummy2 != pd_idx) {
		printk(KERN_ERR "compute_blocknr: map not correct\n");
		return 0;
	}
	return r_sector;
}

static int
handle_write_operations(struct stripe_head *sh, int rcw, int expand)
{
	int locked = 0;
	struct stripe_queue *sq = sh->sq;
	int i, pd_idx = sq->pd_idx, disks = sq->disks;
	int level = sq->raid_conf->level;

	if (rcw) {
		/* if we are not expanding this is a proper write request, and
		 * there will be bios with new data to be drained into the
		 * stripe cache
		 */
		if (!expand) {
			set_bit(STRIPE_OP_BIODRAIN, &sh->ops.pending);
			sh->ops.count++;
		}

		set_bit(STRIPE_OP_POSTXOR, &sh->ops.pending);
		sh->ops.count++;

		for (i = disks; i--; ) {
			struct r5dev *dev = &sh->dev[i];
			struct r5_queue_dev *dev_q = &sq->dev[i];

			if (dev_q->towrite) {
				set_bit(R5_LOCKED, &dev->flags);
				if (!expand)
					clear_bit(R5_UPTODATE, &dev->flags);
				locked++;
			}
		}
	} else {
		BUG_ON(level == 6);
		BUG_ON(!(test_bit(R5_UPTODATE, &sh->dev[pd_idx].flags) ||
			test_bit(R5_Wantcompute, &sh->dev[pd_idx].flags)));

		set_bit(STRIPE_OP_PREXOR, &sh->ops.pending);
		set_bit(STRIPE_OP_BIODRAIN, &sh->ops.pending);
		set_bit(STRIPE_OP_POSTXOR, &sh->ops.pending);

		sh->ops.count += 3;

		for (i = disks; i--; ) {
			struct r5dev *dev = &sh->dev[i];
			struct r5_queue_dev *dev_q = &sq->dev[i];

			if (i == pd_idx)
				continue;

			/* For a read-modify write there may be blocks that are
			 * locked for reading while others are ready to be
			 * written so we distinguish these blocks by the
			 * R5_Wantprexor bit
			 */
			if (dev_q->towrite &&
			    (test_bit(R5_UPTODATE, &dev->flags) ||
			    test_bit(R5_Wantcompute, &dev->flags))) {
				set_bit(R5_Wantprexor, &dev->flags);
				set_bit(R5_LOCKED, &dev->flags);
				clear_bit(R5_UPTODATE, &dev->flags);
				locked++;
			}
		}
	}

	/* keep the parity disks locked while asynchronous operations
	 * are in flight
	 */
	set_bit(R5_LOCKED, &sh->dev[pd_idx].flags);
	clear_bit(R5_UPTODATE, &sh->dev[pd_idx].flags);
	locked++;

	if (level == 6) {
		int qd_idx = raid6_next_disk(pd_idx, disks);
		set_bit(R5_LOCKED, &sh->dev[qd_idx].flags);
		clear_bit(R5_UPTODATE, &sh->dev[qd_idx].flags);
		locked++;
	}

	pr_debug("%s: stripe %llu locked: %d pending: %lx\n",
		__FUNCTION__, (unsigned long long)sh->sector,
		locked, sh->ops.pending);

	return locked;
}

/*
 * Each stripe/dev can have one or more bion attached.
 * toread/towrite point to the first in a chain.
 * The bi_next chain must be in order.
 */
static int add_queue_bio(struct stripe_queue *sq, struct bio *bi, int dd_idx,
			  int forwrite)
{
	struct bio **bip;
	raid5_conf_t *conf = sq->raid_conf;
	int firstwrite=0;

	pr_debug("adding bio (%llu) to queue (%llu)\n",
		(unsigned long long)bi->bi_sector,
		(unsigned long long)sq->sector);

	spin_lock(&sq->lock);
	spin_lock_irq(&conf->device_lock);
	if (forwrite) {
		bip = &sq->dev[dd_idx].towrite;
		set_bit(dd_idx, sq->to_write);
		if (*bip == NULL && sq->dev[dd_idx].written == NULL)
			firstwrite = 1;
	} else {
		bip = &sq->dev[dd_idx].toread;
		set_bit(dd_idx, sq->to_read);
	}

	while (*bip && (*bip)->bi_sector < bi->bi_sector) {
		if ((*bip)->bi_sector + ((*bip)->bi_size >> 9) > bi->bi_sector)
			goto overlap;
		bip = & (*bip)->bi_next;
	}
	if (*bip && (*bip)->bi_sector < bi->bi_sector + ((bi->bi_size)>>9))
		goto overlap;

	BUG_ON(*bip && bi->bi_next && (*bip) != bi->bi_next);
	if (*bip)
		bi->bi_next = *bip;
	*bip = bi;
	bi->bi_phys_segments ++;

	spin_unlock_irq(&conf->device_lock);
	spin_unlock(&sq->lock);

	pr_debug("added bi b#%llu to stripe s#%llu, disk %d.\n",
		(unsigned long long)bi->bi_sector,
		(unsigned long long)sq->sector, dd_idx);

	if (conf->mddev->bitmap && firstwrite) {
		bitmap_startwrite(conf->mddev->bitmap, sq->sector,
				  STRIPE_SECTORS, 0);
		sq->bm_seq = conf->seq_flush+1;
		set_bit(STRIPE_QUEUE_BIT_DELAY, &sq->state);
	}

	if (forwrite) {
		/* check if page is covered */
		sector_t sector = sq->dev[dd_idx].sector;

		for (bi = sq->dev[dd_idx].towrite;
		     sector < sq->dev[dd_idx].sector + STRIPE_SECTORS &&
			     bi && bi->bi_sector <= sector;
		     bi = r5_next_bio(bi, sq->dev[dd_idx].sector)) {
			if (bi->bi_sector + (bi->bi_size>>9) >= sector)
				sector = bi->bi_sector + (bi->bi_size>>9);
		}
		if (sector >= sq->dev[dd_idx].sector + STRIPE_SECTORS)
			set_bit(dd_idx, sq->overwrite);
	}

	return 1;

 overlap:
	set_bit(dd_idx, sq->overlap);
	spin_unlock_irq(&conf->device_lock);
	spin_unlock(&sq->lock);
	return 0;
}

static void end_reshape(raid5_conf_t *conf);

static int stripe_to_pdidx(sector_t stripe, raid5_conf_t *conf, int disks)
{
	int sectors_per_chunk = conf->chunk_size >> 9;
	int pd_idx, dd_idx;
	int chunk_offset = sector_div(stripe, sectors_per_chunk);

	raid5_compute_sector(stripe * (disks - conf->max_degraded)
			     *sectors_per_chunk + chunk_offset,
			     disks, disks - conf->max_degraded,
			     &dd_idx, &pd_idx, conf);
	return pd_idx;
}

static void
handle_requests_to_failed_array(raid5_conf_t *conf, struct stripe_head *sh,
				struct stripe_head_state *s, int disks,
				struct bio **return_bi)
{
	int i;
	struct stripe_queue *sq = sh->sq;

	for (i = disks; i--; ) {
		struct bio *bi;
		int bitmap_end = 0;

		if (test_bit(R5_ReadError, &sh->dev[i].flags)) {
			mdk_rdev_t *rdev;
			rcu_read_lock();
			rdev = rcu_dereference(conf->disks[i].rdev);
			if (rdev && test_bit(In_sync, &rdev->flags))
				/* multiple read failures in one stripe */
				md_error(conf->mddev, rdev);
			rcu_read_unlock();
		}
		spin_lock_irq(&conf->device_lock);
		/* fail all writes first */
		bi = sq->dev[i].towrite;
		sq->dev[i].towrite = NULL;
		clear_bit(i, sq->to_write);
		if (bi) {
			s->to_write--;
			bitmap_end = 1;
		}

		if (test_and_clear_bit(i, sq->overlap))
			wake_up(&conf->wait_for_overlap);

		while (bi && bi->bi_sector <
			sq->dev[i].sector + STRIPE_SECTORS) {
			struct bio *nextbi = r5_next_bio(bi, sq->dev[i].sector);
			clear_bit(BIO_UPTODATE, &bi->bi_flags);
			if (--bi->bi_phys_segments == 0) {
				md_write_end(conf->mddev);
				bi->bi_next = *return_bi;
				*return_bi = bi;
			}
			bi = nextbi;
		}
		/* and fail all 'written' */
		bi = sq->dev[i].written;
		sq->dev[i].written = NULL;
		if (bi) bitmap_end = 1;
		while (bi && bi->bi_sector <
		       sq->dev[i].sector + STRIPE_SECTORS) {
			struct bio *bi2 = r5_next_bio(bi, sq->dev[i].sector);
			clear_bit(BIO_UPTODATE, &bi->bi_flags);
			if (--bi->bi_phys_segments == 0) {
				md_write_end(conf->mddev);
				bi->bi_next = *return_bi;
				*return_bi = bi;
			}
			bi = bi2;
		}

		/* fail any reads if this device is non-operational and
		 * the data has not reached the cache yet.
		 */
		if (!test_bit(R5_Wantfill, &sh->dev[i].flags) &&
		    (!test_bit(R5_Insync, &sh->dev[i].flags) ||
		      test_bit(R5_ReadError, &sh->dev[i].flags))) {
			bi = sq->dev[i].toread;
			sq->dev[i].toread = NULL;
			clear_bit(i, sq->to_read);
			if (test_and_clear_bit(i, sq->overlap))
				wake_up(&conf->wait_for_overlap);
			if (bi) s->to_read--;
			while (bi && bi->bi_sector <
			       sq->dev[i].sector + STRIPE_SECTORS) {
				struct bio *nextbi =
					r5_next_bio(bi, sq->dev[i].sector);
				clear_bit(BIO_UPTODATE, &bi->bi_flags);
				if (--bi->bi_phys_segments == 0) {
					bi->bi_next = *return_bi;
					*return_bi = bi;
				}
				bi = nextbi;
			}
		}
		spin_unlock_irq(&conf->device_lock);
		if (bitmap_end)
			bitmap_endwrite(conf->mddev->bitmap, sh->sector,
					STRIPE_SECTORS, 0, 0);
	}

}

/* __handle_issuing_new_read_requests5 - returns 0 if there are no more disks
 * to process
 */
static int __handle_issuing_new_read_requests5(struct stripe_head *sh,
			struct stripe_head_state *s, int disk_idx, int disks)
{
	struct stripe_queue *sq = sh->sq;
	struct r5dev *dev = &sh->dev[disk_idx];
	struct r5_queue_dev *dev_q = &sq->dev[disk_idx];
	struct r5dev *failed_dev = &sh->dev[s->failed_num];
	struct r5_queue_dev *failed_dev_q = &sq->dev[s->failed_num];

	/* don't schedule compute operations or reads on the parity block while
	 * a check is in flight
	 */
	if ((disk_idx == sq->pd_idx) &&
	     test_bit(STRIPE_OP_CHECK, &sh->ops.pending))
		return ~0;

	/* is the data in this block needed, and can we get it? */
	if (!test_bit(R5_LOCKED, &dev->flags) &&
	    !test_bit(R5_UPTODATE, &dev->flags) && (dev_q->toread ||
	    (dev_q->towrite && !test_bit(R5_OVERWRITE, &dev->flags)) ||
	     s->syncing || s->expanding || (s->failed &&
	     (failed_dev_q->toread || (failed_dev_q->towrite &&
	     !test_bit(R5_OVERWRITE, &failed_dev->flags)
	     ))))) {
		/* 1/ We would like to get this block, possibly by computing it,
		 * but we might not be able to.
		 *
		 * 2/ Since parity check operations potentially make the parity
		 * block !uptodate it will need to be refreshed before any
		 * compute operations on data disks are scheduled.
		 *
		 * 3/ We hold off parity block re-reads until check operations
		 * have quiesced.
		 */
		if ((s->uptodate == disks - 1) &&
		    !test_bit(STRIPE_OP_CHECK, &sh->ops.pending)) {
			set_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
			set_bit(R5_Wantcompute, &dev->flags);
			sh->ops.target = disk_idx;
			sh->ops.target2 = -1; /* no second target */
			s->req_compute = 1;
			sh->ops.count++;
			/* Careful: from this point on 'uptodate' is in the eye
			 * of raid_run_ops which services 'compute' operations
			 * before writes. R5_Wantcompute flags a block that will
			 * be R5_UPTODATE by the time it is needed for a
			 * subsequent operation.
			 */
			s->uptodate++;
			return 0; /* uptodate + compute == disks */
		} else if ((s->uptodate < disks - 1) &&
			test_bit(R5_Insync, &dev->flags)) {
			/* Note: we hold off compute operations while checks are
			 * in flight, but we still prefer 'compute' over 'read'
			 * hence we only read if (uptodate < * disks-1)
			 */
			set_bit(R5_LOCKED, &dev->flags);
			set_bit(R5_Wantread, &dev->flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
			s->locked++;
			pr_debug("Reading block %d (sync=%d)\n", disk_idx,
				s->syncing);
		}
	}

	return ~0;
}

static void handle_issuing_new_read_requests5(struct stripe_head *sh,
			struct stripe_head_state *s, int disks)
{
	int i;

	/* Clear completed compute operations.  Parity recovery
	 * (STRIPE_OP_MOD_REPAIR_PD) implies a write-back which is handled
	 * later on in this routine
	 */
	if (test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete) &&
		!test_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending)) {
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.ack);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
	}

	/* look for blocks to read/compute, skip this if a compute
	 * is already in flight, or if the stripe contents are in the
	 * midst of changing due to a write
	 */
	if (!test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending) &&
		!test_bit(STRIPE_OP_PREXOR, &sh->ops.pending) &&
		!test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending)) {
		for (i = disks; i--; )
			if (__handle_issuing_new_read_requests5(
				sh, s, i, disks) == 0)
				break;
	}
	set_bit(STRIPE_HANDLE, &sh->state);
}

/* __handle_issuing_new_read_requests6 - returns 0 if there are no more disks
 * to process
 */
static int __handle_issuing_new_read_requests6(struct stripe_head *sh,
			struct stripe_head_state *s, struct r6_state *r6s,
			int disk_idx, int disks)
{
	struct stripe_queue *sq = sh->sq;
	struct r5dev *dev = &sh->dev[disk_idx];
	struct r5_queue_dev *dev_q = &sq->dev[disk_idx];
	struct r5dev *failed_dev[2] = { &sh->dev[r6s->failed_num[0]],
					&sh->dev[r6s->failed_num[1]]};
	struct r5_queue_dev *failed_dev_q[2] = { &sq->dev[r6s->failed_num[0]],
						 &sq->dev[r6s->failed_num[1]]};

	/* don't schedule compute operations or reads on
	 * the parity blocks while a check is in flight
	 */
	if ((disk_idx == sq->pd_idx || disk_idx == r6s->qd_idx) &&
	    test_bit(STRIPE_OP_CHECK, &sh->ops.pending))
		return ~0;

	/* is the data in this block needed, and can we get it? */
	if (!test_bit(R5_LOCKED, &dev->flags) &&
	    !test_bit(R5_UPTODATE, &dev->flags) && (dev_q->toread ||
	    (dev_q->towrite && !test_bit(R5_OVERWRITE, &dev->flags)) ||
	     s->syncing || s->expanding ||
	     (s->failed >= 1 && (failed_dev_q[0]->toread ||
	      (failed_dev_q[0]->towrite &&
	      !test_bit(R5_OVERWRITE,&failed_dev[0]->flags)))) ||
	     (s->failed >= 2 && (failed_dev_q[1]->toread ||
	      (failed_dev_q[1]->towrite &&
	      !test_bit(R5_OVERWRITE,&failed_dev[1]->flags))))
             )) {
		/* 1/ We would like to get this block, possibly
		 * by computing it, but we might not be able to.
		 *
		 * 2/ Since parity check operations potentially
		 * make the parity block !uptodate it will need
		 * to be refreshed before any compute operations
		 * on data disks are scheduled.
		 *
		 * 3/ We hold off parity blocks re-reads until check
		 * operations have quiesced.
		 */
		if ((s->uptodate == disks-1) &&
		    !test_bit(STRIPE_OP_CHECK, &sh->ops.pending)) {
			pr_debug("Computing stripe %llu block %d\n",
				 (unsigned long long)sh->sector, disk_idx);
			set_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
			set_bit(R5_Wantcompute, &dev->flags);
			sh->ops.target = disk_idx;
			sh->ops.target2 = -1; /* no second target */
			s->req_compute = 1;
			sh->ops.count++;
			/* Careful: from this point on 'uptodate' is in the eye of
			 * raid_run_ops which services 'compute' operations before
			 * writes. R5_Wantcompute flags a block that will be R5_UPTODATE
			 * by the time it is needed for a  subsequent operation.
			 */
			s->uptodate++;
			return 0; /* s->uptodate + s->compute == disks */
		} else if ( s->uptodate == disks-2 && s->failed >= 2 ) {
			/* Computing 2-failure is *very* expensive; only
			 * do it if failed >= 2
			 */
			int other;
			for (other = disks; other--; ) {
				if (other == disk_idx)
					continue;
				if (!test_bit(R5_UPTODATE, &sh->dev[other].flags))
					break;
			}
			BUG_ON(other < 0);
			pr_debug("Computing stripe %llu blocks %d,%d\n",
				 (unsigned long long)sh->sector,
				 disk_idx, other);
			set_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
			set_bit(R5_Wantcompute, &dev->flags);
			set_bit(R5_Wantcompute, &sh->dev[other].flags);
			sh->ops.target = disk_idx;
			sh->ops.target2 = other;
			s->req_compute = 1;
			sh->ops.count++;
			s->uptodate += 2;
		} else if ((s->uptodate < disks-2) &&
			    test_bit(R5_Insync, &dev->flags)) {
			/* Note: we hold off compute operations while checks
			 * are in flight, but we still prefer 'compute' over 'read'
			 * hence we only read if (uptodate < disks-1) FIXME
			 */
			set_bit(R5_LOCKED, &dev->flags);
			set_bit(R5_Wantread, &dev->flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
			s->locked++;
			pr_debug("Reading block %d (sync=%d)\n", disk_idx,
				s->syncing);
		}
	}

	return ~0;
}

static void handle_issuing_new_read_requests6(struct stripe_head *sh,
			struct stripe_head_state *s, struct r6_state *r6s,
			int disks)
{
	int i;

	/* Clear completed compute operations.  Parity recovery
	 * (STRIPE_OP_MOD_REPAIR_PD) implies a write-back which is handled
	 * later on in this routine
	 */
	if (test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete) &&
		!test_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending)) {
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.ack);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
	}

	/* look for blocks to read/compute, skip this if a compute
	 * is already in flight, or if the stripe contents are in the
	 * midst of changing due to a write
	 */
	if (!test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending) &&
	    !test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending)) {
		for (i = disks; i--;)
			if (!__handle_issuing_new_read_requests6(sh, s, r6s,
			    i, disks))
				break;
	}
	set_bit(STRIPE_HANDLE, &sh->state);
}

static void handle_completed_postxor_requests(struct stripe_head *sh,
	struct stripe_head_state *s, int disks)
{
	struct stripe_queue *sq = sh->sq;
	int i, pd_idx = sq->pd_idx;
	int qd_idx = (sq->raid_conf->level != 6) ? -1 :
		raid6_next_disk(pd_idx, disks);
	struct r5dev *dev;

	clear_bit(STRIPE_OP_BIODRAIN, &sh->ops.complete);
	clear_bit(STRIPE_OP_BIODRAIN, &sh->ops.ack);
	clear_bit(STRIPE_OP_BIODRAIN, &sh->ops.pending);

	clear_bit(STRIPE_OP_POSTXOR, &sh->ops.complete);
	clear_bit(STRIPE_OP_POSTXOR, &sh->ops.ack);
	clear_bit(STRIPE_OP_POSTXOR, &sh->ops.pending);

	/* All the 'written' buffers and the parity block are ready to be
	 * written back to disk
	 */
	BUG_ON(!test_bit(R5_UPTODATE, &sh->dev[pd_idx].flags));
	if (!(qd_idx < 0))
		BUG_ON(!test_bit(R5_UPTODATE, &sh->dev[qd_idx].flags));

	for (i = disks; i--;) {
		struct r5_queue_dev *dev_q = &sq->dev[i];

		dev = &sh->dev[i];
		if (test_bit(R5_LOCKED, &dev->flags) &&
		    (i == pd_idx || i == qd_idx || dev_q->written)) {
			pr_debug("Writing block %d\n", i);
			set_bit(R5_Wantwrite, &dev->flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
			if (!test_bit(R5_Insync, &dev->flags) ||
			    ((i == pd_idx || i == qd_idx) &&
			    s->failed == 0))
				set_bit(STRIPE_INSYNC, &sh->state);
		}
	}
}

/* handle_completed_write_requests
 * any written block on an uptodate or failed drive can be returned.
 * Note that if we 'wrote' to a failed drive, it will be UPTODATE, but
 * never LOCKED, so we don't need to test 'failed' directly.
 */
static void handle_completed_write_requests(raid5_conf_t *conf,
	struct stripe_head *sh, int disks, struct bio **return_bi)
{
	int i;
	struct stripe_queue *sq = sh->sq;

	for (i = disks; i--; )
		if (sq->dev[i].written) {
			struct r5dev *dev = &sh->dev[i];
			struct r5_queue_dev *dev_q = &sq->dev[i];
			if (!test_bit(R5_LOCKED, &dev->flags) &&
				test_bit(R5_UPTODATE, &dev->flags)) {
				/* We can return any write requests */
				struct bio *wbi, *wbi2;
				int bitmap_end = 0;
				pr_debug("Return write for disc %d\n", i);
				spin_lock_irq(&conf->device_lock);
				wbi = dev_q->written;
				dev_q->written = NULL;

				if (dev->dpage) {
					/* with full-stripe write disk-cache
					 * actually is not UPTODATE
					 */
					clear_bit(R5_UPTODATE, &dev->flags);
					dev->vec.bv_page = dev->page;
					dev->dpage = NULL;
				}

				while (wbi && wbi->bi_sector <
					dev_q->sector + STRIPE_SECTORS) {
					wbi2 = r5_next_bio(wbi, dev_q->sector);
					if (--wbi->bi_phys_segments == 0) {
						md_write_end(conf->mddev);
						wbi->bi_next = *return_bi;
						*return_bi = wbi;
					}
					wbi = wbi2;
				}
				if (dev_q->towrite == NULL)
					bitmap_end = 1;
				spin_unlock_irq(&conf->device_lock);
				if (bitmap_end)
					bitmap_endwrite(conf->mddev->bitmap,
							sh->sector,
							STRIPE_SECTORS,
					 !test_bit(STRIPE_DEGRADED, &sh->state),
							0);
			}
		}
}

static void handle_issuing_new_write_requests5(raid5_conf_t *conf,
		struct stripe_head *sh,	struct stripe_head_state *s, int disks)
{
	int rmw = 0, rcw = 0, i;
	struct stripe_queue *sq = sh->sq;

	for (i = disks; i--; ) {
		/* would I have to read this buffer for read_modify_write */
		struct r5dev *dev = &sh->dev[i];
		struct r5_queue_dev *dev_q = &sq->dev[i];

		if ((dev_q->towrite || i == sq->pd_idx) &&
		    !test_bit(R5_LOCKED, &dev->flags) &&
		    !(test_bit(R5_UPTODATE, &dev->flags) ||
		      test_bit(R5_Wantcompute, &dev->flags))) {
			if (test_bit(R5_Insync, &dev->flags))
				rmw++;
			else
				rmw += 2*disks;  /* cannot read it */
		}
		/* Would I have to read this buffer for reconstruct_write */
		if (!test_bit(R5_OVERWRITE, &dev->flags) && i != sq->pd_idx &&
		    !test_bit(R5_LOCKED, &dev->flags) &&
		    !(test_bit(R5_UPTODATE, &dev->flags) ||
		    test_bit(R5_Wantcompute, &dev->flags))) {
			if (test_bit(R5_Insync, &dev->flags)) rcw++;
			else
				rcw += 2*disks;
		}
	}
	pr_debug("for sector %llu, rmw=%d rcw=%d\n",
		(unsigned long long)sh->sector, rmw, rcw);
	set_bit(STRIPE_HANDLE, &sh->state);
	if (rmw < rcw && rmw > 0)
		/* prefer read-modify-write, but need to get some data */
		for (i = disks; i--; ) {
			struct r5dev *dev = &sh->dev[i];
			struct r5_queue_dev *dev_q = &sq->dev[i];

			if ((dev_q->towrite || i == sq->pd_idx) &&
			    !test_bit(R5_LOCKED, &dev->flags) &&
			    !(test_bit(R5_UPTODATE, &dev->flags) ||
			    test_bit(R5_Wantcompute, &dev->flags)) &&
			    test_bit(R5_Insync, &dev->flags)) {
				pr_debug("Read_old block %d for r-m-w\n", i);
				set_bit(R5_LOCKED, &dev->flags);
				set_bit(R5_Wantread, &dev->flags);
				if (!test_and_set_bit(STRIPE_OP_IO,
				    &sh->ops.pending))
					sh->ops.count++;
				s->locked++;
			}
		}
	if (rcw <= rmw && rcw > 0)
		/* want reconstruct write, but need to get some data */
		for (i = disks; i--; ) {
			struct r5dev *dev = &sh->dev[i];

			if (!test_bit(R5_OVERWRITE, &dev->flags) &&
			    i != sq->pd_idx &&
			    !test_bit(R5_LOCKED, &dev->flags) &&
			    !(test_bit(R5_UPTODATE, &dev->flags) ||
			    test_bit(R5_Wantcompute, &dev->flags)) &&
			    test_bit(R5_Insync, &dev->flags)) {
				pr_debug("Read_old block "
					 "%d for Reconstruct\n", i);
				set_bit(R5_LOCKED, &dev->flags);
				set_bit(R5_Wantread, &dev->flags);
				if (!test_and_set_bit(STRIPE_OP_IO,
				    &sh->ops.pending))
					sh->ops.count++;
				s->locked++;
			}
		}
	/* now if nothing is locked, and if we have enough data,
	 * we can start a write request
	 */
	/* since handle_stripe can be called at any time we need to handle the
	 * case where a compute block operation has been submitted and then a
	 * subsequent call wants to start a write request.  raid_run_ops only
	 * handles the case where compute block and postxor are requested
	 * simultaneously.  If this is not the case then new writes need to be
	 * held off until the compute completes.
	 */
	if ((s->req_compute ||
	    !test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending)) &&
		(s->locked == 0 && (rcw == 0 || rmw == 0)))
		s->locked += handle_write_operations(sh, rcw == 0, 0);
}

static void handle_issuing_new_write_requests6(raid5_conf_t *conf,
		struct stripe_head *sh,	struct stripe_head_state *s,
		struct r6_state *r6s, int disks)
{
	struct stripe_queue *sq = sh->sq;
	int rcw = 0, must_compute = 0, pd_idx = sq->pd_idx, i;
	int qd_idx = r6s->qd_idx;
	for (i = disks; i--; ) {
		struct r5dev *dev = &sh->dev[i];
		/* Would I have to read this buffer for reconstruct_write */
		if (!test_bit(R5_OVERWRITE, &dev->flags) &&
		    i != pd_idx && i != qd_idx &&
		    !test_bit(R5_LOCKED, &dev->flags) &&
		    !test_bit(R5_UPTODATE, &dev->flags) &&
		    !test_bit(R5_Wantcompute, &dev->flags)) {
			if (test_bit(R5_Insync, &dev->flags)) rcw++;
			else {
				pr_debug("raid6: must_compute: "
					"disk %d flags=%#lx\n", i, dev->flags);
				must_compute++;
			}
		}
	}
	pr_debug("for sector %llu, rcw=%d, must_compute=%d\n",
	       (unsigned long long)sh->sector, rcw, must_compute);
	set_bit(STRIPE_HANDLE, &sh->state);

	if (rcw > 0)
		/* want reconstruct write, but need to get some data */
		for (i = disks; i--; ) {
			struct r5dev *dev = &sh->dev[i];
			if (!(!test_bit(R5_OVERWRITE, &dev->flags) &&
			    !(s->failed == 0 && (i == pd_idx || i == qd_idx)) &&
			    !test_bit(R5_LOCKED, &dev->flags) &&
			    !test_bit(R5_UPTODATE, &dev->flags) &&
			    !test_bit(R5_Wantcompute, &dev->flags) &&
			    test_bit(R5_Insync, &dev->flags)))
				continue;
			pr_debug("Read_old stripe %llu "
				 "block %d for Reconstruct\n",
				 (unsigned long long)sh->sector, i);
			set_bit(R5_LOCKED, &dev->flags);
			set_bit(R5_Wantread, &dev->flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
			s->locked++;
		}
	/* now if nothing is locked, and if we have enough data, we can start a
	 * write request
	 */
	/* since handle_stripe can be called at any time we need to handle the case
	 * where a compute block operation has been submitted and then a subsequent
	 * call wants to start a write request.  raid_run_ops only handles the case where
	 * compute block and postxor are requested simultaneously.  If this
	 * is not the case then new writes need to be held off until the compute
	 * completes.
	 */

	if (s->locked == 0 && rcw == 0) {
		if (must_compute > 0) {
			/* We have failed blocks and need to compute them */
			switch (s->failed) {
			case 0:
				BUG();
			case 1:
				set_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
				set_bit(R5_Wantcompute,
					&sh->dev[r6s->failed_num[0]].flags);
				sh->ops.target = r6s->failed_num[0];
				sh->ops.target2 = -1; /* no second target */
				s->req_compute = 1;
				sh->ops.count++;
				break;
			case 2:
				set_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
				set_bit(R5_Wantcompute,
					&sh->dev[r6s->failed_num[0]].flags);
				set_bit(R5_Wantcompute,
					&sh->dev[r6s->failed_num[1]].flags);
				sh->ops.target = r6s->failed_num[0];
				sh->ops.target2 = r6s->failed_num[1];
				s->req_compute = 1;
				sh->ops.count++;
				break;
			default:
				BUG();
			}
		}

		if (s->req_compute ||
		    !test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending))
			s->locked += handle_write_operations(sh, rcw == 0, 0);
	}
}

static void handle_parity_checks5(raid5_conf_t *conf, struct stripe_head *sh,
				struct stripe_head_state *s, int disks)
{
	struct stripe_queue *sq = sh->sq;
	int canceled_check = 0;

	set_bit(STRIPE_HANDLE, &sh->state);
	/* complete a check operation */
	if (test_and_clear_bit(STRIPE_OP_CHECK, &sh->ops.complete)) {
	    clear_bit(STRIPE_OP_CHECK, &sh->ops.ack);
	    clear_bit(STRIPE_OP_CHECK, &sh->ops.pending);
		if (s->failed == 0) {
			if (sh->ops.zero_sum_result == 0)
				/* parity is correct (on disc,
				 * not in buffer any more)
				 */
				set_bit(STRIPE_INSYNC, &sh->state);
			else {
				conf->mddev->resync_mismatches +=
					STRIPE_SECTORS;
				if (test_bit(
				     MD_RECOVERY_CHECK, &conf->mddev->recovery))
					/* don't try to repair!! */
					set_bit(STRIPE_INSYNC, &sh->state);
				else {
					set_bit(STRIPE_OP_COMPUTE_BLK,
						&sh->ops.pending);
					set_bit(STRIPE_OP_MOD_REPAIR_PD,
						&sh->ops.pending);
					set_bit(R5_Wantcompute,
						&sh->dev[sq->pd_idx].flags);
					sh->ops.target = sq->pd_idx;
					sh->ops.target2 = -1; /* no second target */
					sh->ops.count++;
					s->uptodate++;
				}
			}
		} else
			canceled_check = 1; /* STRIPE_INSYNC is not set */
	}

	/* check if we can clear a parity disk reconstruct */
	if (test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete) &&
		test_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending)) {

		clear_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.ack);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
	}

	/* start a new check operation if there are no failures, the stripe is
	 * not insync, and a repair is not in flight
	 */
	if (s->failed == 0 &&
	    !test_bit(STRIPE_INSYNC, &sh->state) &&
	    !test_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending)) {
		if (!test_and_set_bit(STRIPE_OP_CHECK, &sh->ops.pending)) {
			BUG_ON(s->uptodate != disks);
			clear_bit(R5_UPTODATE, &sh->dev[sq->pd_idx].flags);
			sh->ops.count++;
			s->uptodate--;
		}
	}

	/* Wait for check parity and compute block operations to complete
	 * before write-back.  If a failure occurred while the check operation
	 * was in flight we need to cycle this stripe through handle_stripe
	 * since the parity block may not be uptodate
	 */
	if (!canceled_check && !test_bit(STRIPE_INSYNC, &sh->state) &&
	    !test_bit(STRIPE_OP_CHECK, &sh->ops.pending) &&
	    !test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending)) {
		struct r5dev *dev;

		/* either failed parity check, or recovery is happening */
		if (s->failed == 0)
			s->failed_num = sq->pd_idx;
		dev = &sh->dev[s->failed_num];
		BUG_ON(!test_bit(R5_UPTODATE, &dev->flags));
		BUG_ON(s->uptodate != disks);

		set_bit(R5_LOCKED, &dev->flags);
		set_bit(R5_Wantwrite, &dev->flags);
		if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
			sh->ops.count++;

		clear_bit(STRIPE_DEGRADED, &sh->state);
		s->locked++;
		set_bit(STRIPE_INSYNC, &sh->state);
	}
}


static void handle_parity_checks6(raid5_conf_t *conf, struct stripe_head *sh,
				struct stripe_head_state *s,
				struct r6_state *r6s,
				int disks)
{
	struct stripe_queue *sq = sh->sq;
	int pd_idx = sq->pd_idx;
	int qd_idx = r6s->qd_idx;

	set_bit(STRIPE_HANDLE, &sh->state);

	BUG_ON(s->failed > 2);

	/* Want to check and possibly repair P and Q.
	 * However there could be one 'failed' device, in which
	 * case we can only check one of them, possibly using the
	 * other to generate missing data
	 */
	if (s->failed <= 1 && !test_bit(STRIPE_OP_MOD_REPAIR_PD,
	    &sh->ops.pending)) {
		/* If one or no disks failed */
		if (!test_and_set_bit(STRIPE_OP_CHECK, &sh->ops.pending)) {
			/* Run check operation */
			pr_debug("run check with uptodate = %d of %d\n",
				s->uptodate, disks);
			BUG_ON(s->uptodate != disks);
			if ( s->failed == r6s->q_failed ) {
				/* no or only q-disk failed - check p */
				clear_bit(R5_UPTODATE, &sh->dev[pd_idx].flags);
				set_bit(STRIPE_OP_CHECK_PP, &sh->ops.pending);
				s->uptodate--;
			}
			if ( !r6s->q_failed ) {
				/* Q-disk is OK - then check Q-parity also */
				clear_bit(R5_UPTODATE, &sh->dev[qd_idx].flags);
				set_bit(STRIPE_OP_CHECK_QP, &sh->ops.pending);
				s->uptodate--;
			}
			sh->ops.count++;
		} else if (test_and_clear_bit(STRIPE_OP_CHECK,
		    &sh->ops.complete)) {
			/* Check operation has been completed */
			clear_bit(STRIPE_OP_CHECK, &sh->ops.ack);
			clear_bit(STRIPE_OP_CHECK, &sh->ops.pending);
				/* See what we've got */
			if (test_and_clear_bit(STRIPE_OP_CHECK_PP,
			    &sh->ops.pending) && sh->ops.zero_sum_result != 0) {
				/* P-parity is wrong */
				set_bit(STRIPE_OP_UPDATE_PP, &sh->ops.pending);
			}
			if (test_and_clear_bit(STRIPE_OP_CHECK_QP, &sh->
			    ops.pending) && sh->ops.zero_qsum_result != 0) {
				/* Q-parity is wrong */
				set_bit(STRIPE_OP_UPDATE_QP, &sh->ops.pending);
			}
			if (!test_bit(STRIPE_OP_UPDATE_PP, &sh->ops.pending) &&
			    !test_bit(STRIPE_OP_UPDATE_QP, &sh->ops.pending)) {
				/* Both parities are correct */
				set_bit(STRIPE_INSYNC, &sh->state);
			} else {
				/* One or both parities are wrong */
				conf->mddev->resync_mismatches +=
				    STRIPE_SECTORS;
				if (test_bit(MD_RECOVERY_CHECK,
				    &conf->mddev->recovery)) {
					/* Don't try to repair */
					clear_bit(STRIPE_OP_UPDATE_PP,
					    &sh->ops.pending);
					clear_bit(STRIPE_OP_UPDATE_QP,
					    &sh->ops.pending);
					set_bit(STRIPE_INSYNC, &sh->state);
				} else {
					/*
					 * One or both parities have to be
					 * updated
					 */
					pr_debug("Computing ... ");
					BUG_ON(test_and_set_bit(
						STRIPE_OP_COMPUTE_BLK,
						&sh->ops.pending));
					set_bit(STRIPE_OP_MOD_REPAIR_PD,
						&sh->ops.pending);
					sh->ops.count++;
					if (test_bit(STRIPE_OP_UPDATE_PP,
					    &sh->ops.pending)) {
						pr_debug("P ");
						BUG_ON(test_and_set_bit(
						    R5_Wantcompute,
						    &sh->dev[pd_idx].flags));
						sh->ops.target = pd_idx;
						s->uptodate++;
					} else
						sh->ops.target = -1;
					if (test_bit(STRIPE_OP_UPDATE_QP,
					    &sh->ops.pending)) {
						pr_debug("Q ");
						BUG_ON(test_and_set_bit(
						    R5_Wantcompute,
						    &sh->dev[qd_idx].flags));
						sh->ops.target2 = qd_idx;
						s->uptodate++;
					} else
						sh->ops.target2 = -1;
					pr_debug("disk(s)\n");
				}
			}
		}
	}

	/* check if we can clear a parity disk reconstruct */
	if (test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete) &&
	    test_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending)) {
		clear_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.complete);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.ack);
		clear_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending);
	}

	/* Wait for check parity and compute block operations to complete
	 * before write-back
	 */
	if (!test_bit(STRIPE_INSYNC, &sh->state) &&
		!test_bit(STRIPE_OP_CHECK, &sh->ops.pending) &&
		!test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending)) {
		struct r5dev *dev;

		/* now write out any block on a failed drive,
		 * or P or Q if they need it
		 */

		if (s->failed == 2) {
			dev = &sh->dev[r6s->failed_num[1]];
			s->locked++;
			set_bit(R5_LOCKED, &dev->flags);
			set_bit(R5_Wantwrite, &dev->flags);
			BUG_ON(!test_bit(R5_UPTODATE, &dev->flags));
		}
		if (s->failed >= 1) {
			dev = &sh->dev[r6s->failed_num[0]];
			s->locked++;
			set_bit(R5_LOCKED, &dev->flags);
			set_bit(R5_Wantwrite, &dev->flags);
			BUG_ON(!test_bit(R5_UPTODATE, &dev->flags));
		}

		if (test_and_clear_bit(STRIPE_OP_UPDATE_PP, &sh->ops.pending)) {
			dev = &sh->dev[pd_idx];
			s->locked++;
			set_bit(R5_LOCKED, &dev->flags);
			set_bit(R5_Wantwrite, &dev->flags);
			BUG_ON(!test_bit(R5_UPTODATE, &dev->flags));
		}
		if (test_and_clear_bit(STRIPE_OP_UPDATE_QP, &sh->ops.pending)) {
			dev = &sh->dev[qd_idx];
			s->locked++;
			set_bit(R5_LOCKED, &dev->flags);
			set_bit(R5_Wantwrite, &dev->flags);
			BUG_ON(!test_bit(R5_UPTODATE, &dev->flags));
		}
		if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
			sh->ops.count++;
		clear_bit(STRIPE_DEGRADED, &sh->state);

		set_bit(STRIPE_INSYNC, &sh->state);
	}
}

static void handle_stripe_expansion(raid5_conf_t *conf, struct stripe_head *sh,
				struct r6_state *r6s)
{
	int i;
	struct stripe_queue *sq = sh->sq;

	/* We have read all the blocks in this stripe and now we need to
	 * copy some of them into a target stripe for expand.
	 */
	struct dma_async_tx_descriptor *tx = NULL;
	clear_bit(STRIPE_EXPAND_SOURCE, &sh->state);
	for (i = 0; i < sq->disks; i++)
		if (i != sq->pd_idx && (!r6s || i != r6s->qd_idx)) {
			int dd_idx, pd_idx, j;
			struct stripe_head *sh2;
			struct stripe_queue *sq2;
			int disks = conf->raid_disks;

			sector_t bn = compute_blocknr(conf, sq->disks,
						sh->sector, sq->pd_idx, i);
			sector_t s = raid5_compute_sector(bn, conf->raid_disks,
						disks -
						conf->max_degraded, &dd_idx,
						&pd_idx, conf);
			sq2 = get_active_queue(conf, s, disks, pd_idx, 1);
			if (sq2)
				sh2 = get_active_stripe(sq, disks, 1);
			if (!(sq2 && sh2)) {
				/* so far only the early blocks of this stripe
				 * have been requested.  When later blocks
				 * get requested, we will try again
				 */
				if (sq2)
					release_queue(sq2);
				continue;
			}

			if (!test_bit(STRIPE_QUEUE_EXPANDING, &sq2->state) ||
			    test_bit(R5_Expanded, &sh2->dev[dd_idx].flags)) {
				/* must have already done this block */
				release_stripe(sh2);
				release_queue(sq2);
				continue;
			}

			/* place all the copies on one channel */
			tx = async_memcpy(sh2->dev[dd_idx].page,
				sh->dev[i].page, 0, 0, STRIPE_SIZE,
				ASYNC_TX_DEP_ACK, tx, NULL, NULL);

			set_bit(R5_Expanded, &sh2->dev[dd_idx].flags);
			set_bit(R5_UPTODATE, &sh2->dev[dd_idx].flags);
			for (j = 0; j < conf->raid_disks; j++)
				if (j != sq2->pd_idx &&
				    (!r6s || j != raid6_next_disk(sq2->pd_idx,
								 sq2->disks)) &&
				    !test_bit(R5_Expanded, &sh2->dev[j].flags))
					break;
			if (j == conf->raid_disks) {
				set_bit(STRIPE_EXPAND_READY, &sh2->state);
				set_bit(STRIPE_HANDLE, &sh2->state);
			}
			release_stripe(sh2);
			release_queue(sq2);

		}
	/* done submitting copies, wait for them to complete */
	if (tx) {
		async_tx_ack(tx);
		dma_wait_for_async_tx(tx);
	}
}

/*
 * handle_stripe - do things to a stripe.
 *
 * We lock the stripe and then examine the state of various bits
 * to see what needs to be done.
 * Possible results:
 *    return some read request which now have data
 *    return some write requests which are safely on disc
 *    schedule a read on some buffers
 *    schedule a write of some buffers
 *    return confirmation of parity correctness
 *
 * buffers are taken off read_list or write_list, and bh_cache buffers
 * get BH_Lock set before the stripe lock is released.
 *
 */

static void handle_stripe5(struct stripe_head *sh)
{
	struct stripe_queue *sq = sh->sq;
	raid5_conf_t *conf = sq->raid_conf;
	int disks = sq->disks, i;
	struct bio *return_bi = NULL;
	struct stripe_head_state s;
	struct r5dev *dev;
	unsigned long pending = 0;

	memset(&s, 0, sizeof(s));
	pr_debug("handling stripe %llu, state=%#lx cnt=%d, pd_idx=%d "
		"ops=%lx:%lx:%lx\n", (unsigned long long)sh->sector, sh->state,
		atomic_read(&sh->count), sq->pd_idx,
		sh->ops.pending, sh->ops.ack, sh->ops.complete);

	spin_lock(&sq->lock);
	clear_bit(STRIPE_HANDLE, &sh->state);

	s.syncing = test_bit(STRIPE_SYNCING, &sh->state);
	s.expanding = test_bit(STRIPE_EXPAND_SOURCE, &sh->state);
	s.expanded = test_bit(STRIPE_EXPAND_READY, &sh->state);
	/* Now to look around and see what can be done */

	/* clean-up completed biofill operations */
	if (test_bit(STRIPE_OP_BIOFILL, &sh->ops.complete)) {
		clear_bit(STRIPE_OP_BIOFILL, &sh->ops.pending);
		clear_bit(STRIPE_OP_BIOFILL, &sh->ops.ack);
		clear_bit(STRIPE_OP_BIOFILL, &sh->ops.complete);
	}

	rcu_read_lock();
	for (i=disks; i--; ) {
		mdk_rdev_t *rdev;
		struct r5dev *dev = &sh->dev[i];
		struct r5_queue_dev *dev_q = &sq->dev[i];
		clear_bit(R5_Insync, &dev->flags);
		if (test_and_clear_bit(i, sq->overwrite))
			set_bit(R5_OVERWRITE, &dev->flags);

		pr_debug("check %d: state 0x%lx toread %p read %p write %p "
			"written %p\n",	i, dev->flags, dev_q->toread,
			dev_q->read, dev_q->towrite, dev_q->written);

		/* maybe we can request a biofill operation
		 *
		 * new wantfill requests are only permitted while
		 * STRIPE_OP_BIOFILL is clear
		 */
		if (test_bit(R5_UPTODATE, &dev->flags) && dev_q->toread &&
			!test_bit(STRIPE_OP_BIOFILL, &sh->ops.pending))
			set_bit(R5_Wantfill, &dev->flags);

		/* now count some things */
		if (test_bit(R5_LOCKED, &dev->flags)) s.locked++;
		if (test_bit(R5_UPTODATE, &dev->flags)) s.uptodate++;
		if (test_bit(R5_Wantcompute, &dev->flags)) s.compute++;

		if (test_bit(R5_Wantfill, &dev->flags))
			s.to_fill++;
		else if (dev_q->toread)
			s.to_read++;
		if (dev_q->towrite) {
			s.to_write++;
			if (!test_bit(R5_OVERWRITE, &dev->flags))
				s.non_overwrite++;
		}
		if (dev_q->written)
			s.written++;
		rdev = rcu_dereference(conf->disks[i].rdev);
		if (!rdev || !test_bit(In_sync, &rdev->flags)) {
			/* The ReadError flag will just be confusing now */
			clear_bit(R5_ReadError, &dev->flags);
			clear_bit(R5_ReWrite, &dev->flags);
		}
		if (!rdev || !test_bit(In_sync, &rdev->flags)
		    || test_bit(R5_ReadError, &dev->flags)) {
			s.failed++;
			s.failed_num = i;
		} else
			set_bit(R5_Insync, &dev->flags);
	}
	rcu_read_unlock();

	if (s.to_fill && !test_and_set_bit(STRIPE_OP_BIOFILL, &sh->ops.pending))
		sh->ops.count++;

	pr_debug("locked=%d uptodate=%d to_read=%d"
		" to_write=%d failed=%d failed_num=%d\n",
		s.locked, s.uptodate, s.to_read, s.to_write,
		s.failed, s.failed_num);
	/* check if the array has lost two devices and, if so, some requests might
	 * need to be failed
	 */
	if (s.failed > 1 && s.to_read+s.to_write+s.written)
		handle_requests_to_failed_array(conf, sh, &s, disks,
						&return_bi);
	if (s.failed > 1 && s.syncing) {
		md_done_sync(conf->mddev, STRIPE_SECTORS,0);
		clear_bit(STRIPE_SYNCING, &sh->state);
		s.syncing = 0;
	}

	/* might be able to return some write requests if the parity block
	 * is safe, or on a failed drive
	 */
	dev = &sh->dev[sq->pd_idx];
	if ( s.written &&
	     ((test_bit(R5_Insync, &dev->flags) &&
	       !test_bit(R5_LOCKED, &dev->flags) &&
	       test_bit(R5_UPTODATE, &dev->flags)) ||
		(s.failed == 1 && s.failed_num == sq->pd_idx)))
		handle_completed_write_requests(conf, sh, disks, &return_bi);

	/* Now we might consider reading some blocks, either to check/generate
	 * parity, or to satisfy requests
	 * or to load a block that is being partially written.
	 */
	if (s.to_read || s.non_overwrite ||
	    (s.syncing && (s.uptodate + s.compute < disks)) || s.expanding ||
	    test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending))
		handle_issuing_new_read_requests5(sh, &s, disks);

	/* Now we check to see if any write operations have recently
	 * completed
	 */

	/* leave prexor set until postxor is done, allows us to distinguish
	 * a rmw from a rcw during biodrain
	 */
	if (test_bit(STRIPE_OP_PREXOR, &sh->ops.complete) &&
		test_bit(STRIPE_OP_POSTXOR, &sh->ops.complete)) {

		clear_bit(STRIPE_OP_PREXOR, &sh->ops.complete);
		clear_bit(STRIPE_OP_PREXOR, &sh->ops.ack);
		clear_bit(STRIPE_OP_PREXOR, &sh->ops.pending);

		for (i = disks; i--; )
			clear_bit(R5_Wantprexor, &sh->dev[i].flags);
	}

	/* if only POSTXOR is set then this is an 'expand' postxor */
	if (test_bit(STRIPE_OP_BIODRAIN, &sh->ops.complete) &&
	    test_bit(STRIPE_OP_POSTXOR, &sh->ops.complete))
		handle_completed_postxor_requests(sh, &s, disks);

	/* Now to consider new write requests and what else, if anything
	 * should be read.  We do not handle new writes when:
	 * 1/ A 'write' operation (copy+xor) is already in flight.
	 * 2/ A 'check' operation is in flight, as it may clobber the parity
	 *    block.
	 */
	if (s.to_write && !test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending) &&
			  !test_bit(STRIPE_OP_CHECK, &sh->ops.pending))
		handle_issuing_new_write_requests5(conf, sh, &s, disks);

	/* maybe we need to check and possibly fix the parity for this stripe
	 * Any reads will already have been scheduled, so we just see if enough
	 * data is available.  The parity check is held off while parity
	 * dependent operations are in flight.
	 */
	if ((s.syncing && s.locked == 0 &&
	     !test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending) &&
	     !test_bit(STRIPE_INSYNC, &sh->state)) ||
	      test_bit(STRIPE_OP_CHECK, &sh->ops.pending) ||
	      test_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending))
		handle_parity_checks5(conf, sh, &s, disks);

	if (s.syncing && s.locked == 0 && test_bit(STRIPE_INSYNC, &sh->state)) {
		md_done_sync(conf->mddev, STRIPE_SECTORS,1);
		clear_bit(STRIPE_SYNCING, &sh->state);
	}

	/* If the failed drive is just a ReadError, then we might need to progress
	 * the repair/check process
	 */
	if (s.failed == 1 && !conf->mddev->ro &&
	    test_bit(R5_ReadError, &sh->dev[s.failed_num].flags)
	    && !test_bit(R5_LOCKED, &sh->dev[s.failed_num].flags)
	    && test_bit(R5_UPTODATE, &sh->dev[s.failed_num].flags)
		) {
		dev = &sh->dev[s.failed_num];
		if (!test_bit(R5_ReWrite, &dev->flags)) {
			set_bit(R5_Wantwrite, &dev->flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
			set_bit(R5_ReWrite, &dev->flags);
			set_bit(R5_LOCKED, &dev->flags);
			s.locked++;
		} else {
			/* let's read it back */
			set_bit(R5_Wantread, &dev->flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
			set_bit(R5_LOCKED, &dev->flags);
			s.locked++;
		}
	}

	/* Finish postxor operations initiated by the expansion
	 * process
	 */
	if (test_bit(STRIPE_OP_POSTXOR, &sh->ops.complete) &&
		!test_bit(STRIPE_OP_BIODRAIN, &sh->ops.pending)) {

		clear_bit(STRIPE_QUEUE_EXPANDING, &sq->state);

		clear_bit(STRIPE_OP_POSTXOR, &sh->ops.pending);
		clear_bit(STRIPE_OP_POSTXOR, &sh->ops.ack);
		clear_bit(STRIPE_OP_POSTXOR, &sh->ops.complete);

		for (i = conf->raid_disks; i--; ) {
			set_bit(R5_Wantwrite, &sh->dev[i].flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
		}
	}

	if (s.expanded && test_bit(STRIPE_QUEUE_EXPANDING, &sq->state) &&
		!test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending)) {
		/* Need to write out all blocks after computing parity */
		sq->disks = conf->raid_disks;
		sq->pd_idx = stripe_to_pdidx(sh->sector, conf,
			conf->raid_disks);
		s.locked += handle_write_operations(sh, 1, 1);
	} else if (s.expanded &&
		!test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending)) {
		clear_bit(STRIPE_EXPAND_READY, &sh->state);
		atomic_dec(&conf->reshape_stripes);
		wake_up(&conf->wait_for_overlap);
		md_done_sync(conf->mddev, STRIPE_SECTORS, 1);
	}

	if (s.expanding && s.locked == 0 &&
	    !test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending))
		handle_stripe_expansion(conf, sh, NULL);

	if (sh->ops.count)
		pending = get_stripe_work(sh);

	spin_unlock(&sq->lock);

	if (pending)
		raid_run_ops(sh, pending);

	return_io(return_bi);

}

static void handle_stripe6(struct stripe_head *sh)
{
	struct stripe_queue *sq = sh->sq;
	raid6_conf_t *conf = sq->raid_conf;
	int disks = sq->disks;
	struct bio *return_bi = NULL;
	int i, pd_idx = sq->pd_idx;
	struct stripe_head_state s;
	struct r6_state r6s;
	struct r5dev *dev, *pdev, *qdev;
	unsigned long pending = 0;

	r6s.qd_idx = raid6_next_disk(pd_idx, disks);
	memset(&s, 0, sizeof(s));
	pr_debug("handling stripe %llu, state=%#lx, cnt=%d, "
		"pd_idx=%d, qd_idx=%d "
		"ops=%lx:%lx:%lx\n", (unsigned long long)sh->sector, sh->state,
		atomic_read(&sh->count), sq->pd_idx, r6s.qd_idx,
		sh->ops.pending, sh->ops.ack, sh->ops.complete);

	spin_lock(&sq->lock);
	clear_bit(STRIPE_HANDLE, &sh->state);

	s.syncing = test_bit(STRIPE_SYNCING, &sh->state);
	s.expanding = test_bit(STRIPE_EXPAND_SOURCE, &sh->state);
	s.expanded = test_bit(STRIPE_EXPAND_READY, &sh->state);

	/* clean-up completed biofill operations */
	if (test_bit(STRIPE_OP_BIOFILL, &sh->ops.complete)) {
		clear_bit(STRIPE_OP_BIOFILL, &sh->ops.pending);
		clear_bit(STRIPE_OP_BIOFILL, &sh->ops.ack);
		clear_bit(STRIPE_OP_BIOFILL, &sh->ops.complete);
	}

	/* Now to look around and see what can be done */

	rcu_read_lock();
	for (i=disks; i--; ) {
		mdk_rdev_t *rdev;
		struct r5_queue_dev *dev_q = &sq->dev[i];

		dev = &sh->dev[i];
		clear_bit(R5_Insync, &dev->flags);
		if (test_and_clear_bit(i, sq->overwrite))
			set_bit(R5_OVERWRITE, &dev->flags);

		pr_debug("check %d: state 0x%lx toread %p write %p "
			"written %p\n",	i, dev->flags, dev_q->toread,
			dev_q->towrite, dev_q->written);
		/* maybe we can reply to a read
		 * new wantfill requests are only permitted while
		 * STRIPE_OP_BIOFILL is clear
		 */
		if (test_bit(R5_UPTODATE, &dev->flags) && dev_q->toread &&
		    !test_bit(STRIPE_OP_BIOFILL, &sh->ops.pending))
			set_bit(R5_Wantfill, &dev->flags);

		/* now count some things */
		if (test_bit(R5_LOCKED, &dev->flags)) s.locked++;
		if (test_bit(R5_UPTODATE, &dev->flags)) s.uptodate++;

		if (test_bit(R5_Wantfill, &dev->flags)) s.to_fill++;
		else if (dev_q->toread) s.to_read++;

		if (test_bit(R5_Wantcompute, &dev->flags))
			BUG_ON(++s.compute > 2);

		if (dev_q->towrite) {
			s.to_write++;
			if (!test_bit(R5_OVERWRITE, &dev->flags))
				s.non_overwrite++;
		}
		if (dev_q->written)
			s.written++;
		rdev = rcu_dereference(conf->disks[i].rdev);
		if (!rdev || !test_bit(In_sync, &rdev->flags)) {
			/* The ReadError flag will just be confusing now */
			clear_bit(R5_ReadError, &dev->flags);
			clear_bit(R5_ReWrite, &dev->flags);
		}
		if (!rdev || !test_bit(In_sync, &rdev->flags)
		    || test_bit(R5_ReadError, &dev->flags)) {
			if (s.failed < 2)
				r6s.failed_num[s.failed] = i;
			s.failed++;
		} else
			set_bit(R5_Insync, &dev->flags);
	}
	rcu_read_unlock();

	if (s.to_fill && !test_and_set_bit(STRIPE_OP_BIOFILL, &sh->ops.pending))
		sh->ops.count++;

	pr_debug("locked=%d uptodate=%d to_read=%d"
	       " to_write=%d failed=%d failed_num=%d,%d\n",
	       s.locked, s.uptodate, s.to_read, s.to_write, s.failed,
	       r6s.failed_num[0], r6s.failed_num[1]);
	/* check if the array has lost >2 devices and, if so, some requests
	 * might need to be failed
	 */
	if (s.failed > 2 && s.to_read+s.to_write+s.written)
		handle_requests_to_failed_array(conf, sh, &s, disks,
						&return_bi);
	if (s.failed > 2 && s.syncing) {
		md_done_sync(conf->mddev, STRIPE_SECTORS,0);
		clear_bit(STRIPE_SYNCING, &sh->state);
		s.syncing = 0;
	}

	/*
	 * might be able to return some write requests if the parity blocks
	 * are safe, or on a failed drive
	 */
	pdev = &sh->dev[pd_idx];
	r6s.p_failed = (s.failed >= 1 && r6s.failed_num[0] == pd_idx)
		|| (s.failed >= 2 && r6s.failed_num[1] == pd_idx);
	qdev = &sh->dev[r6s.qd_idx];
	r6s.q_failed = (s.failed >= 1 && r6s.failed_num[0] == r6s.qd_idx)
		|| (s.failed >= 2 && r6s.failed_num[1] == r6s.qd_idx);

	if ( s.written &&
	     ( r6s.p_failed || ((test_bit(R5_Insync, &pdev->flags)
			     && !test_bit(R5_LOCKED, &pdev->flags)
			     && test_bit(R5_UPTODATE, &pdev->flags)))) &&
	     ( r6s.q_failed || ((test_bit(R5_Insync, &qdev->flags)
			     && !test_bit(R5_LOCKED, &qdev->flags)
			     && test_bit(R5_UPTODATE, &qdev->flags)))))
		handle_completed_write_requests(conf, sh, disks, &return_bi);

	/* Now we might consider reading some blocks, either to check/generate
	 * parity, or to satisfy requests
	 * or to load a block that is being partially written.
	 */
	if (s.to_read || s.non_overwrite || (s.to_write && s.failed) ||
	    (s.syncing && (s.uptodate + s.compute < disks)) || s.expanding ||
	    test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending))
		handle_issuing_new_read_requests6(sh, &s, &r6s, disks);

	/* Now we check to see if any write operations have recently
	 * completed
	 */

	/* if only POSTXOR is set then this is an 'expand' postxor */
	if (test_bit(STRIPE_OP_BIODRAIN, &sh->ops.complete) &&
	    test_bit(STRIPE_OP_POSTXOR, &sh->ops.complete))
		handle_completed_postxor_requests(sh, &s, disks);

	/* 1/ Now to consider new write requests and what else,
	 * if anything shuold be read
	 * 2/ Check operations clobber the parity block so do not start
	 * new writes while a check is in flight
	 * 3/ Write operations do not stack
	 */
	if (s.to_write && !test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending) &&
	    !test_bit(STRIPE_OP_CHECK, &sh->ops.pending))
		handle_issuing_new_write_requests6(conf, sh, &s, &r6s, disks);

	/* 1/ Maybe we need to check and possibly fix the parity for this stripe
	 * Any reads will already have been scheduled, so we just see
	 * if enough data is available
	 * 2/ Hold off parity checks while parity dependent operations are
	 * in flight (conflicting writes are protected by the 'locked' variable)
	 */
	if ((s.syncing && s.locked == 0 &&
	    !test_bit(STRIPE_OP_COMPUTE_BLK,&sh->ops.pending) &&
	    !test_bit(STRIPE_INSYNC, &sh->state)) ||
	    test_bit(STRIPE_OP_CHECK, &sh->ops.pending) ||
	    test_bit(STRIPE_OP_MOD_REPAIR_PD, &sh->ops.pending))
		handle_parity_checks6(conf, sh, &s, &r6s, disks);

	if (s.syncing && s.locked == 0 && test_bit(STRIPE_INSYNC, &sh->state)) {
		md_done_sync(conf->mddev, STRIPE_SECTORS,1);
		clear_bit(STRIPE_SYNCING, &sh->state);
	}

	/* If the failed drives are just a ReadError, then we might need
	 * to progress the repair/check process
	 */
	if (s.failed <= 2 && !conf->mddev->ro)
		for (i = 0; i < s.failed; i++) {
			dev = &sh->dev[r6s.failed_num[i]];
			if (test_bit(R5_ReadError, &dev->flags)
			    && !test_bit(R5_LOCKED, &dev->flags)
			    && test_bit(R5_UPTODATE, &dev->flags)
				) {
				if (!test_bit(R5_ReWrite, &dev->flags)) {
					set_bit(R5_Wantwrite, &dev->flags);
					set_bit(R5_ReWrite, &dev->flags);
				} else {
					/* let's read it back */
					set_bit(R5_Wantread, &dev->flags);
				}
				if (!test_and_set_bit(STRIPE_OP_IO,
							&sh->ops.pending))
					sh->ops.count++;

				set_bit(R5_LOCKED, &dev->flags);
				s.locked++;
			}
		}

	/* Finish postxor operations initiated by the expansion
	 * process
	 */
	if (test_bit(STRIPE_OP_POSTXOR, &sh->ops.complete) &&
	    !test_bit(STRIPE_OP_BIODRAIN, &sh->ops.pending)) {
		clear_bit(STRIPE_QUEUE_EXPANDING, &sq->state);
		clear_bit(STRIPE_OP_POSTXOR, &sh->ops.pending);
		clear_bit(STRIPE_OP_POSTXOR, &sh->ops.ack);
		clear_bit(STRIPE_OP_POSTXOR, &sh->ops.complete);

		for (i = conf->raid_disks; i--;  ) {
			set_bit(R5_Wantwrite, &sh->dev[i].flags);
			if (!test_and_set_bit(STRIPE_OP_IO, &sh->ops.pending))
				sh->ops.count++;
		}
	}

	if (s.expanded && test_bit(STRIPE_QUEUE_EXPANDING, &sq->state) &&
	    !test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending)) {
		/* Need to write out all blocks after computing P&Q */
		sq->disks = conf->raid_disks;
		sq->pd_idx = stripe_to_pdidx(sh->sector, conf,
					     conf->raid_disks);
		s.locked += handle_write_operations(sh, 0, 1);
	} else if (s.expanded &&
		   !test_bit(STRIPE_OP_POSTXOR, &sh->ops.pending)) {
		clear_bit(STRIPE_EXPAND_READY, &sh->state);
		atomic_dec(&conf->reshape_stripes);
		wake_up(&conf->wait_for_overlap);
		md_done_sync(conf->mddev, STRIPE_SECTORS, 1);
	}

	if (s.expanding && s.locked == 0 &&
	    !test_bit(STRIPE_OP_COMPUTE_BLK, &sh->ops.pending))
		handle_stripe_expansion(conf, sh, &r6s);

	if (sh->ops.count)
		pending = get_stripe_work(sh);

	spin_unlock(&sq->lock);

	if (pending)
		raid_run_ops(sh, pending);

	return_io(return_bi);
}

static void handle_stripe(struct stripe_head *sh)
{
	if (sh->sq->raid_conf->level == 6)
		handle_stripe6(sh);
	else
		handle_stripe5(sh);
}

static void handle_queue(struct stripe_queue *sq, int disks, int data_disks)
{
	int to_write = io_weight(sq->to_write, disks);

	pr_debug("%s: sector %llu "
		 "state: %#lx r: %lu w: %lu o: %lu\n", __FUNCTION__,
		 (unsigned long long) sq->sector, sq->state,
		 io_weight(sq->to_read, disks),
		 io_weight(sq->to_write, disks),
		 io_weight(sq->overwrite, disks));

	/* process i/o if the cache is inactive */
	if (to_write == data_disks || io_weight(sq->to_read, disks) ||
	    (to_write && test_bit(STRIPE_QUEUE_PREREAD_ACTIVE, &sq->state))) {
		struct stripe_head *sh = get_active_stripe(sq, disks, 1);
		if (sh) {
			set_bit(STRIPE_HANDLE, &sh->state);
			release_stripe(sh);
		}
	}
}

static void raid5_activate_delayed(raid5_conf_t *conf)
{
	if (atomic_read(&conf->preread_active_queues) < IO_THRESHOLD) {
		struct stripe_queue *sq, *_sq;
		pr_debug("%s\n", __FUNCTION__);
		list_for_each_entry_safe(sq, _sq, &conf->delayed_q_list,
					 list_node) {
			list_del_init(&sq->list_node);
			atomic_inc(&sq->count);
			if (!test_and_set_bit(STRIPE_QUEUE_PREREAD_ACTIVE,
						&sq->state))
				atomic_inc(&conf->preread_active_queues);
			__release_queue(conf, sq);
		}
	} else
		blk_plug_device(conf->mddev->queue);
}

static void activate_bit_delay(raid5_conf_t *conf)
{
	/* device_lock is held */
	struct list_head head;
	list_add(&head, &conf->bitmap_q_list);
	list_del_init(&conf->bitmap_q_list);
	while (!list_empty(&head)) {
		struct stripe_queue *sq = list_entry(head.next,
						     struct stripe_queue,
						     list_node);
		list_del_init(&sq->list_node);
		atomic_inc(&sq->count);
		__release_queue(conf, sq);
	}
}

static void unplug_slaves(mddev_t *mddev)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);
	int i;

	rcu_read_lock();
	for (i=0; i<mddev->raid_disks; i++) {
		mdk_rdev_t *rdev = rcu_dereference(conf->disks[i].rdev);
		if (rdev && !test_bit(Faulty, &rdev->flags) && atomic_read(&rdev->nr_pending)) {
			struct request_queue *r_queue = bdev_get_queue(rdev->bdev);

			atomic_inc(&rdev->nr_pending);
			rcu_read_unlock();

			blk_unplug(r_queue);

			rdev_dec_pending(rdev, mddev);
			rcu_read_lock();
		}
	}
	rcu_read_unlock();
}

static void raid5_unplug_device(struct request_queue *q)
{
	mddev_t *mddev = q->queuedata;
	raid5_conf_t *conf = mddev_to_conf(mddev);
	unsigned long flags;

	spin_lock_irqsave(&conf->device_lock, flags);

	if (blk_remove_plug(q)) {
		conf->seq_flush++;
		raid5_activate_delayed(conf);
	}

	queue_work(conf->workqueue, &conf->stripe_queue_work);

	spin_unlock_irqrestore(&conf->device_lock, flags);

	unplug_slaves(mddev);
}

static int raid5_congested(void *data, int bits)
{
	mddev_t *mddev = data;
	raid5_conf_t *conf = mddev_to_conf(mddev);

	/* No difference between reads and writes.  Just check
	 * how busy the stripe_queue is
	 */
	if (conf->inactive_queue_blocked)
		return 1;
	if (conf->quiesce)
		return 1;
	if (list_empty_careful(&conf->inactive_q_list))
		return 1;

	return 0;
}

/* We want read requests to align with chunks where possible,
 * but write requests don't need to.
 */
static int raid5_mergeable_bvec(struct request_queue *q, struct bio *bio, struct bio_vec *biovec)
{
	mddev_t *mddev = q->queuedata;
	sector_t sector = bio->bi_sector + get_start_sect(bio->bi_bdev);
	int max;
	unsigned int chunk_sectors = mddev->chunk_size >> 9;
	unsigned int bio_sectors = bio->bi_size >> 9;

	if (bio_data_dir(bio) == WRITE)
		return biovec->bv_len; /* always allow writes to be mergeable */

	max =  (chunk_sectors - ((sector & (chunk_sectors - 1)) + bio_sectors)) << 9;
	if (max < 0) max = 0;
	if (max <= biovec->bv_len && bio_sectors == 0)
		return biovec->bv_len;
	else
		return max;
}


static int in_chunk_boundary(mddev_t *mddev, struct bio *bio)
{
	sector_t sector = bio->bi_sector + get_start_sect(bio->bi_bdev);
	unsigned int chunk_sectors = mddev->chunk_size >> 9;
	unsigned int bio_sectors = bio->bi_size >> 9;

	return  chunk_sectors >=
		((sector & (chunk_sectors - 1)) + bio_sectors);
}

/*
 *  add bio to the retry LIFO  ( in O(1) ... we are in interrupt )
 *  later sampled by raid5d.
 */
static void add_bio_to_retry(struct bio *bi,raid5_conf_t *conf)
{
	unsigned long flags;

	spin_lock_irqsave(&conf->device_lock, flags);

	bi->bi_next = conf->retry_read_aligned_list;
	conf->retry_read_aligned_list = bi;

	spin_unlock_irqrestore(&conf->device_lock, flags);
	md_wakeup_thread(conf->mddev->thread);
}


static struct bio *remove_bio_from_retry(raid5_conf_t *conf)
{
	struct bio *bi;

	bi = conf->retry_read_aligned;
	if (bi) {
		conf->retry_read_aligned = NULL;
		return bi;
	}
	bi = conf->retry_read_aligned_list;
	if(bi) {
		conf->retry_read_aligned_list = bi->bi_next;
		bi->bi_next = NULL;
		bi->bi_phys_segments = 1; /* biased count of active stripes */
		bi->bi_hw_segments = 0; /* count of processed stripes */
	}

	return bi;
}


/*
 *  The "raid5_align_endio" should check if the read succeeded and if it
 *  did, call bio_endio on the original bio (having bio_put the new bio
 *  first).
 *  If the read failed..
 */
static void raid5_align_endio(struct bio *bi, int error)
{
	struct bio* raid_bi  = bi->bi_private;
	mddev_t *mddev;
	raid5_conf_t *conf;
	int uptodate = test_bit(BIO_UPTODATE, &bi->bi_flags);
	mdk_rdev_t *rdev;

	bio_put(bi);

	mddev = raid_bi->bi_bdev->bd_disk->queue->queuedata;
	conf = mddev_to_conf(mddev);
	rdev = (void*)raid_bi->bi_next;
	raid_bi->bi_next = NULL;

	rdev_dec_pending(rdev, conf->mddev);

	if (!error && uptodate) {
		bio_endio(raid_bi, 0);
		if (atomic_dec_and_test(&conf->active_aligned_reads))
			wake_up(&conf->wait_for_queue);
		return;
	}


	pr_debug("raid5_align_endio : io error...handing IO for a retry\n");

	add_bio_to_retry(raid_bi, conf);
}

static int bio_fits_rdev(struct bio *bi)
{
	struct request_queue *q = bdev_get_queue(bi->bi_bdev);

	if ((bi->bi_size>>9) > q->max_sectors)
		return 0;
	blk_recount_segments(q, bi);
	if (bi->bi_phys_segments > q->max_phys_segments ||
	    bi->bi_hw_segments > q->max_hw_segments)
		return 0;

	if (q->merge_bvec_fn)
		/* it's too hard to apply the merge_bvec_fn at this stage,
		 * just just give up
		 */
		return 0;

	return 1;
}


static int chunk_aligned_read(struct request_queue *q, struct bio * raid_bio)
{
	mddev_t *mddev = q->queuedata;
	raid5_conf_t *conf = mddev_to_conf(mddev);
	const unsigned int raid_disks = conf->raid_disks;
	const unsigned int data_disks = raid_disks - conf->max_degraded;
	unsigned int dd_idx, pd_idx;
	struct bio* align_bi;
	mdk_rdev_t *rdev;

	if (!in_chunk_boundary(mddev, raid_bio)) {
		pr_debug("chunk_aligned_read : non aligned\n");
		return 0;
	}
	/*
 	 * use bio_clone to make a copy of the bio
	 */
	align_bi = bio_clone(raid_bio, GFP_NOIO);
	if (!align_bi)
		return 0;
	/*
	 *   set bi_end_io to a new function, and set bi_private to the
	 *     original bio.
	 */
	align_bi->bi_end_io  = raid5_align_endio;
	align_bi->bi_private = raid_bio;
	/*
	 *	compute position
	 */
	align_bi->bi_sector =  raid5_compute_sector(raid_bio->bi_sector,
					raid_disks,
					data_disks,
					&dd_idx,
					&pd_idx,
					conf);

	rcu_read_lock();
	rdev = rcu_dereference(conf->disks[dd_idx].rdev);
	if (rdev && test_bit(In_sync, &rdev->flags)) {
		atomic_inc(&rdev->nr_pending);
		rcu_read_unlock();
		raid_bio->bi_next = (void*)rdev;
		align_bi->bi_bdev =  rdev->bdev;
		align_bi->bi_flags &= ~(1 << BIO_SEG_VALID);
		align_bi->bi_sector += rdev->data_offset;

		if (!bio_fits_rdev(align_bi)) {
			/* too big in some way */
			bio_put(align_bi);
			rdev_dec_pending(rdev, mddev);
			return 0;
		}

		spin_lock_irq(&conf->device_lock);
		wait_event_lock_irq(conf->wait_for_queue,
				    conf->quiesce == 0,
				    conf->device_lock, /* nothing */);
		atomic_inc(&conf->active_aligned_reads);
		spin_unlock_irq(&conf->device_lock);

		generic_make_request(align_bi);
		return 1;
	} else {
		rcu_read_unlock();
		bio_put(align_bi);
		return 0;
	}
}


static int make_request(struct request_queue *q, struct bio * bi)
{
	mddev_t *mddev = q->queuedata;
	raid5_conf_t *conf = mddev_to_conf(mddev);
	unsigned int dd_idx, pd_idx;
	sector_t new_sector;
	sector_t logical_sector, last_sector;
	struct stripe_queue *sq;
	const int rw = bio_data_dir(bi);
	int remaining;

	if (unlikely(bio_barrier(bi))) {
		bio_endio(bi, -EOPNOTSUPP);
		return 0;
	}

	md_write_start(mddev, bi);

	disk_stat_inc(mddev->gendisk, ios[rw]);
	disk_stat_add(mddev->gendisk, sectors[rw], bio_sectors(bi));

	if (rw == READ &&
	     mddev->reshape_position == MaxSector &&
	     chunk_aligned_read(q,bi))
            	return 0;

	logical_sector = bi->bi_sector & ~((sector_t)STRIPE_SECTORS-1);
	last_sector = bi->bi_sector + (bi->bi_size>>9);
	bi->bi_next = NULL;
	bi->bi_phys_segments = 1;	/* over-loaded to count active stripes */

	for (;logical_sector < last_sector; logical_sector += STRIPE_SECTORS) {
		DEFINE_WAIT(w);
		int disks, data_disks;

	retry:
		prepare_to_wait(&conf->wait_for_overlap, &w, TASK_UNINTERRUPTIBLE);
		if (likely(conf->expand_progress == MaxSector))
			disks = conf->raid_disks;
		else {
			/* spinlock is needed as expand_progress may be
			 * 64bit on a 32bit platform, and so it might be
			 * possible to see a half-updated value
			 * Ofcourse expand_progress could change after
			 * the lock is dropped, so once we get a reference
			 * to the stripe that we think it is, we will have
			 * to check again.
			 */
			spin_lock_irq(&conf->device_lock);
			disks = conf->raid_disks;
			if (logical_sector >= conf->expand_progress)
				disks = conf->previous_raid_disks;
			else {
				if (logical_sector >= conf->expand_lo) {
					spin_unlock_irq(&conf->device_lock);
					schedule();
					goto retry;
				}
			}
			spin_unlock_irq(&conf->device_lock);
		}
		data_disks = disks - conf->max_degraded;

 		new_sector = raid5_compute_sector(logical_sector, disks, data_disks,
						  &dd_idx, &pd_idx, conf);
		pr_debug("raid5: make_request, sector %llu logical %llu\n",
			(unsigned long long)new_sector,
			(unsigned long long)logical_sector);

		sq = get_active_queue(conf, new_sector, disks, pd_idx,
					(bi->bi_rw & RWA_MASK));
		if (sq) {
			if (unlikely(conf->expand_progress != MaxSector)) {
				/* expansion might have moved on while waiting for a
				 * queue, so we must do the range check again.
				 * Expansion could still move past after this
				 * test, but as we are holding a reference to
				 * 'sq', we know that if that happens,
				 * STRIPE_QUEUE_EXPANDING will get set and the
				 * expansion won't proceed until we finish
				 * with the queue.
				 */
				int must_retry = 0;
				spin_lock_irq(&conf->device_lock);
				if (logical_sector <  conf->expand_progress &&
				    disks == conf->previous_raid_disks)
					/* mismatch, need to try again */
					must_retry = 1;
				spin_unlock_irq(&conf->device_lock);
				if (must_retry) {
					release_queue(sq);
					goto retry;
				}
			}
			/* FIXME what if we get a false positive because these
			 * are being updated.
			 */
			if (logical_sector >= mddev->suspend_lo &&
			    logical_sector < mddev->suspend_hi) {
				release_queue(sq);
				schedule();
				goto retry;
			}

			if (test_bit(STRIPE_QUEUE_EXPANDING, &sq->state) ||
			    !add_queue_bio(sq, bi, dd_idx,
					   bi->bi_rw & RW_MASK)) {
				/* Stripe is busy expanding or
				 * add failed due to overlap.  Flush everything
				 * and wait a while
				 */
				raid5_unplug_device(mddev->queue);
				release_queue(sq);
				schedule();
				goto retry;
			}
			finish_wait(&conf->wait_for_overlap, &w);
			handle_queue(sq, disks, data_disks);
			release_queue(sq);
		} else {
			/* cannot get queue for read-ahead, just give-up */
			clear_bit(BIO_UPTODATE, &bi->bi_flags);
			finish_wait(&conf->wait_for_overlap, &w);
			break;
		}
			
	}
	spin_lock_irq(&conf->device_lock);
	remaining = --bi->bi_phys_segments;
	spin_unlock_irq(&conf->device_lock);
	if (remaining == 0) {

		if ( rw == WRITE )
			md_write_end(mddev);

		bi->bi_end_io(bi,
			      test_bit(BIO_UPTODATE, &bi->bi_flags)
			        ? 0 : -EIO);
	}
	return 0;
}

static struct stripe_head *
wait_for_inactive_cache(raid5_conf_t *conf, sector_t sector,
			int disks, int pd_idx)
{
	struct stripe_head *sh;

	do {
		struct stripe_queue *sq;
		wait_queue_t wait;
		init_waitqueue_entry(&wait, current);
		add_wait_queue(&conf->wait_for_stripe, &wait);
		for (;;) {
			sq = get_active_queue(conf, sector, disks, pd_idx, 0);
			set_current_state(TASK_UNINTERRUPTIBLE);
			sh = get_active_stripe(sq, disks, 1);
			if (sh)
				break;
			release_queue(sq);
			schedule();
		}
		current->state = TASK_RUNNING;
		remove_wait_queue(&conf->wait_for_stripe, &wait);
	} while (0);

	return sh;
}


static sector_t reshape_request(mddev_t *mddev, sector_t sector_nr, int *skipped)
{
	/* reshaping is quite different to recovery/resync so it is
	 * handled quite separately ... here.
	 *
	 * On each call to sync_request, we gather one chunk worth of
	 * destination stripes and flag them as expanding.
	 * Then we find all the source stripes and request reads.
	 * As the reads complete, handle_stripe will copy the data
	 * into the destination stripe and release that stripe.
	 */
	raid5_conf_t *conf = (raid5_conf_t *) mddev->private;
	struct stripe_head *sh;
	struct stripe_queue *sq;
	int pd_idx;
	sector_t first_sector, last_sector;
	int raid_disks = conf->previous_raid_disks;
	int data_disks = raid_disks - conf->max_degraded;
	int new_data_disks = conf->raid_disks - conf->max_degraded;
	int i;
	int dd_idx;
	sector_t writepos, safepos, gap;

	if (sector_nr == 0 &&
	    conf->expand_progress != 0) {
		/* restarting in the middle, skip the initial sectors */
		sector_nr = conf->expand_progress;
		sector_div(sector_nr, new_data_disks);
		*skipped = 1;
		return sector_nr;
	}

	/* we update the metadata when there is more than 3Meg
	 * in the block range (that is rather arbitrary, should
	 * probably be time based) or when the data about to be
	 * copied would over-write the source of the data at
	 * the front of the range.
	 * i.e. one new_stripe forward from expand_progress new_maps
	 * to after where expand_lo old_maps to
	 */
	writepos = conf->expand_progress +
		conf->chunk_size/512*(new_data_disks);
	sector_div(writepos, new_data_disks);
	safepos = conf->expand_lo;
	sector_div(safepos, data_disks);
	gap = conf->expand_progress - conf->expand_lo;

	if (writepos >= safepos ||
	    gap > (new_data_disks)*3000*2 /*3Meg*/) {
		/* Cannot proceed until we've updated the superblock... */
		wait_event(conf->wait_for_overlap,
			   atomic_read(&conf->reshape_stripes)==0);
		mddev->reshape_position = conf->expand_progress;
		set_bit(MD_CHANGE_DEVS, &mddev->flags);
		md_wakeup_thread(mddev->thread);
		wait_event(mddev->sb_wait, mddev->flags == 0 ||
			   kthread_should_stop());
		spin_lock_irq(&conf->device_lock);
		conf->expand_lo = mddev->reshape_position;
		spin_unlock_irq(&conf->device_lock);
		wake_up(&conf->wait_for_overlap);
	}

	for (i=0; i < conf->chunk_size/512; i+= STRIPE_SECTORS) {
		int j;
		int skipped = 0;
		pd_idx = stripe_to_pdidx(sector_nr+i, conf, conf->raid_disks);
		sh = wait_for_inactive_cache(conf, sector_nr+i,
					     conf->raid_disks, pd_idx);
		sq = sh->sq;

		set_bit(STRIPE_QUEUE_EXPANDING, &sq->state);
		atomic_inc(&conf->reshape_stripes);
		/* If any of this stripe is beyond the end of the old
		 * array, then we need to zero those blocks
		 */
		for (j = sq->disks; j--;) {
			sector_t s;
			int pd_idx = sq->pd_idx;

			if (j == pd_idx)
				continue;
			if (conf->level == 6 &&
			    j == raid6_next_disk(pd_idx, sq->disks))
				continue;
			s = compute_blocknr(conf, sq->disks, sh->sector,
					    pd_idx, j);
			if (s < (mddev->array_size<<1)) {
				skipped = 1;
				continue;
			}
			memset(page_address(sh->dev[j].page), 0, STRIPE_SIZE);
			set_bit(R5_Expanded, &sh->dev[j].flags);
			set_bit(R5_UPTODATE, &sh->dev[j].flags);
		}
		if (!skipped) {
			set_bit(STRIPE_EXPAND_READY, &sh->state);
			set_bit(STRIPE_HANDLE, &sh->state);
		}
		release_stripe(sh);
		release_queue(sq);
	}
	spin_lock_irq(&conf->device_lock);
	conf->expand_progress = (sector_nr + i) * new_data_disks;
	spin_unlock_irq(&conf->device_lock);
	/* Ok, those stripe are ready. We can start scheduling
	 * reads on the source stripes.
	 * The source stripes are determined by mapping the first and last
	 * block on the destination stripes.
	 */
	first_sector =
		raid5_compute_sector(sector_nr*(new_data_disks),
				     raid_disks, data_disks,
				     &dd_idx, &pd_idx, conf);
	last_sector =
		raid5_compute_sector((sector_nr+conf->chunk_size/512)
				     *(new_data_disks) -1,
				     raid_disks, data_disks,
				     &dd_idx, &pd_idx, conf);
	if (last_sector >= (mddev->size<<1))
		last_sector = (mddev->size<<1)-1;
	while (first_sector <= last_sector) {
		pd_idx = stripe_to_pdidx(first_sector, conf,
					 conf->previous_raid_disks);
		sh = wait_for_inactive_cache(conf, first_sector,
					     conf->previous_raid_disks,
					     pd_idx);
		sq = sh->sq;
		set_bit(STRIPE_EXPAND_SOURCE, &sh->state);
		set_bit(STRIPE_HANDLE, &sh->state);
		release_stripe(sh);
		release_queue(sq);
		first_sector += STRIPE_SECTORS;
	}
	/* If this takes us to the resync_max point where we have to pause,
	 * then we need to write out the superblock.
	 */
	sector_nr += conf->chunk_size>>9;
	if (sector_nr >= mddev->resync_max) {
		/* Cannot proceed until we've updated the superblock... */
		wait_event(conf->wait_for_overlap,
			   atomic_read(&conf->reshape_stripes) == 0);
		mddev->reshape_position = conf->expand_progress;
		set_bit(MD_CHANGE_DEVS, &mddev->flags);
		md_wakeup_thread(mddev->thread);
		wait_event(mddev->sb_wait,
			   !test_bit(MD_CHANGE_DEVS, &mddev->flags)
			   || kthread_should_stop());
		spin_lock_irq(&conf->device_lock);
		conf->expand_lo = mddev->reshape_position;
		spin_unlock_irq(&conf->device_lock);
		wake_up(&conf->wait_for_overlap);
	}
	return conf->chunk_size>>9;
}

/* FIXME go_faster isn't used */
static inline sector_t sync_request(mddev_t *mddev, sector_t sector_nr, int *skipped, int go_faster)
{
	raid5_conf_t *conf = (raid5_conf_t *) mddev->private;
	struct stripe_head *sh;
	struct stripe_queue *sq;
	int pd_idx;
	int raid_disks = conf->raid_disks;
	sector_t max_sector = mddev->size << 1;
	int sync_blocks;
	int still_degraded = 0;
	int i;

	if (sector_nr >= max_sector) {
		/* just being told to finish up .. nothing much to do */
		unplug_slaves(mddev);
		if (test_bit(MD_RECOVERY_RESHAPE, &mddev->recovery)) {
			end_reshape(conf);
			return 0;
		}

		if (mddev->curr_resync < max_sector) /* aborted */
			bitmap_end_sync(mddev->bitmap, mddev->curr_resync,
					&sync_blocks, 1);
		else /* completed sync */
			conf->fullsync = 0;
		bitmap_close_sync(mddev->bitmap);

		return 0;
	}

	if (test_bit(MD_RECOVERY_RESHAPE, &mddev->recovery))
		return reshape_request(mddev, sector_nr, skipped);

	/* No need to check resync_max as we never do more than one
	 * stripe, and as resync_max will always be on a chunk boundary,
	 * if the check in md_do_sync didn't fire, there is no chance
	 * of overstepping resync_max here
	 */

	/* if there is too many failed drives and we are trying
	 * to resync, then assert that we are finished, because there is
	 * nothing we can do.
	 */
	if (mddev->degraded >= conf->max_degraded &&
	    test_bit(MD_RECOVERY_SYNC, &mddev->recovery)) {
		sector_t rv = (mddev->size << 1) - sector_nr;
		*skipped = 1;
		return rv;
	}
	if (!bitmap_start_sync(mddev->bitmap, sector_nr, &sync_blocks, 1) &&
	    !test_bit(MD_RECOVERY_REQUESTED, &mddev->recovery) &&
	    !conf->fullsync && sync_blocks >= STRIPE_SECTORS) {
		/* we can skip this block, and probably more */
		sync_blocks /= STRIPE_SECTORS;
		*skipped = 1;
		return sync_blocks * STRIPE_SECTORS; /* keep things rounded to whole stripes */
	}


	bitmap_cond_end_sync(mddev->bitmap, sector_nr);

	pd_idx = stripe_to_pdidx(sector_nr, conf, raid_disks);

	sh = wait_for_inactive_cache(conf, sector_nr, raid_disks, pd_idx);
	sq = sh->sq;

	/* Need to check if array will still be degraded after recovery/resync
	 * We don't need to check the 'failed' flag as when that gets set,
	 * recovery aborts.
	 */
	for (i=0; i<mddev->raid_disks; i++)
		if (conf->disks[i].rdev == NULL)
			still_degraded = 1;

	bitmap_start_sync(mddev->bitmap, sector_nr, &sync_blocks, still_degraded);

	spin_lock(&sq->lock);
	set_bit(STRIPE_SYNCING, &sh->state);
	clear_bit(STRIPE_INSYNC, &sh->state);
	spin_unlock(&sq->lock);

	handle_stripe(sh);
	release_stripe(sh);
	release_queue(sq);

	return STRIPE_SECTORS;
}

static int  retry_aligned_read(raid5_conf_t *conf, struct bio *raid_bio)
{
	/* We may not be able to submit a whole bio at once as there
	 * may not be enough stripe_heads available.
	 * We cannot pre-allocate enough stripe_heads as we may need
	 * more than exist in the cache (if we allow ever large chunks).
	 * So we do one stripe head at a time and record in
	 * ->bi_hw_segments how many have been done.
	 *
	 * We *know* that this entire raid_bio is in one chunk, so
	 * it will be only one 'dd_idx' and only need one call to raid5_compute_sector.
	 */
	struct stripe_queue *sq;
	int dd_idx, pd_idx;
	sector_t sector, logical_sector, last_sector;
	int scnt = 0;
	int remaining;
	int handled = 0;
	int disks = conf->raid_disks;
	int data_disks = disks - conf->max_degraded;

	logical_sector = raid_bio->bi_sector & ~((sector_t)STRIPE_SECTORS-1);
	sector = raid5_compute_sector(	logical_sector,
					disks,
					data_disks,
					&dd_idx,
					&pd_idx,
					conf);
	last_sector = raid_bio->bi_sector + (raid_bio->bi_size>>9);

	for (; logical_sector < last_sector;
	     logical_sector += STRIPE_SECTORS,
		     sector += STRIPE_SECTORS,
		     scnt++) {
		struct stripe_head *sh;

		if (scnt < raid_bio->bi_hw_segments)
			/* already done this stripe */
			continue;

		sq = get_active_queue(conf, sector, disks, pd_idx, 1);
		if (sq)
			sh = get_active_stripe(sq, disks, 1);
		if (!(sq && sh)) {
			/* failed to get a queue/stripe - must wait */
			raid_bio->bi_hw_segments = scnt;
			conf->retry_read_aligned = raid_bio;
			if (sq)
				release_queue(sq);
			return handled;
		}

		set_bit(R5_ReadError, &sh->dev[dd_idx].flags);
		if (!add_queue_bio(sq, raid_bio, dd_idx, 0)) {
			release_stripe(sh);
			release_queue(sq);
			raid_bio->bi_hw_segments = scnt;
			conf->retry_read_aligned = raid_bio;
			return handled;
		}

		handle_queue(sq, disks, data_disks);
		release_stripe(sh);
		release_queue(sq);
		handled++;
	}
	spin_lock_irq(&conf->device_lock);
	remaining = --raid_bio->bi_phys_segments;
	spin_unlock_irq(&conf->device_lock);
	if (remaining == 0) {

		raid_bio->bi_end_io(raid_bio,
			      test_bit(BIO_UPTODATE, &raid_bio->bi_flags)
			        ? 0 : -EIO);
	}
	if (atomic_dec_and_test(&conf->active_aligned_reads))
		wake_up(&conf->wait_for_queue);
	return handled;
}

static void raid456_cache_arbiter(struct work_struct *work)
{
	raid5_conf_t *conf = container_of(work, raid5_conf_t,
					  stripe_queue_work);
	struct list_head *sq_entry;
	int attach = 0;

	/* attach queues to stripes in priority order */
	pr_debug("+++ %s active\n", __FUNCTION__);
	spin_lock_irq(&conf->device_lock);
	do {
		sq_entry = NULL;
		if (!list_empty(&conf->io_hi_q_list))
			sq_entry = conf->io_hi_q_list.next;
		else if (!list_empty(&conf->io_lo_q_list))
			sq_entry = conf->io_lo_q_list.next;

		/* "these aren't the droids you're looking for..."
		 * do not handle the delayed list while there are better
		 * things to do
		 */
		if (!sq_entry) {
			if (conf->seq_flush != conf->seq_write) {
				int seq = conf->seq_flush;
				spin_unlock_irq(&conf->device_lock);
				bitmap_unplug(conf->mddev->bitmap);
				spin_lock_irq(&conf->device_lock);
				conf->seq_write = seq;
				activate_bit_delay(conf);
			}
		} else {
			struct stripe_queue *sq;
			struct stripe_head *sh;
			sq = list_entry(sq_entry, struct stripe_queue,
					list_node);

			list_del_init(sq_entry);
			atomic_inc(&sq->count);
			BUG_ON(atomic_read(&sq->count) != 1);

			spin_unlock_irq(&conf->device_lock);
			sh = get_active_stripe(sq, conf->raid_disks, 0);
			spin_lock_irq(&conf->device_lock);
			if (sh) {
				attach++;
				set_bit(STRIPE_HANDLE, &sh->state);
				__release_stripe(conf, sh);
			}
			__release_queue(conf, sq);
		}
	} while (sq_entry);
	spin_unlock_irq(&conf->device_lock);
	pr_debug("%d stripe(s) attached\n", attach);
	pr_debug("--- %s inactive\n", __FUNCTION__);
}

/*
 * This is our raid5 kernel thread.
 *
 * We scan the hash table for stripes which can be handled now.
 * During the scan, completed stripes are saved for us by the interrupt
 * handler, so that they will not have to wait for our next wakeup.
 */
static void raid5d(mddev_t *mddev)
{
	struct stripe_head *sh;
	raid5_conf_t *conf = mddev_to_conf(mddev);
	int handled;

	pr_debug("+++ raid5d active\n");

	md_check_recovery(mddev);

	handled = 0;
	spin_lock_irq(&conf->device_lock);
	while (1) {
		struct list_head *first;
		struct bio *bio;

		while ((bio = remove_bio_from_retry(conf))) {
			int ok;
			spin_unlock_irq(&conf->device_lock);
			ok = retry_aligned_read(conf, bio);
			spin_lock_irq(&conf->device_lock);
			if (!ok)
				break;
		}

		if (list_empty(&conf->handle_list)) {
			queue_work(conf->workqueue, &conf->stripe_queue_work);
			async_tx_issue_pending_all();
			break;
		}

		first = conf->handle_list.next;
		sh = list_entry(first, struct stripe_head, lru);

		list_del_init(first);
		atomic_inc(&sh->count);
		BUG_ON(atomic_read(&sh->count)!= 1);
		spin_unlock_irq(&conf->device_lock);
		
		handled++;
		handle_stripe(sh);
		release_stripe(sh);

		spin_lock_irq(&conf->device_lock);
	}
	pr_debug("%d stripes handled\n", handled);

	spin_unlock_irq(&conf->device_lock);

	unplug_slaves(mddev);

	pr_debug("--- raid5d inactive\n");
}

static ssize_t
raid5_show_stripe_cache_size(mddev_t *mddev, char *page)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);
	if (conf)
		return sprintf(page, "%d\n", conf->max_nr_stripes);
	else
		return 0;
}

static ssize_t
raid5_store_stripe_cache_size(mddev_t *mddev, const char *page, size_t len)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);
	char *end;
	int new, queue, i;

	if (len >= PAGE_SIZE)
		return -EINVAL;
	if (!conf)
		return -ENODEV;

	new = simple_strtoul(page, &end, 10);
	if (!*page || (*end && *end != '\n') )
		return -EINVAL;
	if (new <= 16 || new > 32768)
		return -EINVAL;
	while (new < conf->max_nr_stripes) {
		if (drop_one_stripe(conf))
			conf->max_nr_stripes--;
		else
			break;

		for (i = 0, queue = 0; i < STRIPE_QUEUE_SIZE; i++)
			queue += drop_one_queue(conf);

		if (queue < STRIPE_QUEUE_SIZE)
			break;
	}
	md_allow_write(mddev);
	while (new > conf->max_nr_stripes) {
		for (i = 0, queue = 0; i < STRIPE_QUEUE_SIZE; i++)
			queue += grow_one_queue(conf);

		if (queue < STRIPE_QUEUE_SIZE)
			break;

		if (grow_one_stripe(conf))
			conf->max_nr_stripes++;
		else break;
	}
	return len;
}

static struct md_sysfs_entry
raid5_stripecache_size = __ATTR(stripe_cache_size, S_IRUGO | S_IWUSR,
				raid5_show_stripe_cache_size,
				raid5_store_stripe_cache_size);

static ssize_t
stripe_cache_active_show(mddev_t *mddev, char *page)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);
	if (conf)
		return sprintf(page, "%d\n", atomic_read(&conf->active_stripes));
	else
		return 0;
}

static struct md_sysfs_entry
raid5_stripecache_active = __ATTR_RO(stripe_cache_active);

static ssize_t
stripe_queue_active_show(mddev_t *mddev, char *page)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);
	if (conf)
		return sprintf(page, "%d\n", atomic_read(&conf->active_queues));
	else
		return 0;
}

static struct md_sysfs_entry
raid5_stripequeue_active = __ATTR_RO(stripe_queue_active);

static struct attribute *raid5_attrs[] =  {
	&raid5_stripecache_size.attr,
	&raid5_stripecache_active.attr,
	&raid5_stripequeue_active.attr,
	NULL,
};
static struct attribute_group raid5_attrs_group = {
	.name = NULL,
	.attrs = raid5_attrs,
};

static int run(mddev_t *mddev)
{
	raid5_conf_t *conf;
	int raid_disk, memory, avail_stripes;
	mdk_rdev_t *rdev;
	struct disk_info *disk;
	struct list_head *tmp;
	struct sysinfo val;
	int working_disks = 0;

	if (mddev->level != 5 && mddev->level != 4 && mddev->level != 6) {
		printk(KERN_ERR "raid5: %s: raid level not set to 4/5/6 (%d)\n",
		       mdname(mddev), mddev->level);
		return -EIO;
	}

	if (mddev->reshape_position != MaxSector) {
		/* Check that we can continue the reshape.
		 * Currently only disks can change, it must
		 * increase, and we must be past the point where
		 * a stripe over-writes itself
		 */
		sector_t here_new, here_old;
		int old_disks;
		int max_degraded = (mddev->level == 5 ? 1 : 2);

		if (mddev->new_level != mddev->level ||
		    mddev->new_layout != mddev->layout ||
		    mddev->new_chunk != mddev->chunk_size) {
			printk(KERN_ERR "raid5: %s: unsupported reshape "
			       "required - aborting.\n",
			       mdname(mddev));
			return -EINVAL;
		}
		if (mddev->delta_disks <= 0) {
			printk(KERN_ERR "raid5: %s: unsupported reshape "
			       "(reduce disks) required - aborting.\n",
			       mdname(mddev));
			return -EINVAL;
		}
		old_disks = mddev->raid_disks - mddev->delta_disks;
		/* reshape_position must be on a new-stripe boundary, and one
		 * further up in new geometry must map after here in old
		 * geometry.
		 */
		here_new = mddev->reshape_position;
		if (sector_div(here_new, (mddev->chunk_size>>9)*
			       (mddev->raid_disks - max_degraded))) {
			printk(KERN_ERR "raid5: reshape_position not "
			       "on a stripe boundary\n");
			return -EINVAL;
		}
		/* here_new is the stripe we will write to */
		here_old = mddev->reshape_position;
		sector_div(here_old, (mddev->chunk_size>>9)*
			   (old_disks-max_degraded));
		/* here_old is the first stripe that we might need to read
		 * from */
		if (here_new >= here_old) {
			/* Reading from the same stripe as writing to - bad */
			printk(KERN_ERR "raid5: reshape_position too early for "
			       "auto-recovery - aborting.\n");
			return -EINVAL;
		}
		printk(KERN_INFO "raid5: reshape will continue\n");
		/* OK, we should be able to continue; */
	}


	mddev->private = kzalloc(sizeof (raid5_conf_t), GFP_KERNEL);
	if ((conf = mddev->private) == NULL)
		goto abort;
	if (mddev->reshape_position == MaxSector) {
		conf->previous_raid_disks = conf->raid_disks = mddev->raid_disks;
	} else {
		conf->raid_disks = mddev->raid_disks;
		conf->previous_raid_disks = mddev->raid_disks - mddev->delta_disks;
	}

	conf->disks = kzalloc(conf->raid_disks * sizeof(struct disk_info),
			      GFP_KERNEL);
	if (!conf->disks)
		goto abort;

	conf->mddev = mddev;

	if ((conf->stripe_hashtbl = kzalloc(PAGE_SIZE, GFP_KERNEL)) == NULL)
		goto abort;

	sprintf(conf->workqueue_name, "%s_cache_arb",
		mddev->gendisk->disk_name);
	conf->workqueue = create_singlethread_workqueue(conf->workqueue_name);
	if (!conf->workqueue)
		goto abort;

	spin_lock_init(&conf->device_lock);
	init_waitqueue_head(&conf->wait_for_stripe);
	init_waitqueue_head(&conf->wait_for_queue);
	init_waitqueue_head(&conf->wait_for_overlap);
	INIT_LIST_HEAD(&conf->handle_list);
	INIT_LIST_HEAD(&conf->inactive_list);
	INIT_LIST_HEAD(&conf->bitmap_q_list);
	INIT_LIST_HEAD(&conf->io_hi_q_list);
	INIT_LIST_HEAD(&conf->io_lo_q_list);
	INIT_LIST_HEAD(&conf->delayed_q_list);
	INIT_LIST_HEAD(&conf->inactive_q_list);
	atomic_set(&conf->active_stripes, 0);
	atomic_set(&conf->active_queues, 0);
	atomic_set(&conf->preread_active_queues, 0);
	atomic_set(&conf->active_aligned_reads, 0);
	INIT_WORK(&conf->stripe_queue_work, raid456_cache_arbiter);

	pr_debug("raid5: run(%s) called.\n", mdname(mddev));

	rdev_for_each(rdev, tmp, mddev) {
		raid_disk = rdev->raid_disk;
		if (raid_disk >= conf->raid_disks
		    || raid_disk < 0)
			continue;
		disk = conf->disks + raid_disk;

		disk->rdev = rdev;

		if (test_bit(In_sync, &rdev->flags)) {
			char b[BDEVNAME_SIZE];
			printk(KERN_INFO "raid5: device %s operational as raid"
				" disk %d\n", bdevname(rdev->bdev,b),
				raid_disk);
			working_disks++;
		}
	}

	/*
	 * 0 for a fully functional array, 1 or 2 for a degraded array.
	 */
	mddev->degraded = conf->raid_disks - working_disks;
	conf->mddev = mddev;
	conf->chunk_size = mddev->chunk_size;
	conf->level = mddev->level;
	if (conf->level == 6)
		conf->max_degraded = 2;
	else
		conf->max_degraded = 1;
	conf->algorithm = mddev->layout;

	/* Set the safe stripe cache size */
	si_meminfo(&val);
	avail_stripes = (val.freeram + val.freehigh) * val.mem_unit /
			(sizeof(struct stripe_head) + conf->raid_disks *
			((sizeof(struct bio) + PAGE_SIZE))) - 1;
	conf->max_nr_stripes = (avail_stripes < NR_STRIPES) ? avail_stripes :
			       NR_STRIPES;
	conf->expand_progress = mddev->reshape_position;

	/* device size must be a multiple of chunk size */
	mddev->size &= ~(mddev->chunk_size/1024 -1);
	mddev->resync_max_sectors = mddev->size << 1;

	if (conf->level == 6 && conf->raid_disks < 4) {
		printk(KERN_ERR "raid6: not enough configured devices for %s (%d, minimum 4)\n",
		       mdname(mddev), conf->raid_disks);
		goto abort;
	}
	if (!conf->chunk_size || conf->chunk_size % 4) {
		printk(KERN_ERR "raid5: invalid chunk size %d for %s\n",
			conf->chunk_size, mdname(mddev));
		goto abort;
	}
	if (conf->algorithm > ALGORITHM_RIGHT_SYMMETRIC) {
		printk(KERN_ERR
			"raid5: unsupported parity algorithm %d for %s\n",
			conf->algorithm, mdname(mddev));
		goto abort;
	}
	if (mddev->degraded > conf->max_degraded) {
		printk(KERN_ERR "raid5: not enough operational devices for %s"
			" (%d/%d failed)\n",
			mdname(mddev), mddev->degraded, conf->raid_disks);
		goto abort;
	}

	if (mddev->degraded > 0 &&
	    mddev->recovery_cp != MaxSector) {
		if (mddev->ok_start_degraded)
			printk(KERN_WARNING
			       "raid5: starting dirty degraded array: %s"
			       "- data corruption possible.\n",
			       mdname(mddev));
		else {
			printk(KERN_ERR
			       "raid5: cannot start dirty degraded array for %s\n",
			       mdname(mddev));
			goto abort;
		}
	}

	{
		mddev->thread = md_register_thread(raid5d, mddev, "%s_raid5");
		if (!mddev->thread) {
			printk(KERN_ERR
				"raid5: couldn't allocate thread for %s\n",
				mdname(mddev));
			goto abort;
		}
	}
	memory = conf->max_nr_stripes * (sizeof(struct stripe_head) +
		 conf->raid_disks * ((sizeof(struct bio) + PAGE_SIZE))) / 1024;
	if (grow_stripes(conf, conf->max_nr_stripes)) {
		printk(KERN_ERR
			"raid5: couldn't allocate %dkB for buffers\n", memory);
		shrink_stripes(conf);
		md_unregister_thread(mddev->thread);
		goto abort;
	} else
		printk(KERN_INFO "raid5: allocated %dkB for %s\n",
			memory, mdname(mddev));

	conf->stripe_queue_tree = RB_ROOT;

	if (mddev->degraded == 0)
		printk("raid5: raid level %d set %s active with %d out of %d"
			" devices, algorithm %d\n", conf->level, mdname(mddev),
			mddev->raid_disks-mddev->degraded, mddev->raid_disks,
			conf->algorithm);
	else
		printk(KERN_ALERT "raid5: raid level %d set %s active with %d"
			" out of %d devices, algorithm %d\n", conf->level,
			mdname(mddev), mddev->raid_disks - mddev->degraded,
			mddev->raid_disks, conf->algorithm);

	print_raid5_conf(conf);

	if (conf->expand_progress != MaxSector) {
		printk("...ok start reshape thread\n");
		conf->expand_lo = conf->expand_progress;
		atomic_set(&conf->reshape_stripes, 0);
		clear_bit(MD_RECOVERY_SYNC, &mddev->recovery);
		clear_bit(MD_RECOVERY_CHECK, &mddev->recovery);
		set_bit(MD_RECOVERY_RESHAPE, &mddev->recovery);
		set_bit(MD_RECOVERY_RUNNING, &mddev->recovery);
		mddev->sync_thread = md_register_thread(md_do_sync, mddev,
							"%s_reshape");
	}

	/* read-ahead size must cover two whole stripes, which is
	 * 2 * (datadisks) * chunksize where 'n' is the number of raid devices
	 */
	{
		int data_disks = conf->previous_raid_disks - conf->max_degraded;
		int stripe = data_disks *
			(mddev->chunk_size / PAGE_SIZE);
		if (mddev->queue->backing_dev_info.ra_pages < 2 * stripe)
			mddev->queue->backing_dev_info.ra_pages = 2 * stripe;
	}

	/* Ok, everything is just fine now */
	if (sysfs_create_group(&mddev->kobj, &raid5_attrs_group))
		printk(KERN_WARNING
		       "raid5: failed to create sysfs attributes for %s\n",
		       mdname(mddev));

	mddev->queue->unplug_fn = raid5_unplug_device;
	mddev->queue->backing_dev_info.congested_data = mddev;
	mddev->queue->backing_dev_info.congested_fn = raid5_congested;

	mddev->array_size =  mddev->size * (conf->previous_raid_disks -
					    conf->max_degraded);

	blk_queue_merge_bvec(mddev->queue, raid5_mergeable_bvec);

	return 0;
abort:
	if (conf) {
		print_raid5_conf(conf);
		if (conf->workqueue)
			destroy_workqueue(conf->workqueue);
		kfree(conf->disks);
		kfree(conf->stripe_hashtbl);
		kfree(conf);
	}
	mddev->private = NULL;
	printk(KERN_ALERT "raid5: failed to run raid set %s\n", mdname(mddev));
	return -EIO;
}



static int stop(mddev_t *mddev)
{
	raid5_conf_t *conf = (raid5_conf_t *) mddev->private;

	md_unregister_thread(mddev->thread);
	mddev->thread = NULL;
	shrink_stripes(conf);
	kfree(conf->stripe_hashtbl);
	mddev->queue->backing_dev_info.congested_fn = NULL;
	blk_sync_queue(mddev->queue); /* the unplug fn references 'conf'*/
	sysfs_remove_group(&mddev->kobj, &raid5_attrs_group);
	kfree(conf->disks);
	destroy_workqueue(conf->workqueue);
	kfree(conf);
	mddev->private = NULL;
	return 0;
}

#ifdef DEBUG
static void print_sh (struct seq_file *seq, struct stripe_head *sh)
{
	int i;

	seq_printf(seq, "sh %llu, state %ld.\n",
		   (unsigned long long)sh->sector, sh->state);
	seq_printf(seq, "sh %llu,  count %d.\n",
		   (unsigned long long)sh->sector, atomic_read(&sh->count));
	seq_printf(seq, "sh %llu, ", (unsigned long long)sh->sector);
	for (i = 0; i < sh->sq->disks; i++) {
		seq_printf(seq, "(cache%d: %p %ld) ",
			   i, sh->dev[i].page, sh->dev[i].flags);
	}
	seq_printf(seq, "\n");
}

static void print_sq(struct seq_file *seq, struct stripe_queue *sq)
{
	int disks = sq->disks;

	seq_printf(seq, "sq %llu, pd_idx %d, state %ld.\n",
		   (unsigned long long)sq->sector, sq->pd_idx, sq->state);
	seq_printf(seq, "sq %llu,  count %d to_write: %lu to_read: %lu "
		   "overwrite: %lu\n", (unsigned long long)sq->sector,
		   atomic_read(&sq->count), io_weight(sq->to_write, disks),
		   io_weight(sq->to_read, disks),
		   io_weight(sq->overwrite, disks));
	seq_printf(seq, "sq %llu, ", (unsigned long long)sq->sector);
}

static void printall(struct seq_file *seq, raid5_conf_t *conf)
{
	struct stripe_queue *sq;
	struct stripe_head *sh;
	struct rb_node *rbn;
	struct hlist_node *hn;
	int i;

	spin_lock_irq(&conf->device_lock);
	rbn = rb_first(&conf->stripe_queue_tree);
	while (rbn) {
		sq = rb_entry(rbn, struct stripe_queue, rb_node);
		print_sq(seq, sq);
		rbn = rb_next(rbn);
	}
	for (i = 0; i < NR_HASH; i++) {
		hlist_for_each_entry(sh, hn, &conf->stripe_hashtbl[i], hash) {
			if (!sh->sq)
				continue;
			print_sh(seq, sh);
		}
	}
	spin_unlock_irq(&conf->device_lock);
}
#endif

static void status (struct seq_file *seq, mddev_t *mddev)
{
	raid5_conf_t *conf = (raid5_conf_t *) mddev->private;
	int i;

	seq_printf (seq, " level %d, %dk chunk, algorithm %d", mddev->level, mddev->chunk_size >> 10, mddev->layout);
	seq_printf (seq, " [%d/%d] [", conf->raid_disks, conf->raid_disks - mddev->degraded);
	for (i = 0; i < conf->raid_disks; i++)
		seq_printf (seq, "%s",
			       conf->disks[i].rdev &&
			       test_bit(In_sync, &conf->disks[i].rdev->flags) ? "U" : "_");
	seq_printf (seq, "]");
#ifdef DEBUG
	seq_printf (seq, "\n");
	printall(seq, conf);
#endif
}

static void print_raid5_conf (raid5_conf_t *conf)
{
	int i;
	struct disk_info *tmp;

	printk("RAID5 conf printout:\n");
	if (!conf) {
		printk("(conf==NULL)\n");
		return;
	}
	printk(" --- rd:%d wd:%d\n", conf->raid_disks,
		 conf->raid_disks - conf->mddev->degraded);

	for (i = 0; i < conf->raid_disks; i++) {
		char b[BDEVNAME_SIZE];
		tmp = conf->disks + i;
		if (tmp->rdev)
		printk(" disk %d, o:%d, dev:%s\n",
			i, !test_bit(Faulty, &tmp->rdev->flags),
			bdevname(tmp->rdev->bdev,b));
	}
}

static int raid5_spare_active(mddev_t *mddev)
{
	int i;
	raid5_conf_t *conf = mddev->private;
	struct disk_info *tmp;

	for (i = 0; i < conf->raid_disks; i++) {
		tmp = conf->disks + i;
		if (tmp->rdev
		    && !test_bit(Faulty, &tmp->rdev->flags)
		    && !test_and_set_bit(In_sync, &tmp->rdev->flags)) {
			unsigned long flags;
			spin_lock_irqsave(&conf->device_lock, flags);
			mddev->degraded--;
			spin_unlock_irqrestore(&conf->device_lock, flags);
		}
	}
	print_raid5_conf(conf);
	return 0;
}

static int raid5_remove_disk(mddev_t *mddev, int number)
{
	raid5_conf_t *conf = mddev->private;
	int err = 0;
	mdk_rdev_t *rdev;
	struct disk_info *p = conf->disks + number;

	print_raid5_conf(conf);
	rdev = p->rdev;
	if (rdev) {
		if (test_bit(In_sync, &rdev->flags) ||
		    atomic_read(&rdev->nr_pending)) {
			err = -EBUSY;
			goto abort;
		}
		p->rdev = NULL;
		synchronize_rcu();
		if (atomic_read(&rdev->nr_pending)) {
			/* lost the race, try later */
			err = -EBUSY;
			p->rdev = rdev;
		}
	}
abort:

	print_raid5_conf(conf);
	return err;
}

static int raid5_add_disk(mddev_t *mddev, mdk_rdev_t *rdev)
{
	raid5_conf_t *conf = mddev->private;
	int found = 0;
	int disk;
	struct disk_info *p;

	if (mddev->degraded > conf->max_degraded)
		/* no point adding a device */
		return 0;

	/*
	 * find the disk ... but prefer rdev->saved_raid_disk
	 * if possible.
	 */
	if (rdev->saved_raid_disk >= 0 &&
	    conf->disks[rdev->saved_raid_disk].rdev == NULL)
		disk = rdev->saved_raid_disk;
	else
		disk = 0;
	for ( ; disk < conf->raid_disks; disk++)
		if ((p=conf->disks + disk)->rdev == NULL) {
			clear_bit(In_sync, &rdev->flags);
			rdev->raid_disk = disk;
			found = 1;
			if (rdev->saved_raid_disk != disk)
				conf->fullsync = 1;
			rcu_assign_pointer(p->rdev, rdev);
			break;
		}
	print_raid5_conf(conf);
	return found;
}

static int raid5_resize(mddev_t *mddev, sector_t sectors)
{
	/* no resync is happening, and there is enough space
	 * on all devices, so we can resize.
	 * We need to make sure resync covers any new space.
	 * If the array is shrinking we should possibly wait until
	 * any io in the removed space completes, but it hardly seems
	 * worth it.
	 */
	raid5_conf_t *conf = mddev_to_conf(mddev);

	sectors &= ~((sector_t)mddev->chunk_size/512 - 1);
	mddev->array_size = (sectors * (mddev->raid_disks-conf->max_degraded))>>1;
	set_capacity(mddev->gendisk, mddev->array_size << 1);
	mddev->changed = 1;
	if (sectors/2  > mddev->size && mddev->recovery_cp == MaxSector) {
		mddev->recovery_cp = mddev->size << 1;
		set_bit(MD_RECOVERY_NEEDED, &mddev->recovery);
	}
	mddev->size = sectors /2;
	mddev->resync_max_sectors = sectors;
	return 0;
}

#ifdef CONFIG_MD_RAID5_RESHAPE
static int raid5_check_reshape(mddev_t *mddev)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);
	int err;

	if (mddev->delta_disks < 0 ||
	    mddev->new_level != mddev->level)
		return -EINVAL; /* Cannot shrink array or change level yet */
	if (mddev->delta_disks == 0)
		return 0; /* nothing to do */

	/* Can only proceed if there are plenty of stripe_heads.
	 * We need a minimum of one full stripe,, and for sensible progress
	 * it is best to have about 4 times that.
	 * If we require 4 times, then the default 256 4K stripe_heads will
	 * allow for chunk sizes up to 256K, which is probably OK.
	 * If the chunk size is greater, user-space should request more
	 * stripe_heads first.
	 */
	if ((mddev->chunk_size / STRIPE_SIZE) * 4 > conf->max_nr_stripes ||
	    (mddev->new_chunk / STRIPE_SIZE) * 4 > conf->max_nr_stripes) {
		printk(KERN_WARNING "raid5: reshape: not enough stripes.  Needed %lu\n",
		       (mddev->chunk_size / STRIPE_SIZE)*4);
		return -ENOSPC;
	}

	err = resize_stripes(conf, conf->raid_disks + mddev->delta_disks);
	if (err)
		return err;

	if (mddev->degraded > conf->max_degraded)
		return -EINVAL;
	/* looks like we might be able to manage this */
	return 0;
}

static int raid5_start_reshape(mddev_t *mddev)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);
	mdk_rdev_t *rdev;
	struct list_head *rtmp;
	int spares = 0;
	int added_devices = 0;
	unsigned long flags;

	if (test_bit(MD_RECOVERY_RUNNING, &mddev->recovery))
		return -EBUSY;

	rdev_for_each(rdev, rtmp, mddev)
		if (rdev->raid_disk < 0 &&
		    !test_bit(Faulty, &rdev->flags))
			spares++;

	if (spares - mddev->degraded < mddev->delta_disks - conf->max_degraded)
		/* Not enough devices even to make a degraded array
		 * of that size
		 */
		return -EINVAL;

	atomic_set(&conf->reshape_stripes, 0);
	spin_lock_irq(&conf->device_lock);
	conf->previous_raid_disks = conf->raid_disks;
	conf->raid_disks += mddev->delta_disks;
	conf->expand_progress = 0;
	conf->expand_lo = 0;
	spin_unlock_irq(&conf->device_lock);

	/* Add some new drives, as many as will fit.
	 * We know there are enough to make the newly sized array work.
	 */
	rdev_for_each(rdev, rtmp, mddev)
		if (rdev->raid_disk < 0 &&
		    !test_bit(Faulty, &rdev->flags)) {
			if (raid5_add_disk(mddev, rdev)) {
				char nm[20];
				set_bit(In_sync, &rdev->flags);
				added_devices++;
				rdev->recovery_offset = 0;
				sprintf(nm, "rd%d", rdev->raid_disk);
				if (sysfs_create_link(&mddev->kobj,
						      &rdev->kobj, nm))
					printk(KERN_WARNING
					       "raid5: failed to create "
					       " link %s for %s\n",
					       nm, mdname(mddev));
			} else
				break;
		}

	spin_lock_irqsave(&conf->device_lock, flags);
	mddev->degraded = (conf->raid_disks - conf->previous_raid_disks) - added_devices;
	spin_unlock_irqrestore(&conf->device_lock, flags);
	mddev->raid_disks = conf->raid_disks;
	mddev->reshape_position = 0;
	set_bit(MD_CHANGE_DEVS, &mddev->flags);

	clear_bit(MD_RECOVERY_SYNC, &mddev->recovery);
	clear_bit(MD_RECOVERY_CHECK, &mddev->recovery);
	set_bit(MD_RECOVERY_RESHAPE, &mddev->recovery);
	set_bit(MD_RECOVERY_RUNNING, &mddev->recovery);
	mddev->sync_thread = md_register_thread(md_do_sync, mddev,
						"%s_reshape");
	if (!mddev->sync_thread) {
		mddev->recovery = 0;
		spin_lock_irq(&conf->device_lock);
		mddev->raid_disks = conf->raid_disks = conf->previous_raid_disks;
		conf->expand_progress = MaxSector;
		spin_unlock_irq(&conf->device_lock);
		return -EAGAIN;
	}
	md_wakeup_thread(mddev->sync_thread);
	md_new_event(mddev);
	return 0;
}
#endif

static void end_reshape(raid5_conf_t *conf)
{
	struct block_device *bdev;

	if (!test_bit(MD_RECOVERY_INTR, &conf->mddev->recovery)) {
		conf->mddev->array_size = conf->mddev->size *
			(conf->raid_disks - conf->max_degraded);
		set_capacity(conf->mddev->gendisk, conf->mddev->array_size << 1);
		conf->mddev->changed = 1;

		bdev = bdget_disk(conf->mddev->gendisk, 0);
		if (bdev) {
			mutex_lock(&bdev->bd_inode->i_mutex);
			i_size_write(bdev->bd_inode, (loff_t)conf->mddev->array_size << 10);
			mutex_unlock(&bdev->bd_inode->i_mutex);
			bdput(bdev);
		}
		spin_lock_irq(&conf->device_lock);
		conf->expand_progress = MaxSector;
		spin_unlock_irq(&conf->device_lock);
		conf->mddev->reshape_position = MaxSector;

		/* read-ahead size must cover two whole stripes, which is
		 * 2 * (datadisks) * chunksize where 'n' is the number of raid devices
		 */
		{
			int data_disks = conf->previous_raid_disks - conf->max_degraded;
			int stripe = data_disks *
				(conf->mddev->chunk_size / PAGE_SIZE);
			if (conf->mddev->queue->backing_dev_info.ra_pages < 2 * stripe)
				conf->mddev->queue->backing_dev_info.ra_pages = 2 * stripe;
		}
	}
}

static void raid5_quiesce(mddev_t *mddev, int state)
{
	raid5_conf_t *conf = mddev_to_conf(mddev);

	switch(state) {
	case 2: /* resume for a suspend */
		wake_up(&conf->wait_for_overlap);
		break;

	case 1: /* stop all writes */
		spin_lock_irq(&conf->device_lock);
		conf->quiesce = 1;
		wait_event_lock_irq(conf->wait_for_queue,
				    atomic_read(&conf->active_queues) == 0 &&
				    atomic_read(&conf->active_aligned_reads) == 0,
				    conf->device_lock, /* nothing */);
		spin_unlock_irq(&conf->device_lock);
		break;

	case 0: /* re-enable writes */
		spin_lock_irq(&conf->device_lock);
		conf->quiesce = 0;
		wake_up(&conf->wait_for_queue);
		wake_up(&conf->wait_for_overlap);
		spin_unlock_irq(&conf->device_lock);
		break;
	}
}

static struct mdk_personality raid6_personality =
{
	.name		= "raid6",
	.level		= 6,
	.owner		= THIS_MODULE,
	.make_request	= make_request,
	.run		= run,
	.stop		= stop,
	.status		= status,
	.error_handler	= error,
	.hot_add_disk	= raid5_add_disk,
	.hot_remove_disk= raid5_remove_disk,
	.spare_active	= raid5_spare_active,
	.sync_request	= sync_request,
	.resize		= raid5_resize,
#ifdef CONFIG_MD_RAID5_RESHAPE
	.check_reshape	= raid5_check_reshape,
	.start_reshape  = raid5_start_reshape,
#endif
	.quiesce	= raid5_quiesce,
};
static struct mdk_personality raid5_personality =
{
	.name		= "raid5",
	.level		= 5,
	.owner		= THIS_MODULE,
	.make_request	= make_request,
	.run		= run,
	.stop		= stop,
	.status		= status,
	.error_handler	= error,
	.hot_add_disk	= raid5_add_disk,
	.hot_remove_disk= raid5_remove_disk,
	.spare_active	= raid5_spare_active,
	.sync_request	= sync_request,
	.resize		= raid5_resize,
#ifdef CONFIG_MD_RAID5_RESHAPE
	.check_reshape	= raid5_check_reshape,
	.start_reshape  = raid5_start_reshape,
#endif
	.quiesce	= raid5_quiesce,
};

static struct mdk_personality raid4_personality =
{
	.name		= "raid4",
	.level		= 4,
	.owner		= THIS_MODULE,
	.make_request	= make_request,
	.run		= run,
	.stop		= stop,
	.status		= status,
	.error_handler	= error,
	.hot_add_disk	= raid5_add_disk,
	.hot_remove_disk= raid5_remove_disk,
	.spare_active	= raid5_spare_active,
	.sync_request	= sync_request,
	.resize		= raid5_resize,
#ifdef CONFIG_MD_RAID5_RESHAPE
	.check_reshape	= raid5_check_reshape,
	.start_reshape  = raid5_start_reshape,
#endif
	.quiesce	= raid5_quiesce,
};

static int __init raid5_init(void)
{
	int e;

	e = raid6_select_algo();
	if ( e )
		return e;
	register_md_personality(&raid6_personality);
	register_md_personality(&raid5_personality);
	register_md_personality(&raid4_personality);
	return 0;
}

static void raid5_exit(void)
{
	unregister_md_personality(&raid6_personality);
	unregister_md_personality(&raid5_personality);
	unregister_md_personality(&raid4_personality);
}

module_init(raid5_init);
module_exit(raid5_exit);
MODULE_LICENSE("GPL");
MODULE_ALIAS("md-personality-4"); /* RAID5 */
MODULE_ALIAS("md-raid5");
MODULE_ALIAS("md-raid4");
MODULE_ALIAS("md-level-5");
MODULE_ALIAS("md-level-4");
MODULE_ALIAS("md-personality-8"); /* RAID6 */
MODULE_ALIAS("md-raid6");
MODULE_ALIAS("md-level-6");

/* This used to be two separate modules, they were: */
MODULE_ALIAS("raid5");
MODULE_ALIAS("raid6");
