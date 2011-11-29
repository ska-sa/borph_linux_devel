/**************************************************
 * $Id: borph.h,v 1.12 2006/10/31 07:28:57 skhay Exp $
 * File  : borph.h
 * Author: Hayden So
 * Date  : 12/13/2005
 * Description:
 *    Top level header file for all BORPH related files
 **************************************************/
#ifndef _BORPH_H_
#define _BORPH_H_

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/bof.h>
#include <linux/kselectmap.h>
#include <linux/file.h>

/* a file to be exec-ed to FPGA */
struct execq_item {
  struct list_head list;
  struct linux_binprm *bprm;
  struct task_struct *task;
};

/* all info for communicating to/from bkexecd
 * It has much resemblance to workqueue in 2.6 kernel */
struct bkexecd_info {
  wait_queue_head_t more_exec;  /* bkexecd to be awaken */
  wait_queue_head_t exec_done;  /* exec-er to be notified done exec */

  spinlock_t execq_lock;
  struct list_head execq_list;  /* list of execq_item */
};

/* function exported to the rest of kernel */
extern void borph_exit_fpga(struct task_struct* tsk);

/* each reconfigurable region in BORPH is represented by a 
 * struct borph_hw_region */
struct borph_hw_region {
	struct hwr_addr addr;
	char* strtab;
	struct list_head list;
};

/*
 * each reconfigurable region can have zero or more ioreg
 */
struct borph_ioreg {
	char* name;
	uint32_t mode;
	uint32_t loc;
	uint32_t len;
	struct hwr_addr* hwraddr;
	struct list_head list;
	struct borph_info *bi;
};

/* A dummy struct so that we can create a task_list list without
 * messing with the actual structure of task_list */
struct task_list {
	struct task_struct* tsk;
	rwlock_t data_lock;
	void* data;  // argument for passing to fringe
	struct list_head tsk_list;
};

extern struct kmem_cache* task_list_cachep;

#define BORPH_STATUS_DEVICE_READY (1 << 0)
/* borph_info contains all hardware specific information in a
 * task_struct.  */
struct borph_info {
	struct list_head hw_region;
	struct list_head ioreg;

	/* This lists all the kernel thread to be created by mkd
	   on behalf of the hw portion for read/write, etc.  We need
	   this list for exit, SIGCONT, SIGTSTP, etc...
	   We need another structure to point to task_struct because I 
	   don't want to mess with adding yet another list to
	   task_struct */
	rwlock_t hcptr_lock;
	struct list_head hcptr;
	struct task_list* fringes[NR_OPEN_DEFAULT];

	int ioreg_mode;  // 0 for ascii, 1 for raw
  unsigned int status; 
};

/************************************************
 * ppacket
 ************************************************/
/* Read
 * +-----+------+------+------+------+------+------+------+----+----+----+----+
 * | CMD | LOC2 | LOC1 | LOC0 | OFF3 | OFF2 | OFF1 | OFF0 | S3 | S2 | S1 | S0 |
 * +-----+------+------+------+------+------+------+------+----+----+----+----+
 * ReadAck
 * +-----+------+------+------+----+----+----+----+---------+
 * | CMD | LOC2 | LOC1 | LOC0 | S3 | S2 | S1 | S0 | payload |
 * +-----+------+------+------+----+----+----+----+---------+
 * Write
 * +-----+------+------+------+------+------+------+------+----+----+----+----
 * | CMD | LOC2 | LOC1 | LOC0 | OFF3 | OFF2 | OFF1 | OFF0 | S3 | S2 | S1 | S0 
 * +-----+------+------+------+------+------+------+------+----+----+----+----
 * -+-----------
 *  | payload 
 * -+-----------
 *
 */
/*
#define PP_CMD_READ    1
#define PP_CMD_READACK 2
#define PP_CMD_WRITE   3
#define PP_CMD_WRITEACK 4
#define PP_CMD_EXIT     5
// #define NR_PP_CMD      4
#define PP_CMD_GREET 16
#define PP_CMD_BYE 17*/
enum ppacket_t { PP_CMD_READ = 1, PP_CMD_READACK, PP_CMD_WRITE, PP_CMD_WRITEACK, PP_CMD_EXIT, PP_CMD_GREET = 16, PP_CMD_BYE };


/*******************************************
 * functions to control the hwr device
 *******************************************/

struct hwr_iobuf {
  enum ppacket_t cmd;
  unsigned int location;
	unsigned int offset;
	size_t size;
	buf_t* data;
};

typedef struct hwr_iobuf* (*hwr_getiobuf_t) (void);

struct hwr_operations {
  int (*configure) (struct hwr_addr*, struct file*, uint32_t, uint32_t);
  int (*unconfigure) (struct hwr_addr*);
  struct phyhwr* (*reserve_hwr)(struct hwr_addr* a);
  void (*release_hwr) (struct hwr_addr* a);
  hwr_getiobuf_t get_iobuf;
  ssize_t (*put_iobuf) (struct hwr_iobuf* iobuf);
  ssize_t (*send_iobuf) (struct hwr_iobuf* iobuf, ssize_t ask);
	ssize_t (*recv_iobuf) (struct hwr_iobuf* iobuf, ssize_t ask);
};

struct hwrtype {
	char* name;
	uint16_t type;  // identical to hwr_addr.class
	atomic_t count; // use count
  uint16_t num_devs;  // number of physical devices
	struct hwr_operations *hwr_ops;
};

/* each physical hwr is represented by a hwr in kernel */
struct phyhwr {
	struct hwr_addr hwraddr;
	atomic_t count;            // use count
	atomic_t active;   // set to 1 if it is actively running
	struct task_struct *task;   // task that uses this hwr
};

#define MAX_HWRTYPES 16

extern struct hwrtype* hwrtypes[MAX_HWRTYPES];
extern struct phyhwr** phyhwrs[MAX_HWRTYPES];

extern int register_hwrtype(struct hwrtype*);
extern int unregister_hwrtype(struct hwrtype*);
extern struct hwr_operations* get_hwrops(struct hwr_addr* a);
extern void put_hwrops(struct hwr_addr* a);
extern struct phyhwr* reserve_hwr(struct hwr_addr*);
extern void release_hwr(struct hwr_addr* a);
extern struct phyhwr* get_hwr(struct hwr_addr* a);
extern void put_hwr(struct hwr_addr* a);
extern void __put_hwr(struct phyhwr* hwr);
extern struct hwr_iobuf* get_iobuf(struct hwr_addr* a);
extern ssize_t put_iobuf(struct hwr_addr* a, struct hwr_iobuf* iobuf);
extern ssize_t send_iobuf(struct hwr_addr* a, struct hwr_iobuf* iobuf, ssize_t ask);
extern ssize_t recv_iobuf(struct hwr_addr* a, struct hwr_iobuf* iobuf, ssize_t ask);
extern int hwr_inuse(struct hwr_addr* a);
extern void hwr_activate(struct hwr_addr* a);
extern void hwr_deactivate(struct hwr_addr* a);
extern void hwr_init(void);

/**************************************************
 * mkd stuff
 **************************************************/
struct mkd_struct {
	unsigned selectmap_pending;
	unsigned fifo_pending;
	void * intrreg;
	int dbg_lvl;

	/* circular buffer for packets (HHH pretty hacky)*/
	unsigned char* mkd_buf[SELECTMAP_NUM_DEVS];
	unsigned char* mkd_bufs[SELECTMAP_NUM_DEVS];
	unsigned char* mkd_bufe[SELECTMAP_NUM_DEVS];
	struct semaphore bufrsem[SELECTMAP_NUM_DEVS];
	wait_queue_head_t mkdrdq;  // 2 rdq to accelerate processing
	wait_queue_head_t bufrdq[SELECTMAP_NUM_DEVS];  // mkd always on mkdrdq, others on bufrdq
};
extern struct mkd_struct *mkd_info;

#define MKDBUF_SIZE PAGE_SIZE

extern inline unsigned char*  mkdbuf_add(unsigned char* buf, int incr) 
{ 
	/* Each buffer is PAGE_SIZE = 4k, terefore mask = 0xFFF */ \
        const unsigned long mask = 0xFFF; \
	unsigned long addr = (unsigned long) buf; \
	unsigned long h = addr & ~mask; \
	addr = h | ((addr + incr) & mask); \
	return (unsigned char*) addr;\
}


#define IncrBufS(id,val) \
  do {uint32_t d;\
      mkd_info->mkd_bufs[id] = mkdbuf_add(mkd_info->mkd_bufs[id],(val));\
      in_intr_reg(SELMAP_INTR_IPIER, d);\
      out_intr_reg(SELMAP_INTR_IPIER, d | (0x1UL << id)); } while(0)

#define IncrBufE(id,val) \
  do {mkd_info->mkd_bufe[id] = mkdbuf_add(mkd_info->mkd_bufe[id],(val));} while(0)
extern unsigned char* mkdbuf_add(unsigned char* buf, int incr);
#define mkdbuf_empty(id) (mkd_info->mkd_bufe[id] == mkd_info->mkd_bufs[id])
extern inline unsigned char* mkdbuf_add(unsigned char* buf, int incr);
#define mkdbufe(id) (mkd_info->mkd_bufe[id])
#define mkdbufs(id) (mkd_info->mkd_bufs[id])

#define __mkdbuf_len(_bs, _be) \
    ((_be < _bs)?(_be + (MKDBUF_SIZE) - _bs):(_be - _bs))
#define mkdbuf_len(id)          \
  ({buf_t* mybufe = mkdbufe(id);\
    buf_t* mybufs = mkdbufs(id);\
    __mkdbuf_len(mybufs, mybufe);})
#define mkdbuf_lenp(bufs, bufe) \
  ({buf_t* mybufe = (bufe);     \
    buf_t* mybufs = (bufs);     \
    __mkdbuf_len(mybufs, mybufe);})
#define mkdbuf_free(id) \
  ((MKDBUF_SIZE) - mkdbuf_len(id) - 1)
#define mkdbuf_freep(bufs, bufe) \
  ((MKDBUF_SIZE) - mkdbuf_lenp(bufs, bufe) - 1)

/*
 * get a 4 byte word from @offset from bufs[id], with 
 * possible wrap around
 */
static inline uint32_t mkdbuf_getword(int id, int offset)
{
	unsigned char* bufs = mkd_info->mkd_bufs[id];
	unsigned char* p;
	uint32_t ret = 0;
	p = mkdbuf_add(bufs, offset);
	ret |= (*p << 24);
	p = mkdbuf_add(p, 1);
	ret |= (*p << 16);
	p = mkdbuf_add(p, 1);
	ret |= (*p << 8);
	p = mkdbuf_add(p, 1);
	ret |= (*p);
	return ret;
}

/************************************************
 * misc
 ************************************************/
extern void borph_init(void);
// HHH remove once bkexecd is merged into borph.c
extern void borph_exit_fpga(struct task_struct* tsk);
#endif /* _BORPH_H_ */
