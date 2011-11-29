/*********************************************************************
 * $Id: 
 * File  : hwrtype_roach.c
 * Author: Hayden Kwok-Hay So, Brandon Hamilton
 * Date  : 
 * Description:
 *   Define hwrtype for roach evaluation board
 *********************************************************************/
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/slab.h>    /* kmalloc/kfree            */
#include <linux/init.h>
#include <linux/ioport.h>  /* request_mem_region */

#include <linux/bof.h>
#include <linux/borph.h>
#define HDEBUG
#define HDBG_NAME "hwrtype_roach"
#ifdef CONFIG_MKD
# define HDBG_LVL mkd_info->dbg_lvl
#else
# define HDBG_LVL 9
#endif
#include <linux/hdebug.h>

#include <asm/uaccess.h>   /* copy_from_user */
#include <asm/io.h>        /* out_8 etc */

/* Some device specific definitions */
#define ROACH_CPLD_BASE         0x1C0000000
#define ROACH_SMAP_BASE         0x1C0100000
#define ROACH_EBC_BASE          0x1D0000000

#define ROACH_CPLD_LENGTH 0x000001000
#define ROACH_SMAP_LENGTH 0x000100000
#define ROACH_EBC_LENGTH  0x008000000

#define CPLD_SM_STATUS  0x8
#define CPLD_SM_OREGS   0x9
#define CPLD_SM_DATA    0xa
#define CPLD_SM_CTRL    0xb

#define CPLD_SM_DONE    0x2
#define CPLD_SM_INIT_N  0x1

#define SM_INITB_WAIT 100000
#define SM_DONE_WAIT  100000

/* Certain hardware revisions flip the byte ordering for selectmap */
/* define LITTLE_ENDIAN_SMAP to enable this flip */

#ifdef LITTLE_ENDIAN_SMAP

#define SMAP_READW \
  (in_le16((void *)(roach_ebc->smap_virt)))

#define SMAP_WRITEW(value) \
  (out_le16((void *)(roach_ebc->smap_virt), value))

#else

#define SMAP_READW \
  (in_be16((void *)(roach_ebc->smap_virt)))

#define SMAP_WRITEW(value) \
  (out_be16((void *)(roach_ebc->smap_virt), value))

#endif


#define FPGA_READW(offset) \
  (in_be16((void *)(roach_ebc->ebc_virt + offset)))

#define FPGA_WRITEW(offset, value) \
  (out_be16((void *)(roach_ebc->ebc_virt + offset), value))

#define FPGA_WRITEL(offset, value) \
  (out_be32((void *)(roach_ebc->ebc_virt + offset),value))

#define CPLD_CFG_OUT(offset, value) \
  (out_8((void *)(roach_ebc->cpld_virt + offset), (value)))

#define CPLD_CFG_IN(offset) \
  (in_le32(roach_ebc->cpld_virt + offset))


/*
 * A static buffer for data transfer.  It should be expanded to a 
 * kmem_cache when higher performance is needed.  (Right now, there
 * can only be one ioreg performing I/O at a time.
 */
static buf_t *roach_page;
static struct hwr_iobuf *iobuf;

/* Mutex Semaphore to ensure proper access to page buffer */

static DECLARE_MUTEX(roach_mutex);

typedef struct ebc_map {
  resource_size_t cpld_base;    /* Base address of memory region   */
  void *cpld_virt;              /* Virtual base address of region  */
  resource_size_t smap_base;   
  void *smap_virt;
  resource_size_t ebc_base;   
  void *ebc_virt;
} roach_ebc_map_t;

static roach_ebc_map_t *roach_ebc;

static void hwrtype_roach_finish(void);

/*****************************************************************
 * functions definitions
 *****************************************************************/
static ssize_t roach_send_iobuf(struct hwr_iobuf *iobuf, ssize_t ask)
{
	unsigned short *src;
  unsigned int dst;
	int i, words;

	PDEBUG(9, "Writing IOREG (size = %d bytes) to location 0x%x\n", ask, (unsigned int) (roach_ebc->ebc_virt + iobuf->location));

  /* looks overdone, but does rounds bytes */
  words = ask / sizeof(unsigned short);
	src = (unsigned short *)iobuf->data;	
	dst = iobuf->location + iobuf->offset;	

	for (i = 0; i < words; i++){
		FPGA_WRITEW(dst, *src);
    dst += sizeof(unsigned short);
		src++; 
	}

	//PDEBUG(9, "Writing to location 0x%x: 0x%x\n", roach_ebc->fpga_virt + iobuf->location, rrd);
	
	return (words * sizeof(unsigned short)); /* report the rounded value */
}

static ssize_t roach_recv_iobuf(struct hwr_iobuf *iobuf, ssize_t ask)
{
	unsigned short *dst;
  unsigned int src;
	int i, words;

	PDEBUG(9, "Reading IOREG (size = %d bytes) from location 0x%x\n", iobuf->size,  (unsigned int) (roach_ebc->ebc_virt + iobuf->location + iobuf->offset));

  words = ask / sizeof(unsigned short);
	dst = (unsigned short *)iobuf->data;	
  src = iobuf->location + iobuf->offset;

	for (i = 0; i < words; i++){
		*dst = FPGA_READW(src);
    src += sizeof(unsigned short);
		dst++; 
	}

	//PDEBUG(9, "Reading from location 0x%x: 0x%x\n", roach_ebc->fpga_virt + iobuf->location, wrd);
	
	return (words * sizeof(unsigned short));
}

static struct hwr_iobuf *roach_get_iobuf(void)
{
        PDEBUG(9, "Locking IOBUF\n");

        /* If there is different buffer for differ hwr, should
         * deferentiate them here using *reg* */

        if (down_interruptible(&roach_mutex)) {
                /* signal received, semaphore not acquired ... */
                return NULL;
        }

	iobuf->cmd = 0;
	iobuf->size = PAGE_SIZE - offsetof(struct hwr_iobuf, data);

        return iobuf; //HHH
}

static ssize_t roach_put_iobuf(struct hwr_iobuf *iobuf)
{
  PDEBUG(9, "Unlocking IOBUF\n");
	up(&roach_mutex);
	return 0; //HHH
}

static int roach_configure(struct hwr_addr *addr, struct file *file, uint32_t offset, uint32_t len)
{
	int i;
	int count;
	int retval = -EIO;
	volatile unsigned short *src;

	PDEBUG(9, "Configuring ROACH fpga %u from (offset %u, len %u) of %s\n", addr->addr, offset, len, file->f_dentry->d_name.name);

	if (addr->addr != 0) {
		PDEBUG(9, "Invalid FPGA #%d for ROACH\n", addr->addr); /* Roach only has 1 HWR */
		goto out; 
	}

	/* Disable the init_n output */
	CPLD_CFG_OUT(CPLD_SM_CTRL, 0x00);
	wmb();

	/* Set Write mode, and enable init_n and prog_b pins  */ 
	CPLD_CFG_OUT(CPLD_SM_OREGS, 0x03);
	wmb();

	for (i=0; i < 32; i++) { /* Delay for at least 350ns  */
		/* Set Write mode, and disable init_n and prog_b pins */
		CPLD_CFG_OUT(CPLD_SM_OREGS, 0x00);
	}
	wmb();	
	
	/* Set Write mode, and enable init_n and prog_b pins    */
	CPLD_CFG_OUT(CPLD_SM_OREGS, 0x03);
	wmb();

	/* Poll until init_n is enabled */
	for (i=0; i < SM_INITB_WAIT + 1; i++) {
		if (CPLD_CFG_IN(CPLD_SM_STATUS) & CPLD_SM_INIT_N) {
			break;
		}
		if (i == SM_INITB_WAIT) {
			PDEBUG(9, "SelectMap - Init_n pin has not been asserted\n");
			goto out_free_mutex;
		}
	}
   
	count = 0;

	/* Read bitstream from file and write it out over SelectMap */
#ifdef LITTLE_ENDIAN_SMAP
	PDEBUG(9, "Streaming data using little endian selectmap interface \n");
#else
	PDEBUG(9, "Streaming data using big endian selectmap interface \n");
#endif
	while (len > 0) {
		count = min(PAGE_SIZE, len);
		retval = kernel_read(file, offset, roach_page, count);
		if (retval < 0) {
			goto out_free_mutex;
		}
		if (retval != count) {
			printk("kernel_read returns less than requested...\n");
			count = retval;
		}

		len -= count;
		offset += count;

		i = 0; retval = -EIO;

		src = (unsigned short *)(roach_page);  		
		while(count > 0) {
			SMAP_WRITEW(*src);
			src++;
			count -= 2;
		}
	}

	/* Poll until done pin is enabled */
  for (i=0; i < SM_INITB_WAIT + 1; i++) {
    if (CPLD_CFG_IN(CPLD_SM_STATUS) & CPLD_SM_DONE) {
      break;
    }
    if (i == SM_DONE_WAIT) {
      PDEBUG(9, "SelectMap - Done pin has not been asserted\n");
      goto out_free_mutex;
    }
	}

	PDEBUG(9, "ROACH Virtex-5 configuration completed successfully\n");
	retval = 0;

out_free_mutex:
out:
	return retval;	
}

static int roach_unconfigure(struct hwr_addr* addr)
/* Pulse progb to clear FPGA config */
{
  int i;
	PDEBUG(9, "Unconfiguring ROACH fpga %u\n", addr->addr);

	/* Disable the init_n output */
	CPLD_CFG_OUT(CPLD_SM_CTRL, 0x00);
	wmb();

	/* Set Write mode, and enable init_n and prog_b pins  */ 
	CPLD_CFG_OUT(CPLD_SM_OREGS, 0x03);
	wmb();

	for (i=0; i < 32; i++) { /* Delay for at least 350ns  */
		/* Set Write mode, and disable init_n and prog_b pins */
		CPLD_CFG_OUT(CPLD_SM_OREGS, 0x00);
	}

	/* Set Write mode, and enable init_n and prog_b pins  */ 
	CPLD_CFG_OUT(CPLD_SM_OREGS, 0x03);
	wmb();

	return 0;	
}

struct phyhwr* roach_reserve_hwr(struct hwr_addr* a)
{
  struct phyhwr* ret;
  if (a && a->class != HAC_ROACH) {
    return NULL;
  }
	
	/* safety check */
	if (a->addr >= 1)
		return NULL;

	ret = phyhwrs[a->class][a->addr];
	if (!atomic_inc_and_test(&ret->count)) {
		atomic_dec(&ret->count);
		/* was being used */
		return NULL;
	}
	/* count is now a usage count */
	atomic_inc(&ret->count);
	return ret;
}

void roach_release_hwr(struct hwr_addr* a)
{
	struct phyhwr* hwr;
	if (a && a->class != HAC_ROACH)
		return;

	/* safety check */
	if (a->addr >= 1)
		return;
	
	hwr = phyhwrs[a->class][a->addr];
	if (atomic_dec_and_test(&hwr->count)) {
		hwr->task = NULL;
		atomic_set(&hwr->count, -1);
	}
}

static struct hwr_operations roach_hwr_operations = {
	.configure = roach_configure,
	.unconfigure = roach_unconfigure,
	.reserve_hwr = roach_reserve_hwr,
	.release_hwr = roach_release_hwr,
	.get_iobuf = roach_get_iobuf,
	.put_iobuf = roach_put_iobuf,
	.send_iobuf = roach_send_iobuf,
	.recv_iobuf = roach_recv_iobuf
};

static struct hwrtype hwrtype_roach = {
	name: "roach",
	type: HAC_ROACH,
	count: ATOMIC_INIT(0),
  num_devs: 1,
	hwr_ops: &roach_hwr_operations,
};

static int __init hwrtype_roach_init(void)
{
	int retval = 0;

  /* shouldn't one register things only when ready to go ? */
	if ((retval = register_hwrtype(&hwrtype_roach)) < 0) {
		printk("Error registering hwrtype\n");
    return retval;
	}

  printk("hwrtype_roach version CVS-$Revision: 1.1 $ registered\n");

  atomic_set(&(phyhwrs[HAC_ROACH][0])->count, -1);
	      	
  roach_ebc = NULL;
  roach_page = NULL;

	iobuf = (struct hwr_iobuf*) kmalloc(sizeof(struct hwr_iobuf), GFP_KERNEL);
	if (iobuf == NULL){
		printk("roach: unable to kmalloc iobuf\n");
    hwrtype_roach_finish();
		return -ENOMEM;
  }

	roach_page = (buf_t*)__get_free_page(GFP_KERNEL);
  if(roach_page == NULL){
		printk("roach: unable to get free roach page\n");
    hwrtype_roach_finish();
		return -ENOMEM;
  }

	iobuf->data = roach_page + offsetof(struct hwr_iobuf, data);
	iobuf->size = PAGE_SIZE - offsetof(struct hwr_iobuf, data);

  roach_ebc = kmalloc(sizeof(roach_ebc_map_t), GFP_KERNEL);
  if(roach_ebc == NULL){
		printk("roach: unable to allocate ebc structure\n");
    hwrtype_roach_finish();
		return -ENOMEM;
  }

	roach_ebc->cpld_base = 0;
	roach_ebc->cpld_virt = NULL;

	roach_ebc->smap_base = 0;
	roach_ebc->smap_virt = NULL;

	roach_ebc->ebc_base = 0;
	roach_ebc->ebc_virt = NULL;

  /* request and ioremap regions, macro to avoid repetition */
#define acquire_roach_region(p, q)  \
  if(!request_mem_region(ROACH_##q##_BASE, ROACH_##q##_LENGTH, "roach-" #p)){ \
    printk("roach: unable to request region 0x%llx\n", ROACH_##q##_BASE); \
    hwrtype_roach_finish(); \
    return -ENOMEM; \
  } \
  roach_ebc->p##_base = ROACH_##q##_BASE; \
  roach_ebc->p##_virt = ioremap64(ROACH_##q##_BASE,ROACH_##q##_LENGTH); \
  if(!roach_ebc->p##_virt){ \
    printk("roach: unable to map region 0x%llx\n", ROACH_##q##_BASE); \
    hwrtype_roach_finish(); \
    return -ENOMEM; \
  } \
  PDEBUG(9, "roach: " #p " iomem at 0x%llx:0x%llx mapped to 0x%p\n", ROACH_##q##_BASE, ROACH_##q##_BASE + ROACH_##q##_LENGTH, roach_ebc->p##_virt); 

  acquire_roach_region(cpld, CPLD)
  acquire_roach_region(smap, SMAP)
  acquire_roach_region(ebc, EBC)
#undef acquire_roach_region

	return 0;
}

static void hwrtype_roach_finish(void)
{
	if(roach_ebc){
#define release_roach_region(p, q) \
    if(roach_ebc->p##_virt){ \
      iounmap(roach_ebc->p##_virt); \
      roach_ebc->p##_virt = NULL; \
    } \
    if(roach_ebc->p##_base){ \
      release_mem_region(roach_ebc->p##_base, ROACH_##q##_LENGTH); \
      roach_ebc->p##_base = 0; \
    }
 
    release_roach_region(cpld, CPLD)
    release_roach_region(smap, SMAP)
    release_roach_region(ebc, EBC)
#undef release_roach_region

		kfree(roach_ebc);
		roach_ebc = NULL;
	}

	if(roach_page){
		free_page((unsigned long) roach_page);
    roach_page = NULL;
	}

	/* Free iobuf */
	if(iobuf){
    iobuf->data = NULL;
    iobuf->size = 0;
		kfree(iobuf);
    iobuf = NULL;
	}

	if(unregister_hwrtype(&hwrtype_roach)){
		printk("Error unregistering hwrtyp\n");
	} else {
		printk("hwrtype_roach CVS-$Revision: 1.1 $ unregistered\n");
	}
}

static void __exit hwrtype_roach_exit(void)
{
  hwrtype_roach_finish();
}

module_init(hwrtype_roach_init);
module_exit(hwrtype_roach_exit);

MODULE_AUTHOR("Hayden So");
MODULE_DESCRIPTION("Add hwrtype roach to program FPGA on sequoia as hw process");
MODULE_LICENSE("GPL");
