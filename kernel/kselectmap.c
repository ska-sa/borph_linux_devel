
#include <linux/ioport.h>  /* request/release region   */
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>

#define HDEBUG
#define HDBG_NAME "kselectmap"//"hwrtype_b2fpga"
#define HDBG_LVL  9
#include <linux/hdebug.h>
#include <linux/kselectmap.h>
/* no xparameters for roach at the moment */

// change back to static once code from mkd is merged back here
//static selectmap_t* selectmap_devs;
selectmap_t* selectmap_devs;

selectmap_t* get_selmapdev(int addr)
{
	selectmap_t* dev;
	if (addr < 0 || addr >= SELECTMAP_NUM_DEVS) return NULL;
	dev = &selectmap_devs[addr];
	down_interruptible(&dev->sem);
	return dev;
}
EXPORT_SYMBOL(get_selmapdev);

void put_selmapdev(selectmap_t* dev)
{
	up(&dev->sem);
}
EXPORT_SYMBOL(put_selmapdev);

static inline void free_selectmap_mem(void)
{
	int i;
	for (i=0; i < SELECTMAP_NUM_DEVS; i++) {
		if (selectmap_devs[i].base) {
			iounmap(selectmap_devs[i].vbase);
			release_mem_region(selectmap_devs[i].base, 
					   SELECTMAP_LENGTH);
		}
		if (selectmap_devs[i].buf) {
			kfree(selectmap_devs[i].buf);
		}
	}
	kfree(selectmap_devs);
}

int __init kselectmap_init(void) {
	int retval, i;
	u32 base = 0;

	/* Allocate the device structures */
	retval = -ENOMEM;
	if (!(selectmap_devs = kmalloc(SELECTMAP_NUM_DEVS * sizeof(selectmap_t), GFP_KERNEL))) {
		goto out;
	}

	/* Reset the device structures */
	memset(selectmap_devs, 0, SELECTMAP_NUM_DEVS * sizeof(selectmap_t));
	for (i=0; i < SELECTMAP_NUM_DEVS; i++) {
		base = XPAR_OPB_SELECTMAP_0_BASEADDR + (i * SELECTMAP_LENGTH);
		selectmap_devs[i].base = 0;
		selectmap_devs[i].vbase = 0;
		selectmap_devs[i].buf = NULL;
		selectmap_devs[i].id = i;
		init_MUTEX(&selectmap_devs[i].sem);

		/* allocate the read/write buffer */
		selectmap_devs[i].buf = kmalloc(SELECTMAP_BUFSIZE, GFP_KERNEL);
		if (selectmap_devs[i].buf == NULL) {
			printk(KERN_ERR "%d: unable to allocate read buffer\n",
			       i);
			goto out_freemem;
		}

		/* request the I/O memory region */
		retval = -EBUSY;
		if (!request_mem_region(base, SELECTMAP_LENGTH, "kselectmap")){
			printk(KERN_ERR "%s%d: memory range 0x%08x to 0x%08x is in use\n",
			       "kselectmap", i, 
			       (u32)base,
			       (u32)(base + SELECTMAP_LENGTH));
			goto out_freemem;
		}

		selectmap_devs[i].base = XPAR_OPB_SELECTMAP_0_BASEADDR + (i * SELECTMAP_LENGTH);
		selectmap_devs[i].vbase = ioremap(selectmap_devs[i].base, SELECTMAP_LENGTH);

		PDEBUG(5, "(%d) new I/O memory range 0x%08x to 0x%08x allocated (virt:0x%p)\n",
		       i, (u32)selectmap_devs[i].base,
		       (u32)(selectmap_devs[i].base + SELECTMAP_LENGTH),
		       selectmap_devs[i].vbase);
	}
	printk("kselectmap: Initialized\n");
	return 0;
 out_freemem:
	free_selectmap_mem();
 out:
	return retval;
}

void __exit kselectmap_exit(void)
{
	free_selectmap_mem();
}
