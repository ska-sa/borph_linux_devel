/*********************************************************************
 * $Id: borph.c,v 1.10 2006/10/31 07:28:57 skhay Exp $
 * File  : borph.c
 * Author: Hayden Kwok-Hay So
 * Date  : 12/14/2005
 * Description:
 *   Main entry point for BORPH + Linux
 *********************************************************************/
#include <linux/borph.h>
#include <linux/slab.h>
#include <linux/init.h>    // needed for __init
#include <linux/module.h>
#define HDEBUG
#define HDBG_NAME "borph"
#define HDBG_LVL 9 //mkd_info->dbg_lvl
#include <linux/hdebug.h>

/*
 * borph_exit_fpga()
 * De-configura fpga associated with @tsk
 */
void borph_exit_fpga(struct task_struct *tsk)
{
	struct borph_hw_region *region, *tmp;
	struct borph_ioreg *reg, *tmpreg;
	struct task_list *tl, *tmptl;
	struct borph_info *bi;
	struct hwr_operations *hwrops;

	if (!(bi = tsk->borph_info)) {
	    goto exit;
	}
	/* HHH Race between /proc/<pid>/hw and here... */
	if (!list_empty(&bi->ioreg)) {
		list_for_each_entry_safe(reg, tmpreg, &bi->ioreg, list) {
			list_del(&reg->list);
			// slab, please...
			kfree(reg);
		}
	}
	if (!list_empty(&bi->hw_region)) {
		list_for_each_entry_safe(region, tmp, &bi->hw_region, list) {
			PDEBUG(0,"kill fpga process at region 0x%X\n",
			       region->addr.addr);

			kfree(region->strtab);
			// so that the rest of kernel knows this hwr is a zombie now
			hwr_deactivate(&region->addr);
			/* disable interrupt from this hwr */
			{
#ifdef CONFIG_MKD
			    uint32_t d, mask;
			    mask = 1UL << region->addr.addr;
			    in_intr_reg(SELMAP_INTR_IPIER, d);
			    out_intr_reg(SELMAP_INTR_IPIER, d & ~mask);

			    /* If mkd died, there's no point for us to do 
			     * anything...  This is bad... but
			     * hopefully only happens because we are
			     * shutting down. */

			    if (mkd_info) {
				mkd_info->selectmap_pending &= ~mask;
				mkd_info->fifo_pending &= ~mask;
				// HHH check address class
				mkd_info->mkd_bufs[region->addr.addr] = 
				    mkd_info->mkd_bufe[region->addr.addr];
			    }
#endif
			}
			hwrops = get_hwrops(&region->addr);
			if (hwrops && hwrops->unconfigure) {
				hwrops->unconfigure(&region->addr);
			} else {
				printk("unknown hwr type %d\n", region->addr.class);
			}
			put_hwrops(&region->addr);

			// HHH 
			release_hwr(&region->addr);
			list_del(&region->list);
			// slab, please...
			kfree(region);
		}
	}
	/* kill all kernel threads that are acting on behave of this process */
	/* Supposingly by now, all hwr are released, so no one will generate
	 * any new fringe.  Therefore, we don't need to lock.  Otherwise, we 
	 * would have an ugly deadlock between here and all killed fringe */
	PDEBUG(9,"check hcptr\n");
	if (!list_empty(&bi->hcptr)) {
		PDEBUG(9,"hcptr not empty, kill my fringes\n");
		list_for_each_entry_safe(tl, tmptl, &bi->hcptr, tsk_list) {
			PDEBUG(9,"kill fringe thread (pid=%d)\n", 
			       tl->tsk->pid);
			force_sig(SIGKILL, tl->tsk);
		}
	}

	// again.. please... slab.....
	kfree(tsk->borph_info);
	tsk->borph_info = NULL;
 exit:
	return;
}

/**************************************
 * mkd related functions
 **************************************/
struct mkd_struct *mkd_info = NULL;

/**************************************
 * Slab
 **************************************/
struct kmem_cache* task_list_cachep;

/**************************************
 * Initialization specific to BORPH 
 **************************************/
void __init borph_init(void)
{
	hwr_init();

	/* create slab for tsk_list (HHH When is it destroyed?)*/
	task_list_cachep = kmem_cache_create("task_list", 
					     sizeof(struct task_list),
					     0, 0, NULL);
	if (!task_list_cachep) {
	    return;
	}

	printk("BORPH version CVS-$Revision: 1.10 $ Initialized\n");
	return;
}
