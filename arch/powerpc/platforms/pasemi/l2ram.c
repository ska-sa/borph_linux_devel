/*
 * Copyright (C) 2006-2007 PA Semi, Inc
 *
 * Maintained by: Olof Johansson <olof@lixom.net>
 *
 * Driver for the PA Semi L2 as RAM mode
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/sysdev.h>
#include <asm/uaccess.h>
#include <asm/mmu.h>

#define L2CAP_RAM_BASE_ADDR_HI			0x054
#define   L2CAP_RAM_BASE_ADDR_HI_BASEHI_M	0x00000fff
#define L2CAP_RAM_BASE_ADDR_LO			0x050
#define L2CFG_GEN				0x100
#define   L2CCFG_GEN_EN_L2			0x00000001
#define   L2CCFG_GEN_WAY_128K			0x00000004
#define   L2CCFG_GEN_FLUSH_CACHE		0x10000000
#define   L2CCFG_GEN_RAM_ENA			0x00100000
#define   L2CCFG_GEN_RAM_WAYS_M			0x00600000
#define   L2CCFG_GEN_RAM_WAYS_S			21
#define L2CFG_WAY_DISABLE			0x104
#define   L2CFG_WAY_DISABLE_FUSE_WAY_DIS_M	0xffff0000
#define   L2CFG_WAY_DISABLE_FUSE_WAY_DIS_S	16
#define   L2CFG_WAY_DISABLE_WAY_DIS_M		0x000000ff
#define L2CFG_WAY_MASK0				0x108
#define   L2CFG_WAY_MASK0_CPU0_WAY_MASK_M	0x000000ff
#define   L2CFG_WAY_MASK0_CPU1_WAY_MASK_M	0x00ff0000
#define   L2CFG_WAY_MASK0_CPU1_WAY_MASK_S	16
#define L2CFG_WAY_MASK4				0x118
#define   L2CFG_WAY_MASK4_IOB_WAY_MASK_M	0x000000ff


#define MODULE_NAME				"l2ram"
#define L2RAM_DEVICE_NAME			"l2ram"

struct l2ram {
	char			name[DEVICE_NAME_SIZE];
	phys_addr_t		phys_addr;
	void __iomem		*addr;
	size_t			size;
	spinlock_t		l2_lock;
	struct kobject		kobj;
	struct completion	kobj_complete;
	struct miscdevice	misc;
	struct file_operations	fops;
	int			open_count;
	u32			l2ram_way_mask;
};

/* sysfs object: /sys/devices/system/l2ram */
static struct sysdev_class l2ram_class = {
	set_kset_name("l2ram"),
};

/* We can have 0, 1, 2, 4, or 8 L2 ways allocated to be used as RAM */
static int available_l2ram_sizes[] = {
	0,
	0,
	0,
	0,
	0,
	-1
};

/* /sys/devices/system/l2ram data structures and methods */
static ssize_t l2ram_size_show(struct l2ram *bank,
			       char *buffer)
{
	return sprintf(buffer, "%lu\n", bank->size / 1024);
}

static ssize_t l2ram_available_sizes_show(struct l2ram *bank,
					  char *buffer)
{
	int i, count = 0;
	for (i = 0; available_l2ram_sizes[i] != -1; i++)
		count += sprintf(&buffer[count], "%d ",
				 available_l2ram_sizes[i]);

	count += sprintf(&buffer[count], "\n");
	return count;
}

static ssize_t l2ram_size_store(struct l2ram *bank,
				const char *buffer, size_t count)
{
	struct pci_dev *pdev;
	int req_size, ways_cnt, i, retval = count, ways_sel = -2;
	u32 ways_dis, gen, tmp, way_mask = 0;

	spin_lock(&bank->l2_lock);

	if (bank->open_count) {
		retval = -EBUSY;
		goto out;
	}

	req_size = simple_strtoul(buffer, NULL, 0);

	for (i = 0; available_l2ram_sizes[i] != -1; i++) {
		if (req_size == available_l2ram_sizes[i]) {
			ways_sel = i - 1;
			break;
		}
	}
	if (ways_sel == -2) {
		retval = -EINVAL;
		goto out;
	}

	bank->size = available_l2ram_sizes[i] * 1024;
	if (bank->addr) iounmap(bank->addr);

	bank->addr = ioremap_flags(bank->phys_addr, bank->size, 0);

	pdev = container_of(bank->misc.parent, struct pci_dev, dev);

	 /* Disable Cache and Flush it */

	pci_read_config_dword(pdev, L2CFG_GEN, &gen);
	gen &= ~(L2CCFG_GEN_EN_L2 | L2CCFG_GEN_RAM_ENA);
	pci_write_config_dword(pdev, L2CFG_GEN, gen);
	pci_write_config_dword(pdev, L2CFG_GEN, gen | L2CCFG_GEN_FLUSH_CACHE);

	do {
		pci_read_config_dword(pdev,L2CFG_GEN, &gen);
	} while (gen & L2CCFG_GEN_FLUSH_CACHE);

	if (ways_sel == -1) {
		/* Disable L2 as RAM */

		gen &= ~(L2CCFG_GEN_RAM_ENA | L2CCFG_GEN_RAM_WAYS_M);
		gen |= L2CCFG_GEN_EN_L2;
	} else if (ways_sel == 3) {
		/* Allocate all 8 Ways to L2 as RAM
		 * in this case we HAVE to disable L2 cache
		 */

		gen = (gen & ~L2CCFG_GEN_EN_L2) | L2CCFG_GEN_RAM_WAYS_M
			| L2CCFG_GEN_RAM_ENA;
	} else {
		gen = (gen & ~L2CCFG_GEN_RAM_WAYS_M)
			| (ways_sel << L2CCFG_GEN_RAM_WAYS_S)
			| L2CCFG_GEN_EN_L2 | L2CCFG_GEN_RAM_ENA;

		/* Some ways might have been disabled by a fuse or sw, so
		 * we need to go and mask out good ways for use as RAM
		 */

		pci_read_config_dword(pdev, L2CFG_WAY_DISABLE, &tmp);

		ways_dis = ((tmp & L2CFG_WAY_DISABLE_FUSE_WAY_DIS_M) >>
			    L2CFG_WAY_DISABLE_FUSE_WAY_DIS_S) |
			(tmp & L2CFG_WAY_DISABLE_WAY_DIS_M);
		ways_cnt = 1 << ways_sel;

		for (i = 7; (i >= 0) && ways_cnt; i--)
			if (!(ways_dis & (1 << i))) {
				way_mask |= (1 << i);
				ways_cnt--;
			}

		if (ways_cnt) {
			retval = -ENOMEM;
			goto out;
		}
	}

	bank->l2ram_way_mask = (way_mask & L2CFG_WAY_MASK0_CPU0_WAY_MASK_M)
		| (way_mask << L2CFG_WAY_MASK0_CPU1_WAY_MASK_S);
	pci_write_config_dword(pdev, L2CFG_WAY_MASK0, bank->l2ram_way_mask);

	pci_read_config_dword(pdev, L2CFG_WAY_MASK4, &tmp);
	pci_write_config_dword(pdev, L2CFG_WAY_MASK4,
			       (tmp & ~L2CFG_WAY_MASK4_IOB_WAY_MASK_M)
			       | way_mask);

	pci_write_config_dword(pdev, L2CFG_GEN, gen);

	/* Zero out the area to avoid exposing private data */
	memset(bank->addr, 0, bank->size);

 out:
	spin_unlock(&bank->l2_lock);
	return retval;
}

static ssize_t cpu_way_mask_show(struct l2ram *bank, char *buffer)
{
	struct pci_dev *pdev;
	u32 cpu_mask;
	pdev = container_of(bank->misc.parent, struct pci_dev, dev);

	pci_read_config_dword(pdev, L2CFG_WAY_MASK0, &cpu_mask);
	return sprintf(buffer, "0x%08x\n", cpu_mask);
}

static ssize_t cpu_way_mask_set(struct l2ram *bank,
				const char *buffer, size_t count)
{
	struct pci_dev *pdev;
	u32 req_mask;

	req_mask = simple_strtoul(buffer, NULL, 0);

	/* L2 as RAM gets priority, only allow clearing bits not in the
	 * l2ram_way_mask
	 */

	if ((req_mask & bank->l2ram_way_mask) != bank->l2ram_way_mask)
		return -EINVAL;

	pdev = container_of(bank->misc.parent, struct pci_dev, dev);

	pci_write_config_dword(pdev, L2CFG_WAY_MASK0, req_mask);
	return count;
}

struct l2ram_dev_attribute {
	struct attribute attr;
	ssize_t (*show)(struct l2ram *, char *);
	ssize_t (*store)(struct l2ram *, const char *, size_t);
};

/* Set of show/store abstract level functions for l2ram object */
static ssize_t l2ram_dev_show(struct kobject *kobj,
				  struct attribute *attr, char *buffer)
{
	struct l2ram_dev_attribute *l2ram_dev =
		(struct l2ram_dev_attribute*)attr;
	struct l2ram *bank = container_of(kobj, struct l2ram, kobj);

	if (l2ram_dev->show)
		return l2ram_dev->show(bank,buffer);

	return -EIO;
}

static ssize_t l2ram_dev_store(struct kobject *kobj, struct attribute *attr,
		const char *buffer, size_t count)
{
	struct l2ram_dev_attribute *l2ram_dev =
		(struct l2ram_dev_attribute*)attr;
	struct l2ram *bank = container_of(kobj, struct l2ram, kobj);

	if (l2ram_dev->store)
		return l2ram_dev->store(bank, buffer, count);

	return -EIO;
}

static struct sysfs_ops l2ramfs_ops = {
	.show   = l2ram_dev_show,
	.store  = l2ram_dev_store
};

#define L2RAM_ATTR(_name,_mode,_show,_store)			\
struct l2ram_dev_attribute _name = {				\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show   = _show,					\
	.store  = _store,					\
};

L2RAM_ATTR(l2ram_available_sizes, S_IRUGO,l2ram_available_sizes_show, NULL);
L2RAM_ATTR(l2ram_size, S_IRUGO|S_IWUSR, l2ram_size_show, l2ram_size_store);
L2RAM_ATTR(cpu_way_mask, S_IRUGO|S_IWUSR, cpu_way_mask_show, cpu_way_mask_set);

static struct l2ram_dev_attribute *l2ram_attr[] = {
	&l2ram_available_sizes,
	&l2ram_size,
	&cpu_way_mask,
	NULL,
};

/* Main L2 as RAM kobject release() function */
static void l2ram_kobj_release(struct kobject *kobj)
{
	struct l2ram *bank = container_of(kobj, struct l2ram, kobj);
	complete(&bank->kobj_complete);
}

static struct kobj_type ktype_l2ram = {
	.release = l2ram_kobj_release,
	.sysfs_ops = &l2ramfs_ops,
	.default_attrs = (struct attribute **) l2ram_attr,
};

/* l2ram_open - open() method for device
 * @inode, @file: see file_operations method
 */
static int l2ram_open(struct inode *inode, struct file *file)
{
	struct l2ram *bank;

	if (S_ISCHR(inode->i_mode)) {
		file->private_data = container_of(file->f_op,
						  struct l2ram, fops);
		bank = (struct l2ram*) file->private_data;
		spin_lock(&bank->l2_lock);
		bank->open_count++;
		spin_unlock(&bank->l2_lock);
	} else
		return -ENODEV;

	return 0;
}

/* l2ram_release - release() method for device
 * @inode, @file: see file_operations method
 */
static int l2ram_release(struct inode *inode, struct file *file)
{
	struct l2ram *bank;

	if (S_ISCHR(inode->i_mode)) {
		bank = (struct l2ram*)
			file->private_data;
		spin_lock(&bank->l2_lock);
		bank->open_count--;
		spin_unlock(&bank->l2_lock);
	} else
		return -ENODEV;

	return 0;
}

/* l2ram_llseek - llseek() method for character device
 * @file, @offset, @origin: see file_operations method
 */
static loff_t l2ram_llseek(struct file *file, loff_t offset, int origin)
{
	struct l2ram *bank;

	bank = (struct l2ram*) file->private_data;

	switch (origin) {
	case SEEK_CUR:
		offset += file->f_pos;
		break;

	case SEEK_END:
		offset += bank->size;
		break;
	}

	/* No access outside of L2 as RAM area */
	if (offset < 0 || offset > bank->size)
		return -EINVAL;

	return file->f_pos = offset;
}

/* l2ram_read - read() method for character device
 * @file, @buffer, @size, @offset: see file_operations method
 */
static ssize_t l2ram_read(struct file *file,
			   char __user *buffer, size_t size, loff_t *offset)
{
	struct l2ram *bank;

	bank = (struct l2ram*) file->private_data;

	/* No access outside of L2 as RAM area,
	 * but still do as much as possible.
	 */
	if (*offset + size > bank->size)
		size = bank->size - *offset;
	copy_to_user(buffer, bank->addr + *offset, size);
	*offset += size;

	return size;
}

/* l2ram_write - write() method for character device
 * @file, @buffer, @size, @offset: see file_operations method
 */
static ssize_t l2ram_write(struct file *file, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	struct l2ram *bank;

	bank = (struct l2ram*) file->private_data;

	/* No access outside of L2 as RAM area,
	 * but still do as much as possible.
	 */
	if (*offset + size > bank->size)
		size = bank->size - *offset;
	copy_from_user(bank->addr + *offset, buffer, size);
	*offset += size;

	return size;
}

/* l2ram_mmap - mmap() method for character device
 * @file, @vm: see file_operations method
 */
static int l2ram_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct l2ram *bank;
	unsigned long page;
	unsigned long size;
	pgprot_t page_prot;

	bank = (struct l2ram*) file->private_data;

	size = vma->vm_end - vma->vm_start;

	/* No access outside of L2 as RAM area */
	if ((vma->vm_pgoff << PAGE_SHIFT) + size > bank->size)
		return -ERANGE;

	/* Transfer real address to page number. */
	page = bank->phys_addr;
	page >>= PAGE_SHIFT;
	page += vma->vm_pgoff;

	/* Switch off page-guard. */
	page_prot = pgprot_val(vma->vm_page_prot);
	page_prot &= ~(_PAGE_NO_CACHE | _PAGE_GUARDED);
	vma->vm_page_prot = __pgprot(page_prot);
	if (remap_pfn_range(vma, vma->vm_start, page, size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int __devinit l2ram_probe(struct pci_dev *pdev,
				  const struct pci_device_id *ent)
{
	struct l2ram *bank;
	u32 tmp, l2ccfg;
	int i, err = 0;

	bank = kzalloc(sizeof(struct l2ram), GFP_KERNEL);
	if (bank == NULL) {
		dev_err(&pdev->dev, "Out of memory!\n");
		return -ENOMEM;
	}
	pci_set_drvdata(pdev, (void*) bank);

	snprintf(bank->name, DEVICE_NAME_SIZE, "%s",
		 L2RAM_DEVICE_NAME);

	pci_read_config_dword(pdev, L2CAP_RAM_BASE_ADDR_HI, &tmp);
	bank->phys_addr =
		((unsigned long)tmp & L2CAP_RAM_BASE_ADDR_HI_BASEHI_M) << 32;

	pci_read_config_dword(pdev, L2CAP_RAM_BASE_ADDR_LO, &tmp);
	bank->phys_addr |= tmp;

	bank->addr = NULL;
	bank->size = 0;
	bank->l2ram_way_mask = 0;
	bank->open_count = 0;

	bank->misc.minor = MISC_DYNAMIC_MINOR;
	bank->misc.name = bank->name;
	bank->misc.fops = &bank->fops;
	bank->misc.parent = &pdev->dev;

	bank->fops.owner = THIS_MODULE;
	bank->fops.open = l2ram_open;
	bank->fops.release = l2ram_release;
	bank->fops.llseek = l2ram_llseek;
	bank->fops.read = l2ram_read;
	bank->fops.write = l2ram_write;
	bank->fops.mmap = l2ram_mmap;

	if (misc_register(&bank->misc) != 0) {
		dev_err(&pdev->dev, "Cannot register character device!\n");
		return -ENODEV;
	}

	/* create the /sys/devices/system/l2ram directory */
	err = sysdev_class_register(&l2ram_class);

	if (err) {
		dev_dbg(&pdev->dev, " error=%d\n", err);
		return err;
	}

	/* Init the L2 as RAM kobject */
	memset(&bank->kobj, 0, sizeof(bank->kobj));
	bank->kobj.parent = &l2ram_class.kset.kobj;
	bank->kobj.ktype = &ktype_l2ram;

	/* generate sysfs "..../l2ram"   */
	err = kobject_set_name(&bank->kobj, "l2ram");

	if (err)
		goto failsys;

	err = kobject_register(&bank->kobj);

	if (err) {
		dev_dbg(&pdev->dev, "Failed to register '.../l2ram/l2ram'\n");
		goto failsys;
	}

	pci_read_config_dword(pdev, L2CFG_GEN, &l2ccfg);

	/* Initialize available sizes in KB */
	for (i = 0; i < 4; i++)
		available_l2ram_sizes[i+1] = ((l2ccfg & L2CCFG_GEN_WAY_128K) ?
					      128 : 256) * (1 << i);

	return 0;

 failsys:
	sysdev_class_unregister(&l2ram_class);
	return err;
}

static void __devexit l2ram_remove(struct pci_dev *pdev)
{
	struct l2ram *bank;
	int rc = 0;

	bank = (struct l2ram*) pci_get_drvdata(pdev);

	init_completion(&bank->kobj_complete);
	kobject_unregister(&bank->kobj);
	wait_for_completion(&bank->kobj_complete);

	/* Unregister the 'l2ram' object */
	sysdev_class_unregister(&l2ram_class);

	if (bank->misc.minor != MISC_DYNAMIC_MINOR)
		rc = misc_deregister(&bank->misc);

	if (rc == 0)
		kfree(bank);
}

static const struct pci_device_id l2ram_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_PASEMI, 0xa009) },
	{ 0, 0 },
};

MODULE_DEVICE_TABLE(pci, l2ram_pci_tbl);

static struct pci_driver l2ram_driver = {
	.name = MODULE_NAME,
	.probe = l2ram_probe,
	.remove = __devexit_p(l2ram_remove),
	.id_table = l2ram_pci_tbl,
};

static int __init l2ram_init(void)
{
	return pci_register_driver(&l2ram_driver);
}

static void __exit l2ram_exit(void)
{
	pci_unregister_driver(&l2ram_driver);
}

module_init(l2ram_init);
module_exit(l2ram_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Egor Martovetsky <egor@pasemi.com>");
MODULE_DESCRIPTION("L2 as RAM device driver for PA Semi processor");
