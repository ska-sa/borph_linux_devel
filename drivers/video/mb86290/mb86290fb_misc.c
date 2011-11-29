/*
 *	mb86290fb_misc.c  --  MB86290 Series FrameBuffer Driver
 *
 *	Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/mm.h>   	/* add this line for mips environment */
#include <asm/pgtable.h>

#if 0
#include <linux/wrapper.h>	/* mem_map_reserve */
#endif
#define mem_map_reserve(p)      set_bit(PG_reserved, &((p)->flags))
#define mem_map_unreserve(p)    clear_bit(PG_reserved, &((p)->flags))

#include "mb86290fbdev.h"

int mb86290fb_memory_mapping(struct vm_area_struct *vma)
{
	unsigned long	vram_phys;
	unsigned long	offset;
	int		ret;

	offset = vma->vm_pgoff << PAGE_SHIFT;
	if (offset >= MB86290FB_MAPSIZE_64MB) {

		PDEBUG("[mb86290fb]dmamem_memma case \n");
		return (mb86290fb_dmamem_memmap(vma));
	} else {
		vram_phys = mb86290fb_pInfo->base_phys + offset;
		PDEBUG("[mb86290fb]io_remap_page_range case 0x%lx, off 0x%lx\n",
		       vram_phys, offset);

		vma->vm_flags |= VM_RESERVED;
		vma->vm_flags |= VM_IO;

		/* cache off */
		#if defined(__i386__) || defined(__x86_64__)
		pgprot_val(vma->vm_page_prot) |= _PAGE_PCD;
		#elif defined(__mips__)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		#elif defined(__powerpc__)
		pgprot_val(vma->vm_page_prot) |= _PAGE_NO_CACHE|_PAGE_GUARDED;
		#endif
		ret = io_remap_pfn_range(vma, vma->vm_start, vram_phys >> PAGE_SHIFT,
					 vma->vm_end - vma->vm_start, vma->vm_page_prot);
		if (ret) {
			printk(KERN_ALERT"[mb86290fb]io_remap_page_range has failed\n");
			return -ENOMEM;
		}
		return 0;
	}
}

int mb86290fb_ioctl_read_register(mb86290fb_drawman_t* drawman,
						unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	address, *pdata;

	parg = (GDC_ULONG*)arg;
	get_user(address, (GDC_ULONG*)  parg);
	get_user(pdata, (GDC_ULONG**)(parg+1));

	put_user(MB86290FB_READ_REG(address), pdata);
	PDEBUG("ReadReg 0x%08lx = 0x%08lx.\n", address, MB86290FB_READ_REG(address));

	return 0;
}

int mb86290fb_ioctl_write_register(mb86290fb_drawman_t* drawman,
							unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	address, data;

	parg = (GDC_ULONG*)arg;
	get_user(address, parg);
	get_user(data, parg+1);

	MB86290FB_WRITE_REG(address, data);
	PDEBUG("WriteReg 0x%08lx = 0x%08lx.\n", address, data);

	return 0;
}

int mb86290fb_ioctl_getsysinfo(unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	*pdata;

	parg = (GDC_ULONG*)arg;

	get_user(pdata, (GDC_ULONG**)parg);

	if (!access_ok(VERIFY_WRITE, (void*)pdata, sizeof(GDC_SYSTEMINFO))){
		return -EFAULT;
	}
	copy_to_user(pdata, &mb86290fb_pInfo->gdcsysinfo_user,
		     sizeof(GDC_SYSTEMINFO));
	return 0;
}

int mb86290fb_ioctl_getsem(mb86290fb_info_t *pinfo, unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	var;

	parg = (GDC_ULONG*)arg;
	get_user(var, parg);

	if (var>=MB86290FB_SEM_NUM)
		return -EINVAL;

	if (down_interruptible(&pinfo->sem[var]))
		return -ERESTARTSYS;
	return 0;
}

int mb86290fb_ioctl_ungetsem(mb86290fb_info_t *pinfo, unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	var;

	parg = (GDC_ULONG*)arg;
	get_user(var, parg);

	if (var>=MB86290FB_SEM_NUM)
		return -EINVAL;

	up(&pinfo->sem[var]);
	return 0;
}

int mb86290fb_ioctl_sioreadwrite(mb86290fb_info_t *pinfo, unsigned long arg)
{
	int		i;
	GDC_ULONG	*parg;
	GDC_ULONG	bit_len, data_len, strobe_mode;
	GDC_UCHAR	*pwdata, *prdata;
	GDC_UCHAR	wdata, rdata;
	GDC_ULONG	tls = 0x00010000;

	parg = (GDC_ULONG*)arg;
	get_user(bit_len, parg);
	get_user(data_len, parg+1);
	get_user(strobe_mode, parg+2);
	get_user(pwdata, (GDC_UCHAR**)(parg+3));
	get_user(prdata, (GDC_UCHAR**)(parg+4));

	for (i=0; i<data_len; i++) {
		/* Write */
		get_user(wdata, (pwdata+i));
		pinfo->serial.data_arrive = 0;
		MB86290FB_WRITE_HOST_REGISTER(GDC_HOST_REG_SERIALDATA,
					      (tls>>bit_len) | wdata | strobe_mode);
		/* Wait */
		if (pinfo->serial.data_arrive) {
			return -EBUSY;
		}
		wait_event_interruptible(pinfo->serial.waitq,
					 pinfo->serial.data_arrive==1);
		/* Read */
		if (prdata) {
			rdata = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_SERIALDATA) & 0xFF;
			put_user(rdata, (GDC_UCHAR*)(prdata+i));
		}
	}

	return 0;
}

int mb86290fb_ioctl_waitgpio(mb86290fb_info_t *pinfo, unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	mask, pattern, mode, *pdata;

	parg = (GDC_ULONG*)arg;
	get_user(mask, parg);
	get_user(pattern, parg+1);
	get_user(mode, parg+2);
	get_user(pdata, (GDC_ULONG**)(parg+3));

	pinfo->gpio.data_read = MB86290FB_READ_HOST_REGISTER(GDC_HOST_REG_GD);
	if (mode==GDC_WAIT_AND) {
		wait_event_interruptible(pinfo->gpio.waitq,
					 (pinfo->gpio.data_read&mask)==(pattern&mask) );
	} else {
		wait_event_interruptible(pinfo->gpio.waitq,
					 ((~(pinfo->gpio.data_read^pattern))&mask) );
	}

	put_user(pinfo->gpio.data_read, pdata);
	return 0;
}

void mb86290fb_initcapinfo(mb86290fb_capinfo_t *capinfo)
{
	capinfo->using = 0;
}

int mb86290fb_ioctl_getcapture(mb86290fb_capinfo_t *capinfo)
{
	if (capinfo->using) {
		if (current->pid==capinfo->pid_using)
			return 0;
		return -EAGAIN;
	}

	capinfo->using     = 1;
	capinfo->pid_using = current->pid;
	return 0;
}

int mb86290fb_ioctl_releasecapture(mb86290fb_capinfo_t *capinfo)
{
	if (capinfo->using) {
		if (current->pid == capinfo->pid_using) {
			capinfo->using = 0;
			return 0;
		}
		return -EACCES;
	}
	return -EINPROGRESS;
}

int mb86290fb_ioctl_setbit(mb86290fb_info_t *pinfo, unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	offset;
	GDC_ULONG	pattern;
	GDC_ULONG	reg;

	parg = (GDC_ULONG*)arg;
	get_user(offset, parg);
	get_user(pattern, parg+1);

	reg = MB86290FB_READ_REGISTER(offset);
	MB86290FB_WRITE_REGISTER(offset, reg|pattern);

	return 0;
}

int mb86290fb_ioctl_clearbit(mb86290fb_info_t *pinfo, unsigned long arg)
{
	GDC_ULONG	*parg;
	GDC_ULONG	offset;
	GDC_ULONG	pattern;
	GDC_ULONG	reg;

	parg = (GDC_ULONG*)arg;
	get_user(offset, parg);
	get_user(pattern, parg+1);

	reg = MB86290FB_READ_REGISTER(offset);
	MB86290FB_WRITE_REGISTER(offset, reg&=pattern);

	return 0;
}

int mb86290fb_ioctl_read_disp_register(unsigned long arg)
{
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	put_user(MB86290FB_READ_DISP_REGISTER(address), (GDC_ULONG*)arg + 1);

	return 0;
}

int mb86290fb_ioctl_write_disp_register(unsigned long arg)
{
	GDC_ULONG data;
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	get_user(data, (GDC_ULONG*)arg + 1);

	MB86290FB_WRITE_DISP_REGISTER(address, data);

	return 0;
}

int mb86290fb_ioctl_read_cap_register(unsigned long arg)
{
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	put_user(MB86290FB_READ_CAP_REGISTER(address), (GDC_ULONG*)arg + 1);

	return 0;
}

int mb86290fb_ioctl_write_cap_register(unsigned long arg)
{
	GDC_ULONG data;
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	get_user(data, (GDC_ULONG*)arg + 1);

	MB86290FB_WRITE_CAP_REGISTER(address, data);

	return 0;
}

int mb86290fb_ioctl_read_geo_register(unsigned long arg)
{
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	put_user(MB86290FB_READ_GEO_REGISTER(address), (GDC_ULONG*)arg + 1);

	return 0;
}

int mb86290fb_ioctl_write_geo_register(unsigned long arg)
{
	GDC_ULONG data;
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	get_user(data, (GDC_ULONG*)arg + 1);

	MB86290FB_WRITE_GEO_REGISTER(address, data);

	return 0;
}

int mb86290fb_ioctl_read_draw_register(unsigned long arg)
{
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	put_user(MB86290FB_READ_DRAW_REGISTER(address), (GDC_ULONG*)arg + 1);

	return 0;
}

int mb86290fb_ioctl_write_draw_register(unsigned long arg)
{
	GDC_ULONG data;
	GDC_ULONG address;

	get_user(address, (GDC_ULONG*)arg);
	get_user(data, (GDC_ULONG*)arg + 1);

	MB86290FB_WRITE_DRAW_REGISTER(address, data);

	return 0;
}
