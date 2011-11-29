/*
 * ppc64 "iomap" interface implementation.
 *
 * (C) Copyright 2004 Linus Torvalds
 */
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/pci-bridge.h>

#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
extern spinlock_t lbi_lock;
#endif

/*
 * Here comes the ppc64 implementation of the IOMAP 
 * interfaces.
 */
unsigned int ioread8(void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int ret, flags;
	spin_lock_irqsave(&lbi_lock, flags);
	ret = readb(addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
	return ret;
#else
	return readb(addr);
#endif
}
unsigned int ioread16(void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int ret, flags;
	spin_lock_irqsave(&lbi_lock, flags);
	ret = readw(addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
	return ret;
#else
	return readw(addr);
#endif
}
unsigned int ioread16be(void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int ret, flags;
	spin_lock_irqsave(&lbi_lock, flags);
	ret = in_be16(addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
	return ret;
#else
	return in_be16(addr);
#endif
}
unsigned int ioread32(void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int ret, flags;
	spin_lock_irqsave(&lbi_lock, flags);
	ret = readl(addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
	return ret;
#else
	return readl(addr);
#endif
}
unsigned int ioread32be(void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int ret, flags;
	spin_lock_irqsave(&lbi_lock, flags);
	ret = in_be32(addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
	return ret;
#else
	return in_be32(addr);
#endif
}
EXPORT_SYMBOL(ioread8);
EXPORT_SYMBOL(ioread16);
EXPORT_SYMBOL(ioread16be);
EXPORT_SYMBOL(ioread32);
EXPORT_SYMBOL(ioread32be);

void iowrite8(u8 val, void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	writeb(val, addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	writeb(val, addr);
#endif
}
void iowrite16(u16 val, void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int ret, flags;
	spin_lock_irqsave(&lbi_lock, flags);
	writew(val, addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	writew(val, addr);
#endif
}
void iowrite16be(u16 val, void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	out_be16(addr, val);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	out_be16(addr, val);
#endif
}
void iowrite32(u32 val, void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	writel(val, addr);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	writel(val, addr);
#endif
}
void iowrite32be(u32 val, void __iomem *addr)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	out_be32(addr, val);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	out_be32(addr, val);
#endif
}
EXPORT_SYMBOL(iowrite8);
EXPORT_SYMBOL(iowrite16);
EXPORT_SYMBOL(iowrite16be);
EXPORT_SYMBOL(iowrite32);
EXPORT_SYMBOL(iowrite32be);

/*
 * These are the "repeat read/write" functions. Note the
 * non-CPU byte order. We do things in "IO byteorder"
 * here.
 *
 * FIXME! We could make these do EEH handling if we really
 * wanted. Not clear if we do.
 */
void ioread8_rep(void __iomem *addr, void *dst, unsigned long count)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	_insb((u8 __iomem *) addr, dst, count);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	_insb((u8 __iomem *) addr, dst, count);
#endif
}
void ioread16_rep(void __iomem *addr, void *dst, unsigned long count)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	_insw_ns((u16 __iomem *) addr, dst, count);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	_insw_ns((u16 __iomem *) addr, dst, count);
#endif
}
void ioread32_rep(void __iomem *addr, void *dst, unsigned long count)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	_insl_ns((u32 __iomem *) addr, dst, count);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	_insl_ns((u32 __iomem *) addr, dst, count);
#endif
}
EXPORT_SYMBOL(ioread8_rep);
EXPORT_SYMBOL(ioread16_rep);
EXPORT_SYMBOL(ioread32_rep);

void iowrite8_rep(void __iomem *addr, const void *src, unsigned long count)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	_outsb((u8 __iomem *) addr, src, count);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	_outsb((u8 __iomem *) addr, src, count);
#endif
}
void iowrite16_rep(void __iomem *addr, const void *src, unsigned long count)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	_outsw_ns((u16 __iomem *) addr, src, count);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	_outsw_ns((u16 __iomem *) addr, src, count);
#endif
}
void iowrite32_rep(void __iomem *addr, const void *src, unsigned long count)
{
#ifdef CONFIG_PPC_PASEMI_A2_WORKAROUNDS
	int flags;
	spin_lock_irqsave(&lbi_lock, flags);
	_outsl_ns((u32 __iomem *) addr, src, count);
	spin_unlock_irqrestore(&lbi_lock, flags);
#else
	_outsl_ns((u32 __iomem *) addr, src, count);
#endif
}
EXPORT_SYMBOL(iowrite8_rep);
EXPORT_SYMBOL(iowrite16_rep);
EXPORT_SYMBOL(iowrite32_rep);

void __iomem *ioport_map(unsigned long port, unsigned int len)
{
	return (void __iomem *) (port + _IO_BASE);
}

void ioport_unmap(void __iomem *addr)
{
	/* Nothing to do */
}
EXPORT_SYMBOL(ioport_map);
EXPORT_SYMBOL(ioport_unmap);

void __iomem *pci_iomap(struct pci_dev *dev, int bar, unsigned long max)
{
	resource_size_t start = pci_resource_start(dev, bar);
	resource_size_t len = pci_resource_len(dev, bar);
	unsigned long flags = pci_resource_flags(dev, bar);

	if (!len)
		return NULL;
	if (max && len > max)
		len = max;
	if (flags & IORESOURCE_IO)
		return ioport_map(start, len);
	if (flags & IORESOURCE_MEM)
		return ioremap(start, len);
	/* What? */
	return NULL;
}

void pci_iounmap(struct pci_dev *dev, void __iomem *addr)
{
	if (isa_vaddr_is_ioport(addr))
		return;
	if (pcibios_vaddr_is_ioport(addr))
		return;
	iounmap(addr);
}

EXPORT_SYMBOL(pci_iomap);
EXPORT_SYMBOL(pci_iounmap);
