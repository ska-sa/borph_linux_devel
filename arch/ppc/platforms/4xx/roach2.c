/*
 * arch/ppc/platforms/4xx/roach.c
 *
 * Roach board specific routines
 *
 * Copyright 2006-2007 DENX Software Engineering, Stefan Roese <sr@denx.de>
 *
 * Based on bamboo.c from Wade Farnsworth <wfarnsworth@mvista.com>
 *	Copyright 2004 MontaVista Software Inc.
 *	Copyright 2006 AMCC
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/time.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <linux/irq.h>
#include <linux/root_dev.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/ocp.h>
#include <asm/bootinfo.h>
#include <asm/ppc4xx_pic.h>
#include <asm/ppcboot.h>

#include <syslib/gen550.h>
#include <syslib/ibm440gx_common.h>

#include "roach2.h"

#if defined(CONFIG_ROACH2)
#define BOARDNAME  "440EPx Roach2"
#endif

extern bd_t __res;

static struct ibm44x_clocks clocks __initdata;

/*
 * Sequoia external IRQ triggering/polarity settings
 */
unsigned char ppc4xx_uic_ext_irq_cfg[] __initdata = {
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index0 - IRQ4: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index1 - IRQ7: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index2 - IRQ8: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index3 - IRQ9: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index4 - IRQ0: Ethernet 0 */
	(IRQ_SENSE_EDGE  | IRQ_POLARITY_POSITIVE), /* Index5 - IRQ1: Monitor */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index6 - IRQ5: CPLD*/
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index7 - IRQ6: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index8 - IRQ2: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index9 - IRQ3: */
};

/*
 * NOR FLASH configuration (using mtd physmap driver)
 */

/* start will be added dynamically, end is always fixed */
static struct resource roach_nor_resource = {
		.flags = IORESOURCE_MEM,
};

#define RW_PART0_OF	0
#define RW_PART0_SZ	0x180000
#define RW_PART1_SZ	0x280000
/* Partition 2 will be autosized dynamically... */
#define RW_PART3_SZ	0x40000
#define RW_PART4_SZ	0x60000

static struct mtd_partition roach_nor_parts[] = {
	{
		.name = "kernel",
		.offset = 0,
		.size = RW_PART0_SZ
	},
	{
		.name = "root",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART1_SZ,
	},
	{
		.name = "user",
		.offset = MTDPART_OFS_APPEND,
	},
	{
		.name = "env",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART3_SZ,
	},
	{
		.name = "u-boot",
		.offset = MTDPART_OFS_APPEND,
		.size = RW_PART4_SZ,
	}
};

static struct physmap_flash_data roach_nor_data = {
	.width		= 2,
	.parts		= roach_nor_parts,
	.nr_parts	= ARRAY_SIZE(roach_nor_parts),
};

static struct platform_device roach_nor_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev = {
			.platform_data = &roach_nor_data,
		},
	.num_resources	= 1,
	.resource	= &roach_nor_resource,
};

static int roach_setup_flash(void)
{
	roach_nor_resource.start = __res.bi_flashstart;
	roach_nor_resource.end = __res.bi_flashstart +
		__res.bi_flashsize - 1;

	/*
	 * Adjust partition 2 to flash size
	 */
	roach_nor_parts[2].size = __res.bi_flashsize -
		RW_PART0_SZ - RW_PART1_SZ - RW_PART3_SZ - RW_PART4_SZ;

	platform_device_register(&roach_nor_device);

	return 0;
}
arch_initcall(roach_setup_flash);

/*
 * get size of system memory from Board Info .
 */
unsigned long __init roach_find_end_of_memory(void)
{
	/* board info structure defined in /include/asm-ppc/ppcboot.h */
	return  __res.bi_memsize;
}

static void __init roach_calibrate_decr(void)
{
	unsigned int freq;

	if (mfspr(SPRN_CCR1) & CCR1_TCS)
		freq = ROACH_TMRCLK;
	else
		freq = clocks.cpu;

	ibm44x_calibrate_decr(freq);

}

static int roach_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: AMCC\n");
	seq_printf(m, "machine\t\t: PPC" BOARDNAME "\n");
	seq_printf(m, "cpu Hz\t\t: %d * %d\n", tb_ticks_per_jiffy, HZ);

	return 0;
}

static void __init roach_set_emacdata(void)
{
	struct ocp_def *def;
	struct ocp_func_emac_data *emacdata;

	/* Set mac_addr, phy mode and unsupported phy features for each EMAC */

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 0);
	emacdata = def->additions;
	memcpy(emacdata->mac_addr, __res.bi_enetaddr, 6);
	emacdata->phy_mode = PHY_MODE_RGMII;

	/* define NO_PHY_HACK here to prevent the kernel seeing/using a phy */

#ifdef NO_PHY_HACK
	emacdata->phy_map = 0xffffffff;	/* 0xff is special, causes code to run without phy */
#endif

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 1);
	emacdata = def->additions;
	memcpy(emacdata->mac_addr, __res.bi_enet1addr, 6);
	emacdata->phy_mode = PHY_MODE_RGMII;
#ifdef NO_PHY_HACK
	emacdata->phy_map = 0xfffffffe;	/* prevent eth1 from grabbing the phy of eth0 */
#endif

        /*
         * Clear PLB4A0_ACR[WRP] (Write Pipeline Control)
         * This fix will make the MAL burst disabling patch for the Linux
         * EMAC driver obsolete.
         */
	mtdcr(DCRN_PLB4A0_ACR, mfdcr(DCRN_PLB4A0_ACR) & ~(0x80000000 >> 7));
}

extern void abort(void);
extern int roach_late_iic_init(void);
extern void roach_late_iic_xfer(int iic_addr, int rd_w_n, unsigned char* data, int size);

static void roach_restart(char *cmd)
{
  char iic_data[4];
  if (roach_late_iic_init()){
	  printk("IIC Init Hardware Reset Error\n");
    goto soft_reset;
  }

	printk("System Hardware Reset\n");

  iic_data[0] = 0x2;
  roach_late_iic_xfer(0xf, 0, iic_data, 1);

  iic_data[1] = 0x2;
  iic_data[0] = 0x82;
  iic_data[3] = 0x0;
  iic_data[2] = 0x0;
  roach_late_iic_xfer(0xf, 0, iic_data, 4);

  msleep(10);

	printk("Unknown Hardware Reset Error\n");
soft_reset:
	printk("System Soft Reset\n");
	abort();
}

void roach_system_halt(void)
{
	printk("System Halted\n");
	local_irq_disable();
	while (1) ;
}

static void roach_power_off(void)
{
  char iic_data[4];
  if (roach_late_iic_init()){
	  printk("IIC Init Hardware Powerdown Error\n");
    roach_system_halt();
  }

	printk("System Hardware Powerdown\n");

  iic_data[0] = 0x2;
  roach_late_iic_xfer(0xf, 0, iic_data, 1);

  iic_data[1] = 0x2;
  iic_data[0] = 0x82;
  iic_data[3] = 0xff;
  iic_data[2] = 0xff;
  roach_late_iic_xfer(0xf, 0, iic_data, 4);

  msleep(10);

	printk("Unknown Hardware Powerdown Error\n");

  roach_system_halt();
}

static void roach_halt(void)
{
	printk("Performing System Halt\n");
  roach_system_halt();
}

static void __init roach_early_serial_map(void)
{
	struct uart_port port;

	/* Setup ioremapped serial port access */
	memset(&port, 0, sizeof(port));
	port.membase = ioremap64(PPC440EPX_UART0_ADDR, 8);
	port.irq = UART0_INT;
	port.uartclk = clocks.uart0;
	port.regshift = 0;
	port.iotype = SERIAL_IO_MEM;
	port.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
	port.line = 0;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 0 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/* Configure debug serial access */
	gen550_init(0, &port);
#endif

	port.membase = ioremap64(PPC440EPX_UART1_ADDR, 8);
	port.irq = UART1_INT;
	port.uartclk = clocks.uart1;
	port.line = 1;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 1 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/* Configure debug serial access */
	gen550_init(1, &port);
#endif

	port.membase = ioremap64(PPC440EPX_UART2_ADDR, 8);
	port.irq = UART2_INT;
	port.uartclk = clocks.uart2;
	port.line = 2;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 2 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/* Configure debug serial access */
	gen550_init(2, &port);
#endif

	port.membase = ioremap64(PPC440EPX_UART3_ADDR, 8);
	port.irq = UART3_INT;
	port.uartclk = clocks.uart3;
	port.line = 3;

	if (early_serial_setup(&port) != 0)
		printk("Early serial init of port 3 failed\n");
}

static void __init roach_setup_arch(void)
{
	roach_set_emacdata();

	/* parm1 = sys clock is OK , parm 2 ser_clock to be checked */
	ibm440gx_get_clocks(&clocks, 33333333, 6 * 1843200);
	ocp_sys_info.opb_bus_freq = clocks.opb;
	ocp_sys_info.plb_bus_freq = clocks.plb;

	/* init to some ~sane value until calibrate_delay() runs */
        loops_per_jiffy = 50000000/HZ;

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else {
#ifdef CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_HDA1;
#endif
	}
#endif

	roach_early_serial_map();

	/* Identify the system */
	printk("AMCC PowerPC " BOARDNAME " Platform\n");
}

static void __init roach_init_irq(void)
{
	ppc4xx_pic_init();
}

void __init platform_init(unsigned long r3, unsigned long r4,
			  unsigned long r5, unsigned long r6, unsigned long r7)
{
	ibm44x_platform_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = roach_setup_arch;
	ppc_md.show_cpuinfo = roach_show_cpuinfo;
	ppc_md.find_end_of_memory = roach_find_end_of_memory;
	ppc_md.get_irq = NULL;		/* Set in ppc4xx_pic_init() */

	ppc_md.calibrate_decr = roach_calibrate_decr;
	ppc_md.time_init = NULL;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;

  ppc_md.restart    = roach_restart;
  ppc_md.power_off  = roach_power_off;
  ppc_md.halt   = roach_halt;

	ppc_md.init_IRQ = roach_init_irq;

#ifdef CONFIG_KGDB
	ppc_md.early_serial_map = roach_early_serial_map;
#endif
}
