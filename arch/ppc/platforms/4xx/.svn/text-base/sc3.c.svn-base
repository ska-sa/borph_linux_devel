/*
 *
 *  (C) Copyright 2007
 *  Heiko Schocher, DENX Software Engineering, <hs@denx.de>.
 *
 *  From:
 *    Copyright(c) 2003 Juergen Beisert <jbeisert@netscape.net>
 *
 *    Module name: solidcard3.c
 *    Licence: GPL
 *
 *    Description:
 *      Architecture- / platform-specific boot-time initialization code for
 *      IBM PowerPC 4xx based boards. Adapted from original
 *      code by Gary Thomas, Cort Dougan <cort@fsmlabs.com>, and Dan Malek
 *      <dan@net4x.com>.
 *
 *      History: 06/2003 jb
 *      First release
 */

#include <linux/autoconf.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/blkdev.h>
#include <linux/pci.h>
#include <linux/rtc.h>
#include <linux/serial.h>
#include <linux/tty.h>	/* for linux/serial_core.h */
#include <linux/serial_core.h>

#include <asm/system.h>
#include <asm/pci-bridge.h>
#include <asm/processor.h>
#include <asm/machdep.h>
#include <asm/page.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/ocp.h>
#include <asm/ibm_ocp.h>
#include <asm/ibm_ocp_pci.h>
#include <asm/ppc4xx_pic.h>

#ifdef CONFIG_KGDB

#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>
#include <asm/ibm4xx.h>
#include <asm/serial.h>
#include <asm/ocp.h>
extern struct serial_state rs_table[]; /* from drivers/char/serial.c */
extern void early_uart_init(void);  /* from ocp_uart.c */

#endif

extern int ds1337_set_rtc_time(unsigned long nowtime);
extern ulong ds1337_get_rtc_time(void);

unsigned long isa_memory_access = 0UL;
unsigned long isa_io_access = 0UL;
static unsigned long SRAM_access = 0UL;

#if SC3_IRQ_SENSE_TABLE
/*
 * Define all of the IRQ senses and polarities.
 */

unsigned char ppc4xx_uic_ext_irq_cfg[] __initdata = {
	(IRQ_SENSE_EDGE  | IRQ_POLARITY_POSITIVE),	/* 19: Ext Int 7 */
	(IRQ_SENSE_EDGE  | IRQ_POLARITY_POSITIVE),	/* 20: Ext Int 8 */
	(IRQ_SENSE_EDGE  | IRQ_POLARITY_POSITIVE),	/* 21: Ext Int 9 */
	(IRQ_SENSE_EDGE  | IRQ_POLARITY_POSITIVE),	/* 22: Ext Int 10 */
	(IRQ_SENSE_EDGE  | IRQ_POLARITY_POSITIVE),	/* 23: Ext Int 11 */
	(IRQ_SENSE_EDGE  | IRQ_POLARITY_POSITIVE),	/* 24: Ext Int 12 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* 25: Ext Int 0 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* 26: Ext Int 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* 27: Ext Int 2 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* 28: Ext Int 3 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* 29: Ext Int 4 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* 30: Ext Int 5 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* 31: Ext Int 6 */
};
#endif

#if SC3_EARLY_UART_MAP
static void __init
sc3_early_serial_map(void)
{
	struct uart_port uart_req;
 
	/* Setup serial port access */
	memset(&uart_req, 0, sizeof (uart_req));
	uart_req.line = 2;
	uart_req.irq = 19;
	uart_req.flags = STD_COM_FLAGS;
	uart_req.uartclk = 1843200;
	uart_req.iotype = SERIAL_IO_MEM;
	uart_req.mapbase = 0x790003f8;
	uart_req.membase = ioremap(uart_req.mapbase, 0x10);
	uart_req.type = PORT_16650;

	if (early_serial_setup(&uart_req) != 0)
		printk("Early serial init of port 0 failed\n");
}
#endif

#ifdef CONFIG_PCI
/**
 *	ppc405_map_irq - Maps the PCI interrupts lines to logical interrupt channels
 *	@dev: pointer to the current pci device
 *	@idsel: ID select to access the current pci device
 *	@pin: Interrupt pin the PCI device wants to usge
 *
 *	Every hardware platform could do its own interrupt line routing.
 *	This function maps the correct logical interrupt to the used
 *	hardware interrupt line. The logical interrupt number is returned.
 */

int __init ppc405_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	/*-------------------------------------------------------------------------+
          | This is the routing on the SolidCard II and III evaluation board
          |             ,-.     ,-.        ,-.        ,-.        ,-.
          |   IRQ28 ----|B|-----|P|-.    ,-|P|-.    ,-| |-.    ,-|G|
          |             |R|     |C|  \  /  |C|  \  /  |E|  \  /  |r|
          |   IRQ29 ----|I|-----|1|-. `/---|1|-. `/---|t|-. `/---|a|
          |             |D|     |0|  \/    |0|  \/    |h|  \/    |p|
          |   IRQ30 ----|G|-----|4|-./`----|4|-./`----|e|-./`----|h|
          |             |E|     |+| /\     |+| /\     |r| /\     |i|
          |   IRQ31 ----| |-----| |-  `----| |-  `----| |-  `----|c|
          |             `-'     `-'        `-'        `-'        `-'
          |   Slot      0       10         11         12         13
          |   REQ#              0          1          2          *
          |   GNT#              0          1          2          *
          +-------------------------------------------------------------------------*/
        
	const int pci_irq_table[][4] = {
		/*   A   B   C   D  */
		{31, 30, 29, 28},    /* IDSEL 10 - PC104+ slot */
		{30, 29, 28, 31},    /* IDSEL 11 - PC104+ slot */
		{29, 28, 31, 30},    /* IDSEL 12 - Ethernet */
		{28, 31, 30, 29},    /* IDSEL 13 - VGA */
	};

	const long min_idsel = 10, max_idsel = 13, irqs_per_slot = 4;

	return PCI_IRQ_TABLE_LOOKUP;
};
#endif

#ifdef CONFIG_KGDB
/**
 *	very_early_uarts_init	- Init both uarts to be usable when kernel debugger starts
 *
 *	Init UARTs for: First uart only for console, second uart for debugger
 *	(declaration of baudrate in gen550_dbg.c as macro SERIAL_BAUD)
 *
 *	If the embedded kernel debugger should run, we have here to setup the
 *	serial port to connect this remote machine to the host. We have also
 *	to setup the console port here.
 *	If we do not embed the kernel debugger the console port is initalized elsewhere
 */

static void __init very_early_uarts_init(void)
{
	struct serial_struct serial_req={0,};
	/*
	 * at first initialize the console port (must be done here!)
	 */
	early_uart_init();
	/*
	 * And then our gdb port
	 */
	serial_req.baud_base = BASE_BAUD;
	serial_req.flags = (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST);
	serial_req.io_type = SERIAL_IO_MEM;

	serial_req.line = 1;
	serial_req.port = 1;
	serial_req.irq = UART1_INT;
	serial_req.iomem_base = (void*)UART1_IO_BASE;

	gen550_init(1,&serial_req);

	if (early_serial_setup(&serial_req) != 0)
		printk(KERN_ERR"Early serial init of uart1 failed.\n");
}
#endif

void __init sc3_setup_arch(void)
{
	struct ocp_def *def;
	struct ocp_func_emac_data *emacdata;

	/* Set mac_addr for each EMAC */
	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 0);
	emacdata = (struct ocp_func_emac_data *)def->additions;
	emacdata->phy_map = 0x00000001;	/* Skip 0x00 */
	emacdata->phy_mode = PHY_MODE_RMII;
	memcpy(emacdata->mac_addr, __res.bi_enetaddr, 6);

	ppc4xx_setup_arch();

	/* Identify the system */
	printk("SolidCard3 DENX <hs@denx.de> port.\n");
}

void __init bios_fixup(struct pci_controller *hose, struct pcil0_regs *pcip)
{
}

void __init sc3_map_io(void)
{
	ppc4xx_map_io();
	/*
	 * The internal SRAM uses only virtual addresses, never physical addresses
	 * Who the f*ck has designed such a thing..?
	 * So we should map this memory to the same address
	 */
	io_block_mapping(0xF8000000,0xF8000000,4*1024,_PAGE_IO);
	SRAM_access=0xF8000000UL;
	/*
	 * Map the emulated ISA bus to virtual space
	 * Physical: 0x78000000 -> generates memory access (16 Meg)
	 * Physical: 0x79000000 -> generates io access (64k)
	 */
	isa_memory_access=(unsigned long)ioremap_nocache(0x78000000, 16 * 1024 * 1024);
	isa_io_access=(unsigned long)ioremap_nocache(0x79000000, 64 * 1024);
}

void __init board_init(void)
{
#ifdef CONFIG_KGDB
	/* very_early_uarts_init() is called from
	 * kernel/gen550_kgdb.c/kgdb_map_scc(), called from kernel/setup.c/setup_arch()
	 * in the case CONFIG_KGDB is defined
	 */
	ppc_md.early_serial_map=very_early_uarts_init;
#endif
}

/* -------------- RTC ------------------------ */
ulong sc3_get_rtc_time(void)
{
	static	int	calls = 0;
	int	result = ds1337_get_rtc_time();

	if (result == -ENODEV) {
		/* DS1337 not initialized, prevent realtime
		   Clock is stuck warning */
		return calls++;
	}
	return result;
}

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	ppc4xx_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = sc3_setup_arch;
	ppc_md.setup_io_mappings = sc3_map_io;
	ppc_md.set_rtc_time	= ds1337_set_rtc_time;
	ppc_md.get_rtc_time	= sc3_get_rtc_time;
}
