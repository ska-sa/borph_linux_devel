/*
 *
 *    Copyright (c) 2003 EuroDesign embedded technologies GmbH <info@eurodsn.de>
 *
 *    Description:
 *      Macros, definitions, and data structures specific to the IBM PowerPC
 *      based boards.
 *
 *    405GPr "SolidCard 3" evaluation board
 *
 * Diese Datei wird in include/asm/ibm4xx.h eingebunden 
 */
#ifdef __KERNEL__
#ifndef __ASM_SOLIDCARD3_H__
#define __ASM_SOLIDCARD3_H__

/* 
 * We have a 405GPr core on the SolidCard III 
 */

#include <platforms/4xx/ibm405gpr.h>

#ifndef __ASSEMBLY__

/*
 * Description of the running hardware for kernel 2.4.23-rc2-devel
 *
 * This structure is filled by the U-Boot loader and given to the Linux kernel.
 * Consider the declaration must be the same in both worlds.....
 *
 * New since 2.4.21 ported kernel:
 * entry bi_opb_busfreq: needed as "bi_opbfreq" by the new IBM IIC driver
 * entry bi_iic_fast: needed by the new IBM IIC driver
 */

typedef struct board_info {
        unsigned long	bi_memstart; /* start of DRAM memory */
        unsigned long	bi_memsize; /* size of DRAM memory in bytes */
        unsigned long	bi_flashstart; /* start of FLASH memory */
        unsigned long	bi_flashsize; /* size of FLASH memory */
        unsigned long	bi_flashoffset; /* reserved area for startup monitor */
        unsigned long	bi_sramstart; /* start of SRAM memory */
        unsigned long	bi_sramsize; /* size of SRAM memory */
        unsigned long	bi_bootflags; /* boot / reboot flag (for LynxOS) */
        unsigned long	bi_ip_addr; /* IP Address */
        unsigned char	bi_enetaddr[6]; /* Ethernet adress */
        unsigned short	bi_ethspeed; /* Ethernet speed in Mbps */
        unsigned long	bi_intfreq; /* Internal Freq, in MHz (processor clock, same as bi_procfreq)) */
        unsigned long	bi_busfreq; /* Bus Freq, in MHz (same like bi_plb_busfreq) */
        unsigned long	bi_baudrate; /* Console Baudrate */
        unsigned char	bi_s_version[4]; /* Version of this structure */
        unsigned char	bi_r_version[32]; /* Version of the ROM (IBM) */
        unsigned int	bi_procfreq; /* CPU (Internal) Freq, in Hz */
        unsigned int	bi_plb_busfreq; /* PLB Bus speed, in Hz */
        unsigned int	bi_pci_busfreq; /* PCI Bus speed, in Hz */
        unsigned char	bi_pci_enetaddr[6]; /* PCI Ethernet MAC address */
/* up to here we are synchron to u-boot */
        unsigned int	bi_opb_busfreq; /* OPB Bus speed, in Hz */
        int		bi_iic_fast[2]; /* Use fast i2c mode */
} bd_t;

/*
 * The new IBM IIC drivers uses "bi_opbfreq", but "bi_opb_busfreq" is cleaner...
 */
#define bi_opbfreq bi_opb_busfreq

/*
 * Some 4xx parts use a different timebase frequency from the internal clock.
 */
#define bi_tbfreq bi_intfreq

#ifdef CONFIG_IBM405GP_INTERNAL_CLOCK
# define BASE_BAUD    921600
#else
# define BASE_BAUD   (1843200 / 2) /*691200*/
#endif
#define BBAUD_8250 (1843200 / 16)
#define PPC4xx_MACHINE_NAME	"EuroDesign SolidCard III"


#define STD_COM_FLAGS (ASYNC_SKIP_TEST)

/*
 *
 * Memory map for the SolidCard III 405GPr evaluation board.
 *
 * 0x00000000 .... 0x07FFFFFF -> RAM (up to 128MiB)
 * 0x77C00000 .... 0x77CFFFFF -> USB HC (1 MiB)
 * 0x77D00000 .... 0x77DFFFFF -> NAND-Flash (1 MiB)
 * 0x78000000 .... 0x78FFFFFF -> ISA-Bus Speicherzugriff (16 MiB)
 * 0x79000000 .... 0x7900FFFF -> ISA-Bus IO-Zugriff (16 MiB, mapped: 64kiB)
 * 0x79010000 .... 0x79FFFFFF -> ISA-Bus IO-Zugriff (mirrored)
 * 0x7A000000 .... 0x7A0FFFFF -> IDE emulation (1MiB)
 *
 * 0x80000000 .... 0x9FFFFFFF -> PCI-Bus Speicherzugriff (512MiB, mapped: 1:1)
 * 0xA0000000 .... 0xBFFFFFFF -> PCI-Bus Speicherzugriff (512MiB, mapped: 0x00000000...0x1FFFFFFF)
 * 0xE8000000 .... 0xE800FFFF -> PCI-Bus IO-Zugriff (64kiB, translated to PCI: 0x0000...0xFFFF)
 * 0xE8800000 .... 0xEBFFFFFF -> PCI-Bus IO-Zugriff (56MiB, translated to PCI: 0x00800000...0x3FFFFFF)
 * 0xEED00000 .... 0xEED00003 -> PCI-Bus
 * 0xEF400000 .... 0xEF40003F -> PCI-Bus Local Configuration Registers
 * 0xEF40003F .... 0xEF5FFFFF -> reserved
 * 0xEF600000 .... 0xEFFFFFFF -> 405GP internal Devices (10 MiB)
 * 0xF0000000 .... 0xF01FFFFF -> Flash-ROM (2 MiB)
 * 0xF0200000 .... 0xF7FFFFFF -> free for flash devices
 * 0xF8000000 .... 0xF8000FFF -> OnChipMemory (4kiB)
 * 0xF8001000 .... 0xFFDFFFFF -> free for flash devices
 * 0xFFE00000 .... 0xFFFFFFFF -> BOOT-ROM (2 MiB)
 */


/*
 * Linux wan't realy on the u-boot setup
 */
#define SC3_IRQ_SENSE_TABLE 1
/*
 * Make an early serial setup (1) or not (0)
 */
#define SC3_EARLY_UART_MAP 0

/* IO_BASE is for PCI I/O.
 * ISA _is_ supported, just here to resolve compilation.
 */
#define _IO_BASE	0xE8000000        // PCI IO-Space

#define _ISA_MEM_BASE   0x00000000
#define _ISA_IO_BASE    0x00000000

#define PCI_DRAM_OFFSET	0

#define SC3_FLASH_BASE	0xffe00000
#define SC3_FLASH_SIZE	0x200000


#endif /* !__ASSEMBLY__ */
#endif /* __ASM_SOLIDCARD3_H__ */
#endif /* __KERNEL__ */
