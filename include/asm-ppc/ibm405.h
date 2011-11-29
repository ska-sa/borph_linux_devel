/*
 * Author: Armin Kuster <akuster@mvista.com>
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifdef __KERNEL__
#ifndef __ASM_IBM405_H__
#define __ASM_IBM405_H__

#ifdef DCRN_BE_BASE
#define DCRN_BEAR	(DCRN_BE_BASE + 0x0)	/* Bus Error Address Register */
#define DCRN_BESR	(DCRN_BE_BASE + 0x1)	/* Bus Error Syndrome Register */
#endif
/* DCRN_BESR */
#define BESR_DSES	0x80000000	/* Data-Side Error Status */
#define BESR_DMES	0x40000000	/* DMA Error Status */
#define BESR_RWS	0x20000000	/* Read/Write Status */
#define BESR_ETMASK	0x1C000000	/* Error Type */
#define ET_PROT	0
#define ET_PARITY	1
#define ET_NCFG	2
#define ET_BUSERR	4
#define ET_BUSTO	6

/* Clock and power management shifts for emacs */
#define IBM_CPM_EMMII	0	/* Shift value for MII */
#define IBM_CPM_EMRX	1	/* Shift value for recv */
#define IBM_CPM_EMTX	2	/* Shift value for MAC */

#ifdef DCRN_CHCR_BASE
#define DCRN_CHCR0	(DCRN_CHCR_BASE + 0x0)	/* Chip Control Register 1 */
#define DCRN_CHCR1	(DCRN_CHCR_BASE + 0x1)	/* Chip Control Register 2 */
#endif
#define CHR1_PCIPW	0x00008000	/* PCI Int enable/Peripheral Write enable */

#ifdef DCRN_CHPSR_BASE
#define DCRN_CHPSR	(DCRN_CHPSR_BASE + 0x0)	/* Chip Pin Strapping */
#endif

#ifdef DCRN_CPMFR_BASE
#define DCRN_CPMFR	(DCRN_CPMFR_BASE + 0x0)	/* CPM Force */
#endif

#ifdef DCRN_CPMSR_BASE
#define DCRN_CPMSR	(DCRN_CPMSR_BASE + 0x0)	/* CPM Status */
#define DCRN_CPMER	(DCRN_CPMSR_BASE + 0x1)	/* CPM Enable */
#endif

#ifdef DCRN_DCP0_BASE
/* Decompression Controller Address */
#define DCRN_DCP0_CFGADDR	(DCRN_DCP0_BASE + 0x0)
/* Decompression Controller Data */
#define DCRN_DCP0_CFGDATA	(DCRN_DCP0_BASE + 0x1)
#else
#define DCRN_DCP0_CFGADDR	0x0
#define DCRN_DCP0_CFGDATA	0x0
#endif

#ifdef DCRN_DMA0_BASE
/* DMA Channel Control Register 0 */
#define DCRN_DMACR0	(DCRN_DMA0_BASE + 0x0)
#define DCRN_DMACT0	(DCRN_DMA0_BASE + 0x1)	/* DMA Count Register 0 */
/* DMA Destination Address Register 0 */
#define DCRN_DMADA0	(DCRN_DMA0_BASE + 0x2)
/* DMA Source Address Register 0 */
#define DCRN_DMASA0	(DCRN_DMA0_BASE + 0x3)
#ifdef DCRNCAP_DMA_CC
/* DMA Chained Count Register 0 */
#define DCRN_DMACC0	(DCRN_DMA0_BASE + 0x4)
#endif
#ifdef DCRNCAP_DMA_SG
/* DMA Scatter/Gather Descriptor Addr 0 */
#define DCRN_ASG0	(DCRN_DMA0_BASE + 0x4)
#endif
#endif

#ifdef DCRN_DMA1_BASE
/* DMA Channel Control Register 1 */
#define DCRN_DMACR1	(DCRN_DMA1_BASE + 0x0)
#define DCRN_DMACT1	(DCRN_DMA1_BASE + 0x1)	/* DMA Count Register 1 */
/* DMA Destination Address Register 1 */
#define DCRN_DMADA1	(DCRN_DMA1_BASE + 0x2)
/* DMA Source Address Register 1 */
#define DCRN_DMASA1	(DCRN_DMA1_BASE + 0x3)	/* DMA Source Address Register 1 */
#ifdef DCRNCAP_DMA_CC
/* DMA Chained Count Register 1 */
#define DCRN_DMACC1	(DCRN_DMA1_BASE + 0x4)
#endif
#ifdef DCRNCAP_DMA_SG
/* DMA Scatter/Gather Descriptor Addr 1 */
#define DCRN_ASG1	(DCRN_DMA1_BASE + 0x4)
#endif
#endif

#ifdef DCRN_DMA2_BASE
#define DCRN_DMACR2	(DCRN_DMA2_BASE + 0x0)	/* DMA Channel Control Register 2 */
#define DCRN_DMACT2	(DCRN_DMA2_BASE + 0x1)	/* DMA Count Register 2 */
#define DCRN_DMADA2	(DCRN_DMA2_BASE + 0x2)	/* DMA Destination Address Register 2 */
#define DCRN_DMASA2	(DCRN_DMA2_BASE + 0x3)	/* DMA Source Address Register 2 */
#ifdef DCRNCAP_DMA_CC
#define DCRN_DMACC2	(DCRN_DMA2_BASE + 0x4)	/* DMA Chained Count Register 2 */
#endif
#ifdef DCRNCAP_DMA_SG
#define DCRN_ASG2	(DCRN_DMA2_BASE + 0x4)	/* DMA Scatter/Gather Descriptor Addr 2 */
#endif
#endif

#ifdef DCRN_DMA3_BASE
#define DCRN_DMACR3	(DCRN_DMA3_BASE + 0x0)	/* DMA Channel Control Register 3 */
#define DCRN_DMACT3	(DCRN_DMA3_BASE + 0x1)	/* DMA Count Register 3 */
#define DCRN_DMADA3	(DCRN_DMA3_BASE + 0x2)	/* DMA Destination Address Register 3 */
#define DCRN_DMASA3	(DCRN_DMA3_BASE + 0x3)	/* DMA Source Address Register 3 */
#ifdef DCRNCAP_DMA_CC
#define DCRN_DMACC3	(DCRN_DMA3_BASE + 0x4)	/* DMA Chained Count Register 3 */
#endif
#ifdef DCRNCAP_DMA_SG
#define DCRN_ASG3	(DCRN_DMA3_BASE + 0x4)	/* DMA Scatter/Gather Descriptor Addr 3 */
#endif
#endif

#ifdef DCRN_DMASR_BASE
#define DCRN_DMASR	(DCRN_DMASR_BASE + 0x0)	/* DMA Status Register */
#ifdef DCRNCAP_DMA_SG
#define DCRN_ASGC	(DCRN_DMASR_BASE + 0x3)	/* DMA Scatter/Gather Command */
/* don't know if these two registers always exist if scatter/gather exists */
#define DCRN_POL	(DCRN_DMASR_BASE + 0x6)	/* DMA Polarity Register */
#define DCRN_SLP	(DCRN_DMASR_BASE + 0x5)	/* DMA Sleep Register */
#endif
#endif

#ifdef DCRN_EBC_BASE
#define DCRN_EBCCFGADR	(DCRN_EBC_BASE + 0x0)	/* Peripheral Controller Address */
#define DCRN_EBCCFGDATA	(DCRN_EBC_BASE + 0x1)	/* Peripheral Controller Data */
#endif

#ifdef DCRN_EXIER_BASE
#define DCRN_EXIER	(DCRN_EXIER_BASE + 0x0)	/* External Interrupt Enable Register */
#endif

#ifdef DCRN_EXISR_BASE
#define DCRN_EXISR	(DCRN_EXISR_BASE + 0x0)	/* External Interrupt Status Register */
#endif

#define EXIER_CIE	0x80000000	/* Critical Interrupt Enable */
#define EXIER_SRIE	0x08000000	/* Serial Port Rx Int. Enable */
#define EXIER_STIE	0x04000000	/* Serial Port Tx Int. Enable */
#define EXIER_JRIE	0x02000000	/* JTAG Serial Port Rx Int. Enable */
#define EXIER_JTIE	0x01000000	/* JTAG Serial Port Tx Int. Enable */
#define EXIER_D0IE	0x00800000	/* DMA Channel 0 Interrupt Enable */
#define EXIER_D1IE	0x00400000	/* DMA Channel 1 Interrupt Enable */
#define EXIER_D2IE	0x00200000	/* DMA Channel 2 Interrupt Enable */
#define EXIER_D3IE	0x00100000	/* DMA Channel 3 Interrupt Enable */
#define EXIER_E0IE	0x00000010	/* External Interrupt 0 Enable */
#define EXIER_E1IE	0x00000008	/* External Interrupt 1 Enable */
#define EXIER_E2IE	0x00000004	/* External Interrupt 2 Enable */
#define EXIER_E3IE	0x00000002	/* External Interrupt 3 Enable */
#define EXIER_E4IE	0x00000001	/* External Interrupt 4 Enable */

#ifdef DCRN_IOCR_BASE
#define DCRN_IOCR	(DCRN_IOCR_BASE + 0x0)	/* Input/Output Configuration Register */
#endif
#define IOCR_E0TE	0x80000000
#define IOCR_E0LP	0x40000000
#define IOCR_E1TE	0x20000000
#define IOCR_E1LP	0x10000000
#define IOCR_E2TE	0x08000000
#define IOCR_E2LP	0x04000000
#define IOCR_E3TE	0x02000000
#define IOCR_E3LP	0x01000000
#define IOCR_E4TE	0x00800000
#define IOCR_E4LP	0x00400000
#define IOCR_EDT	0x00080000
#define IOCR_SOR	0x00040000
#define IOCR_EDO	0x00008000
#define IOCR_2XC	0x00004000
#define IOCR_ATC	0x00002000
#define IOCR_SPD	0x00001000
#define IOCR_BEM	0x00000800
#define IOCR_PTD	0x00000400
#define IOCR_ARE	0x00000080
#define IOCR_DRC	0x00000020
#define IOCR_RDM(x)	(((x) & 0x3) << 3)
#define IOCR_TCS	0x00000004
#define IOCR_SCS	0x00000002
#define IOCR_SPC	0x00000001

#define DCRN_MALCR(base)	(base + 0x0)	/* MAL Configuration */
#define DCRN_MALDBR(base)	((base) + 0x3)	/* Debug Register */
#define DCRN_MALESR(base)	((base) + 0x1)	/* Error Status */
#define DCRN_MALIER(base)	((base) + 0x2)	/* Interrupt Enable */
#define DCRN_MALTXCARR(base)	((base) + 0x5)	/* TX Channed Active Reset Register */
#define DCRN_MALTXCASR(base)	((base) + 0x4)	/* TX Channel Active Set Register */
#define DCRN_MALTXDEIR(base)	((base) + 0x7)	/* Tx Descriptor Error Interrupt */
#define DCRN_MALTXEOBISR(base)	((base) + 0x6)	/* Tx End of Buffer Interrupt Status */
#define DCRN_MALRXCARR(base)	((base) + 0x11)	/* RX Channed Active Reset Register */
#define DCRN_MALRXCASR(base)	((base) + 0x10)	/* RX Channel Active Set Register */
#define DCRN_MALRXDEIR(base)	((base) + 0x13)	/* Rx Descriptor Error Interrupt */
#define DCRN_MALRXEOBISR(base)	((base) + 0x12)	/* Rx End of Buffer Interrupt Status */
#define DCRN_MALRXCTP0R(base)	((base) + 0x40)	/* Channel Rx 0 Channel Table Pointer */
#define DCRN_MALRXCTP1R(base)	((base) + 0x41)	/* Channel Rx 1 Channel Table Pointer */
#define DCRN_MALTXCTP0R(base)	((base) + 0x20)	/* Channel Tx 0 Channel Table Pointer */
#define DCRN_MALTXCTP1R(base)	((base) + 0x21)	/* Channel Tx 1 Channel Table Pointer */
#define DCRN_MALTXCTP2R(base)	((base) + 0x22)	/* Channel Tx 2 Channel Table Pointer */
#define DCRN_MALTXCTP3R(base)	((base) + 0x23)	/* Channel Tx 3 Channel Table Pointer */
#define DCRN_MALRCBS0(base)	((base) + 0x60)	/* Channel Rx 0 Channel Buffer Size */
#define DCRN_MALRCBS1(base)	((base) + 0x61)	/* Channel Rx 1 Channel Buffer Size */

 /* DCRN_MALCR */
#define MALCR_MMSR		0x80000000	/* MAL Software reset */
#define MALCR_PLBP_1		0x00400000	/* MAL reqest priority: */
#define MALCR_PLBP_2		0x00800000	/* lowsest is 00 */
#define MALCR_PLBP_3		0x00C00000	/* highest */
#define MALCR_GA		0x00200000	/* Guarded Active Bit */
#define MALCR_OA		0x00100000	/* Ordered Active Bit */
#define MALCR_PLBLE		0x00080000	/* PLB Lock Error Bit */
#define MALCR_PLBLT_1		0x00040000	/* PLB Latency Timer */
#define MALCR_PLBLT_2 		0x00020000
#define MALCR_PLBLT_3		0x00010000
#define MALCR_PLBLT_4		0x00008000
#define MALCR_PLBLT_DEFAULT	0x00078000	/* JSP: Is this a valid default?? */
#define MALCR_PLBB		0x00004000	/* PLB Burst Deactivation Bit */
#define MALCR_OPBBL		0x00000080	/* OPB Lock Bit */
#define MALCR_EOPIE		0x00000004	/* End Of Packet Interrupt Enable */
#define MALCR_LEA		0x00000002	/* Locked Error Active */
#define MALCR_MSD		0x00000001	/* MAL Scroll Descriptor Bit */
/* DCRN_MALESR */
#define MALESR_EVB		0x80000000	/* Error Valid Bit */
#define MALESR_CIDRX		0x40000000	/* Channel ID Receive */
#define MALESR_DE		0x00100000	/* Descriptor Error */
#define MALESR_OEN		0x00080000	/* OPB Non-Fullword Error */
#define MALESR_OTE		0x00040000	/* OPB Timeout Error */
#define MALESR_OSE		0x00020000	/* OPB Slave Error */
#define MALESR_PEIN		0x00010000	/* PLB Bus Error Indication */
#define MALESR_DEI		0x00000010	/* Descriptor Error Interrupt */
#define MALESR_ONEI		0x00000008	/* OPB Non-Fullword Error Interrupt */
#define MALESR_OTEI		0x00000004	/* OPB Timeout Error Interrupt */
#define MALESR_OSEI		0x00000002	/* OPB Slace Error Interrupt */
#define MALESR_PBEI		0x00000001	/* PLB Bus Error Interrupt */
/* DCRN_MALIER */
#define MALIER_DE		0x00000010	/* Descriptor Error Interrupt Enable */
#define MALIER_NE		0x00000008	/* OPB Non-word Transfer Int Enable */
#define MALIER_TE		0x00000004	/* OPB Time Out Error Interrupt Enable */
#define MALIER_OPBE		0x00000002	/* OPB Slave Error Interrupt Enable */
#define MALIER_PLBE		0x00000001	/* PLB Error Interrupt Enable */
/* DCRN_MALTXEOBISR */
#define MALOBISR_CH0		0x80000000	/* EOB channel 1 bit */
#define MALOBISR_CH2		0x40000000	/* EOB channel 2 bit */

#ifdef DCRN_PLB0_BASE
#define DCRN_PLB0_BESR	(DCRN_PLB0_BASE + 0x0)
#define DCRN_PLB0_BEAR	(DCRN_PLB0_BASE + 0x2)
/* doesn't exist on stb03xxx? */
#define DCRN_PLB0_ACR	(DCRN_PLB0_BASE + 0x3)
#endif

#ifdef DCRN_PLB1_BASE
#define DCRN_PLB1_BESR	(DCRN_PLB1_BASE + 0x0)
#define DCRN_PLB1_BEAR	(DCRN_PLB1_BASE + 0x1)
/* doesn't exist on stb03xxx? */
#define DCRN_PLB1_ACR	(DCRN_PLB1_BASE + 0x2)
#endif

#ifdef DCRN_PLLMR_BASE
#define DCRN_PLLMR	(DCRN_PLLMR_BASE + 0x0)	/* PL1 Mode */
#endif

#ifdef DCRN_POB0_BASE
#define DCRN_POB0_BESR0	(DCRN_POB0_BASE + 0x0)
#define DCRN_POB0_BEAR	(DCRN_POB0_BASE + 0x2)
#define DCRN_POB0_BESR1	(DCRN_POB0_BASE + 0x4)
#endif

#define DCRN_UIC_SR(base)	(base + 0x0)
#define DCRN_UIC_ER(base)	(base + 0x2)
#define DCRN_UIC_CR(base)	(base + 0x3)
#define DCRN_UIC_PR(base)	(base + 0x4)
#define DCRN_UIC_TR(base)	(base + 0x5)
#define DCRN_UIC_MSR(base)	(base + 0x6)
#define DCRN_UIC_VR(base)	(base + 0x7)
#define DCRN_UIC_VCR(base)	(base + 0x8)

#ifdef DCRN_SDRAM0_BASE
#define DCRN_SDRAM0_CFGADDR	(DCRN_SDRAM0_BASE + 0x0)	/* Memory Controller Address */
#define DCRN_SDRAM0_CFGDATA	(DCRN_SDRAM0_BASE + 0x1)	/* Memory Controller Data */
#endif

#ifdef DCRN_OCM0_BASE
#define DCRN_OCMISARC	(DCRN_OCM0_BASE + 0x0)	/* OCM Instr Side Addr Range Compare */
#define DCRN_OCMISCR	(DCRN_OCM0_BASE + 0x1)	/* OCM Instr Side Control */
#define DCRN_OCMDSARC	(DCRN_OCM0_BASE + 0x2)	/* OCM Data Side Addr Range Compare */
#define DCRN_OCMDSCR	(DCRN_OCM0_BASE + 0x3)	/* OCM Data Side Control */
#endif

#ifdef CONFIG_405EZ
/* DCR defines */
#define DCRN_CPMSR_BASE         0x0BA
#define DCRN_CPMFR_BASE         0x0B9

#define DCRN_CPR_BASE           0x0C
#define DCRN_CPR_CFGADDR        (DCRN_CPR_BASE+0x00)
#define DCRN_CPR_CFGDATA        (DCRN_CPR_BASE+0x01)

#define CPR_CLKUPD              0x0020
#define CPR_ENPLLCH             0x40000000 /* Enable CPR PLL changes */
#define CPR_ENDVCH              0x20000000 /* Enable CPR Sys Divider changes */

#define CPR_PLLC                0x0040
#define CPR_PLLC_SRC_MASK       0x20000000
#define CPR_PLL_TUNING_MASK     0x000003FF

#define CPR_PLLD		0x0060
#define CPR_PLLD_FBDV_MASK      0x1F000000 /* PLL feedback divider value */
#define CPR_PLLD_FWDVA_MASK     0x000F0000 /* PLL forward divider A value */
#define CPR_PLLD_FWDVB_MASK     0x00000700 /* PLL forward divider B value */

#define CPR_PRIMAD		0x0080
#define CPR_PRIMAD_CPUDV_MASK	0x0F000000
#define CPR_PRIMAD_PLBDV_MASK	0x000F0000
#define CPR_PRIMAD_OPBDV_MASK	0x00000F00
#define CPR_PRIMAD_EBCDV_MASK	0x0000000F

#define CPR_PERD0               0x00E0
#define CPR_PERD0_PWMDV_MASK	0xFF000000
#define CPR_PERD0_SPIDV_MASK	0x000F0000
#define CPR_PERD0_U0DV_MASK	0x0000FF00
#define CPR_PERD0_U1DV_MASK	0x000000FF

#define CPR_PERD1		0x00E1
#define CPR_PERD1_DACDV_MASK	0xFF000000
#define CPR_PERD1_ADCDV_MASK	0x00FF0000
#define CPR_PERD1_EMACDV_MASK	0x0000FF00

/* Clock and Power Management (CPM) */
#define IBM_CPM_ADC             0x10000000  /* ADC Logic */
#define IBM_CPM_OCM             0x04000000  /* OCM interface */
#define IBM_CPM_MADMAL          0x02000000  /* MADMAL interface */
#define IBM_CPM_OPB2PLB         0x01000000  /* OPB to PLB bridge */
#define IBM_CPM_CAN0            0x00800000  /* CAN Controller 0 */
#define IBM_CPM_CAN1            0x00400000  /* CAN Controller 1 */
#define IBM_CPM_SPI             0x00200000  /* CAN Controller 1 */
#define IBM_CPM_1588            0x00100000  /* IEEE 1588 */
#define IBM_CPM_EMACTX          0x00080000  /* Ethernet MAC TX Clock */
#define IBM_CPM_EMACRX          0x00040000  /* Ethernet MAC RX Clock */
#define IBM_CPM_EMAC0           0x00020000  /* Ethernet MAC Management clock */
#define IBM_CPM_UIC             0x00010000  /* Universal Interrupt Controller */
#define IBM_CPM_CPU             0x00008000  /* 405 processor core */
#define IBM_CPM_NDFC            0x00002000  /* NAND Flash Controller */
#define IBM_CPM_GPIO0           0x00001000  /* General Purpose IO Core 0 */
#define IBM_CPM_GPIO1           0x00000800  /* General Purpose IO Core 1 */
#define IBM_CPM_TMRCLK          0x00000400  /* CPU timers */
#define IBM_CPM_PWM             0x00000200  /* PWM - Chameleon Timer */
#define IBM_CPM_PLB             0x00000100  /* Processor Local Bus */
#define IBM_CPM_PLB2OPB         0x00000080  /* PLB to OPB bridge */
#define IBM_CPM_DMA             0x00000040  /* DMA controller */
#define IBM_CPM_IIC0            0x00000010  /* IIC interface */
#define IBM_CPM_DAC             0x00000008  /* DAC Logic */
#define IBM_CPM_DACCUR          0x00000004  /* DAC Logic */
#define IBM_CPM_UART0           0x00000001  /* serial port 0 */
#define IBM_CPM_UART1           0x00000002  /* serial port 1 */

#define DFLT_IBM4xx_PM		~( IBM_CPM_CPU | IBM_CPM_DMA \
					| IBM_CPM_OPB  | IBM_CPM_EBC \
					| IBM_CPM_OBRG | IBM_CPM_PLB \
					| IBM_CPM_UIC  | IBM_CPM_TMRCLK)

#define DCRN_DMA0_BASE     0x100
#define DCRN_DMA1_BASE     0x108
#define DCRN_DMA2_BASE     0x110
#define DCRN_DMA3_BASE     0x118

/* DMA Channel 0 */
#define DCRN_DMA_CR0      (DCRN_DMA0_BASE + 0x0)    /* DMA Channel Control */
#define DCRN_DMA_CT0      (DCRN_DMA0_BASE + 0x1)    /* DMA Count */
#define DCRN_DMA_DA0      (DCRN_DMA0_BASE + 0x2)    /* DMA Dest Addr */
#define DCRN_DMA_SA0      (DCRN_DMA0_BASE + 0x3)    /* DMA Src Addr */
#define DCRN_DMA_SG0      (DCRN_DMA0_BASE + 0x4)    /* DMA SG Desc Addr */

/* DMA Channel 1 */
#define DCRN_DMA_CR1      (DCRN_DMA1_BASE + 0x0)    /* DMA Channel Control */
#define DCRN_DMA_CT1      (DCRN_DMA1_BASE + 0x1)    /* DMA Count */
#define DCRN_DMA_DA1      (DCRN_DMA1_BASE + 0x2)    /* DMA Dest Addr */
#define DCRN_DMA_SA1      (DCRN_DMA1_BASE + 0x3)    /* DMA Src Addr */
#define DCRN_DMA_SG1      (DCRN_DMA1_BASE + 0x4)    /* DMA SG Desc Addr */

/* DMA Channel 2 */
#define DCRN_DMA_CR2      (DCRN_DMA2_BASE + 0x0)    /* DMA Channel Control */
#define DCRN_DMA_CT2      (DCRN_DMA2_BASE + 0x1)    /* DMA Count */
#define DCRN_DMA_DA2      (DCRN_DMA2_BASE + 0x2)    /* DMA Dest Addr */
#define DCRN_DMA_SA2      (DCRN_DMA2_BASE + 0x3)    /* DMA Src Addr */
#define DCRN_DMA_SG2      (DCRN_DMA2_BASE + 0x4)    /* DMA SG Desc Addr */

/* DMA Channel 3 */
#define DCRN_DMA_CR3      (DCRN_DMA3_BASE + 0x0)    /* DMA Channel Control */
#define DCRN_DMA_CT3      (DCRN_DMA3_BASE + 0x1)    /* DMA Count */
#define DCRN_DMA_DA3      (DCRN_DMA3_BASE + 0x2)    /* DMA Dest Addr */
#define DCRN_DMA_SA3      (DCRN_DMA3_BASE + 0x3)    /* DMA Src Addr */
#define DCRN_DMA_SG3      (DCRN_DMA3_BASE + 0x4)    /* DMA SG Desc Addr */

#define DCRN_DMA_SR        0x120   /* DMA Status Register */
#define DCRN_DMA_SGC       0x123   /* DMA Scatter/Gather Command */
#define DCRN_DMA_SLP       0x125   /* DMA Sleep Mode Register */

#define DCRNCAP_DMA_SG		1	/* have DMA scatter/gather capability */

#define DCRN_DMASR_BASE		0x120
#define DCRN_EBC_BASE		0x012
#define DCRN_DCP0_BASE		0x014
#define DCRN_MAL_BASE           0x380
#define DCRN_OCM0_BASE		0x018
#define DCRN_PLB0_BASE		0x084
#define DCRN_PLLMR_BASE		0x0B0
#define DCRN_OBRG_BASE          0x0B0
#define DCRN_POB0_BASE		0x0A0
#define DCRN_UIC0_BASE		0x0C0
#define UIC0			DCRN_UIC0_BASE

#define DCRN_UIC1_BASE		0x0E0 /* 1588 UIC Snapshot Source */
#define UIC1			DCRN_UIC1_BASE
#define UIC0_UIC1NC		0x08000000 /* 1588 SYNC Interrupt */
#define NR_UICS			2

#define DCRN_SDR_ICINTSTAT	0x4510
#define ICINSTAT_ICRX		0x80000000
#define ICINSTAT_ICTX0		0x40000000
#define ICINSTAT_ICTX1		0x20000000
#define ICINSTAT_ICTX		0x60000000

/*
 * EMAC interrupt coalesing
 */
#define MAX_COAL_FRAMES		0xFF
#define MAX_COAL_TIMER		0xFFFFFFFF

#define DCRN_SDR_RXICCR0	0x4400
#define DCRN_SDR_RXICCR1	0x4410
#define DCRN_SDR_TX0ICCR0	0x4430
#define DCRN_SDR_TX0ICCR1	0x4440
#define	DCRN_SDR_ICINT_EN	0x4500

#define	RX_ICCR_ICEN		0x80000000
#define	RX_ICCR_FLUSH		0x20000000
#define	RX_ICCR_COR_EN		0x08000000
#define	RX_ICCR_THR_MASK	0x01FE0000

#define TX_ICCR_ICEN		0x80000000
#define TX_ICCR_FLUSH		0x20000000
#define TX_ICCR_COR_EN		0x08000000
#define	TX_ICCR_THR_MASK	0x01FE0000

#define RX_ICCR_ICFT_ENCODE(val) (((uint)val<<17) & 0x01FE0000)
#define TX_ICCR_ICFT_ENCODE(val) (((uint)val<<17) & 0x01FE0000)

#define ICRX_EN			0x80000000
#define ICTX0_EN		0x40000000

#endif /* CONFIG_405EZ */

#define DCRN_SDR_UART0		0x0120
#define DCRN_SDR_UART1		0x0121
#define SDR_UART_U0DIV		0x000000ff
#define SDR_UART_U0EC		0x00800000

#define DCRN_SDR_CONFIG_ADDR    0x00E
#define DCRN_SDR_CONFIG_DATA    0x00F

/* SDR read/write helper macros */
#define SDR_READ(offset) ({			\
	mtdcr(DCRN_SDR_CONFIG_ADDR, offset);	\
	mfdcr(DCRN_SDR_CONFIG_DATA);})
#define SDR_WRITE(offset, data) ({		\
	mtdcr(DCRN_SDR_CONFIG_ADDR, offset);	\
	mtdcr(DCRN_SDR_CONFIG_DATA, data);})

/* SDR read/write helper macros */
#define CPR_READ(offset) ({			\
	mtdcr(DCRN_CPR_CFGADDR, offset);	\
	mfdcr(DCRN_CPR_CFGDATA);})
#define CPR_WRITE(offset, data) ({		\
	mtdcr(DCRN_CPR_CFGADDR, offset);	\
	mtdcr(DCRN_CPR_CFGDATA, data);})

#endif				/* __ASM_IBM405_H__ */
#endif				/* __KERNEL__ */
