/*
 * Alsa Driver for AIC26 codec on MPC5200B platform board
 *
 * Copyright (C) 2006 AppSpec Computer Technologies Corp.
 *                    Jeff Gibbons <jeff.gibbons@appspec.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 *
 */

#ifndef __MPC52XX_AIC26_H
#define __MPC52XX_AIC26_H

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>

#define AIC26_READ_COMMAND_WORD(ADDR)	((1 << 15) | ADDR)
#define AIC26_WRITE_COMMAND_WORD(ADDR)	((0 << 15) | ADDR)

#define AIC26_PAGE_ADDR(PAGE, OFFSET)	((PAGE << 11) | OFFSET << 5)

/* Page 0: Auxillary data registers */
#define AIC26_BAT1_ADDR			AIC26_PAGE_ADDR(0, (0x05))
#define AIC26_BAT2_ADDR			AIC26_PAGE_ADDR(0, (0x06))
#define AIC26_AUX_ADDR			AIC26_PAGE_ADDR(0, (0x07))
#define AIC26_TEMP1_ADDR		AIC26_PAGE_ADDR(0, (0x09))
#define AIC26_TEMP2_ADDR		AIC26_PAGE_ADDR(0, (0x0A))

/* Page 1: Auxillary control registers */
#define AIC26_AUX_ADC_ADDR		AIC26_PAGE_ADDR(1, (0x00))
typedef struct aic26_aux_adc{
	unsigned reserved1:1;
	unsigned adst:1;
	unsigned adscm:4;
	unsigned resol:2;
	unsigned adavg:2;
	unsigned adcr:2;
	unsigned reserved2:3;
	unsigned avgfs:1;
}  __attribute__((packed, aligned(2))) aic26_aux_adc;

#define AIC26_STATUS_ADDR		AIC26_PAGE_ADDR(1, (0x01))
typedef struct aic26_status {
	unsigned neg_dav:2;
	unsigned pwrdn:1;
	unsigned reserved1:1;
	unsigned davail:1;
	unsigned reserved2:4;
	unsigned b1stat:1;
	unsigned b2stat:1;
	unsigned axstat:1;
	unsigned reserved3:1;
	unsigned t1stat:1;
	unsigned t2stat:1;
	unsigned reserved4:1;
} __attribute__((packed, aligned(2))) aic26_status;

#define AIC26_REFERENCE_ADDR		AIC26_PAGE_ADDR(1, (0x03))
typedef struct aic26_reference_ctrl {
	unsigned reserved:11;
	unsigned vrefm:1;
	unsigned rpwudl:2;
	unsigned rpwdn:1;
	unsigned irefc:1;
} __attribute__((packed, aligned(2))) aic26_reference_ctrl;

#define AIC26_RESET_ADDR		AIC26_PAGE_ADDR(1, (0x04))

/* Page 2: Audio control registers */
#define AIC26_AUDIO_CTRL1_ADDR		AIC26_PAGE_ADDR(2, (0x00))
typedef struct aic26_audio_ctrl1 {
	unsigned adchpf:2;
	unsigned adcin:2;
	unsigned wlen:2;
	unsigned datfm:2;
	unsigned reserved:2;
	unsigned dacfs:3;
	unsigned adcfs:3;
} __attribute__((packed, aligned(2))) aic26_audio_ctrl1;

#define AIC26_CODEC_ADC_GAIN_ADDR	AIC26_PAGE_ADDR(2, (0x01))
typedef struct aic26_codec_adc_gain {
	unsigned admut:1;
	unsigned adpga:7;
	unsigned agctg:3;
	unsigned agctc:4;
	unsigned agcen:1;
} __attribute__((packed, aligned(2))) aic26_codec_adc_gain;

#define AIC26_CODEC_DAC_GAIN_ADDR	AIC26_PAGE_ADDR(2, (0x02))
typedef struct aic26_codec_dac_gain {
	unsigned dalmu:1;
	unsigned dalvl:7;
	unsigned darmu:1;
	unsigned darvl:7;
} __attribute__((packed, aligned(2))) aic26_codec_dac_gain;

#define AIC26_CODEC_SIDETONE_ADDR	AIC26_PAGE_ADDR(2, (0x03))
typedef struct aic26_codec_sidetone {
	unsigned astmu:1;
	unsigned astg:7;
	unsigned dstmu:1;
	unsigned dstg:6;
	unsigned astgf:1;
} __attribute__((packed, aligned(2))) aic26_codec_sidetone;

#define AIC26_AUDIO_CTRL2_ADDR		AIC26_PAGE_ADDR(2, (0x04))
typedef struct aic26_audio_ctrl2 {
	unsigned kclen:1;
	unsigned kclac:3;
	unsigned apgass:1;
	unsigned kclfrq:3;
	unsigned kclln:4;
	unsigned dlgaf:1;
	unsigned drgaf:1;
	unsigned dastc:1;
	unsigned adgaf:1;
} __attribute__((packed, aligned(2))) aic26_audio_ctrl2;

#define AIC26_CODEC_POWER_CTRL_ADDR	AIC26_PAGE_ADDR(2, (0x05))
typedef struct aic26_codec_power_ctrl {
	unsigned pwdnc:1;
	unsigned reserved1:1;
	unsigned astpwd:1;
	unsigned daodrc:1;
	unsigned astpwf:1;
	unsigned dapwdn:1;
	unsigned adpwdn:1;
	unsigned vgpwdn:1;
	unsigned adpwdf:1;
	unsigned dapwof:1;
	unsigned adwsf:1;
	unsigned vbias:1;
	unsigned reserved2:2;
	unsigned effctl:1;
	unsigned deempf:1;
} __attribute__((packed, aligned(2))) aic26_codec_power_ctrl;

#define AIC26_AUDIO_CTRL3_ADDR		AIC26_PAGE_ADDR(2, (0x06))
typedef struct aic26_audio_ctrl3 {
	unsigned dmsvol:2;
	unsigned reffs:1;
	unsigned daxfm:1;
	unsigned slvms:1;
	unsigned dapk2pk:2;
	unsigned adcovf:1;
	unsigned dalovf:1;
	unsigned darovf:1;
	unsigned agcnl:2;
	unsigned clpst:1;
	unsigned reserved:3;
} __attribute__((packed, aligned(2))) aic26_audio_ctrl3;

#define AIC26_FILTER_COEFF_L_N0_ADDR	AIC26_PAGE_ADDR(2, (0x07))
#define AIC26_FILTER_COEFF_L_N1_ADDR	AIC26_PAGE_ADDR(2, (0x08))
#define AIC26_FILTER_COEFF_L_N2_ADDR	AIC26_PAGE_ADDR(2, (0x09))
#define AIC26_FILTER_COEFF_L_N3_ADDR	AIC26_PAGE_ADDR(2, (0x0A))
#define AIC26_FILTER_COEFF_L_N4_ADDR	AIC26_PAGE_ADDR(2, (0x0B))
#define AIC26_FILTER_COEFF_L_N5_ADDR	AIC26_PAGE_ADDR(2, (0x0C))
#define AIC26_FILTER_COEFF_L_D1_ADDR	AIC26_PAGE_ADDR(2, (0x0D))
#define AIC26_FILTER_COEFF_L_D2_ADDR	AIC26_PAGE_ADDR(2, (0x0E))
#define AIC26_FILTER_COEFF_L_D4_ADDR	AIC26_PAGE_ADDR(2, (0x0F))
#define AIC26_FILTER_COEFF_L_D5_ADDR	AIC26_PAGE_ADDR(2, (0x10))
#define AIC26_FILTER_COEFF_R_N0_ADDR	AIC26_PAGE_ADDR(2, (0x11))
#define AIC26_FILTER_COEFF_R_N1_ADDR	AIC26_PAGE_ADDR(2, (0x12))
#define AIC26_FILTER_COEFF_R_N2_ADDR	AIC26_PAGE_ADDR(2, (0x13))
#define AIC26_FILTER_COEFF_R_N3_ADDR	AIC26_PAGE_ADDR(2, (0x14))
#define AIC26_FILTER_COEFF_R_N4_ADDR	AIC26_PAGE_ADDR(2, (0x15))
#define AIC26_FILTER_COEFF_R_N5_ADDR	AIC26_PAGE_ADDR(2, (0x16))
#define AIC26_FILTER_COEFF_R_D1_ADDR	AIC26_PAGE_ADDR(2, (0x17))
#define AIC26_FILTER_COEFF_R_D2_ADDR	AIC26_PAGE_ADDR(2, (0x18))
#define AIC26_FILTER_COEFF_R_D4_ADDR	AIC26_PAGE_ADDR(2, (0x19))
#define AIC26_FILTER_COEFF_R_D5_ADDR	AIC26_PAGE_ADDR(2, (0x1A))

#define AIC26_PLL_PROG1_ADDR		AIC26_PAGE_ADDR(2, (0x1B))
typedef struct aic26_pll_prog1 {
	unsigned pllsel:1;
	unsigned qval:4;
	unsigned pval:3;
	unsigned jval:6;
	unsigned reserved:2;
} __attribute__((packed, aligned(2))) aic26_pll_prog1;

#define AIC26_PLL_PROG2_ADDR		AIC26_PAGE_ADDR(2, (0x1C))
typedef struct aic26_pll_prog2 {
	unsigned dval:14;
	unsigned reserved:2;
} __attribute__((packed, aligned(2))) aic26_pll_prog2;

#define AIC26_AUDIO_CTRL4_ADDR		AIC26_PAGE_ADDR(2, (0x1D))
typedef struct aic26_audio_ctrl4 {
	unsigned astrd:1;
	unsigned dastpd:1;
	unsigned asstpd:1;
	unsigned dstpd:1;
	unsigned reserved:1;
	unsigned agc_hyst:2;
	unsigned shckt_dis:1;
	unsigned shckt_pd:1;
	unsigned shckt_flag:1;
	unsigned dac_pop_red:1;
	unsigned dac_pop_red_set1:1;
	unsigned dac_pop_red_set2:2;
	unsigned pgid:2;
} __attribute__((packed, aligned(2))) aic26_audio_ctrl4;

#define AIC26_AUDIO_CTRL5_ADDR		AIC26_PAGE_ADDR(2, (0x1E))
typedef struct aic26_audio_ctrl5 {
	unsigned max_agc_pga:6;
	unsigned agc_nio_deb:3;
	unsigned agc_sig_deb:3;
	unsigned drv_pop_dis:1;
	unsigned drv_pop_len:1;
	unsigned reserved:2;
} __attribute__((packed, aligned(2))) aic26_audio_ctrl5;

/* Common data structure to prevent a lot of casting in program */
union aic26_data {
	u16 value;
	struct aic26_aux_adc		aux_adc;
	struct aic26_status		status;
	struct aic26_reference_ctrl	reference_ctrl;
	struct aic26_audio_ctrl1	audio_ctrl1;
	struct aic26_codec_adc_gain	codec_adc_gain;
	struct aic26_codec_dac_gain	codec_dac_gain;
	struct aic26_codec_sidetone	codec_sidetone;
	struct aic26_audio_ctrl2 	audio_ctrl2;
	struct aic26_codec_power_ctrl	codec_power_ctrl;
	struct aic26_audio_ctrl3	audio_ctrl3;
	struct aic26_pll_prog1		pll_prog1;
	struct aic26_pll_prog2		pll_prog2;
	struct aic26_audio_ctrl4	audio_ctrl4;
	struct aic26_audio_ctrl5	audio_ctrl5;
} __attribute__((packed, aligned(2)));

/* fsref dividers */
#define AIC26_SAMPLE_RATE_DIV_1		0
#define AIC26_SAMPLE_RATE_DIV_1_5	1
#define AIC26_SAMPLE_RATE_DIV_2		2
#define AIC26_SAMPLE_RATE_DIV_3		3
#define AIC26_SAMPLE_RATE_DIV_4		4
#define AIC26_SAMPLE_RATE_DIV_5		5
#define AIC26_SAMPLE_RATE_DIV_5_5	6
#define AIC26_SAMPLE_RATE_DIV_6		7

/*
 * Buffer management for alsa and dma
 */
struct audio_stream {
	spinlock_t dma_lock;
	struct sdma *sdma;	/* dma that device */
	unsigned active:1;	/* dma active or not */
	int irq;		/* IRQ number of that device */
	void* period_start;
	void* period_end;
	void* period_next_pt;
	int period_byte_size;
	snd_pcm_substream_t *stream;	/* the pcm stream */
};

/*
 * Alsa card structure for AIC26
 */
struct snd_card_mpc52xx_aic26 {
	snd_pcm_t *pcm;			/* for suspend callback use */
	int psc_idx;			/* PSC id of that device (0,1,2,5) */
	unsigned char __iomem *membase;	/* read/write[bwl] */
	unsigned long mapbase;		/* for ioremap */
	int spi_ss;			/* SPI SS pin */
	struct audio_stream s[2];	/* playback & capture */
	int irq;			/* IRQ number of for error handling */
};

void audio_aic26_write(int dev, u16 address, u16 data);
u16 audio_aic26_read(int dev, u16 address);

/*********** Function Prototypes *************************/
void audio_dma_callback(void *);

#endif /* __MPC52XX_AIC26_H */
