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
 * Support up to 4 devices simultaneously by
 * using PSC1, PSC2, PSC3, PSC6 as I2S port
 *
 */

#include <linux/config.h>
#include <sound/driver.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <sound/initval.h>
#include <sound/control.h>

#include "mpc52xx-aic26.h"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPC52XX AIC26 Alsa mixer driver");

#define MIXER_NAME		"Mixer AIC26"

static int snd_mpc52xx_aic26_cap_sw_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int snd_mpc52xx_aic26_cap_sw_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR);

	ucontrol->value.integer.value[0] = data.codec_adc_gain.admut ? 0 : 1;

	return 0;
}

static int snd_mpc52xx_aic26_cap_sw_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR);

	if(ucontrol->value.integer.value[0] == (data.codec_adc_gain.admut ? 0 : 1)) {
		return 0;
	}

	data.codec_adc_gain.admut = (ucontrol->value.integer.value[0] ? 0 : 1);
	audio_aic26_write(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR, data.value);

	return 1;
}



static int snd_mpc52xx_aic26_pb_sw_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 2; /* Playback has stereo channel */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int snd_mpc52xx_aic26_pb_sw_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR);

	ucontrol->value.integer.value[0] = data.codec_dac_gain.dalmu ? 0 : 1;
	ucontrol->value.integer.value[1] = data.codec_dac_gain.darmu ? 0 : 1;

	return 0;
}

static int snd_mpc52xx_aic26_pb_sw_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR);

	if((ucontrol->value.integer.value[0] == (data.codec_dac_gain.dalmu ? 0 : 1)) &&
		(ucontrol->value.integer.value[1] == (data.codec_dac_gain.darmu ? 0 : 1))) {
		return 0;
	}

	data.codec_dac_gain.dalmu = ucontrol->value.integer.value[0] ? 0 : 1;
	data.codec_dac_gain.darmu = ucontrol->value.integer.value[1] ? 0 : 1;
	audio_aic26_write(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR, data.value);

	return 1;
}

static int snd_mpc52xx_aic26_pb_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;	/* -63.5 dB */
	uinfo->value.integer.max = 127;	/*  0 dB */

	return 0;
}

static int snd_mpc52xx_aic26_pb_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR);

	ucontrol->value.integer.value[0] = 127 - data.codec_dac_gain.dalvl;
	ucontrol->value.integer.value[1] = 127 - data.codec_dac_gain.darvl;

	return 0;
}

static int snd_mpc52xx_aic26_pb_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR);

	if((ucontrol->value.integer.value[0] == (127 - data.codec_dac_gain.dalvl)) &&
		(ucontrol->value.integer.value[1] == (127 -data.codec_dac_gain.darvl))) {
		return 0;
	}

	data.codec_dac_gain.dalvl = 127 - ucontrol->value.integer.value[0];
	data.codec_dac_gain.darvl = 127 - ucontrol->value.integer.value[1];
	audio_aic26_write(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR, data.value);

	return 1;
}

static int snd_mpc52xx_aic26_cap_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	/* Assume AGC is disable */
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;	/*  0 dB */
	uinfo->value.integer.max = 119;	/*  59.5 dB */

	return 0;
}

static int snd_mpc52xx_aic26_cap_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR);

	if(data.codec_adc_gain.adpga > 119)
		data.codec_adc_gain.adpga = 119;

	ucontrol->value.integer.value[0] = data.codec_adc_gain.adpga;

	return 0;
}

static int snd_mpc52xx_aic26_cap_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
	union aic26_data data;

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR);

	if(ucontrol->value.integer.value[0] == data.codec_adc_gain.adpga) {
		return 0;
	}

	data.codec_adc_gain.adpga = ucontrol->value.integer.value[0];
	audio_aic26_write(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR, data.value);

	return 1;
}

static int snd_mpc52xx_aic26_cap_sel_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
        static char *texts[3] = { "Mic in", "Aux in", "Diff-input MicIn and Aux" };

        uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
        uinfo->count = 1;
        uinfo->value.enumerated.items = 3;
        if (uinfo->value.enumerated.item > 2)
		uinfo->value.enumerated.item = 2;
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);
        return 0;
}

static int snd_mpc52xx_aic26_cap_sel_get(struct snd_kcontrol * kcontrol,
		struct snd_ctl_elem_value * ucontrol)
{
        struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
        union aic26_data data;

        data.value = audio_aic26_read(chip->spi_ss, AIC26_AUDIO_CTRL1_ADDR);
	if(data.audio_ctrl1.adcin == 3)
		data.audio_ctrl1.adcin = 2;

	ucontrol->value.enumerated.item[0] =  data.audio_ctrl1.adcin;

        return 0;
}

static int snd_mpc52xx_aic26_cap_sel_put(struct snd_kcontrol * kcontrol,
                                        struct snd_ctl_elem_value * ucontrol)
{
        struct snd_card_mpc52xx_aic26 *chip = snd_kcontrol_chip(kcontrol);
        union aic26_data data;

        data.value = audio_aic26_read(chip->spi_ss, AIC26_AUDIO_CTRL1_ADDR);

	if(data.audio_ctrl1.adcin == 3)
		data.audio_ctrl1.adcin = 2;

	if(ucontrol->value.enumerated.item[0] == data.audio_ctrl1.adcin)
		return 0;

	data.audio_ctrl1.adcin = ucontrol->value.enumerated.item[0];

	audio_aic26_write(chip->spi_ss, AIC26_AUDIO_CTRL1_ADDR, data.value);

	return 1;
}

static snd_kcontrol_new_t snd_mpc52xx_aic26_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Playback Volume",
		.info = snd_mpc52xx_aic26_pb_info,
		.get = snd_mpc52xx_aic26_pb_get,
		.put = snd_mpc52xx_aic26_pb_put,
		.private_value = 0
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Playback Switch",
		.info = snd_mpc52xx_aic26_pb_sw_info,
		.get = snd_mpc52xx_aic26_pb_sw_get,
		.put = snd_mpc52xx_aic26_pb_sw_put,
		.private_value = 0
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Capture Volume",
		.info = snd_mpc52xx_aic26_cap_info,
		.get = snd_mpc52xx_aic26_cap_get,
		.put = snd_mpc52xx_aic26_cap_put,
		.private_value = 0
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Capture Switch",
		.info = snd_mpc52xx_aic26_cap_sw_info,
		.get = snd_mpc52xx_aic26_cap_sw_get,
		.put = snd_mpc52xx_aic26_cap_sw_put,
		.private_value = 0
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Mic/AUX Capture",
		.info = snd_mpc52xx_aic26_cap_sel_info,
		.get = snd_mpc52xx_aic26_cap_sel_get,
		.put = snd_mpc52xx_aic26_cap_sel_put,
		.private_value = 0
	}
};

void snd_mpc52xx_aic26_exit_mixer(struct snd_card_mpc52xx_aic26 *chip)
{
}

/* Set default master volume for playback and capture */
void snd_mpc52xx_aic26_init_mixer(struct snd_card_mpc52xx_aic26 *chip)
{
	union aic26_data data;

	/* Unmute ADC and DAC */
	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR);
	data.codec_adc_gain.admut = 0;
	data.codec_adc_gain.adpga = 59; /* Default volume */
	audio_aic26_write(chip->spi_ss, AIC26_CODEC_ADC_GAIN_ADDR, data.value);

	data.value = audio_aic26_read(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR);
	data.codec_dac_gain.dalmu = 0;
	data.codec_dac_gain.darmu = 0;
	data.codec_dac_gain.dalvl = 0; /* Maximum volume */
	data.codec_dac_gain.darvl = 0; /* Maximum volume */
	audio_aic26_write(chip->spi_ss, AIC26_CODEC_DAC_GAIN_ADDR, data.value);
}

void snd_omap_exit_mixer(struct snd_card_mpc52xx_aic26 *chip)
{
}

int snd_mpc52xx_aic26_mixer(snd_card_t *card)
{
	struct snd_card_mpc52xx_aic26 *chip;
	unsigned int idx;
	int err;
	chip = (struct snd_card_mpc52xx_aic26 *)card->private_data;

	strcpy(card->mixername, MIXER_NAME);

	/* Registering alsa mixer controls */
	for (idx = 0; idx < ARRAY_SIZE(snd_mpc52xx_aic26_controls); idx++) 
		if ((err = snd_ctl_add(card, snd_ctl_new1(&snd_mpc52xx_aic26_controls[idx], chip))) < 0)
			return err;

	return 0;
}

