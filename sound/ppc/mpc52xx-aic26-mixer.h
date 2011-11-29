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

#ifndef __MPC52XX_AIC26_MIXER_H
#define __MPC52XX_AIC26_MIXER_H

int snd_mpc52xx_aic26_mixer(snd_card_t *card);
void snd_mpc52xx_aic26_init_mixer(struct snd_card_mpc52xx_aic26 *chip);
void snd_omap_exit_mixer(struct snd_card_mpc52xx_aic26 *chip);

#endif /* __MPC52XX_AIC26_MIXER_H */
