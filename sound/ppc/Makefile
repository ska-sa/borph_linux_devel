#
# Makefile for ALSA
# Copyright (c) 2001 by Jaroslav Kysela <perex@perex.cz>
#

snd-powermac-objs := powermac.o pmac.o awacs.o burgundy.o daca.o tumbler.o keywest.o beep.o
snd-mpc52xx-aic26-objs := mpc52xx-aic26.o mpc52xx-aic26-mixer.o

# Toplevel Module Dependency
obj-$(CONFIG_SND_MPC52xx_AIC26) += snd-aic26.o
obj-$(CONFIG_SND_POWERMAC)	+= snd-powermac.o
obj-$(CONFIG_SND_PS3)		+= snd_ps3.o
