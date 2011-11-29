/***********************************************************************
 *
 * (C) Copyright 2004, 2007
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Adapted to Linux 2.6 by Piotr Kruszynski <ppk@semihalf.com>:
 * - separated generic and hardware dependent functions
 * - automatic calculations of gpt prescale value based on IPB
 * - code cleanup
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 ***********************************************************************/
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/wd.h>

#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/mpc52xx.h>

/*
 * This file implements low-level functions to access watchdog timer
 * on MPC5200 platform for generic watchdog driver.
 */

#define WD_MPC5200_TIMEOUT	2 /* watchdog expires after 2 seconds */
#define WD_MPC5200_PERIOD	1 /* should be re-triggered after 1 second */
#define WD_MPC5200_COUNT	0xffff /* default count value for wdt */
#define WD_MPC5200_MAX_PRESCALE	0xffff /* maximum value of prescale */

/*
 * This function performs watchdog timer initialization.
 * Argument tpp is period for re-trigger (in jiffies).
 */
int wd_mpc5200_init(unsigned long *tpp)
{
	struct device_node *np;
	unsigned int ipb_freq;
	u32 prescale;
	volatile struct mpc52xx_gpt *mpc52xx_wdt;

	mpc52xx_wdt = mpc52xx_get_wdt();

	/* check if we have wdt */
	if (mpc52xx_wdt == NULL) {
		printk("%s: couldn't find wdt\n", __FUNCTION__);
		return -ENXIO;
	}

	/* find MPC5200 cpu node */
	np = of_find_node_by_type(NULL, "cpu");
	if (np == NULL) {
		printk("%s: couldn't find cpu node\n", __FUNCTION__);
		return -ENXIO;
	}

	/* find IPB frequency and put node */
	ipb_freq = mpc52xx_find_ipb_freq(np);
	of_node_put(np);

	if (ipb_freq == 0) {
		printk("%s: couldn't find IPB frequency\n", __FUNCTION__);
		return -ENXIO;
	}

	/* calculate correct value of prescale */
	prescale = (ipb_freq / WD_MPC5200_COUNT) * WD_MPC5200_TIMEOUT;
	if (prescale == 0 || prescale > WD_MPC5200_MAX_PRESCALE) {
		printk("%s: wrong value of IPB frequency(%u), couldn't "
			"calculate prescale\n", __FUNCTION__, ipb_freq);
		return -ENXIO;
	}

	/* disable watchdog */
	mpc52xx_wdt->mode = 0x00000000;

	/* configure watchdog */
	mpc52xx_wdt->mode |= 0x00000004;
	mpc52xx_wdt->mode |= 0x00001000;

	/* set watchdog timeout */
	mpc52xx_wdt->count = (prescale << 16) | WD_MPC5200_COUNT;

	/* enable watchdog */
	mpc52xx_wdt->mode |= 0x00008000;

	/*
	 * return number of seconds for re-triggering, calculations are
	 * done in jiffies
	 */
	*tpp = WD_MPC5200_PERIOD * HZ;

	return 0;
}

/*
 * This function resets watchdog counter.
 */
void wd_mpc5200_kick(void)
{
	volatile struct mpc52xx_gpt *mpc52xx_wdt;

	mpc52xx_wdt = mpc52xx_get_wdt();

	/* watchdog internal trigger */
	mpc52xx_wdt->mode |= 0xA5000000;
}

/*
 * This function stops watchdog timer
 */
void wd_mpc5200_delete(void)
{
	volatile struct mpc52xx_gpt *mpc52xx_wdt;

	mpc52xx_wdt = mpc52xx_get_wdt();

	/* disable watchdog */
	mpc52xx_wdt->mode &= ~0x00008000;
}

/*
 * This function performs emergency reboot.
 */
void wd_mpc5200_machine_restart(void)
{
	machine_restart(NULL);
}
