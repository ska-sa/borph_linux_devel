/***********************************************************************
 *
 * (C) Copyright 2008
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * (C) Copyright 2004-2008
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
#include <linux/jiffies.h>
#include <linux/wd.h>

#include <asm/io.h>

/*
 * This file implements low-level functions to access watchdog timer
 * on LWMON5 platform for generic watchdog driver.
 */

#define WD_LWMON5_TIMEOUT	2 /* watchdog expires after 2 seconds */
#define WD_LWMON5_PERIOD	1 /* should be re-triggered after 1 second */
#define WD_LWMON5_COUNT		0xffff /* default count value for gpt0 */
#define WD_LWMON5_MAX_PRESCALE	0xffff /* maximum value of prescale */

#define CFG_GPIO1_WATCHDOG	31

#define GPIO_VAL(gpio)		(0x80000000 >> (gpio))

/*
 * We need to map the GPIO output register very early on bootup.
 * Mapping in wd_lwmon5_init() is too late since the WD needs to
 * get kicked earlier because of the very short timeout. So the
 * GPIO output register gets mapped in the board specific platform
 * code upon bootup.
 */
extern u32 __iomem *lwmon5_gpio1_or;

static int wd_running = 0;

/*
 * This function performs watchdog timer initialization.
 * Argument tpp is period for re-trigger (in jiffies).
 */
int wd_lwmon5_init(unsigned long *tpp)
{
	/*
	 * Not much to do here, since the watchdog is already running
	 * (setup in U-Boot) and has a fixed timeout value
	 */

	/*
	 * Return number of seconds for re-triggering, calculations are
	 * done in jiffies
	 */
	*tpp = HZ / 25;		/* 1s/25 = 40ms */

	wd_running = 1;

	return 0;
}

/*
 * This function triggers the external watchdog
 */
void wd_lwmon5_kick(void)
{
	u32 val;

	val = in_be32(lwmon5_gpio1_or);
	val ^= GPIO_VAL(CFG_GPIO1_WATCHDOG);
	out_be32(lwmon5_gpio1_or, val);
}

void wd_lwmon5_kick_early(void)
{
	/*
	 * When the "real" watchdog is running, the early bootup
	 * kick function is not needed anymore, so just return
	 */
	if (wd_running)
		return;

	wd_lwmon5_kick();
}

/*
 * This function stops watchdog timer
 */
void wd_lwmon5_delete(void)
{
	/* disable watchdog not possible on LWMON5 */
}

/*
 * This function performs emergency reboot.
 */
void wd_lwmon5_machine_restart(void)
{
	machine_restart(NULL);
}
