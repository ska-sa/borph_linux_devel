/***********************************************************************
 *
 * (C) Copyright 2007 Semihalf, Piotr Kruszynski <ppk@semihalf.com>
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

#ifndef _LINUX_WD_HW_H
#define _LINUX_WD_HW_H
#ifdef CONFIG_WD

#include <linux/wd.h>

/*
 * This file provide declaration of functions exported by low-level
 * watchdog drivers. Pointers to this functions are passed to
 * wd_hw_functions structure (see include/linux/wd.h) before
 * generic watchdog driver start.
 *
 * Each function set should have following declaration structure:
 *
 * #ifdef CONFIG_WD_XXXX
 * extern wd_init_t		wd_xxxx_init;
 * extern wd_kick_t		wd_xxxx_kick;
 * extern wd_delete_t		wd_xxxx_delete;
 * extern wd_machine_restart_t	wd_xxxx_machine_restart;
 * #endif /_* CONFIG_WD_XXXX *_/
 *
 * Where xxxx and XXXX is low-level driver name and CONFIG_WD_XXXX
 * is name of option added to drivers/char/watchdog/Kconfig. Additional
 * informations about wd_*_t functions are in include/linux/wd.h.
 */

#ifdef CONFIG_WD_MPC5200
/* Extern declarations of low-level MPC5200 watchdog functions */
extern wd_init_t		wd_mpc5200_init;
extern wd_kick_t		wd_mpc5200_kick;
extern wd_delete_t		wd_mpc5200_delete;
extern wd_machine_restart_t	wd_mpc5200_machine_restart;
#endif /* CONFIG_WD_MPC5200 */

#ifdef CONFIG_WD_LWMON5
/* Extern declarations of low-level LWMON5 watchdog functions */
extern wd_init_t		wd_lwmon5_init;
extern wd_kick_t		wd_lwmon5_kick;
extern wd_delete_t		wd_lwmon5_delete;
extern wd_machine_restart_t	wd_lwmon5_machine_restart;
#endif /* CONFIG_WD_LWMON5 */

#endif /* CONFIG_WD */
#endif /* !_LINUX_WD_HW_H */
