/***********************************************************************
 *
 * (C) Copyright 2004, 2007
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Adapted to Linux 2.6 by Piotr Kruszynski <ppk@semihalf.com>:
 * - separated generic and hardware dependent functions
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
/*
 * The purpose of this header file is to provide an interface for the
 * driver of the watchdog timer wd. In essence this interface
 * consists of the three macros WD_INIT, WD_SERVICE,
 * WD_CLOSE, and its functionality is described as follows:
 *
 * WD_INIT:     opens the driver and initializes the timer to
 *                      300 seconds;
 *
 * WD_SERVICE:  writes the value defined by the macro
 *                      WD_DEF_SERVICE_TIME to the variable,
 *                      which serves as a timer counter;
 *
 * WD_CLOSE:    closes the watchdog driver;
 *
 * Finally there is a macro called WD_SET_SERVICE_TIME(sec)
 * for altering the value written to the timer counter to a value,
 * which is specified by sec.
 */

#ifndef _LINUX_WD_H
#define _LINUX_WD_H

#define WD_MAX_USR_ACTIONS 3

typedef struct wd_param {
	unsigned chainid;
	unsigned long timer_count[WD_MAX_USR_ACTIONS];
	int action[WD_MAX_USR_ACTIONS];
	int signal;
} wd_param_t;

/* Constants for the action[] fields */
#define WD_ACTION_SIGNAL  1
#define WD_ACTION_KILL    2
#define WD_ACTION_REBOOT  3
#define WD_ACTION_RESET   4

#define WD_IOCTL_BASE	'W'

#define WD_OPEN_ONLY	_IO (WD_IOCTL_BASE, 0)
#define WD_ALWAYS	_IO (WD_IOCTL_BASE, 1)
#define WD_REGISTER	_IOW(WD_IOCTL_BASE, 2, wd_param_t)
#define WD_RESET	_IOW(WD_IOCTL_BASE, 3, int)
#define WD_UNREGISTER	_IOW(WD_IOCTL_BASE, 4, int)

#ifdef __KERNEL__

/*
 * wd_init_t - initialization of watchdog driver and activation timer
 * wd_kick_t - reset watchdog counter
 * wd_delete_t - stop watchdog timer and free resources
 * wd_machine_restart_t - emergency restart
 *
 * NOTE: Functions wd_kick_t and wd_machine_restart_t can be called from
 * interrupt context.
 */
typedef int (wd_init_t)(unsigned long *);
typedef void (wd_kick_t)(void);
typedef void (wd_delete_t)(void);
typedef void (wd_machine_restart_t)(void);

struct wd_hw_functions {
	wd_init_t		*wd_init;
	wd_kick_t		*wd_kick;
	wd_delete_t		*wd_delete;
	wd_machine_restart_t	*wd_machine_restart;
};
extern struct wd_hw_functions wd_hw_functions;

#else /* !__KERNEL__ */

#include <fcntl.h>
#include <unistd.h>
#include <linux/ioctl.h>

#define WD_DEVICE "/dev/watchdog"
#define WD_DEF_SERVICE_TIME 300

int wd_fd;
int wd_value = WD_DEF_SERVICE_TIME;

#define WD_INIT (wd_fd = open(WD_DEVICE, O_RDWR, 0))

#define WD_SET_SERVICE_TIME(sec) wd_value = (sec);

#define WD_SERVICE write(wd_fd, (char *) &wd_value, sizeof (wd_value))

#define WD_CLOSE close(wd_fd)

#endif /* __KERNEL__ */

#endif /* _LINUX_WD_H */
