/*
 *  linux/include/asm-ppc/nmi.h
 */
#ifndef ASM_NMI_H
#define ASM_NMI_H

#ifdef BOARD_WATCHDOG_FUNC
#define touch_nmi_watchdog	BOARD_WATCHDOG_FUNC
#else
static inline void touch_nmi_watchdog(void)
{
	touch_softlockup_watchdog();
}
#endif

#endif /* ASM_NMI_H */
