/* arch/powerpc/platforms/8xx/mgsuvd.c
 *
 * Platform setup for the Keymile mgsuvd board
 *
 * Heiko Schocher <hs@denx.de>
 *
 * Copyright 2007 DENX Software Engineering GmbH
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/root_dev.h>

#include <linux/fs_enet_pd.h>
#include <linux/fs_uart_pd.h>
#include <linux/mii.h>

#include <asm/delay.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <asm/time.h>
#include <asm/mpc8xx.h>
#include <asm/8xx_immap.h>
#include <asm/cpm1.h>
#include <asm/fs_pd.h>
#include <asm/prom.h>

#include <linux/fsl_devices.h>
#include <sysdev/fsl_soc.h>
#include <linux/mod_devicetable.h>
#include <asm/of_platform.h>
#include "mpc8xx.h"

extern void cpm_reset(void);
struct cpm_pin {
	int port, pin, flags;
};

static struct cpm_pin mgsuvd_pins[] = {
	/* SMC1 */
	{CPM_PORTB, 24, CPM_PIN_INPUT}, /* RX */
	{CPM_PORTB, 25, CPM_PIN_INPUT | CPM_PIN_SECONDARY}, /* TX */

	/* SCC3 */
	{0, 10, CPM_PIN_INPUT},
	{0, 11, CPM_PIN_INPUT},
	{0, 3, CPM_PIN_INPUT},
	{0, 2, CPM_PIN_INPUT},
	{2, 13, CPM_PIN_INPUT},
};

static void __init init_ioports(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mgsuvd_pins); i++) {
		struct cpm_pin *pin = &mgsuvd_pins[i];
		cpm1_set_pin(pin->port, pin->pin, pin->flags);
	}

	setbits16(&mpc8xx_immr->im_ioport.iop_pcso, 0x300);
	cpm1_clk_setup(CPM_CLK_SCC3, CPM_CLK5, CPM_CLK_RX);
	cpm1_clk_setup(CPM_CLK_SCC3, CPM_CLK5, 0);
	setbits32(&mpc8xx_immr->im_cpm.cp_pbpar, 0x300);
	setbits32(&mpc8xx_immr->im_cpm.cp_pbdir, 0x300);

	cpm1_clk_setup(CPM_CLK_SMC1, CPM_BRG1, CPM_CLK_RTX);
}

static void __init mgsuvd_setup_arch(void)
{
	struct device_node *cpu;

	cpu = of_find_node_by_type(NULL, "cpu");
	if (cpu != 0) {
		const unsigned int *fp;

		fp = of_get_property(cpu, "clock-frequency", NULL);
		if (fp != 0)
			loops_per_jiffy = *fp / HZ;
		else
			loops_per_jiffy = 50000000 / HZ;
		of_node_put(cpu);
	}

	cpm_reset();

	init_ioports();

	ROOT_DEV = Root_NFS;
}

static struct of_device_id __initdata of_bus_ids[] = {
	{ .name = "soc", },
	{ .name = "cpm", },
	{ .name = "localbus", },
	{},
};

static int __init declare_of_platform_devices(void)
{
	if (!machine_is(mgsuvd))
		return 0;

	of_platform_bus_probe(NULL, of_bus_ids, NULL);

	return 0;
}
device_initcall(declare_of_platform_devices);

static int __init mgsuvd_probe(void)
{
	char *model = of_get_flat_dt_prop(of_get_flat_dt_root(),
					  "model", NULL);
	if (model == NULL)
		return 0;
	if (strcmp(model, "MGSUVD"))
		return 0;

	return 1;
}

define_machine(mgsuvd) {
	.name			= "MGSUVD",
	.probe			= mgsuvd_probe,
	.setup_arch		= mgsuvd_setup_arch,
	.init_IRQ		= mpc8xx_pics_init,
	.get_irq		= mpc8xx_get_irq,
	.restart		= mpc8xx_restart,
	.calibrate_decr		= mpc8xx_calibrate_decr,
	.set_rtc_time		= mpc8xx_set_rtc_time,
	.get_rtc_time		= mpc8xx_get_rtc_time,
};
