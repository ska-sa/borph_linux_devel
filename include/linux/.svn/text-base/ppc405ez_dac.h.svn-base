/*******************************************************************************
 * ppc405ez_dac.h
 *
 * Header for the PPC405EZ DAC device driver.
 *
 * Author: Victor Gallardo <vgallardo@amcc.com>
 * Date: November 2006
 *
 * Copyright 2006 Applied Micro Circuits Corporation
 *
 * Copyright 2007 DENX Software Engineering - Stefan Roese <sr@denx.de>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR   IMPLIED
 * WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 * USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************/

#ifndef PPC405EZ_DAC_H
#define PPC405EZ_DAC_H

#include <linux/ioctl.h>

/*******************************************************************************
 * IOCTL info for mknod
 ******************************************************************************/

#define PPC405EZ_DAC_DEV_FILENAME	"/dev/ppc405ez_dac"
#define PPC405EZ_DAC_MAJOR		241
#define PPC405EZ_DAC_MINOR		0

/*******************************************************************************
 * IOCTL supported commands
 ******************************************************************************/

#define PPC405EZ_DAC_WRITE_REG	_IO(PPC405EZ_DAC_MAJOR, 1)
#define PPC405EZ_DAC_PUT	_IO(PPC405EZ_DAC_MAJOR, 2)

/* defines */
#define PPC405EZ_DAC_BUF_0	0x00
#define PPC405EZ_DAC_BUF_1	0x04
#define PPC405EZ_DAC_BUF_2	0x08
#define PPC405EZ_DAC_BUF_3	0x0C
#define PPC405EZ_DAC_BUF_4	0x10
#define PPC405EZ_DAC_BUF_5	0x14
#define PPC405EZ_DAC_BUF_6	0x18
#define PPC405EZ_DAC_BUF_7	0x1C
#define PPC405EZ_DAC_BUF_8	0x20
#define PPC405EZ_DAC_BUF_9	0x24
#define PPC405EZ_DAC_BUF_10	0x28
#define PPC405EZ_DAC_BUF_11	0x2C
#define PPC405EZ_DAC_BUF_12	0x30
#define PPC405EZ_DAC_BUF_13	0x34
#define PPC405EZ_DAC_BUF_14	0x38
#define PPC405EZ_DAC_BUF_15	0x3C
#define PPC405EZ_DAC_PTR_INT_EN	0x90
#define PPC405EZ_DAC_INT_EN	0x94
#define PPC405EZ_DAC_PTR_INT_STAT 0x98
#define PPC405EZ_DAC_INT_STAT	0x9C
#define PPC405EZ_DAC_CFG	0xA0
#define PPC405EZ_DAC_PTR_CFG	0xA4
#define PPC405EZ_DAC_PTR_STAT	0xC0
#define PPC405EZ_DAC_DMAREQ_EN	0xC4

/*******************************************************************************
 * Data Structure Types
 ******************************************************************************/

union ppc405ez_dac_conv {
	unsigned int buf[16];
	unsigned short val[32];
};

struct ppc405ez_dac_ptr_cfg_reg {
	unsigned int rollover:5, ptr_set:5, ptr_set_en:1, reserved:21;
};

union ppc405ez_dac_ptr_cfg {
	unsigned int data;
	struct ppc405ez_dac_ptr_cfg_reg reg;
};

struct ppc405ez_dac_cfg_reg {
	unsigned int trig_src:2,
	    filter_cnt:4,
	    dmareq_en:1,
	    reserved1:1,
	    dmasize:1,
	    reserved2:1,
	    trig:1, pinsen:2, pwmsen:2, reserved3:1, trig_cnt_max:16;
};

union ppc405ez_dac_cfg {
	unsigned int data;
	struct ppc405ez_dac_cfg_reg reg;
};

struct ppc405ez_dac_ptr_stat_reg {
	unsigned int ptr_val:5, reserved:27;
};

union ppc405ez_dac_ptr_stat {
	unsigned int data;
	struct ppc405ez_dac_ptr_stat_reg reg;
};

struct dac_reg {
	unsigned int offs;
	union {
		unsigned int val;
		struct ppc405ez_dac_ptr_cfg_reg ptr_cfg_reg;
		struct ppc405ez_dac_cfg_reg cfg_reg;
		struct ppc405ez_dac_ptr_stat_reg ptr_stat_reg;
	} data;
};

#endif /* PPC405EZ_DAC_IOCTL_H */
