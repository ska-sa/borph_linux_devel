/*
 *	mb86290fb_disp.c  --  MB86290 Series FrameBuffer Driver
 *
 *	Copyright (C) FUJITSU LIMITED 2003
 *	1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <asm/uaccess.h>

#include "mb86290fbdev.h"

#define REG_DCEM	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_EXT_MODE)
#define REG_DCM		MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_MODE)
#define REG_HDP_HDB	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_H_PERIOD)
#define REG_HDP		MB86290FB_LO_WORD(REG_HDP_HDB)
#define REG_HDB		MB86290FB_HI_WORD(REG_HDP_HDB)
#define REG_VSP_VDP	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_V_PERIOD_POS)
#define REG_VDP		MB86290FB_HI_WORD(REG_VSP_VDP)
#define REG_L0BLD	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE)
#define REG_KEYC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_KEY_COLOR)
#define REG_CUTC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_CURSOR_MODE)
#define REG_L0WX	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_WIN_POS)
#define REG_L0WW	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_WIN_SIZE)
#define REG_L0M		MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_MODE_W_H)
#define REG_L0OA	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_ORG_ADR)
#define REG_L0DX_DY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_C_DISP_POS)
#define REG_L0DX	MB86290FB_LO_WORD(REG_L0DX_DY)
#define REG_L0DY	MB86290FB_HI_WORD(REG_L0DX_DY)
#define REG_L0ETC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_TRANS)
#define REG_L0EM	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L0_EXT_MODE)
#define REG_L1WX_WY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L1_WIN_POS)
#define REG_L1WW_WH	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L1_WIN_SIZE)
#define REG_WX_WY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_WIN_POS)
#define REG_WW_WH	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_WIN_SIZE)
#define REG_L1M		MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_W_WIDTH)
#define REG_L1DA	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_W_ORG_ADR)
#define REG_L1ETC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L1_TRANS)
#define REG_L1EM	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L1_EXT_MODE)
#define REG_L1BLD	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L1)
#define REG_L2WX_WY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L2_WIN_POS)
#define REG_L2WW_WH	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L2_WIN_SIZE)
#define REG_L2M		MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_ML_MODE_W_H)
#define REG_L2OA0	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_ML_ORG_ADR1)
#define REG_L2OA1	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_ML_ORG_ADR2)
#define REG_L2DX_DY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_ML_DISP_POS)
#define REG_L2DX	MB86290FB_LO_WORD(REG_L2DX_DY)
#define REG_L2DY	MB86290FB_HI_WORD(REG_L2DX_DY)
#define REG_L2ETC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L2_TRANS)
#define REG_L2EM	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L2_EXT_MODE)
#define REG_L2BLD	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L2)
#define REG_L3WX_WY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L3_WIN_POS)
#define REG_L3WW_WH	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L3_WIN_SIZE)
#define REG_L3M		MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_MR_MODE_W_H)
#define REG_L3OA0	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_MR_ORG_ADR1)
#define REG_L3OA1	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_MR_ORG_ADR2)
#define REG_L3DX_DY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_MR_DISP_POS)
#define REG_L3DX	MB86290FB_LO_WORD(REG_L3DX_DY)
#define REG_L3DY	MB86290FB_HI_WORD(REG_L3DX_DY)
#define REG_L3ETC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L3_TRANS)
#define REG_L3EM	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L3_EXT_MODE)
#define REG_L3BLD	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L3)
#define REG_L4WX_WY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L4_WIN_POS)
#define REG_L4WW_WH	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L4_WIN_SIZE)
#define REG_L4M		MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BL_MODE_W_H)
#define REG_L4OA0	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BL_ORG_ADR1)
#define REG_L4OA1	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BL_ORG_ADR2)
#define REG_L4DX_DY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BL_DISP_POS)
#define REG_L4DX	MB86290FB_LO_WORD(REG_L4DX_DY)
#define REG_L4DY	MB86290FB_HI_WORD(REG_L4DX_DY)
#define REG_L4ETC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L4_TRANS)
#define REG_L4EM	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L4_EXT_MODE)
#define REG_L4BLD	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L4)
#define REG_L5WX_WY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L5_WIN_POS)
#define REG_L5WW_WH	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L5_WIN_SIZE)
#define REG_L5M		MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BR_MODE_W_H)
#define REG_L5OA0	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BR_ORG_ADR1)
#define REG_L5OA1	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BR_ORG_ADR2)
#define REG_L5DX_DY	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BR_DISP_POS)
#define REG_L5DX	MB86290FB_LO_WORD(REG_L5DX_DY)
#define REG_L5DY	MB86290FB_HI_WORD(REG_L5DX_DY)
#define REG_L5ETC	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L5_TRANS)
#define REG_L5EM	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_L5_EXT_MODE)
#define REG_L5BLD	MB86290FB_READ_DISP_REGISTER(GDC_DISP_REG_BLEND_MODE_L5)

int mb86290fb_ioctl_getdispinfo(mb86290fb_drawman_t* drawman, unsigned long arg)
{
	GDC_ULONG *parg;
	GDC_ULONG *pdata;
	GDC_DISPPARAM GdcDispParameter;
	GDC_ULONG hdp;
	GDC_ULONG vdp;
	GDC_ULONG hdb;

	parg = (GDC_ULONG*)arg;
	get_user(pdata, (GDC_ULONG**)parg++);

	/* COMMON */
	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		GdcDispParameter.mode_ext = REG_DCEM;
	} else {
		GdcDispParameter.mode = REG_DCM;
	}
	GdcDispParameter.hdp = hdp	= REG_HDP;
	GdcDispParameter.vdp = vdp	= REG_VDP;
	GdcDispParameter.hdb = hdb	= REG_HDB;
	GdcDispParameter.blend_mode	= REG_L0BLD;
	GdcDispParameter.chromakey	= REG_KEYC;
	GdcDispParameter.cursor_mode	= REG_CUTC;

	/* C Layer */
	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		GdcDispParameter.C.enable = (REG_DCEM & GDC_DISP_L0_ENABLE_MASK) >>
					    GDC_DISP_L0_ENABLE_SHIFT;
		GdcDispParameter.C.window_pos = REG_L0WX;
		GdcDispParameter.C.window_size = REG_L0WW;
	} else {
		GdcDispParameter.C.enable = (REG_DCM & GDC_DISP_C_ENABLE_MASK) >>
					    GDC_DISP_C_ENABLE_SHIFT;
		GdcDispParameter.C.window_pos = 0;
		GdcDispParameter.C.window_size = MB86290FB_XY_PACK(vdp, hdp);
	}
	GdcDispParameter.C.cmode	= REG_L0M;
	GdcDispParameter.C.ladrs0	= REG_L0OA;
	GdcDispParameter.C.ladrs1	= 0;
	GdcDispParameter.C.width = GdcDispParameter.C.Lw =
		((GdcDispParameter.C.cmode >> GDC_DISP_WIDTH_SHIFT) & GDC_DISP_WIDTH_MASK) << 6;
	GdcDispParameter.C.height=
		(GdcDispParameter.C.cmode & GDC_DISP_HEIGHT_MASK) + 1;
	GdcDispParameter.C.dx		= REG_L0DX;
	GdcDispParameter.C.dy		= REG_L0DY;
	GdcDispParameter.C.transparency = REG_L0ETC;
	GdcDispParameter.C.ext_mode	= REG_L0EM;
	GdcDispParameter.C.blend_mode	= REG_L0BLD;

	/* W Layer */
	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		GdcDispParameter.W.enable=
			(REG_DCEM & GDC_DISP_L1_ENABLE_MASK) >> GDC_DISP_L1_ENABLE_SHIFT;
		GdcDispParameter.W.window_pos	= REG_L1WX_WY;
		GdcDispParameter.W.window_size	= REG_L1WW_WH;
	} else {
		GdcDispParameter.W.enable =
			(REG_DCM & GDC_DISP_W_ENABLE_MASK) >> GDC_DISP_W_ENABLE_SHIFT;
		GdcDispParameter.W.window_pos	= REG_WX_WY;
		GdcDispParameter.W.window_size	= REG_WW_WH;
	}
	GdcDispParameter.W.cmode  = REG_L1M; /* ==GDC_DISP_REG_W_MODE_W_H */
	GdcDispParameter.W.ladrs0 = REG_L1DA;
	GdcDispParameter.W.ladrs1 = 0;
	GdcDispParameter.W.width = GdcDispParameter.W.Lw =
		((GdcDispParameter.W.cmode>>GDC_DISP_WIDTH_SHIFT) & GDC_DISP_WIDTH_MASK) << 6;
	GdcDispParameter.W.height = 0;
	GdcDispParameter.W.dx = 0;	/* Position Parameter was not supported */
	GdcDispParameter.W.dy = 0;	/* Position Parameter was not supported */
	GdcDispParameter.W.transparency = REG_L1ETC;
	GdcDispParameter.W.ext_mode	= REG_L1EM;
	GdcDispParameter.W.blend_mode = REG_L1BLD;

	/* ML Layer */
	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		GdcDispParameter.ML.enable =
			(REG_DCEM & GDC_DISP_L2_ENABLE_MASK) >> GDC_DISP_L2_ENABLE_SHIFT;
		GdcDispParameter.ML.window_pos	= REG_L2WX_WY;
		GdcDispParameter.ML.window_size	= REG_L2WW_WH;
	} else {
		GdcDispParameter.ML.enable =
			(REG_DCM & GDC_DISP_M_ENABLE_MASK) >> GDC_DISP_M_ENABLE_SHIFT;
		GdcDispParameter.ML.window_pos	= 0;
		GdcDispParameter.ML.window_size	= MB86290FB_XY_PACK(vdp, hdp);
	}
	GdcDispParameter.ML.cmode	= REG_L2M;
	GdcDispParameter.ML.ladrs0	= REG_L2OA0;
	GdcDispParameter.ML.ladrs1	= REG_L2OA1;
	GdcDispParameter.ML.width = GdcDispParameter.ML.Lw =
		((GdcDispParameter.ML.cmode >> GDC_DISP_WIDTH_SHIFT) & GDC_DISP_WIDTH_MASK) << 6;
	GdcDispParameter.ML.height = (GdcDispParameter.ML.cmode & GDC_DISP_HEIGHT_MASK) + 1;
	GdcDispParameter.ML.dx = REG_L2DX;
	GdcDispParameter.ML.dy = REG_L2DY;
	GdcDispParameter.ML.transparency = REG_L2ETC;
	GdcDispParameter.ML.ext_mode = REG_L2EM;
	GdcDispParameter.ML.blend_mode = REG_L2BLD;

	/* MR Layer */
	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		GdcDispParameter.MR.enable =
			(REG_DCEM & GDC_DISP_L3_ENABLE_MASK) >> GDC_DISP_L3_ENABLE_SHIFT;
		GdcDispParameter.MR.window_pos	= REG_L3WX_WY;
		GdcDispParameter.MR.window_size	= REG_L3WW_WH;
	} else {
		GdcDispParameter.MR.enable =
			(REG_DCM & GDC_DISP_M_ENABLE_MASK) >> GDC_DISP_M_ENABLE_SHIFT;
		GdcDispParameter.MR.window_pos	= MB86290FB_XY_PACK(0, hdb-1);
		GdcDispParameter.MR.window_size	= MB86290FB_XY_PACK(vdp, vdp-hdb);
	}
	GdcDispParameter.MR.cmode	= REG_L3M;
	GdcDispParameter.MR.ladrs0	= REG_L3OA0;
	GdcDispParameter.MR.ladrs1	= REG_L3OA1;
	GdcDispParameter.MR.width = GdcDispParameter.MR.Lw =
		((GdcDispParameter.MR.cmode >> GDC_DISP_WIDTH_SHIFT) & GDC_DISP_WIDTH_MASK) << 6;
	GdcDispParameter.MR.height = (GdcDispParameter.MR.cmode & GDC_DISP_HEIGHT_MASK) + 1;
	GdcDispParameter.MR.dx = REG_L3DX;
	GdcDispParameter.MR.dy = REG_L3DY;
	GdcDispParameter.MR.transparency= REG_L3ETC;
	GdcDispParameter.MR.ext_mode = REG_L3EM;
	GdcDispParameter.MR.blend_mode = REG_L3BLD;

	/* BL Layer */
	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
		GdcDispParameter.BL.enable =
			(REG_DCEM & GDC_DISP_L4_ENABLE_MASK) >> GDC_DISP_L4_ENABLE_SHIFT;
		GdcDispParameter.BL.window_pos	= REG_L4WX_WY;
		GdcDispParameter.BL.window_size	= REG_L4WW_WH;
	} else {
		GdcDispParameter.BL.enable =
			(REG_DCM & GDC_DISP_B_ENABLE_MASK) >> GDC_DISP_B_ENABLE_SHIFT;
		GdcDispParameter.BL.window_pos	= 0;
		GdcDispParameter.BL.window_size	= MB86290FB_XY_PACK(vdp, hdb);
	}
	GdcDispParameter.BL.cmode	= REG_L4M;
	GdcDispParameter.BL.ladrs0	= REG_L4OA0;
	GdcDispParameter.BL.ladrs1	= REG_L4OA1;
	GdcDispParameter.BL.width = GdcDispParameter.BL.Lw =
		((GdcDispParameter.BL.cmode >> GDC_DISP_WIDTH_SHIFT) & GDC_DISP_WIDTH_MASK) << 6;
	GdcDispParameter.BL.height = (GdcDispParameter.BL.cmode & GDC_DISP_HEIGHT_MASK) + 1;
	GdcDispParameter.BL.dx = REG_L4DX;
	GdcDispParameter.BL.dy = REG_L4DY;
	GdcDispParameter.BL.transparency= REG_L4ETC;
	GdcDispParameter.BL.ext_mode = REG_L4EM;
	GdcDispParameter.BL.blend_mode = REG_L4BLD;

	/* BR Layer */
	if (GdcSystemInfo.gdctype >= GDC_TYPE_MB86293) {
	GdcDispParameter.BR.enable =
		(REG_DCEM & GDC_DISP_L5_ENABLE_MASK) >> GDC_DISP_L5_ENABLE_SHIFT;
		GdcDispParameter.BR.window_pos	= REG_L5WX_WY;
		GdcDispParameter.BR.window_size	= REG_L5WW_WH;
	} else {
		GdcDispParameter.BR.enable =
			(REG_DCM & GDC_DISP_B_ENABLE_MASK) >> GDC_DISP_B_ENABLE_SHIFT;
		GdcDispParameter.BR.window_pos	= MB86290FB_XY_PACK(0, hdb);
		GdcDispParameter.BR.window_size	= MB86290FB_XY_PACK(vdp, hdp-hdb);
	}
	GdcDispParameter.BR.cmode	= REG_L5M;
	GdcDispParameter.BR.ladrs0	= REG_L5OA0;
	GdcDispParameter.BR.ladrs1	= REG_L5OA1;
	GdcDispParameter.BR.width = GdcDispParameter.BR.Lw =
		((GdcDispParameter.BR.cmode >> GDC_DISP_WIDTH_SHIFT) & GDC_DISP_WIDTH_MASK) << 6;
	GdcDispParameter.BR.height = (GdcDispParameter.BR.cmode & GDC_DISP_HEIGHT_MASK) + 1;
	GdcDispParameter.BR.dx = REG_L5DX;
	GdcDispParameter.BR.dy = REG_L5DY;
	GdcDispParameter.BR.transparency= REG_L5ETC;
	GdcDispParameter.BR.ext_mode = REG_L5EM;
	GdcDispParameter.BR.blend_mode = REG_L5BLD;

	if (!access_ok(VERIFY_WRITE, (void*)pdata, sizeof(GDC_DISPPARAM))) {
		return -EFAULT;
	}
	copy_to_user(pdata, &GdcDispParameter, sizeof(GDC_DISPPARAM));

	return 0;
}

int mb86290fb_ioctl_setframesize(GDC_DISPPARAM *dispparam, unsigned long arg)
{
	unsigned long *parg;
	unsigned long layer;
	unsigned long x, y;

	parg = (GDC_ULONG*)arg;
	get_user(layer, parg);
	get_user(x, parg+1);
	get_user(y, parg+2);

	switch (layer) {
	case GDC_DISP_LAYER_C:
	case GDC_DISP_LAYER_L0:
		dispparam->C.width  = x;
		dispparam->C.height = y;
		break;
	case GDC_DISP_LAYER_W:
	case GDC_DISP_LAYER_L1:
		dispparam->W.width  = x;
		dispparam->W.height = y;
		break;
	case GDC_DISP_LAYER_ML:
	case GDC_DISP_LAYER_L2:
		dispparam->ML.width  = x;
		dispparam->ML.height = y;
		break;
	case GDC_DISP_LAYER_MR:
	case GDC_DISP_LAYER_L3:
		dispparam->MR.width  = x;
		dispparam->MR.height = y;
		break;
	case GDC_DISP_LAYER_BL:
	case GDC_DISP_LAYER_L4:
		dispparam->BL.width  = x;
		dispparam->BL.height = y;
		break;
	case GDC_DISP_LAYER_BR:
	case GDC_DISP_LAYER_L5:
		dispparam->BR.width  = x;
		dispparam->BR.height = y;
		break;
	}

    return 0;
}

int mb86290fb_ioctl_dispflipautof(mb86290fb_eventinfo_t *event,
				  unsigned long arg)
{
	unsigned long *parg;
	unsigned long layer;
	unsigned long bank;
	unsigned long condition;
	int n;

	parg = (unsigned long*)arg;
	get_user(layer, parg);
	get_user(bank, parg+1);
	get_user(condition, parg+2);

	n = layer - GDC_DISP_LAYER_ML;
	event->flip[n].bank = bank;
	event->flip[n].condition = condition;
	return 0;
}

void mb86290fb_initscrollinfo(mb86290fb_scrollinfo_t *scroll)
{
	int i;

	atomic_set(&scroll->C.waitcnt, 0);
	init_waitqueue_head(&scroll->C.waitq);
	scroll->C.rp = &scroll->C.param[0];
	scroll->C.wp = &scroll->C.param[1];
	scroll->C.param[0].ready = 0;
	scroll->C.param[1].ready = 0;

	atomic_set(&scroll->W.waitcnt, 0);
	init_waitqueue_head(&scroll->W.waitq);
	scroll->W.rp = &scroll->W.param[0];
	scroll->W.wp = &scroll->W.param[1];
	scroll->W.param[0].ready = 0;
	scroll->W.param[1].ready = 0;

	for (i=0;i<4;i++) {
		atomic_set(&scroll->MB[i].waitcnt, 0);
		init_waitqueue_head(&scroll->MB[i].waitq);
		scroll->MB[i].rp = &scroll->MB[i].param[0];
		scroll->MB[i].wp = &scroll->MB[i].param[1];
		scroll->MB[i].param[0].ready = 0;
		scroll->MB[i].param[1].ready = 0;
	}
}

int mb86290fb_ioctl_disppossync(mb86290fb_scrollinfo_t *scroll,
				unsigned long arg)
{
	unsigned long		*parg;
	unsigned long		layer, disp0, disp1, xy, complete;
	mb86290fb_scrollparamC_t	*wpC;
	mb86290fb_scrollparamW_t	*wpW;
	mb86290fb_scrollparamMB_t	*wpMB;
	mb86290fb_scrollinfoMB_t	*mb;
	int				idx;

	parg = (unsigned long*)arg;
	get_user(layer, parg);
	get_user(disp0, parg+1);
	get_user(disp1, parg+2);
	get_user(xy, parg+3);
	get_user(complete, parg+4);

	switch (layer) {
	case GDC_DISP_LAYER_C:
		#ifdef SCROLL_IN_ORDER
		if (atomic_read(&scroll->C.waitcnt)) {
			interruptible_sleep_on(&scroll->C.waitq);
		}
		#endif

		wpC = scroll->C.wp;

		wpC->disp_addr = disp0;
		wpC->xy = xy;

		/* Ready */
		wpC->ready = 1;

		/* Swap R/W pointers */
		scroll->C.wp = scroll->C.rp;
		scroll->C.rp = wpC;

		if (complete==GDC_WAIT_COMPLETE) {
			atomic_inc(&scroll->C.waitcnt);
			wait_event_interruptible(scroll->C.waitq, wpC->ready==0);
			atomic_dec(&scroll->C.waitcnt);
		}
		break;

	case GDC_DISP_LAYER_W:
		#ifdef SCROLL_IN_ORDER
		if (atomic_read(&scroll->W.waitcnt)) {
			interruptible_sleep_on(&scroll->W.waitq);
		}
		#endif
		wpW = scroll->W.wp;

		wpW->disp_addr = disp0;

		/* Ready */
		wpW->ready = 1;

		/* Swap R/W pointers */
		scroll->W.wp = scroll->W.rp;
		scroll->W.rp = wpW;

		if (complete==GDC_WAIT_COMPLETE) {
			atomic_inc(&scroll->W.waitcnt);
			wait_event_interruptible(scroll->W.waitq, wpW->ready==0);
			atomic_dec(&scroll->W.waitcnt);
		}
		break;

	case GDC_DISP_LAYER_ML:
	case GDC_DISP_LAYER_MR:
	case GDC_DISP_LAYER_BL:
	case GDC_DISP_LAYER_BR:
		idx = layer - GDC_DISP_LAYER_ML;
		mb = &scroll->MB[idx];

		#ifdef SCROLL_IN_ORDER
		if (atomic_read(&mb->waitcnt)) {
			interruptible_sleep_on(&mb->waitq);
		}
		#endif
		wpMB = mb->wp;

		wpMB->disp_addr0 = disp0;
		wpMB->disp_addr1 = disp1;
		wpMB->xy = xy;

		/* Ready */
		wpMB->ready = 1;

		/* Swap R/W pointers */
		mb->wp = mb->rp;
		mb->rp = wpMB;

		if (complete==GDC_WAIT_COMPLETE) {
			atomic_inc(&mb->waitcnt);
			wait_event_interruptible(mb->waitq, wpMB->ready==0);
			atomic_dec(&mb->waitcnt);
		}
		break;
	}

	return 0;
}
