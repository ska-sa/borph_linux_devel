/****************************************************************************
 *        MB86290 Series Graphics Controller Access Library
 *        ALL RIGHTS RESERVED, COPYRIGHT (C) FUJITSU LIMITED 1999-2003
 *        LICENSED MATERIAL - PROGRAM PROPERTY OF FUJITSU LIMITED
 *        1.01.002
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 ****************************************************************************/
#ifndef _gdctypes_h_
#define _gdctypes_h_

/* GDC format */
typedef int		GDC_BOOL;
typedef unsigned char	GDC_UCHAR;
typedef unsigned char	GDC_COL8;
typedef unsigned short	GDC_COL16;
typedef unsigned long	GDC_COL24;
typedef unsigned long	GDC_COL32;
typedef unsigned long	GDC_BINIMAGE;
typedef long		GDC_FIXED32;
typedef short		GDC_FIXED16;
typedef short		GDC_FIXED16TEX;
typedef float		GDC_SFLOAT;
typedef short		GDC_SHORT;
typedef unsigned short	GDC_USHORT;
typedef long		GDC_LONG;
typedef unsigned long	GDC_ULONG;
typedef unsigned long	GDC_FRAC32;
typedef unsigned long	GDC_COLOR32;

/* For video capture scale, FIXED 5, FLAC 11 */
typedef unsigned short GDC_FIXED_SCALE;

#ifndef NULL
#define NULL	((void *)0)
#endif

/* System information */
#define	GDC_CURSOR_MAX	2

typedef struct GDC_SYSTEMINFO_REC {
	int			fd;
	GDC_ULONG		cputype;	/* CPU type */
	GDC_ULONG		gdctype;	/* GDC type */
	GDC_ULONG		tran_unit;	/* Transfer unit */
	GDC_ULONG		mapsize;	/* VRAM mapping size */
	GDC_ULONG		locate;		/* Register Location */
	GDC_ULONG		gdcbase;	/* GDC mapping address */
	volatile GDC_ULONG	*host_reg;	/* Host interface register */
	volatile GDC_ULONG	*disp_reg;	/* Display register */
	volatile GDC_ULONG	*draw_reg;	/* Drawing register */
	volatile GDC_ULONG	*geo_reg;	/* Geometry register */
	volatile GDC_ULONG	*cap_reg;	/* Capture register */
	volatile GDC_ULONG	*i2c_reg;	/* I2C register */
} GDC_SYSTEMINFO;

typedef struct GdcLayerParamRec {
	GDC_UCHAR	enable;
	GDC_ULONG	cmode;
	GDC_ULONG	ladrs0;
	GDC_ULONG	ladrs1;
	GDC_USHORT	Lw;
	GDC_USHORT	dx;
	GDC_USHORT	dy;
	GDC_USHORT	width;
	GDC_USHORT	height;
	GDC_ULONG	transparency;
	/* for MB86293 later start */
	GDC_ULONG	ext_mode;
	GDC_ULONG	window_pos;
	GDC_ULONG	window_size;
	GDC_ULONG	blend_mode;
	/* for MB86293 later end */
} GdcLayerParam;

typedef struct GDC_DISPPARAM_REC {
	int		initflag;
	GDC_ULONG	mode;
	GDC_ULONG	hdp, hdb;
	GDC_ULONG	vdp;	/* for defalut window height */
	/* for MB86293 before start */
	GDC_ULONG	blend_mode;
	/* for MB86293 before end */
	GdcLayerParam	C;
	GdcLayerParam	W;
	GdcLayerParam	ML;
	GdcLayerParam	MR;
	GdcLayerParam	BL;
	GdcLayerParam	BR;
	GDC_ULONG	chromakey;
	GDC_ULONG	cursor_mode;
	GDC_ULONG	lpCursor[GDC_CURSOR_MAX];
	/* for MB86293 later start */
	GDC_ULONG	mode_ext;
	/* for MB86293 later end */
} GDC_DISPPARAM;

#endif /* _gdctypes_h_ */
