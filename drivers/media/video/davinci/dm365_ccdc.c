/*
 * Copyright (C) 2008-2009 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This is the isif hardware module for DM365.
 * TODO: 1) Raw bayer parameter settings and bayer capture
 *	 2) Add support for control ioctl
 *	 3) Add support for linearization
 *	 4) Test VPFE_CMD_S_CCDC_RAW_RARAMS ioctl
 */
#include <linux/delay.h>
#include <mach/mux.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/videodev2.h>
#include <media/davinci/dm365_ccdc.h>
#include <media/davinci/vpss.h>
#include "dm365_ccdc_regs.h"
#include "ccdc_hw_device.h"

static struct device *dev;

/* holds live module configuation paramaters */
static struct ccdc_params_raw ccdc_hw_params_bayer = {
	.pix_fmt = CCDC_PIXFMT_RAW,
	.frm_fmt = CCDC_FRMFMT_PROGRESSIVE,
	.win = CCDC_WIN_VGA,
	.fid_pol = VPFE_PINPOL_POSITIVE,
	.vd_pol = VPFE_PINPOL_POSITIVE,
	.hd_pol = VPFE_PINPOL_POSITIVE,
	.gain = {
		.r_ye = {1, 0},
		.gr_cy = {1, 0},
		.gb_g = {1, 0},
		.b_mg = {1, 0},
	},
	.cfa_pat = CCDC_CFA_PAT_MOSAIC,
	.data_msb = CCDC_BIT_MSB_11,
	.config_params = {
		.data_size = CCDC_12_BITS,
		.data_shift = CCDC_NO_SHIFT,
		.col_pat_field0 = {
			.olop = CCDC_GREEN_BLUE,
			.olep = CCDC_BLUE,
			.elop = CCDC_RED,
			.elep = CCDC_GREEN_RED
		},
		.col_pat_field1 = {
			.olop = CCDC_GREEN_BLUE,
			.olep = CCDC_BLUE,
			.elop = CCDC_RED,
			.elep = CCDC_GREEN_RED
		},
		.test_pat_gen = 0
	}
};

static struct ccdc_ycbcr_config ccdc_hw_params_ycbcr = {
	.pix_fmt = CCDC_PIXFMT_YCBCR_8BIT,
	.frm_fmt = CCDC_FRMFMT_INTERLACED,
	.win = CCDC_WIN_NTSC,
	.fid_pol = VPFE_PINPOL_POSITIVE,
	.vd_pol = VPFE_PINPOL_POSITIVE,
	.hd_pol = VPFE_PINPOL_POSITIVE,
	.pix_order = CCDC_PIXORDER_CBYCRY,
	.buf_type = CCDC_BUFTYPE_FLD_INTERLEAVED
};

static enum ccdc_data_pack data_pack = CCDC_DATA_PACK8;

/* Defauts for module configuation paramaters */
static struct ccdc_config_params_raw ccdc_config_defaults = {
	.linearize = {
		.en = 0,
		.corr_shft = CCDC_NO_SHIFT,
		.scale_fact = {1, 0},
	},
	.df_csc = {
		.df_or_csc = 0,
		.csc = {
			.en = 0
		},
	},
	.dfc = {
		.en = 0
	},
	.bclamp = {
		.en = 0
	},
	.gain_offset = {
		.gain = {
			.r_ye = {1, 0},
			.gr_cy = {1, 0},
			.gb_g = {1, 0},
			.b_mg = {1, 0},
		},
	},
	.culling = {
		.hcpat_odd = 0xff,
		.hcpat_even = 0xff,
		.vcpat = 0xff
	},
	.compress = {
		.alg = CCDC_ALAW,
	},
};


static enum vpfe_hw_if_type ccdc_if_type;
static void *__iomem isif_base_addr;
static int isif_addr_size;

/* Raw Bayer formats */
static u32 ccdc_raw_bayer_pix_formats[] =
		{V4L2_PIX_FMT_SBGGR8, V4L2_PIX_FMT_SBGGR16};

/* Raw YUV formats */
static u32 ccdc_raw_yuv_pix_formats[] =
		{V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_YUYV};

/* register access routines */
static inline u32 regr(u32 offset)
{
	return __raw_readl(isif_base_addr + offset);
}

static inline void regw(u32 val, u32 offset)
{
	__raw_writel(val, isif_base_addr + offset);
}

static inline u32 ccdc_merge(u32 mask, u32 val, u32 offset)
{
	u32 new_val = (regr(offset) & ~mask) | (val & mask);

	regw(new_val, offset);
	return new_val;
}

static void ccdc_disable_all_modules(void)
{
	/* disable BC */
	regw(0, CLAMPCFG);
	/* disable vdfc */
	regw(0, DFCCTL);
	/* disable CSC */
	regw(0, CSCCTL);
	/* disable linearization */
	regw(0, LINCFG0);
	/* disable other modules here as they are supported */
}

static void ccdc_enable(int en)
{
	if (!en) {
		/* Before disable isif, disable all ISIF modules */
		ccdc_disable_all_modules();
		/* wait for next VD. Assume lowest scan rate is 12 Hz. So
		 * 100 msec delay is good enough
		 */
	}
	msleep(100);
	ccdc_merge(CCDC_SYNCEN_VDHDEN_MASK, en, SYNCEN);
}

static void ccdc_set_ccdc_base(void *addr, int size)
{
	isif_base_addr = addr;
	isif_addr_size = size;
}

static void ccdc_enable_output_to_sdram(int en)
{
	ccdc_merge(CCDC_SYNCEN_WEN_MASK, en << CCDC_SYNCEN_WEN_SHIFT, SYNCEN);
}

static void ccdc_config_culling(struct ccdc_cul *cul)
{
	u32 val;

	/* Horizontal pattern */
	val = (cul->hcpat_even) << CULL_PAT_EVEN_LINE_SHIFT;
	val |= cul->hcpat_odd;
	regw(val, CULH);

	/* vertical pattern */
	regw(cul->vcpat, CULV);

	/* LPF */
	ccdc_merge((CCDC_LPF_MASK << CCDC_LPF_SHIFT),
		   (cul->en_lpf << CCDC_LPF_SHIFT), MODESET);
}

static void ccdc_config_gain_offset(void)
{
	struct ccdc_gain_offsets_adj *gain_off_ptr =
		&ccdc_hw_params_bayer.config_params.gain_offset;
	u32 val;

	val = ((gain_off_ptr->gain_sdram_en & 1) << GAIN_SDRAM_EN_SHIFT) |
	((gain_off_ptr->gain_ipipe_en & 1) << GAIN_IPIPE_EN_SHIFT) |
	((gain_off_ptr->gain_h3a_en & 1) << GAIN_H3A_EN_SHIFT) |
	((gain_off_ptr->offset_sdram_en & 1) << OFST_SDRAM_EN_SHIFT) |
	((gain_off_ptr->offset_ipipe_en & 1) << OFST_IPIPE_EN_SHIFT) |
	((gain_off_ptr->offset_h3a_en & 1) << OFST_H3A_EN_SHIFT);

	ccdc_merge(GAIN_OFFSET_EN_MASK, val, CGAMMAWD);

	val = ((gain_off_ptr->gain.r_ye.integer	& GAIN_INTEGER_MASK)
		<< GAIN_INTEGER_SHIFT);
	val |= (ccdc_hw_params_bayer.
		config_params.gain_offset.gain.r_ye.decimal &
		GAIN_DECIMAL_MASK);
	regw(val, CRGAIN);

	val = ((gain_off_ptr->gain.gr_cy
		.integer & GAIN_INTEGER_MASK) << GAIN_INTEGER_SHIFT);
	val |= (gain_off_ptr->gain.gr_cy
		.decimal & GAIN_DECIMAL_MASK);
	regw(val, CGRGAIN);

	val = ((gain_off_ptr->gain.gb_g
		.integer & GAIN_INTEGER_MASK) << GAIN_INTEGER_SHIFT);
	val |= (gain_off_ptr->gain.gb_g
		.decimal & GAIN_DECIMAL_MASK);
	regw(val, CGBGAIN);

	val = ((gain_off_ptr->gain.b_mg
		.integer & GAIN_INTEGER_MASK) << GAIN_INTEGER_SHIFT);
	val |= (gain_off_ptr->gain.b_mg
		.decimal & GAIN_DECIMAL_MASK);
	regw(val, CBGAIN);

	regw((gain_off_ptr->offset &
		OFFSET_MASK), COFSTA);
}

static void ccdc_restore_defaults(void)
{
	enum vpss_ccdc_source_sel source = VPSS_CCDCIN;
	int i;

	memcpy(&ccdc_hw_params_bayer.config_params, &ccdc_config_defaults,
		sizeof(struct ccdc_config_params_raw));

	/* disable CCDC */
	dev_dbg(dev, "\nstarting ccdc_restore_defaults...");
	/* Enable clock to ISIF, IPIPEIF and BL */
	vpss_enable_clock(VPSS_CCDC_CLOCK, 1);
	vpss_enable_clock(VPSS_IPIPEIF_CLOCK, 1);
	vpss_enable_clock(VPSS_BL_CLOCK, 1);

	/* set all registers to default value */
	for (i = 0; i <= 0x1f8; i += 4)
		regw(0, i);

	/* no culling support */
	regw(0xffff, CULH);
	regw(0xff, CULV);

	/* Set default offset and gain */
	ccdc_config_gain_offset();

	vpss_select_ccdc_source(source);

	dev_dbg(dev, "\nEnd of ccdc_restore_defaults...");
}

static int ccdc_open(struct device *device)
{
	dev = device;
	ccdc_restore_defaults();

	return 0;
}

/* This function will configure the window size to be capture in CCDC reg */
static void ccdc_setwin(struct v4l2_rect *image_win,
			enum ccdc_frmfmt frm_fmt, int ppc)
{
	int horz_start, horz_nr_pixels;
	int vert_start, vert_nr_lines;
	int mid_img = 0;

	dev_dbg(dev, "\nStarting ccdc_setwin...");
	/**
	 * ppc - per pixel count. indicates how many pixels per cell
	 * output to SDRAM. example, for ycbcr, it is one y and one c, so 2.
	 * raw capture this is 1
	 */
	horz_start = image_win->left << (ppc - 1);
	horz_nr_pixels = ((image_win->width) << (ppc - 1)) - 1;

	/* Writing the horizontal info into the registers */
	regw(horz_start & START_PX_HOR_MASK, SPH);
	regw(horz_nr_pixels & NUM_PX_HOR_MASK, LNH);
	vert_start = image_win->top;

	if (frm_fmt == CCDC_FRMFMT_INTERLACED) {
		vert_nr_lines = (image_win->height >> 1) - 1;
		vert_start >>= 1;
		/* To account for VD since line 0 doesn't have any data */
		vert_start += 1;
	} else {
		/* To account for VD since line 0 doesn't have any data */
		vert_start += 1;
		vert_nr_lines = image_win->height - 1;
		/* configure VDINT0 and VDINT1 */
		mid_img = vert_start + (image_win->height / 2);
		regw(mid_img, VDINT1);
	}

	regw(0, VDINT0);
	regw(vert_start & START_VER_ONE_MASK, SLV0);
	regw(vert_start & START_VER_TWO_MASK, SLV1);
	regw(vert_nr_lines & NUM_LINES_VER, LNV);
}

static void ccdc_config_bclamp(struct ccdc_black_clamp *bc)
{
	u32 val;

	/**
	 * DC Offset is always added to image data irrespective of bc enable
	 * status
	 */
	val = bc->dc_offset & CCDC_BC_DCOFFSET_MASK;
	regw(val, CLDCOFST);

	if (bc->en) {
		val = (bc->bc_mode_color & CCDC_BC_MODE_COLOR_MASK) <<
			CCDC_BC_MODE_COLOR_SHIFT;

		/* Enable BC and horizontal clamp caculation paramaters */
		val = val | 1 | ((bc->horz.mode & CCDC_HORZ_BC_MODE_MASK) <<
		CCDC_HORZ_BC_MODE_SHIFT);

		regw(val, CLAMPCFG);

		if (bc->horz.mode != CCDC_HORZ_BC_DISABLE) {
			/**
			 * Window count for calculation
			 * Base window selection
			 * pixel limit
			 * Horizontal size of window
			 * vertical size of the window
			 * Horizontal start position of the window
			 * Vertical start position of the window
			 */
			val = (bc->horz.win_count_calc &
				CCDC_HORZ_BC_WIN_COUNT_MASK) |
				((bc->horz.base_win_sel_calc & 1)
				<< CCDC_HORZ_BC_WIN_SEL_SHIFT) |
				((bc->horz.clamp_pix_limit & 1)
				<< CCDC_HORZ_BC_PIX_LIMIT_SHIFT) |
				((bc->horz.win_h_sz_calc &
				CCDC_HORZ_BC_WIN_H_SIZE_MASK)
				<< CCDC_HORZ_BC_WIN_H_SIZE_SHIFT) |
				((bc->horz.win_v_sz_calc &
				CCDC_HORZ_BC_WIN_V_SIZE_MASK)
				<< CCDC_HORZ_BC_WIN_V_SIZE_SHIFT);

			regw(val, CLHWIN0);

			val = (bc->horz.win_start_h_calc &
				CCDC_HORZ_BC_WIN_START_H_MASK);
			regw(val, CLHWIN1);

			val =
			    (bc->horz.
			     win_start_v_calc & CCDC_HORZ_BC_WIN_START_V_MASK);
			regw(val, CLHWIN2);
		}

		/* vertical clamp caculation paramaters */

		/* OB H Valid */
		val = (bc->vert.ob_h_sz_calc & CCDC_VERT_BC_OB_H_SZ_MASK);

		/* Reset clamp value sel for previous line */
		val |= ((bc->vert.reset_val_sel &
			CCDC_VERT_BC_RST_VAL_SEL_MASK)
			<< CCDC_VERT_BC_RST_VAL_SEL_SHIFT);

		/* Line average coefficient */
		val |= (bc->vert.line_ave_coef <<
			CCDC_VERT_BC_LINE_AVE_COEF_SHIFT);
		regw(val, CLVWIN0);

		/* Configured reset value */
		if (bc->vert.reset_val_sel ==
		    CCDC_VERT_BC_USE_CONFIG_CLAMP_VAL) {
			val =
			    (bc->vert.
			     reset_clamp_val & CCDC_VERT_BC_RST_VAL_MASK);
			regw(val, CLVRV);
		}

		/* Optical Black horizontal start position */
		val = (bc->vert.ob_start_h & CCDC_VERT_BC_OB_START_HORZ_MASK);
		regw(val, CLVWIN1);

		/* Optical Black vertical start position */
		val = (bc->vert.ob_start_v & CCDC_VERT_BC_OB_START_VERT_MASK);
		regw(val, CLVWIN2);

		val = (bc->vert.ob_v_sz_calc & CCDC_VERT_BC_OB_VERT_SZ_MASK);
		regw(val, CLVWIN3);

		/* Vertical start position for BC subtraction */
		val = (bc->vert_start_sub & CCDC_BC_VERT_START_SUB_V_MASK);
		regw(val, CLSV);
	}
}

static void ccdc_config_dfc(struct ccdc_dfc *vdfc)
{
#define DFC_WRITE_WAIT_COUNT	1000
	u32 val, count = DFC_WRITE_WAIT_COUNT;
	int i;

	if (!vdfc->en)
		return;

	/* Correction mode */
	val = ((vdfc->corr_mode & CCDC_VDFC_CORR_MOD_MASK)
		<< CCDC_VDFC_CORR_MOD_SHIFT);

	/* Correct whole line or partial */
	if (vdfc->corr_whole_line)
		val |= 1 << CCDC_VDFC_CORR_WHOLE_LN_SHIFT;

	/* level shift value */
	val |= (vdfc->def_level_shift & CCDC_VDFC_LEVEL_SHFT_MASK) <<
		CCDC_VDFC_LEVEL_SHFT_SHIFT;

	regw(val, DFCCTL);

	/* Defect saturation level */
	val = vdfc->def_sat_level & CCDC_VDFC_SAT_LEVEL_MASK;
	regw(val, VDFSATLV);

	regw(vdfc->table[0].pos_vert & CCDC_VDFC_POS_MASK, DFCMEM0);
	regw(vdfc->table[0].pos_horz & CCDC_VDFC_POS_MASK, DFCMEM1);
	if (vdfc->corr_mode == CCDC_VDFC_NORMAL ||
	    vdfc->corr_mode == CCDC_VDFC_HORZ_INTERPOL_IF_SAT) {
		regw(vdfc->table[0].level_at_pos, DFCMEM2);
		regw(vdfc->table[0].level_up_pixels, DFCMEM3);
		regw(vdfc->table[0].level_low_pixels, DFCMEM4);
	}

	val = regr(DFCMEMCTL);
	/* set DFCMARST and set DFCMWR */
	val |= 1 << CCDC_DFCMEMCTL_DFCMARST_SHIFT;
	val |= 1;
	regw(val, DFCMEMCTL);

	while (count && (regr(DFCMEMCTL) & 0x01))
		count--;

	val = regr(DFCMEMCTL);
	if (!count) {
		dev_err(dev, "defect table write timeout !!!\n");
		return;
	}

	for (i = 1; i < vdfc->num_vdefects; i++) {
		regw(vdfc->table[i].pos_vert & CCDC_VDFC_POS_MASK,
			   DFCMEM0);
		regw(vdfc->table[i].pos_horz & CCDC_VDFC_POS_MASK,
			   DFCMEM1);
		if (vdfc->corr_mode == CCDC_VDFC_NORMAL ||
		    vdfc->corr_mode == CCDC_VDFC_HORZ_INTERPOL_IF_SAT) {
			regw(vdfc->table[i].level_at_pos, DFCMEM2);
			regw(vdfc->table[i].level_up_pixels, DFCMEM3);
			regw(vdfc->table[i].level_low_pixels, DFCMEM4);
		}
		val = regr(DFCMEMCTL);
		/* clear DFCMARST and set DFCMWR */
		val &= ~(1 << CCDC_DFCMEMCTL_DFCMARST_SHIFT);
		val |= 1;
		regw(val, DFCMEMCTL);

		count = DFC_WRITE_WAIT_COUNT;
		while (count && (regr(DFCMEMCTL) & 0x01))
			count--;

		val = regr(DFCMEMCTL);
		if (!count) {
			dev_err(dev, "defect table write timeout !!!\n");
			return;
		}
	}
	if (vdfc->num_vdefects < CCDC_VDFC_TABLE_SIZE) {
		/* Extra cycle needed */
		regw(0, DFCMEM0);
		regw(0x1FFF, DFCMEM1);
		val = 1;
		regw(val, DFCMEMCTL);
	}

	/* enable VDFC */
	ccdc_merge((1 << CCDC_VDFC_EN_SHIFT), (1 << CCDC_VDFC_EN_SHIFT),
		   DFCCTL);

	ccdc_merge((1 << CCDC_VDFC_EN_SHIFT), (0 << CCDC_VDFC_EN_SHIFT),
		   DFCCTL);

	regw(0x6, DFCMEMCTL);
	for (i = 0 ; i < vdfc->num_vdefects; i++) {
		count = DFC_WRITE_WAIT_COUNT;
		while (count && (regr(DFCMEMCTL) & 0x2))
			count--;

		val = regr(DFCMEMCTL);
		if (!count) {
			dev_err(dev, "defect table write timeout !!!\n");
			return;
		}

		val = regr(DFCMEM0) | regr(DFCMEM1) | regr(DFCMEM2) |
			regr(DFCMEM3) | regr(DFCMEM4);
		regw(0x2, DFCMEMCTL);
	}
}

static void ccdc_config_csc(struct ccdc_df_csc *df_csc)
{
	u32 val1 = 0, val2 = 0, i;

	if (!df_csc->csc.en) {
		regw(0, CSCCTL);
		return;
	}
	for (i = 0; i < CCDC_CSC_NUM_COEFF; i++) {
		if ((i % 2) == 0) {
			/* CSCM - LSB */
			val1 =
				((df_csc->csc.coeff[i].integer &
				CCDC_CSC_COEF_INTEG_MASK)
				<< CCDC_CSC_COEF_INTEG_SHIFT) |
				((df_csc->csc.coeff[i].decimal &
				CCDC_CSC_COEF_DECIMAL_MASK));
		} else {

			/* CSCM - MSB */
			val2 =
				((df_csc->csc.coeff[i].integer &
				CCDC_CSC_COEF_INTEG_MASK)
				<< CCDC_CSC_COEF_INTEG_SHIFT) |
				((df_csc->csc.coeff[i].decimal &
				CCDC_CSC_COEF_DECIMAL_MASK));
			val2 <<= CCDC_CSCM_MSB_SHIFT;
			val2 |= val1;
			regw(val2, (CSCM0 + ((i-1) << 1)));
		}
	}

	/* program the active area */
	regw(df_csc->start_pix & CCDC_DF_CSC_SPH_MASK, FMTSPH);
	/**
	 * one extra pixel as required for CSC. Actually number of
	 * pixel - 1 should be configured in this register. So we
	 * need to subtract 1 before writing to FMTSPH, but we will
	 * not do this since csc requires one extra pixel
	 */
	regw((df_csc->num_pixels) & CCDC_DF_CSC_SPH_MASK, FMTLNH);
	regw(df_csc->start_line & CCDC_DF_CSC_SPH_MASK, FMTSLV);
	/**
	 * one extra line as required for CSC. See reason documented for
	 * num_pixels
	 */
	regw((df_csc->num_lines) & CCDC_DF_CSC_SPH_MASK, FMTLNV);

	/* Enable CSC */
	regw(1, CSCCTL);
}

static int ccdc_config_raw(int mode)
{
	struct ccdc_params_raw *params = &ccdc_hw_params_bayer;
	struct ccdc_config_params_raw *module_params =
		&ccdc_hw_params_bayer.config_params;
	struct vpss_pg_frame_size frame_size;
	struct vpss_sync_pol sync;
	u32 val;

	dev_dbg(dev, "\nStarting ccdc_config_raw..\n");
	ccdc_restore_defaults();

	/* Configure CCDCFG register */

	/**
	 *Set CCD Not to swap input since input is RAW data
	 *set FID detection function to Latch at V-Sync
	 *set WENLOG - ccdc valid area
	 *set TRGSEL
	 *set EXTRG
	 * Packed to 8 or 16 bits
	 */

	val = CCDC_YCINSWP_RAW | CCDC_CCDCFG_FIDMD_LATCH_VSYNC |
		CCDC_CCDCFG_WENLOG_AND | CCDC_CCDCFG_TRGSEL_WEN |
		CCDC_CCDCFG_EXTRG_DISABLE | (data_pack & CCDC_DATA_PACK_MASK);

	dev_dbg(dev, "Writing 0x%x to ...CCDCFG \n", val);
	regw(val, CCDCFG);

	/**
	 * Configure the vertical sync polarity(MODESET.VDPOL)
	 * Configure the horizontal sync polarity (MODESET.HDPOL)
	 * Configure frame id polarity (MODESET.FLDPOL)
	 * Configure data polarity
	 * Configure External WEN Selection
	 * Configure frame format(progressive or interlace)
	 * Configure pixel format (Input mode)
	 * Configure the data shift
	 */

	val = CCDC_VDHDOUT_INPUT |
		((params->vd_pol & CCDC_VD_POL_MASK) << CCDC_VD_POL_SHIFT) |
		((params->hd_pol & CCDC_HD_POL_MASK) << CCDC_HD_POL_SHIFT) |
		((params->fid_pol & CCDC_FID_POL_MASK) << CCDC_FID_POL_SHIFT) |
		((CCDC_DATAPOL_NORMAL & CCDC_DATAPOL_MASK)
			<< CCDC_DATAPOL_SHIFT) |
		((CCDC_EXWEN_DISABLE & CCDC_EXWEN_MASK) << CCDC_EXWEN_SHIFT) |
		((params->frm_fmt & CCDC_FRM_FMT_MASK) << CCDC_FRM_FMT_SHIFT) |
		((params->pix_fmt & CCDC_INPUT_MASK) << CCDC_INPUT_SHIFT) |
		((params->config_params.data_shift & CCDC_DATASFT_MASK)
			<< CCDC_DATASFT_SHIFT);

	regw(val, MODESET);
	dev_dbg(dev, "Writing 0x%x to MODESET...\n", val);

	/**
	 * Configure GAMMAWD register
	 * CFA pattern setting
	 */
	val = (params->cfa_pat & CCDC_GAMMAWD_CFA_MASK) <<
		CCDC_GAMMAWD_CFA_SHIFT;

	/* Gamma msb */
	if (module_params->compress.alg == CCDC_ALAW) {
		val = val | CCDC_ALAW_ENABLE |
			((params->data_msb & CCDC_ALAW_GAMA_WD_MASK) <<
			CCDC_ALAW_GAMA_WD_SHIFT);
	}

	regw(val, CGAMMAWD);

	/* Configure DPCM compression settings */
	if (module_params->compress.alg == CCDC_DPCM) {
		val =  1 << CCDC_DPCM_EN_SHIFT;
		val |= (module_params->compress.pred &
			CCDC_DPCM_PREDICTOR_MASK) << CCDC_DPCM_PREDICTOR_SHIFT;
	}

	regw(val, MISC);
	/* Configure Gain & Offset */

	ccdc_config_gain_offset();

	/* Configure Color pattern */
	val = (params->config_params.col_pat_field0.olop) |
	(params->config_params.col_pat_field0.olep << 2) |
	(params->config_params.col_pat_field0.elop << 4) |
	(params->config_params.col_pat_field0.elep << 6) |
	(params->config_params.col_pat_field1.olop << 8) |
	(params->config_params.col_pat_field1.olep << 10) |
	(params->config_params.col_pat_field1.elop << 12) |
	(params->config_params.col_pat_field1.elep << 14);
	regw(val, CCOLP);
	dev_dbg(dev, "Writing %x to CCOLP ...\n", val);

	/* Configure HSIZE register  */
	val =
	    (params->
	     horz_flip_en & CCDC_HSIZE_FLIP_MASK) << CCDC_HSIZE_FLIP_SHIFT;

	/* calculate line offset in 32 bytes based on pack value */
	if (data_pack == CCDC_PACK_8BIT)
		val |= (((params->win.width + 31) >> 5) & CCDC_LINEOFST_MASK);
	else if (data_pack == CCDC_PACK_12BIT)
		val |= ((((params->win.width +
			   (params->win.width >> 2)) +
			  31) >> 5) & CCDC_LINEOFST_MASK);
	else
		val |=
		    ((((params->win.width * 2) +
		       31) >> 5) & CCDC_LINEOFST_MASK);
	regw(val, HSIZE);

	/* Configure SDOFST register  */
	if (params->frm_fmt == CCDC_FRMFMT_INTERLACED) {
		if (params->image_invert_en) {
			/* For interlace inverse mode */
			regw(0x4B6D, SDOFST);
			dev_dbg(dev, "Writing 0x4B6D to SDOFST...\n");
		} else {
			/* For interlace non inverse mode */
			regw(0x0B6D, SDOFST);
			dev_dbg(dev, "Writing 0x0B6D to SDOFST...\n");
		}
	} else if (params->frm_fmt == CCDC_FRMFMT_PROGRESSIVE) {
		if (params->image_invert_en) {
			/* For progessive inverse mode */
			regw(0x4000, SDOFST);
			dev_dbg(dev, "Writing 0x4000 to SDOFST...\n");
		} else {
			/* For progessive non inverse mode */
			regw(0x0000, SDOFST);
			dev_dbg(dev, "Writing 0x0000 to SDOFST...\n");
		}
	}

	/* Configure video window */
	ccdc_setwin(&params->win, params->frm_fmt, 1);

	/* Configure Black Clamp */
	ccdc_config_bclamp(&module_params->bclamp);

	/* Configure Vertical Defection Pixel Correction */
	ccdc_config_dfc(&module_params->dfc);

	if (!module_params->df_csc.df_or_csc) {
		/* Configure Color Space Conversion */
		ccdc_config_csc(&module_params->df_csc);
	}

	/* Configure Culling */
	ccdc_config_culling(&module_params->culling);

	/* Configure Horizontal and vertical offsets(DFC,LSC,Gain) */
	val = module_params->horz_offset & CCDC_DATA_H_OFFSET_MASK;
	regw(val, DATAHOFST);

	val = module_params->vert_offset & CCDC_DATA_V_OFFSET_MASK;
	regw(val, DATAVOFST);

	/* Setup test pattern if enabled */
	if (params->config_params.test_pat_gen) {
		/* configure pattern register  */

		/* Use the HD/VD pol settings from user */
		sync.ccdpg_hdpol = params->hd_pol & CCDC_HD_POL_MASK;
		sync.ccdpg_vdpol = params->vd_pol & CCDC_VD_POL_MASK;

		vpss_set_sync_pol(sync);

		frame_size.hlpfr = ccdc_hw_params_bayer.win.width;
		frame_size.pplen = ccdc_hw_params_bayer.win.height;
		vpss_set_pg_frame_size(frame_size);
		vpss_select_ccdc_source(VPSS_PGLPBK);
	} else
	dev_dbg(dev, "\nEnd of ccdc_config_ycbcr...\n");

	return 0;
}

static int ccdc_validate_df_csc_params(struct ccdc_df_csc *df_csc)
{
	struct ccdc_color_space_conv *csc;
	int i, csc_df_en = 0;
	int err = -EINVAL;

	if (!df_csc->df_or_csc) {
		/* csc configuration */
		csc = &df_csc->csc;
		if (csc->en) {
			csc_df_en = 1;
			for (i = 0; i < CCDC_CSC_NUM_COEFF; i++) {
				if (csc->coeff[i].integer >
					CCDC_CSC_COEF_INTEG_MASK ||
				    csc->coeff[i].decimal >
					CCDC_CSC_COEF_DECIMAL_MASK) {
					printk(KERN_ERR
					       "invalid csc coefficients \n");
					return err;
				}
			}
		}
	}

	if (df_csc->start_pix > CCDC_DF_CSC_SPH_MASK) {
		dev_err(dev, "invalid df_csc start pix value \n");
		return err;
	}
	if (df_csc->num_pixels > CCDC_DF_NUMPIX) {
		dev_err(dev, "invalid df_csc num pixels value \n");
		return err;
	}
	if (df_csc->start_line > CCDC_DF_CSC_LNH_MASK) {
		dev_err(dev, "invalid df_csc start_line value \n");
		return err;
	}
	if (df_csc->num_lines > CCDC_DF_NUMLINES) {
		dev_err(dev, "invalid df_csc num_lines value \n");
		return err;
	}

	return 0;
}

static int ccdc_validate_dfc_params(struct ccdc_dfc *dfc)
{
	int err = -EINVAL;
	int i;

	if (dfc->en) {
		if (dfc->corr_whole_line > 1) {
			dev_err(dev, "invalid corr_whole_line value \n");
			return err;
		}

		if (dfc->def_level_shift > 4) {
			dev_err(dev, "invalid def_level_shift value \n");
			return err;
		}

		if (dfc->def_sat_level > 4095) {
			dev_err(dev, "invalid def_sat_level value \n");
			return err;
		}
		if ((!dfc->num_vdefects) || (dfc->num_vdefects > 8)) {
			dev_err(dev, "invalid num_vdefects value \n");
			return err;
		}
		for (i = 0; i < CCDC_VDFC_TABLE_SIZE; i++) {
			if (dfc->table[i].pos_vert > 0x1fff) {
				dev_err(dev, "invalid pos_vert value \n");
				return err;
			}
			if (dfc->table[i].pos_horz > 0x1fff) {
				dev_err(dev, "invalid pos_horz value \n");
				return err;
			}
		}
	}

	return 0;
}

static int ccdc_validate_bclamp_params(struct ccdc_black_clamp *bclamp)
{
	int err = -EINVAL;

	if (bclamp->dc_offset > 0x1fff) {
		dev_err(dev, "invalid bclamp dc_offset value \n");
		return err;
	}

	if (bclamp->en) {
		if (bclamp->horz.clamp_pix_limit > 1) {
			printk(KERN_ERR
			       "invalid bclamp horz clamp_pix_limit value \n");
			return err;
		}

		if (bclamp->horz.win_count_calc < 1 ||
		    bclamp->horz.win_count_calc > 32) {
			printk(KERN_ERR
			       "invalid bclamp horz win_count_calc value \n");
			return err;
		}

		if (bclamp->horz.win_start_h_calc > 0x1fff) {
			printk(KERN_ERR
			       "invalid bclamp win_start_v_calc value \n");
			return err;
		}

		if (bclamp->horz.win_start_v_calc > 0x1fff) {
			printk(KERN_ERR
			       "invalid bclamp win_start_v_calc value \n");
			return err;
		}

		if (bclamp->vert.reset_clamp_val > 0xfff) {
			printk(KERN_ERR
			       "invalid bclamp reset_clamp_val value \n");
			return err;
		}

		if (bclamp->vert.ob_v_sz_calc > 0x1fff) {
			dev_err(dev, "invalid bclamp ob_v_sz_calc value \n");
			return err;
		}

		if (bclamp->vert.ob_start_h > 0x1fff) {
			dev_err(dev, "invalid bclamp ob_start_h value \n");
			return err;
		}

		if (bclamp->vert.ob_start_v > 0x1fff) {
			dev_err(dev, "invalid bclamp ob_start_h value \n");
			return err;
		}
	}

	return 0;
}

static int ccdc_validate_gain_ofst_params(struct ccdc_gain_offsets_adj
					  *gain_offset)
{
	int err = -EINVAL;

	if (gain_offset->gain_sdram_en ||
	    gain_offset->gain_ipipe_en ||
	    gain_offset->gain_h3a_en) {
		if ((gain_offset->gain.r_ye.integer > 7) ||
		    (gain_offset->gain.r_ye.decimal > 0x1ff)) {
			dev_err(dev, "invalid  gain r_ye\n");
			return err;
		}
		if ((gain_offset->gain.gr_cy.integer > 7) ||
		    (gain_offset->gain.gr_cy.decimal > 0x1ff)) {
			dev_err(dev, "invalid  gain gr_cy\n");
			return err;
		}
		if ((gain_offset->gain.gb_g.integer > 7) ||
		    (gain_offset->gain.gb_g.decimal > 0x1ff)) {
			dev_err(dev, "invalid  gain gb_g\n");
			return err;
		}
		if ((gain_offset->gain.b_mg.integer > 7) ||
		    (gain_offset->gain.b_mg.decimal > 0x1ff)) {
			dev_err(dev, "invalid  gain b_mg\n");
			return err;
		}
	}
	if (gain_offset->offset_sdram_en ||
	    gain_offset->offset_ipipe_en ||
	    gain_offset->offset_h3a_en) {
		if (gain_offset->offset > 0xfff) {
			dev_err(dev, "invalid  gain b_mg\n");
			return err;
		}
	}

	return 0;
}

static int validate_ccdc_config_params_raw(struct ccdc_params_raw *param)
{
	struct ccdc_config_params_raw *module_params;
	int err = 0;

	if ((param == NULL) || (&param->config_params == NULL)) {
		printk(KERN_ERR
		"Invalid argument for validate_ccdc_config_params_raw()\n");
		return err;
	}
	module_params = kmalloc(sizeof(struct ccdc_config_params_raw),
				GFP_KERNEL);
	if (module_params == NULL) {
		dev_err(dev, "memory allocation failure\n");
		return -EFAULT;
	}
	if (copy_from_user(module_params, &param->config_params,
			   sizeof(struct ccdc_config_params_raw))) {
		printk(KERN_ERR
		       "error in copying ccdc params to kernel\n");
		kfree(module_params);
		return -EFAULT;
	}
	err = ccdc_validate_df_csc_params(&module_params->df_csc);
	if (err)
		goto free_exit;
	err |= ccdc_validate_dfc_params(&module_params->dfc);
	if (err)
		goto free_exit;
	err |= ccdc_validate_bclamp_params(&module_params->bclamp);
	if (err)
		goto free_exit;
	err |= ccdc_validate_gain_ofst_params(&module_params->gain_offset);
	if (err)
		goto free_exit;
	memcpy(&ccdc_hw_params_bayer.config_params, module_params,
		sizeof(struct ccdc_config_params_raw));

free_exit:
		kfree(module_params);
		return err;
}

static int ccdc_set_buftype(enum ccdc_buftype buf_type)
{
	if (ccdc_if_type == VPFE_RAW_BAYER)
		ccdc_hw_params_bayer.buf_type = buf_type;
	else
		ccdc_hw_params_ycbcr.buf_type = buf_type;

	return 0;

}
static enum ccdc_buftype ccdc_get_buftype(void)
{
	if (ccdc_if_type == VPFE_RAW_BAYER)
		return ccdc_hw_params_bayer.buf_type;

	return ccdc_hw_params_ycbcr.buf_type;
}

static int ccdc_enum_pix(u32 *pix, int i)
{
	int ret = -EINVAL;

	if (ccdc_if_type == VPFE_RAW_BAYER) {
		if (i < ARRAY_SIZE(ccdc_raw_bayer_pix_formats)) {
			*pix = ccdc_raw_bayer_pix_formats[i];
			ret = 0;
		}
	} else {
		if (i < ARRAY_SIZE(ccdc_raw_yuv_pix_formats)) {
			*pix = ccdc_raw_yuv_pix_formats[i];
			ret = 0;
		}
	}

	return ret;
}

static int ccdc_set_pixel_format(unsigned int pixfmt)
{
	if (ccdc_if_type == VPFE_RAW_BAYER) {
		if (pixfmt == V4L2_PIX_FMT_SBGGR8) {
			if ((ccdc_hw_params_bayer.config_params.compress.alg !=
					CCDC_ALAW) &&
			    (ccdc_hw_params_bayer.config_params.compress.alg !=
					CCDC_DPCM)) {
				dev_err(dev, "Either configure A-Law or"
						"DPCM\n");
				return -EINVAL;
			}
			data_pack = CCDC_PACK_8BIT;
		} else if (pixfmt == V4L2_PIX_FMT_SBGGR16) {
			if (ccdc_hw_params_bayer.config_params.compress.alg !=
					CCDC_NO_COMPRESSION) {
				dev_err(dev, "Disable compression"
						" for this pixel format\n");
				return -EINVAL;
			}
			data_pack = CCDC_PACK_16BIT;
		} else
			return -EINVAL;
		ccdc_hw_params_bayer.pix_fmt = CCDC_PIXFMT_RAW;
	} else {
		if (pixfmt == V4L2_PIX_FMT_YUYV)
			ccdc_hw_params_ycbcr.pix_order = CCDC_PIXORDER_YCBYCR;
		else if (pixfmt == V4L2_PIX_FMT_UYVY)
			ccdc_hw_params_ycbcr.pix_order = CCDC_PIXORDER_CBYCRY;
		else
			return -EINVAL;
		data_pack = CCDC_PACK_8BIT;
	}

	return 0;
}

static u32 ccdc_get_pixel_format(void)
{
	u32 pixfmt;

	if (ccdc_if_type == VPFE_RAW_BAYER)
		if (ccdc_hw_params_bayer.config_params.compress.alg
			== CCDC_ALAW
			|| ccdc_hw_params_bayer.config_params.compress.alg
			== CCDC_DPCM)
				pixfmt = V4L2_PIX_FMT_SBGGR8;
		else
			pixfmt = V4L2_PIX_FMT_SBGGR16;
	else {
		if (ccdc_hw_params_ycbcr.pix_order == CCDC_PIXORDER_YCBYCR)
			pixfmt = V4L2_PIX_FMT_YUYV;
		else
			pixfmt = V4L2_PIX_FMT_UYVY;
	}

	return pixfmt;
}

static int ccdc_set_image_window(struct v4l2_rect *win)
{
	if (ccdc_if_type == VPFE_RAW_BAYER) {
		ccdc_hw_params_bayer.win.top = win->top;
		ccdc_hw_params_bayer.win.left = win->left;
		ccdc_hw_params_bayer.win.width = win->width;
		ccdc_hw_params_bayer.win.height = win->height;
	} else {
		ccdc_hw_params_ycbcr.win.top = win->top;
		ccdc_hw_params_ycbcr.win.left = win->left;
		ccdc_hw_params_ycbcr.win.width = win->width;
		ccdc_hw_params_ycbcr.win.height = win->height;
	}

	return 0;
}

static void ccdc_get_image_window(struct v4l2_rect *win)
{
	if (ccdc_if_type == VPFE_RAW_BAYER)
		*win = ccdc_hw_params_bayer.win;
	else
		*win = ccdc_hw_params_ycbcr.win;
}

static unsigned int ccdc_get_line_length(void)
{
	unsigned int len;

	if (ccdc_if_type == VPFE_RAW_BAYER) {
		if (data_pack == CCDC_PACK_8BIT)
			len = ((ccdc_hw_params_bayer.win.width));
		else if (data_pack == CCDC_PACK_12BIT)
			len = (((ccdc_hw_params_bayer.win.width * 2) +
				 (ccdc_hw_params_bayer.win.width >> 2)));
		else
			len = (((ccdc_hw_params_bayer.win.width * 2)));
	} else
		len = (((ccdc_hw_params_ycbcr.win.width * 2)));

	return ALIGN(len, 32);
}

static int ccdc_set_frame_format(enum ccdc_frmfmt frm_fmt)
{
	if (ccdc_if_type == VPFE_RAW_BAYER)
		ccdc_hw_params_bayer.frm_fmt = frm_fmt;
	else
		ccdc_hw_params_ycbcr.frm_fmt = frm_fmt;

	return 0;
}
static enum ccdc_frmfmt ccdc_get_frame_format(void)
{
	if (ccdc_if_type == VPFE_RAW_BAYER)
		return ccdc_hw_params_bayer.frm_fmt;
	else
		return ccdc_hw_params_ycbcr.frm_fmt;
}

static int ccdc_getfid(void)
{
	return (regr(MODESET) >> 15) & 0x1;
}

/* misc operations */
static void ccdc_setfbaddr(unsigned long addr)
{
	regw((addr >> 21) & 0x07ff, CADU);
	regw((addr >> 5) & 0x0ffff, CADL);
}

static int ccdc_set_hw_if_params(struct vpfe_hw_if_param *params)
{
	ccdc_if_type = params->if_type;

	switch (params->if_type) {
	case VPFE_BT656:
	case VPFE_BT656_10BIT:
	case VPFE_YCBCR_SYNC_8:
		ccdc_hw_params_ycbcr.pix_fmt = CCDC_PIXFMT_YCBCR_8BIT;
		ccdc_hw_params_ycbcr.pix_order = CCDC_PIXORDER_CBYCRY;
		break;
	case VPFE_BT1120:
	case VPFE_YCBCR_SYNC_16:
		ccdc_hw_params_ycbcr.pix_fmt = CCDC_PIXFMT_YCBCR_16BIT;
		ccdc_hw_params_ycbcr.pix_order = CCDC_PIXORDER_CBYCRY;
		break;
	case VPFE_RAW_BAYER:
		ccdc_hw_params_bayer.pix_fmt = CCDC_PIXFMT_RAW;
		break;
	default:
		dev_err(dev, "Invalid interface type\n");
		return -EINVAL;
	}

	return 0;
}

/* Parameter operations */
static int ccdc_set_params(void __user *params)
{
	struct ccdc_params_raw ccdc_raw_params;
	int ret;

	/* only raw module parameters can be set through the IOCTL */
	if (ccdc_if_type != VPFE_RAW_BAYER)
		return -EINVAL;

	ret = copy_from_user(&ccdc_raw_params, params, sizeof(ccdc_raw_params));
	if (ret) {
		dev_dbg(dev, "ccdc_set_params: error in copying ccdc"
			"params, %d\n", ret);
		return -EFAULT;
	}

	if (!validate_ccdc_config_params_raw(&ccdc_raw_params)) {
		/**
		 *TODO:we dont need to do this here because its already done in
		 *validate_ccdc_config_params_raw
		 */
		memcpy(&ccdc_hw_params_bayer.config_params,
			&ccdc_raw_params,
			sizeof(ccdc_raw_params));
		return 0;
	}

	return -EINVAL;
}

/* This function will configure CCDC for YCbCr parameters. */
static int ccdc_config_ycbcr(void)
{
	struct ccdc_ycbcr_config *params = &ccdc_hw_params_ycbcr;
	struct vpss_pg_frame_size frame_size;
	u32 modeset = 0, ccdcfg = 0;
	struct vpss_sync_pol sync;

	/**
	 * first reset the CCDC
	 * all registers have default values after reset
	 * This is important since we assume default values to be set in
	 * a lot of registers that we didn't touch
	 */
	dev_dbg(dev, "\nStarting ccdc_config_ycbcr...");
	ccdc_restore_defaults();

	/* configure pixel format or input mode */
	modeset = modeset | ((params->pix_fmt & CCDC_INPUT_MASK)
		<< CCDC_INPUT_SHIFT) |
	((params->frm_fmt & CCDC_FRM_FMT_MASK) << CCDC_FRM_FMT_SHIFT) |
	(((params->fid_pol & CCDC_FID_POL_MASK) << CCDC_FID_POL_SHIFT))	|
	(((params->hd_pol & CCDC_HD_POL_MASK) << CCDC_HD_POL_SHIFT)) |
	(((params->vd_pol & CCDC_VD_POL_MASK) << CCDC_VD_POL_SHIFT));

	/* pack the data to 8-bit CCDCCFG */
	switch (ccdc_if_type) {
	case VPFE_BT656:
		if (params->pix_fmt != CCDC_PIXFMT_YCBCR_8BIT) {
			dev_err(dev, "Invalid pix_fmt(input mode)\n");
			return -1;
		}
		modeset |=
			((VPFE_PINPOL_NEGATIVE & CCDC_VD_POL_MASK)
			<< CCDC_VD_POL_SHIFT);
		regw(3, REC656IF);
		ccdcfg = ccdcfg | CCDC_DATA_PACK8 | CCDC_YCINSWP_YCBCR;
		break;
	case VPFE_BT656_10BIT:
		if (params->pix_fmt != CCDC_PIXFMT_YCBCR_8BIT) {
			dev_err(dev, "Invalid pix_fmt(input mode)\n");
			return -1;
		}
		/* setup BT.656, embedded sync  */
		regw(3, REC656IF);
		/* enable 10 bit mode in ccdcfg */
		ccdcfg = ccdcfg | CCDC_DATA_PACK8 | CCDC_YCINSWP_YCBCR |
			CCDC_BW656_ENABLE;
		break;
	case VPFE_BT1120:
		if (params->pix_fmt != CCDC_PIXFMT_YCBCR_16BIT) {
			dev_err(dev, "Invalid pix_fmt(input mode)\n");
			return -EINVAL;
		}
		regw(3, REC656IF);
		break;

	case VPFE_YCBCR_SYNC_8:
		ccdcfg |= CCDC_DATA_PACK8;
		ccdcfg |= CCDC_YCINSWP_YCBCR;
		if (params->pix_fmt != CCDC_PIXFMT_YCBCR_8BIT) {
			dev_err(dev, "Invalid pix_fmt(input mode)\n");
			return -EINVAL;
		}
		break;
	case VPFE_YCBCR_SYNC_16:
		if (params->pix_fmt != CCDC_PIXFMT_YCBCR_16BIT) {
			dev_err(dev, "Invalid pix_fmt(input mode)\n");
			return -EINVAL;
		}
		break;
	default:
		/* should never come here */
		dev_err(dev, "Invalid interface type\n");
		return -EINVAL;
	}

	regw(modeset, MODESET);

	/* Set up pix order */
	ccdcfg |= (params->pix_order & CCDC_PIX_ORDER_MASK) <<
		CCDC_PIX_ORDER_SHIFT;

	regw(ccdcfg, CCDCFG);

	/* configure video window */
	if ((ccdc_if_type == VPFE_BT1120) ||
	    (ccdc_if_type == VPFE_YCBCR_SYNC_16))
		ccdc_setwin(&params->win, params->frm_fmt, 1);
	else
		ccdc_setwin(&params->win, params->frm_fmt, 2);

	/**
	 * configure the horizontal line offset
	 * this is done by rounding up width to a multiple of 16 pixels
	 * and multiply by two to account for y:cb:cr 4:2:2 data
	 */
	regw(((((params->win.width * 2) + 31) & 0xffffffe0) >> 5), HSIZE);

	/* configure the memory line offset */
	if ((params->frm_fmt == CCDC_FRMFMT_INTERLACED) &&
	    (params->buf_type == CCDC_BUFTYPE_FLD_INTERLEAVED)) {
		/* two fields are interleaved in memory */
		regw(0x00000249, SDOFST);
	}

	/* Setup test pattern if enabled */
	if (ccdc_hw_params_bayer.config_params.test_pat_gen) {
		sync.ccdpg_hdpol = (params->hd_pol & CCDC_HD_POL_MASK);
		sync.ccdpg_vdpol = (params->vd_pol & CCDC_VD_POL_MASK);
		vpss_set_sync_pol(sync);
		vpss_set_pg_frame_size(frame_size);
	}

	return 0;
}

static int ccdc_configure(void)
{
	if (ccdc_if_type == VPFE_RAW_BAYER)
		return ccdc_config_raw(1);
	else
		ccdc_config_ycbcr();

	return 0;
}

static int ccdc_close(struct device *device)
{
	/* copy defaults to module params */
	memcpy(&ccdc_hw_params_bayer.config_params,
	       &ccdc_config_defaults,
	       sizeof(struct ccdc_config_params_raw));

	return 0;
}

static struct ccdc_hw_device ccdc_hw_dev = {
	.name = "DM365 ISIF",
	.owner = THIS_MODULE,
	.hw_ops = {
		.open = ccdc_open,
		.close = ccdc_close,
		.set_ccdc_base = ccdc_set_ccdc_base,
		.enable = ccdc_enable,
		.enable_out_to_sdram = ccdc_enable_output_to_sdram,
		.set_hw_if_params = ccdc_set_hw_if_params,
		.set_params = ccdc_set_params,
		.configure = ccdc_configure,
		.set_buftype = ccdc_set_buftype,
		.get_buftype = ccdc_get_buftype,
		.enum_pix = ccdc_enum_pix,
		.set_pixel_format = ccdc_set_pixel_format,
		.get_pixel_format = ccdc_get_pixel_format,
		.set_frame_format = ccdc_set_frame_format,
		.get_frame_format = ccdc_get_frame_format,
		.set_image_window = ccdc_set_image_window,
		.get_image_window = ccdc_get_image_window,
		.get_line_length = ccdc_get_line_length,
		.setfbaddr = ccdc_setfbaddr,
		.getfid = ccdc_getfid,
	},
};

static int dm365_ccdc_init(void)
{
	int ret;
	printk(KERN_NOTICE "dm365_ccdc_init\n");
	davinci_cfg_reg(DM365_VIN_CAM_WEN);
	davinci_cfg_reg(DM365_VIN_CAM_VD);
	davinci_cfg_reg(DM365_VIN_CAM_HD);
	davinci_cfg_reg(DM365_VIN_YIN4_7_EN);
	davinci_cfg_reg(DM365_VIN_YIN0_3_EN);
	ret = vpfe_register_ccdc_device(&ccdc_hw_dev);
	if (ret < 0)
		return ret;
	printk(KERN_NOTICE "%s is registered with vpfe.\n",
		ccdc_hw_dev.name);

	return 0;
}

static void dm365_ccdc_exit(void)
{
	vpfe_unregister_ccdc_device(&ccdc_hw_dev);
}

module_init(dm365_ccdc_init);
module_exit(dm365_ccdc_exit);


MODULE_LICENSE("GPL");