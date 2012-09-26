/*
 * ALSA SoC TLV320AIC23 codec driver
 *
 * Author:      Arun KS, <arunks@mistralsolutions.com>
 * Copyright:   (C) 2008 Mistral Solutions Pvt Ltd.,
 *
 * Based on sound/soc/codecs/wm8731.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 *  The AIC23 is a driver for a low power stereo audio
 *  codec tlv320aic23
 *
 *  The machine layer should disable unsupported inputs/outputs by
 *  snd_soc_dapm_disable_pin(codec, "LHPOUT"), etc.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/attribute_container.h>		// DJS - sysfs stuff
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>

#include "tlv320aic23.h"

#define AIC23_VERSION "0.1"

#define AM

#define DEBUG
#define DEBUG_WRITES

/*
 * AIC23 register cache
 */
static const u16 tlv320aic23_reg[] = {
	0x0097, 0x0097, 0x00F9, 0x00F9,	/* 0 */
	0x001A, 0x0004, 0x0007, 0x0001,	/* 4 */
	0x0020, 0x0000, 0x0000, 0x0000,	/* 8 */
	0x0000, 0x0000, 0x0000, 0x0000,	/* 12 */
};

/*
 * read tlv320aic23 register cache
 */
static inline unsigned int tlv320aic23_read_reg_cache(struct snd_soc_codec
						      *codec, unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= ARRAY_SIZE(tlv320aic23_reg))
		return -1;
	return cache[reg];
}

/*
 * write tlv320aic23 register cache
 */
static inline void tlv320aic23_write_reg_cache(struct snd_soc_codec *codec,
					       u8 reg, u16 value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= ARRAY_SIZE(tlv320aic23_reg))
		return;
	cache[reg] = value;
}

#ifdef DEBUG_WRITES
static const char *tlv320aic23_regnames[] = {
	"TLV320AIC23_LINVOL",
	"TLV320AIC23_RINVOL",
	"TLV320AIC23_LCHNVOL",
	"TLV320AIC23_RCHNVOL",
	"TLV320AIC23_ANLG",
	"TLV320AIC23_DIGT",
	"TLV320AIC23_PWR",
	"TLV320AIC23_DIGT_FMT",
	"TLV320AIC23_SRATE",
	"TLV320AIC23_ACTIVE",
	"", "", "", "", "",
	"TLV320AIC23_RESET",
};
#endif

/*
 * write to the tlv320aic23 register space
 */
static int tlv320aic23_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{

	u8 data[2];

	/* TLV320AIC23 has 7 bit address and 9 bits of data
	 * so we need to switch one data bit into reg and rest
	 * of data into val
	 */

	if ((reg < 0 || reg > 9) && (reg != 15)) {
		printk(KERN_WARNING "%s Invalid register R%u\n", __func__, reg);
		return -1;
	}

	data[0] = (reg << 1) | (value >> 8 & 0x01);
	data[1] = value & 0xff;

	tlv320aic23_write_reg_cache(codec, reg, value);
	#ifdef DEBUG_WRITES
	pr_info("tlv320aic23_write(%s, 0x%x)\n", tlv320aic23_regnames[reg], value);
	#endif

	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;

	printk(KERN_ERR "%s cannot write %03x to register R%u\n", __func__,
	       value, reg);

	return -EIO;
}

static const char *rec_src_text[] = { "Line", "Mic" };
static const char *deemph_text[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *switch_off_on[] = { "Off", "On"}; //Off = 0, On = 1
static const char *switch_on_off[] = { "On", "Off"}; //On = 0, Off = 1
static const char *switch_normal_muted[] = { "Normal", "Muted"}; //Normal = 0, Muted =1
static const char *switch_mic_boost[] = {"0 dB", "20 dB"};
static const char *switch_disabled_enabled[] = {"Disabled", "Enabled"};


static const struct soc_enum rec_src_enum =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 2, 2, rec_src_text);

#ifndef AM
static const struct snd_kcontrol_new tlv320aic23_rec_src_mux_controls =
    SOC_DAPM_ENUM("Input Select", rec_src_enum);
#else
    static const struct snd_kcontrol_new tlv320aic23_rec_src_mux_controls =
    SOC_DAPM_ENUM("Line Capture Route", rec_src_enum);
#endif

#ifndef AM
static const struct soc_enum tlv320aic23_rec_src =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 2, 2, rec_src_text);
#endif

static const struct soc_enum tlv320aic23_deemph =
	SOC_ENUM_SINGLE(TLV320AIC23_DIGT, 1, 4, deemph_text);

static const DECLARE_TLV_DB_SCALE(out_gain_tlv, -12100, 100, 0);
static const DECLARE_TLV_DB_SCALE(input_gain_tlv, -1725, 75, 0);
static const DECLARE_TLV_DB_SCALE(sidetone_vol_tlv, -1800, 300, 0);

static int snd_soc_tlv320aic23_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 val, reg;

	val = (ucontrol->value.integer.value[0] & 0x07);

	/* linear conversion to userspace
	* 000	=	-6db
	* 001	=	-9db
	* 010	=	-12db
	* 011	=	-18db (Min)
	* 100	=	0db (Max)
	*/
	val = (val >= 4) ? 4  : (3 - val);

	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_ANLG) & (~0x1C0);
	tlv320aic23_write(codec, TLV320AIC23_ANLG, reg | (val << 6));

	return 0;
}

static int snd_soc_tlv320aic23_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 val;

	val = tlv320aic23_read_reg_cache(codec, TLV320AIC23_ANLG) & (0x1C0);
	val = val >> 6;
	val = (val >= 4) ? 4  : (3 -  val);
	ucontrol->value.integer.value[0] = val;
	return 0;

}

#define SOC_TLV320AIC23_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_tlv320aic23_get_volsw,\
	.put = snd_soc_tlv320aic23_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

#ifndef AM
static const struct snd_kcontrol_new tlv320aic23_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Digital Playback Volume", TLV320AIC23_LCHNVOL,
			 TLV320AIC23_RCHNVOL, 0, 127, 0, out_gain_tlv),
	SOC_SINGLE("Digital Playback Switch", TLV320AIC23_DIGT, 3, 1, 1),
	SOC_DOUBLE_R("Line Input Switch", TLV320AIC23_LINVOL,
		     TLV320AIC23_RINVOL, 7, 1, 0),
	SOC_DOUBLE_R_TLV("Line Input Volume", TLV320AIC23_LINVOL,
			 TLV320AIC23_RINVOL, 0, 31, 0, input_gain_tlv),
	SOC_SINGLE("Mic Input Switch", TLV320AIC23_ANLG, 1, 1, 1),
	SOC_SINGLE("Mic Booster Switch", TLV320AIC23_ANLG, 0, 1, 0),
	SOC_TLV320AIC23_SINGLE_TLV("Sidetone Volume", TLV320AIC23_ANLG,
				  6, 4, 0, sidetone_vol_tlv),
	SOC_ENUM("Playback De-emphasis", tlv320aic23_deemph),
	SOC_ENUM("Input Switch", tlv320aic23_rec_src)
};
#endif

static const struct soc_enum tlv320aic23_playback_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_DIGT, 3, 2, switch_off_on);

static const struct soc_enum tlv320aic23_left_capture_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_LINVOL,7, 2, switch_normal_muted);

static const struct soc_enum tlv320aic23_right_capture_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_RINVOL,7, 2, switch_normal_muted);

static const struct soc_enum tlv320aic23_line_input_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_PWR, 0, 2, switch_on_off);

static const struct soc_enum tlv320aic23_mic_input_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_PWR, 1, 2, switch_on_off);

static const struct soc_enum tlv320aic23_micm_input_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 1, 2, switch_normal_muted);

static const struct soc_enum tlv320aic23_micb_input_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 0, 2, switch_mic_boost);

static const struct soc_enum tlv320aic23_adchp_input_switch =
	SOC_ENUM_SINGLE(TLV320AIC23_DIGT, 0, 2, switch_disabled_enabled);

static const struct soc_enum tlv320aic23_adc_input_switch =
		SOC_ENUM_SINGLE(TLV320AIC23_PWR, 2, 2, switch_on_off);


static const struct snd_kcontrol_new tlv320aic23_snd_controls[] = {

	SOC_DOUBLE_R_TLV("Speaker Playback Volume", TLV320AIC23_LCHNVOL,
			 TLV320AIC23_RCHNVOL, 0, 127, 0, out_gain_tlv),
	//SOC_SINGLE("Playback Switch", TLV320AIC23_DIGT, 3, 1, 1),
	SOC_DOUBLE_R_TLV("Capture Volume", TLV320AIC23_LINVOL,
			 TLV320AIC23_RINVOL, 0, 31, 0, input_gain_tlv),
	//SOC_DOUBLE_R("Capture Switch", TLV320AIC23_LINVOL,
	//		 		     TLV320AIC23_RINVOL, 7, 1, 0),
	SOC_ENUM("De-emphasis Capture", tlv320aic23_deemph),
	SOC_ENUM("DACM Playback Switch", tlv320aic23_playback_switch),
	SOC_ENUM("Left Capture Switch", tlv320aic23_left_capture_switch),
	SOC_ENUM("Right Capture Switch", tlv320aic23_right_capture_switch),
	SOC_ENUM("PwrMic Capture Switch", tlv320aic23_mic_input_switch),
	SOC_ENUM("PwrLine Capture Switch",tlv320aic23_line_input_switch),
	SOC_ENUM("MICM Capture Switch",tlv320aic23_micm_input_switch),
	SOC_ENUM("MICB Capture Switch",tlv320aic23_micb_input_switch),
    SOC_ENUM("ADCHP Capture Switch",tlv320aic23_adchp_input_switch),
    SOC_ENUM("PwrADC Capture Switch",tlv320aic23_adc_input_switch),


	SOC_DOUBLE_R("Simult.Update Playback Switch", TLV320AIC23_LINVOL,
			TLV320AIC23_RINVOL, 8, 1, 1),
};


static const struct soc_enum tlv320aic23_line_bypass =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 3, 2, switch_off_on);

static const struct soc_enum tlv320aic23_mic_sidetone =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 5, 2, switch_off_on);

static const struct soc_enum tlv320aic23_playback =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 4, 2, switch_off_on);



/* PGA Mixer controls for Line and Mic switch */
static const struct snd_kcontrol_new tlv320aic23_output_mixer_controls[] = {

#ifndef AM
	SOC_DAPM_SINGLE("Line Bypass Switch", TLV320AIC23_ANLG, 3, 1, 0),
	SOC_DAPM_SINGLE("Mic Sidetone Switch", TLV320AIC23_ANLG, 5, 1, 0),
	SOC_DAPM_SINGLE("Playback Switch", TLV320AIC23_ANLG, 4, 1, 0),
#else
	SOC_DAPM_ENUM("Line Bypass Switch", tlv320aic23_line_bypass),
	SOC_DAPM_ENUM("Mic Sidetone Switch", tlv320aic23_mic_sidetone),
	SOC_DAPM_ENUM("Playback Switch", tlv320aic23_playback),
#endif
};


static const struct snd_soc_dapm_widget tlv320aic23_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", TLV320AIC23_PWR, 3, 1),
	SND_SOC_DAPM_ADC("ADC", "Capture", TLV320AIC23_PWR, 2, 1),

	SND_SOC_DAPM_MUX("Input Mux", TLV320AIC23_PWR, 0, 1,
			 &tlv320aic23_rec_src_mux_controls),


#ifndef AM
	SND_SOC_DAPM_MIXER("Output Mixer", TLV320AIC23_PWR, 4, 1,
			   &tlv320aic23_output_mixer_controls[0],
			   ARRAY_SIZE(tlv320aic23_output_mixer_controls)),
#endif

    SND_SOC_DAPM_MIXER_NAMED_CTL("Output Mixer", TLV320AIC23_PWR, 4, 1,
			   &tlv320aic23_output_mixer_controls[0],
			   ARRAY_SIZE(tlv320aic23_output_mixer_controls)),

	SND_SOC_DAPM_PGA("Line Input", TLV320AIC23_PWR, 0, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Mic Input", TLV320AIC23_PWR, 1, 1, NULL, 0),

	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),

	SND_SOC_DAPM_INPUT("LLINEIN"),
	SND_SOC_DAPM_INPUT("RLINEIN"),

	SND_SOC_DAPM_INPUT("MICIN"),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* Output Mixer */
	{"Output Mixer", "Line Bypass Switch", "Line Input"},
	{"Output Mixer", "Playback Switch", "DAC"},
	{"Output Mixer", "Mic Sidetone Switch", "Mic Input"},

	/* Outputs */
	{"RHPOUT", NULL, "Output Mixer"},
	{"LHPOUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},

	/* Inputs */
	{"Line Input", "NULL", "LLINEIN"},
	{"Line Input", "NULL", "RLINEIN"},
	{"Mic Input", "NULL", "MICIN"},

	/* input mux */
	{"Input Mux", "Line", "Line Input"},
	{"Input Mux", "Mic", "Mic Input"},
	{"ADC", NULL, "Input Mux"},

};

/* AIC23 driver data */
struct aic23 {
	struct snd_soc_codec codec;
	int mclk;
	int requested_adc;
	int requested_dac;
};

/*
 * Common Crystals used
 * 11.2896 Mhz /128 = *88.2k  /192 = 58.8k
 * 12.0000 Mhz /125 = *96k    /136 = 88.235K
 * 12.2880 Mhz /128 = *96k    /192 = 64k
 * 16.9344 Mhz /128 = 132.3k /192 = *88.2k
 * 18.4320 Mhz /128 = 144k   /192 = *96k
 */

/*
 * Normal BOSR 0-256/2 = 128, 1-384/2 = 192
 * USB BOSR 0-250/2 = 125, 1-272/2 = 136
 */
static const int bosr_usb_divisor_table[] = {
	128, 125, 192, 136
};
#define LOWER_GROUP ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<6) | (1<<7))
#define UPPER_GROUP ((1<<8) | (1<<9) | (1<<10) | (1<<11)        | (1<<15))
static const unsigned short sr_valid_mask[] = {
	LOWER_GROUP|UPPER_GROUP,	/* Normal, bosr - 0*/
	LOWER_GROUP,			/* Usb, bosr - 0*/
	LOWER_GROUP|UPPER_GROUP,	/* Normal, bosr - 1*/
	UPPER_GROUP,			/* Usb, bosr - 1*/
};
/*
 * Every divisor is a factor of 11*12
 */
#define SR_MULT (11*12)
#define A(x) (SR_MULT/x)
static const unsigned char sr_adc_mult_table[] = {
	A(2), A(2), A(12), A(12),  0, 0, A(3), A(1),
	A(2), A(2), A(11), A(11),  0, 0, 0, A(1)
};
static const unsigned char sr_dac_mult_table[] = {
	A(2), A(12), A(2), A(12),  0, 0, A(3), A(1),
	A(2), A(11), A(2), A(11),  0, 0, 0, A(1)
};

static unsigned get_score(int adc, int adc_l, int adc_h, int need_adc,
		int dac, int dac_l, int dac_h, int need_dac)
{
	if ((adc >= adc_l) && (adc <= adc_h) &&
			(dac >= dac_l) && (dac <= dac_h)) {
		int diff_adc = need_adc - adc;
		int diff_dac = need_dac - dac;
		return abs(diff_adc) + abs(diff_dac);
	}
	return UINT_MAX;
}

static int find_rate(int mclk, u32 need_adc, u32 need_dac)
{
	int i, j;
	int best_i = -1;
	int best_j = -1;
	int best_div = 0;
	unsigned best_score = UINT_MAX;
	int adc_l, adc_h, dac_l, dac_h;

	need_adc *= SR_MULT;
	need_dac *= SR_MULT;
	/*
	 * rates given are +/- 1/32
	 */
	adc_l = need_adc - (need_adc >> 5);
	adc_h = need_adc + (need_adc >> 5);
	dac_l = need_dac - (need_dac >> 5);
	dac_h = need_dac + (need_dac >> 5);
	for (i = 0; i < ARRAY_SIZE(bosr_usb_divisor_table); i++) {
		int base = mclk / bosr_usb_divisor_table[i];
		int mask = sr_valid_mask[i];
		for (j = 0; j < ARRAY_SIZE(sr_adc_mult_table);
				j++, mask >>= 1) {
			int adc;
			int dac;
			int score;
			if ((mask & 1) == 0)
				continue;
			adc = base * sr_adc_mult_table[j];
			dac = base * sr_dac_mult_table[j];
			score = get_score(adc, adc_l, adc_h, need_adc,
					dac, dac_l, dac_h, need_dac);
			if (best_score > score) {
				best_score = score;
				best_i = i;
				best_j = j;
				best_div = 0;
			}
			score = get_score((adc >> 1), adc_l, adc_h, need_adc,
					(dac >> 1), dac_l, dac_h, need_dac);
			/* prefer to have a /2 */
			if ((score != UINT_MAX) && (best_score >= score)) {
				best_score = score;
				best_i = i;
				best_j = j;
				best_div = 1;
			}
		}
	}
	return (best_j << 2) | best_i | (best_div << TLV320AIC23_CLKIN_SHIFT);
}

#ifdef DEBUG
static void get_current_sample_rates(struct snd_soc_codec *codec, int mclk,
		u32 *sample_rate_adc, u32 *sample_rate_dac)
{
	int src = tlv320aic23_read_reg_cache(codec, TLV320AIC23_SRATE);
	int sr = (src >> 2) & 0x0f;
	int val = (mclk / bosr_usb_divisor_table[src & 3]);
	int adc = (val * sr_adc_mult_table[sr]) / SR_MULT;
	int dac = (val * sr_dac_mult_table[sr]) / SR_MULT;
	if (src & TLV320AIC23_CLKIN_HALF) {
		adc >>= 1;
		dac >>= 1;
	}
	*sample_rate_adc = adc;
	*sample_rate_dac = dac;
}
#endif

static int set_sample_rate_control(struct snd_soc_codec *codec, int mclk,
		u32 sample_rate_adc, u32 sample_rate_dac)
{
	/* Search for the right sample rate */
	int data = find_rate(mclk, sample_rate_adc, sample_rate_dac);
	if (data < 0) {
		printk(KERN_ERR "%s:Invalid rate %u,%u requested\n",
				__func__, sample_rate_adc, sample_rate_dac);
		return -EINVAL;
	}
	tlv320aic23_write(codec, TLV320AIC23_SRATE, data);
#ifdef DEBUG
	{
		u32 adc, dac;
		get_current_sample_rates(codec, mclk, &adc, &dac);
		printk(KERN_DEBUG "actual samplerate = %u,%u reg=%x\n",
			adc, dac, data);
	}
#endif
	return 0;
}

static int tlv320aic23_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, tlv320aic23_dapm_widgets,
				  ARRAY_SIZE(tlv320aic23_dapm_widgets));

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int tlv320aic23_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u16 iface_reg;
	int ret;
	struct aic23 *aic23 = container_of(codec, struct aic23, codec);
	u32 sample_rate_adc = aic23->requested_adc;
	u32 sample_rate_dac = aic23->requested_dac;
	u32 sample_rate = params_rate(params);

	//pr_info("tlv320aic23_hw_params\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aic23->requested_dac = sample_rate_dac = sample_rate;
		if (!sample_rate_adc)
			sample_rate_adc = sample_rate;
	} else {
		aic23->requested_adc = sample_rate_adc = sample_rate;
		if (!sample_rate_dac)
			sample_rate_dac = sample_rate;
	}
	ret = set_sample_rate_control(codec, aic23->mclk, sample_rate_adc,
			sample_rate_dac);
	if (ret < 0)
		return ret;

	iface_reg =
	    tlv320aic23_read_reg_cache(codec,
				       TLV320AIC23_DIGT_FMT) & ~(0x03 << 2);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface_reg |= (0x01 << 2);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface_reg |= (0x02 << 2);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface_reg |= (0x03 << 2);
		break;
	}
	tlv320aic23_write(codec, TLV320AIC23_DIGT_FMT, iface_reg);

	return 0;
}

static int tlv320aic23_pcm_prepare(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	pr_info("tlv320aic23_pcm_prepare\n");

	/* set active */
	// DJS - avoid rewriting the register if its already set to the desired value
	// this fixes an issue where occasional glitches were occurring
	//if(tlv320aic23_read_reg_cache(codec, TLV320AIC23_ACTIVE) != 0x0001)
		tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0001);

	return 0;
}

static void tlv320aic23_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct aic23 *aic23 = container_of(codec, struct aic23, codec);

	pr_info("tlv320aic23_shutdown\n");

	/* deactivate */
	if (!codec->active) {
		udelay(50);
		tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0);
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic23->requested_dac = 0;
	else
		aic23->requested_adc = 0;
}

static int tlv320aic23_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg, oreg;

	pr_info("tlv320aic23_mute: %d\n", mute);

	oreg = reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_DIGT);
	if (mute) {
		reg |= TLV320AIC23_DACM_MUTE;
	} else {
		reg &= ~TLV320AIC23_DACM_MUTE;
	}

	// DJS - avoid rewriting the register if its already set to the desired value
	// this fixes an issue where occasional glitches were occurring
	if(reg != oreg)
		tlv320aic23_write(codec, TLV320AIC23_DIGT, reg);

	return 0;
}

static int tlv320aic23_set_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface_reg;

	pr_info("tlv320aic23_set_dai_fmt: %u\n", fmt);

	iface_reg =
	    tlv320aic23_read_reg_cache(codec, TLV320AIC23_DIGT_FMT) & (~0x03);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface_reg |= TLV320AIC23_MS_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;

	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface_reg |= TLV320AIC23_FOR_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= TLV320AIC23_LRP_ON;
	case SND_SOC_DAIFMT_DSP_B:
		iface_reg |= TLV320AIC23_FOR_DSP;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= TLV320AIC23_FOR_LJUST;
		break;
	default:
		return -EINVAL;

	}

	tlv320aic23_write(codec, TLV320AIC23_DIGT_FMT, iface_reg);

	return 0;
}

static int tlv320aic23_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				      int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic23 *aic23 = container_of(codec, struct aic23, codec);
	aic23->mclk = freq;
	return 0;
}

static int tlv320aic23_set_bias_level(struct snd_soc_codec *codec,
				      enum snd_soc_bias_level level)
{
	u16 reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_PWR) & 0xff7f;

	switch (level) {
	case SND_SOC_BIAS_ON:
		pr_info("tlv320aic23_set_bias_level: SND_SOC_BIAS_ON\n");
		/* vref/mid, osc on, dac unmute */
		tlv320aic23_write(codec, TLV320AIC23_PWR, reg);
		u16 pwr = reg;

#ifndef AM
		//Switch on MIC_ON |             LINE_IN_ON              | ADC_ON
		pwr &= (~TLV320AIC23_MIC_OFF) | (~TLV320AIC23_LINE_OFF) | (~TLV320AIC23_ADC_OFF);
		tlv320aic23_write(codec, TLV320AIC23_PWR, pwr);

		u16 anlg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_ANLG);
		//ACTIVATE LINE-IN
		anlg &= (~TLV320AIC23_INSEL_MIC);
		tlv320aic23_write(codec, TLV320AIC23_ANLG, anlg);

		//UNMUTE LINE-IN LEFT
		u16 linvol = tlv320aic23_read_reg_cache(codec, TLV320AIC23_LINVOL);
		linvol &= (~TLV320AIC23_LIM_MUTED);
		tlv320aic23_write(codec, TLV320AIC23_LINVOL, linvol);

		//UNMUTE LINE-IN RIGHT
		u16 rinvol = tlv320aic23_read_reg_cache(codec, TLV320AIC23_RINVOL);
		rinvol &= (~TLV320AIC23_LIM_MUTED);
		tlv320aic23_write(codec, TLV320AIC23_RINVOL, rinvol);
#endif
		break;

	case SND_SOC_BIAS_PREPARE:
		pr_info("tlv320aic23_set_bias_level: SND_SOC_BIAS_ON\n");
		break;
	case SND_SOC_BIAS_STANDBY:
		pr_info("tlv320aic23_set_bias_level: SND_SOC_BIAS_STANDBY\n");
		/* everything off except vref/vmid, */
#ifndef AM
		tlv320aic23_write(codec, TLV320AIC23_PWR, reg | \
			TLV320AIC23_CLK_OFF);
#else
		tlv320aic23_write(codec, TLV320AIC23_PWR, reg);
#endif


		break;
	case SND_SOC_BIAS_OFF:
		pr_info("tlv320aic23_set_bias_level: SND_SOC_BIAS_OFF\n");
		/* everything off, dac mute, inactive */
		tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0);
		tlv320aic23_write(codec, TLV320AIC23_PWR, 0xffff);
		break;
	}
	codec->bias_level = level;
	return 0;
}

#define AIC23_RATES	SNDRV_PCM_RATE_8000_96000
#define AIC23_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops tlv320aic23_dai_ops = {
	.prepare	= tlv320aic23_pcm_prepare,
	.hw_params	= tlv320aic23_hw_params,
	.shutdown	= tlv320aic23_shutdown,
	.digital_mute	= tlv320aic23_mute,
	.set_fmt	= tlv320aic23_set_dai_fmt,
	.set_sysclk	= tlv320aic23_set_dai_sysclk,
};

struct snd_soc_dai tlv320aic23_dai = {
	.name = "tlv320aic23",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = AIC23_RATES,
		     .formats = AIC23_FORMATS,},
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 2,
		    .channels_max = 2,
		    .rates = AIC23_RATES,
		    .formats = AIC23_FORMATS,},
	.ops = &tlv320aic23_dai_ops,
};
EXPORT_SYMBOL_GPL(tlv320aic23_dai);

static int tlv320aic23_suspend(struct platform_device *pdev,
			       pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	pr_info("tlv320aic23_suspend\n");

	tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0);
	tlv320aic23_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int tlv320aic23_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u16 reg;

	pr_info("tlv320aic23_resume\n");

	/* Sync reg_cache with the hardware */
	for (reg = 0; reg <= TLV320AIC23_ACTIVE; reg++) {
		u16 val = tlv320aic23_read_reg_cache(codec, reg);
		tlv320aic23_write(codec, reg, val);
	}

	tlv320aic23_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	tlv320aic23_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

/*
 * initialise the AIC23 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int tlv320aic23_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;
	u16 reg;

	codec->name = "tlv320aic23";
	codec->owner = THIS_MODULE;
	codec->read = tlv320aic23_read_reg_cache;
	codec->write = tlv320aic23_write;
	codec->set_bias_level = tlv320aic23_set_bias_level;
	codec->dai = &tlv320aic23_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(tlv320aic23_reg);
	codec->reg_cache =
	    kmemdup(tlv320aic23_reg, sizeof(tlv320aic23_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* Reset codec */
	tlv320aic23_write(codec, TLV320AIC23_RESET, 0);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "tlv320aic23: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	tlv320aic23_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	tlv320aic23_write(codec, TLV320AIC23_DIGT, TLV320AIC23_DEEMP_44K);

	/* Unmute input */
	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_LINVOL);
	tlv320aic23_write(codec, TLV320AIC23_LINVOL,
			  (reg & (~TLV320AIC23_LIM_MUTED)) |
			  (TLV320AIC23_LRS_ENABLED));

	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_RINVOL);
	tlv320aic23_write(codec, TLV320AIC23_RINVOL,
			  (reg & (~TLV320AIC23_LIM_MUTED)) |
			  TLV320AIC23_LRS_ENABLED);

	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_ANLG);
	tlv320aic23_write(codec, TLV320AIC23_ANLG,
			 (reg) & (~TLV320AIC23_BYPASS_ON) &
			 (~TLV320AIC23_MICM_MUTED));

	/* Default output volume */
	tlv320aic23_write(codec, TLV320AIC23_LCHNVOL,
			  (TLV320AIC23_DEFAULT_OUT_VOL &
			  TLV320AIC23_OUT_VOL_MASK)|TLV320AIC23_LRS_ON);
	tlv320aic23_write(codec, TLV320AIC23_RCHNVOL,
			  (TLV320AIC23_DEFAULT_OUT_VOL &
			  TLV320AIC23_OUT_VOL_MASK)|TLV320AIC23_LRS_ON);

	tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x1);

	snd_soc_add_controls(codec, tlv320aic23_snd_controls,
				ARRAY_SIZE(tlv320aic23_snd_controls));
	tlv320aic23_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "tlv320aic23: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}
static struct snd_soc_device *tlv320aic23_socdev;

// DJS - nicked from davinci_platform.c and modified
// allows codec registers to be read and written using sysfs
static ssize_t reg_store(struct device *cdev, struct device_attribute *attr, const char *buffer, size_t count)
{
	struct snd_soc_device *socdev = tlv320aic23_socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	char *bufv = 0;
	int addr = 0;
	int val = 0;
	int len = 0;
	char str[64];

	if (!buffer || (count == 0) || (count >= 64))
		return 0;

	strcpy(str, buffer);
	/* overwrite the '\n' */
	strcpy(str + count - 1, "\0");

	/* format: <address> [<value>]
	   if only <address> present, it is a read
	   if <address> <value>, then it is a write */
	len = strcspn(str, " ");
	addr = simple_strtoul(str, NULL, 16);

	if((addr < 0) || (addr > TLV320AIC23_RESET))
		return 0;

	if (len != count - 1) {
		bufv = str;
		strsep(&bufv, " ");
		val = simple_strtoul(bufv, NULL, 16);
	}

	if(bufv != 0) {
		pr_info("tlv320aic23 write: 0x%x = 0x%x\n", addr, val);
		tlv320aic23_write(codec, addr, val);
	} else {
		pr_info("tlv320aic23 read : 0x%x = 0x%x\n", addr, tlv320aic23_read_reg_cache(codec, addr));
	}
	return count;
}

static ssize_t reg_show(struct device *cdev, struct device_attribute *attr, char *buf)
{
	return 0;
}

struct device_attribute reg_attr = {
	.attr = {
		.name = "reg",
		.mode = S_IRWXUGO,
		.owner = THIS_MODULE,
	},
	.show = reg_show,
	.store = reg_store,
};


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 * If the i2c layer weren't so broken, we could pass this kind of data
 * around
 */
static int tlv320aic23_codec_probe(struct i2c_client *i2c,
				   const struct i2c_device_id *i2c_id)
{
	struct snd_soc_device *socdev = tlv320aic23_socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = tlv320aic23_init(socdev);
	if (ret < 0) {
		printk(KERN_ERR "tlv320aic23: failed to initialise AIC23\n");
		goto err;
	}
	return ret;

err:
	kfree(codec);
	kfree(i2c);
	return ret;
}
static int __exit tlv320aic23_i2c_remove(struct i2c_client *i2c)
{
	put_device(&i2c->dev);
	return 0;
}

static const struct i2c_device_id tlv320aic23_id[] = {
	{"tlv320aic23", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tlv320aic23_id);

static struct i2c_driver tlv320aic23_i2c_driver = {
	.driver = {
		   .name = "tlv320aic23",
		   },
	.probe = tlv320aic23_codec_probe,
	.remove = __exit_p(tlv320aic23_i2c_remove),
	.id_table = tlv320aic23_id,
};

#endif

static int tlv320aic23_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct aic23 *aic23;
	int ret = 0;

	printk(KERN_INFO "AIC23 Audio Codec %s\n", AIC23_VERSION);

	aic23 = kzalloc(sizeof(struct aic23), GFP_KERNEL);
	if (aic23 == NULL)
		return -ENOMEM;
	codec = &aic23->codec;
	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	tlv320aic23_socdev = socdev;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	codec->hw_write = (hw_write_t) i2c_master_send;
	codec->hw_read = NULL;
	ret = i2c_add_driver(&tlv320aic23_i2c_driver);
	if (ret != 0)
		printk(KERN_ERR "can't add i2c driver");
#endif

	ret = device_create_file(socdev->dev, &reg_attr);

	return ret;
}

static int tlv320aic23_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct aic23 *aic23 = container_of(codec, struct aic23, codec);

	if (codec->control_data)
		tlv320aic23_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&tlv320aic23_i2c_driver);
#endif
	kfree(codec->reg_cache);
	kfree(aic23);

	return 0;
}
struct snd_soc_codec_device soc_codec_dev_tlv320aic23 = {
	.probe = tlv320aic23_probe,
	.remove = tlv320aic23_remove,
	.suspend = tlv320aic23_suspend,
	.resume = tlv320aic23_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_tlv320aic23);

static int __init tlv320aic23_modinit(void)
{
	return snd_soc_register_dai(&tlv320aic23_dai);
}
module_init(tlv320aic23_modinit);

static void __exit tlv320aic23_exit(void)
{
	snd_soc_unregister_dai(&tlv320aic23_dai);
}
module_exit(tlv320aic23_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC23 codec driver");
MODULE_AUTHOR("Arun KS <arunks@mistralsolutions.com>");
MODULE_LICENSE("GPL");
