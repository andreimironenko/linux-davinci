/*
 * ASoC driver for TI DAVINCI HTC platform
 *
 * Author:      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include <mach/asp.h>
#include <mach/edma.h>
#include <mach/mux.h>

#include "../codecs/tlv320aic23.h"
#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-mcasp.h"

#define AM

// DSP B, codec is clock master, recv clk falling edge CLKR + xmit clk rising edge CLKX
//#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)
// DSP B, cpu is clock master, recv clk falling edge CLKR + xmit clk rising edge CLKX
//#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_IB_NF)

//David's changes
// DSP A, codec is clock master, recv clk falling edge CLKR + xmit clk rising edge CLKX
//#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)
//#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_IF)

//#define AUDIO_FORMAT (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM| SND_SOC_DAIFMT_NB_NF)
//#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_NB_NF)

//Tested:
//null #define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_NB_NF)
//null#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_IF)
//null#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_NB_IF)
#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)

static int htc_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	unsigned sysclk;

	//sysclk = 27000000;
	sysclk = 12000000;
	//sysclk = 18432000;
	//sysclk = 12288000;
	//sysclk = 24576000;
	//sysclk = 1000000;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	//David's change
	//ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops htc_ops = {
	.hw_params = htc_hw_params,
};

#ifndef AM
/* davinci-htc machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};
#endif

/* davinci-htc machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {

#ifndef AM
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
#endif

	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),

};

/* davinci-htc machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {

		{ "Headphone Jack", NULL, "LHPOUT"},
		{ "Headphone Jack", NULL, "RHPOUT"},

		{ "Line Out", NULL, "LOUT" },
		{ "Line Out", NULL, "ROUT" },

		{ "LLINEIN", NULL, "Line In"},
		{ "RLINEIN", NULL, "Line In"},

		{ "MICIN", NULL, "Mic Jack"},
#ifndef AM
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},


	/* Lineout connected to HPLOUT, HPROUT */
	{"Line out", NULL, "LOUT"},
	{"Line out", NULL, "ROUT"},

	/* Davinci line in is connected to LLINEIN/RLINEIN*/
	{"LLINEIN", NULL, "Line In"},
	{"RLINEIN", NULL, "Line In"}
#endif


};

/* Logic for a aic3x as connected on a davinci-htc */
static int htc_aic23_init(struct snd_soc_codec *codec)
{

	/* Add davinci-htc specific widgets */
	snd_soc_dapm_new_controls(codec, aic3x_dapm_widgets,
				  ARRAY_SIZE(aic3x_dapm_widgets));

	/* Set up davinci-htc specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

#ifndef AM
	/* not connected */
	//snd_soc_dapm_disable_pin(codec, "MONO_LOUT");
	//snd_soc_dapm_disable_pin(codec, "HPLCOM");
	//snd_soc_dapm_disable_pin(codec, "HPRCOM");

	/* always connected */
	//snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Line Out");
	//snd_soc_dapm_enable_pin(codec, "Mic Jack");
	snd_soc_dapm_enable_pin(codec, "Line In");
#endif

	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Line In");
	snd_soc_dapm_enable_pin(codec, "Line Out");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");

	snd_soc_dapm_sync(codec);
	return 0;
}

/* davinci-htc digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link htc_dai = {
	.name = "TLV320AIC23",
	.stream_name = "TLV320",
	.cpu_dai = &davinci_i2s_dai,
	.codec_dai = &tlv320aic23_dai,
	.init = htc_aic23_init,
	.ops = &htc_ops,
};

/* davinci dm6446, dm355 or dm365 htc audio machine driver */
static struct snd_soc_card snd_soc_card_htc = {
	.name = "HTC Audio",
	.platform = &davinci_soc_platform,
	.dai_link = &htc_dai,
	.num_links = 1,
};

//static struct aic3x_setup_data aic3x_setup;

/* htc audio subsystem */
static struct snd_soc_device htc_snd_devdata = {
	.card = &snd_soc_card_htc,
	.codec_dev = &soc_codec_dev_tlv320aic23,
	//.codec_data = &aic3x_setup,
};

static struct platform_device *htc_snd_device;

static int __init htc_init(void)
{
	struct snd_soc_device *htc_snd_dev_data;
	int index;
	int ret;

	htc_snd_dev_data = &htc_snd_devdata;
	index = 0;

	//davinci_i2s_dai.dev = &htc_snd_device->dev;
	//tlv320aic23_dai.dev = &htc_snd_device->dev;

	htc_snd_device = platform_device_alloc("soc-audio", index);
	if (!htc_snd_device)
		return -ENOMEM;

	platform_set_drvdata(htc_snd_device, htc_snd_dev_data);
	htc_snd_dev_data->dev = &htc_snd_device->dev;
	ret = platform_device_add(htc_snd_device);
	if (ret)
		platform_device_put(htc_snd_device);

	return ret;
}

static void __exit htc_exit(void)
{
	platform_device_unregister(htc_snd_device);
}

module_init(htc_init);
module_exit(htc_exit);

MODULE_AUTHOR("David Steinberg");
MODULE_DESCRIPTION("Hanover HTC ASoC driver");
MODULE_LICENSE("GPL");
