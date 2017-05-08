/*
 *  bytcr_wm5102.c - ASoc Machine driver for Intel Byt CR platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Subhransu S. Prusty <subhransu.s.prusty@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define DEBUG
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/mfd/arizona/registers.h>
#include "../../codecs/wm5102.h"
#include "../atom/sst-atom-controls.h"
#include <asm/platform_sst_audio.h>
#include <linux/clk.h>

#define BYT_PLAT_CLK_3_HZ	25000000
#define WM5102_MAX_SYSCLK_1 49152000 /*max sysclk for 4K family*/
#define WM5102_MAX_SYSCLK_2 45158400 /*max sysclk for 11.025K family*/

// References MCLK to enable it
static struct clk *mclk;

static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route byt_audio_map[] = {
	{"Headphone", NULL, "HPOUT1L"},
	{"Headphone", NULL, "HPOUT1R"},
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},
	{"Headset Mic", NULL, "MICBIAS1"},
	{"Headset Mic", NULL, "MICBIAS2"},
	{"IN1L", NULL, "Headset Mic"},
	{"Int Mic", NULL, "MICBIAS3"},
	{"IN3L", NULL, "Int Mic"},

        {"AIF1 Playback", NULL, "ssp0 Tx"},
        {"ssp0 Tx", NULL, "codec_out0"},
        {"ssp0 Tx", NULL, "codec_out1"},
        {"codec_in0", NULL, "ssp0 Rx"},
        {"codec_in1", NULL, "ssp0 Rx"},
        {"ssp0 Rx", NULL, "AIF1 Capture"},

};

static const struct snd_kcontrol_new byt_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static int byt_aif1_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *wm5102_codec = rtd->codec;
	int ret;

	int sr = params_rate(params);
	int sr_mult = (params_rate(params) % 4000 == 0) ? (WM5102_MAX_SYSCLK_1/params_rate(params)) : (WM5102_MAX_SYSCLK_2/params_rate(params));

	/*Open MCLK before Set DAI CLK*/
	ret = clk_prepare_enable(mclk);
	if (!ret)
		clk_disable_unprepare(mclk);
	ret = clk_set_rate(mclk, BYT_PLAT_CLK_3_HZ);

	if (ret)
		dev_err(wm5102_codec->dev, "unable to set MCLK rate\n");

	ret = clk_prepare_enable(mclk);

	if (ret < 0) {
		dev_err(wm5102_codec->dev, "could not configure MCLK state");
		return ret;
	}

	ret = snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1_REFCLK,
				  ARIZONA_FLL_SRC_NONE, 0, 0);

	ret = snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1,
				  ARIZONA_FLL_SRC_NONE, 0, 0);

	ret = snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1, ARIZONA_CLK_SRC_MCLK1,
				  BYT_PLAT_CLK_3_HZ,
				  sr * sr_mult);
	if (ret < 0) {
		dev_err(wm5102_codec->dev, "Failed to enable FLL1 with Ref Clock Loop: %d\n", ret);
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
			ARIZONA_CLK_SYSCLK,
			ARIZONA_CLK_SRC_FLL1,
			sr * sr_mult,
			SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to set AYNCCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
					ARIZONA_CLK_SYSCLK, 0,
					sr * sr_mult,
					SND_SOC_CLOCK_OUT);
        if (ret < 0) {
                dev_err(rtd->dev, "can't set OPCLK %d\n", ret);
        }

	return 0;
}

static const struct snd_soc_pcm_stream byt_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static int byt_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP0 to 16-bit */
	params_set_format(params, SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static int byt_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
			SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static struct snd_soc_ops byt_aif1_ops = {
	.startup = byt_aif1_startup,
};

static struct snd_soc_ops byt_be_ssp2_ops = {
	.hw_params = byt_aif1_hw_params,
};

static struct snd_soc_dai_link byt_dailink[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Baytrail Audio Port",
		.stream_name = "Baytrail Audio",
		.cpu_dai_name = "media-cpu-dai",
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-mfld-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &byt_aif1_ops,
	},
	[MERR_DPCM_COMPR] = {
		.name = "Baytrail Compressed Port",
		.stream_name = "Baytrail Compress",
		.cpu_dai_name = "compress-cpu-dai",
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-mfld-platform",
	},
		/* back ends */
	{
		.name = "SSP0-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-mfld-platform",
		.no_pcm = 1,
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
						| SND_SOC_DAIFMT_CBS_CFS,
		.be_hw_params_fixup = byt_codec_fixup,
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &byt_be_ssp2_ops,
	},
};


static int snd_byt_mc_late_probe(struct snd_soc_card *card)
{
	int ret;

	ret = snd_soc_dai_set_sysclk(card->rtd[0].codec_dai,  ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret != 0) {
		dev_err(card->rtd[0].codec->dev, "Failed to set codec dai clk domain: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(card->rtd[1].codec_dai, ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret != 0) {
		dev_err(card->rtd[0].codec->dev, "Failed to set codec dai clk domain: %d\n", ret);
		return ret;
	}

	/*Configure SAMPLE_RATE_1 by default to
	48KHz this value can be changed in runtime by corresponding
	DAI hw_params callback */
	snd_soc_update_bits(card->rtd[0].codec, ARIZONA_SAMPLE_RATE_1,
		ARIZONA_SAMPLE_RATE_1_MASK, 0x03);

	return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_byt = {
	.name = "baytrailcraudio",
	.owner = THIS_MODULE,
	.dai_link = byt_dailink,
	.num_links = ARRAY_SIZE(byt_dailink),
	.late_probe = snd_byt_mc_late_probe,
	.dapm_widgets = byt_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(byt_dapm_widgets),
	.dapm_routes = byt_audio_map,
	.num_dapm_routes = ARRAY_SIZE(byt_audio_map),
	.controls = byt_mc_controls,
	.num_controls = ARRAY_SIZE(byt_mc_controls),
};

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;

	/* register the soc card */
	snd_soc_card_byt.dev = &pdev->dev;

	ret_val = devm_snd_soc_register_card(&pdev->dev, &snd_soc_card_byt);
	if (ret_val) {
		dev_err(&pdev->dev, "devm_snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}

	// Try to reference MCLK
	// based on: https://www.spinics.net/lists/stable/msg167030.html
	mclk = devm_clk_get(&pdev->dev, "pmc_plt_clk_3");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev,
			"Failed to get MCLK from pmc_plt_clk_3: %ld\n",
			PTR_ERR(mclk));
		return PTR_ERR(mclk);
	}

	platform_set_drvdata(pdev, &snd_soc_card_byt);
	return ret_val;
}

static struct platform_driver snd_byt_mc_driver = {
	.driver = {
		.name = "bytwm5102-audio",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_byt_mc_probe,
};

module_platform_driver(snd_byt_mc_driver);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail CR Machine driver");
MODULE_AUTHOR("Subhransu S. Prusty <subhransu.s.prusty@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bytwm5102-audio");
