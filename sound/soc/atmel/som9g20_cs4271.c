/*
 * som9g20_cs4271  --  SoC audio for AT91SAM9G20-based
 * 			EMAC SOM-9G20M board.
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *  Copyright (C) 2009 EMAC Inc.
 *
 * Based on sam9g20_wm8731.c by:
 * Sedji Gaouaou <sedji.gaouaou@atmel.com>
 *
 * Based on ati_b1_cs4271.c by:
 * Frank Mandarino <fmandarino@endrelia.com>
 * Copyright 2006 Endrelia Technologies Inc.
 * Based on corgi.c by:
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
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
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/atmel-ssc.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <mach/hardware.h>
#include <mach/gpio.h>


#include "../codecs/cs4271.h"
#include "atmel-pcm.h"
#include "atmel_ssc_dai.h"


static int som9g20_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;

	/* codec system clock is supplied by PCK0, set to 12MHz */
#ifdef CONFIG_SOM9G20_CS4271_MASTER
	ret = snd_soc_dai_set_sysclk(codec_dai, CS4271_SYSCLK,
		12288000, SND_SOC_CLOCK_IN);
#else
	ret = snd_soc_dai_set_sysclk(codec_dai, CS4271_SYSCLK,
		12288000, SND_SOC_CLOCK_IN);
#endif

	if (ret < 0)
		return ret;

	return 0;
}

static void som9g20_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);

	dev_dbg(rtd->socdev->dev, "shutdown");
}

static int som9g20_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct atmel_ssc_info *ssc_p = cpu_dai->private_data;
	struct ssc_device *ssc = ssc_p->ssc;
	int ret;

	unsigned int rate;
	int cmr_div, period;

	if (ssc == NULL) {
		printk(KERN_INFO "som9g20_hw_params: ssc is NULL!\n");
		return -EINVAL;
	}

	/* set codec DAI configuration */
#ifdef CONFIG_SOM9G20_CS4271_MASTER
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM); 
#else
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif
	if (ret < 0)
		return ret;

#ifdef CONFIG_SOM9G20_CS4271_MASTER
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#else	
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif

	if (ret < 0)
		return ret;

	rate = params_rate(params);

	switch (rate) {
	case 8000:
	case 11025:
	case 16000:
	case 22050:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		cmr_div = 43;	/* BCLK = 133MHz/(2*43) =  */
		period = 15;	/* LRC = BCLK/(2*(15+1)) = 48000Hz */
		break;
	default:
		printk(KERN_WARNING "unsupported rate %d"
				" on at91sam9g20ek board\n", rate);
		return -EINVAL;
	}

	/* set the MCK divider for BCLK */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, ATMEL_SSC_CMR_DIV, cmr_div);
	if (ret < 0)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* set the BCLK divider for DACLRC */
		ret = snd_soc_dai_set_clkdiv(cpu_dai,
						ATMEL_SSC_TCMR_PERIOD, period);
	} else {
		/* set the BCLK divider for ADCLRC */
		ret = snd_soc_dai_set_clkdiv(cpu_dai,
						ATMEL_SSC_RCMR_PERIOD, period);
	}
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops som9g20_ops = {
	.startup = som9g20_startup,
	.hw_params = som9g20_hw_params,
	.shutdown = som9g20_shutdown,
};


static int som9g20_cs4271_init(struct snd_soc_codec *codec)
{
	printk(KERN_DEBUG
			"som9g20_cs4271 "
			": som9g20_cs4271_init() called\n");

	snd_soc_dapm_sync(codec);

	return 0;
}

#ifdef CONFIG_PM
static int snd_cs4271_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifndef CONFIG_SOM9G20_CS4271_MASTER
	struct clk *pck0;

	pck0 = clk_get(NULL, "pck0");

	clk_disable(pck0);
#endif
	return 0;
}

static int snd_cs4271_resume(struct platform_device *pdev)
{
#ifndef CONFIG_SOM9G20_CS4271_MASTER
	struct clk *pck0;

	pck0 = clk_get(NULL, "pck0");
	
	clk_enable(pck0);
#endif	

	return 0;
}
#else
#define snd_cs4271_suspend NULL
#define snd_cs4271_resume NULL
#endif

static struct snd_soc_dai_link som9g20_dai = {
	.name = "CS4271",
	.stream_name = "CS4271 PCM",
	.cpu_dai = &atmel_ssc_dai[0],
	.codec_dai = &cs4271_dai,
	.init = som9g20_cs4271_init,
	.ops = &som9g20_ops,
};

static struct snd_soc_machine snd_soc_machine_som9g20 = {
	.name = "CS4271",
	.dai_link = &som9g20_dai,
	.num_links = 1,

	.suspend_post = snd_cs4271_suspend,
	.resume_pre = snd_cs4271_resume,
};

static struct cs4271_setup_data som9g20_cs4271_setup = {
	.spi = 1,
};

static struct snd_soc_device som9g20_snd_devdata = {
	.machine = &snd_soc_machine_som9g20,
	.platform = &atmel_soc_platform,
	.codec_dev = &soc_codec_dev_cs4271,
	.codec_data = &som9g20_cs4271_setup,
};

static struct platform_device *som9g20_snd_device;

static int __init som9g20_init(void)
{
	struct atmel_ssc_info *ssc_p = som9g20_dai.cpu_dai->private_data;
	struct ssc_device *ssc = NULL;
	int ret;

	/*
	 * Request SSC device
	 */

	ssc = ssc_request(0);
	if (IS_ERR(ssc)) {
		ret = PTR_ERR(ssc);
		ssc = NULL;
		goto err_ssc;
	}
	ssc_p->ssc = ssc;

	som9g20_snd_device = platform_device_alloc("soc-audio", -1);
	if (!som9g20_snd_device) {
		printk(KERN_ERR
				"platform device allocation failed\n");
		ret = -ENOMEM;
	}

	platform_set_drvdata(som9g20_snd_device,
			&som9g20_snd_devdata);
	som9g20_snd_devdata.dev = &som9g20_snd_device->dev;

	ret = platform_device_add(som9g20_snd_device);
	if (ret) {
		printk(KERN_ERR
				"platform device allocation failed\n");
		platform_device_put(som9g20_snd_device);
	}

	return ret;

err_ssc:
	return ret;
}

static void __exit som9g20_exit(void)
{
	struct atmel_ssc_info *ssc_p = som9g20_dai.cpu_dai->private_data;
	struct ssc_device *ssc;

	if (ssc_p != NULL) {
		ssc = ssc_p->ssc;
		if (ssc != NULL)
			ssc_free(ssc);
		ssc_p->ssc = NULL;
	}

	platform_device_unregister(som9g20_snd_device);
	som9g20_snd_device = NULL;
}

module_init(som9g20_init);
module_exit(som9g20_exit);

/* Module information */
MODULE_AUTHOR("Michael Welling");
MODULE_DESCRIPTION("ALSA SoC SOM9G20_CS4271");
MODULE_LICENSE("GPL");
