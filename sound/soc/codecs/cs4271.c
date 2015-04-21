/*
 * cs4271.c  --  CS4271 ALSA SoC Audio driver
 *
 * Copyright 2009 EMAC Inc.
 *
 * Based on wm8753.c by Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "cs4271.h"

#define CS4271_VERSION "0.00"

struct snd_soc_codec_device soc_codec_dev_cs4271;

/* codec private data */
struct cs4271_priv {
	unsigned int sysclk;
};

/*
 * cs4271 register cache
 */
static u16 cs4271_reg[CS4271_CACHEREGNUM] =
{
	0x00,	/* 00 - NONE      */
	0x00,	/* 01 - MODE1     */
	0x80,	/* 02 - DAC_CTRL  */
	0x29,	/* 03 - VOL_MIX   */
	0x00,	/* 04 - VOL_CHANA */
	0x00,	/* 05 - VOL_CHANB */
	0x00,	/* 06 - ADC_CTRL  */
	0x00,	/* 07 - MODE2     */
	0x00,	/* 08 - DEVICE    */
};

/*
 * read cs4271 register cache
 */
static inline unsigned int cs4271_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= CS4271_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write cs4271 register cache
 */
static inline void cs4271_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= CS4271_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the CS4271 register space
 */
static int cs4271_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	data[0] = reg;
	data[1] = value & 0xFF;

	cs4271_write_reg_cache(codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

static const struct snd_kcontrol_new cs4271_snd_controls[] = {

SOC_DOUBLE_R("Master Playback Volume", CS4271_DAC_VOL_CHANA_REG, CS4271_DAC_VOL_CHANB_REG,
	0, 127, 1),

};

/* add non dapm controls */
static int cs4271_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(cs4271_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&cs4271_snd_controls[i],
						codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:4;
	u8 bosr:1;
	u8 usb:1;
};

/* codec mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0, 0x0},
	{18432000, 48000, 384, 0x0, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x0, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0x6, 0x0, 0x0},
	{18432000, 32000, 576, 0x6, 0x1, 0x0},
	{12000000, 32000, 375, 0x6, 0x0, 0x1},

	/* 8k */
	{12288000, 8000, 1536, 0x3, 0x0, 0x0},
	{18432000, 8000, 2304, 0x3, 0x1, 0x0},
	{11289600, 8000, 1408, 0xb, 0x0, 0x0},
	{16934400, 8000, 2112, 0xb, 0x1, 0x0},
	{12000000, 8000, 1500, 0x3, 0x0, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x7, 0x0, 0x0},
	{18432000, 96000, 192, 0x7, 0x1, 0x0},
	{12000000, 96000, 125, 0x7, 0x0, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x8, 0x0, 0x0},
	{16934400, 44100, 384, 0x8, 0x1, 0x0},
	{12000000, 44100, 272, 0x8, 0x1, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0xf, 0x0, 0x0},
	{16934400, 88200, 192, 0xf, 0x1, 0x0},
	{12000000, 88200, 136, 0xf, 0x1, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}
	return 0;
}

static int cs4271_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct cs4271_priv *cs4271 = codec->private_data;
	u16 iface = 0; //cs4271_read_reg_cache(codec, CS4271_IFACE) & 0xfff3;
	int i = get_coeff(cs4271->sysclk, params_rate(params));
	u16 srate = (coeff_div[i].sr << 2) |
		(coeff_div[i].bosr << 1) | coeff_div[i].usb;

	//cs4271_write(codec, CS4271_SRATE, srate);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	}

//	cs4271_write(codec, CS4271_IFACE, iface);
#endif
	return 0;
}

static int cs4271_pcm_prepare(struct snd_pcm_substream *substream)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	/* set active */
	cs4271_write(codec, CS4271_ACTIVE, 0x0001);
#endif

	return 0;
}

static void cs4271_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	/* deactivate */
	if (!codec->active) {
		udelay(50);
	//	cs4271_write(codec, CS4271_ACTIVE, 0x0);
	}
}

static int cs4271_mute(struct snd_soc_dai *dai, int mute)
{
#if 0
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = 0; //cs4271_read_reg_cache(codec, CS4271_APDIGI) & 0xfff7;

/*	if (mute)
		cs4271_write(codec, CS4271_APDIGI, mute_reg | 0x8);
	else
		cs4271_write(codec, CS4271_APDIGI, mute_reg);*/
#endif
	return 0;
}

static int cs4271_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4271_priv *cs4271 = codec->private_data;

	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
		cs4271->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}


static int cs4271_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	//cs4271_write(codec, CS4271_IFACE, iface);
	return 0;
}

static int cs4271_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	//u16 reg = 0;//cs4271_read_reg_cache(codec, CS4271_PWR) & 0xff7f;

	switch (level) {
	case SND_SOC_BIAS_ON:
		/* vref/mid, osc on, dac unmute */
		//cs4271_write(codec, CS4271_PWR, reg);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* everything off except vref/vmid, */
		//cs4271_write(codec, CS4271_PWR, reg | 0x0040);
		break;
	case SND_SOC_BIAS_OFF:
		/* everything off, dac mute, inactive */
		//cs4271_write(codec, CS4271_ACTIVE, 0x0);
		//cs4271_write(codec, CS4271_PWR, 0xffff);
		break;
	}
	codec->bias_level = level;
	return 0;
}

#define CS4271_RATES (SNDRV_PCM_RATE_8000 |\
		SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_32000 |\
		SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_96000)


#define CS4271_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_dai cs4271_dai = {
	.name = "CS4271",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CS4271_RATES,
		.formats = CS4271_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CS4271_RATES,
		.formats = CS4271_FORMATS,},
	.ops = {
		.prepare = cs4271_pcm_prepare,
		.hw_params = cs4271_hw_params,
		.shutdown = cs4271_shutdown,
	},
	.dai_ops = {
		.digital_mute = cs4271_mute,
		.set_sysclk = cs4271_set_dai_sysclk,
		.set_fmt = cs4271_set_dai_fmt,
	}
};
EXPORT_SYMBOL_GPL(cs4271_dai);

static int cs4271_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	//cs4271_write(codec, CS4271_ACTIVE, 0x0);
	//cs4271_set_bias_level(codec, SND_SOC_BIAS_OFF);
#endif
	return 0;
}

static int cs4271_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(cs4271_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		//codec->hw_write(codec->control_data, data, 2);
	}
	//cs4271_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	//cs4271_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

#ifdef CONFIG_RDAC_CARRIER
#define PLD_WRITE(index,data) *(volatile u8 *)(pld_addr+index)=(u8)data
#define PLD_READ(index) *(volatile u8 *)(pld_addr+index)

#define PLDBASE 	0x10000000

u8 *pld_addr;
#endif

/*
 * initialise the CS4271 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int cs4271_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	codec->name = "CS4271";
	codec->owner = THIS_MODULE;
	codec->read = cs4271_read_reg_cache;
	codec->write = cs4271_write;
	codec->set_bias_level = cs4271_set_bias_level;
	codec->dai = &cs4271_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(cs4271_reg);
	codec->reg_cache = kmemdup(cs4271_reg, sizeof(cs4271_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "cs4271: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	//cs4271_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
#ifdef CONFIG_RDAC_CARRIER	
	pld_addr = ioremap_nocache(PLDBASE,0xF);

	PLD_WRITE(0xA, (PLD_READ(0xA)) | 0x08);
#endif
	
	cs4271_write(codec, CS4271_MODE2_REG, 0x03);
#ifdef CONFIG_SOM9G20_CS4271_MASTER
	cs4271_write(codec, CS4271_MODE1_REG, 0x09);
#else
	cs4271_write(codec, CS4271_MODE1_REG, 0x01);
#endif	
	cs4271_write(codec, CS4271_DAC_VOL_CHANA_REG, 0x00);
	cs4271_write(codec, CS4271_DAC_VOL_CHANB_REG, 0x00);
	cs4271_write(codec, CS4271_ADC_CTRL_REG, 0x00);
	cs4271_write(codec, CS4271_DAC_CTRL_REG, 0x20);
	cs4271_write(codec, CS4271_MODE2_REG, 0x02);

	cs4271_add_controls(codec);
	
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "cs4271: failed to register card\n");
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

static struct snd_soc_device *cs4271_socdev;

static int __devinit cs4271_spi_probe(struct spi_device *spi)
{
	struct snd_soc_device *socdev = cs4271_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;

	codec->control_data = spi;

	ret = cs4271_init(socdev);
	if (ret < 0)
		dev_err(&spi->dev, "failed to initialise CS4271\n");

	return ret;
}

static int __devexit cs4271_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver cs4271_spi_driver = {
	.driver = {
		.name	= "cs4271",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= cs4271_spi_probe,
	.remove		= __devexit_p(cs4271_spi_remove),
};

static int cs4271_spi_write(struct spi_device *spi, const char *data, int len)
{
	struct spi_transfer t;
	struct spi_message m;
	u8 msg[3];

	if (len <= 0)
		return 0;

	msg[0] = 0x20;
	msg[1] = data[0];
	msg[2] = data[1];

	spi_message_init(&m);
	memset(&t, 0, (sizeof t));

	t.tx_buf = &msg[0];
	t.len = len + 1;

	spi_message_add_tail(&t, &m);
	spi_sync(spi, &m);

	udelay(10);

	return len;
}

static int cs4271_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct cs4271_setup_data *setup;
	struct snd_soc_codec *codec;
	struct cs4271_priv *cs4271;
	int ret = 0;

	pr_info("CS4271 Audio Codec %s", CS4271_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	cs4271 = kzalloc(sizeof(struct cs4271_priv), GFP_KERNEL);
	if (cs4271 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = cs4271;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	cs4271_socdev = socdev;
	ret = -ENODEV;

	if (setup->spi) {
		codec->hw_write = (hw_write_t)cs4271_spi_write;
		ret = spi_register_driver(&cs4271_spi_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add spi driver");
	}

	if (ret != 0) {
		kfree(codec->private_data);
		kfree(codec);
	}
	return ret;
}

/* power down chip */
static int cs4271_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		cs4271_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	spi_unregister_driver(&cs4271_spi_driver);
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_cs4271 = {
	.probe = 	cs4271_probe,
	.remove = 	cs4271_remove,
	.suspend = 	cs4271_suspend,
	.resume =	cs4271_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_cs4271);

MODULE_DESCRIPTION("ASoC CS4271 driver");
MODULE_AUTHOR("Michael Welling");
MODULE_LICENSE("GPL");
