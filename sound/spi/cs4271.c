/*
 * Driver for CS4271 24-bit stereo Codec connected to Atmel SSC
 *
 * Copyright (C) 2006-2007 Atmel Norway
 * Copyright (C) 2008 EMAC Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <sound/initval.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/pcm.h>

#include <linux/atmel-ssc.h>

#include <linux/spi/spi.h>
#include <linux/spi/cs4271.h>

#include "cs4271.h"

#define BITRATE_MIN	 8000 /* Hardware limit? */
#define BITRATE_TARGET	CONFIG_SND_CS4271_TARGET_BITRATE
#define BITRATE_MAX	50000 /* Hardware limit. */

/* Initial (hardware reset) CS4271 register values. */
static u8 snd_cs4271_original_image[9] =
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

struct snd_cs4271 {
	struct snd_card			*card;
	struct snd_pcm			*pcm;
	struct snd_pcm_substream	*substream;
	struct cs4271_board_info	*board;
	int				irq;
	int				period;
	unsigned long			bitrate;
	struct clk			*bitclk;
	struct ssc_device		*ssc;
	struct spi_device		*spi;
	u8				spi_wbuffer[3];
	u8				spi_rbuffer[3];
	/* Image of the SPI registers in CS4271. */
	u8				reg_image[9];
	/* Protect SSC registers against concurrent access. */
	spinlock_t			lock;
	/* Protect mixer registers against concurrent access. */
	struct mutex			mixer_lock;
};

#define get_chip(card) ((struct snd_cs4271 *)card->private_data)

static int
snd_cs4271_write_reg(struct snd_cs4271 *chip, u8 reg, u8 val)
{
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.len		= 3,
		.cs_change	= 0,
	};
	int retval;

	spi_message_init(&msg);

	chip->spi_wbuffer[0] = 0x20;
	chip->spi_wbuffer[1] = reg;
	chip->spi_wbuffer[2] = val;

	msg_xfer.tx_buf = chip->spi_wbuffer;
	msg_xfer.rx_buf = chip->spi_rbuffer;
	spi_message_add_tail(&msg_xfer, &msg);

	retval = spi_sync(chip->spi, &msg);

	if (!retval)
		chip->reg_image[reg] = val;

	udelay(10);

	return retval;
}

static struct snd_pcm_hardware snd_cs4271_capture_hw = {
	.info		= SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_BLOCK_TRANSFER,
#ifdef __BIG_ENDIAN
	.formats	= SNDRV_PCM_FMTBIT_S16_BE,
#else
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
#endif
	.rates		= (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			   SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
			   SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
			   SNDRV_PCM_RATE_KNOT),
	.rate_min	= 8000,  /* Replaced by chip->bitrate later. */
	.rate_max	= 48000, /* Replaced by chip->bitrate later. */
	.channels_min	= 1,
	.channels_max	= 2,
	.buffer_bytes_max = 64 * 1024 - 1,
	.period_bytes_min = 512,
	.period_bytes_max = 64 * 1024 - 1,
	.periods_min	= 4,
	.periods_max	= 1024,
};

static struct snd_pcm_hardware snd_cs4271_playback_hw = {
	.info		= SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_BLOCK_TRANSFER,
#ifdef __BIG_ENDIAN
	.formats	= SNDRV_PCM_FMTBIT_S16_BE,
#else
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
#endif
	.rates		= (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			   SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
			   SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
			   SNDRV_PCM_RATE_KNOT),
	.rate_min	= 8000,  /* Replaced by chip->bitrate later. */
	.rate_max	= 48000, /* Replaced by chip->bitrate later. */
	.channels_min	= 1,
	.channels_max	= 2,
	.buffer_bytes_max = 64 * 1024 - 1,
	.period_bytes_min = 512,
	.period_bytes_max = 64 * 1024 - 1,
	.periods_min	= 4,
	.periods_max	= 1024,
};

/*
 * Calculate and set bitrate and divisions.
 */
static int snd_cs4271_set_bitrate(struct snd_cs4271 *chip)
{
	unsigned long ssc_rate = clk_get_rate(chip->ssc->clk);
	unsigned long dac_rate_new, ssc_div;
	int status;
	unsigned long ssc_div_max, ssc_div_min;
	int max_tries;

	/*
	 * We connect two clocks here, picking divisors so the I2S clocks
	 * out data at the same rate the DAC clocks it in ... and as close
	 * as practical to the desired target rate.
	 *
	 * The DAC master clock (MCLK) is programmable, and is either 256
	 * or (not here) 384 times the I2S output clock (BCLK).
	 */

	/* SSC clock / (bitrate * stereo * 16-bit). */

	ssc_div = ssc_rate / (BITRATE_TARGET * 2 * 16);
	ssc_div_min = ssc_rate / (BITRATE_MAX * 2 * 16);
	ssc_div_max = ssc_rate / (BITRATE_MIN * 2 * 16);
	max_tries = (ssc_div_max - ssc_div_min) / 2;

	if (max_tries < 1)
		max_tries = 1;

	/* ssc_div must be a power of 2. */
	ssc_div = (ssc_div + 1) & ~1UL;
	
	if ((ssc_rate / (ssc_div * 2 * 16)) < BITRATE_MIN) {
		ssc_div -= 2;
		if ((ssc_rate / (ssc_div * 2 * 16)) > BITRATE_MAX)
			return -ENXIO;
	}

	/* Search for a possible bitrate. */
	
	do {
		/* SSC clock / (ssc divider * 16-bit * stereo). */
		if ((ssc_rate / (ssc_div * 2 * 16)) < BITRATE_MIN)
			return -ENXIO;

		/* 256 / (2 * 16) = 8 */
		dac_rate_new = 8 * (ssc_rate / ssc_div);

		status = clk_round_rate(chip->board->dac_clk, dac_rate_new);
		if (status < 0)
			return status;

		/* Ignore difference smaller than 256 Hz. */
		if ((status/256) == (dac_rate_new/256))
			goto set_rate;

		ssc_div += 2;
	} while (--max_tries);

	/* Not able to find a valid bitrate. */
	return -ENXIO;

set_rate:
	status = clk_set_rate(chip->board->dac_clk, status);
	
	if (status < 0)
		return status;

	/* Set divider in SSC device. */
	ssc_writel(chip->ssc->regs, CMR, ssc_div/2);

	/* SSC clock / (ssc divider * 16-bit * stereo). */
	chip->bitrate = ssc_rate / (ssc_div * 16 * 2); 

	dev_info(&chip->spi->dev,
			"cs4271: supported bitrate is %lu (%lu divider)\n",
			chip->bitrate, ssc_div);

	return 0;
}

static int snd_cs4271_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_cs4271 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;

	snd_cs4271_playback_hw.rate_min = chip->bitrate;
	snd_cs4271_playback_hw.rate_max = chip->bitrate;

	snd_cs4271_capture_hw.rate_min = chip->bitrate;
	snd_cs4271_capture_hw.rate_max = chip->bitrate;

	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK)
		runtime->hw = snd_cs4271_playback_hw;
	else 
		runtime->hw = snd_cs4271_capture_hw;

	chip->substream = substream;

	return 0;
}

static int snd_cs4271_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_cs4271 *chip = snd_pcm_substream_chip(substream);
	
	chip->substream = NULL;
	return 0;
}

static int snd_cs4271_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_cs4271_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int snd_cs4271_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_cs4271 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	int block_size;

	block_size = frames_to_bytes(runtime, runtime->period_size);

	chip->period = 0;

	if (stream_id == SNDRV_PCM_STREAM_CAPTURE)
	{
		ssc_writel(chip->ssc->regs, PDC_RPR,
				(long)runtime->dma_addr);
		ssc_writel(chip->ssc->regs, PDC_RCR, runtime->period_size * 2);
		ssc_writel(chip->ssc->regs, PDC_RNPR,
				(long)runtime->dma_addr + block_size);
		ssc_writel(chip->ssc->regs, PDC_RNCR, runtime->period_size * 2);
	}
	else
	{
		ssc_writel(chip->ssc->regs, PDC_TPR,
			(long)runtime->dma_addr);
		ssc_writel(chip->ssc->regs, PDC_TCR, runtime->period_size * 2);
		ssc_writel(chip->ssc->regs, PDC_TNPR,
			(long)runtime->dma_addr + block_size);
		ssc_writel(chip->ssc->regs, PDC_TNCR, runtime->period_size * 2);
	}

	return 0;
}

static int snd_cs4271_pcm_trigger(struct snd_pcm_substream *substream,
				   int cmd)
{
	struct snd_cs4271 *chip = snd_pcm_substream_chip(substream);
	int stream_id = substream->pstr->stream;
	int retval = 0;

	spin_lock(&chip->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (stream_id == SNDRV_PCM_STREAM_CAPTURE)
		{
			ssc_writel(chip->ssc->regs, IER, SSC_BIT(IER_ENDRX));
			ssc_writel(chip->ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_RXTEN));
		}
		else
		{
			ssc_writel(chip->ssc->regs, IER, SSC_BIT(IER_ENDTX));
			ssc_writel(chip->ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_TXTEN));
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (stream_id == SNDRV_PCM_STREAM_CAPTURE)
		{
			ssc_writel(chip->ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_RXTDIS));
			ssc_writel(chip->ssc->regs, IDR, SSC_BIT(IDR_ENDRX));		
		}
		else
		{
			ssc_writel(chip->ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_TXTDIS));
			ssc_writel(chip->ssc->regs, IDR, SSC_BIT(IDR_ENDTX));
		}
		break;
	default:
		dev_dbg(&chip->spi->dev, "spurious command %x\n", cmd);
		retval = -EINVAL;
		break;
	}

	spin_unlock(&chip->lock);

	return retval;
}

static snd_pcm_uframes_t
snd_cs4271_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_cs4271 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	snd_pcm_uframes_t pos;
	unsigned long bytes;

	if(stream_id == SNDRV_PCM_STREAM_CAPTURE)
	{
		bytes = ssc_readl(chip->ssc->regs, PDC_RPR)
			- (unsigned long)runtime->dma_addr;
	}
	else
	{
		bytes = ssc_readl(chip->ssc->regs, PDC_TPR)
			- (unsigned long)runtime->dma_addr;
	}

	pos = bytes_to_frames(runtime, bytes);
	if (pos >= runtime->buffer_size)
		pos -= runtime->buffer_size;

	return pos;
}

static struct snd_pcm_ops cs4271_capture_ops = {
	.open		= snd_cs4271_pcm_open,
	.close		= snd_cs4271_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_cs4271_pcm_hw_params,
	.hw_free	= snd_cs4271_pcm_hw_free,
	.prepare	= snd_cs4271_pcm_prepare,
	.trigger	= snd_cs4271_pcm_trigger,
	.pointer	= snd_cs4271_pcm_pointer,
};

static struct snd_pcm_ops cs4271_playback_ops = {
	.open		= snd_cs4271_pcm_open,
	.close		= snd_cs4271_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_cs4271_pcm_hw_params,
	.hw_free	= snd_cs4271_pcm_hw_free,
	.prepare	= snd_cs4271_pcm_prepare,
	.trigger	= snd_cs4271_pcm_trigger,
	.pointer	= snd_cs4271_pcm_pointer,
};

static void snd_cs4271_pcm_free(struct snd_pcm *pcm)
{
	struct snd_cs4271 *chip = snd_pcm_chip(pcm);
	if (chip->pcm) {
		snd_pcm_lib_preallocate_free_for_all(chip->pcm);
		chip->pcm = NULL;
	}
}

static int __devinit snd_cs4271_pcm_new(struct snd_cs4271 *chip, int device)
{
	struct snd_pcm *pcm;
	int retval;

	retval = snd_pcm_new(chip->card, chip->card->shortname,
			device, 1, 1, &pcm);
	if (retval < 0)
		goto out;

	pcm->private_data = chip;
	pcm->private_free = snd_cs4271_pcm_free;
	pcm->info_flags = SNDRV_PCM_INFO_BLOCK_TRANSFER;
	strcpy(pcm->name, "cs4271");
	chip->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &cs4271_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &cs4271_capture_ops);

	retval = snd_pcm_lib_preallocate_pages_for_all(chip->pcm,
			SNDRV_DMA_TYPE_DEV, &chip->ssc->pdev->dev,
			64 * 1024, 64 * 1024);
out:
	return retval;
}

static irqreturn_t snd_cs4271_interrupt(int irq, void *dev_id)
{
	struct snd_cs4271 *chip = dev_id;
	struct snd_pcm_runtime *runtime = chip->substream->runtime;
	u32 status;
	int offset;
	int block_size;
	int next_period;
	int retval = IRQ_NONE;

	spin_lock(&chip->lock);

	block_size = frames_to_bytes(runtime, runtime->period_size);
	status = ssc_readl(chip->ssc->regs, IMR);

	if (status & SSC_BIT(IMR_ENDTX)) 
	{
		chip->period++;
		if (chip->period == runtime->periods)
			chip->period = 0;
		next_period = chip->period + 1;
		if (next_period == runtime->periods)
			next_period = 0;

		offset = block_size * next_period;

		ssc_writel(chip->ssc->regs, PDC_TNPR,
				(long)runtime->dma_addr + offset);
		ssc_writel(chip->ssc->regs, PDC_TNCR, runtime->period_size * 2);
		retval = IRQ_HANDLED;
	}
	else if (status & SSC_BIT(IMR_ENDRX)) 
	{
		chip->period++;
		if (chip->period == runtime->periods)
			chip->period = 0;
		next_period = chip->period + 1;
		if (next_period == runtime->periods)
			next_period = 0;

		offset = block_size * next_period;

		ssc_writel(chip->ssc->regs, PDC_RNPR,
				(long)runtime->dma_addr + offset);
		ssc_writel(chip->ssc->regs, PDC_RNCR, runtime->period_size * 2);
		retval = IRQ_HANDLED;
	}
	else	
	{
		printk(KERN_WARNING
				"Spurious SSC interrupt, status = 0x%08lx\n",
				(unsigned long)status);
		ssc_writel(chip->ssc->regs, IDR, status);
	}

	ssc_readl(chip->ssc->regs, IMR);
	spin_unlock(&chip->lock);

	if ((status & SSC_BIT(IMR_ENDTX))||(status & SSC_BIT(IMR_ENDRX)))
		snd_pcm_period_elapsed(chip->substream);

	return retval;
}

/*
 * Mixer functions.
 */
static int snd_cs4271_stereo_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	int mask = (kcontrol->private_value >> 24) & 0xff;

	if (mask == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask;

	return 0;
}

static int snd_cs4271_stereo_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_cs4271 *chip = snd_kcontrol_chip(kcontrol);
	int left_reg = kcontrol->private_value & 0xff;
	int right_reg = (kcontrol->private_value >> 8) & 0xff;
	int shift_left = (kcontrol->private_value >> 16) & 0x07;
	int shift_right = (kcontrol->private_value >> 19) & 0x07;
	int mask = (kcontrol->private_value >> 24) & 0xff;
	int invert = (kcontrol->private_value >> 22) & 1;

	mutex_lock(&chip->mixer_lock);

	ucontrol->value.integer.value[0] =
		(chip->reg_image[left_reg] >> shift_left) & mask;
	ucontrol->value.integer.value[1] =
		(chip->reg_image[right_reg] >> shift_right) & mask;

	if (invert) {
		ucontrol->value.integer.value[0] =
			mask - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] =
			mask - ucontrol->value.integer.value[1];
	}

	mutex_unlock(&chip->mixer_lock);

	return 0;
}

static int snd_cs4271_stereo_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_cs4271 *chip = snd_kcontrol_chip(kcontrol);
	int left_reg = kcontrol->private_value & 0xff;
	int right_reg = (kcontrol->private_value >> 8) & 0xff;
	int shift_left = (kcontrol->private_value >> 16) & 0x07;
	int shift_right = (kcontrol->private_value >> 19) & 0x07;
	int mask = (kcontrol->private_value >> 24) & 0xff;
	int invert = (kcontrol->private_value >> 22) & 1;
	int change, retval;
	unsigned short val1, val2;

	val1 = ucontrol->value.integer.value[0] & mask;
	val2 = ucontrol->value.integer.value[1] & mask;
	if (invert) {
		val1 = mask - val1;
		val2 = mask - val2;
	}
	val1 <<= shift_left;
	val2 <<= shift_right;

	mutex_lock(&chip->mixer_lock);

	val1 = (chip->reg_image[left_reg] & ~(mask << shift_left)) | val1;
	val2 = (chip->reg_image[right_reg] & ~(mask << shift_right)) | val2;
	change = val1 != chip->reg_image[left_reg]
		|| val2 != chip->reg_image[right_reg];
	retval = snd_cs4271_write_reg(chip, left_reg, val1);
	if (retval) {
		mutex_unlock(&chip->mixer_lock);
		goto out;
	}
	retval = snd_cs4271_write_reg(chip, right_reg, val2);
	if (retval) {
		mutex_unlock(&chip->mixer_lock);
		goto out;
	}

	mutex_unlock(&chip->mixer_lock);

	return change;

out:
	return retval;
}

#define CS4271_STEREO(xname, xindex, left_reg, right_reg, shift_left, shift_right, mask, invert) \
{									\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,				\
	.name = xname,							\
	.index = xindex,						\
	.info = snd_cs4271_stereo_info,					\
	.get = snd_cs4271_stereo_get,					\
	.put = snd_cs4271_stereo_put,					\
	.private_value = (left_reg | (right_reg << 8)			\
			| (shift_left << 16) | (shift_right << 19)	\
			| (mask << 24) | (invert << 22))		\
}

static struct snd_kcontrol_new snd_cs4271_controls[] __devinitdata = {
CS4271_STEREO("PCM Playback Volume", 0, DAC_VOL_CHANA_REG, DAC_VOL_CHANB_REG, 0, 0, 0x7f, 1),
};

static int __devinit snd_cs4271_mixer(struct snd_cs4271 *chip)
{
	struct snd_card *card;
	int errval, idx;

	if (chip == NULL || chip->pcm == NULL)
		return -EINVAL;

	card = chip->card;

	strcpy(card->mixername, chip->pcm->name);

	for (idx = 0; idx < ARRAY_SIZE(snd_cs4271_controls); idx++) {
		errval = snd_ctl_add(card,
				snd_ctl_new1(&snd_cs4271_controls[idx],
					chip));
		if (errval < 0)
			goto cleanup;
	}

	return 0;

cleanup:
	for (idx = 1; idx < ARRAY_SIZE(snd_cs4271_controls) + 1; idx++) {
		struct snd_kcontrol *kctl;
		kctl = snd_ctl_find_numid(card, idx);
		if (kctl)
			snd_ctl_remove(card, kctl);
	}
	return errval;
}

/*
 * Device functions
 */
static int snd_cs4271_ssc_init(struct snd_cs4271 *chip)
{
	ssc_writel(chip->ssc->regs, TCMR,
			SSC_BF(TCMR_CKO, 1)
			| SSC_BF(TCMR_START, 7)
			| SSC_BF(TCMR_STTDLY, 0)
			| SSC_BF(TCMR_PERIOD, 16 - 1));

	ssc_writel(chip->ssc->regs, RCMR,
			SSC_BF(RCMR_CKO, 1)
			| SSC_BF(RCMR_CKI, 1)
			| SSC_BF(RCMR_START, 0)
			| SSC_BF(RCMR_STTDLY, 0)
			| SSC_BF(RCMR_PERIOD, 16 - 1));
	
	ssc_writel(chip->ssc->regs, TFMR,
			SSC_BF(TFMR_DATLEN, 16 - 1)
			| SSC_BIT(TFMR_MSBF)
			| SSC_BF(TFMR_DATNB, 1)
			| SSC_BF(TFMR_FSLEN, 16 - 1)
			| SSC_BF(TFMR_FSOS, 1));

	ssc_writel(chip->ssc->regs, RFMR,
			SSC_BF(RFMR_DATLEN, 16 - 1)
			| SSC_BIT(RFMR_MSBF)
			| SSC_BF(RFMR_DATNB, 1)
			| SSC_BF(RFMR_FSLEN, 16 - 1)
			| SSC_BF(RFMR_FSOS, 1));
	return 0;
}

static int snd_cs4271_chip_init(struct snd_cs4271 *chip)
{
	int retval;

	retval = snd_cs4271_set_bitrate(chip);
	if (retval)
		goto out;

	/* Enable DAC master clock. */
	clk_enable(chip->board->dac_clk);
	
	/* Enable I2S device, i.e. clock output. */
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXEN));
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_RXEN));

	/* Enable control */
	snd_cs4271_write_reg(chip, MODE2_REG, 0x03);

	/* up to 24 bit I2S mode */
	//snd_cs4271_write_reg(chip, MODE1_REG, 0x01);

	/* Left justified format (seems to work better) */
	snd_cs4271_write_reg(chip, MODE1_REG, 0x00);

	/* I2S format / 16 bit dither */
	//snd_cs4271_write_reg(chip, ADC_CTRL_REG, 0x30);

	/* Left justified format / 16 bit dither */
	snd_cs4271_write_reg(chip, ADC_CTRL_REG, 0x20);

	/* Turn auto-mute off */
	snd_cs4271_write_reg(chip, DAC_CTRL_REG, 0x00);

	/* Volume 100% */
	snd_cs4271_write_reg(chip, DAC_VOL_CHANA_REG, 0);
	snd_cs4271_write_reg(chip, DAC_VOL_CHANB_REG, 0);

	/* Enable the CODEC */
	snd_cs4271_write_reg(chip, MODE2_REG, 0x02);

	goto out;

out:
	return retval;
}

static int snd_cs4271_dev_free(struct snd_device *device)
{
	struct snd_cs4271 *chip = device->device_data;

	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXDIS));
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_RXDIS));

	if (chip->irq >= 0) {
		free_irq(chip->irq, chip);
		chip->irq = -1;
	}

	return 0;
}

static int __devinit snd_cs4271_dev_init(struct snd_card *card,
					 struct spi_device *spi)
{
	static struct snd_device_ops ops = {
		.dev_free	= snd_cs4271_dev_free,
	};
	struct snd_cs4271 *chip = get_chip(card);
	int irq, retval;


	irq = chip->ssc->irq;
	if (irq < 0)
		return irq;

	spin_lock_init(&chip->lock);
	mutex_init(&chip->mixer_lock);
	chip->card = card;
	chip->irq = -1;

	retval = request_irq(irq, snd_cs4271_interrupt, 0, "cs4271", chip);
	if (retval) {
		dev_dbg(&chip->spi->dev, "unable to request irq %d\n", irq);
		goto out;
	}

	chip->irq = irq;

	memcpy(&chip->reg_image, &snd_cs4271_original_image,
			sizeof(snd_cs4271_original_image));

	retval = snd_cs4271_ssc_init(chip);
	if (retval)
		goto out_irq;

	retval = snd_cs4271_chip_init(chip);
	if (retval)
		goto out_irq;

	retval = snd_cs4271_pcm_new(chip, 0);
	if (retval)
		goto out_irq;

	retval = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (retval)
		goto out_irq;

	retval = snd_cs4271_mixer(chip);
	if (retval)
		goto out_snd_dev;

	snd_card_set_dev(card, &spi->dev);

	goto out;

out_snd_dev:
	snd_device_free(card, chip);
out_irq:
	free_irq(chip->irq, chip);
	chip->irq = -1;
out:
	return retval;
}

static int snd_cs4271_probe(struct spi_device *spi)
{
	struct snd_card			*card;
	struct snd_cs4271		*chip;
	struct cs4271_board_info	*board;
	int				retval;
	char				id[16];

	board = spi->dev.platform_data;
	if (!board) {
		dev_dbg(&spi->dev, "no platform_data\n");
		return -ENXIO;
	}

	if (!board->dac_clk) {
		dev_dbg(&spi->dev, "no DAC clk\n");
		return -ENXIO;
	}

	if (IS_ERR(board->dac_clk)) {
		dev_dbg(&spi->dev, "no DAC clk\n");
		return PTR_ERR(board->dac_clk);
	}

	retval = -ENOMEM;

	/* Allocate "card" using some unused identifiers. */
	snprintf(id, sizeof id, "cs4271_%d", board->ssc_id);
	card = snd_card_new(-1, id, THIS_MODULE, sizeof(struct snd_cs4271));
	if (!card)
		goto out;

	chip = card->private_data;
	chip->spi = spi;
	chip->board = board;

	chip->ssc = ssc_request(board->ssc_id);
	if (IS_ERR(chip->ssc)) {
		dev_dbg(&spi->dev, "could not get ssc%d device\n",
				board->ssc_id);
		retval = PTR_ERR(chip->ssc);
		goto out_card;
	}

	retval = snd_cs4271_dev_init(card, spi);
	if (retval)
		goto out_ssc;

	strcpy(card->driver, "cs4271");
	strcpy(card->shortname, board->shortname);
	sprintf(card->longname, "%s on irq %d", card->shortname, chip->irq);

	retval = snd_card_register(card);
	if (retval)
		goto out_ssc;

	dev_set_drvdata(&spi->dev, card);

	goto out;

out_ssc:
	ssc_free(chip->ssc);
out_card:
	snd_card_free(card);
out:
	return retval;
}

static int __devexit snd_cs4271_remove(struct spi_device *spi)
{
	struct snd_card *card = dev_get_drvdata(&spi->dev);
	struct snd_cs4271 *chip = card->private_data;

	/* Stop playback. */
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXDIS));
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_RXDIS));

	clk_disable(chip->board->dac_clk);

	ssc_free(chip->ssc);
	snd_card_free(card);
	dev_set_drvdata(&spi->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int snd_cs4271_suspend(struct spi_device *spi, pm_message_t msg)
{
	struct snd_card *card = dev_get_drvdata(&spi->dev);
	struct snd_cs4271 *chip = card->private_data;

	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXDIS));
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_RXDIS));
	clk_disable(chip->board->dac_clk);

	return 0;
}

static int snd_cs4271_resume(struct spi_device *spi)
{
	struct snd_card *card = dev_get_drvdata(&spi->dev);
	struct snd_cs4271 *chip = card->private_data;

	clk_enable(chip->board->dac_clk);
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXEN));
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_RXEN));	

	return 0;
}
#else
#define snd_cs4271_suspend NULL
#define snd_cs4271_resume NULL
#endif

static struct spi_driver cs4271_driver = {
	.driver		= {
		.name	= "cs4271",
	},
	.probe		= snd_cs4271_probe,
	.suspend	= snd_cs4271_suspend,
	.resume		= snd_cs4271_resume,
	.remove		= __devexit_p(snd_cs4271_remove),
};

static int __init cs4271_init(void)
{
	return spi_register_driver(&cs4271_driver);
}
module_init(cs4271_init);

static void __exit cs4271_exit(void)
{
	spi_unregister_driver(&cs4271_driver);
}
module_exit(cs4271_exit);

MODULE_AUTHOR("Michael Welling");
MODULE_DESCRIPTION("Sound driver for CS4271 with Atmel SSC");
MODULE_LICENSE("GPL");
