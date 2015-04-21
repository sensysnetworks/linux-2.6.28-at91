/*
 * cs4271.h  --  CS4271 Soc Audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on wm8753.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _CS4271_H
#define _CS4271_H

/* CS4271 register space */
#define CS4271_MODE1_REG		0x01
#define CS4271_DAC_CTRL_REG		0x02
#define CS4271_DAC_VOL_MIX_REG		0x03
#define CS4271_DAC_VOL_CHANA_REG	0x04
#define CS4271_DAC_VOL_CHANB_REG	0x05
#define CS4271_ADC_CTRL_REG		0x06
#define CS4271_MODE2_REG		0x07
#define CS4271_DEVICE_REG		0x08

#define CS4271_CACHEREGNUM 	9

#define CS4271_SYSCLK	0
#define CS4271_DAI	0

struct cs4271_setup_data {
	int            spi;
	int            i2c_bus;
	unsigned short i2c_address;
};

extern struct snd_soc_dai cs4271_dai;
extern struct snd_soc_codec_device soc_codec_dev_cs4271;

#endif
