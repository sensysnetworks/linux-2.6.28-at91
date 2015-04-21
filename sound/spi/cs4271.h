/*
 * Driver for the CS4271 16-bit stereo DAC
 *
 * Copyright (C) 2006 Atmel
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * The full GNU General Public License is included in this
 * distribution in the file called COPYING.
 */

#ifndef SND_CS4271_MIXER_H_
#define SND_CS4271_MIXER_H_

/* Mode Control Register 1 */
#define MODE1_REG 0x01

/* DAC Control Register */
#define DAC_CTRL_REG 0x02

/* DAC Volume/ Mixing Control Register */
#define DAC_VOL_MIX_REG 0x03

/* DAC Ch. A Volume Control Register */
#define DAC_VOL_CHANA_REG 0x04

/* DAC Ch. B Volume Control Register */
#define DAC_VOL_CHANB_REG 0x05

/* ADC Control Register */
#define ADC_CTRL_REG 0x06

/* Mode Control Register 2 */
#define MODE2_REG 0x07

#endif

