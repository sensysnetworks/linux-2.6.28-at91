#ifndef SPI_INTERFACE_H_
#define SPI_INTERFACE_H_

/**
 * Provides an interface between the SPI class
 * driver and the Linux SPI layer for devices that
 * already implement a device specific driver and
 * register this interface in the mach.
 * Naming convention:
 * Linux SPI Interface: lsi
 * EMAC SPI Class: esc
 * -> lsi2esc
 * 
 * Copyright (C) 2007, EMAC Inc
 */

#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/class/spi.h>

int lsi2esc_spi_tip(struct spi_s *s,int ofs);
int lsi2esc_spi_xmit(struct spi_s *s, u8 *mosi, u8 *miso, int size);
int lsi2esc_spi_confwrite(struct spi_s *s, spi_control config);
spi_control lsi2esc_spi_confread(struct spi_s *s);
int lsi2esc_spi_speedwrite(struct spi_s *s, spi_control speed);
spi_control lsi2esc_spi_speedread(struct spi_s *s);

#endif /*SPI_INTERFACE_H_*/
