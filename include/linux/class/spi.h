#ifndef SPI_CLASS_H_
#define SPI_CLASS_H_
#include <linux/autoconf.h>
#include <linux/device.h>
#if defined(CONFIG_LSI2ESC)
#include <linux/spi/spi.h>
#endif

#ifdef CONFIG_SPICLASS

#define SPICL_CPHA	(0x01)
#define SPICL_CPOL	(0x02)

/* common bit settings -- should be mutually exclusive */
/* it would be possible to let 8 bit mode be 0x00 and let
 * this be the default, but this does not work well for the
 * LSI2ESC interface as the default controller setting is to
 * leave bits_per_word set to 0. Leave 8 bit as an explicit
 * setting. */
#define SPICL_EIGHTBIT		(0x04)	/* 8 bit is default */
#define SPICL_TENBIT		(0x08)	/* 10 bits per transfer */
#define SPICL_TWELVEBIT		(0x10)	/* 12 bits per transfer */
#define SPICL_SIXTEENBIT	(0x20)	/* 16 bits per transfer */

typedef u8 spi_data;
typedef u32 spi_control;

//control definitions for the tip function
#define TIPOFF		0
#define TIPON		1
#define TIPSTATUS	2

/**********************
 * spi class structure
 */
typedef struct spi_s {
	const char *name;
	int subclass;
	spi_data *buf;
	int bsize;
	/* if the SPI interface is used there needs to be a pointer to
	 * the associated spi_device struct used by the Linux SPI layer
	 */
#if defined(CONFIG_LSI2ESC)
	struct spi_device *lsi;
#endif
	int (*tip) (struct spi_s * s, int ofs);	//declare transfer in process, for locking purposes
	int (*xmit) (struct spi_s * s, u8 * mosi, u8 * miso, int size);
	int (*confwrite) (struct spi_s * s, spi_control config);
	 spi_control(*confread) (struct spi_s * s);
	int (*speedwrite) (struct spi_s * s, spi_control speed);
	 spi_control(*speedread) (struct spi_s * s);
	/* method for creating a GPIO interface to this device */
	void *gpio_data; /* generic data for passing to the GPIO class */
	char *gpio_name; /* name of the gpio device to create */
	struct device *(*gpio_create) (struct spi_s * spi, const char *name);
} spi_t;

#define SPI_BASECLASNUM	0xC0
#define SPI_SUBCLASS 	(SPI_BASECLASNUM+0)

/***************************************************************************
 * initial class declaration, doesn't hurt to call it mulitiple times,
 * automatically checked during device instantiation 
 */
struct class *spi_declare(void);

/***************************************************************************
 * class instantiation
 */
struct device *spi_register_device(spi_t * s);

/***************************************************************************
 * atomic method wrappers
 * these should be used for all method calls to maintain sychronization across the
 * various interfaces
 */
int atomic_spi_xmit(struct spi_s *s, u8 * mosi, u8 * miso, int size);
int atomic_spi_conf_write(struct spi_s *s, spi_control config);
spi_control atomic_spi_conf_read(struct spi_s *s);
int atomic_spi_speed_write(struct spi_s *s, spi_control config);
spi_control atomic_spi_speed_read(struct spi_s *s);
int atomic_spi_tip_write(struct spi_s *s, spi_control config);
spi_control atomic_spi_tip_read(struct spi_s *s);

#endif

#endif /*SPI_CLASS_H_ */
