#include <linux/class/spi_interface.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/spi/spi.h>

/**
 * Provides an interface betwee the Linux SPI interface and the
 * EMAC SPI class (lsi2esc).
 * 
 * Copyright (C) 2007, EMAC Inc
 */

//#define DEBUG_LSI

#ifdef DEBUG_LSI
#define DPRINTK(string, args...) printk("lsi2esc: " string, ##args)
#else
#define DPRINTK(string, args...)
#endif

/**
 * function to set/clear a transfer-in-progrees for
 * the given device. This function should be implemented
 * by the driver itself to make sure that the same device
 * is being accessed and as such this function does nothing
 */
int lsi2esc_spi_tip(struct spi_s *s, int ofs)
{
	DPRINTK("lsi2esc_spi_tip\n");
	return 0;
}

/**
 * function to transfer data on the SPI bus
 * Data is written/read using an spi_sync call which causes
 * the chip select to remain active throughout the entire transfer
 * but also requires that the available size of the buffers is the
 * sum of the write data plus the read data. i.e. to write an 8 bit
 * command and read a 8 bit response the response would be stored in
 * the last 8 bits of a 16 bit miso, the data to be written would need
 * to be placed in the top 8 bits of a 16 bit mosi.
 * Assumptions: MOSI and MISO can be the same buffer,
 * if MOSI is NULL 0xFF is transmitted, if MISO is NULL
 * received data is discarded, there is no limit on buffer
 * size but both buffers must be the same size if they both exist.
 * 
 * @param s the EMAC SPI class device to tranfer data on
 * @param mosi the transmit buffer (master->slave)
 * @param miso the receive buffer (slave->master)
 * @param size the size of the data to transfer
 * @return 0 or negative error code
 */
int lsi2esc_spi_xmit(struct spi_s *s, u8 * mosi, u8 * miso, int size)
{
	int result = 0;
	u8 *local_mosi = mosi;
	u8 *local_miso = miso;
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.cs_change = 0,	/* hold cs for entire transfer */
	};

	DPRINTK("lsi2esc_spi_xmit %d bytes\n", size);

	if (!local_mosi) {
		if (!(local_mosi = kmalloc(size, GFP_KERNEL)))
			return -ENOMEM;
		memset(local_mosi, 0xFF, size);
	}
	if (!local_miso) {
		/* data is discarded, but a buffer must exist */
		if (!(local_miso = kmalloc(size, GFP_KERNEL)))
			return -ENOMEM;
	}
#ifdef DEBUG_LSI
	int temp_size;
	for (temp_size = size; temp_size > 0; temp_size--) {
		DPRINTK("mosi[%d] = 0x%X\n", temp_size - 1,
			mosi ? mosi[temp_size - 1] : 0);
	}
#endif
	spi_message_init(&msg);
	msg_xfer.tx_buf = local_mosi;
	msg_xfer.rx_buf = local_miso;
	msg_xfer.len = size;
	spi_message_add_tail(&msg_xfer, &msg);

	result = spi_sync(s->lsi, &msg);

#ifdef DEBUG_LSI
	for (temp_size = size; temp_size > 0; temp_size--) {
		DPRINTK("miso[%d] = 0x%X\n", temp_size - 1,
			local_miso ? local_miso[temp_size - 1] : 0);
	}
#endif

	if (!mosi)
		kfree(local_mosi);
	if (!miso)
		kfree(local_miso);

	return result;
}

/**
 * function to read the current configuration settings
 * @param s the EMAC SPI class device to read from
 * @return the current configuration
 */
spi_control lsi2esc_spi_confread(struct spi_s * s)
{
	spi_control flags = 0;
	DPRINTK("lsi2esc_spi_confread\n");

	flags = s->lsi->mode;	/* SPI and SPICL same here */

	DPRINTK("bits_per_word = %d\n", s->lsi->bits_per_word);
	if (s->lsi->bits_per_word == 8)
		flags |= SPICL_EIGHTBIT;	/* actually does nothing */
	else if (s->lsi->bits_per_word == 10)
		flags |= SPICL_TENBIT;
	else if (s->lsi->bits_per_word == 12)
		flags |= SPICL_TWELVEBIT;
	else if (s->lsi->bits_per_word == 16)
		flags |= SPICL_SIXTEENBIT;

	return flags;
}

/**
 * function to change the current configuration settings
 * @param s the EMAC SPI class device to configure
 * @param config the new configuration to write
 * @return 0 or the result of spi_setup
 */
int lsi2esc_spi_confwrite(struct spi_s *s, spi_control config)
{
	DPRINTK("lsi2esc_spi_confwrite: %d\n", config);

	/* set the mode */
	s->lsi->mode = 0;
	if (config & SPI_CPOL)
		s->lsi->mode |= SPI_CPOL;
	if (config & SPI_CPHA)
		s->lsi->mode |= SPI_CPHA;

	/* explicitly set the bits per word if specified
	 * otherwise leave unchanged */
	if (config & SPICL_EIGHTBIT)
		s->lsi->bits_per_word = 8;
	else if (config & SPICL_TENBIT)
		s->lsi->bits_per_word = 10;
	else if (config & SPICL_TWELVEBIT)
		s->lsi->bits_per_word = 12;
	else if (config & SPICL_SIXTEENBIT)
		s->lsi->bits_per_word = 16;

	DPRINTK("lsi2esc_spi_confwrite: calling spi_setup\r\n");

	return spi_setup(s->lsi);
}

/**
 * function to get the current speed settings
 * @param s the EMAC SPI class device
 * @return the speed setting of the associated spi_device
 */
spi_control lsi2esc_spi_speedread(struct spi_s * s)
{
	DPRINTK("lsi2esc_spi_speedread\n");
	return s->lsi->max_speed_hz;
}

/**
 * function to change the speed settings
 * @param s the EMAC SPI class device
 * @param speed the new speed to set
 */
int lsi2esc_spi_speedwrite(struct spi_s *s, spi_control speed)
{
	DPRINTK("lsi2esc_spi_speedwrite\n");
	s->lsi->max_speed_hz = speed;
	/* error checking and adjustment must be provided by
	 * the SPI controller driver to use this directly */
	return spi_setup(s->lsi);
}

/******************************************************************************/

/**
 * The lsi2esc interface actually acts like an SPI protocol driver.
 * It fakes as an SPI device, but performs generic access to SPI
 * only through the EMAC class interface rather than device specific
 * access. The following section provides the SPI protocol driver
 * code.
 */

/**
 * probe function to initialize the interface
 */

static int __devinit lsi2esc_probe(struct spi_device *spi)
{
	struct spi_s *esc;
	/* the platform data of dev must be set to the spi_t in
	 * the board specific file */
	/* kind of creates a loop... but thats OK */
	esc = spi->dev.platform_data;
	DPRINTK("lsi2esc_probe: %s\n", esc->name);
	esc->lsi = spi;

	if (!esc)
		return -ENODEV;
	if (!spi_register_device(esc))
		return -ENOMEM;
	/* register associated GPIO interface if necessary */
	if (esc->gpio_create)
		esc->gpio_create(esc, esc->gpio_name);

	return 0;
}

/**
 * remove function to provide a clean exit
 */

static int __devexit lsi2esc_remove(struct spi_device *spi)
{
	/* nothing to do here */
	return 0;
}

static struct spi_driver lsi2esc_driver = {
	.driver = {
		   .name = "lsi2esc",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = lsi2esc_probe,
	.remove = __devexit_p(lsi2esc_remove),
};

static __init int lsi2esc_init(void)
{
	DPRINTK("Registering LSI2ESC SPI driver\n");
	return spi_register_driver(&lsi2esc_driver);
}

static __exit void lsi2esc_exit(void)
{
	spi_unregister_driver(&lsi2esc_driver);
}

module_init(lsi2esc_init);
module_exit(lsi2esc_exit);

MODULE_AUTHOR("EMAC.Inc (support@emacinc.com)");
MODULE_DESCRIPTION("Generic EMAC SPI class to Linux SPI Interface Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

// EOF
