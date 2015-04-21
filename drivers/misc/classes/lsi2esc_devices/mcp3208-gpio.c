/*
 * drivers/misc/classes/lsi2esc_devices/mcp3208-gpio.c
 * EMAC.Inc support for indexed GPIO interface to the MCP3208
 * ADC used on the SoM-150ES. Provides a generic wrapper to the
 * LSI2ESC functions used to communicate to the device.
 *
 * Copyright (C) 2008 EMAC.Inc <support@emacinc.com>
 */

#include <linux/class/lsi2esc/mcp3208-gpio.h>
#include <linux/class/spi.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>

static struct spi_s *spi_dev;

/**
 * function to initialize the settings for the interface.
 * This cannot be called from the class create function because
 * the LSI2ESC may not have been probed yet.
 */
static void mcp3208_init(void)
{
	static spi_control config = SPICL_EIGHTBIT;
	spi_dev->confwrite(spi_dev, config);
}

/**
 * function to read the value of the ADC at the current
 * index.
 */
static gpio_data mcp3208_data_read(struct gpio_s *gpio)
{
	u32 size;
	u8 reg = gpio->index;
	u8 reg_msb;
	u8 mosi[3];
	u8 miso[3];

	/*
	 * The command for the mcp3208 is as follows:
	 * leading 0's
	 * start bit
	 * 1 to signal single mode
	 * 3 bits to represent the channel to read
	 */
	size = 3;

	reg_msb = (reg >> 2) & 0x01; /* XYZ >> 2 = X */
	mosi[0] = reg_msb | 0x06;
	/* the last 2 bits of the reg need to go into the top of the
	 * second byte transmitted */
	mosi[1] = reg << 6; /* 00000xxx << 6 = xx000000 */

	memset(miso, 0x0, sizeof(miso));
	spi_dev->xmit(spi_dev, mosi, miso, size);
	return ((miso[1] & 0x0F) << 8) | (miso[2] & 0x00FF);
}

/**
 * function to register the class
 */
struct device *mcp3208_gpio_class_create(struct spi_s *spi, const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	spi_dev = spi;

	mcp3208_init();
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPI_SUBCLASS;
	gpio->data_write = gpio_empty_write;
	gpio->range = 7;
	gpio->index_write = gpio_index_write;
	gpio->index_read = gpio_index_read;
	gpio->data_read = mcp3208_data_read;
	printk("registering mcp3208 GPIO interface: %s\n", gpio->name);

	return gpio_register_device(gpio);
}

