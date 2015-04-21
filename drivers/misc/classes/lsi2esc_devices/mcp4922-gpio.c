/*
 * drivers/misc/classes/lsi2esc_devices/mcp4922-gpio.c
 * EMAC.Inc support for indexed GPIO interface to the MCP4922
 * DAC used on the SoM-150ES. Provides a generic wrapper to the
 * LSI2ESC functions used to communicate to the device.
 *
 * Copyright (C) 2008 EMAC.Inc <support@emacinc.com>
 */

#include <linux/class/lsi2esc/mcp4922-gpio.h>
#include <linux/class/spi.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>

static struct spi_s *spi_dev;

static u16 dac_value[3] = { 0, 0, 0x3 };

/**
 * function to initialize the settings for the interface.
 * This cannot be called from the class create function because
 * the LSI2ESC may not have been probed yet.
 */
static void mcp4922_init(void)
{
	static spi_control config = SPICL_EIGHTBIT;
	spi_dev->confwrite(spi_dev, config);
}

/**
 * function to read the value of the last value written to the DAC 
 * at the current index.
 */
int mcp4922_data_write(struct gpio_s *gpio, gpio_data data)
{
	u32 size;
	u8 reg = gpio->index;
	u8 mosi[2];
	u8 miso[2];

	if (reg > 2) return -1;

	size = 2;
	
	dac_value[reg] = data & 0xFFF; 

	if(reg == 2) return 0;

	if (reg == 0) 	mosi[0] = 0x00;
	else 		mosi[0] = 0x80;

	mosi[0] |= (u8) ((dac_value[2] & 0x07) << 4);
	mosi[0] |= (u8) ((data >> 8) & 0x0f);
	mosi[1]  = (u8) (data & 0xFF);

	memset(miso, 0x0, sizeof(miso));
	spi_dev->xmit(spi_dev, mosi, miso, size);

	return 0;
}

/**
 * function to read the value of the last value written to the DAC 
 * at the current index.
 */
static gpio_data mcp4922_data_read(struct gpio_s *gpio)
{
	if (gpio->index > 2) return 0;
	
	return dac_value[gpio->index];
}

/**
 * function to register the class
 */
struct device *mcp4922_gpio_class_create(struct spi_s *spi, const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	spi_dev = spi;

	mcp4922_init();

	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPI_SUBCLASS;
	gpio->data_write = mcp4922_data_write;
	gpio->range = 7;
	gpio->index_write = gpio_index_write;
	gpio->index_read = gpio_index_read;
	gpio->data_read = mcp4922_data_read;
	printk("registering mcp4922 GPIO interface: %s\n", gpio->name);

	return gpio_register_device(gpio);
}

