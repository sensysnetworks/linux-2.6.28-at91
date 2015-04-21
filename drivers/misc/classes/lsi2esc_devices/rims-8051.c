/********************************************************************
 * File Name:  drivers/misc/classes/lsi2esc_devices/rims-8051.c
 *
 * Description: SPI interface driver for the RIMS MCB's ARM to 8051 transactions.
 *
 * Copyright (C) 2010 EMAC.Inc <support@emacinc.com>
 ********************************************************************/

#include <linux/class/lsi2esc/rims-8051.h>
#include <linux/class/spi.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <linux/interrupt.h>

DEFINE_MUTEX(rims8051_lock);

static struct spi_s *spi_dev;

#define MAX_MSG_LEN 1024

static u8 *message;
static u8 param1;
static u16 length;
static u8 curptr;

/* 
 * rims8051_init
 *
 * Initializes the SPI controller to use 8 bit transactions.
 * Defaults to mode 0 (PHASE = 0, POLARITY = 0)
 */
static void rims8051_init(void)
{
	static spi_control config = SPICL_EIGHTBIT;
	spi_dev->confwrite(spi_dev, config);
	curptr = length = 0;

	message = kmalloc(sizeof(u8) * MAX_MSG_LEN, GFP_KERNEL);
}

/* 
 * rims8051_data_read
 *
 * This function is called when a DATAREAD ioctl occurs on the device.
 */
static gpio_data rims8051_data_read(struct gpio_s *gpio)
{
	if(curptr == length) return -1;
	curptr++;
	return message[curptr-1];
}

/* 
 * rims8051_data_write
 *
 * This function is called when a DATAWRITE ioctl occurs on the device.
 */
static int rims8051_data_write(struct gpio_s *gpio, gpio_data data)
{
	u32 size;
	u8 mosi[1];
	u8 miso[1];

	mosi[0] = (data & 0xFF);
	size = 1;

	spi_dev->xmit(spi_dev, mosi, miso, size);
	return 0;
}

/*
 * rims8051_handler
 *
 * This function is called after an interrupt occurs.
 */
static gpio_data rims8051_handler(struct gpio_s *gpio)
{
	u32 size;
	u8 mosi[256];
	u8 miso[256];

	if (mutex_lock_interruptible(&rims8051_lock)) {
		rims8051_handler(gpio);
		return 0;
	}

	/* Read the three byte message header */
	size = 3;
	memset(mosi, 0xFF, size * sizeof(char));

	spi_dev->xmit(spi_dev, mosi, miso, size);

	length = miso[0] + (miso[1] << 8);
	param1 = miso[2];
	curptr = 0;

	/* Read the message */
	if (length > 0)
	{
		size = length;
		memset(mosi, 0xFF, size * sizeof(char));

		spi_dev->xmit(spi_dev, mosi, message, size);
	}
	
	mutex_unlock(&rims8051_lock);
	return 1;
}

/*
 * rims8051_get_queue
 *
 * This function is called automatically after the interrupt handler.
 *
 * The value returned will be queued and returned to user-space during a
 * DATAREADQ ioctl.
 */
static gpio_data rims8051_get_queue(struct gpio_s *gpio, gpio_data notify)
{
	return (param1 << 16) + length;
}

/*
 * rims8051_irq_config
 *
 * Not applicable to this driver.
 * Current implementation requires this function must be defined even if not used.
 * This function is used to configure shared interrupts.
 */
static void rims8051_irq_config(struct gpio_s *gpio)
{
}

/*
 * rims8051_change_notify
 *
 * Not applicable to this driver. 
 * Current implementation requires this function must be defined even if not used.
 * This function is used to notify on a subset shared interrupts.
 */
static void rims8051_change_notify(struct gpio_s *gpio, gpio_data curr_notify,
		gpio_data new_notify)
{
}

/* 
 * rims8051_gpio_class_create
 *
 * Create the GPIO class wrapper for the SPI device.
 * This function is called registered in the boardspec.
 */
struct device *rims8051_gpio_class_create(struct spi_s *spi, const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	spi_dev = spi;

	rims8051_init();
	memset(gpio, 0, sizeof(gpio_t));
	
	gpio->name = name;
	gpio->subclass = GPI_SUBCLASS;
	gpio->data_read = rims8051_data_read;
	gpio->data_write = rims8051_data_write;
	gpio->index_write = NULL;
	gpio->index_read = NULL;

	gpio->irq_data.irq = AT91SAM9260_ID_IRQ0;
	gpio->irq_data.irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	gpio->irq_data.handler = rims8051_handler;
	gpio->irq_data.get_queue = rims8051_get_queue;
	gpio->irq_data.irq_config = rims8051_irq_config;
	gpio->irq_data.change_notify = rims8051_change_notify;

	printk("Registering RIMS 8051 GPIO interface: %s\n", gpio->name);

	return gpio_register_device(gpio);
}

