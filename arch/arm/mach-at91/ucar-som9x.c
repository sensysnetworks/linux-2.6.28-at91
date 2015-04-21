/*
 * arch/arm/mach-at91/gpio-som9x.c
 * EMAC.Inc SOM9260M/SOM9G20M support for the UCAR GPIOs.
 *
 * The SOM9260M & SOM9G20M are pin compatable. SOM9X refers to them interchangeably.
 *
 * Copyright (C) 2010 EMAC.Inc <support@emacinc.com>
 */

#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <mach/board.h>
#include <mach/at91sam9260.h>
#include <mach/gpio.h>

/* #define UCAR_DEBUG */

#ifdef UCAR_DEBUG
#define DPRINT(string, args...) printk(string, ##args)
#else
#define DPRINT(args...)
#endif


/**
 * static array defining GPIO set
 */
static unsigned ucar_gpio_set[] = {
	AT91_PIN_PC0,
	AT91_PIN_PC1,
	AT91_PIN_PC2,
	AT91_PIN_PC3,
	AT91_PIN_PB0,
	AT91_PIN_PB1,
	AT91_PIN_PB2,
	AT91_PIN_PB3,
};

#define UCAR_NUM_GPIO ARRAY_SIZE(ucar_gpio_set)

gpio_data ddr_value_shadow = 0x0;

/**
 * function to initialize the digital inputs and outputs
 */
static int ucar_gpio_init(void)
{
	int i;
	
	DPRINT("ucar_gpio_init: %d GPIO\n", UCAR_NUM_GPIO);

	/* 
	 * set all GPIOs to inputs by default 
	 */
	for (i = 0; i < UCAR_NUM_GPIO; i++)
		at91_set_gpio_input(ucar_gpio_set[i], 0);

	return 0;
}

/**
 * function to read back the current status of the GPIO lines
 * no shadow register is needed because we can read the current
 * state of output lines on the 9260/9G20
 */
static gpio_data ucar_gpio_data_read(struct gpio_s *gpio)
{
	gpio_data value;
	int i;

	DPRINT("ucar_gpio_data_read\n");

	value = 0;

	for (i = 0; i < UCAR_NUM_GPIO; i++)
	{
		value |= (at91_get_gpio_value(ucar_gpio_set[i]) << i);
	}


	return value;
}

/**
 * function to set the value of the GPIO lines
 */
static int ucar_gpio_data_write(struct gpio_s *gpio, gpio_data data)
{
	int i;

	DPRINT("ucar_gpo_data_write: 0x%X\n", data);
	
	for (i = 0; i < UCAR_NUM_GPIO; i++) 
	{
		at91_set_gpio_value(ucar_gpio_set[i], (data >> i) & 1);
		DPRINT("  wrote %d to offset %d\n", (data >> i) & 1, i);
	}

	return 0;
}

/* no index */

/**
 * data direction configuration read
 */
static gpio_data ucar_gpio_ddr_read(struct gpio_s *gpio)
{
	return ddr_value_shadow;
}

/**
 * data direction configuration write
 */
static int ucar_gpio_ddr_write(struct gpio_s *gpio, gpio_data data)
{
	int i;

	ddr_value_shadow = data;

	for (i = 0; i < UCAR_NUM_GPIO; i++)
	{
		if(data & (1 << i)) 
		{
			at91_set_gpio_output(ucar_gpio_set[i], 0);
			at91_set_multi_drive(ucar_gpio_set[i], 0);
		}
		else
		{
			at91_set_gpio_input(ucar_gpio_set[i], 0);
		}
	}

	return 0;
}

/**
 * function to register the class for the gpo register
 */
struct device *ucar_gpio_class_create(void)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);

	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = "ucar_gpio";
	gpio->subclass = GPO_SUBCLASS;
	gpio->data_write = ucar_gpio_data_write;
	gpio->data_read = ucar_gpio_data_read;
	gpio->ddr_write = ucar_gpio_ddr_write;
	gpio->ddr_read = ucar_gpio_ddr_read;
	printk("registering gpio device: %s\n", gpio->name);

	return gpio_register_device(gpio);
}

void ucar_board_init(void)
{
	ucar_gpio_init();
	ucar_gpio_class_create();
}

