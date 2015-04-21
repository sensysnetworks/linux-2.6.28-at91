/*
 * arch/arm/mach-at91/rims-mcb-som9x.c
 * EMAC.Inc SOM9260M/SOM9G20M support for the ATE Magic RIMS Microcontroller
 * Carrier board.
 *
 * The SOM9260M & SOM9G20M are pin compatable. SOM9X refers to them interchangeably.
 *
 * Copyright (C) 2009 EMAC.Inc <support@emacinc.com>
 */

#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <mach/board.h>
#include <mach/at91sam9260.h>
#include <mach/gpio.h>

/* #define RM_DEBUG */

#ifdef RM_DEBUG
#define DPRINT(string, args...) printk(string, ##args)
#else
#define DPRINT(args...)
#endif


/**
 * static array defining GPIO set
 */
static unsigned rm_gpio_set[] = {
	AT91_PIN_PB6,
	AT91_PIN_PB7,
	AT91_PIN_PB12,
	AT91_PIN_PB13,
	AT91_PIN_PB16,
	AT91_PIN_PB17,
	AT91_PIN_PB18,
	AT91_PIN_PB19,
	AT91_PIN_PB22,
	AT91_PIN_PB23,
	AT91_PIN_PB24,
	AT91_PIN_PB25,
	AT91_PIN_PB26,
	AT91_PIN_PB27,
	AT91_PIN_PB28,
	AT91_PIN_PB29,
	AT91_PIN_PC7,
	AT91_PIN_PC0, /*JAM Player TCK*/
	AT91_PIN_PC1, /*JAM Player TDI*/
	AT91_PIN_PC2, /*JAM Player TDO*/
	AT91_PIN_PC3, /*JAM Player TMS*/
	AT91_PIN_PC6  /*JAM Player TRST*/
};

#define RM_NUM_GPIO ARRAY_SIZE(rm_gpio_set)

gpio_data ddr_value_shadow = 0x0;

/**
 * function to initialize the digital inputs and outputs
 */
static int rims_mcb_gpio_init(void)
{
	int i;
	
	DPRINT("rims_mcb_gpio_init: %d GPIO\n", RM_NUM_GPIO);

	/* 
	 * set all GPIOs to inputs by default 
	 */
	for (i = 0; i < RM_NUM_GPIO; i++)
		at91_set_gpio_input(rm_gpio_set[i], 0);

	return 0;
}

/**
 * function to read back the current status of the GPIO lines
 * no shadow register is needed because we can read the current
 * state of output lines on the 9260/9G20
 */
static gpio_data rm_gpio_data_read(struct gpio_s *gpio)
{
	gpio_data value;
	int i;

	DPRINT("rm_gpio_data_read\n");

	value = 0;

	for (i = 0; i < RM_NUM_GPIO; i++)
	{
		value |= (at91_get_gpio_value(rm_gpio_set[i]) << i);
	}


	return value;
}

/**
 * function to set the value of the GPIO lines
 */
static int rm_gpio_data_write(struct gpio_s *gpio, gpio_data data)
{
	int i;

	DPRINT("rm_gpo_data_write: 0x%X\n", data);
	
	for (i = 0; i < RM_NUM_GPIO; i++) 
	{
		at91_set_gpio_value(rm_gpio_set[i], (data >> i) & 1);
		DPRINT("  wrote %d to offset %d\n", (data >> i) & 1, i);
	}

	return 0;
}

/* no index */

/**
 * data direction configuration read
 */
static gpio_data rm_gpio_ddr_read(struct gpio_s *gpio)
{
	return ddr_value_shadow;
}

/**
 * data direction configuration write
 */
static int rm_gpio_ddr_write(struct gpio_s *gpio, gpio_data data)
{
	int i;

	ddr_value_shadow = data;

	for (i = 0; i < RM_NUM_GPIO; i++)
	{
		if(data & (1 << i)) 
		{
			at91_set_gpio_output(rm_gpio_set[i], 0);
			at91_set_multi_drive(rm_gpio_set[i], 0);
		}
		else
		{
			at91_set_gpio_input(rm_gpio_set[i], 0);
		}
	}

	return 0;
}

/**
 * function to register the class for the gpo register
 */
struct device *rims_mcb_gpio_class_create(void)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);

	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = "rims_mcb_gpio";
	gpio->subclass = GPO_SUBCLASS;
	gpio->data_write = rm_gpio_data_write;
	gpio->data_read = rm_gpio_data_read;
	gpio->ddr_write = rm_gpio_ddr_write;
	gpio->ddr_read = rm_gpio_ddr_read;
	printk("registering gpio device: %s\n", gpio->name);

	return gpio_register_device(gpio);
}

void rims_mcb_board_init(void)
{
	rims_mcb_gpio_init();
	rims_mcb_gpio_class_create();
}

