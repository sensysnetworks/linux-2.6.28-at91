/*
 * arch/arm/mach-at91/led-som9260m.c
 * EMAC.Inc SOM9260m low level led interactions
 *
 * Copyright (C) 2007 EMAC.Inc <support@emacinc.com>
 */

#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <mach/board.h>
#include <mach/at91sam9260.h>
#include <mach/gpio.h>

/* Global Variables Used in the Driver. */

#ifdef CONFIG_DENM_CARRIER

#define LED_CNT 3 /* PA9 or PB20 might be LED depending on SOM rev */
static int led_array[LED_CNT] = { AT91_PIN_PA9, AT91_PIN_PA10, AT91_PIN_PB20 };
static char led_value[LED_CNT] = { 0, 0, 0 };

#else 
#if defined(CONFIG_SOM_150ES_REV3) || defined(CONFIG_SOM_150ES_REV2) || defined(CONFIG_RDAC_CARRIER) || defined(CONFIG_RIMS_MCB) /* can't use PA9 -- MMC */
#define LED_CNT 1
static int led_array[LED_CNT] = { AT91_PIN_PB20 };
static char led_value[LED_CNT] = { 0 };

#else

/* Could be PB20 or PA9 depending on SOM rev */
#define LED_CNT 2
static int led_array[LED_CNT] = { AT91_PIN_PB20, AT91_PIN_PA9 };
static char led_value[LED_CNT] = { 0, 0 };
#endif /* CONFIG_SOM_150ES_REV2 */
#endif

int som9260m_led_init(void)
{
	int i;

	for(i = 0; i < LED_CNT; i++)
		at91_set_gpio_output(led_array[i],led_value[i]);
	return 0;
}

static gpio_data led_data_read(struct gpio_s *gpio)
{
	return at91_get_gpio_value(led_array[gpio->index]);
}

static int led_data_write(struct gpio_s *gpio,gpio_data data)
{
	led_value[gpio->index] = (char)data;
	at91_set_gpio_value(led_array[gpio->index],led_value[gpio->index]);
	return 0;
}

static gpio_data led_index_read(struct gpio_s *gpio)
{
	return gpio->index;
}

static int led_index_write(struct gpio_s *gpio, gpio_data index)
{
	if (index >= LED_CNT)
		return -1;
	if (index < 0)
		return -1;
	gpio->index = index;
	return 0;
}

struct device *som9260m_led_class_create(const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data_write = led_data_write;
	gpio->data_read = led_data_read;
	gpio->index_write = led_index_write;
	gpio->index_read = led_index_read;
	gpio->index = 0;
	printk("registering indexed led device: %s\n",name);
	return gpio_register_device(gpio);
}

