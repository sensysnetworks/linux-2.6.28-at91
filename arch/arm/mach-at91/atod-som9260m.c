/*
 * arch/arm/mach-at91/atod-som9260m.c
 * EMAC.Inc SOM9260m low level AtoD interactions
 *
 * Copyright (C) 2007 EMAC.Inc <support@emacinc.com>
 */


#include "atod-som9260m.h"

/* Global Variables Used in the Driver. */
int drdy;
void __iomem *adc_base;
struct clk *adc_clk;


int som9260m_atod_init(void)
{
	adc_clk = clk_get(NULL, "adc_clk");
	clk_enable(adc_clk);

	at91_set_A_periph(AT91_PIN_PC0,0);
	at91_set_A_periph(AT91_PIN_PC1,0);
	at91_set_A_periph(AT91_PIN_PC2,0);
#ifndef CONFIG_RDAC_CARRIER
	at91_set_A_periph(AT91_PIN_PC3,0);
#endif

	/* Set up the ADC */
	adc_base = ioremap(AT91SAM9260_BASE_ADC, SZ_16K);
	/* Mode setup */
	__raw_writel((SHTIM << 24 | STARTUP << 16 | PRESCAL << 8 | SLEEP_MODE << 5 | 
				LOWRES << 4 | TRGSEL << 1 | TRGEN), adc_base + ADC_MR);
	/* Disable Channels 1-3 */
	__raw_writel((0x1 << 0)^(0xF), adc_base + ADC_CHDR);
	/* Enable Channel 0 */
	__raw_writel((0x1 << 0), adc_base + ADC_CHER);
	return 0;
}

int som9260m_atod_switch(int channel)
{
	/* Disable Channels */	
	__raw_writel((0x1 << channel)^(0xF), adc_base + ADC_CHDR);
	/* Enable specified channel */
	__raw_writel((0x1 << channel), adc_base + ADC_CHER);

	return 0;
}

int som9260m_atod_read_current(void)
{
	int data;

	drdy = __raw_readl(adc_base + ADC_SR);
	if (drdy & 0x10000) {
		data = __raw_readl(adc_base + ADC_LCDR);
		return data;
	}
	else
		return -1;
	return (data);
}

static gpio_data atod_data_read(struct gpio_s *gpio)
{
	int data;

	/* start the conversion */	
	__raw_writel(0x02, adc_base + ADC_CR);
	/* loop to get the first available data */
	do {
		data = som9260m_atod_read_current();
	} while (data == -1);

	return data;	
}

static gpio_data atod_index_read(struct gpio_s *gpio)
{
	return gpio->index;
}

static int atod_index_write(struct gpio_s *gpio,gpio_data index)
{
	if(index > 3)
		return -1;
	if(index < 0)
		return -1;
	gpio->index = index;
	som9260m_atod_switch(index);	
	return 0;
}

struct device *som9260m_atod_class_create(const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data_write = gpio_empty_write;
	gpio->data_read = atod_data_read;
	gpio->index_write = atod_index_write;
	gpio->index_read = atod_index_read;
	gpio->index = 0;
	printk("registering indexed atod device: %s\n",name);
	return gpio_register_device(gpio);
}
