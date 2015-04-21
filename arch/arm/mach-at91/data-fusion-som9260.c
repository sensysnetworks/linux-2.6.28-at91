/*
 * arch/arm/mach-at91/data-fusion-som9260.c
 * EMAC.Inc SOM9260m support for Compass Systems Data Fusion
 * Carrier board.
 *
 * Copyright (C) 2007 EMAC.Inc <support@emacinc.com>
 */

#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <mach/board.h>
#include <mach/at91sam9260.h>
#include <mach/gpio.h>

/* #define DF_DEBUG */

#ifdef DF_DEBUG
#define DPRINT(string, args...) printk(string, ##args)
#else
#define DPRINT(args...)
#endif


/**
 * static array defining digital output set
 */
static unsigned df_gpo_set[] = {
	AT91_PIN_PC13,  /* VIS ON  */
	AT91_PIN_PB16,  /* LAS ON  */
	AT91_PIN_PB17,  /* GPS ON  */
	AT91_PIN_PB18,  /* IR ON   */
	AT91_PIN_PB19,  /* IND ON  */
	AT91_PIN_PB20,  /* AHRS ON */
};

/* GPO register bit-map */
#define DF_VIS_ON_OFFSET  (0)
#define DF_LAS_ON_OFFSET  (1)
#define DF_GPS_ON_OFFSET  (2)
#define DF_IR_ON_OFFSET   (3)
#define DF_IND_ON_OFFSET  (4)
#define DF_AHRS_ON_OFFSET (5)

#define DF_NUM_GPO ARRAY_SIZE(df_gpo_set)

/**
 * static array defining digital input set
 */
static unsigned df_gpi_set[] = {
	AT91_PIN_PB21,  /* TRIG ON    */
	AT91_PIN_PB0,   /* VIS FAULT  */
	AT91_PIN_PB1,   /* LAS FAULT  */
	AT91_PIN_PB2,   /* GPS FAULT  */
	AT91_PIN_PB3,   /* IR FAULT   */
	AT91_PIN_PB30,  /* IND FAULT  */
	AT91_PIN_PC6,   /* AHRS FAULT */
	AT91_PIN_PC9,   /* GPS STATUS */
};

/* GPI register bit-map */
#define DF_TRIG_ON_OFFSET    (0)
#define DF_VIS_FAULT_OFFSET  (1)
#define DF_LAS_FAULT_OFFSET  (2)
#define DF_GPS_FAULT_OFFSET  (3)
#define DF_IR_FAULT_OFFSET   (4)
#define DF_IND_FAULT_OFFSET  (5)
#define DF_AHRS_FAULT_OFFSET (6)
#define DF_GPS_STATUS_OFFSET (7)

#define DF_NUM_GPI ARRAY_SIZE(df_gpi_set)

#define DF_GET_GPO_BIT(name) \
	(at91_get_gpio_value(df_gpo_set[DF_##name##_OFFSET]) << DF_##name##_OFFSET)

#define DF_GET_GPI_BIT(name) \
	(at91_get_gpio_value(df_gpi_set[DF_##name##_OFFSET]) << DF_##name##_OFFSET)

#define DF_GPO_BIT_VALUE(data, offset) \
	((data & (1 << offset)) >> offset)

/**
 * function to initialize the digital inputs and outputs
 */
static int data_fusion_gpio_init(void)
{
	int i;
	
	DPRINT("data_fusion_gpio_init: %d GPI and %d GPO\n", DF_NUM_GPI, DF_NUM_GPO);
	/* set all GPIs to GPIO peripheral and input
	 * external pull-ups are provided 
	 */
	for (i = 0; i < DF_NUM_GPI; i++)
		at91_set_gpio_input(df_gpo_set[i], 0);

	/* set all GPOs to GPIO peripheral and output
	 * multi-drive is off for all of these */
	for (i = 0; i < DF_NUM_GPO; i++) {
		at91_set_gpio_output(df_gpo_set[i], 0);
		at91_set_multi_drive(df_gpo_set[i], 0);
	}

	return 0;
}

/* There are two classes gpio devices created, one for the GPOs and one for the GPIs */

#ifdef DF_DEBUG

void df_show_gpo(void)
{
	DPRINT("VIS_ON : 0x%X\n", at91_get_gpio_value(df_gpo_set[DF_VIS_ON_OFFSET]));
	DPRINT("LAS_ON : 0x%X\n", at91_get_gpio_value(df_gpo_set[DF_LAS_ON_OFFSET]));
	DPRINT("GPS_ON : 0x%X\n", at91_get_gpio_value(df_gpo_set[DF_GPS_ON_OFFSET]));
	DPRINT("IR_ON  : 0x%X\n", at91_get_gpio_value(df_gpo_set[DF_IR_ON_OFFSET]));
	DPRINT("IND_ON : 0x%X\n", at91_get_gpio_value(df_gpo_set[DF_IND_ON_OFFSET]));
	DPRINT("AHRS_ON: 0x%X\n", at91_get_gpio_value(df_gpo_set[DF_AHRS_ON_OFFSET]));
}

void df_show_gpi(void)
{
	DPRINT("TRIG_ON   : 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_TRIG_ON_OFFSET]));
	DPRINT("VIS_FAULT : 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_VIS_FAULT_OFFSET]));
	DPRINT("LAS_FAULT : 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_LAS_FAULT_OFFSET]));
	DPRINT("GPS_FAULT : 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_GPS_FAULT_OFFSET]));
	DPRINT("IR_FAULT  : 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_IR_FAULT_OFFSET]));
	DPRINT("IND_FAULT : 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_IND_FAULT_OFFSET]));
	DPRINT("AHRS_FAULT: 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_AHRS_FAULT_OFFSET]));
	DPRINT("GPS_STATUS: 0x%X\n", at91_get_gpio_value(df_gpi_set[DF_GPS_STATUS_OFFSET]));
}

#else
#define df_show_gpo()
#define df_show_gpi()
#endif

/**
 * function to read back the current status of the GPO lines
 * no shadow register is needed because we can read the current
 * state of output lines on the 9260
 */
static gpio_data df_gpo_data_read(struct gpio_s *gpio)
{
	DPRINT("df_gpo_data_read\n");
	df_show_gpo();

	return (DF_GET_GPO_BIT(VIS_ON) |
			DF_GET_GPO_BIT(LAS_ON) |
			DF_GET_GPO_BIT(GPS_ON) |
			DF_GET_GPO_BIT(IR_ON) |
			DF_GET_GPO_BIT(IND_ON) |
			DF_GET_GPO_BIT(AHRS_ON));
}

/**
 * function to read back the current status of the GPI lines
 */
static gpio_data df_gpi_data_read(struct gpio_s *gpio)
{
	DPRINT("df_gpi_data_read\n");
	df_show_gpi();

	return (DF_GET_GPI_BIT(TRIG_ON) |
			DF_GET_GPI_BIT(VIS_FAULT) |
			DF_GET_GPI_BIT(LAS_FAULT) |
			DF_GET_GPI_BIT(GPS_FAULT) |
			DF_GET_GPI_BIT(IR_FAULT) |
			DF_GET_GPI_BIT(IND_FAULT) |
			DF_GET_GPI_BIT(AHRS_FAULT) |
			DF_GET_GPI_BIT(GPS_STATUS));
}

/**
 * function to set the value of the GPO lines
 */
static int df_gpo_data_write(struct gpio_s *gpio, gpio_data data)
{
	int i;
	DPRINT("df_gpo_data_write: 0x%X\n", data);
	/* loop through all lines and set value based on single bit in data */
	for (i = 0; i < DF_NUM_GPO; i++) {
		at91_set_gpio_value(df_gpo_set[i], DF_GPO_BIT_VALUE(data, i));
		DPRINT("  wrote %d to offset %d\n", DF_GPO_BIT_VALUE(data, i), i);
	}

	return 0;
}

/* no index */
/* no ddr   */

/**
 * function to register the class for the gpo register
 */
struct device *data_fusion_gpo_class_create(void)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);

	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = "data_fusion_gpo";
	gpio->subclass = GPO_SUBCLASS;
	gpio->data_write = df_gpo_data_write;
	gpio->data_read = df_gpo_data_read;
	printk("registering data fusion gpo device: %s\n", gpio->name);
	return gpio_register_device(gpio);
}

/**
 * function to register the class for the gpi register
 */
struct device *data_fusion_gpi_class_create(void)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);

	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = "data_fusion_gpi";
	gpio->subclass = GPI_SUBCLASS;
	gpio->data_write = gpio_empty_write;
	gpio->data_read = df_gpi_data_read;
	printk("registering data fusion gpi device: %s\n", gpio->name);
	return gpio_register_device(gpio);
}

void data_fusion_board_init(void)
{
	data_fusion_gpio_init();
	data_fusion_gpo_class_create();
	data_fusion_gpi_class_create();
}

