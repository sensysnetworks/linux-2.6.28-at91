/*
 * linux/arch/arm/mach-at91rm9200/board-som9260m.c
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2006 Atmel
 *  Copyright (C) 2009 EMAC.Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/cs4271.h>
#include <linux/clk.h>
#include <linux/ioex/ecoreex.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/plat-ram.h>
#include <linux/interrupt.h>

#ifdef CONFIG_SPICLASS
#include <linux/class/spi.h>
#include <linux/class/spi_interface.h>
#endif /* CONFIG_SPICLASS */

#include <linux/class/lsi2esc/mcp3208-gpio.h>
#include <linux/class/lsi2esc/mcp4922-gpio.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9260.h>
#include <mach/at91_shdwc.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_tc.h>

#include "generic.h"
#include "keypad.h"
#include "atod-som9260m.h"
#include "data-fusion-som9260.h"
#include "rims-mcb-som9x.h"

static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DGBU on ttyS5. (Rx & Tx only) */
	at91_register_uart(0, 5, 0);

	/* USART0 on ttyS0. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
	at91_register_uart(AT91SAM9260_ID_US0, 0, ATMEL_UART_CTS | ATMEL_UART_RTS
			| ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
			| ATMEL_UART_RI);

	/* USART1 on ttyS1. (Rx, Tx, RTS, CTS) */
	at91_register_uart(AT91SAM9260_ID_US1, 1, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* USART2 on ttyS2. (Rx, Tx) */
	/* 
	 * CTS2 and RTS2 (PA4 and PA5) are used for SPI_CS1 and SPI_CS2 in the
	 * SoM Spec
	 */
	at91_register_uart(AT91SAM9260_ID_US2, 2, 0);

	/* USART3 on ttyS3. (Rx, Tx) */
	/* CTS3 and RTS3 are used for NOR addressing and PLD CS */
	/* CTS3 is shared with A25 -- needed for 64 MB flash */
	at91_register_uart(AT91SAM9260_ID_US3, 3, 0);

	/* USART5 on ttyS4. (Rx & Tx only) */
	at91_register_uart(AT91SAM9260_ID_US5, 4, 0);

	/* set serial console to ttyS3 */
	at91_set_serial_console(3);	
}

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}

#ifdef CONFIG_RIMS_MCB
static void __init rims_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DBG UART on ttyS5 */
	at91_register_uart(0, 5, 0);

	/* USART3 on ttyS3. */
	at91_register_uart(AT91SAM9260_ID_US3, 3, 0);

	/* set serial console to ttyS3 */
	at91_set_serial_console(3);	
}
#endif

/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PB31,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

/*
 * Audio
 */
static struct cs4271_board_info cs4271_data = {
	.ssc_id		= 0,
	.shortname	= "CS4271 Audio Codec",
};

#if defined(CONFIG_SND_CS4271) || defined(CONFIG_SND_CS4271_MODULE)
static void __init cs4271_set_clk(struct cs4271_board_info *info)
{
	struct clk *pck0;
	struct clk *plla;

	pck0 = clk_get(NULL, "pck0");
	plla = clk_get(NULL, "plla");

	/* MCK Clock */
	at91_set_A_periph(AT91_PIN_PB30, 0);	/* PCK0 */

	clk_set_parent(pck0, plla);

	clk_put(plla);

	info->dac_clk = pck0;
}
#else
static void __init cs4271_set_clk(struct cs4271_board_info *info) {}
#endif

/*
 * SPI devices.
 */

/************************************************************
 * mcp3208 atod interface over EMAC SPI class
 */
#if defined(CONFIG_SOM_150ES_REV2) || defined(CONFIG_SOM_150ES_REV3)
#ifdef CONFIG_LSI2ESC_MCP3208
static struct spi_s mcp3208_spi = 
{
	.name = "mcp3208",
	.subclass = 0,
	.tip = lsi2esc_spi_tip,
	.xmit = lsi2esc_spi_xmit,
	.confwrite = lsi2esc_spi_confwrite,
	.confread = lsi2esc_spi_confread,
	.speedread = lsi2esc_spi_speedread,
	.speedwrite = lsi2esc_spi_speedwrite,
	.gpio_name = "mcp3208-gpio",
	.gpio_create = mcp3208_gpio_class_create,
	.gpio_data = NULL,
};
#endif

#ifdef CONFIG_LSI2ESC_MCP4922
static struct spi_s mcp4922_spi = 
{
	.name = "mcp4922",
	.subclass = 0,
	.tip = lsi2esc_spi_tip,
	.xmit = lsi2esc_spi_xmit,
	.confwrite = lsi2esc_spi_confwrite,
	.confread = lsi2esc_spi_confread,
	.speedread = lsi2esc_spi_speedread,
	.speedwrite = lsi2esc_spi_speedwrite,
	.gpio_name = "mcp4922-gpio",
	.gpio_create = mcp4922_gpio_class_create,
	.gpio_data = NULL,
};
#endif
#endif

static struct spi_s at25fs010_spi = 
{
	.name = "at25fs010",
	.subclass = 0,
	.tip = lsi2esc_spi_tip,
	.xmit = lsi2esc_spi_xmit,
	.confwrite = lsi2esc_spi_confwrite,
	.confread = lsi2esc_spi_confread,
	.speedread = lsi2esc_spi_speedread,
	.speedwrite = lsi2esc_spi_speedwrite,
};


static struct spi_board_info ek_spi_devices[] = {
#if defined(CONFIG_SND_CS4271) || defined(CONFIG_SND_CS4271_MODULE)
	{	/* CS4271 codec */
		.modalias	= "cs4271",
#if defined(CONFIG_SOM_150ES_REV2) || defined(CONFIG_SOM_150ES_REV3)
		.chip_select	= 2,
		.bus_num	= 0,
#else
		.chip_select	= 0,
		.bus_num	= 1,
#endif
		.max_speed_hz	= 6 * 1000 * 1000,
		.platform_data	= &cs4271_data,
	},
#endif
#if defined(CONFIG_SOM_150ES_REV2) || defined(CONFIG_SOM_150ES_REV3)
#ifdef CONFIG_LSI2ESC_MCP3208
	{ /* mcp3208 ADC */
		.modalias = "lsi2esc", /* use the SPI class interface */
		.chip_select = 0,
		.bus_num = 0,
		.max_speed_hz = 1e6,
		.platform_data = &mcp3208_spi,
	},
#endif
#ifdef CONFIG_LSI2ESC_MCP4922
	{ /* mcp4922 DAC */
		.modalias = "lsi2esc", /* use the SPI class interface */
		.chip_select = 1,
		.bus_num = 0,
		.max_speed_hz = 15e6,
		.platform_data = &mcp4922_spi,
	},
#endif
#endif
	{ /* SEEPROM Interface */
		.modalias = "lsi2esc", /* use the SPI class interface */
		.chip_select = 1,
		.max_speed_hz = 50 * 1000 * 1000,
		.bus_num = 0,
		.platform_data = &at25fs010_spi,	
	},
#ifdef CONFIG_RIMS_MCB
	/* RIMS has 8 general purpose SPI Chip selects */
	RIMS_LSI2ESC_CHIP(0,0,1e6) /* 8051 */
	RIMS_LSI2ESC_CHIP(1,2,1e6) /* Module 1 */
	RIMS_LSI2ESC_CHIP(2,3,1e6) /* Module 2 */
	RIMS_LSI2ESC_CHIP(3,4,1e6) /* Module 3 */
	RIMS_LSI2ESC_CHIP(4,5,1e6) /* Moudle 4 */
	RIMS_LSI2ESC_CHIP(5,6,1e6) /* Spare 1 */
	RIMS_LSI2ESC_CHIP(6,7,1e6) /* Spare 2 */
#endif
};

/*
 * MACB Ethernet device
 */
static struct __initdata at91_eth_data ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA30,
	.is_rmii	= 0,
};


/*
 * NAND flash
 */
#ifdef  CONFIG_MACH_SOM9260M_NAND
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "Boot Partition",
		.offset	= 0,
		.size	= 64 * 2 * 1024,
	},
	{
		.name	= "UBoot Partition",
		.offset	= 64 * 2 * 1024,
		.size	= 64 * 14 * 1024,
	},
	{
		.name	= "Kernel Partition",
		.offset	= 64 * 16 * 1024,
		.size	= 64 * 48 * 1024,
	},
	{
		.name	= "Disk Partition",
		.offset	= 64 * 64 * 1024,
		.size	= 64 * 256 * 1024,
	},
};


static struct mtd_partition *nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

static struct at91_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	//	.det_pin	= ... not connected
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_AT91_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};
#else

/*
 *  NOR Flash
 */

#define SOM_FLASH_BASE	0x10000000

#ifdef CONFIG_MACH_SOM9260M_16
#define SOM_FLASH_SIZE	0x1000000
#else
#define SOM_FLASH_SIZE	0x2000000
#endif

static struct mtd_partition som_flash_partitions[] = {
	{
		.name	= "Boot Partition",
		.offset	= 0,
		.size	= 64 * 2 * 1024, /* ~125 KB */
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name	= "U-Boot Partition",
		.offset	= 64 * 2 * 1024,
		.size	= 64 * 14 * 1024, /* ~875 KB */
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name	= "Kernel Partition",
		.offset	= 64 * 16 * 1024,
		.size	= 64 * 48 * 1024, /* 3 MB */
	},
	{
		.name	= "Disk Partition",
		.offset	= 64 * 64 * 1024,
		.size = SOM_FLASH_SIZE - (64 * 64 * 1024),
	},
#ifdef CONFIG_MACH_SOM9260M_64
	{
		.name	= "Auxilliary Partition",
		.offset	= SOM_FLASH_SIZE,
		.size	= SOM_FLASH_SIZE, /* 32 MB */
	},
#endif
};

static struct physmap_flash_data som_flash_data = {
	.width		= 2,
	.parts		= som_flash_partitions,
	.nr_parts	= ARRAY_SIZE(som_flash_partitions),
};

static struct resource som_flash_resources[] = {
	{
		.start	= SOM_FLASH_BASE,
		.end	= SOM_FLASH_BASE + SOM_FLASH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#ifdef CONFIG_MACH_SOM9260M_64
	{ /* 2nd bank of 32 MB */
		.start	= SOM_FLASH_BASE + SOM_FLASH_SIZE,
		.end	= SOM_FLASH_BASE + SOM_FLASH_SIZE + SOM_FLASH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	}
#endif
};

static struct platform_device som_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data = &som_flash_data,
	},
	.resource	= som_flash_resources,
	.num_resources	= ARRAY_SIZE(som_flash_resources),
};
#endif

#ifdef CONFIG_EVENT_MEDICAL
static struct platdata_mtd_ram som_ram_data = {
	.mapname		= "Dual Port RAM",
	.bankwidth		= 1,
};

static struct resource som_ram_resources[] = {
	{
		.start	= 0x50000000,
		.end	= 0x500007ff,
		.flags	= IORESOURCE_MEM,
	},
};


static struct platform_device som_ram = {
	.name		= "mtd-ram",
	.id		= 1,
	.dev		= {
		.platform_data = &som_ram_data,
	},
	.resource	= som_ram_resources,
	.num_resources	= ARRAY_SIZE(som_ram_resources),
};
#endif

/*
 * MCI (SD/MMC)
 */
static struct at91_mmc_data __initdata ek_mmc_data = {
#ifdef CONFIG_SOM_150ES_REV1 
	.slot_b		= 1,
#else
	.slot_b		= 0,
#endif
	.wire4		= 1,
#if defined(CONFIG_SOM_150ES_REV3)
	.det_pin	= AT91_PIN_PA29,
#else
	.det_pin	= AT91_PIN_PB21,
#endif
};


/*
 * keypad device
 */
#ifdef CONFIG_KEYPAD
static struct keypad_data_s keypad_data;
#endif

#ifdef CONFIG_ECOREEX_SOM9260M

/************************************************************
 * IOEX Device
 * Onboard EMAC I/O core platform device used by the ecoreex driver
 */
#define CS4_START ((unsigned long)0x50000000)
#define CS4_END  ((unsigned long)0x5FFFFFFF)

/* key is defined, but may be overridden by the ecoreex driver
 * based on the static key definition in the config file
 */
#define CPLDKEY 0xF

static struct ecoreex_data som9260m_ecoreex_data = {
	.key_offset	= CPLDKEY,
#ifdef CONFIG_KEYPAD
	.keypad		= &keypad_data,
#endif
	.irq = AT91SAM9260_ID_IRQ1,
};

static struct resource som9260m_ecoreex_resource = {
	.start	= CS4_START,
	.end	= CS4_END,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device som9260m_ecoreex_device = {
	.name		= "ecoreex",
	.id		= -1,
	.dev		= {
		.platform_data	= &som9260m_ecoreex_data,
	},
	.num_resources	= 1,
	.resource	= &som9260m_ecoreex_resource,
};

/**
 * enable a generic memory map on cs4 and required module I/O
 */
static inline void cs4_setup(void)
{
	at91_sys_write(AT91_SMC_SETUP(4),
			0
		      );

	at91_sys_write(AT91_SMC_PULSE(4),
			AT91_SMC_NWEPULSE_(4)|
			AT91_SMC_NCS_WRPULSE_(6)|
			AT91_SMC_NRDPULSE_(3)|
			AT91_SMC_NCS_RDPULSE_(5)
		      );

	at91_sys_write(AT91_SMC_CYCLE(4),
			AT91_SMC_NWECYCLE_(5)|
			AT91_SMC_NRDCYCLE_(6) 
		      );

	at91_sys_write(AT91_SMC_MODE(4),
			AT91_SMC_READMODE|
			AT91_SMC_WRITEMODE|
			AT91_SMC_EXNWMODE_DISABLE|
			AT91_SMC_DBW_8|
			AT91_SMC_TDF_(0X0f)
		      );
	//at91_sys_write(AT91C_PIOC_PDR,0x100);
	//at91_sys_write(AT91C_PIOC_OER,0x100);

	at91_set_A_periph(AT91_PIN_PC8, 1);	
}

/**
 * creates the 200k clock in using timer1 outputB
 * NOTES: hard coded clock values could be more flexibly done through the clock registery, 
 * but it would require some 
 * fixed point math, which will be put off for a full PWM implementation
 * at which point it's methods will replace this code block.
 */

#define MCLK (25000000)
#define RC_VAL 20

#define at91_tc_read(reg)		__raw_readl(reg)
#define at91_tc_write(reg, val)	__raw_writel((val), reg)

static inline void set200kclock(void)
{
	struct clk *tc0_clk = clk_get(NULL, "tc0_clk");
	struct clk *tc1_clk = clk_get(NULL, "tc1_clk");
	struct clk *tc2_clk = clk_get(NULL, "tc2_clk");
	unsigned long tc0 = (unsigned long)ioremap(AT91SAM9260_BASE_TC0, SZ_16K);
	unsigned long tc1 = (unsigned long)ioremap(AT91SAM9260_BASE_TC1, SZ_16K);
	unsigned long tc2 = (unsigned long)ioremap(AT91SAM9260_BASE_TC2, SZ_16K);

	clk_enable(tc0_clk); /* enable tc0, tc1, tc2 clock in the pmc */
	clk_enable(tc1_clk);
	clk_enable(tc2_clk);

	/* tc1 setup for waveform on TCIOB1 */

#ifdef CONFIG_SOM_9260_1K_TC
	/* Set to 800 Hz rather than 200 KHz */
	at91_tc_write((tc1 + AT91_TC_CMR),
			AT91_TC_TIMER_CLOCK5 | /* slow clock */
			AT91_TC_WAVE |
			AT91_TC_EEVT_XC0 |
			AT91_TC_WAVESEL_UP_AUTO |
			AT91_TC_BCPB_TOGGLE
		     );
	/* set the frequency */
	at91_tc_write((tc1 + AT91_TC_RC), RC_VAL);

	/* set RB to < RC for toggle (50% duty) */
	at91_tc_write((tc1 + AT91_TC_RB), RC_VAL / 2);
	
#else
	/* set to  12.5 MHz (as fast as it can go) */
	at91_tc_write((tc0 + AT91_TC_CMR),
			AT91_TC_TIMER_CLOCK1 |
			AT91_TC_WAVE |
			AT91_TC_EEVT_XC0 |
			AT91_TC_WAVESEL_UP_AUTO |
			AT91_TC_BCPB_TOGGLE
		     );
	/* set the frequency */
	 at91_tc_write((tc0 + AT91_TC_RC),(MCLK/12500000));

		/* set RB to < RC for toggle (50% duty) */
	at91_tc_write((tc0 + AT91_TC_RB),((MCLK/12500000)/2));

	/* set to 200 KHz */
	at91_tc_write((tc1 + AT91_TC_CMR),
			AT91_TC_TIMER_CLOCK1 |
			AT91_TC_WAVE |
			AT91_TC_EEVT_XC0 |
			AT91_TC_WAVESEL_UP_AUTO |
			AT91_TC_BCPB_TOGGLE
		     );
	/* set the frequency */
	 at91_tc_write((tc1 + AT91_TC_RC),(MCLK/200000));

		/* set RB to < RC for toggle (50% duty) */
	at91_tc_write((tc1 + AT91_TC_RB),((MCLK/200000)/2));

	/* set to  8 MHz */
	at91_tc_write((tc2 + AT91_TC_CMR),
			AT91_TC_TIMER_CLOCK1 |
			AT91_TC_WAVE |
			AT91_TC_EEVT_XC0 |
			AT91_TC_WAVESEL_UP_AUTO |
			AT91_TC_BCPB_TOGGLE
		     );
	/* set the frequency */
	 at91_tc_write((tc2 + AT91_TC_RC),(MCLK/8000000));

		/* set RB to < RC for toggle (50% duty) */
	at91_tc_write((tc2 + AT91_TC_RB),((MCLK/8000000)/2));
	
#endif /* CONFIG_SOM_9260_1K_TC */
	
	/* reset and start the timer */
	at91_tc_write((tc0 + AT91_TC_CCR), AT91_TC_CLKEN | AT91_TC_SWTRG);	
	at91_tc_write((tc1 + AT91_TC_CCR), AT91_TC_CLKEN | AT91_TC_SWTRG);
	at91_tc_write((tc2 + AT91_TC_CCR), AT91_TC_CLKEN | AT91_TC_SWTRG);

	/* enable timer functionality on PC7, TIOB1 */
	at91_set_A_periph(AT91_PIN_PC7, 1);
	at91_set_B_periph(AT91_PIN_PC9, 1);
	at91_set_A_periph(AT91_PIN_PC6, 1);


}

static inline void at91_add_device_ecoreex4(void)
{
	cs4_setup();
#ifndef CONFIG_RIMS_MCB
	set200kclock();
#endif
	at91_set_A_periph(AT91_PIN_PC15, 1); /* enable IRQ1 on PC15 */
	platform_device_register(&som9260m_ecoreex_device);
}
#else
#define at91_add_device_ecoreex4()
#endif /* CONFIG_ECOREEX_SOM9260M */

#ifdef CONFIG_EVENT_MEDICAL
static inline void event_medical_board_init(void)
{
	at91_sys_write(AT91_SMC_SETUP(4),0x0e0e0e0e);
	at91_sys_write(AT91_SMC_PULSE(4),0x0e0e0e0e);
	at91_sys_write(AT91_SMC_CYCLE(4),0x002f002f);
	at91_sys_write(AT91_SMC_MODE(4) ,0x00000003);
	at91_set_A_periph(AT91_PIN_PC8, 1);

	platform_device_register(&som_ram);
}
#else
#define event_medical_board_init()
#endif

/************************************************************/
#ifdef CONFIG_RTSCLS
#define GPIOBIT			0
#define AUTORTSBIT		1
#define GPIOMASK		(1<<GPIOBIT)
#define AUTORTSMASK		(1<<AUTORTSBIT)
#define RS485CHANNEL 	1

static int is_auto_toggle = 0;
/**
 * set the rts line manually or put it in auto mode.
 * bit 1 (0x02) is the auto rts control 0-off 1-on
 * bit 0 (0x01) is the state of the pin, this cannot be set manually while in auto mode.
 */
static int rts_data_write(struct gpio_s *gpio, gpio_data data)
{
	if (data & AUTORTSMASK) {
		if (!is_auto_toggle) { /* if auto toggle is already set, do nothing */
			at91_set_A_periph(AT91_PIN_PB28, 1);//return RTS to primary functionality
			at91_auto485_serial(RS485CHANNEL, 1);//call down into the driver to enable RTS auto toggle
			is_auto_toggle = 1;
		}
	}
	else {
		if (is_auto_toggle) { /* change from peripheral to GPIO and set value */
			at91_auto485_serial(RS485CHANNEL, 0);//Auto control off
			at91_set_gpio_output(AT91_PIN_PB28, (char)(data & GPIOMASK));//set data manually
			is_auto_toggle = 0;
		}
		else { /* already set to GPIO, change the value of the line */
			at91_set_gpio_value(AT91_PIN_PB28, (char)(data & GPIOMASK));
		}
	}
	return 0;
}

/**
 * returns the current auto flow control state and 
 * state of the pins by reading the hardware registers.
 */
static gpio_data rts_data_read(struct gpio_s *gpio)
{
	return (at91_get_gpio_value(AT91_PIN_PB28) | (at91_auto485_serial(RS485CHANNEL, 2) << AUTORTSBIT));
}

/**
 * Create gpio rts interface through boardspec
 */
static inline struct device *som9260m_rtscls(void)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	is_auto_toggle = 0;
	at91_set_gpio_output(AT91_PIN_PB28, 1); /* rts line */
	gpio->name = "rtsctl";
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data_write = rts_data_write;
	gpio->data_read = rts_data_read;
	printk("registering gpio device: %s\n",gpio->name);
	return gpio_register_device(gpio);	
}

#else /* CONFIG_RTSCLS */
#define som9260m_rtscls()
#endif

static int physw_write(struct gpio_s *gpio,gpio_data data)
{
	at91_set_gpio_value(AT91_PIN_PA31,(char)data);
	return 0;
}

static gpio_data physw_read(struct gpio_s *gpio)
{
	return at91_get_gpio_value(AT91_PIN_PA31);
}

static inline struct device *som9260m_physw(void){
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	at91_set_gpio_output(AT91_PIN_PA31, 0);
	gpio->name = "physw";
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data_write = physw_write;
	gpio->data_read = physw_read;
	printk("registering gpio device: %s\n",gpio->name);
	return gpio_register_device(gpio);
}

/************************************************************
 * device registrations from the arch go here, 
 * which are called by the boardspec ioex driver
 */
static int som9260m_classes(void)
{
	som9260m_atod_init();
	som9260m_atod_class_create("indexed_atod");

	som9260m_led_init();
	som9260m_led_class_create("indexed_led");

	som9260m_rtscls();
	som9260m_physw();

	//set wakeup mask to all uarts
	set_irq_wake(AT91SAM9260_ID_US0,1);
	set_irq_wake(AT91SAM9260_ID_US1,1);
	set_irq_wake(AT91SAM9260_ID_US2,1);
	set_irq_wake(AT91SAM9260_ID_US3,1);
	set_irq_wake(AT91SAM9260_ID_US4,1);
	//	set_irq_wake(AT91SAM9260_ID_US5,1);

	return 0;
}

static struct platform_device boardspec_device = {
	.name = "boardspec",
	.id = 1,
	.dev		= {
		.platform_data	= &som9260m_classes,
	},
};

static inline void at91_add_device_boardspec(void){
	platform_device_register(&boardspec_device);
}
/************************************************************/

static void __init ek_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
	at91_add_device_mmc(0, &ek_mmc_data);

	/* SSC (to cs4271) */
	cs4271_set_clk(&cs4271_data);
	at91_add_device_ssc(AT91SAM9260_ID_SSC, ATMEL_SSC_TX);

	/* I2C if used (i.e. data fusion) */
	at91_add_device_i2c(NULL, 0);

#ifdef CONFIG_KEYPAD
	keypad_init(&keypad_data);
#endif
	/* EMAC Core Extensions */
	at91_add_device_ecoreex4();
	/* Board Specific */
	at91_add_device_boardspec();

	/* Data fusion board support */
	data_fusion_board_init();
	/* eVent medical board support */
	event_medical_board_init();
	/* RIMS MCB board support */
	rims_mcb_board_init();

#ifdef  CONFIG_MACH_SOM9260M_NAND
	/* NAND */
	at91_add_device_nand(&ek_nand_data);
#else
	/* NOR Flash */
	platform_device_register(&som_flash);
#endif
}

MACHINE_START(AT91SAM9260EK, "EMAC SoM-9260M")
	/* Maintainer: EMAC.Inc */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
#ifdef CONFIG_RIMS_MCB
	.map_io		= rims_map_io,
#else
	.map_io		= ek_map_io,
#endif
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END

