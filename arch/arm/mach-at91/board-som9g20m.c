/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *  Copyright (C) 2009 EMAC Inc.
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
#include <linux/spi/flash.h>
#include <linux/spi/cs4271.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mtd/plat-ram.h>
#include <linux/delay.h>

#ifdef CONFIG_SPICLASS
#include <linux/class/spi.h>
#include <linux/class/spi_interface.h>
#endif /* CONFIG_SPICLASS */

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <mach/at91sam9_smc.h>

#include <linux/ioex/ecoreex.h>
#include <linux/keypad.h>
#include <mach/at91_tc.h>
#include <linux/mtd/mtd.h>

#include <linux/class/lsi2esc/mcp3208-gpio.h>
#include <linux/class/lsi2esc/mcp4922-gpio.h>
#include <linux/class/lsi2esc/rdac-aio.h>

#include <mach/rdac-interrupts.h>
#include <mach/rdac-uart.h>
#include "sam9_smc.h"
#include "generic.h"
#include "keypad.h"
#include "atod-som9260m.h"
#include "rims-mcb-som9x.h"
#include "ucar-som9x.h"

static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DBGU on ttyS5 */
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

#ifdef CONFIG_SOM9G20M_EXTRA_SERIAL
	/* USART4 on ttyS6 */
	at91_register_uart(AT91SAM9260_ID_US4, 6, 0);
#endif

	/* set serial console to ttyS3 */
	at91_set_serial_console(3);	
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

#ifdef CONFIG_RDAC_CARRIER
static void __init rdac_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/*
	 * Serial map for the RDAC rev1 board is as follows:
	 * ttyS0 / COMC: CN1 full handshake
	 * ttyS1 / COMB: HDR6 RTS/CTS and 485
	 * ttyS2 / CAN : TT Modem RTS/CTS
	 * ttyS3 / COMA: ISM Radio RTS/CTS
	 * ttyS4 / USART5: Linux Console HDR4 Rx/Tx
	 * ttyS5 / DBGU: Not connected
	 * ttyS6 / extra: Not connected
	 */

	/* initialize modem status handling functions */
	rdac_uart_fns_init();

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
	at91_register_uart(AT91SAM9260_ID_US2, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* USART3 on ttyS3. (Rx, Tx) */
	/* CTS3 and RTS3 are used for NOR addressing and PLD CS */
	/* CTS3 is shared with A25 -- needed for 64 MB flash */
	at91_register_uart(AT91SAM9260_ID_US3, 3, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* USART5 on ttyS4. (Rx & Tx only) */
	at91_register_uart(AT91SAM9260_ID_US5, 4, 0);

	/* set serial console to ttyS4 */
	at91_set_serial_console(4);
}
#endif

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


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
 * I2C
 */
static struct i2c_board_info __initdata apcc_i2c_devices[] = {
	/*
	 * The two power monitors in the Sensys system
	 * live at the same I2C address. To overcome this limitation
	 * in the ds2746 driver, we just have an attribute that
	 * turns on the I2C clock of the one we want to talk to.
	 */
	{
		I2C_BOARD_INFO("ds2746", 0x36), 
		.type	= "ds2746",
	},
	{
		I2C_BOARD_INFO("at24c1024", 0x54),
		.type	= "at24c1024",
		/*.platform_data = &sensys_i2c_eeprom;*/
	},
};

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

#ifdef CONFIG_RDAC_CARRIER
static struct spi_s btaio_spi = 
{
	.name = "aio_spi",
	.subclass = 0,
	.tip = lsi2esc_spi_tip,
	.xmit = lsi2esc_spi_xmit,
	.confwrite = lsi2esc_spi_confwrite,
	.confread = lsi2esc_spi_confread,
	.speedread = lsi2esc_spi_speedread,
	.speedwrite = lsi2esc_spi_speedwrite,
	.gpio_name = "aio_gpio",
	.gpio_create = rdac_aio_create,
	.gpio_data = NULL,
};

static struct spi_s spi1cs1 = 
{
	.name = "spi1cs1",
	.subclass = 0,
	.tip = lsi2esc_spi_tip,
	.xmit = lsi2esc_spi_xmit,
	.confwrite = lsi2esc_spi_confwrite,
	.confread = lsi2esc_spi_confread,
	.speedread = lsi2esc_spi_speedread,
	.speedwrite = lsi2esc_spi_speedwrite,
	.gpio_name = NULL,
	.gpio_create = NULL,
	.gpio_data = NULL,
};

static struct spi_s spi1cs2 = 
{
	.name = "spi1cs2",
	.subclass = 0,
	.tip = lsi2esc_spi_tip,
	.xmit = lsi2esc_spi_xmit,
	.confwrite = lsi2esc_spi_confwrite,
	.confread = lsi2esc_spi_confread,
	.speedread = lsi2esc_spi_speedread,
	.speedwrite = lsi2esc_spi_speedwrite,
	.gpio_name = NULL,
	.gpio_create = NULL,
	.gpio_data = NULL,
};

static struct spi_s spi1cs3 = 
{
	.name = "spi1cs3",
	.subclass = 0,
	.tip = lsi2esc_spi_tip,
	.xmit = lsi2esc_spi_xmit,
	.confwrite = lsi2esc_spi_confwrite,
	.confread = lsi2esc_spi_confread,
	.speedread = lsi2esc_spi_speedread,
	.speedwrite = lsi2esc_spi_speedwrite,
	.gpio_name = NULL,
	.gpio_create = NULL,
	.gpio_data = NULL,
};

#endif

static struct mtd_partition spi_flash_partitions[] =
{
   /* Note there are 8 pages per block, so put all sections on a block boundary */
   {
      .name = "df_boot",
      .offset = 0,
      .size = 0x4200,
   },
   {
      .name = "df_env0",
      .offset = MTDPART_OFS_NXTBLK,
      .size = 0x2100
   },
   {
      .name = "df_env1",
      .offset = MTDPART_OFS_NXTBLK,
      .size = 0x2100
   },
   {
      .name = "df_uboot",
      .offset = MTDPART_OFS_NXTBLK,
      .size = 0x39c00
   },
   {
      .name = "df_kernel0",
      .offset = MTDPART_OFS_NXTBLK,
      .size = 0x1bf900
   },
   {
      .name = "df_kernel1",
      .offset = MTDPART_OFS_NXTBLK,
      .size = 0x1bf900
   },
   {
      .name = "df_fpga0",
      .offset = MTDPART_OFS_NXTBLK,
      .size = 0x8400
   },
   {
      .name = "df_fpga1",
      .offset = MTDPART_OFS_NXTBLK,
      .size = 0x8400
   },
   {
      .name = "df_aux",
      .offset = MTDPART_OFS_NXTBLK,
      .size = MTDPART_SIZ_FULL,
   },
};

static struct flash_platform_data spi_df = {
   .name = "spi_flash",
   .parts = spi_flash_partitions,
   .nr_parts = ARRAY_SIZE(spi_flash_partitions)
};

static struct spi_board_info ek_spi_devices[] = {
#if defined(CONFIG_SND_SOC_CS4271) || defined(CONFIG_SND_SOC_CS4271_MODULE)
	{	/* CS4271 codec */
		.modalias	= "cs4271",
#if defined(CONFIG_SOM_150ES_REV2) || defined(CONFIG_SOM_150ES_REV3) || defined(CONFIG_RDAC_CARRIER)
		.chip_select	= 2,
		.bus_num	= 0,
#else
		.chip_select	= 0,
		.bus_num	= 1,
#endif
		.max_speed_hz	= 1 * 1000 * 1000,
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

#ifdef CONFIG_RDAC_CARRIER
	{ /* RDAC Analog I/O board */
		.modalias = "lsi2esc", /* use the SPI class interface */
		.chip_select = 0,
		.bus_num = 1,
		.max_speed_hz = 1e6,
		.platform_data = &btaio_spi,
	},
	{ /* SPI1 CS1 test */
		.modalias = "lsi2esc",
		.chip_select = 1,
		.bus_num = 1,
		.max_speed_hz = 1e6,
		.platform_data = &spi1cs1,
	},
	{ /* SPI1 CS2 test */
		.modalias = "lsi2esc",
		.chip_select = 2,
		.bus_num = 1,
		.max_speed_hz = 1e6,
		.platform_data = &spi1cs2,
	},
	{ /* SPI1 CS3 test */
		.modalias = "lsi2esc",
		.chip_select = 3,
		.bus_num = 1,
		.max_speed_hz = 1e6,
		.platform_data = &spi1cs3,
	},
#if 0
	{ /* SPI1 CS4 test */
		.modalias = "lsi2esc",
		.chip_select = 4,
		.bus_num = 1,
		.max_speed_hz = 1e6,
		.platform_data = &spi1cs4,
	},
	{ /* SPI1 CS5 test */
		.modalias = "lsi2esc",
		.chip_select = 5,
		.bus_num = 1,
		.max_speed_hz = 1e6,
		.platform_data = &spi1cs5,
	},
#endif /* 0 */
#endif
	{ /* DataFlash Interface */
		.modalias = "mtd_dataflash", /* Uses the MTD SPI Flash driver */
#if defined (CONFIG_SOM_150ES_REV2) || defined(CONFIG_SOM_150ES_REV3)
		.chip_select = 3,
#else
		.chip_select = 1,
#endif 
		.max_speed_hz = 12000000, /* 12 MHz in low-freq mode */
		.bus_num = 0,
		.platform_data = &spi_df,
	},
#ifdef CONFIG_RIMS_MCB
	/* RIMS has 8 general purpose SPI Chip selects */
	RIMS_LSI2ESC_CHIP(0, 0, 1e6) /* 8051 */
	RIMS_LSI2ESC_CHIP(1, 2, 1e6) /* Module 1 */
	RIMS_LSI2ESC_CHIP(2, 3, 1e6) /* Module 2 */
	RIMS_LSI2ESC_CHIP(3, 4, 1e6) /* Module 3 */
	RIMS_LSI2ESC_CHIP(4, 5, 1e6) /* Moudle 4 */
	RIMS_LSI2ESC_CHIP(5, 6, 1e6) /* Spare 1 */
	RIMS_LSI2ESC_CHIP(6, 7, 1e6) /* Spare 2 */
#endif
};

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA28,
	.is_rmii	= 1,
};

/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name   = "Bootstrap",
		.offset = 0,
		.size   = SZ_4M,
	},
	{
		.name	= "Partition 1",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 60 * SZ_1M,
	},
	{
		.name	= "Partition 2",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init ek_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

	at91_add_device_nand(&ek_nand_data);
}

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

#define CONFIG_APCC

/*
 * MCI (SD/MMC)
 * wp_pin and vcc_pin are not connected
 */
static struct at91_mmc_data __initdata ek_mmc_data = {
#ifdef CONFIG_APCC
	.slot_b		= 0,
#elif defined(CONFIG_SOM_150ES_REV1)
	.slot_b		= 1,
#else
	.slot_b		= 0,
#endif
	.wire4		= 1,

#ifdef CONFIG_APCC
	.det_pin	= AT91_PIN_PB21,
#elif defined(CONFIG_SOM_150ES_REV3) || defined(CONFIG_RDAC_CARRIER)
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

/************************************************************
 * IOEX Device
 * Onboard EMAC I/O core platform device used by the ecoreex driver
 */

#define CS0_START ((unsigned long)0x10000000)
#define CS0_END   ((unsigned long)0x1FFFFFFF)

#define CS4_START ((unsigned long)0x50000000)
#define CS4_END  ((unsigned long)0x5FFFFFFF)
#define CPLDKEY 0xF

static struct ecoreex_data som9260m_ecoreex_data = {
	.key_offset	= CPLDKEY,
#ifdef CONFIG_KEYPAD
	.keypad		= &keypad_data,
#endif
	.irq		= AT91SAM9260_ID_IRQ1,
};

static struct resource som9260m_ecoreex_resource = {
#ifdef CONFIG_RDAC_CARRIER
	.start	= CS0_START,
	.end	= CS0_END,
#else
	.start	= CS4_START,
	.end	= CS4_END,
#endif
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

#define SETUP 2
#define PULSE 7
#define HOLD  2
struct sam9_smc_config pld_smc_config = {
	.ncs_read_setup		= 2,
	.nrd_setup		= 3,
	.ncs_write_setup	= SETUP,
	.nwe_setup		= SETUP+1,

	.ncs_read_pulse		= 9,
	.nrd_pulse		= 7,
	.ncs_write_pulse	= PULSE+2,
	.nwe_pulse		= PULSE,

	.read_cycle		= 13,
	.write_cycle		= SETUP+PULSE+HOLD+3,

	.mode			= AT91_SMC_READMODE | \
				  AT91_SMC_WRITEMODE | \
				  AT91_SMC_EXNWMODE_DISABLE | \
				  AT91_SMC_DBW_8,
	.tdf_cycles		= 0x0F,
};

#ifdef CONFIG_RDAC_CARRIER
#define PLD_CS	(0)
#else
#define PLD_CS	(4)
#endif

static inline void pld_cs_setup(void)
{
	sam9_smc_configure(PLD_CS, &pld_smc_config);
	if (PLD_CS == 4) {
		at91_set_A_periph(AT91_PIN_PC8, 1);
	}
#ifdef CONFIG_RIMS_MCB
	/* Uses two chips selects on the EBI */
	sam9_smc_configure(0, &pld_smc_config);
#endif	
}

#define MCLK (33000000)
#define RC_VAL 18

#define at91_tc_read(reg)		__raw_readl(reg)
#define at91_tc_write(reg, val)	__raw_writel((val), reg)

static void set_tc_clocks(void)
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
#endif

	/* set to  14.3 MHz (16.5 Mhz) */
	at91_tc_write((tc0 + AT91_TC_CMR),
			AT91_TC_TIMER_CLOCK1 |
			AT91_TC_WAVE |
			AT91_TC_EEVT_XC0 |
			AT91_TC_WAVESEL_UP_AUTO |
			AT91_TC_ACPA_TOGGLE
		     );
	/* set the frequency */
	at91_tc_write((tc0 + AT91_TC_RC),(MCLK/11000000));

	/* set RA to < RC for toggle (50% duty) */
	at91_tc_write((tc0 + AT91_TC_RA),((MCLK/11000000)/2));

#ifdef CONFIG_RDAC_CARRIER
	/* set to 800 Hz */
	at91_tc_write((tc2 + AT91_TC_CMR),
			AT91_TC_TIMER_CLOCK5 | /* slow clock */
			AT91_TC_WAVE |
			AT91_TC_EEVT_XC0 |
			AT91_TC_WAVESEL_UP_AUTO |
			AT91_TC_BCPB_TOGGLE
		     );

	/* set the frequency */
	at91_tc_write((tc2 + AT91_TC_RC), RC_VAL);

	/* set RB to < RC for toggle (50% duty) */
	at91_tc_write((tc2 + AT91_TC_RB), RC_VAL / 2);
#else
	/* set to 8 MHz */
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
#endif

	/* reset and start the timer */
	at91_tc_write((tc0 + AT91_TC_CCR), AT91_TC_CLKEN | AT91_TC_SWTRG);	
	at91_tc_write((tc1 + AT91_TC_CCR), AT91_TC_CLKEN | AT91_TC_SWTRG);
	at91_tc_write((tc2 + AT91_TC_CCR), AT91_TC_CLKEN | AT91_TC_SWTRG);

	/* enable timer functionality on PC7, TIOB1 */
	at91_set_A_periph(AT91_PIN_PC7, 1);
	at91_set_A_periph(AT91_PIN_PA26, 1);
	at91_set_A_periph(AT91_PIN_PC6, 1);

}

static inline void at91_add_device_ecoreex4(void)
{
	pld_cs_setup();
#ifndef CONFIG_RIMS_MCB
	set_tc_clocks();
#endif

#ifdef CONFIG_RDAC_CARRIER	
	/* Toggle reset on the Audio Codec */
	at91_set_gpio_output(AT91_PIN_PC3, 0);
	udelay(1000);
	at91_set_gpio_output(AT91_PIN_PC3, 1);
#endif
	
	at91_set_B_periph(AT91_PIN_PC15, 1); /* enable IRQ1 on PC15 */
	platform_device_register(&som9260m_ecoreex_device);
}

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
	at91_set_gpio_value(AT91_PIN_PA25,(char)data);
	return 0;
}

static gpio_data physw_read(struct gpio_s *gpio)
{
	return at91_get_gpio_value(AT91_PIN_PA25);
}

static inline struct device *som9g20m_physw(void){
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	at91_set_gpio_output(AT91_PIN_PA25, 1);
	gpio->name = "physw";
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data_write = physw_write;
	gpio->data_read = physw_read;
	printk("registering gpio device: %s\n",gpio->name);
	return gpio_register_device(gpio);
}

static int anasw_write(struct gpio_s *gpio,gpio_data data)
{
	at91_set_gpio_value(AT91_PIN_PA22,(char)data ^ 1);
	return 0;
}

static gpio_data anasw_read(struct gpio_s *gpio)
{
	return at91_get_gpio_value(AT91_PIN_PA22) ^ 1;
}

static inline struct device *som9g20m_anasw(void){
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	at91_set_gpio_output(AT91_PIN_PA22, 0);
	gpio->name = "anasw";
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data_write = anasw_write;
	gpio->data_read = anasw_read;
	printk("registering gpio device: %s\n",gpio->name);
	return gpio_register_device(gpio);
}


/************************************************************
 * device registrations from the arch go here, 
 * which are called by the boardspec ioex driver
 */
static int som9260m_classes(void)
{
#ifndef CONFIG_UCAR_GPIO
	som9260m_atod_init();
	som9260m_atod_class_create("indexed_atod");
#endif
	som9260m_led_init();
	som9260m_led_class_create("indexed_led");

	som9260m_rtscls();
	som9g20m_physw();
	som9g20m_anasw();

	return 0;
}

static struct platform_device boardspec_device = {
	.name = "boardspec",
	.id = 1,
	.dev		= {
		.platform_data	= &som9260m_classes,
	},
};

static inline void at91_add_device_boardspec(void)
{
	platform_device_register(&boardspec_device);
}

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
	/* NAND */
	ek_add_device_nand();
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
	at91_add_device_mmc(0, &ek_mmc_data);
	/* SSC */
	at91_add_device_ssc(AT91SAM9260_ID_SSC, ATMEL_SSC_TX | ATMEL_SSC_RD | ATMEL_SSC_RF);
	/* I2C */
	at91_add_device_i2c(apcc_i2c_devices, ARRAY_SIZE(apcc_i2c_devices));
	/* EMAC Core Extension */
#ifdef CONFIG_KEYPAD
	keypad_init(&keypad_data);
#endif
	at91_add_device_ecoreex4();
	/* Board specific devices */
	at91_add_device_boardspec();

	/* eVent medical board support */
	event_medical_board_init();
	/* RIMS MCB board support */
	rims_mcb_board_init();
	/* UCAR GPIO support */
	ucar_board_init();	
}

MACHINE_START(AT91SAM9G20EK, "EMAC SOM-9G20M")
/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
#ifdef CONFIG_RIMS_MCB
	.map_io		= rims_map_io,
#elif defined (CONFIG_RDAC_CARRIER)
	.map_io		= rdac_map_io,
#else
	.map_io		= ek_map_io,
#endif
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
