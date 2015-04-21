#ifndef RIMS_MCB_SOM9X_H_
#define RIMS_MCB_SOM9X_H_

#ifdef CONFIG_RIMS_MCB

#include <linux/class/lsi2esc/rims-8051.h>

void rims_mcb_board_init(void);

#define RIMS_LSI2ESC_CHIP(num,cs,speed) \
	{ \
		.modalias = "lsi2esc", \
		.chip_select = cs, \
		.bus_num = 0, \
		.max_speed_hz = speed, \
		.platform_data = &rims_mcb_cs[num], \
	},

#define RIMS_LSI2ESC_CHIP_SELECT(dname,gname,cfunc) \
	{ \
		.name = dname, \
		.subclass = 0, \
		.tip = lsi2esc_spi_tip, \
		.xmit = lsi2esc_spi_xmit, \
		.confwrite = lsi2esc_spi_confwrite, \
		.confread = lsi2esc_spi_confread, \
		.speedread = lsi2esc_spi_speedread, \
		.speedwrite = lsi2esc_spi_speedwrite, \
		.gpio_name = gname, \
		.gpio_create = cfunc, \
		.gpio_data = NULL, \
	}, 


static struct spi_s rims_mcb_cs[]  = 
{
#ifdef CONFIG_LSI2ESC_RIMS_8051
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_8051", "rims_8051-gpio", rims8051_gpio_class_create) /* 8051 */
#else
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_8051", NULL, NULL) /* 8051 */
#endif
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_mod1", NULL, NULL) /* Module 1 */
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_mod2", NULL, NULL) /* Module 2 */
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_mod3", NULL, NULL) /* Module 3 */
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_mod4", NULL, NULL) /* Module 4 */
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_spr1", NULL, NULL) /* Spare 1 */
	RIMS_LSI2ESC_CHIP_SELECT("rims_cs_spr2", NULL, NULL) /* Spare 2 */
};

#else
#define rims_mcb_board_init()
#endif

#endif /* RIMS_MCB_SOM9X_H_ */
