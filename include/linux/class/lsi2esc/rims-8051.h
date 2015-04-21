#ifndef RIMS_8051_H_
#define RIMS_8051_H_

#include <linux/class/spi_interface.h>

#ifdef CONFIG_LSI2ESC_RIMS_8051
struct device *rims8051_gpio_class_create(struct spi_s *spi, const char *name);
#else
#define rims8051_gpio_class_create(s,n) {}
#endif

#endif /* RIMS_8051_H_ */

