#ifndef MCP4922_GPIO_H_
#define MCP4922_GPIO_H_

#include <linux/class/spi_interface.h>

#ifdef CONFIG_LSI2ESC_MCP4922
struct device *mcp4922_gpio_class_create(struct spi_s *spi, const char *name);
#else
#define mcp4922_gpio_class_create(s,n) {}
#endif

#endif /* MCP4922_GPIO_H_ */

