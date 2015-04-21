#ifndef MCP3208_GPIO_H_
#define MCP3208_GPIO_H_

#include <linux/class/spi_interface.h>

#ifdef CONFIG_LSI2ESC_MCP3208
struct device *mcp3208_gpio_class_create(struct spi_s *spi, const char *name);
#else
#define mcp3208_gpio_class_create(s,n) {}
#endif

#endif /* MCP3208_GPIO_H_ */

