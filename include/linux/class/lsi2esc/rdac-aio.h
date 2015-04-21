#ifndef RDAC_AIO_H_
#define RDAC_AIO_H_

#include <linux/class/spi_interface.h>

#ifdef CONFIG_LSI2ESC_RDAC_AIO
struct device *rdac_aio_create(struct spi_s *spi, const char *name);
#else
#define rdac_aio_create(s,n) {}
#endif

#endif /* MCP3208_GPIO_H_ */

