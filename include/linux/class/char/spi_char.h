#ifndef SPI_CHAR_H_
#define SPI_CHAR_H_

#ifdef __KERNEL__
int spi_char_init(void);
int spi_char_create(struct spi_s *spi);

#ifndef SPI_MAJOR
#define SPI_MAJOR 0
#endif

/* we use the max_devs define to register a region on init */
#define SPI_MAX_DEVS 15
#endif /*__KERNEL__*/

typedef struct spi_transfer_s{
	spi_data *mosi;
	spi_data *miso;
	ssize_t size;
}spi_transfer_t;

/* This header defines the ioctl commands */
#include <linux/class/rtdm/spi_rtdm.h>

#define CHAR_CLASS_SPI RTDM_CLASS_SPI

#endif /*SPI_CHAR_H_*/
