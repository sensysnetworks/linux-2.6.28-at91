#ifndef _GPIO_RDAC_H_
#define _GPIO_RDAC_H_

/**
 * GPIO definitions for the SPI CS lines on the RDAC Carrier board. These are
 * necessary to provide a less complicated interface to the SPI driver.
 */

#define RDAC_PIN_BASE (AT91_PIN_PE31 + 1)

#define RDAC_PIN_SPI1CS0 (RDAC_PIN_BASE + 0)
#define RDAC_PIN_SPI1CS1 (RDAC_PIN_BASE + 1)
#define RDAC_PIN_SPI1CS2 (RDAC_PIN_BASE + 2)
#define RDAC_PIN_SPI1CS3 (RDAC_PIN_BASE + 3)
#define RDAC_PIN_SPI1CS4 (RDAC_PIN_BASE + 4)
#define RDAC_PIN_SPI1CS5 (RDAC_PIN_BASE + 5)

void rdac_init_spi_cs(u8 *rdac_spi);

#endif /* _GPIO_RDAC_H_ */

