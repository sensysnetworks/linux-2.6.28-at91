#ifndef SPI_RTDM_H_
#define SPI_RTDM_H_

#include <linux/ioctl.h>

//arbitrary assignment, come back to this later
#define RTDM_CLASS_SPI 0x90 

#ifdef __KERNEL__	
int rt_spi_device_create(struct spi_s *spi);
#endif//__KERNEL__

#define CONFREAD		_IOR(RTDM_CLASS_SPI,0,spi_control)
#define CONFWRITE		_IOW(RTDM_CLASS_SPI,0,spi_control)
#define SPEEDREAD		_IOR(RTDM_CLASS_SPI,1,spi_control)
#define SPEEDWRITE		_IOW(RTDM_CLASS_SPI,1,spi_control)
#define TIPREAD			_IOR(RTDM_CLASS_SPI,2,spi_control)
#define TIPWRITE		_IOW(RTDM_CLASS_SPI,2,spi_control)
#define XMIT			_IOW(RTDM_CLASS_SPI,3,spi_transfer_t)

#endif //SPI_RTDM_H_
