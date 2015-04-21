#ifndef GPIO_RTDM_H_
#define GPIO_RTDM_H_

#include <linux/ioctl.h>

//arbitrary assignment, come back to this later
#define RTDM_CLASS_GPIO 0x80

#ifdef __KERNEL__
int rt_gpio_device_create(struct gpio_s *gpio);
#endif /* __KERNEL__ */

#define DDRREAD		_IOR(RTDM_CLASS_GPIO, 0, char)
#define DDRWRITE	_IOW(RTDM_CLASS_GPIO, 0, char)
#define DATAREAD	_IOR(RTDM_CLASS_GPIO, 1, char)
#define DATAWRITE	_IOW(RTDM_CLASS_GPIO, 1, char)
#define INDEXREAD	_IOR(RTDM_CLASS_GPIO, 2, char)
#define INDEXWRITE	_IOW(RTDM_CLASS_GPIO, 2, char)

#define DDRREAD_NL	_IOR(RTDM_CLASS_GPIO, 3, char)
#define DDRWRITE_NL 	_IOW(RTDM_CLASS_GPIO, 3, char)
#define DATAREAD_NL	_IOR(RTDM_CLASS_GPIO, 4, char)
#define DATAWRITE_NL	_IOW(RTDM_CLASS_GPIO, 4, char)
#define INDEXREAD_NL	_IOR(RTDM_CLASS_GPIO, 5, char)
#define INDEXWRITE_NL	_IOW(RTDM_CLASS_GPIO, 5, char)

#define GPIOLOCK	_IO( RTDM_CLASS_GPIO, 6)
#define GPIOUNLOCK	_IO( RTDM_CLASS_GPIO, 7)

#define DATAREADQ	_IOR(RTDM_CLASS_GPIO, 8, char)
#define GETNOTIFY	_IOR(RTDM_CLASS_GPIO, 9, char)
#define SETNOTIFY	_IOW(RTDM_CLASS_GPIO, 10, char)
#define GETQUEUESIZE	_IOR(RTDM_CLASS_GPIO, 11, char)

#endif /* GPIO_RTDM_H_ */
