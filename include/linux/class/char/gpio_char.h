#ifndef GPIO_CHAR_H_
#define GPIO_CHAR_H_

#ifdef __KERNEL__
int gpio_char_init(void);
int gpio_char_create(struct gpio_s *gpio);

#ifndef GPIO_MAJOR
#define GPIO_MAJOR 0
#endif

/* we use the max_devs define to register a region on init */
#define GPIO_MAX_DEVS 30
#endif /*__KERNEL__*/

/* This header defines the ioctl commands */
#include <linux/class/rtdm/gpio_rtdm.h>

#define CHAR_CLASS_GPIO RTDM_CLASS_GPIO
/* the following are defined in gpio_rtdm.h */
/*
 * #define DDRREAD         _IOR(RTDM_CLASS_GPIO,0,char)
 * #define DDRWRITE        _IOW(RTDM_CLASS_GPIO,0,char)
 * #define DATAREAD        _IOR(RTDM_CLASS_GPIO,1,char)
 * #define DATAWRITE       _IOW(RTDM_CLASS_GPIO,1,char)
 // additions for the char driver
 * #define INDEXREAD       _IOR(RTDM_CLASS_GPIO,2,char)
 * #define INDEXWRITE      _IOW(RTDM_CLASS_GPIO,2,char)
 */

#endif /*GPIO_CHAR_H_*/
