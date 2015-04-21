#ifndef GPIO_H_
#define GPIO_H_

#include <linux/autoconf.h>
#include <linux/device.h>
#include <linux/mutex.h>

#ifdef CONFIG_GPIOCLASS

typedef u32 gpio_data;

struct gpio_s;

/**
 * structure allowing generic interrupt registration per gpio devices.
 * Currently only used by the character device interface, but could be
 * expanded in the future.
 * @irq: the interrupt associated with this device
 * @irq_flags: the flags to use when registering the irq
 * @handler: interrupt handler to be wrapped by generic gpio irq handler.
 *   the handler function returns a bit mask for the notification
 *   functionality. Note that the device mutex will be locked before calling
 *   this function.
 * @get_queue: function to determine what data to add to the queue based on a
 *   notification bitmask (i.e. may be used to read a particular index). Note
 *   that the device mutex will be locked before calling this function.
 * @irq_config: function to configure the irq and enable it. This should be
 *   called after request_irq in the gpio class driver.
 * @change_notify: function called by the gpio_char driver on SETNOTIFY ioctl
 *   and file release to inform the driver of a change in the notification
 *   mask. Usage is implementation specific (i.e. to keep track of which
 *   interrupts should be enabled).
 */
struct gpio_irq_data {
	int irq;
	unsigned int irq_flags;
	gpio_data (*handler)(struct gpio_s *gpio);
	gpio_data (*get_queue)(struct gpio_s *gpio, gpio_data notify);
	void (*irq_config)(struct gpio_s *gpio);
	void (*change_notify)(struct gpio_s *gpio, gpio_data curr_notify,
			gpio_data new_notify);
};

/**********************
 * gpio class structure
 */
typedef struct gpio_s {
	const char *name;
	int subclass;
	gpio_data index;
	gpio_data range;
	void *ddr;
	void *data;
	gpio_data shadow;
	 gpio_data(*data_read) (struct gpio_s * gpio);
	int (*data_write) (struct gpio_s * gpio, gpio_data data);
	 gpio_data(*ddr_read) (struct gpio_s * gpio);
	int (*ddr_write) (struct gpio_s * gpio, gpio_data data);
	 gpio_data(*index_read) (struct gpio_s * gpio);
	int (*index_write) (struct gpio_s * gpio, gpio_data data);
	struct mutex lock;
	struct gpio_irq_data irq_data;
} gpio_t;

#define GPIO_BASECLASNUM	0xA0
#define GPIO_SUBCLASS 	(GPIO_BASECLASNUM+0)
#define GPI_SUBCLASS 	(GPIO_BASECLASNUM+1)
#define GPO_SUBCLASS 	(GPIO_BASECLASNUM+2)

/***************************************************************************
 * typical low level methods for accessing 8 bit ports
 */
int gpio_ddr_write8(gpio_t * gpio, gpio_data data);
int gpio_data_write8(gpio_t * gpio, gpio_data data);
int gpio_index_write(gpio_t * gpio, gpio_data data);
int gpio_empty_write(gpio_t * gpio, gpio_data data);
gpio_data gpio_ddr_read8(gpio_t * gpio);
gpio_data gpio_data_read8(gpio_t * gpio);
gpio_data gpio_index_read(gpio_t * gpio);
gpio_data gpio_shadow_read8(gpio_t * gpio);
gpio_data gpio_ff_read(gpio_t * gpio);
gpio_data gpio_zero_read(gpio_t * gpio);

/***************************************************************************
 * initial class declaration, doesn't hurt to call it mulitiple times,
 * automatically checked during device instantiation 
 */
struct class *gpio_declare(void);

/***************************************************************************
 * class instantiation
 */
struct device *gpio_register_device(gpio_t * gpio);

/***************************************************************************
 * atomic method wrappers
 * these should be used for all method calls to maintain sychronization across the
 * various interfaces
 */
int atomic_gpio_ddr_write(gpio_t * gpio, gpio_data data);
int atomic_gpio_ddr_write_lock(gpio_t * gpio, gpio_data data);
gpio_data atomic_gpio_ddr_read(gpio_t * gpio);
gpio_data atomic_gpio_ddr_read_lock(gpio_t * gpio);
int atomic_gpio_data_write(gpio_t * gpio, gpio_data data);
int atomic_gpio_data_write_lock(gpio_t * gpio, gpio_data data);
gpio_data atomic_gpio_data_read(gpio_t * gpio);
gpio_data atomic_gpio_data_read_lock(gpio_t * gpio);
int atomic_gpio_index_write(gpio_t * gpio, gpio_data data);
int atomic_gpio_index_write_lock(gpio_t * gpio, gpio_data data);
gpio_data atomic_gpio_index_read(gpio_t * gpio);
gpio_data atomic_gpio_index_read_lock(gpio_t * gpio);

/************************************************************************************
 * full gpio ports, 
 * bit configurable gpio ports with a bidirectional data register
 * if the data register or ddr are NULL their interfaces are not created.
 */
static inline struct device *gpio_device_create(void *data, void *ddr,
						const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->ddr = ddr;
	gpio->data = data;
	gpio->shadow = 0;
	gpio->data_write = gpio_data_write8;
	gpio->data_read = gpio_data_read8;

	if (ddr) {
		gpio->ddr_write = gpio_ddr_write8;
		gpio->ddr_read = gpio_ddr_read8;
	}
	printk("registering gpio device: %s\n", name);
	return gpio_register_device(gpio);
}

/* create a gpio_s struct and initialize for gpio port without registering it */

static inline struct gpio_s *gpio_device_create_unregistered(void *data,
							     void *ddr,
							     const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->ddr = ddr;
	gpio->data = data;
	gpio->shadow = 0;
	gpio->data_write = gpio_data_write8;
	gpio->data_read = gpio_data_read8;

	if (ddr) {
		gpio->ddr_write = gpio_ddr_write8;
		gpio->ddr_read = gpio_ddr_read8;
	}
	return gpio;
}

/************************************************************************************
 * shadow gpo ports, 
 * these ports are write only and have no register to read back the current state
 * a shadow port is therefore created to keep track of the current state in software.
 * this is not as complete an implementation from a hardware standpoint as a user can only
 * see the hardware changes if a rogue process if it "plays fair" and modifies the shadow register.
 * It is, however, much cheaper from a gate implementation standpoint and probably just fine for
 * most applications.
 */

static inline struct device *gpo_device_create(void *data, const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPO_SUBCLASS;
	gpio->data = data;
	gpio->shadow = 0;
	gpio->data_write = gpio_data_write8;
	gpio->data_read = gpio_shadow_read8;
	gpio->ddr_write = gpio_empty_write;
	gpio->ddr_read = gpio_ff_read;
	printk("registering gpo device: %s\n", name);
	return gpio_register_device(gpio);
}

static inline struct gpio_s *gpo_device_create_unregistered(void *data,
							    const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPO_SUBCLASS;
	gpio->data = data;
	gpio->shadow = 0;
	gpio->data_write = gpio_data_write8;
	gpio->data_read = gpio_shadow_read8;
	gpio->ddr_write = gpio_empty_write;
	gpio->ddr_read = gpio_ff_read;
	return gpio;
}

/************************************************************************************
 * gpi ports, gpi ports are functionally the same as gpio input only ports,
 * but their ddr ports are not implemented in hardware 
 * and are assumed based upon the PLD key. 
 * This is a cheaper gate implementation. 
 */
static inline struct device *gpi_device_create(void *data, const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPI_SUBCLASS;
	gpio->data = data;
	gpio->shadow = 0;
	gpio->data_write = gpio_empty_write;
	gpio->data_read = gpio_data_read8;
	gpio->ddr_write = gpio_empty_write;
	gpio->ddr_read = gpio_zero_read;
	printk("registering gpi device: %s\n", name);
	return gpio_register_device(gpio);
}

/* create a gpio_s struct and initialize for gpi port without registering it */

static inline struct gpio_s *gpi_device_create_unregistered(void *data,
							    const char *name)
{
	gpio_t *gpio = kmalloc(sizeof(gpio_t), GFP_KERNEL);
	memset(gpio, 0, sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPI_SUBCLASS;
	gpio->data = data;
	gpio->shadow = 0;
	gpio->data_write = gpio_empty_write;
	gpio->data_read = gpio_data_read8;
	gpio->ddr_write = gpio_empty_write;
	gpio->ddr_read = gpio_zero_read;
	return gpio;
}

#endif

#endif /* GPIO_H_ */
