/**
 * A class for simple gpio ports
 * Several types of general purpose devices are available 
 * which all export the same basic functionity 
 * through different underlying methods
 * This class can also be used to export simple interfaces
 * to an 8 bit port into user space
 */
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/kdev_t.h>
#include <linux/chelper.h>
#include <linux/class/gpio.h>
#include <asm/io.h>

#ifdef CONFIG_GPIOCLASS_RTDM
#include <rtdm/rtdm_driver.h>
#include <linux/class/rtdm/gpio_rtdm.h>
#define ATOMIC(a) RTDM_EXECUTE_ATOMICALLY(a)
#else
#define ATOMIC(a) a
#endif //CONFIG_GPIOCLASS_RTDM

#ifdef CONFIG_GPIOCLASS_CHAR
#include <linux/class/char/gpio_char.h>
#endif

/************************************************************
 * the global device class
 */
static struct class *gpioclass = NULL;
static DEFINE_MUTEX(gpio_lock); /* race on declare */

struct class *gpio_declare(void)
{
	if (mutex_lock_interruptible(&gpio_lock))
		return gpioclass; /* NULL or the desired class */
	if (!gpioclass) {
		printk("registering GPIO class\n");
		gpioclass = class_create(THIS_MODULE, "gpio");
#ifdef CONFIG_GPIOCLASS_CHAR
		gpio_char_init();
#endif
	}
	mutex_unlock(&gpio_lock);
	return gpioclass;
}

/***************************************************************************
 * typical low level methods for accessing 8 bit ports
 */

int gpio_ddr_write8(gpio_t * gpio, gpio_data data)
{
	iowrite8(data, gpio->ddr + gpio->index);
	return 0;
}

int gpio_data_write8(gpio_t * gpio, gpio_data data)
{
	iowrite8(data, gpio->data + gpio->index);
	gpio->shadow = data;
	return 0;
}

int gpio_index_write(gpio_t * gpio, gpio_data data)
{
	data = (data > gpio->range) ? gpio->range : data;
	gpio->index = data;
	return 0;
}

int gpio_empty_write(gpio_t * gpio, gpio_data data)
{
	return 0;
}

gpio_data gpio_ddr_read8(gpio_t * gpio)
{
	return ioread8(gpio->ddr + gpio->index);
}

gpio_data gpio_data_read8(gpio_t * gpio)
{
	return ioread8(gpio->data + gpio->index);
}

gpio_data gpio_index_read(gpio_t * gpio)
{
	return gpio->index;
}

gpio_data gpio_shadow_read8(gpio_t * gpio)
{
	return gpio->shadow;
}

gpio_data gpio_ff_read(gpio_t * gpio)
{
	return 0xff;
}

gpio_data gpio_zero_read(gpio_t * gpio)
{
	return 0;
}

/***************************************************************************
 * Atomic method wrappers
 */
int atomic_gpio_ddr_write(gpio_t * gpio, gpio_data data)
{
	ATOMIC(gpio->ddr_write(gpio, data);
	    )
	    return 0;
}

gpio_data atomic_gpio_ddr_read(gpio_t * gpio)
{
	gpio_data retval;
	ATOMIC(retval = gpio->ddr_read(gpio);
	    )
	    return retval;
}

int atomic_gpio_data_write(gpio_t * gpio, gpio_data data)
{
	ATOMIC(gpio->data_write(gpio, data);
	    )
	    return 0;
}

gpio_data atomic_gpio_data_read(gpio_t * gpio)
{
	gpio_data retval;
	ATOMIC(retval = gpio->data_read(gpio);
	    )
	    return retval;
}

int atomic_gpio_index_write(gpio_t * gpio, gpio_data data)
{
	ATOMIC(gpio->index_write(gpio, data);
	    )
	    return 0;
}

gpio_data atomic_gpio_index_read(gpio_t * gpio)
{
	gpio_data retval;
	ATOMIC(retval = gpio->index_read(gpio);
	    )
	    return retval;
}

/**
 * functions for accessing the device while acquiring the device mutex.
 */
int atomic_gpio_ddr_write_lock(gpio_t *gpio, gpio_data data)
{
	if (mutex_lock_interruptible(&gpio->lock))
		return 1;
	atomic_gpio_ddr_write(gpio, data);
	mutex_unlock(&gpio->lock);
	return 0;
}

gpio_data atomic_gpio_ddr_read_lock(gpio_t *gpio)
{
	gpio_data ret;

	if (mutex_lock_interruptible(&gpio->lock))
		return 0;
	ret = atomic_gpio_ddr_read(gpio);
	mutex_unlock(&gpio->lock);
	return ret;
}

int atomic_gpio_data_write_lock(gpio_t *gpio, gpio_data data)
{
	if (mutex_lock_interruptible(&gpio->lock))
		return 1;
	atomic_gpio_data_write(gpio, data);
	mutex_unlock(&gpio->lock);
	return 0;
}

gpio_data atomic_gpio_data_read_lock(gpio_t *gpio)
{
	gpio_data ret;

	if (mutex_lock_interruptible(&gpio->lock))
		return 0;
	ret = atomic_gpio_data_read(gpio);
	mutex_unlock(&gpio->lock);
	return ret;
}

int atomic_gpio_index_write_lock(gpio_t *gpio, gpio_data data)
{
	if (mutex_lock_interruptible(&gpio->lock))
		return 1;
	atomic_gpio_index_write(gpio, data);
	mutex_unlock(&gpio->lock);
	return 0;
}

gpio_data atomic_gpio_index_read_lock(gpio_t *gpio)
{
	gpio_data ret;

	if (mutex_lock_interruptible(&gpio->lock))
		return 0;
	ret = atomic_gpio_index_read(gpio);
	mutex_unlock(&gpio->lock);
	return ret;
}

/***************************************************************************
 * gpio sysfs operations
 */
#ifdef CONFIG_GPIOCLASS_SYSFS
static ssize_t gpio_data_store(struct device *cls,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	size_t size;
	gpio_t *gpio = dev_get_drvdata(cls);
	gpio_data data = sfs_getint(buf, count, &size);
	atomic_gpio_data_write_lock(gpio, data);
	return size;
}

static ssize_t gpio_ddr_store(struct device *cls, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	size_t size;
	gpio_t *gpio = dev_get_drvdata(cls);
	gpio_data data = sfs_getint(buf, count, &size);
	atomic_gpio_ddr_write_lock(gpio, data);
	return size;
}

static ssize_t gpio_index_store(struct device *cls,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	size_t size;
	gpio_t *gpio = dev_get_drvdata(cls);
	gpio_data data = sfs_getint(buf, count, &size);
	atomic_gpio_index_write_lock(gpio, data);
	return size;
}

static ssize_t gpio_data_show(struct device *cls, struct device_attribute *attr,
			      char *buf)
{
	gpio_t *gpio = dev_get_drvdata(cls);
	return sprintf(buf, "%x\r\n", atomic_gpio_data_read_lock(gpio));
}

static ssize_t gpio_ddr_show(struct device *cls, struct device_attribute *attr,
			     char *buf)
{
	gpio_t *gpio = dev_get_drvdata(cls);
	return sprintf(buf, "%x\r\n", atomic_gpio_ddr_read_lock(gpio));
}

static ssize_t gpio_index_show(struct device *cls,
			       struct device_attribute *attr, char *buf)
{
	gpio_t *gpio = dev_get_drvdata(cls);
	return sprintf(buf, "%x\r\n", atomic_gpio_index_read_lock(gpio));
}

static DEVICE_ATTR(ddr, S_IRUGO | S_IWUGO, gpio_ddr_show, gpio_ddr_store);
static DEVICE_ATTR(data, S_IRUGO | S_IWUGO, gpio_data_show, gpio_data_store);
static DEVICE_ATTR(index, S_IRUGO | S_IWUGO, gpio_index_show, gpio_index_store);
#endif //CONFIG_GPIOCLASS_SYSFS

/***************************************************************************
 * class instantiation
 */
struct device *gpio_register_device(gpio_t *gpio)
{
	struct class *gpio_master = gpio_declare();
	struct device *dev;
	dev_t devnum = MKDEV(0, 0);

	mutex_init(&gpio->lock);
#ifdef CONFIG_GPIOCLASS_CHAR
	devnum = gpio_char_create(gpio);
#endif

	dev = device_create(gpio_master, NULL, devnum, NULL, gpio->name);
	dev->driver_data = gpio;

#ifdef CONFIG_GPIOCLASS_SYSFS
	if ((gpio->ddr_write) && (gpio->ddr_read))
		if (device_create_file(dev, &dev_attr_ddr))
			printk(KERN_ERR "Error creating sysfs ddr interface for %s\n",
					gpio->name);
	if ((gpio->data_write) && (gpio->data_read))
		if (device_create_file(dev, &dev_attr_data))
			printk(KERN_ERR "Error creating sysfs data interface for %s\n",
					gpio->name);
	if ((gpio->index_write) && (gpio->index_read))
		if (device_create_file(dev, &dev_attr_index))
			printk(KERN_ERR "Error creating sysfs index interface for %s\n",
					gpio->name);
#endif

#ifdef CONFIG_GPIOCLASS_RTDM
	rt_gpio_device_create(gpio);
#endif

	return dev;
}

