/**
 * A class for simple SPI ports
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
#include <linux/class/spi.h>
#include <asm/io.h>

#ifdef CONFIG_SPICLASS_RTDM
#include <rtdm/rtdm_driver.h>
#include <linux/class/rtdm/spi_rtdm.h>
#define ATOMIC(a) RTDM_EXECUTE_ATOMICALLY(a)
#else
#define ATOMIC(a) a
#endif //CONFIG_GPIOCLASS_RTDM

#ifdef CONFIG_SPICLASS_CHAR
#include <linux/class/char/spi_char.h>
#endif

/************************************************************
 * the global device class
 */
static struct class *spiclass = NULL;
static DEFINE_MUTEX(spi_lock);

struct class *spi_declare(void)
{
	if (mutex_lock_interruptible(&spi_lock))
		return spiclass;
	if (!spiclass) {
		printk("registering GPIO class\n");
		spiclass = class_create(THIS_MODULE, "spi");
#ifdef CONFIG_SPICLASS_CHAR
		spi_char_init();
#endif
	}
	mutex_unlock(&spi_lock);
	return spiclass;
}

/***************************************************************************
 * Atomic method wrappers
 */
int atomic_spi_tip(struct spi_s *s, int ofs)
{
	spi_data retval;
	ATOMIC(retval = s->tip(s, ofs);
	    )
	    return retval;
}

int atomic_spi_xmit(struct spi_s *s, u8 * mosi, u8 * miso, int size)
{
	ATOMIC(s->xmit(s, mosi, miso, size);
	    )
	    return 0;
}

int atomic_spi_tip_write(struct spi_s *s, spi_control config)
{
	ATOMIC(s->tip(s, (config > 0));
	    )
	    return 0;
}

spi_control atomic_spi_tip_read(struct spi_s * s)
{
	spi_data retval;
	ATOMIC(retval = s->tip(s, TIPSTATUS);
	    )
	    return (retval > 0);
}

int atomic_spi_conf_write(struct spi_s *s, spi_control config)
{
	ATOMIC(s->confwrite(s, config);
	    )
	    return 0;
}

spi_control atomic_spi_conf_read(struct spi_s * s)
{
	spi_control retval;
	ATOMIC(retval = s->confread(s);
	    )
	    return retval;
}

int atomic_spi_speed_write(struct spi_s *s, spi_control speed)
{
	ATOMIC(s->speedwrite(s, speed);
	    )
	    return 0;
}

spi_control atomic_spi_speed_read(struct spi_s * s)
{
	spi_control retval;
	ATOMIC(retval = s->speedread(s);
	    )
	    return retval;
}

/***************************************************************************
 * gpio sysfs operations
 */
#ifdef CONFIG_SPICLASS_SYSFS
#define SPIXMITDELIM ","

static ssize_t spi_xmit_store(struct device *cls, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	spi_t *s = cls->driver_data;
	size_t size;
	//atomic_spi_tip(s,TIPON);
	if (s->buf) {
		kfree(s->buf);
		s->bsize = 0;
	}			/* this is not safe unless buf is set to NULL on init */
	//worst case allocation, char-delim-char-delim = count/2 * 4 bytes per word
	s->buf = kmalloc(count * 2, GFP_KERNEL);
	s->bsize = sfs_getstream(buf, count, &size, SPIXMITDELIM, s->buf);
	{
		int i;
		printk("%s-data: ", __FUNCTION__);
		for (i = 0; i < s->bsize; i++)
			printk("0x%x ", s->buf[i]);
		printk("\n");
	}
	atomic_spi_xmit(s, s->buf, s->buf, s->bsize);
	//atomic_spi_tip(s,TIPOFF);

	return size;
}

static ssize_t spi_xmit_show(struct device *cls, struct device_attribute *attr,
			     char *buf)
{
	spi_t *s = cls->driver_data;
	int chars = 0;
	int i;

	for (i = 0; i < s->bsize; i++) {
		if (chars >= (PAGE_SIZE - 10)) {
			chars += sprintf(&buf[chars], "...OVF");
			return chars;
		}
		if (i)
			chars += sprintf(&buf[chars], SPIXMITDELIM);
		chars += sprintf(&buf[chars], "%x", s->buf[i]);
	}
	chars += sprintf(&buf[chars], "\n");
	return chars;
}

static ssize_t spi_flags_store(struct device *cls,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	size_t size;
	spi_t *s = cls->driver_data;
	spi_control data = sfs_getint(buf, count, &size);
	atomic_spi_conf_write(s, data);
	return size;
}

static ssize_t spi_flags_show(struct device *cls, struct device_attribute *attr,
			      char *buf)
{
	spi_t *s = cls->driver_data;
	return sprintf(buf, "%x\r\n", atomic_spi_conf_read(s));
}

static ssize_t spi_speed_store(struct device *cls,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	size_t size;
	spi_t *s = cls->driver_data;
	spi_control data = sfs_getint(buf, count, &size);
	atomic_spi_speed_write(s, data);
	return size;
}

static ssize_t spi_speed_show(struct device *cls, struct device_attribute *attr,
			      char *buf)
{
	spi_t *s = cls->driver_data;
	return sprintf(buf, "%d\r\n", atomic_spi_speed_read(s));
}

static ssize_t spi_tip_store(struct device *cls, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	size_t size;
	spi_t *s = cls->driver_data;
	spi_data data = sfs_getint(buf, count, &size);
	atomic_spi_tip_write(s, data);
	return size;
}

static ssize_t spi_tip_show(struct device *cls, struct device_attribute *attr,
			    char *buf)
{
	spi_t *s = cls->driver_data;
	return sprintf(buf, "%x\r\n", (atomic_spi_tip_read(s)));
}

static DEVICE_ATTR(xmit, S_IRUGO | S_IWUGO, spi_xmit_show, spi_xmit_store);
static DEVICE_ATTR(tip, S_IRUGO | S_IWUGO, spi_tip_show, spi_tip_store);
static DEVICE_ATTR(speed, S_IRUGO | S_IWUGO, spi_speed_show, spi_speed_store);
static DEVICE_ATTR(conf, S_IRUGO | S_IWUGO, spi_flags_show, spi_flags_store);
#endif //CONFIG_SPICLASS_SYSFS
/***************************************************************************
 * class instantiation
 */
struct device *spi_register_device(spi_t * s)
{
	struct class *spi_master = spi_declare();
	struct device *dev;
	dev_t devnum = MKDEV(0, 0);

#ifdef CONFIG_SPICLASS_CHAR
	devnum = spi_char_create(s);
#endif

	dev = device_create(spi_master, NULL, devnum, NULL, s->name);
	dev->driver_data = s;

	s->bsize = 0;
	s->buf = NULL;

#ifdef CONFIG_SPICLASS_SYSFS
	if (s->xmit)
		if (device_create_file(dev, &dev_attr_xmit))
			printk(KERN_ERR "Error creating sysfs xmit interface for %s\n",
					s->name);
	if (s->tip)
		if (device_create_file(dev, &dev_attr_tip))
			printk(KERN_ERR "Error creating sysfs tip interface for %s\n",
					s->name);
	if ((s->speedwrite) && (s->speedread))
		if (device_create_file(dev, &dev_attr_speed))
			printk(KERN_ERR "Error creating sysfs speed interface for %s\n",
					s->name);
	if ((s->confwrite) && (s->confread))
		if (device_create_file(dev, &dev_attr_conf))
			printk(KERN_ERR "Error creating sysfs conf interface for %s\n",
					s->name);
#endif

#ifdef CONFIG_SPICLASS_RTDM
	rt_spi_device_create(s);
#endif

	return dev;
}
