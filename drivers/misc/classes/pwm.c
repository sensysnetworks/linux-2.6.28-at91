/**
 * A class for simple pwm ports
 * Several types of devices are available 
 * which all export the same basic functionity 
 * through differnet underlying methods
 */
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/kdev_t.h>
#include <linux/chelper.h>
#include <linux/class/pwm.h>
#include <asm/io.h>

#ifdef CONFIG_PWMCLASS_RTDM
#include <rtdm/rtdm_driver.h>
#include <linux/class/rtdm/pwm_rtdm.h>
#define ATOMIC(a) RTDM_EXECUTE_ATOMICALLY(a)
#else
#define ATOMIC(a) a
#endif //CONFIG_PWMCLASS_RTDM

/************************************************************
 * the global device class
 */
static struct class *pwmclass = NULL;

struct class *pwm_declare(void)
{
	if (!pwmclass) {
		printk("registering PWM class\n");
		pwmclass = class_create(THIS_MODULE, "pwm");
	}
	return pwmclass;
}

/***************************************************************************
 * typical low level methods
 */

/** need to ajust this to use a clock method rather than getsysclock 
static inline int us2reg(__u32 *value, int Mhz){
		switch(get_sclk()){
			case 66000000:{
				const __u32 regmax = (0xffff/66);
				if(*value>regmax)*value=regmax;
				else *value*=66;
				// pwms seem to do funny things when zeros are written to period or width;
				if(*value==0)*value=1;
				return 0;
			}
			default:return SYSCLOCK_UNSUPPORTED;
	}
}

static inline int reg2us(__u32 *value){
		switch(get_sclk()){
			case 66000000:
				*value/=66;
				return 0;		
			default:return SYSCLOCK_UNSUPPORTED;
	}
}
*/

int pwm_widthus_write8(pwm_t * pwm, pwm_data data)
{
	iowrite8(data, pwm->widthus);
	return 0;
}

int pwm_widthus_write16(pwm_t * pwm, pwm_data data)
{
	iowrite16(data, pwm->widthus);
	return 0;
}

int pwm_periodus_write8(pwm_t * pwm, pwm_data data)
{
	iowrite8(data, pwm->periodus);
	return 0;
}

int pwm_periodus_write16(pwm_t * pwm, pwm_data data)
{
	iowrite16(data, pwm->periodus);
	return 0;
}

int pwm_empty_write(pwm_t * pwm, pwm_data data)
{
	return 0;
}

pwm_data pwm_widthus_read8(pwm_t * pwm)
{
	return ioread8(pwm->widthus);
}

pwm_data pwm_widthus_read16(pwm_t * pwm)
{
	return ioread16(pwm->widthus);
}

pwm_data pwm_periodus_read8(pwm_t * pwm)
{
	return ioread8(pwm->periodus);
}

pwm_data pwm_periodus_read16(pwm_t * pwm)
{
	return ioread16(pwm->periodus);
}

pwm_data pwm_widthusshadow_read(pwm_t * pwm)
{
	return pwm->widthus_shadow;
}

pwm_data pwm_ff_read(pwm_t * pwm)
{
	return 0xff;
}

pwm_data pwm_zero_read(pwm_t * pwm)
{
	return 0;
}

/***************************************************************************
 * Atomic method wrappers
 */
int atomic_pwm_widthus_write(pwm_t * pwm, pwm_data data)
{
	ATOMIC(pwm->widthus_write(pwm, data);
	    )
	    return 0;
}

int atomic_pwm_periodus_write(pwm_t * pwm, pwm_data data)
{
	ATOMIC(pwm->periodus_write(pwm, data);
	    )
	    return 0;
}

int atomic_pwm_invert_write(pwm_t * pwm, pwm_data data)
{
	ATOMIC(pwm->invert_write(pwm, data);
	    )
	    return 0;
}

pwm_data atomic_pwm_widthus_read(pwm_t * pwm)
{
	pwm_data retval;
	ATOMIC(retval = pwm->widthus_read(pwm);
	    )
	    return retval;
}

pwm_data atomic_pwm_periodus_read(pwm_t * pwm)
{
	pwm_data retval;
	ATOMIC(retval = pwm->periodus_read(pwm);
	    )
	    return retval;
}

pwm_data atomic_pwm_invert_read(pwm_t * pwm)
{
	pwm_data retval;
	ATOMIC(retval = pwm->invert_read(pwm);
	    )
	    return retval;
}

/************************************************************************************
 * gpio sysfs operations
 */
#ifdef CONFIG_PWMCLASS_SYSFS

static ssize_t pwm_widthus_store(struct device *cls,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	size_t size;
	pwm_t *pwm = cls->driver_data;
	pwm_data data = sfs_getint(buf, count, &size);
	atomic_pwm_widthus_write(pwm, data);
	return size;
}

static ssize_t pwm_periodus_store(struct device *cls,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	size_t size;
	pwm_t *pwm = cls->driver_data;
	pwm_data data = sfs_getint(buf, count, &size);
	atomic_pwm_periodus_write(pwm, data);
	return size;
}

static ssize_t pwm_invert_store(struct device *cls,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	size_t size;
	pwm_t *pwm = cls->driver_data;
	pwm_data data = sfs_getint(buf, count, &size);
	atomic_pwm_invert_write(pwm, data);
	return size;
}

static ssize_t pwm_widthus_show(struct device *cls,
				struct device_attribute *attr, char *buf)
{
	pwm_t *pwm = cls->driver_data;
	return sprintf(buf, "%u\r\n", atomic_pwm_widthus_read(pwm));
}

static ssize_t pwm_periodus_show(struct device *cls,
				 struct device_attribute *attr, char *buf)
{
	pwm_t *pwm = cls->driver_data;
	return sprintf(buf, "%u\r\n", atomic_pwm_periodus_read(pwm));
}

static ssize_t pwm_invert_show(struct device *cls,
			       struct device_attribute *attr, char *buf)
{
	pwm_t *pwm = cls->driver_data;
	return sprintf(buf, "%x\r\n", atomic_pwm_invert_read(pwm));
}

static DEVICE_ATTR(widthus, S_IRUGO | S_IWUGO, pwm_widthus_show,
		   pwm_widthus_store);
static DEVICE_ATTR(periodus, S_IRUGO | S_IWUGO, pwm_periodus_show,
		   pwm_periodus_store);
static DEVICE_ATTR(inversion, S_IRUGO | S_IWUGO, pwm_invert_show,
		   pwm_invert_store);

#endif //CONFIG_PWMCLASS_SYSFS
/***************************************************************************
* class instantiation
*/
struct device *pwm_register_device(pwm_t * pwm)
{
	struct class *pwm_master = pwm_declare();
	struct device *dev =
	    device_create(pwm_master, NULL, MKDEV(0, 0), NULL, pwm->name);
	dev->driver_data = pwm;

#ifdef CONFIG_PWMCLASS_SYSFS
	if ((pwm->widthus_write) && (pwm->widthus_read))
		if (device_create_file(dev, &dev_attr_widthus))
			printk(KERN_ERR "Error creating sysfs widthus for %s\n",
					pwm->name);
	if ((pwm->periodus_write) && (pwm->periodus_read))
		if (device_create_file(dev, &dev_attr_periodus))
			printk(KERN_ERR "Error creating sysfs periodus for %s\n",
					pwm->name);
	if ((pwm->invert_write) && (pwm->invert_read))
		if (device_create_file(dev, &dev_attr_inversion))
			printk(KERN_ERR "Error creating sysfs invert for %s\n",
					pwm->name);
#endif

#ifdef CONFIG_PWMCLASS_RTDM
	rt_pwm_device_create(pwm);
#endif

	return dev;
}
