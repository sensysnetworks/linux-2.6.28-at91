/**
 * A class for simple gpio ports
 * Several types of general purpose devices are available 
 * which all export the same basic functionity 
 * through differenet underlying methods
 * This class can also be used to export simple interfaces
 * to an 8 bit port into user space
 */

#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/class/gpio.h>
#include <linux/class/rtdm/gpio_rtdm.h>
#include <rtdm/rtdm_driver.h>

typedef struct rtgpio_device_s {
	struct rtdm_device rtd;
	gpio_t *gpio;		//pointer to parent gpio structure.
} rtgpio_device_t;

static int rt_gpio_open(struct rtdm_dev_context *context,
			rtdm_user_info_t * user_info, int oflags)
{
	return 0;
}

static int rt_gpio_close(struct rtdm_dev_context *context,
			 rtdm_user_info_t * user_info)
{
	return 0;
}

static int rt_gpio_ioctl(struct rtdm_dev_context *context,
			 rtdm_user_info_t *user_info, unsigned int request, void __user *umem)
{
	rtgpio_device_t *dev =
	    container_of(context->device, rtgpio_device_t, rtd);
	gpio_t *gpio = dev->gpio;

	gpio_data kmem[1];

	switch (request) {
	case DDRREAD:
		if (gpio->ddr_read)
			kmem[0] = atomic_gpio_ddr_read(gpio);
		else
			return -EFAULT;
		return rtdm_safe_copy_to_user(user_info, umem, kmem,
					      sizeof(gpio_data));

	case DDRWRITE:
		rtdm_safe_copy_from_user(user_info, kmem, umem,
					 sizeof(gpio_data));
		if (gpio->ddr_write)
			atomic_gpio_ddr_write(gpio, kmem[0]);
		else
			return -EFAULT;
		return 0;

	case DATAREAD:
		if (gpio->data_read)
			kmem[0] = atomic_gpio_data_read(gpio);
		else
			return -EFAULT;
		return rtdm_safe_copy_to_user(user_info, umem, kmem,
					      sizeof(gpio_data));

	case DATAWRITE:
		rtdm_safe_copy_from_user(user_info, kmem, umem,
					 sizeof(gpio_data));
		if (gpio->data_write)
			atomic_gpio_data_write(gpio, kmem[0]);
		else
			return -EFAULT;
		return 0;
	}
	return -ENOTTY;
}

static const struct rtdm_device __initdata device_tmpl = {
      struct_version:RTDM_DEVICE_STRUCT_VER,

      device_flags:RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
      device_name:"",
      open_rt:rt_gpio_open,
      ops:{
	      close_rt:rt_gpio_close,
	      close_nrt:rt_gpio_close,
	      ioctl_rt:rt_gpio_ioctl,
	 },
      device_class:RTDM_CLASS_GPIO,
      driver_name:"gpio_rtd",
      driver_version:RTDM_DRIVER_VER(1, 0, 0),
      peripheral_name:"gpio",
      provider_name:"EMAC.Inc",
};

int rt_gpio_device_create(struct gpio_s *gpio)
{
	rtgpio_device_t *dev = kmalloc(sizeof(rtgpio_device_t), GFP_KERNEL);
	dev->gpio = gpio;
	memcpy(&dev->rtd, &device_tmpl, sizeof(struct rtdm_device));
	strncpy(dev->rtd.device_name, dev->gpio->name, RTDM_MAX_DEVNAME_LEN);
	dev->rtd.device_sub_class = dev->gpio->subclass;
	dev->rtd.proc_name = dev->gpio->name;
	if (rtdm_dev_register(&dev->rtd)) {
		printk("couldn't register rtgpio device %s\n",
		       dev->rtd.device_name);
		kfree(dev);
		return -1;
	}
	return 0;
}
