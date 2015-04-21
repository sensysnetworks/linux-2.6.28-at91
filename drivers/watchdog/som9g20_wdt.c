/*
 * drivers/char/watchdog/som9g20_wdt.c
 *
 * Watchdog driver for the SOM-9G20.
 *
 * Copyright (C) 2009 EMAC Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/gpio.h>

#define MODULE_NAME "SoM-9G20-WDT: "

#define DEFAULT_TIMEOUT  1

static int nowayout = WATCHDOG_NOWAYOUT;

static DEFINE_MUTEX(io_lock);
static unsigned long wdt_status;

#define WDT_IN_USE        0
#define WDT_OK_TO_CLOSE   1
#define WDT_DEVICE_INITED 2

static void wdt_service(void)
{
	mutex_lock(&io_lock);
	at91_set_gpio_output(AT91_PIN_PB20,1);
	msleep(1);
	at91_set_gpio_output(AT91_PIN_PB20,0);
	mutex_unlock(&io_lock);
}

static void wdt_enable(void)
{
	mutex_lock(&io_lock);
	at91_set_gpio_output(AT91_PIN_PA27,0);
	mutex_unlock(&io_lock);
}

static void wdt_disable(void)
{
	spin_lock(&io_lock);
	at91_set_gpio_output(AT91_PIN_PA27,1);
	spin_unlock(&io_lock);
}

static int som9g20_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(WDT_IN_USE, &wdt_status))
		return -EBUSY;

	if (nowayout)
		__module_get(THIS_MODULE);

	wdt_enable();

	return nonseekable_open(inode, file);
}

static ssize_t som9g20_wdt_write(struct file *file, const char *data, size_t len,  loff_t *ppos)
{
	if (len) 
	{
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

			for (i = 0; i != len; i++) {
				char c;
				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					set_bit(WDT_OK_TO_CLOSE, &wdt_status);
			}
		}
		wdt_service();
	}
	return len;
}

static int som9g20_wdt_release(struct inode *inode, struct file *file)
{
	if (test_bit(WDT_OK_TO_CLOSE, &wdt_status)) 
	{
		wdt_disable();
		clear_bit(WDT_IN_USE, &wdt_status);
	} 
	else 
	{
		wdt_service();
		printk(KERN_ERR MODULE_NAME
			"Unexpected close, not stopping watchdog!\n");
	}

	clear_bit(WDT_IN_USE, &wdt_status);
	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

	return 0;
}

static struct watchdog_info ident = {
	.options = WDIOF_KEEPALIVEPING,
	.identity = "SoM-9G20 Watchdog",
};

static long som9g20_wdt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -ENOTTY;

	switch (cmd) {
	case WDIOC_SETOPTIONS:
		if (arg == WDIOS_DISABLECARD) set_bit(WDT_OK_TO_CLOSE, &wdt_status);
		else clear_bit(WDT_OK_TO_CLOSE, &wdt_status);
		wdt_service();
		break;

	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info *)arg, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		ret = put_user(0, (int *)arg);
		break;

	case WDIOC_KEEPALIVE:
		wdt_service();
		ret = 0;
		break;

	case WDIOC_GETTIMEOUT:
		ret = put_user(DEFAULT_TIMEOUT, (int *)arg);
		break;
	}
	return ret;
}

static const struct file_operations som9g20_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = som9g20_wdt_write,
	.unlocked_ioctl = som9g20_wdt_ioctl,
	.open = som9g20_wdt_open,
	.release = som9g20_wdt_release,
};

static struct miscdevice som9g20_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &som9g20_wdt_fops,
};

static int som9g20_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk(KERN_ERR MODULE_NAME "SoM-9G20 Watchdog Timer: fixed timeout = %d sec, nowayout = %d\n", DEFAULT_TIMEOUT, nowayout);

	ret = misc_register(&som9g20_wdt_miscdev);
	if (ret < 0) {
		printk(KERN_ERR MODULE_NAME "cannot register misc device\n");
	} else {
		set_bit(WDT_DEVICE_INITED, &wdt_status);
	}

	return ret;
}

static int som9g20_wdt_remove(struct platform_device *pdev)
{
	misc_deregister(&som9g20_wdt_miscdev);
	return 0;
}

static struct platform_driver platform_wdt_driver = {
	.driver = {
		.name = "som9g20_wdt",
		.owner	= THIS_MODULE,
	},
	.probe = som9g20_wdt_probe,
	.remove = som9g20_wdt_remove,
};

static int __init som9g20_wdt_init(void)
{
	return platform_driver_register(&platform_wdt_driver);
}

static void __exit som9g20_wdt_exit(void)
{
	platform_driver_unregister(&platform_wdt_driver);
}

module_init(som9g20_wdt_init);
module_exit(som9g20_wdt_exit);

MODULE_AUTHOR("EMAC Inc.");
MODULE_DESCRIPTION("SoM-9G20 Watchdog Driver");

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
		__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:watchdog");
