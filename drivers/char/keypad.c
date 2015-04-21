/**
 *	keypad.c -- simple keypad driver.
 *
 *	(C) Copyright 2001, Greg Ungerer (gerg@snapgear.com)
 *	(C) Copyright 2001, Lineo Inc. (www.lineo.com)
 *	(C) Copyright 2009, EMAC Inc. <support@emacinc.com>
 * 
 * Modified 2005 NZG EMAC.Inc to load in the 2.6 kernel and support the
 * SoM-100ES. Also modified to support loading the keypad matrix via IOCTL
 * Low level calls are broken into inlines in keypad.h
 *
 * Modified 2009 EMAC Inc to rewrite as a platform driver for use with EMAC
 * PLD Keypad Controllers. Now has the capability to register multiple
 * devices.
 */

//#define DEBUG 1

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/keypad.h>

#include <asm/uaccess.h>
#include <asm/param.h>
#include <asm/types.h>

#include <asm/mach/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>

#include "keypad.h"

static int keypad_major = 0;

static struct keypad_dev *keypads[KEYMINORNUM];
static struct class *keypad_class;

static DEFINE_MUTEX(keypads_lock);

/**
 * function for file ops to open the specified keypad device
 */
static int keypad_open(struct inode *inode, struct file *filp)
{
	struct keypad_dev *k = inode_to_keypad(inode);
	dev_dbg(k->dev, "%s\n", __func__);

	k->blocking = (filp->f_flags & O_NONBLOCK) ? 0 : 1;
	k->isopen++;
	filp->private_data = k;

	return 0;
}

/**
 * function for file ops to close/release the specified keypad device
 */
static int keypad_release(struct inode *inode, struct file *filp)
{
	struct keypad_dev *k = inode_to_keypad(inode);

	dev_dbg(k->dev, "%s\n", __func__);
	k->isopen--;

	return 0;
}

/**
 * function for file ops to read the specified keypad device
 */
static ssize_t keypad_read(struct file *file, char __user * buf, size_t count,
			   loff_t * loff)
{
	int i;
	struct keypad_dev *k = file->private_data;

	if (count == 0)
		return 0;

	dev_dbg(k->dev, "keypad_read(buf = 0x%X, count = %d)\n", 
			(unsigned int)buf, count);

	if (unlikely(k->data->polling)) {
		del_timer(&k->poll_timer);
		put_user(k->buf[k->bufhead], buf);

		if (unlikely(++(k->bufhead) >= KEYPAD_BUFSIZE))
			k->bufhead = 0;
		k->bufcount--;
		if (unlikely(k->bufcount < 0))
			k->bufcount = 0;

		k->poll_timer.expires = jiffies + TIMEOUT;
		add_timer(&k->poll_timer);
		return 1;
	}

	if (k->blocking)
		wait_event_interruptible(k->wait_queue, k->bufcount);

	/* 
	 * this was originally a spinlock but put_user is not necessarily
	 * atomic so a mutex must be used instead
	 */
	if (mutex_lock_interruptible(&k->lock)) {
		return 0;
	}
	if (unlikely(count > k->bufcount))
		count = k->bufcount;
	for (i = 0; i < count; i++) {
		put_user(k->buf[k->bufhead], buf);
		if (unlikely(++(k->bufhead) >= KEYPAD_BUFSIZE))
			k->bufhead = 0;
	}
	k->bufcount -= i;
	mutex_unlock(&k->lock);
	return i;
}


/**
 * function to implement a poll for the specified keypad device
 */
static unsigned int keypad_poll(struct file *file,
				struct poll_table_struct *wait)
{
	int mask = 0;
	struct keypad_dev *k = file->private_data;

	poll_wait(file, &k->wait_queue, wait);

	if (&k->bufcount > 0)
		mask = POLLIN | POLLRDNORM;

	return mask;
}

/**
 * function to add a key to the queue of data on the specified keypad device
 */
static void enqueue_key(struct keypad_dev *k, u8 key)
{
	int pos;
	pos = (k->bufhead + k->bufcount);
	while (pos >= KEYPAD_BUFSIZE)
		pos -= KEYPAD_BUFSIZE;

	k->buf[pos] = key;
	k->bufcount++;

	if (unlikely(k->bufcount >= KEYPAD_BUFSIZE))
		k->bufcount--;
}

/**
 * function to be called when the keypad timer is fired
 */
static void keypad_timer(unsigned long data)
{
	struct keypad_dev *k = (struct keypad_dev *)data;

	del_timer(&k->poll_timer);

	dev_dbg(k->dev, "keypad_timer(countdown = %d) keyheld = %x\n",
		k->countdown, k->data->get_keyheld(k->data));

	if (k->countdown) {
		if (k->data->get_keyheld(k->data)) {
			k->countdown--;
			restart_timer(k);
		} else {
			k->countdown = 0;
			timer_exit(k);
			return;
		}
	} else {
		unsigned short pos = getkeypos(k);
		char key = k->matrix[ROW(pos) * KEYPAD_COLUMNS + COLUMN(pos)];

		dev_dbg(k->dev, "keypad_timer(row = %u, column = %u, key = %c)"
			"keyheld = %x\n", ROW(pos), COLUMN(pos), key,
			k->data->get_keyheld(k->data));

		enqueue_key(k, key);

		wake_up_interruptible(&k->wait_queue);
		if (k->data->get_keyheld(k->data)) {
			k->countdown = REPEAT_DELAY;
			restart_timer(k);
		} else {
			timer_exit(k);
			return;
		}
	}
}

/**
 * function to implement the ioctl interface to the keypad device. supports
 * setting the key matrix and retrieving the current matrix.
 */
int keypad_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		 unsigned long arg)
{
	int rc = 0;
	struct keypad_dev *k = inode_to_keypad(inode);

	dev_dbg(k->dev, "%s\n", __func__);

	switch (cmd) {
	case SETKEYARRAY:
		if (access_ok(VERIFY_READ, (char *)arg, sizeof(k->matrix)))
			rc = copy_from_user(k->matrix, (char *)arg,
					    sizeof(k->matrix));
		else
			return -EINVAL;
		break;
	case GETKEYARRAY:
		if (access_ok(VERIFY_WRITE, (char *)arg, sizeof(k->matrix)))
			rc = copy_to_user((char *)arg, k->matrix,
					  sizeof(k->matrix));
		else
			return -EINVAL;
		break;
		break;
	default:
		rc = -EINVAL;
		break;
	}
	if (rc > 0)
		rc = -EFAULT;
	return (rc);
}

/**
 * interrupt handler for keypad devices. The low level implementation of the
 * irq masking and status reading is handled by the functions specified in the
 * platform device registration.
 */
irqreturn_t keypad_isr(int irq, void *dev_id)
{
	struct keypad_dev *k = dev_id;

	dev_dbg(k->dev, "%s\n", __func__);

	/**
	 * ISR is a trigger for timer state machine
	 */
	if (likely(k->data->int_status(k->data))) {
		if (k->data->get_keyheld(k->data) && !k->intimer) {
			k->data->mask_int(k->data);
			restart_timer(k);
		}
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}


/**
 * function to handle read requests from the procfile. Reports the number of
 * rows and columns and prints out the currently loaded key matrix.
 */
int readprocfile(char *buffer, char **start, off_t offset, int size, int *eof,
		 void *data)
{
	char *temp = buffer;
	int row, column;
	struct keypad_dev *k = (struct keypad_dev *)data;

	/* Print module configuration */
	temp += sprintf(temp, "rows:    %d\n"
			"columns: %d\n\n", KEYPAD_ROWS, KEYPAD_COLUMNS);

	for (row = 0; row < KEYPAD_ROWS; row++) {
		for (column = 0; column < KEYPAD_COLUMNS; column++)
			temp +=
			    sprintf(temp, "%c ",
				    k->matrix[row * KEYPAD_COLUMNS +
						  column]);
		temp += sprintf(temp, "\n");
	}
	return temp - buffer;
}

/**
 * Exported file operations structure for driver...
 */
struct file_operations keypad_fops = {
	.owner = THIS_MODULE,
	.read = keypad_read,
	.poll = keypad_poll,
	.ioctl = keypad_ioctl,
	.open = keypad_open,
	.release = keypad_release,
};

/**
 * probe function for a new keypad device registered through the platform
 * driver interface. Allocates memory and creates user interfaces.
 */
static int __devinit keypad_probe(struct platform_device *pdev)
{
	int result;
	dev_t dev = 0;
	int i;
	struct keypad_dev *k;
	dev_t devno;
	int err;
	char device_name[10];

	mutex_lock(&keypads_lock);

	if (!keypad_major) {
		result = alloc_chrdev_region(&dev, KEYMINORNUM, 1, DRIVER_NAME);
		if (unlikely(result < 0)) {
			dev_err(&pdev->dev,
				"Unable to register driver region!");
			mutex_unlock(&keypads_lock);
			return result;
		}
		keypad_major = MAJOR(dev);
		keypad_class = class_create(THIS_MODULE, DRIVER_NAME);
	}

	/* find the next available keypad device */
	for (i = 0; i < KEYMINORNUM; i++) {
		if (!keypads[i])
			break;
	}
	if (i == KEYMINORNUM) {
		dev_warn(&pdev->dev, "too many keypad devices\n");
		mutex_unlock(&keypads_lock);
		return -ENOMEM;
	}

	keypads[i] = kzalloc(sizeof(struct keypad_dev), GFP_KERNEL);
	if (unlikely(!keypads[i])) {
		dev_warn(&pdev->dev,
			 "kmalloc for keypads structure %d failed!\n", i);
		mutex_unlock(&keypads_lock);
		return -ENOMEM;
	}
	k = keypads[i];

	cdev_init(&k->cdev, &keypad_fops);
	k->cdev.owner = THIS_MODULE;

	devno = MKDEV(keypad_major, i);
	err = cdev_add(&k->cdev, devno, 1);
	if (unlikely(err)) {
		dev_warn(&pdev->dev, "cdev_add failed for keypads %d\n", i);
		mutex_unlock(&keypads_lock);
		return -ENOMEM;
	}
	k->isopen = 0;
	k->intimer = 0;
	k->data = pdev->dev.platform_data;
	init_timer(&k->poll_timer);
	k->poll_timer.function = keypad_timer;
	k->poll_timer.data = (unsigned long)k;
	k->dev = &pdev->dev;
	init_waitqueue_head(&k->wait_queue);
	mutex_init(&k->lock);

	dev_set_drvdata(&pdev->dev, k);

	memset(k->matrix, KEYPAD_UNDEFINED_CHAR, sizeof(k->matrix));

	if (unlikely(k->data->polling)) {
		k->poll_timer.expires = jiffies + TIMEOUT;
		add_timer(&k->poll_timer);
	} else {
		int rc;

		if (k->data->init_int)
			k->data->init_int(k->data);

		dev_dbg(k->dev, "Registering Keypad irq %d, flags = 0x%X\n",
				k->data->irq, k->data->irq_flags);
		rc = request_irq(k->data->irq, keypad_isr, k->data->irq_flags,
				 DRIVER_NAME, k);
		if (unlikely(rc)) {
			dev_warn(k->dev,
				 "Keypad irq not registered. Error: %d. Assuming polled mode.\n",
				 rc);
			k->data->polling = 1;
		}
	}
	snprintf(device_name, sizeof(device_name), "%s%d", DRIVER_NAME, i);
	
	device_create(keypad_class, NULL, devno, NULL, device_name);

	if (!create_proc_read_entry(device_name, 0, 0, readprocfile, k)) {
		dev_err(k->dev, "Can't create keypad procfs interface %s\n",
				device_name);
		mutex_unlock(&keypads_lock);
		return -ENOMEM;
	}

	mutex_unlock(&keypads_lock);

	return 0;
}

/**
 * function to remove a keypad device associated with the given
 * platform_device.
 */
static int __devexit keypad_remove(struct platform_device *pdev)
{
	struct keypad_dev *k = dev_get_drvdata(&pdev->dev);
	int i;
	char device_name[10];

	dev_dbg(&pdev->dev, "%s\n", __func__);

	mutex_lock(&keypads_lock);
	/* find out the index for the device to be removed */
	for (i = 0; i < KEYMINORNUM; i++) {
		if (keypads[i] == k)
			break;
	}
	k->data->mask_int(k->data);
	timer_exit(k);

	if (k->data->free_int)
		k->data->free_int(k->data);
	free_irq(k->data->irq, k);
	snprintf(device_name, sizeof(device_name), "%s%d", DRIVER_NAME, i);

	dev_dbg(&pdev->dev, "%s: removing keypad %d\n", __func__, i);
	platform_device_unregister(pdev);
	cdev_del(&k->cdev);
	remove_proc_entry(device_name, NULL);

	kfree(keypads[i]);
	keypads[i] = NULL;

	mutex_unlock(&keypads_lock);
	return 0;
}

static struct platform_driver keypad_driver = {
	.probe = keypad_probe,
	.remove = keypad_remove,
	.driver = {
		.name = "keypad",
		.owner = THIS_MODULE,
	},
};

/**
 * function in initialize the keypad platform driver interface. Does not
 * create any devices.
 */
static int __init keypad_start(void)
{
	printk(KERN_INFO "Keypad platform driver registering\n");
	memset(keypads, 0, sizeof(keypads));
	return platform_driver_register(&keypad_driver);
}

/**
 * function to exit the keypad driver, making sure that all memory is free and
 * devices are unregistered.
 */
static void __exit keypad_exit(void)
{
	dev_t dev = MKDEV(keypad_major, FIRSTKEYMINOR);
	pr_debug("removing keypad driver\n");

	platform_driver_unregister(&keypad_driver);
	dev = MKDEV(keypad_major, 0);
	unregister_chrdev_region(dev, KEYMINORNUM);
}

module_init(keypad_start);
module_exit(keypad_exit);
