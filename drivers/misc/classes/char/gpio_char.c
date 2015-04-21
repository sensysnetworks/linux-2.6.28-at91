/**
 * A character device interface to gpio devices. Part of the GPIO class.
 * 
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/class/gpio.h>
#include <linux/class/char/gpio_char.h>
#include <asm/uaccess.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

//#define DEBUG_GC

#ifdef DEBUG_GC
#define DPRINTK(string, args...) printk("gpio_char: " string, ##args)
#else
#define DPRINTK(string, args...)
#endif

#define MAX_QUEUE_SIZE 16

static int gpio_major = GPIO_MAJOR;
static int gpio_minor = 0;
static int gpio_num_devs = GPIO_MAX_DEVS;

DEFINE_MUTEX(gchar_lock); /* race on register */

/* workqueue for interrupt handling */
static struct workqueue_struct *gpio_workqueue;
static void gpio_work(struct work_struct *w);

/**
 * struct for queueing data on interrupt
 * @list: list_head struct for implementing FIFO behavior
 * @data: the data latched during this interrupt
 */
struct data_queue {
	struct list_head list;
	gpio_data data;
};

/**
 * struct for implementing the "notify" on interrupt functionality
 * @filp: pointer to the file struct associated with this object
 * @fasync_fd: file-descriptor argument passed to fasync
 * @fasync_mode: mode argument passed to fasync
 * @list: list_head for maintaining a device-wide list of objects for analysis
 * @queue: data queue for return on DATAREADQ ioctls
 * @queue_size: number of elements in the queue
 * @notify: bitmask set by SETNOTIFY ioctl determining when the device should
 *   be notified of interrupt status
 * @lock: lock for maintaining synchronization of file_list and other
 *   variables in the struct
 */
struct gpio_file_data {
	struct file *filp;
	int fasync_fd;
	int fasync_mode;
	struct list_head list;
	struct list_head queue;
	u8 queue_size;
	gpio_data notify;
	spinlock_t lock;
};

/**
 * struct representing a gpio character device
 * @worker: work struct for IRQ processing
 * @gpio: pointer to the associated gpio class device
 * @cdev: the char device associated with the interface
 * @file_list: list of file objects for notification implementation
 */
struct gpio_char_dev {
	struct work_struct worker;
	gpio_t *gpio;
	struct cdev cdev;
	struct list_head file_list;
};

#define inode_to_gcd(i) (container_of(i->i_cdev, struct gpio_char_dev, cdev))
static gpio_data gfd_dequeue(struct gpio_file_data *gfd);
static void gfd_enqueue(struct gpio_file_data *gfd, gpio_data data);


static void queue_del(struct list_head *q)
{
	/* traverse the list and free all members */
	struct list_head *cur;
	struct list_head *next;
	struct data_queue *tmp;

	list_for_each_safe(cur, next, q) {
		tmp = list_entry(cur, struct data_queue, list);
		DPRINTK("queue_del deleting item\n");
		list_del(cur);
		kfree(tmp);
	}
}

static void file_list_del(struct gpio_file_data *gfd)
{
	/* delete one member of the file list */

	DPRINTK("file_list_del deleting item\n");
	queue_del(&gfd->queue);
	list_del(&gfd->list);
	kfree(gfd);
}

static void construct_notify_list(struct gpio_char_dev *dev, struct
		fasync_struct **fa, gpio_data notify)
{
	struct list_head *cur;
	struct gpio_file_data *tmp;
	gpio_data q_data;

	list_for_each(cur, &dev->file_list) {
		tmp = list_entry(cur, struct gpio_file_data, list);
		if ((tmp->filp->f_flags & FASYNC) && (tmp->notify & notify)) {
			/* add to fasync list */
			DPRINTK("construct_notify_list: adding fd %d to list\n",
					tmp->fasync_fd);
			fasync_helper(tmp->fasync_fd, tmp->filp,
					tmp->fasync_mode, fa);
			q_data = dev->gpio->irq_data.get_queue(dev->gpio, notify);
			gfd_enqueue(tmp, q_data);
		}
	}
}

static void delete_notify_list(struct gpio_char_dev *dev, struct fasync_struct
		**fa)
{
	struct list_head *cur;
	struct gpio_file_data *tmp;

	DPRINTK("delete_notify_list\n");
	list_for_each(cur, &dev->file_list) {
		tmp = list_entry(cur, struct gpio_file_data, list);
		fasync_helper(-1, tmp->filp, 0, fa);
	}
}

static gpio_data gfd_dequeue(struct gpio_file_data *gfd)
{
	/* get list head, then run list_del on it, return data */
	struct data_queue *head;
	struct list_head *ptr = gfd->queue.next;
	gpio_data data;

	head = list_entry(ptr, struct data_queue, list);
	data = head->data;

	list_del(ptr);
	kfree(head);
	gfd->queue_size--;

	DPRINTK("gfd_dequeue: dequeue %d\n", data);

	return data;
}

static void gfd_enqueue(struct gpio_file_data *gfd, gpio_data data)
{
	/* add entry at tail */
	struct data_queue *q;

	if (gfd->queue_size >= MAX_QUEUE_SIZE) {
		DPRINTK("gfd_enqueue: queue_size exceeded\n");
		return;
	}
	if (!(q = kmalloc(sizeof(struct data_queue), GFP_KERNEL))) {
		return;
	}
	DPRINTK("gfd_enqueue: enqueue %d\n", data);
	INIT_LIST_HEAD(&q->list);
	q->data = data;

	list_add_tail(&q->list, &gfd->queue);
	gfd->queue_size++;
}

/* function prototypes */
static int gpio_char_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg);
static int gpio_char_open(struct inode *inode, struct file *file);
static int gpio_char_fasync(int fd, struct file *filp, int mode);
static int gpio_char_release(struct inode *inode, struct file *file);
static int gpio_char_setup_cdev(struct gpio_char_dev *dev);

/* interrupt handling */
static irqreturn_t gpio_interrupt(int irq, void *dev_id)
{
	struct gpio_char_dev *dev = (struct gpio_char_dev *)dev_id;

	DPRINTK("gpio_interrupt: %s\n", dev->gpio->name);
	/* schedule bottom-half */
	queue_work(gpio_workqueue, &dev->worker);

	return IRQ_HANDLED;
}

static void gpio_work(struct work_struct *work) {
	struct gpio_char_dev *dev = container_of(work, struct gpio_char_dev,
			worker);

	gpio_data notify;
	struct fasync_struct *fa = NULL;

	DPRINTK("gpio_work: %s\n", dev->gpio->name);
	if (mutex_lock_interruptible(&dev->gpio->lock))
		return;
	notify = dev->gpio->irq_data.handler(dev->gpio);
	construct_notify_list(dev, &fa, notify);
	
	DPRINTK("gpio_work: sending signals on 0x%X\n", notify);
	/* 
	 * if a signal other than SIGIO was set on this device it will be sent
	 * instead of SIGIO by the kill_fasync function
	 */
	if (fa) {
		DPRINTK("gpio_work: calling kill_fasync\n");
		kill_fasync(&fa, SIGIO, POLL_IN);
	}

	delete_notify_list(dev, &fa);

	mutex_unlock(&dev->gpio->lock);
}


/* struct for fops declarations */
static const struct file_operations gpio_char_fops = {
	.ioctl = gpio_char_ioctl,
	.fasync = gpio_char_fasync,
	.open = gpio_char_open,
	.release = gpio_char_release,
};

static int gpio_char_open(struct inode *inode, struct file *file)
{
	struct gpio_file_data *gfd;
	struct gpio_char_dev *dev = inode_to_gcd(inode);
	struct gpio_irq_data *idat = &dev->gpio->irq_data;

	DPRINTK("gpio_char_open\n");

	file->private_data = NULL;
	/* Only initialize the irq/notification structures if necessary */
	if (idat->irq) {
		if (!(gfd = kmalloc(sizeof(struct gpio_file_data), GFP_KERNEL))) {
			return -1;
		}
		gfd->filp = file;
		file->private_data = gfd;

		INIT_LIST_HEAD(&gfd->queue);
		INIT_LIST_HEAD(&gfd->list);
		spin_lock_init(&gfd->lock);
		gfd->notify = 0;
		gfd->queue_size = 0;

		/* add this gpio_file_data to the list */
		if (mutex_lock_interruptible(&dev->gpio->lock))
			return -EFAULT;
		list_add(&gfd->list, &dev->file_list);
		mutex_unlock(&dev->gpio->lock);
	}

	return 0;
}

static int gpio_char_release(struct inode *inode, struct file *file)
{
	struct gpio_file_data *gfd;
	struct gpio_char_dev *dev = inode_to_gcd(inode);
	struct gpio_irq_data *idat = &dev->gpio->irq_data;

	DPRINTK("gpio_char_release\n");

	if (idat->irq) {
		gfd = (struct gpio_file_data *)(file->private_data);

		/* destroy all members of the file data struct for this file */
		/* use device lock, file_list will be changed */
		mutex_lock(&dev->gpio->lock);
		idat->change_notify(dev->gpio, gfd->notify, 0);
		file_list_del(gfd);
		mutex_unlock(&dev->gpio->lock);
	}

	return 0;
}

static int gpio_char_fasync(int fd, struct file *file, int mode)
{
	unsigned long flags;
	struct gpio_file_data *gfd = file->private_data;

	DPRINTK("gpio_char_fasync: fd = %d, mode = %d\n", fd, mode);

	spin_lock_irqsave(&gfd->lock, flags);
	gfd->fasync_fd = fd;
	gfd->fasync_mode = mode;
	spin_unlock_irqrestore(&gfd->lock, flags);
	
	return 0;
}

static int gpio_char_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct gpio_char_dev *dev = inode_to_gcd(inode);

	struct gpio_file_data *gfd;
	gpio_t *gpio = dev->gpio;
	gpio_data kmem[1];
	gpio_data notify;
	unsigned long flags;
	int ret;

	/* make sure that this command is indeed one of gpio_char's */
	if (_IOC_TYPE(cmd) != CHAR_CLASS_GPIO)
		return -ENOTTY;

	gfd = (struct gpio_file_data *)(file->private_data);

	switch (cmd) {
		/* standard functions which take and release the device mutex */
	case DDRREAD:
		DPRINTK("DDRREAD ioctl\n");
		if (gpio->ddr_read)
			kmem[0] = atomic_gpio_ddr_read_lock(gpio);
		else
			return -EFAULT;
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;
		break;
	case DDRWRITE:
		DPRINTK("DDRWRITE ioctl\n");
		if (copy_from_user(kmem, (void *)arg, sizeof(gpio_data)) != 0)
			return -EFAULT;
		if (gpio->ddr_write)
			atomic_gpio_ddr_write_lock(gpio, kmem[0]);
		else
			return -EFAULT;
		return 0;
		break;
	case DATAREAD:
		DPRINTK("DATAREAD ioctl\n");
		if (gpio->data_read)
			kmem[0] = atomic_gpio_data_read_lock(gpio);
		else
			return -EFAULT;
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;
		break;
	case DATAWRITE:
		DPRINTK("DATAWRITE ioctl\n");
		if (copy_from_user(kmem, (void *)arg, sizeof(gpio_data)) != 0)
			return -EFAULT;
		if (gpio->data_write)
			atomic_gpio_data_write_lock(gpio, kmem[0]);
		else {
			DPRINTK("error: invalid data_write\n");
			return -EFAULT;
		}
		return 0;
		break;
	case INDEXREAD:
		DPRINTK("INDEXREAD ioctl\n");
		if (gpio->index_read)
			kmem[0] = atomic_gpio_index_read_lock(gpio);
		else
			return -EFAULT;
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;
		break;
	case INDEXWRITE:
		DPRINTK("INDEXWRITE ioctl\n");
		if ((ret = copy_from_user(kmem, (void __user *)arg,
						sizeof(gpio_data))) != 0) {
			DPRINTK("copy_from_user returned error: %d bytes not copied\n",
				 ret);
			return -EFAULT;
		}
		if (gpio->index_write)
			atomic_gpio_index_write_lock(gpio, kmem[0]);
		else {
			DPRINTK("invalid index_write command\n");
			return -EFAULT;
		}
		return 0;
		break;

		/* Functions to lock / unlock the device */
	case GPIOLOCK:
		DPRINTK("LOCK ioctl\n");
		if (mutex_lock_interruptible(&gpio->lock))
			return -EFAULT; /* lock not held */
		return 0; /* lock held */
		break;
	case GPIOUNLOCK:
		DPRINTK("UNLOCK ioctl\n");
		mutex_unlock(&gpio->lock);
		return 0; /* device unlocked */
		break;

		/* 
		 * Functions to access the device without taking the lock (i.e.
		 * LOCK / UNLOCK must be used as wrappers
		 */
	case DDRREAD_NL:
		DPRINTK("DDRREAD_NL ioctl\n");
		if (gpio->ddr_read)
			kmem[0] = atomic_gpio_ddr_read(gpio);
		else
			return -EFAULT;
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;
		break;
	case DDRWRITE_NL:
		DPRINTK("DDRWRITE_NL ioctl\n");
		if (copy_from_user(kmem, (void *)arg, sizeof(gpio_data)) != 0)
			return -EFAULT;
		if (gpio->ddr_write)
			atomic_gpio_ddr_write(gpio, kmem[0]);
		else
			return -EFAULT;
		return 0;
		break;
	case DATAREAD_NL:
		DPRINTK("DATAREAD_NL ioctl\n");
		if (gpio->data_read)
			kmem[0] = atomic_gpio_data_read(gpio);
		else
			return -EFAULT;
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;
		break;
	case DATAWRITE_NL:
		DPRINTK("DATAWRITE ioctl\n");
		if (copy_from_user(kmem, (void *)arg, sizeof(gpio_data)) != 0)
			return -EFAULT;
		if (gpio->data_write)
			atomic_gpio_data_write(gpio, kmem[0]);
		else {
			DPRINTK("error: invalid data_write\n");
			return -EFAULT;
		}
		return 0;
		break;
	case INDEXREAD_NL:
		DPRINTK("INDEXREAD_NL ioctl\n");
		if (gpio->index_read)
			kmem[0] = atomic_gpio_index_read(gpio);
		else
			return -EFAULT;
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;
		break;
	case INDEXWRITE_NL:
		DPRINTK("INDEXWRITE ioctl\n");
		if ((ret = copy_from_user(kmem, (void __user *)arg,
						sizeof(gpio_data))) != 0) {
			DPRINTK
				("copy_from_user returned error: %d bytes not copied\n",
				 ret);
			return -EFAULT;
		}
		if (gpio->index_write)
			atomic_gpio_index_write(gpio, kmem[0]);
		else {
			DPRINTK("invalid index_write command\n");
			return -EFAULT;
		}
		return 0;
		break;

	/** functions for interrupt notification */
	/* Note that these are open-file specific, not device specific, so the
	 * device mutex is not locked, only the gpio_file_data spinlock */
	case DATAREADQ:
		DPRINTK("DATAREADQ ioctl %d\n", gfd->queue_size);
		kmem[0] = 0;
		if (gfd) {
			spin_lock_irqsave(&gfd->lock, flags);
			if (gfd->queue_size) {
				/* pull the item from the front of the queue */
				kmem[0] = gfd_dequeue(gfd);
			}
			spin_unlock_irqrestore(&gfd->lock, flags);
		}
		else {
			DPRINTK("not implemented\n");
		}
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;

		break;
	case SETNOTIFY:
		DPRINTK("SETNOTIFY ioctl\n");
		/* copy_from_user to see what bits the user is interested in
		 * on this fd */
		if (gfd) {
			if ((ret = copy_from_user(kmem, (void __user *)arg,
						sizeof(gpio_data))) != 0) {
				DPRINTK("copy_from_user returned error: %d bytes not copied\n",
					 ret);
				return -EFAULT;
			}
			spin_lock_irqsave(&gfd->lock, flags);
			notify = gfd->notify;
			gfd->notify = kmem[0];
			spin_unlock_irqrestore(&gfd->lock, flags);
			if (mutex_lock_interruptible(&gpio->lock))
				return -EFAULT;
			gpio->irq_data.change_notify(gpio, notify,
					gfd->notify);
			mutex_unlock(&gpio->lock);
		}
		else {
			DPRINTK("Not implemented\n");
		}
		return 0;
		break;
	case GETNOTIFY:
		DPRINTK("GETNOTIFY ioctl\n");
		/* copy_to_user the bitmask that the user is listening on for
		 * this fd */
		kmem[0] = 0;
		if (gfd) {
			spin_lock_irqsave(&gfd->lock, flags);
			kmem[0] = gfd->notify;
			spin_unlock_irqrestore(&gfd->lock, flags);
		}
		else {
			DPRINTK("Not implemented\n");
		}
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;

		break;
	case GETQUEUESIZE:
		DPRINTK("GETQUEUESIZE ioctl %d\n", gfd->queue_size);
		kmem[0] = 0;
		if (gfd) {
			spin_lock_irqsave(&gfd->lock, flags);
			kmem[0] = gfd->queue_size;
			spin_unlock_irqrestore(&gfd->lock, flags);
		} else {
			DPRINTK("Not implemented\n");
		}
		return (copy_to_user((void *)arg, kmem, sizeof(gpio_data)) ==
				0) ? 0 : -EFAULT;
		break;
	default:
		DPRINTK("ioctl: no such command\n");
		return -ENOTTY;
	}			/* END switch(cmd) */
	DPRINTK("Invalid state, broke out of switch\n");
	return -EFAULT;		/* Error, we should never reach here */
}

/* initialize the character interface -- called by gpio class on init */

int gpio_char_init(void)
{
	int result;
	dev_t dev = 0;
	DPRINTK("gpio_char_init\n");
	/* dynamic and static character device allocation */
	if (mutex_lock_interruptible(&gchar_lock))
		return -1;

	if (gpio_major) {
		dev = MKDEV(gpio_major, gpio_minor);
		result =
			register_chrdev_region(dev, gpio_num_devs, "gpio_char");
	} else {
		result = alloc_chrdev_region(&dev, gpio_minor,
				gpio_num_devs, "gpio_char");
		gpio_major = MAJOR(dev);
	}
	gpio_workqueue = create_workqueue("gpio_workqueue");

	mutex_unlock(&gchar_lock);
	if (result < 0)
		printk(KERN_WARNING "gpio_char: can't get major %d, err %d\n",
				gpio_major, result);

	return result;
}

/* registers the actual char device, called by create when invoked by gpio class */

static int gpio_char_setup_cdev(struct gpio_char_dev *dev)
{
	static int index = 0;
	int err;
	int devno;
	struct gpio_irq_data *idat = &dev->gpio->irq_data;

	if (mutex_lock_interruptible(&gchar_lock))
		return -1;

	devno = MKDEV(gpio_major, gpio_minor + index);

	DPRINTK("gpio_char_setup_cdev\n");

	cdev_init(&dev->cdev, &gpio_char_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &gpio_char_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_ERR "gpio_char: Error %d adding gpio_char%d\n",
				err, index);

	index++;

	if (idat->irq) {
		DPRINTK("requesting IRQ %d for %s\n", idat->irq,
				dev->gpio->name);
		if (request_irq(idat->irq, gpio_interrupt, idat->irq_flags,
				dev->gpio->name, dev) < 0)
			DPRINTK("request_irq failed\n");
		/* configure / enable the irq */
		idat->irq_config(dev->gpio);
	}
	mutex_unlock(&gchar_lock);

	return devno;
}

/* create and register a new char device */

int gpio_char_create(struct gpio_s *gpio)
{
	struct gpio_char_dev *chardev =
		kmalloc(sizeof(struct gpio_char_dev), GFP_KERNEL);
	DPRINTK("gpio_char_create\n");
	if (!chardev) {
		printk(KERN_ERR "Error allocating memory\n");
		return -ENOMEM;
	}
	chardev->gpio = gpio;
	INIT_LIST_HEAD(&chardev->file_list);
	INIT_WORK(&chardev->worker, gpio_work);

	return gpio_char_setup_cdev(chardev);
}

