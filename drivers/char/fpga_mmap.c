/*******************************************************************
 * File Name:  drivers/char/fpga_mmap.c
 *
 * Decription: Simple mmap driver to access a memory mapped FPGA.
 *
 * Copyright (C) 2010 EMAC.Inc <support@emacinc.com>
 ******************************************************************/

#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/mman.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/init.h>

#include <asm/mach/irq.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#define FPGA_MINOR 1

#define PHYSICAL_ADDR 0x50000000

unsigned int fpga_irq = AT91SAM9260_ID_IRQ1;

int fpga_major;
static struct cdev *fpga_cdev;
spinlock_t fpga_lock;
struct fasync_struct *async_queue = NULL;

/*
 * Maps the FPGA into User Space
 */
static int fpga_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_flags |= VM_SHARED | VM_RESERVED;

	if(remap_pfn_range(vma, vma->vm_start, (PHYSICAL_ADDR >> PAGE_SHIFT), vma->vm_end - vma->vm_start, PAGE_SHARED))
	{
		printk("FPGA MMAP: mmap failed.\n");
		return -ENXIO;
	}
	return 0;
}

/*
 * Interrupt service routine
 *  Triggers a signal to the user-space application to perform an FPGA transaction.
 *  The service routine could also trigger a workqueue to perform the transaction at the kernel level.
 *
 *  Simple transactions can be handled directly in the ISR.
 */
irqreturn_t fpga_isr(int irq, void *dev_id)
{
	if (async_queue) kill_fasync(&async_queue, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

int fpga_open(struct inode *inode, struct file *file) 
{ 
	return 0; 
}

int fpga_close(struct inode *inode, struct file *file) 
{ 
	return 0; 
}

static int fpga_fasync(int fd, struct file *file, int mode)
{
	return fasync_helper(fd, file, mode, &async_queue);
}

static struct file_operations fpga_fops = 
{
	mmap:     fpga_mmap,
	open:     fpga_open,
	release:  fpga_close,
	fasync:   fpga_fasync,
};

/*
 * Driver initialization routine
 *
 * Creates the device and register the interrupt to a service routine.
 *
 */
static int __init fpga_init(void)
{
	dev_t dev = MKDEV(0, 0);
	int result;

	result = alloc_chrdev_region(&dev, FPGA_MINOR, 1, "fpga-mmap");
	fpga_major = MAJOR(dev);

	if(result < 0) {
		printk(KERN_ERR "FPGA MMAP: Device register failed.\n");
		return -EIO;
	}

	printk("FPGA MMAP: Device registered. (%d,%d)\n", fpga_major,FPGA_MINOR);

	fpga_cdev = cdev_alloc();
	fpga_cdev->owner = THIS_MODULE;
	fpga_cdev->ops = &fpga_fops;
	kobject_set_name(&fpga_cdev->kobj, "fpga-mmap");

	if ( cdev_add(fpga_cdev, dev, 1) ) { 
		printk ("FPGA MMAP: Char device add failed\n");
		return -1; 
	} 
	else 
		printk ("FPGA MMAP: Char device added.\n"); 

	device_create(class_create(THIS_MODULE, "fpga"), NULL, dev, NULL, "fpga-mmap");

	result = request_irq(fpga_irq, fpga_isr, IRQF_SHARED | IRQF_TRIGGER_RISING, "fpga-mmap", &dev);

	if (result) {
		printk(KERN_ERR "fpga-mmap: can't get irq %i\n", fpga_irq);
	}

	return 0;
}

static void __exit fpga_exit(void)
{
	dev_t dev = MKDEV(fpga_major,FPGA_MINOR);

	printk("FPGA MMAP: Device unregistered (%d,%d)\n", fpga_major,FPGA_MINOR);

	free_irq(fpga_irq, &dev);

	remove_proc_entry( "fpga", 0 );
	unregister_chrdev_region(dev, 1);
}

module_init(fpga_init);
module_exit(fpga_exit);


