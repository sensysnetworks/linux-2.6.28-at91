 /***************************************************************************
                          		boardspec.c    
			Registration of board specific classes & devices. 
			Registers and calls a function pointer from the arch's platform device
			Important in that it allows for a proper module loading order, which may be
			required as some methods call other modules (specifically Xenomai)             
                             -------------------
	author				 : NZG
    begin                : Tue May 15 2007
    copyright          	 : (C) 2007 by EMAC.Inc
    email                : support@emacinc.com
 ***************************************************************************/
#include <linux/kernel.h>
#include <linux/autoconf.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/ioex/ecoreex.h>
#include <linux/platform_device.h>
#include <asm/io.h> 
 
#define DRV_MODULE_NAME 	"boardspec"
#define DRV_MODULE_VERSION 	"1.0"

static int boardspec_probe(struct platform_device *pdev){
	int (*device_init)(void);
	if (pdev == NULL)return -ENODEV;
	device_init = pdev->dev.platform_data;
	return device_init();
}

//driver currently has no removal method.
static struct platform_driver boardspec_driver = {
	.probe		= boardspec_probe,
	.driver		= {
		.name	= "boardspec",
	},
};

static int __init boardspec_init_module(void)
{
	printk(KERN_INFO DRV_MODULE_NAME " version " DRV_MODULE_VERSION " loading\n");
	return platform_driver_register(&boardspec_driver);
}

static void __exit boardspec_cleanup_module(void)
{
	platform_driver_unregister(&boardspec_driver);
}


module_init(boardspec_init_module);
module_exit(boardspec_cleanup_module);
