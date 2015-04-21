/*
 * drivers/net/phy/micrel.c
 *
 * Driver for Micrel PHYs
 *
 * Author: Michael Welling
 *
 * Copyright (c) 2009 EMAC, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <mach/gpio.h>

/* KSZ8041 Interrupt Register */
#define MII_KSZ8041_INTR		0x1B
#define MII_KSZ8041_INTR_JABBER		0x0080	
#define MII_KSZ8041_INTR_RX_ERR		0x0040
#define MII_KSZ8041_INTR_PAGE_RECV	0x0020
#define MII_KSZ8041_INTR_PARA_FAULT	0x0010
#define MII_KSZ8041_INTR_LINK_ACK	0x0008
#define MII_KSZ8041_INTR_LINK_DOWN	0x0004
#define MII_KSZ8041_INTR_RMT_FAULT	0x0002
#define MII_KSZ8041_INTR_LINK_UP	0x0001

#define MII_KSZ8041_INTR_JABBER_M	0x8000	
#define MII_KSZ8041_INTR_RX_ERR_M	0x4000
#define MII_KSZ8041_INTR_PAGE_RECV_M	0x2000
#define MII_KSZ8041_INTR_PARA_FAULT_M	0x1000
#define MII_KSZ8041_INTR_LINK_ACK_M	0x0800
#define MII_KSZ8041_INTR_LINK_DOWN_M	0x0400
#define MII_KSZ8041_INTR_RMT_FAULT_M	0x0200
#define MII_KSZ8041_INTR_LINK_UP_M	0x0100

#define MII_KSZ8041_INTR_STOP 		(MII_KSZ8041_INTR_LINK_UP_M | MII_KSZ8041_INTR_LINK_DOWN_M)

MODULE_DESCRIPTION("Micrel PHY driver");
MODULE_AUTHOR("EMAC Inc.");
MODULE_LICENSE("GPL");

#define DM9161_DELAY 1

static int ksz8041_config_init(struct phy_device *phydev)
{
	int val;
	u32 features;

	/* For now, I'll claim that the generic driver supports
	 * all possible port types */
	features = (SUPPORTED_TP | SUPPORTED_MII
			| SUPPORTED_AUI | SUPPORTED_FIBRE |
			SUPPORTED_BNC);

	/* Do we support autonegotiation? */
	val = phy_read(phydev, MII_BMSR);

	if (val < 0)
		return val;

	if (val & BMSR_ANEGCAPABLE)
		features |= SUPPORTED_Autoneg;

	if (val & BMSR_100FULL)
		features |= SUPPORTED_100baseT_Full;
	if (val & BMSR_100HALF)
		features |= SUPPORTED_100baseT_Half;
	if (val & BMSR_10FULL)
		features |= SUPPORTED_10baseT_Full;
	if (val & BMSR_10HALF)
		features |= SUPPORTED_10baseT_Half;

	if (val & BMSR_ESTATEN) {
		val = phy_read(phydev, MII_ESTATUS);

		if (val < 0)
			return val;

		if (val & ESTATUS_1000_TFULL)
			features |= SUPPORTED_1000baseT_Full;
		if (val & ESTATUS_1000_THALF)
			features |= SUPPORTED_1000baseT_Half;
	}

	phydev->supported = features;
	phydev->advertising = features;

	return 0;
}

static int ksz8041_config_intr(struct phy_device *phydev)
{
	int temp;

	temp = phy_read(phydev, MII_KSZ8041_INTR);

	if (temp < 0)
		return temp;

	if(PHY_INTERRUPT_ENABLED == phydev->interrupts )
		temp |= MII_KSZ8041_INTR_STOP;
	else
		temp &= ~(MII_KSZ8041_INTR_STOP);

	temp = phy_write(phydev, MII_KSZ8041_INTR, temp);

	return temp;
}

int ksz8041_suspend(struct phy_device *phydev)
{
	int temp;

	temp = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, temp | BMCR_PDOWN);
	mdelay(100);
	at91_set_gpio_value(AT91_PIN_PA25,0);

	return 0;
}

int ksz8041_resume(struct phy_device *phydev)
{
	int temp;

	at91_set_gpio_value(AT91_PIN_PA25,1);
	mdelay(100);
	temp = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, temp & ~BMCR_PDOWN);

	return 0;
}

static int ksz8041_ack_interrupt(struct phy_device *phydev)
{
	int err = phy_read(phydev, MII_KSZ8041_INTR);

	return (err < 0) ? err : 0;
}

static struct phy_driver ksz8041_driver = {
	.phy_id 	= 0x00221512,
	.name		= "Micrel KSZ8041NL",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_BASIC_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_init	= ksz8041_config_init, 
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.ack_interrupt	= ksz8041_ack_interrupt,
	.config_intr	= ksz8041_config_intr,
	.suspend	= ksz8041_suspend,
	.resume		= ksz8041_resume,
	.driver		= { .owner = THIS_MODULE,},
};

static int __init micrel_init(void)
{
	int ret;

	ret = phy_driver_register(&ksz8041_driver);
	
	return ret;
}

static void __exit micrel_exit(void)
{
	phy_driver_unregister(&ksz8041_driver);
}

#ifdef MODULE
module_init(micrel_init);
module_exit(micrel_exit);
#else
subsys_initcall(micrel_init);
#endif
