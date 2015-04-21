/***************************************************************************
  ecoreex.c    
  EMAC soft cores device registration             
  -------------------
author				 : NZG
rewrite              : Tue May 15 2007
copyright          	 : (C) 2007 by EMAC.Inc
email                : support@emacinc.com
 ***************************************************************************/

#include <linux/kernel.h>
#include <linux/autoconf.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/class/gpio.h>
#include <linux/class/pwm.h>
#include <linux/ioex/ecoreex.h>
#include <linux/ioex/pwmd.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>

/*********************************************************************/
#ifdef CONFIG_ECOREEX_RDAC

#include <mach/gpio-rdac.h>
#include <mach/gpio.h>
#include <mach/rdac-interrupts.h>
#include <mach/rdac-uart.h>

#define CPLD_RDAC_NAME_R0 "RDAC GPI/O expansion R0.0"
#define CPLD_RDAC_NAME_R1 "RDAC GPI/O expansion R1.0"
#define CPLD_RDAC_NAME_R2 "RDAC GPI/O expansion R2.0"
#define CPLD_RDAC_NAME_R3 "RDAC GPI/O expansion R3.0"
#define CPLD_RDAC_NAME_R4 "RDAC GPI/O expansion R4.0"
#define CPLD_RDAC_R0 0xC1
#define CPLD_RDAC_R1 0xE0
#define CPLD_RDAC_R2 0xE1
#define CPLD_RDAC_R3 0xE2
#define CPLD_RDAC_R4 0xE3

static struct rdac_ic ic;
static spinlock_t dtr_lock = SPIN_LOCK_UNLOCKED;

#define RDAC_RF_ISM_ASSOC	(1 << 6)
#define RDAC_RF_ISM_ON		(1 << 5)
#define RDAC_RF_TT_JAMDET	(1 << 3)
#define RDAC_RF_TT_RF_TXMON	(1 << 2)
#define RDAC_RF_TT_ALARM	(1 << 1)
#define RDAC_RF_TT_POWRMON	(1 << 0)

#define RDAC_RF_NUM_BITS	(6)
#define RDAC_RF_ALL (RDAC_RF_ISM_ASSOC | \
		RDAC_RF_ISM_ON | \
		RDAC_RF_TT_JAMDET | \
		RDAC_RF_TT_RF_TXMON | \
		RDAC_RF_TT_ALARM | \
		RDAC_RF_TT_POWRMON)

#define RDAC_XBEE_NUM_BITS	(2)
/* shift amount to translate between XBEE status and Interrupt registers */
#define RDAC_XBEE_SHIFT		(4)
#define RDAC_XBEE_ALL (RDAC_RF_ISM_ASSOC | \
		RDAC_RF_ISM_ON)

#define RDAC_CELL_NUM_BITS	(4)
#define RDAC_CELL_ALL (RDAC_RF_TT_JAMDET | \
		RDAC_RF_TT_RF_TXMON | \
		RDAC_RF_TT_ALARM | \
		RDAC_RF_TT_POWRMON)

static u8 rdac_irq_usage[RDAC_RF_NUM_BITS+1]; /* [4] is unused */
static u8 rdac_cell_bitpos[RDAC_CELL_NUM_BITS] = { 0, 1, 2, 3 };
static u8 rdac_xbee_bitpos[RDAC_XBEE_NUM_BITS] = { 5, 6 };

/* config -- currently just all positive edge */
#define RDAC_RF_CONFIG   (0)
#define RDAC_XBEE_CONFIG (0)
#define RDAC_CELL_CONFIG (0)

/**
 * function to configure and enable the interrupts on the RDAC PLD interrupt
 * controller
 */
static void rdac_irq_config(struct gpio_s *gpio)
{
	unsigned long flags;
	u8 tmp;

	spin_lock_irqsave(&ic.lock, flags);

	/* write the configuration */
	tmp = ioread8(ic.icr);
	iowrite8((tmp | RDAC_RF_CONFIG), ic.icr);
	/* clear the status register */
	iowrite8(RDAC_RF_ALL, ic.isr);
	/* enable the interrupt */
	tmp = ioread8(ic.ier) | RDAC_RF_ALL;
	iowrite8(tmp, ic.ier);
	
	ic.num_registered++;

	spin_unlock_irqrestore(&ic.lock, flags);

	set_irq_wake(gpio->irq_data.irq, 1);
}

/* necessary for rev3 PLD design */
static void rdac_xbee_irq_config(struct gpio_s *gpio)
{
	unsigned long flags;
	u8 tmp;

	spin_lock_irqsave(&ic.lock, flags);

	/* write the configuration */
	tmp = ioread8(ic.icr);
	iowrite8((tmp | RDAC_XBEE_CONFIG), ic.icr);
	/* clear the status register */
	iowrite8(RDAC_XBEE_ALL, ic.isr);
	/* disable the interrupts */
	tmp = ioread8(ic.ier) & (~RDAC_XBEE_ALL);
	iowrite8(tmp, ic.ier);
	
	ic.num_registered++;

	spin_unlock_irqrestore(&ic.lock, flags);

	set_irq_wake(gpio->irq_data.irq, 1);
}

static void rdac_cell_irq_config(struct gpio_s *gpio)
{
	unsigned long flags;
	u8 tmp;

	spin_lock_irqsave(&ic.lock, flags);

	/* write the configuration */
	tmp = ioread8(ic.icr);
	iowrite8((tmp | RDAC_CELL_CONFIG), ic.icr);
	/* clear the status register */
	iowrite8(RDAC_CELL_ALL, ic.isr);
	/* disable the interrupts */
	tmp = ioread8(ic.ier) & (~RDAC_CELL_ALL);
	iowrite8(tmp, ic.ier);
	
	ic.num_registered++;

	spin_unlock_irqrestore(&ic.lock, flags);

	set_irq_wake(gpio->irq_data.irq, 1);
}

static void rdac_update_cell_ier(void)
{
	int i;
	u8 new_ier = 0;
	u8 ier;
	unsigned long flags;

	for (i = 0; i < RDAC_CELL_NUM_BITS; i++) {
		new_ier |= ((rdac_irq_usage[rdac_cell_bitpos[i]] ? 1 : 0) <<
				rdac_cell_bitpos[i]);
	}
	spin_lock_irqsave(&ic.lock, flags);
	/* mask so that only cell bits are affected */
	ier = ioread8(ic.ier) & ~RDAC_CELL_ALL;
	ier |= new_ier;
	iowrite8(ier, ic.ier);
	spin_unlock_irqrestore(&ic.lock, flags);
}

static void rdac_update_xbee_ier(void)
{
	int i;
	u8 new_ier = 0;
	u8 ier;
	unsigned long flags;

	for (i = 0; i < RDAC_XBEE_NUM_BITS; i++) {
		new_ier |= ((rdac_irq_usage[rdac_xbee_bitpos[i]] ? 1 : 0) <<
				rdac_xbee_bitpos[i]);
	}
	spin_lock_irqsave(&ic.lock, flags);
	/* mask so that only xbee bits are affected */
	ier = ioread8(ic.ier) & ~RDAC_XBEE_ALL;
	ier |= new_ier;
	iowrite8(ier, ic.ier);
	spin_unlock_irqrestore(&ic.lock, flags);
}

static void rdac_cell_change_notify(struct gpio_s *gpio, gpio_data curr_notify,
		gpio_data new_notify)
{
	int i;
	int ier_change = 0;
	gpio_data mask;

	for (i = 0; i < RDAC_CELL_NUM_BITS; i++) {
		mask = 1 << (rdac_cell_bitpos[i]);
		if ((mask & curr_notify) && !(mask & new_notify)) {
			/* bit disabled, dec usage */
			if ((--rdac_irq_usage[rdac_cell_bitpos[i]]) == 0) {
				ier_change = 1;
			}
		} else if ((mask & new_notify) && !(mask & curr_notify)) {
			/* bit enabled, inc usage */
			if ((++rdac_irq_usage[rdac_cell_bitpos[i]]) == 1) {
				ier_change = 1;
			}
		}
	}
	if (ier_change)
		rdac_update_cell_ier();
}

static void rdac_xbee_change_notify(struct gpio_s *gpio, gpio_data curr_notify,
		gpio_data new_notify)
{
	int i;
	int ier_change = 0;
	int mask;

	/* shift to match interrupt bit locations */
	curr_notify <<= RDAC_XBEE_SHIFT;
	new_notify <<= RDAC_XBEE_SHIFT;

	for (i = 0; i < RDAC_XBEE_NUM_BITS; i++) {
		mask = 1 << rdac_xbee_bitpos[i];
		if ((mask & curr_notify) && !(mask & new_notify)) {
			/* bit disabled, dec usage */
			if ((--rdac_irq_usage[rdac_xbee_bitpos[i]]) == 0) {
				ier_change = 1;
			}
		} else if ((mask & new_notify) && !(mask & curr_notify)) {
			/* bit enabled, inc usage */
			if ((++rdac_irq_usage[rdac_xbee_bitpos[i]]) == 1) {
				ier_change = 1;
			}
		}
	}
	if (ier_change)
		rdac_update_xbee_ier();
}

/**
 * function to handle interrupts generated by the RDAC PLD interrupt
 * controller. This function is called in a workqueue by the GPIO class
 * character driver interface.
 */
static gpio_data rdac_handler(struct gpio_s *gpio)
{
	unsigned long flags;
	u8 isr;
	u8 run_check = 0;

	spin_lock_irqsave(&ic.lock, flags);

	/* determine notify mask and clear pertinent bits in the isr */
	isr = ioread8(ic.isr);
	iowrite8(isr & RDAC_RF_ALL, ic.isr);

	ic.num_handled++;
	if (ic.num_handled == ic.num_registered)
		run_check = 1;
	spin_unlock_irqrestore(&ic.lock, flags);

	if (run_check) 
		ic.check_handled(&ic);

	return isr & RDAC_RF_ALL;
}

/* necessary for rev3 PLD images */
static gpio_data rdac_xbee_handler(struct gpio_s *gpio)
{
	unsigned long flags;
	u8 isr;
	u8 run_check = 0;

	spin_lock_irqsave(&ic.lock, flags);

	/* determine notify mask and clear pertinent bits in the isr */
	isr = ioread8(ic.isr);
	iowrite8(isr & RDAC_XBEE_ALL, ic.isr);
	ic.num_handled++;
	if (ic.num_handled == ic.num_registered)
		run_check = 1;
	spin_unlock_irqrestore(&ic.lock, flags);

	if (run_check)
		ic.check_handled(&ic);

	/* 
	 * ISM_ASSOC and ISM_ON are the relevant bits in the ISR. These need
	 * to be shifted to line up with their bit position in the Xbee status
	 * register.
	 */

	return ((isr & RDAC_XBEE_ALL) >> RDAC_XBEE_SHIFT);
}

static gpio_data rdac_cell_handler(struct gpio_s *gpio)
{
	unsigned long flags;
	u8 isr;
	u8 run_check = 0;

	spin_lock_irqsave(&ic.lock, flags);

	/* determine notify mask and clear pertinent bits in the isr */
	isr = ioread8(ic.isr);
	iowrite8(isr & RDAC_CELL_ALL, ic.isr);

	ic.num_handled++;
	if (ic.num_handled == ic.num_registered)
		run_check = 1;
	spin_unlock_irqrestore(&ic.lock, flags);

	if (run_check)
		ic.check_handled(&ic);

	/*
	 * TT_* ISR bits are elegible for notification. These bits are aligned
	 * with their position in the cell status register.
	 */

	return (isr & RDAC_CELL_ALL);
}

/**
 * function to get the data to add to the device-specific queue. Called from
 * the GPIO class character device interface. Note that this is called with
 * the gpio->lock mutex held.
 */
static gpio_data rdac_get_queue(struct gpio_s *gpio, gpio_data notify)
{
	return atomic_gpio_data_read(gpio);
}

static irqreturn_t rdac_interrupt(int irq, void *na)
{
	printk(KERN_DEBUG "RDAC Interrupt detected!!\n");
	return IRQ_HANDLED;
}

/**
 * function to check if all pending interrupts have been handled. This is
 * required because the RDAC uses a shared level-triggered interrupt, so
 * interrupts that occur before the previous interrupt has been ACK'd may be
 * missed. Note that due to the interrupt handler bottom half for the RDAC IRQ
 * being called from a workqueue, it is possible that this function will
 * detect a missed interrupt when one did not really occur. This has no ill
 * side effects besides a possible extra-pass through the interrupt handler
 * loop in which no new interrupts would be detected. The possibilities of
 * this happening are very slim.
 */
static void rdac_check_handled(struct rdac_ic *ic)
{
	unsigned long flags;
	u8 ier;
	u8 isr;
	
	spin_lock_irqsave(&ic->lock, flags);
	ic->num_handled = 0;
	ier = ioread8(ic->ier);
	isr = ioread8(ic->isr);

	if (isr & ier) {
		/* there may be an unhandled interrupt, trigger a pulse on the
		 * interrupt line without touching the ISR
		 */
		iowrite8(0, ic->ier);
		iowrite8(ier, ic->ier);
	}
	spin_unlock_irqrestore(&ic->lock, flags);
}

#define RDAC_DTR_BIT	(1 << 0)

/* DTR-masking write functions for xbee_ctrl and modem_ctrl registers */
static int dtr_data_write(struct gpio_s *gpio, gpio_data data)
{
	u8 curr;

	/* dtr lock should never be accessed from interrupt context so irqsave
	 * is not required */
	spin_lock(&dtr_lock);
	curr = ioread8(gpio->data);
	/* make sure that DTR is not changed regardless of passed data */
	if (curr & RDAC_DTR_BIT)
		data |= RDAC_DTR_BIT;
	else /* DTR is 0 */
		data &= ~RDAC_DTR_BIT;
	iowrite8((u8)data, gpio->data);
	spin_unlock(&dtr_lock);
	return 0;
}


static int CPLD_RDAC_R0_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name,
				struct ecoreex_data *data)
{
	gpio_t *gpio_spi_cs;
	gpio_t *gpio_ism_dio;
	gpio_t *gpio_rf_ctrl;
	gpio_t *gpio_pwm;
	gpio_t *gpio_tty_in;
	gpio_t *gpio_tty_out;
	gpio_t *gpio_power_ctrl;
	gpio_t *gpio_intr_en;
	gpio_t *gpio_intr_stat;
	gpio_t *gpio_rf_stat;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	gpio_spi_cs =
	    gpio_device_create_unregistered(&virt_addr[2], NULL, "spi_cs");
	gpio_spi_cs->index_write = NULL;
	gpio_spi_cs->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_spi_cs);

	gpio_ism_dio =
	    gpio_device_create_unregistered(&virt_addr[3], &virt_addr[4],
					    "ism_dio");
	gpio_ism_dio->index_write = NULL;
	gpio_ism_dio->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_ism_dio);

	gpio_rf_ctrl =
	    gpio_device_create_unregistered(&virt_addr[5], NULL, "rf_ctrl");
	gpio_rf_ctrl->index_write = NULL;
	gpio_rf_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_rf_ctrl);

	gpio_pwm =
	    gpio_device_create_unregistered(&virt_addr[6], NULL, "gpio_pwm");
	gpio_pwm->index_write = NULL;
	gpio_pwm->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_pwm);

	gpio_tty_in =
	    gpio_device_create_unregistered(&virt_addr[8], NULL, "mdm_out");
	gpio_tty_in->index_write = NULL;
	gpio_tty_in->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_tty_in);

	gpio_tty_out =
	    gpio_device_create_unregistered(&virt_addr[9], NULL, "mdm_in");
	gpio_tty_out->index_write = NULL;
	gpio_tty_out->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_tty_out);

	gpio_power_ctrl =
	    gpio_device_create_unregistered(&virt_addr[10], NULL, "power_ctrl");
	gpio_power_ctrl->index_write = NULL;
	gpio_power_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_power_ctrl);

	gpio_intr_en =
	    gpio_device_create_unregistered(&virt_addr[11], NULL, "intr_en");
	gpio_intr_en->index_write = NULL;
	gpio_intr_en->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_intr_en);

	gpio_intr_stat =
	    gpio_device_create_unregistered(&virt_addr[12], NULL, "intr_stat");
	gpio_intr_stat->index_write = NULL;
	gpio_intr_stat->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_intr_stat);

	gpio_rf_stat =
	    gpio_device_create_unregistered(&virt_addr[13], NULL, "rf_stat");
	gpio_rf_stat->index_write = NULL;
	gpio_rf_stat->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_rf_stat);

	gpio_register_device(gpio_spi_cs);
	gpio_register_device(gpio_ism_dio);
	gpio_register_device(gpio_rf_ctrl);
	gpio_register_device(gpio_pwm);
	gpio_register_device(gpio_tty_in);
	gpio_register_device(gpio_tty_out);
	gpio_register_device(gpio_power_ctrl);
	gpio_register_device(gpio_intr_en);
	gpio_register_device(gpio_intr_stat);
	gpio_register_device(gpio_rf_stat);

	/*
	 * Allocate the IRQ
	 */
	if (data->irq) {
		int err;

		err =
		    request_irq(data->irq, rdac_interrupt,
				IRQF_SHARED, name, data);
		if (err < 0)
			printk("ecoreex request for irq %d failed with %d\n",
			       data->irq, err);
		else
			set_irq_wake(data->irq, 1);	//if the board provides an irq, register it to wake up pm.
	}

	return 0;
}

static int CPLD_RDAC_R1_2_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name,
				struct ecoreex_data *data)
{
	gpio_t *gpio_ism_dio;
	gpio_t *gpio_rf_ctrl;
	gpio_t *gpio_pwm;
	gpio_t *gpio_tty_in;
	gpio_t *gpio_tty_out;
	gpio_t *gpio_power_ctrl;
	gpio_t *gpio_rf_stat;
	gpio_t *gpio_key;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	ic.icr = &virt_addr[13];
	ic.ier = &virt_addr[11];
	ic.isr = &virt_addr[12];
	ic.num_registered = 0;
	ic.num_handled = 0;
	ic.check_handled = rdac_check_handled;
	spin_lock_init(&ic.lock);

	gpio_ism_dio =
	    gpio_device_create_unregistered(&virt_addr[3], &virt_addr[4],
					    "ism_dio");
	gpio_ism_dio->index_write = NULL;
	gpio_ism_dio->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_ism_dio);

	gpio_rf_ctrl =
	    gpio_device_create_unregistered(&virt_addr[5], NULL, "rf_ctrl");
	gpio_rf_ctrl->index_write = NULL;
	gpio_rf_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_rf_ctrl);

	gpio_pwm =
	    gpio_device_create_unregistered(&virt_addr[6], NULL, "gpio_pwm");
	gpio_pwm->index_write = NULL;
	gpio_pwm->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_pwm);

	gpio_tty_in =
	    gpio_device_create_unregistered(&virt_addr[8], NULL, "mdm_out");
	gpio_tty_in->index_write = NULL;
	gpio_tty_in->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_tty_in);

	gpio_tty_out =
	    gpio_device_create_unregistered(&virt_addr[9], NULL, "mdm_in");
	gpio_tty_out->index_write = NULL;
	gpio_tty_out->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_tty_out);

	gpio_power_ctrl =
	    gpio_device_create_unregistered(&virt_addr[10], NULL, "power_ctrl");
	gpio_power_ctrl->index_write = NULL;
	gpio_power_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_power_ctrl);

	gpio_rf_stat =
	    gpio_device_create_unregistered(&virt_addr[14], NULL, "rf_stat");
	gpio_rf_stat->index_write = NULL;
	gpio_rf_stat->index_read = NULL;
	gpio_rf_stat->irq_data.irq = data->irq;
	gpio_rf_stat->irq_data.irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	gpio_rf_stat->irq_data.handler = rdac_handler;
	gpio_rf_stat->irq_data.get_queue = rdac_get_queue;
	gpio_rf_stat->irq_data.irq_config = rdac_irq_config;
	ecoreex_setup_data_access(data->e_access, gpio_rf_stat);

	gpio_key = gpio_device_create_unregistered(&virt_addr[15], NULL,
			"rdac_key");
	gpio_key->index_write = NULL;
	gpio_key->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_key);

	gpio_register_device(gpio_ism_dio);
	gpio_register_device(gpio_rf_ctrl);
	gpio_register_device(gpio_pwm);
	gpio_register_device(gpio_tty_in);
	gpio_register_device(gpio_tty_out);
	gpio_register_device(gpio_power_ctrl);
	gpio_register_device(gpio_rf_stat);
	gpio_register_device(gpio_key);

#ifdef CONFIG_KEYPAD
	if (data->keypad) {
		struct platform_device *k;
		k = kzalloc(sizeof(struct platform_device), GFP_KERNEL);

		k->name = "keypad";
		k->id = 0;
		k->dev.platform_data = data->keypad;
		data->keypad->controller_base = &virt_addr[7];
		data->keypad->private_data = &ic;
		k->num_resources = 0;
		platform_device_register(k);
	}
#endif

	rdac_init_spi_cs(&virt_addr[2]);

	return 0;
}

/* Rev3-specific interrupt handling code */


static int CPLD_RDAC_R3_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name,
				struct ecoreex_data *data)
{
	gpio_t *gpio_ism_dio;
	gpio_t *gpio_cell_ctrl;
	gpio_t *gpio_xbee_ctrl;
	gpio_t *gpio_ser_ctrl;
	gpio_t *gpio_cell_stat;
	gpio_t *gpio_power_ctrl;
	gpio_t *gpio_xbee_stat;
	gpio_t *gpio_pld_id;
	gpio_t *gpio_prod_id;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	ic.icr = &virt_addr[13];
	ic.ier = &virt_addr[11];
	ic.isr = &virt_addr[12];
	ic.num_registered = 0;
	ic.num_handled = 0;
	ic.check_handled = rdac_check_handled;
	spin_lock_init(&ic.lock);

	gpio_ism_dio =
	    gpio_device_create_unregistered(&virt_addr[3], &virt_addr[4],
					    "ism_dio");
	gpio_ism_dio->index_write = NULL;
	gpio_ism_dio->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_ism_dio);

	gpio_cell_ctrl =
	    gpio_device_create_unregistered(&virt_addr[5], NULL, "cell_ctrl");
	gpio_cell_ctrl->index_write = NULL;
	gpio_cell_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_cell_ctrl);

	gpio_xbee_ctrl =
	    gpio_device_create_unregistered(&virt_addr[6], NULL,
			    "xbee_ctrl");
	gpio_xbee_ctrl->index_write = NULL;
	gpio_xbee_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_xbee_ctrl);

	gpio_ser_ctrl =
	    gpio_device_create_unregistered(&virt_addr[8], NULL, "ser_ctrl");
	gpio_ser_ctrl->index_write = NULL;
	gpio_ser_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_ser_ctrl);

	gpio_cell_stat =
	    gpio_device_create_unregistered(&virt_addr[9], NULL, "cell_stat");
	gpio_cell_stat->index_write = NULL;
	gpio_cell_stat->index_read = NULL;
	gpio_cell_stat->irq_data.irq = data->irq;
	gpio_cell_stat->irq_data.irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	gpio_cell_stat->irq_data.handler = rdac_cell_handler;
	gpio_cell_stat->irq_data.get_queue = rdac_get_queue;
	gpio_cell_stat->irq_data.irq_config = rdac_cell_irq_config;
	gpio_cell_stat->irq_data.change_notify = rdac_cell_change_notify;
	ecoreex_setup_data_access(data->e_access, gpio_cell_stat);

	gpio_power_ctrl =
	    gpio_device_create_unregistered(&virt_addr[10], NULL, "power_ctrl");
	gpio_power_ctrl->index_write = NULL;
	gpio_power_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_power_ctrl);

	gpio_xbee_stat =
	    gpio_device_create_unregistered(&virt_addr[14], NULL, "xbee_stat");
	gpio_xbee_stat->index_write = NULL;
	gpio_xbee_stat->index_read = NULL;
	gpio_xbee_stat->irq_data.irq = data->irq;
	gpio_xbee_stat->irq_data.irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	gpio_xbee_stat->irq_data.handler = rdac_xbee_handler;
	gpio_xbee_stat->irq_data.get_queue = rdac_get_queue;
	gpio_xbee_stat->irq_data.irq_config = rdac_xbee_irq_config;
	gpio_xbee_stat->irq_data.change_notify = rdac_xbee_change_notify;
	ecoreex_setup_data_access(data->e_access, gpio_xbee_stat);

	gpio_pld_id = gpio_device_create_unregistered(&virt_addr[15], NULL,
			"pld_id");
	gpio_pld_id->index_write = NULL;
	gpio_pld_id->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_pld_id);

	gpio_prod_id = gpio_device_create_unregistered(&virt_addr[16], NULL,
			"prod_id");
	gpio_prod_id->index_write = NULL;
	gpio_prod_id->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_prod_id);

	gpio_register_device(gpio_ism_dio);
	gpio_register_device(gpio_cell_ctrl);
	gpio_register_device(gpio_xbee_ctrl);
	gpio_register_device(gpio_ser_ctrl);
	gpio_register_device(gpio_cell_stat);
	gpio_register_device(gpio_power_ctrl);
	gpio_register_device(gpio_xbee_stat);
	gpio_register_device(gpio_pld_id);
	gpio_register_device(gpio_prod_id);

#ifdef CONFIG_KEYPAD
	if (data->keypad) {
		struct platform_device *k;
		k = kzalloc(sizeof(struct platform_device), GFP_KERNEL);

		k->name = "keypad";
		k->id = 0;
		k->dev.platform_data = data->keypad;
		data->keypad->controller_base = &virt_addr[7];
		data->keypad->private_data = &ic;
		k->num_resources = 0;
		platform_device_register(k);
	}
#endif

	rdac_init_spi_cs(&virt_addr[2]);

	return 0;
}

/* Rev4-specific interrupt handling code */


static int CPLD_RDAC_R4_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name,
				struct ecoreex_data *data)
{
	gpio_t *gpio_ism_dio;
	gpio_t *gpio_cell_ctrl;
	gpio_t *gpio_xbee_ctrl;
	gpio_t *gpio_ser_ctrl;
	gpio_t *gpio_cell_stat;
	gpio_t *gpio_power_ctrl;
	gpio_t *gpio_xbee_stat;
	gpio_t *gpio_pld_id;
	gpio_t *gpio_prod_id;
	gpio_t *gpio_rev_id;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	ic.icr = &virt_addr[13];
	ic.ier = &virt_addr[11];
	ic.isr = &virt_addr[12];
	ic.num_registered = 0;
	ic.num_handled = 0;
	ic.check_handled = rdac_check_handled;
	spin_lock_init(&ic.lock);
	
	gpio_ism_dio =
	    gpio_device_create_unregistered(&virt_addr[3], &virt_addr[4],
					    "ism_dio");
	gpio_ism_dio->index_write = NULL;
	gpio_ism_dio->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_ism_dio);

	gpio_cell_ctrl =
	    gpio_device_create_unregistered(&virt_addr[5], NULL, "cell_ctrl");
	gpio_cell_ctrl->index_write = NULL;
	gpio_cell_ctrl->index_read = NULL;
	gpio_cell_ctrl->data_write = dtr_data_write;
	ecoreex_setup_data_access(data->e_access, gpio_cell_ctrl);

	gpio_xbee_ctrl =
	    gpio_device_create_unregistered(&virt_addr[6], NULL,
			    "xbee_ctrl");
	gpio_xbee_ctrl->index_write = NULL;
	gpio_xbee_ctrl->index_read = NULL;
	gpio_xbee_ctrl->data_write = dtr_data_write;
	ecoreex_setup_data_access(data->e_access, gpio_xbee_ctrl);

	gpio_ser_ctrl =
	    gpio_device_create_unregistered(&virt_addr[8], NULL, "ser_ctrl");
	gpio_ser_ctrl->index_write = NULL;
	gpio_ser_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_ser_ctrl);

	gpio_cell_stat =
	    gpio_device_create_unregistered(&virt_addr[9], NULL, "cell_stat");
	gpio_cell_stat->index_write = NULL;
	gpio_cell_stat->index_read = NULL;
	gpio_cell_stat->irq_data.irq = data->irq;
	gpio_cell_stat->irq_data.irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	gpio_cell_stat->irq_data.handler = rdac_cell_handler;
	gpio_cell_stat->irq_data.get_queue = rdac_get_queue;
	gpio_cell_stat->irq_data.irq_config = rdac_cell_irq_config;
	gpio_cell_stat->irq_data.change_notify = rdac_cell_change_notify;
	ecoreex_setup_data_access(data->e_access, gpio_cell_stat);

	gpio_power_ctrl =
	    gpio_device_create_unregistered(&virt_addr[10], NULL, "power_ctrl");
	gpio_power_ctrl->index_write = NULL;
	gpio_power_ctrl->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_power_ctrl);

	gpio_xbee_stat =
	    gpio_device_create_unregistered(&virt_addr[14], NULL, "xbee_stat");
	gpio_xbee_stat->index_write = NULL;
	gpio_xbee_stat->index_read = NULL;
	gpio_xbee_stat->irq_data.irq = data->irq;
	gpio_xbee_stat->irq_data.irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	gpio_xbee_stat->irq_data.handler = rdac_xbee_handler;
	gpio_xbee_stat->irq_data.get_queue = rdac_get_queue;
	gpio_xbee_stat->irq_data.irq_config = rdac_xbee_irq_config;
	gpio_xbee_stat->irq_data.change_notify = rdac_xbee_change_notify;
	ecoreex_setup_data_access(data->e_access, gpio_xbee_stat);

	gpio_pld_id = gpio_device_create_unregistered(&virt_addr[15], NULL,
			"pld_id");
	gpio_pld_id->index_write = NULL;
	gpio_pld_id->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_pld_id);

	gpio_prod_id = gpio_device_create_unregistered(&virt_addr[16], NULL,
			"carrier_board_id");
	gpio_prod_id->index_write = NULL;
	gpio_prod_id->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_prod_id);

	gpio_rev_id = gpio_device_create_unregistered(&virt_addr[17], NULL,
			"carrier_board_rev");
	gpio_rev_id->index_write = NULL;
	gpio_rev_id->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_rev_id);

	gpio_register_device(gpio_ism_dio);
	gpio_register_device(gpio_cell_ctrl);
	gpio_register_device(gpio_xbee_ctrl);
	gpio_register_device(gpio_ser_ctrl);
	gpio_register_device(gpio_cell_stat);
	gpio_register_device(gpio_power_ctrl);
	gpio_register_device(gpio_xbee_stat);
	gpio_register_device(gpio_pld_id);
	gpio_register_device(gpio_prod_id);
	gpio_register_device(gpio_rev_id);

#ifdef CONFIG_KEYPAD
	if (data->keypad) {
		struct platform_device *k;
		k = kzalloc(sizeof(struct platform_device), GFP_KERNEL);

		k->name = "keypad";
		k->id = 0;
		k->dev.platform_data = data->keypad;
		data->keypad->controller_base = &virt_addr[7];
		data->keypad->private_data = &ic;
		k->num_resources = 0;
		platform_device_register(k);
	}
#endif

	rdac_init_spi_cs(&virt_addr[2]);

	rdac_uart_cntrl_init(gpio_xbee_ctrl, gpio_cell_ctrl, gpio_cell_stat,
			&dtr_lock, &ic, data->irq);

	return 0;
}
#endif //CONFIG_ECOREEX_RDAC

#ifdef CONFIG_ECOREEX_HWMS
#define CPLD_HWMS_NAME "HWMS GPI/O expansion R1.0"
#define CPLD_HWMS 0xC0

/**
 * handler currently just indicates the that interrupt was handled and returns
 */
static irqreturn_t hwms_interrupt(int irq, void *na)
{
	return IRQ_HANDLED;
}

static int CPLD_HWMS_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name,
				struct ecoreex_data *data)
{
	gpio_t *gpio_counter;
	gpio_t *gpio_control;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	gpio_control =
	    gpio_device_create_unregistered(&virt_addr[0], NULL, "control");
	ecoreex_setup_data_access(data->e_access, gpio_control);
	/* no index read or write for the control -- only one register */
	gpio_control->index_write = NULL;
	gpio_control->index_read = NULL;

	gpio_counter =
	    gpio_device_create_unregistered(&virt_addr[1], NULL, "counter");
	gpio_counter->index_write = gpio_index_write;
	gpio_counter->index_read = gpio_index_read;
	gpio_counter->range = 1;
	ecoreex_setup_data_access(data->e_access, gpio_counter);

	gpio_register_device(gpio_control);
	gpio_register_device(gpio_counter);

	/*
	 * Allocate the IRQ
	 */
	if (data->irq) {
		int err =
		    request_irq(data->irq, hwms_interrupt, IRQ_TYPE_EDGE_RISING,
				name, NULL);
		if (err < 0)
			printk("ecoreex request for irq %d failed with %d\n",
			       data->irq, err);
		//printk("ecoreex request for irq %d failed\n",e->irq);

		else
			set_irq_wake(data->irq, 1);	//if the board provides an irq, register it to wake up pm.
	}

	return 0;
}

#endif /* CONFIG_ECOREEX_HWMS */

#ifdef CONFIG_ECOREEX_SOM150

/**
 * handler currently just indicates the that interrupt was handled and returns
 */
static irqreturn_t som150_interrupt(int irq, void *na)
{
	return IRQ_HANDLED;
}

#ifdef CONFIG_SOM_150ES_REV1

#define CPLD_SOM150_NAME "EMAC SoM-150ES GPI/O expansion R1.0"
#define CPLD_SOM150 0xC0

static int CPLD_SOM150_map(unsigned long phys_addr, u8 * virt_addr,
				  unsigned long size, const char *name,
				  struct ecoreex_data *data)
{
	gpio_t *gpio_porta;
	gpio_t *gpio_portb;
	gpio_t *gpio_portc;
	gpio_t *gpio_gpout;
	gpio_t *gpio_gpin;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	gpio_porta =
	    gpio_device_create_unregistered(&virt_addr[2], &virt_addr[3],
					    "porta");
	gpio_porta->index_write = NULL;
	gpio_porta->index_read = NULL;
	gpio_porta->range = 1;
	ecoreex_setup_data_access(data->e_access, gpio_porta);

	gpio_portb =
	    gpio_device_create_unregistered(&virt_addr[4], &virt_addr[5],
					    "portb");
	gpio_portb->index_write = NULL;
	gpio_portb->index_read = NULL;
	gpio_portb->range = 1;
	ecoreex_setup_data_access(data->e_access, gpio_portb);

	gpio_portc =
	    gpio_device_create_unregistered(&virt_addr[6], &virt_addr[7],
					    "portc");
	gpio_portc->index_write = NULL;
	gpio_portc->index_read = NULL;
	gpio_portc->range = 1;
	ecoreex_setup_data_access(data->e_access, gpio_portc);

	gpio_gpout =
	    gpio_device_create_unregistered(&virt_addr[9], NULL, "gpout");
	gpio_gpout->index_write = NULL;
	gpio_gpout->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_gpout);

	gpio_gpin =
	    gpio_device_create_unregistered(&virt_addr[10], NULL, "gpin");
	gpio_gpin->index_write = NULL;
	gpio_gpin->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_gpin);

	gpio_register_device(gpio_porta);
	gpio_register_device(gpio_portb);
	gpio_register_device(gpio_portc);
	gpio_register_device(gpio_gpout);
	gpio_register_device(gpio_gpin);

#ifdef CONFIG_KEYPAD
	if (data->keypad) {
		struct platform_device *k;
		k = kzalloc(sizeof(struct platform_device), GFP_KERNEL);

		k->name = "keypad";
		k->id = 0;
		k->dev.platform_data = data->keypad;
		data->keypad->controller_base = &virt_addr[7];
		k->num_resources = 0;
		platform_device_register(k);
	}
#endif

	/*
	 * Allocate the IRQ
	 */
	if (data->irq) {
		int err = request_irq(data->irq, som150_interrupt,
				      IRQ_TYPE_EDGE_RISING, name, NULL);
		if (err < 0)
			printk("ecoreex request for irq %d failed with %d\n",
			       data->irq, err);
		else
			set_irq_wake(data->irq, 1);	//if the board provides an irq, register it to wake up pm.
	}

	return 0;
}

#else /* REV2 */
#define CPLD_SOM150_NAME "EMAC SoM-150ES GPI/O expansion R2.0"
#define CPLD_SOM150 0xC1

/* This version re-enumerates some of the gpout lines (and removes SPI_CS)
 * and adds a register of 6 GPIO_HNDY lines */
static int CPLD_SOM150_map(unsigned long phys_addr, u8 * virt_addr,
				  unsigned long size, const char *name,
				  struct ecoreex_data *data)
{
	gpio_t *gpio_porta;
	gpio_t *gpio_portb;
	gpio_t *gpio_portc;
	gpio_t *gpio_control;
	gpio_t *gpio_gpin;
	gpio_t *gpio_hndy;

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("Could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
	}
	gpio_declare();

	gpio_porta =
	    gpio_device_create_unregistered(&virt_addr[2], &virt_addr[3],
					    "porta");
	gpio_porta->index_write = NULL;
	gpio_porta->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_porta);

	gpio_portb =
	    gpio_device_create_unregistered(&virt_addr[4], &virt_addr[5],
					    "portb");
	gpio_portb->index_write = NULL;
	gpio_portb->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_portb);

	/* High Drive */
	gpio_portc =
	    gpio_device_create_unregistered(&virt_addr[6], NULL, "portc");
	gpio_portc->index_write = NULL;
	gpio_portc->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_portc);

	gpio_control =
	    gpio_device_create_unregistered(&virt_addr[8], NULL, "control");
	gpio_control->index_write = NULL;
	gpio_control->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_control);

	gpio_gpin =
	    gpio_device_create_unregistered(&virt_addr[9], NULL, "gpin");
	gpio_gpin->index_write = NULL;
	gpio_gpin->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_gpin);

	gpio_hndy =
	    gpio_device_create_unregistered(&virt_addr[10], &virt_addr[11],
					    "handy");
	gpio_hndy->index_write = NULL;
	gpio_hndy->index_read = NULL;
	ecoreex_setup_data_access(data->e_access, gpio_hndy);

	gpio_register_device(gpio_porta);
	gpio_register_device(gpio_portb);
	gpio_register_device(gpio_portc);
	gpio_register_device(gpio_control);
	gpio_register_device(gpio_gpin);
	gpio_register_device(gpio_hndy);

#ifdef CONFIG_KEYPAD
	if (data->keypad) {
		struct platform_device *k;
		k = kzalloc(sizeof(struct platform_device), GFP_KERNEL);

		k->name = "keypad";
		k->id = 0;
		k->dev.platform_data = data->keypad;
		data->keypad->controller_base = &virt_addr[7];
		k->num_resources = 0;
		platform_device_register(k);
	}
#endif

	/*
	 * Allocate the IRQ
	 */
	if (data->irq) {
		int err = request_irq(data->irq, som150_interrupt,
				      IRQ_TYPE_EDGE_FALLING | IRQF_SHARED, "som150-pld", data);
		if (err < 0)
			printk("ecoreex request for irq %d failed with %d\n",
			       data->irq, err);
		else
			set_irq_wake(data->irq, 1);	//if the board provides an irq, register it to wake up pm.
	}

	return 0;
}
#endif /* CONFIG_SOM_150ES_REV# */

#endif /* CONFIG_ECOREEX_SOM150 */

//---------------------------------------------------------------------------------
#ifdef CONFIG_ECOREEX_TGPIO
#define CPLD_BASE_NAME "iPac GPI/O expansion R1.0"
#define CPLD_BASE 0xA0

#define CPLD_BASE_NAME_11 "iPac GPI/O expansion R1.1"
#define CPLD_BASE_11 0xA3

static inline int CPLD_BASE_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name)
{

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}
	//add appropriate class devices for this version to the system
	printk("%s detected at %lx\n", name, phys_addr);

#ifdef CONFIG_GPIOCLASS
	gpio_declare();
	gpio_device_create(&virt_addr[0], &virt_addr[1], "portw");
	gpio_device_create(&virt_addr[2], &virt_addr[3], "portx");
	gpio_device_create(&virt_addr[4], &virt_addr[5], "porty");
	gpio_device_create(&virt_addr[6], &virt_addr[7], "portz");
#endif

	return 0;
}
#endif //CONFIG_ECOREEX_TGPIO
//---------------------------------------------------------------------------------
#ifdef CONFIG_ECOREEX_SGPWM
#define CPLD_GPWM_NAME "iPac Shadow GPI/O & PWM expansion R1.0"
#define CPLD_GPWM 0xA1

#define CPLD_GPWM_NAME_11 "iPac Shadow GPI/O & PWM expansion R1.1"
#define CPLD_GPWM_11 0xA2

#define CPLD_GPWM_NAME_12 "iPac Shadow GPI/O & PWM expansion R1.2"
#define CPLD_GPWM_12 0xA4

static inline int CPLD_GPWM_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name,
				struct ecoreex_data *e)
{
	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}

	printk("%s detected at %lx\n", name, phys_addr);
	//add appropriate class devices for this version to the system

#ifdef CONFIG_GPIOCLASS
	{
		struct device *status =
		    gpi_device_create(&virt_addr[6], "status");
		gpi_device_create(&virt_addr[0], "portw");
		gpo_device_create(&virt_addr[1], "portx");
		gpo_device_create(&virt_addr[2], "porty");
		gpi_device_create(&virt_addr[3], "portz");
		gpo_device_create(&virt_addr[7], "control");

	}
#endif //CONFIG_GPIOCLASS

#ifdef CONFIG_PWMCLASS
	pwmd_device_create(&virt_addr[4], "gpwma", e->read_periodusa);
	pwmd_device_create(&virt_addr[5], "gpwmb", e->read_periodusa);
#endif

	return 0;
}

#endif //CONFIG_ECOREEX_SGPWM

//---------------------------------------------------------------------------------
#ifdef CONFIG_ECOREEX_GCMB
#define CPLD_GCMB_NAME_10 "GCMB CPLD expansion R1.0"
#define CPLD_GCMB 0xB0

static inline int CPLD_GCMB_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name)
{

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}

	printk("%s detected at %lx\n", name, phys_addr);
	//add appropriate class devices for this version to the system

#ifdef CONFIG_GPIOCLASS
	gpio_declare();
	gpo_device_create(&virt_addr[0], "dlais321");
	gpi_device_create(&virt_addr[4], "rlwc3210");
	gpio_device_create(&virt_addr[2], NULL, "spi_7730");
#endif //CONFIG_GPIOCLASS

	return 0;
}

#endif //CONFIG_ECOREEX_GCMB

//---------------------------------------------------------------------------------
#ifdef CONFIG_ECOREEX_DENM
#define CPLD_DENM_NAME_12  "DENM CPLD expansion R1.2/R1.3"
#define CPLD_DENM_NAME_14  "DENM CPLD expansion R1.4"
#define CPLD_DENM_NAME_15  "DENM CPLD expansion R1.5"
#define CPLD_DENM_NAME_16  "DENM CPLD expansion R1.6"
#define CPLD_DENM_NAME_17  "DENM CPLD expansion R1.7"
#define CPLD_DENM_NAME_18  "DENM CPLD expansion R1.8"
#define CPLD_DENM_NAME_19  "DENM CPLD expansion R1.9"
#define CPLD_DENM_NAME_110 "DENM CPLD expansion R1.10"
#define CPLD_DENM    0x92
#define CPLD_DENM14  0x93
#define CPLD_DENM15  0x94
#define CPLD_DENM16  0x95
#define CPLD_DENM17  0x98
#define CPLD_DENM18  0x99
#define CPLD_DENM19  0x9A
#define CPLD_DENM110 0x9B

static inline int CPLD_DENM_map(unsigned long phys_addr, u8 * virt_addr,
				unsigned long size, const char *name)
{

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}

	printk("%s detected at %lx\n", name, phys_addr);
	//add appropriate class devices for this version to the system

#ifdef CONFIG_GPIOCLASS
	gpio_declare();
	gpi_device_create(&virt_addr[2], "counter0");
	gpi_device_create(&virt_addr[4], "counter1");
	gpi_device_create(&virt_addr[5], "counter2");
	gpi_device_create(&virt_addr[8], "counter3");
	gpi_device_create(&virt_addr[10], "counter4");
	gpi_device_create(&virt_addr[12], "counter5");
	gpi_device_create(&virt_addr[14], "ct543210");

	gpo_device_create(&virt_addr[16], "oxrs4321");
	gpo_device_create(&virt_addr[17], "xxrt321b");

	gpio_device_create(&virt_addr[3], NULL, "rollover");
#endif //CONFIG_GPIOCLASS

	return 0;
}

static inline int CPLD_DENM14_map(unsigned long phys_addr, u8 * virt_addr,
				  unsigned long size, const char *name)
{

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}

	printk("%s detected at %lx\n", name, phys_addr);
	//add appropriate class devices for this version to the system

#ifdef CONFIG_GPIOCLASS
	gpio_declare();
	gpi_device_create(&virt_addr[2], "counter0");
	gpi_device_create(&virt_addr[4], "counter1");
	gpi_device_create(&virt_addr[6], "counter2");
	gpi_device_create(&virt_addr[8], "counter3");
	gpi_device_create(&virt_addr[10], "counter4");
	gpi_device_create(&virt_addr[12], "counter5");
	gpi_device_create(&virt_addr[14], "ct543210");

	gpio_device_create(&virt_addr[16], NULL, "oxrs4321");
	gpio_device_create(&virt_addr[17], NULL, "xxrt321b");
	gpio_device_create(&virt_addr[3], NULL, "rollover");
	gpo_device_create(&virt_addr[1], "debounce");

#endif //CONFIG_GPIOCLASS

	return 0;
}

/**
 * handler currently just indicates the that interrupt was handled and returns
 */
static irqreturn_t denm_interrupt(int irq, void *na)
{
	//printk("denm interrupt\n");
	return IRQ_HANDLED;
}

static inline int CPLD_DENM15_map(unsigned long phys_addr, u8 * virt_addr,
				  unsigned long size, const char *name,
				  struct ecoreex_data *e)
{

	if (request_mem_region(phys_addr, size, name) == NULL) {
		printk
		    ("could not obtain physical memory at %lx for EMAC core\n",
		     phys_addr);
		iounmap(virt_addr);
		return -1;
	}

	printk("%s detected at %lx\n", name, phys_addr);
	//add appropriate class devices for this version to the system

#ifdef CONFIG_GPIOCLASS
	gpio_declare();
	gpi_device_create(&virt_addr[2], "counter0");
	gpi_device_create(&virt_addr[4], "counter1");
	gpi_device_create(&virt_addr[6], "counter2");
	gpi_device_create(&virt_addr[8], "counter3");
	gpi_device_create(&virt_addr[10], "counter4");
	gpi_device_create(&virt_addr[12], "counter5");
	gpi_device_create(&virt_addr[14], "ct543210");

	gpio_device_create(&virt_addr[16], NULL, "oxrs4321");
	gpio_device_create(&virt_addr[17], NULL, "xxrt321b");
	gpio_device_create(&virt_addr[3], NULL, "rollover");
	gpio_device_create(&virt_addr[1], NULL, "debounce");

	/*
	 * Allocate the IRQ
	 */
	if (e->irq) {
		int err =
		    request_irq(e->irq, denm_interrupt, IRQ_TYPE_EDGE_RISING,
				name, NULL);
		if (err < 0)
			printk("ecoreex request for irq %d failed with %d\n",
			       e->irq, err);
		else
			set_irq_wake(e->irq, 1);	//if the board provides an irq, register it to wake up pm.
	}
#endif //CONFIG_GPIOCLASS

	return 0;
}

#endif //CONFIG_ECOREEX_DENM

/************************************************************
 * core mappings based upon a key
 */
static inline void map_core(unsigned long phys_addr, unsigned long size,
			    struct ecoreex_data *data)
{
	u8 *virt_addr = ioremap_nocache(phys_addr, size);
	int version = VERSION_KEY;

	if (virt_addr == NULL)
		printk("could not remap physical memory at %lx for EMAC core\n",
		       phys_addr);
	else {
		if (VERSION_KEY == -1)
			version = ioread8(&virt_addr[data->key_offset]);
		//    version = data->e_access->ecoreex_key_read(data->key_offset, virt_addr);
		/* else the version is predefined */
		printk("EMAC core version %x detected at %lx\n", version,
		       phys_addr + data->key_offset);
		switch (version) {
#ifdef CONFIG_ECOREEX_RDAC
		case CPLD_RDAC_R0:
			CPLD_RDAC_R0_map(phys_addr, virt_addr, size,
				      CPLD_RDAC_NAME_R0, data);
			break;
		case CPLD_RDAC_R1:
			CPLD_RDAC_R1_2_map(phys_addr, virt_addr, size,
					CPLD_RDAC_NAME_R1, data);
			break;
		case CPLD_RDAC_R2:
			CPLD_RDAC_R1_2_map(phys_addr, virt_addr, size,
					CPLD_RDAC_NAME_R2, data);
			break;
		case CPLD_RDAC_R3:
			CPLD_RDAC_R3_map(phys_addr, virt_addr, size,
					CPLD_RDAC_NAME_R3, data);
			break;
		case CPLD_RDAC_R4:
			CPLD_RDAC_R4_map(phys_addr, virt_addr, size,
					CPLD_RDAC_NAME_R4, data);
			break;
#endif
#ifdef CONFIG_ECOREEX_HWMS
		case CPLD_HWMS:
			CPLD_HWMS_map(phys_addr, virt_addr, size,
				      CPLD_HWMS_NAME, data);
			break;
#endif
#ifdef CONFIG_ECOREEX_SOM150
		case CPLD_SOM150:
			CPLD_SOM150_map(phys_addr, virt_addr, size,
					CPLD_SOM150_NAME, data);
			break;
#endif
			//-----------------------------------------------------------
#ifdef CONFIG_ECOREEX_TGPIO
		case CPLD_BASE:
			CPLD_BASE_map(phys_addr, virt_addr, size,
				      CPLD_BASE_NAME);
			break;
		case CPLD_BASE_11:
			CPLD_BASE_map(phys_addr, virt_addr, size,
				      CPLD_BASE_NAME_11);
			break;
#endif //CONFIG_ECOREEX_TGPIO
			//-----------------------------------------------------------
#ifdef CONFIG_ECOREEX_SGPWM
		case CPLD_GPWM:
			CPLD_GPWM_map(phys_addr, virt_addr, size,
				      CPLD_GPWM_NAME, data);
			break;
		case CPLD_GPWM_11:
			CPLD_GPWM_map(phys_addr, virt_addr, size,
				      CPLD_GPWM_NAME_11, data);
			break;
		case CPLD_GPWM_12:
			CPLD_GPWM_map(phys_addr, virt_addr, size,
				      CPLD_GPWM_NAME_12, data);
			break;
#endif //CONFIG_ECOREEX_SGPWM
			//-----------------------------------------------------------
#ifdef CONFIG_ECOREEX_GCMB
		case CPLD_GCMB:
			CPLD_GCMB_map(phys_addr, virt_addr, size,
				      CPLD_GCMB_NAME_10);
			break;
#endif //CONFIG_ECOREEX_GCMB
			//-----------------------------------------------------------                   
#ifdef CONFIG_ECOREEX_DENM
		case CPLD_DENM:
			CPLD_DENM_map(phys_addr, virt_addr, size,
				      CPLD_DENM_NAME_12);
			break;
		case CPLD_DENM14:
			CPLD_DENM14_map(phys_addr, virt_addr, size,
					CPLD_DENM_NAME_14);
			break;
		case CPLD_DENM15:
			CPLD_DENM15_map(phys_addr, virt_addr, size,
					CPLD_DENM_NAME_15, data);
			break;
		case CPLD_DENM16:
			CPLD_DENM15_map(phys_addr, virt_addr, size,
					CPLD_DENM_NAME_16, data);
			break;
		case CPLD_DENM17:
			CPLD_DENM15_map(phys_addr, virt_addr, size,
					CPLD_DENM_NAME_17, data);
			break;
		case CPLD_DENM18:
			CPLD_DENM15_map(phys_addr, virt_addr, size,
					CPLD_DENM_NAME_18, data);
			break;
		case CPLD_DENM19:
			CPLD_DENM15_map(phys_addr, virt_addr, size,
					CPLD_DENM_NAME_19, data);
			break;
		case CPLD_DENM110:
			CPLD_DENM15_map(phys_addr, virt_addr, size,
					CPLD_DENM_NAME_110, data);
			break;
#endif //CONFIG_ECOREEX_DENM
			//--------------------------------------------------------

		default:	//unrecognized CPLD or no CPLD available, silently fail and the release physical memory region
			iounmap(virt_addr);
		}
	}
}

static int ecoreex_probe(struct platform_device *pdev)
{
	//printk("ecoreex_probe\n");    
	if (pdev == NULL)
		return -ENODEV;
	map_core(pdev->resource->start,
		 (pdev->resource->end - pdev->resource->start + 1),
		 pdev->dev.platform_data);
	return 0;
}

//driver currently has no removal method.
static struct platform_driver ecoreex_driver = {
	.probe = ecoreex_probe,
	.driver = {
		   .name = "ecoreex",
		   },
};

#define DRV_MODULE_NAME 	"ecoreex"
#define DRV_MODULE_VERSION 	"1.2"
static int __init ecoreex_init_module(void)
{
	printk(KERN_INFO DRV_MODULE_NAME " version " DRV_MODULE_VERSION
	       " loading\n");
	return platform_driver_register(&ecoreex_driver);
}

static void __exit ecoreex_cleanup_module(void)
{
	platform_driver_unregister(&ecoreex_driver);
}

module_init(ecoreex_init_module);
module_exit(ecoreex_cleanup_module);
