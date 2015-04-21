#include <linux/types.h>
#include <linux/keypad.h>
#include <asm/mach/irq.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <mach/rdac-interrupts.h>

#define RDAC_IC_KEYPAD			(1 << 7)
#define RDAC_IC_KEYPAD_TRIGGER_FALLING	(1 << 7)
#define RDAC_IC_KEYPAD_TRIGGER_RISING	(0 << 7)

static void rdac_init_int(struct keypad_data_s *k)
{
	struct rdac_ic *ic = k->private_data;
	unsigned long flags;
	u8 tmp;

	at91_set_B_periph(AT91_PIN_PC15, 0);
	at91_set_deglitch(AT91_PIN_PC15, 0);

	spin_lock_irqsave(&ic->lock, flags);

	ic->num_registered++;
	/* trigger on falling edge */ 
	tmp = ioread8(ic->icr) | RDAC_IC_KEYPAD_TRIGGER_FALLING;
	iowrite8(tmp, ic->icr);
	iowrite8(RDAC_IC_KEYPAD, ic->isr);
	tmp = ioread8(ic->ier) | RDAC_IC_KEYPAD;
	iowrite8(tmp, ic->ier);
	spin_unlock_irqrestore(&ic->lock, flags);
}

static int rdac_get_keyheld(struct keypad_data_s *k)
{
	return (ioread8(k->controller_base) & 0x04);
}

static int rdac_int_status(struct keypad_data_s *k)
{
	struct rdac_ic *ic = k->private_data;
	unsigned long flags;
	u8 tmp;
	u8 run_check = 0;

	spin_lock_irqsave(&ic->lock, flags);
	tmp = ioread8(ic->isr);
	iowrite8(tmp & RDAC_IC_KEYPAD, ic->isr);

	ic->num_handled++;
	if (ic->num_handled == ic->num_registered)
		run_check = 1;
	spin_unlock_irqrestore(&ic->lock, flags);
	
	if (run_check) {
		ic->check_handled(ic);
	}

	return (tmp & RDAC_IC_KEYPAD);
}

static void rdac_mask_int(struct keypad_data_s *k)
{
	struct rdac_ic *ic = k->private_data;
	unsigned long flags;
	u8 tmp;

	spin_lock_irqsave(&ic->lock, flags);
	tmp = ioread8(ic->ier) & ~RDAC_IC_KEYPAD;
	iowrite8(tmp, ic->ier);
	spin_unlock_irqrestore(&ic->lock, flags);
}

static void rdac_unmask_int(struct keypad_data_s *k)
{
	struct rdac_ic *ic = k->private_data;
	unsigned long flags;
	u8 tmp;
	
	/* clear int and re-enable */
	spin_lock_irqsave(&ic->lock, flags);
	iowrite8(RDAC_IC_KEYPAD, ic->isr);
	tmp = ioread8(ic->ier) | RDAC_IC_KEYPAD;
	iowrite8(tmp, ic->ier);
	spin_unlock_irqrestore(&ic->lock, flags);
}

int rdac_keypad_init(struct keypad_data_s *k)
{
	k->irq = AT91SAM9260_ID_IRQ1;
	k->irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	k->controller_base = NULL; /* to be set in ecoreex */
	k->polling = 0; /* use interrupt mode */
	k->mask_int = rdac_mask_int;
	k->unmask_int = rdac_unmask_int;
	k->int_status = rdac_int_status;
	k->init_int = rdac_init_int;
	k->get_keyheld = rdac_get_keyheld;
	k->free_int = rdac_mask_int;
	k->private_data = NULL; /* to be set in ecoreex */

	return 0;
}

