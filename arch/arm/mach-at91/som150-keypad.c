#include <linux/types.h>
#include <linux/keypad.h>
#include <asm/mach/irq.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>

static void som150_init_int(struct keypad_data_s *k)
{
	at91_set_B_periph(AT91_PIN_PC15, 0);
	at91_set_deglitch(AT91_PIN_PC15, 0);
}

static int som150_get_keyheld(struct keypad_data_s *k)
{
	return ((*k->controller_base) & 0x04);
}

static int som150_int_status(struct keypad_data_s *k)
{
	int status;

	status = (1 << at91_sys_read(AT91_AIC_ISR)) & ~at91_sys_read(AT91_AIC_IMR);
	return (status & (1 << k->irq));
}

static void som150_mask_int(struct keypad_data_s *k)
{
	disable_irq(k->irq);
}

static void som150_unmask_int(struct keypad_data_s *k)
{
	enable_irq(k->irq);
}

int som150_keypad_init(struct keypad_data_s *k)
{
	k->irq = AT91SAM9260_ID_IRQ1;
	k->irq_flags = IRQ_TYPE_EDGE_FALLING | IRQF_SHARED;
	k->controller_base = NULL; /* to be set in ecoreex */
	k->polling = 0; /* use interrupt mode */
	k->mask_int = som150_mask_int;
	k->unmask_int = som150_unmask_int;
	k->int_status = som150_int_status;
	k->init_int = som150_init_int;
	k->get_keyheld = som150_get_keyheld;
	k->free_int = NULL;

	return 0;
}


