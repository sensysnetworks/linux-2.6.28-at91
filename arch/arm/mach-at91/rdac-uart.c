/**
 * arch/arm/mach-at91/uart-rdac.c
 *
 * Implementation for external UART handshaking lines on the RDAC Carrier
 * board for the SoM-9260/SoM-9G20. The struct defining the control is
 * initialized in the ecoreex driver after mapping the PLD. This code is then
 * called from the atmel serial driver.
 *
 * Copyright (C) 2009 EMAC, Inc. <support@emacinc.com>
 *
 */

#include <linux/types.h>
#include <linux/serial_core.h>
#include <linux/atmel_serial.h>
#include <linux/class/gpio.h>
#include <mach/rdac-interrupts.h>
#include <mach/rdac-uart.h>
#include <asm/mach/serial_at91.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>


struct rdac_uart_cntrl {
	unsigned int irq;
	struct gpio_s *xbee_ctrl;
	struct gpio_s *cell_ctrl;
	struct gpio_s *cell_stat;
	spinlock_t *dtr_lock;
	struct rdac_ic *ic;
	u8 initialized;
};

#define RDAC_MODEM_DTR 	(1 << 0)
#define RDAC_XBEE_DTR	(1 << 0)
#define RDAC_MODEM_RI	(1 << 4) /* RI is same position in all registers */

#define RDAC_ID_MODEM	(2)
#define RDAC_ID_XBEE	(3)

/** 
 * check to see if the port uses external handshaking 
 */
static inline int is_ex_cntrl(struct uart_port *port)
{
	if ((port->line == RDAC_ID_MODEM) || (port->line == RDAC_ID_XBEE)) {
		return 1;
	}
	return 0;
}

/** 
 * local uart_ex_cntrl structure defining the interface 
 */
static struct rdac_uart_cntrl u;

static irqreturn_t rdac_uart_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned long flags;
	u8 isr;
	u8 run_check = 0;

	spin_lock_irqsave(&u.ic->lock, flags);

	isr = ioread8(u.ic->isr);
	iowrite8(isr & RDAC_MODEM_RI, u.ic->isr);
	
	u.ic->num_handled++;
	if (u.ic->num_handled == u.ic->num_registered)
		run_check = 1;
	spin_unlock_irqrestore(&u.ic->lock, flags);
	if (run_check) {
		u.ic->check_handled(u.ic);
	}
	if (isr & RDAC_MODEM_RI) {
		/* schedule tasklet in uart driver */
		atmel_uart_handle_external(port, ATMEL_US_RI);
	}
	return IRQ_HANDLED;
}

/**
 * function to set mctrl lines (DTR in this case)
 */
void rdac_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u8 curr;
	struct gpio_s *gpio;

	if (is_ex_cntrl(port) && u.initialized) {
		gpio = (port->line == RDAC_ID_MODEM) ? u.cell_ctrl :
			u.xbee_ctrl;
		spin_lock(u.dtr_lock);
		curr = ioread8(gpio->data);
		
		/* Note: RDAC_MODEM_DTR == RDAC_XBEE_DTR! */
		if (mctrl & TIOCM_DTR) {
			/* set DTR bit = 0 (active low) */
			curr &= ~(RDAC_MODEM_DTR);
		} else {
			/* set DTR bit = 1 */
			curr |= RDAC_MODEM_DTR;
		}
		
		iowrite8(curr, gpio->data);
		spin_unlock(u.dtr_lock);
	}
}

/**
 * function to get the current status of mctrl lines (RI)
 */
unsigned int rdac_uart_get_mctrl(struct uart_port *port)
{
	unsigned int ret = 0;

	if ((port->line == RDAC_ID_MODEM) && u.initialized) {
		/* cell_stat is readonly so no need for locking */
		/* RI is active low */
		if (ioread8(u.cell_stat->data) & RDAC_MODEM_RI) {
			ret = 0;
		} else {
			ret = TIOCM_RI;
		}
	}
	return ret;
}

/**
 * function to enable modem status interrupts (RI)
 */
void rdac_uart_enable_ms(struct uart_port *port)
{
	unsigned long flags;
	u8 ier;

	if ((port->line == RDAC_ID_MODEM) && u.initialized) {
		spin_lock_irqsave(&u.ic->lock, flags);
		iowrite8(RDAC_MODEM_RI, u.ic->isr);
		ier = ioread8(u.ic->ier);
		iowrite8(ier | RDAC_MODEM_RI, u.ic->ier);
		spin_unlock_irqrestore(&u.ic->lock, flags);
	}
}

/**
 * function for opening port and registering (not enabling) interrupts
 */
int rdac_uart_open(struct uart_port *port)
{
	int ret = 0;
	unsigned long flags;

	if (port->line != RDAC_ID_MODEM)
		return 0;
	if (!u.initialized) 
		return 1;

	ret = request_irq(u.irq, rdac_uart_interrupt, IRQF_SHARED |
			IRQF_TRIGGER_RISING, "rdac-uart", port);

	spin_lock_irqsave(&u.ic->lock, flags);
	u.ic->num_registered++;
	spin_unlock_irqrestore(&u.ic->lock, flags);

	return ret;
}

/**
 * function for cleanup and freeing irq's
 */
void rdac_uart_close(struct uart_port *port)
{
	unsigned long flags;
	u8 ier;

	/* nothing to do for XBEE interface, free and disable IRQ on MODEM */
	if (port->line == RDAC_ID_MODEM) {
		spin_lock_irqsave(&u.ic->lock, flags);
		ier = ioread8(u.ic->ier);
		iowrite8(ier & ~RDAC_MODEM_RI, u.ic->ier);
		free_irq(u.irq, port);
		u.ic->num_registered--;
		spin_unlock_irqrestore(&u.ic->lock, flags);
	}
}

/**
 * initialize the local rdac_uart_cntrl struct. Called from the ecoreex
 * initialization.
 */
void rdac_uart_cntrl_init(struct gpio_s *xbee_ctrl, struct gpio_s *cell_ctrl,
		struct gpio_s *cell_stat, spinlock_t *dtr_lock, 
		struct rdac_ic *ic, unsigned int irq)
{
	u.xbee_ctrl = xbee_ctrl;
	u.cell_ctrl = cell_ctrl;
	u.cell_stat = cell_stat;
	u.dtr_lock = dtr_lock;
	u.ic = ic;
	u.irq = irq;
	u.initialized = 1;
}

void __init rdac_uart_fns_init(void)
{
	/* temporary variable used only in init */
	struct atmel_port_fns fns;
	memset(&fns, 0, sizeof(fns));

	fns.open = rdac_uart_open;
	fns.close = rdac_uart_close;
	fns.enable_ms_hook = rdac_uart_enable_ms;
	fns.get_mctrl_hook = rdac_uart_get_mctrl;
	fns.set_mctrl_hook = rdac_uart_set_mctrl;

	atmel_register_uart_fns(&fns);
}

