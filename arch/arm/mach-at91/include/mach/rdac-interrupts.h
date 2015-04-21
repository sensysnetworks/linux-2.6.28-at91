#ifndef RDAC_INTERRUPTS_H_
#define RDAC_INTERRUPTS_H_

#include <asm/atomic.h>
#include <linux/spinlock.h>

struct rdac_ic;

/**
 * struct used to describe the rdac interrupt controller registers used by the
 * keypad driver. The ecoreex driver passes in pointers to the ISR, ICR, and
 * IER on the RDAC PLD.
 * @lock: spinlock to control access to the registers in the interrupt
 *   controller
 * @check_handled: function to evaluate if all interrupts have been handled.
 *   This is important b/c the interrupt is level-triggered so any interrupts
 *   that occur while the interrupt line is active have the potential to be
 *   missed, holding the interrupt line indefinitely.
 * @num_registered: the number of devices registered on this interrupt line
 * @num_handled: the number of devices/handlers that have been processed
 * @icr: Interrupt config register (trigger level)
 * @ier: Interrupt enable register
 * @isr: Interrupt status register
 */
struct rdac_ic {
	spinlock_t lock;
	void(*check_handled) (struct rdac_ic *ic);
	u8 num_registered;
	u8 num_handled;
	u8 *icr;
	u8 *ier;
	u8 *isr;
};

#endif /* RDAC_INTERRUPTS_H_ */
