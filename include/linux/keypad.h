#ifndef _LINUX_KEYPAD_H_
#define _LINUX_KEYPAD_H_

/**
 * struct to be passed in through platform registration
 * to define access to the keypad controller
 * @mask_int: function pointer to the code used to mask the interrupt
 * @unmask_int: function pointer to the code used for unmasking the interrupt
 * @init_int: function pointer to code that should be called before
 *   request_irq to set up the interrupt line properly
 * @free_int: function pointer to code that should be called before calling
 *   free_irq to release the interrupt
 * @int_status: function pointer to code used for checking if the interrupt is
 *   active, returns 1 if the interrupt is active, 0 otherwise
 * @controller_base: pointer to the base register of the controller
 * @get_keyheld: function pointer, returns true if key is currently pressed
 * @irq: the IRQ for this device
 * @irq_flags: the flags to use for request_irq (shared, trigger, etc)
 * @polling: flag specifying that the device should be polled rather than
 *   interrupt driven
 * @private_data: pointer to data that may be necessary on some platforms for
 *   implementing the required functionality
 */
struct keypad_data_s {
	void (*mask_int)(struct keypad_data_s *k);
	void (*unmask_int)(struct keypad_data_s *k);
	void (*init_int)(struct keypad_data_s *k);
	void (*free_int)(struct keypad_data_s *k);
	int (*int_status)(struct keypad_data_s *k);
	int (*get_keyheld)(struct keypad_data_s *k);
	u8 *controller_base;
	unsigned int irq;
	unsigned int irq_flags;
	unsigned int polling;
	void *private_data;
};

#endif /* _LINUX_KEYPAD_H_ */
