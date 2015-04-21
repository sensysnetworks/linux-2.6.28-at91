#ifndef RDAC_UART_H_
#define RDAC_UART_H_

void rdac_uart_fns_init(void);

void rdac_uart_cntrl_init(struct gpio_s *xbee_ctrl, struct gpio_s *cell_ctrl,
		struct gpio_s *cell_stat, spinlock_t *dtr_lock, 
		struct rdac_ic *ic, unsigned int irq);

#endif /* RDAC_UART_H_ */
