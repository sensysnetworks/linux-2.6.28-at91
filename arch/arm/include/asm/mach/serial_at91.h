/*
 *  arch/arm/include/asm/mach/serial_at91.h
 *
 *  Based on serial_sa1100.h  by Nicolas Pitre
 *
 *  Copyright (C) 2002 ATMEL Rousset
 *
 *  Low level machine dependent UART functions.
 */

struct uart_port;

/*
 * This is a temporary structure for registering these
 * functions; it is intended to be discarded after boot.
 */
struct atmel_port_fns {
	void	(*set_mctrl)(struct uart_port *, u_int);
	u_int	(*get_mctrl)(struct uart_port *);
	void	(*enable_ms)(struct uart_port *);
	void	(*pm)(struct uart_port *, u_int, u_int);
	int	(*set_wake)(struct uart_port *, u_int);
	int	(*open)(struct uart_port *);
	void	(*close)(struct uart_port *);
	/* additional hooks to enable more flexible external control rather
	 * than replacing internal functions completely */
	void	(*set_mctrl_hook)(struct uart_port *, u_int);
	u_int	(*get_mctrl_hook)(struct uart_port *);
	void	(*enable_ms_hook)(struct uart_port *);
};

#if defined(CONFIG_SERIAL_ATMEL)
void atmel_register_uart_fns(struct atmel_port_fns *fns);
int atmel_auto485(struct uart_port *port, int fos);
void atmel_uart_handle_external(struct uart_port *port, unsigned int status);
#else
#define atmel_register_uart_fns(fns) do { } while (0)
#define atmel_auto485(port, fos) do { } while (0)
#define atmel_uart_handle_external(p, s) do { } while (0)
#endif


