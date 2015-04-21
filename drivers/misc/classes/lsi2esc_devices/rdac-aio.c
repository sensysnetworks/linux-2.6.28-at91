/*
 * drivers/misc/classes/lsi2esc_devices/rdaci-aio.c
 * EMAC.Inc support for GPIO interface to the RDAC carrier AIO expansion board
 * PLD. Provides a generic wrapper to the LSI2ESC functions used to
 * communicate with the device.
 *
 * Copyright (C) 2009 EMAC.Inc <support@emacinc.com>
 */

#include <linux/class/lsi2esc/rdac-aio.h>
#include <linux/class/spi.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <linux/interrupt.h>

/* Register definitions */
#define AIO_REGULATOR	0x00
#define AIO_INPUT_CTRL	0x01
#define AIO_GPIO_DATA	0x02
#define AIO_GPIO_DDR	0x03
#define AIO_ANALOG_CTRL	0x04
#define AIO_ANALOG_STAT	0x05
#define AIO_CNTA_MSB	0x06
#define AIO_CNTA_LSB	0x07
#define AIO_CNTB_MSB	0x08
#define AIO_CNTB_LSB	0x09
#define AIO_PWM		0x0A
#define AIO_IER		0x0B
#define AIO_ISR		0x0C
#define AIO_ID		0x0D
#define AIO_REV		0x0E
#define AIO_PLD_KEY	0x0F
#define AIO_INTERVAL	0x10

#define AIO_IRQ_GPIO0	(1 << 0)
#define AIO_IRQ_GPIO1	(1 << 1)
#define AIO_IRQ_GPIO2	(1 << 2)
#define AIO_IRQ_GPIO3	(1 << 3)
#define AIO_IRQ_ERRF1	(1 << 4)
#define AIO_IRQ_ERRF2	(1 << 5)
#define AIO_IRQ_AICRDY	(1 << 6)
#define AIO_IRQ_INTER	(1 << 7)

#define AIO_IRQ_NUM_BITS (7)

#define AIO_GPIO_IRQ_NUM_BITS	(4)
#define AIO_GPIO_IRQ_ALL	(AIO_IRQ_GPIO0 | \
		AIO_IRQ_GPIO1 | \
		AIO_IRQ_GPIO2 | \
		AIO_IRQ_GPIO3)

#define AIO_ANALOG_IRQ_NUM_BITS	(4)
#define AIO_ANALOG_IRQ_ALL	(AIO_IRQ_ERRF1 | \
		AIO_IRQ_ERRF2  | \
		AIO_IRQ_AICRDY | \
		AIO_IRQ_INTER)

#define AIO_ANALOG_SHIFT 4
#define AIO_NUM_DEVICES	12

static struct spi_s *spi_dev;

DEFINE_MUTEX(aio_lock);
static int num_handled;
static int num_registered;

static u8 aio_irq_usage[AIO_IRQ_NUM_BITS];
static u8 aio_gpio_bitpos[AIO_GPIO_IRQ_NUM_BITS] = { 0, 1, 2, 3 };
static u8 aio_analog_bitpos[AIO_ANALOG_IRQ_NUM_BITS] = { 4, 5, 6, 7 };


static void aio_check_handled(struct gpio_s *gpio);
static void aio_update_gpio_ier(struct gpio_s *gpio);

/**
 * function to initialize the settings for the interface.
 */
static void rdac_aio_init(void)
{
	static spi_control config = SPICL_EIGHTBIT;
	spi_dev->confwrite(spi_dev, config);
}

/**
 * function to read the value of a given register in the AIO PLD
 */
static gpio_data aio_data_read(__u8 reg, struct gpio_s *gpio)
{
	u32 size;
	u8 mosi[3];
	u8 miso[3];

	/*
	 * command structure:
	 * byte 0[7]   : WR/~RD
	 * byte 0[6:0] : ADDRESS
	 * byte 1[7:0] : WRITE VALUE (ignored for read)
	 * byte 2[7:0] : READ VALUE (returned from device)
	 */
	mosi[0] = reg & 0x7F;
	/* bytes 1 and 2 ignored */
	mosi[1] = 0xFF;
	mosi[2] = 0xFF;
	size = 3;

	memset(miso, 0x0, sizeof(miso));
	spi_dev->xmit(spi_dev, mosi, miso, size);
	return miso[2];
}

/**
 * register read wrapper functions
 */

static gpio_data aio_regulator_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_REGULATOR, gpio);
}

static gpio_data aio_analog_input_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_INPUT_CTRL, gpio);
}

static gpio_data aio_gpio_data_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_GPIO_DATA, gpio);
}

static gpio_data aio_gpio_ddr_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_GPIO_DDR, gpio);
}

static gpio_data aio_analog_ctrl_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_ANALOG_CTRL, gpio);
}

static gpio_data aio_analog_stat_read(struct gpio_s *gpio)
{
	/* customer wants these bits to read inverted as they are active low
	 * in hardware */

	return (~(aio_data_read(AIO_ANALOG_STAT, gpio)) & 0x07);
}

/* TODO: the PLD design should allow reading both the MSB and the LSB of the
 * counters in one extended SPI transaction consisting of the first command (3
 * bytes) followed by a dummy byte, followed by the command to read the LSB.
 * This would improve performance and accuracy slightly.
 */
static gpio_data aio_cnt_a_read(struct gpio_s *gpio)
{
	gpio_data cnta;

	cnta = aio_data_read(AIO_CNTA_MSB, gpio) << 8;
	cnta |= aio_data_read(AIO_CNTA_LSB, gpio);
	return cnta;
}

static gpio_data aio_cnt_b_read(struct gpio_s *gpio)
{
	gpio_data cntb;

	cntb = aio_data_read(AIO_CNTB_MSB, gpio) << 8;
	cntb |= aio_data_read(AIO_CNTB_LSB, gpio);
	return cntb;
}

static gpio_data aio_ier_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_IER, gpio);
}

static gpio_data aio_isr_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_ISR, gpio);
}

static gpio_data aio_pwm_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_PWM, gpio);
}

static gpio_data aio_key_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_PLD_KEY, gpio);
}

static gpio_data aio_id_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_ID, gpio);
}

static gpio_data aio_rev_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_REV, gpio);
}

static gpio_data aio_int_read(struct gpio_s *gpio)
{
	return aio_data_read(AIO_INTERVAL, gpio);
}

/**
 * function to write the value of a given register in the AIO PLD
 */
static int aio_data_write(__u8 reg, struct gpio_s *gpio, gpio_data data)
{
	u32 size;
	u8 mosi[3];
	u8 miso[3];

	/*
	 * command structure:
	 * byte 0[7]   : WR/~RD
	 * byte 0[6:0] : ADDRESS
	 * byte 1[7:0] : WRITE VALUE
	 * byte 2[7:0] : READ VALUE (ignored on write)
	 */
	mosi[0] = reg | 0x80;
	mosi[1] = (data & 0xFF);
	mosi[2] = 0xFF;
	size = 3;

	spi_dev->xmit(spi_dev, mosi, miso, size);
	return 0;
}

/**
 * register write wrapper functions
 */
static int aio_regulator_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_REGULATOR, gpio, data);
}

static int aio_analog_input_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_INPUT_CTRL, gpio, data);
}

static int aio_gpio_data_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_GPIO_DATA, gpio, data);
}

static int aio_gpio_ddr_write(struct gpio_s *gpio, gpio_data data)
{
	int ret;
	ret = aio_data_write(AIO_GPIO_DDR, gpio, data);
	aio_update_gpio_ier(gpio);
	return ret;
}

static int aio_analog_ctrl_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_ANALOG_CTRL, gpio, data);
}

static int aio_cnt_a_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_CNTA_MSB, gpio, data);
}

static int aio_cnt_b_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_CNTB_MSB, gpio, data);
}

static int aio_ier_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_IER, gpio, data);
}

static int aio_isr_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_ISR, gpio, data);
}

static int aio_pwm_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_PWM, gpio, data);
}

static int aio_int_write(struct gpio_s *gpio, gpio_data data)
{
	return aio_data_write(AIO_INTERVAL, gpio, data);
}

/*
 * Interrupt handling functions
 */

/**
 * function to handle interrupts generated by the AIO PLD interrupt
 * controller. This function is called in a workqueue by the GPIO class
 * character driver interface. This function will handle GPIO interrupts.
 */
static gpio_data aio_gpio_handler(struct gpio_s *gpio)
{
	gpio_data tmp;
	u8 run_check = 0;

	if (mutex_lock_interruptible(&aio_lock)) {
		aio_gpio_handler(gpio);
		return 0;
	}

	tmp = aio_isr_read(gpio);
	if (tmp & AIO_GPIO_IRQ_ALL) {
		/* wrap in if() to prevent unnecessary SPI transactions */
		aio_isr_write(gpio, tmp & AIO_GPIO_IRQ_ALL);
	}
	
	num_handled++;
	if (num_handled == num_registered)
		run_check = 1;

	mutex_unlock(&aio_lock);

	if (run_check)
		aio_check_handled(gpio);

	return tmp & AIO_GPIO_IRQ_ALL;
}

/**
 * function to get the data to add to the device-specific queue. Called from
 * the GPIO class character device interface. Note that this is called with
 * the gpio->lock mutex held. This function is used with the GPIO device.
 */
static gpio_data aio_gpio_get_queue(struct gpio_s *gpio, gpio_data notify)
{
	return atomic_gpio_data_read(gpio);
}

/**
 * function to configure and enable the interrupts on the AIO PLD interrupt
 * controller for GPIO interrupts
 */
static void aio_gpio_irq_config(struct gpio_s *gpio)
{
	gpio_data tmp;

	if (mutex_lock_interruptible(&aio_lock)) {
		return;
	}
	num_registered++;

	/* clear the status register */
	aio_isr_write(gpio, AIO_GPIO_IRQ_ALL);
	/* disable all interrupts */
	tmp = aio_ier_read(gpio);
	aio_ier_write(gpio, tmp & (~AIO_GPIO_IRQ_ALL));

	mutex_unlock(&aio_lock);
	
	set_irq_wake(gpio->irq_data.irq, 1);
}

/**
 * function to handle interrupts generated by the AIO PLD interrupt
 * controller. This function is called in a workqueue by the GPIO class
 * character driver interface. This function will handle analog status
 * interrupts.
 */
static gpio_data aio_analog_handler(struct gpio_s *gpio)
{
	gpio_data tmp;
	u8 run_check = 0;

	if (mutex_lock_interruptible(&aio_lock)) {
		aio_analog_handler(gpio);
		return 0;
	}

	tmp = aio_isr_read(gpio);
	if (tmp & AIO_ANALOG_IRQ_ALL) {
		/* wrap in if() to prevent unnecessary SPI transactions */
		aio_isr_write(gpio, tmp & AIO_ANALOG_IRQ_ALL);
	}

	num_handled++;
	if (num_handled == num_registered)
		run_check = 1;

	mutex_unlock(&aio_lock);

	if (run_check)
		aio_check_handled(gpio);

	return (tmp & AIO_ANALOG_IRQ_ALL) >> AIO_ANALOG_SHIFT;
}

/**
 * function to check if all interrupts have been handled in this pass. This
 * prevents missed interrupts that can occur due to the fact that the AIO
 * interrupt is level-triggered and shared among several devices.
 */
static void aio_check_handled(struct gpio_s *gpio)
{
	gpio_data isr;
	gpio_data ier;

	if (mutex_lock_interruptible(&aio_lock))
		return;

	num_handled = 0;
	isr = aio_isr_read(gpio);
	ier = aio_ier_read(gpio);

	if (isr & ier) {
		/* there may be an unhandled interrupt, trigger a pulse on the
		 * interrupt line without touching the ISR
		 */
		aio_ier_write(gpio, 0);
		aio_ier_write(gpio, ier);
	}
	mutex_unlock(&aio_lock);
}

static void aio_update_gpio_ier(struct gpio_s *gpio)
{
	int i;
	gpio_data new_ier = 0;
	gpio_data ier;
	gpio_data ddr;

	/* determine DDR... mask important bits */
	ddr = aio_gpio_ddr_read(gpio) & 0x0F;

	for (i = 0; i < AIO_GPIO_IRQ_NUM_BITS; i++) {
		new_ier |= ((aio_irq_usage[aio_gpio_bitpos[i]] ? 1 : 0) <<
				aio_gpio_bitpos[i]);
	}
	/* if DDR[n] = 1 -> mask interrupt regardless of usage */
	new_ier &= ~ddr;
	if (mutex_lock_interruptible(&aio_lock))
		return;
	/* mask so that only gpio bits are affected */
	ier = aio_ier_read(gpio) & ~AIO_GPIO_IRQ_ALL;
	ier |= new_ier;
	aio_ier_write(gpio, ier);
	mutex_unlock(&aio_lock);
}

/**
 * function to enable/disable interrupts based on a change in the notification
 * mask of one of the GPIO devices.
 */
static void aio_gpio_change_notify(struct gpio_s *gpio, gpio_data curr_notify,
		gpio_data new_notify)
{
	int i;
	int ier_change = 0;
	gpio_data mask;

	for (i = 0; i < AIO_GPIO_IRQ_NUM_BITS; i++) {
		mask = 1 << (aio_gpio_bitpos[i]);
		if ((mask & curr_notify) && !(mask & new_notify)) {
			/* bit disabled, dec usage */
			if ((--aio_irq_usage[aio_gpio_bitpos[i]]) == 0) {
				ier_change = 1;
			}
		} else if ((mask & new_notify) && !(mask & curr_notify)) {
			/* bit enabled, inc usage */
			if ((++aio_irq_usage[aio_gpio_bitpos[i]]) == 1) {
				ier_change = 1;
			}
		}
	}
	if (ier_change)
		aio_update_gpio_ier(gpio);
}

static void aio_update_analog_ier(struct gpio_s *gpio)
{
	int i;
	gpio_data new_ier = 0;
	gpio_data ier;

	for (i = 0; i < AIO_ANALOG_IRQ_NUM_BITS; i++) {
		new_ier |= ((aio_irq_usage[aio_analog_bitpos[i]] ? 1 : 0) <<
				aio_analog_bitpos[i]);
	}
	if (mutex_lock_interruptible(&aio_lock))
		return;
	/* mask so that only analog bits are affected */
	ier = aio_ier_read(gpio) & ~AIO_ANALOG_IRQ_ALL;
	ier |= new_ier;
	aio_ier_write(gpio, ier);
	mutex_unlock(&aio_lock);
}

/**
 * function to enable/disable interrupts based on a change in the notification
 * mask of one of the GPIO devices.
 */
static void aio_analog_change_notify(struct gpio_s *gpio, gpio_data curr_notify,
		gpio_data new_notify)
{
	int i;
	int ier_change = 0;
	gpio_data mask;

	curr_notify <<= AIO_ANALOG_SHIFT;
	new_notify <<= AIO_ANALOG_SHIFT;

	for (i = 0; i < AIO_ANALOG_IRQ_NUM_BITS; i++) {
		mask = 1 << (aio_analog_bitpos[i]);
		if ((mask & curr_notify) && !(mask & new_notify)) {
			/* bit disabled, dec usage */
			if ((--aio_irq_usage[aio_analog_bitpos[i]]) == 0) {
				ier_change = 1;
			}
		} else if ((mask & new_notify) && !(mask & curr_notify)) {
			/* bit enabled, inc usage */
			if ((++aio_irq_usage[aio_analog_bitpos[i]]) == 1) {
				ier_change = 1;
			}
		}
	}
	if (ier_change)
		aio_update_analog_ier(gpio);
}

/**
 * function to get the data to add to the device-specific queue. Called from
 * the GPIO class character device interface. Note that this is called with
 * the gpio->lock mutex held. This function is used with the analog status
 * device.
 */
static gpio_data aio_analog_get_queue(struct gpio_s *gpio, gpio_data notify)
{
	return atomic_gpio_data_read(gpio);
}

/**
 * function to configure and enable the interrupts on the AIO PLD interrupt
 * controller for analog status interrupts 
 */
static void aio_analog_irq_config(struct gpio_s *gpio)
{
	u32 tmp;

	if (mutex_lock_interruptible(&aio_lock)) {
		return;
	}
	num_registered++;

	/* clear the status register */
	aio_isr_write(gpio, AIO_ANALOG_IRQ_ALL);
	/* disable all interrupts */
	tmp = aio_ier_read(gpio);
	aio_ier_write(gpio, tmp & (~AIO_ANALOG_IRQ_ALL));

	mutex_unlock(&aio_lock);
	
	set_irq_wake(gpio->irq_data.irq, 1);
}

/**
 * function to initialize and register a gpio device with
 * the functions passed as parameters
 */
static struct device *aio_gpio_init(struct gpio_s *gpio, gpio_data(*data_read)
			      (struct gpio_s * gpio),
			     gpio_data(*ddr_read) (struct gpio_s * gpio),
			     int (*data_write) (struct gpio_s * gpio,
						gpio_data data),
			     int (*ddr_write) (struct gpio_s * gpio,
					       gpio_data data),
			     const char *name, int use_irq)
{
	gpio->name = name;
	gpio->data_read = data_read;
	gpio->ddr_read = ddr_read;
	gpio->data_write = data_write;
	gpio->ddr_write = ddr_write;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->index_read = NULL;
	gpio->index_write = NULL;

	if (use_irq) {
		gpio->irq_data.irq = AT91SAM9260_ID_IRQ0;
		gpio->irq_data.irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
	}

	return gpio_register_device(gpio);
}

/**
 * function to register the classes
 */
struct device *rdac_aio_create(struct spi_s *spi, const char *name)
{
	struct gpio_s *gpio[AIO_NUM_DEVICES];
	int i;
	struct device *dev;

	spi_dev = spi;
	rdac_aio_init();

	for (i = 0; i < AIO_NUM_DEVICES; i++) {
		gpio[i] = kmalloc(sizeof(struct gpio_s), GFP_KERNEL);
		memset(gpio[i], 0, sizeof(struct gpio_s));
	}
	printk(KERN_INFO "Registering RDAC AIO devices\n");

	i = 0;
	dev = aio_gpio_init(gpio[i++], aio_regulator_read, NULL,
			    aio_regulator_write, NULL, "aio_regulator", 0);
	dev = aio_gpio_init(gpio[i++], aio_analog_input_read, NULL,
			    aio_analog_input_write, NULL, "aio_input_ctrl", 0);

	gpio[i]->irq_data.handler = aio_gpio_handler;
	gpio[i]->irq_data.get_queue = aio_gpio_get_queue;
	gpio[i]->irq_data.irq_config = aio_gpio_irq_config;
	gpio[i]->irq_data.change_notify = aio_gpio_change_notify;
	dev = aio_gpio_init(gpio[i++], aio_gpio_data_read, aio_gpio_ddr_read,
			    aio_gpio_data_write, aio_gpio_ddr_write,
			    "aio_gpio", 1);
	dev =
	    aio_gpio_init(gpio[i++], aio_analog_ctrl_read, NULL,
			  aio_analog_ctrl_write, NULL, "aio_analog_ctrl", 0);

	gpio[i]->irq_data.handler = aio_analog_handler;
	gpio[i]->irq_data.get_queue = aio_analog_get_queue;
	gpio[i]->irq_data.irq_config = aio_analog_irq_config;
	gpio[i]->irq_data.change_notify = aio_analog_change_notify;
	dev =
	    aio_gpio_init(gpio[i++], aio_analog_stat_read, NULL,
			  gpio_empty_write, NULL, "aio_analog_stat", 1);

	dev =
	    aio_gpio_init(gpio[i++], aio_cnt_a_read, NULL, aio_cnt_a_write,
			  NULL, "aio_counter_a", 0);
	dev =
	    aio_gpio_init(gpio[i++], aio_cnt_b_read, NULL, aio_cnt_b_write,
			  NULL, "aio_counter_b", 0);
	dev = aio_gpio_init(gpio[i++], aio_pwm_read, NULL, aio_pwm_write,
			NULL, "aio_pwm", 0);
	dev =
	    aio_gpio_init(gpio[i++], aio_id_read, NULL, gpio_empty_write, NULL,
			  "daughter_board_id", 0);
	dev =
	    aio_gpio_init(gpio[i++], aio_rev_read, NULL, gpio_empty_write, NULL,
			  "daughter_board_rev", 0);
	dev =
	    aio_gpio_init(gpio[i++], aio_key_read, NULL, gpio_empty_write, NULL,
			  "aio_key", 0);
	dev =
	    aio_gpio_init(gpio[i++], aio_int_read, NULL, aio_int_write, NULL,
			  "aio_interval", 0);

	return dev;
}

