#ifndef ECOREEX_H_
#define ECOREEX_H_

#include <linux/class/pwm.h>
#include <linux/class/gpio.h>
#include <linux/keypad.h>

#ifdef CONFIG_ECOREEX_KEY
#define VERSION_KEY CONFIG_ECOREEX_STATIC_KEY
#else
#define VERSION_KEY -1
#endif

/* Use this structure to specify read/write commands
 * that differ from the direct ioread/write functions
 * in the gpio class. If these members are not specified,
 * the corresponding gpio function will be used as the
 * default
 */
struct ecoreex_access_s
{
    gpio_data (*ecoreex_data_read)(struct gpio_s *gpio);
    int (*ecoreex_data_write)(struct gpio_s *gpio,gpio_data data);
    gpio_data (*ecoreex_ddr_read)(struct gpio_s *gpio); 
    int (*ecoreex_ddr_write)(struct gpio_s *gpio,gpio_data data);
    gpio_data (*ecoreex_index_read)(struct gpio_s *gpio);   
    int (*ecoreex_index_write)(struct gpio_s *gpio,gpio_data data);
    u8 (*ecoreex_key_read)(u8 index, u8 *virt_addr); /* function to read the key offset -- defaults to ioread8 */
};

struct ecoreex_data
{
    int key_offset;
#ifdef CONFIG_PWMCLASS
    __u32 (*read_periodusa)(pwm_t *pwm);//a pwm input clock, defined as a period to minimize calculation
#endif
#ifdef CONFIG_KEYPAD
    struct keypad_data_s *keypad;
#endif
    unsigned int irq;
    struct ecoreex_access_s *e_access;  
};

static inline u8 ecoreex_default_key_read(u8 index, u8 *virt_addr)
{
    return ioread8(&(virt_addr[index]));
}

static inline int ecoreex_setup_data_access(struct ecoreex_access_s *e_access, struct gpio_s *gpio)
{
    if (!e_access || !gpio) return -1;
    if (e_access->ecoreex_data_read) gpio->data_read = e_access->ecoreex_data_read;
    if (e_access->ecoreex_data_write) gpio->data_write = e_access->ecoreex_data_write;
    if (e_access->ecoreex_ddr_read) gpio->ddr_read = e_access->ecoreex_ddr_read;
    if (e_access->ecoreex_ddr_write) gpio->ddr_write = e_access->ecoreex_ddr_write;
    if (e_access->ecoreex_index_read) gpio->index_read = e_access->ecoreex_index_read;
    if (e_access->ecoreex_index_write) gpio->index_write = e_access->ecoreex_index_write;
    if (!e_access->ecoreex_key_read) e_access->ecoreex_key_read = ecoreex_default_key_read;
    
    return 0;
}

#endif /*ECOREEX_H_*/
