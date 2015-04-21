#ifndef PWM_H_
#define PWM_H_
#include <linux/autoconf.h>
#include <linux/device.h>


#ifdef CONFIG_PWMCLASS

//SUBCLASS assignment currently unused and arbitrary
#define PWMD_SUBCLASS 77 

typedef u32 pwm_data;

/**********************
 * pwm class structure
 */
typedef struct pwm_s{
	const char *name;
	int subclass;
	void *widthus;
	void *periodus;
	void *control;
	pwm_data widthus_shadow;
	pwm_data periodus_shadow;
	pwm_data (*widthus_read)(struct pwm_s *pwm);//width us
	int (*widthus_write)(struct pwm_s *pwm,pwm_data data);
	pwm_data (*periodus_read)(struct pwm_s *pwm);	//period us
	int (*periodus_write)(struct pwm_s *pwm,pwm_data data);
	pwm_data (*invert_read)(struct pwm_s *pwm);	//pulse inversion
	int (*invert_write)(struct pwm_s *pwm,pwm_data data);
	pwm_data (*read_pwmclock)(struct pwm_s *pwm);
	int (*write_pwmclock)(struct pwm_s *pwm,pwm_data data);
}pwm_t;


struct class *pwm_declare(void);
struct device *pwm_register_device(pwm_t *pwm);

/***************************************************************************
 * typical low level methods
 */
int  pwm_widthus_write8(pwm_t *pwm, pwm_data data);
int  pwm_widthus_write16(pwm_t *pwm, pwm_data data);
int  pwm_periodus_write8(pwm_t *pwm, pwm_data data);
int  pwm_periodus_write16(pwm_t *pwm, pwm_data data);
int  pwm_empty_write(pwm_t *pwm, pwm_data data);
pwm_data pwm_widthus_read8(pwm_t *pwm);
pwm_data pwm_widthus_read16(pwm_t *pwm);
pwm_data pwm_periodus_read8(pwm_t *pwm);
pwm_data pwm_periodus_read16(pwm_t *pwm);
pwm_data pwm_ff_read(pwm_t *pwm);
pwm_data pwm_zero_read(pwm_t *pwm);
pwm_data pwm_widthusshadow_read(pwm_t *pwm);

/***************************************************************************
 * atomic method wrappers
 * these should be used for all method calls to maintain sychronization across the
 * various interfaces
 */
int atomic_pwm_widthus_write(pwm_t *pwm,pwm_data data);
int atomic_pwm_periodus_write(pwm_t *pwm,pwm_data data);
int atomic_pwm_invert_write(pwm_t *pwm,pwm_data data);
pwm_data atomic_pwm_widthus_read(pwm_t *pwm);
pwm_data atomic_pwm_periodus_read(pwm_t *pwm);
pwm_data atomic_pwm_invert_read(pwm_t *pwm);

#endif //CONFIG_PWMCLASS

#endif /*PWM_H_*/
