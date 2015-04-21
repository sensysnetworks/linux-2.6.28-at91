#ifndef PWMD_H_
#define PWMD_H_

struct device *pwmd_device_create(void *widthus,const char *name,pwm_data (*read_pwmclock)(pwm_t *pwm));

#endif /*PWMD_H_*/
