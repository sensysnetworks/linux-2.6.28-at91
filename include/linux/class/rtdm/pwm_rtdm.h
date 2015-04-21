#ifndef PWM_RTDM_H_
#define PWM_RTDM_H_

#include <linux/ioctl.h>

//arbitrary assignment, come back to this later
#define RTDM_CLASS_PWM 0x90 

#ifdef __KERNEL__	
int rt_pwm_device_create(struct pwm_s *pwm);
#endif//__KERNEL__

#define PERIODUSREAD		_IOR(RTDM_CLASS_PWM,0,char)
#define PERIODUSWRITE 		_IOW(RTDM_CLASS_PWM,0,char)
#define WIDTHUSREAD			_IOR(RTDM_CLASS_PWM,1,char)
#define WIDTHUSWRITE 		_IOW(RTDM_CLASS_PWM,1,char)

#endif //PWM_RTDM_H_
