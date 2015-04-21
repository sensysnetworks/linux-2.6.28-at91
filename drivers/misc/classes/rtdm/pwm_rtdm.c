/**
 * A rtdm interface for pwm classes
 */

#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/class/pwm.h>
#include <linux/class/pwm.h>
#include <linux/class/rtdm/pwm_rtdm.h>
#include <rtdm/rtdm_driver.h>

typedef struct rtpwm_device_s {
struct rtdm_device rtd;
pwm_t *pwm;//pointer to parent pwm structure.
}rtpwm_device_t; 

static int rt_pwm_open(struct rtdm_dev_context *context,
                  rtdm_user_info_t *user_info, int oflags){
    return 0;
}

static int rt_pwm_close(struct rtdm_dev_context *context,
                   rtdm_user_info_t *user_info){
return 0;
}

static int rt_pwm_ioctl(struct rtdm_dev_context *context,
                   rtdm_user_info_t *user_info, int request, void *umem){
    rtpwm_device_t *dev = container_of(context->device,rtpwm_device_t,rtd);
    pwm_t *pwm = dev->pwm;
       
    pwm_data kmem[1];
             
	switch(request){
	case PERIODUSREAD:
	if(pwm->periodus_read) kmem[0] = atomic_pwm_periodus_read(pwm);
	else return -2;
	return rtdm_safe_copy_to_user (user_info, umem, kmem, sizeof(pwm_data));		
	
	case PERIODUSWRITE:
	rtdm_safe_copy_from_user (user_info, kmem, umem, sizeof(pwm_data));	
	if(pwm->periodus_write) atomic_pwm_periodus_write(pwm,kmem[0]);
	else return -2;
	return 0;
	
	case WIDTHUSREAD:
	if(pwm->widthus_read)kmem[0] = atomic_pwm_widthus_read(pwm);
	else return -2;
	return rtdm_safe_copy_to_user (user_info, umem, kmem, sizeof(pwm_data));
	
	case WIDTHUSWRITE:
	rtdm_safe_copy_from_user (user_info, kmem, umem, sizeof(pwm_data));	
	if(pwm->widthus_write)atomic_pwm_widthus_write(pwm,kmem[0]);	
	else return -2;
	return 0;
	}             
                   	                 	
return -1;//no such op, need to find the right op code for this.
}


static const struct rtdm_device __initdata device_tmpl = {
    struct_version:     RTDM_DEVICE_STRUCT_VER,

    device_flags:       RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
    device_name:        "",
    open_rt:            rt_pwm_open,
    ops: {
        close_rt:      	rt_pwm_close,
	close_nrt:	rt_pwm_close,
        ioctl_rt:       rt_pwm_ioctl,
    },
    device_class:       RTDM_CLASS_PWM,
    driver_name:        "pwm_rtd",
    driver_version:     RTDM_DRIVER_VER(1, 0, 0),
    peripheral_name:    "pwm",
    provider_name:      "EMAC.Inc",
};


int rt_pwm_device_create(struct pwm_s *pwm){
		rtpwm_device_t *dev = kmalloc(sizeof(rtpwm_device_t),GFP_KERNEL);
		dev->pwm = pwm;
		memcpy(&dev->rtd, &device_tmpl, sizeof(struct rtdm_device));
		strncpy(dev->rtd.device_name, dev->pwm->name, RTDM_MAX_DEVNAME_LEN);
		dev->rtd.device_sub_class = dev->pwm->subclass;
		dev->rtd.proc_name = dev->pwm->name;
		if(rtdm_dev_register(&dev->rtd)){
			printk("couldn't register rtpwm device %s\n",dev->rtd.device_name);	
			kfree(dev);
			return -1;
		}
	return 	0;	
}


