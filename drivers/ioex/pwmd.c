/**
 * PWMD's 
 * pwmd's modulate an external frequency, 
 * this frequency is passed in as method, which is used to calcuate the widthus
 * when written or read.
*/

#include <linux/class/pwm.h>
#include <asm/io.h>

#define PWMD_PWM_SUBCLASS 93

#define REGMAX 255
#define PERIODMAX ((0xffffffff)/(REGMAX))

//internal errors
#define SYSCLOCK_UNSUPPORTED	-2

/**
 * methods using a shadow register to set the periodus.
 */
static pwm_data pwmd_clock_read(struct pwm_s *pwm){
	return pwm->periodus_shadow;
}

static int pwmd_clock_write(struct pwm_s *pwm,pwm_data clock){
	if(!clock)clock=1;
	if(clock>PERIODMAX)clock=PERIODMAX;//to prevent overflows
	pwm->periodus_shadow = clock;
	return 0;
}

static inline int us2reg(__u32 *value,__u32 periodus_in){
	if(periodus_in)*value/=periodus_in;
	//if the input period is zero written value has no effect and will be interpreted as zero in any case.
	if(*value>REGMAX)*value=REGMAX;
	return 0;
}

static inline int reg2us(__u32 *value,__u32 periodus_in){
	*value*=periodus_in;
	return 0;		
}

static int pwmd_widthus_write(struct pwm_s *pwm, pwm_data width){
	pwm->widthus_shadow = width;
	if(us2reg(&width,pwm->read_pwmclock(pwm))==SYSCLOCK_UNSUPPORTED)
		{printk("sysclock unsupported\n");return SYSCLOCK_UNSUPPORTED;}
	
	iowrite16((u16)width,pwm->widthus);
	return 0;
}

static pwm_data pwmd_widthus_read(struct pwm_s *pwm){
	__u32 width = pwm->widthus_shadow;
	
	if(reg2us(&width,pwm->read_pwmclock(pwm))==SYSCLOCK_UNSUPPORTED)
		{printk("sysclock unsupported\n");return SYSCLOCK_UNSUPPORTED;}
		
	return (width);
}

static pwm_data pwmd_periodus_read(struct pwm_s *pwm){
	__u32 period = 0xff;
	
	if(reg2us(&period,pwm->read_pwmclock(pwm))==SYSCLOCK_UNSUPPORTED)
		{printk("sysclock unsupported\n");return SYSCLOCK_UNSUPPORTED;}
		
	return (period); 
}

static int pwmd_periodus_write(struct pwm_s *pwm, pwm_data periodus){
	pwmd_clock_write(pwm,periodus/REGMAX);
	return 0;
}

struct device *pwmd_device_create(
 void *widthus,
 const char *name,
 pwm_data (*read_pwmclock)(pwm_t *pwm)){
	
	pwm_t *pwm = kmalloc(sizeof(pwm_t),GFP_KERNEL);
	memset(pwm,0,sizeof(pwm_t));
	pwm->name = name;
	pwm->subclass = PWMD_PWM_SUBCLASS;

	pwm->widthus = widthus;
	
	pwm->widthus_write = pwmd_widthus_write;
    pwm->widthus_read = pwmd_widthus_read;
    pwm->periodus_read = pwmd_periodus_read;
    
	if(pwm->read_pwmclock)
		pwm->periodus_write = pwm_empty_write;
	else{
		pwm->read_pwmclock = pwmd_clock_read;
    	pwm->periodus_write = pwmd_periodus_write;
    	pwmd_clock_write(pwm,1);
	}
	printk("registering pwmd device: %s\n",name);

    return pwm_register_device(pwm);
}


