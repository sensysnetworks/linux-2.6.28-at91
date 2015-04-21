#ifndef ATODSOM9260M_H_
#define ATODSOM9260M_H_

#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/class/gpio.h>
#include <linux/clk.h>
#include <mach/board.h>
#include <mach/at91sam9260.h>
#include <mach/gpio.h>

#define ADC_CR          0x00    //Control Register Offset
#define ADC_MR          0x04    //Mode Register Offset
#define ADC_CHER        0x10    //Channel Enable Register Offset
#define ADC_CHDR        0x14    //Channel Disable Register Offset
#define ADC_CHSR        0x18    //Channel Status Register Offset
#define ADC_SR          0x1C    //Status Register Offset
#define ADC_LCDR        0x20    //Last Converted Data Register Offset
#define ADC_IER         0x24    //Interrupt Enable Register Offset
#define ADC_IDR         0x28    //Interrupt Disable Register Offset
#define ADC_IMR         0x2C    //Interrupt Mask Register Offset
#define ADC_CDR0        0x30    //Channel Data Register 0 Offset
#define ADC_CDR1        0x34    //Channel Data Register 1 Offset
#define ADC_CDR2        0x38    //Channel Data Register 2 Offset
#define ADC_CDR3        0x3C    //Channel Data Register 3 Offset

/* Define some of the values we will want in the registers
 * This can be changed to reflect the needs of the driver
 */

#define TRGEN           0x00    //Trigger Enable
#define TRGSEL          0x00    //Trigger Select
#define LOWRES          0x00    //Resolution
#define SLEEP_MODE      0x00    //Sleep Mode
#define PRESCAL         0x22   //Prescaler Rate Selection (22 --> ~100 samples/sec)
#define STARTUP         0x02    //Start Up Time
#define SHTIM           0x00    //Sample and Hold Time


int som9260m_atod_init(void);
int som9260m_atod_switch(int channel);
int som9260m_atod_read_current(void);
struct device *som9260m_atod_class_create(const char *name);

#endif /*ATODSOM9260M_H_*/
