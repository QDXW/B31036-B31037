#ifndef __ADC_H__
#define __ADC_H__

#ifndef _STDINT
#include <stdint.h>
#endif

extern int ADC_Init(void);
extern void ADC_test(void);
extern void ADC_start(void);
extern int16_t ADC_read(void);

#endif
