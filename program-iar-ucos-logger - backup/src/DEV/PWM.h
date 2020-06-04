#ifndef __PWM_H_
#define __PWM_H_

#include "chip.h"

#define PWM00    0x01
#define PWM01    0x02
#define PWM02    0x06
#define PWM03    0x08

#define PWM10    0x10
#define PWM11    0x20
#define PWM12    0x40
#define PWM13    0x80

extern uint8_t PWM_s_duty;
extern uint8_t PWM_s_quick;

extern void PWM_Init(uint8_t port,uint8_t pwm_duty);
extern void PWM_Deinit(void);
extern void PWM_SetDuty(uint8_t out,uint16_t duty);
extern void PWM_SetFreq(uint16_t f);

#endif
