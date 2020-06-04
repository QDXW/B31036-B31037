#ifndef __LED_H_
#define __LED_H_

#include  <stdint.h>
#include "ucos_ii.h"

#define LedCtrl1(s)       GPIO_SetBit(2,20,s)
#define LedCtrl2(s)       GPIO_SetBit(0,12,s)
#define LedCtrl3(s)       GPIO_SetBit(1,11,s)
#define LedCtrl4(s)       GPIO_SetBit(0,11,s)
#define LedCtrl5(s)       GPIO_SetBit(1,29,s)
#define LedCtrl6(s)       GPIO_SetBit(0,22,s)

#define LedTwinkle1()     GPIO_TwinkleBit(2,20)
#define LedTwinkle2()     GPIO_TwinkleBit(0,12)
#define LedTwinkle3()     GPIO_TwinkleBit(1,11)
#define LedTwinkle4()     GPIO_TwinkleBit(0,11)
#define LedTwinkle5()     GPIO_TwinkleBit(1,29)
#define LedTwinkle6()     GPIO_TwinkleBit(0,22)

#define TaskLedCtrlStkSize 64
extern OS_STK TaskLedCtrlStk[TaskLedCtrlStkSize];	      // 定义任务堆栈大小
extern void TaskLedCtrl(void *p);

extern void LED_Init(void);
extern void LED_Ctrl(void);
#endif
