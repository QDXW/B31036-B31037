#ifndef __UART0PROCESS_H__
#define __UART0PROCESS_H__

#include <stdint.h>
#include "ucos_ii.h"


#define TASKUART0STKSIZE   128

extern void TaskUart0Process(void *p);
//extern void PrintThisInfo(void);
extern uint8_t Rs485DebugRead(void);
extern void PrintThisInfo(void);

extern OS_STK TaskUart0ProcesStk[TASKUART0STKSIZE];	      //定义任务堆栈大小

#endif
