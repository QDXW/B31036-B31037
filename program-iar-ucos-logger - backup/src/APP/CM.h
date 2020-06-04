#ifndef __CM_H_
#define __CM_H_

#include "ucos_ii.h"

#define TaskModemProcessStkSize   256   // 查询任务堆栈大小
extern OS_STK TaskModemProcessStk[TaskModemProcessStkSize];	      // 定义任务堆栈大小


extern void TaskModemProcess(void *p);

#endif
