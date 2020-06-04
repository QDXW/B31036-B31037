#ifndef SLAVEUNIT_H_
#define SLAVEUNIT_H_


#include "ucos_ii.h"

#ifndef _STDINT
#include <stdint.h>
#endif

#define TaskSouthInquireStkSize   256   // 查询任务堆栈大小
#define TaskSouthWriteStkSize     256   // 写寄存器任务堆栈大小

#define SOUTH_CMD_YK     (1)   // 南向写命令：遥控
#define SOUTH_CMD_SD     (2)   // 南向写命令：设点
#define SOUTH_CMD_SYNC   (3)   // 南向写命令：同步时间
#define SOUTH_CMD_READSD (4)   // 南向读命令：读设点


#define BAUDRATE_2400     1
#define BAUDRATE_4800     2
#define BAUDRATE_9600     3
#define BAUDRATE_19200    4
#define BAUDRATE_38400    5
#define BAUDRATE_115200   6




//extern DT1000UPDATA_DATA_T g_DT1000DataLen;
//extern static uint8_t  s_uSouthReadSd=SOUTH_CMD_READSD; //南向读设点
extern OS_STK TaskSouthInquireStk[TaskSouthInquireStkSize];	  // 定义任务堆栈大小
extern OS_STK TaskSouthWriteStk[TaskSouthWriteStkSize];	      // 定义任务堆栈大小

extern void TaskSouthWrite(void *p);
extern void TaskSouthInquire(void *p);

extern void AlarmAck(uint8_t uLen,const uint8_t *puData);
extern uint8_t AlarmReport(uint8_t addr,uint8_t err_code,uint16_t reg_addr,uint8_t offset,uint8_t event,uint8_t imd);
extern int8_t CheckDevState(void);
#endif
