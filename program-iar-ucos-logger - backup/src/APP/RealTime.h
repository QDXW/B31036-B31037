#ifndef __REALTIME_H_
#define __REALTIME_H_

#include <stdint.h>
#include "time.h"
/*--------------------------------------------------------------*/
/*                     定义时间类型                             */
/*--------------------------------------------------------------*/
typedef struct _SYSTEMTIME_
{
    unsigned char Second;
    unsigned char Minute;
    unsigned char Hour;
    unsigned char Week;
    unsigned char Date;
    unsigned char Month;
    unsigned char Year;
    //unsigned char DateString[9];
    //unsigned char TimeString[9];
}SYSTEMTIME;

extern SYSTEMTIME sDateTime;
//extern SYSTEMTIME Date_Time;


extern void RealTimeInit(void);
extern SYSTEMTIME *RealTimeGet(void);


extern void RealTimeSet(SYSTEMTIME *sTime);
extern void RealTimeSetPlus(uint8_t act,uint8_t uYear,uint8_t uMonth,uint8_t uDate,uint8_t uHour,uint8_t uMinute,uint8_t uSecond);
extern void RealTimeTicktock(void);
extern time_t RealTimeGetTick(void);

#endif
