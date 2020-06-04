#include "RealTime.h"
//#include "IIC.h"
#include "IIC_polling.h"
#include "GlobalVar.h"
#include "InRtc.h"

#define REAL_TIME_ADDR             (0x68)//(0xD0)  瀹為檯鍦板潃涓?xD0 搴曞眰椹卞姩浼氬彸绉讳竴浣?
#define REAL_TIME__RD_CMD_BIT      (0x01)

SYSTEMTIME sDateTime;
time_t g_TimeStamp = 0;         //Cplus系统时间戳

void RealTimeInit(void)
{
    if(!g_TimeStamp)
    {
        InitInRtc();
    }
}
time_t RealTimeGetTick(void)
{
    return g_TimeStamp;
}

SYSTEMTIME *RealTimeGet(void)
{
    
    struct tm *time_temp = 0;
    g_TimeStamp = GetInRtcCount();

    time_temp = localtime(&g_TimeStamp) ;

    sDateTime.Year = time_temp->tm_year-100;
    sDateTime.Month = time_temp->tm_mon+1;
    sDateTime.Date = time_temp->tm_mday;
    sDateTime.Week = time_temp->tm_wday;
    sDateTime.Hour = time_temp->tm_hour;
    sDateTime.Minute = time_temp->tm_min;
    sDateTime.Second = time_temp->tm_sec;

    return &sDateTime;
}
void RealTimeSet(SYSTEMTIME *sTime)
{
   
    struct tm time = {0};       //Cplus系统时间

    time.tm_sec = sTime->Second;
    time.tm_min = sTime->Minute;
    time.tm_hour = sTime->Hour;
    time.tm_mday = sTime->Date;
    time.tm_wday = sTime->Week;
    time.tm_mon = sTime->Month-1;
    time.tm_year = sTime->Year+100;//！！！特别注意：这里是从1900经过的年数，此处要与src计算差值可以忽略具体年份
    time.tm_isdst = 0;   //不实行夏令时
    g_TimeStamp = mktime(&time);
    //DEBUGOUT("g_TimeStamp:%d\r\n",g_TimeStamp);
    SetInRtcCount(g_TimeStamp);
	//msleep(1);

}
// 计时按秒累加
void RealTimeTicktock(void)
{
    g_TimeStamp++;
    SetInRtcCount(g_TimeStamp);
}
void RealTimeSetPlus(uint8_t act,uint8_t uYear,uint8_t uMonth,uint8_t uDate,uint8_t uHour,uint8_t uMinute,uint8_t uSecond)
{
    if(act&0x01)
    {
        sDateTime.Year   = uYear;
    }
    if(act&0x02)
    {
        sDateTime.Month  = uMonth;
    }
    if(act&0x04)
    {
        sDateTime.Date   = uDate;
    }
    if(act&0x08)
    {
        sDateTime.Hour   = uHour;
    }
    if(act&0x10)
    {
        sDateTime.Minute = uMinute;
    }
    if(act&0x20)
    {
        sDateTime.Second = uSecond;
    }
    RealTimeSet(&sDateTime);
}
