/**********************************************************
*
*软定时器
*
*
*
*
**********************************************************/
#include "SoftTimer.h"


uint16_t g_uSoftTimerTick=0;   // 定时器变量
uint8_t  g_uOverflow=0;        // 定时器溢出标记
/******************************************************************************
* 名    称：SoftTimerClock()
* 功    能：设定时器变量累计
* 入口参数：
*           无
* 出口参数：无
* 范    例:
* 说    明：需要在定时器中运行。
******************************************************************************/
void SoftTimerClock(void)
{
    g_uSoftTimerTick++;

    // 计数器小于50认为还是溢出状态，避免整个系统大循环时间比较长时不能清除单个定时器的溢出状态
    // 定时中长度不能大于65535-50，建议小于65000；
    if(g_uSoftTimerTick>50)
    {
        g_uOverflow = 0;
    }
    else
    {
        g_uOverflow = 1;  // 计数器溢出
    }
}
/******************************************************************************
* 名    称：SoftTimerSet()
* 功    能：设置定时时长。。
* 入口参数：
*           *pt     定时变量
            it      定时变量
            start   设置后状态，1：启动；0：不启动
* 出口参数：无
* 范    例:
******************************************************************************/
void SoftTimerSet(SOFT_TIMER_T *pt,uint16_t it,uint8_t start)
{
    pt->interval = it;
    pt->record   = (g_uSoftTimerTick + it)&0xffff;
    if(pt->record<g_uSoftTimerTick) // 数据溢出
    {
        pt->over = 1;
    }
    else
    {
        pt->over = 0;
    }
    pt->start    = start;
}

/******************************************************************************
* 名    称：soft_timer_expired()
* 功    能：判断计时。
* 入口参数：
*           *pt     定时变量

* 出口参数：定时到返回1，未到定时返回0
* 范    例:
******************************************************************************/
uint8_t SoftTimerExpired(SOFT_TIMER_T *pt)
{
    if(!pt->over)
    {
        return pt->start && (g_uSoftTimerTick >= pt->record);
    }
    else
    {
        /*if(g_uSoftTimerTick <= pt->record)  // 系统计数器也溢出后溢出标记清除
        {
            pt->over = 0;
        }*/
        if(g_uOverflow)
        {
            pt->over = 0;
        }
        return 0;
    }
}

/******************************************************************************
* 名    称：soft_timer_restart()
* 功    能：开始计时
* 入口参数：
*           *pt     定时变量

* 出口参数：无
* 范    例:
******************************************************************************/
void SoftTimerRestart(SOFT_TIMER_T *pt)
{
    //pt->interval = it;
    pt->record = (g_uSoftTimerTick + pt->interval)&0xffff;
    if(pt->record<g_uSoftTimerTick) // 数据溢出
    {
        pt->over = 1;
    }
    else
    {
        pt->over = 0;
    }
    pt->start  = 1;
}
/******************************************************************************
* 名    称：soft_timer_reset()
* 功    能：重新设置定时时长。。
* 入口参数：
*           *pt     定时变量

* 出口参数：无
* 范    例:
******************************************************************************/
void SoftTimerReset(SOFT_TIMER_T *pt)
{
    //pt->interval = it;
    pt->record = (g_uSoftTimerTick + pt->interval)&0xffff;
    if(pt->record<g_uSoftTimerTick) // 数据溢出
    {
        pt->over = 1;
    }
    else
    {
        pt->over = 0;
    }
}
/******************************************************************************
* 名    称：soft_timer_stop()
* 功    能：开始计时
* 入口参数：
*           *pt     定时变量

* 出口参数：无
* 范    例:
******************************************************************************/
void SoftTimerStop(SOFT_TIMER_T *pt)
{
    //pt->interval = it;
    //pt->record = g_uSoftTimerTick + pt->interval;
    pt->start  = 0;
}

/******************************************************************************
* 名    称：soft_timer_status()
* 功    能：判断计时。
* 入口参数：
*           *pt     定时变量

* 出口参数：定时停止返回0，定时启动返回1
* 范    例:
******************************************************************************/
uint8_t SoftTimerStatus(SOFT_TIMER_T *pt)
{
    return (pt->start);
}
