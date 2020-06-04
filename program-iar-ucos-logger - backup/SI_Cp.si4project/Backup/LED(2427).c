#include "LED.h"
#include "GPIO.h"
#include "GlobalVar.h"
#include "RealTime.h"
#include "WatchDog.h"

//================================================================
#define LED_ON            1
#define LED_OF            2
#define LED_TWINKLE_SLOW  3     //COM1 灯，慢闪
#define LED_TWINKLE_FAST  4     //COM1 灯，快闪
#define LED_TWINKLE_THREE 5     //3S间隔
//================================================================
#if(Logger_CP==LOGGER_TYPE)

#define LED1_IO_CONFIG()  GPIO_SetDir(2,20,Output)   // LED1
#define LED2_IO_CONFIG()  GPIO_SetDir(0,12,Output)   // LED2
#define LED3_IO_CONFIG()  GPIO_SetDir(1,11,Output)   // LED3
#define LED4_IO_CONFIG()  GPIO_SetDir(0,11,Output)   // LED4
#define LED5_IO_CONFIG()  GPIO_SetDir(1,29,Output)   // LED5
#define LED6_IO_CONFIG()  GPIO_SetDir(0,22,Output)   // LED6

#elif(Logger_C==LOGGER_TYPE)
#endif
//================================================================
typedef struct
{
    uint8_t uLed1;
    uint8_t uLed2;
    uint8_t uLinkBlue;
    uint8_t uLinkRed;
    uint8_t uSouthBlue;
    uint8_t uSouthRed;
}LED_STATUS_T;

//================================================================
static uint32_t g_uClockOld;   // 用于时钟计时
//================================================================
void LedShow(LED_STATUS_T sLed);
//================================================================
void LedInit(void)
{
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, 0x91);  // LED4
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, 0x91);  // LED2

    LED1_IO_CONFIG();   // LED1  POWER
    LED2_IO_CONFIG();   // LED2  LINK
    LED3_IO_CONFIG();   // LED3  MODEN
    LED4_IO_CONFIG();   // LED4  GPS
    LED5_IO_CONFIG();   // LED5  COM
    LED6_IO_CONFIG();   // LED6  ALARM
}

//=================================================================
/******************************************************************************
* 名    称：LedCtrl()
* 功    能：解析控制LED灯状态。
* 入口参数：
            无
*
* 出口参数：无
* 范    例:
******************************************************************************/
void LedCtrl(void)
{
    uint8_t i,k;
    LED_STATUS_T sLedCtrl={0,0,0,0,0,0};

    sLedCtrl.uLed2 = LED_TWINKLE_SLOW;
    //------------------------------------
    // Link灯
    if(NORTH_OK==g_LoggerRun.north_status)
    {
        sLedCtrl.uLinkRed = LED_OF;

        if(g_LoggerRun.uCSQ <= 12)  // 信号弱
        {
            sLedCtrl.uLinkBlue = LED_TWINKLE_SLOW;
        }
        else
        {
            sLedCtrl.uLinkBlue = LED_ON;
        }
    }
    else
    {
        sLedCtrl.uLinkBlue = LED_OF;
        sLedCtrl.uLinkRed  = LED_ON;
    }

    //------------------------------------
    // 南向设备灯
    for(i=0;i<MAX_device;i++)
    {
        if(g_DeviceSouth.device_inf[i].addr)
        {
            k++;
        }
    }
    if(RUNNING_WORK_READ==g_LoggerRun.run_status && 0!=k)
    {
        if(g_LoggerRun.err_lost)
        {
            sLedCtrl.uSouthBlue = LED_OF;
            sLedCtrl.uSouthRed = LED_ON;
        }
        else
        {
            sLedCtrl.uSouthRed = LED_OF;
            sLedCtrl.uSouthBlue = LED_TWINKLE_SLOW;
        }
    }
    else
    {
        sLedCtrl.uSouthRed = LED_OF;
        sLedCtrl.uSouthBlue = LED_ON;
    }
    LedShow(sLedCtrl);
}
//=================================================================
/******************************************************************************
* 名    称：LED_Show()
* 功    能：输出LED灯控制。
* 入口参数：
            无
*
* 出口参数：无
* 范    例:
******************************************************************************/
void LedShow(const LED_STATUS_T sLed)
{
    uint32_t uClockNew;
    uint32_t uClockGap;

    uint8_t  uSlowShow=0;

    //=================================================================================
    // 实时时钟处理
    uClockNew = OSTimeGet();  //

    uClockGap = (uClockNew>=g_uClockOld)?(uClockNew-g_uClockOld):(uClockNew+0xFFFFFFFF-g_uClockOld);  // 计时可能溢出加以判断

    if(uClockGap>=100)  // 10mS累加一次，1S
    {
        RealTimeTicktock();
        g_uClockOld += 100;
        uSlowShow = 1;
    }
    //=================================================================================
    //OSTimeDlyHMSM(0,0,1,0);

    //------------------------------------------------------------
    /*switch(sLed.uLed1)
    {
    case LED_ON:
        LedCtrl1(ON);
        break;
    case LED_OF:
        LedCtrl1(OF);
        break;
    case LED_TWINKLE_SLOW:
        if(uSlowShow)
        {LedTwinkle1();}
        break;
    case LED_TWINKLE_FAST:
        //if(uFastShow)
        {LedTwinkle1();}
        break;
    case LED_TWINKLE_THREE:
        //if(uThreeShow)
        {LedTwinkle1();}
        break;
    default:
        break;
    }*/
    //------------------------------------------------------------
    switch(sLed.uLed2)
    {
    case LED_ON:
        LedCtrl2(ON);
        break;
    case LED_OF:
        LedCtrl2(OF);
        break;
    case LED_TWINKLE_SLOW:
        if(uSlowShow)
        {LedTwinkle2();}
        break;
    case LED_TWINKLE_FAST:
        //if(uFastShow)
        {LedTwinkle2();}
        break;
    case LED_TWINKLE_THREE:
        //if(uThreeShow)
        {LedTwinkle2();}
        break;
    default:
        break;
    }
    //------------------------------------------------------------
    switch(sLed.uLinkBlue)
    {
    case LED_ON:
        LedCtrl3(ON);
        break;
    case LED_OF:
        LedCtrl3(OF);
        break;
    case LED_TWINKLE_SLOW:
        if(uSlowShow)
        {LedTwinkle3();}
        break;
    case LED_TWINKLE_FAST:
        //if(uFastShow)
        {LedTwinkle3();}
        break;
    case LED_TWINKLE_THREE:
        //if(uThreeShow)
        {LedTwinkle3();}
        break;
    default:
        break;
    }
    //------------------------------------------------------------
    switch(sLed.uLinkRed)
    {
    case LED_ON:
        LedCtrl4(ON);
        break;
    case LED_OF:
        LedCtrl4(OF);
        break;
    case LED_TWINKLE_SLOW:
        if(uSlowShow)
        {LedTwinkle4();}
        break;
    case LED_TWINKLE_FAST:
        //if(uFastShow)
        {LedTwinkle4();}
        break;
    case LED_TWINKLE_THREE:
        //if(uThreeShow)
        {LedTwinkle4();}
        break;
    default:
        break;
    }
    //------------------------------------------------------------
    switch(sLed.uSouthBlue)
    {
    case LED_ON:
        LedCtrl5(ON);
        break;
    case LED_OF:
        LedCtrl5(OF);
        break;
    case LED_TWINKLE_SLOW:
        if(uSlowShow)
        {LedTwinkle5();}
        break;
    case LED_TWINKLE_FAST:
        //if(uFastShow)
        {LedTwinkle5();}
        break;
    case LED_TWINKLE_THREE:
        //if(uThreeShow)
        {LedTwinkle5();}
        break;
    default:
        break;
    }
    //------------------------------------------------------------
    switch(sLed.uSouthRed)
    {
    case LED_ON:
        LedCtrl6(ON);
        break;
    case LED_OF:
        LedCtrl6(OF);
        break;
    case LED_TWINKLE_SLOW:
        if(uSlowShow)
        {LedTwinkle6();}
        break;
    case LED_TWINKLE_FAST:
        //if(uFastShow)
        {LedTwinkle6();}
        break;
    case LED_TWINKLE_THREE:
        //if(uThreeShow)
        {LedTwinkle6();}
        break;
    default:
        break;
    }
}
/****************************************************************************
* 名    称：TaskLedCtrl()
* 功    能：LED控制任务
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
OS_STK TaskLedCtrlStk[TaskLedCtrlStkSize];	      // 定义任务堆栈大小
void TaskLedCtrl(void *p)
{
    p = p;

    LedInit();
    g_uClockOld = OSTimeGet();

    while(1)
    {
        //------------------------------------
        LedCtrl();
        OSTimeDly(10);  // 100mS
        FEED_DOG();     // 喂看门狗

        printf("Oldtime = %d   g_South_Action_Newtime = %d",OSTimeGet(),g_South_Action_Newtime);
		if(abs(OSTimeGet() - g_South_Action_Newtime) > 90000 && (RUNNING_EMPTY != g_LoggerRun.run_status) )
		{
			Reboot();
		}
    }
}


