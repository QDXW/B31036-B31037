#include "chip.h"
#include "ucos_ii.h"
#include "GlobalVar.h"
#include "KEY_SCAN.h"
#include "Usart.h"
#include "SPI.h"
#include "GPIO.h"
#include "InEEPROM.h"
#include "WatchDog.h"
#include "InTIMER.h"
#include "Update.h"
#include "CRC16.h"
#include "RealTime.h"
#include "DataFlash.h"
#include "Memory.h"
#include "DataTransferArea.h"
#include "SlaveUnit.h"
#include "Record.h"
#include "Uart0process.h"
#include "CM.h"
#include "IEC104.h"
#include "LED.h"
#include "log.h"

//============================================================================================
#define DEBUG_MODE   (1)   // 调试模式

//============================================================================================
void DataInit(void);
void DeviceInit(void);
uint8_t Crystal_oscillator_Init2(void);
// 南北方向数据转发
void NorthToSouth(void);
void EepUpdataMes(void);

//============================================================================================
#define SIDE_A_VTOR_OFFSET         (0x00006000)
#define SIDE_B_VTOR_OFFSET         (0x00028000)
void ReAllocateNVIC(uint32_t offset)
{
    /*__disable_irq();
    NVIC_SetVTOR(VTOR_OFFSET);
    __enable_irq();*/

    uint32_t* src,*dst;
    int32_t size;

    __disable_irq();
    // copy vector table
    src = (uint32_t*)offset;
    dst = (uint32_t*)0x10000000;  // RAM的起始地址
    size = 192;  // 一共48个中断入口

    while(size > 0)  // 拷贝中断向量到内存
    {
        *dst++ = *src++;
        size -= 4;
    }
     LPC_SYSCTL->SYSMEMREMAP = 0x1;    /*LPC_SYSCON remap to internal RAM */
    //__enable_irq();

}

//============================================================================================
int main(void)
{
    DeviceInit();   // 设备初始化
    PrintThisInfo();   // 打印基本信息
    OSInit();    // init uc/os-ii
    SysMesInit();   // 系统信号初始化

    OSTaskCreateExt (TaskModemProcess,                                      // 建立扩展任务(任务代码指针)
                        (void *)0,                                          // 传递参数指针
                        &TaskModemProcessStk[TaskModemProcessStkSize-1],    // 分配任务堆栈栈顶指针
                        3,                                                  // 分配任务优先级
                        3,                                                  // (未来的)优先级标识(与优先级相同)
                        &TaskModemProcessStk[0],                            // 分配任务堆栈栈底指针
                        TaskModemProcessStkSize,                            // 指定堆栈的容量(检验用)
                        (void *)0,                                          // 指向用户附加的数据域的指针
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // 建立任务设定选项

    OSTaskCreateExt (TaskIec104Process,                                     // 建立扩展任务(任务代码指针)
                        (void *)0,                                          // 传递参数指针
                        &TaskIec104ProcessStk[TaskIec104ProcessStkSize-1],  // 分配任务堆栈栈顶指针
                        5,                                                  // 分配任务优先级
                        5,                                                  // (未来的)优先级标识(与优先级相同)
                        &TaskIec104ProcessStk[0],                           // 分配任务堆栈栈底指针
                        TaskIec104ProcessStkSize,                           // 指定堆栈的容量(检验用)
                        (void *)0,                                          // 指向用户附加的数据域的指针
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // 建立任务设定选项

    OSTaskCreateExt (TaskSouthInquire,                                      // 建立扩展任务(任务代码指针)
                        (void *)0,                                          // 传递参数指针
                        &TaskSouthInquireStk[TaskSouthInquireStkSize-1],    // 分配任务堆栈栈顶指针
                        6,                                                  // 分配任务优先级
                        6,                                                  // (未来的)优先级标识(与优先级相同)
                        &TaskSouthInquireStk[0],                            // 分配任务堆栈栈底指针
                        TaskSouthInquireStkSize,                            // 指定堆栈的容量(检验用)
                        (void *)0,                                          // 指向用户附加的数据域的指针
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // 建立任务设定选项

    OSTaskCreateExt (TaskSouthWrite,                                        // 建立扩展任务(任务代码指针)
                        (void *)0,                                          // 传递参数指针
                        &TaskSouthWriteStk[TaskSouthWriteStkSize-1],        // 分配任务堆栈栈顶指针
                        7,                                                  // 分配任务优先级
                        7,                                                  // (未来的)优先级标识(与优先级相同)
                        &TaskSouthWriteStk[0],                              // 分配任务堆栈栈底指针
                        TaskSouthWriteStkSize,                              // 指定堆栈的容量(检验用)
                        (void *)0,                                          // 指向用户附加的数据域的指针
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // 建立任务设定选项

    OSTaskCreateExt (TaskUart0Process,                                      // 建立扩展任务(任务代码指针)
                        (void *)0,                                          // 传递参数指针
                        &TaskUart0ProcesStk[TASKUART0STKSIZE-1],            // 分配任务堆栈栈顶指针
                        8,                                                  // 分配任务优先级
                        8,                                                  // (未来的)优先级标识(与优先级相同)
                        &TaskUart0ProcesStk[0],                             // 分配任务堆栈栈底指针
                        TASKUART0STKSIZE,                                   // 指定堆栈的容量(检验用)
                        (void *)0,                                          // 指向用户附加的数据域的指针
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // 建立任务设定选项

    OSTaskCreateExt (TaskLedCtrl,                                           // 建立扩展任务(任务代码指针)
                        (void *)0,                                          // 传递参数指针
                        &TaskLedCtrlStk[TaskLedCtrlStkSize-1],              // 分配任务堆栈栈顶指针
                        9,                                                  // 分配任务优先级
                        9,                                                  // (未来的)优先级标识(与优先级相同)
                        &TaskLedCtrlStk[0],                                 // 分配任务堆栈栈底指针
                        TaskLedCtrlStkSize,                                 // 指定堆栈的容量(检验用)
                        (void *)0,                                          // 指向用户附加的数据域的指针
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // 建立任务设定选项
    OSStart();
}

/****************************************************************************
* 名    称：DeviceInit()
* 功    能：设备初始化。
* 入口参数：无
* 出口参数：无
* 范    例: 无
****************************************************************************/
void DeviceInit(void)
{
    __disable_irq();

    // 偏移中断向量表，复制到RAM
    if(GetVerS2()&0x0001)  // 小版本号最低位，1：程序在A面；0：程序在B面
    {
        ReAllocateNVIC(SIDE_A_VTOR_OFFSET);
    }
    else
    {
        ReAllocateNVIC(SIDE_B_VTOR_OFFSET);
    }

    Chip_GPIO_Init(LPC_GPIO);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);  // 必须，初始化外部晶振前运行，不然不能分配外部晶振引脚Chip_GPIO_Init(LPC_GPIO);

    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_RAM1);    // 使能SRAM1时钟
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USBRAM);  // 使能SRAM2时钟

    // 晶振初始化
    if(0==Crystal_oscillator_Init2())  // 外部晶振初始化失败
    {
        LED3(ON);
    }

    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock/100);   // SysTick interrupt @ 100 Hz = 10 msec

	//RealTimeInit(); 	// 时钟初始化，IIC初始化

    UartInit(uart0,115200,UART_PARITY_NONE);      // 串口0初始化 RS232
    //UartInit(uart1,9600,  UART_PARITY_NONE);      // 串口1初始化 Zigbee
    //UartInit(uart2,9600,  UART_PARITY_NONE);      // 串口2初始化 RS485
    UartInit(uart3,9600,  UART_PARITY_NONE);      // 串口3初始化 RS232
    UartInit(uart4,115200,UART_PARITY_NONE);      // 串口4初始化 RS232  模块

    #if(0==DEBUG_MODE)
    WatchDog_Init();// 初始化看门狗
    FEED_DOG();     // 喂狗
    #endif

    DataFlashInit();   // DataFlash初始化
    
    #if(0==DEBUG_MODE)
    FEED_DOG();     // 喂狗
    #endif

    #if(1==DEBUG_MODE)
    DEBUGOUT("--------------调试模式程序--------------\n");
    #endif

    DataFlash_Read_ID();   // 读取DataFlsh ID

    __enable_irq();    // 使能中断

    DataInit();        // 数据初始化
    RecordInit(0);     // 历史数据存储初始化

	if(NULL!=SOUTH_DATA_LOG)
    {
        SOUTH_DATA_LOG = WMemFree(SOUTH_DATA_LOG);
    }
    SaveLogInit(0);

    // 读取更新eeprom里的升级信息
    EepUpdataMes();
}
/****************************************************************************
* 名    称：DataInit()
* 功    能：数据初始化。
* 入口参数：无
* 出口参数：无
* 范    例: 无
****************************************************************************/
void DataInit(void)
{
    uint16_t uEepRead=0;
    uint16_t sucess=0;
    uint8_t  i=0;
    SYSTEMTIME *pTime;

    // 读取数采本机信息
    uEepRead = ReadEepData(EEP_LOGGER_INF);
    if(EEP_ERR==uEepRead) // 读取数据CRC校验错误
    {
        SetLoggerDefInfo();  // 设置为默认信息
        SaveEepData(EEP_LOGGER_INF);
        sucess |= 0x01;  // 数采信息读取成功
    }
    else
    {
        SaveEepData(EEP_LOGGER_INF);
        sucess |= 0x01;  // 数采信息读取成功
    }

    // 读设备南向信息 和 点表信息
    uEepRead = ReadEepData(EEP_DEVICE_SOUTH);
    if(EEP_ERR==uEepRead) // 读取数据CRC校验错误
    {
        AllReset(1);
    }
    else
    {
        sucess |= 0x02;  // 点表基础信息读取成功

        // 读取南向设备ESN号
        uEepRead = ReadEepData(EEP_DEVICE_ESN);
        uEepRead = ReadEepData(EEP_DEVICE_SOFT);
    }

    //-----------------------------------------------------------------------------
    // 读点表信息
    for(i=0;i<MAX_device;i++)//&&i<g_DeviceSouth.device_sum
    {
        if(0!=g_DeviceSouth.protocol[i].protocol_num && 0!=g_DeviceSouth.protocol[i].mess_point_sum && 0!=g_DeviceSouth.protocol[i].flash_addr)
        {
            g_pRegPoint[i] = (LOGGER_MODBUS_REG_T*)WMemMalloc(g_pRegPoint[i],g_DeviceSouth.protocol[i].mess_point_sum*sizeof(LOGGER_MODBUS_REG_T));   // 申请空间
            if(NULL!=g_pRegPoint[i]) // 申请空间成功
            {
                DataFlash_Read(g_DeviceSouth.protocol[i].flash_addr,(uint8_t*)&g_pRegPoint[i][0],g_DeviceSouth.protocol[i].mess_point_sum*sizeof(LOGGER_MODBUS_REG_T));
                sucess |= 0x04;   // 点表读取成功
            }
            else
            {
                DEBUGOUT("要点表空间失败-%d\n",i);
                sucess &= ~0x04;  // 点表读取失败
            }
        }
    }
    //-----------------------------------------------------------------------------
    //-----------------------------------------------------------------------------
    // 读告警信息
    ReadEepData(EEP_ALARM);

    // 读取预置点表信息点表编码
    //ReadEepData(EEP_TABLE_SEQ);
    //Init_DevicesTypes_Table();

    //-------------------------------------------------------------------
    if((0x07)==sucess && 0!=g_DeviceSouth.device_sum)
    {
        g_LoggerRun.run_status = RUNNING_WORK_READ;
    }
    else
    {
        g_LoggerRun.run_status = RUNNING_EMPTY;//RUNNING_WORK_READ;
    }
    //-----------------------------------------------------------------------------


    //===========================================================
    pTime = RealTimeGet();

//    if(!g_TimeStamp)
//    {
//        InitInRtc();
//    }

    if(pTime->Year>100 || pTime->Month>12 || pTime->Date>31 || pTime->Hour>23 || pTime->Minute>59 || pTime->Second>59)
    {
//        Date_Time.Year   = 0x00;
//        Date_Time.Month  = 0x00;
//        Date_Time.Date   = 0x00;
//        Date_Time.Week   = 0x00;
//        Date_Time.Hour   = 0x00;
//        Date_Time.Minute = 0x00;
//        Date_Time.Second = 0x00;
        //RealTime_Set();

        SetRecordAllow(0);  // 时间不对，不记录历史数据
    }
    else
    {
        SetRecordAllow(1);  // 时间对，记录历史数据
    }
}

//============================================================================
// 先初始化外部晶振，尝试设置外部晶振为PLL源，若锁定成功设置外部晶振为主时钟，若失败内部IRC为主时钟，关闭PLL
uint8_t Crystal_oscillator_Init(void)
{
	volatile int i;

    for (i = 0; i < 250; i++) {}
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // 外部晶振引脚
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // 外部晶振引脚
    for (i = 0; i < 250; i++) {}

	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);  // 主时钟晶振供电

    //Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_IRC_PD);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);  // 主时钟设置为内部IRC


	for (i = 0; i < 2500; i++) {} // Wait for at least 580uS for osc to stabilize

    Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);     // 关闭PLL


	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);  // 设置PLL源为主时钟晶振

    while (!(LPC_SYSCTL->SYSPLLCLKUEN & 0x01));     //等待更新完成

	/* Setup FLASH access */
    //MCU的主频设置为48MHz，FLASHTIM应该设置为0x02（3个系统等待周期）。 LPCOPEN实际设置为2个时钟等待周期，这个与UM规定相左。按照UM修改成0x02后，高温测试后重新上电/复位正常
    Chip_FMC_SetFLASHAccess(FLASHTIM_3CLK_CPU);
	//Chip_FMC_SetFLASHAccess(FLASHTIM_2CLK_CPU);

	// Power down PLL to change the PLL divider ratio
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	// Powerup system PLL
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	// Wait for PLL to lock
    i = 0;
	while (!Chip_Clock_IsSystemPLLLocked())
    {
        i++;
        if(i>100)
        {
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);  // 关闭PLL
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSOSC_PD);  // 主时钟晶振断电
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, 0x90);
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, 0x90);

            for (i = 0; i < 2500; i++) {}

            Chip_Clock_SetSysClockDiv(1);
            return 0;
        }
    }

    Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	// Set system clock divider to 1
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 48MHz
	   for the main clock and 48MHz for the system clock */
	//Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLIN);

    for (i = 0; i < 250; i++) {}
    for (i = 0; i < 250; i++) {}

    return 1;
}
//============================================================================
// 先初始化外部晶振，尝试设置外部晶振为PLL源，若锁定成功设置外部晶振为PLL源，PLL设置为主时钟，若失败内部IRC为主时钟，关闭PLL
uint8_t Crystal_oscillator_Init2(void)
{
	volatile int i;

    for (i = 0; i < 250; i++) {}
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // 外部晶振引脚
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // 外部晶振引脚
    for (i = 0; i < 250; i++) {}

	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);  // 主时钟晶振供电

    //Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_IRC_PD);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);  // 主时钟设置为内部IRC

	for (i = 0; i < 2500; i++) {} // Wait for at least 580uS for osc to stabilize

    Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);     // 关闭PLL

	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);  // 设置PLL源为主时钟晶振

    while (!(LPC_SYSCTL->SYSPLLCLKUEN & 0x01));     //等待更新完成

	/* Setup FLASH access */
    //MCU的主频设置为48MHz，FLASHTIM应该设置为0x02（3个系统等待周期）。 LPCOPEN实际设置为2个时钟等待周期，这个与UM规定相左。按照UM修改成0x02后，高温测试后重新上电/复位正常
    Chip_FMC_SetFLASHAccess(FLASHTIM_3CLK_CPU);
	//Chip_FMC_SetFLASHAccess(FLASHTIM_2CLK_CPU);

	// Power down PLL to change the PLL divider ratio
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	// Powerup system PLL
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	// Wait for PLL to lock
    i = 0;
	while (!Chip_Clock_IsSystemPLLLocked())
    {
        i++;
        if(i>100)
        {
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);  // 关闭PLL
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSOSC_PD);  // 主时钟晶振断电
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, 0x90);
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, 0x90);

            for (i = 0; i < 2500; i++) {}

            /*
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);  // 关闭PLL

            Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_IRC);  // 设置PLL源为内部RC
            while (!(LPC_SYSCTL->SYSPLLCLKUEN & 0x01));          // 等待更新完成

            //Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);
            Chip_Clock_SetupSystemPLL(3, 1);
            Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

            while (!Chip_Clock_IsSystemPLLLocked()){}


            // Set system clock divider to 1
            Chip_Clock_SetSysClockDiv(1);


            //Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
            */

            Chip_Clock_SetSysClockDiv(1);
            return 0;
        }
    }

	// Set system clock divider to 1
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 48MHz
	   for the main clock and 48MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

    for (i = 0; i < 250; i++) {}
    for (i = 0; i < 250; i++) {}

    return 1;
}
//============================================================================
// 设置内部晶振为PLL源，若锁定成功设置外部晶振为主时钟，若失败则死循环
#if 0
void Chip_SetupIrcClocking2(void)
{
    volatile int i;

	/* Turn on the IRC by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_IRC_PD);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);  // 主时钟设置为内部IRC

    for (i = 0; i < 250; i++) {}

	/* Select the PLL input in the IRC */
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_IRC);

	/* Setup FLASH access */
    //MCU的主频设置为48MHz，FLASHTIM应该设置为0x02（3个系统等待周期）。 LPCOPEN实际设置为2个时钟等待周期，这个与UM规定相左。按照UM修改成0x02后，高温测试后重新上电/复位正常
    Chip_FMC_SetFLASHAccess(FLASHTIM_3CLK_CPU);
	//Chip_FMC_SetFLASHAccess(FLASHTIM_2CLK_CPU);


	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Configure the PLL M and P dividers */
	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	/* Turn on the PLL by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsSystemPLLLocked()) {}

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

}
#endif
//-------------------------------------------------------------
// 南北方向数据转发
/*void NorthToSouth(void)
{
    #define COM_N         uart2
    #define COM_STATE_N   UART2_STATE
    #define COM_S         uart1
    #define COM_STATE_S   UART1_STATE
    uint16_t  read_count;
    uint8_t  inbuf[265]={0};

    // 收到北向通讯
    if(1==COM_STATE_N.Rec_OK)
    {
        //LED2_twinkle();  // LED3闪烁

        COM_STATE_N.Rec_OK = 0;

        read_count = UART_GetRxCount(COM_N);

        read_count = UARTn_Read(COM_N,(uint8_t*)&inbuf[0],read_count);

        UARTn_Send(COM_S,inbuf,read_count);   // 向485转发
        UART_ClearRxBuff(COM_N);              // 重新初始化缓存区，清空。
    }

    // 收到南向通讯
    if(1==COM_STATE_S.Rec_OK)
    {
        //LED3_twinkle();  // LED3闪烁

        COM_STATE_S.Rec_OK = 0;

        read_count = UART_GetRxCount(COM_S);

        read_count = UARTn_Read(COM_S,(uint8_t*)&inbuf[0],read_count);

        UARTn_Send(COM_N,inbuf,read_count);  // 向232转发

        UART_ClearRxBuff(COM_S);             // 重新初始化缓存区，清空。
    }
}*/
/******************************************************************************
* 名    称：EepUpdataMes()
* 功    能：EEPROM升级信息判断。
* 入口参数：
*           无      无

* 出口参数：
* 范    例:
******************************************************************************/
void EepUpdataMes(void)
{
    UPDATA_MARK_T sUpdata;

    uint16_t uSoftVer;

    uSoftVer = GetVerS2();//最低位1:在A面；0:在B面

    EepReadData(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sUpdata,sizeof(UPDATA_MARK_T),&sUpdata.CRC);// 升级信息读取
    if(0xAA!=sUpdata.sucess || ((uSoftVer&0x0001) && 0xAA!=sUpdata.side) || (0==(uSoftVer&0x0001) && 0xBB!=sUpdata.side))
    {
        sUpdata.sucess = 0xAA;

        if(uSoftVer&0x0001)  // 小版本号最低位，1：程序在A面；0：程序在B面
        {
            sUpdata.side = 0xAA;
        }
        else
        {
            sUpdata.side = 0xBB;
        }
        EepSavedata(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sUpdata,sizeof(UPDATA_MARK_T),&sUpdata.CRC);// 升级信息存储
    }
}

