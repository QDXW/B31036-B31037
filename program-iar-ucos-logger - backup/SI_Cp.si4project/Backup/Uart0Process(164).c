#include "Uart0process.h"


#include "stdio.h"
#include <stdarg.h>  

#include "DataFlash.h"
#include "GlobalVar.h"
#include "InEeprom.h"
#include "string.h"
#include "CRC16.h"
#include "WatchDog.h"
#include "Usart.h"
#include "tool.h"

#include "DataTransferArea.h"
#include "Memory.h"
#include "RealTime.h"

#include "ucos_ii.h"
#include "IEC104.h"

#include "ModbusMaster.h"

//=======================================================
#define UPDATE_UART_USE           uart3         // 使用的串口
#define CM_UART_USE               uart4         // 使用的串口
//=======================================================
#define JUDGE(x)  ((x)==0?'N':'Y')  // 判断数值是否为0，为0返回0，非0返回1
//=======================================================
//====================================================================
#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // 延时x秒
//====================================================================
//====================================================================
#define SIZE 128
static uint8_t g_TestData[SIZE];
//static uint8_t g_TestOut[4096];

static uint8_t g_uRecData[200];
static uint8_t *g_uUpdateBuffer=NULL;
//=======================================================

//=======================================================
OS_STK TaskUart0ProcesStk[TASKUART0STKSIZE]@0x20004400;	      //定义任务堆栈大小
//=======================================================
//=======================================================
void DealCmd(char *uRec,uint16_t uLen);
uint8_t UartUpdate(uint8_t *pUpdteBuffer);
//=======================================================
/*
#if(1==USE_485_DEBUG)
static uint8_t s_uRs485Debug=0;
void Rs485DebugMark(void)
{
    s_uRs485Debug = 1;
}
uint8_t Rs485DebugRead(void)
{
    return s_uRs485Debug;
}
void Rs485DebugClear(void)
{
    s_uRs485Debug = 0;
}

    #define WDEBUGOUT(...) \
    do{\
        if(s_uRs485Debug)\
        {\
            memset(g_uRecData,0,100);\
            sprintf((char*)g_uRecData,__VA_ARGS__);\
            UartWrite(UPDATE_UART_USE,g_uRecData,strlen((char*)g_uRecData));\
        }\
        printf(__VA_ARGS__);\
        while(UartTxLen(UPDATE_UART_USE)>0 && s_uRs485Debug);\
    }while(0)
#else
#define WDEBUGOUT(...) printf(__VA_ARGS__)
#endif
*/
        
#if(1==USE_485_DEBUG)
static uint8_t s_uRs485Debug=0;
void Rs485DebugMark(void)
{
    s_uRs485Debug = 1;
}
uint8_t Rs485DebugRead(void)
{
    return s_uRs485Debug;
}
void Rs485DebugClear(void)
{
    s_uRs485Debug = 0;
}

void WDEBUGOUT(const char *cmd, ...)
{
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数 
    
    if(s_uRs485Debug)
    {
        memset(g_uRecData,0,100);
        
        va_start(args,cmd); //使args指向可变参数的第一个参数  
        vsprintf((char*)g_uRecData,cmd,args);  //必须用vprintf等带V的  
        va_end(args);       //结束可变参数的获取
        UartWrite(UPDATE_UART_USE,g_uRecData,strlen((char*)g_uRecData));
    }
    
    va_start(args,cmd); //使args指向可变参数的第一个参数  
    vprintf(cmd,args);  //必须用vprintf等带V的  
    va_end(args);       //结束可变参数的获取
    
    while(UartTxLen(UPDATE_UART_USE)>0 && s_uRs485Debug);
}
#else
#define WDEBUGOUT(...) printf(__VA_ARGS__)
#endif
/****************************************************************************
* 名    称：PrintThisInfo()
* 功    能：打印基本信息。
* 入口参数：
*           无

* 出口参数：无
* 范    例: 无
****************************************************************************/
void PrintThisInfo(void)
{
    SYSTEMTIME *pGetTime;
    pGetTime = RealTimeGet();

    WDEBUGOUT("\n时间  :%d-%d-%d %d:%d:%d-%d\n",pGetTime->Year,pGetTime->Month,pGetTime->Date,pGetTime->Hour,pGetTime->Minute,pGetTime->Second,pGetTime->Week);
    WDEBUGOUT("版本  :%02X%02X%02X-%04X\n",GetVerType(),GetVerS1(),GetVerS2(),GetVerS3());
    WDEBUGOUT("ESN   :%-20.20s\n",g_LoggerInfo.esn);//WDEBUGOUT("ESN    :%-20.20s\r\n",g_LoggerInfo.esn);
    WDEBUGOUT("名称  :%-20.20s\n",g_LoggerInfo.name);
    WDEBUGOUT("型号  :%-20.20s\n",g_LoggerInfo.model);
    WDEBUGOUT("类型  :%-20.20s\n",g_LoggerInfo.type);
    WDEBUGOUT("IP    :%-0.32s\n",g_LoggerInfo.server_domain);
    WDEBUGOUT("端口  :%d\n",g_LoggerInfo.server_port);
    WDEBUGOUT("号码  :%-11.11s\n",g_LoggerInfo.phonenum);
    WDEBUGOUT("功率  :%d\n",g_LoggerRun.uCSQ);
    //WDEBUGOUT("点表号:%d\n",g_PerSetTableResq);
    WDEBUGOUT("间隔   :%ds\n",g_LoggerInfo.inquire_interval_time);

    if(GetVerS2()&0x01)  // 小版本号最低位，1：程序在A面；0：程序在B面
    {
        WDEBUGOUT("A面-");
    }
    else
    {
        WDEBUGOUT("B面-");
    }
    /*
    RUNNING_ENPTY          = 0,    // 新，未开站
    RUNNING_SEARCH_HW      = 1,    // 搜索华为设备
    RUNNING_SEARCH_END     = 2,    // 搜索华为设备结束
    RUNNING_INPUT_START    = 3,    // 启动导表C5
    RUNNING_INPUT_SOUTH    = 4,    // 导入下联设备信息C4 92
    RUNNING_INPUT_GOLB     = 5,    // 导入全局信息BB 88
    RUNNING_INPUT_TABLE    = 6,    // 导入点表BB 89
    RUNNING_INPUT_104      = 7,    // 导入设备104表信息BB 8A
    RUNNING_WORK_READ      = 10,   // 已经开站完成，正常工作
    */
    switch(g_LoggerRun.run_status)
    {
    case RUNNING_EMPTY:
        WDEBUGOUT("空数采\n");
        break;

    case RUNNING_SEARCH_HW:
        WDEBUGOUT("搜索设备\n");
        break;

    case RUNNING_SEARCH_END:
        WDEBUGOUT("搜索结束\n");
        break;

    case RUNNING_INPUT_START:
        WDEBUGOUT("启动导表C5\n");
        break;

    case RUNNING_INPUT_SOUTH:
        WDEBUGOUT("导设备信息C492\n");
        break;

    case RUNNING_INPUT_GOLB:
        WDEBUGOUT("导全局信息BB88\n");
        break;

    case RUNNING_INPUT_TABLE:
        WDEBUGOUT("导点表BB89\n");
        break;

    case RUNNING_INPUT_104:
        WDEBUGOUT("导地址分配BB8A\n");
        break;

    case RUNNING_WORK_READ:
        WDEBUGOUT("工作中\n");
        break;
    }

    WDEBUGOUT("北向:");
    switch(g_LoggerRun.north_status)
    {
    case NORTH_DISCON:
        WDEBUGOUT("断联\n");
        break;

    case NORTH_CMERR:
        WDEBUGOUT("模块未返回>\n");
        break;

    case NORTH_RDY:
        WDEBUGOUT("模块准备好\n");
        break;

    case NORTH_CONNECT:
        WDEBUGOUT("模块连接中\n");
        break;

    case NORTH_POWERDOWN:
        WDEBUGOUT("模块关机\n");
        break;

    case NORTH_OK:
        WDEBUGOUT("OK\n");
        break;

    default:
        WDEBUGOUT("其他\n");
        break;
    }
}
/****************************************************************************
* 名    称：TastUart0Process()
* 功    能：串口0处理任务
* 入口参数：
* 出口参数：无
* 范    例: 无
****************************************************************************/
void TaskUart0Process(void *p)
{
    p = p;

    uint16_t iResult;
    uint32_t uEnter485Time=0;
    uint32_t uNowTime=0;


    while(1)
    {

		iResult = UartRxLen(uart0);   // 读取串口缓冲区的字节数量
        if(iResult<=0) // 没有接收到数据
        {
            msleep(20);
        }
        else
        {
            iResult = UartRead(uart0, g_uRecData, 64, 2);
           /* DEBUGOUT("iResult:%d\r\n",iResult);
            DEBUGOUT("g_uRecData");
            for(uint8_t i=0;i<iResult;i++)
            {
               DEBUGOUT("%02x ",g_uRecData[i]);
            }
            DEBUGOUT("\r\n");*/
            
            if(iResult>0)
            {
                
              DealCmd((char *)g_uRecData,iResult);	
            }

        }


        #if(1==USE_485_DEBUG)
        if(Rs485DebugRead())
        {
            uNowTime = OSTimeGet();
   
            if(TIMEOUT==TimeOut(uEnter485Time,uNowTime,180000))  // 进入485调试后，3分钟没有发送指令
            {
                Reboot();
            }
        }

        if(0==g_LoggerRun.update)
        {
            iResult = UartRxLen(UPDATE_UART_USE);   // 读取串口缓冲区的字节数量

            if(iResult>1)
            {
                UartShowBytes(UPDATE_UART_USE, g_uRecData, 2);
                if('A'==g_uRecData[0] && 'A'==g_uRecData[1])
                {
                    iResult = UartRead(UPDATE_UART_USE, g_uRecData, 64, 2);
                    if(iResult<4)
                    {
                        UartClearRecBuffer(UPDATE_UART_USE);  // 清空缓存
                    }
                    else
                    {
                        Rs485DebugMark();  // 标记进入Rs485调试模式
                        DealCmd((char *)g_uRecData,iResult);

                        uEnter485Time = uNowTime;
                    }
                }
                else
                {
                    if(Rs485DebugRead())
                    {
                        UartClearRecBuffer(UPDATE_UART_USE);
                    }
                }
            }
        }
        #endif

        if(0x0A==g_LoggerRun.update)
        {
            UartUpdate(g_uUpdateBuffer);
        }
    }

}

/****************************************************************************
* 名    称：DealCmd()
* 功    能：MODBUS写
* 入口参数：
*         pMaster:主站参数结构体指针
*         uAddr:从设备地址
*         uCmd:功能码 0x06  0x10
* 出口参数：无
* 范    例: 无
****************************************************************************/
#define  PRERESET  0x01   // 预置恢复出厂设置
#define  PREBACK   0x02   // 预置程序回滚
#define  UPDATE    0x04   // 预置程序升级

void DealCmd(char *uRec,uint16_t uLen)
{
    static uint8_t uPreSet=0;
    uint8_t uCmd;
    uint8_t uMesHead,uMesTail,i;
    uint16_t uValue;
    uint32_t uTemp;

    uint16_t j;
    OS_STK_DATA sStk;
    
    if(uLen < 4)
    {
        return ;
    }
    
    uValue = CRC16((uint8_t *)uRec,uLen-2);
    
    if(uValue!=((uRec[uLen-2]<<8)|uRec[uLen-1]))
    {
        WDEBUGOUT("CRC错误\r\n");
        return ;
    }

    if('A'==uRec[0] && 'A'==uRec[1])
    {
        uCmd = (uRec[2] - '0')*10 + uRec[3] - '0';
    }
    else
    {
        uCmd = (uRec[0] - '0')*10 + uRec[1] - '0';
    }

    for(i=3,uMesHead=0,uMesTail=0; i<uLen; i++)
    {
        if('\"'==uRec[i])
        {
            if(0==uMesHead)
            {
                uMesHead = i;
                continue;
            }
            if(0==uMesTail)
            {
                uMesTail = i;
                break;
            }
        }
    }

    switch(uCmd)
    {
    case 1:   // 显示数采信息
        ReadEepData(EEP_LOGGER_INF);   // 先从EEprom读取信息

        PrintThisInfo();
        break;

    case 2:// 显示设备信息
        WDEBUGOUT("\n序号\t地址\t点表号\tESN\t\t协议类型 遥信\t遥测\t遥控\t遥调\t波特率\t设备软件版本\t %d台\t\n",g_DeviceSouth.device_sum);
        if(0==g_DeviceSouth.device_sum)
        {
            WDEBUGOUT("没有设备\r\n");
            break;
        }
        for(i=0; i<MAX_device; i++)
        {
            switch(g_DeviceSouth.device_inf[i].baud_rate)
            {
            case 1:
                uTemp = 2400;
                break;

            case 2:
                uTemp = 4800;
                break;

            case 3:
                uTemp = 9600;
                break;

            case 4:
                uTemp = 19200;
                break;

            case 5:
                uTemp = 38400;
                break;

            case 6:
                uTemp = 115200;
                break;

            default:
                uTemp = 0;
                break;
            }
            if(0==g_DeviceSouth.device_inf[i].addr)
            {
                continue;
            }
            else
            {
                WDEBUGOUT("%02d\t%02d\t%d\t%-17.17s %d\t0x%04X\t0x%04X\t0x%04X\t0x%04X\t%d\t%-17.17s\t%c%c\n",//WDEBUGOUT("%02d    %02d    %d   %-20.20s %d    0x%04X   0x%04X   0x%04X   %d   %c%c    %0.17s\n",
                         i,g_DeviceSouth.device_inf[i].addr,g_DeviceSouth.device_inf[i].protocol_num,
                         g_DeviceEsn.cDeviceEsn[i],g_DeviceSouth.device_inf[i].protocol_type,
                         g_DeviceSouth.device_inf[i].yx_start_addr,
                         g_DeviceSouth.device_inf[i].yc_start_addr,
                         g_DeviceSouth.device_inf[i].yk_start_addr,
                         g_DeviceSouth.device_inf[i].sd_start_addr,
                         uTemp,
                         g_DeviceSoft.cDeviceSoft[i],
                         JUDGE(g_LoggerRun.err_lost & (1<<i)),
                         JUDGE(g_LoggerAlarm.dev_lost & (1<<i))
                        );
				msleep(5);
            }
        }
        break;

    case 3:// 配置数采ESN
        if(!strstr(uRec,"03-ESN"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }

        if((uMesTail-uMesHead)<1 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("ESN错\n");
        }
        else
        {
            memset(g_LoggerInfo.esn, 0, 20);
            strncpy(g_LoggerInfo.esn,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("ESN:%-20.20s\n",g_LoggerInfo.esn);

            SaveEepData(EEP_LOGGER_INF);// 存储
        }
        break;

    case 4:  // 配置数采名称
        if(!strstr(uRec,"04-NAME"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }

        if((uMesTail-uMesHead)<3 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("名称错\n");
        }
        else
        {
            memset(g_LoggerInfo.name, 0, 20);
            strncpy(g_LoggerInfo.name,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("名称:%-20.20s\n",g_LoggerInfo.name);

            SaveEepData(EEP_LOGGER_INF);// 存储
        }
        break;

    case 5:  // 配置数采型号
        if(!strstr(uRec,"05-MOD"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }

        if((uMesTail-uMesHead)<3 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("型号错\n");
        }
        else
        {
            memset(g_LoggerInfo.model, 0, 20);
            strncpy(g_LoggerInfo.model,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("型号:%-20.20s\n",g_LoggerInfo.model);

            SaveEepData(EEP_LOGGER_INF);// 存储
        }
        break;

    case 6:  // 配置数采类型
        if(!strstr(uRec,"06-TYPE"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }

        if((uMesTail-uMesHead)<3 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("类型错\n");
        }
        else
        {
            memset(g_LoggerInfo.type, 0, 20);
            strncpy(g_LoggerInfo.type,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("类型:%-20.20s\n",g_LoggerInfo.type);

            SaveEepData(EEP_LOGGER_INF);// 存储
        }
        break;

    case 7:  // 配置服务器IP
        if(!strstr(uRec,"07-IP"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }
        uValue = 0;
        uTemp = 0;
        for(i=uMesHead; i<=uMesTail; i++)
        {
            if('\"'!=uRec[i] && '.'!=uRec[i])
            {
                if(uRec[i]<'0' || uRec[i]>'9')
                {
                    uValue = 0;
                    break;
                }
                uTemp = uTemp*10 + uRec[i]-'0';
            }
            if('.'==uRec[i] || '\"'==uRec[i])
            {
                if(uTemp>255)
                {
                    uValue = 0;
                    break;
                }
                uTemp = 0;
                uValue++;
            }
        }
        if(uValue<3 || (uMesTail-uMesHead)>16)
        {
            WDEBUGOUT("IP错\n");
        }
        else
        {
            uRec[uMesTail+1] = '\0';
            memset(g_LoggerInfo.server_domain,0,sizeof(g_LoggerInfo.server_domain));
            strncpy(g_LoggerInfo.server_domain,&uRec[uMesHead],uMesTail-uMesHead+2);
            WDEBUGOUT("IP:%s\n",g_LoggerInfo.server_domain);

            SaveEepData(EEP_LOGGER_INF);// 存储
        }
        break;

    case 8: // 配置域名
        if(!strstr(uRec,"08-DOMAIN"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }
        if((uMesTail-uMesHead)<5 || (uMesTail-uMesHead)>31)
        {
            WDEBUGOUT("域名错\n");
        }
        else
        {
            uRec[uMesTail+1] = '\0';
            memset(g_LoggerInfo.server_domain,0,sizeof(g_LoggerInfo.server_domain));
            strncpy(g_LoggerInfo.server_domain,&uRec[uMesHead],uMesTail-uMesHead+2);
            WDEBUGOUT("域名:%s\n",g_LoggerInfo.server_domain);

            SaveEepData(EEP_LOGGER_INF);// 存储
        }
        break;

    case 9:  // 配置端口
        if(!strstr(uRec,"09-PORT"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }

        if((uMesTail-uMesHead)<2 || (uMesTail-uMesHead)>6)
        {
            WDEBUGOUT("端口错\n");
        }
        else
        {
            if(Str2UI(&uRec[uMesHead],&uValue))
            {
                WDEBUGOUT("端口错\n");
            }
            else if(uValue<1 || 65534< (uValue-1))
            { 
                WDEBUGOUT("请输入端口号范围1到65535!\n");
            }
            else
            {
                g_LoggerInfo.server_port = uValue;
                WDEBUGOUT("端口:%d\n",g_LoggerInfo.server_port);

                SaveEepData(EEP_LOGGER_INF);// 存储
            }
        }
        break;

    
    case 10:  // 配置查询间隔时间
        //WDEBUGOUT("不可改\r\n");
		for(i=0,uMesHead=0,uMesTail=0; i<uLen; i++)
		{
			if('\"'==uRec[i])
			{
				if(0==uMesHead)
				{
					uMesHead = i;
					continue;
				}
				if(0==uMesTail)
				{
					uMesTail = i;
					break;
				}
			}
		}

        if((uMesTail-uMesHead)<1)
        {
            WDEBUGOUT("时间错误\r\n");
        }
        else
        {
            if(Str2UI(&uRec[uMesHead],&g_LoggerInfo.inquire_interval_time))
            {
                WDEBUGOUT("时间错误\r\n");
            }
            else
            {
               /* if(uValue > 600) // 超过10分钟
                {
                    WDEBUGOUT("最长600秒\r\n");
                    uValue = 600;
                }*/
                if(g_LoggerInfo.inquire_interval_time > 900) // 超过15分钟
                {
                    WDEBUGOUT("能源采集最长900秒\n");
                    g_LoggerInfo.inquire_interval_time = 900;
                }
                else if(g_LoggerInfo.inquire_interval_time < 60) // 小于1分钟
                {
                    WDEBUGOUT("能源采集最短60秒\n");
                    g_LoggerInfo.inquire_interval_time = 60;
                }
                //g_LoggerInfo.inquire_interval_time = uValue;
                WDEBUGOUT("时间:%ds\r\n",g_LoggerInfo.inquire_interval_time);

                SaveEepData(EEP_LOGGER_INF);// 存储
               // ChangeIntervalTime();
            }
        }
        break;
        
    case 11:  // 恢复出厂设置
        if(uPreSet&PRERESET)
        {
            AllReset(1);
            //soft_timer_set(&SoftReboo,30000,1);  // 设置时间并开始计时
            g_LoggerRun.update = 0x00;
            uPreSet &= ~PRERESET;
            WDEBUGOUT("恢复完成\n");
        }
        else
        {
            uPreSet |= PRERESET;
            WDEBUGOUT("再次设置以确认\n");
        }
        break;

    case 12:  // 回滚
        if(0==(uPreSet&PREBACK))
        {
            uPreSet |= PREBACK;
            WDEBUGOUT("再次设置以确认,\"ESC\"取消\n");
        }
        else
        {
            uPreSet &= ~PREBACK;

            // 存储升级信息到EEPROM，然后重启芯片
            uValue = ReadEepData(EEP_UPDATA);//EEP_Read_data(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)updata,sizeof(UPDATA_MARK_T),&updata->CRC);// 升级信息读取
            if(EEP_OK==uValue) // 读取数据成功,且 回滚数据标志标记为有回滚数据
            {
                if(0xBB==g_LoggerUpdate.rollback_allow) // 有历史程序，可以回滚
                {
                    g_LoggerUpdate.frame_sum      = 0x00;
                    g_LoggerUpdate.updata_mark    = 0xBB;       // 0xAA升级；0xBB回滚；0x55无
                    g_LoggerUpdate.rollback_allow = 0xBB;

                    if(GetVerS2()&0x01)  // 小版本号最低位，1：程序在A面；0：程序在B面
                    {
                        g_LoggerUpdate.a_version[0] = GetVerS2();   // 当前版本1
                        g_LoggerUpdate.a_version[1] = GetVerS1();   // 当前版本1
                        g_LoggerUpdate.a_version[2] = GetVerType(); // 当前版本1
                    }
                    else
                    {
                        g_LoggerUpdate.b_version[0] = GetVerS2();   // 当前版本1
                        g_LoggerUpdate.b_version[1] = GetVerS1();   // 当前版本1
                        g_LoggerUpdate.b_version[2] = GetVerType(); // 当前版本1
                    }

                    g_LoggerUpdate.reserve = 0x00;

                    SaveEepData(EEP_UPDATA);// EEP_Save_data(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)updata,sizeof(UPDATA_MARK_T),&updata->CRC);// 升级信息存储

                    WDEBUGOUT("重启…\n");
                    Reboot();  // 重启
                }
                else
                {
                    WDEBUGOUT("无历史程序\n");
                }
            }
        }
        break;

    case 13:  // 取消预置操作
        uPreSet = 0x00;

        if(0x0A==g_LoggerRun.update)
        {
            g_LoggerRun.update &= 0xF0;
            WDEBUGOUT("UartInterface case 13 reboot!");
            Reboot();  // 重启芯片
        }

        WDEBUGOUT("操作取消\r\n");
        break;

    case 14:  // 复位，重启芯片
        WDEBUGOUT("UartInterface case 14 reboot！");
        Reboot();  // 重启
        break;

    case 15:
        WDEBUGOUT("总数-遥信:%d 遥测:%d 电度:%d 遥控:%d 设点:%d 告警：%d\n",g_DeviceSouth.yx_sum,g_DeviceSouth.yc_sum,g_DeviceSouth.dd_sum,g_DeviceSouth.yk_sum,g_DeviceSouth.sd_sum);
        for(i=0; i<MAX_device; i++)
        {
            if(g_DeviceSouth.protocol[i].protocol_num)  // 有点表数据 点表号不会是0
            {
                WDEBUGOUT("\n序号:%d，点表号:%d\n",i,g_DeviceSouth.protocol[i].protocol_num);
                WDEBUGOUT("寄存器\t\t信息类型\t信息长度\t数据类型\n");
                for(j=0; j<g_DeviceSouth.protocol[i].mess_point_sum; j++)
                {
                    
						WDEBUGOUT("%4X\t%d\t%d\t%02d\t%d\n",

						g_pRegPoint[i][j].reg_addr,
						g_pRegPoint[i][j].reg_addr,
						g_pRegPoint[i][j].reg_type.type.mess,
						g_pRegPoint[i][j].reg_count,
						g_pRegPoint[i][j].reg_type.type.data);
                    
                }
            }
            else
            {
                continue;
            }
        }
        WDEBUGOUT("\n--打印完成--\n");
        break;

    case 16:
        if(!strstr(uRec,"16-PHONE"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }
        if(11!=(uMesTail-uMesHead-1))
        {
            WDEBUGOUT("号码错\n");
        }
        else
        {
            //memset(Logger_inf., 0, 20);
            strncpy(g_LoggerInfo.phonenum,&uRec[uMesHead+1],11);
            WDEBUGOUT("号码:%-11.11s\n",g_LoggerInfo.phonenum);

            SaveEepData(EEP_LOGGER_INF);// 存储
        }
        break;

    case 17:
        if(0==g_LoggerRun.err)
        {
            WDEBUGOUT("无数采本机告警\n");
        }
        else
        {
            if(g_LoggerRun.err & err_power_off)
            {
                WDEBUGOUT("[数采]断电\n");
            }
        }
        //WDEBUGOUT("[数采]RAM Space %d Bytes\n",FreeRamSpace());

        //----------------------------------------
        // 南向设备告警
        WDEBUGOUT("南向设备告警\n");
        for(i=0; i<MAX_device&&i<g_DeviceSouth.device_sum; i++)
        {
            WDEBUGOUT("序号:%2d,地址:%2d:\n",i,g_DeviceSouth.device_inf[i].addr);
            uValue = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[i].rel_num].alarm_sum;
            for(j=0; j<uValue; j++)
            {
                WDEBUGOUT("MOD:%d,%d:NEW:0x%04X,ACK:0x%04X\n",g_psSouthAlarmCopy[i][j].mdbus_addr,g_psSouthAlarm[i][j].mdbus_addr,g_psSouthAlarmCopy[i][j].alarm_value,g_psSouthAlarm[i][j].alarm_value);
            }
        }
        WDEBUGOUT("-打印完成-\n");

        break;

      case 20:
		    for(i=2,uMesHead=0,uMesTail=0; i<uLen; i++)
		    {
		        if('-'==uRec[i])
		        {
		            if(0x30==uRec[i+1])
		            {
		                SouthSwich = 0;
		                WDEBUGOUT("关闭南向报文显示");
		            }
		            if(0x31==uRec[i+1])
		            {
		                SouthSwich = 1;
		                WDEBUGOUT("显示南向报文发送");
		            }
		            if(0x32==uRec[i+1])
		            {
		                SouthSwich = 2;
		                WDEBUGOUT("显示南向报文接收");
		            }
		            if(0x33==uRec[i+1])
		            {
		                SouthSwich = 3;
		                WDEBUGOUT("显示南向收发报文");
		            }
		        }
		    }
		    break;

  /*  case 21:  // 预置点表
        if(!strstr(uRec,"21-TABLE"))
        {
            WDEBUGOUT("格式错\n");
            break;
        }
        if((uMesTail-uMesHead)<2 || (uMesTail-uMesHead)>6)
        {
            WDEBUGOUT("编号错\n");
            break;
        }

        Str2UI(&uRec[uMesHead],&uValue);
        if(uValue<=60000)
        {
            WDEBUGOUT("取消预置:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }
        else
        {
            WDEBUGOUT("预置编号:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }

        g_LoggerInfo.phonenum[2] = (uValue/10000)%10 +0x30;
        g_LoggerInfo.phonenum[3] = (uValue/1000)%10 +0x30;
        g_LoggerInfo.phonenum[4] = (uValue/100)%10 +0x30;
        g_LoggerInfo.phonenum[5] = (uValue/10)%10 +0x30;
        g_LoggerInfo.phonenum[6] = uValue%10 +0x30;

        SaveEepData(EEP_LOGGER_INF);
        break;
*/
    case 22:
        /*
        // 南向写测试
        Str2UI(&uRec[uMesHead],&uValue);
        WDEBUGOUT("value=%d\n",uValue);


        if(1==uValue)
        {
            IEC104_DATA_YK[uValue] = uValue+1;
            g_uTransData = 1;
        }
        else
        {
            g_uTransData = uValue;
        }

        OSQPost(MesQ, &g_uTransData);
        */
        break;

    case 23:
        /*WDEBUGOUT("数据写入发送链表\n");
        for(i=0;i<20;i++)
        {
            g_uRecData[i] = i;
        }

        CmStoreSendLink(g_uRecData,20);*/
        break;

    case 24:
        Str2UI(&uRec[uMesHead],&uValue);

        DataFlash_Read(uValue,g_uRecData,200);
        WDEBUGOUT("%d起200字节\n",uValue);
        for(i=0;i<200;i++)
        {
            WDEBUGOUT("%02X ",g_uRecData[i]);
        }
        break;
	case 25:
        DEBUGOUT("\ng_DeviceSouth.device_sum:%d\r\n",g_DeviceSouth.device_sum);
		for(uint8_t m=0;m<MAX_device+1;m++)
		 {
			 DEBUGOUT("uEsnMark[%d]:%d\r\n",m,g_DeviceEsn.uEsnMark[m]);
		 }
		break;
	/*case 26:
            for(uint8_t i=0;i<10;i++)
            {
				DEBUGOUT("%d %s\r\n",i+1,uAllocation.cDeviceEsn[i]);
			}
			
			break;*/

	case 30:
	    DEBUGOUT("%%%%%%%%%%%%%%%%%%%%%%%%%%% DATAFLASH READ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
	    uint16_t i = 0, j = 0;
        while (DATAFLASH_RECORD_END < (DATAFLASH_RECORD_HEAD + i))
        {
            DataFlash_Read(DATAFLASH_RECORD_HEAD + i, g_TestData, SIZE);
            for (j = 0; j < SIZE; ++j)
            {
                DEBUGOUT("%02x", g_TestData[j]);
            }
            DEBUGOUT("/n");
            i = i + SIZE;
        }
        
	    break;
    case 33:
        /*for(i=0; i<128; i++)
        {
            g_TestData[i] = i;
            //WDEBUGOUT("%02X ",g_TestData[j*128+i]);
        }
        WDEBUGOUT("Write to EEPROM\n");
        EepSavedata(0x44,g_TestData,128,NULL);
        WDEBUGOUT("Read from EEPROM\n");
        EepReadData(0x44,g_TestOut,128,NULL);
        WDEBUGOUT("Compare Data\n");
        for(j=0;j<128;j++)
        {
            if(g_TestOut[j] != g_TestData[j])
            {
                WDEBUGOUT("Compare Err %d\n",j);
                break;
                //break;
            }
        }*/
        WDEBUGOUT("Compare Data OK\n");
        break;

    case 44:
        /*Str2UI(&uRec[uMesHead],&uValue);
        WDEBUGOUT("value=%d\n",uValue);
        if(uValue>2047)
        {
            WDEBUGOUT("扇区超限\n");
            break;
        }

        for(j=0;j<32;j++)
        {
            for(i=0;i<128;i++)
            {
                g_TestData[j*128+i] = i;
                //WDEBUGOUT("%02X ",g_TestData[j*128+i]);
            }
            //WDEBUGOUT("\n");
        }
        WDEBUGOUT("Erase DataFlash sector %d\n",uValue);
        DataFlash_Sector_Erase(uValue*4096);
        WDEBUGOUT("Write to DataFlash\n");
        DataFlash_Write(uValue*4096,g_TestData,4096);
        WDEBUGOUT("Read from DataFlash\n");
        DataFlash_Read(uValue*4096,g_TestOut,4096);
        WDEBUGOUT("Compare Data\n");
        for(j=0;j<4096;j++)
        {
            if(g_TestOut[j] != g_TestData[j])
            {
                WDEBUGOUT("Compare Err %d\n",j);
                break;
            }
        }
        WDEBUGOUT("Compare Data OK\n");*/
        break;

    case 55:
        WDEBUGOUT("OSTime=%ld\n",OSTimeGet());
        break;

    case 88:
        WDEBUGOUT("[数采]RAM Pool Free %d Bytes\n",FreeRamSpace());
        WDEBUGOUT("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
        OSTaskStkChk(3,&sStk);
        WDEBUGOUT("TaskModemProcess  used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(5,&sStk);
        WDEBUGOUT("TaskIec104Process used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(6,&sStk);
        WDEBUGOUT("TaskSouthInquire  used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(7,&sStk);
        WDEBUGOUT("TaskSouthWrite    used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(8,&sStk);
        WDEBUGOUT("TaskUart0Process  used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(9,&sStk);
        WDEBUGOUT("TaskLedCtrl       used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));

        OSTaskStkChk(OS_TASK_IDLE_PRIO,&sStk);
        WDEBUGOUT("TaskIDLE %d       used/free:%d/%d  usage:%d%%\n",OS_TASK_IDLE_PRIO,sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        WDEBUGOUT("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
        break;

    case 99:
        if(uPreSet&UPDATE)
        {
            /*if(uRec[8]>='0' && uRec[8]<='9')
            {
                if((uRec[8]-'0')&0x01)
                {
                    g_uUpdateSide = 0xAA;
                }
                else
                {
                    g_uUpdateSide = 0xBB;
                }
            }
            else if(uRec[8]>='A' && uRec[8]<='F')
            {
                if((uRec[8]-'A')&0x01)
                {
                    g_uUpdateSide = 0xAA;
                }
                else
                {
                    g_uUpdateSide = 0xBB;
                }
            }
            else
            {
                WDEBUGOUT("Version Err\r\n");
                break;
            }*/

            if(0==g_LoggerRun.update)
            {
                g_LoggerRun.update = 0x0A;  // 近端升级
                uPreSet &= ~UPDATE;
            }
            else
            {
                WDEBUGOUT("升级中\n");
                uPreSet &= ~UPDATE;
                break;
            }

            // DataFlash存储升级数据区域擦除，共96KB，一个区块+8个扇区
            DataFlash_Block_Erase(0);// 擦除块0

            DataFlash_Sector_Erase(0x010000);
            DataFlash_Sector_Erase(0x011000);
            DataFlash_Sector_Erase(0x012000);
            DataFlash_Sector_Erase(0x013000);

            DataFlash_Sector_Erase(0x014000);
            DataFlash_Sector_Erase(0x015000);
            DataFlash_Sector_Erase(0x016000);
            DataFlash_Sector_Erase(0x017000);

            UartInit(UPDATE_UART_USE,9600,UART_PARITY_NONE);      // 串口3初始化 RS485

            g_uUpdateBuffer = (uint8_t*)WMemMalloc(g_uUpdateBuffer,270);

            if(NULL==g_uUpdateBuffer)
            {
                WDEBUGOUT("升级缓存申请错误,延时复位\n");
                sleep(1);
                Reboot();// 重启芯片
            }

            WDEBUGOUT("进入升级模式\n");
        }
        else
        {
            uPreSet |= UPDATE;
            WDEBUGOUT("待升级版本:%-6.6s\n",&uRec[3]);
            WDEBUGOUT("再次设置以确认\n");
        }
        break;

    default:
        WDEBUGOUT("无效指令\n");
        break;
    }
}

/******************************************************************************
* 名    称：UartUpdate()
* 功    能：通过近端串口升级程序。
* 入口参数：
            无
*
* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t UartUpdate(uint8_t *pUpdteBuffer)
{
    static  uint16_t uFrameCount=0;   // 接收帧计数
    uint16_t uCrc;                    // CRC校验数据
    //uint16_t uDataCount;              // 串口读取字节数量
    uint16_t uRecSeq;                 // 接收到的帧序号

    int16_t iCom;
    uint8_t  *uRrecData = (uint8_t *)pUpdteBuffer;          // 串口数据缓存

    iCom = UartRxLen(UPDATE_UART_USE);   // 读取串口缓冲区的字节数量
    if(iCom<=0)
    {
        return 0;
    }

    iCom = UartRead(UPDATE_UART_USE, pUpdteBuffer, 270, 2);

    if(iCom<262)
    {
        return 0;
    }

    uRecSeq = (uRrecData[2] <<8) | uRrecData[3];

    uCrc = CRC16(uRrecData,iCom-2);

    //-----------------------------------------------------
    if(uCrc!=(uRrecData[iCom-2]<<8 | uRrecData[iCom-1]))
    {
        uRrecData[4] = 0x00;
        uRrecData[5] = 0x55;  // 校验失败
        uCrc = CRC16(uRrecData,6);

        uRrecData[6] = uCrc>>8;
        uRrecData[7] = uCrc&0xFF;

        DEBUGOUT("Updata CRC ERR\n");
        UartWrite(UPDATE_UART_USE,uRrecData,8);
        return 0;
    }
    //-----------------------------------------------------
    if(uFrameCount != (uRecSeq&0x7fff))  // 接收帧序号不对
    {
        uRrecData[2] = uFrameCount>>8;
        uRrecData[3] = uFrameCount&0xFF;  // 需要重传的帧
        uRrecData[4] = 0x00;
        uRrecData[5] = 0x01;  // 接收帧序号不对
        uCrc = CRC16(uRrecData,6);

        uRrecData[6] = uCrc>>8;
        uRrecData[7] = uCrc&0xFF;

        DEBUGOUT("Updata Seq ERR\n");
        UartWrite(UPDATE_UART_USE,uRrecData,8);
        return 0;
    }


    // 存储到DataFlash升级数据区域，数据长度：减去104报文和CRC校验
    DataFlash_Write(uFrameCount * 256,(uint8_t *)&uRrecData[4],iCom-6);

    uFrameCount++;

    //--------------------------------------------------------------------
    if(uRecSeq&0x8000)  // 最后一帧
    {
        uCrc = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// 升级信息读取

        if(0==uCrc) // 读取数据失败,有回滚数据标志标记为没有回滚数据
        {
            g_LoggerUpdate.rollback_allow = 0x55;
        }
        g_LoggerUpdate.frame_sum = uFrameCount;// 256个字节一帧，总帧数

        if(GetVerS2()&0x01)  // 小版本号最低位，1：程序在A面；0：程序在B面
        {
            g_LoggerUpdate.a_version[0] = GetVerS2();   // 当前版本1
            g_LoggerUpdate.a_version[1] = GetVerS1();   // 当前版本1
            g_LoggerUpdate.a_version[2] = GetVerType(); // 当前版本1
        }
        else
        {
            g_LoggerUpdate.b_version[0] = GetVerS2();   // 当前版本1
            g_LoggerUpdate.b_version[1] = GetVerS1();   // 当前版本1
            g_LoggerUpdate.b_version[2] = GetVerType(); // 当前版本1
        }

        g_LoggerUpdate.reserve = 0x00;  // 预留，占位

        if(2==(uRrecData[1]>>4))
        {
            g_LoggerUpdate.side_tobe = 0xBB;  // 升级到B面
        }
        else
        {
            g_LoggerUpdate.side_tobe = 0xAA;  // 升级到A面
        }

        //g_LoggerUpdate.side_tobe = uSide;  // 目标升级到AB面


        if(!uFrameCount)  // 没有升级包数据
        {
            g_LoggerUpdate.updata_mark = 0x55;  //  data[2] = 0x55;    // 0xAA升级；0xBB回滚；0x55无
        }
        else
        {
            g_LoggerUpdate.updata_mark = 0xAA;  // data[2] = 0xAA;    // 0xAA升级；0xBB回滚；0x55无
        }

        SaveEepData(EEP_UPDATA);

        DEBUGOUT("UPDATA REC END,REBOOT\n");

        uFrameCount = 0;
        WMemFree(g_uUpdateBuffer);
    }
    else
    {
        DEBUGOUT("%d.",uFrameCount);
    }

    //--------------------------------------------------------------------
    uRrecData[4] = 0x00;
    uRrecData[5] = 0xAA;  // 数据正确
    uCrc = CRC16(uRrecData,6);

    uRrecData[6] = uCrc>>8;
    uRrecData[7] = uCrc&0xFF;

    UartWrite(UPDATE_UART_USE,uRrecData,8);

    if(uRecSeq&0x8000)  // 最后一帧
    {
        sleep(2);

        Reboot();// 重启芯片
    }
    //--------------------------------------------------------------------
    return 0;
}
