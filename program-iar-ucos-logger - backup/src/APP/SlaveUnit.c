#include "SlaveUnit.h"
#include <time.h>
#include "ModbusMaster.h"
#include "GlobalVar.h"
#include "Usart.h"
#include "DataTransferArea.h"
#include "IEC104.h"
#include "Record.h"
//#include "PersetTable.h"
#include "Uart0process.h"
#include <stdlib.h>
//#include "tool.h"
#include "WatchDog.h"
#include <math.h>
#include "tool.h"
#include "log.h"
#include "Memory.h"
#include "DataFlash.h"
#include "CRC16.h"

//====================================================================
#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // 延时x秒
//====================================================================
#define JUDGE(x)  ((x)==0?0:1)  // 判断数值是否为0，为0返回0，非0返回1
//====================================================================
#define SLAVE_UART_USE            uart3         // 使用串口3
//====================================================================
#define  DISCORY_ROUND_TIME           (300000)     // 南向查询循环时间ms
#define  SLAVE_ROUND_TIME             (g_LoggerInfo.inquire_interval_time*1000)    
//====================================================================

// 记录查询南向的信息记录
typedef struct
{
    uint8_t  uAddr;            // 发送的设备地址--绝对地址
    uint8_t  uFun;             // 发送的功能号
    uint16_t uRegAddr;         // 发送的寄存器地址
    uint8_t  uRegCount;        // 寄存器数量
    uint8_t  uType;            // 数据类型，遥信，遥测，遥控，设点，电度
    uint16_t uPointCount;      // 发送的点数记录

    uint16_t uYcCount;         // 累积发送遥测点计数
    uint8_t  uYxCount;         // 累积发送遥信点计数
    uint8_t  uDdCount;         // 累积发送电度点计数

    uint16_t uPointHead;       // 当次发送的点的序号
    uint8_t  uPointSum;        // 当次发送的点的计数
    uint8_t  uBaudRate;        // 当前使用波特率  1:2400；2:4800；3:9600；4:19200；5:38400；6:115200
}DEVICE_INQUIRE_INFO_T;


// 南向查询的寄存器地址、长度，功能码记录
typedef struct
{
    uint16_t uRegAddr;     // MODBUS寄存器地址
    uint8_t  uRegCount;    // 寄存器数量
    uint8_t  uFun;         // 功能码
    uint8_t  uDevAddr;     // 设备通讯地址
}DEVICE_COM_INFO_T;
typedef struct
{
    uint16_t uRegAddr;     // MODBUS寄存器地址
    uint8_t  uRegCount;    // 寄存器数量
    uint8_t  uFun;         // 功能码
    uint8_t  uDevAddr;     // 设备通讯地址
}DEVICE_ALARM_COM_INFO_T;

//---------------------用于突发--------------------------
typedef struct
{
    uint8_t yx_data[250];       // 遥信突发数据，一个遥信点占用4字节，最多60点，共240字节
    uint8_t yc_data[240];       // 遥测突发数据，一个遥测点占用8字节，最多30点，共240字节
    uint8_t yx_count;           // 遥信突发上传点计数
    uint8_t yc_count;           // 遥测突发上传点计数
    uint8_t alarm_report[240];  // 一个告警点占用6字节
    uint8_t alarm_count;        // 告警点计数
}IEC104_BURST_T;

/*typedef struct
{
    char  cDeviceEsn[MAX_device][18];       // 南向设备
    //uint8_t uEsnMark[MAX_device];            //分配地址与未分配地址的表计
    //uint16_t  CRC;                     // 存储数据的CRC校验  
}DEVICE_ADDR_INFO_T;*/

// 表计升级包总字节数
//typedef struct
//{
//    uint32_t nDataLen;  //文件字节长度
//    uint32_t nFileCrc;  //文件数据CRC
//} DT1000UPDATA_DATA_T;
// 用于表计升级

//====================================================================
//static DT1000UPDATA_DATA_T g_DT1000DataLen;
//static uint8_t  s_uSouthReadSd=SOUTH_CMD_READSD; //南向读设点
static uint8_t uNewHWDevCount=0;      // 搜索华为设备时，搜索到设备数量计数
//static uint8_t uSetNewDeviceAddr=1;
static IEC_FORMAT_T    g_sIecSend;     // 用于IEC104组帧
//static IEC104_MAIN_T   *g_sIecRec;     // 用于IEC104组帧
static IEC104_BURST_T g_sBurst;       // 突发数据存储，信息点计数
MODBUS_MASTER_T g_sMaster;            // 主站结构体
static uint8_t  uRecBuffer[256];      // 接收缓存
OS_EVENT *pUartLock;                   // 串口锁

static DEVICE_INQUIRE_INFO_T sSouth;  // 查询信息记录
static DEVICE_INQUIRE_INFO_T sAlarmSouth; // 查询告警记录

static uint8_t g_uAlarmReport;        // 告警状态
static uint32_t g_uTimeNow;           // 当前的时间戳
//static uint8_t  s_uNorthReadSd = NORTH_CMD_READSD; //南向读设点

uint8_t SetBaudRate(uint8_t now,uint8_t target);
void ResetIecData(uint8_t uRelAddr);
void AlarmAddDev(uint8_t uRelAddr,uint8_t uSum,uint16_t uModAddr,uint16_t uValue);
uint8_t AlarmCheckout(void);
uint8_t TimedReboot(uint8_t uRandom);
uint8_t AlarmReport(uint8_t addr,uint8_t err_code,uint16_t reg_addr,uint8_t offset,uint8_t event,uint8_t imd);
uint32_t g_South_Action_Newtime = 0,gImport_Table_time = 0;

/******************************************************************************
* 名    称：SlaveAddrConvert()
* 功    能：根据通许地址，找出对应的数组下标（相对地址）。
* 入口参数：
            uRealAddr   设备真实地址
*
* 出口参数：设备相对地址
* 范    例:
******************************************************************************/
uint8_t SlaveAddrConvert(uint8_t uRealAddr)
{
	uint8_t uRelAddr=0xff;
	uint8_t i=0;
    if(!uRealAddr)      //通讯地址不存在
    {
    	return uRelAddr;
    }
    for(i=0; i<MAX_device; i++)
    {
        if(g_DeviceSouth.device_inf[i].addr==uRealAddr)
        {
            uRelAddr = i;
            break;
        }
    }

	return uRelAddr;
}
//===============================================================================
/******************************************************************************
* 名    DTEsnChange()
* 功    能：从0x1B查询到的数据中查找ESN号与已存在设备的比对，不同进行2B，有相同则退出。
* 入口参数：
            uAddr:     设备通讯地址
            data       收到的数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
int8_t DTEsnChange(uint8_t *data)
{
    uint8_t i;
    uint8_t uEsn[17] = {0};
	
    memcpy(uEsn,data,17);
	for(i = 0;i < MAX_device;)
	{
		if(0 == memcmp(g_DeviceEsn.cDeviceEsn[g_DeviceSouth.device_inf[i].addr],uEsn,17)) // ESN不相同
		{
		    return 0;
		}
		i++;
	}
	return 1; 
}
/******************************************************************************
* 名    称：HwEsnChange()
* 功    能：从0x2B查询到的数据中查找ESN号与保存的比对，有更改就上报。
* 入口参数：
            uAddr:     设备通讯地址
            data       收到的数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
void HwEsnChange(uint8_t uAddr,uint8_t* data)
{
    //static uint16_t esn_changed=0;
    uint8_t uAddrTmp;
    uint8_t uEsn[17]={0};

    uAddrTmp = SlaveAddrConvert(uAddr);  // 将通讯地址转化为相对地址

    if(uAddrTmp>=MAX_device)  // 最多MAXDEVICE台，设备地址最大到MAXDEVICE,相对地址为0~MAXDEVICE
    {
        return;
    }
	
    if(0!=memcmp(uEsn,uAllocation.cDeviceEsn[uAddrTmp],17)) // ESN不相同
    {
        // 发送
        g_sIecSend.format.maddrL = g_DeviceSouth.device_inf[uAddrTmp].addr;
        g_sIecSend.format.maddrM = 0x00;
        g_sIecSend.format.maddrH = 0x00;

        memcpy(g_sIecSend.format.data,&uEsn[0],17);// I帧数据
        IecCreateFrameI(P_HW_ESN,0x01,R_SETTING,17,&g_sIecSend); // 字节数为信息点数*4 - 已经计数的第一个信息体地址
    }
    /*else
    {
        esn_changed &= ~(1<<addr); // 标记已经上报的ESN号更改
    }*/

}
/******************************************************************************
* 名    称：HwDeviceEsn()
* 功    能：从0x2B查询到的数据中查找ESN号。
* 入口参数：
            uAddr:      设备通讯地址
            data:       收到的数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
void HwDeviceEsn(uint8_t uAddr,uint8_t* data)
{
    uint8_t i,k;
    uint8_t uRelAddr;

    if(uAddr>MAX_device || 0==uAddr)  // 最多10台，设备地址最大到10 || 0==addr
    {
        return;
    }

    k = data[10] + 11;
    for(i=11;i<k;i++)
    {
        if('4'==data[i])
        {
            i++;
            if('='==data[i])
            {
                break;
            }
            else
            {
                continue;
            }
        }
    }
    i++;

    for(uRelAddr=0;uRelAddr<MAX_device;uRelAddr++)
    {
        if(0==g_DeviceSouth.device_inf[uRelAddr].addr)
        {
            g_DeviceSouth.device_inf[uRelAddr].addr = uAddr;
            break;
        }
    }

    if(uRelAddr>=MAX_device)
    {
        return;
    }

    memset(g_DeviceEsn.cDeviceEsn[uRelAddr],0,20);  // 清空原来的数据

    for(k=0;k<20;k++)
    {
        if(';'!=data[i])
        {
            g_DeviceEsn.cDeviceEsn[uRelAddr][k] = data[i];
            i++;
        }
        else
        {
            break;
        }
    }
}
/******************************************************************************
* 名    称：HwDeviceSoft()
* 功    能：从0x2B查询到的数据中查找设备软件版本号。
* 入口参数：
            addr:      设备通讯地址
            data       收到的数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
void HwDeviceSoft(uint8_t uAddr,uint8_t* data)
{
	uint8_t i,k;
    uint8_t uRelAddr;

    if(uAddr>MAX_device || 0==uAddr)  // 最多10台，设备地址最大到10 || 0==addr
    {
        return;
    }
	//DEBUGOUT("data:%s",&data[0]);

    k = data[10] + 11;
    for(i=11;i<k;i++)
    {
        if('2'==data[i])
        {
            i++;
            if('='==data[i])
            {
                break;
            }
            else
            {
                continue;
            }
        }
    }
    i++;
    uRelAddr = SlaveAddrConvert(uAddr);

    if(uRelAddr>MAX_device)
    {
        return;
    }

    memset(g_DeviceSoft.cDeviceSoft[uRelAddr],0,17);  // 清空原来的数据
    for(k=0;k<17;k++)
    {
        if(';'!=data[i])
        {
            g_DeviceSoft.cDeviceSoft[uRelAddr][k] = data[i];
            i++;
        }
        else
        {
			break;
        }
    }
}
/******************************************************************************
* 名    称：HwSoftChange()
* 功    能：从0x2B查询到的数据中查找soft号与保存的比对，有更改就上报。
* 入口参数：
            addr:      设备相对地址
            data       收到的数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
void HwSoftChange(uint8_t uAddr,uint8_t* data)
{
    uint8_t i,k,addr;
    uint8_t soft[17]={0};

    addr = SlaveAddrConvert(uAddr);  // 将通讯地址转化为相对地址

    if(addr>=MAX_device)  // 最多3台，设备地址最大到3,相对地址为0~3
    {
        return;
    }

    k = data[10] + 11;
    for(i=11;i<k;i++)
    {
        if('2'==data[i])
        {
            i++;
            if('='==data[i])
            {
                break;
            }
            else
            {
                continue;
            }
        }
    }

    i++;
    for(k=0;k<17;k++)
    {
        if(';'!=data[i+k])
        {
            soft[k] = data[i+k];
        }
        else
        {
            break;
        }
    }

    if(ReportCtrlRead(REPORT_HW_SOFT))// g_uNextFrame & REPORT_HW_DEVICE)  // 上报了一帧华为设备信息，等待平台回复后再处理软件版本号上报
    {
        return;
    }

    if(0!=memcmp(soft,g_DeviceSoft.cDeviceSoft[addr],17)) // 软件号不相同
    {
        memcpy(g_DeviceSoft.cDeviceSoft[addr],soft,17);

        // 发送
        g_sIecSend.format.maddrL = 0x00;
        g_sIecSend.format.maddrM = 0x00;
        g_sIecSend.format.maddrH = 0x00;

        g_sIecSend.format.data[0] = g_DeviceSouth.device_inf[addr].addr;               //设备地址
        g_sIecSend.format.data[1] = 0x01;                                             //连接端口

        memcpy(&(g_sIecSend.format.data[2]),&data[10],data[10]+1);// I帧数据
        IecCreateFrameI(P_HW_INFO,0x01,R_INFO_REPORT,data[10]+3,&g_sIecSend); // 字节数为信息点数*4 - 已经计数的第一个信息体地址

        ReportCtrlSet(REPORT_HW_SOFT);  // 标记发送了一帧上报华为设备软件版本信息报文
    }
    return;
}
/******************************************************************************
* 名    称：SouthDeviceCheck()
* 功    能: 南向设备检查
* 入口参数：
            无
*
* 出口参数：无
* 范    例:
******************************************************************************/
 uint8_t SouthDeviceCheck(uint8_t CheckMark)
{
	for(uint8_t k = 1;k < (MAX_device+1);)
	{
		if(0 == memcmp(&CheckMark,&g_DeviceSouth.device_inf[k-1].addr,1))
		{		

		    HwSoftChange(g_DeviceSouth.device_inf[k-1].addr,uRecBuffer);
            sleep(2);
			return 0;
		}
		k++;
	}
	return 1;

}
/******************************************************************************
* 名    称：SouthDeviceReport()
* 功    能：上报已存在的南向设备。
* 入口参数：
            无
*
* 出口参数：无
* 范    例:
******************************************************************************/
void SouthDeviceReport(uint16_t uReason)
{
    uint8_t i,j,k=0;

    g_sIecSend.format.maddrL = 0x00;
    g_sIecSend.format.maddrM = 0x00;
    g_sIecSend.format.maddrH = 0x00;

    for(i=0; i<MAX_device; i++)
    {
        if(g_DeviceSouth.device_inf[i].addr)
        {
            j = k * 25;

            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].addr;
            j += 1;
            memcpy(&g_sIecSend.format.data[j],&g_DeviceEsn.cDeviceEsn[i],20);// I帧数据
            j += 20;
            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].protocol_num&0xff;
            j += 1;
            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].protocol_num>>8;
            j += 1;
            g_sIecSend.format.data[j] = 1;
            j += 1;
            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].protocol_type;

            k++;

            if(k>=9)  // 一帧报文最多9个设备信息
            {
                IecCreateFrameI(P_SOUTH_INFO,1,uReason,25*k,&g_sIecSend);//R_SETTING
                k = 0;
            }
        }
        else
        {
            continue;
        }
    }

    if(k)
    {
        IecCreateFrameI(P_SOUTH_INFO,1,R_SETTING,25*k,&g_sIecSend);
        k = 0;
    }
}
/******************************************************************************
* 名    称：HwDeviceReport()
* 功    能：上报华为设备自发现。
* 入口参数：
            uStep    1：暂时存数据，直到存满发送；2：只立即发送数据；3：存数据且立即发送数据
            pData    输入的数据数组指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void HwDeviceReport(uint8_t uStep,const uint8_t *pData)
{
    uint8_t i;
    static uint8_t uDataHead=0;   // 华为设备自发现上报的数组下标
    //------------------------------------------------------
    if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        return;
    }

    do
    {
        if((240-uDataHead)>(pData[10]+3) && (1==uStep || 3==uStep))  // 一帧长度256字节，除去协议其他帧内容共15字节// 此帧已经放不下一台设备数据
        {
            g_sBurst.yx_data[uDataHead] = pData[0]; // 设备地址
            uDataHead++;
            g_sBurst.yx_data[uDataHead] = 1;  // 连接端口
            uDataHead++;
            g_sBurst.yx_data[uDataHead] = pData[10];   // 信息长度
            uDataHead++;
            for(i=0; i<pData[10]; i++)
            {
                g_sBurst.yx_data[uDataHead] = pData[i+11];
                uDataHead++;
            }
            g_sBurst.yx_count++;  // 查询到的0x2B设备数量加1

            if(1==uStep)
            {
                uStep = 0;
            }
        }

        if((240-uDataHead)<(pData[10]+3) || (uStep>=2 && g_sBurst.yx_count))  // 一帧长度256字节，除去协议其他帧内容共15字节// 此帧已经放不下一台设备数据
        {
            g_sIecSend.format.maddrL = 0x00;
            g_sIecSend.format.maddrM = 0x00;
            g_sIecSend.format.maddrH = 0x00;
            memcpy(g_sIecSend.format.data,&g_sBurst.yx_data,(uDataHead));// I帧数据
            IecCreateFrameI(P_HW_INFO,1,R_SETTING,(uDataHead),&g_sIecSend); // 0xC7  //199  // 华为设备信息
            g_sBurst.yx_count = 0;  // 查询到的0x2B设备数量
            uDataHead = 0;

            uStep = 0;

            ReportCtrlSet(REPORT_HW_DEVICE);  // 标记发送了一帧上报华为设备信息报文
        }
        else
        {
            uStep = 0;
        }
    }while(uStep);

    //==============================================================
    //------------------------------------------------------
    /*if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        return;
    }

    if(240>(pData[9]+3))  // 一帧长度256字节，除去协议其他帧内容共15字节// 此帧已经放不下一台设备数据
    {
        g_sIecSend.format.maddrL = 0x00;
        g_sIecSend.format.maddrM = 0x00;
        g_sIecSend.format.maddrH = 0x00;

        g_sIecSend.format.data[0] = pData[0]; // 设备地址
        g_sIecSend.format.data[1] = 1;        // 连接端口
        g_sIecSend.format.data[2] = pData[9]; // 信息长度

        memcpy(&g_sIecSend.format.data[3],&pData[10],pData[9]);// I帧数据
        IecCreateFrameI(P_HW_INFO,1,R_SETTING,pData[9]+3,&g_sIecSend); // 0xC7  //199  // 华为设备信息

        ReportCtrlSet(REPORT_HW_DEVICE);  // 标记发送了一帧上报华为设备信息报文
    }*/
}
/******************************************************************************
* 名    称：HwDeviceNew()
* 功    能：从0x2B查询到的数据中查找ESN号与保存的比对，有更改就上报。
* 入口参数：
            uAddr:      设备通讯地址
            data:       收到的数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
void HwDeviceNew(uint8_t uAddr,uint8_t* data)
{
    uint8_t uAddrTmp;

    uAddrTmp = SlaveAddrConvert(uAddr);  // 将通讯地址转化为相对地址
	//DEBUGOUT("uAddrTmp:%d",uAddrTmp);

    if(0xff==uAddrTmp)
    {
        //DEBUGOUT("uNewHWDevCount:%d",uNewHWDevCount);
		if(0==uNewHWDevCount)
        {
            SouthDeviceReport(R_SETTING);
        }
        HwDeviceEsn(data[0],data);             // 提取设备的ESN号
        HwDeviceSoft(data[0],data);     // 提取设备的软件版本号
        msleep(5);
        uNewHWDevCount++;
        HwDeviceReport(1,data);  // 上报 
    }
}
/******************************************************************************
* 名    称：EmptyAdrr()
* 功    能：查询从站设备ESN自动分配地址。
* 入口参数：
*
* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t EmptyAddr()
{
    uint8_t EmptyAddr;
	for(EmptyAddr=1;EmptyAddr<MAX_device+1;)
	{
		if(g_DeviceEsn.uEsnMark[EmptyAddr]==0)
		{
			return EmptyAddr;
		}
		EmptyAddr++;
	}
	return 0;
}
/****************************************************************************
* 名	 称：SlaveDeviceAutoAllocation()
* 功	 能：自分配自发现，1B,2B,3B
* 入口参数：
* 出口参数：0：搜索中；
			1：搜索结束，不需要导表；
			2：搜索结束，需要导表；
			3：平台未连接
* 范	 例: 无
****************************************************************************/
#define  SEARCH_ING         0    // 0：搜索中；
#define  SEARCH_END         1    // 1：搜索结束，不需要导表；
#define  SEARCH_END_IMPORT  2    // 2：搜索结束，需要导表；
#define  SEARCH_NOTCONNECT  3    // 3：平台未连接
int8_t SlaveDeviceAutoAllocation(void)
{
	int8_t	iSearchResult;			// 发起查询结果反馈
	int8_t	iAssignResult[MAX_device];		//分配地址结果反馈
	int8_t	iDiscoveryResult[MAX_device];			  //自发现结果反馈			 
	uint8_t uTempAddr;
	int8_t iResult1B;
	uint8_t uLen;
	uint8_t Confirm_Count = 0;
	static uint8_t uDTAddr = 1;       //D5 1B轮询

	if(NORTH_OK != g_LoggerRun.north_status)
	{
		return SEARCH_NOTCONNECT;
	}

	while(Confirm_Count > 2)
	{
		if(ReportCtrlRead(REPORT_HW_DEVICE)) // 上报的设备信息没有被平台确认
		{
			sleep(1);
			Confirm_Count = 3;
//			return SEARCH_ING;
		}
		else
		{
			Confirm_Count++;
		}
	}

	if(uDTAddr > 1) 
	{ 
		 for(uint8_t uAddr=1;uAddr<MAX_device+1;)
		 {
			 msleep(1500);
		     memset(uRecBuffer,0,256);
			 iDiscoveryResult[uAddr] = ComMasterRead(&g_sMaster,uAddr,DISCOVERY_REPORT,0,0,uRecBuffer,NULL);

			 if(gImport_Table_time)
				 gImport_Table_time = OSTimeGet();
			 if((iDiscoveryResult[uAddr] > 0) && (uRecBuffer[0] == uAddr) && (uRecBuffer[1] == DISCOVERY_REPORT) && (uRecBuffer[2] == 0x45))
		     {
				  if(RUNNING_WORK_READ == g_LoggerRun.run_status)
                  {
					  gImport_Table_time = 0;
					 if(SouthDeviceCheck(uAddr))  //在进行3B轮询的时候，已上报设备地址的只接收设备信息不上报       2B
					 {
						 DEBUGOUT("[Working Discovery]%d: %s\n",uRecBuffer[0],&uRecBuffer[11]);
					     HwDeviceNew(uRecBuffer[0],uRecBuffer);
						 g_DeviceEsn.uEsnMark[uAddr] = 1;
						 gImport_Table_time = OSTimeGet();
					 }

				  }
				  else if(RUNNING_SEARCH_HW == g_LoggerRun.run_status)// 新数采搜索华为设备
				  {
					  DEBUGOUT("[Empty Device Discovery]%d: %s\n",uRecBuffer[0],&uRecBuffer[11]);
					  g_DeviceEsn.uEsnMark[uAddr] = 1;
					  HwDeviceNew(uRecBuffer[0],uRecBuffer);
				  }  
		     }
			 else
			 {
				  g_DeviceEsn.uEsnMark[uAddr] = 0;
			 }
			 uAddr++;
		 }	 

		 if(uNewHWDevCount) // 有搜索到华为设备
		 {
//			 DEBUGOUT("/**********Pinnet Report Have Device!!!\r\n");
			 HwDeviceReport(2,uRecBuffer);  // 上报可能暂存的数据
			 uNewHWDevCount = 0;
			 g_LoggerRun.run_status = RUNNING_SEARCH_END;  // 标记数采状态为搜索华为设备结束
			 msleep(1500);// 延时
			 //return SEARCH_ING;
		 }

		 if(RUNNING_SEARCH_END == g_LoggerRun.run_status)
		 {
			 msleep(200);// 延时
			 g_sIecSend.format.maddrL = 0x00;
			 g_sIecSend.format.maddrM = 0x00;
			 g_sIecSend.format.maddrH = 0x00;
			 g_sIecSend.format.data[0] = 0x01;
			 IecCreateFrameI(P_TABLE,1,R_TABLE_START,1,&g_sIecSend);// 发送启动导表到平台
			 gImport_Table_time = OSTimeGet();
			 uDTAddr = 1;
			 return SEARCH_END_IMPORT;					
		 }
		 else if((RUNNING_SEARCH_HW == g_LoggerRun.run_status ))
		 { 
			 OSTimeDlyHMSM(0,4,0,0);//DEBUGOUT("空数采搜索延时5分钟！");
			 uDTAddr = 1;
			 return SEARCH_ING;
		 }
		 uDTAddr = 1;
		 return SEARCH_END;
	}

	if(RUNNING_WORK_READ != g_LoggerRun.run_status)  // 数采，已经开站，正常工作
	{
		g_LoggerRun.run_status = RUNNING_SEARCH_HW;  // 标记数采状态为搜索华为设备
	}
	
	memset(uRecBuffer,0,256);
	iSearchResult = ComMasterReadDiscovery(&g_sMaster,0xD5,DISCOVERY_START,0,0,uRecBuffer,NULL);
	if(iSearchResult<0)
	{
        //DEBUGOUT("iSearchResult:%d\r\n",iSearchResult);
		if(RUNNING_WORK_READ == g_LoggerRun.run_status)  // 运行过程中搜索华为设备
		{	
			for(uint8_t uWorkSetAddr = 1;uWorkSetAddr < MAX_device+1;)				
			{
				uLen=strlen(uAllocation.cDeviceEsn[uWorkSetAddr]);				

				if(uLen != 0)
				{
					uTempAddr = EmptyAddr();
					msleep(1);
					if(0 != uTempAddr)
					{
						memcpy(uAllocation.cDeviceEsn[uTempAddr],uAllocation.cDeviceEsn[uWorkSetAddr],17);
						//memset(uAllocation.cDeviceEsn[uWorkSetAddr],0,18);
						uAllocation.cDeviceEsn[uTempAddr][17] = uTempAddr;			
						DEBUGOUT("\n[Working Allocation Addr]：%d ,[ESN]：%-17s\n",uAllocation.cDeviceEsn[uTempAddr][17],uAllocation.cDeviceEsn[uTempAddr]);
						msleep(1);
						iResult1B = DTEsnChange((uint8_t *)uAllocation.cDeviceEsn[uTempAddr]);
						if(iResult1B > 0)
						{
							memset(uRecBuffer,0,256);
							iAssignResult[uTempAddr] = ComMasterRead(&g_sMaster,0xD5,DISCOVERY_SET_ADDR,0,0,uRecBuffer,(uint8_t *)uAllocation.cDeviceEsn[uTempAddr]);
							gImport_Table_time = 0;
							if((iAssignResult[uTempAddr] > 0) && (uRecBuffer[1] == DISCOVERY_SET_ADDR) && (uRecBuffer[20] == uTempAddr) && (uRecBuffer[2] == 0x12))
							{
								DEBUGOUT("\nSuccess Allocation Addr is：%d\n",uRecBuffer[20]);
								g_DeviceEsn.uEsnMark[uTempAddr] = 1;
								memset(uAllocation.cDeviceEsn[uTempAddr],0,18);
								gImport_Table_time = OSTimeGet();
							}
							else
							{
								DEBUGOUT("\nFailed Allocation Addr is：%d\n",uRecBuffer[20]);
//								g_DeviceEsn.uEsnMark[uTempAddr] = 1;
								g_DeviceEsn.uEsnMark[uTempAddr] = 0;
								memset(uAllocation.cDeviceEsn[uTempAddr],0,18);
							}
						}
						else
						{
							DEBUGOUT("Device:%-17s Existing\n",uAllocation.cDeviceEsn[uTempAddr]);
						}
					}
				}
				uWorkSetAddr++;
		    }
			memset(&uAllocation,0,sizeof(uAllocation));
		}
		else if(RUNNING_SEARCH_HW == g_LoggerRun.run_status)// 新数采搜索华为设备
		{
			for(uint8_t uNewSetAddr = 1;uNewSetAddr < MAX_device+1;)
			{ 
				uLen=strlen(uAllocation.cDeviceEsn[uNewSetAddr]);
				if(uLen != 0)
				{
					uAllocation.cDeviceEsn[uNewSetAddr][17] = uNewSetAddr;
					DEBUGOUT("\n[Empty Device Allocation Addr]：%d ,[ESN]：%-17s\n",uAllocation.cDeviceEsn[uNewSetAddr][17],uAllocation.cDeviceEsn[uNewSetAddr]);
					memset(uRecBuffer,0,256);
					iAssignResult[uNewSetAddr] = ComMasterRead(&g_sMaster,0xD5,DISCOVERY_SET_ADDR,0,0,uRecBuffer,(uint8_t *)uAllocation.cDeviceEsn[uNewSetAddr]);
					gImport_Table_time = 0;
					if((iAssignResult[uNewSetAddr] >0 ) && (uRecBuffer[1] == DISCOVERY_SET_ADDR) && (uRecBuffer[20] == uNewSetAddr) && (uRecBuffer[2] == 0x12))
					{
						g_DeviceEsn.uEsnMark[uNewSetAddr] = 1;
						memset(uAllocation.cDeviceEsn[uNewSetAddr],0,18);
						DEBUGOUT("\nSuccess Allocation Addr is：%d \n",uRecBuffer[20]);
						gImport_Table_time = OSTimeGet();
					}
					else if(iAssignResult[uNewSetAddr]<0)
					{
						g_DeviceEsn.uEsnMark[uNewSetAddr] = 1;
						memset(uAllocation.cDeviceEsn[uNewSetAddr],0,18);
					}
				}
				uNewSetAddr++;
			}
			memset(&uAllocation,0,sizeof(uAllocation));
		}
	}
	uDTAddr++;
	return SEARCH_ING;
}
//===============================================================================
/******************************************************************************
* 名    称：YxReport()
* 功    能：突发遥信数据。
* 入口参数：
            uIecAddr  IEC104相对地址
            uIecVal   IEC104值
            uStep    1：暂时存数据，直到存满发送；2：只立即发送数据；3：存数据且立即发送数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
void YxReport(uint16_t uIecAddr,uint8_t uIecVal,uint8_t uStep)
{
    Uint_Char_Convert  uCTemp;

    if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        return;
    }

    do
    {
        if(g_sBurst.yx_count<60 &&  (1==uStep || 3==uStep)) // 一帧104报文，最多可以容纳60个遥信点
        {
            // 信息体地址
            uCTemp.u = uIecAddr + 1;
            g_sBurst.yx_data[g_sBurst.yx_count*4]   = uCTemp.c[0];
            g_sBurst.yx_data[g_sBurst.yx_count*4+1] = uCTemp.c[1];
            g_sBurst.yx_data[g_sBurst.yx_count*4+2] = 0x00;
            // 信息点值
            g_sBurst.yx_data[g_sBurst.yx_count*4+3] = uIecVal;
            // 计数点增加
            g_sBurst.yx_count++;

            if(1==uStep)
            {
                uStep = 0;
            }
        }

        if(g_sBurst.yx_count>=60 || (uStep>=2 && g_sBurst.yx_count))// 一帧104报文，最多可以容纳60个遥信点
        {
            // 一帧满先上传。
            g_sIecSend.format.maddrL = g_sBurst.yx_data[0];
            g_sIecSend.format.maddrM = g_sBurst.yx_data[1];
            g_sIecSend.format.maddrH = g_sBurst.yx_data[2];

            memcpy(g_sIecSend.format.data,&g_sBurst.yx_data[3],(g_sBurst.yx_count*4-3));// I帧数据
            IecCreateFrameI(M_SP_NA_1,g_sBurst.yx_count,R_BURST,(g_sBurst.yx_count*4-3),&g_sIecSend); // 字节数为信息点数*4 - 已经计数的第一个信息体地址

            // 重新开始记录
            g_sBurst.yx_count = 0x00;

            uStep = 0;
        }
        else
        {
            uStep = 0;
        }
    }while(uStep);
}
/******************************************************************************
* 名    称：YcReport()
* 功    能：突发遥测数据。
* 入口参数：
            uType     遥测点数据类型
            uIecAddr  IEC104相对地址
            fIecVal   IEC104值
            uStep    1：暂时存数据，直到存满发送；2：只立即发送数据；3：存数据且立即发送数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
void YcReport(uint8_t uType,uint16_t uIecAddr,uint32_t fIecVal,uint8_t uStep)
{
    Uint_Char_Convert  uCTemp;
    U32_F_Char_Convert yc_temp;

    if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        return;
    }

    do
    {
        if(g_sBurst.yc_count<30 &&  (1==uStep || 3==uStep)) // 一帧104报文，最多可以容纳30个遥测点
        {
            // 信息体地址
            uCTemp.u = uIecAddr + 0x4001;
            g_sBurst.yc_data[g_sBurst.yc_count*8]   = uCTemp.c[0];
            g_sBurst.yc_data[g_sBurst.yc_count*8+1] = uCTemp.c[1];
            g_sBurst.yc_data[g_sBurst.yc_count*8+2] = 0x00;
            // 信息点值
            yc_temp.u = fIecVal;


             
            //***********************************突发极大值置位全F值************            
            if(((0x4F800000==yc_temp.u)&&(uType == T_UINT32)) ||
                ((0x477FFF00==yc_temp.u)&&(uType == T_UINT16)))
            {
                g_sBurst.yc_data[g_sBurst.yc_count*8+3] = 0xFF;
                g_sBurst.yc_data[g_sBurst.yc_count*8+4] = 0xFF;
                g_sBurst.yc_data[g_sBurst.yc_count*8+5] = 0xFF;
                g_sBurst.yc_data[g_sBurst.yc_count*8+6] = 0xFF;
            }
            else
            {
                g_sBurst.yc_data[g_sBurst.yc_count*8+3] = yc_temp.c[0];
                g_sBurst.yc_data[g_sBurst.yc_count*8+4] = yc_temp.c[1];
                g_sBurst.yc_data[g_sBurst.yc_count*8+5] = yc_temp.c[2];
                g_sBurst.yc_data[g_sBurst.yc_count*8+6] = yc_temp.c[3];
            }

            
            g_sBurst.yc_data[g_sBurst.yc_count*8+7] = 0x00;  // 品质描述
            // 计数点增加
            g_sBurst.yc_count++;

            if(1==uStep)
            {
                uStep = 0;
            }
        }

        if(g_sBurst.yc_count>=30 || (uStep>=2 && g_sBurst.yc_count)) // 一帧104报文，最多可以容纳30个遥测点
        {
            // 一帧满先上传。
            g_sIecSend.format.maddrL = g_sBurst.yc_data[0];
            g_sIecSend.format.maddrM = g_sBurst.yc_data[1];
            g_sIecSend.format.maddrH = g_sBurst.yc_data[2];

            memcpy(g_sIecSend.format.data,&g_sBurst.yc_data[3],(g_sBurst.yc_count*8-3));// I帧数据
            IecCreateFrameI(M_ME_NC_1,g_sBurst.yc_count,R_BURST,(g_sBurst.yc_count*8-3),&g_sIecSend); // 字节数为信息点数*4 - 已经计数的第一个信息体地址

            // 重新开始记录
            g_sBurst.yc_count = 0x00;

            uStep = 0;
        }
        else
        {
            uStep = 0;
        }
    }while(uStep);
}

/******************************************************************************
* 名    称：SouthRecDeal()
* 功    能：南向数据处理。
* 入口参数：
*           无
* 出口参数：crc校验通过返回1，失败返回0
* 范    例:
******************************************************************************/
static uint8_t SouthRecDeal(void)
{
    uint8_t  i;
    uint8_t  uRelNum=0;
    uint16_t uIecAddr;
    uint16_t point=0;   // 信息点
    Uint_Char_Convert   uCTemp;
    U32_F_Char_Convert  uYcU;
    uint8_t uYxTemp;
    uint8_t uRelAddr;  // 设备相对地址


    if(0==uRecBuffer[0])
    {
        return 0;
    }
    uRelAddr = SlaveAddrConvert(uRecBuffer[0]);// 将通讯地址转化为相对地址
    //--------------------------------------------------------------------------
    // 标记丢帧的南向设备，重新连接
    if(uRelAddr < MAX_device)
    {
        if(g_LoggerRun.err_lost & (1<<uRelAddr))//if(g_LoggerAlarm.dev_lost & (1<<uRelAddr))
        {
            g_LoggerRun.err_lost &= ~(1<<uRelAddr);
            DEBUGOUT("设备%d连接\n",uRelAddr);
        }
    }
    //--------------------------------------------------------------------------

    switch(uRecBuffer[1])
    {
    case 0x03:
    case 0x04:
        //------------------------------------------------------------------------------------------------------
        // 不是正常工作模式 收到数据的通讯地址和发送的通讯地址不同  相对地址超限
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sSouth.uAddr) || (uRelAddr>=MAX_device))
        {
            return 0;
        }
        uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // 相对点表号

        switch(sSouth.uType)
        {
        case TYPE_YX:  // 发送查询遥信
            // 计算存储到IEC104表的起始下标
            if(g_DeviceSouth.device_inf[uRelAddr].yx_start_addr<1)
            {
                break;
            }

            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].yx_start_addr-1 + (sSouth.uYxCount - sSouth.uPointSum);
            point = sSouth.uPointHead;//Slave_sent.send_point_count;
            for(i=0; i<uRecBuffer[2];)
            {

                uYxTemp = uRecBuffer[i+4];
                //-----------------突发-------------------------
                if(NULL==IEC104_DATA_YX )   // IEC104表遥信指针为空
                {
                    DEBUGOUT("无遥信空间！");
                    break;
                }
                if(uIecAddr>(g_DeviceSouth.yx_sum-1))    // 遥信数据下标超过遥信总数
                {
                    DEBUGOUT("uIecAddr=%d yx_sum=%d",uIecAddr,g_DeviceSouth.yx_sum);
                    break;
                }
                if(uYxTemp != IEC104_DATA_YX[uIecAddr])
                {
                    IEC104_DATA_YX[uIecAddr] = uYxTemp;

                    YxReport(uIecAddr,uYxTemp,1);  // 突发遥信数据
                }
                //----------------------------------------------
                i += 2;
                uIecAddr++;
            }

            break;

        case TYPE_YC:  // 发送查询遥测
            // 计算存储到IEC104表的起始下标
            if(g_DeviceSouth.device_inf[uRelAddr].yc_start_addr<0x4001)
            {
                break;
            }

            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].yc_start_addr-0x4001 + (sSouth.uYcCount - sSouth.uPointSum);
            point = sSouth.uPointHead;//Slave_sent.send_point_count;

            for(i=0; i<uRecBuffer[2];)
            {
                if(1==g_pRegPoint[uRelNum][point].reg_count) // 数据长度1
                {
                    uYcU.u = 0; //yc_temp.f = 0;
                    uYcU.c[1] = uRecBuffer[i+3];
                    uYcU.c[0] = uRecBuffer[i+4];
                    i += 2;
                }
                else if(2==g_pRegPoint[uRelNum][point].reg_count) // 数据长度2
                {
                    uYcU.u = 0;
                    if(BIG_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // 大端模式
                    {
                        uYcU.c[3] = uRecBuffer[i+3];
                        uYcU.c[2] = uRecBuffer[i+4];
                        uYcU.c[1] = uRecBuffer[i+5];
                        uYcU.c[0] = uRecBuffer[i+6];
                    }
                    else if(LITTLE_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // 小端模式
                    {
                        uYcU.c[3] = uRecBuffer[i+5];
                        uYcU.c[2] = uRecBuffer[i+6];
                        uYcU.c[1] = uRecBuffer[i+3];
                        uYcU.c[0] = uRecBuffer[i+4];
                    }

                    i += 4;
                }
                if(((g_pRegPoint[g_DeviceSouth.device_inf[uRelAddr].rel_num][point].reg_type.type.data == T_UINT32) && (0xFFFFFFFF == uYcU.u)) ||
						((g_pRegPoint[g_DeviceSouth.device_inf[uRelAddr].rel_num][point].reg_type.type.data == T_UINT16) && (0xFFFF == uYcU.u16)))
				{
                	if((0x01&SouthSwich)&0x01)
					{
                		printf("ModbusAddr:0x%X Nothing 0xFFFFFFFF!!!\r\n",g_pRegPoint[g_DeviceSouth.device_inf[uRelAddr].rel_num][point].reg_addr);
					}
					uYcU.u = 0xFFFFFFFF;
                	uIecAddr++;
                	point++;
					continue;
				}
                //--------------数据类型转换-------------------------
                switch(g_pRegPoint[uRelNum][point].reg_type.type.data)
                {
                case T_UINT16:  // 无符号16位整型
                    uYcU.f = uYcU.u16;
                    break;

                case T_UINT32:  // 无符号32位整型
                    uYcU.f = uYcU.u;
                    break;

                case T_INT16:  // 有符号16位整型
                    uYcU.f = uYcU.i16;
                    break;

                case T_INT32:  // 有符号32位整型
                    uYcU.f = uYcU.i;
                    break;

                case T_FLOAT:  // 浮点类型
                  //uYcU.f = uYcU.f;
                    break;

                case T_EPOCHTIME:
                    if(0xFFFFFFFF==uYcU.u)
                    {
                        break;
                    }

                default:
                    uYcU.f = uYcU.u;
                    break;
                }

                //-----------------突发-------------------------
                if(NULL==IEC104_DATA_YC )   // IEC104表遥测指针为空
                {
                    DEBUGOUT("无遥测空间！");
                    break;
                }
                if(uIecAddr>(g_DeviceSouth.yc_sum-1))     //遥测数据下标超过遥测总数
                {
                    DEBUGOUT("uIecAddr=%d yc_sum=%d",uIecAddr,g_DeviceSouth.yc_sum);
                    break;
                }
                if(IEC104_DATA_YC[uIecAddr] != uYcU.u)//
                {
                    IEC104_DATA_YC[uIecAddr] = uYcU.u;//yc_temp.f;//

                    YcReport(g_pRegPoint[uRelNum][point].reg_type.type.data,uIecAddr,uYcU.u,1); // 突发遥测数据
                }

                uIecAddr++;
                point++;
            }
            break;

        case TYPE_DD:  // 发送查询电度
            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].dd_start_addr-0x6401 + (sSouth.uDdCount - sSouth.uPointSum);
            point = sSouth.uPointHead;//Slave_sent.send_point_count;
            break;

        case TYPE_GJ:  // 读告警点
            point = sSouth.uPointHead;
            for(i=0; i<uRecBuffer[2];)
            {
                uCTemp.u = 0;
                //if(BIG_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // 大端模式
                {
                    uCTemp.c[1] = uRecBuffer[i+3];
                    uCTemp.c[0] = uRecBuffer[i+4];
                }
                /*else if(LITTLE_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // 小端模式
                {
                    uCTemp.c[1] = uRecBuffer[i+4];
                    uCTemp.c[0] = uRecBuffer[i+3];
                }*/

                //-------------------------------------------------------------------
               if(NORTH_OK==g_LoggerRun.north_status)  // 连接服务器完成
                {
                    uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // 相对点表号

                    AlarmAddDev(uRelAddr,
                                g_DeviceSouth.protocol[uRelNum].alarm_sum,
                                (sSouth.uRegAddr+i/2),
                                uCTemp.u);
                }
                //-------------------------------------------------------------------
                i += 2;
            }
            break;

        default:
            break;
        }
        break;
        //------------------------------------------------------------------------------------------------------
    case 0x83:
        // 不是正常工作模式 收到数据的通讯地址和发送的通讯地址不同  相对地址超限
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sSouth.uAddr) || (uRelAddr>=MAX_device))
        {
            return 0;
        }
        if(65534 != sSouth.uRegAddr)
        {
            break;
        }
        switch(uRecBuffer[2])
        {
        case 0x02:  // 非法地址
            // 计算存储到IEC104表的起始下标
            if(g_DeviceSouth.device_inf[uRelAddr].yx_start_addr<1)
            {
                break;
            }
            if(0 == g_DeviceSouth.device_inf[uRelAddr].addr)
            {
                break;
            }
            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].yx_start_addr-1 + (sSouth.uYxCount - sSouth.uPointSum);

            YxReport(uIecAddr,0x01,1);  // 华为逆变器点表的65534点

            uIecAddr++;
            break;

//		case 0x06:  // 非法地址
//
//			break;
        }
        break;
    default:
        break;
    }
    return 0;
}

/****************************************************************************
* 名    称：SouthInquire()
* 功    能：正常南向查询
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
#define SOUTH_EMPTY    (0)   // 设备空
#define SOUTH_WAIT     (1)   // 等待设备,可能在导表过程中
#define SOUTH_OVER     (2)   // 一个循环完成
#define SOUTH_WORK     (3)   // 工作中
int8_t SouthInquire(void)
{
    static uint8_t  uRelativeAddr = 0;     // 南向设备相对地址
    static uint8_t  uNextDev = 0;          // 查询下一台设备标记

    uint16_t uMessPointSum;               // 信息点总数
    int16_t  iReadResult;                 // 发起查询结果反馈
    uint8_t  uRelativePointNum = 0;         // 相对点表号
    uint8_t  i;


    DEVICE_COM_INFO_T     sCom;    // 南向查询寄存器信息

    if(RUNNING_EMPTY == g_LoggerRun.run_status)  // 新数采，数据信息为空
    {
        return SOUTH_EMPTY;
    }
    if(RUNNING_WORK_READ != g_LoggerRun.run_status)
    {
        return SOUTH_WAIT;
    }

    //while(1)
    {
        if(uNextDev&0x01)
        {
            uNextDev = 0;

            sSouth.uAddr       = 0;
            sSouth.uFun        = 0;
            sSouth.uRegAddr    = 0;
            sSouth.uRegCount   = 0;
            sSouth.uType       = 0;
            sSouth.uPointCount = 0;
            sSouth.uYcCount    = 0;
            sSouth.uYxCount    = 0;
            sSouth.uDdCount    = 0;
            sSouth.uPointHead  = 0;
            sSouth.uPointSum   = 0;

            do
            {
                uRelativeAddr++;
                 // 相对地址超过设备总数  g_DeviceSouth.device_sum  ,在添加设备，删除设备等操作后可能出现 uRelativeAddr大于设备总数的情况
                if(uRelativeAddr >= MAX_device)
                {
                    break;
                }
                if(0 != g_DeviceSouth.device_inf[uRelativeAddr].addr)  // 设备通讯地址有效，不为0；为0可能设备被删除
                {
                    break;
                }

            }while(1);

            if(uRelativeAddr >= MAX_device)   // 相对地址超过设备总数，一个循环查询完成
            {
                uRelativeAddr = 0;

                if(0 == g_DeviceSouth.device_inf[uRelativeAddr].addr)
                {
                    uNextDev = 1;
                }

                //---------------判断是否有突发数据没有上报--------------------------------
                if(NORTH_OK == g_LoggerRun.north_status)  // 连接服务器完成
                {

//                    DEBUGOUT("************************ yc_count: %d ********************************\n", g_sBurst.yc_count);
                    if(g_sBurst.yc_count)
                    {
                        YcReport(0,0,0,2);// 发送可能暂存的遥测突发数据
                    }
                    if(g_sBurst.yx_count)
                    {
                        YxReport(0,0,2);// 发送可能暂存的遥信突发数据
                    }
                }
                //-------------------------------------------------------------------
                return SOUTH_OVER;
            }
        }
        //--------------------------------------------------------------------------

        if(0 == g_DeviceSouth.device_inf[uRelativeAddr].addr)
        {
            uNextDev = 1;
            return SOUTH_WORK;
        }

        uRelativePointNum = g_DeviceSouth.device_inf[uRelativeAddr].rel_num;  // 相对点表号

        if((g_pRegPoint[uRelativePointNum] != NULL) && (g_DeviceSouth.device_inf[uRelativeAddr].protocol_num == g_DeviceSouth.protocol[uRelativePointNum].protocol_num)) // 指向点表数组的指针
        {
            // 信息点总数
            uMessPointSum = g_DeviceSouth.protocol[uRelativePointNum].mess_point_sum;

            sCom.uRegCount = 0;  // 寄存器数量初始化为0；


            for(; sSouth.uPointCount < uMessPointSum; sSouth.uPointCount++)
            {
                sSouth.uType = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_type.type.mess;

                if(TYPE_YX == sSouth.uType)//YX:0x02  YC:0x01  YK:0x04  SD:0x05   DD:0x03
                {
                    sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // 寄存器地址

                    sSouth.uYxCount++;
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;


                    sSouth.uPointHead = sSouth.uPointCount;
                    sSouth.uPointCount++;
                    sSouth.uPointSum = 1;
                    break;
                }
                else if(TYPE_YC == sSouth.uType)//0x01
                {
                    sSouth.uYcCount++;

                    if(g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count <= 2)// 信息点长度大于2，不查询
                    {
                        sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // 寄存器地址
                        sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                        sSouth.uPointHead = sSouth.uPointCount;
                        sSouth.uPointSum = 1;
                        sSouth.uPointCount++;
                    }
                    else
                    {
                        continue;
                    }
                    break;
                }
                else if(TYPE_DD == sSouth.uType)
                {
                    sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // 寄存器地址

                    sSouth.uDdCount++;
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                    sSouth.uPointHead = sSouth.uPointCount;
                    sSouth.uPointCount++;
                    sSouth.uPointSum = 1;
                    break;
                }
              else if(TYPE_GJ == sSouth.uType)
                {
                    sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // 寄存器地址
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                    sSouth.uPointHead = sSouth.uPointCount;
                    sSouth.uPointCount++;
                    sSouth.uPointSum = 1;
                    break;
                }
            }

            for(i = sSouth.uPointCount; i < uMessPointSum; i++)
            {
                if((sSouth.uType == g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_type.type.mess)  // 信息类型相同
                   && ((g_pRegPoint[uRelativePointNum][sSouth.uPointCount-1].reg_addr + g_pRegPoint[uRelativePointNum][sSouth.uPointCount-1].reg_count) == g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr) // 前一个点与后一个点地址连续
                   && (sSouth.uPointCount>0) 
                   && (g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count <= 2)  // 信息点长度不大于2
                   )
                {
                    if((sCom.uRegCount + g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count) <= 125)
                    {
                        sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                        if(TYPE_YX == sSouth.uType)
                        {
                            sSouth.uYxCount++;
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                        else if(TYPE_YC == sSouth.uType)
                        {
                            sSouth.uYcCount++;
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                        else if(TYPE_DD == sSouth.uType)
                        {
                            sSouth.uDdCount++;
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                        else if(TYPE_GJ == sSouth.uType)
                        {
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            uNextDev = 1;
        }

//        printf("uRegCount:%d!!!\r\n",sCom.uRegCount);
        if(sCom.uRegCount > 0)
        {
            sSouth.uRegAddr  = sCom.uRegAddr;       // 寄存器地址

            if(0x01 == g_DeviceSouth.protocol[uRelativePointNum].protocol_type)// if(0x01==g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // 华为能源MODBUS
            {
                sCom.uFun = 0x03;
            }
            else if(0x02 == g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // 标准MODBUS
            {
                if(sCom.uRegAddr >= 40000)   // 读数据保持寄存器40001~49999
                {
                    sCom.uFun = 0x03;
                    sCom.uRegAddr -= 40000;
                }
                else if(sCom.uRegAddr >= 30000)  // 读输入寄存器30001~39999
                {
                    sCom.uFun = 0x04;
                    sCom.uRegAddr -= 30000;
                }
                else
                {
                    sCom.uFun = 0x03;
                }
            }
            // 数据大小端
            /*if(BIG_ENDIAN==g_DeviceSouth.device_inf[uRelativeAddr].big_little_endian)
            {
                sCom.endian = 0;   // 大端
            }
            else
            {
                sCom.endian = 1;   // 小端
            }*/
            // 通讯波特率
            sSouth.uBaudRate = SetBaudRate(sSouth.uBaudRate,g_DeviceSouth.device_inf[uRelativeAddr].baud_rate);

            sCom.uDevAddr = g_DeviceSouth.device_inf[uRelativeAddr].addr;  // 实际的通讯地址

            sSouth.uAddr          = sCom.uDevAddr;      // 绝对地址
            sSouth.uFun           = sCom.uFun;          // 发送的功能指令
            sSouth.uRegCount      = sCom.uRegCount;     // 查询寄存器个数

            iReadResult = ComMasterRead(&g_sMaster,sCom.uDevAddr,sCom.uFun,sCom.uRegAddr,sCom.uRegCount,uRecBuffer,NULL);
            if((iReadResult > 0) || (-MODBUS_ILLEGAL_ADDR == iReadResult) || (-MODBUS_ILLEGAL_BUSY == iReadResult))  // 接收正常或0x83异常码为2
            {
                SouthRecDeal();  // 接收数据处理
                //PrintHex(01,uRecBuffer,iReadResult);
                /*for(i=0;i<iReadResult;i++)
                {
                    DEBUGOUT("%02X ",uRecBuffer[i]);
                }*/
            }
            else if(-MASTER_lOST == iReadResult)  // 丢帧
            {
                if((NORTH_OK == g_LoggerRun.north_status) && (RUNNING_WORK_READ == g_LoggerRun.run_status))  // 连接服务器完成
                {
                    if(0 == (g_LoggerRun.err_lost & (1<<uRelativeAddr)))
                    {
                        ResetIecData(uRelativeAddr);
                        g_LoggerRun.err_lost |= (1<<uRelativeAddr);  // 标记丢帧的南向设备
                        DEBUGOUT("设备%d断连\n",uRelativeAddr);
                    }
                }
                uNextDev = 1;  // 本设备通讯丢帧，跳过本设备
            }
            else if(-MASTER_CRC == iReadResult)  // 丢帧
            {
                DEBUGOUT("CRC错误\n");
            }
        }
        //----------------------------------------------------
        if((0 == uNextDev) && (sSouth.uPointCount >= uMessPointSum))
        {
            uNextDev = 1;
        }
    }
    return SOUTH_WORK;
}
/******************************************************************************
* 名    称：SouthRecDeal()
* 功    能：南向数据处理。
* 入口参数：
*           无
* 出口参数：crc校验通过返回1，失败返回0
* 范    例:
******************************************************************************/
static uint8_t SouthRecDealAlarm(void)
{   
    uint8_t  i;
    uint8_t  uRelNum = 0;
//    uint16_t point = 0;   // 信息点
    Uint_Char_Convert   uCTemp;
    uint8_t uRelAddr;  // 设备相对地址

    if(0 == uRecBuffer[0])
    {
        return 0;
    }
    uRelAddr = SlaveAddrConvert(uRecBuffer[0]);// 将通讯地址转化为相对地址	
	// 标记丢帧的南向设备，重新连接
	if(uRelAddr < MAX_device)
	{
		if(g_LoggerRun.err_lost & (1<<uRelAddr))//if(g_LoggerAlarm.dev_lost & (1<<uRelAddr))
		{
			g_LoggerRun.err_lost &= ~(1<<uRelAddr);
			DEBUGOUT("设备%d连接\n",uRelAddr);
		}
	}
	
    switch(uRecBuffer[1])
    {
    case 0x03:
    case 0x04:
        // 不是正常工作模式 收到数据的通讯地址和发送的通讯地址不同  相对地址超限
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sAlarmSouth.uAddr) || (uRelAddr >= MAX_device))
        {
            return 0;
        }
        uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // 相对点表号
        
        switch(sAlarmSouth.uType)
        {
       
        case TYPE_GJ:  // 读告警点
//            point = sAlarmSouth.uPointHead;
			
            for(i = 0; i < uRecBuffer[2];)
            {
                uCTemp.u = 0;
                uCTemp.c[1] = uRecBuffer[i+3];
                uCTemp.c[0] = uRecBuffer[i+4];
                //-------------------------------------------------------------------
                if(NORTH_OK == g_LoggerRun.north_status)  // 连接服务器完成
                {
                    uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // 相对点表号
                    AlarmAddDev(uRelAddr,
                                g_DeviceSouth.protocol[uRelNum].alarm_sum,
                                (sAlarmSouth.uRegAddr+i/2),
                                uCTemp.u);
                }
                //-------------------------------------------------------------------
                i += 2;
            }
            break;
        default:
            break;
        }
        break; 
     case 0x83:
        // 不是正常工作模式 收到数据的通讯地址和发送的通讯地址不同  相对地址超限
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sAlarmSouth.uAddr) || (uRelAddr >= MAX_device))
        {
            return 0;
        }
        if(65534 != sAlarmSouth.uRegAddr)
        {
            break;
        }
		break;
    default:
        break;
    }
    return 0;
}
/****************************************************************************
* 名    称：SouthInquire()
* 功    能：正常南向查询
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
//#define SOUTH_ALARM_EMPTY    (0)   // 设备空
//#define SOUTH_ALARM_WAIT     (1)   // 等待设备,可能在导表过程中
//#define SOUTH_ALARM_OVER     (2)   // 一个循环完成
//#define SOUTH_ALARM_WORK     (3)   // 工作中
int8_t SouthInquireAlarm(void)
{
    static uint8_t  uRelativeAddr=0;     // 南向设备相对地址
    static uint8_t  uNextDev=0;          // 查询下一台设备标记
    uint16_t uMessPointSum;               // 信息点总数
    int16_t  iReadResult;                 // 发起查询结果反馈
    uint8_t  uRelativePointNum=0;         // 相对点表号
    uint8_t  i;
    DEVICE_ALARM_COM_INFO_T     sCom;    // 南向查询寄存器信息
    
    if(RUNNING_EMPTY==g_LoggerRun.run_status)  // 新数采，数据信息为空
    {
        return SOUTH_EMPTY;
    }
    if(RUNNING_WORK_READ!=g_LoggerRun.run_status)
    {
        return SOUTH_WAIT;
    }
    //while(1)
    {
        if(uNextDev&0x01)
        {
            uNextDev = 0;
            sAlarmSouth.uAddr       = 0;
            sAlarmSouth.uFun        = 0;
            sAlarmSouth.uRegAddr    = 0;
            sAlarmSouth.uRegCount   = 0;
            sAlarmSouth.uType       = 0;
            sAlarmSouth.uPointCount = 0;
            sAlarmSouth.uYcCount    = 0;
            sAlarmSouth.uYxCount    = 0;
            sAlarmSouth.uDdCount    = 0;
            sAlarmSouth.uPointHead  = 0;
            sAlarmSouth.uPointSum   = 0;

            do
            {
                uRelativeAddr++;
                 // 相对地址超过设备总数  g_DeviceSouth.device_sum  ,在添加设备，删除设备等操作后可能出现 uRelativeAddr大于设备总数的情况
                if(uRelativeAddr>=MAX_device)
                {
                    break;
                }
                if(0!=g_DeviceSouth.device_inf[uRelativeAddr].addr)  // 设备通讯地址有效，不为0；为0可能设备被删除
                {
                    break;
                }

            }while(1);

            if(uRelativeAddr>=MAX_device)   // 相对地址超过设备总数，一个循环查询完成Device_south.device_sum
            {
                uRelativeAddr = 0;

                if(0==g_DeviceSouth.device_inf[uRelativeAddr].addr)
                {
                    uNextDev = 1;
                }
                return SOUTH_OVER;
            }
        }
        //--------------------------------------------------------------------------
        if(0==g_DeviceSouth.device_inf[uRelativeAddr].addr)
        {
            uNextDev = 1;
            return SOUTH_WORK;
        }

        uRelativePointNum = g_DeviceSouth.device_inf[uRelativeAddr].rel_num;  // 相对点表号
        if((g_pRegPoint[uRelativePointNum]!=NULL) && (g_DeviceSouth.device_inf[uRelativeAddr].protocol_num==g_DeviceSouth.protocol[uRelativePointNum].protocol_num)) // 指向点表数组的指针
        {
            // 信息点总数
            uMessPointSum = g_DeviceSouth.protocol[uRelativePointNum].mess_point_sum;
            sCom.uRegCount = 0;  // 寄存器数量初始化为0；
            for(; sAlarmSouth.uPointCount<uMessPointSum; sAlarmSouth.uPointCount++)
            {
                sAlarmSouth.uType = g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_type.type.mess;
                if(TYPE_GJ==sAlarmSouth.uType)
                {
					sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_addr;  // 寄存器地址
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count;
                    sAlarmSouth.uPointHead = sAlarmSouth.uPointCount;
                    sAlarmSouth.uPointCount++;
                    sAlarmSouth.uPointSum = 1;
                    break;
                }
            }

            for(i=sAlarmSouth.uPointCount; i<uMessPointSum; i++)
            {
                if((sAlarmSouth.uType==g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_type.type.mess) // 信息类型相同
					&& ((g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount-1].reg_addr+g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount-1].reg_count) == g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_addr)  // 前一个点与后一个点地址连续
                    && (sAlarmSouth.uPointCount>0) 
                    && (g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count<=2)  // 信息点长度不大于2
                   )
                {
                    if((sCom.uRegCount + g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count) <= 125)
                    {
                        sCom.uRegCount += g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count;

                        if(TYPE_GJ==sAlarmSouth.uType)
                        {
                            sAlarmSouth.uPointSum++;
                            sAlarmSouth.uPointCount++;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            uNextDev = 1;
        }

        if(sCom.uRegCount>0)
        {
			sAlarmSouth.uRegAddr  = sCom.uRegAddr;       // 寄存器地址

            if(0x01==g_DeviceSouth.protocol[uRelativePointNum].protocol_type)// if(0x01==g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // 华为能源MODBUS
            {
                sCom.uFun = 0x03;
            }
            else if(0x02==g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // 标准MODBUS
            {
                if(sCom.uRegAddr>=40000)   // 读数据保持寄存器40001~49999
                {
                    sCom.uFun = 0x03;
                    sCom.uRegAddr -= 40000;
                }
                else if(sCom.uRegAddr>=30000)  // 读输入寄存器30001~39999
                {
                    sCom.uFun = 0x04;
                    sCom.uRegAddr -= 30000;
                }
                else
                {
                    sCom.uFun = 0x03;
                }
            }

            // 通讯波特率
            sAlarmSouth.uBaudRate = SetBaudRate(sAlarmSouth.uBaudRate,g_DeviceSouth.device_inf[uRelativeAddr].baud_rate);
            sCom.uDevAddr = g_DeviceSouth.device_inf[uRelativeAddr].addr;  // 实际的通讯地址
            sAlarmSouth.uAddr = sCom.uDevAddr;      // 绝对地址
            sAlarmSouth.uFun = sCom.uFun;          // 发送的功能指令
            sAlarmSouth.uRegCount = sCom.uRegCount;     // 查询寄存器个数
            iReadResult = ComMasterRead(&g_sMaster,sCom.uDevAddr,sCom.uFun,sCom.uRegAddr,sCom.uRegCount,uRecBuffer,NULL);
            if(iReadResult>0 || -MODBUS_ILLEGAL_ADDR==iReadResult)  // 接收正常或0x83异常码为2
            {
                SouthRecDealAlarm();  // 接收数据处理
            }
            else if(-MASTER_lOST==iReadResult)  // 丢帧
            {
                if(NORTH_OK==g_LoggerRun.north_status && RUNNING_WORK_READ==g_LoggerRun.run_status)  // 连接服务器完成
                {
                    if(0==(g_LoggerRun.err_lost & (1<<uRelativeAddr)))
                    {
                        ResetIecData(uRelativeAddr);
                        g_LoggerRun.err_lost |= (1<<uRelativeAddr);  // 标记丢帧的南向设备
                        DEBUGOUT("设备%d断连\n",uRelativeAddr);
                    }
                }
                uNextDev = 1;  // 本设备通讯丢帧，跳过本设备
            }
            else if(-MASTER_CRC==iReadResult)  // 丢帧
            {
                DEBUGOUT("CRC错误\n");
            }
        }
		
        if(0==uNextDev && (sAlarmSouth.uPointCount>=uMessPointSum))
        {
            uNextDev = 1;
        }
    }
    return SOUTH_WORK;
}

/****************************************************************************
* 名    称：DT1000ReadDataflashData
* 功    能：将表计升级数据从DataFlash读出
* 入口参数：
*           nFrame   帧序号，从0开始
*           *pData   数据指针
*           nLen     数据长度
* 出口参数：存储成功返回读取的字节数；最后一帧最高位置1；无数据返回0
* 范    例：无
* 作    者：WQY
****************************************************************************/
uint16_t DT1000ReadDataflashData(const uint16_t nFrame,uint8_t *pData, const uint8_t nLen)
{
    static uint32_t nAllDataLen=0;
    uint32_t nDataFalshAddr; // DataFlash数据存储地址
    
    uint8_t nReadLen=0;
    
    uint8_t nNeedLen=nLen;

    if(0==nAllDataLen)
    {
        nAllDataLen = g_DT1000Updata.nDataLen;
    }

    nDataFalshAddr = DATAFLASH_DT1000_UPDATA_HEAD + (nFrame & 0x7fff) * nLen; // 256为每帧数据长度
   
    
    if(nAllDataLen<=nLen)
    {
        nNeedLen = nAllDataLen;
    }

    nReadLen = DataFlash_Read(nDataFalshAddr, pData, nNeedLen);
	if(nReadLen<=3)
	{
		nReadLen = DataFlash_Read(nDataFalshAddr, pData, nNeedLen);
	}
	msleep(5);
	DEBUGOUT("ReadLen:%d\n",nReadLen);

    nAllDataLen = nAllDataLen - nReadLen;
	//DEBUGOUT("nAllDataLen:%d\r\n",nAllDataLen);

    /*if(0==nAllDataLen)  // 所有数据都读完
    {
        nReadLen = nReadLen | 0x8000;
    }*/
    
    return nReadLen;
}
/****************************************************************************
* 名    称：SouthBroadcastUpdata()
* 功    能：南向升级广播任务
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
int8_t SouthBroadcastUpdata(void)
{
	uint8_t  *pTmpData = NULL;
	uint16_t check_crc;
	uint16_t uTmpCrc;
	uint16_t s_uSendseq=0;//s_uSendAll=0; // 接收序号
	uint8_t uReadDataLen=0;   //读取数据长度
	uint8_t uFrameLen = 240;
	U32_F_Char_Convert uTemp;
	//====================================================================================
	//先广播升级文件信息
	//====================================================================================
	//memset(uTmpData,0,250);

	pTmpData = (uint8_t *)WMemMalloc(pTmpData,27*sizeof(uint8_t));
	pTmpData[0] = 0xFF;
	pTmpData[1] = DT1000UPDATE_START;
	pTmpData[2] = 0x01; 	 //子功能码升级0x01
	pTmpData[3] = 0x16; 	 //文件信息长度

	memcpy(&pTmpData[4],g_DT1000Updata.version,17);  //升级表计版本号
	
	uTemp.u = g_DT1000Updata.nDataLen;
	pTmpData[21] = uTemp.c[3];
	pTmpData[22] = uTemp.c[2];
	pTmpData[23] = uTemp.c[1];
	pTmpData[24] = uTemp.c[0];
	
	pTmpData[25] = uFrameLen; //每帧数据长度
	
	uTmpCrc = CRC16(pTmpData,26);
	pTmpData[26] = uTmpCrc>>8;
	pTmpData[27] = uTmpCrc&0XFF;
	
	UartWrite(SLAVE_UART_USE,pTmpData,28);
	FEED_DOG();
	//DEBUGOUT("[数采]RAM Pool Free %d Bytes\n",FreeRamSpace());

	/*if(0x01&SouthSwich)
	{
		DEBUGOUT("SOUTH_UPDATA:");
		for(uint8_t i=0;i<27;i++)
		{
			DEBUGOUT("%02X ",uTmpData[i]);
		}
		DEBUGOUT("\r\n");
	}*/
	
	DEBUGOUT("Upgrade Vertion：%17s File Len：%d\n",&pTmpData[4],g_DT1000Updata.nDataLen);
    
	sleep(3);
	
	//====================================================================================
	//升级数据传输
	//====================================================================================
	for(;(s_uSendseq*uFrameLen) < g_DT1000Updata.nDataLen;s_uSendseq++)
	{
		//memset(uTmpData,0,256);
		pTmpData = (uint8_t *)WMemMalloc(pTmpData,248*sizeof(uint8_t));
		
		pTmpData[0] = 0xFF;
		pTmpData[1] = DT1000UPDATE_DATA;
		pTmpData[2] = 0x01; 	 //子功能码升级0x01
		
		pTmpData[3] = s_uSendseq>>8;
		pTmpData[4] = s_uSendseq&0XFF;
	   
		uReadDataLen =DT1000ReadDataflashData(s_uSendseq,&pTmpData[6],uFrameLen);
		if(0 == uReadDataLen)
		{
			DEBUGOUT("Flash Read Fail!!!\n");
			return 0;
		}
		pTmpData[5] = uReadDataLen;  //数据长度
		
		check_crc = CalculateCRC(&pTmpData[6],uReadDataLen);
		//pTmpData[uReadDataLen+6] = check_crc>>8;
		//pTmpData[uReadDataLen+7] = check_crc&0XFF;
		uTmpCrc = CRC16(pTmpData,uReadDataLen+6);
		pTmpData[uReadDataLen+6]=uTmpCrc>>8;
		pTmpData[uReadDataLen+7]=uTmpCrc&0XFF;
		
		UartWrite(SLAVE_UART_USE,pTmpData,uReadDataLen+8);
		FEED_DOG();
		DEBUGOUT("\nSouth Upgrade %d Frame\n",s_uSendseq);
		/*if(0x01&SouthSwich)
		{
			DEBUGOUT("SOUTH_UPDATA:");
			for(uint8_t i=0;i<(uReadDataLen+10);i++)
			{
				DEBUGOUT("%02X ",uTmpData[i]);
			}
			DEBUGOUT("\r\n");
		}*/

		sleep(1);
	}
	
	//====================================================================================
	//广播升级结束信息!!!
	//====================================================================================
	//memset(pTmpData,0,250);

	pTmpData = (uint8_t *)WMemMalloc(pTmpData,10*sizeof(uint8_t));
	
	pTmpData[0]=0xFF;
	pTmpData[1]=DT1000UPDATE_END;
	pTmpData[2]=0x01;  //子功能码升级0x01
	pTmpData[3]=0x04;  //长度
	pTmpData[4]=s_uSendseq>>8;
	pTmpData[5]=s_uSendseq&0XFF;	//发送帧总数
	pTmpData[6]=check_crc>>8;
	pTmpData[7]=check_crc&0XFF;
	uTmpCrc = CRC16(pTmpData,8);
	pTmpData[8]=uTmpCrc>>8;
	pTmpData[9]=uTmpCrc&0XFF;
	
	UartWrite(SLAVE_UART_USE,pTmpData,10);
	DEBUGOUT("Broadcast Upgrade End!!!\n");
	pTmpData=WMemFree(pTmpData);
   /* if(0x01&SouthSwich)
	{
		DEBUGOUT("SOUTH_UPDATA:");
		for(uint8_t i=0;i<10;i++)
		{
			DEBUGOUT("%02X ",uTmpData[i]);
		}
		DEBUGOUT("\r\n");
	}*/
	//====================================================================================

	return 0;
}

/****************************************************************************
* 名    称：SouthUpdataSoftChange()
* 功    能：南向升级软件变更
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
void SouthUpdataSoftChange(void)
{
	int16_t	iResult[MAX_device];			// 发起查询结果反馈
	for(uint8_t i=0;i<g_DeviceSouth.device_sum;i++)
	{
	    memset(uRecBuffer,0,256);
		
		iResult[i] = ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[i].addr,DISCOVERY_REPORT,0,0,uRecBuffer,NULL);
		if(iResult[i]>0)
		{
            HwSoftChange(g_DeviceSouth.device_inf[i].addr,uRecBuffer);
			sleep(2);	
		}
	}
}
/****************************************************************************
* 名    称：SouthSingleUpdata()
* 功    能：南向升级任务
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
void SouthSingleUpdata(void)
{
	int16_t	iResult[MAX_device];			// 发起查询结果反馈
    uint16_t s_uSendseq=0; // 接收序号
    uint8_t uReadDataLen=0;   //读取数据长度
    uint8_t uTmpAddr;
	uint8_t  uTmpData[256];

	
	for(uint8_t i=0;i<g_DeviceSouth.device_sum;i++)
	{
		uReadDataLen=0;
        FEED_DOG();     // 喂狗
	    if(0!=memcmp(g_DeviceSoft.cDeviceSoft[i],g_DT1000Updata.version,17))
		{
			uTmpAddr = g_DeviceSouth.device_inf[i].addr;

			DEBUGOUT("Sigle Upgrade Device Addr：%d\n",uTmpAddr);
			memset(uTmpData,0,256);
			memcpy(&uTmpData,&g_DT1000Updata.version,17);
            msleep(1);
			memset(uRecBuffer,0,256);
			
			iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,DT1000UPDATE_START,0,0,uRecBuffer,uTmpData);
			if((iResult[uTmpAddr]>0))
            {
                sleep(3);
				for(;(s_uSendseq*240) < g_DT1000Updata.nDataLen;s_uSendseq++)
				{
                    memset(uTmpData,0,256);
                    FEED_DOG();     // 喂狗
					uTmpData[0] = s_uSendseq>>8;
	                uTmpData[1] = s_uSendseq&0XFF;

	                uReadDataLen =DT1000ReadDataflashData(s_uSendseq,&uTmpData[3],240);
					msleep(1);
	                uTmpData[2] = uReadDataLen;  //数据长度

                    memset(uRecBuffer,0,256);
					iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,DT1000UPDATE_DATA,0,uReadDataLen+3,uRecBuffer,uTmpData);

					if(iResult[uTmpAddr]>0)
					{

						DEBUGOUT("\nSigle Upgrade: %d Frame\r\n",s_uSendseq);
					}else
					{
						DEBUGOUT("\nSigle Upgrade: %d Frame Fail\r\n",s_uSendseq);
						DEBUGOUT("\nQuit Sigle Upgrade：%d!!!\r\n",uTmpAddr);
						break;
					}
					sleep(1);
				}
				
				sleep(1);
				
				memset(uTmpData,0,256);
				uTmpData[0] = s_uSendseq>>8;
	            uTmpData[1] = s_uSendseq&0XFF;
				
				memset(uRecBuffer,0,256);
				iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,DT1000UPDATE_END,0,0,uRecBuffer,uTmpData);
				if(iResult[uTmpAddr]>0)
				{
					DEBUGOUT("\nSigle Upgrade End!!!\r\n");
					
					//SouthUpdataSoftChange();
					//memset(uRecBuffer,0,256);
					//FEED_DOG();     // 喂狗
					//sleep(30);
					/*iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,0x3B,0,0,uRecBuffer,NULL);
					if(uTmpAddr > 0)
					{
						HwSoftChange(uTmpAddr,uRecBuffer);
					}
					
					sleep(1);*/
				}
			}
			s_uSendseq=0;
			sleep(3);
		}
	}
}

/****************************************************************************
* 名    称：SouthLogStart()
* 功    能：南向日志开始
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
uint32_t SouthLogStart(void)
{
    int16_t iResult;
	uint8_t uTmpData[2];
	U32_F_Char_Convert uTemp;

	uTmpData[0] = S_FILE_EXPORT;  //文件导出子功能码
	uTmpData[1] = g_DT1000Updata.uData;   //导出日期
	iResult = ComMasterRead(&g_sMaster,g_DT1000Updata.uDevAddr,DT1000LOG_START,0,0,uRecBuffer,uTmpData);
	
	if((iResult > 0) && (uRecBuffer[1]== DT1000LOG_START))
	{
        if(0 == uRecBuffer[4]) //判断表计回复计算文件大小是否完成，0x01表示表计计算文件大小完成并反馈；0x00表示计算文件大小未完成反馈
        {
             return 0;
			 
		}else
		{    
		     uTemp.c[0] = uRecBuffer[8];
			 uTemp.c[1] = uRecBuffer[7];
			 uTemp.c[2] = uRecBuffer[6];
			 uTemp.c[3] = uRecBuffer[5];
		     g_DT1000Updata.nDataLen = uTemp.u;
			 DEBUGOUT("\nLogFileLen: %d    ",g_DT1000Updata.nDataLen);
			 return  g_DT1000Updata.nDataLen;
		}
	}

	return 0;
}
/****************************************************************************
* 名    称：SouthLogTransmission()
* 功    能：南向日志数据传输
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
uint8_t SouthLogTransmission(uint16_t uFrameCount)
{
    int16_t  iResult;
	uint8_t uTmpData[4];
	uint8_t uDataLen;
	Uint_Char_Convert uTemp;
	
	g_DT1000Updata.CRC=0xFFFF;
    FEED_DOG();
	
    uTmpData[0] = S_FILE_EXPORT;   //文件导出类型
    uTmpData[1] = g_DT1000Updata.uData;  //日期
    uTmpData[2] = uFrameCount>>8;   //包序号高位
  	uTmpData[3] = uFrameCount&0xFF;  //包序号低位
  	iResult = ComMasterRead(&g_sMaster,g_DT1000Updata.uDevAddr,DT1000LOG_DATA,0,0,uRecBuffer,uTmpData);
	
	if((iResult > 0) && (uRecBuffer[1] == DT1000LOG_DATA))
	{
         uTemp.c[0] = uRecBuffer[5];
		 uTemp.c[1] = uRecBuffer[4];
		 
		 if(uTemp.u == uFrameCount)           //发送与接收的包序号对比，正确则转发给平台
		 { 
//			  DEBUGOUT("\nSend Package : %d   Rec Package:%d   ",uTemp.u ,uFrameCount);
			 g_sIecSend.format.maddrL = (uFrameCount - 1)&0xFF;  //包序号低位
             g_sIecSend.format.maddrM = (uFrameCount - 1)>>8;  //包序号高位
//			 g_sIecSend.format.maddrL = uTmpData[3];  //包序号低位
//             g_sIecSend.format.maddrM = uTmpData[2];  //包序号高位
             g_sIecSend.format.maddrH = 0x00;
			 uDataLen = uRecBuffer[6];   //获取日志帧长度
			  
             g_DT1000Updata.CRC = CalculateCRC(&uRecBuffer[7],uDataLen);
			 g_sIecSend.format.data[uDataLen]   = g_DT1000Updata.CRC&0xFF;
			 g_sIecSend.format.data[uDataLen+1] = g_DT1000Updata.CRC<<8;
//			 DEBUGOUT("PackageDataLen: %d  g_DT1000Updata.CRC:%04X\n",uDataLen,g_DT1000Updata.CRC);
			 //DataFlash_Write(DATAFLASH_DT1000_LOG+uFrameSum*i,&uRecBuffer[7],uRecBuffer[6]);
			 memcpy(g_sIecSend.format.data,&uRecBuffer[7],uDataLen);
			 msleep(5);
			
			 IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_TRANS,uDataLen+2,&g_sIecSend);    // 日志数据传输
		 }
	}
    else
	{
         return 0;
	}
		
	return 1;
}
/****************************************************************************
* 名    称：SouthLogEnd()
* 功    能：南向日志传输结束
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
void SouthLogEnd(void)
{
    int16_t iResult;
	uint8_t uTmpData[2];
	Uint_Char_Convert uTmpCrc;
	
	uTmpData[0] = S_FILE_EXPORT;  //日志导出子功能码
	uTmpData[1] = g_DT1000Updata.uData;   //导出日期

    iResult = ComMasterRead(&g_sMaster,g_DT1000Updata.uDevAddr,DT1000LOG_END,0,0,uRecBuffer,uTmpData);
    DEBUGOUT("iResult:%d\n",iResult);
	if(iResult > 0)
	{
		uTmpCrc.c[1] = uRecBuffer[4];  //接收到表计文件CRC高位
		uTmpCrc.c[0] = uRecBuffer[5];  //接收到表计文件CRC低位

//        DEBUGOUT("g_DT1000Updata.CRC:%04X  uTmpCrc.u:%04X",g_DT1000Updata.CRC,uTmpCrc.u);
		if(g_DT1000Updata.CRC == uTmpCrc.u)
		{
             //传输总包数
		     g_sIecSend.format.maddrL = g_DT1000Updata.frame_sum&0xFF;  //包序号低位
		     g_sIecSend.format.maddrM = g_DT1000Updata.frame_sum<<8;  //包序号高位
		     g_sIecSend.format.maddrH = 0x00;

			 //文件CRC
			 g_sIecSend.format.data[0] = uRecBuffer[5];
			 g_sIecSend.format.data[1] = uRecBuffer[4];

			 IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_FINISH,2,&g_sIecSend);   // 日志数据传输	
			 g_DT1000Updata.CRC=0xFFFF;
		}
	}
	return;
}
//===============================================================================
OS_STK TaskSouthInquireStk[TaskSouthInquireStkSize]@0x20004000;	      // 定义任务堆栈大小
/****************************************************************************
* 名    称：TaskSouthInquire()
* 功    能：南向查询任务
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
#define  SOUTH_INQUIRE         0    // 0：南向数据查询；
#define  SOUTH_DISCORY         1    // 1：南向自发现；
#define  SOUTH_POLL            2    // 2：南向自发现，数据查询，告警查询轮询；
#define  SOUTH_TIME            3    // 3：获取时间
#define  SOUTH_UPDATA          4    // 4: 升级表计
void TaskSouthInquire(void *p)
{
    p = p;

    uint32_t uRoundStartTime;    		// 大循环开始的时间戳
    uint32_t uRoundNowTime;      		// 大循环现在的时间戳
	uint32_t uDiscoryStartTime;
	uint32_t uDiscoryNowTime;
	
    uint8_t  uRoundEnd;                	// 南向查询一个循环结束
    uint8_t err = 0;
	static uint8_t  uSouthStep = SOUTH_INQUIRE;      // 南向查询步骤
    static uint8_t  uSigleUpdate=0;
	uint8_t uLogOutState;    			//日志导出状态
	U32_F_Char_Convert ctemp;
	uint16_t uFrameCount = 0;
	uint16_t uRecFrame;  				//从平台接收确认帧
    uint8_t TimeCount=0;

    pUartLock = OSMutexCreate(4,&err);	// 锁初始化

    // 主站结构体初始化
    g_sMaster.pComLock = pUartLock;		// 串口锁指针
    g_sMaster.iComFd = uart3;			// 串口文件号
    g_sMaster.uRecLostMax = 3;			// 最大丢帧次数，超过认为断连              ----->>>设定最大运行的丢帧次数，超过后认为断连
    g_sMaster.uRecCrcMax = 3;			// 最大CRC错误次数，超过认为断连           ----->>>设定最大运行的CRC校验次数，超过后认为断连
    g_sMaster.uFailTimeOut = 500;		// 丢帧超时，超过后，再次发送数据          ----->>>设定丢帧后再次发送的间隔时间，以1ms为计时单位，原500
    g_sMaster.u1BTimeOut = 6000;		//查询1B的时候接收延时时间
    g_sMaster.uSuccessDelay = 200;		// 一帧成功，延时，延时后查询下一帧        ----->>>设定一帧查询成功后查询下一帧的间隔时间
    g_sMaster.sSuccessTime = 0;			// %内部%一帧查询成功时的时间记录
    g_sMaster.uPreAddr = 0;				// -%内部%-已发送查询设备的地址            ----->内部使用，初始化为0
    g_sMaster.uPreAddr = 0;				// -%内部%-已发送的功能码                  ----->内部使用，初始化为0
    sSouth.uBaudRate = BAUDRATE_9600;  	// 波特率9600
    uRoundStartTime = OSTimeGet();   	// 大循环起始时间
	uDiscoryStartTime = OSTimeGet();  	//自发现起始时间
	
    while(1)
    {
        g_uTimeNow = OSTimeGet();   	// 获取当前的时间戳

        if(g_LoggerRun.update)  		// 数采升级中
        {
            sleep(10);
            continue;
        }
		
		/************************近端读取*485**********************************/
        #if(1==USE_485_DEBUG)
        if(Rs485DebugRead())
        {
        	DEBUGOUT("\r\nRs485DebugRead!!!\r\n");
            sleep(10);
            continue;
        }
        #endif

		/************************表计升级**************************************/
        if(g_LoggerRun.uFileIO_status == 0x01)
        {
			DEBUGOUT("\r\nSouth Upgrade Start!!!\r\n");
			SouthBroadcastUpdata();			//广播升级
			FEED_DOG();     				// 喂狗
		    sleep(50);                      //延时变更 from B31028&B31029，原参数为30
			SouthUpdataSoftChange();		//升级后软件版本变更
			
			uSigleUpdate=1;
		}

		if((uSigleUpdate == 1) && (g_LoggerRun.uFileIO_status == 0x01))
		{
			SouthSingleUpdata();			//单播升级
			sleep(50);                      //延时变更 from B31028&B31029，原参数为30
			g_LoggerRun.uFileIO_status = 0x00;
			SouthUpdataSoftChange();		//升级后软件版本变更
			uSigleUpdate=0;
		}

		/******************************表计日志导出********************************/
		if(S_FILE_EXPORT == g_LoggerRun.uFileIO_status)
		{
			DEBUGOUT("\r\nSouth Log Output!!!\r\n");
			for (uint8_t i = 0;  i < 10;i++)
			{
				g_DT1000Updata.nDataLen = SouthLogStart();   	//南向日志启动，去获取表计日志长度
				
				if(g_DT1000Updata.nDataLen > 0)
				{
				     //信息体地址，即日志导出类型
					 g_sIecSend.format.maddrL = S_FILE_EXPORT;
                     g_sIecSend.format.maddrM = 0x00;
                     g_sIecSend.format.maddrH = 0x00;

		 			 //获取文件长度
		 			 ctemp.u = g_DT1000Updata.nDataLen;
					 g_sIecSend.format.data[0] = ctemp.c[0];  	//104协议是小端模式
					 g_sIecSend.format.data[1] = ctemp.c[1];
					 g_sIecSend.format.data[2] = ctemp.c[2];
					 g_sIecSend.format.data[3] = ctemp.c[3];
					 g_sIecSend.format.data[4] = 0xC8;       	//报文以200字节数据传输

					 if((g_DT1000Updata.nDataLen%200) != 0)  	//判断文件长度除以200是否有余数，来计算帧总数
					 {
						 g_DT1000Updata.frame_sum = g_DT1000Updata.nDataLen/200 + 1;
					 }
					 else
					 {
						 g_DT1000Updata.frame_sum = g_DT1000Updata.nDataLen/200;
					 }
					
					 uFrameCount = g_DT1000Updata.frame_sum;
					 IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_START_ACK,5,&g_sIecSend);
                     break;            
				}
				else
				{     
				    sleep(5);
				}
			}
		   sleep(2);

		   while (uFrameCount)
		   {
			   TimeCount=0;
			   uLogOutState = SouthLogTransmission(g_DT1000Updata.frame_sum - uFrameCount + 1); //日志传输

			   if(uLogOutState > 0)
			   {
				   while(1)
				   {
                       sleep(1);
					   if((g_sIEC.recv.format.type == P_FILE_INOUT) && (g_sIEC.recv.format.reasonL == R_DATA_TRANS))  //接收确认帧类型及传输原因
					   {
						   uRecFrame = (g_sIEC.recv.format.data[0] | g_sIEC.recv.format.data[1]<<8);  //接收帧序号
						   if(uRecFrame == (g_DT1000Updata.frame_sum - uFrameCount))   //比对接收与发送序号是否相同
						   {
							   uFrameCount--;
							   break;
						   } 
 					   }
					   
			   		   TimeCount++;
					   if(TimeCount>=30)   //如果平台确认帧时间超过30s，退出日志传输
					   {
						   TimeCount=0;
						   uFrameCount=0;
						   break;
					   }
				  }
			   }
			   else
			   {
                   break;
			   }    
		   }
		   
           if((uFrameCount == 0) && (g_DT1000Updata.nDataLen != 0))
           {
			   SouthLogEnd();	//表计日志导出结束
		   }
		   g_LoggerRun.uFileIO_status = 0;
		}
		g_LoggerRun.uFileIO_status = 0;
		//--------------------------------------------------------------------------------
        TimedReboot(120);    //定时重启
        AlarmCheckout();     //告警检出
        //--------------------------------------------------------------------------------
        //南向查询步骤
        //--------------------------------------------------------------------------------
        switch(uSouthStep)
        {
        case SOUTH_INQUIRE:
            uRoundEnd = SouthInquire();  // 查询南向设备
            if(SOUTH_WORK == uRoundEnd || SOUTH_WAIT == uRoundEnd)
            {
                msleep(100);
            }
            else if(SOUTH_OVER == uRoundEnd)
            {
                sSouth.uBaudRate = SetBaudRate(sSouth.uBaudRate,BAUDRATE_9600);// 波特率切换到9600，用于搜索南向设备
				uSouthStep = SOUTH_POLL;
				DEBUGOUT("South Poll!!!\r\n",uRoundEnd);
                msleep(100);
            }
            else if(SOUTH_EMPTY == uRoundEnd)
            {
                sSouth.uBaudRate = SetBaudRate(sSouth.uBaudRate,BAUDRATE_9600);// 波特率切换到9600，用于搜索南向设备
                uSouthStep = SOUTH_DISCORY;
                DEBUGOUT("South Discory!!!\r\n");
            }	
            break;

        case SOUTH_DISCORY:
            uRoundEnd = SlaveDeviceAutoAllocation();   // 搜索表计设备并分配地址
            //sleep(3);
            if(SEARCH_END == uRoundEnd) // 一个循搜索询结束，不需要导表      时间同步
            {
				uDiscoryStartTime = g_uTimeNow;
				uSouthStep = SOUTH_POLL;
				DEBUGOUT("South Poll!!!\r\n",uRoundEnd);
            }
            else if(SEARCH_END_IMPORT == uRoundEnd) // 一个循搜索询结束，需要导表
            {
				uDiscoryStartTime = g_uTimeNow;
				uSouthStep = SOUTH_TIME;
            }
            else if(SEARCH_NOTCONNECT == uRoundEnd) // 平台未连接
            {
                if(RUNNING_EMPTY == g_LoggerRun.run_status)   // 空数采 g_sIecPointCount
                {
                    OSTimeDlyHMSM(0,0,10,0);
                }
                else
                {
                    uSouthStep = SOUTH_POLL;
                }
            }
            else
            {
            	DEBUGOUT("South Discory uRoundEnd：%d!!!\r\n",uRoundEnd);
                msleep(100);
            }
            break;

        case SOUTH_POLL:
            uRoundNowTime = g_uTimeNow;   // 大循环现在时间
			uDiscoryNowTime = g_uTimeNow;  //自发现现在时间
			if(TimeGapJudge(uDiscoryStartTime,uDiscoryNowTime,DISCORY_ROUND_TIME))
		    {
                 if(TimeGapJudge(uRoundStartTime,uRoundNowTime,SLAVE_ROUND_TIME))
                 {
					 sleep(1);
					 if(NORTH_OK == g_LoggerRun.north_status)
					 {
						 uRoundEnd = SouthInquireAlarm();  // 查询南向设备 
					 }
                 }
                 else
                 {
                      uRoundStartTime = g_uTimeNow;   // 大循环起始时间
                      uSouthStep = SOUTH_INQUIRE;
                      DEBUGOUT("South Inquire !!!\r\n");
                 }
		    }
		    else if((g_LoggerRun.run_status > RUNNING_INPUT_104) || (g_LoggerRun.run_status < RUNNING_INPUT_START))
			{
				uDiscoryStartTime=g_uTimeNow;
                if(GetRecordAllow())
				{
					// 检查南向设备状态，有南向设备且没有全部丢帧
					if(0 == CheckDevState()) //&& g_DeviceSouth.yx_sum && g_DeviceSouth.yc_sum)
					{
						//OSMutexPend(pRecordLock,0,&err);//请求信号量
						uSaveSouthLog(sDateTime,SOUTH_DATA_LOG,NULL,20,0);
						DEBUGOUT("YCSum = %d   YXSum = %d !!!\r\n",g_DeviceSouth.yc_sum,g_DeviceSouth.yx_sum);
						//规避方案，问题单939，遥测数为0则重读eep，遥测空间为空则重启
						if(!g_DeviceSouth.yc_sum)
						{
							EepReadData(EEP_LOGGER_DEVICE_INF_HEAD,(uint8_t *)&g_DeviceSouth,sizeof(g_DeviceSouth),&g_DeviceSouth.CRC);// 设备信息和点表信息读取
						}

						if(NULL == IEC104_DATA_YC)
							DEBUGOUT("Repeat IEC104_DATA_YC Space!!!\r\n");
						RecordHistory(IEC104_DATA_YX,(uint8_t *)IEC104_DATA_YC,g_DeviceSouth.yx_sum,g_DeviceSouth.yc_sum);// 保存数据
					}
				}
				else
				{
					if(NORTH_OK == g_LoggerRun.north_status)// 连接服务器完成
					{
						if(0 == (g_LoggerRun.err&err_power_off))
						{
							SetRecordAllow(1);  // 允许存储历史数据
						}
					}
				}

				uSouthStep = SOUTH_DISCORY;
				DEBUGOUT("South Discory!!!\r\n");
			}
            break;

        case SOUTH_TIME:
            if(RUNNING_WORK_READ == g_LoggerRun.run_status)
            {
                uRoundNowTime = g_uTimeNow;   // 大循环现在时间
                uDiscoryNowTime = g_uTimeNow;   // 大循环现在时间
                uSouthStep = SOUTH_INQUIRE;
                DEBUGOUT("South Inquire !!!\r\n");
            }
            else
            {
                msleep(200);
            }
            break;

        default:
            uSouthStep = SOUTH_POLL;
            break;
        }
    }
}
/******************************************************************************
* 名    称：SouthWriteYk()
* 功    能：南向遥控
* 入口参数：
*           无      无

* 出口参数：
* 范    例:
******************************************************************************/
uint8_t SouthWriteYk(void)
{
    uint16_t uValue;       // 寄存器值
    uint16_t uYkCount;
    uint16_t uIecYk=0;      // IEC104表的遥控地址点
    uint16_t uModbusAddr=0; // 遥控点对应的MODBUS寄存器地址
    uint16_t j;
    uint8_t  uRelDevAddr;   // 南向设备相对地址
    uint8_t  uRelNum;       // 设备相对点表号
    uint8_t  uTableYkSum;   // 点表遥控点总数
    uint8_t  uYkPoint;      // 设备的第几个遥控点
    int8_t   iResult;
    /*
    如，设备1的遥控起始地址为0x6004,平台下发的遥控点为0x6007
    则为设备1的第4个遥控点
    */
    if(NULL==IEC104_DATA_YK)
    {
        return 0;
    }
    for(uYkCount=0; uYkCount<g_DeviceSouth.yk_sum; uYkCount++)
    {
        if(0!=IEC104_DATA_YK[uYkCount])
        {
//            if(0x80&IEC104_DATA_YK[uYkCount])
//            {
//                IEC104_DATA_YK[uYkCount] = 0x00; //逆变器开关机
//            }
            uValue = IEC104_DATA_YK[uYkCount] & 0x7F;
            uIecYk = uYkCount + 0x6001;
            break;
        }
    }

    if(uIecYk)
    {
        for(uRelDevAddr=0; uRelDevAddr<MAX_device; uRelDevAddr++)  // 搜索设备
        {
            uRelNum = g_DeviceSouth.device_inf[uRelDevAddr].rel_num;
            uTableYkSum = g_sIecPointCount[uRelNum].uYkSum;

            if(g_DeviceSouth.device_inf[uRelDevAddr].yk_start_addr<=uIecYk && uIecYk<(g_DeviceSouth.device_inf[uRelDevAddr].yk_start_addr+uTableYkSum))
            {
                uYkPoint = uIecYk - g_DeviceSouth.device_inf[uRelDevAddr].yk_start_addr + 1;
                break;
            }
        }
        if(uRelDevAddr<MAX_device) // 有搜索到设备
        {
            for(j=0; j<g_DeviceSouth.protocol[uRelNum].mess_point_sum; j++) // 搜索MODBUS地址
            {
                if(TYPE_YK==g_pRegPoint[uRelNum][j].reg_type.type.mess)
                {
                    uYkPoint--;
                    if(0==uYkPoint)
                    {
                        uModbusAddr = g_pRegPoint[uRelNum][j].reg_addr;
                        break;
                    }
                }
            }
        }
    }

    if(uModbusAddr)
    {
        if(0x02==g_DeviceSouth.device_inf[uRelDevAddr].protocol_type) // 标准MODBUS
        {
            if(uModbusAddr>=40000)   // 读数据保持寄存器40001~49999
            {
                uModbusAddr -= 40000;
            }
            else if(uModbusAddr>=30000)  // 读输入寄存器30001~39999
            {
                uModbusAddr -= 30000;
            }
        }

        iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x06,uModbusAddr,1,&uValue);
        if(iResult<0)
        {
            DEBUGOUT("遥控失败%d",g_DeviceSouth.device_inf[uRelDevAddr].addr);
        }
        /*else
        {
            iResult = ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x03,uModbusAddr,1,&uRecBuffer);
            if(iResult>0 || -MODBUS_ILLEGAL_ADDR==iReadResult)  // 读取成功
            {

            }
        }*/

        IEC104_DATA_YK[uYkCount] = 0x00;

        return 0;
    }

    return 0;
}

/******************************************************************************
* 名    称：SouthSdModbus()
* 功    能：南向设点
* 入口参数：
*           uModbusAddr：设点的modbus地址
*           uRelDevAddr：相对设备地址
*
* 出口参数：
* 范    例:
******************************************************************************/
uint8_t SouthSdModbus(uint16_t uModbusAddr,uint8_t  uRelDevAddr,uint16_t uSdCount,uint32_t uValue,uint8_t uReg_count)
{
    int8_t   iResult;
	uint16_t uDataTemp[2];

	if(uModbusAddr)
	{
		if(1==uReg_count)
		{
			uDataTemp[0] = uValue & 0xFFFF;

		    iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x06,uModbusAddr,uReg_count,uDataTemp);

			if(iResult<0)
			{
				 DEBUGOUT("######SdDevice：%d RegAddr：%d SdFailFailFail!!!######",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr);
			}
			else
			{
				DEBUGOUT("######SdDevice：%d RegAddr：%d Value：%d######\n",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr,uValue);
			}
			IEC104_DATA_SD[uSdCount] = 0x00;
		}
		else if(2==uReg_count)
		{
			uDataTemp[0] = uValue>>16;
			uDataTemp[1] = uValue & 0xFFFF;

		    iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x10,uModbusAddr,uReg_count,uDataTemp);
		
			if(iResult<0)
			{
				 DEBUGOUT("######SdDevice：%d RegAddr：%d SdFailFailFail!!!######",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr);
			}
			else
			{
				 DEBUGOUT("######SdDevice：%d RegAddr：%d Value：0x%08X######\n",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr,uValue);
			}
			IEC104_DATA_SD[uSdCount] = 0x00;
		}
	}

	return 0;
}
/****************************************************************************
* 名	 称：SouthIecSd()
* 功	 能：设点104地址查找点表中的modbus地址
* 入口参数：uIecSd：IEC104表的设点地址点
*           uValue：104设点地址对应的值
*           uSdCount：设点计数
*           uSdSum：设点总数
* 出口参数：
* 范	 例: 无
****************************************************************************/
uint16_t SouthIecSd(uint16_t uIecSd,uint32_t uValue,uint16_t uSdCount,uint8_t uSdSum)
{
     uint8_t  uRelNum;       	//设备相对点表号
	 uint16_t uModbusAddr = 0; 	// 遥控点对应的MODBUS寄存器地址
     uint16_t j;
	 uint8_t  uRelDevAddr;   	// 南向设备相对地址
	 uint8_t  uSdPoint;      	// 设备的第几个设点点
	 uint8_t  uReg_count;   	//信号点的寄存器个数

	 if(uIecSd)
     {
         for(uRelDevAddr = 0; uRelDevAddr < MAX_device; uRelDevAddr++)  // 搜索设备
         {
             uRelNum = g_DeviceSouth.device_inf[uRelDevAddr].rel_num;
             //uTableSdSum = g_sIecPointCount[uRelNum].uSdSum;
             if((g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr <= uIecSd) 
			 	&& (uIecSd < (g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr+(g_DeviceSouth.sd_sum/g_DeviceSouth.device_sum))))
             {
                 uSdPoint = uIecSd - g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr + 1;
                 break;
             }
         }

         if(uRelDevAddr < MAX_device) // 有搜索到设备
         {
             for(j = 0; j < g_DeviceSouth.protocol[uRelNum].mess_point_sum; j++) // 搜索MODBUS地址
             {
                 if(TYPE_SD == g_pRegPoint[uRelNum][j].reg_type.type.mess)
                 {
                     uSdPoint--;
                     if(0 == uSdPoint)
                     {
						 uModbusAddr = g_pRegPoint[uRelNum][j].reg_addr;
						 if(1==g_pRegPoint[uRelNum][j].reg_count)
						 {
							 uReg_count=1;
							 SouthSdModbus(uModbusAddr,uRelDevAddr,uSdCount,uValue,uReg_count);
						 }
						 else if(2==g_pRegPoint[uRelNum][j].reg_count)
						 {
							 uReg_count=2;
							 SouthSdModbus(uModbusAddr,uRelDevAddr,uSdCount,uValue,uReg_count);
						 }
					     continue;
                     }
                }
            }
         }
     }
	 return 0;
}
/******************************************************************************
* 名    称：SouthWriteSD()
* 功    能：南向设点
* 入口参数：
*           无      无
*
* 出口参数：
* 范    例:
******************************************************************************/
uint8_t SouthWriteSD(void)
{
    uint32_t uValue;       // 寄存器值
    uint16_t uIecSd = 0;      // IEC104表的设点地址点
	uint8_t  uSdSum = 0;     //设点总数
    /*
    如，设备1的遥调起始地址为0x6201,平台下发的遥控点为0x6203
    则为设备1的第3个遥控点
    */
    if(NULL == IEC104_DATA_SD)
    {
//        DEBUGOUT("IEC104_DATA_SD为空\r\n");
		return 0;
    }

    uSdSum = IEC104_DATA_SD[0];

    for(uint8_t k = 0;k < uSdSum;k++)
    {
    	uIecSd = IEC104_DATA_SD[2*k+1];
		uValue = IEC104_DATA_SD[(k+1)*2];
		DEBUGOUT("uValue:%d uIecSd：%X k = %d\r\n",uIecSd,uValue,k);
    	SouthIecSd(uIecSd,uValue,k+1,uSdSum);
    	msleep(5);
    	IEC104_DATA_SD[2*k+1] = 0;
    	IEC104_DATA_SD[(k+1)*2] = 0;
    }
    IEC104_DATA_SD[0] = 0;

    return 0;
}

/******************************************************************************
* 名    称：SouthReadSD()
* 功    能：南向设点
* 入口参数：
*           无      无
*
* 出口参数：
* 范    例:
******************************************************************************/
uint8_t SouthReadSD(void)
{
    uint16_t uValue;       		// 寄存器值
    uint16_t uSdCount = 0;
    uint16_t uIecSd = 0;      	// IEC104表的设点地址点
    uint16_t uModbusAddr = 0; 	// 遥控点对应的MODBUS寄存器地址
    uint16_t j,i;
    uint8_t  uRelDevAddr = 0;   	// 南向设备相对地址
    uint8_t  uRelNum,sRegist_Count = 0;       	// 设备相对点表号
    uint8_t  uSdPoint;      	// 设备的第几个设点点
	uint8_t  uIntervalPoint = 2,South_SendCount = 0;
	U32_F_Char_Convert ctemp;
	uint32_t uValue_Temp = 0;       		// 寄存器值
	DEBUGOUT("YT ACQUIRY TASK!!! \r\n");
	/* 如,设备1的遥控起始地址为0x6004,平台下发的遥控点为0x6007
    *	则为设备1的第4个遥控点
    */
    if(NULL == IEC104_DATA_SD)
    {
    	DEBUGOUT("无遥调空间 \r\n");
        return 0;
    }

	uValue = IEC104_DATA_SD[0];			//查询的遥调点总数
	uIecSd = IEC104_DATA_SD[1];			//查询的104地址

//	DEBUGOUT("uIecSd = 0x%d  uValue = %d!!!\r\n",uIecSd,uValue);
	if(uIecSd < 0x6201)
		return 0;

	/*********************根据平台地址获取所需的遥调地址***********************/
	memset(IEC104_DATA_SD,0,sizeof(IEC104_DATA_SD));
	uValue = uIecSd+uValue;
	for(;uIecSd < uValue;uIecSd++)
	{
		for(;uRelDevAddr < MAX_device; uRelDevAddr++)	//遍历设备
		{
			uRelNum = g_DeviceSouth.device_inf[uRelDevAddr].rel_num;	//设备的相对点表号
			if(g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr != 0)
			{
				if((g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr <= uIecSd)
					&& (uIecSd < (g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr + g_sIecPointCount[uRelNum].uSdSum)))
				{
//					printf("/**** Pinnet END g_sIecPointCount[%d].uSdSum = %d\r\n",uRelNum,g_sIecPointCount[uRelNum].uSdSum);
					uSdPoint = uIecSd - g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr + 1;
					break;
				}
			}
		}

		if(uRelDevAddr < MAX_device) 		//有搜索到设备
        {
            for(j = 0; j < g_DeviceSouth.protocol[uRelNum].mess_point_sum; j++)	//搜索MODBUS地址
            {
				if(TYPE_SD == g_pRegPoint[uRelNum][j].reg_type.type.mess)
                {
					uSdPoint--;
					if(0 == uSdPoint)
                    {
                        uModbusAddr = g_pRegPoint[uRelNum][j].reg_addr;
                        IEC104_DATA_SD[uSdCount] = (g_pRegPoint[uRelNum][j].reg_count == 1)?uModbusAddr:(uModbusAddr|(1<<16));
//                        printf("IEC104_DATA_SD[%d] = 0x%08X\r\n",uSdCount,IEC104_DATA_SD[uSdCount]);
                        uSdCount++;
                    }
                }
            }
        }
	}

	/*********************************冒泡*************************************/
	if(0 != uSdCount)
	{
		for (j = 0; j < uSdCount - 1; j++)
		{
			for (i = 0; i < uSdCount - 1 - j; i++)
			{
				if(((IEC104_DATA_SD[i])<<16) > ((IEC104_DATA_SD[i+1])<<16))
				{
					memcpy(&uValue_Temp,&IEC104_DATA_SD[i],sizeof(uValue_Temp));
					memcpy(&IEC104_DATA_SD[i],&IEC104_DATA_SD[i+1],sizeof(uValue_Temp));
					memcpy(&IEC104_DATA_SD[i+1],&uValue_Temp,sizeof(uValue_Temp));
				}
			}
		}
	}
	else
	{
		DEBUGOUT("Not YT Point!!!\r\n");
		return 0;
	}

	/******************************分段查询************************************/
	uValue = 0;
	for(i = 1; i < (uSdCount+1); i++)
	{
		uValue++;
		sRegist_Count += (((IEC104_DATA_SD[i-1])>>16)+1);
		if((((IEC104_DATA_SD[i])<<16)>>16) != ((((IEC104_DATA_SD[i-1])<<16)>>16) + (((IEC104_DATA_SD[i-1])>>16)+1)))
		{
//			printf("addr:%d  uValue:%d  IEC104_DATA_SD:0x%X\r\n",(i - uValue),sRegist_Count,(IEC104_DATA_SD[i - uValue]));
			memset(uRecBuffer,0,256);
			for(South_SendCount = 0;South_SendCount < 2;South_SendCount++)
			{
				if(ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,03,IEC104_DATA_SD[i - uValue],sRegist_Count,uRecBuffer,NULL) > 0)
				{
					break;
				}
				else
				{
					if(South_SendCount == 1)
						memset(IEC104_DATA_SD,0,sizeof(IEC104_DATA_SD));
					return 0;
				}
			}

			for(j = (i - uValue);j < i;j++)
			{
				uint8_t k = 0;
				if(0 == ((IEC104_DATA_SD[j])>>16))
				{
//					DEBUGOUT("\r\n寄存器地址：%d  ",((IEC104_DATA_SD[j])<<16)>>16);
					for(k = 0;k < 2;k++)
					{
						ctemp.c[k] = uRecBuffer[uIntervalPoint + (2-k)];
						IEC104_DATA_SD[j] = ctemp.u16;
					}
					uIntervalPoint += 2;
//					DEBUGOUT("数据长度1 - 数据为：%X\r\n",IEC104_DATA_SD[j]);
				}
				else if(1 == ((IEC104_DATA_SD[j])>>16))
				{
//					DEBUGOUT("\r\n寄存器地址：%d  ",((IEC104_DATA_SD[j])<<16)>>16);
					for(k = 0;k < 4;k++)
					{
						ctemp.c[k] = uRecBuffer[uIntervalPoint + (4-k)];
						IEC104_DATA_SD[j] = ctemp.u;
					}
					uIntervalPoint += 4;
//					DEBUGOUT("数据长度2 - 数据为：%X\r\n",IEC104_DATA_SD[j]);
				}
			}
			uValue = 0;
			sRegist_Count = 0;
			uIntervalPoint = 2;
		}

	}
    return 0;
}
/******************************************************************************
* 名    称：SouthSynctime()
* 功    能：南向同步时间
* 入口参数：
*           无      无
* 出口参数：
* 范    例:
******************************************************************************/
void SouthSynctime(void)
{
    time_t uTimeTick;
    //uint32_t uTimeTick;
    uint16_t uDataTemp[2];
    struct tm *p;
    uint8_t uAddr = 0;
    int8_t iResult;
	uint8_t uSynctimeCount;
    while(1)
    {
        if(uAddr >= MAX_device)
        {
            break;
        }
        if(0 == g_DeviceSouth.device_inf[uAddr].addr)
        {
            uAddr++;
            continue;
        }

		iResult = ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[uAddr].addr,DISCOVERY_REPORT,0,0,uRecBuffer,NULL);
					
		if(iResult > 0)  // 读取3B成功
		{
			msleep(300);
			uTimeTick = RealTimeGetTick();
			uDataTemp[0] = uTimeTick>>16;
			uDataTemp[1] = uTimeTick & 0xFFFF;
			//DEBUGOUT("写时间寄存器：%d",uTimeTick);
			uSynctimeCount = 0;
            if(uSynctimeCount < 3)
            {
				iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uAddr].addr,0x10,40002,2,uDataTemp);  //写两个寄存器的时间
								
				if(iResult < 0)
				{
					uSynctimeCount++;
					DEBUGOUT("表计%d时间同步失败%d次",g_DeviceSouth.device_inf[uAddr].addr,uSynctimeCount);
					msleep(200);
				}
				else
				{
                   uSynctimeCount = 3;
				   p = localtime(&uTimeTick);
				   DEBUGOUT("表计%d同步时间为：%d-%d-%d %d:%d:%d",g_DeviceSouth.device_inf[uAddr].addr,1900+p->tm_year,1+p->tm_mon,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
				}
			}	
		}
        uAddr++;
        msleep(300);
    }
}
/****************************************************************************
* 名    称：TaskSouthWrite()
* 功    能：南向查询任务
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
OS_STK TaskSouthWriteStk[TaskSouthWriteStkSize];	      // 定义任务堆栈大小
void TaskSouthWrite(void *p)
{
    uint8_t *pMes;
    uint8_t err;
    p = p;

    while(1)
    {
        pMes = OSQPend(MesQ,0,&err);    //请求消息队列
//        DEBUGOUT("队列收:%d\n",*pMes);
        if(SOUTH_CMD_YK == *pMes)     // 遥控
        {
            SouthWriteYk();
        }
        else if(SOUTH_CMD_SYNC == *pMes)  // 同步时间
        {
            SouthSynctime();
        }
        else if(SOUTH_CMD_SD == *pMes)  // 设点
        {
            SouthWriteSD(); 
        }
		else if(SOUTH_CMD_READSD == *pMes)  //读设点
        {
			SouthReadSD();
		}
    }
}

/******************************************************************************
* 名    称：AlarmAddDev()
* 功    能：添加下联设备告警。
* 入口参数：
            uRelAddr:      设备相对地址
            uSum:          点表告警点总数
            uModAddr：     寄存器地址
            uBit：         告警点位置
*
* 出口参数：无
* 范    例:
******************************************************************************/
void AlarmAddDev(uint8_t uRelAddr,uint8_t uSum,uint16_t uModAddr,uint16_t uValue)
{
    uint8_t temp;
    if(NULL==g_psSouthAlarmCopy[uRelAddr])
    {
        return;
    }

    for(temp=0; temp<uSum; temp++)
    {
        if(g_psSouthAlarmCopy[uRelAddr][temp].mdbus_addr == uModAddr)
        {
            break;
        }
    }
    if (0xFFFF != uValue)//采集到全FF告警丢弃
    {        
        g_psSouthAlarmCopy[uRelAddr][temp].alarm_value =uValue;
    }
}
/******************************************************************************
* 名    称：AlarmCheckout()
* 功    能：上报南向设备告警。
* 入口参数：

*
* 出口参数：
* 范    例:
******************************************************************************/
#define ALARM_POWER_DOWN   0x04   // 掉电告警
#define ALARM_COM_LOST     0x01   // 通讯断连告警
#define ALARM_DEV_ALARM    0x02   // 设备告警
#define ALARM_COM_DEV      0x03   // 通讯断连 和 设备告警
uint8_t AlarmCheckout(void)
{
    static uint32_t s_uAlarmTimeOut=0;            // 告警上报后，超时没有确认再次上报。
    static uint32_t s_uPowerOffAlarmTimeOut = 0;  // 电源断电后，超时没有确认再次上报
    uint8_t   i,j;
    uint8_t   uRes;
    uint8_t   uSum;       // 告警点总数
    uint16_t  uNewAlarm;  // 新告警值
    uint16_t  uOldAlarm;  // 老告警值
    uint16_t  uBit;       // 告警点所在位
    uint8_t   uNeedToSave; // 需要保存告警点

    if(NORTH_OK!=g_LoggerRun.north_status)// || RUNNING_WORK_READ!=Logger_run.run_status)  // 连接服务器未完成
    {
        s_uAlarmTimeOut = g_uTimeNow;
        s_uPowerOffAlarmTimeOut = g_uTimeNow;
        return 0;
    }
    //-----------------------------------------------------------------------------------
    // 数采断电告警
    if(g_uAlarmReport & ALARM_POWER_DOWN)
    {
        if(TIMEOUT == TimeOut(s_uPowerOffAlarmTimeOut,g_uTimeNow,5000))//if(s_uPowerOffAlarmTimeOut>500)  // 5s
        {
            g_uAlarmReport &= ~ALARM_POWER_DOWN;
            s_uPowerOffAlarmTimeOut = g_uTimeNow;
            if((g_LoggerRun.err&err_power_off) != (g_LoggerAlarm.log_alarm&err_power_off))
            {
                AlarmReport(0,     	// 设备通讯地址
                            0x03,   // 异常代码-数采设备异常
                            0x0001, // MODBUS寄存器地址
                            0,      // 偏移量
                            JUDGE((g_LoggerRun.err&err_power_off)),      // 告警产生或恢复
                            1);     // 立即上报
                g_uAlarmReport |= ALARM_POWER_DOWN;
            }
        }
    }
    else
    {
        s_uPowerOffAlarmTimeOut = g_uTimeNow;
    }

    //-----------------------------------------------------------------------------------
    if(g_uAlarmReport & ALARM_COM_DEV)					// 南向设备告警 或 南向设备通讯告警
    {
        if(TIMEOUT == TimeOut(s_uAlarmTimeOut,g_uTimeNow,30000))//if(s_uAlarmTimeOut>3000)  // 30s
        {
            g_uAlarmReport &= ~ALARM_COM_DEV;
            s_uAlarmTimeOut = g_uTimeNow;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        if(TIMEOUT == TimeOut(s_uAlarmTimeOut,g_uTimeNow,5000))//if(s_uAlarmTimeOut<500)  // 5s
        {
            s_uAlarmTimeOut = g_uTimeNow;
        }
        else
        {
            return 1;
        }
    }

    //-----------------------------------------------------------------------------------
    // 南向设备通讯告警
    if(g_LoggerRun.err_lost != g_LoggerAlarm.dev_lost)
    {
        for(uBit=0;uBit<16;uBit++)
        {
            if((g_LoggerRun.err_lost&(1<<uBit)) == (g_LoggerAlarm.dev_lost&(1<<uBit)) )
            {
                continue;
            }
            if(0==g_DeviceSouth.device_inf[uBit].addr)
            {
                g_LoggerAlarm.dev_lost &= ~(1<<uBit);
                g_LoggerRun.err_lost &= ~(1<<uBit);
                continue;
            }
            uRes = AlarmReport(g_DeviceSouth.device_inf[uBit].addr,              // 设备通讯地址
                                         0x01,                                  // 南向设备通讯异常
                                         0x0000,                                // MODBUS寄存器地址
                                         0x00,                                  // 偏移量
                                         JUDGE(g_LoggerRun.err_lost&(1<<uBit)), // 告警产生或恢复
                                         0);    // 不立即上报
            if(g_LoggerRun.err_lost&(1<<uBit))//告警产生
            {
                g_LoggerAlarm.dev_lost |= (1<<uBit);
                uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // 保存告警值
            }

            if(uRes) // 一帧满
            {
                g_uAlarmReport |= ALARM_COM_LOST;

                if(uNeedToSave)
                {
                    uNeedToSave = 0;
                    SaveEepData(EEP_ALARM);  // 保存告警值
                }
                return 2;
            }
        }
    }
    //-----------------------------------------------------------------------------------
    // 南向设备告警

    for(i=0; i<MAX_device&&i<g_DeviceSouth.device_sum; i++)
    {
		

		uSum = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[i].rel_num].alarm_sum;
        for(j=0; j<uSum; j++)
        {
			uNewAlarm = g_psSouthAlarmCopy[i][j].alarm_value;
            uOldAlarm = g_psSouthAlarm[i][j].alarm_value;

            if(uNewAlarm == uOldAlarm)
            {
                continue;
            }
			for(uBit=0; uBit<16; uBit++)
			{
				if((uNewAlarm&(1<<uBit)) == (uOldAlarm&(1<<uBit)))
				{
					continue;
				}
				//DEBUGOUT("uNewAlarm:%duOldAlarm:%d\n",uNewAlarm,uOldAlarm);
				uRes = AlarmReport(g_DeviceSouth.device_inf[i].addr,		// 设备通讯地址
								   0x02,								 // 南向设备异常
								   g_psSouthAlarmCopy[i][j].mdbus_addr,  // MODBUS寄存器地址
								   uBit,								 // 偏移量
								   JUDGE(uNewAlarm&(1<<uBit)),			 // 告警产生或恢复
								   0);	  // 不立即上报

				if(uNewAlarm&(1<<uBit))//告警产生
				{
					g_psSouthAlarm[i][j].alarm_value |= (1<<uBit);
					uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // 保存告警值
				}
				if(uRes) // 一帧满
				{
					g_uAlarmReport |= ALARM_DEV_ALARM;
					if(uNeedToSave)
					{
						uNeedToSave = 0;
						SaveEepData(EEP_ALARM);  // 保存告警值
					}
					return 2;
				}
			}
        }
    }

    uRes = AlarmReport(0,   // 设备通讯地址
                    0,   // 南向设备异常
                    0,   // MODBUS寄存器地址
                    0,   // 偏移量
                    0,   // 告警产生或恢复
                    1);  // 立即上报
    if(uRes) // 一帧满
    {
        g_uAlarmReport |= ALARM_COM_DEV;
        if(uNeedToSave)
        {
            uNeedToSave = 0;
            SaveEepData(EEP_ALARM);  // 保存告警值
        }
        return 2;
    }
    if(uNeedToSave)
    {
        uNeedToSave = 0;
        SaveEepData(EEP_ALARM);  // 保存告警值
    }
    return 0;
}
/******************************************************************************
* 名    称：AlarmAck()
* 功    能：确认告警。
* 入口参数：
            uLen         有效数据长度
            puData       收到的数据
*
* 出口参数：无
* 范    例:
* 备    注： 此函数在IEC104.c中收到告警确认帧调用
******************************************************************************/
void AlarmAck(uint8_t uLen,const uint8_t *puData)
{
    uint8_t i;
    uint8_t temp;
    uint8_t uSum;
    uint8_t  uRelAddr;  // 设备相对地址
    uint16_t uModAddr;  // MODBUS告警点地址，寄存器地址
    uint8_t  uNeedToSave=0; // 需要保存告警点

    for(i=0;i<uLen;i+=6)
    {
    	FEED_DOG();
        uRelAddr = SlaveAddrConvert(puData[i]);
        uModAddr = puData[i+2] | (puData[i+3]<<8);

        if(uRelAddr >= MAX_device)    //数采本身地址是0，查找相对地址返回0xFF
        {
            return;
        }
        FEED_DOG();
        switch(puData[i+1])
        {
        case 0x02:// 南向设备告警

            if(NULL==g_psSouthAlarm[uRelAddr])
            {
                break;
            }
            uSum = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[uRelAddr].rel_num].alarm_sum;

            for(temp=0; temp<uSum; temp++)
            {
                if(g_psSouthAlarm[uRelAddr][temp].mdbus_addr == uModAddr)
                {
                    break;
                }
            }

            if(temp>=uSum)
            {
                break;
            }

            if(puData[i+5])  // 1异常产生，0异常恢复
            {
                //g_psSouthAlarm[uRelAddr][temp].alarm_value |= (1<<puData[i+4]);
				//uNeedToSave = 1;
            }
            else
            {
                g_psSouthAlarm[uRelAddr][temp].alarm_value &= ~(1<<puData[i+4]);
                uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // 保存告警值
            }
            //DEBUGOUT("g_psSouthAlarm[uRelAddr][temp].alarm_value:%d i:%d",g_psSouthAlarm[uRelAddr][temp].alarm_value,i);
            g_uAlarmReport &= ~ALARM_DEV_ALARM;
            break;

        case 0x01:// 南向通讯告警
            if(puData[i+5])
            {
                //g_LoggerAlarm.dev_lost |= (1<<uRelAddr);
            }
            else
            {
                g_LoggerAlarm.dev_lost &= ~(1<<uRelAddr);
            }
            uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // 保存告警值

            g_uAlarmReport &= ~ALARM_COM_LOST;
            break;

        case 0x03:// 数采告警
            if(0x0001==uModAddr && 0x00==puData[i+4])  // 断电告警
            {
                g_LoggerAlarm.log_alarm = g_LoggerRun.err;
                g_uAlarmReport &= ~ALARM_POWER_DOWN;
            }
            if(0==g_LoggerRun.err&err_power_off)     //告警恢复才在此处保存
            {
                uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // 保存告警值
            }
            break;
		default:
            break;
        }

        if(uNeedToSave)
        {
            SaveEepData(EEP_ALARM);  // 保存告警值
        }
        FEED_DOG();
    }
	 return;
}
/******************************************************************************
* 名    称：AlarmReport()
* 功    能：告警上报点创建。
* 入口参数：
            addr:      异常设备地址，0为数采本身
            err_code:  异常代码，为0为上报未上报的告警
            reg_addr:  MODBUS寄存器地址
            offset:    偏移量
            event:     1:异常产生；0:异常恢复
            imd:       1:立即上报；0：组帧上报
*
* 出口参数：没有连接到服务器返回0
* 范    例:
******************************************************************************/
uint8_t AlarmReport(uint8_t addr,uint8_t err_code,uint16_t reg_addr,uint8_t offset,uint8_t event,uint8_t imd)
{
    if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        return 0;
    }
    //-----------------------------------------------------------------
    do
    {
        if(g_sBurst.alarm_count<30 && err_code)   // 一个告警点占用6个字节
        {
            g_sBurst.alarm_report[g_sBurst.alarm_count*6]   = addr;            // 设备通讯地址
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+1] = err_code;        // 南向设备异常
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+2] = reg_addr&0xff;   // MODBUS寄存器地址低位
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+3] = reg_addr>>8;     // MODBUS寄存器地址高位
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+4] = offset;          // 偏移量
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+5] = event;           // 1:异常产生；0：异常恢复

            g_sBurst.alarm_count++;

            err_code = 0;
        }

        if((imd && 0!=g_sBurst.alarm_count) || g_sBurst.alarm_count>=30)
        {
            // 一帧满先上传。
            g_sIecSend.format.maddrL = 0x00;
            g_sIecSend.format.maddrM = 0x00;
            g_sIecSend.format.maddrH = 0x00;

            memcpy(g_sIecSend.format.data,&g_sBurst.alarm_report[0],g_sBurst.alarm_count*6);  // I帧数据
            IecCreateFrameI(P_ERR_PROCESS,0x01,R_ACTIVE,g_sBurst.alarm_count*6,&g_sIecSend); // 字节数为信息点数*4 - 已经计数的第一个信息体地址

            // 重新开始记录
            g_sBurst.alarm_count = 0x00;

            return 1;
        }
    }while(err_code);
    //-----------------------------------------------------------------
    return 0;
}
/******************************************************************************
* 名    称：SetBaudRate()
* 功    能：
* 入口参数：
            addr:      设备地址
            data       收到的数据
*
* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t SetBaudRate(uint8_t now,uint8_t target)
{
    uint32_t  uBaudRate;
//1:2400；2:4800；3:9600；4:19200；5:38400；6:115200
    if(now==target)
    {
        return now;
    }

    switch(target)
    {
    case BAUDRATE_2400:
        uBaudRate = 2400;
        break;

    case BAUDRATE_4800:
        uBaudRate = 4800;
        break;

    case BAUDRATE_9600:
        uBaudRate = 9600;
        break;

    case BAUDRATE_19200:
        uBaudRate = 19200;
        break;

    case BAUDRATE_38400:
        uBaudRate = 38400;
        break;

    case BAUDRATE_115200:
        uBaudRate = 115200;
        break;

    default:
        target = BAUDRATE_9600;
        uBaudRate = 9600;
        break;
    }

    UartInit(SLAVE_UART_USE,uBaudRate,UART_PARITY_NONE);      // 串口初始化

    return target;
}
/******************************************************************************
* 名    称：ResetIecData()
* 功    能：重置南向查询数据。
* 入口参数：
            无
*
* 出口参数：无
* 范    例:
******************************************************************************/
void ResetIecData(uint8_t uRelAddr)
{
    uint16_t iec_addr; //
    uint16_t count=0;
    uint16_t yc_sum=0;   // 遥测点数统计
    uint16_t yx_sum=0;   // 遥信点数统计
    uint8_t  rel_num;


    rel_num = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // 相对点表号
    /*for(count=0; count<g_DeviceSouth.protocol[rel_num].mess_point_sum; count++)
    {
        if(TYPE_YC==Reg_point[rel_num][count].reg_type.type.mess)
        {
            yc_sum++;
        }
        else if(TYPE_YX==Reg_point[rel_num][count].reg_type.type.mess)
        {
            yx_sum++;
        }
    }*/
    yc_sum = g_sIecPointCount[rel_num].uYcSum;
    yx_sum = g_sIecPointCount[rel_num].uYxSum;


    if(g_DeviceSouth.device_inf[uRelAddr].yx_start_addr >= 1)
    {
        iec_addr = g_DeviceSouth.device_inf[uRelAddr].yx_start_addr - 1; // 找出遥信起始地址
        count  = iec_addr;           // 地址偏移，得出第一个点对应的iec104数组表下标
        yx_sum = yx_sum + iec_addr;  // 加上地址偏移，得出最后一个点对应的iec104数组表下标
        for(; count<yx_sum && NULL!=IEC104_DATA_YX; count++)
        {
            IEC104_DATA_YX[count] = 0xff;
        }
    }

    if(g_DeviceSouth.device_inf[uRelAddr].yc_start_addr>=0x4001)
    {
        iec_addr = g_DeviceSouth.device_inf[uRelAddr].yc_start_addr - 0x4001; // 找出遥测起始地址
        count  = iec_addr;           // 地址偏移，得出第一个点对应的iec104数组表下标
        yc_sum = yc_sum + iec_addr;  // 加上地址偏移，得出最后一个点对应的iec104数组表下标
        for(; count<yc_sum && NULL!=IEC104_DATA_YC; count++)
        {
            IEC104_DATA_YC[count] = 0xffffffff;
        }
    }
}
/******************************************************************************
* 名    称：CheckDevState()
* 功    能：检查南向设备状态。
* 入口参数：
            无
*
* 出口参数：-1：没有设备  0：有且没有全部丢帧  其他：全部丢帧的位标记
* 范    例:
******************************************************************************/
int8_t CheckDevState(void)
{
    uint8_t i;
    uint8_t uDevExist=0;  // 有设备
    uint8_t uAllLost=0;   // 设备丢帧

    for(i=0;i<MAX_device;i++)
    {
        if(g_DeviceSouth.device_inf[i].addr)
        {
            uDevExist |= (1<<i);

            if(g_LoggerRun.err_lost&(1<<i))
            {
                uAllLost |= (1<<i);
            }
        }
    }

    if(0==uDevExist)        //无设备
    {
        return -1;
    }
    else if(0==(uDevExist&uAllLost))    //所有设备都在线
    {
        return 0;
    }
    else                                //有设备断链告警
    {
        return (uDevExist&uAllLost);        //三方设备晚上会关机，但仍然得记录历史数据，便于补采搜索
    }

}
/**************************************************************************************
* 名    称： TimedRestart()
* 功    能： 定时重启，每天1点26分1秒定时随机0~120分钟后重启数采，前提是数采已跟平台对时过
* 入口参数：uRandom：随机最大值
* 出口参数：随机函数产生的实际分钟数(0~uRandom)
* 作者:		
* 日期：		
**************************************************************************************/
uint8_t TimedReboot(uint8_t uRandom)
{
	static uint8_t uRandTime = 121;	// 随机分钟数，0-120
	static uint8_t uRebotMark = 0;    
    uint16_t uRandSeeds = 0;			// 随机种子，用于产生随机数
    uint32_t uRandNum = 0;				// 随机数，用于产生随机分钟数
    static uint32_t uStartTime = 0;
	uint32_t uNowTime;
    SYSTEMTIME *pGetTime;
    pGetTime = RealTimeGet();
	g_uTimeNow = OSTimeGet();   // 获取当前的时间戳
    if(1 == pGetTime->Hour)
    {
        if((1 == pGetTime->Minute) && (uRebotMark == 0))
        {
           // if(1==pGetTime->Second)
            {
			     uRebotMark = 1;
				 uRandSeeds = CalculateCRC((uint8_t*)g_LoggerInfo.esn,20);	//以ESN的CRC作为随机种子
                 srand(uRandSeeds);                                        //设置随机数种子
                 uRandTime = 0;
				
                 uRandNum = rand();                                        //生成随机数
                 uRandTime = uRandNum%uRandom;
				 uStartTime = g_uTimeNow;
                 DEBUGOUT("######%d分钟后定时重启######\n",uRandTime);
            }
        }
    }

    if(uRandTime < uRandom)
    {
        uNowTime = g_uTimeNow;
	    if(TimeGapJudge(uStartTime,uNowTime,uRandTime*60000))
        {
            msleep(100);
        }
		else
		{
			OSTimeDlyHMSM(0,1,5,0);
			DEBUGOUT("######定时时间到开始重启!!!######\n");
			uRebotMark = 0;
            Reboot();
		}
    } 
    return uRandTime;
}
