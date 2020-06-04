#include "IEC104.h"
#include "GlobalVar.h"
#include "DataTransferArea.h"
#include "Memory.h"
#include "RealTime.h"
#include "DataFlash.h"
#include "InEEPROM.h"
#include "Record.h"
#include "CRC16.h"
#include "SlaveUnit.h"
#include "tool.h"
#include "log.h"

#include "SubCollection.h"
//========================================================================
#define JUDGE(x)  ((x)==0?0:1)  // 判断数值是否为0，为0返回0，非0返回1
//========================================================================
#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // 延时x秒
//========================================================================
#define MAX_POINT_ONE_TABLE    315                   // 一张点表最大点数，申请临时点表空间用
//========================================================================
#define LOGFRAMELENTH          200                   // 日志上传一帧发送的字节数
//========================================================================
typedef struct
{
    uint16_t uLastTableNum;       // 上次系统发送过来的点表号
    uint16_t uLastPointCount;     // 上次已经发送的信息点，点数统计
    uint8_t  uRelPoint;           // 相对点表号
    uint8_t  uRelDev;             // 设备信息结构体数组下标，相对设备通讯地址
} POINT_RECORD_TEMP_T;

typedef union
{
    uint16_t uInfAddr;
    uint8_t uInfTemp[2];
}LOG_INF_ADDE;

//========================================================================
//========================================================================
uint8_t  *IEC104_DATA_YX;   // IEC104数据-遥信指针，动态申请空间
uint32_t *IEC104_DATA_YC;   // IEC104数据-遥测指针，动态申请空间
uint8_t  *IEC104_DATA_YK;   // IEC104数据-遥控指针，动态申请空间
uint32_t *IEC104_DATA_SD;   // IEC104数据-设点指针，动态申请空间
uint32_t *IEC104_DATA_DD;   // IEC104数据-电度指针，动态申请空间
uint16_t     preFrameDataCrc = 0;       // 上一次的I帧数据CRC值
//========================================================================
IEC104_COUNT_T g_sIecPointCount[MAX_device];  // 点表信息点统计

IEC104_MAIN_T g_sIEC;
IEC_RUNNING_T g_sIecRun;

uint16_t      g_IecRecSeq=0;          // 接收序列
uint16_t      g_IecSendSeq=0;         // 发送序列
uint8_t 	  Data_resend_count = 0;
//========================================================================
static uint8_t g_uReportCtrl=0;                    // 发送控制标记
static uint8_t g_TimeMarker[7] = {0};              // 补采数据需要上报的时标
static uint8_t g_uSubData[YCBUFFSIZE];             // 用于补采

static LOGGER_MODBUS_REG_T *pRegPointTemp=NULL;    // 临时导点表用的点表指针
static POINT_RECORD_TEMP_T s_sPointRecord;         // 记录导表的信息

static uint32_t g_uNowTime;     // 当前时标。
static uint32_t g_uUpdataTime;  // 升级时标，用于远程升级超级保护
//static uint32_t g_uDT1000UpdataTime;  //表计升级时标，用于远程升级表计超时保护

static uint8_t  s_uSouthYk=SOUTH_CMD_YK;       // 南向遥控
static uint8_t  s_uSouthSd=SOUTH_CMD_SD;       // 南向设点
static uint8_t  s_uSouthSync=SOUTH_CMD_SYNC;   // 南向同步时间
static uint8_t  s_uSouthReadSd=SOUTH_CMD_READSD; //南向读设点

//========================================================================
void IecCollectProcess(IEC104_MAIN_T *pA,IEC_RUNNING_T *call);  // 总招
void IecSubSCollect(IEC104_MAIN_T *pA,IEC_RUNNING_T *call);     // 补采
void Iec104HandleHwSoftRenovate(IEC104_MAIN_T *pA);
//========================================================================
// 清除上报控制
void ReportCtrlClear(uint8_t uCtrl)
{
    g_uReportCtrl &= ~uCtrl;
}
// 设置上报控制
void ReportCtrlSet(uint8_t uCtrl)
{
    g_uReportCtrl |= uCtrl;
}
// 读上报控制
uint8_t ReportCtrlRead(uint8_t uCtrl)
{
    return (g_uReportCtrl&uCtrl)?1:0;
}
/******************************************************************************
* 名    称：IecInit()
* 功    能：IEC104  初始化。
* 入口参数：
*           无

* 出口参数：无
* 范    例:
******************************************************************************/
void IecInit(void)
{
    uint16_t i,uCcount;
    uint16_t uYxCount;
    uint16_t uYcCount;
    uint16_t uYkCount;
    uint16_t uSdCount;

    for(i=0;i<MAX_device;i++)
    {
        if(NULL!=g_pRegPoint[i])
        {
            uYxCount = 0;
            uYcCount = 0;
            uYkCount = 0;

            for(uCcount=0,uYcCount=0,uYxCount=0; uCcount<g_DeviceSouth.protocol[i].mess_point_sum; uCcount++)
            {
                if(TYPE_YC==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uYcCount++;
                }
                else if(TYPE_YX==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uYxCount++;
                }
                else if(TYPE_YK==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uYkCount++;
                }
                else if(TYPE_SD==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uSdCount++;
                }
            }
            g_sIecPointCount[i].uYxSum = uYxCount;
            g_sIecPointCount[i].uYcSum = uYcCount;
            g_sIecPointCount[i].uYkSum = uYkCount;
            g_sIecPointCount[i].uSdSum = uSdCount;
			//DEBUGOUT("g_sIecPointCount[%d].uSdSum:%d",i,g_sIecPointCount[i].uSdSum);
        }
    }
    //--------------------------------------
    // IEC104表，内存空间初始化
    if(g_DeviceSouth.yx_sum>0)       // 申请遥信数据内存空间
    {
        IEC104_DATA_YX = (uint8_t*)WMemMalloc(IEC104_DATA_YX,g_DeviceSouth.yx_sum*sizeof(uint8_t));
        if(NULL==IEC104_DATA_YX)
        {
            DEBUGOUT("要遥信空间失败\n");
            return;
        }
        for(i=0;i<g_DeviceSouth.yx_sum;i++)
        {
            IEC104_DATA_YX[i] = 0xFF;
        }
    }
    else
    {
        IEC104_DATA_YX = NULL;
    }

    if(g_DeviceSouth.yc_sum>0)       // 申请遥测数据内存空间
    {
        IEC104_DATA_YC = (uint32_t*)WMemMalloc(IEC104_DATA_YC,g_DeviceSouth.yc_sum*sizeof(uint32_t));
        if(NULL==IEC104_DATA_YC)
        {
            DEBUGOUT("要遥测空间失败\n");
            return;
        }
        for(i=0;i<g_DeviceSouth.yc_sum;i++)
        {
            IEC104_DATA_YC[i] = 0xFFFFFFFF;
        }
    }
    else
    {
        IEC104_DATA_YC = NULL;
    }


    if(g_DeviceSouth.yk_sum>0)       // 申请遥控数据内存空间
    {
        IEC104_DATA_YK = (uint8_t*)WMemMalloc(IEC104_DATA_YK,g_DeviceSouth.yk_sum*sizeof(uint8_t));
        if(NULL==IEC104_DATA_YK)
        {
            DEBUGOUT("要遥控空间失败\n");
            return;
        }
        for(i=0;i<g_DeviceSouth.yk_sum;i++)
        {
            IEC104_DATA_YK[i] = 0x00;
        }
    }
    else
    {
        IEC104_DATA_YK = NULL;
    }

    if(g_DeviceSouth.sd_sum>0)       // 申请设点数据内存空间
    {
        IEC104_DATA_SD = (uint32_t*)WMemMalloc(IEC104_DATA_SD,g_DeviceSouth.sd_sum*sizeof(uint32_t));
        if(NULL==IEC104_DATA_SD)
        {
            DEBUGOUT("要设点空间失败\n");
            return;
        }
    }
    else
    {
        IEC104_DATA_SD = NULL;
    }

    if(g_DeviceSouth.dd_sum>0)       // 申请电度数据内存空间
    {
        IEC104_DATA_DD = (uint32_t*)WMemMalloc(IEC104_DATA_DD,g_DeviceSouth.dd_sum*sizeof(float));
        if(NULL==IEC104_DATA_DD)
        {
            DEBUGOUT("要电度空间失败\n");
            return;
        }
    }
    else
    {
        IEC104_DATA_DD = NULL;
    }

    g_LoggerRun.uDevExist = 0;
    for(i=0;i<MAX_device;i++)
    {
        if(g_DeviceSouth.device_inf[i].addr)
        {
            g_LoggerRun.uDevExist |= 1<<i;
        }
    }
}
/******************************************************************************
* 名    称：Iec104FrameSend()
* 功    能：发送数据到发送链表。
* 入口参数：
            bytes     数据长度
            *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
/*tatic void Iec104FrameSend(IEC104_MAIN_T *pA,uint8_t bytes)
{
    CmStoreSendLink((uint8_t *)&pA->send.buff,bytes);  // 存储到链表
}*/
/******************************************************************************
* 名    称：Iec104CreateFrameI()
* 功    能：IEC104 组成I帧。
* 入口参数：
*           type      类型标识
            limit     可变结构限定词
            reason    传输原因
            *data     数据指针
            bytes     数据长度  data段数据长度
            *pA       IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104CreateFrameI2(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC104_MAIN_T *pA)
{
    Uint_Char_Convert temp;

    pA->send.format.start   = IEC104_HEAD;  // 启动符
    pA->send.format.len     = 13 + bytes;   // 长度
    temp.u = g_IecSendSeq++;
    pA->send.format.SseqL   = temp.c[0];    // 发送序号字节1
    pA->send.format.SseqH   = temp.c[1];    // 发送序号字节1
    temp.u = g_IecRecSeq;
    pA->send.format.RseqL   = temp.c[0];    // 接收序号字节1
    pA->send.format.RseqH   = temp.c[1];    // 接收序号字节2
    pA->send.format.type    = type;         // 类型标识
    pA->send.format.limit   = limit;        // 可变结构限定词
    pA->send.format.reasonL = reason&0xff;  // 传输原因字节1
    pA->send.format.reasonH = reason>>8;    // 传输原因字节2
    pA->send.format.addrL   = g_LoggerInfo.ADDR;         // 公共地址低字节
    pA->send.format.addrH   = 0x00;         // 公共地址高字节

    //Iec104FrameSend(pA,pA->send.format.len+2);
    //CmStoreSendLink((uint8_t *)&pA-> buff,pA->format.len+2);  // 存储到链表
}
/******************************************************************************
* 名    称：IecCreateFrameI()
* 功    能：IEC104 组成I帧。
* 入口参数：
*           type      类型标识
            limit     可变结构限定词
            reason    传输原因
            *data     数据指针
            bytes     数据长度  data段数据长度
            *pA       IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void IecCreateFrameI(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC_FORMAT_T *pA)
{
    Uint_Char_Convert temp;

    pA->format.start   = IEC104_HEAD;  // 启动符
    pA->format.len     = 13 + bytes;   // 长度
    temp.u = g_IecSendSeq<<1;
    pA->format.SseqL   = temp.c[0];    // 发送序号字节1
    pA->format.SseqH   = temp.c[1];    // 发送序号字节1
    temp.u = g_IecRecSeq<<1;
    pA->format.RseqL   = temp.c[0];    // 接收序号字节1
    pA->format.RseqH   = temp.c[1];    // 接收序号字节2
    pA->format.type    = type;         // 类型标识
    pA->format.limit   = limit;        // 可变结构限定词
    pA->format.reasonL = reason&0xff;  // 传输原因字节1
    pA->format.reasonH = reason>>8;    // 传输原因字节2
    pA->format.addrL   = g_LoggerInfo.ADDR;         // 公共地址低字节
    pA->format.addrH   = 0x00;         // 公共地址高字节

    //Iec104FrameSend(pA,pA->format.len+2);
    CmStoreSendLink((uint8_t *)&pA->buff,pA->format.len+2);  // 存储到链表

    g_IecSendSeq++;
}
/******************************************************************************
* 名    称：IecReportLogInfo()
* 功    能：连接到服务器后发送数采信息。
* 入口参数：
*           uint8_t uReason         传输原因

* 出口参数：无
* 范    例:
******************************************************************************/
void IecReportLogInfo(uint8_t uReason)//(IEC104_MAIN_T *pA)
{
    g_sIEC.send.format.maddrL = 0x00;// 信息体地址
    g_sIEC.send.format.maddrM = 0x00;
    g_sIEC.send.format.maddrH = 0x00;

    memset(g_sIEC.send.format.data, 0, 80);//memset(g_sIEC.ibuff, 0, 80);

    memcpy(&g_sIEC.send.format.data[0],g_LoggerInfo.name,20);// 设备名称0~19

    memcpy(&g_sIEC.send.format.data[20],g_LoggerInfo.esn,20); // ESN号20~39//strncpy(info,g_LoggerInfo.esn,20);  // ESN号

    memcpy(&g_sIEC.send.format.data[40],g_LoggerInfo.model,20);// 设备型号40~59

    memcpy(&g_sIEC.send.format.data[60],g_LoggerInfo.type,20);// 设备类型60~79

    g_sIEC.send.format.data[80] = 0;//g_LoggerRun.IP[0];// IP地址80~83
    g_sIEC.send.format.data[81] = 0;//g_LoggerRun.IP[1];
    g_sIEC.send.format.data[82] = 0;//g_LoggerRun.IP[2];
    g_sIEC.send.format.data[83] = 0;//g_LoggerRun.IP[3];

    if(RUNNING_WORK_READ==g_LoggerRun.run_status)
    {
        g_sIEC.send.format.data[84] = 0x01;    // 运行状态84
    }
    else
    {
        g_sIEC.send.format.data[84] = 0x00;    //0x00
    }


    g_sIEC.send.format.data[85] = GetVerS2(); // 软件版本85~87
    g_sIEC.send.format.data[86] = GetVerS1();
    g_sIEC.send.format.data[87] = GetVerType();

    IecCreateFrameI(P_DEV_INFO,1,uReason,88,&g_sIEC.send);
}


/******************************************************************************
* 名    称：IecProcessFrameS()
* 功    能：IEC104 S帧处理。
* 入口参数：
*           *pA         IEC信息指针
            dir         S帧收发，1：发送S帧，0：收到S帧
* 出口参数：无
* 范    例:
******************************************************************************/
static void IecProcessFrameS(IEC104_MAIN_T *pA,uint8_t dir)
{
    Uint_Char_Convert temp;
    if(dir)
    {
        pA->send.format.start = IEC104_HEAD;
        pA->send.format.len   = 0x04;
        pA->send.format.SseqL = IEC104_S_MAK;
        pA->send.format.SseqH = 0x00;
        temp.u = g_IecRecSeq;
        pA->send.format.RseqL = temp.c[0];
        pA->send.format.RseqH = temp.c[1];

        //Iec104FrameSend(pA,6);
        CmStoreSendLink((uint8_t *)&pA->send.buff,6);  // 存储到链表
    }
    else
    {
        if(g_LoggerRun.update)
        {
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_S_MAK;
            pA->send.format.SseqH = 0x00;
            temp.u = g_IecRecSeq;//pA->RecSeq << 1;
            pA->send.format.RseqL = temp.c[0];
            pA->send.format.RseqH = temp.c[1];

            //Iec104FrameSend(pA,6);
            CmStoreSendLink((uint8_t *)&pA->send.buff,6);  // 存储到链表
        }

    }
}
/******************************************************************************
* 名    称：IecProcessFrameU()
* 功    能：IEC104 U帧处理。
* 入口参数：
*           *pA         IEC信息指针
            dir         U帧收发，1：发送U帧，0：收到U帧
* 出口参数：无
* 范    例:
******************************************************************************/
static void IecProcessFrameU(IEC104_MAIN_T *pA,uint8_t cmd,uint8_t dir)
{
    if(0==dir)
    {
        switch(cmd)
        {
        case IEC104_U_STARTDT:
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_U_STARTDT_ACK | IEC104_U_MAK;
            pA->send.format.SseqH = 0x00;
            pA->send.format.RseqL = 0x00;
            pA->send.format.RseqH = 0x00;

            break;

        case IEC104_U_STARTDT_ACK:
            break;

        case IEC104_U_STOPDT:
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_U_STOPDT_ACK | IEC104_U_MAK;
            pA->send.format.SseqH = 0x00;
            pA->send.format.RseqL = 0x00;
            pA->send.format.RseqH = 0x00;
            break;

        case IEC104_U_STOPDT_ACK:
            break;

        case IEC104_U_TESTFR:
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_U_TESTFR_ACK | IEC104_U_MAK;
            pA->send.format.SseqH = 0x00;
            pA->send.format.RseqL = 0x00;
            pA->send.format.RseqH = 0x00;
            break;

        case IEC104_U_TESTFR_ACK:
            break;

        default:
            break;
        }
    }
    else
    {
        pA->send.format.start = IEC104_HEAD;
        pA->send.format.len   = 0x04;
        pA->send.format.SseqL = cmd | IEC104_U_MAK;
        pA->send.format.SseqH = 0x00;
        pA->send.format.RseqL = 0x00;
        pA->send.format.RseqH = 0x00;

        //Iec104FrameSend(pA,6);
    }

    //Iec104FrameSend(pA,6);
    CmStoreSendLink((uint8_t *)&pA->send.buff,6);  // 存储到链表
}
/******************************************************************************
* 名    称：Iec104ReportMaxDevice()
* 功    能：上报下联设备最大值。
* 入口参数：
*           uint8_t uReason     传输原因

* 出口参数：
* 范    例:
******************************************************************************/
void Iec104ReportMaxDevice(uint8_t uReason,IEC104_MAIN_T *pA)
{
    pA->send.format.maddrL = 0x00;
    pA->send.format.maddrM = 0x00;
    pA->send.format.maddrH = 0x00;
    pA->send.format.data[0] = MAX_device;

    IecCreateFrameI(P_MAX_DEVICE,1,uReason,1,&pA->send);
}
/******************************************************************************
* 名    称：IecCpoyMesAddr()
* 功    能：将收到的IEC数据的信息体地址复制到IEC发送数据的信息体地址。
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void IecCpoyMesAddr(IEC104_MAIN_T *pA)
{
    pA->send.format.maddrL = pA->recv.format.maddrL;
    pA->send.format.maddrM = pA->recv.format.maddrM;
    pA->send.format.maddrH = pA->recv.format.maddrH;
}
//根据输入的设备编号地址返回设备对应数组下标
uint8_t IecAddrConvert(uint8_t MbVirAddr)
{
	//to do
	uint8_t uMbRelAddr=0xff; // 如果搜索不到对应的地址，返回0xFF，即表示没有相应的设备
    uint8_t i=0;
    if(!MbVirAddr)      //通讯地址不存在
    {
        return uMbRelAddr;
    }
    for(i=0; i<MAX_device; i++)
    {
        if(g_DeviceSouth.device_inf[i].addr==MbVirAddr)
        {
            uMbRelAddr=i;
            break;
        }
    }

	return uMbRelAddr;
}
/******************************************************************************
* 名    称：Iec104SyncTime()
* 功    能：对时。67
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104SyncTime(IEC104_MAIN_T *pA)
{
    uint8_t uBit=0;
    Uint_Char_Convert temp;
    SYSTEMTIME sSysTime;
    SYSTEMTIME *pGetTime;
    static uint8_t uSysTimeMark=0;
	RealTimeInit();
    if(R_ACTIVE==pA->recv.format.reasonL) // 激活06
    {
        temp.c[0] = pA->recv.format.data[0]; //pA->rec_buff[15];  // 毫秒低位
        temp.c[1] = pA->recv.format.data[1]; //pA->rec_buff[16];  // 毫秒高位

        sSysTime.Second = temp.u/1000;
        sSysTime.Minute = pA->recv.format.data[2]; //pA->rec_buff[17]; // 分
        sSysTime.Hour   = pA->recv.format.data[3]; //pA->rec_buff[18]; // 时
        sSysTime.Date   = pA->recv.format.data[4]&0x1f; //pA->rec_buff[19]; // 日，，高3位为星期
        sSysTime.Month  = pA->recv.format.data[5]; //pA->rec_buff[20]; // 月
        sSysTime.Year   = pA->recv.format.data[6]; //pA->rec_buff[21]; // 年
        sSysTime.Week   = pA->recv.format.data[4]>>5; //pA->rec_buff[19]; // 日，，高3位为星期
        
        msleep(1);
        RealTimeSet(&sSysTime);// 设置到时钟芯片
        msleep(10);
        //HwDeviceTime_Set();
        pGetTime = RealTimeGet();

        DEBUGOUT("对时%d:%d-%d-%d %d:%d:%d 周%d\n",uSysTimeMark,
                 pGetTime->Year,pGetTime->Month,pGetTime->Date,pGetTime->Hour,pGetTime->Minute,pGetTime->Second,pGetTime->Week);
        if((uSysTimeMark < 3) && (pGetTime->Year==226) && (pGetTime->Month==1) && (pGetTime->Date==1))
        {
            uSysTimeMark++;
            RealTimeInit();
            msleep(1);
            RealTimeSet(&sSysTime);// 设置到时钟芯片
            msleep(1);
            //HwDeviceTime_Set();
            pGetTime = RealTimeGet();

            DEBUGOUT("对时%d:%d-%d-%d %d:%d:%d 周%d\n",uSysTimeMark,
                     pGetTime->Year,pGetTime->Month,pGetTime->Date,pGetTime->Hour,pGetTime->Minute,pGetTime->Second,pGetTime->Week);
            //continue;
        }
        
       /* if(uSysTimeMark >= 3)
        {
           sleep(2);
           Reboot();
        }*/
        
        IecCpoyMesAddr(pA);// 复制信息体地址

        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
        IecCreateFrameI(C_CS_NA_1,pA->recv.format.limit,R_ACTIVE_ACK,(pA->recv.format.len-13),&pA->send);

        g_LoggerRun.north_status = NORTH_OK;  // 北向通讯服务器连接，收到对时帧数据

        Iec104ReportMaxDevice(R_INFO_REPORT,pA);
        //Slave_Init(g_LoggerInfo.inquire_interval_time*100,1);     // 南向查询初始化
        for(uBit=0;uBit<MAX_device;uBit++)
        {
            if(g_LoggerRun.err_lost&(1<<uBit))
            {
                AlarmReport(g_DeviceSouth.device_inf[uBit].addr,              // 设备通讯地址
                                             0x01,                                 // 南向设备通讯异常
                                             0x0000,                               // MODBUS寄存器地址
                                             0x00,                                 // 偏移量
                                             JUDGE(g_LoggerRun.err_lost&(1<<uBit)), // 告警产生或恢复
                                             1);    // 不立即上报
                //g_LoggerAlarm.dev_lost &= ~(1<<uBit);              // 清除南向设备断链告警标记，会重新上报
            }
        }
        //soft_timer_stop(&OnTime);

        OSQPost(MesQ, &s_uSouthSync);
    }
}
/******************************************************************************
* 名    称：Iec104InputTable()
* 功    能：导入点表。BB 88 89 8A
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104InputTable(IEC104_MAIN_T *pA)
{
    uint16_t i,j;
    uint8_t k;
    Uint_Char_Convert temp;

    if(R_INPUT_GLO_INFO==pA->recv.format.reasonL)// 传输原因-导入全局信息0x88
    {
        g_LoggerRun.run_status = RUNNING_INPUT_GOLB;  // 标记数采状态为导入全局信息

		g_DeviceSouth.device_sum = pA->recv.format.data[0];  // 设备总数
		g_DeviceSouth.yx_sum     = pA->recv.format.data[2]<<8 | pA->recv.format.data[1];  // 遥信总数
		g_DeviceSouth.yc_sum     = pA->recv.format.data[4]<<8 | pA->recv.format.data[3];  // 遥信总数
		g_DeviceSouth.yk_sum     = pA->recv.format.data[6]<<8 | pA->recv.format.data[5];  // 遥信总数
		g_DeviceSouth.sd_sum     = pA->recv.format.data[8]<<8 | pA->recv.format.data[7];  // 遥信总数
		g_DeviceSouth.dd_sum     = pA->recv.format.data[10]<<8 | pA->recv.format.data[9]; // 遥信总数

		IecCpoyMesAddr(pA);

		memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
		IecCreateFrameI(P_INPUT_TABLE,pA->recv.format.limit,R_INPUT_GLO_ACK,(pA->recv.format.len-13),&pA->send);
		//--------------------------------------------------------------------------------------------------
		//iec_run.running = Iec104_status_process(iec_run.running,P_INPUT_TABLE,R_INPUT_GLO_INFO);//RUNNING_INPUT_GLO;
		g_LoggerRun.run_status = RUNNING_INPUT_GOLB;
    }
    else if(R_INPUT_TYPE_INFO==pA->recv.format.reasonL)// 传输原因-导入设备类型信息0x89
    {
    	g_LoggerRun.run_status = RUNNING_INPUT_TABLE;  // 标记数采状态为导入点表信息
		if(NULL==pRegPointTemp)
		{
			pRegPointTemp = (LOGGER_MODBUS_REG_T*)WMemMalloc(pRegPointTemp,MAX_POINT_ONE_TABLE * sizeof(LOGGER_MODBUS_REG_T));   // 申请空间

			s_sPointRecord.uLastTableNum = 0;
			s_sPointRecord.uRelPoint = 0;
			s_sPointRecord.uLastPointCount = 0;

			if(NULL==pRegPointTemp)
			{
//				DEBUGOUT("要点表临时空间失败\n");// 申请空间失败
				return;
			}
		}

		temp.c[0] = pA->recv.format.maddrL;
		temp.c[1] = pA->recv.format.maddrM;

		if(s_sPointRecord.uLastTableNum != temp.u)
		{
			if(0!=s_sPointRecord.uLastTableNum)  // 记录的点表号非零，上一张点表传输完成
			{
				g_pRegPoint[s_sPointRecord.uRelPoint] = (LOGGER_MODBUS_REG_T*)WMemMalloc(g_pRegPoint[s_sPointRecord.uRelPoint],s_sPointRecord.uLastPointCount*sizeof(LOGGER_MODBUS_REG_T));   // 申请空间

				if(NULL!=g_pRegPoint[s_sPointRecord.uRelPoint])
				{
					memcpy(g_pRegPoint[s_sPointRecord.uRelPoint],pRegPointTemp,sizeof(LOGGER_MODBUS_REG_T)*s_sPointRecord.uLastPointCount);
				}
				else
				{
					DEBUGOUT("要点表空间失败1\n");
				}

				s_sPointRecord.uRelPoint++;
				s_sPointRecord.uLastTableNum = temp.u;  // 记录本次点表号
				s_sPointRecord.uLastPointCount = 0;        // 已经记录的信息点数清零
			}
			else // 头一张点表
			{
				s_sPointRecord.uLastTableNum = temp.u;  // 记录本次点表号
			}

			for(i=1,j=s_sPointRecord.uLastPointCount; i<pA->recv.format.len-14; j++) //
			{
				pRegPointTemp[j].reg_addr           = pA->recv.format.data[i+1]<<8 | pA->recv.format.data[i];  // 寄存器地址
				pRegPointTemp[j].reg_type.type.mess = pA->recv.format.data[i+2];   // 信息类型，遥信、遥测……
				pRegPointTemp[j].reg_count          = pA->recv.format.data[i+3];   // 信息点长度
				pRegPointTemp[j].reg_type.type.data = pA->recv.format.data[i+4];   // 数据类型，整型、浮点、字符串……
				i += 5;
			}
			s_sPointRecord.uLastPointCount = j;
		}
		else //继续传输点表
		{
			if(Data_resend_count > 1)
			{
				g_LoggerRun.run_status = RUNNING_WORK_READ;  	//数采运行状态-导表完成开始正常工作
				return;
			}

			if(preFrameDataCrc != CalculateCRC(pA->recv.format.data,(pA->recv.format.len-13)))
			{
				preFrameDataCrc = CalculateCRC(pA->recv.format.data,(pA->recv.format.len-13));
				for(i=1,j=s_sPointRecord.uLastPointCount; i<pA->recv.format.len-14; j++)
				{
					if(j <= MAX_POINT_ONE_TABLE)
					{
						pRegPointTemp[j].reg_addr           = pA->recv.format.data[i+1]<<8 | pA->recv.format.data[i];  //寄存器地址
						pRegPointTemp[j].reg_type.type.mess = pA->recv.format.data[i+2];   //信息类型，遥信、遥测……
						pRegPointTemp[j].reg_count          = pA->recv.format.data[i+3];   //信息点长度
						pRegPointTemp[j].reg_type.type.data = pA->recv.format.data[i+4];   //数据类型，整型、浮点、字符串……
						i += 5;
					}
					else
					{
						DEBUGOUT("\nSignal point overshoot!\nAll signal location：%d\n",s_sPointRecord.uLastPointCount);
						j -= 1;
						return;
					}
				}
				s_sPointRecord.uLastPointCount = j;
			}
			else
			{
				Data_resend_count += 1;
				g_LoggerRun.run_status = RUNNING_WORK_READ;  	//数采运行状态-导表完成开始正常工作
				return;
			}
		}

		Data_resend_count = 0;
		g_DeviceSouth.protocol[s_sPointRecord.uRelPoint].protocol_num   = s_sPointRecord.uLastTableNum;  	//点表号
		g_DeviceSouth.protocol[s_sPointRecord.uRelPoint].protocol_type  = pA->recv.format.data[0];         	//点表类型
		g_DeviceSouth.protocol[s_sPointRecord.uRelPoint].mess_point_sum = s_sPointRecord.uLastPointCount;   //信息点总数
		DEBUGOUT("\nAll signal location：%d\n",s_sPointRecord.uLastPointCount);

		IecCpoyMesAddr(pA);
		memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));		// I帧数据
		IecCreateFrameI(P_INPUT_TABLE,pA->recv.format.limit,R_INPUT_TYPE_ACK,(pA->recv.format.len-13),&pA->send);

		g_LoggerRun.run_status = RUNNING_INPUT_TABLE;
    }
    else if(R_INPUT_DEV_INFO==pA->recv.format.reasonL)// 传输原因-导入设备信息0x8A
    {
        g_LoggerRun.run_status = RUNNING_INPUT_104;  // 标记数采状态为导入设备104表信息

        //for(i=0,j=0; i<(pA->recv.format.len-10) && j<MAX_device; j++)
        for(i=0,j=0; i<(pA->recv.format.len-10);)
        {
            j = IecAddrConvert(pA->recv.buff[i+12]);
            if(0xff==j) // 没有相应的设备
            {
                i += 15;
                continue;
            }
            //g_DeviceSouth.device_inf[j].addr          = pA->recv.buff[i+12];  // 通讯地址
            g_DeviceSouth.device_inf[j].protocol_num  = pA->recv.buff[i+16]<<8 | pA->recv.buff[i+15];  // 绝对点表号
            g_DeviceSouth.device_inf[j].yx_start_addr = pA->recv.buff[i+18]<<8 | pA->recv.buff[i+17];  // 遥信起始地址
            g_DeviceSouth.device_inf[j].yc_start_addr = pA->recv.buff[i+20]<<8 | pA->recv.buff[i+19];  // 遥测起始地址
            g_DeviceSouth.device_inf[j].yk_start_addr = pA->recv.buff[i+22]<<8 | pA->recv.buff[i+21];  // 遥控起始地址
            g_DeviceSouth.device_inf[j].sd_start_addr = pA->recv.buff[i+24]<<8 | pA->recv.buff[i+23];  // 设点起始地址
            g_DeviceSouth.device_inf[j].dd_start_addr = pA->recv.buff[i+26]<<8 | pA->recv.buff[i+25];  // 电度起始地址
            i += 15;

            // 找出相对点表号
            for(k=0; k<MAX_device; k++)
            {
                if(g_DeviceSouth.device_inf[j].protocol_num == g_DeviceSouth.protocol[k].protocol_num)
                {
                    g_DeviceSouth.device_inf[j].rel_num = k;  // 点表相对地址，存储点表的数组下标
                    g_DeviceSouth.device_inf[j].protocol_type = g_DeviceSouth.protocol[k].protocol_type;
                    break;
                }
            }
        }

        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
        IecCreateFrameI(P_INPUT_TABLE,pA->recv.format.limit,R_INPUT_DEV_ACK,(pA->recv.format.len-13),&pA->send);
        //--------------------------------------------------------------------------------------------------
        //iec_run.running = Iec104_status_process(iec_run.running,P_INPUT_TABLE,R_INPUT_DEV_INFO);
        g_LoggerRun.run_status = RUNNING_INPUT_104;
    }
}
/******************************************************************************
* 名    称：Iec104TableState()
* 功    能：导表指令。C5
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104TableState(IEC104_MAIN_T *pA)
{
    //static uint8_t s_uTableStart=0;
    uint8_t  i,temp,rel_num;
    uint16_t j;
    uint16_t space;  // 占用空间，字节数
    uint32_t addr;   // 存储地址

    if(0x01==pA->recv.format.data[0] ) // 启动导表
    {
        g_LoggerRun.run_status = RUNNING_INPUT_START;  // 标记数采状态为启动导表

        //s_uTableStart = 1;
    }
    else if(0x04==pA->recv.format.data[0])// && s_uTableStart) // 导表结束
    {
        //s_uTableStart = 0;

        if(s_sPointRecord.uLastPointCount)  // 导入的最后一张点表，还没有复制到点表数组
        {
            //g_pRegPoint[s_sPointRecord.uRelPoint] = (LOGGER_MODBUS_REG_T*)calloc(s_sPointRecord.uLastPointCount,sizeof(LOGGER_MODBUS_REG_T));   // 申请空间

            /*if(NULL!=g_pRegPoint[s_sPointRecord.uRelPoint]) // 如果空间指针非空，先释放空间，再申请空间
            {
                g_pRegPoint[s_sPointRecord.uRelPoint] = WMemFree(g_pRegPoint[s_sPointRecord.uRelPoint]);
            }*/

            g_pRegPoint[s_sPointRecord.uRelPoint] = (LOGGER_MODBUS_REG_T*)WMemMalloc(g_pRegPoint[s_sPointRecord.uRelPoint],s_sPointRecord.uLastPointCount*sizeof(LOGGER_MODBUS_REG_T));   // 申请空间

            if(NULL!=g_pRegPoint[s_sPointRecord.uRelPoint])
            {
                memcpy(g_pRegPoint[s_sPointRecord.uRelPoint],pRegPointTemp,sizeof(LOGGER_MODBUS_REG_T)*s_sPointRecord.uLastPointCount);
            }
            else
            {
                DEBUGOUT("要点表空间失败2\n");
            }

            s_sPointRecord.uRelPoint        = 0x00;   // 点表相对点表号
            s_sPointRecord.uLastTableNum    = 0x00;   // 记录本次点表号
            s_sPointRecord.uLastPointCount  = 0x00;   // 已经记录的信息点数清零  Iec104Init
        }

        pRegPointTemp = WMemFree(pRegPointTemp); // 释放空间
        RecordInit(1);   // 历史数据存储重置

        // 擦除DataFlash空间，一共擦除两个扇区共8KB
        DataFlash_Sector_Erase(DATAFLASH_POINT_HEAD);
        DataFlash_Sector_Erase(DATAFLASH_POINT_HEAD+0x8000);
        // 存入新数据
        for(i=0,addr=DATAFLASH_POINT_HEAD; i<MAX_device; i++)//&&i<g_DeviceSouth.device_sum
        {
            if(0!=g_DeviceSouth.protocol[i].mess_point_sum)
            {
                space = sizeof(LOGGER_MODBUS_REG_T)*g_DeviceSouth.protocol[i].mess_point_sum;
                DataFlash_Write(addr,(uint8_t*)g_pRegPoint[i],space);  // 存入到DataFlash

                g_DeviceSouth.protocol[i].flash_addr = addr;
                addr += space;

                for(j=0,temp=0; j<g_DeviceSouth.protocol[i].mess_point_sum; j++)
                {
                    if(TYPE_GJ==g_pRegPoint[i][j].reg_type.type.mess) // 信息点类型为告警//    g_DeviceSouth.protocol[i].protocol_num)
                    {
                        temp++;
                    }
                }
                //-------------------------------------------------
                g_DeviceSouth.protocol[i].alarm_sum = temp;  // 记录告警点个数
                //-------------------------------------------------

            }
            else
            {
                continue;
            }
        }

        SaveEepData(EEP_DEVICE_SOUTH);//EepSavedata(EEP_LOGGER_DEVICE_INF_HEAD,(uint8_t *)&g_DeviceSouth,sizeof(g_DeviceSouth),&g_DeviceSouth.CRC);// 设备信息和点表信息存储

        //-----------------------------------------------------------------
        addr = EEP_ALARM_HEAD + sizeof(g_LoggerAlarm);  // 存储南向设备告警值的起始地址
        for(i=0; i<MAX_device; i++) // &&i<g_DeviceSouth.device_sum
        {
            if(0==g_DeviceSouth.device_inf[i].addr)
            {
                continue;
            }
            rel_num = g_DeviceSouth.device_inf[i].rel_num;   // 南向设备对应的相对点表号
            space   = g_DeviceSouth.protocol[rel_num].alarm_sum * sizeof(SOUTH_ALARM_T);

            if(space)     // 对应的点表有告警点
            {
                /*if(NULL!=g_psSouthAlarm[i])
                {
                    g_psSouthAlarm[i] = WMemFree(g_psSouthAlarm[i]);
                }*/
                g_psSouthAlarm[i] = (SOUTH_ALARM_T*)WMemMalloc(g_psSouthAlarm[i],space);   // 申请空间


                /*if(NULL!=g_psSouthAlarmCopy[i])
                {
                    g_psSouthAlarmCopy[i] = WMemFree(g_psSouthAlarmCopy[i]);
                }*/
                g_psSouthAlarmCopy[i] = (SOUTH_ALARM_T*)WMemMalloc(g_psSouthAlarmCopy[i],space);   // 申请空间


                if(NULL!=g_psSouthAlarm[i])
                {
                    // 搜索出点表里的告警点寄存器地址，并赋值给南向设备的告警结构体
                    for(j=0,temp=0; j<g_DeviceSouth.protocol[rel_num].mess_point_sum; j++)
                    {
                        if(TYPE_GJ == g_pRegPoint[rel_num][j].reg_type.type.mess) // 信息点类型为告警//    g_DeviceSouth.protocol[i].protocol_num)
                        {
                            g_psSouthAlarm[i][temp].mdbus_addr  = g_pRegPoint[rel_num][j].reg_addr;
                            g_psSouthAlarm[i][temp].alarm_value = 0x00;

                            g_psSouthAlarmCopy[i][temp].mdbus_addr  = g_pRegPoint[rel_num][j].reg_addr;
                            g_psSouthAlarmCopy[i][temp].alarm_value = 0x00;
                            temp++;
                        }
                    }
                }
                // 存储
                EepSavedata(addr,(uint8_t *)&g_psSouthAlarm[i][0],space,NULL);
                addr += space;
            }
        }
        //-----------------------------------------------------------------
		//g_DeviceEsn.uEsnMark[g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].addr]=1;      
        //DEBUGOUT("addr:%d",g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].addr);
		SaveEepData(EEP_DEVICE_ESN);  // 存储南向设备ESN
        SaveEepData(EEP_DEVICE_SOFT);  // 存储南向设备ESN
        //-----------------------------------------------------------------
        IecInit();  // iec104数据表重新初始化
        //-----------------------------------------------------------------
        g_LoggerRun.run_status = RUNNING_WORK_READ;  // 数采运行状态-导表完成开始正常工作
    }
    IecCpoyMesAddr(pA);

    memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
	IecCreateFrameI(P_TABLE,pA->recv.format.limit,R_TABLE_START,(pA->recv.format.len-13),&pA->send);
	
}
/******************************************************************************
* 名    称：IecReportSouthDevice()
* 功    能：上报已存在的南向设备。（响应平台查询）
* 入口参数：
            无
*
* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t IecReportSouthDevice(uint16_t uReason)
{
    uint8_t j,k=0;
    static uint8_t uRelAddr=0;

    g_sIEC.send.format.maddrL = 0x00;
    g_sIEC.send.format.maddrM = 0x00;
    g_sIEC.send.format.maddrH = 0x00;

    for(;uRelAddr<MAX_device;uRelAddr++)
    {
        if(g_DeviceSouth.device_inf[uRelAddr].addr)
        {
            j = k * 25;

            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].addr;
            j += 1;
            memcpy(&g_sIEC.send.format.data[j],&g_DeviceEsn.cDeviceEsn[uRelAddr],20);// I帧数据
            j += 20;
            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].protocol_num&0xff;
            j += 1;
            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].protocol_num>>8;
            j += 1;
            g_sIEC.send.format.data[j] = 1;
            j += 1;
            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].protocol_type;
            k++;

            if(k>=9)  // 一帧报文最多9个设备信息
            {
                uRelAddr++;
                break;
            }
        }
        else
        {
            continue;
        }
    }

    if(k)
    {
        IecCreateFrameI(P_SOUTH_INFO,1,uReason,25*k,&g_sIEC.send);
        k = 0;

        return 1;
    }
    else
    {
        if(uRelAddr>=MAX_device)
        {
            uRelAddr = 0;
            return 0;
        }
    }
    return 0;
}
/******************************************************************************
* 名    称：Iec104SouthDevInfo()
* 功    能：南向设备。C4
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104SouthDevInfo(IEC104_MAIN_T *pA)
{
    uint8_t uRec,i;

    if(NULL != pRegPointTemp)
	{
		pRegPointTemp = WMemFree(pRegPointTemp); 	  	//释放空间
	}
	Data_resend_count = 0;

    if(R_SETTING==pA->recv.format.reasonL)// 传输原因-设置0x92
    {
        if(RUNNING_INPUT_START==g_LoggerRun.run_status ||  // 已经收到C5 01
           RUNNING_INPUT_GOLB ==g_LoggerRun.run_status ||  // BB 88中
           RUNNING_INPUT_TABLE==g_LoggerRun.run_status ||  // BB 89中
           RUNNING_INPUT_104  ==g_LoggerRun.run_status)    // BB 8A中
        {
            return;
        }

        if(0x01==pA->recv.format.reasonH || RUNNING_INPUT_SOUTH!=g_LoggerRun.run_status) // 头一帧信息
        {
            s_sPointRecord.uRelDev = 0x00;
            AllReset(1); // 重置数采
        }

        g_LoggerRun.run_status = RUNNING_INPUT_SOUTH;  // 标记数采状态为导入信息

        for(i=0; i<(pA->recv.format.len-13) && s_sPointRecord.uRelDev<10;)
        {
            if(pA->recv.format.data[i] > MAX_device)  // 通讯地址大于最大数量
            {
                i += 25;
                continue;
            }
            if(0xff!=IecAddrConvert(pA->recv.format.data[i]))  // 下发的设备地址重复
            {
                i += 25;
                continue;
            }
            g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].addr = pA->recv.format.data[i];     // 南向设备通讯地址
            memcpy(g_DeviceEsn.cDeviceEsn[s_sPointRecord.uRelDev],&pA->recv.format.data[i+1],20); // 南向设备ESN号
            g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].protocol_num = (pA->recv.format.data[i+22]<<8 | pA->recv.format.data[i+21]); // 点表标识，点表号
            // [i+23]为连接端口，数采B只有一个通讯端口，此配置忽略
            g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].protocol_type = pA->recv.format.data[i+24]; // 南向设备协议类型,1:华为MODBUS；2：标志MODBUS
            s_sPointRecord.uRelDev++;
            i += 25;
        }

        IecCpoyMesAddr(pA);


        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
        IecCreateFrameI(P_SOUTH_INFO,pA->recv.format.limit,(pA->recv.format.reasonH<<8 | R_SET_SUC),(pA->recv.format.len-13),&pA->send);

        //iec_run.running = Iec104_status_process(iec_run.running,P_SOUTH_INFO,R_SETTING);
        SaveEepData(EEP_DEVICE_ESN);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL)// 传输原因-设置0x90
    {
        uRec = IecReportSouthDevice(R_INQUIRE_SUC);  //

        if(!uRec)  // 已经没未上报的南向设备
        {
            IecCpoyMesAddr(pA);


            pA->send.format.data[0] = 0x95;
            IecCreateFrameI(P_SOUTH_INFO,pA->recv.format.limit,R_INQUIRE_SUC,1,&pA->send);
        }
    }
    else if(R_SET_SUC==pA->recv.format.reasonL)// 传输原因-设置0x93
    {
        ReportCtrlClear(REPORT_OTHERS); // 已经无三方设备待发送
    }
}
/******************************************************************************
* 名    称：Iec104DevReportAck()
* 功    能：上报自发现设备确认。C7
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104DevReportAck(IEC104_MAIN_T *pA)
{
    if(R_SET_SUC==pA->recv.format.reasonL)// 传输原因-设置0x93
    {
        if(ReportCtrlRead(REPORT_HW_DEVICE))
        {
            ReportCtrlClear(REPORT_HW_DEVICE); // 取消标记发送了一帧上报华为设备信息报文
        }
        if(ReportCtrlRead(REPORT_HW_SOFT))
        {
            ReportCtrlClear(REPORT_HW_SOFT); // 取消标记发送了一帧上报华为设备信息报文
            Iec104HandleHwSoftRenovate(pA);
        }
    }
}

/******************************************************************************
* 名    称：Iec104LoggerUpdate()
* 功    能：升级接收数据。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
static void Iec104LoggerUpdate(IEC104_MAIN_T *pA)
{
    uint16_t check_crc;
    uint16_t uTmp;
    uint16_t uTmpCrc;
    Uint_Char_Convert temp;
    static uint16_t s_uRecseq=0,s_uRecAll=0; // 接收序号
    static uint8_t  s_uUpdateMark=0;         // 开始升级标记
    //static uint16_t uUpdataDataAllLen=0;   //表计升级数据总长度
    //OS_CPU_SR  cpu_sr;

    uint8_t uTmpData[200];

    if(R_TRANS_START==pA->recv.format.reasonL) // 开始传输数据0x80
    {
        if(0==g_LoggerRun.update)
        {
            if(RUNNING_INPUT_START ==g_LoggerRun.run_status || // 启动导表C5
                RUNNING_INPUT_SOUTH==g_LoggerRun.run_status || // 导入下联设备信息C4 92
                RUNNING_INPUT_GOLB ==g_LoggerRun.run_status || // 导入全局信息BB 88
                RUNNING_INPUT_TABLE==g_LoggerRun.run_status || // 导入点表BB 89
                RUNNING_INPUT_104  ==g_LoggerRun.run_status    // 导入设备104表信息BB 8A
              )
            {
                return;
            }
        }
        else
        {
            return;
        }

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = 0x81;

        //FEED_DOG();  // 喂看门狗

        // DataFlash存储升级数据区域擦除，共96KB，一个区块+8个扇区
        //FEED_DOG();     // 喂狗
        DataFlash_Block_Erase(0);// 擦除块0
        DataFlash_Sector_Erase(0x010000);
        DataFlash_Sector_Erase(0x011000);
        DataFlash_Sector_Erase(0x012000);
        DataFlash_Sector_Erase(0x013000);
        DataFlash_Sector_Erase(0x014000);
        DataFlash_Sector_Erase(0x015000);
        DataFlash_Sector_Erase(0x016000);
        DataFlash_Sector_Erase(0x017000);

        s_uRecseq = 0;
        s_uRecAll = 0;

        //uSide = pA->recv.format.maddrL&0x0f;
        if(pA->recv.format.maddrL&0x01)
        {
            g_LoggerRun.update = 0xA0;   // 升级到A面
        }
        else
        {
            g_LoggerRun.update = 0xB0;   // 升级到B面
        }

        g_uNowTime = OSTimeGet();
        g_uUpdataTime = g_uNowTime;  // 更新超时保护时标

        DEBUGOUT("\n启动升级\n");

        IecCreateFrameI(P_UPDATA,0x01,R_TRANS_START_ACK,1,&pA->send);

        s_uUpdateMark = 1;
    }
    else if(R_DATA_TRANS==pA->recv.format.reasonL) // 数据传输中0x82
    {
        if(0==s_uUpdateMark)
        {
            DEBUGOUT("\n弃升级包\n");
            return;
        }

        temp.c[1] = pA->recv.buff[pA->recv.format.len+1];
        temp.c[0] = pA->recv.buff[pA->recv.format.len];

        check_crc = CalculateCRC(pA->recv.format.data,pA->recv.format.len-15);//CRC16(pA->recv.format.data,pA->recv.format.len-15);

        if((check_crc==temp.u))  // CRC校验通过
        {
            temp.c[0] = pA->recv.buff[12];
            temp.c[1] = pA->recv.buff[13];

            if((temp.u == (s_uRecseq+1)) || (0==temp.u && 0==s_uRecseq))
            {
                s_uRecseq = temp.u;
                s_uRecAll++;
                // 接收的包序号
                IecCpoyMesAddr(pA);

                uTmp = pA->recv.format.len - 15;

                if(uTmp>200)
                {
                    DEBUGOUT("升级失败\n");
                    s_uUpdateMark = 0;
                    msleep(100);
                    Reboot();
                    return;
                }


                //OS_ENTER_CRITICAL();
                // 存储到DataFlash升级数据区域，数据长度：减去104报文和CRC校验
                DataFlash_Write(s_uRecseq * 200,(uint8_t *)pA->recv.format.data,uTmp);
                //OS_EXIT_CRITICAL();

                msleep(10);

                memset(uTmpData,0,200);

                //OS_ENTER_CRITICAL();
                temp.u = DataFlash_Read(s_uRecseq * 200,uTmpData,uTmp);
                uTmpCrc = CalculateCRC(uTmpData,uTmp);
                //OS_EXIT_CRITICAL();

                if(check_crc!=uTmpCrc)   //读出来校验CRC
                {
                    DEBUGOUT("升级重读DF\n");
                    temp.u = DataFlash_Read(s_uRecseq * 200,uTmpData,uTmp);
                    uTmpCrc = CalculateCRC(uTmpData,uTmp);

                    if(check_crc!=uTmpCrc)   //读出来校验CRC
                    {
                        DEBUGOUT("升级%d存失败%d-%d,算%04X,收%04X\n",s_uRecseq,temp.u,uTmp,uTmpCrc,check_crc);

                        for(temp.u=0;temp.u<uTmp;temp.u++)
                        {
                            DEBUGOUT("%02X ",uTmpData[temp.u]);
                        }

                        s_uUpdateMark = 0;

                        msleep(100);
                        Reboot();
                        return;
                    }
                }

                pA->send.format.data[0] = 0x01;//iec.ibuff[2] = 0x01;   // 传输成功

                IecCreateFrameI(P_UPDATA,0x01,R_DATA_TRANS,1,&pA->send);
                DEBUGOUT("\n升级%d\n",s_uRecseq);
				//uUpdataDataAllLen += uTmp;
            }
            /*else if(temp.u <= s_uRecseq) // 重复发送
            {
                // 接收的包序号
                pA->send.format.maddrL = pA->recv.format.maddrL;
                pA->send.format.maddrM = pA->recv.format.maddrM;
                pA->send.format.maddrH = pA->recv.format.maddrH;
                pA->send.format.data[0] = 0x01;//iec.ibuff[2] = 0x01;   // 传输成功
                IecCreateFrameI(P_UPDATA,0x01,R_DATA_TRANS,1,pA);
            }*/
            else// if(temp.u > (s_uRecseq+1))
            {
                // 接收的包序号
                IecCpoyMesAddr(pA);

                DEBUGOUT("\n升级收%d需%d\n",temp.u,s_uRecseq + 1);

                temp.u = s_uRecseq + 1;
                pA->send.format.data[0] = temp.c[0];// pA->rec_buff[12];// 需要重传的包序号
                pA->send.format.data[1] = temp.c[1];// pA->rec_buff[13];
                pA->send.format.data[2] = 0x00;
                IecCreateFrameI(P_UPDATA,0x01,R_DATA_RETRY,3,&pA->send); // 要数据重传
            }

        }
        else // CRC校验不通过
        {
            // 接收的包序号
            IecCpoyMesAddr(pA);

            // 需要重传的包序号
            pA->send.format.data[0] = pA->recv.format.maddrL;
            pA->send.format.data[1] = pA->recv.format.maddrM;
            pA->send.format.data[2] = pA->recv.format.maddrH;

            IecCreateFrameI(P_UPDATA,0x01,R_DATA_RETRY,3,&pA->send); // 要数据重传
            DEBUGOUT("\n升级CRC错误,算%04X,收%04X\n",check_crc,temp.u);
        }

        g_uUpdataTime = g_uNowTime;  // 更新超时保护时标
    }
    else if(R_TRANS_FINISH==pA->recv.format.reasonL) // 数据传输完成0x86
    {
        //iec_run.running = Iec104_status_process(iec_run.running,P_UPDATA,R_TRANS_FINISH);

        s_uRecseq = pA->recv.format.maddrM<<8 | pA->recv.format.maddrL;  // 升级包总数

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = R_TRANS_FINISH_ACK;
        IecCreateFrameI(P_UPDATA,0x01,R_TRANS_FINISH_ACK,1,&pA->send);
        DEBUGOUT("\n收总数:%d包总数:%d\n",s_uRecAll,s_uRecseq);
		//DEBUGOUT("表计升级包总长度:%d\n",uUpdataDataAllLen);
        if(s_uRecAll!=s_uRecseq)
        {
            return;
        }

        // 存储升级信息到EEPROM，然后重启芯片

        //EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)data,11,NULL);// 升级信息存储

        check_crc = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// 升级信息读取

        if(0==check_crc) // 读取数据失败,有回滚数据标志标记为没有回滚数据
        {
            g_LoggerUpdate.rollback_allow = 0x55;
        }
        g_LoggerUpdate.frame_sum = s_uRecseq * 200 / 256 + 1;// 256个字节一帧，总帧数

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

        if(0xA0==g_LoggerRun.update) // 升级到A面
        {
            g_LoggerUpdate.side_tobe = 0xAA;  // 目标升级到A面
        }
        else
        {
            g_LoggerUpdate.side_tobe = 0xBB;  // 目标升级到B面
        }



        if(!s_uRecseq)  // 没有升级包数据
        {
            g_LoggerUpdate.updata_mark = 0x55;  //  data[2] = 0x55;    // 0xAA升级；0xBB回滚；0x55无
        }
        else
        {
            g_LoggerUpdate.updata_mark = 0xAA;  // data[2] = 0xAA;    // 0xAA升级；0xBB回滚；0x55无
        }

        SaveEepData(EEP_UPDATA);//EepSavedata(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// 升级信息存储

        s_uRecseq = 0x00;

        sleep(3);

        Reboot();// 重启芯片
        //uSide = 0x55;// 标记重启芯片 reboot();  // 重启芯片
    }
    else if(R_TRANS_STOP==pA->recv.format.reasonL) // 停止升级0x84
    {
        pA->send.format.maddrL = 0;
        pA->send.format.maddrM = 0;
        pA->send.format.maddrH = 0;
        pA->send.format.data[0] = R_TRANS_STOP_ACK;
        IecCreateFrameI(P_UPDATA,0x01,R_TRANS_STOP_ACK,1,&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104DT1000Update()
* 功    能：升级接收数据。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
static void Iec104DT1000Update(IEC104_MAIN_T *pA)
{
    static uint32_t uUpdateDataAllLen=0;   //表计升级数据总长度
	uint16_t check_crc;
    uint16_t uTmp;
    uint16_t uTmpCrc;
    Uint_Char_Convert temp;
	U32_F_Char_Convert ctemp;
    static uint16_t s_uRecseq=0,s_uRecAll=0; // 接收序号
    static uint8_t  s_uUpdateMark=0;         // 开始升级标记

    static uint8_t uFrameLen = 0;
    uint8_t uTmpData[200];
	static uint8_t uFileStateIO = 0;

    if(R_TRANS_START==pA->recv.format.reasonL) // 开始传输数据0x80
    {

		if(0==g_LoggerRun.update)
		{
			if(RUNNING_INPUT_START ==g_LoggerRun.run_status || // 启动导表C5
				RUNNING_INPUT_SOUTH==g_LoggerRun.run_status || // 导入下联设备信息C4 92
				RUNNING_INPUT_GOLB ==g_LoggerRun.run_status || // 导入全局信息BB 88
				RUNNING_INPUT_TABLE==g_LoggerRun.run_status || // 导入点表BB 89
				RUNNING_INPUT_104  ==g_LoggerRun.run_status ||  // 导入设备104表信息BB 8A
				(g_LoggerRun.uFileIO_status != 0)
			  )
			{
				return;
			}
		}
		else
		{
			return;
		}

        IecCpoyMesAddr(pA);

		if(pA->recv.format.maddrL == S_FILE_EXPORT)  //文件导出状态
		{
			 g_DT1000Updata.uDevAddr = pA->recv.format.data[0];//获取设备地址
			 g_DT1000Updata.uData = pA->recv.format.data[1];			 
			 g_LoggerRun.uFileIO_status = S_FILE_EXPORT;       //用于南向开启日志导出状态
			 
			 uFileStateIO = S_FILE_EXPORT;  
			 return;
		}else if(pA->recv.format.maddrL == S_FILE_IMPORT)  //文件导入状态
		{
             uFileStateIO = S_FILE_IMPORT;       
			 DEBUGOUT("\n升级文件导入\n");
		}

		if(S_FILE_IMPORT == uFileStateIO)
		{
			//获取文件长度
			ctemp.c[0] = pA->recv.format.data[19];
			ctemp.c[1] = pA->recv.format.data[20];
			ctemp.c[2] = pA->recv.format.data[21];
			ctemp.c[3] = pA->recv.format.data[22];
			g_DT1000Updata.nDataLen = ctemp.u;
			DEBUGOUT("\nUpdate FileLen:%d	 ",g_DT1000Updata.nDataLen);
	
			uFrameLen = pA->recv.format.data[23];
			memcpy(pA->send.format.data,pA->recv.format.data,24);
	
			if(pA->recv.format.data[18]&0x01)	//暂时用屏蔽南向信息
			{
				g_LoggerRun.update = 0xA0;	 // 升级到A面
			}
			else
			{
				g_LoggerRun.update = 0xB0;	 // 升级到B面
			}
	
			// DataFlash存储升级数据区域擦除，共96KB，一个区块+8个扇区	   
			DataFlash_Block_Erase(5);// 擦除块3
			msleep(1);
			DataFlash_Block_Erase(6);// 擦除块4
			msleep(1);
	
			s_uRecseq = 0;
			s_uRecAll = 0;
	
			memcpy(g_DT1000Updata.version,&pA->recv.format.data[2],17);
		   
			DEBUGOUT("DT1000_Vertion: ");
			for(uint8_t i=0;i<17;i++)
			{
				DEBUGOUT("%c",g_DT1000Updata.version[i]);
			}
			DEBUGOUT("\r\n");
	
			g_uNowTime = OSTimeGet();
			g_uUpdataTime = g_uNowTime;  // 更新超时保护时标
	
			DEBUGOUT("\n启动表计升级文件传输\n");
			IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_START_ACK,24,&pA->send);
			uUpdateDataAllLen=0;
			s_uUpdateMark = 1;
		}
    }
    else if(R_DATA_TRANS==pA->recv.format.reasonL) // 数据传输中0x82
    {
		if(S_FILE_IMPORT == uFileStateIO)
		{
			if(0==s_uUpdateMark)
			{
				DEBUGOUT("\n弃升级包\n");
				return;
			}
	
			//帧文件CRC校验
			temp.c[1] = pA->recv.buff[pA->recv.format.len+1];
			temp.c[0] = pA->recv.buff[pA->recv.format.len];
	
			check_crc = CalculateCRC(pA->recv.format.data,pA->recv.format.len-15);//CRC16(pA->recv.format.data,pA->recv.format.len-15);
	
			if((check_crc==temp.u))  // CRC校验通过
			{
				//包序号判断
				temp.c[0] = pA->recv.buff[12];
				temp.c[1] = pA->recv.buff[13];
	
				if((temp.u == (s_uRecseq+1)) || (0==temp.u && 0==s_uRecseq))
				{
					s_uRecseq = temp.u;
					s_uRecAll++;
					
					//IecCpoyMesAddr(pA);// 接收的包序号
					pA->send.format.maddrL = 0x00;
					pA->send.format.maddrM = 0x00;
					pA->send.format.maddrH = 0x00;
	
					uTmp = pA->recv.format.len - 15;
	
					if(uTmp>200)
					{
						DEBUGOUT("传输失败\n");
						s_uUpdateMark = 0;
						uFileStateIO = 0;
						msleep(100);
						Reboot();
						return;
					}
	
					//OS_ENTER_CRITICAL();
					// 存储到DataFlash升级数据区域，数据长度：减去104报文和CRC校验
					DataFlash_Write(DATAFLASH_DT1000_UPDATA_HEAD+s_uRecseq * uFrameLen,(uint8_t *)pA->recv.format.data,uTmp);
					//OS_EXIT_CRITICAL();
	
					msleep(10);
					memset(uTmpData,0,200);
	
					//OS_ENTER_CRITICAL();
					temp.u = DataFlash_Read(DATAFLASH_DT1000_UPDATA_HEAD+s_uRecseq * uFrameLen,uTmpData,uTmp);
					uTmpCrc = CalculateCRC(uTmpData,uTmp);
					//OS_EXIT_CRITICAL();
	
					if(check_crc!=uTmpCrc)	 //读出来校验CRC
					{
						DEBUGOUT("传输重读DF\n");
						temp.u = DataFlash_Read(DATAFLASH_DT1000_UPDATA_HEAD+s_uRecseq * uFrameLen,uTmpData,uTmp);
						uTmpCrc = CalculateCRC(uTmpData,uTmp);
	
						if(check_crc!=uTmpCrc)	 //读出来校验CRC
						{
							DEBUGOUT("传输%d存失败%d-%d,算%04X,收%04X\n",s_uRecseq,temp.u,uTmp,uTmpCrc,check_crc);
	
							for(temp.u=0;temp.u<uTmp;temp.u++)
							{
								DEBUGOUT("%02X ",uTmpData[temp.u]);
							}
	
							s_uUpdateMark = 0;
							uFileStateIO = 0;
	
							msleep(100);
							Reboot();
							return;
						}
					}
	
					//pA->send.format.data[0] = 0x01;//iec.ibuff[2] = 0x01;   // 传输成功
					pA->send.format.data[0] = pA->recv.format.maddrL;
					pA->send.format.data[1] = pA->recv.format.maddrM;
					pA->send.format.data[2] = pA->recv.format.maddrH;
	
					IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_TRANS,3,&pA->send);
					DEBUGOUT("\n传输帧%d\n",s_uRecseq);
					uUpdateDataAllLen += uTmp;
					DEBUGOUT("\nUpdateAllLen:%d\n",uUpdateDataAllLen);
				}
				else
				{
					IecCpoyMesAddr(pA);// 接收的包序号
	
					DEBUGOUT("\n传输帧收%d需%d\n",temp.u,s_uRecseq + 1);
	
					temp.u = s_uRecseq + 1;
					pA->send.format.data[0] = temp.c[0];// pA->rec_buff[12];// 需要重传的包序号
					pA->send.format.data[1] = temp.c[1];// pA->rec_buff[13];
					pA->send.format.data[2] = 0x00;
					IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_RETRY,3,&pA->send); // 要数据重传
				}
			}
			else // CRC校验不通过
			{
				IecCpoyMesAddr(pA);// 接收的包序号
	
				// 需要重传的包序号
				pA->send.format.data[0] = pA->recv.format.maddrL;
				pA->send.format.data[1] = pA->recv.format.maddrM;
				pA->send.format.data[2] = pA->recv.format.maddrH;
	
				IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_RETRY,3,&pA->send); // 要数据重传
				DEBUGOUT("\n传输CRC错误,算%04X,收%04X\n",check_crc,temp.u);
			}
	
			g_uUpdataTime = g_uNowTime;  // 更新超时保护时标
		}
    }
    else if(R_TRANS_FINISH==pA->recv.format.reasonL) // 数据传输完成0x86
    {
		if(S_FILE_IMPORT == uFileStateIO)
		{
			//iec_run.running = Iec104_status_process(iec_run.running,P_UPDATA,R_TRANS_FINISH);
	
			s_uRecseq = pA->recv.format.maddrM<<8 | pA->recv.format.maddrL;  // 升级包总数
	
			IecCpoyMesAddr(pA);
	
			pA->send.format.data[0] = pA->recv.format.data[0];	//两个字节的文件CRC校验
			pA->send.format.data[1] = pA->recv.format.data[1];
			IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_FINISH_ACK,1,&pA->send);
			DEBUGOUT("\n收总数:%d包总数:%d\n",s_uRecAll,s_uRecseq);
			g_DT1000Updata.frame_sum=s_uRecAll;
			
			DEBUGOUT("DT1000 File Len:%d Frame Sum:%d\n",g_DT1000Updata.nDataLen,g_DT1000Updata.frame_sum);
			
			if(s_uRecAll!=s_uRecseq)
			{
				g_LoggerRun.update = 0x00;	  //暂时用屏蔽南向信息
	
				return;
			}
			
			/*if(g_DT1000Updata.version[16]&0x01)  // 小版本号最低位，1：程序在A面；0：程序在B面
			{
				g_LoggerRun.uFileIO_status = 0x01;	 // 升级到B面
			}
			else //if((g_DT1000Updata.version[16])%2 == 0)
			{
				g_LoggerRun.uFileIO_status = 0x01;	 // 升级到B面
			}*/
			
		   g_LoggerRun.uFileIO_status = 0x01;	   // 南向升级导入开始
		   g_LoggerRun.update = 0x00;	 //取消屏蔽南向信息
		   
	       uFileStateIO = 0;
		   s_uRecseq = 0x00;
		   uUpdateDataAllLen = 0;
		}     
    }
    else if(R_TRANS_STOP==pA->recv.format.reasonL) // 停止升级0x84
    {
		if(S_FILE_IMPORT == uFileStateIO)
		{
			g_LoggerRun.update = 0x00;
			s_uRecseq = 0x00;
			s_uRecAll = 0x00;
			uUpdateDataAllLen = 0;
			
			pA->send.format.maddrL = 0;
			pA->send.format.maddrM = 0;
			pA->send.format.maddrH = 0;
			pA->send.format.data[0] = R_TRANS_STOP_ACK;
			IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_STOP_ACK,1,&pA->send);
			uFileStateIO = 0;
		}
    }
}
/******************************************************************************
* 名    称：Iec104IP()
* 功    能：IP配置查询。B5
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104IP(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据  memcpy(&iec.ibuff[0],&pA->rec_buff[12],(pA->rec_buff[1]-10));// I帧数据
        IecCreateFrameI(P_SET_IP,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = 0;//Logger_run.IP[0];
        pA->send.format.data[1] = 0;//Logger_run.IP[1];
        pA->send.format.data[2] = 0;//Logger_run.IP[2];
        pA->send.format.data[3] = 0;//Logger_run.IP[3];

        IecCreateFrameI(P_SET_IP,pA->recv.format.limit,R_INQUIRE_SUC,4,&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104HandleModBusEndian()
* 功    能：104设置查询MODBUS大小端。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t Iec104HandleModBusEndian(IEC104_MAIN_T *pA)
{
    uint8_t uMbEndian;//大小端
    //uint8_t uMbVirAddr;//设备地址:限定最多接0-9十个逆变器
    uint8_t uMbRelAddr=0;//根据MbVirAddr映射出的设备实际地址
    memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));
    //uMbVirAddr = pA->recv.format.data[0];
    uMbEndian  = pA->recv.format.data[1];
    uMbRelAddr = IecAddrConvert(pA->recv.format.data[0]);

    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        if(uMbRelAddr<MAX_device)
        {
            g_DeviceSouth.device_inf[uMbRelAddr].big_little_endian = uMbEndian;
            SaveEepData(EEP_DEVICE_SOUTH);
        }

        IecCreateFrameI(P_MODBUS_ENDIAN,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        if(uMbRelAddr<MAX_device)
        {
            pA->send.format.data[1] = g_DeviceSouth.device_inf[uMbRelAddr].big_little_endian;
        }
        else
        {
            pA->send.format.data[1] = 1;
        }

        IecCreateFrameI(P_MODBUS_ENDIAN,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
    }

    return 0;
}
/******************************************************************************
* 名    称：Iec104HandleModBusBd()
* 功    能：IEC104 设备波特率处理。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleModBusBd(IEC104_MAIN_T *pA)
{
	uint32_t uMbBd;//波特率
	uint8_t uMbVirAddr;//设备地址:限定最多接0-9是个逆变器(报文中的下标)
	uint8_t uMbRelAddr=0;//根据MbVirAddr映射出的设备实际地址(能找到所需操作设备的下标)
	uint8_t uSendReason;
	uint8_t uMbBR=0;

	memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));
	uMbVirAddr = pA->recv.format.data[0];
	uMbBd      = (pA->recv.format.data[3]<<16)+(pA->recv.format.data[2]<<8)+(pA->recv.format.data[1]);
	uMbRelAddr = IecAddrConvert(uMbVirAddr);


	if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        if(uMbRelAddr<MAX_device)
        {
            switch(uMbBd)
            {
            case 2400:
                uMbBR = BAUDRATE_2400;
                break;

            case 4800:
                uMbBR = BAUDRATE_4800;
                break;

            case 9600:
                uMbBR = BAUDRATE_9600;
                break;

            case 19200:
                uMbBR = BAUDRATE_19200;
                break;

            case 38400:
                uMbBR = BAUDRATE_38400;
                break;

            case 115200:
                uMbBR = BAUDRATE_115200;
                break;

            default:
                uMbBR = BAUDRATE_9600;
                break;
            }
            //
            g_DeviceSouth.device_inf[uMbRelAddr].baud_rate = uMbBR;
            SaveEepData(EEP_DEVICE_SOUTH);
        }
    	uSendReason=pA->recv.format.reasonL+1;

        IecCreateFrameI(P_MODBUS_BAUD,pA->recv.format.limit,uSendReason,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        if(uMbRelAddr<MAX_device)
        {
            uMbBR = g_DeviceSouth.device_inf[uMbRelAddr].baud_rate;
        }
        switch(uMbBR)
        {
        case BAUDRATE_2400:
            uMbBd = 2400;
            break;

        case BAUDRATE_4800:
            uMbBd = 4800;
            break;

        case BAUDRATE_9600:
            uMbBd = 9600;
            break;

        case BAUDRATE_19200:
            uMbBd = 19200;
            break;

        case BAUDRATE_38400:
            uMbBd = 38400;
            break;

        case BAUDRATE_115200:
            uMbBd = 115200;
            break;

        default:
            uMbBd = 9600;
            break;
        }
        pA->send.format.data[3] = uMbBd>>16;
        pA->send.format.data[2] = uMbBd>>8;
        pA->send.format.data[1] = uMbBd;

        IecCreateFrameI(P_MODBUS_BAUD,pA->recv.format.limit,R_INQUIRE_SUC,(pA->recv.format.len-13),&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104PublicAddr()
* 功    能：公共地址。B8
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104PublicAddr(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        g_LoggerInfo.ADDR = pA->recv.format.data[0];//pA->recv.format.addrL;// >rec_buff[15];  // 新的公共地址，需要保存eeprom

        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
        IecCreateFrameI(P_ADDR,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);

        DEBUGOUT("存地址\n");
        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = g_LoggerInfo.ADDR;
        pA->send.format.data[1] = 0x00;
        IecCreateFrameI(P_ADDR,pA->recv.format.limit,R_INQUIRE_SUC,2,&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104InquireVer()
* 功    能：版本查询。B9
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104InquireVer(IEC104_MAIN_T *pA)
{
    uint8_t uRes;
    IecCpoyMesAddr(pA);

    pA->send.format.data[0] = GetVerS2();
    pA->send.format.data[1] = GetVerS1();
    pA->send.format.data[2] = GetVerType();

    uRes = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&eepdata,sizeof(UPDATA_MARK_T),&eepdata.CRC);// 升级信息存储
    if(uRes)
    {
        if(GetVerS2()&0x01)// 小版本号最低位，1：程序在A面；0：程序在B面
        {
            pA->send.format.data[3] = g_LoggerUpdate.a_version[0];
            pA->send.format.data[4] = g_LoggerUpdate.a_version[1];
            pA->send.format.data[5] = g_LoggerUpdate.a_version[2];
        }
        else
        {
            pA->send.format.data[3] = g_LoggerUpdate.b_version[0];
            pA->send.format.data[4] = g_LoggerUpdate.b_version[1];
            pA->send.format.data[5] = g_LoggerUpdate.b_version[2];
        }
    }

    IecCreateFrameI(P_VERSION,0x01,R_INQUIRE_SUC,6,&pA->send);
}
/******************************************************************************
* 名    称：Iec104LoggerRollBack()
* 功    能：升级接收数据。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
static void Iec104LoggerRollBack(IEC104_MAIN_T *pA)
{
    uint8_t check_crc;

    IecCpoyMesAddr(pA);

    pA->send.format.data[0] = pA->recv.format.data[0];
    pA->send.format.data[1] = pA->recv.format.data[1];
    pA->send.format.data[2] = pA->recv.format.data[2];
    IecCreateFrameI(P_ROLLBAOCK,0x01,R_SET_SUC,3,&pA->send);


    //iec_run.running = Iec104_status_process(iec_run.running,P_ROLLBAOCK,R_TRANS_FINISH);

    // 存储升级信息到EEPROM，然后重启芯片
    //EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)data,11,NULL);// 升级信息存储
    check_crc = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// 升级信息读取

    //check_crc = CRC16(data,9);
    //if(check_crc==((data[9]<<8)|data[10]) && 0xBB==data[3])
    if(0!=check_crc) // 读取数据成功,且 回滚数据标志标记为有回滚数据
    {
        if(0xBB==g_LoggerUpdate.rollback_allow) // 有历史程序，可以回滚
        {
            g_LoggerUpdate.frame_sum      = 0x00;
            g_LoggerUpdate.updata_mark    = 0xBB;       // 0xAA升级；0xBB回滚；0x55无
            g_LoggerUpdate.rollback_allow = 0xBB;


            if(GetVerS2()&0x01)// 小版本号最低位，1：程序在A面；0：程序在B面
            {
                pA->send.format.data[3] = g_LoggerUpdate.a_version[0];
                pA->send.format.data[4] = g_LoggerUpdate.a_version[1];
                pA->send.format.data[5] = g_LoggerUpdate.a_version[2];
            }
            else
            {
                pA->send.format.data[3] = g_LoggerUpdate.b_version[0];
                pA->send.format.data[4] = g_LoggerUpdate.b_version[1];
                pA->send.format.data[5] = g_LoggerUpdate.b_version[2];
            }

            g_LoggerUpdate.reserve = 0x00;

            SaveEepData(EEP_UPDATA);//EepSavedata(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// 升级信息存储

            sleep(5);
            Reboot();  // 重启芯片
        }
    }
}
/******************************************************************************
* 名    称：Iec104PhoneNum()
* 功    能：上报号码。BC
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104PhoneNum(IEC104_MAIN_T *pA)
{
    //uint16_t uValue=0;
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        IecCpoyMesAddr(pA);

		memcpy(g_LoggerInfo.phonenum,pA->recv.format.data,11);
        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
        IecCreateFrameI(P_REPORT_NUM,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);

        DEBUGOUT("存号码\n");
        SaveEepData(EEP_LOGGER_INF);

        // 解析预置点表号：3~6位号码
        //uValue = (g_LoggerInfo.phonenum[2]-'0')*10000 + (g_LoggerInfo.phonenum[3]-'0')*1000 + (g_LoggerInfo.phonenum[4]-'0')*100 + (g_LoggerInfo.phonenum[5]-'0')*10 +(g_LoggerInfo.phonenum[6]-'0');
       /* if(uValue<=60000)
        {
            DEBUGOUT("点表取消预置:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }
        else
        {
            DEBUGOUT("点表预置:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }*/
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,g_LoggerInfo.phonenum,11);

        IecCreateFrameI(P_REPORT_NUM,pA->recv.format.limit,R_INQUIRE_SUC,11,&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104CommMode()
* 功    能：通信方式。C6
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104CommMode(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = pA->recv.format.data[0];

        IecCreateFrameI(P_COM_MODE,pA->recv.format.limit,R_SET_SUC,1,&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询0x90
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = 0x01;  //无线方式通讯
        IecCreateFrameI(P_COM_MODE,pA->recv.format.limit,R_INQUIRE_SUC,1,&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104ServiceIP()
* 功    能：服务器域名/IP。C8
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104ServiceIP(IEC104_MAIN_T *pA)
{
    uint8_t uLen;
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        // 报文解析……
        uLen = strlen((char *)pA->recv.format.data);
        memset(g_LoggerInfo.server_domain,'\0',34);
        if(uLen>=30)
        {
            uLen = 30;
        }
        strncpy(&g_LoggerInfo.server_domain[1],(char *)pA->recv.format.data,uLen);
        g_LoggerInfo.server_domain[0] = '\"';
        if(uLen<30)
        {
            g_LoggerInfo.server_domain[uLen+1] = '\"';
        }
        else
        {
            memset(&g_LoggerInfo.server_domain[31],'\0',3);
            g_LoggerInfo.server_domain[31]='\"';
        }
        g_LoggerInfo.server_port = pA->recv.format.data[31]<<8 | pA->recv.format.data[30];
        DEBUGOUT("存域名\n");
        SaveEepData(EEP_LOGGER_INF);  // 保存信息

        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I帧数据
        IecCreateFrameI(P_SERVICE_IP,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询0x90
    {
        memset(pA->send.format.data,0x00,30);
        uLen = strlen(g_LoggerInfo.server_domain);
        strncpy((char *)pA->send.format.data,&g_LoggerInfo.server_domain[1],uLen-2); // 存储的域名为 "域名"需要去掉双引号
        pA->send.format.data[30] = g_LoggerInfo.server_port&0xff;
        pA->send.format.data[31] = g_LoggerInfo.server_port>>8;

        pA->send.format.maddrL = 0x00;
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        IecCreateFrameI(P_SERVICE_IP,pA->recv.format.limit,R_INQUIRE_SUC,32,&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104LogExport()
* 功    能：平台日志导出。
* 入口参数：
*           *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
* ADD BY: chenyang
******************************************************************************/
uint32_t Iec104LogExport(IEC104_MAIN_T *pA)
{
    
    static uint8_t uMenLogData[LOGFRAMELENTH] = {0};
    static uint32_t uLogDataFlash= 0;
    static LOG_INF_ADDE Iaddr = {0};
    static LOG_INF_ADDE counts = {0};

    if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        return 0 ;
    }

    switch(pA->recv.format.reasonL)
    {
    case R_TRANS_START: //开始传输0x80

        counts.uInfAddr = SerchLog(pA->recv.format.data[0],&uLogDataFlash);

        pA->send.format.maddrL = 0x01;  // 信息体地址
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = counts.uInfTemp[0];  // 计算出总共需要发送的日志包数量
        pA->send.format.data[1] = counts.uInfTemp[1];
        pA->send.format.data[2] = 0x00;
        IecCreateFrameI(P_LOG,0x01,R_TRANS_START_ACK,3,&pA->send);
        break;

    case R_DATA_TRANS:  //数据传输0x82
        if(Iaddr.uInfAddr>=counts.uInfAddr)//当发送的数据帧等于总包数，数采发送0x86 停止传输
        {

            pA->send.format.maddrL = Iaddr.uInfTemp[0];  // 信息体地址，总包数
            pA->send.format.maddrM = Iaddr.uInfTemp[1];
            pA->send.format.maddrH = 0x00;
            pA->send.format.data[0] = 0x86;
            IecCreateFrameI(P_LOG,0x01,R_TRANS_FINISH,1,&pA->send); //传输完成

            memset(uMenLogData,0,LOGFRAMELENTH);
            uLogDataFlash= 0;
            Iaddr.uInfAddr = 0;
            counts.uInfAddr = 0;
            break;
        }
        pA->send.format.maddrL = Iaddr.uInfTemp[0]; // 信息体地址低位
        pA->send.format.maddrM = Iaddr.uInfTemp[1];
        pA->send.format.maddrH = 0x00;

        DataFlash_Read(uLogDataFlash+LOGFRAMELENTH*Iaddr.uInfAddr,&uMenLogData[0],LOGFRAMELENTH);
        for(int i=0; i<LOGFRAMELENTH; i++)
        {
            pA->send.format.data[i] = uMenLogData[i];  // 日志数据
        }
        pA->send.format.data[200] = 0x55;   //CRC校验低位
        pA->send.format.data[201] = 0xAA;   //CRC校验高位
        IecCreateFrameI(P_LOG,0x01,R_DATA_TRANS,LOGFRAMELENTH+2,&pA->send);

        Iaddr.uInfAddr++;

        break;

//        平台并不校验CRC
//        }
//        else if (0x02==pA->recv.format.data[0])
//        {
//            for(int i=0; i<LOGFRAMELENTH; i++)
//            {
//                pA->send.format.data[i] = uMenLogData[i];  // 日志数据
//            }
//            pA->send.format.data[200] = 0xAA;   //CRC校验低位
//            pA->send.format.data[201] = 0x55;   //CRC校验高位
//            IecCreateFrameI(P_LOG,0x01,R_DATA_TRANS,LOGFRAMELENTH+2,pA);
//        CRC校验失败时，需要直接重新发送

    case R_DATA_RETRY:  //数据重传0x83

        if(Iaddr.uInfAddr>=counts.uInfAddr)
        {
            return 0;
        }
        Iaddr.uInfTemp[0] = pA->recv.format.data[0];     //重传包序
        Iaddr.uInfTemp[1] = pA->recv.format.data[1];
        DataFlash_Read(uLogDataFlash+LOGFRAMELENTH*Iaddr.uInfAddr,&uMenLogData[0],LOGFRAMELENTH);

        pA->send.format.maddrL = Iaddr.uInfTemp[0];     // 信息体地址低位
        pA->send.format.maddrM = Iaddr.uInfTemp[1];
        pA->send.format.maddrH = 0x00;
        for(int i=0; i<LOGFRAMELENTH; i++)
        {
            pA->send.format.data[i] = uMenLogData[i];  // 日志数据
        }
        pA->send.format.data[200] = 0x55;   //CRC校验低位
        pA->send.format.data[201] = 0xAA;   //CRC校验高位
        IecCreateFrameI(P_LOG,0x01,R_DATA_TRANS,LOGFRAMELENTH+2,&pA->send);

        Iaddr.uInfAddr++;
        break;

    case R_TRANS_STOP:

        Iaddr.uInfAddr = Iaddr.uInfAddr -1;
        pA->send.format.maddrL = 0x00;  // 信息体地址
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x85;
        IecCreateFrameI(P_LOG,0x01,R_TRANS_STOP_ACK,1,&pA->send);   //停止传输确认

        pA->send.format.maddrL = Iaddr.uInfTemp[0];  // 信息体地址，总包数
        pA->send.format.maddrM = Iaddr.uInfTemp[1];
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x86;
        IecCreateFrameI(P_LOG,0x01,R_TRANS_FINISH,1,&pA->send); //传输完成

        memset(uMenLogData,0,LOGFRAMELENTH);
        uLogDataFlash= 0;
        Iaddr.uInfAddr = 0;
        counts.uInfAddr = 0;

        break;

    case R_TRANS_FINISH_ACK:

        break;
    }
    
    return 0;
}
/******************************************************************************
* 名    称：Iec104HandleEsn()
* 功    能：数采ESN处理。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleEsn(IEC104_MAIN_T *pA)
{
	if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.esn,pA->recv.format.data,20);
        DEBUGOUT("存Esn\n");
        SaveEepData(EEP_LOGGER_INF);
    }
	else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.esn,20);
    }

    IecCreateFrameI(P_CL_ESN,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* 名    称：Iec104HandleLoggerType()
* 功    能：处理数采TYPE。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleLoggerType(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.type,pA->recv.format.data,20);
        DEBUGOUT("存类型\n");

        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.type,20);
    }

    IecCreateFrameI(P_CL_TYPE,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* 名    称：Iec104HandleLoggerModel()
* 功    能：处理数采MODEL。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleLoggerModel(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.model,pA->recv.format.data,20);
        DEBUGOUT("存型号\n");
        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.model,20);
    }

    IecCreateFrameI(P_CL_MODEL,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* 名    称：Iec104HandleLoggerName()
* 功    能：处理数采TYPE。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleLoggerName(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // 设置92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.name,pA->recv.format.data,20);
        DEBUGOUT("存名称\n");
        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 查询90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.name,20);
    }

    IecCreateFrameI(P_CL_NAME,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* 名    称：Iec104HandleHwEsnRenovate()
* 功    能：下联设备esn查询(仅限华为设备）D4
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleHwEsnRenovate(IEC104_MAIN_T *pA)
{
	uint8_t uMbVirAddr;   // 设备地址:限定最多接0-9十个逆变器
	uint8_t uMbRelAddr=0; // 根据MbVirAddr映射出的设备实际地址

    if(0x93==pA->recv.format.reasonL)
    {
        uMbVirAddr = pA->recv.format.maddrL;
        uMbRelAddr = IecAddrConvert(uMbVirAddr);

        if(uMbRelAddr<MAX_device)
        {
            memcpy(g_DeviceEsn.cDeviceEsn[uMbRelAddr],pA->recv.format.data,20);
            SaveEepData(EEP_DEVICE_ESN);
        }
    }
}
/******************************************************************************
* 名    称：Iec104HandleHwSoftRenovate()
* 功    能：下联设备软件版本n查询(仅限华为设备）
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleHwSoftRenovate(IEC104_MAIN_T *pA)
{
    uint8_t i,k;

    IEC104_MAIN_T *pstIec104=pA;
    uint8_t uMbVirAddr;   // 设备地址:限定最多接0-9十个逆变器
    uint8_t uMbRelAddr=0; // 根据MbVirAddr映射出的设备实际地址


    uMbVirAddr = pstIec104->recv.format.data[0];
    uMbRelAddr = IecAddrConvert(uMbVirAddr);

    if(uMbRelAddr<MAX_device)
    {
        k = pstIec104->recv.format.len -13;
        for(i=0;i<k;i++)
        {
            if('2'==pstIec104->recv.format.data[i])
            {
                i++;
                if('='==pstIec104->recv.format.data[i])
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
        /*
        if(0==g_uNextFrame)  // 上报了一帧华为设备软件版本信息（未上报C4 92）
        {
            memcpy(Device_soft.device_soft[uMbRelAddr],&pstIec104->recv.format.data[i],17);
        }*/
        DEBUGOUT("\n设备%d软件版本变为: %0.17s\n",uMbVirAddr,g_DeviceSoft.cDeviceSoft[uMbRelAddr]);
        SaveEepData(EEP_DEVICE_SOFT);

    }
}
/******************************************************************************
* 名    称：Iec104HandleDelHwDev()
* 功    能：下联设备删除（仅限华为设备）D5
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleDelHwDev(IEC104_MAIN_T *pA)
{
	//uint8_t uMbVirAddr;//设备地址:限定最多接0-9十个逆变器
	uint8_t uMbRelAddr=0;//根据MbVirAddr映射出的设备实际地址
	uint8_t uDelCmd;

	//memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

	IecCpoyMesAddr(pA);

	pA->send.format.data[0] = pA->recv.format.data[0];

	uMbRelAddr=IecAddrConvert(pA->recv.format.maddrL);

	uDelCmd=pA->recv.format.data[0];

    if(R_ACTIVE==pA->recv.format.reasonL && 0x81==uDelCmd)
    {
        //Iec104PreDelHwDev();
    }
    else if(R_ACTIVE==pA->recv.format.reasonL && 0x01==uDelCmd)
    {
        if(uMbRelAddr<MAX_device)
        {
            g_DeviceSouth.device_inf[uMbRelAddr].addr=0;
			g_DeviceEsn.uEsnMark[pA->recv.format.maddrL]=0;
			DEBUGOUT("addr:%d uEsnMark:%d",pA->recv.format.maddrL,g_DeviceEsn.uEsnMark[g_DeviceSouth.device_inf[uMbRelAddr].addr]);
            if(g_DeviceSouth.device_sum>=1)
            {
                g_DeviceSouth.device_sum--;
            }
            g_LoggerRun.err_lost &= ~(1<<uMbRelAddr);
            g_LoggerAlarm.dev_lost &= ~(1<<uMbRelAddr);

            SaveEepData(EEP_DEVICE_SOUTH);
			SaveEepData(EEP_DEVICE_ESN);  //为了保存分配地址表计
            SaveEepData(EEP_ALARM);  // 保存告警值
        }
    }

    IecCreateFrameI(P_HW_DEL,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
}
/******************************************************************************
* 名    称：Iec104HandleReset()
* 功    能：恢复出厂设置
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleReset(IEC104_MAIN_T *pA)
{
	uint8_t uRstCmd;

	uRstCmd=pA->recv.format.data[0];
	//memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

    IecCpoyMesAddr(pA);

	pA->send.format.data[0] = pA->recv.format.data[0];

    if(R_ACTIVE==pA->recv.format.reasonL && 0x81==uRstCmd)
    {
        //做相关预置操作
        //Iec104PreSet();
        IecCreateFrameI(P_CL_RESET,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
    }
    else if(R_ACTIVE==pA->recv.format.reasonL && 0x01==uRstCmd)
    {
        AllReset(1);
        IecCreateFrameI(P_CL_RESET,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);

        sleep(5);
        Reboot();
    }
}
/******************************************************************************
* 名    称：Iec104HandleReboot()
* 功    能：数采复位
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104HandleReboot(IEC104_MAIN_T *pA)
{
	uint8_t uRbtCmd;

	uRbtCmd=pA->recv.format.data[0];
	//memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

	IecCpoyMesAddr(pA);
	pA->send.format.data[0] = pA->recv.format.data[0];

    if(R_ACTIVE==pA->recv.format.reasonL && 0x81==uRbtCmd)
    {
        //做相关预置操作
        IecCreateFrameI(P_CL_REBOOT,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
    }
    else if(R_ACTIVE==pA->recv.format.reasonL && 0x01==uRbtCmd)
    {
        IecCreateFrameI(P_CL_REBOOT,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
        sleep(5);
        Reboot();
    }
}
/******************************************************************************
* 名    称：Iec104TimeCollectPre()
* 功    能：补采预处理
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104TimeCollectPre(IEC104_MAIN_T *pA)
{
    if(R_ACTIVE==pA->recv.format.reasonL) // 激活06
    {
        // 测试，字节结束总召
        //if(SUBCOLLECT_NULL == iec_subcollect_run.subcollect)
        {
            //将收到的时标保存下来
            g_TimeMarker[0] = pA->recv.format.data[1];
            g_TimeMarker[1] = pA->recv.format.data[2];
            g_TimeMarker[2] = pA->recv.format.data[3];
            g_TimeMarker[3] = pA->recv.format.data[4];
            g_TimeMarker[4] = pA->recv.format.data[5];
            g_TimeMarker[5] = pA->recv.format.data[6];
            g_TimeMarker[6] = pA->recv.format.data[7];

            g_sIecRun.subcollect = SUBCOLLECT_START;
        }

    }
    else if(R_RECOLLECT_END==pA->recv.format.reasonL) // 0x8F   // 补采结束
    {
        if(0==pA->recv.format.data[0]) // 查询是否有数据存在
        {
            // 测试直接发送有数据
            IecCpoyMesAddr(pA);

            pA->send.format.data[0] = 0x01;
            IecCreateFrameI(P_TIME_COLLECT,pA->recv.format.limit,R_RECOLLECT_END,1,&pA->send);
        }
        else if(0x8F==pA->recv.format.data[0]) // 补采结束
        {
            IecProcessFrameS(pA,1);// 发送S帧确认
        }
    }
    else if(R_ACTIVE_END==pA->recv.format.reasonL) // 0x0A   //激活结束
    {
        IecProcessFrameS(pA,1);// 发送S帧确认
    }
}
/******************************************************************************
* 名    称：Iec104Yk()
* 功    能：遥控。2E
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104Yk(IEC104_MAIN_T *pA)
{
    Uint_Char_Convert temp;
    if(R_ACTIVE==pA->recv.format.reasonL) // 激活06
    {
        temp.c[0] = pA->recv.format.maddrL;
        temp.c[1] = pA->recv.format.maddrM;

        temp.u = temp.u - 0x6001;  // 计算出相对遥控地址

        if(temp.u >= g_DeviceSouth.yk_sum)
        {
            DEBUGOUT("非法遥控地址\n");
        }
//        if(g_uRecYk&0x01)
//        {
//            DEBUGOUT("已存在未完成的遥控指令!\n");
//            return;
//        }
        if(pA->recv.format.data[0]&0x80)
        {
            IEC104_DATA_YK[temp.u] = pA->recv.format.data[0];  // 遥控点值
        }

        if(0==(pA->recv.format.data[0]&0x80) && IEC104_DATA_YK[temp.u]&0x80)  // 遥控确认
        {
            //g_uRecYk |= 0x01;  // 标记有收到遥控信息
            OSQPost(MesQ, &s_uSouthYk);
        }

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = pA->recv.format.data[0];
        IecCreateFrameI(C_DC_NA_1,pA->recv.format.limit,R_ACTIVE_ACK,1,&pA->send);
    }
    else if(R_ACTIVE_STOP==pA->recv.format.reasonL) // 停止激活08
    {
        temp.c[0] = pA->recv.format.maddrL;
        temp.c[1] = pA->recv.format.maddrM;

        temp.u = temp.u - 0x6001;  // 计算出相对遥控地址
        IEC104_DATA_YK[temp.u] = 0x00;  // 遥控点值

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = pA->recv.format.data[0];
        IecCreateFrameI(C_DC_NA_1,pA->recv.format.limit,R_ACTIVE_STOP_ACK,1,&pA->send);
    }
}
/******************************************************************************
* 名    称：Iec104SD()
* 功    能：设点。D9
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void Iec104SD(IEC104_MAIN_T *pA)
{
    Uint_Char_Convert temp,temp_104;
	U32_F_Char_Convert  ctemp;
	uint8_t SdSum;
	//uint8_t err;
	//uint8_t *pMes;

    if(R_SETTING==pA->recv.format.reasonL) // 激活06
    {
		SdSum = pA->recv.format.limit & 0x7f;
		SdContinuousAddr=0;    //遥调的非连续地址
		temp.c[0] = pA->recv.format.maddrL;
		temp.c[1] = pA->recv.format.maddrM;

		temp_104.u= temp.u;

		if(temp.u > 0x6400)
		{
			return;
		}
		temp.u = temp.u - 0x6201;  // 计算出相对遥控地址

		if(temp.u >= g_DeviceSouth.sd_sum)
		{
			DEBUGOUT("非法遥调地址\n");
		}
		memcpy(ctemp.c,pA->recv.format.data,sizeof(ctemp.c));
		ctemp.u=ctemp.f;
		IEC104_DATA_SD[0] = SdSum;  // 遥调点数
		IEC104_DATA_SD[1] = temp_104.u;  // 遥调地址
		IEC104_DATA_SD[2] = ctemp.u;  	// 遥调点值

		for(uint8_t i=1;i< SdSum;i++)
		{
			temp.c[0] = pA->recv.format.data[i*7-3];
			temp.c[1] = pA->recv.format.data[i*7-2];

			temp_104.u= temp.u;

			if(temp.u > 0x6400)
			{
				return;
			}
			temp.u = temp.u - 0x6201;  // 计算出相对遥调地址
			memcpy(ctemp.c,&pA->recv.format.data[i*7],sizeof(ctemp.c));
			ctemp.u=ctemp.f;
			
			IEC104_DATA_SD[i*2+1] = temp_104.u;  	// 遥调地址
			IEC104_DATA_SD[(i+1)*2] = ctemp.u;  	// 遥调点值
		}
		OSQPost(MesQ, &s_uSouthSd);

		sleep(SdSum/2);
        IecCpoyMesAddr(pA);
		memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-10));// I帧数据
        IecCreateFrameI(p_SD,pA->recv.format.limit,R_SET_SUC,7*SdSum-3,&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // 停止激活08
   {
		if(pA->recv.format.limit & 0x80)	 //遥调的连续地址
		{
			 SdSum = pA->recv.format.limit - 0x80;

			 SdContinuousAddr=1;    //遥调的连续地址 
			 temp.c[0] = pA->recv.format.maddrL;
			 temp.c[1] = pA->recv.format.maddrM;
             
             if(temp.u > 0x6400)
             {
                 return;
             }
			 temp.u = temp.u - 0x6201;	// 计算出相对遥控地址

			 if(temp.u >= g_DeviceSouth.sd_sum)
			 {
				  DEBUGOUT("非法遥调地址\n");
			 }
			 IEC104_DATA_SD[0] = SdSum;		// 查询的遥调点个数
			 IEC104_DATA_SD[1] = temp.u;	// 查询的第几个遥调点
			 OSQPost(MesQ, &s_uSouthReadSd);
			 
			 sleep(2);
			 for(uint8_t i=0;i<SdSum;i++)
			 {
				ctemp.u=IEC104_DATA_SD[i];
				ctemp.f=ctemp.u;
				pA->send.format.data[0+i*4]=ctemp.c[0];
				pA->send.format.data[1+i*4]=ctemp.c[1];
				pA->send.format.data[2+i*4]=ctemp.c[2];
				pA->send.format.data[3+i*4]=ctemp.c[3];
				IEC104_DATA_SD[i]=0;
			 }
			IecCpoyMesAddr(pA);
			IecCreateFrameI(p_SD,pA->recv.format.limit,R_INQUIRE_SUC,4*SdSum,&pA->send);
		} 
    }
}
/******************************************************************************
* 名    称：IecProcessFrameI()
* 功    能：IEC104 I帧处理。
* 入口参数：
*           *pA         IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
static void IecProcessFrameI(IEC104_MAIN_T *pA)
{
    switch(pA->recv.format.type)  				// 类型标识
    {
    case C_IC_NA_1:								// 总召唤命令
        if(R_ACTIVE==pA->recv.format.reasonL) 	// 总召激活
        {
            IecProcessFrameS(pA,1);    			// 发送S帧
            g_sIecRun.collect = COLLECT_START; 	// 开始总召
            DEBUGOUT("发起总召\n");
        }
        break;

    case C_CI_NA_1:   							// 电度总召唤命令
        g_sIecRun.dd_collect = COLLECT_DD_START;// 开始总召
        break;

    case M_SP_TB_1:     						//0x1E  //30   // 带时标CP56Time2a的单点信息
        break;

    case M_DP_TB_1:     						//0x1F  //31   // 带时标CP56Time2a的双点信息
        break;

    case M_ME_TF_1:     						//0x24  //36   // 带时标CP56Time2a的测量值，短浮点数
        break;

    case C_CS_NA_1:   							// 时钟同步命令67
        Iec104SyncTime(pA);
        break;

    case C_SC_NA_1:   							// 单点信息0x2D
        break;

    case C_DC_NA_1:   							// 双点信息0x2E
        Iec104Yk(pA);
        break;
	case p_SD:       							// 平台遥调设点0xD9
        Iec104SD(pA);
        break;

    case P_UPDATA:   							// 升级B4
        Iec104LoggerUpdate(pA);
        break;
    case P_FILE_INOUT:
		Iec104DT1000Update(pA); 				//表计升级F0
		break;
    case P_SET_IP:   							//设备IP配置B5
        Iec104IP(pA);
        break;

    case P_MODBUS_ENDIAN:						// MODBUS大小端配置B6
        Iec104HandleModBusEndian(pA);
      	break;

    case P_MODBUS_BAUD:							// MODBUS波特率配置B7
        Iec104HandleModBusBd(pA);
		 break;

    case P_ADDR:								// 设备公共地址配置B8
        Iec104PublicAddr(pA);
        break;

    case P_VERSION:								// 版本查询B9
        Iec104InquireVer(pA);
        break;

    case P_ROLLBAOCK:							// 版本回滚BA
        Iec104LoggerRollBack(pA);
        break;

    case P_INPUT_TABLE:							// 点表导入BB
        Iec104InputTable(pA);
        break;

    case P_REPORT_NUM:							// 上报号码配置BC
        Iec104PhoneNum(pA);
        break;

    case P_INIT_FINISH:							// 设备初始化是否完成BD

        break;

    case P_TIME_COLLECT:						// 带时标总召唤BE  补采
        Iec104TimeCollectPre(pA);
        DEBUGOUT("发起补采\n");
        break;

    case P_ERR_PROCESS:							// 异常处理BF
		if(R_ACTIVE_ACK==pA->recv.format.reasonL)	//异常处理
	   {
		   AlarmAck(pA->recv.format.len-13,pA->recv.format.data);
	   }
        break;

    case P_DEV_INFO:							// 设备基本信息C0
        IecReportLogInfo(R_INQUIRE_SUC);  		// 上报数采信息
        break;

    case P_STATION_INFO:						// 站点基本信息C1
        break;

    case P_SERVICE_INFO:						// 平台基本信息C2
        break;

    case P_CREATE_STATION:						// 建站指令C3
        break;

    case P_SOUTH_INFO:							// 下联设备信息C4
        Iec104SouthDevInfo(pA);
        break;

    case P_TABLE:								// 导表指令C5
        Iec104TableState(pA);
        break;

    case P_COM_MODE:							// 通信方式C6
        Iec104CommMode(pA);
        break;

    case P_HW_INFO:								// 华为设备信息C7
        Iec104DevReportAck(pA);
        break;

    case P_SERVICE_IP:							// 服务器IP/域名C8
        Iec104ServiceIP(pA);
        break;

    case P_SERVICE_PORT:						// 服务器端口C9

        break;

    case P_LOG:									// 日志处理CA
        Iec104LogExport(pA);
        break;

    case P_LOCATION:							// 经纬度CB
        break;

    case P_SOUTH_STATUS:						// 下联设备状态CC
        break;

    case P_CL_ESN:								// 数采ESN配置D0
        Iec104HandleEsn(pA);
        break;

    case P_CL_TYPE:								// 数采类型配置D1
        Iec104HandleLoggerType(pA);
        break;

    case P_CL_MODEL:							// 数采型号配置D2
        Iec104HandleLoggerModel(pA);
        break;

    case P_CL_NAME:								// 数采设备名称配置D3
        Iec104HandleLoggerName(pA);
        break;

    case P_HW_ESN:								// 下联设备esn查询(仅限华为设备）D4
        Iec104HandleHwEsnRenovate(pA);
        break;

    case P_HW_DEL:								// 下联设备删除（仅限华为设备）D5
        Iec104HandleDelHwDev(pA);
        break;

    case P_SIM_ID: 								// SIM卡ID上报
        break;

    case P_MAX_DEVICE:

        break;

    case P_CL_RESET:							// 恢复出厂设置FA
        Iec104HandleReset(pA);
        break;

    case P_CL_REBOOT:							// 复位FB
        Iec104HandleReboot(pA);
        break;

    default:
        break;
    }
}

//===================================================================================
OS_STK TaskIec104ProcessStk[TaskIec104ProcessStkSize]@0x20000400;	      // 定义任务堆栈大小
/****************************************************************************
* 名    称：TaskSouthInquire()
* 功    能：南向查询任务
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
void TaskIec104Process(void *p)
{
    p = p;
    uint8_t uDataLen;

    IecInit();

    while(1)
    {
        if(g_sIecRun.collect || g_sIecRun.dd_collect)
        {
            IecCollectProcess(&g_sIEC,&g_sIecRun);  // 总召
        }
        else if(g_sIecRun.subcollect)
        {
            IecSubSCollect(&g_sIEC,&g_sIecRun);   // 补采
        }

        if(g_LoggerRun.update&0xF0)  // 远程升级超时保护
        {
            g_uNowTime = OSTimeGet();

            if(TIMEOUT==TimeOut(g_uUpdataTime,g_uNowTime,90000)) // 升级超时保护90秒
            {
                DEBUGOUT("升级超时,复位\n");
                Reboot();
            }
        }

        uDataLen = CmRecLinkLen();

        if(0==uDataLen)
        {
            msleep(20);
            continue;
        }

        memset(g_sIEC.recv.buff,0,sizeof(g_sIEC.recv.buff));

        uDataLen = CmReadRecLink(g_sIEC.recv.buff);

        if(IEC104_HEAD == g_sIEC.recv.format.start)
        {
            if(uDataLen != (g_sIEC.recv.format.len+2))
            {
                DEBUGOUT("\n104长度不对\n");
                return;
            }
            g_sIEC.FrameCommand = g_sIEC.recv.format.SseqL & 0x03;

            if(1==g_sIEC.FrameCommand)   // 收到S帧
            {
                IecProcessFrameS(&g_sIEC,0);
            }
            else if(3==g_sIEC.FrameCommand) // 收到U帧
            {
                IecProcessFrameU(&g_sIEC,g_sIEC.recv.format.SseqL & 0xfc,0);
            }
            else // 收到I帧
            {
                g_IecRecSeq++;  // 接收序列+1
                IecProcessFrameI(&g_sIEC);
            }
        }
    }
}
/******************************************************************************
* 名    称：IecCollectProcess()
* 功    能：IEC104 总召处理。
* 入口参数：
            bytes     数据长度
            *pA       IEC信息指针

* 出口参数：无
* 范    例:
******************************************************************************/
void IecCollectProcess(IEC104_MAIN_T *pA,IEC_RUNNING_T *call)
{
    static uint16_t s_uIecHead; // 要发送数据的头
    uint16_t uIecCount;         // 信息点数量
    uint16_t uCollectSum;       //
    uint8_t qty=0;              // 一帧IEC I帧包含总召点数计数
    uint8_t temp;
    uint8_t uRelTable;          // 相对点表号

    Uint_Char_Convert  addr;
    U32_F_Char_Convert yctemp;


    if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        call->collect = COLLECT_NULL;
        call->dd_collect = COLLECT_NULL;
        return;
    }

    if(g_LoggerRun.update)  // 进入升级
    {
        if(COLLECT_NULL!=call->collect)
        {
            call->collect = COLLECT_END;      // // 总召结束
        }

        if(COLLECT_NULL!=call->dd_collect)
        {
            call->dd_collect = COLLECT_DD_END;   // // 总召结束
        }
    }

    switch(call->collect)
    {
    case COLLECT_START:  // 开始总召
        //Iec104ProcessFrameS(pA,1);    // 发送S帧
        pA->send.format.maddrL = 0x00;// 信息体地址
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x14;  // 。。。。。。
        IecCreateFrameI(C_IC_NA_1,0x01,R_ACTIVE_ACK,1,&pA->send);

        if(RUNNING_WORK_READ!=g_LoggerRun.run_status|| -1==CheckDevState())  // 数采，没有开站，暂停南向工作；所有南向设备地址都为0
        {
            call->collect = COLLECT_END;  // 总召结束
            break;
        }

        s_uIecHead = 0;  // 遥信起始地址为0x0001，相对存储数据的数组下标0开始
        call->collect = COLLECT_YX;  // 下次发送遥信数据
        call->uRelAddr = 0;
        break;

    case COLLECT_YX:     // 发送遥信数据
        // 一帧255字节，基础信息占用12字节，单点占用4个字节，一共可传输

        msleep(500);  // 延时500ms

        if(NULL==IEC104_DATA_YX)
        {
            DEBUGOUT("无遥信空间\n");
            if(g_DeviceSouth.yx_sum)  // 如果遥信点总数为0，则不报信息
            {
                DEBUGOUT("无遥信点\n");
            }

            call->collect = COLLECT_YC;  // 下次发送遥测
        }

        //----------------------------------------------------------------
        // 一帧255字节，基础信息占用12字节，起始信息体地址占用3字节，单点占用5字节，一共可传输48个点
        if(!s_uIecHead)
        {
            for(temp=call->uRelAddr;temp<g_DeviceSouth.device_sum;temp++)
            {
                 // 点表没有遥信点时，遥信起始地址为0
                if(g_DeviceSouth.device_inf[call->uRelAddr].addr && g_DeviceSouth.device_inf[call->uRelAddr].yx_start_addr)
                {
                    s_uIecHead = g_DeviceSouth.device_inf[call->uRelAddr].yx_start_addr - 0x01;
                    break;
                }
                else
                {
                    call->uRelAddr++;
                }
            }

            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)
            {
                s_uIecHead = 0;  // 遥信起始地址为0x0001，相对存储数据的数组下标0开始
                call->uRelAddr = 0;
                call->collect = COLLECT_YC;  // 下次发送遥测数据
                break;
            }
        }


        //----------------------------------------------------------------

        uRelTable = g_DeviceSouth.device_inf[call->uRelAddr].rel_num; // 相对点表号

        //addr.u = s_uIecHead+0x01;  // 信息体地址

        //pA->send.format.maddrL = addr.c[0];// 信息体地址
        //pA->send.format.maddrM = addr.c[1];
        //pA->send.format.maddrH = 0x00;

        uCollectSum = g_DeviceSouth.device_inf[call->uRelAddr].yx_start_addr - 0x01 + g_sIecPointCount[uRelTable].uYxSum;

        //for(uIecCount=s_uIecHead,qty=0; uIecCount<g_DeviceSouth.yx_sum && qty<60; uIecCount++)
        for(uIecCount=s_uIecHead,qty=0; uIecCount<uCollectSum && qty<60; uIecCount++)
        {
            temp = qty*4;
            addr.u = s_uIecHead + 0x01;          // 信息体地址
            pA->send.buff[temp+12] = addr.c[0]; // iec.ibuff[temp]   = addr.c[0]; // uIecCount&0xff;信息体地址低位
            pA->send.buff[temp+13] = addr.c[1]; // iec.ibuff[temp+1] = addr.c[1]; // uIecCount>>8;  信息体地址高位
            pA->send.buff[temp+14] = 0x00;      // iec.ibuff[temp+2] = 0x00;      // 信息体地址，无效位
            pA->send.buff[temp+15] = IEC104_DATA_YX[uIecCount];//iec.ibuff[temp+3] = IEC104_DATA_YX[uIecCount];

            qty++;
            s_uIecHead++;
        }

        if(uIecCount >= uCollectSum)  // 一台遥信总召发送完成
        {
            if(call->uRelAddr < (g_DeviceSouth.device_sum-1) && call->uRelAddr < (MAX_device-1))
            {
                s_uIecHead = 0;
                call->uRelAddr++;
            }
            else
            {
                s_uIecHead = 0;
                call->uRelAddr = 0;
                call->collect = COLLECT_YC;  // 下次发送遥测
            }
        }

        if(qty)
        {
            IecCreateFrameI(M_SP_NA_1,qty,R_COLLECT_ACK,qty*4-3,&pA->send);
        }

        /*if(uIecCount>=g_DeviceSouth.yx_sum)  // 遥信数据已经发送完成
        {
            s_uIecHead = 0;// 遥信起始地址为0x4001，相对存储数据的数组下标0开始
            call->collect = COLLECT_YC;  // 下次发送遥测数据
            call->uRelAddr = 0; // 从相对地址为0的设备开始响应总召
        }*/
        break;

    case COLLECT_YC:     // 发送遥测数据

        msleep(500);  // 延时500ms

        if(NULL==IEC104_DATA_YC)
        {

            DEBUGOUT("无遥测空间\n");
			
            if(g_DeviceSouth.yc_sum)  // 如果遥测点总数为0，则不报信息
            {
                DEBUGOUT("无遥测点\n");
            }

            call->collect = COLLECT_END;  // 总召结束
        }
        //----------------------------------------------------------------
        // 一帧255字节，基础信息占用12字节，起始信息体地址占用3字节，单点占用5字节，一共可传输48个点
        if(!s_uIecHead)
        {
            for(temp=call->uRelAddr;temp<g_DeviceSouth.device_sum;temp++)
            {
                if(g_DeviceSouth.device_inf[call->uRelAddr].addr && g_DeviceSouth.device_inf[call->uRelAddr].yc_start_addr>=0x4001)
                {
                    s_uIecHead = g_DeviceSouth.device_inf[call->uRelAddr].yc_start_addr - 0x4001;
                    break;
                }
                else
                {
                    call->uRelAddr++;
                }
            }
            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)
            {
                s_uIecHead = 0;  // 遥信起始地址为0x0001，相对存储数据的数组下标0开始
                call->uRelAddr = 0;
                call->collect = COLLECT_END;  // 总召结束
                break;
            }
        }

        //----------------------------------------------------------------

        uRelTable = g_DeviceSouth.device_inf[call->uRelAddr].rel_num; // 相对点表号

        addr.u = s_uIecHead + 0x4001;  // 信息体地址

        pA->send.format.maddrL = addr.c[0];// 信息体地址
        pA->send.format.maddrM = addr.c[1];
        pA->send.format.maddrH = 0x00;

        uCollectSum = g_DeviceSouth.device_inf[call->uRelAddr].yc_start_addr - 0x4001 + g_sIecPointCount[uRelTable].uYcSum;

        for(uIecCount=s_uIecHead,qty=0; uIecCount<uCollectSum && qty<48; uIecCount++)
        {
            temp = qty*5;

            //if(0xFFFFFFFF!=IEC104_DATA_YC[uIecCount])
            {
                yctemp.u = IEC104_DATA_YC[uIecCount];
                pA->send.format.data[temp]   = yctemp.c[0];
                pA->send.format.data[temp+1] = yctemp.c[1];
                pA->send.format.data[temp+2] = yctemp.c[2];
                pA->send.format.data[temp+3] = yctemp.c[3];
            }
            /*else
            {
                pA->send.format.data[temp]   = 0xFF;
                pA->send.format.data[temp+1] = 0xFF;
                pA->send.format.data[temp+2] = 0xFF;
                pA->send.format.data[temp+3] = 0xFF;
            }*/

            pA->send.format.data[temp+4] = 0x00; // 品质描述

            qty++;
            s_uIecHead++;
        }

        if(uIecCount >= uCollectSum)  // 一台遥测总召发送完成
        {
            if(call->uRelAddr < (g_DeviceSouth.device_sum-1) && call->uRelAddr < (MAX_device-1))
            {
                s_uIecHead = 0;
                call->uRelAddr++;
            }
            else
            {
                call->collect = COLLECT_END;  // 总召结束
            }
        }

        if(qty)  // 有需要上报的遥测点
        {
            IecCreateFrameI(M_ME_NC_1,qty|0x80,R_COLLECT_ACK,qty*5,&pA->send); // 连续信息点，可变结构限定词最高位置1
        }

        /*if(uIecCount>=g_DeviceSouth.yc_sum)  // 遥信数据已经发送完成
        {
            s_uIecHead = 0;
            call->collect = COLLECT_END;  // 总召结束
        }*/
        break;

    case COLLECT_END:    // 总召结束
        pA->send.format.maddrL = 0x00;// 信息体地址
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x14;  // 。。。。。。
        IecCreateFrameI(C_IC_NA_1,0x01,R_ACTIVE_END,1,&pA->send);

        call->collect = COLLECT_NULL;  // 没有总召
        break;

    default:
        call->collect = COLLECT_NULL;  // 没有总召
        break;
    }
    // 电度总召
    switch(call->dd_collect)
    {
    case COLLECT_DD_START:  // 电度总召
        /*pA->send.format.maddrL = 0x00;// 信息体地址
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x45;  // 。。。。。。
        IecCreateFrameI(C_CI_NA_1,0x01,R_ACTIVE_ACK,1,pA); // 电度总召确认
        break;*/

    case COLLECT_DD_END:    // 电度总召结束
        pA->send.format.maddrL = 0x00;// 信息体地址
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x45;  // 。。。。。。
        IecCreateFrameI(C_CI_NA_1,0x01,R_ACTIVE_END,1,&pA->send);

        call->dd_collect = COLLECT_NULL;  // 没有总召
        break;

    default:
        call->dd_collect = COLLECT_NULL;  // 没有总召
        break;
    }
    return;
}

/******************************************************************************
* 名    称：IecSubSCollect()
* 功    能：平台数据补采。
* 入口参数：
*           *pA         IEC信息指针
*			*subcall
* 出口参数：无
* 范    例:
* ADD BY: chenyang
******************************************************************************/
void IecSubSCollect(IEC104_MAIN_T *pA,IEC_RUNNING_T *subcall)
{
    //static uint16_t uTimer=0;                   // 发送补采数据间隔时间
    static uint16_t uPerDeviceSum = 0;	         // 当前设备已发送的信息点总数sum++
    static uint16_t sAllSum = 0;                // 已发送的所有遥信遥信信息点的总和
    static int32_t uRecord = NO_DATA;
    static uint32_t uNeedReadFlash = 0;         // 需要读取发送的dataflash地址

    uint8_t temp,tmp;
    uint8_t qty=0;			                    // 一帧IEC I帧包含补采点数计数
    uint8_t uRest;                              // 每台设备剩余的待发送信息点数
    uint8_t uRelTable;		                    // 相对点表号
    uint16_t iec_count; 	                    // 信息点计数
    uint16_t uCollectSum;			            // 每台设备的信息点总数
    uint32_t uReadFlashAddr;                    // DataFlash地址
    //uint8_t uMemYxAddr[YXBUFFSIZE] = {0};       // 读取遥信的长度，一帧读取58个长度,预留8个标志位。
    //uint8_t uMemYcAddr[YCBUFFSIZE] = {0};       // 读取遥测的长度，一帧读取46*4个长度，预留前面8个标志位。
    Uint_Char_Convert  addr;
    U32_F_Char_Convert yctemp;

    //uint8_t err;

    if(SUBCOLLECT_NULL==subcall->subcollect || COLLECT_NULL!= subcall->collect)// 没有补采或总召正在进行
    {
    	return;
    }
    if(NORTH_OK!=g_LoggerRun.north_status)  // 连接服务器未完成
    {
        subcall->subcollect = SUBCOLLECT_NULL;
        return;
    }

    /*根据平台下发的时标来查找应该上报的数据*/
    if(SUBCOLLECT_START == subcall->subcollect)
    {
        uRecord = CollectData(pA,&uNeedReadFlash);
        if(NO_DATA == uRecord)
        {
            subcall->subcollect = SUBCOLLECT_END;
        }
        else
        {

            subcall->uRelAddr = 0;
            subcall->subcollect = SUBCOLLECT_YX;
        }

        pA->send.format.maddrL = pA->recv.format.maddrL;// 信息体地址
        pA->send.format.maddrM = pA->recv.format.maddrM;
        pA->send.format.maddrH = pA->recv.format.maddrH;
        pA->send.format.data[0] = R_COLLECT_ACK;    //响应总召唤
        pA->send.format.data[1] = g_TimeMarker[0];
        pA->send.format.data[2] = g_TimeMarker[1];
        pA->send.format.data[3] = g_TimeMarker[2];
        pA->send.format.data[4] = g_TimeMarker[3];
        pA->send.format.data[5] = g_TimeMarker[4];
        pA->send.format.data[6] = g_TimeMarker[5];
        pA->send.format.data[7] = g_TimeMarker[6];

        IecCreateFrameI(P_TIME_COLLECT,0x01,R_ACTIVE_ACK,8,&pA->send);

        return;//历史数据中未查找到需要上报的时标，该时间段记录已丢失
    }

    switch(subcall->subcollect)
    {
    case SUBCOLLECT_YX:     // 发送遥信数据

        if(NULL==IEC104_DATA_YX)
        {

            DEBUGOUT("无遥信空间\n");
            if(g_DeviceSouth.yx_sum)  // 如果遥信点总数为0，则不报信息
            {
                DEBUGOUT("无遥信点\n");
            }

            subcall->uRelAddr = 0;
            subcall->subcollect = SUBCOLLECT_YC;  // 下一次开始遥测补采

            break;
        }
        /*
         *一帧255字节，基础信息占用12字节，起始信息体地址占用3字节，
         *第1个信息点占用data[0]，和起始信息体地址3字节
         *尾部携带7个字节的时标，单点占用4字节，从第2个点开始计算，可以存到59
         **************************************************************/
        if(!uPerDeviceSum)
        {
            for(temp=subcall->uRelAddr; temp<MAX_device; temp++)
            {
                if(g_DeviceSouth.device_inf[subcall->uRelAddr].addr)//判断设备是否存在
                {
                    break;
                }
                else
                {
                    subcall->uRelAddr++;
                }
            }

            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)
            {
                subcall->uRelAddr = 0;
                subcall->subcollect = SUBCOLLECT_YC;  // 下次发送遥测数据
                return;
            }
        }

        //----------------------------------------------------------------

        uRelTable = g_DeviceSouth.device_inf[subcall->uRelAddr].rel_num; // 相对点表号
        uCollectSum =g_sIecPointCount[uRelTable].uYxSum;
        uRest = g_sIecPointCount[uRelTable].uYxSum - uPerDeviceSum;
        if(uRest>0&&uRest<=59)
        {
            uReadFlashAddr = uNeedReadFlash+8+sAllSum;

            //OSMutexPend(pRecordLock,0,&err);//请求信号量

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],uRest);

            //OSMutexPost(pRecordLock);   //释放信号量

        }
        else if(uRest>59)
        {
            //每台设备尚未发送的信息点数超过一帧所能发送的59信息点数
            uReadFlashAddr = uNeedReadFlash+8+sAllSum;

            //OSMutexPend(pRecordLock,0,&err);//请求信号量

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],59);

            //OSMutexPost(pRecordLock);   //释放信号量
        }
        else
        {
            subcall->uRelAddr++;
            break;  //没有遥信点的直接跳出本次
        }
        addr.u = sAllSum +0x01;  // 信息体地址
        pA->send.format.maddrL = addr.c[0];// 每帧报文第1个点的信息体地址
        pA->send.format.maddrM = addr.c[1];
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = g_uSubData[8];
        sAllSum ++;
        qty++;
        for(iec_count=uPerDeviceSum,qty=1; iec_count<(uCollectSum-1) && qty<58; )
        {
            temp = qty*4;
            addr.u = sAllSum +1+ 0x01;  // 从第2个点的信息体地址开始
            pA->send.format.data[temp+1] = addr.c[0]; // iec.ibuff[temp]   = addr.c[0]; // iec_count&0xff;信息体地址低位
            pA->send.format.data[temp+2] = addr.c[1]; // iec.ibuff[temp+1] = addr.c[1]; // iec_count>>8;  信息体地址高位
            pA->send.format.data[temp+3] = 0x00;      // iec.ibuff[temp+2] = 0x00;      // 信息体地址，无效位
            pA->send.format.data[temp+4] = g_uSubData[qty+1+8];
            pA->send.format.data[temp+5] = g_TimeMarker[0];
            pA->send.format.data[temp+6] = g_TimeMarker[1];
            pA->send.format.data[temp+7] = g_TimeMarker[2];
            pA->send.format.data[temp+8] = g_TimeMarker[3];
            pA->send.format.data[temp+9] = g_TimeMarker[4];
            pA->send.format.data[temp+10] = g_TimeMarker[5];
            pA->send.format.data[temp+11] = g_TimeMarker[6];

            sAllSum ++;
            uPerDeviceSum++;
            qty++;
        }

        if(iec_count >= (uCollectSum-1))  // 一台遥信补采发送完成
        {
            uPerDeviceSum=0;
            if(subcall->uRelAddr < (g_DeviceSouth.device_sum-1) && subcall->uRelAddr < (MAX_device-1))
            {
                subcall->uRelAddr++;
            }
            else
            {

                subcall->uRelAddr = 0;
                sAllSum = 0;
                subcall->subcollect = SUBCOLLECT_YC;  // 下一次开始遥测补采
            }
        }

        if(qty)
        {
            IecCreateFrameI(M_SP_TB_1,qty,R_COLLECT_ACK,((qty-1)*4+1+7),&pA->send);
        }
        break;

    case SUBCOLLECT_YC:     // 发送遥测数据

        msleep(500);// 延时500ms
        //----------------------------------------------------------------
        /*
         *一帧255字节，基础信息占用12字节，起始信息体地址占用3字节，报文末尾携带7字节时标
         *单点占用5字节，一共可传输46个点
         */
        if(!uPerDeviceSum)
        {
            for(temp=subcall->uRelAddr; temp<g_DeviceSouth.device_sum; temp++)
            {
                if(g_DeviceSouth.device_inf[subcall->uRelAddr].addr)
                {
                    break;
                }
                else
                {
                    subcall->uRelAddr++;
                }
            }
            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)
            {
                subcall->uRelAddr = 0;
                uRecord = NO_DATA;
                subcall->subcollect = SUBCOLLECT_END;  // 补采结束
                break;
            }
        }

        //----------------------------------------------------------------

        uRelTable = g_DeviceSouth.device_inf[subcall->uRelAddr].rel_num; // 相对点表号

        uCollectSum =g_sIecPointCount[uRelTable].uYcSum;
        uRest = g_sIecPointCount[uRelTable].uYcSum - uPerDeviceSum;
        if(uRest<=46)
        {
            uReadFlashAddr = uNeedReadFlash + 8 + g_DeviceSouth.yx_sum + sAllSum*4;

            //OSMutexPend(pRecordLock,0,&err);//请求信号量

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],uRest*4);

            //OSMutexPost(pRecordLock);   //释放信号量
        }
        else
        {
            //每台设备尚未发送的信息点数超过一帧所能发送的46信息点数
            uReadFlashAddr = uNeedReadFlash + 8 + g_DeviceSouth.yx_sum + sAllSum*4;

            //OSMutexPend(pRecordLock,0,&err);//请求信号量

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],46*4);

            //OSMutexPost(pRecordLock);   //释放信号量
        }

        addr.u = sAllSum + 0x4001;  // 信息体地址
        pA->send.format.maddrL = addr.c[0];// 信息体地址
        pA->send.format.maddrM = addr.c[1];
        pA->send.format.maddrH = 0x00;
        for(iec_count=uPerDeviceSum,qty=0; iec_count<uCollectSum && qty<46; iec_count++)
        {
            temp = qty*5;
            tmp  = qty*4;
            yctemp.c[0] = g_uSubData[tmp+8];
            yctemp.c[1] = g_uSubData[tmp+9];
            yctemp.c[2] = g_uSubData[tmp+10];
            yctemp.c[3] = g_uSubData[tmp+11];

            pA->send.format.data[temp]   = yctemp.c[0];
            pA->send.format.data[temp+1] = yctemp.c[1];
            pA->send.format.data[temp+2] = yctemp.c[2];
            pA->send.format.data[temp+3] = yctemp.c[3];
            pA->send.format.data[temp+4] = 0x00; // 品质描述
            pA->send.format.data[temp+5] = g_TimeMarker[0];
            pA->send.format.data[temp+6] = g_TimeMarker[1];
            pA->send.format.data[temp+7] = g_TimeMarker[2];
            pA->send.format.data[temp+8] = g_TimeMarker[3];
            pA->send.format.data[temp+9] = g_TimeMarker[4]; //上报的数据，低5位是日期，高3位是星期
            pA->send.format.data[temp+10] = g_TimeMarker[5];
            pA->send.format.data[temp+11] = g_TimeMarker[6];

            sAllSum ++;
            uPerDeviceSum++;
            qty++;
        }
        if(iec_count >= uCollectSum)  // 一台遥测补采发送完成
        {
            uPerDeviceSum=0;
            if(subcall->uRelAddr < (g_DeviceSouth.device_sum-1) && subcall->uRelAddr < (MAX_device-1))
            {
                subcall->uRelAddr++;
            }
            else
            {

                subcall->uRelAddr = 0;
                uNeedReadFlash = 0;
                uRecord = NO_DATA;
                sAllSum = 0;
                subcall->subcollect = SUBCOLLECT_END;  // 补采结束
            }
        }

        if(qty)  // 有需要上报的遥测点
        {
            IecCreateFrameI(M_ME_TF_1,qty|0x80,R_COLLECT_ACK,(qty*5+7),&pA->send); // 连续信息点，可变结构限定词最高位置1
        }
        break;

    case SUBCOLLECT_END:    // 一次数据发送结束

        pA->send.format.maddrL = 0x00;// 信息体地址
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = R_COLLECT_ACK;    //响应总召唤
        pA->send.format.data[1] = g_TimeMarker[0];
        pA->send.format.data[2] = g_TimeMarker[1];
        pA->send.format.data[3] = g_TimeMarker[2];
        pA->send.format.data[4] = g_TimeMarker[3];
        pA->send.format.data[5] = g_TimeMarker[4];
        pA->send.format.data[6] = g_TimeMarker[5];
        pA->send.format.data[7] = g_TimeMarker[6];


        subcall->uRelAddr = 0;
        subcall->subcollect = SUBCOLLECT_NULL;  // 没有补采
        IecCreateFrameI(P_TIME_COLLECT,0x01,R_ACTIVE_END,8,&pA->send);
        break;

        /*case SUBCOLLECT_CHECK:	// 平台查询是否还有数据需要采集

            pA->send.format.maddrL = 0x00;	// 信息体地址
            pA->send.format.maddrM = 0x00;
            pA->send.format.maddrH = 0x00;
            pA->send.format.data[0] = 0x01;	//00：不存在；01：存在

            Iec104CreateFrameI(P_TIME_COLLECT,0x01,R_RECOLLECT_END,1,pA);
            break;

        case SUBCOLLECT_END_CONFIRM:

            pA->send.format.start   = IEC104_HEAD;  // 启动符
            pA->send.format.len     = 0x04;
            pA->send.format.SseqL   = 0x01;    // 发送序号字节1
            pA->send.format.SseqH   = 0x00;    // 发送序号字节2
            pA->send.format.RseqL   = 0x0A;    // 接收序号字节1
            pA->send.format.RseqH   = 0x00;    // 接收序号字节2

            Iec104FrameSend(pA,pA->send.format.len+2);
            break;
           */

    default:

        subcall->collect = SUBCOLLECT_NULL;  // 没有补采
        memset(g_TimeMarker,0,7);
        break;

    }
    return;
}
