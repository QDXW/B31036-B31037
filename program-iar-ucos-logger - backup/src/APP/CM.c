#include "CM.h"
#include "GlobalVar.h"
#include "DataTransferArea.h"
#include "Usart.h"
#include "GPIO.h"
#include "IEC104.h"
#include "LED.h"
#include "tool.h"
#include "WatchDog.h"

//========================================================================

//========================================================================
#define  MI_JUDGE_OK         0     // 数据判断正确
#define  MI_JUDGE_TIMEOUT    1     // 等待数据超时
#define  MI_JUDGE_ERR        2     // 数据判读错误


#define  MI_START            10     // 模块初始化起始
#define  MI_AT               11     // 发送AT
#define  MI_NO_ECHO          12     // 关回显
#define  MI_BAUDRATE         13     // 保存波特率
#define  MI_CHECK_MODEM      14     // 查看模块型号
#define  MI_FIND_PIN         15     // 查看SIM卡PIN
#define  MI_GET_CSQ          16     // 获取信号强度
#define  MI_FIND_NETWORK     17     // 查询GSM网络状态
#define  MI_FIND_GPRS        18     // 查询GPRS网络状态
#define  MI_PDP_ONE          19     // 激活PDP场景
#define  MI_ACT_WLAN         20     // 激活无线场景
#define  MI_ASK_IP           21     // 请求IP
#define  MI_DEACT_PDP        22     // 关闭PDP场景
#define  MI_OPEN_DNS         23     // 打开DNS
#define  MI_FIND_SIMID       24     // 请求IMSI
#define  MI_OPEN_SERVER      25     // 连接服务器
#define  MI_CLOSE            26     // 关闭连接
#define  MI_NWSCANMODE       27     // 自动查询网络
#define  MI_TLS1       		 28     // 配置SSL版本
#define  MI_TLS2       		 29     // 配置加密套接字失败
#define  MI_TLS3       		 30     // 配置加密认证方式
#define  MI_TLS4       		 31     // 配置根证书的路径(受信任的CA证书的路径)
#define  MI_TLS5       		 32     // 为指定的SSL上下文配置客户端证书路径
#define  MI_TLS6       		 33     // 配置客户端密钥到密钥库
#define  MI_TLS7       		 34     // 配置忽略时间
#define  MI_CONFIG_GSM       35     // 自动查询网络
#define  MI_LOCATION       	 36     // 自动查询网络
#define  MI_CONFIG_AUTO      37     // 自动查询网络
#define  MI_CLOSE_SERVER     38     // 关闭连接
#define  MI_OPEN_SERVER      39     // 连接服务器
#define  MI_REGISTER      	 63     // 自动查询网络
#define  MI_RDY      	 	 64     // 读取初始化结果

#define  MI_ADD_HEAD         61     // 接收数据增加头
#define  MI_QIRDI            62     // 接收数据提示

#define  MI_REBOOT           40     // 模块重启
#define  MI_SEND_INFO        50     // 模块连接到平台后发送数采信息
#define  MI_INIT_OK          51     // 模块初始化完成
#define MI_SET_GPSOUTPORT	 52		//配置NMEA语句输出端口（GPS）
#define MI_OPERATE_GPS		 53		// 运行GPS会话

//====================================================================
#define CM_UART_USE           uart4           // 通讯模块使用的串口
//====================================================================

#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // 延时x秒
//====================================================================
#define CmPowerIoInit()        GPIO_SetDir(1,26,Output)   // 通讯模块电源控制IO口
#define CmPowerKeyIoInit()     GPIO_SetDir(1,4,Output)    // 通讯模块开关机控制IO口

#define CmPowerOn()            GPIO_SetBit(1,26,0)
#define CmPowerOff()           GPIO_SetBit(1,26,1)

#define CmPowerKeyHigh()       GPIO_SetBit(1,4,0)
#define CmPowerKeyLow()        GPIO_SetBit(1,4,1)
//====================================================================
// 环形缓冲区结构体
#define RING_LEN  524
typedef struct
{
    uint16_t uDataIn;         // 存入数据下标
    uint16_t uDataOut;        // 读取数据下标
    uint16_t uDataLen;        // 缓存数据数量
    uint8_t  uDataBuf[RING_LEN];   // 缓存数组
    uint16_t uDataSizeMax;    // 缓存数据大小
} RING_BUFFER_t;
//====================================================================
#define DATA_BUFFER_LEN   350
#define READ_MODEM_LEN    270  // 读模块长度270字节，缓冲区524字节。
static RING_BUFFER_t g_sRingData={0,0,0,{0},RING_LEN};   // 104报文环形缓冲区
static char g_uDataBuffer[DATA_BUFFER_LEN];         // 临时接收数据存储数组
static char g_uSendBuffer[256];                     // 临时发送数据存储数组

static uint32_t g_uTimerForSyncTime;  // 用于对时超时计时时标
static uint32_t g_uTimerForRecData;   // 用于长时间没有收到数据计时时标


uint8_t gWirelessMode = 0;

static IEC_FORMAT_T   g_sIecCmSend;     // 用于IEC104组帧
//====================================================================
//====================================================================
//====================================================================
/****************************************************************************
* 名    称：RingClear()
* 功    能：清空缓冲数据。
* 入口参数：
*           缓冲区指针
* 出口参数：出错返回 -1
*            正确返回 0
* 范    例: 无
****************************************************************************/
int8_t RingClear(RING_BUFFER_t *p)
{
    p->uDataIn = 0;
    p->uDataOut = 0;
    p->uDataLen = 0;

    return 0;
}
/****************************************************************************
* 名    称：RingNotEmpty
* 功    能：获取缓冲区是否有数据
* 入口参数：
            *p   缓存结构体指针
* 出口参数：空返回0，非空返回1
* 范    例: 无
****************************************************************************/
static uint8_t RingNotEmpty(RING_BUFFER_t *p)
{
    return (p->uDataLen?1:0);
}
/****************************************************************************
* 名    称：RingDataIn
* 功    能：写入数据到缓冲区
* 入口参数：
            *p      缓存结构体指针
            *data   数据指针
            bytes   数据数量
* 出口参数：写入的数据数量
* 范    例: 无
****************************************************************************/
static uint16_t RingDataIn(RING_BUFFER_t *p, const void *data,uint16_t bytes)
{
    uint16_t ret=0;
    uint8_t *p8 = (uint8_t *) data;

    while(0!=bytes)
    {
        if(p->uDataLen < p->uDataSizeMax)
        {
            // buf not full yet.
            p->uDataBuf[p->uDataIn++] = *p8;
            p->uDataIn %= p->uDataSizeMax;
            p->uDataLen++;

            bytes--;
            p8++;
            ret++;
        }
        else
        {
            return ret;
        }
    }

    return ret;
}
/****************************************************************************
* 名    称：RingDataOut
* 功    能：从缓冲区读取数据
* 入口参数：
            *p      缓存结构体指针
            *data   数据指针
            bytes   数据数量
* 出口参数：读取的数据数量
* 范    例: 无
****************************************************************************/
static uint16_t RingDataOut(RING_BUFFER_t *p, void *data,uint16_t bytes)
{
    uint16_t ret=0;
    uint8_t *p8 = (uint8_t *) data;

    while(0!=bytes)
    {
        if(p->uDataLen)
        {
            *p8 = p->uDataBuf[p->uDataOut++];
            p->uDataOut %= p->uDataSizeMax;

            p->uDataLen--;

            bytes--;
            p8++;
            ret++;
        }
        else
        {
            return ret;
        }
    }

    return ret;
}
/****************************************************************************
* 名    称：RingDataShow
* 功    能：显示缓冲区数据
* 入口参数：
            *p      缓存结构体指针
            *data   数据指针
            bytes   数据数量
* 出口参数：显示的数据数量
* 范    例: 无
****************************************************************************/
static uint16_t RingDataShow(RING_BUFFER_t *p, void *data,uint16_t bytes)
{
    uint16_t ret=0;
    uint16_t uDataOut;
    uint16_t uDataLen;
    uint8_t *p8 = (uint8_t *) data;

    uDataOut = p->uDataOut;
    uDataLen = p->uDataLen;

    while(0!=bytes)
    {
        if(uDataLen)
        {
            *p8 = p->uDataBuf[uDataOut++];
            uDataOut %= p->uDataSizeMax;

            uDataLen--;

            bytes--;
            p8++;
            ret++;
        }
        else
        {
            return ret;
        }
    }

    return ret;
}
/****************************************************************************
* 名    称：RingDataLen
* 功    能：获取缓冲区数据数量
* 入口参数：
            *p      缓存结构体指针
* 出口参数：缓冲区数据数量
* 范    例: 无
****************************************************************************/
static uint16_t RingDataLen(RING_BUFFER_t *p)
{
    return p->uDataLen;
}
/****************************************************************************
* 名    称：RingDataThrow
* 功    能：丢弃缓冲区数据
* 入口参数：
            *p       缓存结构体指针
             bytes   数据数量
* 出口参数：丢弃数据数量
* 范    例: 无
****************************************************************************/
static uint16_t RingDataThrow(RING_BUFFER_t *p,uint16_t bytes)
{
    uint16_t ret=0;
    while(0!=bytes)
    {
        if(p->uDataLen)
        {
            p->uDataOut++;
            p->uDataOut %= p->uDataSizeMax;

            p->uDataLen--;

            bytes--;

            ret++;
        }
        else
        {
            return ret;
        }
    }
    return ret;
}
//====================================================================
//====================================================================
//====================================================================
//====================================================================
/****************************************************************************
* 名    称：ReadModem()
* 功    能：读模块数据
* 入口参数：
            uLen：数量
* 出口参数：
* 范    例: 无
****************************************************************************/
int16_t ReadModem(uint16_t uLen)
{
    int16_t iLen;

    iLen = UartRead(CM_UART_USE, g_uDataBuffer, uLen, 3);

    if(iLen>0)
    {
        DEBUGOUT("<CM>:%s\n",g_uDataBuffer);
    }

    return iLen;
}
/****************************************************************************
* 名    称：WriteModem()
* 功    能：发送数据到模块
* 入口参数：
            pData：数据指针
            uLen： 数量
* 出口参数：
* 范    例: 无
****************************************************************************/
int16_t WriteModem(const void* pData,uint16_t uLen)
{
    int16_t iResult;
    iResult = UartWrite(CM_UART_USE,pData,uLen);

    return iResult;
}
/****************************************************************************
* 名    称：JudgeFeedback()
* 功    能：判断模块返回的数据
* 入口参数：
            Timeout：超时时间
            pTarget：目标字符串
* 出口参数：0：成功；1：超时；2：错误
* 范    例: 无
****************************************************************************/
int8_t JudgeFeedback(uint32_t Timeout,const char *pTarget)
{
    int16_t iDataLen;

    uint32_t uStartTime;      // 开始的时间戳
    uint32_t uNowTime;        // 现在的时间戳

    uStartTime = OSTimeGet();
    memset(g_uDataBuffer,0,256);

    do
    {
        iDataLen = UartRxLen(CM_UART_USE);
        if(iDataLen<=0)
        {
            uNowTime = OSTimeGet();
            if(NOTIMEOUT == TimeOut(uStartTime,uNowTime,Timeout))  // 没有超时
            {
                msleep(40);
            }
            else
            {
                return MI_JUDGE_TIMEOUT;
            }
        }
        else
        {
            iDataLen = ReadModem(256);
            if(NULL!=strstr(g_uDataBuffer,pTarget))
            {
                return MI_JUDGE_OK;
            }
            else
            {
                return MI_JUDGE_ERR;
            }
        }
    }
    while(1);
}
/******************************************************************************
* 名    称：ReportSIMID()
* 功    能：上报SIM卡ID。
* 入口参数：
            *pA         IEC信息指针
*
* 出口参数：无
* 范    例:
******************************************************************************/
void ReportSIMID(void)
{
    g_sIecCmSend.format.maddrL = 0x00;
    g_sIecCmSend.format.maddrM = 0x00;
    g_sIecCmSend.format.maddrH = 0x00;

    //strncpy(pA->send.format.data,Logger_run.IMSI,15);
    memcpy(g_sIecCmSend.format.data,g_LoggerRun.IMSI,15);
    g_sIecCmSend.format.data[15] = 0x00;
    g_sIecCmSend.format.data[16] = 0x00;
    g_sIecCmSend.format.data[17] = 0x00;
    g_sIecCmSend.format.data[18] = 0x00;
    g_sIecCmSend.format.data[19] = 0x00;

    IecCreateFrameI(P_SIM_ID,1,R_INFO_REPORT,20,&g_sIecCmSend);
}
/****************************************************************************
* 名    称：GetCsq()
* 功    能：解析出无线功率并上报
* 入口参数：
            Timeout：超时时间
            pTarget：目标字符串
* 出口参数：0：成功；1：超时；2：错误
* 范    例: 无
****************************************************************************/
uint8_t GetCsq(const char* cCsqData,uint8_t uCmd)
{
    char *pRec;
    uint8_t i;
    uint8_t uCsq=0;

    pRec = strstr(cCsqData,"+CSQ");

    if(NULL!=pRec)  // 模块有收到数据提示
    {
        pRec += 5;

        for(i=0;i<5;i++)
        {
            if(','==*pRec)
            {
                break;
            }
            else if(*pRec>='0' && *pRec<='9')
            {
                uCsq = uCsq*10 + *pRec-'0';
            }
            pRec++;
        }
        DEBUGOUT("无线功率:%d\n",uCsq);
        g_LoggerRun.uCSQ = uCsq;

        if(99!=uCsq)
        {
            if(uCmd)
            {
                g_sIecCmSend.format.maddrL = 0x00;
                g_sIecCmSend.format.maddrM = 0x00;
                g_sIecCmSend.format.maddrH = 0x00;
                g_sIecCmSend.format.data[0] = uCsq;

                IecCreateFrameI(P_WL_QUALITY,0x01,R_INFO_REPORT,1,&g_sIecCmSend);
            }

        }
    }
    return uCsq;
}
/****************************************************************************
* 名    称：CmInit()
* 功    能：通讯模块初始化
* 入口参数：
            uCmInitStep：模块初始化步骤
* 出口参数：1：初始化完成；0：初始化中
* 范    例: 无
****************************************************************************/
#define CM_INIT_OK      0
#define CM_INIT_ING     1
int8_t CmInit(uint8_t uCmInitStep)
{
    static uint8_t s_uStep=MI_START;  // 模块初始化步骤
    static uint8_t s_uRound=0;        // 单步执行次数
    static uint8_t s_uRinitCount=0;   // 模块复位次数
    static uint8_t s_uReOpen=0;       // 重新打开连接服务器次数

    uint8_t uMIFeedback;

   // char   strtemp[7]={'\0'};

    int16_t iDataLen;

    if(uCmInitStep)
    {
        s_uStep = uCmInitStep;
    }

    if(gWirelessNetMode == 0)
    {
    	switch(s_uStep)
		{
		case MI_START:
			sleep(1);
			DEBUGOUT("模块供电-%d\n",s_uRinitCount);
			CmPowerOn();
			CmPowerKeyHigh();

			DEBUGOUT("模块开机\n");

			sleep(1);
			CmPowerKeyLow();
			sleep(2);
			CmPowerKeyHigh();

			UartClear(CM_UART_USE);  // 清空一下串口缓冲区

			sleep(1);

			s_uStep = MI_AT;
			break;

		case MI_AT:
			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("AT-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT\r\n",4);
			iDataLen = 4;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");

			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(500);
				s_uStep = MI_NO_ECHO;
				g_LoggerRun.north_status = NORTH_RDY;
			}
			break;

		case MI_NO_ECHO:

			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("关回显-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"ATE0\r\n",6);
			iDataLen = 6;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(500);
				s_uStep = MI_CHECK_MODEM;
			}
			break;

		case MI_BAUDRATE:
			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("模块波特率-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+IPR=115200&W\r\n",17);
			iDataLen = 17;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(1000,"OK");

			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_FIND_PIN;
			}
			break;

		case MI_CHECK_MODEM:
			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("检测模块-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+GMR\r\n",8);
			iDataLen = 8;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(6000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_BAUDRATE;
			}
			break;

		case MI_FIND_PIN:
			if(s_uRound>10)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("查SIM-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CPIN?\r\n",10);
			iDataLen = 10;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(10000,"READY");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_FIND_NETWORK;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_FIND_NETWORK:
			if(s_uRound>30)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			sleep(3);
			DEBUGOUT("找网络-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CREG?\r\n",10);
			iDataLen = 10;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(60000,"0,");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				if(NULL!=strstr(g_uDataBuffer,"0,1") || NULL!=strstr(g_uDataBuffer,"0,5"))
				{
					s_uRound = 0;
					msleep(300);
					s_uStep = MI_FIND_GPRS;
				}
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_FIND_GPRS:
			if(s_uRound>30)
			{
				s_uRound = 0;
				s_uStep = MI_PDP_ONE;
				break;
			}
			sleep(3);
			DEBUGOUT("找GPRS-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CGREG?\r\n",11);
			iDataLen = 11;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(60000,"0,");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				if(NULL!=strstr(g_uDataBuffer,"0,1") || NULL!=strstr(g_uDataBuffer,"0,5"))
				{
					s_uRound = 0;
					msleep(300);
					s_uStep = MI_GET_CSQ;
				}
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_GET_CSQ:
			if(s_uRound>10)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("信号强度-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CSQ\r\n",8);
			iDataLen = 8;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(60000,"+CSQ");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				GetCsq(g_uDataBuffer,0);
				msleep(300);
				s_uStep = MI_PDP_ONE;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_PDP_ONE:
			if(s_uRound>10)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
				break;
			}
			DEBUGOUT("激活PDP-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QIREGAPP\r\n",13);
			iDataLen = 13;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(180000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_ACT_WLAN;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_ACT_WLAN:
			if(s_uRound>3)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("激活无线场景-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QIACT\r\n",10);
			iDataLen = 10;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(180000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_ASK_IP;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_ASK_IP:
			if(s_uRound>3)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
				break;
			}
			DEBUGOUT("请求IP-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QILOCIP\r\n",12);
			iDataLen = 12;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(180000,".");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_OPEN_DNS;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_DEACT_PDP:
			if(s_uRound>10)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("关闭PDP-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QIDEACT\r\n",12);
			iDataLen = 12;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(90000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_FIND_PIN;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_OPEN_DNS:
			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("打开DNS-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QIDNSIP=1\r\n",14);
			iDataLen = 14;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(90000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_FIND_SIMID;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_FIND_SIMID:
			if(s_uRound>10)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
				break;
			}
			DEBUGOUT("请求IMSI-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CIMI\r\n",9);
			iDataLen = 9;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(90000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				strncpy(g_LoggerRun.IMSI,&g_uDataBuffer[2],15);
				DEBUGOUT("IMSI:%-15.15s\r\n",g_LoggerRun.IMSI);
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_QIRDI;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_QIRDI:
			if(s_uRound>3)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
				break;
			}
			DEBUGOUT("接收提示-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QINDI=1\r\n",12);
			iDataLen = 12;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(400,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_OPEN_SERVER;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_OPEN_SERVER:
			if(s_uRound>5)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
				break;
			}
			s_uReOpen++;
			if(s_uReOpen>5)
			{
				s_uRound = 0;
				s_uReOpen = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			g_LoggerRun.north_status = NORTH_DISCON;

			DEBUGOUT("连服务器-%d\n",s_uRound);

			memset(g_uSendBuffer,0,256);
			snprintf(g_uSendBuffer,sizeof(g_uSendBuffer),"AT+QIOPEN=\"TCP\",%s,%d\r\n",g_LoggerInfo.server_domain,g_LoggerInfo.server_port);

			WriteModem(g_uSendBuffer,strlen(g_uSendBuffer));
			DEBUGOUT("%s\n",g_uSendBuffer);
			s_uRound++;

			uMIFeedback = JudgeFeedback(90000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
				break;
			}
			else
			{
				sleep(1);
				break;
			}
			uMIFeedback = JudgeFeedback(90000,"CONNECT OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				s_uRinitCount = 0;
				s_uStep = MI_SEND_INFO;
				g_LoggerRun.north_status = NORTH_CONNECT;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
			}
			else
			{
				if(NULL!=strstr(g_uDataBuffer,"ALREADY CONNECT"))  // 已经连接
				{
					s_uRound = 0;
					s_uRinitCount = 0;
					s_uStep = MI_SEND_INFO;
					g_LoggerRun.north_status = NORTH_CONNECT;
				}
				sleep(3);
			}
			break;

		case MI_CLOSE:
			if(s_uRound>3)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("关连接-%d\n",s_uRound);

			g_LoggerRun.north_status = NORTH_DISCON;

			strncpy(g_uSendBuffer,"AT+QICLOSE\r\n",12);
			iDataLen = 12;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(90000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_OPEN_SERVER;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_REBOOT:

			DEBUGOUT("模块关机\n");
			CmPowerKeyLow();
			sleep(1);
			CmPowerKeyHigh();

			g_LoggerRun.north_status = NORTH_POWERDOWN;

			sleep(13);
			DEBUGOUT("模块断电-%d\n",s_uRinitCount);
			CmPowerOff();

			s_uRinitCount++;

			if(s_uRinitCount<2)
			{
				DEBUGOUT("模块初始化失败%d次；重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,0,10,0);
			}
			else if(2==s_uRinitCount)
			{
				DEBUGOUT("模块初始化失败%d次；5分钟后重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,5,0,0);
			}
			else if(3==s_uRinitCount)
			{
				DEBUGOUT("模块初始化失败%d次；10分钟后重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,10,0,0);
			}
			else if(s_uRinitCount>=4 && s_uRinitCount<=18)
			{
				DEBUGOUT("模块初始化失败%d次；15分钟后重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,15,0,0);
			}
			else
			{
				//sleep(5);
				DEBUGOUT("与平台断链4小时后，Reboot!!!\n");
				Reboot();
			}

			s_uStep = MI_START;
			break;

		case MI_SEND_INFO:
			s_uReOpen = 0;

			CmDelSendLink();                          // 清空发送链表
			CmDelRecLink();                           // 清空接收链表
			RingClear(&g_sRingData);                  // 清空缓冲
			DEBUGOUT("上报数采信息\n");
			IecReportLogInfo(R_INFO_REPORT);          // 上报数采信息
			ReportSIMID();                            // 上报SIM卡ID
			g_uTimerForSyncTime = OSTimeGet();        // 上报数采信息后记录时标用于对时超时比较
			g_uTimerForRecData = g_uTimerForSyncTime; // 长时间没有接收到数据时标
			s_uStep = MI_INIT_OK;
			break;

		case MI_INIT_OK:
			return CM_INIT_OK;
			break;

		default:
			s_uStep = MI_START;
			s_uRound = 0;
			DEBUGOUT("模块异常断电\n");
			CmPowerOff();    // 不供电
			break;
		}
    }

    /*******************************EC200S Configuration***********************/
    if(gWirelessNetMode == 1)
	{
		switch(s_uStep)
		{
		case MI_START:
			sleep(1);
			DEBUGOUT("模块供电-%d\n",s_uRinitCount);
			CmPowerOn();
			CmPowerKeyHigh();

			DEBUGOUT("模块开机\n");

			sleep(1);
			CmPowerKeyLow();
			sleep(2);
			CmPowerKeyHigh();

			UartClear(CM_UART_USE);  // 清空一下串口缓冲区

			sleep(1);

			s_uStep = MI_RDY;
			break;

		case MI_RDY:
			if(s_uRound>1)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			s_uRound++;
			uMIFeedback = JudgeFeedback(18000,"RDY");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(500);
				s_uStep = MI_AT;
			}
			break;

		case MI_AT:

			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("AT-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT\r\n",4);
			iDataLen = 4;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");

			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(500);
				s_uStep = MI_NO_ECHO;
				g_LoggerRun.north_status = NORTH_RDY;
			}
			break;

		case MI_NO_ECHO:

			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("关回显-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"ATE0\r\n",6);
			iDataLen = 6;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(500);
				s_uStep = MI_CHECK_MODEM;
			}
			break;

		case MI_CHECK_MODEM:
			if(s_uRound>15)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("检测模块-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+GMR\r\n",8);
			iDataLen = 8;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(6000,"OK");

			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_FIND_PIN;
			}
			break;

		case MI_FIND_PIN:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("查SIM-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CPIN?\r\n",10);
			iDataLen = 10;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(600,"READY");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_GET_CSQ;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_GET_CSQ:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("信号强度-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CSQ\r\n",8);
			iDataLen = 8;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(60000,"+CSQ");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				GetCsq(g_uDataBuffer,0);
				msleep(300);
				s_uStep = MI_FIND_NETWORK;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_FIND_NETWORK:
			if(s_uRound>29)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("找网络-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CREG?\r\n",10);
			iDataLen = 10;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(60000,"0,");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				if(NULL!=strstr(g_uDataBuffer,"0,1") || NULL!=strstr(g_uDataBuffer,"0,5"))
				{
					s_uRound = 0;
					msleep(300);
					s_uStep = MI_FIND_GPRS;
				}
				else
				{
					sleep(1);
				}
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				sleep(1);
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_FIND_GPRS:
			if(s_uRound>29)
			{
				s_uRound = 0;
				s_uStep = MI_PDP_ONE;
				break;
			}
			DEBUGOUT("找GPRS-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+CGREG?\r\n",11);
			iDataLen = 11;
			WriteModem(g_uSendBuffer,iDataLen);
			s_uRound++;

			uMIFeedback = JudgeFeedback(60000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				if(NULL!=strstr(g_uDataBuffer,"0,1") || NULL!=strstr(g_uDataBuffer,"0,5"))
				{
					s_uRound = 0;
					msleep(300);
					s_uStep = MI_REGISTER;
				}
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_REGISTER:
			if(s_uRound>5)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			strncpy(g_uSendBuffer,"AT+QICSGP=1,1,\"UNINET\",\"\",\"\",1\r\n",sizeof("AT+QICSGP=1,1,\"UNINET\",\"\",\"\",1\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QICSGP=1,1,\"UNINET\",\"\",\"\",1\r\n"));
			printf("AT+QICSGP=1,1,\"UNINET\",\"\",\"\",1\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(9000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_DEACT_PDP;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_DEACT_PDP:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("关闭PDP-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QIDEACT=1\r\n",sizeof("AT+QIDEACT=1\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QIDEACT=1\r\n"));
			s_uRound++;

			uMIFeedback = JudgeFeedback(90000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_ACT_WLAN;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_ACT_WLAN:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("激活无线场景-%d\n",s_uRound);

			strncpy(g_uSendBuffer,"AT+QIACT=1\r\n",sizeof("AT+QIACT=1\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QIACT=1\r\n"));
			printf("AT+QIACT=1\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(180000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_TLS1;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_TLS1:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QSSLCFG=\"sslversion\",0,1\r\n",sizeof("AT+QSSLCFG=\"sslversion\",0,1\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QSSLCFG=\"sslversion\",0,1\r\n"));
			printf("AT+QSSLCFG=\"sslversion\",0,1\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_TLS2;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_TLS2:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QSSLCFG=\"ciphersuite\",0,0xFFFF\r\n",sizeof("AT+QSSLCFG=\"ciphersuite\",0,0xFFFF\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QSSLCFG=\"ciphersuite\",0,0xFFFF\r\n"));
			printf("AT+QSSLCFG=\"ciphersuite\",0,0xFFFF\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_TLS3;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_TLS3:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QSSLCFG=\"cacert\",0,\"UFS:ca-cert.pem\"\r\n",sizeof("AT+QSSLCFG=\"cacert\",0,\"UFS:ca-cert.pem\"\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QSSLCFG=\"cacert\",0,\"UFS:ca-cert.pem\"\r\n"));
			printf("AT+QSSLCFG=\"cacert\",0,\"UFS:ca-cert.pem\"\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_TLS4;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_TLS4:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QSSLCFG=\"clientcert\",0,\"UFS:client.pem\"\r\n",sizeof("AT+QSSLCFG=\"clientcert\",0,\"UFS:client.pem\"\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QSSLCFG=\"clientcert\",0,\"UFS:client.pem\"\r\n"));
			printf("AT+QSSLCFG=\"clientcert\",0,\"UFS:client.pem\"\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_TLS5;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_TLS5:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QSSLCFG=\"clientkey\",0,\"UFS:ckey.pem\"\r\n",sizeof("AT+QSSLCFG=\"clientkey\",0,\"UFS:ckey.pem\"\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QSSLCFG=\"clientkey\",0,\"UFS:ckey.pem\"\r\n"));
			printf("AT+QSSLCFG=\"clientkey\",0,\"UFS:ckey.pem\"\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_TLS6;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_TLS6:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QSSLCFG=\"seclevel\",0,0\r\n",sizeof("AT+QSSLCFG=\"seclevel\",0,0\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QSSLCFG=\"seclevel\",0,0\r\n"));
			printf("AT+QSSLCFG=\"seclevel\",0,0\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_TLS7;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_TLS7:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QSSLCFG=\"ignorelocaltime\",1\r\n",sizeof("AT+QSSLCFG=\"ignorelocaltime\",1\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QSSLCFG=\"ignorelocaltime\",1\r\n"));
			printf("AT+QSSLCFG=\"ignorelocaltime\",1\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_NWSCANMODE;
			}
			else if(MI_JUDGE_TIMEOUT==uMIFeedback)
			{
				s_uRound = 0;
				s_uStep = MI_NWSCANMODE;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_NWSCANMODE:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			strncpy(g_uSendBuffer,"at+qcfg=\"nwscanmode\",0\r\n",sizeof("at+qcfg=\"nwscanmode\",0\r\n"));
			WriteModem(g_uSendBuffer,sizeof("at+qcfg=\"nwscanmode\",0\r\n"));
			printf("at+qcfg=\"nwscanmode\",0\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(180000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_FIND_SIMID;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_FIND_SIMID:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			strncpy(g_uSendBuffer,"AT+CIMI\r\n",sizeof("AT+CIMI\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+CIMI\r\n"));
			printf("AT+CIMI\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(100,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_CLOSE_SERVER;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_CLOSE_SERVER:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}

			strncpy(g_uSendBuffer,"AT+QICLOSE=0\r\n",sizeof("AT+QICLOSE=0\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QICLOSE=0\r\n"));
			printf("AT+QICLOSE=0\r\n");
			s_uRound++;

			uMIFeedback = JudgeFeedback(500,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_OPEN_SERVER;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_OPEN_SERVER:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_DEACT_PDP;
				break;
			}

			if(s_uReOpen>10)
			{
				s_uRound = 0;
				s_uReOpen = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			s_uReOpen++;
			g_LoggerRun.north_status = NORTH_DISCON;
			DEBUGOUT("连服务器-%d\n",s_uReOpen);
			memset(g_uSendBuffer,0,256);
			snprintf(g_uSendBuffer,sizeof(g_uSendBuffer),"AT+QIOPEN=1,0,\"TCP\",%s,%d,0,0\r\n",g_LoggerInfo.server_domain,g_LoggerInfo.server_port);
			sleep(1);
			WriteModem(g_uSendBuffer,strlen(g_uSendBuffer));
			DEBUGOUT("%s\n",g_uSendBuffer);
			s_uRound++;
			sleep(3);
			uMIFeedback = JudgeFeedback(90000,"0,0");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				s_uRinitCount = 0;
				s_uStep = MI_SEND_INFO;
				g_LoggerRun.north_status = NORTH_CONNECT;
			}
			break;

		case MI_CLOSE:
			if(s_uRound>4)
			{
				s_uRound = 0;
				s_uStep = MI_REBOOT;
				break;
			}
			DEBUGOUT("关连接-%d\n",s_uRound);

			g_LoggerRun.north_status = NORTH_DISCON;

			strncpy(g_uSendBuffer,"AT+QICLOSE=0\r\n",sizeof("AT+QICLOSE=0\r\n"));
			WriteModem(g_uSendBuffer,sizeof("AT+QICLOSE=0\r\n"));
			s_uRound++;

			uMIFeedback = JudgeFeedback(90000,"OK");
			if(MI_JUDGE_OK==uMIFeedback)
			{
				s_uRound = 0;
				msleep(300);
				s_uStep = MI_START;
			}
			else
			{
				sleep(1);
			}
			break;

		case MI_REBOOT:
			DEBUGOUT("模块关机\n");
			CmPowerKeyLow();
			sleep(1);
			CmPowerKeyHigh();

			g_LoggerRun.north_status = NORTH_POWERDOWN;

			sleep(3);
			DEBUGOUT("模块断电-%d\n",s_uRinitCount);
			CmPowerOff();

			s_uRinitCount++;

			if(s_uRinitCount<2)
			{
				DEBUGOUT("模块初始化失败%d次；重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,0,10,0);
			}
			else if(2==s_uRinitCount)
			{
				DEBUGOUT("模块初始化失败%d次；5分钟后重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,5,0,0);
			}
			else if(3==s_uRinitCount)
			{
				DEBUGOUT("模块初始化失败%d次；10分钟后重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,10,0,0);
			}
			else if(s_uRinitCount>=4 && s_uRinitCount<=18)
			{
				DEBUGOUT("模块初始化失败%d次；15分钟后重新初始化\r\n", s_uRinitCount);
				OSTimeDlyHMSM(0,15,0,0);
			}
			else
			{
				//sleep(5);
				DEBUGOUT("与平台断链4小时后，Reboot!!!\n");
				Reboot();
			}

			s_uStep = MI_START;
			break;

		case MI_SEND_INFO:
			s_uReOpen = 0;
			CmDelSendLink();                          // 清空发送链表
			CmDelRecLink();                           // 清空接收链表
			RingClear(&g_sRingData);                  // 清空缓冲
			DEBUGOUT("上报数采信息\n");
			IecReportLogInfo(R_INFO_REPORT);          // 上报数采信息
			ReportSIMID();                            // 上报SIM卡ID
			g_uTimerForSyncTime = OSTimeGet();        // 上报数采信息后记录时标用于对时超时比较
			g_uTimerForRecData = g_uTimerForSyncTime; // 长时间没有接收到数据时标
			s_uStep = MI_INIT_OK;
			break;

		case MI_INIT_OK:
			return CM_INIT_OK;
			break;

		default:
			s_uStep = MI_START;
			s_uRound = 0;
			DEBUGOUT("模块异常断电\n");
			CmPowerOff();    // 不供电
			break;
		}
    }

    return s_uStep;
}

//======================================================================================
#define W_WAIT_DATA     0  // 等待数据产生
#define W_SEND_AT       1  // 发送发送数据AT指令
#define W_WAIT_SIGN     2  // 等待“>”
#define W_SEND_DATA     3  // 发送数据
#define W_WAIT_OK       4  // 等待“SEND OK”
#define W_PAUSE         5  // 暂停发送数据
#define W_WAIT_TIME     6  // 等待暂停超时

#define R_WAIT_TIME     0   // 等待超时主动发起查询数据
#define R_WAIT_COM      1   // 等待串口资源
#define R_SEND_AT       2   // 发送AT指令读取数据
#define R_WAIT_DATA     3   // 等待数据
#define R_PROCESS_DATA  4   // 处理数据
#define R_RESTART       5   // 重新开始

OS_STK TaskModemProcessStk[TaskModemProcessStkSize]@0x20000000;	      // 定义任务堆栈大小，指定内存地址为@0x20000000
/****************************************************************************
* 名    称：TaskModemProcess()
* 功    能：通讯模块任务
* 入口参数：

* 出口参数：无
* 范    例: 无
****************************************************************************/
void TaskModemProcess(void *p)
{
    p = p;
    static uint8_t uCmInitStep=0;
    static uint8_t s_uMainInit=CM_INIT_ING;   // 主通道初始化状态

    int16_t iRecLen;
    char *pRec;
    uint16_t i;
    uint16_t uDataLen;

    static uint8_t s_uWriteStep=0;      // 写模块状态步骤
    static uint8_t s_uReadStep=0;       // 读模块状态步骤
    static uint32_t s_uWriteTimeMark;   // 写模块时标
    static uint32_t s_uReadTimeMark;    // 读模块时标
    static uint32_t s_uTimeForCsq;      // 查询信号强度时标

    uint32_t uNowTimemark;

    CmPowerIoInit();                     // 模块电源控制IO口初始化
    CmPowerKeyIoInit();                  // 模块开关机控制IO口初始化

    s_uWriteTimeMark = OSTimeGet();      // 获取当前时标
    s_uReadTimeMark  = s_uWriteTimeMark;
    s_uTimeForCsq    = s_uReadTimeMark;

    // 模块数据收发链表初始化
    CmLinkInit();

    while(1)
    {
		//FEED_DOG(); 	// 喂狗
		if(0x0A==g_LoggerRun.update)
        {
            sleep(10);
            continue;
        }

        if(s_uMainInit)
        {
            s_uMainInit = CmInit(uCmInitStep);  // 模块初始化
            uCmInitStep = 0;

            s_uWriteStep = W_WAIT_DATA;
            s_uReadStep  = R_WAIT_TIME;
            continue;
        }

        uNowTimemark = OSTimeGet();  // 更新当下时间时标

        // 初步处理模块数据
        if(UartRxLen(CM_UART_USE)<=0) // 收取模块发送给芯片的数据
        {
        	// 对时超时保护
            if(NORTH_OK!=g_LoggerRun.north_status)
            {
                if(TIMEOUT == TimeOut(g_uTimerForSyncTime,uNowTimemark,60000))  // 超时未收到对时报文@60秒
                {
                    uCmInitStep = MI_CLOSE;      // 模块关闭连接
                    s_uMainInit = CM_INIT_ING;   // 进入模块初始化函数
                    DEBUGOUT("未对时\n");
                    continue;
                }
            }

            // 心跳超时保护
            if(TIMEOUT == TimeOut(g_uTimerForRecData,uNowTimemark,100000))    // 超时未收到数据报文@100秒
            {
                uCmInitStep = MI_CLOSE;      // 模块关闭连接
                s_uMainInit = CM_INIT_ING;   // 进入模块初始化函数
                DEBUGOUT("无心跳\n");
                continue;
            }

            // 定期查询信号强度
            if(NORTH_OK==g_LoggerRun.north_status)
            {
                if(W_WAIT_DATA==s_uWriteStep && R_WAIT_TIME==s_uReadStep && 0==g_LoggerRun.update)  // 模块读写都是空闲的时候 且 不在升级过程
                {
                    if(TIMEOUT == TimeOut(s_uTimeForCsq,uNowTimemark,60000))       // 每分钟查询信号强度报文@60秒
                    {
                        WriteModem("AT+CSQ\r\n",8);   // 查询模块信号强度
                        s_uTimeForCsq = uNowTimemark; // 更新时标
                        s_uWriteStep = W_PAUSE;       // 暂停发送数据
                        s_uReadStep  = R_RESTART;     // 重置读模块计时
                    }
                }
            }
            else
            {
                s_uTimeForCsq = uNowTimemark; // 更新时标
            }

			FEED_DOG();
            msleep(10);
            
            LedTwinkle1();
        }
        else
        {
        	FEED_DOG();
			memset(g_uDataBuffer,0,DATA_BUFFER_LEN);
            iRecLen = UartRead(CM_UART_USE, g_uDataBuffer, DATA_BUFFER_LEN, 3);

            if(iRecLen<=0)
            {
                continue;
            }
            DEBUGOUT("CM %d:%s\n",iRecLen,g_uDataBuffer);

            if(strstr(g_uDataBuffer,">"))  // 发送数据有收到“>”
            {
                if(W_WAIT_SIGN==s_uWriteStep)
                {
                    s_uWriteStep = W_SEND_DATA;
                }
            }

            if(strstr(g_uDataBuffer,"SEND OK"))  // 发送数据有收到“SEND OK”
            {
                if(W_WAIT_OK==s_uWriteStep)
                {
                    s_uWriteStep = W_WAIT_DATA;
                }
            }
            else if(strstr(g_uDataBuffer,"OK"))  // 主动查询模块数据，返回OK。重新开始
            {
                if(R_WAIT_DATA==s_uReadStep)
                {
                    s_uReadStep = R_RESTART;
                }
            }

            if(strstr(g_uDataBuffer,"+QIRDI"))  // 模块有收到数据提示
            {
                s_uReadStep = R_WAIT_COM;
            }
            else if(strstr(g_uDataBuffer,"+QIRD"))   // 收到模块推送的数据
            {
                //if(R_WAIT_DATA==s_uReadStep)
                {
                    s_uReadStep = R_PROCESS_DATA;
                }
            }

            if(strstr(g_uDataBuffer,"+CSQ"))  // 模块接收到信号强度
            {
                GetCsq(g_uDataBuffer,1);
            }

            if(strstr(g_uDataBuffer,"CLOSED"))  // 模块推送“close”
            {
                uCmInitStep = MI_OPEN_SERVER;
                s_uMainInit = CM_INIT_ING;   // 进入模块初始化函数
                continue;
            }

            if(strstr(g_uDataBuffer,"+PDP DEACT"))
            {
                uCmInitStep = MI_PDP_ONE;
                s_uMainInit = CM_INIT_ING;   // 进入模块初始化函数
            }
        }
        
        // 处理模块数据发送
        switch(s_uWriteStep)
        {
        case W_WAIT_DATA:
            uDataLen = CmSendLinkLen();
            if(uDataLen && R_WAIT_TIME==s_uReadStep)  // 链表有数据 且 不再读模块数据过程中
            {
                s_uWriteStep = W_SEND_AT;
            }
            break;
        case W_SEND_AT:
            memset(g_uSendBuffer,0,256);

            (gWirelessNetMode == 1)?strncpy(g_uSendBuffer,"AT+QISEND=0,",12):strncpy(g_uSendBuffer,"AT+QISEND=",10);
            I2Str(uDataLen,&g_uSendBuffer[((gWirelessNetMode == 1)?12:10)]);
            strcat(g_uSendBuffer,"\r\n");
            WriteModem(g_uSendBuffer,strlen(g_uSendBuffer));

            DEBUGOUT("\r\n--\r\n");
            s_uWriteTimeMark = uNowTimemark;
            s_uWriteStep = W_WAIT_SIGN;
            break;

        case W_WAIT_SIGN:// 等待">"
            if(TIMEOUT == TimeOut(s_uWriteTimeMark,uNowTimemark,500))  // 超时
            {
                DEBUGOUT("无>\n");
                s_uWriteStep = W_SEND_DATA;    // 收不到>继续发送数据
            }
            break;
        case W_SEND_DATA:
            memset(g_uSendBuffer,0,256);
            uDataLen = CmReadSendLink((uint8_t *)g_uSendBuffer); // 读数据
            WriteModem(g_uSendBuffer,uDataLen);
            if(1 == (g_uSendBuffer[2] & 0x03))
            {
                DEBUGOUT("SendS:");
            }
            else if(3 == (g_uSendBuffer[2] & 0x03))
            {
                DEBUGOUT("SendU:");
            }
            else
            {
                DEBUGOUT("SendI:");
            }

            for(i=0; i<uDataLen; i++)
            {
                DEBUGOUT("%02X ",g_uSendBuffer[i]);
            }
            DEBUGOUT("\n");

            s_uWriteTimeMark = uNowTimemark;
            s_uWriteStep = W_WAIT_OK;
            break;
        case W_WAIT_OK: // 等待“SEND OK”
            if(TIMEOUT == TimeOut(s_uWriteTimeMark,uNowTimemark,500))  // 超时
            {
                DEBUGOUT("无SEND OK,重连\n");
                uCmInitStep = MI_CLOSE;  // 模块关闭连接
                s_uMainInit = CM_INIT_ING;   // 进入模块初始化函数
            }
            break;

        case W_PAUSE:
            s_uWriteTimeMark = uNowTimemark;
            s_uWriteStep = W_WAIT_TIME;
            break;

        case W_WAIT_TIME:
            if(TIMEOUT == TimeOut(s_uWriteTimeMark,uNowTimemark,500))  // 暂停500ms
            {
                s_uWriteStep = W_WAIT_DATA;
            }
            break;

        default:
            s_uWriteStep = W_WAIT_DATA;
            break;
        }

        // 处理模块数据接收
        switch(s_uReadStep)
        {
        case R_WAIT_TIME:
//        	DEBUGOUT("R_WAIT_TIME\r\n");
            if(TIMEOUT == TimeOut(s_uReadTimeMark,uNowTimemark,(gWirelessNetMode == 1)?500:3000))   // 以前是10秒//现在接收空闲时间5S
            {
                s_uReadStep = R_WAIT_COM;
            }
            break;
        case R_WAIT_COM:
            if(W_WAIT_DATA==s_uWriteStep)  // 发送数据没有占用模块资源
            {
                s_uReadStep = R_SEND_AT;
            }
            break;
        case R_SEND_AT:
            DEBUGOUT("读模块\n");
            (gWirelessNetMode == 1)?WriteModem("AT+QIRD=0,270\r\n",strlen("AT+QIRD=0,270\r\n")):WriteModem("AT+QIRD=0,1,0,270\r\n",19);
            s_uReadTimeMark  = uNowTimemark;
            s_uReadStep = R_WAIT_DATA;
            break;
        case R_WAIT_DATA: // 等待数据
            if(TIMEOUT == TimeOut(s_uReadTimeMark,uNowTimemark,400))  // 超时
            {
                DEBUGOUT("读模块超时\n");
                s_uReadStep = R_RESTART;
            }
            break;

        case R_PROCESS_DATA:
        	pRec = (gWirelessNetMode == 1)?strstr(g_uDataBuffer,":"):strstr(g_uDataBuffer,"TCP");

        	if(NULL == pRec)
            {
                s_uReadTimeMark  = uNowTimemark;
                s_uReadStep = R_WAIT_TIME;
                break;
            }

            for(i=0,uDataLen=0;i<7;i++)
            {
                if(*pRec<='9' && *pRec>='0')
                {
                    uDataLen = uDataLen*10 + *pRec - '0';
                }
                else if('\r'==*pRec)
                {
                    break;
                }
                pRec++;
            }

            if((iRecLen-(pRec - g_uDataBuffer)-8) < uDataLen && (!(gWirelessNetMode == 1)))  // 收到数据数量小于模块推送的数据数量
            {
                s_uReadTimeMark  = uNowTimemark;
                s_uReadStep = R_WAIT_TIME;
                RingClear(&g_sRingData);   // 清空缓冲
                DEBUGOUT("数据少:%d<%d\n",(iRecLen-(pRec - g_uDataBuffer)-8),uDataLen);
                break;
            }

            DEBUGOUT("DataLen:%d\n",uDataLen);

            pRec++;
            pRec++;
            RingDataIn(&g_sRingData,pRec,uDataLen); // 放入模块接收数据接收环形缓冲区，避免断帧

            g_uTimerForRecData = uNowTimemark;   // 更新收到数据时标
            //=============================================================
            // 将缓冲区的数据，存入链表
            while(RingNotEmpty(&g_sRingData))
            {
				if(2==RingDataShow(&g_sRingData,g_uDataBuffer,2))  // 先显示出缓冲区里的两个字节
                {
                    if(0x68==g_uDataBuffer[0])  // 头字节为0x68即为104头，否则扔掉数据
                    {
                        if(RingDataLen(&g_sRingData) >= (g_uDataBuffer[1]+2)) // 缓冲区里的数据满足一帧104报文，否则跳出
                        {
                            RingDataOut(&g_sRingData,g_uDataBuffer,g_uDataBuffer[1]+2);
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    {
                        RingDataThrow(&g_sRingData,1);
                        continue;
                    }
                }
                else
                {
                    break;
                }


                if(1==(g_uDataBuffer[2]&0x03))
                {
                    DEBUGOUT("\nRecvS:");
                }
                else if(3==(g_uDataBuffer[2]&0x03))
                {
                    DEBUGOUT("\nRecvU:");
                }
                else
                {
                    DEBUGOUT("\nRecvI:");
                }

                uDataLen = g_uDataBuffer[1] + 2;
                CmStoreRecLink((uint8_t*)g_uDataBuffer,uDataLen);   //接收数据存放至链表

                for(i=0; i<uDataLen; i++)
                {
                    DEBUGOUT("%02X ",g_uDataBuffer[i]);
                }
				
            }
            //=============================================================
            s_uReadTimeMark  = uNowTimemark;
            s_uReadStep = R_WAIT_TIME;

            break;

        case R_RESTART:
            s_uReadTimeMark  = uNowTimemark;
            s_uReadStep = R_WAIT_TIME;
            break;

        default:
            s_uReadStep = R_WAIT_TIME;
            break;
        }
        //==========================================================================================================
    }
}

