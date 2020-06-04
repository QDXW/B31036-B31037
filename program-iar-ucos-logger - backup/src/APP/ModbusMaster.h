#ifndef MODBUS_MASTER_H_
#define MODBUS_MASTER_H_
#include "ucos_ii.h"
#include "GlobalVar.h"
#ifndef _STDINT
#include <stdint.h>
#endif
//-----------------------------------------------------------------------------------------
#define MASTER_OK       (0)     // 广播后返回
#define MASTER_lOST     (41)    // 丢帧
#define MASTER_CRC      (42)    // CRC校验错误
#define MASTER_ERR      (43)    // 错误

#define MODBUS_ILLEGAL_FUN    (1)  //
#define MODBUS_ILLEGAL_ADDR   (2)  //
#define MODBUS_ILLEGAL_VALUE  (3)  //
#define MODBUS_ILLEGAL_DEV    (4)  //
#define MODBUS_ILLEGAL_BUSY   (6)  //

#define DT1000UPDATE_START   0x1C     //升级启动帧
#define DT1000UPDATE_DATA    0x2C     //升级数据传输帧
#define DT1000UPDATE_END     0x3C     //升级结束帧

#define DT1000LOG_START      0x1D     //日志启动帧
#define DT1000LOG_DATA       0x2D     //日志数据传输帧
#define DT1000LOG_END        0x3D     //日志结束帧


#define DISCOVERY_START      0x1B   //自发现广播帧
#define DISCOVERY_SET_ADDR   0x2B   //地址自分配
#define DISCOVERY_REPORT     0x3B   //自发现设备上报
/*
0 - No error.
1 - Illegal function.
2 - Illegal data address.
3 - Illegal data value.
4 - Slave device failure.
5 - Acknowledge.
6 - Slave device busy.
7 - Negative acknowledge.
8 - Memory parity error.
*/
//-----------------------------------------------------------------------------------------
#define MASTER_ROUND_OVER   (0)   // 1:开启大循环控制；0:关闭大循环控制
//-----------------------------------------------------------------------------------------
typedef struct
{
    OS_EVENT *pComLock;         // 串口锁指针

    int16_t iComFd;             // 串口文件号

    uint8_t  uRecLostMax;       // 最大丢帧次数，超过认为断连              ----->>>设定最大运行的丢帧次数，超过后认为断连
    uint8_t  uRecCrcMax;        // 最大CRC错误次数，超过认为断连           ----->>>设定最大运行的CRC校验次数，超过后认为断连
    uint16_t uFailTimeOut;      // 丢帧超时，超过后，再次发送数据          ----->>>设定丢帧后再次发送的间隔时间，以1ms为计时单位
    uint16_t u1BTimeOut;        //1B超时时间
    uint16_t uSuccessDelay;     // 一帧成功，延时，延时后查询下一帧        ----->>>设定一帧查询成功后查询下一帧的间隔时间，以1ms为计时单位,设定的值需要比需求的值小100mS左右。

    uint32_t sSuccessTime;      // %内部%一帧查询成功时的时间记录

    #if(1==MASTER_ROUND_OVER)
    uint16_t uRoundTimeOut;     // 一个查询循环总时间                      ----->>>设定一个查询周期的时间，以1s为计时单位
    uint8_t  uRoundOver;        // %内部%一个查询循环结束
    uint32_t uRoundStartTime;   // %内部%大循环起始时间
    #endif


    //uint8_t  uRecLostCount;  // -%内部%-丢帧次数计数                    ----->内部使用，初始化为0
    //uint8_t  uRecCrcCount;   // -%内部%-CRC错误次数计数                 ----->内部使用，初始化为0

    uint8_t  uPreAddr;          // -%内部%-已发送查询设备的地址            ----->内部使用，初始化为0
    uint8_t  uPreFun;           // -%内部%-已发送的功能码                  ----->内部使用，初始化为0
    //uint8_t  uSendLen;       // -%内部%-发送字节数量                    ----->内部使用，初始化为0

}MODBUS_MASTER_T;
typedef struct
{
	char  cDeviceEsn[MAX_device+1][18];		// 南向设备
    
	//uint8_t uEsnMark[MAX_device]; 		   //分配地址与未分配地址的表计
	//uint16_t	CRC;					 // 存储数据的CRC校验  
}DEVICE_ADDR_INFO_T;

//-----------------------------------------------------------------------------------------
extern DEVICE_ADDR_INFO_T uAllocation;

extern int16_t ComMasterWrite(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,const uint16_t *pReg);
extern int16_t ComMasterRead(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData);
extern int16_t ComMasterReadDiscovery(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData);
extern int16_t ComMasterUpdata(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData);


#if(1==MASTER_ROUND_OVER)
extern int16_t ComMasterRoundOver(MODBUS_MASTER_T *pMaster);
#endif
extern int16_t TimeGapJudge(uint32_t sStart,uint32_t sNow,uint32_t uGap);
//-----------------------------------------------------------------------------------------


#endif
