#include <stdio.h>
#include <time.h>
#include "log.h"
#include "IEC104.h"
#include "GlobalVar.h"
#include "DataFlash.h"
#include "InEEPROM.h"
#include "WatchDog.h"
#include "memory.h"


#define RECORD_SECTOR_SIZE 4096  // 4kB  DataFlash一个扇区为4KB
#define FEED_WATCH_DOG()   FEED_DOG()    // 喂狗

/******************************************************************************
* 变量声明
******************************************************************************/
uint8_t g_uSaveLog=0;            // 存储历史数据允许，收到对时之后允许存储，检测到掉电后不允许存储
LOG_STRUCT_T sLogData={0};       // 记录存储数据最后一条的DataFlash

uint8_t  *SOUTH_DATA_LOG;        // 南向数据指针，动态申请空间

/******************************************************************************
* 名     称：CopyData()
* 功     能：南向数据采集
* 入口参数：     *rec        南向报文                          
* 出口参数：无
* 范     例: 无
******************************************************************************/
uint8_t CopyData(uint8_t *rec)
{
	if(rec[0] > 0 && rec[0] <= 10)
	{
		if(rec[1] == 0x3B || rec[1] == 0xBB)
		{
			return 0;
		}

		if(SOUTH_DATA_LOG[rec[0]*2 - 1] == 0xFF)
		{
			SOUTH_DATA_LOG[(rec[0]-1)*2] = rec[0];
			SOUTH_DATA_LOG[rec[0]*2 - 1] = rec[1];
		}
		else if(!(SOUTH_DATA_LOG[rec[0]*2 - 1] & 0x80))
		{
			SOUTH_DATA_LOG[(rec[0]-1)*2] = rec[0];
			SOUTH_DATA_LOG[rec[0]*2 - 1] = rec[1];
		}
	}
    return 0;
}

/******************************************************************************
* 名     称：SerchLog()
* 功     能：搜索日志所在
* 入口参数：     date 时间  结构体               
* 出口参数：*uLogDataFlash      日志所在dataflash地址
* 范     例: 无 
******************************************************************************/
uint16_t SerchLog(uint8_t date,uint32_t *uLogDataFlash)
{
    uint8_t dst_date = 0;
    uint16_t result = 0;
    uint8_t uCheckLog[30] = {0};
    uint32_t uLogDataReadFlash;
    
    dst_date = date;

    if(dst_date == sDateTime.Date)
    {
        result = (sDateTime.Hour*12+sDateTime.Minute/5)*26/200+1;     //  26/200=0.13
    }else{
        result = 38;   //12*24*26/200
    }
    
    uLogDataReadFlash = DATAFLASH_LOG_ONE;

    DataFlash_Read(uLogDataReadFlash,uCheckLog,30);

    if(uCheckLog[3] == dst_date || uCheckLog[29] == dst_date)
    {
        if((0xBB==uCheckLog[0] && 0x66==uCheckLog[1]) || (0xBB==uCheckLog[26] && 0x66==uCheckLog[27]))
        {
             *uLogDataFlash = uLogDataReadFlash;
             DEBUGOUT("\nDATAFLASH_LOG_ONE\n");
             return result;
        }
    }
    uLogDataReadFlash = DATAFLASH_LOG_TWO;
    DataFlash_Read(uLogDataReadFlash,uCheckLog,30);

    if(uCheckLog[3] == dst_date || uCheckLog[29] == dst_date)
    {
        if((0xBB==uCheckLog[0] && 0x66==uCheckLog[1]) || (0xBB==uCheckLog[26] && 0x66==uCheckLog[27]))
        {
             *uLogDataFlash = uLogDataReadFlash;
             DEBUGOUT("\nDATAFLASH_LOG_TWO\n");
             return result;
        }
    } 
    uLogDataReadFlash = DATAFLASH_LOG_THREE;
    DataFlash_Read(uLogDataReadFlash,uCheckLog,30);
    if(uCheckLog[3] == dst_date || uCheckLog[29] == dst_date)
    {
        if((0xBB==uCheckLog[0] && 0x66==uCheckLog[1]) || (0xBB==uCheckLog[26] && 0x66==uCheckLog[27]))
        {
             *uLogDataFlash = uLogDataReadFlash;
             DEBUGOUT("\nDATAFLASH_LOG_THREE\n");
             return result;
        }
    }else{
        *uLogDataFlash = 0;
    }
    
    return 0;
}

/******************************************************************************
* 名     称：uGetLastRecordAddr()
* 功     能：搜索日志上一次存储的起始地址
* 入口参数：     sTime 时间  结构体               
* 出口参数：无
* 范     例: 无
******************************************************************************/
uint8_t uGetLastRecordAddr(SYSTEMTIME sTime)
{
    uint8_t uSaveCheck[30] = {0};

    if((sLogData.LogFlashAddr==DATAFLASH_LOG_ONE) || (sLogData.LogFlashAddr==DATAFLASH_LOG_TWO) || (sLogData.LogFlashAddr==DATAFLASH_LOG_THREE))
    {
        return 0;
    }
    else if((DATAFLASH_LOG_ONE<sLogData.LogFlashAddr && DATAFLASH_LOG_TWO>sLogData.LogFlashAddr))
    {        
        DataFlash_Read(DATAFLASH_LOG_ONE,uSaveCheck,30);
    }
    else if(DATAFLASH_LOG_TWO<sLogData.LogFlashAddr && DATAFLASH_LOG_THREE>sLogData.LogFlashAddr)
    {
        DataFlash_Read(DATAFLASH_LOG_TWO,uSaveCheck,30);
    }
    else if(DATAFLASH_LOG_THREE<sLogData.LogFlashAddr && DATAFLASH_LOG_END>sLogData.LogFlashAddr)
    {
        DataFlash_Read(DATAFLASH_LOG_THREE,uSaveCheck,30);
    }   
        
    if(sTime.Date==uSaveCheck[3] || sTime.Date==uSaveCheck[29])
    {
        return 0;
    }
/*
    if((DATAFLASH_LOG_ONE<sLogData.LogFlashAddr && DATAFLASH_LOG_TWO>sLogData.LogFlashAddr))
    {        
        DataFlash_Read(sLogData.LogFlashAddr,uSaveCheck,30);
    }else if(DATAFLASH_LOG_TWO<sLogData.LogFlashAddr && DATAFLASH_LOG_THREE>sLogData.LogFlashAddr)
    {
        DataFlash_Read(sLogData.LogFlashAddr,uSaveCheck,30);
    }else if(DATAFLASH_LOG_THREE<sLogData.LogFlashAddr && DATAFLASH_LOG_END>sLogData.LogFlashAddr)
    {
        DataFlash_Read(sLogData.LogFlashAddr,uSaveCheck,30);
    }   
        
    if(sTime.Date==uSaveCheck[3])
    {
        return 0;
    }
*/
    if((DATAFLASH_LOG_ONE<sLogData.LogFlashAddr && DATAFLASH_LOG_TWO>sLogData.LogFlashAddr))
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_TWO-26;
        DataFlash_Sector_Erase(DATAFLASH_LOG_TWO);
        DataFlash_Sector_Erase(DATAFLASH_LOG_TWO+RECORD_SECTOR_SIZE);
    }
    else if(DATAFLASH_LOG_TWO<sLogData.LogFlashAddr && DATAFLASH_LOG_THREE>sLogData.LogFlashAddr)
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_THREE-26;
        DataFlash_Sector_Erase(DATAFLASH_LOG_THREE);
        DataFlash_Sector_Erase(DATAFLASH_LOG_THREE+RECORD_SECTOR_SIZE);

    }
    else if(DATAFLASH_LOG_THREE<sLogData.LogFlashAddr && DATAFLASH_LOG_END>sLogData.LogFlashAddr)
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_ONE-26;
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE);
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE+RECORD_SECTOR_SIZE);
    }
    return 0;
}

/******************************************************************************
* 名     称：SaveLogInit()
* 功     能：日志存储初始化
* 入口参数：     sTime               时间  结构体
*           puSouthLog          南向日志起始指针
*           puNorthLog          北向日志起始指针
*           uSouthLogPoints     南向日志数据长度
*           uNorthLogPoints     北向日志数据长度               
* 出口参数：无
* 范     例: 无
******************************************************************************/
uint32_t uSaveSouthLog(SYSTEMTIME sTime,const uint8_t *puSouthLog,const uint8_t *puNorthLog,uint16_t uSouthLogPoints,uint16_t uNorthLogPoints)
{
    uint8_t uHeadMark[6] = {0};
    uint8_t  i;    
    uint8_t  uSectorErase;          // 需要擦除的扇区数量
    uint16_t uSouthLenAll;          // 南向日志数据长度
    uint16_t uSectorEmpty;          // 扇区剩余的大小
    uint32_t uWriteFlashAddrStart;  // 下一个数据的写起始地址
    uint32_t uNextSectorAddr;       // 下一个扇区起始地址
    uint32_t uEraseSectorAddr;      // 待擦除扇区起始地址
    puNorthLog = puNorthLog;
    uNorthLogPoints = uNorthLogPoints;

    uHeadMark[0] = LOG_HEAD_A;
    uHeadMark[1] = LOG_HEAD_B;

    uHeadMark[2] = sTime.Month;
    uHeadMark[3] = sTime.Date;
    uHeadMark[4] = sTime.Hour;
    uHeadMark[5] = sTime.Minute;

    uGetLastRecordAddr(sTime);   //存日志前判断应该存在哪个区间

    uSouthLenAll = uSouthLogPoints + 6;   // 两个头标 + 时标 + 有效数据
    uWriteFlashAddrStart = sLogData.LogFlashAddr + uSouthLenAll;   // 写数据的起始地址
    FEED_WATCH_DOG();  // 喂狗
    
    if((uWriteFlashAddrStart+uSouthLenAll > DATAFLASH_LOG_END) || (0==sLogData.uIsExist))  // 写起始地址到Flash结束地址空间不够存一帧数据，回到头地址写数??
    {
       uWriteFlashAddrStart = DATAFLASH_LOG_ONE;
       uNextSectorAddr      = DATAFLASH_LOG_ONE;
       uSectorEmpty         = 0x00;

       sLogData.LogFlashAddr   = DATAFLASH_LOG_ONE;  // 本帧（最新一帧）存储的起始地址
    }
    else
    {
       uNextSectorAddr = (uWriteFlashAddrStart/RECORD_SECTOR_SIZE + 1)*RECORD_SECTOR_SIZE;  // 计算出下个扇区的起始地址
       uSectorEmpty    = uNextSectorAddr - uWriteFlashAddrStart;   // 计算本扇区的空余容量

       sLogData.LogFlashAddr = uWriteFlashAddrStart;  // 本帧（最新一帧）存储的起始地址
    }
    
    DEBUGOUT("#LOG START ADDR:0x%0.6X\r\n",uWriteFlashAddrStart);  // 此帧的起始地址

   if(uSouthLenAll > uSectorEmpty)
   {
       uSectorErase = (uSouthLenAll - uSectorEmpty) / RECORD_SECTOR_SIZE;
       if((uSouthLenAll - uSectorEmpty) % RECORD_SECTOR_SIZE)
       {
           uSectorErase++;
       }
       
       for(i=0;i<uSectorErase;i++)
       {
           FEED_WATCH_DOG();  // 喂狗
       
           uEraseSectorAddr = uNextSectorAddr + RECORD_SECTOR_SIZE * i;
       
           if(uEraseSectorAddr > DATAFLASH_LOG_END)   // 待擦除的扇区超过芯片容量，返回起始扇区擦??
           {
               uEraseSectorAddr = uEraseSectorAddr - (DATAFLASH_LOG_END + 1) + DATAFLASH_LOG_ONE;
           }
       
           //DataFlash_Sector_Erase(uEraseSectorAddr);   // 擦除扇区
       }
   }
       
   //---------------------------------------------------------------------------------------------
   if((uWriteFlashAddrStart+uSouthLenAll) < DATAFLASH_LOG_END)  // 到Flash结束地址前可以写完数次数
   {
       // 存储到DataFlash数据区域
       DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)uHeadMark,6);            // 存储头标和时间
       
       FEED_WATCH_DOG();  // 喂狗
       uWriteFlashAddrStart += 6;
       DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)puSouthLog,uSouthLogPoints);   // 存储南向日志
   }

   sLogData.uIsExist = 1;
   // 存储
   EepSavedata(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);
   
   //============================================================================================
   DEBUGOUT("#LOG END ADDR:0x%0.6X\r\n",sLogData.LogFlashAddr + uSouthLenAll); // 此帧的结束地址
   memset(SOUTH_DATA_LOG,0,20);
           
   return 0;
}

/******************************************************************************
* 名     称：SaveLogInit()
* 功     能：日志存储初始化
* 入口参数：     uReset  1:初始化恢复出厂设置            0:正常初始化               
* 出口参数：无
* 范     例: 无
******************************************************************************/
void SaveLogInit(uint8_t uReset)
{
    uint8_t uRes;
    uint8_t uSaveCheck[6] = {0};
    uint32_t uLogDataSaveFlash;

    SOUTH_DATA_LOG = WMemMalloc(SOUTH_DATA_LOG,20);  //10台设备包含设备编号和返回功能码

    if(NULL==SOUTH_DATA_LOG)
    {
        DEBUGOUT("申请日志空间失败!\r\n");
        return;
    }
    for(int i=0;i<20;i++)
    {
        SOUTH_DATA_LOG[i] = 0xFF;
    }

    if(uReset)
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_ONE;
        sLogData.uIsExist     = 0;
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE);
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE+RECORD_SECTOR_SIZE);

        EepSavedata(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);
    }
    else
    {   
    
        uRes = EepReadData(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);// 数采读取
        if(0==uRes) // 读取失败，手动检索最后一条地址
        {
            
            DataFlash_Read(DATAFLASH_LOG_ONE,uSaveCheck,6);
            if(uSaveCheck[3] == sDateTime.Date)
            {
                if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
                {
                    sLogData.LogFlashAddr = DATAFLASH_LOG_ONE+(sDateTime.Hour*12+sDateTime.Minute/5)*26;
                }
            }
            
            FEED_WATCH_DOG();  // 喂狗
            DataFlash_Read(DATAFLASH_LOG_TWO,uSaveCheck,6);
            if(uSaveCheck[3] == sDateTime.Date)
            {
                if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
                {
                    sLogData.LogFlashAddr = DATAFLASH_LOG_TWO+(sDateTime.Hour*12+sDateTime.Minute/5)*26;
                }
            }
            
            FEED_WATCH_DOG();  // 喂狗
            DataFlash_Read(DATAFLASH_LOG_THREE,uSaveCheck,6);
            if(uSaveCheck[3] == sDateTime.Date)
            {
                if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
                {
                    sLogData.LogFlashAddr = DATAFLASH_LOG_THREE+(sDateTime.Hour*12+sDateTime.Minute/5)*26;
                }
            }
            
            sLogData.uIsExist = 1;        
            EepSavedata(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);
            return;
        }

        uLogDataSaveFlash = sLogData.LogFlashAddr;
        DataFlash_Read(uLogDataSaveFlash,uSaveCheck,6);
        if(uSaveCheck[3] == sDateTime.Date)
        {
            if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
            {
                return;
            }
        }
    }
    return;
}
