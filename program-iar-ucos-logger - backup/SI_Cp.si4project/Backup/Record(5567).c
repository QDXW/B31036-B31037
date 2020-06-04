
#include "Record.h"

#include "GlobalVar.h"
#include "DataFlash.h"
#include "RealTime.h"
#include "InEEPROM.h"
#include "WatchDog.h"
#include "ucos_ii.h"
#include "time.h"
//========================================================================
//#define  DATAFLASH_RECORD_HEAD        0x01E000     // 历史数据存放在DataFlash的起始地址
//#define  DATAFLASH_RECORD_END         0x7FFFFF     // 历史数据存放在DataFlash的结束地址

//========================================================================
#define FEED_WATCH_DOG()   FEED_DOG()    // 喂狗

#define RECORD_HEAD_A  0xAA
#define RECORD_HEAD_B  0x55

#define RECORD_SECTOR_SIZE 4096  // 4kB  DataFlash一个扇区为4KB
/******************************************************************************
* 变量声明
******************************************************************************/
uint8_t g_uSaveRecord=0;     // 存储历史数据允许，收到对时之后允许存储，检测到掉电后不允许存储
RECORD_STRUCT_T sRecord={0}; // 记录存储数据最后一条的DataFlash

/******************************************************************************
* 名    称：SetRecordAllow()
* 功    能：数据记录允许。
* 入口参数：
*           uAllow        1：允许记录数据   0：不允许记录数据

* 出口参数：无
* 范    例:
******************************************************************************/
void SetRecordAllow(uint8_t uAllow)
{
    g_uSaveRecord = uAllow;
}

/******************************************************************************
* 名    称：GetRecordAllow()
* 功    能：获取数据记录允许。
* 入口参数：
*           uAllow        1：允许记录数据   0：不允许记录数据

* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t GetRecordAllow(void)
{
    return g_uSaveRecord;
}
/******************************************************************************
* 名    称：GetRecordExist()
* 功    能：获取是否有计时数据记录。
* 入口参数：
*           无

* 出口参数：1：有历史数据；0：无历史数据
* 范    例:
******************************************************************************/
uint8_t GetRecordExist(void)
{
    return sRecord.uExist;
}

/******************************************************************************
* 名    称：TimeToLong()
* 功    能：将时间转为从1970年开始的计秒
* 入口参数：
*           uType           类型，1：历史数据；2：日志
            uDataLen        一条数据长度

* 出口参数：最后一条记录的存储地址
* 范    例:
******************************************************************************/
time_t TimeToLong(uint8_t *UTC)
{
    time_t uTime;
    struct tm sT;
    /*struct tm {
        int tm_sec;     // seconds after the minute - [0,59]
        int tm_min;     // minutes after the hour - [0,59]
        int tm_hour;    // hours since midnight - [0,23]
        int tm_mday;    // day of the month - [1,31]
        int tm_mon;     // months since January - [0,11]
        int tm_year;    // years since 1900
        int tm_wday;    // days since Sunday - [0,6]
        int tm_yday;    // days since January 1 - [0,365]
        int tm_isdst;   // daylight savings time flag
       };

        可以看出，这个机构定义了年、月、日、时、分、秒、星期、当年中的某一天、夏令时。可以用这个结构很方便的显示时间。
    */

    sT.tm_year = UTC[0]+100;   // 系统函数计算年从1900开始
    sT.tm_mon  = UTC[1] - 1;
    sT.tm_mday = UTC[2] & 0x1F;
    sT.tm_hour = UTC[3];
    sT.tm_min  = UTC[4];
    sT.tm_sec  = UTC[5];
    sT.tm_isdst = 0;   //不实行夏令时

    uTime = mktime(&sT);

    return uTime;
}

/******************************************************************************
* 名    称：SearchRecordLastAddr()
* 功    能：搜索最后一条记录的存储地址。
* 入口参数：
*           uType           类型，1：历史数据；2：日志
            uDataLen        一条数据长度

* 出口参数：最后一条记录的存储地址
* 范    例:
******************************************************************************/
uint32_t SearchRecordLastAddr(uint8_t uType,uint16_t uDataLen)
{
    uint8_t  uaMark[8];
    uint32_t uFlashAddr=0;  // 数据的写地址
    uint32_t uFlashAddrTemp;  // 数据的写地址
    uint32_t uFlashEnd;

    time_t uTime1;
    time_t uTime2;

    if(1==uType)
    {
        uFlashAddrTemp = DATAFLASH_RECORD_HEAD;

        DataFlash_Read(uFlashAddrTemp,(uint8_t*)uaMark,8);
        if(RECORD_HEAD_A==uaMark[0] && RECORD_HEAD_B==uaMark[1])
        {
            uTime1 = TimeToLong(&uaMark[2]);
        }

        uFlashEnd = DATAFLASH_RECORD_END - uDataLen + 1; // 最接近存满数据的数据帧地址
        uFlashAddrTemp += uDataLen;
        do
        {
            DataFlash_Read(uFlashAddrTemp,(uint8_t*)uaMark,8);
            if(RECORD_HEAD_A==uaMark[0] && RECORD_HEAD_B==uaMark[1])
            {
                uTime2 = TimeToLong(&uaMark[2]);

                if(uTime2 > uTime1)
                {
                    uTime1 = uTime2;
                    uFlashAddr = uFlashAddrTemp;
                }
            }
            uFlashAddrTemp += uDataLen;
        }while(uFlashAddrTemp<=uFlashEnd);
    }
    return uFlashAddr;
}
/******************************************************************************
* 名    称：RecordAllow()
* 功    能：数据记录允许。
* 入口参数：
*           uAllow        1：允许记录数据   0：不允许记录数据
            uReset        1：初始化恢复出厂设置    0：正常初始化

* 出口参数：无
* 范    例:
******************************************************************************/
void RecordInit(uint8_t uReset)
{
    uint8_t uRes;

    if(uReset)
    {
        sRecord.uFlashAddr = DATAFLASH_RECORD_HEAD;
        sRecord.uExist     = 0;
        //sRecord.uLen       = g_DeviceSouth.yc_sum * 4 + g_DeviceSouth.yx_sum;  // 遥测点总数，每个点占用4个字节；遥信点总数，每个点占用1个字节
        DataFlash_Sector_Erase(DATAFLASH_RECORD_HEAD);   // 擦除扇区

        EepSavedata(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);
    }
    else
    {
        uRes = EepReadData(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);// 数采读取

        if(EEP_ERR==uRes) // 读取失败，手动检索最后一条地址
        {
            if(0!=g_DeviceSouth.yx_sum + g_DeviceSouth.yc_sum*4)
            {
                sRecord.uFlashAddr = SearchRecordLastAddr(1,g_DeviceSouth.yx_sum + g_DeviceSouth.yc_sum*4 + 8);
            }
            else
            {
                sRecord.uFlashAddr = 0;
            }

            if(0==sRecord.uFlashAddr)
            {
                sRecord.uFlashAddr = DATAFLASH_RECORD_HEAD;
                sRecord.uExist = 0;
            }
            EepSavedata(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);
        }
    }
}
//========================================================================
/******************************************************************************
* 名    称：RecordHistory()
* 功    能：数据存储。
* 入口参数：
            *puYxData           遥信数据起始指针
            *puYcData           遥测数据起始指针
            uYxPoint            遥信点数
            uYcPoint            遥测点数

* 出口参数：无
* 范    例:
******************************************************************************/
uint8_t RecordHistory(const uint8_t *puYxData,const uint8_t *puYcData,const uint16_t uYxPoint,const uint16_t uYcPoint)
{
    uint8_t  uaMark[8];
    //uint8_t  check2[8]={2};
    uint8_t  i,result;
    uint8_t  uSectorErase;          // 需要擦除的扇区数量
    uint16_t uLenAll;               // 总数据长度
    uint16_t uSectorEmpty;          // 扇区剩余的大小
    uint32_t uWriteFlashAddrStart;  // 下一个数据的写起始地址
    uint32_t uNextSectorAddr;       // 下一个扇区起始地址
    uint32_t uEraseSectorAddr;      // 待擦除扇区起始地址

    SYSTEMTIME *pTime;

    pTime = RealTimeGet();

    uaMark[0] = RECORD_HEAD_A;
    uaMark[1] = RECORD_HEAD_B;

    uaMark[2] = pTime->Year;
    uaMark[3] = pTime->Month;
    uaMark[4] = pTime->Week<<5 | pTime->Date;     // 高3位为星期
    uaMark[5] = pTime->Hour;
    uaMark[6] = pTime->Minute;
    uaMark[7] = pTime->Second;

    /*if(DATAFLASH_RECORD_HEAD == uRec.uFlashAddr)
    {
        DataFlash_Sector_Erase(DATAFLASH_RECORD_HEAD);
    }*/
    //-------------------------------------------------------------------------------------
    uLenAll = uYxPoint + uYcPoint*4 + 8;   // 两个头标识 + 时标 + 有效数据

    uWriteFlashAddrStart = sRecord.uFlashAddr + uLenAll;   // 写数据的起始地址

    if((uWriteFlashAddrStart+uLenAll > DATAFLASH_RECORD_END) || (0==sRecord.uExist))  // 写起始地址到Flash结束地址空间不够存一帧数据，回到头地址写数据
    {
        uWriteFlashAddrStart = DATAFLASH_RECORD_HEAD;
        uNextSectorAddr      = DATAFLASH_RECORD_HEAD;
        uSectorEmpty         = 0x00;

        sRecord.uFlashAddr   = DATAFLASH_RECORD_HEAD;  // 本帧（最新一帧）存储的起始地址
    }
    else
    {
        uNextSectorAddr = (uWriteFlashAddrStart/RECORD_SECTOR_SIZE + 1)*RECORD_SECTOR_SIZE;  // 计算出下个扇区的起始地址
        uSectorEmpty    = uNextSectorAddr - uWriteFlashAddrStart;   // 计算本扇区的空余容量

        sRecord.uFlashAddr = uWriteFlashAddrStart;  // 本帧（最新一帧）存储的起始地址
    }

    DEBUGOUT("#RECORD START ADDR:0x%0.6X\r\n",uWriteFlashAddrStart);  // 此帧的起始地址

    if(uLenAll > uSectorEmpty)
    {
        uSectorErase = (uLenAll - uSectorEmpty) / RECORD_SECTOR_SIZE;
        if((uLenAll - uSectorEmpty) % RECORD_SECTOR_SIZE)
        {
            uSectorErase++;
        }
        FEED_WATCH_DOG();  // 喂狗

        for(i=0;i<uSectorErase;i++)
        {
            uEraseSectorAddr = uNextSectorAddr + RECORD_SECTOR_SIZE * i;

            if(uEraseSectorAddr > DATAFLASH_RECORD_END)   // 待擦除的扇区超过芯片容量，返回起始扇区擦除
            {
                uEraseSectorAddr = uEraseSectorAddr - (DATAFLASH_RECORD_END + 1) + DATAFLASH_RECORD_HEAD;
            }

            result = DataFlash_Sector_Erase(uEraseSectorAddr);   // 擦除扇区
            if(!result)
            {
                DEBUGOUT("flash uEraseSectorAddr failed at：0x%0.6X !\n",uEraseSectorAddr);
                result = DataFlash_Sector_Erase(uEraseSectorAddr);   // 擦除扇区
            }

            DEBUGOUT("flash uEraseSectorAddr at：0x%0.6X !\n",uEraseSectorAddr);
        }
    }

    //---------------------------------------------------------------------------------------------
    if((uWriteFlashAddrStart+uLenAll) < DATAFLASH_RECORD_END)  // 到Flash结束地址前可以写完数据
    {
        // 存储到DataFlash数据区域
        DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)uaMark,8);            // 存储头+时标
        DEBUGOUT("#RECORD TIME:%d-%d-%d %d:%d:%d\n",pTime->Year,pTime->Month,pTime->Date,pTime->Hour,pTime->Minute,pTime->Second);
        OSTimeDly(5);

        FEED_WATCH_DOG();  // 喂狗
        //DataFlash_Read(uWriteFlashAddrStart, check2, 8);
        /*if(0x01==SouthSwich)
        {
            DEBUGOUT("#AFTER RECORD :");
            for(int i=0;i<8;i++)
            {
                DEBUGOUT("%d ",check2[i]);
            }
			DEBUGOUT("\r\n");
        }*/

        uWriteFlashAddrStart += 8;
        DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)puYxData,uYxPoint);   // 存储遥信点

        FEED_WATCH_DOG();  // 喂狗

        uWriteFlashAddrStart += uYxPoint;
        DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)puYcData,uYcPoint*4); // 存储遥测点
    }
    
    sRecord.uExist = 1;
    // 存储
    EepSavedata(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);
    //============================================================================================
    DEBUGOUT("$RECORD END ADDR:0x%0.6X\r\n",sRecord.uFlashAddr + uLenAll); // 此帧的结束地址
    return 0;
}









