#include "chip.h"
//#include "flash.h"

#include "InEeprom.h"
#include "DataFlash.h"
#include "Update.h"

#include "CRC16.h"
#include "WatchDog.h"
#include "GlobalVar.h"
#include "SoftTimer.h"
#include "DataFlash.h"
//=======================================================
#define UPDATA_UART_USE           uart3         // 使用的串口
#define UPDATA_UART_USE_STATE     UART3_STATE   // 串口状态

//=======================================================

#define SOURCE_CODE   0x00028000   // 存储升级代码的ROM地址
//#define TARGET_CODE   0x00006000   // 目标运行程序的ROM地址

#define START_Sector  26            // 目标运行程序起始扇区
#define END_Sector    28            // 目标运行程序起始扇区
//=======================================================
#define SIDE_A      0x00006000
#define SIDE_B      0x00028000
//=======================================================

/****************************************************************************
* 名    称：Updata()
* 功    能：近端升级数据接收保存到DataFlash。
* 入口参数：
            *puData   数据指针
            uLen      数据长度
*
* 出口参数：CRC校验错误返回1，长度不对返回2，正确返回0
* 范    例: 无
****************************************************************************/
uint8_t Update(void)
{
#if(0)
    uint8_t  uDataTemp;

    uint16_t uLen;
    uint16_t uCrc;
    uint16_t uCrcRead;
    uint16_t uRecSeq;                 // 接收到的帧序号
    static  uint16_t uFrameCount=0;   // 接收帧计数
    uint8_t  uaRecData[262];

    UPDATA_MARK_T sEepUpdataMes;
    SOFT_TIMER_T  s_sUpdataTimerDelay;


    uDataTemp = GetRunUpdataMode();

    if(0==UPDATA_UART_USE_STATE.Rec_OK && (0x1A!=uDataTemp) && (0x2A!=uDataTemp))
    {
        return 3;
    }

    UPDATA_UART_USE_STATE.Rec_OK = 0;

    uDataTemp = GetStoreAddr();   // 获取通讯地址

    uLen = UART_GetRxCount(UPDATA_UART_USE);

    if((uLen<258 && 0==uFrameCount) || (uLen<262 && 0!=uFrameCount))
    {
        uFrameCount = 0x00;

        Reboot();  // 复位芯片
    }

    if(0!=uFrameCount)
    {
        uLen = UARTn_Read(UPDATA_UART_USE,uaRecData,262);
    }
    else
    {
        uaRecData[0] = uDataTemp;
        uaRecData[1] = GetRunUpdataMode();

        SetRunUpdataMode(1); // 设置为升级模式

        //--------------------------------
        // 擦除DataFlash
        // DataFlash存储升级数据区域擦除，共96KB，一个区块+8个扇区
        FEED_DOG();
        DataFlash_Block_Erase(0);// 擦除块0
        DataFlash_Sector_Erase(0x010000);
        DataFlash_Sector_Erase(0x011000);
        DataFlash_Sector_Erase(0x012000);
        FEED_DOG();
        DataFlash_Sector_Erase(0x013000);
        DataFlash_Sector_Erase(0x014000);
        DataFlash_Sector_Erase(0x015000);
        DataFlash_Sector_Erase(0x016000);
        DataFlash_Sector_Erase(0x017000);

        uaRecData[2] = 0x00;
        uaRecData[3] = 0x00;

        FEED_DOG();

        uLen = UARTn_Read(UPDATA_UART_USE,&uaRecData[4],258);
    }

    if(uDataTemp!=uaRecData[0] || (0x1A!=uaRecData[1] && 0x2A!=uaRecData[1]))
    {
        // 头错误结束接收升级数据,清除升级标志
        //received_all = 0;
        uFrameCount = 0;
        return 0;
    }

    uCrc     = CRC16 (uaRecData,260);
    uCrcRead = uaRecData[260]<<8 | uaRecData[261];

    if(uCrc!=uCrcRead)
    {
        // 数据校验错误结束接收升级数据,清除升级标志
        uaRecData[4] = 0x00;
        uaRecData[5] = 0x55;  // 校验失败
        uCrc = CRC16(uaRecData,6);

        uaRecData[6] = uCrc>>8;
        uaRecData[7] = uCrc&0xFF;

        //DEBUGOUT("Updata CRC ERR\r\n");
        UARTn_Send(UPDATA_UART_USE,uaRecData,8);
        return 1;
    }

    uRecSeq = (uaRecData[2] <<8) | uaRecData[3];

    if(uFrameCount!=(uRecSeq&0x7FFF))
    {
        uaRecData[2] = (uFrameCount&0x7FFF)>>8;
        uaRecData[3] = uFrameCount&0xFF;  // 需要重传的帧

        uaRecData[4] = 0x00;
        uaRecData[5] = 0x01;  // 接收帧序号不对
        uCrc = CRC16(uaRecData,6);

        uaRecData[6] = uCrc>>8;
        uaRecData[7] = uCrc&0xFF;

        //DEBUGOUT("Updata Seq ERR\r\n");
        UARTn_Send(UPDATA_UART_USE,uaRecData,8);
        return 2;
    }

    if(0==uFrameCount)
    {
        // 擦除DataFlash
    }

    // 存储到DataFlash升级数据区域
    DataFlash_Write(uFrameCount * 256,(uint8_t *)&uaRecData[4],256);
    //DataFlash_Read(uFrameCount * 256,(uint8_t *)&uaRecData[4],256);

    //DataFlash_Read(uFrameCount * 256,(uint8_t *)&uaRecData[0],256);

    //DataFlash_Read(0,(uint8_t *)&uaRecData[0],256);

    uFrameCount++;

    if(uRecSeq&0x8000) // 最后一帧，数据接收完成//if(0xEDAA==((RECdata[2] <<8) | RECdata[3])) // 最后一帧，数据接收完成
    {
        uCrc = EepReadData(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sEepUpdataMes,sizeof(sEepUpdataMes),&sEepUpdataMes.CRC);

        if(0==uCrc)
        {
            sEepUpdataMes.rollback_allow = 0x55;
        }

        sEepUpdataMes.frame_sum = uFrameCount;// 256个字节一帧，总帧数

        sEepUpdataMes.b_version[0] = 0x02;   // 当前版本1
        sEepUpdataMes.b_version[1] = 0x02;   // 当前版本1
        sEepUpdataMes.b_version[2] = 0x02;   // 当前版本1

        sEepUpdataMes.reserve = 0x00;        // 预留，占位

        if(2==(uaRecData[1]>>4))
        {
            sEepUpdataMes.side_tobe = 0xBB;  // 升级到B面
        }
        else
        {
            sEepUpdataMes.side_tobe = 0xAA;  // 升级到A面
        }


        if(!uFrameCount)  // 没有升级包数据
        {
            sEepUpdataMes.updata_mark = 0x55;  //  data[2] = 0x55;    // 0xAA升级；0xBB回滚；0x55无
        }
        else
        {
            sEepUpdataMes.updata_mark = 0xAA;  // data[2] = 0xAA;    // 0xAA升级；0xBB回滚；0x55无
        }

        EepSavedata(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sEepUpdataMes,sizeof(sEepUpdataMes),&sEepUpdataMes.CRC);// 存储

        //DEBUGOUT("UPDATA REC END,REBOOT\r\n");

        //uSide = 0;
        uFrameCount = 0;

        SoftTimerSet(&s_sUpdataTimerDelay,100,SOFT_TIMER_START);
    }

    //--------------------------------------------------------------------
    uaRecData[4] = 0x00;
    uaRecData[5] = 0xAA;  // 数据正确
    uCrc = CRC16(uaRecData,6);

    uaRecData[6] = uCrc>>8;
    uaRecData[7] = uCrc&0xFF;

    UPDATA_UART_USE_STATE.Send_OK = 0;
    UARTn_Send(UPDATA_UART_USE,uaRecData,8);


    if(uRecSeq&0x8000)
    {
        while(1)
        {
            if(SoftTimerExpired(&s_sUpdataTimerDelay))
            {
                Reboot();
            }
        }
    }
    //--------------------------------------------------------------------
#endif
    return 2;

}
