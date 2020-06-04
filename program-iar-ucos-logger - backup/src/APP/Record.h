#ifndef RECORD_H_
#define RECORD_H_

#include <stdint.h>
#include "time.h"
#include "RealTime.h"

//====================================================
typedef struct
{
    uint32_t  uFlashAddr;      // 点表Dataflash存储地址
    uint16_t  uExist;          // 有数据//预留uReserve
    uint16_t  CRC;             // 数据CRC校验
}RECORD_STRUCT_T;
extern RECORD_STRUCT_T sRecord; // 记录存储数据最后一条的DataFlash

//====================================================
extern uint32_t SearchRecordLastAddr(uint8_t uType,uint16_t uDataLen);
extern void RecordInit(uint8_t uReset);
extern void SetRecordAllow(uint8_t uAllow);
extern uint8_t GetRecordAllow(void);
extern uint8_t GetRecordExist(void);
extern uint8_t RecordHistory(const uint8_t *puYxData,const uint8_t *puYcData,const uint16_t uYxPoint,const uint16_t uYcPoint);
extern time_t TimeToLong(uint8_t *UTC);


#endif // RECORD_H_
