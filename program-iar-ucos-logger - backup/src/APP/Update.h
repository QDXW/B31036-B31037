#ifndef __UPDATE_H_
#define __UPDATE_H_

#include <stdint.h>

#define EEPROM_UPDATA_MARK_ADDRESS  0x00000500  // 在eeprom里写升级标志

// 用于升级
typedef struct
{
    uint16_t  frame_sum;       // 升级总帧数，256字节一帧
    uint8_t   updata_mark;     // 升级标志0xAA升级；0xBB回滚；0x55无
    uint8_t   rollback_allow;  // 回滚允许；0xBB有回滚数据，0x55无回滚数据
    //uint32_t  flash_addr;      // 程序存储的起始flash地址
    uint8_t   a_version[3];    // A面程序版本
    uint8_t   b_version[3];    // B面程序版本
    uint8_t   sucess;          // 跳转成功0xAA:成功；0x55失败
    uint8_t   side;            // 0xAA:程序在A面；0xBB：程序在B面
    uint8_t   side_tobe;       // 升级目标位置，0xAA:程序在A面；0xBB：程序在B面
    uint8_t   reserve;         // 预留
    uint16_t  CRC;             // 数据CRC校验
}UPDATA_MARK_T;


#define UPDATE_ENTER_NORMAL    1
#define UPDATE_ENTER_ALL       2
#define UPDATE_ENTER_SINGEL    3
#define UPDATE_LEAVE           0

extern  uint8_t Update(void);

#endif
