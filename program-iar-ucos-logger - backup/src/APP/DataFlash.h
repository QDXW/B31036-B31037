#ifndef __DATA_FLASH_H__
#define __DATA_FLASH_H__

#include <stdint.h>

#define DATAFLASH_OK    (0)
#define DATAFLASH_ERR   (1)

extern void DataFlashInit(void);
extern void DataFlash_Read_ID(void);
extern uint16_t DataFlash_Write(uint32_t dstAdd,uint8_t *ptr,uint16_t byteswrt);
extern uint16_t DataFlash_Read(uint32_t dstAdd,uint8_t *ptr,uint16_t bytesrd);
extern uint8_t DataFlash_Sector_Erase(uint32_t sectorAddr);
extern uint8_t DataFlash_Block_Erase(uint8_t blockAddr);
extern uint8_t DataFlash_Chip_Erase(void);

#endif
