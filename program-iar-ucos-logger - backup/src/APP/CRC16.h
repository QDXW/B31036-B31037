#ifndef __CRC16_H__
#define __CRC16_H__

#ifndef _STDINT
#include <stdint.h>
#endif

extern uint16_t CRC16 (uint8_t *puchMsg,int16_t usDataLen);
extern uint16_t CalculateCRC(uint8_t *pBuffer, uint16_t nLen);
#endif
