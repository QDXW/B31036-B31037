#ifndef __SPI_H__
#define __SPI_H__

#ifndef _STDINT
#include <stdint.h>
#endif

#define SPI0  0
#define SPI1  1

extern void SPI_Init(uint8_t sp);
extern uint16_t SPI_Send(uint8_t sp,uint8_t *data,uint16_t bytes);
extern uint16_t SPI_Read(uint8_t sp,uint8_t *data,uint16_t bytes);
extern void SPI_Deinit(uint8_t sp);
#endif