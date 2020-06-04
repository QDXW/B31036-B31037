#ifndef __GPIO_H__
#define __GPIO_H__

#include "chip.h"
#ifndef _STDINT
#include <stdint.h>
#endif

//extern void GPIO_SetDir(uint8_t port,uint8_t pin,bool dir);
//extern void GPIO_SetBit(uint8_t port,uint8_t pin,bool val);

#define Input  false
#define Output true

#define High  true
#define Low   false

#define ON   true
#define OF   false

#define GPIO_SetDir(x,y,t)    Chip_GPIO_WriteDirBit(LPC_GPIO, (x), (y), (t))
#define GPIO_SetBit(x,y,t)    Chip_GPIO_WritePortBit(LPC_GPIO, (x), (y), (t))
#define GPIO_ReadBit(x,y)     (Chip_GPIO_ReadPortBit(LPC_GPIO,x,y))
#define GPIO_TwinkleBit(x,y)  Chip_GPIO_SetPinToggle(LPC_GPIO, x, y)
#define GPIO_SetMux(x,y,t)    Chip_IOCON_PinMuxSet(LPC_IOCON, (x), (y), (t))
#endif
