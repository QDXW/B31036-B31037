#ifndef __KEY_SCAN_H__
#define __KEY_SCAN_H__

#ifndef _STDINT
#include <stdint.h>
#endif

typedef  uint8_t   KEYINT;

//以下按键所在的位，需要根据实际电路更改
#define Key_button   0x0001
#define Key_SCARM    0x0002
#define Key1         0x0004
#define Key2         0x0008
#define Key3         0x0010
#define Key4         0x0020

#define LED1(s)          GPIO_SetBit(1,26,s)
#define LED2(s)          GPIO_SetBit(1,27,s)
#define LED3(s)          GPIO_SetBit(1,4,s)
#define LED1_twinkle()   GPIO_TwinkleBit(1,26)
#define LED2_twinkle()   GPIO_TwinkleBit(1,27)
#define LED3_twinkle()   GPIO_TwinkleBit(1,4)

extern KEYINT  KeyDown;  // 按下触发
extern KEYINT  KeyUp;    // 松开触发
extern KEYINT  KeyOn;    // 连续按住


extern  void KeyScan(void);
extern  void LED_KEY_IO_Init(void);

#endif
