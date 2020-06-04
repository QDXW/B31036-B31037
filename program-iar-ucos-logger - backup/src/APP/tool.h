#ifndef TOOL_H__
#define TOOL_H__


#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define TIMEOUT    0
#define NOTIMEOUT  1

#define Hw  1
#define Lw  0
// 数据转换
typedef union
{
    float   f;
    uint8_t  u[4];  // u[0] 低位；u[3] 高位
}Float_Uint_Convert;

typedef union
{
    uint16_t u;
    int16_t  i;
}Uint_Int_Convert;

typedef union
{
    uint16_t u;
    uint8_t  c[2];  // c[0] 低位；c[1] 高位
}Uint_Char_Convert;

typedef union
{
    float    f;
    uint32_t u;
    int32_t  i;
    uint16_t u16;
    int16_t  i16;
    uint8_t  c[4];
}U32_F_Char_Convert;

extern uint8_t gWirelessNetMode;

extern void Str2IP(const char* str,uint8_t *ip);
extern uint8_t Str2UI(const char* str,uint16_t *result);
extern char *I2Str(const uint16_t in,char* result);
//extern int8_t NoTimeOut(uint32_t sStart,uint32_t sNow,uint32_t uGap);
extern int8_t TimeOut(uint32_t sStart,uint32_t sNow,uint32_t uGap);
extern void ToolDelay(uint16_t uInDelay);
#endif
