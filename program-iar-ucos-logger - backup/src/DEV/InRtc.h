#ifndef INRTC_H_
#define INRTC_H_

#include <stdint.h>

typedef struct
{
    uint8_t uYear;
    uint8_t uMonth;
    uint8_t uDate;
    uint8_t uHour;
    uint8_t uMinute;
    uint8_t uSecond;
    uint8_t uWeek;
}INRTC_MES_T;


extern void GetInRtc(void);
extern void InitInRtc(void);
extern void SetInRtc(void);

extern INRTC_MES_T g_sRTC;

extern uint32_t GetInRtcCount(void);

extern void SetInRtcCount(uint32_t count);

#endif
