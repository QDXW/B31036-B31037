#ifndef CM_H_
#define CM_H_

#include <stdint.h>

typedef struct
{
    uint16_t interval;
    uint16_t record;
    uint8_t  start;
    uint8_t  over;
}SOFT_TIMER_T;


#define SOFT_TIMER_START  1
#define SOFT_TIMER_STOP   0


extern void    SoftTimerRestart(SOFT_TIMER_T *pt);
extern uint8_t SoftTimerExpired(SOFT_TIMER_T *pt);
extern void    SoftTimerSet(SOFT_TIMER_T *pt,uint16_t it,uint8_t start);
extern void    SoftTimerStop(SOFT_TIMER_T *pt);
extern void    SoftTimerReset(SOFT_TIMER_T *pt);
extern uint8_t SoftTimerStatus(SOFT_TIMER_T *pt);

extern void SoftTimerClock(void);
#endif
