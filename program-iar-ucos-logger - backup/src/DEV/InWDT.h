#ifndef __INWDR_H_
#define __INWDR_H_


#define FeedDog()  Chip_WWDT_Feed(LPC_WWDT)

extern void InWDT_Init(void);
extern void InWDT_Deinit(void);


#endif
