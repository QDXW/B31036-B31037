#ifndef MEMORY_H_
#define MEMORY_H_

#include <stdint.h>

extern void *WMemMalloc(void *ptr,unsigned int size);
extern void *WMemFree(void *ptr);
extern uint16_t FreeRamSpace(void);
#endif
