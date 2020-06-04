#ifndef TRANSMITDATA_H_
#define TRANSMITDATA_H_
#include "ucos_ii.h"

#ifndef _STDINT
#include <stdint.h>
#endif
#include <stdio.h>
#include <string.h>

//=======================================================================

//=======================================================================
extern void CmLinkInit(void);
extern uint8_t CmStoreRecLink(uint8_t *input,uint8_t bytes);
extern uint8_t CmReadRecLink(uint8_t *output);
extern uint8_t CmStoreSendLink(uint8_t *input,uint8_t bytes);
extern uint8_t CmReadSendLink(uint8_t *output);
extern void CmDelSendLink(void);
extern void CmDelRecLink(void);

extern uint16_t CmRecLinkLen(void);
extern uint16_t CmSendLinkLen(void);
//=======================================================================

extern OS_EVENT   *MesQ;
//extern OS_MEM     *pMemBank;         // 定义内存控制块指针
extern OS_EVENT *pRecordLock;

extern void SysMesInit(void);
//extern void MemBankInit(void);




//extern uint16_t   g_uTransData;

#endif
