#ifndef __INEEPROM_H_
#define __INEEPROM_H_

/*
Remark: The top 64 bytes of the 4 KB EEPROM memory are reserved and
cannot be written to. The entire EEPROM is writable for smaller EEPROM sizes.
*/
#define InEEPROM_ADD_START      0x00000100

#define EEP_OK    (0)
#define EEP_ERR   (1)


extern void InEEPROM_Write(uint32_t dstAdd, uint8_t *ptr, uint32_t byteswrt);
extern void InEEPROM_Read(uint32_t srcAdd, uint8_t *ptr, uint32_t bytesrd);

//extern uint8_t Read_data(void);
//extern uint8_t Save_data(void);

extern uint16_t EepSavedata(uint32_t addr,uint8_t *data,uint16_t bytes,uint16_t *crc);
extern uint16_t EepReadData(uint32_t addr,uint8_t *data,uint16_t bytes,uint16_t *crc);

#endif
