#ifndef __IIC_POLLING_H__
#define __IIC_POLLING_H__

extern void IIC_Init(void);
extern uint8_t IIC_read(uint8_t AddressI2C,uint8_t *buff,uint8_t len);
extern uint8_t IIC_write(uint8_t AddressI2C,uint8_t *buff,uint8_t len);

#endif
