#ifndef __IIC_H__
#define __IIC_H__

extern void IIC_Init(void);
extern uint8_t IIC_read(uint8_t *buff,uint8_t len);
extern uint8_t IIC_write(uint8_t *buff,uint8_t len);

#endif
