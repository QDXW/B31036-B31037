#ifndef __USART_H_
#define __USART_H_


#ifndef _STDINT
#include <stdint.h>
#endif

#define uart0    0
#define uart1    1
#define uart2    2
#define uart3    3
#define uart4    4
#define uart_all 10

typedef enum
{
    UART_PARITY_NONE 	= 0,					/*!< No parity */
    UART_PARITY_ODD,	 						/*!< Odd parity */
    UART_PARITY_EVEN, 							/*!< Even parity */
    UART_PARITY_SP_1, 							/*!< Forced "1" stick parity */
    UART_PARITY_SP_0 							/*!< Forced "0" stick parity */
} UART_PARITY_Type;
typedef enum
{
    UART_DATABIT_5		= 0,     		/*!< UART 5 bit data mode */
    UART_DATABIT_6,		     			/*!< UART 6 bit data mode */
    UART_DATABIT_7,		     			/*!< UART 7 bit data mode */
    UART_DATABIT_8		     			/*!< UART 8 bit data mode */
} UART_DATABIT_Type;
typedef enum
{
    UART_STOPBIT_1		= (0),   					/*!< UART 1 Stop Bits Select */
    UART_STOPBIT_2,		 							/*!< UART Two Stop Bits Select */
} UART_STOPBIT_Type;


extern int8_t  UartInit(uint8_t uUartId,uint32_t baud,UART_PARITY_Type eParity);
extern int16_t UartWrite(uint8_t uUartId,const void *data,uint16_t bytes);
extern int16_t UartRead(uint8_t uUartId, void *pdata, uint16_t uLen, uint16_t uTimeout);
extern int16_t UartShowBytes(uint8_t uUartId, uint8_t *pdata, uint16_t uLen);
extern int16_t UartRxLen(uint8_t uUartId);
extern int16_t UartTxLen(uint8_t uUartId);
extern int8_t  UartClear(uint8_t uUartId);
extern int8_t  UartClearSendBuffer(uint8_t uUartId);
extern int8_t  UartClearRecBuffer(uint8_t uUartId);
extern void    UartDeinit(uint8_t uUartId);
//extern uint8_t RxBufIsEmpty(USART_t *p);
#endif
