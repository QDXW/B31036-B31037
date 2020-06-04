#include "Usart.h"
#include "chip.h"
#include "GPIO.h"
#include "GlobalVar.h"
#include "ucos_ii.h"
#include <stdio.h>

//======================================================================================
#define USE_UART0 (1)  // 使用串口0，1：使用；0：不使用
#define USE_UART1 (0)  // 使用串口1，1：使用；0：不使用
#define USE_UART2 (0)  // 使用串口2，1：使用；0：不使用
#define USE_UART3 (1)  // 使用串口3，1：使用；0：不使用
#define USE_UART4 (1)  // 使用串口4，1：使用；0：不使用
//======================================================================================
#define UART_SRB_SIZE_0 32	    /* Send */
#define UART_RRB_SIZE_0 64	    /* Receive 只能定义成2的整数次。128,256,512等*/

#define UART_SRB_SIZE_1 32	    /* Send */
#define UART_RRB_SIZE_1 32	    /* Receive 只能定义成2的整数次。128,256,512等*/

#define UART_SRB_SIZE_2 32	    /* Send */
#define UART_RRB_SIZE_2 32   	/* Receive 只能定义成2的整数次。128,256,512等*/

#define UART_SRB_SIZE_3 256	    /* Send */
#define UART_RRB_SIZE_3 512	    /* Receive 只能定义成2的整数次。128,256,512等*/

#define UART_SRB_SIZE_4 512	    /* Send */
#define UART_RRB_SIZE_4 512	/* Receive 只能定义成2的整数次。128,256,512等*/
//======================================================================================
#define UART0_RX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN))
#define UART0_TX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 19, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN))

#define UART1_RX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, 0x94)
#define UART1_TX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, 0x94)

#define UART2_RX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 0, 0x93)
#define UART2_TX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6, 0x92)

#define UART3_RX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 3, 0x91)
#define UART3_TX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 4, 0x91)

#define UART4_RX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 11, 0x91)
#define UART4_TX_PIN_INIT() Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 12, 0x91)
//======================================================================================
//======================================================================================
//======================================================================================
//======================================================================================
//======================================================================================
//======================================================================================
//======================================================================================
// 串口0收发控制IO初始化
void Uart0Io(void)
{
    ;
}
// 串口1收发控制IO初始化
void Uart1Io(void)
{
    ;
}
// 串口2收发控制IO初始化
void Uart2Io(void)
{
    ;
}
// 串口3收发控制IO初始化
void Uart3Io(void)
{
    GPIO_SetDir(2,6,Output);
    GPIO_SetDir(2,7,Output);
}
// 串口4收发控制IO初始化
void Uart4Io(void)
{
    ;
}
//---------------------------------------
// 串口0进入接收模式
void Uart0Rec(void)
{
    ;
}
// 串口1进入接收模式
void Uart1Rec(void)
{
    ;
}
// 串口2进入接收模式
void Uart2Rec(void)
{
    ;
}
// 串口3进入接收模式
void Uart3Rec(void)
{
    GPIO_SetBit(2,7,Low);
    GPIO_SetBit(2,6,Low);
}
// 串口4进入接收模式
void Uart4Rec(void)
{
    ;
}
//---------------------------------------
// 串口0进入发送模式
void Uart0Send(void)
{
    ;
}
// 串口1进入发送模式
void Uart1Send(void)
{
    ;
}
// 串口2进入发送模式
void Uart2Send(void)
{
    ;
}
// 串口3进入发送模式
void Uart3Send(void)
{
    GPIO_SetBit(2,6,High);
    GPIO_SetBit(2,7,High);
}
// 串口4进入发送模式
void Uart4Send(void)
{
    ;
}


//======================================================================================
//======================================================================================
#if (USE_UART0==1)
static uint8_t uUart0TxBuf[UART_SRB_SIZE_0];
static uint8_t uUart0RxBuf[UART_RRB_SIZE_0];
#endif
#if (USE_UART1==1)
static uint8_t uUart1TxBuf[UART_SRB_SIZE_1];
static uint8_t uUart1RxBuf[UART_RRB_SIZE_1];
#endif
#if (USE_UART2==1)
static uint8_t uUart2TxBuf[UART_SRB_SIZE_2];
static uint8_t uUart2RxBuf[UART_RRB_SIZE_2];
#endif
#if (USE_UART3==1)
static uint8_t uUart3TxBuf[UART_SRB_SIZE_3];
static uint8_t uUart3RxBuf[UART_RRB_SIZE_3];
#endif
#if (USE_UART4==1)
static uint8_t uUart4TxBuf[UART_SRB_SIZE_4];
static uint8_t uUart4RxBuf[UART_RRB_SIZE_4];
#endif
//======================================================================================

typedef struct
{
    uint32_t Baud_rate;   		/**< UART baud rate */
    UART_PARITY_Type Parity;    	/**< Parity selection, should be:
							   - UART_PARITY_NONE: No parity
							   - UART_PARITY_ODD: Odd parity
							   - UART_PARITY_EVEN: Even parity
							   - UART_PARITY_SP_1: Forced "1" stick parity
							   - UART_PARITY_SP_0: Forced "0" stick parity
							   */
    UART_DATABIT_Type Databits;   /**< Number of data bits, should be:
							   - UART_DATABIT_5: UART 5 bit data mode
							   - UART_DATABIT_6: UART 6 bit data mode
							   - UART_DATABIT_7: UART 7 bit data mode
							   - UART_DATABIT_8: UART 8 bit data mode
							   */
    UART_STOPBIT_Type Stopbits;   /**< Number of stop bits, should be:
							   - UART_STOPBIT_1: UART 1 Stop Bits Select
							   - UART_STOPBIT_2: UART 2 Stop Bits Select
							   */
} UART_CFG_Type;
//======================================================================================

typedef void (*UartHook_t)(void);

typedef struct
{
    LPC_USARTN_T *uart; // uart regs

    uint16_t uRxIn;
    uint16_t uRxOut;
    uint16_t uRxLen;

    uint16_t uTxIn;
    uint16_t uTxOut;
    uint16_t uTxLen;


    UartHook_t tx_hook;    // 发送时调用
    UartHook_t rx_hook;    // 发送结束后调用

    UART_CFG_Type config;

    uint32_t bit_len;       // bit长度，单位为cycle，用于计算发送完成时间

    uint8_t *RxBuf;
    uint8_t *TxBuf;
    uint16_t uRxBufSize;
    uint16_t uTxBufSize;

    UartHook_t IoInit;
    UartHook_t IoSend;
    UartHook_t IoRec;

} USART_t;
//======================================================================================



//======================================================================================
// uarts struct
#if (USE_UART0==1)
static USART_t sUart0 = {         0, 0,0,0, 0,0,0, (UartHook_t)0,   (UartHook_t)0,   {9600, UART_PARITY_NONE, UART_DATABIT_8, UART_STOPBIT_1}, 0,uUart0RxBuf,uUart0TxBuf,UART_RRB_SIZE_0,UART_SRB_SIZE_0,Uart0Io,Uart0Send,Uart0Rec};
#endif
#if (USE_UART1==1)
static USART_t sUart1 = {LPC_USART1, 0,0,0, 0,0,0, (UartHook_t)0,   (UartHook_t)0,   {9600, UART_PARITY_NONE, UART_DATABIT_8, UART_STOPBIT_1}, 0,uUart1RxBuf,uUart1TxBuf,UART_RRB_SIZE_1,UART_SRB_SIZE_1,Uart1Io,Uart1Send,Uart1Rec};
#endif
#if (USE_UART2==1)
static USART_t sUart2 = {LPC_USART2, 0,0,0, 0,0,0, (UartHook_t)0,   (UartHook_t)0,   {9600, UART_PARITY_NONE, UART_DATABIT_8, UART_STOPBIT_1}, 0,uUart2RxBuf,uUart2TxBuf,UART_RRB_SIZE_2,UART_SRB_SIZE_2,Uart2Io,Uart2Send,Uart2Rec};
#endif
#if (USE_UART3==1)
static USART_t sUart3 = {LPC_USART3, 0,0,0, 0,0,0, (UartHook_t)0,   (UartHook_t)0,   {9600, UART_PARITY_NONE, UART_DATABIT_8, UART_STOPBIT_1}, 0,uUart3RxBuf,uUart3TxBuf,UART_RRB_SIZE_3,UART_SRB_SIZE_3,Uart3Io,Uart3Send,Uart3Rec};
#endif
#if (USE_UART4==1)
static USART_t sUart4 = {LPC_USART4, 0,0,0, 0,0,0, (UartHook_t)0,   (UartHook_t)0,   {9600, UART_PARITY_NONE, UART_DATABIT_8, UART_STOPBIT_1}, 0,uUart4RxBuf,uUart4TxBuf,UART_RRB_SIZE_4,UART_SRB_SIZE_4,Uart4Io,Uart4Send,Uart4Rec};
#endif

static USART_t *USART_tab[] = {
    #if (USE_UART0==1)
    &sUart0,
    #else
    NULL,
    #endif

    #if (USE_UART1==1)
    &sUart1,
    #else
    NULL,
    #endif

    #if (USE_UART2==1)
    &sUart2,
    #else
    NULL,
    #endif

    #if (USE_UART3==1)
    &sUart3,
    #else
    NULL,
    #endif

    #if (USE_UART4==1)
    &sUart4,
    #else
    NULL,
    #endif
    };

//======================================================================================
/****************************************************************************
* 名    称： IntDelay()
* 功    能：短延时
* 入口参数：
*           uInDelay     输入延时时间，多少个
* 出口参数：无
* 范    例: 无
****************************************************************************/
void IntDelay(uint16_t uInDelay)
{
    // 10000约为2.71mS。单点约为0.271uS
    volatile uint16_t uDelay;

    uDelay = uInDelay;

    while(uDelay--)
    {
        if(LPC_USART4->STAT & (UARTN_STAT_RXRDY))   // 模块使用串口4，优先保障模块的接收中断
        {
            break;
        }
    }
}
//======================================================================================
static uint8_t TxFifoInMult(USART_t *p, const void *data,uint16_t bytes);
static uint8_t TxTifoOut(USART_t *p, uint8_t *data);
static uint8_t RxFifoOut(USART_t *p, uint32_t timeout, uint8_t *perro);
static uint8_t RxBufIsEmpty(USART_t *p);
//======================================================================================
//======================================================================================
// 获取接收缓冲区字节数量
/****************************************************************************
* 名    称：UartRxLen()
* 功    能：串口接收缓冲区数量。
* 入口参数：
*           uUartId:串口号
* 出口参数：出错返回 -1
*            正确返回 缓冲区数据数量
* 范    例: 无
****************************************************************************/
int16_t UartRxLen(uint8_t uUartId)
{
    if(uUartId>4 || NULL==USART_tab[uUartId])
    {
        return -1;
    }
    return USART_tab[uUartId]->uRxLen;
}
/****************************************************************************
* 名    称：UartTxLen()
* 功    能：串口接收缓冲区数量。
* 入口参数：
*           uUartId:串口号
* 出口参数：出错返回 -1
*            正确返回 缓冲区数据数量
* 范    例: 无
****************************************************************************/
int16_t UartTxLen(uint8_t uUartId)
{
    if(uUartId>4 || NULL==USART_tab[uUartId])
    {
        return -1;
    }
    return USART_tab[uUartId]->uTxLen;
}
/****************************************************************************
* 名    称：UartClear()
* 功    能：清空串口数据。
* 入口参数：
*           uUartId:串口号
* 出口参数：出错返回 -1
*            正确返回 0
* 范    例: 无
****************************************************************************/
int8_t UartClear(uint8_t uUartId)
{
    if(uUartId>4 || NULL==USART_tab[uUartId])
    {
        return -1;
    }
    USART_tab[uUartId]->uRxIn  = 0;
    USART_tab[uUartId]->uRxOut = 0;
    USART_tab[uUartId]->uRxLen = 0;
    USART_tab[uUartId]->uTxIn  = 0;
    USART_tab[uUartId]->uTxOut = 0;
    USART_tab[uUartId]->uTxLen = 0;

    return 0;
}
/****************************************************************************
* 名    称：UartClearSendBuffer()
* 功    能：清空串口发送数据。
* 入口参数：
*           uUartId:串口号
* 出口参数：出错返回 -1
*            正确返回 0
* 范    例: 无
****************************************************************************/
int8_t UartClearSendBuffer(uint8_t uUartId)
{
    if(uUartId>4 || NULL==USART_tab[uUartId])
    {
        return -1;
    }
    USART_tab[uUartId]->uTxIn  = 0;
    USART_tab[uUartId]->uTxOut = 0;
    USART_tab[uUartId]->uTxLen = 0;

    return 0;
}
/****************************************************************************
* 名    称：UartClearRecBuffer()
* 功    能：清空串口接收数据。
* 入口参数：
*           uUartId:串口号
* 出口参数：出错返回 -1
*            正确返回 0
* 范    例: 无
****************************************************************************/
int8_t UartClearRecBuffer(uint8_t uUartId)
{
    if(uUartId>4 || NULL==USART_tab[uUartId])
    {
        return -1;
    }
    USART_tab[uUartId]->uRxIn  = 0;
    USART_tab[uUartId]->uRxOut = 0;
    USART_tab[uUartId]->uRxLen = 0;

    return 0;
}
/****************************************************************************
* 名    称：UartWrite()
* 功    能：串口发送数据。
* 入口参数：
*           uUartId:串口号
*           data:数据数组地址
*           bytes:输出数据数量
* 出口参数：出错返回 -1
*            正确返回输出数量
* 范    例: 无
****************************************************************************/
int16_t UartWrite(uint8_t uUartId,const void *data,uint16_t bytes)
{
    uint8_t ch;
    volatile uint8_t uDelay;
    uint16_t ret;
    uint8_t *p8 = (uint8_t *) data;
    USART_t *pUsart;

    if(3 == uUartId)
    {
    	g_South_Action_Newtime = OSTimeGet();
    }

    if(0!=uUartId)
    {
        if(uUartId>4 || NULL==USART_tab[uUartId])
        {
            return -1;
        }
        pUsart = USART_tab[uUartId];

        if(pUsart->IoSend != 0)
        {
            (*(pUsart->IoSend))(); // 转入发送状态 uRegCount
            uDelay = 100;      //
            while(uDelay--);
        }

        Chip_UARTN_IntDisable(pUsart->uart, UARTN_INTEN_TXRDY);

        /* Move as much data as possible into transmit ring buffer */
        //ret = RingBuffer_InsertMult(pRB, p8, bytes);
        ret = TxFifoInMult(pUsart,p8,bytes);
        while ((Chip_UARTN_GetStatus(pUsart->uart) & UARTN_STAT_TXRDY) != 0 && TxTifoOut(pUsart, &ch))
        {
            Chip_UARTN_SendByte(pUsart->uart, ch);
        }

        /* Add additional data to transmit ring buffer if possible */
        //ret += RingBuffer_InsertMult(pRB, (p8 + ret), (bytes - ret));
        ret += TxFifoInMult(pUsart,(p8+ret),(bytes-ret));

        /* Enable UART transmit interrupt */
        Chip_UARTN_IntEnable(pUsart->uart, UARTN_INTEN_TXRDY);

        return ret;
    }
    else
    {
        #if (USE_UART0==1)

        if(sUart0.IoSend != 0)
        {
            sUart0.IoSend();
        }

        Chip_UART0_IntDisable(LPC_USART0, UART0_IER_THREINT);

        /* Move as much data as possible into transmit ring buffer */
        //ret = RingBuffer_InsertMult(pRB, p8, bytes);
        ret = TxFifoInMult(&sUart0,p8,bytes);
        while ((Chip_UART0_ReadLineStatus(LPC_USART0) & UART0_LSR_THRE) != 0 && TxTifoOut(&sUart0, &ch))
        {
            Chip_UART0_SendByte(LPC_USART0, ch);
        }

        /* Add additional data to transmit ring buffer if possible */
        //ret += RingBuffer_InsertMult(pRB, (p8 + ret), (bytes - ret));
        ret += TxFifoInMult(&sUart0,(p8+ret),(bytes-ret));

        /* Enable UART transmit interrupt */
        Chip_UART0_IntEnable(LPC_USART0, UART0_IER_THREINT);

        return ret;

        #endif
    }
}
// 接收第一个字节由传入的timeout 作为超时
// 如果200ms 没有收到下一字节，则超时返回
/****************************************************************************
* 名    称：UartRead()
* 功    能：串口读数据。
* 入口参数：
*           uUartId :串口号
*           pdata   :数据数组地址
*           uLen    :读数据数量
*           uTimeout:超时时间，多少个tick
* 出口参数：出错返回 -1
*            正确返回读取数量
* 范    例: 无
****************************************************************************/
int16_t UartRead(uint8_t uUartId, void *pdata, uint16_t uLen, uint16_t uTimeout)
{
    if(uUartId>4 || NULL==USART_tab[uUartId])
    {
        return -1;
    }

    uint16_t i = 0;
    uint16_t tmptimeout = uTimeout;
    uint8_t perro;
    uint8_t data;

    uint8_t *p8 = (uint8_t *) pdata;
    
    if(uTimeout)
    {
        while(RxBufIsEmpty(USART_tab[uUartId]))
        {
            OSTimeDly(5);  //原来为2  OS_TICKS_PER_SEC
        }
    }

    while (1)
    {
        if (uLen != 0)
        {
            data = RxFifoOut(USART_tab[uUartId], tmptimeout, &perro);
            if (perro == 0)
            {
                tmptimeout = uTimeout;

                *p8++ = data;
                uLen--;
                i++;
            }
            else
            {
				break;
            }
        }
        else
        {
            break;
        }
    }

    return i;
}
/****************************************************************************
* 名    称：UartShowBytes()
* 功    能：串口缓冲区显示数据，不从缓冲区读取。
* 入口参数：
*           uUartId :串口号
*           pdata   :数据数组地址
*           uLen    :读数据数量
* 出口参数：出错返回 -1
*            正确返回读取数量
* 范    例: 无
****************************************************************************/
int16_t UartShowBytes(uint8_t uUartId, uint8_t *pdata, uint16_t uLen)
{
    uint32_t cpu_sr;
    uint16_t uRxLenTmp;
    uint16_t uRxOutTmp;
    uint16_t i;
    USART_t *p;

    p = USART_tab[uUartId];
    if (uUartId > 4 || NULL==p)
    {
        return -1;
    }

    uRxLenTmp = p->uRxLen;
    uRxOutTmp = p->uRxOut;

    OS_ENTER_CRITICAL();
    while(uLen && uRxLenTmp)
    {
        *pdata++ = p->RxBuf[uRxOutTmp++];

        uRxLenTmp--;
        uLen--;
        i++;
    }
    OS_EXIT_CRITICAL();

    return i;
}
//======================================================================================
//======================================================================================
//======================================================================================
//======================================================================================
//======================================================================================

//======================================================================================
// 判断接收缓冲区是否为空，空返回1，非空返回0
static uint8_t RxBufIsEmpty(USART_t *p)
{
    return (p->uRxLen?0:1);
}
// 判断发送缓冲区是否为空，空返回1，非空返回0
static uint8_t TxBufIsEmpty(USART_t *p)
{
    return (p->uTxLen?0:1);
}
// 写入数据到接收缓冲区，中断调用
static void RxFifoIn(USART_t *p, uint8_t data)
{
    if (p->uRxLen < p->uRxBufSize)
    {
        // not full yet.
        p->RxBuf[p->uRxIn++] = data;
        p->uRxIn %= p->uRxBufSize;
        p->uRxLen++;
    }
}
// 读取接收缓冲区
static uint8_t RxFifoOut(USART_t *p, uint32_t timeout, uint8_t *perro)
{
    uint32_t cpu_sr;
    uint8_t data = 0;

    if (timeout == 0)
    {
        while(p->uRxLen == 0)
        {
            OSTimeDly(1);
        }
    }
    else
    {
        while (timeout != 0)
        {
            if (p->uRxLen == 0)
            {
                timeout--;
                OSTimeDly(1);
            }
            else
            {
                break;
            }
        }

        if (timeout == 0)
        {
            *perro = 1;
            return 0;
        }
    }

    OS_ENTER_CRITICAL();
    if (p->uRxLen != 0)
    {
        p->uRxLen--;
        data = p->RxBuf[p->uRxOut++];
        p->uRxOut %= p->uRxBufSize;//sizeof(p->RxBuf);  // /sizeof(p->RxBuf[0])
    }
    OS_EXIT_CRITICAL();

    *perro = 0;
    return data;
}

// 数据写入发送缓冲区
/*static uint8_t TxFifoIn(USART_t *p, uint8_t data)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();

    if (p->uTxLen < p->uTxBufSize)  ///sizeof(p->TxBuf[0])
    {
        // buf not full yet.
        p->TxBuf[p->uTxIn++] = data;
        p->uTxIn %= p->uTxBufSize;  ///sizeof(p->TxBuf[0])
        p->uTxLen++;
        OS_EXIT_CRITICAL();
        return 0;
    }
    else
    {
        OS_EXIT_CRITICAL();
        return 1;
    }
}*/
// 多字节数据写入发送缓冲区
static uint8_t TxFifoInMult(USART_t *p, const void *data,uint16_t bytes)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    uint16_t ret=0;
    uint8_t *p8 = (uint8_t *) data;

    while(0!=bytes)
    {
        if (p->uTxLen < p->uTxBufSize)  ///sizeof(p->TxBuf[0])
        {
            // buf not full yet.
            p->TxBuf[p->uTxIn++] = *p8;
            p->uTxIn %= p->uTxBufSize;  ///sizeof(p->TxBuf[0])
            p->uTxLen++;

            bytes--;
            p8++;
            ret++;
        }
        else
        {
            OS_EXIT_CRITICAL();
            return ret;
        }
    }

    OS_EXIT_CRITICAL();
    return ret;
}

// 从发送缓冲区读取数据，发送中断调用
static uint8_t TxTifoOut(USART_t *p, uint8_t *data)
{
    //uint8_t data = 0;

    if (p->uTxLen != 0)
    {
        if(p->tx_hook != 0)
        {
            (*(p->tx_hook))();
        }

        p->uTxLen--;
        *data = p->TxBuf[p->uTxOut++];
        p->uTxOut %= p->uTxBufSize;//sizeof(p->TxBuf);  ///sizeof(p->TxBuf[0])
        return 1;
    }
    else
    {
        return 0;
    }
}

//======================================================================================
// 串口0初始化
static void Uart0Init(uint32_t baud)
{
    #if (USE_UART0==1)
    // 引脚初始化
    UART0_RX_PIN_INIT();
	UART0_TX_PIN_INIT();


    /* Setup UART for 115.2K8N1 */
	Chip_UART0_Init(LPC_USART0);
	Chip_UART0_SetBaud(LPC_USART0, baud);
	Chip_UART0_ConfigData(LPC_USART0, (UART0_LCR_WLEN8 | UART0_LCR_SBS_1BIT));
	Chip_UART0_SetupFIFOS(LPC_USART0, (UART0_FCR_FIFO_EN | UART0_FCR_TRG_LEV2));
	Chip_UART0_TXEnable(LPC_USART0);

	/* Enable receive data and line status interrupt */
	Chip_UART0_IntEnable(LPC_USART0, UART0_IER_RBRINT);//(UART0_IER_RBRINT | UART0_IER_THREINT | UART0_IER_RLSINT));

	/* Enable UART 0 interrupt */
	NVIC_EnableIRQ(USART0_IRQn);
    #endif
}
/****************************************************************************
* 名    称：UartInit()
* 功    能：串口初始化。
* 入口参数：
*           uUartId :串口号
*           baud    :串口波特率
*           eParity :校验
* 出口参数：出错返回 -1
*            正确返回 0
* 范    例: 无
****************************************************************************/
int8_t UartInit(uint8_t uUartId,uint32_t baud,UART_PARITY_Type eParity)
{
    if(uUartId>4 || NULL==USART_tab[uUartId])
    {
        return -1;
    }

    USART_t *pUsart;
    uint8_t uParity;

    switch(uUartId)
    {
    case uart0:
        Uart0Init(baud);
        //pUsart = &sUart0;

        return 0;
        break;
    case uart1:
        pUsart = USART_tab[uUartId];
        if(NULL==pUsart)
        {
            break;
        }

        UART1_RX_PIN_INIT();//RXD
        UART1_TX_PIN_INIT();//TXD

        break;
    case uart2:
        pUsart = USART_tab[uUartId];
        if(NULL==pUsart)
        {
            break;
        }

        UART2_RX_PIN_INIT(); // RXD
        UART2_TX_PIN_INIT(); // TXD

        break;
    case uart3:
        pUsart = USART_tab[uUartId];
        if(NULL==pUsart)
        {
            break;
        }

        UART3_RX_PIN_INIT();  // RXD
        UART3_TX_PIN_INIT();  // TXD

        break;
    case uart4:
        pUsart = USART_tab[uUartId];
        if(NULL==pUsart)
        {
            break;
        }

        UART4_RX_PIN_INIT();
        UART4_TX_PIN_INIT();

        break;
    default:
        return-1;
        break;
    }


    if(pUsart->IoInit!= 0)
    {
        (*(pUsart->IoInit))(); // IO口初始化
    }
    if(pUsart->IoSend!= 0)
    {
        (*(pUsart->IoRec))(); // 转入接收状态
    }


    #if defined(USE_INTEGER_CLOCK)
	/* Use 256x expected UART baud rate for integer mode to reduce
	   the clock error at higher baud rates. */
	Chip_Clock_SetUSARTNBaseClockRate((115200 * 256), false);

    #else
	/* Use 48x expected UART baud rate for fractional baud mode. (Lower limit
	   is 16x.) */
	Chip_Clock_SetUSARTNBaseClockRate((115200 * 48), true);
    #endif

    switch(eParity)
    {
    default:
    case UART_PARITY_NONE:
        uParity = UARTN_CFG_PARITY_NONE;
        break;

    case UART_PARITY_EVEN:
        uParity = UARTN_CFG_PARITY_EVEN;
        break;

    case UART_PARITY_ODD:
        uParity = UARTN_CFG_PARITY_ODD;
        break;
    }
    /* Setup UART */
	Chip_UARTN_Init(pUsart->uart);
	Chip_UARTN_ConfigData(pUsart->uart, UARTN_CFG_DATALEN_8 | uParity | UARTN_CFG_STOPLEN_1);
	Chip_UARTN_SetBaud(pUsart->uart, baud);

	/* Optional for low clock rates only: Chip_UARTN_SetBaudWithRTC32K(LPC_USART, 300); */
	Chip_UARTN_Enable(pUsart->uart);
	Chip_UARTN_TXEnable(pUsart->uart);


	/* Enable receive data and line status interrupt */
	Chip_UARTN_IntEnable(pUsart->uart, UARTN_INTEN_RXRDY);
	Chip_UARTN_IntDisable(pUsart->uart, UARTN_INTEN_TXRDY);	/* May not be needed */

	/* Enable UART interrupt */
    switch(uUartId)
    {
    case uart1:
    case uart4:
        NVIC_EnableIRQ(USART1_4_IRQn);
        break;
    case uart2:
    case uart3:
        NVIC_EnableIRQ(USART2_3_IRQn);
        break;
    default:
        break;
    }
    //Chip_UARTN_SendBlocking(pUART, inst3, sizeof(inst3) - 1);
	//Chip_UARTN_SendRB(pUART, txring_temp, inst2, sizeof(inst2) - 1);

    return 0;
}
/****************************************************************************
* 名    称：UartDeinit()
* 功    能：串口反初始化。
* 入口参数：
*           uUartId :串口号
* 出口参数：无
* 范    例: 无
****************************************************************************/
void UartDeinit(uint8_t uUartId)
{
    switch(uUartId)
    {
    case uart0:
        NVIC_DisableIRQ(USART0_IRQn);
        Chip_UART0_DeInit(LPC_USART0);
        return;
        break;
    case uart1:
        NVIC_DisableIRQ(USART1_4_IRQn);
        Chip_UARTN_DeInit(LPC_USART1);
        break;
    case uart2:
        NVIC_DisableIRQ(USART2_3_IRQn);
        Chip_UARTN_DeInit(LPC_USART2);
        break;
    case uart3:
        NVIC_DisableIRQ(USART2_3_IRQn);
        Chip_UARTN_DeInit(LPC_USART3);
        break;
    case uart4:
        NVIC_DisableIRQ(USART1_4_IRQn);
        Chip_UARTN_DeInit(LPC_USART4);
        break;
    case uart_all:
        NVIC_DisableIRQ(USART0_IRQn);
        NVIC_DisableIRQ(USART1_4_IRQn);
        NVIC_DisableIRQ(USART2_3_IRQn);

        Chip_UART0_DeInit(LPC_USART0);
        Chip_UARTN_DeInit(LPC_USART1);
        Chip_UARTN_DeInit(LPC_USART2);
        Chip_UARTN_DeInit(LPC_USART3);
        Chip_UARTN_DeInit(LPC_USART4);
        break;
    default:
        break;
    }
}
//======================================================================================
//======================================================================================
/*void UsartIsr(USART_t *p)
{
    uint8_t ch;
    volatile uint8_t uDelay;

    if(p->uart->STAT & (UARTN_STAT_RXRDY))//接收中断
    {
        while ((Chip_UARTN_GetStatus(p->uart) & UARTN_STAT_RXRDY) != 0)
        {
            ch = Chip_UARTN_ReadByte(p->uart);
            RxFifoIn(p, ch);
        }
    }

    if((p->uart->INTENSET & UARTN_INTEN_TXRDY) && (p->uart->STAT & (UARTN_STAT_TXRDY)))// 发送中断
    {
        if (TxBufIsEmpty(p) && (p->uart->STAT&UARTN_STAT_TXIDLE))  // 缓冲区空且发送空闲
        {
            Chip_UARTN_IntDisable(p->uart, UARTN_INTEN_TXRDY);

            uDelay = 250;
            while(uDelay--);
            (*(p->IoRec))(); // 转入接收状态

            if(LPC_USART3==p->uart)
            {
                if(g_uTest<5)
                {
                    g_uTest++;
                }
                g_uTest = 0;
            }
        }
        else
        {
            // Fill FIFO until full or until TX ring buffer is empty
            while (((Chip_UARTN_GetStatus(p->uart) & UARTN_STAT_TXRDY) != 0) &&
            TxTifoOut(p, &ch))
            {
                Chip_UARTN_SendByte(p->uart, ch);
                g_uTest++;
            }
        }
    }
}
*/
#if (USE_UART1==1)
void Usart1Isr(void)
{
    uint8_t ch;
    volatile uint16_t uDelay;

    if(sUart1.uart->STAT & (UARTN_STAT_RXRDY))//接收中断
    {
        while ((Chip_UARTN_GetStatus(sUart1.uart) & UARTN_STAT_RXRDY) != 0)
        {
            ch = Chip_UARTN_ReadByte(sUart1.uart);
            RxFifoIn(&sUart1, ch);
        }
    }

    if((sUart1.uart->INTENSET & UARTN_INTEN_TXRDY) && (sUart1.uart->STAT & (UARTN_STAT_TXRDY)))// 发送中断
    {
        if (TxBufIsEmpty(&sUart1) && (sUart1.uart->STAT&UARTN_STAT_TXIDLE))  // 缓冲区空且发送空闲
        {
            Chip_UARTN_IntDisable(sUart1.uart, UARTN_INTEN_TXRDY);

            IntDelay(1500);
            (*(sUart1.IoRec))(); // 转入接收状态
        }
        else
        {
            // Fill FIFO until full or until TX ring buffer is empty
            while (((Chip_UARTN_GetStatus(sUart1.uart) & UARTN_STAT_TXRDY) != 0) &&
            TxTifoOut(&sUart1, &ch))
            {
                Chip_UARTN_SendByte(sUart1.uart, ch);
            }
        }
    }
}
#endif
#if (USE_UART2==1)
void Usart2Isr(void)
{
    uint8_t ch;

    if(sUart2.uart->STAT & (UARTN_STAT_RXRDY))//接收中断
    {
        while ((Chip_UARTN_GetStatus(sUart2.uart) & UARTN_STAT_RXRDY) != 0)
        {
            ch = Chip_UARTN_ReadByte(sUart2.uart);
            RxFifoIn(&sUart2, ch);
        }
    }

    if((sUart2.uart->INTENSET & UARTN_INTEN_TXRDY) && (sUart2.uart->STAT & (UARTN_STAT_TXRDY)))// 发送中断
    {
        if (TxBufIsEmpty(&sUart2) && (sUart2.uart->STAT&UARTN_STAT_TXIDLE))  // 缓冲区空且发送空闲
        {
            Chip_UARTN_IntDisable(sUart2.uart, UARTN_INTEN_TXRDY);

            IntDelay(1500);
            (*(sUart2.IoRec))(); // 转入接收状态
        }
        else
        {
            // Fill FIFO until full or until TX ring buffer is empty
            while (((Chip_UARTN_GetStatus(sUart2.uart) & UARTN_STAT_TXRDY) != 0) &&
            TxTifoOut(&sUart2, &ch))
            {
                Chip_UARTN_SendByte(sUart2.uart, ch);
            }
        }
    }
}
#endif
#if (USE_UART3==1)
void Usart3Isr(void)
{
    uint8_t ch;

    if(sUart3.uart->STAT & (UARTN_STAT_RXRDY))//接收中断
    {
        while ((Chip_UARTN_GetStatus(sUart3.uart) & UARTN_STAT_RXRDY) != 0)
        {
            ch = Chip_UARTN_ReadByte(sUart3.uart);
            RxFifoIn(&sUart3, ch);
        }
    }

    if((sUart3.uart->INTENSET & UARTN_INTEN_TXRDY) && (sUart3.uart->STAT & (UARTN_STAT_TXRDY)))// 发送中断
    {
        if (TxBufIsEmpty(&sUart3) && (sUart3.uart->STAT&UARTN_STAT_TXIDLE))  // 缓冲区空且发送空闲
        {
            Chip_UARTN_IntDisable(sUart3.uart, UARTN_INTEN_TXRDY);

            IntDelay(1500);
            (*(sUart3.IoRec))(); // 转入接收状态
        }
        else
        {
            // Fill FIFO until full or until TX ring buffer is empty
            while (((Chip_UARTN_GetStatus(sUart3.uart) & UARTN_STAT_TXRDY) != 0) &&
            TxTifoOut(&sUart3, &ch))
            {
                Chip_UARTN_SendByte(sUart3.uart, ch);
            }
        }
    }
}
#endif
#if (USE_UART4==1)
void Usart4Isr(void)
{
    uint8_t ch;

    if(sUart4.uart->STAT & (UARTN_STAT_RXRDY))//接收中断
    {
        while ((Chip_UARTN_GetStatus(sUart4.uart) & UARTN_STAT_RXRDY) != 0)
        {
            ch = Chip_UARTN_ReadByte(sUart4.uart);
            RxFifoIn(&sUart4, ch);
        }
    }

    if((sUart4.uart->INTENSET & UARTN_INTEN_TXRDY) && (sUart4.uart->STAT & (UARTN_STAT_TXRDY)))// 发送中断
    {
        if (TxBufIsEmpty(&sUart4) && (sUart4.uart->STAT&UARTN_STAT_TXIDLE))  // 缓冲区空且发送空闲
        {
            Chip_UARTN_IntDisable(sUart4.uart, UARTN_INTEN_TXRDY);

            (*(sUart4.IoRec))(); // 转入接收状态
        }
        else
        {
            // Fill FIFO until full or until TX ring buffer is empty
            while (((Chip_UARTN_GetStatus(sUart4.uart) & UARTN_STAT_TXRDY) != 0) &&
            TxTifoOut(&sUart4, &ch))
            {
                Chip_UARTN_SendByte(sUart4.uart, ch);
            }
        }
    }
}
#endif
//======================================================================================
//======================================================================================
void USART0_IRQHandler(void)
{
    #if (USE_UART0==1)
    uint8_t ch;

    /*OS_CPU_SR  cpu_sr;
    OS_ENTER_CRITICAL();
    OSIntNesting++;
    OS_EXIT_CRITICAL();
    */
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */
	//Chip_UART0_IRQRBHandler(LPC_USART0, &rxring, &txring);

    /* Handle transmit interrupt if enabled */
    if (LPC_USART0->IER & UART0_IER_THREINT)
    {
        //Chip_UART0_TXIntHandlerRB(LPC_USART0, &txring);

        while ((Chip_UART0_ReadLineStatus(LPC_USART0) & UART0_LSR_THRE) != 0 && TxTifoOut(&sUart0, &ch))
        {
            Chip_UART0_SendByte(LPC_USART0, ch);
        }


        // Disable transmit interrupt if the ring buffer is empty
        if (TxBufIsEmpty(&sUart0))
        {
            Chip_UART0_IntDisable(LPC_USART0, UART0_IER_THREINT);
        }
    }
    /* Handle receive interrupt */
    /* New data will be ignored if data not popped in time */
    while (Chip_UART0_ReadLineStatus(LPC_USART0) & UART0_LSR_RDR)  //
    {
        ch = Chip_UART0_ReadByte(LPC_USART0);

        RxFifoIn(&sUart0,ch);

        //UART0_STATE.Rec_start = 1;
    }

    //OSIntExit();
    #endif
}

void HardFault_Handler(void)
{
    printf("HardFault\n");
    while(1);
}
void USART1_4_IRQHandler(void)
{
/*    OS_CPU_SR  cpu_sr;
    OS_ENTER_CRITICAL();
    OSIntNesting++;
    OS_EXIT_CRITICAL();
*/
    #if (USE_UART1==1)
    Usart1Isr();
    #endif

    #if (USE_UART4==1)
    Usart4Isr();
    #endif

    //OSIntExit();
}

void USART2_3_IRQHandler(void)
{
/*    OS_CPU_SR  cpu_sr;
    OS_ENTER_CRITICAL();
    OSIntNesting++;
    OS_EXIT_CRITICAL();
*/
    #if (USE_UART2==1)
    Usart2Isr();
    #endif

    #if (USE_UART3==1)
    Usart3Isr();
    #endif

    //OSIntExit();
}
//======================================================================================
