/*
 * @brief SSP example
 * This example show how to use the SSP in 2 modes : Polling and Interrupt.
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "SPI.h"
#include "chip.h"
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define LOOPBACK_TEST       1
#define SSP_MODE_TEST       1	/*1: Master, 0: Slave */
#define POLLING_MODE        1
#if POLLING_MODE
#define INTERRUPT_MODE      0
#else
#define INTERRUPT_MODE      1
#endif
#define BUFFER_SIZE                         (0x100)
#define SSP_DATA_BITS                       (SSP_BITS_8)
#define SSP_DATA_BIT_NUM(databits)          (databits + 1)
#define SSP_DATA_BYTES(databits)            (((databits) > SSP_BITS_8) ? 2 : 1)
#define SSP_LO_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? 0xFF : (0xFF >> \
																					  (8 - SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? (0xFF >> \

/******************************************************************************/
// SSP Polling Write in blocking mode
uint16_t SSP_WriteFrames_Blocking(LPC_SSP_T *pSSP, uint8_t *buffer, uint16_t buffer_len)
{
    uint16_t tx_cnt = 0, rx_cnt = 0;//
    uint8_t *wdata8;

    wdata8 = buffer;

    // Clear all remaining frames in RX FIFO
    while (Chip_SSP_GetStatus(pSSP, SSP_STAT_RNE))
    {
        Chip_SSP_ReceiveFrame(pSSP);
    }


    // Clear status
    Chip_SSP_ClearIntPending(pSSP, SSP_INT_CLEAR_BITMASK);


    while (tx_cnt < buffer_len || rx_cnt < buffer_len)//
    {
        // write data to buffer
        if ((Chip_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && (tx_cnt < buffer_len))
        {
            Chip_SSP_SendFrame(pSSP, *wdata8);
            wdata8++;
            tx_cnt++;
        }

        // Check overrun error
        if (Chip_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET)
        {
            return ERROR;
        }

        // Check for any data available in RX FIFO
        while (Chip_SSP_GetStatus(pSSP, SSP_STAT_RNE) == SET && rx_cnt < buffer_len)
        {
            Chip_SSP_ReceiveFrame(pSSP);	// read dummy data
            rx_cnt++;
        }
    }

    return tx_cnt;
}

/******************************************************************************/
// SSP Polling Read in blocking mode
uint16_t SSP_ReadFrames_Blocking(LPC_SSP_T *pSSP, uint8_t *buffer, uint16_t buffer_len)
{
    uint16_t rx_cnt = 0, tx_cnt = 0;//
    uint8_t *rdata8;

    rdata8 = buffer;

    // Clear all remaining frames in RX FIFO
    while (Chip_SSP_GetStatus(pSSP, SSP_STAT_RNE))
    {
        Chip_SSP_ReceiveFrame(pSSP);
    }

    // Clear status
    Chip_SSP_ClearIntPending(pSSP, SSP_INT_CLEAR_BITMASK);


    while (tx_cnt < buffer_len ||rx_cnt < buffer_len)//
    {
        // write data to buffer
        if ((Chip_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && (tx_cnt < buffer_len))
        {
            Chip_SSP_SendFrame(pSSP, 0xFF);	// just send dummy data
            tx_cnt++;
        }

        // Check overrun error
        if (Chip_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET)
        {
            return ERROR;
        }

        // Check for any data available in RX FIFO
        while (Chip_SSP_GetStatus(pSSP, SSP_STAT_RNE) == SET && rx_cnt < buffer_len)
        {
            *rdata8 = Chip_SSP_ReceiveFrame(pSSP);
            rdata8++;
            rx_cnt++;
        }
    }

    return rx_cnt;
}

/******************************************************************************/
void SPI_Init(uint8_t sp)
{
    LPC_SSP_T *pSSP;
    if(SPI0==sp)
    {
        pSSP = LPC_SSP0;
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 8, 0x91); // MISO P58
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 9, 0x91); // MOSI P59
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 7, 0x91); // SCLK P
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 29, 0x91); // SCLK P63
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, 0x92);  // SCLK P44

        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP0);
        Chip_Clock_SetSSP0ClockDiv(1);//Chip_SSP_SetSSPClkDivider(pSSP, 1);
        Chip_SYSCTL_PeriphReset(RESET_SSP0);

        //Chip_SSP_Set_Mode(pSSP, SSP_MODE_MASTER);
        //Chip_SSP_SetFormat(pSSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);
        Chip_SSP_SetBitRate(pSSP, 1000000);
    }
    else if(SPI1==sp)
    {
        pSSP = LPC_SSP1;
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 21, 0x92); // MOSI P33
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 20, 0x92); // SCK  P29
        //Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 21, 0x92); // MISO P56

        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP1);
        Chip_Clock_SetSSP1ClockDiv(1);//Chip_SSP_SetSSPClkDivider(pSSP, 1);
        Chip_SYSCTL_PeriphReset(RESET_SSP1);

        //Chip_SSP_Set_Mode(pSSP, SSP_MODE_MASTER);
        //Chip_SSP_SetFormat(pSSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);
        Chip_SSP_SetBitRate(pSSP, 2000000);
    }
    else
    {
        return;
    }

    //Chip_SSP_Init(pSSP);

    //--
    Chip_SSP_SetFormat(pSSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
	Chip_SSP_SetMaster(pSSP, SSP_MODE_TEST);
	Chip_SSP_Enable(pSSP);
}

/******************************************************************************/
uint16_t SPI_Send(uint8_t sp,uint8_t *data,uint16_t bytes)
{
    LPC_SSP_T *pSSP;

    if(SPI0==sp)
    {
        pSSP = LPC_SSP0;
    }
    else if(SPI1==sp)
    {
        pSSP = LPC_SSP1;
    }
    else
    {
        return 0;
    }
    return (SSP_WriteFrames_Blocking(pSSP, data, bytes));
}

/******************************************************************************/
uint16_t SPI_Read(uint8_t sp,uint8_t *data,uint16_t bytes)
{
    LPC_SSP_T *pSSP;

    if(SPI0==sp)
    {
        pSSP = LPC_SSP0;
    }
    else if(SPI1==sp)
    {
        pSSP = LPC_SSP1;
    }
    else
    {
        return 0;
    }
    return (SSP_ReadFrames_Blocking(pSSP, data, bytes));
}

/******************************************************************************/
void SPI_Deinit(uint8_t sp)
{
    LPC_SSP_T *pSSP;

    if(SPI0==sp)
    {
        pSSP = LPC_SSP0;
    }
    else if(SPI1==sp)
    {
        pSSP = LPC_SSP1;
    }
    else
    {
        return;
    }
    Chip_SSP_DeInit(pSSP);
}
