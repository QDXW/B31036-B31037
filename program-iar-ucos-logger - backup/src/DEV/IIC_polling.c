
/*
 * @brief I2CM bus master example using polling mode
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

#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* I2CM transfer record */
static I2CM_XFER_T  i2cmXferRec;

/* 1Mbps I2C bit-rate */
#define I2C_BITRATE         (10000)

#define I2C_RD_CMD_BIT      (0x01)

#define I2C_FASTPLUS_BIT    (0)
#if (I2C_DEFAULT_SPEED > SPEED_400KHZ)
#undef  I2C_FASTPLUS_BIT
#define I2C_FASTPLUS_BIT IOCON_FASTI2C_EN
#endif

/* Current state for LED control via I2C cases */
//static volatile int state;

//static volatile int intErrCode;


static void SetupXferRecAndExecute(uint8_t devAddr,
								   uint8_t *txBuffPtr,
								   uint16_t txSize,
								   uint8_t *rxBuffPtr,
								   uint16_t rxSize)
{
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = devAddr;
	i2cmXferRec.options = 0;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = txSize;
	i2cmXferRec.rxSz = rxSize;
	i2cmXferRec.txBuff = txBuffPtr;
	i2cmXferRec.rxBuff = rxBuffPtr;
	Chip_I2CM_XferBlocking(LPC_I2C0, &i2cmXferRec);
}
//================================================================================
void IIC_Init(void)
{
    //uint8_t errcode;

    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, IOCON_FUNC1 | I2C_FASTPLUS_BIT);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, IOCON_FUNC1 | I2C_FASTPLUS_BIT);

    /* Enable I2C clock and reset I2C peripheral - the boot ROM does not
	   do this */
	Chip_I2CM_Init(LPC_I2C0);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C0, I2C_BITRATE);

    /* Disable the interrupt for the I2C */
	NVIC_DisableIRQ(I2C0_IRQn);
}

uint8_t IIC_read(uint8_t AddressI2C,uint8_t *buff,uint8_t len)
{
    uint8_t rx_buffer[3],i;
	//uint8_t tx_buffer[2];

	//tx_buffer[0] = 0xD0;
	//tx_buffer[1] = 0;
    //SetupXferRecAndExecute(AddressI2C, tx_buffer, 1, rx_buffer, 2);
    SetupXferRecAndExecute(AddressI2C, NULL, 0, rx_buffer, len);

    /* Test for valid operation */
    if (i2cmXferRec.status == I2CM_STATUS_OK)
    {
        /* Note results are only valid when there are no errors */
        //*buff++ = rx_buffer[0];
        //*buff++ = rx_buffer[1];

        for(i=0;i<len;i++)
        {
            buff[i] = rx_buffer[i];
        }
    }

    return i2cmXferRec.status;
}

uint8_t IIC_write(uint8_t AddressI2C,uint8_t *buff,uint8_t len)
{
    //uint8_t tx_buffer[3];

    SetupXferRecAndExecute(AddressI2C, buff, len, NULL, 0);

    return i2cmXferRec.status;
}
