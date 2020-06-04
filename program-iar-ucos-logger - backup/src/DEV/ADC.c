#include "ADC.h"
/*
 * @brief LPC11u6x ADC example
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

#include "chip.h"//"board.h"//
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

//static volatile int ticks;
static bool sequenceComplete;//, thresholdCrossed;


#define ADC_CH 0

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from ADC sequencer A
 * @return	Nothing
 */
void ADCA_IRQHandler(void)
{
    uint32_t pending;

    /* Get pending interrupts */
    pending = Chip_ADC_GetFlags(LPC_ADC);

    /* Sequence A completion interrupt */
    if (pending & ADC_FLAGS_SEQA_INT_MASK)
    {
        sequenceComplete = true;
    }

    /* Threshold crossing interrupt on ADC input channel */
    /*if (pending & ADC_FLAGS_THCMP_MASK(ADC_CH))
    {
        thresholdCrossed = true;
    }*/

    /* Clear any pending interrupts */
    Chip_ADC_ClearFlags(LPC_ADC, pending);
}

void ADC_start(void)
{
    /* Manual start for ADC conversion sequence A */
    Chip_ADC_StartSequencer(LPC_ADC, ADC_SEQA_IDX);
}

int16_t ADC_read(void)
{
    uint32_t rawSample;
    int16_t result;

    if (sequenceComplete)
    {
        sequenceComplete = false;

        rawSample = Chip_ADC_GetDataReg(LPC_ADC, ADC_CH);
        /* Show some ADC data */
        if ((rawSample & (ADC_DR_OVERRUN | ADC_SEQ_GDAT_DATAVALID)) != 0)
        {
            result = ADC_DR_RESULT(rawSample);
        }
        else
        {
            result = -1;
        }

        //ADC_start();
    }
    else
    {
        result = -2;
    }

    return result;
}

int ADC_Init(void)
{
    // ADC input 0 is on PIO0_12 mapped to FUNC2
	//Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));

	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 9, (IOCON_FUNC3 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // 1,9


    // Setup ADC for 12-bit mode and normal power
	Chip_ADC_Init(LPC_ADC, 0);

	// Need to do a calibration after initialization and trim
	Chip_ADC_StartCalibration(LPC_ADC);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC))) {}

	// Setup for maximum ADC clock rate using sycnchronous clocking
	Chip_ADC_SetClockRate(LPC_ADC, ADC_MAX_SAMPLE_RATE);

	// Setup a sequencer to do the following:  Perform ADC conversion of ADC channels 0 only
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX,
		(ADC_SEQ_CTRL_CHANSEL(ADC_CH) | ADC_SEQ_CTRL_MODE_EOS));

	// Use higher voltage trim
	Chip_ADC_SetTrim(LPC_ADC, ADC_TRIM_VRANGE_HIGHV);//ADC_TRIM_VRANGE_HIGHV

	// Setup threshold 0 low and high values to about 25% and 75% of max
	//Chip_ADC_SetThrLowValue(LPC_ADC, 0, ((1 * 0xFFF) / 4));
	//Chip_ADC_SetThrHighValue(LPC_ADC, 0, ((3 * 0xFFF) / 4));

	// Clear all pending interrupts
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

	// Enable ADC overrun and sequence A completion interrupts
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));

	// Use threshold 0 for ADC channel and enable threshold interrupt mode for channel as crossing
	//Chip_ADC_SelectTH0Channels(LPC_ADC, ADC_THRSEL_CHAN_SEL_THR1(0));  // demo板使用ADC8
	//Chip_ADC_SetThresholdInt(LPC_ADC, 0, ADC_INTEN_THCMP_CROSSING);

	//Enable ADC NVIC interrupt
	NVIC_EnableIRQ(ADC_A_IRQn);

	// Enable sequencer
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);

    return 0;
}

/*
void ADC_test(void)
{
    static uint32_t rawSample;
	int j;

    if (thresholdCrossed)
    {
        thresholdCrossed = false;
        //DEBUGSTR("********ADC threshold event********\r\n");
    }

    // Is a conversion sequence complete?
    if (sequenceComplete)
    {
        sequenceComplete = false;

        // Get raw sample data for channels 0-11
        for (j = 0; j < 12; j++)
        {
            rawSample = Chip_ADC_GetDataReg(LPC_ADC, j);

            // Show some ADC data
            if ((rawSample & (ADC_DR_OVERRUN | ADC_SEQ_GDAT_DATAVALID)) != 0)
            {
                // 以下需要“board.h”
                //DEBUGOUT("Sample value = 0x%x (Data sample %d)\r\n", ADC_DR_RESULT(rawSample), j);
                //DEBUGOUT("Threshold range = 0x%x\r\n", ADC_DR_THCMPRANGE(rawSample));
                //DEBUGOUT("Threshold cross = 0x%x\r\n", ADC_DR_THCMPCROSS(rawSample));
            }
        }

        //DEBUGOUT("Overrun    = %d\r\n", ((rawSample & ADC_DR_OVERRUN) != 0));
        //DEBUGOUT("Data valid = %d\r\n", ((rawSample & ADC_SEQ_GDAT_DATAVALID) != 0));
        //DEBUGSTR("\r\n");
    }
}
*/
