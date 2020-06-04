


#include "InTIMER.h"
#include "GPIO.h"
#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ1 (10)/* 1 ticks per second */
#define TICKRATE_HZ2 (10)/* 2 ticks per second */
#define PRESCALE_HZ2 (0xFFFF)	/* 16-bit prescale count */

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
 * @brief	Handle interrupt from 32-bit timer 0
 * @return	Nothing
 */
void TIMER32_1_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER32_1, 1)) 
    {
		Chip_TIMER_ClearMatch(LPC_TIMER32_1, 1);
		//GPIO_ToggleBit(2,5);
	}
}

/**
 * @brief	Handle interrupt from 16-bit timer 0
 * @return	Nothing
 */
void TIMER16_0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER16_0, 1)) 
    {
		Chip_TIMER_ClearMatch(LPC_TIMER16_0, 1);
		//GPIO_ToggleBit(2,10);
	}
}


void InTIMER_Init(void)
{
    uint32_t timerFreq;


	/* Initialize 32-bit timer 1 clock */
	Chip_TIMER_Init(LPC_TIMER32_1);

	/* Initialize 16-bit timer 0 clock */
	Chip_TIMER_Init(LPC_TIMER16_0);

	/* Timer rate is system clock rate */
	timerFreq = Chip_Clock_GetSystemClockRate();

	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER32_1);
	Chip_TIMER_Reset(LPC_TIMER16_0);

	/* Enable both timers to generate interrupts when time matches */
	Chip_TIMER_MatchEnableInt(LPC_TIMER32_1, 1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER16_0, 1);

	/* Setup prescale value on 16-bit timer to extend range */
	Chip_TIMER_PrescaleSet(LPC_TIMER16_0, PRESCALE_HZ2);

	/* Setup 32-bit timer's duration (32-bit match time) */
	Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, (timerFreq / TICKRATE_HZ1));

	/* Setup 16-bit timer's duration (16-bit match time) */
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 1, (timerFreq / TICKRATE_HZ2) >> 16);

	/* Setup both timers to restart when match occurs */
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER32_1, 1);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_0, 1);

	/* Start both timers */
	Chip_TIMER_Enable(LPC_TIMER32_1);
	Chip_TIMER_Enable(LPC_TIMER16_0);

	/* Clear both timers of any pending interrupts */
	NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);
	NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);

	/* Enable both timer interrupts */
	NVIC_EnableIRQ(TIMER_32_1_IRQn);
	NVIC_EnableIRQ(TIMER_16_0_IRQn);
}

