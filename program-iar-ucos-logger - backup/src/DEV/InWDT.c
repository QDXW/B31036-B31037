#include "InWDT.h"
#include "chip.h"



void BOD_WDT_IRQHandler(void)
{
	uint32_t wdtStatus = Chip_WWDT_GetStatus(LPC_WWDT);

	/* The chip will reset before this happens, but if the WDT doesn't
	   have WWDT_WDMOD_WDRESET enabled, this will hit once */
	if (wdtStatus & WWDT_WDMOD_WDTOF) {
		/* A watchdog feed didn't occur prior to window timeout */
		Chip_WWDT_UnsetOption(LPC_WWDT, WWDT_WDMOD_WDEN);	/* Stop WDT */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);
		Chip_WWDT_Start(LPC_WWDT);	/* Needs restart */
	}

	/* Handle warning interrupt */
	if (wdtStatus & WWDT_WDMOD_WDINT) {
		/* A watchdog feed didn't occur prior to warning timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDINT);
		//Chip_WWDT_Feed(LPC_WWDT);
	}
}


void InWDT_Init(void)
{
    uint32_t wdtFreq;

    /* WDT oscillator freq = 0.6MHz divided by 64 = 9.375khz */
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 64);

    /* The WDT divides the input frequency into it by 4 */
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;

	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Use WDTOSC as the WDT clock */
	Chip_WWDT_SelClockSource(LPC_WWDT, WWDT_CLKSRC_WATCHDOG_WDOSC);

    /* Enable the power to the WDT */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_WDTOSC_PD);

	/* The WDT divides the input frequency into it by 4 */
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;

	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Use WDTOSC as the WDT clock */
	Chip_WWDT_SelClockSource(LPC_WWDT, WWDT_CLKSRC_WATCHDOG_WDOSC);

	/* Set watchdog feed time constant to approximately 2s
	   Set watchdog warning time to 512 ticks after feed time constant
	   Set watchdog window time to 3s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq * 2);
	Chip_WWDT_SetWarning(LPC_WWDT, 512);
	Chip_WWDT_SetWindow(LPC_WWDT, wdtFreq * 3);

	/* Configure WWDT to reset on timeout */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);

	/* Clear watchdog warning and timeout interrupts */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

	/* Allow WDT to wake from deep sleep. */
	//Chip_SYSCTL_EnablePeriphWakeup(SYSCTL_WAKEUP_BOD_WDT_INT);

	/* Don't power down WDT (or BOD) during deep sleep. */
	Chip_SYSCTL_SetDeepSleepPD(0);

	/* Power everything up except for ADC, USB and temp sensor on wake up from
	   deep sleep. */
	/*Chip_SYSCTL_SetWakeup(SYSCTL_SLPWAKE_ADC_PD    |
						  SYSCTL_SLPWAKE_USBPLL_PD |
						  SYSCTL_SLPWAKE_USBPAD_PD |
						  SYSCTL_SLPWAKE_TS_PD
						  );
    */
	/* Clear and enable watchdog interrupt */
	NVIC_ClearPendingIRQ(BOD_WDT_IRQn);
	NVIC_EnableIRQ(BOD_WDT_IRQn);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);
}

void InWDT_Deinit(void)
{
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_WDT);
}
