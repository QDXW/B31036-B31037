#include "InRtc.h"

#include "chip.h"


#define rILR    (*(volatile unsigned*)0x40024000)
#define rCCR    (*(volatile unsigned*)0x40024008)
#define rCIIR   (*(volatile unsigned*)0x4002400C)
#define rAMR    (*(volatile unsigned*)0x40024010)
#define rCALIBRATION    (*(volatile unsigned*)0x40024040)

#define rYEAR   (*(volatile unsigned*)0x4002403C)
#define rMONTH  (*(volatile unsigned*)0x40024038)
#define rDOM    (*(volatile unsigned*)0x4002402C)
#define rHOUR   (*(volatile unsigned*)0x40024028)
#define rMIN    (*(volatile unsigned*)0x40024024)
#define rSEC    (*(volatile unsigned*)0x40024020)

#define rALSEC  (*(volatile unsigned*)0x40024060)

#define rCTIME0 (*(volatile unsigned*)0x40024014)
#define rCTIME1 (*(volatile unsigned*)0x40024018)
#define rCTIME2 (*(volatile unsigned*)0x4002401C)

unsigned char flag_setTime=1;
unsigned char flag_receiveStatus=0;
unsigned char timeData[14],cnt;

INRTC_MES_T g_sRTC;

static uint32_t g_uTest=0;

void SetInRtc(void)
{
    rCCR &= ~(0x1<<0);

    rYEAR  = g_sRTC.uYear;
    rMONTH = g_sRTC.uMonth;
    rDOM   = g_sRTC.uDate;
    rHOUR  = g_sRTC.uHour;
    rMIN   = g_sRTC.uMinute;
    rSEC   = g_sRTC.uSecond;
}

void InitInRtc(void)
{
    /*rILR = 0;
    rCCR = 0;
    rCIIR = 0;
    rAMR = 0xff;
    rCALIBRATION = 0;

    rCCR |= 0x1<<1;   //CTC Reset
    rCCR &= ~(0x1<<1);*/

    // Enable the RTC oscillator, oscillator rate can be determined by
	//   calling Chip_Clock_GetRTCOscRate()
	Chip_Clock_EnableRTCOsc();

	// Initialize RTC driver (enables RTC clocking)
	Chip_RTC_Init(LPC_RTC);

	// RTC reset
	Chip_RTC_Reset(LPC_RTC);

    // Start RTC at a count of 0 when RTC is disabled. If the RTC is enabled, you
	// need to disable it before setting the initial RTC count.
	//Chip_RTC_Disable(LPC_RTC);
	//Chip_RTC_SetCount(LPC_RTC, 0);

	// Set a long alarm time so the interrupt won't trigger
	//Chip_RTC_SetAlarm(LPC_RTC, 1);//1000

	// Enable RTC and high resolution timer - this can be done in a single
	// call with Chip_RTC_EnableOptions(LPC_RTC, (RTC_CTRL_RTC1KHZ_EN | RTC_CTRL_RTC_EN));
	Chip_RTC_Enable1KHZ(LPC_RTC);
	Chip_RTC_Enable(LPC_RTC);
/*
	// Clear latched RTC interrupt statuses
	Chip_RTC_ClearStatus(LPC_RTC, (RTC_CTRL_OFD | RTC_CTRL_ALARM1HZ | RTC_CTRL_WAKE1KHZ));

	// Enable RTC as a peripheral wakeup event
	//Chip_SYSCTL_EnablePeriphWakeup(SYSCTL_WAKEUP_RTCINT);

	// Don't power down WDT (or BOD) during deep sleep.
	Chip_SYSCTL_SetDeepSleepPD(0);

	// Power everything up except for ADC, USB and temp sensor on wake up from deep sleep.
	Chip_SYSCTL_SetWakeup(SYSCTL_SLPWAKE_ADC_PD    |
						  SYSCTL_SLPWAKE_USBPLL_PD |
						  SYSCTL_SLPWAKE_USBPAD_PD |
						  SYSCTL_SLPWAKE_TS_PD
						  );
    // Enable RTC interrupt
	//NVIC_EnableIRQ(RTC_IRQn);

    // Enable RTC alarm interrupt
	//Chip_RTC_EnableWakeup(LPC_RTC, (RTC_CTRL_ALARMDPD_EN));
    //Chip_RTC_EnableWakeup(LPC_RTC, (RTC_CTRL_ALARMDPD_EN | RTC_CTRL_WAKEDPD_EN));
*/
}

void GetInRtc(void)
{
    g_sRTC.uYear   = rYEAR;
    g_sRTC.uMonth  = rMONTH;
    g_sRTC.uDate   = rDOM;
    g_sRTC.uHour   = rHOUR;
    g_sRTC.uMinute = rMIN;
    g_sRTC.uSecond = rSEC;
}


/**
 * @brief	RTC Interrupt Handler
 * @return	None
 */
void RTC_IRQHandler(void)
{
	uint32_t rtcStatus;

    uint8_t rtcWake, rtcAlarm;

	//Board_LED_Toggle(0);

	/* Get RTC status register */
	rtcStatus = Chip_RTC_GetStatus(LPC_RTC);

	/* Check RTC 1KHz match interrupt */
	if (rtcStatus & RTC_CTRL_WAKE1KHZ) {
		/* RTC high resultiuon wakeup interrupt */
		rtcWake = true;
        rtcWake = rtcWake;
	}

	/* Check RTC 1Khz match interrupt */
	if (rtcStatus & RTC_CTRL_ALARM1HZ) {
		/* Alarm */
		rtcAlarm = true;
        rtcAlarm = rtcAlarm;
        g_uTest += 10;
        Chip_RTC_SetAlarm(LPC_RTC, g_uTest);
	}

	/* Clear only latched RTC status */
	Chip_RTC_EnableOptions(LPC_RTC,
						   (rtcStatus & (RTC_CTRL_WAKE1KHZ | RTC_CTRL_ALARM1HZ)));
}


uint32_t GetInRtcCount(void)
{
    return Chip_RTC_GetCount(LPC_RTC);
}

void SetInRtcCount(uint32_t count)
{
    Chip_RTC_Disable(LPC_RTC);
    Chip_RTC_SetCount(LPC_RTC,count);
    Chip_RTC_Enable(LPC_RTC);
}