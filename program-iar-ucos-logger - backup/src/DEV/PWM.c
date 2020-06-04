/*****************************************************************************
*  LPC11U6x_E6x PWM Decode demo program
*
*  Use SCT timer capture and capture control features. It implements a
*  PWM decoder which measures the duty cycle of a PWM signal and determines
*  whether it is above (max_width) or below (min_width) a specific value.
*  The PWM signal frequency is assumed to be 10 kHz. Two output signals
*  (width_error and timeout) are included to indicate when the 10 kHz signal
*  has an error or is missing.
*****************************************************************************/
#include "pwm.h"

/*****************************************************************************
*  LPC11U6x SCT PWM Reload program
*
*  Use the match reload registers to change the duty cycle of two PWM
*  signals and maintain their dead-time intervals using the NORELOAD_L bit
*  in the SCT Configuration register
*****************************************************************************/
//#include "board.h"
/*
#define DC1        (170)                                   						// duty cycle 1
#define DC2        (135)                                   						// duty cycle 2
#define hperiod    (180)



void SCT0_Init(void)
{

	Chip_SCT_Init(LPC_SCT0);							   						//enable clock for SCT0/1

	Chip_SCT_Config(LPC_SCT0,  SCT_CONFIG_AUTOLIMIT_L);							//two 16-bit timers, clocked internally, auto limit


	Chip_SCT_SetControl(LPC_SCT0, SCT_CTRL_BIDIR_L(0x01));						// Bidir mode, prescaler = 6, SCT clock = 2 MHz

	Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_0,hperiod);					   	// match 0 @ 10/2MHz = 5 usec (100 KHz PWM freq)
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_0, hperiod);

	Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_1,DC1);					   		// match 1 used for duty cycle (in 10 steps)
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_1, DC1);

	Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_2,DC2);					   		// match 1 used for duty cycle (in 10 steps)
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_2, DC2);

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_0, ENABLE_ALL_STATES);				// event 0 happens in all states
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_0, (CHIP_SCT_EVENTCTRL_T) ( SCT_IOCOND_FALL	|			// IN_0 falling edge
                                                                        SCT_COMBMODE_IO	));			// COMBMODE[13:12] = match condition only

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_1, ENABLE_ALL_STATES);				// event 1 happens in all states
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_1, (CHIP_SCT_EVENTCTRL_T) ( SCT_IOCOND_RAISE	|			// IN_0 rising edge
                                                                        SCT_COMBMODE_IO	  ));			// COMBMODE[13:12] = match condition only

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_2, ENABLE_ALL_STATES);				// event 2 happens in all states
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_2, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH1	|			// related to match 1
                                                                        SCT_COMBMODE_MATCH	));			// COMBMODE[13:12] = match condition only

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_3, ENABLE_ALL_STATES);				// event 3 happens in all states
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_3, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH2	|			// related to match 2
                                                                        SCT_COMBMODE_MATCH	));			// COMBMODE[13:12] = match condition only



	Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_0 , (CHIP_SCT_EVENT_T)  ( SCT_EVT_0 |						// event 0 and 2 set OUT0 (blue LED)
                                                                    SCT_EVT_2 ));

	Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_0, SCT_EVT_2);					// event 2 clears OUT0 (blue LED)

	Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_1 , SCT_EVT_3);						// event 3 sets OUT1 (red LED)


	Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_1, (CHIP_SCT_EVENT_T)  (  SCT_EVT_0 |
                                                                      SCT_EVT_3 ));					// event 0 and 3 clear OUT1 (red LED)

	Chip_SCT_SetConflictResolution(LPC_SCT0, 0, 0x0F);							// toggle OUT0 and OUT1 on conflict

	Chip_SCT_Output(LPC_SCT0, 0x01 );											// default set OUT0 and and clear OUT1


	LPC_SCT0->STOP_L           = (1 << 0);                 						// event 0 will stop the timer
	LPC_SCT0->EVEN             = (1 << 1);                 						// event 1 will generate an irq

	NVIC_EnableIRQ(SCT0_1_IRQn);                           						// enable SCT0/1 interrupt

	Chip_SCT_ClearControl(LPC_SCT0, SCT_CTRL_HALT_L);							// unhalt it by clearing bit 2 of the CTRL register
}





void SCT0_1_IRQHandler(void)
{
	Chip_SCT_SetControl(LPC_SCT0, SCT_CTRL_CLRCTR_L);	// clear the L counter

	Chip_SCT_ClearControl(LPC_SCT0, SCT_CTRL_STOP_L);	// start the L counter

	Chip_SCT_ClearEventFlag(LPC_SCT0, SCT_EVT_1);		// clear event 1 interrupt flag

}*/
//--------------------------------------------------
// 加速时间100ms，PWM频率10K时，总共计数为1000个点。加速时间或频率调整，相应的拉长或缩短曲线。
// 0-100%
//const uint8_t S_curve[100]={49,37,25,21,17,16,15,13,13,11,12,10,10,10,10,9,9,8,9,8,8,7,8,7,7,8,7,6,7,7,6,6,7,6,6,6,6,6,5,6,5,6,5,6,5,5,6,5,5,5,
//6,5,5,5,6,5,5,6,5,5,6,6,5,6,6,6,6,6,7,6,6,7,7,6,7,7,8,7,8,7,8,8,9,8,9,9,10,9,11,10,12,11,13,13,15,15,18,21,25,36};

//15-100%
//const uint8_t S_curve[100]={54,39,28,22,19,17,16,15,13,13,12,12,11,10,11,9,10,9,9,9,9,8,8,8,8,8,7,8,7,7,7,7,7,6,7,6,7,6,6,7,6,6,5,5,6,6,7,6,6,7,
//6,7,6,7,7,7,7,7,8,7,8,8,8,8,8,9,9,9,9,10,9,11,10,11,12,12,13,13,15,16,17,19,22,28,39,55};

//50-100%
//const uint8_t S_curve[51]={70,52,36,29,25,22,20,19,18,17,16,15,14,14,13,13,13,12,12,11,11,11,11,10,10,13,10,10,11,11,11,11,12,12,13,13,13,14,14,15,16,17,18,19,20,22,25,29,36,52,71};
const uint16_t S_curve[51]={350,260,180,145,125,110,100,95,90,85,80,75,70,70,65,65,65,60,60,55,55,55,55,50,50,65,50,50,55,55,55,55,60,60,65,65,65,70,70,75,80,85,90,95,100,110,125,145,180,260,355};

static uint16_t PWM_s_count=0;
uint8_t PWM_s_duty=0;
uint8_t PWM_s_quick=1;

void SCT0_1_IRQHandler(void)
{
    /*if(PWM_s_quick)
    {
        LPC_SCT0->MATCHREL[1].L = 90;
    }
    else
    {
        LPC_SCT0->MATCHREL[1].L = 10;
    }*/

    if(PWM_s_duty>=2 && PWM_s_duty<=98)
    {
        PWM_s_count++;
        if(PWM_s_count>S_curve[PWM_s_duty-50])
        {
            PWM_s_count = 0;
            if(PWM_s_quick)
            {
                PWM_s_duty++;
            }
            else
            {
                PWM_s_duty--;
            }
            LPC_SCT0->MATCHREL[1].L = PWM_s_duty;
        }
    }
	Chip_SCT_ClearEventFlag(LPC_SCT0, SCT_EVT_1);		// clear event 1 interrupt flag

}

void PWM_Init(uint8_t port,uint8_t pwm_duty)
{

    //Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 2, 0x93);   // 端口初始化

	Chip_SCT_Init(LPC_SCT0);							   						                    //enable clock for SCT0/1

	Chip_SCT_Config(LPC_SCT0, SCT_CONFIG_AUTOLIMIT_L);	//SCT_CONFIG_32BIT_COUNTER|  //one 32-bit timers, clocked internally, auto limit


	Chip_SCT_SetControl(LPC_SCT0, SCT_CTRL_PRE_L((SystemCoreClock/1000000-1)));	// set prescaler, SCT clock = 1 MHz

	//Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_0,99);					   		// 19 match 0 @ 20/1MHz = 20 usec (50 KHz PWM freq)
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_0,99);

	//Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_1,1);					        // 5 match 1 used for duty cycle (in 20 steps)
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_1,pwm_duty);

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_0, ENABLE_STATE0);					    // event 0 only happens in state 0
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_0, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH0 |	     // related to match 0
                                                                          SCT_COMBMODE_MATCH |		 // COMBMODE[13:12] = match condition only
                                                                          SCT_STATELD_1		 |	     // STATELD[14] = STATEV is loaded into state
                                                                          SCT_STATEEV_0	     ));	 // STATEV[15] = 0

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_1, ENABLE_STATE0);					    // event 1 only happens in state 0
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_1, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH1 |	     // related to match 1
                                                                          SCT_COMBMODE_MATCH |		 // COMBMODE[13:12] = match condition only
                                                                          SCT_STATELD_1		 |		 // STATELD[14] = STATEV is loaded into state
                                                                          SCT_STATEEV_0		));		 // STATEV[15] = 0


    switch(port)
    {
    case PWM01:
        Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 2, 0x8b);   // 端口初始化0x93

        Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_1 , SCT_EVT_0);						      // event 0 will set SCTx_OUT1
        Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_1, SCT_EVT_1);					      // event 1 will clear SCTx_OUT1
        break;
    case PWM02:
        Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 7, 0x8a);
        Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_2 , SCT_EVT_0);						      // event 0 will set SCTx_OUT2
        Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_2, SCT_EVT_1);					      // event 1 will clear SCTx_OUT2
        break;
    case PWM03:
        Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 13, 0x8a);
        Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_3 , SCT_EVT_0);						      // event 0 will set SCTx_OUT2
        Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_3, SCT_EVT_1);					      // event 1 will clear SCTx_OUT2
        break;
    default:
        break;
    }



    LPC_SCT0->EVEN  = (1 << 1);                 						// event 1 will generate an irq

	NVIC_EnableIRQ(SCT0_1_IRQn);                           				// enable SCT0/1 interrupt

	Chip_SCT_ClearControl(LPC_SCT0, SCT_CTRL_HALT_L);					// unhalt it by clearing bit 2 of the CTRL register
}

void PWM_Deinit(void)
{
    Chip_SCT_DeInit(LPC_SCT0);
    NVIC_DisableIRQ(SCT0_1_IRQn);
}

void PWM_SetDuty(uint8_t port,uint16_t duty)
{
    static uint8_t PWMing=0;
    if(duty<98 && duty>1)
    {
        if(0==PWMing)
        {
            PWM_Init(port,duty);
            PWMing = 1;
        }

        //Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_1,duty);
        //Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_1,duty);
        LPC_SCT0->MATCHREL[1].L = duty;
    }
    else if(duty>=98)
    {
        if(1==PWMing)
        {
            PWMing = 0;
            PWM_Deinit();
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 2, 0x90);   // 端口初始化
            Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 2, true);
            Chip_GPIO_WritePortBit(LPC_GPIO, 2, 2, true);
        }
        else
        {
            Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 2, true);
            Chip_GPIO_WritePortBit(LPC_GPIO, 2, 2, true);
        }
    }
    else if(duty<=1)
    {
        if(1==PWMing)
        {
            PWMing = 0;
            PWM_Deinit();
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 2, 0x90);   // 端口初始化
            Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 2, true);
            Chip_GPIO_WritePortBit(LPC_GPIO, 2, 2, false);
        }
        else
        {
            Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 2, true);
            Chip_GPIO_WritePortBit(LPC_GPIO, 2, 2, false);
        }
    }
}

void PWM_SetFreq(uint16_t f)
{
    if(f<10000 && f>50)
    {
        Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_0,f);
        Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_1,f/2);
    }
}
//-----------------------------------------------------
/*void SysTick_Handler(void)
{

	Chip_SCT_SetControl(LPC_SCT1, SCT_CONFIG_NORELOADL_U);			// stop reload process for L counter

    if(Chip_GPIO_ReadPortBit(LPC_GPIO,0,20))//if (Chip_GPIO_GetPinState(LPC_GPIO, 0, 16))              		// P0_16 high?
    {
        if (LPC_SCT1->MATCHREL[2].L < hperiod-1)        			// check if DC2 < Period of PWM
        {
            LPC_SCT1->MATCHREL[1].L ++;
            LPC_SCT1->MATCHREL[2].L ++;

        }
    }
    else if (LPC_SCT1->MATCHREL[1].L > 1)              				// check if DC1 > 1
    {
        LPC_SCT1->MATCHREL[1].L --;
        LPC_SCT1->MATCHREL[2].L --;
        Chip_GPIO_WritePortBit(LPC_GPIO,2,10,true);
    }

    LPC_SCT1->CONFIG &= ~(1 << 7);                     				// enable reload process for L counter

}*/



