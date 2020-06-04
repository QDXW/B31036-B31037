;/*****************************************************************************
; * @file:    startup_LPC11u6x.s
; * @purpose: CMSIS Cortex-M0PLUS Core Device Startup File 
; *           for the NXP LPC11u6x Device Series (manually edited)
; * @version: V1.0
; * @date:    25. Nov. 2008
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; * Copyright (C) 2008 ARM Limited. All rights reserved.
; * ARM Limited (ARM) is supplying this software for use with Cortex-M0 
; * processor based microcontrollers.  This file can be freely distributed 
; * within development tools that are supporting such ARM based processors. 
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; *****************************************************************************/

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

				DCD PIN_INT0_IRQHandler           ; Pin interrupt 0
				DCD PIN_INT1_IRQHandler           ; Pin interrupt 1
				DCD PIN_INT2_IRQHandler           ; Pin interrupt 2
				DCD PIN_INT3_IRQHandler           ; Pin interrupt 3
				DCD PIN_INT4_IRQHandler           ; Pin interrupt 4
				DCD PIN_INT5_IRQHandler           ; Pin interrupt 5
				DCD PIN_INT6_IRQHandler           ; Pin interrupt 6
				DCD PIN_INT7_IRQHandler           ; Pin interrupt 7
				DCD GINT0_IRQHandler              ; Port interrupt group 0
				DCD GINT1_IRQHandler              ; Port interrupt group 1
				DCD I2C1_IRQHandler               ; I2C1 interrupt
				DCD USART1_4_IRQHandler           ; USARTS 1 and 4 shared interrupt
				DCD USART2_3_IRQHandler           ; USARTS 2 and 3 shared interrupt
				DCD SCT0_1_IRQHandler             ; SCT 0 and 1 shared interrupt
				DCD SSP1_IRQHandler               ; SSP1 interrupt
				DCD I2C0_IRQHandler               ; I2C0 interrupt
				DCD TIMER16_0_IRQHandler          ; CT16B0 (16-bit Timer 0)
				DCD TIMER16_1_IRQHandler          ; CT16B1 (16-bit Timer 1)
				DCD TIMER32_0_IRQHandler          ; CT32B0 (32-bit Timer 0)
				DCD TIMER32_1_IRQHandler          ; CT32B0 (32-bit Timer 1)
				DCD SSP0_IRQHandler               ; SSP0 interrupt interrupt
				DCD USART0_IRQHandler             ; USART 0 interrupt interrupt
				DCD USB_IRQHandler                ; USB IRQ interrupt
				DCD USB_FIQHandler                ; USB FIQ interrupt
				DCD ADCA_IRQHandler               ; ADC A sequence (A/D Converter) interrupt
				DCD RTC_IRQHandler                ; RTC interrupt
				DCD BOD_WDT_IRQHandler            ; Shared BOD (Brownout Detect) and WDT interrupt
				DCD FMC_IRQHandler                ; Flash Memory Controller interrupt
				DCD DMA_IRQHandler                ; DMA Controller interrupt
				DCD ADCB_IRQHandler               ; ADC B sequence interrupt
				DCD USBWakeup_IRQHandler          ; USB wake-up interrupt
				DCD Reserved_IRQHandler

                IF      :LNOT::DEF:NO_CRP
                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
CRP_Key         DCD     0xFFFFFFFF
                ENDIF

                AREA    |.text|, CODE, READONLY

; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)                

; now, under COMMON NMI.c and NMI.h, a real NMI handler is created if NMI is enabled 
; for particular peripheral.
NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
Reserved_IRQHandler PROC
                EXPORT  Reserved_IRQHandler       [WEAK]
                B       .
                ENDP

Default_Handler PROC
				EXPORT  PIN_INT0_IRQHandler       [WEAK]
				EXPORT  PIN_INT1_IRQHandler       [WEAK]
				EXPORT  PIN_INT2_IRQHandler       [WEAK]
				EXPORT  PIN_INT3_IRQHandler       [WEAK]
				EXPORT  PIN_INT4_IRQHandler       [WEAK]
				EXPORT  PIN_INT5_IRQHandler       [WEAK]
				EXPORT  PIN_INT6_IRQHandler       [WEAK]
				EXPORT  PIN_INT7_IRQHandler       [WEAK]
				EXPORT  GINT0_IRQHandler          [WEAK]
				EXPORT  GINT1_IRQHandler          [WEAK]
				EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT	USART1_4_IRQHandler       [WEAK]
                EXPORT	USART2_3_IRQHandler       [WEAK]
				EXPORT  SCT0_1_IRQHandler         [WEAK]
				EXPORT  SSP1_IRQHandler           [WEAK]
				EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT	TIMER16_0_IRQHandler      [WEAK]
                EXPORT	TIMER16_1_IRQHandler      [WEAK]
                EXPORT	TIMER32_0_IRQHandler      [WEAK]
                EXPORT	TIMER32_1_IRQHandler      [WEAK]
                EXPORT	SSP0_IRQHandler           [WEAK]
                EXPORT	USART0_IRQHandler         [WEAK]
                EXPORT	USB_IRQHandler            [WEAK]
                EXPORT	USB_FIQHandler            [WEAK]
                EXPORT	ADCA_IRQHandler           [WEAK]
                EXPORT	RTC_IRQHandler            [WEAK]
                EXPORT	BOD_WDT_IRQHandler        [WEAK]
                EXPORT	FMC_IRQHandler            [WEAK]
                EXPORT	DMA_IRQHandler            [WEAK]
                EXPORT	ADCB_IRQHandler           [WEAK]
                EXPORT	USBWakeup_IRQHandler      [WEAK]

PIN_INT0_IRQHandler
PIN_INT1_IRQHandler
PIN_INT2_IRQHandler
PIN_INT3_IRQHandler
PIN_INT4_IRQHandler
PIN_INT5_IRQHandler
PIN_INT6_IRQHandler
PIN_INT7_IRQHandler
GINT0_IRQHandler
GINT1_IRQHandler
I2C1_IRQHandler
USART1_4_IRQHandler
USART2_3_IRQHandler
SCT0_1_IRQHandler
SSP1_IRQHandler
I2C0_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
USART0_IRQHandler
USB_IRQHandler
USB_FIQHandler
ADCA_IRQHandler
RTC_IRQHandler
BOD_WDT_IRQHandler
FMC_IRQHandler
DMA_IRQHandler
ADCB_IRQHandler
USBWakeup_IRQHandler

                B       .

                ENDP

                ALIGN

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF

                END
