;******************************************************************************
;
;                             INTERRUPT VECTORS
;                                    ARM
;                             KEIL's uVision3 
;                   (RealView Microprocessor Developer Kit)
;
; Filename      : vectors.s
;******************************************************************************
                PRESERVE8
                AREA   VECT, CODE, READONLY                     ; Name this block of code                                   ;
                THUMB

                ENTRY

;******************************************************************************
;                                  IMPORTS
;******************************************************************************

		        IMPORT	SysTick_Handler
				IMPORT	HardFault_Handler
		        IMPORT	OS_CPU_PendSVHandler
                IMPORT  ResetHndlr
                IMPORT  ||Image$$ARM_LIB_STACK$$ZI$$Limit||     ; Import stack limit from scatter-loading file

;******************************************************************************
;                                  EXPORTS
;******************************************************************************


;******************************************************************************
;                                DEFINITIONS
;******************************************************************************


;******************************************************************************
;                      INITIALIZE EXCEPTION VECTORS
;******************************************************************************




Vectors
        DCD     ||Image$$ARM_LIB_STACK$$ZI$$Limit||         ;  0, SP start value.                                         
        DCD     ResetHndlr                                  ;  1, PC start value.                                         
        DCD     App_NMI_ISR                                 ;  2, NMI                                                     
;        DCD     App_Fault_ISR                               ;  3, Hard Fault                                              
        DCD 	HardFault_Handler
        DCD     App_Spurious_ISR                            ;  4, Memory Management                                      
        DCD     App_Spurious_ISR                            ;  5, Bus Fault                                               
        DCD     App_Spurious_ISR                            ;  6, Usage Fault                                             
        DCD     0                                           ;  7, Reserved                                                
        DCD     0                                           ;  8, Reserved                                                
        DCD     0                                           ;  9, Reserved                                                
        DCD     0                                           ; 10, Reserved                                                
        DCD     App_Spurious_ISR                            ; 11, SVCall                                                  
        DCD     App_Spurious_ISR                            ; 12, Debug Monitor                                           
        DCD     0											; 13, Reserved                                                
        DCD     OS_CPU_PendSVHandler                                    ; 14, PendSV Handler                                          
        DCD     SysTick_Handler							; 15, SysTick Handler,uC/OS-II Tick ISR Handler                               

                EXTERN  WWDG_IRQHandler           
                EXTERN  PVD_IRQHandler             
                EXTERN  TAMPER_IRQHandler          
                EXTERN  RTC_IRQHandler             
                EXTERN  FLASH_IRQHandler           
                EXTERN  RCC_IRQHandler             
                EXTERN  EXTI0_IRQHandler           
                EXTERN  EXTI1_IRQHandler           
                EXTERN  EXTI2_IRQHandler           
                EXTERN  EXTI3_IRQHandler           
                EXTERN  EXTI4_IRQHandler           
                EXTERN  DMA1_Channel1_IRQHandler   
                EXTERN  DMA1_Channel2_IRQHandler   
                EXTERN  DMA1_Channel3_IRQHandler   
                EXTERN  DMA1_Channel4_IRQHandler   
                EXTERN  DMA1_Channel5_IRQHandler   
                EXTERN  DMA1_Channel6_IRQHandler   
                EXTERN  DMA1_Channel7_IRQHandler   
                EXTERN  ADC1_2_IRQHandler          
                EXTERN  USB_HP_CAN1_TX_IRQHandler  
                EXTERN  USB_LP_CAN1_RX0_IRQHandler 
                EXTERN  CAN1_RX1_IRQHandler        
                EXTERN  CAN1_SCE_IRQHandler        
                EXTERN  EXTI9_5_IRQHandler         
                EXTERN  TIM1_BRK_IRQHandler        
                EXTERN  TIM1_UP_IRQHandler         
                EXTERN  TIM1_TRG_COM_IRQHandler    
                EXTERN  TIM1_CC_IRQHandler         
                EXTERN  TIM2_IRQHandler            
                EXTERN  TIM3_IRQHandler            
                EXTERN  TIM4_IRQHandler            
                EXTERN  I2C1_EV_IRQHandler         
                EXTERN  I2C1_ER_IRQHandler         
                EXTERN  I2C2_EV_IRQHandler         
                EXTERN  I2C2_ER_IRQHandler         
                EXTERN  SPI1_IRQHandler            
                EXTERN  SPI2_IRQHandler            
                EXTERN  USART1_IRQHandler          
                EXTERN  USART2_IRQHandler          
                EXTERN  USART3_IRQHandler          
                EXTERN  EXTI15_10_IRQHandler       
                EXTERN  RTCAlarm_IRQHandler        
                EXTERN  USBWakeUp_IRQHandler       
                EXTERN  TIM8_BRK_IRQHandler        [WEAK]
                EXTERN  TIM8_UP_IRQHandler         [WEAK]
                EXTERN  TIM8_TRG_COM_IRQHandler    [WEAK]
                EXTERN  TIM8_CC_IRQHandler         [WEAK]
                EXTERN  ADC3_IRQHandler            [WEAK]
                EXTERN  FSMC_IRQHandler            [WEAK]
                EXTERN  SDIO_IRQHandler            [WEAK]
                EXTERN  TIM5_IRQHandler            [WEAK]
                EXTERN  SPI3_IRQHandler            [WEAK]
                EXTERN  UART4_IRQHandler           [WEAK]
                EXTERN  UART5_IRQHandler           [WEAK]
                EXTERN  TIM6_IRQHandler            [WEAK]
                EXTERN  TIM7_IRQHandler            [WEAK]
                EXTERN  DMA2_Channel1_IRQHandler   [WEAK]
                EXTERN  DMA2_Channel2_IRQHandler   [WEAK]
                EXTERN  DMA2_Channel3_IRQHandler   [WEAK]
                EXTERN  DMA2_Channel4_5_IRQHandler [WEAK]
                
                ; External Interrupts
                DCD     WWDG_IRQHandler            ; Window Watchdog
                DCD     PVD_IRQHandler             ; PVD through EXTI Line detect
                DCD     TAMPER_IRQHandler          ; Tamper
                DCD     RTC_IRQHandler             ; RTC
                DCD     FLASH_IRQHandler           ; Flash
                DCD     RCC_IRQHandler             ; RCC
                DCD     EXTI0_IRQHandler           ; EXTI Line 0
                DCD     EXTI1_IRQHandler           ; EXTI Line 1
                DCD     EXTI2_IRQHandler           ; EXTI Line 2
                DCD     EXTI3_IRQHandler           ; EXTI Line 3
                DCD     EXTI4_IRQHandler           ; EXTI Line 4
                DCD     DMA1_Channel1_IRQHandler   ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler   ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler   ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler   ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler   ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler   ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler   ; DMA1 Channel 7
                DCD     ADC1_2_IRQHandler          ; ADC1 & ADC2
                DCD     USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
                DCD     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
                DCD     CAN1_RX1_IRQHandler        ; CAN1 RX1
                DCD     CAN1_SCE_IRQHandler        ; CAN1 SCE
                DCD     EXTI9_5_IRQHandler         ; EXTI Line 9..5
                DCD     TIM1_BRK_IRQHandler        ; TIM1 Break
                DCD     TIM1_UP_IRQHandler         ; TIM1 Update
                DCD     TIM1_TRG_COM_IRQHandler    ; TIM1 Trigger and Commutation
                DCD     TIM1_CC_IRQHandler         ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler            ; TIM2
                DCD     TIM3_IRQHandler            ; TIM3
                DCD     TIM4_IRQHandler            ; TIM4
                DCD     I2C1_EV_IRQHandler         ; I2C1 Event
                DCD     I2C1_ER_IRQHandler         ; I2C1 Error
                DCD     I2C2_EV_IRQHandler         ; I2C2 Event
                DCD     I2C2_ER_IRQHandler         ; I2C2 Error
                DCD     SPI1_IRQHandler            ; SPI1
                DCD     SPI2_IRQHandler            ; SPI2
                DCD     USART1_IRQHandler          ; USART1
                DCD     USART2_IRQHandler          ; USART2
                DCD     USART3_IRQHandler          ; USART3
                DCD     EXTI15_10_IRQHandler       ; EXTI Line 15..10
                DCD     RTCAlarm_IRQHandler        ; RTC Alarm through EXTI Line
                DCD     USBWakeUp_IRQHandler       ; USB Wakeup from suspend
                DCD     TIM8_BRK_IRQHandler        ; TIM8 Break
                DCD     TIM8_UP_IRQHandler         ; TIM8 Update
                DCD     TIM8_TRG_COM_IRQHandler    ; TIM8 Trigger and Commutation
                DCD     TIM8_CC_IRQHandler         ; TIM8 Capture Compare
                DCD     ADC3_IRQHandler            ; ADC3
                DCD     FSMC_IRQHandler            ; FSMC
                DCD     SDIO_IRQHandler            ; SDIO
                DCD     TIM5_IRQHandler            ; TIM5
                DCD     SPI3_IRQHandler            ; SPI3
                DCD     UART4_IRQHandler           ; UART4
                DCD     UART5_IRQHandler           ; UART5
                DCD     TIM6_IRQHandler            ; TIM6
                DCD     TIM7_IRQHandler            ; TIM7
                DCD     DMA2_Channel1_IRQHandler   ; DMA2 Channel1
                DCD     DMA2_Channel2_IRQHandler   ; DMA2 Channel2
                DCD     DMA2_Channel3_IRQHandler   ; DMA2 Channel3
                DCD     DMA2_Channel4_5_IRQHandler ; DMA2 Channel4 & Channel5
        
        
;******************************************************************************
;                          DEFAULT HANDLERS
;******************************************************************************

;                AREA   VECTq, CODE, readwrite
				EXPORT	BadAlignedLDM

				
;				THUMB
;num equ   2
 ;   	  ENTRY



BadAlignedLDM
        MOVS r0, #1 
        LDM r0,{r1-r2} 
        BX LR;
		
;		END 

App_NMI_ISR         B       App_NMI_ISR

App_Fault_ISR       B       App_Fault_ISR		; 由于分配给任务的RAM不足,会引起硬件失效

App_Spurious_ISR    B       App_Spurious_ISR

;---------------------------------------------------- 03-26 
		export	sysreset
sysreset
	LDR R0,=0xE000ED0C
	LDR R1,=0x05FA0001
	STR R1,[R0]
	b .
;		mov		r0,		#1
;		msr		FAULTMASK,	r0
;		ldr		r0,		=0xE000ED0C
;		ldr     r1,		=0x05FA0004
;		STR		r1,    [r0]
;		b .
;		mov		r0,		#4
;		ldr		r0,		[r0]
;		add		r0,		#1
;		bx		r0
;---------------------------------------------------- 03-26 
                ALIGN
                END




