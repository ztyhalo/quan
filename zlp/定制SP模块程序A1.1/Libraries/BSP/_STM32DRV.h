#ifndef __STM32F103DRV_H__
#define __STM32F103DRV_H__


#define TIMER1_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define TIMER2_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define TIMER3_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define TIMER4_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define TIMER5_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define TIMER6_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define TIMER7_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define TIMER8_EN                                 //��ʱ��ģ��ʹ��ʹ��
#define USART1_EN
#define USART2_EN
#define SYSTICK_EN                               //SysTickģ��ʹ��ʹ��
#define CAN_EN
#define SPI1_EN
//#define SPI2_EN
#define FSMC_EN 
#define DMA_EN
#define ADC1_EN                                  //ADģ��ʹ��ʹ��
#define RTC_EN
#define IWATCHDOG_EN


#define TIMER1_INT_EN
#define TIMER2_INT_EN
#define TIMER3_INT_EN
#define TIMER4_INT_EN
#define TIMER5_INT_EN
#define TIMER6_INT_EN
#define TIMER7_INT_EN
#define TIMER8_INT_EN

#define CAN_INT_EN
#define USART1_INT_EN
#define USART2_INT_EN
#define SPI1_INT_EN
#define SPI2_INT_EN
#define FSMC_INT_EN
#define DMA_INT_EN 
#define ADC1_INT_EN   
#define RTC_INT_EN
#define IWATCHDOG_INT_EN




/*****************************��������*****************************************/

void RCCInit_Base(void);
void NVICInit_Base(void);
void SysTickInit_Base(void);
void SysTickHandleFun(void);
void Delay(u32 nTime);
void Timer1Init_Base(void);
void Timer2Init_Base(void);
void Timer3Init_Base(void);
void Timer4Init_Base(void);
void Timer5Init_Base(void);
void Timer6Init_Base(void);
void Timer7Init_Base(void);
void Timer8Init_Base(void);
void ADC1Init_Base(void);
void DMAInit_Base(void);
void FSMCInit_Base(void);
void USART1Init_Base(void);
void USART2Init_Base(void);
void CANInit_Base(void);
void SPI1Init_Base(void);
void SPI2Init_Base(void);
void IWDGInit_Base(void);
void RTCInit_Base(void);

void ADC1_Start(void);
void SPI1_SendData(u8 data);


#endif

/****************************************END OF FILE************************/

