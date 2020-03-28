/*
*********************************************************************************************
*                                       天津华宁电子有限公司
*                             			   开发一部
*
*                                    CORTEX STM32F103系列模板文件
*
*
* 文件名 : _STM32DRV.c
* 作  者 : 赵健胜
* 版  本 : V1.0
* 日  期 : 2009-08-29
*********************************************************************************************
*                                              描 述
*
*         该源文件中提供CORTEX 常用外设的初始化配置,使用时根据需要可以更改
*
*********************************************************************************************
*/


/********************************************************************************************
*                                               头文件
********************************************************************************************/
#include "BSP_STM32Lib.h"

/********************************************************************************************
*                                               宏定义
********************************************************************************************/

#define ADC1_DR_Address    ((u32)0x4001244C)

/********************************************************************************************
*                                              常量定义
********************************************************************************************/

/********************************************************************************************
*                                              变量定义
********************************************************************************************/
volatile u32 TimingDelay; //该变量需要在中断处理文件中声明
volatile u32 SysTickTimer;
volatile u8 SysTickTimeUp;
vu16 ADCConvertedValue[80] = {0};
/********************************************************************************************
*                                              函数声明
********************************************************************************************/



/****************************************************************************
* 函数名称: RCCInit_Base
* 功能描述: 系统时钟及外设时钟源初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明: 
****************************************************************************/
void RCCInit_Base(void)
{
	RCC_DeInit();							               // RCC配置为默认模式
	/* Enable HSI */
    RCC_AdjustHSICalibrationValue(0x0f);
    RCC_HSICmd(ENABLE);					                   // 允许内部高速时钟

	/* HCLK = SYSCLK */
	RCC_HCLKConfig(RCC_SYSCLK_Div2);		               // 系统时钟分频(1,2,4,8-512),2分频	AHB clock
	  
	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1);			               // APB2分频(1,2,4-16),1分频,最大72MHz
	
	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config(RCC_HCLK_Div2);			               // APB1分频(1,2,4-16),2分频,最大36MHz
	FLASH_SetLatency(FLASH_Latency_2);		               // SYSCLK(系统时钟)周期与闪存访问时间的比例
												           // FLASH_Latency_0:零等待状态,当0<SYSCLK≤24MHz
												           // FLASH_Latency_1:一个等待状态,当24MHz<SYSCLK≤48MHz
												           // FLASH_Latency_2:两个等待状态,当48MHz<SYSCLK≤72MHz
	
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	// 使能闪存预取缓冲区

	/* PLLCLK = 4MHz * 12 = 48 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);	// PLL倍频系数,12倍频
	
    /* Enable PLL */ 
	RCC_PLLCmd(ENABLE);						                // 使能PLL
	
	/* Wait till PLL is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};	// 等待PLL正常
	
	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	            // 选择PLL作为系统时钟Fcclk
	
	/* Wait till PLL is used as system clock source */
	while(RCC_GetSYSCLKSource() != 0x08){};	                // 等待PLL作为系统时钟正常

}




/****************************************************************************
* 函数名称: DMAInit_Base
* 功能描述: ADC1初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void NVICInit_Base(void)
{
//	NVIC_InitTypeDef NVIC_InitStructure;
    	
	// 设置中断向量表空间
#ifdef  VECT_TAB_RAM  
  	/* Set the Vector Table base location at 0x20000000 */ 
  	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  	/* Set the Vector Table base location at 0x08000000 */ 
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

    /* 配置可抢占中断优先级 */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);



}

#ifdef SYSTICK_EN

/****************************************************************************
* 函数名称: SysTick_Init_Base
* 功能描述: 系统时钟初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明: 该函数只能在前后台系统中用，用作延时函数。在带操作系统（UCOS和RTX）
*           情况下不能使用 。
*           条件：外部晶振为8MHz，系统时钟为72MHz，SysTick的频率9MHz，SysTick产生1ms的中断。
****************************************************************************/
void SysTickInit_Base(void)
{
	SysTick_SetReload(9000);	                                  // 设定SysTick达到1ms计数结束
	SysTick_ITConfig(ENABLE);                                     // 使能SysTick中断
}
/****************************************************************************
* 函数名称: SysTickHandleFun
* 功能描述: 系统时钟中断处理函数
* 入口参数: 无
* 出口参数: 无
* 使用说明: 该函数只能在前后台系统中用，用作延时函数。在带操作系统（UCOS和RTX）
*           情况下不能使用 
****************************************************************************/
void SysTickHandleFun (void) 
{
	if (TimingDelay != 0x00)
	TimingDelay--;

	if(SysTickTimer != 0x00){
		SysTickTimer --;
	}else{
		SysTickTimeUp = 1;
		SysTickTimer = 0x0F;
	}
}
/****************************************************************************
* 函数名称: Delay
* 功能描述: 使用SysTick的延时处理函数
* 入口参数: 无
* 出口参数: 无
* 使用说明: 该函数只能在前后台系统中用，用作延时函数。在带操作系统（UCOS和RTX）
*           情况下不能使用。
*           使用条件：调用SysTickInit_Base（）完成初始化 。在SystickHandle中调用
*			中断处理函数SysTickHandler。
*           延时时间为：nTime × 1ms
****************************************************************************/
void Delay(u32 nTime)
{
	SysTick_CounterCmd(SysTick_Counter_Enable); // 使能SysTick计数器
	TimingDelay = nTime; // 读取延时时间
	while(TimingDelay != 0); // 判断延时是否结束
	SysTick_CounterCmd(SysTick_Counter_Disable); // 关闭SysTick计数器
	SysTick_CounterCmd(SysTick_Counter_Clear); // 清除SysTick计数器
}

#endif  /*end SYSTICK_EN*/

#ifdef TIMER1_EN
/****************************************************************************
* 函数名称: Timer1Init_Base
* 功能描述: 定时器1初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer1Init_Base(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	#ifdef TIMER1_INT_EN

	/*配置定时器1使能中断--------------------------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQChannel;    //配置定时器1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
	#endif

	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM1, TIM_FLAG_Update);                           // 清除TIM1溢出中断标志
 	
   	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);                      //TIM1溢出中断允许

}

#endif

#ifdef TIMER2_EN
/****************************************************************************
* 函数名称: Timer2Init_Base
* 功能描述: 定时器2初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer2Init_Base(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	#ifdef TIMER2_INT_EN
    
	/*配置定时器2使能中断--------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	#endif

	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM2, TIM_FLAG_Update);                           // 清除TIM2溢出中断标志
 	
   	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                      //TIM2溢出中断允许

}
#endif

#ifdef TIMER3_EN
/****************************************************************************
* 函数名称: Timer3Init_Base
* 功能描述: 定时器3初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer3Init_Base(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	#ifdef TIMER3_INT_EN
	
	/*配置定时器3使能中断---------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	#endif

	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM3, TIM_FLAG_Update);                           // 清除TIM3溢出中断标志
 	
   	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                      //TIM3溢出中断允许

}

#endif

#ifdef TIMER4_EN
/****************************************************************************
* 函数名称: Timer4Init_Base
* 功能描述: 定时器4初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer4Init_Base(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	#ifdef TIMER4_INT_EN
	/*配置定时器4使能中断---------------------------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;       //配置定时器4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

	#endif
	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM4, TIM_FLAG_Update);                           // 清除TIM4溢出中断标志
 	
   	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);                      //TIM4溢出中断允许

}
#endif

#ifdef TIMER5_EN
/****************************************************************************
* 函数名称: Timer4Init_Base
* 功能描述: 定时器4初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer5Init_Base(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

	#ifdef TIMER5_INT_EN
	/*配置定时器5使能中断---------------------------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQChannel;       //配置定时器5中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
	#endif

	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM5, TIM_FLAG_Update);                           // 清除TIM5溢出中断标志
 	
   	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);                      //TIM5溢出中断允许

}
#endif

#ifdef TIMER6_EN
/****************************************************************************
* 函数名称: Timer6Init_Base
* 功能描述: 定时器6初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer6Init_Base(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

	#ifdef TIMER6_INT_EN
	
	/*配置定时器6使能中断---------------------------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQChannel;       //配置定时器6中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
	#endif

	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM6, TIM_FLAG_Update);                           // 清除TIM6溢出中断标志
 	
   	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);                      //TIM6溢出中断允许

}
#endif

#ifdef TIMER7_EN
/****************************************************************************
* 函数名称: Timer7Init_Base
* 功能描述: 定时器7初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer7Init_Base(void)
{

	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);

	#ifdef TIMER7_INT_EN
	
	/*配置定时器7使能中断---------------------------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQChannel;       //配置定时器7中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
	#endif

	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM7, TIM_FLAG_Update);                           // 清除TIM7溢出中断标志
 	
   	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);                      //TIM7溢出中断允许

}

#endif

#ifdef TIMER8_EN
/****************************************************************************
* 函数名称: Timer8Init_Base
* 功能描述: 定时器8初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void Timer8Init_Base(void)
{

	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);

	#ifdef TIMER8_INT_EN
	
	/*配置定时器8使能中断---------------------------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQChannel;       //配置定时器8中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
	#endif

	TIM_TimeBaseStructure.TIM_Period = 0x5DC0;                      //最大计数值24000 产生１ＭＳ时基　t = Rperiod/(计数器时钟／(分频数加１))       
	TIM_TimeBaseStructure.TIM_Prescaler = 0x09;                     //0x90分频为分频数加1;      	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;                  // 时钟分割  	//计数器时钟为48MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //计数方向向上计数
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
 	                                                 
   	TIM_ClearFlag(TIM8, TIM_FLAG_Update);                           // 清除TIM8溢出中断标志
 	
   	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);                      //TIM8溢出中断允许

}
#endif


#ifdef RTC_EN
/****************************************************************************
* 函数名称: RTCInit_Base
* 功能描述: RTC初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void RTCInit_Base(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	#ifdef RTC_INT_EN
                                      //???????????????????????????????????????
	#endif
	/* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* 使能内部低速晶振 */
    RCC_LSICmd(ENABLE);
    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(39999); /* RTC period = RTCCLK/RTC_PR = (40 KHz)/(39999+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

}


#endif /*RTC_EN*/

#ifdef ADC1_EN
/****************************************************************************
* 函数名称: ADC1Init_Base
* 功能描述: ADC1初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void ADC1Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
	ADC_InitTypeDef ADC_InitStructure;
	/*使能用到的时钟资源*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA
	 						|RCC_APB2Periph_GPIOB
	 						|RCC_APB2Periph_GPIOC,
							ENABLE);
 
	/* ADC1 对应的16个采集通道配置 ------------------------------------*/
    /*输入端口A配置*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|
                                  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*输入端口B配置*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*输入端口C配置*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	#ifdef AD1_INT_EN
                                        //????????????????????????????????????????????????????????????????
	#endif

    /* ADC1 配置 ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 16;	                    //用于转换的通道数
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 规则模式通道配置 */ 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 5, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 8, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 9, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2,10, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1,11, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,12, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13,13, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12,14, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11,15, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10,16, ADC_SampleTime_55Cycles5);
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
  
    /* 启动ADC1 */
    ADC_Cmd(ADC1, ENABLE);


	// 下面是ADC自动校准，开机后需执行一次，保证精度
    /* 使能ADC1复位校正*/   
    ADC_ResetCalibration(ADC1);
    /* */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* 开始ADC1复位校正 */
    ADC_StartCalibration(ADC1);
    /* 检验是否校正完毕 */
    while(ADC_GetCalibrationStatus(ADC1));
	// ADC自动校准结束---------------
//     
//    /* 开启ADC1转换 */ 
//    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/****************************************************************************
* 函数名称: ADC1_Start
* 功能描述: ADC1启动
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void ADC1_Start(void)
{
	/* 开启ADC1转换 */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

#endif /*ADC1_EN*/



#ifdef DMA_EN

/****************************************************************************
* 函数名称: DMAInit_Base
* 功能描述: ADC1初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void DMAInit_Base(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	
	/*DMA1通道１配置 ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCConvertedValue[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;		  //dma传输方向单向
    DMA_InitStructure.DMA_BufferSize = 80;					  //设置DMA在传输时缓冲区的长度
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //设置DMA的外设递增模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		   //设置DMA的内存递增模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	 //转换结果的数据大小
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;					  //设置DMA的传输模式：连续不断的循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
    /* 使能DMA1通道１*/
    DMA_Cmd(DMA1_Channel1, ENABLE);

}
#endif /*ADC_EN*/

#ifdef FSMC_EN
/****************************************************************************
* 函数名称: FSMCInit_Base
* 功能描述: FSMC初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void FSMCInit_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  p;
  
/* 使能 FSMC 时钟及对应数据总线线、地址线总线、控制线的时钟源------------- */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD 
						   |RCC_APB2Periph_GPIOE 
						   |RCC_APB2Periph_GPIOF
						   |RCC_APB2Periph_GPIOG,
						   ENABLE);

/******DEFG口数据总线，地址总线配置相关、当使用FSMC外设时配置数据线、地址线***/
	/*数据总线配置共16根数据线*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/*地址总线配置共22根地址线*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; 
  	GPIO_Init(GPIOE, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | 
                                GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_Init(GPIOF, &GPIO_InitStructure);
  
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5;
  	GPIO_Init(GPIOG, &GPIO_InitStructure);


	/*NOE , NWE读写使能配置*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	/*NE1, NE2, NE3外部存储器片选配置 */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;			    //NE1
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 |GPIO_Pin_10; //NE2,NE3
	GPIO_Init(GPIOG, &GPIO_InitStructure);


	/*NBL0, NBL1字节读写控制配置*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	#ifdef FSMC_INT_EN
                                    //??????????????????????????????????????????????????????
	#endif
/*-- FSMC Configuration ---------------------------------------------------*/
/******************************外部RAM接口**********************************/
	p.FSMC_AddressSetupTime = 0;
	p.FSMC_AddressHoldTime = 0;
	p.FSMC_DataSetupTime = 2;
	p.FSMC_BusTurnAroundDuration = 0;
	p.FSMC_CLKDivision = 0;
	p.FSMC_DataLatency = 0;
	p.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsyncWait = FSMC_AsyncWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;


	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);


    /* Enable FSMC Bank1_SRAM Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);

}
#endif /*FSMC_EN*/


#ifdef USART1_EN
/****************************************************************************
* 函数名称: USARTInit_Base
* 功能描述: 串口初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void USART1Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 配置USART1的时钟--------------------------------------------------- */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); 
	
	/* 配置USART1的GPIO--------------------------------------------------- */
	/* 配置USART1 Tx (PA.09) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置USART1 Rx (PA.10) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	#ifdef USART1_INT_EN
	/* 配置USART1使能中断-------------------------------------------------- */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
    
    USART_InitStructure.USART_BaudRate = 9600;			   //初始化usart1参数配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);		  //使能发送中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);		  //使能接受中断

}

#endif /*USART1_EN*/
#ifdef USART2_EN
/****************************************************************************
* 函数名称: USARTInit_Base
* 功能描述: 串口初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void USART2Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 配置USART1的时钟--------------------------------------------------- */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* 配置USART1的GPIO--------------------------------------------------- */
	/* 配置USART2 Tx (PA.02)****/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//？？？？？？？要开启AFIO？？？？
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置USART2 Rx (PA.03) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	#ifdef USART2_INT_EN
	/* 配置USART2使能中断-------------------------------------------------- */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

    USART_InitStructure.USART_BaudRate = 9600;			   //初始化usart2参数配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, DISABLE);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);		  //使能发送中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);		  //使能接受中断

}

#endif /*USART2_EN*/


#ifdef CAN_EN
/****************************************************************************
* 函数名称: CANInit_Base
* 功能描述: CAN 初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void CANInit_Base(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
	NVIC_InitTypeDef       NVIC_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/*CAN用到的数据总线配置*/
	/* 配置 CAN－RX pin(PA.11)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	/* 配置 CAN-TX pin(PA.12) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	#ifdef CAN_INT_EN
	/* 使能CAN RX0 接收*/

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	CAN_ITConfig(CAN_IT_FMP0, ENABLE);
	#endif
	
    /* 配置CAN 单元  */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
    CAN_InitStructure.CAN_Prescaler = 5;
    CAN_Init(&CAN_InitStructure);

    /* 配置CAN 过滤器 共有14个下面为其中一个的配置 */
    CAN_FilterInitStructure.CAN_FilterNumber=0;
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

}

#endif /*CAN_EN*/


#ifdef SPI1_EN
/****************************************************************************
* 函数名称: SPI1Init_Base
* 功能描述: SPI1初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void SPI1Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;

	/* 配置SPI1时钟源 ----------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	
	/* 配置SPI1 引脚: SCK, MISO and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	#ifdef SPI1_INT_EN
                               //???????????????????????????????????????????????????/

	#endif

	/* SPI1 配置*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI1, &SPI_InitStructure);
}
/****************************************************************************
* 函数名称: SPI1_SendData
* 功能描述: SPI1数据发送函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void SPI1_SendData(u8 data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, data);
}

#endif /*SPI1_EN*/
#ifdef SPI2_EN
/****************************************************************************
* 函数名称: SPI2Init_Base
* 功能描述: SPI2初始化函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void SPI2Init_Base(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;//????????????????????????????????????????????????????????????/

	/* 配置SPI1时钟源 ----------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI2,ENABLE);

	/* 配置SPI2 引脚: SCK, MISO and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	#ifdef SPI2_INT_EN
                               //???????????????????????????????????????????????????/

	#endif

	/* SPI2 配置*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStructure);
	//此处添加代码
}

#endif /*SPI2_EN*/

#ifdef IWATCHDOG_EN
/****************************************************************************
* 函数名称: IWDGInit_Base
* 功能描述: 看门狗初始化配置函数
* 入口参数: 无
* 出口参数: 无
* 使用说明:
****************************************************************************/
void IWDGInit_Base(void)
{

	#ifdef IWATCHDOG_INT_EN

	#endif
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        RCC_ClearFlag();
    }
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  /*使能写IWDG_PR and IWDG_RLR*/
    
    IWDG_SetPrescaler(IWDG_Prescaler_64);          /* 看门狗时钟: 40KHz(LSI) / 64 = 0.6125 KHz */
    IWDG_SetReload(4095);                          /*定时2S*/
    IWDG_ReloadCounter();                   
    IWDG_Enable();                                 /*使能看门狗(the LSI oscillator will be enabled by hardware) */

}
#endif

/****************************************END OF FILE************************/

