/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                
*
* 描  述 :进行初始化操作，读取版号
* 文件名 : target.c
* 作  者 : 
* 版  本 : V1.0
* 日  期 :
*********************************************************************************************
*/

#include "include.h"
//#include "target.h"
//#include "voice.h"
/********************************************************************************************
*                                              函数声明
********************************************************************************************/
void RCCConfiguration(void);
void NVICConfiguration(void);
void GPIOConfiguration(void);
void I2S_CODEC_Init(void);
void IWDG_Configuration(void);
/********************************************************************************************
* 函数名称：TargetInit
* 功能描述：片上外设初始化
* 入口参数：无
* 出口参数：无 
* 使用说明：无
********************************************************************************************/
void TargetInit(void)
{
	RCCConfiguration();							// 系统时钟初始化 
	NVICConfiguration();						// 可屏蔽中断初始化
    GPIOConfiguration();						// IO初始化
	SD_Init();                      //sd卡初始化
	I2S_CODEC_Init();               //I2S初始化
	//IWDG_Configuration();
}
/*******************************************************************************************
* 函数名称：RCCConfiguration
* 功能描述：RCC初始化,配置系统时钟源
* 入口参数：无
* 出口参数：无 
* 使用说明：无
********************************************************************************************/
void RCCConfiguration(void)
{
    ErrorStatus HSEStartUpStatus;  
    //RCC设置为默认值
    RCC_DeInit();
    //使能外部时钟
    RCC_HSEConfig(RCC_HSE_ON);
    //等待外部时钟设准备好
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
	//如果外部时钟已经准备好
    if (HSEStartUpStatus == SUCCESS)
    {
        //使能预存缓冲区
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        // Flash 2 wait state 设置代码延时时间为2个时钟周期
        //SYSCLK(系统时钟)周期与闪存访问时间的比例
        //FLASH_Latency_0:零等待状态,当0<SYSCLK≤24MHz
        //FLASH_Latency_1:一个等待状态,当24MHz<SYSCLK≤48MHz
        //FLASH_Latency_2:两个等待状态,当48MHz<SYSCLK≤72MHz
        FLASH_SetLatency(FLASH_Latency_2);
        //HCLK等于系统时钟
        RCC_HCLKConfig(RCC_SYSCLK_Div1); 
        //APB1时钟等于HCLK时钟
        RCC_PCLK2Config(RCC_HCLK_Div1); 
        //APB2时钟等于HCLK时钟2分频
        RCC_PCLK1Config(RCC_HCLK_Div2);
		//如果是互联型产品，PLL时钟是通过PLL2得到
	#ifdef STM32F10X_CL
    #ifndef Product_Mode
        //先4分频 再9倍频 PLL2 configuration: PLL2CLK = (HSE / 4) * 9 = 36 MHz ，HSE=16MHZ
        RCC_PREDIV2Config(RCC_PREDIV2_Div4);
        RCC_PLL2Config(RCC_PLL2Mul_9);
        // Enable PLL2 使能PLL2
        RCC_PLL2Cmd(ENABLE);
        //Wait till PLL2 is ready 等待到PLL2准备好
        while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
        {}
		//PLL时钟源选择PLL2，4分频
        RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div4);
        // 8倍频 PLL configuration: PLLCLK = (PLL1 / 4) * 8 = 72 MHz  
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_8);
    #else
        // PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
        RCC_PREDIV2Config(RCC_PREDIV2_Div5);
        RCC_PLL2Config(RCC_PLL2Mul_8);
        //Enable PLL2 使能PLL2
        RCC_PLL2Cmd(ENABLE);
        //Wait till PLL2 is ready 等待到PLL2准备好
        while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
        {}
        RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
        // PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz 
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
    #endif
	    //如果不是互联型产品，时钟源来自hse倍频
#else
	    // PLLCLK = 8MHz * 9 = 72 MHz */
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
#endif

        //Enable PLL使能 
        RCC_PLLCmd(ENABLE);
        //等待PLL准备好
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {}
        // 选择PLL作为系统时钟
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        //等待PLL已经作为了系统时钟
        while(RCC_GetSYSCLKSource() != 0x08)
        {}
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
       User can add here some code to deal with this error */    
    /* Go to infinite loop */
     while (1)
     {
     }
  }
}
/************************************************************************************************
* 名  称:	void NVICConfiguration(void)
* 参  数:	无
* 返回值:	无
* 功  能:	可屏蔽中断配置
* 备  注:	
************************************************************************************************/
void NVICConfiguration(void)
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

    /* Configure one bit for preemption priority */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

//    /*配置定时器2使能中断-------------------------------------------------------*/
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);


}

 /************************************************************************************************
* 名  称:	void GPIOConfiguration(void)
* 参  数:	无
* 返回值:	无
* 功  能:	可屏蔽中断配置
* 备  注:	
************************************************************************************************/
void GPIOConfiguration(void)
{
  	 GPIO_InitTypeDef GPIO_InitStructure; 
	 /*使能系统中用到的时钟资源*/
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
	 						|RCC_APB2Periph_GPIOB
	 						|RCC_APB2Periph_GPIOC
							|RCC_APB2Periph_GPIOD 
							|RCC_APB2Periph_GPIOE 
							|RCC_APB2Periph_AFIO	        // 辅助功能IO(复用功能IO
							|RCC_APB2Periph_SPI1
							|RCC_APB1Periph_SPI2
	
							 ,ENABLE);
   

/****************************B口配置相关**********************************************************************/
	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

/**************************************************************************************************************/

    	
}


/********************************************************************************************
* 函数名称：IWDG_Configuration ()
* 功能描述：看门狗初始化
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void IWDG_Configuration(void)
{
  /* 使能寄存器 IWDG_PR 和 IWDG_RLR 写访问 */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* 看门狗分频时钟为: LSI/256 */
  IWDG_SetPrescaler(IWDG_Prescaler_128);

  /* 设置看门狗的重载值.20S*/
  IWDG_SetReload(0x080);

  /* 喂狗 */
  IWDG_ReloadCounter();

  /* 使能看门狗 */
  IWDG_Enable();
}
/***********************************************************************************************
* Name:		UserTick
* Input:	
* Output:
* Function:	应用程序节拍事件处理
* Notes:	
***********************************************************************************************/
void UserTick(void)
{
    if(SoftTimer.PlayIntervalTimer > 0)
    {
        SoftTimer.PlayIntervalTimer--;
    }

    if(SoftTimer.OffLineTimer > 0)
    {
        SoftTimer.OffLineTimer--;
        
    }
    else
    {
        sysreset();
    }

}
/***********************************************************************************************
* Name:		sysreset
* Input:	
* Output:
* Function:	应用程序复位
* Notes:	
***********************************************************************************************/
// void sysreset(void)
// {
// 	__set_FAULTMASK(1);  //提升使能优先级到-1       
// 	NVIC_SystemReset(); //复位cpu        
// }
/***********************************************************************************************
* Name:		PlayTick
* Input:	
* Output:
* Function:	应用程序节拍事件处理
* Notes:	
***********************************************************************************************/
void PlayTick(void)
{
    if(SoftTimer.PlayIntervalTimer > 0)
    {
        SoftTimer.PlayIntervalTimer--;
        
    }
}
/***********************************************************************************************
* Name:		void SysTickConfiguartion(void)
* Input:	
* Output:
* Function:	系统定时器配置
* Notes:	
***********************************************************************************************/
void SysTickConfiguartion(void)
{
	SysTick_Config(SystemCoreClock / 1000);
}
/*
*********************************************************************************************************
*                                            BSP_CPU_ClkFreq()
*
* Description : Read CPU registers to determine the CPU clock frequency of the chip.
*
* Argument(s) : none.
*
* Return(s)   : The CPU clock frequency, in Hz.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT32U  BSP_CPU_ClkFreq (void)
{
    RCC_ClocksTypeDef  rcc_clocks;


    RCC_GetClocksFreq(&rcc_clocks);

    return ((CPU_INT32U)rcc_clocks.HCLK_Frequency);
}



/*
*********************************************************************************************************
*********************************************************************************************************
*                                         OS CORTEX-M3 FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                         OS_CPU_SysTickClkFreq()
*
* Description : Get system tick clock frequency.
*
* Argument(s) : none.
*
* Return(s)   : Clock frequency (of system tick).
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

INT32U  OS_CPU_SysTickClkFreq (void)
{
    INT32U  freq;


    freq = BSP_CPU_ClkFreq();
    return (freq);
}






/**************************************************end of file********************************************/
