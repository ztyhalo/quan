/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                
*
* ��  �� :���г�ʼ����������ȡ���
* �ļ��� : target.c
* ��  �� : 
* ��  �� : V1.0
* ��  �� :
*********************************************************************************************
*/

#include "include.h"
//#include "target.h"
//#include "voice.h"
/********************************************************************************************
*                                              ��������
********************************************************************************************/
void RCCConfiguration(void);
void NVICConfiguration(void);
void GPIOConfiguration(void);
void I2S_CODEC_Init(void);
void IWDG_Configuration(void);
/********************************************************************************************
* �������ƣ�TargetInit
* ����������Ƭ�������ʼ��
* ��ڲ�������
* ���ڲ������� 
* ʹ��˵������
********************************************************************************************/
void TargetInit(void)
{
	RCCConfiguration();							// ϵͳʱ�ӳ�ʼ�� 
	NVICConfiguration();						// �������жϳ�ʼ��
    GPIOConfiguration();						// IO��ʼ��
	SD_Init();                      //sd����ʼ��
	I2S_CODEC_Init();               //I2S��ʼ��
	//IWDG_Configuration();
}
/*******************************************************************************************
* �������ƣ�RCCConfiguration
* ����������RCC��ʼ��,����ϵͳʱ��Դ
* ��ڲ�������
* ���ڲ������� 
* ʹ��˵������
********************************************************************************************/
void RCCConfiguration(void)
{
    ErrorStatus HSEStartUpStatus;  
    //RCC����ΪĬ��ֵ
    RCC_DeInit();
    //ʹ���ⲿʱ��
    RCC_HSEConfig(RCC_HSE_ON);
    //�ȴ��ⲿʱ����׼����
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
	//����ⲿʱ���Ѿ�׼����
    if (HSEStartUpStatus == SUCCESS)
    {
        //ʹ��Ԥ�滺����
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        // Flash 2 wait state ���ô�����ʱʱ��Ϊ2��ʱ������
        //SYSCLK(ϵͳʱ��)�������������ʱ��ı���
        //FLASH_Latency_0:��ȴ�״̬,��0<SYSCLK��24MHz
        //FLASH_Latency_1:һ���ȴ�״̬,��24MHz<SYSCLK��48MHz
        //FLASH_Latency_2:�����ȴ�״̬,��48MHz<SYSCLK��72MHz
        FLASH_SetLatency(FLASH_Latency_2);
        //HCLK����ϵͳʱ��
        RCC_HCLKConfig(RCC_SYSCLK_Div1); 
        //APB1ʱ�ӵ���HCLKʱ��
        RCC_PCLK2Config(RCC_HCLK_Div1); 
        //APB2ʱ�ӵ���HCLKʱ��2��Ƶ
        RCC_PCLK1Config(RCC_HCLK_Div2);
		//����ǻ����Ͳ�Ʒ��PLLʱ����ͨ��PLL2�õ�
	#ifdef STM32F10X_CL
    #ifndef Product_Mode
        //��4��Ƶ ��9��Ƶ PLL2 configuration: PLL2CLK = (HSE / 4) * 9 = 36 MHz ��HSE=16MHZ
        RCC_PREDIV2Config(RCC_PREDIV2_Div4);
        RCC_PLL2Config(RCC_PLL2Mul_9);
        // Enable PLL2 ʹ��PLL2
        RCC_PLL2Cmd(ENABLE);
        //Wait till PLL2 is ready �ȴ���PLL2׼����
        while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
        {}
		//PLLʱ��Դѡ��PLL2��4��Ƶ
        RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div4);
        // 8��Ƶ PLL configuration: PLLCLK = (PLL1 / 4) * 8 = 72 MHz  
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_8);
    #else
        // PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
        RCC_PREDIV2Config(RCC_PREDIV2_Div5);
        RCC_PLL2Config(RCC_PLL2Mul_8);
        //Enable PLL2 ʹ��PLL2
        RCC_PLL2Cmd(ENABLE);
        //Wait till PLL2 is ready �ȴ���PLL2׼����
        while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
        {}
        RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
        // PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz 
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
    #endif
	    //������ǻ����Ͳ�Ʒ��ʱ��Դ����hse��Ƶ
#else
	    // PLLCLK = 8MHz * 9 = 72 MHz */
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
#endif

        //Enable PLLʹ�� 
        RCC_PLLCmd(ENABLE);
        //�ȴ�PLL׼����
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {}
        // ѡ��PLL��Ϊϵͳʱ��
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        //�ȴ�PLL�Ѿ���Ϊ��ϵͳʱ��
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
* ��  ��:	void NVICConfiguration(void)
* ��  ��:	��
* ����ֵ:	��
* ��  ��:	�������ж�����
* ��  ע:	
************************************************************************************************/
void NVICConfiguration(void)
{
//	NVIC_InitTypeDef NVIC_InitStructure;
    	
	// �����ж�������ռ�
#ifdef  VECT_TAB_RAM  
  	/* Set the Vector Table base location at 0x20000000 */ 
  	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  	/* Set the Vector Table base location at 0x08000000 */ 
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

    /* Configure one bit for preemption priority */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

//    /*���ö�ʱ��2ʹ���ж�-------------------------------------------------------*/
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);


}

 /************************************************************************************************
* ��  ��:	void GPIOConfiguration(void)
* ��  ��:	��
* ����ֵ:	��
* ��  ��:	�������ж�����
* ��  ע:	
************************************************************************************************/
void GPIOConfiguration(void)
{
  	 GPIO_InitTypeDef GPIO_InitStructure; 
	 /*ʹ��ϵͳ���õ���ʱ����Դ*/
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
	 						|RCC_APB2Periph_GPIOB
	 						|RCC_APB2Periph_GPIOC
							|RCC_APB2Periph_GPIOD 
							|RCC_APB2Periph_GPIOE 
							|RCC_APB2Periph_AFIO	        // ��������IO(���ù���IO
							|RCC_APB2Periph_SPI1
							|RCC_APB1Periph_SPI2
	
							 ,ENABLE);
   

/****************************B���������**********************************************************************/
	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

/**************************************************************************************************************/

    	
}


/********************************************************************************************
* �������ƣ�IWDG_Configuration ()
* �������������Ź���ʼ��
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void IWDG_Configuration(void)
{
  /* ʹ�ܼĴ��� IWDG_PR �� IWDG_RLR д���� */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* ���Ź���Ƶʱ��Ϊ: LSI/256 */
  IWDG_SetPrescaler(IWDG_Prescaler_128);

  /* ���ÿ��Ź�������ֵ.20S*/
  IWDG_SetReload(0x080);

  /* ι�� */
  IWDG_ReloadCounter();

  /* ʹ�ܿ��Ź� */
  IWDG_Enable();
}
/***********************************************************************************************
* Name:		UserTick
* Input:	
* Output:
* Function:	Ӧ�ó�������¼�����
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
* Function:	Ӧ�ó���λ
* Notes:	
***********************************************************************************************/
// void sysreset(void)
// {
// 	__set_FAULTMASK(1);  //����ʹ�����ȼ���-1       
// 	NVIC_SystemReset(); //��λcpu        
// }
/***********************************************************************************************
* Name:		PlayTick
* Input:	
* Output:
* Function:	Ӧ�ó�������¼�����
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
* Function:	ϵͳ��ʱ������
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
