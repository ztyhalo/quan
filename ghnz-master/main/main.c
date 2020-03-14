/**
  ******************************************************************************
  * �ļ�����: main.c
  * ��    ��: ������
	* ��ǰ�汾��V1.0
	* ������ڣ�
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"
#include "app_cfg.h"
#include "l4spi.h"
#include "usart.h"

#include "lcd.h"
#include "port.h"
#include "sleep.h"

#if defined(USE_OLED)
	#include "oled.h"
#endif
#include "uwb_app.h"
#include "dw1000_bus.h"
#include "spi_bus.h"
#include "can_app.h"
#if DECA_WORK_MODE == UWB_ALARM_MODE
#include "uwb_anchor.h"
#endif /*DECA_WORK_MODE == UWB_ALARM_MODE */





/*���������ջ������256*/
OS_STK      TaskStartStk[APPMNG_TASK_STK_SIZE];
#if DECA_WORK_MODE == UWB_LISTEN_MODE
OS_STK      UWB_RxStk[UWBRX_TASK_STK_SIZE];
#elif DECA_WORK_MODE == UWB_ALARM_MODE
OS_STK      UWB_AnchorStk[UWB_ANCHOR_TASK_STK_SIZE];
#else
OS_STK      UWB_AnchorStk[UWB_ANCHOR_TASK_STK_SIZE];
#endif  /*DECA_WORK_MODE == UWB_LISTEN_MODE*/


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	OSTimeDly(1000);
}

/*******************************************************************************************
* �������ƣ�void SystemClock_Config(void)
* ����������ϵͳʱ������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
#if defined(STM32L412xx)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	
#elif defined(STM32L452xx)
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}




/*******************************************************************************************
* �������ƣ�TaskStart(void *pdata)
* ����������OSϵͳ���еĵ�һ�����񣬸��������ʼ������������Ĵ���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TaskStart(void *pdata)
{
	/* ��λ�������裬����Systick����1ms */
  HAL_Init();
  /* ����ϵͳʱ��*/
  SystemClock_Config();
  /* ��ʼ������ */
	Dw1000Init();
//	MX_GPIO_Init();
//	Dw1000Init();
	Reset_DW1000();	
  SpiBusInit();
	
//	 MX_GPIO_Init();
//	 Reset_DW1000();	
//  MX_USART1_UART_Init();
//  MX_SPI1_Init();

//	SpiBusDataInit();
#if defined(USE_OLED)	
	OLED_Init();			//��ʼ��OLED  
	OLED_Clear();
#endif
	
	SPI_SetRateLow();																	//����SPIʱ��Ƶ��72/32MHz��Init״̬�£�SPICLK������3MHz��
	if (DWT_Initialise(DWT_LOADUCODE) == DWT_ERROR)		//��ʼ��DW1000
	{
#if defined(USE_OLED)
		OLED_ShowString(0, 2, "Failed...", 16);
#endif
		while (1)
		{
			Error_Handler();
		}
	}
#if DECA_WORK_MODE == UWB_LISTEN_MODE
	CanAppInit();             												//��ʼ��canͨ�Ų������շ�����
	
	OSTaskCreateExt(UwbReceiveTask,
									(void *)0,
									(OS_STK *)&UWB_RxStk[UWBRX_TASK_STK_SIZE - 1],
									UWBRX_TASK_PRIO,
									UWBRX_TASK_PRIO,
									(OS_STK *)&UWB_RxStk[0],
									UWBRX_TASK_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

#elif DECA_WORK_MODE == UWB_ALARM_MODE
	CanAppInit();         												//��ʼ��canͨ�Ų������շ�����
	OSTaskCreateExt(UwbAnchorIrqTask,
									(void *)0,
									(OS_STK *)&UWB_AnchorStk[UWB_ANCHOR_TASK_STK_SIZE - 1],
									UWB_ANCHOR_TASK_PRIO,
									UWB_ANCHOR_TASK_PRIO,
									(OS_STK *)&UWB_AnchorStk[0],
									UWB_ANCHOR_TASK_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
#else
									

#endif /*DECA_WORK_MODE == UWB_LISTEN_MODE*/

	OSTaskDel(OS_PRIO_SELF);

}




int main(void)
{
	OSInit();           //��ʼ��OSϵͳ
	
	OSTaskCreateExt(TaskStart,
									(void *)0,
									(OS_STK *)&TaskStartStk[APPMNG_TASK_STK_SIZE - 1],
									APPMNG_TASK_START_PRIO,
									APPMNG_TASK_START_PRIO,
									(OS_STK *)&TaskStartStk[0],
									APPMNG_TASK_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSStart();         //��ʼ����OSϵͳ
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
