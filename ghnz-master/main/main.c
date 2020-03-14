/**
  ******************************************************************************
  * 文件名称: main.c
  * 作    者: 张雨生
	* 当前版本：V1.0
	* 完成日期：
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





/*创建任务堆栈，容量256*/
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
* 函数名称：void SystemClock_Config(void)
* 功能描述：系统时钟配置
* 入口参数：无
* 出口参数：无
* 使用说明：无
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
* 函数名称：TaskStart(void *pdata)
* 功能描述：OS系统运行的第一个任务，负责外设初始化及其它任务的创建
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TaskStart(void *pdata)
{
	/* 复位所有外设，配置Systick周期1ms */
  HAL_Init();
  /* 配置系统时钟*/
  SystemClock_Config();
  /* 初始化外设 */
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
	OLED_Init();			//初始化OLED  
	OLED_Clear();
#endif
	
	SPI_SetRateLow();																	//降低SPI时钟频率72/32MHz（Init状态下，SPICLK不超过3MHz）
	if (DWT_Initialise(DWT_LOADUCODE) == DWT_ERROR)		//初始化DW1000
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
	CanAppInit();             												//初始化can通信并建立收发任务
	
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
	CanAppInit();         												//初始化can通信并建立收发任务
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
	OSInit();           //初始化OS系统
	
	OSTaskCreateExt(TaskStart,
									(void *)0,
									(OS_STK *)&TaskStartStk[APPMNG_TASK_STK_SIZE - 1],
									APPMNG_TASK_START_PRIO,
									APPMNG_TASK_START_PRIO,
									(OS_STK *)&TaskStartStk[0],
									APPMNG_TASK_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSStart();         //开始运行OS系统
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
