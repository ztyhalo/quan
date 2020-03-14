/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#if defined(USE_OLED)
	#include "oled.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t DECA_WorkMode;
extern uint8_t TagExitWorkSpaceTimBegin;
extern uint8_t TagExitWorkSpace;
extern uint32_t TagExitWorkSpaceTIM;
extern uint32_t TagCommCycleTIM;

extern uint8_t SW_PressCnt;
extern uint8_t SystemResetTimBegin;
extern uint8_t ID_ConfigTimBegin;
extern uint32_t SystemResetTim;
extern uint32_t ID_ConfigTim;

extern OS_EVENT  *TagCommSem;
//extern OS_EVENT  *ID_ConfigSem;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */

//void PendSV_Handler(void)
//{
//  /* USER CODE BEGIN PendSV_IRQn 0 */

//  /* USER CODE END PendSV_IRQn 0 */
//  /* USER CODE BEGIN PendSV_IRQn 1 */

//  /* USER CODE END PendSV_IRQn 1 */
//}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    uint32_t systick;
  /* USER CODE BEGIN SysTick_IRQn 0 */
	OSIntEnter();                                    //进入中断
	OSTimeTick();                                    //调用ucos的时钟服务函数
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	switch(DECA_WorkMode)
	{
		case DECA_TAG:
			systick = HAL_GetTick();
			if((TagExitWorkSpaceTimBegin == 1) && (systick == TagExitWorkSpaceTIM))
			{
				TagExitWorkSpace = 1;
			}

//			if(systick == TagCommCycleTIM)
//			{
//				OSSemPost(TagCommSem);		//恢复标签任务
//			}

/******************************按键功能************************************/			
#if defined(PCB_V2)
			if((systick >= SystemResetTim) && (SystemResetTimBegin == 1))
			{
				NVIC_SystemReset();				//软件复位
			}
			
//			if((SW_PressCnt == 2) && (ID_ConfigTimBegin == 1))
//			{
//				SW_PressCnt = 0;
//				ID_ConfigTimBegin = 0;
//				OSSemPost(ID_ConfigSem);		//进入ID配置模式
//			}
			else if(systick == ID_ConfigTim)
			{
				SW_PressCnt = 0;
				ID_ConfigTimBegin = 0;
			}
#endif
/**************************************************************************/
			break;
	}
	
	OSIntExit();                                     //触发任务切换软中断
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void DWM1000_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	OSIntEnter();
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(D_IRQ_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	OSIntExit();
  /* USER CODE END EXTI0_IRQn 1 */
}


#if defined(STM32L412xx) && defined(PCB_V2)


extern ADC_HandleTypeDef hadc1;
void ADC1_2_IRQHandler(void)
{
	if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_AWD1) == 1)
	{
		__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD1);
	}
}


void CHARGE_WAKEUP_IRQHandler(void)
{
	OSIntEnter();
	HAL_GPIO_EXTI_IRQHandler(CHARGE_WAKEUP_Pin);
	OSIntExit();
}
#endif



/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
