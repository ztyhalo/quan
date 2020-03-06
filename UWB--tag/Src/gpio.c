/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
#include "deca_regs.h"
#include "deca_device_api.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

extern uint8_t DECA_WorkMode;
/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(D_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(D_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(D_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(D_WAKEUP_GPIO_Port, &GPIO_InitStruct);
	
	/*RS485发送接收控制脚*/
//	GPIO_InitStruct.Pin = D_RS485_TX_Ctrl_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(D_RS485_TX_Ctrl_GPIO_Port, &GPIO_InitStruct);
	
	/* 状态指示灯D3/D4 */
	GPIO_InitStruct.Pin = LED3_PIN | LED4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED3_LED4_PIN_Port, &GPIO_InitStruct);
	
	/* 状态指示灯D1 */
	GPIO_InitStruct.Pin = LED1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LED1_PIN_Port, &GPIO_InitStruct);

#if defined(STM32L412xx) && defined(PCB_V2)
	
	
	/* 电池充电指示开关量 */
	GPIO_InitStruct.Pin = CHARGE_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CHARGE_WAKEUP_GPIO_Port, &GPIO_InitStruct);
	
	/*LTC3401模式切换*/
	GPIO_InitStruct.Pin = LTC3401_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LTC3401_MODE_Port, &GPIO_InitStruct);

#endif
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(D_IRQ_EXTI_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(D_IRQ_EXTI_IRQn);
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D_CS_GPIO_Port, D_CS_Pin, GPIO_PIN_SET);
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D_WAKEUP_GPIO_Port, D_WAKEUP_Pin, GPIO_PIN_RESET);
	
	/*点亮指示灯*/
#if defined(STM32L412xx) && defined(PCB_V2)
	HAL_GPIO_WritePin(LED_PIN_Port, LED5_PIN | LED6_PIN, GPIO_PIN_RESET);
	LTC3401_BURST_MODE_DISABLE;					//关闭Burst模式
#else
	HAL_GPIO_WritePin(LED3_LED4_PIN_Port, LED3_PIN | LED4_PIN, GPIO_PIN_RESET);
#endif
}

/* USER CODE BEGIN 2 */
/*******************************************************************************************
* 函数名称：HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* 功能描述：GPIO外部中断回调函数
* 入口参数：GPIO_Pin
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case D_IRQ_Pin:
			if(DWT_Read32BitReg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_TXFRS | SYS_STATUS_ALL_RX_ERR))
			{
				if(DECA_WorkMode == DECA_ANCHOR)
				{
					OSTaskResume(UWB_ANCHOR_PRIO);
				}
				else if(DECA_WorkMode == DECA_TAG)
				{
					OSTaskResume(UWB_TAG_PRIO);
				}
			}
			break;
#if defined(STM32L412xx) && defined(PCB_V2)
		case CHARGE_WAKEUP_Pin:
			if(HAL_GPIO_ReadPin(CHARGE_WAKEUP_GPIO_Port, CHARGE_WAKEUP_Pin) == GPIO_PIN_SET)
			{
				
			}
			else
			{
				
			}
#endif
	}
	
}

/*******************************************************************************************
* 函数名称：LED_Blink(void *pdata)
* 功能描述：OS系统运行指示灯（绿色）
* 入口参数：*pdata
* 出口参数：无
* 使用说明：无
********************************************************************************************/
#if defined(STM32L412xx) && defined(PCB_V2)
extern ADC_HandleTypeDef hadc1;
void LED_Blink(void *pdata)
{
	for(;;)
	{
		OSTimeDly(1000);
		HAL_ADC_Start(&hadc1);
    HAL_GPIO_TogglePin(LED_PIN_Port, LED6_PIN);
	}
}
#else
void LED_Blink(void *pdata)
{
	for(;;)
	{
		OSTimeDly(1000);
    HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED3_PIN);
	}
}
#endif

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
