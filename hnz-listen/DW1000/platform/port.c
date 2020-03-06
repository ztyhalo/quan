/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "sleep.h"
#include "lcd.h"
#include "port.h"
#include "main.h"




/* DW1000 IRQ handler definition. */
port_DecaISR_f fpport_Deca_ISR = NULL;





int No_Configuration(void)
{
	return -1;
}

unsigned long portGetTickCnt(void)
{
	return uwTick;
}


void Dw100GpioInit(void)
{
	
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	UWB_RST_GPIO_CLK_ENABLE();
	UWB_WEAK_GPIO_CLK_ENABLE();
	UWB_CS_GPIO_CLK_ENABLE();
	UWB_IRQ_GPIO_CLK_ENABLE();
	
	  /*Configure GPIO pin : 复位 */
	 
  GPIO_InitStruct.Pin = UWB_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UWB_RST_GPORT, &GPIO_InitStruct);

  /*Configure GPIO pin : spi 片选 */
  GPIO_InitStruct.Pin = UWB_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(UWB_CS_GPORT, &GPIO_InitStruct);

  /*Configure GPIO pin : dw1000 中断 */
  GPIO_InitStruct.Pin = UWB_IRQ_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(UWB_IRQ_GPORT, &GPIO_InitStruct);

  /*Configure GPIO pin : 唤醒 */
  GPIO_InitStruct.Pin = UWB_WEAK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(UWB_WEAK_GPORT, &GPIO_InitStruct);
	
	  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(UWB_EXTI_IRQn, UWB_EXTI_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(UWB_EXTI_IRQn);
	
		/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UWB_CS_GPORT, UWB_CS_PIN, GPIO_PIN_SET);
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UWB_WEAK_GPORT, UWB_WEAK_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
	ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
  /* Check the parameters */

  enablestatus =  EXTI->IMR1 & EXTI_Line;
  if (enablestatus != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;

}



/*******************************************************************************************
* 函数名称：Reset_DW1000()
* 功能描述：DW1000复位函数
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void Reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO used for DW1000 reset
	GPIO_InitStruct.Pin = UWB_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(UWB_RST_GPORT, &GPIO_InitStruct);
	
	//drive the RSTn pin low 10ns at least for reset DW1000
	HAL_GPIO_WritePin(UWB_RST_GPORT, UWB_RST_PIN, GPIO_PIN_RESET);

	//put the pin back to tri-state ... as input
	GPIO_InitStruct.Pin = UWB_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UWB_RST_GPORT, &GPIO_InitStruct);

  sleep_ms(2);
}

/*******************************************************************************************
* 函数名称：SPI_ChangeRate()
* 功能描述：SPI1波特率更改
* 入口参数：波特率分频系数
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void SPI_ChangeRate(uint16_t ScalingFactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPI1->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= ScalingFactor;

	/* Write to SPIx CR1 */
	SPI1->CR1 = tmpreg;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn SPI_SetRateLow()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void SPI_SetRateLow (void)
{
    SPI_ChangeRate(SPI_BAUDRATEPRESCALER_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn SPI_SetRateHigh()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void SPI_SetRateHigh (void)
{
    SPI_ChangeRate(SPI_BAUDRATEPRESCALER_4);
}




/**
  * @brief This function handles EXTI line0 interrupt.
  */
void DWM1000_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	OSIntEnter();
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(UWB_IRQ_PIN);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	OSIntExit();
  /* USER CODE END EXTI0_IRQn 1 */
}







