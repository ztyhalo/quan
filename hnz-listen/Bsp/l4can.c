/*! ----------------------------------------------------------------------------
 * @file	l4can.c
 * @brief	stm32l452 can 驱动
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include "l4can.h"

HAL_StatusTypeDef     gCanState;

CAN_HandleTypeDef     CanHandle;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

uint32_t              gTclk;

void CanError_Handler(void)
{
	while(1)
	{
		;
	}
}

/*******************************************************************************************
* 函数名称：void CanGpioInit(void)
* 功能描述：can gpio 初始化
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/

void CanGpioInit(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*1.时钟使能*/
  CANx_CLK_ENABLE();
  CANx_GPIO_CLK_ENABLE();

	/*2.gpio配置*/
  GPIO_InitStruct.Pin = CANx_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate =  CANx_TX_AF;

  HAL_GPIO_Init(CANx_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CANx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate =  CANx_RX_AF;

  HAL_GPIO_Init(CANx_RX_GPIO_PORT, &GPIO_InitStruct);
	
	/*配置NVIC*/
	HAL_NVIC_SetPriority(CANx_RX_IRQn, CAN_RX_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(CANx_RX_IRQn);
		
}


/*******************************************************************************************
* 函数名称：void CanGpioInit(void)
* 功能描述：can gpio 初始化
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/

void CAN_Config(uint32_t speed)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CANx;

  CanHandle.Init.TimeTriggeredMode = DISABLE;
  CanHandle.Init.AutoBusOff = DISABLE;
  CanHandle.Init.AutoWakeUp = ENABLE;
  CanHandle.Init.AutoRetransmission = ENABLE;
  CanHandle.Init.ReceiveFifoLocked = DISABLE;
  CanHandle.Init.TransmitFifoPriority = ENABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  CanHandle.Init.TimeSeg1 = CAN_BS1_3TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
	gTclk = HAL_RCC_GetPCLK1Freq();
  CanHandle.Init.Prescaler = gTclk / 1000 / speed / (1+ 3 + 2);

  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initialization Error */
    CanError_Handler();
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    CanError_Handler();
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&CanHandle) != HAL_OK)
  {
    /* Start Error */
    CanError_Handler();
  }

  /*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    CanError_Handler();
  }
 
}



/*******************************************************************************************
* 函数名称：void StartCanRx(void)
* 功能描述：
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void StartCanRx(void)
{
//	uint32_t interrupts = READ_REG(hcan->Instance->IER);
	if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    CanError_Handler();
  }

}


/*******************************************************************************************
* 函数名称：void CanGpioInit(void)
* 功能描述：can gpio 初始化
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void CANx_RX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&CanHandle);
}


__weak void CAN_BusRxDataCallback(uint16_t canid, CAN_RxHeaderTypeDef head, uint8_t * data)
{
	 UNUSED(canid);
	 UNUSED(head);
	 UNUSED(data);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    CanError_Handler();
  }
	else
	{
		CAN_BusRxDataCallback(0, RxHeader, RxData);   //此为can1的回调函数
	}

  /* Display LEDx */
//  if ((RxHeader.StdId == 0x321) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  {
//    LED_Display(RxData[0]);
//    ubKeyNumber = RxData[0];
  }
}



/*********************************************************

************************************************************/
int32_t STM32_CAN_Write(uint16_t  canid, sCAN_FRAME txMessage, uint16_t frametype)
{
	if(frametype == CAN_ID_STD)
		TxHeader.StdId = txMessage.Stdid;
	else
		TxHeader.ExtId = txMessage.Stdid;
	TxHeader.IDE = frametype;
	TxHeader.DLC = txMessage.DLC;
	memcpy(TxData, txMessage.Data, txMessage.DLC);
	gCanState = HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox);
	if (gCanState != HAL_OK)
  {
    /* Reception Error */
		if(CanHandle.ErrorCode == HAL_CAN_ERROR_PARAM)
			return 1;
    CanError_Handler();
		
		
  }
	//return HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox);
	return 0;
}






