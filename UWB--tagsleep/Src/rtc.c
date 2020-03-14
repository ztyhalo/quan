#include "rtc.h"
#include "spi.h"

RTC_HandleTypeDef RTC_InitStruct;
EXTI_HandleTypeDef  EXTI_Handle = {0};
void RTC_Config(void)
{
	EXTI_ConfigTypeDef	EXTI_Config = {0};
	
	__HAL_RCC_RTC_ENABLE();
	
	RTC_InitStruct.Instance = RTC;
	RTC_InitStruct.Init.HourFormat = RTC_HOURFORMAT_24;
	RTC_InitStruct.Init.AsynchPrediv = 0x7F;
	RTC_InitStruct.Init.SynchPrediv = 0xFF;
	RTC_InitStruct.Init.OutPut = RTC_OUTPUT_WAKEUP;
	RTC_InitStruct.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	RTC_InitStruct.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RTC_InitStruct.Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;
	if(HAL_RTC_Init(&RTC_InitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	
	/*配置外部中断线*/
	EXTI_Config.Line = EXTI_LINE_20;
	EXTI_Config.Mode = EXTI_MODE_INTERRUPT;
	EXTI_Config.Trigger = EXTI_TRIGGER_RISING;
	HAL_EXTI_SetConfigLine(&EXTI_Handle, &EXTI_Config);
	
	/*配置中断优先级*/
	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 5, 0);			//抢占优先级9，子优先级0
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);				    //使能USART1中断通道
	
	__HAL_RTC_WAKEUPTIMER_DISABLE(&RTC_InitStruct);
//	HAL_RTCEx_SetWakeUpTimer_IT(&RTC_InitStruct, 199, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 30);
}

void RTC_WKUP_IRQHandler()
{
	OSIntEnter();
	if(__HAL_RTC_WAKEUPTIMER_GET_FLAG(&RTC_InitStruct, RTC_FLAG_WUTF) != 0U)
	{
		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RTC_InitStruct, RTC_FLAG_WUTF);
		__HAL_RTC_WAKEUPTIMER_DISABLE(&RTC_InitStruct);
	}
	HAL_EXTI_ClearPending(&EXTI_Handle, EXTI_TRIGGER_RISING);
	HAL_GPIO_TogglePin(LED3_LED4_PIN_Port, LED3_PIN);
	OSIntExit();
}





