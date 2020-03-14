#include "key.h"

#if defined(PCB_V2)

OS_EVENT  *KeyCheckSem;		//按键检测信号量
extern OS_EVENT  *ID_ConfigSem;

OS_STK			KeyCheckStk[OS_TASK_STAT_STK_SIZE];			//开栈

/*******************************************************************************************
* 函数名称：void KeyInit(void)
* 功能描述：按键初始化函数
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void KeyInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	/* 开关量输入 */
	GPIO_InitStruct.Pin = SW_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SW_PIN_Port, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(SW_EXTI_IRQ, 4, 0);
	HAL_NVIC_EnableIRQ(SW_EXTI_IRQ);
	
	KeyCheckSem = OSSemCreate(0);
	
	OSTaskCreateExt(KeyCheck,
									(void *)0,
									(OS_STK *)&KeyCheckStk[OS_TASK_STAT_STK_SIZE - 1],
									KEY_CHECK_PRIO,
									KEY_CHECK_PRIO,
									(OS_STK *)&KeyCheckStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
}


/*******************************************************************************************
* 函数名称：void SW_IRQHandler(void)
* 功能描述：按键中断服务函数
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void SW_IRQHandler(void)
{
	OSIntEnter();
	if(__HAL_GPIO_EXTI_GET_IT(SW_PIN) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(SW_PIN);
    OSSemPost(KeyCheckSem);
  }
	OSIntExit();
}


/*******************************************************************************************
* 函数名称：void KeyCheck(void *pdata)
* 功能描述：按键状态检测任务
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
uint8_t ChargeSleep;		//电池充电指示：0--未充电，1--充电中

uint8_t SW_PressCnt;					//SW按钮按下次数计数器

uint8_t SystemResetTimBegin;
uint8_t ID_ConfigTimBegin;
uint32_t SystemResetTim;			//系统强制复位判定计时器
uint32_t ID_ConfigTim;				//进入ID配置模式判定计时器

void KeyCheck(void *pdata)
{
	INT8U err;
	for(;;)
	{
		OSSemPend(KeyCheckSem,0,&err);
		if(HAL_GPIO_ReadPin(SW_PIN_Port, SW_PIN) == GPIO_PIN_SET)						//SW按下
		{
			SystemResetTimBegin = 1;	//进入系统强制复位判定
			SystemResetTim = HAL_GetTick() + SYSTEM_RESET_JUDGE_TIME_MS;
			
			ID_ConfigTimBegin = 1;		//进入ID配置判定
			ID_ConfigTim = HAL_GetTick() + ID_CONFIG_JUDGE_TIME_MS;
		}
		else																																//SW松开
		{
			SystemResetTimBegin = 0;	//退出系统强制复位判定
			SW_PressCnt++;
			
			if((SW_PressCnt == 2) && (ID_ConfigTimBegin == 1))
			{
				SW_PressCnt = 0;
				ID_ConfigTimBegin = 0;
				OSSemPost(ID_ConfigSem);		//进入ID配置模式
			}
		}
	}
}


#endif



