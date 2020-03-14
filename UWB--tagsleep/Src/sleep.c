#include "uwbsleep.h"
#include "rtc.h"

OS_STK		UWBSleepStk[OS_TASK_STAT_STK_SIZE];

OS_EVENT  *UWB_SleepSem;		//ID配置信号量

extern uint32_t TagCommCycleTIM;
extern uint8_t ChargeSleep;
extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc1;
extern __IO uint32_t uwTick;

int8_t DWT_WakeUp(void);


void SleepInit(void)
{
	OSTaskCreateExt(UWB_Sleep,
									(void *)0,
									(OS_STK *)&UWBSleepStk[OS_TASK_STAT_STK_SIZE - 1],
									UWB_SLEEP_PRIO,
									UWB_SLEEP_PRIO,
									(OS_STK *)&UWBSleepStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
									
	UWB_SleepSem = OSSemCreate(0);
}



void UWB_Sleep(void *pdata)
{
	INT8U err;
	uint32_t STM_SleepTimer;
	uint32_t SysTimeNow;
	
	for(;;)
	{
		OSSemPend(UWB_SleepSem,0,&err);
		
		SysTimeNow = HAL_GetTick();
		STM_SleepTimer = (TagCommCycleTIM <= SysTimeNow ? 0 : TagCommCycleTIM - SysTimeNow);
		
		if(STM_SleepTimer != 0)
		{
			DWT_EnterSleep();									//DWM1000开始睡眠
			HAL_SPI_MspDeInit(&hspi1);				//关闭SPI时钟
			__HAL_RCC_RNG_CLK_DISABLE();			//关闭RNG
			
#ifdef PCB_V2
			HAL_ADC_MspDeInit(&hadc1);				//关闭ADC时钟
			LTC3401_BURST_MODE_ENABLE;				//电源模块进入省电模式
#endif
			if(ChargeSleep == 0)
			{
				HAL_RTCEx_SetWakeUpTimer_IT(&RTC_InitStruct, (STM_SleepTimer << 1) - 1, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0);		//填充RTC计数器，时能RTC唤醒
			}
			HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFI);
			
#ifdef PCB_V2
			LTC3401_BURST_MODE_DISABLE;		//电源模块进入正常模式
			HAL_ADC_MspInit(&hadc1);			//开启ADC时钟
#endif
			__HAL_RCC_RNG_CLK_ENABLE();		//开启RNG，46个RNG_CLK后产生第一个随机数
			HAL_SPI_MspInit(&hspi1);			//开启SPI时钟
			DWT_WakeUp();									//唤醒DWT
			uwTick = SysTimeNow + STM_SleepTimer;	//更新系统节拍器
		}
	}
}



