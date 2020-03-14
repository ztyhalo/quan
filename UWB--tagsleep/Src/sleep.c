#include "uwbsleep.h"
#include "rtc.h"

OS_STK		UWBSleepStk[OS_TASK_STAT_STK_SIZE];

OS_EVENT  *UWB_SleepSem;		//ID�����ź���

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
			DWT_EnterSleep();									//DWM1000��ʼ˯��
			HAL_SPI_MspDeInit(&hspi1);				//�ر�SPIʱ��
			__HAL_RCC_RNG_CLK_DISABLE();			//�ر�RNG
			
#ifdef PCB_V2
			HAL_ADC_MspDeInit(&hadc1);				//�ر�ADCʱ��
			LTC3401_BURST_MODE_ENABLE;				//��Դģ�����ʡ��ģʽ
#endif
			if(ChargeSleep == 0)
			{
				HAL_RTCEx_SetWakeUpTimer_IT(&RTC_InitStruct, (STM_SleepTimer << 1) - 1, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0);		//���RTC��������ʱ��RTC����
			}
			HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFI);
			
#ifdef PCB_V2
			LTC3401_BURST_MODE_DISABLE;		//��Դģ���������ģʽ
			HAL_ADC_MspInit(&hadc1);			//����ADCʱ��
#endif
			__HAL_RCC_RNG_CLK_ENABLE();		//����RNG��46��RNG_CLK�������һ�������
			HAL_SPI_MspInit(&hspi1);			//����SPIʱ��
			DWT_WakeUp();									//����DWT
			uwTick = SysTimeNow + STM_SleepTimer;	//����ϵͳ������
		}
	}
}



