#include "key.h"

#if defined(PCB_V2)

OS_EVENT  *KeyCheckSem;		//��������ź���
extern OS_EVENT  *ID_ConfigSem;

OS_STK			KeyCheckStk[OS_TASK_STAT_STK_SIZE];			//��ջ

/*******************************************************************************************
* �������ƣ�void KeyInit(void)
* ����������������ʼ������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void KeyInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	/* ���������� */
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
* �������ƣ�void SW_IRQHandler(void)
* ���������������жϷ�����
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
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
* �������ƣ�void KeyCheck(void *pdata)
* ��������������״̬�������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
uint8_t ChargeSleep;		//��س��ָʾ��0--δ��磬1--�����

uint8_t SW_PressCnt;					//SW��ť���´���������

uint8_t SystemResetTimBegin;
uint8_t ID_ConfigTimBegin;
uint32_t SystemResetTim;			//ϵͳǿ�Ƹ�λ�ж���ʱ��
uint32_t ID_ConfigTim;				//����ID����ģʽ�ж���ʱ��

void KeyCheck(void *pdata)
{
	INT8U err;
	for(;;)
	{
		OSSemPend(KeyCheckSem,0,&err);
		if(HAL_GPIO_ReadPin(SW_PIN_Port, SW_PIN) == GPIO_PIN_SET)						//SW����
		{
			SystemResetTimBegin = 1;	//����ϵͳǿ�Ƹ�λ�ж�
			SystemResetTim = HAL_GetTick() + SYSTEM_RESET_JUDGE_TIME_MS;
			
			ID_ConfigTimBegin = 1;		//����ID�����ж�
			ID_ConfigTim = HAL_GetTick() + ID_CONFIG_JUDGE_TIME_MS;
		}
		else																																//SW�ɿ�
		{
			SystemResetTimBegin = 0;	//�˳�ϵͳǿ�Ƹ�λ�ж�
			SW_PressCnt++;
			
			if((SW_PressCnt == 2) && (ID_ConfigTimBegin == 1))
			{
				SW_PressCnt = 0;
				ID_ConfigTimBegin = 0;
				OSSemPost(ID_ConfigSem);		//����ID����ģʽ
			}
		}
	}
}


#endif



