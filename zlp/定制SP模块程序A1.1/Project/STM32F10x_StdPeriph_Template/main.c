/*
*********************************************************************************************
*                                   ������������޹�˾
*                                        Ƕ��ʽ������
*
*                                 STM32F10XX��Ƭ��ģ���ļ�
*
*
* �ļ��� : main.c
* ��  �� : 
* ��  �� : V1.0
* ��  �� : 
*********************************************************************************************
*                                          �� ��
*
*   �������ӱ�׼ģ��,����ARM-CortexM3ϵ�е�Ƭ��.
*   ʹ��ʱ�����Լ��������ļ��ʹ��뼴��.
*
*   MCU  : STM32F10XX
*   IDE  : MDK-ARM    V4.14
*   CC   : Armcc.exe  V4.1.0.567
*
*********************************************************************************************
*/
/*
*********************************************************************************************
*           ����������
*           �������ƣ�����SPģ���·��
*           ���󣺣�1��������λ��ָ������������������SD����
*
* �ļ��� : 
* ��  �� : 
* ��  �� : V1.0
* ��  �� : 2016.6
*********************************************************************************************
*/

/* ----------------------------����ͷ�ļ�------------------------------------*/

//#include "ucos_ii.h"
//#include "stm32f10x.h"
//#include "Target.h"
//#include "usart1.h"
//#include "ff.h"
//#include "voice.h"
#include "include.h"
/*  -------------------------- �Զ���������� ---------------------------------*/
#define START_STK_SIZE              512
#define TASK_STK_SIZE_PlayLogic     512
#define TASK_STK_SIZE_PLAYSOUND     512
#define TASK_STK_SIZE_CANSENDMSG    512
 
#define START_TASK_Prio			       9
#define PRI_PlayLogic              10 
#define PRI_PLAYSOUND              11
#define PRI_CANSENDMSG              8 


static OS_STK	TaskPlayLogicStk[TASK_STK_SIZE_PlayLogic];			// ��������1��ջ
static OS_STK	TaskPLAYSOUNDStk[TASK_STK_SIZE_PLAYSOUND];			// ��������2��ջ
static OS_STK   TaskCanSendMsgStk[TASK_STK_SIZE_CANSENDMSG];  //��������3��ջ
static OS_STK	TASK_START_STK[START_STK_SIZE];			            // ��ʼ�����ջ



//u8 InputErr =0;         //��������־λ


/*  ---------------------�Զ��庯������--------------------------*/

void TaskStart(void *pdata);
void TaskPlayLogic(void *pdata);
void TaskPLAYSOUND(void *pdata);
void TaskCanSendMessage(void *pdata);
/********************************************************************************************
* �������ƣ�OSCreateEvent
* �������������������¼��������ź��������䡢��־��
* ��ڲ�������
* ���ڲ�������
* ʹ��˵���������ź������������񼶴����л��߶���������ǰ��
********************************************************************************************/
void OSCreateEvent(void)
{
    MsgQueue = OSQCreate(&MsgQueueTbl[0],MSG_QUEUE_SIZE); 
}

/********************************************************************************************
* �������ƣ�main ()
* ����������������
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
int main(void)
{
//	USART_INIT();
	OSInit();									             // ��ʼ�� uC/OS-II
	OSCreateEvent();							         // ���������ź����������ź������������񼶴����л��߶���������ǰ���
	TargetInit();
	Init_FreeList();              //��ʼ����������
	OSTaskCreate(TaskStart,	
	(void *)0,	
	(OS_STK *) & TASK_START_STK[START_STK_SIZE-1],	
	 START_TASK_Prio 
	);	
	OSStart();
    return 0;
}
/********************************************************************************************
* �������ƣ�TaskStart ()
* ������������������ʼ�����Լ�����
* ��ڲ�������
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TaskStart(void * pdata)
{
//	#if OS_CRITICAL_METHOD == 3                    
//    OS_CPU_SR  cpu_sr = 0;
//    #endif
	pdata = pdata;
	SoftTimer.OffLineTimer = OFFLINETICK;
	SysTickConfiguartion();					              	// ϵͳ��ʱ����ʼ��

//	OS_ENTER_CRITICAL();   


  OSTaskCreateExt(TaskPlayLogic,					             // ������������1,������
           (void *)0,
           &TaskPlayLogicStk[TASK_STK_SIZE_PlayLogic - 1],
           PRI_PlayLogic,
           2,
           &TaskPlayLogicStk[0],
           TASK_STK_SIZE_PlayLogic,
           (void *)0,
           OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);
				

	OSTaskCreateExt(TaskPLAYSOUND,					             // ������������2,������
					(void *)0,
					&TaskPLAYSOUNDStk[TASK_STK_SIZE_PLAYSOUND - 1],
					PRI_PLAYSOUND,
					4,
					&TaskPLAYSOUNDStk[0],
					TASK_STK_SIZE_PLAYSOUND,
					(void *)0,
					OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);
                    
	OSTaskCreateExt(TaskCanSendMessage,					      // ������������3,������
					(void *)0,
					&TaskCanSendMsgStk[TASK_STK_SIZE_CANSENDMSG - 1],
					PRI_CANSENDMSG,
					5,
					&TaskCanSendMsgStk[0],
					TASK_STK_SIZE_CANSENDMSG,
					(void *)0,
					OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);   
                    

// 	OSTaskSuspend(START_TASK_Prio);	                          //��������
    OSTaskDel(START_TASK_Prio);

//	OS_EXIT_CRITICAL();
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
