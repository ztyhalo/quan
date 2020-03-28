/*
*********************************************************************************************
*                                   天津华宁电子有限公司
*                                        嵌入式开发组
*
*                                 STM32F10XX单片机模板文件
*
*
* 文件名 : main.c
* 作  者 : 
* 版  本 : V1.0
* 日  期 : 
*********************************************************************************************
*                                          描 述
*
*   华宁电子标准模板,用于ARM-CortexM3系列单片机.
*   使用时加入自己的启动文件和代码即可.
*
*   MCU  : STM32F10XX
*   IDE  : MDK-ARM    V4.14
*   CC   : Armcc.exe  V4.1.0.567
*
*********************************************************************************************
*/
/*
*********************************************************************************************
*           工程描述：
*           单板名称：定制SP模块电路板
*           需求：（1）根据上位机指令播放语音，语音存放在SD卡中
*
* 文件名 : 
* 作  者 : 
* 版  本 : V1.0
* 日  期 : 2016.6
*********************************************************************************************
*/

/* ----------------------------包含头文件------------------------------------*/

//#include "ucos_ii.h"
//#include "stm32f10x.h"
//#include "Target.h"
//#include "usart1.h"
//#include "ff.h"
//#include "voice.h"
#include "include.h"
/*  -------------------------- 自定义变量声明 ---------------------------------*/
#define START_STK_SIZE              512
#define TASK_STK_SIZE_PlayLogic     512
#define TASK_STK_SIZE_PLAYSOUND     512
#define TASK_STK_SIZE_CANSENDMSG    512
 
#define START_TASK_Prio			       9
#define PRI_PlayLogic              10 
#define PRI_PLAYSOUND              11
#define PRI_CANSENDMSG              8 


static OS_STK	TaskPlayLogicStk[TASK_STK_SIZE_PlayLogic];			// 测试任务1堆栈
static OS_STK	TaskPLAYSOUNDStk[TASK_STK_SIZE_PLAYSOUND];			// 测试任务2堆栈
static OS_STK   TaskCanSendMsgStk[TASK_STK_SIZE_CANSENDMSG];  //测试任务3堆栈
static OS_STK	TASK_START_STK[START_STK_SIZE];			            // 起始任务堆栈



//u8 InputErr =0;         //输入错误标志位


/*  ---------------------自定义函数声明--------------------------*/

void TaskStart(void *pdata);
void TaskPlayLogic(void *pdata);
void TaskPLAYSOUND(void *pdata);
void TaskCanSendMessage(void *pdata);
/********************************************************************************************
* 函数名称：OSCreateEvent
* 功能描述：建立所有事件，包括信号量、邮箱、标志组
* 入口参数：无
* 出口参数：无
* 使用说明：建立信号量必须在任务级代码中或者多任务启动前完
********************************************************************************************/
void OSCreateEvent(void)
{
    MsgQueue = OSQCreate(&MsgQueueTbl[0],MSG_QUEUE_SIZE); 
}

/********************************************************************************************
* 函数名称：main ()
* 功能描述：主函数
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
int main(void)
{
//	USART_INIT();
	OSInit();									             // 初始化 uC/OS-II
	OSCreateEvent();							         // 建立所有信号量，建立信号量必须在任务级代码中或者多任务启动前完成
	TargetInit();
	Init_FreeList();              //初始化播放链表
	OSTaskCreate(TaskStart,	
	(void *)0,	
	(OS_STK *) & TASK_START_STK[START_STK_SIZE-1],	
	 START_TASK_Prio 
	);	
	OSStart();
    return 0;
}
/********************************************************************************************
* 函数名称：TaskStart ()
* 功能描述：测试任务开始任务，自己挂起
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void TaskStart(void * pdata)
{
//	#if OS_CRITICAL_METHOD == 3                    
//    OS_CPU_SR  cpu_sr = 0;
//    #endif
	pdata = pdata;
	SoftTimer.OffLineTimer = OFFLINETICK;
	SysTickConfiguartion();					              	// 系统定时器初始化

//	OS_ENTER_CRITICAL();   


  OSTaskCreateExt(TaskPlayLogic,					             // 创建例子任务1,测试用
           (void *)0,
           &TaskPlayLogicStk[TASK_STK_SIZE_PlayLogic - 1],
           PRI_PlayLogic,
           2,
           &TaskPlayLogicStk[0],
           TASK_STK_SIZE_PlayLogic,
           (void *)0,
           OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);
				

	OSTaskCreateExt(TaskPLAYSOUND,					             // 创建例子任务2,测试用
					(void *)0,
					&TaskPLAYSOUNDStk[TASK_STK_SIZE_PLAYSOUND - 1],
					PRI_PLAYSOUND,
					4,
					&TaskPLAYSOUNDStk[0],
					TASK_STK_SIZE_PLAYSOUND,
					(void *)0,
					OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);
                    
	OSTaskCreateExt(TaskCanSendMessage,					      // 创建例子任务3,测试用
					(void *)0,
					&TaskCanSendMsgStk[TASK_STK_SIZE_CANSENDMSG - 1],
					PRI_CANSENDMSG,
					5,
					&TaskCanSendMsgStk[0],
					TASK_STK_SIZE_CANSENDMSG,
					(void *)0,
					OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);   
                    

// 	OSTaskSuspend(START_TASK_Prio);	                          //挂起任务
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
