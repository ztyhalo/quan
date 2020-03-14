/*
*********************************************************************************************************
*
*                                      APPLICATION CONFIGURATION
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                           STM3240G-EVAL
*                                         Evaluation Board
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : FT
*                 DC
*********************************************************************************************************
*/

#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT

/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/
//#define  APP_CFG_SERIAL_EN                     DEF_ENABLED

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/
/*** 0��1��2��3Ԥ����uc/osϵͳ***************/
//#define	 CANRX_TASK_PRIO												 4u      /* can receive  task                        */
#define  UWBRX_TASK_PRIO                   			   5u	    /*uwb�����������ȼ����                        */
//#define  UWB_DEBUG_TASK_PRIO                   		 5u	    /*uwb�����������ȼ�                       */
#define  UWB_ANCHOR_TASK_PRIO                      6u
#define  UWB_TAG_TASK_PRIO                         7u
//#define  TAG_REPORT_TAST_PRIO                    8u
//#define	 UWBAPP_TASK_PRIO												 9u      /*  task                        */
//#define	 CANTX_TASK_PRIO												 10u      /* can transmit task                        */


#define  APPMNG_TASK_START_PRIO                  27u    /*ϵͳ��ʼ���� */

#define  OS_TASK_TMR_PRIO                        (OS_LOWEST_PRIO - 2u)
/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/
#define  UWBRX_TASK_STK_SIZE               256u
#define  UWB_ANCHOR_TASK_STK_SIZE          256u

#define  APPMNG_TASK_STK_SIZE              256u



#endif
