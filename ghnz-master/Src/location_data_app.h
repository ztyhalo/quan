/*
*********************************************************************************************************
*	                                            UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : location_data_app.h
*    Module  : 
*    Version : V1.0
*    History :
*   -----------------
*             
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/

#ifndef __LOCATION_DATA_APP_H__
#define __LOCATION_DATA_APP_H__

#include "olist.h"



#define TAG_OFF_TIME   5000
#define TAG_OFF_COUNT   2

enum {
	TAG_ENTER = 1,
	TAG_OFF,
	TAG_ERROR
};


enum {
	TAG_DEV_OK = 0,
	FIRST_OFF_ERR = 1,
	TAG_DEV_OFF_LINE,
	
};


typedef struct
{
	uint8_t  tagNum;   //������Χ�ı�ǩ����
}sUwbContInfo;

#define REPORT_D_DISTANCE  30

#define REPORT_TIME_INTERVAL  40

/****************************************************************************************************
CAN rx and tx task  priority
 ***************************************************************************************************/
#define  TAG_REPORT_TAST_PRIO                    8u
#define	 UWBAPP_TASK_PRIO												 9u      /*  task                        */

/****************************************************************************************************
 CAN rx and tx task  stacks
 ***************************************************************************************************/
#define		UWBAPP_TASK_SIZE                        256u
#define		TAG_REPORT_TASK_SIZE                    256u

void LocationDataAppInit(void);


uint8_t GetPeopleNum(void);
void EnterPeopleAdd(void);
void ExitPeoplePlus(void);

#endif /*__LOCATION_DATA_APP_H__*/


