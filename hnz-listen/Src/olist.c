 /*
*********************************************************************************************************
*	                                           UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : olist.c
*    Module  : can driver
*    Version : V1.0
*    History :
*   -----------------
*             list data
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/
#include "olist.h"

#include <string.h>


//sOList gOListData;


/******************************************************************************
** ��������: void CanBusDataInit(uint16_t canid)
** ��������: ��ʼ���豸��������ݽṹ
** ����������can�ײ�������ʼ��
*******************************************************************************/
//void CanBufferDataInit(sCAN_BUFFER * databuf)
//{
//	memset(databuf, 0x00, sizeof(sCAN_BUFFER));	  //init
//#ifndef NO_SYS
//	databuf->dataSem = OSSemCreate(0);
//#endif  /*NO_SYS*/

//}


void oListDataInit(sOList * listdata)
{
	int32_t i; 
	memset(listdata->buf, 0x00, sizeof(listdata->buf));
	
	for(i = 0; i < LIST_SIZE -1; i++)
	{
		listdata->p[i].next = &(listdata->p[i + 1]);
		listdata->p[i].p = &(listdata->buf[i]);
	}
	listdata->p[LIST_SIZE -1].next = NULL;
	listdata->p[LIST_SIZE -1].p = &(listdata->buf[LIST_SIZE -1]);
	
	listdata->free_p = listdata->p;
	listdata->rw_p = NULL;
	
}


int32_t oList_Write_data(sOList * list, LIST_STRUCT data)
{
	int err = 0;
	sList_N *midpoint = NULL;
	
	if(list->free_p != NULL)
	{
		midpoint = list->free_p;
		list->free_p = list->free_p->next;
		*(midpoint->p) = data;
		midpoint->next = list->rw_p;
		list->rw_p = midpoint;
	}
	else
		err = -1;
	return err;
}

