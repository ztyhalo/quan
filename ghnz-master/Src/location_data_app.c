/*
*********************************************************************************************************
*	                                            UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : location_data_app.c
*    Module  : uwb
*    Version : V1.0
*    History :
*   -----------------
*              uwb app 
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/

#include "location_data_app.h"
#include "can_app.h"
#include "main.h"
#include "uwb_anchor.h"


/********************************************************************************
*��������
*********************************************************************************/

OS_STK UWB_APPstk[UWBAPP_TASK_SIZE];
OS_STK TAG_Reportstk[TAG_REPORT_TASK_SIZE];

OS_EVENT		* gTagDataSem;	
sOList 				gUwbListData;

sUwbContInfo  gUwbConInfo;

sList_N * gTimeReport = NULL;

/*******************************************************************************************
**�������ƣ�
**�䡡�룺None
** �䡡����None
** ������������ⷶΧ�ڱ�ǩ������ز���
*******************************************************************************************/

__inline  void EnterPeopleAdd(void)
{
	gUwbConInfo.tagNum++;    
}

 __inline void ExitPeoplePlus(void)
{
	gUwbConInfo.tagNum > 0 ? gUwbConInfo.tagNum-- :0;
}

__inline uint8_t GetPeopleNum(void)
{
	return gUwbConInfo.tagNum;
}

/*******************************************************************************************
**�������ƣ�int32_t CheckIsReport(sTagData * data)
**�䡡�룺None
** �䡡����0:����Ҫ�ϱ� 1���ϱ�
** ��������������Ƿ��ϱ�����
*******************************************************************************************/
int32_t CheckIsReport(sTagData * data)
{
	uint16_t dvalue = (data->val > data->oldVal) ? 
							(data->val - data->oldVal) : (data->oldVal - data->val);
	if(dvalue > REPORT_D_DISTANCE)
	{
		data->oldVal = data->val;
		return 1;
	}
	return 0;
}
/*******************************************************************************************
**�������ƣ�void NewTagData(uint16_t tagId, uint16_t dist, sTagData * data)
**�䡡�룺None
** �䡡����None
** ����������Can��������
*******************************************************************************************/
void NewTagData(uint16_t tagId, uint16_t dist, sTagData * data)
{
	memset(data, 0x00, sizeof(sTagData));
	data->tagId = tagId;
	data->time = TAG_OFF_TIME;
	data->wr = 1;
	data->size = 1;
  data->dist[0] = dist;
	data->val = dist;
	data->oldVal = dist;
	data->state = TAG_ENTER;
}

/*******************************************************************************************
**�������ƣ�void TagDataProcess(sTagData * data, uint16_t dist)
**�䡡�룺None
** �䡡����None
** ������������ǩ���ݴ������
*******************************************************************************************/
void TagDataProcess(sTagData * data, uint16_t dist)
{
	uint16_t num = 0;
	uint16_t i;
	uint32_t  tmp = 0;
	
	if(data->size < UWB_APP_BUFF)
	{
		data->dist[data->size++] = dist;
		data->wr++;
		data->wr %= UWB_APP_BUFF;
		num = data->size;
	}
	else
	{
		data->dist[data->wr++] = dist;
		data->wr %= UWB_APP_BUFF;
		num = UWB_APP_BUFF;
	}
	for(i = 0; i < num; i++)
	{
		tmp += data->dist[i];
	}
	data->time = TAG_OFF_TIME;
	data->val = tmp/num;
}

/*******************************************************************************************
**�������ƣ�void FirstAddProcess(uint16_t tagId, uint16_t dist)
**�䡡�룺None
** �䡡����None
** ������������һ����ӱ�ǩ������
*******************************************************************************************/
int FirstAddProcess(uint16_t tagId, uint16_t dist)
{
	sTagData tmp;
	if(dist > gUwbRunPara.extent)
	{
		return FIRST_OFF_ERR;//��ǩ��һ�γ��ּ��뿪����������
	}
	else
	{
		EnterPeopleAdd();                    
		NewTagData(tagId, dist, &tmp);
		oList_Write_data(&gUwbListData,  tmp);
		WriteReportInfo(GetPeopleNum(), tagId, dist, TAG_ENTER);  //д���͵�can����
		OSSemPost(gTagDataSem);
		return  TAG_DEV_OK;
	}
}
/*******************************************************************************************
**�������ƣ�void TagExitProcess(uint16_t tagId, uint16_t dist)
**�䡡�룺None
** �䡡����None
** ������������ǩ�뿪��ⷶΧ������
*******************************************************************************************/
void TagExitProcess(uint16_t tagId, uint16_t dist)
{
	ExitPeoplePlus();
	WriteReportInfo(GetPeopleNum(), tagId, dist, TAG_OFF);       //д���͵�can����
	OSSemPost(gTagDataSem);
}

/*******************************************************************************************
**�������ƣ�void TagOffLineProcess(uint16_t tagId, uint16_t dist)
**�䡡�룺None
** �䡡����None
** ������������ǩ���ߴ�����
*******************************************************************************************/
void TagOffLineProcess(uint16_t tagId, uint16_t dist)
{
	ExitPeoplePlus();
	WriteReportInfo(GetPeopleNum(), tagId, dist, TAG_ERROR);       //д���͵�can����
	OSSemPost(gTagDataSem);
}
	

/*******************************************************************************************
**�������ƣ�void TagOffLineProcess(uint16_t tagId, uint16_t dist)
**�䡡�룺None
** �䡡����None
** ������������ǩ���ߴ�����
*******************************************************************************************/
//int GetTimeReportData(void)
//{
//		if(gTimeReport != NULL)   //�б�ǩ�ܹ�����
//		{
//			WriteReportInfo(GetPeopleNum(), gTimeReport->p->tagId, gTimeReport->p->val, TAG_ENTER);// д��������
//			gTimeReport = gTimeReport->next;
//			
//		}
//	
//}
/*******************************************************************************************
**�������ƣ�int WriteTagData(uint16_t tagId, uint16_t dist)
**�䡡�룺None
** �䡡����None
** ���������������ݷ������������
*******************************************************************************************/

void ReInitDataBuf(sDistNote * tag_p)
{
	memset(tag_p, 0x00, sizeof(sDistNote));
}

int CheckTagOffLine(sDistNote * tag_p)
{
	if(tag_p->oldHeartCout == tag_p->heartCout)
	{
		if(tag_p->errCount >= TAG_OFF_COUNT)
			return TAG_DEV_OFF_LINE;
		else
			tag_p->errCount++;
	}
	else
	{
		tag_p->oldHeartCout = tag_p->heartCout;
		tag_p->errCount = 0;
		
	}
	return TAG_DEV_OK;
}


int WriteTagData(sDistNote * tag_p)
{
	uint16_t tagId;
	uint16_t dist;

//#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
//    	OS_CPU_SR  cpu_sr = 0;
//#endif
//	sTagData tmp;
//	uint8_t err = 0;
	int err = 0;
	sList_N * tempoint = NULL;
	sList_N * retpoint = NULL;
	sList_N * midpoint = gUwbListData.rw_p;
	tagId = tag_p->tagId;
	dist = tag_p->realDist;
	
//	OS_ENTER_CRITICAL();
	if(midpoint == NULL)   //��buf
	{
		err = FirstAddProcess(tagId, dist);
		goto WEnd;
	}
	
	while(midpoint != NULL) //����
	{
		if(midpoint->p->tagId == tagId)  //����tagId
		{
			tempoint = midpoint;
			break;
		}
		retpoint = midpoint;
		midpoint = midpoint->next;
	}
	
	if(tempoint != NULL)  //����tagId
	{
		err = CheckTagOffLine(tag_p);    //�豸���ߴ����ж�
		if(err == TAG_DEV_OFF_LINE)
		{
			TagOffLineProcess(tagId, dist);
		}
		else
		{
			TagDataProcess(tempoint->p, dist);  //�����ݴ���
			err = CheckTagState(tempoint->p->val);
			if(err == 1)  //�뿪��ⷶΧ
			{
				TagExitProcess(tagId, tempoint->p->val);
			}
		}
		if(err)  
		{
			
			/******ɾ���������**************/
			if(retpoint == NULL)
			{
				gUwbListData.rw_p = tempoint->next;
				tempoint->next = gUwbListData.free_p;
				gUwbListData.free_p = tempoint;
			}
			else
			{
				retpoint->next = tempoint->next;
				tempoint->next = gUwbListData.free_p;
				gUwbListData.free_p = tempoint;
			}
		}
		else
		{
			//�ڼ�ⷶΧ�ڲ鿴�Ƿ�����仯���룬�������ϱ�
			if(CheckIsReport(tempoint->p))
			{
				WriteReportInfo(GetPeopleNum(), tagId, tempoint->p->val, TAG_ENTER);   //д���͵�can����
				OSSemPost(gTagDataSem);
			}				
		}
	}
	else 
	{
		err = FirstAddProcess(tagId,  dist);
	}
	
WEnd:	
//	OS_EXIT_CRITICAL();
	
//	if(err != TAG_DEV_OK)
//	{
//		ReInitDataBuf(tag_p);   //ɾ��һ�������е�����
//	}
	return err;
}


/*******************************************************************************************
**�������ƣ�void PollCheckTagOffLine(void)
**�䡡�룺None
** �䡡����None
** ����������ѭ������ǩ�Ƿ�����
*******************************************************************************************/
void PollCheckTagOffLine(void)
{
	sList_N * tempoint = NULL;
	sList_N * retpoint = NULL;
	sList_N * midpoint = gUwbListData.rw_p;
	
	if(midpoint == NULL)   //��buf
	{
		return;
	}
	
	while(midpoint != NULL) //����
	{
		if(midpoint->p->time != TAG_OFF_TIME)  //δ����
		{
			tempoint = midpoint->next;
//			ExitPeoplePlus();
			TagOffLineProcess(midpoint->p->tagId, midpoint->p->val);
			if(retpoint == NULL)
			{
				gUwbListData.rw_p = midpoint->next;
				midpoint->next = gUwbListData.free_p;
				gUwbListData.free_p = midpoint;
			}
			else
			{
				retpoint->next = midpoint->next;
				midpoint->next = gUwbListData.free_p;
				gUwbListData.free_p = midpoint;
			}
			midpoint = tempoint;
		}
		else
		{
			midpoint->p->time = 0;
			retpoint = midpoint;
			midpoint = midpoint->next;
		}
	}
}

/*******************************************************************************************
**�������ƣ�void UpdateDataProcess(void)
**�䡡�룺None
** �䡡����None
** ��������������һ��������buff����
*******************************************************************************************/
void UpdateDataProcess(void)
{
	uint8_t i;
#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
    	OS_CPU_SR  cpu_sr = 0;
#endif
	sDistList_N * midPoint;
	sDistList_N * beforeP;
	sDistList_N * deletP;
	OS_ENTER_CRITICAL();
	
	for(midPoint = GetDistBufPoint(),beforeP = NULL;midPoint != NULL; )
	                               //   beforeP = midPoint, midPoint = midPoint->next)
	{
		DistBufDataCount(midPoint->p);
		if(WriteTagData(midPoint->p) != TAG_DEV_OK) 
		{
			deletP = midPoint;
			midPoint = midPoint->next;
			DelDistBufData(beforeP, deletP);
		}
		else
		{
			 beforeP = midPoint;
			 midPoint = midPoint->next;
		}
		
	}
//	for(i = 0; i < ANC_COMM_NUM_MAX_WITH_TAG; i++)
//	{
//		if(TagDistBuff[i].TagID != 0 && TagDistBuff[i].mark == 1)    //�������ݽ��д���
//		{
//			WriteTagData(TagDistBuff+i);
//		}
//	}
	OS_EXIT_CRITICAL();
}


/*******************************************************************************************
**�������ƣ�void CanRx_Task(void *p_arg)
**�䡡�룺None
** �䡡����None
** ����������Can recevie task
*******************************************************************************************/

void UwbAppTask(void *p_arg)
{
	
//	INT16S     err;


	
	while(1)
	{
		UpdateDataProcess();
//		PollCheckTagOffLine();
		OSTimeDly(250);
	}

}


/*******************************************************************************************
**�������ƣ�void TagReportTask(void *p_arg)
**�䡡�룺None
** �䡡����None
** ����������
*******************************************************************************************/

void TagReportTask(void *p_arg)
{
	
	INT8U err;

	
	while(1)
	{
		gTimeReport = gUwbListData.rw_p;
		OSSemPend(gTagDataSem, GetUwbReportTime(), &err);
		if( err == OS_ERR_NONE)
		{
			//gTimeReport = gUwbListData.rw_p;
			;
		}
		else if(err == OS_ERR_TIMEOUT)
		{
			if(gTimeReport != NULL)   //�б�ǩ�ܹ�����
			{
				WriteReportInfo(GetPeopleNum(), gTimeReport->p->tagId, gTimeReport->p->val, TAG_ENTER);// д��������
				gTimeReport = gTimeReport->next;
				while(gTimeReport != NULL)
				{
				
					OSSemPend(gTagDataSem, REPORT_TIME_INTERVAL, &err);
					if(err == OS_ERR_TIMEOUT)
					{
						if(gTimeReport != NULL)   //�б�ǩ�ܹ�����
						{
							WriteReportInfo(GetPeopleNum(), gTimeReport->p->tagId, gTimeReport->p->val, TAG_ENTER);// д��������
							gTimeReport = gTimeReport->next;
							
						}
					}
					else
					{
						break;
					}
				}
			}

		}
		else
		{
			;//
		}
	}

}



/******************************************************************************
** ��������: void UwbAppInit   (void)
** ��������: uwb app init
** ����������
*******************************************************************************/
void LocationDataAppInit   (void)
{

	INT8U err;

	/* data init */ 
	gTagDataSem = OSSemCreate(0);
	oListDataInit(&gUwbListData);
	
	err = OSTaskCreateExt((void (*)(void *))TagReportTask,
					(void          * )0,
					(OS_STK        * )&TAG_Reportstk[TAG_REPORT_TASK_SIZE - 1],
					(uint8_t         )TAG_REPORT_TAST_PRIO,
					(uint16_t        )TAG_REPORT_TAST_PRIO,
					(OS_STK        * )&TAG_Reportstk[0],
					(INT32U          )TAG_REPORT_TASK_SIZE,
					(void          * )0,
					(uint16_t        )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
	if (err == OS_ERR_NONE)
	{
		OSTaskNameSet(UWBAPP_TASK_PRIO, (uint8_t *)"TAG REPORT Task", &err);
	}
	

	err = OSTaskCreateExt((void (*)(void *))UwbAppTask,
					(void          * )0,
					(OS_STK        * )&UWB_APPstk[UWBAPP_TASK_SIZE - 1],
					(uint8_t         )UWBAPP_TASK_PRIO,
					(uint16_t        )UWBAPP_TASK_PRIO,
					(OS_STK        * )&UWB_APPstk[0],
					(INT32U          )UWBAPP_TASK_SIZE,
					(void          * )0,
					(uint16_t        )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
	if (err == OS_ERR_NONE)
	{
		OSTaskNameSet(UWBAPP_TASK_PRIO, (uint8_t *)"UWB APP Task", &err);
	}

}

