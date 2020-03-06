/*
*********************************************************************************************************
*	                                            UWB
*                           Commuication Controller For The Mining Industry
*
*                                  (c) Copyright 1994-2013  HNDZ 
*                       All rights reserved.  Protected by international copyright laws.
*
*
*    File    : can_app.c
*    Module  : can driver
*    Version : V1.0
*    History :
*   -----------------
*              can app 
*              Version  Date           By            Note
*              v1.0     2013-09-09     xxx
*
*********************************************************************************************************
*/

#include "can_app.h"



/********************************************************************************
*变量定义
*********************************************************************************/

OS_STK CanRx_stk[CANRX_TASK_SIZE];

OS_STK CanTx_stk[CANTX_TASK_SIZE];

uint32_t 				gUwbState = 0;

sCAN_BUFFER 		gUwbCanTxBuf;
sCAN_FRAME  		gCanRxcData;
#if CAN_TASK_MODE == CAN_MONITOR_MODE
sUWB_RUN_INFO 	gUwbRunPara= { 5,0,0,100, 0};
#else
sUWB_RUN_INFO 	gUwbRunPara;
#endif /* CAN_TASK_MODE == CAN_MONITOR_MODE*/

OS_EVENT			* gUwbStart;	

uint32_t      gACkframe;
uint8_t       gTtl;
static u8	s_u8CRC_TABLE[]= {0x00,0x31,0x62,0x53,0xC4,0xF5,0xA6,0x97,0xB9,0x88,0xDB,0xEA,0x7D,0x4C,0x1F,0x2E};	 //CRC

/*******************************************************************************************
**函数名称：void CanTx_Task(void *p_arg)
**输　入：None
** 输　出：None
** 功能描述：Can发送任务
*******************************************************************************************/
u8	CRC_8(u8 *PData, u8 Len)
{
	u8	CRC_Temp = 0;
	u8	Temp, i;
	u8	PData_H = 0;
	u8	PData_L = 0;

	for (i = 0; i < Len; i++)
	{
		PData_L = PData[i];
		if (i < (Len-1))
		{
			PData_H = PData[i+1];

			Temp = CRC_Temp>>4;
			CRC_Temp <<= 4;
			CRC_Temp ^= s_u8CRC_TABLE[Temp^(PData_H>>4)];
			Temp = CRC_Temp>>4;
			CRC_Temp <<= 4;
			CRC_Temp ^= s_u8CRC_TABLE[Temp^(PData_H&0x0F)];
			i ++;
		}

		Temp = CRC_Temp>>4;
		CRC_Temp <<= 4;
		CRC_Temp ^= s_u8CRC_TABLE[Temp^(PData_L>>4)];
		Temp = CRC_Temp>>4;
		CRC_Temp <<= 4;
		CRC_Temp ^= s_u8CRC_TABLE[Temp^(PData_L&0x0F)];
	}

	return (CRC_Temp);
}


sCAN_FRAME SendUwbData(uint32_t func, uint8_t dest, uint8_t * data, uint8_t size)
{
		CanHeadID  tmp;
		sCAN_FRAME ret;
		uint8_t    buf[12];
		memset(&tmp, 0x00, sizeof(tmp));
		memset(&ret, 0x00, sizeof(ret));
		switch (func)
		{
			case UWB_CONFIG_REQ:
			case UWB_HEART:
				tmp.ID.TID = UWB_ID;
				memcpy(ret.Data, "ALM", 3);
				ret.Data[3] = UWB_MODEL;          //指定为人员定位模块
				ret.Data[4] = (func == UWB_CONFIG_REQ) ? 1 : 0;
				ret.Data[5] = 1;   //版本号
				ret.DLC = 8;
				ret.Stdid = tmp.u32Id;
				break;
			case UWB_INFO_REPORT:
				if(gTtl >= 0x10)
					gTtl = 0;
				tmp.ID.TID = UWB_ID;
				tmp.ID.FT = 1;
				tmp.ID.SN = gTtl;
				gTtl++;
				memcpy(ret.Data, data, size);
//				ret.Data[7] = CRC_8(ret.Data,  7);
				ret.DLC = 8;
			  ret.Stdid = tmp.u32Id;
			
				memcpy(buf, &ret.Stdid, 4);
				buf[4] = 8;
				memcpy(buf+5, ret.Data, 7);
				ret.Data[7] = CRC_8(buf,  12);
			
				break;
			default:
				break;
		}
		return ret;
		
}

int32_t  WriteReportInfo(uint16_t tagNum, uint16_t tagId, uint16_t dist, uint16_t state)
{
	sTag_Info 	tmp;
	sCAN_FRAME  cantmp;
	
	memset(&tmp, 0x00, sizeof(tmp));
	tmp.peoplenum = tagNum;
	tmp.tagInfo.sTag.tagState = state;
	tmp.tagId = tagId;
	tmp.tagDist = dist;
	
	cantmp = SendUwbData(UWB_INFO_REPORT, 0, (uint8_t *)&tmp, 6);
	return CanBufferWrite(&gUwbCanTxBuf, cantmp);
	
}

/*******************************************************************************************
**函数名称：void CanTx_Task(void *p_arg)
**输　入：None
** 输　出：None
** 功能描述：Can发送任务
*******************************************************************************************/
#if CAN_TASK_MODE == CAN_POSITION_MODE
void CanTx_Task(void *p_arg)
{
	INT8U  	        	err;

	sCAN_FRAME        sendData;
//  OS_CPU_SR       cpu_sr = 0;
	
	CanBufferDataInit(&gUwbCanTxBuf);

	while(1)
	{  

		if(gUwbState == UWB_INIT)
		{
			STM32_CAN_Write(0, SendUwbData(UWB_CONFIG_REQ, 0, NULL,0), CAN_ID_EXT);
			OSTimeDly(REQ_CONFIG_DELAY);
		}
		else
		{
			OSSemPend(gUwbCanTxBuf.dataSem, UWB_HEART_DELAY, &err);
			if( err == OS_ERR_NONE)
			{
				if(CanBufferRead(&gUwbCanTxBuf, &sendData) == 0)
				{
					STM32_CAN_Write(0, sendData, CAN_ID_EXT); // 写发送数据
				}
				else{//can发送buf满
					gUwbState= UWB_INIT;
					
					
				}

			}
			else if(err == OS_ERR_TIMEOUT)
			{
				STM32_CAN_Write(0, SendUwbData(UWB_HEART, 0, NULL,0), CAN_ID_EXT); // 写心跳帧
			}
			else
			{
				;//
			}
      	
 	
		}

	}
	  
} 
#else
void CanTx_Task(void *p_arg)
{
	INT8U  	        	err;

	sCAN_FRAME        sendData;
//  OS_CPU_SR       cpu_sr = 0;
	
	CanBufferDataInit(&gUwbCanTxBuf);

	while(1)
	{  

			OSSemPend(gUwbCanTxBuf.dataSem, UWB_HEART_DELAY, &err);
			if( err == OS_ERR_NONE)
			{
				if(CanBufferRead(&gUwbCanTxBuf, &sendData) == 0)
				{
					while(STM32_CAN_Write(0, sendData, CAN_ID_EXT)) // 写发送数据
					{
						OSTimeDly(1);
					}
				}
				else{//can发送buf满
					gUwbState= UWB_INIT;
					
					
				}

			}
		}

}
	   
#endif /*CAN_TASK_MODE == CAN_POSITION_MODE*/

void WaitUwbStartSem(void)
{
	INT8U  	        	err;
	OSSemPend(gUwbStart, 0, &err);
}


uint16_t GetUwbId(void)
{
	return gUwbRunPara.standNum;
}

__inline uint16_t GetUwbReportTime(void)
{
	uint16_t tmp;
	tmp = gUwbRunPara.interval *100;
	return tmp;
}
/*******************************************************************************************
**函数名称：int32_t CheckTagState(uint16_t dist)
**输　入：标签距离
** 输　出：1：离开检测范围 0：在检测范围内
** 功能描述：
*******************************************************************************************/
int32_t CheckTagState(uint16_t dist)
{
	return (dist > gUwbRunPara.extent) ? 1:0;
}
/*******************************************************************************************

*******************************************************************************************/
void InitCanRxProc(sCAN_FRAME CanRX_Proc)
{
//	INT16U Id;

//	OS_CPU_SR  cpu_sr = 0;
	CanHeadID tmp;
	tmp.u32Id = CanRX_Proc.Stdid;
	if(tmp.ID.FT == SET_UWB_PARA && tmp.ID.RID == UWB_ID)
	{
		memcpy(&gUwbRunPara, CanRX_Proc.Data, sizeof(gUwbRunPara));
		gUwbState = UWB_NORMAL;
		OSSemPost(gUwbStart);
	}
	else
	{
		return;
	}

}
/*******************************************************************************************

*******************************************************************************************/
void NormalCanRxProc(sCAN_FRAME CanRX_Proc)
{
			CanHeadID tmp;
			tmp.u32Id = CanRX_Proc.Stdid;
			switch (tmp.ID.FT)
			{
				case REQ_UWB_INFO:
					break;
				case SET_UWB_PARA:
					if(tmp.ID.RID == UWB_ID)
					{
						memcpy(&gUwbRunPara, CanRX_Proc.Data, sizeof(gUwbRunPara));
					}
					break;
				default:
					break;
			}
}

/*******************************************************************************************
**函数名称：CanRxProc
**输　入：None
** 输　出：None
** 功能描述：Can发送正常时接收数据处理
*******************************************************************************************/
void CanRxProc(sCAN_FRAME CanRX_Proc)
{
	switch(gUwbState)
	{
		case UWB_INIT:
			InitCanRxProc(CanRX_Proc);
		break;
		case UWB_NORMAL:
			NormalCanRxProc(CanRX_Proc);
		break;
		
		default:
		break;											
  }

}
/*******************************************************************************************
**函数名称：void CanRx_Task(void *p_arg)
**输　入：None
** 输　出：None
** 功能描述：Can recevie task
*******************************************************************************************/

void CanRx_Task(void *p_arg)
{
	
	INT16S     err;

	gUwbStart = OSSemCreate(0);
	CAN_Config(200);
	
	while(1)
	{
		err = DevBusRead(0, (void *)&gCanRxcData, sizeof(sCAN_FRAME));
 		if(err == CANBUS_RX_TIMEOVER)
 		{
 			gUwbState = UWB_INIT;											//change uwb module of state 
 		}
 		else 
		{
		 	CanRxProc(gCanRxcData);
		}
		
	}

}
/******************************************************************************
** 函数名称: 
** 功能描述: 初始化设备所需的数据结构
** 参数描述：CAN APP 初始化
*******************************************************************************/
void CanAppInit   (void)
{

	INT8U err;

	/* init can bus */ 

	CanGpioInit();
	CanBusDataInit(CAN_DEV_ID);
#if CAN_TASK_MODE == CAN_MONITOR_MODE
	gUwbRunPara.extent = 100;
#endif /*CAN_TASK_MODE == CAN_MONITOR_MODE*/
	err = OSTaskCreateExt((void (*)(void *))CanRx_Task,
					(void          * )0,
					(OS_STK        * )&CanRx_stk[CANRX_TASK_SIZE - 1],
					(uint8_t         )CANRX_TASK_PRIO,
					(uint16_t        )CANRX_TASK_PRIO,
					(OS_STK        * )&CanRx_stk[0],
					(INT32U          )CANRX_TASK_SIZE,
					(void          * )0,
					(uint16_t        )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
	if (err == OS_ERR_NONE)
	{
		OSTaskNameSet(CANRX_TASK_PRIO, (uint8_t *)"Can Rx Task", &err);
	}
	
	err = OSTaskCreateExt((void (*)(void *))CanTx_Task,
			        (void          * )0,
			        (OS_STK        * )&CanTx_stk[CANTX_TASK_SIZE - 1],
			        (uint8_t         )CANTX_TASK_PRIO,
			        (uint16_t        )CANTX_TASK_PRIO,
			        (OS_STK        * )&CanTx_stk[0],
			        (INT32U          )CANTX_TASK_SIZE,
			        (void          * )0,
			        (uint16_t        )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
	if (err == OS_ERR_NONE)
	{
		OSTaskNameSet(CANTX_TASK_PRIO, (uint8_t *)"CAN TX TASK", &err);
	}


}

